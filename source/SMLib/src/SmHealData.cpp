// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmHealData.cpp  
* PURPOSE: Source file for implementation of SmHealData methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmConfig.h>
#include <SmHealData.h>
#include <SmTopologyTraverser.h>
#include <SmGeomUtility.h>
// Remove Composites
//  #include <SmCFace.h>
#include <SmCrvOnSurf.h>
#include <SmHashTable.h>
#include <SmTrackTopologyChanges.h>

/*******************************************************************//**
PURPOSE: Debug convenience object_by_index tracking through the heal sequence
         and in any other methods with access to the SmFaceProps, SmEdgeProps, and SmVertexProps objs.

NOTES: To Debug One Target Face through the heal sequence: (same for Target Edge or Vertex)
         1. always Set lDebugFaceIndx = Indx of face of interest  (in file SmHealData.cpp)
         2. either Set bDebugAllFaceIndxCalls: TRUE = break for every SmDebugThisFaceIndx()  (in file SmHealData.cpp)
            or     Set bDebugThisIndxCall    : TRUE = break only SmDebutThisFaceIndx() calling DEBUG_CODE block with bDebugThisIndxCall == TRUE
         3. Set break point on line in function SmDebugThisFaceIndex
         4. Edit debug blocks of interest to start with,
                  if(bDebugMe || SmDebugThisFaceIndex(rFaceProps.m_lFaceIndx, bDebugThisIndxCall))
         5. When done remember to reset these values before releasing code:
                 lDebugFaceIndx         = SM_UNDEF_ULONG ;
                 bDebugAllFaceIndxCalls = FALSE ;
                 bDebugThisIndxCall     = FALSE ;
***********************************************************************/
#ifdef SM_DEBUG_CODE
  ULONG GWC_SET_NEXT_3_LINES_TO___SM_UNDEF_ULONG___BEFORE_RELEASE ;
  static constexpr ULONG lDebugFaceIndx   = 232 ;            // SM_UNDEF_ULONG = No FaceIndx     DebugBlock entries, else FaceIndx   = enter debugBlocks automatically for FaceIndx cases
  static constexpr ULONG lDebugEdgeIndx   = SM_UNDEF_ULONG ; // SM_UNDEF_ULONG = No EdgeIndx     DebugBlock entries, else EdgeIndx   = enter debugBlocks automatically for EdgeIndx cases
  static constexpr ULONG lDebugVertexIndx = SM_UNDEF_ULONG ; // SM_UNDEF_ULONG = No VertexIndx   DebugBlock entries, else VertexIndx = enter debugBlocks automatically for VertexIndx cases

 ULONG GWC_SET_NEXT_3_LINES_TO___FALSE___BEFORE_RELEASE ;
  static constexpr SmBoolean bDebugAllFaceIndxCalls   = FALSE ;
  static constexpr SmBoolean bDebugAllEdgeIndxCalls   = FALSE ;
  static constexpr SmBoolean bDebugAllVertexIndxCalls = FALSE ;
  
/*******************************************************************/
SmBoolean SmDebugThisFaceIndex(ULONG lFaceIndx, SmBoolean bDebugThisIndxCall)     { if(lFaceIndx   == lDebugFaceIndx   && (bDebugThisIndxCall || bDebugAllFaceIndxCalls))
    // ====> Place breakpoint here to watch lDebugFaceIndx
    { return TRUE ; } 
    // ====> Place breakpoint here to watch lDebugFaceIndx

  return FALSE ;
}
/*******************************************************************/             
SmBoolean SmDebugThisEdgeIndex(ULONG lEdgeIndx, SmBoolean bDebugThisIndxCall)     { if(lEdgeIndx   == lDebugEdgeIndx   && (bDebugThisIndxCall || bDebugAllEdgeIndxCalls))
    // ====> Place breakpoint here to watch lDebugEdgeIndx
    { return TRUE ; } 
    // ====> Place breakpoint here to watch lDebugEdgeIndx

  return FALSE ;
}
/*******************************************************************/             
SmBoolean SmDebugThisVertexIndex(ULONG lVertexIndx, SmBoolean bDebugThisIndxCall) { if(lVertexIndx == lDebugVertexIndx && (bDebugThisIndxCall || bDebugAllVertexIndxCalls))
    // ====> Place breakpoint here to watch lDebugVertexIndx
    { return TRUE ; } 
    // ====> Place breakpoint here to watch lDebugVertexIndx

  return FALSE ;
}
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Begin Heal Section for HealBrep method, support functions and
         and helper classes.

NOTES:
***********************************************************************/

/*******************************************************************//**
PURPOSE: Empty Constructor

NOTES:
***********************************************************************/
SmHealData::SmHealData()
 : m_pBrep(NULL), 
   m_pVerticesInVoxels(NULL) 
{              
  // allocate m_pVerticesInVoxels  - not a contained object because the template class uses pBrep->Method()
  //                                 and the include <SmHashTable.h> in SmHealData needs to be removed forcing
  //                                 m_pVerticesInVoxels to be a pointer

  // allocate m_pVerticesInVoxels object
  m_pVerticesInVoxels = new SmObjsInVoxels<SmVertexProps*>() ;

  // init all 
  ReSet(TRUE) ; // TRUE = Reset the m_sNewObj array lists, FALSE = don't

} // end SmHealData empty constructor

/*******************************************************************//**
PURPOSE: SmHealData Constructor

NOTES:
***********************************************************************/
SmHealData::SmHealData
 (SmBrep               * pBrep,       ///< [in] : target Brep                                                                                                  <br>
  SmTArray<SmVertex*>  & rVertices,   ///< [in] : TgtVertex list copied into SmHealData arrays, (any deleted vertices will be removed from the copied array)   <br>
  SmTArray<SmEdge*>    & rEdges,      ///< [in] : EdgeProps list copied into SmHealData arrays, (any deleted edges will be removed from the copied array)      <br>
  SmTArray<SmFace*>    & rFaces)      ///< [in] : FaceProps list copied into SmHealData arrays, (any deleted face will be removed from the copied array)       <br>
 : m_pBrep(NULL),
   m_pVerticesInVoxels(NULL)
{
  // allocate m_pVerticesInVoxels before ReSet() which dereferences it
  m_pVerticesInVoxels = new SmObjsInVoxels<SmVertexProps*>() ;

  // init all member values
  ReSet(TRUE) ; // TRUE = Reset the m_sNewObj array lists, FALSE = don't

  // Load tgt arrays
  LoadTargets( pBrep, rVertices, rEdges, rFaces) ;  // alloc Tgt ObjProps array objects 

} // end SmHealData constructor

/*******************************************************************//**
PURPOSE: SmHealData assignment operator

NOTES: SmHealData::operator= is deleted in the header (shallow copy of 
       owning *Props pointers would cause double-free). The   
       implementation body has been removed.
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmHealData Equality operator

NOTES:
***********************************************************************/
SmBoolean SmHealData::operator==( const SmHealData &crOther )
{
  // return value
  SmBoolean bRtn = TRUE ;

  // no work
  if(this == &crOther)
    { return TRUE ; }

  // TgtBrep
  bRtn &= m_pBrep == crOther.m_pBrep ;

  // m_pBrep: leave pointer alone

  // largest HealerOp value passed to HealBrep for this HealData obj 
  bRtn &= m_eStopHealOp == crOther.m_eStopHealOp ;
  bRtn &= m_eDoneHealOp == crOther.m_eDoneHealOp ; 
  bRtn &= m_lRecursionCount  == crOther.m_lRecursionCount ;

  // TgtObjects and TgtProps
  bRtn &= m_sTgtVertices == crOther.m_sTgtVertices ;
  bRtn &= m_sTgtEdges    == crOther.m_sTgtEdges    ; 
  bRtn &= m_sTgtFaces    == crOther.m_sTgtFaces    ; 
     
  bRtn &= m_sTgtVertexProps == crOther.m_sTgtVertexProps ;
  bRtn &= m_sTgtEdgeProps   == crOther.m_sTgtEdgeProps   ;
  bRtn &= m_sTgtFaceProps   == crOther.m_sTgtFaceProps   ;

  bRtn &= m_lOldVertexCnt ==  crOther.m_lOldVertexCnt ; // 0 = default for running healer, n=OldCnt for rerunning healer on newly created topology
  bRtn &= m_lOldEdgeCnt   ==  crOther.m_lOldEdgeCnt   ; // 0 = default for running healer, n=OldCnt for rerunning healer on newly created topology
  bRtn &= m_lOldFaceCnt   ==  crOther.m_lOldFaceCnt   ; // 0 = default for running healer, n=OldCnt for rerunning healer on newly created topology

  // All/Any Steps 
  bRtn &= m_sNotYetFaceProps == crOther. m_sNotYetFaceProps ;

  // Step: SmHealData::Cache_EdgeProps()
  bRtn &= m_sBadObjProps_BackPointers    == crOther.m_sBadObjProps_BackPointers    ;
  bRtn &= m_sBadEdgeProps_Uncontained    == crOther.m_sBadEdgeProps_Uncontained    ;
  bRtn &= m_sBadEdgeProps_SmallZoneTol3d == crOther.m_sBadEdgeProps_SmallZoneTol3d ;
  bRtn &= m_sBadEdgeProps_LargeZoneTol3d == crOther.m_sBadEdgeProps_LargeZoneTol3d ;
  bRtn &= m_sWarnEdgeProps_LargeGap3d    == crOther.m_sWarnEdgeProps_LargeGap3d    ;
  bRtn &= m_sBadEdgeProps_DegenEdges     == crOther.m_sBadEdgeProps_DegenEdges     ;  

  bRtn &= m_sLoopGap3d_Histogram     == crOther.m_sLoopGap3d_Histogram ;   
  bRtn &= m_sEdgeFaceGap3d_Histogram == crOther.m_sEdgeFaceGap3d_Histogram ;
  bRtn &= m_sEdgeLen3d_Histogram     == crOther.m_sEdgeLen3d_Histogram    ; 
  bRtn &= m_dMaxEdgeLength           == crOther.m_dMaxEdgeLength          ;
  bRtn &= m_dAvgEdgeLength           == crOther.m_dAvgEdgeLength          ;
  bRtn &= m_dNonDegen_MinEdgeLength  == crOther.m_dNonDegen_MinEdgeLength ;
  bRtn &= m_lNonDegenEdgeCnt         == crOther.m_lNonDegenEdgeCnt ;

  // Step: SmHealData::Cache_VertexProps()
  bRtn &= m_sBadVertexProps_SmallZoneTol3d == crOther.m_sBadVertexProps_SmallZoneTol3d ;
  bRtn &= m_sBadVertexProps_LargeZoneTol3d == crOther.m_sBadVertexProps_LargeZoneTol3d ;
  bRtn &= m_sWarnVertexProps_LargeGap3d    == crOther.m_sWarnVertexProps_LargeGap3d    ;
  bRtn &= m_sVertEdgeGap3d_Histogram       == crOther.m_sVertEdgeGap3d_Histogram       ;
  bRtn &= m_sVertFaceGap3d_Histogram       == crOther.m_sVertFaceGap3d_Histogram       ;

  // Step: SmHealData::Cache_FaceProps_Gaps()                                
  bRtn &= m_sBadFaceProps_SmallZoneTol3d == crOther.m_sBadFaceProps_SmallZoneTol3d ;           
  bRtn &= m_sBadFaceProps_LargeZoneTol3d == crOther.m_sBadFaceProps_LargeZoneTol3d ;

  // Step: SmHealData::Fix_TolSizes()

  // Step: SmHealData::Cache_CoinVertices()
  bRtn &=  m_lXCnt                        ==  crOther.m_lXCnt ;
  bRtn &=  m_lYCnt                        ==  crOther.m_lYCnt ;
  bRtn &=  m_lZCnt                        ==  crOther.m_lZCnt ;
  bRtn &=  m_sVertexBBox                  ==  crOther.m_sVertexBBox ;
  SM_ASSERT_BREAK_MSG(m_pVerticesInVoxels != NULL && crOther.m_pVerticesInVoxels != NULL, _T("SmHealData::operator==: m_pVerticesInVoxels ptr is NULL")) ;
  if(m_pVerticesInVoxels && crOther.m_pVerticesInVoxels) 
    { bRtn &= *m_pVerticesInVoxels        == *crOther.m_pVerticesInVoxels ; }
  bRtn &=  m_sBadVertexProps_CoinVertices ==  crOther.m_sBadVertexProps_CoinVertices ;
  bRtn &=  m_sVertVertGap3d_Histogram     ==  crOther.m_sVertVertGap3d_Histogram ;
  // Step  SmHealData::Fix_CoinVertices()

  // Step: SmHealData::Cache_DegenFaces() 
  bRtn &= m_sBadFaceProps_DegenFaces == crOther.m_sBadFaceProps_DegenFaces ;
  // Step: SmHealData::Fix_DegenFaces() 

  // Step: SmHealData::Fix_DegenEdges() 

  // Step: SmHealData::Cache_CoinEdges() ** TODO **
  bRtn &= m_sBadEdgeProps_CoinEdges == crOther.m_sBadEdgeProps_CoinEdges ;
  // Step  SmHealData::Fix_CoinEdges() ** TODO **

  // Step: SmHealData::Cache_MissedEdgeXSects() ** TODO **
  bRtn &= m_sBadEdgeProps_MissedEdgeXSects == crOther.m_sBadEdgeProps_MissedEdgeXSects ;
  // Step  SmHealData::Fix_MissedEdgeXSects() ** TODO **

  // Step: SmHealData::Fix_UncontainedEdges()                                                       
  // Step: SmHealData::Fix_BadGaps   ** TODO **

  // Step: SmHealData::Cache_FaceProps_Stage2()
  bRtn &= m_sFaceProps_Sheets          == crOther.m_sFaceProps_Sheets          ;
  bRtn &= m_sFaceProps_NoVertexPole    == crOther.m_sFaceProps_NoVertexPole    ;
  bRtn &= m_sBadFaceProps_FlatCorner   == crOther.m_sBadFaceProps_FlatCorner   ;
  bRtn &= m_sBadFaceProps_MissingSeam  == crOther.m_sBadFaceProps_MissingSeam  ;
  bRtn &= m_sBadFaceProps_CrossedSeam  == crOther.m_sBadFaceProps_CrossedSeam  ;
  bRtn &= m_sBadFaceProps_NearMissSeam == crOther.m_sBadFaceProps_NearMissSeam ;
                                                                                               
  // Step: SmHealData::Fix_MoveSeam()
  bRtn &= m_sRanFaceProps_MoveSeam == crOther.m_sRanFaceProps_MoveSeam ;

  // Step: SmHealData::Fix_SplitEdgesAtSeam()
  bRtn &= m_sRanFaceProps_SplitEdgesAtSeam == crOther.m_sRanFaceProps_SplitEdgesAtSeam ;

  // Step: SmHealData::Fix_SplitEdgesAtSeam()
  bRtn &= m_sRanFaceProps_SplitEdgesAtSeam == crOther.m_sRanFaceProps_SplitEdgesAtSeam ;

  // Step: SmHealData::Fix_BadSheets()                                                              
  bRtn &= m_lRanRegion_TestClosure_Cnt == crOther.m_lRanRegion_TestClosure_Cnt ;
  bRtn &= m_lFixRegion_SplitRegion_Cnt == crOther.m_lFixRegion_SplitRegion_Cnt ;
  
  // Step: SmHealData::Cache_FaceProps_Stage3()
  bRtn &= m_sBadFaceProps_LoopProblems       == crOther.m_sBadFaceProps_LoopProblems ;
  // Step: SmHealData::Fix_BadLoops()
                                                                                           
  bRtn &= m_sBadFaceProps_OuterLoopOrder     == crOther.m_sBadFaceProps_OuterLoopOrder ;
  bRtn &= m_sBadFaceProps_OrientLoop         == crOther.m_sBadFaceProps_OrientLoop ;
  bRtn &= m_sBadFaceProps_NoArea_Loop        == crOther.m_sBadFaceProps_NoArea_Loop ;
  bRtn &= m_sBadFaceProps_MultiOuterLoops    == crOther.m_sBadFaceProps_MultiOuterLoops ;
  bRtn &= m_sBadFaceProps_NestedOuterLoops   == crOther.m_sBadFaceProps_NestedOuterLoops ;
  bRtn &= m_sBadFaceProps_UnpairedNoAreaLoop == crOther.m_sBadFaceProps_UnpairedNoAreaLoop ;
  bRtn &= m_sBadFaceProps_Closed3dLoop       == crOther.m_sBadFaceProps_Closed3dLoop ;
  bRtn &= m_sBadFaceProps_ClosedPtrLoop      == crOther.m_sBadFaceProps_ClosedPtrLoop ;
  bRtn &= m_sBadFaceProps_MissingPole        == crOther.m_sBadFaceProps_MissingPole ;
                                                                                                  
  // Step: SmHealData::Fix_BadLoops()

  // Step: SmHealData::Fix_SplitFaceAtSeams()
  bRtn &= m_sRanFaceProps_SplitFaceAtSeam == crOther.m_sRanFaceProps_SplitFaceAtSeam ;

  // Step: SmHealData::Make_UVTrimCurves()
  // Step: SmHealData::Fix_InfiniteRegion()

  // New objects made by Fix functions
  bRtn &= m_sNewFaces_FromFixDegenFaces           == crOther.m_sNewFaces_FromFixDegenFaces ;
  bRtn &= m_sNewEdges_FromSplitEdgesAtSeams       == crOther.m_sNewEdges_FromSplitEdgesAtSeams ;
  bRtn &= m_sNewVertices_FromSplitEdgesAtSeams    == crOther.m_sNewVertices_FromSplitEdgesAtSeams ;
  bRtn &= m_sNewVerticesOnPoles_FromFixBadLoops   == crOther.m_sNewVerticesOnPoles_FromFixBadLoops ;
  bRtn &= m_sNewLoopsOnPoles_FromFixBadLoops      == crOther.m_sNewLoopsOnPoles_FromFixBadLoops ;
  bRtn &= m_sNewFaces_FromExtraLoops              == crOther.m_sNewFaces_FromExtraLoops ;
  bRtn &= m_sNewFaces_FromSplitFaceAtSeams        == crOther.m_sNewFaces_FromSplitFaceAtSeams ;
  bRtn &= m_sNewEdgesOnSeams_FromSplitFaceAtSeams == crOther.m_sNewEdgesOnSeams_FromSplitFaceAtSeams ;
  bRtn &= m_sNewVertices_FromSplitFaceAtSeams     == crOther.m_sNewVertices_FromSplitFaceAtSeams ;
  bRtn &= m_sNewRegions_FromClosedSheets          == crOther.m_sNewRegions_FromClosedSheets ;
  bRtn &= m_sNewShells_FromClosedSheets           == crOther.m_sNewShells_FromClosedSheets ;

  // all done
  return(bRtn) ;

} // end SmHealData equality operator

/*******************************************************************//**
PURPOSE: SmHealData destructor

NOTES:
***********************************************************************/
SmHealData::~SmHealData()
{
  // Free ObjProps memory
  FreeTargets() ;

#ifdef SM_DEBUG_CODE
  if(m_lPropsAllocCount != m_lPropsFreeCount)
    {
      TCHAR sBuff[SM_TBLOCK_SIZE] ;
      SM_SPRINTF(sBuff, _T("\nSmHealData LEAK: alloc=%lu free=%lu delta=%ld"),
          m_lPropsAllocCount, m_lPropsFreeCount,
          (long)m_lPropsAllocCount - (long)m_lPropsFreeCount) ;
      smos_WriteBuffer(sBuff) ;
      SM_ASSERT_MSG(FALSE, sBuff) ;
    }
#endif // SM_DEBUG_CODE

  // free allocated objects
  if(m_pVerticesInVoxels) { m_pVerticesInVoxels->SetSize(0,0,0,NULL,0) ;
                            delete m_pVerticesInVoxels ; 
                            m_pVerticesInVoxels = NULL ;
                          }

} // end SmHealData destructor

/*******************************************************************//**
PURPOSE: Gather Tgt Arrays - allocate ObjProps memory

NOTES: 1. Copies input Vertex, Edge, and Face lists into member m_sTgts array lists:
              m_sTgtVertices, m_sTgtEdges, m_sTgtFaces
       2. Sizes and inits with empty objects associated member m_sTgtObjProps arrays:
              m_sTgtVertexProps, m_sTgtEdgeProps, m_sTgtFaceProps
***********************************************************************/
void SmHealData::LoadTargets
 (SmBrep              * pBrep,      // in : TgtBrep contains all Tgt Vertices, Edges, Faces
  SmTArray<SmVertex*> & rVertices,  // in : TgtVertex list copied into SmHealData arrays
  SmTArray<SmEdge*>   & rEdges,     // in : EdgeProps list copied into SmHealData arrays
  SmTArray<SmFace*>   & rFaces)     // in : FaceProps list copied into SmHealData arrays
{
  // TgtBrep
  m_pBrep = pBrep ;

  // check state - NonNULL pBrep
  SM_ASSERT_MSG(pBrep != NULL, _T("SmHealData::LoadTargets: attempted to Load HealData with a NULL pBrep pointer")) ; 

  // locals
  ULONG ii ;
  const SmContext *cpContext =   m_pBrep != NULL         ? m_pBrep->GetContext()
                               : rVertices.GetSize() > 0 ? rVertices[0]->GetContext()
                               : rEdges.GetSize() > 0    ? rEdges[0]->GetContext()
                               : rFaces.GetSize() > 0    ? rFaces[0]->GetContext()
                               : NULL ;
#ifdef SM_DEBUG_CODE
  // check state - Tgt components are in pBrep
  ULONG di ;
  SmBoolean bOK = TRUE ;
  for(di=0;bOK && di<rVertices.GetSize();di++) { bOK &= (pBrep == rVertices[di]->GetBrep()) ; }
  for(di=0;bOK && di<rEdges.   GetSize();di++) { bOK &= (pBrep == rEdges[di]   ->GetBrep()) ; }
  for(di=0;bOK && di<rFaces.   GetSize();di++) { bOK &= (pBrep == rFaces[di]   ->GetBrep()) ; }
  SM_ASSERT_MSG(bOK == TRUE, _T("SmHealData::LoadTargets: attempted to Load HealData with a NULL pBrep pointer")) ; 
#endif // SM_DEBUG_CODE
  
  // TgtCnts
  ULONG lVtxCnt  = rVertices.GetSize() ;
  ULONG lEdgeCnt = rEdges.GetSize() ;
  ULONG lFaceCnt = rFaces.GetSize() ;

  // Copy target arrays
  m_sTgtVertices = rVertices;
  m_sTgtEdges    = rEdges;
  m_sTgtFaces    = rFaces;

  // Size PropsArrays  - allocate ObjProps array members
  m_sTgtVertexProps.SetSize(lVtxCnt) ; for(ii=0;ii<lVtxCnt;ii++)  { m_sTgtVertexProps[ii] = new SmVertexProps(cpContext) ; }
  m_sTgtEdgeProps.SetSize(lEdgeCnt) ;  for(ii=0;ii<lEdgeCnt;ii++) { m_sTgtEdgeProps[ii]   = new SmEdgeProps(cpContext) ;   }
  m_sTgtFaceProps.SetSize(lFaceCnt) ;  for(ii=0;ii<lFaceCnt;ii++) { m_sTgtFaceProps[ii]   = new SmFaceProps(cpContext) ;   }
#ifdef SM_DEBUG_CODE
  m_lPropsAllocCount += lVtxCnt + lEdgeCnt + lFaceCnt ;
#endif // SM_DEBUG_CODE

  // Init PropArray elems
  for(ii=0;ii<lVtxCnt;ii++)  { m_sTgtVertexProps[ii]->ReSet() ; } // Yuk! Calloc inits to Zero, Healer needs InitValues
  for(ii=0;ii<lEdgeCnt;ii++) { m_sTgtEdgeProps[ii]->ReSet() ; }   // Yuk! Calloc inits to Zero, Healer needs InitValues
  for(ii=0;ii<lFaceCnt;ii++) { m_sTgtFaceProps[ii]->ReSet() ; }   // Yuk! Calloc inits to Zero, Healer needs InitValues

  // nothing to do upon construction for m_sNotYetFaceProps 

} // end SmHealData::LoadTargets

/*******************************************************************//**
PURPOSE: Free ObjProps memory

NOTES: 1. frees the member m_sTgts arrays:
              m_sTgtVertices, m_sTgtEdges, m_sTgtFaces
       2. frees the m_sNotYetFaceProps array members
       3. empties the m_sTgtProp arrays: m_sTgtVertexProps, m_sTgtEdgeProps, 
                                         m_sTgtFaceProps, m_sNotYetFaceProps
***********************************************************************/
void SmHealData::FreeTargets()
{
 // locals
  ULONG ii ;
  ULONG lVtxCnt    = m_sTgtVertexProps.GetSize() ;
  ULONG lEdgeCnt   = m_sTgtEdgeProps.GetSize() ;
  ULONG lFaceCnt   = m_sTgtFaceProps.GetSize() ;
  ULONG lNotYetCnt = m_sNotYetFaceProps.GetSize() ;

  // Free ObjProps Array Members
  ULONG lActualFreeCount = 0 ;
  for(ii=0;ii<lVtxCnt;ii++)    { if(m_sTgtVertexProps[ii] ) { delete m_sTgtVertexProps[ii]  ; m_sTgtVertexProps[ii]  = NULL ; lActualFreeCount++ ; } }
  for(ii=0;ii<lEdgeCnt;ii++)   { if(m_sTgtEdgeProps[ii]   ) { delete m_sTgtEdgeProps[ii]    ; m_sTgtEdgeProps[ii]    = NULL ; lActualFreeCount++ ; } }
  for(ii=0;ii<lFaceCnt;ii++)   { if(m_sTgtFaceProps[ii]   ) { delete m_sTgtFaceProps[ii]    ; m_sTgtFaceProps[ii]    = NULL ; lActualFreeCount++ ; } }
                               
  // Free NotYetFaceProps Members
  // m_sNotYetFaceProps.m_sProbArray[ii] pts point to objs that are owned here - delete them
  for(ii=0;ii<lNotYetCnt;ii++) { if(m_sNotYetFaceProps[ii]) { delete m_sNotYetFaceProps[ii] ; m_sNotYetFaceProps[ii] = NULL ; lActualFreeCount++ ; } }

#ifdef SM_DEBUG_CODE
  m_lPropsFreeCount += lActualFreeCount ;
#endif // SM_DEBUG_CODE

  // Clear the array memory
  m_sTgtVertexProps .RemoveAll() ;
  m_sTgtEdgeProps   .RemoveAll() ;  
  m_sTgtFaceProps   .RemoveAll() ;  
  m_sNotYetFaceProps.RemoveAll() ;

} // end SmHealData::FreeTargets 

/*******************************************************************//**
PURPOSE: clear all SmHealData values - init the ProbArrays labels and links

NOTES: does not clear the New objects made by Fix functions arrays
***********************************************************************/
void SmHealData::ReSet
 (SmBoolean bReSetNewObjArrays) // in : TRUE = Reset the m_sNewObj array lists, FALSE = don't
                                //      default:[TRUE]
{
  // Free any existing *Props before clearing arrays to prevent leaks.
  // Safe on empty/uninitialized arrays (FreeTargets checks size before iterating).
  FreeTargets() ;

  // clear all SmProbArray<SmObjType> lists: refresh prob array links, labels, and sizes
  //    so those SmProbArray<SmObjType> members are not listed in this ReSet() method.
  SetupEmptyProbArrays() ;

  // m_pBrep: leave pointer alone

  // largest HealerOp value passed to HealBrep for this HealData obj 
  m_eStopHealOp = SM_HO_NONE ;
  m_eDoneHealOp = SM_HO_NONE ;
  m_lRecursionCount  = 0 ;

  // TgtObjects and TgtProps
  m_sTgtVertices.ReSet() ; 
  m_sTgtEdges.ReSet() ; 
  m_sTgtFaces.ReSet() ; 
     
  m_sTgtVertexProps.ReSet() ; 
  m_sTgtEdgeProps.ReSet() ; 
  m_sTgtFaceProps.ReSet() ;

  m_lOldVertexCnt = 0 ; // 0 = default for running healer, n=OldCnt for rerunning healer on newly created topology
  m_lOldEdgeCnt   = 0 ; // 0 = default for running healer, n=OldCnt for rerunning healer on newly created topology
  m_lOldFaceCnt   = 0 ; // 0 = default for running healer, n=OldCnt for rerunning healer on newly created topology

  // Step: SmHealData::Cache_EdgeProps() 
  m_sLoopGap3d_Histogram    .SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ; m_sLoopGap3d_Histogram    .SetAll(0) ;
  m_sEdgeFaceGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ; m_sEdgeFaceGap3d_Histogram.SetAll(0) ;
  m_sEdgeLen3d_Histogram    .SetSize(SM_HISTOGRAM_EDGELEN_BUCKETCNT) ; m_sEdgeLen3d_Histogram    .SetAll(0) ;
  m_dMaxEdgeLength = 0.0 ;
  m_dAvgEdgeLength = 0.0 ;
  m_dNonDegen_MinEdgeLength = SM_BIG_DOUBLE ;
  m_lNonDegenEdgeCnt = 0 ;

  // Step: SmHealData::Cache_VertexProps()          
  m_sVertEdgeGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ; m_sVertEdgeGap3d_Histogram.SetAll(0) ;
  m_sVertFaceGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ; m_sVertFaceGap3d_Histogram.SetAll(0) ;

  // Step: SmHealData::Cache_FaceProps_Gaps()

  // Step: SmHealData::Fix_TolSizes()

  // Step: Cache_CoinVertices()
  m_pVerticesInVoxels->SetSize(0,0,0,NULL,0) ;

  // Step: Fix_CoinVertices()
  
  // Step: SmHealData::Cache_DegenEdges()
  // Step: SmHealData::Fix_DegenEdges()

  // Step: SmHealData::Cache_CoinEdges() ** TODO **
  m_sVertVertGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ; m_sVertVertGap3d_Histogram.SetAll(0) ;
  // Step: SmHealData::Fix_CoinEdges() ** TODO **

  // Step: SmHealData::Fix_UncontainedEdges()
  // Step: SmHealData::FixBadGaps() ** TODO **

  // Step: SmHealData::Cache_FaceProps_Stage2()

  // Step: SmHealData::Fix_MoveSeam()

  // Step: SmHealData::Fix_SplitEdgesAtSeam()

  // Step: SmHealData::Fix_BadSheets()
  m_lRanRegion_TestClosure_Cnt = 0 ;
  m_lFixRegion_SplitRegion_Cnt = 0 ;
  
  // Step: SmHealData::Cache_FaceProps_Stage3()

  // Step: SmHealData::Fix_BadLoops()

  // Step: SmHealData::Fix_SplitFaceAtSeams()

  // Step: SmHealData::Make_UVTrimCurves()

  // Step: SmHealData::Fix_InfiniteRegion()

  // When asked - clear NewObjArray lists
  if(bReSetNewObjArrays)
    {
      m_sNewFaces_FromFixDegenFaces          .ReSet() ;
      m_sNewEdges_FromSplitEdgesAtSeams      .ReSet() ;
      m_sNewVertices_FromSplitEdgesAtSeams   .ReSet() ;
      m_sNewVerticesOnPoles_FromFixBadLoops  .ReSet() ;
      m_sNewLoopsOnPoles_FromFixBadLoops     .ReSet() ;
      m_sNewFaces_FromExtraLoops             .ReSet() ;
      m_sNewFaces_FromSplitFaceAtSeams       .ReSet() ;
      m_sNewEdgesOnSeams_FromSplitFaceAtSeams.ReSet() ;
      m_sNewVertices_FromSplitFaceAtSeams    .ReSet() ;
      m_sNewRegions_FromClosedSheets         .ReSet() ;
      m_sNewShells_FromClosedSheets          .ReSet() ;
    } // end need to ReSet NewObjArray lists

} // end SmHealData::ReSet

/*******************************************************************//**
PURPOSE: Setup and ReSet ProbArrays 

NOTES: 1.) Each array is set in 3 different sections.
             a. set link pointers        - constant per compile
             b. set PrettyPrint labels   - constant per compile
             c. ReSet() the contained m_sProbArray prob list 
       
       2.) When adding a new Array to SmHealData three lines of code need to be added. 
           One line in each of the 3 sections as:
             a. NewArrayName.Setup(SmHealerOpType, plObjProbFlag) ;
             b. NewArrayName.SetLabels(pOutputLabel, pDataLabel, pProbLabel)
             c. NewArrayName.ReSet() ;
***********************************************************************/
void SmHealData::SetupEmptyProbArrays()
{
  // Section 1 Scope: Set ProbArray Links
  {
    // All/Any step data - unsupported probs
    m_sNotYetFaceProps.                Setup(SM_HO_NONE, &SmFaceProps::m_bBadHasNotYetProb) ;
                                                                                         
    // Step: SmHealData::Cache_EdgeProps()   (now contains SmHealData::Cache_DegenEdges()
    m_sBadObjProps_BackPointers.       Setup(SM_HO_FIX_BACKPOINTERS,  &SmObjProps::m_bBadBackPointer) ;
    m_sBadEdgeProps_Uncontained.       Setup(SM_HO_CACHE_EDGEPROPS,   &SmEdgeProps::m_bBadUncontainedEdge) ;
    m_sBadEdgeProps_SmallZoneTol3d.    Setup(SM_HO_CACHE_EDGEPROPS,   &SmEdgeProps::m_bBadSmallZoneTol3d) ; 
    m_sBadEdgeProps_LargeZoneTol3d.    Setup(SM_HO_CACHE_EDGEPROPS,   &SmEdgeProps::m_bBadLargeZoneTol3d) ; 
    m_sWarnEdgeProps_LargeGap3d.       Setup(SM_HO_CACHE_EDGEPROPS,   &SmEdgeProps::m_bWarnLargeGap3d) ;  
    m_sBadEdgeProps_DegenEdges.        Setup(SM_HO_CACHE_EDGEPROPS,   &SmEdgeProps::m_bBadDegenEdge) ;
                                                                    
    // Step: SmHealData::Cache_VertexProps()          
    m_sBadVertexProps_SmallZoneTol3d.  Setup(SM_HO_CACHE_VERTEXPROPS, &SmVertexProps::m_bBadSmallZoneTol3d) ;
    m_sBadVertexProps_LargeZoneTol3d.  Setup(SM_HO_CACHE_VERTEXPROPS, &SmVertexProps::m_bBadLargeZoneTol3d) ;
    m_sWarnVertexProps_LargeGap3d.     Setup(SM_HO_CACHE_VERTEXPROPS, &SmVertexProps::m_bWarnLargeGap3d) ;  
                                                                       
    // Step: SmHealData::Cache_FaceProps_Gaps()
    m_sBadFaceProps_SmallZoneTol3d.    Setup(SM_HO_CACHE_FACEPROPS_GAPS, &SmFaceProps::m_bBadSmallZoneTol3d) ;
    m_sBadFaceProps_LargeZoneTol3d.    Setup(SM_HO_CACHE_FACEPROPS_GAPS, &SmFaceProps::m_bBadLargeZoneTol3d) ;
                                                                                    
    // Step: SmHealData::Fix_TolSizes()
  
    // Step: Cache_CoinVertices()
    m_sBadVertexProps_CoinVertices.    Setup(SM_HO_CACHE_COIN_VERTICES, &SmVertexProps::m_bBadCoincidentVertex) ; 
    // Step: Fix_CoinVertices()

    // Step: SmHealData::Cache_DegenFaces() 
    m_sBadFaceProps_DegenFaces.        Setup(SM_HO_CACHE_DEGEN_FACES, &SmFaceProps::m_bBadDegenFace) ;
    // Step: SmHealData::Fix_DegenFaces() 

    // Step SmHealData::Cache_CoinEdges() ** TODO **
    m_sBadEdgeProps_CoinEdges.         Setup(SM_HO_CACHE_COIN_EDGES,  &SmEdgeProps::m_bBadCoincidentEdge) ;
    // Step SmHealData::Fix_CoinEdges() ** TODO **
                                     
    // Step: SmHealData::Cache_MissedEdgeXSects() ** TODO **
    m_sBadEdgeProps_MissedEdgeXSects.  Setup(SM_HO_CACHE_MISSED_EDGEXSECTS, &SmEdgeProps::m_bBadMissedEdgeXSect) ;
    // Step  SmHealData::Fix_MissedEdgeXSects() ** TODO **

    // Step: SmHealData::Fix_UncontainedEdges()
    // Step SmHealData::Fix_BadGaps() ** TODO **
  
    // Step: SmHealData::Cache_FaceProps_Stage2()
    m_sFaceProps_Sheets.               Setup(SM_HO_CACHE_FACEPROPS_2, &SmFaceProps::m_eSheet) ; 
    m_sFaceProps_NoVertexPole.         Setup(SM_HO_CACHE_FACEPROPS_2, &SmFaceProps::m_ePoles) ; 
    m_sBadFaceProps_FlatCorner.        Setup(SM_HO_CACHE_FACEPROPS_2, &SmFaceProps::m_lBadFlatCorners) ; 
                                                                             
    m_sBadFaceProps_MissingSeam.       Setup(SM_HO_CACHE_FACEPROPS_2, &SmFaceProps::m_bBadMissingSeam) ;   
    m_sBadFaceProps_CrossedSeam.       Setup(SM_HO_CACHE_FACEPROPS_2, &SmFaceProps::m_bBadCrossedSeam) ;  
    m_sBadFaceProps_NearMissSeam.      Setup(SM_HO_CACHE_FACEPROPS_2, &SmFaceProps::m_bBadNearMissSeam) ; 

    // Step: SmHealData::Fix_BadSheets()

    // Step: SmHealData::Fix_MoveSeam()
    m_sRanFaceProps_MoveSeam.          Setup(SM_HO_FIX_MOVESEAM, &SmFaceProps::m_eBeenThroughMoveSeam) ;

    // Step: SmHealData::Fix_SplitEdgesAtSeam()
    m_sRanFaceProps_SplitEdgesAtSeam.  Setup(SM_HO_FIX_SPLITEDGE_ATSEAM, &SmFaceProps::m_eBeenThroughSplitEdgesAtSeam) ;

    // Step: SmHealData::Fix_BadSheets()

    // Step: SmHealData::Cache_FaceProps_Stage3()
    m_sBadFaceProps_LoopProblems.      Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_bBadLoopProblems) ;
    m_sBadFaceProps_OuterLoopOrder.    Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_bBadOuterLoopOrder) ;
    m_sBadFaceProps_OrientLoop.        Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lBadOrient_LoopCnt) ;
    m_sBadFaceProps_NoArea_Loop.       Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lBadNoArea_LoopCnt) ;

    m_sBadFaceProps_MultiOuterLoops.   Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lOuterLoopCnt) ;
    m_sBadFaceProps_NestedOuterLoops.  Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lBadNested_LoopCnt) ;
    m_sBadFaceProps_UnpairedNoAreaLoop.Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_bBadUnpairedNoArea_Loop) ;
    m_sBadFaceProps_Closed3dLoop.      Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lBadClosed3d_LoopCnt) ;  
    m_sBadFaceProps_ClosedPtrLoop.     Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lBadClosedPtr_LoopCnt) ; 
    m_sBadFaceProps_MissingPole.       Setup(SM_HO_CACHE_FACEPROPS_3, &SmFaceProps::m_lBadMissingPoles) ;       

    // Step: SmHealData::Fix_SplitFaceAtSeams()
    m_sRanFaceProps_SplitFaceAtSeam.   Setup(SM_HO_FIX_SPLITFACE_ATSEAMS, &SmFaceProps::m_eBeenThroughSplitFacesAtSeam) ;

    // Step: SmHealData::Fix_BadLoops()

    // Step: SmHealData::Make_UVTrimCurves()
    // Step: SmHealData::Fix_InfiniteRegion()

  } // end Section 1 Scope: Set ProbArray Links

  // begin Section 2 Scope: Set ProbArray Labels
  { // hint: keep all the constant string lengths the same for better looking pretty print reports
    

    // common end heal report labels for problem cases
    // summary string used when ProbArray contains any objects problems marked as still having the problem/property
    TCHAR sBuffBad[20];
    smos_snprintf(sBuffBad, 20, _T("%s"),_T(" - Bad "));

    TCHAR sBuffWarn[20];
    smos_snprintf(sBuffWarn, 20, _T("%s"), _T(" - Warn "));

    TCHAR sBuffOkay[20];
    smos_snprintf(sBuffOkay, 20, _T("%s"),_T(" - Can Be Okay "));

    m_sNotYetFaceProps.SetLabels(_T("Faces with NotYet Problems              "), _T("Unfixable Face Cnt "), _T("Warn - faces with probs not yet fixable"));

    // Step: SmHealData::Fix_BackPointers()
    m_sBadObjProps_BackPointers.SetLabels(_T("Obj BackPointer Not Set"), _T("BadBackPointer Cnt"), sBuffBad) ;

    // Step: SmHealData::Cache_EdgeProps()   (now contains SmHealData::Cache_DegenEdges()
    m_sBadEdgeProps_Uncontained.SetLabels(_T("Edge ParamLength exceeds Crv ParamDomain"), _T("Uncontained Cnt    "), sBuffBad) ;
    m_sBadEdgeProps_SmallZoneTol3d.SetLabels(_T("Edge ZoneTol3d too small                "), _T("SmallTol Edge Cnt  "), sBuffBad) ;

    m_sBadEdgeProps_LargeZoneTol3d.    SetLabels(_T("Edge ZoneTol3d too large                "), _T("LargeTol Edge Cnt  "), sBuffBad) ;
    m_sWarnEdgeProps_LargeGap3d.       SetLabels(_T("Edge Gaps larger than def XSectTol3d    "), _T("LargeGap Edge Cnt  "), sBuffWarn) ;
    m_sBadEdgeProps_DegenEdges.        SetLabels(_T("Edges too short                         "), _T("Degen Edge Cnt     "), sBuffBad) ;
                                                                                                                        
    // Step: SmHealData::Cache_VertexProps()                                                                            
    m_sBadVertexProps_SmallZoneTol3d.  SetLabels(_T("Vertex ZoneTol3d too small              "), _T("SmallTol Vert Cnt  "), sBuffBad) ;
    m_sBadVertexProps_LargeZoneTol3d.  SetLabels(_T("Vertex ZoneTol3d too large              "), _T("LargeTol Vert Cnt  "), sBuffBad) ;
    m_sWarnVertexProps_LargeGap3d.     SetLabels(_T("Vertex Gaps larger than def XSectTol3d  "), _T("LargeGap Vert Cnt  "), sBuffWarn) ;
                                                                                                                        
    // Step: SmHealData::Cache_FaceProps_Gaps()                                                                         
    m_sBadFaceProps_SmallZoneTol3d.    SetLabels(_T("Face ZoneTol3d too small                "), _T("SmallTol Face Cnt  "), sBuffBad) ;
    m_sBadFaceProps_LargeZoneTol3d.    SetLabels(_T("Face ZoneTol3d too large                "), _T("LargeTol Face Cnt  "), sBuffBad) ;
                                                                                                                        
    // Step: SmHealData::Fix_TolSizes()                                                                                 
                                                                                                                        
    // Step: Cache_CoinVertices()                                                                                       
    m_sBadVertexProps_CoinVertices.    SetLabels(_T("Vertices too close to one another       "), _T("Coincident PairCnt "), sBuffBad) ;
    // Step: Fix_CoinVertices()                                                                                         
                                                                                                                        
    // Step: SmHealData::Cache_DegenEdges()                                                                             
    m_sBadFaceProps_DegenFaces.        SetLabels(_T("Faces too small                         "), _T("Degen Face Cnt     "), sBuffBad) ;
    // Step: SmHealData::Fix_DegenEdges()                                                                               
                                                                                                                        
    // Step SmHealData::Cache_CoinEdges() ** TODO **                                                                    
    // Step SmHealData::Fix_CoinEdges() ** TODO **                                                                      
    m_sBadEdgeProps_CoinEdges.         SetLabels(_T("Edges too close to one another          "), _T("Coincident PairCnt "), sBuffBad) ;
                                                                                                                        
    // Step: SmHealData::Cache_MissedEdgeXSects() ** TODO **                                                            
    m_sBadEdgeProps_MissedEdgeXSects.  SetLabels(_T("Edge XSects missing Vertices            "), _T("Missed EdgeXSectCnt"), sBuffBad) ;
    // Step  SmHealData::Fix_MissedEdgeXSects() ** TODO **                                                              
                                                                                                                        
    // Step: SmHealData::Fix_UncontainedEdges()                                                                         
    // Step SmHealData::Fix_BadGaps() ** TODO **                                                                        
                                                                                                                        
    // Step: SmHealData::Cache_FaceProps_Stage2()                                                                       
    m_sFaceProps_Sheets.               SetLabels(_T("Sheet Face counts                       "), _T("                   "), _T("")) ;
    m_sFaceProps_NoVertexPole.         SetLabels(_T("Surface NoVertex Poles                  "), _T("                   "), _T("")) ;
    m_sBadFaceProps_FlatCorner.        SetLabels(_T("Surface Flat Corners (Su==Sv at corner) "), _T("Flat Corner Cnt    "), sBuffBad) ;
                                                                                                                        
    m_sBadFaceProps_MissingSeam.       SetLabels(_T("Face on closed srf missing a seam       "), _T("MissingSeam Cnt    "),  sBuffBad) ;
    m_sBadFaceProps_CrossedSeam.       SetLabels(_T("Face on closed srf crosses a seam       "), _T("CrossedSeam Cnt    "),  sBuffBad) ;
    m_sBadFaceProps_NearMissSeam.      SetLabels(_T("Face on closed srf comes close to a seam"), _T("NearMissSeam Cnt   "),  sBuffBad) ;
                                                                                                                        
    // Step: SmHealData::Fix_BadSheets()                                                                                
                                                                                                                        
    // Step: SmHealData::Fix_MoveSeam()                                                                                 
    m_sRanFaceProps_MoveSeam.          SetLabels(_T("Faces sent through Fix_MoveSeam         "), _T("                   "), _T("")) ;
                                                                                                                        
    // Step: SmHealData::Fix_SplitEdgesAtSeam()                                                                         
    m_sRanFaceProps_SplitEdgesAtSeam.  SetLabels(_T("Faces sent through Fix_SplitEdgesAtSeam "), _T("                   "), _T("")) ;
                                                                                                                        
    m_sBadFaceProps_LoopProblems.      SetLabels(_T("All Faces with any Loop Problems        "), _T("ProbLoop FaceCnt   "), sBuffBad) ;

    // Step: SmHealData::Fix_BadSheets()

    // Step: SmHealData::Cache_FaceProps_Stage3()
    m_sBadFaceProps_OuterLoopOrder.    SetLabels(_T("OuterLoop not 1st on Face LoopList      "), _T("Bad OuterLoop Face cnt           "), sBuffBad) ;
    m_sBadFaceProps_OrientLoop.        SetLabels(_T("LoopOrient-outer/inner Loop conflict    "), _T("Bad OrientedLoop Face cnt        "), sBuffBad) ;
    m_sBadFaceProps_NoArea_Loop.       SetLabels(_T("Loop fails to enclose an area in UVSpace"), _T("Bad NoAreaLoop Face cnt          "), sBuffBad) ;
    m_sBadFaceProps_MultiOuterLoops.   SetLabels(_T("Multiple Outer Loops on Face            "), _T("Bad multiple OuterLoop Face cnt  "), sBuffBad) ;
    m_sBadFaceProps_NestedOuterLoops.  SetLabels(_T("Nested OuterLoops on Face               "), _T("Bad nested OuterLoop Face cnt    "), sBuffBad) ;

    m_sBadFaceProps_UnpairedNoAreaLoop.SetLabels(_T("odd NoAreaLoopCnt on Face: no AddSeamFix"), _T("Bad unpaired NoAreaLoop Face Cnt "), sBuffBad) ;
    m_sBadFaceProps_Closed3dLoop.      SetLabels(_T("FaceLoop Edge EndPts not closed in 3d   "), _T("Bad 3d UnclosedLoop Face Cnt     "), sBuffBad) ;
    m_sBadFaceProps_ClosedPtrLoop.     SetLabels(_T("FaceLoop Edge EndPtrs not a closed loop "), _T("Bad ptr UnclosedLoop Face Cnt    "), sBuffBad) ;
    m_sBadFaceProps_MissingPole.       SetLabels(_T("Face contains an unmarked srf pole      "), _T("Bad Missing Vert on Pole Face Cnt"), sBuffBad) ;

    // Step: SmHealData::Fix_BadLoops()

    // Step: SmHealData::Fix_SplitFaceAtSeams()
    m_sRanFaceProps_SplitFaceAtSeam.   SetLabels(_T("Faces sent through Fix_SplitFaceAtSeam  "), _T("                   "), _T("")) ;

    // Step: SmHealData::Make_UVTrimCurves()
    // Step: SmHealData::Fix_InfiniteRegion()
  }  // end Section 2 Scope: Set ProbArray Labels

  // begin Section 3 Scope: Clear ProbArray ProbLists
  {
    m_sNotYetFaceProps.                ReSet() ;

    m_sBadObjProps_BackPointers.       ReSet() ;

    m_sBadEdgeProps_Uncontained.       ReSet() ;
    m_sBadEdgeProps_SmallZoneTol3d.    ReSet() ;
    m_sBadEdgeProps_LargeZoneTol3d.    ReSet() ;
    m_sWarnEdgeProps_LargeGap3d.       ReSet() ;
    m_sBadEdgeProps_DegenEdges.        ReSet() ;
                                           
    m_sBadVertexProps_SmallZoneTol3d.  ReSet() ;
    m_sBadVertexProps_LargeZoneTol3d.  ReSet() ;
    m_sWarnVertexProps_LargeGap3d.     ReSet() ;

    m_sBadFaceProps_SmallZoneTol3d.    ReSet() ;
    m_sBadFaceProps_LargeZoneTol3d.    ReSet() ;
                                           
    m_sBadVertexProps_CoinVertices.    ReSet() ;

    m_sBadFaceProps_DegenFaces.        ReSet() ;
  
    m_sBadEdgeProps_CoinEdges.         ReSet() ;
  
    m_sBadEdgeProps_MissedEdgeXSects.  ReSet() ;

    m_sFaceProps_Sheets.               ReSet() ;
    m_sFaceProps_NoVertexPole.         ReSet() ;
    m_sBadFaceProps_FlatCorner.        ReSet() ;
                                           
    m_sBadFaceProps_MissingSeam.       ReSet() ;
    m_sBadFaceProps_CrossedSeam.       ReSet() ;
    m_sBadFaceProps_NearMissSeam.      ReSet() ;
                                                   
    m_sRanFaceProps_MoveSeam.          ReSet() ;

    m_sRanFaceProps_SplitEdgesAtSeam.  ReSet() ;

    m_sBadFaceProps_LoopProblems.      ReSet() ;

    m_sBadFaceProps_OuterLoopOrder.    ReSet() ;
    m_sBadFaceProps_OrientLoop.        ReSet() ;
    m_sBadFaceProps_NoArea_Loop.       ReSet() ;

    m_sBadFaceProps_MultiOuterLoops.   ReSet() ;
    m_sBadFaceProps_NestedOuterLoops.  ReSet() ;

    m_sBadFaceProps_UnpairedNoAreaLoop.ReSet() ;
    m_sBadFaceProps_Closed3dLoop.      ReSet() ;
    m_sBadFaceProps_ClosedPtrLoop.     ReSet() ;
    m_sBadFaceProps_MissingPole.       ReSet() ;

    m_sRanFaceProps_SplitFaceAtSeam.   ReSet() ;

  } // end Section 3 Scope: Clear ProbArray ProbLists

} // end SmHealData::SetupEmptyProbArrays()

/*******************************************************************//**
PURPOSE: Get Array of FaceProps associated with SmFace objs in the 
         input Topology list.

NOTES: rtns number of FaceProps found
***********************************************************************/
ULONG SmHealData::GetFaceProps_ForTgts  
 (SmTArray<SmTopology *>  * pTgts,        // in : Array of topology objs that might be a current target
  SmTArray<SmFaceProps *> * pFaceProps,   // out: array of all FaceProps associated with tgt Faces in pTgts
  SmBoolean                 bStaleTgts)   // in : TRUE  = pTgts are stale and don't use pTgts->IsKindOf(), ...
                                          //      FALSE = pTgts are current and can use pTgts->IsKindof(), ...
                                          //      default:[TRUE]
{
  // init output
  pFaceProps->ReSet() ;

  // locals
  ULONG ii, lIndx ;

  // for every Tgt
  for(ii=0;ii<pTgts->GetSize();ii++)
    {
      SmTopology * pTgt = pTgts->GetAt(ii) ; 

      // when Tgt is a SmFace_TYPE
      if(bStaleTgts == TRUE || pTgt->IsKindOf(SmFace_TYPE))
        { 
          // find pTgt in m_sTgtFaces
          SmBoolean bIsIn = m_sTgtFaces.FindElement((SmFace *)pTgt, lIndx) ;

          // When Face is a Target
          if(bIsIn)
            {
              // load associated FaceProps into output
              pFaceProps->Add(m_sTgtFaceProps[lIndx]) ; 
            } // end found pTgt check
        } // end pTgt is a SmFace_TYPE check
    } // end iter all Tgts

  // all done
  return(pFaceProps->GetSize()) ;

} // end SmHealData::GetFaceProps_ForTgts

/*******************************************************************//**
PURPOSE: Get Array of EdgeProps associated with SmEdge objs in the 
         input Topology list.

NOTES: rtns number of EdgeProps found
***********************************************************************/
ULONG SmHealData::GetEdgeProps_ForTgts  
 (SmTArray<SmTopology *>  * pTgts,        // in : Array of topology objs that might be a current target
  SmTArray<SmEdgeProps *> * pEdgeProps,   // out: array of all EdgeProps associated with tgt Edges in pTgts
  SmBoolean                 bStaleTgts)   // in : TRUE  = pTgts are stale and don't use pTgts->IsKindOf(), ...
                                          //      FALSE = pTgts are current and can use pTgts->IsKindof(), ...
                                          //      default:[TRUE]
{
  // init output
  pEdgeProps->ReSet() ;

  // locals
  ULONG ii, lIndx ;

  // for every Tgt
  for(ii=0;ii<pTgts->GetSize();ii++)
    {
      SmTopology * pTgt = pTgts->GetAt(ii) ; 

      // when Tgt is a SmEdge_TYPE
      if(bStaleTgts == TRUE || pTgt->IsKindOf(SmEdge_TYPE))
        {       
          // find pTgt in m_sTgtEdges
          SmBoolean bIsIn = m_sTgtEdges.FindElement((SmEdge *)pTgt, lIndx) ;

          // When Edge is a Target
          if(bIsIn)
            {
              // load associated EdgeProps into output
              pEdgeProps->Add(m_sTgtEdgeProps[lIndx]) ; 
            } // end found pTgt check
        } // end pTgt is a SmEdge_TYPE check
    } // end iter all Tgts

  // all done
  return(pEdgeProps->GetSize()) ;

} // end SmHealData::GetEdgeProps_ForTgts

/*******************************************************************//**
PURPOSE: Get Array of VertexProps associated with SmVertex objs in the 
         input Topology list.

NOTES: rtns number of VertexProps found
***********************************************************************/
ULONG SmHealData::GetVertexProps_ForTgts  
 (SmTArray<SmTopology *>    * pTgts,        // in : Array of topology objs that might be a current target
  SmTArray<SmVertexProps *> * pVertexProps, // out: array of all VertexProps associated with tgt Vertexs in pTgts
  SmBoolean                   bStaleTgts)   // in : TRUE  = pTgts are stale and don't use pTgts->IsKindOf(), ...
                                            //      FALSE = pTgts are current and can use pTgts->IsKindof(), ...
                                            //      default:[TRUE]
{
  // init output
  pVertexProps->ReSet() ;

  // locals
  ULONG ii, lIndx ;

  // for every Tgt
  for(ii=0;ii<pTgts->GetSize();ii++)
    {
      SmTopology * pTgt = pTgts->GetAt(ii) ; 

      // when Tgt is a SmVertex_TYPE
      if(bStaleTgts == TRUE || pTgt->IsKindOf(SmVertex_TYPE))
        {       
          // find pTgt in m_sTgtVertexs
          SmBoolean bIsIn = m_sTgtVertices.FindElement((SmVertex *)pTgt, lIndx) ;

          // When Vertex is a Target
          if(bIsIn)
            {
              // load associated VertexProps into output
              pVertexProps->Add(m_sTgtVertexProps[lIndx]) ; 
            } // end found pTgt check
        } // end pTgt is a SmVertex_TYPE check
    } // end iter all Tgts

  // all done
  return(pVertexProps->GetSize()) ;

} // end SmHealData::GetVertexProps_ForTgts

// begin obsolete
// /*******************************************************************//**
// PURPOSE: Refreshes all FaceProp values thru given HealerOperation Step 
//          
// NOTES: 
//   1. Don't call this function with a eThruHealerOp value greater than the last
//      HealStep that has actually run on this part.
//      The HealSteps change the HealData state and that State is used to 
//      control the flow of execution in subsequent HealSteps.  
//      If this method is called with a too large eThruHealerOp
//      value the HealerData obj will think that it's been through Heal steps
//      that have not yet run. Any subsequent HealStep calls will have unpredictable
//      behavior (usually not fixing fixable heal problems, thinking there are
//      database problems that don't exist, and failures in the Heal fix functions).
//      Odds are making this mistake will corrupt the Brep obj being healed.
// 
//   2. Use this function to rebuild the values of a FaceProp after a heal step
//      edits the representation of the FaceProps->Face.  This has to be
//      done to capture the wins of a cascading fix and the losses of one
//      fix exposing a second hidden problem that only becomes apparent after
//      the first fix.
// 
//   3. This is a very expensive call to run - figure out how to write 
//      the heal steps to minimize the number of times this method gets
//      called.
// ***********************************************************************/
// void SmHealData::RefreshFaceProps
//  (SmFaceProps  * pTgtFaceProps,  // in : the Tgt FaceProps to clear and refresh 
//   ULONG          lFaceIndx,      // in : associated Indx in SmHealData::m_sTgtFaces list for pTgtFaceProps->m_pFace
//   SmHealerOpType eThruHealerOp,  // in : Refresh TgtFaceProps up thru this HealerStep operation limited to SmHealData::m_eDoneHealOp
//   ULONG          lOptLabel)      // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
// {
//   const SmFace * cpTgtFace = pTgtFaceProps->m_pFace ;
//   pTgtFaceProps->ReSet() ; // resets all SM_PROPSTAGE_GAPS, SM_PROPSTAGE_2, and SM_PROPSTAGE_3 values
// 
//   // limit the eThruHealerOp to the current SmHealData::m_eDoneHealOp value
//   if(eThruHealerOp > m_eDoneHealOp)
//     { eThruHealerOp = m_eDoneHealOp ; }
// 
//   // 1st Get Stage of FaceProps Values 
//   if(eThruHealerOp >= SM_HO_CACHE_FACEPROPS_GAPS)
//     { 
//       // sets: m_pFace              
//       //       m_sZoneTol3d          m_sOrigZoneTol3d          
//       //       m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d  
//       pTgtFaceProps->SetProps(SM_PROPSTAGE_GAPS, cpTgtFace, lFaceIndx, m_eDoneHealOp) ; 
//     }
// 
//   // 2nd Get Stage of FaceProps Values 
//   if(eThruHealerOp >= SM_HO_CACHE_FACEPROPS_2)
//     { 
//       // sets: m_cpSurface           m_sNaturalUVDomain
//       //       m_bSheetFace          
//       //       m_bClosedU            m_bClosedV          m_bClosedSurf
//       //       m_eClosedValU  m_eClosedValV
//       //       m_lPoles              m_lApproxPoles      m_lNoVertexPoles
//       //       m_sPolePoints         m_sPoleNormals      
//       //       m_lBadFlatCorners     m_sFlatCornerPoints 
//       //       m_pCrvClassU          m_pCrvClassV        
//       //       m_sRawTouchListU      m_sRawTouchListV
//       //       m_sDoneTouchListU     m_sDoneTouchListV
//       //       m_eBadMissingSeam     m_eBadCrossedSeam   m_eBadNearMissSeam
//       pTgtFaceProps->SetProps(SM_PROPSTAGE_2, cpTgtFace, lFaceIndx, m_eDoneHealOp) ; 
//     }
// 
//   // 3rd Get Stage of FaceProps Values 
//   if(eThruHealerOp >= SM_HO_CACHE_FACEPROPS_3)
//     { 
//       // sets: m_sLoopProps
//       //       m_lClosedLoopCnt       m_lOuterLoopCnt          
//       //       m_bBadOuterLoopOrder   m_lBadOrient_LoopCnt     
//       //       m_lBadNoArea_LoopCnt   m_lBadNested_LoopCnt     
//       //       m_lBadMissingPoles     m_bBadUnpairedNoArea_Loop
//       //       m_lBadClosed3d_LoopCnt m_lBadClosedPtr_LoopCnt
//       pTgtFaceProps->SetProps(SM_PROPSTAGE_3, cpTgtFace, lFaceIndx, m_eDoneHealOp) ; 
//     }
// 
// } // end SmHealData::RefreshFaceProps
// end obsolete

/*******************************************************************//**
PURPOSE: convenience call to SmHealData::UpdateTargetLists main method

NOTES: 
***********************************************************************/
SmStatus SmHealData::UpdateTargetLists
 (SmTrackTopologyChanges & rTopologyChanges, // in : container for RmList, RmTypes, and AddList
  SmHealerOpType           eCurrentHealOp)   // in : Tgt HealOp to execute upon new ObjProps associated with AddList items
{ 
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugLog = FALSE ;
  if(bDebugLog)
    {
      smos_WriteBuffer(_T("\nEntered UpdateTargetLists()")) ;
    }
  if(bDebugMe)
    {         
      this->Dump() ;
      rTopologyChanges.Dump() ;
    }
#endif // SM_DEBUG_CODE

  // check state - requires that the rTopologyChanges obj is not actively tracking topology changes
  SM_ASSERT_MSG(FALSE == rTopologyChanges.IsTracking(), 
                _T("SmHealData::UpdateTargetLists: Error - don't run UpdateTargetLists() on SmTrackTopologyChanges objs actively tracking topo changes. Call StopTracking()") ) ;

  // Quit when rTopologyChanges tracker is actively tracking topo changes
  //   gwc: could turn off the tracing here?
  if(rTopologyChanges.IsTracking())
    { return(SM_ERR) ; }

  // pass the call along
  SER(UpdateTargetLists(rTopologyChanges.GetRmList(), 
                        rTopologyChanges.GetAddList(), 
                        eCurrentHealOp)) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {         
      this->Dump() ;
      rTopologyChanges.Dump() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;
  
} // end SmHealData::UpdateTargetLists

/*******************************************************************//**
PURPOSE: Add and remove topology objects from the TargetObj and associated
  problem lists to support heal functions that delete or create new
  topology objects during the heal sequence.

NOTES: 1. Remove and delete stale topology objs and their ObjProps from all the HealData Arrays.
       2. Add new topology objs to the Target Obj lists 
       3. for every new Topology obj
           a. make new ObjProps obj and add that to ObjProps arrays
           b. run the heal sequence on the new topology up to the current heal op.
***********************************************************************/
SmStatus SmHealData::UpdateTargetLists
 (SmTArray<SmTopology *> * pRmList,     // in : List of Topo objs removed from m_pBrep TopoGraph to remove from HealData Arrays
  SmTArray<SmTopology *> * pAddList,    // in : List of Topo objs added to m_pBrep TopoGraph to add to HealData Arrays
  SmHealerOpType         eThruHealerOp) // in : The HealStep to which the new ObjProps are run to catch up with the existing TargetObjs limited to SmHealData::m_eDoneHealOp
{
  NER(pRmList) ;
  NER(pAddList) ;
  // locals
  ULONG ii ;
  SmTArray<SmFaceProps *>   sRemoveFaceProps,   sAddFaceProps ;
  SmTArray<SmEdgeProps *>   sRemoveEdgeProps,   sAddEdgeProps ;
  SmTArray<SmVertexProps *> sRemoveVertexProps, sAddVertexProps ;

#ifdef SM_DEBUG_CODE
TCHAR     sBuff[SM_TBLOCK_SIZE] ;
// ULONG GWC_SET_NEXT_LINE_FOR_DEBUGLOG_TO_FALSE_BEFORE_RELEASE_AND_TRUE_HEAL_DEBUG ;
SmBoolean bDebugLog = FALSE ;  // GWC::SET_LINE_TO_TRUE_FOR_HEAL_DEBUG
  if(bDebugLog)
    {
      smos_WriteBuffer(_T("\nEntered UpdateTargetLists()")) ;
    }
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {         
      this->Dump() ; 
    }
#endif // SM_DEBUG_CODE

  // When pRmList topology Tgts exist
  if(pRmList->GetSize() > 0)
    {
      // gather associated ObjProps for RmList members
      GetFaceProps_ForTgts  (pRmList, &sRemoveFaceProps,   TRUE) ; // TRUE  = pTgts are stale and don't use pTgts->IsKindOf(), ...
      GetEdgeProps_ForTgts  (pRmList, &sRemoveEdgeProps,   TRUE) ; // TRUE  = pTgts are stale and don't use pTgts->IsKindOf(), ...
      GetVertexProps_ForTgts(pRmList, &sRemoveVertexProps, TRUE) ; // TRUE  = pTgts are stale and don't use pTgts->IsKindOf(), ...

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {         
          SmTArray<SmTopology *>    sCommonTopologies ;  
          pRmList->FindCommonElements(*pAddList, sCommonTopologies) ; 

          if(sCommonTopologies.GetSize() > 0)
            {
              smos_WriteBuffer(_T("\n  Common Ptrs in the Remove And Add lists: ")) ; 
              smos_WriteBuffer(_T("\n    Common Add and Remove List Ptrs are Okay.  This happens when a deleted obj was replaced by a new one reusing the old ptr value. ")) ; 
              smos_WriteBuffer(_T("\n      (Example: Split - The parent ptr value is always reused by one of the two new children objects.)")) ; 
              sCommonTopologies.Dump() ;  
            }

          smos_WriteBuffer(_T("\n  RmList - before removing: ")) ;  pRmList->Dump() ;
          DumpArraysContaining(0, &sRemoveFaceProps,
                                  &sRemoveEdgeProps,
                                  &sRemoveVertexProps,
                                  TRUE) ;  // TRUE = skip 
          smos_WriteBuffer(_T("\n  AddList: ")) ;  pAddList->Dump() ;
        }
#endif // SM_DEBUG_CODE

      // remove the Tgt Objs from Tgt, TgtProps, and ProbArray lists, then free the ObjProps
      for(ii=0;ii<sRemoveFaceProps.GetSize();  ii++) { 
                                                       RemoveFaceProps_FromLists  (sRemoveFaceProps[ii]) ; 
#ifdef SM_DEBUG_CODE
                                                       if(bDebugMe)
                                                         { DumpFaceListsContaining(sRemoveFaceProps[ii]) ; }
#endif // SM_DEBUG_CODE
                                                       delete sRemoveFaceProps[ii] ; sRemoveFaceProps[ii] = NULL ;
                                                     }
      for(ii=0;ii<sRemoveEdgeProps.GetSize();  ii++) {  
                                                       RemoveEdgeProps_FromLists  (sRemoveEdgeProps[ii]) ; 
#ifdef SM_DEBUG_CODE
                                                       if(bDebugMe)
                                                         { DumpEdgeListsContaining(sRemoveEdgeProps[ii]) ; }
#endif // SM_DEBUG_CODE
                                                       delete sRemoveEdgeProps[ii] ; sRemoveEdgeProps[ii] = NULL ;
                                                     }
      for(ii=0;ii<sRemoveVertexProps.GetSize();ii++) {  
                                                       RemoveVertexProps_FromLists(sRemoveVertexProps[ii]) ; 
#ifdef SM_DEBUG_CODE
                                                       if(bDebugMe)
                                                         { DumpVertexListsContaining(sRemoveVertexProps[ii]) ; }
#endif // SM_DEBUG_CODE
                                                       delete sRemoveVertexProps[ii] ; sRemoveVertexProps[ii] = NULL ;
                                                     }
#ifdef SM_DEBUG_CODE
      m_lPropsFreeCount += sRemoveFaceProps.GetSize() + sRemoveEdgeProps.GetSize() + sRemoveVertexProps.GetSize() ;
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {         
          SmTArray<SmTopology *>    sCommonTopologies ;  
          pRmList->FindCommonElements(*pAddList, sCommonTopologies) ; 

          if(sCommonTopologies.GetSize() > 0)
              smos_WriteBuffer(_T("\n  Common Ptrs in the Remove And Add lists: ")) ; 
            { smos_WriteBuffer(_T("\n    Common Add and Remove List Ptrs are Okay.  This happens when a deleted obj was replaced by a new one reusing the old ptr value. ")) ; 
              smos_WriteBuffer(_T("\n      (Example: Split - The parent ptr value is always reused by one of the two new children objects.)")) ; 
              sCommonTopologies.Dump() ;  
            }

          smos_WriteBuffer(_T("\n  RmList - after removing: ")) ;  pRmList->Dump() ;
          DumpArraysContaining(1, &sRemoveFaceProps,
                                  &sRemoveEdgeProps,
                                  &sRemoveVertexProps,
                                  FALSE) ;  //      FALSE = Only PrettyPrint array reports for arrays containing one or more Tgts
          smos_WriteBuffer(_T("\n  AddList: ")) ;  pAddList->Dump() ;
        }
#endif // SM_DEBUG_CODE
    } // end pRmList topology Tgts exist check

  // when pAddList topology Tgts exist - run the heal sequence on the AddList topology objects up to the current heal step.
  if(pAddList->GetSize() > 0)
    {
      // locals
      SmTArray<SmFace *>   sNewFaces ;
      SmTArray<SmEdge *>   sNewEdges ;
      SmTArray<SmVertex *> sNewVertices ;
      const SmContext    * cpContext =   m_pBrep != NULL        ? m_pBrep->GetContext()
                                       : pAddList->GetSize() > 0 ? pAddList->GetAt(0)->GetContext()
                                       : NULL ;

      // sort and count the new tgt Faces, Edges, and Vertices
      for(ii=0;ii<pAddList->GetSize();ii++)
        {
          // defensive: never dereference a null tracked add entry
          SmTopology * pAddTgt = pAddList->GetAt(ii) ;
          if(pAddTgt == NULL) { continue ; }

          // defensive: skip stale/freed entries. A heal op can free topology that was recorded on the
          // tracker's add list (e.g. temporaries created and destroyed by CombineCoincidentVertices, or
          // objects freed after StopTracking), leaving a dangling pointer here. IsLiveTopologyMember only
          // compares pointer identity against the brep's live lists, so it is safe on a dangling pointer,
          // whereas the IsKindOf() calls below would dereference it and crash.
          if(m_pBrep != NULL && !m_pBrep->IsLiveTopologyMember(pAddTgt)) { continue ; }

          // switch on topology type
          if(pAddTgt->IsKindOf(SmFace_TYPE))   { sNewFaces.Add   ((SmFace *)  pAddTgt) ; }
          if(pAddTgt->IsKindOf(SmEdge_TYPE))   { sNewEdges.Add   ((SmEdge *)  pAddTgt) ; }
          if(pAddTgt->IsKindOf(SmVertex_TYPE)) { sNewVertices.Add((SmVertex *)pAddTgt) ; }
        } // end iter every input Tgt Topology

      // save the current old sizes - needed for upcoming Heal the new objects call 
      //  Update Target calls can nest, so the temp change is from the current OldValue to the current TgtObj size
      //  because if this is a nested call the current OldValue will be smaller than the current TgtObj Size.
      //  For a simple non-nested call the current OldValue should be 0.  When all the updating is done
      //  all the m_sOldObjCnt values should be set back to zero
      SmTemporaryChangeValue<ULONG> sChangeOldFaceCnt  (m_lOldFaceCnt,   m_sTgtFaces.GetSize()) ;
      SmTemporaryChangeValue<ULONG> sChangeOldEdgeCnt  (m_lOldEdgeCnt,   m_sTgtEdges.GetSize()) ;
      SmTemporaryChangeValue<ULONG> sChangeOldVertexCnt(m_lOldVertexCnt, m_sTgtVertices.GetSize()) ;

      // Add new Tgt Objs to the m_sTgtObjs arrays
      m_sTgtFaces   .Append(sNewFaces) ; 
      m_sTgtEdges   .Append(sNewEdges) ; 
      m_sTgtVertices.Append(sNewVertices) ; 

      // Size associated PropsArrays  - allocate and init ObjProps array members - Yuk! Calloc inits to Zero, Healer needs explicit InitValues (Add ReSet() calls)
      m_sTgtFaceProps.  SetSize(m_sTgtFaces   .GetSize()) ; for(ii=m_lOldFaceCnt  ;ii<m_sTgtFaces   .GetSize();ii++) { m_sTgtFaceProps[ii]   = new SmFaceProps(cpContext) ;   m_sTgtFaceProps[ii]->ReSet() ;   }
      m_sTgtEdgeProps.  SetSize(m_sTgtEdges   .GetSize()) ; for(ii=m_lOldEdgeCnt  ;ii<m_sTgtEdges   .GetSize();ii++) { m_sTgtEdgeProps[ii]   = new SmEdgeProps(cpContext) ;   m_sTgtEdgeProps[ii]->ReSet() ;   }
      m_sTgtVertexProps.SetSize(m_sTgtVertices.GetSize()) ; for(ii=m_lOldVertexCnt;ii<m_sTgtVertices.GetSize();ii++) { m_sTgtVertexProps[ii] = new SmVertexProps(cpContext) ; m_sTgtVertexProps[ii]->ReSet() ; }
#ifdef SM_DEBUG_CODE
      m_lPropsAllocCount += (m_sTgtFaces.GetSize()    - m_lOldFaceCnt)
                          + (m_sTgtEdges.GetSize()    - m_lOldEdgeCnt)
                          + (m_sTgtVertices.GetSize()  - m_lOldVertexCnt) ;
#endif // SM_DEBUG_CODE
                                                                   
      // limit the eThruHealerOp to the current SmHealData::m_eDoneHealOp value
      if(eThruHealerOp > m_eDoneHealOp)
        { eThruHealerOp = m_eDoneHealOp ; }

      // Restore HealerOp State when exiting this scope
      SmTemporaryChangeValue<SmHealerOpType> sChangeStop(m_eStopHealOp, eThruHealerOp) ;
      SmTemporaryChangeValue<SmHealerOpType> sChangeDone(m_eDoneHealOp, SM_HO_NONE) ;

      // run the heal sequence up to this eThruHealerOp on all the new Targets
      //   The heal sequence is setup to only run on the new tgts when m_lOldFaceCnt, m_lOldEdgeCnt, and/or m_lOldVertexCnt are Greater than 0
      m_lRecursionCount++ ;
#ifdef SM_DEBUG_CODE
      if(bDebugLog)
        { 
          smos_WriteBuffer(_T("\n\n***")) ;
          smos_sprintf(sBuff, _T("\n*** SmHealData::UpdateTargetLists: Beg Recursion RecDepth:[%d]::HealStep:[%d] for new NewTgtFaces:[%d], NewTgtEdges:[%d], NewTgtVertices:[%d] ***"), 
                              m_lRecursionCount, 
                              eThruHealerOp,
                              m_sTgtFaces.GetSize(), 
                              m_sTgtEdges.GetSize(), 
                              m_sTgtVertices.GetSize() ); 
          smos_WriteBuffer(sBuff) ;
          smos_WriteBuffer(_T("\n***\n")) ;
        }
#endif // SM_DEBUG_CODE

      // run recursive healing on New Tgts
      HealTgts(eThruHealerOp, FALSE) ; // FALSE = don't rebuild UVTrimCurves - doesn't matter, ThruHealerOp < SM_HO_MAKE_UVTRIMCURVES

#ifdef SM_DEBUG_CODE
      if(bDebugLog)
        { 
          smos_WriteBuffer(_T("\n\n***")) ;
          smos_sprintf(sBuff, _T("\n*** SmHealData::UpdateTargetLists: End Recursion RecDepth:[%d]::HealStep:[%d] for new NewTgtFaces:[%d], NewTgtEdges:[%d], NewTgtVertices:[%d] ***"), 
                              m_lRecursionCount, 
                              eThruHealerOp,
                              m_sTgtFaces.GetSize(), 
                              m_sTgtEdges.GetSize(), 
                              m_sTgtVertices.GetSize() ); 
          smos_WriteBuffer(sBuff) ;
          smos_WriteBuffer(_T("\n***\n")) ;
        }
#endif // SM_DEBUG_CODE
      if(m_lRecursionCount > 0) { m_lRecursionCount-- ; }

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { 
          ULONG di ;
          SmTArray<SmFaceProps *>   sNewFacesProps ;    for(di=m_lOldVertexCnt;di<m_sTgtFaces   .GetSize();di++) { sNewFacesProps   .Add(m_sTgtFaceProps  [ii]) ; }
          SmTArray<SmEdgeProps *>   sNewEdgesProps ;    for(di=m_lOldEdgeCnt  ;di<m_sTgtEdges   .GetSize();di++) { sNewEdgesProps   .Add(m_sTgtEdgeProps  [ii]) ; }
          SmTArray<SmVertexProps *> sNewVerticesProps ; for(di=m_lOldFaceCnt  ;di<m_sTgtVertices.GetSize();di++) { sNewVerticesProps.Add(m_sTgtVertexProps[ii]) ; }
          SmTArray<SmTopology *>    sCommonTopologies ;

          pRmList->FindCommonElements(*pAddList, sCommonTopologies) ; 

          if(sCommonTopologies.GetSize() > 0)
              smos_WriteBuffer(_T("\n  Common Ptrs in the Remove And Add lists: ")) ; 
            { smos_WriteBuffer(_T("\n    Common Add and Remove List Ptrs are Okay.  This happens when a deleted obj was replaced by a new one reusing the old ptr value. ")) ; 
              smos_WriteBuffer(_T("\n      (Example: Split - The parent ptr value is always reused by one of the two new children objects.)")) ; 
              sCommonTopologies.Dump() ;  
            }

          smos_WriteBuffer(_T("\n  RmList - after removing and adding: ")) ;  pRmList->Dump() ;
          DumpArraysContaining(0, &sRemoveFaceProps,
                                  &sRemoveEdgeProps,
                                  &sRemoveVertexProps) ;
          smos_WriteBuffer(_T("\n  AddList - after removing and adding : ")) ;  pAddList->Dump() ;
          DumpArraysContaining(1, &sNewFacesProps,  
                                  &sNewEdgesProps,  
                                  &sNewVerticesProps) ;
        } 
#endif // SM_DEBUG_CODE

   } // end pAddList topology Tgts exist check - run the heal sequence on the AddList topology objects up to the current heal step

  return SM_SUCCESS ;

} // end SmHealData::UpdateTargetLists

/*******************************************************************//**
PURPOSE: Remove Tgt VertexProps from all the SmHealData lists. Do NOT
         delete the Tgt VertexProps object

NOTES: also removes pVertex      from m_sTgtVertexs and
                    pVertexProps from m_sTgtVertexProps
***********************************************************************/
void SmHealData::RemoveVertexProps_FromLists
 (SmVertexProps * pTgtVertexProps,           ///< [in] : object to remove
  SmBoolean     * pOptRmFromTgtVertices,     ///< [out]: TRUE = Tgt Vertex was on TgtVertices lists, FALSE = wasn't, NULL to ignore, default:[NULL]
  SmBoolean     * pOptRmFromTgtVertexProps)  ///< [out]: TRUE = Tgt VertexProps was on TgtVertexProps lists, FALSE = wasn't, NULL to ignore, default:[NULL]
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { DumpVertexListsContaining(pTgtVertexProps) ; }
#endif // SM_DEBUG_CODE

  // remove Tgt Vertex and VertexProps from the Target lists
 SmBoolean bRmFromTgtVertices    = m_sTgtVertices.Remove((SmVertex*)pTgtVertexProps->m_pVertex) ; 
 SmBoolean bRmFromTgtVertexProps = m_sTgtVertexProps.Remove( pTgtVertexProps) ; 

  // when asked - set outputs
  if(pOptRmFromTgtVertices   ) { *pOptRmFromTgtVertices    = bRmFromTgtVertices ; }   
  if(pOptRmFromTgtVertexProps) { *pOptRmFromTgtVertexProps = bRmFromTgtVertexProps ; }

  // Base class
  m_sBadObjProps_BackPointers     .Remove( pTgtVertexProps) ;

  // remove Tgt VertexProps Ptr from all the various Heal VertexProps Ptr lists
  m_sBadVertexProps_SmallZoneTol3d.Remove( pTgtVertexProps) ;
  m_sBadVertexProps_LargeZoneTol3d.Remove( pTgtVertexProps) ;
  m_sWarnVertexProps_LargeGap3d   .Remove( pTgtVertexProps) ;
  m_sBadVertexProps_CoinVertices  .Remove( pTgtVertexProps) ;
  m_pVerticesInVoxels            ->Remove( pTgtVertexProps) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { DumpVertexListsContaining(pTgtVertexProps) ; }
#endif // SM_DEBUG_CODE

} // end SmHealData::RemoveVertexProps_FromLists

/*******************************************************************//**
PURPOSE: Remove Tgt EdgeProps from all the SmHealData lists. Do NOT
         delete the Tgt EdgeProps object

NOTES: also removes pEdge      from m_sTgtEdges and
                    pEdgeProps from m_sTgtEdgeProps
***********************************************************************/
void SmHealData::RemoveEdgeProps_FromLists
 (SmEdgeProps   * pTgtEdgeProps,           ///< [in] : object to remove
  SmBoolean     * pOptRmFromTgtEdges,      ///< [out]: TRUE = Tgt Edge was on TgtEdges lists, FALSE = wasn't, NULL to ignore, default:[NULL]
  SmBoolean     * pOptRmFromTgtEdgeProps)  ///< [out]: TRUE = Tgt EdgeProps was on TgtEdgeProps lists, FALSE = wasn't, NULL to ignore, default:[NULL]
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { DumpEdgeListsContaining(pTgtEdgeProps) ; }
#endif // SM_DEBUG_CODE

  // remove Tgt Edge and EdgeProps from the Target lists
 SmBoolean bRmFromTgtEdges     = m_sTgtEdges.Remove((SmEdge*)pTgtEdgeProps->m_pEdge) ; 
 SmBoolean bRmFromTgtEdgeProps = m_sTgtEdgeProps.Remove( pTgtEdgeProps) ; 

  // when asked - set outputs
  if(pOptRmFromTgtEdges    ) { *pOptRmFromTgtEdges     = bRmFromTgtEdges ; }   
  if(pOptRmFromTgtEdgeProps) { *pOptRmFromTgtEdgeProps = bRmFromTgtEdgeProps ; }

  // Base class
  m_sBadObjProps_BackPointers     .Remove( pTgtEdgeProps) ;

  // remove Tgt EdgeProps Ptr from all the various Heal EdgeProps Ptr lists
  m_sBadEdgeProps_Uncontained     .Remove( pTgtEdgeProps) ;
  m_sBadEdgeProps_SmallZoneTol3d  .Remove( pTgtEdgeProps) ;
  m_sBadEdgeProps_LargeZoneTol3d  .Remove( pTgtEdgeProps) ;
  m_sWarnEdgeProps_LargeGap3d     .Remove( pTgtEdgeProps) ;
  m_sBadEdgeProps_DegenEdges      .Remove( pTgtEdgeProps) ;
  m_sBadEdgeProps_CoinEdges       .Remove( pTgtEdgeProps) ;
  m_sBadEdgeProps_MissedEdgeXSects.Remove( pTgtEdgeProps) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { DumpEdgeListsContaining(pTgtEdgeProps) ; }
#endif // SM_DEBUG_CODE

} // end SmHealData::RemoveEdgeProps_FromLists

/*******************************************************************//**
PURPOSE: Remove Tgt FaceProps from all the SmHealData lists. Do NOT
         delete the Tgt FaceProps object

NOTES: also removes pLoopFace      from m_sTgtFaces and
                    pFaceProps from m_sTgtFaceProps
***********************************************************************/
void SmHealData::RemoveFaceProps_FromLists
 (SmFaceProps * pTgtFaceProps,           ///< [in] : object to remove
  SmBoolean   * pOptRmFromTgtFaces,      ///< [out]: TRUE = Tgt Face was on TgtFaces lists, FALSE = wasn't, NULL to ignore, default:[NULL]
  SmBoolean   * pOptRmFromTgtFaceProps)  ///< [out]: TRUE = Tgt FaceProps was on TgtFaceProps lists, FALSE = wasn't, NULL to ignore, default:[NULL]
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { DumpFaceListsContaining(pTgtFaceProps) ; }
#endif // SM_DEBUG_CODE

  // remove Tgt Face and FaceProps from the Target lists
 SmBoolean bRmFromTgtFaces     = m_sTgtFaces.Remove(pTgtFaceProps->m_pFace) ; 
 SmBoolean bRmFromTgtFaceProps = m_sTgtFaceProps.Remove( pTgtFaceProps) ; 

  // when asked - set outputs
  if(pOptRmFromTgtFaces    ) { *pOptRmFromTgtFaces     = bRmFromTgtFaces ; }   
  if(pOptRmFromTgtFaceProps) { *pOptRmFromTgtFaceProps = bRmFromTgtFaceProps ; }

  // Base class
  m_sBadObjProps_BackPointers     .Remove( pTgtFaceProps) ;

  // remove Tgt FaceProps Ptr from all the various Heal FaceProps Ptr lists
  m_sNotYetFaceProps                 .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_SmallZoneTol3d     .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_LargeZoneTol3d     .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_DegenFaces         .Remove(pTgtFaceProps) ;
  m_sFaceProps_Sheets                .Remove(pTgtFaceProps) ;
  m_sFaceProps_NoVertexPole          .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_FlatCorner         .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_MissingSeam        .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_CrossedSeam        .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_NearMissSeam       .Remove(pTgtFaceProps) ;
  m_sRanFaceProps_MoveSeam           .Remove(pTgtFaceProps) ;
  m_sRanFaceProps_SplitEdgesAtSeam   .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_LoopProblems       .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_OuterLoopOrder     .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_OrientLoop         .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_NoArea_Loop        .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_MultiOuterLoops    .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_NestedOuterLoops   .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_UnpairedNoAreaLoop .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_Closed3dLoop       .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_ClosedPtrLoop      .Remove(pTgtFaceProps) ;
  m_sBadFaceProps_MissingPole        .Remove(pTgtFaceProps) ;
  m_sRanFaceProps_SplitFaceAtSeam    .Remove(pTgtFaceProps) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { DumpFaceListsContaining(pTgtFaceProps) ; }
#endif // SM_DEBUG_CODE

} // end SmHealData::RemoveFaceProps_FromLists

/*******************************************************************//**
PURPOSE: Move a Face/FaceProps pair with a NotYet supported problem
         out of the list of Heal TgtFaces

NOTES:
***********************************************************************/
void SmHealData::MoveToNotYetProblems
 (SmFaceProps * pFaceProps)    ///< [in] : object to copy
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { DumpFaceListsContaining(pFaceProps) ; }
#endif // SM_DEBUG_CODE

  // locals
  SmBoolean bRmFromTgtFaces ;
  SmBoolean bRmFromTgtFaceProps ;
  
  // remove Tgt FaceProps Ptr from all the various Heal FaceProps Ptr lists (includes m_sNotYetFaceProps
  RemoveFaceProps_FromLists(pFaceProps, &bRmFromTgtFaces, &bRmFromTgtFaceProps) ;

  // Add FaceProps to NotYetFaceProps List - memory ownership moves with this pointer
  m_sNotYetFaceProps.Add(pFaceProps) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { DumpFaceListsContaining(pFaceProps) ; }
#endif // SM_DEBUG_CODE

} // end SmHealData::MoveToNotYetProblems

/*******************************************************************//**
PURPOSE: Look for and fix problems for the Topology objects within
         the target Brep.  Unless otherwise specified every Brep Face, 
         Edge, and Vertex will be targeted.

NOTES: The healer is a MakeTopologyFromData() helper function that finds
  and fixes database problems upon read so that the rest of SMLib can be written
  assuming the database they are working upon is valid.

  Once the SmHealData::m_sTgts Arrays (sTgtFaces, sTgtEdges, sTgtVertices) 
  and associated ObjProps are set the call is passed along 
  to SmHealData::HealTgts() where the actual healing is done.
***********************************************************************/
SmStatus SmHealData::HealBrep
 (SmBrep               * pBrep,              // in : Tgt Brep to heal
  SmHealerOpType         eHealerOp,          // in : Run Healer sequence up to and including this operation.  Oneof:
                                             //      SM_HO_NONE = 0,                           //  No Healer Steps run                                           <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_BACKPOINTERS,                   // Ran All Healer Steps thru SmHealData::Fix_BackPointers         <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_EDGEPROPS,                    // Ran All Healer Steps thru SmHealData::Cache_EdgeProps()        <br>
                                             //      SM_HO_CACHE_VERTEXPROPS,                  // Ran All Healer Steps thru SmHealData::Cache_VertexProps()      <br>   
                                             //      SM_HO_CACHE_FACEPROPS_GAPS,               // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Gaps()   <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_TOLSIZES,                       // Ran All Healer Steps thru SmHealData::Fix_TolSizes()           <br>
                                             //      SM_HO_CACHE_COIN_VERTICES,                // Ran All Healer Steps thru SmHealData::Cache_CoinVertices()     <br>
                                             //      SM_HO_FIX_COIN_VERTICES,                  // Ran All Healer Steps thru SmHealData::Fix_CoinVertices()       <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_DEGEN_FACES,                  // Ran All Healer Steps thru SmHealData::Cache_DegenFaces()       <br>
                                             //      SM_HO_FIX_DEGEN_FACES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenFaces()         <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_DEGEN_EDGES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenEdges()         <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_COIN_EDGES,        /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_CoinEdges()        <br>
                                             //      SM_HO_FIX_COIN_EDGES,          /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_CoinEdges()          <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_MISSED_EDGEXSECTS, /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_MissedEdgeXSects() <br>
                                             //      SM_HO_FIX_MISSED_EDGEXSECTS,   /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_MissedEdgeXSects()   <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_UNCONTAINED_EDGES,   /* STUB */ // Ran All Healer Steps thru SmHealData::Fix_UncontainedEdges()   <br>
                                             //      SM_HO_FIX_BADGAPS,             /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_BadGaps()            <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_FACEPROPS_2,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage2() <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_MOVESEAM,                       // Ran All Healer Steps thru SmHealData::Fix_MoveSeam()           <br>
                                             //      SM_HO_FIX_SPLITEDGE_ATSEAM,               // Ran All Healer Steps thru SmHealData::Fix_SplitEdgesAtSeam()   <br>
                                             //      SM_HO_FIX_BADSHEETS,                      // Ran All Healer Steps thru SmHealData::Fix_BadSheets()          <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_FACEPROPS_3,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage3() <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_SPLITFACE_ATSEAMS,              // Ran All Healer Steps thru SmHealData::Fix_SplitFaceAtSeams()   <br>
                                             //      SM_HO_FIX_BADLOOPS,                       // Ran All Healer Steps thru SmHealData::Fix_BadLoops()           <br>
                                             //      SM_HO_MAKE_UVTRIMCURVES,                  // Ran All Healer Steps thru SmHealData::Make_UVTrimCurves()      <br>
                                             //      SM_HO_FIX_INFINITE_REGIONS,               // Ran All Healer Steps thru SmHealData::Fix_InfiniteRegion()     <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_ALL                                 // Ran All Healer Steps                                           <br>
  SmTArray<SmVertex*>  * pOptVertices,       // in : optional TgtVertex list, NULL=use all Brep Vertices, (any vertices deleted by healing will be removed from this list)
                                             //      default:[NULL]
  SmTArray<SmEdge*>    * pOptEdges,          // in : optional EdgeProps list, NULL=use all Brep Edges,    (any edges    deleted by healing will be removed from this list)
                                             //      default:[NULL]
  SmTArray<SmFace*>    * pOptFaces,          // in : optional FaceProps list, NULL=use all Brep Faces,    (any faces    deleted by healing will be removed from this list)
                                             //      default:[NULL]
  SmBoolean              bMakeUVTrimCurves   // in : TRUE=call SmFace::CreateUVTrimCurves for each face, default:[FALSE]=Don't
 )
{
  // no work - no Brep
  if(pBrep == NULL)
    { return SM_SUCCESS ; }

  // TgtLocals
  SmTArray<SmVertex *> sTgtVertices ;
  SmTArray<SmEdge *>   sTgtEdges ;
  SmTArray<SmFace *>   sTgtFaces ;

  // TgtVertices - When pOptVertices is NULL Tgt all Vertices
  if(pOptVertices == NULL) { pBrep->GetVertices(sTgtVertices) ; }
  else                     { sTgtVertices.Append(*pOptVertices) ; }

  // TgtEdges - When pOptEdges is NULL Tgt all Edges
  if(pOptEdges == NULL)    { pBrep->GetEdges(sTgtEdges) ; }
  else                     { sTgtEdges.Append(*pOptEdges) ; }
                      
  // TgtFaces - When pOptFaces is NULL Tgt all Faces
  if(pOptFaces == NULL)    { pBrep->GetFaces(sTgtFaces) ; }
  else                     { sTgtFaces.Append(*pOptFaces) ; } 

  // clear this HealData object
  ReSet(TRUE) ; // TRUE = Reset the m_sNewObj array lists, FALSE = don't

#ifdef SM_DEBUG_CODE
TCHAR     sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
// ULONG GWC_CHANGE_NEXT_LINE_TO_FALSE_BEFORE_RELEASE_AND_TRUE_HEAL_DEBUG ;
SmBoolean bDebugMe          = FALSE ; // GWC::SET_LINE_TO_TRUE_FOR_HEAL_DEBUG
  if(bDebugMe)
    {
      // starting to Heal inform the public
      smos_sprintf(sBuff,        _T("\nSmHealData::HealBrep: *** Before LoadTargets() and HealTgts() calls on pBrep:[0x%p] ***"), pBrep);
      smos_sprintf(sBuffForFile, _T("\nSmHealData::HealBrep: *** Before LoadTargets() and HealTgts() calls on pBrep:[0x%p] ***"), _T("notNULL"));
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }
#endif // SM_DEBUG_CODE                                                         

  // Gather Tgt Arrays          (m_sTgtVertices,    m_sTgtEdges,     m_sTgtFaces) 
  // - allocate TgtProps memory (m_sTgtVertexProps, m_sTgtEdgeProps, m_sTgtFaceProps)
  LoadTargets(pBrep, sTgtVertices, sTgtEdges, sTgtFaces) ;

  // arrive here: the SmHealData::Tgts Arrays are set, 
  // next: pass the call to HealTgts for healing

  // Heal the tgts in m_sTgtObjs and m_sTgtObjProps
  SmStatus sRtn = HealTgts(eHealerOp, bMakeUVTrimCurves) ; 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // starting to Heal inform the public
      smos_sprintf(sBuff,        _T("\nSmHealData::HealBrep: *** After LoadTargets() and HealTgts() calls on pBrep:[0x%p] ***"), pBrep);
      smos_sprintf(sBuffForFile, _T("\nSmHealData::HealBrep: *** After LoadTargets() and HealTgts() calls on pBrep:[0x%p] ***"), _T("notNULL"));
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }
#endif // SM_DEBUG_CODE                                                         

  return( sRtn ) ; 

} // end SmHealData::HealBrep

/*******************************************************************//**
PURPOSE: Run the Heal Cache/Fix step sequence on the targets stored
  within the SmHealData Target arrays.

NOTES: 
  The healer is a MakeTopologyFromData() helper function that finds and
  fixes database problems upon read so that the rest of SMLib can be written
  assuming the database they are working upon is valid.

  Healer general Design Rules: 
    1. The healer is implemented as a sequence of steps where each Heal Step 
       moves a Brep from an unknown but possibly invalid state one step closer
       towards being a valid Brep.  There are two kinds of steps: Cache steps and Fix steps.
       
    2. Cache steps calculate and store properties of a tgt object and check those
       values for possible problems.  When a Cache step finds a problem it loads
       a ptr to that TgtObj's SmObjProp into that problem's ProbArray and sets 
       a member problem flag value within that TgtObj's SmObjProp object.
       
    3. FIX steps fix a problem for all the entries in the associated SmHealData::ProbArray. 
       When the fix algorithm succeeds, it clears the member problem flag value 
       within that TgtObj's SmObjProp object
       
    4. Every known problem is associated with a SmHealData::ProbArray and a boolean
       (or ULONG) value within the SmObjProps objects.  When a Obj is found to have
       a particular problem within a Cache step, a pointer to the Obj's associated SmObjProps
       object is loaded into the associated ProbArray and the problem bit in the SmObjProps
       object is set.

    5. When a FIX function fixes the problem, the SmObjProps problem bit is cleared. If
       the fix deletes the SmObj then the stale ptrs to the deleted object are removed
       from all the SmHealData::ProbArrays.  Otherwise the SmObjProps are left in the
       problem array.  An object only needs fixing if it is both in a ProbArray and
       has its problem bit set.  The fixed SmObjProps ptrs are left in the problem
       array for later reporting and display.  This data will answer the question,
       which objects in the Brep were fixed?

    6. The heart of the healer is that the fixes for the anticipated Brep problems have been
       engineered and sequenced so that later heal steps can assume that problem types fixed
       by earlier heal steps are not in the Brep. For example, removing degenerate edges happens
       before inserting missing Seam edges.  The missing seam edge code can be written assuming
       the Brep has no degenerate edges.

    7. Marks:
        Clever use of marks can change some O:(N*N) problems into O:(N) or even O:(const) ones
        currently marks are being used to help accelerate the heal steps that: 
              o. Find and Split Shells with SheetFaces that link two closed Shells together by mistake, 
              o. Find and Split Edges that cross Seams.
       If a step uses marks, it can use a pair of global mark variables available to all the
       healer steps as it wishes but each healer step has to be done using the mark by  
       the time it exits its step scope.

    8. To improve performance and readability, object state is passed from heal step to 
       another by the classes SmFaceProps, SmEdgeProps, SmLoopProps, and SmVertexProps. 
       These ObjectProperty classes store a mix of Obj Geometry values (ex. EdgeLength)
       and Obj database problems (ex. IsDegenerateEdge). The member values of those 
       classes can be set in one heal step and then used any number
       of times in subsequent heal steps. The objectProperty class members should probably
       be implemented with lazy evaluation and ready bits but that was not done.
       Any order dependencies in the member values has been imbedded in the code.

    9. Problem arrays are instances of the SmProbArray<TYPE> class hierarchy.
       The classes derived from SmProbArray<TYPE> allow the SmObjProps::ProbFlag member
       to be of various different kinds of enums that are appropriate for tracking an objects
       state as it moves through the healer in a more complicated fashion than
       having the problem and being fixed.

       The SmProbArray classes manage pretty printing and drawing in standard manners.

    10. If you can, as you extend the healer, try sticking with this design.
          For each new problem:
           o. Create a new SmHealData::ProbArray(). Use copy and edit to make
              sure that Array is managed in the same places in the code as
              all the other ProbArrays.
           o. Create a new SmObjProps::m_bBadProblem flag.
           o. Write one predicate for the problem.  Odds are that predicate
              can be added to an existing Cache_ObjProps() method.  But 
              if not, create your own new SmHealData::Cache_MyProblem() method.
           o. Write one new Fix_MyProblem() method.  Copy and edit any of the
              existing Fix_problem() methods to make sure the new Fix method
              is written in the same style as the rest of the methods.

    11. Many fix functions will Remove and/or Add topology objects to the
        Brep's topology graph.  Whenever that happens a tremendous amount
        of work needs to be done to Update all the ProbArrays and to
        create and set new SmObjProps objects for each newly added
        topology objects.  Moreover, pointer values get reused, deleting
        an edge and then creating a new one may end up reusing the
        pointer value.  The system has to distinguish between a ptr to 
        an unchanged topology object and a reused ptr.  The class
        SmTrackTopologyChanges and the method SmHealData::UpdateTargetLists()
        manage this issue.  Look in the existing Fix() methods to see how
        they are used.

    Enjoy.
***********************************************************************/
SmStatus SmHealData::HealTgts
 (SmHealerOpType         eHealerOp,          // in : Run Healer sequence up to and including this operation.  Oneof:
                                             //      SM_HO_NONE = 0,                           //  No Healer Steps run                                           <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_BACKPOINTERS,                   // Ran All Healer Steps thru SmHealData::Fix_BackPointers         <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_EDGEPROPS,                    // Ran All Healer Steps thru SmHealData::Cache_EdgeProps()        <br>
                                             //      SM_HO_CACHE_VERTEXPROPS,                  // Ran All Healer Steps thru SmHealData::Cache_VertexProps()      <br>   
                                             //      SM_HO_CACHE_FACEPROPS_GAPS,               // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Gaps()   <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_TOLSIZES,                       // Ran All Healer Steps thru SmHealData::Fix_TolSizes()           <br>
                                             //      SM_HO_CACHE_COIN_VERTICES,                // Ran All Healer Steps thru SmHealData::Cache_CoinVertices()     <br>
                                             //      SM_HO_FIX_COIN_VERTICES,                  // Ran All Healer Steps thru SmHealData::Fix_CoinVertices()       <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_DEGEN_FACES,                  // Ran All Healer Steps thru SmHealData::Cache_DegenFaces()       <br>
                                             //      SM_HO_FIX_DEGEN_FACES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenFaces()         <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_DEGEN_EDGES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenEdges()         <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_COIN_EDGES,        /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_CoinEdges()        <br>
                                             //      SM_HO_FIX_COIN_EDGES,          /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_CoinEdges()          <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_MISSED_EDGEXSECTS, /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_MissedEdgeXSects() <br>
                                             //      SM_HO_FIX_MISSED_EDGEXSECTS,   /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_MissedEdgeXSects()   <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_UNCONTAINED_EDGES,   /* STUB */ // Ran All Healer Steps thru SmHealData::Fix_UncontainedEdges()   <br>
                                             //      SM_HO_FIX_BADGAPS,             /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_BadGaps()            <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_FACEPROPS_2,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage2() <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_MOVESEAM,                       // Ran All Healer Steps thru SmHealData::Fix_MoveSeam()           <br>
                                             //      SM_HO_FIX_SPLITEDGE_ATSEAM,               // Ran All Healer Steps thru SmHealData::Fix_SplitEdgesAtSeam()   <br>
                                             //      SM_HO_FIX_BADSHEETS,                      // Ran All Healer Steps thru SmHealData::Fix_BadSheets()          <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_CACHE_FACEPROPS_3,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage3() <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_FIX_SPLITFACE_ATSEAMS,              // Ran All Healer Steps thru SmHealData::Fix_SplitFaceAtSeams()   <br>
                                             //      SM_HO_FIX_BADLOOPS,                       // Ran All Healer Steps thru SmHealData::Fix_BadLoops()           <br>
                                             //      SM_HO_MAKE_UVTRIMCURVES,                  // Ran All Healer Steps thru SmHealData::Make_UVTrimCurves()      <br>
                                             //      SM_HO_FIX_INFINITE_REGIONS,               // Ran All Healer Steps thru SmHealData::Fix_InfiniteRegion()     <br>
                                             //                                                                                                                  <br>
                                             //      SM_HO_ALL                                 // Ran All Healer Steps                                           <br>
  SmBoolean              bMakeUVTrimCurves   // in : TRUE=call SmFace::CreateUVTrimCurves for each face, default:[FALSE]=Don't
 )
{  // locals
  const SmContext * cpContext = m_pBrep->GetContext() ;  // use Brep's context for new obj construction (temp and persistent)

  // HealerOp Management - set number of Heal Steps to execute
  m_eStopHealOp = eHealerOp ;
  m_eDoneHealOp = SM_HO_NONE ;

  // enable Brep Editing 
  SmTemporaryChangeValue<SmBoolean> sTmpChange(m_pBrep->m_bEditingEnabled, TRUE);

  // Marks - mark use changes some O:(N*N) problems into O:(N) or even O:(const) ones
  //         currently marks are being used in 
  //               Heal_2: Find and Split Shells with SheetFaces that link two closed Shells together by mistake, 
  //           and Heal_4: when Faces have LoopEdges crossing seams - split Edges at Seam
  SmNewMarkAndLock sMarkLock(cpContext) ;

  // Begin Method local MACRO - Lbl not used - just for code readability
#ifdef SM_DEBUG_CODE
#define SM_HEALSTEP_0ARG(lbl, Func, StepNo) if(m_eDoneHealOp < eHealerOp) \
                                              { if(bReportStepProgress) { smos_sprintf(sBuff, _T("\n*** SmHealData::HealTgts: Before Heal Func call, RecDepth:[%d]::HealStep:[%d], %s() ***"), m_lRecursionCount, lbl, _T(#Func)); smos_WriteBuffer(sBuff) ; } \
                                                SmStatus sMacroStat = Func(m_pBrep, StepNo) ;   \
                                                if(sMacroStat == SM_SUCCESS) { m_eDoneHealOp = StepNo ; } \
                                                else { SER(sMacroStat) ; } \
                                                if(bReportStepProgress) { smos_sprintf(sBuff, _T("\n*** SmHealData::HealTgts: After  Heal Func call, RecDepth:[%d]::HealStep:[%d], %s() ***"), m_lRecursionCount, lbl, _T(#Func));  smos_WriteBuffer(sBuff) ; } \
                                              }

#define SM_HEALSTEP_1ARG(lbl, Func, PtrToBoolArg, StepNo) if(m_eDoneHealOp < eHealerOp)    \
                                                   { if(bReportStepProgress) { smos_sprintf(sBuff, _T("\n*** SmHealData::HealTgts: Before Heal Func call, RecDepth:[%d]::HealStep:[%d], %s() ***"), m_lRecursionCount, lbl, _T(#Func));  smos_WriteBuffer(sBuff) ; } \
                                                     SmStatus sMacroStat = Func(m_pBrep, PtrToBoolArg, StepNo) ; \
                                                     if(sMacroStat == SM_SUCCESS) { m_eDoneHealOp = StepNo ; } \
                                                     else { SER(sMacroStat) ; } \
                                                     if(bReportStepProgress) { smos_sprintf(sBuff, _T("\n*** SmHealData::HealTgts: After  Heal Func call, RecDepth:[%d]::HealStep:[%d], %s() ***"), m_lRecursionCount, lbl, _T(#Func));  smos_WriteBuffer(sBuff) ; } \
                                                   }
#define SM_HEALSTEP_BEG_STATE_DUMP(lbl, Func)  if(bReportStepProgress) { smos_sprintf(sBuff, _T("\n*** SmHealData::HealTgts: Begin  HealState Dump, RecDepth:[%d]::PostStep:[%d], %s() ***\n"), m_lRecursionCount, lbl, _T(#Func) );  smos_WriteBuffer(sBuff) ; }                                                  
#define SM_HEALSTEP_END_STATE_DUMP(lbl, Func)  if(bReportStepProgress) { smos_sprintf(sBuff, _T("\n\n*** SmHealData::HealTgts: End    HealState Dump, RecDepth:[%d]::PostStep:[%d], %s() ***"), m_lRecursionCount, lbl, _T(#Func) );  smos_WriteBuffer(sBuff) ; }                                                  

#else // no SM_DEBUG_CODE
#define SM_HEALSTEP_0ARG(lbl, Func, StepNo) if(m_eDoneHealOp < eHealerOp)   \
                                              { SmStatus sMacroStat = Func(m_pBrep, StepNo) ;     \
                                                if(sMacroStat == SM_SUCCESS) { m_eDoneHealOp = StepNo ; } \
                                                else { SER(sMacroStat) ; } \
                                              }
#define SM_HEALSTEP_1ARG(lbl, Func, arg, StepNo) if(m_eDoneHealOp < eHealerOp)    \
                                                   { SmStatus sMacroStat = Func(m_pBrep, arg, StepNo) ; \
                                                     if(sMacroStat == SM_SUCCESS) { m_eDoneHealOp = StepNo ; } \
                                                     else { SER(sMacroStat) ; } \
                                                   }
#endif // no SM_DEBUG_CODE                                                         

  // End Method local Macro
  
  SmBoolean bCache_EdgeProps_FoundNewProbs = FALSE;
  SmBoolean bCache_VertexProps_FoundNewProbs = FALSE;
  SmBoolean bCache_FaceProps_FoundNewProbs = FALSE;
  SmBoolean bCache_CoincVertices_FoundNewProbs = FALSE;
  SmBoolean bCache_DegenFace_FoundNewProbs = FALSE;
  SmBoolean bCache_CoinEdges_FoundNewProbs = FALSE;
  SmBoolean bCache_MissedEdgeXSects_FoundNewProbs = FALSE;
  SmBoolean bCache_FaceProps_Stage2_FoundNewProbs = FALSE;
  SmBoolean bCache_FaceProps_Stage3_FoundNewProbs = FALSE;

#ifdef SM_DEBUG_CODE
// ULONG GWC_CHANGE_NEXT_LINE_TO_FALSE_BEFORE_RELEASE_AND_TRUE_HEAL_DEBUG ; 
SmBoolean bDebugMe          = FALSE ;   // GWC::SET_LINE_TO_TRUE_FOR_HEAL_DEBUG
SmBoolean bReportErrorFinds = FALSE ;   // GWC::SET_LINE_TO_TRUE_FOR_FOUND_A_DATABASE_ERROR_REPORTS
SmBoolean bReportStepProgress = FALSE ; // GWC::SET_LINE_TO_TRUE_FOR_START_AND_STOP_HEALSTEP_REPORTS
TCHAR     sBuff[SM_TBLOCK_SIZE] ;

// debug block locals

  // define debug macros - helpful in keeping the upcoming sequence tidy
  #define SM__DRAW_FACES_AND_SURFACES { ULONG mi ; \
                                        smgfx_Erase(TRUE) ; \
                                        for(mi=0;mi<m_sTgtFaces.GetSize();mi++) \
                                          { if(m_sTgtFaces[mi]->IsSheet()) { smgfx_SetLook(1,2, 1,0,1) ; m_sTgtFaces[mi]->DrawUV() ; sm_GraphicsLoop() ; } \
                                            else                           { smgfx_SetLook(1,2, 0,0,1) ; m_sTgtFaces[mi]->DrawUV() ; sm_GraphicsLoop() ; } \
                                            smgfx_SetLook(1,2, 0,1,1) ;    m_sTgtFaces[mi]->GetSurface()->DrawUV(11,11) ; \
                                            smgfx_SetLook(3,4, .3,.3,.3) ; m_sTgtFaces[mi]->GetSurface()->DrawSeams() ; \
                                          } \
                                      }
  #define SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP { m_pBrep->Dump() ; \
                                                    Dump() ; \
                                                    smgfx_Erase(TRUE) ; \
                                                    smgfx_SetLook(2,3, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; \
                                                  }
#else // no SM_DEBUG_CODE
  #define SM__DRAW_BREP_FACES_AND_SURFACES
  #define SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP                                  
#endif // no SM_DEBUG_CODE                                                         

  // The Heal Sequence

  // 1. The set of Cache_ObjProps() calls gather and cache obj property data and test that data
  //    for known problems.  Detected problems are marked by setting ProbFlag member values
  //    within the SmObjProbs objects.  Ptrs to SmObjProps objects whose ProbFlags are set are
  //    added to associated SmHealData::ProbArrays.
  // 2. The set of Fix_Problem() calls are built to fix one specific problem (or set of interacting problems)
  //    for one kind of SmObj class.  Each FixProblem() method iterates through the members
  //    of the SmHealData::ProbArray list for that problem.  When a problem is fixed
  //    the associated SmObjProps::ProblemFlag is changed appropriately.
  //    2a. To allow new objects to be healed to the same state as the initial objects
  //        the Fix_Problem() calls only iterate over the newly added members.  This
  //        is managed by the m_lOldVertexCnt
  //                          m_lOldEdgeCnt  
  //                          m_lOldFaceCnt 
  //        values.  When those are set to zero - every obj in a ProbArray is run through the
  //        fix sequence.  When that number is larger than 0 only the last members of the
  //        list are run through the fix sequence.  The call SmHealData::UpdateTargetLists()
  //        manages all this for the caller.  When a heal function creates new Objs,
  //        those are added to the SmHealData target lists and brought up to date
  //        by being passed to SmHealData::UpdateTargetLists()
  // 3. The heal sequence is very order dependent.  That allows heal steps later in the
  //    sequence to assume that the Brep is free of certain problems as they run.  
  //    For example, The step that finds coincident vertices can assume that the vertices
  //    all have problem free tolerances because the steps to check and fix tolerances
  //    run before the step to find coincident vertices.  The order in which 
  //    properties are computed, problems detected, and problems fixed has been
  //    engineered based on heuristic experience in fixing database problems.

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
    { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] Just Entered HealTgts() - no heal steps run\n"), m_lRecursionCount) ; 
      smos_WriteBuffer(sBuff,NULL,sBuff) ;
      SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
      SM__DRAW_FACES_AND_SURFACES
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Build ObjPropsLists for Vertices, Edges, Faces allowing for EdgesCrossingSeam problems
  //   - runs pEdgeProps[ii]->SetProps()
  SM_HEALSTEP_1ARG(1, Cache_EdgeProps, &bCache_EdgeProps_FoundNewProbs, SM_HO_CACHE_EDGEPROPS)
#ifdef SM_DEBUG_CODE
      if(bCache_EdgeProps_FoundNewProbs && (m_eDoneHealOp >= 1))
        {  
          SM_HEALSTEP_BEG_STATE_DUMP(1, Cache_EdgeProps);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_CoinVertices detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 1 = Cache_EdgeProps\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(1, Cache_EdgeProps);
        }
#endif // SM_DEBUG_CODE
                                     
  SM_HEALSTEP_1ARG(2, Cache_VertexProps, &bCache_VertexProps_FoundNewProbs, SM_HO_CACHE_VERTEXPROPS)
#ifdef SM_DEBUG_CODE
      if(bCache_VertexProps_FoundNewProbs && (m_eDoneHealOp >= 2))
        {  
          SM_HEALSTEP_BEG_STATE_DUMP(2, Cache_VertexProps);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_CoinVertices detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 2 = Cache_VertexProps\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(2, Cache_VertexProps);
        }
#endif // SM_DEBUG_CODE
                
  SM_HEALSTEP_1ARG(3, Cache_FaceProps_Gaps, &bCache_FaceProps_FoundNewProbs, SM_HO_CACHE_FACEPROPS_GAPS)
#ifdef SM_DEBUG_CODE
      if(bCache_FaceProps_FoundNewProbs && (m_eDoneHealOp >= 3))
        {  
          SM_HEALSTEP_BEG_STATE_DUMP(3, Cache_FaceProps_Gaps);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_CoinVertices detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 3 = Cache_FaceProps_Gaps\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(3, Cache_FaceProps_Gaps);
        }
#endif // SM_DEBUG_CODE
                
  // Fix DefZoneTol3d based on a PartSize estimate, and Set ObjTols based on DefZoneTol3d and ObjGapSizes
  //   - a precondition for SmHealData::Fix_MoveSeam()
  SM_HEALSTEP_0ARG(4, Fix_TolSizes, SM_HO_FIX_TOLSIZES)
#ifdef SM_DEBUG_CODE
       if(bDebugMe && (m_eDoneHealOp >= 4))
         { SM_HEALSTEP_BEG_STATE_DUMP(4, Fix_TolSizes); 
           SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 4 = Fix_TolSizes\n"), m_lRecursionCount) ;
           SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
           SM__DRAW_FACES_AND_SURFACES
           sm_GraphicsLoop() ;
           SM_HEALSTEP_END_STATE_DUMP(4, Fix_TolSizes); 
         }
#endif // SM_DEBUG_CODE

  // Find/Mend CoinVertices - a precondition for Find/Mend Degenerate Edges/Faces
SM_HEALSTEP_1ARG(5, Cache_CoinVertices, &bCache_CoincVertices_FoundNewProbs, SM_HO_CACHE_COIN_VERTICES)
#ifdef SM_DEBUG_CODE
      if(bCache_CoincVertices_FoundNewProbs && (m_eDoneHealOp >= 5))
        {  
          SM_HEALSTEP_BEG_STATE_DUMP(5, Cache_CoinVertices);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_CoinVertices detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 5 = Cache_CoinVertices\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(5, Cache_CoinVertices);
        }
#endif // SM_DEBUG_CODE

  SM_HEALSTEP_0ARG(6, Fix_CoinVertices, SM_HO_FIX_COIN_VERTICES)                           
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 6))
        { SM_HEALSTEP_BEG_STATE_DUMP(6, Fix_CoinVertices);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 6 = Fix_CoinVertices\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(6, Fix_CoinVertices);
        }
#endif // SM_DEBUG_CODE
                                                                                          
  // Find/Mend DegenFaces - a precondition for Find/Mend Degenerate Edges
  SM_HEALSTEP_1ARG(7, Cache_DegenFaces, &bCache_DegenFace_FoundNewProbs, SM_HO_CACHE_DEGEN_FACES)
#ifdef SM_DEBUG_CODE
      if(bCache_DegenFace_FoundNewProbs && (m_eDoneHealOp >= 7))
        { 
          SM_HEALSTEP_BEG_STATE_DUMP(7, Cache_DegenFaces);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_DegenFaces detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 7 = Cache_DegenFaces\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(7, Cache_DegenFaces);
        }
#endif // SM_DEBUG_CODE

  // Squeeze degenerate faces 
  SM_HEALSTEP_0ARG(8, Fix_DegenFaces, SM_HO_FIX_DEGEN_FACES)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 8))
        { SM_HEALSTEP_BEG_STATE_DUMP(8, Fix_DegenFaces);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 8 = Fix_DegenFaces\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(8, Fix_DegenFaces);
        }
#endif // SM_DEBUG_CODE

  // squeeze degenerate edges
  SM_HEALSTEP_0ARG(9, Fix_DegenEdges, SM_HO_FIX_DEGEN_EDGES)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 9))
        { SM_HEALSTEP_BEG_STATE_DUMP(9, Fix_DegenEdges);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 9 = Fix_DegenEdges\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(9, Fix_DegenEdges);
        }
#endif // SM_DEBUG_CODE

  // TODO: NotYet Implemented
  // Find/Mend CoinEdges - a precondition for Get EdgeProps             
  SM_HEALSTEP_1ARG(10, Cache_CoinEdges, &bCache_CoinEdges_FoundNewProbs, SM_HO_CACHE_COIN_EDGES)            
#ifdef SM_DEBUG_CODE
      if(bCache_CoinEdges_FoundNewProbs && (m_eDoneHealOp >= 10))
        { 
          SM_HEALSTEP_BEG_STATE_DUMP(10, Cache_CoinEdges);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_CoinEdges detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 10 = Cache_CoinEdges\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(10, Cache_CoinEdges);
        }
#endif // SM_DEBUG_CODE
                                                                        
  // TODO: NotYet Implemented
  SM_HEALSTEP_0ARG(11, Fix_CoinEdges, SM_HO_FIX_COIN_EDGES)            
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 11))
        { SM_HEALSTEP_BEG_STATE_DUMP(11, Fix_CoinEdges);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 11 = Fix_CoinEdges\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(11, Fix_CoinEdges);
        }
#endif // SM_DEBUG_CODE

  // Cache MissedEdgeXSects
  SM_HEALSTEP_1ARG(12, Cache_MissedEdgeXSects, &bCache_MissedEdgeXSects_FoundNewProbs, SM_HO_CACHE_MISSED_EDGEXSECTS)
#ifdef SM_DEBUG_CODE
      if(bCache_MissedEdgeXSects_FoundNewProbs && (m_eDoneHealOp >= 12))
        { 
          SM_HEALSTEP_BEG_STATE_DUMP(12, Cache_MissedEdgeXSects);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_MissedEdgeXSects detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 12 = Cache_MissedEdgeXSects \n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(12, Cache_MissedEdgeXSects);
        }
#endif // SM_DEBUG_CODE

  // fix MissedEdgeXSects
  SM_HEALSTEP_0ARG(13, Fix_MissedEdgeXSects, SM_HO_FIX_MISSED_EDGEXSECTS)            
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 13))
        { SM_HEALSTEP_BEG_STATE_DUMP(13, Fix_MissedEdgeXSects);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 13 = Fix_MissedEdgeXSects\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(13, Fix_MissedEdgeXSects);
        }
#endif // SM_DEBUG_CODE

  // Fix UncontainedEdges
  SM_HEALSTEP_0ARG(14, Fix_UncontainedEdges,     SM_HO_FIX_UNCONTAINED_EDGES)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 14))
        { SM_HEALSTEP_BEG_STATE_DUMP(14, Fix_UncontainedEdges);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 14 = Fix_UncontainedEdges\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(14, Fix_UncontainedEdges);
        }
#endif // SM_DEBUG_CODE

  // TODO: SmHealData::Fix_BigGaps()
  //       Idea: use a combination of
  //               SetVertexToFaceFaceFaceXSect()
  //               TrimOrExtendEdgeToVertex()
  //               TrimOrExtendFaceToBoundaries()
  //               DeformFaceToBoundaries()
  //               DeformEdgeToBoundaries()
  //             To create a 'GapHealer' that changes ObjShapes to minimize gaps.  
  //             For every Object whose shape has changed - rerun the appropriate SmHealData::Cache_ObjProps().
  //       Observation: When working with a part with big gaps one has two ways to think about the part
  //             1. The FaceSurfaces are primary: Extend Surfaces if needed to guarantee that neighbor
  //                surfaces intersect without gaps.  Reintersect the Surfaces to find target locations 
  //                and curve shapes for moving VertexPoints and reshaping EdgeCurves to fit the given FaceSurface shapes.
  //                1a. This approach will break down when two surfaces are tangent to one another.  In which
  //                    case, select VertexPoint and EdgeCurve shapes along the Tangent boundary and
  //                    Deform the Surface to interpolate VertexPoints and minimize GapSizes of the EdgeCurves.
  //             2. The VertexPoints are primary, the EdgeCurves are secondary, and the FaceSurfaces tertiary:
  //                Extend/Trim Edges to VertexPoints to guarantee Vertex/Edge gaps are perp to the EdgeCurve, 
  //                Deform Edges to VertexPoints to interpolate VertexPoints, Extend/Trim FaceSurfaces to guarantee
  //                Face/Vertex and Face/Edge gaps are perpendicular to the FaceSurface.  Deform FaceSurface
  //                to interpolate VertexPoints and minimize the Face/Edge Gaps.

  // TODO: NotYet Implemented
  // Fix Gap problems - can edit shapes - reruns GetProps methods as needed                  
  SM_HEALSTEP_0ARG(15, Fix_BadGaps, SM_HO_FIX_BADGAPS)                     
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 15))
        { SM_HEALSTEP_BEG_STATE_DUMP(15, Fix_BadGaps);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 15 = Fix_BadGaps\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(15, Fix_BadGaps);
        }
#endif // SM_DEBUG_CODE
                                                                              
  // Build FaceProps before Splitting SeamCrossing edges
  //   - runs SmFaceProps::SetProps_Stage2()
  SM_HEALSTEP_1ARG(16, Cache_FaceProps_Stage2, &bCache_FaceProps_Stage2_FoundNewProbs, SM_HO_CACHE_FACEPROPS_2)
#ifdef SM_DEBUG_CODE
      if(bCache_FaceProps_Stage2_FoundNewProbs && (m_eDoneHealOp >= 16))
        {  
          SM_HEALSTEP_BEG_STATE_DUMP(16, Cache_FaceProps_Stage2);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_FaceProps_Stage2 detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 16 = Cache_FaceProps_Stage2\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(16, Cache_FaceProps_Stage2);
        }
#endif // SM_DEBUG_CODE

  // Try Fixing SeamProblem Faces with MoveSeam(). 
  //   - If possible out of the Face, otherwise to minimize SliverFaces and NearSeam FaceBoundaries.
  //   - a precondition for SmHealData::Fix_SplitEdgesAtSeam()  
  SM_HEALSTEP_1ARG(17, Fix_MoveSeam, sMarkLock, SM_HO_FIX_MOVESEAM)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 17))
        { SM_HEALSTEP_BEG_STATE_DUMP(17, Fix_MoveSeam);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 17 = Fix_MoveSeam\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(17, Fix_MoveSeam);
        }
#endif // SM_DEBUG_CODE

  // Fix Edges Crossing Seams 
  //   - Split Edges at SeamCrossingPoints creating new ChildEdges and Vertices
  //   - a precondition for SmHealData::Cache_FaceProps_Stage3() 
  SM_HEALSTEP_1ARG(18, Fix_SplitEdgesAtSeam, sMarkLock, SM_HO_FIX_SPLITEDGE_ATSEAM)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 18))
        { SM_HEALSTEP_BEG_STATE_DUMP(18, Fix_SplitEdgesAtSeam);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 18 = Fix_SplitEdgesAtSeam\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(18, Fix_SplitEdgesAtSeam);
        }
#endif // SM_DEBUG_CODE
  
  // Fix BadSheets (Can Split Regions)
  SM_HEALSTEP_1ARG(19, Fix_BadSheets, sMarkLock, SM_HO_FIX_BADSHEETS)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 19))
        { SM_HEALSTEP_BEG_STATE_DUMP(19, Fix_BadSheets);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 19 = Fix_BadSheets\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(19, Fix_BadSheets);
        }
#endif // SM_DEBUG_CODE

  // From this point on it should be okay to use UVTrimCurves since edges no longer cross seams
  //   and the tolerances have been adjusted for the GapSizes.
  // However, our experience has shown that creating UVTrimCurves is always problematic in Breps with 
  //   database errors waiting to be fixed.  As a rule try to build any following heal functions 
  //   so they don't use UVTrimCurves.

  // Build FaceProps after splitting SeamCrossing Edges
  //   - Builds LoopProps and finds Loop problems
  SM_HEALSTEP_1ARG(20, Cache_FaceProps_Stage3, &bCache_FaceProps_Stage3_FoundNewProbs, SM_HO_CACHE_FACEPROPS_3)
#ifdef SM_DEBUG_CODE
      if(bCache_FaceProps_Stage3_FoundNewProbs && (m_eDoneHealOp >= 20))
        { 
          SM_HEALSTEP_BEG_STATE_DUMP(20, Cache_FaceProps_Stage3);
          if(bReportErrorFinds) { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] HealStep Cache_FaceProps_Stage3 detected database problems"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; }
          if(bDebugMe)
            { SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 20 = Cache_FaceProps_Stage3\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
              SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
              SM__DRAW_FACES_AND_SURFACES
              sm_GraphicsLoop() ;
            }
          SM_HEALSTEP_END_STATE_DUMP(20, Cache_FaceProps_Stage3);
        }
#endif // SM_DEBUG_CODE

  // Fix SeamCrossing Faces 
  //   - Split Faces at Seam (makes new SplitFace_ChildFaces and SeamEdges)
  SM_HEALSTEP_0ARG(21, Fix_SplitFaceAtSeams, SM_HO_FIX_SPLITFACE_ATSEAMS)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 21))
        { SM_HEALSTEP_BEG_STATE_DUMP(21, Fix_SplitFaceAtSeams);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 21 = Fix_SplitFaceAtSeams\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(21, Fix_SplitFaceAtSeams);
        }
#endif // SM_DEBUG_CODE

  // Fix Loops problems                                                       
  //   - can reorder Loops, change Loop Orientations, and split faces 
  //   - a precondition for SplitFacesAtSeams()

// Keep Fix_BadLoops disabled: classification and repair of broken loops are not reliable.
// Direct helper regressions do not establish that the full stage is safe to enable.
//   SM_HEALSTEP_1ARG(22, Fix_BadLoops, sMarkLock, SM_HO_FIX_BADLOOPS)
//   #ifdef SM_DEBUG_CODE
//         if(bDebugMe && (m_eDoneHealOp >= 22))
//           { SM_HEALSTEP_BEG_STATE_DUMP(22, Fix_BadLoops);
//             SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 22 = Fix_BadLoops\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
//             SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
//             SM__DRAW_FACES_AND_SURFACES
//             sm_GraphicsLoop() ;
//             SM_HEALSTEP_END_STATE_DUMP(22, Fix_BadLoops);
//           }
//   #endif // SM_DEBUG_CODE

  // when asked - CreateUVTrimCurves
  SM_HEALSTEP_1ARG(23, Make_UVTrimCurves, bMakeUVTrimCurves, SM_HO_MAKE_UVTRIMCURVES)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 23))
        { SM_HEALSTEP_BEG_STATE_DUMP(23, Make_UVTrimCurves);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] After Heal Step - 23 = Make_UVTrimCurves\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_HEALSTEP_END_STATE_DUMP(23, Make_UVTrimCurves);
        }
#endif // SM_DEBUG_CODE

  // Find and Set Infinite/Solid Region
  SM_HEALSTEP_0ARG(24, Fix_InfiniteRegion, SM_HO_FIX_INFINITE_REGIONS)
#ifdef SM_DEBUG_CODE
      if(bDebugMe && (m_eDoneHealOp >= 24))
        { SM_HEALSTEP_BEG_STATE_DUMP(24, Fix_InfiniteRegion);
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] DONE with heal steps\n After Heal Step - 24 = Fix_InfiniteRegion\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
          SM__DRAW_FACES_AND_SURFACES
          sm_GraphicsLoop() ;
          SM_SPRINTF(sBuff, _T("\n RecDepth:[%2ld] DONE with heal steps\n"), m_lRecursionCount) ; smos_WriteBuffer(sBuff,NULL,sBuff) ; 
          SM_HEALSTEP_END_STATE_DUMP(24, Fix_InfiniteRegion);
        }
#endif // SM_DEBUG_CODE

// obsolete
// // done running Finished Healer Steps
// // next legacy code for fixing regions and the old AssertHeal mechanism
// 
// #ifdef SM_DEBUG_CODE
// SmBoolean bDumpHealResults = FALSE ;              
//   if(bDumpHealResults)
//     {
//       SM_DUMP(m_pBrep) ;
//       this->Dump() ;
//       SM_DUMP(m_pBrep) ;
//     }
// #endif // SM_DEBUG_CODE
// 
// #ifdef SM_DEBUG_CODE
//   if(bDebugMe)
//     {
//       SmHealData * pAfterFixDegenData = new SmHealData() ;
//       SmTypedDelete<SmHealData *> sObjDelete(pAfterFixDegenData) ;
//       pAfterFixDegenData->HealBrep(m_pBrep, SM_HO_FIX_DEGEN_EDGES, &m_sTgtVertices, &m_sTgtEdges, &m_sTgtFaces, bMakeUVTrimCurves) ;
//       this->Dump(0) ;
//       pAfterFixDegenData->Dump(8) ;
// 
//       SmTArray<SmFace*> sDbgFaces ;
//       m_pBrep->GetFaces(sDbgFaces) ;
// 
//       smgfx_Erase(TRUE) ;
//       smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//       for(ULONG di=0;di<sDbgFaces.GetSize();di++)
//         { smgfx_Erase(TRUE) ;
//           smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//           smgfx_SetLook(4,5, 1,0,0) ; if(m_pBrep) sDbgFaces[di]->DrawUV() ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
//       sm_GraphicsLoop() ;
// 
//       SM__DUMP_BREP__DUMP_HEALDATA__DRAW_BREP
//       SM__DRAW_FACES_AND_SURFACES
//       sm_GraphicsLoop() ;
//     }
// #endif // SM_DEBUG_CODE
// 
// #ifdef SM_DEBUG_CODE
//   if (bDebugMe)
//     {
//       SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;
//     }
// #endif // SM_DEBUG_CODE
// end obsolete

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::HealTgts

/*******************************************************************//**
PURPOSE: HealBrep helper function: Fix BackPointers

NOTES: walks the Brep topology graph checking BackPointers
       When a BackPointer is found that is not properly set
         o sets the back pointer value
         o adds the obj's associated ObjProps ptr to the
             m_sBadObjProps_BackPointers prob array
         o sets the SmObjProps::m_bBadBackPointer = FALSE
       When BackPointers are ok
         o sets the SmObjProps::m_bBadBackPointer = FALSE
***********************************************************************/
SmStatus SmHealData::Fix_BackPointers
 (SmBrep         * pBrep,         // NotUsed: in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // NotUsed: out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
SM_REF3(pBrep, pbNewProbs, eThisHealerOp) ;
//  // init output
//  *pbNewProbs = FALSE ;
//
//  // locals 
//  ULONG ii, jj ;
//  const SmContext         * cpContext  = pBrep->GetContext();
//  SmTArray<SmEdge *>      & rEdges     = m_sTgtEdges ;
//  SmTArray<SmEdgeProps *> & rEdgeProps = m_sTgtEdgeProps ;
//  SmTArray<SmTopology *>    sChanges ;
//  SmBoolean                 bThisChanges = FALSE, bMadeChanges = FALSE ;
//
//  // Check and Fix Brep->Regions list
//  pBrep->FixBackPointers(sChanges, bThisChanges) ;
//  bMadeChanges &= bThisChanges ; 
//
//  // for every Region
//  SmTArray<SmRegion *> sRegions ;
//  pBrep->GetRegions(sRegions) ;
//  for(ii=0;ii<sRegions.GetSize();ii++)
//    {
//      SmRegion * pRegion = sRegions[ii] ;
//
//      // Check and Fix Region->Shells
//      pRegion->FixBackPointers(sChanges, bThisChanges) ;
//      bMadeChanges &= bThisChanges ;
//
//      // for every Shell
//      SmTArray<SmShell *> sShells ;
//      pRegion->GetShells(sShells) ;
//      for(jj=0;jj<sShells.GetSize();jj++)
//        {
//          SmShell * pShell = sShells[ii] ;
//
//          // check and Fix Shell->ListMembers
//          pShell->FixBackPointers(sChanges, bThisChanges) ;
//
//          if(pShell->IsVertexShell())
//            {
//              GetVertex()
//            } // end VertexShell check
//
//          if(pShell->IsWireShell())
//            {
//              GetWireEdges()
//            } // end WireShell check
//
//          if(pShell->IsFaceuseShell())
//            {
//              GetFaceuses() ;
//              // for 
//            } // end FaceuseShell check
//
//          bMadeChanges &= bThis Changes ;
  // all done
  return(SM_SUCCESS) ;

} // end  SmHealData::Fix_BackPointers
  
/*******************************************************************//**
PURPOSE: HealBrep helper function: Build EdgeProps

NOTES: Build BadEdge Object lists
side effects: 1. calls SmEdgeProps::SetProps()
              2. sets SmHealData::members       
                    m_sLoopGap3d_Histogram    
                    m_sEdgeFaceGap3d_Histogram
                    m_sEdgeLen3d_Histogram
           
                    m_sBadEdgeProps_Uncontained
                    m_sBadEdgeProps_SmallZoneTol3d
                    m_sBadEdgeProps_LargeZoneTol3d
                    m_sWarnEdgeProps_LargeGap3d

                    if(ThisEdgeProps->m_bBadDegenEdge)
                      m_sBadEdgeProps_DegenEdges
                    else
                      m_dMaxEdgeLength
                      m_dAvgEdgeLength
                      m_dNonDegen_MinEdgeLength
                      m_lNonDegenEdgeCnt
***********************************************************************/
SmStatus SmHealData::Cache_EdgeProps
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // no work - no Edges or no new Edges
  if(   m_sTgtEdgeProps.GetSize() == 0
     || m_sTgtEdgeProps.GetSize() == m_lOldEdgeCnt)
    { return SM_SUCCESS ; }

  // locals 
  ULONG ii ;
  const SmContext         * cpContext  = pBrep->GetContext();
  SmTArray<SmEdge *>      & rEdges     = m_sTgtEdges ;
  SmTArray<SmEdgeProps *> & rEdgeProps = m_sTgtEdgeProps ;
  
 // reset histograms when processing all ObjProps: skip reset when processing new ObjProps
  if(m_lOldEdgeCnt == 0)
    {
      m_dMaxEdgeLength          = 0.0 ;
      m_dAvgEdgeLength          = 0.0 ;
      m_dNonDegen_MinEdgeLength = SM_BIG_DOUBLE ;
      m_lNonDegenEdgeCnt        = 0 ;

      m_sLoopGap3d_Histogram    .SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ;
      m_sLoopGap3d_Histogram    .SetAll(0) ;
      m_sEdgeFaceGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ;
      m_sEdgeFaceGap3d_Histogram.SetAll(0) ;
      m_sEdgeLen3d_Histogram    .SetSize(SM_HISTOGRAM_EDGELEN_BUCKETCNT) ;
      m_sEdgeLen3d_Histogram    .SetAll(0) ;
    }
  else // when m_lOldEdgeCnt != 0, Adding more cases
    {
      // change AvgEdgeLength back to a total
      if(m_lNonDegenEdgeCnt > 0)
        { m_dAvgEdgeLength *= (double)m_lNonDegenEdgeCnt ; }
    }

  // for every EdgeProps - (m_lOldEdgeCnt != 0 when updating TgtLists with new objs)
  for(ii=m_lOldEdgeCnt;ii<rEdgeProps.GetSize();ii++)
    {
      SmEdgeProps * pThisEdgeProps = rEdgeProps[ii] ;
      pThisEdgeProps->SetContext( cpContext ) ;

      // set EdgeProps_Gaps - (possibly done when pOptEdgeProps != NULL)
      if(FALSE == pThisEdgeProps->IsSet_Gaps())  // and gaps are not yet set
        {
          // sets: pThisEdgeProps:  
          //        IfNeeded { m_pEdge         m_sEdgeInterval         //m_pEdge->m_pEdgeProps   
          //                   m_sZoneTol3d    m_sOrigZoneTol3d        m_dApproxEdgeLength3d    
          //                   m_cpCurve       m_sNaturalInterval      
          //                   m_bBadDegenEdge m_bBadUncontainedEdge }
          //        Always   { m_pMaxGap3d_Edge      m_bBadSmallZoneTol3d  m_bWarnLargeGap3d
          //                   m_pMaxGap3d_Face      m_bBadLargeZoneTol3d  m_bBadUncontainedEdge }
          pThisEdgeProps->SetProps( rEdges[ii],                   // in : tgt Edge
                                    ii,                           // in : Associated Indx for rEdges[ii] in managing SmHealData::m_TgtEdges list
                                   &m_sLoopGap3d_Histogram,       // i/o: accumulating Loop EdgeuseEnd/EdgeuseEnd Gap3d histogram
                                   &m_sEdgeFaceGap3d_Histogram,   // i/o: accumulating Edge/Face Gap3d histogram
                                    ii) ;                         // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
        }  

      // m_bBadCoincidentEdge is not set by the SmEdgeProps::SetProps_Gap() call
      // that happens elsewhere in SmHealData::Cache_CoinEdges().
      
      // m_bBadMissedEdgeXSect is not set by the SmEdgeProps::SetProps_Gap() call
      // that happens elsewhere in SmHealData::Cache_MissedEdgeXSects().

      // m_sBadEdgeProps_Uncontained
      if(pThisEdgeProps->m_bBadUncontainedEdge == TRUE)
        { m_sBadEdgeProps_Uncontained.Add(pThisEdgeProps) ; *pbNewProbs = TRUE ; }

      // m_sBadEdgeProps_SmallZoneTol3d
      if(pThisEdgeProps->m_bBadSmallZoneTol3d == TRUE)
        { m_sBadEdgeProps_SmallZoneTol3d.Add(pThisEdgeProps) ; *pbNewProbs = TRUE ; }

      // m_sBadEdgeProps_LargeZoneTol3d
      if(pThisEdgeProps->m_bBadLargeZoneTol3d == TRUE)
        { m_sBadEdgeProps_LargeZoneTol3d.Add(pThisEdgeProps) ; *pbNewProbs = TRUE ; }

      // m_sWarnEdgeProps_LargeGap3d
      if(pThisEdgeProps->m_bWarnLargeGap3d == TRUE)
        { m_sWarnEdgeProps_LargeGap3d.Add(pThisEdgeProps) ; }

      // EdgeLength Histogram
      m_sEdgeLen3d_Histogram[SM_HISTOGRAM_EDGELEN_INDEX(pThisEdgeProps->m_dApproxEdgeLength3d)]++ ;

      // BadDegen EdgeList
      if(pThisEdgeProps->m_bBadDegenEdge == TRUE)
        { m_sBadEdgeProps_DegenEdges.Add(pThisEdgeProps) ; *pbNewProbs = TRUE ; 
        }
      else // NonDegenerate Edge branch
        {
          // accumulate m_dMaxEdgeLength statistics
          if(pThisEdgeProps->m_dApproxEdgeLength3d > m_dMaxEdgeLength)
            { m_dMaxEdgeLength = pThisEdgeProps->m_dApproxEdgeLength3d ; }

          // accumulate m_dNonDegen_MinEdgeLength statistics
          if(pThisEdgeProps->m_dApproxEdgeLength3d < m_dNonDegen_MinEdgeLength)
            { m_dNonDegen_MinEdgeLength = pThisEdgeProps->m_dApproxEdgeLength3d ; }

          // accumulatem_dAvgEdgeLength 
          m_dAvgEdgeLength += pThisEdgeProps->m_dApproxEdgeLength3d ;
          m_lNonDegenEdgeCnt++ ;
        }
    } // end iter EdgeProps building BadEdgeProps ObjList

  // finish getting AvgEdgeLength
  if(m_lNonDegenEdgeCnt > 0) 
    { m_dAvgEdgeLength /= (double)m_lNonDegenEdgeCnt ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_EdgeProps

/*******************************************************************//**
PURPOSE: Build VertexProps 

NOTES: 1. calls SmVertexProps::SetProps()
       2. sets SmHealData::members       
             m_sVertEdgeGap3d_Histogram
             m_sVertFaceGap3d_Histogram

             m_sBadVertexProps_SmallZoneTol3d
             m_sBadVertexProps_LargeZoneTol3d
             m_sWarnVertexProps_LargeGap3d
***********************************************************************/
SmStatus SmHealData::Cache_VertexProps
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // no work - no vertices or no new vertices
  if(   m_sTgtVertexProps.GetSize() == 0
     || m_sTgtVertexProps.GetSize() == m_lOldVertexCnt)
    { return SM_SUCCESS ; }

  // locals
  ULONG ii ;
  SmTArray<SmVertex *>      & rVertices    = m_sTgtVertices ;
  SmTArray<SmVertexProps *> & rVertexProps = m_sTgtVertexProps ;
  const SmContext           * cpContext    = pBrep->GetContext() ;

  // reset histograms when processing all ObjProps: skip reset when processing new ObjProps
  if(m_lOldVertexCnt == 0)
    {
      m_sVertEdgeGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ;
      m_sVertEdgeGap3d_Histogram.SetAll(0) ;
      m_sVertFaceGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ;
      m_sVertFaceGap3d_Histogram.SetAll(0) ;  
    }

  // begin Scope for Setup_1: Gather VertexProps 
  //   VertexProps[ii]: m_pVertex            m_sZoneTol3d
  //                    m_pMaxGap3d_Edge     m_pMaxGap3d_Face
  //                    m_bBadSmallZoneTol3d
  //                    m_bBadLargeZoneTol3d

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
if(bDebugMe)
  {
    SM_DUMP_AND_ASSERT_VALID(pBrep) ;

    smgfx_Erase(TRUE) ;
    smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // for every VertexProp obj - Get VertexProperties - (m_lOldVertexCnt != 0 when updating TgtLists with new objs)
  for(ii=m_lOldVertexCnt;ii<rVertexProps.GetSize();ii++)
    {
      // set m_pVertex and compute:[XSectTol3d, bSheet, bClosed, lPoles, lFlatCorners]
      SmVertexProps * pThisVertexProps = rVertexProps[ii] ;
      pThisVertexProps->SetContext(cpContext) ; 
        
      // set VertexProps - (possibly done when pOptVertexProps != NULL)
      if(FALSE == pThisVertexProps->IsSet())
        {
          // sets: pThisVertexProps: m_pVertex         m_sZoneTol3d
          //                         m_pMaxGap3d_Edge  m_bBadSmallZoneTol3d  m_bWarnLargeGap3d
          //                         m_pMaxGap3d_Face  m_bBadLargeZoneTol3d
          pThisVertexProps->SetProps( rVertices[ii],                   // in : tgt Vertex
                                      ii,                              // in : Associated Indx for rVertices[ii] in managing SmHealData::m_TgtVerticess list
                                     &m_sVertEdgeGap3d_Histogram,      // i/o: accumulating Vertex/Edge Gap3d histogram
                                     &m_sVertFaceGap3d_Histogram,      // i/o: accumulating Vertex/Face Gap3d histogram
                                      ii) ;                            // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
        }  

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          pThisVertexProps->Dump(ii) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw() ; sm_GraphicsLoop() ;
          pThisVertexProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sHighlightColor         def:[ 1, 0, 0]
                                                             // sBadSmallZoneTol3dColor def:[ 0, 1, 1]
                                                             // sBadLargeZoneTol3dColor def:[ 1, 0, 1]
                                                             // sWarnLargeGap3dColor    def:[ 1,.5,.1]
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // m_bBadSmallZoneTol3d                               
      if(pThisVertexProps->m_bBadSmallZoneTol3d == TRUE)    
        { m_sBadVertexProps_SmallZoneTol3d.Add(pThisVertexProps) ; *pbNewProbs = TRUE ; }

      // m_bBadLargeZoneTol3d
      if(pThisVertexProps->m_bBadLargeZoneTol3d == TRUE)
        { m_sBadVertexProps_LargeZoneTol3d.Add(pThisVertexProps) ; *pbNewProbs = TRUE ; }

      // m_bWarnLargeGap3d
      if(pThisVertexProps->m_bWarnLargeGap3d == TRUE)
        { m_sWarnVertexProps_LargeGap3d.Add(pThisVertexProps) ; }

    } // end iter every Vertex - computing and saving face classification properties

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_VertexProps

/*******************************************************************//**
PURPOSE: HealBrep helper function: 
         Build EdgeProps before Split SeamCrossingEdges

NOTES: 1. calls SmFaceProps::SetProps_Gaps()
       2. sets SmHealData::members  
             m_sBadFaceProps_SmallZoneTol3d 
             m_sBadFaceProps_LargeZoneTol3d 
***********************************************************************/
SmStatus SmHealData::Cache_FaceProps_Gaps
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // no work - no Faces or no new Faces
  if(   m_sTgtFaceProps.GetSize() == 0
     || m_sTgtFaceProps.GetSize() == m_lOldFaceCnt)
    { return SM_SUCCESS ; }

  // locals
  ULONG ii ;
  SmTArray<SmFace *>      & rFaces     = m_sTgtFaces ;
  SmTArray<SmFaceProps *> & rFaceProps = m_sTgtFaceProps ;
  const SmContext         * cpContext  = pBrep->GetContext() ;

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE ;
if(bDebugMe)
  {
    SM_DUMP_AND_ASSERT_VALID(pBrep) ;

    smgfx_Erase(TRUE) ;
    smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // for every FaceProp obj - Get FaceProperties - (m_lOldFaceCnt != 0 when updating TgtLists with new objs)
  for(ii=m_lOldFaceCnt;ii<rFaceProps.GetSize();ii++)
    {
      // set m_pFace and compute:[XSectTol3d, bSheet, bClosed, lPoles, lFlatCorners]
      SmFaceProps * pThisFaceProps = rFaceProps[ii] ;
      pThisFaceProps->SetContext(cpContext) ; 
        
      // set FaceProps_Stage1 - (possibly done when pOptFaceProps != NULL)
      if(FALSE == pThisFaceProps->HasProps(SM_PROPSTAGE_GAPS, pThisFaceProps->m_pFace))
        {
          // sets: pThisFaceProps: 
          //    m_pFace               m_sZoneTol3d          m_sOrigZoneTol3d 
          //    m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d   
          pThisFaceProps->SetProps(SM_PROPSTAGE_GAPS, rFaces[ii], ii, m_eDoneHealOp, ii) ; 
        }  

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          pThisFaceProps->Dump(ii) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                           // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                           // sMissingSeamColor   - def:[ 1, 0, 0]
                                                           // sMissingPoleColor   - def:[ 0, 0, 1]
                                                           // sBadOrientLoopColor - def:[.3,.3,.3]
                                                           // sNoArea_LoopColor   - def:[ 0, 0, 0]
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // count the kinds of faces that force healing checks later on

      // m_sBadFaceProps_SmallZoneTol3d
      if(pThisFaceProps->m_bBadSmallZoneTol3d == TRUE)
        { m_sBadFaceProps_SmallZoneTol3d.Add(pThisFaceProps) ; *pbNewProbs = TRUE ; }

      // m_sBadFaceProps_LargeZoneTol3d
      if(pThisFaceProps->m_bBadLargeZoneTol3d == TRUE)
        { m_sBadFaceProps_LargeZoneTol3d.Add(pThisFaceProps) ; *pbNewProbs = TRUE ; }

    } // end iter every Face - computing and saving face classification properties

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(di=0;di<m_sFaceProps_Sheets.GetSize();di++)
        { SmFaceProps * pThisFaceProps = rFaceProps[di] ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                           // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                           // sMissingSeamColor   - def:[ 1, 0, 0]
                                                           // sMissingPoleColor   - def:[ 0, 0, 1]
                                                           // sBadOrientLoopColor - def:[.3,.3,.3]
                                                           // sNoArea_LoopColor   - def:[ 0, 0, 0]
        }
      smgfx_SetLook(4,5, .2,.4,.6) ; for(di=0;di<m_sBadEdgeProps_Uncontained.GetSize();di++)
                                      { if(m_sBadEdgeProps_Uncontained[di]) m_sBadEdgeProps_Uncontained[di]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ; 
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_FaceProps_Gaps

/*******************************************************************//**
PURPOSE: HealBrep helper function: 
         Set Object Tolerance based on Object Connection Gap Sizes

NOTES: Also picks and saves a default Zone tol value for this part
       sets SmBrep::m_dThisModelSizeEstimate
            SmBrep::m_sThisZoneTol3d        
FIXES: problems in SmHealData::m_sBadVertexProps_SmallZoneTol3d
                   SmHealData::m_sBadVertexProps_LargeZoneTol3d

                   SmHealData::m_sBadEdgeProps_SmallZoneTol3d
                   SmHealData::m_sBadEdgeProps_LargeZoneTol3d

                   SmHealData::m_sBadFaceProps_LargeZoneTol3d
                   SmHealData::m_sBadFaceProps_LargeZoneTol3d
***********************************************************************/
SmStatus SmHealData::Fix_TolSizes
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
#endif // SM_DEBUG_CODE

  // locals
  SmTArray<SmEdge *>      & rEdges = m_sTgtEdges ;
  SmTArray<SmVertexProps *> sBadTolVertexProps ;  // union of Bad Small & Large ZoneTol3d props                                       
  SmTArray<SmEdgeProps *>   sBadTolEdgeProps ;    // union of Bad Small & Large ZoneTol3d props
  SmTArray<SmFaceProps *>   sBadTolFaceProps ;    // union of Bad Small & Large ZoneTol3d props

  // Single list of problem Vertex objs
  sBadTolVertexProps.Append(m_sBadVertexProps_SmallZoneTol3d.m_sProbArray) ;
  sBadTolVertexProps.Append(m_sBadVertexProps_LargeZoneTol3d.m_sProbArray) ;

  // Single list of problem Edge objs
  sBadTolEdgeProps.Append(m_sBadEdgeProps_SmallZoneTol3d.m_sProbArray) ;
  sBadTolEdgeProps.Append(m_sBadEdgeProps_LargeZoneTol3d.m_sProbArray) ;

  // Single list of problem Face objs
  sBadTolFaceProps.Append(m_sBadFaceProps_SmallZoneTol3d.m_sProbArray) ;
  sBadTolFaceProps.Append(m_sBadFaceProps_LargeZoneTol3d.m_sProbArray) ;

  // Set Context default ZoneTol3d based on Part size
  double dThisModelSize =  rEdges.GetSize() != 0
                         ? m_dMaxEdgeLength
                         : 50 ;

  // ModelSize picks Working tolerance values, There's leeway in making this assignment
  dThisModelSize =   (dThisModelSize <= 500   && dThisModelSize > .5)     ?     50
                   : (dThisModelSize <= 50000 && dThisModelSize > 500)    ?   5000
                   : (dThisModelSize <= .5    && dThisModelSize > .005)   ?      0.05
                   : (dThisModelSize <= 5e6   && dThisModelSize > 50000)  ? 500000
                   : (dThisModelSize <= .005  && dThisModelSize > .00005) ?      0.0005
                   : dThisModelSize ; 
  double dBrepModelSize    = SmTol::GetModelSizeEstimate(*pBrep) ; 
  double dContextModelSize = SmTol::GetModelSizeEstimate((SmContext &)*pBrep->GetContext()) ; 

  SmZoneTol3d sThisEstZoneTol3d    = SmTol::SizeEstimateToZoneTol3d(dThisModelSize) ;
  SmZoneTol3d sBrepEstZoneTol3d    = SmTol::SizeEstimateToZoneTol3d(dBrepModelSize) ;
  SmZoneTol3d sContextEstZoneTol3d = SmTol::SizeEstimateToZoneTol3d(dContextModelSize) ;

  // When The BrepTolerance needs to be set - set it based on ModelSizeEstimate
  if(sThisEstZoneTol3d != sBrepEstZoneTol3d)
    {
      SM_NEWTOL_LINE if(sThisEstZoneTol3d == sContextEstZoneTol3d)
      SM_NEWTOL_LINE   { SmTol::SetModelSizeEstimate(*pBrep, SM_USE_DEFAULT) ; }
      SM_NEWTOL_LINE else
      SM_NEWTOL_LINE   { SmTol::SetModelSizeEstimate(*pBrep, dThisModelSize) ; }
    }
  SM_OLDTOL_LINE pBrep->SetTolerance(sThisEstZoneTol3d) ; 

  // check the defZoneTol3d values - see if they have changed as desired
  SM_ASSERT_MSG(   sThisEstZoneTol3d == SmTol::GetZoneTol3d(pBrep),
                _T("SmHealData::Fix_TolSizes: Setting Context DefZoneTol3d is not working - debug needed")) ;

// for every problem Face - Calc and Store a new ZoneTol3d
for(ULONG ii=0;ii<sBadTolFaceProps.GetSize();ii++)
  {
    SmFaceProps * pBadTolFaceProps = sBadTolFaceProps[ii] ;
    SmFace      * pBadTolFace      = (SmFace *)pBadTolFaceProps->m_pFace ; 

    // skip faces already fixed
    if(   pBadTolFaceProps->m_bBadSmallZoneTol3d == FALSE
       && pBadTolFaceProps->m_bBadLargeZoneTol3d == FALSE)
      { continue ; }

    // update the tolerance
    SmZoneTol3d sZoneTol3d = SmTol::CalcFaceZoneTol3d(pBrep) ;

    if(pBadTolFaceProps->m_sOrigZoneTol3d == SM_UNDEF_DOUBLE)
      { pBadTolFaceProps->m_sOrigZoneTol3d = SmTol::GetZoneTol3d(pBadTolFace) ; }
    pBadTolFace->SetTolerance(sZoneTol3d, FALSE) ;  // FALSE = allow shrinking tolerance
    pBadTolFaceProps->m_sZoneTol3d         = sZoneTol3d ;

    // remember the tol has been fixed on this Face
    pBadTolFaceProps->m_bBadSmallZoneTol3d = FALSE ;
    pBadTolFaceProps->m_bBadLargeZoneTol3d = FALSE ;

    // check tol is changed as desired
    SM_ASSERT_MSG(   sZoneTol3d == pBadTolFace->GetTolerance()
                  && sZoneTol3d == pBadTolFaceProps->m_sZoneTol3d,
                  _T("SmHealData::Fix_TolSizes: Setting Face Context DefZoneTol3d is not working - debug needed")) ;

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
      {
        if(ii<3)
          { pBadTolFaceProps->Dump(ii) ; }

        if(ii==0) smgfx_Erase(TRUE) ;
        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep && ii==0) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(2,3, 1,0,0) ; if(pBadTolFace) pBadTolFace->DrawUV() ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

  } // end iter every BadTolFace setting new ZoneTol3d values

// Edge: for every problem edge - Calc and Store a new ZoneTol3d
for(ULONG ii=0;ii<sBadTolEdgeProps.GetSize();ii++)
  {
    SmEdgeProps        * pBadTolEdgeProps = sBadTolEdgeProps[ii] ;
    SmEdge             * pBadTolEdge      = (SmEdge *)pBadTolEdgeProps->m_pEdge ; 
    SmZoneTol3d          sMaxFaceTol( 0 );
    SmTArray<SmFace *>   sFaces;

    // skip edges already fixed
    if(   pBadTolEdgeProps->m_bBadSmallZoneTol3d == FALSE
       && pBadTolEdgeProps->m_bBadLargeZoneTol3d == FALSE)
      { continue ; }

    // Get the largest FaceTol connected to this Edge
    pBadTolEdge->GetFaces( sFaces );
    ULONG lNumFaces = sFaces.GetSize();
    for ( ULONG jj = 0; jj < lNumFaces; ++jj )
      {
        SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d( sFaces[jj] );
        sMaxFaceTol = ( sFaceZoneTol3d > sMaxFaceTol ) ? sFaceZoneTol3d : sMaxFaceTol;
      }

    // update the tolerance
    double dMaxEdgeEdgeGap3d_Length = pBadTolEdgeProps->m_pMaxGap3d_EdgeEdge ? pBadTolEdgeProps->m_pMaxGap3d_EdgeEdge->GetLength() : 0.0 ;
    double dMaxEdgeFaceGap3d_Length = pBadTolEdgeProps->m_pMaxGap3d_EdgeFace ? pBadTolEdgeProps->m_pMaxGap3d_EdgeFace->GetLength() : 0.0 ;
    SmZoneTol3d sZoneTol3d = SmTol::CalcEdgeZoneTol3d( dMaxEdgeEdgeGap3d_Length,
                                                       dMaxEdgeFaceGap3d_Length,
                                                       pBrep,
                                                      &sMaxFaceTol) ;
    if(pBadTolEdgeProps->m_sOrigZoneTol3d == SM_UNDEF_DOUBLE)
      { pBadTolEdgeProps->m_sOrigZoneTol3d = SmTol::GetZoneTol3d(pBadTolEdge) ; }
    pBadTolEdge->SetTolerance(sZoneTol3d, FALSE, FALSE) ;  // FALSE = allow shrinking tolerance
                                                           // FALSE = don't chg EdgeVertex tolerances to be as big as EdgeTol
    pBadTolEdgeProps->m_sZoneTol3d         = sZoneTol3d ;
    
    // remember the tol has been fixed on this Edge
    pBadTolEdgeProps->m_bBadSmallZoneTol3d = FALSE ;
    pBadTolEdgeProps->m_bBadLargeZoneTol3d = FALSE ;

    // check tol is changed as desired
    SM_ASSERT_MSG(   sZoneTol3d == pBadTolEdge->GetTolerance()
                  && sZoneTol3d == pBadTolEdgeProps->m_sZoneTol3d,
                  _T("SmHealData::Fix_TolSizes: Setting Edge Context DefZoneTol3d is not working - debug needed")) ;

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
      {
        if(ii<3)
          { pBadTolEdgeProps->Dump(ii) ; }

        if(ii==0) smgfx_Erase(TRUE) ;
        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep && ii==0) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(4,5, 1,0,0) ; if(pBadTolEdge) pBadTolEdge->Draw() ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE
  } // end Edge: iter every BadTol setting new ZoneTol3d values

// for every problem Vertex - Calc and Store a new ZoneTol3d
for(ULONG ii=0;ii<sBadTolVertexProps.GetSize();ii++)
  {
    SmVertexProps     * pBadTolVertexProps = sBadTolVertexProps[ii] ;
    SmVertex          * pBadTolVertex      = (SmVertex *)pBadTolVertexProps->m_pVertex ; 
    SmZoneTol3d         sMaxEdgeTol3d( 0 );
    SmTArray<SmEdge*>   sEdges;

    // skip vertices already fixed
    if(   pBadTolVertexProps->m_bBadSmallZoneTol3d == FALSE
       && pBadTolVertexProps->m_bBadLargeZoneTol3d == FALSE)
      { continue ; }

    // Get the largest EdgeTol connected to this vertex
    pBadTolVertex->GetEdges( sEdges );
    ULONG lNumEdges = sEdges.GetSize();
    for ( ULONG jj = 0; jj < lNumEdges; ++jj )
    {
        SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d( sEdges[jj] );
        sMaxEdgeTol3d = ( sEdgeZoneTol3d > sMaxEdgeTol3d ) ? sEdgeZoneTol3d : sMaxEdgeTol3d;
    }

    // update the tolerance
    double dMaxVertexEdgeGap3d_Length = pBadTolVertexProps->m_pMaxGap3d_VtxEdge ? pBadTolVertexProps->m_pMaxGap3d_VtxEdge->GetLength() : 0.0 ;
    double dMaxVertexFaceGap3d_Length = pBadTolVertexProps->m_pMaxGap3d_VtxFace ? pBadTolVertexProps->m_pMaxGap3d_VtxFace->GetLength() : 0.0 ;
    SmZoneTol3d sZoneTol3d = SmTol::CalcVertexZoneTol3d( dMaxVertexEdgeGap3d_Length,
                                                         dMaxVertexFaceGap3d_Length,
                                                         pBrep,
                                                        &sMaxEdgeTol3d );
    if(pBadTolVertexProps->m_sOrigZoneTol3d == SM_UNDEF_DOUBLE)
      { pBadTolVertexProps->m_sOrigZoneTol3d = SmTol::GetZoneTol3d(pBadTolVertex) ; }
    pBadTolVertex->SetTolerance(sZoneTol3d, FALSE) ;  // FALSE = allow shrinking tolerances
    pBadTolVertexProps->m_sZoneTol3d         = sZoneTol3d ;
    
    // remember the tol has been fixed on this Vertex
    pBadTolVertexProps->m_bBadSmallZoneTol3d = FALSE ;
    pBadTolVertexProps->m_bBadLargeZoneTol3d = FALSE ;

    // check tol is changed as desired
    SM_ASSERT_MSG(   sZoneTol3d == pBadTolVertex->GetTolerance()
                  && sZoneTol3d == pBadTolVertexProps->m_sZoneTol3d,
                  _T("SmHealData::Fix_ObjTolSizes: Setting Vertex Context DefZoneTol3d is not working - debug needed")) ;

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
      {
        if(ii<3)
          { pBadTolVertexProps->Dump(ii) ; }

        if(ii==0) smgfx_Erase(TRUE) ;
        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep && ii==0) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(4,5, 1,0,0) ; if(pBadTolVertex) pBadTolVertex->Draw() ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

  } // end iter every BadTolVertex setting new ZoneTol3d values

#ifdef SM_DEBUG_CODE
if(bDebugMe)
  {
    SM_DUMP_AND_ASSERT_VALID(pBrep) ;

    smgfx_Erase(TRUE) ;
    smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_TolSizes

/*******************************************************************//**
PURPOSE: HealBrep helper function: Find Coincident Vertices

NOTES: 1. places vertices into spatial buckets - check dists between vertices in common buckets
         vertices on bucket boundaries get but into both abutting buckets forcing the
         algorithm to check for redundant coincident vertex pair findings.

       2. sets SmHealData::members  
              m_sVertVertGap3d_Histogram
              m_sBadVertexProps_CoinVertices
***********************************************************************/
SmStatus SmHealData::Cache_CoinVertices 
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugOk = TRUE ;
// ULONG GWC_REMOVE_NEXT_STATIC_VARIABLE_BEFORE_RELEASE ;
// static ULONG lDebugCnt = 0 ;
// lDebugCnt++ ;
  if(bDebugMe)
    {
      this->Dump(0) ;
      if(m_pVerticesInVoxels) bDebugOk &= m_pVerticesInVoxels->AssertValid() ;
      if(m_pVerticesInVoxels) m_pVerticesInVoxels->Dump(0, TRUE) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(m_pVerticesInVoxels) m_pVerticesInVoxels->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // no work - no vertices or no new vertices
  if(   m_sTgtVertexProps.GetSize() == 0
     || m_sTgtVertexProps.GetSize() == m_lOldVertexCnt)
    { return SM_SUCCESS ; }

  // init output
  *pbNewProbs = FALSE ;

  // locals 
  ULONG ii ;
  SmVector3d             sInwardNorm ;      // used for within tol of neighbor voxel calculations
  SmTArray<ULONG>        sUpdatedBuckets ;  // only used when m_lOldVertexCnt > 0, stores the buckets loaded with new vertices - only they need to be checked for coin

  // SmZoneTol3d            sSysZoneTol3d = SmTol::GetZoneTol3d() ;
  // SmTArray<SmVertexProps *>  sTgtVertexProps ;
  // SmTArray<SmPoint3d>        sTgtPoints3d ;
  // SmTArray<SmZoneTol3d>      sTgtZoneTols3d ;

  // check state
  SM_ASSERT_BREAK_MSG( (m_pVerticesInVoxels != NULL) && (m_pVerticesInVoxels->AssertValid()), _T("SmHealData::Cache_CoinVertices: m_pVerticesInVoxels is NULL or not Valid")) ; 

 // // gather this set of tgt vertices and positions (Positions used to sort Tgts into Voxels)
 // for(ii=m_lOldVertexCnt;ii<m_sTgtVertexProps.GetSize();ii++)
 //   {
 //     // // gather SmPointItem data to be added later to m_pVerticesInVoxels 
 //     //                      sTgtVertexProps.Add(m_sTgtVertexProps[ii]) ;
 //     // ULONG lPoint3dIndx = sTgtPoints3d.   Add(m_sTgtVertexProps[ii]->m_pVertex->GetPoint()) ;
 //     //                      sTgtZoneTols3d .Add(SmTol::GetZoneTol3d(m_sTgtVertexProps[ii]->m_pVertex)) ;
 //
 //     // build bounding box for Point set
 //     sTgtBBox.AddPoint3d(m_sTgtVertexProps[ii]->m_pVertex->GetPoint()) ;
 //   }
  
  // On the first call or when asked to recheck all the vertices - reset the Gap Histogram, VertexBoundingBox, and m_pVerticesInVoxels data
  if(m_lOldVertexCnt == 0) // when checking all vertices - true on first call
    {
      // always reset histograms when processing all vertices 
      m_sVertVertGap3d_Histogram.SetSize(SM_HISTOGRAM_GAPSIZE_BUCKETCNT) ;
      m_sVertVertGap3d_Histogram.SetAll(0) ;

      // find TgtVertex BoundingBox
      m_sVertexBBox.Init() ;

      // for every TgtVertex - grow BoundingBox
      for(ii=m_lOldVertexCnt;ii<m_sTgtVertexProps.GetSize();ii++)
        {
          // build bounding box for Point set
          m_sVertexBBox.AddPoint3d(m_sTgtVertexProps[ii]->m_pVertex->GetPoint()) ;
        }

      // set the voxel model size
      // Reserve 7 * vertexCount (each vertex may hit up to 7 voxels) so m_sPointItems never reallocs
      // mid-add and dangles the aliasing bucket pointers (UAF in FindCoincidentPairs).
      m_pVerticesInVoxels->SetSize(m_lXCnt,                            // in : Number of Voxels in X dir 
                                   m_lYCnt,                            // in : Number of Voxels in Y dir 
                                   m_lZCnt,                            // in : Number of Voxels in Z dir 
                                   &m_sVertexBBox,                     // in : Voxel Extent 
                                   m_sTgtVertexProps.GetSize() * 7) ;  // in : estimated number of items to be placed in the buckets 

      // note: 1. Do not change the size of the m_sVertexBBox after it was initially created
      //          If that's done it changes the meaning of the voxel sizes which means that the
      //          vertices assigned to voxels before the box changes size might now be in the wrong voxel
      //          and future vertex additions might not get into the same voxel as their coincident partners.
      //       2. adding vertices beyond the voxel model size is not a problem.  Those get snapped back to
      //          the nearest voxel causing that voxel to have more vertices in it than it would
      //          normally expect.  That's not a problem it just means that the check for coin vertices
      //          in that one voxel won't be as efficient as it could be

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          this->Dump(1) ;
          SM_DUMP_AND_ASSERT_VALID(pBrep) ;
          if(m_pVerticesInVoxels) bDebugOk &= m_pVerticesInVoxels->AssertValid() ;
          if(m_pVerticesInVoxels) m_pVerticesInVoxels->Dump(1, TRUE) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 1,0,0) ; if(m_pVerticesInVoxels) m_pVerticesInVoxels->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#else // no SM_DEBUG_CODE
      SM_REF1(pBrep);
#endif // no SM_DEBUG_CODE

    } // end m_lOldVertexCnt == 0 check

  // When adding more points to the voxels - do the extra work to keep track of the updated buckets
  SmTArray<ULONG> * pUpdatedBuckets = m_lOldVertexCnt == 0 ? NULL : &sUpdatedBuckets ;
  ULONG           * pOptLimitIndx   = m_lOldVertexCnt == 0 ? NULL : &m_lOldVertexCnt ;
  
  // add the incremental tgt vertices and positions into the voxel model 
  //    ii range: [0,sTgtVertexSize-1] where sTgtVertexSize = m_sTgtVertexPropsSize - m_lOldVertexCnt
  for(ii=m_lOldVertexCnt;ii<m_sTgtVertexProps.GetSize();ii++)
  // for(ii=0;ii<sTgtVertexProps.GetSize();ii++)
    {
      // locals - SmPointItem data to be added to m_pVerticesInVoxels 
      SmPoint3d   sPoint3d  (m_sTgtVertexProps[ii]->m_pVertex->GetPoint()) ;
      SmZoneTol3d sZoneTol3d(SmTol::GetZoneTol3d(m_sTgtVertexProps[ii]->m_pVertex)) ;

      // Create and Add SmPointItem obj to m_pVerticesInVoxels
      ULONG lVoxelIndx = m_pVerticesInVoxels->Add(m_sTgtVertexProps[ii], // in : TgtItem being added to the hash table
                                                  sPoint3d,              // in : TgtItem's Space3d location
                                                  sZoneTol3d,            // in : TgtItem's ZoneTol3d about the TgtItem's rPoint3d
                                                  pUpdatedBuckets,       // i/o: NotNULL = accumulate list of Bucket Indices to which Items have been added
                                                                         //      NULL to ignore, default:[NULL]
                                                                         //      note: building the pUpdatedBuckets list uses a slow AddUnique() call,
                                                                         //            only use this when adding a few items to already existing large list.
                                                                         //            Don't use when building a large list from scratch 
                                                                         //             - the m_sHashTable.GetUsedBucketCount() and GetUsedBucket(ii) will
                                                                         //               give you the used bucket list in that case.
                                                  pBrep) ;               // in : for debug draw only, NULL to ignore, default:[NULL]
      SM_REF1(lVoxelIndx) ;
      SM_ASSERT_BREAK( lVoxelIndx != SM_UNDEF_ULONG ) ;
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      if(m_pVerticesInVoxels) bDebugOk &= m_pVerticesInVoxels->AssertValid() ;
      if(m_pVerticesInVoxels) m_pVerticesInVoxels->Dump(2, TRUE) ;
  
      smgfx_Erase(TRUE) ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(m_pVerticesInVoxels) m_pVerticesInVoxels->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
  // next - check for coincidence
  SmTArray<SmPointItem<SmVertexProps *> *> sCoin1 ;
  SmTArray<SmPointItem<SmVertexProps *> *> sCoin2 ;

  // Check m_pVerticesInVoxels for coincident pairs (when adding more points - only in UpdatedBuckets) 
  m_pVerticesInVoxels->FindCoincidentPairs(sCoin1,                        // out: array of PointItem1s for each CoincidentPair:[Pair1_PointItem1, Pair2_PointItem1,.. , PairN_PointItem1]
                                           sCoin2,                        // out: array of PointItem2s for each CoincidentPair:[Pair1_PointItem2, Pair2_PointItem2,.. , PairN_PointItem2]
                                                                          //        note: while a PointItem might show up many times in each list,
                                                                          //              Each combined [PointItem1, PointItem2] coin pair is unique
                                           pOptLimitIndx,                 // in : NotNULL, Chk PointItems with Indxs >= *pOptLimitIndx against all PointItems for coin,
                                                                          //      NULL,    chk all PointItems against all PointItems for coin,
                                                                          //      default:[NULL]
                                           pUpdatedBuckets,               // in : NotNULL = List of buckets to be checked for coincident pairs
                                                                          //      NULL    = Check all buckets, default:[NULL]
                                                                          //      note: When adding a few Items to the voxels and then checking for coincidence
                                                                          //            set pOptStartIndx = SizeOf m_sPointItems before adding new Items
                                                                          //            let pOptBuckets   = Add's pOptUpdatedBuckets argument
                                           &m_sVertVertGap3d_Histogram) ; // out: Optional Histogram of PtObj/PtObj gap sizes

  // for every coincident pair - add entry pair to SmHealData::m_sBadVertexProps_CoinVertices problem array
  for(ii=0;ii<sCoin1.GetSize();ii++)
    {
      // defensive: skip pairs referencing stale/freed vertex props. An earlier merge in this heal pass can
      // free a SmVertexProps and remove it from m_sTgtVertexProps, but the voxel model can still surface a
      // coincident pair referencing it (a removed PointItem left behind in a bucket chain keeps a dangling
      // m_sTgtItem). Adding a freed props here crashes later when SmProbArray::Add dereferences it via
      // pNewElement->*m_plObjProbFlag. FindElement only compares pointer identity, so it is safe on a stale
      // pointer value. m_sTgtVertexProps holds exactly the live props, so a miss means the props is gone.
      { ULONG lLiveIdx ;
        if(   !m_sTgtVertexProps.FindElement(sCoin1[ii]->m_sTgtItem, lLiveIdx)
           || !m_sTgtVertexProps.FindElement(sCoin2[ii]->m_sTgtItem, lLiveIdx))
          { continue ; }
      }

      // add coincident pair Props to prob array and set linked SmVertexProps::&m_bBadCoincidentVertex flag to TRUE.
      m_sBadVertexProps_CoinVertices.Add(sCoin1[ii]->m_sTgtItem) ; 
      m_sBadVertexProps_CoinVertices.Add(sCoin2[ii]->m_sTgtItem) ;

      m_sBadVertexProps_CoinVertices.m_lOrigCoinPairCnt++ ;

      // set output
      *pbNewProbs = TRUE ;

    } // end iter every found coincident pair

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      this->Dump(3) ;
      if(m_pVerticesInVoxels) bDebugOk &= m_pVerticesInVoxels->AssertValid() ;
      if(m_pVerticesInVoxels) m_pVerticesInVoxels->Dump(3, TRUE) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_CoinVertices

/*******************************************************************//**
PURPOSE: HealBrep helper function: Merge Coincident Vertices

NOTES: Find and remove coincident Vertices first because
       that'll help keeping the cost of finding degen edges and faces down
FIXES: problems in SmHealData::
                    m_sBadVertexProps_CoinVertices
***********************************************************************/
SmStatus SmHealData::Fix_CoinVertices
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;

  // locals
  // Use the current array size (2 entries/pair), not m_lOrigCoinPairCnt: that accumulates across heal
  // iterations and drifts past the array, overflowing the 2*index addressing below.
  ULONG lPairCount = m_sBadVertexProps_CoinVertices.m_sProbArray.GetSize() / 2 ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // no work
  if(lPairCount == 0)
    { return(SM_SUCCESS) ; }

  // Iterate a stable snapshot: the per-merge UpdateTargetLists() below removes props from the live
  // m_sProbArray, which would otherwise shrink it and invalidate the 2*index addressing.
  SmTArray<SmVertexProps *> sProbArray = m_sBadVertexProps_CoinVertices.m_sProbArray ;

  // enable editing
  SmTemporaryChangeValue<SmBoolean> sSaveEditing(pBrep->m_bEditingEnabled, TRUE) ;

  // for every coincident vertex pair - merge one at a time with per-pair topology tracking.
  // SmObjsInVoxels::Remove has a pre-existing cleanup bug (uses loop counter instead of
  // found-index to clear PointItems). Batching multiple vertex removals into one
  // UpdateTargetLists call corrupts the hash table. Per-pair tracking avoids this.
  for(ULONG ii=0; ii<lPairCount; ii++)
    {
      SmVertexProps * pSurvivorProps = sProbArray[2*ii] ;
      SmVertexProps * pDeletedProps  = sProbArray[2*ii + 1] ;

      // skip self-pairs (both sides merged to same survivor by earlier fixup)
      if(pSurvivorProps == pDeletedProps)
        { continue ; }

      // skip pairs where either props was already removed by a prior UpdateTargetLists call
      { ULONG lIdx ;
        if(   !m_sTgtVertexProps.FindElement(pSurvivorProps, lIdx)
           || !m_sTgtVertexProps.FindElement(pDeletedProps,  lIdx))
          { continue ; }
      }

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          TCHAR sBuff[SM_TBLOCK_SIZE] ;
          smos_sprintf(sBuff, _T("\nFix_CoinVertices: pair[%ld] survivor=0x%p deleted=0x%p"),
              ii, (void*)pSurvivorProps->m_pVertex, (void*)pDeletedProps->m_pVertex) ;
          smos_WriteBuffer(sBuff) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,6, 1,0,0) ; pSurvivorProps->m_pVertex->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,8, 0,0,1) ; pDeletedProps->m_pVertex->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // track topology changes for this single merge
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, TRUE) ;

      // merge the coincident vertices
      SmEdge * pDeletedEdge = NULL ;
      SER(pBrep->CombineCoincidentVertices(pSurvivorProps->m_pVertex,  // in : vertex to survive
                                           pDeletedProps->m_pVertex,   // in : vertex to delete
                                           pDeletedEdge)) ;            // out: edge deleted by squeeze (if any)

      // stop tracking and update healer state for this single merge
      sTopologyChanges.StopTracking() ;
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_COIN_VERTICES)) ;

      // fixup remaining pairs: replace all references to the deleted vertex's Props
      // with the survivor's Props so subsequent pairs that involved the deleted vertex
      // now target the survivor instead.
      for(ULONG jj=ii+1; jj<lPairCount; jj++)
        {
          if(sProbArray[2*jj]     == pDeletedProps) { sProbArray[2*jj]     = pSurvivorProps ; }
          if(sProbArray[2*jj + 1] == pDeletedProps) { sProbArray[2*jj + 1] = pSurvivorProps ; }
        }

    } // end iter every coincident vertex pair

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_CoinVertices

/*******************************************************************//**
PURPOSE: HealBrep helper function: Find Degen Faces

NOTES: 1. calls SmFace::IsDegenerate()
       2. Sets SmHealData::
             m_sBadFaceProps_DegenFaces
***********************************************************************/
SmStatus SmHealData::Cache_DegenFaces 
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // locals
  // const SmContext         * cpContext  = pBrep->GetContext();
  SmTArray<SmFace *>      & rFaces     = m_sTgtFaces;
  SmTArray<SmFaceProps *> & rFaceProps = m_sTgtFaceProps;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID( pBrep );

      smgfx_Erase(TRUE);
      smgfx_SetLook( 2, 3, 0, 0, 1 ); if ( pBrep ) pBrep->Draw( TRUE ); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if ( pBrep ) pBrep->DrawUV( TRUE ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // Delay Building SmHealData Problem ObjLists m_sBadFaceProps_DegenFaces
  // until after tolerances have been fixed.
  //  - avoids degenerating valid faces imported with invalid tolerances

  // for every FaceProps - determine if face is degenerate.
  ULONG lNumFaces = rFaceProps.GetSize();
  for(ULONG ii=0;ii<lNumFaces;ii++)
    {
      SmFaceProps * pThisFaceProps = rFaceProps[ii];
      //  pThisFaceProps->SetContext(cpContext) ; // this should be done
      pThisFaceProps->m_pFace = rFaces[ii];

      // Can't use UV with these invalid faces
      SmBoolean bSkipQuickCheck = TRUE;

      // Is Face Degenerate - DegenFace = Every Point in the Face is within Tol of a Face->Edge - found by checking for coincidence between FaceEdges
      //      gwc: the current definition of face degeneracy may need to change - 
      //           what about a panhandle shape where the pan is fine by the handle is a sliver piece
      //           as things stand that face will be classified as NonDegenerate.
      //           That may be okay because the sliver piece might be fixed by the Fix_CoinEdges() call.
      if(pThisFaceProps->m_pFace->IsDegenerate(bSkipQuickCheck))
        {
          m_sBadFaceProps_DegenFaces.Add(pThisFaceProps) ; *pbNewProbs = TRUE ; // side effects: marks vertex objprobFlag, counts orig problems
        }
    } // end SM_HO_CACHE_DEGEN_FACES

  // all done
  return( SM_SUCCESS );

} // end SmHealData::Cache_DegenFaces

/*******************************************************************//**
PURPOSE: HealBrep helper function: Replace Degen Faces with Edges and Vertices

NOTES: 1. Remove Degenerate Faces and Edges in order
         a. Remove Degenerate Faces first because that might remove edges and vertices
         b. Remove Degenerate Edges second because that might remove vertices
       2. Special case STEP closed surfaces (eg. tori) - they import with no seams and 
          1 Outer VertexLoop.  Rebuild the face with Seams and an 1 Outer EdgeLoop.
FIXES: problems in SmHealData::
                    m_sBadFaceProps_DegenFaces
***********************************************************************/
SmStatus SmHealData::Fix_DegenFaces
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
  // locals
  ULONG ii, jj ;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ; 
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; for(di=0;di<m_sBadFaceProps_DegenFaces.GetSize();di++) 
                                    { m_sBadFaceProps_DegenFaces[di]->Dump(m_sBadFaceProps_DegenFaces[di]->m_lFaceIndx) ;
                                      m_sBadFaceProps_DegenFaces[di]->m_pFace->Dump() ;
                                      m_sBadFaceProps_DegenFaces[di]->m_pFace->DumpTopology() ;
                                      m_sBadFaceProps_DegenFaces[di]->Draw() ; 
                                      m_sBadFaceProps_DegenFaces[di]->m_pFace->DrawUV() ; sm_GraphicsLoop() ; 
                                    } 
      for(di=0;di<m_sTgtFaces.GetSize();di++) 
        { if(m_sTgtFaces[di]->IsSheet()) { smgfx_SetLook(1,2, 1,0,1) ; m_sTgtFaces[di]->DrawUV() ; sm_GraphicsLoop() ; } 
          else                           { smgfx_SetLook(1,2, 0,0,1) ; m_sTgtFaces[di]->DrawUV() ; sm_GraphicsLoop() ; } 
        }  
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when there are DegenFaces - Replace all Degenerate Faces with Edges and Vertices
  ULONG lNumDegenFaces = m_sBadFaceProps_DegenFaces.GetSize();
  if(lNumDegenFaces > 0)
    {
      // prepare to remove DegenFaces
      SmTemporaryChangeValue<SmBoolean> sSaveEditing( pBrep->m_bEditingEnabled, TRUE) ;

      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      SmTArray<SmEdge *> sFaceEdges ;
      SmTArray<SmLoop *> sFaceLoops;

      // Heal or squeeze each degen face
      for(ii=0;ii<lNumDegenFaces;ii++)
        {
          // Loop locals
          SmFaceProps *& prThisFaceProps = m_sBadFaceProps_DegenFaces[ii] ;

          // A re-entrant heal step (via UpdateTargetLists->HealTgts) can free a FaceProps still queued
          // here; skip if it is no longer in the live m_sTgtFaceProps (FindElement is identity-only).
          { ULONG lLiveIdx ;
            if(!m_sTgtFaceProps.FindElement(prThisFaceProps, lLiveIdx))
              { continue ; }
          }

          SmFace       * pThisFace       = prThisFaceProps->m_pFace;

          // skip problems already fixed
          if(prThisFaceProps->m_bBadDegenFace == FALSE)
            { continue ; }

          // Squeezing/deleting another degen face can free this still-queued face; skip if not live.
          // IsLiveTopologyMember is pointer-identity only, safe on a dangling ptr (GetEdges is not).
          if(m_pBrep != NULL && !m_pBrep->IsLiveTopologyMember(pThisFace))
            { continue ; }

          // loop locals
          pThisFace->GetEdges( sFaceEdges );
          pThisFace->GetLoops( sFaceLoops );

          // Special case: STEP import of Torus as degen face. Closed surfaces (e.g. torus) are imported by STEP with a vertex as OuterLoop
          // Fix: Add seam Edge(s) and Vertex as necessary.
          if(   sFaceLoops.GetSize() == 1                                            //     Face has 1 Loop
             && sFaceLoops[0]->GetLoopuse()->GetLoopuseType() == SmVertexuse_TYPE)   // and loop is a VertexLoop
            { 
              SmXSectTol3d sXSectTol3d      = SmTol::GetXSectTol3d( pThisFace, pThisFace );
              SmSurface  * pSurface         = pThisFace->GetSurface();
              SmExtent2d   sNaturalUVDomain = pSurface->GetNaturalUVDomain();

              // Closed pThisFace->Surface classification: m_bClosedU, m_bClosedV 
              SmBoolean bClosedU = pSurface->IsClosed( sNaturalUVDomain, SM_SP_U, &sXSectTol3d.val );
              SmBoolean bClosedV = pSurface->IsClosed( sNaturalUVDomain, SM_SP_V, &sXSectTol3d.val );

              // Singularity classification: m_lPoles
              ULONG lSingularities = pSurface->GetSingularities();

              // If the surface is closed, rebuild it correctly
              if ( ( bClosedU && bClosedV )   // torus
                   || ( bClosedV && ( lSingularities & SM_SS_UMIN ) && ( lSingularities & SM_SS_UMAX ) )   // sphere
                   || ( bClosedU && ( lSingularities & SM_SS_VMIN ) && ( lSingularities & SM_SS_VMAX ) ) ) // sphere
                {
                  // Locals
                  ULONG       lIndx = 0 ;
                  SmSurface * pNewSurface ;
                  SmVertex  * pNewVertex = NULL ;
                  SmFace    * pNewFace ;
                  SmShell   * pNewShell ;
                  SmRegion  * pOldRegion = pThisFace->GetUpwardFaceuse()->GetShell()->GetRegion() ;
                  SmVertex  * pOldVertex = sFaceLoops[0]->GetLoopuse()->GetVertexuse()->GetVertex() ;
                  SmTArray<SmTopology *> sDelObjs ;

                  // if possible promote the surface to an analytic
                  pSurface->CopyAndAddAnalytics( *pBrep->GetContext(), pNewSurface) ;

                  // Remove the improperly created face from the brep
                  pBrep->DeleteFace(pThisFace,    // in : target face
                                    TRUE,         // in : TRUE = delete connected edges and vertices not connected to anything else
                                    TRUE,         // in : TRUE = combine regions/shells as appropriate
                                    &sDelObjs,    // out: list of all stale edge and vertex ptrs that were deleted
                                    pOldRegion) ; // in : Opt Region kept when bDoRegionNesting = TRUE

                  // note: if pOldVertex becomes a shell vertex it gets deleted in DeleteFace()
                  //       if pOldVertex remains a EdgeVertex it did not get deleted - 
                  //                     expect it to be coincident with one of the new Vertices built by CreateFaceInRegionFromSurface()

                  // Create a new Face with proper topology - this face will have one or two new edges and two new vertices
                  SE( pBrep->CreateFaceInRegionFromSurface(pOldRegion,                         
                                                           pNewSurface,                        
                                                           pNewSurface->GetNaturalUVDomain(),
                                                           pNewShell,
                                                           pNewFace) );

                  // When NewFace is created - save it
                  m_sNewFaces_FromFixDegenFaces.Add(pNewFace) ;

                  // when pOldVertex still exists - glue it to the coincident new vertex
                  if(FALSE == sDelObjs.FindElement(pOldVertex, lIndx))
                    {
                      SmTArray<SmVertex*> sTempNewVertices ;
                      pNewFace->GetVertices( sTempNewVertices) ;
                  
                      // Search for a new vertex with same location as pOldVertex - pThisFace has only 1 vertex
                      for(jj=0;jj<sTempNewVertices.GetSize();jj++)
                        {
                          SmVertex     *pThisVertex = sTempNewVertices[jj];
                          SmXSectTol3d sXSectTol = SmTol::GetXSectTol3d( pThisVertex, pOldVertex );

                          // look for coincidence
                          if ( ( pThisVertex->GetPoint() - pOldVertex->GetPoint() ).Length() < sXSectTol )
                            {
                              // found one - remember it - glue it
                              pNewVertex = pThisVertex;
                              SE( pBrep->GlueVertices( pNewVertex, pOldVertex ) );
                              break;
                            }
                        } // end search over all new vertices

                      // check state: always expect one of the new vertices to be coincident with the pOldVertex
                      SM_ASSERT_MSG(pNewVertex != NULL,
                                    _T("SmHealData::FixDegenFaces broken assumption - needs code change: surviving vertex was assumed to be coincident to new vertices - or deleted")) ;

                    } // end pOldVertex existence check

                } // end pThisFace is a closed branch - build new closed-surface face branch
              else // pThisFace is not a closed face (torus or Sphere) - case: a single degen sheet with one LoopVertex
                {
                  SmTArray<SmTopology*> sDeletedTopology;
                  SE( pBrep->DeleteFace( pThisFace, TRUE, FALSE, &sDeletedTopology ) ); // TRUE = delete connected edges and vertices
                                                                                        // FALSE = don't combine regions/shells as appropriate

                  // This Face has only one piece of topology. Record if the Vertex was deleted
                  SM_ASSERT( sDeletedTopology.GetSize() <= 1 );

                } // end pThisFace is not a closed Face branch
            } // End Face has 1 OuterLoop==VertexLoop branch (special STEP case)
          else // Face is not a special STEP case with just 1 outer LoopVertex branch - squeeze it 
            {
              SmTArray<SmEdge *> sTempNEdges, sTempDEdges ;

              // Remove a face from the topology graph leaving one or more vertices and edges behind. 
              SE( pBrep->SqueezeFace( pThisFace, NULL, NULL, &sTempNEdges, &sTempDEdges ) );
              sFaceEdges.AppendUnique(sTempNEdges) ;

              // begin scope for deleting wire edges made by squeezing DegenFaces. (wire edges == non-manifold)
                {
                  // remove sTempDEdges from sFaceEdges array
                  sFaceEdges.RemoveElements( sTempDEdges, sFaceEdges) ;
                  ULONG lNumEdges = sFaceEdges.GetSize();
                  SmBoolean bRemovedDuplicates = FALSE;
                  for(jj=0;jj<lNumEdges;jj++)
                    {
                      // when Edge is wire - delete it - delete any vertices that become shell vertices 
                      if ( sFaceEdges[jj]->IsWire() )
                        {
                          // Grab the vertices
                          SmVertex *pVertex1 = sFaceEdges[jj]->GetVertex();
                          SmVertex *pVertex2 = sFaceEdges[jj]->GetOtherVertex( pVertex1 );

                          // Delete the edge
                          pBrep->DeleteEdge( sFaceEdges[jj]) ;

                          // Delete any vertices that are now shell vertices

                          // when Edge was closed and singleVertex is a ShellVertex
                          if(pVertex1 == pVertex2 && pVertex1->IsShellVertex() )
                            { pBrep->DeleteVertex( pVertex1) ;  // end single ShellVertex branch    
                            } 
                          else // Open Edge
                            {
                              if(pVertex1->IsShellVertex()) { pBrep->DeleteVertex( pVertex1) ; } // Vertex1 Is now a Shell - delete it
                              if(pVertex2->IsShellVertex()) { pBrep->DeleteVertex( pVertex2) ; } // Vertex2 is now a Shell - delete it
                            } // End deleting OpenEdge Vertices made into ShellVertices when Edge was deleted branch

                          // If there are wires, there may be duplicates in sFaceEdges. Remove them so we don't delete a stale edge pointer
                          // This can happen when connected degenerate faces are squeezed
                          // for valid faces this branch should never run - if we have a bad face with nested loops it might come up
                          if( !bRemovedDuplicates)
                            {
                              sFaceEdges.RemoveDuplicates();
                              lNumEdges = sFaceEdges.GetSize();
                              bRemovedDuplicates = TRUE;
                            }
                        } // end if this edge is a wire check
                    } // end iter every FaceEdge looking to delete original edges that became wires
                } // end scope for deleting wire edges made by squeezing DegenFaces. (wire edges == non-manifold)
            } // end squeeze DegenFace branch
        } // End iter and squeezing every DegenFace
        
      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_DEGEN_FACES)) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_ASSERT(1) ;
      this->Dump() ;
      sTopologyChanges.Dump() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      // all m_sBadEdgeProps_DegenFaces should have been deleted - added to the sTopologyChanges->RmList and then removed from the SmHealData Prob arrays
      SM_ASSERT_MSG(m_sBadFaceProps_DegenFaces.m_sProbArray.GetSize() == 0,
                    _T("SmHealData::Fix_DegenFaces broken state: m_sBadFaceProps_DegenFaces still has entries after Fix_DegenFaces() has run - should be empty")) ;

    } // end has DegenFaces check 

  // all done
  return( SM_SUCCESS );

} // end SmHealData::Fix_DegenFaces

/*******************************************************************//**
PURPOSE: HealBrep helper function: Remove Degen Edges

NOTES: replace Degenerate Edges with Vertices
***********************************************************************/
SmStatus SmHealData::Fix_DegenEdges
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{                      
  SM_REF1(eThisHealerOp) ;
  // locals
  ULONG lNumBadEdgeProps_Degen = m_sBadEdgeProps_DegenEdges.m_sProbArray.GetSize();

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .5,.5,1) ; for(di=0;di<m_sBadEdgeProps_DegenEdges.m_sProbArray.GetSize();di++)
                                      { if(m_sBadEdgeProps_DegenEdges.m_sProbArray[di]) m_sBadEdgeProps_DegenEdges.m_sProbArray[di]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // replace Degenerate Edges with Vertices
  if(lNumBadEdgeProps_Degen > 0)
    {
      // prepare to remove DegenEdges
      SmTemporaryChangeValue<SmBoolean> sSaveEditing(pBrep->m_bEditingEnabled, TRUE);

      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // for every DegenEdge - delete it
      for(ULONG ii=lNumBadEdgeProps_Degen;ii>0; ii--)
        {
          SmEdgeProps  * pThisEdgeProps = m_sBadEdgeProps_DegenEdges.m_sProbArray[ii-1] ; 
          if(pThisEdgeProps == NULL || pThisEdgeProps->m_pEdge == NULL)
            { SER_MSG(SM_ERR, _T("Null edge properties in degenerate-edge queue")) ; }

          SmEdge       * pEdge          = (SmEdge *)pThisEdgeProps->m_pEdge ;
          SmVertex     * pStartVertex   = NULL ;
          SmVertex     * pEndVertex     = NULL ;

          // skip Edges already fixed
          if(pThisEdgeProps->m_bBadDegenEdge == FALSE)
            { continue ; }

          // get Edge->Vertices
          pEdge->GetVertices( pStartVertex, pEndVertex) ;

          // Squeeze the edge from vertex-edge-vertex to vertex
          if(pStartVertex != pEndVertex)
            { SER( pBrep->SqueezeEdge( pEdge, pStartVertex )) ; }
          else
            { SER( pBrep->DeleteEdge(pEdge) ) ; }

          // EdgeProps cleanup is handled by UpdateTargetLists below,
          // which removes the props from all lists and deletes them.

        } // end iter every DegenEdge

      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_DEGEN_EDGES)) ;

      // all m_sBadEdgeProps_DegenEdges should have been deleted - added to the sTopologyChanges->RmList and then removed from the SmHealData Prob arrays
      SM_ASSERT_MSG(m_sBadEdgeProps_DegenEdges.m_sProbArray.GetSize() == 0,
                    _T("SmHealData::Fix_DegenEdges broken state: m_sBadEdgeProps_DegenEdges still has entries after Fix_DegenEdges() has run - should be empty")) ;

    } // end has DegenEdges check 

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_DegenEdges

/*******************************************************************//**
PURPOSE: HealBrep helper function: Find Coincident Edges

NOTES: 1. Manage FaceProps state -
          Faces with CoinEdges are placed on list m_sBadEdgeProps_CoinEdges ** TODO **
          Faces with no CoinEdges are marked
            m_bAfterFix_CoinEdges == TRUE                         ** DONE **
       2. Find and remove coincident Edges first because
          that might help finding degen edges and faces for cheap
TODO: Not yet implemented!
***********************************************************************/
SmStatus SmHealData::Cache_CoinEdges /* TODO */
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;// init output
  *pbNewProbs = FALSE ;

  // no work - no Edges or no new Edges
  if(   m_sTgtEdgeProps.GetSize() == 0
     || m_sTgtEdgeProps.GetSize() == m_lOldEdgeCnt)
    { return SM_SUCCESS ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
    SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii ;
  SmTArray<SmEdgeProps *> & rEdgeProps = m_sTgtEdgeProps ;

  // for every EdgeProp obj - Get EdgeProperties  - (m_lOldEdgeCnt != 0 when updating TgtLists with new objs)
  for(ii=m_lOldEdgeCnt;ii<rEdgeProps.GetSize();ii++)
    {
      // SmEdgeProps * pThisEdgeProps = rEdgeProps[ii] ;
      // pThisEdgeProps->SetContext( cpContext ) ;  // not needed - set in SmHealData::Cache_EdgeProps()

      // TODO  Add detect coinEdge code here

      // TODO  Add SmHealData::m_sBadEdgeProps_CoinEdges, *pbNewProbs = TRUE ;  (contains merge surviving Edges)

    } // end iter every Edge - computing and saving face classification properties

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_CoinEdges

/*******************************************************************//**
PURPOSE: HealBrep helper function: Merge Coincident Edges

NOTES: Find and remove coincident Edges first because
       that might help finding degen edges and faces for cheap
TODO: Not yet implemented!
***********************************************************************/
SmStatus SmHealData::Fix_CoinEdges /* TODO */
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // when there are coincident Edges - not yet implemented, report error
  if(m_sBadEdgeProps_CoinEdges.GetSize() > 0)
    { return(SM_ERR) ; }

  // all done - no problems to fix
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_CoinEdges

/*******************************************************************//**
PURPOSE: HealBrep helper function: Find Missed Edge XSects

NOTES: 1. Manage FaceProps state -
          Faces with MissedEdgeXSects are placed on list m_sBadEdgeProps_MissedEdgeXSects ** TODO **
          Faces with no MissedEdgeXSects are marked
            m_bAfterFix_MissedEdgeXSects == TRUE                         ** TODO **
TODO: Not yet implemented!
***********************************************************************/
SmStatus SmHealData::Cache_MissedEdgeXSects /* TODO */
 (SmBrep         * pBrep,           // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,      // out: TRUE  = Found new problems while caching data
                                    //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // no work - no Edges or no new Edges
  if(   m_sTgtEdgeProps.GetSize() == 0
     || m_sTgtEdgeProps.GetSize() == m_lOldEdgeCnt)
    { return SM_SUCCESS ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
    SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii ;
  SmTArray<SmEdgeProps *> & rEdgeProps = m_sTgtEdgeProps ;

  // for every EdgeProp obj - Get EdgeProperties  - (m_lOldEdgeCnt != 0 when updating TgtLists with new objs)
  for(ii=m_lOldEdgeCnt;ii<rEdgeProps.GetSize();ii++)
    {
      // SmEdgeProps * pThisEdgeProps = rEdgeProps[ii] ;
      // pThisEdgeProps->SetContext( cpContext ) ;  // not needed - set in SmHealData::Cache_EdgeProps()

      // TODO  Add detect MissedEdgeXSect code here

      // TODO  Add SmHealData::m_sBadEdgeProps_MissedEdgeXSects, *pbNewProbs = TRUE ;  (contains merge surviving Edges)

    } // end iter every Edge - computing and saving face classification properties

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_MissedEdgeXSects

/*******************************************************************//**
PURPOSE: HealBrep helper function: Insert Vertex at MissedEdgeXSects

NOTES: Insert Vertex at every MissedEdgeXSect
TODO: Not yet implemented!
***********************************************************************/
SmStatus SmHealData::Fix_MissedEdgeXSects /* TODO */
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // when there are missed edge intersections - not yet implemented, report error
  if(m_sBadEdgeProps_MissedEdgeXSects.GetSize() > 0)
    { return(SM_ERR) ; }

  // all done - no problems to fix
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_MissedEdgeXSects

/*******************************************************************//**
PURPOSE: HealBrep helper function: Fix Uncontained Edges

NOTES: fix Uncontained Edge/Curve errors 
***********************************************************************/
SmStatus SmHealData::Fix_UncontainedEdges
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{
  SM_REF1(eThisHealerOp) ;
  // fix Uncontained Edge/Curve errors - accumulate counts and lists for any other problems
  if(m_sBadEdgeProps_Uncontained.GetSize() > 0)
    {
      // for every prob object
      for(ULONG ii=0;ii<m_sBadEdgeProps_Uncontained.GetSize();ii++)
        {
          SmEdgeProps * pThisEdgeProps = m_sBadEdgeProps_Uncontained[ii] ;  

          // GWC_BEFORE_NEXT_RELEASE replace_next_block_of_code_before_release__This_is_a_Place_holder_for_a_good_fix_Scheme ; 
            { // begin Scope for place holder fix that needs to be replaced by a good fix algorithm

              if(pThisEdgeProps->m_sEdgeInterval.GetMin() < pThisEdgeProps->m_sNaturalInterval.GetMin())
                { pThisEdgeProps->m_sEdgeInterval.SetMin(pThisEdgeProps->m_sNaturalInterval.GetMin()) ; }

              if(pThisEdgeProps->m_sEdgeInterval.GetMax() > pThisEdgeProps->m_sNaturalInterval.GetMax())
                { pThisEdgeProps->m_sEdgeInterval.SetMax(pThisEdgeProps->m_sNaturalInterval.GetMax()) ; }

              // remember the fix
              pThisEdgeProps->m_bBadUncontainedEdge = FALSE ;

            } // end Scope for place holder fix that needs to be replaced by a good fix algorithm
            
        } // end iter every Uncontained Edge/Curve problems

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          smgfx_Erase(TRUE) ;
          smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .2,.4,.6) ; for(di=0;di<m_sBadEdgeProps_Uncontained.GetSize();di++)
                                          { if(m_sBadEdgeProps_Uncontained[di]) m_sBadEdgeProps_Uncontained[di]->Draw() ; sm_GraphicsLoop() ; }
          sm_GraphicsLoop() ;
        }
#else
        SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

    } // end need to heal uncontained_EdgeIvls check

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_UncontainedEdges

/*******************************************************************//**
PURPOSE: HealBrep helper function: Fix BadGaps

NOTES: Edit Geometry as needed to reduce Gap Sizes

TODO: Not yet implemented!
***********************************************************************/
SmStatus SmHealData::Fix_BadGaps
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_BadGaps

/*******************************************************************//**
PURPOSE: HealBrep helper function: 
         Build EdgeProps before Split SeamCrossingEdges

NOTES: set FaceProps[ii]: 
         m_pFace               m_sXSectTol3d          
         m_cpSurface           m_sNaturalUVDomain
         m_bSheetFace          
         m_bClosedU            m_bClosedV          m_bClosedSurf
         m_eClosedValU  m_eClosedValV
         m_lPoles              m_lApproxPoles      m_lNoVertexPoles
         m_sPolePoints         m_sPoleNormals      
         m_lBadFlatCorners     m_sFlatCornerPoints 
         m_pCrvClassU          m_pCrvClassV        
         m_sRawTouchListU      m_sRawTouchListV
         m_sDoneTouchListU     m_sDoneTouchListV
         m_eBadMissingSeam     m_eBadCrossedSeam   m_eBadNearMissSeam          
***********************************************************************/
SmStatus SmHealData::Cache_FaceProps_Stage2
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // no work - no Faces or no new Faces
  if(   m_sTgtFaceProps.GetSize() == 0
     || m_sTgtFaceProps.GetSize() == m_lOldFaceCnt)
    { return SM_SUCCESS ; }

  // locals
  ULONG ii ;
  SmTArray<SmFaceProps *> & rFaceProps = m_sTgtFaceProps ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
if(bDebugMe)
  {
    SM_DUMP_AND_ASSERT_VALID(pBrep) ;

    smgfx_Erase(TRUE) ;
    smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;
  }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // for every FaceProp obj - Get FaceProperties - (m_lOldFaceCnt != 0 when updating TgtLists with new objs)
  for(ii=m_lOldFaceCnt;ii<rFaceProps.GetSize();ii++)
    {
      // set m_pFace and compute:[XSectTol3d, bSheet, bClosed, lPoles, lFlatCorners]
      SmFaceProps * pThisFaceProps = rFaceProps[ii] ;
        
      // set FaceProps_Stage2 - (possibly done when pOptFaceProps != NULL)
      if(FALSE == pThisFaceProps->HasProps(SM_PROPSTAGE_2, pThisFaceProps->m_pFace))
        {
          // sets: pThisFaceProps: 
          //        m_pFace               m_sXSectTol3d          
          //        m_cpSurface           m_sNaturalUVDomain
          //        m_bSheetFace          
          //        m_bClosedU            m_bClosedV          m_bClosedSurf
          //        m_eClosedValU         m_eClosedValV              
          //        m_lPoles              m_lApproxPoles      m_lNoVertexPoles
          //        m_sPolePoints         m_sPoleNormals      
          //        m_lBadFlatCorners     m_sFlatCornerPoints 
          //        m_pCrvClassU          m_pCrvClassV        
          //        m_sRawTouchListU      m_sRawTouchListV
          //        m_sDoneTouchListU     m_sDoneTouchListV
          //        m_eBadMissingSeam     m_eBadCrossedSeam   m_eBadNearMissSeam
          pThisFaceProps->SetProps(SM_PROPSTAGE_2, pThisFaceProps->m_pFace, ii, m_eDoneHealOp, ii) ; 
        }  

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          pThisFaceProps->Dump(ii) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                           // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                           // sMissingSeamColor   - def:[ 1, 0, 0]
                                                           // sMissingPoleColor   - def:[ 0, 0, 1]
                                                           // sBadOrientLoopColor - def:[.3,.3,.3]
                                                           // sNoArea_LoopColor   - def:[ 0, 0, 0]
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // count the kinds of faces that force healing checks later on

      // SheetFace List
      if(pThisFaceProps->m_eSheet != SM_MP_NOPROP)
        { m_sFaceProps_Sheets.Add(pThisFaceProps, pThisFaceProps->m_eSheet) ; }

      // NoVertex_Poles Face List - not a problem when pole is not 'in' or 'on' Face 
      if(pThisFaceProps->m_ePoles != SM_MP_NOPROP)
        { m_sFaceProps_NoVertexPole.Add(pThisFaceProps, pThisFaceProps->m_ePoles) ; }

      // FlatCorners Face List
      if(pThisFaceProps->m_lBadFlatCorners != 0)
        { m_sBadFaceProps_FlatCorner.Add(pThisFaceProps, pThisFaceProps->m_lBadFlatCorners) ; *pbNewProbs = TRUE ; }

      // Missing Seam List
      if(pThisFaceProps->m_bBadMissingSeam != 0)
        { m_sBadFaceProps_MissingSeam.Add(pThisFaceProps, pThisFaceProps->m_bBadMissingSeam) ; *pbNewProbs = TRUE ; }

      // Crossed Seam List
      if(pThisFaceProps->m_bBadCrossedSeam != 0)
        { m_sBadFaceProps_CrossedSeam.Add(pThisFaceProps, pThisFaceProps->m_bBadCrossedSeam) ; *pbNewProbs = TRUE ; }

      // Near Miss Seam List
      if(pThisFaceProps->m_bBadNearMissSeam != 0)
        { m_sBadFaceProps_NearMissSeam.Add(pThisFaceProps, pThisFaceProps->m_bBadNearMissSeam) ; *pbNewProbs = TRUE ; }

      // note: BadCrossingSeam vs. BadMissingSeam Faces
      //  A BadCrossingSeam Face is a Face with any Edge that crosses a Seam.  If the Seam can't be moved
      //    outside of the face, these edges need to be split creating a new vertex placed on the seam.
      //  A BadMissingSeam Face is a Face that contains any SeamInterval that is not marked by an Edge.
      //    If the Seam can't be moved outside of the face, the SeamIntervals in the face need to be merged
      //    into the face creating new SeamEdges and possibly splitting the orig face into children faces.
      //
      //  All BadCrossingSeam faces will also be BadMissingSeam faces but not all BadMissingSeam faces
      //    will be BadCrossingSeam faces because some MissingSeam intervals might already be bounded by
      //    vertices.

    } // end iter every Face - computing and saving face classification properties

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(ULONG di=0;di<m_sFaceProps_Sheets.GetSize();di++)
        { SmFaceProps * pThisFaceProps = rFaceProps[di] ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                           // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                           // sMissingSeamColor   - def:[ 1, 0, 0]
                                                           // sMissingPoleColor   - def:[ 0, 0, 1]
                                                           // sBadOrientLoopColor - def:[.3,.3,.3]
                                                           // sNoArea_LoopColor   - def:[ 0, 0, 0]
        }
      smgfx_SetLook(4,5, .2,.4,.6) ; for(ULONG di=0;di<m_sBadEdgeProps_Uncontained.GetSize();di++)
                                      { if(m_sBadEdgeProps_Uncontained[di]) m_sBadEdgeProps_Uncontained[di]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ; 
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_FaceProps_Stage2

/*******************************************************************//**
PURPOSE: HealBrep helper function: 
           For All Seam problems - MoveSeam to better spot

NOTES: For Faces with LoopEdges Crossing Seams,
                      MissingSeamEdges, or
                      LoopBoundaries near Seams
       Move Seam.  If possible completely out of the Face
                   Else to face center away from LoopBoundaries
                   so that a future Face->SplitAtSeam call does
                   not create any sliver faces.
***********************************************************************/
SmStatus SmHealData::Fix_MoveSeam
 (SmBrep           * pBrep,           // in : Tgt Brep being healed
  SmNewMarkAndLock & rMarkLock,       // i/o: Mark available for this Heal helper function
  SmHealerOpType     eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
  // locals
  ULONG ii ;
  
#ifdef SM_DEBUG_CODE
  SmTArray<SmFaceProps*>& rFaceProps = m_sTgtFaceProps;

// GWC_SET_NEXT__TWO_LINES__TO_FALSE_BEFORE_RELEASE ;
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugThisFaceIndx = FALSE ; 
  if(bDebugMe)
    {
      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // build total seam problem array - may contain duplicates and that's okay
  SmTArray<SmFaceProps *> sSeamProbs ;
  if(m_lOldFaceCnt != 0) // doing an update Tgts
    {
      for(ii=m_lOldFaceCnt;ii<m_sTgtFaceProps.GetSize();ii++)
        {
          SmFaceProps * pThisFaceProps = m_sTgtFaceProps[ii] ;
          if(   pThisFaceProps->m_bBadMissingSeam 
             || pThisFaceProps->m_bBadCrossedSeam 
             || pThisFaceProps->m_bBadNearMissSeam)
            { sSeamProbs.Add(pThisFaceProps) ; }
        }
    }
  else // working on all known problems
    {
      sSeamProbs.Append(m_sBadFaceProps_MissingSeam.m_sProbArray) ;
      sSeamProbs.AppendUnique(m_sBadFaceProps_CrossedSeam.m_sProbArray) ; 
      sSeamProbs.AppendUnique(m_sBadFaceProps_NearMissSeam.m_sProbArray) ;
    }

  // when there are seam problems
  if(sSeamProbs.GetSize() > 0)
    {
      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // init mark
      rMarkLock.NewMark() ; // increment the mark value - very cheap

      // for every Face    with Edges that Cross Seams 
      //                or have MissingSeam Intervals 
      //                or have NearSeam LoopBndries - try moving the seam out of the face
      for(ii=sSeamProbs.GetSize();ii>0;ii--)
        {
          // locals
          SmFaceProps * pThisFaceProps = sSeamProbs[ii-1] ;
          SmFace      * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ;

  #ifdef SM_DEBUG_CODE
          // debug draw - CrossingSeamEdges, MissingSeamFaces, and NearMissSeamLoops
          if(bDebugMe || SmDebugThisFaceIndex(pThisFaceProps->m_lFaceIndx, bDebugThisFaceIndx))
            {
              ULONG dCnt = rFaceProps.GetSize() ;
              ULONG lIndx = 0 ;

              for(ULONG di=0;di<dCnt;di++) { if(rFaceProps[di] == pThisFaceProps) { lIndx = di%dCnt ; break ; } }          
              pThisFaceProps->Dump(lIndx) ;
              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                               // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                               // sMissingSeamColor   - def:[ 1, 0, 0]
                                                               // sMissingPoleColor   - def:[ 0, 0, 1]
                                                               // sBadOrientLoopColor - def:[.3,.3,.3]
                                                               // sNoArea_LoopColor   - def:[ 0, 0, 0]
              pThisFaceProps->Dump(lIndx) ;
              sm_GraphicsLoop() ;
            }
  #endif // SM_DEBUG_CODE

          // for now - skip Faces already run through Fix_MoveSeam(). This stops Fix_MoveSeam() from running twice on duplicate members of sSeamProbs.
          if(pThisFace->IsMarked(rMarkLock.GetMarkType()))
            { continue ; }

          // Mark pBrep Edge - its about to get split
          pThisFace->Mark(rMarkLock.GetMarkType()) ; 

          // GWC: don't skip any cases - MoveSeam will fix MissingSeam problems when Face is not FullPeriodic 
          //   // skip faces without a LoopBoundary Crossing or NearMiss Seam problem
          //   if(   pThisFaceProps->m_eBadCrossedSeam  == SM_SP_NEITHER
          //      && pThisFaceProps->m_eBadNearMissSeam == SM_SP_NEITHER)
          //     { continue ; }

          // Move Seam out of Face if possible, else towards center of Face to avoid making sliver faces
          SmFixSeamPlanType eMoveSeamResult_U ;
          SmFixSeamPlanType eMoveSeamResult_V ;
          SmStatus sRtn = pThisFace->MoveSeam  
               (eMoveSeamResult_U,                  ///< [out]: orof SM_MS_NO_MOVE, SM_MS_OUT_OF_FACE_U, SM_MS_OUT_OF_FACE_V   <br>
                eMoveSeamResult_V,                  ///<                            SM_MS_IN_FACE_U,     SM_MS_IN_FACE_V        <br>
                *pThisFaceProps->m_pCrvClassU,      ///< [in] : ConstU_Seam/Face->Edges XSects, from HasSeamProblem() call <br>  
                *pThisFaceProps->m_pCrvClassV,      ///< [in] : ConstV_Seam/Face->Edges XSects, from HasSeamProblem() call <br> 
                 pThisFaceProps->m_eBadCrossedSeam, ///< [in] : from HasSeamProblem() output(bOuterLoopOnly==FALSE) <br>
                                                    ///<      : SM_SP_U       = periodic U bndry seam crossed by Face->Edges <br>                
                                                    ///<      : SM_SP_V       = periodic V bndry seam crossed by Face->Edges <br>               
                                                    ///<      : SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges <br>    
                                                    ///<      : SM_SP_NEITHER = no seams crossed by Face->Edges <br>                             
                 pThisFaceProps->m_eBadMissingSeam, ///< [in] : from HasSeamProblem() call output(bOuterLoopOnly==FALSE) <br>  
                                                    ///<      : SM_SP_U       = periodic U bndry seam not represented by Face->Edges <br>         
                                                    ///<      : SM_SP_V       = periodic V bndry seam not represented by Face->Edges <br>           
                                                    ///<      : SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges <br>
                                                    ///<      : SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges <br>
                 pThisFaceProps->m_eBadNearMissSeam,///< [in] : from HasSeamProblem() call output(bOuterLoopOnly==FALSE) <br>  
                                                    ///<      : For future use once Fix_BadGaps() gets built
                                                    ///<      : SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam <br>
                                                    ///<      : SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam <br>
                                                    ///<      : SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams <br>
                                                    ///<      : SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams <br>
                                                    ///<      : NearMiss Topo connections are valid - future Fix_BadGaps() will refine geom to tighten gaps from NearMiss to Exact
     /* ===> */ TRUE,                               ///< [in] : TRUE =okay to move Seam in Face when seam can't move out of Face to avoid sliverFaces <br>  
                                                    ///<        FALSE=only move Seam out of Face <br>  
     /* ===> */ FALSE,                              ///< [in] : TRUE = split Face at Seam(s) when seam can't move out of Face <br>
                                                    ///<      : FALSE= don't split Face at Seam(s) <br>
                NULL,                               ///< [out]: NewFaces made if any, only with bOKToSplit==TRUE.  (does not include this face) <br>
                                                    ///<        NULL to ignore, default:[NULL] <br>
                NULL,                               ///< [out]: New SeamEdges added if any, only with bOkToSplit==TRUE  <br>    
                NULL,                               ///< [out]: New Vertices added at the Seam if any, only with bOkToSplit==TRUE  <br>          
                pThisFaceProps) ;                   ///< [in] : optional FaceProps to save time, NULL to ignore, default:[NULL]  <br>   
          SM_ASSERT(sRtn == SM_SUCCESS) ;

          // when changes were made - rebuild ThisFaceProps (updates stale cache vals and sees if any probs remain)
          if(eMoveSeamResult_U != SM_FSP_NO_CHANGES || eMoveSeamResult_V != SM_FSP_NO_CHANGES)
            {
  #ifdef SM_DEBUG_CODE
              if(bDebugMe || SmDebugThisFaceIndex(pThisFaceProps->m_lFaceIndx, bDebugThisFaceIndx))
                {
                  ULONG lIndx = 0 ;
                  ULONG dCnt = rFaceProps.GetSize() ;
                  for(ULONG di=0;di<dCnt;di++) { if(rFaceProps[di] == pThisFaceProps) { lIndx = di%dCnt ; break ; } }
                  pThisFaceProps->Dump(lIndx) ;

                  smgfx_Erase(TRUE) ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                                   // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                                   // sMissingSeamColor   - def:[ 1, 0, 0]
                                                                   // sMissingPoleColor   - def:[ 0, 0, 1]
                                                                   // sBadOrientLoopColor - def:[.3,.3,.3]
                                                                   // sNoArea_LoopColor   - def:[ 0, 0, 0]
                  pThisFaceProps->Dump(lIndx) ;
                  sm_GraphicsLoop() ;
                }
  #endif // SM_DEBUG_CODE
            } // end changes were made check

        } // end iter every face looking for problem Face->Loop->Edges that cross Seams to fix

      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Rebuild seam classifications before the next seam-healing step uses them.
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_MOVESEAM)) ;

      // Refresh replaces FaceProps; the locked face mark survives the nested heal pass.
      for(ii=0;ii<m_sTgtFaceProps.GetSize();ii++)
        {
          SmFaceProps * pThisFaceProps = m_sTgtFaceProps[ii] ; 
          if (pThisFaceProps == NULL) 
            { 
              SE_MSG(SM_ERR, _T("FaceProps is NULL - needs review")) ; 
              continue ; 
            }
          if(pThisFaceProps->m_pFace->IsMarked(rMarkLock.GetMarkType()))
            {
              m_sRanFaceProps_MoveSeam.Add(pThisFaceProps,
                    (   pThisFaceProps->m_bBadMissingSeam  == FALSE
                     && pThisFaceProps->m_bBadCrossedSeam  == FALSE
                     && pThisFaceProps->m_bBadNearMissSeam == FALSE)
                    ? SM_TRY_FIXED : SM_TRY_NOFIX) ;
            }
        }

    } // end when there are seam problems check

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_MoveSeam

/*******************************************************************//**
PURPOSE: HealBrep helper function: Split Edges Crossing Seams

NOTES: 
***********************************************************************/
SmStatus SmHealData::Fix_SplitEdgesAtSeam
 (SmBrep           * pBrep,           // in : Tgt Brep being healed
  SmNewMarkAndLock & rMarkLock,       // i/o: Mark available for this Heal helper function
  SmHealerOpType     eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
#ifdef SM_DEBUG_CODE
    SmTArray<SmFaceProps*>& rFaceProps = m_sTgtFaceProps;

// GWC_SET_NEXT__TWO_LINES__TO_FALSE_BEFORE_RELEASE ;
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugThisFaceIndx = FALSE ; 
  if(bDebugMe)
    {
      Dump() ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // when Faces have LoopEdges crossing seams - split Edges at Seam
  if(m_sBadFaceProps_CrossedSeam.GetSize() > 0) 
    {        
      // locals
      SmTArray<SmFace *>   sThisNewFaces ;
      SmTArray<SmEdge *>   sThisNewEdgesFromSplits ;
      SmTArray<SmEdge *>   sSplitEdges, sThisSplitEdges ;
      SmTArray<SmVertex *> sThis_sNewVerticesOnSeams ;
      SmStatus sMergeResultU = SM_SUCCESS, sMergeResultV = SM_SUCCESS ; 

      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // init mark
      rMarkLock.NewMark() ; // increment the mark value - very cheap

      // for every Face with Edges that cross MissingSeams - Split the Edges at MissingSeam
      for(ULONG ii=m_sBadFaceProps_CrossedSeam.GetSize();ii>0;ii--)
        {
          // locals
          SmFaceProps * pThisFaceProps    = m_sBadFaceProps_CrossedSeam[ii-1] ;

          // A re-entrant heal step (the preceding Fix_MoveSeam via UpdateTargetLists->HealTgts, or an edge
          // split earlier in this loop) can free a FaceProps still queued here; skip it if it is no longer
          // in the live m_sTgtFaceProps. FindElement compares pointer identity only, so it is safe on a
          // dangling FaceProps pointer, whereas reading its members below would dereference freed memory.
          { ULONG lLiveIdx ;
            if(!m_sTgtFaceProps.FindElement(pThisFaceProps, lLiveIdx))
              { continue ; }
          }

          SmFace      * pThisFace         = (SmFace *)pThisFaceProps->m_pFace ;
          SmBoolean     bRebuildFaceProps = FALSE ; 

          // skip if the face itself was freed (pointer-identity check, safe on a dangling face pointer)
          if(pBrep != NULL && !pBrep->IsLiveTopologyMember(pThisFace))
            { continue ; }

          // skip faces without a LoopEdge crossing a Seam problem (some might be m_eBadMissingSeam or m_eNearMissSeams only faces)
          if(pThisFaceProps->m_bBadCrossedSeam == FALSE)
            { continue ; } 

          // Recompute this FaceProps when its cached seam classification (m_pCrvClassU/V, from an earlier
          // HasSeamProblem pass) holds a dangling topology pointer: the preceding Fix_MoveSeam (via
          // Fix_CoinVertices) can merge/free a vertex still referenced by the classification, and the
          // MergeIntervals/FindEdgeObjects calls below would dereference freed memory. Rebuilding against
          // current topology (the same ReSet+SetProps used for the already-split-edge case) refreshes the
          // classification with live pointers; if the recompute clears the crossed-seam problem, skip.
          if(   (pThisFaceProps->m_pCrvClassU != NULL && pThisFaceProps->m_pCrvClassU->HasDeadTopologyPointObject(pBrep))
             || (pThisFaceProps->m_pCrvClassV != NULL && pThisFaceProps->m_pCrvClassV->HasDeadTopologyPointObject(pBrep)))
            {
              pThisFaceProps->ReSet(SM_PROPSTAGE_2) ;
              SM_ASSERT_MSG(pThisFaceProps->m_lFaceIndx != SM_UNDEF_ULONG, _T("Face Indexing bug - needs review")) ;
              pThisFaceProps->SetProps(SM_PROPSTAGE_2, pThisFaceProps->m_pFace, pThisFaceProps->m_lFaceIndx, m_eDoneHealOp) ;
              if(pThisFaceProps->m_bBadCrossedSeam == FALSE)
                { continue ; }
            }

#ifdef SM_DEBUG_CODE
          if(bDebugMe || SmDebugThisFaceIndex(pThisFaceProps->m_lFaceIndx, bDebugThisFaceIndx))
            {
              ULONG lIndx = 0 ;
              ULONG dCnt  = rFaceProps.GetSize() ;
              for(ULONG di=0;di<dCnt;di++) { if(rFaceProps[di] == pThisFaceProps) { lIndx = 0 ; break ; } }
              pThisFaceProps->Dump(lIndx) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                               // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                               // sMissingSeamColor   - def:[ 1, 0, 0]
                                                               // sMissingPoleColor   - def:[ 0, 0, 1]
                                                               // sBadOrientLoopColor - def:[.3,.3,.3]
                                                               // sNoArea_LoopColor   - def:[ 0, 0, 0]
              pThisFaceProps->Dump(ii) ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // Use Marks to tell when an Edge has been split by another face - if so, rebuild FaceProps
          sSplitEdges.ReSet() ;

          // get edges crossing CurveClassification U
          if(   pThisFaceProps->m_eBadCrossedSeam == SM_SP_U
             || pThisFaceProps->m_eBadCrossedSeam == SM_SP_BOTH)
            {
              pThisFaceProps->m_pCrvClassU->FindEdgeObjects(sThisSplitEdges,NULL,2) ;  // 2=Get XSecting EdgeObjects from CrvInterval.m_vStart and m_vEnd
              sSplitEdges.Append(sThisSplitEdges) ;
            }

          // get edges crossing CurveClassification V
          if(   pThisFaceProps->m_eBadCrossedSeam == SM_SP_V
             || pThisFaceProps->m_eBadCrossedSeam == SM_SP_BOTH)
            {
              pThisFaceProps->m_pCrvClassV->FindEdgeObjects(sThisSplitEdges,NULL,2) ;  // 2=Get XSecting EdgeObjects from CrvInterval.m_vStart and m_vEnd
              sSplitEdges.Append(sThisSplitEdges) ;
            }

          // Mark these Split Edges - they are about to be split - remember if any have already been split
          for(ULONG jj=0;jj<sThisSplitEdges.GetSize();jj++)
            {
              // remember when an edge has already been split
              if(sThisSplitEdges[jj]->IsMarked(rMarkLock.GetMarkType()))
                { bRebuildFaceProps = TRUE ; }

              // Mark pBrep Edge - its about to get split
              sThisSplitEdges[jj]->Mark(rMarkLock.GetMarkType()) ; 

            } // end iter every Split Edge looking for edges split elsewhere by another face

          // when any edge about to be split has already been split - rebuild the FaceProps
          if(bRebuildFaceProps)
            { 
              pThisFaceProps->ReSet(SM_PROPSTAGE_2) ; 
              SM_ASSERT_MSG(pThisFaceProps->m_lFaceIndx != SM_UNDEF_ULONG, _T("Face Indexing bug - needs review")) ;
              pThisFaceProps->SetProps(SM_PROPSTAGE_2, pThisFaceProps->m_pFace, pThisFaceProps->m_lFaceIndx, m_eDoneHealOp) ; 
            }

          // Split Edges at Seam_U by merging Vertices only
          if(   pThisFaceProps->m_eBadCrossedSeam == SM_SP_U
             || pThisFaceProps->m_eBadCrossedSeam == SM_SP_BOTH)
            {
              // Split Edges crossing the U seam by merging the CrvClassificationU Interval EndPoints that cross the face
              sMergeResultU = pThisFaceProps->m_pCrvClassU->MergeIntervals
                       (&sThisNewFaces,             // out : list of new faces, NULL to ignore.                    
                                                    //       When an existing face is split into 2                 
                                                    //       the oldFace is reused and 1 newFace is                
                                                    //       created and placed on pBrep list.                      
                        &sThisNewEdgesFromSplits,   // out : list of new edges, NULL to ignore                     
                        &sThis_sNewVerticesOnSeams, // out : list of new vertices, NULL to ignore                  
                        pThisFace,                  // in  : pointer to original face - used for debug display only
             /* ===> */ TRUE) ;                     // in  : TRUE = Merge Vertices only                            
                                                    //       default:[FALSE] = Merge Vertices 1st and Edges 2nd               

#ifdef SM_DEBUG_CODE
static ULONG lSplitEdgeCnt = 0 ;
          if(bDebugMe || SmDebugThisFaceIndex(pThisFaceProps->m_lFaceIndx, bDebugThisFaceIndx))
                {
                  const SmContext * cpContext  =  pBrep->GetContext() ;
                  SmEdgeProps  sDbgEdgeProps(cpContext) ;

                  for(ULONG di=0;di<sThisNewEdgesFromSplits.GetSize();di++,lSplitEdgeCnt++)
                    {            
                      SmEdge     * pNewSplitEdge = sThisNewEdgesFromSplits[di] ; 
                      sDbgEdgeProps.SetProps(pNewSplitEdge, 0) ; // 0 is a dummy EdgeIndx value - NewEdges don't have an EdgeIndx yet
                      sDbgEdgeProps.Dump(lSplitEdgeCnt) ; 
                      if(sDbgEdgeProps.m_bBadUncontainedEdge == TRUE)
                        {
                          smgfx_Erase(TRUE) ;
                          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                          smgfx_SetLook(4,5, 1,0,0) ; sDbgEdgeProps.Draw() ; sm_GraphicsLoop() ;
                          sm_GraphicsLoop() ;
                        }
                    } // end iter new edges looking for problems
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

              // accumulate results
              SM_ASSERT(sThisNewFaces.GetSize() == 0) ; 
              m_sNewEdges_FromSplitEdgesAtSeams.Append(sThisNewEdgesFromSplits) ;
              m_sNewVertices_FromSplitEdgesAtSeams.Append(sThis_sNewVerticesOnSeams) ;

              // track the number of SplitEdge events
              m_sRanFaceProps_SplitEdgesAtSeam.Add(pThisFaceProps, SM_TRY_RAN) ;

            } // end Edges cross Seam_U check

          // Split Edges at Seam_V by merging Vertices only
          if(   pThisFaceProps->m_eBadCrossedSeam == SM_SP_V
             || pThisFaceProps->m_eBadCrossedSeam == SM_SP_BOTH)
            {
              // Split Edges crossing the V seam by merging the CrvClassificationV Interval EndPoints that cross the face
              sMergeResultV = pThisFaceProps->m_pCrvClassV->MergeIntervals
                       (&sThisNewFaces,           // out : list of new faces, NULL to ignore.                    
                                                  //       When an existing face is split into 2                 
                                                  //       the oldFace is reused and 1 newFace is                
                                                  //       created and placed on pBrep list.                      
                        &sThisNewEdgesFromSplits, // out : list of new edges, NULL to ignore                     
                        &sThis_sNewVerticesOnSeams, // out : list of new vertices, NULL to ignore                  
                        pThisFace,                // in  : pointer to original face - used for debug display only
             /* ===> */ TRUE) ;                   // in  : TRUE = Merge Vertices only                            
                                                  //       default:[FALSE] = Merge Vertices 1st and Edges 2nd               
              // accumulate results
              SM_ASSERT(sThisNewFaces.GetSize() == 0) ; 
              m_sNewEdges_FromSplitEdgesAtSeams.Append(sThisNewEdgesFromSplits) ;
              m_sNewVertices_FromSplitEdgesAtSeams.Append(sThis_sNewVerticesOnSeams) ;

              // track the faces run though Fix_SplitEdgesAtSeams()
              m_sRanFaceProps_SplitEdgesAtSeam.Add(pThisFaceProps, SM_TRY_RAN) ;

            } // end Edges cross Seam_V check
              
          // when successful - remember the change: set rFaceProps[lIndx].m_eBadCrossedSeam to SM_SP_NEITHER
          if(sMergeResultU == SM_SUCCESS && sMergeResultV == SM_SUCCESS)
            { 
              // changes were made - don't rerun the predicate (CrvClasses are already updated) just update the flag
              pThisFaceProps->m_eBadCrossedSeam = SM_SP_NEITHER ;

              // remember that this face has gone through SplitEdgeAtSeam and whether that fixed the face or not
              pThisFaceProps->m_eBeenThroughSplitEdgesAtSeam =  (   pThisFaceProps->m_bBadMissingSeam  == FALSE
                                                                 && pThisFaceProps->m_bBadCrossedSeam  == FALSE
                                                                 && pThisFaceProps->m_bBadNearMissSeam == FALSE)
                                                               ? SM_TRY_FIXED
                                                               : SM_TRY_NOFIX ;
      
              // gwc removed next lines: NearMiss probs will be fixed in future Fix_BadGaps() which refines Geom to tighter NearMiss into Exact connections
              // something for m_eBadNearMissSeam flag
              // GWC_NEEDS_WORK NEED_SOMETHING_FOR_NEAR_MISS_SEAM_VALUE_HERE ;
                
              // remember this face has been fixed
              pThisFaceProps->m_bBadCrossedSeam = FALSE ;
              pThisFaceProps->m_eBadCrossedSeam = SM_SP_NEITHER ;

            } // end SplitResult okay and changes were made check

#ifdef SM_DEBUG_CODE
          if(bDebugMe || SmDebugThisFaceIndex(pThisFaceProps->m_lFaceIndx, bDebugThisFaceIndx))
            {
              ULONG lIndx = SM_UNDEF_ULONG ; 
              ULONG dCnt = rFaceProps.GetSize() ;
              for(ULONG di=0;di<dCnt;di++) { if(rFaceProps[di] == pThisFaceProps) { lIndx = di%dCnt ; break ; } }
              if(lIndx != SM_UNDEF_ULONG) { pThisFaceProps->Dump(lIndx) ; }

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                               // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                               // sMissingSeamColor   - def:[ 1, 0, 0]
                                                               // sMissingPoleColor   - def:[ 0, 0, 1]
                                                               // sBadOrientLoopColor - def:[.3,.3,.3]
                                                               // sNoArea_LoopColor   - def:[ 0, 0, 0]
              pThisFaceProps->Dump(ii) ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

        } // end iter every face looking for problem Face->Loop->Edges that cross Seams to fix

      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_SPLITEDGE_ATSEAM)) ;

    } // end Faces exist with LoopEdges that cross seams check

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_SplitEdgesAtSeam

/*******************************************************************//**
PURPOSE: HealBrep helper function: 
         Fix BadFaceProps_Sheet problems

NOTES: Split Shells with SheetFaces that link two closed Shells together 
       by mistake
***********************************************************************/
SmStatus SmHealData::Fix_BadSheets
 (SmBrep           * pBrep,           // in : Tgt Brep being healed
  SmNewMarkAndLock & rMarkLock,       // i/o: Mark available for this Heal helper function
  SmHealerOpType     eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{                  
  SM_REF1(eThisHealerOp) ;
  // locals
  const SmContext * cpContext                 = pBrep->GetContext() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; for(di=0;di<m_sFaceProps_Sheets.GetSize();di++) 
                                    { SM_ASSERT_MSG(   m_sFaceProps_Sheets[di]->m_pFace->IsSheet() == TRUE,
                                                    _T("SmHealData::Fix_BadSheets: Face on m_sFaceProps_Sheets list not a sheet")) ;
                                      m_sFaceProps_Sheets[di]->m_pFace->DrawUV() ; sm_GraphicsLoop() ; 
                                    }
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // Heal_2: Find and Split Shells with SheetFaces that link two closed Shells together by mistake
  if(m_sFaceProps_Sheets.GetSize() > 0) 
    {
      // locals
      SmNewMarkAndLock sSheetMarkLock(cpContext) ; 
      SmMarkType       eSheetMarkType = sSheetMarkLock.GetMarkType() ;               

      SmTopologyTraverser  sTopoTraverse ;
      SM_ASSERT_MSG(rMarkLock.GetMarkType() != eSheetMarkType, _T("MakeTopologyFromData:: Failed to get two unique Marks - rewrite Mark Mechanism.")) ; 
      SmBoolean            bIsUpClosed = FALSE, bIsDownClosed = FALSE, bHasCommonElements = FALSE ;
      SmTArray<SmFaceuse*> sUpFaceuses, sDownFaceuses ;

      // check every SheetFace to see if a SheetFace is really a boundary between two shells (in linear time - use marks)
      for(ULONG ii=0;ii<m_sFaceProps_Sheets.GetSize();ii++)
        {
          // locals
          const SmFace * pLoopFace = m_sFaceProps_Sheets[ii]->m_pFace ; 
          SmFaceuse    * pUpFaceuse=NULL, * pDownFaceuse=NULL ; 
          pLoopFace->GetFaceuses(pUpFaceuse, pDownFaceuse) ; 

          // skip faces (after counting them) whose faceuses are already marked
          if(   pUpFaceuse->IsMarked(eSheetMarkType)
             || pDownFaceuse->IsMarked(eSheetMarkType))
            { 
              // mark this face as a bad FaceSheet
              m_sFaceProps_Sheets.GetAt(ii)->m_eSheet = pUpFaceuse->GetRegion() == pDownFaceuse->GetRegion()
                                                        ? SM_MP_OKAY
                                                        : SM_MP_FIXED ;
              continue ; 
            } // end Skip BadSheetFace count

          // check up and down faceuse shell closures
          sTopoTraverse.ClosureTraversal( pUpFaceuse,   bIsUpClosed,   &sUpFaceuses,   rMarkLock) ; // inc rMarkLock
          sTopoTraverse.ClosureTraversal( pDownFaceuse, bIsDownClosed, &sDownFaceuses, rMarkLock) ; // inc rMarkLock again

          // at this time the sMarkLock Marks on sUpFaceuses were cleared when the traversal on 
          // sDownFaceuses began.  Any faceuses now marked on the sUpFaceuses array are also 
          // present in the sDownFaceuses list.

          // locals
          SmShell   * pUpShell    = pUpFaceuse->GetShell() ;
          SmShell   * pDownShell  = pDownFaceuse->GetShell() ;
          // SmRegion  * pUpRegion   = pUpFaceuse->GetRegion() ;
          // SmRegion  * pDownRegion = pDownFaceuse->GetRegion() ;
          SmShell   * pNewShell   = NULL ;
          SmRegion  * pNewRegion  = NULL ; 

          // iter UpFaceuses looking for any faces also in DownFaceuses (in linear time)
          bHasCommonElements = FALSE ;
          for(ULONG jj=0;jj<sUpFaceuses.GetSize();jj++)
            {
              SmFaceuse * pFaceuse = sUpFaceuses[jj] ;

              if(pFaceuse->IsMarked(rMarkLock.GetMarkType()))
                { bHasCommonElements = TRUE ; 
                  break ;
                }
            } // end iter sUpFaceuses looking for marked members in both sUpFaceuses and sDownFaceuses

          // when a SheetFace is actually a boundary between different shells (a database error to fix)
          //  - fix it: make sure the Shells and Regions are properly sorted
          if(   bIsUpClosed                   //     pUpFaceuse   is part of a closed shell
             && bIsDownClosed                 // and pDownFaceuse is part of a closed shell
             && bHasCommonElements == FALSE   // and none of the UpShell Faceuses are also in the DownShell
             && pUpShell == pDownShell)       // and the UpShell is the same as the DownShell (a database error when bHasCommonElements is FALSE)
            {
              // // count the Bad SheetFaces
              // m_sBadFaceProps_Sheet.AddUnique(m_sFaceProps_Sheets[ii]) ;

              // fix - Split the Region on either side of the BadSheet into two different regions - increments mark again
              pBrep->TestClosureSplitRegion(TRUE,         // in : TRUE = find new infinite region by RayCasting
                                                          //      FALSE=
                                            pUpFaceuse,   // in : target Faceuse
                                            pNewShell,    // out: not NULL when target Faceuse splits region
                                            pNewRegion) ; // out: not NULL when target Faceuse splits region

              // count the TestClosureSplitRegion runs - could replace this with a SmTriedProbArray
              m_lRanRegion_TestClosure_Cnt++ ;

              // Mark the FaceProp Sheet problem
              m_sFaceProps_Sheets[ii]->m_eSheet =   pUpFaceuse->GetRegion() == pDownFaceuse->GetRegion()
                                                  ? SM_MP_OKAY
                                                  : SM_MP_FIXED ;
              
              // mark the Up and Down faceuses as already classified so that they can be skipped in the next iteration
              for(ULONG jj=0;jj<sUpFaceuses.GetSize();  jj++) { sUpFaceuses[jj]  ->Mark(eSheetMarkType) ; }
              for(ULONG jj=0;jj<sDownFaceuses.GetSize();jj++) { sDownFaceuses[jj]->Mark(eSheetMarkType) ; }

              // count the number of fixes due to the TestClosureSplitRegion runs
              if(pNewShell)
                { m_lFixRegion_SplitRegion_Cnt++ ;
                  m_sNewShells_FromClosedSheets.Add(pNewShell) ;
                }
              if(pNewRegion)
                { m_sNewRegions_FromClosedSheets.Add(pNewRegion) ; }

            } // end Found an improperly labeled SheetFace check
        } // end iter every SheetFace testing for missed regions
    } // end check SheetFaces exist check

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_BadSheets

/*******************************************************************//**
PURPOSE: HealBrep helper function: 
         Get FaceProps After Edges split at seams

NOTES: Now that Edges that cross seams have been split at those seams
 its possible to classify the loops of the face.

 For every Loop on Every Face being healed gather SmLoopProps data as:
    LoopProps: ClosedLoopCnt, 
               BadOrientLoopCnt(needs healing), NoArea_LoopCnt        (needs healing), 
               NestedLoopCnt   (needs healing), Unpaired_NoAreaLoopCnt(can't be healed)
    
// A NOTE ON LOOP ORIENTATIONS FOR LOOPS ON FACES WITH MISSING SEAMS
// when missing a seam: 
//   A loop can be Closed or Open.
//   A Closed loop can either be With or Without Area (called Area_Loops and NoArea_Loops). 
//   A Closed Area_Loop is a valid loop that partitions the UVDomain into 2 areas where
//     one of those areas will be isolated from the surface's natural UVBoundaries in the sense 
//     that a path starting at a point of the 'isolated' area will have to cross the loop boundary 
//     before exiting the UVDomain.
//   A Closed NoArea_Loop is an invalid loop only possible on a MissingSeamFace. This Loop will also 
//     partition the UVDomain into two areas but neither of those will be 'isolated' areas in that a path 
//     from any point in either area can always be constructed that exits the UVDomain without 
//     crossing the loop boundary.
//
// Observation: 1. An Area_Loop will cross the MissingSeam an even number of times  e.g.:[0,2,4,..]
//                 A NoArea_Loop will cross the MissingSeam an odd number of times. e.g.:[1,3,5,..]
//                 An inexpensive predicate for classifying a ClosedLoop on a Face with a MissingSeam
//                 as an Area_Loop or a NoArea_Loop can be built by counting the number of times the
//                 loop crosses the MissingSeam.
//              2. Example of a pair of NoArea_Loops being connected into proper CCW Area_Loops
//                 by Splitting the Face at the Seam.  In this case a NoArea_Loop with 3 SeamCrossings
//                 is connected to a NoArea_Loop with 1 SeamCrossing by merging two segments
//                 of the SeamCurve into the Face resulting in 2 New SeamEdges, 2 CCW Area_Loops, 
//                 and 2 Faces.
//
//                        Period 0         Period 1      Period 0         Period 1   
//                       Seam             Seam          Seam             Seam        
//                       +----------------+----...      +----------------+----...    
//                       |                |             |// UVRegion_C //|           
// Seam Crossings:[3]+---#<_            _<#<_           #<_////////////_<#<_         
//                    \  |  `o      _.<'  |  `o         |\\`o//////_.<'\\|\\`o       
//                     +-#>'    _.<'    _>#>'           #>'////_.<'\\\\_>#>'         
//                      \|  _.<'      o'  |             |//_.<' \\\\\<'  |           
//         NoArea_Loop2--#<'            `<#             #<'\\\\\\\\\\\\`<#--NoArea_Loop2            
//    [SM_OT_LOWERDOMAIN]|                |             |\\ UVRegion_B \\|    orient:[SM_OT_LOWERDOMAIN]        
//                       |                |             |\\\\\\\\\\\\\\\\|           
//         NoArea_Loop1--#-->--->--->--->-#->--...      #-->--->--->--->-#--NoArea_Loop1           
//    [SM_OT_UPPERDOMAIN]|                |             |////////////////|    orient:[SM_OT_UPPERDOMAIN]       
//                       |                |             |// UVRegion_A //|           
//                       +----------------+----...      +----------------+----...
//          NoArea_Loop2 Orientation:[SM_OT_LOWERDOMAIN]: RightHandDomain = UVRegion_C
//          NoArea_Loop2                                  LeftHandDomain  = UVRegion_B \_ Future interior of 
//          NoArea_Loop1 Orientation:[SM_OT_UPPERDOMAIN]: LeftHandDomain  = UVRegion_B /  Face_1 and Face_2
//          NoArea_Loop1                                  RightHandDomain = UVRegion_A    After merging MissingSeam
//
//     Add MissingSeam: Merge 2 SeamSegments as NewEdges, Split Face into 2 Faces. 
//                       
//                        Period 0         Period 1      Just the
//                       Seam             Seam             Seam
//                       +----------------+----...          +
//                       |     Outside    |                 |
//                       #<_            _<#<_               #--]
//               Face_2--$v `o      _.<' ^$v `o             $   }- New Seam Edge_2
//                       #>'    _.<'    _>#>' Face_2        #--]
//                       |  _.<'      o'  |                 |
//                       #<'            `<#                 #--]
//                       $v    Face_1    ^$                 $   }- New Seam Edge_1
//                       $v              ^$                 $  ]
//                       #-->--->--->--->-#->--...          #--]
//                       |                |                 |
//                       |     Outside    |                 |
//                       +----------------+----...          +
//                   
//           note: 1. The MissingSeam Curve is split into 5 segments 
//                    by the NoArea_Loop SeamCrossings.  Merging the 
//                    first inside Seam_Segment creates New Seam Edge_1 
//                    connecting NoArea_Loop_1 and NoArea_Loop_2 into 
//                    one CCW OuterLoop for Face_1. 
//                     (this Face_1 Loop still crosses the seam)
//           note: 2. Merging the second inside Seam_Segment creates new
//                    Seam Edge_2 which Splits the one CCW Loop into
//                    2 CCW Loops while also Spliting Face_1 into  
//                    Face_1 and Face_2. 
//                     (Face_1 and Face_2 Loops do not cross the seam)
//
// Computing the desired orientation of a loop on a face without a MissingSeam is easy.
// The OuterLoop must be oriented CCW, and all InnerLoops must be oriented CW.
//
// The desired orientation of a loop on a face with a MissingSeam is more complicated.
// If the Loop is a Closed Area_Loop, then it follows the same orientation rules as for
// any loop on a valid Face; the OuterLoop is CCW and the InnerLoops are CW.
//
// A pair of invalid Closed NoArea_Loops can be turned into one valid Area_Loop CCW outerloop
// by adding the MisingSeamEdge to the Face.  For another example, consider a cylinder bounded 
// by upper and lower circles but missing the seam.  Both circles will be 
// Single-SeamCrossing NoArea-Loops .  When their orientations are consistent with one another 
// together they can define a UVDomain SubRegion that is 'isolated' in that a path starting at 
// any point in the 'isolated' region will have to cross one of the two NoArea-Loops before 
// exiting the UVDomain (crossing the seam does not count as exiting the domain).  
// Adding the MissingSeam will connect this matched pair of NoArea_Loops into a single valid 
// CCW OuterLoop. (Observation: any Loop that includes a SeamEdge must be an OuterLoop.)
//
// We can set the orientations of closed NoArea_Loops so that when they are connected 
// by adding a missing seam later they will create a properly oriented OuterLoop. Each 
// Single SeamCrossing NoArea-Loop divides the UVDomain into an 'upper' and a 'lower' half.  
// Using the Edgeuse's LeftHand rule for 'inside' and the UV parameterization of the domain to 
// name the two sides we define the following for Single SeamCrossing NoArea-Loop orientations:
//   Rule: a SM_OT_SAME oriented NoArea-Loop's 'Lefthand-Inside' = 'upper' UV Partition   
//         a SM_OT_OPPOSITE oriented NoArea-Loop's 'LeftHand-Inside' = 'lower' UV Partition.
//
// If we order theNoArea_Loops of a Face with a Missing seam from lowest partition
// param value to highest, we can see the requirements for NoArea-Loop orientations
// so that an addition of the MissingSeam can connect those into valid OuterLoops.
//   rules:  1. NoArea_Loops must come in pairs. (a VertexLoop pole on a seam is a NoArea_Loop IsoLine in UV space)
//           2. NoArea_Loops must cross the MissingSeam.
//           3. When Single SeamCrossing NoArea_Loops come in pairs (the common case)
//             4a. The Lowest  param NoArea_Loop member of each pair must be oriented SM_OT_SAME
//             4b. The Highest param NoArea_Loop member of each pair must be oriented SM_OT_OPPOSITE
//             4c. The number of outer loops created by adding the MissingSeam to a Face with a pair
//                 of properly oriented NoArea_Loops will equal the number of SeamCrossings/2. 
//                 CCW OuterLoopCount After Adding SeamEdges to a pair of NoArea_Loops = SeamCrossings/2
//
// A common read problem is a Face on a disk modeled as a periodic surface with a singularity
// where both the Seam and the Pole are missing.  In these Face models its common to have
// just one NoArea-Loop.  In this case we add a Vertex at the pole which will become the
// second NoArea-Loop.  After which, adding the MissingSeam connects the two NoArea-Loops
// into a valid single OuterLoop.  The desired orientation of the one NoAreaLoop depends on
// the parameter value for the MissingPole.  If the Pole is at UVMin, then the orientation 
// needs to be SM_OT_OPPOSITE, and conversely if the Pole is at UVMax the orientation needs
// to be SM_OT_SAME.  
//
// Another common read problem is a Face on a periodic Surface with no singularities but just
// one NoArea-Loop.  There is no way to turn that loop into a valid OuterLoop by just adding
// a MissingSeam.  There are two options available for fixing the problem.  The Face might
// degenerate to a single Edge consisting of the NoArea-Loop.  Or the Face may need the addition
// of another NoArea-Loop.  There is no 'right' way to add that missing NoArea-Loop without
// changing the geometry and the topology of model.  If the part being read is intended to be
// manifold, adding the missing NoArea-Loop might require the creation and addition of another
// Face.  We will leave this problem for a later time. (perhaps that time is now for you!)
***********************************************************************/
SmStatus SmHealData::Cache_FaceProps_Stage3
 (SmBrep         * pBrep,         // in : Tgt Brep being healed
  SmBoolean      * pbNewProbs,    // out: TRUE  = Found new problems while caching data
                                  //      FALSE = no problems found
  SmHealerOpType   eThisHealerOp) // NotUsed: in : This function's Step Number within the healer step sequence
{                 
  SM_REF1(eThisHealerOp) ;
  // init output
  *pbNewProbs = FALSE ;

  // no work - no Faces or no new Faces
  if(   m_sTgtFaceProps.GetSize() == 0
     || m_sTgtFaceProps.GetSize() == m_lOldFaceCnt)
    { return SM_SUCCESS ; }

  // locals
  ULONG ii ;
  SmTArray<SmFaceProps *> & rFaceProps =  m_sTgtFaceProps ;
 // const SmContext         * cpContext  =  pBrep->GetContext() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // Get AfterSplitEdges FaceProperties - run iter backwards because a face might be removed from list - (m_lOldFaceCnt != 0 when updating TgtLists with new objs)
  for(ii=rFaceProps.GetSize();ii>m_lOldFaceCnt;ii--)
    {
      // set m_pFace and compute:[XSectTol3d, bSheet, bClosed, lPoles, lFlatCorners]
      SmFaceProps * pThisFaceProps = rFaceProps[ii-1] ;
 //    pThisFaceProps->SetContext(cpContext) ;        // not needed - set in SmHealData::Cache_FaceProps_Gaps()
        
      // when needed - compute FaceProps After Fix_SplitEdgesAtSeam() - SM_PROPSTAGE_3
      if(FALSE == pThisFaceProps->HasProps(SM_PROPSTAGE_3, pThisFaceProps->m_pFace))          
        { 
          // AfterEdgeSplit SetProps 
          SM_ASSERT_MSG(pThisFaceProps->m_lFaceIndx != SM_UNDEF_ULONG, _T("Face Indexing bug - needs review")) ;
          SmStatus sRtn = pThisFaceProps->SetProps(SM_PROPSTAGE_3, pThisFaceProps->m_pFace, pThisFaceProps->m_lFaceIndx, m_eDoneHealOp, ii-1) ; 
                  //  sets pThisFaceProps: m_sLoopProps
                  //                       m_lClosedLoopCnt      m_lOuterLoopCnt          
                  //                       m_bBadOuterLoopOrder  m_lBadOrient_LoopCnt     
                  //                       m_lBadNoArea_LoopCnt  m_lBadNested_LoopCnt     
                  //                       m_lBadMissingPoles    m_bBadUnpairedNoArea_Loop

          // When Face has NotYet supported problems
          if(sRtn == SM_ERR_NOTYET_HEAL_FACE)
            {
              // isolate the face from the heal sequence - move it to pHealData->m_sNotYetFaceProps
              MoveToNotYetProblems(pThisFaceProps) ;  

              // skip any further processing on this FaceProps
              continue ;
            } // end found a yet to be supported face problem check 
        } // end need to SetProps_Stage3 check

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          SmTArray<SmLoop*> sLoops ; 
          pThisFaceProps->m_pFace->GetLoops(sLoops) ; 

          pThisFaceProps->Dump(ii-1) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                       // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                       // sMissingSeamColor   - def:[ 1, 0, 0]
                                                       // sMissingPoleColor   - def:[ 0, 0, 1]
                                                       // sBadOrientLoopColor - def:[.3,.3,.3]
                                                       // sNoArea_LoopColor   - def:[ 0, 0, 0]
          smgfx_SetLook(3,4, .5,.5,1) ; for(ULONG di=0;di<sLoops.GetSize();di++)
                                          { if(sLoops[di]) { sLoops[di]->Draw() ; } sm_GraphicsLoop() ; 
                                            if(FALSE) 
                                              { sLoops[di]->Dump() ; } 
                                          }
          sm_GraphicsLoop() ;  
        }
#endif // SM_DEBUG_CODE

      // count the kinds of faces that force healing checks later on
      SmBoolean bHasLoopProb = FALSE ;

      // BadOuterLoopOrder Face List
      if(pThisFaceProps->m_bBadOuterLoopOrder > 0)
        { 
          m_sBadFaceProps_OuterLoopOrder.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadOrient_Loop Face List
      if(pThisFaceProps->m_lBadOrient_LoopCnt > 0)
        { 
          m_sBadFaceProps_OrientLoop.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadNoArea_Loop Face List
      if(pThisFaceProps->m_lBadNoArea_LoopCnt > 0)
        {
#ifdef SM_DEBUG_CODE
          SmBoolean bIsIn = m_sBadFaceProps_MissingSeam.m_sProbArray.IsIn(pThisFaceProps) ;
          SM_ASSERT_MSG(bIsIn, 
                        _T("SmBrep::HealBrep - Found a FaceProps with NoArea_Loops not on the MissingSeams list - a Bug")) ; 
#endif // SM_DEBUG_CODE
          m_sBadFaceProps_NoArea_Loop.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadMultiOuterLoop and/or BadNested_Loop Face List
      if(pThisFaceProps->m_lOuterLoopCnt > 1)
        {
          m_sBadFaceProps_MultiOuterLoops.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      if(   pThisFaceProps->m_lBadNested_LoopCnt > 0)
        { 
          m_sBadFaceProps_NestedOuterLoops.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadMissingPole Face List
      if(pThisFaceProps->m_lBadMissingPoles != SM_SS_NONE)
        { 
          m_sBadFaceProps_MissingPole.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadUnpairedNoArea_Loop Face List
      if(pThisFaceProps->m_bBadUnpairedNoArea_Loop == TRUE)
        { 
          m_sBadFaceProps_UnpairedNoAreaLoop.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadClosed3d_Loop - EdgeEnd/EdgeEnd Gaps too big
      if(pThisFaceProps->m_lBadClosed3d_LoopCnt > 0)
        { 
          m_sBadFaceProps_Closed3dLoop.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // BadClosedPtr_Loop - Edgeuse->CCW Linked Ptr list is not closed when all EdgeEnd/EdgeEnd gaps are within tol
      if(pThisFaceProps->m_lBadClosedPtr_LoopCnt > 0)
        { 
          m_sBadFaceProps_ClosedPtrLoop.Add(pThisFaceProps) ; 
          bHasLoopProb = TRUE ; 
        }

      // accumulate list of All FaceProps with Any LoopProblems  
      if(bHasLoopProb)
        {
          m_sBadFaceProps_LoopProblems.Add(pThisFaceProps) ;
        }

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          SmTArray<SmLoop*> sLoops ; 
          pThisFaceProps->m_pFace->GetLoops(sLoops) ; 

          pThisFaceProps->Dump(ii-1) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                       // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                       // sMissingSeamColor   - def:[ 1, 0, 0]
                                                       // sMissingPoleColor   - def:[ 0, 0, 1]
                                                       // sBadOrientLoopColor - def:[.3,.3,.3]
                                                       // sNoArea_LoopColor   - def:[ 0, 0, 0]
          smgfx_SetLook(3,4, .5,.5,1) ; for(ULONG di=0;di<sLoops.GetSize();di++)
                                          { if(sLoops[di]) { sLoops[di]->Draw() ; } sm_GraphicsLoop() ; 
                                            if(FALSE) 
                                              { sLoops[di]->Dump() ; } 
                                          }
          sm_GraphicsLoop() ;  
        }
#endif // SM_DEBUG_CODE

    } // end iter every Face - computing and saving AfterEdgeSplit face classification properties

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Cache_FaceProps_Stage3

// begin obsolete
//  /*******************************************************************//**
//  PURPOSE: Helper function for Fix_BadLoops updates input FaceProps
//  
//  NOTES: Input a FaceProps after the Face was modified by a Fix_BadLoops heal function.
//      This method will update the FaceProps and associated HealData lists
//  ***********************************************************************/
//  SmStatus sm_UpdateFaceProps
//   (SmHealData    & rHealData,          // i/o: Current HealData
//    SmFaceProps   * pChangedFaceProps,  // i/o: FaceProps of Face that was healed
//    SmHealerOpType  eThisHealerOp)      // in : 
//  {
//    SmFace        * pThisFace = (SmFace *) pChangedFaceProps->m_pFace ;
//  
//    rHealData.m_sBadFaceProps_OrientLoop.Remove( pChangedFaceProps ) ;
//    rHealData.m_sBadFaceProps_MissingPole.Remove( pChangedFaceProps ) ;
//    rHealData.m_sBadFaceProps_OuterLoopOrder.Remove( pChangedFaceProps ) ;
//    rHealData.m_sBadFaceProps_MultiOuterLoops.Remove( pChangedFaceProps ) ;
//    rHealData.m_sBadFaceProps_NestedOuterLoops.Remove( pChangedFaceProps ) ;
//  
//#ifdef SM_DEBUG_CODE
//    SmBoolean bDebugMe = FALSE ;
//    if ( bDebugMe )
//      {
//        SmBrep             *pBrep = rHealData.m_pBrep ;
//        ULONG               di ;
//        SmTArray< SmLoop* > sDBGLoops ;
//        pChangedFaceProps->Dump() ;
//        rHealData.DumpFaceListsContaining( pChangedFaceProps ) ;
//  
//        pThisFace->GetLoops( sDBGLoops ) ;
//  
//        smgfx_Erase(TRUE);
//        smgfx_SetLook( 1, 2, 0, 0, 1 ) ; if ( pBrep ) pBrep->Draw( TRUE ) ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 1, 2, 0, 0, 1 ) ; if ( pChangedFaceProps ) pChangedFaceProps->Draw() ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 3, 4, 1, 0, 0 ) ; if ( pThisFace ) pThisFace->GetSurface()->DrawSingularities() ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 2, 3, .3, .3, .3 ) ; if ( pThisFace ) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 2, 3, 0, 1, 0 ) ; for (di=0;di<sDBGLoops.GetSize();di++)
//        { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
//        sm_GraphicsLoop() ;
//      }
//#endif // SM_DEBUG_CODE
//  
//    // Refresh pThisFaceProps - faces with no problems get removed from their prob arrays
//    rHealData.RefreshFaceProps( pChangedFaceProps, pChangedFaceProps->m_lFaceIndx, eThisHealerOp ) ;
//  
//    // We expect all these problems to be fixed - but check anyway and rebuild problem lists as needed
//    //  - these are healer bugs and smsHeal_Fix_BadLoops() needs to be extended
//    if (pChangedFaceProps->m_lBadMissingPoles != SM_SS_NONE) { rHealData.m_sBadFaceProps_MissingPole     .Add(pChangedFaceProps) ; }
//    if (pChangedFaceProps->m_lBadOrient_LoopCnt > 0)         { rHealData.m_sBadFaceProps_OrientLoop      .Add(pChangedFaceProps) ; }
//    if (pChangedFaceProps->m_bBadOuterLoopOrder == TRUE)     { rHealData.m_sBadFaceProps_OuterLoopOrder  .Add(pChangedFaceProps) ; }
//    if (pChangedFaceProps->m_lOuterLoopCnt > 1)              { rHealData.m_sBadFaceProps_MultiOuterLoops .Add(pChangedFaceProps) ; }
//    if (pChangedFaceProps->m_lBadNested_LoopCnt > 0)         { rHealData.m_sBadFaceProps_NestedOuterLoops.Add(pChangedFaceProps) ; }
//  
//#ifdef SM_DEBUG_CODE
//    if ( !( (pChangedFaceProps->m_lBadMissingPoles == SM_SS_NONE)
//         && (pChangedFaceProps->m_lBadOrient_LoopCnt == 0)
//         && (pChangedFaceProps->m_bBadOuterLoopOrder == FALSE)
//         && (pChangedFaceProps->m_lBadNested_LoopCnt == 0)
//         && (pChangedFaceProps->m_lOuterLoopCnt == 1)))
//      {
//        SM_ASSERT_MSG(    (pChangedFaceProps->m_lBadMissingPoles == SM_SS_NONE)
//                       && (pChangedFaceProps->m_lBadOrient_LoopCnt == 0)
//                       && (pChangedFaceProps->m_bBadOuterLoopOrder == FALSE)
//                       && (pChangedFaceProps->m_lBadNested_LoopCnt == 0)
//                       && (pChangedFaceProps->m_lOuterLoopCnt > 1),
//                       _T( "smsHeal_Fix_BadLoops: Failed to fix a problem its intended to fix - prob case needs review and this function needs to be extended." ) ) ;
//      }
//#endif // SM_DEBUG_CODE
//  
//    // when there are any cascading fixes - remove those Faces from their Prob arrays (Yay!)
//    if(pChangedFaceProps->m_lBadNoArea_LoopCnt == 0)          { rHealData.m_sBadFaceProps_NoArea_Loop       .Remove(pChangedFaceProps) ; }
//    if(pChangedFaceProps->m_bBadUnpairedNoArea_Loop == FALSE) { rHealData.m_sBadFaceProps_UnpairedNoAreaLoop.Remove(pChangedFaceProps) ; }
//    if(pChangedFaceProps->m_lBadClosed3d_LoopCnt == 0)        { rHealData.m_sBadFaceProps_Closed3dLoop      .Remove(pChangedFaceProps) ; }
//    if(pChangedFaceProps->m_lBadClosedPtr_LoopCnt == 0)       { rHealData.m_sBadFaceProps_ClosedPtrLoop     .Remove(pChangedFaceProps) ; }
//  
//    // if the face has no other Loop based problems - remove it from the m_sBadFaceProps_LoopProblems problem array
//    if ( FALSE == pChangedFaceProps->HasProblems( 4 ) ) // 4 = check AfterEdgeSplit Problems (= LoopProblems) only
//      {
//        rHealData.m_sBadFaceProps_LoopProblems.Remove( pChangedFaceProps ) ;
//      }
//  
//#ifdef SM_DEBUG_CODE
//    if ( bDebugMe )
//      {
//        SmBrep             *pBrep = rHealData.m_pBrep;
//        ULONG               di;
//        SmTArray< SmLoop* > sDBGLoops;
//  
//        pChangedFaceProps->Dump( ) ;
//        pThisFace->GetLoops( sDBGLoops ) ;
//  
//        smgfx_Erase(TRUE) ;
//        smgfx_SetLook( 1, 2, 0, 0, 1 ) ; if ( pBrep ) pBrep->Draw( TRUE ) ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 1, 2, 0, 0, 1 ) ; if ( pChangedFaceProps ) pChangedFaceProps->Draw() ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 3, 4, 1, 0, 0 ) ; if ( pThisFace ) pThisFace->GetSurface()->DrawSingularities() ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 2, 3, .3, .3, .3 ) ; if ( pThisFace ) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
//        smgfx_SetLook( 2, 3, 0, 1, 0 ) ; for ( di = 0; di < sDBGLoops.GetSize() ; di++ )
//        { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
//        sm_GraphicsLoop() ;
//    }
//#endif // SM_DEBUG_CODE
//  
//    return SM_SUCCESS;
//  } // end sm_UpdateFaceProps
// end obsolete

/*******************************************************************//**
PURPOSE: HealBrep helper function: Fix MissingSeamEdges

NOTES: Split Faces at Missing Seam Edges.  This adds new Edges along the
  SeamIvls that are inside the face causing one or more Face splits creating
  new Child Faces.
***********************************************************************/
SmStatus SmHealData::Fix_SplitFaceAtSeams
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
  // locals
  ULONG ii ;

#ifdef SM_DEBUG_CODE
ULONG di ;
// GWC_SET_NEXT__TWO_LINES__TO_FALSE_BEFORE_RELEASE ;
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugThisFaceIndx = FALSE ; 
  if(bDebugMe)
    {
      Dump() ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(di=0;di<m_sBadFaceProps_MissingSeam.GetSize();di++)
        { m_sBadFaceProps_CrossedSeam[di]->Draw(TRUE) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ; 
    }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // When Faces are crossing a Seam
  if(m_sBadFaceProps_MissingSeam.GetSize() > 0)
    {
      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't      SmTArray<SmFace *> sThisNewFaces ;
      SmTArray<SmFace *> sThisNewFaces ;

      // for every face with a MissingSeam
      for(ii=m_sBadFaceProps_MissingSeam.GetSize();ii>0;ii--)
        {
          SmFaceProps * pThisFaceProps = m_sBadFaceProps_MissingSeam[ii-1] ;

          // A re-entrant heal step (the preceding Fix_MoveSeam/Fix_SplitEdgesAtSeam via
          // UpdateTargetLists->HealTgts, or a face split earlier in this loop) can free a FaceProps still
          // queued here; skip it if it is no longer in the live m_sTgtFaceProps. FindElement compares
          // pointer identity only, so it is safe on a dangling FaceProps pointer.
          { ULONG lLiveIdx ;
            if(!m_sTgtFaceProps.FindElement(pThisFaceProps, lLiveIdx))
              { continue ; }
          }

          SmFace      * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ; 
          sThisNewFaces.ReSet() ;

          // skip if the face itself was freed (pointer-identity check, safe on a dangling face pointer)
          if(pBrep != NULL && !pBrep->IsLiveTopologyMember(pThisFace))
            { continue ; }

          // skip Faces without MissingSeams
          if(pThisFaceProps->m_bBadMissingSeam == FALSE)
            { continue ; }

          // Recompute this FaceProps when its cached seam classification (m_pCrvClassU/V, from an earlier
          // HasSeamProblem pass) holds a dangling topology pointer: the preceding Fix_MoveSeam (via
          // Fix_CoinVertices) can merge/free a vertex still referenced by the classification, and the
          // SplitAtSeam call below would dereference freed memory. Rebuild against current topology so the
          // classification carries live pointers; if the recompute clears the missing-seam problem, skip.
          if(   (pThisFaceProps->m_pCrvClassU != NULL && pThisFaceProps->m_pCrvClassU->HasDeadTopologyPointObject(pBrep))
             || (pThisFaceProps->m_pCrvClassV != NULL && pThisFaceProps->m_pCrvClassV->HasDeadTopologyPointObject(pBrep)))
            {
              pThisFaceProps->ReSet(SM_PROPSTAGE_2) ;
              SM_ASSERT_MSG(pThisFaceProps->m_lFaceIndx != SM_UNDEF_ULONG, _T("Face Indexing bug - needs review")) ;
              pThisFaceProps->SetProps(SM_PROPSTAGE_2, pThisFaceProps->m_pFace, pThisFaceProps->m_lFaceIndx, m_eDoneHealOp) ;
              if(pThisFaceProps->m_bBadMissingSeam == FALSE)
                { continue ; }
            }

#ifdef SM_DEBUG_CODE
      if(bDebugMe || SmDebugThisFaceIndex(pThisFaceProps->m_lFaceIndx, bDebugThisFaceIndx))
        {
          pThisFaceProps->Dump(ii) ;

          smgfx_Erase(TRUE) ;
          smgfx_SetLook(1,2, 0,0,1) ;    if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
          pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                       // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                       // sMissingSeamColor   - def:[ 1, 0, 0]
                                                       // sMissingPoleColor   - def:[ 0, 0, 1]
                                                       // sBadOrientLoopColor - def:[.3,.3,.3]
                                                       // sNoArea_LoopColor   - def:[ 0, 0, 0]
          sm_GraphicsLoop() ; 
        }
#endif // SM_DEBUG_CODE
                                                  
          // Insert the MissingSeam Edge segments into the Face - may split a face one or more times
          pThisFace->SplitAtSeam(*pThisFaceProps->m_pCrvClassU,      // in : ConstU_Seam from HasSeamProblem() output(bTgtLoopsOnly==FALSE), NULL = execute HasSeamProblem() in pBrep call 
                                 *pThisFaceProps->m_pCrvClassV,      // in : ConstV_Seam from HasSeamProblem() output(bTgtLoopsOnly==FALSE), NULL = execute HasSeamProblem() in pBrep call 
                                 pThisFaceProps->m_eBadCrossedSeam,  // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL 
                                                                     //    : SM_SP_U       = periodic U bndry seam crossed by Face->Edges                 
                                                                     //    : SM_SP_V       = periodic V bndry seam crossed by Face->Edges                
                                                                     //    : SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges     
                                                                     //    : SM_SP_NEITHER = no seams crossed by Face->Edges                              
                                 pThisFaceProps->m_eBadMissingSeam,  // in : from HasSeamProblem() call output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL   
                                                                     //    : SM_SP_U       = periodic U bndry seam not represented by Face->Edges          
                                                                     //    : SM_SP_V       = periodic V bndry seam not represented by Face->Edges            
                                                                     //    : SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges 
                                                                     //    : SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges 
                                 pThisFaceProps->m_eBadNearMissSeam, // in : from HasSeamProblem() call output(bTgtLoopsOnly==FALSE),for future use one Fix_BadGaps() gets built  
                                                                     //    : SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
                                                                     //    : SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
                                                                     //    : SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
                                                                     //    : SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams 
                                                                     //    : NearMiss Topo connections are valid - future Fix_BadGaps() will refine geom to tighten gaps from NearMiss to Exact
                                &sThisNewFaces,                      // out: NewFaces made by splitting if any.  (does not include this face)    
                                 NULL,                               // out: Edges Added at the Seam if any.  
                                 NULL,                               // out: Vertices Added at the Seam if any.
                                 pThisFaceProps) ;                   // in : optional pre-cached set of this-Face properites, NULL to ignore
                              
          // when Face is split - pThisFaceProps is now obsolete and should be on the sTopologyChanges RmList
          SM_ASSERT_MSG(sTopologyChanges.IsInRmList(pThisFace) == TRUE, _T("SmHealData::Fix_SplitFaceAtSeams bad assumption: ChangeTracker RmList does not contain split Face - needs debug")) ;

          // accumulate outputs
          if(sThisNewFaces.GetSize() > 0) { m_sRanFaceProps_SplitEdgesAtSeam.IncOrigProbCount() ; }
          else                            { m_sRanFaceProps_SplitEdgesAtSeam.Add(pThisFaceProps, SM_TRY_NOFIX) ; }

        } // end iter every Face with MissingSeam

      // remember the set of new geometry made in this function
      SmTArray<SmTopology *> *pAddList = sTopologyChanges.GetAddList() ;
      for(ii=0;ii<pAddList->GetSize();ii++)
        { if     (pAddList->GetAt(ii)->IsKindOf(SmFace_TYPE))   { m_sNewFaces_FromSplitFaceAtSeams.       Add( (SmFace *)  pAddList->GetAt(ii) ) ; }
          else if(pAddList->GetAt(ii)->IsKindOf(SmEdge_TYPE))   { m_sNewEdgesOnSeams_FromSplitFaceAtSeams.Add( (SmEdge *)  pAddList->GetAt(ii) ) ; }
          else if(pAddList->GetAt(ii)->IsKindOf(SmVertex_TYPE)) { m_sNewVertices_FromSplitFaceAtSeams.    Add( (SmVertex *)pAddList->GetAt(ii) ) ; }
        } 
          
      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_SPLITEDGE_ATSEAM)) ;

    } // end Faces with MisingSeam existence check

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_SplitFaceAtSeams

/*******************************************************************//**
PURPOSE: HealBrep helper function: Fix BadLoop problems

NOTES: Heals known Loop problems in order as:
       1. Bad UnpairedNoAreaLoop Faces                 - Look for Surface pairs sharing a duplicated surface and merge them
       2. Bad Missing PoleVertex problems              - Add Vertex at Pole
       3. Bad Loop Orientations                        - Flip Loop Orientation: SmLoop::FlipLoopOrientation()
       4. Bad Face LoopList Orders                     - Place 1st OuterLoop at head of Faceuse lists
       5. Bad MulipleOuterLoop and NestedLoop Problems - SplitFace to make each OuterLoop and Each InnerLoop an OuterLoop on its own face
       6. Bad NoAreaLoop_Pair problems are not fixed here.  They are fixed later by SmFace::SplitAtSeam()

       For every Face with BadLoopProblems, calls recursive function, 
       SmHealData::Fix_BadLoops() to walk the Loop tree and fix all known
       problems
***********************************************************************/
SmStatus SmHealData::Fix_BadLoops
 (SmBrep           * pBrep,           // in : Tgt Brep being healed
  SmNewMarkAndLock & rMarkLock,       // i/o: Mark available for this Heal helper function
  SmHealerOpType     eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
  // locals
  ULONG ii, jj ;
  SmTArray<SmFace *> sNewFaces ;
                          
#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE ;
SmTArray<SmLoop *> sDBGLoops ;
#endif // SM_DEBUG_CODE

// Let's wait to see if this special case is still needed - perhaps the classify loops rewrite was enough
//    // GWC: I think this was a work around made for a failing case - ClassifyLoops has been updated - should not be a failing case
//    // if this block of code has to go back in the function it needs to be updated to the new healer architecture
//    static SmBoolean bDoMissingSeamFix = TRUE;
//      if(bDoMissingSeamFix)
//        { // begin Scope: Face contains missing seam edge
//    
//          // for every Face with two outer loops with orientation problems and a missing seam
//          //   - imprint missing seam, then refresh FaceProps
//    
//          for(ii=0;ii<m_sBadFaceProps_OrientLoop.GetSize();ii++)
//            {
//              // locals
//              SmFaceProps * pThisFaceProps = m_sBadFaceProps_OrientLoop[ii];
//              SmFace      * pThisFace = (SmFace *) pThisFaceProps->m_pFace;
//              SmBoolean     bMadeChange = FALSE;
//              SmBoolean     bMisorientedTopLoop = FALSE;
//              SmBoolean     bMisorientedBotLoop = FALSE;
//    
//              // If the face has a MissingSeam, then do the standard healing step Fix_SplitAtSeam
//              if ( pThisFaceProps->m_eBadMissingSeam == SM_SP_NEITHER )
//                { continue; }
//    
//  #ifdef SM_DEBUG_CODE
//              if ( bDebugMe )
//                {
//                  pThisFaceProps->Dump( ii );
//                  pThisFace->GetLoops( sDBGLoops );
//    
//                  smgfx_Erase(TRUE);
//                  smgfx_SetLook( 1, 2, 0, 0, 1 ); if ( pBrep ) pBrep->Draw( TRUE ); sm_GraphicsLoop();
//                  smgfx_SetLook( 1, 2, 0, 0, 1 ); if ( pThisFaceProps ) pThisFaceProps->Draw(); sm_GraphicsLoop();
//                  smgfx_SetLook( 3, 4, 1, 0, 0 ); if ( pThisFace ) pThisFace->GetSurface()->DrawSingularities(); sm_GraphicsLoop();
//                  smgfx_SetLook( 2, 3, .3, .3, .3 ); if ( pThisFace ) pThisFace->GetSurface()->DrawSeams(); sm_GraphicsLoop();
//                  smgfx_SetLook( 2, 3, 0, 1, 0 ); for ( di = 0; di < sDBGLoops.GetSize(); di++ )
//                  { sDBGLoops[di]->GetLoopuse()->Draw(); sm_GraphicsLoop(); }
//                  sm_GraphicsLoop();
//                }
//  #endif // SM_DEBUG_CODE
//    
//              // look for a misoriented pair of loops. They may be misclassified, due to missing seams
//              for(jj=0;jj<pThisFaceProps->m_sLoopProps.GetSize() && !(bMisorientedBotLoop && bMisorientedTopLoop); ++jj )
//                {
//                  SmLoopProps * pThisLoopProps = &(pThisFaceProps->m_sLoopProps[jj]) ;
//    
//                  if(pThisLoopProps->m_bGoodOrient == TRUE)
//                     { continue ; }
//    
//                  bMisorientedBotLoop |= pThisLoopProps->m_eContainmentType == SM_CMT_BOTLOOP ;
//                  bMisorientedTopLoop |= pThisLoopProps->m_eContainmentType == SM_CMT_TOPLOOP ;
//                }
//    
//              // If both BotLoop and TopLoop are badly oriented, insert seam and refresh FaceProps. 
//              // This will find if loops are okay, but being misclassified
//              if(bMisorientedBotLoop && bMisorientedTopLoop)
//                {
//                  SmTArray<SmFace *>     sThisNewFaces ;
//                  SmTArray<SmEdge *>     sThisNewSeamEdges ;
//                  SmTArray<SmVertex *>   sThisNewSeamVertices ;
//                  SmTArray<SmTopology *> sThisAddObjs, sThisRemoveObjs ;
//    
//                  // Insert the MissingSeam Edge segments into the Face - may split a face one or more times
//                  pThisFace->SplitAtSeam(*pThisFaceProps->m_pCrvClassU,      // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), NULL = execute HasSeamProblem() in pBrep call 
//                                         *pThisFaceProps->m_pCrvClassV,      // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), NULL = execute HasSeamProblem() in pBrep call 
//                                         pThisFaceProps->m_eBadCrossedSeam,  // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL 
//                                                                             //    : SM_SP_U       = periodic U bndry seam crossed by Face->Edges                 
//                                                                             //    : SM_SP_V       = periodic V bndry seam crossed by Face->Edges                
//                                                                             //    : SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges     
//                                                                             //    : SM_SP_NEITHER = no seams crossed by Face->Edges                              
//                                         pThisFaceProps->m_eBadMissingSeam,  // in : from HasSeamProblem() call output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL   
//                                                                             //    : SM_SP_U       = periodic U bndry seam not represented by Face->Edges          
//                                                                             //    : SM_SP_V       = periodic V bndry seam not represented by Face->Edges            
//                                                                             //    : SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges 
//                                                                             //    : SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges 
//                                         pThisFaceProps->m_eBadNearMissSeam, // in : from HasSeamProblem() call output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL   
//                                                                             //    : SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
//                                                                             //    : SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
//                                                                             //    : SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
//                                                                             //    : SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams 
//                                         &sThisNewFaces,                     // out: NewFaces made by splitting if any.  (does not include this face)    
//                                         &sThisNewSeamEdges,                 // out: Edges Added at the Seam if any.  
//                                         &sThisNewSeamVertices,              // out: Vertices Added at the Seam if any.
//                                         pThisFaceProps );                   // in : optional pre-cached set of this-Face properites, NULL to ignore
//    
//                  // accumulate outputs
//    
//                  // when face was not split - remember the run
//                  if(sNewFaces.GetSize() == 0) { m_sRanFaceProps_SplitFaceAtSeam.Add(pThisFaceProps, SM_TRY_RAN) ; }
//                  // else face was split - only count the run and update the New/Remove FaceLists fro upcoming UpdateTargetLists() call.
//                  else                         { m_sRanFaceProps_SplitFaceAtSeam.IncOrigProbCount() ; // pThisFace is now a child and not the parent - no parent left to put on the list
//                                                 sThisRemoveObjs.Add(pThisFace) ; // remove the stale parent face data
//                                                 sThisNewFaces.Add(pThisFace) ;   // add the child using the ThisPtr to the ThisNewFaces list.
//                                               }
//                  // build the AddObjs array for the upcoming UpdateTargetLists() call
//                  for(jj=0;jj<sThisNewFaces.GetSize();jj++) { sThisAddObjs.Add(sThisNewFaces[jj]) ; }
//                  for(jj=0;jj<sThisNewFaces.GetSize();jj++) { sThisAddObjs.Add(sThisNewSeamEdges[jj]) ; }
//                  for(jj=0;jj<sThisNewFaces.GetSize();jj++) { sThisAddObjs.Add(sThisNewSeamVertices[jj]) ; }
//    
//                  // Update the HealData lists - removing ptrs to the stale parent obj and adding new ObjProps for all the new topology
//                  if(   sThisRemoveObjs.GetSize() > 0
//                     || sThisAddObjs.GetSize() > 0)
//                    { UpdateTargetLists( &sThisRemoveObjs, &sThisAddObjs, SM_HO_FIX_BADLOOPS) ; }
//    
//                  // Insert the seam if it's missing
//                  if(pThisFaceProps->m_eBadMissingSeam != SM_SP_NEITHER)
//                    {
//                      // Insert the MissingSeam Edge segments into the Face - may split a face one or more times
//                      pThisFace->SplitAtSeam(*pThisFaceProps->m_pCrvClassU,       // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), NULL = execute HasSeamProblem() in pBrep call 
//                                             *pThisFaceProps->m_pCrvClassV,       // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), NULL = execute HasSeamProblem() in pBrep call 
//                                              pThisFaceProps->m_eBadCrossedSeam,  // in : from HasSeamProblem() output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL 
//                                                                                  //    : SM_SP_U       = periodic U bndry seam crossed by Face->Edges                 
//                                                                                  //    : SM_SP_V       = periodic V bndry seam crossed by Face->Edges                
//                                                                                  //    : SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges     
//                                                                                  //    : SM_SP_NEITHER = no seams crossed by Face->Edges                              
//                                              pThisFaceProps->m_eBadMissingSeam,  // in : from HasSeamProblem() call output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL   
//                                                                                  //    : SM_SP_U       = periodic U bndry seam not represented by Face->Edges          
//                                                                                  //    : SM_SP_V       = periodic V bndry seam not represented by Face->Edges            
//                                                                                  //    : SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges 
//                                                                                  //    : SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges 
//                                              pThisFaceProps->m_eBadNearMissSeam, // in : from HasSeamProblem() call output(bTgtLoopsOnly==FALSE), not used when pOptCrvClass == NULL   
//                                                                                  //    : SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
//                                                                                  //    : SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
//                                                                                  //    : SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
//                                                                                  //    : SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams 
//                                             &sThisNewFaces,                      // out: NewFaces made by splitting if any.  (does not include this face)    
//                                             &sThisNewSeamEdges,                  // out: Edges Added at the Seam if any.  
//                                             &sThisNewSeamVertices,               // out: Vertices Added at the Seam if any.
//                                              pThisFaceProps );                // in : optional pre-cached set of this-Face properites, NULL to ignore
//    
//                      // accumulate outputs
//                      m_lRanFaceCnt_SplitFaceAtSeam++;
//                      m_sNewFaces_FromSplitFaceAtSeams.Append( sThisNewFaces );
//                      m_sNewEdgesOnSeams_FromSplitFaceAtSeams.Append( sThisNewSeamEdges );
//                      m_sNewVertices_FromSplitFaceAtSeams.Append( sThisNewSeamVertices );
//                    }
//    
//                  // Current Loop Classification algorithm doesn't work with Tori with internal missing seams, so both have been inserted
//                  RefreshFaceProps( pThisFaceProps, pThisFaceProps->m_lFaceIndx, SM_HO_FIX_BADGAPS, ii );
//    
//                  bMadeChange = TRUE;
//                  m_sBadFaceProps_OrientLoop.Remove( pThisFaceProps );
//                } // end seam insertion for pair of badly oriented loops
//    
//            // remember the changes
//              if ( bMadeChange )
//                { sm_UpdateFaceProps(*this, pThisFaceProps, eThisHealerOp) ; }
//              //sChangedFaceProps.AddUnique( pThisFaceProps ); 
//    
//            }  // end for each face with badly oriented loops
//        } // end Scope: Face contains missing seam edge
// end Let's wait to see if this special case is still needed - perhaps the classify loops rewrite was enough

// Let's wait to see if this 2nd special case is still needed - perhaps the classify loops rewrite was enough
//  static SmBoolean bDoMissingOuterLoopFix = TRUE;
//    if(bDoMissingOuterLoopFix)
//      {
//        // Seam with one loop, correctly oriented as opposite. Should be in inner loop, but it's the only loop, so labeled as outer
//        // Add the Surface Natural boundaries as a loop -> this is the outer loop.
//  
//        // for every Face whose first loop is an inner loop
//        //   - add the surface natural boundary as an outer loop
//  
//        // when there are m_sBadFaceProps_OrientLoop problems
//        if(m_sBadFaceProps_OrientLoop.GetSize())
//          {
//            // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
//            //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
//            //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
//            //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
//            SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
//                                                    TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
//                                                             //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
//                                                             //      FALSE= don't
//  
//            // for every BadOrientLoop - look for cases where the natural boundary outer loop is missing and add it
//            for(ii=m_sBadFaceProps_OrientLoop.GetSize();ii > 0; --ii)
//              {
//                // locals
//                SmFaceProps * pThisFaceProps = m_sBadFaceProps_OrientLoop[ii - 1];
//                SmFace      * pThisFace = pThisFaceProps->m_pFace;
//                SmBoolean     bMadeChange = FALSE;
//  
//                if ( pThisFaceProps->m_sLoopProps.GetSize() == 0 )
//                  { continue; }
//  
//                SmLoopProps * pThisLoopProps = &( pThisFaceProps->m_sLoopProps[0] );
//  
//  #ifdef SM_DEBUG_CODE
//                if ( bDebugMe )
//                  {
//                    pThisFaceProps->Dump( ii );
//                    pThisFace->GetLoops( sDBGLoops );
//  
//                    smgfx_Erase(TRUE);
//                    smgfx_SetLook( 1, 2, 0, 0, 1 ); if ( pBrep ) pBrep->Draw( TRUE ); sm_GraphicsLoop();
//                    smgfx_SetLook( 1, 2, 0, 0, 1 ); if ( pThisFaceProps ) pThisFaceProps->Draw(); sm_GraphicsLoop();
//                    smgfx_SetLook( 3, 4, 1, 0, 0 ); if ( pThisFace ) pThisFace->GetSurface()->DrawSingularities(); sm_GraphicsLoop();
//                    smgfx_SetLook( 2, 3, .3, .3, .3 ); if ( pThisFace ) pThisFace->GetSurface()->DrawSeams(); sm_GraphicsLoop();
//                    smgfx_SetLook( 2, 3, 0, 1, 0 ); for ( di = 0; di < sDBGLoops.GetSize(); di++ )
//                    { sDBGLoops[di]->GetLoopuse()->Draw(); sm_GraphicsLoop(); }
//                    sm_GraphicsLoop();
//                  }
//  #endif // SM_DEBUG_CODE
//  
//                // Surface must be closed (2 seams (torus) or 1 seam + 2 singularities (Sphere)) AND
//                // ContainmentType == SM_CMT_OUTERLOOP because it's the only loop
//                // LoopOrient      == SM_OT_OPPOSITE because it's an inner loop
//                // DesiredLoopOrient == SM_OT_SAME because ContainmentType is OUTERLOOP (redundant condition, I think)
//                if(   (   (pThisFaceProps->m_bClosedU && pThisFaceProps->m_bClosedV)
//                       || (pThisFaceProps->m_bClosedU && pThisFaceProps->m_lPoles == (SM_SS_VMIN | SM_SS_VMAX))
//                       || (pThisFaceProps->m_bClosedV && pThisFaceProps->m_lPoles == (SM_SS_UMIN | SM_SS_UMAX)))
//                   && (pThisLoopProps->m_eContainmentType   == SM_CMT_OUTERLOOP)
//                   && (pThisLoopProps->m_eLoopOrient        == SM_OT_OPPOSITE)
//                   && (pThisLoopProps->m_eDesiredLoopOrient == SM_OT_SAME) )
//                  {
//                    // Locals
//                    SmCrvOnSurf           * pIsoCurve;
//                    SmCurve               * pUVIsoCurve;
//                    SmTArray<SmCurve *>     sIsoCurves;
//                    SmTArray<SmCurve *>     sUVIsoCurves;
//                    SmObjsDelete<SmCurve*>  sCleanIsoCurves( &sIsoCurves );
//  
//                    const SmContext       * cpContext = pThisFace->GetContext();
//                    SmSurface             * pSurface  = pThisFace->GetSurface();
//                    SmExtent2d              sUVDomain = pSurface->GetNaturalUVDomain();
//                    //SmApproxTol3d           sApproxTol3d = SmTol::GetApproxTol3d( pThisFace );
//                    SmZoneTol3d             sZoneTol3d = SmTol::GetZoneTol3d( pThisFace );
//  
//                    // Create 1 Curve for each non-degenerate, unique surface boundary
//                    SmPoint3d sStart, sEnd;
//                    for(jj=0;jj<4;jj++)
//                      {
//                        if ( jj == 0 && !( pThisFaceProps->m_lPoles & SM_SS_UMIN ) )
//                          {
//                            sStart.Set( sUVDomain.GetUMin(), sUVDomain.GetVMin(), 0.0 );
//                            sEnd  .Set( sUVDomain.GetUMin(), sUVDomain.GetVMax(), 0.0 );
//                          }
//                        else if ( jj == 1 && !( pThisFaceProps->m_lPoles & SM_SS_UMAX ) && !pThisFaceProps->m_bClosedU )
//                          {
//                            sStart.Set( sUVDomain.GetUMax(), sUVDomain.GetVMin(), 0.0 );
//                            sEnd  .Set( sUVDomain.GetUMax(), sUVDomain.GetVMax(), 0.0 );
//                          }
//                        else if ( jj == 2 && !( pThisFaceProps->m_lPoles & SM_SS_VMIN ) )
//                          {
//                            sStart.Set( sUVDomain.GetUMin(), sUVDomain.GetVMin(), 0.0 );
//                            sEnd  .Set( sUVDomain.GetUMax(), sUVDomain.GetVMin(), 0.0 );
//                          }
//                        else if ( jj == 3 && !( pThisFaceProps->m_lPoles & SM_SS_VMAX ) && !pThisFaceProps->m_bClosedV )
//                          {
//                            sStart.Set( sUVDomain.GetUMin(), sUVDomain.GetVMax(), 0.0 );
//                            sEnd  .Set( sUVDomain.GetUMax(), sUVDomain.GetVMax(), 0.0 );
//                          }
//                        else
//                          { continue; }
//  
//                        // Create the UVCurve and the 3dCurve
//                        SmSurface  * pCurveSurface;
//                        pSurface->Copy( *cpContext, pCurveSurface );
//                        SmExtent2d   sCurveSurfaceUVDomain = pCurveSurface->GetNaturalUVDomain();
//                        pUVIsoCurve = new ( cpContext ) SmLine     ( sStart, sEnd, 2, cpContext );
//                        pIsoCurve   = new ( cpContext ) SmCrvOnSurf( *pUVIsoCurve, *pCurveSurface, &sCurveSurfaceUVDomain, 3, cpContext );
//  
//                        // Add the curves to the lists
//                        sIsoCurves  .Add( pIsoCurve );
//                        sUVIsoCurves.Add( pUVIsoCurve );
//                      } // end creation of boundary curves
//  
//                    // Imprint each curve on the Face as an edge
//                    SmTArray<SmFace *> sThisNewFaces;
//                    SmTArray<SmEdge *> sThisNewEdges;
//                    SmTArray<SmVertex *> sThisNewVertices;
//                    SmEdge * pNewEdge = NULL;
//                    for(jj=0;jj<sIsoCurves.GetSize();jj++)
//                      {
//                        const SmCurve * pThisCurve = sIsoCurves[jj];
//                        const SmCurve * pThisUVCurve = sUVIsoCurves[jj];
//                        SmCurveClassification sCC( pThisCurve, pThisCurve->GetNaturalInterval(), pThisUVCurve,
//                                                   sZoneTol3d, 0, NULL, FALSE, TRUE, pSurface );
//                        pThisFace->CurveOnClassify( FALSE, sCC );
//  
//                        // Set Curve Classifications to the Face
//                        for(ULONG kk=0;kk<sCC.GetSize(); ++kk)
//                          {
//                            // Local
//                            SmCurveInterval & rCI = sCC[kk];
//  
//                            // Mid classification probably didn't work, because the face is broken. We're assuming this curve is in the face. 
//                            rCI.m_vMid  .SetClassObject( SM_PC_FACE, pThisFace );
//  
//                            // End points need to classify to the Face, if no existing vertex is present
//                            if ( rCI.m_vStart.GetPointClass() == SM_PC_UNKNOWN )
//                              { rCI.m_vStart.SetClassObject( SM_PC_FACE, pThisFace ); }
//                            else if ( rCI.m_vStart.GetPointClass() != SM_PC_VERTEX )
//                              { SM_DBG_WARN( _T( "Unexpected curve classification" ) ); }
//  
//                            if ( rCI.m_vEnd.GetPointClass() == SM_PC_UNKNOWN )
//                              { rCI.m_vEnd.SetClassObject( SM_PC_FACE, pThisFace ); }
//                            else if ( rCI.m_vEnd.GetPointClass() != SM_PC_VERTEX )
//                              { SM_DBG_WARN( _T( "Unexpected curve classification" ) ); }
//                          }
//  
//                        // Imprint the curves
//                        SE( sCC.MergeIntervals( &sThisNewFaces, &sThisNewEdges, &sThisNewVertices ) );
//  
//                        // Record the new Topology
//                        m_sNewFaces_FromExtraLoops.Append( sThisNewFaces );
//                        m_sNewEdgesOnSeams_FromSplitFaceAtSeams.Append( sThisNewEdges );
//                        m_sNewVertices_FromSplitEdgesAtSeams.Append( sThisNewVertices );
//  
//                        // Remember one of the new edges
//                        if ( !pNewEdge && sThisNewEdges.GetSize() > 0 )
//                          { pNewEdge = sThisNewEdges[0]; }
//                      }
//  
//                    // The new loop is the Outer loop. Elevate it to be the first loop.
//                    if ( pNewEdge )
//                      {
//                        SmLoop * pThisLoop = pNewEdge->GetLoopOfFace( pThisFace );
//                        if ( pThisLoop )
//                          {
//                            SmLoopuse * pUpwardLoopuse = NULL, *pDownwardLoopuse = NULL;
//                            SmFaceuse * pUpwardFaceuse = NULL, *pDownwardFaceuse = NULL;
//                            pThisLoop->GetLoopuses( pUpwardLoopuse, pDownwardLoopuse );
//                            pThisFace->GetFaceuses( pUpwardFaceuse, pDownwardFaceuse );
//                            pUpwardFaceuse->SetList( pUpwardLoopuse );
//                            pDownwardFaceuse->SetList( pDownwardLoopuse );
//                          }
//                        else 
//                        { SM_DBG_WARN(_T("Failed to find Edge->Loop to elevate as outer loop.")) }
//                      }
//  
//                    // Record change occurred. 
//                    bMadeChange = TRUE;
//                    if(pThisFaceProps->m_lBadOrient_LoopCnt > 0) { pThisFaceProps->m_lBadOrient_LoopCnt-- ; } 
//                  } // end creation of the outer loop from the surface natural boundary
//  
//                // close topology change tracking
//                sTopologyChanges.StopTracking() ;
//           
//                // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
//                UpdateTargetLists(sTopologyChanges.GetRmList(), sTopologyChanges.GetAddList(), SM_HO_FIX_SPLITEDGE_ATSEAM) ;
//  
//              } // end for each BadFaceProps_OrientLoop
//          } // end bDoMissingOuterLoopFix
// end Let's wait to see if this 2nd special case is still needed - perhaps the classify loops rewrite was enough

// gwc: I believe this special case is looking for and fixing MissedEdgeXSects where two edges secretly XSect without meeting at a vertex
//      rather than be a special case here - we need to make this a general SmHealData prob type.  I've added
//      stub functions for Cache_MissedEdgeXSects() and Fix_MissedEdgeXSects() and the members
//        SmHealData::m_sBadEdgeProps_MissedEdgeXSects  and SmFaceProps::m_bBadMissedEdgeXSect
//      TODO:: write the algorithm in Cache_MissedEdgeXSects() to find the missed EdgeXSects
//                   (that's not hard to do as an N**2 edge/edge intersection problem - but we want to be faster than that)
//             write the algorithm in Fix_MissedEdgeXSects() to insert the missing vertices.
//                   (may want to save the classify data from Cache_MissedEdgeXSects() so Fix_MissedEdgeXSects won't
//                    have to redo the missed Edge/Edge intersections again.)
//     FOR NOW - comment out block to be mined for the writing of Cache_MissedEdgeXSects() and Fix_MissedEdgeXSects()

// begin - FOR NOW - comment out block to be mined for the writing of Cache_MissedEdgeXSects() and Fix_MissedEdgeXSects()
//   // Look for cases where a Face has all its edges (no missing seam), but the outer loop is split into multiple loops
//   // In this case we'll delete the existing seam, then send the face to be split at it's seam.
//   static SmBoolean bDoIntrafaceLoopMatching = TRUE;
//   if ( bDoIntrafaceLoopMatching )
//     {
//       // Look for a Loop within this face that intersects the NoAreaLoop
//       for(ii=0;ii<m_sBadFaceProps_NoArea_Loop.GetSize();ii++)
//         {
//           // Locals
//           SmFaceProps           * pFaceProps = m_sBadFaceProps_NoArea_Loop[ii];
//           SmFace                * pLoopFace = pFaceProps->m_pFace;
//           SmTArray<SmLoopProps> & rLoopProps = pFaceProps->m_sLoopProps;
//           SmTArray<SmEdge *>      sTheseEdges, sOtherEdges;
//           SmBoolean               bMadeChange = FALSE;
// 
//           // Skip this FaceProps if it's missing a seam, or isn't closed.
//           if(    (pFaceProps->m_eBadMissingSeam != SM_SP_NEITHER)
//              || !(pFaceProps->m_bClosedU || pFaceProps->m_bClosedV))
//             { continue; }
// 
//           // For each NoAreaLoop, XSect with all other loops
//           for(jj=0;jj<rLoopProps.GetSize() && !bMadeChange;jj++)
//             {
//               // Locals
//               SmLoopProps     sThisLoopProp = rLoopProps[jj];
//               SmSolutionArray sSols;
//               sThisLoopProp.m_cpLoop->GetEdges( sTheseEdges );
// 
//               if ( sThisLoopProp.m_bArea_Loop )
//                 { continue ; }
// 
//               // For each other Loop
//               for(ULONG kk=0;kk<rLoopProps.GetSize() && kk!=jj && !bMadeChange; ++kk)
//                 {
//                   // Locals
//                   SmLoopProps sOtherLoopProp = rLoopProps[kk];
//                   sOtherLoopProp.m_cpLoop->GetEdges( sOtherEdges );
// 
//                   // For each Edge in ThisLoopProp.m_cpLoop
//                   for(ULONG ll=0;ll<sTheseEdges.GetSize(); ++ll)
//                     {
//                       // Locals
//                       SmEdge     * pThisEdge  = sTheseEdges[ll];
//                       SmCurve    * pThisCurve = pThisEdge->GetCurve();
//                       SmExtent1d   sThisIvl   = pThisEdge->GetInterval();
//                       
//                       // We want to find a Seam Edge that will XSect another Loop. Skip non-seam Edges
//                       SmBoolean   bDeleteSeam = TRUE;
//                       if ( !pThisEdge->IsSeam() && bDeleteSeam )
//                         { continue; }
// 
//                       // If seam curve is part of two loops, DeleteEdge will delete this face, then crash
//                       SmEdgeuse *pEU = pThisEdge->GetPrimaryEdgeuse();
//                       SmEdgeuse *pEURad = pEU->GetRadial();
//                       if ( pEU->GetLoopuse() != pEURad->GetLoopuse() )
//                         { continue; }
// 
//                       // For each Edge in OtherLoopProp.m_cpLoop
//                       for(ULONG mm=0;mm<sOtherEdges.GetSize(); ++mm)
//                         {
//                           // Locals
//                           SmEdge     * pOtherEdge  = sOtherEdges[mm];
//                           SmCurve    * pOtherCurve = pOtherEdge->GetCurve();
//                           SmExtent1d   sOtherIvl   = pOtherEdge->GetInterval();
// 
//                           SmXSectTol3d sXSectTol = SmTol::GetXSectTol3d( pThisEdge, pOtherEdge );
// 
//                           // XSect the Edge->Curves from ThisLoop and OtherLoop
//                           pThisCurve->GlobalCurveIntersect( sThisIvl,
//                                                             *pOtherCurve,
//                                                             sOtherIvl,
//                                                             sXSectTol,
//                                                             sSols );
// 
//                           // If the Seam Edge intersects OtherLoop, then delete the SeamEdge.
//                           // This triggers healing steps in current working order.
// 
//                           // Using XSections to trim Edges, then joining the loops at those new vertices
//                           // is a method that would work on general surfaces. However, both SmBrep::GlueVertices
//                           // and SmBrep::CombineCoincidentVertices don't work with these broken databases.
//                           // So for now we restrict ourselves to surfaces with seams; the only known cases
//                           // of Faces with this particular deficiency.
//                           SmVertex  * pStartVertex, *pEndVertex;
//                           ULONG       lNumSols    = sSols.GetSize();
//                           if ( bDeleteSeam && lNumSols > 0 )
//                             {
//                               // Delete the Seam Edge and any Vertices left without an Edge
//                               pThisEdge->GetVertices( pStartVertex, pEndVertex );
//                               pBrep->DeleteEdge( pThisEdge );
//                               if ( !pStartVertex->IsEdgeVertex() )
//                                 { pBrep->DeleteVertex( pStartVertex ); }
// 
//                               if ( !pEndVertex->IsEdgeVertex() )
//                                 { pBrep->DeleteVertex( pEndVertex ); }
// 
//                               // Move the seam if necessary
//                               ULONG lMoveSeamResult;
//                               RefreshFaceProps( pFaceProps, pFaceProps->m_lFaceIndx, SM_HO_FIX_BADLOOPS, mm);
//                               pLoopFace->MoveSeam( lMoveSeamResult,
//                                                *pFaceProps->m_pCrvClassU,
//                                                *pFaceProps->m_pCrvClassV,
//                                                pFaceProps->m_eBadCrossedSeam,
//                                                pFaceProps->m_eBadMissingSeam,
//                                                pFaceProps->m_eBadNearMissSeam,
//                                                TRUE, FALSE, NULL, NULL, NULL, pFaceProps );
// 
//                               bMadeChange = TRUE;
//                               break;
//                             } // end deleting the seam
//                           // The following else{} is a broken attempt to join the loops through new vertices
//                           //else
//                           //  {
//                           //    // For each XSection
//                           //    for ( ULONG nn = 0; nn < lNumSols && !bMadeChange; ++nn )
//                           //      {
//                           //        // Locals
//                           //        SmSolution          sSol = sSols[nn];
//                           //        SmTArray<SmEdge*>   sVEdges;
// 
//                           //        // Split edges at XSections
//                           //        if ( sSol.m_eSolutionType == SM_ST_SINGLE_VALUE )
//                           //          {
//                           //            SmTArray<SmEdge*> sTempNewEdges( 4, NULL, 4 );
//                           //            SmVertex *pNewV1, *pNewV2;
//                           //            pBrep->MakeVertexSplitEdge( pThisEdge, sSol.m_vStart[0], sTempNewEdges[0], sTempNewEdges[1], pNewV1 );
//                           //            pBrep->MakeVertexSplitEdge( pOtherEdge, sSol.m_vStart[1], sTempNewEdges[2], sTempNewEdges[3], pNewV2 );
// 
//                           //            // Both of the following fail to give us what we want
//                           //            pBrep->GlueVertices( pNewV1, pNewV2 ); // doesn't hook up loops
//                           //            //pBrep->CombineCoincidentVertices( pNewV1, pNewV2, pTempE ); // SER failure
// 
//                           //            // Delete Strut edges created from the split and marrying process
//                           //            for ( ULONG oo = 0; oo < 4; ++oo )
//                           //              {
//                           //                SmEdge * pEdge = sTempNewEdges[oo];
//                           //                if ( pEdge && pEdge->IsStrut() )
//                           //                  {
//                           //                    SmVertex *pVertex = pEdge->GetOtherVertex( pNewV1 );
//                           //                    pBrep->DeleteEdge( pEdge );
//                           //                    pBrep->DeleteVertex( pVertex );
//                           //                  }
//                           //              }  // end for each new Edge
// 
//                           //            bMadeChange = TRUE;
//                           //          } // End if Sol is Single Value
//                           //      } // End for each XSection
//                           //  } // End if branch for joining loops at intersection points
//                         } // end for each Edge in OtherLoopProps.m_cpLoop
//                     } // end for each Edge in ThisLoopProps.m_cpLoop
//                 } // end for each OtherLoop
//             } //  end for each ThisLoop
// 
//           // remember the changes, MoveSeam if necessary
//           if ( bMadeChange )
//             {
//               sm_UpdateFaceProps( *this, pFaceProps, eThisHealerOp );
//               m_sBadFaceProps_SeamProbs.Add( pFaceProps );
//               //sChangedFaceProps.AddUnique( pFaceProps );
//             } // end if MadeChange
// 
// #ifdef SM_DEBUG_CODE
//           if(bDebugMe)
//             {
//               this->Dump(77) ;
//               pFaceProps->Dump(0) ;
// 
//               smgfx_Erase(TRUE) ;
//               smgfx_SetLook(1,2, 0,0,1) ; pFaceProps->Draw() ; sm_GraphicsLoop() ;
//               sm_GraphicsLoop() ;
// 
//               smgfx_Erase(TRUE) ;
//               smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//               smgfx_SetLook(2,3, 1,0,0) ; if(pLoopFace) pLoopFace->Draw() ; sm_GraphicsLoop() ;
//               sm_GraphicsLoop() ;
//             }
// #endif // SM_DEBUG_CODE
//         } // end for each FaceProps with edges on a seam and a NoArea_Loop
//     } // end scope: MissedEdgeXSects
// end - FOR NOW - comment out block to be mined for the writing of Cache_MissedEdgeXSects() and Fix_MissedEdgeXSects()

  { // begin Scope: UnPairedNoAreaLoop

    // locals
    SmBoolean bDoNext_ii = TRUE ; 
    
    // for every Face with an UnpairedNoAreaLoop
    for(ii=0;(ii+1)<m_sBadFaceProps_UnpairedNoAreaLoop.GetSize();bDoNext_ii?ii++:ii)
      {
        SmFaceProps * pThisFaceProps = m_sBadFaceProps_UnpairedNoAreaLoop[ii] ;
        SmFace      * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ;
        SmSurface   * pThisSurface   = pThisFace->GetSurface() ; 

        bDoNext_ii = TRUE ; 

        for(jj=ii+1;jj<m_sBadFaceProps_UnpairedNoAreaLoop.GetSize();jj++)
          {
            SmFaceProps * pOtherFaceProps = m_sBadFaceProps_UnpairedNoAreaLoop[jj] ;
            SmFace      * pOtherFace      = (SmFace *)pOtherFaceProps->m_pFace ;
            SmSurface   * pOtherSurface   = pOtherFace->GetSurface() ; 

#ifdef SM_DEBUG_CODE
                if(bDebugMe)
                  {
                    this->Dump(77) ;
                    pThisFaceProps->Dump(0) ;
                    pOtherFaceProps->Dump(1) ;

                    smgfx_Erase(TRUE) ;
                    smgfx_SetLook(1,2, 0,0,1) ; pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
                    smgfx_SetLook(1,2, 1,0,1) ; pOtherFaceProps->Draw() ; sm_GraphicsLoop() ;
                    sm_GraphicsLoop() ;

                    smgfx_Erase(TRUE) ;
                    smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                    smgfx_SetLook(2,3, 1,0,0) ; if(pThisFace) pThisFace->Draw() ; sm_GraphicsLoop() ;
                    smgfx_SetLook(2,3, 1,0,0) ; if(pOtherFace) pOtherFace->Draw() ; sm_GraphicsLoop() ;
                    sm_GraphicsLoop() ;
                  }
#endif // SM_DEBUG_CODE
            // when two faces share a duplicated surface
            if(pThisSurface == pOtherSurface)
              {
#ifdef SM_DEBUG_CODE
                if(bDebugMe)
                  {
                    this->Dump(77) ;
                    pThisFaceProps->Dump(0) ;
                    pOtherFaceProps->Dump(1) ;

                    smgfx_Erase(TRUE) ;
                    smgfx_SetLook(1,2, 0,0,1) ; pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
                    smgfx_SetLook(1,2, 1,0,1) ; pOtherFaceProps->Draw() ; sm_GraphicsLoop() ;
                    sm_GraphicsLoop() ;
                  }
#endif // SM_DEBUG_CODE

// block awaiting debug
//                 // Now Move all OtherFace loops to ThisFace
//                 SmLoop *sLData[32];
//                 SmTArray<SmLoop*> sLoops(32,sLData);
//                 pOtherFace->GetLoops( sLoops );
//                 for (ULONG kk=0; kk<sLoops.GetSize(); kk++)
//                   {
//                     SmLoop *pMoveLoop = sLoops[kk];
//                     SER(pOtherFace->RemoveLoop(pMoveLoop));
// 
//                     // gwc: I don't think we have to worry about faceuse orientation when the surfaces are equal
// 
//                     SER(pThisFace->InsertLoop(pMoveLoop));
//                   } // end iter moving all OtherFace Loops to ThisFace
// 
//                 // See if we have a composite face that needs to be updated
//                 SmObject *pOwner = pOtherFace->GetSurface()->GetOwner();
//                 if (pOwner != pOtherFace)
//                   {
//                     SmCFace *pCF = SM_CAST_PTR( SmCFace, pOwner );
//                     if ( pCF != NULL )
//                       {
//                         // It's possible that we caused this just above, so no warning message.
//                         if ( pCF->ContainsFace( pOtherFace ) )
//                           { pCF->RemoveFace( pOtherFace ); }
//                       }
//                   } // end pOtherFace is part of a CFace check
// 
//                 // remove OtherFace->Faceuses from their shells and delete them
//                 SmFaceuse *pOtherFU, *pOtherFUMate;
//                 pOtherFace->GetFaceuses(pOtherFU,pOtherFUMate);
//                 pOtherFU->GetShell()->Remove(pOtherFU);
//                 pOtherFUMate->GetShell()->Remove(pOtherFUMate);
//                 SM_ASSERT(pOtherFU     != NULL) ; delete pOtherFU;     pOtherFU     = NULL ;
//                 SM_ASSERT(pOtherFUMate != NULL) ; delete pOtherFUMate; pOtherFUMate = NULL ;
// 
//                 // remove DelFace from Brep->FaceList
//                 pBrep->m_pFaceListHead->Remove(pOtherFace);
// 
//                 // delete pOtherFace (also pOtherFace->m_pSurface)
//                 SM_ASSERT(pOtherFace != NULL) ; delete pOtherFace ; pOtherFace = NULL ;
// 
//                 // update the HealData state
// 
//                 // remove and delete pOTherFaceProps from the HealData Arrays
//                 Remove(pOtherFaceProps) ;
//                 SM_ASSERT(pOtherFaceProps != NULL) ; delete pOtherFaceProps ; pOtherFaceProps = NULL ; 
// 
//                 // Refresh pThisFaceProps - faces with no problems get removed from their prob arrays
//                 RefreshFaceProps(pThisFaceProps, pThisFaceProps->m_lFaceIndx, eThisHealerOp, ii) ;
// 
// #ifdef SM_DEBUG_CODE
//                 if(bDebugMe)
//                   {
//                     this->Dump(77) ;
//                     pThisFaceProps->Dump(0) ;
//                     pOtherFaceProps->Dump(1) ;
// 
//                     smgfx_Erase(TRUE) ;
//                     smgfx_SetLook(1,2, 0,0,1) ; pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
//                     smgfx_SetLook(1,2, 1,0,1) ; pOtherFaceProps->Draw() ; sm_GraphicsLoop() ;
//                     sm_GraphicsLoop() ;
//                   }
// #endif // SM_DEBUG_CODE
//
//                // set next iteration up expecting pThisFaceProps has been removed from m_sBadFaceProps_UnpairedNoAreaLoop
//                bDoNext_ii = FALSE ; 
//
//  end block awaiting debug
              } // end two faces sharing a duplicated surface check
          } // end iter jj, every Face with an UnpairedNoAreaLoop
      } // end iter ii, every Face with an UnpairedNoAreaLoop
  } // end Scope: UnPairedNoAreaLoop 

  // Missing PoleVertices
  if(m_sBadFaceProps_MissingPole.GetSize() > 0)
    { 
      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // for every Face with Missing PoleVertices
      //   - make sure m_sBadFaceProps_MissingPole is rebuilt in upcoming changed_and_new_Face section. 
      for(ii=0;ii<m_sBadFaceProps_MissingPole.GetSize();ii++)
        {
          // locals
          SmFaceProps * pThisFaceProps = m_sBadFaceProps_MissingPole[ii] ; 
          SmFace      * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ;
          SmLoop      * pNewLoop       = NULL ;
          SmVertex    * pNewVertex     = NULL ; 

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                                             { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // for all possible Surface Poles
          for(jj=0;jj<4;jj++)
            {
              // when the VertexLoop is missing on the Pole
              if(pThisFaceProps->m_lBadMissingPoles & (  jj == 0 ? SM_SS_UMIN
                                                       : jj == 1 ? SM_SS_VMIN
                                                       : jj == 2 ? SM_SS_UMAX
                                                       :           SM_SS_VMAX))
                {
                  // Make a Vertex at the MissingPole
                  pBrep->MakeVertexLoop(pThisFace,                          // in : target face 
                                        pThisFaceProps->m_sPolePoints[jj],  // in :    
                                        pNewLoop,                           // out: new loop                     
                                        pNewVertex) ;                       // out: new vertex                   
                  if(pNewLoop)
                    { m_sNewLoopsOnPoles_FromFixBadLoops.Add(pNewLoop) ; }

                  if(pNewVertex)
                    { m_sNewVerticesOnPoles_FromFixBadLoops.Add(pNewVertex) ; }

                } // end MissingVertex on Pole check
            } // end iter jj, all possible Surface Poles

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                                            { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

        } // end iter ii, all BadFaces with MissingPoles

      // Refresh only after the loop: updating target lists deletes its FaceProps.
      sTopologyChanges.StopTracking() ;
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_BADLOOPS)) ;
    } // end Has Missing PoleVertices check

  // Bad Loop Orientations
  if(m_sBadFaceProps_OrientLoop.GetSize() > 0)
    {
      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // for every Face with Loop Orientation Problems
      for(ii=0;ii<m_sBadFaceProps_OrientLoop.GetSize();ii++)
        {
          // locals
          SmFaceProps * pThisFaceProps = m_sBadFaceProps_OrientLoop[ii] ; 

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SmFace  * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ;
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                                             { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE


 // #ifdef ALLOW_LOOP_FLIP_ORIENTATION
          // for every FaceLoop
          for(jj=0;jj<pThisFaceProps->m_sLoopProps.GetSize();jj++)
            {
              SmLoopProps * pThisLoopProps = &(pThisFaceProps->m_sLoopProps[jj]) ;
              SmLoop      * pThisLoop      = (SmLoop *)pThisLoopProps->m_cpLoop ; 

              // look for bad orientations
              if(FALSE == pThisLoopProps->m_bGoodOrient)
                {
                   // flip this Loop's orientation - update the LoopProps Orientation status
                   SER (pThisLoop->FlipLoopOrientation()) ;

                   // count the fix

                   // Toggle LoopProps->Orient value
                   pThisLoopProps->m_eLoopOrient = (  pThisLoopProps->m_eLoopOrient == SM_OT_OPPOSITE 
                                                    ? SM_OT_SAME 
                                                    : SM_OT_OPPOSITE) ;
                   // clear LoopProps->BadOrient state
                   pThisLoopProps->m_bGoodOrient = TRUE ;

                } // end Loop has a bad orientation check
            } // end iter jj, every BadFace->LoopProps
  // #endif // ALLOW_FLIP_ORIENTATION

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              SmFace * pThisFace = (SmFace *)pThisFaceProps->m_pFace ;
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                                             { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

        } // end iter ii, every BadFace with a OrientLoop problem

      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_BADLOOPS)) ;
    } // end has m_sBadFaceProps_OrientLoop problems

  // Bad Loop order
  if(m_sBadFaceProps_OuterLoopOrder.GetSize() > 0 )
    { 
      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // for every Face with bad OuterLoop ordering
      for(ii=0;ii<m_sBadFaceProps_OuterLoopOrder.GetSize();ii++)
        {
          // locals
          SmFaceProps * pThisFaceProps = m_sBadFaceProps_OuterLoopOrder[ii] ; 
          SmFace      * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ;

          // for every FaceLoop - find the 1st OuterLoop Loop
#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                               { smgfx_ChangeColor(di!=0) ; sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          for(jj=0;jj<pThisFaceProps->m_sLoopProps.GetSize();jj++)
            {
              SmLoopProps * pThisLoopProps = &(pThisFaceProps->m_sLoopProps[jj]) ;
              SmLoop      * pLoop          = (SmLoop *)pThisLoopProps->m_cpLoop ; 

              // find 1st OuterLoop Loop or SM_CMT_BOTLOOP NoAreaLoop
              if(   pThisLoopProps->m_eContainmentType == SM_CMT_OUTERLOOP
                 || pThisLoopProps->m_eContainmentType == SM_CMT_BOTLOOP)
                {
                  // locals
                  SmFaceuse * pUpperFaceuse = NULL ;
                  SmFaceuse * pLowerFaceuse = NULL ;
                  pThisFace->GetFaceuses(pUpperFaceuse, pLowerFaceuse) ;

                  SmLoopuse * pUpperLoopuse = pLoop->GetLoopuse() ;
                  SmLoopuse * pLowerLoopuse = pUpperLoopuse->GetOtherLoopuse() ;
                  SM_ASSERT_MSG(   pUpperLoopuse->GetOwner() == pUpperFaceuse
                                && pLowerLoopuse->GetOwner() == pLowerFaceuse,
                                _T("SmHealData::Fix_BadLoops from SmBrep::HealBrep(): found Loopuse Mates not consistently connected to Faceuse Mates")) ;

                  // make pLoop 1st in Face's LoopList by making its two Loopuses 1st in each Faceuse->LoopuseList
                  pThisFace->Notify(SM_NO_PRE_EDIT, pThisFace, pBrep, NULL) ;
                  pUpperFaceuse->SetList(pUpperLoopuse) ;
                  pLowerFaceuse->SetList(pLowerLoopuse) ;
                  pThisFace->Notify(SM_NO_POST_EDIT, pThisFace, pBrep, NULL) ;

                  // only promote the 1st OuterLoop to being the Face's 1st Loop
                  break ;

                } // end 1st OuterLoop check - (make it the Face's 1st loop)
            } // end iter jj, every BadFace->Loops

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                               { smgfx_ChangeColor(di!=0) ; sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

        } // end iter ii, every Face with BadOuterLoopOrder problems

      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_BADLOOPS)) ;

    } // end Bad Loop order  check

  // Bad MultiOuter Loops and/or Bad Nested Loops
  if(m_sBadFaceProps_NestedOuterLoops.GetSize() > 0 || m_sBadFaceProps_MultiOuterLoops.GetSize() > 0)
    { 
      SmTArray<SmFaceProps *> sBadOuterLoops ;
        
      sBadOuterLoops.Append      (m_sBadFaceProps_MultiOuterLoops.m_sProbArray) ;
      sBadOuterLoops.AppendUnique(m_sBadFaceProps_NestedOuterLoops.m_sProbArray) ; 

      // turn on Notify mechanism construction of SmTopoGraph change tracking Rm and Add Lists
      //   note: call UpdateTargetLists()  to update the SmHealData::Arrays using the sTopologyChanges's m_sRmList and m_sAddList arrays.
      //   loads SmTrackTopologyChanges::m_sRmList  with all Faces, Edges, Vertices removed from m_pBrep TopologyGraph that have to be removed from the SmHealData::ProbArrays
      //         SmTrackTopologyChanges::m_sAddList with all Faces, Edges, Vertices added to the m_pBrep TopologyGraph that have to be added to the SmHealData::Tgt arrays and associated with up to date ObjProps objs
      SmTrackTopologyChanges sTopologyChanges(m_pBrep, // in : Tgt Brep whose topology graph changes are being tracked 
                                              TRUE) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                       //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                       //      FALSE= don't
      // init mark
      rMarkLock.NewMark() ; // increment the mark value - very cheap

      // for every BadFace - Split out all the MultipleOuterLoops and NestedOuterLoops
      //   - make sure m_sBadFaceProps_MultiOuterLoops and m_sBadFaceProps_NestedOuterLoops is rebuilt in upcoming changed_and_new_Face section. 
      for(ii=0;ii<sBadOuterLoops.GetSize();ii++)
        {
          SmFaceProps * pThisFaceProps = sBadOuterLoops[ii] ; 
          SmFace      * pThisFace      = (SmFace *)pThisFaceProps->m_pFace ;
     
          // iter locals
          SmBoolean bFound1stOuter = FALSE ; // the 1st OuterLoop is not bad

#ifdef SM_DEBUG_CODE
          if(bDebugMe)
            {
              pThisFaceProps->Dump(ii) ;
              pThisFace->GetLoops(sDBGLoops) ;

              smgfx_Erase(TRUE) ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                                             { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // skip Faces already processed
          if(pThisFace->IsMarked(rMarkLock.GetMarkType()))
            { continue ; }

          // Mark Face - its about to get split
          pThisFace->Mark(rMarkLock.GetMarkType()) ; 

          // for every BadFace->Loop - Look for MultipleOuterLoops and NestedOuterLoops
          for(jj=0;jj<pThisFaceProps->m_sLoopProps.GetSize();jj++)
            {
              SmLoopProps * pThisLoopProps = &(pThisFaceProps->m_sLoopProps[jj]) ;
              SmLoop      * pThisLoop      = (SmLoop *)pThisLoopProps->m_cpLoop ; 

              // skip 1st OuterLoops - every Face gets one - it's not a problem
              if(   pThisLoopProps->m_eContainmentType == SM_CMT_OUTERLOOP
                 && bFound1stOuter == FALSE)
                { 
                  bFound1stOuter = TRUE ;
                  continue ; 
                }

              // look for bad MultipleOuterLoops and NestedOuterLoops - they all need to be split into their own faces
              if(   pThisLoopProps->m_eContainmentType == SM_CMT_OUTERLOOP
                 || pThisLoopProps->m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN)
                {
                  // Locals
                  SmFace                  * pLoopFace = pThisLoop->GetFace() ; // needed because Loop could have moved Faces in prev iter
                  SmFace                  * pNewFace  = NULL ; 
                  SmTArray<SmLooptreeItem>  sDescendants ; 
                  SmTArray<SmLoop *>        sLoopDescendants ;

                  // Find Loop in the Looptree
                  SmLooptreeItem * pParentItem = NULL ; 
                  SmLooptreeItem * pThisItem   = pThisFaceProps->m_pLooptreeRoot->FindItem(pThisLoop, pParentItem) ;
                  SM_ASSERT_MSG(pThisItem != NULL, 
                                _T("SmHealData::Fix_BadLoops from SmBrep::HealBrep(): Couldn't Find TgtLoop in Looptree - Bug needs fixing")) ;

                  // Get the descendants of this loop from the Loop Tree
                  // note: The descendants will not include any VertexLoops that may need moving.
                  //       The Looptree created by ClassifyLoops() only contains the EdgeLoops.  
                  //       We'll ask MakeFaceFromLoopOnFace to find the VertexLoops that need to be moved.
                  pThisItem->GetDescendants(sDescendants, sLoopDescendants, TRUE) ;

                  // Make Face from Loop on Face - Move Descendants from TgtFace to NewFace
                  pBrep->MakeFaceFromLoopOnFace
                           (pLoopFace,        // in : The Tgt Face
                            pThisLoop,        // in : The Tgt Loop to become the OuterLoop of NewFace
                            pNewFace,         // out: NewFace Created for the LoopToMove
                           &sLoopDescendants, // in : List of all TgtFace Loops contained within the TgtLoop
                                              //      default:[NULL] = Test for contained loops
                            TRUE) ;           // in : TRUE = Check VertexLoops for Moves even if ContainedLoops is given
                                              //      FALSE= VertexLoops are in ContainedLoops, don't check them

                   m_sNewFaces_FromExtraLoops.Add(pNewFace) ;
                   sNewFaces.Add(pNewFace) ;    

#ifdef SM_DEBUG_CODE
                  if(bDebugMe)
                    {
                      pThisFaceProps->Dump(ii) ;
                      pThisFace->GetLoops(sDBGLoops) ;

                      smgfx_Erase(TRUE) ;
                      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                      smgfx_SetLook(1,2, 0,0,1) ; if(pThisFaceProps) pThisFaceProps->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,4, 1,0,0) ; if(pThisFace) pThisFace->GetSurface()->DrawPoles() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, .3,.3,.3) ; if(pThisFace) pThisFace->GetSurface()->DrawSeams() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(3,4, 0,0,0) ; if(pLoopFace) pLoopFace->DrawUV() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(4,5, 1,0,0) ; if(pNewFace) pNewFace->DrawUV() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(5,6, 1,0,1) ; if(pThisLoop) pThisLoop->GetLoopuse()->Draw() ; sm_GraphicsLoop() ;
                      smgfx_SetLook(2,3, 0,1,0) ; for(di=0;di<sDBGLoops.GetSize();di++)
                                                     { sDBGLoops[di]->GetLoopuse()->Draw() ; sm_GraphicsLoop() ; }
                      sm_GraphicsLoop() ;
                    }
#endif // SM_DEBUG_CODE

                } // end Loop has a bad orientation check
            } // end iter jj, every LoopProps
        } // end iter ii, every BadFace with a OrientLoop problem

      // close topology change tracking
      sTopologyChanges.StopTracking() ;

      // Update Tgt and prob arrays from the sTopologyChanges's Rm and Add lists
      SER(UpdateTargetLists(sTopologyChanges, SM_HO_FIX_BADLOOPS)) ;

      // Make sure Prob array m_sBadFaceProps_MultiOuter_or_NestedOuterLoops is cleared 
      // in the changed and new Face section below. 

    } // End Bad MultiOuter and/or Bad Nested Loops check

  // note: NoAreaLoop problems are fixed when MissingSeams are added in SmHealData::Fix_SplitFaceAtSeams()

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_BadLoops

/*******************************************************************//**
PURPOSE: HealBrep helper function: Create UVTrimCurves

NOTES: fix Uncontained Edge/Curve errors 
***********************************************************************/
SmStatus SmHealData::Make_UVTrimCurves
 (SmBrep         * pBrep,              // in : Tgt Brep being healed
  SmBoolean        bMakeUVTrimCurves,  // in : User request for making UVTrimCurves
  SmHealerOpType   eThisHealerOp)      // NotUsed: in : This function's Step Number within the healer step sequence
{ 
  SM_REF1(eThisHealerOp) ;
  // no work - no Faces or no new Faces
  if(   m_sTgtFaceProps.GetSize() == 0
     || m_sTgtFaceProps.GetSize() == m_lOldFaceCnt)
    { return SM_SUCCESS ; }

  // locals
  ULONG ii ;
  SmTArray<SmFaceProps *> & rFaceProps = m_sTgtFaceProps ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
if(bDebugMe)
  {
    SM_DUMP_AND_ASSERT_VALID(pBrep) ;

    smgfx_Erase(TRUE) ;
    smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
    smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
    sm_GraphicsLoop() ;
  }
#else
  SM_REF1(pBrep);
#endif // SM_DEBUG_CODE

  // When asked - Create UVTrimCurves and set tolerances
  if(bMakeUVTrimCurves)
    {
      // locals
      double dMeanS, dMaxS, dMaxAllS=0.0, dMeanAllE=0.0 ;
      double dMeanE, dMaxE, dMaxAllE=0.0, dMeanAllV=0.0 ;
      double dMeanV, dMaxV, dMaxAllV=0.0, dMeanAllS=0.0 ;

      // for every face - (m_lOldFaceCnt != 0 when updating TgtLists with new objs)
      for(ii=m_lOldFaceCnt;ii<rFaceProps.GetSize();ii++)
        {
          SmFaceProps  * pThisFaceProps = rFaceProps[ii] ; 
          const SmFace * pLoopFace          = pThisFaceProps->m_pFace ;

          if(SM_SUCCESS != pLoopFace->CreateUVTrimCurves
              (TRUE,    // [in] : TRUE = adjust Edge and Vertex tolerances to include measured gaps
               NULL,    // [in] : optional array of UVTrimCurves to assign to Face->Edgeuses cpOptOrientations must be notNULL.
               NULL,    // [in] : associated orientations. If cpOptUVCurves notNULL, then
               dMeanS,  // [out]: average new UVTrimCurve/Surface distance
               dMaxS,   // [out]: max     UVTrimCurve/Surface distance
               dMeanE,  // [out]: average Vertex/EdgeuseEndPoint distance
               dMaxE,   // [out]: max     Vertex/EdgeuseEndPoint distance
               dMeanV,  // [out]: average UVTrimCurve/CCW_UVTrimCurve 2D gap
               dMaxV))  // [out]: max     UVTrimCurve/CCW_UVTrimCurve 2D gap
            {
#ifdef SM_DEBUG_CODE
              if(bDebugMe)
                {
                  pThisFaceProps->Dump(ii) ;

                  smgfx_Erase(TRUE) ;
                  smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                  pThisFaceProps->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                               // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                               // sMissingSeamColor   - def:[ 1, 0, 0]
                                                               // sMissingPoleColor   - def:[ 0, 0, 1]
                                                               // sBadOrientLoopColor - def:[.3,.3,.3]
                                                               // sNoArea_LoopColor   - def:[ 0, 0, 0]
                  sm_GraphicsLoop() ; 
                }
#endif // SM_DEBUG_CODE

              // inform the public - given previous healing, expect all CreateUVTrimCurve calls to succeed
              WARN(_T("HealBrep: CreateUVTrimCurves() failed on a Face")) ; 
            }

          dMaxAllS   = smos_Max(dMaxAllS, dMaxS);
          dMaxAllE   = smos_Max(dMaxAllE, dMaxE);
          dMaxAllV   = smos_Max(dMaxAllV, dMaxV);
          dMeanAllE += dMeanE;
          dMeanAllV += dMeanE;
          dMeanAllS += dMeanS;

        } // end iter all Faces
    } // end if asked to Create UVTrimCurves and set tolerances check

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Make_UVTrimCurves

/*******************************************************************//**
PURPOSE: HealBrep helper function: Fix InfiniteRegion

NOTES: 1. Brep has exactly 2 regions, One should be void and infinite
                                      The other should be solid
***********************************************************************/
SmStatus SmHealData::Fix_InfiniteRegion
 (SmBrep       * pBrep,           // in : Tgt Brep being healed
  SmHealerOpType eThisHealerOp)   // NotUsed: in : This function's Step Number within the healer step sequence
{    
  SM_REF1(eThisHealerOp) ;
  // locals
  SmRegion * pInfiniteRegion = pBrep->GetInfiniteRegion() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      smgfx_Erase(TRUE) ;
      smgfx_SetLook(2,3, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->DrawUV(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ; 
    }
#endif // SM_DEBUG_CODE 

  // case: Sheet models - 1 Void Region
  if(pInfiniteRegion->GetNext() == pInfiniteRegion)
    {
      pInfiniteRegion->SetIsVoid(TRUE) ;
    }

  // case: 2 regions - 1 Void InfiniteRegion, 1 solid Region
  else if(pInfiniteRegion->GetNext()->GetNext() == pInfiniteRegion ) // has exactly two regions
    {
      SmRegion * pNextRegion = (SmRegion *)pInfiniteRegion->GetNext() ;

      // only check obviously broken Regions 
      if(   pInfiniteRegion->IsVoid() != TRUE
         || pNextRegion->IsVoid()     != FALSE)
        { 
          // use ray casting to find and set the InfiniteRegion
          pBrep->FindAndSetInfiniteRegion() ;

          // refresh Region ptrs
          pInfiniteRegion = pBrep->GetInfiniteRegion() ;
          pNextRegion     = (SmRegion *)pInfiniteRegion->GetNext() ;

          // Set region Void/Solid data
          pInfiniteRegion->SetIsVoid(TRUE) ;
          pNextRegion->SetIsVoid(FALSE) ;

        } // end obviously broken region check
    } // end case exactly 2 regions

  else // case: more than 2 regions
    {
      // only check obviously broken Regions 
      if(pInfiniteRegion->IsVoid() != TRUE)
        {
          // use ray casting to find and set the InfiniteRegion
          pBrep->FindAndSetInfiniteRegion() ;

          // Set infinite region Void/Solid data
          pInfiniteRegion->SetIsVoid(TRUE) ;

          // we can't predict here which of the subsequent regions are void or solid - all combinations are possible

        } // end obviously broken region check
    } // end case: more than 2 regions

  // all done
  return(SM_SUCCESS) ;

} // end SmHealData::Fix_InfiniteRegion

/*******************************************************************//**
PURPOSE: SmHealData Pretty Print

NOTES:
***********************************************************************/
void SmHealData::Dump
  (ULONG lLabel)  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nBegin SmHealData Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmHealData Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);
                                                                                                              
  // Header 
  smos_sprintf(sBuff,        _T("\n SmHealData:[0x%p] for Brep:[0x%p]: TgtFaceCnt:[%lu], TgtEdgeCnt:[%lu], TgtVertexCnt:[%lu]"),
             this,  
             m_pBrep,
             m_sTgtFaces.GetSize(),
             m_sTgtEdges.GetSize(),
             m_sTgtVertices.GetSize());

  smos_sprintf(sBuffForFile, _T("\n SmHealData:[%s] for Brep:[%s]: TgtFaceCnt:[%lu], TgtEdgeCnt:[%lu], TgtVertexCnt:[%lu]"),
             _T("notNULL"),
             _T("notNULL"),
             m_sTgtFaces.GetSize(),
             m_sTgtEdges.GetSize(),   
             m_sTgtVertices.GetSize()) ;

  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // Faces with NotYet Problems
  m_sNotYetFaceProps.Dump(m_eDoneHealOp) ;
  
  // check state, TgtObjCnts == TgtPropsCnt
  if(   m_sTgtVertices.GetSize() != m_sTgtVertexProps.GetSize()
     || m_sTgtEdges.GetSize()    != m_sTgtEdgeProps.GetSize()  
     || m_sTgtFaces.GetSize()    != m_sTgtFaceProps.GetSize())
    {
     smos_WriteBuffer(  _T("\n FAILURE: TgtObjCnts must equal TgtPropsCnt")) ;
     if(m_sTgtVertices.GetSize() != m_sTgtVertexProps.GetSize())
       { smos_sprintf(sBuff, _T(" - TgtVtxCnt;[%lu] != TgtPropsCnt[%lu] "), 
                           m_sTgtVertices.GetSize(), 
                           m_sTgtVertexProps.GetSize()) ;   
         smos_WriteBuffer(sBuff, sBuffForFile) ;
       }
     if(m_sTgtVertices.GetSize() != m_sTgtEdgeProps.GetSize())
       { smos_sprintf(sBuff, _T(" - TgtEdgeCnt;[%lu] != TgtPropsCnt[%lu] "), 
                           m_sTgtEdges.GetSize(), 
                           m_sTgtEdgeProps.GetSize()) ;   
         smos_WriteBuffer(sBuff, sBuffForFile) ;
       }
     if(m_sTgtVertices.GetSize() != m_sTgtFaceProps.GetSize())
       { smos_sprintf(sBuff, _T(" - TgtFaceCnt;[%lu] != TgtPropsCnt[%lu] "), 
                           m_sTgtFaces.GetSize(), 
                           m_sTgtFaceProps.GetSize()) ;   
         smos_WriteBuffer(sBuff, sBuffForFile) ;
       }
    } // end check state, TgtObjCnts == TgtPropsCnt 

  // Old Target Counts
  if(    m_lOldVertexCnt > 0
     ||  m_lOldEdgeCnt   > 0
     ||  m_lOldFaceCnt   > 0)
    {
      smos_sprintf(sBuff,_T("%s"), _T("\n Updating TgtLists with Newly created Objs:")) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("\n   TgtFaceCnt  [Old, New] :[%lu, %lu] "), m_lOldFaceCnt, m_sTgtFaces.GetSize() - m_lOldFaceCnt) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("\n   TgtEdgeCnt  [Old, New] :[%lu, %lu] "), m_lOldEdgeCnt, m_sTgtEdges.GetSize() - m_lOldEdgeCnt) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("\n   TgtVertexCnt[Old, New] :[%lu, %lu] "), m_lOldVertexCnt, m_sTgtVertices.GetSize() - m_lOldVertexCnt) ; smos_WriteBuffer(sBuff) ; 
    }

  // Total StepCnt
  smos_sprintf(sBuff, _T("\n Total Number of Healer Steps:[%d]"), SM_HO_ALL - 1) ; smos_WriteBuffer(sBuff) ; 
  
  // HealState: m_eStopHealOp,  m_eDoneHealOp
  smos_sprintf(sBuff, _T("\n Max HealerOp requested      :[%d=Run All Healer Steps thru %s]"), m_eStopHealOp, SM_HEALOP_STRING(m_eStopHealOp)) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n Max HealerOp Ran            :[%d=Ran All Healer Steps thru %s]"), m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; smos_WriteBuffer(sBuff) ;

  // output the HealData from Each step atomically because steps get split, renamed, reordered, and merged
  smos_sprintf(sBuff, _T("\n PART PROPERTY HISTOGRAMS: Through step %s"), SM_HEALOP_STRING(m_eDoneHealOp)) ; smos_WriteBuffer(sBuff) ;

  // Histograms for every Done Heal Op - pretty print status
    { // begin scope - pretty print histograms
      if(m_eDoneHealOp >= SM_HO_CACHE_EDGEPROPS) 
        { 
          // m_sLoopGap3d_Histogram    Edge-Edge gap3ds
          smos_WriteBuffer( _T("\n                             +-------------------------------------------------------------------------------+")) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d]   LoopGap3ds     : [ %4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [%4lu] [%4lu]"), 
                             SM_HO_CACHE_EDGEPROPS,
                             m_sLoopGap3d_Histogram[ 0], m_sLoopGap3d_Histogram[ 1], m_sLoopGap3d_Histogram[ 2],
                             m_sLoopGap3d_Histogram[ 3], m_sLoopGap3d_Histogram[ 4], m_sLoopGap3d_Histogram[ 5],
                             m_sLoopGap3d_Histogram[ 6], m_sLoopGap3d_Histogram[ 7], m_sLoopGap3d_Histogram[ 8],
                             m_sLoopGap3d_Histogram[ 9], m_sLoopGap3d_Histogram[10]) ;                            
          smos_WriteBuffer(sBuff) ; 
          smos_WriteBuffer( _T("\n              Histogram  from: [less ] [1e-10] [1e-8] [1e-6] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.]")) ; 
          smos_WriteBuffer( _T("\n                           to: [1e-10] [1e-8 ] [1e-6] [1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [more]")) ;
          // end m_sLoopGap3d_Histogram

          // m_sEdgeFaceGap3d_Histogram
          smos_WriteBuffer( _T("\n                             +-------------------------------------------------------------------------------+")) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d]   EdgeFaceGap3ds : [ %4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [%4lu] [%4lu]"), 
                             SM_HO_CACHE_EDGEPROPS,
                             m_sEdgeFaceGap3d_Histogram[ 0], m_sEdgeFaceGap3d_Histogram[ 1], m_sEdgeFaceGap3d_Histogram[ 2],
                             m_sEdgeFaceGap3d_Histogram[ 3], m_sEdgeFaceGap3d_Histogram[ 4], m_sEdgeFaceGap3d_Histogram[ 5],
                             m_sEdgeFaceGap3d_Histogram[ 6], m_sEdgeFaceGap3d_Histogram[ 7], m_sEdgeFaceGap3d_Histogram[ 8],
                             m_sEdgeFaceGap3d_Histogram[ 9], m_sEdgeFaceGap3d_Histogram[10]) ;                            
          smos_WriteBuffer(sBuff) ; 
          smos_WriteBuffer( _T("\n              Histogram  from: [less ] [1e-10] [1e-8] [1e-6] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.]")) ; 
          smos_WriteBuffer( _T("\n                           to: [1e-10] [1e-8 ] [1e-6] [1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [more]")) ;
          // end m_sEdgeFaceGap3d_Histogram

          // m_sEdgeLen3d_Histogram
          smos_WriteBuffer( _T("\n                             +-----------------------------------------------------------------------------------------------------+")) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d]   EdgeLengths    :                        [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu]"), 
                             SM_HO_CACHE_EDGEPROPS,
                             m_sEdgeLen3d_Histogram[ 0], m_sEdgeLen3d_Histogram[ 1], m_sEdgeLen3d_Histogram[ 2],
                             m_sEdgeLen3d_Histogram[ 3], m_sEdgeLen3d_Histogram[ 4], m_sEdgeLen3d_Histogram[ 5],
                             m_sEdgeLen3d_Histogram[ 6], m_sEdgeLen3d_Histogram[ 7], m_sEdgeLen3d_Histogram[ 8],
                             m_sEdgeLen3d_Histogram[ 9], m_sEdgeLen3d_Histogram[10]) ;                            
          smos_WriteBuffer(sBuff) ; 
          smos_WriteBuffer( _T("\n              Histogram  from:                        [less] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.] [ 100.] [1e+3] [1e+4]")) ; 
          smos_WriteBuffer( _T("\n                           to:                        [1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [100.] [1000.] [1e+4] [more]")) ;
          // end m_sEdgeLen3d_Histogram

          // summary
          smos_sprintf(sBuff, _T("\n  step:[%2d] MaxEdgeLength         : [%16.16lf]"), SM_HO_CACHE_EDGEPROPS, m_dMaxEdgeLength ) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d] AvgEdgeLength         : [%16.16lf]"), SM_HO_CACHE_EDGEPROPS, m_dAvgEdgeLength ) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d] NonDegen MinEdgeLength: [%16.16lf]"), SM_HO_CACHE_EDGEPROPS, m_dNonDegen_MinEdgeLength) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d] NonDegen EdgeCnt      : [%lu]"), SM_HO_CACHE_EDGEPROPS, m_lNonDegenEdgeCnt) ; 

        } // end if(m_eDoneHealOp >= SM_HO_CACHE_EDGEPROPS)
                                            
      if(m_eDoneHealOp >= SM_HO_CACHE_VERTEXPROPS)
        { // m_sVertEdgeGap3d_Histogram
          smos_WriteBuffer( _T("\n                             +-------------------------------------------------------------------------------+")) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d]   VertEdgeGap3ds : [ %4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [%4lu] [%4lu]"), 
                             SM_HO_CACHE_VERTEXPROPS,
                             m_sVertEdgeGap3d_Histogram[ 0], m_sVertEdgeGap3d_Histogram[ 1], m_sVertEdgeGap3d_Histogram[ 2],
                             m_sVertEdgeGap3d_Histogram[ 3], m_sVertEdgeGap3d_Histogram[ 4], m_sVertEdgeGap3d_Histogram[ 5],
                             m_sVertEdgeGap3d_Histogram[ 6], m_sVertEdgeGap3d_Histogram[ 7], m_sVertEdgeGap3d_Histogram[ 8],
                             m_sVertEdgeGap3d_Histogram[ 9], m_sVertEdgeGap3d_Histogram[10]) ;                            
          smos_WriteBuffer(sBuff) ; 
          smos_WriteBuffer( _T("\n              Histogram  from: [less ] [1e-10] [1e-8] [1e-6] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.]")) ; 
          smos_WriteBuffer( _T("\n                           to: [1e-10] [1e-8 ] [1e-6] [1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [more]")) ;
          // end m_sVertEdgeGap3d_Histogram

          // m_sVertFaceGap3d_Histogram
          smos_WriteBuffer( _T("\n                             +-------------------------------------------------------------------------------+")) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d]   VertFaceGap3ds : [ %4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [%4lu] [%4lu]"), 
                             SM_HO_CACHE_VERTEXPROPS,
                             m_sVertFaceGap3d_Histogram[ 0], m_sVertFaceGap3d_Histogram[ 1], m_sVertFaceGap3d_Histogram[ 2],
                             m_sVertFaceGap3d_Histogram[ 3], m_sVertFaceGap3d_Histogram[ 4], m_sVertFaceGap3d_Histogram[ 5],
                             m_sVertFaceGap3d_Histogram[ 6], m_sVertFaceGap3d_Histogram[ 7], m_sVertFaceGap3d_Histogram[ 8],
                             m_sVertFaceGap3d_Histogram[ 9], m_sVertFaceGap3d_Histogram[10]) ;                            
          smos_WriteBuffer(sBuff) ; 
          smos_WriteBuffer( _T("\n              Histogram  from: [less ] [1e-10] [1e-8] [1e-6] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.]")) ; 
          smos_WriteBuffer( _T("\n                           to: [1e-10] [1e-8 ] [1e-6] [1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [more]")) ;
          // end m_sVertFaceGap3d_Histogram

         } // end if(m_eDoneHealOp >= SM_HO_CACHE_VERTEXPROPS)
                                             
      if(m_eDoneHealOp >= SM_HO_CACHE_COIN_VERTICES)
        { // m_sVertVertGap3d_Histogram
          smos_WriteBuffer( _T("\n                             +-------------------------------------------------------------------------------+")) ; 
          smos_sprintf(sBuff, _T("\n  step:[%2d]   VertVertDist3ds: [ %4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [ %4lu] [%4lu] [%4lu] [%4lu] [%4lu] [%4lu]"), 
                             SM_HO_CACHE_VERTEXPROPS,
                             m_sVertVertGap3d_Histogram[ 0], m_sVertVertGap3d_Histogram[ 1], m_sVertVertGap3d_Histogram[ 2],
                             m_sVertVertGap3d_Histogram[ 3], m_sVertVertGap3d_Histogram[ 4], m_sVertVertGap3d_Histogram[ 5],
                             m_sVertVertGap3d_Histogram[ 6], m_sVertVertGap3d_Histogram[ 7], m_sVertVertGap3d_Histogram[ 8],
                             m_sVertVertGap3d_Histogram[ 9], m_sVertVertGap3d_Histogram[10]) ;                            
          smos_WriteBuffer(sBuff) ; 
          smos_WriteBuffer( _T("\n              Histogram  from: [less ] [1e-10] [1e-8] [1e-6] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.]")) ; 
          smos_WriteBuffer( _T("\n                           to: [1e-10] [1e-8 ] [1e-6] [1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [more]")) ;
          // end m_sVertVertGap3d_Histogram
      
        } // end if(m_eDoneHealOp >= SM_HO_FIX_COIN_VERTICES)
    } // end scope - pretty print histograms

  // Vertex buckets
  if(m_eDoneHealOp >= SM_HO_CACHE_COIN_VERTICES)
    {
      smos_sprintf(sBuff, _T("\n\n step:[%2u] %s: Voxel Counts:[%2lu, %2lu, %2lu], Vertex BoundingBox: "),
                       m_eDoneHealOp,
                       SM_HEALOP_STRING(m_eDoneHealOp),
                       m_lXCnt,
                       m_lYCnt,
                       m_lZCnt) ;
      smos_WriteBuffer(sBuff) ;
      m_sVertexBBox.Dump() ;
    }

  // output the HealData from Each step atomically because steps get split, renamed, reordered, and merged
  smos_sprintf(sBuff, _T("\n\n NEW OBJS: Through step:[%2u] %s"), m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; smos_WriteBuffer(sBuff) ;

  // for every Done Heal Op - pretty print status
  ULONG lProbCnt = 0 ;
  if(m_eDoneHealOp >= SM_HO_NONE) 
    { /* new objs */
      // MEMBER LOCAL MACRO
      #define SM_NEWOBJS(Array, StepNo, ArrayStr, ObjStr, StepStr) \
      if(Array.GetSize() > 0)                              \
        { smos_sprintf(sBuff, _T("\n   Step[%2d] %s: New%s:[%3lu] - from %s"),      \
                            StepNo, ArrayStr, ObjStr, Array.GetSize(), StepStr) ; \
          smos_WriteBuffer(sBuff) ;                                               \
        }
      // END MEMBER LOCAL MACROS

      ULONG lNewObjCnt =   m_sNewFaces_FromFixDegenFaces          .GetSize()
                         + m_sNewEdges_FromSplitEdgesAtSeams      .GetSize()
                         + m_sNewVertices_FromSplitEdgesAtSeams   .GetSize()
                         + m_sNewVerticesOnPoles_FromFixBadLoops  .GetSize()
                         + m_sNewLoopsOnPoles_FromFixBadLoops     .GetSize()
                         + m_sNewFaces_FromExtraLoops             .GetSize()
                         + m_sNewFaces_FromSplitFaceAtSeams       .GetSize()
                         + m_sNewEdgesOnSeams_FromSplitFaceAtSeams.GetSize()
                         + m_sNewVertices_FromSplitFaceAtSeams    .GetSize()
                         + m_sNewRegions_FromClosedSheets         .GetSize()
                         + m_sNewShells_FromClosedSheets          .GetSize() ;

      SM_NEWOBJS(m_sNewFaces_FromFixDegenFaces          , SM_HO_FIX_DEGEN_FACES       , _T("NewFacesFrom_FixDegenFaces "), _T("faces"), _T("SmHealData::Fix_DegenFaces()      ") )    
      SM_NEWOBJS(m_sNewEdges_FromSplitEdgesAtSeams      , SM_HO_FIX_SPLITEDGE_ATSEAM  , _T("NewEdges_FromSplitAtSeams  "), _T("Edges"), _T("SmHealData::Fix_SplitEdgesAtSeam()") )    
      SM_NEWOBJS(m_sNewVertices_FromSplitEdgesAtSeams   , SM_HO_FIX_SPLITEDGE_ATSEAM  , _T("NewVertices_OnSeams        "), _T("Verts"), _T("SmHealData::Fix_SplitEdgesAtSeam()") )    
      SM_NEWOBJS(m_sNewVerticesOnPoles_FromFixBadLoops                 , SM_HO_FIX_BADLOOPS          , _T("NewVertices_OnPoles        "), _T("Verts"), _T("SmHealData::Fix_BadLoops()        ") )    
      SM_NEWOBJS(m_sNewLoopsOnPoles_FromFixBadLoops                    , SM_HO_FIX_BADLOOPS          , _T("NewLoops_OnPoles           "), _T("Loops"), _T("SmHealData::Fix_BadLoops()        ") )    
      SM_NEWOBJS(m_sNewFaces_FromExtraLoops             , SM_HO_FIX_BADLOOPS          , _T("NewFaces_FromExtraLoops    "), _T("Faces"), _T("SmHealData::Fix_BadLoops()        ") )    
      SM_NEWOBJS(m_sNewFaces_FromSplitFaceAtSeams        , SM_HO_FIX_SPLITFACE_ATSEAMS, _T("NewFaces_FromSplitAtSeams  "), _T("Faces"), _T("SmHealData::Fix_SplitFaceAtSeams()") )    
      SM_NEWOBJS(m_sNewEdgesOnSeams_FromSplitFaceAtSeams, SM_HO_FIX_SPLITFACE_ATSEAMS , _T("NewEdges_OnSeams           "), _T("Edges"), _T("SmHealData::Fix_SplitFaceAtSeams()") )    
      SM_NEWOBJS(m_sNewVertices_FromSplitFaceAtSeams    , SM_HO_FIX_SPLITFACE_ATSEAMS , _T("NewVertices_OnSeams        "), _T("Verts"), _T("SmHealData::Fix_SplitFaceAtSeams()") )    
      SM_NEWOBJS(m_sNewRegions_FromClosedSheets         , SM_HO_FIX_BADSHEETS         , _T("NewRegions_FromClosedSheets"), _T("Regs "), _T("SmHealData::Fix_BadSheets()       ") )    
      SM_NEWOBJS(m_sNewShells_FromClosedSheets          , SM_HO_FIX_BADSHEETS         , _T("NewShells_FromClosedSheets "), _T("Shells"), _T("SmHealData::Fix_BadSheet()        ") )    

      if(lNewObjCnt > 0) { smos_sprintf(sBuff, _T("\n  Summary: New Objs:[%3lu] From Healing through step:[%2d] %s"), lNewObjCnt, m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; }
      else               { smos_sprintf(sBuff, _T("\n  Summary: No New Objs From Healing through step:[%2d] %s"), m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; }
      smos_WriteBuffer(sBuff) ; 

    } // end if(m_eDoneHealOp >= SM_HO_NONE)

  // output the HealData from Each step atomically because steps get split, renamed, reordered, and merged
  smos_sprintf(sBuff, _T("\n\n PROBARRAYS: Through step:[%2d] %s"), m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; smos_WriteBuffer(sBuff) ;

  if(m_eDoneHealOp >= SM_HO_CACHE_EDGEPROPS)         { lProbCnt += m_sBadEdgeProps_Uncontained    .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadEdgeProps_SmallZoneTol3d .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadEdgeProps_LargeZoneTol3d .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sWarnEdgeProps_LargeGap3d    .Dump(m_eDoneHealOp) ;   
                                                       lProbCnt += m_sBadEdgeProps_DegenEdges     .Dump(m_eDoneHealOp) ; 
                                                     }  
  if(m_eDoneHealOp >= SM_HO_CACHE_VERTEXPROPS)       { lProbCnt += m_sBadVertexProps_SmallZoneTol3d.Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadVertexProps_LargeZoneTol3d.Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sWarnVertexProps_LargeGap3d   .Dump(m_eDoneHealOp) ;
                                                     }  
  if(m_eDoneHealOp >= SM_HO_CACHE_FACEPROPS_GAPS)    { lProbCnt +=  m_sBadFaceProps_SmallZoneTol3d.Dump(m_eDoneHealOp) ;
                                                       lProbCnt +=  m_sBadFaceProps_LargeZoneTol3d.Dump(m_eDoneHealOp) ;
                                                     }  
  if(m_eDoneHealOp >= SM_HO_FIX_TOLSIZES)            { }
                                                     
  if(m_eDoneHealOp >= SM_HO_CACHE_COIN_VERTICES)     { lProbCnt += m_sBadVertexProps_CoinVertices.Dump(m_eDoneHealOp) ; } 
  if(m_eDoneHealOp >= SM_HO_FIX_COIN_VERTICES)       { }
                                                     
  if(m_eDoneHealOp >= SM_HO_CACHE_DEGEN_FACES)       { lProbCnt += m_sBadFaceProps_DegenFaces    .Dump(m_eDoneHealOp) ; } 
  if(m_eDoneHealOp >= SM_HO_FIX_DEGEN_FACES)         { }
  if(m_eDoneHealOp >= SM_HO_FIX_DEGEN_EDGES)         { }  
  
  if(m_eDoneHealOp >= SM_HO_CACHE_COIN_EDGES)        { lProbCnt += m_sBadEdgeProps_CoinEdges.Dump(m_eDoneHealOp) ; }  
  if(m_eDoneHealOp >= SM_HO_FIX_COIN_EDGES)          { }
  if(m_eDoneHealOp >= SM_HO_CACHE_MISSED_EDGEXSECTS) { }
  if(m_eDoneHealOp >= SM_HO_FIX_MISSED_EDGEXSECTS)   { }
  if(m_eDoneHealOp >= SM_HO_FIX_UNCONTAINED_EDGES)   { }
  if(m_eDoneHealOp >= SM_HO_FIX_BADGAPS)             { }
                                             
  if(m_eDoneHealOp >= SM_HO_CACHE_FACEPROPS_2)       { lProbCnt += m_sFaceProps_Sheets         .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sFaceProps_NoVertexPole   .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadFaceProps_FlatCorner  .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadFaceProps_MissingSeam .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadFaceProps_CrossedSeam .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadFaceProps_NearMissSeam.Dump(m_eDoneHealOp) ;
                                                     }  
                                                     
  if(m_eDoneHealOp >= SM_HO_FIX_MOVESEAM)            { lProbCnt += m_sRanFaceProps_MoveSeam.Dump(m_eDoneHealOp) ; }  
  if(m_eDoneHealOp >= SM_HO_FIX_SPLITEDGE_ATSEAM)    { lProbCnt += m_sRanFaceProps_SplitEdgesAtSeam.Dump(m_eDoneHealOp) ; }  
                                                     
  if(m_eDoneHealOp >= SM_HO_FIX_BADSHEETS)           { smos_sprintf(sBuff, _T("\n  step:[%2d] Ran TestRegionClosure Count: [%4lu]"), SM_HO_FIX_BADSHEETS, m_lRanRegion_TestClosure_Cnt ) ;
                                                       smos_sprintf(sBuff, _T("\n  step:[%2d] Ran SplitRegion       Count: [%4lu]"), SM_HO_FIX_BADSHEETS, m_lFixRegion_SplitRegion_Cnt ) ;
                                                     }  // end if(m_eDoneHealOp >= SM_HO_FIX_BADSHEETS  
                                                     
  if(m_eDoneHealOp >= SM_HO_CACHE_FACEPROPS_3)       { lProbCnt += m_sBadFaceProps_LoopProblems      .Dump(m_eDoneHealOp) ;             
                                                       lProbCnt += m_sBadFaceProps_OuterLoopOrder    .Dump(m_eDoneHealOp) ;           
                                                       lProbCnt += m_sBadFaceProps_OrientLoop        .Dump(m_eDoneHealOp) ;               
                                                       lProbCnt += m_sBadFaceProps_NoArea_Loop       .Dump(m_eDoneHealOp) ;              
                                                       lProbCnt += m_sBadFaceProps_MultiOuterLoops   .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadFaceProps_NestedOuterLoops  .Dump(m_eDoneHealOp) ;
                                                       lProbCnt += m_sBadFaceProps_UnpairedNoAreaLoop.Dump(m_eDoneHealOp) ;       
                                                       lProbCnt += m_sBadFaceProps_Closed3dLoop      .Dump(m_eDoneHealOp) ;             
                                                       lProbCnt += m_sBadFaceProps_ClosedPtrLoop     .Dump(m_eDoneHealOp) ;            
                                                       lProbCnt += m_sBadFaceProps_MissingPole       .Dump(m_eDoneHealOp) ;              
                                                     }  

  if(m_eDoneHealOp >= SM_HO_FIX_SPLITFACE_ATSEAMS)   { lProbCnt += m_sRanFaceProps_SplitFaceAtSeam.Dump(m_eDoneHealOp) ; } 
  if(m_eDoneHealOp >= SM_HO_FIX_BADLOOPS)            { }
  if(m_eDoneHealOp >= SM_HO_MAKE_UVTRIMCURVES)       { }
  if(m_eDoneHealOp >= SM_HO_FIX_INFINITE_REGIONS)    { }

  // summary
  if(lProbCnt == 0) { smos_sprintf(sBuff, _T("\n  Summary: No problems through step:[%2u]: %s - Good"), m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; }
  else              { smos_sprintf(sBuff, _T("\n  Summary: Problem Count:[%3lu] through step[%2d]: %s - Bad"), lProbCnt, m_eDoneHealOp, SM_HEALOP_STRING(m_eDoneHealOp)) ; }
  smos_WriteBuffer(sBuff) ;

  // End
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nEnd SmHealData Dump")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd SmHealData Dump   [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

} // end SmHealData::Dump

/*******************************************************************//**
PURPOSE: For debugging - Pretty Print ListNames containing Tgt VertexProps obj

NOTES:
***********************************************************************/
void SmHealData::DumpVertexListsContaining
 (SmVertexProps * pVertexProps,  // in : Tgt VertexProps obj
  ULONG           lLabel)        // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]

 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // SM_INLIST uses SM_STRINGIZE to print the prob-array symbol name in debug output.
  #define SM_STRINGIZE(a) #a
  #define SM_INLIST(a) if((a).m_sProbArray.IsIn(pVertexProps)) { smos_sprintf(sBuff, _T("\n  in list %s"), _T(SM_STRINGIZE(a))) ; smos_WriteBuffer(sBuff) ; }

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff       , _T("\nBegin SmHealData DumpVertexListsContaining pVertexProps:[0x%p]"), 
                                                          pVertexProps) ; 
                               }                        
  else                         { smos_sprintf(sBuff       , _T("\nBegin SmHealData DumpVertexListsContaining [%3lu] for pVertexProps:[0x%p]"),
                                                          lLabel, 
                                                          pVertexProps) ;
                               }                        
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuffForFile, _T("\nBegin SmHealData DumpVertexListsContaining pVertexProps:[%s]"), 
                                                          pVertexProps ? _T("NotNULL") : _T("NULL") ) ; 
                               }                        
  else                         { smos_sprintf(sBuffForFile, _T("\nBegin SmHealData DumpVertexListsContaining [%3lu] for pVertexProps:[%s]"),
                                                          lLabel, 
                                                          pVertexProps ? _T("NotNULL") : _T("NULL") ) ;
                               }
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // check the lists
  // VertexLists and associated VertexPropsList - they are a pair
  if(   pVertexProps->m_pVertex  
     && m_sTgtVertices.IsIn((SmVertex*)pVertexProps->m_pVertex))
    { 
      ULONG lCnt = 0 ;
      for(ULONG ii=0;ii<m_sTgtVertices.GetSize();ii++) { if(m_sTgtVertices[ii] == pVertexProps->m_pVertex) { lCnt++ ; } }
      smos_sprintf(sBuff, _T("\n  in list m_sTgtVertices %lu times"), lCnt) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff,_T("%s"), _T("\n  in list m_sTgtVertexProps")) ; smos_WriteBuffer(sBuff) ; 
    } 

  // various Heal VertexProps lists
  SM_INLIST(m_sBadVertexProps_SmallZoneTol3d) ;
  SM_INLIST(m_sBadVertexProps_LargeZoneTol3d) ;
  SM_INLIST(m_sWarnVertexProps_LargeGap3d   ) ;
  SM_INLIST(m_sBadVertexProps_CoinVertices  ) ;

  // End
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nEnd SmHealData DumpVertexListsContaining")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd SmHealData DumpVertexListsContaining   [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

#undef SM_STRINGIZE
#undef SM_INLIST

} // end SmHealData::DumpVertexListsContaining

/*******************************************************************//**
PURPOSE: For debugging - Pretty Print ListNames containing Tgt EdgeProps obj

NOTES:
***********************************************************************/
void SmHealData::DumpEdgeListsContaining
 (SmEdgeProps * pEdgeProps,  // in : Tgt EdgeProps obj
  ULONG         lLabel)      // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]

 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // SM_INLIST uses SM_STRINGIZE to print the prob-array symbol name in debug output.
  #define SM_STRINGIZE(a) #a
  #define SM_INLIST(a) if((a).m_sProbArray.IsIn(pEdgeProps)) { smos_sprintf(sBuff, _T("\n  in list %s"), _T(SM_STRINGIZE(a))) ; smos_WriteBuffer(sBuff) ; }

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff       , _T("\nBegin SmHealData DumpEdgeListsContaining pEdgeProps:[0x%p]"), 
                                                          pEdgeProps) ; 
                               }                        
  else                         { smos_sprintf(sBuff       , _T("\nBegin SmHealData DumpEdgeListsContaining [%3lu] for pEdgeProps:[0x%p]"),
                                                          lLabel, 
                                                          pEdgeProps) ;
                               }                        
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuffForFile, _T("\nBegin SmHealData DumpEdgeListsContaining pEdgeProps:[%s]"), 
                                                          pEdgeProps ? _T("NotNULL") : _T("NULL") ) ; 
                               }                        
  else                         { smos_sprintf(sBuffForFile, _T("\nBegin SmHealData DumpEdgeListsContaining [%3lu] for pEdgeProps:[%s]"),
                                                          lLabel, 
                                                          pEdgeProps ? _T("NotNULL") : _T("NULL") ) ;
                               }
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // check the lists
  // EdgeLists and associated EdgePropsList - they are a pair
  if(   pEdgeProps->m_pEdge  
     && m_sTgtEdges.IsIn((SmEdge*)pEdgeProps->m_pEdge))
    { 
      ULONG lCnt = 0 ;
      for(ULONG ii=0;ii<m_sTgtEdges.GetSize();ii++) { if(m_sTgtEdges[ii] == pEdgeProps->m_pEdge) { lCnt++ ; } }
      smos_sprintf(sBuff, _T("\n  in list m_sTgtEdges %lu times"), lCnt) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff,_T("%s"), _T("\n  in list m_sTgtEdgeProps")) ; smos_WriteBuffer(sBuff) ; 
    } 

  // various Heal EdgeProps lists
  SM_INLIST(m_sBadEdgeProps_Uncontained     ) ;
  SM_INLIST(m_sBadEdgeProps_SmallZoneTol3d  ) ;
  SM_INLIST(m_sBadEdgeProps_LargeZoneTol3d  ) ;
  SM_INLIST(m_sWarnEdgeProps_LargeGap3d     ) ;
  SM_INLIST(m_sBadEdgeProps_DegenEdges      ) ;
  SM_INLIST(m_sBadEdgeProps_CoinEdges       ) ;
  SM_INLIST(m_sBadEdgeProps_MissedEdgeXSects) ; 

  // End
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff,_T("%s"), _T("\nEnd SmHealData DumpEdgeListsContaining")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd SmHealData DumpEdgeListsContaining   [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

#undef SM_STRINGIZE
#undef SM_INLIST

} // end SmHealData::DumpEdgeListsContaining

/*******************************************************************//**
PURPOSE: For debugging - Pretty Print ListNames containing Tgt FaceProps obj

NOTES:
***********************************************************************/
void SmHealData::DumpFaceListsContaining
 (SmFaceProps * pFaceProps,  // in : Tgt FaceProps obj
  ULONG         lLabel)      // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]

 const
{
  // locals
  ULONG lTgtCnt = 0, lInCnt = 0 ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // SM_INLIST uses SM_STRINGIZE to print the prob-array symbol name in debug output.
  #define SM_STRINGIZE(a) #a
  #define SM_INLIST(a) if((a).m_sProbArray.IsIn(pFaceProps)) { smos_sprintf(sBuff, _T("\n    %s"), _T(SM_STRINGIZE(a))) ; smos_WriteBuffer(sBuff) ; lInCnt++ ; }

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nBegin SmHealData::DumpFaceListsContaining()") ) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin [%3lu] SmHealData::DumpFaceListsContaining()"), lLabel) ; }                       
  smos_WriteBuffer(sBuff) ;

  // header
  smos_sprintf(sBuff       , _T("\n  pFaceProps:[0x%p] is in Lists:"), pFaceProps) ; 
  smos_sprintf(sBuffForFile, _T("\n  pFaceProps:[%s] is in Lists:"), pFaceProps ? _T("NotNULL") : _T("NULL") ) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // check the lists
  // FaceLists and associated FacePropsList - they are a pair
  if(   pFaceProps->m_pFace  
     && m_sTgtFaces.IsIn(pFaceProps->m_pFace))
    { 
      ULONG lCnt = 0 ;
      for(ULONG ii=0;ii<m_sTgtFaces.GetSize();ii++) { if(m_sTgtFaces[ii] == pFaceProps->m_pFace) { lCnt++ ; } }
      smos_sprintf(sBuff, _T("\n    m_sTgtFaces     %lu time%s"), lCnt, lCnt==1 ? _T("") : _T("s")) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("%s"), _T("\n    m_sTgtFaceProps")) ; smos_WriteBuffer(sBuff) ; 
      if(lCnt > 0) { lTgtCnt++ ; }
    } 

  // various Heal FaceProps lists
  SM_INLIST(m_sNotYetFaceProps                ) ;
  SM_INLIST(m_sBadFaceProps_SmallZoneTol3d    ) ;
  SM_INLIST(m_sBadFaceProps_LargeZoneTol3d    ) ;
  SM_INLIST(m_sBadFaceProps_DegenFaces        ) ;
  SM_INLIST(m_sFaceProps_Sheets               ) ;
  SM_INLIST(m_sFaceProps_NoVertexPole         ) ;
  SM_INLIST(m_sBadFaceProps_FlatCorner        ) ;
  SM_INLIST(m_sBadFaceProps_MissingSeam       ) ;
  SM_INLIST(m_sBadFaceProps_CrossedSeam       ) ;
  SM_INLIST(m_sBadFaceProps_NearMissSeam      ) ;
  SM_INLIST(m_sRanFaceProps_MoveSeam          ) ;
  SM_INLIST(m_sRanFaceProps_SplitEdgesAtSeam  ) ;
  SM_INLIST(m_sBadFaceProps_LoopProblems      ) ;
  SM_INLIST(m_sBadFaceProps_OuterLoopOrder    ) ;
  SM_INLIST(m_sBadFaceProps_OrientLoop        ) ;
  SM_INLIST(m_sBadFaceProps_NoArea_Loop       ) ;
  SM_INLIST(m_sBadFaceProps_MultiOuterLoops   ) ;
  SM_INLIST(m_sBadFaceProps_NestedOuterLoops  ) ;
  SM_INLIST(m_sBadFaceProps_UnpairedNoAreaLoop) ;
  SM_INLIST(m_sBadFaceProps_Closed3dLoop      ) ;
  SM_INLIST(m_sBadFaceProps_ClosedPtrLoop     ) ;
  SM_INLIST(m_sBadFaceProps_MissingPole       ) ;
  SM_INLIST(m_sRanFaceProps_SplitFaceAtSeam   ) ;

  // summary
  if(lTgtCnt > 0) { smos_sprintf(sBuff, _T("\n  Summary: in TgtList, In %ld %sProbArray%s"), lInCnt, lInCnt > 1 ? _T("different ") : _T(""), lInCnt > 1 ? _T("s") : _T("") ) ; }
  else            { smos_sprintf(sBuff, _T("%s"),_T("\n  Summary: NOT in TgtList")) ; }
  smos_WriteBuffer(sBuff) ;

  // End
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nEnd SmHealData::DumpFaceListsContaining( pFaceProps )")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd [%3ld] SmHealData::DumpFaceListsContaining( pFaceProps )"), lLabel) ; }
  smos_WriteBuffer(sBuff);

#undef SM_STRINGIZE
#undef SM_INLIST

} // end SmHealData::DumpFaceListsContaining

/*******************************************************************//**
PURPOSE: Print SmHealData arrays that contain any members of the given Tgt list

NOTES:
***********************************************************************/
void SmHealData::DumpArraysContaining
  (ULONG                      lLabel,          // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
  SmTArray<SmFaceProps *>   * pTgtFaceProps,   // in : list of tgt faces for searches
  SmTArray<SmEdgeProps *>   * pTgtEdgeProps,   // in : list of tgt edges for searches
  SmTArray<SmVertexProps *> * pTgtVertexProps, // in : list of tgt vertices for searches
  SmBoolean                   bRptNoTgtArrays) // in : TRUE  = PrettyPrint all array reports 
                                               //      FALSE = Only PrettyPrint array reports for arrays containing one or more Tgts
 const                                         //      default:[FALSE]
{
  ULONG ii, lRtn ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  ULONG lTgtsInArrays = 0, lArryCnt = 0 ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nBegin ContainingArrays Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin ContainingArrays Dump [%3lu]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // begin scope - pretty print the input targets
    {                                                                                                           
      // input tgt Faces
      ULONG lCnt = pTgtFaceProps->GetSize() ;
      smos_sprintf(sBuff,        _T("\n Tgt Face Array  :[0x%p] Size:[%3lu]: %s:["), pTgtFaceProps, lCnt,
                                                                                     lCnt==0 ? _T("no Tgted Members") 
                                                                                   : lCnt==1 ? _T("   Tgted Member ")
                                                                                   :           _T("   Tgted Members")) ;  
      smos_sprintf(sBuffForFile, _T("\n Tgt Face Array  :[%s]   Size:[%3lu]: %s:["), pTgtFaceProps ? _T("notNULL") : _T("NULL"), lCnt,
                                                                                     lCnt==0 ? _T("no Tgted Members") 
                                                                                   : lCnt==1 ? _T("   Tgted Member ")
                                                                                   :           _T("   Tgted Members")) ;  
      smos_WriteBuffer(sBuff, sBuffForFile) ;
      if(lCnt == 0) { smos_WriteBuffer(_T("none]"), _T("none]")) ; } 
      else 
        { for(ii=0;ii<pTgtFaceProps->GetSize();ii++) { smos_sprintf(sBuff,        _T("0x%p"), pTgtFaceProps->GetAt(ii)) ;  
                                                       smos_sprintf(sBuffForFile, _T("%s"), pTgtFaceProps->GetAt(ii) ? _T("notNULL") : _T("NULL")) ;  
                                                       smos_WriteBuffer(sBuff, sBuffForFile) ;
                                                       if(ii+1 == pTgtFaceProps->GetSize())  { smos_WriteBuffer(_T("]"), _T("]")) ; } 
                                                       else                                  { smos_WriteBuffer(_T(", "), _T(", ")) ; } 
                                                     }
        }

      // input tgt Edges
      lCnt = pTgtEdgeProps->GetSize() ;
      smos_sprintf(sBuff,        _T("\n Tgt Edge Array  :[0x%p] Size:[%3lu]: %s:["), pTgtEdgeProps, lCnt,
                                                                                     lCnt==0 ? _T("no Tgted Members") 
                                                                                   : lCnt==1 ? _T("   Tgted Member ")
                                                                                   :           _T("   Tgted Members")) ;  
      smos_sprintf(sBuffForFile, _T("\n Tgt Edge Array  :[%s]   Size:[%3lu]: %s:["), pTgtEdgeProps ? _T("notNULL") : _T("NULL"), lCnt,
                                                                                     lCnt==0 ? _T("no Tgted Members") 
                                                                                   : lCnt==1 ? _T("   Tgted Member ")
                                                                                   :           _T("   Tgted Members")) ;  
      smos_WriteBuffer(sBuff, sBuffForFile) ;
      if(lCnt == 0) { smos_WriteBuffer(_T("none]"), _T("none]")) ; } 
      else 
        { for(ii=0;ii<pTgtEdgeProps->GetSize();ii++) { smos_sprintf(sBuff,        _T("0x%p"), pTgtEdgeProps->GetAt(ii)) ;  
                                                       smos_sprintf(sBuffForFile, _T("%s"), pTgtEdgeProps->GetAt(ii) ? _T("notNULL") : _T("NULL")) ;  
                                                       smos_WriteBuffer(sBuff, sBuffForFile) ;
                                                       if(ii+1 == pTgtEdgeProps->GetSize())  { smos_WriteBuffer(_T("]"), _T("]")) ; } 
                                                       else                                  { smos_WriteBuffer(_T(", "), _T(", ")) ; } 
                                                     }
        }

      // input tgt Vertices
      lCnt = pTgtVertexProps->GetSize() ;
      smos_sprintf(sBuff,        _T("\n Tgt Vertex Array:[0x%p] Size:[%3lu]: %s:["), pTgtVertexProps, lCnt,
                                                                                     lCnt==0 ? _T("no Tgted Members") 
                                                                                   : lCnt==1 ? _T("   Tgted Member ")
                                                                                   :           _T("   Tgted Members")) ;  
      smos_sprintf(sBuffForFile, _T("\n Tgt Vertex Array:[%s]   Size:[%3lu]: %s:["), pTgtVertexProps ? _T("notNULL") : _T("NULL"), lCnt,
                                                                                     lCnt==0 ? _T("no Tgted Members") 
                                                                                   : lCnt==1 ? _T("   Tgted Member ")
                                                                                   :           _T("   Tgted Members")) ;  
      smos_WriteBuffer(sBuff, sBuffForFile) ;
      if(lCnt == 0) { smos_WriteBuffer(_T("none]"), _T("none]")) ; } 
      else 
        { for(ii=0;ii<pTgtVertexProps->GetSize();ii++) { smos_sprintf(sBuff,        _T("0x%p"), pTgtVertexProps->GetAt(ii)) ;  
                                                         smos_sprintf(sBuffForFile, _T("%s"), pTgtVertexProps->GetAt(ii) ? _T("notNULL") : _T("NULL")) ;  
                                                         smos_WriteBuffer(sBuff, sBuffForFile) ;
                                                         if(ii+1 == pTgtVertexProps->GetSize())  { smos_WriteBuffer(_T("]"), _T("]")) ; } 
                                                         else                                    { smos_WriteBuffer(_T(", "), _T(", ")) ; } 
                                                       }
        }
    } // end scope - pretty print the input targets

  // begin scope - pretty print arrays containing any targets
    {
      // locals
      SmTArray<SmFace *>   sTgtFaces,    sCommonFaces ;
      SmTArray<SmEdge *>   sTgtEdges,    sCommonEdges ;
      SmTArray<SmVertex *> sTgtVertices, sCommonVertices ;

      // build the TgtFaces, TgtEdges, TgtVertices lists from the input TgtProps lists
      for(ii=0;ii<pTgtFaceProps->GetSize();ii++)   { sTgtFaces.   Add(pTgtFaceProps->  GetAt(ii)->m_pFace) ;  }  
      for(ii=0;ii<pTgtEdgeProps->GetSize();ii++)   { sTgtEdges.   Add(pTgtEdgeProps->  GetAt(ii)->m_pEdge) ;  }  
      for(ii=0;ii<pTgtVertexProps->GetSize();ii++) { sTgtVertices.Add(pTgtVertexProps->GetAt(ii)->m_pVertex) ;  } 

      smos_WriteBuffer(_T("\n The ProbArrays containing any of these targets include:")) ;

// MEMBER LOCAL MACRO 
// SM_RPT_TARGETS - use on any Array containing ptrs to objs derived from SmTopology
//  arguments: NameStr - Name label for report line.  not used by SmProbArrays, instead SmProbArrays.m_sOutputLabel is used
//             Array   - Arry in IsHealData to study and report
//             Tgts    - Array of possible Tgts to be found within Array
//             Result  - a scratch array or the same types as the elemn in Array
#define SM_RPT_TARGETS(NameStr, Array, Tgts, Result)  \
      { (Array).FindCommonElements((Tgts), (Result)) ; \
        if((Result).GetSize() == 0) \
             { if(bRptNoTgtArrays == TRUE) \
                 { smos_sprintf(sBuff, _T("\n sized:[%3lu] %s contains:[no ] Tgted members: [none]"), (Array).GetSize(), NameStr) ; smos_WriteBuffer(sBuff) ; } \
             } \
        else { smos_sprintf(sBuff, _T("\n sized:[%3lu] %s contains:[%3lu] Tgted members: ["),     (Array).GetSize(), NameStr, (Result).GetSize()) ; smos_WriteBuffer(sBuff) ; \
               for(ULONG mi=0;mi<(Result).GetSize();mi++) \
                 { smos_sprintf(sBuff,        _T("0x%p"), (Result)[mi]) ; \
                   smos_sprintf(sBuffForFile, _T("%s"),   (Result)[mi] ? _T("notNULL") : _T("NULL")) ; \
                   smos_WriteBuffer(sBuff, sBuffForFile) ; \
                   if(mi+1 == (Result).GetSize()) { smos_WriteBuffer(_T("]"), _T("]")) ; } \
                   else                           { smos_WriteBuffer(_T(", "), _T(", ")) ; } \
                 } \
              } \
       }            

#define SM_DUMP_AND_COUNT(sArray, pTgtProps)              \
  { lRtn = sArray.DumpTgts(*pTgtProps, bRptNoTgtArrays) ; \
    if(lRtn > 0) { lTgtsInArrays += lRtn ; lArryCnt++ ; } \
  }
// END MEMBER LOCAL MACRO

      // for every Done Heal Op - pretty print status
      for(ii=SM_HO_NONE;ii<=SM_HO_ALL;ii++)
        {
          // output data for completed steps
          switch(ii)
            {
              case SM_HO_NONE                    : { // new objects
                                                     SM_RPT_TARGETS(_T("NewFaces_FromFixDegenFaces      "), m_sNewFaces_FromFixDegenFaces          , sTgtFaces,    sCommonFaces) ;    
                                                     SM_RPT_TARGETS(_T("NewEdges_FromSplitAtSeams       "), m_sNewEdges_FromSplitEdgesAtSeams      , sTgtEdges,    sCommonEdges) ;      
                                                     SM_RPT_TARGETS(_T("NewVertices_FromSplitAtSeams    "), m_sNewVertices_FromSplitEdgesAtSeams   , sTgtVertices, sCommonVertices) ;  
                                                     SM_RPT_TARGETS(_T("NewVertices_OnPoles             "), m_sNewVerticesOnPoles_FromFixBadLoops                 , sTgtVertices, sCommonVertices) ;   
                    // not yet checking for loops  : SM_RPT_TARGETS(_T("NewLoops_OnPoles                "), m_sNewLoopsOnPoles_FromFixBadLoops                    , sTgtLoop,     sCommonLoops) ;
                                                     SM_RPT_TARGETS(_T("NewFaces_FromExtraLoops         "), m_sNewFaces_FromExtraLoops             , sTgtFaces,    sCommonFaces) ;       
                                                     SM_RPT_TARGETS(_T("NewFaces_FromSplitAtSeams       "), m_sNewFaces_FromSplitFaceAtSeams       , sTgtFaces,    sCommonFaces) ;       
                                                     SM_RPT_TARGETS(_T("NewEdgesOnSeams_FromSplitAtSeams"), m_sNewEdgesOnSeams_FromSplitFaceAtSeams, sTgtEdges,    sCommonEdges) ;     
                                                     SM_RPT_TARGETS(_T("NewVertices_FromSplitAtSeams    "), m_sNewVertices_FromSplitFaceAtSeams    , sTgtVertices, sCommonVertices) ;  
                    // not yet checking for regions: SM_RPT_TARGETS(_T("NewRegions_FromClosedSheets     "), m_sNewRegions_FromClosedSheets         , sTgtRegions,  sCommonRegions) ;     
                    // not yet checking for shells : SM_RPT_TARGETS(_T("NewShells_FromClosedSheets      "), m_sNewShells_FromClosedSheets          , sTgtShells,   sCommonShells) ;      
                                                   } break ; // end case SM_HO_NONE

              case SM_HO_CACHE_EDGEPROPS         : { SM_DUMP_AND_COUNT(m_sBadEdgeProps_Uncontained   , pTgtEdgeProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadEdgeProps_SmallZoneTol3d, pTgtEdgeProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadEdgeProps_LargeZoneTol3d, pTgtEdgeProps) ;
                                                     SM_DUMP_AND_COUNT(m_sWarnEdgeProps_LargeGap3d   , pTgtEdgeProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadEdgeProps_DegenEdges    , pTgtEdgeProps) ;

                                                   } break ; // end case SM_HO_CACHE_EDGEPROPS
                                            
              case SM_HO_CACHE_VERTEXPROPS       : { SM_DUMP_AND_COUNT(m_sBadVertexProps_SmallZoneTol3d, pTgtVertexProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadVertexProps_LargeZoneTol3d, pTgtVertexProps) ;
                                                     SM_DUMP_AND_COUNT(m_sWarnVertexProps_LargeGap3d   , pTgtVertexProps) ;

                                                   } break ; // end case SM_HO_CACHE_VERTEXPROPS
                                             
              case SM_HO_CACHE_FACEPROPS_GAPS    : { SM_DUMP_AND_COUNT(m_sBadFaceProps_SmallZoneTol3d, pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_LargeZoneTol3d, pTgtFaceProps) ;
                                                   } break ; // end case SM_HO_CACHE_FACEPROPS_GAPS
                                             
              case SM_HO_FIX_TOLSIZES            : { } break ; // end case SM_HO_FIX_TOLSIZES    
          
              case SM_HO_CACHE_COIN_VERTICES     : { SM_DUMP_AND_COUNT(m_sBadVertexProps_CoinVertices, pTgtVertexProps) ;
                                                   } break ;   // end case SM_HO_FIX_COIN_VERTICES
                                             
              case SM_HO_FIX_COIN_VERTICES       : { } break ; // end case SM_HO_FIX_COIN_VERTICES
                                             
              case SM_HO_CACHE_DEGEN_FACES       : { SM_DUMP_AND_COUNT(m_sBadFaceProps_DegenFaces, pTgtFaceProps) ;
                                                   } break ;   // end case SM_HO_FIX_DEGEN_FACES

              case SM_HO_FIX_DEGEN_FACES         : { } break ; // end case SM_HO_FIX_DEGEN_FACES
                                                
              case SM_HO_FIX_DEGEN_EDGES         : { } break ; // end case SM_HO_FIX_DEGEN_EDGES  
                                             
              case SM_HO_CACHE_COIN_EDGES        : { SM_DUMP_AND_COUNT(m_sBadEdgeProps_CoinEdges, pTgtEdgeProps) ;
                                                   } break ; // end case SM_HO_FIX_COIN_EDGES
              case SM_HO_FIX_COIN_EDGES          : { } break ; // end case SM_HO_FIX_COIN_EDGES
                                             
              case SM_HO_CACHE_MISSED_EDGEXSECTS : { } break ; // end case SM_HO_CACHE_MISSED_EDGEXSECTS
              case SM_HO_FIX_MISSED_EDGEXSECTS   : { } break ; // end case SM_HO_FIX_MISSED_EDGEXSECTS
                                             
              case SM_HO_FIX_UNCONTAINED_EDGES   : { } break ; // end case SM_HO_FIX_UNCONTAINED_EDGES
              case SM_HO_FIX_BADGAPS             : { } break ; // end case SM_HO_FIX_BADGAPS
          
              case SM_HO_CACHE_FACEPROPS_2       : { SM_DUMP_AND_COUNT(m_sFaceProps_Sheets         , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sFaceProps_NoVertexPole   , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_FlatCorner  , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_MissingSeam , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_CrossedSeam , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_NearMissSeam, pTgtFaceProps) ;

                                                   } break ; // end case SM_HO_CACHE_FACEPROPS_2   
                                             
              case SM_HO_FIX_MOVESEAM            : { SM_DUMP_AND_COUNT(m_sRanFaceProps_MoveSeam, pTgtFaceProps) ;
                                                   } break ; // end case SM_HO_FIX_MOVESEAM   
                                                
              case SM_HO_FIX_SPLITEDGE_ATSEAM    : { SM_DUMP_AND_COUNT(m_sRanFaceProps_SplitEdgesAtSeam, pTgtFaceProps) ;
                                                   } break ; // end case SM_HO_FIX_SPLITEDGE_ATSEAM 
                                                
              case SM_HO_FIX_BADSHEETS           : { } break ; // end case SM_HO_FIX_BADSHEETS  
                                             
              case SM_HO_CACHE_FACEPROPS_3       : { SM_DUMP_AND_COUNT(m_sBadFaceProps_LoopProblems      , pTgtFaceProps) ;     
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_OuterLoopOrder    , pTgtFaceProps) ;   
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_OrientLoop        , pTgtFaceProps) ;       
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_NoArea_Loop       , pTgtFaceProps) ;      
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_MultiOuterLoops   , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_NestedOuterLoops  , pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_UnpairedNoAreaLoop, pTgtFaceProps) ;
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_Closed3dLoop      , pTgtFaceProps) ;     
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_ClosedPtrLoop     , pTgtFaceProps) ;    
                                                     SM_DUMP_AND_COUNT(m_sBadFaceProps_MissingPole       , pTgtFaceProps) ;      
                                                   } break ; // end case SM_HO_FIX_BADLOOPS  

              case SM_HO_FIX_SPLITFACE_ATSEAMS   : { SM_DUMP_AND_COUNT(m_sRanFaceProps_SplitFaceAtSeam, pTgtFaceProps) ;
                                                   } break ; // end case SM_HO_FIX_SPLITFACE_ATSEAMS
                                                 
              case SM_HO_FIX_BADLOOPS            : { } break ; // end case SM_HO_FIX_BADLOOPS 
                                                 
              case SM_HO_MAKE_UVTRIMCURVES       : { } break ; // end case SM_HO_MAKE_UVTRIMCURVES    
              case SM_HO_FIX_INFINITE_REGIONS    : { } break ; // end case SM_HO_FIX_INFINITE_REGIONS 
              default: break ; 

            } // end switch(ii) - Pretty Print data for SmHealData step ii
        } // end iter ii - every completed SmHealData step

      // summary
      if(lTgtsInArrays == 0) { smos_sprintf(sBuff, _T("%s"),_T("\n  Summary: Cnt:[no ] Tgts were in any ProbArrays")) ; }
      else                   { smos_sprintf(sBuff, _T("\n  Summary: Cnt:[%3lu] Tgts were found in cnt:[%3lu] ProbArrays"), lTgtsInArrays, lArryCnt) ; }
      smos_WriteBuffer(sBuff);

    } // end scope - pretty print arrays containing any targets

  // End
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff,_T("%s"), _T("\nEnd ContainingArrays Dump")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd ContainingArrays Dump [%3lu]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

#undef SM_RPT_TARGETS 
#undef SM_DUMP_AND_COUNT

} // end SmHealData::DumpArraysContaining
