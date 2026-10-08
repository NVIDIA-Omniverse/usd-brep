// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmEdgeProps.cpp
* PURPOSE: Source file for implementation of SmEdgeProps methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmHealData.h>
#include <SmEdgeProps.h>
#include <SmFaceProps.h>
#include <SmVertexProps.h>
#include <SmEdge.h>
#include <SmCurve.h>
#include <SmTol.h>
#include <SmGraphicsOutput.h>
#ifdef SM_DEBUG_CODE
  #include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: SmEdgeProps Method implementations
***********************************************************************/

/*******************************************************************//**
 PURPOSE:
 NOTES:
***********************************************************************/
SmBoolean SmEdgeProps::IsKindOf( SM_TYPE t ) const
{
  return ((SmEdgeProps_TYPE == t) ? TRUE : SmObjProps::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: SmEdgeProps Default Constructor

NOTES: 
***********************************************************************/
SmEdgeProps::SmEdgeProps(const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  ReSet() ; 

} // end SmEdgeProps::SmEdgeProps default constructor

/*******************************************************************//**
PURPOSE: SmEdgeProps Constructor

NOTES: 
***********************************************************************/
SmEdgeProps::SmEdgeProps(SmEdge * pEdge, ULONG lEdgeIndx, const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  ReSet() ; 
  m_pEdge     = pEdge ; 
  m_lEdgeIndx = lEdgeIndx ; 
//  if(m_pEdge) { m_pEdge->SetEdgeProps(this) ; }

} // end SmEdgeProps::SmEdgeProps default constructor

/*******************************************************************//**
PURPOSE: SmEdgeProps Copy Constructor

NOTES: 
***********************************************************************/
SmEdgeProps::SmEdgeProps(const SmEdgeProps & crOther) 
{
  // locals
  m_cpContext = crOther.m_cpContext ; 

  m_pEdge               = crOther.m_pEdge ; 
  m_lEdgeIndx           = crOther.m_lEdgeIndx ;
  m_sEdgeInterval       = crOther.m_sEdgeInterval ;     
  m_sZoneTol3d          = crOther.m_sZoneTol3d ;       
  m_sOrigZoneTol3d      = crOther.m_sOrigZoneTol3d ;       
  m_dApproxEdgeLength3d = crOther.m_dApproxEdgeLength3d ;            
  m_cpCurve             = crOther.m_cpCurve ;            
  m_sNaturalInterval    = crOther.m_sNaturalInterval ; 
  
  m_pMaxGap3d_EdgeEdge  = crOther.m_pMaxGap3d_EdgeEdge ;   
  m_pMaxGap3d_EdgeFace  = crOther.m_pMaxGap3d_EdgeFace ;   
  m_bBadSmallZoneTol3d  = crOther.m_bBadSmallZoneTol3d ;
  m_bBadLargeZoneTol3d  = crOther.m_bBadLargeZoneTol3d ;
  m_bWarnLargeGap3d     = crOther.m_bWarnLargeGap3d ;
  
  m_bBadDegenEdge       = crOther.m_bBadDegenEdge ;     
  m_bBadUncontainedEdge = crOther.m_bBadUncontainedEdge ;
  m_bBadCoincidentEdge  = crOther.m_bBadCoincidentEdge ;
  m_bBadMissedEdgeXSect = crOther.m_bBadMissedEdgeXSect ;

} // end SmEdgeProps::SmEdgeProps copy constructor

/*******************************************************************//**
PURPOSE: SmEdgeProps assignment operator

NOTES: 
***********************************************************************/
SmEdgeProps & SmEdgeProps::operator= (const SmEdgeProps &crOther)
{
  // no work - same object
  if(this == &crOther)
    { return *this ; }

  m_cpContext           = crOther.m_cpContext ; 
                       
  m_pEdge               = crOther.m_pEdge ;   
  m_lEdgeIndx           = crOther.m_lEdgeIndx ;
  m_sEdgeInterval       = crOther.m_sEdgeInterval ;     
  m_sZoneTol3d          = crOther.m_sZoneTol3d ;       
  m_sOrigZoneTol3d      = crOther.m_sOrigZoneTol3d ;       
  m_dApproxEdgeLength3d = crOther.m_dApproxEdgeLength3d ;            
  m_cpCurve             = crOther.m_cpCurve ;            
  m_sNaturalInterval    = crOther.m_sNaturalInterval ;   
  
  m_pMaxGap3d_EdgeEdge  = crOther.m_pMaxGap3d_EdgeEdge ;   
  m_pMaxGap3d_EdgeFace  = crOther.m_pMaxGap3d_EdgeFace ;   
  m_bBadSmallZoneTol3d  = crOther.m_bBadSmallZoneTol3d ;
  m_bBadLargeZoneTol3d  = crOther.m_bBadLargeZoneTol3d ;
  m_bWarnLargeGap3d     = crOther.m_bWarnLargeGap3d ;
  
  m_bBadDegenEdge       = crOther.m_bBadDegenEdge ;     
  m_bBadUncontainedEdge = crOther.m_bBadUncontainedEdge ;
  m_bBadCoincidentEdge  = crOther.m_bBadCoincidentEdge ; 
  m_bBadMissedEdgeXSect = crOther.m_bBadMissedEdgeXSect ;

  // all done
  return(*this) ;
  
} // end SmEdgeProps::operator= assignment operator

/*******************************************************************//**
PURPOSE: SmEdgeProps equality operator

NOTES: 
***********************************************************************/
SmBoolean SmEdgeProps::operator== (const SmEdgeProps &crOther) const
{
  // locals
  SmBoolean bRtn = TRUE ; 

  // no work - same object
  if(this == &crOther)
    { return bRtn ; }

  bRtn &= m_pEdge               == crOther.m_pEdge ;
  bRtn &= m_lEdgeIndx           == crOther.m_lEdgeIndx ;
  bRtn &= m_sEdgeInterval       == crOther.m_sEdgeInterval ;     
  bRtn &= m_sZoneTol3d          == crOther.m_sZoneTol3d ;       
  bRtn &= m_sOrigZoneTol3d      == crOther.m_sOrigZoneTol3d ;       
  bRtn &= m_dApproxEdgeLength3d == crOther.m_dApproxEdgeLength3d ;            
  bRtn &= m_cpCurve             == crOther.m_cpCurve ;            
  bRtn &= m_sNaturalInterval    == crOther.m_sNaturalInterval ;   
  
  bRtn &= m_pMaxGap3d_EdgeEdge  == crOther.m_pMaxGap3d_EdgeEdge ;   
  bRtn &= m_pMaxGap3d_EdgeFace  == crOther.m_pMaxGap3d_EdgeFace ;   
  bRtn &= m_bBadSmallZoneTol3d  == crOther.m_bBadSmallZoneTol3d ;
  bRtn &= m_bBadLargeZoneTol3d  == crOther.m_bBadLargeZoneTol3d ;
  bRtn &= m_bWarnLargeGap3d     == crOther.m_bWarnLargeGap3d ;
  
  bRtn &= m_bBadDegenEdge       == crOther.m_bBadDegenEdge ;     
  bRtn &= m_bBadUncontainedEdge == crOther.m_bBadUncontainedEdge ;
  bRtn &= m_bBadCoincidentEdge  == crOther.m_bBadCoincidentEdge ;
  bRtn &= m_bBadMissedEdgeXSect == crOther.m_bBadMissedEdgeXSect ;

  // all done
  return(bRtn) ;
  
} // end SmEdgeProps::operator== equality operator

/*******************************************************************//**
PURPOSE: initialize SmEdgeProps all member values

NOTES: This method resets newly allocated SmEdgeProps and previously
       used SmEdgeProps objects back to the same uninitialized state.

       But leaves the m_cpContext unchanged. 
       (Simplifies reusing an SmEdgeProps object)
***********************************************************************/
void SmEdgeProps::ReSet                                      
 (SmBoolean bResetEdge_Metadata) // in : default:[TRUE] = reset both SetProps() and SetProps_Metadata() properties
                                 //      FALSE          = reset only SetProps() properties
{                              
  // nothing to do for Context

  // SetProps_MetaData()
  if(bResetEdge_Metadata)
    { 
      // if(m_pEdge) { m_pEdge->SetEdgeProps(NULL) ; }
      m_pEdge                 = NULL ;
      m_lEdgeIndx             = SM_UNDEF_ULONG ;

      m_sEdgeInterval.Init() ;
      m_sZoneTol3d            = SM_UNDEF_DOUBLE ;
      m_sOrigZoneTol3d        = SM_UNDEF_DOUBLE ;
      m_dApproxEdgeLength3d   = SM_UNDEF_DOUBLE ;

      m_cpCurve               = NULL ;
      m_sNaturalInterval.Init() ; 
  
      m_bBadDegenEdge         = UNSURE ; 
      m_bBadUncontainedEdge   = UNSURE ; 
    } 

  // SetProps()
  m_pMaxGap3d_EdgeEdge        = NULL ;    // set:[SetProps()]
  m_pMaxGap3d_EdgeFace        = NULL ;    // set:[SetProps()]
  m_bBadSmallZoneTol3d        = UNSURE ;  // set:[SetProps()]
  m_bBadLargeZoneTol3d        = UNSURE ;  // set:[SetProps()]
  m_bWarnLargeGap3d           = UNSURE ;  // set:[SetProps()]

  // Other
  m_bBadCoincidentEdge        = UNSURE ;  // set:[SmHealData::Cache_CoinVertices()]
  m_bBadMissedEdgeXSect       = UNSURE ;  // Set:[SmHealData::Cache_MissedEdgeXSects()]
                       
} // end SmEdgeProps::ReSet

/*******************************************************************//**
PURPOSE: Return TRUE if member values have been set

NOTES: Member values are set with a call to SetProps() 
***********************************************************************/
SmBoolean SmEdgeProps::IsSet_Metadata() const
{
  SmBoolean bIsSet = TRUE ; 

  // member values
  bIsSet &= m_pEdge                                != NULL ;
 // bIsSet &= m_lEdgeIndx                            != SM_UNDEF_ULONG ;  // don't check optional value
  bIsSet &= m_sEdgeInterval.HasNegativeLength()    == FALSE ;

  bIsSet &= m_sZoneTol3d                           != SM_UNDEF_DOUBLE ;
  bIsSet &= m_sOrigZoneTol3d                       != SM_UNDEF_DOUBLE ;
  bIsSet &= m_dApproxEdgeLength3d                  != SM_UNDEF_DOUBLE ;

  bIsSet &= m_cpCurve                              != NULL ;
  bIsSet &= m_sNaturalInterval.HasNegativeLength() == FALSE ; 
  
  bIsSet &= m_bBadDegenEdge                        != UNSURE ; 
  bIsSet &= m_bBadUncontainedEdge                  != UNSURE ;
  
  return(bIsSet) ;
                       
} // end SmEdgeProps::IsSet_Metadata

/*******************************************************************//**
PURPOSE: Return TRUE if member values have been set

NOTES: Member values are set with a call to SetProps() 
***********************************************************************/
SmBoolean SmEdgeProps::IsSet_Gaps() const
{
  SmBoolean bIsSet = TRUE ; 

  // member values
  bIsSet &= m_pMaxGap3d_EdgeEdge != NULL ;   
  bIsSet &= m_pMaxGap3d_EdgeFace != NULL ;  
  
  bIsSet &= m_bBadSmallZoneTol3d != UNSURE ;
  bIsSet &= m_bBadLargeZoneTol3d != UNSURE ;
  bIsSet &= m_bWarnLargeGap3d    != UNSURE ;
  
  return(bIsSet) ;
                       
} // end SmEdgeProps::IsSet_Gaps

/*******************************************************************//**
PURPOSE: Return TRUE if obj has any problems

NOTES: 1. Only Set member values are checked
       2. All Member values are set by SetProps() 
       3. Only Size member values are set by SetProps_Metadata()
       4. An Unset object has no Problems and FALSE is returned
***********************************************************************/
SmBoolean SmEdgeProps::HasProblems() const
{
  // return value
  SmBoolean bBad = FALSE ; 

  // check problem list 
  bBad |= m_bBadSmallZoneTol3d  == TRUE ;
  bBad |= m_bBadLargeZoneTol3d  == TRUE ;
  bBad |= m_bWarnLargeGap3d     == TRUE ;

  bBad |= m_bBadDegenEdge       == TRUE ;
  bBad |= m_bBadUncontainedEdge == TRUE ;
        
  bBad |= m_bBadCoincidentEdge  == TRUE ;

  bBad |= m_bBadMissedEdgeXSect == TRUE ;

  // all done
  return(bBad) ;
                       
} // end SmEdgeProps::HasProblems

/*******************************************************************//**
PURPOSE: set all SmEdgeProps Gap data, If needed also sets SmEdgeProps
         Metadata.

NOTES: This method resets newly allocated SmEdgeProps and previously
       used SmEdgeProps objects back to the same uninitialized state.
***********************************************************************/
SmStatus SmEdgeProps::SetProps
  (SmEdge          * cpEdge,                      // in : tgt Edge
   ULONG             lEdgeIndx,                   // NotUsed: in : associated Indx of cpEdge in managing SmHeadData::m_TgtEdges list
   SmTArray<ULONG> * pOptLoopGap3d_Histogram,     // i/o: accumulating Loop EdgeuseEnd/EdgeuseEnd Gap3d histogram, NULL to ignore
   SmTArray<ULONG> * pOptEdgeFaceGap3d_Histogram, // i/o: accumulating Edge/Face Gap3d histogram, NULL to ignore
   ULONG             lOptLabel)                   // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
                                                  // eff: if(Unset) { SetProps_Metadata() to set EdgeProps:
                                                  //                  m_pEdge m_pEdge        m_lEdgeIndx
                                                  //                  m_sEdgeInterval         //m_pEdge->m_pEdgeProps   
                                                  //                  m_sZoneTol3d    m_sOrigZoneTol3d        m_dApproxEdgeLength3d    
                                                  //                  m_cpCurve       m_sNaturalInterval      
                                                  //                  m_bBadDegenEdge m_bBadUncontainedEdge
                                                  //                }
                                                  //      then sets EdgeProps:
                                                  //                { m_pMaxGap3d_EdgeEdge m_bBadSmallZoneTol3d  m_bWarnLargeGap3d
                                                  //                  m_pMaxGap3d_EdgeFace m_bBadLargeZoneTol3d  m_bBadUncontainedEdge
                                                  //                }
                                                  //     note: m_bBadCoincidentEdge is not set by this call. That's done by the SmHealData::Cache_CoinEdges() call.
                                                  //           m_bBadMissedEdgeXSect is not set by ths call. That's done by the SmHealData::Cache_MissedEdgeXSects() call.

{
SM_REF1(lEdgeIndx) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe1 = FALSE ; 
SmBoolean bDebugMe2 = FALSE ; 
#endif // SM_DEBUG_CODE

  // Calc and Cache EdgeProps <== HERE
  if(FALSE == IsSet_Metadata())
    { SetProps_Metadata(cpEdge, lOptLabel) ; }
      
  // clear SmEdgeProps_Gap data - preserve the Metadata just computed
  ReSet(FALSE) ;  // FALSE = don't reset EdgeProps_Metadata

  // nothing to do here for m_cpContext

  // Get MaxGap3ds from Edge->Edgeuses Gap3d
  ULONG ii ; 
  SmTArray<SmEdgeuse *> sEdgeuses ;
  cpEdge->GetEdgeuses(sEdgeuses) ;

  // for every Edge->Edgeuse - Save MaxGap3d seen
  for(ii=0;ii<sEdgeuses.GetSize();ii++)
    {
      SmEdgeuse     * pEdgeuse        = sEdgeuses[ii] ;
      SmEdgeEdgeGap * pEdgeCCWEdgeGap = pEdgeuse->GetEdgeCCWEdgeGap() ;
      SmEdgeFaceGap * pEdgeFaceGap    = pEdgeuse->GetMaxEdgeFaceGap() ;
      
      // save Max EdgeEndPt/CCWEdgeStartPt Gap
      if(pEdgeCCWEdgeGap)
        {
          if(   m_pMaxGap3d_EdgeEdge == NULL
             || m_pMaxGap3d_EdgeEdge->GetLength() < pEdgeCCWEdgeGap->GetLength())
            { m_pMaxGap3d_EdgeEdge = pEdgeCCWEdgeGap ; }

          if(pOptLoopGap3d_Histogram)
            { (*pOptLoopGap3d_Histogram)[SM_HISTOGRAM_GAPSIZE_INDEX(pEdgeCCWEdgeGap->GetLength())]++ ; }
        }

      // save Max Edge/Face Gap
      if(pEdgeFaceGap)
        {
          if(   m_pMaxGap3d_EdgeFace == NULL
             || m_pMaxGap3d_EdgeFace->GetLength() < pEdgeFaceGap->GetLength())
            { m_pMaxGap3d_EdgeFace = pEdgeFaceGap ; }

          if(pOptEdgeFaceGap3d_Histogram)
            { (*pOptEdgeFaceGap3d_Histogram)[SM_HISTOGRAM_GAPSIZE_INDEX(pEdgeFaceGap->GetLength())]++ ; }
        }

#ifdef SM_DEBUG_CODE
     if(bDebugMe1)
        {
          this->Dump(lOptLabel) ;
          if(pEdgeFaceGap) { pEdgeFaceGap->Dump() ; }
          if(pEdgeCCWEdgeGap) { pEdgeCCWEdgeGap->Dump() ; }
          SmEdge * pEdge    = pEdgeCCWEdgeGap ? pEdgeCCWEdgeGap->GetEdge(0) : NULL ;
          SmEdge * pCCWEdge = pEdgeCCWEdgeGap ? pEdgeCCWEdgeGap->GetEdge(1) : NULL ;

          SmBrep * pBrep = m_pEdge ? m_pEdge->GetBrep() : NULL ; 

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,1,1) ; if(m_pEdge) m_pEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,1) ; if(pEdgeuse) pEdgeuse->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,6, 1,0,0) ; if(pEdgeCCWEdgeGap) pEdgeCCWEdgeGap->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,6, 0,1,0) ; if(pEdge) pEdge->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,6, 0,0,1) ; if(pCCWEdge) pCCWEdge->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,6, 1,0,1) ; if(pEdgeFaceGap) pEdgeFaceGap->Draw() ; sm_GraphicsLoop() ;
          this->Draw(TRUE) ; sm_GraphicsLoop() ; // sHighlightColor,         def:[ 1, 0, 0]
                                                 // sBadSmallZoneTol3dColor, def:[ 0, 1, 1]
                                                 // sBadLargeZoneTol3dColor, def:[ 1, 0, 1]
                                                 // sWarnLargeGap3dColor,    def:[ 1,.5,.1]
                                                 // sBadDegenColor,          def:[ 0, 1, 1]
                                                 // sBadUncontainedColor,    def:[ 1, 0, 1]
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end iter all Edgeuses looking for MaxGap3d values

  // locals - Edge Tols and gaps
  SmZoneTol3d sDefZoneTol3d            = SmTol::GetZoneTol3d(m_cpContext) ;
  double      dEdgeEdge_MaxGapLength3d = m_pMaxGap3d_EdgeEdge ? m_pMaxGap3d_EdgeEdge->GetLength() : 0.0 ;
  double      dEdgeFace_MaxGapLength3d = m_pMaxGap3d_EdgeFace ? m_pMaxGap3d_EdgeFace->GetLength() : 0.0 ;
  SmZoneTol3d sCalcZoneTol3d           = SmTol::CalcEdgeZoneTol3d(dEdgeEdge_MaxGapLength3d,
                                                                  dEdgeFace_MaxGapLength3d,
                                                                  NULL,   // Don't send MaxFaceZoneTol, as those Tols haven't been set yet.
                                                                  m_cpContext) ;
  // check - CalcZoneTol3d should = OrigZoneTol3d
  m_bBadSmallZoneTol3d = m_sOrigZoneTol3d <= sCalcZoneTol3d - SM_EFF_ZERO ; 
  m_bBadLargeZoneTol3d = m_sOrigZoneTol3d >= sCalcZoneTol3d + SM_EFF_ZERO ;

  // remember when geometry gap is bigger than DefZoneTol3d
  m_bWarnLargeGap3d = (   dEdgeEdge_MaxGapLength3d >= sDefZoneTol3d
                       || dEdgeFace_MaxGapLength3d >= sDefZoneTol3d) ;
  
#ifdef SM_DEBUG_CODE
 if(bDebugMe2)
    {
      this->Dump(lOptLabel) ; 
      SmBrep * pBrep = m_pEdge ? m_pEdge->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      this->Draw(TRUE) ; sm_GraphicsLoop() ; // sHighlightColor,         def:[ 1, 0, 0]
                                             // sBadSmallZoneTol3dColor, def:[ 0, 1, 1]
                                             // sBadLargeZoneTol3dColor, def:[ 1, 0, 1]
                                             // sWarnLargeGap3dColor,    def:[ 1,.5,.1]
                                             // sBadDegenColor,          def:[ 0, 1, 1]
                                             // sBadUncontainedColor,    def:[ 1, 0, 1]
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmEdgeProps::SetProps

/*******************************************************************//**
PURPOSE: set only the m_pDegenEdge property

NOTES: 
***********************************************************************/
SmStatus SmEdgeProps::SetProps_Metadata
  (SmEdge * cpEdge,     // in : tgt Edge
   ULONG    lEdgeIndx,  // in : associated Indx of cpEdge in managing SmHeadData::m_TgtEdges list
   ULONG    lOptLabel)  // NotUsed: in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
                        // sets : EdgeProps:                
                        //          m_pEdge         m_lEdgeIndx
                        //          m_sEdgeInterval         //m_pEdge->m_pEdgeProps   
                        //          m_sZoneTol3d    m_sOrigZoneTol3d        m_dApproxEdgeLength3d    
                        //          m_cpCurve       m_sNaturalInterval      
                        //          m_bBadDegenEdge m_bBadUncontainedEdge
{
  SM_REF1(lOptLabel) ;
  // locals
  SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d() ;

  // set property member values
  m_pEdge     = cpEdge ;          // forward ptr: EdgeProps->Edge
  m_lEdgeIndx = lEdgeIndx ;
  // cpEdge->SetEdgeProps(this) ; // back    ptr: Edge->EdgeProps

  // Tolerance
  m_sZoneTol3d          = SmTol::GetZoneTol3d(m_pEdge) ;
  if(m_sOrigZoneTol3d == SM_UNDEF_DOUBLE)
    { m_sOrigZoneTol3d  = m_sZoneTol3d ; }
  
  // Edge
  m_sEdgeInterval       = cpEdge->GetInterval() ; 
  m_cpCurve             = cpEdge->GetCurve() ;
  m_dApproxEdgeLength3d = m_cpCurve->ApproximateLength(m_sEdgeInterval, 5) ;
  m_sNaturalInterval    = m_cpCurve->GetNaturalInterval() ;

  // Bad Degen Edge
  m_bBadDegenEdge = m_cpCurve->IsDegenerate(sApproxTol3d, &m_sEdgeInterval) ;

  // Bad UnContained Edge
  m_bBadUncontainedEdge  = !m_sEdgeInterval.IsContainedBy(m_sNaturalInterval,0.0) ;

  // all done
  return(SM_SUCCESS) ;

 } // end SmEdgeProps::SetProps_Metadata

/****************************************************************
PURPOSE: add graphics for problem EdgeEdges

NOTES: i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
       NULL to ignore. default:[NULL]
****************************************************************/
SmDisplayList * SmEdgeProps::Draw
 (SmBoolean bDrawAllCases,            // in : TRUE = output graphics for all Edges (with or without probs)
                                      //      default:[FALSE] = output graphics only for Edges with probs 
  SmVector3d sHighlightColor,         // in : def:[ 1, 0, 0]: If EdgeHasProblems Draw EdgeUV for highlight
  SmVector3d sBadSmallZoneTol3dColor, // in : def:[ 0, 1, 1]: Edges with too small ZoneTol3d Highlight Color    
  SmVector3d sBadLargeZoneTol3dColor, // in : def:[ 1, 0, 1]: Edges with too large ZoneTol3d Highlight Color
  SmVector3d sWarnLargeGap3dColor,    // in : def:[ 1,.5,.1]: Edges with Gaps larger than DefZoneTol3d Highlight Color
  SmVector3d sBadDegenColor,          // in : def:[ 0, 1, 1]: Edges mislabled as Sheets splitting their Shell Highlight Color    
  SmVector3d sBadUncontainedColor,    // in : def:[ 1, 0, 1]: Loops Crossing Seams Highlight Color
  SmVector3d sBadCoincidentColor,     // in : def:[.5, 1,.5]: Edges coincident with other edges
  SmVector3d sBadMissedXSectColor,    // in : def:[.2,.5,.8]: Edges with EdgeXSects missing vertices
  SmGfxArraySet * pOptGfxSet          // in : When given, output GfxVertexArrays not GL calls. 
 ) const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  // locals: global display parameters
  // const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters();

  // start new displayList (unless one is already open)
  SmVector3d sFEdgePropsColor(0,1,0) ; 
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(sFEdgePropsColor, NULL, NULL, FALSE, pOptGfxSet);

  // output Problem Edge graphics

  // check need to draw
  if(   IsSet_Metadata()
     && (   bDrawAllCases
         || HasProblems()))
    {
      // highlight - when asked
      if(bDrawAllCases)                 { smgfx_SetLook(2,4, sHighlightColor) ; 
                                          m_pEdge->Draw() ; 
                                          if(m_pMaxGap3d_EdgeEdge) m_pMaxGap3d_EdgeEdge->Draw() ;
                                          if(m_pMaxGap3d_EdgeFace) m_pMaxGap3d_EdgeFace->Draw() ; 
                                        }

      // Bad ZoneTol3d - too small
      if(m_bBadSmallZoneTol3d == TRUE)  { smgfx_SetLook(4,6, sBadSmallZoneTol3dColor) ; 
                                          m_pEdge->Draw() ; 
                                          if(m_pMaxGap3d_EdgeEdge) m_pMaxGap3d_EdgeEdge->Draw() ;
                                          if(m_pMaxGap3d_EdgeFace) m_pMaxGap3d_EdgeFace->Draw() ;
                                        }
      // Bad ZoneTol3d - too large
      if(m_bBadLargeZoneTol3d == TRUE)  { smgfx_SetLook(5,7, sBadLargeZoneTol3dColor) ; 
                                          m_pEdge->Draw() ; 
                                          if(m_pMaxGap3d_EdgeEdge) m_pMaxGap3d_EdgeEdge->Draw() ;
                                          if(m_pMaxGap3d_EdgeFace) m_pMaxGap3d_EdgeFace->Draw() ; 
                                        }
                                        
      // Warn Gap3d - too large         
      if(m_bWarnLargeGap3d == TRUE)     { smgfx_SetLook(6,8, sWarnLargeGap3dColor) ; 
                                          m_pEdge->Draw() ; 
                                          if(m_pMaxGap3d_EdgeEdge) m_pMaxGap3d_EdgeEdge->Draw() ;
                                          if(m_pMaxGap3d_EdgeFace) m_pMaxGap3d_EdgeFace->Draw() ; 
                                        }
      // Bad Degen Edge Draw
      if(  m_bBadDegenEdge == TRUE)     { smgfx_SetLook(4,6, sBadDegenColor) ; 
                                          SmPoint3d sPt ;
                                          m_cpCurve->EvaluatePoint(m_sEdgeInterval.GetMin(), sPt) ;
                                          sPt.Draw() ; 
                                        }
      // Bad Uncontained Edge/Curve Draw
      if(m_bBadUncontainedEdge == TRUE) { smgfx_SetLook(4,6, sBadUncontainedColor) ;
                                          m_pEdge->Draw() ; 
                                        }

      // Bad Uncontained Edge/Curve Draw
      if(m_bBadCoincidentEdge  == TRUE) { smgfx_SetLook(4,6, sBadCoincidentColor) ;
                                          m_pEdge->Draw() ; 
                                        }

      // Bad Uncontained Edge/Curve Draw
      if(m_bBadMissedEdgeXSect == TRUE) { smgfx_SetLook(4,6, sBadMissedXSectColor) ;
                                          m_pEdge->Draw() ; 
                                        }       
    } // end need to draw check

  // end new displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF10(
      bDrawAllCases,
      sHighlightColor,
      sBadSmallZoneTol3dColor,
      sBadLargeZoneTol3dColor,
      sWarnLargeGap3dColor,
      sBadDegenColor,
      sBadUncontainedColor,
      sBadCoincidentColor,
      sBadMissedXSectColor,
      pOptGfxSet
  );
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmEdgeProps::Draw

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmEdgeProps::Dump() const
{
  // pass the call along
  Dump(SM_UNDEF_ULONG) ;

}// end SmEdgeProps::Dump()

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmEdgeProps::Dump
  (ULONG lLabel)  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmEdgeProps Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmEdgeProps Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // base class
  SmObjProps::Dump() ;

  // no work - No contents - EdgeProps_BeforeEdgeSplit() has not yet been run 
  if(IsSet_Metadata() == FALSE) 
    {
      smos_sprintf(sBuff,        _T("\n SmEdgeProps : [0x%p] - UnSet"), this) ;
      smos_sprintf(sBuffForFile, _T("\n SmEdgeProps : [%s] - UnSet"), _T("notNULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile);
      smos_WriteBuffer( _T("\nEnd SmEdgeProps Dump ")) ;

      // all done
      return ;

    } // end BeforeEdgeSplit Props not set branch

  // arrive here when SetProps() has run and IsSet_Metadata() == TRUE

  // Header = PropsPtr, EdgePtr, SurfacePtr, and XSectTol3d val.
  smos_sprintf(sBuff,        _T("\n SmEdgeProps : [0x%p] for Edge:[0x%p], EdgeIndx:[%4lu], Curve:[0x%p], ZoneTol3d:[%5.7lf], OrigZoneTol3d:[%5.7lf]"),
             this,
             m_pEdge,
             m_lEdgeIndx,
             m_cpCurve,
             m_sZoneTol3d.val, 
             m_sOrigZoneTol3d.val) ; 
  smos_sprintf(sBuffForFile, _T("\n SmEdgeProps:[%s] for Edge:[%s], EdgeIndx:[%4lu],  Curve:[%s], ZoneTol3d:[%5.7lf], OrigZoneTol3d:[%5.7lf]"),
             _T("notNULL"),
             m_pEdge    ? _T("notNULL") : _T("NULL"),
             m_lEdgeIndx,
             m_cpCurve ? _T("notNULL") : _T("NULL"),
             m_sZoneTol3d.val,
             m_sOrigZoneTol3d.val) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // Properties: EdgeIvl
  smos_sprintf(sBuff, _T("\n   Properties: Edge Interval    :Min:[%lf] Max:[%lf]"), m_sEdgeInterval.GetMin(), m_sEdgeInterval.GetMax()) ; 
  smos_WriteBuffer(sBuff) ;

  // Properties: Curve NaturalIvl
  smos_sprintf(sBuff, _T("\n             : Curve Interval   :Min:[%lf] Max:[%lf]"), m_sNaturalInterval.GetMin(), m_sNaturalInterval.GetMax()) ; 
  smos_WriteBuffer(sBuff) ;

  // IsSet_Gaps()
  if(IsSet_Gaps())
    {
      // Properties: Max Edge_Edge Gap3d
      smos_sprintf(sBuff, _T("\n             : Max Edge_Edge Gap3d:[%s%lf] %s"), 
                        m_pMaxGap3d_EdgeEdge ? _T("") : _T("none="),
                        m_pMaxGap3d_EdgeEdge ? m_pMaxGap3d_EdgeEdge->GetLength() : 0,
                        m_pMaxGap3d_EdgeEdge ? _T("") : (  m_pEdge->IsWire() ? _T("for Wire ShellEdge")
                                                     : _T(""))) ; 
      smos_WriteBuffer(sBuff) ;              

      // Properties: Max Edge_Face Gap3d
      smos_sprintf(sBuff, _T("\n             : Max Edge_Face Gap3d[%s%lf] %s"), 
                        m_pMaxGap3d_EdgeFace ? _T("") : _T("none="),
                        m_pMaxGap3d_EdgeFace ? m_pMaxGap3d_EdgeFace->GetLength() : 0,
                        m_pMaxGap3d_EdgeFace ? _T("") : (  m_pEdge->IsWire() ? _T("for Wire ShellEdge")
                                                     : _T(""))) ;
      smos_WriteBuffer(sBuff) ;

    } // end IsSet_Gaps() check

  // EdgeProblems
  if(HasProblems() == FALSE)
    { 
      smos_WriteBuffer(_T("\n    Problems : [None]")) ; 
    } // end no problems branch

  else // Has Problems branch
    {
      // locals
      SmZoneTol3d sDefZoneTol3d  = SmTol::GetZoneTol3d(m_cpContext) ;
      SmZoneTol3d sCalcZoneTol3d = SmTol::CalcEdgeZoneTol3d(m_pMaxGap3d_EdgeEdge ? m_pMaxGap3d_EdgeEdge->GetLength() : 0.0,
                                                            m_pMaxGap3d_EdgeFace ? m_pMaxGap3d_EdgeFace->GetLength() : 0.0,
                                                            NULL, // Don't use MaxFaceTol. BadTol set prior to Fix_ObjTol
                                                            m_cpContext) ;

      // IsSet_Gaps()
      if(IsSet_Gaps())
        {
          // m_bBadSmallZoneTol3d             
          smos_sprintf(sBuff, _T("\n    Problems : BadZoneTol3d Too Small :[%s]"),
                       (m_bBadSmallZoneTol3d == TRUE)  ? _T("TRUE")
                     : (m_bBadSmallZoneTol3d == FALSE) ? _T("FALSE")
                     :                                   _T("UNSURE")) ;
          smos_WriteBuffer(sBuff) ;
          if(m_bBadSmallZoneTol3d == TRUE)
            { smos_sprintf(sBuff,_T("  Bad : Increase OrigZoneTol3d:[%lf] to SmTol::CalcVertexZoneTol3d:[%lf]"),
                               m_sOrigZoneTol3d.val,
                               sCalcZoneTol3d.val) ; 
              smos_WriteBuffer(sBuff) ;
            }
          else if(m_bBadSmallZoneTol3d == FALSE && m_sOrigZoneTol3d < m_sZoneTol3d)
            {
              smos_sprintf(sBuff,_T(" Okay: OrigZoneTol3d:[%lf] increased to ZoneTol3d:[%lf]"),
                               m_sOrigZoneTol3d.val,
                               m_sZoneTol3d.val) ; 
              smos_WriteBuffer(sBuff) ;
            }
          else
            { smos_WriteBuffer(_T(" Okay")) ; } 
          // end m_bBadSmallZoneTol3d             
      
          // m_bBadLargeZoneTol3d             
          smos_sprintf(sBuff, _T("\n             : BadZoneTol3d Too Large :[%s]"),
                       (m_bBadLargeZoneTol3d == TRUE)  ? _T("TRUE")
                     : (m_bBadLargeZoneTol3d == FALSE) ? _T("FALSE")
                     :                                   _T("UNSURE")) ;
          smos_WriteBuffer(sBuff) ;
          if(m_bBadLargeZoneTol3d == TRUE)
            { smos_sprintf(sBuff,_T("  Bad : Decrease OrigZoneTol3d:[%lf] to SmTol::CalcVertexZoneTol3d:[%lf]"),
                               m_sOrigZoneTol3d.val,
                               sCalcZoneTol3d.val) ; 
              smos_WriteBuffer(sBuff) ;
            }
          else if(m_bBadSmallZoneTol3d == FALSE && m_sOrigZoneTol3d > m_sZoneTol3d)
            {
              smos_sprintf(sBuff,_T(" Okay: OrigZoneTol3d:[%lf] decreased to ZoneTol3d:[%lf]"),
                               m_sOrigZoneTol3d.val,
                               m_sZoneTol3d.val) ; 
              smos_WriteBuffer(sBuff) ;
            }
          else
            { smos_WriteBuffer(_T(" Okay")) ; } 
          // end m_bBadLargeZoneTol3d             
      
          // m_bWarnLargeGap3d
          smos_sprintf(sBuff, _T("\n             : Warn LargeGap3d        :[%s]"),
                       (m_bWarnLargeGap3d == TRUE)  ? _T("TRUE")
                     : (m_bWarnLargeGap3d == FALSE) ? _T("FALSE")
                     :                                _T("UNSURE")) ;
          smos_WriteBuffer(sBuff) ;
          if(m_bWarnLargeGap3d == TRUE)
            { smos_sprintf(sBuff,_T("  Warn: Max EdgeEnd/CCWEdgeEnd:[%lf] or Edge/Face:[%lf] Gap3d larger than DefZoneTol3d:[%lf]"),
                               m_pMaxGap3d_EdgeEdge ? m_pMaxGap3d_EdgeEdge->GetLength() : 0.0,
                               m_pMaxGap3d_EdgeFace ? m_pMaxGap3d_EdgeFace->GetLength() : 0.0,
                               sDefZoneTol3d.val) ; 
              smos_WriteBuffer(sBuff) ;
            }
          else
            { smos_WriteBuffer(_T(" Okay")) ; } 
          // end m_bWarnLargeGap3d
        } // end IsSet_Gaps() ;
      
      // IsSet_Metadata()
      if(IsSet_Metadata())
        {
          // m_bBadDegenEdge
          smos_sprintf(sBuff, _T("\n             : BadDegenEdge           :[%s]"),
                       (m_bBadDegenEdge == TRUE)  ? _T("TRUE")
                     : (m_bBadDegenEdge == FALSE) ? _T("FALSE")
                     :                              _T("UNSURE")) ;
          smos_WriteBuffer(sBuff) ;
          if(m_bBadDegenEdge == TRUE)
            { smos_WriteBuffer(_T("  Bad - Replace DegenEdge with Vertex")) ; }
          else
            { smos_WriteBuffer(_T(" Okay")) ; } 
          // end m_bBadDegenEdge
      
          // m_bBadUncontainedEdge
          smos_sprintf(sBuff, _T("\n             : BadUncontained EdgeIvl :[%s]"),
                       (m_bBadUncontainedEdge == TRUE)  ? _T("TRUE")
                     : (m_bBadUncontainedEdge == FALSE) ? _T("FALSE")
                     :                                    _T("UNSURE")) ;
          smos_WriteBuffer(sBuff) ;
          if(m_bBadUncontainedEdge == TRUE)
            { smos_WriteBuffer(_T("  Bad - Either ExtendCurve to cover EdgeIvl or TrimEdge to fit in CurveIvl")) ; }
          else
            { smos_WriteBuffer(_T(" Okay")) ; } 
          // end m_bBadUncontainedEdge
        } // end IsSet_Metadata() check

      // other 
      if(m_bBadCoincidentEdge != UNSURE)
        {
           // m_bBadUncontainedEdge
           smos_sprintf(sBuff, _T("\n             : BadCoincident  Edge    :[%s]"),
                        (m_bBadCoincidentEdge == TRUE)  ? _T("TRUE")
                      : (m_bBadCoincidentEdge == FALSE) ? _T("FALSE")
                      :                                   _T("UNSURE")) ;
           smos_WriteBuffer(sBuff) ;
           if(m_bBadCoincidentEdge == TRUE)
             { smos_WriteBuffer(_T("  Bad - Coincident Edge pairs need to be 'glued' together")) ; }
           else
             { smos_WriteBuffer(_T(" Okay")) ; } 
         } // end m_bBadCoincidentEdge
      
      // other 
      if(m_bBadMissedEdgeXSect != UNSURE)
        {
           // m_bBadMissedEdgeXSect
           smos_sprintf(sBuff, _T("\n             : BadMissedEdgeXSect Edge:[%s]"),
                        (m_bBadMissedEdgeXSect == TRUE)  ? _T("TRUE")
                      : (m_bBadMissedEdgeXSect == FALSE) ? _T("FALSE")
                      :                                   _T("UNSURE")) ;
           smos_WriteBuffer(sBuff) ;
           if(m_bBadMissedEdgeXSect == TRUE)
             { smos_WriteBuffer(_T("  Bad - Vertices need to be inserted at missed EdgeXSects")) ; }
           else
             { smos_WriteBuffer(_T(" Okay")) ; } 
         } // end m_bBadMissedEdgeXSect

    } // end Edge has Problems branch

  // all done
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmEdgeProps Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd   SmEdgeProps Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

} // end SmEdgeProps::Dump
