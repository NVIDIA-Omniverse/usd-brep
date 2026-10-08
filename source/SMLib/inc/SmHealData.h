// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmHealData.h
* PURPOSE: Header file for SmProbArray and SmHealData classes.
**********************************************************************/

#ifndef __SMHEALDATA_H__
#define __SMHEALDATA_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored these warnings to clean up errors on linux

/*******************************************************************//**
PURPOSE: Required includes

NOTES:
***********************************************************************/

#ifndef __SMHEALPROBARRAY_H__
#include <SmHealProbArray.h>
#endif // no __SMHEALPROBARRAY_H__ 

template<class TYPE> class SmObjsInVoxels ;
class SmTrackTopologyChanges ;

/*******************************************************************//**
PURPOSE: Debug convenience object_by_index tracking through the heal sequence
         and in any other methods with access to the SmFaceProps, SmEdgeProps, and SmVertexProps objs.

NOTES: To Debug One Target Face through the heal sequence: (same for Target Edge or Vertex)
         1. always Set lDebugFaceIndx = Indx of face of interest  (in file SmHealData.cpp)
         2. either Set bDebugAllFaceIndxCalls: TRUE = break for every SmDebugThisFaceIndx()  (in file SmHealData.cpp)
            or     Set bDebugThisIndxCall    : TRUE = break only SmDebugThisFaceIndx() calling DEBUG_CODE block with bDebugThisIndxCall == TRUE
         3. Set break point on line in function SmDebugThisFaceIndex
         4. Edit debug blocks of interest to start with,
                  if(bDebugMe || SmDebugThisFaceIndex(rFaceProps.m_lFaceIndx, bDebugThisIndxCall))
         5. When done remember to reset these values before releasing code:
                 lDebugFaceIndx         = SM_UNDEF_ULONG ;
                 bDebugAllFaceIndxCalls = FALSE ;
                 bDebugThisIndxCall     = FALSE ;
***********************************************************************/
#ifdef SM_DEBUG_CODE
  SmBoolean SmDebugThisFaceIndex  (ULONG lFaceIndx  , SmBoolean bDebugThisIndxCall) ;  // intended use in SM_DEBUG_CODE blocks: if(bDebugMe || SmDebugThisFaceIndex  (rFaceProps  .m_lFaceIndx)   { . . . }
  SmBoolean SmDebugThisEdgeIndex  (ULONG lEdgeIndx  , SmBoolean bDebugThisIndxCall) ;  // intended use in SM_DEBUG_CODE blocks: if(bDebugMe || SmDebugThisEdgeIndex  (rEdgeProps  .m_lEdgeIndx)   { . . . }
  SmBoolean SmDebugThisVertexIndex(ULONG lVertexIndx, SmBoolean bDebugThisIndxCall) ;  // intended use in SM_DEBUG_CODE blocks: if(bDebugMe || SmDebugThisVertexIndex(rVertexProps.m_lVertexIndx) { . . . }

#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: This class contains lists of Brep Objects with found and 
         fixed problems  

NOTES: Lists are composed by SmHealData::HealBrep()

OWNERSHIP RULES:
  SmHealData owns all heap-allocated *Props objects stored in:
    m_sTgtVertexProps, m_sTgtEdgeProps, m_sTgtFaceProps, m_sNotYetFaceProps

  1. FreeTargets() is the single cleanup function that deletes all owned
     *Props objects. Called by ~SmHealData() and at the top of ReSet().

  2. Remove*Props_FromLists() removes a *Props pointer from all SmHealData
     arrays but does NOT delete the object. It is a list-management utility.

  3. UpdateTargetLists() is the correct place to delete *Props for removed
     topology. It calls Remove*Props_FromLists() then deletes the object.

  4. Fix functions must NOT delete *Props directly. Topology removals are
     tracked by SmTrackTopologyChanges and cleaned up by UpdateTargetLists().

  5. MoveToNotYetProblems() transfers ownership of a FaceProps from
     m_sTgtFaceProps to m_sNotYetFaceProps (removes from source first).

  6. Copy construction and assignment are deleted — shallow copy of owning
     *Props pointers would cause double-free.
***********************************************************************/
class SM_EXPORT SmHealData
{
 public:
  SmBrep         * m_pBrep = NULL ;               // TgtBrep being Healed 
  SmHealerOpType   m_eStopHealOp = SM_HO_NONE ;   // largest HealerOp Step Number to be Run
  SmHealerOpType   m_eDoneHealOp = SM_HO_NONE ;   // largest HealerOp Step Number already Run
  ULONG            m_lRecursionCount  = 0 ;       // depth of recursive calls running through heal sequence for new Tgts made by Fix functions - used to improve pretty print logs

#ifdef SM_DEBUG_CODE
  ULONG            m_lPropsAllocCount = 0 ;       // debug: total *Props allocations (incremented in LoadTargets/UpdateTargetLists)
  ULONG            m_lPropsFreeCount  = 0 ;       // debug: total *Props deletions   (incremented in FreeTargets/UpdateTargetLists)
#endif // SM_DEBUG_CODE
 
 public:
  // TgtObjects and TgtProps
  SmTArray<SmVertex *>      m_sTgtVertices ;      // built on entry to SmHealData::HealBrep() - constant throughout Heal Sequence
  SmTArray<SmEdge *>        m_sTgtEdges ;         // built on entry to SmHealData::HealBrep() - constant throughout Heal Sequence
  SmTArray<SmFace *>        m_sTgtFaces ;         // built on entry to SmHealData::HealBrep() - constant throughout Heal Sequence
                                                                                               
  SmTArray<SmVertexProps *> m_sTgtVertexProps ;   // sized on entry to SmHealData::HealBrep() - vals update but size is const during Heal Sequence             
  SmTArray<SmEdgeProps *>   m_sTgtEdgeProps ;     // sized on entry to SmHealData::HealBrep() - vals update but size is const during Heal Sequence 
  SmTArray<SmFaceProps *>   m_sTgtFaceProps ;     // sized on entry to SmHealData::HealBrep() - vals update but size is const during Heal Sequence 

  // rerun heal state management
  ULONG                     m_lOldVertexCnt = 0 ; // manages state when rerunning heal sequence for newly created topology objs
  ULONG                     m_lOldEdgeCnt   = 0 ; // manages state when rerunning heal sequence for newly created topology objs
  ULONG                     m_lOldFaceCnt   = 0 ; // manages state when rerunning heal sequence for newly created topology objs
                                                  // 0 = heal steps run on all targets
                                                  // n = heal steps run only on targets ii = range[OldCnt, CntArray.GetSize()]

  // All/Any Steps - unsupported probs - FaceProps with NotYet supported problems removed from heal sequence
  SmProbArray<SmFaceProps> m_sNotYetFaceProps ;   // link: SmFaceProps::&m_bBadHasNotYetProb 
                                                  // set in any GetProps method - 
                                                  // The m_sNotYetFaceProps.m_sProbArray[ii] objs are owned here
                                                  //   The objects pointed to by those pointers have to be freed upon delete
                                                  //   current problems: 
                                                  //     1. No CrvClassifications in SetProps_Stage3()
                                                  //         - likely degenerate face 
                                                  //            (2 edges classify to same ClassifyCrv Param then are removed by
                                                  //             sms_ReviewStartAndStopOnSeamCrvClassifies() when the                       
                                                  //             interval inside the face between the Edges is found to be
                                                  //             degen length)

  // Step: SmHealData::Fix_BackPointers()
  SmProbArray<SmObjProps> m_sBadObjProps_BackPointers ;      // Link: SmObjProps::&m_bBadBackPointer      // Bad

  // Step: SmHealData::Cache_EdgeProps() - now contains SmHealData::Cache_DegenEdges()
  SmProbArray<SmEdgeProps> m_sBadEdgeProps_Uncontained ;     // link: SmEdgeProps::&m_bBadUncontainedEdge // Bad - Edge Ivl not contained in Curve Natural Ivl
  SmProbArray<SmEdgeProps> m_sBadEdgeProps_SmallZoneTol3d ;  // link: SmEdgeProps::&m_bBadSmallZoneTol3d  // Bad
  SmProbArray<SmEdgeProps> m_sBadEdgeProps_LargeZoneTol3d ;  // link: SmEdgeProps::&m_bBadLargeZoneTol3d  // Bad
  SmProbArray<SmEdgeProps> m_sWarnEdgeProps_LargeGap3d ;     // link: SmEdgeProps::&m_bWarnLargeGap3d     // Warn
  SmProbArray<SmEdgeProps> m_sBadEdgeProps_DegenEdges ;      // link: SmEdgeProps::&m_bBadDegenEdge       // Bad - DegenEdges are removed - this will end up empty  

  // All Gap Histograms use the same bucket scheme       // indx:[ 0] = No. of Gaps with GapLen:[less than or equal 1e-10] 
                                                         // indx:[ 1] = No. of Gaps with GapLen:[from     1e-10 to     1e-8] 
                                                         // indx:[ 2] = No. of Gaps with GapLen:[from     1e-8  to     1e-6] 
                                                         // indx:[ 3] = No. of Gaps with GapLen:[from     1e-6  to     1e-5] 
                                                         // indx:[ 4] = No. of Gaps with GapLen:[from     1e-5  to     1e-4] 
                                                         // indx:[ 5] = No. of Gaps with GapLen:[from     .0001 to    0.001] 
                                                         // indx:[ 6] = No. of Gaps with GapLen:[from     .001  to    0.01 ]     
                                                         // indx:[ 7] = No. of Gaps with GapLen:[from     .01   to    0.1  ]     
                                                         // indx:[ 8] = No. of Gaps with GapLen:[from     .1    to    1.0  ]     
                                                         // indx:[ 9] = No. of Gaps with GapLen:[from    1.0    to   10.0  ]       
                                                         // indx:[10] = No. of Gaps with GapLen:[greater than 10.0]              
                                                         // indx = floor(log10(EdgeLength))+9 trimmed to [0 10]
  SmTArray<ULONG>           m_sLoopGap3d_Histogram ;     // Loop EdgeuseEnd_to_Edgeuse_End Gap3d histogram of all EdgeuseEnd_to_Edgeuse_End Gaps in m_pBrep
                                                         // Bucket Index = SM_HISTOGRAM_GAPSIZE_INDEX(GapSize), (currently 11 buckets)
  SmTArray<ULONG>           m_sEdgeFaceGap3d_Histogram ; // Edge_to_Face Gap3d histogram of all Edge_to_Face Gaps in m_pBrep
                                                         // Bucket Index = SM_HISTOGRAM_GAPSIZE_INDEX(GapSize)
  SmTArray<ULONG>           m_sEdgeLen3d_Histogram ;     // EdgeLength histogram of all Edges in m_pBrep
                                                         // indx:[ 0] = No. of Edges with EdgeLen:[less than or equal 1e-5] 
                                                         // indx:[ 1] = No. of Edges with EdgeLen:[from      1e-5 to     1e-4] 
                                                         // indx:[ 2] = No. of Edges with EdgeLen:[from     .0001 to     .001] 
                                                         // indx:[ 3] = No. of Edges with EdgeLen:[from     .001  to     .01 ] 
                                                         // indx:[ 4] = No. of Edges with EdgeLen:[from     .01   to     .1  ] 
                                                         // indx:[ 5] = No. of Edges with EdgeLen:[from     .1    to    1.0  ] 
                                                         // indx:[ 6] = No. of Edges with EdgeLen:[from    1.0    to   10.0  ]     
                                                         // indx:[ 7] = No. of Edges with EdgeLen:[from   10.0    to  100.0  ]     
                                                         // indx:[ 8] = No. of Edges with EdgeLen:[from  100.0    to 1000.0  ]     
                                                         // indx:[ 9] = No. of Edges with EdgeLen:[from  1e+3     to 1e+4    ]       
                                                         // indx:[10] = No. of Edges with EdgeLen:[greater than 1e+4]              
                                                         // indx = floor(log10(EdgeLength))+6 trimmed to [0 10]
  double                    m_dMaxEdgeLength = 0.0 ;
  double                    m_dAvgEdgeLength = 0.0 ;
  double                    m_dNonDegen_MinEdgeLength = 0.0 ; 
  ULONG                     m_lNonDegenEdgeCnt = 0 ;
                            
  // Step: SmHealData::Cache_VertexProps()          
  SmProbArray<SmVertexProps> m_sBadVertexProps_SmallZoneTol3d ; // link: SmVertexProps::&m_bBadSmallZoneTol3d  // Bad
  SmProbArray<SmVertexProps> m_sBadVertexProps_LargeZoneTol3d ; // link: SmVertexProps::&m_bBadLargeZoneTol3d  // Bad
  SmProbArray<SmVertexProps> m_sWarnVertexProps_LargeGap3d ;    // link: SmVertexProps::&m_bWarnLargeGap3d     // Warn

  SmTArray<ULONG>           m_sVertEdgeGap3d_Histogram ; // Vertex_to_Edge Gap3d histogram of all Vertex_to_Edge Gaps in m_pBrep
                                                         // Bucket Index = SM_HISTOGRAM_GAPSIZE_INDEX(GapSize)

  SmTArray<ULONG>           m_sVertFaceGap3d_Histogram ; // Vertex_to_Face Gap3d histogram of all Vertex_to_Face Gaps in m_pBrep
                                                         // Bucket Index = SM_HISTOGRAM_GAPSIZE_INDEX(GapSize)
                           
  // Step: SmHealData::Cache_FaceProps_Gaps()                                
  SmProbArray<SmFaceProps> m_sBadFaceProps_SmallZoneTol3d ;  // link: SmFaceProps::&m_bBadSmallZoneTol3d  // Bad           
  SmProbArray<SmFaceProps> m_sBadFaceProps_LargeZoneTol3d ;  // link: SmFaceProps::&m_bBadLargeZoneTol3d  // Bad

  // Step: SmHealData::Fix_TolSizes()
  //  - Adjust Tol for every Vertex in m_sBadVertexProps_SmallZoneTol3d and m_sBadVertexProps_LargeZoneTol3d
  //  - Adjust Tol for every Edge   in m_sBadEdgeProps_SmallZoneTol3d   and m_sBadEdgeProps_LargeZoneTol3d
  //  - Adjust Tol for every Face   in m_sBadFaceProps_SmallZoneTol3d   and m_sBadFaceProps_LargeZoneTol3d

  // Step: SmHealData::Cache_CoinVertices()
  ULONG                             m_lXCnt = 12 ;          // number of X buckets in the m_sVertexBuckets bucket set
  ULONG                             m_lYCnt = 12 ;          // number of Y buckets in the m_sVertexBuckets bucket set
  ULONG                             m_lZCnt = 12 ;          // number of Z buckets in the m_sVertexBuckets bucket set
  SmExtent3d                        m_sVertexBBox ;        // Bounding box of all vertices
  SmObjsInVoxels<SmVertexProps *> * m_pVerticesInVoxels = NULL ;  // a voxel bucket sort of all TgtVertices - used for fast coincidence checking

  // Step: SmHealData::Fix_CoinVertices()
  SmPairProbArray<SmVertexProps> m_sBadVertexProps_CoinVertices ;   // link: SmVertexProps::&m_bBadCoincidentVertex
                                                                    // before heal: contains all coincident vertex pairs
                                                                    // after  heal: contains only merge survivors after Fix_CoinVertices()
  SmTArray<ULONG>                m_sVertVertGap3d_Histogram ;       // Vertex_to_Vertex Gap3d histogram of vertices found within
                                                                    // about 1/8 of the model size distance from one another.
                                                                    // This list only contains vertex_to_vertex gap distances
                                                                    //  of vertices near to one another.  Ex: For a simple
                                                                    //  cube with just 8 vertices, no entries will be added
                                                                    //  to this histogram since all vertex_to_vertex distances
                                                                    //  are the model's size.
                                                                    // Bucket Index = SM_HISTOGRAM_GAPSIZE_INDEX(GapSize)
                                 
  // Step: SmHealData::Cache_DegenFaces() 
  // Step: SmHealData::Fix_DegenFaces() 
  SmProbArray<SmFaceProps> m_sBadFaceProps_DegenFaces ;     // link: SmFaceProps::&m_bBadDegenFace
                                                            // Bad - DegenFaces are removed - this will end up empty  

  // Step: SmHealData::Fix_DegenEdges() 
  //  - Deletes m_sBadEdgeProps_DegenEdges edges, 
  //  - ReSets Arrays m_sBadEdgeProps_DegenEdges
  //  - Removes stale ptrs from m_sTgtEdges and m_sTgtEdgeProps

  // Step: SmHealData::Cache_CoinEdges() ** TODO **
  // Step: SmHealData::Fix_CoinEdges() ** TODO **
  SmPairProbArray<SmEdgeProps> m_sBadEdgeProps_CoinEdges ;   // link: SmEdgeProps::&m_bBadCoincidentEdge
                                                             // before heal: contains all coincident edges
                                                             // after  heal: contains only merge survivors after Fix_CoinEdges()
  // Step: SmHealData::Cache_MissedEdgeXSects() ** TODO **
  // Step: SmHealData::Fix_MissedEdgeXSects()   ** TODO **
  SmPairProbArray<SmEdgeProps> m_sBadEdgeProps_MissedEdgeXSects ; // link: SmEdgeProps::&m_BadMissedEdgeXSect
                                                                  // before heal: contains all edges with a MissedEdgeXSect
                                                                  // after  heal: contains all EdgeSplit children after Fix_MissedEdgeXSects()
  // Step: SmHealData::Fix_UncontainedEdges()                                                       
  //  - Adjusts EdgeIvls and CrvIvls for Every Edge in m_sBadEdgeProps_Uncontained()           
                                                                                               
  // Step: SmHealData::Fix_BadGaps   ** TODO **
  //  todo : SmProbArray<SmVertexProps> m_sBadVertexProps_TooBigGaps
  //  todo : SmProbArray<SmEdgeProps>   m_sBadEdgeProps_TooBigGaps
  //  todo : SmProbArray<SmFaceProps>   m_sBadFaceProps_TooBigGaps

  // Step: SmHealData::Cache_FaceProps_Stage2()
  SmMixedProbArray<SmFaceProps> m_sFaceProps_Sheets ;        // link: SmFaceProps::&m_eSheet // not always a problem - some sheets really are sheets, not mislabeled manifold faces
  SmMixedProbArray<SmFaceProps> m_sFaceProps_NoVertexPole ;  // link: SmFaceProps::&m_ePoles // not always a problem - some surface poles are not in some Face bndrys
  SmProbArray<SmFaceProps>      m_sBadFaceProps_FlatCorner ; // link: SmFaceProps::&m_lBadFlatCorners // Bad
                                                                                    
  SmProbArray<SmFaceProps> m_sBadFaceProps_MissingSeam ;     // link: SmFaceProps::&m_bBadMissingSeam  // list of Faces with Missing Seams 
  SmProbArray<SmFaceProps> m_sBadFaceProps_CrossedSeam ;     // link: SmFaceProps::&m_bBadCrossedSeam  // list of Faces with LoopEdges crossing a seam 
  SmProbArray<SmFaceProps> m_sBadFaceProps_NearMissSeam ;    // link: SmFaceProps::&m_bBadNearMissSeam // list of Faces with LoopEdges nearly coin with a seam 
                                                                                               
  // Step: SmHealData::Fix_MoveSeam()
  //  - for every Face in m_sBadFaceProps_MissingSeam,
  //                      m_sBadFaceProps_CrossedSeam, and 
  //                      m_sBadFaceProps_NearMissSeam ;
  //     + try to move seam out of face or far from vertices
  //     + when seam was moved - rerun SetProps_Stage2()
  //     + when Face SeamProbs are fixed - remove FaceProp from associated ProbArray
  SmTriedProbArray<SmFaceProps> m_sRanFaceProps_MoveSeam ;     // link: SmFaceProps::&m_eBeenThroughMoveSeam
                                                               //       SM_TRY_NONE, SM_TRY_RAN, SM_TRY_FIXED, SM_TRY_NOFIX
                                                               // Faces sent through Fix_MoveSeam

  // Step: SmHealData::Fix_SplitEdgesAtSeam()
  //  - for every Face remaining in m_sBadFaceProps_CrossedSeam 
  SmTriedProbArray<SmFaceProps> m_sRanFaceProps_SplitEdgesAtSeam ;   // link: SmFaceProps::&m_eBeenThroughSplitEdgesAtSeam
                                                                     //       SM_TRY_NONE, SM_TRY_RAN, SM_TRY_FIXED, SM_TRY_NOFIX
                                                                     // ends: m_lOrigProbCount = total number of faces sent to Fix_SplitFaceAtSeams
                                                                     //       FaceProps sent through Fix_SplitEdgesAtSeam
                                                                     //         that did not get split.
  // Step: SmHealData::Fix_BadSheets()                                                              
  //  - Finds and Fixes Every Bad Face orig labeled as sheets that were really manifold in m_sFaceProps_Sheets                              
  ULONG m_lRanRegion_TestClosure_Cnt = 0 ; // number of times SmBrep::TestClosureSplitRegion() was run
  ULONG m_lFixRegion_SplitRegion_Cnt = 0 ; // number of SplitRegion fixes = number of new shells
  
  // Step: SmHealData::Cache_FaceProps_Stage3()
  // Step: SmHealData::Fix_BadLoops()
  SmProbArray<SmFaceProps> m_sBadFaceProps_LoopProblems ;        // link: SmFaceProps::&m_bBadLoopProblems    // Bad Faces = Union of all Loop problem lists - Never Cleared of Fixes 

  SmProbArray<SmFaceProps> m_sBadFaceProps_OuterLoopOrder ;      // link: SmFaceProps::&m_bBadOuterLoopOrder  // Bad - fix by reordering FaceLoop list  - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_OrientLoop ;          // link: SmFaceProps::&m_lBadOrient_LoopCnt  // Bad - fix by flipping loop orientation - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_NoArea_Loop ;         // link: SmFaceProps::&m_lBadNoArea_LoopCnt  // Bad - fix by SplitAtSeam               - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_MultiOuterLoops ;     // link: SmFaceProps::&m_lOuterLoopCnt > 1   // Bad - fix by splitting face            - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_NestedOuterLoops ;    // link: SmFaceProps::&m_lBadNested_LoopCnt  // Bad - fix by splitting face            - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_UnpairedNoAreaLoop ;  // link: SmFaceProps::&m_bBadUnpairedNoArea_Loop // Bad - No fix ideas (yet?)              - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_Closed3dLoop ;        // link: SmFaceProps::&m_lBadClosed3d_LoopCnt    // Bad - NotYet - Edit EdgeGeom to Rm EdgeEnd/EdgeEnd Gaps - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_ClosedPtrLoop ;       // link: SmFaceProps::&m_lBadClosedPtr_LoopCnt   // Bad - NotYet - Edit EdgeusePtrs to close CCW LinkedList - Cleared of Fixes
  SmProbArray<SmFaceProps> m_sBadFaceProps_MissingPole ;         // link: SmFaceProps::&m_lBadMissingPoles        // Bad - fix by Adding Vertex at Pole     - Cleared of Fixes

  // Step: SmHealData::Fix_BadLoops()
  //  - ForEvery m_sBadFaceProps_MissingPole               FaceProps - MakeVertexLoop()
  //  + Add NewVertex and NewLoop objs to m_sNewVerticesOnPoles_FromFixBadLoops and m_sNewLoopsOnPoles_FromFixBadLoops
  //  - ForEvery m_sBadFaceProps_OrientLoop                FaceProps - FlipLoopOrientation() on MisOriented Loops
  //  - ForEvery m_sBadFaceProps_OuterLoopOrder            FaceProps - Make 1st Outer (or BotLoop) Loop 1st Loop on FaceLoop List
  //  - ForEvery m_sBadFaceProps_MultiOuterLoops            FaceProps - MakeFaceFromLoopOnFace()
  //  - ForEvery m_sBadFaceProps_NestedOuterLoops          FaceProps - MakeFaceFromLoopOnFace()
  //  - for every changed Face - Rerun SmHealData::Cache_FaceProps_Stage2() and SmHealData::Cache_FaceProps_Stage3()
  //     + Remove fixed entries from: m_sBadFaceProps_SeamProbs
  //                                  m_sBadFaceProps_MultiOuterLoops
  //                                  m_sBadFaceProps_NestedOuterLoops
  //                                  m_sBadFaceProps_OuterLoopOrder
  //                                  m_sBadFaceProps_OrientLoop
  //                                  m_sBadFaceProps_MissingPole
  //  - for every new Face - add Problem FaceProps to
  //    . . .  (TODO)
  

  // Step: SmHealData::Fix_SplitFaceAtSeams()
  //  - ForEvery m_sBadFaceProps_SeamProbs  FaceProps - SmFace::SplitAtSeams()
  // (TODO ?)
  //  - forEvery Changed Face ReRun SmHealData::Cache_FaceProps_Stage3()
  //     + Removed fixed FaceProps from Problem Lists
  SmTriedProbArray<SmFaceProps> m_sRanFaceProps_SplitFaceAtSeam ;  // link: SmFaceProps::&m_eBeenThroughSplitFacesAtSeam
                                                                   //       SM_TRY_NONE, SM_TRY_RAN, SM_TRY_FIXED, SM_TRY_NOFIX
                                                                   // Faces whose SeamProbs were fixed by Fix_SplitFaceAtSeams

  // Step: SmHealData::Make_UVTrimCurves()
  //   - ForEvery Face - SmFace::CreateUVTrimCurves()

  // Step: SmHealData::Fix_InfiniteRegion()
  //   - ForBrep - FindAndSetInfiniteRegion()
  //             - SetIsVoid()

  // New objects made by Fix functions
  SmTArray<SmFace   *> m_sNewFaces_FromFixDegenFaces ;           // from: SM_HO_FIX_DEGEN_FACES
  SmTArray<SmEdge   *> m_sNewEdges_FromSplitEdgesAtSeams ;       // from: SM_HO_FIX_SPLITEDGE_ATSEAM  - SmHealData::Fix_SplitEdgesAtSeam()
  SmTArray<SmVertex *> m_sNewVertices_FromSplitEdgesAtSeams ;    // from: SM_HO_FIX_SPLITEDGE_ATSEAM  - SmHealData::Fix_SplitEdgesAtSeam()
  SmTArray<SmVertex *> m_sNewVerticesOnPoles_FromFixBadLoops ;   // from: SM_HO_FIX_BADLOOPS          - SmHealData::Fix_BadLoops()         
  SmTArray<SmLoop   *> m_sNewLoopsOnPoles_FromFixBadLoops ;      // from: SM_HO_FIX_BADLOOPS          - SmHealData::Fix_BadLoops()         
  SmTArray<SmFace   *> m_sNewFaces_FromExtraLoops ;              // from: SM_HO_FIX_BADLOOPS          - SmHealData::Fix_BadLoops()         
  SmTArray<SmFace   *> m_sNewFaces_FromSplitFaceAtSeams ;        // from: SM_HO_FIX_SPLITFACE_ATSEAMS - SmHealData::Fix_SplitFaceAtSeams()  
  SmTArray<SmEdge   *> m_sNewEdgesOnSeams_FromSplitFaceAtSeams ; // from: SM_HO_FIX_SPLITFACE_ATSEAMS - SmHealData::Fix_SplitFaceAtSeams() 
  SmTArray<SmVertex *> m_sNewVertices_FromSplitFaceAtSeams ;     // from: SM_HO_FIX_SPLITFACE_ATSEAMS - SmHealData::Fix_SplitFaceAtSeams()  
  SmTArray<SmRegion *> m_sNewRegions_FromClosedSheets ;          // from: SM_HO_FIX_BADSHEETS         - SmHealData::Fix_BadSheets()         
  SmTArray<SmShell  *> m_sNewShells_FromClosedSheets ;           // from: SM_HO_FIX_BADSHEETS         - SmHealData::Fix_BadSheet()

 public:

  // empty constructor
  SmHealData() ; 

  // constructor
  SmHealData(SmBrep               * pBrep,       // in : target Brep                                                                                                  
             SmTArray<SmVertex*>  & rVertices,   // in : TgtVertex list copied into SmHealData arrays, (any deleted vertices will be removed from the copied array)   
             SmTArray<SmEdge*>    & rEdges,      // in : EdgeProps list copied into SmHealData arrays, (any deleted edges will be removed from the copied array)      
             SmTArray<SmFace*>    & rFaces) ;    // in : FaceProps list copied into SmHealData arrays, (any deleted face will be removed from the copied array)       

  // copy constructor and assignment operator deleted — shallow copy of owning
  // *Props pointers would cause double-free when both instances are destroyed.
  SmHealData(const SmHealData &crOther) = delete ;
  SmHealData & operator=(const SmHealData &crOther) = delete ;

  // equality operator
  SmBoolean operator==(const SmHealData &crOther) ;

  // destructor - free Tgt ObjProps array objects and nested allocated objects
  ~SmHealData() ;

  // Load Targets - alloc ObjProps List memory
  void LoadTargets(SmBrep              * pBrep,     // in : TgtBrep contains all Tgt Vertices, Edges, Faces 
                   SmTArray<SmVertex*> & rVertices, // in : TgtVertex list copied into SmHealData arrays    
                   SmTArray<SmEdge*>   & rEdges,    // in : EdgeProps list copied into SmHealData arrays    
                   SmTArray<SmFace*>   & rFaces) ;  // in : FaceProps list copied into SmHealData arrays    

  // Free Targets - Free ObjProps List memory:[m_sTgtVertexProps, m_sTgtEdgeProps, m_sTgtFaceProps, m_sNotYetFaceProps]
  void FreeTargets() ;   

  // ReSet - init all member values except for m_pBrep - init the ProbArrays labels and links
  void ReSet(SmBoolean bReSetNewObjArrays = TRUE) ; // TRUE = Reset the m_sNewObj array lists, FALSE = don't

  // ReSet helper function - Setup all the prob arrays - set link pointers and set PrettyPrint labels
  //   When you add a new ProbArray to the healer 1. Select as desired from the SmProbArray<TYPE> class hierarchy
  //                                                 and declare new ProbArray in SmHealData
  //                                              2. Add a Boolean or an Enum to the associated ObjsProps class
  //                                                 to be linked to the new array
  //                                              3. Add the new array to: 
  //                                                      a. SmHealData::SetupProbArray() // est link to ObjsProps member
  //                                                      b. SmHealData::ReSet() to clear the ProbArray list members
  //                                                      c. SmHealData::Dump() to prettyprint
  void SetupEmptyProbArrays() ;

  // object index access
  ULONG GetFaceProps_ForTgts  (SmTArray<SmTopology *> * pTgts, SmTArray<SmFaceProps *>   * pFaceProps,   SmBoolean bStaleTgts=TRUE) ; 
  ULONG GetEdgeProps_ForTgts  (SmTArray<SmTopology *> * pTgts, SmTArray<SmEdgeProps *>   * pEdgeProps,   SmBoolean bStaleTgts=TRUE) ; 
  ULONG GetVertexProps_ForTgts(SmTArray<SmTopology *> * pTgts, SmTArray<SmVertexProps *> * pVertexProps, SmBoolean bStaleTgts=TRUE) ; 
  SmBoolean FindFaceIndx  (SmFace   * pFace,   ULONG &lIndx) { return m_sTgtFaces.   FindElement(pFace,   lIndx) ; }
  SmBoolean FindEdgeIndx  (SmEdge   * pEdge,   ULONG &lIndx) { return m_sTgtEdges.   FindElement(pEdge,   lIndx) ; }
  SmBoolean FindVertexIndx(SmVertex * pVertex, ULONG &lIndx) { return m_sTgtVertices.FindElement(pVertex, lIndx) ; }

  // obsolete
  // // ReSet all member vals then Rebuild FaceProps with appropriate SetProps(SM_PROPSTAGE_GAPS, SM_PROPSTAGE_2, SM_PROPSTAGE_3) 
  // void RefreshFaceProps
  // (
  //   SmFaceProps  * pTgtFaceProps,                 // in : The Tgt SmFaceProps to clear and refresh  
  //   ULONG          lFaceIndx,                     // in : associated Indx in SmHealData::m_sTgtFaces list for pTgtFaceProps->m_pFace  
  //   SmHealerOpType eThruHealerOp,                 // in : Refresh TgtFaceProps up thru this HealerStep operation limited to SmHealData::m_eDoneHealOp 
  //   ULONG          lOptLabel = SM_UNDEF_ULONG     // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]  
  // ) ;
  // end obsolete

  // Update SmHealData Tgt and Prob arrays removing refs to pStartChangeTracking->RmList items and adding pStartChangeTracking->AddList items
  SmStatus UpdateTargetLists(SmTrackTopologyChanges & rTopologyChanges, // in : container for RmList, RmTypes, and AddList
                             SmHealerOpType           eCurrentHealOp) ; // in : Tgt HealOp to execute upon new ObjProps associated with AddList items

  SmStatus UpdateTargetLists(SmTArray<SmTopology *> * pRmList,           // in : List of Topo objs removed from m_pBrep TopoGraph to remove from HealData Arrays
                             SmTArray<SmTopology *> * pAddList,          // in : List of Topo objs added to m_pBrep TopoGraph to add to HealData Arrays
                             SmHealerOpType           eCurrentHealOp) ;  // in : Tgt HealOp to execute upon new ObjProps associated with AddList items
       
  // Remove Vertex and VertexProps from all the SmHealData lists, does not delete TgtVertex or TgtVertexProps
  void RemoveVertexProps_FromLists(SmVertexProps * pTgtVertexProps,                 // in : the Tgt VertexProps (contains Tgt Vertex) to remove  
                                   SmBoolean     * pOptRmFromTgtVertices=NULL,      // out: TRUE = Tgt Vertex was on TgtVertices lists, FALSE = wasn't 
                                   SmBoolean     * pOptRmFromTgtVertexProps=NULL) ; // out: TRUE = Tgt VertexProps was on TgtVertexProps lists, FALSE = wasn't 

  // Remove Edge and EdgeProps from all the SmHealData lists, does not delete TgtEdge or TgtEdgeProps
  void RemoveEdgeProps_FromLists(SmEdgeProps * pTgtEdgeProps,                       // in : Tgt EdgeProps (contains Tgt Edge) to remove  
                                 SmBoolean   * pOptRmFromTgtEdges=NULL,             // out: TRUE = Tgt Edge was on TgtEdges lists, FALSE = wasn't 
                                 SmBoolean   * pOptRmFromTgtEdgeProps=NULL) ;       // out: TRUE = Tgt EdgeProps was on TgtEdgeProps lists, FALSE = wasn't 

  // Remove Face and FaceProps from all the SmHealData lists, does not delete TgtFace or TgtFaceProps
  void RemoveFaceProps_FromLists(SmFaceProps * pTgtFaceProps,                       // in : Tgt FaceProps (contains Tgt Face) to remove  
                                 SmBoolean   * pOptRmFromTgtFaces=NULL,             // out: TRUE = Tgt Face was on TgtFaces lists, FALSE = wasn't 
                                 SmBoolean   * pOptRmFromTgtFaceProps=NULL) ;       // out: TRUE = Tgt FaceProps was on TgtFaceProps lists, FALSE = wasn't 

  // Move TgtFaceProp to m_sNotYetFaceProps list - removing it from all other lists
  void MoveToNotYetProblems(SmFaceProps * pFaceProps) ;

  // Healer interface: Load all Brep Topology Objects into this TgtArrays and call HealTgts()
  SmStatus HealBrep(SmBrep              * pBrep,                     // in : Target Brep to heal                                   
                    SmHealerOpType        eHealerOp,                 // in : Run Healer Sequence up to and including this operation
                    SmTArray<SmVertex*> * pOptVertices=NULL,         // in : optional TgtVertex list, NULL=use all Brep Vertices, (any vertices deleted by healing will be removed from this list)
                                                                     //      default:[NULL]                                                                                                       
                    SmTArray<SmEdge*>   * pOptEdges=NULL,            // in : optional EdgeProps list, NULL=use all Brep Edges,    (any edges    deleted by healing will be removed from this list)
                                                                     //      default:[NULL]                                                                                                       
                    SmTArray<SmFace*>   * pOptFaces=NULL,            // in : optional FaceProps list, NULL=use all Brep Faces,    (any faces    deleted by healing will be removed from this list)
                                                                     //      default:[NULL]                                                                                                       
                    SmBoolean             bMakeUVTrimCurves=FALSE) ; // in : TRUE=call SmFace::CreateUVTrimCurves for each face, default:[FALSE]=Don't                               

  // The Healer - execute healing on all the objects stored in the SmHealData::TgtArrays
  SmStatus HealTgts(SmHealerOpType eHealerOp,            // in : SM_HO_NONE = 0,                           //  No Healer Steps run                                           
                                                         //                                                                                                                  
                                                         //      SM_HO_FIX_BACKPOINTERS,                   // Ran All Healer Steps thru SmHealData::Fix_BackPointers         
                                                         //                                                                                                                  
                                                         //      SM_HO_CACHE_EDGEPROPS,                    // Ran All Healer Steps thru SmHealData::Cache_EdgeProps()        
                                                         //      SM_HO_CACHE_VERTEXPROPS,                  // Ran All Healer Steps thru SmHealData::Cache_VertexProps()         
                                                         //      SM_HO_CACHE_FACEPROPS_GAPS,               // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Gaps()   
                                                         //                                                                                                                  
                                                         //      SM_HO_FIX_TOLSIZES,                       // Ran All Healer Steps thru SmHealData::Fix_TolSizes()           
                                                         //      SM_HO_CACHE_COIN_VERTICES,                // Ran All Healer Steps thru SmHealData::Cache_CoinVertices()     
                                                         //      SM_HO_FIX_COIN_VERTICES,                  // Ran All Healer Steps thru SmHealData::Fix_CoinVertices()       
                                                         //                                                                                                                  
                                                         //      SM_HO_CACHE_DEGEN_FACES,                  // Ran All Healer Steps thru SmHealData::Cache_DegenFaces()       
                                                         //      SM_HO_FIX_DEGEN_FACES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenFaces()         
                                                         //                                                                                                                  
                                                         //      SM_HO_FIX_DEGEN_EDGES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenEdges()         
                                                         //                                                                                                                  
                                                         //      SM_HO_CACHE_COIN_EDGES,        /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_CoinEdges()        
                                                         //      SM_HO_FIX_COIN_EDGES,          /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_CoinEdges()          
                                                         //                                                                                                                  
                                                         //      SM_HO_CACHE_MISSED_EDGEXSECTS, /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_MissedEdgeXSects() 
                                                         //      SM_HO_FIX_MISSED_EDGEXSECTS,   /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_MissedEdgeXSects()   
                                                         //                                                                                                                  
                                                         //      SM_HO_FIX_UNCONTAINED_EDGES,   /* STUB */ // Ran All Healer Steps thru SmHealData::Fix_UncontainedEdges()   
                                                         //      SM_HO_FIX_BADGAPS,             /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_BadGaps()            
                                                         //                                                                                                                  
                                                         //      SM_HO_CACHE_FACEPROPS_2,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage2() 
                                                         //                                                                                                                  
                                                         //      SM_HO_FIX_MOVESEAM,                       // Ran All Healer Steps thru SmHealData::Fix_MoveSeam()           
                                                         //      SM_HO_FIX_SPLITEDGE_ATSEAM,               // Ran All Healer Steps thru SmHealData::Fix_SplitEdgesAtSeam()   
                                                         //      SM_HO_FIX_BADSHEETS,                      // Ran All Healer Steps thru SmHealData::Fix_BadSheets()          
                                                         //                                                                                                                  
                                                         //      SM_HO_CACHE_FACEPROPS_3,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage3() 
                                                         //                                                                                                                  
                                                         //      SM_HO_FIX_SPLITFACE_ATSEAMS,              // Ran All Healer Steps thru SmHealData::Fix_SplitFaceAtSeams()   
                                                         //      SM_HO_FIX_BADLOOPS,                       // Ran All Healer Steps thru SmHealData::Fix_BadLoops()           
                                                         //      SM_HO_MAKE_UVTRIMCURVES,                  // Ran All Healer Steps thru SmHealData::Make_UVTrimCurves()      
                                                         //      SM_HO_FIX_INFINITE_REGIONS,               // Ran All Healer Steps thru SmHealData::Fix_InfiniteRegion()     
                                                         //                                                                                                                  
                                                         //      SM_HO_ALL                                 // Ran All Healer Steps                                           
                    SmBoolean bMakeUVTrimCurves=FALSE) ; // in : TRUE=call SmFace::CreateUVTrimCurves for each face, default:[FALSE]=Don't                                    

  // The heal Steps - declared in the order in which they run within HealBrep()
  // where:  pBrep          = in : Tgt Brep being healed
  //         eThisHealerOp  = in : This function's Step Number within the healer step sequence
  SmStatus Fix_BackPointers          (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in : currently a stub function
                                                                                                                             
  SmStatus Cache_EdgeProps           (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
  // now a part of Cache_EdgeProps:                                                                                          
  // SmStatus Cache_DegenEdges       (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Cache_VertexProps         (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
  SmStatus Cache_FaceProps_Gaps      (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
                                                                                                                             
  // fix tols before looking for coincidences                                                                                
  SmStatus Fix_TolSizes              (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Cache_CoinVertices        (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
  SmStatus Fix_CoinVertices(SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Cache_DegenFaces          (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
  SmStatus Fix_DegenFaces            (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Fix_DegenEdges            (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Cache_CoinEdges/* TODO */ (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
  SmStatus Fix_CoinEdges  /* TODO */ (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Cache_MissedEdgeXSects /* TODO */ (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;        // eThisHealerOp NotUsed: in :
  SmStatus Fix_MissedEdgeXSects   /* TODO */ (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                
                                                                                                                             
  SmStatus Fix_UncontainedEdges /* needs better fix function*/ (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;              // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Fix_BadGaps    /* TODO */ (SmBrep * pBrep, SmHealerOpType eThisHealerOp) ;                                        // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Cache_FaceProps_Stage2    (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                // eThisHealerOp NotUsed: in :
                                                                                                                             
  SmStatus Fix_MoveSeam              (SmBrep * pBrep, SmNewMarkAndLock & rMarkLock, SmHealerOpType eThisHealerOp) ;          // eThisHealerOp NotUsed: in :
  SmStatus Fix_SplitEdgesAtSeam      (SmBrep * pBrep, SmNewMarkAndLock & rMarkLock, SmHealerOpType eThisHealerOp) ;          // eThisHealerOp NotUsed: in :
  SmStatus Fix_BadSheets             (SmBrep * pBrep, SmNewMarkAndLock & rMarkLock, SmHealerOpType eThisHealerOp) ;          // eThisHealerOp NotUsed: in :
                                                                                                                             // eThisHealerOp NotUsed: in :
  SmStatus Cache_FaceProps_Stage3    (SmBrep * pBrep, SmBoolean * pbNewProbs, SmHealerOpType eThisHealerOp) ;                
                                                                                                                             
  SmStatus Fix_SplitFaceAtSeams      (SmBrep * pBrep, SmHealerOpType     eThisHealerOp) ;                                    // eThisHealerOp NotUsed: in :
  SmStatus Fix_BadLoops              (SmBrep * pBrep, SmNewMarkAndLock & rMarkLock, SmHealerOpType     eThisHealerOp) ;      // eThisHealerOp NotUsed: in :
  SmStatus Make_UVTrimCurves         (SmBrep * pBrep, SmBoolean          bMakeUVTrimCurves, SmHealerOpType eThisHealerOp) ;  // eThisHealerOp NotUsed: in :
  SmStatus Fix_InfiniteRegion        (SmBrep * pBrep, SmHealerOpType     eThisHealerOp) ;                                    // eThisHealerOp NotUsed: in :

  // pretty print
  void Dump                     (ULONG lLabel=SM_UNDEF_ULONG) const ;
  void DumpVertexListsContaining(SmVertexProps * pTgtVertexProps, ULONG lLabel=SM_UNDEF_ULONG) const ; 
  void DumpEdgeListsContaining  (SmEdgeProps   * pTgtEdgeProps,   ULONG lLabel=SM_UNDEF_ULONG) const ; 
  void DumpFaceListsContaining  (SmFaceProps   * pTgtFaceProps,   ULONG lLabel=SM_UNDEF_ULONG) const ; 
  void DumpArraysContaining     (ULONG lLabel, SmTArray<SmFaceProps *>   * pTgtFaceProps,
                                               SmTArray<SmEdgeProps *>   * pTgtEdgeProps,
                                               SmTArray<SmVertexProps *> * pTgtVertexProps,
                                               SmBoolean                   bRptNoTgtArrays=FALSE) const ; 
} ; // end container class SmHealData

// add a SmTArray<SmLoopProps> template to the dll interface
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmHealData) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmHealData*) ;

#endif // !__SMHEALDATA_H__
