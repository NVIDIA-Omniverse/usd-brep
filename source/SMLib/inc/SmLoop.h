// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmLoop.h
* PURPOSE: Header file for SmLoop class.
**********************************************************************/

#ifndef __SMLOOP_H__
#define __SMLOOP_H__

#ifndef __SMTOPOLOGY_H__
#include <SmTopology.h>
#endif

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMLOOPUSE_H__
#include <SmLoopuse.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

/*******************************************************************//**
PURPOSE: This class represents topological Loops.

NOTES: 
***********************************************************************/
class SM_EXPORT SmLoop : public SmTopology
{
    friend class SmBrep;
    friend class SmLoopuse;
    friend class SmFace;
    friend class SmBrepConstructor;
protected:    
    // inherited:
    // SmTopology::m_pListOwner  - not used
    // SmTopology::m_pNext       - not used
    // SmTopology::m_pLast        

    SmLoopuse *m_pLU;     // primary Loopuse of Loop (connected to Face's primary Faceuse) and
                          // note: secondary Loopuse (mpLU->GetMate() is connected to
                          //       Face's secondary Faceuse.)

protected:
    SmLoop() : m_pLU(NULL) { // report construction at SmObject::Notify level - skip other levels
                             SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ;
                           }
public:
    virtual ~SmLoop()      { Notify(SM_NO_DESTRUCTION, this, NULL, NULL) ; 
                             m_pLU = NULL ; 
                           }

    SmStatus FlipLoopOrientation();

  SmBrep      * GetBrep      ()                                     const { return(m_pLU ? m_pLU->GetBrep() : NULL) ; }
  SmFace      * GetFace      ()                                     const { SM_ASSERT(m_pLU != NULL && m_pLU->GetFaceuse() != NULL) ;
                                                                            SM_ASSERT(m_pLU->GetFaceuse()->GetFace() != NULL) ;
                                                                            return(m_pLU->GetFaceuse()->GetFace()) ;
                                                                          }
  SmLoopuse   * GetLoopuse   ()                                     const { return(m_pLU) ; }
  void          GetVertices  (SmTArray<SmVertex*>    & rVertices)   const { m_pLU->GetVertices( rVertices ) ; }
  void          GetVertexuses(SmTArray<SmVertexuse*> & rVertexuses) const { m_pLU->GetVertexuses( rVertexuses ) ; }
  void          GetEdges     (SmTArray<SmEdge*>      & rEdges)      const { m_pLU->GetEdges( rEdges ) ; }
  void          GetEdgeuses  (SmTArray<SmEdgeuse*>   & rEdgeuses)   const { m_pLU->GetEdgeuses( rEdgeuses ) ; }
  void          GetLoopuses  (SmLoopuse *& rpLoopuseOfLoop1,              // out: m_pLU
                              SmLoopuse *& rpLoopuseOfLoop2)        const // out: m_pLU->GetOtherLoopuse();
                                                                          { SM_ASSERT(m_pLU != NULL) ; 
                                                                            rpLoopuseOfLoop1 = m_pLU ;
                                                                            rpLoopuseOfLoop2 = m_pLU->GetOtherLoopuse() ; 
                                                                            SM_ASSERT(rpLoopuseOfLoop2 != NULL) ;
                                                                          }
  SmAObject   * GetAOwner     ()                                    const { return ((SmAObject*)GetFace()->GetBrep()); } // get attribute inheritance owner

  // :--------------------------------------------------------------------------------------------:
  // :                          UpDim, DownDim, and SameDim Gaps                                  :
  // :----------+--------------------------+---------------------------+--------------------------+            
  // : Topology : UpDim Gap types          : DownDim Gap types         : SameDim Gap types        :      
  // :----------+--------------------------+---------------------------+--------------------------+       
  // :  Vertex  : Vertex/Edge, Vertex/Face : none                      : none                     :       
  // :  Edge    : Edge/Face                : Vertex/Edge               : none                     :       
  // :  Face    : none                     : Vertex/Face, Edge/Face    : none                     :       
  // :  Loop    : none                     : none                      : EdgeEnd/EdgeEnd LoopGaps :       
  // :----------+--------------------------+---------------------------+--------------------------+   
  //        MaxGap = Max( LoopGap }
  virtual const SmGap * GetMaxGap3d       (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const { return(GetMaxSameDimGap3d(pOptGapArray, pOptTol3d) ) ; } 
  virtual const SmGap * GetMaxSameDimGap3d(SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const ;

  SmBoolean         IsOuterLoop  () const ;                                 // rtn: TRUE when this loop is the first loop->Faceuse->LoopList
  SmBoolean         IsLamina     () const ;                                 // rtn: TRUE when all edges in loop are lamina edges
  SmBoolean         IsManifold   () const ;                                 // rtn: TRUE when all edges in loop are manifold edges
  SmBoolean         IsClosed3d   (                                                // rtn: TRUE when all EdgeEnd/EdgeEnd Gap3ds are within their associated XSectTol3d limit values
                                  SmBoolean       bCheckAllGaps = TRUE,           // in : TRUE = Check all gaps after finding the first open EdgeuseEnd-EdgeuseEnd gap to get true MaxGap values
                                                                 //      FALSE= quit after finding the 1st open EdgeuseEnd-EdgeuseEnd gap to save time.
                         SmEdgeuse    ** ppOptEdgeuse  = NULL,   // out: When any gaps are out-of-tol, the 1st edgeuse of the gap that exceeds its XSectTol3d value the most,
                                                                 //      When all gaps are in-tol, the largest gap between Edgeuse ends.
                                                                 //      CCW Edgeuse is the other end of gap. NULL to ignore, Default:[NULL]
                         double        * pdOptMaxGap   = NULL,   // out: Dist for gap being returned by ppOptEdgeuse
                                                                 //      NULL to ignore, Default:[NULL]
                                  SmXSectTol3d  * psOptXSectTol3d = NULL) const ; // out: XSectTol3d value for gap being returned by ppOptEdgeuse. 
  SmBoolean         IsClosedPtrs ()                                       const ; // rtn: TRUE when Edgeuse LinkedList of CCWEdgeuse ptrs form a closed LinkedList
    virtual SmBoolean IsConnectedTo(const SmTopology *cpConnectTgt) const ;
  SmBoolean         IsEmbedded   (                                                // rtn: TRUE when all edges are embedded in owner face
                                  SmBoolean bFaceHasSeam=TRUE)            const ; // in : TRUE = Face known to have SeamEdge - do Expensive IsSeam() check
                                                                          //      FALSE= Face known to have no SeamEdge - skip IsSeam() check

  //void SetLoopProps(SmLoopProps * pLoopProps) const { m_pLoopProps = pLoopProps ; }

    SmPointObjectContainmentType ContainsUVPoint                                     
    (const SmPoint2d   & crUVPoint,       // in : Point to classify
     double              dTol3d,          // NotUsed: in : 3-space tolerance.
     SmEdge          * & rpOnEdge,        // out: only set when rtn value is SM_POC_ON_BOUNDARY
     double            & rdEdgeParam,     // out: only set when rtn value is SM_POC_ON_BOUNDARY
     SmOrientType    & eOrientation) ;    // out: only set when rtn value is SM_POC_INSIDE

    // get Loop UVBoundary corners touching given UVDomain square - useful when loop is bounded by a surface singularity
    // rtn: SM_SS_NONE or orof:[SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX]
    ULONG GetUVCornerBoundaries(const SmExtent2d   & crUVDomain,   // in : UVDomain to test double
                                double               dTol,         // in : tolerance in uv space
                                SmTArray<SmEdge *> & rOnEdges,     // out: list of edges with endPoints on a crUVDomain boundary
                                SmTArray<ULONG>    & rOnBndries,   // out: associated list of boundaries for rOnEdges list
                                SmTArray<double>   & rOnParams) ;  // out: associated edge param for rOnEdges list

    SmStatus RebuildFromEdgeCurves() ;

    virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //    : default:[SM_LEVEL_0] 
                                  SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                  SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                 const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
    SmDisplayList * Draw(ULONG           bVUandEUDraw=3,           // in : oneof: 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EUs
                         SmBoolean       bDrawUVPlane=FALSE,       // in : TRUE = Draw a UV Plane display of Loop near to 3d Loop rendering, default:[FALSE]
                         SmPlane      ** pOptOutPlane=NULL,        // out: Set to Plane used for 2d graphics when bDrawUVPlane == TRUE, 
                                                                   //      NULL to ignore. default:[NULL],
                                                                   //      caller must delete this returned object
                         SmBoolean       bDrawUVTrimCurves=TRUE,   // in : TRUE = Draw UVTrimCurves when present, FALSE=don't
                                                                   //        note: when UVTrimCurves are good - they draw on top of the edges and aren't seen
                                                                   //              when UVTrimCurves are bad - they vary widely from the edges and indicate debugging is needed
                                                                   //      default:[TRUE]
                         SmGfxArraySet * pOptGfxSet=NULL)          // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                        const ;                                    //      NULL to ignore. default:[NULL]
                         

    void Dump(SmBoolean bAbbrev) const ; // in : FALSE = Also Dump UVTrimCurves, TRUE=don't, default:[FALSE]

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmLoop,SmTopology,SmLoop_TYPE);

} ; // end class SmLoop

#endif // !__SMLOOP_H__

