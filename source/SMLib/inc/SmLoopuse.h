// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmLoopuse.h
* PURPOSE: Header file for SmLoopuse class.
**********************************************************************/

#ifndef __SMLOOPUSE_H__
#define __SMLOOPUSE_H__

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

#ifndef __SMSHELL_H__
#include <SmShell.h> 
#endif

#ifndef __SMFACEUSE_H__
#include <SmFaceuse.h>
#endif

#ifndef __UNORDERED_SET__
#define __UNORDERED_SET__
#include <unordered_set>
#endif 

class SmPlane ;

/*******************************************************************//**
PURPOSE: This class represents topological Loopuses.

NOTES: 
***********************************************************************/
class SM_EXPORT SmLoopuse : public SmTopology
{
    friend class SmBrep;
    friend class SmLoop;
    friend class SmEdgeuse;
    friend class SmFace;
    friend class SmShell;
    friend class SmBrepConstructor;

protected:
    // inherited
    // smTopology::m_pListOwner - used to store pointer to owning Faceuse
    // SmTopology::m_pNext      - used to store doubly linked list of 
    // SmTopology::m_pLast           SmLoopuse objects belonging to Faceuse

    SmLoop             * m_pL;            // this Loopuse's Loop
    SmLoopuse          * m_pLUMate;       // Loopuse on face's other side
    SmOrientType         m_eOrientation;  // SM_OT_SAME     = outer Loop
                                          // SM_OT_OPPOSITE = inner Loop 
    SM_TYPE              m_tLoopuseType;  // type of loop: SmVertexuse_TYPE (16006)
                                          //               SmEdgeuse_TYPE   (16005)
    SmTopology         * m_pEUorVU;       // pointer to either loop's start Edgeuse or 
                                          //                   single Vertexuse
protected:
    SmLoopuse() : m_pL(NULL), 
                  m_pLUMate(NULL), 
                  m_eOrientation(SM_OT_UNKNOWN),
                  m_tLoopuseType(SmUnknown_TYPE), 
                  m_pEUorVU(NULL) { 
                                    // report construction at SmObject::Notify level - skip other levels
                                    SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);                 
                                  }
public:
    virtual ~SmLoopuse() { Notify(SM_NO_DESTRUCTION, this, NULL, NULL);
                           m_pL           = NULL ;
                           m_pLUMate      = NULL ;
                           m_eOrientation = SM_OT_UNKNOWN ;
                           m_tLoopuseType = SmUnknown_TYPE ;
                           m_pEUorVU      = NULL ;
                         }

    // set loop's Edgeuse ptr and 
    // walk Edgeuse CCW_ptrs setting each Edgeuse's loopuse BackPtr to this Loopuse 
    SmStatus CollectEdgeuses(SmEdgeuse * pEdgeuse);

    SmStatus CalculateBoundingBox(SmExtent3d  & rBBox3d,
                                  SmPseudoBox * pOptPseudoBox=NULL,
                                  SmExtent2d  * pOptBBoxUV=NULL) const;
	// Overloaded GetEdges and GetVertices for use with unordered_set, JLMCC
	void                GetEdges				(std::unordered_set<SmEdge*>& rEdges, ULONG* pOptAttributeId = NULL) const;
	void                GetVertices				(std::unordered_set<SmVertex*>& rVertices, ULONG* pOptAttributeId = NULL) const;

    virtual SmBrep    * GetBrep()         const { return ((SmFaceuse*)GetOwner())->GetShell()->GetBrep(); }
    void                GetEdges                (SmTArray<SmEdge*>      & rEdges,      ULONG *pOptAttributeId=NULL) const ;
    void                GetEdgeuses             (SmTArray<SmEdgeuse*>   & rEdgeuses,   ULONG *pOptAttributeId=NULL, SmTopology *pOptConnectedToTarget=NULL) const ;
    void                GetVertices             (SmTArray<SmVertex*>    & rVertices,   ULONG *pOptAttributeId=NULL) const ;
    void                GetVertexuses           (SmTArray<SmVertexuse*> & rVertexuses, ULONG *pOptAttributeId=NULL) const ;
    void                GetEdgeusesAndVertexuses(SmTArray<SmEdgeuse*>   & rEdgeuses, SmTArray<SmVertexuse*> & rVertexuses, ULONG *pOptAttributeId=NULL) const ;  
    SmVertexuse       * GetVertexuse ()    const { return (m_tLoopuseType == SmVertexuse_TYPE) ? (SmVertexuse*)m_pEUorVU : NULL ; }
    SmFaceuse         * GetFaceuse()       const { return (SmFaceuse*)GetOwner(); }
    SmLoop            * GetLoop()          const { SM_ASSERT(m_pL != NULL); return m_pL; }
    SmOrientType        GetOrientation()   const { return m_eOrientation; }
    SmLoopuse         * GetOtherLoopuse()  const { SM_ASSERT(m_pLUMate != NULL); return m_pLUMate; }
    SmShell           * GetShell()         const ;
    virtual SmAObject * GetAOwner()        const { return (SmAObject*)GetLoop() ; } // get attribute inheritance owner
    SmTopology        * GetEUorVU()        const { return m_pEUorVU ; }
    SM_TYPE             GetLoopuseType()   const { return m_tLoopuseType ; }
    double              GetTurningAngDeg() const ;

    SmBoolean           IsVertexLoopuse()  const { return(m_tLoopuseType == SmVertexuse_TYPE) ; }
    SmBoolean           IsEdgeLoopuse()    const { return(m_tLoopuseType == SmEdgeuse_TYPE) ; }
    SmBoolean           IsClosed3d                                 // rtn: TRUE when all EdgeEnd/EdgeEnd Gap3ds are within their associated XSectTol3d limit values
                          (SmBoolean       bCheckAllGaps = TRUE,   // in : TRUE = Check all gaps after finding the first open EdgeuseEnd-EdgeuseEnd gap to get true MaxGap values
                                                                   //      FALSE= quit after finding the 1st open EdgeuseEnd-EdgeuseEnd gap to save time.
                           SmEdgeuse    ** ppOptEdgeuse  = NULL,   // out: When any gaps are out-of-tol, the 1st edgeuse of the gap that exceeds its XSectTol3d value the most,
                                                                   //      When all gaps are in-tol, the largest gap between Edgeuse ends.
                                                                   //      CCW Edgeuse is the other end of gap. NULL to ignore, Default:[NULL]
                           double        * pdOptMaxGap   = NULL,   // out: Dist for gap being returned by ppOptEdgeuse
                                                                   //      NULL to ignore, Default:[NULL]
                           SmXSectTol3d  * psOptXSectTol3d = NULL) // out: XSectTol3d value for gap being returned by ppOptEdgeuse. 
                                           const ;                 //      NULL to ignore, Default:[NULL]
                                                                   
    SmBoolean           IsClosedPtrs()     const ;  // rtn: TRUE when Edgeuse LinkedList of CCWEdgeuse ptrs form a closed LinkedList
    virtual SmBoolean   IsConnectedTo(const SmTopology *cpConnectTgt) const ;
    SmBoolean           IsLoopuseSideInSurface(SmBoolean * pOptPeriodicU=NULL,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]
                                               SmBoolean * pOptPeriodicV=NULL,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]
                                               SmBoolean * pOptOnNaturalBoundary=NULL, // in : TRUE=at least 1 edgeuse is on SurfNatBndry, FALSE=Not, NULL to ignore, default:[NULL]
                                               ULONG     * pOptEdgeuseIndx=NULL)        // out: When FALSE is returned, optional Index value of 1st Edgeuse 
                                           const ;                                     //        that fails the Side test, NULL to ignore, default:[NULL]
    SmBoolean           IsMissingSeamAtVertices() const ; // rtn: TRUE when any Loop->VertexSector spans a missing seam
                                                          //      Use only after splitting all Edges at the SeamCurve

    void     SetOrientation(SmOrientType eNewOrientation) { m_eOrientation = eNewOrientation; }
    void     SetLoop       (SmLoop *pLoop)                { SM_ASSERT(m_pL == NULL || pLoop == NULL); m_pL = pLoop ; }
    SmStatus SetEdgeuse    (SmEdgeuse* pEdgeuse);
                                                              
    SmDisplayList   * Draw(ULONG           bVUAndEUDraw=3,          // in : oneof 1=Draw VUs, 2=Draw EUs, 3=Draw VUs and EUs
                           SmBoolean       bDrawUVPlane=FALSE,     // in : TRUE = Draw a UV Plane display of Loop near to 3d Loop rendering, default:[FALSE]
                           SmPlane      ** pOptOutPlane=NULL,      // out: Set to Plane used for 2d graphics when bDrawUVPlane == TRUE, 
                                                                   //      NULL to ignore. default:[NULL],
                                                                   //      caller must delete this returned object
                           SmBoolean       bDrawUVTrimCurves=TRUE, // in : TRUE = Draw UVTrimCurves when present, FALSE=don't
                                                                   //        note: when UVTrimCurves are good - they draw on top of the edges and aren't seen
                                                                   //              when UVTrimCurves are bad - they vary widely from the edges and indicate debugging is needed
                                                                   //      default:[TRUE]
                           SmGfxArraySet * pOptGfxSet=NULL)        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                          const ;                                  //      NULL to ignore. default:[NULL]

    void              Dump(SmBoolean bAbbrev) const ; // in : FALSE=, TRUE=, default:[TRUE]

    virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //      default:[SM_LEVEL_0] 
                                  SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                  SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                 const ;
    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmLoopuse,SmTopology,SmLoopuse_TYPE);

} ; // end class SmLoopuse

#endif // !__SMLOOPUSE_H__

