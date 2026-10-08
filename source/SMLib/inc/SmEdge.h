// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmEdge.h
* PURPOSE: Header file for SmEdge class.
**********************************************************************/

#ifndef __SMEDGE_H__
#define __SMEDGE_H__

#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#ifndef __SMVERTEUSE_H__
#include <SmVertexuse.h>
#endif

#ifndef __SMSHELL_H__
#include <SmShell.h>
#endif

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif
#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMTOLERANCE_H__
#include <SmTol.h>
#endif

#ifndef __SMCONTEXT_H__
#include <SmContext.h>
#endif

#ifndef __UNORDERED_SET__
#define __UNORDERED_SET__
#include <unordered_set>
#endif

class SmGapArray ;
class SmGfxArraySet ;

/*******************************************************************//**
PURPOSE: This class represents topological Edges within a Brep.

NOTES:
***********************************************************************/
class SM_EXPORT SmEdge : public SmOwningTopology
{
    friend class SmBrep;
    friend class SmFace;
    friend class SmEdgeuse;
// Remove Composites
    // friend class SmCEdge;
    friend class SmBrepConstructor;
    friend class SmObjsDelete<SmEdge*>;

protected:
    // inherited:
    // SmTopology::m_pListOwner - when in a Brep - SmBrep::m_pEdgeListHead
    // SmTopology::m_pNext      - Next member of edge list owned by SmBrep::m_pEdgeListHead
    // SmTopology::m_pLast      - Last member of edge list owned by SmBrep::m_pEdgeListHead
    //
    // SmOwningTopology::m_pList      - used to store pointer to primary SmEdgeuse (head of mate/radial link-list)
    //                                  The 2nd Edgeuse on the radial link-list must be the primary Edgeuse's mate.
    //                                  Depending on EdgeCurve/FaceSurface orientations the primary edgeuse can
    //                                  be either a member of the face's positive or negative sides but it must be the first edgeuse
    //                                  attached to a face connected to the edge that is encountered on a right-hand rule traversal around the edge.
    // SmOwningTopology::m_lListSize  - used to store number of Edgeuses for this Edge
    SM_NEWTOL_LINE // SmTopology::m_bIsSmallTopology - used by Face, Edge, Vertex (rarely needed - leave defaulted 99.99% of the time)
    //                                - TRUE = Intended small geometry size (Pinhole in Battleship) - gets tighter tolerances
    //                                - FALSE= Typical size - gets typical tolerances
    //                                -   default:[FALSE]

    SmCurve                           * m_pCurve;              // Pointer to curve defining geometry of edge
    SmExtent1d                          m_vInterval;           // Interval of edge along curve

    SM_OLDTOL_LINE mutable SmZoneTol3d m_sZoneTol3d ;         // Stored ZoneTol3d for this object

#ifdef USE_DEBUG_COUNTER
public:
    ULONG        m_lDebugCount; // For Debugging purposes
#endif

protected:

    // constructor
    SmEdge()                    : m_pCurve(NULL)
        SM_OLDTOL_LINE,m_sZoneTol3d(0.0)
    {
#ifdef USE_DEBUG_COUNTER
        m_lDebugCount = 0;
#endif // USE_DEBUG_COUNTER
        // report construction at SmObject::Notify level - skip other levels
        SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);
    } // end SmEdge constructor

    // destructor
    virtual ~SmEdge() ;

public:
    SmStatus AddOrientedEUPair
    (
      SmEdgeuse * pEU1,        ///< [in] :                                                      <br>
      SmEdgeuse * pEU2,        ///< [in] :                                                      <br>
      SmEdgeuse * pEUSector    ///< [in] : Specifies radial sector in which to insert the pair  <br>
    );         

    // Bounding box based on fine tessellation sample points.
    // SmBSplineCurve uses controlPolygon of containing spans.
    SmStatus CalculateBoundingBox
    (
      SmExtent3d  * pNormalBox = NULL, ///< [out]: axis aligned bounding box, NULL to ignore, default:[NULL]                    <br>
      SmPseudoBox * pPseudoBox = NULL, ///< [out]: non-Axis Aligned boundingBox, NULL to ignore, default:[NULL]                 <br>
      SmPolarBox  * pPolarBox = NULL,  ///< [out]: Curve tangent vector field bounding box, NULL to ignore, default:[NULL]      <br>
      SmBoolean     bTight = FALSE     ///< [in] : TRUE = compute minimal box  (expensive)                                      <br>
                                       ///<      : FALSE= compute any box larger than this Face (cheaper)                       <br>
    )  const ;                         

    // Calculate the precise bounding box.
    SmStatus CalculateTightBoundingBox( SmExtent3d * pNormalBox = NULL ) const;

    // find shape defined by faces connected to this edge to within ApproxTol (currently only a surf/surf intersection)
    SmStatus CalcEdgeFormFromFaces
    (
      SmBoolean        & rbFoundFaceEdgeForm,       ///< [out]: TRUE = Edge's EdgeForm defined by Face/Face intersection, rpBestCurve found.                                       <br>
      double           & rdMaxGap3d,                ///< [out]: when rbFoundFaceEdgeForm == TRUE, max distance between rpBestCurve and Face->Surfaces.                             <br>
      SmCurve         *& rpBestCurve,               ///< [out]: when rbFoundFaceEdgeForm == TRUE, the approximate Face/Face XSect Curve                                            <br>
      SmPoint3d        & rBestStartVertexPos,       ///< [out]: when rbFoundFaceEdgeForm == TRUE, StartVertexPos refined to Face/Face XSect Curve                                  <br>
      SmPoint3d        & rBestEndVertexPos,         ///< [out]: when rbFoundFaceEdgeForm == TRUE, EndVertexPos   refined to Face/Face XSect Curve                                  <br>
      SmFace          ** pOptFace1 = NULL,          ///< [out]: when rbFoundFaceEdgeForm == TRUE, Face1 of 2 used for XSect, NULL to ignore, default:[NULL]                        <br>
      SmBSplineCurve  ** pOptUVTrimCurve1 = NULL,   ///< [out]: when rbFoundFaceEdgeForm == TRUE, UVTrimCurve on Face1->Surface of new XSectCurve, NULL to ignore, default:[NULL]  <br>
      SmFace          ** pOptFace2 = NULL,          ///< [out]: when rbFoundFaceEdgeForm == TRUE, Face2 of 2 used for XSect, NULL to ignore, default:[NULL]                        <br>
      SmBSplineCurve  ** pOptUVTrimCurve2 = NULL    ///< [out]: when rbFoundFaceEdgeForm == TRUE, UVTrimCurve on Face2->Surface of new XSectCurve, NULL to ignore, default:[NULL]  <br>
    ) const ;

    // rtn TRUE when Faces parallel anywhere along the Edge
    SmBoolean CalcMaxParallelFaceGap
    (
      SmFace    & rFace1,                             ///< [in] : face1 of MaxParallelFace/Face Gap pair                                     <br>
      SmFace    & rFace2,                             ///< [in] : face2 of MaxParallelFace/Face Gap pair                                     <br>
      double    & rMinDihedralAngDeg,                 ///< [out]: Min Dihedral AngDeg seen between Faces                                     <br>
      double    & rMaxParallelFaceGap,                ///< [out]: Max gap between parallel face pairs                                        <br>
      double    * pOptAngTolDeg = NULL,               ///< [in] : optional max angle deg between parallel vectors,                           <br>
                                                      ///<      : NULL=SM_ANG_TOL_DEG, default:[NULL]                                        <br>
      SmPoint2d * pOptUV1 = NULL,                     ///< [out]: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair     <br>
      SmPoint2d * pOptUV2 = NULL                      ///< [out]: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair     <br>
    ) const ;

    // For a manifold edge in a manifold brep, returns exterior angle between faces, in radians
    SmStatus CalcOuterAngle (
      double    dParam,     ///< [in] :Edge parameter at which angle is calculated <br>
      double & rdAngle      ///< [out]:Outer angle at dParam in radians            <br>
    ) const;

    SmStatus ClosestPoint
    (
      const SmPoint3d & crPoint,      ///< [in] : Point to test
      SmBoolean & rbSuccess,          ///< [out]: TRUE = found a point, false= didn't
      double & rdParameter,           ///< [out]: curve param of closest point
      double & rdDistance             ///< [out]: dist to closest point
    );

    SmStatus ComputePreciseProperties
    (
      double dDesiredAccuracy,                  ///< [in] :             <br>
      const SmPoint3d & crOriginOfComputation,  ///< [in] :             <br>
      double dCrossSectionRadius,               ///< [out]:             <br>
      double & rdEdgeLength,                    ///< [out]:             <br>
      double & rdDeltaArea,                     ///< [out]:             <br>
      double & rdDeltaVolume,                   ///< [out]:             <br>
      SmTArray<SmVector3d> & rDeltaMoments      ///< [out]:             <br>
    ) const;

    // create a splineCurve copy of this edge's curve trimmed to the edge's interval
    SmBSplineCurve * CreateTrimmedNURBSCurve(const SmContext & crContext) const;

    // find radial sector containing (or Faceuse coincident with) given Edgeuse->Face
    SmStatus FindRadialSector
    (
      const SmEdgeuse * cpEdgeuseToClassify,            ///< [in] : contains face whose geometry will be checked               <br>
      const SmRegion  * cpRegionOfSector,               ///< [in] : If NULL it will                                            <br>
                                                        ///<      : test sectors in all regions for a possible answer.         <br>
      SmOrientType      eRequiredEUOrientation,         ///< [in] : The answer must be the edgeuse which has this orientation  <br>
                                                        ///<      : of the faceuse of an edgeuse with this orientation.        <br>
      SmEdgeuse      *& rpRadialEdgeuse,                ///< [out]: If an answer is found then this will be the                <br>
                                                        ///<      : resulting edgeuse; otherwise Null.  In case of             <br>
                                                        ///<      : coincidence this will be the edgeuse that's in             <br>
                                                        ///<      : rpCoincidentFaceuse.                                       <br>
      SmFaceuse      *& rpCoincidentFaceuse,            ///< [out]: If there is no coincidence this will be NULL.  If a        <br>
                                                        ///<      : coincidence is found then this is the coincident           <br>
                                                        ///<      : faceuse.  Note that we should never get a                  <br>
                                                        ///<      : coincidence if the region is specified.                    <br>
      SmTArray<SmFaceuse*> * pInsideFaceuses = NULL     ///< [in] : Inside face uses // LocalMerge                             <br>
    ) const;

    // classify Edgeuse against given radial sector near specified edgePoint
    SmStatus RadialSectorClassify
    (
      double            dNormalizedParam,               ///< [in] : edge location (normalized units)                       <br>
      const SmEdgeuse * cpEdgeuseOfSector,              ///< [in] : specifies sector region being tested                   <br>
      const SmEdgeuse * cpEdgeuseToClassify,            ///< [in] : contains face to classify                              <br>
      ULONG           & rlResult,                       ///< [out]: 0 - outside,                                           <br>
                                                        ///<      : 1 - inside,                                            <br>
                                                        ///<      : 2 - coincident with cpEdgeuseOfSector's face           <br>
                                                        ///<      : 3 - coincident with cpEdgeuseOfSector's mate face      <br>
                                                        ///<      : 4 - coincident with radial EU's face                   <br>
                                                        ///<      : 5 - coincident with radial mate's face                 <br>
                                                        ///<      : 6 - zero sector, in cpEdgeuseOfSector's face           <br>
                                                        ///<      : 7 - zero sector, in cpEdgeuseOfSector's mate face      <br>
      SmTArray<SmFaceuse*>  * pInsideFaceuses = NULL    ///< NotUsed: [in] : Faceuses that are 'inside' //LocalMerge                <br>
    ) const;

    // find radial sector containing (or Face coincident with) given curve
    SmStatus FindRadialSectorOfCurve
    (
      double             dEdgeParam,              ///< [in] : given edgePoint to examine                                     <br>
      const SmCurve    & crCurve,                 ///< [in] : curve to classify                                              <br>
      const SmExtent1d & crInterval,              ///< [in] : interval of curve                                              <br>
      SmOrientType       eOrientationToSector,    ///< [in] : If SM_OT_SAME - classify along positive direction of           <br>
                                                  ///<      :                 curve starting at crInterval.GetMin()          <br>
                                                  ///<      : If SM_OT_OPPOSITE - classify along reverse direction of        <br>
                                                  ///<      :                     curve starting at crInterval.GetMax()      <br>
      SmBoolean          bFourceSectoring,        ///< [in] : If TRUE will force the finding of RadialEdgeuse                <br>
                                                  ///<      : and not return coincident faceuse.                             <br>
      SmEdgeuse      *& rpRadialEdgeuse,          ///< [out]: Edgeuse of containing region or coincident face                <br>
      SmFaceuse      *& rpCoincidentFaceuse       ///< [out]: NULL for coincident curves                                     <br>
    ) const;                                      ///<      : else Faceuse containing given curve.                           <br>

    // classify curve against given radial sector at point where curve is bounded by edge
    SmStatus RadialSectorOfCurveClassify
    (
      const SmEdgeuse  * cpEdgeuseOfSector,       ///< [in] : contains face and radialPartner->face that define sector boundary     <br>
      double             dEdgeParam,              ///< [in] : point on edge to examine                                              <br>
      const SmCurve    & crCurve,                 ///< [in] : curve to classify                                                     <br>
      const SmExtent1d & crInterval,              ///< [in] : interval of curve                                                     <br>
      SmOrientType       eOrientationToSector,    ///< [in] : If SM_OT_SAME - classify curve along positive direction               <br>
                                                  ///<      :                 starting at crInterval.GetMin()                       <br>
                                                  ///<      : If SM_OT_OPPOSITE - classify curve along reverse direction            <br>
                                                  ///<      :                     starting at crInterval.GetMax()                   <br>
      SmBoolean          bForceSectoring,         ///< [in] : Will force a sector resolution even if coincidence                    <br>
                                                  ///<      : is found.  Result will be either 0 or 1 never 2 or 3.                 <br>
      ULONG            & rlResult                 ///< [out]: 0 - outside,                                                          <br>
                                                  ///<      : 1 - inside,                                                           <br>
                                                  ///<      : 2 - coincident with cpEdgeuseOfSector's face                          <br>
                                                  ///<      : 3 - coincident with radial EU's face                                  <br>
    ) const;

    SmStatus FindTangentEdgeString
    (
      double                   dAngleTolDeg,
      SmTArray<SmEdge*>      & rEdgeString,      ///< [in] : String of                                                        <br>
                                                 ///< [in] : connected edges ordered relative to the curve orientation        <br>
                                                 ///< [in] : of the first edges's curve.  Edges which connect to the          <br>
                                                 ///< [in] : this->GetInterval()->GetMin() end of the edge will be            <br>
                                                 ///< [in] : before this in the list.  Those which connect near the           <br>
                                                 ///< [in] : high parameter end of the edge's curve will be after this        <br>
                                                 ///< [in] : edge in the list.                                                <br>
      SmTArray<SmOrientType> & rEdgeOrients      ///< [in] : The orientation                                                  <br>
                                                 ///< [in] : of this edge will be SM_OT_SAME.  The other edges orientations   <br>
                                                 ///< [in] : will be how their curve aligns in the string relative to the     <br>
                                                 ///< [in] : edge.  The result should be that we can take the curves from     <br>
                                                 ///< [in] : the edges and produce a composite directly or with little work.  <br>
    );

    SmStatus InsertManifoldEdgeEnd(SmVertexuse * pVertexUse, SmBoolean bIsStartVertex);

    // clear cached  Edge->Curve param data including: Edge Edgeuse->MaxEdgeFaceGap3d, Edgeuse->EdgeCCWEdgeGap3d and vertexuse->VertexEdgeGap3ds
    SmStatus RemoveUVTrimCurves() { return ClearUVCaches() ; }  // backward compatible
    SmStatus ClearUVCaches() ;

    // replace a nonSmBSpline edge->curve with an SmBSpline Curve approximation
    SmStatus ApproximateWithBSpline
    (
      double  dApproxTol3d,       ///< [in] : maximum allowed distance between approx and original curves
      double &dAchievedTol3d      // out: actual max distance between approx and original curves
    ) ;

    ULONG    GetNumberFaceOccurances( const SmFace * cpFace )        const;
    void     GetFaces       ( SmTArray<SmFace*>      & rFaces,      ULONG *pOptAttributeId=NULL ) const;
    SmStatus GetFacesOrdered( SmTArray<SmFace*>      & rFaces )      const;  // Return Faces ordered around the Edge.
    SmStatus GetEdgeuses    ( SmTArray<SmEdgeuse*>   & rEdgeuses,   ULONG *pOptAttributeId=NULL, const SmTopology *pOptConnectedTo=NULL) const;
    SmStatus GetEdgeuses    ( std::unordered_set<SmEdgeuse*>& rEdgeuses ) const;
    void     GetVertices    ( SmTArray<SmVertex*>    & rVertices,   ULONG *pOptAttributeId=NULL ) const;
    void     GetVertexuses  ( SmTArray<SmVertexuse*> & rVertexuses, ULONG *pOptAttributeId=NULL  ) const;
    void     GetShells      ( SmTArray<SmShell *>    & rShells,     ULONG *pOptAttributeId=NULL  ) const;
    void     GetRegions     ( SmTArray<SmRegion *>   & rRegions,    ULONG *pOptAttributeId=NULL  ) const;

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
    //        MaxGap3d = Max( Edge/Face Vertex/Edge Gaps )  - does not include LoopGaps (get those from SmLoop)
    virtual const SmGap * GetMaxGap3d       (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const ; // Max(VertexEdge EdgeFace) Gap
    virtual const SmGap * GetMaxUpDimGap3d  (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const ; // Max EdgeFace Gap
    virtual const SmGap * GetMaxDownDimGap3d(SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const ; // Max VertexEdge Gap

    virtual SmBrep      * GetBrep               () const ;
    SmAObject           * GetAOwner             () const { return( (SmAObject*)GetBrep()) ; }
// Remove Composites
// SmCEdge             * GetCompositeEdgeOwner () const ;
    ULONG                 GetEdgeNumberInBrep   () const ;
    ULONG                 GetEdgeuseCount       () const  { return(m_lListSize) ; }
    SmEdgeuse           * GetPrimaryEdgeuse     () const  { SM_ASSERT(m_pList != NULL || m_cpContext == NULL || m_cpContext->GetDoingBoolean() );
                                                            return (SmEdgeuse*)m_pList ;
                                                          }
    SmEdgeuse           * GetSameOrientedEdgeuse() const ;
    SmExtent1d            GetInterval           () const  { return m_vInterval; }
    SmCurve             * GetCurve              () const  { return m_pCurve; }
    SmVertex            * GetVertex             () const ;
    SmVertex            * GetStartVertex        () const ;
    SmVertex            * GetOtherVertex        (const SmVertex *pVertex)   const ;
    SmVertex            * GetEndVertex          () const ;
    void                  GetVertices           (SmVertex *& rpStartVertex, SmVertex *& rpEndVertex)   const ;
    SmEdgeuse           * GetEdgeuseOfFace      (const SmFace *pFace)       const ;
    SmLoop              * GetLoopOfFace         (const SmFace *pFace)       const ;
    SmEdgeuse           * GetUpwardEdgeuseOfFace(const SmFace *pFace)       const ;
    SmEdgeuse           * GetBlendEdgeuse       ()                          const ; // rtn Edge->Edgeuse if edge is filletable, else rtn NULL
    SmStatus              GetEndTangent         (SmBoolean bAtStart, SmVector3d & rTangent)     const ;
    SmContinuityType      GetContinuity         (double dAngTolDeg=SM_CONTINUITY_ANGLE) const ; // If edge is manifold - return Continuity between faces
                                                                                                // else see SmEdgeuse::GetSectorContinuity.
                                                                                                // get or create and place a UVTrimCurve for this edge on its edgeuse.
    SmBSplineCurve      * GetUVTrimCurveOfSurface(const SmSurface * pSurface);

    SmBoolean        IsClosed()   const ;                          // GetVertex() == GetOtherVertex()
    SmBoolean        IsWire()     const ;                          // has 0 faces and Edge->PrimaryEdgeuse()->EdgeuseType == SmShell_TYPE
    SmBoolean        IsLamina()   const ;                          // has 2 edgeuses and 1 face = an edge of a face
    SmBoolean        IsManifold() const ;                          // has 4 edgeuses, 1 or 2 faces and edgeuse types are SmLoopuse_TYPE
    SmBoolean        IsSpine()    const ;                          // has 6 or more edgeuses, connects to faces 3 or more times, and edgeuse types are SmLoopuse_TYPE
    SmBoolean        IsStrut()    const ;                          // has an end which is connected to a vertex which is only connected to this edge
    SmBoolean        IsSeam()     const ;                          // is being used as a seam for any surface connected to it.
    SmBoolean        IsSeam( const SmSurface & crSurface ) const;  // has 4 edgeuses, 1 face, face's surface is closed, and 2 different UVTrimCurves.
    SmBoolean        IsEmbedded(const SmFace  * pOptFace=NULL,      // Return TRUE when 
                                                                    //      pOptFace NotNULL= has 4 edguses that connect to pOptFace
                                                                    //      pOptFace NULL   = has 4 edgeuses that connect to Face connected to Edge
                                SmBoolean bFaceHasSeam=TRUE) const; // bOptFaceHasSeam: TRUE = Face known to have SeamEdge - do Expensive IsSeam() check
                                                                    //                  FALSE= Face known to have no SeamEdge - skip IsSeam() check  

    // this edge is manifold, and its two faces are tangent along the edge.
    SmBoolean        IsTangentEdge 
    (
      double    dTangencyTolDeg,          
      SmBoolean bCoincidenceTest=FALSE    // Test for coincidence instead of tangency between faces.
    )  const; 

    SmBoolean        IsG2Edge() const ;
    SmBoolean        IsG3Edge() const ;

    // TRUE = Manifold Edge that can be removed without changing the shape of the Brep
    SmBoolean        IsTopological                          
    (
      SmSurface *&rpSurfToKeep,             ///< [out]: Surface to keep if 'this' gets deleted to merge two faces into one <br>
      double     &rDistBetween,             ///< [out]: max dist found between covered and covering surfaces               <br>
      double     dXSectTol3d_Gain=1.0,       ///< [in] : multiplier applied to Gap Tolerance checks                         <br>
      SmExtent2d *pOptMergedFaceDomain=NULL ///< [out]: optional union in retained surface's final NURBS parameters         <br>
                                           ///<        valid when TRUE
    ) const ;

    // TRUE = SM_ARE_SAME(m_sZoneTol3d == SmTol::GetZoneTol3d())
    SmBoolean        IsZoneTol3dConsistent(SmZoneTol3d * pOptZoneTol3d=NULL) const; 

    // TRUE  = edge is part of a tolerant corner where more than 3 faces come together not at a single point.
    SmBoolean        IsComplexCornerMember                                          
    (
      SmTArray<SmTopology*> *pOptComplexCornerElements=NULL,    ///< [out]: optional list of vertices and edges making up the corner  <br>
                                                                ///<      : NULL to ignore, default:[NULL].                           <br>
      SmTArray<SmFace*>     *pOptComplexCornerFaces=NULL        ///< [out]: optional list of faces meeting at this corner             <br>
                                                                ///<      : NULL to ignore, default:[NULL].                           <br>
    ) const;

    SmBoolean         IsConnectedToVertex(const SmVertex   * cpVertex)     const ;
    SmBoolean         IsConnectedToFace  (const SmFace     * cpFace)       const ;
    virtual SmBoolean IsConnectedTo      (const SmTopology * cpConnectTgt) const ;
// Remove Composites
//    SmBoolean         IsCompositeEdge    ()                               const { // TRUE = this edge is part of a composite edge
//                                                                                  return GetCurve() && this != (SmEdge *)GetCurve()->GetOwner() ; 
//                                                                                }
    SmBoolean        IsWithinXSectTol3dOfFaces
    (
      double    * pOptMaxGap3d=NULL,
      SmFace   ** pOptMaxGapFace=NULL,
      SmBoolean   bForceCalc=FALSE
    ) const ;
      
    // not being used
    //  // Swap order Edgeuse pair order in the Mate->Radial->Mate m_pList->m_pNext/m_pLast list, used as low level topo function for SmLoop::FlipLoopOrientation()
    //  SmStatus SwapEdgeuseOrder(SmEdgeuse & rEdgeuse1, SmEdgeuse & rEdgeuse2) ;

    SmBoolean        IsOnFaceEdgeForm   (double * pdOptMaxGap3d=NULL) const ;  // TRUE = Edge->Curve is within ApproxTol3d of all Edge->Faces

    SmBoolean        HasParallelFaces
    (
      double    * pOptAngTolDeg=NULL,          ///< [in] : optional max angle deg between parallel vectors,                        <br>
                                               ///<      : NULL=SM_ANG_TOL_DEG, default:[NULL]                                     <br>
      double    * pOptMinDihedralAngDeg=NULL,  ///< [out]: optional Min Dihedral AngDeg seen                                       <br>
      double    * pOptMaxParallelFaceGap=NULL, ///< [out]: when Rtn == TRUE, optional Max gap between parallel face pairs          <br>
      SmFace   ** pOptFace1=NULL,              ///< [out]: when Rtn == TRUE, optional face1 of MaxParallelFace/Face Gap pair       <br>
      SmFace   ** pOptFace2=NULL,              ///< [out]: when Rtn == TRUE, optional face2 of MaxParallelFace/Face Gap pair       <br>
      SmPoint2d * pOptUV1=NULL,                ///< [out]: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair  <br>
      SmPoint2d * pOptUV2=NULL                 ///< [out]: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair  <br>
    ) const ;

    // Set primary Edgeuse, keeping proper track of pointers, etc.
    SmStatus ResetPrimaryEdgeuse( SmEdgeuse *pNewPrimary );

    SM_OLDTOL_LINE // local tolerance management
    SM_OLDTOL_LINE virtual SmZoneTol3d GetTolerance    () const { return(m_sZoneTol3d) ; }   // old Tol ZoneTol3d stored on the object
    
    SM_OLDTOL_LINE virtual void        SetTolerance    (SmZoneTol3d sNewZoneTol3d, 
                                                        SmBoolean bUpdateOnlyIfLarger=TRUE,
                                                        SmBoolean bCascadeToBndries=TRUE) ;
    SM_OLDTOL_LINE SmStatus            RefreshTolerance(SmBoolean bNestedRefreshes=TRUE) ;  // Assign Edge and Faces connected to Edge consistent tolerance values

    SM_NEWTOL_LINE // local tolerance management
    SM_NEWTOL_LINE //   inherited from SmTopology
    SM_NEWTOL_LINE //     tolerance model: for pinholes in battleships (rarely used) - default:[FALSE] (99.9% of the time - leave it that way)
    SM_NEWTOL_LINE //        SmBoolean   IsSmallTopology() const ;
    SM_NEWTOL_LINE //        void        SetIsSmallTopology(SmBoolean bIsSmall) ;
    SM_NEWTOL_LINE //
    SM_NEWTOL_LINE //     tolerance model: obsolete old-style compatible - instead use SmTol::GetZoneTol3d(this) ;
    SM_NEWTOL_LINE //        SmZoneTol3d GetTolerance() const                   { return SmTol::GetZoneTol3d(this) ; } // GWC: obsolete
    SM_NEWTOL_LINE //        void        SetTolerance(SmZoneTol3d, SmBoolean)   { /* no action - only for backward compatibility */ ; }

    void     SetInterval   (const SmExtent1d & crInterval)  { m_vInterval = crInterval ; }
    void     SetOrientation(SmOrientType       eOrientation); // assumes radially ordered edgeuses - assigns eOrientation to 1st EU and Opposite to next and so on

    // set Edge->UVTrimCurve for the specifed Face (deletes PreExisting UVTrimCurve for that Face)
    SmStatus SetTrimCurve  
    (
      SmFace         * pFace,                   ///< [in] : Face connected to this edge                   <br>
      SmBSplineCurve * pUVTrimCurve,            ///< [in] : UVTrimCurve to store                          <br>
      double           dMaxDistanceToSurface,   ///< [in] : known Max Edge/>FaceTrimCurve gap             <br>
      SmBoolean        bDoValidation = TRUE     ///< [in] : TRUE = run ValidateGeometry() after change    <br>
                                                ///<      : FALSE=                                        <br>
    );                                          

    // Set Edge->Curve. Retaining the old curve clears its owner only if it is
    // this edge. The caller must set the new curve's owner separately.
    void SetCurve      
    (
      SmCurve  * pCurve,                ///< [in] : New Curve for Edge                                                                   <br>
      SmBoolean  bDeleteOldCurve=FALSE, ///< [in] : TRUE = delete Old Curve, FALSE=don't, default:[FALSE]                                <br>
      SmBoolean  bDbgWarnLeaks=TRUE,    ///< [in] : TRUE = Signal DBGWarn when saving new curve over old, FALSE=don't, default:[TRUE]    <br>
      SmBoolean  bCallNotify=TRUE       ///< [in] : TRUE = delete all UVTrimCurves, FALSE=don't, default:[TRUE]                          <br>
    );

    void     ReverseOrientation() ;                          // assumes oriented edgeuses and Curve - reverses CurveParam & Edgeuse order, negates Edgeuse orients

    // Given a sequence of Edges connected end-to-end, set all orientations in the same direction as the sequence.
    static SmStatus OrientEdgeSequence( SmTArray< SmEdge* > & rEdgeList );

    // Set Edge->Curve and Curve->Owner back ptrs: works for CEdge and Edges - (calls SetCurve which deletes PreExisting Curve and UVTrimCurves)
    SmStatus Replace3DCurve(SmCurve *p3DCurve) ;

    // Compute curve shape based on surf/surf intersections if possible
    SmStatus RefineGeometry
    (
      SmMarkType eMarkType,     ///< [in] : uses without incrementing eMarkType value.                                         <br>
                                ///<      : skip marked edges and mark all others.                                             <br>
                                ///<      : oneof SM_MT_NOMARK = don't check marks before refining geometry                    <br>
                                ///<      :       SM_MT_MARK   = skip marked edges and mark all others                         <br>
                                ///<      :       SM_MT_MARK2                                                                  <br>
                                ///<      :       SM_MT_MARKIO                                                                 <br>
      SmBoolean &bMadeChange    ///< [out]: TRUE = replaced edgeCurve with new surf/surf intersection and updated tolerance.   <br>
                                ///<      : FALSE= no changes made because it could not find a good surf/surf intersection     <br>
    );                         

    // For when curve parameterization changes:
    virtual SmStatus UpdateDomain();

    virtual void Notify
    (
      SmNotifyOperation eNotifyOperation,     // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
      SmObject        * pData1,               //       event                | caller      |  pData1  | pData2                | pData3                   
      SmObject        * pData2,               //----------------------------+-------------+----------+-----------------------+--------------------------
      SmObject        * pData3                // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL     
                                              // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                     
                                              // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
                                              // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                              // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                              // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
                                              // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
                                              // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
                                              // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                      
                                              // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
                                              // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                              // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                              // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL   
                                              // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
                                              // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                              // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     
    ) ;

    // output Edge polyline with optional knot point and/or controlPoint polygon
    SmStatus OutputGraphics
    (
      const SmDisplayParameters & crDisp,             ///< [in] : graphic display parameters                                                                        <br>
      SmGfxArraySet             * pOptGfxSet = NULL   ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore.   <br>                                
    ) const;

    // output edge, as a polyline, to new displayList
    virtual SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet= NULL) const;

    // output edge with ordered Edgeuses
    SmDisplayList         * DrawNeighbors(SmGfxArraySet * pOptGfxSet= NULL) const;

    // output micro view of geometry connecting to this edge
    SmDisplayList * DrawMicro
    (
      double        * pOptMicroParam = NULL, ///< [in] : optional Edge param to be center of Micro Drawing                                                <br>
      SmGfxArraySet * pOptGfxSet = NULL      ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore.  <br>
    ) const;

    // output transformed Edge->Curve copy to new displayList
    SmDisplayList         * DrawWTransform(const SmAxis2Placement & crTransform) const;

    // Draw Edge->Curve(Ivl) marked with sequenced Param Points increasing in size and changing color from green to blue
    SmDisplayList         * DrawParams(SmGfxArraySet * pOptGfxSet=NULL) const
    {
        return(  m_pCurve
                 ? m_pCurve->DrawParams(&m_vInterval,
                                        pOptGfxSet)
                 : NULL) ;
    }

    SmStatus ValidateGeometry(SmBoolean bFixTolerances=FALSE) const;

    // get memory used for edge, its curve, all its edgeuses and their attributes
    virtual ULONG GetMemoryUsed
    (
      ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes    <br>
      SmMarkType eMarkType=SM_MT_NOMARK  ///< [in] : uses without increment eMarkType value          <br>
    ) const ;

    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
                                                ///<      : default:[SM_LEVEL_0]                                                                              <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
    ) const ;                                      

    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmEdge,SmOwningTopology,SmEdge_TYPE);

    void Dump(ULONG) const;
    void Dump(TCHAR * message) const;
    void Dump(SmBoolean bAbbrev)const;
    void DumpTopology(ULONG lWalkDepth=0) const ;  // lWalkDepth[0] = no walk, [99] = walk to bottom

} ; // end class SmEdge

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmEdge*) ;

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE
//
//    extern SmEdge * dbgEdge1 ;
//    extern SmEdge * dbgEdge2 ;
//
//    #endif // SM_DEBUG_CODE

#endif // !__SMEDGE_H__
