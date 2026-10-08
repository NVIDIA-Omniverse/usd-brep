// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPlane.h
* PURPOSE: Header file for Planar Surface class.
**********************************************************************/

#ifndef __SMPLANE_H__
#define __SMPLANE_H__

#ifndef __SMBSPLINESURFACE_H__
#include <SmBSplineSurface.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

// Make analytics from higher order surfaces if the geometry fits.
// If you don't define this it will only use degree 1 surfaces to make planes.  
#define HIGHER_ORDER_USE_ANALYTICS 1

// When converting to analytics this will reduce the degree of things such as degree 3 planes.
#define REDUCE_DEGREE_FOR_ANALYTICS 1

/*******************************************************************//**
PURPOSE:  A plane is an unbounded or a bounded surface with a constant normal.
    It is defined by a point on the plane and the normal direction
    to the plane.

NOTES: 

  PlaneEvaluation(sUV) =   m_vPosition.GetOriginRef()
                         + sUV.x * m_vUVScale.x * m_vPosition.GetXAxisRef()
                         + sUV.y * m_vUVScale.y * m_vPosition.GetYAxisRef() ;

  The analytic definition is used for evaluation.  This means that the
  analytic and nurbs parameter domains must be the same.
***********************************************************************/
class SM_EXPORT SmPlane : public SmBSplineSurface
{
private:
  // PlaneEquation =   Origin                                                 
  //                 + UScale * XAxis * u                                      
  //                 + VScale * YAxis * v                                     
  SmAxis2Placement m_vPosition;        // Origin, unit XAxis and unit YAxis of PlaneEquation
  SmExtent2d       m_vAnalUVDomain;    // Valid parmetric domain,
                                       //   min/max values define the 
                                       //   associated m_pNurb domain limits. 
  SmVector2d       m_vUVScale;         // UScale and VScale of PlaneEquation

public:
  // constructor - finite plane 
  SmPlane
  (
    const SmPoint3d  & crOrigin,         ///< [in] : 3d loc of parametric origin = Plane_Evaluate(0,0)              <br>
    const SmVector3d & crXAxis,          ///< [in] : 3D X Axis - made unit                                          <br>
    const SmVector3d & crYAxis,          ///< [in] : 3D Y Axis - made unit - should be perp to X                    <br>
    const SmVector2d & crUVScale,        ///< [in] : UScale and VScale                                              <br>
    const SmExtent2d & crUVDomain,       ///< [in] : UV Domain limiting allowed evaluations                         <br>
    const SmContext  * cpContext = NULL  ///< [in] : Set context if given, default:[NULL]                           <br>
  );

  // constructor - infinite plane
  SmPlane
  (
    const SmPoint3d  & crOrigin,         ///< [in] : 3d loc of parametric origin = Plane.Evaluate(0,0)                  <br>
    const SmVector3d & crPlaneNormal,    ///< [in] : normal vetor to plane (does not have to be unit)                   <br>
    const SmContext  * cpContext = NULL  ///< [in] : Set context if given, default:[NULL]                               <br>
                                         // notes: UScale = VScale = 1.0; unitize(PlaneNormal) = cross(XAxis,YAxis)     <br>
  );

  // empty constructor for I/O
  SmPlane() {}

  // copy constructor
  SmPlane( const SmPlane & crPlane );

  // destructor
  virtual ~SmPlane() {}

  // equality operator
  virtual SmBoolean operator==( const SmSurface &crOther ) const;

  virtual SmStatus AdjustSTEPUVDomain( const SmExtent2d & crNewSTEPUVDomain );

  virtual SmStatus Copy
  (
    const SmContext & crContext,
    SmSurface      *& rpNewSurface
  ) const;

  SmStatus Copy
  (
      const SmContext& crContext,
      SmPlane*& rpNewPlane
  ) const;

  // create a step parameterized infinite surface                          
  static SmStatus CreateCanonical
  (
    const SmContext        & crContext,                   ///< [in] : context for new object construction     <br>
    const SmAxis2Placement & crOrigin,                    ///< [in] : position and orientation of new plane   <br>
    SmPlane               *& rpNewPlane                   ///< [out]: the new plane                           <br>
  );                                                                                       

  virtual SmStatus CreateIsoParametricCurve
  (
    const SmContext  & crContext,                         ///< [in] : context for created objects                                                            <br>
    SmSurfParamType    eSurfParam,                        ///< [in] : Defines which Nurb parameter direction on surface to extract curve from                <br>
                                                          //      SM_SP_U = create constant u isoParameter curve                                             <br>
                                                          //      SM_SP_V = create constant v isoParameter curve                                             <br>
    double             dIsoParameter,                     ///< [in] : Defines Nurb parametric value at which to extract the curve.                           <br>
                                                          //      If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                       <br>
                                                          //      then this is the V parameter                                                               <br>
    SmApproxTol3d      sApproxTol3d,                      ///< NotUsed: [in] : passed to ApproximateCurve() when approximation is required.                           <br>
                                                          //      If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]<br>
    SmBSplineCurve  *& rpNewIsoCurve,                     ///< [out]: 3d IsoParameterCurve                                                                   <br>
    const SmExtent2d * pOptDomain = NULL,                 ///< [in] : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]              <br>
    double           * pOptMaxGap3d = NULL,               ///< [out]: opt achieved max gap, NULL to ignore, default:[NULL]                                   <br>
    SmCurve         ** pOptUVIsoCurve = NULL              ///< [out]: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]        <br>
  )  const;

  // create offset surface - Out Surf->Domain(s) will equal input surface domain
  virtual SmStatus CreateOffsetSurface
  (
    const SmContext      & crContext,                    ///< [in] : context for new obj construction                                              <br>
    double                 dSignedOffsetDistance,        ///< [in] : offset dist, (neg val = Offset dir opposite surface normal)                   <br>
    SmApproxTol3d          sApproxTol3d,                 ///< NotUsed: [in] : Max Dist between ApproxOffsetSurface and ideal offset shape                   <br>
    SmSurface* &          rOffsetSurface                 ///< [out]: Offset Surf Approx, may be more than 1 when offsets have self-intersections   <br>
  ) const;

  virtual SmStatus CreateSurfaceRay
  (
    const SmContext  & crContext,                        ///< [in] : Context for new objects          <br>
    const SmExtent2d & crUVDomain,                       ///< [in] : Surface domain to consider       <br>
    const SmPoint2d  & crStartPoint,                     ///< [in] : Ray Start Point on Surface       <br>
    SmBoolean          bCurveInParameterSpace,           ///< [in] : TRUE  = build 2d Curve,          <br>
                                                         //      FALSE = build 3d curve               <br>
    SmBoolean          bTowardUpperBoundaryOfDomain,     ///< [in] : TRUE  = direct ray up            <br>
                                                         //      FALSE = direct ray down              <br>
    SmSurfParamType    eSurfParam,                       ///< [in] : parameter held constant          <br>
    SmApproxTol3d      d3DTolerance,                     ///< [in] : Not used for planes              <br>
    SmBSplineCurve  *& rpRayCurve                        ///< [out]: Ray                              <br>
  ) const;

  virtual SmStatus DropCurve
  (
    const SmContext           & crContext,               ///< [in] : context for new object construction                                                          <br>
    const SmExtent2d          & crUVDomain,              ///< [in] : domain of interest for this surface                                                          <br>
    const SmCurve             & cr3dCurve,               ///< [in] : Curve to project onto the surface                                                            <br>
    const SmExtent1d          & crInterval,              ///< [in] : interval of interest for target curve                                                        <br>
    SmApproxTol3d               dApproxTol,              ///< [in] : max allowed distance between drop point and m_SrfNormal line at drop point                   <br>
    double                    & rdMaxDropToSurf,         ///< [out]: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.         <br>
    double                    & rdMaxApproxDev,          ///< [out]: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0      <br>     
    SmTArray<SmBSplineCurve*> & rUVCurves,               ///< [out]: 1 (or 2) curves constructed by projection.                                                   <br>  
                                                         ///<        (2 curves for closed surfaces when cr3dCurve is coincident with seam)                        <br>  
    SmBoolean                   bKeepIfOnSurface,        ///< [in] : TRUE = return all drop curves that stay over the surface (those that wander off do not drop) <br>
                                                         ///<        FALSE= only return drop curves with drop distances less than dApproxTol                      <br>
    SmDropCurveFail            * pOptDropCurveFail,      ///< NotUsed: [out]: Optional data container of a DropCurve fail, not used otherwise                              <br>
                                                         ///<        NUll to ignore, default:[NULL]                                                               <br>
    SmBoolean                   bCreatingUVTrimCurve     ///< [in] : TRUE when creating a UVTrimCurve. This forces the UVTrimcurve parameterization to match crInterval <br>
  ) const;      

  // find point that drops within analytic boundary to within tolerance - else find no solutions
  virtual SmStatus DropPointFast
  (
    const SmExtent2d       & crUVDomain,                ///< [in] : this plane's domain                                             <br>
    SmSolverOperationType    eSolverOperation,          ///< [in] : oneof SM_SO_MINIMIZE, SM_SO_NORMALIZE, or SM_SO_INTERSECT       <br>
    const SmPoint3d        & crTestPoint,               ///< [in] : target point                                                    <br>
    const SmVector3d       * cpOptInPointingVector,     ///< NotUsed: [in] : cpOptInPointingVector not used                                  <br>
    double                   dDistanceTolerance,        ///< [in] : when eSolverOperation == SM_SO_INTERSECT                        <br>
                                                        //        skip solutions larger than DistanceTolerance                      <br>
    const double           * cpdOptTargetDistance,      ///< [in] : when eSolverOperation == SM_SO_MINIMIZE                         <br>
                                                        //        skip solutions larger than TargetDistance                         <br>
                                                        //        NULL to ignore.                                                   <br>
    SmSolutionRequestedType  eSolutionRequested,        ///< NotUsed: [in] :                                                                 <br>
    SmSolutionArray        & rSolutions                 ///< [out]: Solution array with 0 or 1 entries                              <br>
  ) const;

  // evaluate clamped UVPoints
  virtual SmStatus EvaluatePoint( const SmPoint2d & crUV, SmPoint3d & rPoint ) const;

  // evaluate unclamped UVPoints
  SmStatus EvaluatePointFast( const SmPoint2d & crStepUV, SmPoint3d & rPoint ) const;

  // evaluate surface without any adjustments for ZeroTangents
  virtual SmStatus EvaluateSimple
  (
    const SmPoint2d & crUV,                       ///< [in] :             <br>
    ULONG             lHighestUDeriv,             ///< [in] :             <br>
    ULONG             lHighestVDeriv,             ///< [in] :             <br>
    SmBoolean         bUFromLeft,                 ///< [in] :             <br>
    SmBoolean         bVFromLeft,                 ///< [in] :             <br>
    SmBoolean         bOnlyUpperHalf,             ///< [in] :             <br>
    SmVector3d      * aDerivatives                ///< [in] :             <br>
  ) const
  {
    return Evaluate( crUV, lHighestUDeriv, lHighestVDeriv, bUFromLeft, bVFromLeft, bOnlyUpperHalf, aDerivatives );
  }

  // evaluate clamped UVPoints
  virtual SmStatus Evaluate
  (
    const SmPoint2d & crUV,            ///< [in] : param value to evaluate                                                              <br>
    ULONG lHighestUDeriv,              ///< [in] : number of U derivatives                                                              <br>
    ULONG lHighestVDeriv,              ///< [in] : number of V derivatives to compute                                                   <br>
    SmBoolean bUFromLeft,              ///< [in] : if P is on U interval boundary                                                       <br>
                                       //      TRUE  = evaluate P in upper interval where P is on the left of the interval              <br>
                                       //      FALSE = evaluate P in lower interval where P is on the right of the interval             <br>
    SmBoolean bVFromLeft,              ///< [in] : if P is on V interval boundary                                                       <br>
                                       //      TRUE  = evaluate P in upper interval where P is on the left of the interval              <br>
                                       //      FALSE = evaluate P in lower interval where P is on the right of the interval             <br>
    SmBoolean bOnlyUpperHalf,          ///< [in] : TRUE=compute upper half of matrix only                                               <br>
                                       //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            <br>
                                       //                [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)       <br>
                                       //                              [Dvv  ---   ---]                                                 <br>
    SmVector3d *aDerivatives,          ///< [out]: matrix of evaluations values                                                         <br>
                                       //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               <br>
                                       //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)    <br>
                                       //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )    <br>
                                       //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                       <br>
                                       //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                       <br>
                                       //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] <br>
   SmBoolean bNonZeroTangents = TRUE,  ///< NotUsed: [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors         <br>
                                       //      FALSE= return exact tangent values                                                       <br>
                                       //      note: Surprisingly TRUE is the common choice because most tangent uses                   <br>
                                       //            are for their direction (Binorm, SurfNorm comps), but when the                     <br>
                                       //            tangent is being used for its magnitude (like an arc-length comp)                  <br>
                                       //            then set this to FALSE.                                                            <br>
                                       //      default:[TRUE]                                                                           <br>
   SmBoolean bDoZeroSampling = TRUE    ///< [in] : for internal use only, always set to TRUE, default:[TRUE]                            <br>
  ) const;

  virtual SmStatus EvaluateNormal
  (
    const SmPoint2d & crUV,
    SmBoolean bUFromLeft,              ///< [in] : if P is on U interval boundary                                                 <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval    <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval   <br>
    SmBoolean bVFromLeft,
    SmVector3d & rSurfaceNormal
  ) const;

  SmStatus GetCanonical( SmAxis2Placement & rOrigin ) const;

  virtual SmExtent2d GetSTEPUVDomain() const;


  const SmAxis2Placement & GetPosition() const;

  virtual SmVector2d GetUVScale() const;

  virtual SmStatus GlobalPointSolveSTEP
  (
    const SmExtent2d      & crSTEPUVDomain,             ///< [in] : step domain               <br>
    SmSolverOperationType   eSolverOperation,           ///< [in] :                           <br>
    const SmPoint3d       & crTestPoint,                ///< [in] :                           <br>
    double                  dDistanceTolerance,         ///< [in] :                           <br>
    const double          * cpdOptTargetDistance,       ///< [in] :                           <br>
    SmSolutionRequestedType eSolutionRequested,         ///< [in] :                           <br>
    SmSolutionArray       & rSolutions                  ///< [out]: step domain parameters    <br>
  );

  virtual SmStatus GlobalLineIntersect
  (
    const SmExtent2d & crUVDomain,                     ///< [in] : Plane's Domain                                                    <br>
    const SmPoint3d  & crLinePoint,                    ///< [in] : Point on the infinite line                                        <br>
    const SmVector3d & crLineVector,                   ///< [in] : Direction vector of the infinite line                             <br>
    const SmExtent1d * cpOptLineInterval,              ///< [in] : If specified bounds the line to a specific segment                <br>
    SmBoolean          bFireRay,                       ///< [in] : TRUE = bounded at StartPt and proceeds along Vec to infinity.     <br>
                                                       ///<      : FALSE= bounded by cpOptLineInterval if given, else infinite.      <br>
    double             dDistanceTolerance,             ///< [in] : Max Dist at which two points still intersect                      <br>
    SmSolutionArray  & rSolutions                      ///< [out]: solutions: sSol.m_vStart[0] = Line param                          <br>
                                                       ///<      :        sSol.m_vStart[1] = Plane param U                           <br>
                                                       ///<      :        sSol.m_vStart[2] = Plane param V                           <br>
  ) const;                                                                                                               

  virtual SmStatus GlobalSurfaceIntersect
  (
    const SmContext     & crContext,                   ///< [in] : Context for the creation of curves                                                    <br>
    const SmExtent2d    & crUVDomain,                  ///<      :                                                                                       <br>
    const SmSurface     & crOtherSurface,              ///< [in] : target 2nd intersecting surface                                                       <br>
    const SmExtent2d    & crOtherUVDomain,             ///<      :                                                                                       <br>
    const SmBoolean       bUseSurfaceEdges[2],         ///< [in] : Normally both are TRUE unless you know                                                <br>
                                                       ///<      : that the edges of one surface do not intersect                                        <br>
                                                       ///<      : the other surface.  It is a slight optimization                                       <br>
                                                       ///<      : to set the flag to FALSE                                                              <br>
    const SmApproxTol3d * pdOptApproxTol3d,            ///< [in] : If not given it uses 1/1000 of surface size                                           <br>
                                                       ///<      : approximation tolerance                                                               <br>
    const double        * pdOptAngTolRad,              ///< [in] : If not given it uses 30 degrees                                                       <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,                ///< [out]: 3D curves produced by intersection                                                    <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,        ///< [out]: UV curves on this surface produced by intersection                                    <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,        ///< [out]: UV curves on crOtherSurface produced by intersection                                  <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,       ///< [out]: What type of curve is produced -                                                      <br>
                                                       ///<      : oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)             <br>
                                                       ///<      :    SM_TC_CROSSING   - curve intersection (surf norms not parallel)                    <br>
                                                       ///<      :    SM_TC_TANGENT    - curve intersection (surf norms parallel)                        <br>
                                                       ///<      :    SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal) <br>
                                                       ///<      :    SM_TC_NEAR_TANGENT    - curve has small angle of intersection                      <br>
                                                       ///<      :    SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident  <br>
    SmTArray<double>    * pOptDeviations               ///< [out]:                                                                                       <br>
  ) const;

  SmStatus IntersectWithPlane
  (
    const SmContext     & crContext,                   ///< [in] : context for new object construction                                                     <br>
    const SmExtent2d    & crUVDomain,                  ///< [in] : intersection limit for this surface                                                     <br>
    const SmPlane       & crOtherPlane,                ///< [in] : target intersection plane                                                               <br>
    const SmExtent2d    & crOtherUVDomain,             ///< [in] : intersection limit for target plane                                                     <br>
    const SmBoolean       bUseSurfaceEdges[2],         ///< [in] : TRUE = find xsect curve start points from boundaryCurve/surface xsects                  <br>
                                                       ///<      : typically these values are TRUE - its a small savings if you know                       <br>
                                                       ///<      : the boundaries of one surface don't intersect the other surface                         <br>
    const SmApproxTol3d * pdOptApproxTol3d,            ///< [in] :                                                                                         <br>
                                                       ///< [in] :                                                                                         <br>
    const double        * pdOptAngTolRad,              ///< [out]: TRUE = special case intersection failed-use general intersection                        <br>
    SmBoolean           & rbNeedsMoreIntersections,    ///< [out]: Intersection 3DCurves, NULL to ignore                                                   <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,                ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                 <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,        ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                        <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,        ///< [out]: oneof for each 3DCurve, NULL to ignore                                                  <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,       ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                 <br>
                                                       ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                    <br>
                                                       ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                        <br>
                                                       ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal) <br>
                                                       ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                           <br>
                                                       ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident       <br>
    SmTArray<double>    * pOptDeviations               ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                              <br>
  ) const;

  static SmBoolean IsNurbSurfacePlane
  (
    const SmContext & crContext,                       ///< [in] : context for new object construction                               <br>
    const SmBSplineSurface * pTestSurface,             ///< [in] : surface to examine                                                <br>
    SmPlane *& rpPlane,                                ///< [out]: new output surface when pTestSurface is a plane, otherwise NULL   <br>
    double dToleranceScale = 1.0                       ///< [in] : Max 3d variation allowed for plane                                <br>
  );

  virtual SmBoolean IsPlanar
  (
    double       dTol3d = SM_EFF_ZERO,
    SmPoint3d  * pOptPlanePoint = NULL,
    SmVector3d * pOptPlaneNormal = NULL,
    double     * pOptActDeviation = NULL
  ) const
  {
    SM_REF1(dTol3d) ; 
    if(pOptPlanePoint) *pOptPlanePoint = m_vPosition.GetOrigin();
    if(pOptPlaneNormal) *pOptPlaneNormal = m_vPosition.GetZAxis();
    if(pOptActDeviation) *pOptActDeviation = 0.0;
    return TRUE;
  }

  virtual SmBoolean IsOnSeam
  (                                                                                                                                
    const SmPoint2d     & crUVPoint,               ///< [in] : UV Point to check                                                           <br>
    const SmTol3d       * pOptTol3d = NULL,        ///< [in] : max 3d deviation allowed, [NULL = SmTol::GetZoneTol3d(this))]               <br>
    SmSurfParamType     * eOptSeamDir = NULL,      ///< [out]: When IsOnSeam, Dir of Seam picked by crUVPoint (Not a Surf classification), <br>
                                                   ///<      : oneof SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER, NULL to ignore           <br>
    SmTArray<SmPoint2d> * pOptCrossSeamUVs = NULL  ///< [out]: when IsOnSeam, 1 or 3 UVpts across seams, NULL to ignore                    <br>
  ) const
  {
    SM_REF2(crUVPoint, pOptTol3d) ; 
    if(eOptSeamDir)      { *eOptSeamDir = SM_SP_NEITHER; }
    if(pOptCrossSeamUVs) { pOptCrossSeamUVs->ReSet(); }
    return FALSE;
  }

  virtual SmBoolean IsSingularity
  (
    const SmPoint2d  & crUVToTest,                   ///< NotUsed: [in] : UVpoint to test                                                                       <br>
    SmSurfParamType  & reSingularDirection,          ///< [out]: SM_SP_U:[Su=0(U varies, V const)] or SM_SP_V:[Sv=0(U const, V Varies)] or SM_SP_BOTH  <br>
    double             dTol3d = SM_EFF_ZERO,         ///< NotUsed: [in] : min dist between distinct 3d points                                                   <br>
    SmBoolean          bPtTestOnly = FALSE           ///< NotUsed: [in] : TRUE = Pt tests only                                                                  <br>
                                                     ///<      : FALSE= Pt and Surf tests                                                              <br>
                                                     ///<      : default:[FALSE] typical behavior except for some debug cases                          <br>
  ) const;

  virtual SmBoolean IsSingularity
  (
    const SmPoint2d           & crUVToTest,           ///< NotUsed: [in] : UVpoint to test                                                              <br>
    ULONG                       lSingularities,       ///< [in] : SM_SS_NONE or one of: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
    const SmTArray<SmPoint3d> * sSrfPolePoints,       ///< [in] : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                      ///<        NonSingular side values set to SmPoint3d::SetUninitialized(),
    SmSurfParamType           & reSingularDirection,  ///< [out]: SM_SP_U:[Su=0(U varies, V const)] or SM_SP_V:[Sv=0(U const, V Varies)] or SM_SP_BOTH  <br>
    double                      dTol3d = SM_EFF_ZERO, ///< NotUsed: [in] : min dist between distinct 3d points                                          <br>
    SmBoolean                   bPtTestOnly = FALSE   ///< NotUsed: [in] : TRUE = Pt tests only                                                         <br>
                                                      ///<      : FALSE= Pt and Surf tests                                                              <br>
                                                      ///<      : default:[FALSE] typical behavior except for some debug cases                          <br>
  ) const;

  virtual SmStatus CoincidenceCheck
  (
    const SmSurface & crSurface,                         ///< [in] :       <br>
    double            d3DTolerance,                      ///< [in] :       <br>
    SmBoolean       & rbAreCoincident,                   ///< [in] :       <br>
    double          & rMaxDistanceBetween,               ///< [in] :       <br>
    SmBoolean         bReciprocalCoincidence,            ///< [in] :       <br>
    SmSurface      *& rpContainingSurface                ///< [in] :       <br>
  ) const;

  // set m_pNurb = degree 1x1, 4 control Point, domain=m_vAnalUVDomain gw_SURFACE
  SmStatus MakeNurb();

  // Get UVPoint of 3DPoint projected to an infinite plane
  SmStatus ProjectPointToUVDomain
  (
    const SmPoint3d & cr3DPoint,
    SmPoint2d & rUVPoint
  ) const;

  virtual SmStatus Reparameterize
  (
    const SmExtent2d & crNewDomain    ///< [in] : new parameter range for BSPlineSurface 
  );

  // Plane is so simple we don't store a m_SwapUV bit - instead
  //    the Nurb definition is modified so that AnalUVDomain == NaturalUVDomain.
  virtual void     ToggleSwapUVBit();

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  // trim plane domain to given TrimDomain
  virtual SmStatus TrimWithDomain( SmExtent2d & crTrimDomain );

  virtual SmBoolean IsBounded() const;  // TRUE=finite (FALSE=infinite) parameter range

  // eff: planes are never closed - always return FALSE
  virtual SmBoolean IsClosed
  (
    const SmExtent2d &,               ///< [in] : not used            <br>
    SmSurfParamType,                  ///< [in] : not used            <br>
    double           * = NULL,        ///< [in] : not used            <br>
    SmContinuityType * pTyp = NULL    ///< [out]: continuity level    <br>
  ) const
  {
    if(pTyp != NULL) *pTyp = SM_CT_DISCONTINUOUS;
    return FALSE;
  }

  // Trim Line in Plane to Plane given domain
  SmStatus TrimPlaneLineToPlaneDomain
  (
    const SmExtent2d & crUVDomain,          ///< [in] : Extent of this plane                                        <br>
    const SmPoint3d  & rStartPt,            ///< [in] : start point of line segment                                 <br>
    const SmPoint3d  & rEndPt,              ///< [in] : end point of line segment                                   <br>
    double             d3dTolerance,        ///< [in] : Max distance line is allowed to extend beyond the UVDomain  <br>
    SmPoint3d        & rStartTrim,          ///< [out]: start point of trimmed line segment                         <br>
    SmPoint3d        & rEndTrim,            ///< [out]: end point of trimmed line segment                           <br>
    SmBoolean        & bFoundInterval       ///< [out]: TRUE = some part of segment is in plane crUVDomain          <br>
                                            ///<      : FALSE= segment is outside of plane crUVDomain               <br>
  ) const;

   // Trim Curve in Plane to Plane given domain
  SmStatus TrimCurveToPlaneDomain
  (
    const SmExtent2d    & crUVDomain,      ///< [in] : Extent of this plane                                     <br>
    const SmCurve       & crCurve,         ///< [in] : Curve to be trimmed                                      <br>
    double                d3dTolerance,    ///< [in] : max distance between curve and plane domain boundaries   <br>
                                           ///<      : to count as an intersection.                             <br>
    SmTArray<SmCurve *> & cCurves          ///< [out]: A Curve for every crCurve.Interval inside plane domain   <br>
  ) const;

  // Make a line, known to be in the target plane, trimmed to the target Plane's UVDomain.
  SmBSplineCurve *MakeNewPlaneLine
  (
    const SmContext  & crContext,            ///< [in] : context for new object construction            <br>
    const SmExtent2d & crPlaneUVDomain,      ///< [in] : plane's UVDomain                               <br>
    const SmPoint3d  & crStartPt,            ///< [in] : Line Start Point                               <br>
    const SmPoint3d  & crEndPt,              ///< [in] : Line End Point                                 <br>
    double             dTol,                 ///< [in] : min 3d distance between distinct points        <br>
    SmBoolean & rbFoundInterval              ///< [out]: TRUE = Returned a Line curve                   <br>
                                             ///<      : FALSE= Returned no Curve or a Degenerate Curve <br>
  ) const;

  // Make an ellipse, known to be in the target plane, trimmed to the target Plane's UVDomain.
  //   Trimming a circle to the plane can result in 0 to 5 different output curves.
  SmStatus MakeNewPlaneEllipse
  (
    const SmContext            & crContext,                ///< [in] : context for new object construction        <br>
    const SmExtent2d           & crPlaneUVDomain,          ///< [in] : plane's UVDomain                           <br>
    const SmPoint3d            & crCircCenter,             ///< [in] : Center of circle                           <br>
    double                       dCircRadiusX,             ///< [in] : radius of circle at X Axis                 <br>
    double                       dCircRadiusY,             ///< [in] : radius of circle at Y Axis                 <br>
    const SmVector3d           & crCircXAxis,              ///< [in] : vector marking start/end of circle         <br>
    const SmVector3d           & crCircYAxis,              ///< [in] : vector marking circle 90 degree point      <br>
    const SmExtent1d           & crCircIvl,                ///< [in] : domain of circle [0 to 360]                <br>
    double                       dTol,                     ///< [in] : min 3d distance between distinct points    <br>
    SmTArray<SmBSplineCurve *> & rCrvs                     ///< [out]: 0 to 5 new Circle Arc curves               <br>
  ) const;

  void SetUVScale( SmVector2d &rUVScale )
  {
    m_vUVScale = rUVScale;
    if(m_pNurb && !m_bNurbIsBorrowed)
    {
      smos_Free( m_pNurb );
    }
    m_pNurb = NULL;
    m_bNurbIsBorrowed = FALSE;
  }

  void SetPosition( const SmAxis2Placement &rPosition )
  {
    m_vPosition = rPosition;
    if(m_pNurb && !m_bNurbIsBorrowed)
    {
      smos_Free( m_pNurb );
    }
    m_pNurb = NULL;
    m_bNurbIsBorrowed = FALSE;
  }

  virtual SmStatus UpdateAnalyticalDomain( const SmExtent2d & crNewNurbsDomain )
  {
    m_vAnalUVDomain = crNewNurbsDomain;
    if(m_pNurb && !m_bNurbIsBorrowed)
    {
      smos_Free( m_pNurb );
    }
    m_pNurb = NULL;
    m_bNurbIsBorrowed = FALSE;
    return SM_SUCCESS;
  }

  // Recreate the analytic definition from the Nurbs definition.
  virtual SmStatus RebuildSTEPFromNURBParameters();

  // get memory used for curve but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,          ///< [out]: bigger size of all allocated memory in bytes     <br>
    SmMarkType eMarkType = SM_MT_NOMARK    ///< [in] : uses without increment eMarkType value           <br>
  ) const;

  static  SmDisplayList * Draw( const SmPoint3d &crPlanePoint, const SmVector3d &crPlaneNormal )
  {
    return(crPlaneNormal.DrawPlane( crPlanePoint ));
  }

  virtual SmDisplayList * Draw
  ( 
    SmBoolean bAddToUIPickList = FALSE,
    SmGfxArraySet * pOptGfxSet = NULL
  ) const 
  { return(SmSurface::Draw( bAddToUIPickList, pOptGfxSet )); }

  virtual void Dump( const TCHAR * message ) const;
  virtual void Dump( ULONG ) const;

  virtual SmBoolean AssertValid
  (                                                                                                                                           
    SmAssertArray    * pAList = NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
    SmAssertTestLevel  eTestLevel = SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
                                                ///<      : default:[SM_LEVEL_0]                                                                              <br>
    SmAssertWalking    eWalkTree = SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
    SmTArray<ULONG>  * pTestRequests = NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
  ) const;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                         ///< [in] : target output stream                                                               <br>
    ULONG          lDBVersionNumber             ///< [in] : database version to get proper sequence of writes                                  <br>
  ) const;

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,                     ///< NotUsed: [in] : Object type to be read                                                            <br>
    SmDatabaseIO    & rDB,                       ///< [in] : target output stream                                                              <br>
    const SmContext & crContext,                 ///< [in] : context for new object construction                                               <br>
    SmSurface      *& rpNewSurface,              ///< [out]: NULL on input = new object allocated in this routine built from stream data       <br>
                                                 ///<      : NotNULL on input = pointer to an empty object to be filled by this routine        <br>
    ULONG             lDBVersionNumber           ///< [in] : database version to get proper sequence of writes                                 <br>
  );

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON( SmPlane, SmBSplineSurface, SmPlane_TYPE );

}; // end class SmPlane

#endif // !__SMPLANE_H__
