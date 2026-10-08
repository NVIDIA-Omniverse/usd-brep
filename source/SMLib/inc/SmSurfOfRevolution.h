// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfOfRevolution.h
* PURPOSE: Header file for SurfOfRevolution Surface class.
**********************************************************************/

#ifndef __SMSURFOFREVOLUTION_H__
#define __SMSURFOFREVOLUTION_H__

#ifndef __SMBSPLINESURFACE_H__
#include <SmBSplineSurface.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMELLIPSE_H__
#include <SmEllipse.h>
#endif


/*******************************************************************//**
PURPOSE: This object is a SurfOfRevolution surface

NOTES: For now, we'll create surface-of-revolution only when
    generator curves are coplanar with the axis of revolution.
    The SmSurfOfRevolution is parametrized by the underlying Nurb,
    thus the parametrization is non-uniform. 

    The angular and lateral extents can not be used as parameters,
    only through a geometric param -> 3d location -> inversion 
    (drop) procedure.

  The analytic expression:

    Given UV point = [u,v]  u = angle in degrees, u rangeDeg:[-360 to 360] maxLength = 360
                            v = parameter along generator curve
           P(v)  = GeneratorCurve(v) 
           C(v)  = projection of P(v) onto rotation axis Z
    Surface(u,v) = C(v) 
                   + cos(u) * A
                   + sin(u) * B;
        where 
              A = P-C
                  When GeneratorCurve is planar in (C,X,Z) plane (usual case)
                     A will be in the direction of XAxis and
                     B will be in the direction of YAxis.
              B = Z cross A.

  The NURBs definition may be parameterized the same as the
  STEP definition, or its u/v may be swapped so that U is the
  sweep direction, and V is along the generator curve.
  The member m_bSwapUV indicates this.

  The GenCurve parameterization must match the Surface Domain Intervals as
     1. GenCurve AnalInterval == Surface AnalVInterval
     2. GenCurve NURBInterval == m_bSwapUV ? Surface->NURBUInterval : Surface->NURBVInterval
     3. GenCurve KnotVector   == m_bSwapUV ? Surface->NURBUKnot : Surface->NURBVKnot

  When PolarConverter has a PolarCurve, the PolarCurve parameterization must match the Surface Domain Intervals as
     1. PolarCurve AnalInterval == Surface AnalUInterval
     2. PolarCurve NURBInterval == m_bSwapUV ? Surface->NURBVInterval : Surface->NURBUInterval
     3. PolarCurve KnotVector   == m_bSwapUV ? Surface->NURBVKnot : Surface->NURBUKnot

***********************************************************************/
class SM_EXPORT SmSurfOfRevolution : public SmBSplineSurface
{
protected:
  SmBSplineCurve   * m_pGenCurve = NULL ;            // 3D generating curve. 
                                                     //  Its owner is this SmSurfOfRevolution surface
  SmBoolean          m_bMakeNurbGenCurve = FALSE ;   // Used for unbounded analytic shapes
                                                     //  FALSE = unbounded analytic - don't make an equivalent NURB surface
                                                     //  TRUE  = bounded   analytic - do make equivalent NURB Surface
  SmBoolean          m_bPlanarGenerator = FALSE ;    // TRUE = generator curve is planar and on X/Z plane
                                                     // FALSE= All other cases
  SmAxis2Placement   m_vPosition ;                   // The Origin is a point on the revolution axis,
                                                     // The Z axis is directon of the revolution axis,
                                                     // the X axis is used as the '0 degree' reference for the angular domain
  SmExtent2d         m_vAnalUVDomain ;               // angles from X axis, in deg, CCW
                                                     // x is the angular Deg dimension,
                                                     // y is parameter domain of the generator curve, and not the 
                                                     //   longitudinal metric dimension
  SmBoolean          m_bSwapUV = FALSE ;             // tells if the underlying Nurb's Uparam
                                                     // coincides  with the angular dimension 
                                                     // of the SmSurfOfRevolution. It is set
                                                     // by IsNurbSurfaceSurfOfRevolution. 
  SmPolarConversion  m_vPolarConverter ;             // A cached table used to convert sweep directon 
                                                     // polar coordinates to/from the NURBS parameters.
                                                     // Like all caches - kept up to date with the Notify mechanism.
  // constructor
  SmSurfOfRevolution
  (
    SmBSplineCurve * pGenCurve,           ///< [in ]: New object owns this pointer      <br>
    const SmPoint3d  & crOrigin,          ///< [in ]:                                   <br>
    const SmVector3d & crXAxis,           ///< [in ]:                                   <br>
    const SmVector3d & crYAxis,           ///< [in ]:                                   <br>
    const SmExtent2d & crAnalUVDomain,    ///< [in ]:                                   <br>
    SmBoolean bSwapUV                     ///< [in ]:                                   <br>
  ) ;

  // empty constructor for I/O
  SmSurfOfRevolution() { } ;

  // copy constructor
  SmSurfOfRevolution(const SmSurfOfRevolution & crSurfOfRevolution) ;

  // maintenance
  SmStatus RebuildPolarConverter() ;

public:
  // destructor
  virtual ~SmSurfOfRevolution() ;

  // equality operator
  virtual SmBoolean operator==(const SmSurface &crOther) const;

  // set surf analytic domain and genCurve analytic interval, range:[-360 to 360, maxLength = 360, new GenCurve range]
  virtual SmStatus AdjustSTEPUVDomain(const SmExtent2d & crNewSTEPUVDomain) ;

  virtual SmBoolean AreSTEPAndNURBCurrent() const ;
  virtual SmStatus  RebuildSTEPFromNURBParameters() ;
  virtual SmStatus  MakeNurb();
  SmStatus          MakeNurbWithSweepParams(const SmCircle *pSweepCurve) ;

  virtual SmStatus Copy
  ( 
    const SmContext & crContext,                   ///< [in ]:    <br>
    SmSurface *& rpNewSurface                      ///< [out]:    <br>
  ) const;

  // create a step parameterized surface                          
  static SmStatus CreateCanonical
  (
    const SmContext     & crContext,               ///< [in ]: context for new object construction                                <br>
    SmBSplineCurve      * pSweptCurve,             ///< [in ]: Curve to sweep into surface                                        <br>
    const SmPoint3d     & rAxisPoint,              ///< [in ]: Point on axis of rotation                                          <br>
    const SmVector3d    & rAxisDirection,          ///< [in ]: Direction of axis of rotation                                      <br>
    SmSurfOfRevolution *& rpNewSurfOfRevolution,   ///< [out]: New surface or NULL when input can't be swept                      <br>
    SmBoolean             bSwapUV = FALSE          ///< [in ]: TRUE = underlying NURB v maps to rotation direction                <br>      
                                                   ///<        FALSE= NURB u maps to rotation direction                           <br>

  ) ;

  // create offset surface - Approximate When necessary - Out Surf->Domain(s) may be trimmed but not scaled
  virtual SmStatus CreateOffsetSurface
  (
    const SmContext      & crContext,               ///< [in ]: context for new obj construction                                                <br>
    double                 dSignedOffsetDistance,   ///< [in ]: offset dist, (neg val = Offset dir opposite surface normal)                     <br>
    SmApproxTol3d          sApproxTol3d,            ///< [in ]: Max Dist between ApproxOffsetSurface and ideal offset shape                     <br>
    SmSurface* &           rOffsetSurface           ///< [out]: Offset Surf Approx, may be more than 1 when offsets have self-intersections     <br>
  ) const;

  virtual SmStatus CreateApproxSurface
  (
    const SmContext     & crContext,                 ///< [in ]: context for new object construction                                        <br>
     double                dThisApproxTol3d,         ///< [in ]: max allowed deviation between returned surface and current surface         <br>
     SmSurfOfRevolution *& rpNewSurfOfRevolution     ///< [out]: Approximating Surface                                                      <br>
  ) const ; 


  // build isoParamCurve from underlying NurbSurface and use analytic parameters to return SmCircles when appropriate
  virtual SmStatus CreateIsoParametricCurve
  (                                                                                                                             
    const SmContext  & crContext,                   ///< [in ]: context for created objects                                                                  <br>
    SmSurfParamType    eSurfParam,                  ///< [in ]: Defines which Nurb parameter direction on surface to extract curve from                      <br>
                                                    ///<      : SM_SP_U = create constant u isoParameter curve                                               <br>
                                                    ///<      : SM_SP_V = create constant v isoParameter curve                                               <br>
    double             dIsoParameter,               ///< [in ]: Defines Nurb parametric value at which to extract the curve.                                 <br>
                                                    ///<      : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                         <br>
                                                    ///<      : then this is the V parameter                                                                 <br>
    SmApproxTol3d      sApproxTol3d,                ///< NotUsed: [in ]: passed to ApproximateCurve() when approximation is required.                                 <br>
                                                    ///<      : If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]  <br>
    SmBSplineCurve  *& rpNewIsoCurve,               ///< [out]: 3d IsoParameterCurve                                                                         <br>
    const SmExtent2d * pOptDomain = NULL,           ///< [in ]: optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]                    <br>
    double           * pOptMaxGap3d = NULL,         ///< [out]: opt achieved max gap, NULL to ignore, default:[NULL]                                         <br>
    SmCurve         ** pOptUVIsoCurve = NULL        ///< [out]: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]              <br>
  ) const;

  // create circular SmBsplineCurve from analytic parameters and given point
  SmStatus CreateDirectrixFromPoint
  (
    const SmContext & crContext,                   ///< [in ]: new object context                               <br>
    const SmPoint3d & rPointOnSurf,                ///< [in ]: Point on the SurfOfRevolution                    <br>
    double            d3DTolerance,                ///< NotUsed: [in ]: Not Used                                         <br>
    SmBSplineCurve *& rpDirectrix                  ///< [out]: Sweep Curve interpolating PointOnSurf            <br>
  ) const ;

  // build circular SmBSplineCurve for given generator param value
  SmStatus CreateDirectrixAtGeneratorParam
  (
    const SmContext & crContext,                  ///< [in ]: context for new object construction                  <br>
    double            dGeneratorParam,            ///< [in ]: Param in NurbDomain of GenCurve                      <br>
    SmBSplineCurve *& rpDirectrix,                ///< [out]: Circular Arc Curve for given param                   <br>
    double          & dRadius,                    ///< [out]: Radius of returned Directrix                         <br>
    SmPoint3d       & rAxisLocation               ///< [out]: CenterPoint of returned Directrix                    <br>
  ) const ;

  // copy and rotate genCurve to appropriate point
  SmStatus CreateGeneratorFromAngle
  (
    const SmContext & crContext,                  ///< [in ]: new object context                                          <br>
    double            dAngleDeg,                  ///< [in ]: desired angle in degrees                                    <br>
    SmBSplineCurve *& rpGenerator                 ///< [out]: new curve, same derived type as genCurve                    <br>
  ) const ;

  SmStatus GetCanonical
  (
    SmBSplineCurve *& pSweptCurve,                ///< [out]: Gen Curve                                                     <br>
    SmPoint3d       & rAxisPoint,                 ///< [out]: Center Point of rotation axis                                 <br>
    SmVector3d      & rAxisDirection,             ///< [out]: direction    of rotation axis                                 <br>
    SmBoolean       * pOptSwapUV = NULL           ///< [out]: TRUE = Nurb and Analytic U and V directions are swapped       <br>
                                                  ///<      : NULL to ignore, default:[NULL]                                <br>
  ) const;

  virtual SmBoolean        GetInsideOut()           const { return FALSE ; } // only SmCone, SmSphere, and SmTorus have InsideOut
  SmBoolean                GetSwapUV()              const { return m_bSwapUV ; }
  virtual SmExtent2d       GetSTEPUVDomain()        const { return m_vAnalUVDomain ; }
  double                   GetStartAngleDeg()       const { return (m_vAnalUVDomain.GetMin().x) ; } 
  double                   GetEndAngleDeg()         const { return (m_vAnalUVDomain.GetMax().x) ; }
  double                   GetGeneratorStartParam() const { return (m_vAnalUVDomain.GetMin().y) ; }
  double                   GetGeneratorEndParam()   const { return (m_vAnalUVDomain.GetMax().y) ; }
  virtual SmExtent2d       GetMaxAnalyticDomain()   const;

  SmBSplineCurve         * GetGenCurve()            const { return m_pGenCurve ; }
  SmExtent1d               GetRotateParamExtent (SmSurfParamType * pOptRotSurfParam=NULL)   const ; // rtn RotateDir NURB ivl
  SmExtent1d               GetGenDirParamExtent (SmSurfParamType * pOptGenSurfParam=NULL)   const ; // rtn GenDir NURB ivl

  const SmAxis2Placement & GetPosition()            const { return m_vPosition ; }  
  const SmPoint3d        & GetOrigin()              const { return m_vPosition.GetOriginRef() ; }
  const SmVector3d       & GetXAxis()               const { return m_vPosition.GetXAxisRef () ; }
  const SmVector3d       & GetYAxis()               const { return m_vPosition.GetYAxisRef () ; }
  SmPoint3d                GetZAxis()               const { return m_vPosition.GetZAxis () ; }

  SmBoolean HavePolarConversion() const { return m_vPolarConverter.IsPolarConversionPossible() ; }

  virtual SmStatus ConvertUVFromSTEPToNURBS
  (
    const SmPoint2d & crSTEPUV,                       ///< [in ]:  STEP Domain UV point     <br>
    SmPoint2d & rNURBSUV                              ///< [out]:  Nurb Domain UV point     <br>
  ) const ;

  virtual SmStatus ConvertUVFromNURBSToSTEP
  (
    const SmPoint2d & crNURBSUV,                      ///< [in ]: UV Point in Nurb domain    <br>
    SmPoint2d & rSTEPUV                               ///< [out]: UV Point in STEP domain    <br>
  ) const ;

  virtual SmStatus DropPointFast
  (
    const SmExtent2d       & crUVDomain,              ///< [in ]: NURB Domain of surface to search for solutions                                           <br>
    SmSolverOperationType    eSolverOperation,        ///< [in ]: oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT                  <br>
    const SmPoint3d        & crTestPoint,             ///< [in ]: target point                                                                             <br>
    const SmVector3d       * cpOptInPointingVector,   ///< [in ]: specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams    <br>
    double                   dDistanceTolerance,      ///< [in ]: Used to decide whether result in on a seam (and hence return two solutions)              <br>
    const double           * cpdOptTargetDistance,    ///< [in ]: The pdOptTargetDistance, if not NULL, will be the corresponding                          <br>
                                                      ///<      : limit to a minimize/maximize operations.  In otherwords, it will                         <br>
                                                      ///<      : ask the solver to find a minimum value only if it is less than                           <br>
                                                      ///<      : the target distance or maximum value only if it is greater than                          <br>
                                                      ///<      : the target distance.                                                                     <br>
    SmSolutionRequestedType  eSolutionRequested,      ///< [in ]: SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                              <br>
    SmSolutionArray        & rSolutions               ///< [out]: array of problem solutions reported as surface UV parameter values                       <br>
  ) const ;

  virtual SmStatus EvaluateSTEP
  (
    const SmPoint2d & crUV,                           ///< [in ]: U=CCW Rot about Z from X in degrees,          [0 to 360]                                  <br>
                                                      ///<      : V=param from bot pole to top pole in degrees, [-90 to 90]                                 <br>
                                                      ///<      : when m_bMakeNurbGenCurve = FALSE, V is angle in degrees                                   <br>
                                                      ///<      :      m_bMakeNurbGenCurve = TRUE , V is close to an angle in degrees                       <br>
    ULONG lHighestUDeriv,                             ///< [in ]: Requested triangular derivative order in U; must equal lHighestVDeriv; max 3               <br>
    ULONG lHighestVDeriv,                             ///< [in ]: Requested triangular derivative order in V; must equal lHighestUDeriv; max 3               <br>
    SmBoolean bUFromLeft,                             ///< [in ]: not used - if P is on U interval boundary                                                 <br>
                                                      ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                                      ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean bVFromLeft,                             ///< [in ]: not used - if P is on V interval boundary                                                 <br>
                                                      ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                                      ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean bOnlyUpperHalf,                         ///< [in ]: not used output always set for bOnlyUpperHalf=TRUE                                        <br>                                             
    SmVector3d *aDerivatives                          ///< [out]: matrix of evaluations values                                                              <br>
                                                      ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                <br>
                                                      ///<      : 2d organized: [D    Du   ]                                                                <br>
                                                      ///<      :               [Dv   -    ]                                                                <br>
                                                      ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]  <br>
  ) const;

  virtual SmStatus EvaluateSTEPPoint
  (
    const SmPoint2d & crUV,                               ///< [in ]: crUV.x = angle in degrees CCW       <br>
                                                          ///<      : crUV.y = Gen Curve parameter        <br>
    SmPoint3d & rPoint                                    ///< [out]:                                     <br>
  ) const ;

  virtual SmStatus GlobalPointSolveSTEP
  (
    const SmExtent2d    & crSTEPUVDomain,                 ///< [in ]:        <br>
    SmSolverOperationType eSolverOperation,               ///< [in ]:        <br>
    const SmPoint3d     & crTestPoint,                    ///< [in ]:        <br>
    double                dDistanceTolerance,             ///< [in ]:        <br>
    const double        * cpdOptTargetDistance,           ///< [in ]:        <br>
    SmSolutionRequestedType eSolutionRequested,           ///< [in ]:        <br>
    SmSolutionArray     & rSolutions                      ///< [out]:        <br>
  ) ;        

  virtual SmStatus GlobalSurfaceIntersect
  (
    const SmContext     & crContext,                       ///< [in ]: Context for the creation of curves                                                           <br>
    const SmExtent2d    & crUVDomain,                      ///< [in ]:                                                                                              <br>
    const SmSurface     & crOtherSurface,                  ///< [in ]: target 2nd intersecting surface                                                              <br>
    const SmExtent2d    & crOtherUVDomain,                 ///< [in ]:                                                                                              <br>
    const SmBoolean       bUseSurfaceEdges[2],             ///< [in ]: Normally both are TRUE unless you know that the edges of one surface do not intersect        <br>
                                                           ///<      : the other surface.  It is a slight optimization to set the flag to FALSE                     <br>
    const SmApproxTol3d * pdOptApproxTol3d,                ///< [in ]: approximation tolerance. If not given it uses 1/1000 of surface size                         <br>
    const double        * pdOptAngTolRad,                  ///< [in ]: If not given it uses 30 degrees                                                              <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,                    ///< [out]: 3D curves produced by intersection                                                           <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,            ///< [out]: UV curves on this surface produced by intersection                                           <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,            ///< [out]: UV curves on crOtherSurface produced by intersection                                         <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,           ///< [out]: What type of curve is produced -                                                             <br>
    SmTArray<double>    * pOptDeviations                   ///<      : oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)                    <br>
                                                           ///<      :        SM_TC_CROSSING   - curve intersection (surf norms not parallel)                       <br>
                                                           ///<      :        SM_TC_TANGENT    - curve intersection (surf norms parallel)                           <br>
                                                           ///<      :        SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)    <br>
                                                           ///<      :        SM_TC_NEAR_TANGENT    - curve has small angle of intersection                         <br>
                                                           ///<      :        SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident     <br>
                                  

  ) const ;

  virtual SmStatus IntersectWithPlane
  (
    const SmContext     & crContext,                        ///< [in ]: context for new object construction                                                     <br>
    const SmExtent2d    & crUVDomain,                       ///< NotUsed: [in ]: intersection limit for this surface                                                     <br>
    const SmPlane       & crPlane,                          ///< [in ]: target intersection plane                                                               <br>
    const SmExtent2d    & crPlaneUVDomain,                  ///< [in ]: intersection limit for target plane                                                     <br>
    const SmBoolean       bUseSurfaceEdges[2],              ///< [in ]: TRUE = find xsect curve start points from boundaryCurve/surface xsects                  <br>
                                                            ///<      : typically these values are TRUE - its a small savings if you know                       <br>
                                                            ///<      : the boundaries of one surface don't intersect the other surface                         <br>
    const SmApproxTol3d * pdOptApproxTol3d,                 ///< [in ]:                                                                                         <br>
    const double        * pdOptAngTolRad,                   ///< [in ]:                                                                                         <br>
    SmBoolean           & rbNeedsMoreIntersections,         ///< [out]: TRUE = special case intersection failed-use general intersection                        <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,                     ///< [out]: Intersection 3DCurves, NULL to ignore                                                   <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,             ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                 <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,             ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                        <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,            ///< [out]: oneof for each 3DCurve, NULL to ignore                                                  <br>
    SmTArray<double>    * pOptDeviations                    ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                 <br>
                                                            ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                    <br>
                                                            ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                        <br>
                                                            ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal) <br>
                                                            ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                           <br>
                                                            ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident       <br>
                                                            ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                              <br>
  ) const ;

  virtual SmStatus IntersectWithCone
  (
    const SmContext     & crContext,                        ///< [in ]:      <br>
    const SmExtent2d    & crUVDomain,                       ///< [in ]:      <br>
    const SmCone        & crCone,                           ///< [in ]:      <br>
    const SmExtent2d    & crPlaneUVDomain,                  ///< [in ]:      <br>
    const SmBoolean       bUseSurfaceEdges[2],              ///< [in ]:      <br>
    const SmApproxTol3d * pdOptApproxTol3d,                 ///< [in ]:      <br>
    const double        * pdOptAngTolRad,                   ///< [in ]:      <br>
    SmBoolean           & rbNeedsMoreIntersections,         ///< [in ]:      <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,                     ///< [in ]:      <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,             ///< [in ]:      <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,             ///< [in ]:      <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,            ///< [in ]:      <br>
    SmTArray<double>    * pOptDeviations                    ///< [in ]:      <br>
  ) const ;

  virtual SmStatus IntersectWithSphere
  (
    const SmContext     & crContext,                        ///< [in ]:      <br>
    const SmExtent2d    & crUVDomain,                       ///< [in ]:      <br>
    const SmSphere      & crSphere,                         ///< [in ]:      <br>
    const SmExtent2d    & crPlaneUVDomain,                  ///< [in ]:      <br>
    const SmBoolean       bUseSurfaceEdges[2],              ///< [in ]:      <br>
    const SmApproxTol3d * pdOptApproxTol3d,                 ///< [in ]:      <br>
    const double        * pdOptAngTolRad,                   ///< [in ]:      <br>
    SmBoolean           & rbNeedsMoreIntersections,         ///< [in ]:      <br>
    SmTArray<SmCurve*>  * pOpt3DCurves,                     ///< [in ]:      <br>
    SmTArray<SmCurve*>  * pOptSurface1UVCurves,             ///< [in ]:      <br>
    SmTArray<SmCurve*>  * pOptSurface2UVCurves,             ///< [in ]:      <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,            ///< [in ]:      <br>
    SmTArray<double>    * pOptDeviations                    ///< [in ]:      <br>
  ) const ;
    
  virtual SmStatus IntersectWithSurfOfRevolution
  (
    const SmContext          & crContext,                    ///< [in ]: new object context                                      <br>
    const SmExtent2d         & crUVDomain,                   ///< [in ]: ThisSurface NurbDomain limit                            <br>
    const SmSurfOfRevolution & crSurfOfRevolution,           ///< [in ]: OtherSurface to intersect                               <br>
    const SmExtent2d         & crPlaneUVDomain,              ///< [in ]: OtherSurface Nurbdomain limit                           <br>
    const SmBoolean            bUseSurfaceEdges[2],          ///< [in ]:  bUseSurfaceEdges - not used                            <br>
    const SmApproxTol3d      * pdOptApproxTol3d,             ///< [in ]: max distance between 3d points                          <br>
    const double             * pdOptAngTolRad,               ///< NotUsed: [in ]: angle tolerance                                         <br>
    SmBoolean                & rbNeedsMoreIntersections,     ///< [out]: TRUE = call general intersector                         <br>
    SmTArray<SmCurve*>       * pOpt3DCurves,                 ///< [out]: new intersection curves                                 <br>
    SmTArray<SmCurve*>       * pOptSurface1UVCurves,         ///< [out]: associated ThisSurface  UVTrimCurves                    <br>
    SmTArray<SmCurve*>       * pOptSurface2UVCurves,         ///< [out]: associated OtherSurface UVTrimCurves                    <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,             ///< [out]: associated IntersectionCurve classifications            <br>
    SmTArray<double>         * pOptDeviations                ///< [out]: Max distance between surfaces near exact intersections  <br>
  ) const ;

  virtual SmBoolean IsBounded() const ; // TRUE=finite (FALSE=infinite) parameter range
    
  static SmBoolean IsNurbSurfaceSurfOfRevolution
  (
    const SmContext & crContext,                   ///< [in ]: new object construction                                                                          <br>
    const SmBSplineSurface * pTestSurface,         ///< [in ]: surface to examine                                                                               <br>
    SmSurfOfRevolution *& rpSurfOfRevolution,      ///< [out]: new SurfOfRev when TestSurface can be one                                                        <br>
    double dToleranceScale = 1.0                   ///< [in ]: max allowed variation from circular sweeps = SM_EFF_ZERO * dToleranceScale * ANALYTIC_TOL_SCALE  <br>
  ) ;

  SmStatus RotationAboutAxisZ
  (
    const SmContext & crContext,                   ///< [in ]:         <br>
    double dAngleDeg,                              ///< [in ]:         <br>
    SmSurfOfRevolution *& rpNewSurf                ///< [in ]:         <br>
  );

  void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                 m_pGenCurve->SetContext(cpContext);
                                                 m_vPolarConverter.SetContext(cpContext); }

  virtual void SetOutOfBoundsEnabled( SmBoolean bOutOfBoundsEnabled )
    {
      SmBSplineSurface::SetOutOfBoundsEnabled( bOutOfBoundsEnabled );
      m_pGenCurve->SetOutOfBoundsEnabled( bOutOfBoundsEnabled );
    }

  // gwc: can use SmBSplineSurface version:  virtual SmStatus SwapUV();
  virtual void      ToggleSwapUVBit()  { m_bSwapUV = m_bSwapUV ? FALSE : TRUE ; }
  virtual SmBoolean GetSwapUVBit()     { return m_bSwapUV ; }

  virtual SmStatus Reparameterize(const SmExtent2d & crNewDomain);

  virtual SmStatus STEPInversion
  (
    const SmExtent2d & crAnalUVDomain,            ///< [in ]: domain limit for valid transformations                                   <br>
                                                  ///<      : Passed on to SmBSplineSurface::STEPINVERSION for nonPlanar GenCurves     <br>
    const SmPoint3d  & crPointOnSurf,             ///< [in ]: Target Point - must be on surface within Tolerance                       <br>
    double             d3DTolerance,              ///< [in ]: Max allowed distance between Point and Surface                           <br>
                                                  ///<      : Also used to determine whether result is on a seam                       <br>
    SmPoint2d        & rdAnalUVParameter,         ///< [out]: STEP UV params for target point                                          <br>
                                                  ///<      : [U = Degrees about rotation Axis,                                        <br>
                                                  ///<      :  V = STEP Param of rotatedPoint on genCurve]                             <br>
    SmLocationType   & reLocation,                ///< [out]: oneof SM_LT_POLE,                                                        <br>
                                                  ///<      :   SM_LT_U_SEAM,                                                          <br>
                                                  ///<      :   SM_LT_V_SEAM,                                                          <br>
                                                  ///<      :   SM_LT_UV_SEAM,                                                         <br>
                                                  ///<      :   SM_LT_INTERIOR,                                                        <br>
                                                  ///<      :   SM_LT_EXTERIOR                                                         <br>
    SmPoint2d        * pUVGuess = NULL            ///< [in,out] : guess parameter.                                                     <br>
  ) const;

  // Rotate the given point about the Z axis until it is on the Z, X plane.
  SmStatus TransformPointToStartPlane
  (
    const SmPoint3d & crPointToTransform,
    double            dDistTol3d,          ///< [in ]: Dist3d when points are close enough to seams to return 2 answers                       <br>
    SmPoint3d       & rTransformedPoint,   ///< [out]: PointToTransform rotated to GenCurve plane postion                                     <br>
    ULONG           & rlNumAngles,         ///< [out]: 1 - point is not on closed seam                                                        <br>
                                           ///<      : 2 - point is on closed seam boundary or on a pole (a point on the rotation axis)       <br>
    double            adAnglesDeg[2],      ///< [out]: Angles in degrees to rotate rTransformedPoint back to original position                <br>
                                           ///<      : range:[m_vAnalDomain] or positive(0 to 360.0)                                          <br>
    SmBoolean       & bInside,             ///< [out]: TRUE = point is in sweep interval, FALSE = isn't                                       <br>
    SmBoolean         bSnapToSeams=FALSE   ///< [in ]: TRUE = rtn 2 snapped values at seams, FALSE = rtn 1 exact and 1 snapped val at seams   <br>
                                           ///<      : TRUE=previous behavior, default:[FALSE]                                                <br>
  ) const ;

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  virtual SmStatus TrimWithDomain(SmExtent2d & crTrimDomain) ;

  virtual SmStatus UpdateAnalyticalDomain( const SmExtent2d & crNewNurbsDomain ) ;

  // get memory used for curve but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes     <br>
    SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value           <br>
  ) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                             <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                      ///< [in ]: target output stream                                                         <br>
    ULONG          lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes                            <br>
  ) const ;                                                                        

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                            <br>
    SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                              <br>
    const SmContext & crContext,          ///< [in ]: context for new object construction                                               <br>
    SmSurface      *& rpNewSurface,       ///< [out]: NULL on input = new object allocated in this routine built from stream data       <br>
                                          ///<      : NotNULL on input = pointer to an empty object to be filled by this routine        <br>
    ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                                 <br>
  );

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmSurfOfRevolution,SmBSplineSurface,SmSurfOfRevolution_TYPE) ;

} ; // end class SmSurfOfRevolution

/*******************************************************************//**
PURPOSE:  Helper functions to test BSpline Surfaces for
             sequences of circular IsoParameter curve all sharing
             a common reference frame.

NOTES:
***********************************************************************/
SmStatus sm_TestForIsoCircles
(
  const SmBSplineSurface * pTestSurface,    ///< [in ]: Surface to check                                                                                <br>
  SmSurfParamType          eSurfParam,      ///< [in ]: isoParameter direction                                                                          <br>
  double                   dScaledZero,     ///< [in ]: 3d distance tolerance                                                                           <br>
  double                   dAngTol,         ///< [in ]: Max allowed angular deviation for planarGenCurve checks                                         <br>
  SmBoolean             & rbFoundCircles,   ///< [out]: TRUE = all tested isoParameterCurves are circular arcs                                          <br>
  SmBoolean             & rbParallelPlanes, ///< [out]: TRUE = all tested isoParameterCurves are on parallel planes                                     <br>
  SmBoolean             & rbSamePlane,      ///< [out]: TRUE = all tested isoParameterCurves are on the same plane                                      <br>
  SmBoolean             & rbSameCenters,    ///< [out]: TRUE = all tested isoParameterCurves have same centers                                          <br>
  SmBoolean             & rbCommonAxis,     ///< [out]: TRUE = all tested isoParameterCurves lie on plane with 1 single common axis (tori minor curves) <br>
  SmBoolean             & rbPlanarGenCurve, ///< [out]: TRUE = All circular arcs start and stop on same plane                                           <br>
  double                & rdRadius,         ///< [out]: radius of 1st nonDegenerate isoParameter curve tested                                           <br>
  SmExtent1d            & rAngles,          ///< [out]: Start/End angles for 1st nonDegenerate isoParameter curve tested                                <br>
  SmAxis2Placement      & rReferenceFrame,  ///< [out]: ref frame for 1st nonDegenerate isoParameter curve tested                                       <br>
  SmPoint3d             & rCommonLinePoint, ///< [out]: when rbCommonAxis == TRUE - Point  of common isoParam Plane line                                <br>
  SmVector3d            & rCommonLineVec    ///< [out]: when rbCommonAxis == TRUE - UnitVector of common isoParam Plane line                            <br>
);


#endif // __SMSURFOFREVOLUTION_H__

