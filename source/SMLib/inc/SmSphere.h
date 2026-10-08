// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSphere.h
* PURPOSE: Header file for Sphere Surface class.
**********************************************************************/

#ifndef __SMSPHERE_H__
#define __SMSPHERE_H__

#ifndef __SMSURFOFREVOLUTION_H__
#include <SmSurfOfRevolution.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

/*******************************************************************//**
PURPOSE: Represent a Sphere surface made from a SmSurfOfRevolution surface.

    The STEP equation of the sphere is

    S(U,V) =   origin                             
             + radius * cos(V) * cos(U) * XAxis   
             + radius * cos(V) * sin(U) * YAxis
             + radius * sin(V)          * ZAxis
       with U rangeDeg:[-360 to 360] maxLength = 360
            V rangeDeg:[ -90 to 90 ]
    
    U parameter = CCW rotation around the sphere's Z axis from the XAxis in degrees (0.0 - 360.0). 
    V parameter = up and down the semiCircular GenCurve between Sphere's poles.

    V minimum and maximum curves are degenerate and on the poles.
    Parameterization is in degrees.

NOTES:

 The STEP parameterization used for the sphere is different than
 the Nurb parameterization used for the underlying NURB surface.
 The map from one to the other is not linear. 

  Translating between STEP and NURB parameters:
    m_bSwapUV    is set when the underlying NURB V param
                 maps to the rotation direction.
    m_bInsideout is set when the underlying GenCurve Nurb direction
                 runs from the +90 Pole to the -90 Pole.

    1. Rotation direction - SmSurfOfRevolution::m_vPolarConverter
        The ZAxis is always set so that the sphere sweeps are made
        CCW about Z
    2. Sphere genCurves are SmCircles and have 3 parameter spaces:
         1. StepParams for a 'STEPGenCurve' are in degrees from [-90 to +90]
         2. PolarParams for a 'PolarCurve' are in degrees either 
                   from [-90 to +90] or [+90 to -90] depending on m_bInsideOut
         3. Nurb params are determined by the genCurve's knot vector but
            always run in the same general direction as the PpolarParams
       To Convert GenCurve STEPToNurb:  StepParam -> PolarParam -> NurbParam
       To Convert GenCurve NurbToStep:  NurbParam -> PolarParam -> StepParam
    3. m_bSwapUV = TRUE 
       NurbU = STEPV and
       NurbV = STEPU

  NOTES: m_bSwapUV and m_bInsideOut are not used to make STEP evaluations

         STEP/Nurb Evaluators when NurbUV = Convert(StepUV)
           EvaluatePoint(NurbUV) == EvaluateSTEPPoint(StepUV) 
           However Tangents will have different magnitudes and
                            are swapped when m_bSwapUV is true
                            has a negated GenCurve tangent when m_bInsideOut is true.
                   Normals will be negated when
                            (m_bSwapUV XOR m_bInsideOut).

   +----------------------------+-------------------+----------------------------+
   | Evaluator                  |       Input       |  OUTPUT                    |
   +----------------------------+-------------------+----------------------------+
   | EvaluatePoint()            | NURB parameter    | Euclidean Point            |
   | Evaluate()                 | NURB parameter    | Euclidean Point and derivs |
   | EvaluateSTEPPoint()        | STEP parameter    | Euclidean Point            |
   | EvaluateSTEP()             | STEP parameter    | Euclidean Point and derivs |
   | ConvertUVFromSTEPToNURBS() | STEP parameter    | NURB parameter             |
   | ConvertUVFromNURBSToSTEP() | NURB parameter    | STEP parameter             |
   | STEPInversion()            | Euclidean Point   | STEP parameter             |
   | DropPointFast()            | Euclidean Point   | NURB parameter             |
   | GlobalPointSolve()         |                   | NURB parameter             |
   | LocalPointSolve()          | NURB parameter    | NURB parameter             |
   +----------------------------+-------------------+----------------------------+

***********************************************************************/
class SM_EXPORT SmSphere : public SmSurfOfRevolution
{
private:
  // inherited
// Remove Composites
//   // SmSurface::m_pOwner                 - NULL or ptr to SmFace or SmCFace
  // SmSurface::m_pOwner                 - NULL or ptr to SmFace
  // SmBSplineSurface::m_pNurb           - ptr to BSplineSurface controlPoints and knots
  //                                       The natural UV Domain of the Nurb is not related to 
  //                                       the AnalUVDomain.  When constructed by CreateCanonical()
  //                                         it has the range:[0_to_1, 0_to_1]
  // SmSurfOfRevolution::m_vPosition     - m_vOrigin = Sphere Center Point
  //                                       m_vZAxis  = axis of revolution
  //                                       m_vXAxis  = 0/360 vector of angular domain
  // SmSurfOfRevolution::m_pGenCurve;    - 3D generating curve, an SmCircle    
  // SmSurfOfRevolution::m_vAnalUVDomain - x = angular domain in degs measured CCW from x [-360 to 360] maxLength=360
  //                                       y = angle from bottom pole to top pole [-90 to 90]              
  //                                           but secretly stored as [0 to 180]
  //                                           and converted when returned by GetSTEPUVDomain(). 
  // SmSurfOfRevolution::m_bSwapUV       - TRUE: NurbU = STEPV,  FALSE: NurbU = STEPU
  //                                             NurbV = STEPU          NurbV = STEPV

  double     m_dRadius = SM_UNDEF_DOUBLE ;  // Radius of sphere
  SmBoolean  m_bInsideOut = FALSE ;         // FALSE if normal does not point outward.

  // empty constructor for I/O
  SmSphere() {}

public:
  // Construction - most users use CreateCanonical()  
  SmSphere
  (
    const SmPoint3d  & crOrigin,                 ///< [in] : Sphere origin                                                                    <br>
    const SmVector3d & crXAxis,                  ///< [in] : start direction of rotational angle [0 to 360]                                   <br>
    const SmVector3d & crYAxis,                  ///< [in] : defines Origin Z as XAxis cross YAxis                                            <br>
    const SmExtent2d & crAnalUVDomain,           ///< [in] : range or allowed parameters                                                      <br>
                                                 ///<      : input as [-360 to 360 MaxLength=360, -90 to 90]                                  <br>
    double             dRadius,                  ///< [in] : distance from sphere origin to its surface                                       <br>
    SmBoolean          bSwapUV = FALSE,          ///< [in] : TRUE = Underlying n_pNurb u and v directions are swapped from AnalUVDomain       <br>
                                                 ///<      : FALSE= Underlying n_pNurb u and v direction are same as AnalUVDomain directions  <br>
    SmBoolean          bInsideOut = FALSE,       ///< [in] : TRUE = GenCurve runs from +90 pole to -90 pole                                   <br>
                                                 ///<      : FALSE= GenCurve runs from -90 pole to +90 pole                                   <br>
    SmBSplineCurve   * pOptCircleNurb = NULL,    ///< [in] : Optional Curve Pointer to define GenCurve parameterization                       <br>
                                                 ///<      :  SurfOfRevolution owns this curve and will delete it when destructed.            <br>
    const SmContext  * cpContext = NULL          ///< [in] : Set context if given, default:[NULL]                                             <br>
  );                                                                                                                        

  // copy constructor
  SmSphere( const SmSphere & crSourceSurface );

  // destructor
  virtual ~SmSphere() {}

  // equality operator
  virtual SmBoolean operator==( const SmSurface &crOther ) const;

  virtual SmStatus Copy
  (
    const SmContext & crContext,
    SmSurface *& rpNewSurface
  ) const;

  virtual SmStatus RebuildSTEPFromNURBParameters();

  // also SmBSplineSurface::AreSTEPAndNURBCurrent() const ;
  //      SmBSplineSurface::RebuildNURBFromSTEPParameters() ;

  // create a step parameterized sphere surface as                         
  // U param = CCW from XAxis about ZAxis in degrees   : range:[ 0  to 360]
  // V param = angle from bottom pole to top in degrees: range:[-90 to  90]
  static SmStatus CreateCanonical
  (
    const SmContext        & crContext,          ///< [in] : new object context                                   <br>
    const SmAxis2Placement & crOrigin,           ///< [in] : sphere origin coordinate system                      <br>
    double                   dRadius,            ///< [in] : distance from originPoint to sphere surface          <br>
    SmSphere              *& rpNewSphere         ///< [out]: new object                                           <br>
  );

  // create offset surface - Out Surf->Domain(s) will equal input surface domain
  virtual SmStatus CreateOffsetSurface
  (
    const SmContext      & crContext,               ///< [in] : context for new obj construction
    double                 dSignedOffsetDistance,   ///< [in] : offset dist, (neg val = Offset dir opposite surface normal)                    <br>
    SmApproxTol3d          sApproxTol3d,            ///< [in] : Max Dist between ApproxOffsetSurface and ideal offset shape                    <br>
    SmSurface* &           rOffsetSurface           ///< [out]: Offset Surf Approx, may have self-intersections                                <br>
  ) const;

  // create IsoParamCurve from underlying Nurb Surface definition - use analytic params to return as SmCircle
  virtual SmStatus CreateIsoParametricCurve
  (
    const SmContext  & crContext,                  ///< [in] : context for created objects                                                                    <br>
    SmSurfParamType    eSurfParam,                 ///< [in] : Defines which Nurb parameter direction on surface to extract curve from                        <br>
                                                   ///<      : SM_SP_U = create constant u isoParameter curve                                                 <br>
                                                   ///<      : SM_SP_V = create constant v isoParameter curve                                                 <br>
    double             dIsoParameter,              ///< [in] : Defines Nurb parametric value at which to extract the curve.                                   <br>
                                                   ///<      : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                           <br>
                                                   ///<      : then this is the V parameter                                                                   <br>
    SmApproxTol3d      sApproxTol3d,               ///< NotUsed: [in] : passed to ApproximateCurve() when approximation is required.                                   <br>
                                                   ///<      : If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]    <br>
    SmBSplineCurve  *& rpNewIsoCurve,              ///< [out]: 3d IsoParameterCurve                                                                           <br>
    const SmExtent2d * pOptDomain = NULL,          ///< [in] : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]                      <br>
    double           * pOptMaxGap3d = NULL,        ///< [out]: opt achieved max gap, NULL to ignore, default:[NULL]                                           <br>
    SmCurve         ** pOptUVIsoCurve = NULL       ///< [out]: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]                <br>
  ) const;

  //          // create sweep circle from analytic parameters and given point
  //          virtual SmStatus CreateDirectrixFromPoint(const SmContext & crContext,
  //                                                    const SmPoint3d & rPointArg,
  //                                                    SmBSplineCurve *& rpDirectrixCurveArg) const;
  //      
  //          // create PoleToPole circle from m_pGenCurve when possible - else from analytic definition
  //          virtual SmStatus CreateGeneratorFromAngle(const SmContext & crContext,
  //                                                    double            dAngleDegArg,
  //                                                    SmBSplineCurve *& rpGeneratorCurveArg) const;

  virtual SmStatus ConvertUVFromSTEPToNURBS
  (
    const SmPoint2d & crSTEPUV,           ///< [in] : Target 2d Point in [degrees, height]     <br>
    SmPoint2d       & rNURBSUV            ///< [out]: 2d Point in SMLib parameterization      <br>
  ) const;

  virtual SmStatus ConvertUVFromNURBSToSTEP
  (
    const SmPoint2d & crNURBSUV,         ///< [in] : Target 2d Point in SMLib parameterization       <br>
    SmPoint2d & rSTEPUV                  ///< [out]: 2d Point in [degrees, height]                    <br>
  ) const;

  // Evaluate the point and triangular derivatives through order 3 at the given STEP UV point.
  // Angular parameters are in degrees; derivatives are per degree. See SmSTEPSurface.h.
  virtual SmStatus EvaluateSTEP
  (
    const       SmPoint2d & crUV,       ///< [in] : U=CCW Rot about Z from X in degrees,          [-360 to 360]  with max length = 360        <br>
                                        ///<      : V=param from bot pole to top pole in degrees, [-90 to 90]                                 <br>
                                        ///<      : when m_bMakeNurbGenCurve = FALSE, V is angle in degrees                                   <br>
                                        ///<      :      m_bMakeNurbGenCurve = TRUE , V is close to an angle in degrees                       <br>
    ULONG       lHighestUDeriv,         ///< [in] : Requested triangular derivative order in U; must equal lHighestVDeriv; max 3               <br>
    ULONG       lHighestVDeriv,         ///< [in] : Requested triangular derivative order in V; must equal lHighestUDeriv; max 3               <br>
    SmBoolean   bUFromLeft,             ///< [in] : not used - if P is on U interval boundary                                                 <br>
                                        ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                        ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean   bVFromLeft,             ///< [in] : not used - if P is on V interval boundary                                                 <br>
                                        ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                        ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
    SmBoolean   bOnlyUpperHalf,         ///< [in] : not used output always set for bOnlyUpperHalf=TRUE                                        <br>
    SmVector3d *aDerivatives            ///< [out]: matrix of evaluations values                                                              <br>
                                        ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                <br>
                                        ///<      : 2d organized: [D    Du   ]                                                                <br>
                                        ///<      :               [Dv   -    ]                                                                <br>
                                        ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]  <br>
  ) const;

  // Get 3dPoint for given UVPoint [0_to_360, -90_to_90]
  virtual SmStatus EvaluateSTEPPoint( const SmPoint2d & crUV, SmPoint3d & rPoint ) const;

  // Evaluate unit surface normal (not STEP parameters)
  virtual SmStatus EvaluateNormal
  (
    const SmPoint2d & crUV,       ///< [in] : uv parameter value                  <br>
    SmBoolean,                    ///< [in] : not used in this virtual method     <br>
    SmBoolean,                    ///< [in] : not used in this virtual method     <br>
    SmVector3d & rSurfaceNormal   ///< [out]: unit-normal                         <br>
  )  const;


  // return stepDomain [0 to 360, -90 to 90]  
  virtual SmExtent2d GetSTEPUVDomain() const;

  virtual SmStatus GlobalPointSolveSTEP
  (
    const SmExtent2d        & crSTEPUVDomain,         ///< [in] : domain within [-360_to_360 max length 360, -90_to_90 ]                          <br>
    SmSolverOperationType     eSolverOperation,       ///< [in] : oneof SM_SO_MINIMIZE,                                                           <br>
                                                      ///<      :   SM_SO_MAXIMIZE,                                                               <br>
                                                      ///<      :   SM_SO_NORMALIZE,                                                              <br>
                                                      ///<      :   SM_SO_INTERSECT                                                               <br>
    const SmPoint3d         & crTestPoint,            ///< [in] : Target Point                                                                    <br>
    double                    dDistanceTolerance,     ///< [in] : for SM_SO_MINIMIZE  - only used with cpdOptTargetDistance                       <br>
                                                      ///<      :  SM_SO_MAXIMIZE  - only used with cpdOptTargetDistance                          <br>
                                                      ///<      :  SM_SO_NORMALIZE - not used                                                     <br>
                                                      ///<      :  SM_SO_INTERSECT - only return answers less than this                           <br>
    const double            * cpdOptTargetDistance,   ///< [in] : for SM_SO_MINIMIZE  - only return answers less than this + distanceTolerance    <br>
                                                      ///<      : SM_SO_MAXIMIZE  - only return answers greater than this - distanceTolerance     <br>
                                                      ///<      : SM_SO_NORMALIZE - not used                                                      <br>
                                                      ///<      : SM_SO_INTERSECT - not used                                                      <br>
    SmSolutionRequestedType   eSolutionRequested,     ///< [in] : oneof SM_SR_ALL, SM_SR_SINGLE                                                   <br>
    SmSolutionArray         & rSolutions              ///< [out]:                                                                                 <br>
  );


  virtual SmBoolean GetInsideOut()                  const { return m_bInsideOut; }
  double            GetRadius()                     const { return m_dRadius; }
  double            GetStartAngleDegFromNorthPole() const { return m_vAnalUVDomain.GetMin().y; }
  double            GetEndAngleDegFromNorthPole()   const { return m_vAnalUVDomain.GetMax().y; }

  SmStatus          GetCanonical
  (
    SmAxis2Placement & rOrigin,                     ///< [out]: Sphere coordinate system                                     <br>
    double           & rdRadius,                    ///< [out]: distance from originPoint to Surface                         <br>
    SmBoolean        * pOptSwapUV = NULL,           ///< [out]: TRUE = Nurb and Analytic U and V directions are swapped      <br>
                                                    ///<      : NULL to ignore, default:[NULL]                               <br>
    SmBoolean        * pOptInsideOut = NULL         ///< [out]: TRUE = Nurb and Analytic genCurve directions are swapped     <br>
                                                    ///<      : NULL to ignore, default:[NULL]                               <br>
  ) const;

  virtual SmExtent2d GetMaxAnalyticDomain() const { SmExtent2d sDom( 0, -90, 360, 90 ); return sDom; }

  virtual SmStatus GlobalSurfaceIntersect
  (
    const SmContext            & crContext,                     ///< [in] : Context for the creation of curves                                                       <br>
    const SmExtent2d           & crUVDomain,                    ///< [in] : in range:[-360_to_360 max length 360, -90_to_90]                                         <br>
    const SmSurface            & crOtherSurface,                ///< [in] : target 2nd intersecting surface                                                          <br>
    const SmExtent2d           & crOtherUVDomain,               ///< [in] : other surface domain                                                                     <br>
    const SmBoolean              bUseSurfaceEdges[2],           ///< [in] : Normally both are TRUE unless you know                                                   <br>
                                                                ///<      : that the edges of one surface do not intersect                                           <br>
                                                                ///<      : the other surface.  It is a slight optimization                                          <br>
                                                                ///<      : to set the flag to FALSE                                                                 <br>
    const SmApproxTol3d        * pdOptApproxTol3d,              ///< [in] : If not given it uses 1/1000 of surface size                                              <br>
                                                                ///<      : approximation tolerance                                                                  <br>
    const double               * pdOptAngTolRad,                ///< [in] : If not given it uses 30 degrees                                                          <br>
    SmTArray<SmCurve*>         * pOpt3DCurves,                  ///< [out]: 3D curves produced by intersection                                                       <br>
    SmTArray<SmCurve*>         * pOptSurface1UVCurves,          ///< [out]: UV curves on this surface produced by intersection                                       <br>
    SmTArray<SmCurve*>         * pOptSurface2UVCurves,          ///< [out]: UV curves on crOtherSurface produced by intersection                                     <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,                ///< [out]: What type of curve is produced -                                                         <br>
    SmTArray<double>           * pOptDeviations                 ///<      : oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)                <br>
                                                                ///<      :        SM_TC_CROSSING   - curve intersection (surf norms not parallel)                   <br>
                                                                ///<      :        SM_TC_TANGENT    - curve intersection (surf norms parallel)                       <br>
                                                                ///<      :        SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)<br>
                                                                ///<      :        SM_TC_NEAR_TANGENT    - curve has small angle of intersection                     <br>
                                                                ///<      :        SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident <br>
                                                                ///< [out]:                                                                                          <br>
  ) const;

  SmStatus IntersectWithPlane
  (                                                                                    
    const SmContext            & crContext,                     ///< [in] : context for new object construction                                                      <br>
    const SmExtent2d           & crUVDomain,                    ///< [in] : in range [-360_to_360 max length 360, -90_to_90]                                         <br>
    const SmPlane              & crPlane,                       ///< [in] : target intersection plane                                                                <br>
    const SmExtent2d           & crOtherUVDomain,               ///< [in] : intersection limit for target plane                                                      <br>
    const SmBoolean              bUseSurfaceEdges[2],           ///< [in] : TRUE = find xsect curve start points from boundaryCurve/surface xsects                   <br>
                                                                ///<      : typically these values are TRUE - its a small savings if you know                        <br>
                                                                ///<      : the boundaries of one surface don't intersect the other surface                          <br>
    const SmApproxTol3d        * pdOptApproxTol3d,              ///< [in] : max distance between coincident points                                                   <br>
    const double               * pdOptAngTolRad,                ///< [in] :                                                                                          <br>
    SmBoolean                  & rbNeedsMoreIntersections,      ///< [out]: TRUE = special case intersection failed-use general intersection                         <br>
    SmTArray<SmCurve*>         * pOpt3DCurves,                  ///< [out]: Intersection 3DCurves, NULL to ignore                                                    <br>
    SmTArray<SmCurve*>         * pOptSurface1UVCurves,          ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                  <br>
    SmTArray<SmCurve*>         * pOptSurface2UVCurves,          ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                         <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,                ///< [out]: oneof for each 3DCurve, NULL to ignore                                                   <br>
                                                                ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                  <br>
                                                                ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                     <br>
                                                                ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                         <br>
                                                                ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)  <br>
                                                                ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                            <br>
                                                                ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident        <br>
    SmTArray<double>           * pOptDeviations                 ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                               <br>
  ) const;

  SmStatus IntersectWithCone
  (                                                              
    const SmContext            & crContext,                     ///< [in] : context for new object construction                                                      <br>
    const SmExtent2d           & crUVDomain,                    ///< [in] : in range [-360_to_360 max length 360, -90_to_90]                                         <br>
    const SmCone               & crCone,                        ///< [in] :                                                                                          <br>
    const SmExtent2d           & crOtherUVDomain,               ///< [in] : intersection limit for target plane                                                      <br>
    const SmBoolean              bUseSurfaceEdges[2],           ///< [in] : TRUE = find xsect curve start points from boundaryCurve/surface xsects                   <br>
                                                                ///<      : typically these values are TRUE - its a small savings if you know                        <br>
                                                                ///<      : the boundaries of one surface don't intersect the other surface                          <br>
    const SmApproxTol3d        * pdOptApproxTol3d,              ///< [in] : max distance between coincident points                                                   <br>
    const double               * pdOptAngTolRad,                ///< [in] :                                                                                          <br>
    SmBoolean                  & rbNeedsMoreIntersections,      ///< [out]: TRUE = special case intersection failed-use general intersection                         <br>
    SmTArray<SmCurve*>         * pOpt3DCurves,                  ///< [out]: Intersection 3DCurves, NULL to ignore                                                    <br>
    SmTArray<SmCurve*>         * pOptSurface1UVCurves,          ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                  <br>
    SmTArray<SmCurve*>         * pOptSurface2UVCurves,          ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                         <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,                ///< [out]: oneof for each 3DCurve, NULL to ignore                                                   <br>
                                                                ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                  <br>
                                                                ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                     <br>
                                                                ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                         <br>
                                                                ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)  <br>
                                                                ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                            <br>
                                                                ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident        <br>
    SmTArray<double>           * pOptDeviations                 ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                               <br>
  ) const;

  SmStatus IntersectWithSphere
  (
    const SmContext            & crContext,                     ///< [in] : context for new object construction                                                      <br>
    const SmExtent2d           & crUVDomain,                    ///< [in] : intersection limit for this surface                                                      <br>
    const SmSphere             & crSphere,                      ///< [in] : other target sphere                                                                      <br>
    const SmExtent2d           & crOtherUVDomain,               ///< [in] : intersection limit for target plane                                                      <br>
    const SmBoolean              bUseSurfaceEdges[2],           ///< [in] : TRUE = find xsect curve start points from boundaryCurve/surface xsects                   <br>
                                                                ///<      : typically these values are TRUE - its a small savings if you know                        <br>
                                                                ///<      : the boundaries of one surface don't intersect the other surface                          <br>
    const SmApproxTol3d        * pdOptApproxTol3d,              ///< [in] :                                                                                          <br>
    const double               * pdOptAngTolRad,                ///< [in] :                                                                                          <br>
    SmBoolean                  & rbNeedsMoreIntersections,      ///< [out]: TRUE = special case intersection failed-use general intersection                         <br>
    SmTArray<SmCurve*>         * pOpt3DCurves,                  ///< [out]: Intersection 3DCurves, NULL to ignore                                                    <br>
    SmTArray<SmCurve*>         * pOptSurface1UVCurves,          ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                  <br>
    SmTArray<SmCurve*>         * pOptSurface2UVCurves,          ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                         <br>
    SmTArray<SmTsectCurveType> * pOptCurveTypes,                ///< [out]: oneof for each 3DCurve, NULL to ignore                                                   <br>
                                                                ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                  <br>
                                                                ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                     <br>
                                                                ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                         <br>
                                                                ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)  <br>
                                                                ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                            <br>
                                                                ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident        <br>
    SmTArray<double>           * pOptDeviations                 ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                               <br>
  ) const;

  // make a SphereCircle trimmed to sphere domain
  SmStatus CreateTrimmedCircleSegments
  (                                                            
    const SmContext     & crContext,                            ///< [in] : new object context                                                                       <br>
    const SmPoint3d     & crCircleCenter,                       ///< [in] : center of circle                                                                         <br>
    const SmVector3d    & crCircleXAxis,                        ///< [in] : xAxis of circle                                                                          <br>
    const SmVector3d    & crCircleYAxis,                        ///< [in] : YAxis of Circle                                                                          <br>
    const SmExtent1d    & crCircleIvl,                          ///< [in] : Circle domain in Degrees (0 = point on xAxis, 90.0 = point on YAxis)                     <br>
    double                dTol3d,                               ///< [in] : max distance between coincident points                                                   <br>
    SmTArray<SmCurve *> & r3dCurves                             ///< [out]: array of trimmed circle arcs                                                             <br>
  ) const;

  // when Nurb is Sphere, rtn TRUE and make SmSphere object with same Nurb parameterization
  static SmBoolean IsNurbSurfaceSphere
  (
    const SmContext        & crContext,                         ///< [in] : new object context                               <br>
    const SmBSplineSurface * pTestSurface,                      ///< [in] : BSplineSurface to be checked                     <br>
    SmSphere              *& rpSphere,                          ///< [out]: new Sphere or NULL                               <br>
    double                   dToleranceScale = 1.0              ///< [in] : extra scale value for ScaledZero computation     <br>
  );

  virtual SmBoolean IsClosed
  (
    const SmExtent2d & crUVDomain,                              ///< [in] : Nurb domain to check                            <br>
    SmSurfParamType    eSurfParam,                              ///< [in] : oneof SM_SP_U    [check Vmin == Vmax]           <br>
                                                                ///<      :       SM_SP_V    [check Umin == Umax]           <br>
                                                                ///<      :       SM_SP_BOTH                                <br>
    double           * pdOptTolerance = NULL,                   ///< NotUsed: [in] : min distance between distinct points            <br>
    SmContinuityType * peOptContinuity = NULL                   ///<      : NULL = scale to 1000*SM_EFF_ZERO*SurfPosition   <br>
                                                                ///< [out]: oneof oneof SM_CT_DISCONTINUOUS                 <br>
                                                                ///<      :              SM_CT_C1_G2                        <br>
                                                                ///<      : NULL to ignore                                  <br>
  ) const;

  SmBoolean IsClosedU() const; // TRUE = all sweep latitude circles are closed
  SmBoolean IsClosedV() const; // TRUE = all genCurve longitude circles run from pole to pole

  // get STEP UV param [-360_to_360,-90_to_90] for Point dropped to Surface, rtn:SM_SUCCESS when Point is orig on Trimmed Surface
  virtual SmStatus STEPInversion
  (
    const SmExtent2d & crAnalUVDomain,                           ///< [in] : domain limit for valid transformations                  <br>
    const SmPoint3d  & crPointOnSurf,                            ///< [in] : Target Point - must be on surface within Tolerance      <br>
    double             d3DTolerance,                             ///< [in] : Max allowed distance between Point and Surface          <br>
    SmPoint2d        & rdAnalUVParameter,                        ///< [out]: STEP UV params for target point                         <br>
                                                                 ///<      : [U = Degrees about rotation Axis,                       <br>
                                                                 ///<      :  V = STEP Param of rotatedPoint on genCurve]            <br>
    SmLocationType   & reLocation,                               ///< [out]: oneof SM_LT_POLE,                                       <br>
                                                                 ///<      :   SM_LT_U_SEAM,                                         <br>
                                                                 ///<      :   SM_LT_V_SEAM,                                         <br>
                                                                 ///<      :   SM_LT_UV_SEAM,                                        <br>
                                                                 ///<      :   SM_LT_INTERIOR                                        <br>
                                                                 ///<      :   SM_LT_EXTERIOR                                        <br>
    SmPoint2d        * pUVGuess = NULL                           ///< [in] : guess parameter.                                        <br>
  ) const;

  SmStatus RotationAboutAxisX
  (                                                              ///< [in] : new object context           <br>
    const SmContext & crContext,                                 ///< [in] : rotation amount              <br>
    double            dAngleDeg,                                 ///< [out]: new Sphere                   <br>
    SmSphere       *& rpNewSphere
  );

  SmStatus RotationAboutAxisY
  (
    const SmContext & crContext,                                ///< [in] : new object context             <br>
    double            dAngleDeg,                                ///< [in] : rotation amount                <br>
    SmSphere       *& rpNewSphere                               ///< [out]: newly allocated sphere         <br>
  );

  SmStatus RotationAboutAxisZ
  (
    const SmContext & crContext,                                ///< [in] : new object context            <br>
    double            dAngleDeg,                                ///< [in] : rotation amount               <br>
    SmSphere       *& rpNewSphere                               ///< [out]: new Sphere                    <br>
  );

  //          virtual SmStatus SwapUV();

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  // utilities

  // get memory used for curve but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,         ///< [out]: bigger size of all allocated memory in bytes        <br>
    SmMarkType eMarkType = SM_MT_NOMARK   ///< [in] : uses without increment eMarkType value              <br>
  ) const;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList = NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
    SmAssertTestLevel  eTestLevel = SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
                                                ///<      :      default:[SM_LEVEL_0]                                                                         <br>
    SmAssertWalking    eWalkTree = SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
    SmTArray<ULONG>  * pTestRequests = NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
  ) const;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                      ///< [in] : target output stream                                <br>
    ULONG          lDBVersionNumber          ///< [in] : database version to get proper sequence of writes   <br>
  ) const;

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in] : Object type to be read                                                        <br>
    SmDatabaseIO    & rDB,                ///< [in] : target output stream                                                          <br>
    const SmContext & crContext,          ///< [in] : context for new object construction                                           <br>
    SmSurface      *& rpNewSurface,       ///< [out]: NULL on input = new object allocated in this routine built from stream data   <br>
                                          ///<      : NotNULL on input = pointer to an empty object to be filled by this routine    <br>
    ULONG             lDBVersionNumber    ///< [in] : database version to get proper sequence of writes                             <br>
  );

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON( SmSphere, SmSurfOfRevolution, SmSphere_TYPE );

}; // end class SmSphere
       
#endif // __SMSPHERE_H__ 
