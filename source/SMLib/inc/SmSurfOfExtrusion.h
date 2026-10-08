// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfOfExtrusion.h
* PURPOSE: Header file for SurfOfExtrusion Surface class.
**********************************************************************/

#ifndef __SMSURFOFEXTRUSION_H__
#define __SMSURFOFEXTRUSION_H__

#ifndef __SMBSPLINESURFACE_H__
#include <SmBSplineSurface.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

/*******************************************************************//**
PURPOSE: enum of SmSurfOfExtrusion GenCurve classifications

USAGE NOTES:
***********************************************************************/
enum SmCurveOrientType 
{
  SM_CO_NOT_PLANAR,      // GenCurve is not planar
  SM_CO_PERPENDICULAR,   // Planar GenCurve and GenCurve plane is perpendicular to Extrusion Vec
  SM_CO_INDEPENDENT,     // Planar GenCurve and GenCurve plane is at an angle to Extrusion Vec
  SM_CO_COPLANAR,        // Planar GenCurve and GenCurve plane is coplanar with Extrusion Vec
  SM_CO_LINEAR,          // Linear GenCurve
  SM_CO_UNDEFINED          // value not yet set
} ;

/*******************************************************************//**
PURPOSE: This object is a SurfOfExtrusion surface.

NOTES:
- The swept curve (generator) should not self-intersect,
  that is the resulting surface is not checked for self-intersections.

As with all analytic subclasses, SurfOfExtrusion supports both
a canonical STEP definition and a Nurbs definition.

  The analytic (STEP) expression:
    Given UV point = [u,v]  a point in the analytic domain m_vAnaUVDomain
    Surface(u,v) = GenCurve(u) + v * ExtrusionVec

  1st Derivatives:
    d(Surface)/du     = d(GenCurve(u))/du
    d(Surface)/dv     = ExtrusionVec

  2nd Derivatives:
    d(Surface)**2/dudu     = d(GenCurve(u))**2/dudu
    d(Surface)**2/dudv     = 0.0
    d(Surface)**2/dvvv     = 0.0

  nonZero 3rd derivative:
    d(Surface)**3/dududu     = d(GenCurve(u))**2/dududu


  The u-parameter is the generator curve's parameter,
  and v is the direction of extrusion.
  The extrusion vector need not be a unit vector; the 'speed' in v
  equals its magnitude.  In practice, most methods of this class
  create surfaces with unitized extrusion vector.

  When the GenCurve is analytic, its analytic domain range must
  equal the Surface analytic domain range as
  GenCurve->GetSTEPInterval    == m_vAnalUVDomain.GetUInterval(), and
  GenCurve->GetNaturalInterval == m_bSwapUV ? GetNaturalDomain().GetVInterval() : GetNaturalDomain().GetUInterval() ;

  The NURBs definition may be parameterized the same as the
  STEP definition, or its u/v may be swapped so that U is the
  extrusion direction, and V is along the generator curve.
  The member m_bSwapUV indicates this.

  SmSurfOfExtrusion parameterization rules
  Rule 1. GenCurve->AnalInterval  == AnalUVDomain.GetUInterval
  Rule 2. GenCurve->NurbInterval  == m_bSwapUV ? sNaturalUVDomain.GetVInterval : sNaturalUVDomain.GetUInterval
  Rule 3. GenCurve NURBKnotVector == m_bSwapUV ? Surface->NURBKnotV : Surface->NURBKnotU
***********************************************************************/
class SM_EXPORT SmSurfOfExtrusion : public SmBSplineSurface
{
protected:
  SmBSplineCurve       * m_pGenCurve = NULL ;             // 3D generating curve, its Interval may be equal to or
                                                          //  or larger than the surfaces equivalent m_vAnalUVDomain range.
                                                          //  Its owner is this SmSurfOfExtrusion surface 
  SmPoint3d              m_vOrigin;                       // Origin for extrusion vector domain, 
                                                          //   Some point on GenCurve.
  SmVector3d             m_vExtrusionVec;                 // sweep direction and length
  SmExtent2d             m_vAnalUVDomain;                 // U: parameter domain of the generator curve.
                                                          // V: parameter domain of extrusion.
                                                          //    Can be anywhere between (-Infinity to Infinity)
  SmCurveOrientType m_eOrientation = SM_CO_UNDEFINED ; // one of: SM_CO_NOT_PLANAR
                                                          //         SM_CO_PERPENDICULAR,
                                                          //         SM_CO_INDEPENDENT,  
                                                          //         SM_CO_COPLANAR,
                                                          //         SM_CO_LINEAR      
  SmBoolean              m_bSwapUV = FALSE ;              // TRUE  : STEP_U = NURB_V, and STEP_V = NURB_U
                                                          // FALSE : STEP_U = NURB_U, and STEP_V = NURB_V 

  SmVector3d             m_vCurvePoint ;             // When GenCurve is Planar or Linear - point on Plane or Line
  SmVector3d             m_vCurveVec ;               // When GenCurve is Planar - Plane Normal, 
                                                     //      GenCurve is Linear - Line Vec

  //   SmBoolean                 m_bPlanarGenerator; // TRUE = generator curve is planar
  //   SmBoolean                 m_bPerpendicular;   // TRUE if the extrusion vector is perpendicular
  //                                                 // to the plane of the generator curve.
  // inherited from SmBSplineSurface               
  //   gw_SURFACE*           m_pNurb;                // ptr to Bspline data of type gw_SURFACE
  //   SmBoolean             m_bNurbIsBorrowed;      // borrowed Nurb pointers are on the stack and
  //                                                 // can't do global operations
  //   SmBoolean             m_bOutOfBoundsEnabled ; // GWC:OUTOFBOUNDS MODIFICATION
  //                                                 // TRUE=RETURN EVALUATIONS FOR UV POINTS BEYOND DOMAIN
  //                                                 // default:[FALSE]
  //   SmKnotType            m_eKnotType;
  //   SmBSplineSurfaceForm  m_eBSplineSurfaceForm;
  //
  // inherited from SmSurface
  //   SmObject            * m_pOwner;               // Pointer to an SmFace object when used by a single Face.    
// Remove Composites
//  //                                                 // Pointer to an SmCFace object when used by a composite Face.
  //                                                 // NULL when not used by a Face.                              

  // empty constructor for I/O
  SmSurfOfExtrusion() { }
                         
public:
  // construct surface = curve extruded a specified distance along ExtrusionVec
  SmSurfOfExtrusion
  (
    SmBSplineCurve * pGenCurve,          ///< [in ]: Curve to extrude                                                     <br>
    const SmPoint3d  & crOrigin,         ///< [in ]:                                                                      <br>
    const SmVector3d & crExtrusionVec,   ///< [in ]: Direction of Extrusion                                               <br>
    const SmExtent2d & crAnalUVDomain,   ///< [in ]: U Range = GenCurve domain,                                           <br>
                                         ///<      : V range = extrusion limits.                                          <br>
    SmBoolean bSwapUV                    ///< [in ]: TRUE  = GenCurve is a constant U iso-Curve in the m_pNurb Surface    <br>
                                         ///<      :         and m_pNurb U Range = GenCurve domain,                       <br>
                                         ///<      :             m_pNurb V range = extrusion limits.                      <br>
                                         ///<      : FALSE = GenCurve is a constant V iso-Curve in the m_pNurb Surface    <br>
                                         ///<      :         and m_pNurb U Range = extrusion limits,                      <br>
                                         ///<      :             m_pNurb V range = GenCurve domain.                       <br>
  );

  // construct surface = curve extruded infinitely along ExtrusionVec
  SmSurfOfExtrusion
  (
    SmBSplineCurve   * pGenCurve,        ///< [in ]: Curve to extrude                                                      <br>
    const SmVector3d & crExtrusionVec,   ///< [in ]: Direction of Extrusion                                                <br>
    SmBoolean          bSwapUV           ///< [in ]: TRUE  = GenCurve is a constant U iso-Curve in the m_pNurb Surface     <br>
                                         ///<      :         and m_pNurb U Range = GenCurve domain,                        <br>
                                         ///<      :             m_pNurb V range = extrusion limits.                       <br>
                                         ///<      : FALSE = GenCurve is a constant V iso-Curve in the m_pNurb Surface     <br>
                                         ///<      :         and m_pNurb U Range = extrusion limits,                       <br>
                                         ///<      :             m_pNurb V range = GenCurve domain.                        <br>
  );

  // copy constructor
  SmSurfOfExtrusion(const SmSurfOfExtrusion & crSurfOfExtrusion);

    // destructor
    virtual ~SmSurfOfExtrusion();

    // equality operator
    virtual SmBoolean operator==(const SmSurface &crOther) const;

    // Set AnalUVDomain to desire STEP UVDomain - rebuild m_pNurb data. 
    //  useful for turning an infinite domain surface into a finite one.
    virtual SmStatus AdjustSTEPUVDomain(const SmExtent2d & crNewSTEPUVDomain);

    // Set AnalUVDomain from desired Nurbs UVDomain - don't rebuild m_pNurb data 
    virtual SmStatus UpdateAnalyticalDomain( const SmExtent2d & crNewNurbsDomain );

    virtual SmStatus ConvertUVFromSTEPToNURBS
    (
      const SmPoint2d & crSTEPUV,      ///< [in ]: STEP Domain UV point     <br>
      SmPoint2d & rNURBSUV             ///< [out]: Nurb Domain UV point     <br>
    ) const;

    virtual SmStatus ConvertUVFromNURBSToSTEP
    (
      const SmPoint2d & crNURBSUV,     ///< [in ]:      <br>
      SmPoint2d & rSTEPUV              ///< [out]:      <br>
    ) const;

    virtual SmStatus Copy
    (
      const SmContext & crContext,      ///< [in ]:       <br>
      SmSurface *& rpNewSurface         ///< [out]:       <br>
    ) const;
                              
    // create a step parameterized inifinitly swept surface                          
    static SmStatus CreateCanonical
    (
      const SmContext    & crContext,              ///< [in ]: context for new object construction             <br>
      SmBSplineCurve     * pSweptCurve,            ///< [in ]: GenCurve                                        <br>
      const SmVector3d   & crExtrusionVec,         ///< [in ]:                                                 <br>
      SmSurfOfExtrusion *& rpNewSurfOfExtrusion    ///< [out]:
    );

    // create offset surface - Approximate When necessary - Out Surf->Domain(s) may be trimmed but not scaled
    /*
    virtual SmStatus CreateOffsetSurface
    (
      const SmContext      & crContext,               ///< [in ]: context for new obj construction                                              <br>
      double                 dSignedOffsetDistance,   ///< [in ]: offset dist, (neg val = Offset dir opposite surface normal)                   <br>
      SmApproxTol3d          sApproxTol3d,            ///< [in ]: Max Dist between ApproxOffsetSurface and ideal offset shape                   <br>
      SmSurface* &           rOffsetSurface           ///< [out]: Offset Surf Approx, may be more than 1 when offsets have self-intersections   <br>
    ) const ; 
    */
    virtual SmStatus EvaluateSTEP
    (
      const SmPoint2d & crUV,          ///< [in ]: target surface point of Surf(u,v) = GenCurve(u) + v * ExtrusionVex ;                        <br>
      ULONG lHighestUDeriv,            ///< [in ]: Requested highest U derivative                                                              <br>
      ULONG lHighestVDeriv,            ///< [in ]: Requested highest V derivative                                                              <br>
      SmBoolean bUFromLeft,            ///< [in ]: bUFromLeft = if P is on U interval boundary                                                 <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the intervalal               <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the intervalval             <br>
      SmBoolean bVFromLeft,            ///< [in ]: bVFromLeft = if P is on V interval boundary                                                 <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval                 <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval                <br>
      SmBoolean bOnlyUpperHalf,        ///< [in ]: bOnlyUpperHalf = TRUE=compute upper half of matrix only                                     <br>
                                       ///<      : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value               <br>
                                       ///<      :           [Dv --]       [Dv   Duv   ---]                                                    <br>
                                       ///<      :                         [Dvv  ---   ---]                                                    <br>
      SmVector3d *aDerivatives         ///< [out]: matrix of evaluations values                                                                <br>
                                       ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                  <br>
                                       ///<      : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]                                          <br>
                                       ///<      :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]                                          <br>
                                       ///<      :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                          <br>
                                       ///<      :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                          <br>
                                       ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]    <br>
    ) const;

    virtual SmStatus EvaluateSTEPPoint
    (
      const SmPoint2d & crUV,         ///< [in ]: target step domain point
      SmPoint3d & rPoint              ///< [out]: 3D point
    ) const;

    // simple access
    SmStatus           GetCanonical            
    (
      SmBSplineCurve *& rpSweptCurve,          ///< [out]: GenCurve pointer       <br>
      SmVector3d      & rExtrusionVec          ///< [out]: ExtrusionVector        <br>
    )           const ;

    virtual SmExtent2d GetSTEPUVDomain         ()                                          const { return m_vAnalUVDomain ; }
    SmExtent1d         GetSTEPLinearParamExtent(SmSurfParamType * pOptExtrusionParam=NULL) const ;
    virtual SmExtent2d GetMaxAnalyticDomain    ()                                          const ;
    SmExtent1d         GetLinearParamExtent    (SmSurfParamType * pOptExtSurfParam=NULL)   const ; // rtn ExtrusionDir NURB ivl
    SmExtent1d         GetGenDirParamExtent    (SmSurfParamType * pOptGenSurfParam=NULL)   const ; // rtn GenDir NURB ivl

    SmPoint3d          GetOrigin               ()                                          const ;
    SmVector3d         GetExtrusionVector      ()                                          const ;
    double             GetBotHeight            ()                                          const ;
    double             GetTopHeight            ()                                          const ;
    SmBSplineCurve   * GetGenCurve             ()                                          const { return m_pGenCurve; }

    // predicates
    SmBoolean          GetIsPerpendicular      ()                                          const { return (m_eOrientation == SM_CO_PERPENDICULAR) ; }
    SmBoolean          GetIsPlanarGenerator    ()                                          const { return (m_eOrientation != SM_CO_NOT_PLANAR) ; }

    virtual SmStatus CreateGeneratorFromDistance
    (
      const SmContext & crContext,              ///< [in ]: context for new object construction                   <br>
      double dLocationAlongExtrusionArg,        ///< [in ]: Tgt parameter, range:[m_vAnalUVDomain.GetMin().y,     <br>
                                                ///<      :                       m_vAnalUVDomain.GetMax().y]     <br>
      SmBSplineCurve*& rpGeneratorCurveArg      ///< [out]: appropriately translated version of GenCurve          <br>
    ) const;

    virtual SmStatus CreateDirectrixFromPoint
    (
      const SmContext & crContext,             ///< [in ]: context for new object construction          <br>
      const SmPoint3d & rPointOnSurface,       ///< [in ]: Point to interpolate                         <br>
      double            d3DTolerance,          ///< [in ]: max 3d distance for out of domain points     <br>
      SmBSplineCurve *& rpDirectrix            ///< [out]: lineSeg through rPointOnSurface with length  <br>
                                               ///<      : and direction of ExtrusionVector             <br>
    ) const;

    virtual SmStatus DropPointFast
    (
      const SmExtent2d       & crUVDomain,                ///< [in ]: Domain of surface to search for solutions                                              <br>
      SmSolverOperationType    eSolverOperation,          ///< [in ]: oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT                <br>
      const SmPoint3d        & crTestPoint,               ///< [in ]: target point                                                                           <br>
      const SmVector3d       * cpOptInPointingVector,     ///< [in ]: specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams  <br>
      double                   dDistanceTolerance,        ///< [in ]:                                                                                        <br>
      const double           * cpdOptTargetDistance,      ///< [in ]: The pdOptTargetDistance, if not NULL, will be the corresponding                        <br>
                                                          ///<      : limit to a minimize/maximize operations.  In otherwords, it will                       <br>
                                                          ///<      : ask the solver to find a minimum value only if it is less than                         <br>
                                                          ///<      : the target distance or maximum value only if it is greater than                        <br>
                                                          ///<      : the target distance.                                                                   <br>
      SmSolutionRequestedType  eSolutionRequested,        ///< [in ]: SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                            <br>
      SmSolutionArray        & rSolutions                 ///< [out]: array of problem solutions reported as nurb surface UV parameter values                <br>
    ) const;

    virtual SmStatus GlobalPointSolveSTEP
    (
      const SmExtent2d    & crSTEPUVDomain,               ///< [in ]: STEP-based Domain of surface              <br>
      SmSolverOperationType eSolverOperation,             ///< [in ]: oneof SM_SO_MINIMIZE                      <br>
                                                          ///<      :  SM_SO_MAXIMIZE                           <br>
                                                          ///<      :  SM_SO_NORMALIZE                          <br>
                                                          ///<      :  SM_SO_INTERSECT                          <br>
      const SmPoint3d     & crTestPoint,                  ///< [in ]: target point                              <br>
      double                dDistanceTolerance,           ///< [in ]:                                           <br>
      const double        * cpdOptTargetDistance,         ///< [in ]:                                           <br>
      SmSolutionRequestedType eSolutionRequested,         ///< [in ]: oneof SM_SR_ALL, SM_SR_SINGLE             <br>
      SmSolutionArray     & rSolutions                    ///< [out]:                                           <br>
    );       

    virtual SmStatus GlobalSurfaceIntersect
    (
      const SmContext     & crContext,                    ///< [in ]: Context for the creation of curves                                                        <br>
      const SmExtent2d    & crUVDomain,                   ///<      :                                                                                           <br>
      const SmSurface     & crOtherSurface,               ///< [in ]: target 2nd intersecting surface                                                           <br>
      const SmExtent2d    & crOtherUVDomain,              ///<      :                                                                                           <br>
      const SmBoolean       bUseSurfaceEdges[2],          ///< [in ]:  = Normally both are TRUE unless you know                                                 <br>
                                                          ///<      :  that the edges of one surface do not intersect                                           <br>
                                                          ///<      :  the other surface.  It is a slight optimization                                          <br>
                                                          ///<      :  to set the flag to FALSE                                                                 <br>
      const SmApproxTol3d * pdOptApproxTol3d,             ///< [in ]: approximation tolerance If not given it uses 1/1000 of surface size                       <br>
      const double        * pdOptAngTolRad,               ///< [in ]: If not given it uses 30 degrees                                                           <br>
      SmTArray<SmCurve*>  * pOpt3DCurves,                 ///< [out]: 3D curves produced by intersection                                                        <br>
      SmTArray<SmCurve*>  * pOptSurface1UVCurves,         ///< [out]: UV curves on this surface produced by intersection                                        <br>
      SmTArray<SmCurve*>  * pOptSurface2UVCurves,         ///< [out]: UV curves on crOtherSurface produced by intersection                                      <br>
      SmTArray<SmTsectCurveType> * pOptCurveTypes,        ///< [out]: What type of curve is produced -                                                          <br>
                                                          ///<      : oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)                 <br>
                                                          ///<      :        SM_TC_CROSSING   - curve intersection (surf norms not parallel)                    <br>
                                                          ///<      :        SM_TC_TANGENT    - curve intersection (surf norms parallel)                        <br>
                                                          ///<      :        SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal) <br>
                                                          ///<      :        SM_TC_NEAR_TANGENT    - curve has small angle of intersection                      <br>
                                                          ///<      :        SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident  <br>
      SmTArray<double>    * pOptDeviations                ///< [out]:                                                                                           <br>
    ) const;

    SmStatus IntersectWithPlane
    (
      const SmContext     & crContext,                    ///< [in ]: context for new object construction                                                     <br>
      const SmExtent2d    & crUVDomain,                   ///< [in ]: intersection limit in Nurb Domain for this surface                                      <br>
      const SmPlane       & crOtherPlane,                 ///< [in ]: target intersection plane                                                               <br>
      const SmExtent2d    & crOtherUVDomain,              ///< [in ]: intersection limit for target plane                                                     <br>
      const SmBoolean       bUseSurfaceEdges[2],          ///< [in ]: TRUE = find xsect curve start points from boundaryCurve/surface xsects                  <br>
                                                          ///<      : typically these values are TRUE - its a small savings if you know                       <br>
                                                          ///<      : the boundaries of one surface don't intersect the other surface                         <br>
      const SmApproxTol3d * pdOptApproxTol3d,             ///< [in ]:                                                                                         <br>
      const double        * pdOptAngTolRad,               ///< [in ]:                                                                                         <br>
      SmBoolean           & rbNeedsMoreIntersections,     ///< [out]: TRUE = special case intersection failed-use general intersection                        <br>
      SmTArray<SmCurve*>  * pOpt3DCurves,                 ///< [out]: Intersection 3DCurves, NULL to ignore                                                   <br>
      SmTArray<SmCurve*>  * pOptSurface1UVCurves,         ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                 <br>
      SmTArray<SmCurve*>  * pOptSurface2UVCurves,         ///< [out]: associated UVTrimCurves on plane, NULL to ignore                                        <br>
      SmTArray<SmTsectCurveType> * pOptCurveTypes,        ///< [out]: one of for each 3DCurve, NULL to ignore                                                 <br>
                                                          ///<      : SM_TC_TOUCHING        - single point intersection (surf norms parallel)                 <br>
                                                          ///<      : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                    <br>
                                                          ///<      : SM_TC_TANGENT         - curve intersection (surf norms parallel)                        <br>
                                                          ///<      : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal) <br>
                                                          ///<      : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                           <br>
                                                          ///<      : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident       <br>
      SmTArray<double>    * pOptDeviations                ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                              <br>
    ) const;

    SmStatus IntersectWithCylinder
    (
      const SmContext     & crContext,                    ///< [in ]: context for new object construction                                                       <br>
      const SmExtent2d    & crUVDomain,                   ///< NotUsed: [in ]: intersection limit for this surface                                                       <br>
      const SmCone        & crOtherCylinder,              ///< [in ]: target intersection cylinder                                                              <br>
      const SmExtent2d    & crOtherUVDomain,              ///< NotUsed: [in ]: intersection limit for target cylinder                                                    <br>
      const SmBoolean       bUseSurfaceEdges[2],          ///< NotUsed: [in ]: bUseSurfaceEdges: TRUE = find xsect curve start points from boundaryCurve/surface xsects  <br>
                                                          ///<      : typically these values are TRUE - its a small savings if you know                         <br>
                                                          ///<      : the boundaries of one surface don't intersect the other surface                           <br>
      const SmApproxTol3d * pdOptApproxTol3d,             ///< [in ]:                                                                                           <br>
      const double        * pdOptAngTolRad,               ///< [in ]: pdOptAngTolRad =                                                                          <br>
      SmBoolean           & rbNeedsMoreIntersections,     ///< [out]: TRUE = special case intersection failed - use general intersection algorithm              <br>
      SmTArray<SmCurve*>  * pOpt3DCurves,                 ///< [out]: Intersection 3DCurves, NULL to ignore                                                     <br>
      SmTArray<SmCurve*>  * pOptSurface1UVCurves,         ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                   <br>
      SmTArray<SmCurve*>  * pOptSurface2UVCurves,         ///< [out]: associated UVTrimCurves on target cylinder, NULL to ignore                                <br>
      SmTArray<SmTsectCurveType> * pOptCurveTypes,        ///< [out]: associated intersecton type, NULL to ignore                                               <br>
      SmTArray<double>    * pOptDeviations                ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                                <br>
    ) const;

    SmStatus IntersectWithSurfOfExtrusion
    (
      const SmContext         & crContext,                 ///< NotUsed: [in ]: context for new object construction                                                       <br>
      const SmExtent2d        & crUVDomain,                ///< NotUsed: [in ]: intersection limit for this surface                                                       <br>
      const SmSurfOfExtrusion & crOtherSurfOfExtrusion,    ///< [in ]: target intersection cylinder                                                              <br>
      const SmExtent2d        & crOtherUVDomain,           ///< NotUsed: [in ]: intersection limit for target cylinder                                                    <br>
      const SmBoolean           bUseSurfaceEdges[2],       ///< [in ]: bUseSurfaceEdges: TRUE = find xsect curve start points from boundaryCurve/surface xsects  <br>
                                                           ///<      : typically these values are TRUE - its a small savings if you know                         <br>
                                                           ///<      : the boundaries of one surface don't intersect the other surface                           <br>
      const SmApproxTol3d     * pdOptApproxTol3d,          ///< [in ]:                                                                                           <br>
      const double            * pdOptAngTolRad,            ///< [in ]: pdOptAngTolRad =                                                                          <br>
      SmBoolean               & rbNeedsMoreIntersections,  ///< [out]: TRUE = special case intersection failed - use general intersection algorithm              <br>
      SmTArray<SmCurve*>      * pOpt3DCurves,              ///< [out]: Intersection 3DCurves, NULL to ignore                                                     <br>
      SmTArray<SmCurve*>      * pOptSurface1UVCurves,      ///< [out]: associated UVTrimCurves on this surface, NULL to ignore                                   <br>
      SmTArray<SmCurve*>      * pOptSurface2UVCurves,      ///< [out]: associated UVTrimCurves on target cylinder, NULL to ignore                                <br>
      SmTArray<SmTsectCurveType> * pOptCurveTypes,         ///< [out]: associated intersecton type, NULL to ignore                                               <br>
      SmTArray<double>        * pOptDeviations             ///< [out]: associated max 3DCurve to surface distance, NULL to ignore                                <br>
    ) const;

    static SmBoolean IsNurbSurfaceSurfOfExtrusion
    (
      const SmContext        & crContext,                  ///< [in ]: context for new object construction
      const SmBSplineSurface * pTestSurface,               ///< [in ]: target surface                                                                                                            <br>
      SmSurfOfExtrusion     *& rpSurfOfExtrusion,          ///< [out]: new surface when target surface is a SurfaceofExtrusion, or NULL                                                          <br>
      double                   dToleranceScale = 1.0       ///< [in ]: tol =  dToleranceScale * ANALYTIC_TOL_SCALE * SM_EFF_ZERO * (1.0 + sMinPnt.GetMaxDimension() + sMaxPnt.GetMaxDimension()) <br>
    );

    void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                   m_pGenCurve->SetContext(cpContext); }

    virtual void SetOutOfBoundsEnabled( SmBoolean bOutOfBoundsEnabled )
      {
        SmBSplineSurface::SetOutOfBoundsEnabled( bOutOfBoundsEnabled );
        m_pGenCurve->SetOutOfBoundsEnabled( bOutOfBoundsEnabled );
      }

    virtual SmStatus MakeNurb();

    virtual SmStatus Reparameterize(const SmExtent2d & crNewDomain);

    virtual SmStatus STEPInversion
    (
      const SmExtent2d & crAnalUVDomain,            ///< [in ]: domain limit for valid transformations                               <br>
                                                    ///<      : Passed on to SmBSplineSurface::STEPINVERSION for nonPlanar GenCurves <br>
      const SmPoint3d  & crPointOnSurf,             ///< [in ]: Target Point - must be on surface within Tolerance                   <br>
      double             dDistanceTolerance,        ///< [in ]: Max allowed distance between Point and Surface                       <br>
      SmPoint2d        & rdAnalUVParameter,         ///< [out]: STEP UV params for target point                                      <br>
                                                    ///<      : [U = Degrees about rotation Axis,                                    <br>
                                                    ///<      :  V = STEP Param of rotatedPoint on genCurve]                         <br>
      SmLocationType   & reLocation,                ///< [out]: oneof SM_LT_POLE,                                                    <br>
                                                    ///<      :  SM_LT_U_SEAM,                                                       <br>
                                                    ///<      :  SM_LT_V_SEAM,                                                       <br>
                                                    ///<      :  SM_LT_UV_SEAM,                                                      <br>
                                                    ///<      :  SM_LT_INTERIOR,                                                     <br>
                                                    ///<      :  SM_LT_EXTERIOR                                                      <br>
      SmPoint2d        * pUVGuess = NULL            ///< [in,out]: guess parameter.                                                  <br>
    ) const;

    // gwc: can use SmBSplineSurface version:  virtual SmStatus SwapUV();
    virtual void      ToggleSwapUVBit()  { m_bSwapUV = m_bSwapUV ? FALSE : TRUE ; }
    virtual SmBoolean GetSwapUVBit() { return m_bSwapUV ; }

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

    virtual SmStatus TrimWithDomain(SmExtent2d & crTrimDomain);

    // get memory used for curve but not its attributes
    virtual ULONG GetMemoryUsed
    (
      ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes         <br>
      SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value               <br>
    ) const ;

    virtual SmStatus WriteToDB
    (
      SmDatabaseIO & rDB,                      ///< [in ]: target output stream                                 <br>
      ULONG          lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes    <br>                                                                   
    ) const;

    static  SmStatus ReadFromDB
    (
      SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                                          <br>
      SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                                            <br>
      const SmContext & crContext,          ///< [in ]: context for new object construction                                                             <br>
      SmSurface      *& rpNewSurface,       ///< [out]: NULL on input = new object allocated in this routine built from stream data                     <br>
                                            ///<      : NotNULL on input = pointer to an empty object to be filled by this routine                      <br>
      ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                                               <br>
    );

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                                ///<      : default:[SM_LEVEL_0]                                                                               <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
    ) const ;

    // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
    SM_COMMON(SmSurfOfExtrusion,SmBSplineSurface,SmSurfOfExtrusion_TYPE);

protected:

    SmPoint3d  GetCurvePoint() const { return (m_vCurvePoint) ; }
    SmVector3d GetCurveVec()   const { return (m_vCurveVec) ; }

    void SetCurvePoint (SmPoint3d         sCurvePoint) { m_vCurvePoint  = sCurvePoint ; }
    void SetCurveVec   (SmVector3d        sCurveVec)   { m_vCurveVec    = sCurveVec ; }
    void SetOrientation(SmCurveOrientType eOrient)     { m_eOrientation = eOrient ; }

} ; // end class SmSurfOfExtrusion

#endif // __SMSURFOFEXTRUSION_H__

