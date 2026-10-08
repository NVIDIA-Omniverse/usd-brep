// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmCurves.h

PURPOSE: 
   Contains popular high level "C" type functions that create/modify curves.
    Additional functions exist in SMLib

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib
**********************************************************************/


#ifndef __SmApiCurves_H__
#define __SmApiCurves_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmTArray.h>
#include <SmCurveTypes.h>

class SmContext;
class SmPoint3d;
class SmLine;
class SmCircle;
class SmCurve;


SMAPI_EXPORT SmApiStatus SmApiCreateLineSegment(
    const SmPoint3d& crStartPt,           ///< [in ]: Start Point                                               <br>
    const SmPoint3d& crEndPt,             ///< [in ]: End Point                                                 <br>
    SmLine*&  rpResult                   ///< [out]: Resulting SmLine                                          <br>
);                                       
                                         
SMAPI_EXPORT SmApiStatus SmApiCreateLine(                                        
    const SmPoint3d& crStartPt,                 ///< [in ]: Start Point                                                <br>
    const SmVector3d& crDirection,              ///< [in ]: Direction vector                                           <br>
    SmLine*&  rpResult                   ///< [out]: Resulting SmLine                                           <br>
);                                      
                                        
/// Center coordinates and radius must be valid doubles in [-1e18, 1e18]
/// (SM_IS_VALID_DOUBLE / SM_BIG_DOUBLE/100). The radius must exceed SM_EFF_ZERO
/// (1e-12), and the center/radius combination must be representable: a center
/// whose magnitude dwarfs the radius (e.g. (1e18,0,0) with radius 1) collapses
/// the circle's extents in double precision and is rejected. Any violation
/// returns SM_ERR_INVALID_INPUT with rpResult == NULL; rpResult is only valid on
/// SM_SUCCESS.
SMAPI_EXPORT SmApiStatus SmApiCreateCircle(                                       
    const SmPoint3d& crCenterPt,          ///< [in ]: Start Point                                                 <br>
    double dRadius,                       ///< [in ]: Radius                                                      <br>
    SmBSplineCurve*&  rpResult           ///< [out]: Resulting SmBSplineCurve                                    <br>
);

/// Center coordinates and radius must be valid doubles in [-1e18, 1e18]
/// (SM_IS_VALID_DOUBLE / SM_BIG_DOUBLE/100). The radius must exceed SM_EFF_ZERO
/// (1e-12), and the center/radius combination must be representable: a center
/// whose magnitude dwarfs the radius (e.g. (1e18,0,0) with radius 1) collapses
/// the circle's extents in double precision and is rejected. Any violation
/// returns SM_ERR_INVALID_INPUT with rpResult == NULL; rpResult is only valid on
/// SM_SUCCESS.
SMAPI_EXPORT SmApiStatus SmApiCreateCircle( 
    const SmPoint3d& crCenterPt,          ///< [in ]: Start Point                                                 <br>
    double dRadius,                       ///< [in ]: Radius                                                      <br>
    SmCircle*&  rpResult                 ///< [out]: Resulting SmCircle                                          <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateEllipse( 
    const SmPoint3d& crOrigin,                   ///< [in ]: Origin of the ellipse                                         <br>
    double dRadius1,                       ///< [in ]: Dist from origin to ellipse circumference in XAxis direction  <br>
    double dRadius2,                       ///< [in ]: Dist from origin to ellipse circumference in YAxis direction  <br>
    SmEllipse*&  rpResult                 ///< [out]: Resulting SmEllipse                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateArc( 
    const SmPoint3d& crCenterPt,                 ///< [in ]: Start Point                                               <br>
    double dRadius,                        ///< [in ]: Radius                                                    <br>
    double dStartAngleDeg,                 ///< [in ]: Start Angle in Degrees                                    <br>
    double dEndAngleDeg,                   ///< [in ]: End Angle in Degrees                                      <br>
    SmBSplineCurve*& rpResult             ///< [out]: Resulting SmBSplineCurve                                  <br>
);

/// Every input is validated against a finite domain before the fitter runs; any
/// out-of-domain value returns SM_ERR_INVALID_INPUT with rpResult == NULL.
/// dTolerance is a hard contract: the achieved maximum 3D deviation from the
/// analytic helix is verified after the fit, and if it exceeds dTolerance the
/// call returns SM_ERR_NOT_WITHIN_TOLERANCE with rpResult == NULL.
/// Fractional dNumTurns is accepted, but below ~1 turn the helix winds steeply
/// (nearly straight): a tight dTolerance may be unmeetable (returning
/// SM_ERR_NOT_WITHIN_TOLERANCE) and such paths may be rejected by SmApiCreatePipeSweep.
/// Prefer dNumTurns >= 1 for a reliable sweep path.
SMAPI_EXPORT SmApiStatus SmApiCreateHelix( 
    const SmPoint3d& crOrigin,                   ///< [in ]: Base point (finite); helix runs along +Z over [Z, Z+dHeight] <br>
    double dHeight,                        ///< [in ]: Axial length along +Z. Range [1e-7, 1e18]                 <br>
    double dRadiusStart,                   ///< [in ]: Radius at the base. Range [1e-7, 1e18] (zero rejected)    <br>
    double dRadiusEnd,                     ///< [in ]: Radius at the top; != start gives a cone. Range [1e-7, 1e18] <br>
    double dNumTurns,                      ///< [in ]: 360-degree turns over the height (fractions OK). Range [1e-7, 124.25] <br>
    SmBoolean bRightHanded,                ///< [in ]: TRUE = right handed, FALSE = left handed                  <br>
    double dTolerance,                     ///< [in ]: Max deviation of the B-spline fit from the true helix. Range [1e-6, 1e18] <br>
    SmBSplineCurve*& rpResult             ///< [out]: Resulting SmBSplineCurve; set to NULL and only valid when  <br>
                                          ///<      : the returned SmApiStatus is SM_SUCCESS                    <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateRectangle( 
    const SmPoint3d& crCenterPt,                 ///< [in ]: Center Point                                               <br>
    double dLength,                        ///< [in ]: Length along X axis                                                    <br>
    double dWidth,                         ///< [in ]: Width along Y axis                                    <br>
    SmTArray<SmBSplineCurve*>& rpResult   ///< [out]: Resulting SmBSplineCurve                                  <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateRegularPolygon( 
    const SmPoint3d& crCenterPt,                 ///< [in ]: Center Point                                               <br>
    ULONG nSides,                          ///< [in ]:
    double dRadius,                        ///< [in ]: Radius of circumscribed  circle                                                    <br>
    SmTArray<SmBSplineCurve*>& rpResult   ///< [out]: Resulting SmBSplineCurve                                  <br>
);

/// Requires at least 4 points (cubic); fewer returns SM_ERR_INVALID_INPUT.
SMAPI_EXPORT SmApiStatus SmApiCreateCurve( 
    const SmTArray<SmPoint3d>& crPoints,         ///< [in ]: Points                                                     <br>
    SmBSplineCurve*&  rpResult            ///< [out]: Resulting SmBSplineCurve                                   <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCanonicalCurve(
    const SmTArray<SmPoint3d>& crPoints,    ///< [in ]: Points                                                       <br>
    const SmTArray<double>& crKnots,        ///< [in ]: Knots                                                        <br>
    const SmTArray<ULONG>& crKnotMultiplicities, ///< [in ]: Knots multiplicities of the end knots must be lDegree+1 and  <br>
                                            ///<      : and internal multiplicities must be lDegree or less.         <br>
    const SmTArray<double>* crWeights,      ///< [in ]: Weights - optional                                           <br>
    ULONG lDegree,                          ///< [in ]: Degree                                                       <br>
    SmBSplineCurveForm eBSplineCurveForm,   ///< [in ]: SM_CF_POLYLINE_FORM,  SM_CF_PARABOLIC_ARC,                   <br>
                                            ///<      : SM_CF_CIRCULAR_ARC,   SM_CF_HYPERBOLIC_ARC,                  <br>
                                            ///<      : SM_CF_ELLIPTIC_ARC,   SM_CF_UNSPECIFIED,                     <br>
                                            ///<      : SM_CF_HELICAL_ARC                                            <br>
    SmBSplineCurve*&  rpResult             ///< [out]: Resulting SmBSplineCurve                                     <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateCurveApproxPoints( 
    const SmTArray<SmPoint3d>& crPoints,          ///< [in ]: Points                                                       <br>
    SmCurve*&  rpResult                    ///< [out]: Resulting SmCurve                                            <br>
);

/// Requires at least 4 finite points (degree-3 fit). Fewer points or any
/// non-finite point returns SM_ERR_INVALID_INPUT with rpResult == NULL.
/// For the distance-based parameterizations (SM_CP_CHORDLENGTH and
/// SM_CP_CENTRIPETAL) successive points must be distinct: a coincident pair
/// (separation <= SM_EFF_ZERO) also returns SM_ERR_INVALID_INPUT, since a
/// zero-length span has no defined chord/centripetal spacing. SM_CP_UNIFORM
/// ignores spacing and tolerates duplicates. rpResult is only valid on
/// SM_SUCCESS.
SMAPI_EXPORT SmApiStatus SmApiCreateCurveInterpPoints( 
    const SmTArray<SmPoint3d>& crPoints,          ///< [in ]: Points: >= 4, all finite; distinct if distance-based        <br>
    SmCurve*&  rpResult,                    ///< [out]: Resulting SmCurve                                           <br>
    SmCurveParameterizationType eParameterization = SM_CP_UNIFORM  ///< [in ]: Knot parameterization of the fit    <br>
);

SMAPI_EXPORT SmApiStatus SmApiOffsetCurve( 
    SmBSplineCurve* pCurveToOffset,         ///< [in ]: Curve to offset, in its own plane; a straight curve        <br>
                                            ///<      : is offset in the XY plane (normal 0,0,1), or in the XZ      <br>
                                            ///<      : plane (normal 0,1,0) if it is parallel to Z                <br>
    double dOffsetDistance,                 ///< [in ]: Distance of offset ( positive to right of input curve      <br>
                                            ///<      :                      negative to left of input curve )     <br>
    SmBSplineCurve*&  rpResult             ///< [out]: Resulting SmBSplineCurve                                   <br>
);

SMAPI_EXPORT SmApiStatus SmApiEvaluateCurve( 
    const SmCurve* pCurve,                        ///< [in ]: Curve to evaluate                                           <br>
    double dParameter,                      ///< [in ]: Parameter at which to evaluate                              <br>
    SmVector3d*& crPoint,                   ///< [out]: Opt - Resulting Point at this parameter                     <br>
    SmVector3d*& crDeriv1,                  ///< [out]: Opt - First Derivative (tangent) at this parameter          <br>
    SmVector3d*& crDeriv2                   ///< [opt]: - Second Derivative at this parameter                       <br>
);

/// Returns SM_ERR_INVALID_INPUT for a NULL curve or a parameter outside its
/// natural interval (including NaN); otherwise returns the kernel status.
SMAPI_EXPORT SmApiStatus SmApiEvaluateContinuity( 
    const SmCurve* pCurve,                        ///< [in ]: Curve to evaluate                                           <br>
    double dParameter,                      ///< [in ]: Parameter at which to evaluate                              <br>
    SmContinuityType& crType                ///< [out]: Continuity at this parameter; valid on success              <br>
);

/// Either output may be NULL to ignore it. Outputs are appended to only on success;
/// a failed lift returns its status and leaves the outputs unchanged.
SMAPI_EXPORT SmApiStatus SmApiProjectCurve( 
    const SmBSplineCurve* pCurve,                  ///< [in ]: Curve to drop                                              <br>
    const SmSurface* pSurface,                     ///< [in ]: Surface                                                    <br>
    SmTArray<SmBSplineCurve*>* pOptCurves2d, ///< [out]: Resulting uv curves on surface                             <br>
    SmTArray<SmBSplineCurve*>* pOptCurves3d ///< [out]: Resulting 3d curves on surface                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiDropCurveToSrf( 
    SmBSplineCurve* pCurve,                  ///< [in ]: Curve to drop                                              <br>
    SmSurface* pSurface,                     ///< [in ]: Surface                                                    <br>
    SmTArray<SmBSplineCurve*>* pOptCurves2d, ///< [out]: Resulting uv curves on surface (NULL to ignore)            <br>
    SmTArray<SmBSplineCurve*>* pOptCurves3d ///< [out]: Resulting 3d curves on surface (NULL to ignore);          <br>
                                            ///<        outputs are appended to only on success                  <br>
);

SMAPI_EXPORT SmApiStatus SmApiLiftUVCurveFromSrf( 
    const SmBSplineCurve* pUVCurve,                ///< [in ]: UV curve to lift                                           <br>
    const SmSurface* pSurface,                     ///< [in ]: Surface                                                    <br>
    SmTArray<SmBSplineCurve*>* pCurves3d    ///< [out]: Resulting 3d curves on surface                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiMakeCurvesCompatible(
    SmTArray<SmBSplineCurve*>& pCurves      ///< [in/out]: NURBS curves; modified in place, possibly only partly  <br>
                                            ///<          on failure                                            <br>
);

SMAPI_EXPORT SmApiStatus SmApiOrderCurves(
    SmTArray<SmBSplineCurve*>& pCurves      ///< [in/out]: Curves; on success reordered in place into loop       <br>
                                            ///<          sequence. Curves are not reversed, and per-curve     <br>
                                            ///<          orientation and loop boundaries are not reported,    <br>
                                            ///<          so consecutive curves need not meet head to tail.    <br>
);

/// Remove B-spline knots in place using the curve's effective approximation tolerance.
/// Non-B-spline curve types are unchanged. Returns SM_ERR_INVALID_INPUT for a null curve; propagates knot-removal failures.
SMAPI_EXPORT SmApiStatus SmApiRemoveCurveKnots(
    SmCurve* pCurve                       ///< [in/out]: Curve to simplify
);

SMAPI_EXPORT SmApiStatus SmApiConvertToLinesAndArcs( 
    SmBSplineCurve* pCurve,                  ///< [in ]: Curve to convert; must not be degenerate                     <br>
    SmTArray<SmBSplineCurve*>& pLinesAndArcs ///< [out]: Resulting pieces (array is reset first); a line, an arc,   <br>
                                             ///<        or a curve smaller than the 0.001 fit tolerance is returned  <br>
                                             ///<        as a copy of itself                                          <br>
);

#endif
