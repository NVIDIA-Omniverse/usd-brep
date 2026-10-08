// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiIntersectors.h

PURPOSE: 
    Contains popular, high level "C" type functions that intersect two objects.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib
**********************************************************************/

#ifndef __SmApiIntersectors_H__
#define __SmApiIntersectors_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

SMAPI_EXPORT SmApiStatus SmApiIntersectCurves( 
    SmCurve* pCurve1,                       ///< [in ]: Curve to intersect                                           <br>
    SmCurve* pCurve2,                       ///< [in ]: Curve to intersect                                           <br>
    SmTArray<SmPoint3d>* p3DPoints,         ///< [out]: Resulting intersection points                                <br>
    SmTArray<double>* parametersOnCurve1,   ///< [out]: Resulting parameters on first curve                          <br>
    SmTArray<double>* parametersOnCurve2   ///< [out]: Resulting parameters on second curve                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiIntersectSurfaces( 
    SmSurface* pSurface1,                 ///< [in ]: Surface to intersect                                              <br>
    SmSurface* pSurface2,                 ///< [in ]: Surface to intersect                                              <br>
    SmTArray<SmCurve*> * p3DCurves       ///< [out]: Resulting 3d curves                                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiIntersectFaces(
    SmFace* pFace1,                       ///< [in ]: Pointer to first face                                            <br>
    SmFace* pFace2,                       ///< [in ]: Pointer to other face                                            <br>
    SmTArray<SmCurve*> & r3DCurves,       ///< [out]: Caller-owned intersection curves; empty on failure.              <br>
                                        ///<        The array is reset on entry; release prior curves first.         <br>
    SmTArray<SmPoint3d> * pOpt3DPoints   ///< [opt]: Intersection points; reset on entry and empty on failure          <br>
);


SMAPI_EXPORT SmApiStatus SmApiIntersectBreps(
    SmBrep* pBrep1,                         ///< [in ]: Pointer to brep                                              <br>
    SmBrep* pBrep2,                         ///< [in ]: Pointer to other brep                                        <br>
    SmTArray<SmCurve*> & rCurves3d,         ///< [out]: Resulting 3d intersection curves                             <br>
    SmTArray<SmPoint3d> * pOptPoints3d     ///< [out]: Resulting 3d intersection points                             <br>
);

SMAPI_EXPORT SmApiStatus SmApiIntersectBrepWithPlane(
    SmBrep*  pBrep,                         ///< [in ]: Pointer to brep                                              <br>
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane                            <br>
    const SmVector3d& rPlaneNormal,         ///< [in ]: Plane normal; must be non-zero, need not be unit             <br>
    SmTArray<SmCurve*>& rSectionCurves     ///< [out]: Resulting section curves, appended to the array              <br>
);

// These three curve-intersection queries append caller-owned results on
// success. No new hits returns SM_ERR, even if the output arrays were nonempty
// on entry. On a solve, classification, copy, trim, or evaluation failure,
// return the kernel status and remove results added by this call only.
SMAPI_EXPORT SmApiStatus SmApiIntersectCurveSurface(
    SmCurve* pCurve,                        ///< [in ]: Pointer to curve                                              <br>
    SmSurface*  pSurface,                   ///< [in ]: Pointer to surface                                            <br>
    SmTArray<SmCurve*>& rCurves,            ///< [out]: Resulting intersection curves                                 <br>
    SmTArray<SmPoint3d>& rPoints           ///< [out]: Resulting intersection points                                 <br>
);                                                                                                                   

SMAPI_EXPORT SmApiStatus SmApiIntersectCurveFace(
    SmCurve* pCurve,                        ///< [in ]: Pointer to curve                                              <br>
    SmFace*  pFace,                         ///< [in ]: Pointer to face; hits outside its trim loops are dropped     <br>
    SmTArray<SmCurve*>& rCurves,            ///< [out]: Resulting intersection curves (overlaps). An overlap is kept  <br>
                                            ///<        or dropped whole, by whether its midpoint is on the face;     <br>
                                            ///<        it is not clipped to the face trim.                           <br>
    SmTArray<SmPoint3d>& rPoints           ///< [out]: Resulting intersection points                                 <br>
);

SMAPI_EXPORT SmApiStatus SmApiIntersectCurveBrep(
    SmCurve* pCurve,                        ///< [in ]: Pointer to curve                                              <br>
    SmBrep*  pBrep,                         ///< [in ]: Pointer to brep                                               <br>
    SmTArray<SmCurve*>& rCurves,            ///< [out]: Resulting intersection curves (overlaps), appended. As for   <br>
                                            ///<        SmApiIntersectCurveFace, overlaps are not clipped to face     <br>
                                            ///<        trims; one lying along a shared edge is reported once.        <br>
    SmTArray<SmPoint3d>& rPoints           ///< [out]: Resulting intersection points, appended; a point on a shared  <br>
                                           ///<        edge is reported once. If intersecting any face fails, both   <br>
                                           ///<        arrays are restored to their sizes on entry.                  <br>
);

#endif
