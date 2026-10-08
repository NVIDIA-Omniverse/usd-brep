// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmSurfaces.h

PURPOSE: 
    Contains popular, high level "C" type functions that create 
    primitive objects (cube, sphere, etc).

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/

#ifndef __SmApiSurfaces_H__
#define __SmApiSurfaces_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmTArray.h>

class SmContext;
class SmBrep;
class SmPlane;
class SmCylinder;
class SmCone;
class SmVector3d;

/// Requires at least 4 rows and 4 columns (bicubic); fewer returns SM_ERR_INVALID_INPUT.
SMAPI_EXPORT SmApiStatus SmApiCreateSurface( 
    const SmTArray<SmPoint3d> & rControlPoints,  ///< [in ]: List of control points                                     <br>
    ULONG lNumRows,                        ///< [in ]: Number of rows                                             <br>
    ULONG lNumColumns,                     ///< [in ]: Number of columns                                          <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSurfaceFromCornerPoints(
    const SmPoint3d& crCornerUminVmin,     ///< [in ]:                                                            <br>
    const SmPoint3d& crCornerUmaxVmin,     ///< [in ]:                                                            <br>
    const SmPoint3d& crCornerUminVmax,     ///< [in ]:                                                            <br>
    const SmPoint3d& crCornerUmaxVmax,     ///< [in ]:                                                            <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                        <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSurfaceFromOrderedPoints( 
    const SmTArray<SmPoint3d> & rPoints,         ///< [in ]: points ordered: P[row][col] = p[row*lNumCols + col]         <br>
    ULONG lNumRows,                        ///< [in ]: Number of rows                                              <br>
    ULONG lNumColumns,                     ///< [in ]: Number of columns                                           <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSurfaceFromRandomPoints( 
    const SmTArray<SmPoint3d> & rPoints,         ///< [in ]: Input points                                                <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateExtrude( 
    SmBSplineCurve*  pCurve,               ///< [in ]: Generation curve                                            <br>
    const SmVector3d & rSweepVector,             ///< [in ]: Sweep vector                                                <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSweep( 
    SmBSplineCurve*  pCurve,               ///< [in ]: Generation curve                                            <br>
    SmBSplineCurve*  pPath,               ///< [in ]: Sweep vector                                                <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateRuledSurface( 
    SmBSplineCurve* rCurve1,               ///< [in ]: Generation curve                                            <br>
    SmBSplineCurve* rCurve2,               ///< [in ]: Generation curve                                            <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSurfaceRevolution( 
    SmBSplineCurve* rCurve,                ///< [in ]: Generation curve                                            <br>
    const SmPoint3d& rOrigin,                    ///< [in ]: Origin of axis of revolution                                <br>
    const SmVector3d& rAxisDir,                  ///< [in ]: Axis of revolution                                          <br>
    double dAngleDeg,                      ///< [in ]: Angle of revolution                                         <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateSkin( 
    const SmTArray<SmBSplineCurve*>& rProfiles,  ///< [in ]: Profile curves                                              <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
);

SMAPI_EXPORT SmApiStatus SmApiCreateOffsetSurface( 
    SmSurface* pSrfToOffset,                ///< [in ]: Surface to offset                                               <br>
    double dOffsetDistance,                 ///< [in ]: Distance of offset ( positive: normal dir of input surface      <br>
                                            //                           negative: opp normal dir of input surface )    <br>
    SmSurface*&  rpResult                  ///< [out]: Resulting SmBSplineSurface                                      <br>
);                   

/// The three surface evaluators below write their outputs only on success.
SMAPI_EXPORT SmApiStatus SmApiEvaluateSurfacePoint( 
    const SmSurface* pSurface,                  ///< [in ]: Surface to evaluate                                            <br>
    const SmVector2d& crUV,                     ///< [in ]: Parameter (uv) at which to evaluate                            <br>
    SmVector3d& crPoint                   ///< [out]: Resulting Point at this parameter                              <br>
);

SMAPI_EXPORT SmApiStatus SmApiEvaluateSurfaceNormal( 
    const SmSurface* pSurface,                  ///< [in ]: Surface to evaluate                                          <br>
    const SmVector2d& crUV,                     ///< [in ]: Parameter (uv) at which to evaluate                          <br>
    SmVector3d& crNormal                  ///< [out]: Surface Normal at parameter                                   <br>
);

SMAPI_EXPORT SmApiStatus SmApiEvaluateSurfaceDerivatives( 
    const SmSurface* pSurface,                  ///< [in ]: Surface to evaluate                                      <br>
    const SmVector2d& crUV,                     ///< [in ]: Parameter (uv) at which to evaluate                      <br>
    SmVector3d& crPoint,                  ///< [out]: Resulting Point at this parameter                        <br>
    SmVector3d& rDU,                      ///< [out]: Resulting U derivative                                   <br>
    SmVector3d& rDV                       ///< [out]: Resulting V derivative                                   <br>
);

#endif
