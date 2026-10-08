// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiTrimmedSurfaces.h

PURPOSE:
    Contains API-level functions and utilities for creating and
    manipulating trimmed surfaces (surface trimming operations,
    helpers, and related types).

GENERAL NOTES:
    High level functions may assume some input parameters
    for ease of use. For maximum flexibility, related functions
    can be found in SMLib.
**********************************************************************/

#ifndef __SmApiTrimmedSurfaces_H__
#define __SmApiTrimmedSurfaces_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmTArray.h>

class SmContext;
class SmBrep;
class SmSurface;
class SmVector3d;


/// Ownership: on success pSurface and the cr3DCurves curves are consumed into rpResult.
/// On failure rpResult is NULL and ownership of pSurface and the curves is indeterminate:
/// some may already be attached to a partial Brep that is not freed.
SMAPI_EXPORT SmApiStatus SmApiTrimSurfaceWith3dCurves( 
    SmSurface* pSurface,             ///< [in ]: Surface to trim (consumed)                                 <br>
    const SmTArray<ULONG>& crCurveLoops,   ///< [in ]: Number of curves in each loop                              <br>
    const SmTArray<SmCurve*>& cr3DCurves,  ///< [in ]: Trimming curves in head to tail order                      <br>
    SmBrep*& rpResult               ///< [out]: Resulting SmBrep                                           <br>
);

SMAPI_EXPORT SmApiStatus SmApiTrimProjectParallel( 
    SmObject* pObject,                 ///< [in ]: Brep or standalone Surface to trim (consumed on success)            <br>
    SmBSplineCurve* crCurveToProject,  ///< [in ]: Curve to project to the faces                                       <br>
    const SmVector3d& crProjectionDirection, ///< [in ]: Defines direction of projection                                     <br>
    const SmPoint3d & crReferencePoint,      ///< [in ]: Used only if trimType is SM_TT_KEEP_POINT or SM_TT_DELETE_POINT.    <br>
    SmTrimType eTrimType,              ///< [in ]: SM_TT_KEEP_POINT: Uses crReferencePoint to determine side to keep   <br>
                                       ///< [   ]: SM_TT_DELETE_POINT: Uses crReferencePoint to determine side to delete   <br>
                                       ///< [   ]: SM_TT_SPLIT: Keep both sections of the face. Ignores crReferencePoint.  <br>
    SmBrep*& rpResult                 ///< [out]: Resulting SmBrep; NULL on failure. On failure a Brep pObject     <br>
                                      ///<        stays owned by the caller but may be partially trimmed. A       <br>
                                      ///<        rejected input leaves a Surface pObject untouched; if the kernel<br>
                                      ///<        trim itself fails, the Surface stays attached to an internal    <br>
                                      ///<        Brep that is not freed.                                          <br>
);

/// Ownership of crPlanarCurves: consumed on success (the new faces use them; do
/// not delete or reuse them). On failure they are left with the caller, who must
/// delete them, and rpResult is set to NULL.
SMAPI_EXPORT SmApiStatus SmApiCreatePlanarFaces( 
    const SmTArray<SmCurve*>& crPlanarCurves,  ///< [in ]: Planar bounding curves                                            <br>
    SmBrep*& rpResult                   ///< [out]: Resulting SmBrep                                                  <br>
);


#endif
