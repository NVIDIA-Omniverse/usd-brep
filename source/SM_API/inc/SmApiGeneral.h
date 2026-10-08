// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiGeneral.h

PURPOSE: 
    Contains popular high level "C" type functions of general use.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib, etc
**********************************************************************/


#ifndef __SmApiGeneral_H__
#define __SmApiGeneral_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmCoreTypes.h>

class SmBrep;
class SmFace;
class SmContext;
class SmObject;
class SmPolyBrep;
class SmSurface;
class SmBSplineCurve;
class SmCurve;
class SmAxis2Placement;
class SmVector3d;


// Context and concurrency: many SM_API operations use the process-global default
// SmContext (SmApiGetOrCreateContext()), whose traversal mark counters are unlocked.
// Some operations use other contexts; this does not make arbitrary SmApi* calls
// safe to run concurrently in one process. Use one worker process per concurrent
// job, with a long-lived default context. In the shipped build, caches belong to
// objects, so context lifetime alone does not preserve them across object
// replacement. Context-owned SmGlobalCache queues require SM_USE_GLOBAL_CACHE,
// which is disabled by default. Operations run single-threaded in the shipped
// build. Lazy initialization is not synchronized: call SmApiCreateContext() once
// at startup before spawning threads. See .agents/operations/concurrency.md.
SMAPI_EXPORT void SmApiCreateContext();
SMAPI_EXPORT SmContext* SmApiGetOrCreateContext();

SMAPI_EXPORT SmApiStatus SmApiDraw(
    SmBrep* pBrep,                 ///< [in ]: Draw this object                            <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]   <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]  <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]  <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw(
    SmFace* pFace,                 ///< [in ]: Draw this object                             <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]    <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]   <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]   <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw(
    SmCurve* pCurve,               ///< [in ]: Draw this object                              <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]     <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]    <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]    <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw(
    SmSurface* pSurface,           ///< [in ]: Draw this object                               <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]      <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]     <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]     <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw( 
    const SmTArray<SmCurve*>& pArray,    ///< [in ]: SmObject to draw                               <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]      <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]     <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]     <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw( 
    const SmTArray<SmPoint3d>& pArray,   ///< [in ]: SmObject to draw                                <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]       <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]      <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]      <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw(
    const SmPoint3d& rBasePt,      ///< [in ]: Base point of vector to draw                     <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]        <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]       <br>
    SmBoolean bCenter = false );   ///< [in ]: Center object   [Optional] default:[false]       <br>

SMAPI_EXPORT SmApiStatus SmApiDraw(
    const SmPoint3d& rBasePt,      ///< [in ]: Base point of vector to draw                     <br>
    const SmVector3d& rDirection,  ///< [in ]: Vector to draw                                   <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]        <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]       <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]       <br>
);

SMAPI_EXPORT SmApiStatus SmApiDraw( 
    SmPolyBrep* pPolyBrep,         ///< [in ]: SmObject to draw                                 <br>
    const SmVector3d* pColor = NULL, ///< [in ]: Color of object [Optional] default:[NULL]        <br>
    SmBoolean bClearFirst = false, ///< [in ]: Clear viewport  [Optional] default:[false]       <br>
    SmBoolean bCenter = false      ///< [in ]: Center object   [Optional] default:[false]       <br>
);


/**
 * Transform operations accept independent modeling objects: Breps,
 * PolyBreps, standalone Curves, and standalone Surfaces. Curves and Surfaces
 * owned by a Brep cannot be transformed independently; transform the owning
 * Brep instead. Unsupported SmObject types return SM_ERR_INVALID_INPUT.
 *
 * Non-uniform scaling is supported by Breps, PolyBreps, generic B-spline
 * geometry, and lines. This API permits only positive, uniform scale factors
 * for other specialized standalone Curve and Surface representations. All
 * scale factors must be finite and non-degenerate. PolyBrep scale factors must
 * additionally be positive.
 *
 * Rigid transforms preserve the Brep's geometry pointers (unlike a
 * representation-changing scale) and leave topology intact, so cached results
 * can be repositioned instead of recomputed for instanced geometry: transform a
 * cached PolyBrep's points/normals by the same matrix rather than
 * re-tessellating, and reuse mass properties (area, volume, mass and inertia
 * magnitudes are rigid-invariant; the centroid moves, and a *centroidal*
 * inertia tensor rotates -- a tensor about a fixed crOrigin also needs a
 * parallel-axis correction). SmApiTranslate and SmApiRotate are always rigid;
 * SmApiTransform is rigid only when its SmAxis2Placement is orthonormal (one
 * built via SetFrom4x4 can encode scale/shear, which invalidates this reuse).
 */
SMAPI_EXPORT SmApiStatus SmApiTransform(
    SmObject* pObj,                 ///< [in ]: Object to transform                    <br>
    const SmAxis2Placement & rRotateMove ///< [in ]: Transformation (rotate and translate) <br>
);

/// @warning A representation-changing Brep scale can replace and delete
/// analytic Surfaces and Curves owned by the Brep. Discard cached pointers to
/// Brep-owned geometry before this call and reacquire them from their surviving
/// Face or Edge afterward. Positive uniform Brep scales preserve those
/// geometry pointers.
SMAPI_EXPORT SmApiStatus SmApiScale(
    SmObject* pObj,                 ///< [in ]: Object to scale        <br>
    const SmVector3d& rScale       ///< [in ]: Scale factor in x, y, z <br>
);

/// @warning The same borrowed-geometry invalidation contract as SmApiScale
/// applies to representation-changing Brep scales about a point.
SMAPI_EXPORT SmApiStatus SmApiScaleByPt(
    SmObject* pObj,                 ///< [in ]: Object to scale                       <br>
    const SmVector3d& rRefPt,       ///< [in ]: Point about which to scale             <br>
    const SmVector3d& rScale       ///< [in ]: Scale factor in x, y, z                <br>
);

SMAPI_EXPORT SmApiStatus SmApiTranslate(
    SmObject* pObj,                 ///< [in ]: Object to translate <br>
    const SmVector3d& crTranslate  ///< [in ]: Translation vector  <br>
);

SMAPI_EXPORT SmApiStatus SmApiRotate(
    SmObject* pObj,                  ///< [in ]: Object to rotate                 <br>
    const SmVector3d& rRefPt,        ///< [in ]: Point on the rotation axis       <br>
    const SmVector3d& rAxis,         ///< [in ]: Rotation axis direction          <br>
    double dAngleDeg                ///< [in ]: Angle of rotation, in degrees    <br>
);

#endif
