// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiPolygons.h

PURPOSE: 
    Contains high level "C" type functions that operate on SmPolyBreps.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmMerge, etc
**********************************************************************/


#ifndef __SmApiPolygons_H__
#define __SmApiPolygons_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmPolyMerge.h>

class SmContext;
class SmBrep;
class SmPolyBrep;


/// Ownership contract for the PolyBrep Booleans: pPolyBrep1 is modified in place and
/// returned as rpResult; pPolyBrep2 is deleted on success, except for
/// SM_PBO_PARTIAL_MERGE. SM_PBO_IMPRINT does delete pPolyBrep2. A NULL operand or
/// pPolyBrep1 == pPolyBrep2 returns SM_ERR_INVALID_INPUT with both inputs untouched.
SMAPI_EXPORT SmApiStatus SmApiPolyBoolean( 
    SmPolyBrep*  pPolyBrep1,                  ///< [in/out]: Primary brep; modified and returned as rpResult            <br>
    SmPolyBrep*  pPolyBrep2,                  ///< [in/out]: Second brep; deleted on success (see contract above)       <br>
    SmPolyBooleanOperationType lOperation,    ///< [in ]: specify boolean operation                                     <br>
    SmPolyBrep*& rpResult                    ///< [out]: Resulting SmBrep                                              <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBooleanUnion( 
    SmPolyBrep*  pPolyBrep1,                  ///< [in/out]: Primary brep; modified and returned as rpResult            <br>
    SmPolyBrep*  pPolyBrep2,                  ///< [in/out]: Second brep; deleted on success (see contract above)       <br>
    SmPolyBrep*& rpResult                    ///< [out]: Resulting SmBrep                                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBooleanDifference( 
    SmPolyBrep*  pPolyBrep1,                  ///< [in/out]: Primary brep; modified and returned as rpResult            <br>
    SmPolyBrep*  pPolyBrep2,                  ///< [in/out]: Second brep; deleted on success (see contract above)       <br>
    SmPolyBrep*& rpResult                    ///< [out]: Resulting SmBrep                                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBooleanIntersection( 
    SmPolyBrep*  pPolyBrep1,                  ///< [in/out]: Primary brep; modified and returned as rpResult            <br>
    SmPolyBrep*  pPolyBrep2,                  ///< [in/out]: Second brep; deleted on success (see contract above)       <br>
    SmPolyBrep*& rpResult                    ///< [out]: Resulting SmBrep                                               <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBooleanMerge( 
    SmPolyBrep*  pPolyBrep1,                  ///< [in/out]: Primary brep; modified and returned as rpResult            <br>
    SmPolyBrep*  pPolyBrep2,                  ///< [in/out]: Second brep; deleted on success (see contract above)       <br>
    SmPolyBrep*& rpResult                    ///< [out]: Resulting SmBrep                                              <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBrepIsManifoldSolid(
    const SmPolyBrep* pPolyBrep,          ///< [in ]: Pointer to poly brep                                         <br>
    SmBoolean & rbIsManifoldSolid         ///< [out]: TRUE = manifold solid (all edges manifold), FALSE = not      <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBrepComputeVolume(
    const SmPolyBrep*  pPolyBrep,                     ///< [in ]: Pointer to poly brep                                         <br>
    double     & rdVolume                              ///< [out]: Enclosed volume                                               <br>
);

/// Mesh counterpart of SmApiBrepComputeMassProperties: same outputs and
/// conventions, computed exactly from the mesh's triangles (one pass, no
/// accuracy parameter -- accuracy is that of the tessellation).  Independent
/// of winding direction: a closed mesh wound inside-out (e.g. as imported)
/// gives the same result as its correctly wound counterpart.  Returns
/// SM_ERR_INVALID_INPUT for a non-finite / sub-SM_EFF_ZERO density, a
/// non-finite origin, a mesh that is not a closed manifold solid, or a
/// zero-volume result.
SMAPI_EXPORT SmApiStatus SmApiPolyBrepComputeMassProperties(
    const SmPolyBrep* pPolyBrep,          ///< [in ]: Mesh to query (closed manifold solid; read-only)              <br>
    double       dDensity,                 ///< [in ]: Uniform mass density (mass = density * volume); must be a      <br>
                                           ///<        valid double >= SM_EFF_ZERO (1e-12)                            <br>
    const SmPoint3d & crOrigin,            ///< [in ]: Origin the moments/products are taken about (axes parallel    <br>
                                           ///<        to world axes); pass the centroid for centroidal inertia.     <br>
                                           ///<        Must be finite                                                 <br>
    double &     rdArea,                   ///< [out]: Total surface area                                            <br>
    double &     rdVolume,                 ///< [out]: Enclosed volume                                               <br>
    double &     rdMass,                   ///< [out]: Mass = dDensity * volume                                      <br>
    SmPoint3d &  rCentroid,                ///< [out]: Mass centroid in world coordinates; independent of crOrigin   <br>
    SmVector3d & rMomentsOfInertia,        ///< [out]: Mass moments of inertia (Ixx, Iyy, Izz) about axes through    <br>
                                           ///<        crOrigin parallel to the world axes                           <br>
    SmVector3d & rProductsOfInertia        ///< [out]: Mass products of inertia about crOrigin as raw positive       <br>
                                           ///<        integrals (Iyz, Izx, Ixy); tensor off-diagonals are negatives <br>
);

SMAPI_EXPORT SmApiStatus SmApiPolyBrepCopy(
    const SmPolyBrep*  pPolyBrep,               ///< [in ]: Source PolyBrep (not modified)                                <br>
    SmPolyBrep*& rpResult                 ///< [out]: Newly allocated deep copy in the SM_API context               <br>
);


#endif
