// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdTessellate.h
* PURPOSE: Tessellate a loaded BrepArray prim into UsdGeomMesh-ready arrays.
*          Unlike the file-based SmApiUsd.h, this uses pxr types.
**********************************************************************/

#ifndef _SM_API_USD_TESSELLATE_H_
#define _SM_API_USD_TESSELLATE_H_

#include "SmApiUsdConfig.h"
#include <SmMessages.h>
#include <SmTypes.h>
#include <SmApiTessellationParams.h>

#include "UsdBrepHeaders.h" // pxr headers with warning suppression (BREP_USD_DATA)
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <string>
#include <vector>

/// Mesh data for one Brep of a BrepArray. A negative packed index marks a whole-prim
/// failure. Boundaries are kept even if the surface fails.
struct SmApiUsdTessellatedBrep
{
    std::int64_t                   m_iPackedBrepIndex = -1;
    SmStatus                       m_sStatus = SM_ERR;              ///< SM_SUCCESS when the mesh arrays are valid
    std::string                    m_sMessage;                      ///< why m_sStatus is not SM_SUCCESS
    pxr::VtIntArray                m_sFailedFaces;                  ///< GetFaces() indices of faces that failed
    SmStatus                       m_sBoundaryStatus = SM_SUCCESS;  ///< GetBoundaryData status
    pxr::VtArray<pxr::GfVec3f>     m_sBoundaryPoints;               ///< see GetBoundaryData
    pxr::VtIntArray                m_sBoundaryVertexCounts;
    pxr::VtArray<pxr::GfVec3f>     m_sExtent;                       ///< this and below: see GetTessellatedMeshData
    pxr::VtArray<pxr::GfVec3f>     m_sPoints;
    pxr::VtIntArray                m_sFaceVertexCounts;
    pxr::VtIntArray                m_sFaceVertexIndices;
    pxr::VtArray<pxr::GfVec3f>     m_sNormals;
    pxr::VtIntArray                m_sNormalsIndices;
    pxr::TfToken                   m_sNormalsInterpolation;
    pxr::TfToken                   m_sSubdivisionScheme;
    pxr::VtArray<pxr::VtIntArray>  m_sSubsetIndices;
    pxr::VtArray<pxr::SdfPath>     m_sSubsetMaterialPaths;
    pxr::SdfPath                   m_sMaterialPath;                 ///< bound by a "brep" GeomSubset; empty if the BrepArray's binding applies
};

/// Import, tessellate and extract each Brep of a BrepArray prim, one result per member in
/// packed order. Members fail independently; SmApiTessellate keeps partial meshes. Uses a
/// context private to the call, so concurrent calls are safe while the stage is not being
/// modified. Returns an error with one whole-prim result if the prim
/// cannot be read or has no Breps; malformed packed data stops at that member.
SM_API_USD_EXPORT SmStatus SmApiUsdTessellateBrepArray
(
    const pxr::UsdPrim                     & crBrepArray,          ///< [in ]: BrepArray prim
    const SmTessellationParams             & crParams,             ///< [in ]: tessellation quality controls
    SmBoolean                                bHealerIsEnabled,     ///< [in ]: run the healer while importing
    double                                   dBoundaryAngleTolDeg, ///< [in ]: edge sampling angle (deg), 0 = none
    std::vector<SmApiUsdTessellatedBrep>   & rResults              ///< [out]: one entry per member
);

#endif // _SM_API_USD_TESSELLATE_H_
