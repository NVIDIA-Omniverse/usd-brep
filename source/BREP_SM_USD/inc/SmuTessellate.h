// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuTessellate.h
* PURPOSE: Header file for tessellation of USD objects using SMLib
**********************************************************************/

#ifndef _SMU_TESSELLATE_H_
#define _SMU_TESSELLATE_H_

#include "SmuConfig.h"  // define: SMU_EXPORT

// pixar includes
#include <pxr/usd/usdGeom/xformOp.h>

// SMLib includes
#include "SmTypes.h"
#include <memory>

class SmBrep;
class SmPolyBrep;
class SmFace;
class SmSdfPathAttribute;
template<class TYPE> class SmTArray ;

namespace SMU_BrepConvert
{
    // Replace USD boundary arrays; finite, positive angle in degrees. Omit edges with fewer than two points.
    // On sampling failure, retain valid boundaries.
    SMU_EXPORT SmStatus GetBoundaryData
    (
        SmBrep* pBrep,
        double dAngleTolDeg,
        pxr::VtArray<pxr::GfVec3f>& rPoints,
        pxr::VtIntArray& rVertexCounts
    );

    // Get Mesh data from an SmPolyBrep, prepared for authoring UsdGeomMesh
    SMU_EXPORT SmStatus GetMeshData
    (
        SmPolyBrep                           & crSmPolyBrep,                 // in : Get mesh data from this SmPolyBrep                               
        pxr::VtArray<pxr::GfVec3f>           & rExtent,                      // out: Mesh Extent                                                          
        pxr::VtArray<pxr::GfVec3f>           & rPoints,                      // out: Mesh vertex locations                                               
        pxr::VtIntArray                      & rFaceVertexCounts,            // out: Number of vertices in each face of the mesh                         
        pxr::VtIntArray                      & rFaceVertexIndices,           // out: Flat list of the index of each vertex of each face in the mesh      
        pxr::VtArray<pxr::GfVec3f>           & rPrimvarNormals,              // out: Mesh vertex normals                                                 
        pxr::VtIntArray                      & rPrimvarNormalsIndices,       // out: Flat list of the index of each normal of each vertex in the mesh    
        pxr::TfToken                         & sPrimvarNormalsInterpolation, // out: face varying normals interpolation                                  
        pxr::TfToken                         & rSubdivisionScheme,           // out: None                                                                 
        pxr::VtArray<pxr::VtIntArray>        * pOptGeomSubsetIndices   = NULL,
        SmTArray<SmSdfPathAttribute*>        * pOptSubsetMaterialPaths = NULL
    );

    // Own a standalone tessellated mesh, i.e. ConvertToPolyBrep output.
    //
    // Deletion, not extraction: ~SmPolyBrep clears source vertex back-pointers while the
    // mesh's flag is on, but a standalone mesh's polyvertices point into the internal
    // tessellation copy, which ConvertToPolyBrep already destroyed before returning.
    // Keeping the caller's source Brep alive does not make that read safe. The deleter
    // therefore disables the flag first, so deletion is safe in either source/mesh order
    // and whatever the flag was left set to.
    //
    // Do not put a borrowed SmTess cache mesh in here: those are owned elsewhere and keep
    // their back-pointers on purpose.
    struct TessellatedMeshDeleter
    {
        SMU_EXPORT void operator()(SmPolyBrep* pMesh) const;
    };
    using TessellatedMeshPtr = std::unique_ptr<SmPolyBrep, TessellatedMeshDeleter>;

    // Extract USD mesh arrays; deep-copies material paths and restores the mesh's
    // back-pointer flag on exit.
    // The source Brep must stay alive AND topologically unmodified until extraction
    // returns: the mesh labels faces with SmFace pointers into the source, and nothing
    // tracks faces replaced or deleted after tessellation, so a mutator run in between
    // leaves labels that are read here for normals and material lookup.
    // That is the extraction contract and is separate from deletion, which
    // TessellatedMeshPtr handles.
    // rMaterialPaths pairs with rSubsetIndices; an unbound subset gives an empty path.
    // An empty mesh yields empty arrays and an empty extent, and still returns SM_SUCCESS.
    // Not thread-safe on a single mesh.
    SMU_EXPORT SmStatus GetTessellatedMeshData
    (
        SmPolyBrep& rMesh,
        pxr::VtArray<pxr::GfVec3f>& rExtent,
        pxr::VtArray<pxr::GfVec3f>& rPoints,
        pxr::VtIntArray& rFaceVertexCounts,
        pxr::VtIntArray& rFaceVertexIndices,
        pxr::VtArray<pxr::GfVec3f>& rNormals,
        pxr::VtIntArray& rNormalsIndices,
        pxr::TfToken& rNormalsInterpolation,
        pxr::TfToken& rSubdivisionScheme,
        pxr::VtArray<pxr::VtIntArray>& rSubsetIndices,
        pxr::VtArray<pxr::SdfPath>& rMaterialPaths
    );

    // Author precomputed mesh arrays (as from GetTessellatedMeshData) onto a UsdGeomMesh
    SMU_EXPORT SmStatus PopulateMeshAttr
    (
        const pxr::VtArray<pxr::GfVec3f>    & crExtent,
        const pxr::VtArray<pxr::GfVec3f>    & crPoints,
        const pxr::VtIntArray               & crFaceVertexCounts,
        const pxr::VtIntArray               & crFaceVertexIndices,
        const pxr::VtArray<pxr::GfVec3f>    & crNormals,
        const pxr::VtIntArray               & crNormalsIndices,
        const pxr::TfToken                  & crNormalsInterpolation,
        const pxr::TfToken                  & crSubdivisionScheme,
        const pxr::VtArray<pxr::VtIntArray> & crSubsetIndices,
        const pxr::VtArray<pxr::SdfPath>    & crMaterialPaths, // pairs with crSubsetIndices; empty = unbound
        const pxr::SdfPrimSpecHandle        & rPrimSpecHandle
    );

    // Convert an SmPolyBrep to a UsdGeomMesh
    SMU_EXPORT SmStatus PopulateMeshAttr
    (
        SmPolyBrep                       & crSmPolyBrep,   // in : PolyBrep being converted to a UsdGeomMesh      
        const pxr::SdfPrimSpecHandle     & rPrimSpecHandle // in : PrimSpecHandle of the UsdGeomMesh     
    );

} // end namespace SMU_BrepConvert

#endif // no _SMU_TESSELLATE_H_
