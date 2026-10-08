// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HD_USD_BREP_ARRAY_PROCEDURAL_H
#define HD_USD_BREP_ARRAY_PROCEDURAL_H

#include "HdUsdBrepApi.h"

#include "UsdBrepHeaders.h"
#include "pxr/pxr.h"
#include "pxr/imaging/hdGp/generativeProcedural.h"
#include "pxr/imaging/hdGp/generativeProceduralPlugin.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"

#include <mutex>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

// --------------------------------------------------------------------------
// HdGpGenerativeProcedural subclass: BrepArray meshes
// --------------------------------------------------------------------------

/// \class HdUsdBrepArrayProcedural
///
/// Emits the adapter's tessellation of a BrepArray as child mesh, boundary-curve
/// and subset prims that carry the source's transform, visibility, purpose and bindings.
///
class HdUsdBrepArrayProcedural : public HdGpGenerativeProcedural
{
public:

    HDUSDBREP_API HdUsdBrepArrayProcedural(const SdfPath& proceduralPrimPath);

    // Depends on all of the source prim's data.
    HDUSDBREP_API DependencyMap UpdateDependencies(const HdSceneIndexBaseRefPtr& inputScene) override;

    // Read the mesh data and return the child prims.
    HDUSDBREP_API ChildPrimTypeMap Update(
        const HdSceneIndexBaseRefPtr& inputScene,
        const ChildPrimTypeMap& previousResult,
        const DependencyMap& dirtiedDependencies,
        HdSceneIndexObserver::DirtiedPrimEntries* outputDirtiedPrims
    ) override;

    // Data source for a child mesh, curve or subset prim.
    HDUSDBREP_API HdSceneIndexPrim GetChildPrim(const HdSceneIndexBaseRefPtr& inputScene, const SdfPath& childPrimPath) override;

private:

    struct _MeshData
    {
        size_t brepIndex = 0;
        VtArray<GfVec3f> edgePoints;
        VtIntArray edgeVertexCounts;
        VtArray<GfVec3f> points;
        VtArray<int> faceVertexCounts;
        VtArray<int> faceVertexIndices;
        VtArray<GfVec3f> normals;
        VtArray<VtIntArray> subsetIndices;
        VtArray<SdfPath> subsetMaterials;
        SdfPath brepMaterial; // empty: the BrepArray's binding applies
    };

    std::vector<_MeshData> _meshes;
    bool _cooked = false;
    std::mutex _mutex; // Guards _meshes.

    // Read the adapter's usdBrepTessellatorData container into _meshes.
    void _ReadMeshData(const HdSceneIndexBaseRefPtr& inputScene);
};

// --------------------------------------------------------------------------
// Plugin class
// --------------------------------------------------------------------------

/// \class HdUsdBrepArrayProceduralPlugin
///
/// Factory for HdUsdBrepArrayProcedural, registered in plugInfo.json.
///
class HdUsdBrepArrayProceduralPlugin : public HdGpGenerativeProceduralPlugin
{
public:

    HDUSDBREP_API HdGpGenerativeProcedural* Construct(const SdfPath& proceduralPrimPath) override;
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif // HD_USD_BREP_ARRAY_PROCEDURAL_H
