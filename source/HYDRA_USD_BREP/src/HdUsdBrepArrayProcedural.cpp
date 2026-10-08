// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HdUsdBrepImagingHeaders.h"

#include "HdUsdBrepArrayProcedural.h"
#include "HdUsdBrepTokens.h"

#include "pxr/pxr.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/type.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/basisCurvesSchema.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/pxOsd/tokens.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/imaging/hd/purposeSchema.h"
#include "pxr/imaging/hd/instancedBySchema.h"
#include "pxr/imaging/hd/materialBindingsSchema.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/geomSubsetSchema.h"
#include "pxr/imaging/hd/primOriginSchema.h"
#include <algorithm>
#include "pxr/imaging/hdGp/generativeProceduralPluginRegistry.h"

PXR_NAMESPACE_OPEN_SCOPE

namespace
{
HdContainerDataSourceHandle _SourceOrigin(const HdSceneIndexBaseRefPtr& scene, const SdfPath& path)
{
    if (scene)
    {
        auto source = scene->GetPrim(path).dataSource;
        auto origin = HdPrimOriginSchema::GetFromParent(source);
        if (origin)
        {
            return origin.GetContainer();
        }
    }
    return HdRetainedContainerDataSource::New(
        HdPrimOriginSchemaTokens->scenePath,
        HdRetainedTypedSampledDataSource<HdPrimOriginSchema::OriginPath>::New(HdPrimOriginSchema::OriginPath(path))
    );
}

// Child prim names: <prefix><index>.
const std::string kEdgesPrefix = "brep_edges_";
const std::string kMeshPrefix = "tessellated_mesh_";
const std::string kSubsetPrefix = "material_";

// Parse the index from a child prim name "<prefix><index>".
bool _ParseIndex(const std::string& name, const std::string& prefix, size_t& index)
{
    if (!TfStringStartsWith(name, prefix))
    {
        return false;
    }
    try
    {
        index = std::stoul(name.substr(prefix.size()));
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// Read a typed value from a container; leaves value unchanged if absent.
template <typename T>
void _Read(const HdContainerDataSourceHandle& container, const TfToken& name, T& value)
{
    if (auto ds = HdTypedSampledDataSource<T>::Cast(container ? container->Get(name) : nullptr))
    {
        value = ds->GetTypedValue(0.0f);
    }
}

// First element of a primvar authored on source; leaves value unchanged if absent or empty.
template <typename T>
bool _AuthoredFirst(const HdContainerDataSourceHandle& source, const TfToken& name, T& value)
{
    if (auto valueSource = HdPrimvarsSchema::GetFromParent(source).GetPrimvar(name).GetPrimvarValue())
    {
        const VtValue authored = valueSource->GetValue(0);
        if (authored.IsHolding<VtArray<T>>() && !authored.UncheckedGet<VtArray<T>>().empty())
        {
            value = authored.UncheckedGet<VtArray<T>>()[0];
            return true;
        }
    }
    return false;
}

template <typename T>
HdContainerDataSourceHandle _Primvar(const VtArray<T>& value, const TfToken& interpolation, const TfToken& role)
{
    return HdRetainedContainerDataSource::New(
        HdPrimvarSchemaTokens->primvarValue,
        HdRetainedTypedSampledDataSource<VtArray<T>>::New(value),
        HdPrimvarSchemaTokens->interpolation,
        HdRetainedTypedSampledDataSource<TfToken>::New(interpolation),
        HdPrimvarSchemaTokens->role,
        HdRetainedTypedSampledDataSource<TfToken>::New(role)
    );
}
} // namespace

// --------------------------------------------------------------------------
// HdUsdBrepArrayProcedural
// --------------------------------------------------------------------------

HdUsdBrepArrayProcedural::HdUsdBrepArrayProcedural(const SdfPath& proceduralPrimPath) : HdGpGenerativeProcedural(proceduralPrimPath)
{
}

HdGpGenerativeProcedural::DependencyMap HdUsdBrepArrayProcedural::UpdateDependencies(const HdSceneIndexBaseRefPtr& /*inputScene*/)
{
    DependencyMap deps;
    // Children carry source state too, so any source edit dirties them.
    deps[_GetProceduralPrimPath()] = HdDataSourceLocatorSet::UniversalSet();
    return deps;
}

void HdUsdBrepArrayProcedural::_ReadMeshData(const HdSceneIndexBaseRefPtr& inputScene)
{
    _meshes.clear();

    const SdfPath primPath = _GetProceduralPrimPath();
    HdSceneIndexPrim prim = inputScene->GetPrim(primPath);

    if (!prim.dataSource)
    {
        TF_WARN("HdUsdBrepArrayProcedural: No data source for '%s'", primPath.GetText());
        return;
    }

    // Tessellation data from the adapter
    HdContainerDataSourceHandle tessDs = HdContainerDataSource::Cast(prim.dataSource->Get(HdUsdBrepTokens->usdBrepTessellatorData));

    if (!tessDs)
    {
        TF_WARN(
            "HdUsdBrepArrayProcedural: No tessellation data for '%s'. "
            "BrepArrayAdapter may not have injected it.",
            primPath.GetText()
        );
        return;
    }

    // One entry per Brep, keyed by packed index; see _BuildTessellationDataSource.
    for (const TfToken& name : tessDs->GetNames())
    {
        _MeshData mesh;
        const HdContainerDataSourceHandle brepDs = HdContainerDataSource::Cast(tessDs->Get(name));
        if (!brepDs || !_ParseIndex(name.GetString(), std::string(), mesh.brepIndex))
        {
            TF_WARN("HdUsdBrepArrayProcedural: Bad Brep entry '%s' for '%s'", name.GetText(), primPath.GetText());
            continue;
        }

        const HdContainerDataSourceHandle meshDs = HdContainerDataSource::Cast(brepDs->Get(HdPrimTypeTokens->mesh));
        _Read(meshDs, HdTokens->points, mesh.points);
        _Read(meshDs, HdMeshTopologySchemaTokens->faceVertexCounts, mesh.faceVertexCounts);
        _Read(meshDs, HdMeshTopologySchemaTokens->faceVertexIndices, mesh.faceVertexIndices);
        _Read(meshDs, HdTokens->normals, mesh.normals);

        const HdContainerDataSourceHandle subsetDs = HdContainerDataSource::Cast(brepDs->Get(HdPrimTypeTokens->geomSubset));
        _Read(subsetDs, HdGeomSubsetSchemaTokens->indices, mesh.subsetIndices);
        _Read(subsetDs, HdUsdBrepTokens->materialPaths, mesh.subsetMaterials);
        _Read(subsetDs, HdUsdBrepTokens->brepMaterialPath, mesh.brepMaterial);

        const HdContainerDataSourceHandle curvesDs = HdContainerDataSource::Cast(brepDs->Get(HdPrimTypeTokens->basisCurves));
        _Read(curvesDs, HdTokens->points, mesh.edgePoints);
        _Read(curvesDs, HdBasisCurvesTopologySchemaTokens->curveVertexCounts, mesh.edgeVertexCounts);

        if ((!mesh.points.empty() && !mesh.faceVertexIndices.empty()) || !mesh.edgePoints.empty())
        {
            _meshes.push_back(std::move(mesh));
        }
    }
}

HdGpGenerativeProcedural::ChildPrimTypeMap HdUsdBrepArrayProcedural::Update(
    const HdSceneIndexBaseRefPtr& inputScene,
    const ChildPrimTypeMap& previousResult,
    const DependencyMap& dirtiedDependencies,
    HdSceneIndexObserver::DirtiedPrimEntries* outputDirtiedPrims
)
{
    std::lock_guard<std::mutex> lock(_mutex);

    // Re-tessellate only if the tessellation changed; forward other edits to the children.
    HdDataSourceLocatorSet childDirty;
    for (const auto& [path, locators] : dirtiedDependencies)
    {
        childDirty.insert(locators);
    }
    if (!_cooked || childDirty.Intersects(HdDataSourceLocator(HdUsdBrepTokens->usdBrepTessellatorData)))
    {
        _ReadMeshData(inputScene);
        _cooked = true;
        childDirty = HdDataSourceLocatorSet::UniversalSet();
    }

    ChildPrimTypeMap result;
    for (size_t i = 0; i < _meshes.size(); ++i)
    {
        if (!_meshes[i].edgePoints.empty())
        {
            result[_GetProceduralPrimPath().AppendChild(TfToken(kEdgesPrefix + std::to_string(_meshes[i].brepIndex)))] = HdPrimTypeTokens->basisCurves;
        }
        if (_meshes[i].points.empty() || _meshes[i].faceVertexIndices.empty())
        {
            continue;
        }
        SdfPath childPath = _GetProceduralPrimPath().AppendChild(TfToken(kMeshPrefix + std::to_string(_meshes[i].brepIndex)));
        result[childPath] = HdPrimTypeTokens->mesh;
        for (size_t j = 0; j < _meshes[i].subsetIndices.size(); ++j)
        {
            result[childPath.AppendChild(TfToken(kSubsetPrefix + std::to_string(j)))] = HdPrimTypeTokens->geomSubset;
        }
    }

    if (!previousResult.empty() && outputDirtiedPrims && !childDirty.IsEmpty())
    {
        for (const auto& [path, type] : result)
        {
            outputDirtiedPrims->emplace_back(path, childDirty);
        }
    }

    return result;
}

HdSceneIndexPrim HdUsdBrepArrayProcedural::GetChildPrim(const HdSceneIndexBaseRefPtr& inputScene, const SdfPath& childPrimPath)
{
    std::lock_guard<std::mutex> lock(_mutex);

    // Parse mesh index from child name
    const bool isSubset = childPrimPath.GetParentPath() != _GetProceduralPrimPath();
    const SdfPath meshPath = isSubset ? childPrimPath.GetParentPath() : childPrimPath;
    const std::string childName = meshPath.GetName();
    const bool isEdges = TfStringStartsWith(childName, kEdgesPrefix);
    size_t meshIdx = 0;
    if (!_ParseIndex(childName, isEdges ? kEdgesPrefix : kMeshPrefix, meshIdx))
    {
        return {};
    }
    auto found = std::find_if(
        _meshes.begin(),
        _meshes.end(),
        [meshIdx](const _MeshData& m)
        {
            return m.brepIndex == meshIdx;
        }
    );
    if (found == _meshes.end())
    {
        return {};
    }
    const _MeshData& mesh = *found;
    HdSceneIndexPrim result;
    result.primType = HdPrimTypeTokens->mesh;
    if (isSubset)
    {
        size_t subsetIdx = 0;
        if (!_ParseIndex(childPrimPath.GetName(), kSubsetPrefix, subsetIdx) || subsetIdx >= mesh.subsetIndices.size() ||
            subsetIdx >= mesh.subsetMaterials.size())
        {
            return {};
        }
        result.primType = HdPrimTypeTokens->geomSubset;
        auto binding = HdMaterialBindingSchema::Builder()
                           .SetPath(HdRetainedTypedSampledDataSource<SdfPath>::New(mesh.subsetMaterials[subsetIdx]))
                           .Build();
        result.dataSource = HdRetainedContainerDataSource::New(
            HdGeomSubsetSchemaTokens->geomSubset,
            HdGeomSubsetSchema::Builder()
                .SetType(HdRetainedTypedSampledDataSource<TfToken>::New(HdGeomSubsetSchemaTokens->typeFaceSet))
                .SetIndices(HdRetainedTypedSampledDataSource<VtIntArray>::New(mesh.subsetIndices[subsetIdx]))
                .Build(),
            HdMaterialBindingsSchemaTokens->materialBindings,
            HdRetainedContainerDataSource::New(TfToken(), binding),
            HdPrimOriginSchemaTokens->primOrigin,
            _SourceOrigin(inputScene, _GetProceduralPrimPath())
        );
        return result;
    }

    const auto source = inputScene ? inputScene->GetPrim(_GetProceduralPrimPath()).dataSource : nullptr;
    HdContainerDataSourceHandle meshDs;
    if (isEdges)
    {
        result.primType = HdPrimTypeTokens->basisCurves;
        meshDs = HdBasisCurvesSchema::Builder()
                     .SetTopology(HdBasisCurvesTopologySchema::Builder()
                                      .SetCurveVertexCounts(HdRetainedTypedSampledDataSource<VtIntArray>::New(mesh.edgeVertexCounts))
                                      .SetType(HdRetainedTypedSampledDataSource<TfToken>::New(HdTokens->linear))
                                      .SetBasis(HdRetainedTypedSampledDataSource<TfToken>::New(HdTokens->bezier))
                                      .SetWrap(HdRetainedTypedSampledDataSource<TfToken>::New(HdTokens->nonperiodic))
                                      .Build())
                     .Build();
    }
    else
    {
        // Build mesh topology
        HdContainerDataSourceHandle topologyDs = HdMeshTopologySchema::Builder()
                                                     .SetFaceVertexCounts(HdRetainedTypedSampledDataSource<VtArray<int>>::New(mesh.faceVertexCounts))
                                                     .SetFaceVertexIndices(HdRetainedTypedSampledDataSource<VtArray<int>>::New(mesh.faceVertexIndices)
                                                     )
                                                     .Build();

        meshDs = HdMeshSchema::Builder()
                     .SetTopology(topologyDs)
                     // Triangles: "none" stops Storm subdividing them and dropping the normals.
                     .SetSubdivisionScheme(HdRetainedTypedSampledDataSource<TfToken>::New(PxOsdOpenSubdivTokens->none))
                     // The adapter exposes the source's doubleSided on its mesh schema.
                     .SetDoubleSided(HdMeshSchema::GetFromParent(source).GetDoubleSided())
                     .Build();
    }

    // displayColor: the source's authored constant value if any, else light grey.
    VtArray<GfVec3f> displayColor(1, GfVec3f(0.85f, 0.87f, 0.9f));
    _AuthoredFirst(source, HdTokens->displayColor, displayColor[0]);
    std::vector<TfToken> primvarNames = { HdTokens->points, HdTokens->displayColor };
    std::vector<HdDataSourceBaseHandle> primvarValues = {
        _Primvar(isEdges ? mesh.edgePoints : mesh.points, HdPrimvarSchemaTokens->vertex, HdPrimvarSchemaTokens->point),
        _Primvar(displayColor, HdPrimvarSchemaTokens->constant, HdPrimvarSchemaTokens->color)
    };
    // displayOpacity: the source's authored constant value if any; absent means opaque.
    VtArray<float> displayOpacity(1);
    if (_AuthoredFirst(source, HdTokens->displayOpacity, displayOpacity[0]))
    {
        primvarNames.push_back(HdTokens->displayOpacity);
        primvarValues.push_back(_Primvar(displayOpacity, HdPrimvarSchemaTokens->constant, TfToken()));
    }
    if (!isEdges && !mesh.normals.empty())
    {
        primvarNames.push_back(HdTokens->normals);
        primvarValues.push_back(_Primvar(mesh.normals, HdPrimvarSchemaTokens->faceVarying, HdPrimvarSchemaTokens->normal));
    }
    HdContainerDataSourceHandle primvarsDs = HdRetainedContainerDataSource::New(primvarNames.size(), primvarNames.data(), primvarValues.data());

    // Children must carry the source's resolved state, including the transform.
    std::vector<TfToken> names = { isEdges ? HdBasisCurvesSchemaTokens->basisCurves : HdMeshSchemaTokens->mesh,
                                   HdPrimvarsSchemaTokens->primvars,
                                   HdPrimOriginSchemaTokens->primOrigin,
                                   HdUsdBrepTokens->usdBrepEdges };
    std::vector<HdDataSourceBaseHandle> values = { meshDs,
                                                   primvarsDs,
                                                   _SourceOrigin(inputScene, _GetProceduralPrimPath()),
                                                   HdRetainedTypedSampledDataSource<bool>::New(isEdges) };
    if (source)
    {
        for (const TfToken& name : { HdInstancedBySchemaTokens->instancedBy,
                                     HdXformSchemaTokens->xform,
                                     HdVisibilitySchemaTokens->visibility,
                                     HdPurposeSchemaTokens->purpose,
                                     HdMaterialBindingsSchemaTokens->materialBindings })
        {
            if (isEdges && name == HdMaterialBindingsSchemaTokens->materialBindings)
            {
                continue;
            }
            HdDataSourceBaseHandle data = source->Get(name);
            // A "brep" GeomSubset binding is this mesh's all-purpose binding; as in USD, the
            // BrepArray's purpose-specific bindings still win, and face subsets override both.
            if (name == HdMaterialBindingsSchemaTokens->materialBindings && !mesh.brepMaterial.IsEmpty())
            {
                data = HdOverlayContainerDataSource::OverlayedContainerDataSources(
                    HdRetainedContainerDataSource::New(
                        HdMaterialBindingsSchemaTokens->allPurpose,
                        HdMaterialBindingSchema::Builder().SetPath(HdRetainedTypedSampledDataSource<SdfPath>::New(mesh.brepMaterial)).Build()
                    ),
                    HdContainerDataSource::Cast(data)
                );
            }
            if (data)
            {
                names.push_back(name);
                values.push_back(data);
            }
        }
    }
    result.dataSource = HdRetainedContainerDataSource::New(names.size(), names.data(), values.data());
    return result;
}

// --------------------------------------------------------------------------
// HdUsdBrepArrayProceduralPlugin
// --------------------------------------------------------------------------

HdGpGenerativeProcedural* HdUsdBrepArrayProceduralPlugin::Construct(const SdfPath& proceduralPrimPath)
{
    return new HdUsdBrepArrayProcedural(proceduralPrimPath);
}

// --------------------------------------------------------------------------
// Plugin Registration
// --------------------------------------------------------------------------

TF_REGISTRY_FUNCTION(TfType)
{
    HdGpGenerativeProceduralPluginRegistry::Define<HdUsdBrepArrayProceduralPlugin, HdGpGenerativeProceduralPlugin>();
}

PXR_NAMESPACE_CLOSE_SCOPE
