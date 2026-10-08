// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HdUsdBrepArrayAdapter.h" // first: brings HdUsdBrepImagingHeaders.h

#include "SmApiTessellationParams.h"
#include "SmApiUsdTessellate.h"
#include "HdUsdBrepTokens.h"

#include "pxr/pxr.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/geomSubsetSchema.h"
#include "pxr/imaging/hd/lazyContainerDataSource.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/usdImaging/usdImaging/tokens.h"
#include <algorithm>

PXR_NAMESPACE_OPEN_SCOPE

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    // USD 25.11 does not export HdGpGenerativeProceduralTokens on Windows.
    ((generativeProcedural, "hydraGenerativeProcedural"))
    ((proceduralType, "hdGp:proceduralType"))
    (usdBrepTessellation)
);

TF_DEFINE_PUBLIC_TOKENS(HdUsdBrepTokens, HD_USD_BREP_TOKENS);

TF_REGISTRY_FUNCTION(TfType)
{
    using Adapter = HdUsdBrepArrayAdapter;
    TfType t = TfType::Define<Adapter, TfType::Bases<Adapter::BaseAdapter>>();
    t.SetFactory<UsdImagingPrimAdapterFactory<Adapter>>();
}

// Angular tolerance, in degrees, for sampling the BRep edge wireframe.
static constexpr double kBoundaryAngleTolDeg = 5.0;

// Tessellate prim for the procedural, which then needs no stage access.
static HdContainerDataSourceHandle _BuildTessellationDataSource(const UsdPrim& prim)
{
    // The healer stays off to preserve the authored topology during import.
    std::vector<SmApiUsdTessellatedBrep> results;
    SmApiUsdTessellateBrepArray(prim, SmTessellationParams(), FALSE, kBoundaryAngleTolDeg, results);

    // One container per Brep with a mesh or boundary curves, keyed by packed index:
    // <index>/mesh/{points, faceVertexCounts, faceVertexIndices, normals}
    // <index>/geomSubset/{indices, materialPaths, brepMaterialPath}
    // <index>/basisCurves/{points, curveVertexCounts}
    std::vector<TfToken> brepNames;
    std::vector<HdDataSourceBaseHandle> brepDataSources;
    for (const SmApiUsdTessellatedBrep& result : results)
    {
        const bool success = result.m_sStatus == SM_SUCCESS;
        if (result.m_sBoundaryStatus != SM_SUCCESS)
        {
            TF_WARN("BrepArray '%s', body %lld: could not sample some edges", prim.GetPath().GetText(), (long long)result.m_iPackedBrepIndex);
        }
        if (!success)
        {
            TF_WARN("BrepArray '%s', body %lld: %s", prim.GetPath().GetText(), (long long)result.m_iPackedBrepIndex, result.m_sMessage.c_str());
        }
        else if (!result.m_sFailedFaces.empty())
        {
            std::string faceIndices;
            for (const int faceIndex : result.m_sFailedFaces)
            {
                faceIndices += (faceIndices.empty() ? "" : ", ") + std::to_string(faceIndex);
            }
            TF_WARN(
                "BrepArray '%s', body %lld: %zu faces failed to tessellate (indices: %s); displaying partial mesh",
                prim.GetPath().GetText(),
                (long long)result.m_iPackedBrepIndex,
                result.m_sFailedFaces.size(),
                faceIndices.c_str()
            );
        }

        std::vector<TfToken> names;
        std::vector<HdDataSourceBaseHandle> values;
        if (success && !result.m_sPoints.empty())
        {
            std::vector<TfToken> meshNames = { HdTokens->points,
                                               HdMeshTopologySchemaTokens->faceVertexCounts,
                                               HdMeshTopologySchemaTokens->faceVertexIndices };
            std::vector<HdDataSourceBaseHandle> meshValues = { HdRetainedTypedSampledDataSource<VtArray<GfVec3f>>::New(result.m_sPoints),
                                                               HdRetainedTypedSampledDataSource<VtIntArray>::New(result.m_sFaceVertexCounts),
                                                               HdRetainedTypedSampledDataSource<VtIntArray>::New(result.m_sFaceVertexIndices) };
            if (!result.m_sNormals.empty())
            {
                // Normals are indexed per face corner; expand them to face-varying.
                VtArray<GfVec3f> cornerNormals(result.m_sNormalsIndices.size());
                for (size_t j = 0; j < result.m_sNormalsIndices.size(); ++j)
                {
                    cornerNormals[j] = result.m_sNormals[result.m_sNormalsIndices[j]];
                }
                meshNames.push_back(HdTokens->normals);
                meshValues.push_back(HdRetainedTypedSampledDataSource<VtArray<GfVec3f>>::New(cornerNormals));
            }
            names.push_back(HdPrimTypeTokens->mesh);
            values.push_back(HdRetainedContainerDataSource::New(meshNames.size(), meshNames.data(), meshValues.data()));
            names.push_back(HdPrimTypeTokens->geomSubset);
            values.push_back(HdRetainedContainerDataSource::New(
                HdGeomSubsetSchemaTokens->indices,
                HdRetainedTypedSampledDataSource<VtArray<VtIntArray>>::New(result.m_sSubsetIndices),
                HdUsdBrepTokens->materialPaths,
                HdRetainedTypedSampledDataSource<VtArray<SdfPath>>::New(result.m_sSubsetMaterialPaths),
                HdUsdBrepTokens->brepMaterialPath,
                HdRetainedTypedSampledDataSource<SdfPath>::New(result.m_sMaterialPath)
            ));
        }
        if (!result.m_sBoundaryPoints.empty())
        {
            names.push_back(HdPrimTypeTokens->basisCurves);
            values.push_back(HdRetainedContainerDataSource::New(
                HdTokens->points,
                HdRetainedTypedSampledDataSource<VtArray<GfVec3f>>::New(result.m_sBoundaryPoints),
                HdBasisCurvesTopologySchemaTokens->curveVertexCounts,
                HdRetainedTypedSampledDataSource<VtIntArray>::New(result.m_sBoundaryVertexCounts)
            ));
        }
        if (!names.empty())
        {
            brepNames.push_back(TfToken(std::to_string(result.m_iPackedBrepIndex)));
            brepDataSources.push_back(HdRetainedContainerDataSource::New(names.size(), names.data(), values.data()));
        }
    }

    if (brepNames.empty())
    {
        TF_WARN("BrepArrayAdapter: No tessellation results for '%s'", prim.GetPath().GetText());
        return HdRetainedContainerDataSource::New();
    }
    return HdRetainedContainerDataSource::New(brepNames.size(), brepNames.data(), brepDataSources.data());
}

// ----------------------------------------------------------------------------
// Scene index path
// ----------------------------------------------------------------------------

// The BrepArray's GeomSubset children are read during tessellation. Populated as Hydra prims,
// they would name the procedural as their mesh, which Storm rejects.
UsdImagingPrimAdapter::PopulationMode HdUsdBrepArrayAdapter::GetPopulationMode()
{
    return RepresentsSelfAndDescendents;
}

TfTokenVector HdUsdBrepArrayAdapter::GetImagingSubprims(UsdPrim const& /*prim*/)
{
    return { TfToken() };
}

TfToken HdUsdBrepArrayAdapter::GetImagingSubprimType(UsdPrim const& /*prim*/, TfToken const& subprim)
{
    if (subprim.IsEmpty())
    {
        return _tokens->generativeProcedural;
    }
    return TfToken();
}

HdContainerDataSourceHandle HdUsdBrepArrayAdapter::GetImagingSubprimData(
    UsdPrim const& prim,
    TfToken const& subprim,
    const UsdImagingDataSourceStageGlobals& stageGlobals
)
{
    if (!subprim.IsEmpty())
    {
        return nullptr;
    }

    // Base prim data (xform, visibility, purpose, etc.)
    HdContainerDataSourceHandle baseDs = UsdImagingDataSourceGprim::New(prim.GetPath(), prim, stageGlobals);

    // Synthesize hdGp:proceduralType primvar
    HdContainerDataSourceHandle procTypePrimvar = HdRetainedContainerDataSource::New(
        HdPrimvarSchemaTokens->primvarValue,
        HdRetainedTypedSampledDataSource<TfToken>::New(_tokens->usdBrepTessellation),
        HdPrimvarSchemaTokens->interpolation,
        HdRetainedTypedSampledDataSource<TfToken>::New(HdPrimvarSchemaTokens->constant),
        HdPrimvarSchemaTokens->role,
        HdRetainedTypedSampledDataSource<TfToken>::New(TfToken())
    );


    // Tessellated on first read: scene indices fetch the prim several times while loading,
    // but only the procedural reads this.
    HdContainerDataSourceHandle tessData = HdLazyContainerDataSource::New(
        [prim]()
        {
            return _BuildTessellationDataSource(prim);
        }
    );

    // doubleSided is a mesh schema field, which the Gprim base does not provide; the
    // procedural forwards it to the generated meshes.
    HdContainerDataSourceHandle meshDs = HdMeshSchema::Builder()
                                             .SetDoubleSided(
                                                 UsdImagingDataSourceAttribute<bool>::New(UsdGeomGprim(prim).GetDoubleSidedAttr(), stageGlobals)
                                             )
                                             .Build();

    // The overlay merges this primvar into the base prim's primvars.
    return HdOverlayContainerDataSource::New(
        HdRetainedContainerDataSource::New(
            HdPrimvarsSchemaTokens->primvars,
            HdRetainedContainerDataSource::New(_tokens->proceduralType, procTypePrimvar),
            HdMeshSchemaTokens->mesh,
            meshDs,
            HdUsdBrepTokens->usdBrepTessellatorData,
            tessData
        ),
        baseDs
    );
}

HdDataSourceLocatorSet HdUsdBrepArrayAdapter::InvalidateImagingSubprim(
    UsdPrim const& prim,
    TfToken const& subprim,
    TfTokenVector const& properties,
    UsdImagingPropertyInvalidationType invalidationType
)
{
    if (!subprim.IsEmpty())
    {
        return HdDataSourceLocatorSet();
    }

    // The base handles xform, visibility and purpose edits.
    HdDataSourceLocatorSet result = UsdImagingDataSourceGprim::Invalidate(prim, subprim, properties, invalidationType);

    // The base does not invalidate doubleSided, which the adapter adds to the mesh schema.
    if (std::find(properties.begin(), properties.end(), UsdGeomTokens->doubleSided) != properties.end())
    {
        result.insert(HdMeshSchema::GetDoubleSidedLocator());
    }

    // Geometry edits repopulate; subset edits are not tracked and need a stage reload.
    static const char* const kGeomPrefixes[] = { "brep:", "region:",  "shell:", "faceuse:",  "face:",
                                                 "loop:", "edgeuse:", "edge:",  "wireEdge:", "vertex:" };
    for (const TfToken& p : properties)
    {
        const std::string& name = p.GetString();
        for (const char* pre : kGeomPrefixes)
        {
            if (TfStringStartsWith(name, pre))
            {
                result.insert(HdDataSourceLocator(UsdImagingTokens->stageSceneIndexRepopulate));
                return result;
            }
        }
    }
    return result;
}

PXR_NAMESPACE_CLOSE_SCOPE
