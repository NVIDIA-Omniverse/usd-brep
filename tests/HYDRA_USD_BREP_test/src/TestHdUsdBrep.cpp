// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HdUsdBrepArrayAdapter.h" // first: brings HdUsdBrepImagingHeaders.h
#include "HYDRA_USD_BREP_test.h"
#include "SmApiTessellationParams.h"
#include "SmApiUsdTessellate.h"

#include "HdUsdBrepArrayProcedural.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/imaging/hd/purposeSchema.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdShade/material.h"
#include "pxr/imaging/hd/materialBindingsSchema.h"
#include "pxr/imaging/hd/geomSubsetSchema.h"
#include "pxr/imaging/hd/primOriginSchema.h"
#include "pxr/usd/usdGeom/primvarsAPI.h"
#include "pxr/usdImaging/usdImaging/dataSourceStageGlobals.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include <algorithm>
#include <iostream>
PXR_NAMESPACE_USING_DIRECTIVE

// Same inputs as the adapter, so results can be compared with the Hydra handoff.
static std::vector<SmApiUsdTessellatedBrep> Tessellate(const UsdPrim& prim)
{
    std::vector<SmApiUsdTessellatedBrep> results;
    SmApiUsdTessellateBrepArray(prim, SmTessellationParams(), FALSE, 5.0, results);
    return results;
}
class Globals : public UsdImagingDataSourceStageGlobals
{
    UsdTimeCode GetTime() const override
    {
        return UsdTimeCode::Default();
    }
    void FlagAsTimeVarying(const SdfPath&, const HdDataSourceLocator&) const override
    {
    }
    void FlagAsAssetPathDependent(const SdfPath&) const override
    {
    }
};
// Stateless and referenced by the adapter's data sources, so it must outlive them.
static const Globals s_globals;
// The adapter's data for prim, as a generative procedural in a retained scene.
static HdRetainedSceneIndexRefPtr AdapterScene(const UsdPrim& prim)
{
    HdUsdBrepArrayAdapter adapter;
    auto scene = HdRetainedSceneIndex::New();
    scene->AddPrims({ { prim.GetPath(), TfToken("generativeProcedural"), adapter.GetImagingSubprimData(prim, TfToken(), s_globals) } });
    return scene;
}
static void TestMultipleBodies(const char* filename)
{
    auto stage = UsdStage::Open(filename);
    Check(bool(stage), "open multiple-body fixture");
    auto prim = stage->GetPrimAtPath(SdfPath("/World/Brep0"));
    // Body 0 has its own material; body 1 keeps the BrepArray's, which Hydra resolves.
    auto red = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Red"));
    auto blue = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Blue"));
    Check(UsdShadeMaterialBindingAPI::Apply(prim).Bind(red), "bind BrepArray material");
    auto bodySubset = UsdGeomSubset::Define(stage, prim.GetPath().AppendChild(TfToken("BodyMaterial")));
    Check(bodySubset.GetElementTypeAttr().Set(TfToken("brep")), "author body subset elementType");
    Check(bodySubset.GetIndicesAttr().Set(VtIntArray{ 0 }), "author body subset indices");
    Check(UsdShadeMaterialBindingAPI::Apply(bodySubset.GetPrim()).Bind(blue), "bind body material");
    auto results = Tessellate(prim);
    Check(results.size() == 2, "expected two bodies in one BrepArray");
    auto scene = AdapterScene(prim);
    // A resolved "full" binding on the BrepArray, as usdview supplies; both meshes must keep it.
    const SdfPath full("/Looks/Full");
    auto fullBinding = HdMaterialBindingSchema::Builder().SetPath(HdRetainedTypedSampledDataSource<SdfPath>::New(full)).Build();
    auto bindings = HdRetainedContainerDataSource::New(TfToken("full"), fullBinding);
    auto source = scene->GetPrim(prim.GetPath()).dataSource;
    scene->AddPrims({ { prim.GetPath(),
                        TfToken("generativeProcedural"),
                        HdOverlayContainerDataSource::New(
                            HdRetainedContainerDataSource::New(HdMaterialBindingsSchemaTokens->materialBindings, bindings),
                            source
                        ) } });
    HdUsdBrepArrayProcedural proc(prim.GetPath());
    HdSceneIndexObserver::DirtiedPrimEntries dirties;
    auto children = proc.Update(scene, HdGpGenerativeProcedural::ChildPrimTypeMap(), HdGpGenerativeProcedural::DependencyMap(), &dirties);
    Check(children.size() == 4, "expected two meshes and two boundary curve prims");
    for (size_t i = 0; i < results.size(); ++i)
    {
        auto edgePath = prim.GetPath().AppendChild(TfToken("brep_edges_" + std::to_string(i)));
        auto edgeChild = proc.GetChildPrim(scene, edgePath);
        Check(edgeChild.primType == TfToken("basisCurves"), "missing boundary curves");
        Check(PrimvarVec3fArray(edgeChild.dataSource, "points") == results[i].m_sBoundaryPoints, "boundary points changed during Hydra handoff");
        auto path = prim.GetPath().AppendChild(TfToken("tessellated_mesh_" + std::to_string(i)));
        Check(children.count(path) == 1, "missing body mesh");
        auto child = proc.GetChildPrim(scene, path);
        Check(child.primType == TfToken("mesh"), "body child is not a mesh");
        Check(PrimvarVec3fArray(child.dataSource, "points") == results[i].m_sPoints, "body mesh points changed during Hydra handoff");
        auto bound = [&](const TfToken& purpose)
        {
            auto material = HdMaterialBindingsSchema::GetFromParent(child.dataSource).GetMaterialBinding(purpose).GetPath();
            return material ? material->GetTypedValue(0) : SdfPath();
        };
        Check(bound(TfToken()) == (i == 0 ? blue.GetPath() : SdfPath()), "body material binding");
        Check(bound(TfToken("full")) == full, "lost the BrepArray's purpose-specific binding");
    }
    std::cout << "PASS: two bodies produce distinct meshes at their authored positions and materials\n";
}
int main(int argc, char** argv)
{
    try
    {
        Check(argc == 5, "usage: HYDRA_USD_BREP_test_app schema-resources brep.usda plugin-resources two-bodies.usda");
        PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
        PlugRegistry::GetInstance().RegisterPlugins(argv[3]);
        TestDisplayAndAnimation(argv[2]);
        TestDoubleSidedAndOpacity(argv[2]);
        TestGeomSubsetChildren(argv[2]);
        TestNativeInstances(argv[2]);
        TestMultipleBodies(argv[4]);
        auto stage = UsdStage::Open(argv[2]);
        Check(bool(stage), "open asset");
        auto prim = stage->GetPrimAtPath(SdfPath("/World/Brep0"));
        auto red = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Red"));
        auto blue = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Blue"));
        Check(UsdShadeMaterialBindingAPI::Apply(prim).Bind(red), "author UsdShadeMaterialBindingAPI::Apply");
        auto sourceSubset = UsdGeomSubset::Define(stage, prim.GetPath().AppendChild(TfToken("FaceMaterial")));
        Check(sourceSubset.GetElementTypeAttr().Set(UsdGeomTokens->face), "author sourceSubset.GetElementTypeAttr");
        Check(sourceSubset.GetIndicesAttr().Set(VtIntArray{ 0 }), "author sourceSubset.GetIndicesAttr");
        Check(sourceSubset.GetFamilyNameAttr().Set(TfToken("materialBind")), "author sourceSubset.GetFamilyNameAttr");
        Check(UsdShadeMaterialBindingAPI::Apply(sourceSubset.GetPrim()).Bind(blue), "author UsdShadeMaterialBindingAPI::Apply");
        auto results = Tessellate(prim);
        Check(!results.empty() && (results[0].m_sStatus == SM_SUCCESS), "tessellate asset");
        auto colorAttr = UsdGeomPrimvarsAPI(prim).CreatePrimvar(TfToken("displayColor"), SdfValueTypeNames->Color3fArray, UsdGeomTokens->constant);
        Check(colorAttr.Set(VtArray<GfVec3f>{ { 1, 0, 0 } }), "author colorAttr.Set");
        auto translate = UsdGeomXformable(prim).AddTranslateOp();
        Check(translate.Set(GfVec3d(1, 2, 3)), "author translate.Set");
        auto scene = AdapterScene(prim);
        // usdview resolves bindings before the procedural; supply that state here.
        auto sourceData = scene->GetPrim(prim.GetPath()).dataSource;
        auto bindResolved = [&](const SdfPath& path)
        {
            auto binding = HdMaterialBindingSchema::Builder().SetPath(HdRetainedTypedSampledDataSource<SdfPath>::New(path)).Build();
            scene->AddPrims({ { prim.GetPath(),
                                TfToken("generativeProcedural"),
                                HdOverlayContainerDataSource::New(
                                    HdRetainedContainerDataSource::New(
                                        HdMaterialBindingsSchemaTokens->materialBindings,
                                        HdRetainedContainerDataSource::New(TfToken(), binding)
                                    ),
                                    sourceData
                                ) } });
        };
        bindResolved(red.GetPath());
        HdUsdBrepArrayProcedural proc(prim.GetPath());
        HdSceneIndexObserver::DirtiedPrimEntries dirties;
        auto children = proc.Update(scene, HdGpGenerativeProcedural::ChildPrimTypeMap(), HdGpGenerativeProcedural::DependencyMap(), &dirties);
        const auto isMesh = [](const auto& entry)
        {
            return entry.second == TfToken("mesh");
        };
        Check(size_t(std::count_if(children.begin(), children.end(), isMesh)) == results.size(), "generated mesh count");
        Check(!results[0].m_sSubsetIndices.empty(), "missing material groups");
        for (size_t i = 0; i < results[0].m_sSubsetIndices.size(); ++i)
        {
            auto subset = proc.GetChildPrim(
                scene,
                prim.GetPath().AppendChild(TfToken("tessellated_mesh_0")).AppendChild(TfToken("material_" + std::to_string(i)))
            );
            Check(subset.primType == TfToken("geomSubset"), "missing Hydra material subset");
            Check(
                Require(HdGeomSubsetSchema::GetFromParent(subset.dataSource).GetIndices(), "missing subset indices")->GetTypedValue(0) ==
                    results[0].m_sSubsetIndices[i],
                "Hydra material face indices"
            );
            Check(
                Require(HdMaterialBindingsSchema::GetFromParent(subset.dataSource).GetMaterialBinding().GetPath(), "missing subset material")
                        ->GetTypedValue(0) == results[0].m_sSubsetMaterialPaths[i],
                "Hydra material path"
            );
        }
        auto child = proc.GetChildPrim(scene, prim.GetPath().AppendChild(TfToken("tessellated_mesh_0")));
        auto normal = HdPrimvarsSchema::GetFromParent(child.dataSource).GetPrimvar(TfToken("normals"));
        Check(bool(normal) && bool(normal.GetInterpolation()), "missing Hydra normals");
        Check(normal.GetInterpolation()->GetTypedValue(0) == UsdGeomTokens->faceVarying, "Hydra interpolation");
        auto flat = PrimvarVec3fArray(child.dataSource, "normals");
        const auto& expected = results[0];
        Check(flat.size() == expected.m_sFaceVertexIndices.size(), "Hydra corner count");
        for (size_t i = 0; i < flat.size(); ++i)
        {
            Check(flat[i] == expected.m_sNormals[expected.m_sNormalsIndices[i]], "Hydra corner normal");
        }
        auto dependencies = proc.UpdateDependencies(scene);
        Check(dependencies[prim.GetPath()].Intersects(HdDataSourceLocator(TfToken("xform"))), "missing transform dependency");
        Check(dependencies[prim.GetPath()].Intersects(HdDataSourceLocator(TfToken("primvars"))), "missing primvar dependency");
        Check(UsdShadeMaterialBindingAPI(prim).Bind(blue), "author UsdShadeMaterialBindingAPI");
        bindResolved(blue.GetPath());
        Check(colorAttr.Set(VtArray<GfVec3f>{ { 0, 1, 0 } }), "author colorAttr.Set");
        Check(translate.Set(GfVec3d(4, 5, 6)), "author translate.Set");
        Check(UsdGeomImageable(prim).CreateVisibilityAttr().Set(UsdGeomTokens->invisible), "author visibility");
        Check(UsdGeomImageable(prim).CreatePurposeAttr().Set(UsdGeomTokens->render), "author UsdGeomImageable");
        dirties.clear();
        proc.Update(scene, children, dependencies, &dirties);
        Check(dirties.size() == children.size(), "live edits did not dirty children");
        child = proc.GetChildPrim(scene, prim.GetPath().AppendChild(TfToken("tessellated_mesh_0")));
        Check(
            !Require(HdVisibilitySchema::GetFromParent(child.dataSource).GetVisibility(), "missing visibility")->GetTypedValue(0),
            "lost visibility"
        );
        Check(
            Require(HdPurposeSchema::GetFromParent(child.dataSource).GetPurpose(), "missing purpose")->GetTypedValue(0) == UsdGeomTokens->render,
            "lost purpose"
        );
        auto materialBinding = HdMaterialBindingsSchema::GetFromParent(child.dataSource).GetMaterialBinding();
        Check(bool(materialBinding.GetPath()), "missing whole-prim material binding");
        Check(materialBinding.GetPath()->GetTypedValue(0) == blue.GetPath(), "stale whole-prim material binding");
        auto colors = PrimvarVec3fArray(child.dataSource, "displayColor");
        Check(!colors.empty(), "empty display color");
        Check(colors[0] == GfVec3f(0, 1, 0), "stale display color");
        auto matrix = Require(HdXformSchema::GetFromParent(child.dataSource).GetMatrix(), "missing transform")->GetTypedValue(0);
        Check(matrix.ExtractTranslation() == GfVec3d(4, 5, 6), "stale transform");
        Check(
            HdPrimOriginSchema::GetFromParent(child.dataSource).GetOriginPath(HdPrimOriginSchemaTokens->scenePath) == prim.GetPath(),
            "missing selection origin"
        );
        // Children are named by Brep index, not position: re-key body 0 as 7.
        auto tessData = Require(HdContainerDataSource::Cast(sourceData->Get(TfToken("usdBrepTessellatorData"))), "missing tessellation data");
        auto remapped = HdRetainedContainerDataSource::New(
            TfToken("usdBrepTessellatorData"),
            HdRetainedContainerDataSource::New(TfToken("7"), Require(tessData->Get(TfToken("0")), "missing body 0"))
        );
        scene->AddPrims({ { prim.GetPath(), TfToken("generativeProcedural"), remapped } });
        HdUsdBrepArrayProcedural remappedProc(prim.GetPath());
        auto remappedChildren = remappedProc
                                    .Update(scene, HdGpGenerativeProcedural::ChildPrimTypeMap(), HdGpGenerativeProcedural::DependencyMap(), &dirties);
        SdfPath stablePath = prim.GetPath().AppendChild(TfToken("tessellated_mesh_7"));
        Check(remappedChildren.count(stablePath) == 1, "body identity compacted");
        Check(remappedChildren.count(prim.GetPath().AppendChild(TfToken("tessellated_mesh_0"))) == 0, "body named by position");
        Check(remappedProc.GetChildPrim(scene, stablePath).dataSource != nullptr, "stable child lookup failed");
        std::cout << "PASS: normals, display state, materials and selection identity\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
