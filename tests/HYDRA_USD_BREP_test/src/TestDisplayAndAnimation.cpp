// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HdUsdBrepImagingHeaders.h"
#include "HYDRA_USD_BREP_test.h"
#include "HdUsdBrepDisplaySceneIndexPlugin.h"
#include "pxr/imaging/hd/legacyDisplayStyleSchema.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/primvarsAPI.h"
#include "pxr/usd/usdGeom/subset.h"
#include "pxr/usd/usdShade/materialBindingAPI.h"


#include "pxr/imaging/hd/sceneIndexPluginRegistry.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/imaging/hd/instancedBySchema.h"
#include "pxr/usdImaging/usdImaging/sceneIndices.h"
#include "pxr/usdImaging/usdImaging/stageSceneIndex.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdShade/material.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include <algorithm>
#include <atomic>
#include <thread>
#include <iostream>
PXR_NAMESPACE_USING_DIRECTIVE
namespace
{
struct Observer : HdSceneIndexObserver
{
    int generatedNotices = 0;
    const std::thread::id updateThread = std::this_thread::get_id();
    std::atomic<bool> wrongThread{ false };
    void PrimsAdded(const HdSceneIndexBase&, const AddedPrimEntries& entries) override
    {
        Count(entries);
    }
    void PrimsDirtied(const HdSceneIndexBase&, const DirtiedPrimEntries& entries) override
    {
        Count(entries);
    }
    void PrimsRemoved(const HdSceneIndexBase&, const RemovedPrimEntries& entries) override
    {
        Count(entries);
    }
    void PrimsRenamed(const HdSceneIndexBase&, const RenamedPrimEntries&) override
    {
    }
    template <class Entries>
    void Count(const Entries& entries)
    {
        if (std::this_thread::get_id() != updateThread)
        {
            wrongThread.store(true);
            return;
        }
        for (const auto& entry : entries)
        {
            if (entry.primPath.GetString().find("tessellated_mesh_") != std::string::npos)
            {
                ++generatedNotices;
            }
        }
    }
};
// Collects the locators dirtied on one prim.
struct DirtyRecorder : HdSceneIndexObserver
{
    SdfPath path;
    HdDataSourceLocatorSet locators;
    void PrimsAdded(const HdSceneIndexBase&, const AddedPrimEntries&) override
    {
    }
    void PrimsDirtied(const HdSceneIndexBase&, const DirtiedPrimEntries& entries) override
    {
        for (const auto& entry : entries)
        {
            if (entry.primPath == path)
            {
                locators.insert(entry.dirtyLocators);
            }
        }
    }
    void PrimsRemoved(const HdSceneIndexBase&, const RemovedPrimEntries&) override
    {
    }
    void PrimsRenamed(const HdSceneIndexBase&, const RenamedPrimEntries&) override
    {
    }
};
} // namespace
void TestDisplayAndAnimation(const char* filename)
{
    auto layer = SdfLayer::CreateAnonymous();
    layer->TransferContent(SdfLayer::FindOrOpen(filename));
    auto stage = UsdStage::Open(layer);
    auto prim = stage->GetPrimAtPath(SdfPath("/World/Brep0"));
    auto blue = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Blue"));
    auto red = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Red"));
    auto input = UsdImagingStageSceneIndex::New();
    // Use the same registry ordering as a renderer: our refresh filter must
    // auto-load before the default hdGp resolver, not be manually updated.
    auto output = HdSceneIndexPluginRegistry::GetInstance().AppendSceneIndicesForRenderer("GL", input);
    Observer observer;
    output->AddObserver(HdSceneIndexObserverPtr(&observer));
    input->SetStage(stage);
    const SdfPath mesh("/World/Brep0/tessellated_mesh_0");
    const SdfPath edges("/World/Brep0/brep_edges_0");
    auto visible = [&](const SdfPath& path)
    {
        auto ds = HdVisibilitySchema::GetFromParent(output->GetPrim(path).dataSource).GetVisibility();
        return !ds || ds->GetTypedValue(0);
    };
    const auto cachedPoints = PrimvarVec3fArray(output->GetPrim(mesh).dataSource, "points");
    Check(output->GetPrim(edges).primType == TfToken("basisCurves"), "resolved boundary curves missing");
    Check(visible(mesh) && !visible(edges), "shaded display includes boundary curves");
    const int notices = observer.generatedNotices;
    HdUsdBrepSetDisplayMode(1);
    Check(visible(mesh) && !visible(edges), "queued request changed scene before update");
    Check(observer.generatedNotices == notices, "setter sent immediate notices");
    HdUsdBrepApplyPendingDisplayMode();
    Check(!visible(mesh) && visible(edges), "wireframe must show only boundary curves");
    Check(observer.generatedNotices > notices, "mode switch did not dirty the mesh");
    HdUsdBrepSetDisplayMode(2);
    HdUsdBrepApplyPendingDisplayMode();
    Check(visible(mesh) && visible(edges), "combined display must show surfaces and boundaries");
    auto repr = HdLegacyDisplayStyleSchema::GetFromParent(output->GetPrim(mesh).dataSource).GetReprSelector();
    Check(repr && repr->GetTypedValue(0) == VtTokenArray{ TfToken("smoothHull") }, "combined display retains triangle wires");
    HdUsdBrepSetDisplayMode(0);
    HdUsdBrepApplyPendingDisplayMode();
    Check(visible(mesh) && !visible(edges), "shaded display was not restored");
    const auto currentPoints = PrimvarVec3fArray(output->GetPrim(mesh).dataSource, "points");
    Check(cachedPoints.cdata() == currentPoints.cdata(), "mode switch regenerated geometry");
    // A flush from another thread must not notify this scene's observers.
    std::atomic<bool> stop{ false }, missingPrim{ false };
    std::thread reader(
        [&]
        {
            int mode = 0;
            while (!stop.load())
            {
                HdUsdBrepSetDisplayMode(mode++ % 3);
                HdUsdBrepApplyPendingDisplayMode();
                if (!output->GetPrim(mesh).dataSource || !output->GetPrim(edges).dataSource)
                {
                    missingPrim.store(true);
                }
            }
        }
    );
    auto emptyInput = HdRetainedSceneIndex::New();
    for (int i = 0; i < 100; ++i)
    {
        auto transient = HdSceneIndexPluginRegistry::GetInstance().AppendSceneIndex(TfToken("HdUsdBrepDisplaySceneIndexPlugin"), emptyInput, nullptr);
        // Drop the last reference while the update thread snapshots the registry.
        std::thread destroy(
            [scene = std::move(transient)]() mutable
            {
                scene.Reset();
            }
        );
        HdUsdBrepApplyPendingDisplayMode();
        destroy.join();
    }
    stop.store(true);
    reader.join();
    Check(!missingPrim.load(), "concurrent display updates lost geometry");
    Check(!observer.wrongThread.load(), "display notices ran off the scene update thread");
    HdUsdBrepSetDisplayMode(0);
    HdUsdBrepApplyPendingDisplayMode();
    Check(visible(mesh) && !visible(edges), "final queued mode was not applied");
    // Display animation remains supported without time-sampled BRep reads.
    auto translate = UsdGeomXformable(prim).AddTranslateOp();
    auto visibility = UsdGeomImageable(prim).GetVisibilityAttr();
    Check(translate.Set(GfVec3d(1, 2, 3), UsdTimeCode(1)), "author first transform sample");
    Check(translate.Set(GfVec3d(4, 5, 6), UsdTimeCode(2)), "author second transform sample");
    Check(visibility.Set(UsdGeomTokens->inherited, UsdTimeCode(1)), "author first visibility sample");
    Check(visibility.Set(UsdGeomTokens->invisible, UsdTimeCode(2)), "author second visibility sample");
    const auto staticPoints = PrimvarVec3fArray(output->GetPrim(mesh).dataSource, "points");
    for (int frame : { 1, 2, 1 })
    {
        input->SetTime(UsdTimeCode(frame));
        input->ApplyPendingUpdates();
        auto data = output->GetPrim(mesh).dataSource;
        Check(PrimvarVec3fArray(data, "points").cdata() == staticPoints.cdata(), "animation re-tessellated the BRep");
        auto matrix = HdXformSchema::GetFromParent(data).GetMatrix();
        auto animatedVisibility = HdVisibilitySchema::GetFromParent(data).GetVisibility();
        Check(bool(matrix), "missing animated transform");
        Check(matrix->GetTypedValue(0).ExtractTranslation() == (frame == 1 ? GfVec3d(1, 2, 3) : GfVec3d(4, 5, 6)), "stale animated transform");
        // An absent visibility opinion means visible.
        Check((!animatedVisibility || animatedVisibility->GetTypedValue(0)) == (frame == 1), "stale animated visibility");
    }
    std::cout << "PASS: static BRep with animated transform and visibility, including backward scrubbing\n";
    output->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

void TestDoubleSidedAndOpacity(const char* filename)
{
    auto layer = SdfLayer::CreateAnonymous();
    layer->TransferContent(SdfLayer::FindOrOpen(filename));
    auto stage = UsdStage::Open(layer);
    auto prim = stage->GetPrimAtPath(SdfPath("/World/Brep0"));
    auto doubleSided = UsdGeomGprim(prim).CreateDoubleSidedAttr();
    auto opacity = UsdGeomPrimvarsAPI(prim).CreatePrimvar(HdTokens->displayOpacity, SdfValueTypeNames->FloatArray, UsdGeomTokens->constant);
    Check(doubleSided.Set(true), "author doubleSided");
    Check(opacity.Set(VtFloatArray{ 0.25f }), "author displayOpacity");
    auto input = UsdImagingStageSceneIndex::New();
    auto output = HdSceneIndexPluginRegistry::GetInstance().AppendSceneIndicesForRenderer("GL", input);
    DirtyRecorder recorder;
    recorder.path = SdfPath("/World/Brep0/tessellated_mesh_0");
    output->AddObserver(HdSceneIndexObserverPtr(&recorder));
    input->SetStage(stage);
    auto checkState = [&](bool expectedDoubleSided, float expectedOpacity, const std::string& when)
    {
        auto data = output->GetPrim(recorder.path).dataSource;
        auto sided = HdMeshSchema::GetFromParent(data).GetDoubleSided();
        Check(sided && sided->GetTypedValue(0) == expectedDoubleSided, "doubleSided " + when);
        auto opacityPrimvar = HdPrimvarsSchema::GetFromParent(data).GetPrimvar(HdTokens->displayOpacity);
        Check(bool(opacityPrimvar.GetPrimvarValue()), "missing displayOpacity " + when);
        Check(opacityPrimvar.GetPrimvarValue()->GetValue(0).Get<VtFloatArray>() == VtFloatArray{ expectedOpacity }, "displayOpacity " + when);
        Check(
            opacityPrimvar.GetInterpolation() && opacityPrimvar.GetInterpolation()->GetTypedValue(0) == HdPrimvarSchemaTokens->constant,
            "displayOpacity interpolation " + when
        );
    };
    checkState(true, 0.25f, "as authored");
    const auto points = PrimvarVec3fArray(output->GetPrim(recorder.path).dataSource, "points");
    // Edits must dirty the generated mesh, or a renderer keeps drawing the old state.
    recorder.locators = HdDataSourceLocatorSet();
    Check(doubleSided.Set(false), "edit doubleSided");
    Check(opacity.Set(VtFloatArray{ 0.5f }), "edit displayOpacity");
    input->ApplyPendingUpdates();
    Check(recorder.locators.Intersects(HdMeshSchema::GetDoubleSidedLocator()), "doubleSided edit did not dirty the mesh");
    Check(
        recorder.locators.Intersects(HdPrimvarsSchema::GetDefaultLocator().Append(HdTokens->displayOpacity)),
        "displayOpacity edit did not dirty the mesh"
    );
    checkState(false, 0.5f, "after edit");
    Check(PrimvarVec3fArray(output->GetPrim(recorder.path).dataSource, "points").cdata() == points.cdata(), "display edit re-tessellated the BRep");
    output->RemoveObserver(HdSceneIndexObserverPtr(&recorder));
    std::cout << "PASS: doubleSided and constant displayOpacity reach the mesh and follow edits\n";
}

void TestGeomSubsetChildren(const char* filename)
{
    auto layer = SdfLayer::CreateAnonymous();
    layer->TransferContent(SdfLayer::FindOrOpen(filename));
    auto stage = UsdStage::Open(layer);
    auto prim = stage->GetPrimAtPath(SdfPath("/World/Brep0"));
    auto blue = UsdShadeMaterial::Define(stage, SdfPath("/Looks/Blue"));
    auto subset = UsdGeomSubset::Define(stage, prim.GetPath().AppendChild(TfToken("FaceMaterial")));
    Check(subset.GetElementTypeAttr().Set(UsdGeomTokens->face), "author subset elementType");
    Check(subset.GetIndicesAttr().Set(VtIntArray{ 0 }), "author subset indices");
    Check(subset.GetFamilyNameAttr().Set(TfToken("materialBind")), "author subset familyName");
    Check(UsdShadeMaterialBindingAPI::Apply(subset.GetPrim()).Bind(blue), "bind subset material");
    auto input = UsdImagingStageSceneIndex::New();
    auto output = HdSceneIndexPluginRegistry::GetInstance().AppendSceneIndicesForRenderer("GL", input);
    input->SetStage(stage);
    // The source subset is tessellation input only: populated as a Hydra geomSubset it would name
    // the procedural as its mesh, which Storm's render index rejects.
    const auto children = output->GetChildPrimPaths(prim.GetPath());
    Check(std::find(children.begin(), children.end(), subset.GetPath()) == children.end(), "source GeomSubset populated as a Hydra prim");
    auto generated = output->GetPrim(SdfPath("/World/Brep0/tessellated_mesh_0/material_0"));
    Check(generated.primType == TfToken("geomSubset"), "generated mesh lost its material subset");
    std::cout << "PASS: BrepArray GeomSubsets feed tessellation and are not Hydra prims\n";
}

void TestNativeInstances(const char* filename)
{
    auto stage = UsdStage::CreateInMemory();
    for (int i = 0; i < 2; ++i)
    {
        auto prim = stage->DefinePrim(SdfPath(i ? "/Right" : "/Left"), TfToken("Xform"));
        Check(prim.GetReferences().AddReference(filename, SdfPath("/World")), "reference box");
        Check(prim.SetInstanceable(true), "instance box");
        Check(UsdGeomXformable(prim).AddTranslateOp().Set(GfVec3d(i * 30, 0, 0)), "translate instance");
    }
    UsdImagingCreateSceneIndicesInfo info;
    auto indices = UsdImagingCreateSceneIndices(info);
    auto output = HdSceneIndexPluginRegistry::GetInstance().AppendSceneIndicesForRenderer("GL", indices.finalSceneIndex);
    indices.stageSceneIndex->SetStage(stage);
    indices.postInstancingNoticeBatchingSceneIndex->Flush();
    std::vector<SdfPath> pending{ SdfPath::AbsoluteRootPath() };
    int meshes = 0;
    while (!pending.empty())
    {
        auto path = pending.back();
        pending.pop_back();
        auto prim = output->GetPrim(path);
        if (prim.primType == TfToken("mesh") || prim.primType == TfToken("basisCurves"))
        {
            auto instancers = HdInstancedBySchema::GetFromParent(prim.dataSource).GetPaths();
            Check(instancers && !instancers->GetTypedValue(0).empty(), "generated prototype mesh lost instancedBy");
            ++meshes;
        }
        for (const auto& child : output->GetChildPrimPaths(path))
        {
            pending.push_back(child);
        }
    }
    Check(meshes > 0, "no native-instance meshes");
    std::cout << "PASS: native instance meshes retain instancer associations\n";
}
