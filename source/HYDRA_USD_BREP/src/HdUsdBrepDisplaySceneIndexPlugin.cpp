// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HdUsdBrepImagingHeaders.h"
#include "HdUsdBrepDisplaySceneIndexPlugin.h"
#include "HdUsdBrepTokens.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/imaging/hd/filteringSceneIndex.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/legacyDisplayStyleSchema.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/imaging/hd/sceneIndexPluginRegistry.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/basisCurves.h"
#include "pxr/imaging/hd/sceneIndexPlugin.h"
#include "pxr/imaging/hdGp/sceneIndexPlugin.h"
#include "pxr/base/tf/weakPtr.h"
#include <atomic>
#include <mutex>
#include <set>
#include <thread>

PXR_NAMESPACE_OPEN_SCOPE
namespace
{
const TfToken boundaryRepr("usdBrepBoundary");
std::atomic<int> requestedDisplayMode{ 0 };
std::mutex instancesMutex;
class _DisplaySceneIndex;
std::set<_DisplaySceneIndex*> instances;

class _DisplaySceneIndex : public HdSingleInputFilteringSceneIndexBase
{
public:

    explicit _DisplaySceneIndex(const HdSceneIndexBaseRefPtr& input)
        : HdSingleInputFilteringSceneIndexBase(input),
          _updateThread(std::this_thread::get_id()),
          _displayMode(requestedDisplayMode.load(std::memory_order_relaxed))
    {
    }
    ~_DisplaySceneIndex() override
    {
        // Keep the TfRefBase alive while the registry promotes weak pointers.
        std::lock_guard<std::mutex> lock(instancesMutex);
        instances.erase(this);
    }
    HdSceneIndexPrim GetPrim(const SdfPath& path) const override
    {
        auto prim = _GetInputSceneIndex()->GetPrim(path);
        auto marker = prim.dataSource ? HdBoolDataSource::Cast(prim.dataSource->Get(HdUsdBrepTokens->usdBrepEdges)) : nullptr;
        if (!marker)
        {
            return prim;
        }
        // One snapshot keeps visibility and representation consistent for this prim.
        const int displayMode = _displayMode.load(std::memory_order_relaxed);
        const bool edges = marker->GetTypedValue(0);
        const bool hidden = edges ? displayMode == 0 : displayMode == 1;
        if (hidden)
        {
            prim.dataSource = HdOverlayContainerDataSource::New(
                HdRetainedContainerDataSource::New(
                    HdVisibilitySchemaTokens->visibility,
                    HdVisibilitySchema::Builder().SetVisibility(HdRetainedTypedSampledDataSource<bool>::New(false)).Build()
                ),
                prim.dataSource
            );
        }
        // Pixel-width curves; in the combined mode, hide the mesh's wire overlay.
        if (edges || displayMode == 2)
        {
            prim.dataSource = HdOverlayContainerDataSource::New(
                HdRetainedContainerDataSource::New(
                    HdLegacyDisplayStyleSchemaTokens->displayStyle,
                    HdLegacyDisplayStyleSchema::Builder()
                        .SetReprSelector(HdRetainedTypedSampledDataSource<VtTokenArray>::New(VtTokenArray{
                            edges ? (displayMode == 1 ? boundaryRepr : HdReprTokens->wire) : HdReprTokens->smoothHull }))
                        .Build()
                ),
                prim.dataSource
            );
        }
        return prim;
    }
    SdfPathVector GetChildPrimPaths(const SdfPath& path) const override
    {
        return _GetInputSceneIndex()->GetChildPrimPaths(path);
    }
    bool IsUpdateThread() const
    {
        return _updateThread == std::this_thread::get_id();
    }
    void ApplyDisplayMode(int mode)
    {
        if (_displayMode.exchange(mode, std::memory_order_relaxed) == mode)
        {
            return;
        }
        HdSceneIndexObserver::DirtiedPrimEntries entries;
        // Traverse only on a mode change; no geometry is regenerated.
        std::vector<SdfPath> pending{ SdfPath::AbsoluteRootPath() };
        while (!pending.empty())
        {
            SdfPath path = pending.back();
            pending.pop_back();
            auto prim = _GetInputSceneIndex()->GetPrim(path);
            if (prim.dataSource && prim.dataSource->Get(HdUsdBrepTokens->usdBrepEdges))
            {
                entries.emplace_back(
                    path,
                    HdDataSourceLocatorSet{ HdVisibilitySchema::GetDefaultLocator(), HdLegacyDisplayStyleSchema::GetReprSelectorLocator() }
                );
            }
            const auto children = GetChildPrimPaths(path);
            pending.insert(pending.end(), children.begin(), children.end());
        }
        _SendPrimsDirtied(entries);
    }

private:

    const std::thread::id _updateThread;
    std::atomic<int> _displayMode;

protected:

    void _PrimsAdded(const HdSceneIndexBase&, const HdSceneIndexObserver::AddedPrimEntries& entries) override
    {
        _SendPrimsAdded(entries);
    }
    void _PrimsRemoved(const HdSceneIndexBase&, const HdSceneIndexObserver::RemovedPrimEntries& entries) override
    {
        _SendPrimsRemoved(entries);
    }
    void _PrimsDirtied(const HdSceneIndexBase&, const HdSceneIndexObserver::DirtiedPrimEntries& entries) override
    {
        _SendPrimsDirtied(entries);
    }
};
} // namespace

class HdUsdBrepDisplaySceneIndexPlugin : public HdSceneIndexPlugin
{
protected:

    HdSceneIndexBaseRefPtr _AppendSceneIndex(const HdSceneIndexBaseRefPtr& input, const HdContainerDataSourceHandle& args) override;
};

HdSceneIndexBaseRefPtr HdUsdBrepDisplaySceneIndexPlugin::_AppendSceneIndex(const HdSceneIndexBaseRefPtr& input, const HdContainerDataSourceHandle&)
{
    auto scene = TfCreateRefPtr(new _DisplaySceneIndex(input));
    std::lock_guard<std::mutex> lock(instancesMutex);
    instances.insert(get_pointer(scene));
    return scene;
}
TF_REGISTRY_FUNCTION(TfType)
{
    HdSceneIndexPluginRegistry::Define<HdUsdBrepDisplaySceneIndexPlugin>();
}
TF_REGISTRY_FUNCTION(HdSceneIndexPlugin)
{
    HdBasisCurves::ConfigureRepr(boundaryRepr, HdBasisCurvesReprDesc(HdBasisCurvesGeomStyleWire, HdBasisCurvesReprDescTokens->surfaceShaderUnlit));
    HdSceneIndexPluginRegistry::GetInstance().RegisterSceneIndexForRenderer(
        "",
        TfToken("HdUsdBrepDisplaySceneIndexPlugin"),
        nullptr,
        HdGpSceneIndexPlugin::GetInsertionPhase() + 1,
        HdSceneIndexPluginRegistry::InsertionOrderAtEnd
    );
}
PXR_NAMESPACE_CLOSE_SCOPE

PXR_NAMESPACE_USING_DIRECTIVE

extern "C" void HdUsdBrepSetDisplayMode(int mode)
{
    if (mode >= 0 && mode <= 2)
    {
        requestedDisplayMode.store(mode, std::memory_order_relaxed);
    }
}

extern "C" void HdUsdBrepApplyPendingDisplayMode()
{
    const int mode = requestedDisplayMode.load(std::memory_order_relaxed);
    std::vector<TfRefPtr<_DisplaySceneIndex>> scenes;
    {
        std::lock_guard<std::mutex> lock(instancesMutex);
        scenes.reserve(instances.size());
        for (auto* scene : instances)
        {
            if (scene->IsUpdateThread())
            {
                // The destructor takes this mutex, so promotion fails safely once the scene is gone.
                auto ref = TfCreateRefPtrFromProtectedWeakPtr(TfWeakPtr<_DisplaySceneIndex>(scene));
                if (ref)
                {
                    scenes.push_back(std::move(ref));
                }
            }
        }
    }
    // No lock during callbacks; the strong references keep each scene alive.
    for (const auto& scene : scenes)
    {
        scene->ApplyDisplayMode(mode);
    }
}
