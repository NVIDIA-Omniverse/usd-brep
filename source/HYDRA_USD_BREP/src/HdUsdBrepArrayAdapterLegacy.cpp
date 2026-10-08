// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "HdUsdBrepArrayAdapter.h" // first: brings HdUsdBrepImagingHeaders.h

// LEGACY render index (UsdImagingDelegate) overrides. They are pure virtual, so they
// must exist, but BrepArrays display only on the scene index path: these are no-ops
// and Populate warns once.

#include "pxr/base/tf/diagnostic.h"
#include <mutex>

PXR_NAMESPACE_OPEN_SCOPE

SdfPath HdUsdBrepArrayAdapter::Populate(UsdPrim const& prim, UsdImagingIndexProxy* /*index*/, UsdImagingInstancerContext const* /*instancerContext*/)
{
    static std::once_flag s_warned;
    std::call_once(
        s_warned,
        [&prim]()
        {
            TF_WARN("BrepArray '%s' is not displayed: hdUsdBrep needs the Hydra scene index path", prim.GetPath().GetText());
        }
    );
    return SdfPath();
}

void HdUsdBrepArrayAdapter::TrackVariability(UsdPrim const&, SdfPath const&, HdDirtyBits*, UsdImagingInstancerContext const*) const
{
}

void HdUsdBrepArrayAdapter::UpdateForTime(UsdPrim const&, SdfPath const&, UsdTimeCode, HdDirtyBits, UsdImagingInstancerContext const*) const
{
}

HdDirtyBits HdUsdBrepArrayAdapter::ProcessPropertyChange(UsdPrim const&, SdfPath const&, TfToken const&)
{
    return HdChangeTracker::Clean;
}

void HdUsdBrepArrayAdapter::MarkDirty(UsdPrim const&, SdfPath const&, HdDirtyBits, UsdImagingIndexProxy*)
{
}

void HdUsdBrepArrayAdapter::_RemovePrim(SdfPath const&, UsdImagingIndexProxy*)
{
}

PXR_NAMESPACE_CLOSE_SCOPE
