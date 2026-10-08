// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HD_USD_BREP_BREP_ARRAY_ADAPTER_H
#define HD_USD_BREP_BREP_ARRAY_ADAPTER_H

#include "HdUsdBrepApi.h"

#include "HdUsdBrepImagingHeaders.h"
#include "pxr/pxr.h"

PXR_NAMESPACE_OPEN_SCOPE

/// \class HdUsdBrepArrayAdapter
///
/// Presents BrepArray prims to Hydra as generative procedurals for
/// HdUsdBrepArrayProcedural, supplying hdGp:proceduralType and the tessellation.
///
class HdUsdBrepArrayAdapter : public UsdImagingInstanceablePrimAdapter
{
public:

    using BaseAdapter = UsdImagingInstanceablePrimAdapter;

    // ---------------------------------------------------------------------- //
    /// \name Scene index path
    // ---------------------------------------------------------------------- //

    HDUSDBREP_API TfTokenVector GetImagingSubprims(UsdPrim const& prim) override;
    HDUSDBREP_API PopulationMode GetPopulationMode() override;

    HDUSDBREP_API TfToken GetImagingSubprimType(UsdPrim const& prim, TfToken const& subprim) override;

    HDUSDBREP_API HdContainerDataSourceHandle
    GetImagingSubprimData(UsdPrim const& prim, TfToken const& subprim, const UsdImagingDataSourceStageGlobals& stageGlobals) override;

    HDUSDBREP_API HdDataSourceLocatorSet InvalidateImagingSubprim(
        UsdPrim const& prim,
        TfToken const& subprim,
        TfTokenVector const& properties,
        UsdImagingPropertyInvalidationType invalidationType
    ) override;

    // ---------------------------------------------------------------------- //
    /// \name Legacy render index: required no-ops, in HdUsdBrepArrayAdapterLegacy.cpp
    // ---------------------------------------------------------------------- //

    HDUSDBREP_API SdfPath
    Populate(UsdPrim const& prim, UsdImagingIndexProxy* index, UsdImagingInstancerContext const* instancerContext = nullptr) override;

    HDUSDBREP_API void TrackVariability(
        UsdPrim const& prim,
        SdfPath const& cachePath,
        HdDirtyBits* timeVaryingBits,
        UsdImagingInstancerContext const* instancerContext = nullptr
    ) const override;

    HDUSDBREP_API void UpdateForTime(
        UsdPrim const& prim,
        SdfPath const& cachePath,
        UsdTimeCode time,
        HdDirtyBits requestedBits,
        UsdImagingInstancerContext const* instancerContext = nullptr
    ) const override;

    HDUSDBREP_API HdDirtyBits ProcessPropertyChange(UsdPrim const& prim, SdfPath const& cachePath, TfToken const& propertyName) override;

    HDUSDBREP_API void MarkDirty(UsdPrim const& prim, SdfPath const& cachePath, HdDirtyBits dirty, UsdImagingIndexProxy* index) override;

protected:

    HDUSDBREP_API void _RemovePrim(SdfPath const& cachePath, UsdImagingIndexProxy* index) override;
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif // HD_USD_BREP_BREP_ARRAY_ADAPTER_H
