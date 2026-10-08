// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_USD_WRITER_H
#define BREP_USD_WRITER_H

#include "BrepStagingState.h"
#include "BrepStatus.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/range2d.h>
#include <pxr/base/gf/range3d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <cstdint>
#include <string>

namespace UsdBrepData
{
class UsdBrepBuilder;
}


class StagedBrepUsdWriter
{
public:

    explicit StagedBrepUsdWriter(const BrepStagingState& rState) : m_rState(rState)
    {
    }

    BrepStatus BuildUsdBrepFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;

    // Author a uniquely named BrepArray prim under rRootPath from the staged model: builds the UsdBrep arrays and
    // serializes them to USD. rSource tags the prim's customData["source"] (e.g. "OCCT"). On failure returns a
    // non-success BrepStatus and, when pError is non-null, a human-readable explanation. When pOutPath is non-null it
    // receives the path of the created BrepArray prim on success.
    BrepStatus WriteBrepArrayPrim(
        const pxr::UsdStageRefPtr& rStage,
        const pxr::SdfPath& rRootPath,
        const std::string& rSource,
        std::string* pError = nullptr,
        pxr::SdfPath* pOutPath = nullptr
    ) const;

private:

    bool PopulateDirectUsdBrepMetadataFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepRegionsFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepShellsFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepFaceusesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepFacesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepFaceSurfaceFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder, uint32_t uiFaceIndex, const pxr::TfToken& rSurfaceType)
        const;
    bool AppendDirectUsdBrepLoopsFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepEdgeusesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepEdgesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepEdgeCurveFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder, uint32_t uiEdgeIndex, const pxr::TfToken& rCurveType) const;
    bool AppendDirectUsdBrepWireEdgesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool AppendDirectUsdBrepWireEdgeCurveFromStagedData(
        UsdBrepData::UsdBrepBuilder& rBuilder,
        uint32_t uiWireEdgeIndex,
        const pxr::TfToken& rCurveType
    ) const;
    bool AppendDirectUsdBrepVerticesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const;
    bool PopulateCurrentStagedBrepDirectlyToUsdBrepBuilder(UsdBrepData::UsdBrepBuilder& rBuilder, std::string* pErrorStage = nullptr) const;

    double GetStagedBrepTolerance() const;
    pxr::GfRange3d GetStagedBrepExtent() const;
    uint32_t GetStagedRegionCount() const;
    StagedRegionType GetStagedRegionType(uint32_t uiRegionIndex) const;
    uint32_t GetStagedRegionShellCount(uint32_t uiRegionIndex) const;
    uint32_t GetStagedShellCount() const;
    uint32_t GetStagedShellFaceuseCount(uint32_t uiShellIndex) const;
    uint32_t GetStagedShellWireEdgeCount(uint32_t uiShellIndex) const;
    StagedShellPointType GetStagedShellPointType(uint32_t uiShellIndex) const;
    bool TryGetStagedShellPoint(uint32_t uiShellPointIndex, pxr::GfVec3d& rPoint) const;
    uint32_t GetStagedFaceuseCount() const;
    uint32_t GetStagedFaceuseFaceIndex(uint32_t uiFaceuseIndex) const;
    StagedOrientType GetStagedFaceuseOrientation(uint32_t uiFaceuseIndex) const;
    uint32_t GetStagedFaceCount() const;
    StagedSurfaceType GetStagedFaceSurfaceType(uint32_t uiFaceIndex) const;
    StagedTrimType GetStagedFaceTrimType(uint32_t uiFaceIndex) const;
    uint32_t GetStagedFaceLoopCount(uint32_t uiFaceIndex) const;
    bool TryGetStagedFaceDomain(uint32_t uiFaceIndex, pxr::GfRange2d& rUVDomain) const;
    const StagedSurfaceData* GetStagedFaceSurfaceRecord(uint32_t uiSurfaceIndex) const;
    uint32_t GetStagedLoopCount() const;
    uint32_t GetStagedLoopEdgeuseCount(uint32_t uiLoopIndex) const;
    uint32_t GetStagedLoopVertexIndex(uint32_t uiLoopIndex) const;
    uint32_t GetStagedEdgeuseCount() const;
    StagedEdgeuseData GetStagedEdgeuse(uint32_t uiEdgeuseIndex) const;
    const StagedCurveData2d* GetStagedEdgeuseCurve2d(uint32_t uiEdgeuseIndex) const;
    uint32_t GetStagedEdgeCount() const;
    StagedCurveType GetStagedEdgeCurveType(uint32_t uiEdgeIndex) const;
    pxr::GfRange1d GetStagedEdgeInterval(uint32_t uiEdgeIndex) const;
    uint32_t GetStagedEdgeStartVertexIndex(uint32_t uiEdgeIndex) const;
    uint32_t GetStagedEdgeEndVertexIndex(uint32_t uiEdgeIndex) const;
    const StagedCurveData* GetStagedEdgeCurveRecord(uint32_t uiCurveIndex) const;
    uint32_t GetStagedWireEdgeCount() const;
    const StagedEdgeData* GetStagedWireEdgeRecord(uint32_t uiEdgeIndex) const;
    const StagedCurveData* GetStagedWireEdgeCurveRecord(uint32_t uiCurveIndex) const;
    uint32_t GetStagedVertexCount() const;
    uint32_t GetStagedVertexPointCount() const;
    StagedVertexType GetStagedVertexType(uint32_t uiVertexIndex) const;
    bool TryGetStagedVertexPoint(uint32_t uiVertexIndex, pxr::GfVec3d& rPoint) const;

    const BrepStagingState& m_rState;
};

#endif // BREP_USD_WRITER_H
