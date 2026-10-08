// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_EDGE_VERTEX_BUILDER_H
#define BREP_EDGE_VERTEX_BUILDER_H

#include "BrepSourceData.h"
#include "BrepStatus.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/vt/array.h>

#include <cstdint>

namespace UsdBrepData
{
class UsdBrepBuilder;
}

class NaturalBoundaryBuilder;


class StagedBrepEdgeVertexBuilder
{
public:

    explicit StagedBrepEdgeVertexBuilder(BrepStagingState& rState);

    BrepStatus AddSourceVertex(const SourceVertexData& rVertexData, uint32_t& rVertexIndex);
    BrepStatus AddSourceEdgeForCurrentTraversal(
        const SourceEdgeData& rEdgeData,
        uint32_t uiStartVertexIndex,
        uint32_t uiEndVertexIndex,
        pxr::GfRange1d& rEdgeInterval
    );

private:

    friend class NaturalBoundaryBuilder;

    uint32_t AddStagedLineCurve(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rDirection);
    uint32_t AddStagedCircleCurve(const pxr::GfVec3d& rCenter, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius);
    uint32_t AddStagedBSplineCurve(
        uint32_t uiVertexCount,
        uint32_t uiOrder,
        const pxr::VtArray<double>& rKnots,
        const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
        const pxr::VtArray<double>& rWeights
    );
    uint32_t AddStagedEdgeForCurrentTraversal(
        StagedCurveType eCurveType,
        const pxr::GfRange1d& rInterval,
        uint32_t uiStartVertexIndex,
        uint32_t uiEndVertexIndex
    );
    void AddStagedImpliedBoundaryEdgeuse(uint32_t uiEdgeIndex, StagedOrientType eOrientType, StagedRadialEntryType eEntrySideType);
    void AddStagedImpliedBoundaryEdgeuses(uint32_t uiFirstEdgeuseIndex);
    uint32_t SetStagedImpliedBoundaryEdgeuses(uint32_t uiFirstEdgeuseIndex);
    uint32_t AddStagedVertexPoint(const pxr::GfVec3d& rPoint);

    uint32_t AddStagedEdgeCurve(const StagedCurveData& rCurveData);
    uint32_t AddStagedEdge(const StagedEdgeData& rEdgeData);
    bool TryGetStagedVertexPoint(uint32_t uiVertexIndex, pxr::GfVec3d& rPoint) const;

    BrepStagingState& m_rState;
};

#endif // BREP_EDGE_VERTEX_BUILDER_H
