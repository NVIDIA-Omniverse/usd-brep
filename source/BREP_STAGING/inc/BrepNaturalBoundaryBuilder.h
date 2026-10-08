// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_NATURAL_BOUNDARY_BUILDER_H
#define BREP_NATURAL_BOUNDARY_BUILDER_H

#include "BrepEdgeVertexBuilder.h"
#include "BrepSourceData.h"
#include "BrepStatus.h"
#include "BrepTopologyBuilder.h"

class NaturalBoundaryBuilder
{
public:

    NaturalBoundaryBuilder(
        BrepStagingState& rState,
        StagedBrepEdgeVertexBuilder sEdgeVertexBuilder,
        StagedBrepTopologyBuilder sTopologyBuilder,
        uint32_t uiAugmentedOuterLoopEdgeuseCount,
        uint32_t uiOuterLoopEdgeuseStartIndex
    );

    BrepStatus ExtractNaturalBoundaryDataForCurrentFace(SourceNaturalBoundaryData& rBoundaryData);
    void ApplyNaturalBoundaryData(const SourceNaturalBoundaryData& rBoundaryData);
    BrepStatus ExtractImpliedBoundaryEdgesForCurrentFace(
        const SourceNaturalBoundaryVertexResolver& rFindExistingBoundaryVertex,
        uint32_t uiContactVertexIndex
    );
    BrepStatus AddImpliedNaturalBoundaryAsLoopForCurrentFace();
    BrepStatus AddImpliedNaturalBoundaryToLoop();

private:

    BrepStatus extractNaturalBoundaryVertexPositions(StagedSurfaceData& rSurfaceData);
    BrepStatus extractImpliedBoundaryEdges(
        const SourceNaturalBoundaryVertexResolver& rFindExistingBoundaryVertex,
        StagedSurfaceData& rSurfaceData,
        uint32_t uiContactVertexIndex
    );
    BrepStatus addImpliedNaturalBoundaryAsLoop();
    StagedFaceData* GetCurrentStagedFace();
    const StagedFaceData* GetCurrentStagedFace() const;
    pxr::GfRange2d GetCurrentStagedFaceDomain() const;
    StagedSurfaceData* GetCurrentStagedFaceSurface();
    StagedEdgeuseData GetStagedEdgeuse(uint32_t uiEdgeuseIndex) const;
    uint32_t GetStagedEdgeuseEdgeIndex(uint32_t uiEdgeuseIndex) const;
    uint32_t GetStagedEdgeuseNextRadialIndex(uint32_t uiEdgeuseIndex) const;
    uint32_t GetStagedEdgeuseEndVertexIndex(uint32_t uiEdgeuseIndex) const;
    uint32_t GetStagedImpliedBoundarySegmentEdgeIndex(uint32_t uiSegmentIndex) const;
    void SetStagedEdgeuse(uint32_t uiEdgeuseIndex, const StagedEdgeuseData& rEdgeuseData);
    void SetStagedEdgeuseNextRadialIndex(uint32_t uiEdgeuseIndex, uint32_t uiNextRadialEdgeuseIndex);

    StagedBrepEdgeVertexBuilder m_sEdgeVertexBuilder;
    StagedBrepTopologyBuilder m_sTopologyBuilder;
    BrepStagingState& m_rState;
    uint32_t m_uiAugmentedOuterLoopEdgeuseCount = 0;
    uint32_t m_uiOuterLoopEdgeuseStartIndex = 0;
};

#endif // BREP_NATURAL_BOUNDARY_BUILDER_H
