// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_FACE_TRAVERSAL_PLANNER_H
#define BREP_FACE_TRAVERSAL_PLANNER_H

#include "BrepSourceData.h"
#include "BrepStatus.h"

#include <cstdint>
#include <vector>

// Resolve, from kernel-neutral inputs, which loop bounds a face (the "outer" loop) and whether the
// face's boundary is implied by the surface's natural parameter range. This is a staging decision that
// is identical across CAD kernels, so it lives in the shared layer and is fed neutral facts by each
// IBrepSource adapter (instead of being re-implemented per kernel).
//
// Decision order (source index is authoritative when in range):
//   index <  loopCount : explicit outer; trust it; ignore orientation.
//   index == loopCount : source declares implied natural boundary; trust it.
//   index >  loopCount : unknown — orientation fallback only:
//                          closed + all-CW → implied natural boundary;
//                          else recover a CCW outer if present;
//                          else flag out-of-range without inventing a boundary.
//
//   uiSourceOuterLoopIndex : the loop the source declares as outer; == uiLoopCount means "none /
//                            implied natural boundary"; > uiLoopCount is unknown/malformed.
//   bClosedSurface         : surface is closed in U and/or V (its natural range can imply a boundary).
//   bAllLoopsCW            : every explicit loop runs clockwise on the surface (so none is the outer).
//   uiCCWLoopCount         : number of effectively-CCW loops on the surface.
//   vLoopEffectivelyCCW    : per-loop "effectively CCW on the surface" flags (size uiLoopCount); only
//                            consulted in the unknown-index orientation fallback.
//   vLoopCoedgeCount       : per-loop coedge count (size uiLoopCount), used to report the outer loop's
//                            edgeuse count.
BrepStatus PlanFaceOuterLoop(
    uint32_t uiLoopCount,
    uint32_t uiSourceOuterLoopIndex,
    bool bClosedSurface,
    bool bAllLoopsCW,
    uint32_t uiCCWLoopCount,
    const std::vector<bool>& vLoopEffectivelyCCW,
    const std::vector<uint32_t>& vLoopCoedgeCount,
    SourceFaceOuterLoopData& rOuterLoopData
);

struct StagedFacePlanningResult
{
    bool bMarkStandaloneImpliedLoopProcessed = false;
    bool bSetStandaloneImpliedOuterLoopEdgeuseCount = false;
    bool bExtractImpliedBoundaryEdges = false;
    bool bAddStandaloneImpliedBoundaryLoop = false;
    bool bMarkShellHasAugmentedFace = false;
    bool bFlipPriorFaceuseOrientationsForAugmentedFace = false;
    bool bSetAugmentedOuterLoopForCurrentFace = false;
    bool bSetCCWLoopAsExplicitOuterLoop = false;
    uint32_t uiCCWLoopExplicitOuterSourceLoopIndex = kStagedNoObjectIndex;
    uint32_t uiSelectedBoundarySourceLoopCoedgeCount = 0;
    bool bLoopCountChanged = false;
    uint32_t uiLoopCount = 0;
};

class StagedFaceTraversalPlanner
{
public:

    StagedFaceTraversalPlanner(BrepStagingState& rState, const SourceFaceData& rSourceFaceData, uint32_t uiFaceIndex);

    BrepStatus PlanCurrentFace(
        uint32_t& rLoopCount,
        const SourceFaceBoundaryContactData& rBoundaryContactData,
        StagedFacePlanningResult& rPlanningResult
    );

private:

    void ResetImpliedBoundaryState();
    void SetImpliedBoundaryCase(uint32_t uiTouchingCWCount, uint32_t uiTouchingCCWCount);
    bool IsImpliedBoundaryCase(ImpliedBoundaryCase eCase) const;
    bool NeedsImpliedBoundaryEdges() const;
    void SetImpliedBoundaryAugmentTarget(uint32_t uiSourceLoopIndex, uint32_t uiContactVertexIndex);
    void SetCurrentFaceLoopCount(uint32_t uiLoopCount);

    BrepStagingState& m_rState;
    const SourceFaceData& m_rSourceFaceData;
    uint32_t m_uiFaceIndex = 0;
};

#endif // BREP_FACE_TRAVERSAL_PLANNER_H
