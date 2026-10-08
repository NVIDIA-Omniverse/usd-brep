// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_COEDGE_BUILDER_H
#define BREP_COEDGE_BUILDER_H

#include "BrepSourceData.h"
#include "BrepStagingState.h"

struct StagedCoedgeBuildResult
{
    uint32_t uiEdgeuseTargetIndex = kStagedNoObjectIndex;
    bool bFirstCoedge = false;
    bool bExplicitOuterLoop = false;
    bool bReverseStagedLoop = false;
    bool bStagedEdgeuseMatchesSourceOrient = false;
    bool bBottomSideEntry = false;
};

class StagedBrepCoedgeBuilder
{
public:

    StagedBrepCoedgeBuilder(
        BrepStagingState& rState,
        uint32_t uiCurrentLoopTraversalCount,
        uint32_t uiCurrentCoedgeTraversalCount,
        uint32_t uiOuterLoopSourceIndex,
        uint32_t uiOuterLoopEdgeuseStartIndex,
        uint32_t uiOuterLoopEdgeuseCount,
        bool bHasImpliedNaturalBoundary
    );

    bool IsFirstLoop() const;
    bool IsFirstCoedge() const;
    bool IsCurrentLoopExplicitOuterLoop() const;
    bool IsCurrentLoopAugmentedOuterLoop() const;
    bool IsCurrentOuterLoop() const;
    uint32_t GetReserveCountForCurrentLoop(uint32_t uiLoopEdgeuseCount) const;
    bool ShouldReverseLoop(bool bSourceSameLoopOrient) const;
    bool ShouldEdgeuseMatchSourceOrientation(bool bReverseLoop, bool bSourceSameEdgeOrient) const;
    bool ShouldEdgeuseEnterBottomSide(bool bSameEdgeuseOrient) const;
    uint32_t GetEdgeuseTargetIndexForCurrentCoedge(bool bReverseLoop, uint32_t uiLoopEdgeuseCount) const;
    void ReserveEdgeuses(uint32_t uiAdditionalEdgeuseCount);
    void SetEdgeuseForCurrentTraversal(uint32_t uiEdgeuseIndex, bool bSameEdgeuseOrient, bool bBottomSideEntry);
    StagedCoedgeBuildResult AddSourceCoedgeForCurrentTraversal(const SourceLoopData& rLoopData, const SourceCoedgeData& rCoedgeData);

private:

    bool ShouldReserveAugmentedOuterLoopForFirstInnerLoop(bool bFirstLoop, bool bIsOuterLoop) const;

    BrepStagingState& m_rState;
    uint32_t m_uiCurrentLoopTraversalCount = 0;
    uint32_t m_uiCurrentCoedgeTraversalCount = 0;
    uint32_t m_uiOuterLoopSourceIndex = 0;
    uint32_t m_uiOuterLoopEdgeuseStartIndex = 0;
    uint32_t m_uiOuterLoopEdgeuseCount = 0;
    bool m_bHasImpliedNaturalBoundary = false;
};

#endif // BREP_COEDGE_BUILDER_H
