// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_TOPOLOGY_BUILDER_H
#define BREP_TOPOLOGY_BUILDER_H

#include "BrepStagingState.h"

#include <cstdint>
#include <vector>

namespace UsdBrepData
{
class UsdBrepBuilder;
}

struct StagedFaceuseBuildResult
{
    uint32_t uiFirstFaceuseIndex = kStagedNoObjectIndex;
    uint32_t uiSecondFaceuseIndex = kStagedNoObjectIndex;
    bool bFirstFaceuseSame = false;
};

class StagedBrepTopologyBuilder
{
public:

    StagedBrepTopologyBuilder(
        BrepStagingState& rState,
        uint32_t& rSolidRegionTargetIndex,
        uint32_t& rShellTargetIndex,
        uint32_t& rShellPairedSpacing,
        uint32_t& rInfiniteFaceuseTargetIndex,
        uint32_t& rFaceuseTargetIndex,
        uint32_t& rFaceusePairedSpacing
    );

    void ResetStagedTopologyTargetIndicesForBrep();
    void BeginStagedConnectedComponentTopology();
    void ResetStagedConnectedComponentTopologyPairedSpacing();
    void ReserveStagedConnectedComponentTopology(uint32_t uiShellBlockSize, uint32_t uiFaceuseBlockSize);
    void AddStagedShellForCurrentTraversal(
        uint32_t uiParentRegionIndex,
        uint32_t ui1stShellIndex,
        uint32_t ui2ndShellIndex,
        uint32_t uiFaceCount,
        bool bClosedShell
    );
    void AddClosedStagedShellPair(uint32_t uiParentRegionIndex, uint32_t ui1stShellIndex, uint32_t ui2ndShellIndex, const StagedShellData& rShellData);
    void AddOpenStagedShell(uint32_t uiParentRegionIndex, uint32_t uiShellIndex, const StagedShellData& rShellData);
    void GetStagedShellTargetIndicesForCurrentTraversal(
        bool bFirstShell,
        uint32_t uiConnectedComponentCount,
        bool bClosedShell,
        uint32_t& rFirstShellIndex,
        uint32_t& rSecondShellIndex
    );
    void GetStagedFaceuseTargetIndicesForCurrentTraversal(
        bool bFirstShell,
        bool bClosedShell,
        uint32_t& rFirstFaceuseIndex,
        uint32_t& rSecondFaceuseIndex
    );
    StagedFaceuseBuildResult AddStagedFaceusePairForCurrentTraversal(
        bool bFirstShell,
        bool bClosedShell,
        uint32_t uiFaceIndex,
        bool bSourceSameFaceOrient
    );
    void SetStagedFaceusePairForFace(uint32_t ui1stFaceuseIndex, uint32_t ui2ndFaceuseIndex, uint32_t uiFaceIndex, bool bFirstFaceuseSame);
    void SetStagedFaceuses(
        uint32_t ui1stFaceuseIndex,
        const StagedFaceuseData& rFaceuse1,
        uint32_t ui2ndFaceuseIndex,
        const StagedFaceuseData& rFaceuse2
    );
    void FlipPriorStagedFaceuseOrientationsForAugmentedFace(uint32_t uiFaceCountInCurrentShell);
    void FlipStagedFaceuseOrientation(uint32_t uiFaceuseIndex);
    void AddStagedImpliedBoundaryLoop(uint32_t uiEdgeuseCount);
    void RegisterStagedEdgeuseForEdge(uint32_t uiEdgeuseIndex, uint32_t uiEdgeIndex);
    void AppendStagedEdgeuseToRadialList(uint32_t uiEdgeuseIndex, uint32_t uiRadialHeadEdgeuseIndex, uint32_t uiRadialTailEdgeuseIndex);
    void EnsureStagedEdgeRadialList(uint32_t uiEdgeIndex, uint32_t uiRadialHeadEdgeuseIndex, uint32_t uiRadialTailEdgeuseIndex);
    void RemapStagedEdgeRadialList(
        uint32_t uiEdgeIndex,
        uint32_t uiOldEdgeuseStart,
        uint32_t uiOldEdgeuseCount,
        const std::vector<uint32_t>& rOldToNewEdgeuseIndices
    );

private:

    BrepStagingState& m_rState;
    uint32_t& m_rSolidRegionTargetIndex;
    uint32_t& m_rShellTargetIndex;
    uint32_t& m_rShellPairedSpacing;
    uint32_t& m_rInfiniteFaceuseTargetIndex;
    uint32_t& m_rFaceuseTargetIndex;
    uint32_t& m_rFaceusePairedSpacing;
};

#endif // BREP_TOPOLOGY_BUILDER_H
