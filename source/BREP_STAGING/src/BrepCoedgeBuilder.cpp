// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepCoedgeBuilder.h"

#include <cassert>

StagedBrepCoedgeBuilder::StagedBrepCoedgeBuilder(
    BrepStagingState& rState,
    uint32_t uiCurrentLoopTraversalCount,
    uint32_t uiCurrentCoedgeTraversalCount,
    uint32_t uiOuterLoopSourceIndex,
    uint32_t uiOuterLoopEdgeuseStartIndex,
    uint32_t uiOuterLoopEdgeuseCount,
    bool bHasImpliedNaturalBoundary
)
    : m_rState(rState),
      m_uiCurrentLoopTraversalCount(uiCurrentLoopTraversalCount),
      m_uiCurrentCoedgeTraversalCount(uiCurrentCoedgeTraversalCount),
      m_uiOuterLoopSourceIndex(uiOuterLoopSourceIndex),
      m_uiOuterLoopEdgeuseStartIndex(uiOuterLoopEdgeuseStartIndex),
      m_uiOuterLoopEdgeuseCount(uiOuterLoopEdgeuseCount),
      m_bHasImpliedNaturalBoundary(bHasImpliedNaturalBoundary)
{
}

bool StagedBrepCoedgeBuilder::IsFirstLoop() const
{
    return m_uiCurrentLoopTraversalCount == 1;
}

bool StagedBrepCoedgeBuilder::IsFirstCoedge() const
{
    return m_uiCurrentCoedgeTraversalCount == 1;
}

bool StagedBrepCoedgeBuilder::IsCurrentLoopExplicitOuterLoop() const
{
    return !m_bHasImpliedNaturalBoundary && (m_uiCurrentLoopTraversalCount == m_uiOuterLoopSourceIndex + 1);
}

bool StagedBrepCoedgeBuilder::IsCurrentLoopAugmentedOuterLoop() const
{
    return m_rState.GetImpliedBoundaryCase() == BoundaryCase_AugmentCWLoop &&
           (m_uiCurrentLoopTraversalCount == m_rState.GetImpliedBoundaryAugmentSourceLoopIndex() + 1);
}

bool StagedBrepCoedgeBuilder::IsCurrentOuterLoop() const
{
    return IsCurrentLoopExplicitOuterLoop() || IsCurrentLoopAugmentedOuterLoop();
}

uint32_t StagedBrepCoedgeBuilder::GetReserveCountForCurrentLoop(uint32_t uiLoopEdgeuseCount) const
{
    const bool bFirstLoop = IsFirstLoop();
    const bool bIsOuterLoop = IsCurrentOuterLoop();

    if (IsCurrentLoopAugmentedOuterLoop())
    {
        return m_uiOuterLoopEdgeuseCount;
    }

    if ((bFirstLoop && !bIsOuterLoop && !m_bHasImpliedNaturalBoundary) || ShouldReserveAugmentedOuterLoopForFirstInnerLoop(bFirstLoop, bIsOuterLoop))
    {
        return uiLoopEdgeuseCount + m_uiOuterLoopEdgeuseCount;
    }

    if (!bFirstLoop && bIsOuterLoop)
    {
        return 0;
    }

    return uiLoopEdgeuseCount;
}

bool StagedBrepCoedgeBuilder::ShouldReverseLoop(bool bSourceSameLoopOrient) const
{
    // On a face with an explicit outer loop, some CAD sources store every loop (outer and inner) in
    // their use-direction relative to the surface, so the staged USD reversal must follow each loop's own
    // orientation-with-surface uniformly. Applying a different rule to inner loops over-reverses holes,
    // making them circulate the same way as the outer loop and breaking trimmed-face triangulation.
    // The implied-natural-boundary (periodic/seam) path keeps its prior behavior.
    if (m_bHasImpliedNaturalBoundary)
    {
        return bSourceSameLoopOrient;
    }
    return !bSourceSameLoopOrient;
}

bool StagedBrepCoedgeBuilder::ShouldEdgeuseMatchSourceOrientation(bool bReverseLoop, bool bSourceSameEdgeOrient) const
{
    return (!bReverseLoop && bSourceSameEdgeOrient) || (bReverseLoop && !bSourceSameEdgeOrient);
}

bool StagedBrepCoedgeBuilder::ShouldEdgeuseEnterBottomSide(bool bSameEdgeuseOrient) const
{
    return bSameEdgeuseOrient;
}

uint32_t StagedBrepCoedgeBuilder::GetEdgeuseTargetIndexForCurrentCoedge(bool bReverseLoop, uint32_t uiLoopEdgeuseCount) const
{
    const bool bIsOuterLoop = IsCurrentOuterLoop();

    if (!bReverseLoop && bIsOuterLoop)
    {
        return m_uiOuterLoopEdgeuseStartIndex + m_uiCurrentCoedgeTraversalCount - 1;
    }

    if (!bReverseLoop && !bIsOuterLoop)
    {
        return m_rState.GetEdgeuseCount() + m_uiCurrentCoedgeTraversalCount - uiLoopEdgeuseCount - 1;
    }

    if (bReverseLoop && bIsOuterLoop)
    {
        return m_uiOuterLoopEdgeuseStartIndex + uiLoopEdgeuseCount - m_uiCurrentCoedgeTraversalCount;
    }

    return m_rState.GetEdgeuseCount() - m_uiCurrentCoedgeTraversalCount;
}

void StagedBrepCoedgeBuilder::ReserveEdgeuses(uint32_t uiAdditionalEdgeuseCount)
{
    if (uiAdditionalEdgeuseCount == 0)
    {
        return;
    }

    m_rState.ResizeEdgeuseRecords(m_rState.GetEdgeuseCount() + uiAdditionalEdgeuseCount);
}

void StagedBrepCoedgeBuilder::SetEdgeuseForCurrentTraversal(uint32_t uiEdgeuseIndex, bool bSameEdgeuseOrient, bool bBottomSideEntry)
{
    StagedEdgeuseData sEdgeuseData;
    sEdgeuseData.eOrientType = bSameEdgeuseOrient ? StagedOrientType::Same : StagedOrientType::Opposite;
    sEdgeuseData.uiNextRadialEdgeuseIndex = uiEdgeuseIndex;
    sEdgeuseData.eEntrySideType = bBottomSideEntry ? StagedRadialEntryType::BottomSideEntry : StagedRadialEntryType::TopSideEntry;

    if (StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex))
    {
        *pEdgeuseData = sEdgeuseData;
    }
}

StagedCoedgeBuildResult StagedBrepCoedgeBuilder::AddSourceCoedgeForCurrentTraversal(
    const SourceLoopData& rLoopData,
    const SourceCoedgeData& rCoedgeData
)
{
    StagedCoedgeBuildResult sResult;
    sResult.bFirstCoedge = IsFirstCoedge();

    const uint32_t uiLoopEdgeuseCount = rLoopData.uiCoedgeCount;
    if (sResult.bFirstCoedge)
    {
        ReserveEdgeuses(GetReserveCountForCurrentLoop(uiLoopEdgeuseCount));
    }

    // Source loop orientation may be stored opposite of its BREP use; UsdBrep stores the loop in use direction.
    sResult.bExplicitOuterLoop = IsCurrentLoopExplicitOuterLoop();
    sResult.bReverseStagedLoop = ShouldReverseLoop(rLoopData.bSameOrientationWithSurface);
    sResult.bStagedEdgeuseMatchesSourceOrient = ShouldEdgeuseMatchSourceOrientation(sResult.bReverseStagedLoop, rCoedgeData.bSameOrientationWithLoop);
    sResult.bBottomSideEntry = ShouldEdgeuseEnterBottomSide(sResult.bStagedEdgeuseMatchesSourceOrient);
    sResult.uiEdgeuseTargetIndex = GetEdgeuseTargetIndexForCurrentCoedge(sResult.bReverseStagedLoop, uiLoopEdgeuseCount);

    assert(sResult.uiEdgeuseTargetIndex < m_rState.GetEdgeuseCount());
    SetEdgeuseForCurrentTraversal(sResult.uiEdgeuseTargetIndex, sResult.bStagedEdgeuseMatchesSourceOrient, sResult.bBottomSideEntry);
    return sResult;
}

bool StagedBrepCoedgeBuilder::ShouldReserveAugmentedOuterLoopForFirstInnerLoop(bool bFirstLoop, bool bIsOuterLoop) const
{
    return bFirstLoop && !bIsOuterLoop && m_rState.GetImpliedBoundaryCase() == BoundaryCase_AugmentCWLoop;
}
