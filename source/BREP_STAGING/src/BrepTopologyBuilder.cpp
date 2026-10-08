// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepTopologyBuilder.h"

#include "UsdBrepArrayData.h"

#include <cassert>

using namespace pxr;

StagedBrepTopologyBuilder::StagedBrepTopologyBuilder(
    BrepStagingState& rState,
    uint32_t& rSolidRegionTargetIndex,
    uint32_t& rShellTargetIndex,
    uint32_t& rShellPairedSpacing,
    uint32_t& rInfiniteFaceuseTargetIndex,
    uint32_t& rFaceuseTargetIndex,
    uint32_t& rFaceusePairedSpacing
)
    : m_rState(rState),
      m_rSolidRegionTargetIndex(rSolidRegionTargetIndex),
      m_rShellTargetIndex(rShellTargetIndex),
      m_rShellPairedSpacing(rShellPairedSpacing),
      m_rInfiniteFaceuseTargetIndex(rInfiniteFaceuseTargetIndex),
      m_rFaceuseTargetIndex(rFaceuseTargetIndex),
      m_rFaceusePairedSpacing(rFaceusePairedSpacing)
{
}

void StagedBrepTopologyBuilder::ResetStagedTopologyTargetIndicesForBrep()
{
    m_rShellTargetIndex = 0;
    m_rFaceuseTargetIndex = 0;
    m_rInfiniteFaceuseTargetIndex = 0;
}

void StagedBrepTopologyBuilder::BeginStagedConnectedComponentTopology()
{
    // Capture this connected component's starting positions before shell traversal fills the staged arrays.
    m_rSolidRegionTargetIndex = m_rState.GetRegionCount();
    m_rShellTargetIndex = m_rState.GetShellCount();
    m_rFaceuseTargetIndex = m_rState.GetFaceuseCount();
    ResetStagedConnectedComponentTopologyPairedSpacing();
}

void StagedBrepTopologyBuilder::ResetStagedConnectedComponentTopologyPairedSpacing()
{
    m_rShellPairedSpacing = 0;
    m_rFaceusePairedSpacing = 0;
}

void StagedBrepTopologyBuilder::ReserveStagedConnectedComponentTopology(uint32_t uiShellBlockSize, uint32_t uiFaceuseBlockSize)
{
    m_rState.ResizeShellRecords(m_rState.GetShellCount() + uiShellBlockSize);
    m_rState.ResizeFaceuseRecords(m_rState.GetFaceuseCount() + uiFaceuseBlockSize);
}

void StagedBrepTopologyBuilder::GetStagedShellTargetIndicesForCurrentTraversal(
    bool bFirstShell,
    uint32_t uiConnectedComponentCount,
    bool bClosedShell,
    uint32_t& rFirstShellIndex,
    uint32_t& rSecondShellIndex
)
{
    rFirstShellIndex = 0;
    rSecondShellIndex = 0;

    if (bFirstShell && bClosedShell)
    {
        rFirstShellIndex = uiConnectedComponentCount - 1;
        rSecondShellIndex = m_rShellTargetIndex;
        m_rShellTargetIndex++;
        return;
    }

    if (bFirstShell && !bClosedShell)
    {
        rFirstShellIndex = uiConnectedComponentCount - 1;
        return;
    }

    rFirstShellIndex = m_rShellTargetIndex;
    if (bClosedShell)
    {
        rSecondShellIndex = m_rShellTargetIndex + m_rShellPairedSpacing;
    }
    else if (m_rShellPairedSpacing > 0)
    {
        m_rShellPairedSpacing--;
    }

    m_rShellTargetIndex++;
}

void StagedBrepTopologyBuilder::GetStagedFaceuseTargetIndicesForCurrentTraversal(
    bool bFirstShell,
    bool bClosedShell,
    uint32_t& rFirstFaceuseIndex,
    uint32_t& rSecondFaceuseIndex
)
{
    if (bFirstShell && bClosedShell)
    {
        rFirstFaceuseIndex = m_rInfiniteFaceuseTargetIndex;
        rSecondFaceuseIndex = m_rFaceuseTargetIndex;
        m_rInfiniteFaceuseTargetIndex++;
        m_rFaceuseTargetIndex++;
        return;
    }

    if (bFirstShell && !bClosedShell)
    {
        rFirstFaceuseIndex = m_rInfiniteFaceuseTargetIndex;
        rSecondFaceuseIndex = m_rInfiniteFaceuseTargetIndex + 1;
        m_rInfiniteFaceuseTargetIndex += 2;
        return;
    }

    rFirstFaceuseIndex = m_rFaceuseTargetIndex;
    if (bClosedShell)
    {
        rSecondFaceuseIndex = m_rFaceuseTargetIndex + m_rFaceusePairedSpacing;
        m_rFaceuseTargetIndex++;
        return;
    }

    rSecondFaceuseIndex = m_rFaceuseTargetIndex + 1;
    m_rFaceuseTargetIndex += 2;
    if (m_rFaceusePairedSpacing >= 2)
    {
        m_rFaceusePairedSpacing -= 2;
    }
}

StagedFaceuseBuildResult StagedBrepTopologyBuilder::AddStagedFaceusePairForCurrentTraversal(
    bool bFirstShell,
    bool bClosedShell,
    uint32_t uiFaceIndex,
    bool bSourceSameFaceOrient
)
{
    StagedFaceuseBuildResult sResult;
    sResult.bFirstFaceuseSame = bFirstShell ? bSourceSameFaceOrient : !bSourceSameFaceOrient;

    GetStagedFaceuseTargetIndicesForCurrentTraversal(bFirstShell, bClosedShell, sResult.uiFirstFaceuseIndex, sResult.uiSecondFaceuseIndex);
    SetStagedFaceusePairForFace(sResult.uiFirstFaceuseIndex, sResult.uiSecondFaceuseIndex, uiFaceIndex, sResult.bFirstFaceuseSame);
    return sResult;
}

void StagedBrepTopologyBuilder::AddStagedShellForCurrentTraversal(
    uint32_t uiParentRegionIndex,
    uint32_t ui1stShellIndex,
    uint32_t ui2ndShellIndex,
    uint32_t uiFaceCount,
    bool bClosedShell
)
{
    StagedShellData sShellData;
    sShellData.uiFaceuseCount = bClosedShell ? uiFaceCount : 2 * uiFaceCount;
    sShellData.uiWireEdgeCount = 0; // Shells with wire edges are not supported.
    sShellData.eShellPointType = StagedShellPointType::None;

    if (bClosedShell)
    {
        AddClosedStagedShellPair(uiParentRegionIndex, ui1stShellIndex, ui2ndShellIndex, sShellData);
    }
    else
    {
        AddOpenStagedShell(uiParentRegionIndex, ui1stShellIndex, sShellData);
    }
}

void StagedBrepTopologyBuilder::AddClosedStagedShellPair(
    uint32_t uiParentRegionIndex,
    uint32_t ui1stShellIndex,
    uint32_t ui2ndShellIndex,
    const StagedShellData& rShellData
)
{
    const StagedRegionData* pParentRegionData = m_rState.GetRegionRecord(uiParentRegionIndex);

    StagedRegionData sNestedRegionData;
    sNestedRegionData.uiShellCount = 0;
    sNestedRegionData.eRegionType = (pParentRegionData && pParentRegionData->eRegionType == StagedRegionType::Solid) ? StagedRegionType::Void :
                                                                                                                       StagedRegionType::Solid;

    uint32_t uiNestedRegionIndex = m_rState.AddRegionRecord(sNestedRegionData);

    if (StagedRegionData* pParentRegion = m_rState.GetRegionRecord(uiParentRegionIndex))
    {
        pParentRegion->uiShellCount++;
    }
    if (StagedShellData* pFirstShell = m_rState.GetShellRecord(ui1stShellIndex))
    {
        *pFirstShell = rShellData;
    }

    if (StagedRegionData* pNestedRegion = m_rState.GetRegionRecord(uiNestedRegionIndex))
    {
        pNestedRegion->uiShellCount++;
    }
    if (StagedShellData* pSecondShell = m_rState.GetShellRecord(ui2ndShellIndex))
    {
        *pSecondShell = rShellData;
    }
}

void StagedBrepTopologyBuilder::AddOpenStagedShell(uint32_t uiParentRegionIndex, uint32_t uiShellIndex, const StagedShellData& rShellData)
{
    if (StagedRegionData* pParentRegion = m_rState.GetRegionRecord(uiParentRegionIndex))
    {
        pParentRegion->uiShellCount++;
    }

    if (StagedShellData* pShell = m_rState.GetShellRecord(uiShellIndex))
    {
        *pShell = rShellData;
    }
}

void StagedBrepTopologyBuilder::SetStagedFaceusePairForFace(
    uint32_t ui1stFaceuseIndex,
    uint32_t ui2ndFaceuseIndex,
    uint32_t uiFaceIndex,
    bool bFirstFaceuseSame
)
{
    StagedFaceuseData sFaceuse1;
    StagedFaceuseData sFaceuse2;
    sFaceuse1.uiFaceIndex = uiFaceIndex;
    sFaceuse2.uiFaceIndex = uiFaceIndex;
    sFaceuse1.eOrientType = bFirstFaceuseSame ? StagedOrientType::Same : StagedOrientType::Opposite;
    sFaceuse2.eOrientType = bFirstFaceuseSame ? StagedOrientType::Opposite : StagedOrientType::Same;

    SetStagedFaceuses(ui1stFaceuseIndex, sFaceuse1, ui2ndFaceuseIndex, sFaceuse2);
}

void StagedBrepTopologyBuilder::SetStagedFaceuses(
    uint32_t ui1stFaceuseIndex,
    const StagedFaceuseData& rFaceuse1,
    uint32_t ui2ndFaceuseIndex,
    const StagedFaceuseData& rFaceuse2
)
{
    StagedFaceuseData* pFirstFaceuseData = m_rState.GetFaceuseRecord(ui1stFaceuseIndex);
    StagedFaceuseData* pSecondFaceuseData = m_rState.GetFaceuseRecord(ui2ndFaceuseIndex);
    if (!pFirstFaceuseData || !pSecondFaceuseData)
    {
        return;
    }

    *pFirstFaceuseData = rFaceuse1;
    *pSecondFaceuseData = rFaceuse2;
}

void StagedBrepTopologyBuilder::FlipPriorStagedFaceuseOrientationsForAugmentedFace(uint32_t uiFaceCountInCurrentShell)
{
    uint32_t uiStagedFaceCount = m_rState.GetFaceCount();
    uint32_t uiStagedFaceuseCount = m_rState.GetFaceuseCount();
    if (uiFaceCountInCurrentShell <= 1 || uiStagedFaceCount == 0)
    {
        return;
    }

    uint32_t uiFirstFaceInShell = uiStagedFaceCount - uiFaceCountInCurrentShell;
    uint32_t uiAugmentedFaceIndex = uiStagedFaceCount - 1;
    for (uint32_t uiFaceuseIndex = 0; uiFaceuseIndex < uiStagedFaceuseCount; uiFaceuseIndex++)
    {
        const StagedFaceuseData* pFaceuseData = m_rState.GetFaceuseRecord(uiFaceuseIndex);
        if (pFaceuseData && pFaceuseData->uiFaceIndex >= uiFirstFaceInShell && pFaceuseData->uiFaceIndex < uiAugmentedFaceIndex)
        {
            FlipStagedFaceuseOrientation(uiFaceuseIndex);
        }
    }
}

void StagedBrepTopologyBuilder::FlipStagedFaceuseOrientation(uint32_t uiFaceuseIndex)
{
    StagedFaceuseData* pFaceuseData = m_rState.GetFaceuseRecord(uiFaceuseIndex);
    if (!pFaceuseData)
    {
        return;
    }

    pFaceuseData->eOrientType = (pFaceuseData->eOrientType == StagedOrientType::Same) ? StagedOrientType::Opposite : StagedOrientType::Same;
}

void StagedBrepTopologyBuilder::AddStagedImpliedBoundaryLoop(uint32_t uiEdgeuseCount)
{
    StagedLoopData sLoopData;
    sLoopData.uiEdgeuseCount = uiEdgeuseCount;
    sLoopData.uiVertexIndex = static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);

    m_rState.AppendLoopRecord(sLoopData);
}

void StagedBrepTopologyBuilder::AppendStagedEdgeuseToRadialList(
    uint32_t uiEdgeuseIndex,
    uint32_t uiRadialHeadEdgeuseIndex,
    uint32_t uiRadialTailEdgeuseIndex
)
{
    StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex);
    StagedEdgeuseData* pRadialTailEdgeuseData = m_rState.GetEdgeuseRecord(uiRadialTailEdgeuseIndex);
    if (!pEdgeuseData || !pRadialTailEdgeuseData)
    {
        return;
    }

    pEdgeuseData->uiNextRadialEdgeuseIndex = uiRadialHeadEdgeuseIndex;
    pRadialTailEdgeuseData->uiNextRadialEdgeuseIndex = uiEdgeuseIndex;
}

void StagedBrepTopologyBuilder::RegisterStagedEdgeuseForEdge(uint32_t uiEdgeuseIndex, uint32_t uiEdgeIndex)
{
    StagedEdgeRadialValues* pEdgeValues = m_rState.GetEdgeRadialList(uiEdgeIndex);
    if (!pEdgeValues)
    {
        // StageEdgeForCurrentTraversal() seeds the radial list before later edgeuses register against the same staged edge.
        return;
    }

    const uint32_t uiRadialHeadEdgeuseIndex = pEdgeValues->uiRadialHead_EUIndex;
    const uint32_t uiRadialTailEdgeuseIndex = pEdgeValues->uiRadialTail_EUIndex;

    if (StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex))
    {
        pEdgeuseData->uiEdgeIndex = uiEdgeIndex;
    }
    AppendStagedEdgeuseToRadialList(uiEdgeuseIndex, uiRadialHeadEdgeuseIndex, uiRadialTailEdgeuseIndex);

    // Remember this edgeuse as the radial tail for the next coedge sharing this edge.
    pEdgeValues->uiRadialTail_EUIndex = uiEdgeuseIndex;
}

void StagedBrepTopologyBuilder::EnsureStagedEdgeRadialList(uint32_t uiEdgeIndex, uint32_t uiRadialHeadEdgeuseIndex, uint32_t uiRadialTailEdgeuseIndex)
{
    if (m_rState.HasEdgeRadialList(uiEdgeIndex))
    {
        return;
    }

    m_rState.SetEdgeRadialList(uiEdgeIndex, uiRadialHeadEdgeuseIndex, uiRadialTailEdgeuseIndex);
}

void StagedBrepTopologyBuilder::RemapStagedEdgeRadialList(
    uint32_t uiEdgeIndex,
    uint32_t uiOldEdgeuseStart,
    uint32_t uiOldEdgeuseCount,
    const std::vector<uint32_t>& rOldToNewEdgeuseIndices
)
{
    StagedEdgeRadialValues* pEdgeValues = m_rState.GetEdgeRadialList(uiEdgeIndex);
    if (!pEdgeValues)
    {
        return;
    }

    const uint32_t uiOldEdgeuseEnd = uiOldEdgeuseStart + uiOldEdgeuseCount;
    uint32_t& rHeadEdgeuseIndex = pEdgeValues->uiRadialHead_EUIndex;
    uint32_t& rTailEdgeuseIndex = pEdgeValues->uiRadialTail_EUIndex;
    if (rHeadEdgeuseIndex >= uiOldEdgeuseStart && rHeadEdgeuseIndex < uiOldEdgeuseEnd)
    {
        rHeadEdgeuseIndex = rOldToNewEdgeuseIndices[rHeadEdgeuseIndex - uiOldEdgeuseStart];
    }
    if (rTailEdgeuseIndex >= uiOldEdgeuseStart && rTailEdgeuseIndex < uiOldEdgeuseEnd)
    {
        rTailEdgeuseIndex = rOldToNewEdgeuseIndices[rTailEdgeuseIndex - uiOldEdgeuseStart];
    }
}
