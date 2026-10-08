// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepStagingBuilder.h"

#include <limits>

GenericBrepStagingBuilder::GenericBrepStagingBuilder(BrepStagingState& rState) : m_rState(rState)
{
}

void GenericBrepStagingBuilder::ResetForSourceBrep(const SourceBrepData& rBrepData)
{
    static const double dMax = std::numeric_limits<double>::max();
    m_rState.SetTolerance(rBrepData.dTolerance);
    m_rState.SetExtent(pxr::GfVec3d(dMax, dMax, dMax), pxr::GfVec3d(-dMax, -dMax, -dMax));
    m_rState.ClearCollections();

    m_rState.ResizeRegionRecords(1);
    m_rState.ResizeShellRecords(rBrepData.uiConnectedComponentCount);
    m_rState.ResizeFaceuseRecords(rBrepData.uiInfiniteRegionFaceuseCount);

    StagedRegionData sInfiniteRegionData;
    sInfiniteRegionData.uiShellCount = 0;
    sInfiniteRegionData.eRegionType = StagedRegionType::Void;
    if (StagedRegionData* pRegionData = m_rState.GetRegionRecord(0))
    {
        *pRegionData = sInfiniteRegionData;
    }
}

uint32_t GenericBrepStagingBuilder::AddFace(const SourceFaceData& rFaceData, uint32_t uiLoopCount, StagedTrimType eTrimType)
{
    StagedFaceData sFaceData;
    sFaceData.uiLoopCount = uiLoopCount;
    sFaceData.eTrimType = eTrimType;
    sFaceData.sUVDomain = rFaceData.sDomain;

    m_rState.AppendFaceRecord(sFaceData);
    return m_rState.GetFaceCount() - 1;
}

uint32_t GenericBrepStagingBuilder::AddFaceSurface(const SourceSurfaceData& rSurfaceData)
{
    m_rState.AppendFaceSurfaceRecord(rSurfaceData.sSurfaceData);
    return m_rState.GetFaceSurfaceCount() - 1;
}

uint32_t GenericBrepStagingBuilder::AddFaceSurfaceForCurrentFace(const SourceSurfaceData& rSurfaceData, StagedTrimType eTrimType)
{
    const uint32_t uiSurfaceIndex = AddFaceSurface(rSurfaceData);
    SetCurrentFaceTrimAndDomain(eTrimType, rSurfaceData.sDomain);
    if (StagedFaceData* pFaceData = GetCurrentFace())
    {
        pFaceData->eSurfaceType = rSurfaceData.sSurfaceData.eSurfaceType;
    }
    return uiSurfaceIndex;
}

void GenericBrepStagingBuilder::SetCurrentFaceTrimAndDomain(StagedTrimType eTrimType, const pxr::GfRange2d& rUVDomain)
{
    StagedFaceData* pFaceData = GetCurrentFace();
    if (!pFaceData)
    {
        return;
    }

    pFaceData->eTrimType = eTrimType;
    pFaceData->sUVDomain = rUVDomain;
}

void GenericBrepStagingBuilder::SetCurrentFaceLoopCount(uint32_t uiLoopCount)
{
    StagedFaceData* pFaceData = GetCurrentFace();
    if (!pFaceData)
    {
        return;
    }

    pFaceData->uiLoopCount = uiLoopCount;
}

void GenericBrepStagingBuilder::AddLoopForCurrentTraversal(
    const StagedLoopData& rLoopData,
    bool bIsOuterLoop,
    uint32_t uiCurrentLoopTraversalCount,
    bool bHasStagedImpliedNaturalBoundary
)
{
    bool bInsertedLoop = false;

    if (uiCurrentLoopTraversalCount == 1 && !bIsOuterLoop && !bHasStagedImpliedNaturalBoundary)
    {
        m_rState.AppendLoopRecord(rLoopData);
        bInsertedLoop = true;
    }

    if (bIsOuterLoop)
    {
        const uint32_t uiInsertIndex = m_rState.GetLoopCount() - uiCurrentLoopTraversalCount + 1;
        m_rState.InsertLoopRecordAt(uiInsertIndex, rLoopData);
    }
    else if (!bInsertedLoop)
    {
        m_rState.AppendLoopRecord(rLoopData);
    }
}

void GenericBrepStagingBuilder::AddSourceLoopForCurrentTraversal(
    const SourceLoopData& rLoopData,
    bool bIsAugmentedOuterLoop,
    uint32_t uiAugmentedOuterLoopEdgeuseCount,
    bool bIsOuterLoop,
    uint32_t uiCurrentLoopTraversalCount,
    bool bHasStagedImpliedNaturalBoundary
)
{
    StagedLoopData sLoopData;
    sLoopData.uiEdgeuseCount = bIsAugmentedOuterLoop ? uiAugmentedOuterLoopEdgeuseCount : rLoopData.uiCoedgeCount;
    sLoopData.uiVertexIndex = 0; // Vertex loops are not currently staged.
    AddLoopForCurrentTraversal(sLoopData, bIsOuterLoop, uiCurrentLoopTraversalCount, bHasStagedImpliedNaturalBoundary);
}

StagedFaceData* GenericBrepStagingBuilder::GetCurrentFace()
{
    uint32_t uiFaceCount = m_rState.GetFaceCount();
    if (uiFaceCount == 0)
    {
        return nullptr;
    }

    return m_rState.GetFaceRecord(uiFaceCount - 1);
}
