// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepEdgeVertexBuilder.h"

#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/diagnostic.h>
#include <pxr/base/tf/stringUtils.h>
#include <pxr/base/vt/array.h>

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

using namespace pxr;

StagedBrepEdgeVertexBuilder::StagedBrepEdgeVertexBuilder(BrepStagingState& rState) : m_rState(rState)
{
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedLineCurve(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rDirection)
{
    StagedCurveData sCurveData;
    sCurveData.eCurveType = StagedCurveType::Line;
    sCurveData.sOrigin = rOrigin;
    sCurveData.sDirection = rDirection;

    return AddStagedEdgeCurve(sCurveData);
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedCircleCurve(
    const pxr::GfVec3d& rCenter,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dRadius
)
{
    StagedCurveData sCurveData;
    sCurveData.eCurveType = StagedCurveType::Circle;
    sCurveData.sCenter = rCenter;
    sCurveData.sAxis = rAxis;
    sCurveData.sRefDirection = rRefDirection;
    sCurveData.dRadius = dRadius;

    return AddStagedEdgeCurve(sCurveData);
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedBSplineCurve(
    uint32_t uiVertexCount,
    uint32_t uiOrder,
    const pxr::VtArray<double>& rKnots,
    const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
    const pxr::VtArray<double>& rWeights
)
{
    StagedCurveData sCurveData;
    sCurveData.eCurveType = StagedCurveType::BSplineCurve;
    sCurveData.uiNurbVertexCount = uiVertexCount;
    sCurveData.uiNurbOrder = uiOrder;
    sCurveData.vNurbKnots = rKnots;
    sCurveData.vNurbControlVertices = rControlVertices;
    sCurveData.vNurbWeights = rWeights;

    return AddStagedEdgeCurve(sCurveData);
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedEdgeCurve(const StagedCurveData& rCurveData)
{
    m_rState.AppendEdgeCurveRecord(rCurveData);
    return m_rState.GetEdgeCurveCount() - 1;
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedEdgeForCurrentTraversal(
    StagedCurveType eCurveType,
    const pxr::GfRange1d& rInterval,
    uint32_t uiStartVertexIndex,
    uint32_t uiEndVertexIndex
)
{
    StagedEdgeData sEdgeData;
    sEdgeData.eCurveType = eCurveType;
    sEdgeData.sInterval = rInterval;
    sEdgeData.uiStartVertexIndex = uiStartVertexIndex;
    sEdgeData.uiEndVertexIndex = uiEndVertexIndex;

    return AddStagedEdge(sEdgeData);
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedEdge(const StagedEdgeData& rEdgeData)
{
    m_rState.AppendEdgeRecord(rEdgeData);
    return m_rState.GetEdgeCount() - 1;
}


BrepStatus StagedBrepEdgeVertexBuilder::AddSourceVertex(const SourceVertexData& rVertexData, uint32_t& rVertexIndex)
{
    if (!rVertexData.bHasPoint)
    {
        return BrepStatusError;
    }

    rVertexIndex = AddStagedVertexPoint(rVertexData.sPoint);
    return BrepStatusSuccess;
}


BrepStatus StagedBrepEdgeVertexBuilder::AddSourceEdgeForCurrentTraversal(
    const SourceEdgeData& rEdgeData,
    uint32_t uiStartVertexIndex,
    uint32_t uiEndVertexIndex,
    pxr::GfRange1d& rEdgeInterval
)
{
    AddStagedEdgeCurve(rEdgeData.sCurve.sCurveData);
    rEdgeInterval = rEdgeData.sCurve.sInterval;
    AddStagedEdgeForCurrentTraversal(rEdgeData.sCurve.sCurveData.eCurveType, rEdgeInterval, uiStartVertexIndex, uiEndVertexIndex);
    return BrepStatusSuccess;
}


bool StagedBrepEdgeVertexBuilder::TryGetStagedVertexPoint(uint32_t uiVertexIndex, pxr::GfVec3d& rPoint) const
{
    const pxr::GfVec3d* pPoint = m_rState.GetVertexPointRecord(uiVertexIndex);
    if (!pPoint)
    {
        return false;
    }

    rPoint = *pPoint;
    return true;
}


uint32_t StagedBrepEdgeVertexBuilder::AddStagedVertexPoint(const pxr::GfVec3d& rPoint)
{
    StagedVertexData sVertexData;
    sVertexData.eVertexType = StagedVertexType::VertexPoint;
    m_rState.AppendVertexRecord(sVertexData);

    m_rState.AppendVertexPointRecord(rPoint);

    return m_rState.GetVertexCount() - 1;
}


void StagedBrepEdgeVertexBuilder::AddStagedImpliedBoundaryEdgeuse(
    uint32_t uiEdgeIndex,
    StagedOrientType eOrientType,
    StagedRadialEntryType eEntrySideType
)
{
    StagedEdgeuseData sEdgeuseData;
    sEdgeuseData.uiEdgeIndex = uiEdgeIndex;
    sEdgeuseData.eOrientType = eOrientType;
    sEdgeuseData.uiNextRadialEdgeuseIndex = 0;
    sEdgeuseData.eEntrySideType = eEntrySideType;

    m_rState.AppendImpliedBoundarySegment(sEdgeuseData);
}


void StagedBrepEdgeVertexBuilder::AddStagedImpliedBoundaryEdgeuses(uint32_t uiFirstEdgeuseIndex)
{
    uint32_t uiSegmentCount = m_rState.GetImpliedBoundarySegmentCount();
    for (uint32_t ii = 0; ii < uiSegmentCount; ++ii)
    {
        const StagedEdgeuseData* pEdgeuseData = m_rState.GetImpliedBoundarySegment(ii);
        if (!pEdgeuseData)
        {
            continue;
        }

        StagedEdgeuseData sEdgeuseData = *pEdgeuseData;
        sEdgeuseData.uiNextRadialEdgeuseIndex = uiFirstEdgeuseIndex + ii;
        m_rState.AppendEdgeuseRecord(sEdgeuseData);
    }
}


uint32_t StagedBrepEdgeVertexBuilder::SetStagedImpliedBoundaryEdgeuses(uint32_t uiFirstEdgeuseIndex)
{
    uint32_t uiWriteIndex = uiFirstEdgeuseIndex;
    uint32_t uiSegmentCount = m_rState.GetImpliedBoundarySegmentCount();
    for (uint32_t ii = 0; ii < uiSegmentCount; ++ii)
    {
        const StagedEdgeuseData* pEdgeuseData = m_rState.GetImpliedBoundarySegment(ii);
        if (!pEdgeuseData)
        {
            continue;
        }

        StagedEdgeuseData sEdgeuseData = *pEdgeuseData;
        sEdgeuseData.uiNextRadialEdgeuseIndex = uiWriteIndex;
        StagedEdgeuseData* pWriteEdgeuseData = m_rState.GetEdgeuseRecord(uiWriteIndex++);
        if (pWriteEdgeuseData)
        {
            *pWriteEdgeuseData = sEdgeuseData;
        }
    }

    return uiWriteIndex;
}
