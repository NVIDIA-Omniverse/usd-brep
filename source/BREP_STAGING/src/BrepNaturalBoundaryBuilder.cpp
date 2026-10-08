// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepNaturalBoundaryBuilder.h"

#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/vec3d.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <vector>

using namespace pxr;

NaturalBoundaryBuilder::NaturalBoundaryBuilder(
    BrepStagingState& rState,
    StagedBrepEdgeVertexBuilder sEdgeVertexBuilder,
    StagedBrepTopologyBuilder sTopologyBuilder,
    uint32_t uiAugmentedOuterLoopEdgeuseCount,
    uint32_t uiOuterLoopEdgeuseStartIndex
)
    : m_sEdgeVertexBuilder(sEdgeVertexBuilder),
      m_sTopologyBuilder(sTopologyBuilder),
      m_rState(rState),
      m_uiAugmentedOuterLoopEdgeuseCount(uiAugmentedOuterLoopEdgeuseCount),
      m_uiOuterLoopEdgeuseStartIndex(uiOuterLoopEdgeuseStartIndex)
{
}

StagedFaceData* NaturalBoundaryBuilder::GetCurrentStagedFace()
{
    const uint32_t uiFaceCount = m_rState.GetFaceCount();
    return uiFaceCount > 0 ? m_rState.GetFaceRecord(uiFaceCount - 1) : nullptr;
}

const StagedFaceData* NaturalBoundaryBuilder::GetCurrentStagedFace() const
{
    const uint32_t uiFaceCount = m_rState.GetFaceCount();
    return uiFaceCount > 0 ? m_rState.GetFaceRecord(uiFaceCount - 1) : nullptr;
}

pxr::GfRange2d NaturalBoundaryBuilder::GetCurrentStagedFaceDomain() const
{
    const StagedFaceData* pFaceData = GetCurrentStagedFace();
    return pFaceData ? pFaceData->sUVDomain : pxr::GfRange2d();
}

StagedSurfaceData* NaturalBoundaryBuilder::GetCurrentStagedFaceSurface()
{
    const uint32_t uiSurfaceCount = m_rState.GetFaceSurfaceCount();
    return uiSurfaceCount > 0 ? m_rState.GetFaceSurfaceRecord(uiSurfaceCount - 1) : nullptr;
}

StagedEdgeuseData NaturalBoundaryBuilder::GetStagedEdgeuse(uint32_t uiEdgeuseIndex) const
{
    const StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex);
    return pEdgeuseData ? *pEdgeuseData : StagedEdgeuseData();
}

uint32_t NaturalBoundaryBuilder::GetStagedEdgeuseEdgeIndex(uint32_t uiEdgeuseIndex) const
{
    return GetStagedEdgeuse(uiEdgeuseIndex).uiEdgeIndex;
}

uint32_t NaturalBoundaryBuilder::GetStagedEdgeuseNextRadialIndex(uint32_t uiEdgeuseIndex) const
{
    return GetStagedEdgeuse(uiEdgeuseIndex).uiNextRadialEdgeuseIndex;
}

uint32_t NaturalBoundaryBuilder::GetStagedEdgeuseEndVertexIndex(uint32_t uiEdgeuseIndex) const
{
    const StagedEdgeuseData sEdgeuseData = GetStagedEdgeuse(uiEdgeuseIndex);
    const StagedEdgeData* pEdgeData = m_rState.GetEdgeRecord(sEdgeuseData.uiEdgeIndex);
    if (!pEdgeData)
    {
        return kStagedNoObjectIndex;
    }

    return (sEdgeuseData.eOrientType == StagedOrientType::Same) ? pEdgeData->uiEndVertexIndex : pEdgeData->uiStartVertexIndex;
}

uint32_t NaturalBoundaryBuilder::GetStagedImpliedBoundarySegmentEdgeIndex(uint32_t uiSegmentIndex) const
{
    const StagedEdgeuseData* pEdgeuseData = m_rState.GetImpliedBoundarySegment(uiSegmentIndex);
    return pEdgeuseData ? pEdgeuseData->uiEdgeIndex : kStagedNoObjectIndex;
}

void NaturalBoundaryBuilder::SetStagedEdgeuse(uint32_t uiEdgeuseIndex, const StagedEdgeuseData& rEdgeuseData)
{
    StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex);
    if (!pEdgeuseData)
    {
        return;
    }

    *pEdgeuseData = rEdgeuseData;
}

void NaturalBoundaryBuilder::SetStagedEdgeuseNextRadialIndex(uint32_t uiEdgeuseIndex, uint32_t uiNextRadialEdgeuseIndex)
{
    StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex);
    if (!pEdgeuseData)
    {
        return;
    }

    pEdgeuseData->uiNextRadialEdgeuseIndex = uiNextRadialEdgeuseIndex;
}

BrepStatus NaturalBoundaryBuilder::ExtractNaturalBoundaryDataForCurrentFace(SourceNaturalBoundaryData& rBoundaryData)
{
    StagedSurfaceData* pCurrentStagedFaceSurface = GetCurrentStagedFaceSurface();
    if (!pCurrentStagedFaceSurface)
    {
        return BrepStatusError;
    }

    // Preserve existing behavior: this populates staged natural-boundary positions for later classification.
    BrepStatus sStatus = extractNaturalBoundaryVertexPositions(*pCurrentStagedFaceSurface);
    if (sStatus != BrepStatusSuccess)
    {
        return sStatus;
    }

    rBoundaryData.sSurfaceData = *pCurrentStagedFaceSurface;
    rBoundaryData.sDomain = GetCurrentStagedFaceDomain();
    rBoundaryData.vBoundaryVertexPositions = m_rState.GetNaturalBoundaryVertexPositions();
    return BrepStatusSuccess;
}

void NaturalBoundaryBuilder::ApplyNaturalBoundaryData(const SourceNaturalBoundaryData& rBoundaryData)
{
    m_rState.ClearNaturalBoundaryVertexPositions();
    for (const pxr::GfVec3d& rPosition : rBoundaryData.vBoundaryVertexPositions)
    {
        m_rState.AppendNaturalBoundaryVertexPosition(rPosition);
    }
}

BrepStatus NaturalBoundaryBuilder::extractNaturalBoundaryVertexPositions(StagedSurfaceData& rSurfaceData)
{
    m_rState.ClearNaturalBoundaryVertexPositions();

    if (StagedSurfaceData* pPlane = (rSurfaceData.eSurfaceType == StagedSurfaceType::Plane) ? &rSurfaceData : nullptr)
    {
        const pxr::GfRange2d sFaceDomain = GetCurrentStagedFaceDomain();
        const double dMinU = sFaceDomain.GetMin()[0];
        const double dMinV = sFaceDomain.GetMin()[1];
        const double dMaxU = sFaceDomain.GetMax()[0];
        const double dMaxV = sFaceDomain.GetMax()[1];

        pxr::GfVec3d sUDir = pPlane->sRefDirection;
        pxr::GfVec3d sVDir = pxr::GfCross(pPlane->sAxis, pPlane->sRefDirection);
        sUDir.Normalize();
        sVDir.Normalize();

        m_rState.AppendNaturalBoundaryVertexPosition(pPlane->sOrigin + dMinU * sUDir + dMinV * sVDir);
        m_rState.AppendNaturalBoundaryVertexPosition(pPlane->sOrigin + dMaxU * sUDir + dMinV * sVDir);
        m_rState.AppendNaturalBoundaryVertexPosition(pPlane->sOrigin + dMaxU * sUDir + dMaxV * sVDir);
        m_rState.AppendNaturalBoundaryVertexPosition(pPlane->sOrigin + dMinU * sUDir + dMaxV * sVDir);
        return BrepStatusSuccess;
    }

    if (StagedSurfaceData* pCylinder = (rSurfaceData.eSurfaceType == StagedSurfaceType::Cylinder) ? &rSurfaceData : nullptr)
    {
        const pxr::GfRange2d sFaceDomain = GetCurrentStagedFaceDomain();
        const double dMinV = sFaceDomain.GetMin()[1];
        const double dMaxV = sFaceDomain.GetMax()[1];
        const double dRadius = pCylinder->dRadius;

        pxr::GfVec3d sBottomPoint = pCylinder->sOrigin + dMinV * pCylinder->sAxis + dRadius * pCylinder->sRefDirection;
        pxr::GfVec3d sTopPoint = pCylinder->sOrigin + dMaxV * pCylinder->sAxis + dRadius * pCylinder->sRefDirection;

        m_rState.AppendNaturalBoundaryVertexPosition(sBottomPoint);
        m_rState.AppendNaturalBoundaryVertexPosition(sTopPoint);
        return BrepStatusSuccess;
    }

    if (StagedSurfaceData* pCone = (rSurfaceData.eSurfaceType == StagedSurfaceType::Cone) ? &rSurfaceData : nullptr)
    {
        const pxr::GfRange2d sFaceDomain = GetCurrentStagedFaceDomain();
        const double dMinV = sFaceDomain.GetMin()[1];
        const double dMaxV = sFaceDomain.GetMax()[1];
        const double dBaseRadius = pCone->dRadius;
        const double dSemiAngle = pCone->dSemiAngle;
        const double dRadiusMin = dBaseRadius + dMinV * std::tan(dSemiAngle);
        const double dRadiusMax = dBaseRadius + dMaxV * std::tan(dSemiAngle);
        const double dTol = 1.0e-10;

        pxr::GfVec3d sCenterMin = pCone->sOrigin + dMinV * pCone->sAxis;
        pxr::GfVec3d sCenterMax = pCone->sOrigin + dMaxV * pCone->sAxis;

        if (std::abs(dRadiusMin) <= dTol)
        {
            m_rState.AppendNaturalBoundaryVertexPosition(sCenterMin);
        }
        else
        {
            m_rState.AppendNaturalBoundaryVertexPosition(sCenterMin + std::abs(dRadiusMin) * pCone->sRefDirection);
        }

        if (std::abs(dRadiusMax) <= dTol)
        {
            m_rState.AppendNaturalBoundaryVertexPosition(sCenterMax);
        }
        else
        {
            m_rState.AppendNaturalBoundaryVertexPosition(sCenterMax + std::abs(dRadiusMax) * pCone->sRefDirection);
        }
        return BrepStatusSuccess;
    }

    if (StagedSurfaceData* pSphere = (rSurfaceData.eSurfaceType == StagedSurfaceType::Sphere) ? &rSurfaceData : nullptr)
    {
        m_rState.AppendNaturalBoundaryVertexPosition(pSphere->sCenter - pSphere->sAxis * pSphere->dRadius);
        m_rState.AppendNaturalBoundaryVertexPosition(pSphere->sCenter + pSphere->sAxis * pSphere->dRadius);
        return BrepStatusSuccess;
    }

    if (StagedSurfaceData* pTorus = (rSurfaceData.eSurfaceType == StagedSurfaceType::Torus) ? &rSurfaceData : nullptr)
    {
        pxr::GfVec3d sRefDir = pTorus->sRefDirection;
        sRefDir.Normalize();
        pxr::GfVec3d sCenter = pTorus->sOrigin + sRefDir * pTorus->dMajorRadius;
        pxr::GfVec3d sP0 = sCenter + sRefDir * pTorus->dMinorRadius;
        m_rState.AppendNaturalBoundaryVertexPosition(sP0);
        return BrepStatusSuccess;
    }

    if (StagedSurfaceData* pSurf = (rSurfaceData.eSurfaceType == StagedSurfaceType::BSplineSurface) ? &rSurfaceData : nullptr)
    {
        const auto& ctrlPts = pSurf->vNurbControlVertices;
        uint32_t uCount = pSurf->uiNurbUVertexCount;
        uint32_t vCount = pSurf->uiNurbVVertexCount;

        if (ctrlPts.empty() || uCount == 0 || vCount == 0)
        {
            return BrepStatusSuccess;
        }

        auto ptEq = [](const pxr::GfVec3d& a, const pxr::GfVec3d& b) -> bool
        {
            double s = std::max({ a.GetLength(), b.GetLength(), 1.0 });
            return (a - b).GetLength() <= 1.0e-6 * s;
        };

        bool bClosedInU = true;
        for (uint32_t v = 0; v < vCount && bClosedInU; ++v)
        {
            if (!ptEq(ctrlPts[v], ctrlPts[static_cast<size_t>(uCount - 1) * vCount + v]))
            {
                bClosedInU = false;
            }
        }

        bool bClosedInV = true;
        for (uint32_t u = 0; u < uCount && bClosedInV; ++u)
        {
            if (!ptEq(ctrlPts[static_cast<size_t>(u) * vCount], ctrlPts[static_cast<size_t>(u) * vCount + vCount - 1]))
            {
                bClosedInV = false;
            }
        }

        if (bClosedInU && bClosedInV)
        {
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[0]);
        }
        else if (bClosedInU)
        {
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[0]);
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[vCount - 1]);
        }
        else if (bClosedInV)
        {
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[0]);
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[static_cast<size_t>(uCount - 1) * vCount]);
        }
        else
        {
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[0]);
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[static_cast<size_t>(uCount - 1) * vCount]);
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[static_cast<size_t>(uCount - 1) * vCount + (vCount - 1)]);
            m_rState.AppendNaturalBoundaryVertexPosition(ctrlPts[vCount - 1]);
        }
        return BrepStatusSuccess;
    }

    return BrepStatusSuccess;

} // end NaturalBoundaryBuilder::extractNaturalBoundaryVertexPositions

/****************************************************************************************************************************************************
 * PURPOSE: Create edges, vertices, curves, and staged StagedEdgeuseData objects for the natural boundary.
 *          Populates staged implied-boundary segments ordered CCW
 *          starting from the segment departing uiContactVertexIndex.
 *          When uiContactVertexIndex == UINT32_MAX, uses an arbitrary CCW start (for standalone loop case).
 *          The staged objects are consumed by addImpliedNaturalBoundaryAsLoop() or AddImpliedNaturalBoundaryToLoop().
 *
 * NOTES:   For a seam edge there are 2 StagedEdgeuseData entries (one per side of the seam).
 *          For a lamina boundary edge there is 1 StagedEdgeuseData entry.
 *          uiNextRadialEdgeuseIndex is set to 0 as a placeholder — the consuming function sets it.
 ***************************************************************************************************************************************************/
BrepStatus NaturalBoundaryBuilder::ExtractImpliedBoundaryEdgesForCurrentFace(
    const SourceNaturalBoundaryVertexResolver& rFindExistingBoundaryVertex,
    uint32_t uiContactVertexIndex
)
{
    StagedSurfaceData* pCurrentStagedFaceSurface = GetCurrentStagedFaceSurface();
    if (!pCurrentStagedFaceSurface)
    {
        return BrepStatusError;
    }

    return extractImpliedBoundaryEdges(rFindExistingBoundaryVertex, *pCurrentStagedFaceSurface, uiContactVertexIndex);
}

BrepStatus NaturalBoundaryBuilder::extractImpliedBoundaryEdges(
    const SourceNaturalBoundaryVertexResolver& rFindExistingBoundaryVertex,
    StagedSurfaceData& rSurfaceData,
    uint32_t uiContactVertexIndex
)
{
    m_rState.ClearImpliedBoundarySegments();

    // lambda find-or-add vertex: reuses existing source vertices when a loop vertex lies on the implied boundary.
    auto findOrAddVertex = [this, &rFindExistingBoundaryVertex](const pxr::GfVec3d& sTargetPoint) -> uint32_t
    {
        uint32_t uiIndex = rFindExistingBoundaryVertex ? rFindExistingBoundaryVertex(sTargetPoint) : kStagedNoObjectIndex;
        if (uiIndex != kStagedNoObjectIndex)
        {
            return uiIndex;
        }

        return m_sEdgeVertexBuilder.AddStagedVertexPoint(sTargetPoint);
    };

    // sphere boundary: 1 seam edge, 2 pole vertices, 2 segments (one per seam side)
    if (StagedSurfaceData* pSphere = (rSurfaceData.eSurfaceType == StagedSurfaceType::Sphere) ? &rSurfaceData : nullptr)
    {
        const pxr::GfVec3d& sCenter = pSphere->sCenter;
        const pxr::GfVec3d& sAxis = pSphere->sAxis;
        const pxr::GfVec3d& sRefDirection = pSphere->sRefDirection;
        double sRadius = pSphere->dRadius;
        pxr::GfVec3d sNorthPole = sCenter + sAxis * sRadius;
        pxr::GfVec3d sSouthPole = sCenter - sAxis * sRadius;

        uint32_t uiSouthPoleIndex = findOrAddVertex(sSouthPole);
        uint32_t uiNorthPoleIndex = findOrAddVertex(sNorthPole);

        pxr::GfRange1d sSeamInterval(0.0, M_PI);
        uint32_t uiSeamEdgeIndex = m_sEdgeVertexBuilder
                                       .AddStagedEdgeForCurrentTraversal(StagedCurveType::Circle, sSeamInterval, uiSouthPoleIndex, uiNorthPoleIndex);

        pxr::GfVec3d sCircleAxis = pxr::GfCross(sRefDirection, sAxis);
        if (sCircleAxis.Normalize() == 0.0)
        {
            return BrepStatusError;
        }

        m_sEdgeVertexBuilder.AddStagedCircleCurve(sCenter, sCircleAxis, -sAxis, sRadius);

        // order segments starting from the one departing the contact vertex
        // euUp traverses south→north (Same, departs south pole), euDown traverses north→south (Opposite, departs north pole)
        if (uiContactVertexIndex == uiNorthPoleIndex)
        {
            m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdgeIndex, StagedOrientType::Opposite, StagedRadialEntryType::TopSideEntry);
            m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdgeIndex, StagedOrientType::Same, StagedRadialEntryType::BottomSideEntry);
        }
        else if (uiContactVertexIndex == uiSouthPoleIndex)
        {
            m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdgeIndex, StagedOrientType::Same, StagedRadialEntryType::BottomSideEntry);
            m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdgeIndex, StagedOrientType::Opposite, StagedRadialEntryType::TopSideEntry);
        }
        else
        {
            m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdgeIndex, StagedOrientType::Same, StagedRadialEntryType::BottomSideEntry);
            m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdgeIndex, StagedOrientType::Opposite, StagedRadialEntryType::TopSideEntry);
        }

        return BrepStatusSuccess;
    } // end sphere

    // helper lambda: create a line edge+curve, return the edge index
    auto addLineEdgeAndCurve = [this](uint32_t uiStartVtx, uint32_t uiEndVtx, const pxr::GfVec3d& sStart, const pxr::GfVec3d& sEnd) -> uint32_t
    {
        pxr::GfVec3d sDir = sEnd - sStart;
        double dLen = sDir.Normalize();

        uint32_t idx = m_sEdgeVertexBuilder.AddStagedEdgeForCurrentTraversal(StagedCurveType::Line, pxr::GfRange1d(0.0, dLen), uiStartVtx, uiEndVtx);

        m_sEdgeVertexBuilder.AddStagedLineCurve(sStart, sDir);

        return idx;
    };

    // helper lambda: create a closed circle edge+curve (start==end vertex), return edge index
    auto addClosedCircleEdgeAndCurve =
        [this](uint32_t uiVertexIdx, const pxr::GfVec3d& sCenter, const pxr::GfVec3d& sAxis, const pxr::GfVec3d& sRefDir, double dRadius) -> uint32_t
    {
        uint32_t idx = m_sEdgeVertexBuilder
                           .AddStagedEdgeForCurrentTraversal(StagedCurveType::Circle, pxr::GfRange1d(0.0, 2.0 * M_PI), uiVertexIdx, uiVertexIdx);

        m_sEdgeVertexBuilder.AddStagedCircleCurve(sCenter, sAxis, sRefDir, dRadius);

        return idx;
    };

    // helper lambda: stage one implied-boundary edgeuse
    auto stageEU = [this](uint32_t edgeIdx, StagedOrientType orient, StagedRadialEntryType entry)
    {
        m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(edgeIdx, orient, entry);
    };

    // plane boundary: 4 lamina line edges, 4 corner vertices
    if (StagedSurfaceData* pPlane = (rSurfaceData.eSurfaceType == StagedSurfaceType::Plane) ? &rSurfaceData : nullptr)
    {
        const pxr::GfRange2d sFaceDomain = GetCurrentStagedFaceDomain();
        pxr::GfVec3d sUDir = pPlane->sRefDirection;
        pxr::GfVec3d sVDir = pxr::GfCross(pPlane->sAxis, pPlane->sRefDirection);
        if (sUDir.Normalize() == 0.0 || sVDir.Normalize() == 0.0)
        {
            return BrepStatusError;
        }

        double dMinU = sFaceDomain.GetMin()[0], dMinV = sFaceDomain.GetMin()[1];
        double dMaxU = sFaceDomain.GetMax()[0], dMaxV = sFaceDomain.GetMax()[1];
        pxr::GfVec3d sP00 = pPlane->sOrigin + dMinU * sUDir + dMinV * sVDir;
        pxr::GfVec3d sP10 = pPlane->sOrigin + dMaxU * sUDir + dMinV * sVDir;
        pxr::GfVec3d sP11 = pPlane->sOrigin + dMaxU * sUDir + dMaxV * sVDir;
        pxr::GfVec3d sP01 = pPlane->sOrigin + dMinU * sUDir + dMaxV * sVDir;

        uint32_t uiV00 = findOrAddVertex(sP00);
        uint32_t uiV10 = findOrAddVertex(sP10);
        uint32_t uiV11 = findOrAddVertex(sP11);
        uint32_t uiV01 = findOrAddVertex(sP01);

        uint32_t e0 = addLineEdgeAndCurve(uiV00, uiV10, sP00, sP10);
        uint32_t e1 = addLineEdgeAndCurve(uiV10, uiV11, sP10, sP11);
        uint32_t e2 = addLineEdgeAndCurve(uiV11, uiV01, sP11, sP01);
        uint32_t e3 = addLineEdgeAndCurve(uiV01, uiV00, sP01, sP00);

        // CCW order: e0, e1, e2, e3 — all Same/TopEntry (lamina edges)
        uint32_t edges[4] = { e0, e1, e2, e3 };
        uint32_t verts[4] = { uiV00, uiV10, uiV11, uiV01 };

        // find rotation start based on contact vertex
        uint32_t rotStart = 0;
        if (uiContactVertexIndex != UINT32_MAX)
        {
            for (uint32_t ii = 0; ii < 4; ++ii)
            {
                if (verts[ii] == uiContactVertexIndex)
                {
                    rotStart = ii;
                    break;
                }
            }
        }
        for (uint32_t ii = 0; ii < 4; ++ii)
        {
            uint32_t idx = (rotStart + ii) % 4;
            stageEU(edges[idx], StagedOrientType::Same, StagedRadialEntryType::TopSideEntry);
        }
        return BrepStatusSuccess;
    } // end plane

    // cylinder boundary: 1 seam line + 2 closed circles, 4 edgeuses
    if (StagedSurfaceData* pCylinder = (rSurfaceData.eSurfaceType == StagedSurfaceType::Cylinder) ? &rSurfaceData : nullptr)
    {
        const pxr::GfRange2d sFaceDomain = GetCurrentStagedFaceDomain();
        double dMinV = sFaceDomain.GetMin()[1], dMaxV = sFaceDomain.GetMax()[1];
        const pxr::GfVec3d& sAxis = pCylinder->sAxis;
        const pxr::GfVec3d& sRefDir = pCylinder->sRefDirection;
        double dRadius = pCylinder->dRadius;

        pxr::GfVec3d sBotCenter = pCylinder->sOrigin + dMinV * sAxis;
        pxr::GfVec3d sTopCenter = pCylinder->sOrigin + dMaxV * sAxis;
        pxr::GfVec3d sBotPt = sBotCenter + dRadius * sRefDir;
        pxr::GfVec3d sTopPt = sTopCenter + dRadius * sRefDir;

        uint32_t uiBotVtx = findOrAddVertex(sBotPt);
        uint32_t uiTopVtx = findOrAddVertex(sTopPt);

        pxr::GfVec3d sSeamDir = sTopPt - sBotPt;
        if (sSeamDir.Normalize() == 0.0)
        {
            return BrepStatusError;
        }

        uint32_t uiSeamEdge = addLineEdgeAndCurve(uiBotVtx, uiTopVtx, sBotPt, sTopPt);
        uint32_t uiTopEdge = addClosedCircleEdgeAndCurve(uiTopVtx, sTopCenter, sAxis, sRefDir, dRadius);
        uint32_t uiBotEdge = addClosedCircleEdgeAndCurve(uiBotVtx, sBotCenter, sAxis, sRefDir, dRadius);

        // CCW order: seam Same/Top, topCircle Same/Top, seam Opposite/Bottom, botCircle Opposite/Bottom
        // rotate so contact vertex segment comes first
        using OT = StagedOrientType;
        using RT = StagedRadialEntryType;
        struct S
        {
            uint32_t e;
            OT o;
            RT r;
            uint32_t startVtx;
        };
        S segs[4] = { { uiSeamEdge, OT::Same, RT::TopSideEntry, uiBotVtx },
                      { uiTopEdge, OT::Same, RT::TopSideEntry, uiTopVtx },
                      { uiSeamEdge, OT::Opposite, RT::BottomSideEntry, uiTopVtx },
                      { uiBotEdge, OT::Opposite, RT::BottomSideEntry, uiBotVtx } };
        uint32_t rotStart = 0;
        if (uiContactVertexIndex != UINT32_MAX)
        {
            for (uint32_t ii = 0; ii < 4; ++ii)
            {
                if (segs[ii].startVtx == uiContactVertexIndex)
                {
                    rotStart = ii;
                    break;
                }
            }
        }
        for (uint32_t ii = 0; ii < 4; ++ii)
        {
            uint32_t idx = (rotStart + ii) % 4;
            stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
        }
        return BrepStatusSuccess;
    } // end cylinder

    // cone boundary
    if (StagedSurfaceData* pCone = (rSurfaceData.eSurfaceType == StagedSurfaceType::Cone) ? &rSurfaceData : nullptr)
    {
        const pxr::GfRange2d sFaceDomain = GetCurrentStagedFaceDomain();
        double dMinV = sFaceDomain.GetMin()[1], dMaxV = sFaceDomain.GetMax()[1];
        const pxr::GfVec3d& sAxis = pCone->sAxis;
        const pxr::GfVec3d& sRefDir = pCone->sRefDirection;
        double dBaseRadius = pCone->dRadius;
        double dSemiAngle = pCone->dSemiAngle;
        double dRadiusMin = dBaseRadius + dMinV * std::tan(dSemiAngle);
        double dRadiusMax = dBaseRadius + dMaxV * std::tan(dSemiAngle);
        double dTol = 1.0e-10;
        bool bMinIsPole = std::abs(dRadiusMin) <= dTol;
        bool bMaxIsPole = std::abs(dRadiusMax) <= dTol;

        if (bMinIsPole && bMaxIsPole)
        {
            return BrepStatusError;
        }

        pxr::GfVec3d sCenterMin = pCone->sOrigin + dMinV * sAxis;
        pxr::GfVec3d sCenterMax = pCone->sOrigin + dMaxV * sAxis;

        using OT = StagedOrientType;
        using RT = StagedRadialEntryType;

        if (bMinIsPole || bMaxIsPole)
        {
            // one-pole sub-case: 1 seam + 1 circle, 3 edgeuses
            pxr::GfVec3d sPole = bMinIsPole ? sCenterMin : sCenterMax;
            pxr::GfVec3d sCircCenter = bMinIsPole ? sCenterMax : sCenterMin;
            double dCircR = bMinIsPole ? std::abs(dRadiusMax) : std::abs(dRadiusMin);
            pxr::GfVec3d sCircPt = sCircCenter + dCircR * sRefDir;

            uint32_t uiPoleVtx = findOrAddVertex(sPole);
            uint32_t uiCircVtx = findOrAddVertex(sCircPt);

            pxr::GfVec3d sSeamDir = sCircPt - sPole;
            if (sSeamDir.Normalize() == 0.0)
            {
                return BrepStatusError;
            }

            uint32_t uiSeamEdge = addLineEdgeAndCurve(uiPoleVtx, uiCircVtx, sPole, sCircPt);
            uint32_t uiCircEdge = addClosedCircleEdgeAndCurve(uiCircVtx, sCircCenter, sAxis, sRefDir, dCircR);

            struct S
            {
                uint32_t e;
                OT o;
                RT r;
                uint32_t startVtx;
            };
            S segs[3] = { { uiSeamEdge, OT::Same, RT::TopSideEntry, uiPoleVtx },
                          { uiCircEdge, OT::Same, RT::TopSideEntry, uiCircVtx },
                          { uiSeamEdge, OT::Opposite, RT::BottomSideEntry, uiCircVtx } };
            uint32_t rotStart = 0;
            if (uiContactVertexIndex != UINT32_MAX)
            {
                for (uint32_t ii = 0; ii < 3; ++ii)
                {
                    if (segs[ii].startVtx == uiContactVertexIndex)
                    {
                        rotStart = ii;
                        break;
                    }
                }
            }
            for (uint32_t ii = 0; ii < 3; ++ii)
            {
                uint32_t idx = (rotStart + ii) % 3;
                stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
            }
            return BrepStatusSuccess;
        }

        // frustum sub-case: 1 seam + 2 circles, 4 edgeuses (like cylinder)
        double dRMinAbs = std::abs(dRadiusMin), dRMaxAbs = std::abs(dRadiusMax);
        pxr::GfVec3d sBotPt = sCenterMin + dRMinAbs * sRefDir;
        pxr::GfVec3d sTopPt = sCenterMax + dRMaxAbs * sRefDir;

        uint32_t uiBotVtx = findOrAddVertex(sBotPt);
        uint32_t uiTopVtx = findOrAddVertex(sTopPt);

        pxr::GfVec3d sSeamDir = sTopPt - sBotPt;
        if (sSeamDir.Normalize() == 0.0)
        {
            return BrepStatusError;
        }

        uint32_t uiSeamEdge = addLineEdgeAndCurve(uiBotVtx, uiTopVtx, sBotPt, sTopPt);
        uint32_t uiTopEdge = addClosedCircleEdgeAndCurve(uiTopVtx, sCenterMax, sAxis, sRefDir, dRMaxAbs);
        uint32_t uiBotEdge = addClosedCircleEdgeAndCurve(uiBotVtx, sCenterMin, sAxis, sRefDir, dRMinAbs);

        struct S
        {
            uint32_t e;
            OT o;
            RT r;
            uint32_t startVtx;
        };
        S segs[4] = { { uiSeamEdge, OT::Same, RT::TopSideEntry, uiBotVtx },
                      { uiTopEdge, OT::Same, RT::TopSideEntry, uiTopVtx },
                      { uiSeamEdge, OT::Opposite, RT::BottomSideEntry, uiTopVtx },
                      { uiBotEdge, OT::Opposite, RT::BottomSideEntry, uiBotVtx } };
        uint32_t rotStart = 0;
        if (uiContactVertexIndex != UINT32_MAX)
        {
            for (uint32_t ii = 0; ii < 4; ++ii)
            {
                if (segs[ii].startVtx == uiContactVertexIndex)
                {
                    rotStart = ii;
                    break;
                }
            }
        }
        for (uint32_t ii = 0; ii < 4; ++ii)
        {
            uint32_t idx = (rotStart + ii) % 4;
            stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
        }
        return BrepStatusSuccess;
    } // end cone

    // torus boundary: 2 closed seam edges (minor + major circle), 1 vertex, 4 edgeuses
    if (StagedSurfaceData* pTorus = (rSurfaceData.eSurfaceType == StagedSurfaceType::Torus) ? &rSurfaceData : nullptr)
    {
        pxr::GfVec3d sRefDir = pTorus->sRefDirection;
        if (sRefDir.Normalize() == 0.0)
        {
            return BrepStatusError;
        }
        const pxr::GfVec3d& sOrigin = pTorus->sOrigin;
        const pxr::GfVec3d& sAxis = pTorus->sAxis;
        double dMajR = pTorus->dMajorRadius;
        double dMinR = pTorus->dMinorRadius;

        pxr::GfVec3d sCenter = sOrigin + sRefDir * dMajR;
        pxr::GfVec3d sP0 = sCenter + sRefDir * dMinR;
        uint32_t uiVtx = findOrAddVertex(sP0);

        pxr::GfVec3d sMinorAxis = pxr::GfCross(sRefDir, sAxis);
        if (sMinorAxis.Normalize() == 0.0)
        {
            return BrepStatusError;
        }

        uint32_t uiMinorEdge = addClosedCircleEdgeAndCurve(uiVtx, sCenter, sMinorAxis, sRefDir, dMinR);
        uint32_t uiMajorEdge = addClosedCircleEdgeAndCurve(uiVtx, sOrigin, sAxis, sRefDir, dMajR + dMinR);

        using OT = StagedOrientType;
        using RT = StagedRadialEntryType;

        // CCW order: U1(minor Opp/Top), V1(major Same/Bot), U2(minor Same/Bot), V2(major Opp/Top)
        stageEU(uiMinorEdge, OT::Opposite, RT::TopSideEntry);
        stageEU(uiMajorEdge, OT::Same, RT::BottomSideEntry);
        stageEU(uiMinorEdge, OT::Same, RT::BottomSideEntry);
        stageEU(uiMajorEdge, OT::Opposite, RT::TopSideEntry);

        return BrepStatusSuccess;
    } // end torus

    // BSpline surface boundary edges
    if (StagedSurfaceData* pSurf = (rSurfaceData.eSurfaceType == StagedSurfaceType::BSplineSurface) ? &rSurfaceData : nullptr)
    {
        uint32_t uCount = pSurf->uiNurbUVertexCount;
        uint32_t vCount = pSurf->uiNurbVVertexCount;
        const auto& ctrlPts = pSurf->vNurbControlVertices;
        const auto& weights = pSurf->vNurbWeights;

        if (ctrlPts.empty() || uCount == 0 || vCount == 0)
        {
            return BrepStatusError;
        }

        auto ptEq = [](const pxr::GfVec3d& a, const pxr::GfVec3d& b) -> bool
        {
            double s = std::max({ a.GetLength(), b.GetLength(), 1.0 });
            return (a - b).GetLength() <= 1.0e-6 * s;
        };

        bool bClosedInU = true;
        for (uint32_t v = 0; v < vCount && bClosedInU; ++v)
        {
            if (!ptEq(ctrlPts[v], ctrlPts[static_cast<size_t>(uCount - 1) * vCount + v]))
            {
                bClosedInU = false;
            }
        }

        bool bClosedInV = true;
        for (uint32_t u = 0; u < uCount && bClosedInV; ++u)
        {
            if (!ptEq(ctrlPts[static_cast<size_t>(u) * vCount], ctrlPts[static_cast<size_t>(u) * vCount + vCount - 1]))
            {
                bClosedInV = false;
            }
        }

        using OT = StagedOrientType;
        using RT = StagedRadialEntryType;

        auto addBSplineRowCurve = [&](uint32_t vIdx) -> uint32_t
        {
            pxr::VtArray<pxr::GfVec3d> sControlVertices;
            pxr::VtArray<double> sWeights;
            sControlVertices.reserve(uCount);
            sWeights.reserve(weights.empty() ? 0 : uCount);
            for (uint32_t u = 0; u < uCount; ++u)
            {
                sControlVertices.push_back(ctrlPts[static_cast<size_t>(u) * vCount + vIdx]);
                if (!weights.empty())
                {
                    sWeights.push_back(weights[static_cast<size_t>(u) * vCount + vIdx]);
                }
            }

            return m_sEdgeVertexBuilder.AddStagedBSplineCurve(uCount, pSurf->uiNurbUOrder, pSurf->vNurbUKnots, sControlVertices, sWeights);
        };

        auto addBSplineColumnCurve = [&](uint32_t uIdx) -> uint32_t
        {
            pxr::VtArray<pxr::GfVec3d> sControlVertices;
            pxr::VtArray<double> sWeights;
            sControlVertices.reserve(vCount);
            sWeights.reserve(weights.empty() ? 0 : vCount);
            for (uint32_t v = 0; v < vCount; ++v)
            {
                sControlVertices.push_back(ctrlPts[static_cast<size_t>(uIdx) * vCount + v]);
                if (!weights.empty())
                {
                    sWeights.push_back(weights[static_cast<size_t>(uIdx) * vCount + v]);
                }
            }

            return m_sEdgeVertexBuilder.AddStagedBSplineCurve(vCount, pSurf->uiNurbVOrder, pSurf->vNurbVKnots, sControlVertices, sWeights);
        };

        // helper: create an open BSpline seam edge from a U-row of the surface (seam along U)
        auto addBSplineRowSeamEdge = [&](uint32_t uiSVtx, uint32_t uiEVtx, uint32_t vIdx) -> uint32_t
        {
            uint32_t idx = m_sEdgeVertexBuilder.AddStagedEdgeForCurrentTraversal(
                StagedCurveType::BSplineCurve,
                pxr::GfRange1d(pSurf->vNurbUKnots.front(), pSurf->vNurbUKnots.back()),
                uiSVtx,
                uiEVtx
            );
            addBSplineRowCurve(vIdx);
            return idx;
        };

        // helper: create a closed BSpline boundary curve from a U-row of the surface
        auto addClosedBSplineRowEdgeAndCurve = [&](uint32_t uiVertexIdx, uint32_t vIdx) -> uint32_t
        {
            uint32_t idx = m_sEdgeVertexBuilder.AddStagedEdgeForCurrentTraversal(
                StagedCurveType::BSplineCurve,
                pxr::GfRange1d(pSurf->vNurbUKnots.front(), pSurf->vNurbUKnots.back()),
                uiVertexIdx,
                uiVertexIdx
            );
            addBSplineRowCurve(vIdx);
            return idx;
        };

        // helper: create a closed BSpline boundary curve from a V-column of the surface
        auto addClosedBSplineColumnEdgeAndCurve = [&](uint32_t uiVertexIdx, uint32_t uIdx) -> uint32_t
        {
            uint32_t idx = m_sEdgeVertexBuilder.AddStagedEdgeForCurrentTraversal(
                StagedCurveType::BSplineCurve,
                pxr::GfRange1d(pSurf->vNurbVKnots.front(), pSurf->vNurbVKnots.back()),
                uiVertexIdx,
                uiVertexIdx
            );
            addBSplineColumnCurve(uIdx);
            return idx;
        };

        if (bClosedInU && bClosedInV)
        {
            // torus-like: 2 closed seam edges (V-seam from column 0, U-seam from row 0), 1 shared vertex, 4 EUs
            uint32_t uiVtx = findOrAddVertex(ctrlPts[0]);

            uint32_t uiVSeamEdge = addClosedBSplineColumnEdgeAndCurve(uiVtx, 0);
            uint32_t uiUSeamEdge = addClosedBSplineRowEdgeAndCurve(uiVtx, 0);

            stageEU(uiVSeamEdge, OT::Opposite, RT::TopSideEntry);
            stageEU(uiUSeamEdge, OT::Same, RT::BottomSideEntry);
            stageEU(uiVSeamEdge, OT::Same, RT::BottomSideEntry);
            stageEU(uiUSeamEdge, OT::Opposite, RT::TopSideEntry);

            return BrepStatusSuccess;
        } // end closed in both U and V

        if (bClosedInU)
        {
            pxr::GfVec3d vStart = ctrlPts[0];
            pxr::GfVec3d vEnd = ctrlPts[vCount - 1];

            bool bVMinPole = true;
            for (uint32_t u = 1; u < uCount && bVMinPole; ++u)
            {
                if (!ptEq(ctrlPts[static_cast<size_t>(u) * vCount], ctrlPts[0]))
                {
                    bVMinPole = false;
                }
            }

            bool bVMaxPole = true;
            for (uint32_t u = 1; u < uCount && bVMaxPole; ++u)
            {
                if (!ptEq(ctrlPts[static_cast<size_t>(u) * vCount + vCount - 1], ctrlPts[vCount - 1]))
                {
                    bVMaxPole = false;
                }
            }

            uint32_t uiStartVtx = findOrAddVertex(vStart);
            uint32_t uiEndVtx = findOrAddVertex(vEnd);

            double tMin = pSurf->vNurbVKnots.front();
            double tMax = pSurf->vNurbVKnots.back();

            uint32_t uiSeamEdge = m_sEdgeVertexBuilder.AddStagedEdgeForCurrentTraversal(
                StagedCurveType::BSplineCurve,
                pxr::GfRange1d(tMin, tMax),
                uiStartVtx,
                uiEndVtx
            );
            addBSplineColumnCurve(0);

            if (bVMinPole && bVMaxPole)
            {
                if (uiContactVertexIndex == uiEndVtx)
                {
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Opposite, RT::TopSideEntry);
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Same, RT::BottomSideEntry);
                }
                else
                {
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Same, RT::BottomSideEntry);
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Opposite, RT::TopSideEntry);
                }
                return BrepStatusSuccess;
            } // end two-pole

            if (!bVMinPole && !bVMaxPole)
            {
                // no-pole case (cylinder-like): seam + 2 closed boundary curves, 4 edgeuses
                uint32_t uiTopEdge = addClosedBSplineRowEdgeAndCurve(uiEndVtx, vCount - 1);
                uint32_t uiBotEdge = addClosedBSplineRowEdgeAndCurve(uiStartVtx, 0);

                struct S
                {
                    uint32_t e;
                    OT o;
                    RT r;
                    uint32_t startVtx;
                };
                S segs[4] = { { uiSeamEdge, OT::Same, RT::TopSideEntry, uiStartVtx },
                              { uiTopEdge, OT::Same, RT::TopSideEntry, uiEndVtx },
                              { uiSeamEdge, OT::Opposite, RT::BottomSideEntry, uiEndVtx },
                              { uiBotEdge, OT::Opposite, RT::BottomSideEntry, uiStartVtx } };
                uint32_t rotStart = 0;
                if (uiContactVertexIndex != UINT32_MAX)
                {
                    for (uint32_t ii = 0; ii < 4; ++ii)
                    {
                        if (segs[ii].startVtx == uiContactVertexIndex)
                        {
                            rotStart = ii;
                            break;
                        }
                    }
                }
                for (uint32_t ii = 0; ii < 4; ++ii)
                {
                    uint32_t idx = (rotStart + ii) % 4;
                    stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
                }
                return BrepStatusSuccess;
            } // end no-pole

            // one-pole case (cone-like): seam + 1 closed boundary curve at non-pole end, 3 edgeuses
            {
                bool bPolAtStart = bVMinPole;
                uint32_t uiPoleVtx = bPolAtStart ? uiStartVtx : uiEndVtx;
                uint32_t uiOpenVtx = bPolAtStart ? uiEndVtx : uiStartVtx;
                uint32_t uiOpenRow = bPolAtStart ? (vCount - 1) : 0;

                uint32_t uiOpenEdge = addClosedBSplineRowEdgeAndCurve(uiOpenVtx, uiOpenRow);

                struct S
                {
                    uint32_t e;
                    OT o;
                    RT r;
                    uint32_t startVtx;
                };
                S segs[3] = { { uiSeamEdge, bPolAtStart ? OT::Same : OT::Opposite, bPolAtStart ? RT::TopSideEntry : RT::BottomSideEntry, uiPoleVtx },
                              { uiOpenEdge, OT::Same, RT::TopSideEntry, uiOpenVtx },
                              { uiSeamEdge, bPolAtStart ? OT::Opposite : OT::Same, bPolAtStart ? RT::BottomSideEntry : RT::TopSideEntry, uiOpenVtx } };
                uint32_t rotStart = 0;
                if (uiContactVertexIndex != UINT32_MAX)
                {
                    for (uint32_t ii = 0; ii < 3; ++ii)
                    {
                        if (segs[ii].startVtx == uiContactVertexIndex)
                        {
                            rotStart = ii;
                            break;
                        }
                    }
                }
                for (uint32_t ii = 0; ii < 3; ++ii)
                {
                    uint32_t idx = (rotStart + ii) % 3;
                    stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
                }
                return BrepStatusSuccess;
            } // end one-pole
        } // end closedInU

        if (bClosedInV)
        {
            // seam runs along U direction -- extract row 0 as a BSpline curve
            pxr::GfVec3d uStart = ctrlPts[0];
            pxr::GfVec3d uEnd = ctrlPts[static_cast<size_t>(uCount - 1) * vCount];

            bool bUMinPole = true;
            for (uint32_t v = 1; v < vCount && bUMinPole; ++v)
            {
                if (!ptEq(ctrlPts[v], ctrlPts[0]))
                {
                    bUMinPole = false;
                }
            }

            bool bUMaxPole = true;
            for (uint32_t v = 1; v < vCount && bUMaxPole; ++v)
            {
                if (!ptEq(ctrlPts[static_cast<size_t>(uCount - 1) * vCount + v], ctrlPts[static_cast<size_t>(uCount - 1) * vCount]))
                {
                    bUMaxPole = false;
                }
            }

            uint32_t uiStartVtx = findOrAddVertex(uStart);
            uint32_t uiEndVtx = findOrAddVertex(uEnd);

            uint32_t uiSeamEdge = addBSplineRowSeamEdge(uiStartVtx, uiEndVtx, 0);

            if (bUMinPole && bUMaxPole)
            {
                if (uiContactVertexIndex == uiEndVtx)
                {
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Opposite, RT::TopSideEntry);
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Same, RT::BottomSideEntry);
                }
                else
                {
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Same, RT::BottomSideEntry);
                    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuse(uiSeamEdge, OT::Opposite, RT::TopSideEntry);
                }
                return BrepStatusSuccess;
            } // end two-pole closedInV

            if (!bUMinPole && !bUMaxPole)
            {
                // no-pole: seam + 2 closed V-column boundary curves, 4 EUs
                uint32_t uiEndColEdge = addClosedBSplineColumnEdgeAndCurve(uiEndVtx, uCount - 1);
                uint32_t uiStartColEdge = addClosedBSplineColumnEdgeAndCurve(uiStartVtx, 0);

                struct S
                {
                    uint32_t e;
                    OT o;
                    RT r;
                    uint32_t startVtx;
                };
                S segs[4] = { { uiSeamEdge, OT::Same, RT::TopSideEntry, uiStartVtx },
                              { uiEndColEdge, OT::Same, RT::TopSideEntry, uiEndVtx },
                              { uiSeamEdge, OT::Opposite, RT::BottomSideEntry, uiEndVtx },
                              { uiStartColEdge, OT::Opposite, RT::BottomSideEntry, uiStartVtx } };
                uint32_t rotStart = 0;
                if (uiContactVertexIndex != UINT32_MAX)
                {
                    for (uint32_t ii = 0; ii < 4; ++ii)
                    {
                        if (segs[ii].startVtx == uiContactVertexIndex)
                        {
                            rotStart = ii;
                            break;
                        }
                    }
                }
                for (uint32_t ii = 0; ii < 4; ++ii)
                {
                    uint32_t idx = (rotStart + ii) % 4;
                    stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
                }
                return BrepStatusSuccess;
            } // end no-pole closedInV

            // one-pole closedInV
            {
                bool bPolAtStart = bUMinPole;
                uint32_t uiPoleVtx = bPolAtStart ? uiStartVtx : uiEndVtx;
                uint32_t uiOpenVtx = bPolAtStart ? uiEndVtx : uiStartVtx;
                uint32_t uiOpenCol = bPolAtStart ? (uCount - 1) : 0;

                uint32_t uiOpenEdge = addClosedBSplineColumnEdgeAndCurve(uiOpenVtx, uiOpenCol);

                struct S
                {
                    uint32_t e;
                    OT o;
                    RT r;
                    uint32_t startVtx;
                };
                S segs[3] = { { uiSeamEdge, bPolAtStart ? OT::Same : OT::Opposite, bPolAtStart ? RT::TopSideEntry : RT::BottomSideEntry, uiPoleVtx },
                              { uiOpenEdge, OT::Same, RT::TopSideEntry, uiOpenVtx },
                              { uiSeamEdge, bPolAtStart ? OT::Opposite : OT::Same, bPolAtStart ? RT::BottomSideEntry : RT::TopSideEntry, uiOpenVtx } };
                uint32_t rotStart = 0;
                if (uiContactVertexIndex != UINT32_MAX)
                {
                    for (uint32_t ii = 0; ii < 3; ++ii)
                    {
                        if (segs[ii].startVtx == uiContactVertexIndex)
                        {
                            rotStart = ii;
                            break;
                        }
                    }
                }
                for (uint32_t ii = 0; ii < 3; ++ii)
                {
                    uint32_t idx = (rotStart + ii) % 3;
                    stageEU(segs[idx].e, segs[idx].o, segs[idx].r);
                }
                return BrepStatusSuccess;
            } // end one-pole closedInV
        } // end closedInV

        return BrepStatusError;
    } // end BSpline

    return BrepStatusError;

} // end NaturalBoundaryBuilder::extractImpliedBoundaryEdges

/****************************************************************************************************************************************************
 * PURPOSE: AddImpliedNaturalBoundaryToLoop - Splice staged boundary edgeuses into the current outer loop at the contact vertex.
 *          Called from visitLeave(Loop) when Case 2 (BoundaryCase_AugmentCWLoop) is active.
 *
 * NOTES:   The outer loop's edgeuses are already stored in m_sStagingState.Data().vEdgeuses at contiguous positions.
 *          This function:
 *            1. Finds the outer-loop edgeuse that arrives at the contact vertex (end vertex == contact)
 *            2. Rotates the outer-loop EU sequence so the EU departing from contact is first
 *            3. Writes staged boundary edgeuse data into pre-allocated slots
 *            4. Sets up radial linkage for the boundary edgeuses
 *          The result is a single CCW loop: [trimEUs departing contact...trimEUs arriving at contact, boundaryEUs]
 ***************************************************************************************************************************************************/
BrepStatus NaturalBoundaryBuilder::AddImpliedNaturalBoundaryToLoop()
{
    if (!m_rState.HasImpliedBoundarySegments())
    {
        return BrepStatusSuccess;
    }

    uint32_t uiBoundaryEUCount = m_rState.GetImpliedBoundarySegmentCount();
    uint32_t uiOuterLoopEUCount = m_uiAugmentedOuterLoopEdgeuseCount;

    if (uiOuterLoopEUCount == 0)
    {
        return BrepStatusError;
    }

    // the augmented outer loop's EUs start at the pre-allocated outer loop slot
    uint32_t uiLoopEUStart = m_uiOuterLoopEdgeuseStartIndex;

    // find the split point: the outer-loop EU whose traversal END vertex == contact vertex
    uint32_t uiContactVertex = m_rState.GetImpliedBoundaryContactVertexIndex();
    uint32_t uiSplitOffset = UINT32_MAX;

    for (uint32_t ii = 0; ii < uiOuterLoopEUCount; ++ii)
    {
        uint32_t euIdx = uiLoopEUStart + ii;
        uint32_t euEndVertex = GetStagedEdgeuseEndVertexIndex(euIdx);
        if (euEndVertex == uiContactVertex)
        {
            uiSplitOffset = ii;
            break;
        }
    }

    if (uiSplitOffset == UINT32_MAX)
    {
        return BrepStatusError;
    }

    // save copies of the original outer-loop EUs
    std::vector<StagedEdgeuseData> origEUs(uiOuterLoopEUCount);
    for (uint32_t ii = 0; ii < uiOuterLoopEUCount; ++ii)
    {
        origEUs[ii] = GetStagedEdgeuse(uiLoopEUStart + ii);
    }

    // check for coincident edges: if any source loop edge lies along a natural boundary edge
    // TODO: In the future, the coincident edge case must be handled here.
    // When a source loop edge is coincident with a natural boundary edge,
    // that boundary segment should be skipped rather than inserted,
    // and the loop rotation may need adjustment.

    // build old→new position mapping for the rotation
    // rotation: [splitOffset+1..edgeuseCount-1, 0..splitOffset] -> [0..edgeuseCount-1]
    std::vector<uint32_t> oldToNew(uiOuterLoopEUCount);
    for (uint32_t ii = 0; ii < uiOuterLoopEUCount; ++ii)
    {
        uint32_t oldOffset = (uiSplitOffset + 1 + ii) % uiOuterLoopEUCount;
        oldToNew[oldOffset] = uiLoopEUStart + ii;
    }

    // write merged sequence into the pre-allocated EU range: rotated outer-loop EUs then boundary EUs
    uint32_t uiWriteIdx = uiLoopEUStart;

    // write outer-loop EUs after the split point (departing from contact through end of original list)
    for (uint32_t ii = uiSplitOffset + 1; ii < uiOuterLoopEUCount; ++ii)
    {
        SetStagedEdgeuse(uiWriteIdx++, origEUs[ii]);
    }

    // write outer-loop EUs from start through the split point (arriving at contact)
    for (uint32_t ii = 0; ii <= uiSplitOffset; ++ii)
    {
        SetStagedEdgeuse(uiWriteIdx++, origEUs[ii]);
    }

    uiWriteIdx = m_sEdgeVertexBuilder.SetStagedImpliedBoundaryEdgeuses(uiWriteIdx);

    // fix up radial pointers broken by the rotation:
    // any EU in m_sStagingState.Data().vEdgeuses whose uiNextRadialEdgeuseIndex pointed to an old outer-loop EU position
    // must be remapped to the new position after rotation.
    // Also fix the rotated outer-loop EUs' own radial pointers (they may point to each other or to external EUs).
    uint32_t uiTotalEUs = m_rState.GetEdgeuseCount();
    for (uint32_t ii = 0; ii < uiTotalEUs; ++ii)
    {
        uint32_t radialTarget = GetStagedEdgeuseNextRadialIndex(ii);
        if (radialTarget >= uiLoopEUStart && radialTarget < uiLoopEUStart + uiOuterLoopEUCount)
        {
            uint32_t oldOffset = radialTarget - uiLoopEUStart;
            SetStagedEdgeuseNextRadialIndex(ii, oldToNew[oldOffset]);
        }
    }

    // fix up radial linkage for the boundary edgeuses (they form a radial pair on the same seam edge)
    uint32_t uiBoundaryEUStart = uiLoopEUStart + uiOuterLoopEUCount;
    if (uiBoundaryEUCount == 2)
    {
        SetStagedEdgeuseNextRadialIndex(uiBoundaryEUStart, uiBoundaryEUStart + 1);
        SetStagedEdgeuseNextRadialIndex(uiBoundaryEUStart + 1, uiBoundaryEUStart);
    }
    else
    {
        for (uint32_t ii = 0; ii < uiBoundaryEUCount; ++ii)
        {
            uint32_t euIdx = uiBoundaryEUStart + ii;
            SetStagedEdgeuseNextRadialIndex(euIdx, euIdx);
        }
    }

    // register boundary edges in radial map
    for (uint32_t ii = 0; ii < uiBoundaryEUCount; ++ii)
    {
        uint32_t euIdx = uiBoundaryEUStart + ii;
        uint32_t edgeIdx = GetStagedImpliedBoundarySegmentEdgeIndex(ii);
        m_sTopologyBuilder.EnsureStagedEdgeRadialList(edgeIdx, euIdx, euIdx);
    }

    // update radial map entries for rotated outer-loop edges
    for (uint32_t ii = 0; ii < uiOuterLoopEUCount; ++ii)
    {
        uint32_t euIdx = uiLoopEUStart + ii;
        uint32_t edgeIdx = GetStagedEdgeuseEdgeIndex(euIdx);
        m_sTopologyBuilder.RemapStagedEdgeRadialList(edgeIdx, uiLoopEUStart, uiOuterLoopEUCount, oldToNew);
    }

    m_rState.ClearImpliedBoundarySegments();

    return BrepStatusSuccess;

} // end NaturalBoundaryBuilder::AddImpliedNaturalBoundaryToLoop

/****************************************************************************************************************************************************
 * PURPOSE: Add a CCW natural surface loop to ownerFace of the given surface
 *
 * NOTES:  For all surfaces add a CCW loop running around the surface's natural boundary.
 *         Understands that new loop vertices might be coincident with existing source vertices and
 *         uses the processVertex function to make sure coincident vertices share the same staged vertex index.
 *         Assumes new implied OuterLoop edges have to be added to the model that's probably only partially true.
 *            case 1: always true: the edges have to be unique on this face - otherwise the face would already have
 *                    an explicit outerLoop.  Loops which contain parts or all of the surface's natural
 *                    boundary edges have to be contained in explicit OuterLoops.
 *            case 2: probably true: the edges have to be unique to the Brep - otherwise some other face->edge
 *                    that connects to an edge on this face would be missing that connection.  This face has
 *                    to have a loop with an edgeuse to represent that
 *                    OtherFace->loop->edgeuse->edge->edgeuse->loop->Thisface connection.
 *                    That loop has to be an explicit OuterLoop since it contains part or all of the natural boundary.
 *          - Adds pairs of lamina edges in surface open parametric directions.
 *          - Adds a single manifold seam edge in surface closed parametric directions.
 *          - Adds no edges for pole boundaries.
 *          - Adds vertices at head-to-tail edge connections and at poles where we expect edge-poleVertex-edge connections.
 *
 *        In UV-space the implied OuterLoop boundary always consists 4 UVtrimCurves which can map to 2, 3, or 4 edges depending
 *          on the number of seams and poles.
 *          UVTrimCurve 1 (v=0):    All control points along v=0, uses U-direction knots/order
 *          UVTrimCurve 2 (u=max):  All control points along u=max, uses V-direction knots/order
 *          UVTrimCurve 3 (v=max):  All control points along v=max (reversed), uses U-direction knots/order
 *          UVTrimCurve 4 (u=0):    All control points along u=0 (reversed), uses V-direction knots/order
 *
 *        Loop goes CCW: (u=0,v=0) -> (u=max,v=0) -> (u=max,v=max) -> (u=0,v=max) -> back
 *
 * ASSUMES: * surface natural boundaries are rectangular in parameter space.
 *          * the edges and vertices of the new loop do not connect to any other edges or faces.
 *          * edges in closed directions are never singular poles.
 *
 * CASES: +--------------------------+--------+--------+-------+-----------------------+---------------------+-------------------+
 *        |     Surf Type            | UDir   | VDir   | Poles |        Lamina         |        Seam         |      Vertex       |
 *        |                          |        |        |       |       EdgeCount       |      EdgeCount      |      Count        |
 *        +--------------------------+--------+--------+-------+-----------------------+---------------------+-------------------+
 *        | plane,           BSpline | open   | open   |   0   | 4 open lamina edges   | 0 seam edges        | 4 corner vertices |
 *        |                  BSpline | open   | open   |   1   | 3 open lamina edges   | 0 seam edges        | 3 corner vertices |
 *        |                  BSpline | open   | open   |   2   | 2 open lamina edges   | 0 seam edges        | 2 corner vertices |
 *        |                  BSpline | open   | open   |   3   | 1 open lamina edges   | 0 seam edges        | 1 corner vertices |
 *        +--------------------------+--------+--------+-------+-----------------------+---------------------+-------------------+
 *        | Cylinder,        BSpline | closed | open   |   0   | 2 closed lamina edges | 1 open seam edge    | 2 corner vertices |
 *        | Full Cone,       BSpline | closed | open   |   1   | 1 closed lamina edges | 1 open seam edge    | 2 corner vertices |
 *        | Truncated Cone,  BSpline | closed | open   |   0   | 2 closed lamina edges | 1 open seam edge    | 2 corner vertices |
 *        | Sphere,          BSpline | closed | open   |   2   | 0        lamina edges | 1 open seam edge    | 2 corner vertices |
 *        +--------------------------+--------+--------+-------+-----------------------+---------------------+-------------------+
 *        | Cylinder,        BSpline | open   | closed |   0   | 2 closed lamina edges | 1 open seam edge    | 2 corner vertices |
 *        | Full Cone,       BSpline | open   | closed |   1   | 1 closed lamina edges | 1 open seam edge    | 2 corner vertices |
 *        | Truncated Cone,  BSpline | open   | closed |   0   | 2 closed lamina edges | 1 open seam edge    | 2 corner vertices |
 *        | Sphere,          BSpline | open   | closed |   2   | 0        lamina edges | 1 open seam edge    | 2 corner vertices |
 *        +--------------------------+--------+--------+-------+-----------------------+---------------------+-------------------+
 *        | Torus,           BSpline | closed | closed |   0   | 0        lamina edge  | 2 closed seam edges | 1 corner vertex   |
 *        +--------------------------+--------+--------+-------+-----------------------+---------------------+-------------------+
 *
 ***************************************************************************************************************************************************/
BrepStatus NaturalBoundaryBuilder::AddImpliedNaturalBoundaryAsLoopForCurrentFace()
{
    return addImpliedNaturalBoundaryAsLoop();
}

BrepStatus NaturalBoundaryBuilder::addImpliedNaturalBoundaryAsLoop()
{
    // extractImpliedBoundaryEdges() must have been called first to populate staged boundary segments.
    uint32_t uiEUCount = m_rState.GetImpliedBoundarySegmentCount();
    if (uiEUCount == 0)
    {
        return BrepStatusError;
    }

    uint32_t euStart = m_rState.GetEdgeuseCount();

    m_sEdgeVertexBuilder.AddStagedImpliedBoundaryEdgeuses(euStart);

    // fix up radial linkage: group edgeuses by edge index — EUs sharing the same edge are radial partners
    std::map<uint32_t, std::vector<uint32_t>> edgeToEUs;
    for (uint32_t ii = 0; ii < uiEUCount; ++ii)
    {
        edgeToEUs[GetStagedEdgeuseEdgeIndex(euStart + ii)].push_back(euStart + ii);
    }
    for (auto& pair : edgeToEUs)
    {
        auto& euIndices = pair.second;
        if (euIndices.size() == 2)
        {
            SetStagedEdgeuseNextRadialIndex(euIndices[0], euIndices[1]);
            SetStagedEdgeuseNextRadialIndex(euIndices[1], euIndices[0]);
        }
        m_rState.SetEdgeRadialList(pair.first, euIndices.front(), euIndices.back());
    }

    m_sTopologyBuilder.AddStagedImpliedBoundaryLoop(uiEUCount);

    m_rState.ClearImpliedBoundarySegments();

    return BrepStatusSuccess;
} // end NaturalBoundaryBuilder::addImpliedNaturalBoundaryAsLoop

/****************************************************************************************************************************************************
 * Geometry evaluation methods - ADDED FOR Item 2 (Coordinate Extraction)
 *
 ***************************************************************************************************************************************************/
