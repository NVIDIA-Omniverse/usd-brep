// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepExtentBuilder.h"

#include <pxr/base/gf/vec3d.h>
#include <pxr/base/vt/array.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

using namespace pxr;

namespace
{

void ExpandExtentWithPoint(pxr::GfVec3d& rExtentMin, pxr::GfVec3d& rExtentMax, bool& bHasGeometry, const pxr::GfVec3d& rPoint)
{
    bHasGeometry = true;
    for (size_t i = 0; i < 3; ++i)
    {
        if (rPoint[i] < rExtentMin[i])
        {
            rExtentMin[i] = rPoint[i];
        }
        if (rPoint[i] > rExtentMax[i])
        {
            rExtentMax[i] = rPoint[i];
        }
    }
}

void ExpandExtentWithRangeBox(pxr::GfVec3d& rExtentMin, pxr::GfVec3d& rExtentMax, bool& bHasGeometry, const pxr::GfVec3d& rCenter, double dRadius)
{
    pxr::GfVec3d sRadius(dRadius, dRadius, dRadius);
    ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, rCenter + sRadius);
    ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, rCenter - sRadius);
}

// Tight AABB of a disk (a cylinder/cone cap): radius dRadius, centered at rCenter, normal rAxis.
// The disk's half-extent along component i is dRadius*sqrt(1 - axis_i^2) (zero along the axis), so a
// cap does not inflate the extent in the axis direction the way a full radius cube would.
void ExpandExtentWithDisk(
    pxr::GfVec3d& rExtentMin,
    pxr::GfVec3d& rExtentMax,
    bool& bHasGeometry,
    const pxr::GfVec3d& rCenter,
    const pxr::GfVec3d& rAxis,
    double dRadius
)
{
    pxr::GfVec3d sAxis = rAxis.GetNormalized();
    pxr::GfVec3d sHalf(
        std::fabs(dRadius) * std::sqrt((std::max)(0.0, 1.0 - sAxis[0] * sAxis[0])),
        std::fabs(dRadius) * std::sqrt((std::max)(0.0, 1.0 - sAxis[1] * sAxis[1])),
        std::fabs(dRadius) * std::sqrt((std::max)(0.0, 1.0 - sAxis[2] * sAxis[2]))
    );
    ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, rCenter + sHalf);
    ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, rCenter - sHalf);
}

} // namespace

void StagedBrepExtentBuilder::ExpandExtentWithStagedEdgeCurveSet(
    bool bWireEdgeSet,
    pxr::GfVec3d& rExtentMin,
    pxr::GfVec3d& rExtentMax,
    bool& bHasGeometry
)
{
    const uint32_t uiEdgeCount = bWireEdgeSet ? m_rState.GetWireEdgeCount() : m_rState.GetEdgeCount();
    for (uint32_t uiEdgeIndex = 0; uiEdgeIndex < uiEdgeCount; ++uiEdgeIndex)
    {
        const StagedCurveData* pCurve = bWireEdgeSet ? m_rState.GetWireEdgeCurveRecord(uiEdgeIndex) : m_rState.GetEdgeCurveRecord(uiEdgeIndex);
        const StagedEdgeData* pEdge = bWireEdgeSet ? m_rState.GetWireEdgeRecord(uiEdgeIndex) : m_rState.GetEdgeRecord(uiEdgeIndex);
        const pxr::GfRange1d* pInterval = pEdge ? &pEdge->sInterval : nullptr;
        if (!pCurve)
        {
            continue;
        }

        if (pCurve->eCurveType == StagedCurveType::BSplineCurve)
        {
            for (size_t i = 0; i < pCurve->vNurbControlVertices.size(); ++i)
            {
                ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, pCurve->vNurbControlVertices[i]);
            }
            continue;
        }

        if (pCurve->eCurveType == StagedCurveType::Line)
        {
            if (pInterval)
            {
                ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, pCurve->sOrigin + pInterval->GetMin() * pCurve->sDirection);
                ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, pCurve->sOrigin + pInterval->GetMax() * pCurve->sDirection);
            }
            continue;
        }

        if (pCurve->eCurveType == StagedCurveType::Circle)
        {
            ExpandExtentWithRangeBox(rExtentMin, rExtentMax, bHasGeometry, pCurve->sCenter, pCurve->dRadius);
            continue;
        }

        if (pCurve->eCurveType == StagedCurveType::Ellipse)
        {
            double dMaxRadius = (std::max)(pCurve->dXRadius, pCurve->dYRadius);
            ExpandExtentWithRangeBox(rExtentMin, rExtentMax, bHasGeometry, pCurve->sCenter, dMaxRadius);
            continue;
        }
    }
}


void StagedBrepExtentBuilder::ExpandExtentWithStagedFaceSurfaces(pxr::GfVec3d& rExtentMin, pxr::GfVec3d& rExtentMax, bool& bHasGeometry)
{
    for (uint32_t uiSurfaceIndex = 0; uiSurfaceIndex < m_rState.GetFaceSurfaceCount(); ++uiSurfaceIndex)
    {
        const StagedSurfaceData* pSurface = m_rState.GetFaceSurfaceRecord(uiSurfaceIndex);
        if (!pSurface)
        {
            continue;
        }

        if (pSurface->eSurfaceType == StagedSurfaceType::BSplineSurface)
        {
            for (size_t i = 0; i < pSurface->vNurbControlVertices.size(); ++i)
            {
                ExpandExtentWithPoint(rExtentMin, rExtentMax, bHasGeometry, pSurface->vNurbControlVertices[i]);
            }
        }
        else if (pSurface->eSurfaceType == StagedSurfaceType::Sphere)
        {
            ExpandExtentWithRangeBox(rExtentMin, rExtentMax, bHasGeometry, pSurface->sCenter, pSurface->dRadius);
        }
        else if (pSurface->eSurfaceType == StagedSurfaceType::Plane)
        {
            const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiSurfaceIndex);
            if (pFaceData)
            {
                const pxr::GfRange2d sDomain = pFaceData->sUVDomain;
                pxr::GfVec3d sYDirection = pxr::GfCross(pSurface->sAxis, pSurface->sRefDirection);
                ExpandExtentWithPoint(
                    rExtentMin,
                    rExtentMax,
                    bHasGeometry,
                    pSurface->sOrigin + sDomain.GetMin()[0] * pSurface->sRefDirection + sDomain.GetMin()[1] * sYDirection
                );
                ExpandExtentWithPoint(
                    rExtentMin,
                    rExtentMax,
                    bHasGeometry,
                    pSurface->sOrigin + sDomain.GetMax()[0] * pSurface->sRefDirection + sDomain.GetMin()[1] * sYDirection
                );
                ExpandExtentWithPoint(
                    rExtentMin,
                    rExtentMax,
                    bHasGeometry,
                    pSurface->sOrigin + sDomain.GetMin()[0] * pSurface->sRefDirection + sDomain.GetMax()[1] * sYDirection
                );
                ExpandExtentWithPoint(
                    rExtentMin,
                    rExtentMax,
                    bHasGeometry,
                    pSurface->sOrigin + sDomain.GetMax()[0] * pSurface->sRefDirection + sDomain.GetMax()[1] * sYDirection
                );
            }
        }
        else if (pSurface->eSurfaceType == StagedSurfaceType::Cylinder)
        {
            const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiSurfaceIndex);
            if (pFaceData)
            {
                const pxr::GfRange2d sDomain = pFaceData->sUVDomain;
                pxr::GfVec3d sAxisStart = pSurface->sOrigin + sDomain.GetMin()[1] * pSurface->sAxis;
                pxr::GfVec3d sAxisEnd = pSurface->sOrigin + sDomain.GetMax()[1] * pSurface->sAxis;
                ExpandExtentWithDisk(rExtentMin, rExtentMax, bHasGeometry, sAxisStart, pSurface->sAxis, pSurface->dRadius);
                ExpandExtentWithDisk(rExtentMin, rExtentMax, bHasGeometry, sAxisEnd, pSurface->sAxis, pSurface->dRadius);
            }
        }
        else if (pSurface->eSurfaceType == StagedSurfaceType::Cone)
        {
            const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiSurfaceIndex);
            if (pFaceData)
            {
                const pxr::GfRange2d sDomain = pFaceData->sUVDomain;
                double dRadiusStart = pSurface->dRadius + sDomain.GetMin()[1] * std::tan(pSurface->dSemiAngle);
                double dRadiusEnd = pSurface->dRadius + sDomain.GetMax()[1] * std::tan(pSurface->dSemiAngle);
                pxr::GfVec3d sAxisStart = pSurface->sOrigin + sDomain.GetMin()[1] * pSurface->sAxis;
                pxr::GfVec3d sAxisEnd = pSurface->sOrigin + sDomain.GetMax()[1] * pSurface->sAxis;
                ExpandExtentWithDisk(rExtentMin, rExtentMax, bHasGeometry, sAxisStart, pSurface->sAxis, std::fabs(dRadiusStart));
                ExpandExtentWithDisk(rExtentMin, rExtentMax, bHasGeometry, sAxisEnd, pSurface->sAxis, std::fabs(dRadiusEnd));
            }
        }
        else if (pSurface->eSurfaceType == StagedSurfaceType::Torus)
        {
            double dOuterRadius = pSurface->dMajorRadius + pSurface->dMinorRadius;
            ExpandExtentWithRangeBox(rExtentMin, rExtentMax, bHasGeometry, pSurface->sOrigin, dOuterRadius);
        }
    }
}


StagedBrepExtentBuilder::StagedBrepExtentBuilder(BrepStagingState& rState) : m_rState(rState)
{
}


void StagedBrepExtentBuilder::UpdateExtent()
{
    static double dMax = std::numeric_limits<double>::max();
    GfVec3d extentMin(dMax, dMax, dMax);
    GfVec3d extentMax(-dMax, -dMax, -dMax);
    bool hasGeometry = false;

    // Helper lambda to update extent from a point
    auto updateExtentFromPoint = [&](const pxr::GfVec3d& rPt)
    {
        hasGeometry = true;
        for (size_t i = 0; i < 3; ++i)
        {
            if (rPt[i] < extentMin[i])
            {
                extentMin[i] = rPt[i];
            }
            if (rPt[i] > extentMax[i])
            {
                extentMax[i] = rPt[i];
            }
        }
    };
    // Include vertex positions
    for (uint32_t uiVertexIndex = 0; uiVertexIndex < m_rState.GetVertexPointCount(); ++uiVertexIndex)
    {
        const pxr::GfVec3d* pPoint = m_rState.GetVertexPointRecord(uiVertexIndex);
        if (pPoint)
        {
            updateExtentFromPoint(*pPoint);
        }
    }

    // Include surface control points and analytic surface bounds.
    ExpandExtentWithStagedFaceSurfaces(extentMin, extentMax, hasGeometry);

    // Include edge and wire-edge curve geometry
    ExpandExtentWithStagedEdgeCurveSet(false, extentMin, extentMax, hasGeometry);
    ExpandExtentWithStagedEdgeCurveSet(true, extentMin, extentMax, hasGeometry);

    if (hasGeometry)
    {
        m_rState.SetExtent(extentMin, extentMax);
    }
    else
    {
        // Set reasonable default values if no geometry was processed
        m_rState.SetExtent(GfVec3d(-1, -1, -1), GfVec3d(1, 1, 1));
    }
}
