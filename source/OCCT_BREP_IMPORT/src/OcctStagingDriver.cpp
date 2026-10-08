// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OcctStagingDriver.h"

#include "BrepCoedgeBuilder.h"
#include "BrepEdgeVertexBuilder.h"
#include "BrepExtentBuilder.h"
#include "BrepNaturalBoundaryBuilder.h"
#include "BrepSourceData.h"
#include "BrepStagingBuilder.h"
#include "BrepTopologyBuilder.h"
#include "OcctGeometryConvert.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace occt
{

namespace
{

// Shift an angular interval by whole 2*pi periods so its max lands in the primary period (0, 2*pi],
// matching the BrepArray validator's BA_630/BA_631 rules. For 2*pi-periodic parameters (circle/
// ellipse, cylinder/cone/sphere/torus U) this is geometrically exact. OCCT can report a nominally
// full period slightly above or below 2*pi; snap either primary-boundary representation exactly to
// [0, 2*pi] so strict analytic constructors do not reject it. Non-finite intervals are left untouched.
void NormalizeAngularInterval(double& rLo, double& rHi)
{
    const double dTwoPi = 2.0 * M_PI;
    const double dPeriodTol = 1.0e-6;
    if (!std::isfinite(rHi) || !std::isfinite(rLo))
    {
        return;
    }
    const double dShift = dTwoPi * std::floor((rHi - 1.0e-7) / dTwoPi);
    rLo -= dShift;
    rHi -= dShift;

    const double dSpan = rHi - rLo;
    if (std::abs(dSpan - dTwoPi) <= dPeriodTol)
    {
        const bool bNearPrimaryBoundary = std::abs(rLo) <= dPeriodTol && std::abs(rHi - dTwoPi) <= dPeriodTol;
        const bool bNearPreviousBoundary = std::abs(rLo + dTwoPi) <= dPeriodTol && std::abs(rHi) <= dPeriodTol;
        if (bNearPrimaryBoundary || bNearPreviousBoundary)
        {
            rLo = 0.0;
            rHi = dTwoPi;
        }
        else
        {
            // A shifted full-period edge may have a meaningful phase. Preserve its lower endpoint
            // and correct only accumulated span error instead of forcing it onto the surface seam.
            rHi = rLo + dTwoPi;
        }
    }
}

bool IsPeriodicUSurface(StagedSurfaceType eType)
{
    return eType == StagedSurfaceType::Cylinder || eType == StagedSurfaceType::Cone || eType == StagedSurfaceType::Sphere ||
           eType == StagedSurfaceType::Torus;
}

struct SurfaceParameterAdjustment
{
    double dUScale = 1.0;
    double dVScale = 1.0;
    bool bReverseOrientation = false;
};

const SSurface* AnalyticBasisSurface(const SSurface& rSurface)
{
    const SSurface* pSurface = &rSurface;
    while (pSurface && (pSurface->eType == ESurfaceType::RectangularTrimmed || pSurface->eType == ESurfaceType::Offset))
    {
        pSurface = pSurface->pBasisSurface.get();
    }
    if (!pSurface)
    {
        return nullptr;
    }
    switch (pSurface->eType)
    {
        case ESurfaceType::Plane:
        case ESurfaceType::Cylinder:
        case ESurfaceType::Cone:
        case ESurfaceType::Sphere:
        case ESurfaceType::Torus:
            return pSurface;
        default:
            return nullptr;
    }
}

pxr::GfVec3d TransformUnitDirection(const pxr::GfMatrix4d& rTransform, const SDir3& rDirection)
{
    pxr::GfVec3d sResult = rTransform.TransformDir(pxr::GfVec3d(rDirection.dX, rDirection.dY, rDirection.dZ));
    sResult.Normalize();
    return sResult;
}

// OCCT Ax3 placements may be indirect. BrepArray analytic schemas store only axis + refDirection,
// so their Y direction is always reconstructed as axis x refDirection. Record the corresponding
// UV reflection and whether it reverses the surface's natural orientation.
SurfaceParameterAdjustment ComputeSurfaceParameterAdjustment(
    const SSurface& rSourceSurface,
    const pxr::GfMatrix4d& rTransform,
    const StagedSurfaceData& rStagedSurface
)
{
    SurfaceParameterAdjustment sResult;
    const SSurface* pBasis = AnalyticBasisSurface(rSourceSurface);
    if (!pBasis)
    {
        return sResult;
    }

    const pxr::GfVec3d sSourceY = TransformUnitDirection(rTransform, pBasis->sYAxis);
    pxr::GfVec3d sStagedAxis = rStagedSurface.sAxis;
    pxr::GfVec3d sStagedRef = rStagedSurface.sRefDirection;
    if (sSourceY.GetLength() == 0.0 || sStagedAxis.Normalize() == 0.0 || sStagedRef.Normalize() == 0.0)
    {
        return sResult;
    }
    pxr::GfVec3d sStagedY = pxr::GfCross(sStagedAxis, sStagedRef);
    if (sStagedY.Normalize() == 0.0)
    {
        return sResult;
    }

    const double dYSign = pxr::GfDot(sStagedY, sSourceY) < 0.0 ? -1.0 : 1.0;
    if (pBasis->eType == ESurfaceType::Plane)
    {
        // Plane domains are rebuilt by projecting the 3D boundary into the staged frame. Reflect
        // source pcurve V as well when an indirect Ax3 supplied the opposite Y direction.
        sResult.dVScale = dYSign;
        sResult.bReverseOrientation = dYSign < 0.0;
        return sResult;
    }

    sResult.dUScale = dYSign;
    double dOrientationSign = dYSign;
    if (pBasis->eType == ESurfaceType::Cone)
    {
        const pxr::GfVec3d sSourceAxis = TransformUnitDirection(rTransform, pBasis->sAxis);
        if (sSourceAxis.GetLength() != 0.0)
        {
            dOrientationSign *= pxr::GfDot(sStagedAxis, sSourceAxis) < 0.0 ? -1.0 : 1.0;
        }
    }
    sResult.bReverseOrientation = dOrientationSign < 0.0;
    return sResult;
}

pxr::GfRange2d ScaleURange(const pxr::GfRange2d& rRange, double dScale)
{
    if (rRange.IsEmpty() || dScale >= 0.0)
    {
        return rRange;
    }
    return pxr::GfRange2d(
        pxr::GfVec2d(dScale * rRange.GetMax()[0], rRange.GetMin()[1]),
        pxr::GfVec2d(dScale * rRange.GetMin()[0], rRange.GetMax()[1])
    );
}

bool IsSinglyPeriodicRepairSurface(StagedSurfaceType eType)
{
    return eType == StagedSurfaceType::Cylinder || eType == StagedSurfaceType::Cone || eType == StagedSurfaceType::Sphere;
}

bool PointsClose(const pxr::GfVec3d& a, const pxr::GfVec3d& b, double dTol = 1.0e-6)
{
    const double dScale = std::max({ a.GetLength(), b.GetLength(), 1.0 });
    return (a - b).GetLength() <= dTol * dScale;
}

bool IsFiniteNonEmptyRange(const pxr::GfRange1d& rRange)
{
    const double dMin = rRange.GetMin();
    const double dMax = rRange.GetMax();
    return std::isfinite(dMin) && std::isfinite(dMax) && dMax > dMin;
}

bool IsFiniteRange(const pxr::GfRange1d& rRange)
{
    const double dMin = rRange.GetMin();
    const double dMax = rRange.GetMax();
    return std::isfinite(dMin) && std::isfinite(dMax) && dMax >= dMin;
}

bool IsFiniteNonEmptyRange(const pxr::GfRange2d& rRange)
{
    const pxr::GfVec2d sMin = rRange.GetMin();
    const pxr::GfVec2d sMax = rRange.GetMax();
    return std::isfinite(sMin[0]) && std::isfinite(sMin[1]) && std::isfinite(sMax[0]) && std::isfinite(sMax[1]) && sMax[0] > sMin[0] &&
           sMax[1] > sMin[1];
}

bool BuildLineCurveFromEndpoints(const pxr::GfVec3d& rStart, const pxr::GfVec3d& rEnd, StagedCurveData& rCurve, pxr::GfRange1d& rInterval)
{
    pxr::GfVec3d sDir = rEnd - rStart;
    const double dLen = sDir.Normalize();
    if (dLen <= 1.0e-12)
    {
        return false;
    }

    rCurve = StagedCurveData{};
    rCurve.eCurveType = StagedCurveType::Line;
    rCurve.sOrigin = rStart;
    rCurve.sDirection = sDir;
    rInterval = pxr::GfRange1d(0.0, dLen);
    return true;
}

double NormalizeAnglePositive(double dU)
{
    const double dTwoPi = 2.0 * M_PI;
    dU = std::fmod(dU, dTwoPi);
    if (dU < 0.0)
    {
        dU += dTwoPi;
    }
    return dU;
}

double UnwrapAngleNear(double dU, double dReference)
{
    const double dTwoPi = 2.0 * M_PI;
    while (dU - dReference > M_PI)
    {
        dU -= dTwoPi;
    }
    while (dU - dReference < -M_PI)
    {
        dU += dTwoPi;
    }
    return dU;
}

double ClampUnit(double dValue)
{
    return std::max(-1.0, std::min(1.0, dValue));
}

pxr::GfVec3d SurfacePerpDirection(const StagedSurfaceData& rSurface)
{
    pxr::GfVec3d sPerp = pxr::GfCross(rSurface.sAxis, rSurface.sRefDirection);
    if (sPerp.Normalize() == 0.0)
    {
        return pxr::GfVec3d(0.0, 1.0, 0.0);
    }
    return sPerp;
}

pxr::GfVec3d SurfaceRadialAtU(const StagedSurfaceData& rSurface, double dU)
{
    pxr::GfVec3d sRef = rSurface.sRefDirection;
    if (sRef.Normalize() == 0.0)
    {
        return pxr::GfVec3d(1.0, 0.0, 0.0);
    }
    const pxr::GfVec3d sPerp = SurfacePerpDirection(rSurface);
    pxr::GfVec3d sRadial = std::cos(dU) * sRef + std::sin(dU) * sPerp;
    if (sRadial.Normalize() == 0.0)
    {
        return sRef;
    }
    return sRadial;
}

bool SurfaceParamAtPoint(const StagedSurfaceData& rSurface, const pxr::GfVec3d& rPoint, double& rU, double& rV)
{
    pxr::GfVec3d sAxis = rSurface.sAxis;
    pxr::GfVec3d sRef = rSurface.sRefDirection;
    if (sAxis.Normalize() == 0.0 || sRef.Normalize() == 0.0)
    {
        return false;
    }
    const pxr::GfVec3d sPerp = SurfacePerpDirection(rSurface);
    pxr::GfVec3d sVec;
    switch (rSurface.eSurfaceType)
    {
        case StagedSurfaceType::Sphere:
            sVec = rPoint - rSurface.sCenter;
            if (sVec.Normalize() == 0.0 || rSurface.dRadius <= 0.0)
            {
                return false;
            }
            rU = NormalizeAnglePositive(std::atan2(pxr::GfDot(sVec, sPerp), pxr::GfDot(sVec, sRef)));
            rV = std::asin(ClampUnit(pxr::GfDot(sVec, sAxis)));
            return true;
        case StagedSurfaceType::Cylinder:
        case StagedSurfaceType::Cone:
        {
            sVec = rPoint - rSurface.sOrigin;
            rV = pxr::GfDot(sVec, sAxis);
            const pxr::GfVec3d sRadialVec = sVec - rV * sAxis;
            if (sRadialVec.GetLength() <= 1.0e-12)
            {
                return false;
            }
            rU = NormalizeAnglePositive(std::atan2(pxr::GfDot(sRadialVec, sPerp), pxr::GfDot(sRadialVec, sRef)));
            return true;
        }
        case StagedSurfaceType::Torus:
        {
            if (rSurface.dMajorRadius <= 0.0 || rSurface.dMinorRadius <= 0.0)
            {
                return false;
            }
            sVec = rPoint - rSurface.sOrigin;
            const double dAxisOffset = pxr::GfDot(sVec, sAxis);
            pxr::GfVec3d sRadialVec = sVec - dAxisOffset * sAxis;
            const double dRadialLength = sRadialVec.Normalize();
            if (dRadialLength <= 1.0e-12)
            {
                return false;
            }
            rU = NormalizeAnglePositive(std::atan2(pxr::GfDot(sRadialVec, sPerp), pxr::GfDot(sRadialVec, sRef)));
            rV = NormalizeAnglePositive(std::atan2(dAxisOffset, dRadialLength - rSurface.dMajorRadius));
            return true;
        }
        default:
            return false;
    }
}

pxr::GfVec3d SurfacePointAtParam(const StagedSurfaceData& rSurface, double dU, double dV)
{
    const pxr::GfVec3d sRadial = SurfaceRadialAtU(rSurface, dU);
    switch (rSurface.eSurfaceType)
    {
        case StagedSurfaceType::Sphere:
            return rSurface.sCenter + rSurface.dRadius * (std::cos(dV) * sRadial + std::sin(dV) * rSurface.sAxis);
        case StagedSurfaceType::Cylinder:
            return rSurface.sOrigin + dV * rSurface.sAxis + rSurface.dRadius * sRadial;
        case StagedSurfaceType::Cone:
        {
            const double dRadius = rSurface.dRadius + dV * std::tan(rSurface.dSemiAngle);
            return rSurface.sOrigin + dV * rSurface.sAxis + dRadius * sRadial;
        }
        case StagedSurfaceType::Torus:
            return rSurface.sOrigin + (rSurface.dMajorRadius + rSurface.dMinorRadius * std::cos(dV)) * sRadial +
                   rSurface.dMinorRadius * std::sin(dV) * rSurface.sAxis;
        default:
            return pxr::GfVec3d(0.0, 0.0, 0.0);
    }
}

struct RationalQuadraticArc
{
    pxr::VtArray<double> vAngles;
    pxr::VtArray<double> vWeights;
    pxr::VtArray<double> vScales;
    pxr::VtArray<double> vKnots;
};

bool BuildRationalQuadraticArc(double dFirst, double dLast, RationalQuadraticArc& rArc)
{
    rArc = RationalQuadraticArc{};
    const double dSpan = dLast - dFirst;
    if (!std::isfinite(dFirst) || !std::isfinite(dLast) || dSpan <= 1.0e-12)
    {
        return false;
    }

    const int iSegmentCount = std::max(1, static_cast<int>(std::ceil(dSpan / (0.5 * M_PI) - 1.0e-10)));
    if (iSegmentCount > 4096)
    {
        return false;
    }
    const double dSegmentSpan = dSpan / static_cast<double>(iSegmentCount);
    const double dMidWeight = std::cos(0.5 * dSegmentSpan);
    if (dMidWeight <= 1.0e-12)
    {
        return false;
    }

    const int iControlPointCount = 2 * iSegmentCount + 1;
    rArc.vAngles.reserve(static_cast<size_t>(iControlPointCount));
    rArc.vWeights.reserve(static_cast<size_t>(iControlPointCount));
    rArc.vScales.reserve(static_cast<size_t>(iControlPointCount));
    for (int ii = 0; ii < iControlPointCount; ++ii)
    {
        const int iSegment = ii / 2;
        if ((ii % 2) == 0)
        {
            rArc.vAngles.push_back(dFirst + static_cast<double>(iSegment) * dSegmentSpan);
            rArc.vWeights.push_back(1.0);
            rArc.vScales.push_back(1.0);
        }
        else
        {
            rArc.vAngles.push_back(dFirst + (static_cast<double>(iSegment) + 0.5) * dSegmentSpan);
            rArc.vWeights.push_back(dMidWeight);
            rArc.vScales.push_back(1.0 / dMidWeight);
        }
    }

    rArc.vKnots = { dFirst, dFirst, dFirst };
    for (int ii = 1; ii < iSegmentCount; ++ii)
    {
        const double dKnot = dFirst + static_cast<double>(ii) * dSegmentSpan;
        rArc.vKnots.push_back(dKnot);
        rArc.vKnots.push_back(dKnot);
    }
    rArc.vKnots.push_back(dLast);
    rArc.vKnots.push_back(dLast);
    rArc.vKnots.push_back(dLast);
    return true;
}

void ReverseUvNurbCurve(StagedCurveData2d& rCurve)
{
    if (!rCurve.bValid || rCurve.uiNurbOrder == 0 || rCurve.uiNurbVertexCount == 0 ||
        rCurve.vNurbKnots.size() != rCurve.uiNurbVertexCount + rCurve.uiNurbOrder)
    {
        return;
    }

    std::reverse(rCurve.vNurbControlVertices.begin(), rCurve.vNurbControlVertices.end());
    std::reverse(rCurve.vNurbWeights.begin(), rCurve.vNurbWeights.end());
    const double dNaturalMin = rCurve.vNurbKnots[rCurve.uiNurbOrder - 1];
    const double dNaturalMax = rCurve.vNurbKnots[rCurve.uiNurbVertexCount];
    pxr::VtArray<double> vReversedKnots;
    vReversedKnots.reserve(rCurve.vNurbKnots.size());
    for (auto sIt = rCurve.vNurbKnots.rbegin(); sIt != rCurve.vNurbKnots.rend(); ++sIt)
    {
        vReversedKnots.push_back(dNaturalMin + dNaturalMax - *sIt);
    }
    rCurve.vNurbKnots = std::move(vReversedKnots);
}

bool EvaluateUvNurbEndpoint(const StagedCurveData2d& rCurve, bool bEnd, pxr::GfVec2d& rPoint)
{
    if (!rCurve.bValid || rCurve.uiNurbOrder < 2 || rCurve.uiNurbVertexCount < rCurve.uiNurbOrder ||
        rCurve.vNurbControlVertices.size() != rCurve.uiNurbVertexCount || rCurve.vNurbKnots.size() != rCurve.uiNurbVertexCount + rCurve.uiNurbOrder)
    {
        return false;
    }

    const int p = static_cast<int>(rCurve.uiNurbOrder) - 1;
    const int n = static_cast<int>(rCurve.uiNurbVertexCount);
    const int k = bEnd ? (n - 1) : p;
    const double t = bEnd ? rCurve.vNurbKnots[static_cast<size_t>(n)] : rCurve.vNurbKnots[static_cast<size_t>(p)];
    std::vector<pxr::GfVec2d> vPoints(static_cast<size_t>(p + 1));
    std::vector<double> vWeights(static_cast<size_t>(p + 1), 1.0);
    for (int jj = 0; jj <= p; ++jj)
    {
        const int iControl = k - p + jj;
        const double dWeight = rCurve.vNurbWeights.size() == rCurve.uiNurbVertexCount ? rCurve.vNurbWeights[static_cast<size_t>(iControl)] : 1.0;
        vPoints[static_cast<size_t>(jj)] = rCurve.vNurbControlVertices[static_cast<size_t>(iControl)] * dWeight;
        vWeights[static_cast<size_t>(jj)] = dWeight;
    }
    for (int rr = 1; rr <= p; ++rr)
    {
        for (int jj = p; jj >= rr; --jj)
        {
            const int iKnot = k - p + jj;
            const double dDen = rCurve.vNurbKnots[static_cast<size_t>(iKnot + p - rr + 1)] - rCurve.vNurbKnots[static_cast<size_t>(iKnot)];
            const double dAlpha = std::abs(dDen) > 1.0e-12 ? (t - rCurve.vNurbKnots[static_cast<size_t>(iKnot)]) / dDen : 0.0;
            vPoints[static_cast<size_t>(jj)] = (1.0 - dAlpha) * vPoints[static_cast<size_t>(jj - 1)] + dAlpha * vPoints[static_cast<size_t>(jj)];
            vWeights[static_cast<size_t>(jj)] = (1.0 - dAlpha) * vWeights[static_cast<size_t>(jj - 1)] + dAlpha * vWeights[static_cast<size_t>(jj)];
        }
    }
    const double dWeight = vWeights[static_cast<size_t>(p)];
    if (std::abs(dWeight) <= 1.0e-12)
    {
        return false;
    }
    rPoint = vPoints[static_cast<size_t>(p)] / dWeight;
    return std::isfinite(rPoint[0]) && std::isfinite(rPoint[1]);
}

bool StagedUvLoopCloses(const BrepStagingState& rState, uint32_t uiFirstEdgeuse, uint32_t uiEdgeuseCount)
{
    constexpr double kUvClosureTolerance = 1.0e-6;
    if (uiEdgeuseCount == 0)
    {
        return false;
    }
    for (uint32_t ii = 0; ii < uiEdgeuseCount; ++ii)
    {
        const StagedCurveData2d* pCurve = rState.GetEdgeuseCurve2d(uiFirstEdgeuse + ii);
        const StagedCurveData2d* pNextCurve = rState.GetEdgeuseCurve2d(uiFirstEdgeuse + (ii + 1) % uiEdgeuseCount);
        pxr::GfVec2d sEnd;
        pxr::GfVec2d sNextStart;
        if (!pCurve || !pNextCurve || !EvaluateUvNurbEndpoint(*pCurve, true, sEnd) || !EvaluateUvNurbEndpoint(*pNextCurve, false, sNextStart) ||
            (sEnd - sNextStart).GetLength() > kUvClosureTolerance)
        {
            return false;
        }
    }
    return true;
}

bool StagedUvLoopHasCompleteCoverage(const BrepStagingState& rState, uint32_t uiFirstEdgeuse, uint32_t uiEdgeuseCount)
{
    if (uiEdgeuseCount == 0)
    {
        return false;
    }
    for (uint32_t ii = 0; ii < uiEdgeuseCount; ++ii)
    {
        const StagedCurveData2d* pCurve = rState.GetEdgeuseCurve2d(uiFirstEdgeuse + ii);
        if (!pCurve || !pCurve->bValid)
        {
            return false;
        }
    }
    return true;
}

bool UvCurveControlPolygonFitsDomain(const StagedCurveData2d& rCurve, const pxr::GfRange2d& rDomain)
{
    if (!rCurve.bValid || !IsFiniteNonEmptyRange(rDomain))
    {
        return false;
    }

    const pxr::GfVec2d sMin = rDomain.GetMin();
    const pxr::GfVec2d sMax = rDomain.GetMax();
    const double dUMargin = 0.5 * std::max(std::abs(sMax[0] - sMin[0]), 1.0);
    const double dVMargin = 0.5 * std::max(std::abs(sMax[1] - sMin[1]), 1.0);
    for (const pxr::GfVec2d& rControlVertex : rCurve.vNurbControlVertices)
    {
        if (!std::isfinite(rControlVertex[0]) || !std::isfinite(rControlVertex[1]) || rControlVertex[0] < sMin[0] - dUMargin ||
            rControlVertex[0] > sMax[0] + dUMargin || rControlVertex[1] < sMin[1] - dVMargin || rControlVertex[1] > sMax[1] + dVMargin)
        {
            return false;
        }
    }
    return true;
}

bool LowerAngularCurveToNurbs(StagedCurveData& rCurve, const pxr::GfRange1d& rInterval)
{
    if ((rCurve.eCurveType != StagedCurveType::Circle && rCurve.eCurveType != StagedCurveType::Ellipse) || !IsFiniteNonEmptyRange(rInterval))
    {
        return false;
    }

    pxr::GfVec3d sAxis = rCurve.sAxis;
    pxr::GfVec3d sRef = rCurve.sRefDirection;
    if (sAxis.Normalize() == 0.0 || sRef.Normalize() == 0.0)
    {
        return false;
    }
    pxr::GfVec3d sSide = pxr::GfCross(sAxis, sRef);
    if (sSide.Normalize() == 0.0)
    {
        return false;
    }

    RationalQuadraticArc sArc;
    if (!BuildRationalQuadraticArc(rInterval.GetMin(), rInterval.GetMax(), sArc))
    {
        return false;
    }

    const double dXRadius = (rCurve.eCurveType == StagedCurveType::Circle) ? rCurve.dRadius : rCurve.dXRadius;
    const double dYRadius = (rCurve.eCurveType == StagedCurveType::Circle) ? rCurve.dRadius : rCurve.dYRadius;
    StagedCurveData sNurb;
    sNurb.eCurveType = StagedCurveType::BSplineCurve;
    sNurb.uiNurbOrder = 3;
    sNurb.uiNurbVertexCount = static_cast<uint32_t>(sArc.vAngles.size());
    sNurb.vNurbKnots = sArc.vKnots;
    sNurb.vNurbWeights = sArc.vWeights;
    sNurb.vNurbControlVertices.reserve(sArc.vAngles.size());
    for (size_t ii = 0; ii < sArc.vAngles.size(); ++ii)
    {
        const double dAngle = sArc.vAngles[ii];
        const double dScale = sArc.vScales[ii];
        sNurb.vNurbControlVertices.push_back(rCurve.sCenter + dScale * (dXRadius * std::cos(dAngle) * sRef + dYRadius * std::sin(dAngle) * sSide));
    }
    rCurve = std::move(sNurb);
    return true;
}

bool RelocateAnalyticUSeam(StagedSurfaceData& rSurface, double dOldUAtNewSeam)
{
    if (!IsPeriodicUSurface(rSurface.eSurfaceType) || !std::isfinite(dOldUAtNewSeam))
    {
        return false;
    }
    pxr::GfVec3d sAxis = rSurface.sAxis;
    pxr::GfVec3d sRef = rSurface.sRefDirection;
    if (sAxis.Normalize() == 0.0 || sRef.Normalize() == 0.0)
    {
        return false;
    }
    pxr::GfVec3d sSide = pxr::GfCross(sAxis, sRef);
    if (sSide.Normalize() == 0.0)
    {
        return false;
    }
    rSurface.sRefDirection = std::cos(dOldUAtNewSeam) * sRef + std::sin(dOldUAtNewSeam) * sSide;
    return rSurface.sRefDirection.Normalize() != 0.0;
}

bool LowerAnalyticSurfacePatchToNurbs(const StagedSurfaceData& rSurface, const pxr::GfRange2d& rDomain, StagedSurfaceData& rNurb)
{
    if (!IsPeriodicUSurface(rSurface.eSurfaceType) || !IsFiniteNonEmptyRange(rDomain))
    {
        return false;
    }

    pxr::GfVec3d sAxis = rSurface.sAxis;
    pxr::GfVec3d sRef = rSurface.sRefDirection;
    if (sAxis.Normalize() == 0.0 || sRef.Normalize() == 0.0)
    {
        return false;
    }
    pxr::GfVec3d sSide = pxr::GfCross(sAxis, sRef);
    if (sSide.Normalize() == 0.0)
    {
        return false;
    }

    RationalQuadraticArc sUArc;
    if (!BuildRationalQuadraticArc(rDomain.GetMin()[0], rDomain.GetMax()[0], sUArc))
    {
        return false;
    }

    const bool bAngularV = rSurface.eSurfaceType == StagedSurfaceType::Sphere || rSurface.eSurfaceType == StagedSurfaceType::Torus;
    RationalQuadraticArc sVArc;
    if (bAngularV && !BuildRationalQuadraticArc(rDomain.GetMin()[1], rDomain.GetMax()[1], sVArc))
    {
        return false;
    }

    rNurb = StagedSurfaceData{};
    rNurb.eSurfaceType = StagedSurfaceType::BSplineSurface;
    rNurb.uiNurbUOrder = 3;
    rNurb.uiNurbUVertexCount = static_cast<uint32_t>(sUArc.vAngles.size());
    rNurb.vNurbUKnots = sUArc.vKnots;
    if (bAngularV)
    {
        rNurb.uiNurbVOrder = 3;
        rNurb.uiNurbVVertexCount = static_cast<uint32_t>(sVArc.vAngles.size());
        rNurb.vNurbVKnots = sVArc.vKnots;
    }
    else
    {
        rNurb.uiNurbVOrder = 2;
        rNurb.uiNurbVVertexCount = 2;
        rNurb.vNurbVKnots = { rDomain.GetMin()[1], rDomain.GetMin()[1], rDomain.GetMax()[1], rDomain.GetMax()[1] };
    }

    const size_t uiVCount = static_cast<size_t>(rNurb.uiNurbVVertexCount);
    rNurb.vNurbControlVertices.reserve(sUArc.vAngles.size() * uiVCount);
    rNurb.vNurbWeights.reserve(sUArc.vAngles.size() * uiVCount);
    for (size_t iu = 0; iu < sUArc.vAngles.size(); ++iu)
    {
        const double dU = sUArc.vAngles[iu];
        const pxr::GfVec3d sRadial = sUArc.vScales[iu] * (std::cos(dU) * sRef + std::sin(dU) * sSide);
        const double dUWeight = sUArc.vWeights[iu];
        for (size_t iv = 0; iv < uiVCount; ++iv)
        {
            pxr::GfVec3d sPoint;
            double dVWeight = 1.0;
            if (rSurface.eSurfaceType == StagedSurfaceType::Cylinder || rSurface.eSurfaceType == StagedSurfaceType::Cone)
            {
                const double dV = (iv == 0) ? rDomain.GetMin()[1] : rDomain.GetMax()[1];
                const double dRadius = rSurface.eSurfaceType == StagedSurfaceType::Cylinder ? rSurface.dRadius :
                                                                                              rSurface.dRadius + dV * std::tan(rSurface.dSemiAngle);
                sPoint = rSurface.sOrigin + dV * sAxis + dRadius * sRadial;
            }
            else
            {
                const double dV = sVArc.vAngles[iv];
                const double dVScale = sVArc.vScales[iv];
                const double dRadialCoefficient = dVScale * std::cos(dV);
                const double dAxisCoefficient = dVScale * std::sin(dV);
                dVWeight = sVArc.vWeights[iv];
                if (rSurface.eSurfaceType == StagedSurfaceType::Sphere)
                {
                    sPoint = rSurface.sCenter + rSurface.dRadius * (dRadialCoefficient * sRadial + dAxisCoefficient * sAxis);
                }
                else
                {
                    sPoint = rSurface.sOrigin + (rSurface.dMajorRadius + rSurface.dMinorRadius * dRadialCoefficient) * sRadial +
                             rSurface.dMinorRadius * dAxisCoefficient * sAxis;
                }
            }
            rNurb.vNurbControlVertices.push_back(sPoint);
            rNurb.vNurbWeights.push_back(dUWeight * dVWeight);
        }
    }
    return true;
}

bool EvaluateArcScalarAtAngle(const RationalQuadraticArc& rArc, const pxr::VtArray<double>& rControlValues, double dAngle, double& rValue)
{
    if (rArc.vAngles.size() < 3 || rArc.vAngles.size() != rArc.vWeights.size() || rArc.vAngles.size() != rControlValues.size() ||
        dAngle < rArc.vAngles.front() - 1.0e-8 || dAngle > rArc.vAngles.back() + 1.0e-8)
    {
        return false;
    }

    const size_t uiSegmentCount = (rArc.vAngles.size() - 1) / 2;
    const double dTotalSpan = rArc.vAngles.back() - rArc.vAngles.front();
    const double dSegmentSpan = dTotalSpan / static_cast<double>(uiSegmentCount);
    size_t uiSegment = static_cast<size_t>(std::max(0.0, std::floor((dAngle - rArc.vAngles.front()) / dSegmentSpan)));
    uiSegment = std::min(uiSegment, uiSegmentCount - 1);
    const size_t i0 = 2 * uiSegment;
    const size_t i1 = i0 + 1;
    const size_t i2 = i0 + 2;
    const double dA0 = rArc.vAngles[i0];
    const double dA1 = rArc.vAngles[i1];
    const double dA2 = rArc.vAngles[i2];
    const double dW = rArc.vWeights[i1];

    const auto fAngleAt = [&](double q)
    {
        const double b0 = (1.0 - q) * (1.0 - q);
        const double b1 = 2.0 * q * (1.0 - q);
        const double b2 = q * q;
        const double dDenom = b0 + b1 * dW + b2;
        const double x = (b0 * std::cos(dA0) + b1 * std::cos(dA1) + b2 * std::cos(dA2)) / dDenom;
        const double y = (b0 * std::sin(dA0) + b1 * std::sin(dA1) + b2 * std::sin(dA2)) / dDenom;
        return UnwrapAngleNear(std::atan2(y, x), dA0 + q * (dA2 - dA0));
    };

    double dLo = 0.0;
    double dHi = 1.0;
    for (int ii = 0; ii < 60; ++ii)
    {
        const double dMid = 0.5 * (dLo + dHi);
        if (fAngleAt(dMid) < dAngle)
        {
            dLo = dMid;
        }
        else
        {
            dHi = dMid;
        }
    }
    const double q = 0.5 * (dLo + dHi);
    const double b0 = (1.0 - q) * (1.0 - q);
    const double b1 = 2.0 * q * (1.0 - q);
    const double b2 = q * q;
    const double dDenom = b0 + b1 * dW + b2;
    rValue = (b0 * rControlValues[i0] + b1 * dW * rControlValues[i1] + b2 * rControlValues[i2]) / dDenom;
    return std::isfinite(rValue);
}

bool BuildShearedCylinderPatch(
    const StagedSurfaceData& rCylinder,
    const pxr::GfRange1d& rUDomain,
    const pxr::GfRange1d& rVDomain,
    double dAxialStart,
    double dAxialEnd,
    StagedSurfaceData& rNurb,
    RationalQuadraticArc& rUArc,
    pxr::VtArray<double>& rAxialControls
)
{
    if (rCylinder.eSurfaceType != StagedSurfaceType::Cylinder || !IsFiniteNonEmptyRange(rUDomain) || !IsFiniteNonEmptyRange(rVDomain) ||
        !BuildRationalQuadraticArc(rUDomain.GetMin(), rUDomain.GetMax(), rUArc))
    {
        return false;
    }

    pxr::GfVec3d sAxis = rCylinder.sAxis;
    pxr::GfVec3d sRef = rCylinder.sRefDirection;
    if (sAxis.Normalize() == 0.0 || sRef.Normalize() == 0.0)
    {
        return false;
    }
    pxr::GfVec3d sSide = pxr::GfCross(sAxis, sRef);
    if (sSide.Normalize() == 0.0)
    {
        return false;
    }

    rAxialControls.clear();
    rAxialControls.reserve(rUArc.vAngles.size());
    for (size_t iu = 0; iu < rUArc.vAngles.size(); ++iu)
    {
        const double f = static_cast<double>(iu) / static_cast<double>(rUArc.vAngles.size() - 1);
        rAxialControls.push_back(dAxialStart + f * (dAxialEnd - dAxialStart));
    }

    rNurb = StagedSurfaceData{};
    rNurb.eSurfaceType = StagedSurfaceType::BSplineSurface;
    rNurb.uiNurbUOrder = 3;
    rNurb.uiNurbVOrder = 2;
    rNurb.uiNurbUVertexCount = static_cast<uint32_t>(rUArc.vAngles.size());
    rNurb.uiNurbVVertexCount = 2;
    rNurb.vNurbUKnots = rUArc.vKnots;
    rNurb.vNurbVKnots = { rVDomain.GetMin(), rVDomain.GetMin(), rVDomain.GetMax(), rVDomain.GetMax() };
    rNurb.vNurbControlVertices.reserve(rUArc.vAngles.size() * 2);
    rNurb.vNurbWeights.reserve(rUArc.vAngles.size() * 2);
    for (size_t iu = 0; iu < rUArc.vAngles.size(); ++iu)
    {
        const pxr::GfVec3d sRadial = rUArc.vScales[iu] * (std::cos(rUArc.vAngles[iu]) * sRef + std::sin(rUArc.vAngles[iu]) * sSide);
        for (int iv = 0; iv < 2; ++iv)
        {
            const double dV = (iv == 0) ? rVDomain.GetMin() : rVDomain.GetMax();
            rNurb.vNurbControlVertices.push_back(rCylinder.sOrigin + (rAxialControls[iu] + dV) * sAxis + rCylinder.dRadius * sRadial);
            rNurb.vNurbWeights.push_back(rUArc.vWeights[iu]);
        }
    }
    return true;
}

bool BuildCircleArcCurve(
    const pxr::GfVec3d& rCenter,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dRadius,
    double dAngle,
    StagedCurveData& rCurve,
    pxr::GfRange1d& rInterval
)
{
    if (dRadius <= 0.0 || dAngle <= 1.0e-12)
    {
        return false;
    }
    pxr::GfVec3d sAxis = rAxis;
    pxr::GfVec3d sRefDirection = rRefDirection;
    if (sAxis.Normalize() == 0.0 || sRefDirection.Normalize() == 0.0)
    {
        return false;
    }

    rCurve = StagedCurveData{};
    rCurve.eCurveType = StagedCurveType::Circle;
    rCurve.sCenter = rCenter;
    rCurve.sAxis = sAxis;
    rCurve.sRefDirection = sRefDirection;
    rCurve.dRadius = dRadius;
    rInterval = pxr::GfRange1d(0.0, dAngle);
    return true;
}

bool BuildTorusIsoArc(
    const StagedSurfaceData& rSurface,
    bool bAlongU,
    double dFixedParam,
    double dStartParam,
    double dEndParam,
    StagedCurveData& rCurve,
    pxr::GfRange1d& rInterval
)
{
    if (rSurface.eSurfaceType != StagedSurfaceType::Torus || rSurface.dMajorRadius <= 0.0 || rSurface.dMinorRadius <= 0.0)
    {
        return false;
    }
    const double dAngle = dEndParam - dStartParam;
    if (dAngle <= 1.0e-12)
    {
        return false;
    }

    if (bAlongU)
    {
        const pxr::GfVec3d sRef = SurfaceRadialAtU(rSurface, dStartParam);
        const double dRadius = rSurface.dMajorRadius + rSurface.dMinorRadius * std::cos(dFixedParam);
        const pxr::GfVec3d sCenter = rSurface.sOrigin + rSurface.dMinorRadius * std::sin(dFixedParam) * rSurface.sAxis;
        return BuildCircleArcCurve(sCenter, rSurface.sAxis, sRef, dRadius, dAngle, rCurve, rInterval);
    }

    const pxr::GfVec3d sRadial = SurfaceRadialAtU(rSurface, dFixedParam);
    const pxr::GfVec3d sCenter = rSurface.sOrigin + rSurface.dMajorRadius * sRadial;
    const pxr::GfVec3d sAxis = pxr::GfCross(sRadial, rSurface.sAxis);
    const pxr::GfVec3d sRef = std::cos(dStartParam) * sRadial + std::sin(dStartParam) * rSurface.sAxis;
    return BuildCircleArcCurve(sCenter, sAxis, sRef, rSurface.dMinorRadius, dAngle, rCurve, rInterval);
}

bool BuildIsoSeamCurve(const StagedSurfaceData& rSurface, double dU, double dStartV, double dEndV, StagedCurveData& rCurve, pxr::GfRange1d& rInterval)
{
    rCurve = StagedCurveData{};
    const pxr::GfVec3d sStart = SurfacePointAtParam(rSurface, dU, dStartV);
    const pxr::GfVec3d sEnd = SurfacePointAtParam(rSurface, dU, dEndV);
    if ((sEnd - sStart).GetLength() <= 1.0e-12)
    {
        return false;
    }

    if (rSurface.eSurfaceType == StagedSurfaceType::Sphere)
    {
        pxr::GfVec3d sStartVec = sStart - rSurface.sCenter;
        pxr::GfVec3d sEndVec = sEnd - rSurface.sCenter;
        if (sStartVec.Normalize() == 0.0 || sEndVec.Normalize() == 0.0)
        {
            return false;
        }
        pxr::GfVec3d sCurveAxis = pxr::GfCross(sStartVec, sEndVec);
        if (sCurveAxis.Normalize() == 0.0)
        {
            sCurveAxis = pxr::GfCross(SurfaceRadialAtU(rSurface, dU), rSurface.sAxis);
            if (sCurveAxis.Normalize() == 0.0)
            {
                return false;
            }
        }
        const double dAngle = std::acos(ClampUnit(pxr::GfDot(sStartVec, sEndVec)));
        if (dAngle <= 1.0e-12)
        {
            return false;
        }
        rCurve.eCurveType = StagedCurveType::Circle;
        rCurve.sCenter = rSurface.sCenter;
        rCurve.sAxis = sCurveAxis;
        rCurve.sRefDirection = sStartVec;
        rCurve.dRadius = rSurface.dRadius;
        rInterval = pxr::GfRange1d(0.0, dAngle);
        return true;
    }

    pxr::GfVec3d sDir = sEnd - sStart;
    const double dLen = sDir.Normalize();
    if (dLen <= 1.0e-12)
    {
        return false;
    }
    rCurve.eCurveType = StagedCurveType::Line;
    rCurve.sOrigin = sStart;
    rCurve.sDirection = sDir;
    rInterval = pxr::GfRange1d(0.0, dLen);
    return true;
}

bool IsFullPeriodInterval(double dFirst, double dLast)
{
    return std::abs(std::abs(dLast - dFirst) - 2.0 * M_PI) <= 1.0e-5;
}

StagedOrientType OrientFromSame(bool bSame)
{
    return bSame ? StagedOrientType::Same : StagedOrientType::Opposite;
}

StagedRadialEntryType EntryFromSame(bool bSame)
{
    return bSame ? StagedRadialEntryType::BottomSideEntry : StagedRadialEntryType::TopSideEntry;
}

bool IsValidNurbsSurfaceGrid(const StagedSurfaceData& rSurface)
{
    const uint32_t uCount = rSurface.uiNurbUVertexCount;
    const uint32_t vCount = rSurface.uiNurbVVertexCount;
    return rSurface.eSurfaceType == StagedSurfaceType::BSplineSurface && uCount > 1 && vCount > 1 &&
           rSurface.vNurbControlVertices.size() == static_cast<size_t>(uCount) * static_cast<size_t>(vCount) && !rSurface.vNurbUKnots.empty() &&
           !rSurface.vNurbVKnots.empty();
}

pxr::GfVec3d NurbsPointAt(const StagedSurfaceData& rSurface, uint32_t u, uint32_t v)
{
    return rSurface.vNurbControlVertices[static_cast<size_t>(u) * rSurface.uiNurbVVertexCount + v];
}

bool NurbsClosedInU(const StagedSurfaceData& rSurface)
{
    const uint32_t uCount = rSurface.uiNurbUVertexCount;
    const uint32_t vCount = rSurface.uiNurbVVertexCount;
    for (uint32_t v = 0; v < vCount; ++v)
    {
        if (!PointsClose(NurbsPointAt(rSurface, 0, v), NurbsPointAt(rSurface, uCount - 1, v)))
        {
            return false;
        }
    }
    return true;
}

bool NurbsClosedInV(const StagedSurfaceData& rSurface)
{
    const uint32_t uCount = rSurface.uiNurbUVertexCount;
    const uint32_t vCount = rSurface.uiNurbVVertexCount;
    for (uint32_t u = 0; u < uCount; ++u)
    {
        if (!PointsClose(NurbsPointAt(rSurface, u, 0), NurbsPointAt(rSurface, u, vCount - 1)))
        {
            return false;
        }
    }
    return true;
}

bool NurbsURowCollapsed(const StagedSurfaceData& rSurface, uint32_t v)
{
    const uint32_t uCount = rSurface.uiNurbUVertexCount;
    const pxr::GfVec3d sFirst = NurbsPointAt(rSurface, 0, v);
    for (uint32_t u = 1; u < uCount; ++u)
    {
        if (!PointsClose(NurbsPointAt(rSurface, u, v), sFirst))
        {
            return false;
        }
    }
    return true;
}

bool NurbsVColumnCollapsed(const StagedSurfaceData& rSurface, uint32_t u)
{
    const uint32_t vCount = rSurface.uiNurbVVertexCount;
    const pxr::GfVec3d sFirst = NurbsPointAt(rSurface, u, 0);
    for (uint32_t v = 1; v < vCount; ++v)
    {
        if (!PointsClose(NurbsPointAt(rSurface, u, v), sFirst))
        {
            return false;
        }
    }
    return true;
}

pxr::VtArray<double> ReversedKnots(const pxr::VtArray<double>& rKnots)
{
    pxr::VtArray<double> sResult;
    if (rKnots.empty())
    {
        return sResult;
    }
    const double dLo = rKnots.front();
    const double dHi = rKnots.back();
    sResult.reserve(rKnots.size());
    for (auto sIt = rKnots.rbegin(); sIt != rKnots.rend(); ++sIt)
    {
        sResult.push_back(dLo + dHi - *sIt);
    }
    return sResult;
}

bool BuildNurbsIsoCurve(
    const StagedSurfaceData& rSurface,
    bool bAlongU,
    uint32_t uiFixedIndex,
    bool bReverse,
    StagedCurveData& rCurve,
    pxr::GfRange1d& rInterval
)
{
    rCurve = StagedCurveData{};
    if (!IsValidNurbsSurfaceGrid(rSurface))
    {
        return false;
    }

    const uint32_t uCount = rSurface.uiNurbUVertexCount;
    const uint32_t vCount = rSurface.uiNurbVVertexCount;
    const uint32_t uiCount = bAlongU ? uCount : vCount;
    if ((bAlongU && uiFixedIndex >= vCount) || (!bAlongU && uiFixedIndex >= uCount))
    {
        return false;
    }

    rCurve.eCurveType = StagedCurveType::BSplineCurve;
    rCurve.uiNurbVertexCount = uiCount;
    rCurve.uiNurbOrder = bAlongU ? rSurface.uiNurbUOrder : rSurface.uiNurbVOrder;
    rCurve.vNurbControlVertices.reserve(uiCount);
    rCurve.vNurbWeights.reserve(uiCount);
    for (uint32_t ii = 0; ii < uiCount; ++ii)
    {
        const uint32_t jj = bReverse ? (uiCount - 1 - ii) : ii;
        const uint32_t u = bAlongU ? jj : uiFixedIndex;
        const uint32_t v = bAlongU ? uiFixedIndex : jj;
        const size_t uiGridIndex = static_cast<size_t>(u) * vCount + v;
        rCurve.vNurbControlVertices.push_back(rSurface.vNurbControlVertices[uiGridIndex]);
        if (!rSurface.vNurbWeights.empty())
        {
            rCurve.vNurbWeights.push_back(rSurface.vNurbWeights[uiGridIndex]);
        }
    }
    if (rCurve.vNurbWeights.empty())
    {
        rCurve.vNurbWeights.assign(uiCount, 1.0);
    }

    const pxr::VtArray<double>& rKnots = bAlongU ? rSurface.vNurbUKnots : rSurface.vNurbVKnots;
    rCurve.vNurbKnots = bReverse ? ReversedKnots(rKnots) : rKnots;
    if (rCurve.vNurbKnots.empty())
    {
        return false;
    }
    rInterval = pxr::GfRange1d(rCurve.vNurbKnots.front(), rCurve.vNurbKnots.back());
    return rInterval.GetMax() > rInterval.GetMin();
}

bool EvalStagedBSplineCurve(const StagedCurveData& rCurve, double t, pxr::GfVec3d& rOut)
{
    if (rCurve.eCurveType != StagedCurveType::BSplineCurve || rCurve.uiNurbOrder < 2 || rCurve.uiNurbVertexCount == 0 ||
        rCurve.vNurbControlVertices.size() != rCurve.uiNurbVertexCount || rCurve.vNurbKnots.size() != rCurve.uiNurbVertexCount + rCurve.uiNurbOrder)
    {
        return false;
    }

    const int p = static_cast<int>(rCurve.uiNurbOrder) - 1;
    const int n = static_cast<int>(rCurve.uiNurbVertexCount);
    const pxr::VtArray<double>& U = rCurve.vNurbKnots;
    if (t < U[static_cast<size_t>(p)])
    {
        t = U[static_cast<size_t>(p)];
    }
    if (t > U[static_cast<size_t>(n)])
    {
        t = U[static_cast<size_t>(n)];
    }

    int k = p;
    if (t >= U[static_cast<size_t>(n)])
    {
        k = n - 1;
    }
    else
    {
        for (int ii = p; ii < n; ++ii)
        {
            if (t >= U[static_cast<size_t>(ii)] && t < U[static_cast<size_t>(ii + 1)])
            {
                k = ii;
                break;
            }
        }
    }

    std::vector<pxr::GfVec3d> vD(static_cast<size_t>(p + 1));
    std::vector<double> vW(static_cast<size_t>(p + 1), 1.0);
    for (int j = 0; j <= p; ++j)
    {
        const int idx = k - p + j;
        const double w = (rCurve.vNurbWeights.size() == rCurve.uiNurbVertexCount) ? rCurve.vNurbWeights[static_cast<size_t>(idx)] : 1.0;
        vD[static_cast<size_t>(j)] = rCurve.vNurbControlVertices[static_cast<size_t>(idx)] * w;
        vW[static_cast<size_t>(j)] = w;
    }
    for (int r = 1; r <= p; ++r)
    {
        for (int j = p; j >= r; --j)
        {
            const int idx = k - p + j;
            const double dDen = U[static_cast<size_t>(idx + p - r + 1)] - U[static_cast<size_t>(idx)];
            const double a = (std::abs(dDen) > 1.0e-12) ? (t - U[static_cast<size_t>(idx)]) / dDen : 0.0;
            vD[static_cast<size_t>(j)] = (1.0 - a) * vD[static_cast<size_t>(j - 1)] + a * vD[static_cast<size_t>(j)];
            vW[static_cast<size_t>(j)] = (1.0 - a) * vW[static_cast<size_t>(j - 1)] + a * vW[static_cast<size_t>(j)];
        }
    }

    if (std::abs(vW[static_cast<size_t>(p)]) <= 1.0e-12)
    {
        return false;
    }
    rOut = vD[static_cast<size_t>(p)] / vW[static_cast<size_t>(p)];
    return true;
}

bool SourceEdgeMatchesStagedCurve(
    const SCurve3d& rSourceCurve,
    const pxr::GfMatrix4d& rSourceXf,
    double dSourceFirst,
    double dSourceLast,
    const StagedCurveData& rTargetCurve,
    const pxr::GfRange1d& rTargetInterval,
    bool bReverseTarget
)
{
    constexpr int kSamples = 8;
    const double dTol = 1.0e-5;
    for (int ii = 0; ii <= kSamples; ++ii)
    {
        const double f = static_cast<double>(ii) / static_cast<double>(kSamples);
        const pxr::GfVec3d sSrc = EvalCurve3dPoint(rSourceCurve, rSourceXf, dSourceFirst + (dSourceLast - dSourceFirst) * f);
        const double dTargetF = bReverseTarget ? (1.0 - f) : f;
        pxr::GfVec3d sTgt;
        if (!EvalStagedBSplineCurve(rTargetCurve, rTargetInterval.GetMin() + (rTargetInterval.GetMax() - rTargetInterval.GetMin()) * dTargetF, sTgt))
        {
            return false;
        }
        if (!PointsClose(sSrc, sTgt, dTol))
        {
            return false;
        }
    }
    return true;
}

bool CanUseImpliedNaturalBoundaryWithoutPCurves(const StagedSurfaceData& rSurface)
{
    switch (rSurface.eSurfaceType)
    {
        case StagedSurfaceType::Sphere:
        case StagedSurfaceType::Torus:
            return true;
        case StagedSurfaceType::BSplineSurface:
            return IsValidNurbsSurfaceGrid(rSurface) && (NurbsClosedInU(rSurface) || NurbsClosedInV(rSurface));
        default:
            return false;
    }
}

// A natural closed-surface wire contains only pole singularities and paired seam uses. A partial
// face can also span a full UV period (and, on a sphere, both poles), so the UV box alone is not
// enough to distinguish it from the whole surface.
std::unordered_map<int, uint32_t> CountNonDegenerateBoundaryEdgeUses(const OcctFileSource& rSource, const OcctFileSource::Face& rFace)
{
    std::unordered_map<int, uint32_t> mUseCounts;
    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            if (rSource.ResolveEdge(rCoedge.iEdgeStorageIndex).bDegenerated)
            {
                continue;
            }
            ++mUseCounts[rCoedge.iEdgeStorageIndex];
        }
    }
    return mUseCounts;
}

bool HasOnlyNaturalSeamBoundary(const OcctFileSource& rSource, const OcctFileSource::Face& rFace)
{
    const std::unordered_map<int, uint32_t> mUseCounts = CountNonDegenerateBoundaryEdgeUses(rSource, rFace);
    return !mUseCounts.empty() && std::all_of(
                                      mUseCounts.begin(),
                                      mUseCounts.end(),
                                      [](const auto& rEntry)
                                      {
                                          return rEntry.second == 2;
                                      }
                                  );
}

bool HasExplicitSeamBoundary(const OcctFileSource& rSource, const OcctFileSource::Face& rFace)
{
    const std::unordered_map<int, uint32_t> mUseCounts = CountNonDegenerateBoundaryEdgeUses(rSource, rFace);
    return std::any_of(
        mUseCounts.begin(),
        mUseCounts.end(),
        [](const auto& rEntry)
        {
            return rEntry.second >= 2;
        }
    );
}

bool HasCompleteBoundaryPCurves(const OcctFileSource& rSource, const OcctFileSource::Face& rFace)
{
    bool bFoundCoedge = false;
    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            if (rSource.ResolveEdge(rCoedge.iEdgeStorageIndex).bDegenerated)
            {
                continue;
            }
            bFoundCoedge = true;
            OcctFileSource::CoedgePCurve sPCurve;
            if (!rSource.ResolveCoedgePCurve(rCoedge, rFace.iSurfaceIndex, sPCurve) || !sPCurve.pCurve2d)
            {
                return false;
            }
        }
    }
    return bFoundCoedge;
}

enum RectangularBoundarySide : uint8_t
{
    RectangularBoundaryUMin = 1u << 0,
    RectangularBoundaryUMax = 1u << 1,
    RectangularBoundaryVMin = 1u << 2,
    RectangularBoundaryVMax = 1u << 3,
    RectangularBoundaryAll = RectangularBoundaryUMin | RectangularBoundaryUMax | RectangularBoundaryVMin | RectangularBoundaryVMax,
};

bool CoedgeMatchesFaceRangeSide(
    const OcctFileSource& rSource,
    const OcctFileSource::Coedge& rCoedge,
    int iSurface,
    const pxr::GfRange2d& rFaceDomain,
    const pxr::GfVec2d& rUvScale,
    const pxr::GfVec2d& rUvAdd,
    uint8_t& rSide
)
{
    OcctFileSource::CoedgePCurve sPCurve;
    if (!rSource.ResolveCoedgePCurve(rCoedge, iSurface, sPCurve) || !sPCurve.pCurve2d)
    {
        return false;
    }

    StagedCurveData2d sNurb;
    if (!ConvertCurve2dToNurb(*sPCurve.pCurve2d, pxr::GfRange1d(sPCurve.dFirst, sPCurve.dLast), rUvScale, rUvAdd, sNurb) ||
        sNurb.vNurbControlVertices.empty())
    {
        return false;
    }

    pxr::GfRange2d sRange;
    for (const pxr::GfVec2d& rPoint : sNurb.vNurbControlVertices)
    {
        if (!std::isfinite(rPoint[0]) || !std::isfinite(rPoint[1]))
        {
            return false;
        }
        sRange.UnionWith(rPoint);
    }
    if (sRange.IsEmpty())
    {
        return false;
    }

    pxr::GfVec2d sStart;
    pxr::GfVec2d sEnd;
    if (!EvaluateUvNurbEndpoint(sNurb, false, sStart) || !EvaluateUvNurbEndpoint(sNurb, true, sEnd))
    {
        return false;
    }

    const pxr::GfVec2d sDomainMin = rFaceDomain.GetMin();
    const pxr::GfVec2d sDomainMax = rFaceDomain.GetMax();
    const pxr::GfVec2d sSize = sRange.GetSize();
    const double dScale = std::max({ 1.0, std::abs(sDomainMin[0]), std::abs(sDomainMax[0]), std::abs(sDomainMin[1]), std::abs(sDomainMax[1]) });
    const double dTol = 1.0e-6 * dScale;
    const bool bConstantU = sSize[0] <= dTol;
    const bool bConstantV = sSize[1] <= dTol;
    if (bConstantU == bConstantV)
    {
        return false;
    }

    for (const pxr::GfVec2d& rPoint : sNurb.vNurbControlVertices)
    {
        if (rPoint[0] < sDomainMin[0] - dTol || rPoint[0] > sDomainMax[0] + dTol || rPoint[1] < sDomainMin[1] - dTol ||
            rPoint[1] > sDomainMax[1] + dTol)
        {
            return false;
        }
    }

    const auto fNear = [&](double dA, double dB)
    {
        return std::abs(dA - dB) <= dTol;
    };
    const auto fSpans = [&](double dStart, double dEnd, double dMin, double dMax)
    {
        return (fNear(dStart, dMin) && fNear(dEnd, dMax)) || (fNear(dStart, dMax) && fNear(dEnd, dMin));
    };

    if (bConstantU)
    {
        const bool bAtMin = fNear(sRange.GetMin()[0], sDomainMin[0]) && fNear(sRange.GetMax()[0], sDomainMin[0]);
        const bool bAtMax = fNear(sRange.GetMin()[0], sDomainMax[0]) && fNear(sRange.GetMax()[0], sDomainMax[0]);
        if (bAtMin == bAtMax || !fSpans(sStart[1], sEnd[1], sDomainMin[1], sDomainMax[1]))
        {
            return false;
        }
        rSide = bAtMin ? RectangularBoundaryUMin : RectangularBoundaryUMax;
        return true;
    }

    const bool bAtMin = fNear(sRange.GetMin()[1], sDomainMin[1]) && fNear(sRange.GetMax()[1], sDomainMin[1]);
    const bool bAtMax = fNear(sRange.GetMin()[1], sDomainMax[1]) && fNear(sRange.GetMax()[1], sDomainMax[1]);
    if (bAtMin == bAtMax || !fSpans(sStart[0], sEnd[0], sDomainMin[0], sDomainMax[0]))
    {
        return false;
    }
    rSide = bAtMin ? RectangularBoundaryVMin : RectangularBoundaryVMax;
    return true;
}

// True when the face's only loop is the four-sided parameter rectangle described by face:range.
// The rectangle may be inside the surface's natural domain; each pcurve must still coincide with one
// whole side of the authored face domain. Implied and seam-only natural boundaries are known rectangles
// without consulting source pcurves. Planar faces remain general because their outer-loop selection and
// domain come from projected 3D topology, which may be more reliable than the serialized pcurves.
bool FaceHasRectangularTrim(
    const OcctFileSource& rSource,
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurfaceData,
    uint32_t uiStagedLoopCount,
    bool bImpliedNaturalBoundary,
    bool bSourcePCurvesComparable,
    const pxr::GfRange2d& rFaceDomain,
    const pxr::GfVec2d& rUvScale,
    const pxr::GfVec2d& rUvAdd
)
{
    if (uiStagedLoopCount != 1)
    {
        return false;
    }
    if (bImpliedNaturalBoundary)
    {
        return true;
    }
    if (rSurfaceData.eSurfaceType == StagedSurfaceType::Plane)
    {
        return false;
    }
    if (HasOnlyNaturalSeamBoundary(rSource, rFace))
    {
        return true;
    }
    if (!bSourcePCurvesComparable || !IsFiniteNonEmptyRange(rFaceDomain))
    {
        return false;
    }

    size_t uiCoedgeCount = 0;
    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        uiCoedgeCount += rLoop.vCoedges.size();
    }
    if (uiCoedgeCount != 4)
    {
        return false;
    }

    uint8_t uiBoundarySides = 0;
    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            uint8_t uiSide = 0;
            if (!CoedgeMatchesFaceRangeSide(rSource, rCoedge, rFace.iSurfaceIndex, rFaceDomain, rUvScale, rUvAdd, uiSide) ||
                (uiBoundarySides & uiSide) != 0)
            {
                return false;
            }
            uiBoundarySides |= uiSide;
        }
    }
    return uiBoundarySides == RectangularBoundaryAll;
}

// Natural parameter domain of a closed/bounded surface, used for a natural-restriction face (one with
// no boundary wires). Returns an empty range for surfaces whose natural domain is unbounded (cylinder/
// cone extend infinitely along the axis, so a loop-less face cannot be bounded) or unsupported.
pxr::GfRange2d ComputeNaturalDomain(const StagedSurfaceData& rSurface)
{
    const double dTwoPi = 2.0 * M_PI;
    switch (rSurface.eSurfaceType)
    {
        case StagedSurfaceType::Sphere:
            // U = longitude [0, 2*pi]; V = latitude [-pi/2, pi/2].
            return pxr::GfRange2d(pxr::GfVec2d(0.0, -0.5 * M_PI), pxr::GfVec2d(dTwoPi, 0.5 * M_PI));
        case StagedSurfaceType::Torus:
            // U = major angle [0, 2*pi]; V = minor angle [0, 2*pi].
            return pxr::GfRange2d(pxr::GfVec2d(0.0, 0.0), pxr::GfVec2d(dTwoPi, dTwoPi));
        case StagedSurfaceType::BSplineSurface:
            if (!rSurface.vNurbUKnots.empty() && !rSurface.vNurbVKnots.empty())
            {
                return pxr::GfRange2d(
                    pxr::GfVec2d(rSurface.vNurbUKnots.front(), rSurface.vNurbVKnots.front()),
                    pxr::GfVec2d(rSurface.vNurbUKnots.back(), rSurface.vNurbVKnots.back())
                );
            }
            return pxr::GfRange2d();
        default:
            // Plane / Cylinder / Cone: unbounded natural domain -> cannot bound a loop-less face.
            return pxr::GfRange2d();
    }
}

pxr::GfRange2d ComputeFullPeriodAxialDomainFrom3dBoundary(
    const OcctFileSource& rSource,
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurface
)
{
    if (rSurface.eSurfaceType != StagedSurfaceType::Cylinder && rSurface.eSurfaceType != StagedSurfaceType::Cone)
    {
        return pxr::GfRange2d();
    }

    pxr::GfVec3d sAxis = rSurface.sAxis;
    if (sAxis.Normalize() == 0.0)
    {
        return pxr::GfRange2d();
    }

    pxr::GfRange2d sDomain;
    const auto fAccumulate = [&](const pxr::GfVec3d& rWorld)
    {
        const double dV = pxr::GfDot(rWorld - rSurface.sOrigin, sAxis);
        sDomain.UnionWith(pxr::GfVec2d(0.0, dV));
        sDomain.UnionWith(pxr::GfVec2d(2.0 * M_PI, dV));
    };

    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            const OcctFileSource::EdgeGeometry sGeom = rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
            if (sGeom.pCurve3d)
            {
                const pxr::GfMatrix4d sCurveXf = ResolveLocation(rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
                constexpr int kSamples = 24;
                for (int ii = 0; ii <= kSamples; ++ii)
                {
                    const double t = sGeom.dFirst + (sGeom.dLast - sGeom.dFirst) * (static_cast<double>(ii) / static_cast<double>(kSamples));
                    fAccumulate(EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, t));
                }
            }

            const int aiVerts[2] = { sGeom.iStartVertexStorageIndex, sGeom.iEndVertexStorageIndex };
            const int aiLocs[2] = { sGeom.iStartVertexLocation, sGeom.iEndVertexLocation };
            for (int ii = 0; ii < 2; ++ii)
            {
                if (aiVerts[ii] == 0)
                {
                    continue;
                }
                const pxr::GfMatrix4d sVtxLoc = ResolveLocation(rSource.Model(), aiLocs[ii]) * rCoedge.mEdgeLoc;
                fAccumulate(sVtxLoc.Transform(rSource.VertexPoint(aiVerts[ii])));
            }
        }
    }

    if (!sDomain.IsEmpty() && std::abs(sDomain.GetMax()[1] - sDomain.GetMin()[1]) < 1.0e-12)
    {
        const double dV = sDomain.GetMin()[1];
        sDomain = pxr::GfRange2d(pxr::GfVec2d(0.0, dV - 1.0e-9), pxr::GfVec2d(2.0 * M_PI, dV + 1.0e-9));
    }
    return sDomain;
}

pxr::GfRange2d ComputePlaneLoopDomainFrom3dBoundary(
    const OcctFileSource& rSource,
    const OcctFileSource::Loop& rLoop,
    const StagedSurfaceData& rSurface
)
{
    if (rSurface.eSurfaceType != StagedSurfaceType::Plane)
    {
        return pxr::GfRange2d();
    }

    pxr::GfVec3d sAxis = rSurface.sAxis;
    pxr::GfVec3d sUDir = rSurface.sRefDirection;
    if (sAxis.Normalize() == 0.0 || sUDir.Normalize() == 0.0)
    {
        return pxr::GfRange2d();
    }
    pxr::GfVec3d sVDir = pxr::GfCross(sAxis, sUDir);
    if (sVDir.Normalize() == 0.0)
    {
        return pxr::GfRange2d();
    }

    pxr::GfRange2d sDomain;
    const auto fProject = [&](const pxr::GfVec3d& rWorld)
    {
        const pxr::GfVec3d sDelta = rWorld - rSurface.sOrigin;
        sDomain.UnionWith(pxr::GfVec2d(pxr::GfDot(sDelta, sUDir), pxr::GfDot(sDelta, sVDir)));
    };

    for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
    {
        const OcctFileSource::EdgeGeometry sGeom = rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
        if (sGeom.pCurve3d)
        {
            const pxr::GfMatrix4d sCurveXf = ResolveLocation(rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
            constexpr int kSamples = 24;
            for (int ii = 0; ii <= kSamples; ++ii)
            {
                const double t = sGeom.dFirst + (sGeom.dLast - sGeom.dFirst) * (static_cast<double>(ii) / static_cast<double>(kSamples));
                fProject(EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, t));
            }
        }

        const int aiVerts[2] = { sGeom.iStartVertexStorageIndex, sGeom.iEndVertexStorageIndex };
        const int aiLocs[2] = { sGeom.iStartVertexLocation, sGeom.iEndVertexLocation };
        for (int ii = 0; ii < 2; ++ii)
        {
            if (aiVerts[ii] == 0)
            {
                continue;
            }
            const pxr::GfMatrix4d sVertexXf = ResolveLocation(rSource.Model(), aiLocs[ii]) * rCoedge.mEdgeLoc;
            fProject(sVertexXf.Transform(rSource.VertexPoint(aiVerts[ii])));
        }
    }

    return sDomain;
}

enum class PeriodicParameterAxis
{
    U,
    V,
};

bool SurfacePeriodicParameterAtPoint(const StagedSurfaceData& rSurface, const pxr::GfVec3d& rPoint, PeriodicParameterAxis eAxis, double& rParameter)
{
    if (eAxis == PeriodicParameterAxis::V && rSurface.eSurfaceType != StagedSurfaceType::Torus)
    {
        return false;
    }
    if (eAxis == PeriodicParameterAxis::U && rSurface.eSurfaceType != StagedSurfaceType::Cylinder &&
        rSurface.eSurfaceType != StagedSurfaceType::Cone && rSurface.eSurfaceType != StagedSurfaceType::Sphere &&
        rSurface.eSurfaceType != StagedSurfaceType::Torus)
    {
        return false;
    }

    // Longitude is undefined at a sphere pole. Treat it as a break in parameter continuity rather
    // than allowing atan2(0, 0) or floating-point noise to expand a partial U range to a full period.
    if (eAxis == PeriodicParameterAxis::U && rSurface.eSurfaceType == StagedSurfaceType::Sphere)
    {
        pxr::GfVec3d sAxis = rSurface.sAxis;
        if (sAxis.Normalize() == 0.0)
        {
            return false;
        }
        const pxr::GfVec3d sDelta = rPoint - rSurface.sCenter;
        const pxr::GfVec3d sRadial = sDelta - pxr::GfDot(sDelta, sAxis) * sAxis;
        if (sRadial.GetLength() <= 1.0e-10 * std::max(1.0, rSurface.dRadius))
        {
            return false;
        }
    }

    double dU = 0.0;
    double dV = 0.0;
    if (!SurfaceParamAtPoint(rSurface, rPoint, dU, dV))
    {
        return false;
    }
    rParameter = (eAxis == PeriodicParameterAxis::U) ? dU : dV;
    return true;
}

pxr::GfRange1d ComputeTightPeriodicDomainFrom3dBoundary(
    const OcctFileSource& rSource,
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurface,
    PeriodicParameterAxis eAxis
)
{
    const double dTwoPi = 2.0 * M_PI;
    pxr::GfRange1d sDomain;
    std::vector<size_t> vLoopOrder;
    vLoopOrder.reserve(rFace.vLoops.size());
    if (rFace.uiOuterLoopIndex < rFace.vLoops.size())
    {
        vLoopOrder.push_back(rFace.uiOuterLoopIndex);
    }
    for (size_t ii = 0; ii < rFace.vLoops.size(); ++ii)
    {
        if (ii != rFace.uiOuterLoopIndex)
        {
            vLoopOrder.push_back(ii);
        }
    }

    for (size_t uiLoopIndex : vLoopOrder)
    {
        const OcctFileSource::Loop& rLoop = rFace.vLoops[uiLoopIndex];
        pxr::GfRange1d sLoopDomain;
        bool bHavePrevious = false;
        double dPrevious = 0.0;
        const auto fAccumulate = [&](const pxr::GfVec3d& rWorld)
        {
            double dParameter = 0.0;
            if (!SurfacePeriodicParameterAtPoint(rSurface, rWorld, eAxis, dParameter))
            {
                bHavePrevious = false;
                return;
            }
            if (bHavePrevious)
            {
                dParameter = UnwrapAngleNear(dParameter, dPrevious);
            }
            else if (!sLoopDomain.IsEmpty())
            {
                dParameter = UnwrapAngleNear(dParameter, 0.5 * (sLoopDomain.GetMin() + sLoopDomain.GetMax()));
            }
            sLoopDomain.UnionWith(dParameter);
            dPrevious = dParameter;
            bHavePrevious = true;
        };

        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            const OcctFileSource::EdgeGeometry sGeom = rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
            const bool bSame = rCoedge.bSameOrientationWithLoop;
            const int iStartVertex = bSame ? sGeom.iStartVertexStorageIndex : sGeom.iEndVertexStorageIndex;
            const int iStartLocation = bSame ? sGeom.iStartVertexLocation : sGeom.iEndVertexLocation;
            if (iStartVertex != 0)
            {
                const pxr::GfMatrix4d sVertexXf = ResolveLocation(rSource.Model(), iStartLocation) * rCoedge.mEdgeLoc;
                fAccumulate(sVertexXf.Transform(rSource.VertexPoint(iStartVertex)));
            }

            if (sGeom.pCurve3d)
            {
                const pxr::GfMatrix4d sCurveXf = ResolveLocation(rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
                constexpr int kSamples = 96;
                for (int ii = 0; ii <= kSamples; ++ii)
                {
                    const double dFraction = static_cast<double>(ii) / static_cast<double>(kSamples);
                    const double t = bSame ? (sGeom.dFirst + (sGeom.dLast - sGeom.dFirst) * dFraction) :
                                             (sGeom.dLast + (sGeom.dFirst - sGeom.dLast) * dFraction);
                    fAccumulate(EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, t));
                }
            }

            const int iEndVertex = bSame ? sGeom.iEndVertexStorageIndex : sGeom.iStartVertexStorageIndex;
            const int iEndLocation = bSame ? sGeom.iEndVertexLocation : sGeom.iStartVertexLocation;
            if (iEndVertex != 0)
            {
                const pxr::GfMatrix4d sVertexXf = ResolveLocation(rSource.Model(), iEndLocation) * rCoedge.mEdgeLoc;
                fAccumulate(sVertexXf.Transform(rSource.VertexPoint(iEndVertex)));
            }
        }
        if (!IsFiniteRange(sLoopDomain))
        {
            return pxr::GfRange1d();
        }

        if (sDomain.IsEmpty())
        {
            sDomain = sLoopDomain;
            continue;
        }

        const double dDomainMid = 0.5 * (sDomain.GetMin() + sDomain.GetMax());
        const double dLoopMid = 0.5 * (sLoopDomain.GetMin() + sLoopDomain.GetMax());
        const double dShift = dTwoPi * std::round((dDomainMid - dLoopMid) / dTwoPi);
        sDomain.UnionWith(pxr::GfRange1d(sLoopDomain.GetMin() + dShift, sLoopDomain.GetMax() + dShift));
    }

    return IsFiniteNonEmptyRange(sDomain) ? sDomain : pxr::GfRange1d();
}

pxr::GfRange1d ComputeNonPeriodicDomainFrom3dBoundary(
    const OcctFileSource& rSource,
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurface,
    PeriodicParameterAxis eAxis
)
{
    pxr::GfRange1d sDomain;
    const auto fAccumulate = [&](const pxr::GfVec3d& rWorld)
    {
        double dU = 0.0;
        double dV = 0.0;
        if (SurfaceParamAtPoint(rSurface, rWorld, dU, dV))
        {
            sDomain.UnionWith(eAxis == PeriodicParameterAxis::U ? dU : dV);
        }
    };

    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            const OcctFileSource::EdgeGeometry sGeom = rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
            if (sGeom.pCurve3d)
            {
                const pxr::GfMatrix4d sCurveXf = ResolveLocation(rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
                constexpr int kSamples = 48;
                for (int ii = 0; ii <= kSamples; ++ii)
                {
                    const double t = sGeom.dFirst + (sGeom.dLast - sGeom.dFirst) * (static_cast<double>(ii) / static_cast<double>(kSamples));
                    fAccumulate(EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, t));
                }
            }

            const int aiVertices[2] = { sGeom.iStartVertexStorageIndex, sGeom.iEndVertexStorageIndex };
            const int aiLocations[2] = { sGeom.iStartVertexLocation, sGeom.iEndVertexLocation };
            for (int ii = 0; ii < 2; ++ii)
            {
                if (aiVertices[ii] == 0)
                {
                    continue;
                }
                const pxr::GfMatrix4d sVertexXf = ResolveLocation(rSource.Model(), aiLocations[ii]) * rCoedge.mEdgeLoc;
                fAccumulate(sVertexXf.Transform(rSource.VertexPoint(aiVertices[ii])));
            }
        }
    }
    return IsFiniteNonEmptyRange(sDomain) ? sDomain : pxr::GfRange1d();
}

pxr::GfRange1d ComputeShearedCylinderVDomainFrom3dBoundary(
    const OcctFileSource& rSource,
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rCylinder,
    const pxr::GfRange1d& rTargetUDomain,
    const RationalQuadraticArc& rUArc,
    const pxr::VtArray<double>& rAxialControls
)
{
    pxr::GfRange1d sDomain;
    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        std::vector<pxr::GfVec2d> vParameters;
        bool bHavePrevious = false;
        double dPreviousU = 0.0;
        const auto fAccumulate = [&](const pxr::GfVec3d& rWorld)
        {
            double dU = 0.0;
            double dV = 0.0;
            if (!SurfaceParamAtPoint(rCylinder, rWorld, dU, dV))
            {
                return;
            }
            if (bHavePrevious)
            {
                dU = UnwrapAngleNear(dU, dPreviousU);
            }
            vParameters.push_back(pxr::GfVec2d(dU, dV));
            dPreviousU = dU;
            bHavePrevious = true;
        };

        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            const OcctFileSource::EdgeGeometry sGeom = rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
            const bool bSame = rCoedge.bSameOrientationWithLoop;
            const int iStartVertex = bSame ? sGeom.iStartVertexStorageIndex : sGeom.iEndVertexStorageIndex;
            const int iStartLocation = bSame ? sGeom.iStartVertexLocation : sGeom.iEndVertexLocation;
            if (iStartVertex != 0)
            {
                const pxr::GfMatrix4d sVertexXf = ResolveLocation(rSource.Model(), iStartLocation) * rCoedge.mEdgeLoc;
                fAccumulate(sVertexXf.Transform(rSource.VertexPoint(iStartVertex)));
            }
            if (sGeom.pCurve3d)
            {
                const pxr::GfMatrix4d sCurveXf = ResolveLocation(rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
                constexpr int kSamples = 192;
                for (int ii = 0; ii <= kSamples; ++ii)
                {
                    const double f = static_cast<double>(ii) / static_cast<double>(kSamples);
                    const double t = bSame ? sGeom.dFirst + (sGeom.dLast - sGeom.dFirst) * f : sGeom.dLast + (sGeom.dFirst - sGeom.dLast) * f;
                    fAccumulate(EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, t));
                }
            }
            const int iEndVertex = bSame ? sGeom.iEndVertexStorageIndex : sGeom.iStartVertexStorageIndex;
            const int iEndLocation = bSame ? sGeom.iEndVertexLocation : sGeom.iStartVertexLocation;
            if (iEndVertex != 0)
            {
                const pxr::GfMatrix4d sVertexXf = ResolveLocation(rSource.Model(), iEndLocation) * rCoedge.mEdgeLoc;
                fAccumulate(sVertexXf.Transform(rSource.VertexPoint(iEndVertex)));
            }
        }

        if (vParameters.empty())
        {
            continue;
        }
        pxr::GfRange1d sLoopU;
        for (const pxr::GfVec2d& rParameter : vParameters)
        {
            sLoopU.UnionWith(rParameter[0]);
        }
        const double dLoopMid = 0.5 * (sLoopU.GetMin() + sLoopU.GetMax());
        const double dTargetMid = 0.5 * (rTargetUDomain.GetMin() + rTargetUDomain.GetMax());
        const double dShift = 2.0 * M_PI * std::round((dTargetMid - dLoopMid) / (2.0 * M_PI));
        for (const pxr::GfVec2d& rParameter : vParameters)
        {
            double dAxialBaseline = 0.0;
            if (EvaluateArcScalarAtAngle(rUArc, rAxialControls, rParameter[0] + dShift, dAxialBaseline))
            {
                sDomain.UnionWith(rParameter[1] - dAxialBaseline);
            }
        }
    }

    if (!IsFiniteNonEmptyRange(sDomain))
    {
        return pxr::GfRange1d();
    }
    const double dPad = 1.0e-9 * std::max({ 1.0, std::abs(sDomain.GetMin()), std::abs(sDomain.GetMax()) });
    return pxr::GfRange1d(sDomain.GetMin() - dPad, sDomain.GetMax() + dPad);
}

// True when an explicit face is a full, untrimmed closed self-contained analytic surface (sphere or
// torus) whose loop is the natural parameter-space seam (possibly accompanied by degenerate pole edges).
// Emitting such a face verbatim produces a brep that SMLib's own SmBrep::AssertValid rejects
// (non-closing loop, zero-length and non-SameParameter UV trim curves, bad loop orientation). The driver
// preserves OCCT's real seam edge and lets the generic staging preparation demote pole coedges to vertex
// singularities before topology is emitted.
//
// Restricted to sphere/torus: their natural boundary (seam + pole singularities) is self-contained, so
// demoting the pole edges loses no shared topology. A cylinder/cone's V-extent cap circles are shared
// edges with adjacent planar disk faces and must stay explicit, so they are never diverted here.
bool ShouldSynthesizeNaturalBoundary(
    const OcctFileSource& rSource,
    const OcctFileSource::Face& rFace,
    ESurfaceType eRawType,
    const pxr::GfRange2d& sPCurveBox
)
{
    if (rFace.bNaturalRestriction)
    {
        return false; // already a natural-restriction face: handled by the existing implied path
    }
    if (rFace.vLoops.size() != 1)
    {
        return false; // single outer loop only (no holes); trimmed/holed faces stay explicit
    }
    if (eRawType != ESurfaceType::Sphere && eRawType != ESurfaceType::Torus)
    {
        return false; // only self-contained closed boundaries (no edges shared with other faces)
    }
    // The loop must contain a seam edge: one edge referenced by two coedges of the loop.
    const std::vector<OcctFileSource::Coedge>& rCoedges = rFace.vLoops[0].vCoedges;
    bool bSeam = false;
    for (size_t a = 0; a < rCoedges.size() && !bSeam; ++a)
    {
        int iCount = 0;
        for (const OcctFileSource::Coedge& rC : rCoedges)
        {
            if (rC.iEdgeStorageIndex == rCoedges[a].iEdgeStorageIndex)
            {
                ++iCount;
            }
        }
        bSeam = (iCount >= 2);
    }
    if (!bSeam)
    {
        return false;
    }
    if (!HasOnlyNaturalSeamBoundary(rSource, rFace))
    {
        return false;
    }
    // The loop must span the FULL natural rectangle (U ~ 2*pi, V ~ the surface's natural V extent).
    // A trimmed patch (e.g. a hemisphere) also has a seam but a smaller V span; synthesizing the full
    // natural boundary for it would wrongly close it into a complete surface.
    if (sPCurveBox.IsEmpty())
    {
        return false;
    }
    const double dTwoPi = 2.0 * M_PI;
    const double dVSpanNatural = (eRawType == ESurfaceType::Sphere) ? M_PI : dTwoPi;
    const double dUSpan = sPCurveBox.GetMax()[0] - sPCurveBox.GetMin()[0];
    const double dVSpan = sPCurveBox.GetMax()[1] - sPCurveBox.GetMin()[1];
    const double dTol = 1.0e-3;
    return std::abs(dUSpan - dTwoPi) < dTol && std::abs(dVSpan - dVSpanNatural) < dTol;
}

void PrepareFaceLoopsForStaging(const OcctFileSource& rSource, OcctFileSource::Face& rFace)
{
    for (OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        rSource.PrepareCoedgesForStaging(rLoop.vCoedges);
    }
    rFace.vLoops.erase(
        std::remove_if(
            rFace.vLoops.begin(),
            rFace.vLoops.end(),
            [](const OcctFileSource::Loop& rLoop)
            {
                return rLoop.vCoedges.empty();
            }
        ),
        rFace.vLoops.end()
    );
    if (rFace.uiOuterLoopIndex >= rFace.vLoops.size())
    {
        rFace.uiOuterLoopIndex = 0;
    }
}

// Canonical dedup key for a TShape instance: its storage index plus its accumulated world
// transform, quantized so that products composed along different (but physically identical) paths
// hash to the same bucket.
std::string MakeInstanceKey(int iStorageIndex, const pxr::GfMatrix4d& rXf)
{
    std::string sKey;
    sKey.reserve(64);
    sKey += std::to_string(iStorageIndex);
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            const double dRounded = std::round(rXf[r][c] * 1.0e6) / 1.0e6;
            sKey += ':';
            sKey += std::to_string(dRounded + 0.0); // +0.0 normalizes -0
        }
    }
    return sKey;
}

} // namespace

OcctStagingDriver::OcctStagingDriver(const OcctFileSource& rSource, BrepStagingState& rState) : m_rSource(rSource), m_rState(rState)
{
}

StagedBrepTopologyBuilder OcctStagingDriver::MakeTopologyBuilder()
{
    return StagedBrepTopologyBuilder(
        m_rState,
        m_uiSolidRegion_TgtIndex,
        m_uiShell_TgtIndex,
        m_uiShell_PairedSpacing,
        m_uiInfiniteFaceuse_TgtIndex,
        m_uiFaceuse_TgtIndex,
        m_uiFaceuse_PairedSpacing
    );
}

uint32_t OcctStagingDriver::InfiniteRegionFaceuseCount() const
{
    // One infinite-region faceuse per face of each connex's first shell (doubled for open shells).
    uint32_t uiCount = 0;
    for (const OcctFileSource::Solid& rSolid : m_rSource.Solids())
    {
        if (rSolid.vShells.empty())
        {
            continue;
        }
        const OcctFileSource::Shell& rFirstShell = rSolid.vShells.front();
        const uint32_t uiFaceCount = static_cast<uint32_t>(rFirstShell.vFaces.size());
        uiCount += rFirstShell.bClosed ? uiFaceCount : 2 * uiFaceCount;
    }
    return uiCount;
}

OcctStagingDriver::ConnexReserve OcctStagingDriver::ComputeConnexReserve(const OcctFileSource::Solid& rSolid) const
{
    ConnexReserve sReserve;
    for (size_t ii = 0; ii < rSolid.vShells.size(); ++ii)
    {
        const OcctFileSource::Shell& rShell = rSolid.vShells[ii];
        const uint32_t uiFaceCount = static_cast<uint32_t>(rShell.vFaces.size());
        if (ii == 0)
        {
            if (rShell.bClosed)
            {
                sReserve.uiShellBlockSize += 1;
                sReserve.uiFaceuseBlockSize += uiFaceCount;
            }
        }
        else if (rShell.bClosed)
        {
            sReserve.uiShellBlockSize += 2;
            sReserve.uiFaceuseBlockSize += 2 * uiFaceCount;
            sReserve.uiShellPairedSpacingIncrement++;
            sReserve.uiFaceusePairedSpacingIncrement += uiFaceCount;
        }
        else
        {
            sReserve.uiShellBlockSize += 1;
            sReserve.uiFaceuseBlockSize += 2 * uiFaceCount;
            sReserve.uiShellPairedSpacingIncrement++;
            sReserve.uiFaceusePairedSpacingIncrement += 2 * uiFaceCount;
        }
    }
    return sReserve;
}

uint32_t OcctStagingDriver::ParentRegionIndexForCurrentShell(bool bFirstShell) const
{
    if (bFirstShell)
    {
        return 0;
    }
    return (m_uiSolidRegion_TgtIndex < m_rState.GetRegionCount()) ? m_uiSolidRegion_TgtIndex : 0;
}

bool OcctStagingDriver::Run(std::string& rError)
{
    const std::vector<OcctFileSource::Solid>& vSolids = m_rSource.Solids();
    if (vSolids.empty())
    {
        rError = "no solids to stage";
        return false;
    }

    SourceBrepData sBrepData;
    sBrepData.dTolerance = m_rSource.Tolerance();
    sBrepData.uiConnectedComponentCount = static_cast<uint32_t>(vSolids.size());
    sBrepData.uiInfiniteRegionFaceuseCount = InfiniteRegionFaceuseCount();

    MakeTopologyBuilder().ResetStagedTopologyTargetIndicesForBrep();
    m_rState.ClearDedupMaps();
    GenericBrepStagingBuilder(m_rState).ResetForSourceBrep(sBrepData);

    for (const OcctFileSource::Solid& rSolid : vSolids)
    {
        if (!DriveSolid(rSolid, rError))
        {
            return false;
        }
    }

    StagedBrepExtentBuilder(m_rState).UpdateExtent();
    return true;
}

bool OcctStagingDriver::DriveSolid(const OcctFileSource::Solid& rSolid, std::string& rError)
{
    m_uiConnexCount++;
    MakeTopologyBuilder().BeginStagedConnectedComponentTopology();

    const ConnexReserve sReserve = ComputeConnexReserve(rSolid);
    m_uiShell_PairedSpacing += sReserve.uiShellPairedSpacingIncrement;
    m_uiFaceuse_PairedSpacing += sReserve.uiFaceusePairedSpacingIncrement;
    MakeTopologyBuilder().ReserveStagedConnectedComponentTopology(sReserve.uiShellBlockSize, sReserve.uiFaceuseBlockSize);

    for (size_t ii = 0; ii < rSolid.vShells.size(); ++ii)
    {
        if (!DriveShell(rSolid.vShells[ii], ii == 0, rError))
        {
            return false;
        }
    }
    return true;
}

bool OcctStagingDriver::DriveShell(const OcctFileSource::Shell& rShell, bool bFirstShell, std::string& rError)
{
    const uint32_t uiFaceCount = static_cast<uint32_t>(rShell.vFaces.size());

    uint32_t ui1stShellIndex = 0;
    uint32_t ui2ndShellIndex = 0;
    MakeTopologyBuilder()
        .GetStagedShellTargetIndicesForCurrentTraversal(bFirstShell, m_uiConnexCount, rShell.bClosed, ui1stShellIndex, ui2ndShellIndex);

    MakeTopologyBuilder().AddStagedShellForCurrentTraversal(
        ParentRegionIndexForCurrentShell(bFirstShell),
        ui1stShellIndex,
        ui2ndShellIndex,
        uiFaceCount,
        rShell.bClosed
    );

    for (const OcctFileSource::Face& rFace : rShell.vFaces)
    {
        if (!DriveFace(rShell, rFace, bFirstShell, rError))
        {
            return false;
        }
    }
    return true;
}

bool OcctStagingDriver::DriveFace(const OcctFileSource::Shell& rShell, const OcctFileSource::Face& rFaceIn, bool bFirstShell, std::string& rError)
{
    // Pre-pass: a full closed self-contained analytic face (sphere/torus) whose loop is OCCT's natural
    // parameter seam (plus, for a sphere, degenerate pole edges) is malformed when emitted verbatim --
    // the zero-length pole edges cannot carry SameParameter UV trim curves, and the seam loop fails to
    // close in UV. Two cases, distinguished by whether the surface has a pole singularity:
    //
    //  - Sphere (has degenerate pole edges): preserve OCCT's real seam edge and drop the degenerate pole
    //    coedges so the poles become the seam's endpoint vertices, leaving a correctly-ordered seam loop.
    //    The pole singularities absorb the seam's UV discontinuity, so the bare seam loop is valid.
    //
    //  - Torus (no pole edges): a bare seam loop has no singularity to absorb the UV gap and tears at the
    //    seam, so synthesize the natural boundary (which emits the seam-side encoding the explicit loop
    //    lacks) by clearing the loop and treating the face as a natural restriction.
    OcctFileSource::Face sFaceLocal;
    const OcctFileSource::Face* pFace = &rFaceIn;
    bool bPreservedSeamFace = false;
    bool bPreparedForStaging = false;
    if (rFaceIn.iSurfaceIndex >= 1 && static_cast<size_t>(rFaceIn.iSurfaceIndex) <= m_rSource.Model().sSurfaces.size())
    {
        const ESurfaceType eRawType = m_rSource.Model().sSurfaces[static_cast<size_t>(rFaceIn.iSurfaceIndex - 1)].eType;
        const pxr::GfRange2d sRawPCurveBox = m_rSource.ComputeFaceUVDomain(rFaceIn);
        if (ShouldSynthesizeNaturalBoundary(m_rSource, rFaceIn, eRawType, sRawPCurveBox))
        {
            sFaceLocal = rFaceIn;
            bool bHasDegenerateEdge = false;
            for (const OcctFileSource::Loop& rLoop : sFaceLocal.vLoops)
            {
                for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
                {
                    if (m_rSource.ResolveEdge(rCoedge.iEdgeStorageIndex).bDegenerated)
                    {
                        bHasDegenerateEdge = true;
                    }
                }
            }

            if (bHasDegenerateEdge)
            {
                // Sphere: preserve the seam edge, demote the degenerate pole edges to vertices.
                PrepareFaceLoopsForStaging(m_rSource, sFaceLocal);
                bPreservedSeamFace = true;
                bPreparedForStaging = true;
            }
            else
            {
                // Torus: synthesize the natural boundary (no singularity to absorb a bare seam's UV gap).
                sFaceLocal.vLoops.clear();
                sFaceLocal.bNaturalRestriction = true;
            }
            pFace = &sFaceLocal;
        }
    }
    if (!bPreparedForStaging)
    {
        if (pFace != &sFaceLocal)
        {
            sFaceLocal = *pFace;
            pFace = &sFaceLocal;
        }
        PrepareFaceLoopsForStaging(m_rSource, sFaceLocal);
    }
    OcctFileSource::Face& rFace = sFaceLocal;

    const uint32_t uiLoopCount = static_cast<uint32_t>(rFace.vLoops.size());

    // Accumulates the transform from OCCT's native pcurve parameter space to the UV domain we author
    // (analytic-frame reflection, U period shift, cone slant->axial V rescale), so staged edgeuse UV
    // curves match face:range.
    FaceUvCurveContext sUvCtx;
    sUvCtx.iSurface = rFace.iSurfaceIndex;
    bool bUvShiftSimple = true;

    // The pcurve-derived UV box bounds swept-surface lowering (extrusion V distance, revolution V
    // angle, line-basis U) and seeds the non-plane domain below.
    const pxr::GfRange2d sSourcePCurveBox = m_rSource.ComputeFaceUVDomain(rFace);
    pxr::GfRange2d sPCurveBox = sSourcePCurveBox;

    // Surface geometry (converted first so we know whether the parameterization is periodic).
    SourceSurfaceData sSurfaceData;
    bool bSurfaceConverted = false;
    bool bSweptDomainValid = false;
    pxr::GfRange2d sSweptDomain;
    if (rFace.iSurfaceIndex >= 1 && static_cast<size_t>(rFace.iSurfaceIndex) <= m_rSource.Model().sSurfaces.size())
    {
        const SSurface& rSurface = m_rSource.Model().sSurfaces[static_cast<size_t>(rFace.iSurfaceIndex - 1)];
        // World surface transform = surface-representation location, then the face's accumulated location.
        const pxr::GfMatrix4d sXf = ResolveLocation(m_rSource.Model(), rFace.iSurfaceLocation) * rFace.mFaceLoc;
        pxr::GfRange2d sSweptOut;
        const bool bIsSwept = (rSurface.eType == ESurfaceType::LinearExtrusion || rSurface.eType == ESurfaceType::Revolution);
        bSurfaceConverted = ConvertSurface(rSurface, sXf, sSurfaceData.sSurfaceData, &sSourcePCurveBox, &sSweptOut);
        if (bSurfaceConverted && bIsSwept && !sSweptOut.IsEmpty())
        {
            bSweptDomainValid = true;
            sSweptDomain = sSweptOut;
        }
        if (bSurfaceConverted)
        {
            const SurfaceParameterAdjustment sParamAdjustment = ComputeSurfaceParameterAdjustment(rSurface, sXf, sSurfaceData.sSurfaceData);
            sUvCtx.dUScale = sParamAdjustment.dUScale;
            sUvCtx.dVScale = sParamAdjustment.dVScale;
            sPCurveBox = ScaleURange(sPCurveBox, sParamAdjustment.dUScale);
            if (sParamAdjustment.bReverseOrientation)
            {
                rFace.bSameOrientationWithShell = !rFace.bSameOrientationWithShell;
                for (OcctFileSource::Loop& rLoop : rFace.vLoops)
                {
                    rLoop.bSameOrientationWithSurface = !rLoop.bSameOrientationWithSurface;
                }
            }
        }
    }
    if (!bSurfaceConverted)
    {
        rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": unsupported or missing surface";
        return false;
    }

    // A parameter reflection reverses the analytic surface's natural normal. The adjusted source
    // orientation above keeps the staged face's shell orientation equivalent to the OCCT face.
    // The topology builder assumes the first (outer) shell is forward and subsequent (void) shells
    // are reversed. Fold any differing OCCT shell/container orientation into the faceuse sense.
    const bool bExpectedShellSame = bFirstShell;
    const bool bSameFaceOrient = (rShell.bSameOrientationWithSolid == bExpectedShellSame) ? rFace.bSameOrientationWithShell :
                                                                                            !rFace.bSameOrientationWithShell;

    // Parametric (UV) domain from the face's 2D pcurves; required by the BrepArray schema.
    // Planes parameterize directly in their own (refDirection, axis x refDirection) frame, so the
    // exact domain is the projection of the boundary into that frame; we use it unconditionally for
    // planes (OCCT often stores no pcurves for them, and any stored pcurve is in this same frame).
    const bool bPlane = (sSurfaceData.sSurfaceData.eSurfaceType == StagedSurfaceType::Plane);
    pxr::GfRange2d sUVDomain = bPlane ? pxr::GfRange2d() : sPCurveBox;
    // A lowered swept surface uses its own (NURBS-natural) parameterization, so use the param box the
    // converter reported rather than the OCCT pcurve box.
    if (bSweptDomainValid)
    {
        sUVDomain = sSweptDomain;
    }

    // Planes use the projected 3D boundaries for both their face domain and outer-loop selection.
    // The serialized OCCT format does not identify the outer wire, and some valid loops have no
    // pcurve, so relying on pcurve extents or file order alone is insufficient. A hole's projected
    // box is contained by the outer boundary's box, making the largest measurable box the outer loop.
    if (bPlane)
    {
        double dBestArea = -1.0;
        uint32_t uiBestLoop = rFace.uiOuterLoopIndex;
        bool bAllLoopsMeasurable = true;
        for (size_t ii = 0; ii < rFace.vLoops.size(); ++ii)
        {
            const pxr::GfRange2d sLoopDomain = ComputePlaneLoopDomainFrom3dBoundary(m_rSource, rFace.vLoops[ii], sSurfaceData.sSurfaceData);
            sUVDomain.UnionWith(sLoopDomain);
            if (!IsFiniteNonEmptyRange(sLoopDomain))
            {
                bAllLoopsMeasurable = false;
                continue;
            }
            const pxr::GfVec2d sSize = sLoopDomain.GetSize();
            const double dArea = sSize[0] * sSize[1];
            if (dArea > dBestArea)
            {
                dBestArea = dArea;
                uiBestLoop = static_cast<uint32_t>(ii);
            }
        }
        if (rFace.vLoops.size() > 1 && bAllLoopsMeasurable)
        {
            rFace.uiOuterLoopIndex = uiBestLoop;
        }
    }

    // OCCT parameterizes a cone's V as slant distance along the ruling; the BrepArray cone schema
    // (BrepSurfaceConeAPI) uses axial distance: S = origin + V*axis, radius = refRadius + V*tan(semiAngle).
    // Rescale the
    // pcurve-derived V by cos(semiAngle) to convert slant -> axial. A negative OCCT half-angle was emitted
    // as a flipped axis + positive angle (see ConvertSurface), which mirrors the axial direction, so the
    // axial V is also negated; otherwise the trim lands on the wrong (mirrored) sheet of the cone.
    if (sSurfaceData.sSurfaceData.eSurfaceType == StagedSurfaceType::Cone && !sUVDomain.IsEmpty())
    {
        const SSurface& rConeSurface = m_rSource.Model().sSurfaces[static_cast<size_t>(rFace.iSurfaceIndex - 1)];
        const double dCosA = std::cos(sSurfaceData.sSurfaceData.dSemiAngle);
        const double dScale = (rConeSurface.dSemiAngle < 0.0) ? -dCosA : dCosA;
        sUvCtx.dVScale *= dScale;
        const double dVa = sUVDomain.GetMin()[1] * dScale;
        const double dVb = sUVDomain.GetMax()[1] * dScale;
        const double dV0 = (dVa < dVb) ? dVa : dVb;
        const double dV1 = (dVa < dVb) ? dVb : dVa;
        sUVDomain = pxr::GfRange2d(pxr::GfVec2d(sUVDomain.GetMin()[0], dV0), pxr::GfVec2d(sUVDomain.GetMax()[0], dV1));
    }

    // Recover analytic trim bounds from the ordered 3D boundary, independently of pcurves. This is
    // the authoritative signal when a pcurve box spans an entire period spuriously, lies across the
    // primary seam, or is absent. It also distinguishes one genuine period from a multi-turn patch.
    const StagedSurfaceType eAnalyticSurfaceType = sSurfaceData.sSurfaceData.eSurfaceType;
    const bool bAnalyticPeriodicU = IsPeriodicUSurface(eAnalyticSurfaceType);
    const bool bAnalyticTorus = eAnalyticSurfaceType == StagedSurfaceType::Torus;
    const double dTwoPi = 2.0 * M_PI;
    const double dPeriodTol = 1.0e-5;
    // Keep an explicit OCCT seam when both seam-side pcurves are available. Relocating the analytic
    // seam while retaining a repeated seam edge maps its two uses onto the same interior UV curve,
    // collapsing the trim loop. Preserve it only when the source already uses USD's primary angular
    // domain; otherwise the existing relocation path moves both the surface and trims into [0, 2*pi].
    const bool bSourceUInPrimaryPeriod = !sPCurveBox.IsEmpty() && sPCurveBox.GetMin()[0] >= -dPeriodTol &&
                                         sPCurveBox.GetMax()[0] <= dTwoPi + dPeriodTol;
    const bool bPreserveSourcePeriodicU = bAnalyticPeriodicU && bSourceUInPrimaryPeriod && HasExplicitSeamBoundary(m_rSource, rFace) &&
                                          HasCompleteBoundaryPCurves(m_rSource, rFace);
    pxr::GfRange1d sBoundaryU;
    pxr::GfRange1d sBoundaryV;
    if (bAnalyticPeriodicU)
    {
        sBoundaryU = ComputeTightPeriodicDomainFrom3dBoundary(m_rSource, rFace, sSurfaceData.sSurfaceData, PeriodicParameterAxis::U);
        sBoundaryV = bAnalyticTorus ?
                         ComputeTightPeriodicDomainFrom3dBoundary(m_rSource, rFace, sSurfaceData.sSurfaceData, PeriodicParameterAxis::V) :
                         ComputeNonPeriodicDomainFrom3dBoundary(m_rSource, rFace, sSurfaceData.sSurfaceData, PeriodicParameterAxis::V);
    }

    const auto fAlignBoundaryRangeToSource = [&](pxr::GfRange1d& rBoundary, int iComponent)
    {
        if (sUVDomain.IsEmpty() || !IsFiniteNonEmptyRange(rBoundary))
        {
            return;
        }
        const double dSourceMid = 0.5 * (sUVDomain.GetMin()[iComponent] + sUVDomain.GetMax()[iComponent]);
        const double dBoundaryMid = 0.5 * (rBoundary.GetMin() + rBoundary.GetMax());
        const double dShift = dTwoPi * std::round((dSourceMid - dBoundaryMid) / dTwoPi);
        rBoundary = pxr::GfRange1d(rBoundary.GetMin() + dShift, rBoundary.GetMax() + dShift);
    };
    fAlignBoundaryRangeToSource(sBoundaryU, 0);
    if (bAnalyticTorus)
    {
        fAlignBoundaryRangeToSource(sBoundaryV, 1);
    }

    const bool bHaveBoundaryU = IsFiniteNonEmptyRange(sBoundaryU);
    const bool bHaveBoundaryV = IsFiniteNonEmptyRange(sBoundaryV);
    const auto fCrossesPrimaryPeriod = [&](const pxr::GfRange1d& rRange)
    {
        return rRange.GetMin() < -dPeriodTol || rRange.GetMax() > dTwoPi + dPeriodTol;
    };
    const auto fSetDomainComponent = [&](int iComponent, const pxr::GfRange1d& rRange)
    {
        pxr::GfVec2d sMin = sUVDomain.GetMin();
        pxr::GfVec2d sMax = sUVDomain.GetMax();
        sMin[iComponent] = rRange.GetMin();
        sMax[iComponent] = rRange.GetMax();
        sUVDomain = pxr::GfRange2d(sMin, sMax);
    };

    if (sUVDomain.IsEmpty() && bHaveBoundaryU && bHaveBoundaryV)
    {
        sUVDomain = pxr::GfRange2d(pxr::GfVec2d(sBoundaryU.GetMin(), sBoundaryV.GetMin()), pxr::GfVec2d(sBoundaryU.GetMax(), sBoundaryV.GetMax()));
        bUvShiftSimple = false;
    }
    else if (!sUVDomain.IsEmpty())
    {
        if (bHaveBoundaryU && !bPreserveSourcePeriodicU)
        {
            const pxr::GfRange1d sSourceU(sUVDomain.GetMin()[0], sUVDomain.GetMax()[0]);
            const double dSourceSpan = sSourceU.GetMax() - sSourceU.GetMin();
            const double dBoundarySpan = sBoundaryU.GetMax() - sBoundaryU.GetMin();
            if (dSourceSpan >= dTwoPi - dPeriodTol || fCrossesPrimaryPeriod(sSourceU) || dBoundarySpan > dTwoPi + dPeriodTol)
            {
                fSetDomainComponent(0, sBoundaryU);
                bUvShiftSimple = false;
            }
        }
        if (bAnalyticTorus && bHaveBoundaryV)
        {
            const pxr::GfRange1d sSourceV(sUVDomain.GetMin()[1], sUVDomain.GetMax()[1]);
            const double dSourceSpan = sSourceV.GetMax() - sSourceV.GetMin();
            const double dBoundarySpan = sBoundaryV.GetMax() - sBoundaryV.GetMin();
            if (dSourceSpan >= dTwoPi - dPeriodTol || fCrossesPrimaryPeriod(sSourceV) || dBoundarySpan > dTwoPi + dPeriodTol)
            {
                fSetDomainComponent(1, sBoundaryV);
                bUvShiftSimple = false;
            }
        }
    }

    bool bLoweredPeriodicPatch = false;
    const double dBoundaryUSpan = bHaveBoundaryU ? sBoundaryU.GetMax() - sBoundaryU.GetMin() : 0.0;
    const double dBoundaryVSpan = bHaveBoundaryV ? sBoundaryV.GetMax() - sBoundaryV.GetMin() : 0.0;
    const bool bMultiTurnU = !bPreserveSourcePeriodicU && bHaveBoundaryU && dBoundaryUSpan > dTwoPi + dPeriodTol;
    const bool bMultiTurnV = bAnalyticTorus && bHaveBoundaryV && dBoundaryVSpan > dTwoPi + dPeriodTol;
    const bool bTorusVNeedsUnwrapping = bAnalyticTorus && bHaveBoundaryV && dBoundaryVSpan < dTwoPi - dPeriodTol && fCrossesPrimaryPeriod(sBoundaryV);
    if (bMultiTurnV)
    {
        rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": multi-turn analytic V trim requires topology splitting";
        return false;
    }
    if ((bMultiTurnU || bTorusVNeedsUnwrapping) && !sUVDomain.IsEmpty())
    {
        pxr::GfVec2d sLowerMin = sUVDomain.GetMin();
        pxr::GfVec2d sLowerMax = sUVDomain.GetMax();
        if (bHaveBoundaryU)
        {
            sLowerMin[0] = sBoundaryU.GetMin();
            sLowerMax[0] = sBoundaryU.GetMax();
        }
        if (bHaveBoundaryV)
        {
            sLowerMin[1] = sBoundaryV.GetMin();
            sLowerMax[1] = sBoundaryV.GetMax();
        }
        StagedSurfaceData sNurbSurface;
        pxr::GfRange2d sLowerDomain(sLowerMin, sLowerMax);
        bool bLowered = false;
        if (bMultiTurnU && eAnalyticSurfaceType == StagedSurfaceType::Cylinder && bHaveBoundaryV)
        {
            struct ShearedCandidate
            {
                bool bValid = false;
                double dAxialStart = 0.0;
                double dAxialEnd = 0.0;
                pxr::GfRange1d sVDomain;
            };
            const pxr::GfRange1d sUDomain(sLowerMin[0], sLowerMax[0]);
            const double dTurnCount = dBoundaryUSpan / dTwoPi;
            const auto fMakeCandidate = [&](double dAxialStart, double dAxialEnd)
            {
                ShearedCandidate sCandidate;
                sCandidate.dAxialStart = dAxialStart;
                sCandidate.dAxialEnd = dAxialEnd;
                RationalQuadraticArc sArc;
                pxr::VtArray<double> vAxialControls;
                StagedSurfaceData sScratch;
                if (!BuildShearedCylinderPatch(
                        sSurfaceData.sSurfaceData,
                        sUDomain,
                        pxr::GfRange1d(0.0, 1.0),
                        dAxialStart,
                        dAxialEnd,
                        sScratch,
                        sArc,
                        vAxialControls
                    ))
                {
                    return sCandidate;
                }
                sCandidate.sVDomain = ComputeShearedCylinderVDomainFrom3dBoundary(
                    m_rSource,
                    rFace,
                    sSurfaceData.sSurfaceData,
                    sUDomain,
                    sArc,
                    vAxialControls
                );
                if (!IsFiniteNonEmptyRange(sCandidate.sVDomain))
                {
                    return sCandidate;
                }
                const double dAdjacentTurnShift = std::abs(dAxialEnd - dAxialStart) / dTurnCount;
                const double dTrimWidth = sCandidate.sVDomain.GetMax() - sCandidate.sVDomain.GetMin();
                sCandidate.bValid = dAdjacentTurnShift > dTrimWidth + 1.0e-6;
                return sCandidate;
            };

            ShearedCandidate sForward = fMakeCandidate(sBoundaryV.GetMin(), sBoundaryV.GetMax());
            ShearedCandidate sReverse = fMakeCandidate(sBoundaryV.GetMax(), sBoundaryV.GetMin());
            const ShearedCandidate* pCandidate = nullptr;
            if (sForward.bValid && sReverse.bValid)
            {
                const double dForwardWidth = sForward.sVDomain.GetMax() - sForward.sVDomain.GetMin();
                const double dReverseWidth = sReverse.sVDomain.GetMax() - sReverse.sVDomain.GetMin();
                pCandidate = dForwardWidth <= dReverseWidth ? &sForward : &sReverse;
            }
            else if (sForward.bValid)
            {
                pCandidate = &sForward;
            }
            else if (sReverse.bValid)
            {
                pCandidate = &sReverse;
            }

            if (!pCandidate)
            {
                rError = "face " + std::to_string(rFace.iFaceStorageIndex) +
                         ": multi-turn cylinder trim overlaps adjacent periods and requires topology splitting";
                return false;
            }

            RationalQuadraticArc sArc;
            pxr::VtArray<double> vAxialControls;
            bLowered = BuildShearedCylinderPatch(
                sSurfaceData.sSurfaceData,
                sUDomain,
                pCandidate->sVDomain,
                pCandidate->dAxialStart,
                pCandidate->dAxialEnd,
                sNurbSurface,
                sArc,
                vAxialControls
            );
            sLowerDomain = pxr::GfRange2d(
                pxr::GfVec2d(sUDomain.GetMin(), pCandidate->sVDomain.GetMin()),
                pxr::GfVec2d(sUDomain.GetMax(), pCandidate->sVDomain.GetMax())
            );
        }
        else if (bMultiTurnU)
        {
            rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": multi-turn analytic U trim requires topology splitting";
            return false;
        }
        else
        {
            bLowered = LowerAnalyticSurfacePatchToNurbs(sSurfaceData.sSurfaceData, sLowerDomain, sNurbSurface);
        }
        if (!bLowered)
        {
            rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": failed to lower unwrapped analytic patch";
            return false;
        }
        sSurfaceData.sSurfaceData = std::move(sNurbSurface);
        sUVDomain = sLowerDomain;
        bUvShiftSimple = false;
        bLoweredPeriodicPatch = true;
    }
    else if (!bPreserveSourcePeriodicU && bHaveBoundaryU && dBoundaryUSpan < dTwoPi - dPeriodTol && fCrossesPrimaryPeriod(sBoundaryU))
    {
        if (!RelocateAnalyticUSeam(sSurfaceData.sSurfaceData, sBoundaryU.GetMin()))
        {
            rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": failed to relocate analytic U seam";
            return false;
        }
        pxr::GfVec2d sMin = sUVDomain.GetMin();
        pxr::GfVec2d sMax = sUVDomain.GetMax();
        sMin[0] = 0.0;
        sMax[0] = dBoundaryUSpan;
        sUVDomain = pxr::GfRange2d(sMin, sMax);
        bUvShiftSimple = false;
    }

    if (!bLoweredPeriodicPatch && eAnalyticSurfaceType == StagedSurfaceType::Torus && !sUVDomain.IsEmpty())
    {
        const double dVSpan = sUVDomain.GetMax()[1] - sUVDomain.GetMin()[1];
        if (std::abs(dVSpan - dTwoPi) <= dPeriodTol)
        {
            sUVDomain = pxr::GfRange2d(pxr::GfVec2d(sUVDomain.GetMin()[0], 0.0), pxr::GfVec2d(sUVDomain.GetMax()[0], dTwoPi));
            bUvShiftSimple = false;
        }
    }

    // A loop-less natural-restriction face is bounded by the surface's natural parameter range. A
    // closed/periodic surface with explicit wires but no pcurves may also need an implied natural
    // outer loop, with the explicit wires treated as holes. Do not use "missing pcurves + bounded
    // natural domain" as a blanket signal: an exported open NURBS face can have a perfectly valid
    // explicit wire and still lack pcurves, in which case the wire remains the outer loop.
    const pxr::GfRange2d sNaturalDomain = ComputeNaturalDomain(sSurfaceData.sSurfaceData);
    if (rFace.bNaturalRestriction && rFace.vLoops.empty() && sNaturalDomain.IsEmpty())
    {
        rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": natural-restriction surface has no bounded natural domain";
        return false;
    }

    PeriodicBoundaryRepair sPeriodicRepair;
    if (!bLoweredPeriodicPatch && TryBuildPeriodicBoundaryRepair(rFace, sSurfaceData.sSurfaceData, sPCurveBox, sPeriodicRepair) &&
        sPeriodicRepair.bEnabled)
    {
        sUVDomain = sPeriodicRepair.sDomain;
    }

    const bool bHasPeriodicRepair = sPeriodicRepair.bEnabled;
    if (!bHasPeriodicRepair && sUVDomain.IsEmpty())
    {
        const pxr::GfRange2d sBoundary3dDomain = ComputeFullPeriodAxialDomainFrom3dBoundary(m_rSource, rFace, sSurfaceData.sSurfaceData);
        if (!sBoundary3dDomain.IsEmpty())
        {
            sUVDomain = sBoundary3dDomain;
        }
    }

    const bool bLooplessNaturalRestriction = rFace.bNaturalRestriction && rFace.vLoops.empty();
    const bool bClosedSurfaceExplicitWiresWithoutPCurves = !rFace.vLoops.empty() && sUVDomain.IsEmpty() &&
                                                           CanUseImpliedNaturalBoundaryWithoutPCurves(sSurfaceData.sSurfaceData);
    const bool bImpliedNaturalBoundary = !bHasPeriodicRepair && sUVDomain.IsEmpty() && !sNaturalDomain.IsEmpty() &&
                                         (bLooplessNaturalRestriction || bClosedSurfaceExplicitWiresWithoutPCurves);
    if (bImpliedNaturalBoundary)
    {
        sUVDomain = sNaturalDomain;
    }
    else if (sUVDomain.IsEmpty() && !sNaturalDomain.IsEmpty())
    {
        sUVDomain = sNaturalDomain;
    }
    // A preserved-seam face's pcurve-derived domain collapses (the seam pcurves are both isoparameter
    // lines, and the U-extent pcurves lived on the dropped pole edges), so use the full natural rectangle.
    if (bPreservedSeamFace && !sNaturalDomain.IsEmpty())
    {
        sUVDomain = sNaturalDomain;
    }

    // For periodic surfaces the angular U range is shifted into the primary period (0, 2*pi].
    if (IsPeriodicUSurface(sSurfaceData.sSurfaceData.eSurfaceType) && !sUVDomain.IsEmpty())
    {
        const double dULoBefore = sUVDomain.GetMin()[0];
        double dULo = dULoBefore;
        double dUHi = sUVDomain.GetMax()[0];
        // A closed (full-revolution) face whose seam pcurves run in opposite directions can yield a
        // U span > 2*pi (e.g. [-2pi, 2pi]); clamp it to one full period since the surface is periodic.
        // NormalizeAngularInterval canonicalizes a benign ~2*pi span (pcurve endpoints a few ulps
        // past 2*pi) without suppressing the seam UV curves below.
        if (dUHi - dULo > dTwoPi + 1.0e-6)
        {
            dULo = 0.0;
            dUHi = dTwoPi;
            bUvShiftSimple = false; // not a pure shift: don't emit pcurves whose U would not line up
        }
        NormalizeAngularInterval(dULo, dUHi);
        // The same whole-period shift that maps face:range U must be applied to the pcurve U values.
        sUvCtx.dUAdd = dULo - dULoBefore;
        sUVDomain = pxr::GfRange2d(pxr::GfVec2d(dULo, sUVDomain.GetMin()[1]), pxr::GfVec2d(dUHi, sUVDomain.GetMax()[1]));
    }
    if (!IsFiniteNonEmptyRange(sUVDomain))
    {
        rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": invalid finite UV face range";
        return false;
    }

    // SourceFaceData. Explicit-loop faces, plus closed-surface faces with a synthesized implied natural
    // outer boundary (the implied loop + any explicit wires as holes).
    const uint32_t uiStagedLoopCount = bHasPeriodicRepair ? static_cast<uint32_t>(sPeriodicRepair.vLoops.size()) :
                                                            (bImpliedNaturalBoundary ? (uiLoopCount + 1u) : uiLoopCount);
    SourceFaceData sFaceData;
    sFaceData.uiLoopCount = uiStagedLoopCount;
    sFaceData.uiOuterLoopIndex = (bImpliedNaturalBoundary || bHasPeriodicRepair) ? 0u : rFace.uiOuterLoopIndex;
    sFaceData.bSameOrientationWithShell = bSameFaceOrient;
    sFaceData.bClosedSurface = bImpliedNaturalBoundary;
    sFaceData.bAllLoopsCW = bImpliedNaturalBoundary; // the implied loop is the outer; explicit loops are holes
    sFaceData.uiCCWLoopCount = uiLoopCount;
    sFaceData.bHasImpliedNaturalBoundary = bImpliedNaturalBoundary;
    sFaceData.sDomain = sUVDomain;

    // Per-face outer-loop edgeuse cursor (mirrors InitializeStagedOuterLoopStateForCurrentFace).
    m_uiOuterLoop_EdgeusesTgtIndex = m_rState.GetEdgeuseCount();
    if (bHasPeriodicRepair)
    {
        m_uiOuterLoop_EdgeuseCount = !sPeriodicRepair.vLoops.empty() ?
                                         static_cast<uint32_t>(sPeriodicRepair.vLoops[sPeriodicRepair.uiOuterLoopIndex].vCoedges.size()) :
                                         0;
    }
    else
    {
        m_uiOuterLoop_EdgeuseCount = (rFace.uiOuterLoopIndex < rFace.vLoops.size()) ?
                                         static_cast<uint32_t>(rFace.vLoops[rFace.uiOuterLoopIndex].vCoedges.size()) :
                                         0;
    }

    // face:trimType. "rectangular" means the face's single outer loop consists of the four complete
    // isoparametric sides of face:range. It may be an interior rectangle, but merely having four
    // isoparametric curves is insufficient. Natural seam-only and implied boundaries are known cases.
    const bool bSourcePCurvesComparable = !bHasPeriodicRepair && !bImpliedNaturalBoundary && !bSweptDomainValid && bUvShiftSimple &&
                                          !bPreservedSeamFace;
    const StagedTrimType eTrimType = (!bHasPeriodicRepair && FaceHasRectangularTrim(
                                                                 m_rSource,
                                                                 rFace,
                                                                 sSurfaceData.sSurfaceData,
                                                                 uiStagedLoopCount,
                                                                 bImpliedNaturalBoundary,
                                                                 bSourcePCurvesComparable,
                                                                 sUVDomain,
                                                                 pxr::GfVec2d(sUvCtx.dUScale, sUvCtx.dVScale),
                                                                 pxr::GfVec2d(sUvCtx.dUAdd, 0.0)
                                                             )) ?
                                         StagedTrimType::Rectangular :
                                         StagedTrimType::General;

    // Preserve every source pcurve whose surface parameterization maps affinely to the authored
    // face. Repairs and implied boundaries create edgeuses with no source pcurve; lowered swept or
    // seam-relocated patches use a different UV parameterization. Those edgeuses retain (0, 0)
    // placeholders while compatible source pcurves elsewhere remain authored.
    sUvCtx.bParameterizationCompatible = bSourcePCurvesComparable;
    sUvCtx.sDomain = sUVDomain;

    GenericBrepStagingBuilder sGenericBuilder(m_rState);
    const uint32_t uiStagedFaceIndex = sGenericBuilder.AddFace(sFaceData, uiStagedLoopCount, eTrimType);

    MakeTopologyBuilder().AddStagedFaceusePairForCurrentTraversal(bFirstShell, rShell.bClosed, uiStagedFaceIndex, bSameFaceOrient);

    sSurfaceData.sDomain = sUVDomain;
    sGenericBuilder.AddFaceSurfaceForCurrentFace(sSurfaceData, eTrimType);

    // Closed-surface face: synthesize the implied natural outer boundary loop first (the staging layer
    // builds the seam edges/edgeuses per surface type from the face domain). Any explicit wires below
    // are then staged as inner (hole) loops via the implied-natural-boundary coedge path.
    if (bImpliedNaturalBoundary)
    {
        NaturalBoundaryBuilder sNaturalBoundary(
            m_rState,
            StagedBrepEdgeVertexBuilder(m_rState),
            MakeTopologyBuilder(),
            /*uiAugmentedOuterLoopEdgeuseCount*/ 0,
            m_rState.GetEdgeuseCount()
        );

        SourceNaturalBoundaryData sBoundaryData;
        if (sNaturalBoundary.ExtractNaturalBoundaryDataForCurrentFace(sBoundaryData) != BrepStatusSuccess)
        {
            rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": failed to extract natural boundary";
            return false;
        }
        sNaturalBoundary.ApplyNaturalBoundaryData(sBoundaryData);
        if (sNaturalBoundary.ExtractImpliedBoundaryEdgesForCurrentFace(SourceNaturalBoundaryVertexResolver(), kStagedNoObjectIndex) !=
                BrepStatusSuccess ||
            sNaturalBoundary.AddImpliedNaturalBoundaryAsLoopForCurrentFace() != BrepStatusSuccess)
        {
            rError = "face " + std::to_string(rFace.iFaceStorageIndex) + ": failed to build implied natural boundary";
            return false;
        }
    }

    if (bHasPeriodicRepair)
    {
        return StagePeriodicBoundaryRepair(sPeriodicRepair, sUvCtx, rError);
    }

    // Loops + coedges. With an implied natural boundary present, the explicit wires are inner holes.
    uint32_t uiLoopTraversalCount = 0;
    for (const OcctFileSource::Loop& rLoop : rFace.vLoops)
    {
        ++uiLoopTraversalCount;

        SourceLoopData sLoopData;
        sLoopData.uiCoedgeCount = static_cast<uint32_t>(rLoop.vCoedges.size());
        sLoopData.bSameOrientationWithSurface = rLoop.bSameOrientationWithSurface;

        // With an implied natural outer boundary, every explicit wire is an inner (hole) loop.
        const bool bIsOuterLoop = !bImpliedNaturalBoundary && (uiLoopTraversalCount == rFace.uiOuterLoopIndex + 1);

        sGenericBuilder.AddSourceLoopForCurrentTraversal(
            sLoopData,
            /*bIsAugmentedOuterLoop*/ false,
            m_uiOuterLoop_EdgeuseCount,
            bIsOuterLoop,
            uiLoopTraversalCount,
            bImpliedNaturalBoundary
        );

        uint32_t uiLoopFirstEdgeuse = kStagedNoObjectIndex;
        uint32_t uiCoedgeTraversalCount = 0;
        for (const OcctFileSource::Coedge& rCoedge : rLoop.vCoedges)
        {
            ++uiCoedgeTraversalCount;

            StagedBrepCoedgeBuilder sCoedgeBuilder(
                m_rState,
                uiLoopTraversalCount,
                uiCoedgeTraversalCount,
                rFace.uiOuterLoopIndex,
                m_uiOuterLoop_EdgeusesTgtIndex,
                m_uiOuterLoop_EdgeuseCount,
                bImpliedNaturalBoundary
            );

            SourceCoedgeData sCoedgeData;
            sCoedgeData.bSameOrientationWithLoop = rCoedge.bSameOrientationWithLoop;

            const StagedCoedgeBuildResult sCoedgeResult = sCoedgeBuilder.AddSourceCoedgeForCurrentTraversal(sLoopData, sCoedgeData);
            uiLoopFirstEdgeuse = std::min(uiLoopFirstEdgeuse, sCoedgeResult.uiEdgeuseTargetIndex);

            uint32_t uiEdgeIndex = 0;
            if (!StageEdgeForCoedge(rCoedge, sCoedgeResult.uiEdgeuseTargetIndex, uiEdgeIndex, rError))
            {
                return false;
            }

            MakeTopologyBuilder().RegisterStagedEdgeuseForEdge(sCoedgeResult.uiEdgeuseTargetIndex, uiEdgeIndex);

            StageEdgeuseUvCurve(rCoedge, sCoedgeResult.uiEdgeuseTargetIndex, sCoedgeResult.bStagedEdgeuseMatchesSourceOrient, sUvCtx);
        }
        const uint32_t uiLoopEdgeuseCount = static_cast<uint32_t>(rLoop.vCoedges.size());
        if (sUvCtx.bParameterizationCompatible && uiLoopFirstEdgeuse != kStagedNoObjectIndex &&
            StagedUvLoopHasCompleteCoverage(m_rState, uiLoopFirstEdgeuse, uiLoopEdgeuseCount) &&
            !StagedUvLoopCloses(m_rState, uiLoopFirstEdgeuse, uiLoopEdgeuseCount))
        {
            // Every curve is present but the loop is internally inconsistent, so there is no
            // reliable way to identify a single bad record. Reject this loop without affecting
            // valid UV curves on other loops or faces.
            for (uint32_t ii = 0; ii < uiLoopEdgeuseCount; ++ii)
            {
                m_rState.SetEdgeuseCurve2d(uiLoopFirstEdgeuse + ii, StagedCurveData2d{});
            }
        }
    }

    return true;
}

uint32_t OcctStagingDriver::StageVertex(int iVertexStorageIndex, const pxr::GfMatrix4d& rVertexLoc)
{
    const std::string sKey = MakeInstanceKey(iVertexStorageIndex, rVertexLoc);
    const auto sIt = m_mVertexIndexByKey.find(sKey);
    if (sIt != m_mVertexIndexByKey.end())
    {
        return sIt->second;
    }

    uint32_t uiVertexIndex = 0;
    SourceVertexData sVertexData;
    sVertexData.sPoint = rVertexLoc.Transform(m_rSource.VertexPoint(iVertexStorageIndex));
    sVertexData.bHasPoint = true;
    StagedBrepEdgeVertexBuilder(m_rState).AddSourceVertex(sVertexData, uiVertexIndex);

    m_mVertexIndexByKey.emplace(sKey, uiVertexIndex);
    return uiVertexIndex;
}

uint32_t OcctStagingDriver::FindOrAddVertexAtPoint(const pxr::GfVec3d& rPoint)
{
    const double dTol = std::max(1.0e-7, 10.0 * m_rState.GetTolerance());
    for (uint32_t ii = 0; ii < m_rState.GetVertexPointCount(); ++ii)
    {
        const pxr::GfVec3d* pExisting = m_rState.GetVertexPointRecord(ii);
        if (pExisting && (*pExisting - rPoint).GetLength() <= dTol)
        {
            return ii;
        }
    }

    SourceVertexData sVertexData;
    sVertexData.sPoint = rPoint;
    sVertexData.bHasPoint = true;
    uint32_t uiVertexIndex = 0;
    StagedBrepEdgeVertexBuilder(m_rState).AddSourceVertex(sVertexData, uiVertexIndex);
    return uiVertexIndex;
}

bool OcctStagingDriver::TryBuildPeriodicBoundaryRepair(
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurfaceData,
    const pxr::GfRange2d& rPCurveBox,
    PeriodicBoundaryRepair& rRepair
)
{
    return TryBuildAnalyticSinglyPeriodicRepair(rFace, rSurfaceData, rPCurveBox, rRepair) ||
           TryBuildNurbsSinglyPeriodicRepair(rFace, rSurfaceData, rPCurveBox, rRepair) ||
           TryBuildDoublyPeriodicBoundaryRepair(rFace, rSurfaceData, rPCurveBox, rRepair);
}

bool OcctStagingDriver::TryBuildAnalyticSinglyPeriodicRepair(
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurfaceData,
    const pxr::GfRange2d& rPCurveBox,
    PeriodicBoundaryRepair& rRepair
)
{
    rRepair = PeriodicBoundaryRepair{};
    if (!rPCurveBox.IsEmpty() || rFace.bNaturalRestriction || !IsSinglyPeriodicRepairSurface(rSurfaceData.eSurfaceType))
    {
        return false;
    }

    struct RingLoop
    {
        uint32_t uiLoopIndex = 0;
        OcctFileSource::Coedge sCoedge;
        uint32_t uiVertexIndex = kStagedNoObjectIndex;
        int iVertexStorageIndex = 0;
        pxr::GfMatrix4d sVertexLoc = pxr::GfMatrix4d(1.0);
        pxr::GfVec3d sVertexPoint;
        double dU = 0.0;
        double dV = 0.0;
        bool bSourceIncreasesU = true;
        bool bUseMatchesSourceEdge = true;
    };

    auto fAnalyzeRing = [&](uint32_t uiLoopIndex, const OcctFileSource::Loop& rLoop, RingLoop& rRing) -> bool
    {
        if (rLoop.vCoedges.size() != 1)
        {
            return false;
        }

        const OcctFileSource::Coedge& rCoedge = rLoop.vCoedges.front();
        const OcctFileSource::EdgeGeometry sGeom = m_rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
        if (sGeom.bDegenerated || !sGeom.pCurve3d || sGeom.iStartVertexStorageIndex == 0 ||
            sGeom.iStartVertexStorageIndex != sGeom.iEndVertexStorageIndex || !IsFullPeriodInterval(sGeom.dFirst, sGeom.dLast))
        {
            return false;
        }

        StagedCurveData sCurve;
        const pxr::GfMatrix4d sCurveXf = ResolveLocation(m_rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
        pxr::GfRange1d sInterval(sGeom.dFirst, sGeom.dLast);
        if (!ConvertCurve3d(*sGeom.pCurve3d, sCurveXf, sCurve, &sInterval) || sCurve.eCurveType != StagedCurveType::Circle)
        {
            return false;
        }
        pxr::GfVec3d sCurveAxis = sCurve.sAxis;
        pxr::GfVec3d sSurfaceAxis = rSurfaceData.sAxis;
        if (sCurveAxis.Normalize() == 0.0 || sSurfaceAxis.Normalize() == 0.0 || std::abs(pxr::GfDot(sCurveAxis, sSurfaceAxis)) < 1.0 - 1.0e-5)
        {
            return false;
        }

        const pxr::GfMatrix4d sVertexLoc = ResolveLocation(m_rSource.Model(), sGeom.iStartVertexLocation) * rCoedge.mEdgeLoc;
        const pxr::GfVec3d sVertexPoint = sVertexLoc.Transform(m_rSource.VertexPoint(sGeom.iStartVertexStorageIndex));
        double dU = 0.0;
        double dV = 0.0;
        if (!SurfaceParamAtPoint(rSurfaceData, sVertexPoint, dU, dV))
        {
            return false;
        }

        const double dSpan = sGeom.dLast - sGeom.dFirst;
        const double dStep = ((dSpan >= 0.0) ? 1.0 : -1.0) * std::min(std::abs(dSpan) * 1.0e-3, 1.0e-3);
        const pxr::GfVec3d sP0 = EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, sGeom.dFirst);
        const pxr::GfVec3d sP1 = EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, sGeom.dFirst + dStep);
        double dU0 = 0.0, dV0 = 0.0, dU1 = 0.0, dV1 = 0.0;
        if (!SurfaceParamAtPoint(rSurfaceData, sP0, dU0, dV0) || !SurfaceParamAtPoint(rSurfaceData, sP1, dU1, dV1))
        {
            return false;
        }
        dU1 = UnwrapAngleNear(dU1, dU0);
        if (std::abs(dU1 - dU0) <= 1.0e-9)
        {
            return false;
        }

        const bool bReverseLoop = !rLoop.bSameOrientationWithSurface;
        const bool bUseMatchesSourceEdge = (!bReverseLoop && rCoedge.bSameOrientationWithLoop) || (bReverseLoop && !rCoedge.bSameOrientationWithLoop);

        rRing.uiLoopIndex = uiLoopIndex;
        rRing.sCoedge = rCoedge;
        rRing.iVertexStorageIndex = sGeom.iStartVertexStorageIndex;
        rRing.sVertexLoc = sVertexLoc;
        rRing.sVertexPoint = sVertexPoint;
        rRing.dU = dU;
        rRing.dV = dV;
        rRing.bSourceIncreasesU = (dU1 > dU0);
        rRing.bUseMatchesSourceEdge = bUseMatchesSourceEdge;
        return true;
    };

    std::vector<RingLoop> vRings;
    std::vector<uint32_t> vNonRingLoops;
    for (uint32_t ii = 0; ii < static_cast<uint32_t>(rFace.vLoops.size()); ++ii)
    {
        RingLoop sRing;
        if (fAnalyzeRing(ii, rFace.vLoops[ii], sRing))
        {
            vRings.push_back(sRing);
        }
        else
        {
            vNonRingLoops.push_back(ii);
        }
    }
    if (vRings.empty())
    {
        return false;
    }

    auto fMakeSourceCoedge = [](const OcctFileSource::Coedge& rCoedge, bool bSame) -> PeriodicBoundaryCoedge
    {
        PeriodicBoundaryCoedge sResult;
        sResult.bSynthetic = false;
        sResult.sSourceCoedge = rCoedge;
        sResult.bSameOrientationWithEdge = bSame;
        return sResult;
    };
    uint32_t uiNextSyntheticGroup = 0;
    auto fMakeSyntheticCoedge =
        [](const StagedCurveData& rCurve, const pxr::GfRange1d& rInterval, uint32_t uiStart, uint32_t uiEnd, uint32_t uiGroup, bool bSame)
    {
        PeriodicBoundaryCoedge sResult;
        sResult.bSynthetic = true;
        sResult.sSyntheticCurve = rCurve;
        sResult.sSyntheticInterval = rInterval;
        sResult.uiSyntheticStartVertexIndex = uiStart;
        sResult.uiSyntheticEndVertexIndex = uiEnd;
        sResult.uiSyntheticGroupIndex = uiGroup;
        sResult.bSameOrientationWithEdge = bSame;
        return sResult;
    };
    auto fAppendSourceLoop = [&](const OcctFileSource::Loop& rLoop)
    {
        PeriodicBoundaryLoop sLoop;
        const bool bReverseLoop = !rLoop.bSameOrientationWithSurface;
        for (size_t ii = 0; ii < rLoop.vCoedges.size(); ++ii)
        {
            const size_t uiSourceIndex = bReverseLoop ? (rLoop.vCoedges.size() - 1 - ii) : ii;
            const OcctFileSource::Coedge& rCoedge = rLoop.vCoedges[uiSourceIndex];
            const bool bSame = (!bReverseLoop && rCoedge.bSameOrientationWithLoop) || (bReverseLoop && !rCoedge.bSameOrientationWithLoop);
            sLoop.vCoedges.push_back(fMakeSourceCoedge(rCoedge, bSame));
        }
        rRepair.vLoops.push_back(std::move(sLoop));
    };
    auto fStageRingVertex = [&](RingLoop& rRing) -> uint32_t
    {
        if (rRing.uiVertexIndex == kStagedNoObjectIndex)
        {
            rRing.uiVertexIndex = StageVertex(rRing.iVertexStorageIndex, rRing.sVertexLoc);
        }
        return rRing.uiVertexIndex;
    };

    const double dTwoPi = 2.0 * M_PI;
    if (vRings.size() == 1)
    {
        RingLoop& rRing = vRings.front();
        const double dUseSign = (rRing.bSourceIncreasesU ? 1.0 : -1.0) * (rRing.bUseMatchesSourceEdge ? 1.0 : -1.0);
        const bool bInteriorHighSide = dUseSign > 0.0;
        bool bHavePole = false;
        double dPoleV = 0.0;
        pxr::GfVec3d sPolePoint;

        if (rSurfaceData.eSurfaceType == StagedSurfaceType::Sphere)
        {
            dPoleV = bInteriorHighSide ? (0.5 * M_PI) : (-0.5 * M_PI);
            sPolePoint = SurfacePointAtParam(rSurfaceData, rRing.dU, dPoleV);
            bHavePole = true;
        }
        else if (rSurfaceData.eSurfaceType == StagedSurfaceType::Cone)
        {
            const double dTan = std::tan(rSurfaceData.dSemiAngle);
            if (std::abs(dTan) > 1.0e-12)
            {
                const double dCandidatePoleV = -rSurfaceData.dRadius / dTan;
                if ((bInteriorHighSide && dCandidatePoleV > rRing.dV) || (!bInteriorHighSide && dCandidatePoleV < rRing.dV))
                {
                    dPoleV = dCandidatePoleV;
                    sPolePoint = SurfacePointAtParam(rSurfaceData, rRing.dU, dPoleV);
                    bHavePole = true;
                }
            }
        }
        if (!bHavePole)
        {
            return false;
        }

        StagedCurveData sSeamCurve;
        pxr::GfRange1d sSeamInterval;
        if (!BuildIsoSeamCurve(rSurfaceData, rRing.dU, dPoleV, rRing.dV, sSeamCurve, sSeamInterval))
        {
            return false;
        }

        const uint32_t uiPoleVertex = FindOrAddVertexAtPoint(sPolePoint);
        const uint32_t uiRingVertex = fStageRingVertex(rRing);
        const uint32_t uiSeamGroup = uiNextSyntheticGroup++;
        PeriodicBoundaryLoop sOuter;
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiPoleVertex, uiRingVertex, uiSeamGroup, true));
        sOuter.vCoedges.push_back(fMakeSourceCoedge(rRing.sCoedge, rRing.bUseMatchesSourceEdge));
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiPoleVertex, uiRingVertex, uiSeamGroup, false));
        rRepair.vLoops.push_back(std::move(sOuter));

        for (uint32_t uiLoopIndex : vNonRingLoops)
        {
            fAppendSourceLoop(rFace.vLoops[uiLoopIndex]);
        }

        const double dVMin = std::min(dPoleV, rRing.dV);
        const double dVMax = std::max(dPoleV, rRing.dV);
        rRepair.sDomain = pxr::GfRange2d(pxr::GfVec2d(0.0, dVMin), pxr::GfVec2d(dTwoPi, dVMax));
        rRepair.bEnabled = true;
        return true;
    }

    if (vRings.size() == 2)
    {
        RingLoop sLow = vRings[0];
        RingLoop sHigh = vRings[1];
        if (sHigh.dV < sLow.dV)
        {
            std::swap(sLow, sHigh);
        }
        const double dHighUNearLow = UnwrapAngleNear(sHigh.dU, sLow.dU);
        if (std::abs(dHighUNearLow - sLow.dU) > 1.0e-5)
        {
            return false; // Ring seam vertices are not aligned; splitting source rings is a later increment.
        }

        StagedCurveData sSeamCurve;
        pxr::GfRange1d sSeamInterval;
        if (!BuildIsoSeamCurve(rSurfaceData, sLow.dU, sLow.dV, sHigh.dV, sSeamCurve, sSeamInterval))
        {
            return false;
        }

        PeriodicBoundaryLoop sOuter;
        const uint32_t uiSeamGroup = uiNextSyntheticGroup++;
        const uint32_t uiLowVertex = fStageRingVertex(sLow);
        const uint32_t uiHighVertex = fStageRingVertex(sHigh);
        sOuter.vCoedges.push_back(fMakeSourceCoedge(sLow.sCoedge, sLow.bSourceIncreasesU));
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiLowVertex, uiHighVertex, uiSeamGroup, true));
        sOuter.vCoedges.push_back(fMakeSourceCoedge(sHigh.sCoedge, !sHigh.bSourceIncreasesU));
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiLowVertex, uiHighVertex, uiSeamGroup, false));
        rRepair.vLoops.push_back(std::move(sOuter));

        for (uint32_t uiLoopIndex : vNonRingLoops)
        {
            fAppendSourceLoop(rFace.vLoops[uiLoopIndex]);
        }

        rRepair.sDomain = pxr::GfRange2d(pxr::GfVec2d(0.0, sLow.dV), pxr::GfVec2d(dTwoPi, sHigh.dV));
        rRepair.bEnabled = true;
        return true;
    }

    return false;
}

bool OcctStagingDriver::TryBuildNurbsSinglyPeriodicRepair(
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurfaceData,
    const pxr::GfRange2d& rPCurveBox,
    PeriodicBoundaryRepair& rRepair
)
{
    rRepair = PeriodicBoundaryRepair{};
    if (!rPCurveBox.IsEmpty() || rFace.bNaturalRestriction || !IsValidNurbsSurfaceGrid(rSurfaceData))
    {
        return false;
    }

    const bool bClosedU = NurbsClosedInU(rSurfaceData);
    const bool bClosedV = NurbsClosedInV(rSurfaceData);
    if (bClosedU == bClosedV)
    {
        return false;
    }

    struct NurbsRing
    {
        OcctFileSource::Coedge sCoedge;
        uint32_t uiVertexIndex = kStagedNoObjectIndex;
        int iVertexStorageIndex = 0;
        pxr::GfMatrix4d sVertexLoc = pxr::GfMatrix4d(1.0);
        uint32_t uiTransverseIndex = 0;
        double dTransverseParam = 0.0;
        bool bSourceIncreasesPeriodic = true;
        bool bUseMatchesSourceEdge = true;
    };

    const uint32_t uCount = rSurfaceData.uiNurbUVertexCount;
    const uint32_t vCount = rSurfaceData.uiNurbVVertexCount;
    const uint32_t uiLowIndex = 0;
    const uint32_t uiHighIndex = bClosedU ? (vCount - 1) : (uCount - 1);
    const double dLowParam = bClosedU ? rSurfaceData.vNurbVKnots.front() : rSurfaceData.vNurbUKnots.front();
    const double dHighParam = bClosedU ? rSurfaceData.vNurbVKnots.back() : rSurfaceData.vNurbUKnots.back();
    const bool bLowCollapsed = bClosedU ? NurbsURowCollapsed(rSurfaceData, uiLowIndex) : NurbsVColumnCollapsed(rSurfaceData, uiLowIndex);
    const bool bHighCollapsed = bClosedU ? NurbsURowCollapsed(rSurfaceData, uiHighIndex) : NurbsVColumnCollapsed(rSurfaceData, uiHighIndex);

    auto fMakeSourceCoedge = [](const OcctFileSource::Coedge& rCoedge, bool bSame) -> PeriodicBoundaryCoedge
    {
        PeriodicBoundaryCoedge sResult;
        sResult.bSynthetic = false;
        sResult.sSourceCoedge = rCoedge;
        sResult.bSameOrientationWithEdge = bSame;
        return sResult;
    };
    uint32_t uiNextSyntheticGroup = 0;
    auto fMakeSyntheticCoedge =
        [](const StagedCurveData& rCurve, const pxr::GfRange1d& rInterval, uint32_t uiStart, uint32_t uiEnd, uint32_t uiGroup, bool bSame)
    {
        PeriodicBoundaryCoedge sResult;
        sResult.bSynthetic = true;
        sResult.sSyntheticCurve = rCurve;
        sResult.sSyntheticInterval = rInterval;
        sResult.uiSyntheticStartVertexIndex = uiStart;
        sResult.uiSyntheticEndVertexIndex = uiEnd;
        sResult.uiSyntheticGroupIndex = uiGroup;
        sResult.bSameOrientationWithEdge = bSame;
        return sResult;
    };
    auto fAppendSourceLoop = [&](const OcctFileSource::Loop& rLoop)
    {
        PeriodicBoundaryLoop sLoop;
        const bool bReverseLoop = !rLoop.bSameOrientationWithSurface;
        for (size_t ii = 0; ii < rLoop.vCoedges.size(); ++ii)
        {
            const size_t uiSourceIndex = bReverseLoop ? (rLoop.vCoedges.size() - 1 - ii) : ii;
            const OcctFileSource::Coedge& rCoedge = rLoop.vCoedges[uiSourceIndex];
            const bool bSame = (!bReverseLoop && rCoedge.bSameOrientationWithLoop) || (bReverseLoop && !rCoedge.bSameOrientationWithLoop);
            sLoop.vCoedges.push_back(fMakeSourceCoedge(rCoedge, bSame));
        }
        rRepair.vLoops.push_back(std::move(sLoop));
    };
    auto fStageRingVertex = [&](NurbsRing& rRing) -> uint32_t
    {
        if (rRing.uiVertexIndex == kStagedNoObjectIndex)
        {
            rRing.uiVertexIndex = StageVertex(rRing.iVertexStorageIndex, rRing.sVertexLoc);
        }
        return rRing.uiVertexIndex;
    };
    auto fAnalyzeRing = [&](const OcctFileSource::Loop& rLoop, NurbsRing& rRing) -> bool
    {
        if (rLoop.vCoedges.size() != 1)
        {
            return false;
        }
        const OcctFileSource::Coedge& rCoedge = rLoop.vCoedges.front();
        const OcctFileSource::EdgeGeometry sGeom = m_rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
        if (sGeom.bDegenerated || !sGeom.pCurve3d || sGeom.iStartVertexStorageIndex == 0 ||
            sGeom.iStartVertexStorageIndex != sGeom.iEndVertexStorageIndex)
        {
            return false;
        }

        const pxr::GfMatrix4d sCurveXf = ResolveLocation(m_rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
        const bool bRingAlongU = bClosedU;
        for (uint32_t uiCandidate : { uiLowIndex, uiHighIndex })
        {
            StagedCurveData sIsoCurve;
            pxr::GfRange1d sIsoInterval;
            if (!BuildNurbsIsoCurve(rSurfaceData, bRingAlongU, uiCandidate, false, sIsoCurve, sIsoInterval))
            {
                continue;
            }

            bool bSourceIncreases = false;
            if (SourceEdgeMatchesStagedCurve(*sGeom.pCurve3d, sCurveXf, sGeom.dFirst, sGeom.dLast, sIsoCurve, sIsoInterval, false))
            {
                bSourceIncreases = true;
            }
            else if (SourceEdgeMatchesStagedCurve(*sGeom.pCurve3d, sCurveXf, sGeom.dFirst, sGeom.dLast, sIsoCurve, sIsoInterval, true))
            {
                bSourceIncreases = false;
            }
            else
            {
                continue;
            }

            const bool bReverseLoop = !rLoop.bSameOrientationWithSurface;
            const bool bUseMatchesSourceEdge = (!bReverseLoop && rCoedge.bSameOrientationWithLoop) ||
                                               (bReverseLoop && !rCoedge.bSameOrientationWithLoop);
            rRing.sCoedge = rCoedge;
            rRing.iVertexStorageIndex = sGeom.iStartVertexStorageIndex;
            rRing.sVertexLoc = ResolveLocation(m_rSource.Model(), sGeom.iStartVertexLocation) * rCoedge.mEdgeLoc;
            rRing.uiTransverseIndex = uiCandidate;
            rRing.dTransverseParam = (uiCandidate == uiLowIndex) ? dLowParam : dHighParam;
            rRing.bSourceIncreasesPeriodic = bSourceIncreases;
            rRing.bUseMatchesSourceEdge = bUseMatchesSourceEdge;
            return true;
        }
        return false;
    };
    auto fBuildSeamCurve = [&](bool bReverse, StagedCurveData& rCurve, pxr::GfRange1d& rInterval) -> bool
    {
        const bool bSeamAlongU = bClosedV;
        return BuildNurbsIsoCurve(rSurfaceData, bSeamAlongU, 0, bReverse, rCurve, rInterval);
    };
    auto fBoundaryPoint = [&](uint32_t uiTransverseIndex) -> pxr::GfVec3d
    {
        return bClosedU ? NurbsPointAt(rSurfaceData, 0, uiTransverseIndex) : NurbsPointAt(rSurfaceData, uiTransverseIndex, 0);
    };

    std::vector<NurbsRing> vRings;
    std::vector<uint32_t> vNonRingLoops;
    for (uint32_t ii = 0; ii < static_cast<uint32_t>(rFace.vLoops.size()); ++ii)
    {
        NurbsRing sRing;
        if (fAnalyzeRing(rFace.vLoops[ii], sRing))
        {
            vRings.push_back(sRing);
        }
        else
        {
            vNonRingLoops.push_back(ii);
        }
    }
    if (vRings.empty())
    {
        return false;
    }

    if (vRings.size() == 1)
    {
        NurbsRing& rRing = vRings.front();
        const bool bRingAtLow = rRing.uiTransverseIndex == uiLowIndex;
        const bool bPoleAtLow = bLowCollapsed && !bRingAtLow;
        const bool bPoleAtHigh = bHighCollapsed && bRingAtLow;
        if (!bPoleAtLow && !bPoleAtHigh)
        {
            return false;
        }

        const uint32_t uiPoleTransverseIndex = bPoleAtLow ? uiLowIndex : uiHighIndex;
        const bool bReverseSeam = uiPoleTransverseIndex > rRing.uiTransverseIndex;
        StagedCurveData sSeamCurve;
        pxr::GfRange1d sSeamInterval;
        if (!fBuildSeamCurve(bReverseSeam, sSeamCurve, sSeamInterval))
        {
            return false;
        }

        const uint32_t uiPoleVertex = FindOrAddVertexAtPoint(fBoundaryPoint(uiPoleTransverseIndex));
        const uint32_t uiRingVertex = fStageRingVertex(rRing);
        const uint32_t uiSeamGroup = uiNextSyntheticGroup++;
        PeriodicBoundaryLoop sOuter;
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiPoleVertex, uiRingVertex, uiSeamGroup, true));
        sOuter.vCoedges.push_back(fMakeSourceCoedge(rRing.sCoedge, rRing.bUseMatchesSourceEdge));
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiPoleVertex, uiRingVertex, uiSeamGroup, false));
        rRepair.vLoops.push_back(std::move(sOuter));

        for (uint32_t uiLoopIndex : vNonRingLoops)
        {
            fAppendSourceLoop(rFace.vLoops[uiLoopIndex]);
        }

        const double dMin = std::min(bPoleAtLow ? dLowParam : dHighParam, rRing.dTransverseParam);
        const double dMax = std::max(bPoleAtLow ? dLowParam : dHighParam, rRing.dTransverseParam);
        rRepair
            .sDomain = bClosedU ?
                           pxr::GfRange2d(pxr::GfVec2d(rSurfaceData.vNurbUKnots.front(), dMin), pxr::GfVec2d(rSurfaceData.vNurbUKnots.back(), dMax)) :
                           pxr::GfRange2d(pxr::GfVec2d(dMin, rSurfaceData.vNurbVKnots.front()), pxr::GfVec2d(dMax, rSurfaceData.vNurbVKnots.back()));
        rRepair.bEnabled = true;
        return true;
    }

    if (vRings.size() == 2)
    {
        NurbsRing sLow = vRings[0];
        NurbsRing sHigh = vRings[1];
        if (sHigh.dTransverseParam < sLow.dTransverseParam)
        {
            std::swap(sLow, sHigh);
        }
        if (sLow.uiTransverseIndex == sHigh.uiTransverseIndex)
        {
            return false;
        }

        StagedCurveData sSeamCurve;
        pxr::GfRange1d sSeamInterval;
        if (!fBuildSeamCurve(false, sSeamCurve, sSeamInterval))
        {
            return false;
        }

        const uint32_t uiLowVertex = fStageRingVertex(sLow);
        const uint32_t uiHighVertex = fStageRingVertex(sHigh);
        const uint32_t uiSeamGroup = uiNextSyntheticGroup++;
        PeriodicBoundaryLoop sOuter;
        sOuter.vCoedges.push_back(fMakeSourceCoedge(sLow.sCoedge, sLow.bSourceIncreasesPeriodic));
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiLowVertex, uiHighVertex, uiSeamGroup, true));
        sOuter.vCoedges.push_back(fMakeSourceCoedge(sHigh.sCoedge, !sHigh.bSourceIncreasesPeriodic));
        sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiLowVertex, uiHighVertex, uiSeamGroup, false));
        rRepair.vLoops.push_back(std::move(sOuter));

        for (uint32_t uiLoopIndex : vNonRingLoops)
        {
            fAppendSourceLoop(rFace.vLoops[uiLoopIndex]);
        }

        rRepair.sDomain = bClosedU ? pxr::GfRange2d(
                                         pxr::GfVec2d(rSurfaceData.vNurbUKnots.front(), sLow.dTransverseParam),
                                         pxr::GfVec2d(rSurfaceData.vNurbUKnots.back(), sHigh.dTransverseParam)
                                     ) :
                                     pxr::GfRange2d(
                                         pxr::GfVec2d(sLow.dTransverseParam, rSurfaceData.vNurbVKnots.front()),
                                         pxr::GfVec2d(sHigh.dTransverseParam, rSurfaceData.vNurbVKnots.back())
                                     );
        rRepair.bEnabled = true;
        return true;
    }

    return false;
}

bool OcctStagingDriver::TryBuildDoublyPeriodicBoundaryRepair(
    const OcctFileSource::Face& rFace,
    const StagedSurfaceData& rSurfaceData,
    const pxr::GfRange2d& rPCurveBox,
    PeriodicBoundaryRepair& rRepair
)
{
    rRepair = PeriodicBoundaryRepair{};
    if (!rPCurveBox.IsEmpty() || rFace.bNaturalRestriction || rSurfaceData.eSurfaceType != StagedSurfaceType::Torus)
    {
        return false;
    }

    enum class TorusRingDirection
    {
        AlongU,
        AlongV
    };

    struct TorusRing
    {
        OcctFileSource::Coedge sCoedge;
        TorusRingDirection eDirection = TorusRingDirection::AlongU;
        uint32_t uiVertexIndex = kStagedNoObjectIndex;
        int iVertexStorageIndex = 0;
        pxr::GfMatrix4d sVertexLoc = pxr::GfMatrix4d(1.0);
        double dU = 0.0;
        double dV = 0.0;
        bool bSourceIncreasesPeriodic = true;
    };

    auto fMakeSourceCoedge = [](const OcctFileSource::Coedge& rCoedge, bool bSame) -> PeriodicBoundaryCoedge
    {
        PeriodicBoundaryCoedge sResult;
        sResult.bSynthetic = false;
        sResult.sSourceCoedge = rCoedge;
        sResult.bSameOrientationWithEdge = bSame;
        return sResult;
    };
    uint32_t uiNextSyntheticGroup = 0;
    auto fMakeSyntheticCoedge =
        [](const StagedCurveData& rCurve, const pxr::GfRange1d& rInterval, uint32_t uiStart, uint32_t uiEnd, uint32_t uiGroup, bool bSame)
    {
        PeriodicBoundaryCoedge sResult;
        sResult.bSynthetic = true;
        sResult.sSyntheticCurve = rCurve;
        sResult.sSyntheticInterval = rInterval;
        sResult.uiSyntheticStartVertexIndex = uiStart;
        sResult.uiSyntheticEndVertexIndex = uiEnd;
        sResult.uiSyntheticGroupIndex = uiGroup;
        sResult.bSameOrientationWithEdge = bSame;
        return sResult;
    };
    auto fAppendSourceLoop = [&](const OcctFileSource::Loop& rLoop)
    {
        PeriodicBoundaryLoop sLoop;
        const bool bReverseLoop = !rLoop.bSameOrientationWithSurface;
        for (size_t ii = 0; ii < rLoop.vCoedges.size(); ++ii)
        {
            const size_t uiSourceIndex = bReverseLoop ? (rLoop.vCoedges.size() - 1 - ii) : ii;
            const OcctFileSource::Coedge& rCoedge = rLoop.vCoedges[uiSourceIndex];
            const bool bSame = (!bReverseLoop && rCoedge.bSameOrientationWithLoop) || (bReverseLoop && !rCoedge.bSameOrientationWithLoop);
            sLoop.vCoedges.push_back(fMakeSourceCoedge(rCoedge, bSame));
        }
        rRepair.vLoops.push_back(std::move(sLoop));
    };
    auto fStageRingVertex = [&](TorusRing& rRing) -> uint32_t
    {
        if (rRing.uiVertexIndex == kStagedNoObjectIndex)
        {
            rRing.uiVertexIndex = StageVertex(rRing.iVertexStorageIndex, rRing.sVertexLoc);
        }
        return rRing.uiVertexIndex;
    };
    auto fAnalyzeRing = [&](const OcctFileSource::Loop& rLoop, TorusRing& rRing) -> bool
    {
        if (rLoop.vCoedges.size() != 1)
        {
            return false;
        }

        const OcctFileSource::Coedge& rCoedge = rLoop.vCoedges.front();
        const OcctFileSource::EdgeGeometry sGeom = m_rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);
        if (sGeom.bDegenerated || !sGeom.pCurve3d || sGeom.iStartVertexStorageIndex == 0 ||
            sGeom.iStartVertexStorageIndex != sGeom.iEndVertexStorageIndex || !IsFullPeriodInterval(sGeom.dFirst, sGeom.dLast))
        {
            return false;
        }

        StagedCurveData sCurve;
        const pxr::GfMatrix4d sCurveXf = ResolveLocation(m_rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
        pxr::GfRange1d sInterval(sGeom.dFirst, sGeom.dLast);
        if (!ConvertCurve3d(*sGeom.pCurve3d, sCurveXf, sCurve, &sInterval) || sCurve.eCurveType != StagedCurveType::Circle)
        {
            return false;
        }

        pxr::GfVec3d sCurveAxis = sCurve.sAxis;
        pxr::GfVec3d sSurfaceAxis = rSurfaceData.sAxis;
        if (sCurveAxis.Normalize() == 0.0 || sSurfaceAxis.Normalize() == 0.0)
        {
            return false;
        }
        const double dAxisDot = std::abs(pxr::GfDot(sCurveAxis, sSurfaceAxis));

        const pxr::GfMatrix4d sVertexLoc = ResolveLocation(m_rSource.Model(), sGeom.iStartVertexLocation) * rCoedge.mEdgeLoc;
        const pxr::GfVec3d sVertexPoint = sVertexLoc.Transform(m_rSource.VertexPoint(sGeom.iStartVertexStorageIndex));
        double dU = 0.0;
        double dV = 0.0;
        if (!SurfaceParamAtPoint(rSurfaceData, sVertexPoint, dU, dV))
        {
            return false;
        }

        const double dSpan = sGeom.dLast - sGeom.dFirst;
        const double dStep = ((dSpan >= 0.0) ? 1.0 : -1.0) * std::min(std::abs(dSpan) * 1.0e-3, 1.0e-3);
        const pxr::GfVec3d sP0 = EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, sGeom.dFirst);
        const pxr::GfVec3d sP1 = EvalCurve3dPoint(*sGeom.pCurve3d, sCurveXf, sGeom.dFirst + dStep);
        double dU0 = 0.0, dV0 = 0.0, dU1 = 0.0, dV1 = 0.0;
        if (!SurfaceParamAtPoint(rSurfaceData, sP0, dU0, dV0) || !SurfaceParamAtPoint(rSurfaceData, sP1, dU1, dV1))
        {
            return false;
        }

        const double dTol = std::max(1.0e-6, 1.0e-6 * std::max({ rSurfaceData.dMajorRadius, rSurfaceData.dMinorRadius, 1.0 }));
        const bool bMajorRing = dAxisDot > 1.0 - 1.0e-5;
        const bool bMinorRing = dAxisDot < 1.0e-5 && std::abs(sCurve.dRadius - rSurfaceData.dMinorRadius) <= dTol;
        if (bMajorRing)
        {
            dU1 = UnwrapAngleNear(dU1, dU0);
            if (std::abs(dU1 - dU0) <= 1.0e-9)
            {
                return false;
            }
            rRing.eDirection = TorusRingDirection::AlongU;
            rRing.bSourceIncreasesPeriodic = dU1 > dU0;
        }
        else if (bMinorRing)
        {
            dV1 = UnwrapAngleNear(dV1, dV0);
            if (std::abs(dV1 - dV0) <= 1.0e-9)
            {
                return false;
            }
            rRing.eDirection = TorusRingDirection::AlongV;
            rRing.bSourceIncreasesPeriodic = dV1 > dV0;
        }
        else
        {
            return false;
        }

        rRing.sCoedge = rCoedge;
        rRing.iVertexStorageIndex = sGeom.iStartVertexStorageIndex;
        rRing.sVertexLoc = sVertexLoc;
        rRing.dU = dU;
        rRing.dV = dV;
        return true;
    };

    std::vector<TorusRing> vRings;
    std::vector<uint32_t> vNonRingLoops;
    for (uint32_t ii = 0; ii < static_cast<uint32_t>(rFace.vLoops.size()); ++ii)
    {
        TorusRing sRing;
        if (fAnalyzeRing(rFace.vLoops[ii], sRing))
        {
            vRings.push_back(sRing);
        }
        else
        {
            vNonRingLoops.push_back(ii);
        }
    }
    if (vRings.size() != 2 || vRings[0].eDirection != vRings[1].eDirection)
    {
        return false;
    }

    const double dTwoPi = 2.0 * M_PI;
    TorusRing sLow = vRings[0];
    TorusRing sHigh = vRings[1];
    StagedCurveData sSeamCurve;
    pxr::GfRange1d sSeamInterval;

    if (sLow.eDirection == TorusRingDirection::AlongU)
    {
        const double dHighUNearLow = UnwrapAngleNear(sHigh.dU, sLow.dU);
        if (std::abs(dHighUNearLow - sLow.dU) > 1.0e-5)
        {
            return false;
        }
        if (sHigh.dV < sLow.dV)
        {
            std::swap(sLow, sHigh);
        }
        if (!BuildTorusIsoArc(rSurfaceData, /*bAlongU*/ false, sLow.dU, sLow.dV, sHigh.dV, sSeamCurve, sSeamInterval))
        {
            return false;
        }
        rRepair.sDomain = pxr::GfRange2d(pxr::GfVec2d(0.0, sLow.dV), pxr::GfVec2d(dTwoPi, sHigh.dV));
    }
    else
    {
        const double dHighVNearLow = UnwrapAngleNear(sHigh.dV, sLow.dV);
        if (std::abs(dHighVNearLow - sLow.dV) > 1.0e-5)
        {
            return false;
        }
        if (sHigh.dU < sLow.dU)
        {
            std::swap(sLow, sHigh);
        }
        if (!BuildTorusIsoArc(rSurfaceData, /*bAlongU*/ true, sLow.dV, sLow.dU, sHigh.dU, sSeamCurve, sSeamInterval))
        {
            return false;
        }
        rRepair.sDomain = pxr::GfRange2d(pxr::GfVec2d(sLow.dU, 0.0), pxr::GfVec2d(sHigh.dU, dTwoPi));
    }

    const uint32_t uiLowVertex = fStageRingVertex(sLow);
    const uint32_t uiHighVertex = fStageRingVertex(sHigh);
    const uint32_t uiSeamGroup = uiNextSyntheticGroup++;
    PeriodicBoundaryLoop sOuter;
    sOuter.vCoedges.push_back(fMakeSourceCoedge(sLow.sCoedge, sLow.bSourceIncreasesPeriodic));
    sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiLowVertex, uiHighVertex, uiSeamGroup, true));
    sOuter.vCoedges.push_back(fMakeSourceCoedge(sHigh.sCoedge, !sHigh.bSourceIncreasesPeriodic));
    sOuter.vCoedges.push_back(fMakeSyntheticCoedge(sSeamCurve, sSeamInterval, uiLowVertex, uiHighVertex, uiSeamGroup, false));
    rRepair.vLoops.push_back(std::move(sOuter));

    for (uint32_t uiLoopIndex : vNonRingLoops)
    {
        fAppendSourceLoop(rFace.vLoops[uiLoopIndex]);
    }

    rRepair.bEnabled = true;
    return true;
}

bool OcctStagingDriver::StagePeriodicBoundaryRepair(const PeriodicBoundaryRepair& rRepair, const FaceUvCurveContext& rUvContext, std::string& rError)
{
    GenericBrepStagingBuilder sGenericBuilder(m_rState);
    std::unordered_map<uint32_t, uint32_t> mSyntheticEdgeByGroup;

    for (uint32_t uiLoopIndex = 0; uiLoopIndex < static_cast<uint32_t>(rRepair.vLoops.size()); ++uiLoopIndex)
    {
        const PeriodicBoundaryLoop& rLoop = rRepair.vLoops[uiLoopIndex];
        StagedLoopData sLoopData;
        sLoopData.uiEdgeuseCount = static_cast<uint32_t>(rLoop.vCoedges.size());
        sLoopData.uiVertexIndex = 0;
        sGenericBuilder.AddLoopForCurrentTraversal(
            sLoopData,
            uiLoopIndex == rRepair.uiOuterLoopIndex,
            uiLoopIndex + 1,
            /*bHasStagedImpliedNaturalBoundary*/ false
        );

        const uint32_t uiEdgeuseStart = m_rState.GetEdgeuseCount();
        m_rState.ResizeEdgeuseRecords(uiEdgeuseStart + sLoopData.uiEdgeuseCount);

        std::unordered_map<uint32_t, std::vector<uint32_t>> mSyntheticEdgeuses;
        for (uint32_t ii = 0; ii < sLoopData.uiEdgeuseCount; ++ii)
        {
            const PeriodicBoundaryCoedge& rCoedge = rLoop.vCoedges[ii];
            const uint32_t uiEdgeuseIndex = uiEdgeuseStart + ii;

            StagedEdgeuseData sEdgeuseData;
            sEdgeuseData.eOrientType = OrientFromSame(rCoedge.bSameOrientationWithEdge);
            sEdgeuseData.eEntrySideType = EntryFromSame(rCoedge.bSameOrientationWithEdge);
            sEdgeuseData.uiNextRadialEdgeuseIndex = uiEdgeuseIndex;

            if (rCoedge.bSynthetic)
            {
                uint32_t uiEdgeIndex = rCoedge.uiSyntheticEdgeIndex;
                if (uiEdgeIndex == kStagedNoObjectIndex)
                {
                    const auto sExisting = mSyntheticEdgeByGroup.find(rCoedge.uiSyntheticGroupIndex);
                    if (sExisting != mSyntheticEdgeByGroup.end())
                    {
                        uiEdgeIndex = sExisting->second;
                    }
                }
                if (uiEdgeIndex == kStagedNoObjectIndex)
                {
                    m_rState.AppendEdgeCurveRecord(rCoedge.sSyntheticCurve);
                    StagedEdgeData sEdgeData;
                    sEdgeData.eCurveType = rCoedge.sSyntheticCurve.eCurveType;
                    sEdgeData.sInterval = rCoedge.sSyntheticInterval;
                    sEdgeData.uiStartVertexIndex = rCoedge.uiSyntheticStartVertexIndex;
                    sEdgeData.uiEndVertexIndex = rCoedge.uiSyntheticEndVertexIndex;
                    m_rState.AppendEdgeRecord(sEdgeData);
                    uiEdgeIndex = m_rState.GetEdgeCount() - 1;
                    mSyntheticEdgeByGroup[rCoedge.uiSyntheticGroupIndex] = uiEdgeIndex;
                }
                sEdgeuseData.uiEdgeIndex = uiEdgeIndex;
                if (StagedEdgeuseData* pEdgeuse = m_rState.GetEdgeuseRecord(uiEdgeuseIndex))
                {
                    *pEdgeuse = sEdgeuseData;
                }
                mSyntheticEdgeuses[uiEdgeIndex].push_back(uiEdgeuseIndex);
            }
            else
            {
                if (StagedEdgeuseData* pEdgeuse = m_rState.GetEdgeuseRecord(uiEdgeuseIndex))
                {
                    *pEdgeuse = sEdgeuseData;
                }

                uint32_t uiEdgeIndex = 0;
                if (!StageEdgeForCoedge(rCoedge.sSourceCoedge, uiEdgeuseIndex, uiEdgeIndex, rError))
                {
                    return false;
                }
                MakeTopologyBuilder().RegisterStagedEdgeuseForEdge(uiEdgeuseIndex, uiEdgeIndex);
                StageEdgeuseUvCurve(rCoedge.sSourceCoedge, uiEdgeuseIndex, rCoedge.bSameOrientationWithEdge, rUvContext);
            }
        }

        for (const auto& rEntry : mSyntheticEdgeuses)
        {
            const std::vector<uint32_t>& rEdgeuses = rEntry.second;
            if (rEdgeuses.empty())
            {
                continue;
            }
            if (rEdgeuses.size() == 1)
            {
                m_rState.SetEdgeRadialList(rEntry.first, rEdgeuses[0], rEdgeuses[0]);
                continue;
            }
            m_rState.SetEdgeRadialList(rEntry.first, rEdgeuses.front(), rEdgeuses.back());
            for (size_t ii = 0; ii < rEdgeuses.size(); ++ii)
            {
                const uint32_t uiCurrent = rEdgeuses[ii];
                const uint32_t uiNext = rEdgeuses[(ii + 1) % rEdgeuses.size()];
                if (StagedEdgeuseData* pEdgeuse = m_rState.GetEdgeuseRecord(uiCurrent))
                {
                    pEdgeuse->uiNextRadialEdgeuseIndex = uiNext;
                }
            }
        }
    }

    return true;
}

bool OcctStagingDriver::StageEdgeForCoedge(const OcctFileSource::Coedge& rCoedge, uint32_t uiEdgeuseIndex, uint32_t& rEdgeIndex, std::string& rError)
{
    const std::string sEdgeKey = MakeInstanceKey(rCoedge.iEdgeStorageIndex, rCoedge.mEdgeLoc);
    const auto sIt = m_mEdgeIndexByKey.find(sEdgeKey);
    if (sIt != m_mEdgeIndexByKey.end())
    {
        rEdgeIndex = sIt->second;
        return true;
    }

    rEdgeIndex = m_rState.GetEdgeCount();
    m_mEdgeIndexByKey.emplace(sEdgeKey, rEdgeIndex);
    m_rState.SetEdgeRadialList(rEdgeIndex, uiEdgeuseIndex, uiEdgeuseIndex);

    const OcctFileSource::EdgeGeometry sGeom = m_rSource.ResolveEdge(rCoedge.iEdgeStorageIndex);

    // Vertex world transforms compose the edge->vertex reference location onto the edge's location.
    const pxr::GfMatrix4d sStartVertexLoc = ResolveLocation(m_rSource.Model(), sGeom.iStartVertexLocation) * rCoedge.mEdgeLoc;
    const pxr::GfMatrix4d sEndVertexLoc = ResolveLocation(m_rSource.Model(), sGeom.iEndVertexLocation) * rCoedge.mEdgeLoc;
    const uint32_t uiStartVertexIndex = StageVertex(sGeom.iStartVertexStorageIndex, sStartVertexLoc);
    const uint32_t uiEndVertexIndex = StageVertex(sGeom.iEndVertexStorageIndex, sEndVertexLoc);
    const pxr::GfVec3d sStart = sStartVertexLoc.Transform(m_rSource.VertexPoint(sGeom.iStartVertexStorageIndex));
    const pxr::GfVec3d sEnd = sEndVertexLoc.Transform(m_rSource.VertexPoint(sGeom.iEndVertexStorageIndex));

    SourceEdgeData sEdgeData;
    sEdgeData.uiStartSourceVertexIndex = uiStartVertexIndex;
    sEdgeData.uiEndSourceVertexIndex = uiEndVertexIndex;
    sEdgeData.sCurve.sInterval = pxr::GfRange1d(sGeom.dFirst, sGeom.dLast);

    bool bCurveConverted = false;
    if (sGeom.pCurve3d)
    {
        // World curve transform = curve-representation location, then the edge's accumulated location.
        const pxr::GfMatrix4d sXf = ResolveLocation(m_rSource.Model(), sGeom.iCurveLocation) * rCoedge.mEdgeLoc;
        bCurveConverted = ConvertCurve3d(*sGeom.pCurve3d, sXf, sEdgeData.sCurve.sCurveData, &sEdgeData.sCurve.sInterval);
    }
    if (bCurveConverted && sEdgeData.sCurve.sCurveData.eCurveType == StagedCurveType::Line)
    {
        bCurveConverted = BuildLineCurveFromEndpoints(sStart, sEnd, sEdgeData.sCurve.sCurveData, sEdgeData.sCurve.sInterval);
    }
    if (!bCurveConverted)
    {
        // Fallback for degenerate/curve-less edges: a straight line between the two vertices.
        if (!BuildLineCurveFromEndpoints(sStart, sEnd, sEdgeData.sCurve.sCurveData, sEdgeData.sCurve.sInterval))
        {
            rError = "edge " + std::to_string(rCoedge.iEdgeStorageIndex) + ": invalid finite edge range";
            return false;
        }
    }

    // An analytic circle/ellipse schema represents at most one period. Preserve a multi-turn source
    // edge exactly by lowering that finite interval to a nonperiodic rational quadratic NURBS. One-
    // period and partial analytic edges stay analytic and are shifted into the primary interval.
    const StagedCurveType eCurveType = sEdgeData.sCurve.sCurveData.eCurveType;
    if (eCurveType == StagedCurveType::Circle || eCurveType == StagedCurveType::Ellipse)
    {
        const double dSpan = sEdgeData.sCurve.sInterval.GetMax() - sEdgeData.sCurve.sInterval.GetMin();
        if (dSpan > 2.0 * M_PI + 1.0e-5)
        {
            if (!LowerAngularCurveToNurbs(sEdgeData.sCurve.sCurveData, sEdgeData.sCurve.sInterval))
            {
                rError = "edge " + std::to_string(rCoedge.iEdgeStorageIndex) + ": failed to lower multi-turn angular curve";
                return false;
            }
        }
        else
        {
            double dLo = sEdgeData.sCurve.sInterval.GetMin();
            double dHi = sEdgeData.sCurve.sInterval.GetMax();
            NormalizeAngularInterval(dLo, dHi);
            sEdgeData.sCurve.sInterval = pxr::GfRange1d(dLo, dHi);
        }
    }
    if (!IsFiniteNonEmptyRange(sEdgeData.sCurve.sInterval))
    {
        rError = "edge " + std::to_string(rCoedge.iEdgeStorageIndex) + ": invalid finite edge range";
        return false;
    }

    pxr::GfRange1d sEdgeInterval = sEdgeData.sCurve.sInterval;
    StagedBrepEdgeVertexBuilder(m_rState).AddSourceEdgeForCurrentTraversal(sEdgeData, uiStartVertexIndex, uiEndVertexIndex, sEdgeInterval);

    return true;
}

void OcctStagingDriver::StageEdgeuseUvCurve(
    const OcctFileSource::Coedge& rCoedge,
    uint32_t uiEdgeuseIndex,
    bool bStagedEdgeuseMatchesSourceOrient,
    const FaceUvCurveContext& rContext
)
{
    if (!rContext.bParameterizationCompatible)
    {
        return;
    }

    // Degenerated pole coedges are demoted before staging reaches this point. A missing or unusable
    // source pcurve leaves this edgeuse's (0, 0) placeholder without discarding other UV curves.
    OcctFileSource::CoedgePCurve sPCurve;
    if (!m_rSource.ResolveCoedgePCurve(rCoedge, rContext.iSurface, sPCurve) || !sPCurve.pCurve2d)
    {
        return;
    }

    StagedCurveData2d sUvCurve;
    if (ConvertCurve2dToNurb(
            *sPCurve.pCurve2d,
            pxr::GfRange1d(sPCurve.dFirst, sPCurve.dLast),
            pxr::GfVec2d(rContext.dUScale, rContext.dVScale),
            pxr::GfVec2d(rContext.dUAdd, 0.0),
            sUvCurve
        ) &&
        sUvCurve.bValid && UvCurveControlPolygonFitsDomain(sUvCurve, rContext.sDomain))
    {
        if (!bStagedEdgeuseMatchesSourceOrient)
        {
            ReverseUvNurbCurve(sUvCurve);
        }
        m_rState.SetEdgeuseCurve2d(uiEdgeuseIndex, sUvCurve);
    }
}

} // namespace occt
