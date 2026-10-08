// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepGeometryEval.h"

#include "UsdBrepTokens.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace UsdBrep
{
PXR_NAMESPACE_USING_DIRECTIVE
using namespace UsdBrepData;

static constexpr double TWO_PI = 6.28318530717958647692;
static constexpr double ONE_PI = 3.14159265358979323846;

static constexpr double kDivEpsilon = 1e-30; // near-zero guard for divisions
static constexpr double kFiniteDiffStep = 1e-6; // step size for numerical derivatives
static constexpr double kNewtonConvSq = 1e-24; // Newton convergence threshold (distance^2)
static constexpr double kAngleEpsilon = 1e-10; // angular / cosine coincidence
static constexpr double kDegenDerivSq = 1e-16; // degenerate partial derivative (length^2)
static constexpr double kSpanEpsilon = 1e-12; // minimum parametric span

// ===== UsdBrepSurfaceDomain ======================================================================

double UsdBrepSurfaceDomain::UnmapU(double u) const
{
    double span = USpan();
    return (span > kDivEpsilon) ? (u - uvMin[0]) / span : 0.0;
}

double UsdBrepSurfaceDomain::UnmapV(double v) const
{
    double span = VSpan();
    return (span > kDivEpsilon) ? (v - uvMin[1]) / span : 0.0;
}

double UsdBrepSurfaceDomain::ClampU(double u) const
{
    if (u < uvMin[0])
    {
        return uvMin[0];
    }
    if (u > uvMax[0])
    {
        return uvMax[0];
    }
    return u;
}

double UsdBrepSurfaceDomain::ClampV(double v) const
{
    if (v < uvMin[1])
    {
        return uvMin[1];
    }
    if (v > uvMax[1])
    {
        return uvMax[1];
    }
    return v;
}

double UsdBrepSurfaceDomain::WrapU(double u) const
{
    if (!uPeriodic || uPeriod <= kDivEpsilon)
    {
        return ClampU(u);
    }
    double shifted = u - uvMin[0];
    shifted = std::fmod(shifted, uPeriod);
    if (shifted < 0.0)
    {
        shifted += uPeriod;
    }
    return uvMin[0] + shifted;
}

double UsdBrepSurfaceDomain::WrapV(double v) const
{
    if (!vPeriodic || vPeriod <= kDivEpsilon)
    {
        return ClampV(v);
    }
    double shifted = v - uvMin[1];
    shifted = std::fmod(shifted, vPeriod);
    if (shifted < 0.0)
    {
        shifted += vPeriod;
    }
    return uvMin[1] + shifted;
}

// ===== GetSurfacePeriodicity =================================================================

void GetSurfacePeriodicity(const TfToken& surfaceType, bool& oUPeriodic, bool& oVPeriodic, double& oUPeriod, double& oVPeriod)
{
    oUPeriodic = false;
    oVPeriodic = false;
    oUPeriod = 0.0;
    oVPeriod = 0.0;

    if (surfaceType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
    {
        // Both directions linear
    }
    else if (surfaceType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
    {
        oUPeriodic = true;
        oUPeriod = TWO_PI;
    }
    else if (surfaceType == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
    {
        oUPeriodic = true;
        oUPeriod = TWO_PI;
    }
    else if (surfaceType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        oUPeriodic = true;
        oUPeriod = TWO_PI;
        oVPeriodic = false; // latitude is NOT periodic, it's bounded [-pi/2, pi/2]
    }
    else if (surfaceType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
    {
        oUPeriodic = true;
        oUPeriod = TWO_PI;
        oVPeriodic = true;
        oVPeriod = TWO_PI;
    }
    else if (surfaceType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        // NURBS: periodicity depends on knot structure, treat as non-periodic
    }
}

// ===== GetSurfaceDomain ======================================================================

UsdBrepSurfaceDomain GetSurfaceDomain(const UsdBrepView& brep, uint32_t iLocalFace)
{
    UsdBrepSurfaceDomain domain;

    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return domain;
    }

    domain.surfaceType = surfTypes[iLocalFace];

    auto ranges = brep.FaceRanges();
    uint32_t rangeIdx = iLocalFace * 2;
    if (rangeIdx + 1 < ranges.size())
    {
        domain.uvMin = ranges[rangeIdx];
        domain.uvMax = ranges[rangeIdx + 1];
    }

    GetSurfacePeriodicity(domain.surfaceType, domain.uPeriodic, domain.vPeriodic, domain.uPeriod, domain.vPeriod);

    // All periodic directions are angular.  Sphere v (latitude) is the
    // one case where a direction is angular but not periodic.
    domain.uAngular = domain.uPeriodic;
    domain.vAngular = domain.vPeriodic;
    if (domain.surfaceType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        domain.vAngular = true;
    }

    return domain;
}

// ===== Range-aware evaluation ================================================================

GfVec3d EvaluateSurfaceNormalized(const UsdBrepView& brep, uint32_t iLocalFace, double s, double t)
{
    UsdBrepSurfaceDomain domain = GetSurfaceDomain(brep, iLocalFace);
    double u = domain.MapU(s);
    double v = domain.MapV(t);
    return EvaluateSurface(brep, iLocalFace, u, v);
}

GfVec3d EvaluateSurfaceNormalNormalized(const UsdBrepView& brep, uint32_t iLocalFace, double s, double t)
{
    UsdBrepSurfaceDomain domain = GetSurfaceDomain(brep, iLocalFace);
    double u = domain.MapU(s);
    double v = domain.MapV(t);
    return EvaluateSurfaceNormal(brep, iLocalFace, u, v);
}

GfVec3d EvaluateSurfaceClamped(const UsdBrepView& brep, uint32_t iLocalFace, double u, double v)
{
    UsdBrepSurfaceDomain domain = GetSurfaceDomain(brep, iLocalFace);
    return EvaluateSurface(brep, iLocalFace, domain.ClampU(u), domain.ClampV(v));
}

GfVec3d EvaluateSurfaceBoundary(const UsdBrepView& brep, uint32_t iLocalFace, UsdBrepDomainEdge edge, double t)
{
    UsdBrepSurfaceDomain domain = GetSurfaceDomain(brep, iLocalFace);
    double u = 0.0;
    double v = 0.0;

    switch (edge)
    {
        case UsdBrepDomainEdge::UMin:
            u = domain.uvMin[0];
            v = domain.MapV(t);
            break;
        case UsdBrepDomainEdge::UMax:
            u = domain.uvMax[0];
            v = domain.MapV(t);
            break;
        case UsdBrepDomainEdge::VMin:
            u = domain.MapU(t);
            v = domain.uvMin[1];
            break;
        case UsdBrepDomainEdge::VMax:
            u = domain.MapU(t);
            v = domain.uvMax[1];
            break;
    }

    return EvaluateSurface(brep, iLocalFace, u, v);
}

std::vector<GfVec3d> SampleSurfaceBoundary(const UsdBrepView& brep, uint32_t iLocalFace, UsdBrepDomainEdge edge, uint32_t numSegments)
{
    std::vector<GfVec3d> pts;
    if (numSegments == 0)
    {
        return pts;
    }

    pts.reserve(numSegments + 1);
    for (uint32_t i = 0; i <= numSegments; ++i)
    {
        double t = static_cast<double>(i) / static_cast<double>(numSegments);
        pts.push_back(EvaluateSurfaceBoundary(brep, iLocalFace, edge, t));
    }
    return pts;
}

double ApproxBoundaryLength(const UsdBrepView& brep, uint32_t iLocalFace, UsdBrepDomainEdge edge, uint32_t numSegments)
{
    auto pts = SampleSurfaceBoundary(brep, iLocalFace, edge, numSegments);
    double length = 0.0;
    for (size_t i = 1; i < pts.size(); ++i)
    {
        length += (pts[i] - pts[i - 1]).GetLength();
    }
    return length;
}

// ===== Analytic surface evaluators ==========================================================

// S(u,v) = origin + u * refDir + v * (axis x refDir)
GfVec3d EvalPlaneSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    return origin + u * refDir + v * yDir;
}

// S(u,v) = origin + v * axis + radius * [cos(u)*refDir + sin(u)*(axis x refDir)]
GfVec3d EvalCylinderSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double radius, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    return origin + v * axis + radius * (std::cos(u) * refDir + std::sin(u) * yDir);
}

// S(u,v) = origin + v*axis + (radius + v*tan(semiAngle)) * [cos(u)*refDir + sin(u)*(axis x refDir)]
GfVec3d EvalConeSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double radius, double semiAngle, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    double r = radius + v * std::tan(semiAngle);
    return origin + v * axis + r * (std::cos(u) * refDir + std::sin(u) * yDir);
}

// S(u,v) = center + radius * [cos(v)*cos(u)*refDir + cos(v)*sin(u)*(axis x refDir) + sin(v)*axis]
GfVec3d EvalSphereSurface(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double radius, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    double cosV = std::cos(v);
    return center + radius * (cosV * std::cos(u) * refDir + cosV * std::sin(u) * yDir + std::sin(v) * axis);
}

// S(u,v) = origin + (majorR + minorR*cos(v)) * [cos(u)*refDir + sin(u)*(axis x refDir)]
//                  + minorR*sin(v) * axis
GfVec3d EvalTorusSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double majorRadius, double minorRadius, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    double r = majorRadius + minorRadius * std::cos(v);
    return origin + r * (std::cos(u) * refDir + std::sin(u) * yDir) + minorRadius * std::sin(v) * axis;
}

// ===== Analytic surface normal evaluators ====================================================

GfVec3d EvalPlaneSurfaceNormal(const GfVec3d& axis)
{
    return axis;
}

GfVec3d EvalCylinderSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double u)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d n = std::cos(u) * refDir + std::sin(u) * yDir;
    return n.GetNormalized();
}

GfVec3d EvalConeSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double semiAngle, double u)
{
    GfVec3d yDir = GfCross(axis, refDir);
    double cosSA = std::cos(semiAngle);
    double sinSA = std::sin(semiAngle);
    GfVec3d radial = std::cos(u) * refDir + std::sin(u) * yDir;
    GfVec3d n = cosSA * radial - sinSA * axis;
    return n.GetNormalized();
}

GfVec3d EvalSphereSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    double cosV = std::cos(v);
    GfVec3d n = cosV * std::cos(u) * refDir + cosV * std::sin(u) * yDir + std::sin(v) * axis;
    return n.GetNormalized();
}

GfVec3d EvalTorusSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double /*majorRadius*/, double /*minorRadius*/, double u, double v)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d radial = std::cos(u) * refDir + std::sin(u) * yDir;
    GfVec3d n = std::cos(v) * radial + std::sin(v) * axis;
    return n.GetNormalized();
}

// ===== Analytic curve evaluators =============================================================

GfVec3d EvalLineCurve(const GfVec3d& origin, const GfVec3d& direction, double t)
{
    return origin + t * direction;
}

GfVec3d EvalCircleCurve(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double radius, double t)
{
    GfVec3d yDir = GfCross(axis, refDir);
    return center + radius * (std::cos(t) * refDir + std::sin(t) * yDir);
}

GfVec3d EvalEllipseCurve(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double xRadius, double yRadius, double t)
{
    GfVec3d yDir = GfCross(axis, refDir);
    return center + xRadius * std::cos(t) * refDir + yRadius * std::sin(t) * yDir;
}

// ===== NURBS: de Boor evaluation =============================================================

// Find the knot span index such that knots[span] <= t < knots[span+1].
// Clamps to the valid range [degree, nKnots - degree - 2].
static uint32_t FindKnotSpan(TfSpan<const double> knots, uint32_t degree, double t)
{
    uint32_t nKnots = static_cast<uint32_t>(knots.size());
    uint32_t n = nKnots - degree - 1;
    if (n == 0)
    {
        return degree;
    }

    if (t >= knots[n])
    {
        return n - 1;
    }
    if (t <= knots[degree])
    {
        return degree;
    }

    uint32_t lo = degree;
    uint32_t hi = n;
    uint32_t mid = (lo + hi) / 2;
    while (t < knots[mid] || t >= knots[mid + 1])
    {
        if (t < knots[mid])
        {
            hi = mid;
        }
        else
        {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    return mid;
}

// De Boor's algorithm returning both the dehomogenized point and the
// accumulated weight.  The weight output is needed by the tensor-product
// surface evaluator so it can carry rational weights into the second pass.
template <typename VecT>
static std::pair<VecT, double> DeBoorWeighted(
    TfSpan<const VecT> cpts,
    TfSpan<const double> knots,
    TfSpan<const double> weights,
    uint32_t order,
    double t
)
{
    uint32_t degree = order - 1;
    uint32_t vertexCount = static_cast<uint32_t>(cpts.size());
    if (vertexCount == 0 || knots.empty())
    {
        return { VecT(0.0), 1.0 };
    }

    uint32_t span = FindKnotSpan(knots, degree, t);

    constexpr uint32_t dims = VecT::dimension;
    std::vector<double> d((degree + 1) * (dims + 1));

    for (uint32_t j = 0; j <= degree; ++j)
    {
        uint32_t idx = span - degree + j;
        if (idx >= vertexCount)
        {
            idx = vertexCount - 1;
        }

        double w = (!weights.empty()) ? weights[idx] : 1.0;
        for (uint32_t c = 0; c < dims; ++c)
        {
            d[j * (dims + 1) + c] = cpts[idx][c] * w;
        }
        d[j * (dims + 1) + dims] = w;
    }

    for (uint32_t r = 1; r <= degree; ++r)
    {
        for (uint32_t j = degree; j >= r; --j)
        {
            uint32_t ki = span - degree + j;
            double denom = knots[ki + degree - r + 1] - knots[ki];
            double alpha = (denom > kDivEpsilon) ? (t - knots[ki]) / denom : 0.0;
            for (uint32_t c = 0; c <= dims; ++c)
            {
                d[j * (dims + 1) + c] = (1.0 - alpha) * d[(j - 1) * (dims + 1) + c] + alpha * d[j * (dims + 1) + c];
            }
        }
    }

    double w = d[degree * (dims + 1) + dims];
    if (std::fabs(w) < kDivEpsilon)
    {
        w = 1.0;
    }

    VecT result;
    for (uint32_t c = 0; c < dims; ++c)
    {
        result[c] = d[degree * (dims + 1) + c] / w;
    }
    return { result, w };
}

template <typename VecT>
static VecT DeBoor(TfSpan<const VecT> cpts, TfSpan<const double> knots, TfSpan<const double> weights, uint32_t order, double t)
{
    return DeBoorWeighted(cpts, knots, weights, order, t).first;
}

GfVec3d EvalNurbsCurve3d(TfSpan<const GfVec3d> controlVertices, TfSpan<const double> knots, TfSpan<const double> weights, uint32_t order, double t)
{
    return DeBoor<GfVec3d>(controlVertices, knots, weights, order, t);
}

GfVec2d EvalNurbsCurve2d(TfSpan<const GfVec2d> controlVertices, TfSpan<const double> knots, TfSpan<const double> weights, uint32_t order, double t)
{
    return DeBoor<GfVec2d>(controlVertices, knots, weights, order, t);
}

// NURBS surface: tensor product evaluation.
// Evaluate v-direction isoparametric curves at v, then evaluate resulting curve at u.
GfVec3d EvalNurbsSurface(
    TfSpan<const GfVec3d> controlVertices,
    uint32_t uVertexCount,
    uint32_t vVertexCount,
    TfSpan<const double> uKnots,
    TfSpan<const double> vKnots,
    TfSpan<const double> weights,
    uint32_t uOrder,
    uint32_t vOrder,
    double u,
    double v
)
{
    if (uVertexCount == 0 || vVertexCount == 0)
    {
        return GfVec3d(0.0);
    }

    // Tensor-product evaluation: for each u-row, evaluate the v-direction
    // curve at v (keeping the weight), then evaluate the resulting u-curve.
    std::vector<GfVec3d> uCurve(uVertexCount);
    std::vector<double> uWeightsVec(uVertexCount);

    for (uint32_t ui = 0; ui < uVertexCount; ++ui)
    {
        uint32_t rowStart = ui * vVertexCount;
        TfSpan<const GfVec3d> rowCpts(controlVertices.data() + rowStart, vVertexCount);
        TfSpan<const double> rowWeights = !weights.empty() ? TfSpan<const double>(weights.data() + rowStart, vVertexCount) : TfSpan<const double>();

        auto [pt, w] = DeBoorWeighted<GfVec3d>(rowCpts, vKnots, rowWeights, vOrder, v);
        uCurve[ui] = pt;
        uWeightsVec[ui] = w;
    }

    return EvalNurbsCurve3d(TfSpan<const GfVec3d>(uCurve), uKnots, TfSpan<const double>(uWeightsVec), uOrder, u);
}

// ===== High-level dispatch using UsdBrepView =================================================

GfVec3d EvaluateSurface(const UsdBrepView& brep, uint32_t iLocalFace, double u, double v)
{
    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return GfVec3d(0.0);
    }

    const TfToken& surfType = surfTypes[iLocalFace];
    uint32_t idx = brep.FaceTypeSubIndex(iLocalFace);

    if (surfType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
    {
        auto origins = brep.PlaneSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec3d(0.0);
        }
        return EvalPlaneSurface(origins[idx], brep.PlaneSurfaceAxes()[idx], brep.PlaneSurfaceRefDirections()[idx], u, v);
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
    {
        auto origins = brep.CylinderSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec3d(0.0);
        }
        return EvalCylinderSurface(
            origins[idx],
            brep.CylinderSurfaceAxes()[idx],
            brep.CylinderSurfaceRefDirections()[idx],
            brep.CylinderSurfaceRadii()[idx],
            u,
            v
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
    {
        auto origins = brep.ConeSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec3d(0.0);
        }
        return EvalConeSurface(
            origins[idx],
            brep.ConeSurfaceAxes()[idx],
            brep.ConeSurfaceRefDirections()[idx],
            brep.ConeSurfaceRadii()[idx],
            brep.ConeSurfaceSemiAngles()[idx],
            u,
            v
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        auto centers = brep.SphereSurfaceCenters();
        if (idx >= centers.size())
        {
            return GfVec3d(0.0);
        }
        return EvalSphereSurface(
            centers[idx],
            brep.SphereSurfaceAxes()[idx],
            brep.SphereSurfaceRefDirections()[idx],
            brep.SphereSurfaceRadii()[idx],
            u,
            v
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
    {
        auto origins = brep.TorusSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec3d(0.0);
        }
        return EvalTorusSurface(
            origins[idx],
            brep.TorusSurfaceAxes()[idx],
            brep.TorusSurfaceRefDirections()[idx],
            brep.TorusSurfaceMajorRadii()[idx],
            brep.TorusSurfaceMinorRadii()[idx],
            u,
            v
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        auto uVCs = brep.NurbsSurfaceUVertexCounts();
        if (idx >= uVCs.size())
        {
            return GfVec3d(0.0);
        }

        auto vVCs = brep.NurbsSurfaceVVertexCounts();
        auto uOrds = brep.NurbsSurfaceUOrders();
        auto vOrds = brep.NurbsSurfaceVOrders();

        uint32_t uVC = uVCs[idx];
        uint32_t vVC = vVCs[idx];
        uint32_t uOrd = uOrds[idx];
        uint32_t vOrd = vOrds[idx];

        uint32_t cpStart = 0, ukStart = 0, vkStart = 0;
        for (uint32_t i = 0; i < idx; ++i)
        {
            cpStart += uVCs[i] * vVCs[i];
            ukStart += uVCs[i] + uOrds[i];
            vkStart += vVCs[i] + vOrds[i];
        }

        auto cps = brep.NurbsSurfaceControlVertices();
        auto uKnots = brep.NurbsSurfaceUKnots();
        auto vKnots = brep.NurbsSurfaceVKnots();
        auto wts = brep.NurbsSurfaceWeights();

        uint32_t totalCP = uVC * vVC;
        TfSpan<const double> wtsSub = !wts.empty() ? wts.subspan(cpStart, totalCP) : TfSpan<const double>();
        return EvalNurbsSurface(
            cps.subspan(cpStart, totalCP),
            uVC,
            vVC,
            uKnots.subspan(ukStart, uVC + uOrd),
            vKnots.subspan(vkStart, vVC + vOrd),
            wtsSub,
            uOrd,
            vOrd,
            u,
            v
        );
    }

    return GfVec3d(0.0);
}

GfVec3d EvaluateSurfaceNormal(const UsdBrepView& brep, uint32_t iLocalFace, double u, double v)
{
    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return GfVec3d(0.0, 0.0, 1.0);
    }

    const TfToken& surfType = surfTypes[iLocalFace];
    uint32_t idx = brep.FaceTypeSubIndex(iLocalFace);

    if (surfType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
    {
        auto axes = brep.PlaneSurfaceAxes();
        if (idx >= axes.size())
        {
            return GfVec3d(0.0, 0.0, 1.0);
        }
        return EvalPlaneSurfaceNormal(axes[idx]);
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
    {
        auto axes = brep.CylinderSurfaceAxes();
        if (idx >= axes.size())
        {
            return GfVec3d(0.0, 0.0, 1.0);
        }
        return EvalCylinderSurfaceNormal(axes[idx], brep.CylinderSurfaceRefDirections()[idx], u);
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
    {
        auto axes = brep.ConeSurfaceAxes();
        if (idx >= axes.size())
        {
            return GfVec3d(0.0, 0.0, 1.0);
        }
        return EvalConeSurfaceNormal(axes[idx], brep.ConeSurfaceRefDirections()[idx], brep.ConeSurfaceSemiAngles()[idx], u);
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        auto axes = brep.SphereSurfaceAxes();
        if (idx >= axes.size())
        {
            return GfVec3d(0.0, 0.0, 1.0);
        }
        return EvalSphereSurfaceNormal(axes[idx], brep.SphereSurfaceRefDirections()[idx], u, v);
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
    {
        auto axes = brep.TorusSurfaceAxes();
        if (idx >= axes.size())
        {
            return GfVec3d(0.0, 0.0, 1.0);
        }
        return EvalTorusSurfaceNormal(
            axes[idx],
            brep.TorusSurfaceRefDirections()[idx],
            brep.TorusSurfaceMajorRadii()[idx],
            brep.TorusSurfaceMinorRadii()[idx],
            u,
            v
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        double h = kFiniteDiffStep;
        GfVec3d p = EvaluateSurface(brep, iLocalFace, u, v);
        GfVec3d pu = EvaluateSurface(brep, iLocalFace, u + h, v);
        GfVec3d pv = EvaluateSurface(brep, iLocalFace, u, v + h);
        GfVec3d du = (pu - p);
        GfVec3d dv = (pv - p);
        GfVec3d n = GfCross(du, dv);
        double len = n.GetLength();
        if (len > kDivEpsilon)
        {
            return n / len;
        }
    }

    return GfVec3d(0.0, 0.0, 1.0);
}

GfVec3d EvaluateCurve3d(const UsdBrepView& brep, uint32_t iLocalEdge, double t)
{
    auto crvTypes = brep.EdgeCurveTypes();
    if (iLocalEdge >= crvTypes.size())
    {
        return GfVec3d(0.0);
    }

    const TfToken& crvType = crvTypes[iLocalEdge];
    uint32_t idx = brep.EdgeTypeSubIndex(iLocalEdge);

    if (crvType == UsdBrepCurveTokens->brepCurve3dLineAPI)
    {
        auto origins = brep.EdgeLineOrigins();
        if (idx >= origins.size())
        {
            return GfVec3d(0.0);
        }
        return EvalLineCurve(origins[idx], brep.EdgeLineDirections()[idx], t);
    }
    else if (crvType == UsdBrepCurveTokens->brepCurve3dCircleAPI)
    {
        auto centers = brep.EdgeCircleCenters();
        if (idx >= centers.size())
        {
            return GfVec3d(0.0);
        }
        return EvalCircleCurve(centers[idx], brep.EdgeCircleAxes()[idx], brep.EdgeCircleRefDirections()[idx], brep.EdgeCircleRadii()[idx], t);
    }
    else if (crvType == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
    {
        auto centers = brep.EdgeEllipseCenters();
        if (idx >= centers.size())
        {
            return GfVec3d(0.0);
        }
        return EvalEllipseCurve(
            centers[idx],
            brep.EdgeEllipseAxes()[idx],
            brep.EdgeEllipseRefDirections()[idx],
            brep.EdgeEllipseXRadii()[idx],
            brep.EdgeEllipseYRadii()[idx],
            t
        );
    }
    else if (crvType == UsdBrepCurveTokens->brepCurve3dNurbAPI)
    {
        auto vcs = brep.EdgeNurbsVertexCounts();
        if (idx >= vcs.size())
        {
            return GfVec3d(0.0);
        }
        auto ords = brep.EdgeNurbsOrders();
        uint32_t vc = vcs[idx];
        uint32_t ord = ords[idx];

        uint32_t cpStart = 0, kStart = 0;
        for (uint32_t i = 0; i < idx; ++i)
        {
            cpStart += vcs[i];
            kStart += vcs[i] + ords[i];
        }

        auto cps = brep.EdgeNurbsControlVertices();
        auto knots = brep.EdgeNurbsKnots();
        auto wts = brep.EdgeNurbsWeights();

        TfSpan<const double> wtsSub = !wts.empty() ? wts.subspan(cpStart, vc) : TfSpan<const double>();
        return EvalNurbsCurve3d(cps.subspan(cpStart, vc), knots.subspan(kStart, vc + ord), wtsSub, ord, t);
    }

    return GfVec3d(0.0);
}

// ===== 2D pcurve (UV trim curve) evaluation ==================================================

static void GetPCurveSpans(
    const UsdBrepView& brep,
    uint32_t iLocalEdgeuse,
    uint32_t& oCpStart,
    uint32_t& oKStart,
    uint32_t& oVertexCount,
    uint32_t& oOrder
)
{
    oVertexCount = 0;
    oOrder = 0;
    oCpStart = 0;
    oKStart = 0;

    auto vcs = brep.EdgeuseUVNurbsVertexCounts();
    if (iLocalEdgeuse >= vcs.size())
    {
        return;
    }

    auto ords = brep.EdgeuseUVNurbsOrders();
    oVertexCount = vcs[iLocalEdgeuse];
    oOrder = ords[iLocalEdgeuse];

    for (uint32_t i = 0; i < iLocalEdgeuse; ++i)
    {
        oCpStart += vcs[i];
        oKStart += vcs[i] + ords[i];
    }
}

bool HasPCurve(const UsdBrepView& brep, uint32_t iLocalEdgeuse)
{
    auto vcs = brep.EdgeuseUVNurbsVertexCounts();
    if (iLocalEdgeuse >= vcs.size())
    {
        return false;
    }
    return vcs[iLocalEdgeuse] > 0;
}

GfVec2d GetPCurveRange(const UsdBrepView& brep, uint32_t iLocalEdgeuse)
{
    uint32_t cpStart, kStart, vc, ord;
    GetPCurveSpans(brep, iLocalEdgeuse, cpStart, kStart, vc, ord);
    if (vc == 0 || ord == 0)
    {
        return GfVec2d(0, 0);
    }

    auto knots = brep.EdgeuseUVNurbsKnots();
    uint32_t knotCount = vc + ord;
    if (kStart + knotCount > knots.size())
    {
        return GfVec2d(0, 0);
    }

    return GfVec2d(knots[kStart + ord - 1], knots[kStart + vc]);
}

GfVec2d EvaluateCurve2d(const UsdBrepView& brep, uint32_t iLocalEdgeuse, double t)
{
    uint32_t cpStart, kStart, vc, ord;
    GetPCurveSpans(brep, iLocalEdgeuse, cpStart, kStart, vc, ord);
    if (vc == 0 || ord == 0)
    {
        return GfVec2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    }

    auto cps = brep.EdgeuseUVNurbsControlVertices();
    auto knots = brep.EdgeuseUVNurbsKnots();
    auto wts = brep.EdgeuseUVNurbsWeights();

    uint32_t knotCount = vc + ord;
    if (cpStart + vc > cps.size() || kStart + knotCount > knots.size())
    {
        return GfVec2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    }

    TfSpan<const double> wtsSub = !wts.empty() ? wts.subspan(cpStart, vc) : TfSpan<const double>();
    return EvalNurbsCurve2d(cps.subspan(cpStart, vc), knots.subspan(kStart, knotCount), wtsSub, ord, t);
}

// ===== Inverse projection: 3D point -> UV parameters =========================================

// Wrap angle from atan2 into [0, 2*pi)
static double WrapAngleTo02Pi(double angle)
{
    if (angle < 0.0)
    {
        angle += TWO_PI;
    }
    if (angle >= TWO_PI)
    {
        angle -= TWO_PI;
    }
    return angle;
}

GfVec2d InverseProjectPlane(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, const GfVec3d& point)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d d = point - origin;
    return GfVec2d(GfDot(d, refDir), GfDot(d, yDir));
}

GfVec2d InverseProjectCylinder(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double /*radius*/, const GfVec3d& point)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d d = point - origin;
    double v = GfDot(d, axis);
    GfVec3d proj = d - v * axis;
    double cu = GfDot(proj, refDir);
    double su = GfDot(proj, yDir);
    double u = WrapAngleTo02Pi(std::atan2(su, cu));
    return GfVec2d(u, v);
}

GfVec2d InverseProjectCone(
    const GfVec3d& origin,
    const GfVec3d& axis,
    const GfVec3d& refDir,
    double /*radius*/,
    double /*semiAngle*/,
    const GfVec3d& point
)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d d = point - origin;
    double v = GfDot(d, axis);
    GfVec3d proj = d - v * axis;
    double cu = GfDot(proj, refDir);
    double su = GfDot(proj, yDir);
    double u = WrapAngleTo02Pi(std::atan2(su, cu));
    return GfVec2d(u, v);
}

GfVec2d InverseProjectSphere(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double radius, const GfVec3d& point)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d d = (point - center);
    double R = (radius > kDivEpsilon) ? radius : d.GetLength();
    if (R < kDivEpsilon)
    {
        return GfVec2d(0.0, 0.0);
    }

    d = d / R;
    double sinV = GfDot(d, axis);
    sinV = std::clamp(sinV, -1.0, 1.0);
    double v = std::asin(sinV);
    double cosV = std::cos(v);

    if (std::fabs(cosV) < kAngleEpsilon)
    {
        // At a pole: u is indeterminate; return NaN to signal callers
        return GfVec2d(std::numeric_limits<double>::quiet_NaN(), v);
    }

    double cu = GfDot(d, refDir) / cosV;
    double su = GfDot(d, yDir) / cosV;
    double u = WrapAngleTo02Pi(std::atan2(su, cu));
    return GfVec2d(u, v);
}

GfVec2d InverseProjectTorus(
    const GfVec3d& origin,
    const GfVec3d& axis,
    const GfVec3d& refDir,
    double majorRadius,
    double minorRadius,
    const GfVec3d& point
)
{
    GfVec3d yDir = GfCross(axis, refDir);
    GfVec3d d = point - origin;
    double hAxial = GfDot(d, axis);
    GfVec3d proj = d - hAxial * axis;
    double projLen = proj.GetLength();

    double cu = (projLen > kDivEpsilon) ? GfDot(proj, refDir) / projLen : 1.0;
    double su = (projLen > kDivEpsilon) ? GfDot(proj, yDir) / projLen : 0.0;
    double u = WrapAngleTo02Pi(std::atan2(su, cu));

    double cosV = (std::fabs(minorRadius) > kDivEpsilon) ? (projLen - majorRadius) / minorRadius : 1.0;
    double sinV = (std::fabs(minorRadius) > kDivEpsilon) ? hAxial / minorRadius : 0.0;
    double v = WrapAngleTo02Pi(std::atan2(sinV, cosV));
    return GfVec2d(u, v);
}

GfVec2d InverseProjectSurface(const UsdBrepView& brep, uint32_t iLocalFace, const GfVec3d& point)
{
    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return GfVec2d(0.0, 0.0);
    }

    const TfToken& surfType = surfTypes[iLocalFace];
    uint32_t idx = brep.FaceTypeSubIndex(iLocalFace);

    if (surfType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
    {
        auto origins = brep.PlaneSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec2d(0.0, 0.0);
        }
        return InverseProjectPlane(origins[idx], brep.PlaneSurfaceAxes()[idx], brep.PlaneSurfaceRefDirections()[idx], point);
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
    {
        auto origins = brep.CylinderSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec2d(0.0, 0.0);
        }
        return InverseProjectCylinder(
            origins[idx],
            brep.CylinderSurfaceAxes()[idx],
            brep.CylinderSurfaceRefDirections()[idx],
            brep.CylinderSurfaceRadii()[idx],
            point
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
    {
        auto origins = brep.ConeSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec2d(0.0, 0.0);
        }
        return InverseProjectCone(
            origins[idx],
            brep.ConeSurfaceAxes()[idx],
            brep.ConeSurfaceRefDirections()[idx],
            brep.ConeSurfaceRadii()[idx],
            brep.ConeSurfaceSemiAngles()[idx],
            point
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        auto centers = brep.SphereSurfaceCenters();
        if (idx >= centers.size())
        {
            return GfVec2d(0.0, 0.0);
        }
        return InverseProjectSphere(
            centers[idx],
            brep.SphereSurfaceAxes()[idx],
            brep.SphereSurfaceRefDirections()[idx],
            brep.SphereSurfaceRadii()[idx],
            point
        );
    }
    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
    {
        auto origins = brep.TorusSurfaceOrigins();
        if (idx >= origins.size())
        {
            return GfVec2d(0.0, 0.0);
        }
        return InverseProjectTorus(
            origins[idx],
            brep.TorusSurfaceAxes()[idx],
            brep.TorusSurfaceRefDirections()[idx],
            brep.TorusSurfaceMajorRadii()[idx],
            brep.TorusSurfaceMinorRadii()[idx],
            point
        );
    }

    else if (surfType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        // Newton's method inverse projection for NURBS surfaces.
        UsdBrepSurfaceDomain dom = GetSurfaceDomain(brep, iLocalFace);
        double uMin = dom.uvMin[0], uMax = dom.uvMax[0];
        double vMin = dom.uvMin[1], vMax = dom.uvMax[1];

        // Grid search for initial guess.  Use enough samples so the grid
        // spacing is at most ~10 units in each parameter direction; for a
        // 360° periodic NURBS torus this gives 36 samples (10° spacing)
        // which reliably resolves the non-linear parameter-to-angle mapping
        // of rational Bezier segments.
        int N = std::max(10, static_cast<int>(std::ceil((uMax - uMin) / 10.0)));
        int M = std::max(10, static_cast<int>(std::ceil((vMax - vMin) / 10.0)));
        N = std::min(N, 40);
        M = std::min(M, 40);
        double bestDist = std::numeric_limits<double>::max();
        double bestU = (uMin + uMax) * 0.5;
        double bestV = (vMin + vMax) * 0.5;
        for (int i = 0; i <= N; ++i)
        {
            double u = uMin + (uMax - uMin) * i / N;
            for (int j = 0; j <= M; ++j)
            {
                double v = vMin + (vMax - vMin) * j / M;
                GfVec3d p = EvaluateSurface(brep, iLocalFace, u, v);
                double d = (p - point).GetLengthSq();
                if (d < bestDist)
                {
                    bestDist = d;
                    bestU = u;
                    bestV = v;
                }
            }
        }

        double u = bestU, v = bestV;
        constexpr double h = kFiniteDiffStep;
        for (int iter = 0; iter < 15; ++iter)
        {
            GfVec3d S = EvaluateSurface(brep, iLocalFace, u, v);
            GfVec3d r = S - point;
            if (r.GetLengthSq() < kNewtonConvSq)
            {
                break;
            }
            GfVec3d Su = (EvaluateSurface(brep, iLocalFace, u + h, v) - S) / h;
            GfVec3d Sv = (EvaluateSurface(brep, iLocalFace, u, v + h) - S) / h;
            double a11 = GfDot(Su, Su), a12 = GfDot(Su, Sv), a22 = GfDot(Sv, Sv);
            double b1 = -GfDot(Su, r), b2 = -GfDot(Sv, r);
            double det = a11 * a22 - a12 * a12;
            if (std::fabs(det) < kDivEpsilon)
            {
                break;
            }
            u += (a22 * b1 - a12 * b2) / det;
            v += (a11 * b2 - a12 * b1) / det;
            u = std::clamp(u, uMin, uMax);
            v = std::clamp(v, vMin, vMax);
        }

        // At degenerate surface points (cone apex, sphere poles) one partial
        // derivative collapses to zero — the parameter in that direction is
        // indeterminate.  Resolve it with a 1D grid search at a slightly
        // offset position where the surface is non-degenerate.
        // Skip when Newton already converged (residual ≈ 0), since the
        // 1D search at an offset would pick an arbitrary value for the
        // indeterminate parameter.
        {
            GfVec3d S = EvaluateSurface(brep, iLocalFace, u, v);
            double convergenceResidSq = (S - point).GetLengthSq();
            GfVec3d Su = (EvaluateSurface(brep, iLocalFace, u + h, v) - S) / h;
            GfVec3d Sv = (EvaluateSurface(brep, iLocalFace, u, v + h) - S) / h;
            double lenSu = Su.GetLengthSq();
            double lenSv = Sv.GetLengthSq();
            double threshold = kDegenDerivSq;
            double uSpanL = uMax - uMin;
            double vSpanL = vMax - vMin;
            if (lenSu < threshold && lenSv > threshold && convergenceResidSq > kNewtonConvSq)
            {
                double vOff = v + std::copysign(std::max(vSpanL * 0.01, h * 100), vSpanL * 0.5 - (v - vMin));
                vOff = std::clamp(vOff, vMin, vMax);
                double uBest2 = u;
                double dBest2 = std::numeric_limits<double>::max();
                for (int ii = 0; ii <= N; ++ii)
                {
                    double uProbe = uMin + uSpanL * ii / N;
                    GfVec3d p = EvaluateSurface(brep, iLocalFace, uProbe, vOff);
                    double dd = (p - point).GetLengthSq();
                    if (dd < dBest2)
                    {
                        dBest2 = dd;
                        uBest2 = uProbe;
                    }
                }
                u = uBest2;
            }
            else if (lenSv < threshold && lenSu > threshold && convergenceResidSq > kNewtonConvSq)
            {
                double uOff = u + std::copysign(std::max(uSpanL * 0.01, h * 100), uSpanL * 0.5 - (u - uMin));
                uOff = std::clamp(uOff, uMin, uMax);
                double vBest2 = v;
                double dBest2 = std::numeric_limits<double>::max();
                for (int jj = 0; jj <= M; ++jj)
                {
                    double vProbe = vMin + vSpanL * jj / M;
                    GfVec3d p = EvaluateSurface(brep, iLocalFace, uOff, vProbe);
                    double dd = (p - point).GetLengthSq();
                    if (dd < dBest2)
                    {
                        dBest2 = dd;
                        vBest2 = vProbe;
                    }
                }
                v = vBest2;
            }
        }

        return GfVec2d(u, v);
    }

    return GfVec2d(0.0, 0.0);
}

GfVec2d InverseProjectSurfaceWithHint(const UsdBrepView& brep, uint32_t iLocalFace, const GfVec3d& point, const GfVec2d& uvHint)
{
    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return GfVec2d(0.0, 0.0);
    }

    const TfToken& surfType = surfTypes[iLocalFace];

    if (surfType != UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        return InverseProjectSurface(brep, iLocalFace, point);
    }

    UsdBrepSurfaceDomain dom = GetSurfaceDomain(brep, iLocalFace);
    double uMin = dom.uvMin[0], uMax = dom.uvMax[0];
    double vMin = dom.uvMin[1], vMax = dom.uvMax[1];
    double uSpan = uMax - uMin;
    double vSpan = vMax - vMin;

    // Detect periodicity so Newton can wrap across domain boundaries.
    double uPeriod = 0, vPeriod = 0;
    DetectSurfacePeriodicity(brep, iLocalFace, uPeriod, vPeriod);

    double u = uvHint[0], v = uvHint[1];

    // Wrap into valid range for initial guess
    auto wrapToRange = [](double val, double lo, double hi, double period) -> double
    {
        if (period > 0)
        {
            while (val < lo)
            {
                val += period;
            }
            while (val > hi)
            {
                val -= period;
            }
            return std::clamp(val, lo, hi);
        }
        return std::clamp(val, lo, hi);
    };
    u = wrapToRange(u, uMin, uMax, uPeriod);
    v = wrapToRange(v, vMin, vMax, vPeriod);

    constexpr double h = kFiniteDiffStep;
    for (int iter = 0; iter < 20; ++iter)
    {
        GfVec3d S = EvaluateSurface(brep, iLocalFace, u, v);
        GfVec3d r = S - point;
        if (r.GetLengthSq() < kNewtonConvSq)
        {
            break;
        }
        GfVec3d Su = (EvaluateSurface(brep, iLocalFace, u + h, v) - S) / h;
        GfVec3d Sv = (EvaluateSurface(brep, iLocalFace, u, v + h) - S) / h;
        double a11 = GfDot(Su, Su), a12 = GfDot(Su, Sv), a22 = GfDot(Sv, Sv);
        double b1 = -GfDot(Su, r), b2 = -GfDot(Sv, r);
        double det = a11 * a22 - a12 * a12;
        if (std::fabs(det) < kDivEpsilon)
        {
            break;
        }
        double du = (a22 * b1 - a12 * b2) / det;
        double dv = (a11 * b2 - a12 * b1) / det;

        // Limit step to half the domain span to avoid overshooting.
        double maxDU = uSpan * 0.5;
        double maxDV = vSpan * 0.5;
        du = std::clamp(du, -maxDU, maxDU);
        dv = std::clamp(dv, -maxDV, maxDV);

        u += du;
        v += dv;

        // For periodic directions, wrap instead of clamping.
        if (uPeriod > 0)
        {
            while (u < uMin)
            {
                u += uPeriod;
            }
            while (u > uMax)
            {
                u -= uPeriod;
            }
        }
        u = std::clamp(u, uMin, uMax);

        if (vPeriod > 0)
        {
            while (v < vMin)
            {
                v += vPeriod;
            }
            while (v > vMax)
            {
                v -= vPeriod;
            }
        }
        v = std::clamp(v, vMin, vMax);
    }

    // If Newton converged to a local minimum far from the target
    // (common on cones/spheres where the hint is at the base but
    // the point is on a seam running toward the apex), fall back
    // to the full grid search which finds the global minimum.
    {
        GfVec3d Scheck = EvaluateSurface(brep, iLocalFace, u, v);
        double residualSq = (Scheck - point).GetLengthSq();
        if (residualSq > kNewtonConvSq)
        {
            GfVec3d S00 = EvaluateSurface(brep, iLocalFace, uMin, vMin);
            GfVec3d S11 = EvaluateSurface(brep, iLocalFace, uMax, vMax);
            double extentSq = (S11 - S00).GetLengthSq();
            if (extentSq < 1e-30)
            {
                extentSq = 1.0;
            }
            if (residualSq > extentSq * 1e-10)
            {
                return InverseProjectSurface(brep, iLocalFace, point);
            }
        }
    }

    {
        GfVec3d S = EvaluateSurface(brep, iLocalFace, u, v);
        double convergenceResidSq2 = (S - point).GetLengthSq();
        GfVec3d Su = (EvaluateSurface(brep, iLocalFace, u + h, v) - S) / h;
        GfVec3d Sv = (EvaluateSurface(brep, iLocalFace, u, v + h) - S) / h;
        double lenSu = Su.GetLengthSq();
        double lenSv = Sv.GetLengthSq();
        double threshold = kDegenDerivSq;
        if (lenSu < threshold && lenSv > threshold && convergenceResidSq2 > kNewtonConvSq)
        {
            double vOff = v + std::copysign(std::max(vSpan * 0.01, h * 100), vSpan * 0.5 - (v - vMin));
            vOff = std::clamp(vOff, vMin, vMax);
            double bestU = u, bestD = std::numeric_limits<double>::max();
            int N = 40;
            for (int i = 0; i <= N; ++i)
            {
                double uProbe = uMin + (uMax - uMin) * i / N;
                GfVec3d p = EvaluateSurface(brep, iLocalFace, uProbe, vOff);
                double d = (p - point).GetLengthSq();
                if (d < bestD)
                {
                    bestD = d;
                    bestU = uProbe;
                }
            }
            u = bestU;
        }
        else if (lenSv < threshold && lenSu > threshold && convergenceResidSq2 > kNewtonConvSq)
        {
            double uOff = u + std::copysign(std::max(uSpan * 0.01, h * 100), uSpan * 0.5 - (u - uMin));
            uOff = std::clamp(uOff, uMin, uMax);
            double bestV = v, bestD = std::numeric_limits<double>::max();
            int M = 40;
            for (int j = 0; j <= M; ++j)
            {
                double vProbe = vMin + (vMax - vMin) * j / M;
                GfVec3d p = EvaluateSurface(brep, iLocalFace, uOff, vProbe);
                double d = (p - point).GetLengthSq();
                if (d < bestD)
                {
                    bestD = d;
                    bestV = vProbe;
                }
            }
            v = bestV;
        }
    }

    // Snap to the periodic copy of the hint when the result is near a
    // period boundary.  On periodic surfaces (e.g. torus), Newton can
    // converge to U=0 or U=360 for the same 3D point.  smlib resolves
    // this with DropPoint's UV guess; we replicate that by shifting the
    // result to the copy closest to uvHint, but only when the result
    // is within 2% of a period boundary to avoid disrupting genuine
    // large parameter changes (e.g. V traversing a full minor circle).
    auto snapAxis = [](double val, double hint, double period)
    {
        if (period <= 0)
        {
            return val;
        }
        double diff = val - hint;
        double nPeriods = std::round(diff / period);
        if (nPeriods != 0.0 && std::fabs(diff - nPeriods * period) < period * 0.02)
        {
            val -= nPeriods * period;
        }
        return val;
    };
    u = snapAxis(u, uvHint[0], uPeriod);
    v = snapAxis(v, uvHint[1], vPeriod);

    return GfVec2d(u, v);
}

// ===== ComputeTrimmedDomain ==================================================================

UsdBrepSurfaceDomain ComputeTrimmedDomain(const UsdBrepView& brep, uint32_t iLocalFace, uint32_t samplesPerEdge)
{
    UsdBrepSurfaceDomain storedDomain = GetSurfaceDomain(brep, iLocalFace);

    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return storedDomain;
    }

    const TfToken& surfType = surfTypes[iLocalFace];

    auto faceLoopCounts = brep.FaceLoopCounts();
    uint32_t loopStart = 0;
    for (uint32_t f = 0; f < iLocalFace && f < faceLoopCounts.size(); ++f)
    {
        loopStart += faceLoopCounts[f];
    }

    uint32_t loopCount = (iLocalFace < faceLoopCounts.size()) ? faceLoopCounts[iLocalFace] : 0;

    if (loopCount == 0)
    {
        return storedDomain;
    }

    auto loopEuCounts = brep.LoopEdgeuseCounts();
    uint32_t euStart = 0;
    for (uint32_t l = 0; l < loopStart && l < loopEuCounts.size(); ++l)
    {
        euStart += loopEuCounts[l];
    }

    std::vector<double> uSamples;
    std::vector<double> vSamples;
    std::vector<GfVec2d> orderedBoundaryUV;
    double lastValidBoundaryU = std::numeric_limits<double>::quiet_NaN();

    auto euOrientations = brep.EdgeuseOrientationTypes();
    auto edgeCrvTypes = brep.EdgeCurveTypes();
    auto edgeRanges = brep.EdgeRanges();
    auto vtxPositions = brep.VertexPositions();

    for (uint32_t li = 0; li < loopCount; ++li)
    {
        uint32_t loopIdx = loopStart + li;
        if (loopIdx >= loopEuCounts.size())
        {
            break;
        }

        uint32_t euCount = loopEuCounts[loopIdx];

        for (uint32_t ei = 0; ei < euCount; ++ei)
        {
            uint32_t localEu = euStart + ei;
            if (localEu >= brep.EdgeuseCount())
            {
                break;
            }

            uint32_t localEdge = brep.EdgeuseLocalEdgeIndex(localEu);
            if (localEdge >= edgeCrvTypes.size() || 2 * localEdge + 1 >= edgeRanges.size())
            {
                continue;
            }

            double tStart = edgeRanges[2 * localEdge];
            double tEnd = edgeRanges[2 * localEdge + 1];

            bool reversed = false;
            if (localEu < euOrientations.size() && euOrientations[localEu] == UsdBrepSolidTokens->opposite)
            {
                reversed = true;
            }

            for (uint32_t s = 0; s <= samplesPerEdge; ++s)
            {
                double frac = static_cast<double>(s) / static_cast<double>(samplesPerEdge);
                if (reversed)
                {
                    frac = 1.0 - frac;
                }
                double t = tStart + frac * (tEnd - tStart);
                GfVec3d pt3d = EvaluateCurve3d(brep, localEdge, t);
                GfVec2d uv = InverseProjectSurface(brep, iLocalFace, pt3d);

                if (!std::isnan(uv[0]))
                {
                    uSamples.push_back(uv[0]);
                    lastValidBoundaryU = uv[0];
                }
                else if (!std::isnan(lastValidBoundaryU))
                {
                    uv[0] = lastValidBoundaryU;
                }
                if (!std::isnan(uv[1]))
                {
                    vSamples.push_back(uv[1]);
                }
                if (!std::isnan(uv[0]) && !std::isnan(uv[1]))
                {
                    orderedBoundaryUV.push_back(uv);
                }
            }

            if (localEdge < brep.EdgeCount())
            {
                GfVec2i verts = brep.EdgeLocalVertexIndices(localEdge);
                for (uint32_t vi = 0; vi < 2; ++vi)
                {
                    uint32_t localVtx = static_cast<uint32_t>(verts[vi]);
                    if (localVtx < vtxPositions.size())
                    {
                        GfVec3d vtxPos = vtxPositions[localVtx];
                        GfVec2d uv = InverseProjectSurface(brep, iLocalFace, vtxPos);
                        if (!std::isnan(uv[0]))
                        {
                            uSamples.push_back(uv[0]);
                        }
                        if (!std::isnan(uv[1]))
                        {
                            vSamples.push_back(uv[1]);
                        }
                    }
                }
            }
        }

        euStart += euCount;
    }

    if (uSamples.empty() && vSamples.empty())
    {
        return storedDomain;
    }

    struct PeriodicResult
    {
        double min;
        double max;
        double maxGap;
        size_t numSamples;
    };

    // For periodic parameters, find the tightest arc that contains all samples.
    // Use the "largest gap" method: sort angles, find the largest gap between
    // consecutive angles, and the arc complement of that gap is the trimmed range.
    auto computePeriodicRange = [](std::vector<double>& angles, double period) -> PeriodicResult
    {
        for (double& a : angles)
        {
            a = std::fmod(a, period);
            if (a < 0.0)
            {
                a += period;
            }
        }

        std::sort(angles.begin(), angles.end());

        auto last = std::unique(
            angles.begin(),
            angles.end(),
            [](double a, double b)
            {
                return std::fabs(a - b) < kAngleEpsilon;
            }
        );
        angles.erase(last, angles.end());

        size_t N = angles.size();
        if (N <= 1)
        {
            return { angles.empty() ? 0.0 : angles[0], angles.empty() ? period : angles[0], period, N };
        }

        double maxGap = 0.0;
        size_t maxGapIdx = 0;
        for (size_t i = 1; i < N; ++i)
        {
            double gap = angles[i] - angles[i - 1];
            if (gap > maxGap)
            {
                maxGap = gap;
                maxGapIdx = i;
            }
        }
        double wrapGap = (period - angles.back()) + angles.front();
        double actualMaxGap = std::max(maxGap, wrapGap);

        if (wrapGap >= maxGap)
        {
            return { angles.front(), angles.back(), actualMaxGap, N };
        }
        else
        {
            return { angles[maxGapIdx], angles[maxGapIdx - 1] + period, actualMaxGap, N };
        }
    };

    UsdBrepSurfaceDomain trimmed = storedDomain;

    auto shouldOverridePeriodic = [](const PeriodicResult& r, double storedMin, double storedMax, double period) -> bool
    {
        if (r.numSamples < 2)
        {
            return false;
        }
        double avgGap = period / static_cast<double>(r.numSamples);
        if (r.maxGap > 2.0 * avgGap)
        {
            return true;
        }
        double computedSpan = r.max - r.min;
        double storedSpan = storedMax - storedMin;
        if (storedSpan > 0.0 && computedSpan < storedSpan * 0.9)
        {
            return true;
        }
        return false;
    };

    // Detect periodicity (handles both analytic types and NURBS runtime detection).
    double dynPeriodU = 0, dynPeriodV = 0;
    DetectSurfacePeriodicity(brep, iLocalFace, dynPeriodU, dynPeriodV);
    bool dynPeriodicU = (dynPeriodU > 0);
    bool dynPeriodicV = (dynPeriodV > 0);

    bool uOverridden = false;

    if (!uSamples.empty())
    {
        if (dynPeriodicU)
        {
            auto result = computePeriodicRange(uSamples, dynPeriodU);
            if (shouldOverridePeriodic(result, storedDomain.uvMin[0], storedDomain.uvMax[0], dynPeriodU))
            {
                trimmed.uvMin[0] = result.min;
                trimmed.uvMax[0] = result.max;
                uOverridden = true;
            }
        }
        else
        {
            double uMin = *std::min_element(uSamples.begin(), uSamples.end());
            double uMax = *std::max_element(uSamples.begin(), uSamples.end());
            trimmed.uvMin[0] = uMin;
            trimmed.uvMax[0] = uMax;
        }
    }

    if (!vSamples.empty())
    {
        if (dynPeriodicV)
        {
            auto result = computePeriodicRange(vSamples, dynPeriodV);
            if (shouldOverridePeriodic(result, storedDomain.uvMin[1], storedDomain.uvMax[1], dynPeriodV))
            {
                trimmed.uvMin[1] = result.min;
                trimmed.uvMax[1] = result.max;
            }
        }
        else
        {
            double vMin = *std::min_element(vSamples.begin(), vSamples.end());
            double vMax = *std::max_element(vSamples.begin(), vSamples.end());
            trimmed.uvMin[1] = vMin;
            trimmed.uvMax[1] = vMax;
        }
    }

    // Periodic range correction: the "largest gap" method may have picked
    // the wrong half of the angular range.  Use signed area of the ordered
    // boundary polygon to verify: if CW, swap to the other half.
    if (uOverridden && dynPeriodicU && orderedBoundaryUV.size() >= 3)
    {
        bool hasPoles = (surfType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI || surfType == UsdBrepSurfaceTokens->brepSurfaceConeAPI);
        if (!hasPoles && dynPeriodicU)
        {
            bool pVMin = false, pVMax = false;
            DetectSurfacePoles(brep, iLocalFace, pVMin, pVMax);
            hasPoles = pVMin || pVMax;
        }

        std::vector<GfVec2d> unwrapped = orderedBoundaryUV;

        if (hasPoles && unwrapped.size() > 3)
        {
            double poleTol = std::fabs(storedDomain.uvMax[1] - storedDomain.uvMin[1]) * kPoleFraction;
            double poleVMin = storedDomain.uvMin[1] + poleTol;
            double poleVMax = storedDomain.uvMax[1] - poleTol;
            std::vector<GfVec2d> filtered;
            filtered.reserve(unwrapped.size());
            for (const auto& uv : unwrapped)
            {
                if (uv[1] >= poleVMin && uv[1] <= poleVMax)
                {
                    filtered.push_back(uv);
                }
            }
            if (filtered.size() >= 3)
            {
                unwrapped = std::move(filtered);
            }
        }

        double period = dynPeriodU;
        for (size_t i = 1; i < unwrapped.size(); ++i)
        {
            double du = unwrapped[i][0] - unwrapped[i - 1][0];
            if (du > period * 0.5)
            {
                unwrapped[i][0] -= period;
            }
            else if (du < -period * 0.5)
            {
                unwrapped[i][0] += period;
            }
        }
        double signedArea = SignedArea2D(unwrapped);
        if (signedArea < 0.0)
        {
            double span = trimmed.uvMax[0] - trimmed.uvMin[0];
            trimmed.uvMin[0] = trimmed.uvMax[0];
            trimmed.uvMax[0] = trimmed.uvMin[0] + (period - span);
        }
    }

    return trimmed;
}

// ===== Shared geometry helpers ================================================================

double SignedArea2D(const std::vector<GfVec2d>& polygon)
{
    double area = 0.0;
    size_t N = polygon.size();
    if (N < 3)
    {
        return 0.0;
    }
    for (size_t i = 0; i < N; ++i)
    {
        size_t j = (i + 1) % N;
        area += polygon[i][0] * polygon[j][1] - polygon[j][0] * polygon[i][1];
    }
    return area * 0.5;
}

void DetectSurfacePeriodicity(const UsdBrepView& brep, uint32_t iLocalFace, double& oUPeriod, double& oVPeriod)
{
    oUPeriod = 0.0;
    oVPeriod = 0.0;

    auto surfTypes = brep.FaceSurfaceTypes();
    if (iLocalFace >= surfTypes.size())
    {
        return;
    }

    const TfToken& surfType = surfTypes[iLocalFace];

    bool uPer = false, vPer = false;
    GetSurfacePeriodicity(surfType, uPer, vPer, oUPeriod, oVPeriod);

    if (surfType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        UsdBrepSurfaceDomain dom = GetSurfaceDomain(brep, iLocalFace);
        double uSpan = dom.uvMax[0] - dom.uvMin[0];
        double vSpan = dom.uvMax[1] - dom.uvMin[1];

        if (uSpan > kSpanEpsilon)
        {
            double vMid = (dom.uvMin[1] + dom.uvMax[1]) * 0.5;
            GfVec3d pU0 = EvaluateSurface(brep, iLocalFace, dom.uvMin[0], vMid);
            GfVec3d pU1 = EvaluateSurface(brep, iLocalFace, dom.uvMax[0], vMid);
            if ((pU1 - pU0).GetLength() < kPositionCoincidence)
            {
                oUPeriod = uSpan;
            }
        }
        if (vSpan > kSpanEpsilon)
        {
            double uMid = (dom.uvMin[0] + dom.uvMax[0]) * 0.5;
            GfVec3d pV0 = EvaluateSurface(brep, iLocalFace, uMid, dom.uvMin[1]);
            GfVec3d pV1 = EvaluateSurface(brep, iLocalFace, uMid, dom.uvMax[1]);
            if ((pV1 - pV0).GetLength() < kPositionCoincidence)
            {
                oVPeriod = vSpan;
            }
        }
    }
}

void DetectSurfacePoles(const UsdBrepView& brep, uint32_t iLocalFace, bool& oHasPoleVMin, bool& oHasPoleVMax)
{
    oHasPoleVMin = false;
    oHasPoleVMax = false;

    UsdBrepSurfaceDomain stored = GetSurfaceDomain(brep, iLocalFace);
    double uSpan = stored.uvMax[0] - stored.uvMin[0];
    if (uSpan < kSpanEpsilon)
    {
        return;
    }

    double u1 = stored.uvMin[0] + uSpan * 0.25;
    double u2 = stored.uvMin[0] + uSpan * 0.75;

    GfVec3d pMin1 = EvaluateSurface(brep, iLocalFace, u1, stored.uvMin[1]);
    GfVec3d pMin2 = EvaluateSurface(brep, iLocalFace, u2, stored.uvMin[1]);
    oHasPoleVMin = (pMin2 - pMin1).GetLength() < kPositionCoincidence;

    GfVec3d pMax1 = EvaluateSurface(brep, iLocalFace, u1, stored.uvMax[1]);
    GfVec3d pMax2 = EvaluateSurface(brep, iLocalFace, u2, stored.uvMax[1]);
    oHasPoleVMax = (pMax2 - pMax1).GetLength() < kPositionCoincidence;
}

bool GetFaceLoopEdgeuseStart(const UsdBrepView& brep, uint32_t iLocalFace, uint32_t localLoopIndex, uint32_t& oEuStart, uint32_t& oEuCount)
{
    oEuStart = 0;
    oEuCount = 0;

    auto faceLoopCounts = brep.FaceLoopCounts();
    if (iLocalFace >= faceLoopCounts.size())
    {
        return false;
    }

    uint32_t loopStart = 0;
    for (uint32_t f = 0; f < iLocalFace; ++f)
    {
        loopStart += faceLoopCounts[f];
    }

    uint32_t loopCount = faceLoopCounts[iLocalFace];
    if (localLoopIndex >= loopCount)
    {
        return false;
    }

    auto loopEuCounts = brep.LoopEdgeuseCounts();
    uint32_t targetLoop = loopStart + localLoopIndex;
    if (targetLoop >= loopEuCounts.size())
    {
        return false;
    }

    for (uint32_t l = 0; l < targetLoop; ++l)
    {
        oEuStart += loopEuCounts[l];
    }

    oEuCount = loopEuCounts[targetLoop];
    return true;
}

// Walk region -> shell -> faceuse to find the faceuse for a given face.
// Prefers solid regions over void regions.  Returns the region type flags
// and whether the faceuse orientation is "opposite".
struct FaceuseInfo
{
    bool found = false;
    bool isSolid = false;
    bool isVoid = false;
    bool isOpposite = false;
};

static FaceuseInfo FindFaceuseForFace(const UsdBrepView& brep, uint32_t iLocalFace)
{
    auto regionTypes = brep.RegionTypes();
    auto regionShellCounts = brep.RegionShellCounts();
    auto shellFuCounts = brep.ShellFaceuseCounts();
    auto fuOrientations = brep.FaceuseOrientationTypes();

    uint32_t fuBase = 0;
    uint32_t shellBase = 0;
    uint32_t regionCount = static_cast<uint32_t>(regionTypes.size());

    FaceuseInfo result;
    bool foundSolid = false;

    for (uint32_t r = 0; r < regionCount; ++r)
    {
        bool isSolid = (regionTypes[r] == UsdBrepSolidTokens->solidRegion);
        bool isVoid = (regionTypes[r] == UsdBrepSolidTokens->voidRegion);

        uint32_t shellCount = (r < regionShellCounts.size()) ? regionShellCounts[r] : 0u;

        for (uint32_t s = 0; s < shellCount; ++s)
        {
            uint32_t sIdx = shellBase + s;
            uint32_t fuCount = (sIdx < shellFuCounts.size()) ? shellFuCounts[sIdx] : 0u;

            for (uint32_t fu = 0; fu < fuCount; ++fu)
            {
                uint32_t fIdx = fuBase + fu;
                if (fIdx < brep.FaceuseCount() && brep.FaceuseLocalFaceIndex(fIdx) == iLocalFace && (isSolid || isVoid))
                {
                    bool isOpp = (fIdx < fuOrientations.size() && fuOrientations[fIdx] == UsdBrepSolidTokens->opposite);

                    if (isSolid || !result.found)
                    {
                        result.found = true;
                        result.isSolid = isSolid;
                        result.isVoid = isVoid;
                        result.isOpposite = isOpp;
                        foundSolid = isSolid;
                    }
                    if (foundSolid)
                    {
                        break;
                    }
                }
            }
            fuBase += fuCount;
        }
        shellBase += shellCount;
        if (foundSolid)
        {
            break;
        }
    }

    return result;
}

bool IsFaceNormalOutward(const UsdBrepView& brep, uint32_t iLocalFace)
{
    FaceuseInfo info = FindFaceuseForFace(brep, iLocalFace);
    if (!info.found)
    {
        return true;
    }
    return (info.isSolid && info.isOpposite) || (info.isVoid && !info.isOpposite);
}

bool IsFaceuseOpposite(const UsdBrepView& brep, uint32_t iLocalFace)
{
    return FindFaceuseForFace(brep, iLocalFace).isOpposite;
}

} // end namespace UsdBrep
