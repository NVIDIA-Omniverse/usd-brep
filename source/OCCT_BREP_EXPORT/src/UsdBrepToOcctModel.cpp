// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Build an OCCT SOcctModel from a UsdBrepArrayData.
//
// The USD staging model represents a closed solid as a solidRegion plus a nested voidRegion,
// with two faceuses per face (one in each region's shell). To recover the original OCCT shape
// we pair those shells by face membership and emit the solidRegion side once. Unpaired void-region
// shells are real open components and are preserved. Pcurve records use the best available source:
// compatible authored UV trims first, then analytic reconstruction. Topology records are appended
// child-first so every referenced shape precedes its parent; sub-shape file tokens are derived later
// by the writer.

#include "UsdBrepToOcctModel.h"

#include "UsdBrepGeometryEval.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes
#include "UsdBrepIterator.h"
#include "UsdBrepKnots.h"
#include "UsdBrepTokens.h"

#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec2i.h>
#include <pxr/base/gf/vec3d.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

namespace occt
{
namespace
{

PXR_NAMESPACE_USING_DIRECTIVE
using UsdBrep::UsdBrepView;
using UsdBrepData::UsdBrepArrayData;

SPnt3 ToPnt3(const GfVec3d& v)
{
    return SPnt3{ v[0], v[1], v[2] };
}
SDir3 ToDir3(const GfVec3d& v)
{
    return SDir3{ v[0], v[1], v[2] };
}

// OCCT stores an Ax3 as location + main axis (Z) + X axis; Y is the right-handed completion.
SDir3 CrossDir(const GfVec3d& axis, const GfVec3d& refDir)
{
    return ToDir3(GfCross(axis, refDir));
}

// Per-shape OCCT flag bytes chosen to mirror what OCCT writes for a closed solid (matches the
// corpus); flags do not affect topology resolution on re-import.
void SetVertexFlags(SShapeRecord& r)
{
    r.bModified = r.bOrientable = r.bClosed = r.bConvex = true;
}
void SetEdgeFlags(SShapeRecord& r)
{
    r.bModified = r.bOrientable = true;
}
void SetWireFlags(SShapeRecord& r)
{
    r.bModified = r.bOrientable = r.bClosed = true;
}
void SetFaceFlags(SShapeRecord& r)
{
    r.bModified = r.bChecked = r.bOrientable = true;
}
void SetShellFlags(SShapeRecord& r)
{
    r.bModified = r.bOrientable = r.bClosed = true;
}
void SetSolidFlags(SShapeRecord& r)
{
    r.bFree = r.bModified = true;
}

SSubShapeRef Ref(int iStorageIndex, EOrientation eOrient)
{
    SSubShapeRef s;
    s.iStorageIndex = iStorageIndex;
    s.eOrientation = eOrient;
    s.iLocation = 0;
    return s;
}

constexpr double kPi = 3.141592653589793;
constexpr double kTwoPi = 6.283185307179586;
// A seam's iso parameter lies exactly on the period boundary (0 == 2*pi). Its fitted value therefore
// comes out as 0 +/- round-off, and the sign of that round-off is platform-dependent. Values within
// this tolerance of a period boundary are snapped to 0 so the period-wrap below is deterministic.
constexpr double kSeamSnapTol = 1e-6;

GfVec3d ToVec(const SPnt3& p)
{
    return GfVec3d(p.dX, p.dY, p.dZ);
}
GfVec3d ToVec(const SDir3& d)
{
    return GfVec3d(d.dX, d.dY, d.dZ);
}

// Evaluate an analytic 3D curve (line/circle/ellipse) at parameter t.
GfVec3d EvalCurve3dPoint(const SCurve3d& c, double t)
{
    const GfVec3d O = ToVec(c.sLocation);
    switch (c.eType)
    {
        case ECurveType::Line:
            return O + t * ToVec(c.sXAxis);
        case ECurveType::Circle:
            return O + c.dRadius * (std::cos(t) * ToVec(c.sXAxis) + std::sin(t) * ToVec(c.sYAxis));
        case ECurveType::Ellipse:
            return O + c.dRadius * std::cos(t) * ToVec(c.sXAxis) + c.dMinorRadius * std::sin(t) * ToVec(c.sYAxis);
        default:
            return O;
    }
}

bool SurfaceUPeriodic(const SSurface& s)
{
    return s.eType == ESurfaceType::Cylinder || s.eType == ESurfaceType::Cone || s.eType == ESurfaceType::Sphere || s.eType == ESurfaceType::Torus;
}
bool SurfaceVPeriodic(const SSurface& s)
{
    return s.eType == ESurfaceType::Torus;
}

// Inverse parametrization: map a 3D point on an analytic surface back to its (u,v) parameters,
// using OCCT's canonical frames. Returns false for surfaces with no closed-form pcurve need (plane).
bool InvertSurface(const SSurface& s, const GfVec3d& P, double& u, double& v, double* pURadial = nullptr)
{
    const GfVec3d O = ToVec(s.sLocation);
    const GfVec3d X = ToVec(s.sXAxis);
    const GfVec3d Y = ToVec(s.sYAxis);
    const GfVec3d Z = ToVec(s.sAxis);
    const GfVec3d w = P - O;
    const double wx = GfDot(w, X);
    const double wy = GfDot(w, Y);
    const double wz = GfDot(w, Z);

    // u = atan2(wy, wx) is ill-conditioned as the point approaches the rotation axis (a cone apex or
    // sphere pole): wx, wy -> 0 there, so u is undefined and noise-dominated. Report the radial
    // distance so callers can drop these samples from a u fit.
    if (pURadial != nullptr)
    {
        *pURadial = std::sqrt(wx * wx + wy * wy);
    }

    switch (s.eType)
    {
        case ESurfaceType::Cylinder:
            u = std::atan2(wy, wx);
            v = wz;
            return true;
        case ESurfaceType::Cone:
        {
            const double dCos = std::cos(s.dSemiAngle);
            u = std::atan2(wy, wx);
            v = (std::fabs(dCos) > 1e-12) ? wz / dCos : wz;
            return true;
        }
        case ESurfaceType::Sphere:
        {
            double dSin = (s.dRadius != 0.0) ? wz / s.dRadius : 0.0;
            dSin = std::max(-1.0, std::min(1.0, dSin));
            v = std::asin(dSin);
            u = std::atan2(wy, wx);
            return true;
        }
        case ESurfaceType::Torus:
        {
            u = std::atan2(wy, wx);
            const double dRho = std::sqrt(wx * wx + wy * wy) - s.dRadius;
            v = std::atan2(wz, dRho);
            return true;
        }
        default:
            return false;
    }
}

// Project an analytic edge curve onto a curved analytic surface, producing the 2D pcurve as an OCCT
// line "point + t*dir" whose parameter equals the 3D edge parameter (samePar). The corpus edges on
// curved faces are iso-parametric, so the (u,v)(t) map is affine; we recover it by sampling the 3D
// curve, unwrapping the periodic parameter(s), and least-squares fitting a line. Plane faces return
// false (the importer derives their UV domain by projecting the 3D edges directly).
bool ProjectCurveToSurfaceLine(const SSurface& surf, const SCurve3d& curve, double t0, double t1, SCurve2d& rOut)
{
    if (surf.eType == ESurfaceType::Plane)
    {
        return false;
    }

    constexpr int N = 16;
    const bool bUPer = SurfaceUPeriodic(surf);
    const bool bVPer = SurfaceVPeriodic(surf);

    std::vector<double> ts(N + 1), us(N + 1), vs(N + 1);
    std::vector<bool> bUValid(N + 1, true);
    double dUPrev = 0.0;
    double dVPrev = 0.0;
    bool bHavePrevU = false;
    for (int i = 0; i <= N; ++i)
    {
        const double t = (N == 0) ? t0 : t0 + (t1 - t0) * (static_cast<double>(i) / static_cast<double>(N));
        double u = 0.0;
        double v = 0.0;
        double dURadial = 0.0;
        if (!InvertSurface(surf, EvalCurve3dPoint(curve, t), u, v, &dURadial))
        {
            return false;
        }
        // Near the rotation axis u degenerates (an apex/pole sample); exclude it from the u fit so a
        // seam edge that terminates at a cone apex still fits to a clean iso-u line instead of picking
        // up a platform-dependent spurious slope from the noisy near-axis angle.
        const bool bUOk = dURadial > 1e-7;
        bUValid[i] = bUOk;
        // Unwrap the periodic parameter relative to the previous *well-defined* value, so a skipped
        // degenerate sample does not break the 2*pi continuity of the ones that matter.
        if (bUOk && bHavePrevU && bUPer)
        {
            while (u - dUPrev > kPi)
            {
                u -= kTwoPi;
            }
            while (u - dUPrev < -kPi)
            {
                u += kTwoPi;
            }
        }
        if (i > 0 && bVPer)
        {
            while (v - dVPrev > kPi)
            {
                v -= kTwoPi;
            }
            while (v - dVPrev < -kPi)
            {
                v += kTwoPi;
            }
        }
        ts[i] = t;
        us[i] = u;
        vs[i] = v;
        if (bUOk)
        {
            dUPrev = u;
            bHavePrevU = true;
        }
        dVPrev = v;
    }

    // Least-squares fit of y(t) = a + b*t over the samples whose mask bit is set (all of them for v;
    // only the non-degenerate ones for u).
    auto fitAffine = [&](const std::vector<double>& ys, const std::vector<bool>& mask, double& a, double& b)
    {
        double tm = 0.0;
        double ym = 0.0;
        int n = 0;
        for (int i = 0; i <= N; ++i)
        {
            if (!mask[i])
            {
                continue;
            }
            tm += ts[i];
            ym += ys[i];
            ++n;
        }
        if (n == 0)
        {
            a = 0.0;
            b = 0.0;
            return;
        }
        tm /= n;
        ym /= n;
        double num = 0.0;
        double den = 0.0;
        for (int i = 0; i <= N; ++i)
        {
            if (!mask[i])
            {
                continue;
            }
            num += (ts[i] - tm) * (ys[i] - ym);
            den += (ts[i] - tm) * (ts[i] - tm);
        }
        b = (std::fabs(den) > 1e-30) ? num / den : 0.0;
        a = ym - b * tm;
    };

    const std::vector<bool> bAllValid(N + 1, true);
    double au = 0.0;
    double bu = 0.0;
    double av = 0.0;
    double bv = 0.0;
    fitAffine(us, bUValid, au, bu);
    fitAffine(vs, bAllValid, av, bv);

    rOut = SCurve2d{};
    rOut.eType = ECurveType::Line;
    rOut.sLocation = SPnt2{ au, av };
    rOut.sXAxis = SDir2{ bu, bv };
    return true;
}

// Build a 2D NURBS pcurve (SCurve2d BSpline) from an edgeuse's stored UV trim curve. Returns false
// when the edgeuse has no stored UV curve. This inverts how the importer authors per-edgeuse UV trim
// curves for faces whose pcurves USD persists (NURBS surfaces in particular).
bool BuildPCurve2dFromEdgeuse(const UsdBrepView& brep, uint32_t eu, SCurve2d& rOut)
{
    auto vcs = brep.EdgeuseUVNurbsVertexCounts();
    auto ords = brep.EdgeuseUVNurbsOrders();
    if (eu >= vcs.size() || eu >= ords.size())
    {
        return false;
    }
    const uint32_t vc = vcs[eu];
    const uint32_t ord = ords[eu];
    if (vc == 0 || ord == 0)
    {
        return false;
    }

    uint32_t cpStart = 0;
    uint32_t kStart = 0;
    for (uint32_t i = 0; i < eu; ++i)
    {
        cpStart += vcs[i];
        kStart += vcs[i] + ords[i];
    }

    auto cps = brep.EdgeuseUVNurbsControlVertices();
    auto knots = brep.EdgeuseUVNurbsKnots();
    auto wts = brep.EdgeuseUVNurbsWeights();

    // Guard the poles/knots reads against vertex/order metadata that is inconsistent with the flat
    // arrays (partially written / corrupted BrepArray), matching the weight-copy guard below.
    if (cpStart + vc > cps.size() || kStart + vc + ord > knots.size())
    {
        return false;
    }

    rOut = SCurve2d{};
    rOut.eType = ECurveType::BSpline;
    rOut.iDegree = static_cast<int>(ord) - 1;
    rOut.bPeriodic = false;
    for (uint32_t i = 0; i < vc; ++i)
    {
        const GfVec2d p = cps[cpStart + i];
        rOut.sPoles.push_back(SPnt2{ p[0], p[1] });
    }

    bool bRational = false;
    for (uint32_t i = 0; i < vc && cpStart + i < wts.size(); ++i)
    {
        if (std::fabs(wts[cpStart + i] - 1.0) > 1e-12)
        {
            bRational = true;
            break;
        }
    }
    rOut.bRational = bRational;
    if (bRational)
    {
        for (uint32_t i = 0; i < vc && cpStart + i < wts.size(); ++i)
        {
            rOut.dWeights.push_back(wts[cpStart + i]);
        }
    }

    std::vector<double> flat;
    for (uint32_t i = 0; i < vc + ord; ++i)
    {
        flat.push_back(knots[kStart + i]);
    }
    UsdBrep::CompressFlatKnots(flat, rOut.dKnots, rOut.iMults);
    return true;
}

void ReverseBSplineCurve2d(SCurve2d& rCurve)
{
    std::reverse(rCurve.sPoles.begin(), rCurve.sPoles.end());
    std::reverse(rCurve.dWeights.begin(), rCurve.dWeights.end());
    std::reverse(rCurve.iMults.begin(), rCurve.iMults.end());
    if (!rCurve.dKnots.empty())
    {
        const double dKnotSum = rCurve.dKnots.front() + rCurve.dKnots.back();
        std::reverse(rCurve.dKnots.begin(), rCurve.dKnots.end());
        for (double& rKnot : rCurve.dKnots)
        {
            rKnot = dKnotSum - rKnot;
        }
    }
}

// Build the analytic SSurface for a local face. Returns false for non-MVP surface types.
bool BuildSurface(const UsdBrepView& brep, uint32_t f, SSurface& rS, std::string& rError)
{
    const TfToken t = brep.FaceSurfaceType(f);
    const uint32_t s = brep.FaceTypeSubIndex(f);

    if (t == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
    {
        auto origins = brep.PlaneSurfaceOrigins();
        auto axes = brep.PlaneSurfaceAxes();
        auto refDirs = brep.PlaneSurfaceRefDirections();
        if (s >= origins.size() || s >= axes.size() || s >= refDirs.size())
        {
            rError = "plane surface index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rS.eType = ESurfaceType::Plane;
        rS.sLocation = ToPnt3(origins[s]);
        rS.sAxis = ToDir3(ax);
        rS.sXAxis = ToDir3(rd);
        rS.sYAxis = CrossDir(ax, rd);
        return true;
    }
    if (t == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
    {
        auto origins = brep.CylinderSurfaceOrigins();
        auto axes = brep.CylinderSurfaceAxes();
        auto refDirs = brep.CylinderSurfaceRefDirections();
        auto radii = brep.CylinderSurfaceRadii();
        if (s >= origins.size() || s >= axes.size() || s >= refDirs.size() || s >= radii.size())
        {
            rError = "cylinder surface index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rS.eType = ESurfaceType::Cylinder;
        rS.sLocation = ToPnt3(origins[s]);
        rS.sAxis = ToDir3(ax);
        rS.sXAxis = ToDir3(rd);
        rS.sYAxis = CrossDir(ax, rd);
        rS.dRadius = radii[s];
        return true;
    }
    if (t == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
    {
        auto origins = brep.ConeSurfaceOrigins();
        auto axes = brep.ConeSurfaceAxes();
        auto refDirs = brep.ConeSurfaceRefDirections();
        auto radii = brep.ConeSurfaceRadii();
        auto semiAngles = brep.ConeSurfaceSemiAngles();
        if (s >= origins.size() || s >= axes.size() || s >= refDirs.size() || s >= radii.size() || s >= semiAngles.size())
        {
            rError = "cone surface index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rS.eType = ESurfaceType::Cone;
        rS.sLocation = ToPnt3(origins[s]);
        rS.sAxis = ToDir3(ax);
        rS.sXAxis = ToDir3(rd);
        rS.sYAxis = CrossDir(ax, rd);
        rS.dRadius = radii[s];
        rS.dSemiAngle = semiAngles[s];
        return true;
    }
    if (t == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        auto centers = brep.SphereSurfaceCenters();
        auto axes = brep.SphereSurfaceAxes();
        auto refDirs = brep.SphereSurfaceRefDirections();
        auto radii = brep.SphereSurfaceRadii();
        if (s >= centers.size() || s >= axes.size() || s >= refDirs.size() || s >= radii.size())
        {
            rError = "sphere surface index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rS.eType = ESurfaceType::Sphere;
        rS.sLocation = ToPnt3(centers[s]);
        rS.sAxis = ToDir3(ax);
        rS.sXAxis = ToDir3(rd);
        rS.sYAxis = CrossDir(ax, rd);
        rS.dRadius = radii[s];
        return true;
    }
    if (t == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
    {
        auto origins = brep.TorusSurfaceOrigins();
        auto axes = brep.TorusSurfaceAxes();
        auto refDirs = brep.TorusSurfaceRefDirections();
        auto majorRadii = brep.TorusSurfaceMajorRadii();
        auto minorRadii = brep.TorusSurfaceMinorRadii();
        if (s >= origins.size() || s >= axes.size() || s >= refDirs.size() || s >= majorRadii.size() || s >= minorRadii.size())
        {
            rError = "torus surface index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rS.eType = ESurfaceType::Torus;
        rS.sLocation = ToPnt3(origins[s]);
        rS.sAxis = ToDir3(ax);
        rS.sXAxis = ToDir3(rd);
        rS.sYAxis = CrossDir(ax, rd);
        rS.dRadius = majorRadii[s];
        rS.dMinorRadius = minorRadii[s];
        return true;
    }
    if (t == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        // USD stores every non-analytic surface (bspline/bezier/extrusion/revolution, periodic
        // unrolled) as a non-periodic NURBS with U-major poles and flat U/V knots. Invert the
        // importer's BuildFlatKnots with CompressFlatKnots in each direction.
        auto uvcs = brep.NurbsSurfaceUVertexCounts();
        auto vvcs = brep.NurbsSurfaceVVertexCounts();
        auto uords = brep.NurbsSurfaceUOrders();
        auto vords = brep.NurbsSurfaceVOrders();
        if (s >= uvcs.size() || s >= vvcs.size() || s >= uords.size() || s >= vords.size())
        {
            rError = "NURBS surface index out of range";
            return false;
        }
        const uint32_t uvc = uvcs[s];
        const uint32_t vvc = vvcs[s];
        const uint32_t uord = uords[s];
        const uint32_t vord = vords[s];

        uint32_t cpStart = 0;
        uint32_t ukStart = 0;
        uint32_t vkStart = 0;
        for (uint32_t i = 0; i < s; ++i)
        {
            cpStart += uvcs[i] * vvcs[i];
            ukStart += uvcs[i] + uords[i];
            vkStart += vvcs[i] + vords[i];
        }

        auto cps = brep.NurbsSurfaceControlVertices();
        auto uknots = brep.NurbsSurfaceUKnots();
        auto vknots = brep.NurbsSurfaceVKnots();
        auto wts = brep.NurbsSurfaceWeights();

        rS.eType = ESurfaceType::BSpline;
        rS.iUDegree = static_cast<int>(uord) - 1;
        rS.iVDegree = static_cast<int>(vord) - 1;
        rS.iNbUPoles = static_cast<int>(uvc);
        rS.iNbVPoles = static_cast<int>(vvc);
        rS.bUPeriodic = false;
        rS.bVPeriodic = false;

        const uint32_t nP = uvc * vvc;
        // Guard poles/knots reads against per-surface metadata inconsistent with the flat arrays.
        if (cpStart + nP > cps.size() || ukStart + uvc + uord > uknots.size() || vkStart + vvc + vord > vknots.size())
        {
            rError = "NURBS surface data arrays too short";
            return false;
        }
        rS.sPoles.clear();
        for (uint32_t i = 0; i < nP; ++i)
        {
            rS.sPoles.push_back(ToPnt3(cps[cpStart + i]));
        }

        bool bRational = false;
        for (uint32_t i = 0; i < nP && i + cpStart < wts.size(); ++i)
        {
            if (std::fabs(wts[cpStart + i] - 1.0) > 1e-12)
            {
                bRational = true;
                break;
            }
        }
        rS.bURational = bRational;
        rS.bVRational = bRational;
        if (bRational)
        {
            rS.dWeights.clear();
            for (uint32_t i = 0; i < nP && cpStart + i < wts.size(); ++i)
            {
                rS.dWeights.push_back(wts[cpStart + i]);
            }
        }

        std::vector<double> uflat;
        for (uint32_t i = 0; i < uvc + uord; ++i)
        {
            uflat.push_back(uknots[ukStart + i]);
        }
        std::vector<double> vflat;
        for (uint32_t i = 0; i < vvc + vord; ++i)
        {
            vflat.push_back(vknots[vkStart + i]);
        }
        UsdBrep::CompressFlatKnots(uflat, rS.dUKnots, rS.iUMults);
        UsdBrep::CompressFlatKnots(vflat, rS.dVKnots, rS.iVMults);
        return true;
    }

    rError = "unsupported surface type for OCCT export: " + t.GetString();
    return false;
}

// Build the analytic SCurve3d for a local edge. Returns false for non-MVP curve types.
bool BuildCurve3d(const UsdBrepView& brep, uint32_t e, SCurve3d& rC, std::string& rError)
{
    const TfToken t = brep.EdgeCurveType(e);
    const uint32_t s = brep.EdgeTypeSubIndex(e);

    if (t == UsdBrepCurveTokens->brepCurve3dLineAPI)
    {
        auto origins = brep.EdgeLineOrigins();
        auto dirs = brep.EdgeLineDirections();
        if (s >= origins.size() || s >= dirs.size())
        {
            rError = "line curve index out of range";
            return false;
        }
        rC.eType = ECurveType::Line;
        rC.sLocation = ToPnt3(origins[s]);
        rC.sXAxis = ToDir3(dirs[s]);
        return true;
    }
    if (t == UsdBrepCurveTokens->brepCurve3dCircleAPI)
    {
        auto centers = brep.EdgeCircleCenters();
        auto axes = brep.EdgeCircleAxes();
        auto refDirs = brep.EdgeCircleRefDirections();
        auto radii = brep.EdgeCircleRadii();
        if (s >= centers.size() || s >= axes.size() || s >= refDirs.size() || s >= radii.size())
        {
            rError = "circle curve index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rC.eType = ECurveType::Circle;
        rC.sLocation = ToPnt3(centers[s]);
        rC.sAxis = ToDir3(ax);
        rC.sXAxis = ToDir3(rd);
        rC.sYAxis = CrossDir(ax, rd);
        rC.dRadius = radii[s];
        return true;
    }
    if (t == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
    {
        auto centers = brep.EdgeEllipseCenters();
        auto axes = brep.EdgeEllipseAxes();
        auto refDirs = brep.EdgeEllipseRefDirections();
        auto xRadii = brep.EdgeEllipseXRadii();
        auto yRadii = brep.EdgeEllipseYRadii();
        if (s >= centers.size() || s >= axes.size() || s >= refDirs.size() || s >= xRadii.size() || s >= yRadii.size())
        {
            rError = "ellipse curve index out of range";
            return false;
        }
        const GfVec3d ax = axes[s];
        const GfVec3d rd = refDirs[s];
        rC.eType = ECurveType::Ellipse;
        rC.sLocation = ToPnt3(centers[s]);
        rC.sAxis = ToDir3(ax);
        rC.sXAxis = ToDir3(rd);
        rC.sYAxis = CrossDir(ax, rd);
        rC.dRadius = xRadii[s];
        rC.dMinorRadius = yRadii[s];
        return true;
    }
    if (t == UsdBrepCurveTokens->brepCurve3dNurbAPI)
    {
        // USD stores every non-analytic 3D curve (bspline/bezier/parabola/hyperbola, periodic
        // unrolled) as a non-periodic NURBS with flat knots. CompressFlatKnots is the exact inverse
        // of the importer's BuildFlatKnots, so we recover the OCCT (knot, multiplicity) pairs.
        auto vcs = brep.EdgeNurbsVertexCounts();
        auto ords = brep.EdgeNurbsOrders();
        if (s >= vcs.size() || s >= ords.size())
        {
            rError = "NURBS edge index out of range";
            return false;
        }
        const uint32_t vc = vcs[s];
        const uint32_t ord = ords[s];

        uint32_t cpStart = 0;
        uint32_t kStart = 0;
        for (uint32_t i = 0; i < s; ++i)
        {
            cpStart += vcs[i];
            kStart += vcs[i] + ords[i];
        }

        auto cps = brep.EdgeNurbsControlVertices();
        auto knots = brep.EdgeNurbsKnots();
        auto wts = brep.EdgeNurbsWeights();

        // Guard poles/knots reads against per-edge metadata inconsistent with the flat arrays.
        if (cpStart + vc > cps.size() || kStart + vc + ord > knots.size())
        {
            rError = "NURBS curve data arrays too short";
            return false;
        }

        rC.eType = ECurveType::BSpline;
        rC.iDegree = static_cast<int>(ord) - 1;
        rC.bPeriodic = false;
        rC.sPoles.clear();
        for (uint32_t i = 0; i < vc; ++i)
        {
            rC.sPoles.push_back(ToPnt3(cps[cpStart + i]));
        }

        bool bRational = false;
        for (uint32_t i = 0; i < vc && i + cpStart < wts.size(); ++i)
        {
            if (std::fabs(wts[cpStart + i] - 1.0) > 1e-12)
            {
                bRational = true;
                break;
            }
        }
        rC.bRational = bRational;
        if (bRational)
        {
            rC.dWeights.clear();
            for (uint32_t i = 0; i < vc && cpStart + i < wts.size(); ++i)
            {
                rC.dWeights.push_back(wts[cpStart + i]);
            }
        }

        std::vector<double> flat;
        for (uint32_t i = 0; i < vc + ord; ++i)
        {
            flat.push_back(knots[kStart + i]);
        }
        UsdBrep::CompressFlatKnots(flat, rC.dKnots, rC.iMults);
        return true;
    }

    rError = "unsupported edge curve type for OCCT export: " + t.GetString();
    return false;
}

// Append one brep (one UsdBrepView) into rModel. Collects its root candidates, including the
// orientation needed by a free Face, so the caller can build the final root reference.
bool AppendBrep(const UsdBrepView& brep, SOcctModel& rModel, std::vector<SSubShapeRef>& rRootRefs, std::string& rError)
{
    const uint32_t faceCount = brep.FaceCount();
    const uint32_t edgeCount = brep.EdgeCount();
    const uint32_t vertexCount = brep.VertexCount();
    const uint32_t edgeuseCount = brep.EdgeuseCount();
    const auto edgeuseOrientTypes = brep.EdgeuseOrientationTypes();
    const auto edgeuseIsSame = [&](uint32_t edgeuse)
    {
        return edgeuse >= edgeuseOrientTypes.size() || edgeuseOrientTypes[edgeuse] == UsdBrepSolidTokens->same;
    };

    const double dTol = brep.HasBrepXSectTol3d() ? brep.BrepXSectTol3d() : 1e-7;

    // ---- Pre-pass: face-of-edgeuse and edgeuses-of-edge from the loop traversal ----
    std::vector<uint32_t> faceOfEdgeuse(edgeuseCount, 0);
    std::vector<std::vector<uint32_t>> edgeusesOfEdge(edgeCount);
    for (uint32_t f = 0; f < faceCount; ++f)
    {
        const uint32_t loops = brep.FaceLoopCount(f);
        for (uint32_t l = 0; l < loops; ++l)
        {
            uint32_t euStart = 0;
            uint32_t euCount = 0;
            if (!UsdBrep::GetFaceLoopEdgeuseStart(brep, f, l, euStart, euCount))
            {
                continue;
            }
            for (uint32_t k = 0; k < euCount; ++k)
            {
                const uint32_t eu = euStart + k;
                if (eu >= edgeuseCount)
                {
                    continue;
                }
                faceOfEdgeuse[eu] = f;
                const uint32_t edge = brep.EdgeuseLocalEdgeIndex(eu);
                if (edge < edgeCount)
                {
                    edgeusesOfEdge[edge].push_back(eu);
                }
            }
        }
    }

    // ---- Geometry tables ----
    std::vector<int> surfaceIdx(faceCount, 0); // 1-based index into rModel.sSurfaces
    for (uint32_t f = 0; f < faceCount; ++f)
    {
        SSurface sS;
        if (!BuildSurface(brep, f, sS, rError))
        {
            return false;
        }
        rModel.sSurfaces.push_back(std::move(sS));
        surfaceIdx[f] = static_cast<int>(rModel.sSurfaces.size());
    }

    auto edgeRanges = brep.EdgeRanges();

    // A zero parametric range marks a degenerate edge: a pole/apex coedge collapsed to a single
    // vertex (e.g. a full cone's apex). OCCT represents these with the Degenerated flag and a lone
    // iso-parameter pcurve -- never a 3D curve -- so skip building (and referencing) a 3D curve here.
    std::vector<bool> bDegenerateEdge(edgeCount, false);
    for (uint32_t e = 0; e < edgeCount; ++e)
    {
        const double dF = (2 * e + 1 < edgeRanges.size()) ? edgeRanges[2 * e] : 0.0;
        const double dL = (2 * e + 1 < edgeRanges.size()) ? edgeRanges[2 * e + 1] : 0.0;
        bDegenerateEdge[e] = std::fabs(dL - dF) < 1e-12;
    }

    std::vector<int> curve3dIdx(edgeCount, 0);
    for (uint32_t e = 0; e < edgeCount; ++e)
    {
        if (bDegenerateEdge[e])
        {
            continue;
        }
        SCurve3d sC;
        if (!BuildCurve3d(brep, e, sC, rError))
        {
            return false;
        }
        rModel.sCurves.push_back(std::move(sC));
        curve3dIdx[e] = static_cast<int>(rModel.sCurves.size());
    }

    // Canonicalize cone/cylinder frames so the surface axis is parallel (not anti-parallel) to its
    // bounding circles. The importer may flip a cone's axis to force a positive semi-angle while
    // leaving the circular edges parameterized in world space; that anti-parallel frame makes the
    // regenerated pcurve u run backwards relative to the seam placement, producing an incoherent
    // two-period UV loop that OCCT cannot mesh. Re-aligning the axis (negate axis, negate cone
    // semi-angle, recompute Y = axis x X) yields the geometrically identical surface with a single
    // forward-running [0, 2*pi] period, which is what the original .brep authored.
    std::vector<bool> bSurfaceParameterizationChanged(faceCount, false);
    std::vector<double> dSurfaceUReflectionOffset(faceCount, 0.0);
    auto faceRanges = brep.FaceRanges();
    for (uint32_t f = 0; f < faceCount; ++f)
    {
        SSurface& sSurf = rModel.sSurfaces[static_cast<size_t>(surfaceIdx[f] - 1)];
        if (sSurf.eType != ESurfaceType::Cone && sSurf.eType != ESurfaceType::Cylinder)
        {
            continue;
        }
        bool bFound = false;
        GfVec3d circAxis(0.0);
        for (uint32_t e = 0; e < edgeCount && !bFound; ++e)
        {
            if (curve3dIdx[e] == 0)
            {
                continue;
            }
            const SCurve3d& sC = rModel.sCurves[static_cast<size_t>(curve3dIdx[e] - 1)];
            if (sC.eType != ECurveType::Circle)
            {
                continue;
            }
            for (uint32_t eu : edgeusesOfEdge[e])
            {
                if (faceOfEdgeuse[eu] == f)
                {
                    circAxis = ToVec(sC.sAxis);
                    bFound = true;
                    break;
                }
            }
        }
        if (bFound && GfDot(ToVec(sSurf.sAxis), circAxis) < 0.0)
        {
            bSurfaceParameterizationChanged[f] = true;
            if (2 * f + 1 < faceRanges.size())
            {
                const double dUMin = faceRanges[2 * f][0];
                const double dUMax = faceRanges[2 * f + 1][0];
                if (std::isfinite(dUMin) && std::isfinite(dUMax))
                {
                    dSurfaceUReflectionOffset[f] = dUMin + dUMax;
                }
            }
            const GfVec3d newAxis = -ToVec(sSurf.sAxis);
            sSurf.sAxis = ToDir3(newAxis);
            sSurf.sYAxis = CrossDir(newAxis, ToVec(sSurf.sXAxis));
            if (sSurf.eType == ESurfaceType::Cone)
            {
                sSurf.dSemiAngle = -sSurf.dSemiAngle;
            }
        }
    }

    // ---- Topology, child-first: vertices, edges, wires, faces, shells, solids ----

    std::vector<int> vertexStorage(vertexCount, 0);
    {
        auto pos = brep.VertexPositions();
        for (uint32_t v = 0; v < vertexCount; ++v)
        {
            SShapeRecord r;
            r.eType = EShapeType::Vertex;
            r.sVertex.dTolerance = dTol;
            r.sVertex.sPoint = ToPnt3(v < pos.size() ? pos[v] : GfVec3d(0.0));
            SetVertexFlags(r);
            rModel.sShapes.push_back(std::move(r));
            vertexStorage[v] = static_cast<int>(rModel.sShapes.size());
        }
    }

    std::vector<int> edgeStorage(edgeCount, 0);
    for (uint32_t e = 0; e < edgeCount; ++e)
    {
        SShapeRecord r;
        r.eType = EShapeType::Edge;
        r.sEdge.dTolerance = dTol;
        r.sEdge.bSameParameter = true;
        r.sEdge.bSameRange = true;
        r.sEdge.bDegenerated = bDegenerateEdge[e];

        const double dFirst = (2 * e + 1 < edgeRanges.size()) ? edgeRanges[2 * e] : 0.0;
        const double dLast = (2 * e + 1 < edgeRanges.size()) ? edgeRanges[2 * e + 1] : 0.0;

        if (!bDegenerateEdge[e])
        {
            SCurveRepr k1;
            k1.iKind = 1;
            k1.iCurve3d = curve3dIdx[e];
            k1.iLocation = 0;
            k1.dFirst = dFirst;
            k1.dLast = dLast;
            r.sEdge.sReprs.push_back(k1);
        }

        // Always run the best-effort pcurve path. Prefer authored USD UV trims where supported,
        // synthesize analytic trims when possible, and leave only unavailable curves absent.
        {
            // Pcurves on adjacent faces. Use a complete set of stored UV trims when available.
            // Otherwise, regenerate trims for analytic curved surfaces by projecting the 3D edge.
            // A curved face that uses this edge twice is the surface seam: emit a single kind-3
            // representation carrying the two iso-pcurves (parameter low / +2*pi). Missing plane
            // and NURBS pcurves remain absent.
            std::map<uint32_t, std::vector<uint32_t>> usesByFace;
            for (uint32_t eu : edgeusesOfEdge[e])
            {
                usesByFace[faceOfEdgeuse[eu]].push_back(eu);
            }
            for (const auto& kv : usesByFace)
            {
                const uint32_t f = kv.first;
                const SSurface& sSurf = rModel.sSurfaces[static_cast<size_t>(surfaceIdx[f] - 1)];

                if (bDegenerateEdge[e])
                {
                    // Degenerate pole/apex coedge (e.g. the full cone's apex). It carries no 3D curve;
                    // OCCT closes the UV rectangle via a lone iso-v pcurve spanning the full u-period at
                    // the pole's v. Recover that v by inverting the collapsed vertex point onto the
                    // surface (u is indeterminate at the pole, so the line starts at u=0 along +u).
                    if (sSurf.eType == ESurfaceType::Plane)
                    {
                        continue;
                    }
                    const GfVec2i vtxDegen = brep.EdgeLocalVertexIndices(e);
                    auto degenPos = brep.VertexPositions();
                    GfVec3d polePt(0.0);
                    if (vtxDegen[0] >= 0 && static_cast<uint32_t>(vtxDegen[0]) < degenPos.size())
                    {
                        polePt = degenPos[static_cast<uint32_t>(vtxDegen[0])];
                    }
                    double dPoleU = 0.0;
                    double dPoleV = 0.0;
                    if (!InvertSurface(sSurf, polePt, dPoleU, dPoleV))
                    {
                        continue;
                    }
                    SCurve2d sPole;
                    sPole.eType = ECurveType::Line;
                    sPole.sLocation = SPnt2{ 0.0, dPoleV };
                    sPole.sXAxis = SDir2{ 1.0, 0.0 };
                    rModel.sCurve2ds.push_back(sPole);
                    SCurveRepr kPole;
                    kPole.iKind = 2;
                    kPole.iPCurve = static_cast<int>(rModel.sCurve2ds.size());
                    kPole.iSurface = surfaceIdx[f];
                    kPole.iLocation = 0;
                    kPole.dFirst = 0.0;
                    kPole.dLast = kTwoPi;
                    r.sEdge.sReprs.push_back(kPole);
                    continue;
                }

                // Source pcurves are already in the authored surface's parameterization. Use them
                // for every surface type when all uses of this edge on the face are present. OCCT
                // cone V is slant distance while BrepArray V is axial distance; frame
                // canonicalization also reflects both UV axes. Apply those affine maps to the
                // NURBS poles so the source curves remain exact in the emitted OCCT surface.
                bool bUsedStoredPCurves = false;
                std::vector<SCurve2d> sStoredPCurves;
                double dStoredPCurveVScale = bSurfaceParameterizationChanged[f] ? -1.0 : 1.0;
                if (sSurf.eType == ESurfaceType::Cone)
                {
                    const double dCos = std::cos(sSurf.dSemiAngle);
                    if (std::fabs(dCos) <= 1e-12)
                    {
                        dStoredPCurveVScale = 0.0;
                    }
                    else
                    {
                        dStoredPCurveVScale /= dCos;
                    }
                }
                for (uint32_t eu : kv.second)
                {
                    SCurve2d sP;
                    if (dStoredPCurveVScale == 0.0 || !BuildPCurve2dFromEdgeuse(brep, eu, sP))
                    {
                        sStoredPCurves.clear();
                        break;
                    }
                    for (SPnt2& rPole : sP.sPoles)
                    {
                        if (bSurfaceParameterizationChanged[f])
                        {
                            rPole.dX = dSurfaceUReflectionOffset[f] - rPole.dX;
                        }
                        rPole.dY *= dStoredPCurveVScale;
                    }
                    if (!edgeuseIsSame(eu))
                    {
                        // USD trims follow the edgeuse; OCCT stores pcurves on the edge and applies
                        // the wire orientation during traversal.
                        ReverseBSplineCurve2d(sP);
                    }
                    sStoredPCurves.push_back(std::move(sP));
                }
                if (sStoredPCurves.size() == kv.second.size() && !sStoredPCurves.empty())
                {
                    // OCCT kind-3 seam records assign iPCurve to the forward edge use and
                    // iPCurve2 to the reversed use. Loop traversal order is not that contract.
                    if (sStoredPCurves.size() == 2 && !edgeuseIsSame(kv.second[0]) && edgeuseIsSame(kv.second[1]))
                    {
                        std::swap(sStoredPCurves[0], sStoredPCurves[1]);
                    }

                    std::vector<int> iPCurves;
                    for (SCurve2d& rStoredPCurve : sStoredPCurves)
                    {
                        rModel.sCurve2ds.push_back(std::move(rStoredPCurve));
                        iPCurves.push_back(static_cast<int>(rModel.sCurve2ds.size()));
                    }

                    SCurveRepr kStored;
                    kStored.iKind = iPCurves.size() == 1 ? 2 : 3;
                    kStored.iPCurve = iPCurves[0];
                    if (iPCurves.size() >= 2)
                    {
                        kStored.iPCurve2 = iPCurves[1];
                        kStored.eContinuity = EContinuity::CN;
                    }
                    kStored.iSurface = surfaceIdx[f];
                    kStored.iLocation = 0;
                    kStored.dFirst = dFirst;
                    kStored.dLast = dLast;
                    r.sEdge.sReprs.push_back(kStored);
                    bUsedStoredPCurves = true;
                }
                if (bUsedStoredPCurves)
                {
                    continue;
                }

                if (sSurf.eType == ESurfaceType::BSpline || sSurf.eType == ESurfaceType::Plane)
                {
                    continue;
                }

                const SCurve3d& sEdgeCurve = rModel.sCurves[static_cast<size_t>(curve3dIdx[e] - 1)];
                SCurve2d sP1;
                if (!ProjectCurveToSurfaceLine(sSurf, sEdgeCurve, dFirst, dLast, sP1))
                {
                    continue;
                }

                if (kv.second.size() >= 2)
                {
                    // Seam edge: normalize the iso (constant) parameter into [0, 2*pi) for the first
                    // pcurve and offset the second by +2*pi so the two coedge uses land on opposite sides
                    // of the parameter rectangle.
                    SCurve2d sP2 = sP1;
                    const bool bUSeam = SurfaceUPeriodic(sSurf) && std::fabs(sP1.sXAxis.dX) < 1e-6;
                    if (bUSeam)
                    {
                        double dU = sP1.sLocation.dX;
                        if (std::fabs(dU) < kSeamSnapTol || std::fabs(dU - kTwoPi) < kSeamSnapTol)
                        {
                            dU = 0.0;
                        }
                        while (dU < 0.0)
                        {
                            dU += kTwoPi;
                        }
                        while (dU >= kTwoPi)
                        {
                            dU -= kTwoPi;
                        }
                        sP1.sLocation.dX = dU;
                        sP2.sLocation.dX = dU + kTwoPi;
                    }
                    else
                    {
                        double dV = sP1.sLocation.dY;
                        if (std::fabs(dV) < kSeamSnapTol || std::fabs(dV - kTwoPi) < kSeamSnapTol)
                        {
                            dV = 0.0;
                        }
                        while (dV < 0.0)
                        {
                            dV += kTwoPi;
                        }
                        while (dV >= kTwoPi)
                        {
                            dV -= kTwoPi;
                        }
                        sP1.sLocation.dY = dV;
                        sP2.sLocation.dY = dV + kTwoPi;
                    }

                    rModel.sCurve2ds.push_back(sP1);
                    const int iP1 = static_cast<int>(rModel.sCurve2ds.size());
                    rModel.sCurve2ds.push_back(sP2);
                    const int iP2 = static_cast<int>(rModel.sCurve2ds.size());

                    // OCCT pairs the FORWARD coedge with PCurve() and the REVERSED coedge with PCurve2().
                    // For a closed surface the loop only winds CCW if the forward coedge sits on the high
                    // (parameter + period) side and the reversed coedge on the low side; otherwise the UV
                    // rectangle folds and the surface meshes as a degenerate sliver. So emit the +period
                    // copy first (PCurve) and the in-range copy second (PCurve2).
                    SCurveRepr k3;
                    k3.iKind = 3;
                    k3.iPCurve = iP2;
                    k3.iPCurve2 = iP1;
                    k3.eContinuity = EContinuity::CN;
                    k3.iSurface = surfaceIdx[f];
                    k3.iLocation = 0;
                    k3.dFirst = dFirst;
                    k3.dLast = dLast;
                    r.sEdge.sReprs.push_back(k3);
                }
                else
                {
                    rModel.sCurve2ds.push_back(sP1);
                    const int iP1 = static_cast<int>(rModel.sCurve2ds.size());

                    SCurveRepr k2;
                    k2.iKind = 2;
                    k2.iPCurve = iP1;
                    k2.iSurface = surfaceIdx[f];
                    k2.iLocation = 0;
                    k2.dFirst = dFirst;
                    k2.dLast = dLast;
                    r.sEdge.sReprs.push_back(k2);
                }
            }
        }

        // Sub-shapes: start vertex Forward, end vertex Reversed.
        const GfVec2i vtx = brep.EdgeLocalVertexIndices(e);
        const int iStart = (vtx[0] >= 0 && static_cast<uint32_t>(vtx[0]) < vertexCount) ? vertexStorage[vtx[0]] : 0;
        const int iEnd = (vtx[1] >= 0 && static_cast<uint32_t>(vtx[1]) < vertexCount) ? vertexStorage[vtx[1]] : 0;
        if (iStart != 0)
        {
            r.sSubShapes.push_back(Ref(iStart, EOrientation::Forward));
        }
        if (iEnd != 0)
        {
            r.sSubShapes.push_back(Ref(iEnd, EOrientation::Reversed));
        }

        SetEdgeFlags(r);
        rModel.sShapes.push_back(std::move(r));
        edgeStorage[e] = static_cast<int>(rModel.sShapes.size());
    }

    // Wires: one per loop, recorded per face so faces can reference them.
    std::vector<std::vector<int>> wireStorageForFace(faceCount);
    for (uint32_t f = 0; f < faceCount; ++f)
    {
        const uint32_t loops = brep.FaceLoopCount(f);
        for (uint32_t l = 0; l < loops; ++l)
        {
            uint32_t euStart = 0;
            uint32_t euCount = 0;
            if (!UsdBrep::GetFaceLoopEdgeuseStart(brep, f, l, euStart, euCount))
            {
                continue;
            }
            SShapeRecord r;
            r.eType = EShapeType::Wire;
            for (uint32_t k = 0; k < euCount; ++k)
            {
                const uint32_t eu = euStart + k;
                if (eu >= edgeuseCount)
                {
                    continue;
                }
                const uint32_t edge = brep.EdgeuseLocalEdgeIndex(eu);
                if (edge >= edgeCount)
                {
                    continue;
                }
                const bool bSame = edgeuseIsSame(eu);
                r.sSubShapes.push_back(Ref(edgeStorage[edge], bSame ? EOrientation::Forward : EOrientation::Reversed));
            }
            SetWireFlags(r);
            rModel.sShapes.push_back(std::move(r));
            wireStorageForFace[f].push_back(static_cast<int>(rModel.sShapes.size()));
        }
    }

    std::vector<int> faceStorage(faceCount, 0);
    for (uint32_t f = 0; f < faceCount; ++f)
    {
        SShapeRecord r;
        r.eType = EShapeType::Face;
        r.sFace.bNaturalRestriction = false;
        r.sFace.dTolerance = dTol;
        r.sFace.iSurface = surfaceIdx[f];
        r.sFace.iLocation = 0;
        for (int iWire : wireStorageForFace[f])
        {
            r.sSubShapes.push_back(Ref(iWire, EOrientation::Forward));
        }
        SetFaceFlags(r);
        rModel.sShapes.push_back(std::move(r));
        faceStorage[f] = static_cast<int>(rModel.sShapes.size());
    }

    // Shells + roots: build closed solids from solid-region shells, then recover unpaired open
    // components from void-region shells.
    auto regionTypes = brep.RegionTypes();
    auto regionShellCounts = brep.RegionShellCounts();
    auto shellFaceuseCounts = brep.ShellFaceuseCounts();
    auto faceuseOrientTypes = brep.FaceuseOrientationTypes();

    // Prefix sums: region -> first shell, shell -> first faceuse.
    std::vector<uint32_t> shellStartOfRegion(brep.RegionCount(), 0);
    {
        uint32_t acc = 0;
        for (uint32_t rIdx = 0; rIdx < brep.RegionCount(); ++rIdx)
        {
            shellStartOfRegion[rIdx] = acc;
            acc += (rIdx < regionShellCounts.size()) ? regionShellCounts[rIdx] : 0;
        }
    }
    std::vector<uint32_t> faceuseStartOfShell(brep.ShellCount(), 0);
    {
        uint32_t acc = 0;
        for (uint32_t sh = 0; sh < brep.ShellCount(); ++sh)
        {
            faceuseStartOfShell[sh] = acc;
            acc += (sh < shellFaceuseCounts.size()) ? shellFaceuseCounts[sh] : 0;
        }
    }

    // Match each solid-region shell to its staging-only mate on the void side. A real open
    // component appears only once (with both faceuses in that shell), whereas a closed boundary
    // has the same face membership in one solid shell and one void shell.
    const uint32_t totalShellCount = brep.ShellCount();
    std::vector<bool> shellInSolidRegion(totalShellCount, false);
    for (uint32_t rIdx = 0; rIdx < brep.RegionCount(); ++rIdx)
    {
        const bool bSolid = (rIdx >= regionTypes.size()) || (regionTypes[rIdx] == UsdBrepSolidTokens->solidRegion);
        if (!bSolid)
        {
            continue;
        }
        const uint32_t shellCount = (rIdx < regionShellCounts.size()) ? regionShellCounts[rIdx] : 0;
        for (uint32_t si = 0; si < shellCount; ++si)
        {
            const uint32_t sh = shellStartOfRegion[rIdx] + si;
            if (sh < totalShellCount)
            {
                shellInSolidRegion[sh] = true;
            }
        }
    }

    std::vector<std::vector<uint32_t>> shellFaceSignatures(totalShellCount);
    for (uint32_t sh = 0; sh < totalShellCount; ++sh)
    {
        const uint32_t fuStart = faceuseStartOfShell[sh];
        const uint32_t fuCount = (sh < shellFaceuseCounts.size()) ? shellFaceuseCounts[sh] : 0;
        std::vector<uint32_t>& rSignature = shellFaceSignatures[sh];
        rSignature.reserve(fuCount);
        for (uint32_t k = 0; k < fuCount; ++k)
        {
            const uint32_t fu = fuStart + k;
            if (fu < brep.FaceuseCount())
            {
                const uint32_t face = brep.FaceuseLocalFaceIndex(fu);
                if (face >= faceCount)
                {
                    rError = "faceuse face index out of range";
                    return false;
                }
                rSignature.push_back(face);
            }
        }
        std::sort(rSignature.begin(), rSignature.end());
    }

    std::vector<int> pairedVoidForSolid(totalShellCount, -1);
    std::vector<int> pairedSolidForVoid(totalShellCount, -1);
    for (uint32_t solidShell = 0; solidShell < totalShellCount; ++solidShell)
    {
        if (!shellInSolidRegion[solidShell] || shellFaceSignatures[solidShell].empty())
        {
            continue;
        }
        for (uint32_t voidShell = 0; voidShell < totalShellCount; ++voidShell)
        {
            if (shellInSolidRegion[voidShell] || pairedSolidForVoid[voidShell] >= 0)
            {
                continue;
            }
            if (shellFaceSignatures[solidShell] == shellFaceSignatures[voidShell])
            {
                pairedVoidForSolid[solidShell] = static_cast<int>(voidShell);
                pairedSolidForVoid[voidShell] = static_cast<int>(solidShell);
                break;
            }
        }
    }

    // Root candidates are keyed by their void-side shell. Shells in the infinite region are packed
    // in source component order, so this also preserves face and faceuse ordering on re-import.
    std::vector<SSubShapeRef> rootForVoidShell(totalShellCount);
    std::vector<bool> hasRootForVoidShell(totalShellCount, false);
    std::vector<SSubShapeRef> rootsWithoutVoidMate;

    for (uint32_t rIdx = 0; rIdx < brep.RegionCount(); ++rIdx)
    {
        const bool bSolid = (rIdx >= regionTypes.size()) || (regionTypes[rIdx] == UsdBrepSolidTokens->solidRegion);
        if (!bSolid)
        {
            continue;
        }

        const uint32_t shellCount = (rIdx < regionShellCounts.size()) ? regionShellCounts[rIdx] : 0;
        if (shellCount == 0)
        {
            continue;
        }
        std::vector<int> shellStorageForRegion;
        for (uint32_t si = 0; si < shellCount; ++si)
        {
            const uint32_t sh = shellStartOfRegion[rIdx] + si;
            const uint32_t fuStart = (sh < faceuseStartOfShell.size()) ? faceuseStartOfShell[sh] : 0;
            const uint32_t fuCount = (sh < shellFaceuseCounts.size()) ? shellFaceuseCounts[sh] : 0;

            SShapeRecord r;
            r.eType = EShapeType::Shell;
            for (uint32_t k = 0; k < fuCount; ++k)
            {
                const uint32_t fu = fuStart + k;
                if (fu >= brep.FaceuseCount())
                {
                    continue;
                }
                const uint32_t face = brep.FaceuseLocalFaceIndex(fu);
                if (face >= faceCount)
                {
                    continue;
                }
                // The importer maps an OCCT face that is Forward-in-shell to a solid-region faceuse
                // orientation of "opposite" (and Reversed-in-shell to "same"); see
                // BrepTopologyBuilder::AddStagedFaceusePairForCurrentTraversal. Invert that here so a
                // USD->brep->USD round trip reproduces the original faceuse orientations.
                const bool bSame = (fu >= faceuseOrientTypes.size()) || (faceuseOrientTypes[fu] == UsdBrepSolidTokens->same);
                r.sSubShapes.push_back(Ref(faceStorage[face], bSame ? EOrientation::Reversed : EOrientation::Forward));
            }
            SetShellFlags(r);
            rModel.sShapes.push_back(std::move(r));
            shellStorageForRegion.push_back(static_cast<int>(rModel.sShapes.size()));
        }

        SShapeRecord solid;
        solid.eType = EShapeType::Solid;
        for (size_t si = 0; si < shellStorageForRegion.size(); ++si)
        {
            // OCCT convention, and the importer's staging convention, is a forward outer shell
            // followed by reversed inner (void/cavity) shells. Face references above encode the
            // same faceuse side for either slot; the shell reference supplies the remaining sense.
            solid.sSubShapes.push_back(Ref(shellStorageForRegion[si], si == 0 ? EOrientation::Forward : EOrientation::Reversed));
        }
        SetSolidFlags(solid);
        rModel.sShapes.push_back(std::move(solid));
        const SSubShapeRef rootRef = Ref(static_cast<int>(rModel.sShapes.size()), EOrientation::Forward);
        const uint32_t outerSolidShell = shellStartOfRegion[rIdx];
        const int pairedVoidShell = (outerSolidShell < pairedVoidForSolid.size()) ? pairedVoidForSolid[outerSolidShell] : -1;
        if (pairedVoidShell >= 0)
        {
            rootForVoidShell[static_cast<size_t>(pairedVoidShell)] = rootRef;
            hasRootForVoidShell[static_cast<size_t>(pairedVoidShell)] = true;
        }
        else
        {
            rootsWithoutVoidMate.push_back(rootRef);
        }
    }

    // Any non-solid shell without a solid-side mate is a real open component, not staging topology.
    for (uint32_t sh = 0; sh < totalShellCount; ++sh)
    {
        if (shellInSolidRegion[sh] || pairedSolidForVoid[sh] >= 0)
        {
            continue;
        }

        std::vector<uint32_t> distinctFaces;
        std::vector<EOrientation> distinctFaceOrientations;
        std::vector<char> seen(faceCount, 0);
        const uint32_t fuStart = faceuseStartOfShell[sh];
        const uint32_t fuCount = (sh < shellFaceuseCounts.size()) ? shellFaceuseCounts[sh] : 0;
        for (uint32_t k = 0; k < fuCount; ++k)
        {
            const uint32_t fu = fuStart + k;
            if (fu >= brep.FaceuseCount())
            {
                continue;
            }
            const uint32_t face = brep.FaceuseLocalFaceIndex(fu);
            if (face >= faceCount || seen[face])
            {
                continue;
            }
            seen[face] = 1;
            distinctFaces.push_back(face);
            const bool bSame = (fu >= faceuseOrientTypes.size()) || (faceuseOrientTypes[fu] == UsdBrepSolidTokens->same);
            distinctFaceOrientations.push_back(bSame ? EOrientation::Forward : EOrientation::Reversed);
        }

        if (distinctFaces.empty())
        {
            continue;
        }

        if (distinctFaces.size() == 1)
        {
            // Lone unattached face: root directly on it. OCCT marks such a face free and unchecked
            // (flag byte 1101000), unlike a face inside a shell (0111000).
            SShapeRecord& sFace = rModel.sShapes[static_cast<size_t>(faceStorage[distinctFaces.front()] - 1)];
            sFace.bFree = true;
            sFace.bChecked = false;
            rootForVoidShell[sh] = Ref(faceStorage[distinctFaces.front()], distinctFaceOrientations.front());
        }
        else
        {
            SShapeRecord shell;
            shell.eType = EShapeType::Shell;
            for (size_t i = 0; i < distinctFaces.size(); ++i)
            {
                shell.sSubShapes.push_back(Ref(faceStorage[distinctFaces[i]], distinctFaceOrientations[i]));
            }
            SetShellFlags(shell);
            shell.bClosed = false; // open shell
            shell.bFree = true;
            rModel.sShapes.push_back(std::move(shell));
            rootForVoidShell[sh] = Ref(static_cast<int>(rModel.sShapes.size()), EOrientation::Forward);
        }
        hasRootForVoidShell[sh] = true;
    }

    const size_t rootCountBefore = rRootRefs.size();
    for (uint32_t sh = 0; sh < totalShellCount; ++sh)
    {
        if (hasRootForVoidShell[sh])
        {
            rRootRefs.push_back(rootForVoidShell[sh]);
        }
    }
    rRootRefs.insert(rRootRefs.end(), rootsWithoutVoidMate.begin(), rootsWithoutVoidMate.end());
    if (rRootRefs.size() == rootCountBefore)
    {
        rError = "brep has faces but no exportable shell";
        return false;
    }

    return true;
}

} // namespace

bool BuildOcctModel(const UsdBrepArrayData& rArrays, SOcctModel& rModel, std::string& rError)
{
    rModel = SOcctModel{};
    rModel.iFormatVersion = 3;

    std::vector<SSubShapeRef> rootRefs;
    bool bAny = false;
    for (const auto& brep : UsdBrep::UsdBrepRange(rArrays))
    {
        if (brep.FaceCount() == 0)
        {
            continue;
        }
        bAny = true;
        if (!AppendBrep(brep, rModel, rootRefs, rError))
        {
            return false;
        }
    }

    if (!bAny || rootRefs.empty())
    {
        rError = "no exportable shape found in brep array";
        return false;
    }

    // rootRefs holds one root candidate per brep: a Solid for closed solids, or a free open Shell /
    // lone free Face for open shells. Preserve the latter's orientation on the root reference.
    if (rootRefs.size() == 1)
    {
        rModel.sRoot = rootRefs.front();
        rModel.bHasRoot = true;
    }
    else
    {
        // Multiple roots -> wrap them under a single Compound root.
        SShapeRecord comp;
        comp.eType = EShapeType::Compound;
        comp.bFree = true;
        for (const SSubShapeRef& rRootRef : rootRefs)
        {
            comp.sSubShapes.push_back(rRootRef);
        }
        rModel.sShapes.push_back(std::move(comp));
        rModel.sRoot = Ref(static_cast<int>(rModel.sShapes.size()), EOrientation::Forward);
        rModel.bHasRoot = true;
    }

    return true;
}

} // namespace occt
