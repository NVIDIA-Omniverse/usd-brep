// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- usd_brep_utils_test.cpp
 * PURPOSE: Focused unit tests for the kernel-free, pure-math surface of USD_BREP_UTILS:
 *
 *    - UsdBrepKnots: flat<->compact knot conversions and periodic-knot expansion. These are
 *      deterministic representation transforms that the round-trip importer/exporter suites
 *      cannot isolate, yet they were hardened in this branch -- exactly the code that benefits
 *      from a direct table-driven test.
 *
 *    - UsdBrepGeometryEval stand-alone evaluators: the analytic surface/curve evaluators, the
 *      de Boor NURBS evaluators, and the analytic inverse-projection routines. Verified via
 *      known closed-form points and evaluate->inverse-project round-trips to identity.
 *
 * These functions take plain GfVec arguments (no UsdBrepArrayData / USD stage required), so the
 * test is a self-contained ConsoleApp that needs only the USD core libraries at runtime. It
 * returns a non-zero exit code if any check fails, so it can gate CI.
 * ******************************************************************************************************************/

#include "UsdBrepGeometryEval.h"
#include "UsdBrepKnots.h"

#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/span.h>
#include <pxr/base/vt/array.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace pxr;
using namespace UsdBrep;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

int g_checks = 0;
int g_failures = 0;

void ReportFail(const std::string& sWhat, const std::string& sDetail)
{
    ++g_failures;
    std::printf("  FAIL: %s -- %s\n", sWhat.c_str(), sDetail.c_str());
}

void CheckTrue(bool bCond, const std::string& sWhat)
{
    ++g_checks;
    if (!bCond)
    {
        ReportFail(sWhat, "expected true");
    }
}

void CheckClose(double a, double b, const std::string& sWhat, double tol = kTol)
{
    ++g_checks;
    if (std::isnan(a) || std::isnan(b) || std::fabs(a - b) > tol)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "got %.12g, expected %.12g (|d|=%.3g, tol=%.3g)", a, b, std::fabs(a - b), tol);
        ReportFail(sWhat, buf);
    }
}

void CheckVec3(const GfVec3d& v, const GfVec3d& e, const std::string& sWhat, double tol = kTol)
{
    ++g_checks;
    double d = (v - e).GetLength();
    if (std::isnan(d) || d > tol)
    {
        char buf[200];
        std::snprintf(
            buf,
            sizeof(buf),
            "got (%.10g, %.10g, %.10g), expected (%.10g, %.10g, %.10g), |d|=%.3g",
            v[0], v[1], v[2], e[0], e[1], e[2], d
        );
        ReportFail(sWhat, buf);
    }
}

void CheckVec2(const GfVec2d& v, const GfVec2d& e, const std::string& sWhat, double tol = kTol)
{
    ++g_checks;
    double d = (v - e).GetLength();
    if (std::isnan(d) || d > tol)
    {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "got (%.10g, %.10g), expected (%.10g, %.10g), |d|=%.3g", v[0], v[1], e[0], e[1], d);
        ReportFail(sWhat, buf);
    }
}

template <typename T>
TfSpan<const T> Span(const std::vector<T>& v)
{
    return TfSpan<const T>(v.data(), v.size());
}

// ===== UsdBrepKnots ==========================================================================

void TestKnots()
{
    std::printf("[UsdBrepKnots]\n");

    // CompressFlatKnots: a clamped degree-2 vector collapses to distinct knots + multiplicities.
    {
        std::vector<double> flat = { 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0 };
        std::vector<double> knots;
        std::vector<int> mults;
        CompressFlatKnots(flat, knots, mults);
        CheckTrue(knots.size() == 3 && mults.size() == 3, "CompressFlatKnots size");
        if (knots.size() == 3)
        {
            CheckClose(knots[0], 0.0, "CompressFlatKnots knot[0]");
            CheckClose(knots[1], 1.0, "CompressFlatKnots knot[1]");
            CheckClose(knots[2], 2.0, "CompressFlatKnots knot[2]");
            CheckTrue(mults[0] == 3 && mults[1] == 1 && mults[2] == 3, "CompressFlatKnots mults");
        }
    }

    // BuildFlatKnots (clamped) is the exact inverse of CompressFlatKnots.
    {
        std::vector<double> knots = { 0.0, 1.0, 2.0 };
        std::vector<int> mults = { 3, 1, 3 };
        VtArray<double> flat = BuildFlatKnots(knots, mults, /*degree*/ 2, /*periodic*/ false);
        std::vector<double> expected = { 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0 };
        CheckTrue(flat.size() == expected.size(), "BuildFlatKnots(clamped) size");
        for (size_t i = 0; i < expected.size() && i < flat.size(); ++i)
        {
            CheckClose(flat[i], expected[i], "BuildFlatKnots(clamped)[" + std::to_string(i) + "]");
        }
    }

    // Round-trip: compress(build(x)) == x for a clamped vector.
    {
        std::vector<double> knots = { 0.0, 0.5, 1.0 };
        std::vector<int> mults = { 4, 2, 4 };
        VtArray<double> flat = BuildFlatKnots(knots, mults, /*degree*/ 3, /*periodic*/ false);
        std::vector<double> flatVec(flat.begin(), flat.end());
        std::vector<double> rk;
        std::vector<int> rm;
        CompressFlatKnots(flatVec, rk, rm);
        CheckTrue(rk == knots, "knot round-trip values");
        CheckTrue(rm == mults, "knot round-trip mults");
    }

    // PeriodicExtraPoles: M1 = (degree+1) - mult[first]; 0 when clamped/non-periodic.
    {
        std::vector<int> clampedMults = { 3, 1, 3 };
        CheckTrue(PeriodicExtraPoles(clampedMults, 2, false) == 0, "PeriodicExtraPoles non-periodic");
        std::vector<int> uniformMults = { 1, 1, 1, 1, 1 };
        CheckTrue(PeriodicExtraPoles(uniformMults, 2, true) == 2, "PeriodicExtraPoles uniform deg2");
        CheckTrue(PeriodicExtraPoles(uniformMults, 3, true) == 3, "PeriodicExtraPoles uniform deg3");
    }

    // BuildFlatKnots (periodic): standard unclamped extension of a uniform knot sequence.
    // knots {0,1,2,3,4}, mult 1, degree 2 -> M1=2 -> {-2,-1,0,1,2,3,4,5,6}.
    {
        std::vector<double> knots = { 0.0, 1.0, 2.0, 3.0, 4.0 };
        std::vector<int> mults = { 1, 1, 1, 1, 1 };
        VtArray<double> flat = BuildFlatKnots(knots, mults, /*degree*/ 2, /*periodic*/ true);
        std::vector<double> expected = { -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
        CheckTrue(flat.size() == expected.size(), "BuildFlatKnots(periodic) size");
        for (size_t i = 0; i < expected.size() && i < flat.size(); ++i)
        {
            CheckClose(flat[i], expected[i], "BuildFlatKnots(periodic)[" + std::to_string(i) + "]");
        }
    }
}

// ===== Analytic surface evaluators + inverse projection ======================================

void TestAnalyticSurfaces()
{
    std::printf("[Analytic surfaces]\n");

    const GfVec3d origin(0.0, 0.0, 0.0);
    const GfVec3d zAxis(0.0, 0.0, 1.0);
    const GfVec3d xRef(1.0, 0.0, 0.0);

    // Plane: S(u,v) = (u, v, 0); refDir->x, axis x refDir -> y.
    CheckVec3(EvalPlaneSurface(origin, zAxis, xRef, 3.0, 4.0), GfVec3d(3.0, 4.0, 0.0), "EvalPlaneSurface");
    CheckVec3(EvalPlaneSurfaceNormal(zAxis), zAxis, "EvalPlaneSurfaceNormal");
    CheckVec2(InverseProjectPlane(origin, zAxis, xRef, GfVec3d(3.0, 4.0, 0.0)), GfVec2d(3.0, 4.0), "InverseProjectPlane");

    // Cylinder: radius 2 about z. Eval->inverse round-trip.
    {
        const double r = 2.0, u = kPi / 4.0, v = 5.0;
        GfVec3d p = EvalCylinderSurface(origin, zAxis, xRef, r, u, v);
        CheckClose(std::hypot(p[0], p[1]), r, "Cylinder point radius");
        CheckClose(p[2], v, "Cylinder point height");
        CheckVec2(InverseProjectCylinder(origin, zAxis, xRef, r, p), GfVec2d(u, v), "Cylinder round-trip");
        CheckVec3(EvalCylinderSurfaceNormal(zAxis, xRef, 0.0), xRef, "Cylinder normal at u=0");
    }

    // Cone: round-trip and unit normal.
    {
        const double r0 = 1.0, sa = kPi / 6.0, u = kPi / 3.0, v = 2.0;
        GfVec3d p = EvalConeSurface(origin, zAxis, xRef, r0, sa, u, v);
        CheckVec2(InverseProjectCone(origin, zAxis, xRef, r0, sa, p), GfVec2d(u, v), "Cone round-trip");
        CheckClose(EvalConeSurfaceNormal(zAxis, xRef, sa, u).GetLength(), 1.0, "Cone normal is unit");
    }

    // Sphere: round-trip away from the poles, plus a known point and unit normal.
    {
        const double R = 3.0, u = kPi / 4.0, v = kPi / 6.0;
        GfVec3d p = EvalSphereSurface(origin, zAxis, xRef, R, u, v);
        CheckClose(p.GetLength(), R, "Sphere point on radius");
        CheckVec2(InverseProjectSphere(origin, zAxis, xRef, R, p), GfVec2d(u, v), "Sphere round-trip");
        // Equator, u=0 -> +refDir * R.
        CheckVec3(EvalSphereSurface(origin, zAxis, xRef, R, 0.0, 0.0), GfVec3d(R, 0.0, 0.0), "Sphere equator u=0");
        CheckVec3(EvalSphereSurfaceNormal(zAxis, xRef, 0.0, 0.0), xRef, "Sphere normal at equator u=0");
    }

    // Torus: round-trip and on-surface check (distance to tube center == minor radius).
    {
        const double majorR = 5.0, minorR = 1.0, u = kPi / 3.0, v = kPi / 4.0;
        GfVec3d p = EvalTorusSurface(origin, zAxis, xRef, majorR, minorR, u, v);
        double ringDist = std::hypot(std::hypot(p[0], p[1]) - majorR, p[2]);
        CheckClose(ringDist, minorR, "Torus point on tube");
        CheckVec2(InverseProjectTorus(origin, zAxis, xRef, majorR, minorR, p), GfVec2d(u, v), "Torus round-trip");
    }
}

// ===== Analytic curve evaluators =============================================================

void TestAnalyticCurves()
{
    std::printf("[Analytic curves]\n");

    const GfVec3d zAxis(0.0, 0.0, 1.0);
    const GfVec3d xRef(1.0, 0.0, 0.0);

    CheckVec3(EvalLineCurve(GfVec3d(1.0, 2.0, 3.0), GfVec3d(0.0, 0.0, 1.0), 2.0), GfVec3d(1.0, 2.0, 5.0), "EvalLineCurve");

    CheckVec3(EvalCircleCurve(GfVec3d(0.0, 0.0, 0.0), zAxis, xRef, 5.0, 0.0), GfVec3d(5.0, 0.0, 0.0), "Circle t=0");
    CheckVec3(EvalCircleCurve(GfVec3d(0.0, 0.0, 0.0), zAxis, xRef, 5.0, kPi / 2.0), GfVec3d(0.0, 5.0, 0.0), "Circle t=pi/2");

    CheckVec3(EvalEllipseCurve(GfVec3d(0.0, 0.0, 0.0), zAxis, xRef, 3.0, 1.0, 0.0), GfVec3d(3.0, 0.0, 0.0), "Ellipse t=0");
    CheckVec3(EvalEllipseCurve(GfVec3d(0.0, 0.0, 0.0), zAxis, xRef, 3.0, 1.0, kPi / 2.0), GfVec3d(0.0, 1.0, 0.0), "Ellipse t=pi/2");
}

// ===== NURBS de Boor evaluation ==============================================================

void TestNurbs()
{
    std::printf("[NURBS de Boor]\n");

    // Degree-1 (order 2) NURBS line: endpoints interpolated, midpoint at the average.
    {
        std::vector<GfVec3d> cps = { GfVec3d(0.0, 0.0, 0.0), GfVec3d(10.0, 0.0, 0.0) };
        std::vector<double> knots = { 0.0, 0.0, 1.0, 1.0 };
        std::vector<double> w = { 1.0, 1.0 };
        CheckVec3(EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 2, 0.0), GfVec3d(0.0, 0.0, 0.0), "NURBS line t=0");
        CheckVec3(EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 2, 1.0), GfVec3d(10.0, 0.0, 0.0), "NURBS line t=1");
        CheckVec3(EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 2, 0.5), GfVec3d(5.0, 0.0, 0.0), "NURBS line t=0.5");
    }

    // Degree-2 (order 3) polynomial Bezier: known midpoint 0.25*P0 + 0.5*P1 + 0.25*P2.
    {
        std::vector<GfVec3d> cps = { GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 2.0, 0.0), GfVec3d(2.0, 0.0, 0.0) };
        std::vector<double> knots = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
        std::vector<double> w = { 1.0, 1.0, 1.0 };
        CheckVec3(EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 3, 0.0), GfVec3d(0.0, 0.0, 0.0), "Bezier t=0");
        CheckVec3(EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 3, 1.0), GfVec3d(2.0, 0.0, 0.0), "Bezier t=1");
        CheckVec3(EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 3, 0.5), GfVec3d(1.0, 1.0, 0.0), "Bezier midpoint");
    }

    // Rational quarter circle (order 3, middle weight = cos(45deg)): exact unit circle.
    {
        const double w1 = std::cos(kPi / 4.0);
        std::vector<GfVec3d> cps = { GfVec3d(1.0, 0.0, 0.0), GfVec3d(1.0, 1.0, 0.0), GfVec3d(0.0, 1.0, 0.0) };
        std::vector<double> knots = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
        std::vector<double> w = { 1.0, w1, 1.0 };
        GfVec3d mid = EvalNurbsCurve3d(Span(cps), Span(knots), Span(w), 3, 0.5);
        CheckClose(mid.GetLength(), 1.0, "Rational quarter-circle midpoint on unit circle");
        CheckVec3(mid, GfVec3d(std::cos(kPi / 4.0), std::sin(kPi / 4.0), 0.0), "Rational quarter-circle midpoint at 45deg");
    }

    // 2D NURBS line (pcurve form): endpoints interpolated.
    {
        std::vector<GfVec2d> cps = { GfVec2d(0.0, 0.0), GfVec2d(4.0, 2.0) };
        std::vector<double> knots = { 0.0, 0.0, 1.0, 1.0 };
        std::vector<double> w = { 1.0, 1.0 };
        CheckVec2(EvalNurbsCurve2d(Span(cps), Span(knots), Span(w), 2, 0.5), GfVec2d(2.0, 1.0), "NURBS 2D line midpoint");
    }

    // Bilinear NURBS surface (unit square in z=0), row-major u then v.
    {
        std::vector<GfVec3d> cps = {
            GfVec3d(0.0, 0.0, 0.0), GfVec3d(0.0, 1.0, 0.0), // u=0 row (v=0,1)
            GfVec3d(1.0, 0.0, 0.0), GfVec3d(1.0, 1.0, 0.0)  // u=1 row (v=0,1)
        };
        std::vector<double> uk = { 0.0, 0.0, 1.0, 1.0 };
        std::vector<double> vk = { 0.0, 0.0, 1.0, 1.0 };
        std::vector<double> w = { 1.0, 1.0, 1.0, 1.0 };
        auto eval = [&](double u, double v) { return EvalNurbsSurface(Span(cps), 2, 2, Span(uk), Span(vk), Span(w), 2, 2, u, v); };
        CheckVec3(eval(0.0, 0.0), GfVec3d(0.0, 0.0, 0.0), "NURBS surface corner (0,0)");
        CheckVec3(eval(1.0, 1.0), GfVec3d(1.0, 1.0, 0.0), "NURBS surface corner (1,1)");
        CheckVec3(eval(0.5, 0.5), GfVec3d(0.5, 0.5, 0.0), "NURBS surface center");
    }
}
} // anonymous namespace

int main()
{
    std::printf("usd_brep_utils_test: unit tests for UsdBrepKnots + UsdBrepGeometryEval\n");

    TestKnots();
    TestAnalyticSurfaces();
    TestAnalyticCurves();
    TestNurbs();

    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    if (g_failures != 0)
    {
        std::printf("RESULT: FAILED\n");
        return 1;
    }
    std::printf("RESULT: PASSED\n");
    return 0;
}
