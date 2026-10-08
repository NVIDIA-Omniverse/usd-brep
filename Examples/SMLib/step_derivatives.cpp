// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "step_derivatives.h"

#include <SmAxis2Placement.h>
#include <SmBSplineCurve.h>
#include <SmCone.h>
#include <SmContext.h>
#include <SmCylinder.h>
#include <SmSphere.h>
#include <SmSurfOfRevolution.h>
#include <SmTorus.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr ULONG kOrder = 3;
constexpr ULONG kStride = kOrder + 1;
constexpr double kRadiansPerDegree = SM_PI / 180.0;
using Derivatives = std::array<SmVector3d, kStride * kStride>;

void Check(SmStatus status, const char* operation)
{
    if (status != SM_SUCCESS)
    {
        throw std::runtime_error(std::string(operation) + " returned " + std::to_string(status));
    }
}

void RequireClose(const SmVector3d& actual, const SmVector3d& expected, const std::string& label)
{
    // Diagnostic tolerances for these small, smooth fixtures, not modeling
    // tolerances or a universal finite-difference step-selection policy.
    const double error = (actual - expected).Length();
    const double tolerance = 1.0e-9 + 1.0e-5 * expected.Length();
    if (!std::isfinite(error) || !std::isfinite(tolerance) || error > tolerance)
    {
        throw std::runtime_error(label + ": vector error " + std::to_string(error));
    }
}

Derivatives Evaluate(const SmSurface& surface, const SmPoint2d& stepUV)
{
    Derivatives values{};
    // These evaluators require equal U/V orders, at most 3. Allocate the
    // full rectangle, but only use entries i + j <= order. In particular,
    // requesting (1,1) does NOT populate Duv; request at least (2,2).
    Check(surface.EvaluateSTEP(stepUV, kOrder, kOrder, TRUE, TRUE, TRUE, values.data()), "EvaluateSTEP");
    return values;
}

Derivatives CheckSurface(
    const SmSurface& surface,
    const char* name,
    const SmPoint2d& stepUV,
    double hU,
    double hV,
    std::ostream& output)
{
    const auto values = Evaluate(surface, stepUV);
    const auto uMinus = Evaluate(surface, SmPoint2d(stepUV.x - hU, stepUV.y));
    const auto uPlus = Evaluate(surface, SmPoint2d(stepUV.x + hU, stepUV.y));
    const auto vMinus = Evaluate(surface, SmPoint2d(stepUV.x, stepUV.y - hV));
    const auto vPlus = Evaluate(surface, SmPoint2d(stepUV.x, stepUV.y + hV));

    // First derivatives are differences of positions. Higher derivatives are
    // differences of the preceding derivative, avoiding third differences of
    // positions and their cancellation error. Perturb the SAME parameters
    // passed to EvaluateSTEP: degrees for angular axes, not radians.
    double maxError[kOrder + 1] = {};
    for (ULONG i = 0; i <= kOrder; ++i)
    {
        for (ULONG j = 0; i + j <= kOrder; ++j)
        {
            if (i + j == 0)
            {
                continue;
            }
            const ULONG previous = i > 0 ? (i - 1) * kStride + j : j - 1;
            const SmVector3d finiteDifference = i > 0 ? (uPlus[previous] - uMinus[previous]) / (2.0 * hU) :
                                                       (vPlus[previous] - vMinus[previous]) / (2.0 * hV);
            const SmVector3d& derivative = values[i * kStride + j];
            RequireClose(derivative, finiteDifference, std::string(name) + " D(" + std::to_string(i) + "," + std::to_string(j) + ")");
            maxError[i + j] = std::max(maxError[i + j], (derivative - finiteDifference).Length());
        }
    }

    output << name << " at STEP (" << stepUV.x << ", " << stepUV.y << ")\n"
           << "  |Du| = " << values[kStride].Length() << ", |Dv| = " << values[1].Length() << '\n'
           << "  max finite-difference error by order: " << maxError[1] << ", " << maxError[2] << ", " << maxError[3] << '\n';
    return values;
}
} // namespace

void RunSTEPDerivativesExample(std::ostream& output)
{
    // Context outlives every geometry owner. Evaluation is read-only; this
    // example neither loads a model nor changes application-owned geometry.
    SmContext context;
    const SmAxis2Placement placement(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));

    SmCylinder* rawCylinder = nullptr;
    const SmStatus cylinderStatus = SmCylinder::CreateCanonical(context, placement, 10.0, rawCylinder);
    std::unique_ptr<SmCylinder> cylinder(rawCylinder);
    Check(cylinderStatus, "SmCylinder::CreateCanonical");

    const SmPoint2d cylinderUV(30.0, 2.0); // U in degrees; V in length units.
    const auto d = CheckSurface(*cylinder, "Cylinder (U degrees, V length)", cylinderUV, 1.0e-3, 1.0e-4, output);
    const SmVector3d& du = d[1 * kStride + 0]; // Directly usable: NO caller-side pi/180 factor.
    const double angle = cylinderUV.x * kRadiansPerDegree; // Only for the analytic oracle below.
    RequireClose(d[0], SmVector3d(10.0 * std::cos(angle), 10.0 * std::sin(angle), 2.0), "Cylinder position");
    RequireClose(du, SmVector3d(-10.0 * std::sin(angle), 10.0 * std::cos(angle), 0.0) * kRadiansPerDegree, "Cylinder Du per degree");
    RequireClose(d[1], SmVector3d(0.0, 0.0, 1.0), "Cylinder Dv per length unit");
    output << "  expected |Du| = 10*pi/180 = " << 10.0 * kRadiansPerDegree << '\n'
           << "  applying the old pi/180 workaround AGAIN would give " << du.Length() * kRadiansPerDegree << " (wrong)\n";

    // Same 3D point, different parameterization. Convert the UV before using
    // ordinary EvaluatePoint; this conversion does not convert derivatives.
    SmPoint2d nurbsUV;
    SmPoint3d nurbsPoint;
    Check(cylinder->ConvertUVFromSTEPToNURBS(cylinderUV, nurbsUV), "ConvertUVFromSTEPToNURBS");
    Check(cylinder->EvaluatePoint(nurbsUV, nurbsPoint), "EvaluatePoint");
    RequireClose(nurbsPoint, d[0], "STEP/NURBS positions");
    output << "  converted NURBS UV = (" << nurbsUV.x << ", " << nurbsUV.y << "): same position\n";

    SmCone* rawCone = nullptr;
    const SmStatus coneStatus = SmCone::CreateCanonical(context, placement, 1.0, 2.0, 3.0, rawCone);
    std::unique_ptr<SmCone> cone(rawCone);
    Check(coneStatus, "SmCone::CreateCanonical");
    CheckSurface(*cone, "Cone (U degrees, V length)", SmPoint2d(110.0, 1.5), 1.0e-3, 1.0e-4, output);

    SmSphere* rawSphere = nullptr;
    const SmStatus sphereStatus = SmSphere::CreateCanonical(context, placement, 2.0, rawSphere);
    std::unique_ptr<SmSphere> sphere(rawSphere);
    Check(sphereStatus, "SmSphere::CreateCanonical");
    CheckSurface(*sphere, "Sphere (U and V degrees)", SmPoint2d(110.0, 30.0), 1.0e-3, 1.0e-3, output);

    SmTorus* rawTorus = nullptr;
    const SmStatus torusStatus = SmTorus::CreateCanonical(context, placement, 5.0, 1.0, rawTorus);
    std::unique_ptr<SmTorus> torus(rawTorus);
    Check(torusStatus, "SmTorus::CreateCanonical");
    CheckSurface(*torus, "Torus (U and V degrees)", SmPoint2d(250.0, 40.0), 1.0e-3, 1.0e-3, output);

    // A general revolution does NOT imply that V is an angle. Here it is the
    // parameter of a cubic B-spline generator, with interval [0,1].
    SmTArray<SmPoint3d> points;
    points.Add(SmPoint3d(2.0, 0.0, 0.0));
    points.Add(SmPoint3d(2.5, 0.4, 0.5));
    points.Add(SmPoint3d(3.2, -0.2, 1.7));
    points.Add(SmPoint3d(2.7, 0.3, 3.0));
    SmTArray<ULONG> multiplicities;
    multiplicities.Add(4);
    multiplicities.Add(4);
    SmTArray<double> knots;
    knots.Add(0.0);
    knots.Add(1.0);
    SmBSplineCurve* rawGenerator = nullptr;
    const SmStatus generatorStatus = SmBSplineCurve::CreateCanonical(
        context, 3, 3, points, SM_CF_UNSPECIFIED, multiplicities, knots, SM_KT_UNSPECIFIED, nullptr, nullptr, rawGenerator);
    std::unique_ptr<SmBSplineCurve> generator(rawGenerator);
    Check(generatorStatus, "SmBSplineCurve::CreateCanonical");

    SmSurfOfRevolution* rawRevolution = nullptr;
    const SmStatus revolutionStatus = SmSurfOfRevolution::CreateCanonical(
        context, generator.get(), SmPoint3d(0, 0, 0), SmVector3d(0, 0, 1), rawRevolution);
    std::unique_ptr<SmSurfOfRevolution> revolution(rawRevolution);
    // This factory adopts the generator once it creates the surface, even if
    // subsequent NURBS construction fails. Establish ownership before Check.
    if (revolution)
    {
        generator.release();
    }
    Check(revolutionStatus, "SmSurfOfRevolution::CreateCanonical");
    CheckSurface(*revolution, "Revolution (U degrees, V generator parameter)", SmPoint2d(110.0, 0.4), 1.0e-3, 1.0e-4, output);

    output << "STEP derivative checks passed (orders 1-3, including mixed derivatives).\n";
}

#ifdef SM_STEP_DERIVATIVES_EXAMPLE_STANDALONE
int main()
{
    try
    {
        RunSTEPDerivativesExample(std::cout);
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "STEP derivative example: " << error.what() << '\n';
        return 1;
    }
}
#endif
