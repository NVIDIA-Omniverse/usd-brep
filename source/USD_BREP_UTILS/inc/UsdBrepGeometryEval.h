// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepGeometryEval.h
 * PURPOSE: Point evaluation for analytic and NURBS geometry via UsdBrepView.
 *
 * CONTAINS:
 *  namespace UsdBrep {
 *
 *      // Surface domain / range queries
 *      UsdBrepSurfaceDomain         - UV domain with periodicity metadata
 *      GetSurfaceDomain()       - query the UV domain for a face
 *
 *      // Range-aware evaluation
 *      EvaluateSurfaceNormalized()       - evaluate at [0,1]x[0,1] mapped through domain
 *      EvaluateSurfaceNormalNormalized() - normal at [0,1]x[0,1]
 *      EvaluateSurfaceClamped()          - evaluate with clamping to valid domain
 *      EvaluateSurfaceBoundary()         - evaluate iso-curves at domain edges
 *
 *      // Surface evaluation (analytic + NURBS)
 *      EvaluateSurface()
 *      EvaluateSurfaceNormal()
 *
 *      // 3D curve evaluation (analytic + NURBS)
 *      EvaluateCurve3d()
 *
 *      // 2D trim curve evaluation (NURBS)
 *      EvaluateCurve2d()
 *
 *      // Stand-alone analytic evaluators (no UsdBrepView required)
 *      EvalPlaneSurface()
 *      EvalCylinderSurface()
 *      EvalConeSurface()
 *      EvalSphereSurface()
 *      EvalTorusSurface()
 *      EvalLineCurve()
 *      EvalCircleCurve()
 *      EvalEllipseCurve()
 *  }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_GEOMETRY_EVAL_H_
#define _USD_BREP_GEOMETRY_EVAL_H_

#include "UsdBrepIterator.h"
#include "UsdBrepUtilsConfig.h"

#include <vector>

namespace UsdBrep
{
PXR_NAMESPACE_USING_DIRECTIVE

// ---- Shared tolerances ----

// Two 3D points closer than this are considered the same position.
static constexpr double kPositionCoincidence = 1e-8;

// Fraction of v-span to identify a boundary vertex near a degenerate pole.
static constexpr double kPoleFraction = 0.02;

// ---- Surface domain / range ----------------------------------------------------------------

// Describes the valid parametric domain for a face's surface, along with
// metadata about which parameter directions are periodic (angular).
struct USD_BREP_EXPORT UsdBrepSurfaceDomain
{
    GfVec2d uvMin = GfVec2d(0.0, 0.0);
    GfVec2d uvMax = GfVec2d(1.0, 1.0);

    bool uPeriodic = false; // true when u wraps (cylinder, cone, sphere, torus)
    bool vPeriodic = false; // true when v wraps (torus)

    // True when the parameter represents an angle and should use angular
    // deflection for tessellation density.  Every periodic direction is
    // angular, but not vice-versa: sphere v (latitude) is angular yet
    // bounded (not periodic).
    bool uAngular = false;
    bool vAngular = false;

    double uPeriod = 0.0; // natural period when uPeriodic (typically 2*pi)
    double vPeriod = 0.0; // natural period when vPeriodic (typically 2*pi for torus)

    TfToken surfaceType;

    double USpan() const
    {
        return uvMax[0] - uvMin[0];
    }
    double VSpan() const
    {
        return uvMax[1] - uvMin[1];
    }

    // Map normalized coordinate s in [0,1] to parameter value in [min,max]
    double MapU(double s) const
    {
        return uvMin[0] + s * USpan();
    }
    double MapV(double s) const
    {
        return uvMin[1] + s * VSpan();
    }
    GfVec2d MapUV(double s, double t) const
    {
        return GfVec2d(MapU(s), MapV(t));
    }

    // Inverse: map parameter value to [0,1]
    double UnmapU(double u) const;
    double UnmapV(double v) const;

    // Clamp (u,v) to the valid domain
    double ClampU(double u) const;
    double ClampV(double v) const;
    GfVec2d ClampUV(double u, double v) const
    {
        return GfVec2d(ClampU(u), ClampV(v));
    }

    // Wrap periodic parameters into the valid domain
    double WrapU(double u) const;
    double WrapV(double v) const;
};

// Query the UV domain for a face.
// Populates periodicity flags based on the surface type.
USD_BREP_EXPORT UsdBrepSurfaceDomain GetSurfaceDomain(const UsdBrepView& brep, uint32_t iLocalFace);

// Query periodicity for a surface type token without needing a UsdBrepView.
USD_BREP_EXPORT void GetSurfacePeriodicity(const TfToken& surfaceType, bool& oUPeriodic, bool& oVPeriodic, double& oUPeriod, double& oVPeriod);

// ---- Range-aware surface evaluation --------------------------------------------------------

// Evaluate at normalized coordinates (s,t) in [0,1]x[0,1], mapped through the face's UV domain.
USD_BREP_EXPORT GfVec3d EvaluateSurfaceNormalized(const UsdBrepView& brep, uint32_t iLocalFace, double s, double t);

// Surface normal at normalized coordinates.
USD_BREP_EXPORT GfVec3d EvaluateSurfaceNormalNormalized(const UsdBrepView& brep, uint32_t iLocalFace, double s, double t);

// Evaluate with (u,v) clamped to the face's valid UV domain.
USD_BREP_EXPORT GfVec3d EvaluateSurfaceClamped(const UsdBrepView& brep, uint32_t iLocalFace, double u, double v);

// Boundary edge enumeration for the UV domain
enum class UsdBrepDomainEdge
{
    UMin, // v varies, u = uMin  (left edge)
    UMax, // v varies, u = uMax  (right edge)
    VMin, // u varies, v = vMin  (bottom edge)
    VMax // u varies, v = vMax  (top edge)
};

// Evaluate the 3D iso-curve along a domain boundary edge at parameter t in [0,1].
// t=0 maps to the start of the edge, t=1 maps to the end.
USD_BREP_EXPORT GfVec3d EvaluateSurfaceBoundary(const UsdBrepView& brep, uint32_t iLocalFace, UsdBrepDomainEdge edge, double t);

// Sample a boundary edge at N+1 evenly-spaced points (including endpoints).
USD_BREP_EXPORT std::vector<GfVec3d> SampleSurfaceBoundary(const UsdBrepView& brep, uint32_t iLocalFace, UsdBrepDomainEdge edge, uint32_t numSegments);

// Compute the approximate arc length of a domain boundary edge by sampling.
USD_BREP_EXPORT double ApproxBoundaryLength(const UsdBrepView& brep, uint32_t iLocalFace, UsdBrepDomainEdge edge, uint32_t numSegments = 32);

// ---- Stand-alone analytic surface evaluators ------------------------------------------------

USD_BREP_EXPORT GfVec3d EvalPlaneSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double u, double v);

USD_BREP_EXPORT GfVec3d EvalCylinderSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double radius, double u, double v);

USD_BREP_EXPORT GfVec3d
EvalConeSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double radius, double semiAngle, double u, double v);

USD_BREP_EXPORT GfVec3d EvalSphereSurface(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double radius, double u, double v);

USD_BREP_EXPORT GfVec3d
EvalTorusSurface(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double majorRadius, double minorRadius, double u, double v);

// ---- Stand-alone analytic surface normal evaluators -----------------------------------------

USD_BREP_EXPORT GfVec3d EvalPlaneSurfaceNormal(const GfVec3d& axis);

USD_BREP_EXPORT GfVec3d EvalCylinderSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double u);

USD_BREP_EXPORT GfVec3d EvalConeSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double semiAngle, double u);

USD_BREP_EXPORT GfVec3d EvalSphereSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double u, double v);

USD_BREP_EXPORT GfVec3d EvalTorusSurfaceNormal(const GfVec3d& axis, const GfVec3d& refDir, double majorRadius, double minorRadius, double u, double v);

// ---- Inverse projection: 3D point -> UV parameters on analytic surfaces ---------------------

// Given a point known to lie on the plane, return (u,v).
USD_BREP_EXPORT GfVec2d InverseProjectPlane(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, const GfVec3d& point);

USD_BREP_EXPORT GfVec2d InverseProjectCylinder(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double radius, const GfVec3d& point);

USD_BREP_EXPORT GfVec2d
InverseProjectCone(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double radius, double semiAngle, const GfVec3d& point);

USD_BREP_EXPORT GfVec2d InverseProjectSphere(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double radius, const GfVec3d& point);

USD_BREP_EXPORT GfVec2d
InverseProjectTorus(const GfVec3d& origin, const GfVec3d& axis, const GfVec3d& refDir, double majorRadius, double minorRadius, const GfVec3d& point);

// High-level: inverse-project a point onto the surface of a given face.
USD_BREP_EXPORT GfVec2d InverseProjectSurface(const UsdBrepView& brep, uint32_t iLocalFace, const GfVec3d& point);

// Inverse-project with a UV hint for the initial Newton guess.
// For NURBS surfaces, skips the coarse grid search and starts Newton
// iteration from uvHint, which should be near the expected solution
// (e.g. the UV of a neighbouring boundary vertex).
USD_BREP_EXPORT GfVec2d InverseProjectSurfaceWithHint(const UsdBrepView& brep, uint32_t iLocalFace, const GfVec3d& point, const GfVec2d& uvHint);

// ---- Compute trimmed UV domain from edge geometry -------------------------------------------

// Compute the actual UV bounding box for a face by sampling its boundary edges
// and inverse-projecting them onto the surface. Returns a tightened domain.
// Falls back to the stored face range if there are no edges or the surface
// type doesn't support inverse projection.
USD_BREP_EXPORT UsdBrepSurfaceDomain ComputeTrimmedDomain(const UsdBrepView& brep, uint32_t iLocalFace, uint32_t samplesPerEdge = 16);

// ---- Stand-alone analytic curve evaluators --------------------------------------------------

USD_BREP_EXPORT GfVec3d EvalLineCurve(const GfVec3d& origin, const GfVec3d& direction, double t);

USD_BREP_EXPORT GfVec3d EvalCircleCurve(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double radius, double t);

USD_BREP_EXPORT GfVec3d EvalEllipseCurve(const GfVec3d& center, const GfVec3d& axis, const GfVec3d& refDir, double xRadius, double yRadius, double t);

// ---- NURBS evaluation (de Boor) -------------------------------------------------------------

// Evaluate a 3D NURBS curve at parameter t.
// controlVertices, knots, weights are contiguous spans; order = degree+1.
USD_BREP_EXPORT GfVec3d
EvalNurbsCurve3d(TfSpan<const GfVec3d> controlVertices, TfSpan<const double> knots, TfSpan<const double> weights, uint32_t order, double t);

// Evaluate a 2D NURBS curve at parameter t.
USD_BREP_EXPORT GfVec2d
EvalNurbsCurve2d(TfSpan<const GfVec2d> controlVertices, TfSpan<const double> knots, TfSpan<const double> weights, uint32_t order, double t);

// Evaluate a NURBS surface at parameters (u, v).
USD_BREP_EXPORT GfVec3d EvalNurbsSurface(
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
);

// ---- High-level evaluators using UsdBrepView + local indices --------------------------------

// Evaluate the surface of a face at parameters (u,v).
// Dispatches to the appropriate analytic or NURBS evaluator based on face surface type.
USD_BREP_EXPORT GfVec3d EvaluateSurface(const UsdBrepView& brep, uint32_t iLocalFace, double u, double v);

// Evaluate the outward-pointing surface normal at (u,v) for the given face.
USD_BREP_EXPORT GfVec3d EvaluateSurfaceNormal(const UsdBrepView& brep, uint32_t iLocalFace, double u, double v);

// Evaluate the 3D curve of an edge at parameter t.
USD_BREP_EXPORT GfVec3d EvaluateCurve3d(const UsdBrepView& brep, uint32_t iLocalEdge, double t);

// Evaluate the 2D pcurve (UV trim curve) of an edgeuse at parameter t.
// Returns the UV coordinates on the associated face's surface.
// If no pcurve is stored for this edgeuse (vertexCount == 0), returns (NaN, NaN).
USD_BREP_EXPORT GfVec2d EvaluateCurve2d(const UsdBrepView& brep, uint32_t iLocalEdgeuse, double t);

// Return true if a 2D pcurve (UV trim curve) is available for the given edgeuse.
USD_BREP_EXPORT bool HasPCurve(const UsdBrepView& brep, uint32_t iLocalEdgeuse);

// Get the valid parameter range [tStart, tEnd] for the pcurve of the given edgeuse.
// Returns (0,0) if no pcurve exists.
USD_BREP_EXPORT GfVec2d GetPCurveRange(const UsdBrepView& brep, uint32_t iLocalEdgeuse);

// Returns true when the surface's geometric normal already points outward
// (away from the solid interior) for the given face.  Returns false when
// the normal must be negated to obtain the outward direction.
USD_BREP_EXPORT bool IsFaceNormalOutward(const UsdBrepView& brep, uint32_t iLocalFace);

// Returns true when the faceuse orientation is "opposite"
// (face normal = negation of the surface normal).
// This determines the UV winding convention for loop traversal.
USD_BREP_EXPORT bool IsFaceuseOpposite(const UsdBrepView& brep, uint32_t iLocalFace);

// ---- Shared geometry helpers ----------------------------------------------------------------

// Signed area of a 2D polygon (positive = CCW, negative = CW).
USD_BREP_EXPORT double SignedArea2D(const std::vector<GfVec2d>& polygon);

// Detect the effective period for each parameter direction of a face's
// surface.  Analytic types use their known periodicity; NURBS surfaces
// are probed at runtime by comparing opposite domain boundaries.
// Returns 0 for non-periodic directions.
USD_BREP_EXPORT void DetectSurfacePeriodicity(const UsdBrepView& brep, uint32_t iLocalFace, double& oUPeriod, double& oVPeriod);

// Detect degenerate poles at the v-extremes of a face's surface.
// A pole exists where all u-values map to the same 3D point (e.g.
// sphere north/south poles, cone apex).
USD_BREP_EXPORT void DetectSurfacePoles(const UsdBrepView& brep, uint32_t iLocalFace, bool& oHasPoleVMin, bool& oHasPoleVMax);

// Get the local edgeuse start index and count for a specific loop
// of a face.  Returns false if the face or loop index is out of range.
USD_BREP_EXPORT bool GetFaceLoopEdgeuseStart(
    const UsdBrepView& brep,
    uint32_t iLocalFace,
    uint32_t localLoopIndex,
    uint32_t& oEuStart,
    uint32_t& oEuCount
);

} // end namespace UsdBrep

#endif // _USD_BREP_GEOMETRY_EVAL_H_
