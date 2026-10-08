// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_GEOMETRY_CONVERT_H
#define OCCT_BREP_IMPORT_OCCT_GEOMETRY_CONVERT_H

#include "BrepStagingState.h" // StagedSurfaceData / StagedCurveData (BREP_STAGING/inc)
#include "OcctFileRecords.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/range2d.h>

namespace occt
{

// Resolve a 1-based location index from rModel.sLocations into a world placement.
// Index 0 means identity. Elementary locations yield their stored 3x4 affine; composite locations
// yield the product of their referenced datum powers (matching TopLoc_Location composition).
pxr::GfMatrix4d ResolveLocation(const SOcctModel& rModel, int iLocationIndex);

// Convert a parsed 3D curve into staged curve data, applying rTransform (the edge's world placement).
// Line, Circle, Ellipse pass through as analytic; Bezier/BSpline become a NURBS curve; Parabola and
// Hyperbola are lowered to an exact (rational) quadratic NURBS over their edge interval. Trimmed/
// Offset are resolved to their basis curve.
// pInterval (optional): the edge's parameter range on the curve. Required to bound Parabola/Hyperbola
// (which are otherwise unbounded); ignored by the bounded/analytic types. Returns false for
// unsupported curve types or when a bounded range is needed but unavailable (rOut left as NotSet).
bool ConvertCurve3d(const SCurve3d& rCurve, const pxr::GfMatrix4d& rTransform, StagedCurveData& rOut, const pxr::GfRange1d* pInterval = nullptr);

// Convert a parsed 2D pcurve over rInterval into an exact staged NURBS trim curve. The affine
// coordinate transform maps the source surface's UV frame to the authored face frame. Lines,
// conics, Bezier curves, and B-splines are supported; trimmed free-form curves are split to the
// requested interval. A non-zero Offset wrapper is rejected because a general offset is not
// exactly representable as a NURBS curve.
bool ConvertCurve2dToNurb(
    const SCurve2d& rCurve,
    const pxr::GfRange1d& rInterval,
    const pxr::GfVec2d& rScale,
    const pxr::GfVec2d& rTranslation,
    StagedCurveData2d& rOut
);

// Convert a parsed surface into staged surface data, applying rTransform (the face's world placement).
// Supports Plane, Cylinder, Cone, Sphere, Torus, and BSpline directly; surfaces of linear extrusion
// and revolution are lowered to an equivalent NURBS (BSpline) surface. RectangularTrimmed/Offset are
// resolved to their basis surface. Returns false for unsupported surface types (rOut left as NotSet).
//
// pUVHint (optional): the face's pcurve-derived UV box (param space). For swept surfaces it bounds the
// V extent (extrusion distance / revolution angle) and, for a line basis, the U extent.
// pSweptDomainOut (optional): when the surface is a swept (extrusion/revolution) lowering, it is set to
// the NURBS-natural parameter box the caller should use as the face range (the lowered surface uses a
// different internal parameterization than OCCT's, so the pcurve box does not apply directly).
bool ConvertSurface(
    const SSurface& rSurface,
    const pxr::GfMatrix4d& rTransform,
    StagedSurfaceData& rOut,
    const pxr::GfRange2d* pUVHint = nullptr,
    pxr::GfRange2d* pSweptDomainOut = nullptr
);

// Evaluate a 3D curve at parameter t in world space (rTransform applied). Line/Circle/Ellipse are
// exact; Bezier/BSpline are evaluated via de Boor (rational + periodic aware). Trimmed/Offset are
// resolved to their basis. Used to sample edge geometry for domain bounding.
pxr::GfVec3d EvalCurve3dPoint(const SCurve3d& rCurve, const pxr::GfMatrix4d& rTransform, double t);

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_GEOMETRY_CONVERT_H
