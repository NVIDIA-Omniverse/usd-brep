// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepKnots.h
 * PURPOSE: Kernel-agnostic NURBS knot-vector conversions between the USD BrepArray "flat" form
 *          (every knot repeated by its multiplicity) and the compact (knot, multiplicity) form
 *          used by most kernels' file formats. These are pure representation transforms with no
 *          dependency on any modeling kernel, shared by the importers/exporters.
 *
 * CONTAINS:
 *  namespace UsdBrep {
 *      CompressFlatKnots()  - flat knots -> unique knots + multiplicities
 *      BuildFlatKnots()     - (knots, mults, degree, periodic) -> flat knot vector USD expects
 *      PeriodicExtraPoles() - count of wrapped poles a periodic direction adds when unrolled
 *  }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_KNOTS_H_
#define _USD_BREP_KNOTS_H_

#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes
#include "UsdBrepUtilsConfig.h"

#include <pxr/base/vt/array.h>

#include <vector>

namespace UsdBrep
{

// Convert a flat (fully expanded) NURBS knot vector -- the form USD BrepArray stores, where each
// knot is repeated according to its multiplicity -- into the compact form of unique knot values
// plus per-knot integer multiplicities (the form most kernels and file formats use).
//
// Knots within dTolerance of one another are treated as identical. Output vectors are cleared
// first and always have matching sizes. This is the exact inverse of BuildFlatKnots for a
// non-periodic (clamped) knot vector.
USD_BREP_EXPORT void CompressFlatKnots(
    const std::vector<double>& rFlatKnots,
    std::vector<double>& rOutKnots,
    std::vector<int>& rOutMults,
    double dTolerance = 1e-9
);

// Number of extra (wrapped) poles a periodic direction contributes when unrolled to the
// equivalent unclamped (non-periodic) B-spline, defined as M1 = (degree + 1) - mult[first].
// Returns 0 for a non-periodic direction.
USD_BREP_EXPORT int PeriodicExtraPoles(const std::vector<int>& rMults, int iDegree, bool bPeriodic);

// Build the flat knot vector USD expects (length == vertexCount + order). For a clamped
// (non-periodic) B-spline this just expands the (knot, multiplicity) pairs. For a periodic
// B-spline this reproduces the standard unclamped knot sequence: the expanded knots sit in the
// middle and M1 = (degree+1) - mult[first] extra knots are mirrored (period-shifted) onto each
// end, so the result is the geometrically identical unclamped representation over one period.
USD_BREP_EXPORT pxr::VtArray<double> BuildFlatKnots(const std::vector<double>& rKnots, const std::vector<int>& rMults, int iDegree, bool bPeriodic);

} // end namespace UsdBrep

#endif // _USD_BREP_KNOTS_H_
