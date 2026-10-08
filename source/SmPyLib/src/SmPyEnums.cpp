// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges enums from:
//   SmApiPolygons.h (SmBooleanOperationType),
//   SmCurveTypes.h  (SmCurveContinuityType),
//   SmFilletSolver.h (SmFilletSurfaceGeneratorType),
//   SmPolyMerge.h   (SmPolyBooleanOperationType).
//
// Also binds the binding-only ``CapEnds`` and ``OffsetType`` enums
// declared in SmPyCommon.h - the underlying SM_API takes raw ints
// for these selectors and only documents the magic values in header
// comments.  We do not ``export_values()`` them: their members
// (``BOTH``, ``LEFT``, ``RIGHT``, ``NONE``, ``START``, ``END``)
// are too generic for module-level export and would collide with
// each other and with future selectors.  Users write the qualified
// form ``sm.CapEnds.BOTH`` / ``sm.OffsetType.LEFT``.

#include "SmPyCommon.h"

#include <SmApiPolygons.h>
#include <SmCurveTypes.h>
#include <SmFilletSolver.h>
#include <SmPolyMerge.h>

void bind_enums(py::module_& m)
{
    py::enum_<SmBooleanOperationType>(m, "BooleanOp")
        .value("UNION",        SM_BO_UNION)
        .value("DIFFERENCE",   SM_BO_DIFFERENCE)
        .value("INTERSECTION", SM_BO_INTERSECTION)
        .value("MERGE",        SM_BO_MERGE)
        .export_values();

    py::enum_<Sm2DBooleanOperationType>(m, "BooleanOp2D")
        .value("UNION",        SM_2D_UNION)
        .value("INTERSECTION", SM_2D_INTERSECTION)
        .value("DIFFERENCE",   SM_2D_DIFFERENCE)
        .value("EXCLUSIVE_OR", SM_2D_EXCLUSIVE_OR)
        .value("MERGE",        SM_2D_MERGE)
        .export_values();

    py::enum_<SmFilletSurfaceGeneratorType>(m, "FilletXSect")
        .value("LINEAR",   SM_FSG_LINEAR)
        .value("CIRCULAR", SM_FSG_CIRCULAR)
        .value("BLEND",    SM_FSG_BLEND_CURVE)
        .export_values();

    py::enum_<SmTrimType>(m, "TrimType")
        .value("KEEP_POINT",   SM_TT_KEEP_POINT)
        .value("DELETE_POINT", SM_TT_DELETE_POINT)
        .value("SPLIT",        SM_TT_SPLIT)
        .export_values();

    py::enum_<SmContinuityType>(m, "ContinuityType")
        .value("UNDEFINED",     SM_CT_UNDEFINED)
        .value("DISCONTINUOUS", SM_CT_DISCONTINUOUS)
        .value("C0",            SM_CT_C0)
        .value("G1",            SM_CT_G1)
        .value("G1R",           SM_CT_G1R)
        .value("G1_G2",         SM_CT_G1_G2)
        .value("G1_G2_G3",      SM_CT_G1_G2_G3)
        .value("C1",            SM_CT_C1)
        .value("C1_G2",         SM_CT_C1_G2)
        .value("C1_G2_G3",      SM_CT_C1_G2_G3)
        .value("C1_C2",         SM_CT_C1_C2)
        .value("C1_C2_G3",      SM_CT_C1_C2_G3)
        .value("C1_C2_C3",      SM_CT_C1_C2_C3)
        .value("C_INFINITY",    SM_CT_CINFINITY)
        .export_values();

    py::enum_<SmCurveParameterizationType>(m, "CurveParameterization",
        "Knot parameterization for point-interpolating curve fits "
        "(``create_curve_interp_points``). ``UNIFORM`` spaces knots evenly; "
        "``CHORD_LENGTH`` and ``CENTRIPETAL`` space them by the distance "
        "between successive points (centripetal reduces overshoot on sharp "
        "turns).")
        .value("UNIFORM",      SM_CP_UNIFORM)
        .value("CHORD_LENGTH", SM_CP_CHORDLENGTH)
        .value("CENTRIPETAL",  SM_CP_CENTRIPETAL)
        .export_values();

    py::enum_<SmBSplineCurveForm>(m, "BSplineCurveForm")
        .value("POLYLINE",        SM_CF_POLYLINE_FORM)
        .value("CIRCULAR_ARC",    SM_CF_CIRCULAR_ARC)
        .value("ELLIPTIC_ARC",    SM_CF_ELLIPTIC_ARC)
        .value("PARABOLIC_ARC",   SM_CF_PARABOLIC_ARC)
        .value("HYPERBOLIC_ARC",  SM_CF_HYPERBOLIC_ARC)
        .value("HELICAL_ARC",     SM_CF_HELICAL_ARC)
        .value("UNSPECIFIED",     SM_CF_UNSPECIFIED)
        .export_values();

    py::enum_<SmPolyBooleanOperationType>(m, "PolyBooleanOp")
        .value("UNION",         SM_PBO_UNION)
        .value("INTERSECTION",  SM_PBO_INTERSECTION)
        .value("DIFFERENCE",    SM_PBO_DIFFERENCE)
        .value("MERGE",         SM_PBO_MERGE)
        .value("PARTIAL_MERGE", SM_PBO_PARTIAL_MERGE)
        .export_values();

    py::enum_<CapEnds>(m, "CapEnds",
        "End-cap selector for sweeps that distinguish a 'start' end "
        "from an 'end' end (``draft_sweep``, ``taper_extrude``, "
        "``sweep_along_planar_path``). ``START`` and ``END`` map to "
        "the kernel's profile-side and offset/path-side caps "
        "respectively; the docstring on each consumer spells out "
        "which side is which. Sweeps with a symmetric on/off "
        "contract (``linear_sweep``, ``pipe_sweep``, ...) keep their "
        "plain ``bool`` parameter.")
        .value("NONE",  CapEnds::NONE)
        .value("START", CapEnds::START)
        .value("END",   CapEnds::END)
        .value("BOTH",  CapEnds::BOTH);

    py::enum_<OffsetType>(m, "OffsetType",
        "Offset side selector for ``create_offset_profile``. "
        "``LEFT`` / ``RIGHT`` produce a one-sided offset; ``BOTH`` "
        "produces a two-sided band region. ``shell_result=True`` is "
        "only valid with ``LEFT`` or ``RIGHT``.")
        .value("LEFT",  OffsetType::LEFT)
        .value("RIGHT", OffsetType::RIGHT)
        .value("BOTH",  OffsetType::BOTH);
}
