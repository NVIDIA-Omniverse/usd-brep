// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiBrep.h (SmApiTurnToNurbs).
//
// Miscellaneous brep-level operations that act on a single Brep in place
// (analytic-to-NURBS conversion).  Distinct from offset / shell
// (SmPyOffset.cpp), boolean ops (SmPyBooleans.cpp), and topology queries
// (SmPyQueries.cpp).

#include "SmPyCommon.h"

#include <SmApiBrep.h>

void bind_brep_ops(BoundModule& m)
{
    m.def("turn_to_nurbs", [](SmBrep* brep) {
              smpy_preflight_brep_nurbs_conversion(brep, "SmApiTurnToNurbs");
              CHECK_STATUS(SmApiTurnToNurbs(brep));
              return brep;
          }, py::return_value_policy::reference,
          py::arg("brep"),
          "Mutates: brep; returns the same handle for chaining.\n"
          "\n"
          "Replace every analytic / periodic surface in the Brep (cylinder,\n"
          "cone, sphere, torus, plane) with an equivalent NURBS surface\n"
          "in place. No-op on surfaces that are already NURBS.\n"
          "\n"
          "Args:\n"
          "    brep: Brep to convert in place.\n"
          "\n"
          "Notes:\n"
          "    Analytic surfaces, lines and circles are copied exactly from\n"
          "    their NURBS form. Offset surfaces and other edge curves are\n"
          "    approximated, and face UV trim curves are removed (later\n"
          "    operations regenerate them). If a borrowed handle returned by\n"
          "    ``Face.surface()`` or ``Edge.curve()`` would be replaced, the\n"
          "    call raises ``RuntimeError`` before modifying the Brep. Release\n"
          "    those handles and reacquire them after conversion, or convert a\n"
          "    ``brep.copy()`` to preserve handles into the original.\n"
          "\n"
          "Wraps: SmApiTurnToNurbs");
}
