// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiHeal.h (SmApiHealBrep).

#include "SmPyCommon.h"

#include <SmApiHeal.h>

void bind_heal(BoundModule& m)
{
    m.def("heal_brep", [](SmBrep* brep) {
        CHECK_STATUS(SmApiHealBrep(brep));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Run the BRep healer to fix common geometric and topological defects.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to heal in place.\n"
       "\n"
       "Notes:\n"
       "    Runs the full healer sequence (``SM_HO_ALL``) on the Brep.\n"
       "    Supported repairs include adjusting tolerances to measured gaps,\n"
       "    combining coincident vertices, removing degeneracies, and\n"
       "    repairing periodic seams and sheet/region classifications.\n"
       "    General geometric gap closing and endpoint refitting are not\n"
       "    implemented. Validate the resulting topology when required.\n"
       "\n"
       "    Most useful for geometry that came in from outside the kernel\n"
       "    (USD import, manual topology edits). Programmatically built\n"
       "    geometry from this API is normally already clean. The USD\n"
       "    import path runs healing on every imported Brep when its ``heal``\n"
       "    parameter is ``True``; Python imports default to ``False``.\n"
       "\n"
       "See Also:\n"
       "    stitch_into_solid: Recover topological connectivity for loose\n"
       "        face collections.\n"
       "    is_manifold_solid: Validation after healing.\n"
       "\n"
       "Wraps: SmApiHealBrep");
}
