// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiBrep.h (SmApiShellBrepFull, SmApiOffsetBrepFull) and
// SmApiPrimitives.h (SmApiCreateOffsetProfile).

#include "SmPyCommon.h"

#include <SmApiBrep.h>
#include <SmApiPrimitives.h>

void bind_offset(BoundModule& m)
{
    m.def("shell_brep", [](SmBrep* brep, double dist, bool extendedOffset,
                           bool selfInt, bool createSolid,
                           const std::vector<SmFace*>& facesToShell) {
        SmTArray<const SmFace*> arr;
        for (auto* f : facesToShell) arr.Add(f);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiShellBrepFull(brep, dist,
                     extendedOffset ? TRUE : FALSE,
                     selfInt ? TRUE : FALSE,
                     createSolid ? TRUE : FALSE,
                     arr, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("distance"),
       py::kw_only(),
       py::arg("extended_offset") = true, py::arg("self_intersection") = true,
       py::arg("create_solid") = true, py::arg("faces_to_shell") = std::vector<SmFace*>(),
       "Pure: returns new Brep; brep unchanged.\n"
       "\n"
       "Hollow out a solid by removing selected faces and offsetting the rest.\n"
       "\n"
       "Args:\n"
       "    brep: Source Brep. Not modified.\n"
       "    distance: Signed wall thickness. Positive expands outward from\n"
       "        the source; negative creates an inward wall. Must be smaller than\n"
       "        the minimum face dimension to avoid degenerate geometry.\n"
       "    extended_offset: If ``True`` (default), extend offset surfaces at\n"
       "        convex edges until they intersect, producing sharp corners.\n"
       "        ``False`` may leave gaps at sharp convex edges. Keyword-only.\n"
       "    self_intersection: If ``True`` (default), detect and resolve\n"
       "        self-intersections that occur in concave regions when\n"
       "        ``abs(distance)`` exceeds the local curvature radius.\n"
       "        Keyword-only.\n"
       "    create_solid: Join sheet-body lamina edges when ``True``. Solid\n"
       "        inputs always use Boolean wall construction. Keyword-only.\n"
       "    faces_to_shell: Faces to remove. Empty creates a closed wall.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: hollowed shell with ``faces_to_shell`` open.\n"
       "\n"
       "See Also:\n"
       "    offset_brep: Offset every face without removing any.\n"
       "\n"
       "Wraps: SmApiShellBrepFull");

    m.def("offset_brep", [](SmBrep* brep, double dist, bool extendedOffset, bool selfInt) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiOffsetBrepFull(brep, dist,
                     extendedOffset ? TRUE : FALSE,
                     selfInt ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("distance"),
       py::kw_only(),
       py::arg("extended_offset") = true, py::arg("self_intersection") = true,
       "Pure: returns new Brep; brep unchanged.\n"
       "\n"
       "Create a new Brep whose surfaces are uniformly offset from an existing Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Source Brep. Not modified.\n"
       "    distance: Offset distance. Positive offsets outward along surface\n"
       "        normals (larger shape); negative offsets inward (smaller).\n"
       "        Must be smaller than the minimum radius of curvature in any\n"
       "        concave region to avoid invalid geometry.\n"
       "    extended_offset: If ``True`` (default), extend offset surfaces at\n"
       "        convex edges until they intersect, producing sharp corners.\n"
       "        ``False`` may leave gaps. Keyword-only.\n"
       "    self_intersection: If ``True`` (default), detect and resolve\n"
       "        self-intersections in concave regions. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: offset Brep.\n"
       "\n"
       "See Also:\n"
       "    shell_brep: Hollow-out variant with selected faces left open.\n"
       "    create_offset_surface: Offset a single surface in 3D.\n"
       "    offset_curve: Offset a single curve in 2D.\n"
       "\n"
       "Wraps: SmApiOffsetBrepFull");

    m.def("create_offset_profile", [](std::vector<SmCurve*>& curves, double dist,
                                      OffsetType offsetType, bool roundCorners,
                                      bool shellResult) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateOffsetProfile(arr, dist,
                     static_cast<unsigned long>(offsetType),
                     roundCorners ? TRUE : FALSE, shellResult ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("distance"),
       py::kw_only(),
       py::arg("offset_type") = OffsetType::BOTH,
       py::arg("round_corners") = true, py::arg("shell_result") = false,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Create an offset region from a closed loop of planar curves.\n"
       "\n"
       "Args:\n"
       "    curves: Coplanar curves forming at least one closed loop.\n"
       "    distance: Offset distance. Must be positive.\n"
       "    offset_type: ``OffsetType`` selector. ``LEFT`` / ``RIGHT`` produce\n"
       "        a one-sided offset; ``BOTH`` (default) produces a two-sided\n"
       "        band region. Keyword-only.\n"
       "    round_corners: If ``True`` (default), round expanding corners.\n"
       "        ``False`` extends the corners linearly until they meet.\n"
       "        Keyword-only.\n"
       "    shell_result: If ``True``, return a shell topology instead of a\n"
       "        face region. Only valid when ``offset_type`` is ``LEFT`` or\n"
       "        ``RIGHT`` - not ``BOTH``. Defaults to ``False``. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: sheet-body Brep representing the offset region.\n"
       "\n"
       "Notes:\n"
       "    This is a curve-input operation that creates a new Brep, distinct\n"
       "    from ``offset_brep`` which offsets an existing Brep's surfaces.\n"
       "\n"
       "See Also:\n"
       "    offset_brep: Offset the surfaces of an existing Brep.\n"
       "    offset_curve: Offset a single B-spline curve in 2D.\n"
       "    create_planar_faces: Build a planar Brep from curve loops without\n"
       "        offsetting.\n"
       "\n"
       "Wraps: SmApiCreateOffsetProfile");
}
