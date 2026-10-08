// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiFillets.h (SmApiChamferFillet, SmApiCircularFillet,
// SmApiFilletEdges, SmApiVariableRadiusFillet, SmApiRemoveFillet),
// SmApiBrep.h (SmApiFilletEdgesPerEdge, SmApiSurfaceSurfaceFillet,
// SmApiFilletPreview, SmApiSetBevelCorners).

#include "SmPyCommon.h"

#include <SmApiBrep.h>
#include <SmApiFillets.h>
#include <SmFilletSolver.h>

void bind_fillets(BoundModule& m)
{
    m.def("chamfer_fillet", [](SmBrep* brep, double radius) {
        CHECK_STATUS(SmApiChamferFillet(brep, radius));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("radius"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Chamfer (flat-bevel) every edge of a Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to chamfer in place.\n"
       "    radius: Chamfer width: the straight-line distance across the\n"
       "        chamfer face, between the lines where it meets the two\n"
       "        adjacent faces. It is not the setback along each face: at a\n"
       "        90-degree edge each face is cut back by radius / sqrt(2).\n"
       "        Must be positive and smaller than the minimum adjacent face\n"
       "        dimension.\n"
       "\n"
       "Notes:\n"
       "    Applies to every edge with a constant-distance chamfer solver.\n"
       "    ``fillet_edges`` with ``xsect_type=0`` cuts flat between the\n"
       "    lines where a circular fillet of that radius would meet the\n"
       "    faces, so its radius is not a chamfer width: at a 90-degree edge\n"
       "    it is the setback.\n"
       "\n"
       "See Also:\n"
       "    circular_fillet: Round every edge instead of chamfering.\n"
       "    fillet_edges: Per-edge selection with cross-section choice.\n"
       "\n"
       "Wraps: SmApiChamferFillet");

    m.def("circular_fillet", [](SmBrep* brep, double radius) {
        CHECK_STATUS(SmApiCircularFillet(brep, radius));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("radius"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Apply a constant-radius circular fillet to every edge of a Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to fillet in place.\n"
       "    radius: Fillet radius. Must be positive and smaller than the\n"
       "        minimum adjacent face dimension. Rule of thumb: at most one\n"
       "        third of the smallest adjacent face dimension to avoid\n"
       "        adjacent fillets colliding.\n"
       "\n"
       "Notes:\n"
       "    Applies to every edge in the Brep. For selective filleting use\n"
       "    ``fillet_edges``.\n"
       "\n"
       "See Also:\n"
       "    chamfer_fillet: Flat bevel instead of round.\n"
       "    fillet_edges: Per-edge selection with cross-section choice.\n"
       "    variable_radius_fillet: Radius varies along the edge.\n"
       "    remove_fillet: Defeature fillets.\n"
       "\n"
       "Wraps: SmApiCircularFillet");

    m.def("fillet_edges", [](SmBrep* brep, std::vector<SmEdge*>& edges,
                             double radius, unsigned long xsect,
                             unsigned long continuity, double thumbweight) {
        SmTArray<SmEdge*> arr;
        for (auto* e : edges) arr.Add(e);
        CHECK_STATUS(SmApiFilletEdges(brep, arr, radius, xsect, continuity, thumbweight));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("edges"),
       py::kw_only(),
       py::arg("radius"),
       py::arg("xsect_type") = 1, py::arg("continuity") = 1,
       py::arg("thumbweight") = 1.0,
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Fillet selected edges of a Brep with a chosen cross-section type.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to fillet in place.\n"
       "    edges: Edges of ``brep`` to fillet (typically a subset of\n"
       "        ``brep.edges()``).\n"
       "    radius: Fillet radius. Must be positive. Keyword-only.\n"
       "    xsect_type: Cross-section integer: ``0`` = linear (not the\n"
       "        constant-distance ``chamfer_fillet`` solver), ``1`` = circular\n"
       "        (default), ``2`` = blend (variable continuity blend curve).\n"
       "        These are not the ``FilletXSect`` values:\n"
       "        ``FilletXSect.LINEAR`` is ``1`` and selects circular.\n"
       "        Keyword-only.\n"
       "    continuity: For ``xsect_type == 2``: ``1`` = G1 (tangent),\n"
       "        ``2`` = G2 (curvature), ``3`` = G3. Ignored for other\n"
       "        cross-section types. Defaults to ``1``. Where three filleted\n"
       "        edges meet, only G1 continuity is guaranteed across the\n"
       "        corner-patch boundary. Keyword-only.\n"
       "    thumbweight: Fullness control for blend cross-sections; values\n"
       "        greater than ``1`` produce a fuller blend, less than ``1``\n"
       "        a thinner one. Ignored unless ``xsect_type == 2``. Defaults\n"
       "        to ``1.0``. Keyword-only.\n"
       "\n"
       "Notes:\n"
       "    Use G2 for visible consumer surfaces, G3 for automotive Class-A.\n"
       "\n"
       "See Also:\n"
       "    circular_fillet: Apply a circular fillet to all edges.\n"
       "    chamfer_fillet: Apply a chamfer to all edges.\n"
       "    variable_radius_fillet: Linearly varying radius along the edge.\n"
       "    fillet_edges_per_edge: Per-edge radius and cross-section.\n"
       "    fillet_preview: Compute the fillet surfaces without modifying\n"
       "        the Brep.\n"
       "\n"
       "Wraps: SmApiFilletEdges");

    m.def("variable_radius_fillet", [](SmBrep* brep, std::vector<SmEdge*>& edges,
                                       double startR, double endR,
                                       unsigned long xsect, unsigned long continuity) {
        SmTArray<SmEdge*> arr;
        for (auto* e : edges) arr.Add(e);
        CHECK_STATUS(SmApiVariableRadiusFillet(brep, arr, startR, endR, xsect, continuity));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("edges"),
       py::kw_only(),
       py::arg("start_radius"), py::arg("end_radius"),
       py::arg("xsect_type") = 1, py::arg("continuity") = 1,
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Fillet selected edges with linearly varying radius along each edge.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to fillet in place.\n"
       "    edges: Edges to fillet.\n"
       "    start_radius: Fillet radius at the start of each edge. Must be\n"
       "        positive. Keyword-only.\n"
       "    end_radius: Fillet radius at the end of each edge. Must be\n"
       "        positive. Keyword-only.\n"
       "    xsect_type: Cross-section integer: ``0`` = linear, ``1`` =\n"
       "        circular (default), ``2`` = blend. These are not the\n"
       "        ``FilletXSect`` values: ``FilletXSect.LINEAR`` is ``1`` and\n"
       "        selects circular. Keyword-only.\n"
       "    continuity: For ``xsect_type == 2``: ``1`` = G1, ``2`` = G2,\n"
       "        ``3`` = G3. At a vertex where three filleted edges meet, only\n"
       "        G1 continuity is guaranteed across the corner-patch boundary.\n"
       "        Defaults to ``1``. Keyword-only.\n"
       "\n"
       "Notes:\n"
       "    Radius varies linearly from ``start_radius`` to ``end_radius``\n"
       "    along each selected edge.\n"
       "\n"
       "See Also:\n"
       "    fillet_edges: Constant-radius variant.\n"
       "    fillet_edges_per_edge: Per-edge constant radii and cross-sections.\n"
       "\n"
       "Wraps: SmApiVariableRadiusFillet");

    m.def("remove_fillet", [](SmBrep* brep) {
        CHECK_STATUS(SmApiRemoveFillet(brep));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Remove fillet faces from a Brep (defeaturing).\n"
       "\n"
       "Args:\n"
       "    brep: Brep to defeature in place.\n"
       "\n"
       "Notes:\n"
       "    Acts on the entire Brep; the kernel detects fillet faces by\n"
       "    geometric heuristics and removes them, repairing the surrounding\n"
       "    faces back to their pre-fillet topology where possible.\n"
       "\n"
       "See Also:\n"
       "    circular_fillet: Inverse - apply fillets.\n"
       "    fillet_edges: Selective filleting.\n"
       "\n"
       "Wraps: SmApiRemoveFillet");

    m.def("fillet_edges_per_edge", [](SmBrep* brep, std::vector<SmEdge*>& edges,
                                     std::vector<double>& radii,
                                     std::vector<SmFilletSurfaceGeneratorType>& xsectTypes,
                                     unsigned long continuity, double thumbweight) {
        SmTArray<SmEdge*> earr;
        SmTArray<double> rarr;
        SmTArray<SmFilletSurfaceGeneratorType> xarr;
        for (auto* e : edges) earr.Add(e);
        for (auto r : radii) rarr.Add(r);
        for (auto x : xsectTypes) xarr.Add(x);
        CHECK_STATUS(SmApiFilletEdgesPerEdge(brep, earr, rarr, xarr, continuity, thumbweight));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("edges"), py::arg("radii"), py::arg("xsect_types"),
       py::kw_only(),
       py::arg("continuity") = 1, py::arg("thumbweight") = 1.0,
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Fillet selected edges with a per-edge radius and cross-section type.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to fillet in place.\n"
       "    edges: Edges to fillet.\n"
       "    radii: Per-edge fillet radius. Length must equal ``len(edges)``.\n"
       "        Each value must be positive.\n"
       "    xsect_types: Per-edge cross-section as a list of\n"
       "        ``FilletXSect`` values. Length must equal ``len(edges)``.\n"
       "    continuity: For blend cross-sections: ``1`` = G1, ``2`` = G2,\n"
       "        ``3`` = G3. At a vertex where three filleted edges meet, only\n"
       "        G1 continuity is guaranteed across the corner-patch boundary.\n"
       "        Defaults to ``1``. Keyword-only.\n"
       "    thumbweight: Fullness control for blend cross-sections.\n"
       "        Defaults to ``1.0``. Keyword-only.\n"
       "\n"
       "See Also:\n"
       "    fillet_edges: Single radius and cross-section across all edges.\n"
       "    variable_radius_fillet: Radius varies along each edge.\n"
       "\n"
       "Wraps: SmApiFilletEdgesPerEdge");

    m.def("surface_surface_fillet", [](SmSurface* srf1, SmSurface* srf2,
                                       double r1, double r2, double tol,
                                       unsigned long xsect, bool mirror, bool complement) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiSurfaceSurfaceFillet(srf1, srf2, r1, r2, tol, r,
                     xsect, mirror ? TRUE : FALSE, complement ? TRUE : FALSE));
        return r;
    }, py::return_value_policy::reference,
       py::arg("surface1"), py::arg("surface2"),
       py::kw_only(),
       py::arg("radius1"), py::arg("radius2"), py::arg("tolerance"),
       py::arg("xsect_type") = 1, py::arg("mirror") = false, py::arg("complement") = false,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Build a fillet Brep between two surfaces.\n"
       "\n"
       "Args:\n"
       "    surface1: First base surface.\n"
       "    surface2: Second base surface.\n"
       "    radius1: Signed radius from ``surface1`` to the fillet rail.\n"
       "        Keyword-only.\n"
       "    radius2: Signed radius from ``surface2`` to the fillet rail.\n"
       "        Keyword-only.\n"
       "    tolerance: Maximum allowed distance from the computed rails to\n"
       "        the base surfaces. Must be positive. Keyword-only.\n"
       "    xsect_type: Cross-section integer: ``0`` = linear, ``1`` =\n"
       "        approximate circular (default), ``2`` = exact circular.\n"
       "        Keyword-only.\n"
       "    mirror: If ``True``, mirror the fillet (turn it inside-out).\n"
       "        Defaults to ``False``. Keyword-only.\n"
       "    complement: If ``True``, take the complement of the circular\n"
       "        cross-section (the longer arc between the rails). Defaults\n"
       "        to ``False``. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: sheet-body Brep containing the fillet surface(s).\n"
       "\n"
       "Notes:\n"
       "    Stability: this API is currently known to crash inside the kernel\n"
       "    for some surface combinations.\n"
       "\n"
       "See Also:\n"
       "    fillet_edges: Edge-based filleting on a Brep.\n"
       "\n"
       "Wraps: SmApiSurfaceSurfaceFillet");

    m.def("fillet_preview", [](SmBrep* brep, std::vector<SmEdge*>& edges, double radius) {
        SmTArray<SmEdge*> earr;
        SmTArray<SmSurface*> preview;
        for (auto* e : edges) earr.Add(e);
        CHECK_STATUS(SmApiFilletPreview(brep, earr, radius, preview));
        std::vector<SmSurface*> v;
        for (ULONG i = 0; i < preview.GetSize(); i++) v.push_back(preview[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("edges"),
       py::kw_only(),
       py::arg("radius"),
       "Pure: returns new list[Surface]; brep unchanged.\n"
       "\n"
       "Compute the fillet surfaces for a set of edges without modifying the Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Brep providing the edge context. Not modified.\n"
       "    edges: Edges to preview filleting on.\n"
       "    radius: Fillet radius. Must be positive. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    list[Surface]: standalone surfaces representing the fillet\n"
       "    geometry that ``fillet_edges`` would produce with the same\n"
       "    radius.\n"
       "\n"
       "Notes:\n"
       "    Stability: this API is currently known to crash inside the kernel\n"
       "    for some inputs.\n"
       "\n"
       "See Also:\n"
       "    fillet_edges: Apply the fillet for real (modifies the Brep).\n"
       "\n"
       "Wraps: SmApiFilletPreview");

    // Lambda parameter order rearranged so the two collections sit
    // together (``edges``, ``vertices``) followed by the scalar
    // (``radius``); the C ``SmApiSetBevelCorners`` is still called in its
    // original ``(brep, edges, radius, vertices)`` order.  Everything after
    // ``brep`` is keyword-only, so the user-visible call shape is fully
    // name-bound and the previous ``(edges, radius, vertices)`` sandwich
    // is impossible to express.
    m.def("set_bevel_corners", [](SmBrep* brep, std::vector<SmEdge*>& edges,
                                  std::vector<SmVertex*>& vertices, double radius) {
        SmTArray<SmEdge*> earr;
        SmTArray<SmVertex*> varr;
        for (auto* e : edges) earr.Add(e);
        for (auto* v : vertices) varr.Add(v);
        CHECK_STATUS(SmApiSetBevelCorners(brep, earr, radius, varr));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"),
       py::kw_only(),
       py::arg("edges"), py::arg("vertices"), py::arg("radius"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Fillet edges with a constant radius, beveling the named corners.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to fillet in place.\n"
       "    edges: Edges to fillet. Keyword-only.\n"
       "    vertices: Corner vertices that get a bevel instead of a rounded\n"
       "        corner. Each must be a corner where filleted edges meet.\n"
       "        Keyword-only.\n"
       "    radius: Constant fillet radius. Keyword-only.\n"
       "\n"
       "Notes:\n"
       "    This call performs the fillet itself; no follow-on\n"
       "    ``fillet_edges`` call is needed. A vertex that is not a corner of\n"
       "    the filleted edges raises ``RuntimeError``\n"
       "    (``SM_ERR_NULL_POINTER``).\n"
       "\n"
       "See Also:\n"
       "    fillet_edges: Fillet edges without beveled corners.\n"
       "\n"
       "Wraps: SmApiSetBevelCorners");
}
