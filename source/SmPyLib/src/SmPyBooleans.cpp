// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiBrep.h (SmApiBoolean / Union / Difference / Intersection /
// Merge / MergeBreps / Boolean2d / BooleanLists / BooleanTrees /
// BooleanWithOptions / BooleanWithCurves / NonManifoldBoolean /
// PiecewiseMerge).

#include "SmPyCommon.h"

#include <SmApiBrep.h>

void bind_booleans(BoundModule& m)
{
    // Operands ``brep1``/``brep2`` are positional (subject-first, matches
    // the union/difference/intersection wrappers and every other modeling
    // op).  ``operation`` is keyword-only via ``py::kw_only`` so the enum
    // never gets misordered relative to the operands - call sites read
    // ``boolean(a, b, operation=BooleanOp.UNION)``.
    m.def("boolean", [](SmBrep* a, SmBrep* b, SmBooleanOperationType op) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBoolean(a, b, op, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"), py::kw_only(), py::arg("operation"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Combine two Breps with a boolean operation.\n"
       "\n"
       "Args:\n"
       "    brep1: Primary body. For ``DIFFERENCE`` this is the workpiece.\n"
       "        Consumed by the operation.\n"
       "    brep2: Tool body. For ``DIFFERENCE`` this is the cutting shape.\n"
       "        Consumed by the operation.\n"
       "    operation: ``BooleanOp.{UNION, DIFFERENCE, INTERSECTION, MERGE}``.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: combined body. The two input Breps are consumed and must\n"
       "    not be reused.\n"
       "\n"
       "Notes:\n"
       "    SMLib uses exact NURBS surface-surface intersection, so results\n"
       "    maintain exact geometry. ``DIFFERENCE`` is order-sensitive:\n"
       "    ``boolean(A, B, operation=BooleanOp.DIFFERENCE)`` removes ``B``\n"
       "    from ``A``.\n"
       "\n"
       "See Also:\n"
       "    boolean_union: Convenience wrapper for ``UNION``.\n"
       "    boolean_difference: Convenience wrapper for ``DIFFERENCE``.\n"
       "    boolean_intersection: Convenience wrapper for ``INTERSECTION``.\n"
       "    boolean_with_options: Same with cookie-cutter, imprinting flags.\n"
       "    merge_breps: N-ary version that combines a list in one call.\n"
       "\n"
       "Wraps: SmApiBoolean");

    m.def("boolean_union", [](SmBrep* a, SmBrep* b) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBooleanUnion(a, b, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Compute the boolean union (additive) of two Breps.\n"
       "\n"
       "Args:\n"
       "    brep1: First body. Consumed by the operation.\n"
       "    brep2: Second body. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    Brep: union of the two bodies (material from either).\n"
       "\n"
       "See Also:\n"
       "    boolean: Generic operation taking a ``BooleanOp`` value.\n"
       "    boolean_difference: Subtractive variant.\n"
       "    boolean_intersection: Common-overlap variant.\n"
       "    boolean_merge: Combine without removing any faces.\n"
       "\n"
       "Wraps: SmApiBooleanUnion");

    m.def("boolean_difference", [](SmBrep* a, SmBrep* b) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBooleanDifference(a, b, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Compute the boolean difference (subtractive) of two Breps.\n"
       "\n"
       "Args:\n"
       "    brep1: Workpiece body. Consumed by the operation.\n"
       "    brep2: Tool body to subtract. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    Brep: ``brep1`` with ``brep2`` carved out.\n"
       "\n"
       "Notes:\n"
       "    Order-sensitive: ``boolean_difference(A, B)`` is not the same as\n"
       "    ``boolean_difference(B, A)``.\n"
       "\n"
       "See Also:\n"
       "    boolean: Generic operation taking a ``BooleanOp`` value.\n"
       "    boolean_union: Additive variant.\n"
       "    boolean_intersection: Common-overlap variant.\n"
       "\n"
       "Wraps: SmApiBooleanDifference");

    m.def("boolean_intersection", [](SmBrep* a, SmBrep* b) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBooleanIntersection(a, b, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Compute the boolean intersection (common overlap) of two Breps.\n"
       "\n"
       "Args:\n"
       "    brep1: First body. Consumed by the operation.\n"
       "    brep2: Second body. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    Brep: region that is inside both bodies. If the inputs are\n"
       "    disjoint, the result has no volume.\n"
       "\n"
       "See Also:\n"
       "    boolean: Generic operation taking a ``BooleanOp`` value.\n"
       "    boolean_union: Additive variant.\n"
       "    boolean_difference: Subtractive variant.\n"
       "\n"
       "Wraps: SmApiBooleanIntersection");

    m.def("merge_breps", [](std::vector<SmBrep*>& breps, SmBooleanOperationType op) {
        SmTArray<SmBrep*> arr;
        for (auto* b : breps) arr.Add(b);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiMergeBreps(arr, op, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("breps"), py::kw_only(), py::arg("operation"),
       "Consumes: breps; returns new Brep.\n"
       "\n"
       "Combine a list of Breps in one boolean call.\n"
       "\n"
       "Args:\n"
       "    breps: Two or more unique Breps to combine. Once input validation\n"
       "        succeeds, all input Brep handles are consumed.\n"
       "    operation: ``BooleanOp.{UNION, DIFFERENCE, INTERSECTION, MERGE}``.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: combined body.\n"
       "\n"
       "Notes:\n"
       "    Operands retain the kernel's longstanding last-to-first evaluation\n"
       "    order. For example, ``DIFFERENCE``\n"
       "    on ``[a, b, c]`` computes ``(c - b) - a``.\n"
       "    Lists with fewer than two entries, ``None``, or duplicate handles\n"
       "    are rejected without consuming any inputs.\n"
       "\n"
       "See Also:\n"
       "    boolean: Pairwise version.\n"
       "    evaluate_csg_tree: Mixed-operation CSG tree in postfix form.\n"
       "\n"
       "Wraps: SmApiMergeBreps");

    m.def("boolean_with_options", [](SmBrep* a, SmBrep* b,
                                     SmBooleanOperationType op,
                                     bool cookieCutter, bool imprinting,
                                     bool postProcess, bool imprintAndClassify,
                                     bool keepOtherBrep) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBooleanWithOptions(a, b, op,
            cookieCutter      ? TRUE : FALSE,
            imprinting        ? TRUE : FALSE,
            postProcess       ? TRUE : FALSE,
            imprintAndClassify? TRUE : FALSE,
            keepOtherBrep     ? TRUE : FALSE,
            r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"), py::kw_only(), py::arg("operation"),
       py::arg("cookie_cutter") = false, py::arg("imprinting") = false,
       py::arg("post_process") = true, py::arg("imprint_and_classify") = false,
       py::arg("keep_other_brep") = false,
       "Consumes: brep1 (always), brep2 (unless keep_other_brep); returns new Brep.\n"
       "\n"
       "Boolean combine two Breps with fine-grained behavior flags.\n"
       "\n"
       "Args:\n"
       "    brep1: Primary body. Consumed unless the result is sheet-only.\n"
       "    brep2: Tool body. Consumed unless ``keep_other_brep`` is ``True``.\n"
       "    operation: ``BooleanOp.{UNION, DIFFERENCE, INTERSECTION, MERGE}``.\n"
       "        Keyword-only (along with all flags below).\n"
       "    cookie_cutter: If ``True``, only remove faces from ``brep1``\n"
       "        (``brep2`` acts as a cookie cutter). Defaults to ``False``.\n"
       "    imprinting: If ``True``, imprint the intersection curves on\n"
       "        ``brep1`` and stop without removing material. Defaults to\n"
       "        ``False``.\n"
       "    post_process: If ``True`` (default), clean up redundant\n"
       "        topological edges and vertices after the boolean.\n"
       "    imprint_and_classify: If ``True``, imprint and mark faces for\n"
       "        deletion without actually deleting them. Defaults to\n"
       "        ``False``.\n"
       "    keep_other_brep: If ``True``, do not consume ``brep2`` so it can\n"
       "        be reused as a tool. Defaults to ``False``.\n"
       "\n"
       "Returns:\n"
       "    Brep: result of the operation under the supplied flags.\n"
       "\n"
       "See Also:\n"
       "    boolean: Plain boolean without behavior flags.\n"
       "\n"
       "Wraps: SmApiBooleanWithOptions");

    m.def("boolean_2d", [](SmBrep* a, SmBrep* b, Sm2DBooleanOperationType op) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBoolean2d(a, b, op, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"), py::kw_only(), py::arg("operation"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Combine two planar Breps with a 2D boolean operation.\n"
       "\n"
       "Args:\n"
       "    brep1: First planar body. Consumed by the operation.\n"
       "    brep2: Second planar body. Consumed by the operation.\n"
       "    operation: ``BooleanOp2D.{UNION, INTERSECTION, DIFFERENCE,\n"
       "        EXCLUSIVE_OR, MERGE}``. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: planar combined body.\n"
       "\n"
       "Notes:\n"
       "    Both inputs must be coplanar planar bodies. ``EXCLUSIVE_OR``\n"
       "    is equivalent to ``(A union B) - (A intersect B)``.\n"
       "\n"
       "See Also:\n"
       "    boolean: 3D boolean combination.\n"
       "    create_planar_faces: Build a planar Brep from curve loops.\n"
       "\n"
       "Wraps: SmApiBoolean2d");

    m.def("boolean_lists", [](std::vector<SmBrep*>& breps1,
                              std::vector<SmSurface*>& surfaces1,
                              std::vector<SmBrep*>& breps2,
                              std::vector<SmSurface*>& surfaces2,
                              int operation) {
        SmTArray<SmBrep*> b1, b2, rb;
        SmTArray<SmSurface*> s1, s2, rs;
        for (auto* b : breps1)    b1.Add(b);
        for (auto* s : surfaces1) s1.Add(s);
        for (auto* b : breps2)    b2.Add(b);
        for (auto* s : surfaces2) s2.Add(s);
        CHECK_STATUS(SmApiBooleanLists(b1, s1, b2, s2, operation, rb, rs));
        std::vector<SmBrep*> resultBreps;
        std::vector<SmSurface*> resultSurfaces;
        for (ULONG i = 0; i < rb.GetSize(); i++) resultBreps.push_back(rb[i]);
        for (ULONG i = 0; i < rs.GetSize(); i++) resultSurfaces.push_back(rs[i]);
        return py::make_tuple(resultBreps, resultSurfaces);
    }, py::arg("breps1"), py::arg("surfaces1"),
       py::arg("breps2"), py::arg("surfaces2"),
       py::kw_only(), py::arg("operation"),
       "Consumes: breps1, surfaces1, breps2, surfaces2; returns new (breps, surfaces).\n"
       "\n"
       "Combine lists of Breps and planar surfaces in one boolean operation.\n"
       "\n"
       "Args:\n"
       "    breps1: First list of Breps.\n"
       "    surfaces1: First list of surfaces (treated as planar 2D regions).\n"
       "    breps2: Second list of Breps.\n"
       "    surfaces2: Second list of surfaces.\n"
       "    operation: Integer op selector: ``0`` = AND, ``1`` = OR,\n"
       "        ``2`` = XOR, ``3`` = A minus B, ``4`` = B minus A.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[Brep], list[Surface]]: pair ``(result_breps,\n"
       "    result_surfaces)``. The surface list is populated only if all\n"
       "    inputs were 2D.\n"
       "\n"
       "Notes:\n"
       "    Uses an integer op selector rather than the ``BooleanOp`` enum\n"
       "    because the operation set extends to ``XOR`` and the asymmetric\n"
       "    ``B minus A`` form not present in the enum.\n"
       "\n"
       "See Also:\n"
       "    merge_breps: Simpler N-ary Brep combine using ``BooleanOp``.\n"
       "    boolean_2d: Single-Brep planar boolean.\n"
       "\n"
       "Wraps: SmApiBooleanLists");

    // The kernel mutates ``breps`` in place (the first slot becomes the
    // result, surviving inputs follow), which is why the docstring says
    // "Mutates" rather than "Consumes": callers can reuse the input
    // handles directly via ``breps[0]`` after the call (no fresh Brep is
    // allocated for the result).
    m.def("evaluate_csg_tree", [](std::vector<SmBrep*>& breps,
                                  std::vector<long>& postfixTree) {
        SmTArray<SmBrep*> arr;
        SmTArray<long> tree;
        for (auto* b : breps) arr.Add(b);
        for (auto v : postfixTree) tree.Add(v);
        CHECK_STATUS(SmApiBooleanTrees(arr, tree));
        std::vector<SmBrep*> result;
        for (ULONG i = 0; i < arr.GetSize(); i++) result.push_back(arr[i]);
        return result;
    }, py::return_value_policy::reference,
       py::arg("breps"), py::arg("postfix_tree"),
       "Mutates: breps; returns the rewritten breps list (result first).\n"
       "\n"
       "Evaluate a CSG tree of boolean operations expressed in postfix form.\n"
       "\n"
       "Args:\n"
       "    breps: Input Breps referenced by index from ``postfix_tree``.\n"
       "        Modified in place: the result and any unused inputs are\n"
       "        returned in the output list.\n"
       "    postfix_tree: CSG expression in postfix (reverse Polish)\n"
       "        notation. Operand entries are non-negative indices into\n"
       "        ``breps``; operator entries encode ``BooleanOp`` values.\n"
       "\n"
       "Returns:\n"
       "    list[Brep]: result Brep first, followed by any input Breps that\n"
       "    were not consumed by the tree.\n"
       "\n"
       "See Also:\n"
       "    merge_breps: Single-operation N-ary combine.\n"
       "    boolean: Pairwise boolean.\n"
       "\n"
       "Wraps: SmApiBooleanTrees");

    m.def("boolean_merge", [](SmBrep* a, SmBrep* b) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiBooleanMerge(a, b, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Combine two Breps without removing any faces (non-manifold assembly).\n"
       "\n"
       "Args:\n"
       "    brep1: First body. Consumed by the operation.\n"
       "    brep2: Second body. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    Brep: merged body containing all faces of both inputs. Where the\n"
       "    bodies touch, the result is non-manifold (more than two faces\n"
       "    meet at an edge).\n"
       "\n"
       "Notes:\n"
       "    Equivalent to ``boolean(brep1, brep2, BooleanOp.MERGE)``. Useful\n"
       "    for assemblies of touching but non-penetrating parts.\n"
       "\n"
       "See Also:\n"
       "    boolean_union: Material from either body, with shared region\n"
       "        merged.\n"
       "    piecewise_merge: Per-face merge that preserves ``brep2``.\n"
       "\n"
       "Wraps: SmApiBooleanMerge");

    m.def("boolean_with_curves", [](SmBrep* a, SmBrep* b, SmBooleanOperationType op) {
        SmBrep* r = nullptr;
        SmTArray<SmEdge*> edges;
        CHECK_STATUS(SmApiBooleanWithCurves(a, b, op, r, edges));
        std::vector<SmEdge*> ve;
        for (ULONG i = 0; i < edges.GetSize(); i++) ve.push_back(edges[i]);
        return py::make_tuple(r, ve);
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"), py::kw_only(), py::arg("operation"),
       "Consumes: brep1, brep2; returns new (Brep, list[Edge]).\n"
       "\n"
       "Boolean combine two Breps and return the intersection edges.\n"
       "\n"
       "Args:\n"
       "    brep1: Primary body. Consumed by the operation.\n"
       "    brep2: Tool body. Consumed by the operation.\n"
       "    operation: ``BooleanOp.{UNION, DIFFERENCE, INTERSECTION, MERGE}``.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    tuple[Brep, list[Edge]]: pair ``(result, intersection_edges)``\n"
       "    where ``intersection_edges`` are the edges of the result that\n"
       "    lie on the intersection of the two input bodies (sourced from\n"
       "    ``brep1``).\n"
       "\n"
       "Notes:\n"
       "    Useful when downstream code needs to reference the seam between\n"
       "    the two bodies (e.g. for selective filleting).\n"
       "\n"
       "See Also:\n"
       "    boolean: Same operation without returning intersection edges.\n"
       "    fillet_edges: Filleting of selected edges.\n"
       "\n"
       "Wraps: SmApiBooleanWithCurves");

    m.def("non_manifold_boolean", [](SmBrep* a, SmBrep* b, SmBooleanOperationType op) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiNonManifoldBoolean(a, b, op, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"), py::kw_only(), py::arg("operation"),
       "Consumes: brep1, brep2; returns new Brep.\n"
       "\n"
       "Boolean combine two Breps allowing non-manifold (>2 faces per edge) results.\n"
       "\n"
       "Args:\n"
       "    brep1: Primary body. Consumed by the operation.\n"
       "    brep2: Second body. Consumed by the operation.\n"
       "    operation: ``BooleanOp.{UNION, DIFFERENCE, INTERSECTION, MERGE}``.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: combined body that may contain non-manifold topology.\n"
       "\n"
       "Notes:\n"
       "    Differs from ``boolean`` in that it does not enforce manifold\n"
       "    output - useful for assembly modeling where multiple bodies\n"
       "    share faces or edges.\n"
       "\n"
       "See Also:\n"
       "    boolean: Manifold-only variant.\n"
       "    boolean_merge: Single-operation merge.\n"
       "\n"
       "Wraps: SmApiNonManifoldBoolean");

    m.def("piecewise_merge", [](SmBrep* a, SmBrep* b, bool createNewBreps) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiPiecewiseMerge(a, b, createNewBreps ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("brep1"), py::arg("brep2"), py::arg("create_new_breps_for_faces") = false,
       "Consumes: brep1; returns new Brep. brep2 is referenced (not consumed).\n"
       "\n"
       "Merge faces of one Brep into another, preserving the second.\n"
       "\n"
       "Args:\n"
       "    brep1: Primary body. Modified by the operation.\n"
       "    brep2: Donor body. Not consumed - its faces are referenced (or\n"
       "        copied) into the result.\n"
       "    create_new_breps_for_faces: If ``True``, treat each face of\n"
       "        ``brep2`` as a standalone sheet body (one Brep per face) when\n"
       "        merging. Defaults to ``False``.\n"
       "\n"
       "Returns:\n"
       "    Brep: merged result.\n"
       "\n"
       "See Also:\n"
       "    boolean_merge: Standard merge (consumes both bodies).\n"
       "    non_manifold_boolean: Non-manifold-allowing boolean.\n"
       "\n"
       "Wraps: SmApiPiecewiseMerge");
}
