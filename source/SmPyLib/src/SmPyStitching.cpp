// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiBrep.h (SmApiStitch, SmApiSimpleFaceStitch,
// SmApiStitchIntoSolid, SmApiStitchIntoShell, SmApiStitchAdvanced,
// SmApiUnifyNormals).

#include "SmPyCommon.h"

#include <SmApiBrep.h>

void bind_stitching(BoundModule& m)
{
    m.def("stitch_brep", [](SmBrep* brep) {
              CHECK_STATUS(SmApiStitch(brep));
              return brep;
          }, py::return_value_policy::reference,
          py::arg("brep"),
          "Mutates: brep; returns the same handle for chaining.\n"
          "\n"
          "Stitch every lamina edge of a Brep using default tolerances.\n"
          "\n"
          "Args:\n"
          "    brep: Brep to stitch in place.\n"
          "\n"
          "Notes:\n"
          "    One-call entry point with no tunables and no diagnostic\n"
          "    return values. Prefer ``stitch_into_solid`` or\n"
          "    ``stitch_into_shell`` for programmatic workflows that need\n"
          "    to know whether the result is solid and how big the bridged\n"
          "    gaps were.\n"
          "\n"
          "See Also:\n"
          "    stitch_brep_with_face: Restrict stitching to one face.\n"
          "    stitch_into_solid: Tighter form that reports whether the\n"
          "        result is a manifold solid.\n"
          "    stitch_into_shell: Looser tolerance form for shell results.\n"
          "    stitch_advanced: Full control over stitching tolerances and\n"
          "        cleanup flags.\n"
          "\n"
          "Wraps: SmApiStitch");

    m.def("stitch_brep_with_face", [](SmBrep* brep, SmFace* face) {
        CHECK_STATUS(SmApiStitch(brep, face));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("face"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Stitch a Brep along the lamina edges of a single face.\n"
       "\n"
       "Args:\n"
       "    brep: Brep that owns ``face``. Modified in place.\n"
       "    face: Face whose lamina edges should be matched against the\n"
       "        rest of the Brep.\n"
       "\n"
       "Notes:\n"
       "    Useful when adding one face at a time to a partially built\n"
       "    Brep. For whole-Brep stitching use ``stitch_brep`` or\n"
       "    ``stitch_into_solid``.\n"
       "\n"
       "See Also:\n"
       "    stitch_brep: Whole-Brep version with no diagnostics.\n"
       "    simple_face_stitch: Stitch a list of keep/delete face pairs.\n"
       "\n"
       "Wraps: SmApiStitch (Brep + Face overload)");

    m.def("simple_face_stitch", [](SmBrep* brep, std::vector<SmFace*>& keep, std::vector<SmFace*>& remove) {
        SmTArray<SmFace*> k, r;
        for (auto* f : keep) k.Add(f);
        for (auto* f : remove) r.Add(f);
        ULONG stitched = 0;
        double maxVertGap = 0, maxEdgeGap = 0;
        CHECK_STATUS(SmApiSimpleFaceStitch(brep, k, r, stitched, maxVertGap, maxEdgeGap));
        return py::make_tuple(brep, stitched, maxVertGap, maxEdgeGap);
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("faces_to_keep"), py::arg("faces_to_delete"),
       "Mutates: brep; returns (brep, stitched_edges, max_vertex_gap, max_edge_gap).\n"
       "\n"
       "Stitch matched pairs of faces, keeping one face per pair.\n"
       "\n"
       "Args:\n"
       "    brep: Brep that owns the faces. Modified in place.\n"
       "    faces_to_keep: Faces to retain in the result. Must have the\n"
       "        same length as ``faces_to_delete`` - entry ``i`` of\n"
       "        ``faces_to_delete`` is glued onto entry ``i`` of\n"
       "        ``faces_to_keep``.\n"
       "    faces_to_delete: Faces to remove. Each is geometrically\n"
       "        coincident (within tolerance) with the matching keep face.\n"
       "\n"
       "Returns:\n"
       "    tuple[Brep, int, float, float]: ``(brep, stitched_edges,\n"
       "    max_vertex_gap, max_edge_gap)`` - the same brep handle (for\n"
       "    chaining), the number of edge pairs glued, and the largest\n"
       "    vertex/edge gap that was bridged.\n"
       "\n"
       "Notes:\n"
       "    Use this when you already know which faces should be merged\n"
       "    (for example, after a manual import where duplicate faces\n"
       "    exist).\n"
       "\n"
       "See Also:\n"
       "    stitch_into_solid: Whole-Brep stitch with solid-result check.\n"
       "    stitch_advanced: Tunable whole-Brep stitch.\n"
       "\n"
       "Wraps: SmApiSimpleFaceStitch");

    m.def("stitch_into_solid", [](SmBrep* brep) {
        SmBoolean bSolid = FALSE;
        ULONG stitchedEdges = 0;
        double maxVertGap = 0, maxEdgeGap = 0;
        CHECK_STATUS(SmApiStitchIntoSolid(brep, bSolid, stitchedEdges, maxVertGap, maxEdgeGap));
        return py::make_tuple(brep, (bool)bSolid, stitchedEdges, maxVertGap, maxEdgeGap);
    }, py::return_value_policy::reference,
       py::arg("brep"),
       "Mutates: brep; returns (brep, is_solid, stitched_edges, max_vertex_gap, max_edge_gap).\n"
       "\n"
       "Stitch a Brep aiming for a manifold solid result.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to stitch in place. Typically a collection of loose\n"
       "        faces from an import.\n"
       "\n"
       "Returns:\n"
       "    tuple[Brep, bool, int, float, float]: ``(brep, is_solid,\n"
       "    stitched_edges, max_vertex_gap, max_edge_gap)`` where\n"
       "    ``is_solid`` is ``True`` only if every edge ended up shared by\n"
       "    exactly two faces. The first element is the same brep handle\n"
       "    (for chaining).\n"
       "\n"
       "Notes:\n"
       "    Uses the kernel default tolerance. If ``is_solid`` is ``False``\n"
       "    the gap values tell you how much room the unstitched edges\n"
       "    needed - either widen tolerance with ``stitch_into_shell`` or\n"
       "    diagnose the geometry directly. Common failure causes: gaps\n"
       "    larger than tolerance, overlapping faces, or non-manifold\n"
       "    topology (more than two faces sharing an edge).\n"
       "\n"
       "See Also:\n"
       "    stitch_into_shell: Looser tolerance variant.\n"
       "    stitch_advanced: Per-flag control over the stitch.\n"
       "    is_manifold_solid: Final validation check after stitching.\n"
       "    unify_normals: Make face normals consistent post-stitch.\n"
       "\n"
       "Wraps: SmApiStitchIntoSolid");

    m.def("stitch_into_shell", [](SmBrep* brep, double maxRatio, bool shellIsWellFormed) {
        SmBoolean bWellFormed = shellIsWellFormed ? TRUE : FALSE;
        ULONG stitchedEdges = 0;
        double maxVertGap = 0, maxEdgeGap = 0;
        CHECK_STATUS(SmApiStitchIntoShell(brep, bWellFormed, maxRatio,
                     stitchedEdges, maxVertGap, maxEdgeGap));
        return py::make_tuple(brep, shellIsWellFormed, stitchedEdges, maxVertGap, maxEdgeGap);
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("max_stitching_ratio") = 1.0,
       py::kw_only(), py::arg("shell_is_well_formed") = false,
       "Mutates: brep; returns (brep, shell_is_well_formed, stitched_edges, max_vertex_gap, max_edge_gap).\n"
       "\n"
       "Stitch a Brep aiming for a shell, up to a maximum gap distance.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to stitch in place.\n"
       "    max_stitching_ratio: Largest gap to close, as an absolute\n"
       "        distance in model units (despite the name, not a ratio).\n"
       "        Stitching starts at 1/100 of it and widens in steps.\n"
       "        Defaults to ``1.0``.\n"
       "    shell_is_well_formed: Stitching mode. Pass ``True`` only for a\n"
       "        shell known to be well formed (e.g. from a solid modeler);\n"
       "        the default ``False`` glues edges and vertices and ignores\n"
       "        other topology problems. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    tuple[Brep, bool, int, float, float]: ``(brep,\n"
       "    shell_is_well_formed, stitched_edges, max_vertex_gap,\n"
       "    max_edge_gap)``. The second element echoes the\n"
       "    ``shell_is_well_formed`` argument; it is not a property of the\n"
       "    result (use ``is_manifold_solid`` for that). The first element\n"
       "    is the same brep handle (for chaining).\n"
       "\n"
       "Notes:\n"
       "    Less restrictive than ``stitch_into_solid``: the result need\n"
       "    not be solid. With the default ``shell_is_well_formed=False``\n"
       "    it can still have topology problems. Use this when the input is\n"
       "    expected to be a sheet body or when ``stitch_into_solid``\n"
       "    fails because of slightly larger gaps.\n"
       "\n"
       "See Also:\n"
       "    stitch_into_solid: Tighter, solid-targeting variant.\n"
       "    stitch_advanced: Explicit-tolerance variant.\n"
       "\n"
       "Wraps: SmApiStitchIntoShell");

    m.def("unify_normals", [](SmBrep* brep, SmFace* startFace, unsigned long numSamples) {
        SmTArray<SmFace*> flipped;
        CHECK_STATUS(SmApiUnifyNormals(brep, startFace, numSamples, flipped));
        std::vector<SmFace*> v;
        for (ULONG i = 0; i < flipped.GetSize(); i++) v.push_back(flipped[i]);
        return py::make_tuple(brep, v);
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("start_face"), py::arg("num_samples") = 100,
       "Mutates: brep; returns (brep, flipped_faces) for chaining.\n"
       "\n"
       "Orient all face normals consistently within a Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to orient in place.\n"
       "    start_face: Reference face whose existing normal direction is\n"
       "        kept. Choose a face whose orientation you trust (often a\n"
       "        face you know should point outward).\n"
       "    num_samples: Number of ray samples used for inside/outside\n"
       "        classification. Higher = more reliable but slower.\n"
       "        Defaults to ``100``, which is usually sufficient.\n"
       "\n"
       "Returns:\n"
       "    tuple[Brep, list[Face]]: ``(brep, flipped_faces)`` - the same\n"
       "    brep handle (for chaining), and the faces whose normals were\n"
       "    flipped to match the orientation propagated from ``start_face``.\n"
       "\n"
       "Notes:\n"
       "    Typically run after stitching, where individual faces may have\n"
       "    arrived from import with inconsistent orientation.\n"
       "\n"
       "See Also:\n"
       "    stitch_into_solid: Common predecessor in the import workflow.\n"
       "    is_manifold_solid: Final validation after unifying normals.\n"
       "\n"
       "Wraps: SmApiUnifyNormals");

    m.def("stitch_advanced", [](SmBrep* brep, double tol3d, bool squeezeSmall,
                                bool splitEdges, bool manifoldSolid, bool removeSlivers) {
        ULONG stitchedEdges = 0, laminaEdges = 0;
        double maxVertGap = 0, maxEdgeGap = 0;
        CHECK_STATUS(SmApiStitchAdvanced(brep, tol3d,
                     squeezeSmall ? TRUE : FALSE,
                     splitEdges ? TRUE : FALSE,
                     manifoldSolid ? TRUE : FALSE,
                     removeSlivers ? TRUE : FALSE,
                     stitchedEdges, laminaEdges, maxVertGap, maxEdgeGap));
        return py::make_tuple(brep, stitchedEdges, laminaEdges, maxVertGap, maxEdgeGap);
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("tolerance_3d"),
       py::arg("squeeze_small_edges") = true, py::arg("split_edges_with_vertices") = true,
       py::arg("making_manifold_solid") = true, py::arg("remove_laminar_slivers") = false,
       "Mutates: brep; returns (brep, stitched_edges, lamina_edges, max_vertex_gap, max_edge_gap).\n"
       "\n"
       "Stitch a Brep with explicit tolerance and per-step cleanup flags.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to stitch in place.\n"
       "    tolerance_3d: Explicit 3D distance tolerance for gluing edges.\n"
       "        Must be positive.\n"
       "    squeeze_small_edges: If ``True`` (default), collapse edges\n"
       "        shorter than ``tolerance_3d`` before stitching.\n"
       "    split_edges_with_vertices: If ``True`` (default), split edges\n"
       "        at nearby vertices to improve match quality.\n"
       "    making_manifold_solid: If ``True`` (default), only glue lamina\n"
       "        edge pairs (skip non-manifold candidates).\n"
       "    remove_laminar_slivers: If ``True``, remove sliver faces\n"
       "        before stitching. Defaults to ``False``.\n"
       "\n"
       "Returns:\n"
       "    tuple[Brep, int, int, float, float]: ``(brep, stitched_edges,\n"
       "    lamina_edges_remaining, max_vertex_gap, max_edge_gap)`` - the\n"
       "    first element is the same brep handle (for chaining).\n"
       "\n"
       "Notes:\n"
       "    Use when ``stitch_into_solid`` / ``stitch_into_shell`` are too\n"
       "    blunt - for example to stitch with a known custom tolerance.\n"
       "\n"
       "See Also:\n"
       "    stitch_into_solid: Default-tolerance solid-targeting variant.\n"
       "    stitch_into_shell: Default-tolerance shell-targeting variant.\n"
       "    stitch_brep: Single-call variant with no diagnostics.\n"
       "\n"
       "Wraps: SmApiStitchAdvanced");
}
