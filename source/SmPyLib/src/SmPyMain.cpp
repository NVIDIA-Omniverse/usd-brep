// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiGeneral.h (SmApiCreateContext). No function bindings here;
// dispatches to the bind_*() binders declared in SmPyCommon.h.
//
// _omni_solid Python module spine.
// Owns the module declaration, kernel context creation, and the operator-new
// shims pybind11 needs at link time.  All actual bindings are split across
// the SmPy*.cpp files registered through the bind_*() forward declarations
// in SmPyCommon.h.

#include "SmPyCommon.h"

#include <SmApiGeneral.h>

// On Linux/Clang, pybind11 references operator new(size_t) for registered
// types; the SMLib kernel may not export it from the shared library.  Provide
// shims here.  Mark them used + default visibility so --gc-sections /
// -fvisibility=hidden builds do not drop or hide the symbols (fixes undefined
// symbol at dlopen).
#if defined(__GNUC__) && !defined(_WIN32)
#define SMPYLIB_OPNEW_ATTR __attribute__((used, visibility("default")))
#else
#define SMPYLIB_OPNEW_ATTR
#endif

// SMLib kernel types have custom operator new(size_t, SmContext&) but the
// parameterless overload is not exported.  pybind11's type_caster template
// instantiation references it even though it's never called at runtime.
// Provide dead-code shims for Brep / PolyBrep / poly topology types exposed
// to Python so the linker is satisfied.
SMPYLIB_OPNEW_ATTR void* SmBrep::operator new(size_t sz)        { return smos_Malloc(sz); }
SMPYLIB_OPNEW_ATTR void* SmPolyBrep::operator new(size_t sz)    { return smos_Malloc(sz); }
SMPYLIB_OPNEW_ATTR void* SmPolyVertex::operator new(size_t sz)  { return smos_Malloc(sz); }
SMPYLIB_OPNEW_ATTR void* SmPolyEdge::operator new(size_t sz)    { return smos_Malloc(sz); }
SMPYLIB_OPNEW_ATTR void* SmPolyFace::operator new(size_t sz)    { return smos_Malloc(sz); }

PYBIND11_MODULE(_omni_solid, m)
{
    m.doc() =
        "Python bindings for SMLib solid modeling (SM_API + SM_API_USD).\n"
        "\n"
        "Conventions\n"
        "-----------\n"
        "- Angles: degrees, unless an argument name ends in ``_rad``.\n"
        "- Coordinate frame: Z is the up axis. Cylinders, cones, and partial-*\n"
        "  sectors are oriented along Z; angle sweeps are measured in the XY\n"
        "  plane from the X axis. Distances are in modeling units.\n"
        "- Errors: kernel-status failures routed through ``CHECK_STATUS`` raise\n"
        "  ``RuntimeError`` with the wrapped entry and status. Captured callback\n"
        "  events follow in emission order as a kernel trace; when absent, the\n"
        "  message reports the binding call site. The trace does not designate\n"
        "  a root cause. See ``.agents/docs/errors.md`` for both message forms\n"
        "  and per-code guidance.\n"
        "- Object lifetime: context association is non-owning. Python garbage\n"
        "  collection does not reclaim these kernel objects; consuming or\n"
        "  mutating operations can invalidate handles and borrowed geometry.\n"
        "  Follow each operation's contract. ``Face.surface()`` and ``Edge.curve()``\n"
        "  are borrowed views of Brep-owned geometry. A representation-changing\n"
        "  ``turn_to_nurbs`` or scale is rejected before mutation while an\n"
        "  affected borrowed view remains live. Release it before the operation\n"
        "  and reacquire it from the mutated Brep afterward.\n"
        "- Mutation: common Brep / Object / Surface / Curve docstring contracts\n"
        "  are listed below. Check each operation for exceptions and ownership:\n"
        "\n"
        "    Mutates: <arg>; returns the same handle for chaining.\n"
        "      The named arg is modified in place; the call returns that same\n"
        "      handle so chaining works (``box = sm.cut(sm.translate(b, v),\n"
        "      pt, n)``). Examples: ``translate``, ``rotate``, ``scale``,\n"
        "      ``cut``, ``project_and_trim``, every ``fillet_*`` /\n"
        "      ``chamfer_*``, ``stitch_brep``, ``heal_brep``, ``unify_normals``.\n"
        "\n"
        "    Mutates: <arg>; returns (handle, ...diagnostics) for chaining.\n"
        "      Same as above plus a tuple of report values. The handle comes\n"
        "      *first* so unpacking is forward-compatible. Examples:\n"
        "      ``stitch_into_solid``, ``stitch_into_shell``,\n"
        "      ``stitch_advanced``, ``simple_face_stitch``, ``unify_normals``.\n"
        "\n"
        "    Consumes: <args>; returns new Brep.\n"
        "      Ownership of the named args transfers to the operation; the\n"
        "      result may reuse a modified input. Use only the result after the\n"
        "      call. Examples: ``boolean_union``, ``boolean_difference``,\n"
        "      ``merge_breps``.\n"
        "\n"
        "    Pure: returns new <type>; inputs unchanged.\n"
        "      Read-only producer; safe to call repeatedly with the same\n"
        "      inputs. Examples: ``create_box``, ``create_sphere``,\n"
        "      ``linear_sweep``, ``offset_brep``, ``shell_brep``, ``fillet_preview``,\n"
        "      ``project_brep_onto_plane``, ``project_curve``, ``tessellate``,\n"
        "      ``tessellate_boundaries``, ``tessellation_defaults``.\n"
        "\n"
        "  Names alone do not determine mutation: non-manifold sweeps mutate\n"
        "  their source, and ``create_blend_primitive`` inserts into a Brep.\n"
        "\n"
        "  Functions that take only primitive arguments (e.g. ``create_box``)\n"
        "  do not carry an explicit ``Pure:`` tag - the contract is implied.\n"
        "\n"
        "API by category\n"
        "---------------\n"
        "Every function listed below is reachable two ways: as a flat top-level\n"
        "name (``sm.create_box``) and from its category submodule\n"
        "(``sm.primitives.create_box``).  The submodules exist purely for\n"
        "discoverability -- ``dir(sm.primitives)`` and ``help(sm.sweeps)`` give\n"
        "a focused view; tab-completion narrows by category.\n"
        "\n"
        ".. code-block:: text\n"
        "\n"
        "  primitives  - box, sphere, cone, cylinder, torus, pyramid, plane,\n"
        "                partial-* sectors, swung / skin / blend primitives\n"
        "  curves      - line, segment, arc, circle, ellipse, rectangle,\n"
        "                regular polygon, B-spline interp/approx, evaluators,\n"
        "                offset, project / drop / lift, remove_curve_knots\n"
        "  surfaces    - extrude / revolve / sweep / ruled / skin / offset\n"
        "                surfaces, planar faces from curves\n"
        "  booleans    - union, difference, intersection, merge (Brep);\n"
        "                2D booleans; non-manifold and curve-driven variants\n"
        "  sweeps      - linear / rotational / pipe / draft / taper / curve\n"
        "                sweeps, sweep-with-repetitions, non-manifold sweeps\n"
        "  fillets     - circular / chamfer / variable-radius / per-edge\n"
        "                fillets, surface-surface fillet, preview, remove\n"
        "  offset      - shell, offset Brep, offset profile (curve set)\n"
        "  io          - native SMLib single-Brep and part-file loading\n"
        "  brep_ops    - turn_to_nurbs\n"
        "  stitching   - face / shell / solid stitching, normal unification\n"
        "  intersect   - curve / surface / face / Brep intersections, plane cut\n"
        "  transforms  - translate / rotate / scale / placement transforms\n"
        "                for Breps and SmObjects\n"
        "  queries     - closest-point queries, manifold / volume checks,\n"
        "                find_face / find_edge, surface point/normal/derivs\n"
        "  cutting     - cut, project, trim (project_curve, silhouette,\n"
        "                trim_surface_with_3d_curves) -- named ``cutting``\n"
        "                so the flat ``sm.cut`` function stays usable\n"
        "  heal        - heal_brep\n"
        "  poly        - PolyBrep boolean ops, manifold / volume checks\n"
        "  usd         - USD mesh and Brep import / export\n"
        "\n"
        "Top-level (no submodule): the entry points ``tessellate``,\n"
        "``tessellate_boundaries`` and ``tessellation_defaults``, the\n"
        "geometry / topology types (``Brep``, ``PolyBrep``, ``Face``, ``Edge``,\n"
        "``Vertex``, ``Curve``, ``BSplineCurve``, ``Line``, ``Circle``,\n"
        "``Surface``, ``Vec3``, ``Vec2``,\n"
        "``Axis2Placement``,\n"
        "``PolyVertex``, ``PolyEdge``, ``PolyFace``), and the enums\n"
        "(``BooleanOp``, ``BooleanOp2D``, ``PolyBooleanOp``, ``FilletXSect``,\n"
        "``TrimType``, ``ContinuityType``, ``BSplineCurveForm``).\n"
        "\n"
        "Errors: see ``.agents/docs/errors.md`` for symbolic status codes and\n"
        "common causes.";

    SmApiCreateContext();

    // Install the kernel error callback so SER()-macro file:line + message
    // gets captured into the per-thread trail consumed by CHECK_STATUS /
    // SmPyRaise.  Daisy-chains over any callback that was already installed.
    install_sm_error_callback();

    // Class / enum binders go straight to the top level: classes and enums
    // must live in exactly one parent module.  Order matters: these run
    // first because every function-only binder below references their types
    // and enums in argument / return signatures.
    bind_enums(m);
    bind_types(m);
    bind_polybrep(m);

    // Per-category submodules.  Each function-only binder receives a
    // BoundModule that registers every binding both on the flat module and
    // on its category submodule (see SmPyCommon.h).
    auto sub_primitives = m.def_submodule("primitives",
        "Solid primitives: box, sphere, cone, cylinder, torus, plane, pyramid, "
        "partial-* sectors, swung / skin / blend primitives.");
    auto sub_curves = m.def_submodule("curves",
        "Curve creation, evaluation, offset, projection, knot removal.");
    auto sub_surfaces = m.def_submodule("surfaces",
        "Surface creation: extrude, revolve, sweep, ruled, skin, offset, planar.");
    auto sub_booleans = m.def_submodule("booleans",
        "Boolean operations on Breps: union, difference, intersection, merge, "
        "2D booleans, non-manifold and curve-driven variants.");
    auto sub_sweeps = m.def_submodule("sweeps",
        "Sweep operations: linear, rotational, pipe, draft, taper, curve, "
        "sweep-with-repetitions, non-manifold variants.");
    auto sub_fillets = m.def_submodule("fillets",
        "Edge fillets and chamfers: circular, variable-radius, per-edge, "
        "surface-surface fillet, preview, remove.");
    auto sub_offset = m.def_submodule("offset",
        "Offset operations: shell, offset Brep, offset profile.");
    auto sub_io = m.def_submodule("io",
        "Native SMLib file I/O helpers.");
    auto sub_brep_ops = m.def_submodule("brep_ops",
        "Miscellaneous Brep operations: NURBS conversion.");
    auto sub_stitching = m.def_submodule("stitching",
        "Stitching: face / shell / solid stitching, normal unification.");
    auto sub_intersect = m.def_submodule("intersect",
        "Geometric intersections: curve, surface, face, Brep, plane cut.");
    auto sub_transforms = m.def_submodule("transforms",
        "Translate, rotate, scale, placement transforms (Brep and SmObject).");
    auto sub_queries = m.def_submodule("queries",
        "Closest-point queries, manifold / volume checks, topology lookups, "
        "surface point / normal / derivative evaluation.");
    // Submodule named "cutting" rather than "cut" to avoid a name clash with
    // the flat ``sm.cut(brep, plane_pt, plane_norm)`` function.
    auto sub_cut = m.def_submodule("cutting",
        "Cut, project, trim operations (cut, silhouette, project_curve, "
        "trim_surface_with_3d_curves).");
    auto sub_heal = m.def_submodule("heal",
        "BRep healing.");
    auto sub_poly = m.def_submodule("poly",
        "PolyBrep (mesh) operations: boolean ops, manifold / volume checks.");

    BoundModule bm_primitives { m, sub_primitives };
    BoundModule bm_curves     { m, sub_curves };
    BoundModule bm_surfaces   { m, sub_surfaces };
    BoundModule bm_booleans   { m, sub_booleans };
    BoundModule bm_sweeps     { m, sub_sweeps };
    BoundModule bm_fillets    { m, sub_fillets };
    BoundModule bm_offset     { m, sub_offset };
    BoundModule bm_io         { m, sub_io };
    BoundModule bm_brep_ops   { m, sub_brep_ops };
    BoundModule bm_stitching  { m, sub_stitching };
    BoundModule bm_intersect  { m, sub_intersect };
    BoundModule bm_transforms { m, sub_transforms };
    BoundModule bm_queries    { m, sub_queries };
    BoundModule bm_cut        { m, sub_cut };
    BoundModule bm_heal       { m, sub_heal };
    BoundModule bm_poly       { m, sub_poly };

    // Geometry creation.
    bind_primitives(bm_primitives);
    bind_curves(bm_curves);
    bind_surfaces(bm_surfaces);

    // Brep editing.
    bind_booleans(bm_booleans);
    bind_sweeps(bm_sweeps);
    bind_fillets(bm_fillets);
    bind_offset(bm_offset);
    bind_file_io(bm_io);
    bind_brep_ops(bm_brep_ops);
    bind_stitching(bm_stitching);

    // Analysis / queries.
    bind_intersectors(bm_intersect);
    bind_transforms(bm_transforms);
    bind_queries(bm_queries);
    bind_cut_project_trim(bm_cut);
    bind_heal(bm_heal);

    // Mesh path.  Tessellation stays flat-only -- ``sm.tessellate`` and
    // ``sm.tessellate_boundaries`` without a same-named submodule to collide with.
    bind_poly_ops(bm_poly);
    bind_tessellation(m);

    // USD: managed as its own submodule entirely inside bind_usd.
    bind_usd(m);
}
