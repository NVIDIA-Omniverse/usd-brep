// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiBrep.h (SmApiCreate{Linear,Rotational,Draft,Pipe,Taper,Curve}
// Sweep variants, SmApiCreateSweepAlongPlanarPath, SmApiNonManifold*Sweep).

#include "SmPyCommon.h"

#include <SmApiBrep.h>

// DoSweep selects by pointer nullity, so only-empty sequences sweep nothing yet still succeed.
static void check_nonmanifold_selection(const py::object& faces,
                                        const py::object& edges,
                                        const py::object& vertices)
{
    if (faces.is_none() && edges.is_none() && vertices.is_none())
        return;  // all None: sweep the whole Brep

    auto selects = [](const py::object& o) { return !o.is_none() && py::len(o) > 0; };
    if (!selects(faces) && !selects(edges) && !selects(vertices))
        throw std::invalid_argument(
            "non-manifold sweep has an empty selection: pass None for faces, edges and vertices "
            "to sweep the whole Brep, or give at least one non-empty sequence");

    // A None entry casts to a null handle, which the sweep helpers dereference. SM_API
    // rejects it too, but only after the sequence has been converted, so name the
    // offending argument here while we still can.
    auto reject_none_entries = [](const py::object& o, const char* name) {
        if (o.is_none())
            return;
        for (const auto& item : o)
            if (item.is_none())
                throw std::invalid_argument(std::string("non-manifold sweep selection '") + name +
                                            "' contains None; every entry must be topology of brep_to_sweep");
    };
    reject_none_entries(faces, "faces");
    reject_none_entries(edges, "edges");
    reject_none_entries(vertices, "vertices");
}

// SM_API sweeps into brep_to_sweep's own region and returns that Brep, so a distinct
// region_brep would leave the generated topology in one Brep and the stitched, validated,
// returned result in the other. Reject it here to name the argument; SM_API rejects it too.
static void check_nonmanifold_region(SmBrep* to_sweep, const py::object& region)
{
    if (region.is_none())
        return;
    if (region.cast<SmBrep*>() != to_sweep)
        throw std::invalid_argument(
            "non-manifold sweep region_brep must be None or brep_to_sweep itself: a different "
            "Brep would receive the swept topology while brep_to_sweep is what gets stitched "
            "and returned");
}

void bind_sweeps(BoundModule& m)
{
    m.def("linear_sweep", [](std::vector<SmCurve*>& curves,
                             py::tuple dir, double dist, bool cap) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmVector3d d = to_vec(dir);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateLinearSweep(arr, d, dist, cap ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("direction"), py::arg("distance"),
       py::kw_only(),
       py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Extrude a profile along a straight direction to create a Brep.\n"
       "\n"
       "Args:\n"
       "    curves: Profile curves. For a manifold solid the curves must form\n"
       "        a closed planar loop; open curves produce a sheet body.\n"
       "    direction: Sweep direction as ``(dx, dy, dz)``. Magnitude is\n"
       "        ignored - the actual sweep length is ``distance``.\n"
       "    distance: Sweep length, in modeling units. Must be non-zero.\n"
       "    cap_ends: If ``True`` (default), close both ends with planar cap\n"
       "        faces (manifold solid). ``False`` produces an open shell.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: extruded body.\n"
       "\n"
       "See Also:\n"
       "    rotational_sweep: Revolve a profile about an axis.\n"
       "    draft_sweep: Extrude with a draft angle (taper).\n"
       "    linear_sweep_with_repetitions: Multiple end-to-end linear sweeps.\n"
       "    create_extrude_surface: Surface-only extrusion (no caps).\n"
       "\n"
       "Wraps: SmApiCreateLinearSweep");

    m.def("rotational_sweep", [](std::vector<SmCurve*>& curves,
                                 py::tuple basePt, py::tuple axis,
                                 double angleDeg, bool cap) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmPoint3d bp = to_point(basePt);
        SmVector3d ax = to_vec(axis);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateRotationalSweep(arr, bp, ax, angleDeg, cap ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("base_point"), py::arg("axis"),
       py::arg("angle_deg"),
       py::kw_only(),
       py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Revolve a profile about an axis to create a Brep.\n"
       "\n"
       "Args:\n"
       "    curves: Profile curves. Must lie entirely on one side of the\n"
       "        rotation axis (no crossings) to avoid self-intersection.\n"
       "    base_point: A point on the rotation axis as ``(x, y, z)``.\n"
       "    axis: Direction of the rotation axis as ``(dx, dy, dz)``.\n"
       "    angle_deg: Sweep angle, in degrees. Use ``360`` for a closed\n"
       "        solid of revolution; less than 360 produces a wedge.\n"
       "    cap_ends: If ``True`` (default), close the start/end of partial\n"
       "        sweeps with cap faces; ignored when ``angle_deg == 360``.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: revolved body.\n"
       "\n"
       "See Also:\n"
       "    linear_sweep: Straight extrusion along a direction.\n"
       "    rotational_sweep_with_repetitions: Multiple end-to-end revolutions.\n"
       "    create_surface_revolution: Surface-only revolution (no caps).\n"
       "\n"
       "Wraps: SmApiCreateRotationalSweep");

    m.def("draft_sweep", [](std::vector<SmCurve*>& curves,
                            double height, double angleDeg, bool cornerType,
                            CapEnds capEnds) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateDraftSweep(arr, height, angleDeg,
                     cornerType ? TRUE : FALSE, static_cast<int>(capEnds), r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("height"), py::arg("angle_deg"),
       py::kw_only(),
       py::arg("fillet_corners") = false, py::arg("cap_ends") = CapEnds::BOTH,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Extrude a planar profile with a draft (taper) angle.\n"
       "\n"
       "Args:\n"
       "    curves: Profile curves. Must form a closed planar loop.\n"
       "    height: Extrusion height, in modeling units. Must be non-zero.\n"
       "    angle_deg: Draft angle in degrees. ``0`` produces a straight\n"
       "        extrusion (equivalent to ``linear_sweep``); positive values\n"
       "        taper the walls inward.\n"
       "    fillet_corners: If ``True``, round expanding corners between\n"
       "        adjacent drafted faces (``SM_OC_FILLET_CORNER``). If\n"
       "        ``False`` (default), extend corners linearly until they\n"
       "        intersect (``SM_OC_LINEAR_EXTENSION``). Keyword-only.\n"
       "    cap_ends: ``CapEnds`` selector. ``START`` caps the original\n"
       "        profile end; ``END`` caps the drafted (offset) end; ``BOTH``\n"
       "        (default) caps both; ``NONE`` leaves the result open.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: drafted body.\n"
       "\n"
       "See Also:\n"
       "    taper_extrude: Similar with an explicit inner/outer draft\n"
       "        convention and signed ``height``.\n"
       "    linear_sweep: Zero-draft case.\n"
       "\n"
       "Wraps: SmApiCreateDraftSweep");

    // Subject-first ordering: ``path`` is the geometric subject of the
    // operation; ``radius`` and ``cap_ends`` are configuration and become
    // keyword-only.  The previous ``(radius, path)`` order was the only
    // remaining subject-second function in the modeling API and had no
    // analogue elsewhere (every other sweep takes its profile/path as the
    // first positional argument).
    m.def("pipe_sweep", [](SmCurve* path, double radius, bool cap) {
        SmBSplineCurve* bsp = dynamic_cast<SmBSplineCurve*>(path);
        if (!bsp) throw std::invalid_argument("path must be a B-spline curve");
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePipeSweep(radius, bsp, cap ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("path"), py::kw_only(), py::arg("radius"), py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Sweep a circular cross-section along a path curve to create a pipe.\n"
       "\n"
       "Args:\n"
       "    path: Sweep-path B-spline curve. ``TypeError`` is raised if a\n"
       "        non-B-spline curve is passed.\n"
       "    radius: Pipe radius. Must be positive and smaller than the path\n"
       "        curve's minimum radius of curvature to avoid self-intersection.\n"
       "        Keyword-only.\n"
       "    cap_ends: If ``True`` (default), close the ends of the pipe with\n"
       "        planar cap faces (manifold solid). Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: pipe body.\n"
       "\n"
       "See Also:\n"
       "    curve_sweep: Variable-radius sweep with an arbitrary profile and\n"
       "        optional scale curves.\n"
       "    sweep_along_planar_path: Sweep along a planar polyline.\n"
       "\n"
       "Wraps: SmApiCreatePipeSweep");

    m.def("taper_extrude", [](std::vector<SmCurve*>& curves,
                              double height, double draftAngleDeg,
                              CapEnds endCaps, bool filletCorner) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateTaperExtrude(arr, height, draftAngleDeg,
                     static_cast<int>(endCaps), filletCorner ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("height"), py::arg("draft_angle_deg"),
       py::kw_only(),
       py::arg("cap_ends") = CapEnds::BOTH, py::arg("fillet_corner") = false,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Extrude coplanar curves with a draft angle and signed height.\n"
       "\n"
       "Args:\n"
       "    curves: Coplanar profile curves to extrude.\n"
       "    height: Extrusion height. Sign selects extrusion direction\n"
       "        (positive or negative along the profile normal).\n"
       "    draft_angle_deg: Draft angle in degrees. ``0`` to ``90``\n"
       "        produces an inward (narrowing) taper; ``90`` is a straight\n"
       "        extrusion; ``90`` to ``180`` produces an outward (widening)\n"
       "        taper.\n"
       "    cap_ends: ``CapEnds`` selector. ``START`` caps the original\n"
       "        profile end; ``END`` caps the offset (drafted) end; ``BOTH``\n"
       "        (default) caps both; ``NONE`` leaves the result open.\n"
       "        Keyword-only.\n"
       "    fillet_corner: If ``True``, round corners between drafted faces;\n"
       "        if ``False`` (default), extend corners linearly. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: tapered extrusion body.\n"
       "\n"
       "Notes:\n"
       "    Differs from ``draft_sweep`` in two respects: ``height`` is signed\n"
       "    (no separate direction), and ``draft_angle_deg`` uses the\n"
       "    ``0..90..180`` (inner..straight..outer) convention rather than\n"
       "    a single sign-of-taper angle.\n"
       "\n"
       "See Also:\n"
       "    draft_sweep: Alternative draft-angle convention.\n"
       "    linear_sweep: Zero-draft case.\n"
       "\n"
       "Wraps: SmApiCreateTaperExtrude");

    m.def("sweep_along_planar_path", [](std::vector<SmCurve*>& profileCurves,
                                        std::vector<SmCurve*>& pathCurves,
                                        bool moveProfile, CapEnds capEnds) {
        SmTArray<SmCurve*> prof, path;
        for (auto* c : profileCurves) prof.Add(c);
        for (auto* c : pathCurves) path.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateSweepAlongPlanarPath(prof, path,
                     moveProfile ? TRUE : FALSE, static_cast<int>(capEnds), r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("profile_curves"), py::arg("path_curves"),
       py::kw_only(),
       py::arg("move_profile") = true, py::arg("cap_ends") = CapEnds::BOTH,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Sweep profile boundary curves along a planar path.\n"
       "\n"
       "Args:\n"
       "    profile_curves: Coplanar profile boundary curves. Simple closed\n"
       "        loops define planar regions using even/odd nesting; open\n"
       "        profile wires sweep to sheet geometry.\n"
       "    path_curves: Ordered, connected, coplanar path curves forming an\n"
       "        open chain or closed loop.\n"
       "    move_profile: If ``True`` (default), anchor the profile bounding-\n"
       "        box center to the path start. If ``False``, anchor the first\n"
       "        point of the first profile curve there instead. Keyword-only.\n"
       "    cap_ends: ``CapEnds`` selector. ``START`` caps the profile end;\n"
       "        ``END`` caps the far (path) end; ``BOTH`` (default) caps both;\n"
       "        ``NONE`` leaves the result open. Closed paths have no\n"
       "        endpoints, so the selector has no effect. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: Swept sheet, open shell, or nested solid according to the\n"
       "        profile topology, path closure, and requested endpoint caps.\n"
       "\n"
       "Notes:\n"
       "    For an open path, ``BOTH`` caps plus closed profile loops request\n"
       "    a nested solid. ``NONE``, ``START``, or ``END`` leave a sheet or\n"
       "    open shell. A closed path has no endpoint caps and requests a\n"
       "    nested solid when its swept boundary closes. Nested bounded\n"
       "    regions alternate between material and void from the outside in.\n"
       "    A manifold boundary with no material region is rejected.\n"
       "    Tangent-discontinuous path joins use sharp, mitered transitions;\n"
       "    rounded corners are not inserted. Author the rounded motion as\n"
       "    tangent-connected arc or spline path curves when needed.\n"
       "\n"
       "See Also:\n"
       "    pipe_sweep: Specialized circular cross-section along a path.\n"
       "    curve_sweep: Sweep with optional scale curves.\n"
       "\n"
       "Wraps: SmApiCreateSweepAlongPlanarPath");

    m.def("linear_sweep_with_repetitions", [](std::vector<SmCurve*>& curves,
                                              py::tuple dir, double dist,
                                              unsigned long reps, bool cap) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmVector3d d = to_vec(dir);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateLinearSweepWithRepetitions(arr, d, dist, reps,
                     cap ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("direction"), py::arg("distance"),
       py::kw_only(),
       py::arg("repetitions"), py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Linear sweep repeated end-to-end ``repetitions`` times.\n"
       "\n"
       "Args:\n"
       "    curves: Profile curves (closed loop for a manifold result).\n"
       "    direction: Sweep direction as ``(dx, dy, dz)``. Magnitude is\n"
       "        ignored.\n"
       "    distance: Length of each individual sweep step. Must be non-zero.\n"
       "    repetitions: Number of end-to-end sweeps. Must be at least 1.\n"
       "        Keyword-only.\n"
       "    cap_ends: If ``True`` (default), cap the start of the first\n"
       "        repetition and the end of the last repetition. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: extruded body of total length ``distance * repetitions``.\n"
       "\n"
       "See Also:\n"
       "    linear_sweep: Single-step linear extrusion.\n"
       "    rotational_sweep_with_repetitions: Repeated rotational sweeps.\n"
       "\n"
       "Wraps: SmApiCreateLinearSweepWithRepetitions");

    m.def("rotational_sweep_with_repetitions", [](std::vector<SmCurve*>& curves,
                                                  py::tuple basePt, py::tuple axis,
                                                  double angleDeg, unsigned long reps,
                                                  bool cap) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmPoint3d bp = to_point(basePt);
        SmVector3d ax = to_vec(axis);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateRotationalSweepWithRepetitions(arr, bp, ax, angleDeg, reps,
                     cap ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"), py::arg("base_point"), py::arg("axis"),
       py::arg("angle_deg"),
       py::kw_only(),
       py::arg("repetitions"), py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Rotational sweep repeated end-to-end ``repetitions`` times.\n"
       "\n"
       "Args:\n"
       "    curves: Profile curves (must lie on one side of the axis).\n"
       "    base_point: A point on the rotation axis as ``(x, y, z)``.\n"
       "    axis: Direction of the rotation axis as ``(dx, dy, dz)``.\n"
       "    angle_deg: Sweep angle for each individual revolution, in\n"
       "        degrees.\n"
       "    repetitions: Number of end-to-end revolutions. Must be at least 1.\n"
       "        Keyword-only.\n"
       "    cap_ends: If ``True`` (default), cap the start of the first\n"
       "        repetition and the end of the last repetition (when the total\n"
       "        sweep does not close into a full revolution). Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: revolved body covering ``angle_deg * repetitions`` total.\n"
       "\n"
       "See Also:\n"
       "    rotational_sweep: Single-step revolution.\n"
       "    linear_sweep_with_repetitions: Repeated linear sweeps.\n"
       "\n"
       "Wraps: SmApiCreateRotationalSweepWithRepetitions");

    m.def("curve_sweep", [](std::vector<SmCurve*>& profileCurves,
                            SmBSplineCurve* pathCurve,
                            SmBSplineCurve* scaleRef,
                            SmBSplineCurve* scaleCurve,
                            bool capEnds) {
        SmTArray<SmCurve*> prof;
        for (auto* c : profileCurves) prof.Add(c);
        SmSweepOptions options;
        options.bCapEndsArg = capEnds ? TRUE : FALSE;
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateCurveSweep(prof, pathCurve, scaleRef, scaleCurve,
                     &options, r, nullptr, nullptr, nullptr));
        return r;
    }, py::return_value_policy::reference,
       py::arg("profile_curves"), py::arg("path_curve"),
       py::arg("scale_reference") = nullptr, py::arg("scale_curve") = nullptr,
       py::kw_only(),
       py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Sweep a profile along a path curve, with optional variable scaling.\n"
       "\n"
       "Args:\n"
       "    profile_curves: Profile curves to sweep.\n"
       "    path_curve: B-spline path curve.\n"
       "    scale_reference: Optional B-spline reference curve defining the\n"
       "        baseline scale at each point along the path. ``None`` for no\n"
       "        scaling.\n"
       "    scale_curve: Optional B-spline curve whose distance from\n"
       "        ``scale_reference`` defines the scale factor at each point.\n"
       "        ``None`` for no scaling. Must be supplied together with\n"
       "        ``scale_reference``.\n"
       "    cap_ends: If True (default), cap both ends so a closed profile\n"
       "        produces a manifold solid. If False, return the open swept\n"
       "        shell. Capping requires a closed profile.\n"
       "\n"
       "Returns:\n"
       "    Brep: swept body, with cross-sections scaled per the optional\n"
       "    scale curves.\n"
       "\n"
       "See Also:\n"
       "    pipe_sweep: Constant-radius circular cross-section along a path.\n"
       "    curve_sweep_from_faces: Same operation taking face edge loops.\n"
       "    curve_sweep_from_edges: Same operation taking edges.\n"
       "\n"
       "Wraps: SmApiCreateCurveSweep");

    m.def("curve_sweep_from_faces", [](std::vector<SmFace*>& faces, SmBSplineCurve* path,
                                       SmBSplineCurve* scale_ref, SmBSplineCurve* scale_curve,
                                       bool capEnds) {
        SmTArray<SmFace*> arr;
        for (auto* f : faces) arr.Add(f);
        SmSweepOptions options;
        options.bCapEndsArg = capEnds ? TRUE : FALSE;
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateCurveSweepFromFaces(arr, path, scale_ref, scale_curve, &options, r,
                                                    nullptr, nullptr, nullptr));
        return r;
    }, py::return_value_policy::reference,
       py::arg("faces"), py::arg("path_curve"),
       py::arg("scale_reference") = nullptr, py::arg("scale_curve") = nullptr,
       py::kw_only(),
       py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Sweep face edge loops along a path curve, with optional scaling.\n"
       "\n"
       "Args:\n"
       "    faces: Faces whose outer-edge loops define the swept profiles.\n"
       "    path_curve: B-spline path curve.\n"
       "    scale_reference: Optional B-spline scale reference; see\n"
       "        ``curve_sweep`` for details. ``None`` for no scaling.\n"
       "    scale_curve: Optional B-spline scale curve. Must be supplied\n"
       "        together with ``scale_reference``.\n"
       "    cap_ends: If True (default), cap both ends so a closed profile\n"
       "        produces a manifold solid. If False, return the open swept\n"
       "        shell.\n"
       "\n"
       "Returns:\n"
       "    Brep: swept body.\n"
       "\n"
       "See Also:\n"
       "    curve_sweep: Same operation taking standalone profile curves.\n"
       "    curve_sweep_from_edges: Same operation taking edges.\n"
       "\n"
       "Wraps: SmApiCreateCurveSweepFromFaces");

    m.def("curve_sweep_from_edges", [](std::vector<SmEdge*>& edges, SmBSplineCurve* path,
                                       SmBSplineCurve* scale_ref, SmBSplineCurve* scale_curve,
                                       bool capEnds) {
        SmTArray<SmEdge*> arr;
        for (auto* e : edges) arr.Add(e);
        SmSweepOptions options;
        options.bCapEndsArg = capEnds ? TRUE : FALSE;
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateCurveSweepFromEdges(arr, path, scale_ref, scale_curve, &options, r,
                                                    nullptr, nullptr, nullptr));
        return r;
    }, py::return_value_policy::reference,
       py::arg("edges"), py::arg("path_curve"),
       py::arg("scale_reference") = nullptr, py::arg("scale_curve") = nullptr,
       py::kw_only(),
       py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "\n"
       "Sweep edges along a path curve, with optional scaling.\n"
       "\n"
       "Args:\n"
       "    edges: Edges whose curves define the swept profiles.\n"
       "    path_curve: B-spline path curve.\n"
       "    scale_reference: Optional B-spline scale reference; see\n"
       "        ``curve_sweep`` for details. ``None`` for no scaling.\n"
       "    scale_curve: Optional B-spline scale curve. Must be supplied\n"
       "        together with ``scale_reference``.\n"
       "    cap_ends: If True (default), cap both ends so a closed profile\n"
       "        produces a manifold solid. If False, return the open swept\n"
       "        shell.\n"
       "\n"
       "Returns:\n"
       "    Brep: swept body.\n"
       "\n"
       "See Also:\n"
       "    curve_sweep: Same operation taking standalone profile curves.\n"
       "    curve_sweep_from_faces: Same operation taking face edge loops.\n"
       "\n"
       "Wraps: SmApiCreateCurveSweepFromEdges");

    // Lambda parameter order rearranged so the three required args come
    // first (``brep_to_sweep``, ``direction``, ``distance``) and every
    // optional / defaulted arg lives in the keyword-only block - including
    // ``region_brep``, which previously sat as a defaulted positional
    // between the subject and the required ``direction``/``distance``
    // (forcing callers to pass an explicit ``None`` to skip it).  The C
    // ``SmApiNonManifoldSweep`` call still receives args in the kernel's
    // original ``(brep, region, dir, dist, ...)`` order.
    m.def("non_manifold_linear_sweep", [](SmBrep* to_sweep, py::tuple dir, double distance,
                                          py::object region, bool stitch, bool merge,
                                          py::object opt_faces, py::object opt_edges, py::object opt_vertices) {
        check_nonmanifold_selection(opt_faces, opt_edges, opt_vertices);
        check_nonmanifold_region(to_sweep, region);
        SmVector3d d = to_vec(dir);
        SmTArray<SmFace*>* pf = nullptr;
        SmTArray<SmFace*> fstore;
        SmTArray<SmEdge*>* pe = nullptr;
        SmTArray<SmEdge*> estore;
        SmTArray<SmVertex*>* pv = nullptr;
        SmTArray<SmVertex*> vstore;
        if (!opt_faces.is_none()) {
            std::vector<SmFace*> vf = opt_faces.cast<std::vector<SmFace*>>();
            for (auto* x : vf) fstore.Add(x);
            pf = &fstore;
        }
        if (!opt_edges.is_none()) {
            std::vector<SmEdge*> ve = opt_edges.cast<std::vector<SmEdge*>>();
            for (auto* x : ve) estore.Add(x);
            pe = &estore;
        }
        if (!opt_vertices.is_none()) {
            std::vector<SmVertex*> vv = opt_vertices.cast<std::vector<SmVertex*>>();
            for (auto* x : vv) vstore.Add(x);
            pv = &vstore;
        }
        SmBrep* region_ptr = region.is_none() ? nullptr : region.cast<SmBrep*>();
        CHECK_STATUS(SmApiNonManifoldSweep(to_sweep, region_ptr, d, distance, stitch ? TRUE : FALSE,
                                           merge ? TRUE : FALSE, pf, pe, pv));
        return to_sweep;
    }, py::return_value_policy::reference,
       py::arg("brep_to_sweep"), py::arg("direction"), py::arg("distance"),
       py::kw_only(),
       py::arg("region_brep") = py::none(),
       py::arg("do_stitching") = true, py::arg("do_merge") = false,
       py::arg("faces") = py::none(), py::arg("edges") = py::none(), py::arg("vertices") = py::none(),
       "Mutates: brep_to_sweep; returns the same handle for chaining.\n"
       "\n"
       "Linearly sweep selected topology of a Brep, with stitching and merging.\n"
       "\n"
       "Args:\n"
       "    brep_to_sweep: Brep containing the topology to sweep. Modified\n"
       "        in place.\n"
       "    direction: Sweep direction as ``(dx, dy, dz)``. Must be non-zero;\n"
       "        does not need to be unit-length.\n"
       "    distance: Sweep distance, in modeling units. Must be non-zero.\n"
       "    region_brep: Optional Brep that defines the target region. Must be\n"
       "        ``None`` (default) or ``brep_to_sweep`` itself; any other Brep\n"
       "        raises ``ValueError``, because only ``brep_to_sweep`` is\n"
       "        stitched, validated and returned. Keyword-only.\n"
       "    do_stitching: If ``True`` (default), stitch the swept geometry\n"
       "        back into ``brep_to_sweep``. Keyword-only.\n"
       "    do_merge: If ``True``, run a merge pass that catches\n"
       "        self-intersections. Defaults to ``False``. Keyword-only.\n"
       "    faces: Optional list of faces to sweep into solids. Keyword-only.\n"
       "    edges: Optional list of edges to sweep into faces. Keyword-only.\n"
       "    vertices: Optional list of vertices to sweep into edges.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Brep: the same ``brep_to_sweep`` handle, for chaining.\n"
       "\n"
       "Notes:\n"
       "    Used to grow specific topology (a face, an edge, a vertex) of an\n"
       "    existing Brep into higher-dimensional features without rebuilding\n"
       "    the whole body.\n"
       "\n"
       "    All three ``None`` sweeps the whole ``brep_to_sweep``; naming any of\n"
       "    them sweeps only what is listed, so ``faces=[], edges=[], vertices=[v]``\n"
       "    sweeps just ``v``. Only-empty sequences select nothing and raise\n"
       "    ``ValueError``.\n"
       "\n"
       "See Also:\n"
       "    non_manifold_rotational_sweep: Rotational variant.\n"
       "    linear_sweep: Standard linear sweep that returns a new Brep.\n"
       "\n"
       "Wraps: SmApiNonManifoldSweep");

    // Lambda parameter order rearranged so the four required args come
    // first (``brep_to_sweep``, ``base_point``, ``axis``, ``angle_deg``)
    // and every optional / defaulted arg lives in the keyword-only block -
    // including ``region_brep``, which previously sat as a defaulted
    // positional between the subject and the required pose args (forcing
    // callers to pass an explicit ``None`` to skip it).  The C
    // ``SmApiNonManifoldRotationalSweep`` call still receives args in the
    // kernel's original ``(brep, region, base, axis, angle, ...)`` order.
    m.def("non_manifold_rotational_sweep",
          [](SmBrep* to_sweep, py::tuple base_pt, py::tuple axis, double angle_deg,
             py::object region, bool stitch, bool merge,
             py::object opt_faces, py::object opt_edges, py::object opt_vertices) {
              check_nonmanifold_selection(opt_faces, opt_edges, opt_vertices);
              check_nonmanifold_region(to_sweep, region);
              SmPoint3d bp = to_point(base_pt);
              SmVector3d ax = to_vec(axis);
              SmTArray<SmFace*>* pf = nullptr;
              SmTArray<SmFace*> fstore;
              SmTArray<SmEdge*>* pe = nullptr;
              SmTArray<SmEdge*> estore;
              SmTArray<SmVertex*>* pv = nullptr;
              SmTArray<SmVertex*> vstore;
              if (!opt_faces.is_none()) {
                  std::vector<SmFace*> vf = opt_faces.cast<std::vector<SmFace*>>();
                  for (auto* x : vf) fstore.Add(x);
                  pf = &fstore;
              }
              if (!opt_edges.is_none()) {
                  std::vector<SmEdge*> ve = opt_edges.cast<std::vector<SmEdge*>>();
                  for (auto* x : ve) estore.Add(x);
                  pe = &estore;
              }
              if (!opt_vertices.is_none()) {
                  std::vector<SmVertex*> vv = opt_vertices.cast<std::vector<SmVertex*>>();
                  for (auto* x : vv) vstore.Add(x);
                  pv = &vstore;
              }
              SmBrep* region_ptr = region.is_none() ? nullptr : region.cast<SmBrep*>();
              CHECK_STATUS(SmApiNonManifoldRotationalSweep(to_sweep, region_ptr, bp, ax, angle_deg,
                                                           stitch ? TRUE : FALSE, merge ? TRUE : FALSE,
                                                           pf, pe, pv));
              return to_sweep;
          }, py::return_value_policy::reference,
          py::arg("brep_to_sweep"), py::arg("base_point"), py::arg("axis"), py::arg("angle_deg"),
          py::kw_only(),
          py::arg("region_brep") = py::none(),
          py::arg("do_stitching") = true, py::arg("do_merge") = false,
          py::arg("faces") = py::none(), py::arg("edges") = py::none(), py::arg("vertices") = py::none(),
          "Mutates: brep_to_sweep; returns the same handle for chaining.\n"
          "\n"
          "Rotationally sweep selected topology of a Brep, with stitching and merging.\n"
          "\n"
          "Args:\n"
          "    brep_to_sweep: Brep containing the topology to sweep. Modified\n"
          "        in place.\n"
          "    base_point: A point on the rotation axis as ``(x, y, z)``.\n"
          "    axis: Direction of the rotation axis as ``(dx, dy, dz)``.\n"
          "    angle_deg: Sweep angle, in degrees.\n"
          "    region_brep: Optional Brep that defines the target region. Must be\n"
          "        ``None`` (default) or ``brep_to_sweep`` itself; any other Brep\n"
          "        raises ``ValueError``, because only ``brep_to_sweep`` is\n"
          "        stitched, validated and returned. Keyword-only.\n"
          "    do_stitching: If ``True`` (default), stitch the swept geometry\n"
          "        back into ``brep_to_sweep``. Keyword-only.\n"
          "    do_merge: If ``True``, run a merge pass that catches\n"
          "        self-intersections. Defaults to ``False``. Keyword-only.\n"
          "    faces: Optional list of faces to sweep. Keyword-only.\n"
          "    edges: Optional list of edges to sweep. Keyword-only.\n"
          "    vertices: Optional list of vertices to sweep. Keyword-only.\n"
          "\n"
          "Returns:\n"
          "    Brep: the same ``brep_to_sweep`` handle, for chaining.\n"
          "\n"
          "Notes:\n"
          "    All three ``None`` sweeps the whole ``brep_to_sweep``; naming any of\n"
          "    them sweeps only what is listed, so ``faces=[], edges=[], vertices=[v]``\n"
          "    sweeps just ``v``. Only-empty sequences select nothing and raise\n"
          "    ``ValueError``.\n"
          "\n"
          "See Also:\n"
          "    non_manifold_linear_sweep: Linear variant.\n"
          "    rotational_sweep: Standard rotational sweep that returns a\n"
          "        new Brep.\n"
          "\n"
          "Wraps: SmApiNonManifoldRotationalSweep");
}
