// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiCurves.h (curve create/evaluate/offset/project/lift/drop/
// remove-knots/make-compatible/order).

#include "SmPyCommon.h"

#include <SmApiCurves.h>
#include <SmEllipse.h>

void bind_curves(BoundModule& m)
{
    m.def("create_line_segment", [](py::tuple start, py::tuple end) {
        SmPoint3d s = to_point(start), e = to_point(end);
        SmLine* r = nullptr;
        CHECK_STATUS(SmApiCreateLineSegment(s, e, r));
        return (SmCurve*)r;
    }, py::return_value_policy::reference,
       py::arg("start"), py::arg("end"),
       "Create a straight line segment between two points.\n"
       "\n"
       "Args:\n"
       "    start: Start point of the segment as ``(x, y, z)``.\n"
       "    end: End point of the segment as ``(x, y, z)``. Must be distinct\n"
       "        from ``start``.\n"
       "\n"
       "Returns:\n"
       "    Curve: analytical line segment.\n"
       "\n"
       "See Also:\n"
       "    create_line: Infinite line through a point in a direction.\n"
       "\n"
       "Wraps: SmApiCreateLineSegment");

    m.def("create_circle", [](py::tuple center, double radius) {
        SmPoint3d c = to_point(center);
        SmCircle* r = nullptr;
        CHECK_STATUS(SmApiCreateCircle(c, radius, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("center"), py::arg("radius"),
       "Create an analytic circle in the XY plane.\n"
       "\n"
       "Args:\n"
       "    center: Center of the circle as ``(x, y, z)``. Each coordinate must\n"
       "        be a valid double in ``[-1e18, 1e18]``. The circle lies in the\n"
       "        plane Z = ``center.z``.\n"
       "    radius: Radius of the circle. Must be a valid double in\n"
       "        ``[-1e18, 1e18]`` and greater than ``SM_EFF_ZERO`` (1e-12).\n"
       "        An out-of-range ``center``/``radius``, a ``radius`` at or below\n"
       "        1e-12, or a center whose magnitude dwarfs the radius so the\n"
       "        extents collapse in double precision (e.g. ``center=(1e18,0,0)``\n"
       "        with ``radius=1``) raises ``RuntimeError``\n"
       "        (``SM_ERR_INVALID_INPUT``).\n"
       "\n"
       "Returns:\n"
       "    Circle: closed analytic circle. It is a ``BSplineCurve`` (so it\n"
       "        works anywhere a curve is accepted) and additionally exposes\n"
       "        ``center()``, ``radius()``, ``normal()``, ``x_axis()``, and\n"
       "        ``y_axis()``.\n"
       "\n"
       "Notes:\n"
       "    The circle is constructed in the world XY plane and translated to\n"
       "    ``center`` (``x_axis`` = +X, ``y_axis`` = +Y, ``normal`` = +Z).\n"
       "    For circles in non-axis-aligned planes use the C-API overload\n"
       "    that takes an explicit axis placement (not currently bound).\n"
       "\n"
       "See Also:\n"
       "    create_arc: Open circular arc with start/end angles.\n"
       "    create_ellipse: Ellipse with two principal radii.\n"
       "\n"
       "Wraps: SmApiCreateCircle");

    m.def("create_arc", [](py::tuple center, double radius, double startDeg, double endDeg) {
        SmPoint3d c = to_point(center);
        SmBSplineCurve* r = nullptr;
        CHECK_STATUS(SmApiCreateArc(c, radius, startDeg, endDeg, r));
        return (SmCurve*)r;
    }, py::return_value_policy::reference,
       py::arg("center"), py::arg("radius"), py::arg("start_angle_deg"), py::arg("end_angle_deg"),
       "Create a B-spline circular arc in the XY plane.\n"
       "\n"
       "Args:\n"
       "    center: Center of the arc as ``(x, y, z)``. The arc lies in the\n"
       "        plane Z = ``center.z``.\n"
       "    radius: Radius of the arc. Must be positive.\n"
       "    start_angle_deg: Start angle measured counter-clockwise from the\n"
       "        +X axis, in degrees.\n"
       "    end_angle_deg: End angle measured counter-clockwise from the +X\n"
       "        axis, in degrees. Must differ from ``start_angle_deg``.\n"
       "\n"
       "Returns:\n"
       "    Curve: open B-spline arc swept counter-clockwise from start to\n"
       "    end.\n"
       "\n"
       "See Also:\n"
       "    create_circle: Full closed circle.\n"
       "\n"
       "Wraps: SmApiCreateArc");

    m.def("create_helix", [](py::tuple origin, double height, double radius_start,
                             double radius_end, double turns, bool right_handed, double tolerance) {
        SmPoint3d o = to_point(origin);
        SmBSplineCurve* r = nullptr;
        CHECK_STATUS(SmApiCreateHelix(o, height, radius_start, radius_end, turns,
                                      right_handed ? TRUE : FALSE, tolerance, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("height"), py::arg("radius_start"),
       py::arg("radius_end"), py::arg("turns"),
       py::kw_only(),
       py::arg("right_handed") = true,
       py::arg("tolerance") = 0.0001,
       "Create a B-spline helix about the +Z axis.\n"
       "\n"
       "Args:\n"
       "    origin: Base point ``(x, y, z)``. The helix is centered on the +Z\n"
       "        axis through this point, running from Z = ``origin.z`` to\n"
       "        ``origin.z + height`` and starting on the +X side.\n"
       "    height: Axial length along +Z. Must be in ``[1e-7, 1e18]``.\n"
       "    radius_start: Radius at the base (Z = ``origin.z``). Must be in\n"
       "        ``[1e-7, 1e18]`` (zero is rejected).\n"
       "    radius_end: Radius at the top (Z = ``origin.z + height``). Must be\n"
       "        in ``[1e-7, 1e18]`` (zero is rejected); a value different from\n"
       "        ``radius_start`` produces a conical (tapered) helix.\n"
       "    turns: Number of full 360-degree revolutions over ``height``.\n"
       "        Fractional turns are allowed, but below ~1 turn the helix\n"
       "        winds steeply (nearly straight): a tight ``tolerance`` may\n"
       "        be unmeetable (raising ``SM_ERR_NOT_WITHIN_TOLERANCE`` --\n"
       "        loosen it) and such near-straight paths may be rejected by\n"
       "        ``pipe_sweep``. Use ``turns >= 1`` for a reliable sweep\n"
       "        path. Must be in ``[1e-7, 124.25]`` (the upper bound is the\n"
       "        fitter's control-point limit).\n"
       "    right_handed: If ``True`` (default) the helix winds\n"
       "        counter-clockwise about +Z; ``False`` gives a left-handed\n"
       "        helix. Keyword-only.\n"
       "    tolerance: Guaranteed maximum 3D deviation of the returned curve\n"
       "        from the true helix. Must be in ``[1e-6, 1e18]``. This is a\n"
       "        hard bound, not best-effort: the achieved deviation is verified\n"
       "        after fitting and the call fails if it cannot be met (see\n"
       "        Raises). Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    BSplineCurve: helical curve within ``tolerance`` of the analytic\n"
       "    helix.\n"
       "\n"
       "Raises:\n"
       "    RuntimeError: if ``origin`` is not finite or any argument is\n"
       "        outside the ranges above (NaN, infinity, non-positive, or\n"
       "        too large all raise ``SM_ERR_INVALID_INPUT``); or if the fit\n"
       "        cannot meet ``tolerance`` (e.g. very many ``turns`` at a tight\n"
       "        ``tolerance`` exceeds the fitter's control-point limit), which\n"
       "        raises ``SM_ERR_NOT_WITHIN_TOLERANCE`` and returns no curve.\n"
       "\n"
       "Notes:\n"
       "    The helix is built about the world +Z axis at ``origin``. To\n"
       "    orient it along another axis, transform the resulting curve.\n"
       "\n"
       "See Also:\n"
       "    pipe_sweep: Sweep a circular profile along the helix to make a\n"
       "        solid (spring, thread, coil). Best with ``turns >= 1``;\n"
       "        very low-turn (near-straight) helixes may be rejected, but\n"
       "        the sweep never returns degenerate or fragmented topology.\n"
       "\n"
       "Wraps: SmApiCreateHelix");

    m.def("create_curve", [](std::vector<py::tuple>& pts) {
        SmTArray<SmPoint3d> arr;
        for (auto& t : pts) arr.Add(to_point(t));
        SmBSplineCurve* r = nullptr;
        CHECK_STATUS(SmApiCreateCurve(arr, r));
        return (SmCurve*)r;
    }, py::return_value_policy::reference,
       py::arg("points"),
       "Create a B-spline curve from a sequence of control points.\n"
       "\n"
       "Args:\n"
       "    points: Ordered list of control points ``(x, y, z)``. Must contain\n"
       "        at least four points.\n"
       "\n"
       "Returns:\n"
       "    Curve: B-spline curve approximating the control polygon.\n"
       "\n"
       "Notes:\n"
       "    The result is degree 3 (cubic) with knots spanning ``[0, 1]``;\n"
       "    the interior knots are not evenly spaced. The curve approximates the\n"
       "    control points but is not guaranteed to pass through them.\n"
       "\n"
       "See Also:\n"
       "    create_canonical_curve: Full B-spline spec with explicit knots,\n"
       "        weights, degree, and form.\n"
       "    create_curve_approx_points: Fit (approximate) a curve to points.\n"
       "    create_curve_interp_points: Interpolate (pass through) points.\n"
       "\n"
       "Wraps: SmApiCreateCurve");

    m.def(
        "create_canonical_curve",
        [](std::vector<py::tuple>& points, std::vector<double>& knots,
           std::vector<unsigned long>& knot_multiplicities, py::object weights, unsigned long degree,
           SmBSplineCurveForm form) {
            SmTArray<SmPoint3d> pts;
            fill_points_array(points, pts);
            SmTArray<double> kv;
            for (double k : knots)
                kv.Add(k);
            SmTArray<ULONG> km;
            for (unsigned long mult : knot_multiplicities)
                km.Add(static_cast<ULONG>(mult));
            SmTArray<double>* pw = nullptr;
            SmTArray<double> wstorage;
            if (!weights.is_none()) {
                std::vector<double> wv = weights.cast<std::vector<double>>();
                for (double x : wv)
                    wstorage.Add(x);
                pw = &wstorage;
            }
            SmBSplineCurve* r = nullptr;
            CHECK_STATUS(SmApiCreateCanonicalCurve(pts, kv, km, pw, static_cast<ULONG>(degree), form, r));
            return (SmCurve*)r;
        },
        py::return_value_policy::reference, py::arg("points"), py::arg("knots"),
        py::arg("knot_multiplicities"), py::arg("weights") = py::none(), py::arg("degree") = 3,
        py::arg("form") = SM_CF_UNSPECIFIED,
        "Create a B-spline curve from a full canonical specification.\n"
        "\n"
        "Args:\n"
        "    points: Control points as ``(x, y, z)`` tuples.\n"
        "    knots: Knot values, monotonically non-decreasing.\n"
        "    knot_multiplicities: Multiplicity of each knot. End-knot\n"
        "        multiplicities must equal ``degree + 1``; internal knot\n"
        "        multiplicities must be at most ``degree``.\n"
        "    weights: Optional list of per-control-point weights. ``None``\n"
        "        produces a non-rational B-spline. Defaults to ``None``.\n"
        "    degree: Polynomial degree of the curve. Defaults to ``3`` (cubic).\n"
        "    form: Curve-form hint for downstream consumers. One of\n"
        "        ``BSplineCurveForm.{POLYLINE, CIRCULAR_ARC, ELLIPTIC_ARC,\n"
        "        PARABOLIC_ARC, HYPERBOLIC_ARC, HELICAL_ARC, UNSPECIFIED}``.\n"
        "        Defaults to ``UNSPECIFIED``.\n"
        "\n"
        "Returns:\n"
        "    Curve: B-spline curve constructed exactly per the supplied spec.\n"
        "\n"
        "See Also:\n"
        "    create_curve: Simpler version with auto-chosen knots and degree.\n"
        "\n"
        "Wraps: SmApiCreateCanonicalCurve");

    m.def("offset_curve", [](SmBSplineCurve* curve, double dist) {
        SmBSplineCurve* r = nullptr;
        CHECK_STATUS(SmApiOffsetCurve(curve, dist, r));
        return (SmCurve*)r;
    }, py::return_value_policy::reference,
       py::arg("curve"), py::arg("distance"),
       "Create a curve offset from an existing curve by a signed distance.\n"
       "\n"
       "Args:\n"
       "    curve: Source B-spline curve to offset.\n"
       "    distance: Offset distance. Positive offsets to the right of the\n"
       "        curve direction; negative offsets to the left. Must be\n"
       "        non-zero.\n"
       "\n"
       "Returns:\n"
       "    Curve: offset curve.\n"
       "\n"
       "Notes:\n"
       "    The offset is taken in the curve's local plane. A straight curve\n"
       "    has no plane of its own and is offset in the XY plane (normal +Z),\n"
       "    or in the XZ plane (normal +Y) if it is parallel to Z.\n"
       "    If ``abs(distance)`` exceeds the curve's minimum radius of\n"
       "    curvature the result may self-intersect.\n"
       "\n"
       "See Also:\n"
       "    create_offset_profile: Offset a closed loop of curves into a\n"
       "        Brep face.\n"
       "    create_offset_surface: Offset a surface in 3D.\n"
       "\n"
       "Wraps: SmApiOffsetCurve");

    m.def("create_line", [](py::tuple start, py::tuple direction) {
        SmPoint3d s = to_point(start);
        SmVector3d d = to_vec(direction);
        SmLine* r = nullptr;
        CHECK_STATUS(SmApiCreateLine(s, d, r));
        return (SmCurve*)r;
    }, py::return_value_policy::reference,
       py::arg("start"), py::arg("direction"),
       "Create an infinite line through a point in a given direction.\n"
       "\n"
       "Args:\n"
       "    start: A point on the line as ``(x, y, z)``.\n"
       "    direction: Line direction as ``(dx, dy, dz)``. Must be non-zero;\n"
       "        magnitude need not be unit length.\n"
       "\n"
       "Returns:\n"
       "    Curve: analytical infinite line.\n"
       "\n"
       "See Also:\n"
       "    create_line_segment: Bounded line segment between two points.\n"
       "\n"
       "Wraps: SmApiCreateLine");

    m.def("create_ellipse", [](py::tuple origin, double r1, double r2) {
        SmPoint3d o = to_point(origin);
        SmEllipse* r = nullptr;
        CHECK_STATUS(SmApiCreateEllipse(o, r1, r2, r));
        return (SmCurve*)r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius1"), py::arg("radius2"),
       "Create an analytic ellipse in the XY plane.\n"
       "\n"
       "Args:\n"
       "    origin: Center of the ellipse as ``(x, y, z)``. The ellipse lies\n"
       "        in the plane Z = ``origin.z``.\n"
       "    radius1: Semi-axis length along the X direction. Must be positive.\n"
       "    radius2: Semi-axis length along the Y direction. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Curve: closed analytic ellipse.\n"
       "\n"
       "Notes:\n"
       "    For ``radius1 == radius2`` this produces a circle; prefer\n"
       "    ``create_circle`` for that case.\n"
       "\n"
       "See Also:\n"
       "    create_circle: Equal-radius case as a B-spline circle.\n"
       "\n"
       "Wraps: SmApiCreateEllipse");

    m.def("create_rectangle", [](py::tuple center, double length, double width) {
        SmPoint3d c = to_point(center);
        SmTArray<SmBSplineCurve*> result;
        CHECK_STATUS(SmApiCreateRectangle(c, length, width, result));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < result.GetSize(); i++) v.push_back(result[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("center"), py::arg("length"), py::arg("width"),
       "Create a rectangle as four B-spline line segments.\n"
       "\n"
       "Args:\n"
       "    center: Center of the rectangle as ``(x, y, z)``. The rectangle\n"
       "        lies in the plane Z = ``center.z``.\n"
       "    length: Total length along the X axis. Must be positive and finite.\n"
       "    width: Total width along the Y axis. Must be positive and finite.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: four line segments forming a closed loop, ordered\n"
       "    head to tail.\n"
       "\n"
       "See Also:\n"
       "    create_regular_polygon: N-sided regular polygon as line segments.\n"
       "\n"
       "Wraps: SmApiCreateRectangle");

    m.def("create_regular_polygon", [](py::tuple center, unsigned long nSides, double radius) {
        SmPoint3d c = to_point(center);
        SmTArray<SmBSplineCurve*> result;
        CHECK_STATUS(SmApiCreateRegularPolygon(c, nSides, radius, result));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < result.GetSize(); i++) v.push_back(result[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("center"), py::arg("sides"), py::arg("radius"),
       "Create a regular polygon as a list of B-spline line segments.\n"
       "\n"
       "Args:\n"
       "    center: Center of the polygon as ``(x, y, z)``. The polygon lies\n"
       "        in the plane Z = ``center.z``.\n"
       "    sides: Number of sides. Must be at least 3.\n"
       "    radius: Radius of the circumscribed circle (vertex-to-center\n"
       "        distance). Must be positive and finite.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: ``sides`` line segments forming a closed loop,\n"
       "    ordered head to tail.\n"
       "\n"
       "See Also:\n"
       "    create_rectangle: Four-sided centered rectangle (length/width).\n"
       "\n"
       "Wraps: SmApiCreateRegularPolygon");

    m.def("create_curve_approx_points", [](std::vector<py::tuple>& pts) {
        SmTArray<SmPoint3d> arr;
        for (auto& t : pts) arr.Add(to_point(t));
        SmCurve* r = nullptr;
        CHECK_STATUS(SmApiCreateCurveApproxPoints(arr, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("points"),
       "Create a B-spline curve that approximates (fits) a sequence of points.\n"
       "\n"
       "Args:\n"
       "    points: Points to approximate as ``(x, y, z)`` tuples. Must contain\n"
       "        at least two points.\n"
       "\n"
       "Returns:\n"
       "    Curve: B-spline curve fit near the points (does not necessarily\n"
       "    pass through them exactly).\n"
       "\n"
       "Notes:\n"
       "    The result uses uniform knots on the parameter interval ``[0, 1]``.\n"
       "\n"
       "See Also:\n"
       "    create_curve_interp_points: Interpolate (pass through) points\n"
       "        exactly.\n"
       "    create_curve: Use the input points as control points instead of\n"
       "        as fit targets.\n"
       "\n"
       "Wraps: SmApiCreateCurveApproxPoints");

    m.def("create_curve_interp_points", [](std::vector<py::tuple>& pts,
                                           SmCurveParameterizationType parameterization) {
        SmTArray<SmPoint3d> arr;
        for (auto& t : pts) arr.Add(to_point(t));
        SmCurve* r = nullptr;
        CHECK_STATUS(SmApiCreateCurveInterpPoints(arr, r, parameterization));
        return r;
    }, py::return_value_policy::reference,
       py::arg("points"),
       py::kw_only(),
       py::arg("parameterization") = SM_CP_UNIFORM,
       "Create a B-spline curve that interpolates a sequence of points exactly.\n"
       "\n"
       "Args:\n"
       "    points: Points to interpolate as ``(x, y, z)`` tuples. Requires\n"
       "        at least four finite points (degree-3 fit); fewer points or a\n"
       "        non-finite coordinate raises ``RuntimeError``\n"
       "        (``SM_ERR_INVALID_INPUT``). For the distance-based modes\n"
       "        (``CHORD_LENGTH`` and ``CENTRIPETAL``) successive points must\n"
       "        also be distinct: a coincident pair (including all-coincident\n"
       "        input) has no defined spacing and raises ``RuntimeError``\n"
       "        (``SM_ERR_INVALID_INPUT``). ``UNIFORM`` tolerates duplicates.\n"
       "    parameterization: Knot spacing for the fit, a\n"
       "        ``CurveParameterization`` value. ``UNIFORM`` (default) spaces\n"
       "        knots evenly; ``CHORD_LENGTH`` and ``CENTRIPETAL`` space them\n"
       "        by inter-point distance (centripetal reduces overshoot on\n"
       "        sharp turns). Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    Curve: B-spline curve passing through every supplied point on the\n"
       "    parameter interval ``[0, 1]``.\n"
       "\n"
       "See Also:\n"
       "    create_curve_approx_points: Fit (approximate) a curve to points.\n"
       "    create_curve: Use the input points as control points instead of\n"
       "        as interpolation targets.\n"
       "\n"
       "Wraps: SmApiCreateCurveInterpPoints");

    m.def("evaluate_curve", [](SmCurve* curve, double param) {
        // SmApiEvaluateCurve takes ``SmVector3d*&`` for each output and only
        // writes through pointers that are non-null on entry, so we must
        // give it real backing storage.  Earlier revisions of this binding
        // passed ``nullptr`` for all three slots, silently discarding every
        // result and returning an empty dict.
        SmVector3d point_storage, deriv1_storage, deriv2_storage;
        SmVector3d* point  = &point_storage;
        SmVector3d* deriv1 = &deriv1_storage;
        SmVector3d* deriv2 = &deriv2_storage;
        CHECK_STATUS(SmApiEvaluateCurve(curve, param, point, deriv1, deriv2));
        py::dict result;
        result["point"]     = py::make_tuple(point->x,  point->y,  point->z);
        result["tangent"]   = py::make_tuple(deriv1->x, deriv1->y, deriv1->z);
        result["curvature"] = py::make_tuple(deriv2->x, deriv2->y, deriv2->z);
        return result;
    }, py::arg("curve"), py::arg("parameter"),
       "Evaluate a curve at a parametric value, returning point and derivatives.\n"
       "\n"
       "Args:\n"
       "    curve: Curve to evaluate.\n"
       "    parameter: Parameter value, in the curve's natural parametric\n"
       "        domain.\n"
       "\n"
       "Returns:\n"
       "    dict: with keys ``\"point\"`` (3D position), ``\"tangent\"`` (first\n"
       "    derivative) and ``\"curvature\"`` (second derivative). Each value\n"
       "    is an ``(x, y, z)`` tuple.\n"
       "\n"
       "See Also:\n"
       "    Curve.evaluate: Position-only convenience method.\n"
       "    Curve.parameter_range: Valid range for ``parameter``.\n"
       "    evaluate_continuity: Continuity classification at a parameter.\n"
       "\n"
       "Wraps: SmApiEvaluateCurve");

    m.def("evaluate_continuity", [](SmCurve* curve, double param) {
        SmContinuityType ct = SM_CT_DISCONTINUOUS;
        CHECK_STATUS(SmApiEvaluateContinuity(curve, param, ct));
        return ct;
    }, py::arg("curve"), py::arg("parameter"),
       "Classify the parametric continuity of a curve at a given parameter.\n"
       "\n"
       "Args:\n"
       "    curve: Curve to evaluate.\n"
       "    parameter: Parameter within ``curve.parameter_range()``\n"
       "        (endpoints included).\n"
       "\n"
       "Returns:\n"
       "    ContinuityType: continuity classification at the parameter,\n"
       "    including mixed parametric/geometric classes such as\n"
       "    ``C1_G2_G3``.\n"
       "\n"
       "Raises:\n"
       "    RuntimeError: ``SM_ERR_INVALID_INPUT`` for a null curve or a\n"
       "        parameter outside the range (including NaN); otherwise\n"
       "        reports the kernel's continuity-evaluation failure.\n"
       "\n"
       "Notes:\n"
       "    Internally evaluates the curve from the left and from the right\n"
       "    and compares position, tangent direction, and higher-derivative\n"
       "    agreement to assign the continuity class.\n"
       "\n"
       "See Also:\n"
       "    evaluate_curve: Position and derivative values at a parameter.\n"
       "\n"
       "Wraps: SmApiEvaluateContinuity");

    m.def("drop_curve_to_surface", [](SmBSplineCurve* curve, SmSurface* srf) {
        SmTArray<SmBSplineCurve*> curves3d;
        CHECK_STATUS(SmApiDropCurveToSrf(curve, srf, nullptr, &curves3d));
        std::vector<SmCurve*> v3d;
        for (ULONG i = 0; i < curves3d.GetSize(); i++) v3d.push_back(curves3d[i]);
        return v3d;
    }, py::return_value_policy::reference,
       py::arg("curve"), py::arg("surface"),
       "Drop a 3D curve onto a surface along the surface normals.\n"
       "\n"
       "Args:\n"
       "    curve: 3D B-spline curve to drop.\n"
       "    surface: Target surface.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: 3D curves on the surface produced by drop-and-trim.\n"
       "\n"
       "Notes:\n"
       "    Internally drops the curve into the surface's natural UV domain,\n"
       "    trims against the domain, and lifts the trimmed UV curves back to\n"
       "    3D. Equivalent to ``project_curve_to_surface`` but uses a fixed\n"
       "    drop tolerance (``project_curve_to_surface`` derives a tolerance\n"
       "    from the curve's bounding box).\n"
       "\n"
       "See Also:\n"
       "    project_curve_to_surface: Same operation with a bbox-derived\n"
       "        tolerance.\n"
       "    lift_uv_curve: Inverse operation - lift a 2D UV curve to 3D.\n"
       "\n"
       "Wraps: SmApiDropCurveToSrf");

    m.def("convert_to_lines_and_arcs", [](SmBSplineCurve* curve) {
        SmTArray<SmBSplineCurve*> result;
        CHECK_STATUS(SmApiConvertToLinesAndArcs(curve, result));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < result.GetSize(); i++) v.push_back(result[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("curve"),
       "Convert a B-spline curve to a sequence of analytic lines and arcs.\n"
       "\n"
       "Args:\n"
       "    curve: B-spline curve to convert.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: line and arc segments approximating the input curve.\n"
       "    A line, an arc, or a curve smaller than the 0.001 fit tolerance\n"
       "    is returned as a copy of itself. A degenerate curve (a point)\n"
       "    raises ``RuntimeError``.\n"
       "\n"
       "Notes:\n"
       "    Useful for downstream consumers (CAM toolpaths, DXF/IGES export)\n"
       "    that prefer analytic geometry over general B-splines.\n"
       "\n"
       "Wraps: SmApiConvertToLinesAndArcs");

    m.def("project_curve_to_surface", [](SmBSplineCurve* curve, SmSurface* srf) {
        SmTArray<SmBSplineCurve*> curves3d;
        CHECK_STATUS(SmApiProjectCurve(curve, srf, nullptr, &curves3d));
        std::vector<SmCurve*> v3d;
        for (ULONG i = 0; i < curves3d.GetSize(); i++) v3d.push_back(curves3d[i]);
        return v3d;
    }, py::return_value_policy::reference,
       py::arg("curve"), py::arg("surface"),
       "Project a 3D curve onto a surface along the surface normals.\n"
       "\n"
       "Args:\n"
       "    curve: 3D B-spline curve to project.\n"
       "    surface: Target surface.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: 3D curves on the surface produced by drop-and-trim.\n"
       "\n"
       "Notes:\n"
       "    Internally identical to ``drop_curve_to_surface`` but derives the\n"
       "    drop tolerance from the input curve's bounding-box diagonal\n"
       "    (clamped to a minimum of 1.0). Prefer this variant for tightly\n"
       "    bounded curves; for a fixed tolerance use\n"
       "    ``drop_curve_to_surface``.\n"
       "\n"
       "See Also:\n"
       "    drop_curve_to_surface: Same operation with a fixed tolerance.\n"
       "    lift_uv_curve: Inverse operation - lift a 2D UV curve to 3D.\n"
       "\n"
       "Wraps: SmApiProjectCurve");

    m.def("lift_uv_curve", [](SmBSplineCurve* uvCurve, SmSurface* srf) {
        SmTArray<SmBSplineCurve*> curves3d;
        CHECK_STATUS(SmApiLiftUVCurveFromSrf(uvCurve, srf, &curves3d));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < curves3d.GetSize(); i++) v.push_back(curves3d[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("uv_curve"), py::arg("surface"),
       "Lift a 2D UV-space curve onto a surface, producing 3D curves.\n"
       "\n"
       "Args:\n"
       "    uv_curve: 2D B-spline curve in the surface's parametric domain.\n"
       "    surface: Surface providing the UV-to-3D mapping.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: 3D curves on the surface corresponding to the UV\n"
       "    input.\n"
       "\n"
       "See Also:\n"
       "    drop_curve_to_surface: Inverse - drop a 3D curve onto a surface.\n"
       "    project_curve_to_surface: Inverse with bbox-derived tolerance.\n"
       "\n"
       "Wraps: SmApiLiftUVCurveFromSrf");

    m.def("remove_curve_knots", [](SmCurve* curve) {
        CHECK_STATUS(SmApiRemoveCurveKnots(curve));
        return curve;
    }, py::return_value_policy::reference,
       py::arg("curve"),
       "Mutates: curve; returns the same handle for chaining.\n"
       "\n"
       "Remove unnecessary B-spline knots within the curve's approximation tolerance.\n"
       "\n"
       "Args:\n"
       "    curve: Curve to simplify in place.\n"
       "\n"
       "Notes:\n"
       "    Uses the curve's effective approximation tolerance, resolved from its\n"
       "    owning topology or context. Non-B-spline curves are unchanged.\n"
       "\n"
       "Wraps: SmApiRemoveCurveKnots");

    m.def("make_curves_compatible", [](std::vector<SmBSplineCurve*>& curves) {
        SmTArray<SmBSplineCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        CHECK_STATUS(SmApiMakeCurvesCompatible(arr));
    }, py::arg("curves"),
       "Reparameterize a list of B-spline curves to share a compatible knot vector.\n"
       "\n"
       "Args:\n"
       "    curves: B-spline curves to make compatible. Must contain at least\n"
       "        two curves. The curves are modified in place, one at a time,\n"
       "        so a failure can leave some of them already modified.\n"
       "\n"
       "Notes:\n"
       "    Required before operations that need aligned parameterizations\n"
       "    across curves, e.g. skinning (``create_skin_surface``) and ruled\n"
       "    surfaces (``create_ruled_surface``). The original curve handles\n"
       "    are preserved but their internal knot vectors are rewritten.\n"
       "\n"
       "See Also:\n"
       "    create_skin_surface: Lofted surface across multiple compatible\n"
       "        curves.\n"
       "    create_ruled_surface: Linear ruling between two compatible\n"
       "        curves.\n"
       "\n"
       "Wraps: SmApiMakeCurvesCompatible");

    m.def("order_curves", [](std::vector<SmBSplineCurve*>& curves) {
        SmTArray<SmBSplineCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        CHECK_STATUS(SmApiOrderCurves(arr));
        std::vector<SmBSplineCurve*> result;
        for (ULONG i = 0; i < arr.GetSize(); i++) result.push_back(arr[i]);
        return result;
    }, py::return_value_policy::reference,
       py::arg("curves"),
       "Reorder a list of B-spline curves into loop sequence.\n"
       "\n"
       "Args:\n"
       "    curves: Coplanar B-spline curves that form one or more closed\n"
       "        loops. Must contain at least two curves.\n"
       "\n"
       "Returns:\n"
       "    list[BSplineCurve]: the same curves, grouped loop by loop in\n"
       "    connection order.\n"
       "\n"
       "Notes:\n"
       "    Curves are not reversed, and the per-curve orientation and loop\n"
       "    boundaries are not reported, so consecutive curves need not meet\n"
       "    head to tail: a curve may connect through its end rather than its\n"
       "    start. The curves themselves are not modified - only the ordering\n"
       "    of the returned list changes. Open or non-planar input fails.\n"
       "\n"
       "Wraps: SmApiOrderCurves");
}
