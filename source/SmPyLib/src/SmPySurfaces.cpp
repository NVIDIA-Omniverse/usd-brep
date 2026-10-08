// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiSurfaces.h (surface create/offset/ruled/revolution/skin/sweep/
// extrude), SmApiTrimmedSurfaces.h (SmApiCreatePlanarFaces).  Surface
// evaluation (point/normal/derivatives) lives in SmPyQueries.cpp.

#include "SmPyCommon.h"

#include <SmApiSurfaces.h>
#include <SmApiTrimmedSurfaces.h>

void bind_surfaces(BoundModule& m)
{
    m.def("create_surface_from_points", [](std::vector<py::tuple>& pts,
                                           unsigned long rows, unsigned long cols) {
        SmTArray<SmPoint3d> arr;
        for (auto& t : pts) arr.Add(to_point(t));
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSurface(arr, rows, cols, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("control_points"), py::arg("rows"), py::arg("cols"),
       "Create a B-spline surface from a grid of control points.\n"
       "\n"
       "Args:\n"
       "    control_points: Grid of control points in row-major order:\n"
       "        ``P[row][col] = control_points[row * cols + col]``. Total\n"
       "        length must equal ``rows * cols``.\n"
       "    rows: Number of rows in the control-point grid. Must be at\n"
       "        least 2.\n"
       "    cols: Number of columns in the control-point grid. Must be at\n"
       "        least 2.\n"
       "\n"
       "Returns:\n"
       "    Surface: B-spline surface that approximates the control polygon.\n"
       "\n"
       "Notes:\n"
       "    The surface approximates the control points but does not\n"
       "    necessarily interpolate them. For interpolation through a fixed\n"
       "    grid use ``create_surface_from_ordered_points``.\n"
       "\n"
       "See Also:\n"
       "    create_surface_from_corner_points: Bilinear patch from 4 corners.\n"
       "    create_surface_from_ordered_points: Fit through an ordered grid.\n"
       "    create_surface_from_random_points: Fit through unordered points.\n"
       "\n"
       "Wraps: SmApiCreateSurface");

    m.def("create_ruled_surface", [](SmBSplineCurve* c1, SmBSplineCurve* c2) {
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateRuledSurface(c1, c2, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curve1"), py::arg("curve2"),
       "Create a ruled surface that linearly interpolates between two curves.\n"
       "\n"
       "Args:\n"
       "    curve1: First boundary B-spline curve.\n"
       "    curve2: Second boundary B-spline curve.\n"
       "\n"
       "Returns:\n"
       "    Surface: ruled surface formed by straight lines connecting\n"
       "    corresponding points on the two curves.\n"
       "\n"
       "Notes:\n"
       "    Best results require compatible parameterizations on the two\n"
       "    curves. If they differ, call ``make_curves_compatible`` first.\n"
       "\n"
       "See Also:\n"
       "    create_skin_surface: Lofted surface through three or more\n"
       "        profiles.\n"
       "    make_curves_compatible: Reparameterize curves to share knots.\n"
       "\n"
       "Wraps: SmApiCreateRuledSurface");

    m.def("create_surface_revolution", [](SmBSplineCurve* curve,
                                          py::tuple origin, py::tuple axis, double angleDeg) {
        SmPoint3d o = to_point(origin);
        SmVector3d a = to_vec(axis);
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSurfaceRevolution(curve, o, a, angleDeg, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curve"), py::arg("origin"), py::arg("axis"), py::arg("angle_deg"),
       "Create a surface of revolution by rotating a curve about an axis.\n"
       "\n"
       "Args:\n"
       "    curve: Generatrix B-spline curve to revolve.\n"
       "    origin: Point on the axis of revolution as ``(x, y, z)``.\n"
       "    axis: Direction vector of the axis as ``(dx, dy, dz)``. Must be\n"
       "        non-zero.\n"
       "    angle_deg: Sweep angle, in degrees. Use ``360`` for a closed\n"
       "        surface of revolution.\n"
       "\n"
       "Returns:\n"
       "    Surface: surface of revolution.\n"
       "\n"
       "Notes:\n"
       "    The generatrix should not cross the rotation axis - that produces\n"
       "    self-intersection. The result is a standalone surface, not a\n"
       "    solid; for a solid of revolution use ``rotational_sweep``.\n"
       "\n"
       "See Also:\n"
       "    create_extrude_surface: Linear sweep along a vector.\n"
       "    create_sweep_surface: Sweep along a path curve.\n"
       "\n"
       "Wraps: SmApiCreateSurfaceRevolution");

    m.def("create_offset_surface", [](SmSurface* srf, double dist) {
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateOffsetSurface(srf, dist, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("surface"), py::arg("distance"),
       "Create a surface offset from an existing surface along its normals.\n"
       "\n"
       "Args:\n"
       "    surface: Source surface.\n"
       "    distance: Signed offset distance. Positive offsets along the\n"
       "        surface normal; negative offsets opposite the normal. Must\n"
       "        be non-zero.\n"
       "\n"
       "Returns:\n"
       "    Surface: offset surface.\n"
       "\n"
       "Notes:\n"
       "    If ``abs(distance)`` exceeds the surface's minimum radius of\n"
       "    curvature the result may self-intersect. An offset that collapses\n"
       "    the surface (a sphere's radius, a torus's minor radius, or both\n"
       "    cone radii shrinking to zero) produces no surface and raises\n"
       "    ``RuntimeError``.\n"
       "\n"
       "See Also:\n"
       "    offset_curve: Offset a single curve in 2D.\n"
       "    offset_brep: Offset every face of a Brep.\n"
       "\n"
       "Wraps: SmApiCreateOffsetSurface");

    m.def("create_surface_from_corner_points", [](py::tuple c1, py::tuple c2,
                                                  py::tuple c3, py::tuple c4) {
        SmPoint3d p1 = to_point(c1), p2 = to_point(c2);
        SmPoint3d p3 = to_point(c3), p4 = to_point(c4);
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSurfaceFromCornerPoints(p1, p2, p3, p4, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("corner_umin_vmin"), py::arg("corner_umax_vmin"),
       py::arg("corner_umin_vmax"), py::arg("corner_umax_vmax"),
       "Create a bilinear B-spline surface from four corner points.\n"
       "\n"
       "Args:\n"
       "    corner_umin_vmin: Corner at ``(u=0, v=0)`` as ``(x, y, z)``.\n"
       "    corner_umax_vmin: Corner at ``(u=1, v=0)`` as ``(x, y, z)``.\n"
       "    corner_umin_vmax: Corner at ``(u=0, v=1)`` as ``(x, y, z)``.\n"
       "    corner_umax_vmax: Corner at ``(u=1, v=1)`` as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    Surface: bilinear B-spline patch interpolating the four corners.\n"
       "\n"
       "Notes:\n"
       "    Useful as a quick test surface or as a base for further trimming\n"
       "    operations. Corners need not be coplanar - if they are not, the\n"
       "    result is a doubly-curved hyperbolic paraboloid (saddle) patch.\n"
       "\n"
       "See Also:\n"
       "    create_surface_from_points: Surface from a control-point grid.\n"
       "\n"
       "Wraps: SmApiCreateSurfaceFromCornerPoints");

    m.def("create_surface_from_ordered_points", [](std::vector<py::tuple>& pts,
                                                   unsigned long rows, unsigned long cols) {
        SmTArray<SmPoint3d> arr;
        for (auto& t : pts) arr.Add(to_point(t));
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSurfaceFromOrderedPoints(arr, rows, cols, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("points"), py::arg("rows"), py::arg("cols"),
       "Create a B-spline surface that interpolates an ordered grid of points.\n"
       "\n"
       "Args:\n"
       "    points: Grid of points in row-major order:\n"
       "        ``P[row][col] = points[row * cols + col]``. Total length must\n"
       "        equal ``rows * cols``.\n"
       "    rows: Number of rows in the point grid. Must be at least 2.\n"
       "    cols: Number of columns in the point grid. Must be at least 2.\n"
       "\n"
       "Returns:\n"
       "    Surface: B-spline surface passing through every input point.\n"
       "\n"
       "Notes:\n"
       "    Unlike ``create_surface_from_points`` (which uses the inputs as\n"
       "    control points), this fits a surface that interpolates the points\n"
       "    exactly.\n"
       "\n"
       "See Also:\n"
       "    create_surface_from_points: Use inputs as control points instead.\n"
       "    create_surface_from_random_points: Fit through unordered points.\n"
       "\n"
       "Wraps: SmApiCreateSurfaceFromOrderedPoints");

    m.def("create_surface_from_random_points", [](std::vector<py::tuple>& pts) {
        SmTArray<SmPoint3d> arr;
        for (auto& t : pts) arr.Add(to_point(t));
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSurfaceFromRandomPoints(arr, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("points"),
       "Fit a B-spline surface through an unordered cloud of points.\n"
       "\n"
       "Args:\n"
       "    points: Unordered point cloud as a list of ``(x, y, z)`` tuples.\n"
       "\n"
       "Returns:\n"
       "    Surface: B-spline surface fit to the input cloud.\n"
       "\n"
       "Notes:\n"
       "    The kernel infers a parameterization from the spatial distribution\n"
       "    of the points; results are sensitive to point density and outlier\n"
       "    points. For known grid topology prefer\n"
       "    ``create_surface_from_ordered_points``.\n"
       "\n"
       "See Also:\n"
       "    create_surface_from_ordered_points: Fit through an ordered grid.\n"
       "\n"
       "Wraps: SmApiCreateSurfaceFromRandomPoints");

    m.def("create_extrude_surface", [](SmBSplineCurve* curve, py::tuple sweepVec) {
        SmVector3d v = to_vec(sweepVec);
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateExtrude(curve, v, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curve"), py::arg("sweep_vector"),
       "Create an extruded surface by sweeping a curve along a straight vector.\n"
       "\n"
       "Args:\n"
       "    curve: Generatrix B-spline curve to extrude.\n"
       "    sweep_vector: Extrusion vector as ``(dx, dy, dz)``. Magnitude\n"
       "        defines the extrusion length; direction defines the sweep\n"
       "        direction. Must be non-zero.\n"
       "\n"
       "Returns:\n"
       "    Surface: extruded surface.\n"
       "\n"
       "Notes:\n"
       "    Produces a standalone surface, not a solid. For a solid extrusion\n"
       "    of a closed profile use ``linear_sweep``.\n"
       "\n"
       "See Also:\n"
       "    create_sweep_surface: Sweep along an arbitrary path curve.\n"
       "    create_surface_revolution: Sweep along a circular axis.\n"
       "    linear_sweep: Solid extrusion of a closed profile.\n"
       "\n"
       "Wraps: SmApiCreateExtrude");

    m.def("create_sweep_surface", [](SmBSplineCurve* curve, SmBSplineCurve* path) {
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSweep(curve, path, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curve"), py::arg("path"),
       "Create a swept surface by sweeping a profile curve along a path curve.\n"
       "\n"
       "Args:\n"
       "    curve: Generatrix profile B-spline curve.\n"
       "    path: Sweep-path B-spline curve.\n"
       "\n"
       "Returns:\n"
       "    Surface: swept surface.\n"
       "\n"
       "See Also:\n"
       "    create_extrude_surface: Sweep along a straight vector.\n"
       "    create_surface_revolution: Sweep around an axis.\n"
       "    pipe_sweep: Solid pipe along a path.\n"
       "\n"
       "Wraps: SmApiCreateSweep");

    m.def("create_skin_surface", [](std::vector<SmBSplineCurve*>& profiles) {
        SmTArray<SmBSplineCurve*> arr;
        for (auto* c : profiles) arr.Add(c);
        SmSurface* r = nullptr;
        CHECK_STATUS(SmApiCreateSkin(arr, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("profiles"),
       "Create a lofted (skinned) surface through a sequence of profile curves.\n"
       "\n"
       "Args:\n"
       "    profiles: Ordered list of B-spline profile curves. Must contain at\n"
       "        least two profiles.\n"
       "\n"
       "Returns:\n"
       "    Surface: lofted surface passing through every profile.\n"
       "\n"
       "Notes:\n"
       "    Best results require compatible parameterizations across all\n"
       "    profiles. If they differ, call ``make_curves_compatible`` first.\n"
       "\n"
       "See Also:\n"
       "    create_ruled_surface: Two-curve case (linear ruling).\n"
       "    make_curves_compatible: Reparameterize profiles to share knots.\n"
       "\n"
       "Wraps: SmApiCreateSkin");

    m.def("create_planar_faces", [](std::vector<SmCurve*>& curves) {
        SmTArray<SmCurve*> arr;
        for (auto* c : curves) arr.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePlanarFaces(arr, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves"),
       "Consumes: curves; returns new Brep.\n"
       "\n"
       "Build a sheet-body Brep made of planar faces from one or more closed\n"
       "planar curve loops.\n"
       "\n"
       "Args:\n"
       "    curves: Coplanar curves forming one or more closed loops. Curves\n"
       "        may be supplied in any order and any orientation, but they\n"
       "        must not intersect each other and each endpoint must be\n"
       "        coincident with exactly one other endpoint. Consumed on\n"
       "        success: the new faces use them, so do not reuse them.\n"
       "        On failure, the curves remain owned by the caller.\n"
       "\n"
       "Returns:\n"
       "    Brep: sheet-body Brep whose faces are the planar regions bounded\n"
       "    by the input loops.\n"
       "\n"
       "Notes:\n"
       "    The plane of the result is inferred from the input curves; do not\n"
       "    pass a plane normal. Multiple coplanar loops produce one face per\n"
       "    loop (or with-holes faces if loops nest), all packed into a single\n"
       "    Brep. Returns a sheet body (open shell), not a manifold solid -\n"
       "    use ``linear_sweep`` to extrude into a solid.\n"
       "\n"
       "See Also:\n"
       "    create_plane: Single-rectangle planar sheet.\n"
       "    linear_sweep: Solid extrusion of a closed profile.\n"
       "\n"
       "Wraps: SmApiCreatePlanarFaces");
}
