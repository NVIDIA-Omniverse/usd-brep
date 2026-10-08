// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiIntersectors.h (Brep-Brep, Brep-Plane, Curve-Curve,
// Surface-Surface, Face-Face, Curve-Surface, Curve-Brep, Curve-Face).

#include "SmPyCommon.h"

#include <SmApiIntersectors.h>

void bind_intersectors(BoundModule& m)
{
    m.def("intersect_breps", [](SmBrep* a, SmBrep* b) {
        SmTArray<SmCurve*> curves;
        SmTArray<SmPoint3d> points;
        CHECK_STATUS(SmApiIntersectBreps(a, b, curves, &points));
        std::vector<SmCurve*> vc;
        for (ULONG i = 0; i < curves.GetSize(); i++) vc.push_back(curves[i]);
        std::vector<SmPoint3d> vp;
        for (ULONG i = 0; i < points.GetSize(); i++) vp.push_back(points[i]);
        return py::make_tuple(vc, vp);
    }, py::arg("brep1"), py::arg("brep2"),
       "Compute the intersection curves and points between two Breps.\n"
       "\n"
       "Args:\n"
       "    brep1: First Brep. Not modified.\n"
       "    brep2: Second Brep. Not modified.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[Curve], list[Point3d]]: ``(curves, points)`` where\n"
       "    ``curves`` are the surface-surface intersection curves and\n"
       "    ``points`` are isolated tangential or degenerate intersections.\n"
       "\n"
       "Notes:\n"
       "    Pure query - inputs are not consumed (unlike ``boolean``). The\n"
       "    underlying machinery is the same surface-surface intersector\n"
       "    that powers booleans, but with no inside/outside classification\n"
       "    or face removal. Useful for interference checking: a non-empty\n"
       "    result means the two bodies overlap or touch.\n"
       "\n"
       "See Also:\n"
       "    boolean: Same intersection plus boolean classification.\n"
       "    intersect_brep_with_plane: Section a Brep by a plane.\n"
       "    intersect_curve_brep: Curve-versus-Brep intersection.\n"
       "\n"
       "Wraps: SmApiIntersectBreps");

    m.def("intersect_brep_with_plane", [](SmBrep* brep, py::tuple planePt, py::tuple planeNorm) {
        SmVector3d pt = to_vec(planePt);
        SmVector3d nm = to_vec(planeNorm);
        SmTArray<SmCurve*> curves;
        CHECK_STATUS(SmApiIntersectBrepWithPlane(brep, pt, nm, curves));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < curves.GetSize(); i++) v.push_back(curves[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("plane_point"), py::arg("plane_normal"),
       "Compute the section curves where a plane cuts through a Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to section. Not modified.\n"
       "    plane_point: Any point on the cutting plane as ``(x, y, z)``.\n"
       "    plane_normal: Normal vector of the cutting plane as\n"
       "        ``(dx, dy, dz)``. Must be non-zero; does not need to be\n"
       "        unit-length.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: section curves. Each connected cross-section loop\n"
       "    becomes a closed curve.\n"
       "\n"
       "Notes:\n"
       "    Non-destructive query. To actually cut the Brep, use ``cut``\n"
       "    instead.\n"
       "\n"
       "See Also:\n"
       "    cut: Destructive plane-cut variant.\n"
       "    intersect_breps: Brep-versus-Brep intersection.\n"
       "\n"
       "Wraps: SmApiIntersectBrepWithPlane");

    m.def("intersect_curves", [](SmCurve* c1, SmCurve* c2) {
        SmTArray<SmPoint3d> pts;
        SmTArray<double> params1, params2;
        CHECK_STATUS(SmApiIntersectCurves(c1, c2, &pts, &params1, &params2));
        std::vector<py::tuple> vpts;
        for (ULONG i = 0; i < pts.GetSize(); i++)
            vpts.push_back(py::make_tuple(pts[i].x, pts[i].y, pts[i].z));
        std::vector<double> vp1, vp2;
        for (ULONG i = 0; i < params1.GetSize(); i++) vp1.push_back(params1[i]);
        for (ULONG i = 0; i < params2.GetSize(); i++) vp2.push_back(params2[i]);
        return py::make_tuple(vpts, vp1, vp2);
    }, py::arg("curve1"), py::arg("curve2"),
       "Compute the intersection points of two curves.\n"
       "\n"
       "Args:\n"
       "    curve1: First curve. Not modified.\n"
       "    curve2: Second curve. Not modified.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[tuple[float, float, float]], list[float], list[float]]:\n"
       "    ``(points, params_on_curve1, params_on_curve2)`` where each\n"
       "    list has the same length. Element ``i`` is the same\n"
       "    intersection expressed as a 3D point and as parameters on the\n"
       "    two input curves.\n"
       "\n"
       "Notes:\n"
       "    Useful for finding profile crossings, computing trim\n"
       "    parameters, and building intersection graphs.\n"
       "\n"
       "See Also:\n"
       "    intersect_curve_surface: Curve-versus-surface intersection.\n"
       "    intersect_curve_brep: Curve-versus-Brep intersection.\n"
       "    intersect_curve_face: Curve-versus-trimmed-face intersection.\n"
       "\n"
       "Wraps: SmApiIntersectCurves");

    m.def("intersect_surfaces", [](SmSurface* s1, SmSurface* s2) {
        SmTArray<SmCurve*> curves;
        CHECK_STATUS(SmApiIntersectSurfaces(s1, s2, &curves));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < curves.GetSize(); i++) v.push_back(curves[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("surface1"), py::arg("surface2"),
       "Compute the intersection curves of two surfaces.\n"
       "\n"
       "Args:\n"
       "    surface1: First surface. Not modified.\n"
       "    surface2: Second surface. Not modified.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: 3D intersection curves on both surfaces.\n"
       "\n"
       "Notes:\n"
       "    Operates on untrimmed surfaces. For face-versus-face (trimmed)\n"
       "    intersection use ``intersect_faces``.\n"
       "\n"
       "See Also:\n"
       "    intersect_faces: Trimmed-face variant.\n"
       "    intersect_breps: Whole-Brep intersection.\n"
       "\n"
       "Wraps: SmApiIntersectSurfaces");

    m.def("intersect_faces", [](SmFace* f1, SmFace* f2) {
        SmTArray<SmCurve*> curves;
        SmTArray<SmPoint3d> pts;
        CHECK_STATUS(SmApiIntersectFaces(f1, f2, curves, &pts));
        std::vector<SmCurve*> vc;
        for (ULONG i = 0; i < curves.GetSize(); i++) vc.push_back(curves[i]);
        std::vector<py::tuple> vp;
        for (ULONG i = 0; i < pts.GetSize(); i++)
            vp.push_back(py::make_tuple(pts[i].x, pts[i].y, pts[i].z));
        return py::make_tuple(vc, vp);
    }, py::arg("face1"), py::arg("face2"),
       "Compute the intersection of two trimmed faces.\n"
       "\n"
       "Args:\n"
       "    face1: First trimmed face. Not modified.\n"
       "    face2: Second trimmed face. Not modified.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[Curve], list[tuple[float, float, float]]]:\n"
       "    ``(curves, points)`` where ``curves`` are the 3D intersection\n"
       "    curves clipped to both face trims and ``points`` are isolated\n"
       "    tangential intersections.\n"
       "\n"
       "Notes:\n"
       "    Differs from ``intersect_surfaces`` in that the intersection\n"
       "    is clipped by each face's trimming loops.\n"
       "\n"
       "See Also:\n"
       "    intersect_surfaces: Untrimmed surface variant.\n"
       "\n"
       "Wraps: SmApiIntersectFaces");

    m.def("intersect_curve_surface", [](SmCurve* curve, SmSurface* srf) {
        SmTArray<SmCurve*> curves;
        SmTArray<SmPoint3d> pts;
        CHECK_STATUS(SmApiIntersectCurveSurface(curve, srf, curves, pts));
        std::vector<SmCurve*> vc;
        for (ULONG i = 0; i < curves.GetSize(); i++) vc.push_back(curves[i]);
        std::vector<py::tuple> vp;
        for (ULONG i = 0; i < pts.GetSize(); i++)
            vp.push_back(py::make_tuple(pts[i].x, pts[i].y, pts[i].z));
        return py::make_tuple(vc, vp);
    }, py::arg("curve"), py::arg("surface"),
       "Compute the intersection of a curve with a surface.\n"
       "\n"
       "Args:\n"
       "    curve: Input curve. Not modified.\n"
       "    surface: Input surface. Not modified.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[Curve], list[tuple[float, float, float]]]:\n"
       "    ``(curves, points)``. ``curves`` cover any segments where the\n"
       "    input curve lies on the surface; ``points`` are the isolated\n"
       "    crossing points.\n"
       "\n"
       "See Also:\n"
       "    intersect_curve_face: Trimmed-face variant.\n"
       "    intersect_curve_brep: Whole-Brep variant.\n"
       "\n"
       "Wraps: SmApiIntersectCurveSurface");

    m.def("intersect_curve_brep", [](SmCurve* curve, SmBrep* brep) {
        SmTArray<SmCurve*> curves;
        SmTArray<SmPoint3d> pts;
        CHECK_STATUS(SmApiIntersectCurveBrep(curve, brep, curves, pts));
        std::vector<SmCurve*> vc;
        for (ULONG i = 0; i < curves.GetSize(); i++) vc.push_back(curves[i]);
        std::vector<py::tuple> vp;
        for (ULONG i = 0; i < pts.GetSize(); i++)
            vp.push_back(py::make_tuple(pts[i].x, pts[i].y, pts[i].z));
        return py::make_tuple(vc, vp);
    }, py::arg("curve"), py::arg("brep"),
       "Compute the intersection of a curve with a Brep.\n"
       "\n"
       "Args:\n"
       "    curve: Input curve. Not modified.\n"
       "    brep: Input Brep. Not modified.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[Curve], list[tuple[float, float, float]]]:\n"
       "    ``(curves, points)``. ``curves`` cover any segments where the\n"
       "    input curve lies on a face of the Brep; ``points`` are the\n"
       "    isolated entry/exit and tangent points.\n"
       "\n"
       "Notes:\n"
       "    Tests against every trimmed face of the Brep. For a single\n"
       "    face use ``intersect_curve_face``.\n"
       "\n"
       "See Also:\n"
       "    intersect_curve_face: Single-face variant.\n"
       "    intersect_curve_surface: Untrimmed-surface variant.\n"
       "\n"
       "Wraps: SmApiIntersectCurveBrep");

    m.def("intersect_curve_face", [](SmCurve* curve, SmFace* face) {
        SmTArray<SmCurve*> curves;
        SmTArray<SmPoint3d> pts;
        CHECK_STATUS(SmApiIntersectCurveFace(curve, face, curves, pts));
        std::vector<SmCurve*> vc;
        for (ULONG i = 0; i < curves.GetSize(); i++) vc.push_back(curves[i]);
        std::vector<py::tuple> vp;
        for (ULONG i = 0; i < pts.GetSize(); i++)
            vp.push_back(py::make_tuple(pts[i].x, pts[i].y, pts[i].z));
        return py::make_tuple(vc, vp);
    }, py::arg("curve"), py::arg("face"),
       "Compute the intersection of a curve with a single trimmed face.\n"
       "\n"
       "Args:\n"
       "    curve: Input curve. Not modified.\n"
       "    face: Input trimmed face. Not modified.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[Curve], list[tuple[float, float, float]]]:\n"
       "    ``(curves, points)``. The face trim is honored, so only\n"
       "    crossings that fall inside the face's trimming loops are\n"
       "    reported.\n"
       "\n"
       "See Also:\n"
       "    intersect_curve_surface: Untrimmed-surface variant.\n"
       "    intersect_curve_brep: Whole-Brep variant.\n"
       "\n"
       "Wraps: SmApiIntersectCurveFace");
}
