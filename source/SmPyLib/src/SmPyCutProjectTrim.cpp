// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiBrep.h (SmApiCut, SmApiProjectBrepOntoPlane, SmApiProjectCurve,
// SmApiProjectAndTrim, SmApiCreateSilhouetteCurves),
// SmApiTrimmedSurfaces.h (SmApiTrimSurfaceWith3dCurves,
// SmApiTrimProjectParallel).
//
// SmApiCreatePlanarFaces lives in SmPySurfaces.cpp (it constructs a Brep
// from planar curve loops, which is a creation op rather than a cut/trim).

#include "SmPyCommon.h"

#include <SmApiBrep.h>
#include <SmApiTrimmedSurfaces.h>

void bind_cut_project_trim(BoundModule& m)
{
    m.def("cut", [](SmBrep* brep, py::tuple planePt, py::tuple planeNorm) {
        SmVector3d pt = to_vec(planePt);
        SmVector3d nm = to_vec(planeNorm);
        CHECK_STATUS(SmApiCut(brep, pt, nm));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("plane_point"), py::arg("plane_normal"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Cut a Brep with a plane, keeping the half-space on the positive normal side.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to cut in place. The removed half is discarded.\n"
       "    plane_point: Any point on the cutting plane as ``(x, y, z)``.\n"
       "    plane_normal: Plane normal as ``(dx, dy, dz)``. The half-space\n"
       "        in the direction of this normal is kept; flip the normal to\n"
       "        keep the other half. Must be non-zero; does not need to be\n"
       "        unit-length.\n"
       "\n"
       "Notes:\n"
       "    A new planar face is created at the cut. If the plane does not\n"
       "    intersect the Brep, the Brep is unchanged. For a non-destructive\n"
       "    section curve, use ``intersect_brep_with_plane``.\n"
       "\n"
       "See Also:\n"
       "    intersect_brep_with_plane: Non-destructive section curves.\n"
       "    project_and_trim: Trim a Brep using a projected curve loop.\n"
       "    trim_project_parallel: Trim faces with parallel curve projection.\n"
       "\n"
       "Wraps: SmApiCut");

    m.def("project_brep_onto_plane", [](SmBrep* brep, py::tuple planePt, py::tuple planeNorm) {
        SmVector3d pt = to_vec(planePt);
        SmVector3d nm = to_vec(planeNorm);
        SmTArray<SmCurve*> curves;
        CHECK_STATUS(SmApiProjectBrepOntoPlane(brep, pt, nm, curves));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < curves.GetSize(); i++) v.push_back(curves[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("plane_point"), py::arg("plane_normal"),
       "Pure: returns new list[Curve]; brep unchanged.\n"
       "\n"
       "Project the edges of a Brep onto a plane.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to project. Not modified.\n"
       "    plane_point: Any point on the projection plane as ``(x, y, z)``.\n"
       "    plane_normal: Plane normal as ``(dx, dy, dz)``. Projection runs\n"
       "        along this direction. Must be non-zero; does not need to be\n"
       "        unit-length.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: outline curves on the plane.\n"
       "\n"
       "Notes:\n"
       "    Useful for 2D drawings, shadow projections, and silhouette-style\n"
       "    outlines. Non-destructive query.\n"
       "\n"
       "See Also:\n"
       "    create_silhouette_curves: Silhouette of curved surfaces (not\n"
       "        just edges).\n"
       "    intersect_brep_with_plane: Section curves at the plane instead\n"
       "        of projection onto it.\n"
       "\n"
       "Wraps: SmApiProjectBrepOntoPlane");

    m.def("project_and_trim", [](SmBrep* brep, SmBSplineCurve* curve,
                                 py::tuple projDir, py::tuple refPt) {
        SmVector3d d = to_vec(projDir);
        SmPoint3d rp = to_point(refPt);
        CHECK_STATUS(SmApiProjectAndTrim(brep, curve, d, rp));
        return brep;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("curve"), py::arg("projection_dir"), py::arg("ref_point"),
       "Mutates: brep; returns the same handle for chaining.\n"
       "\n"
       "Project a curve onto a Brep and trim using the projected loop.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to trim in place.\n"
       "    curve: B-spline curve to project. Typically a closed loop.\n"
       "    projection_dir: Projection direction as ``(dx, dy, dz)``. Must be\n"
       "        non-zero; does not need to be unit-length.\n"
       "    ref_point: A point in 3D as ``(x, y, z)`` whose projection onto\n"
       "        the Brep lies on the side that should be kept.\n"
       "\n"
       "Notes:\n"
       "    The kernel projects ``curve`` along ``projection_dir`` onto the\n"
       "    Brep faces, then trims away the side that does not contain the\n"
       "    projection of ``ref_point``.\n"
       "\n"
       "See Also:\n"
       "    cut: Plane-based version (no curve projection).\n"
       "    trim_project_parallel: More general variant with split / keep /\n"
       "        delete behavior selected by ``TrimType``.\n"
       "    project_curve: Non-destructive curve-onto-Brep projection.\n"
       "\n"
       "Wraps: SmApiProjectAndTrim");

    m.def("create_silhouette_curves", [](SmBrep* brep, py::tuple planePt, py::tuple planeNorm) {
        SmVector3d pt = to_vec(planePt);
        SmVector3d nm = to_vec(planeNorm);
        SmTArray<SmCurve*> curves;
        CHECK_STATUS(SmApiCreateSilhouetteCurves(brep, pt, nm, curves));
        std::vector<SmCurve*> v;
        for (ULONG i = 0; i < curves.GetSize(); i++) v.push_back(curves[i]);
        return v;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("plane_point"), py::arg("plane_normal"),
       "Pure: returns new list[Curve]; brep unchanged.\n"
       "\n"
       "Compute silhouette curves of a Brep as seen from a viewing plane.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "    plane_point: Unused; the silhouette depends only on\n"
       "        ``plane_normal``. Accepted as ``(x, y, z)`` for compatibility.\n"
       "    plane_normal: Plane normal as ``(dx, dy, dz)``, interpreted as\n"
       "        the viewing direction. Must be non-zero; does not need to be\n"
       "        unit-length.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: silhouette curves on the Brep where the surface\n"
       "    normal is perpendicular to the viewing direction.\n"
       "\n"
       "Notes:\n"
       "    Stability: this API is currently known to crash inside the kernel\n"
       "    on some inputs. Differs from ``project_brep_onto_plane`` in that\n"
       "    silhouettes pick up the curved-surface outline (e.g. the\n"
       "    apparent edge of a sphere), not just the existing topological\n"
       "    edges.\n"
       "\n"
       "See Also:\n"
       "    project_brep_onto_plane: Edge-only projection variant.\n"
       "\n"
       "Wraps: SmApiCreateSilhouetteCurves");

    m.def("project_curve", [](SmBrep* brep, SmBSplineCurve* curve, py::tuple projVec) {
        SmVector3d pv = to_vec(projVec);
        SmTArray<SmCurve*> curves3d;
        CHECK_STATUS(SmApiProjectCurve(brep, curve, pv, curves3d));
        std::vector<SmCurve*> vc;
        for (ULONG i = 0; i < curves3d.GetSize(); i++) vc.push_back(curves3d[i]);
        return vc;
    }, py::arg("brep"), py::arg("curve"), py::arg("projection_vector"),
       "Pure: returns new list[Curve]; brep unchanged.\n"
       "\n"
       "Project a curve onto the faces of a Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Target Brep. Not modified.\n"
       "    curve: B-spline curve to project.\n"
       "    projection_vector: Projection direction as ``(dx, dy, dz)``.\n"
       "        Magnitude is ignored.\n"
       "\n"
       "Returns:\n"
       "    list[Curve]: 3D projected curves on the Brep faces.\n"
       "\n"
       "Notes:\n"
       "    Non-destructive query - the Brep is not modified. Wraps the\n"
       "    Brep variant of ``SmApiProjectCurve`` (declared in\n"
       "    ``SmApiBrep.h``); the curve-onto-single-surface variant from\n"
       "    ``SmApiCurves.h`` is exposed separately as\n"
       "    ``project_curve_to_surface``.\n"
       "\n"
       "See Also:\n"
       "    project_curve_to_surface: Single-surface (uv + 3D) variant.\n"
       "    project_and_trim: Project and use the result to trim the Brep.\n"
       "    drop_curve_to_surface: Curve drop / trim onto a surface.\n"
       "\n"
       "Wraps: SmApiProjectCurve (Brep overload)");

    m.def("trim_surface_with_3d_curves", [](SmSurface* srf,
                                            std::vector<unsigned long>& curveLoops,
                                            std::vector<SmCurve*>& curves) {
        SmTArray<ULONG> loops;
        SmTArray<SmCurve*> carr;
        for (auto v : curveLoops) loops.Add(v);
        for (auto* c : curves) carr.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiTrimSurfaceWith3dCurves(srf, loops, carr, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("surface"), py::arg("curve_loops"), py::arg("curves"),
       "Consumes: surface, curves; returns new Brep.\n"
       "\n"
       "Trim a standalone surface with 3D curve loops to produce a Brep face body.\n"
       "\n"
       "Args:\n"
       "    surface: Untrimmed surface to trim. Consumed by the call, even\n"
       "        if it raises.\n"
       "    curve_loops: List giving the count of curves in each loop. The\n"
       "        sum of all entries must equal ``len(curves)``. For example,\n"
       "        ``[4, 3]`` means the first four curves form the outer loop\n"
       "        and the next three form a hole.\n"
       "    curves: Trimming curves in head-to-tail order, grouped\n"
       "        loop-by-loop according to ``curve_loops``. Each loop must\n"
       "        be closed and lie on (or near) ``surface``. Consumed by the\n"
       "        call, even if it raises: on failure they may already be\n"
       "        deleted. Do not reuse them.\n"
       "\n"
       "Returns:\n"
       "    Brep: sheet body whose faces are the surface clipped to the\n"
       "    supplied loops.\n"
       "\n"
       "See Also:\n"
       "    create_planar_faces: Build a planar Brep from coplanar curve\n"
       "        loops without supplying a surface.\n"
       "    trim_project_parallel: Trim by projecting a curve along a\n"
       "        direction.\n"
       "\n"
       "Wraps: SmApiTrimSurfaceWith3dCurves");

    m.def("trim_project_parallel", [](SmObject* obj, SmBSplineCurve* curve,
                                      py::tuple projDir, py::tuple refPt,
                                      SmTrimType trimType) {
        SmVector3d d = to_vec(projDir);
        SmPoint3d rp = to_point(refPt);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiTrimProjectParallel(obj, curve, d, rp, trimType, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("object"), py::arg("curve"), py::arg("projection_dir"),
       py::arg("ref_point"), py::arg("trim_type") = SM_TT_KEEP_POINT,
       "Consumes: object; returns new Brep.\n"
       "\n"
       "Project a curve onto an object's faces and trim, keeping or splitting per option.\n"
       "\n"
       "Args:\n"
       "    object: Brep or standalone surface to trim. Consumed on\n"
       "        success; see Notes for failure.\n"
       "    curve: B-spline curve to project. Typically a closed loop.\n"
       "    projection_dir: Projection direction as ``(dx, dy, dz)``.\n"
       "    ref_point: Reference point as ``(x, y, z)``. Used only when\n"
       "        ``trim_type`` is ``KEEP_POINT`` or ``DELETE_POINT``;\n"
       "        ignored for ``SPLIT``.\n"
       "    trim_type: ``TrimType`` selector controlling what to do with\n"
       "        the two sides of the projected loop:\n"
       "\n"
       "        - ``KEEP_POINT`` (default): keep the side containing the\n"
       "          projection of ``ref_point``.\n"
       "        - ``DELETE_POINT``: delete the side containing the\n"
       "          projection of ``ref_point``.\n"
       "        - ``SPLIT``: keep both sides as separate faces.\n"
       "\n"
       "Returns:\n"
       "    Brep: trimmed (or split) result.\n"
       "\n"
       "Notes:\n"
       "    On failure the call raises and ``object`` is not consumed. A\n"
       "    Brep stays with the caller but may be partially trimmed. A\n"
       "    standalone surface rejected up front (for example a zero\n"
       "    ``projection_dir``) is left untouched; if the kernel trim itself\n"
       "    fails, the surface stays attached to an internal Brep that is\n"
       "    not freed.\n"
       "\n"
       "See Also:\n"
       "    project_and_trim: Brep-only convenience that always keeps the\n"
       "        side containing ``ref_point``.\n"
       "    cut: Plane-based trim with no curve projection.\n"
       "\n"
       "Wraps: SmApiTrimProjectParallel");
}
