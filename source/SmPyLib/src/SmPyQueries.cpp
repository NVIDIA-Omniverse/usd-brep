// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiQueries.h (closest-point on Brep/Curve/Surface, including
// all-solution variants for Brep and Curve, SmApiGetClosestPoint,
// SmApiFindEdge, SmApiFindFaces,
// SmApiGetFaces, SmApiGetEdges, SmApiBrepIsManifoldSolid,
// SmApiBrepComputeVolume, SmApiBrepBoundingBox, SmApiBrepCopy,
// SmApiFaceBoundingBox, SmApiFaceComputeArea, SmApiEdgeBoundingBox,
// SmApiEdgeComputeLength, SmApiVertexGetPoint), SmApiSurfaces.h
// (SmApiEvaluateSurface{Point,Normal,Derivatives}).

#include "SmPyCommon.h"

#include <SmApiSurfaces.h>
#include <SmApiQueries.h>

void bind_queries(BoundModule& m)
{
    m.def("brep_closest_point", [](SmBrep* brep, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmPoint3d closest;
        double dist = 0;
        CHECK_STATUS(SmApiBrepClosestPoint(brep, p, closest, dist));
        return py::make_tuple(closest, dist);
    }, py::arg("brep"), py::arg("point"),
       "Find the globally closest point on a Brep to a query point.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "    point: Query point as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    tuple[Point3d, float]: ``(closest_point, distance)`` where\n"
       "    ``closest_point`` lies on the Brep surface and ``distance`` is\n"
       "    the Euclidean distance from ``point`` to ``closest_point``.\n"
       "\n"
       "Notes:\n"
       "    Returns one global minimum. For all globally closest solutions\n"
       "    within solver tolerance use\n"
       "    ``brep_closest_point_all``.\n"
       "\n"
       "See Also:\n"
       "    brep_closest_point_all: All globally closest solutions.\n"
       "    get_closest_point: Topology-driven variant returning a plain\n"
       "        ``(x, y, z)`` tuple.\n"
       "    curve_closest_point: Curve variant.\n"
       "    surface_closest_point: Surface variant.\n"
       "\n"
       "Wraps: SmApiBrepClosestPoint");

    m.def("curve_closest_point", [](SmCurve* curve, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmPoint3d closest;
        double param = 0, dist = 0;
        CHECK_STATUS(SmApiCurveClosestPoint(curve, p, closest, param, dist));
        return py::make_tuple(closest, param, dist);
    }, py::arg("curve"), py::arg("point"),
       "Find the globally closest point on a curve to a query point.\n"
       "\n"
       "Args:\n"
       "    curve: Curve to query. Not modified.\n"
       "    point: Query point as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    tuple[Point3d, float, float]: ``(closest_point, parameter,\n"
       "    distance)`` where ``parameter`` is the curve parameter at\n"
       "    ``closest_point``.\n"
       "\n"
       "See Also:\n"
       "    curve_closest_point_all: All globally closest solutions.\n"
       "    brep_closest_point: Brep variant.\n"
       "    surface_closest_point: Surface variant.\n"
       "    evaluate_curve: Evaluate the curve at a known parameter.\n"
       "\n"
       "Wraps: SmApiCurveClosestPoint");

    m.def("surface_closest_point", [](SmSurface* srf, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmPoint3d closest;
        SmTArray<SmPoint2d> uvs;
        double dist = 0;
        CHECK_STATUS(SmApiSurfaceClosestPoint(srf, p, closest, uvs, dist));

        std::vector<SmPoint2d> uvRepresentations;
        uvRepresentations.reserve(uvs.GetSize());
        for (ULONG i = 0; i < uvs.GetSize(); i++)
            uvRepresentations.push_back(uvs[i]);

        return py::make_tuple(closest, uvRepresentations, dist);
    }, py::arg("surface"), py::arg("point"),
       "Find a globally closest point and its known UV representations.\n"
       "\n"
       "Args:\n"
       "    surface: Surface to query. Not modified.\n"
       "    point: Query point as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    tuple[Vec3, list[Vec2], float]: ``(closest_point, uvs, distance)``\n"
       "    where every element of ``uvs`` is a known natural/NURBS\n"
       "    parameter representation of ``closest_point``; evaluate one\n"
       "    with ``surface.evaluate(uv.x, uv.y)``.\n"
       "\n"
       "Notes:\n"
       "    ``uvs`` includes periodic-seam aliases reported by the active\n"
       "    solver. The function does not guarantee discovery of every\n"
       "    spatially distinct tied minimum or every preimage on a surface\n"
       "    that folds over itself.\n"
       "\n"
       "See Also:\n"
       "    brep_closest_point: Brep variant.\n"
       "    curve_closest_point: Curve variant.\n"
       "    evaluate_surface_point: Evaluate at a known ``(u, v)``.\n"
       "\n"
       "Wraps: SmApiSurfaceClosestPoint");

    m.def("is_manifold_solid", [](SmBrep* brep) {
        SmBoolean bResult = FALSE;
        CHECK_STATUS(SmApiBrepIsManifoldSolid(brep, bResult));
        return (bool)bResult;
    }, py::arg("brep"),
       "Check whether a Brep is a closed manifold solid.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to validate. Not modified.\n"
       "\n"
       "Returns:\n"
       "    bool: ``True`` if every edge is shared by exactly two faces\n"
       "    (no lamina edges, no non-manifold junctions); ``False``\n"
       "    otherwise.\n"
       "\n"
       "Notes:\n"
       "    Sheet bodies (open shells, planar circles, etc.) return\n"
       "    ``False``. A ``False`` result is not necessarily an error -\n"
       "    the Brep may be intentionally a sheet body. Use as a\n"
       "    post-condition for booleans, sweeps, or stitching that should\n"
       "    have produced solids.\n"
       "\n"
       "See Also:\n"
       "    compute_volume: Requires a manifold solid to be meaningful.\n"
       "    stitch_into_solid: Recover manifold topology from loose faces.\n"
       "\n"
       "Wraps: SmApiBrepIsManifoldSolid");

    m.def("material_census", [](SmBrep* brep) {
        long solid_count = 0;
        long void_count = 0;
        CHECK_STATUS(SmApiBrepMaterialCensus(brep, solid_count, void_count));
        py::dict result;
        result["solid_count"] = solid_count;
        result["void_count"] = void_count;
        return result;
    }, py::arg("brep"),
       "Count a Brep's material (solid) regions and enclosed void cavities.\n"
       "\n"
       "SMLib stores every connected 3D region as its own region, so nested\n"
       "shells are counted unambiguously: a solid with a hollow cavity that\n"
       "itself contains a nested solid reports ``solid_count == 2`` and\n"
       "``void_count == 1``. A single simple solid is ``solid_count == 1``,\n"
       "``void_count == 0``.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "\n"
       "Returns:\n"
       "    dict: ``{'solid_count': int, 'void_count': int}``.\n"
       "    ``solid_count`` is the number of material (solid) regions.\n"
       "    ``void_count`` is the number of enclosed void cavities and\n"
       "    excludes the single infinite (unbounded) region.\n"
       "\n"
       "Notes:\n"
       "    The counts reflect the Brep's current void flags, which SMLib\n"
       "    construction and boolean operations set and a correct import\n"
       "    preserves. Only a malformed Brep with unset flags needs them\n"
       "    re-established (FindAndSetInfiniteRegion +\n"
       "    SetRegionIsVoidFlagsForNestedSolids).\n"
       "\n"
       "See Also:\n"
       "    is_manifold_solid: Whether the Brep is a closed manifold solid.\n"
       "    compute_volume: Enclosed volume (cavities subtracted).\n"
       "\n"
       "Wraps: SmApiBrepMaterialCensus");

    m.def("compute_volume", [](SmBrep* brep, double accuracy) {
        double vol = 0;
        CHECK_STATUS(SmApiBrepComputeVolume(brep, accuracy, vol));
        return vol;
    }, py::arg("brep"), py::arg("relative_accuracy") = 1.0e-3,
       "Compute the enclosed volume of a manifold-solid Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Manifold-solid Brep. Behaviour is undefined for sheet\n"
       "        bodies (open shells); validate with ``is_manifold_solid``\n"
       "        first if uncertain.\n"
       "    relative_accuracy: Numerical-integration accuracy. Must be\n"
       "        between ``1.0e-1`` (coarse, fast) and ``1.0e-8`` (fine,\n"
       "        slow). Defaults to ``1.0e-3``; non-finite values raise\n"
       "        ``RuntimeError``.\n"
       "\n"
       "Returns:\n"
       "    float: enclosed volume in cubic modeling units.\n"
       "\n"
       "Notes:\n"
       "    Analytical primitives (boxes, spheres, cones) integrate to the\n"
       "    closed-form value at any accuracy. Free-form NURBS bodies\n"
       "    benefit from tighter accuracy.\n"
       "\n"
       "See Also:\n"
       "    is_manifold_solid: Pre-check for a meaningful volume.\n"
       "\n"
       "Wraps: SmApiBrepComputeVolume");

    m.def("get_closest_point", [](SmBrep* brep, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmPoint3d closest;
        double dist = 0;
        CHECK_STATUS(SmApiGetClosestPoint(brep, p, closest, dist));
        return py::make_tuple(
            py::make_tuple(closest.x, closest.y, closest.z), dist);
    }, py::arg("brep"), py::arg("point"),
       "Find the closest point on a Brep using the topology-driven solver.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "    point: Query point as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    tuple[tuple[float, float, float], float]: ``((x, y, z),\n"
       "    distance)``. The point is returned as a plain Python tuple\n"
       "    rather than a ``Point3d``.\n"
       "\n"
       "Notes:\n"
       "    Same computation as ``brep_closest_point``: both wrappers call\n"
       "    the same kernel point solve with the same options. The only\n"
       "    difference is that this one returns the point as a plain tuple;\n"
       "    it is kept for compatibility. Prefer ``brep_closest_point`` for\n"
       "    a ``Point3d`` return.\n"
       "\n"
       "See Also:\n"
       "    brep_closest_point: Closest-point with ``Point3d`` return.\n"
       "    brep_closest_point_all: All globally closest solutions.\n"
       "\n"
       "Wraps: SmApiGetClosestPoint");

    m.def("brep_relationship", [](SmBrep* a, SmBrep* b) {
        int rel = 0;
        double dist = 0.0;
        CHECK_STATUS(SmApiBrepRelationship(a, b, rel, dist));
        static const char* kNames[] = {
            "separate", "touching", "interpenetrating", "a_contains_b", "b_contains_a" };
        const char* name = (rel >= 0 && rel <= 4) ? kNames[rel] : "unknown";
        py::dict d;
        d["relationship"] = name;
        d["distance"] = dist;
        return d;
    }, py::arg("a"), py::arg("b"),
       "Classify how two Breps are arranged, with a distance.\n"
       "\n"
       "Args:\n"
       "    a: First Brep.\n"
       "    b: Second Brep.\n"
       "\n"
       "Returns:\n"
       "    dict: ``{'relationship': str, 'distance': float}``. ``relationship``\n"
       "        is one of ``'separate'``, ``'touching'``,\n"
       "        ``'interpenetrating'``, ``'a_contains_b'``, ``'b_contains_a'``.\n"
       "        ``distance`` is the surface gap between the two surfaces -- the\n"
       "        wall clearance when one contains the other -- and ``0.0`` when\n"
       "        they touch or interpenetrate.\n"
       "\n"
       "Note:\n"
       "    Overlap/containment is decided by classifying sample points -- each\n"
       "    Brep's vertices plus one interior point per face -- against the\n"
       "    other. A shallow or sliver overlap containing none of those points\n"
       "    can still read as ``'touching'``; exact for typical CAD interference.\n"
       "    Use ``brep_distance`` for just the distance number.\n"
       "\n"
       "Wraps: SmApiBrepRelationship");

    m.def("brep_distance", [](SmBrep* a, SmBrep* b) {
        double dist = 0.0;
        CHECK_STATUS(SmApiBrepDistance(a, b, dist));
        return dist;
    }, py::arg("a"), py::arg("b"),
       "Distance between two Breps' boundary surfaces.\n"
       "\n"
       "Args:\n"
       "    a: First Brep.\n"
       "    b: Second Brep.\n"
       "\n"
       "Returns:\n"
       "    float: gap between the two surfaces when the Breps are disjoint --\n"
       "        the wall clearance when one is nested inside the other -- and\n"
       "        ``0.0`` when they touch or interpenetrate. Equals the\n"
       "        ``distance`` field of ``brep_relationship``; use that when the\n"
       "        number needs the arrangement to be meaningful.\n"
       "\n"
       "Wraps: SmApiBrepDistance");

    m.def("find_edge", [](SmBrep* brep, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        ULONG edgeIndex = 0;
        double param = 0;
        CHECK_STATUS(SmApiFindEdge(brep, p, edgeIndex, param));
        return py::make_tuple(edgeIndex, param);
    }, py::arg("brep"), py::arg("point"),
       "Locate the edge of a Brep nearest to a point and the parameter on it.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to search. Not modified.\n"
       "    point: Query point as ``(x, y, z)``. Should be on or near an\n"
       "        edge for a meaningful result.\n"
       "\n"
       "Returns:\n"
       "    tuple[int, float]: ``(edge_index, parameter)`` where\n"
       "    ``edge_index`` is the index into the Brep's internal edge\n"
       "    list and ``parameter`` is the curve parameter on that edge\n"
       "    closest to ``point``.\n"
       "\n"
       "Notes:\n"
       "    There is no distance limit: a point far from every edge still\n"
       "    returns the nearest edge. Check the distance, for example by\n"
       "    evaluating the edge at ``parameter``, if ``point`` may be off\n"
       "    the Brep.\n"
       "\n"
       "    Use the index against ``get_edges_via_api(brep)`` (or the\n"
       "    class-method ``brep.edges()``) to recover the ``Edge`` handle.\n"
       "\n"
       "See Also:\n"
       "    find_face: Face-locating counterpart.\n"
       "    get_edges_via_api: Resolve the index to an ``Edge``.\n"
       "    curve_closest_point: Closest-point on an isolated curve.\n"
       "\n"
       "Wraps: SmApiFindEdge");

    m.def("find_face", [](SmBrep* brep, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmFace* face = nullptr;
        CHECK_STATUS(SmApiFindFaces(brep, p, face));
        return face;
    }, py::return_value_policy::reference,
       py::arg("brep"), py::arg("point"),
       "Locate the face of a Brep nearest to a point.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to search. Not modified.\n"
       "    point: Query point as ``(x, y, z)``. Should be on or near a\n"
       "        face for a meaningful result.\n"
       "\n"
       "Returns:\n"
       "    Face: the located face. Owned by ``brep`` - do not delete.\n"
       "\n"
       "Notes:\n"
       "    Wraps the kernel entry point named ``SmApiFindFaces`` (plural)\n"
       "    despite returning a single face - the kernel name reflects\n"
       "    that an early version returned several candidates.\n"
       "\n"
       "See Also:\n"
       "    find_edge: Edge-locating counterpart.\n"
       "    surface_closest_point: Closest-point on an isolated surface.\n"
       "\n"
       "Wraps: SmApiFindFaces");

    m.def("get_faces_via_api", [](SmBrep* brep) {
        SmTArray<SmFace*> f;
        CHECK_STATUS(SmApiGetFaces(brep, f));
        std::vector<SmFace*> v;
        for (ULONG i = 0; i < f.GetSize(); i++) v.push_back(f[i]);
        return v;
    }, py::return_value_policy::reference, py::arg("brep"),
       "Return all faces of a Brep using the SmAPI helper.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "\n"
       "Returns:\n"
       "    list[Face]: faces in the Brep's internal order.\n"
       "\n"
       "Notes:\n"
       "    Functionally equivalent to the class method ``brep.faces()``.\n"
       "    The ``_via_api`` suffix disambiguates the SmAPI entry point\n"
       "    (``SmApiGetFaces``) from the class accessor and from the\n"
       "    ``PolyBrep.get_faces`` method on tessellated meshes.\n"
       "\n"
       "See Also:\n"
       "    get_edges_via_api: Edge counterpart.\n"
       "    find_face: Locate a face by a 3D point.\n"
       "\n"
       "Wraps: SmApiGetFaces");

    m.def("get_edges_via_api", [](SmBrep* brep) {
        SmTArray<SmEdge*> e;
        CHECK_STATUS(SmApiGetEdges(brep, e));
        std::vector<SmEdge*> v;
        for (ULONG i = 0; i < e.GetSize(); i++) v.push_back(e[i]);
        return v;
    }, py::return_value_policy::reference, py::arg("brep"),
       "Return all edges of a Brep using the SmAPI helper.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "\n"
       "Returns:\n"
       "    list[Edge]: edges in the Brep's internal order.\n"
       "\n"
       "Notes:\n"
       "    Functionally equivalent to the class method ``brep.edges()``.\n"
       "    The ``_via_api`` suffix disambiguates the SmAPI entry point\n"
       "    (``SmApiGetEdges``) from the class accessor and from the\n"
       "    ``PolyBrep.get_edges`` method on tessellated meshes.\n"
       "\n"
       "See Also:\n"
       "    get_faces_via_api: Face counterpart.\n"
       "    find_edge: Locate an edge by a 3D point.\n"
       "\n"
       "Wraps: SmApiGetEdges");

    m.def("brep_closest_point_all", [](SmBrep* brep, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmTArray<SmPoint3d> closestPts;
        SmTArray<double> dists;
        CHECK_STATUS(SmApiBrepClosestPointAll(brep, p, closestPts, dists));
        std::vector<py::tuple> vp;
        std::vector<double> vd;
        for (ULONG i = 0; i < closestPts.GetSize(); i++)
            vp.push_back(py::make_tuple(closestPts[i].x, closestPts[i].y, closestPts[i].z));
        for (ULONG i = 0; i < dists.GetSize(); i++) vd.push_back(dists[i]);
        return py::make_tuple(vp, vd);
    }, py::arg("brep"), py::arg("point"),
       "Find all globally closest points on a Brep within solver tolerance.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "    point: Query point as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[tuple[float, float, float]], list[float]]:\n"
       "    ``(points, distances)`` where the lists are aligned. Element\n"
       "    ``i`` is a closest-point candidate and the corresponding\n"
       "    distance from ``point``.\n"
       "\n"
       "Notes:\n"
       "    Useful when geometry has multiple equidistant or near-tied\n"
       "    closest points (e.g. a point inside a sphere returns two\n"
       "    antipodal candidates). For just the global minimum use\n"
       "    ``brep_closest_point``.\n"
       "\n"
       "See Also:\n"
       "    brep_closest_point: Global-minimum variant.\n"
       "    curve_closest_point_all: Curve variant.\n"
       "    surface_closest_point: Surface variant returning one point and\n"
       "        its known UV representations.\n"
       "\n"
       "Wraps: SmApiBrepClosestPointAll");

    m.def("curve_closest_point_all", [](SmCurve* curve, py::tuple pt) {
        SmPoint3d p = to_point(pt);
        SmTArray<SmPoint3d> closestPts;
        SmTArray<double> params, dists;
        CHECK_STATUS(SmApiCurveClosestPointAll(curve, p, closestPts, params, dists));
        std::vector<py::tuple> vp;
        std::vector<double> vpar, vd;
        for (ULONG i = 0; i < closestPts.GetSize(); i++)
            vp.push_back(py::make_tuple(closestPts[i].x, closestPts[i].y, closestPts[i].z));
        for (ULONG i = 0; i < params.GetSize(); i++) vpar.push_back(params[i]);
        for (ULONG i = 0; i < dists.GetSize(); i++) vd.push_back(dists[i]);
        return py::make_tuple(vp, vpar, vd);
    }, py::arg("curve"), py::arg("point"),
       "Find all globally closest points on a curve within solver tolerance.\n"
       "\n"
       "Args:\n"
       "    curve: Curve to query. Not modified.\n"
       "    point: Query point as ``(x, y, z)``.\n"
       "\n"
       "Returns:\n"
       "    tuple[list[tuple[float, float, float]], list[float], list[float]]:\n"
       "    ``(points, parameters, distances)`` where the three lists are\n"
       "    aligned by index.\n"
       "\n"
       "See Also:\n"
       "    curve_closest_point: Global-minimum variant.\n"
       "    brep_closest_point_all: Brep variant.\n"
       "\n"
       "Wraps: SmApiCurveClosestPointAll");

    m.def("evaluate_surface_point", [](SmSurface* srf, double u, double v) {
        SmVector2d uv(u, v);
        SmVector3d pt;
        CHECK_STATUS(SmApiEvaluateSurfacePoint(srf, uv, pt));
        return py::make_tuple(pt.x, pt.y, pt.z);
    }, py::arg("surface"), py::arg("u"), py::arg("v"),
       "Evaluate the 3D position of a surface at a parametric ``(u, v)`` point.\n"
       "\n"
       "Args:\n"
       "    surface: Surface to evaluate. Not modified.\n"
       "    u: Parametric U coordinate. Must lie in the surface's U range.\n"
       "    v: Parametric V coordinate. Must lie in the surface's V range.\n"
       "\n"
       "Returns:\n"
       "    tuple[float, float, float]: 3D point ``(x, y, z)`` on the surface.\n"
       "\n"
       "See Also:\n"
       "    evaluate_surface_normal: Surface normal at the same point.\n"
       "    evaluate_surface_derivatives: Point + first partials.\n"
       "    surface_closest_point: Inverse query (3D point -> ``(u, v)``).\n"
       "\n"
       "Wraps: SmApiEvaluateSurfacePoint");

    m.def("evaluate_surface_normal", [](SmSurface* srf, double u, double v) {
        SmVector2d uv(u, v);
        SmVector3d nm;
        CHECK_STATUS(SmApiEvaluateSurfaceNormal(srf, uv, nm));
        return py::make_tuple(nm.x, nm.y, nm.z);
    }, py::arg("surface"), py::arg("u"), py::arg("v"),
       "Evaluate the surface normal at a parametric ``(u, v)`` point.\n"
       "\n"
       "Args:\n"
       "    surface: Surface to evaluate. Not modified.\n"
       "    u: Parametric U coordinate.\n"
       "    v: Parametric V coordinate.\n"
       "\n"
       "Returns:\n"
       "    tuple[float, float, float]: unit normal ``(nx, ny, nz)``.\n"
       "\n"
       "Notes:\n"
       "    The normal direction follows the surface's intrinsic\n"
       "    orientation (sign of ``du`` cross ``dv``); on a Brep face it\n"
       "    may be flipped relative to the face's outward normal.\n"
       "\n"
       "See Also:\n"
       "    evaluate_surface_point: 3D position at the same point.\n"
       "    evaluate_surface_derivatives: Underlying ``du``, ``dv``.\n"
       "\n"
       "Wraps: SmApiEvaluateSurfaceNormal");

    m.def("evaluate_surface_derivatives", [](SmSurface* srf, double u, double v) {
        SmVector2d uv(u, v);
        SmVector3d pt, du, dv;
        CHECK_STATUS(SmApiEvaluateSurfaceDerivatives(srf, uv, pt, du, dv));
        return py::make_tuple(
            py::make_tuple(pt.x, pt.y, pt.z),
            py::make_tuple(du.x, du.y, du.z),
            py::make_tuple(dv.x, dv.y, dv.z));
    }, py::arg("surface"), py::arg("u"), py::arg("v"),
       "Evaluate position and first partial derivatives at a parametric point.\n"
       "\n"
       "Args:\n"
       "    surface: Surface to evaluate. Not modified.\n"
       "    u: Parametric U coordinate.\n"
       "    v: Parametric V coordinate.\n"
       "\n"
       "Returns:\n"
       "    tuple[tuple[float, float, float], tuple[float, float, float],\n"
       "    tuple[float, float, float]]: ``((px, py, pz), (dux, duy, duz),\n"
       "    (dvx, dvy, dvz))`` - the 3D point, the partial derivative with\n"
       "    respect to U, and the partial derivative with respect to V.\n"
       "\n"
       "Notes:\n"
       "    The cross product ``du`` cross ``dv``, normalized, is the\n"
       "    surface normal at ``(u, v)`` and matches the result of\n"
       "    ``evaluate_surface_normal`` (up to sign).\n"
       "\n"
       "See Also:\n"
       "    evaluate_surface_point: 3D position alone.\n"
       "    evaluate_surface_normal: Normal alone.\n"
       "\n"
       "Wraps: SmApiEvaluateSurfaceDerivatives");

    // ========================================================================
    //  Geometric / topological properties (bbox, area, length, point, copy)
    // ========================================================================
    //
    // Flat aliases under sm.queries.*.  The same operations are also exposed
    // as methods on the type classes (Brep / Face / Edge / Vertex) in
    // SmPyTypes.cpp -- those are the primary user-facing surface; the flat
    // aliases here exist for symmetry with the rest of sm.queries.*.

    m.def("brep_bounding_box", [](SmBrep* brep, bool tight) {
        SmPoint3d mn, mx;
        CHECK_STATUS(SmApiBrepBoundingBox(brep, tight ? TRUE : FALSE, mn, mx));
        return py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
    }, py::arg("brep"), py::arg("tight") = false,
       "Axis-aligned bounding box of a Brep.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to query. Not modified.\n"
       "    tight: If ``True``, compute the minimal bounding box by\n"
       "        sampling each face's surface (expensive). Default ``False``\n"
       "        returns the loose union of vertex / edge / face boxes.\n"
       "\n"
       "Returns:\n"
       "    tuple[tuple[float, float, float], tuple[float, float, float]]:\n"
       "    ``((min_x, min_y, min_z), (max_x, max_y, max_z))``.\n"
       "\n"
       "See Also:\n"
       "    face_bounding_box: Single-face variant.\n"
       "    edge_bounding_box: Single-edge variant.\n"
       "    Brep.bounding_box: Method form on the Brep class.\n"
       "\n"
       "Wraps: SmApiBrepBoundingBox");

    m.def("brep_copy", [](SmBrep* brep) {
        SmBrep* result = nullptr;
        CHECK_STATUS(SmApiBrepCopy(brep, result));
        return result;
    }, py::return_value_policy::reference,
       py::arg("brep"),
       "Deep-copy a Brep into a fresh handle.\n"
       "\n"
       "Args:\n"
       "    brep: Source Brep. Not modified.\n"
       "\n"
       "Returns:\n"
       "    Brep: independent deep copy. Mutating the result does not\n"
       "    affect the source. The new handle is owned by the SMLib\n"
       "    kernel context (do not delete from Python).\n"
       "\n"
       "Notes:\n"
       "    Useful when you need to apply an in-place mutator (e.g.\n"
       "    ``fillet_edges``, ``cut``, ``translate``) without disturbing\n"
       "    the original Brep. Geometry types are preserved, including NURBS\n"
       "    produced by ``turn_to_nurbs``; no analytic recognition is performed.\n"
       "\n"
       "See Also:\n"
       "    Brep.copy: Method form on the Brep class.\n"
       "\n"
       "Wraps: SmApiBrepCopy");

    m.def("face_bounding_box", [](SmFace* face, bool tight) {
        SmPoint3d mn, mx;
        CHECK_STATUS(SmApiFaceBoundingBox(face, tight ? TRUE : FALSE, mn, mx));
        return py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
    }, py::arg("face"), py::arg("tight") = false,
       "Axis-aligned bounding box of a single face.\n"
       "\n"
       "Args:\n"
       "    face: Face to query. Not modified.\n"
       "    tight: If ``True``, sample the face's surface within its trim\n"
       "        boundary for a minimal box (expensive). Default ``False``\n"
       "        returns the loose face-level box.\n"
       "\n"
       "Returns:\n"
       "    tuple[tuple[float, float, float], tuple[float, float, float]]:\n"
       "    ``((min_x, min_y, min_z), (max_x, max_y, max_z))``.\n"
       "\n"
       "See Also:\n"
       "    Face.bounding_box: Method form on the Face class.\n"
       "    brep_bounding_box: Whole-Brep variant.\n"
       "\n"
       "Wraps: SmApiFaceBoundingBox");

    m.def("face_area", [](SmFace* face, double accuracy) {
        double area = 0.0;
        CHECK_STATUS(SmApiFaceComputeArea(face, accuracy, area));
        return area;
    }, py::arg("face"), py::arg("relative_accuracy") = 1.0e-3,
       "Surface area of a face within its trim boundary.\n"
       "\n"
       "Args:\n"
       "    face: Face to query. Not modified.\n"
       "    relative_accuracy: Numerical-integration accuracy in\n"
       "        ``[1e-4, 1e-1]``. Defaults to ``1e-3``. Out-of-range\n"
       "        values are clamped.\n"
       "\n"
       "Returns:\n"
       "    float: positive surface area in square modeling units. The\n"
       "    return is unsigned regardless of face orientation.\n"
       "\n"
       "See Also:\n"
       "    Face.area: Method form on the Face class.\n"
       "    compute_volume: Brep volume counterpart.\n"
       "\n"
       "Wraps: SmApiFaceComputeArea");

    m.def("edge_bounding_box", [](SmEdge* edge, bool tight) {
        SmPoint3d mn, mx;
        CHECK_STATUS(SmApiEdgeBoundingBox(edge, tight ? TRUE : FALSE, mn, mx));
        return py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
    }, py::arg("edge"), py::arg("tight") = false,
       "Axis-aligned bounding box of a single edge.\n"
       "\n"
       "Args:\n"
       "    edge: Edge to query. Not modified.\n"
       "    tight: If ``True``, evaluate the edge's curve at sample\n"
       "        points for a minimal box (expensive). Default ``False``\n"
       "        returns the loose curve-control-polygon box.\n"
       "\n"
       "Returns:\n"
       "    tuple[tuple[float, float, float], tuple[float, float, float]]:\n"
       "    ``((min_x, min_y, min_z), (max_x, max_y, max_z))``.\n"
       "\n"
       "See Also:\n"
       "    Edge.bounding_box: Method form on the Edge class.\n"
       "    brep_bounding_box: Whole-Brep variant.\n"
       "\n"
       "Wraps: SmApiEdgeBoundingBox");

    m.def("edge_length", [](SmEdge* edge, double accuracy) {
        double length = 0.0;
        CHECK_STATUS(SmApiEdgeComputeLength(edge, accuracy, length));
        return length;
    }, py::arg("edge"), py::arg("desired_accuracy") = 1.0e-6,
       "3D arc length of an edge over its parametric interval.\n"
       "\n"
       "Args:\n"
       "    edge: Edge to query. Not modified.\n"
       "    desired_accuracy: Absolute distance tolerance for the\n"
       "        arc-length integration. Defaults to ``1e-6`` modeling\n"
       "        units.\n"
       "\n"
       "Returns:\n"
       "    float: arc length in modeling units.\n"
       "\n"
       "See Also:\n"
       "    Edge.length: Method form on the Edge class.\n"
       "\n"
       "Wraps: SmApiEdgeComputeLength");

    m.def("vertex_point", [](SmVertex* vertex) {
        SmPoint3d pt;
        CHECK_STATUS(SmApiVertexGetPoint(vertex, pt));
        return point_to_tuple(pt);
    }, py::arg("vertex"),
       "3D position of a vertex.\n"
       "\n"
       "Args:\n"
       "    vertex: Vertex to query.\n"
       "\n"
       "Returns:\n"
       "    tuple[float, float, float]: ``(x, y, z)``.\n"
       "\n"
       "See Also:\n"
       "    Vertex.point: Method form on the Vertex class.\n"
       "\n"
       "Wraps: SmApiVertexGetPoint");
}
