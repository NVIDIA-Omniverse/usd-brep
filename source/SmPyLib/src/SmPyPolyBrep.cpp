// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmPoly.h (SmPolyBrep / SmPolyVertex / SmPolyEdge / SmPolyFace
// topology queries, mesh array extraction, ray intersection, bounding box).

#include "SmPyCommon.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include <pybind11/numpy.h>

#include <SmApiPolygons.h>
#include <SmSolutionArray.h>
#include <SmTools.h>

// ---------------------------------------------------------------------------
//  SmPolyBrep::RayIntersection packs hit data in SmSolution::m_vStart (see SmPoly.h).
// ---------------------------------------------------------------------------
namespace {

constexpr ULONG kPolyRayHitT = 0;
constexpr ULONG kPolyRayHitPx = 3;
constexpr ULONG kPolyRayHitPy = 4;
constexpr ULONG kPolyRayHitPz = 5;
constexpr ULONG kPolyRayHitNx = 6;
constexpr ULONG kPolyRayHitNy = 7;
constexpr ULONG kPolyRayHitNz = 8;

double poly_ray_hit_t(const SmSolution& sol)
{
    return sol.m_vStart[kPolyRayHitT];
}

py::tuple poly_ray_hit_point_tuple(const SmSolution& sol)
{
    return py::make_tuple(
        sol.m_vStart[kPolyRayHitPx],
        sol.m_vStart[kPolyRayHitPy],
        sol.m_vStart[kPolyRayHitPz]);
}

py::tuple poly_ray_hit_normal_tuple(const SmSolution& sol)
{
    return py::make_tuple(
        sol.m_vStart[kPolyRayHitNx],
        sol.m_vStart[kPolyRayHitNy],
        sol.m_vStart[kPolyRayHitNz]);
}

// Contiguous mesh buffers shared by ``to_mesh_arrays`` and ``to_numpy_arrays``.
// Face indices use int32 so both APIs stay within the same indexing contract.
struct MeshArrays
{
    std::vector<float> points;
    std::vector<float> normals;
    std::vector<int32_t> faces;
    std::vector<float> faceNormals;
};

SmVector3d unit_normal(const SmVector3d& normal, const SmVector3d& fallback)
{
    const double length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (!std::isfinite(length) || length <= 1e-30)
        return fallback;
    return normal / length;
}

SmVector3d vertex_normal(SmPolyVertex* vertex, SmFace* original)
{
    const auto& normals = vertex->GetNormalsRef();
    const auto& faces = vertex->GetFacesRef();
    SmVector3d average(0, 0, 0);
    for (ULONG i = 0; i < normals.GetSize(); ++i)
        average += normals[i];
    average = unit_normal(average, SmVector3d(0, 0, 1));
    // Source-face pointers are labels only; the source Brep may be gone.
    if (original)
        for (ULONG i = 0; i < faces.GetSize() && i < normals.GetSize(); ++i)
            if (faces[i] == static_cast<SmTopology*>(original))
                return unit_normal(normals[i], average);
    return average;
}

MeshArrays extract_mesh_arrays(SmPolyBrep& self, bool faceVaryingNormals)
{
    MeshArrays data;

    SmTArray<SmPolyVertex*> polyVerts;
    SmTArray<SmPolyFace*> polyFaces;
    self.GetPolyVertices(polyVerts);
    self.GetPolyFaces(polyFaces);

    const ULONG nVerts = polyVerts.GetSize();
    const ULONG nFaces = polyFaces.GetSize();
    data.points.reserve(static_cast<size_t>(nVerts) * 3);
    data.normals.reserve(static_cast<size_t>(nVerts) * 3);
    data.faces.reserve(static_cast<size_t>(nFaces) * 4);
    data.faceNormals.reserve(static_cast<size_t>(nFaces) * 3);

    std::unordered_map<SmPolyVertex*, ULONG> vertIdx;
    vertIdx.reserve(nVerts);

    for (ULONG i = 0; i < nVerts; ++i)
    {
        SmPolyVertex* pv = polyVerts[i];
        vertIdx[pv] = i;

        const SmPoint3d& pt = pv->GetPoint();
        data.points.push_back(static_cast<float>(pt.x));
        data.points.push_back(static_cast<float>(pt.y));
        data.points.push_back(static_cast<float>(pt.z));

        if (!faceVaryingNormals)
        {
            const SmVector3d normal = vertex_normal(pv, nullptr);
            data.normals.insert(data.normals.end(), {
                static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z)});
        }
    }

    for (ULONG i = 0; i < nFaces; ++i)
    {
        SmPolyFace* pf = polyFaces[i];
        SmPolyLoop* pLoop = pf->GetOuterPolyLoop();
        if (!pLoop)
        {
            continue;
        }

        SmTArray<SmPolyEdge*> edges;
        pLoop->GetPolyEdges(edges);
        const ULONG nEdges = edges.GetSize();
        if (nEdges == 0)
        {
            continue;
        }
        if (nEdges > static_cast<ULONG>(std::numeric_limits<int32_t>::max()))
        {
            throw std::overflow_error("PolyBrep face has too many vertices for an int32 mesh stream");
        }

        std::vector<int32_t> faceVerts;
        faceVerts.reserve(nEdges);
        bool badFace = false;
        for (ULONG j = 0; j < nEdges; ++j)
        {
            SmPolyVertex* pv = edges[j]->GetStartPolyVertex();
            const auto it = vertIdx.find(pv);
            if (it == vertIdx.end())
            {
                badFace = true;
                break;
            }
            if (it->second > static_cast<ULONG>(std::numeric_limits<int32_t>::max()))
            {
                throw std::overflow_error("PolyBrep has too many vertices for int32 mesh indices");
            }
            faceVerts.push_back(static_cast<int32_t>(it->second));
        }
        if (badFace)
        {
            continue;
        }

        data.faces.push_back(static_cast<int32_t>(nEdges));
        data.faces.insert(data.faces.end(), faceVerts.begin(), faceVerts.end());

        if (faceVaryingNormals)
        {
            for (ULONG j = 0; j < nEdges; ++j)
            {
                const SmVector3d normal = vertex_normal(edges[j]->GetStartPolyVertex(), pf->GetOriginalFace());
                data.normals.insert(data.normals.end(), {
                    static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z)});
            }
        }

        const SmVector3d fn = unit_normal(pf->GetNormal(TRUE, FALSE), SmVector3d(0, 0, 1));
        data.faceNormals.push_back(static_cast<float>(fn.x));
        data.faceNormals.push_back(static_cast<float>(fn.y));
        data.faceNormals.push_back(static_cast<float>(fn.z));
    }

    return data;
}

py::dict polybrep_to_mesh_list_arrays(SmPolyBrep& self, bool faceVaryingNormals)
{
    const MeshArrays data = extract_mesh_arrays(self, faceVaryingNormals);

    py::list pyPoints;
    py::list pyNormals;
    py::list pyFaces;
    py::list pyFaceNormals;

    const size_t nVerts = data.points.size() / 3;
    for (size_t i = 0; i < nVerts; ++i)
    {
        const size_t o = i * 3;
        pyPoints.append(py::make_tuple(data.points[o], data.points[o + 1], data.points[o + 2]));
    }
    for (size_t o = 0; o < data.normals.size(); o += 3)
    {
        pyNormals.append(py::make_tuple(data.normals[o], data.normals[o + 1], data.normals[o + 2]));
    }

    for (int32_t faceIdx : data.faces)
    {
        pyFaces.append(faceIdx);
    }

    const size_t nFaceNormals = data.faceNormals.size() / 3;
    for (size_t i = 0; i < nFaceNormals; ++i)
    {
        const size_t o = i * 3;
        pyFaceNormals.append(
            py::make_tuple(data.faceNormals[o], data.faceNormals[o + 1], data.faceNormals[o + 2]));
    }

    py::dict out;
    out["points"] = pyPoints;
    out["normals"] = pyNormals;
    out["faces"] = pyFaces;
    out["face_normals"] = pyFaceNormals;
    return out;
}

py::dict polybrep_to_numpy_arrays(SmPolyBrep& self, bool faceVaryingNormals)
{
    // NumPy views take shared ownership of the contiguous buffers through the
    // capsule below, so no Python list or follow-up NumPy copy is needed.
    auto data = std::make_unique<MeshArrays>(extract_mesh_arrays(self, faceVaryingNormals));

    MeshArrays* owned = data.release();
    py::capsule owner(
        owned,
        [](void* value)
        {
            delete static_cast<MeshArrays*>(value);
        }
    );

    const py::ssize_t pointRows = static_cast<py::ssize_t>(owned->points.size() / 3);
    const py::ssize_t faceNormalRows = static_cast<py::ssize_t>(owned->faceNormals.size() / 3);
    py::dict out;
    out["points"] = py::array_t<float>(
        { pointRows, static_cast<py::ssize_t>(3) },
        { static_cast<py::ssize_t>(3 * sizeof(float)), static_cast<py::ssize_t>(sizeof(float)) },
        owned->points.data(), owner);
    out["normals"] = py::array_t<float>(
        { static_cast<py::ssize_t>(owned->normals.size() / 3), static_cast<py::ssize_t>(3) },
        { static_cast<py::ssize_t>(3 * sizeof(float)), static_cast<py::ssize_t>(sizeof(float)) },
        owned->normals.data(), owner);
    out["faces"] = py::array_t<int32_t>(
        { static_cast<py::ssize_t>(owned->faces.size()) },
        { static_cast<py::ssize_t>(sizeof(int32_t)) },
        owned->faces.data(), owner);
    out["face_normals"] = py::array_t<float>(
        { faceNormalRows, static_cast<py::ssize_t>(3) },
        { static_cast<py::ssize_t>(3 * sizeof(float)), static_cast<py::ssize_t>(sizeof(float)) },
        owned->faceNormals.data(), owner);
    return out;
}


// Deep copy shared by PolyBrep.copy, __copy__ and __deepcopy__.
SmPolyBrep* polybrep_copy(SmPolyBrep& self)
{
    SmPolyBrep* r = nullptr;
    CHECK_STATUS(SmApiPolyBrepCopy(&self, r));
    return r;
}

} // namespace

void bind_polybrep(py::module_& m)
{
    // Poly mesh topology (register before PolyBrep methods return these pointers).
    py::class_<SmPolyVertex, std::unique_ptr<SmPolyVertex, py::nodelete>>(m, "PolyVertex")
        .def("get_point", [](const SmPolyVertex& self) { return point_to_tuple(self.GetPoint()); },
             "Vertex coordinates as (x, y, z).")
        .def("get_poly_brep", &SmPolyVertex::GetPolyBrep,
             py::return_value_policy::reference_internal);

    py::class_<SmPolyFace, std::unique_ptr<SmPolyFace, py::nodelete>>(m, "PolyFace")
        .def("get_poly_brep", &SmPolyFace::GetPolyBrep,
             py::return_value_policy::reference_internal)
        .def("get_normal", [](SmPolyFace& self) { return vec_to_tuple(self.GetNormal()); },
             "Evaluated face normal as (nx, ny, nz).");

    py::class_<SmPolyEdge, std::unique_ptr<SmPolyEdge, py::nodelete>>(m, "PolyEdge")
        .def("get_poly_brep", &SmPolyEdge::GetPolyBrep,
             py::return_value_policy::reference_internal)
        .def("get_start_vertex", &SmPolyEdge::GetStartPolyVertex,
             py::return_value_policy::reference_internal)
        .def("get_end_vertex", &SmPolyEdge::GetEndPolyVertex,
             py::return_value_policy::reference_internal);

    py::class_<SmPolyBrep, SmObject, std::unique_ptr<SmPolyBrep, py::nodelete>>(m, "PolyBrep")
        .def("get_faces", [](SmPolyBrep& self) {
            SmTArray<SmPolyFace*> f;
            self.GetPolyFaces(f);
            std::vector<SmPolyFace*> v;
            v.reserve(static_cast<size_t>(f.GetSize()));
            for (ULONG i = 0; i < f.GetSize(); ++i)
                v.push_back(f[i]);
            return v;
        }, "Return all ``PolyFace`` references (order is kernel-defined).")
        .def("get_vertices", [](SmPolyBrep& self) {
            SmTArray<SmPolyVertex*> pv;
            self.GetPolyVertices(pv);
            std::vector<SmPolyVertex*> v;
            v.reserve(static_cast<size_t>(pv.GetSize()));
            for (ULONG i = 0; i < pv.GetSize(); ++i)
                v.push_back(pv[i]);
            return v;
        }, "Return all ``PolyVertex`` references.")
        .def("get_edges", [](SmPolyBrep& self) {
            SmTArray<SmPolyEdge*> e;
            self.GetPolyEdges(e);
            std::vector<SmPolyEdge*> v;
            v.reserve(static_cast<size_t>(e.GetSize()));
            for (ULONG i = 0; i < e.GetSize(); ++i)
                v.push_back(e[i]);
            return v;
        }, "Return all ``PolyEdge`` references.")
        .def("get_points", [](SmPolyBrep& self) {
            SmTArray<SmPoint3d> pts;
            self.GetPolyPoints(pts);
            py::list out;
            for (ULONG i = 0; i < pts.GetSize(); ++i)
                out.append(point_to_tuple(pts[i]));
            return out;
        }, "Vertex positions as ``(x,y,z), ...`` (same logical order as topology).")
        .def(
            "to_mesh_arrays",
            &polybrep_to_mesh_list_arrays,
            py::arg("face_varying_normals") = false,
            "Walk the mesh topology and return a dict of flat arrays:\n"
            "  ``points``       : list of ``(x, y, z)`` per PolyVertex\n"
            "  ``normals``      : unit normal per vertex, or per corner when requested, as ``(nx, ny, nz)``\n"
            "  ``faces``        : flat polygon stream ``[n, i0, ..., i(n-1), n, ...]``\n"
            "  ``face_normals`` : unit per-face normal as ``(nx, ny, nz)``\n"
            "Same data shape as ``smtess_usd``'s JSON output, but with no USD or "
            "JSON round-trip; suitable for direct consumption by ``UsdGeom.Mesh`` "
            "(``faceVertexCounts`` / ``faceVertexIndices``) or PyVista. Shares the "
            "same mesh walk as ``to_numpy_arrays``.\n"
            "``face_varying_normals=True`` returns source-face corner normals in\n"
            "polygon order without counts; use USD ``faceVarying`` interpolation.\n"
            "Points stay shared. Otherwise normals are averaged per vertex.\n"
            "Missing or degenerate normals use the vertex average, then (0, 0, 1)."
        )
        .def(
            "to_numpy_arrays",
            &polybrep_to_numpy_arrays,
            py::arg("face_varying_normals") = false,
            "Return contiguous NumPy arrays for rendering and bulk processing.\n"
            "\n"
            "Args:\n"
            "    face_varying_normals: Return source-face corner normals instead of\n"
            "        averaged vertex normals; defaults to False.\n"
            "\n"
            "Returns:\n"
            "    dict[str, numpy.ndarray]: ``points`` and ``normals`` are\n"
            "        ``float32`` arrays with three columns; normal rows correspond\n"
            "        to vertices or polygon corners. ``faces`` is a flat\n"
            "        ``int32`` polygon stream; ``face_normals`` is a ``float32``\n"
            "        array shaped ``(m, 3)``.\n"
            "\n"
            "Notes:\n"
            "    The arrays own binding-managed contiguous buffers and remain\n"
            "    valid independently of later operations on this ``PolyBrep``.\n"
            "    Same normal layout and fallbacks as ``to_mesh_arrays``, which\n"
            "    returns Python lists instead."
        )
        .def("calculate_bounding_box", [](const SmPolyBrep& self) {
            SmExtent3d box;
            CHECK_STATUS(self.CalculateBoundingBox(box));
            return py::make_tuple(point_to_tuple(box.GetMin()), point_to_tuple(box.GetMax()));
        }, "Return ``(min_xyz, max_xyz)`` as nested 3-tuples (tolerance-expanded box).")
        .def("is_manifold_solid", &SmPolyBrep::IsManifoldSolid,
             "``True`` if the mesh is classified as a manifold solid.")
        .def("get_tolerance", &SmPolyBrep::GetTolerance, "Mesh tolerance value.")
        .def("copy", &polybrep_copy, py::return_value_policy::reference,
           "Pure: returns new PolyBrep; input unchanged.\n"
           "\n"
           "Deep-copy into a fresh handle with independent topology. Mutating "
           "the result (e.g. transform, boolean) does not affect the source, so "
           "use this before an in-place operation when the original must be "
           "preserved.\n"
           "\n"
           "Wraps: SmApiPolyBrepCopy")
        .def("__copy__", &polybrep_copy, py::return_value_policy::reference,
           "Support ``copy.copy``: a deep copy, same as ``copy()``. A shallow "
           "copy would share topology, so edits to one would change both.")
        .def("__deepcopy__", [](SmPolyBrep& self, py::dict) { return polybrep_copy(self); },
           py::return_value_policy::reference, py::arg("memo"),
           "Support ``copy.deepcopy``: same as ``copy()``.")
        .def("compute_area_centroid", [](SmPolyBrep& self) {
            double area = 0.0;
            SmPoint3d c;
            CHECK_STATUS(self.ComputeAreaCentroid(area, c));
            return py::make_tuple(area, point_to_tuple(c));
        }, "Return ``(area, (cx, cy, cz))``.")
        .def("compute_properties", [](SmPolyBrep& self, py::tuple origin) {
            SmPoint3d o = to_point(origin);
            double area = 0.0;
            double volume = 0.0;
            SmPoint3d bary;
            SmVector3d moments[2];
            CHECK_STATUS(self.ComputeProperties(o, area, volume, bary, moments, SM_MPT_ALL));
            py::dict d;
            d["area"] = area;
            d["volume"] = volume;
            d["barycenter"] = point_to_tuple(bary);
            d["moments_of_inertia"] = py::make_tuple(moments[0].x, moments[0].y, moments[0].z);
            d["products_of_inertia"] = py::make_tuple(moments[1].x, moments[1].y, moments[1].z);
            return d;
        }, py::arg("origin"),
           "Mass properties about ``origin``: area, volume, barycenter, inertia tensors (see SM_MPT_ALL).")
        .def("mass_properties", [](SmPolyBrep& self, double density, py::object origin) {
            double area = 0.0, volume = 0.0, mass = 0.0;
            SmPoint3d centroid;
            SmVector3d moi, poi;
            if (origin.is_none()) {
                // Default = centroidal, as for Brep.mass_properties: recover the
                // centroid about the bbox midpoint, then take inertia about it.
                SmExtent3d box;
                CHECK_STATUS(self.CalculateBoundingBox(box));
                double a1 = 0.0, v1 = 0.0, m1 = 0.0;
                SmVector3d moi1, poi1;
                CHECK_STATUS(SmApiPolyBrepComputeMassProperties(&self, density,
                             box.GetMid(), a1, v1, m1, centroid, moi1, poi1));
                SmPoint3d centroid2;
                CHECK_STATUS(SmApiPolyBrepComputeMassProperties(&self, density,
                             centroid, area, volume, mass, centroid2, moi, poi));
            } else {
                SmPoint3d o = to_point(origin.cast<py::tuple>());
                CHECK_STATUS(SmApiPolyBrepComputeMassProperties(&self, density,
                             o, area, volume, mass, centroid, moi, poi));
            }
            py::dict d;
            d["area"]     = area;
            d["volume"]   = volume;
            d["mass"]     = mass;
            d["centroid"] = point_to_tuple(centroid);
            d["moments_of_inertia"]  = py::make_tuple(moi.x, moi.y, moi.z);
            d["products_of_inertia"] = py::make_tuple(poi.x, poi.y, poi.z);
            return d;
        }, py::arg("density") = 1.0, py::arg("origin") = py::none(),
           "Pure: returns dict; inputs unchanged.\n"
           "\n"
           "Mass properties of a closed manifold mesh, computed exactly from its\n"
           "triangles. Mesh counterpart of ``Brep.mass_properties`` with the\n"
           "same keys and conventions; accuracy is that of the tessellation,\n"
           "so there is no ``relative_accuracy`` argument.\n"
           "\n"
           "Args:\n"
           "    density: Uniform mass density; ``mass = density * volume``.\n"
           "        Must be a valid number ``>= 1e-12`` (``SM_EFF_ZERO``);\n"
           "        zero, negative, sub-epsilon, or non-finite values raise\n"
           "        ``RuntimeError`` (``SM_ERR_INVALID_INPUT``).\n"
           "    origin: Optional ``(x, y, z)`` point that the moments and\n"
           "        products of inertia are taken about (axes parallel to the\n"
           "        world axes). When ``None`` (default) the result is\n"
           "        centroidal (two passes). Pass an explicit origin for a\n"
           "        single pass about that point.\n"
           "\n"
           "Returns:\n"
           "    dict with keys ``area``, ``volume``, ``mass``, ``centroid``\n"
           "    (world coordinates, independent of ``origin``), and the mass\n"
           "    ``moments_of_inertia`` ``(Ixx, Iyy, Izz)`` and\n"
           "    ``products_of_inertia`` ``(Iyz, Izx, Ixy)`` about axes through\n"
           "    ``origin`` (the centroid when ``origin`` is ``None``). The\n"
           "    products are the raw positive integrals; an inertia tensor's\n"
           "    off-diagonals are their negatives.\n"
           "\n"
           "Raises ``RuntimeError`` (``SM_ERR_INVALID_INPUT``) for a mesh that\n"
           "is not a closed manifold solid (see ``is_manifold_solid``) or a\n"
           "non-finite ``origin``.\n"
           "\n"
           "Wraps: SmApiPolyBrepComputeMassProperties")
        .def("section_plane", [](const SmPolyBrep& self, py::tuple plane_origin,
                                 py::tuple plane_normal, double zone_tol) {
            SmPoint3d o = to_point(plane_origin);
            SmVector3d n = to_vec(plane_normal);
            SmTArray<SmPoint3d> segs;
            CHECK_STATUS(self.DoSectioning(o, n, (SmZoneTol3d)zone_tol, &segs, nullptr));
            py::list lines;
            for (ULONG i = 0; i + 1 < segs.GetSize(); i += 2)
                lines.append(py::make_tuple(point_to_tuple(segs[i]), point_to_tuple(segs[i + 1])));
            return lines;
        }, py::arg("plane_origin"), py::arg("plane_normal"), py::arg("zone_tolerance"),
           "Intersect with a plane; returns a list of ``((x0,y0,z0), (x1,y1,z1))`` segments.")
        .def("ray_intersection", [](const SmPolyBrep& self, py::tuple ray_point,
                                    py::object ray_direction, double ray_tolerance,
                                    double ray_stepoff) {
            SmPoint3d rp = to_point(ray_point);
            std::unique_ptr<SmVector3d> dir_holder;
            const SmVector3d* pDir = nullptr;
            if (!ray_direction.is_none()) {
                dir_holder.reset(new SmVector3d(to_vec(ray_direction.cast<py::tuple>())));
                pDir = dir_holder.get();
            }
            SmSolutionArray solutions;
            CHECK_STATUS(self.RayIntersection(rp, pDir, ray_tolerance, ray_stepoff, nullptr, solutions));
            py::list hits;
            for (ULONG i = 0; i < solutions.GetSize(); ++i) {
                const SmSolution& sol = solutions[i];
                py::dict h;
                h["t"] = poly_ray_hit_t(sol);
                h["point"] = poly_ray_hit_point_tuple(sol);
                h["normal"] = poly_ray_hit_normal_tuple(sol);
                if (sol.m_lNumObjects > 0 && sol.m_apObjects[0])
                {
                    SmPolyFace* pHitFace = SM_CAST_PTR(SmPolyFace, sol.m_apObjects[0]);
                    if (pHitFace)
                        h["face"] = py::cast(pHitFace,
                                             py::return_value_policy::reference_internal);
                }
                hits.append(h);
            }
            return hits;
        }, py::arg("ray_point"), py::arg("ray_direction"), py::arg("ray_tolerance"),
           py::arg("ray_stepoff"),
           "Ray / point tests: ``ray_direction`` may be ``None`` for point-in-mesh style queries. "
           "Each hit is a dict with ``t``, ``point``, ``normal``, and optionally ``face``.");
}
