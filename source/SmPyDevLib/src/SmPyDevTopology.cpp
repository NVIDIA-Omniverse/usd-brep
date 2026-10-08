// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Developer-only topology bindings: Loop, Loopuse, Edgeuse, topology pick ray,
// and free-function accessors for edge/face topology walks.

#include "SmPyDevCommon.h"
#include "SmPyCommon.h"

#include <SmApiGeneral.h>
#include <SmLine.h>
#include <SmSolutionArray.h>
#include <SmTopologySolver.h>

#include <algorithm>
#include <stdexcept>

namespace
{
    const char* orient_to_string(SmOrientType orientation)
    {
        switch (orientation) {
            case SM_OT_SAME:
                return "same";
            case SM_OT_OPPOSITE:
                return "opposite";
            case SM_OT_UNKNOWN:
                return "unknown";
            default:
                return "undefined";
        }
    }

    template <typename TopologyT>
    int topology_index(SmTArray<TopologyT*>& topology, TopologyT* item)
    {
        if (item == nullptr) {
            return -1;
        }
        for (ULONG i = 0; i < topology.GetSize(); i++) {
            if (topology[i] == item) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    template <typename TopologyT>
    void append_unique_ptr(std::vector<TopologyT*>& values, TopologyT* value)
    {
        if (value == nullptr) {
            return;
        }
        if (std::find(values.begin(), values.end(), value) == values.end()) {
            values.push_back(value);
        }
    }

    py::object topology_pick_object(SmBrep& brep, SmObject* object,
                                    const char*& kind, int& index)
    {
        if (SmVertex* vertex = SM_CAST_PTR(SmVertex, object)) {
            SmTArray<SmVertex*> vertices;
            brep.GetVertices(vertices);
            kind = "Vertex";
            index = topology_index(vertices, vertex);
            return py::cast(vertex, py::return_value_policy::reference);
        }
        if (SmEdge* edge = SM_CAST_PTR(SmEdge, object)) {
            SmTArray<SmEdge*> edges;
            brep.GetEdges(edges);
            kind = "Edge";
            index = topology_index(edges, edge);
            return py::cast(edge, py::return_value_policy::reference);
        }
        if (SmFace* face = SM_CAST_PTR(SmFace, object)) {
            SmTArray<SmFace*> faces;
            brep.GetFaces(faces);
            kind = "Face";
            index = topology_index(faces, face);
            return py::cast(face, py::return_value_policy::reference);
        }
        kind = nullptr;
        index = -1;
        return py::none();
    }

    py::object topology_pick_normal(SmBrep& brep, SmObject* object, const SmPoint3d& params)
    {
        SmFace* face = SM_CAST_PTR(SmFace, object);
        if (face == nullptr) {
            return py::none();
        }

        SmVector3d normal;
        SmPoint2d uv(params.x, params.y);
        CHECK_STATUS(face->GetSurface()->EvaluateNormal(uv, TRUE, TRUE, normal));
        SmFaceuse* upward = face->GetUpwardFaceuse();
        if (upward && upward->GetShell()
            && brep.GetInfiniteRegion() != upward->GetShell()->GetRegion())
        {
            normal *= -1.0;
        }
        return vec_to_tuple(normal);
    }

    py::dict topology_pick_hit(SmBrep& brep, const SmLine& pick_line,
                               const SmSolution& solution,
                               const SmPoint3d& ray_origin,
                               const SmVector3d& ray_direction)
    {
        ULONG line_object_index = 0;
        ULONG line_param_index = 0;
        if (solution.GetIndex(&pick_line, TRUE, line_object_index, line_param_index) != SM_SUCCESS) {
            return py::dict();
        }

        ULONG object_index = 0;
        ULONG param_index = 0;
        if (solution.GetIndex(&pick_line, FALSE, object_index, param_index) != SM_SUCCESS) {
            return py::dict();
        }

        SmObject* object = solution.m_apObjects[object_index];
        const char* kind = nullptr;
        int index = -1;
        py::object topology = topology_pick_object(brep, object, kind, index);
        if (kind == nullptr) {
            return py::dict();
        }

        SmPoint3d point;
        SmPoint3d params;
        CHECK_STATUS(solution.GetPoint(&pick_line, FALSE, point, &params));
        const double ray_depth = (point - ray_origin).Dot(ray_direction);

        py::dict hit;
        hit["kind"] = kind;
        hit["topology_kind"] = kind;
        hit["topology_index"] = index;
        hit["topology"] = topology;
        hit["point"] = point_to_tuple(point);
        hit["parameters"] = point_to_tuple(params);
        hit["line_parameter"] = solution.m_vStart[line_param_index];
        hit["ray_depth"] = ray_depth;
        hit["normal"] = topology_pick_normal(brep, object, params);
        hit["is_range"] = solution.m_eSolutionType == SM_ST_RANGE_OF_VALUES;
        hit["object_index"] = static_cast<int>(object_index);
        hit["parameter_index"] = static_cast<int>(param_index);
        hit["line_parameter_index"] = static_cast<int>(line_param_index);
        if (SmVertex* vertex = SM_CAST_PTR(SmVertex, object)) {
            hit["vertex"] = py::cast(vertex, py::return_value_policy::reference);
            hit["edge"] = py::none();
            hit["face"] = py::none();
        } else if (SmEdge* edge = SM_CAST_PTR(SmEdge, object)) {
            hit["vertex"] = py::none();
            hit["edge"] = py::cast(edge, py::return_value_policy::reference);
            hit["face"] = py::none();
        } else if (SmFace* face = SM_CAST_PTR(SmFace, object)) {
            hit["vertex"] = py::none();
            hit["edge"] = py::none();
            hit["face"] = py::cast(face, py::return_value_policy::reference);
        }
        return hit;
    }

    py::list topology_pick_ray(SmBrep& brep, py::tuple ray_point,
                               py::tuple ray_direction, double tolerance,
                               double line_padding)
    {
        if (tolerance < 0.0) {
            throw std::invalid_argument("tolerance must be non-negative");
        }
        if (line_padding < 0.0) {
            throw std::invalid_argument("line_padding must be non-negative");
        }

        SmPoint3d origin = to_point(ray_point);
        SmVector3d direction = to_vec(ray_direction);
        CHECK_STATUS(direction.Unitize(TRUE));

        SmExtent3d bbox;
        CHECK_STATUS(brep.CalculateBoundingBox(bbox));

        ULONG found = 0;
        double enter = 0.0;
        double exit = 0.0;
        bbox.IntersectLine(origin, direction, found, enter, exit);
        if (found < 1) {
            SmExtent3d expanded = bbox;
            expanded.ExpandAbsolute(0.5 * tolerance);
            expanded.IntersectLine(origin, direction, found, enter, exit);
            if (found < 1) {
                return py::list();
            }
        }

        if (exit < enter) {
            const double tmp = enter;
            enter = exit;
            exit = tmp;
        }
        const double span = exit - enter;
        SmPoint3d line_start = origin + (enter - 0.5 * span - line_padding) * direction;
        double line_length = 2.0 * span + 2.0 * line_padding;
        if (line_length <= SM_EFF_ZERO) {
            line_length = 2.0 * line_padding + 1.0;
        }

        SmContext* context = SM_CONST_CAST(SmContext*, brep.GetContext());
        if (context == nullptr) {
            context = SmApiGetOrCreateContext();
        }
        SmLine* pick_line = new (*context) SmLine(
            line_start,
            direction,
            SmExtent1d(0.0, line_length),
            1.0,
            3,
            context);
        SmObjDelete cleanup(pick_line);

        SmSolutionArray solutions;
        SmTemporaryChangeValue<SmBoolean> save_editing(brep.m_bEditingEnabled, FALSE);
        CHECK_STATUS(SmTopologySolver::BrepCurveSolve(
            &brep,
            *pick_line,
            pick_line->GetNaturalInterval(),
            SM_SO_INTERSECT,
            SM_SR_ALL,
            tolerance,
            SM_BIG_DOUBLE,
            nullptr,
            solutions));

        py::list hits;
        for (ULONG i = 0; i < solutions.GetSize(); i++) {
            py::dict hit = topology_pick_hit(brep, *pick_line, solutions[i], origin, direction);
            if (py::len(hit) > 0) {
                hits.append(hit);
            }
        }
        return hits;
    }
}

void bind_dev_topology(py::module_& m)
{
    py::class_<SmLoopuse, std::unique_ptr<SmLoopuse, py::nodelete>>(m, "Loopuse")
        .def("edgeuses", [](SmLoopuse& self) {
            SmTArray<SmEdgeuse*> arr;
            self.GetEdgeuses(arr);
            std::vector<SmEdgeuse*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Ordered edgeuses around this loopuse.")
        .def("edges", [](SmLoopuse& self) {
            SmTArray<SmEdge*> arr;
            self.GetEdges(arr);
            std::vector<SmEdge*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Ordered edges around this loopuse.")
        .def("loop", &SmLoopuse::GetLoop, py::return_value_policy::reference,
             "Loop owning this loopuse.")
        .def("face", [](SmLoopuse& self) -> SmFace* {
            SmFaceuse* faceuse = self.GetFaceuse();
            return faceuse ? faceuse->GetFace() : nullptr;
        }, py::return_value_policy::reference,
           "Face on this side of the loop.")
        .def("brep", &SmLoopuse::GetBrep, py::return_value_policy::reference,
             "Parent Brep.")
        .def("other_loopuse", &SmLoopuse::GetOtherLoopuse, py::return_value_policy::reference,
             "Loopuse on the opposite side of the loop.")
        .def("is_edge_loopuse", [](SmLoopuse& self) {
            return (bool)self.IsEdgeLoopuse();
        }, "``True`` when this loopuse is made from edgeuses.")
        .def("is_vertex_loopuse", [](SmLoopuse& self) {
            return (bool)self.IsVertexLoopuse();
        }, "``True`` when this loopuse is a single-vertex loop.")
        .def("orientation", [](SmLoopuse& self) {
            return orient_to_string(self.GetOrientation());
        }, "Loop orientation as ``'same'``, ``'opposite'``, or ``'unknown'``.")
        .def("__repr__", [](SmLoopuse& self) {
            SmTArray<SmEdgeuse*> edgeuses;
            self.GetEdgeuses(edgeuses);
            return std::string("<Loopuse edgeuses=")
                 + std::to_string(edgeuses.GetSize()) + ">";
        });

    py::class_<SmEdgeuse, std::unique_ptr<SmEdgeuse, py::nodelete>>(m, "Edgeuse")
        .def("edge", &SmEdgeuse::GetEdge, py::return_value_policy::reference,
             "Edge used by this oriented edgeuse.")
        .def("face", &SmEdgeuse::GetFace, py::return_value_policy::reference,
             "Face on this side of the edgeuse.")
        .def("loopuse", &SmEdgeuse::GetLoopuse, py::return_value_policy::reference,
             "Loopuse that owns this edgeuse.")
        .def("loop", [](SmEdgeuse& self) -> SmLoop* {
            SmLoopuse* loopuse = self.GetLoopuse();
            return loopuse ? loopuse->GetLoop() : nullptr;
        }, py::return_value_policy::reference,
           "Loop that owns this edgeuse.")
        .def("mate", &SmEdgeuse::GetMate, py::return_value_policy::reference,
             "Mated edgeuse on the same face.")
        .def("cw_edgeuse", &SmEdgeuse::GetCWEdgeuse, py::return_value_policy::reference,
             "Clockwise neighbour in the owning loopuse, or ``None`` for wires.")
        .def("ccw_edgeuse", &SmEdgeuse::GetCCWEdgeuse, py::return_value_policy::reference,
             "Counter-clockwise neighbour in the owning loopuse, or ``None`` for wires.")
        .def("edge_parameter", &SmEdgeuse::GetEdgeParam, py::arg("normalized"),
             "Convert a normalized oriented edgeuse parameter in ``[0, 1]`` "
             "to the underlying edge-curve parameter.")
        .def("point_at_normalized", [](SmEdgeuse& self, double normalized) {
            SmPoint3d pt;
            CHECK_STATUS(self.NormalizedEvaluate(
                normalized,
                /*bParameterSpaceEval=*/FALSE,
                pt,
                /*pOptFirstDeriv=*/nullptr,
                /*bOKToMakeUVTrimCurve=*/TRUE));
            return point_to_tuple(pt);
        }, py::arg("normalized"),
           "Evaluate the oriented 3D edgeuse point at a normalized "
           "parameter in ``[0, 1]``.")
        .def("uv_point_at_normalized", [](SmEdgeuse& self, double normalized) {
            SmPoint3d uv;
            CHECK_STATUS(self.NormalizedEvaluate(
                normalized,
                /*bParameterSpaceEval=*/TRUE,
                uv,
                /*pOptFirstDeriv=*/nullptr,
                /*bOKToMakeUVTrimCurve=*/TRUE));
            return py::make_tuple(uv.x, uv.y);
        }, py::arg("normalized"),
           "Evaluate the synthesized UV trim curve at a normalized oriented "
           "edgeuse parameter in ``[0, 1]``.")
        .def("orientation", [](SmEdgeuse& self) {
            return orient_to_string(self.GetOrientation());
        }, "Edgeuse orientation as ``'same'``, ``'opposite'``, or ``'unknown'``.")
        .def("brep", &SmEdgeuse::GetBrep, py::return_value_policy::reference,
             "Parent Brep.")
        .def("__repr__", [](SmEdgeuse& self) {
            return std::string("<Edgeuse orientation=")
                 + orient_to_string(self.GetOrientation()) + ">";
        });

    py::class_<SmLoop, std::unique_ptr<SmLoop, py::nodelete>>(m, "Loop")
        .def("edgeuses", [](SmLoop& self) {
            SmTArray<SmEdgeuse*> arr;
            self.GetEdgeuses(arr);
            std::vector<SmEdgeuse*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Ordered edgeuses around this loop.")
        .def("edges", [](SmLoop& self) {
            SmTArray<SmEdge*> arr;
            self.GetEdges(arr);
            std::vector<SmEdge*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Ordered edges around this loop.")
        .def("vertices", [](SmLoop& self) {
            SmTArray<SmVertex*> arr;
            self.GetVertices(arr);
            std::vector<SmVertex*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Ordered vertices around this loop.")
        .def("loopuse", &SmLoop::GetLoopuse, py::return_value_policy::reference,
             "Primary loopuse for this loop.")
        .def("loopuses", [](SmLoop& self) {
            SmLoopuse *loopuse1 = nullptr, *loopuse2 = nullptr;
            self.GetLoopuses(loopuse1, loopuse2);
            return py::make_tuple(loopuse1, loopuse2);
        }, py::return_value_policy::reference,
           "``(primary_loopuse, opposite_loopuse)`` for this loop.")
        .def("face", &SmLoop::GetFace, py::return_value_policy::reference,
             "Face that owns this loop.")
        .def("brep", &SmLoop::GetBrep, py::return_value_policy::reference,
             "Parent Brep.")
        .def("is_outer_loop", [](SmLoop& self) {
            return (bool)self.IsOuterLoop();
        }, "``True`` when this is the face's outer loop.")
        .def("__repr__", [](SmLoop& self) {
            SmTArray<SmEdgeuse*> edgeuses;
            self.GetEdgeuses(edgeuses);
            return std::string("<Loop edgeuses=")
                 + std::to_string(edgeuses.GetSize()) + ">";
        });

    m.def(
        "edgeuses",
        [](SmEdge& edge) {
            SmTArray<SmEdgeuse*> arr;
            CHECK_STATUS(edge.GetEdgeuses(arr));
            std::vector<SmEdgeuse*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        },
        py::arg("edge"), py::return_value_policy::reference,
        "All oriented edgeuses that reference ``edge``, in radial list order.");

    m.def(
        "loopuses",
        [](SmEdge& edge) {
            SmTArray<SmEdgeuse*> edgeuses;
            CHECK_STATUS(edge.GetEdgeuses(edgeuses));
            std::vector<SmLoopuse*> loopuses;
            for (ULONG i = 0; i < edgeuses.GetSize(); i++) {
                append_unique_ptr(loopuses, edgeuses[i] ? edgeuses[i]->GetLoopuse() : nullptr);
            }
            return loopuses;
        },
        py::arg("edge"), py::return_value_policy::reference,
        "Unique loopuses that contain ``edge``.");

    m.def(
        "loops",
        [](SmEdge& edge) {
            SmTArray<SmEdgeuse*> edgeuses;
            CHECK_STATUS(edge.GetEdgeuses(edgeuses));
            std::vector<SmLoop*> loops;
            for (ULONG i = 0; i < edgeuses.GetSize(); i++) {
                SmLoopuse* loopuse = edgeuses[i] ? edgeuses[i]->GetLoopuse() : nullptr;
                append_unique_ptr(loops, loopuse ? loopuse->GetLoop() : nullptr);
            }
            return loops;
        },
        py::arg("edge"), py::return_value_policy::reference,
        "Unique loops that contain ``edge``.");

    m.def(
        "edgeuse_of_face",
        &SmEdge::GetEdgeuseOfFace,
        py::arg("edge"), py::arg("face"),
        py::return_value_policy::reference,
        "Edgeuse of ``edge`` on ``face``, or ``None`` if not incident.");

    m.def(
        "loop_of_face",
        &SmEdge::GetLoopOfFace,
        py::arg("edge"), py::arg("face"),
        py::return_value_policy::reference,
        "Loop on ``face`` that contains ``edge``, or ``None`` if not incident.");

    m.def(
        "face_loops",
        [](SmFace& face) {
            SmTArray<SmLoop*> arr;
            face.GetLoops(arr);
            std::vector<SmLoop*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        },
        py::arg("face"), py::return_value_policy::reference,
        "All trim loops of ``face``: index 0 is the outer loop.");

    m.def(
        "topology_counts",
        [](SmBrep& brep) {
            SmTArray<SmRegion*> regions;
            SmTArray<SmShell*> shells;
            SmTArray<SmFace*> faces;
            SmTArray<SmLoop*> loops;
            SmTArray<SmEdge*> edges;
            SmTArray<SmVertex*> vertices;
            brep.GetRegions(regions);
            brep.GetShells(shells);
            brep.GetFaces(faces);
            brep.GetLoops(loops);
            brep.GetEdges(edges);
            brep.GetVertices(vertices);
            py::dict out;
            out["regions"] = static_cast<int>(regions.GetSize());
            out["shells"] = static_cast<int>(shells.GetSize());
            out["faces"] = static_cast<int>(faces.GetSize());
            out["loops"] = static_cast<int>(loops.GetSize());
            out["edges"] = static_cast<int>(edges.GetSize());
            out["vertices"] = static_cast<int>(vertices.GetSize());
            return out;
        },
        py::arg("brep"),
        "Debugger counts of regions, shells, faces, loops, edges, and vertices.");

    m.def(
        "topology_pick_ray",
        [](SmBrep& brep, py::tuple ray_point, py::tuple ray_direction,
           double tolerance, double line_padding) {
            return topology_pick_ray(brep, ray_point, ray_direction, tolerance, line_padding);
        },
        py::arg("brep"), py::arg("ray_point"), py::arg("ray_direction"),
        py::arg("tolerance"), py::arg("line_padding") = 0.5,
        "Intersect a viewport pick ray with a Brep's exact topology.\n"
        "\n"
        "Args:\n"
        "    brep: Brep to pick against.\n"
        "    ray_point: Ray origin as ``(x, y, z)``.\n"
        "    ray_direction: Ray direction as ``(dx, dy, dz)``. Normalized\n"
        "        before solving.\n"
        "    tolerance: Distance tolerance for near hits. Non-negative.\n"
        "    line_padding: Extra distance before/after the bounding-box span.\n"
        "\n"
        "Returns:\n"
        "    list[dict]: Hit records from ``SmTopologySolver::BrepCurveSolve``.\n"
        "\n"
        "Wraps: SmTopologySolver::BrepCurveSolve");

    m.def(
        "poly_face_original_face_index",
        [](SmPolyFace& poly_face) {
            SmFace* face = poly_face.GetOriginalFace();
            SmBrep* brep = face ? face->GetBrep() : nullptr;
            if (face == nullptr || brep == nullptr) {
                return -1;
            }
            SmTArray<SmFace*> faces;
            brep->GetFaces(faces);
            return topology_index(faces, face);
        },
        py::arg("poly_face"),
        "Return the source BRep face index carried by a tessellation face.\n"
        "\n"
        "Returns ``-1`` for polygon faces that were not produced from an exact "
        "BRep face. The source BRep must remain alive.");
}
