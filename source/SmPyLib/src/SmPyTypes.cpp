// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges SMLib kernel types to Python:
//   SmObject.h, SmBrep.h, SmFace.h, SmEdge.h, SmVertex.h, SmSurface.h,
//   SmCurve.h, SmPoly.h (topology), SmVector3d.h, SmPoint3d.h,
//   SmAxis2Placement.h.
//
// Also exposes Brep / Face / Edge / Vertex / Surface / Curve property
// methods (bbox, area, length, point, copy, volume, manifold check,
// uv-domain, parameter-range, evaluate, normal, ...) by wrapping the
// SmApi* entry points in SmApiQueries.h / SmApiSurfaces.h /
// SmApiCurves.h, plus a handful of kernel-direct calls for queries
// that do not yet have a SmApi entry (IsPlanar, IsPeriodic, Length,
// GetNaturalUVDomain, GetNaturalInterval).  The same Brep / Face /
// Edge / Vertex operations are available as flat sm.queries.* aliases
// in SmPyQueries.cpp, and the surface evaluators have flat aliases
// (sm.evaluate_surface_*) in the same file; the methods here are the
// primary user-facing surface for "what is the X of this geometric
// object?"

#include "SmPyCommon.h"

#include <SmApiBrep.h>
#include <SmApiCurves.h>
#include <SmApiGeneral.h>
#include <SmApiQueries.h>
#include <SmApiSurfaces.h>
#include <SmAxis2Placement.h>

#if defined(_UNICODE) && !defined(_WIN32)
#include <codecvt>
#include <locale>
#endif
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#if defined(_UNICODE) && defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
    enum class SmPyNativeBrepFileFormat
    {
        ASCII,
        BINARY,
        UNKNOWN,
    };

    enum class SmPyNativeBrepFileContainer
    {
        SINGLE_BREP,
        PART,
        UNRECOGNIZED_PART_VERSION,
        UNKNOWN,
    };

    struct SmPyNativeBrepFileInfo
    {
        SmPyNativeBrepFileFormat format = SmPyNativeBrepFileFormat::UNKNOWN;
        SmPyNativeBrepFileContainer container = SmPyNativeBrepFileContainer::UNKNOWN;
        std::uint64_t database_version = 0;
    };

#ifdef _UNICODE
    using SmPyTCharPath = std::wstring;
#else
    using SmPyTCharPath = std::string;
#endif

    SmPyTCharPath to_tchar_path(const std::string& filename)
    {
#ifdef _UNICODE
#ifdef _WIN32
        int length = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, filename.c_str(), -1, nullptr, 0);
        if (length <= 0) {
            throw std::runtime_error("Could not decode UTF-8 path: " + filename);
        }
        std::wstring path(static_cast<size_t>(length), L'\0');
        int converted = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, filename.c_str(), -1,
                                            &path[0], length);
        if (converted <= 0) {
            throw std::runtime_error("Could not decode UTF-8 path: " + filename);
        }
        if (!path.empty() && path.back() == L'\0') {
            path.pop_back();
        }
        return path;
#else
        try {
            std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
            return converter.from_bytes(filename);
        } catch (const std::range_error&) {
            throw std::runtime_error("Could not decode UTF-8 path: " + filename);
        }
#endif
#else
        return filename;
#endif
    }

    std::uint64_t read_little_endian_u64(const unsigned char* bytes)
    {
        std::uint64_t value = 0;
        for (size_t i = 0; i < sizeof(value); ++i) {
            value |= static_cast<std::uint64_t>(bytes[i]) << (8 * i);
        }
        return value;
    }

    SmPyNativeBrepFileInfo detect_binary_part_file_info(const unsigned char* header, size_t count)
    {
        if (count < 2 * sizeof(std::uint64_t)) {
            return {};
        }

        constexpr std::uint64_t version_scale = 1000000;
        // Version 40 introduced fixed-width binary longs; version 42 added
        // the healer version encoded in the second part-header count.
        constexpr std::uint64_t fixed_width_database_version = 40;
        constexpr std::uint64_t healer_database_version = 42;
        constexpr std::uint64_t maximum_encoded_version = 999;
        const std::uint64_t database_version = read_little_endian_u64(header) / version_scale;
        const std::uint64_t healer_version = read_little_endian_u64(header + sizeof(std::uint64_t)) /
                                             version_scale;

        // Binary database versions identify discrete persisted formats.
        // Recognize only formats we know rather than assigning an unknown
        // version the behavior of the nearest lower version.
        const bool fixed_width_legacy_part =
            database_version == fixed_width_database_version && healer_version == 0;
        const bool healer_versioned_part =
            database_version == healer_database_version && healer_version > 0 &&
            healer_version <= maximum_encoded_version;
        if (fixed_width_legacy_part || healer_versioned_part) {
            return {SmPyNativeBrepFileFormat::BINARY, SmPyNativeBrepFileContainer::PART,
                    database_version};
        }

        // A fixed-width, version-encoded prefix still identifies a part-like
        // header even when its database version is not recognized.  Keep that
        // state separate from genuinely headerless legacy binary data so an
        // explicit format selector cannot send it to the single-Brep parser.
        const bool plausible_pre_healer_part =
            database_version >= fixed_width_database_version &&
            database_version < healer_database_version && healer_version == 0;
        const bool plausible_healer_versioned_part =
            database_version >= healer_database_version &&
            database_version <= maximum_encoded_version && healer_version > 0 &&
            healer_version <= maximum_encoded_version;
        const bool unrecognized_part_version =
            plausible_pre_healer_part || plausible_healer_versioned_part;
        if (unrecognized_part_version) {
            return {SmPyNativeBrepFileFormat::BINARY,
                    SmPyNativeBrepFileContainer::UNRECOGNIZED_PART_VERSION, database_version};
        }
        return {};
    }

    SmPyNativeBrepFileInfo detect_native_brep_file_info(const std::string& filename)
    {
        SmPyTCharPath path = to_tchar_path(filename);
#ifdef _UNICODE
        FILE* file = _wfopen(path.c_str(), L"rb");
#else
        FILE* file = std::fopen(path.c_str(), "rb");
#endif
        if (file == nullptr) {
            throw std::runtime_error("Could not open native SMLib Brep file: " + filename);
        }

        unsigned char header[32] = {};
        const size_t count = std::fread(header, 1, sizeof(header), file);
        std::fclose(file);

        if (count >= 8 && std::memcmp(header, "Version ", 8) == 0) {
            return {SmPyNativeBrepFileFormat::ASCII, SmPyNativeBrepFileContainer::SINGLE_BREP};
        }
        if (count >= 13 && std::memcmp(header, "//Brep Starts", 13) == 0) {
            return {SmPyNativeBrepFileFormat::ASCII, SmPyNativeBrepFileContainer::SINGLE_BREP};
        }
        constexpr char part_header[] = "//[Output Summary]";
        if (count >= sizeof(part_header) - 1 &&
            std::memcmp(header, part_header, sizeof(part_header) - 1) == 0) {
            return {SmPyNativeBrepFileFormat::ASCII, SmPyNativeBrepFileContainer::PART};
        }
        const SmPyNativeBrepFileInfo binary_part_info = detect_binary_part_file_info(header, count);
        if (binary_part_info.container != SmPyNativeBrepFileContainer::UNKNOWN) {
            return binary_part_info;
        }
        if (count >= 2 && header[0] == static_cast<unsigned char>('V')) {
            return {SmPyNativeBrepFileFormat::BINARY, SmPyNativeBrepFileContainer::SINGLE_BREP};
        }
        return {SmPyNativeBrepFileFormat::UNKNOWN, SmPyNativeBrepFileContainer::UNKNOWN};
    }

    const char* native_brep_file_format_name(SmPyNativeBrepFileFormat format)
    {
        switch (format) {
            case SmPyNativeBrepFileFormat::ASCII:
                return "ASCII";
            case SmPyNativeBrepFileFormat::BINARY:
                return "binary";
            default:
                return "unknown";
        }
    }

    bool resolve_native_brep_ascii(const std::string& filename, py::object ascii)
    {
        const SmPyNativeBrepFileInfo info = detect_native_brep_file_info(filename);
        if (info.container == SmPyNativeBrepFileContainer::UNRECOGNIZED_PART_VERSION) {
            throw std::runtime_error(
                "File appears to have a native SMLib binary part header with unrecognized database version " +
                std::to_string(info.database_version) +
                "; refusing to read it as a single Brep: " + filename);
        }
        if (info.container == SmPyNativeBrepFileContainer::PART) {
            std::string message = "Native SMLib part container cannot be read as a single Brep; use "
                                  "read_part_from_file(filename, ascii=";
            message += info.format == SmPyNativeBrepFileFormat::ASCII ? "True" : "False";
            message += "): ";
            message += filename;
            throw std::runtime_error(message);
        }

        const SmPyNativeBrepFileFormat detected = info.format;
        if (ascii.is_none()) {
            if (detected == SmPyNativeBrepFileFormat::ASCII) {
                return true;
            }
            if (detected == SmPyNativeBrepFileFormat::BINARY) {
                return false;
            }
            throw std::runtime_error("Could not determine native SMLib Brep file format: " + filename);
        }

        const bool requested_ascii = ascii.cast<bool>();
        if (detected == SmPyNativeBrepFileFormat::UNKNOWN) {
            return requested_ascii;
        }
        const bool detected_ascii = detected == SmPyNativeBrepFileFormat::ASCII;
        if (requested_ascii != detected_ascii) {
            std::string message = "Native SMLib Brep file appears to be ";
            message += native_brep_file_format_name(detected);
            message += "; refusing to read it with ascii=";
            message += requested_ascii ? "True" : "False";
            message += ": ";
            message += filename;
            throw std::runtime_error(message);
        }
        return requested_ascii;
    }

    py::list face_boundary_loops(SmFace& face)
    {
        SmTArray<ULONG> edge_counts_per_loop;
        SmTArray<SmEdge*> edges;
        SmTArray<SmOrientType> orientations;
        CHECK_STATUS(SmApiFaceGetBoundaryLoops(
            &face, edge_counts_per_loop, edges, orientations));
        if (orientations.GetSize() != edges.GetSize()) {
            throw std::runtime_error(
                "SmApiFaceGetBoundaryLoops returned inconsistent parallel arrays");
        }

        py::list loops;
        ULONG edge_index = 0;
        for (ULONG ii = 0; ii < edge_counts_per_loop.GetSize(); ii++) {
            const ULONG edge_count = edge_counts_per_loop[ii];
            if (edge_index > edges.GetSize() || edge_count > edges.GetSize() - edge_index) {
                throw std::runtime_error(
                    "SmApiFaceGetBoundaryLoops returned inconsistent loop sizes");
            }

            py::list loop;
            for (ULONG jj = 0; jj < edge_count; jj++, edge_index++) {
                bool forward;
                if (orientations[edge_index] == SM_OT_SAME) {
                    forward = true;
                } else if (orientations[edge_index] == SM_OT_OPPOSITE) {
                    forward = false;
                } else {
                    throw std::runtime_error(
                        "SmApiFaceGetBoundaryLoops returned an unsupported edge orientation");
                }

                loop.append(py::make_tuple(
                    py::cast(edges[edge_index], py::return_value_policy::reference),
                    forward));
            }
            loops.append(loop);
        }

        if (edge_index != edges.GetSize()) {
            throw std::runtime_error(
                "SmApiFaceGetBoundaryLoops returned inconsistent parallel arrays");
        }
        return loops;
    }

    py::list face_boundary_outer_loop(SmFace& face)
    {
        py::list loops = face_boundary_loops(face);
        return loops[0].cast<py::list>();
    }
}

SmBrep* smpy_read_brep_from_file(const std::string& filename, py::object ascii,
                                 bool rebuild_uv_trim_curves)
{
    const bool read_ascii = resolve_native_brep_ascii(filename, ascii);
    SmPyTCharPath path = to_tchar_path(filename);
    SmBrep* result = nullptr;
    CHECK_STATUS(SmApiReadBrepFromFile(
        path.c_str(),
        read_ascii,
        rebuild_uv_trim_curves,
        result));
    return result;
}

py::tuple smpy_read_part_from_file(const std::string& filename, bool ascii)
{
    SmTArray<SmCurve*> curves;
    SmTArray<SmSurface*> surfaces;
    SmTArray<long> boolean_tree_nodes;
    SmTArray<SmBrep*> breps;
    SmObjsDelete<SmCurve*> cleanup_curves(&curves);
    SmObjsDelete<SmSurface*> cleanup_surfaces(&surfaces);
    SmObjsDelete<SmBrep*> cleanup_breps(&breps);
    SmPyTCharPath path = to_tchar_path(filename);

    CHECK_STATUS(SmApiReadPartFromFile(
        path.c_str(),
        ascii,
        curves,
        surfaces,
        boolean_tree_nodes,
        breps));

    py::list py_curves;
    for (ULONG i = 0; i < curves.GetSize(); ++i)
        py_curves.append(py::cast(curves[i], py::return_value_policy::reference));

    py::list py_surfaces;
    for (ULONG i = 0; i < surfaces.GetSize(); ++i)
        py_surfaces.append(py::cast(surfaces[i], py::return_value_policy::reference));

    py::list py_boolean_tree_nodes;
    for (ULONG i = 0; i < boolean_tree_nodes.GetSize(); ++i)
        py_boolean_tree_nodes.append(boolean_tree_nodes[i]);

    py::list py_breps;
    for (ULONG i = 0; i < breps.GetSize(); ++i)
        py_breps.append(py::cast(breps[i], py::return_value_policy::reference));

    cleanup_curves.Clear();
    cleanup_surfaces.Clear();
    cleanup_breps.Clear();
    return py::make_tuple(py_curves, py_surfaces, py_boolean_tree_nodes, py_breps);
}

// Deep copy shared by Brep.copy, __copy__ and __deepcopy__.
SmBrep* smpy_brep_copy(SmBrep& self)
{
    SmBrep* result = nullptr;
    CHECK_STATUS(SmApiBrepCopy(&self, result));
    return result;
}

void bind_types(py::module_& m)
{
    // -----------------------------------------------------------------------
    //  Opaque kernel types.  ``py::nodelete`` because lifetimes are owned by
    //  the SMLib SmContext, not by Python.
    // -----------------------------------------------------------------------
    py::class_<SmObject, std::unique_ptr<SmObject, py::nodelete>>(m, "Object");

    py::class_<SmBrep, SmObject, std::unique_ptr<SmBrep, py::nodelete>>(m, "Brep")
        .def("faces", [](SmBrep& self) {
            SmTArray<SmFace*> f;
            self.GetFaces(f);
            std::vector<SmFace*> v;
            for (ULONG i = 0; i < f.GetSize(); i++) v.push_back(f[i]);
            return v;
        })
        .def("edges", [](SmBrep& self) {
            SmTArray<SmEdge*> e;
            self.GetEdges(e);
            std::vector<SmEdge*> v;
            for (ULONG i = 0; i < e.GetSize(); i++) v.push_back(e[i]);
            return v;
        })
        .def("vertices", [](SmBrep& self) {
            SmTArray<SmVertex*> v;
            self.GetVertices(v);
            std::vector<SmVertex*> r;
            for (ULONG i = 0; i < v.GetSize(); i++) r.push_back(v[i]);
            return r;
        })
        // -- Property methods (wrap SmApiQueries.h) ---------------------
        .def("bounding_box", [](SmBrep& self, bool tight) {
            SmPoint3d mn, mx;
            CHECK_STATUS(SmApiBrepBoundingBox(&self, tight ? TRUE : FALSE, mn, mx));
            return py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
        }, py::arg("tight") = false,
           "Axis-aligned bounding box as ``((min_x, min_y, min_z), "
           "(max_x, max_y, max_z))``. ``tight=True`` samples each face's "
           "surface for a minimal box (expensive); default is the loose "
           "vertex/edge/face union.")
        .def("center", [](SmBrep& self) {
            SmPoint3d mn, mx;
            CHECK_STATUS(SmApiBrepBoundingBox(&self, FALSE, mn, mx));
            return py::make_tuple(0.5 * (mn.x + mx.x),
                                  0.5 * (mn.y + mx.y),
                                  0.5 * (mn.z + mx.z));
        }, "Centre of the loose axis-aligned bounding box as ``(x, y, z)``. "
           "Convenience for ``((min + max) / 2)`` when you only need a "
           "rough anchor point (e.g. as a translation target). For the "
           "mass centre of gravity use ``mass_properties()['centroid']``.")
        .def("volume", [](SmBrep& self, double accuracy) {
            double vol = 0.0;
            CHECK_STATUS(SmApiBrepComputeVolume(&self, accuracy, vol));
            return vol;
        }, py::arg("relative_accuracy") = 1.0e-3,
           "Enclosed volume in cubic modeling units. The Brep should be a "
           "closed manifold solid; behaviour on sheet bodies is undefined. "
           "``relative_accuracy`` is clamped to ``[1e-8, 1e-1]``; non-finite "
           "values raise ``RuntimeError``.")
        .def("area", [](SmBrep& self, double accuracy) {
            double area = 0.0;
            CHECK_STATUS(SmApiBrepComputeArea(&self, accuracy, area));
            return area;
        }, py::arg("relative_accuracy") = 1.0e-3,
           "Total surface area in square modeling units: the sum of "
           "``Face.area()`` over every face, with the same "
           "``relative_accuracy`` range. Works on sheet bodies and open "
           "shells, which ``mass_properties()`` rejects. Non-finite "
           "accuracy raises ``RuntimeError``.\n"
           "\n"
           "Wraps: SmApiBrepComputeArea")
        .def("relationship_to", [](SmBrep& self, SmBrep& other) {
            int rel = 0; double dist = 0.0;
            CHECK_STATUS(SmApiBrepRelationship(&self, &other, rel, dist));
            static const char* kNames[] = {
                "separate", "touching", "interpenetrating", "a_contains_b", "b_contains_a" };
            py::dict d;
            d["relationship"] = (rel >= 0 && rel <= 4) ? kNames[rel] : "unknown";
            d["distance"] = dist;
            return d;
        }, py::arg("other"),
           "Classify how this Brep (a) and ``other`` (b) are arranged. Returns "
           "``{'relationship': str, 'distance': float}`` -- see the module-level "
           "``brep_relationship`` for the relationship values and caveats.")
        .def("distance_to", [](SmBrep& self, SmBrep& other) {
            double dist = 0.0;
            CHECK_STATUS(SmApiBrepDistance(&self, &other, dist));
            return dist;
        }, py::arg("other"),
           "Distance to another Brep's boundary surface: the surface gap when "
           "disjoint (the wall clearance when one contains the other), 0.0 when "
           "they touch or interpenetrate. Pair with ``relationship_to`` for what "
           "the number means.")
        .def("mass_properties", [](const SmBrep& self, double accuracy, double density,
                                   py::object origin) {
            double area = 0.0, volume = 0.0, mass = 0.0;
            SmPoint3d centroid;
            SmVector3d moi, poi;
            if (origin.is_none()) {
                // Default = centroidal: recover the centroid about the bbox
                // midpoint (conditioning), then take inertia about it.
                SmPoint3d mn, mx;
                // Read-only call; const_cast for the non-const API signature.
                CHECK_STATUS(SmApiBrepBoundingBox(const_cast<SmBrep*>(&self),
                                                  FALSE, mn, mx));
                SmPoint3d mid(0.5 * (mn.x + mx.x),
                              0.5 * (mn.y + mx.y),
                              0.5 * (mn.z + mx.z));
                double a1 = 0.0, v1 = 0.0, m1 = 0.0;
                SmVector3d moi1, poi1;
                CHECK_STATUS(SmApiBrepComputeMassProperties(&self, accuracy, density,
                             mid, a1, v1, m1, centroid, moi1, poi1));
                SmPoint3d centroid2;
                CHECK_STATUS(SmApiBrepComputeMassProperties(&self, accuracy, density,
                             centroid, area, volume, mass, centroid2, moi, poi));
            } else {
                SmPoint3d o = to_point(origin.cast<py::tuple>());
                CHECK_STATUS(SmApiBrepComputeMassProperties(&self, accuracy, density,
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
        }, py::arg("relative_accuracy") = 1.0e-3, py::arg("density") = 1.0,
           py::arg("origin") = py::none(),
           "Pure: returns dict; inputs unchanged.\n"
           "\n"
           "Mass properties of a closed manifold solid, computed directly from\n"
           "the exact BRep geometry by adaptive numerical integration to the\n"
           "requested accuracy.\n"
           "\n"
           "Args:\n"
           "    relative_accuracy: Integration accuracy, clamped to\n"
           "        ``[1e-4, 1e-1]`` (the kernel's reliable envelope); a\n"
           "        non-finite value raises ``RuntimeError``.\n"
           "    density: Uniform mass density; ``mass = density * volume``.\n"
           "        Must be a valid number ``>= 1e-12`` (``SM_EFF_ZERO``);\n"
           "        zero, negative, sub-epsilon, or non-finite values raise\n"
           "        ``RuntimeError`` (``SM_ERR_INVALID_INPUT``).\n"
           "    origin: Optional ``(x, y, z)`` point that the moments and\n"
           "        products of inertia are taken about (axes parallel to the\n"
           "        world axes). When ``None`` (default) the result is\n"
           "        centroidal -- the centroid is recovered first and the\n"
           "        inertia is taken about it (two integration passes). Pass an\n"
           "        explicit origin for a single pass about that point.\n"
           "\n"
           "Returns:\n"
           "    dict with keys ``area`` and ``volume`` (floats), ``mass``\n"
           "    (``density * volume``), ``centroid`` (the mass centre of\n"
           "    gravity as ``(x, y, z)`` in world coordinates, independent of\n"
           "    ``origin``), and the mass ``moments_of_inertia``\n"
           "    ``(Ixx, Iyy, Izz)`` and ``products_of_inertia``\n"
           "    ``(Iyz, Izx, Ixy)`` about axes through ``origin`` (the centroid\n"
           "    when ``origin`` is ``None``), parallel to the world axes. The\n"
           "    products are the raw positive integrals ``(int yz dm,\n"
           "    int zx dm, int xy dm)``; an inertia tensor's off-diagonals are\n"
           "    their negatives.\n"
           "\n"
           "Raises ``RuntimeError`` (``SM_ERR_INVALID_INPUT``) for a\n"
           "non-solid / sheet body (see ``is_manifold_solid``), a non-finite\n"
           "``origin``, or a Brep carrying legacy per-entity\n"
           "``SM_AI_MASS_PROPERTIES`` attributes (which would override the\n"
           "uniform density).\n"
           "\n"
           "Wraps: SmApiBrepComputeMassProperties")
        .def("is_manifold_solid", [](SmBrep& self) {
            SmBoolean result = FALSE;
            CHECK_STATUS(SmApiBrepIsManifoldSolid(&self, result));
            return (bool)result;
        }, "``True`` if every edge is shared by exactly two faces (closed "
           "manifold solid); ``False`` for sheet bodies or non-manifold topology.")
        .def("material_census", [](SmBrep& self) {
            long solid_count = 0;
            long void_count = 0;
            CHECK_STATUS(SmApiBrepMaterialCensus(&self, solid_count, void_count));
            py::dict d;
            d["solid_count"] = solid_count;
            d["void_count"] = void_count;
            return d;
        }, "Count material (solid) regions and enclosed void cavities, "
           "counting nested shells unambiguously. Returns "
           "``{'solid_count': int, 'void_count': int}``; a solid with a "
           "cavity holding a nested solid is ``solid_count == 2``, "
           "``void_count == 1``. See the ``material_census`` free function "
           "for full notes. Wraps: SmApiBrepMaterialCensus")
        .def("copy", &smpy_brep_copy, py::return_value_policy::reference,
           "Deep-copy this Brep into a fresh handle. Mutating the result "
           "does not affect the original. Geometry types are preserved; "
           "NURBS are not promoted to analytics. Useful before applying in-place "
           "mutators (``fillet_edges``, ``cut``, ``translate``, ...) when "
           "the original must be preserved.")
        .def("__copy__", &smpy_brep_copy, py::return_value_policy::reference,
           "Support ``copy.copy``: a deep copy, same as ``copy()``. A shallow "
           "copy would share topology, so edits to one would change both.")
        .def("__deepcopy__", [](SmBrep& self, py::dict) { return smpy_brep_copy(self); },
           py::return_value_policy::reference, py::arg("memo"),
           "Support ``copy.deepcopy``: same as ``copy()``.")
        .def("face_count", [](SmBrep& self) {
            SmTArray<SmFace*> f; self.GetFaces(f); return f.GetSize();
        }, "Number of faces in the Brep.")
        .def("edge_count", [](SmBrep& self) {
            SmTArray<SmEdge*> e; self.GetEdges(e); return e.GetSize();
        }, "Number of edges in the Brep.")
        .def("vertex_count", [](SmBrep& self) {
            SmTArray<SmVertex*> v; self.GetVertices(v); return v.GetSize();
        }, "Number of vertices in the Brep.")
        .def("info", [](SmBrep& self) {
            SmTArray<SmFace*>   f; self.GetFaces(f);
            SmTArray<SmEdge*>   e; self.GetEdges(e);
            SmTArray<SmVertex*> v; self.GetVertices(v);

            SmBoolean manifold = FALSE;
            CHECK_STATUS(SmApiBrepIsManifoldSolid(&self, manifold));

            double vol = 0.0;
            if (manifold)
                CHECK_STATUS(SmApiBrepComputeVolume(&self, 1.0e-3, vol));

            SmPoint3d mn, mx;
            CHECK_STATUS(SmApiBrepBoundingBox(&self, FALSE, mn, mx));

            py::dict d;
            d["faces"]    = f.GetSize();
            d["edges"]    = e.GetSize();
            d["vertices"] = v.GetSize();
            d["manifold"] = (bool)manifold;
            d["volume"]   = vol;
            d["bounds"]   = py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
            d["center"]   = py::make_tuple(0.5 * (mn.x + mx.x),
                                           0.5 * (mn.y + mx.y),
                                           0.5 * (mn.z + mx.z));
            return d;
        }, "Pure: returns dict; inputs unchanged.\n"
           "\n"
           "Consolidated introspection dict.\n"
           "\n"
           "Returns a dict with keys ``faces``, ``edges``, ``vertices`` "
           "(int counts), ``manifold`` (bool), ``volume`` (float, ``0.0`` "
           "for non-manifold bodies), ``bounds`` as ``((min_x, min_y, "
           "min_z), (max_x, max_y, max_z))``, and ``center`` as "
           "``(x, y, z)``. Quick one-call summary for debugging and "
           "agent introspection.")
        .def("__repr__", [](SmBrep& self) {
            SmTArray<SmFace*>   f; self.GetFaces(f);
            SmTArray<SmEdge*>   e; self.GetEdges(e);
            SmTArray<SmVertex*> v; self.GetVertices(v);
            return std::string("<Brep faces=") + std::to_string(f.GetSize())
                 + " edges=" + std::to_string(e.GetSize())
                 + " vertices=" + std::to_string(v.GetSize()) + ">";
        })
        .def_static("read_from_file",
             [](const std::string& filename, py::object ascii, bool rebuild_uv_trim_curves) {
                 return smpy_read_brep_from_file(filename, ascii, rebuild_uv_trim_curves);
             },
             py::return_value_policy::reference,
             py::arg("filename"), py::arg("ascii") = py::none(),
             py::arg("rebuild_uv_trim_curves") = false,
             "Pure: returns new Brep; inputs unchanged.\n"
             "\n"
             "Deserialize a native SMLib Brep file from disk.\n"
             "\n"
             "Args:\n"
             "    filename: Path to an SMLib native Brep file.\n"
             "    ascii: Format selector. Use ``None`` to auto-detect,\n"
             "        ``True`` for ASCII, or ``False`` for binary.\n"
             "    rebuild_uv_trim_curves: Rebuild UV trim curves after loading.\n"
             "\n"
             "Returns:\n"
             "    Brep: loaded model owned by the module context.\n"
             "\n"
             "Notes:\n"
             "    Recognized SMLib part containers are rejected; load them with\n"
             "    ``read_part_from_file`` and the matching ASCII/binary selector.\n"
             "\n"
             "See Also:\n"
             "    read_brep_from_file: Flat-function spelling.\n"
             "    write_to_file: Serialize a Brep to native SMLib format.\n"
             "    read_part_from_file: Deserialize an SMLib part container.\n"
             "\n"
             "Wraps: SmApiReadBrepFromFile")
        .def("write_to_file",
             [](SmBrep& self, const std::string& filename, bool ascii) {
                 SmPyTCharPath path = to_tchar_path(filename);
                 CHECK_STATUS(SmApiWriteBrepToFile(&self, path.c_str(), ascii));
             },
             py::arg("filename"), py::arg("ascii") = true,
             "Serialize this Brep to disk via SmApiWriteBrepToFile.\n"
             "ASCII mode produces a deterministic text file suitable for textual\n"
             "diff (e.g. compare 'after creation' vs 'after USD round-trip').")
        .def("tessellate", &smpy_tessellate, py::return_value_policy::reference,
           py::kw_only(),
           py::arg("chord_height_tolerance") = SmTessellationDefaults::kChordHeightTol,
           py::arg("curve_angle_tolerance_deg") = SmTessellationDefaults::kCurveAngleTolDeg,
           py::arg("surface_angle_tolerance_deg") = SmTessellationDefaults::kSurfaceAngleTolDeg,
           py::arg("max_edge_length") = SmTessellationDefaults::kMaxEdgeLength,
           py::arg("max_aspect_ratio") = SmTessellationDefaults::kMaxAspectRatio,
           py::arg("allow_partial") = false, py::arg("return_failures") = false, py::arg("return_report") = false,
           "Pure: returns new PolyBrep; this Brep is unchanged.\n"
           "\n"
           "Tessellate this Brep's exact NURBS geometry to a triangular "
           "``PolyBrep`` mesh. Method form of the flat ``sm.tessellate(brep, ...)``; "
           "the kwargs and defaults are identical (see that function for the "
           "full parameter discussion). All tessellation parameters are "
           "keyword-only. Both angles default to ``25`` degrees; "
           "chord height and the remaining caps default to ``0`` (disabled).\n"

           "Partial output requires allow_partial and either return_failures or return_report.\n"
           "TessellationError retains report, failures, status and status_name on rejection.\n"
           "Returns ``(mesh, report)`` with return_report, or ``(mesh, failures)`` with return_failures; see ``sm.tessellate``.\n"
           "Missing diagnostics raises ValueError; kernel and output errors always raise.\n"
           "\n"
           "Wraps: SmApiTessellate");

    py::class_<SmFace,    std::unique_ptr<SmFace,    py::nodelete>>(m, "Face")
        .def("bounding_box", [](SmFace& self, bool tight) {
            SmPoint3d mn, mx;
            CHECK_STATUS(SmApiFaceBoundingBox(&self, tight ? TRUE : FALSE, mn, mx));
            return py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
        }, py::arg("tight") = false,
           "Axis-aligned bounding box of this face as ``((min_x, min_y, min_z), "
           "(max_x, max_y, max_z))``. ``tight=True`` samples the surface inside "
           "the trim boundary for a minimal box (expensive).")
        .def("area", [](SmFace& self, double accuracy) {
            double a = 0.0;
            CHECK_STATUS(SmApiFaceComputeArea(&self, accuracy, a));
            return a;
        }, py::arg("relative_accuracy") = 1.0e-3,
           "Surface area of this face within its trim boundary, in square "
           "modeling units. Positive regardless of face orientation. "
           "``relative_accuracy`` clamped to ``[1e-4, 1e-1]``.")
        // -- Topology accessors -------------------------------------------
        .def("surface", &SmFace::GetSurface, py::return_value_policy::reference,
             "Underlying NURBS / analytic surface this face is trimmed from. "
             "The surface is owned by the Brep; do not delete. Release this "
             "borrowed handle before a representation-changing "
             "``turn_to_nurbs`` or scale, then reacquire it from the mutated "
             "face. Use the result with ``sm.evaluate_surface_point`` etc.")
        .def("uv_domain", [](SmFace& self) {
            SmExtent2d d = self.GetUVDomain();
            SmPoint2d mn = d.GetMin(), mx = d.GetMax();
            return py::make_tuple(py::make_tuple(mn.x, mn.y),
                                  py::make_tuple(mx.x, mx.y));
        }, "Trim domain on the parent surface as ``((u_min, v_min), "
           "(u_max, v_max))``. This is the rectangular UV box that bounds "
           "the face's outer loop, not the natural surface domain.")
        .def("edges", [](SmFace& self) {
            SmTArray<SmEdge*> arr;
            self.GetEdges(arr);
            std::vector<SmEdge*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Every edge bounding this face, across all loops.")
        .def("vertices", [](SmFace& self) {
            SmTArray<SmVertex*> arr;
            self.GetVertices(arr);
            std::vector<SmVertex*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Every vertex on this face's boundary, across all loops.")
        .def("outer_loop_edges", [](SmFace& self) {
            SmTArray<SmEdge*> arr;
            self.GetOuterLoopEdges(arr);
            std::vector<SmEdge*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "Edges of the outer trim loop only. Use ``edges()`` to also "
           "include holes.")
        .def("boundary_loops", &face_boundary_loops,
           "Pure: returns list; inputs unchanged.\n"
           "\n"
           "Return the oriented boundary loops of this face.\n"
           "\n"
           "Returns:\n"
           "    list[list[tuple[Edge, bool]]]: Boundary loops in SMLib order,\n"
           "        with the outer loop first. Each item is ``(edge, forward)``;\n"
           "        ``forward=True`` traverses from ``edge.start_vertex()`` to\n"
           "        ``edge.end_vertex()``, and ``False`` traverses in reverse.\n"
           "\n"
           "Notes:\n"
           "    A closed loop's starting occurrence is unspecified. Relative\n"
           "    to the parent surface's parameterization, the outer loop is\n"
           "    counterclockwise and inner loops are clockwise. An edge can\n"
           "    occur more than once, for example at a periodic seam. A\n"
           "    vertex-only inner loop is represented by an empty list.\n"
           "\n"
           "    The lists and direction flags are snapshots; each ``Edge`` is\n"
           "    borrowed from the owning Brep. Do not use returned edges after\n"
           "    that Brep is consumed, deleted, or changed by a topology-\n"
           "    modifying operation. Query the face again after such changes.\n"
           "\n"
           "Wraps: SmApiFaceGetBoundaryLoops")
        .def("boundary_outer_loop", &face_boundary_outer_loop,
           "Pure: returns list; inputs unchanged.\n"
           "\n"
           "Return the oriented edge occurrences of this face's outer loop.\n"
           "Equivalent to the first item of ``boundary_loops()`` without\n"
           "returning the inner loops. Each item is ``(edge, forward)`` with\n"
           "the same traversal and borrowed-lifetime contract.\n"
           "\n"
           "See Also:\n"
           "    boundary_loops: Return the outer loop and every inner loop.\n"
           "\n"
           "Wraps: SmApiFaceGetBoundaryLoops")
        .def("point_at_uv", [](SmFace& self, double u, double v) {
            SmVector2d uv(u, v);
            SmVector3d pt;
            CHECK_STATUS(SmApiEvaluateSurfacePoint(self.GetSurface(), uv, pt));
            return vec_to_tuple(pt);
        }, py::arg("u"), py::arg("v"),
           "3D point on the face's surface at parameter ``(u, v)`` as "
           "``(x, y, z)``. ``(u, v)`` should lie in ``uv_domain()``; "
           "evaluation outside the trim is allowed but the point may not "
           "lie on the trimmed face.")
        .def("normal_at_uv", [](SmFace& self, double u, double v) {
            SmVector2d uv(u, v);
            SmVector3d n;
            CHECK_STATUS(SmApiEvaluateSurfaceNormal(self.GetSurface(), uv, n));
            return vec_to_tuple(n);
        }, py::arg("u"), py::arg("v"),
           "Unit *surface* normal at parameter ``(u, v)`` as "
           "``(nx, ny, nz)`` - the cross product of the surface's UV "
           "partials produced by ``SmApiEvaluateSurfaceNormal``.\n"
           "\n"
           "This is the surface's intrinsic normal and does **not** "
           "account for the face's orientation: faces whose orientation "
           "is ``SM_OT_OPPOSITE`` relative to their underlying surface "
           "produce a normal that points *inward* (into the solid). "
           "When a consistently outward face-oriented normal is "
           "required, callers should negate this vector for "
           "opposite-oriented faces.")
        .def("outward_normal_at_uv", [](SmFace& self, double u, double v) {
            SmVector2d uv(u, v);
            SmVector3d n;
            CHECK_STATUS(SmApiEvaluateSurfaceNormal(self.GetSurface(), uv, n));

            SmBrep* brep = self.GetBrep();
            SmFaceuse* upward = self.GetUpwardFaceuse();
            if (brep && upward && upward->GetShell()
                && brep->GetInfiniteRegion() != upward->GetShell()->GetRegion())
            {
                n *= -1.0;
            }
            return vec_to_tuple(n);
        }, py::arg("u"), py::arg("v"),
           "Evaluate the outward face normal at parameter ``(u, v)``.\n"
           "\n"
           "Args:\n"
           "    u: Surface U parameter.\n"
           "    v: Surface V parameter.\n"
           "\n"
           "Returns:\n"
           "    tuple: Unit normal vector as ``(nx, ny, nz)``.\n"
           "\n"
           "Notes:\n"
           "    This starts from the underlying surface normal and flips it "
           "when the face's upward faceuse is not in the Brep's infinite "
           "region, matching the orientation used by SMLib display "
           "faceting.\n"
           "\n"
           "See Also:\n"
           "    normal_at_uv: Raw surface normal before face orientation.\n"
           "\n"
           "Wraps: SmApiEvaluateSurfaceNormal")
        .def("interior_point", [](SmFace& self) {
            SmPoint3d pt;
            CHECK_STATUS(SmApiFaceInternalPoint(&self, pt));
            return point_to_tuple(pt);
        }, "Pure: returns tuple; inputs unchanged.\n"
           "\n"
           "A 3D point ``(x, y, z)`` evaluated on the owning surface at a UV\n"
           "strictly inside the face's trim boundary (interior to the outer\n"
           "loop and outside any holes). If no such point can be evaluated,\n"
           "the call raises ``RuntimeError`` instead of returning an\n"
           "approximation. Useful as a seed for ray tests, region\n"
           "classification, or picking.\n"
           "\n"
           "Wraps: SmApiFaceInternalPoint")
        .def("classify_uv", [](SmFace& self, double u, double v) {
            SmPoint2d uv(u, v);
            int cls = 0;
            CHECK_STATUS(SmApiFaceClassifyUV(&self, uv, cls));
            switch (cls)
            {
                case SM_PC_FACE:   return std::string("inside");
                case SM_PC_UNKNOWN:return std::string("outside");
                case SM_PC_EDGE:   return std::string("boundary");
                case SM_PC_VERTEX: return std::string("vertex");
                default:
                    // SmFace::PointClassify only ever yields the four classes
                    // above.  Any other value means the kernel contract
                    // changed; fail loudly rather than return an undocumented
                    // string that callers doing exhaustive comparisons would
                    // silently mishandle.
                    throw std::runtime_error(
                        "SmApiFaceClassifyUV returned an unsupported "
                        "SmPointClassificationType (" + std::to_string(cls) + ")");
            }
        }, py::arg("u"), py::arg("v"),
           "Pure: returns str; inputs unchanged.\n"
           "\n"
           "Trim-aware classification of a UV parameter against this face's\n"
           "trim boundary. Returns one of:\n"
           "\n"
           "    ``\"inside\"``   -- ``(u, v)`` is interior to the trimmed face;\n"
           "    ``\"outside\"``  -- outside the outer loop or inside a hole;\n"
           "    ``\"boundary\"`` -- on a bounding edge;\n"
           "    ``\"vertex\"``   -- on a boundary vertex.\n"
           "\n"
           "``(u, v)`` is a parameter on the parent ``surface()`` (see\n"
           "``uv_domain()``). Unlike a plain domain check, a UV inside an\n"
           "inner loop (hole) is reported as ``\"outside\"``.\n"
           "\n"
           "Wraps: SmApiFaceClassifyUV")
        .def("closest_point", [](SmFace& self, py::tuple point) {
            SmPoint3d p = to_point(point);
            SmPoint3d closest;
            double dist = 0;
            CHECK_STATUS(SmApiFaceClosestPoint(&self, p, closest, dist));
            return py::make_tuple(point_to_tuple(closest), dist);
        }, py::arg("point"),
           "Pure: returns tuple; inputs unchanged.\n"
           "\n"
           "Closest point on this trimmed face to a query point.\n"
           "\n"
           "Args:\n"
           "    point: Query point as ``(x, y, z)``.\n"
           "\n"
           "Returns:\n"
           "    tuple[tuple, float]: ``(closest_point, distance)`` where\n"
           "    ``closest_point`` is ``(x, y, z)`` and always lies within\n"
           "    this face's trim boundary (never on a hole or off the\n"
           "    outer loop), unlike a plain closest point on the\n"
           "    underlying (untrimmed) ``surface()``.\n"
           "\n"
           "Wraps: SmApiFaceClosestPoint")
        .def("centroid", [](SmFace& self, double relative_accuracy) {
            double area = 0.0;
            SmPoint3d centroid;
            CHECK_STATUS(SmApiFaceComputeCentroid(&self, relative_accuracy, area, centroid));
            return point_to_tuple(centroid);
        }, py::arg("relative_accuracy") = 1.0e-3,
           "Pure: returns tuple; inputs unchanged.\n"
           "\n"
           "Area centroid of this face within its trim boundary, as\n"
           "``(x, y, z)``. ``relative_accuracy`` clamped to ``[1e-4, 1e-1]``,\n"
           "same as ``area()``.\n"
           "\n"
           "Wraps: SmApiFaceComputeCentroid")
        .def("brep", &SmFace::GetBrep, py::return_value_policy::reference,
             "Parent Brep that owns this face.")
        .def("__repr__", [](SmFace& self) {
            SmTArray<SmEdge*> e; self.GetEdges(e);
            return std::string("<Face edges=") + std::to_string(e.GetSize()) + ">";
        });

    py::class_<SmEdge,    std::unique_ptr<SmEdge,    py::nodelete>>(m, "Edge")
        .def("bounding_box", [](SmEdge& self, bool tight) {
            SmPoint3d mn, mx;
            CHECK_STATUS(SmApiEdgeBoundingBox(&self, tight ? TRUE : FALSE, mn, mx));
            return py::make_tuple(point_to_tuple(mn), point_to_tuple(mx));
        }, py::arg("tight") = false,
           "Axis-aligned bounding box of this edge as ``((min_x, min_y, min_z), "
           "(max_x, max_y, max_z))``. ``tight=True`` samples the curve for a "
           "minimal box (expensive).")
        .def("length", [](SmEdge& self, double accuracy) {
            double L = 0.0;
            CHECK_STATUS(SmApiEdgeComputeLength(&self, accuracy, L));
            return L;
        }, py::arg("desired_accuracy") = 1.0e-6,
           "3D arc length of this edge over its parametric interval, in "
           "modeling units.")
        // -- Topology accessors -------------------------------------------
        .def("curve", &SmEdge::GetCurve, py::return_value_policy::reference,
             "Underlying 3D curve. The edge uses a sub-interval of this "
             "curve's full parameter range (see ``point_at`` for the valid "
             "range). The curve is owned by the Brep; do not delete. Release "
             "this borrowed handle before a representation-changing "
             "``turn_to_nurbs`` or scale, then reacquire it from the mutated "
             "edge.")
        .def("start_vertex", &SmEdge::GetStartVertex, py::return_value_policy::reference,
             "Vertex at the low-parameter end of this edge.")
        .def("end_vertex", &SmEdge::GetEndVertex, py::return_value_policy::reference,
             "Vertex at the high-parameter end of this edge.")
        .def("vertices", [](SmEdge& self) {
            SmVertex *vs = nullptr, *ve = nullptr;
            self.GetVertices(vs, ve);
            return py::make_tuple(vs, ve);
        }, py::return_value_policy::reference,
           "``(start_vertex, end_vertex)`` as a 2-tuple. Equivalent to "
           "``(edge.start_vertex(), edge.end_vertex())`` but in one call.")
        .def("other_vertex", &SmEdge::GetOtherVertex, py::return_value_policy::reference,
             py::arg("vertex"),
             "Whichever endpoint of this edge is *not* ``vertex``. Useful "
             "for walking an edge string: ``next = edge.other_vertex(prev)``. "
             "Returns ``None`` if ``vertex`` is not an endpoint of this edge.")
        .def("faces", [](SmEdge& self) {
            SmTArray<SmFace*> arr;
            self.GetFaces(arr);
            std::vector<SmFace*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "All faces sharing this edge. For a manifold edge in a closed "
           "solid this is exactly two faces; lamina edges have one, "
           "non-manifold edges have three or more.")
        .def("point_at", [](SmEdge& self, double t) {
            // SmApiEvaluateCurve writes through caller-provided storage:
            // pass non-null pointers to opt in to point/d1/d2.
            SmVector3d storage;
            SmVector3d* point = &storage;
            SmVector3d* d1 = nullptr;
            SmVector3d* d2 = nullptr;
            CHECK_STATUS(SmApiEvaluateCurve(self.GetCurve(), t, point, d1, d2));
            return vec_to_tuple(storage);
        }, py::arg("t"),
           "3D point on this edge's curve at parameter ``t`` as ``(x, y, z)``. "
           "``t`` should lie within the curve's natural parameter range; the "
           "edge itself uses a sub-interval of that range.")
        .def("closest_point", [](SmEdge& self, py::tuple point) {
            SmPoint3d p = to_point(point);
            SmPoint3d closest;
            double param = 0, dist = 0;
            CHECK_STATUS(SmApiEdgeClosestPoint(&self, p, closest, param, dist));
            return py::make_tuple(point_to_tuple(closest), param, dist);
        }, py::arg("point"),
           "Pure: returns tuple; inputs unchanged.\n"
           "\n"
           "Closest point on this edge to a query point.\n"
           "\n"
           "Args:\n"
           "    point: Query point as ``(x, y, z)``.\n"
           "\n"
           "Returns:\n"
           "    tuple[tuple, float, float]: ``(closest_point, parameter,\n"
           "    distance)``. ``closest_point`` is ``(x, y, z)``; ``parameter``\n"
           "    is the curve parameter of ``closest_point`` (within\n"
           "    ``parameter_range()``, never on a part of the curve outside\n"
           "    this edge's trim interval).\n"
           "\n"
           "Wraps: SmApiEdgeClosestPoint")
        .def("parameter_range", [](SmEdge& self) {
            double lo = 0, hi = 0;
            CHECK_STATUS(SmApiEdgeParameterRange(&self, lo, hi));
            return py::make_tuple(lo, hi);
        }, "Pure: returns tuple; inputs unchanged.\n"
           "\n"
           "This edge's parametric interval on its curve as\n"
           "``(t_min, t_max)``. Distinct from ``curve().parameter_range()``,\n"
           "which is the underlying curve's full natural range -- an edge\n"
           "typically uses only a sub-interval of it.\n"
           "\n"
           "Wraps: SmApiEdgeParameterRange")
        .def("classify_parameter", [](SmEdge& self, double t) {
            int cls = 0;
            CHECK_STATUS(SmApiEdgeClassifyParameter(&self, t, cls));
            switch (cls)
            {
                case SM_PC_EDGE:    return std::string("inside");
                case SM_PC_VERTEX:  return std::string("endpoint");
                case SM_PC_UNKNOWN: return std::string("outside");
                default:
                    // SmApiEdgeClassifyParameter only ever yields the three
                    // classes above; fail loudly on a kernel-contract change
                    // rather than return an undocumented string.
                    throw std::runtime_error(
                        "SmApiEdgeClassifyParameter returned an unsupported "
                        "SmPointClassificationType (" + std::to_string(cls) + ")");
            }
        }, py::arg("t"),
           "Pure: returns str; inputs unchanged.\n"
           "\n"
           "Classify a curve parameter ``t`` against this edge's\n"
           "``parameter_range()``. Returns one of:\n"
           "\n"
           "    ``\"inside\"``   -- strictly within the edge's interval;\n"
           "    ``\"endpoint\"`` -- within tolerance of either end;\n"
           "    ``\"outside\"``  -- outside the interval entirely.\n"
           "\n"
           "Wraps: SmApiEdgeClassifyParameter")
        .def("tangent_at", [](SmEdge& self, double t) {
            SmVector3d tangent;
            CHECK_STATUS(SmApiEdgeTangent(&self, t, tangent));
            return vec_to_tuple(tangent);
        }, py::arg("t"),
           "Pure: returns tuple; inputs unchanged.\n"
           "\n"
           "Unit tangent vector of this edge's curve at parameter ``t``, as\n"
           "``(x, y, z)``. ``t`` should lie within ``parameter_range()``.\n"
           "\n"
           "The vector follows the underlying curve's natural direction, which is\n"
           "opposite the edge's start->end traversal sense on a reversed edge.\n"
           "\n"
           "Wraps: SmApiEdgeTangent")
        .def("brep", &SmEdge::GetBrep, py::return_value_policy::reference,
             "Parent Brep that owns this edge.")
        .def("__repr__", [](SmEdge& self) {
            SmVertex *vs = nullptr, *ve = nullptr;
            self.GetVertices(vs, ve);
            SmPoint3d a = vs ? vs->GetPoint() : SmPoint3d(0, 0, 0);
            SmPoint3d b = ve ? ve->GetPoint() : SmPoint3d(0, 0, 0);
            return std::string("<Edge (")
                 + std::to_string(a.x) + ", " + std::to_string(a.y) + ", " + std::to_string(a.z)
                 + ") -> (" + std::to_string(b.x) + ", " + std::to_string(b.y) + ", " + std::to_string(b.z)
                 + ")>";
        });

    py::class_<SmVertex,  std::unique_ptr<SmVertex,  py::nodelete>>(m, "Vertex")
        .def("point", [](SmVertex& self) {
            SmPoint3d pt;
            CHECK_STATUS(SmApiVertexGetPoint(&self, pt));
            return point_to_tuple(pt);
        }, "3D position of this vertex as ``(x, y, z)``.")
        // -- Topology accessors -------------------------------------------
        .def("edges", [](SmVertex& self) {
            SmTArray<SmEdge*> arr;
            self.GetEdges(arr);
            std::vector<SmEdge*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "All edges incident on this vertex. Empty for an isolated "
           "shell vertex.")
        .def("faces", [](SmVertex& self) {
            SmTArray<SmFace*> arr;
            self.GetFaces(arr);
            std::vector<SmFace*> v;
            for (ULONG i = 0; i < arr.GetSize(); i++) v.push_back(arr[i]);
            return v;
        }, py::return_value_policy::reference,
           "All faces incident on this vertex (via its edges).")
        .def("brep", &SmVertex::GetBrep, py::return_value_policy::reference,
             "Parent Brep that owns this vertex.")
        .def("__repr__", [](SmVertex& self) {
            SmPoint3d pt; SmApiVertexGetPoint(&self, pt);
            return std::string("<Vertex (")
                 + std::to_string(pt.x) + ", "
                 + std::to_string(pt.y) + ", "
                 + std::to_string(pt.z) + ")>";
        });
    py::class_<SmCurve, SmObject, std::unique_ptr<SmCurve, py::nodelete>>(m, "Curve")
        .def("parameter_range", [](SmCurve& self) {
            SmExtent1d ivl = self.GetNaturalInterval();
            return py::make_tuple(ivl.GetMin(), ivl.GetMax());
        }, "Natural parameter range as ``(t_min, t_max)``. ``evaluate(t)`` "
           "is well-defined for ``t`` in this interval.")
        .def("start_point", [](SmCurve& self) {
            SmExtent1d ivl = self.GetNaturalInterval();
            SmPoint3d pt;
            CHECK_STATUS(self.EvaluatePoint(ivl.GetMin(), pt));
            return point_to_tuple(pt);
        }, "3D position at the start of the natural parameter range "
           "(``t_min``).")
        .def("end_point", [](SmCurve& self) {
            SmExtent1d ivl = self.GetNaturalInterval();
            SmPoint3d pt;
            CHECK_STATUS(self.EvaluatePoint(ivl.GetMax(), pt));
            return point_to_tuple(pt);
        }, "3D position at the end of the natural parameter range "
           "(``t_max``).")
        .def("evaluate", [](SmCurve& self, double t) {
            SmPoint3d pt;
            CHECK_STATUS(self.EvaluatePoint(t, pt));
            return point_to_tuple(pt);
        }, py::arg("t"),
           "3D position at parameter ``t`` as ``(x, y, z)``. ``t`` must lie "
           "within ``parameter_range()``. For derivatives use the flat "
           "``sm.evaluate_curve``.")
        .def("length", [](SmCurve& self, double accuracy) {
            SmExtent1d ivl = self.GetNaturalInterval();
            double L = 0.0;
            CHECK_STATUS(self.Length(ivl, accuracy, L));
            return L;
        }, py::arg("desired_accuracy") = 1.0e-6,
           "3D arc length over the natural parameter range, in modeling "
           "units. ``desired_accuracy`` is the maximum acceptable absolute "
           "error of the integration.")
        .def("is_closed", [](SmCurve& self, double tolerance) {
            return (bool)self.IsClosed(self.GetNaturalInterval(), tolerance);
        }, py::arg("tolerance") = 0.0,
           "``True`` if ``start_point()`` and ``end_point()`` coincide within "
           "``tolerance`` (in modeling units; ``0.0`` uses the kernel's "
           "default tolerance).")
        .def("is_planar", [](SmCurve& self, double tolerance) {
            return (bool)self.IsPlanar(tolerance);
        }, py::arg("tolerance") = SM_EFF_ZERO,
           "``True`` if every control point lies within ``tolerance`` of a "
           "best-fit plane. Lines and circles are trivially planar.")
        .def("__repr__", [](SmCurve& self) {
            SmExtent1d ivl = self.GetNaturalInterval();
            return std::string("<Curve t=[")
                 + std::to_string(ivl.GetMin()) + ", "
                 + std::to_string(ivl.GetMax()) + "]>";
        });
    py::class_<SmBSplineCurve, SmCurve, std::unique_ptr<SmBSplineCurve, py::nodelete>>(m, "BSplineCurve");
    py::class_<SmLine,    SmBSplineCurve, std::unique_ptr<SmLine,    py::nodelete>>(m, "Line");
    py::class_<SmCircle,  SmBSplineCurve, std::unique_ptr<SmCircle,  py::nodelete>>(m, "Circle")
        .def("center", [](SmCircle& self) {
            SmAxis2Placement frame; double r = 0.0;
            CHECK_STATUS(self.GetCanonical(frame, r));
            return point_to_tuple(frame.GetOrigin());
        }, "Center point ``(x, y, z)``.")
        .def("radius", [](SmCircle& self) {
            return self.GetRadius();
        }, "Radius in modeling units.")
        .def("normal", [](SmCircle& self) {
            SmAxis2Placement frame; double r = 0.0;
            CHECK_STATUS(self.GetCanonical(frame, r));
            return vec_to_tuple(frame.GetZAxis());
        }, "Unit normal ``(x, y, z)`` of the circle's plane (x_axis x y_axis).")
        .def("x_axis", [](SmCircle& self) {
            SmAxis2Placement frame; double r = 0.0;
            CHECK_STATUS(self.GetCanonical(frame, r));
            return vec_to_tuple(frame.GetXAxis());
        }, "Unit X axis ``(x, y, z)`` (the 0-degree direction).")
        .def("y_axis", [](SmCircle& self) {
            SmAxis2Placement frame; double r = 0.0;
            CHECK_STATUS(self.GetCanonical(frame, r));
            return vec_to_tuple(frame.GetYAxis());
        }, "Unit Y axis ``(x, y, z)`` (the 90-degree direction).");
    py::class_<SmSurface, SmObject, std::unique_ptr<SmSurface, py::nodelete>>(m, "Surface")
        .def("u_domain", [](SmSurface& self) {
            SmExtent2d uv = self.GetNaturalUVDomain();
            return py::make_tuple(uv.GetMin().x, uv.GetMax().x);
        }, "Natural U parameter range as ``(u_min, u_max)``.")
        .def("v_domain", [](SmSurface& self) {
            SmExtent2d uv = self.GetNaturalUVDomain();
            return py::make_tuple(uv.GetMin().y, uv.GetMax().y);
        }, "Natural V parameter range as ``(v_min, v_max)``.")
        .def("uv_domain", [](SmSurface& self) {
            SmExtent2d uv = self.GetNaturalUVDomain();
            return py::make_tuple(
                py::make_tuple(uv.GetMin().x, uv.GetMin().y),
                py::make_tuple(uv.GetMax().x, uv.GetMax().y));
        }, "Natural UV domain as ``((u_min, v_min), (u_max, v_max))``. The "
           "natural domain is the surface's intrinsic parametric extent; on "
           "a Brep face this may be wider than the face's trim domain (see "
           "``Face.uv_domain``).")
        .def("evaluate", [](SmSurface& self, double u, double v) {
            SmVector2d uv(u, v);
            SmVector3d pt;
            CHECK_STATUS(SmApiEvaluateSurfacePoint(&self, uv, pt));
            return vec_to_tuple(pt);
        }, py::arg("u"), py::arg("v"),
           "3D position at parametric ``(u, v)`` as ``(x, y, z)``. Wraps "
           "``SmApiEvaluateSurfacePoint`` (also available as the flat "
           "``sm.evaluate_surface_point``).")
        .def("normal", [](SmSurface& self, double u, double v) {
            SmVector2d uv(u, v);
            SmVector3d nm;
            CHECK_STATUS(SmApiEvaluateSurfaceNormal(&self, uv, nm));
            return vec_to_tuple(nm);
        }, py::arg("u"), py::arg("v"),
           "Unit surface normal at ``(u, v)`` as ``(nx, ny, nz)``, computed "
           "as the cross product of the U and V partials and normalized. "
           "Sign follows the surface's intrinsic orientation; on a Brep "
           "face with reversed orientation (``SM_OT_OPPOSITE``) the value "
           "may point inward and the caller must negate it for an outward "
           "face normal. Wraps ``SmApiEvaluateSurfaceNormal``.")
        .def("derivatives", [](SmSurface& self, double u, double v) {
            SmVector2d uv(u, v);
            SmVector3d pt, du, dv;
            CHECK_STATUS(SmApiEvaluateSurfaceDerivatives(&self, uv, pt, du, dv));
            return py::make_tuple(
                vec_to_tuple(pt),
                vec_to_tuple(du),
                vec_to_tuple(dv));
        }, py::arg("u"), py::arg("v"),
           "Position and first U/V partials at ``(u, v)``. Returns "
           "``(point, du, dv)`` as a tuple of three 3-tuples. Wraps "
           "``SmApiEvaluateSurfaceDerivatives``.")
        .def("is_planar", [](SmSurface& self, double tolerance) {
            return (bool)self.IsPlanar(tolerance);
        }, py::arg("tolerance") = SM_EFF_ZERO,
           "``True`` if the surface lies within ``tolerance`` of a best-fit "
           "plane. Plane primitives are trivially planar.")
        .def("is_periodic_u", [](SmSurface& self) {
            return (bool)self.IsPeriodic(self.GetNaturalUVDomain(), SM_SP_U);
        }, "``True`` if the surface's U direction is periodic (e.g. the "
           "longitude direction of a sphere or cylinder).")
        .def("is_periodic_v", [](SmSurface& self) {
            return (bool)self.IsPeriodic(self.GetNaturalUVDomain(), SM_SP_V);
        }, "``True`` if the surface's V direction is periodic.")
        .def("__repr__", [](SmSurface& self) {
            SmExtent2d uv = self.GetNaturalUVDomain();
            return std::string("<Surface u=[")
                 + std::to_string(uv.GetMin().x) + ", "
                 + std::to_string(uv.GetMax().x) + "] v=["
                 + std::to_string(uv.GetMin().y) + ", "
                 + std::to_string(uv.GetMax().y) + "]>";
        });
    // -----------------------------------------------------------------------
    //  Value types (SmPoint3d == SmVector3d via #define)
    // -----------------------------------------------------------------------
    py::class_<SmVector3d>(m, "Vec3")
        .def(py::init<double, double, double>())
        .def(py::init([](py::tuple t) { return to_vec(t); }))
        .def_readwrite("x", &SmVector3d::x)
        .def_readwrite("y", &SmVector3d::y)
        .def_readwrite("z", &SmVector3d::z)
        .def("__repr__", [](const SmVector3d& v) {
            return "Vec3(" + std::to_string(v.x) + ", " +
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });

    py::class_<SmVector2d>(m, "Vec2")
        .def(py::init<double, double>())
        .def_readwrite("x", &SmVector2d::x)
        .def_readwrite("y", &SmVector2d::y)
        .def("__repr__", [](const SmVector2d& v) {
            return "Vec2(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        });

    py::implicitly_convertible<py::tuple, SmVector3d>();

    py::class_<SmAxis2Placement>(m, "Axis2Placement")
        .def(py::init<>())
        .def(py::init([](py::tuple origin, py::tuple x_axis, py::tuple y_axis) {
            return SmAxis2Placement(to_point(origin), to_vec(x_axis), to_vec(y_axis));
        }),
             py::arg("origin"), py::arg("x_axis"), py::arg("y_axis"));
}
