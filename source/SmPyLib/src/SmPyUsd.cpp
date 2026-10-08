// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiUsd.h (USD import/export of Brep and PolyBrep meshes,
// single-prim and whole-stage variants, optional heal-on-import).
// Exposed as the ``_omni_solid.usd`` submodule.  Doesn't pull in
// SmPyCommon.h (and therefore SmSmlibAll.h) -- USD bindings only need
// the BRep/PolyBrep forward types and the error helpers -- but uses the
// same CHECK_STATUS contract as the rest of the bindings via SmPyError.h
// so failures surface the wrapped SmApi* name and SM_* identifier.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstdint>
#include <utility>

#include <SmApiUsd.h>
#include <SmBrep.h>
#include <SmPoly.h>

#include "SmPyError.h"

namespace py = pybind11;

#define CHECK_STATUS(expr)                                              \
    do {                                                                \
        SmPyErrorTrail::clear();                                        \
        SmStatus _smPyStat = (expr);                                    \
        if (_smPyStat != SM_SUCCESS)                                    \
            SmPyRaise(_smPyStat, #expr, __FILE__, __LINE__);            \
    } while (0)

namespace
{

// Raise for a failed file-level call: the status alone does not say which items failed
// or why, so name them (up to 10) when there are any.
template <typename Result, typename Label>
[[noreturn]] void RaiseWithFailures(SmStatus status, const char* pEntry, const std::vector<Result>& crResults, Label label)
{
    std::string failures;
    size_t failureCount = 0;
    for (const Result& r : crResults)
    {
        if (r.m_sStatus == SM_SUCCESS || ++failureCount > 10)
            continue;
        failures += "; " + label(r) + ": " + r.m_sMessage;
    }
    if (failureCount == 0)
        SmPyRaise(status, pEntry, __FILE__, __LINE__);
    if (failureCount > 10)
        failures += "; and " + std::to_string(failureCount - 10) + " more";
    SmPyErrorTrail::clear();
    throw std::runtime_error(std::string(pEntry).substr(0, std::string(pEntry).find('(')) + " failed: " +
                             sm_status_name(status) + " (" + std::to_string(static_cast<long>(status)) + ")" + failures);
}

struct SmPyUsdBrepImportReport
{
    std::vector<SmBrep*> m_vBreps;
    std::vector<SmApiUsdBrepImportResult> m_vItems;
};

static py::list BrepReferencesToList(const std::vector<SmBrep*>& rBreps)
{
    py::list breps;
    for (SmBrep* pBrep : rBreps)
    {
        breps.append(py::cast(pBrep, py::return_value_policy::reference));
    }
    return breps;
}

} // namespace

void bind_usd(py::module_& parent)
{
    auto m = parent.def_submodule("usd",
        "USD import/export: BrepArray <-> Brep and UsdGeomMesh <-> PolyBrep");

    py::class_<SmApiUsdBrepImportResult>(m, "BrepImportResult",
        "Read-only, hashable outcome of one attempted packed-Brep import.")
        .def_property_readonly("prim_path", [](const SmApiUsdBrepImportResult& rResult) {
            return rResult.m_sPrimPath;
        })
        .def_property_readonly("packed_index", [](const SmApiUsdBrepImportResult& rResult) -> py::object {
            if (rResult.m_iPackedBrepIndex < 0)
                return py::none();
            return py::int_(rResult.m_iPackedBrepIndex);
        })
        .def_property_readonly("brep", [](const SmApiUsdBrepImportResult& rResult) -> py::object {
            if (rResult.m_pBrep == nullptr)
                return py::none();
            return py::cast(rResult.m_pBrep, py::return_value_policy::reference);
        })
        .def_property_readonly("status_code", [](const SmApiUsdBrepImportResult& rResult) {
            return static_cast<long>(rResult.m_sStatus);
        })
        .def_property_readonly("status_name", [](const SmApiUsdBrepImportResult& rResult) {
            return std::string(sm_status_name(rResult.m_sStatus));
        })
        .def_property_readonly("message", [](const SmApiUsdBrepImportResult& rResult) {
            return rResult.m_sMessage;
        })
        .def_property_readonly("ok", [](const SmApiUsdBrepImportResult& rResult) {
            return rResult.m_sStatus == SM_SUCCESS;
        })
        .def("__eq__", [](const SmApiUsdBrepImportResult& rLeft,
                           const py::object& rOther) {
            if (!py::isinstance<SmApiUsdBrepImportResult>(rOther))
                return false;
            const SmApiUsdBrepImportResult& rRight =
                rOther.cast<const SmApiUsdBrepImportResult&>();
            return rLeft.m_sPrimPath == rRight.m_sPrimPath &&
                   rLeft.m_iPackedBrepIndex == rRight.m_iPackedBrepIndex &&
                   rLeft.m_pBrep == rRight.m_pBrep &&
                   rLeft.m_sStatus == rRight.m_sStatus &&
                   rLeft.m_sMessage == rRight.m_sMessage;
        })
        .def("__hash__", [](const SmApiUsdBrepImportResult& rResult) {
            return py::hash(py::make_tuple(
                rResult.m_sPrimPath, rResult.m_iPackedBrepIndex,
                reinterpret_cast<std::uintptr_t>(rResult.m_pBrep),
                static_cast<long>(rResult.m_sStatus), rResult.m_sMessage));
        });

    py::class_<SmApiUsdMeshTessellationResult>(m, "MeshTessellationResult",
        "Read-only outcome of tessellating one packed Brep in ``tessellate_file``.")
        .def_property_readonly("prim_path", [](const SmApiUsdMeshTessellationResult& r) { return r.m_sPrimPath; })
        .def_property_readonly("packed_index", [](const SmApiUsdMeshTessellationResult& r) -> py::object {
            if (r.m_iPackedBrepIndex < 0)
                return py::none(); // whole-prim failure
            return py::int_(r.m_iPackedBrepIndex);
        })
        .def_property_readonly("mesh_path", [](const SmApiUsdMeshTessellationResult& r) -> py::object {
            if (r.m_sMeshPath.empty())
                return py::none();
            return py::str(r.m_sMeshPath);
        })
        .def_property_readonly("status_code", [](const SmApiUsdMeshTessellationResult& r) {
            return static_cast<long>(r.m_sStatus);
        })
        .def_property_readonly("status_name", [](const SmApiUsdMeshTessellationResult& r) {
            return std::string(sm_status_name(r.m_sStatus));
        })
        .def_property_readonly("message", [](const SmApiUsdMeshTessellationResult& r) { return r.m_sMessage; })
        .def_property_readonly("ok", [](const SmApiUsdMeshTessellationResult& r) { return r.m_sStatus == SM_SUCCESS; })
        .def_property_readonly("failed_face_count",
                               [](const SmApiUsdMeshTessellationResult& r) { return r.m_lFailedFaceCount; })
        .def_property_readonly("point_count", [](const SmApiUsdMeshTessellationResult& r) { return r.m_lPointCount; });

    py::class_<SmApiUsdBrepArrayHealResult>(m, "BrepArrayHealResult",
        "Read-only outcome of healing one BrepArray prim in ``heal_file``.")
        .def_property_readonly("prim_path", [](const SmApiUsdBrepArrayHealResult& r) -> py::object {
            if (r.m_sPrimPath.empty())
                return py::none(); // a problem with the file as a whole
            return py::str(r.m_sPrimPath);
        })
        .def_property_readonly("layer", [](const SmApiUsdBrepArrayHealResult& r) { return r.m_sLayer; })
        .def_property_readonly("status_code", [](const SmApiUsdBrepArrayHealResult& r) { return static_cast<long>(r.m_sStatus); })
        .def_property_readonly("status_name", [](const SmApiUsdBrepArrayHealResult& r) { return std::string(sm_status_name(r.m_sStatus)); })
        .def_property_readonly("ok", [](const SmApiUsdBrepArrayHealResult& r) { return r.m_sStatus == SM_SUCCESS; })
        .def_property_readonly("brep_count", [](const SmApiUsdBrepArrayHealResult& r) { return r.m_lBrepCount; })
        .def_property_readonly("message", [](const SmApiUsdBrepArrayHealResult& r) { return r.m_sMessage; });

    py::class_<SmPyUsdBrepImportReport>(m, "BrepImportReport",
        "Results from a completed partial BRep import scan.")
        .def_property_readonly("breps", [](const SmPyUsdBrepImportReport& rReport) {
            return BrepReferencesToList(rReport.m_vBreps);
        })
        .def_property_readonly("items", [](const SmPyUsdBrepImportReport& rReport) {
            return rReport.m_vItems;
        })
        .def_property_readonly("failures", [](const SmPyUsdBrepImportReport& rReport) {
            std::vector<SmApiUsdBrepImportResult> failures;
            for (const SmApiUsdBrepImportResult& rItem : rReport.m_vItems)
            {
                if (rItem.m_sStatus != SM_SUCCESS)
                    failures.push_back(rItem);
            }
            return failures;
        })
        .def_property_readonly("complete", [](const SmPyUsdBrepImportReport& rReport) {
            return std::all_of(
                rReport.m_vItems.begin(),
                rReport.m_vItems.end(),
                [](const SmApiUsdBrepImportResult& rItem) {
                    return rItem.m_sStatus == SM_SUCCESS;
                });
        });

    m.def("ensure_plugin_registered", []() {
        CHECK_STATUS(SmApiUsdEnsurePluginRegistered());
    },
       "Register the omniSolid BrepArray schema plugin with USD.\n"
       "\n"
       "Idempotent. ``import usd_brep`` already calls it, and the import/export\n"
       "functions in this submodule register the plugin on demand. Call it to\n"
       "register explicitly at startup (failing fast on a bad or unset\n"
       "``OMNISOLID_PLUGIN_PATH``) or before reading ``BrepArray`` prims\n"
       "through the raw ``pxr`` USD API.\n"
       "\n"
       "Raises:\n"
       "    RuntimeError: if the plugin cannot be registered, e.g.\n"
       "        ``OMNISOLID_PLUGIN_PATH`` is unset or points at a missing or\n"
       "        stale ``schema/omniSolid/resources`` directory.\n"
       "\n"
       "Wraps: SmApiUsdEnsurePluginRegistered");

    // NOTE: ``heal`` defaults to FALSE. TRUE enables the metadata-selected
    // import healer; SMLib-authored data normally skips that pass. Healing can
    // change topology and tolerances, while conversion normalization and
    // approximation occur independently of this option.
    m.def("import_breps", [](const std::string& filename, bool heal, bool allowPartial) -> py::object {
        std::vector<SmBrep*> breps;
        if (!allowPartial)
        {
            CHECK_STATUS(SmApiUsdImportBreps(filename.c_str(), breps, heal ? TRUE : FALSE));
            return BrepReferencesToList(breps);
        }

        std::vector<SmApiUsdBrepImportResult> items;
        CHECK_STATUS(SmApiUsdImportBreps(
            filename.c_str(),
            breps,
            heal ? TRUE : FALSE,
            TRUE,
            &items));
        // Expected member failures can populate the callback trail even though
        // a completed partial scan returns SM_SUCCESS. Do not retain that trail
        // after constructing a successful Python result.
        SmPyErrorTrail::clear();

        SmPyUsdBrepImportReport report;
        report.m_vBreps = std::move(breps);
        report.m_vItems = std::move(items);
        return py::cast(std::move(report));
    }, py::arg("filename"), py::arg("heal") = false, py::kw_only(), py::arg("allow_partial") = false,
       "Load every Brep prim from a USD stage.\n"
       "\n"
       "Args:\n"
       "    filename: Path to a USD file (``.usda``, ``.usdc``, or\n"
       "        ``.usd``).\n"
       "    heal: Enable metadata-selected healing. Defaults to ``False``\n"
       "        (the C API default is ``True``). SMLib-authored data normally\n"
       "        skips the healer even when enabled. Healing can change topology\n"
       "        and tolerances; there is no fixed tolerance multiplier. Disabling\n"
       "        healing does not disable conversion normalization or approximation.\n"
       "    allow_partial: If ``True``, attempt every locatable packed Brep\n"
       "        and retain successful imports when another member fails.\n"
       "        Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    list[Brep] | BrepImportReport: A list in strict mode (the\n"
       "        default), or a structured report when ``allow_partial=True``.\n"
       "\n"
       "Notes:\n"
       "    Strict mode is all-or-nothing and raises if any packed Brep fails.\n"
       "    Partial mode reports member and prim-level failures without\n"
       "    raising after a completed scan; stage-level and fatal process\n"
       "    failures still raise.\n"
       "    Report items are ordered by stage traversal and packed index. A\n"
       "    prim-level failure has ``packed_index=None``. Successful item\n"
       "    ``brep`` values alias objects in the report's ``breps`` list.\n"
       "\n"
       "See Also:\n"
       "    import_brep: Single-prim variant by ``SdfPath``.\n"
       "    export_breps: Round-trip counterpart.\n"
       "    heal_brep: Apply the healer after import without re-importing.\n"
       "\n"
       "Wraps: SmApiUsdImportBreps");

    m.def("import_brep", [](const std::string& filename, const std::string& primPath, bool heal) {
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiUsdImportBrep(filename.c_str(), primPath.c_str(), r, heal ? TRUE : FALSE));
        return r;
    }, py::return_value_policy::reference,
       py::arg("filename"), py::arg("prim_path"), py::arg("heal") = false,
       "Load a single Brep prim from a USD stage by ``SdfPath``.\n"
       "\n"
       "Args:\n"
       "    filename: Path to a USD file.\n"
       "    prim_path: Absolute SdfPath of the target prim, e.g.\n"
       "        ``\"/World/Brep0\"``. Must point to a prim with\n"
       "        ``BrepArray`` data.\n"
       "    heal: If ``True``, enable metadata-selected healing of the Brep.\n"
       "        Defaults to ``False`` (see ``import_breps`` for the\n"
       "        rationale and side-effects).\n"
       "\n"
       "Returns:\n"
       "    Brep: the loaded Brep. Caller owns the object.\n"
       "\n"
       "Raises:\n"
       "    RuntimeError: if the prim cannot be read or any packed Brep in\n"
       "        the selected ``BrepArray`` fails conversion.\n"
       "\n"
       "Notes:\n"
       "    More efficient than ``import_breps`` plus filtering when you\n"
       "    know the specific prim you want.\n"
       "\n"
       "See Also:\n"
       "    import_breps: Whole-stage variant.\n"
       "    export_brep: Single-Brep export counterpart.\n"
       "\n"
       "Wraps: SmApiUsdImportBrep");

    // Subject-first ordering: ``brep`` precedes ``filename`` to match the
    // rest of the modeling API (every operation is brep-first).  ``filename``
    // is keyword-only on the ``export_uv_curves`` side via ``py::kw_only``
    // below; itself stays positional so the common ``export_brep(b, "x.usda")``
    // call site reads naturally.
    m.def("export_brep", [](SmBrep* brep, const std::string& filename, bool uvCurves, bool shrinkFaceDomains) {
        CHECK_STATUS(SmApiUsdExportBrep(filename.c_str(), brep, uvCurves ? TRUE : FALSE, shrinkFaceDomains ? TRUE : FALSE));
    }, py::arg("brep"), py::arg("filename"), py::kw_only(), py::arg("export_uv_curves") = true, py::arg("shrink_face_domains") = true,
       "Export a single Brep to a new USD file.\n"
       "\n"
       "Args:\n"
       "    brep: Brep to export directly; ownership is retained by the caller.\n"
       "    filename: Output USD path. Created or overwritten.\n"
       "    export_uv_curves: If ``True`` (default), retain eligible existing\n"
       "        B-spline UV trims on faces exported as NURBS. Analytic-face trims\n"
       "        are omitted. Missing trims may be regenerated by downstream\n"
       "        consumers; this option does not guarantee an exact round-trip.\n"
       "        Keyword-only.\n"
       "    shrink_face_domains: If ``True`` (default), replace unbounded\n"
       "        ``face:range`` values in the exported BrepArray with bounds\n"
       "        computed from UV trim curves. Geometry and source face domains\n"
       "        are not shrunk. Set ``False`` to skip trim-derived bounding.\n"
       "        Analytic ranges still use the exported surface's parameters.\n"
       "        Keyword-only.\n"
       "\n"
       "Notes:\n"
       "    Face-range bounding is best-effort: if the kernel cannot create UV\n"
       "    trim curves for an unbounded face, that face keeps its existing domain.\n"
       "    Generated UV trims remain on the input Brep for reuse, including\n"
       "    when ``export_uv_curves=False``. Export also records the destination\n"
       "    USD path as a Brep attribute and may populate geometry caches.\n"
       "    It does not promote NURBS surfaces to analytics. Export ``brep.copy()``\n"
       "    instead if these input-side changes are unwanted. Do not concurrently\n"
       "    operate on the same Brep while exporting it.\n"
       "\n"
       "See Also:\n"
       "    export_breps: Multi-Brep variant.\n"
       "    append_breps: Add to an existing stage instead of creating one.\n"
       "    import_brep: Round-trip counterpart.\n"
       "\n"
       "Wraps: SmApiUsdExportBrep");

    m.def("export_breps", [](std::vector<SmBrep*>& breps, const std::string& filename, bool uvCurves, bool shrinkFaceDomains) {
        CHECK_STATUS(SmApiUsdExportBreps(filename.c_str(), breps, uvCurves ? TRUE : FALSE, shrinkFaceDomains ? TRUE : FALSE));
    }, py::arg("breps"), py::arg("filename"), py::kw_only(), py::arg("export_uv_curves") = true, py::arg("shrink_face_domains") = true,
       "Export multiple Breps to a new USD file.\n"
       "\n"
       "Args:\n"
       "    breps: Breps to export directly; ownership is retained by the caller.\n"
       "    filename: Output USD path. Created or overwritten.\n"
       "    export_uv_curves: If ``True`` (default), include UV trim\n"
       "        curves. See ``export_brep`` for trade-offs. Keyword-only.\n"
       "    shrink_face_domains: If ``True`` (default), replace unbounded\n"
       "        ``face:range`` values in the exported BrepArray with bounds\n"
       "        computed from UV trim curves. Set ``False`` to\n"
       "        skip trim-derived bounding.\n"
       "        Keyword-only.\n"
       "\n"
       "Notes:\n"
       "    Face-range bounding is best-effort: if the kernel cannot create UV\n"
       "    trim curves for an unbounded face, that face keeps its existing domain.\n"
       "\n"
       "See Also:\n"
       "    export_brep: Single-Brep variant; describes retained UV trims,\n"
       "        export-path attributes, and other cache side effects.\n"
       "    append_breps: Add to an existing stage incrementally.\n"
       "    import_breps: Round-trip counterpart.\n"
       "\n"
       "Wraps: SmApiUsdExportBreps");

    m.def("append_breps", [](std::vector<SmBrep*>& breps, const std::string& filename, bool uvCurves, bool shrinkFaceDomains) {
        CHECK_STATUS(SmApiUsdAppendBreps(filename.c_str(), breps, uvCurves ? TRUE : FALSE, shrinkFaceDomains ? TRUE : FALSE));
    }, py::arg("breps"), py::arg("filename"), py::kw_only(), py::arg("export_uv_curves") = true, py::arg("shrink_face_domains") = true,
       "Append Breps to an existing USD file.\n"
       "\n"
       "Args:\n"
       "    breps: Breps to append. New ``BrepArray`` prims are added with\n"
       "        auto-generated names that avoid collisions with existing\n"
       "        prims. Inputs are exported directly with the same retained-trim,\n"
       "        attribute, and cache side effects as ``export_brep``.\n"
       "    filename: Path to an existing USD file. Opened, modified, and\n"
       "        re-saved.\n"
       "    export_uv_curves: If ``True`` (default), include UV trim\n"
       "        curves. See ``export_brep`` for trade-offs. Keyword-only.\n"
       "    shrink_face_domains: If ``True`` (default), replace unbounded\n"
       "        ``face:range`` values in the exported BrepArray with bounds\n"
       "        computed from UV trim curves. Set ``False`` to\n"
       "        skip trim-derived bounding.\n"
       "        Keyword-only.\n"
       "\n"
       "Notes:\n"
       "    Use to build up a scene incrementally without rewriting the\n"
       "    entire file. Existing prims are preserved.\n"
       "    Face-range bounding is best-effort: if the kernel cannot create UV\n"
       "    trim curves for an unbounded face, that face keeps its existing domain.\n"
       "\n"
       "See Also:\n"
       "    export_breps: Create-from-scratch variant.\n"
       "    append_meshes: Mesh counterpart.\n"
       "\n"
       "Wraps: SmApiUsdAppendBreps");

    m.def("import_meshes", [](const std::string& filename) {
        std::vector<SmPolyBrep*> meshes;
        CHECK_STATUS(SmApiUsdImportMeshes(filename.c_str(), meshes));
        return meshes;
    }, py::return_value_policy::reference,
       py::arg("filename"),
       "Load every ``UsdGeomMesh`` prim from a USD stage as a ``PolyBrep``.\n"
       "\n"
       "Args:\n"
       "    filename: Path to a USD file.\n"
       "\n"
       "Returns:\n"
       "    list[PolyBrep]: every active mesh on the stage, in USD\n"
       "    traversal order. Caller owns the returned objects.\n"
       "\n"
       "Notes:\n"
       "    No ``heal`` flag (mesh import does not run the BRep healer).\n"
       "    If the stage contains zero ``UsdGeomMesh`` prims the call\n"
       "    raises ``RuntimeError`` (it does not return an empty list).\n"
       "\n"
       "See Also:\n"
       "    import_mesh: Single-prim variant.\n"
       "    export_meshes: Round-trip counterpart.\n"
       "    import_breps: Brep equivalent (different USD schema).\n"
       "    tessellate: Produce a ``PolyBrep`` from a Brep instead of\n"
       "        loading from disk.\n"
       "\n"
       "Wraps: SmApiUsdImportMeshes");

    m.def("import_mesh", [](const std::string& filename, const std::string& primPath) {
        SmPolyBrep* p = nullptr;
        CHECK_STATUS(SmApiUsdImportMesh(filename.c_str(), primPath.c_str(), p));
        return p;
    }, py::return_value_policy::reference,
       py::arg("filename"), py::arg("prim_path"),
       "Load a single ``UsdGeomMesh`` prim by ``SdfPath`` as a ``PolyBrep``.\n"
       "\n"
       "Args:\n"
       "    filename: Path to a USD file.\n"
       "    prim_path: Absolute SdfPath of the target mesh, e.g.\n"
       "        ``\"/World/Mesh0\"``. The prim must be a ``UsdGeomMesh``.\n"
       "\n"
       "Returns:\n"
       "    PolyBrep: the loaded mesh. Caller owns the object.\n"
       "\n"
       "See Also:\n"
       "    import_meshes: Whole-stage variant.\n"
       "    export_mesh: Single-mesh export counterpart.\n"
       "\n"
       "Wraps: SmApiUsdImportMesh");

    m.def("export_mesh", [](SmPolyBrep* mesh, const std::string& filename) {
        CHECK_STATUS(SmApiUsdExportMesh(filename.c_str(), mesh));
    }, py::arg("mesh"), py::arg("filename"),
       "Export a single ``PolyBrep`` to a new USD file as ``UsdGeomMesh``.\n"
       "\n"
       "Args:\n"
       "    mesh: Mesh to export. Must not be ``None``.\n"
       "    filename: Output USD path. Created or overwritten.\n"
       "\n"
       "Notes:\n"
       "    Writes the mesh as ``/World/Mesh0`` under a fresh ``/World``\n"
       "    xform. Stability: the exporter may update per-vertex index\n"
       "    bookkeeping on ``mesh`` while writing - duplicate or re-import\n"
       "    if you need a pristine object for parallel work. (Brep\n"
       "    export does not have this side-effect.)\n"
       "\n"
       "See Also:\n"
       "    export_meshes: Multi-mesh variant.\n"
       "    append_meshes: Add to an existing stage incrementally.\n"
       "    import_mesh: Round-trip counterpart.\n"
       "\n"
       "Wraps: SmApiUsdExportMesh");

    m.def("export_meshes", [](std::vector<SmPolyBrep*>& meshes, const std::string& filename) {
        CHECK_STATUS(SmApiUsdExportMeshes(filename.c_str(), meshes));
    }, py::arg("meshes"), py::arg("filename"),
       "Export multiple ``PolyBrep`` meshes to a new USD file.\n"
       "\n"
       "Args:\n"
       "    meshes: Meshes to export. Must not contain ``None``. Written\n"
       "        as ``/World/Mesh0``, ``/World/Mesh1``, ... in vector order.\n"
       "    filename: Output USD path. Created or overwritten.\n"
       "\n"
       "Notes:\n"
       "    Same in-memory mesh-mutation behavior as ``export_mesh`` (see\n"
       "    its ``Notes`` block).\n"
       "\n"
       "See Also:\n"
       "    export_mesh: Single-mesh variant.\n"
       "    append_meshes: Add to an existing stage incrementally.\n"
       "    import_meshes: Round-trip counterpart.\n"
       "\n"
       "Wraps: SmApiUsdExportMeshes");

    m.def("append_meshes", [](std::vector<SmPolyBrep*>& meshes, const std::string& filename) {
        CHECK_STATUS(SmApiUsdAppendMeshes(filename.c_str(), meshes));
    }, py::arg("meshes"), py::arg("filename"),
       "Append meshes as new ``UsdGeomMesh`` prims to an existing USD file.\n"
       "\n"
       "Args:\n"
       "    meshes: Meshes to append. Each becomes ``/World/MeshN`` where\n"
       "        ``N`` starts at the count of mesh prims already on the\n"
       "        stage (full-stage traversal). For example, with two\n"
       "        existing meshes the next batch becomes ``Mesh2``,\n"
       "        ``Mesh3``, ...\n"
       "    filename: Path to an existing USD file. Opened, modified, and\n"
       "        re-saved.\n"
       "\n"
       "Notes:\n"
       "    Creates ``/World`` (as ``UsdGeomXform``) if missing. Existing\n"
       "    mesh prims are not modified - only new specs are added. Same\n"
       "    in-memory mesh-mutation behavior as ``export_mesh`` for the\n"
       "    inputs.\n"
       "\n"
       "See Also:\n"
       "    export_meshes: Create-from-scratch variant.\n"
       "    append_breps: Brep counterpart.\n"
       "\n"
       "Wraps: SmApiUsdAppendMeshes");

    m.def("tessellate_file",
        [](const std::string& input, const std::string& output, double chordHeightTol, double curveAngleTolDeg,
           double surfaceAngleTolDeg, double maxEdgeLength, double maxAspectRatio, bool heal, int threads) {
            SmTessellationParams params;
            params.dChordHeightTol = chordHeightTol;
            params.dCurveAngleTolDeg = curveAngleTolDeg;
            params.dSurfaceAngleTolDeg = surfaceAngleTolDeg;
            params.dMaxEdgeLength = maxEdgeLength;
            params.dMaxAspectRatio = maxAspectRatio;
            std::vector<SmApiUsdMeshTessellationResult> results;
            SmPyErrorTrail::clear();
            const SmStatus status = SmApiUsdTessellateFile(input.c_str(), output.c_str(), params, heal ? TRUE : FALSE,
                                                           threads, results);
            if (status != SM_SUCCESS)
                RaiseWithFailures(status, "SmApiUsdTessellateFile()", results, [](const SmApiUsdMeshTessellationResult& r) {
                    return r.m_sPrimPath + (r.m_iPackedBrepIndex >= 0 ? "[" + std::to_string(r.m_iPackedBrepIndex) + "]" : "");
                });
            // Per-Brep failures can populate the error trail on a successful run.
            SmPyErrorTrail::clear();
            return results;
        },
        py::arg("input"), py::arg("output"), py::kw_only(),
        py::arg("chord_height_tolerance") = SmTessellationDefaults::kChordHeightTol,
        py::arg("curve_angle_tolerance_deg") = SmTessellationDefaults::kCurveAngleTolDeg,
        py::arg("surface_angle_tolerance_deg") = SmTessellationDefaults::kSurfaceAngleTolDeg,
        py::arg("max_edge_length") = SmTessellationDefaults::kMaxEdgeLength,
        py::arg("max_aspect_ratio") = SmTessellationDefaults::kMaxAspectRatio,
        py::arg("heal") = false, py::arg("threads") = 0,
       "Tessellate every BrepArray prim in a USD file and write the scene with meshes.\n"
       "\n"
       "Writes a copy of the input stage with one ``UsdGeomMesh`` per Brep, named\n"
       "``tess_<prim>_<brep>`` under the default prim (or ``/Output``). Each mesh is\n"
       "bound to the material of its Brep's brep-element ``GeomSubset``, or else to the\n"
       "BrepArray's material, and keeps the Brep's visibility and purpose. Prims are\n"
       "tessellated in parallel. A failed Brep is reported and skipped; a partial mesh\n"
       "is kept. The input layer is not modified.\n"
       "\n"
       "Args:\n"
       "    input: USD file with BrepArray prims.\n"
       "    output: USD file to write.\n"
       "    chord_height_tolerance, curve_angle_tolerance_deg,\n"
       "        surface_angle_tolerance_deg, max_edge_length, max_aspect_ratio:\n"
       "        Same meaning and defaults as ``tessellate()``. Keyword-only.\n"
       "    heal: Run the healer while importing. Defaults to ``False``. Keyword-only.\n"
       "    threads: Tessellation threads; ``0`` (default) is automatic. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    list[MeshTessellationResult]: one entry per Brep in prim order, or per\n"
       "    prim that could not be read (``packed_index`` is ``None``).\n"
       "\n"
       "Raises:\n"
       "    RuntimeError: The file cannot be opened, has no BrepArray prims, no mesh\n"
       "        was authored, or the output cannot be written. When no mesh was\n"
       "        authored, the message names each failed Brep and why.\n"
       "\n"
       "Wraps: SmApiUsdTessellateFile");

    m.def("heal_file",
        [](const std::string& input, const std::string& output, int threads) {
            std::vector<SmApiUsdBrepArrayHealResult> results;
            SmPyErrorTrail::clear();
            const SmStatus status = SmApiUsdHealFile(input.c_str(), output.c_str(), threads, results);
            if (status != SM_SUCCESS)
                RaiseWithFailures(status, "SmApiUsdHealFile()", results, [](const SmApiUsdBrepArrayHealResult& r) {
                    return r.m_sPrimPath.empty() ? std::string("file") : r.m_sPrimPath;
                });
            SmPyErrorTrail::clear();
            return results;
        },
        py::arg("input"), py::arg("output"), py::kw_only(), py::arg("threads") = 0,
       "Heal BrepArray definitions in the layers used by the current USD composition.\n"
       "\n"
       "Keeps the scene's layers, references, native instancing, transforms and materials:\n"
       "each BrepArray is healed in the layer that defines it, and the other layers are\n"
       "written to ``<output stem>_layers/`` beside ``output``. Package members are\n"
       "written as standalone layers. All or nothing: if any BrepArray fails,\n"
       "nothing is written. External layers used only by unselected variants are\n"
       "not enumerated or healed; their references keep pointing at the original\n"
       "assets.\n"
       "\n"
       "Args:\n"
       "    input: USD file with BrepArray prims.\n"
       "    output: Standalone .usd/.usda/.usdc file to write; must not exist.\n"
       "    threads: Healing threads; ``0`` (default) is automatic. Keyword-only.\n"
       "\n"
       "Returns:\n"
       "    list[BrepArrayHealResult]: one entry per BrepArray.\n"
       "\n"
       "Raises:\n"
       "    RuntimeError: Any BrepArray fails, the output exists, the scene has no\n"
       "        BrepArray, or it cannot be healed faithfully (BRep geometry in a variant or\n"
       "        composed from several layers, or GeomSubsets other than face or Brep\n"
       "        material bindings). Material subsets with opinions outside the\n"
       "        BRep's defining prim spec are also rejected, including overrides\n"
       "        or additional subsets in consuming layers or internal references.\n"
       "        The message names each failed BrepArray and why.\n"
       "\n"
       "Wraps: SmApiUsdHealFile");
}
