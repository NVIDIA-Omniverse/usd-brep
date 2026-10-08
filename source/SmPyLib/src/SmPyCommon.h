// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Docstring conventions for the SmPy*.cpp bindings included via this header
// live in source/SmPyLib/DOCSTRING_STYLE.md.  Read it before adding or
// editing a binding's docstring; the global conventions (angles, units,
// RuntimeError policy, object lifetime) live in the module-level docstring
// in SmPyMain.cpp.

#ifndef __SmPyCommon_H__
#define __SmPyCommon_H__

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <SmSmlibAll.h>
// SmSmlibAll.h intentionally omits SmPoly.h, but SmPolyBrep / SmPolyVertex /
// SmPolyEdge / SmPolyFace are first-class types in the Python API and are
// required as complete types by pybind11's type_caster machinery in any TU
// that names them in a binding signature.
#include <SmPoly.h>
#include <SmApiImportExport.h>

#include "SmPyError.h"

namespace py = pybind11;

inline py::list tessellation_failures_to_list(const SmTArray<SmTessellationFailure>& failures)
{
    py::list result;
    for (const auto& failure : failures)
    {
        const char* stage = failure.eStage == SM_TS_BOUNDARY_PREPARATION ? "boundary_preparation" :
                            failure.eStage == SM_TS_FACE_TESSELLATION ? "face_tessellation" : "polygon_output";
        py::dict entry;
        entry["face_index"] = failure.lFaceIndex;
        entry["stage"] = stage;
        entry["status"] = failure.eStatus;
        entry["status_name"] = sm_status_name(failure.eStatus);
        result.append(entry);
    }
    return result;
}

py::object smpy_tessellate(SmBrep* brep, double chord_height_tol, double curve_angle_tol_deg,
                          double surface_angle_tol_deg, double max_edge_length, double max_aspect_ratio,
                          bool allow_partial, bool return_failures, bool return_report);


// ---------------------------------------------------------------------------
//  Binding-only enums.
//
//  The underlying SM_API takes raw ``int`` / ``ULONG`` for these selectors
//  and documents the magic values only in ``///<`` header comments.  We
//  define real ``enum class`` types here purely on the binding side so the
//  Python API can take ``CapEnds.BOTH`` and ``OffsetType.LEFT`` rather
//  than bare integers.  The binding lambdas cast these to ``int`` /
//  ``ULONG`` before calling the C entry points; the integer values must
//  therefore match the SM_API contract exactly.
//
//  ``CapEnds``: end-cap selector for sweeps that distinguish a "start" end
//  from an "end" end (``draft_sweep``, ``taper_extrude``,
//  ``sweep_along_planar_path``).  Bool-valued ``cap_ends`` callers
//  (``linear_sweep``, ``pipe_sweep``, ...) keep their plain ``bool``
//  parameter; this enum is only used where the kernel exposes the four-
//  way choice.
//
//  ``OffsetType``: offset side selector for ``create_offset_profile``.
// ---------------------------------------------------------------------------
enum class CapEnds : int
{
    NONE  = 0,  ///< No caps.
    START = 1,  ///< Cap only the "start" end (profile / curves side).
    END   = 2,  ///< Cap only the "end"   end (offset / path side).
    BOTH  = 3,  ///< Cap both ends (default for every consumer).
};

enum class OffsetType : int
{
    LEFT  = 1,  ///< One-sided offset to the left of the input curves.
    RIGHT = 2,  ///< One-sided offset to the right of the input curves.
    BOTH  = 3,  ///< Two-sided band on both sides of the input curves.
};

// Status check used by every binding lambda.  On non-success kernel status,
// raises Python ``RuntimeError`` with a message that includes the symbolic
// SmStatus name, the wrapped C entry point (extracted from #expr), and the
// kernel SER() trail captured by install_sm_error_callback().  See
// SmPyError.h for the contract; .agents/docs/errors.md for code-by-code
// guidance for callers.
#define CHECK_STATUS(expr)                                              \
    do {                                                                \
        SmPyErrorTrail::clear();                                        \
        SmStatus _smPyStat = (expr);                                    \
        if (_smPyStat != SM_SUCCESS)                                    \
            SmPyRaise(_smPyStat, #expr, __FILE__, __LINE__);            \
    } while (0)

// ---------------------------------------------------------------------------
//  Python <-> SMLib value-type conversions
// ---------------------------------------------------------------------------
inline SmPoint3d to_point(const py::tuple& t)
{
    if (t.size() != 3)
        throw std::invalid_argument("Expected a 3-tuple for point");
    return SmPoint3d(t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>());
}

inline SmVector3d to_vec(const py::tuple& t)
{
    if (t.size() != 3)
        throw std::invalid_argument("Expected a 3-tuple for vector");
    return SmVector3d(t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>());
}

inline py::tuple point_to_tuple(const SmPoint3d& p)
{
    return py::make_tuple(p.x, p.y, p.z);
}

inline py::tuple vec_to_tuple(const SmVector3d& v)
{
    return py::make_tuple(v.x, v.y, v.z);
}

inline const SmVector3d* optional_color(py::object color, std::unique_ptr<SmVector3d>& holder)
{
    if (color.is_none())
        return nullptr;
    holder.reset(new SmVector3d(to_vec(color.cast<py::tuple>())));
    return holder.get();
}

inline void fill_points_array(const std::vector<py::tuple>& pts, SmTArray<SmPoint3d>& out)
{
    for (const auto& t : pts)
        out.Add(to_point(t));
}

// Representation-changing Brep operations can delete geometry returned by
// Face.surface() / Edge.curve().  The stable Python layer rejects those
// operations while an affected borrowed wrapper is live, before the kernel
// can mutate the Brep.  Implemented in SmPyBorrowedGeometry.cpp.
bool smpy_scale_requires_brep_nurbs_conversion(const SmVector3d& scale);
void smpy_preflight_brep_nurbs_conversion(SmBrep* brep, const char* api_call);

SmBrep* smpy_read_brep_from_file(const std::string& filename, py::object ascii,
                                 bool rebuild_uv_trim_curves);
py::tuple smpy_read_part_from_file(const std::string& filename, bool ascii);

// ---------------------------------------------------------------------------
//  Submodule overlay
//
//  Each function-only binder registers every binding twice: once on the flat
//  top-level module (back-compat: ``sm.create_box(...)``) and once on a
//  category submodule (discoverability: ``sm.primitives.create_box(...)``).
//  ``BoundModule`` is the small dual-target proxy used by those binders;
//  ``m.def(name, ...)`` forwards to both ``flat`` and ``sub``.
//
//  Class/enum binders (types, enums, polybrep) and the USD binder (which
//  manages its own submodule) take a plain ``py::module_&`` instead, since
//  classes/enums must live in exactly one parent module.
// ---------------------------------------------------------------------------
struct BoundModule
{
    py::module_ flat;
    py::module_ sub;

    template <typename Func, typename... Extra>
    BoundModule& def(const char* name, Func&& f, const Extra&... extra)
    {
        flat.def(name, f, extra...);
        sub.def(name, f, extra...);
        return *this;
    }
};

// ---------------------------------------------------------------------------
//  Per-category binders.  Implemented in SmPy*.cpp; called from SmPyMain.cpp.
//  Order at call-site matters: bind_types and bind_enums first because other
//  binders register functions whose signatures reference those classes/enums.
// ---------------------------------------------------------------------------
void bind_types(py::module_& m);
void bind_file_io(BoundModule& m);
void bind_enums(py::module_& m);
void bind_polybrep(py::module_& m);
void bind_primitives(BoundModule& m);
void bind_curves(BoundModule& m);
void bind_surfaces(BoundModule& m);
void bind_booleans(BoundModule& m);
void bind_sweeps(BoundModule& m);
void bind_fillets(BoundModule& m);
void bind_offset(BoundModule& m);
void bind_brep_ops(BoundModule& m);
void bind_stitching(BoundModule& m);
void bind_intersectors(BoundModule& m);
void bind_transforms(BoundModule& m);
void bind_queries(BoundModule& m);
void bind_cut_project_trim(BoundModule& m);
void bind_heal(BoundModule& m);
void bind_poly_ops(BoundModule& m);
void bind_tessellation(py::module_& m);
void bind_usd(py::module_& m);

#endif // __SmPyCommon_H__
