// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Python-only lifetime preflight for Brep-owned Curve / Surface wrappers.
// The kernel owns and may replace these objects; Python must not retain a raw
// wrapper to an object the kernel is about to delete.

#include "SmPyCommon.h"

#include <SmOffsetSurface.h>

namespace
{
bool has_live_python_wrapper(const SmObject* object, py::handle wrapper_type)
{
    if (object == nullptr)
        return false;

    // pybind11 2.11.1 (pinned in deps/target-deps-python*.packman.xml)
    // unregisters an instance as soon as its last Python reference dies. A
    // direct lookup is allocation-free and avoids maintaining a second
    // weak-reference registry whose entries could outlive the wrapper.
    // The registry is keyed by address only, so a stale wrapper of another
    // type (e.g. a Face of a deleted Brep) can sit at a reused address; only
    // count wrappers of the borrowed type itself.
    const auto& instances = py::detail::get_internals().registered_instances;
    const auto range = instances.equal_range(object);
    for (auto it = range.first; it != range.second; ++it)
    {
        PyObject* instance = reinterpret_cast<PyObject*>(it->second);
        const int is_instance = PyObject_IsInstance(instance, wrapper_type.ptr());
        if (is_instance == 1)
            return true;
        if (is_instance < 0)
        {
            // Not expected for a pybind11 type. Clear the error and count the wrapper as live:
            // refusing the operation is safe, replacing geometry a live handle points to is not.
            PyErr_Clear();
            return true;
        }
    }
    return false;
}

bool turn_to_nurbs_may_replace(const SmSurface* surface)
{
    if (surface == nullptr)
        return false;

    // Mirror SmBrep::TurnToNURBS's pointer-preservation gate. Analytic
    // subclasses of SmBSplineSurface are replaced by an exact base-class
    // object. Offset surfaces with a B-spline base may also be replaced after
    // approximation.
    if (surface->IsKindOf(SmBSplineSurface_TYPE))
        return surface->GetType() != SmBSplineSurface_TYPE;

    if (surface->GetType() == SmOffsetSurface_TYPE)
    {
        const SmSurface* base = static_cast<const SmOffsetSurface*>(surface)->GetBaseSurface();
        return base != nullptr && base->IsKindOf(SmBSplineSurface_TYPE);
    }

    return false;
}

bool turn_to_nurbs_may_replace(const SmCurve* curve)
{
    // SmBrep::TurnToNURBS replaces every edge curve whose exact runtime type
    // is not the base SmBSplineCurve type. In particular, SmLine and
    // SmCircle derive from SmBSplineCurve but are still replaced.
    return curve != nullptr && curve->GetType() != SmBSplineCurve_TYPE;
}

[[noreturn]] void raise_live_borrowed_geometry_error(const char* api_call)
{
    std::string message = api_call ? api_call : "Brep geometry conversion";
    message += " failed: ";
    message += sm_status_name(SM_ERR_INVALID_INPUT);
    message += " (";
    message += std::to_string(static_cast<long>(SM_ERR_INVALID_INPUT));
    message += "). The operation would replace Brep-owned Curve or Surface "
               "geometry while a borrowed Python handle is still live. Release "
               "all handles returned by Edge.curve() and Face.surface() "
               "(including aliases stored in containers), retry the operation, "
               "then reacquire them from the mutated Brep. To preserve the "
               "existing handles, copy the Brep and mutate the copy instead.";
    throw std::runtime_error(message);
}
}

bool smpy_scale_requires_brep_nurbs_conversion(const SmVector3d& scale)
{
    // Let SM_API report invalid scale values itself. For accepted Brep
    // scales, this is the exact predicate used by SmBrep::Transform before it
    // calls TurnToNURBS (positive non-uniform, negative uniform, or mixed-sign).
    if (scale.IsUndef() || scale.GetMinDimension() <= SM_EFF_ZERO)
        return false;

    return (   !SM_ARE_SAME(scale.x, scale.y)
            || !SM_ARE_SAME(scale.x, scale.z)
            || scale.x < SM_EFF_ZERO );
}

void smpy_preflight_brep_nurbs_conversion(SmBrep* brep, const char* api_call)
{
    if (brep == nullptr)
        return;

    const py::handle surface_type = py::type::of<SmSurface>();
    const py::handle curve_type = py::type::of<SmCurve>();

    SmTArray<SmSurface*> surfaces;
    brep->GetSurfaces(surfaces);
    for (ULONG i = 0; i < surfaces.GetSize(); ++i)
    {
        if (turn_to_nurbs_may_replace(surfaces[i]) && has_live_python_wrapper(surfaces[i], surface_type))
            raise_live_borrowed_geometry_error(api_call);
    }

    SmTArray<SmCurve*> curves;
    brep->GetCurves(curves);
    for (ULONG i = 0; i < curves.GetSize(); ++i)
    {
        if (turn_to_nurbs_may_replace(curves[i]) && has_live_python_wrapper(curves[i], curve_type))
            raise_live_borrowed_geometry_error(api_call);
    }
}
