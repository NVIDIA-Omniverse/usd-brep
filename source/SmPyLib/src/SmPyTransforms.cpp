// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiGeneral.h (SmApiTranslate, SmApiRotate, SmApiScale,
// SmApiScaleByPt, SmApiTransform).

#include "SmPyCommon.h"

#include <SmApiGeneral.h>
#include <SmAxis2Placement.h>

void bind_transforms(BoundModule& m)
{
    // Every transform entry point takes an independent modeling ``Object``
    // and modifies it in place. pybind11 routes the four public root object
    // families through the hierarchy registered in ``SmPyTypes.cpp``.
    m.def("translate", [](py::object target, py::tuple vec) {
        SmObject* pTarget = target.cast<SmObject*>();
        SmVector3d v = to_vec(vec);
        CHECK_STATUS(SmApiTranslate(pTarget, v));
        return target;
    },
       py::arg("target"), py::arg("translation"),
       "Mutates: target; returns the same handle for chaining.\n"
       "\n"
       "Translate an independent modeling Object by a displacement vector.\n"
       "\n"
       "Args:\n"
       "    target: Brep, PolyBrep, standalone Curve, or standalone Surface.\n"
       "    translation: Displacement vector ``(dx, dy, dz)`` in modeling units.\n"
       "\n"
       "Notes:\n"
       "    The underlying geometry is transformed exactly - no\n"
       "    approximation is introduced. Curves and Surfaces owned by a Brep\n"
       "    must be transformed through that Brep.\n"
       "\n"
       "See Also:\n"
       "    rotate: Rotate around an axis.\n"
       "    scale: Scale by per-axis factors.\n"
       "\n"
       "Wraps: SmApiTranslate");

    m.def("rotate", [](py::object target, py::tuple refPt, py::tuple axis, double angleDeg) {
        SmObject* pTarget = target.cast<SmObject*>();
        SmVector3d rp = to_vec(refPt);
        SmVector3d ax = to_vec(axis);
        CHECK_STATUS(SmApiRotate(pTarget, rp, ax, angleDeg));
        return target;
    },
       py::arg("target"), py::arg("ref_point"), py::arg("axis"), py::arg("angle_deg"),
       "Mutates: target; returns the same handle for chaining.\n"
       "\n"
       "Rotate an independent modeling Object around an axis.\n"
       "\n"
       "Args:\n"
       "    target: Brep, PolyBrep, standalone Curve, or standalone Surface.\n"
       "    ref_point: A point on the rotation axis as ``(x, y, z)``.\n"
       "    axis: Direction of the rotation axis as ``(dx, dy, dz)``.\n"
       "    angle_deg: Rotation angle, in degrees, following the right-hand rule.\n"
       "\n"
       "Notes:\n"
       "    The rotation axis is the line ``ref_point + t * axis``. ``axis``\n"
       "    does not need to be unit-length. Curves and Surfaces owned by a\n"
       "    Brep must be transformed through that Brep.\n"
       "\n"
       "See Also:\n"
       "    translate: Translate by a vector.\n"
       "    scale: Scale by per-axis factors.\n"
       "    transform: Combined rotation/translation via Axis2Placement.\n"
       "\n"
       "Wraps: SmApiRotate");

    m.def("scale", [](py::object target, py::tuple scaleVec) {
        SmObject* pTarget = target.cast<SmObject*>();
        SmVector3d s = to_vec(scaleVec);
        // Test the Python wrapper type without virtual dispatch through the
        // context-owned pointer. Invalid scales bypass this preflight so
        // SM_API can report them before accessing the target object.
        if (   smpy_scale_requires_brep_nurbs_conversion(s)
            && py::isinstance<SmBrep>(target) )
        {
            smpy_preflight_brep_nurbs_conversion(target.cast<SmBrep*>(), "SmApiScale");
        }
        CHECK_STATUS(SmApiScale(pTarget, s));
        return target;
    },
       py::arg("target"), py::arg("scale"),
       "Mutates: target; returns the same handle for chaining.\n"
       "\n"
       "Scale an independent modeling Object about the world origin.\n"
       "\n"
       "Args:\n"
       "    target: Brep, PolyBrep, standalone Curve, or standalone Surface.\n"
       "    scale: Per-axis scale factors ``(sx, sy, sz)``. Non-uniform values\n"
       "        stretch supported geometry independently along each world axis.\n"
       "        Factors must be finite and non-degenerate; PolyBrep factors\n"
       "        must also be positive.\n"
       "\n"
       "Notes:\n"
       "    Scaling is centered at the world origin. To scale about a different\n"
       "    point use ``scale_about_point`` (or translate, scale, translate\n"
       "    back). Volume scales by ``abs(sx * sy * sz)``. Generic B-spline geometry\n"
       "    and lines support non-uniform scaling. This API permits only\n"
       "    positive, uniform factors for other specialized standalone Curve and\n"
       "    Surface representations; unsupported scaling raises ``RuntimeError``\n"
       "    without modifying the target. Owned geometry must be transformed\n"
       "    through its Brep. A Brep scale that requires analytic-to-NURBS\n"
       "    conversion is rejected before mutation while an affected borrowed\n"
       "    ``Face.surface()`` or ``Edge.curve()`` handle remains live. Release\n"
       "    and reacquire those handles, or scale a ``brep.copy()`` instead.\n"
       "\n"
       "See Also:\n"
       "    translate: Translate by a vector.\n"
       "    rotate: Rotate around an axis.\n"
       "    scale_about_point: Scale about an arbitrary reference point.\n"
       "\n"
       "Wraps: SmApiScale");

    m.def("scale_about_point", [](py::object target, py::tuple ref_pt, py::tuple scale_vec) {
        SmObject* pTarget = target.cast<SmObject*>();
        SmVector3d rp = to_vec(ref_pt);
        SmVector3d s = to_vec(scale_vec);
        // Match scale(): preserve SM_API's invalid-scale-before-object-access
        // ordering and select Brep wrappers without virtual dispatch.
        if (   smpy_scale_requires_brep_nurbs_conversion(s)
            && py::isinstance<SmBrep>(target) )
        {
            smpy_preflight_brep_nurbs_conversion(target.cast<SmBrep*>(), "SmApiScaleByPt");
        }
        CHECK_STATUS(SmApiScaleByPt(pTarget, rp, s));
        return target;
    },
       py::arg("target"), py::arg("ref_point"), py::arg("scale"),
       "Mutates: target; returns the same handle for chaining.\n"
       "\n"
       "Scale an independent modeling Object about an arbitrary reference point.\n"
       "\n"
       "Args:\n"
       "    target: Brep, PolyBrep, standalone Curve, or standalone Surface.\n"
       "    ref_point: Center of scaling as ``(x, y, z)``. The kernel\n"
       "        translates this point to the origin, applies ``scale``, and\n"
       "        translates back.\n"
       "    scale: Finite, non-degenerate per-axis scale factors\n"
       "        ``(sx, sy, sz)``. PolyBrep factors must also be positive.\n"
       "\n"
       "Notes:\n"
       "    Generic B-spline geometry and lines support non-uniform scaling.\n"
       "    This API permits only positive, uniform factors for other specialized\n"
       "    standalone Curve and Surface representations. Unsupported scaling\n"
       "    raises ``RuntimeError`` without first translating the target. Owned\n"
       "    geometry must be transformed through its Brep. A Brep scale that\n"
       "    requires analytic-to-NURBS conversion is rejected before mutation\n"
       "    while an affected borrowed ``Face.surface()`` or ``Edge.curve()``\n"
       "    handle remains live. Release and reacquire those handles, or scale a\n"
       "    ``brep.copy()`` instead.\n"
       "\n"
       "See Also:\n"
       "    scale: Scale about the world origin.\n"
       "\n"
       "Wraps: SmApiScaleByPt");

    m.def("transform", [](py::object target, SmAxis2Placement& placement) {
        SmObject* pTarget = target.cast<SmObject*>();
        CHECK_STATUS(SmApiTransform(pTarget, placement));
        return target;
    },
       py::arg("target"), py::arg("placement"),
       "Mutates: target; returns the same handle for chaining.\n"
       "\n"
       "Apply a combined rotation/translation to an independent modeling Object.\n"
       "\n"
       "Args:\n"
       "    target: Brep, PolyBrep, standalone Curve, or standalone Surface.\n"
       "    placement: ``Axis2Placement`` encoding the combined rigid\n"
       "        transform (rotation + translation) to apply.\n"
       "\n"
       "Notes:\n"
       "    Use this when you need a single rigid transform expressed as a\n"
       "    placement. For separate moves, prefer ``translate`` / ``rotate``.\n"
       "    Curves and Surfaces owned by a Brep must be transformed through\n"
       "    that Brep.\n"
       "\n"
       "See Also:\n"
       "    translate: Translation-only convenience.\n"
       "    rotate: Rotation-only convenience.\n"
       "\n"
       "Wraps: SmApiTransform");
}
