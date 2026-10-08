// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiPolygons.h (SmApiPolyBoolean{,Union,Difference,Intersection,
// Merge}, SmApiPolyBrepIsManifoldSolid, SmApiPolyBrepComputeVolume).

#include "SmPyCommon.h"

#include <SmApiPolygons.h>
#include <SmPolyMerge.h>

void bind_poly_ops(BoundModule& m)
{
    m.def("poly_boolean", [](SmPolyBrep* a, SmPolyBrep* b, SmPolyBooleanOperationType op) {
        SmPolyBrep* r = nullptr;
        CHECK_STATUS(SmApiPolyBoolean(a, b, op, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("poly_brep1"), py::arg("poly_brep2"), py::arg("operation"),
       "Combine two polygon meshes with a boolean operation.\n"
       "\n"
       "Args:\n"
       "    poly_brep1: Primary mesh. For ``DIFFERENCE`` this is the\n"
       "        workpiece. Consumed by the operation.\n"
       "    poly_brep2: Tool mesh. For ``DIFFERENCE`` this is the cutting\n"
       "        shape. Consumed by the operation.\n"
       "    operation: ``PolyBooleanOp.{UNION, INTERSECTION, DIFFERENCE,\n"
       "        MERGE, PARTIAL_MERGE}``.\n"
       "\n"
       "Returns:\n"
       "    PolyBrep: combined mesh. The two input meshes are consumed and\n"
       "    must not be reused.\n"
       "\n"
       "Notes:\n"
       "    Operates on triangulated geometry, so the result is approximate\n"
       "    and quality scales with input mesh density - intersection\n"
       "    curves are followed along triangle edges. For exact NURBS\n"
       "    booleans, work in the Brep domain with ``boolean`` and\n"
       "    tessellate at the end. Both input meshes should be manifold\n"
       "    (closed) for best results; check with\n"
       "    ``poly_brep_is_manifold_via_api`` if unsure.\n"
       "\n"
       "    ``DIFFERENCE`` is order-sensitive: ``poly_boolean(A, B,\n"
       "    DIFFERENCE)`` removes ``B`` from ``A``.\n"
       "\n"
       "See Also:\n"
       "    poly_boolean_union: Convenience wrapper for ``UNION``.\n"
       "    poly_boolean_difference: Convenience wrapper for ``DIFFERENCE``.\n"
       "    poly_boolean_intersection: Convenience wrapper for\n"
       "        ``INTERSECTION``.\n"
       "    poly_boolean_merge: Convenience wrapper for ``MERGE``.\n"
       "    boolean: Exact-NURBS Brep equivalent.\n"
       "    tessellate: Convert a Brep to a ``PolyBrep`` first.\n"
       "\n"
       "Wraps: SmApiPolyBoolean");

    m.def("poly_boolean_union", [](SmPolyBrep* a, SmPolyBrep* b) {
        SmPolyBrep* r = nullptr;
        CHECK_STATUS(SmApiPolyBooleanUnion(a, b, r));
        return r;
    }, py::return_value_policy::reference, py::arg("poly_brep1"), py::arg("poly_brep2"),
       "Compute the boolean union (additive) of two polygon meshes.\n"
       "\n"
       "Args:\n"
       "    poly_brep1: First mesh. Consumed by the operation.\n"
       "    poly_brep2: Second mesh. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    PolyBrep: union mesh (material from either body).\n"
       "\n"
       "See Also:\n"
       "    poly_boolean: Generic operation taking a ``PolyBooleanOp`` value.\n"
       "    poly_boolean_difference: Subtractive variant.\n"
       "    poly_boolean_intersection: Common-overlap variant.\n"
       "    boolean_union: Exact-NURBS Brep equivalent.\n"
       "\n"
       "Wraps: SmApiPolyBooleanUnion");

    m.def("poly_boolean_difference", [](SmPolyBrep* a, SmPolyBrep* b) {
        SmPolyBrep* r = nullptr;
        CHECK_STATUS(SmApiPolyBooleanDifference(a, b, r));
        return r;
    }, py::return_value_policy::reference, py::arg("poly_brep1"), py::arg("poly_brep2"),
       "Compute the boolean difference (subtractive) of two polygon meshes.\n"
       "\n"
       "Args:\n"
       "    poly_brep1: Workpiece mesh. Consumed by the operation.\n"
       "    poly_brep2: Tool mesh to subtract. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    PolyBrep: ``poly_brep1`` with ``poly_brep2`` carved out.\n"
       "\n"
       "Notes:\n"
       "    Order-sensitive: swap the arguments to subtract the other way.\n"
       "\n"
       "See Also:\n"
       "    poly_boolean: Generic operation taking a ``PolyBooleanOp`` value.\n"
       "    poly_boolean_union: Additive variant.\n"
       "    poly_boolean_intersection: Common-overlap variant.\n"
       "    boolean_difference: Exact-NURBS Brep equivalent.\n"
       "\n"
       "Wraps: SmApiPolyBooleanDifference");

    m.def("poly_boolean_intersection", [](SmPolyBrep* a, SmPolyBrep* b) {
        SmPolyBrep* r = nullptr;
        CHECK_STATUS(SmApiPolyBooleanIntersection(a, b, r));
        return r;
    }, py::return_value_policy::reference, py::arg("poly_brep1"), py::arg("poly_brep2"),
       "Compute the boolean intersection (common overlap) of two polygon meshes.\n"
       "\n"
       "Args:\n"
       "    poly_brep1: First mesh. Consumed by the operation.\n"
       "    poly_brep2: Second mesh. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    PolyBrep: region inside both meshes. If the inputs are\n"
       "    disjoint the result has no volume.\n"
       "\n"
       "See Also:\n"
       "    poly_boolean: Generic operation taking a ``PolyBooleanOp`` value.\n"
       "    poly_boolean_union: Additive variant.\n"
       "    poly_boolean_difference: Subtractive variant.\n"
       "    boolean_intersection: Exact-NURBS Brep equivalent.\n"
       "\n"
       "Wraps: SmApiPolyBooleanIntersection");

    m.def("poly_boolean_merge", [](SmPolyBrep* a, SmPolyBrep* b) {
        SmPolyBrep* r = nullptr;
        CHECK_STATUS(SmApiPolyBooleanMerge(a, b, r));
        return r;
    }, py::return_value_policy::reference, py::arg("poly_brep1"), py::arg("poly_brep2"),
       "Combine two polygon meshes without removing any faces.\n"
       "\n"
       "Args:\n"
       "    poly_brep1: First mesh. Consumed by the operation.\n"
       "    poly_brep2: Second mesh. Consumed by the operation.\n"
       "\n"
       "Returns:\n"
       "    PolyBrep: merged mesh containing all faces of both inputs.\n"
       "    Where the meshes touch, the result is non-manifold (more than\n"
       "    two faces meet at an edge).\n"
       "\n"
       "Notes:\n"
       "    Equivalent to ``poly_boolean(a, b, PolyBooleanOp.MERGE)``.\n"
       "    Useful for assemblies of touching but non-penetrating meshes.\n"
       "    For finer control over how the seam is treated, use the\n"
       "    generic ``poly_boolean`` with ``PARTIAL_MERGE``.\n"
       "\n"
       "See Also:\n"
       "    poly_boolean_union: Material from either, shared region merged.\n"
       "    poly_boolean: Generic form (offers ``PARTIAL_MERGE``).\n"
       "    boolean_merge: Exact-NURBS Brep equivalent.\n"
       "\n"
       "Wraps: SmApiPolyBooleanMerge");

    m.def("poly_brep_is_manifold_via_api", [](SmPolyBrep* mesh) {
        SmBoolean b = FALSE;
        CHECK_STATUS(SmApiPolyBrepIsManifoldSolid(mesh, b));
        return (bool)b;
    }, py::arg("poly_brep"),
       "Check whether a polygon mesh is a closed manifold solid (SmAPI helper).\n"
       "\n"
       "Args:\n"
       "    poly_brep: Mesh to validate. Not modified.\n"
       "\n"
       "Returns:\n"
       "    bool: ``True`` if every edge of the mesh is shared by exactly\n"
       "    two faces; ``False`` otherwise.\n"
       "\n"
       "Notes:\n"
       "    Functionally equivalent to the class method\n"
       "    ``poly_brep.is_manifold_solid()``. The ``_via_api`` suffix\n"
       "    disambiguates the SmAPI entry point\n"
       "    (``SmApiPolyBrepIsManifoldSolid``) from the class accessor\n"
       "    and from the Brep-level ``is_manifold_solid`` query.\n"
       "\n"
       "See Also:\n"
       "    poly_brep_volume_via_api: Volume counterpart.\n"
       "    is_manifold_solid: Exact-NURBS Brep equivalent.\n"
       "\n"
       "Wraps: SmApiPolyBrepIsManifoldSolid");

    m.def("poly_brep_volume_via_api", [](SmPolyBrep* mesh) {
        double vol = 0;
        CHECK_STATUS(SmApiPolyBrepComputeVolume(mesh, vol));
        return vol;
    }, py::arg("poly_brep"),
       "Compute the enclosed volume of a polygon mesh (SmAPI helper).\n"
       "\n"
       "Args:\n"
       "    poly_brep: Manifold-solid mesh. Behaviour is undefined for\n"
       "        open shells; validate with\n"
       "        ``poly_brep_is_manifold_via_api`` first if uncertain.\n"
       "\n"
       "Returns:\n"
       "    float: enclosed volume in cubic modeling units. The result\n"
       "    approximates the source NURBS volume to within tessellation\n"
       "    error.\n"
       "\n"
       "Notes:\n"
       "    Functionally similar to ``poly_brep.compute_properties\n"
       "    (origin)['volume']``. The ``_via_api`` suffix disambiguates\n"
       "    the SmAPI entry point (``SmApiPolyBrepComputeVolume``) from\n"
       "    the class accessor.\n"
       "\n"
       "See Also:\n"
       "    poly_brep_is_manifold_via_api: Manifold-solid pre-check.\n"
       "    compute_volume: Exact-NURBS Brep equivalent.\n"
       "    tessellate: Source step that produces the mesh.\n"
       "\n"
       "Wraps: SmApiPolyBrepComputeVolume");
}
