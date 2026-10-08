# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Selected-object modify and diagnostic actions for the SMLib GUI."""

from __future__ import annotations

from dataclasses import dataclass

from ..inspection import format_value, object_summary_text
from ..model import ActiveObject
from ..runtime import sm


@dataclass
class ModifyResult:
    object: ActiveObject
    source_line: str
    status: str
    details: str
    retessellate: bool = True


def apply_modify_operation(obj: ActiveObject, operation: str) -> ModifyResult:
    """Apply a supported modify-data operation to a selected active object."""
    if operation == "heal":
        return _heal(obj)
    if operation == "stitch_solid":
        return _stitch_solid(obj)
    if operation == "stitch_shell":
        return _stitch_shell(obj)
    if operation == "unify_normals":
        return _unify_normals(obj)
    if operation == "mass_properties":
        return _mass_properties(obj)
    raise ValueError(f"Unknown modify operation: {operation}")


def _heal(obj: ActiveObject) -> ModifyResult:
    _require_brep(obj, "heal")
    obj.handle = sm.heal_brep(obj.handle)
    return ModifyResult(
        object=obj,
        source_line=f"sm.heal_brep({obj.name})",
        status=f"Healed BRep {obj.name}.",
        details="",
    )


def _stitch_solid(obj: ActiveObject) -> ModifyResult:
    _require_brep(obj, "stitch into solid")
    brep, is_solid, stitched, vertex_gap, edge_gap = sm.stitch_into_solid(obj.handle)
    obj.handle = brep
    details = _modify_details(
        obj,
        "stitch_into_solid",
        {
            "is_solid": is_solid,
            "stitched_edges": stitched,
            "max_vertex_gap": vertex_gap,
            "max_edge_gap": edge_gap,
        },
    )
    return ModifyResult(
        object=obj,
        source_line=(
            f"{obj.name}, is_solid, stitched_edges, max_vertex_gap, "
            f"max_edge_gap = sm.stitch_into_solid({obj.name})"
        ),
        status=f"Stitched {obj.name}: solid={is_solid}, edges={stitched}.",
        details=details,
    )


def _stitch_shell(obj: ActiveObject) -> ModifyResult:
    _require_brep(obj, "stitch into shell")
    # The second element echoes the shell_is_well_formed input mode; it is not a result.
    brep, _mode, stitched, vertex_gap, edge_gap = sm.stitch_into_shell(obj.handle)
    obj.handle = brep
    details = _modify_details(
        obj,
        "stitch_into_shell",
        {
            "stitched_edges": stitched,
            "max_vertex_gap": vertex_gap,
            "max_edge_gap": edge_gap,
        },
    )
    return ModifyResult(
        object=obj,
        source_line=(
            f"{obj.name}, _mode, stitched_edges, max_vertex_gap, "
            f"max_edge_gap = sm.stitch_into_shell({obj.name})"
        ),
        status=f"Stitched {obj.name} into a shell: edges={stitched}.",
        details=details,
    )


def _unify_normals(obj: ActiveObject) -> ModifyResult:
    _require_brep(obj, "unify normals")
    faces = obj.handle.faces()
    if not faces:
        raise RuntimeError("Selected BRep has no faces")
    brep, flipped_faces = sm.unify_normals(obj.handle, faces[0], num_samples=100)
    obj.handle = brep
    details = _modify_details(
        obj,
        "unify_normals",
        {"flipped_faces": len(flipped_faces)},
    )
    return ModifyResult(
        object=obj,
        source_line=(
            f"{obj.name}, flipped_faces = "
            f"sm.unify_normals({obj.name}, {obj.name}.faces()[0], num_samples=100)"
        ),
        status=f"Unified normals for {obj.name}: flipped={len(flipped_faces)}.",
        details=details,
    )


def _mass_properties(obj: ActiveObject) -> ModifyResult:
    if obj.kind not in {"Brep", "PolyBrep"}:
        raise RuntimeError("Mass properties support selected BRep or PolyBrep objects")
    rows = []
    if obj.kind == "Brep":
        rows.append(("manifold", obj.handle.is_manifold_solid()))
        rows.append(("volume", obj.handle.volume()))
        rows.append(("bounds", obj.handle.bounding_box()))
        rows.append(("center", obj.handle.center()))
    poly = obj.display_handle if obj.kind == "Brep" else obj.handle
    if poly is not None and hasattr(poly, "compute_properties"):
        try:
            origin = obj.handle.center() if obj.kind == "Brep" else _bbox_center(poly.calculate_bounding_box())
            props = poly.compute_properties(origin)
        except (RuntimeError, ValueError, OSError) as exc:
            rows.append(("display_mesh_properties", f"<error: {exc}>"))
        else:
            rows.append(("display_mesh_properties", props))
    details = _modify_details(obj, "mass_properties", dict(rows))
    return ModifyResult(
        object=obj,
        source_line="",
        status=f"Computed mass properties for {obj.name}.",
        details=details,
        retessellate=False,
    )


def _modify_details(obj: ActiveObject, operation: str, values: dict[str, object]) -> str:
    lines = [object_summary_text(obj), "", f"Modify: {operation}"]
    for key, value in values.items():
        lines.append(f"{key}: {format_value(value)}")
    return "\n".join(lines)


def _bbox_center(bounds) -> tuple[float, float, float]:
    mn, mx = bounds
    return (
        0.5 * (mn[0] + mx[0]),
        0.5 * (mn[1] + mx[1]),
        0.5 * (mn[2] + mx[2]),
    )


def _require_brep(obj: ActiveObject, label: str) -> None:
    if obj.kind != "Brep":
        raise RuntimeError(f"Select a BRep before running {label}")
