# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Active object model for the SMLib GUI."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ActiveObject:
    """Python GUI registry entry for a model/debug object."""

    object_id: int
    name: str
    kind: str
    handle: object
    color: tuple[float, float, float]
    mesh: dict | None = None
    display_handle: object | None = None
    visible: bool = True
    selected: bool = False
    mesh_error: str = ""

    @property
    def label(self) -> str:
        return f"{self.name} [{self.kind}]"


OBJECT_COLORS = [
    (0.30, 0.60, 0.95),
    (1.00, 0.20, 0.20),
    (0.20, 0.80, 0.30),
    (1.00, 0.85, 0.00),
    (0.70, 0.30, 0.90),
    (0.90, 0.50, 0.10),
    (0.70, 0.70, 0.75),
]

DEBUG_OVERLAY_COLOR = (1.0, 0.0, 0.0)


def smlib_object_kind(val) -> str:
    """Return the supported GUI registry kind for a Python SMLib handle."""
    if all(hasattr(val, attr) for attr in ("faces", "edges", "vertices")):
        return "Brep"
    if all(hasattr(val, attr) for attr in ("to_mesh_arrays", "get_faces", "get_vertices")):
        return "PolyBrep"
    if all(hasattr(val, attr) for attr in ("uv_domain", "evaluate", "normal")):
        return "Surface"
    if all(hasattr(val, attr) for attr in ("parameter_range", "evaluate", "length")):
        return "Curve"
    return ""


def make_active_objects(named_handles: list[tuple[str, object]],
                        starting_id: int = 1) -> list[ActiveObject]:
    """Create registry entries for supported named SMLib handles."""
    objects: list[ActiveObject] = []
    object_id = starting_id
    for name, handle in named_handles:
        kind = smlib_object_kind(handle)
        if not kind:
            continue
        objects.append(ActiveObject(
            object_id=object_id,
            name=name,
            kind=kind,
            handle=handle,
            color=OBJECT_COLORS[(object_id - 1) % len(OBJECT_COLORS)],
        ))
        object_id += 1
    return objects


def legacy_parts_from_objects(objects: list[ActiveObject]) -> list[dict]:
    """Keep the historical _parts shape for scripts and smoke checks."""
    parts = []
    for obj in objects:
        part = {
            "id": obj.object_id,
            "name": obj.name,
            "kind": obj.kind,
            "handle": obj.handle,
            "display_handle": obj.display_handle,
            "color": obj.color,
            "mesh": obj.mesh,
        }
        if obj.kind == "Brep":
            part["brep"] = obj.handle
        elif obj.kind == "PolyBrep":
            part["poly_brep"] = obj.handle
        parts.append(part)
    return parts
