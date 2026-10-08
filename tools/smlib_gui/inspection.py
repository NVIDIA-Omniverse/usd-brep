# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Inspector text for active SMLib GUI objects."""

from __future__ import annotations

import logging

from .model import ActiveObject

logger = logging.getLogger(__name__)


def face_stream_cell_count(faces) -> int:
    count = 0
    i = 0
    face_count = len(faces)
    while i < face_count:
        try:
            n = int(faces[i])
        except (TypeError, ValueError):
            break
        if n < 0:
            logger.warning("Negative face stream count n=%s at i=%s with count=%s and faces length=%s",
                           n, i, count, face_count)
            break
        if i + n + 1 > face_count:
            logger.warning("Truncated face stream count n=%s at i=%s with count=%s and faces length=%s",
                           n, i, count, face_count)
            break
        i += n + 1
        count += 1
    return count


def format_value(value) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, dict):
        return ", ".join(f"{key}={format_value(val)}" for key, val in value.items())
    if isinstance(value, (list, tuple)):
        return "(" + ", ".join(format_value(val) for val in value) + ")"
    return str(value)


def append_query(lines: list[str], label: str, getter) -> None:
    try:
        value = getter()
    except Exception as exc:
        lines.append(f"{label}: <error: {exc}>")
    else:
        lines.append(f"{label}: {format_value(value)}")


def append_mesh_summary(lines: list[str], obj: ActiveObject) -> None:
    if not obj.mesh:
        # Report why, not just that. The status-bar summary built in
        # _retessellate_and_display is overwritten by callers that post their own
        # success text, so the inspector is the only durable place a user can find
        # out why the viewport is empty for this object.
        reason = getattr(obj, "mesh_error", "")
        lines.append(f"Mesh: not displayed ({reason})" if reason else "Mesh: not displayed")
        return
    points = obj.mesh.get("points", ())
    faces = obj.mesh.get("faces", ())
    lines.append(f"Mesh: {len(points):,} points, {face_stream_cell_count(faces):,} faces")


def object_summary_text(obj: ActiveObject) -> str:
    """Build concise inspector text for one active object."""
    lines = [
        f"Name: {obj.name}",
        f"Type: {obj.kind}",
        f"Object id: {obj.object_id}",
        f"Handle: {type(obj.handle).__name__}",
    ]
    if obj.kind == "Brep":
        if hasattr(obj.handle, "info"):
            append_query(lines, "Info", obj.handle.info)
        else:
            append_query(lines, "Faces", obj.handle.face_count)
            append_query(lines, "Edges", obj.handle.edge_count)
            append_query(lines, "Vertices", obj.handle.vertex_count)
        append_query(lines, "Bounds", obj.handle.bounding_box)
        append_query(lines, "Center", obj.handle.center)
    elif obj.kind == "PolyBrep":
        append_query(lines, "Bounds", obj.handle.calculate_bounding_box)
        append_query(lines, "Manifold", obj.handle.is_manifold_solid)
        append_query(lines, "Tolerance", obj.handle.get_tolerance)
    elif obj.kind == "Surface":
        append_query(lines, "UV domain", obj.handle.uv_domain)
        append_query(lines, "Periodic U", obj.handle.is_periodic_u)
        append_query(lines, "Periodic V", obj.handle.is_periodic_v)
    elif obj.kind == "Curve":
        append_query(lines, "Parameter range", obj.handle.parameter_range)
        append_query(lines, "Length", obj.handle.length)
        append_query(lines, "Closed", obj.handle.is_closed)
    elif obj.kind == "Point":
        append_query(lines, "Point", lambda: obj.handle.point)
    elif obj.kind == "Line":
        append_query(lines, "Start", lambda: obj.handle.start)
        append_query(lines, "End", lambda: obj.handle.end)
    elif obj.kind == "Polyline":
        append_query(lines, "Points", lambda: len(obj.handle.points))
    elif obj.kind == "Box":
        append_query(lines, "Min", lambda: obj.handle.minimum)
        append_query(lines, "Max", lambda: obj.handle.maximum)
    elif obj.kind == "Normal":
        append_query(lines, "Origin", lambda: obj.handle.origin)
        append_query(lines, "Vector", lambda: obj.handle.vector)
        append_query(lines, "Scale", lambda: obj.handle.scale)
    elif obj.kind == "Pick":
        append_query(lines, "Point", lambda: obj.handle.point)
        append_query(lines, "Normal", lambda: obj.handle.normal)
        for key, value in obj.handle.metadata.items():
            lines.append(f"{key}: {format_value(value)}")
    elif obj.kind == "CurveSample":
        append_query(lines, "Points", lambda: len(obj.handle.points))
        for key, value in obj.handle.metadata.items():
            lines.append(f"{key}: {format_value(value)}")
    elif obj.kind == "SurfaceSample":
        append_query(lines, "Polylines", lambda: len(obj.handle.polylines))
        for key, value in obj.handle.metadata.items():
            lines.append(f"{key}: {format_value(value)}")
    elif obj.kind == "Topology":
        append_query(lines, "Topology", lambda: obj.handle.topology_kind)
        append_query(lines, "Polylines", lambda: len(obj.handle.polylines))
        append_query(lines, "Points", lambda: len(obj.handle.points))
        for key, value in obj.handle.metadata.items():
            lines.append(f"{key}: {format_value(value)}")
    elif obj.kind == "DisplayList":
        append_query(lines, "Batches", lambda: len(obj.handle.batches))
        for key, value in obj.handle.metadata.items():
            lines.append(f"{key}: {format_value(value)}")
    elif obj.kind == "Section":
        append_query(lines, "Segments", lambda: len(obj.handle.segments))
        for key, value in obj.handle.metadata.items():
            lines.append(f"{key}: {format_value(value)}")
    if obj.kind in {
        "Point",
        "Line",
        "Polyline",
        "Box",
        "Normal",
        "Pick",
        "CurveSample",
        "SurfaceSample",
        "Topology",
        "DisplayList",
        "Section",
    }:
        lines.append("Overlay: displayed")
    else:
        append_mesh_summary(lines, obj)
    return "\n".join(lines)
