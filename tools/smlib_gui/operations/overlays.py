# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Debug overlay registry entries for the SMLib GUI."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from ..model import ActiveObject, DEBUG_OVERLAY_COLOR

logger = logging.getLogger(__name__)


OVERLAY_KINDS = {
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
}


@dataclass
class PointOverlay:
    point: tuple[float, float, float]


@dataclass
class LineOverlay:
    start: tuple[float, float, float]
    end: tuple[float, float, float]


@dataclass
class PolylineOverlay:
    points: list[tuple[float, float, float]]


@dataclass
class BoxOverlay:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]


@dataclass
class NormalOverlay:
    origin: tuple[float, float, float]
    vector: tuple[float, float, float]
    scale: float = 1.0


@dataclass
class PickOverlay:
    point: tuple[float, float, float]
    normal: tuple[float, float, float] | None
    metadata: dict[str, object]


@dataclass
class CurveSampleOverlay:
    points: list[tuple[float, float, float]]
    metadata: dict[str, object]


@dataclass
class SurfaceSampleOverlay:
    polylines: list[list[tuple[float, float, float]]]
    metadata: dict[str, object]


@dataclass
class TopologyOverlay:
    topology_kind: str
    polylines: list[list[tuple[float, float, float]]]
    points: list[tuple[float, float, float]]
    metadata: dict[str, object]


@dataclass
class DrawBatchOverlay:
    batches: list[dict[str, object]]
    metadata: dict[str, object]


@dataclass
class SectionOverlay:
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]]
    metadata: dict[str, object]


def _point3(value) -> tuple[float, float, float]:
    return (float(value[0]), float(value[1]), float(value[2]))


def _safe_range(lo: float, hi: float) -> tuple[float, float]:
    lo = float(lo)
    hi = float(hi)
    if lo == hi:
        hi = lo + 1.0
    if hi < lo:
        lo, hi = hi, lo
    return lo, hi


def _linspace(lo: float, hi: float, count: int) -> list[float]:
    count = max(2, int(count))
    if count == 2:
        return [lo, hi]
    step = (hi - lo) / float(count - 1)
    return [lo + step * i for i in range(count)]


def _bbox_diagonal(bounds) -> float:
    mn, mx = bounds
    return (
        (float(mx[0]) - float(mn[0])) ** 2
        + (float(mx[1]) - float(mn[1])) ** 2
        + (float(mx[2]) - float(mn[2])) ** 2
    ) ** 0.5


def _points_inside_bounds(points, bounds, padding: float) -> bool:
    mn, mx = bounds
    for point in points:
        if not all(math.isfinite(float(component)) for component in point):
            return False
        for axis in range(3):
            value = float(point[axis])
            if value < float(mn[axis]) - padding or value > float(mx[axis]) + padding:
                return False
    return True


def _edge_endpoint_polyline(edge) -> list[tuple[float, float, float]]:
    try:
        start, end = edge.vertices()
        start_point = _point3(start.point())
        end_point = _point3(end.point())
    except Exception:
        return []
    if start_point == end_point:
        return [start_point]
    return [start_point, end_point]


def sample_curve_overlay(curve, samples: int = 32,
                         metadata: dict[str, object] | None = None) -> CurveSampleOverlay:
    """Sample a curve into a polyline overlay."""
    t0, t1 = _safe_range(*curve.parameter_range())
    points = [_point3(curve.evaluate(t)) for t in _linspace(t0, t1, samples)]
    data = {"samples": len(points), "parameter_range": (t0, t1)}
    if metadata:
        data.update(metadata)
    return CurveSampleOverlay(points=points, metadata=data)


def sample_surface_overlay(surface, samples_u: int = 8, samples_v: int = 8,
                           metadata: dict[str, object] | None = None) -> SurfaceSampleOverlay:
    """Sample a surface into a UV grid overlay."""
    (umin, vmin), (umax, vmax) = surface.uv_domain()
    umin, umax = _safe_range(umin, umax)
    vmin, vmax = _safe_range(vmin, vmax)
    us = _linspace(umin, umax, samples_u)
    vs = _linspace(vmin, vmax, samples_v)
    polylines = []
    for u in us:
        polylines.append([_point3(surface.evaluate(u, v)) for v in vs])
    for v in vs:
        polylines.append([_point3(surface.evaluate(u, v)) for u in us])
    data = {
        "u_samples": len(us),
        "v_samples": len(vs),
        "uv_domain": ((umin, vmin), (umax, vmax)),
    }
    if metadata:
        data.update(metadata)
    return SurfaceSampleOverlay(polylines=polylines, metadata=data)


def _sample_edge(edge, samples: int = 20) -> list[tuple[float, float, float]]:
    curve = edge.curve()
    t0, t1 = _safe_range(*curve.parameter_range())
    points = []
    for t in _linspace(t0, t1, samples):
        try:
            points.append(_point3(edge.point_at(t)))
        except Exception:
            points.append(_point3(curve.evaluate(t)))
    try:
        bounds = edge.bounding_box(True)
        padding = max(_bbox_diagonal(bounds) * 1e-4, 1e-6)
        if not _points_inside_bounds(points, bounds, padding):
            return _edge_endpoint_polyline(edge)
    except Exception:
        pass
    return points


def _sample_edgeuse(edgeuse, samples: int = 20) -> list[tuple[float, float, float]]:
    points = []
    for t in _linspace(0.0, 1.0, samples):
        points.append(_point3(edgeuse.point_at_normalized(t)))
    return points


def _index_of_handle(handles, handle) -> int:
    for index, candidate in enumerate(handles):
        if candidate is handle:
            return index
    return -1


def _primitive_counts(batches: list[dict[str, object]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for batch in batches:
        primitive = str(batch.get("primitive", "unknown"))
        counts[primitive] = counts.get(primitive, 0) + 1
    return counts


def kernel_draw_overlay(obj: ActiveObject, handle, topology_kind: str,
                        metadata: dict[str, object] | None = None,
                        variant: str = "default") -> DrawBatchOverlay:
    """Capture a kernel Draw() result as a GUI display-list overlay.

    Raises RuntimeError when the kernel was built without SM_GFX_OUTPUT_CODE.
    Callers should report that debug Draw overlays are unavailable and keep
    tessellated display.
    """
    from ..runtime import smdev

    if smdev is None or not hasattr(smdev.draw, "extract_draw_batches"):
        raise RuntimeError("This build does not expose _smlib_dev kernel draw extraction")
    batches = list(smdev.draw.extract_draw_batches(handle, variant=variant))
    if not batches:
        raise RuntimeError("Kernel Draw() returned no display batches")
    data = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "topology_kind": topology_kind,
        "source": "kernel_draw",
        "draw_variant": variant,
        "batch_count": len(batches),
        "primitive_counts": _primitive_counts(batches),
    }
    if metadata:
        data.update(metadata)
    return DrawBatchOverlay(batches=batches, metadata=data)


def brep_face_highlight_from_face(obj: ActiveObject, face, face_index: int = -1) -> TopologyOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep before adding a face highlight")
    edges = face.outer_loop_edges() or face.edges()
    polylines = [_sample_edge(edge) for edge in edges]
    (umin, vmin), (umax, vmax) = face.uv_domain()
    center = _point3(face.point_at_uv(0.5 * (umin + umax), 0.5 * (vmin + vmax)))
    if face_index < 0:
        face_index = _index_of_handle(obj.handle.faces(), face)
    metadata = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "topology_kind": "face",
        "face_index": face_index,
        "edge_count": len(edges),
        "area": face.area(),
    }
    return TopologyOverlay("face", polylines, [center], metadata)


def brep_face_highlight(obj: ActiveObject, face_index: int = 0) -> TopologyOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep before adding a face highlight")
    faces = obj.handle.faces()
    if not faces:
        raise RuntimeError("Selected BRep has no faces")
    index = min(max(face_index, 0), len(faces) - 1)
    face = faces[index]
    return brep_face_highlight_from_face(obj, face, index)


def brep_edge_highlight_from_edge(obj: ActiveObject, edge, edge_index: int = -1) -> TopologyOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep before adding an edge highlight")
    vertices = edge.vertices()
    if edge_index < 0:
        edge_index = _index_of_handle(obj.handle.edges(), edge)
    metadata = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "topology_kind": "edge",
        "edge_index": edge_index,
        "length": edge.length(),
    }
    return TopologyOverlay(
        "edge",
        [_sample_edge(edge)],
        [_point3(vertex.point()) for vertex in vertices],
        metadata,
    )


def brep_edge_highlight(obj: ActiveObject, edge_index: int = 0) -> TopologyOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep before adding an edge highlight")
    edges = obj.handle.edges()
    if not edges:
        raise RuntimeError("Selected BRep has no edges")
    index = min(max(edge_index, 0), len(edges) - 1)
    edge = edges[index]
    return brep_edge_highlight_from_edge(obj, edge, index)


def brep_edge_polylines(obj: ActiveObject, samples: int = 20) -> list[list[tuple[float, float, float]]]:
    """Sample every BRep edge into display polylines."""
    if obj.kind != "Brep":
        return []
    polylines = []
    for edge in obj.handle.edges():
        try:
            polylines.append(_sample_edge(edge, samples=samples))
        except Exception:
            logger.exception("Failed to sample BRep edge for overlay: edge=%r, samples=%s", edge, samples)
            continue
    return polylines


def brep_vertex_highlight(obj: ActiveObject, vertex_index: int = 0) -> TopologyOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep before adding a vertex highlight")
    vertices = obj.handle.vertices()
    if not vertices:
        raise RuntimeError("Selected BRep has no vertices")
    index = min(max(vertex_index, 0), len(vertices) - 1)
    vertex = vertices[index]
    metadata = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "topology_kind": "vertex",
        "vertex_index": index,
        "edge_count": len(vertex.edges()),
        "face_count": len(vertex.faces()),
    }
    return TopologyOverlay("vertex", [], [_point3(vertex.point())], metadata)


def brep_edgeuse_highlight(obj: ActiveObject, edgeuse, edge_index: int = -1,
                           face_index: int = -1, candidate_index: int = -1,
                           candidate_count: int = 1) -> TopologyOverlay:
    points = _sample_edgeuse(edgeuse)
    metadata = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "topology_kind": "edgeuse",
        "edge_index": edge_index,
        "face_index": face_index,
        "orientation": edgeuse.orientation() if hasattr(edgeuse, "orientation") else "",
        "direction_start": points[0] if points else None,
        "direction_end": points[-1] if points else None,
    }
    if candidate_index >= 0:
        metadata["edgeuse_cycle_index"] = candidate_index
        metadata["candidate_index"] = candidate_index
        metadata["candidate_count"] = candidate_count
    return TopologyOverlay(
        "edgeuse",
        [points],
        [points[0], points[-1]] if len(points) >= 2 else points,
        metadata,
    )


def brep_loopuse_highlight(obj: ActiveObject, loopuse, loop_index: int = -1,
                           face_index: int = -1, candidate_index: int = -1,
                           candidate_count: int = 1) -> TopologyOverlay:
    edgeuses = loopuse.edgeuses()
    polylines = [_sample_edgeuse(edgeuse) for edgeuse in edgeuses]
    metadata = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "topology_kind": "loopuse",
        "loop_index": loop_index,
        "face_index": face_index,
        "edgeuse_count": len(edgeuses),
        "orientation": loopuse.orientation() if hasattr(loopuse, "orientation") else "",
    }
    if candidate_index >= 0:
        metadata["loopuse_cycle_index"] = candidate_index
        metadata["candidate_index"] = candidate_index
        metadata["candidate_count"] = candidate_count
    return TopologyOverlay("loopuse", polylines, [], metadata)


def brep_face_surface_overlay(obj: ActiveObject, face, face_index: int = -1) -> SurfaceSampleOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep before adding a surface overlay")
    if face_index < 0:
        face_index = _index_of_handle(obj.handle.faces(), face)
    return sample_surface_overlay(
        face.surface(),
        metadata={
            "object_id": obj.object_id,
            "object_name": obj.name,
            "source": "face",
            "face_index": face_index,
        },
    )


def section_overlay(obj: ActiveObject, plane_origin=None, plane_normal=(0.0, 0.0, 1.0),
                    zone_tolerance: float | None = None) -> SectionOverlay:
    poly = obj.display_handle
    if poly is None and obj.kind == "PolyBrep":
        poly = obj.handle
    if poly is None or not hasattr(poly, "section_plane"):
        raise RuntimeError(f"{obj.kind} does not expose section-plane overlays")
    mn, mx = object_bounds(obj)
    if plane_origin is None:
        plane_origin = object_center(obj)
    diagonal = (
        (mx[0] - mn[0]) ** 2
        + (mx[1] - mn[1]) ** 2
        + (mx[2] - mn[2]) ** 2
    ) ** 0.5
    if zone_tolerance is None:
        zone_tolerance = max(diagonal * 1e-6, 1e-6)
    segments = [
        (_point3(start), _point3(end))
        for start, end in poly.section_plane(tuple(plane_origin), tuple(plane_normal), zone_tolerance)
    ]
    if not segments:
        raise RuntimeError("Section plane did not intersect the selected object")
    metadata = {
        "object_id": obj.object_id,
        "object_name": obj.name,
        "plane_origin": _point3(plane_origin),
        "plane_normal": _point3(plane_normal),
        "zone_tolerance": zone_tolerance,
        "segments": len(segments),
    }
    return SectionOverlay(segments=segments, metadata=metadata)


def is_overlay(obj: ActiveObject) -> bool:
    return obj.kind in OVERLAY_KINDS


def make_overlay_object(name: str, kind: str, handle: object, object_id: int) -> ActiveObject:
    if kind not in OVERLAY_KINDS:
        raise ValueError(f"Unsupported overlay kind: {kind}")
    return ActiveObject(
        object_id=object_id,
        name=name,
        kind=kind,
        handle=handle,
        color=DEBUG_OVERLAY_COLOR,
    )


def object_bounds(obj: ActiveObject) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if hasattr(obj.handle, "bounding_box"):
        return obj.handle.bounding_box()
    if hasattr(obj.handle, "calculate_bounding_box"):
        return obj.handle.calculate_bounding_box()
    if obj.mesh:
        points = obj.mesh.get("points", ())
        if len(points) > 0:
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            zs = [point[2] for point in points]
            return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))
    raise RuntimeError(f"{obj.kind} does not expose bounds")


def object_center(obj: ActiveObject) -> tuple[float, float, float]:
    if hasattr(obj.handle, "center"):
        return obj.handle.center()
    mn, mx = object_bounds(obj)
    return (
        0.5 * (mn[0] + mx[0]),
        0.5 * (mn[1] + mx[1]),
        0.5 * (mn[2] + mx[2]),
    )


def brep_face_normal(obj: ActiveObject) -> NormalOverlay:
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep to create a face normal overlay")
    faces = obj.handle.faces()
    if not faces:
        raise RuntimeError("Selected BRep has no faces")
    face = faces[0]
    (umin, vmin), (umax, vmax) = face.uv_domain()
    u = 0.5 * (umin + umax)
    v = 0.5 * (vmin + vmax)
    origin = _point3(face.point_at_uv(u, v))
    vector = _point3(face.outward_normal_at_uv(u, v))
    mn, mx = object_bounds(obj)
    diagonal = (
        (mx[0] - mn[0]) ** 2
        + (mx[1] - mn[1]) ** 2
        + (mx[2] - mn[2]) ** 2
    ) ** 0.5
    return NormalOverlay(origin=origin, vector=vector, scale=max(diagonal * 0.15, 1.0))


def brep_face_normals(obj: ActiveObject) -> list[NormalOverlay]:
    """Return one midpoint normal overlay for each face in a BRep."""
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep to create face normal overlays")
    faces = obj.handle.faces()
    if not faces:
        raise RuntimeError("Selected BRep has no faces")
    mn, mx = object_bounds(obj)
    diagonal = (
        (mx[0] - mn[0]) ** 2
        + (mx[1] - mn[1]) ** 2
        + (mx[2] - mn[2]) ** 2
    ) ** 0.5
    scale = max(diagonal * 0.12, 1.0)
    normals = []
    for face in faces:
        try:
            (umin, vmin), (umax, vmax) = face.uv_domain()
            u = 0.5 * (umin + umax)
            v = 0.5 * (vmin + vmax)
            normals.append(NormalOverlay(
                origin=_point3(face.point_at_uv(u, v)),
                vector=_point3(face.outward_normal_at_uv(u, v)),
                scale=scale,
            ))
        except Exception:
            continue
    if not normals:
        raise RuntimeError("No face normals could be evaluated")
    return normals
