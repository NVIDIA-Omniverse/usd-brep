# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Exact topology picking helpers for the SMLib GUI."""

from __future__ import annotations

from collections.abc import Iterable
import logging
from dataclasses import dataclass

from .overlays import object_bounds
from ..model import ActiveObject
from ..runtime import smdev

logger = logging.getLogger(__name__)

_EXPECTED_TOPOLOGY_QUERY_ERRORS = (AttributeError, TypeError, ValueError, IndexError)


def _log_topology_query_failure(action: str, **context) -> None:
    details = ", ".join(f"{key}={value!r}" for key, value in context.items())
    suffix = f": {details}" if details else ""
    logger.warning("Topology pick query failed during %s%s", action, suffix)


def _call_topology_method(owner, method_name: str, *args, **context):
    method = getattr(owner, method_name, None)
    if method is None:
        return None
    try:
        return method(*args)
    except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
        _log_topology_query_failure(method_name, owner=owner, **context)
        return None


@dataclass
class TopologyPick:
    object_id: int
    object_name: str
    topology_kind: str
    topology_index: int
    handle: object
    hit_point: tuple[float, float, float]
    ray_depth: float
    normal: tuple[float, float, float] | None = None
    face: object | None = None
    face_index: int = -1
    edge: object | None = None
    edge_index: int = -1
    vertex: object | None = None
    vertex_index: int = -1
    loop: object | None = None
    loop_index: int = -1
    edgeuse: object | None = None
    loopuse: object | None = None
    edgeuses: tuple[object, ...] = ()
    loops: tuple[object, ...] = ()
    loopuses: tuple[object, ...] = ()
    solver_hit_index: int = -1

    def metadata(self) -> dict[str, object]:
        data = {
            "object_id": self.object_id,
            "object_name": self.object_name,
            "topology_kind": self.topology_kind,
            "topology_index": self.topology_index,
            "ray_depth": self.ray_depth,
            "hit_point": self.hit_point,
        }
        if self.normal is not None:
            data["normal"] = self.normal
        if self.face_index >= 0:
            data["face_index"] = self.face_index
        if self.edge_index >= 0:
            data["edge_index"] = self.edge_index
        if self.vertex_index >= 0:
            data["vertex_index"] = self.vertex_index
        if self.loop_index >= 0:
            data["loop_index"] = self.loop_index
        if self.edgeuses:
            data["edgeuse_count"] = len(self.edgeuses)
        if self.loops:
            data["loop_count"] = len(self.loops)
        if self.loopuses:
            data["loopuse_count"] = len(self.loopuses)
        if self.solver_hit_index >= 0:
            data["solver_hit_index"] = self.solver_hit_index
        return data


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _normalize(v):
    length = (_dot(v, v)) ** 0.5
    if length <= 0.0:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _length(v) -> float:
    return (_dot(v, v)) ** 0.5


def _distance(a, b) -> float:
    return _length(_sub(a, b))


def _as_point3(value) -> tuple[float, float, float]:
    return (float(value[0]), float(value[1]), float(value[2]))


def _handle_index(handles, handle) -> int:
    for index, candidate in enumerate(handles):
        if candidate is handle:
            return index
    return -1


def _append_unique(handles: list[object], handle) -> None:
    if handle is None:
        return
    if not any(candidate is handle for candidate in handles):
        handles.append(handle)


def _topology_pick_tolerance(obj: ActiveObject) -> float:
    try:
        mn, mx = object_bounds(obj)
    except Exception:
        return 1.0e-4
    diagonal = _distance(mn, mx)
    return max(diagonal * 0.02, 1.0e-5)


def _loop_index(face, loop) -> int:
    if smdev is None or face is None or loop is None:
        return -1
    try:
        loops = smdev.face_loops(face)
    except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
        _log_topology_query_failure("face_loops", face=face)
        return -1
    return _handle_index(loops, loop)


def _face_context_for_edge(obj: ActiveObject, edge, face):
    if edge is None:
        return face, _handle_index(obj.handle.faces(), face) if face is not None else -1
    incident_faces = _call_topology_method(edge, "faces", edge=edge) or []
    if face is not None and any(candidate is face for candidate in incident_faces):
        return face, _handle_index(obj.handle.faces(), face)
    face = incident_faces[0] if incident_faces else face
    return face, _handle_index(obj.handle.faces(), face) if face is not None else -1


def _edge_context_candidates(edge, face) -> tuple[tuple[object, ...], tuple[object, ...], tuple[object, ...]]:
    edgeuses: list[object] = []
    loops: list[object] = []
    loopuses: list[object] = []

    def add_loop(loop) -> None:
        _append_unique(loops, loop)

    def add_loopuse(loopuse) -> None:
        _append_unique(loopuses, loopuse)
        if loopuse is None:
            return
        add_loop(_call_topology_method(loopuse, "loop", loopuse=loopuse))

    def add_edgeuse(edgeuse) -> None:
        _append_unique(edgeuses, edgeuse)
        if edgeuse is None:
            return
        add_loopuse(_call_topology_method(edgeuse, "loopuse", edgeuse=edgeuse))
        add_loop(_call_topology_method(edgeuse, "loop", edgeuse=edgeuse))

    if edge is None or smdev is None:
        return (), (), ()

    if face is not None:
        try:
            add_edgeuse(smdev.edgeuse_of_face(edge, face))
            add_loop(smdev.loop_of_face(edge, face))
        except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
            _log_topology_query_failure("edgeuse_of_face", edge=edge, face=face)

    try:
        for edgeuse in smdev.edgeuses(edge):
            add_edgeuse(edgeuse)
    except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
        _log_topology_query_failure("edgeuses", edge=edge)

    try:
        for loopuse in smdev.loopuses(edge):
            add_loopuse(loopuse)
    except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
        _log_topology_query_failure("loopuses", edge=edge)

    try:
        for loop in smdev.loops(edge):
            add_loop(loop)
    except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
        _log_topology_query_failure("loops", edge=edge)

    return tuple(edgeuses), tuple(loops), tuple(loopuses)


def _resolve_edgeuse(edge, face):
    edgeuses, loops, loopuses = _edge_context_candidates(edge, face)
    edgeuse = edgeuses[0] if edgeuses else None
    loop = loops[0] if loops else None
    loopuse = loopuses[0] if loopuses else None
    return edgeuse, loop, loopuse, edgeuses, loops, loopuses


def _make_vertex_pick(obj: ActiveObject, vertex_index: int, vertex, point, depth: float,
                      normal, face, face_index: int, edge=None, edge_index: int = -1) -> TopologyPick:
    if edge is None:
        vertex_edges = _call_topology_method(vertex, "edges", vertex=vertex)
        if vertex_edges is not None:
            edge = vertex_edges[0] if vertex_edges else None
            edge_index = _handle_index(obj.handle.edges(), edge) if edge is not None else -1
        else:
            edge = None
            edge_index = -1
    if face is None:
        vertex_faces = _call_topology_method(vertex, "faces", vertex=vertex)
        if vertex_faces is not None:
            face = vertex_faces[0] if vertex_faces else None
            face_index = _handle_index(obj.handle.faces(), face) if face is not None else -1
        else:
            face = None
            face_index = -1
    face, face_index = _face_context_for_edge(obj, edge, face)
    edgeuse, loop, loopuse, edgeuses, loops, loopuses = _resolve_edgeuse(edge, face)
    return TopologyPick(
        object_id=obj.object_id,
        object_name=obj.name,
        topology_kind="Vertex",
        topology_index=vertex_index,
        handle=vertex,
        hit_point=_as_point3(point),
        ray_depth=float(depth),
        normal=normal,
        face=face,
        face_index=face_index,
        edge=edge,
        edge_index=edge_index,
        vertex=vertex,
        vertex_index=vertex_index,
        loop=loop,
        loop_index=_loop_index(face, loop),
        edgeuse=edgeuse,
        loopuse=loopuse,
        edgeuses=edgeuses,
        loops=loops,
        loopuses=loopuses,
    )


def _make_edge_pick(obj: ActiveObject, edge_index: int, edge, point, depth: float,
                    normal, face, face_index: int, vertex=None, vertex_index: int = -1) -> TopologyPick:
    face, face_index = _face_context_for_edge(obj, edge, face)
    edgeuse, loop, loopuse, edgeuses, loops, loopuses = _resolve_edgeuse(edge, face)
    return TopologyPick(
        object_id=obj.object_id,
        object_name=obj.name,
        topology_kind="Edge",
        topology_index=edge_index,
        handle=edge,
        hit_point=_as_point3(point),
        ray_depth=float(depth),
        normal=normal,
        face=face,
        face_index=face_index,
        edge=edge,
        edge_index=edge_index,
        vertex=vertex,
        vertex_index=vertex_index,
        loop=loop,
        loop_index=_loop_index(face, loop),
        edgeuse=edgeuse,
        loopuse=loopuse,
        edgeuses=edgeuses,
        loops=loops,
        loopuses=loopuses,
    )


def _make_face_pick(obj: ActiveObject, face_index: int, face, point, depth: float,
                    normal, edge=None, edge_index: int = -1, vertex=None,
                    vertex_index: int = -1) -> TopologyPick:
    face_loops = None
    if smdev is not None:
        try:
            face_loops = smdev.face_loops(face)
        except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
            _log_topology_query_failure("face_loops", face=face)
    if face_loops is not None:
        loop = face_loops[0] if face_loops else None
        loopuse = _call_topology_method(loop, "loopuse", loop=loop) if loop is not None else None
        loop_index = 0 if loop is not None else -1
    else:
        loop = None
        loopuse = None
        loop_index = -1
    edgeuse, edge_loop, edge_loopuse, edgeuses, edge_loops, edge_loopuses = _resolve_edgeuse(edge, face)
    if loop is None:
        loop = edge_loop
        loopuse = edge_loopuse
        loop_index = _loop_index(face, loop)
    return TopologyPick(
        object_id=obj.object_id,
        object_name=obj.name,
        topology_kind="Face",
        topology_index=face_index,
        handle=face,
        hit_point=_as_point3(point),
        ray_depth=float(depth),
        normal=normal,
        face=face,
        face_index=face_index,
        edge=edge,
        edge_index=edge_index,
        vertex=vertex,
        vertex_index=vertex_index,
        loop=loop,
        loop_index=loop_index,
        edgeuse=edgeuse,
        loopuse=loopuse,
        edgeuses=edgeuses,
        loops=edge_loops,
        loopuses=edge_loopuses,
    )


def _with_solver_hit_index(pick: TopologyPick | None, hit_index: int) -> TopologyPick | None:
    if pick is not None:
        pick.solver_hit_index = hit_index
    return pick


def _exact_topology_pick_from_hit(obj: ActiveObject, hit: dict, tolerance: float,
                                  hit_index: int) -> TopologyPick | None:
    kind = str(hit.get("topology_kind") or hit.get("kind") or "")
    handle = hit.get("topology")
    point = hit.get("point")
    if not kind or handle is None or point is None:
        return None
    point = _as_point3(point)
    depth = float(hit.get("ray_depth", hit.get("line_parameter", 0.0)))
    if depth < -tolerance:
        return None
    normal = hit.get("normal")
    normal = _as_point3(normal) if normal is not None else None
    topology_index = int(hit.get("topology_index", -1))

    if kind == "Vertex":
        vertex = hit.get("vertex") or handle
        if topology_index < 0:
            try:
                topology_index = _handle_index(obj.handle.vertices(), vertex)
            except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
                _log_topology_query_failure("brep.vertices", object=obj, vertex=vertex)
                topology_index = -1
        return _with_solver_hit_index(_make_vertex_pick(
            obj,
            topology_index,
            vertex,
            point,
            depth,
            normal,
            hit.get("face"),
            -1,
            edge=hit.get("edge"),
        ), hit_index)

    if kind == "Edge":
        edge = hit.get("edge") or handle
        if topology_index < 0:
            try:
                topology_index = _handle_index(obj.handle.edges(), edge)
            except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
                _log_topology_query_failure("brep.edges", object=obj, edge=edge)
                topology_index = -1
        return _with_solver_hit_index(_make_edge_pick(
            obj,
            topology_index,
            edge,
            point,
            depth,
            normal,
            hit.get("face"),
            -1,
            vertex=hit.get("vertex"),
        ), hit_index)

    if kind == "Face":
        face = hit.get("face") or handle
        if topology_index < 0:
            try:
                topology_index = _handle_index(obj.handle.faces(), face)
            except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
                _log_topology_query_failure("brep.faces", object=obj, face=face)
                topology_index = -1
        return _with_solver_hit_index(_make_face_pick(
            obj,
            topology_index,
            face,
            point,
            depth,
            normal,
        ), hit_index)

    return None


def _exact_topology_hits_for_ray(obj: ActiveObject, origin, direction) -> list[TopologyPick]:
    if smdev is None:
        return []
    tolerance = _topology_pick_tolerance(obj)
    try:
        raw_hits = smdev.topology_pick_ray(
            obj.handle, tuple(origin), tuple(direction), tolerance
        )
    except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
        _log_topology_query_failure("topology_pick_ray", object=obj, origin=origin, direction=direction)
        return []
    if raw_hits is None or not isinstance(raw_hits, Iterable):
        _log_topology_query_failure("topology_pick_ray", object=obj, origin=origin, direction=direction)
        return []
    hits = []
    for hit_index, hit in enumerate(raw_hits):
        try:
            pick = _exact_topology_pick_from_hit(obj, hit, tolerance, hit_index)
        except _EXPECTED_TOPOLOGY_QUERY_ERRORS:
            _log_topology_query_failure("exact_topology_pick_from_hit", object=obj, hit=hit)
            pick = None
        if pick is not None:
            hits.append(pick)
    return hits


def _kind_priority(kind: str) -> int:
    return {"Vertex": 0, "Edge": 1, "Face": 2}.get(kind, 3)


def topology_hits_for_ray(objects: list[ActiveObject], ray_point, ray_direction) -> list[TopologyPick]:
    """Return sorted topology hits for all visible BRep objects along a ray."""
    origin = _as_point3(ray_point)
    direction = _normalize(_as_point3(ray_direction))
    hits: list[TopologyPick] = []
    for obj in objects:
        if obj.kind != "Brep" or not obj.visible:
            continue
        hits.extend(_exact_topology_hits_for_ray(obj, origin, direction))

    if not hits:
        return []
    best_priority = min(_kind_priority(pick.topology_kind) for pick in hits)
    hits = [pick for pick in hits if _kind_priority(pick.topology_kind) == best_priority]
    hits.sort(key=lambda pick: (pick.ray_depth, pick.object_id, pick.topology_index))
    return hits
