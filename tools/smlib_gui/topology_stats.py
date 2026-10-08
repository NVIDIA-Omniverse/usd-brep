# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""BRep-first and mesh topology count helpers for the SMLib GUI HUD."""

from __future__ import annotations

from .runtime import smdev
from .settings import (
    TOPOLOGY_COUNTS_BREP,
    TOPOLOGY_COUNTS_BREP_DETAIL,
    TOPOLOGY_COUNTS_MESH,
)


def _safe_int(getter) -> int | None:
    try:
        return int(getter())
    except Exception:
        return None


def _mesh_edge_count(faces) -> int:
    """Count unique undirected edges in a VTK face stream."""
    flat = list(faces) if faces is not None else []
    edges: set[tuple[int, int]] = set()
    offset = 0
    while offset < len(flat):
        try:
            count = int(flat[offset])
        except (TypeError, ValueError):
            break
        if count < 2 or offset + count + 1 > len(flat):
            break
        indices = [int(flat[offset + 1 + i]) for i in range(count)]
        for i in range(count):
            a = indices[i]
            b = indices[(i + 1) % count]
            edges.add((a, b) if a <= b else (b, a))
        offset += count + 1
    return len(edges)


def _brep_detail_from_smdev(brep) -> dict[str, int | None]:
    """Prefer ``smdev.topology_counts``; fall back to face/loop walks."""
    if smdev is not None and hasattr(smdev, "topology_counts"):
        try:
            counts = smdev.topology_counts(brep)
            return {
                "regions": int(counts.get("regions", 0)),
                "shells": int(counts.get("shells", 0)),
                "faces": int(counts.get("faces", 0)),
                "loops": int(counts.get("loops", 0)),
                "edges": int(counts.get("edges", 0)),
                "vertices": int(counts.get("vertices", 0)),
            }
        except Exception:
            pass

    faces = _safe_int(brep.face_count)
    edges = _safe_int(brep.edge_count)
    vertices = _safe_int(brep.vertex_count)
    loops = None
    if smdev is not None and hasattr(brep, "faces"):
        try:
            loops = 0
            for face in brep.faces():
                loops += len(smdev.face_loops(face))
        except Exception:
            loops = None
    return {
        "regions": None,
        "shells": None,
        "faces": faces,
        "loops": loops,
        "edges": edges,
        "vertices": vertices,
    }


def _mesh_edge_count_from_objects_and_meshes(objects, meshes) -> int:
    edge_count = 0
    for obj in objects:
        mesh = getattr(obj, "mesh", None) or {}
        faces = mesh.get("faces")
        if faces is not None:
            edge_count += _mesh_edge_count(faces)
    if edge_count > 0:
        return edge_count
    for mesh in meshes:
        try:
            faces = getattr(mesh, "faces", None)
            if faces is not None and len(faces) > 0:
                edge_count += _mesh_edge_count(faces)
        except Exception:
            continue
    return edge_count


def topology_count_rows(objects, meshes, mode: str) -> list[tuple[str, int | None]]:
    """Return ordered ``(label, count)`` rows for the HUD right column."""
    visible = [obj for obj in objects if getattr(obj, "visible", True)]
    breps = [obj for obj in visible if getattr(obj, "kind", None) == "Brep"]

    if mode == TOPOLOGY_COUNTS_MESH:
        vertex_count = sum(int(mesh.n_points) for mesh in meshes)
        face_count = sum(int(mesh.n_cells) for mesh in meshes)
        edge_count = _mesh_edge_count_from_objects_and_meshes(visible, meshes)
        return [
            ("Objects", len(visible)),
            ("Faces", face_count),
            ("Edges", edge_count),
            ("Vertices", vertex_count),
        ]

    if mode == TOPOLOGY_COUNTS_BREP_DETAIL:
        regions = shells = faces = loops = edges = vertices = 0
        missing_regions = missing_shells = missing_loops = False
        for obj in breps:
            detail = _brep_detail_from_smdev(obj.handle)
            if detail["regions"] is None:
                missing_regions = True
            else:
                regions += detail["regions"]
            if detail["shells"] is None:
                missing_shells = True
            else:
                shells += detail["shells"]
            if detail["faces"] is not None:
                faces += detail["faces"]
            if detail["loops"] is None:
                missing_loops = True
            else:
                loops += detail["loops"]
            if detail["edges"] is not None:
                edges += detail["edges"]
            if detail["vertices"] is not None:
                vertices += detail["vertices"]
        return [
            ("Objects", len(visible)),
            ("Regions", None if (missing_regions or not breps) else regions),
            ("Shells", None if (missing_shells or not breps) else shells),
            ("Faces", faces),
            ("Loops", None if (missing_loops or not breps) else loops),
            ("Edges", edges),
            ("Vertices", vertices),
        ]

    # Default: compact BRep Face / Edge / Vertex.
    face_count = edge_count = vertex_count = 0
    for obj in breps:
        face_count += _safe_int(obj.handle.face_count) or 0
        edge_count += _safe_int(obj.handle.edge_count) or 0
        vertex_count += _safe_int(obj.handle.vertex_count) or 0
    return [
        ("Objects", len(visible)),
        ("Faces", face_count),
        ("Edges", edge_count),
        ("Vertices", vertex_count),
    ]


def topology_counts_label(mode: str) -> str:
    labels = {
        TOPOLOGY_COUNTS_BREP: "BRep F/E/V",
        TOPOLOGY_COUNTS_BREP_DETAIL: "BRep detail",
        TOPOLOGY_COUNTS_MESH: "Mesh F/E/V",
    }
    return labels.get(mode, mode)


__all__ = [
    "topology_count_rows",
    "topology_counts_label",
]
