# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Tessellation and PyVista mesh conversion for the SMLib GUI."""

from __future__ import annotations

from collections import defaultdict

from .model import ActiveObject
from .runtime import np, pv, sm
from .settings import TessellationSettings


def _mesh_has_geometry(mesh: dict | None) -> bool:
    if not mesh:
        return False
    return len(mesh.get("points", ())) > 0 and len(mesh.get("faces", ())) > 0


def mesh_error_summary(objects: list[ActiveObject]) -> str:
    """Return a 'name: reason' summary for objects left without a display mesh."""
    return "; ".join(f"{obj.name}: {obj.mesh_error}" for obj in objects if obj.mesh_error)


def tessellate_objects(objects: list[ActiveObject], chord: float = 0.05,
                       angle: float = 25.0, max_edge_length: float = 0.0,
                       max_aspect_ratio: float = 0.0,
                       settings: TessellationSettings | None = None) -> list[ActiveObject]:
    """Populate display meshes for registry entries that can be rendered."""
    if settings is None:
        settings = TessellationSettings(
            chord_height_tolerance=chord,
            angle_tolerance_deg=angle,
            max_edge_length=max_edge_length,
            max_aspect_ratio=max_aspect_ratio,
        )
    for obj in objects:
        obj.mesh = None
        obj.display_handle = None
        obj.mesh_error = ""
        if not obj.visible:
            continue
        if obj.kind == "Brep":
            pb = sm.tessellate(
                obj.handle,
                chord_height_tolerance=settings.chord_height_tolerance,
                curve_angle_tolerance_deg=settings.angle_tolerance_deg,
                surface_angle_tolerance_deg=settings.angle_tolerance_deg,
                max_edge_length=settings.max_edge_length,
                max_aspect_ratio=settings.max_aspect_ratio,
            )
            if pb is None:
                # ConvertToPolyBrep can report SM_SUCCESS yet return no PolyBrep,
                # for example when the Brep has no faces to mesh.
                obj.mesh_error = "tessellation returned no PolyBrep"
                continue
            obj.display_handle = pb
            obj.mesh = pb.to_numpy_arrays()
        elif obj.kind == "PolyBrep":
            if obj.handle is None:
                obj.mesh_error = "no PolyBrep handle to display"
                continue
            obj.display_handle = obj.handle
            obj.mesh = obj.handle.to_numpy_arrays()
        elif obj.kind in {"Surface", "Curve"}:
            obj.mesh_error = f"no display path for {obj.kind}; use it to construct or trim a Brep"
        if obj.display_handle is not None and not _mesh_has_geometry(obj.mesh):
            obj.mesh_error = "tessellation produced an empty mesh"
    return objects


def orient_face_stream_to_normals(points, faces, face_normals):
    """Flip polygon windings so geometric normals agree with ``face_normals``.

    Tessellation stores BRep-oriented ``face_normals`` (via SmTess reverse based
    on upward faceuse / infinite region), but the extracted VTK face stream can
    still disagree with those normals. Aligning winding here makes VTK
    front/back classification match BRep outside/inside.
    """
    points = np.asarray(points, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64).reshape(-1).copy()
    if face_normals is None:
        return faces.astype(np.int32, copy=False)
    face_normals = np.asarray(face_normals, dtype=np.float64).reshape(-1, 3)
    offset = 0
    face_index = 0
    while offset < len(faces) and face_index < len(face_normals):
        count = int(faces[offset])
        end = offset + count + 1
        if count < 3 or end > len(faces):
            break
        i0 = int(faces[offset + 1])
        i1 = int(faces[offset + 2])
        i2 = int(faces[offset + 3])
        geometric = np.cross(points[i1] - points[i0], points[i2] - points[i0])
        if float(np.dot(geometric, face_normals[face_index])) < 0.0:
            faces[offset + 1:end] = faces[offset + 1:end][::-1]
        offset = end
        face_index += 1
    return faces.astype(np.int32, copy=False)


def objects_to_pyvista(objects: list[ActiveObject],
                       orient_to_face_normals: bool = False) -> list[pv.PolyData]:
    """Convert tessellated registry entries to disposable PyVista meshes."""
    meshes = []
    for obj in objects:
        if not obj.mesh:
            continue
        pts = np.asarray(obj.mesh.get("points", ()), dtype=np.float32)
        flat = obj.mesh.get("faces", ())
        if len(pts) == 0 or len(flat) == 0:
            continue
        if orient_to_face_normals:
            faces_arr = orient_face_stream_to_normals(
                pts, flat, obj.mesh.get("face_normals"))
        else:
            faces_arr = np.asarray(flat, dtype=np.int32)
        poly = pv.PolyData(pts, faces=faces_arr)
        normals = obj.mesh.get("normals")
        if normals is not None and len(normals) == len(pts):
            poly.point_data["Normals"] = np.asarray(normals, dtype=np.float32)
            poly.GetPointData().SetActiveNormals("Normals")
        poly.field_data["DisplayColor"] = np.array(obj.color, dtype=np.float32)
        poly.field_data["ObjectID"] = np.array([obj.object_id], dtype=np.int32)
        poly.field_data["ObjectKind"] = np.array([obj.kind])
        poly.field_data["PartName"] = np.array([obj.name])
        meshes.append(poly)
    return meshes


def _offset_face_indices(flat_faces, point_offset: int) -> tuple[np.ndarray, int]:
    """Return one VTK face stream with point indices shifted by an offset."""
    faces = np.asarray(flat_faces, dtype=np.int64).reshape(-1).copy()
    cursor = 0
    cell_count = 0
    while cursor < len(faces):
        vertex_count = int(faces[cursor])
        end = cursor + vertex_count + 1
        if vertex_count <= 0 or end > len(faces):
            raise RuntimeError("Invalid polygon face stream while batching display meshes")
        faces[cursor + 1:end] += point_offset
        cursor = end
        cell_count += 1
    return faces, cell_count


def objects_to_pyvista_batches(objects: list[ActiveObject],
                               orient_to_face_normals: bool = False) -> list[pv.PolyData]:
    """Combine display meshes into a small number of color/shading batches."""
    groups = defaultdict(list)
    for obj in objects:
        if not obj.mesh:
            continue
        points = np.asarray(obj.mesh.get("points", ()), dtype=np.float32)
        flat_faces = obj.mesh.get("faces", ())
        if len(points) == 0 or len(flat_faces) == 0:
            continue
        if orient_to_face_normals:
            flat_faces = orient_face_stream_to_normals(
                points, flat_faces, obj.mesh.get("face_normals"))
        normals = obj.mesh.get("normals")
        has_normals = normals is not None and len(normals) == len(points)
        key = (tuple(float(component) for component in obj.color), has_normals)
        groups[key].append((obj, points, flat_faces, normals))

    batches = []
    for (color, has_normals), parts in groups.items():
        point_arrays = []
        face_arrays = []
        normal_arrays = []
        object_id_arrays = []
        point_offset = 0
        part_names = []
        object_kinds = []
        object_ids = []
        for obj, points, flat_faces, normals in parts:
            faces, cell_count = _offset_face_indices(flat_faces, point_offset)
            point_arrays.append(points)
            face_arrays.append(faces)
            object_id_arrays.append(
                np.full(cell_count, obj.object_id, dtype=np.int32)
            )
            if has_normals:
                normal_arrays.append(np.asarray(normals, dtype=np.float32))
            point_offset += len(points)
            part_names.append(obj.name)
            object_kinds.append(obj.kind)
            object_ids.append(obj.object_id)

        poly = pv.PolyData(
            np.concatenate(point_arrays, axis=0),
            faces=np.concatenate(face_arrays),
        )
        if has_normals:
            poly.point_data["Normals"] = np.concatenate(normal_arrays, axis=0)
            poly.GetPointData().SetActiveNormals("Normals")
        poly.cell_data["ObjectID"] = np.concatenate(object_id_arrays)
        poly.field_data["DisplayColor"] = np.asarray(color, dtype=np.float32)
        poly.field_data["ObjectIDs"] = np.asarray(object_ids, dtype=np.int32)
        poly.field_data["ObjectKinds"] = np.asarray(object_kinds)
        poly.field_data["PartNames"] = np.asarray(part_names)
        batches.append(poly)
    return batches
