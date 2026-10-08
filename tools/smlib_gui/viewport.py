# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""PyVista viewport projection and camera helpers for the SMLib GUI."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .operations.overlays import (
    BoxOverlay,
    CurveSampleOverlay,
    DrawBatchOverlay,
    LineOverlay,
    NormalOverlay,
    PickOverlay,
    PointOverlay,
    PolylineOverlay,
    SectionOverlay,
    SurfaceSampleOverlay,
    TopologyOverlay,
    brep_edge_polylines,
    brep_face_normals,
    is_overlay,
)
from .runtime import np, pv
from .settings import (
    COLOR_MODE_BREP_ORIENTATION,
    DISPLAY_MODE_MESH_WIREFRAME,
    DISPLAY_MODES_WITH_MESH_WIREFRAME,
    DISPLAY_MODES_WITH_SHADED_SURFACE,
    NAMED_VIEW_NAMES,
    NORMALS_MODE_BREP_FACE,
    NORMALS_MODE_MESH_EDGE,
    NORMALS_MODE_MESH_FACE,
    NORMALS_MODE_MESH_VERTEX,
    NORMALS_MODE_OFF,
    UP_AXIS_Y,
    UP_AXIS_Z,
    color_mode_uses_orientation_backfaces,
)


DEBUG_OVERLAY_LINE_WIDTH = 6
DEBUG_OVERLAY_SURFACE_LINE_WIDTH = 4
DEBUG_OVERLAY_MESH_EDGE_WIDTH = 4
DEBUG_TOPOLOGY_LINE_WIDTH = 8
DEBUG_TOPOLOGY_SECONDARY_LINE_WIDTH = 6

BACKFACE_COLOR = (0.8, 0.2, 0.2)


def _view_up_vector(up_axis: str,
                    offset: tuple[float, float, float] | None = None,
                    ) -> tuple[float, float, float]:
    """Return a view-up vector that is never parallel to the view direction.

    VTK silently discards a view-up that is parallel to the view plane normal,
    so axis-aligned views along the up axis (top/bottom for Z-up) need an
    in-plane substitute. Mirroring the substitute for the opposite side keeps
    the CAD convention that a bottom view is the vertical mirror of the top.
    """
    if up_axis == UP_AXIS_Y:
        nominal = (0.0, 1.0, 0.0)
        substitute = (0.0, 0.0, 1.0)
    else:
        nominal = (0.0, 0.0, 1.0)
        substitute = (0.0, 1.0, 0.0)
    if offset is None:
        return nominal

    direction = _normalize(offset)
    along_up = sum(direction[i] * nominal[i] for i in range(3))
    if abs(along_up) < 0.999:
        return nominal
    sign = -1.0 if along_up < 0.0 else 1.0
    return tuple(sign * component for component in substitute)


def _camera_offset(view_name: str, distance: float) -> tuple[float, float, float] | None:
    """Return camera offset from scene center for CAD-style named views."""
    az = math.radians(25.0)
    el = math.radians(15.0)
    ce = math.cos(el)
    fx = math.sin(az) * ce
    fy = -math.cos(az) * ce
    fz = math.sin(el)
    iso = 1.0 / math.sqrt(3.0)
    offsets = {
        "front": (0.0, -distance, 0.0),
        "back": (0.0, distance, 0.0),
        "left": (-distance, 0.0, 0.0),
        "right": (distance, 0.0, 0.0),
        "top": (0.0, 0.0, distance),
        "bottom": (0.0, 0.0, -distance),
        "isometric": (iso * distance, -iso * distance, iso * distance),
        "front-iso": (fx * distance, fy * distance, fz * distance),
        "front-iso-left": (-fx * distance, fy * distance, fz * distance),
    }
    return offsets.get(view_name)


def _scene_bounds(plotter) -> tuple[float, float, float, float, float, float]:
    bounds = plotter.bounds
    if bounds is None:
        return (0.0, 1.0, 0.0, 1.0, 0.0, 1.0)
    return tuple(float(value) for value in bounds)


def _scene_center_and_distance(plotter) -> tuple[tuple[float, float, float], float]:
    xmin, xmax, ymin, ymax, zmin, zmax = _scene_bounds(plotter)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    diag = max(xmax - xmin, ymax - ymin, zmax - zmin, 1e-6)
    return (cx, cy, cz), 2.5 * diag


@dataclass
class CameraState:
    """Serializable snapshot of the current PyVista camera."""

    position: tuple[float, float, float]
    focal_point: tuple[float, float, float]
    view_up: tuple[float, float, float]
    parallel_projection: bool


def _tuple3(values) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def _normalize(values) -> tuple[float, float, float]:
    x, y, z = float(values[0]), float(values[1]), float(values[2])
    length = (x * x + y * y + z * z) ** 0.5
    if length <= 0.0:
        return (0.0, 0.0, -1.0)
    return (x / length, y / length, z / length)


def _polyline_mesh(polylines) -> tuple[pv.PolyData | None, int]:
    """Pack disconnected polylines into one VTK dataset."""
    point_arrays = [
        np.asarray(points, dtype=np.float32)
        for points in polylines
        if len(points) >= 2
    ]
    if not point_arrays:
        return None, 0

    point_count = sum(len(points) for points in point_arrays)
    cells = np.empty(point_count + len(point_arrays), dtype=np.int64)
    point_offset = 0
    cell_offset = 0
    for points in point_arrays:
        count = len(points)
        cells[cell_offset] = count
        cells[cell_offset + 1:cell_offset + count + 1] = np.arange(
            point_offset,
            point_offset + count,
            dtype=np.int64,
        )
        point_offset += count
        cell_offset += count + 1

    mesh = pv.PolyData()
    mesh.points = np.concatenate(point_arrays, axis=0)
    mesh.lines = cells
    return mesh, len(point_arrays)


class ViewportController:
    """Owns disposable PyVista actors and camera actions."""

    def __init__(self, plotter):
        self._plotter = plotter
        self.saved_camera: CameraState | None = None
        self._topology_pick_marker_actors = []

    def render_meshes(self, meshes: list[pv.PolyData], display_mode: str,
                      color_mode: str = COLOR_MODE_BREP_ORIENTATION,
                      reset_camera: bool = False, render: bool = True) -> None:
        """Render current meshes using disposable PyVista actors."""
        import vtk as _vtk

        render_surface = display_mode in DISPLAY_MODES_WITH_SHADED_SURFACE
        render_mesh_wireframe = display_mode in DISPLAY_MODES_WITH_MESH_WIREFRAME
        is_mesh_wireframe_only = display_mode == DISPLAY_MODE_MESH_WIREFRAME
        use_orientation_backface = (
            render_surface and color_mode_uses_orientation_backfaces(color_mode)
        )

        self._plotter.clear()
        self._topology_pick_marker_actors = []
        if render_surface or is_mesh_wireframe_only:
            for poly in meshes:
                color = (0.5, 0.5, 0.5)
                if "DisplayColor" in poly.field_data:
                    dc = poly.field_data["DisplayColor"]
                    color = (float(dc[0]), float(dc[1]), float(dc[2]))
                style = "wireframe" if is_mesh_wireframe_only else "surface"
                actor = self._plotter.add_mesh(
                    poly, color=color,
                    smooth_shading=not is_mesh_wireframe_only,
                    style=style,
                    show_edges=render_mesh_wireframe and not is_mesh_wireframe_only,
                    edge_color="#00000040", line_width=0.5,
                    reset_camera=False,
                    render=False)
                if render_surface:
                    back_color = BACKFACE_COLOR if use_orientation_backface else color
                    back_prop = _vtk.vtkProperty()
                    back_prop.SetColor(*back_color)
                    back_prop.SetAmbient(0.3)
                    back_prop.SetDiffuse(0.7)
                    actor.SetBackfaceProperty(back_prop)
        if reset_camera:
            self.fit_view(render=False)
        if render:
            self._plotter.render()

    def render_inspection_overlays(
        self,
        meshes: list[pv.PolyData],
        objects,
        *,
        normals_mode: str = NORMALS_MODE_OFF,
        show_vertices: bool = False,
        bbox_diag: float = 1.0,
        render: bool = True,
    ) -> None:
        """Render BRep/mesh inspection overlays on top of current mesh actors."""
        if normals_mode == NORMALS_MODE_BREP_FACE:
            self._render_brep_face_normals_overlay(objects, bbox_diag)
        elif normals_mode == NORMALS_MODE_MESH_FACE:
            self._render_mesh_face_normals_overlay(objects, bbox_diag)
        elif normals_mode == NORMALS_MODE_MESH_EDGE:
            self._render_mesh_edge_normals_overlay(objects, bbox_diag)
        elif normals_mode == NORMALS_MODE_MESH_VERTEX:
            self._render_vertex_normals_overlay(meshes, bbox_diag)
        if show_vertices:
            self._render_brep_vertex_overlay(objects)
        if render:
            self._plotter.render()

    def _build_normal_tubes(self, origins, directions, color: str, bbox_diag: float):
        all_origins = np.vstack(origins) if len(origins) > 1 else origins[0]
        all_dirs = np.vstack(directions) if len(directions) > 1 else directions[0]
        length = float(bbox_diag) * 0.04
        radius = length * 0.025
        ends = all_origins + all_dirs * length
        count = len(all_origins)
        if count == 0 or radius <= 0.0:
            return None
        lines_pts = np.vstack([all_origins, ends]).astype(np.float32)
        # Connectivity must be vtkIdType-wide; a narrower dtype is reinterpreted
        # by VTK into garbage cell sizes and aborts on allocation.
        lines = np.empty((count, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(count)
        lines[:, 2] = np.arange(count) + count
        line_mesh = pv.PolyData(lines_pts, lines=lines.ravel())
        tubes = line_mesh.tube(radius=radius, n_sides=8)
        # Keep the overlay opaque: a translucent pass over this much tube
        # geometry aborts inside VTK's depth-peeling path on offscreen GL.
        return self._plotter.add_mesh(
            tubes, color=color, reset_camera=False, render=False)

    def _render_vertex_normals_overlay(self, meshes: list[pv.PolyData], bbox_diag: float) -> None:
        origins, directions = [], []
        for poly in meshes:
            if poly.n_points == 0 or "Normals" not in poly.point_data:
                continue
            origins.append(np.asarray(poly.points, dtype=np.float64))
            directions.append(np.asarray(poly.point_data["Normals"], dtype=np.float64))
        if origins:
            self._build_normal_tubes(origins, directions, "#A3BE8C", bbox_diag)

    def _face_centers_from_flat_faces(self, points, faces) -> np.ndarray:
        centers = []
        points = np.asarray(points, dtype=np.float64)
        flat = np.asarray(faces, dtype=np.int64)
        offset = 0
        while offset < len(flat):
            count = int(flat[offset])
            indices = flat[offset + 1:offset + 1 + count]
            if len(indices) == 0:
                break
            centers.append(points[indices].mean(axis=0))
            offset += count + 1
        if not centers:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(centers)

    def _render_brep_face_normals_overlay(self, objects, bbox_diag: float) -> None:
        origins, directions = [], []
        for obj in objects:
            if obj.kind != "Brep" or not obj.visible:
                continue
            try:
                normals = brep_face_normals(obj)
            except Exception:
                continue
            for normal in normals:
                origins.append(np.asarray(normal.origin, dtype=np.float64))
                directions.append(np.asarray(normal.vector, dtype=np.float64))
        if origins:
            self._build_normal_tubes(
                [np.vstack(origins)],
                [np.vstack(directions)],
                "#FFD166",
                bbox_diag,
            )

    def _render_mesh_face_normals_overlay(self, objects, bbox_diag: float) -> None:
        origins, directions = [], []
        for obj in objects:
            if not obj.visible or not obj.mesh:
                continue
            face_normals = obj.mesh.get("face_normals")
            points = obj.mesh.get("points")
            faces = obj.mesh.get("faces")
            if face_normals is None or points is None or faces is None:
                continue
            face_normals = np.asarray(face_normals, dtype=np.float64).reshape(-1, 3)
            centers = self._face_centers_from_flat_faces(points, faces)
            if len(centers) == 0 or len(face_normals) != len(centers):
                continue
            origins.append(centers)
            directions.append(face_normals)
        if origins:
            self._build_normal_tubes(origins, directions, "#EBCB8B", bbox_diag)

    def _mesh_edge_normals(self, points, faces, face_normals):
        points = np.asarray(points, dtype=np.float64)
        flat = np.asarray(faces, dtype=np.int64)
        face_normals = np.asarray(face_normals, dtype=np.float64).reshape(-1, 3)
        edge_accum: dict[tuple[int, int], list] = {}
        offset = 0
        face_index = 0
        while offset < len(flat) and face_index < len(face_normals):
            count = int(flat[offset])
            indices = [int(flat[offset + 1 + i]) for i in range(count)]
            normal = face_normals[face_index]
            for i in range(count):
                a = indices[i]
                b = indices[(i + 1) % count]
                key = (a, b) if a <= b else (b, a)
                bucket = edge_accum.setdefault(key, [None, None])
                if bucket[0] is None:
                    bucket[0] = 0.5 * (points[a] + points[b])
                    bucket[1] = np.array(normal, dtype=np.float64)
                else:
                    bucket[1] = bucket[1] + normal
            offset += count + 1
            face_index += 1
        if not edge_accum:
            return None, None
        centers = []
        normals = []
        for midpoint, normal_sum in edge_accum.values():
            length = float(np.linalg.norm(normal_sum))
            if length <= 1e-12:
                continue
            centers.append(midpoint)
            normals.append(normal_sum / length)
        if not centers:
            return None, None
        return np.vstack(centers), np.vstack(normals)

    def _render_mesh_edge_normals_overlay(self, objects, bbox_diag: float) -> None:
        origins, directions = [], []
        for obj in objects:
            if not obj.visible or not obj.mesh:
                continue
            face_normals = obj.mesh.get("face_normals")
            points = obj.mesh.get("points")
            faces = obj.mesh.get("faces")
            if face_normals is None or points is None or faces is None:
                continue
            centers, normals = self._mesh_edge_normals(points, faces, face_normals)
            if centers is None:
                continue
            origins.append(centers)
            directions.append(normals)
        if origins:
            self._build_normal_tubes(origins, directions, "#88C0D0", bbox_diag)

    def _render_face_normals_overlay(self, objects, bbox_diag: float) -> None:
        """Backward-compatible alias for mesh face normals."""
        self._render_mesh_face_normals_overlay(objects, bbox_diag)

    def _render_brep_vertex_overlay(self, objects) -> None:
        points = []
        for obj in objects:
            if obj.kind != "Brep" or not obj.visible:
                continue
            try:
                for vertex in obj.handle.vertices():
                    points.append(vertex.point())
            except Exception:
                continue
        if not points:
            return
        self._plotter.add_mesh(
            pv.PolyData(np.asarray(points, dtype=np.float32)),
            color="#EBCB00",
            point_size=8,
            render_points_as_spheres=True,
            reset_camera=False,
            style="points",
            render=False,
        )

    def render_brep_edges(self, objects, shaded: bool = False,
                          render: bool = True) -> int:
        """Render visible BRep edges as disconnected-line batches by color."""
        count = 0
        polylines_by_color = {}
        for obj in objects:
            if obj.kind != "Brep" or not obj.visible:
                continue
            color = "#101318" if shaded else obj.color
            polylines_by_color.setdefault(color, []).extend(
                brep_edge_polylines(obj)
            )
        for color, polylines in polylines_by_color.items():
            edge_mesh, edge_count = _polyline_mesh(polylines)
            if edge_mesh is None:
                continue
            self._plotter.add_mesh(
                edge_mesh,
                color=color,
                line_width=2,
                lighting=False,
                reset_camera=False,
                render=False,
            )
            count += edge_count
        if render:
            self._plotter.render()
        return count

    def render_overlays(self, objects, render: bool = True) -> None:
        """Render active debug overlays after mesh actors are in place."""
        for obj in objects:
            if not is_overlay(obj):
                continue
            color = obj.color
            handle = obj.handle
            if isinstance(handle, PointOverlay):
                self._plotter.add_points(
                    np.array([handle.point], dtype=np.float32),
                    color=color,
                    point_size=12,
                    render_points_as_spheres=True,
                    render=False,
                )
            elif isinstance(handle, LineOverlay):
                self._add_polyline(
                    [handle.start, handle.end],
                    color,
                    width=DEBUG_OVERLAY_LINE_WIDTH,
                )
            elif isinstance(handle, PolylineOverlay):
                self._add_polyline(handle.points, color, width=DEBUG_OVERLAY_LINE_WIDTH)
            elif isinstance(handle, CurveSampleOverlay):
                self._add_polyline(handle.points, color, width=DEBUG_OVERLAY_LINE_WIDTH)
            elif isinstance(handle, SurfaceSampleOverlay):
                self._add_polylines(
                    handle.polylines,
                    color,
                    width=DEBUG_OVERLAY_SURFACE_LINE_WIDTH,
                )
            elif isinstance(handle, BoxOverlay):
                mn, mx = handle.minimum, handle.maximum
                box = pv.Box(bounds=(mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))
                self._plotter.add_mesh(
                    box,
                    color=color,
                    style="wireframe",
                    line_width=DEBUG_OVERLAY_MESH_EDGE_WIDTH,
                    render=False,
                )
            elif isinstance(handle, NormalOverlay):
                self._add_normal_marker(handle, color, width=DEBUG_OVERLAY_LINE_WIDTH, point_size=9)
            elif isinstance(handle, PickOverlay):
                self._plotter.add_points(
                    np.array([handle.point], dtype=np.float32),
                    color=color,
                    point_size=14,
                    render_points_as_spheres=True,
                    render=False,
                )
                if handle.normal is not None:
                    self._add_normal_marker(
                        NormalOverlay(handle.point, handle.normal, 1.5),
                        color,
                        width=DEBUG_OVERLAY_LINE_WIDTH,
                        point_size=9,
                    )
            elif isinstance(handle, TopologyOverlay):
                width = DEBUG_TOPOLOGY_LINE_WIDTH if handle.topology_kind in {
                    "face",
                    "edge",
                    "edgeuse",
                    "loopuse",
                } else DEBUG_TOPOLOGY_SECONDARY_LINE_WIDTH
                self._add_polylines(handle.polylines, color, width=width)
                if handle.points:
                    self._plotter.add_points(
                        np.array(handle.points, dtype=np.float32),
                        color=color,
                        point_size=16,
                        render_points_as_spheres=True,
                        render=False,
                    )
            elif isinstance(handle, DrawBatchOverlay):
                self._add_draw_batches(handle.batches, color)
            elif isinstance(handle, SectionOverlay):
                self._add_polylines(
                    ([start, end] for start, end in handle.segments),
                    color,
                    width=DEBUG_OVERLAY_LINE_WIDTH,
                )
        if render:
            self._plotter.render()

    def render_face_normals(self, objects) -> int:
        """Render transient midpoint normal markers for visible BRep faces."""
        count = 0
        for obj in objects:
            if obj.kind != "Brep" or not obj.visible:
                continue
            try:
                normals = brep_face_normals(obj)
            except Exception:
                continue
            segments = []
            tips = []
            for normal in normals:
                start = normal.origin
                end = (
                    normal.origin[0] + normal.vector[0] * normal.scale,
                    normal.origin[1] + normal.vector[1] * normal.scale,
                    normal.origin[2] + normal.vector[2] * normal.scale,
                )
                segments.append([start, end])
                tips.append(end)
            if not segments:
                continue
            self._add_polylines(segments, "#ffd166", width=2)
            self._plotter.add_points(
                np.array(tips, dtype=np.float32),
                color="#ffd166",
                point_size=7,
                render_points_as_spheres=True,
                render=False,
            )
            count += len(tips)
        self._plotter.render()
        return count

    def clear_topology_pick_marker(self, render: bool = True) -> None:
        """Remove the transient marker for the current topology pick."""
        for actor in self._topology_pick_marker_actors:
            try:
                self._plotter.remove_actor(actor, render=False)
            except Exception:
                pass
        self._topology_pick_marker_actors = []
        if render:
            self._plotter.render()

    def render_topology_pick_marker(self, point, render: bool = True) -> None:
        """Render a transient point marker at the current topology pick."""
        self.clear_topology_pick_marker(render=False)
        actor = self._plotter.add_points(
            np.array([_tuple3(point)], dtype=np.float32),
            color="#ff3b30",
            point_size=18,
            render_points_as_spheres=True,
            render=False,
        )
        self._topology_pick_marker_actors.append(actor)
        if render:
            self._plotter.render()

    def _add_normal_marker(self, normal: NormalOverlay, color, width: int,
                           point_size: int) -> None:
        start = normal.origin
        end = (
            normal.origin[0] + normal.vector[0] * normal.scale,
            normal.origin[1] + normal.vector[1] * normal.scale,
            normal.origin[2] + normal.vector[2] * normal.scale,
        )
        self._add_polyline([start, end], color, width=width)
        self._plotter.add_points(
            np.array([end], dtype=np.float32),
            color=color,
            point_size=point_size,
            render_points_as_spheres=True,
            render=False,
        )

    def _add_polyline(self, points, color, width: int) -> None:
        self._add_polylines([points], color, width)

    def _add_polylines(self, polylines, color, width: int) -> int:
        mesh, count = _polyline_mesh(polylines)
        if mesh is None:
            return 0
        self._plotter.add_mesh(
            mesh,
            color=color,
            line_width=width,
            lighting=False,
            reset_camera=False,
            render=False,
        )
        return count

    @staticmethod
    def _line_batch_polylines(positions, connected: bool):
        if connected:
            return [positions]
        return [
            positions[index:index + 2]
            for index in range(0, len(positions) - 1, 2)
        ]

    @staticmethod
    def _batch_color(batch, fallback_color):
        color = batch.get("color")
        if isinstance(color, str):
            return color
        try:
            if color is not None and len(color) >= 3:
                rgb = (float(color[0]), float(color[1]), float(color[2]))
                if any(component != 0.0 for component in rgb):
                    return rgb
        except (TypeError, ValueError):
            pass
        return fallback_color

    def _add_draw_batches(self, batches, fallback_color) -> None:
        for batch in batches:
            primitive = batch.get("primitive")
            positions = batch.get("positions") or []
            if not positions:
                continue
            color = self._batch_color(batch, fallback_color)
            if primitive == "point":
                point_size = max(float(batch.get("point_size", 10.0)), 1.0)
                self._plotter.add_points(
                    np.array(positions, dtype=np.float32),
                    color=color,
                    point_size=point_size,
                    render_points_as_spheres=True,
                    render=False,
                )
            elif primitive == "line":
                width = max(float(batch.get("line_width", 2.0)), DEBUG_OVERLAY_LINE_WIDTH)
                self._add_polylines(
                    self._line_batch_polylines(
                        positions,
                        bool(batch.get("connected", True)),
                    ),
                    color,
                    width=width,
                )
            elif primitive in {"simple_mesh", "index_mesh"}:
                mesh = self._draw_batch_mesh(batch, positions)
                if mesh is not None:
                    self._plotter.add_mesh(
                        mesh,
                        color=color,
                        opacity=0.35,
                        smooth_shading=False,
                        show_edges=True,
                        edge_color=color,
                        line_width=max(
                            float(batch.get("line_width", 1.0)),
                            DEBUG_OVERLAY_MESH_EDGE_WIDTH,
                        ),
                        render=False,
                    )

    def _draw_batch_mesh(self, batch, positions):
        points = np.array(positions, dtype=np.float32)
        if len(points) < 3:
            return None
        indices = batch.get("indices") or []
        if batch.get("primitive") == "index_mesh" and len(indices) >= 3:
            triangles = [
                [3, int(indices[i]), int(indices[i + 1]), int(indices[i + 2])]
                for i in range(0, len(indices) - 2, 3)
            ]
        else:
            triangles = [
                [3, i, i + 1, i + 2]
                for i in range(0, len(points) - 2, 3)
            ]
        if not triangles:
            return None
        return pv.PolyData(points, np.array(triangles, dtype=np.int64).ravel())

    def fit_view(self, render: bool = True) -> None:
        self._plotter.reset_camera()
        if render:
            self._plotter.render()

    def screen_to_world_ray(self, x: float, y: float) -> tuple[tuple[float, float, float],
                                                               tuple[float, float, float]]:
        """Convert Qt widget coordinates to a world-space ray."""
        renderer = getattr(self._plotter, "renderer", None)
        if renderer is None:
            raise RuntimeError("Viewport renderer is not available")
        render_window = getattr(self._plotter, "ren_win", None)
        if render_window is None:
            render_window = getattr(self._plotter, "render_window", None)
        if render_window is None:
            raise RuntimeError("Viewport render window is not available")
        width, height = render_window.GetSize()
        display_x = float(max(0.0, min(float(width - 1), float(x))))
        display_y = float(max(0.0, min(float(height - 1), float(height) - float(y) - 1.0)))

        def _display_to_world(z_value: float) -> tuple[float, float, float]:
            renderer.SetDisplayPoint(display_x, display_y, z_value)
            renderer.DisplayToWorld()
            world = renderer.GetWorldPoint()
            w = float(world[3]) if len(world) > 3 else 1.0
            if abs(w) <= 1.0e-12:
                w = 1.0
            return (float(world[0]) / w, float(world[1]) / w, float(world[2]) / w)

        near = _display_to_world(0.0)
        far = _display_to_world(1.0)
        direction = _normalize((far[0] - near[0], far[1] - near[1], far[2] - near[2]))
        return near, direction

    def set_named_view(self, view_name: str, up_axis: str = UP_AXIS_Z) -> None:
        if view_name not in NAMED_VIEW_NAMES:
            raise ValueError(f"Unknown view: {view_name}")

        center, distance = _scene_center_and_distance(self._plotter)
        offset = _camera_offset(view_name, distance)
        if offset is None:
            raise ValueError(f"Unknown view: {view_name}")

        ox, oy, oz = offset
        cx, cy, cz = center
        cam = self._plotter.camera
        cam.SetFocalPoint(cx, cy, cz)
        cam.SetPosition(cx + ox, cy + oy, cz + oz)
        cam.SetViewUp(*_view_up_vector(up_axis, offset))
        # ResetCamera keeps the pose just set and refits distance / parallel
        # scale to the scene, so elongated models zoom correctly per view.
        self._plotter.reset_camera()
        self._plotter.render()

    def set_parallel_projection(self, enabled: bool) -> None:
        camera = self._plotter.camera
        if hasattr(camera, "SetParallelProjection"):
            camera.SetParallelProjection(bool(enabled))
        else:
            camera.parallel_projection = bool(enabled)
        self._plotter.render()

    def is_parallel_projection(self) -> bool:
        camera = self._plotter.camera
        if hasattr(camera, "GetParallelProjection"):
            return bool(camera.GetParallelProjection())
        return bool(getattr(camera, "parallel_projection", False))

    def camera_state(self) -> CameraState:
        camera = self._plotter.camera
        return CameraState(
            position=_tuple3(camera.GetPosition()),
            focal_point=_tuple3(camera.GetFocalPoint()),
            view_up=_tuple3(camera.GetViewUp()),
            parallel_projection=self.is_parallel_projection(),
        )

    def apply_camera_state(self, state: CameraState) -> None:
        camera = self._plotter.camera
        camera.SetPosition(*state.position)
        camera.SetFocalPoint(*state.focal_point)
        camera.SetViewUp(*state.view_up)
        if hasattr(camera, "SetParallelProjection"):
            camera.SetParallelProjection(state.parallel_projection)
        else:
            camera.parallel_projection = state.parallel_projection
        self._plotter.reset_camera_clipping_range()
        self._plotter.render()

    def save_camera(self) -> CameraState:
        self.saved_camera = self.camera_state()
        return self.saved_camera

    def restore_saved_camera(self) -> bool:
        if self.saved_camera is None:
            return False
        self.apply_camera_state(self.saved_camera)
        return True
