# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Reusable offscreen loading and rendering for SMLib GUI model files."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Mapping

from .model import ActiveObject
from .operations.file_io import SUPPORTED_LOAD_EXTENSIONS, import_file_objects
from .runtime import QtGui, QtWidgets, pv, sm
from .settings import (
    COLOR_MODE_BREP_ORIENTATION,
    DISPLAY_MODE_SHADED_WIREFRAME,
    DISPLAY_MODES_WITH_BREP_WIREFRAME,
    DISPLAY_MODES_WITH_SHADED_SURFACE,
    NAMED_VIEW_NAMES,
    UP_AXIS_Y,
    UP_AXIS_Z,
    TessellationSettings,
    color_mode_orients_to_brep_normals,
)
from .tessellation import mesh_error_summary, objects_to_pyvista_batches, tessellate_objects
from .viewport import ViewportController


DEFAULT_VIEW_NAMES = ("isometric", "front", "top", "right")
SUPPORTED_VIEW_NAMES = frozenset(NAMED_VIEW_NAMES)
SUPPORTED_UP_AXES = frozenset({UP_AXIS_Z, UP_AXIS_Y})


@dataclass(frozen=True)
class HeadlessRenderSettings:
    """Tessellation, viewport, and image settings for offscreen renders."""

    chord_height_tolerance: float = 0.05
    angle_tolerance_deg: float = 15.0
    max_edge_length: float = 0.0
    max_aspect_ratio: float = 0.0
    view_names: tuple[str, ...] = DEFAULT_VIEW_NAMES
    window_size: tuple[int, int] = (900, 900)
    display_mode: str = DISPLAY_MODE_SHADED_WIREFRAME
    parallel_projection: bool = False
    color_mode: str = COLOR_MODE_BREP_ORIENTATION
    up_axis: str = UP_AXIS_Z
    show_brep_edges: bool = True
    background_color: str = "#1e2030"
    background_top_color: str = "#2a2d3e"

    def __post_init__(self) -> None:
        view_names = tuple(self.view_names)
        object.__setattr__(self, "view_names", view_names)
        if not view_names:
            raise ValueError("At least one render view is required")
        unknown = sorted(set(view_names) - SUPPORTED_VIEW_NAMES)
        if unknown:
            raise ValueError(f"Unsupported render view(s): {', '.join(unknown)}")
        if self.up_axis not in SUPPORTED_UP_AXES:
            supported = ", ".join(sorted(SUPPORTED_UP_AXES))
            raise ValueError(
                f"Unsupported up axis: {self.up_axis!r}; expected one of: {supported}"
            )
        width, height = self.window_size
        if width <= 0 or height <= 0:
            raise ValueError("Render width and height must be positive")

    @property
    def tessellation(self) -> TessellationSettings:
        return TessellationSettings(
            chord_height_tolerance=self.chord_height_tolerance,
            angle_tolerance_deg=self.angle_tolerance_deg,
            max_edge_length=self.max_edge_length,
            max_aspect_ratio=self.max_aspect_ratio,
        )


@dataclass
class HeadlessRenderResult:
    """Artifacts and in-process objects produced for one source file."""

    source: Path
    objects: list[ActiveObject]
    view_paths: dict[str, Path]
    contact_sheet: Path
    screenshots: dict[str, object] = field(repr=False)


def discover_model_files(input_path: Path) -> list[Path]:
    """Return supported model files below a file or directory, deterministically."""
    input_path = input_path.expanduser().resolve()
    if input_path.is_file():
        if input_path.suffix.lower() not in SUPPORTED_LOAD_EXTENSIONS:
            supported = ", ".join(sorted(SUPPORTED_LOAD_EXTENSIONS))
            raise RuntimeError(
                f"Unsupported input file {input_path}; expected one of: {supported}"
            )
        return [input_path]
    if not input_path.is_dir():
        raise RuntimeError(f"Input path does not exist or is not a directory: {input_path}")

    # Keyed by resolved path so a symlink and its target yield one render, not
    # two workers writing the same output files.
    sort_keys: dict[Path, str] = {}
    for path in input_path.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in SUPPORTED_LOAD_EXTENSIONS:
            continue
        resolved = path.resolve()
        if resolved in sort_keys:
            continue
        try:
            sort_keys[resolved] = resolved.relative_to(input_path).as_posix().lower()
        except ValueError:
            # A symlink can resolve outside the search root.
            sort_keys[resolved] = resolved.as_posix().lower()
    return sorted(sort_keys, key=lambda path: sort_keys[path])


def output_directory_for(source: Path, input_path: Path,
                         output_root: Path | None) -> Path:
    """Map a source to an adjacent or hierarchy-preserving output directory."""
    source = source.expanduser().resolve()
    input_path = input_path.expanduser().resolve()
    if output_root is None:
        return source.parent

    output_root = output_root.expanduser().resolve()
    if input_path.is_file():
        return output_root
    return output_root / source.parent.relative_to(input_path)


def write_contact_sheet(view_paths: Mapping[str, Path], output_path: Path,
                        background_color: str = "#1e2030") -> Path:
    """Compose named view images into a compact contact sheet."""
    if not view_paths:
        raise RuntimeError("Cannot create a contact sheet without view images")

    images = []
    for view_name, path in view_paths.items():
        image = QtGui.QImage(str(path))
        if image.isNull():
            raise RuntimeError(f"Could not read rendered view: {path}")
        images.append((view_name, image))

    tile_width = max(image.width() for _name, image in images)
    tile_height = max(image.height() for _name, image in images)
    columns = min(2, len(images))
    rows = math.ceil(len(images) / columns)
    contact = QtGui.QImage(
        columns * tile_width,
        rows * tile_height,
        QtGui.QImage.Format_RGB32,
    )
    contact.fill(QtGui.QColor(background_color))
    painter = QtGui.QPainter(contact)
    painter.setPen(QtGui.QColor("white"))
    font = painter.font()
    font.setPointSize(14)
    painter.setFont(font)
    metrics = painter.fontMetrics()
    for index, (view_name, image) in enumerate(images):
        x = (index % columns) * tile_width
        y = (index // columns) * tile_height
        label = view_name.replace("_", " ").title()
        label_width = metrics.horizontalAdvance(label) + 24
        painter.drawImage(x, y, image)
        painter.fillRect(
            x + 8,
            y + 8,
            label_width,
            metrics.height() + 12,
            QtGui.QColor(20, 22, 30, 190),
        )
        painter.drawText(x + 16, y + metrics.ascent() + 14, label)
    painter.end()

    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not contact.save(str(output_path)):
        raise RuntimeError(f"Could not save contact sheet: {output_path}")
    return output_path


class HeadlessRenderSession:
    """Own one offscreen PyVista viewport and render one or more model files."""

    def __init__(self, settings: HeadlessRenderSettings | None = None):
        if sm is None:
            raise RuntimeError(
                "_omni_solid did not import; build the repo with a Python 3.12 environment"
            )
        self.settings = settings or HeadlessRenderSettings()
        self._closed = False

        try:
            import vtk  # noqa: PLC0415

            vtk.vtkObject.GlobalWarningDisplayOff()
        except ImportError:
            pass

        pv.global_theme.allow_empty_mesh = True
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication([])
        self.plotter = pv.Plotter(
            off_screen=True,
            window_size=self.settings.window_size,
        )
        self.plotter.set_background(
            self.settings.background_color,
            top=self.settings.background_top_color,
        )
        self.viewport = ViewportController(self.plotter)

    def __enter__(self) -> "HeadlessRenderSession":
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        self.plotter.close()
        self._closed = True

    def load_file(self, source: Path) -> list[ActiveObject]:
        """Load and tessellate one supported source file."""
        source = source.expanduser().resolve()
        objects = import_file_objects(str(source))
        tessellate_objects(objects, settings=self.settings.tessellation)
        return objects

    def render_file(self, source: Path, output_dir: Path,
                    output_prefix: str | None = None) -> HeadlessRenderResult:
        """Load, tessellate, and render one source file."""
        source = source.expanduser().resolve()
        objects = self.load_file(source)
        return self.render_objects(
            source,
            objects,
            output_dir,
            output_prefix=output_prefix,
        )

    def render_objects(self, source: Path, objects: list[ActiveObject],
                       output_dir: Path,
                       output_prefix: str | None = None) -> HeadlessRenderResult:
        """Render already tessellated GUI objects into named views and a contact sheet."""
        if self._closed:
            raise RuntimeError("Headless render session is closed")
        source = source.expanduser().resolve()
        output_dir = output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        prefix = Path(output_prefix or source.name).name

        meshes = objects_to_pyvista_batches(
            [obj for obj in objects if obj.visible],
            orient_to_face_normals=color_mode_orients_to_brep_normals(
                self.settings.color_mode
            ),
        )
        if not meshes:
            reasons = mesh_error_summary(objects)
            message = f"Tessellation produced no renderable meshes for {source}"
            raise RuntimeError(f"{message}: {reasons}" if reasons else message)

        self.viewport.render_meshes(
            meshes,
            self.settings.display_mode,
            color_mode=self.settings.color_mode,
            reset_camera=True,
            render=False,
        )
        if (
            self.settings.show_brep_edges
            and self.settings.display_mode in DISPLAY_MODES_WITH_BREP_WIREFRAME
        ):
            self.viewport.render_brep_edges(
                objects,
                shaded=self.settings.display_mode in DISPLAY_MODES_WITH_SHADED_SURFACE,
                render=False,
            )
        self.viewport.set_parallel_projection(self.settings.parallel_projection)

        view_paths: dict[str, Path] = {}
        screenshots: dict[str, object] = {}
        for view_name in self.settings.view_names:
            self.viewport.set_named_view(view_name, up_axis=self.settings.up_axis)
            view_path = output_dir / f"{prefix}_{view_name}.png"
            screenshots[view_name] = self.plotter.screenshot(
                str(view_path),
                return_img=True,
            )
            view_paths[view_name] = view_path

        contact_sheet = write_contact_sheet(
            view_paths,
            output_dir / f"{prefix}_contact.png",
            background_color=self.settings.background_color,
        )
        return HeadlessRenderResult(
            source=source,
            objects=objects,
            view_paths=view_paths,
            contact_sheet=contact_sheet,
            screenshots=screenshots,
        )


__all__ = [
    "DEFAULT_VIEW_NAMES",
    "HeadlessRenderResult",
    "HeadlessRenderSession",
    "HeadlessRenderSettings",
    "SUPPORTED_LOAD_EXTENSIONS",
    "SUPPORTED_VIEW_NAMES",
    "discover_model_files",
    "output_directory_for",
    "write_contact_sheet",
]
