# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Workbench settings for the SMLib GUI."""

from __future__ import annotations

from dataclasses import dataclass


DISPLAY_MODE_SHADED_WIREFRAME = "shaded-wireframe"
DISPLAY_MODE_SHADED_MESH_WIREFRAME = "shaded-mesh-wireframe"
DISPLAY_MODE_SHADED = "shaded"
DISPLAY_MODE_WIREFRAME = "wireframe"
DISPLAY_MODE_MESH_WIREFRAME = "mesh-wireframe"

DISPLAY_MODE_OPTIONS = [
    ("Shaded + wireframe", DISPLAY_MODE_SHADED_WIREFRAME),
    ("Shaded + mesh-wireframe", DISPLAY_MODE_SHADED_MESH_WIREFRAME),
    ("Shaded", DISPLAY_MODE_SHADED),
    ("wireframe", DISPLAY_MODE_WIREFRAME),
    ("mesh-wireframe", DISPLAY_MODE_MESH_WIREFRAME),
]

DISPLAY_MODES_WITH_SHADED_SURFACE = {
    DISPLAY_MODE_SHADED_WIREFRAME,
    DISPLAY_MODE_SHADED_MESH_WIREFRAME,
    DISPLAY_MODE_SHADED,
}
DISPLAY_MODES_WITH_MESH_WIREFRAME = {
    DISPLAY_MODE_SHADED_MESH_WIREFRAME,
    DISPLAY_MODE_MESH_WIREFRAME,
}
DISPLAY_MODES_WITH_BREP_WIREFRAME = {
    DISPLAY_MODE_SHADED_WIREFRAME,
    DISPLAY_MODE_WIREFRAME,
}

COLOR_MODE_BREP_ORIENTATION = "brep-orientation"
COLOR_MODE_VTK_BACKFACES = "vtk-backfaces"
COLOR_MODE_MATERIAL = "material"
# Legacy alias used by older callers / smoke migrations.
COLOR_MODE_FRONT_BACK = COLOR_MODE_VTK_BACKFACES

COLOR_MODE_OPTIONS = [
    ("BRep orientation", COLOR_MODE_BREP_ORIENTATION),
    ("VTK backfaces", COLOR_MODE_VTK_BACKFACES),
    ("Material", COLOR_MODE_MATERIAL),
]

COLOR_MODE_CYCLE = (
    COLOR_MODE_BREP_ORIENTATION,
    COLOR_MODE_VTK_BACKFACES,
    COLOR_MODE_MATERIAL,
)

COLOR_MODES_WITH_ORIENTATION_BACKFACES = frozenset({
    COLOR_MODE_BREP_ORIENTATION,
    COLOR_MODE_VTK_BACKFACES,
})


def color_mode_orients_to_brep_normals(color_mode: str) -> bool:
    """True when mesh windings should be aligned to tessellation face_normals."""
    return color_mode == COLOR_MODE_BREP_ORIENTATION


def color_mode_uses_orientation_backfaces(color_mode: str) -> bool:
    """True when VTK backfaces should render in the orientation warning color."""
    return color_mode in COLOR_MODES_WITH_ORIENTATION_BACKFACES


UP_AXIS_Z = "z"
UP_AXIS_Y = "y"

UP_AXIS_OPTIONS = [
    ("Z up", UP_AXIS_Z),
    ("Y up", UP_AXIS_Y),
]

NAMED_VIEW_NAMES = (
    "front",
    "back",
    "left",
    "right",
    "top",
    "bottom",
    "isometric",
    "front-iso",
    "front-iso-left",
)

NORMALS_MODE_OFF = "off"
NORMALS_MODE_BREP_FACE = "brep-face"
NORMALS_MODE_MESH_FACE = "mesh-face"
NORMALS_MODE_MESH_EDGE = "mesh-edge"
NORMALS_MODE_MESH_VERTEX = "mesh-vertex"
# Legacy aliases kept for older callers / smoke migrations.
NORMALS_MODE_VERTEX = NORMALS_MODE_MESH_VERTEX
NORMALS_MODE_FACE = NORMALS_MODE_MESH_FACE

NORMALS_MODE_OPTIONS = [
    ("Off", NORMALS_MODE_OFF),
    ("BRep face", NORMALS_MODE_BREP_FACE),
    ("Mesh face", NORMALS_MODE_MESH_FACE),
    ("Mesh edge", NORMALS_MODE_MESH_EDGE),
    ("Mesh vertex", NORMALS_MODE_MESH_VERTEX),
]

NORMALS_MODE_CYCLE = (
    NORMALS_MODE_OFF,
    NORMALS_MODE_BREP_FACE,
    NORMALS_MODE_MESH_FACE,
    NORMALS_MODE_MESH_EDGE,
    NORMALS_MODE_MESH_VERTEX,
)

TOPOLOGY_COUNTS_BREP = "brep"
TOPOLOGY_COUNTS_BREP_DETAIL = "brep-detail"
TOPOLOGY_COUNTS_MESH = "mesh"

TOPOLOGY_COUNTS_OPTIONS = [
    ("BRep F/E/V", TOPOLOGY_COUNTS_BREP),
    ("BRep detail", TOPOLOGY_COUNTS_BREP_DETAIL),
    ("Mesh F/E/V", TOPOLOGY_COUNTS_MESH),
]

TOPOLOGY_COUNTS_CYCLE = (
    TOPOLOGY_COUNTS_BREP,
    TOPOLOGY_COUNTS_BREP_DETAIL,
    TOPOLOGY_COUNTS_MESH,
)

CHORD_TOLERANCE_FRACS = (0.0, 0.01, 0.001, 0.0001)
MAX_EDGE_LENGTH_FRACS = (0.0, 0.5, 0.2, 0.1, 0.05)
MAX_ASPECT_RATIO_PRESETS = (0.0, 10.0, 5.0, 2.0)


@dataclass
class TessellationSettings:
    """Tessellation controls passed to `_omni_solid.tessellate`."""

    chord_height_tolerance: float = 0.05
    angle_tolerance_deg: float = 25.0
    max_edge_length: float = 0.0
    max_aspect_ratio: float = 0.0


@dataclass
class DisplaySettings:
    """Viewport display settings that are independent from model state."""

    display_mode: str = DISPLAY_MODE_SHADED_WIREFRAME
    parallel_projection: bool = False
    color_mode: str = COLOR_MODE_BREP_ORIENTATION
    up_axis: str = UP_AXIS_Z
    normals_mode: str = NORMALS_MODE_OFF
    show_vertices: bool = False
    topology_counts_mode: str = TOPOLOGY_COUNTS_BREP

    @property
    def show_edge_vertices(self) -> bool:
        """Backward-compatible alias for ``show_vertices``."""
        return self.show_vertices

    @show_edge_vertices.setter
    def show_edge_vertices(self, value: bool) -> None:
        self.show_vertices = bool(value)
