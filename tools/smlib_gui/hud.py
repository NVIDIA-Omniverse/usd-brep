# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""In-viewport HUD overlays for the SMLib GUI workbench."""

from __future__ import annotations

from dataclasses import dataclass, field

from .settings import (
    COLOR_MODE_BREP_ORIENTATION,
    COLOR_MODE_MATERIAL,
    COLOR_MODE_VTK_BACKFACES,
    DISPLAY_MODE_OPTIONS,
    DISPLAY_MODE_SHADED_WIREFRAME,
    NORMALS_MODE_OFF,
    TOPOLOGY_COUNTS_BREP,
    TessellationSettings,
)
from .topology_stats import topology_counts_label


_COLOR_MODE_LABELS = {
    COLOR_MODE_BREP_ORIENTATION: "BRep orient",
    COLOR_MODE_VTK_BACKFACES: "VTK backfaces",
    COLOR_MODE_MATERIAL: "Material",
}

_NORMALS_MODE_LABELS = {
    "off": "Off",
    "brep-face": "BRep face",
    "mesh-face": "Mesh face",
    "mesh-edge": "Mesh edge",
    "mesh-vertex": "Mesh vertex",
}


@dataclass
class HudStats:
    """Counts shown in the viewport HUD."""

    rows: list[tuple[str, int | None]] = field(default_factory=list)
    topology_counts_mode: str = TOPOLOGY_COUNTS_BREP


@dataclass
class ViewportHud:
    """Manage left/right PyVista text overlays on a plotter."""

    plotter: object
    left_actor: object | None = None
    left_green_actor: object | None = None
    right_actor: object | None = None

    def clear(self) -> None:
        for attr in ("left_actor", "left_green_actor", "right_actor"):
            actor = getattr(self, attr)
            if actor is not None:
                try:
                    self.plotter.remove_actor(actor, render=False)
                except Exception:
                    pass
                setattr(self, attr, None)

    def update(
        self,
        *,
        display_mode: str,
        color_mode: str,
        tessellation: TessellationSettings,
        stats: HudStats,
        defaults: TessellationSettings | None = None,
        normals_mode: str = NORMALS_MODE_OFF,
        show_vertices: bool = False,
        topology_counts_mode: str = TOPOLOGY_COUNTS_BREP,
    ) -> None:
        defaults = defaults or TessellationSettings()
        self.clear()

        display_mode_labels = {mode: label for label, mode in DISPLAY_MODE_OPTIONS}
        mode_label = display_mode_labels.get(display_mode, display_mode)
        color_label = _COLOR_MODE_LABELS.get(color_mode, color_mode)
        normals_label = _NORMALS_MODE_LABELS.get(normals_mode, normals_mode)
        counts_label = topology_counts_label(topology_counts_mode)
        pad = "  "
        label_width = 14
        value_width = 16

        def _fmt(value: float) -> str:
            if value <= 0:
                return "off"
            if value >= 1.0:
                return f"{value:.1f}"
            return f"{value:.4g}"

        left_entries: list[tuple[str, bool]] = [
            ("", False),
            (f"{pad}[SMLib GUI]", False),
            ("", False),
            (f"{pad}{'Display:':<{label_width}}{mode_label:<{value_width}} [W]",
             display_mode != DISPLAY_MODE_SHADED_WIREFRAME),
            (f"{pad}{'Color:':<{label_width}}{color_label:<{value_width}} [M]",
             color_mode != COLOR_MODE_BREP_ORIENTATION),
            (f"{pad}{'Vertices:':<{label_width}}{'on' if show_vertices else 'off':<{value_width}} [E]",
             show_vertices),
            (f"{pad}{'Normals:':<{label_width}}{normals_label:<{value_width}} [N]",
             normals_mode != NORMALS_MODE_OFF),
            (f"{pad}{'Counts:':<{label_width}}{counts_label:<{value_width}} [T]",
             topology_counts_mode != TOPOLOGY_COUNTS_BREP),
            ("", False),
            (f"{pad}{'Angle:':<{label_width}}{tessellation.angle_tolerance_deg:<5.1f}\u00b0{'':<10s} [+/-]",
             tessellation.angle_tolerance_deg != defaults.angle_tolerance_deg),
            (f"{pad}{'Chord tol:':<{label_width}}{_fmt(tessellation.chord_height_tolerance):<{value_width}} [C]",
             tessellation.chord_height_tolerance != defaults.chord_height_tolerance),
            (f"{pad}{'Max edge:':<{label_width}}{_fmt(tessellation.max_edge_length):<{value_width}} [L]",
             tessellation.max_edge_length != defaults.max_edge_length),
            (f"{pad}{'Aspect max:':<{label_width}}{_fmt(tessellation.max_aspect_ratio):<{value_width}} [R]",
             tessellation.max_aspect_ratio != defaults.max_aspect_ratio),
            ("", False),
            (f"{pad}{'Views 1-7,I  Fit [F]  Fly [Z]'}", False),
        ]

        default_lines: list[str] = []
        green_lines: list[str] = []
        has_green = False
        for text, changed in left_entries:
            if text == "":
                default_lines.append("")
                green_lines.append("")
            elif changed:
                has_green = True
                default_lines.append(" ")
                green_lines.append(text)
            else:
                default_lines.append(text)
                green_lines.append(" ")

        stat_label_width = 12
        stat_value_width = 8
        right_lines = [""]
        for label, value in stats.rows:
            if value is None:
                rendered = f"{'—':>{stat_value_width}}"
            else:
                rendered = f"{value:>{stat_value_width}d}"
            right_lines.append(f"{pad}{label + ':':<{stat_label_width}} {rendered}")

        self.left_actor = self.plotter.add_text(
            "\n".join(default_lines),
            position="upper_left",
            font_size=9,
            color="#D8DEE9",
            font="courier",
            name="smlib_hud_left",
            render=False,
        )
        if has_green:
            self.left_green_actor = self.plotter.add_text(
                "\n".join(green_lines),
                position="upper_left",
                font_size=9,
                color="#50FA7B",
                font="courier",
                name="smlib_hud_left_green",
                render=False,
            )
        self.right_actor = self.plotter.add_text(
            "\n".join(right_lines),
            position="upper_right",
            font_size=9,
            color="#D8DEE9",
            font="courier",
            name="smlib_hud_right",
            render=False,
        )


__all__ = ["HudStats", "ViewportHud"]
