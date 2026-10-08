# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""BBox-relative tessellation preset cycling for the SMLib GUI."""

from __future__ import annotations

from .settings import (
    CHORD_TOLERANCE_FRACS,
    MAX_ASPECT_RATIO_PRESETS,
    MAX_EDGE_LENGTH_FRACS,
)


def bbox_diagonal_from_meshes(meshes) -> float:
    """Return the axis-aligned bounding-box diagonal for rendered meshes."""
    import numpy as np

    if not meshes:
        return 1.0
    point_arrays = [mesh.points for mesh in meshes if mesh.n_points > 0]
    if not point_arrays:
        return 1.0
    points = np.vstack(point_arrays)
    diag = float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))
    return diag if diag > 0.0 else 1.0


def cycle_index(current_index: int, table_size: int) -> int:
    return (current_index + 1) % table_size


def chord_tolerance_for_index(index: int, bbox_diag: float) -> float:
    return CHORD_TOLERANCE_FRACS[index % len(CHORD_TOLERANCE_FRACS)] * bbox_diag


def max_edge_length_for_index(index: int, bbox_diag: float) -> float:
    return MAX_EDGE_LENGTH_FRACS[index % len(MAX_EDGE_LENGTH_FRACS)] * bbox_diag


def max_aspect_ratio_for_index(index: int) -> float:
    return MAX_ASPECT_RATIO_PRESETS[index % len(MAX_ASPECT_RATIO_PRESETS)]


__all__ = [
    "bbox_diagonal_from_meshes",
    "chord_tolerance_for_index",
    "cycle_index",
    "max_aspect_ratio_for_index",
    "max_edge_length_for_index",
]
