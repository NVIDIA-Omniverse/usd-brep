#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compose the bounded OCP-style round-corner pipe reference with public APIs.

This example intentionally adds no new SMLib operation. It reproduces the
closed, orthogonal, circular-pipe reference from the CAD SDK handoff by
combining the existing planar-path sweep, primitive, linear-sweep, transform,
Boolean, and query APIs.

The helper below is deliberately bounded to a four-leg rectangular path. It
is a reference composition, not a general path-rounding implementation.
Adjacent legs must be longer than twice the pipe radius so the spherical
corner cutters remain disjoint.

Run it in an environment where ``usd_brep`` is importable::

    python Examples/PyAPI/round_corner_pipe.py
"""

import argparse
import math

import usd_brep as sm


_GEOMETRY_TOLERANCE = 1.0e-8


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b, strict=True))


def _subtract(a, b):
    return tuple(x - y for x, y in zip(a, b, strict=True))


def _scale(value, vector):
    return tuple(value * component for component in vector)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b, strict=True))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(vector):
    return math.sqrt(_dot(vector, vector))


def _unit(vector):
    length = _length(vector)
    if length <= _GEOMETRY_TOLERANCE:
        raise ValueError("path contains a degenerate leg")
    return _scale(1.0 / length, vector)


def _distance(a, b):
    return _length(_subtract(a, b))


def _validate_rectangular_path(path_curves, radius):
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("radius must be finite and positive")
    if len(path_curves) != 4:
        raise ValueError("the reference composition requires four path legs")
    if any(not isinstance(curve, sm.Line) for curve in path_curves):
        raise ValueError("every path leg must be a Line")

    starts = [curve.start_point() for curve in path_curves]
    ends = [curve.end_point() for curve in path_curves]
    directions = []
    for index, (start, end) in enumerate(zip(starts, ends, strict=True)):
        if any(not math.isfinite(value) for value in (*start, *end)):
            raise ValueError("path points must be finite")
        leg = _subtract(end, start)
        if _length(leg) <= 2.0 * radius + _GEOMETRY_TOLERANCE:
            raise ValueError("each path leg must be longer than twice the radius")
        directions.append(_unit(leg))
        next_start = starts[(index + 1) % len(starts)]
        if _distance(end, next_start) > _GEOMETRY_TOLERANCE:
            raise ValueError("path legs must form an ordered closed loop")

    reference_axis = None
    for index, outgoing in enumerate(directions):
        incoming = directions[index - 1]
        if abs(_dot(incoming, outgoing)) > _GEOMETRY_TOLERANCE:
            raise ValueError("every path corner must be 90 degrees")
        turn_axis = _unit(_cross(incoming, outgoing))
        if reference_axis is None:
            reference_axis = turn_axis
        elif _dot(reference_axis, turn_axis) < 1.0 - _GEOMETRY_TOLERANCE:
            raise ValueError("path must be a consistently wound planar rectangle")

    return starts, directions


def _create_round_corner_cutter(corner, incoming, outgoing, radius):
    """Create ``outer-corner box - sphere`` using public primitives."""
    turn_axis = _unit(_cross(incoming, outgoing))
    outer_u = incoming
    outer_v = _scale(-1.0, outgoing)
    base = _subtract(corner, _scale(2.0 * radius, turn_axis))
    rectangle = (
        base,
        _add(base, _scale(2.0 * radius, outer_u)),
        _add(
            _add(base, _scale(2.0 * radius, outer_u)),
            _scale(2.0 * radius, outer_v),
        ),
        _add(base, _scale(2.0 * radius, outer_v)),
    )
    box_profile = [
        sm.create_line_segment(rectangle[index], rectangle[(index + 1) % 4])
        for index in range(4)
    ]
    box = sm.linear_sweep(
        box_profile, turn_axis, 4.0 * radius, cap_ends=True)
    sphere = sm.create_sphere(corner, radius)
    return sm.boolean_difference(box, sphere)


def build_round_corner_pipe(path_curves, radius):
    """Build the bounded OCP-style rectangular pipe composition.

    ``path_curves`` are borrowed and remain unchanged. Boolean operations
    consume only temporary bodies created inside this function.
    """
    world_corners, world_directions = _validate_rectangular_path(
        path_curves, radius)

    # Construct in a canonical XY frame, then rigidly place the result. This
    # keeps the reference numerics independent of the rectangle's world-space
    # orientation while leaving the caller's Curve handles untouched.
    origin = world_corners[0]
    x_axis = world_directions[0]
    y_axis = world_directions[1]
    z_axis = _unit(_cross(x_axis, y_axis))
    local_corners = []
    for corner in world_corners:
        relative = _subtract(corner, origin)
        local_corners.append((
            _dot(relative, x_axis),
            _dot(relative, y_axis),
            _dot(relative, z_axis),
        ))
    local_path_curves = [
        sm.create_line_segment(
            local_corners[index], local_corners[(index + 1) % 4])
        for index in range(4)
    ]
    corners, directions = _validate_rectangular_path(
        local_path_curves, radius)

    profile = sm.create_circle((0.0, 0.0, 0.0), radius)
    result = sm.sweep_along_planar_path(
        [profile],
        local_path_curves,
        move_profile=True,
        cap_ends=sm.CapEnds.NONE,
    )

    for index, corner in enumerate(corners):
        cutter = _create_round_corner_cutter(
            corner, directions[index - 1], directions[index], radius)
        result = sm.boolean_difference(result, cutter)

    sm.transform(result, sm.Axis2Placement(origin, x_axis, y_axis))

    volume = result.volume(relative_accuracy=1.0e-8)
    if not result.is_manifold_solid() or not math.isfinite(volume) or volume <= 0.0:
        raise RuntimeError("round-corner composition did not produce a material solid")
    return result


def _square_path(side_length):
    points = (
        (0.0, 0.0, 0.0),
        (side_length, 0.0, 0.0),
        (side_length, side_length, 0.0),
        (0.0, side_length, 0.0),
        (0.0, 0.0, 0.0),
    )
    return [
        sm.create_line_segment(start, end)
        for start, end in zip(points[:-1], points[1:], strict=True)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--side", type=float, default=10.0,
                        help="Square centerline side length (default: 10).")
    parser.add_argument("--radius", type=float, default=1.0,
                        help="Circular pipe radius (default: 1).")
    parser.add_argument("--output", default=None,
                        help="Optional native .smb output path.")
    args = parser.parse_args()

    result = build_round_corner_pipe(_square_path(args.side), args.radius)
    print("manifold solid:", result.is_manifold_solid())
    print("topology:", result.face_count(), "faces,",
          result.edge_count(), "edges,", result.vertex_count(), "vertices")
    print("fine volume:", result.volume(relative_accuracy=1.0e-8))
    print("tight bounds:", result.bounding_box(tight=True))
    if args.output:
        result.write_to_file(args.output)
        print("wrote:", args.output)


if __name__ == "__main__":
    main()
