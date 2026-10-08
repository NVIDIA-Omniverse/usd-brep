#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Qualification for the public-API round-corner pipe example."""

import importlib.util
import math
from pathlib import Path
import unittest

import _omni_solid as sm


_REPO_ROOT = Path(__file__).resolve().parents[3]
_EXAMPLE_PATH = _REPO_ROOT / "Examples" / "PyAPI" / "round_corner_pipe.py"
_SPEC = importlib.util.spec_from_file_location(
    "round_corner_pipe", _EXAMPLE_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError(f"could not load example: {_EXAMPLE_PATH}")
_EXAMPLE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_EXAMPLE)


class TestRoundCornerPipeExample(unittest.TestCase):
    @staticmethod
    def _snapshot(curve):
        lo, hi = curve.parameter_range()
        return (
            (lo, hi),
            curve.evaluate(lo),
            curve.evaluate(0.5 * (lo + hi)),
            curve.evaluate(hi),
            curve.length(),
        )

    def _assert_curve_unchanged(self, curve, expected):
        actual = self._snapshot(curve)
        for got, want in zip(actual[0], expected[0], strict=True):
            self.assertAlmostEqual(got, want, places=9)
        for actual_point, expected_point in zip(
                actual[1:4], expected[1:4], strict=True):
            for got, want in zip(actual_point, expected_point, strict=True):
                self.assertAlmostEqual(got, want, places=8)
        self.assertAlmostEqual(actual[4], expected[4], places=8)

    def _assert_bounds_near(self, brep, expected):
        actual = brep.bounding_box(tight=True)
        for actual_point, expected_point in zip(actual, expected, strict=True):
            for got, want in zip(actual_point, expected_point, strict=True):
                self.assertAlmostEqual(got, want, delta=2.0e-4)

    def test_closed_orthogonal_round_pipe_matches_ocp_reference(self):
        cases = (
            (
                "xy",
                ((0, 0, 0), (10, 0, 0), (10, 10, 0),
                 (0, 10, 0), (0, 0, 0)),
                ((-1, -1, -1), (11, 11, 1)),
            ),
            (
                "xy_reversed",
                ((0, 0, 0), (0, 10, 0), (10, 10, 0),
                 (10, 0, 0), (0, 0, 0)),
                ((-1, -1, -1), (11, 11, 1)),
            ),
            (
                "yz",
                ((0, 0, 0), (0, 10, 0), (0, 10, 10),
                 (0, 0, 10), (0, 0, 0)),
                ((-1, -1, -1), (1, 11, 11)),
            ),
            (
                "translated_xy",
                ((37, -19, 23), (47, -19, 23), (47, -9, 23),
                 (37, -9, 23), (37, -19, 23)),
                ((36, -20, 22), (48, -8, 24)),
            ),
        )
        expected_volume = 124.51916300794949

        for name, points, expected_bounds in cases:
            with self.subTest(case=name):
                paths = [
                    sm.create_line_segment(start, end)
                    for start, end in zip(
                        points[:-1], points[1:], strict=True)
                ]
                snapshots = [self._snapshot(curve) for curve in paths]

                result = _EXAMPLE.build_round_corner_pipe(paths, radius=1.0)

                self.assertIsInstance(result, sm.Brep)
                self.assertTrue(result.is_manifold_solid())
                volume = result.volume(relative_accuracy=1.0e-8)
                self.assertTrue(math.isfinite(volume))
                self.assertGreater(volume, 0.0)
                self.assertAlmostEqual(
                    volume, expected_volume, delta=2.0e-5)
                # Every emitted face must be measurable, without constraining
                # how the kernel partitions the cylindrical and corner areas.
                face_areas = [
                    face.area(relative_accuracy=1.0e-4)
                    for face in result.faces()
                ]
                for face_area in face_areas:
                    self.assertTrue(math.isfinite(face_area))
                    self.assertGreater(face_area, 0.0)
                solid_area = result.mass_properties(
                    relative_accuracy=1.0e-4)["area"]
                self.assertAlmostEqual(
                    sum(face_areas), solid_area, delta=2.0e-4 * solid_area)
                self._assert_bounds_near(result, expected_bounds)
                for curve, snapshot in zip(paths, snapshots, strict=True):
                    self._assert_curve_unchanged(curve, snapshot)


if __name__ == "__main__":
    unittest.main()
