# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for reflected Boolean operands and surface planarity."""

import math
import unittest

import _omni_solid as sm


class TestReflectedCoincidentUnion(unittest.TestCase):
    def test_mixed_analytic_and_reflected_nurbs_preserve_both_operands(self):
        # Analytic reference: each 30x10x6 box overlaps half the cylinder's central |y| <= 5 strip.
        cylinder_volume = math.pi * 8**2 * 6
        strip_area = 2 * (5 * math.sqrt(8**2 - 5**2) + 8**2 * math.asin(5 / 8))
        # The overlap volume per box is (strip_area / 2) * 6 = 3 * strip_area.
        input_volume = 1800 + cylinder_volume - 3 * strip_area
        union_volume = 3600 + cylinder_volume - 6 * strip_area

        # Both operands contain the whole cylinder; their boxes extend in opposite x directions.
        operations = (
            (sm.boolean_union, union_volume),
            (sm.boolean_intersection, cylinder_volume),
            (sm.boolean_difference, input_volume - cylinder_volume),
        )

        # Cover both representations, equivalent transforms, and operand orders for each operation.
        for nurbs in (False, True):
            for transform in ("reflect", "rotate"):
                for swap in (False, True):
                    for operation, expected in operations:
                        with self.subTest(nurbs=nurbs, transform=transform, swap=swap,
                                          operation=operation.__name__):
                            # Booleans consume their inputs, so every case needs a fresh pair.
                            a = sm.boolean_union(sm.create_box((0, -5, 0), 30, 10, 6),
                                                 sm.create_cylinder((0, 0, 0), 8, 6))
                            if nurbs:
                                sm.turn_to_nurbs(a)
                            b = a.copy()
                            # Symmetry about y = 0 makes reflection and a 180-degree Z rotation the same point set.
                            if transform == "reflect":
                                sm.scale(b, (-1, 1, 1))
                            else:
                                sm.rotate(b, (0, 0, 0), (0, 0, 1), 180)
                            result = operation(b, a) if swap else operation(a, b)
                            self.assertTrue(result.is_manifold_solid())
                            self.assertAlmostEqual(result.volume(relative_accuracy=1e-8), expected, delta=0.002)
                            # The union must reach both box tips at x = -30 and x = 30.
                            if operation is sm.boolean_union:
                                lo, hi = result.bounding_box(tight=True)
                                for actual, want in zip(lo + hi, (-30, -8, 0, 30, 8, 6)):
                                    self.assertAlmostEqual(actual, want, places=5)

    def test_planarity_measures_geometry_below_collision_tolerance(self):
        # Rigid transforms preserve planarity; converting to NURBS exercises the generic bounding-box check.
        for rotated in (False, True):
            box = sm.create_box((0, 0, 0), 30, 10, 6)
            sm.turn_to_nurbs(box)
            if rotated:
                sm.rotate(box, (0, 0, 0), (1, 2, 3), 37)
                sm.translate(box, (7, -3, 2))
            for face in box.faces():
                self.assertTrue(face.surface().is_planar(1e-9))
        # Curved control: a NURBS sphere must still be classified as nonplanar.
        sphere = sm.create_sphere((0, 0, 0), 8)
        sm.turn_to_nurbs(sphere)
        self.assertTrue(all(not face.surface().is_planar(1e-6) for face in sphere.faces()))


if __name__ == "__main__":
    unittest.main()
