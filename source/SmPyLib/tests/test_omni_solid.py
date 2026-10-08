#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the _omni_solid Python bindings."""

import copy
import gc
import math
import os
import re
import subprocess
import sys
import tempfile
import unittest

try:
    import numpy as np
except ImportError:  # pragma: no cover - CI Python lacks numpy
    np = None

if sys.platform == "win32":
    for p in os.environ.get("PATH", "").split(";"):
        if p and os.path.isdir(p):
            os.add_dll_directory(p)

import _omni_solid as sm


def _inject_xform_ops(usda_text, prim_name, inject_lines):
    """Insert ``inject_lines`` at the top of the body of ``def ... "prim_name"``.

    Avoids a dependency on USD Python (``pxr``), which is not on the
    SmPyLib test PYTHONPATH. Relies on SMLib's stable USDA writer layout:
    a prim ``def`` line followed by an optional ``( ... )`` metadata block
    and then a body-opening line whose sole content is ``{``.
    """
    needle = '"%s"' % prim_name
    out = []
    state = "search"  # search -> found_def -> done
    for line in usda_text.splitlines():
        out.append(line)
        if state == "search" and line.lstrip().startswith("def ") and needle in line:
            state = "found_def"
        elif state == "found_def" and line.strip() == "{":
            out.extend("        " + il for il in inject_lines)
            state = "done"
    if state != "done":
        raise AssertionError("could not inject xformOps into prim %r" % prim_name)
    return "\n".join(out) + "\n"


_TRANSLATE_OPS = lambda x, y, z: [  # noqa: E731
    "double3 xformOp:translate = (%g, %g, %g)" % (x, y, z),
    'uniform token[] xformOpOrder = ["xformOp:translate"]',
]


class TestVec3(unittest.TestCase):
    def test_construct_xyz(self):
        v = sm.Vec3(1.0, 2.0, 3.0)
        self.assertAlmostEqual(v.x, 1.0)
        self.assertAlmostEqual(v.y, 2.0)
        self.assertAlmostEqual(v.z, 3.0)

    def test_repr(self):
        v = sm.Vec3(1.0, 2.0, 3.0)
        self.assertIn("Vec3", repr(v))


class TestVec2(unittest.TestCase):
    def test_construct(self):
        v = sm.Vec2(1.0, 2.0)
        self.assertAlmostEqual(v.x, 1.0)
        self.assertAlmostEqual(v.y, 2.0)

    def test_repr(self):
        v = sm.Vec2(1.0, 2.0)
        self.assertIn("Vec2", repr(v))


class TestEnums(unittest.TestCase):
    def test_boolean_ops(self):
        self.assertIsNotNone(sm.BooleanOp.UNION)
        self.assertIsNotNone(sm.BooleanOp.DIFFERENCE)
        self.assertIsNotNone(sm.BooleanOp.INTERSECTION)
        self.assertIsNotNone(sm.BooleanOp.MERGE)

    def test_boolean_ops_2d(self):
        self.assertIsNotNone(sm.BooleanOp2D.UNION)
        self.assertIsNotNone(sm.BooleanOp2D.INTERSECTION)
        self.assertIsNotNone(sm.BooleanOp2D.DIFFERENCE)
        self.assertIsNotNone(sm.BooleanOp2D.EXCLUSIVE_OR)
        self.assertIsNotNone(sm.BooleanOp2D.MERGE)

    def test_fillet_xsect(self):
        self.assertIsNotNone(sm.FilletXSect.LINEAR)
        self.assertIsNotNone(sm.FilletXSect.CIRCULAR)
        self.assertIsNotNone(sm.FilletXSect.BLEND)


# -----------------------------------------------------------------------
#  Primitives
# -----------------------------------------------------------------------
class TestPrimitives(unittest.TestCase):
    def test_create_box(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertIsInstance(box, sm.Brep)
        self.assertEqual(len(box.faces()), 6)
        self.assertEqual(len(box.edges()), 12)
        self.assertEqual(len(box.vertices()), 8)

    def test_create_sphere(self):
        sph = sm.create_sphere((0, 0, 0), 5)
        self.assertIsInstance(sph, sm.Brep)
        self.assertGreater(len(sph.faces()), 0)

    def test_create_cone(self):
        cone = sm.create_cone((0, 0, 0), 5, 2, 10)
        self.assertIsInstance(cone, sm.Brep)
        self.assertGreater(len(cone.faces()), 0)

    def test_create_cylinder(self):
        cyl = sm.create_cylinder((0, 0, 0), 5, 10)
        self.assertIsInstance(cyl, sm.Brep)
        self.assertGreater(len(cyl.faces()), 0)

    def test_create_torus(self):
        tor = sm.create_torus((0, 0, 0), 10, 3)
        self.assertIsInstance(tor, sm.Brep)
        self.assertGreater(len(tor.faces()), 0)

    def test_create_plane(self):
        pln = sm.create_plane((0, 0, 0), 10, 10)
        self.assertIsInstance(pln, sm.Brep)
        self.assertEqual(len(pln.faces()), 1)

    def test_create_planar_circle(self):
        pc = sm.create_planar_circle((0, 0, 0), 5)
        self.assertIsInstance(pc, sm.Brep)
        self.assertGreater(len(pc.faces()), 0)

    def test_create_pyramid(self):
        pyr = sm.create_pyramid((0, 0, 0), 10, 10)
        self.assertIsInstance(pyr, sm.Brep)
        # Square base + 4 triangular sides stitched into a closed solid.
        self.assertEqual(len(pyr.faces()), 5)
        self.assertEqual(len(pyr.vertices()), 5)
        self.assertTrue(pyr.is_manifold_solid())
        self.assertGreater(pyr.volume(), 0.0)

    def test_create_cylindrical_box(self):
        r = sm.create_cylindrical_box((0, 0, 0), 10, 3, 5, 0, 90)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_partial_sphere(self):
        r = sm.create_partial_sphere((0, 0, 0), 5, 0, 180)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_sphere_no_pole(self):
        r = sm.create_sphere_no_pole((0, 0, 0), 5)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_partial_cone(self):
        r = sm.create_partial_cone((0, 0, 0), 5, 2, 10, 0, 180)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_cone_elliptic_ends(self):
        r = sm.create_cone_elliptic_ends(5, 3, 10)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_partial_cylinder(self):
        r = sm.create_partial_cylinder((0, 0, 0), 5, 10, 0, 180)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_partial_torus(self):
        r = sm.create_partial_torus((0, 0, 0), 10, 3, 0, 180)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_blend_primitive_exposed_on_stable_api(self):
        self.assertTrue(callable(getattr(sm, "create_blend_primitive", None)))


# -----------------------------------------------------------------------
#  Skin profile inputs must survive both success and failure
# -----------------------------------------------------------------------
class TestSkinPrimitiveInputCurvesUnchanged(unittest.TestCase):
    _SKIN_INPUT_LIFETIME_SCRIPT = r"""
import math
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm


def snapshot(curve):
    low, high = curve.parameter_range()
    midpoint = curve.evaluate(0.5 * (low + high))
    return (low, high, tuple(midpoint), curve.length())


def require_unchanged(curve, expected):
    actual = snapshot(curve)
    if len(actual[2]) != len(expected[2]):
        raise AssertionError("Curve evaluation dimension changed")
    for got, want in zip(actual[2], expected[2], strict=True):
        if not math.isclose(got, want, rel_tol=1.0e-9, abs_tol=1.0e-8):
            raise AssertionError("Curve midpoint changed: %r != %r" % (actual[2], expected[2]))
    for got, want in ((actual[0], expected[0]), (actual[1], expected[1]), (actual[3], expected[3])):
        if not math.isclose(got, want, rel_tol=1.0e-9, abs_tol=1.0e-8):
            raise AssertionError("Curve scalar property changed: %r != %r" % (actual, expected))


case = sys.argv[1]
if case == "closed_success":
    lower = sm.create_circle((0, 0, 0), 3)
    upper = sm.create_circle((0, 0, 10), 6)
    lower_snapshot = snapshot(lower)
    upper_snapshot = snapshot(upper)
    result = sm.create_skin_primitive([1, 1], [lower, upper], degree=1, cap_ends=True)
    if result.face_count() != 3 or not result.is_manifold_solid() or result.volume() <= 0.0:
        raise AssertionError("Skin result is not the expected closed solid")
elif case == "open_success":
    lower = sm.create_line_segment((0, 0, 0), (2, 0, 0))
    upper = sm.create_line_segment((0, 0, 10), (2, 0, 10))
    lower_snapshot = snapshot(lower)
    upper_snapshot = snapshot(upper)
    result = sm.create_skin_primitive([1, 1], [lower, upper], degree=1, cap_ends=True)
    if result.face_count() != 1:
        raise AssertionError("Skin result is not the expected open sheet")
elif case == "failure":
    lower = sm.create_circle((0, 0, 0), 3)
    upper = sm.create_line_segment((0, 0, 10), (2, 0, 10))
    lower_snapshot = snapshot(lower)
    upper_snapshot = snapshot(upper)
    try:
        sm.create_skin_primitive([1, 1], [lower, upper], degree=1, cap_ends=True)
    except RuntimeError as error:
        if not str(error):
            raise AssertionError("Skin rejection did not provide an error message")
    else:
        raise AssertionError("Mismatched closed/open profiles unexpectedly succeeded")
elif case == "post_transfer_failure":
    curves = [
        sm.create_circle((-5, 0, 0), 2),
        sm.create_circle((5, 0, 0), 2),
        sm.create_circle((-5, 0, 10), 2),
        sm.create_circle((5, 0, 10), 2),
    ]
    curve_snapshots = [snapshot(curve) for curve in curves]
    try:
        sm.create_skin_primitive([2, 2], curves, degree=1, cap_ends=True)
    except RuntimeError as error:
        if not str(error):
            raise AssertionError("Multi-face profile rejection did not provide an error message")
    else:
        raise AssertionError("Multi-face profiles unexpectedly succeeded")
elif case == "multi_loop":
    curves = [
        sm.create_circle((0, 0, 0), 4),
        sm.create_circle((0, 0, 0), 2),
        sm.create_circle((0, 0, 10), 5),
        sm.create_circle((0, 0, 10), 2.5),
    ]
    curve_snapshots = [snapshot(curve) for curve in curves]
    solid = sm.create_skin_primitive([2, 2], curves, degree=1, cap_ends=True)
    if solid.face_count() != 4 or not solid.is_manifold_solid() or solid.volume() <= 0.0:
        raise AssertionError("Capped multi-loop profile did not produce an annular solid")
    sheet = sm.create_skin_primitive([2, 2], curves, degree=1, cap_ends=False)
    if sheet.face_count() != 2 or sheet.is_manifold_solid():
        raise AssertionError("Uncapped multi-loop profile did not produce two sheet walls")
elif case == "planar_loft_capped":
    # Coplanar profiles have no depth to cap: capping must be rejected as
    # invalid input rather than returning a degenerate zero-volume "solid".
    lower = sm.create_circle((0, 0, 0), 6)
    upper = sm.create_circle((0, 0, 0), 3)
    lower_snapshot = snapshot(lower)
    upper_snapshot = snapshot(upper)
    try:
        sm.create_skin_primitive([1, 1], [lower, upper], degree=1, cap_ends=True)
    except RuntimeError as error:
        if not str(error):
            raise AssertionError("Planar-loft cap rejection did not provide an error message")
    else:
        raise AssertionError("Capping a coplanar (planar) loft unexpectedly succeeded")
elif case == "preflight":
    lower = sm.create_circle((0, 0, 0), 3)
    upper = sm.create_circle((0, 0, 10), 6)
    lower_snapshot = snapshot(lower)
    upper_snapshot = snapshot(upper)
    try:
        sm.create_skin_primitive([1, 0], [lower, upper], degree=1, cap_ends=True)
    except RuntimeError as error:
        if not str(error):
            raise AssertionError("Invalid profile counts did not provide an error message")
    else:
        raise AssertionError("Invalid profile counts unexpectedly succeeded")
else:
    raise AssertionError("Unknown test case: %s" % case)

if case in ("post_transfer_failure", "multi_loop"):
    for curve, curve_snapshot in zip(curves, curve_snapshots, strict=True):
        require_unchanged(curve, curve_snapshot)
else:
    require_unchanged(lower, lower_snapshot)
    require_unchanged(upper, upper_snapshot)
print("skin input curves survived %s" % case)
"""

    def _assert_skin_input_lifetime(self, case):
        try:
            completed = subprocess.run(
                [sys.executable, "-c", self._SKIN_INPUT_LIFETIME_SCRIPT, case],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.fail("create_skin_primitive timed out in the %s case" % case)

        self.assertEqual(
            completed.returncode,
            0,
            msg="child output:\n%s\n%s" % (completed.stdout, completed.stderr),
        )
        self.assertIn("skin input curves survived %s" % case, completed.stdout)

    def test_closed_success_preserves_both_input_curves(self):
        self._assert_skin_input_lifetime("closed_success")

    def test_open_success_preserves_both_input_curves(self):
        self._assert_skin_input_lifetime("open_success")

    def test_failure_after_first_profile_preserves_both_input_curves(self):
        self._assert_skin_input_lifetime("failure")

    def test_failure_after_current_profile_transfer_preserves_all_input_curves(self):
        self._assert_skin_input_lifetime("post_transfer_failure")

    def test_multi_loop_profiles_succeed_and_preserve_all_input_curves(self):
        self._assert_skin_input_lifetime("multi_loop")

    def test_preflight_rejection_preserves_both_input_curves(self):
        self._assert_skin_input_lifetime("preflight")

    def test_planar_loft_cap_rejected_and_preserves_curves(self):
        self._assert_skin_input_lifetime("planar_loft_capped")


# -----------------------------------------------------------------------
#  Multi-section skins cap only their first and last profiles
# -----------------------------------------------------------------------
class TestSkinPrimitiveGeometry(unittest.TestCase):
    def test_capped_nonuniform_multisection_skin(self):
        divisions = 180
        source_points = ((10.0, -10.0), (30.0, -10.0), (30.0, 10.0), (10.0, 10.0))
        curves = []
        profile_points = []

        for index in range(divisions + 1):
            fraction = index / divisions
            angle = math.radians(90.0 * fraction)
            cosine = math.cos(angle)
            sine = math.sin(angle)
            scale_x = 1.0 - 0.5 * fraction
            z = 10.0 * fraction
            points = [
                (scale_x * x * cosine - y * sine, scale_x * x * sine + y * cosine, z)
                for x, y in source_points
            ]
            profile_points.extend(points)
            curves.extend(
                sm.create_line_segment(start, end)
                for start, end in zip(points, points[1:] + points[:1], strict=True)
            )

        solid = sm.create_skin_primitive(
            [4] * (divisions + 1), curves, degree=1, cap_ends=True
        )

        self.assertTrue(solid.is_manifold_solid())
        # Each degree-1 span linearly blends adjacent rotation/scale transforms.
        # Integrating their determinants over the 400-area profile and 10-unit height gives
        # 1000 * (2 + cos(delta_angle)), where delta_angle = pi / (2 * divisions).
        expected_volume = 1000.0 * (2.0 + math.cos(math.pi / (2.0 * divisions)))
        self.assertAlmostEqual(solid.volume(relative_accuracy=1.0e-8), expected_volume, delta=5.0e-3)

        bounds_min, bounds_max = solid.bounding_box(tight=True)
        expected_min = tuple(min(point[axis] for point in profile_points) for axis in range(3))
        expected_max = tuple(max(point[axis] for point in profile_points) for axis in range(3))
        for actual, expected in zip(bounds_min + bounds_max, expected_min + expected_max, strict=True):
            self.assertAlmostEqual(actual, expected, delta=1.0e-5)


# -----------------------------------------------------------------------
#  create_offset_profile must not invalidate a source Face's borrowed edges
# -----------------------------------------------------------------------
class TestOffsetProfileInputOwnership(unittest.TestCase):
    # Runs in a subprocess: on the unfixed revision, either case segfaults
    # the process instead of raising or returning normally.
    _SCRIPT = r"""
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm

case = sys.argv[1]
source = sm.create_planar_faces(sm.create_rectangle((0, 0, 0), 10.0, 10.0))
source_face = source.faces()[0]
area_before = source_face.area()
borrowed_curves = [edge.curve() for edge, _forward in source_face.boundary_outer_loop()]
curve_lengths_before = [curve.length() for curve in borrowed_curves]

if case == "success":
    offset = sm.create_offset_profile(
        borrowed_curves, 1.0, offset_type=sm.OffsetType.RIGHT, round_corners=True,
    )
    if offset.face_count() == 0:
        raise AssertionError("Offset call produced no faces")
elif case in ("shell_left", "shell_right"):
    offset_type = sm.OffsetType.LEFT if case == "shell_left" else sm.OffsetType.RIGHT
    offset = sm.create_offset_profile(
        borrowed_curves, 1.0, offset_type=offset_type,
        round_corners=True, shell_result=True,
    )
    if offset.face_count() == 0:
        raise AssertionError("Shell offset call produced no faces")
elif case == "shell_failure":
    try:
        sm.create_offset_profile(
            borrowed_curves, 5.0, offset_type=sm.OffsetType.LEFT,
            round_corners=True, shell_result=True,
        )
    except RuntimeError:
        pass
    else:
        raise AssertionError("Expected shell offset to fail for this case")
elif case == "failure":
    # An offset wider than the square collapses the loop: OffsetProfile
    # consumes the curves, then fails later while building the offset.
    try:
        sm.create_offset_profile(
            borrowed_curves, 5.0, offset_type=sm.OffsetType.LEFT, round_corners=True,
        )
    except RuntimeError:
        pass
    else:
        raise AssertionError("Expected create_offset_profile to fail for this case")
elif case == "unclosed":
    # An open (non-closed) profile fails before OffsetProfile ever consumes
    # its curves -- they must remain valid and independently owned after.
    open_curves = [
        sm.create_line_segment((0, 0, 0), (10, 0, 0)),
        sm.create_line_segment((10, 0, 0), (10, 10, 0)),
        sm.create_line_segment((10, 10, 0), (0, 10, 0)),
    ]
    lengths_before = [c.length() for c in open_curves]
    try:
        sm.create_offset_profile(open_curves, 1.0, offset_type=sm.OffsetType.LEFT)
    except RuntimeError:
        pass
    else:
        raise AssertionError("Expected create_offset_profile to fail for this case")
    lengths_after = [c.length() for c in open_curves]
    if lengths_after != lengths_before:
        raise AssertionError("Curve lengths changed: %r != %r" % (lengths_after, lengths_before))
    print("input curves survived %s" % case)
    sys.exit(0)
elif case == "partial_transfer_failure":
    # A self-intersecting closed profile can fail after lower-level topology
    # has taken ownership of only some private curve copies.
    crossing_curves = [
        sm.create_line_segment((0, 0, 0), (10, 10, 0)),
        sm.create_line_segment((10, 10, 0), (0, 10, 0)),
        sm.create_line_segment((0, 10, 0), (10, 0, 0)),
        sm.create_line_segment((10, 0, 0), (0, 0, 0)),
    ]
    lengths_before = [c.length() for c in crossing_curves]
    try:
        sm.create_offset_profile(crossing_curves, 1.0, offset_type=sm.OffsetType.LEFT)
    except RuntimeError:
        pass
    else:
        raise AssertionError("Expected self-intersecting offset profile to fail")
    lengths_after = [c.length() for c in crossing_curves]
    if lengths_after != lengths_before:
        raise AssertionError("Curve lengths changed: %r != %r" % (lengths_after, lengths_before))
    print("input curves survived %s" % case)
    sys.exit(0)
else:
    raise AssertionError("Unknown test case: %s" % case)

area_after = source_face.area()
if abs(area_after - area_before) > 1.0e-6:
    raise AssertionError("Source face area changed: %r != %r" % (area_after, area_before))
curve_lengths_after = [curve.length() for curve in borrowed_curves]
if curve_lengths_after != curve_lengths_before:
    raise AssertionError(
        "Borrowed curve lengths changed: %r != %r" %
        (curve_lengths_after, curve_lengths_before)
    )
print("input curves survived %s" % case)
"""

    def _assert_offset_input_ownership(self, case):
        try:
            completed = subprocess.run(
                [sys.executable, "-c", self._SCRIPT, case],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.fail("create_offset_profile timed out in the %s case" % case)

        self.assertEqual(
            completed.returncode,
            0,
            msg="child output:\n%s\n%s" % (completed.stdout, completed.stderr),
        )
        self.assertIn("input curves survived %s" % case, completed.stdout)

    def test_source_face_survives_offset_of_borrowed_curves(self):
        self._assert_offset_input_ownership("success")

    def test_source_face_survives_left_shell_offset(self):
        self._assert_offset_input_ownership("shell_left")

    def test_source_face_survives_right_shell_offset(self):
        self._assert_offset_input_ownership("shell_right")

    def test_source_face_survives_shell_offset_failure(self):
        self._assert_offset_input_ownership("shell_failure")

    def test_source_face_survives_offset_failure_after_consuming_curves(self):
        self._assert_offset_input_ownership("failure")

    def test_curves_survive_offset_failure_before_consuming_curves(self):
        self._assert_offset_input_ownership("unclosed")

    def test_curves_survive_partial_transfer_failure(self):
        self._assert_offset_input_ownership("partial_transfer_failure")


# -----------------------------------------------------------------------
#  Tessellation
# -----------------------------------------------------------------------
class TestTessellateBoundaries(unittest.TestCase):
    def test_box_samples_and_edge_identity(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        points, counts, edges = sm.tessellate_boundaries(box)
        self.assertEqual(counts, [2] * 12)
        self.assertEqual(len(points), 24)
        self.assertEqual(edges, box.edges())
        self.assertEqual(box.face_count(), 6)
        self.assertEqual(box.volume(), 1000.0)
        for start, end in zip(points[::2], points[1::2]):
            self.assertAlmostEqual(sum((a - b) ** 2 for a, b in zip(start, end)), 100.0)
        self.assertEqual(sm.tessellate_boundaries(box, allow_partial=True), (points, counts, edges))

    def test_angle_refinement(self):
        sphere = sm.create_sphere((0, 0, 0), 5)
        coarse, _, _ = sm.tessellate_boundaries(sphere, angle_tolerance_deg=25.0)
        fine, counts, edges = sm.tessellate_boundaries(sphere)
        self.assertGreater(len(fine), len(coarse))
        self.assertEqual(sum(counts), len(fine))
        self.assertEqual(len(counts), len(edges))

    def test_invalid_arguments_are_not_partial_results(self):
        box = sm.create_box((0, 0, 0), 1, 1, 1)
        for partial in (False, True):
            for angle in (0.0, -1.0, float("nan"), float("inf")):
                with self.subTest(partial=partial, angle=angle), self.assertRaises(RuntimeError) as caught:
                    sm.tessellate_boundaries(box, angle_tolerance_deg=angle, allow_partial=partial)
                # The message must name the API entry point, not the binder source text.
                self.assertTrue(
                    str(caught.exception).startswith("SmApiTessellateBoundaries failed:"),
                    str(caught.exception).splitlines()[0],
                )
        with self.assertRaises(TypeError):
            sm.tessellate_boundaries(None)
        with self.assertRaises(TypeError):
            sm.tessellate_boundaries(box, 5.0)


class TestTessellate(unittest.TestCase):
    def test_box_to_polybrep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mesh = sm.tessellate(box, curve_angle_tolerance_deg=0.0)
        self.assertIsInstance(mesh, sm.PolyBrep)
        self.assertGreater(len(mesh.get_faces()), 0)
        self.assertGreater(len(mesh.get_vertices()), 0)

    def test_tessellate_with_tolerances(self):
        sph = sm.create_sphere((0, 0, 0), 5)
        mesh = sm.tessellate(
            sph,
            chord_height_tolerance=0.05,
            curve_angle_tolerance_deg=15.0,
            surface_angle_tolerance_deg=15.0,
            max_edge_length=0.0,
            max_aspect_ratio=0.0,
        )
        self.assertIsInstance(mesh, sm.PolyBrep)
        self.assertTrue(mesh.is_manifold_solid())

    def test_independent_angles(self):
        for parameter, brep in (
            ("curve_angle_tolerance_deg", sm.create_cylinder((0, 0, 0), 5, 10)),
            ("surface_angle_tolerance_deg", sm.create_sphere((0, 0, 0), 5)),
        ):
            for method in (False, True):
                with self.subTest(parameter=parameter, method=method):
                    tessellate = brep.tessellate if method else lambda brep=brep, **kw: sm.tessellate(brep, **kw)
                    options = dict(chord_height_tolerance=0.0,
                                   curve_angle_tolerance_deg=30.0, surface_angle_tolerance_deg=30.0)
                    coarse = tessellate(**options)
                    options[parameter] = 5.0
                    fine = tessellate(**options)
                    self.assertGreater(len(fine.get_faces()), len(coarse.get_faces()))
                    self.assertTrue(fine.is_manifold_solid())

    def test_default_quality(self):
        cylinder = sm.create_cylinder((0, 0, 0), 5, 10)
        for tessellate in (lambda **kwargs: sm.tessellate(cylinder, **kwargs), cylinder.tessellate):
            with self.subTest(tessellate=tessellate):
                default = tessellate()
                explicit = tessellate(
                    chord_height_tolerance=0.0,
                    curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0,
                    max_edge_length=0.0, max_aspect_ratio=0.0,
                )
                self.assertEqual(default.to_mesh_arrays(), explicit.to_mesh_arrays())

    def test_tessellation_defaults_dict(self):
        # inspect.signature() does not work on pybind11 builtins, so this dict is the
        # only programmatic route to the defaults. It must agree with them.
        defaults = sm.tessellation_defaults()
        self.assertEqual(
            set(defaults),
            {"chord_height_tolerance", "curve_angle_tolerance_deg", "surface_angle_tolerance_deg",
             "max_edge_length", "max_aspect_ratio"},
        )
        cylinder = sm.create_cylinder((0, 0, 0), 5, 10)
        self.assertEqual(
            sm.tessellate(cylinder, **defaults).to_mesh_arrays(),
            sm.tessellate(cylinder).to_mesh_arrays(),
        )
        # A fresh dict each call, so mutation cannot leak into later callers.
        defaults["chord_height_tolerance"] = 999.0
        self.assertNotEqual(sm.tessellation_defaults()["chord_height_tolerance"], 999.0)

    def test_tessellation_failure_reporting(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for tessellate in (lambda **kwargs: sm.tessellate(box, **kwargs), box.tessellate):
            mesh = tessellate()
            reported, report = tessellate(return_report=True)
            self.assertEqual(report, dict(failures=[], lamina_edge_count=0, spine_edge_count=0,
                                          input_manifold=True, output_manifold=True, has_mesh=True))
            self.assertEqual(mesh.to_mesh_arrays(), reported.to_mesh_arrays())
            for allow_partial in (False, True):
                reported, failures = tessellate(allow_partial=allow_partial, return_failures=True)
                self.assertEqual(failures, [])
                self.assertEqual(mesh.to_mesh_arrays(), reported.to_mesh_arrays())

    def test_open_mesh_statistics_are_nonfatal(self):
        sheet = sm.create_planar_faces(sm.create_rectangle((0, 0, 0), 10, 10))
        mesh, report = sm.tessellate(sheet, return_report=True)
        self.assertEqual(report["failures"], [])
        self.assertEqual(report["lamina_edge_count"], 4)
        self.assertEqual(report["spine_edge_count"], 0)
        self.assertFalse(report["input_manifold"])
        self.assertFalse(report["output_manifold"])
        self.assertTrue(report["has_mesh"])
        self.assertGreater(len(mesh.get_faces()), 0)

    def test_partial_tessellation_requires_diagnostics(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for tessellate in (lambda **kwargs: sm.tessellate(box, **kwargs), box.tessellate):
            with self.assertRaises(ValueError):
                tessellate(allow_partial=True)
        for allow_partial in (False, True):
            with self.assertRaises(sm.TessellationError) as raised:
                sm.tessellate(None, allow_partial=allow_partial, return_failures=True)
            self.assertEqual(raised.exception.failures, [])
            self.assertFalse(raised.exception.report["has_mesh"])
            self.assertIsInstance(raised.exception.status, int)
            self.assertTrue(raised.exception.status_name.startswith("SM_"))
            self.assertIn(raised.exception.status_name, str(raised.exception))

    def test_brep_method_form(self):
        # Brep.tessellate(...) is the method-form sibling of sm.tessellate(brep, ...)
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mesh = box.tessellate(curve_angle_tolerance_deg=0.0)
        self.assertIsInstance(mesh, sm.PolyBrep)
        self.assertGreater(len(mesh.get_faces()), 0)

    def test_brep_method_form_kwargs_passthrough(self):
        sph = sm.create_sphere((0, 0, 0), 5)
        mesh = sph.tessellate(
            chord_height_tolerance=0.05,
            curve_angle_tolerance_deg=15.0,
            surface_angle_tolerance_deg=15.0,
        )
        self.assertIsInstance(mesh, sm.PolyBrep)
        self.assertTrue(mesh.is_manifold_solid())

    def test_brep_method_form_does_not_mutate(self):
        # Pure: source Brep is unchanged after tessellation.
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face_count_before = box.face_count()
        _ = box.tessellate(curve_angle_tolerance_deg=0.0)
        self.assertEqual(box.face_count(), face_count_before)

    def test_polybrep_to_mesh_arrays(self):
        # CI's bundled Python does not ship numpy; keep list-array coverage here.
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mesh = sm.tessellate(box, curve_angle_tolerance_deg=0.0)
        arrays = mesh.to_mesh_arrays()

        self.assertEqual(set(arrays), {"points", "normals", "faces", "face_normals"})
        self.assertGreater(len(arrays["points"]), 0)
        self.assertEqual(len(arrays["points"]), len(arrays["normals"]))
        self.assertGreater(len(arrays["faces"]), 0)
        self.assertGreater(len(arrays["face_normals"]), 0)
        self.assertEqual(len(arrays["points"][0]), 3)
        self.assertEqual(len(arrays["normals"][0]), 3)
        self.assertEqual(len(arrays["face_normals"][0]), 3)
        self.assertIsInstance(arrays["faces"][0], int)

    def test_face_varying_mesh_normals(self):
        for name, brep in (("box", sm.create_box((0, 0, 0), 10, 10, 10)),
                           ("cylinder", sm.create_cylinder((0, 0, 0), 2, 5))):
            with self.subTest(shape=name):
                mesh = sm.tessellate(brep)
                arrays = mesh.to_mesh_arrays(face_varying_normals=True)
                default = mesh.to_mesh_arrays()
                self.assertEqual(arrays["points"], default["points"])
                self.assertEqual(arrays["faces"], default["faces"])
                self.assertEqual(len(arrays["points"]), len(mesh.get_vertices()))
                for normal in arrays["normals"] + arrays["face_normals"] + default["normals"]:
                    self.assertEqual(len(normal), 3)
                    self.assertTrue(all(math.isfinite(v) for v in normal))
                    self.assertAlmostEqual(sum(v * v for v in normal), 1.0, places=6)

                offset = corner = 0
                edges = {}
                for face_normal in arrays["face_normals"]:
                    count = arrays["faces"][offset]
                    self.assertGreaterEqual(count, 3)
                    indices = arrays["faces"][offset + 1:offset + 1 + count]
                    self.assertEqual(len(indices), count)
                    for index, following in zip(indices, indices[1:] + indices[:1]):
                        point = arrays["points"][index]
                        expected = face_normal
                        if name == "cylinder" and abs(face_normal[2]) < 0.5:
                            radius = math.hypot(point[0], point[1])
                            expected = (point[0] / radius, point[1] / radius, 0)
                        for actual, value in zip(arrays["normals"][corner], expected):
                            self.assertAlmostEqual(actual, value, places=6)
                        edge = tuple(sorted((index, following)))
                        edges[edge] = edges.get(edge, 0) + 1
                        corner += 1
                    offset += count + 1
                self.assertEqual(offset, len(arrays["faces"]))
                self.assertEqual(corner, len(arrays["normals"]))
                self.assertTrue(edges)
                self.assertTrue(all(count == 2 for count in edges.values()))

    @unittest.skipUnless(np is not None, "numpy is required for to_numpy_arrays tests")
    def test_polybrep_numpy_arrays_match_list_arrays(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mesh = sm.tessellate(box, curve_angle_tolerance_deg=0.0)

        list_arrays = mesh.to_mesh_arrays()
        numpy_arrays = mesh.to_numpy_arrays()

        self.assertEqual(set(numpy_arrays), {"points", "normals", "faces", "face_normals"})
        self.assertEqual(numpy_arrays["points"].dtype, np.dtype(np.float32))
        self.assertEqual(numpy_arrays["normals"].dtype, np.dtype(np.float32))
        self.assertEqual(numpy_arrays["faces"].dtype, np.dtype(np.int32))
        self.assertEqual(numpy_arrays["face_normals"].dtype, np.dtype(np.float32))
        self.assertEqual(numpy_arrays["points"].ndim, 2)
        self.assertEqual(numpy_arrays["points"].shape[1], 3)
        self.assertEqual(numpy_arrays["normals"].shape, numpy_arrays["points"].shape)
        self.assertEqual(numpy_arrays["face_normals"].ndim, 2)
        self.assertEqual(numpy_arrays["face_normals"].shape[1], 3)
        np.testing.assert_allclose(
            numpy_arrays["points"], np.asarray(list_arrays["points"], dtype=np.float32)
        )
        np.testing.assert_allclose(
            numpy_arrays["normals"], np.asarray(list_arrays["normals"], dtype=np.float32)
        )
        np.testing.assert_array_equal(
            numpy_arrays["faces"], np.asarray(list_arrays["faces"], dtype=np.int32)
        )
        np.testing.assert_allclose(
            numpy_arrays["face_normals"], np.asarray(list_arrays["face_normals"], dtype=np.float32)
        )


        list_arrays = mesh.to_mesh_arrays(face_varying_normals=True)
        numpy_arrays = mesh.to_numpy_arrays(face_varying_normals=True)
        for key, values in list_arrays.items():
            np.testing.assert_array_equal(
                numpy_arrays[key], np.asarray(values, dtype=numpy_arrays[key].dtype)
            )


class TestNativeBrepFileIo(unittest.TestCase):
    _PART_READER_SCRIPT = """
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm

selector = {"none": None, "true": True, "false": False}[sys.argv[2]]
reader = sm.Brep.read_from_file if sys.argv[3] == "class" else sm.read_brep_from_file
try:
    reader(sys.argv[1], ascii=selector)
except RuntimeError as error:
    print(error)
else:
    raise AssertionError("single-Brep reader unexpectedly accepted a part container")
"""

    def assert_part_rejected_without_crash(
        self, path, ascii_value, expected_ascii, reader_name="flat"
    ):
        selector = "none" if ascii_value is None else str(ascii_value).lower()
        try:
            completed = subprocess.run(
                [sys.executable, "-c", self._PART_READER_SCRIPT, path, selector, reader_name],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.fail("single-Brep reader timed out while rejecting a part container")

        self.assertEqual(
            completed.returncode,
            0,
            msg="child output:\n%s\n%s" % (completed.stdout, completed.stderr),
        )
        self.assertRegex(completed.stdout, r"(?i)part container")
        self.assertIn("read_part_from_file", completed.stdout)
        self.assertIn("ascii=%s" % expected_ascii, completed.stdout)

    def assert_unrecognized_part_version_rejected_without_crash(
        self, path, ascii_value, database_version, reader_name="flat"
    ):
        selector = "none" if ascii_value is None else str(ascii_value).lower()
        try:
            completed = subprocess.run(
                [sys.executable, "-c", self._PART_READER_SCRIPT, path, selector, reader_name],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.fail("single-BRep reader timed out while rejecting an unrecognized part version")

        self.assertEqual(
            completed.returncode,
            0,
            msg="child output:\n%s\n%s" % (completed.stdout, completed.stderr),
        )
        self.assertIn(
            "native SMLib binary part header with unrecognized database version %s;"
            % database_version,
            completed.stdout,
        )
        self.assertNotIn("read_part_from_file", completed.stdout)

    @staticmethod
    def write_binary_part_header(path, values):
        with open(path, "wb") as file:
            for value in values:
                file.write(value.to_bytes(8, byteorder="little"))

    @staticmethod
    def write_empty_part_files(tmpdir):
        ascii_path = os.path.join(tmpdir, "empty_ascii.smp")
        binary_path = os.path.join(tmpdir, "empty_binary.smp")
        with open(ascii_path, "w", encoding="ascii") as file:
            file.write(
                "//[Output Summary]\n"
                "//   (1) 42000000 Curves\n"
                "//   (2) 10000000 Surfaces\n"
                "//   (3) 0 Boolean Tree Nodes\n"
                "//   (4) 0 Breps\n"
                "\n// Boolean Trees\n"
                "\n\n"
                "############################################################################\n"
                "0 // Total number of attributes\n"
            )
        TestNativeBrepFileIo.write_binary_part_header(
            binary_path, (42_000_000, 10_000_000, 0, 0, 0)
        )
        return ascii_path, binary_path

    def test_write_and_read_brep_ascii(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.TemporaryDirectory(prefix="smpy_native_brep_") as tmpdir:
            path = os.path.join(tmpdir, "box.smb")
            box.write_to_file(path)

            loaded = sm.read_brep_from_file(path)
            loaded_forced = sm.read_brep_from_file(path, ascii=True)

        for result in (loaded, loaded_forced):
            self.assertIsInstance(result, sm.Brep)
            self.assertEqual(result.face_count(), box.face_count())
            self.assertEqual(result.edge_count(), box.edge_count())
            self.assertEqual(result.vertex_count(), box.vertex_count())
            self.assertAlmostEqual(result.volume(), box.volume(), places=6)

    def test_write_and_read_brep_binary_auto_detect(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.TemporaryDirectory(prefix="smpy_native_brep_") as tmpdir:
            path = os.path.join(tmpdir, "box_binary.smb")
            box.write_to_file(path, ascii=False)

            loaded = sm.read_brep_from_file(path)
            loaded_forced = sm.read_brep_from_file(path, ascii=False)

        for result in (loaded, loaded_forced):
            self.assertIsInstance(result, sm.Brep)
            self.assertEqual(result.face_count(), box.face_count())
            self.assertAlmostEqual(result.volume(), box.volume(), places=6)

    def test_write_and_read_brep_unicode_path(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.TemporaryDirectory(prefix="smpy_native_brep_") as tmpdir:
            path = os.path.join(tmpdir, "böx_日本.smb")
            box.write_to_file(path)
            loaded = sm.read_brep_from_file(path)

        self.assertIsInstance(loaded, sm.Brep)
        self.assertEqual(loaded.face_count(), box.face_count())
        self.assertAlmostEqual(loaded.volume(), box.volume(), places=6)

    def test_read_brep_unknown_header_requires_explicit_selector(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.TemporaryDirectory(prefix="smpy_native_brep_") as tmpdir:
            for ascii_value in (True, False):
                original_path = os.path.join(tmpdir, "box_%s.smb" % ascii_value)
                prefixed_path = os.path.join(tmpdir, "prefixed_%s.smb" % ascii_value)
                box.write_to_file(original_path, ascii=ascii_value)
                with open(original_path, "rb") as original:
                    contents = original.read()
                with open(prefixed_path, "wb") as prefixed:
                    prefixed.write(b"\n" + contents)

                readers = (("flat", sm.read_brep_from_file), ("class", sm.Brep.read_from_file))
                for reader_name, reader in readers:
                    with self.subTest(ascii=ascii_value, reader=reader_name, selector=None):
                        with self.assertRaisesRegex(
                            RuntimeError, "Could not determine native SMLib Brep file format"
                        ):
                            reader(prefixed_path)

                    with self.subTest(
                        ascii=ascii_value, reader=reader_name, selector=ascii_value
                    ):
                        loaded = reader(prefixed_path, ascii=ascii_value)
                        self.assertIsInstance(loaded, sm.Brep)
                        self.assertEqual(loaded.face_count(), box.face_count())
                        self.assertAlmostEqual(loaded.volume(), box.volume(), places=6)

    def test_read_brep_rejects_part_containers_without_crashing(self):
        with tempfile.TemporaryDirectory(prefix="smpy_native_part_") as tmpdir:
            ascii_path, binary_path = self.write_empty_part_files(tmpdir)

            for path, matching_ascii in ((ascii_path, True), (binary_path, False)):
                for ascii_value in (None, True, False):
                    for reader_name in ("flat", "class"):
                        with self.subTest(
                            path=os.path.basename(path), ascii=ascii_value, reader=reader_name
                        ):
                            self.assert_part_rejected_without_crash(
                                path, ascii_value, matching_ascii, reader_name
                            )

            # A binary part count can have the same first byte as the single-Brep
            # binary marker. Structured part detection must take precedence.
            marker_collision_path = os.path.join(tmpdir, "binary_v_prefix.smp")
            self.write_binary_part_header(marker_collision_path, (42_000_214, 10_000_000, 0, 0))
            self.assertEqual(42_000_214 & 0xFF, ord("V"))
            self.assert_part_rejected_without_crash(marker_collision_path, False, False)

            safety_headers = {
                "truncated_current.smp": (42_000_000, 10_000_000),
                "fixed_width_v40.smp": (40_000_000, 0, 0, 0),
            }
            for filename, values in safety_headers.items():
                path = os.path.join(tmpdir, filename)
                self.write_binary_part_header(path, values)
                with self.subTest(path=filename, ascii=False, reader="flat"):
                    self.assert_part_rejected_without_crash(path, False, False)

            unrecognized_version_headers = {
                "unrecognized_v41.smp": (41_000_000, 0, 0, 0),
                "unrecognized_v43.smp": (43_000_000, 10_000_000, 0, 0),
                "reserved_v44.smp": (44_000_000, 10_000_000, 0, 0),
            }
            for filename, values in unrecognized_version_headers.items():
                path = os.path.join(tmpdir, filename)
                self.write_binary_part_header(path, values)
                database_version = values[0] // 1_000_000
                for ascii_value in (None, True, False):
                    for reader_name in ("flat", "class"):
                        with self.subTest(
                            path=filename, ascii=ascii_value, reader=reader_name
                        ):
                            self.assert_unrecognized_part_version_rejected_without_crash(
                                path, ascii_value, database_version, reader_name
                            )

    def test_read_part_accepts_current_ascii_and_binary_containers(self):
        with tempfile.TemporaryDirectory(prefix="smpy_native_part_") as tmpdir:
            ascii_path, binary_path = self.write_empty_part_files(tmpdir)

            for path, ascii_value in ((ascii_path, True), (binary_path, False)):
                with self.subTest(path=os.path.basename(path)):
                    part = sm.read_part_from_file(path, ascii=ascii_value)
                    self.assertEqual(part, ([], [], [], []))

    def test_read_part_accepts_v40_binary_container(self):
        with tempfile.TemporaryDirectory(prefix="smpy_native_part_") as tmpdir:
            path = os.path.join(tmpdir, "empty_binary_v40.smp")
            self.write_binary_part_header(path, (40_000_000, 0, 0, 0, 0))
            part = sm.read_part_from_file(path, ascii=False)
            self.assertEqual(part, ([], [], [], []))

    def test_read_brep_rejects_explicit_format_mismatch(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.TemporaryDirectory(prefix="smpy_native_brep_") as tmpdir:
            ascii_path = os.path.join(tmpdir, "box_ascii.smb")
            binary_path = os.path.join(tmpdir, "box_binary.smb")
            box.write_to_file(ascii_path, ascii=True)
            box.write_to_file(binary_path, ascii=False)

            with self.assertRaisesRegex(RuntimeError, "appears to be ASCII"):
                sm.read_brep_from_file(ascii_path, ascii=False)
            with self.assertRaisesRegex(RuntimeError, "appears to be binary"):
                sm.read_brep_from_file(binary_path, ascii=True)

    def test_read_legacy_ascii_header_brep_fixture(self):
        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        path = os.path.join(repo, "TestFiles", "pt_TestFiles", "box_with_large_fillet.smb")
        if not os.path.isfile(path):
            self.skipTest("legacy native BRep fixture is not present")
        with open(path, "rb") as file:
            header = file.read(16)
        if not header.startswith(b"//Brep Starts"):
            self.skipTest("legacy native BRep fixture content is not available")

        loaded = sm.read_brep_from_file(path)

        self.assertIsInstance(loaded, sm.Brep)
        self.assertGreater(loaded.face_count(), 0)
        self.assertGreater(loaded.edge_count(), 0)

    def test_read_part_file(self):
        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        path = os.path.join(
            repo,
            "TestFiles",
            "bz_TestFiles",
            "Regressions2009",
            "10.Wilson",
            "SmallBox.smp",
        )
        if not os.path.isfile(path):
            self.skipTest("native SMLib part fixture is not present")

        curves, surfaces, boolean_tree_nodes, breps = sm.read_part_from_file(path)

        self.assertEqual(curves, [])
        self.assertEqual(surfaces, [])
        self.assertEqual(boolean_tree_nodes, [])
        self.assertEqual(len(breps), 1)
        self.assertIsInstance(breps[0], sm.Brep)
        self.assertGreater(breps[0].face_count(), 0)
        self.assertGreater(breps[0].volume(), 0.0)

    def test_brep_static_read_from_file(self):
        sphere = sm.create_sphere((0, 0, 0), 5)
        with tempfile.TemporaryDirectory(prefix="smpy_native_brep_") as tmpdir:
            # Content, not the conventional filename extension, selects the reader.
            path = os.path.join(tmpdir, "sphere.smp")
            sphere.write_to_file(path)

            loaded = sm.Brep.read_from_file(path)

        self.assertIsInstance(loaded, sm.Brep)
        self.assertEqual(loaded.face_count(), sphere.face_count())
        self.assertGreater(loaded.volume(), 0.0)


# -----------------------------------------------------------------------
#  Additional SmApi bindings (utilities, poly mesh, viewport draw, etc.)
# -----------------------------------------------------------------------
class TestSmApiExtraBindings(unittest.TestCase):
    def test_get_faces_via_api_matches_brep_faces(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertEqual(len(sm.get_faces_via_api(box)), len(box.faces()))

    def test_get_edges_via_api_matches_brep_edges(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertEqual(len(sm.get_edges_via_api(box)), len(box.edges()))

    def test_poly_brep_volume_via_api(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mesh = sm.tessellate(box, curve_angle_tolerance_deg=0.0)
        vol = sm.poly_brep_volume_via_api(mesh)
        self.assertGreater(vol, 0.0)

    def test_polybrep_copy_is_independent(self):
        mesh = sm.tessellate(sm.create_box((0, 0, 0), 10, 10, 10),
                             chord_height_tolerance=0.5, curve_angle_tolerance_deg=0.0)
        dup = mesh.copy()
        self.assertIsInstance(dup, sm.PolyBrep)
        self.assertIsNot(dup, mesh)
        faces_before = len(dup.get_faces())
        bbox_before = dup.calculate_bounding_box()
        # Poly booleans consume their operands; consuming the original must not
        # disturb the deep copy (shared topology would corrupt it).
        tool = sm.tessellate(sm.create_box((5, 5, 5), 10, 10, 10),
                             chord_height_tolerance=0.5, curve_angle_tolerance_deg=0.0)
        sm.poly_boolean_difference(mesh, tool)
        self.assertEqual(len(dup.get_faces()), faces_before)
        self.assertEqual(dup.calculate_bounding_box(), bbox_before)

    def test_polybrep_copy_module_is_independent(self):
        for copier in (copy.copy, copy.deepcopy):
            with self.subTest(copier=copier.__name__):
                mesh = sm.tessellate(sm.create_box((0, 0, 0), 10, 10, 10))
                dup = copier(mesh)
                self.assertIsInstance(dup, sm.PolyBrep)
                self.assertIsNot(dup, mesh)
                faces_before = len(dup.get_faces())
                bbox_before = dup.calculate_bounding_box()
                # Cutting a corner leaves the bounding box unchanged but changes the
                # face count of the edited mesh, so compare faces as well.
                sm.poly_boolean_difference(mesh, sm.tessellate(sm.create_box((5, 5, 5), 10, 10, 10)))
                self.assertNotEqual(len(mesh.get_faces()), faces_before)
                self.assertEqual(len(dup.get_faces()), faces_before)
                self.assertEqual(dup.calculate_bounding_box(), bbox_before)

    def test_poly_boolean_op_enum(self):
        self.assertIsNotNone(sm.PolyBooleanOp.UNION)

    def test_bspline_curve_form_enum(self):
        self.assertIsNotNone(sm.BSplineCurveForm.UNSPECIFIED)

    def test_axis_placement_construct(self):
        ap = sm.Axis2Placement((0, 0, 0), (1, 0, 0), (0, 1, 0))
        self.assertIsNotNone(ap)


class TestPolyBrepMassProperties(unittest.TestCase):
    """PolyBrep.mass_properties / SmApiPolyBrepComputeMassProperties.

    A box tessellates exactly, so its mesh properties must match closed-form
    values to rounding, which pins the kernel-slot conversion (second moments
    -> moments of inertia, product reordering) and the density weighting.
    """

    # Box x[1,3] y[2,6] z[3,9]: V = 48, centroid (2, 4, 6).
    ORIGIN, SIZE = (1.0, 2.0, 3.0), (2.0, 4.0, 6.0)

    def _box_mesh(self):
        return sm.tessellate(sm.create_box(self.ORIGIN, *self.SIZE))

    def test_box_matches_closed_form(self):
        (x0, y0, z0), (a, b, c) = self.ORIGIN, self.SIZE
        x1, y1, z1 = x0 + a, y0 + b, z0 + c
        v = a * b * c
        cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
        mesh = self._box_mesh()

        # Centroidal (origin=None), density-weighted: products vanish by symmetry.
        d = 2.5
        m = d * v
        mp = mesh.mass_properties(density=d)
        self.assertAlmostEqual(mp["volume"], v, places=9)
        self.assertAlmostEqual(mp["area"], 2 * (a * b + b * c + c * a), places=9)
        self.assertAlmostEqual(mp["mass"], m, places=9)
        for got, exp in zip(mp["centroid"], (cx, cy, cz), strict=True):
            self.assertAlmostEqual(got, exp, places=9)
        exp_moi = (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12)
        for got, exp in zip(mp["moments_of_inertia"], exp_moi, strict=True):
            self.assertAlmostEqual(got, exp, delta=exp * 1e-9)
        for got in mp["products_of_inertia"]:
            self.assertAlmostEqual(got, 0.0, delta=1e-7)

        # About (0,0,0) every moment and product is non-zero and distinct, so a
        # second-moment/MOI mix-up or a product-order slip changes the answer.
        def mean_sq(lo, hi):
            return (hi ** 3 - lo ** 3) / (3 * (hi - lo))

        sx, sy, sz = v * mean_sq(x0, x1), v * mean_sq(y0, y1), v * mean_sq(z0, z1)
        ref = mesh.mass_properties(density=1.0, origin=(0.0, 0.0, 0.0))
        for got, exp in zip(ref["moments_of_inertia"], (sy + sz, sx + sz, sx + sy), strict=True):
            self.assertAlmostEqual(got, exp, delta=exp * 1e-9)
        for got, exp in zip(ref["products_of_inertia"], (v * cy * cz, v * cz * cx, v * cx * cy), strict=True):
            self.assertAlmostEqual(got, exp, delta=exp * 1e-9)

        # Mass and every inertia term scale linearly with density.
        scaled = mesh.mass_properties(density=3.0, origin=(0.0, 0.0, 0.0))
        self.assertAlmostEqual(scaled["mass"], 3.0 * ref["mass"], delta=1e-9)
        for key in ("moments_of_inertia", "products_of_inertia"):
            for got, exp in zip(scaled[key], ref[key], strict=True):
                self.assertAlmostEqual(got, 3.0 * exp, delta=abs(exp) * 1e-9)

    def test_matches_brep_mass_properties_for_curved_solid(self):
        sphere = sm.create_sphere((5.0, -3.0, 2.0), 10.0)
        exact = sphere.mass_properties(relative_accuracy=1e-4)
        approx = sm.tessellate(sphere, chord_height_tolerance=0.01).mass_properties()
        # Area is left out: fine sphere meshes can exceed the true area (the
        # mesh itself, not this API -- its triangle sum agrees), so it does
        # not converge monotonically with chord height.
        for key in ("volume", "mass"):
            self.assertAlmostEqual(approx[key], exact[key], delta=exact[key] * 2e-3)
        for got, exp in zip(approx["centroid"], exact["centroid"], strict=True):
            self.assertAlmostEqual(got, exp, delta=1e-3)
        for got, exp in zip(approx["moments_of_inertia"], exact["moments_of_inertia"], strict=True):
            self.assertAlmostEqual(got, exp, delta=exp * 5e-3)

    def test_invalid_inputs_raise(self):
        mesh = self._box_mesh()
        # Bad density, non-finite origin, and a non-manifold (open) mesh all reject.
        for bad in (0.0, -1.0, float("nan"), float("inf"), 1e-13):
            with self.subTest(density=bad):
                with self.assertRaises(RuntimeError) as ctx:
                    mesh.mass_properties(density=bad)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
        with self.assertRaises(RuntimeError) as ctx:
            mesh.mass_properties(origin=(float("nan"), 0.0, 0.0))
        self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
        sheet = sm.tessellate(sm.create_planar_circle((0, 0, 0), 5.0))
        self.assertFalse(sheet.is_manifold_solid())
        with self.assertRaises(RuntimeError) as ctx:
            sheet.mass_properties()
        self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))


class TestStableApiExcludesDevSurface(unittest.TestCase):
    def test_dev_symbols_not_on_omni_solid(self):
        self.assertFalse(hasattr(sm, "extract_draw_batches"))
        self.assertFalse(hasattr(sm, "Loopuse"))
        self.assertFalse(hasattr(sm, "Edgeuse"))
        self.assertFalse(hasattr(sm, "Loop"))
        self.assertFalse(hasattr(sm, "draw"))
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertFalse(hasattr(box, "topology_pick_ray"))
        self.assertFalse(hasattr(box.edges()[0], "edgeuses"))


# -----------------------------------------------------------------------
#  Curves
# -----------------------------------------------------------------------
# --- create_helix input-domain contract (mirrors SmApiCreateHelix) ----------
# Kept in sync with source/SM_API/src/SmApiCurves.cpp.
_HELIX_MIN_H = 1.0e-7
_HELIX_MIN_R = 1.0e-7
_HELIX_MIN_TURNS = 1.0e-7
_HELIX_MIN_TOL = 1.0e-6
_HELIX_MAX_MAG = 1.0e18            # SM_IS_VALID_DOUBLE finite cap (SM_BIG_DOUBLE/100)
_HELIX_MAX_TURNS = 994.0 / 8.0     # (NL_CCPLIM - 6) / 8 == 124.25


def _helix_case(**overrides):
    """A create_helix argument set: valid defaults with the given overrides."""
    case = dict(origin=[0.0, 0.0, 0.0], height=10.0, radius_start=2.0,
                radius_end=2.0, turns=5.0, right_handed=True, tolerance=1.0e-3)
    case.update(overrides)
    return case


def _helix_invalid_cases():
    """Table of (label, case) that must all be rejected with SM_ERR_INVALID_INPUT.

    Covers the inputs that historically aborted, hung, or produced invalid
    geometry: non-finite origin/scalars, zero/negative/subnormal/boundary
    magnitudes, the zero-radius contract, bad tolerances, and turn counts at
    and beyond the fitter's control-point cap.
    """
    inf = float("inf")
    nan = float("nan")
    dbl_max = sys.float_info.max
    sub = 5.0e-324  # smallest positive subnormal double
    cases = []
    # NaN and +/-Inf for every origin coordinate.
    for i, axis in enumerate("xyz"):
        for tag, v in (("+inf", inf), ("-inf", -inf), ("nan", nan)):
            o = [0.0, 0.0, 0.0]
            o[i] = v
            cases.append(("origin.%s=%s" % (axis, tag), _helix_case(origin=o)))
    # NaN and +/-Inf for every scalar.
    for name in ("height", "radius_start", "radius_end", "turns", "tolerance"):
        for tag, v in (("+inf", inf), ("-inf", -inf), ("nan", nan)):
            cases.append(("%s=%s" % (name, tag), _helix_case(**{name: v})))
    # Zero / negative height and turns.
    cases += [
        ("height=0", _helix_case(height=0.0)),
        ("height<0", _helix_case(height=-1.0)),
        ("turns=0", _helix_case(turns=0.0)),
        ("turns<0", _helix_case(turns=-1.0)),
    ]
    # Radius contract: each single zero, both zero, and negatives are rejected.
    cases += [
        ("radius_start=0", _helix_case(radius_start=0.0)),
        ("radius_end=0", _helix_case(radius_end=0.0)),
        ("both_radii=0", _helix_case(radius_start=0.0, radius_end=0.0)),
        ("radius_start<0", _helix_case(radius_start=-1.0)),
        ("radius_end<0", _helix_case(radius_end=-1.0)),
    ]
    # Subnormal values (below the 1e-7 / 1e-6 floors).
    cases += [
        ("height=subnormal", _helix_case(height=sub)),
        ("radius_start=subnormal", _helix_case(radius_start=sub)),
        ("turns=subnormal", _helix_case(turns=sub)),
        ("tolerance=subnormal", _helix_case(tolerance=sub)),
    ]
    # Tolerance: zero, negative, and just below the 1e-6 floor.
    cases += [
        ("tolerance=0", _helix_case(tolerance=0.0)),
        ("tolerance<0", _helix_case(tolerance=-1.0)),
        ("tolerance=just_below_min",
         _helix_case(tolerance=math.nextafter(_HELIX_MIN_TOL, 0.0))),
        ("tolerance=1e-9", _helix_case(tolerance=1.0e-9)),
    ]
    # Turns just outside min/max, huge, and DBL_MAX.
    cases += [
        ("turns=just_below_min",
         _helix_case(turns=math.nextafter(_HELIX_MIN_TURNS, 0.0))),
        ("turns=just_above_max",
         _helix_case(turns=math.nextafter(_HELIX_MAX_TURNS, inf))),
        ("turns=125", _helix_case(turns=125.0)),
        ("turns=1e6", _helix_case(turns=1.0e6)),
        ("turns=DBL_MAX", _helix_case(turns=dbl_max)),
    ]
    # Magnitudes just outside the finite cap (1e18) and at DBL_MAX; a finite but
    # huge origin whose derived extent overflows the cap.
    cases += [
        ("height=just_above_max",
         _helix_case(height=math.nextafter(_HELIX_MAX_MAG, inf))),
        ("height=1e19", _helix_case(height=1.0e19)),
        ("height=DBL_MAX", _helix_case(height=dbl_max)),
        ("radius_start=1e19", _helix_case(radius_start=1.0e19)),
        ("radius_start=DBL_MAX", _helix_case(radius_start=dbl_max)),
        ("tolerance=DBL_MAX", _helix_case(tolerance=dbl_max)),
        ("origin.x=1e19", _helix_case(origin=[1.0e19, 0.0, 0.0])),
        ("origin.x=DBL_MAX", _helix_case(origin=[dbl_max, 0.0, 0.0])),
    ]
    return cases


class TestCurves(unittest.TestCase):
    @staticmethod
    def _dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def _norm(a):
        return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])

    @staticmethod
    def _cross(a, b):
        return (a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])

    def test_create_line_segment(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.assertIsInstance(line, sm.Curve)

    def test_create_circle_curve(self):
        circ = sm.create_circle((0, 0, 0), 5)
        self.assertIsInstance(circ, sm.Curve)

    def test_create_circle_is_analytic_circle(self):
        # create_circle now returns an analytic Circle (still a BSplineCurve),
        # exposing center / radius / normal / x_axis / y_axis. Verify the
        # accessors are exact AND geometrically consistent with the actual
        # curve, across several centers and radii.
        for center, radius in (((0, 0, 0), 5.0),
                               ((1, 2, 3), 0.5),
                               ((-4, 7, -2), 12.5),
                               ((100, -100, 0), 1e-3),
                               ((0, 0, 0), 1e4)):
            with self.subTest(center=center, radius=radius):
                circ = sm.create_circle(center, radius)
                self.assertIsInstance(circ, sm.Circle)
                self.assertIsInstance(circ, sm.BSplineCurve)
                self.assertIsInstance(circ, sm.Curve)

                # Exact analytic accessors.
                for got, want in zip(circ.center(), center):
                    self.assertAlmostEqual(got, float(want), delta=1e-9)
                self.assertAlmostEqual(circ.radius(), radius,
                                       delta=1e-12 + 1e-9 * radius)
                x_axis, y_axis, normal = circ.x_axis(), circ.y_axis(), circ.normal()
                for got, want in zip(x_axis, (1.0, 0.0, 0.0)):
                    self.assertAlmostEqual(got, want, delta=1e-9)
                for got, want in zip(y_axis, (0.0, 1.0, 0.0)):
                    self.assertAlmostEqual(got, want, delta=1e-9)
                for got, want in zip(normal, (0.0, 0.0, 1.0)):
                    self.assertAlmostEqual(got, want, delta=1e-9)

                # Frame is orthonormal and right-handed: |x|=|y|=|n|=1,
                # x.y=0, and x cross y == normal.
                self.assertAlmostEqual(self._norm(x_axis), 1.0, delta=1e-9)
                self.assertAlmostEqual(self._norm(y_axis), 1.0, delta=1e-9)
                self.assertAlmostEqual(self._norm(normal), 1.0, delta=1e-9)
                self.assertAlmostEqual(self._dot(x_axis, y_axis), 0.0, delta=1e-9)
                for got, want in zip(self._cross(x_axis, y_axis), normal):
                    self.assertAlmostEqual(got, want, delta=1e-9)

                # The accessors must describe the actual curve: every sampled
                # point is exactly `radius` from `center` and lies in the plane
                # through `center` with `normal`.
                tmin, tmax = circ.parameter_range()
                self.assertTrue(math.isfinite(tmin) and math.isfinite(tmax))
                for i in range(41):
                    t = tmin + (tmax - tmin) * i / 40.0
                    p = circ.evaluate(t)
                    self.assertTrue(all(math.isfinite(c) for c in p))
                    rel = tuple(p[k] - center[k] for k in range(3))
                    self.assertAlmostEqual(self._norm(rel), radius,
                                           delta=1e-7 + 1e-6 * radius)
                    self.assertAlmostEqual(self._dot(rel, normal), 0.0,
                                           delta=1e-7 + 1e-6 * radius)

                # Analytic radius agrees with measured arc length.
                self.assertAlmostEqual(circ.length(),
                                       2.0 * math.pi * radius,
                                       delta=1e-6 + 1e-4 * radius)

    def test_create_circle_is_deterministic(self):
        a = sm.create_circle((1, 2, 3), 5)
        b = sm.create_circle((1, 2, 3), 5)
        self.assertEqual(a.center(), b.center())
        self.assertEqual(a.radius(), b.radius())
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            self.assertEqual(a.evaluate(t), b.evaluate(t))

    def test_create_circle_invalid_inputs(self):
        # Every hostile input must raise RuntimeError(SM_ERR_INVALID_INPUT)
        # with no crash/hang and no degenerate geometry. (Before the guard,
        # NaN/Inf center and Inf radius silently returned a bad circle.)
        inf, nan = float("inf"), float("nan")
        big = sys.float_info.max  # exceeds SM_IS_VALID_DOUBLE's finite window
        bad = []
        for coord in range(3):
            for val in (nan, inf, -inf, big):
                c = [0.0, 0.0, 0.0]
                c[coord] = val
                bad.append(("center[%d]=%r" % (coord, val), tuple(c), 5.0))
        for r in (0.0, -1.0, -1e-9, nan, inf, -inf, big):
            bad.append(("radius=%r" % r, (0.0, 0.0, 0.0), r))
        # Radius at or below SM_EFF_ZERO (1e-12): SmCircle::CreateCanonical
        # requires radius > SM_EFF_ZERO, so these are invalid input rather than
        # a generic kernel SM_ERR.
        for r in (1e-12, 5e-13):
            bad.append(("radius_floor=%r" % r, (0.0, 0.0, 0.0), r))
        # Large-center / small-radius: the derived extents collapse onto the
        # center in double precision (1e18 + 1 == 1e18), which would yield a
        # radius()==1 circle with length()==0.
        bad.append(("big_center_x", (1e18, 0.0, 0.0), 1.0))
        bad.append(("big_center_x_tiny_r", (1e18, 0.0, 0.0), 1e-6))
        bad.append(("big_center_y", (0.0, 1e18, 0.0), 1.0))
        for desc, center, radius in bad:
            with self.subTest(case=desc):
                with self.assertRaises(RuntimeError) as ctx:
                    sm.create_circle(center, radius)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))

    def test_create_circle_boundary_radii_accepted(self):
        # Finite positive radii across a wide range are accepted.
        for radius in (1e-3, 1.0, 1e3, 1e6):
            with self.subTest(radius=radius):
                circ = sm.create_circle((0, 0, 0), radius)
                self.assertAlmostEqual(circ.radius(), radius,
                                       delta=1e-9 * radius + 1e-12)

    def test_create_circle_minimum_radius_boundary(self):
        # Exact boundary: radius == SM_EFF_ZERO (1e-12) is rejected, but a
        # radius just above the floor at a small center is still accepted (the
        # guard must not over-reject valid small circles).
        with self.assertRaises(RuntimeError) as ctx:
            sm.create_circle((0, 0, 0), 1e-12)
        self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
        circ = sm.create_circle((0, 0, 0), 2e-12)
        self.assertIsInstance(circ, sm.Circle)

    def test_create_arc(self):
        arc = sm.create_arc((0, 0, 0), 5, 0, 180)
        self.assertIsInstance(arc, sm.Curve)

    @staticmethod
    def _normalize(v):
        n = math.sqrt(sum(c * c for c in v))
        return tuple(c / n for c in v)

    def _assert_helix_endpoint_tangents(self, h, r0, r1, height, turns, right_handed):
        # The endpoint tangent direction must match the analytic helix
        # derivative dP/dtheta, not just the endpoint position. For integer
        # ``turns`` (cos(2*pi*turns)=1, sin=0) the derivative at an endpoint is
        # (Rdfun, rl*r_end, height/(2*pi*turns)), with Rdfun = (r1-r0)/(2*pi*
        # turns) the radial taper rate and rl = +1 (right) / -1 (left). This is
        # the assertion that catches a corrupted tangent (e.g. normalizing then
        # scaling XY and Z by different factors), which position checks miss.
        two_pi_turns = 2.0 * math.pi * turns
        rdfun = (r1 - r0) / two_pi_turns
        rl = 1.0 if right_handed else -1.0
        a, b = h.parameter_range()
        start_dir = self._normalize((rdfun, rl * r0, height / two_pi_turns))
        end_dir = self._normalize((rdfun, rl * r1, height / two_pi_turns))
        got_start = self._normalize(sm.evaluate_curve(h, a)["tangent"])
        got_end = self._normalize(sm.evaluate_curve(h, b)["tangent"])
        for got, want in zip(got_start, start_dir):
            self.assertAlmostEqual(got, want, places=3)
        for got, want in zip(got_end, end_dir):
            self.assertAlmostEqual(got, want, places=3)

    def test_create_helix_constant_radius(self):
        # Right-handed helix: R=2, height=10, 5 turns, centered on +Z at origin.
        h = sm.create_helix((0, 0, 0), 10.0, 2.0, 2.0, 5.0)
        self.assertIsInstance(h, sm.BSplineCurve)
        self.assertIsInstance(h, sm.Curve)
        a, b = h.parameter_range()
        # Starts on +X at the base, ends on +X at the top after whole turns.
        for got, want in zip(h.evaluate(a), (2.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, want, places=4)
        for got, want in zip(h.evaluate(b), (2.0, 0.0, 10.0)):
            self.assertAlmostEqual(got, want, places=4)
        # Arc length ~ sqrt((2*pi*R*turns)**2 + height**2).
        expected = math.hypot(2 * math.pi * 2.0 * 5.0, 10.0)
        self.assertAlmostEqual(h.length(), expected, delta=0.5)
        self._assert_helix_endpoint_tangents(h, 2.0, 2.0, 10.0, 5.0, True)

    def test_create_helix_conical(self):
        # radius_start != radius_end -> tapered (conical) helix; offset origin.
        h = sm.create_helix((1, 2, 3), 10.0, 1.0, 3.0, 4.0)
        self.assertIsInstance(h, sm.BSplineCurve)
        a, b = h.parameter_range()
        for got, want in zip(h.evaluate(a), (2.0, 2.0, 3.0)):
            self.assertAlmostEqual(got, want, places=4)
        for got, want in zip(h.evaluate(b), (4.0, 2.0, 13.0)):
            self.assertAlmostEqual(got, want, places=4)
        self._assert_helix_endpoint_tangents(h, 1.0, 3.0, 10.0, 4.0, True)

    def test_create_helix_left_handed(self):
        # Handedness flips winding about +Z: right winds toward +Y just past the
        # start, left winds toward -Y.
        rh = sm.create_helix((0, 0, 0), 10.0, 2.0, 2.0, 1.0, right_handed=True)
        lh = sm.create_helix((0, 0, 0), 10.0, 2.0, 2.0, 1.0, right_handed=False)
        a, b = rh.parameter_range()
        t = a + 0.1 * (b - a)
        self.assertGreater(rh.evaluate(t)[1], 0.0)
        self.assertLess(lh.evaluate(t)[1], 0.0)
        self._assert_helix_endpoint_tangents(rh, 2.0, 2.0, 10.0, 1.0, True)
        self._assert_helix_endpoint_tangents(lh, 2.0, 2.0, 10.0, 1.0, False)

    def _create_helix(self, case):
        return sm.create_helix(
            tuple(case["origin"]), case["height"], case["radius_start"],
            case["radius_end"], case["turns"],
            right_handed=case["right_handed"], tolerance=case["tolerance"])

    def _helix_rejected_as_invalid(self, case):
        # True iff create_helix raised SM_ERR_INVALID_INPUT; False if it
        # succeeded or failed some other way (e.g. SM_ERR_NOT_WITHIN_TOLERANCE).
        try:
            self._create_helix(case)
            return False
        except RuntimeError as e:
            return "SM_ERR_INVALID_INPUT" in str(e)

    def test_create_helix_invalid_inputs_matrix(self):
        # Full invalid-input matrix, run in-process. Every hostile input --
        # non-finite origin/scalars, zero/negative/subnormal/out-of-range
        # magnitudes, the zero-radius contract, bad tolerances, and turn counts
        # beyond the fitter's control-point cap -- must be rejected with
        # SM_ERR_INVALID_INPUT and return no curve. These historically could
        # abort/hang the legacy fitter; SmApiCreateHelix now validates them
        # up front, before the fitter runs. (If a future change reintroduces a
        # crash on one of these, fix the validation.)
        cases = _helix_invalid_cases()
        self.assertGreater(len(cases), 0)
        for label, case in cases:
            with self.subTest(case=label):
                with self.assertRaises(RuntimeError) as cm:
                    self._create_helix(case)
                self.assertIn("SM_ERR_INVALID_INPUT", str(cm.exception))

    def test_create_helix_boundary_acceptance(self):
        # Inclusive boundaries are accepted by validation (they may still fail
        # the tolerance contract, which is a different, non-INVALID_INPUT
        # outcome); values just outside are rejected. A loose tolerance keeps
        # the accepted boundary fits fast and successful.
        inf = float("inf")
        accepted = [
            ("turns=min", _helix_case(turns=_HELIX_MIN_TURNS, tolerance=1.0e-2)),
            ("turns=max", _helix_case(turns=_HELIX_MAX_TURNS, tolerance=1.0e-1)),
            ("height=min", _helix_case(height=_HELIX_MIN_H, tolerance=1.0e-2)),
            ("radius=min", _helix_case(radius_start=_HELIX_MIN_R,
                                       radius_end=_HELIX_MIN_R, tolerance=1.0e-2)),
            ("tolerance=min", _helix_case(tolerance=_HELIX_MIN_TOL)),
        ]
        for label, case in accepted:
            with self.subTest(accepted=label):
                self.assertFalse(
                    self._helix_rejected_as_invalid(case),
                    "boundary %s should not be rejected as invalid input" % label)
        rejected = [
            ("turns=just_above_max",
             _helix_case(turns=math.nextafter(_HELIX_MAX_TURNS, inf))),
            ("turns=just_below_min",
             _helix_case(turns=math.nextafter(_HELIX_MIN_TURNS, 0.0))),
            ("tolerance=just_below_min",
             _helix_case(tolerance=math.nextafter(_HELIX_MIN_TOL, 0.0))),
        ]
        for label, case in rejected:
            with self.subTest(rejected=label):
                self.assertTrue(
                    self._helix_rejected_as_invalid(case),
                    "boundary %s should be rejected as invalid input" % label)

    def _max_helix_3d_dev(self, h, origin, height, r0, r1, turns, right_handed, nsamp=8000):
        # Dense, independent full-3D deviation of the fitted curve from the
        # analytic helix: pair each curve point with the helix point at the same
        # axial height (mirrors the C++ contract check). Captures radial and
        # angular error, unlike a position-only endpoint check.
        a, b = h.parameter_range()
        two = 2.0 * math.pi * turns
        rl = 1.0 if right_handed else -1.0
        dmax = 0.0
        for i in range(nsamp + 1):
            t = a + (b - a) * i / nsamp
            c = h.evaluate(t)
            th = (c[2] - origin[2]) * two / height
            th = max(0.0, min(two, th))
            r = (r0 * (two - th) + r1 * th) / two
            d = (origin[0] + r * math.cos(th),
                 origin[1] + rl * r * math.sin(th),
                 origin[2] + height * th / two)
            dev = math.dist(c, d)
            if dev > dmax:
                dmax = dev
        return dmax

    def test_create_helix_tolerance_is_enforced(self):
        # tolerance is a hard contract, not best-effort. The minimum supported
        # tolerance succeeds AND the returned curve is genuinely within it under
        # a dense full-3D comparison with the analytic helix.
        tol = 1e-6
        h = sm.create_helix((0, 0, 0), 10.0, 2.0, 2.0, 5.0, tolerance=tol)
        self.assertLessEqual(
            self._max_helix_3d_dev(h, (0, 0, 0), 10.0, 2.0, 2.0, 5.0, True), tol)
        # A looser request on a tapered helix is also honored within its bound.
        tol2 = 1e-4
        h2 = sm.create_helix((1, 2, 3), 10.0, 1.0, 3.0, 4.0, tolerance=tol2)
        self.assertLessEqual(
            self._max_helix_3d_dev(h2, (1, 2, 3), 10.0, 1.0, 3.0, 4.0, True), tol2)
        # An unattainable request must fail with no curve, not silently return an
        # out-of-tolerance approximation: 120 turns at 1e-6 overruns the fitter's
        # ~994-span control-point cap (true deviation ~2e-3 >> 1e-6).
        with self.assertRaisesRegex(RuntimeError, "SM_ERR_NOT_WITHIN_TOLERANCE"):
            sm.create_helix((0, 0, 0), 10.0, 2.0, 2.0, 120.0, tolerance=1e-6)

    def test_create_curve_from_points(self):
        pts = [(0, 0, 0), (3, 5, 0), (6, 2, 0), (10, 4, 0)]
        crv = sm.create_curve(pts)
        self.assertIsInstance(crv, sm.Curve)

    def test_remove_curve_knots_preserves_shape_and_handle(self):
        for remove_knots in (sm.remove_curve_knots, sm.curves.remove_curve_knots):
            with self.subTest(function=remove_knots):
                # Removing this fixture's sole interior knot straightens its tiny kink.
                curve = sm.create_canonical_curve(
                    [(0, 0, 0), (1, 1e-7, 0), (2, 0, 0)],
                    [0, 0.5, 1], [2, 1, 2], degree=1)
                parameters = [i / 20 for i in range(21)]
                before = [curve.evaluate(t) for t in parameters]

                self.assertIs(remove_knots(curve), curve)

                # Removing the interior knot straightens the tiny kink.
                self.assertAlmostEqual(curve.evaluate(0.5)[1], 0.0, delta=1e-12)
                for t, point in zip(parameters, before, strict=True):
                    self.assertLessEqual(math.dist(curve.evaluate(t), point), 1e-7 + 1e-12)

    def test_create_line(self):
        line = sm.create_line((0, 0, 0), (1, 0, 0))
        self.assertIsInstance(line, sm.Curve)

    def test_create_ellipse(self):
        ell = sm.create_ellipse((0, 0, 0), 10, 5)
        self.assertIsInstance(ell, sm.Curve)

    def test_create_rectangle(self):
        segs = sm.create_rectangle((0, 0, 0), 10, 5)
        self.assertIsInstance(segs, list)
        self.assertEqual(len(segs), 4)

    def test_create_regular_polygon(self):
        segs = sm.create_regular_polygon((0, 0, 0), 6, 5)
        self.assertIsInstance(segs, list)
        self.assertEqual(len(segs), 6)

    def test_create_curve_approx_points(self):
        pts = [(0, 0, 0), (2, 3, 0), (5, 1, 0), (8, 4, 0), (10, 0, 0)]
        crv = sm.create_curve_approx_points(pts)
        self.assertIsInstance(crv, sm.Curve)

    def test_create_curve_interp_points(self):
        pts = [(0, 0, 0), (2, 3, 0), (5, 1, 0), (8, 4, 0), (10, 0, 0)]
        crv = sm.create_curve_interp_points(pts)
        self.assertIsInstance(crv, sm.Curve)

    _PARAMS = (sm.CurveParameterization.UNIFORM,
               sm.CurveParameterization.CHORD_LENGTH,
               sm.CurveParameterization.CENTRIPETAL)

    def _dist_point_to_curve(self, crv, pt):
        _, _, dists = sm.curve_closest_point_all(crv, pt)
        return min(dists) if dists else float("inf")

    def test_create_curve_interp_points_parameterization_interpolates(self):
        # Unevenly spaced points (a cluster then a jump) — the case where the
        # parameterization actually matters. Every parameterization must
        # interpolate (pass through) EVERY input point, not just the endpoints.
        pts = [(0, 0, 0), (1, 0.2, 0), (1.5, 0.1, 0), (2, 3, 0),
               (9, 3.5, 0), (10, 0, 0)]
        for param in self._PARAMS:
            with self.subTest(parameterization=param):
                crv = sm.create_curve_interp_points(pts, parameterization=param)
                self.assertIsInstance(crv, sm.Curve)
                tmin, tmax = crv.parameter_range()
                self.assertTrue(math.isfinite(tmin) and math.isfinite(tmax))
                # Endpoints are exact.
                for got, want in zip(crv.start_point(), pts[0]):
                    self.assertAlmostEqual(got, want, delta=1e-6)
                for got, want in zip(crv.end_point(), pts[-1]):
                    self.assertAlmostEqual(got, want, delta=1e-6)
                # Every interior point lies on the curve.
                for pt in pts:
                    self.assertLess(self._dist_point_to_curve(crv, pt), 1e-5)
                # Sampled curve stays finite and free of the SMLib "infinite"
                # domain sentinel (+/-1234567.0), well within a sane envelope.
                for i in range(51):
                    t = tmin + (tmax - tmin) * i / 50.0
                    p = crv.evaluate(t)
                    for c in p:
                        self.assertTrue(math.isfinite(c))
                        self.assertNotAlmostEqual(abs(c), 1234567.0, delta=1e-3)
                        self.assertLess(abs(c), 1e3)

    def test_create_curve_interp_points_parameterization_has_effect(self):
        # On uneven data the three parameterizations must produce measurably
        # different curves; otherwise the argument would be a silent no-op.
        pts = [(0, 0, 0), (1, 0.2, 0), (1.5, 0.1, 0), (2, 3, 0),
               (9, 3.5, 0), (10, 0, 0)]
        curves = {p: sm.create_curve_interp_points(pts, parameterization=p)
                  for p in self._PARAMS}

        def max_gap(c0, c1):
            g = 0.0
            for i in range(101):
                t = i / 100.0
                a, b = c0.evaluate(t), c1.evaluate(t)
                g = max(g, self._norm(tuple(a[k] - b[k] for k in range(3))))
            return g

        uni = curves[sm.CurveParameterization.UNIFORM]
        chord = curves[sm.CurveParameterization.CHORD_LENGTH]
        centr = curves[sm.CurveParameterization.CENTRIPETAL]
        self.assertGreater(max_gap(uni, chord), 1e-4)
        self.assertGreater(max_gap(uni, centr), 1e-4)
        # chord-length and centripetal weight distance differently, so they
        # differ from each other too.
        self.assertGreater(max_gap(chord, centr), 1e-6)

    def test_create_curve_interp_points_deterministic(self):
        pts = [(0, 0, 0), (2, 3, 0), (5, 1, 0), (8, 4, 0), (10, 0, 0)]
        for param in self._PARAMS:
            with self.subTest(parameterization=param):
                a = sm.create_curve_interp_points(pts, parameterization=param)
                b = sm.create_curve_interp_points(pts, parameterization=param)
                for t in (0.0, 0.3, 0.5, 0.7, 1.0):
                    self.assertEqual(a.evaluate(t), b.evaluate(t))

    def test_create_curve_interp_points_keyword_only(self):
        pts = [(0, 0, 0), (2, 3, 0), (5, 1, 0), (8, 4, 0)]
        with self.assertRaises(TypeError):
            sm.create_curve_interp_points(pts, sm.CurveParameterization.CENTRIPETAL)

    def test_create_curve_interp_points_minimum_count(self):
        # Fewer than 4 points is rejected; exactly 4 is accepted.
        for n in (0, 1, 2, 3):
            pts = [(float(i), float(i % 2), 0.0) for i in range(n)]
            with self.subTest(n=n):
                with self.assertRaises(RuntimeError) as ctx:
                    sm.create_curve_interp_points(pts)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
        crv = sm.create_curve_interp_points(
            [(0, 0, 0), (1, 1, 0), (2, 0, 0), (3, 1, 0)])
        self.assertIsInstance(crv, sm.Curve)

    def test_create_curve_interp_points_invalid_inputs(self):
        # Non-finite coordinates in any position, for any parameterization,
        # must raise RuntimeError(SM_ERR_INVALID_INPUT) with no crash/hang.
        inf, nan = float("inf"), float("nan")
        big = sys.float_info.max
        base = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0),
                (2.0, 0.0, 0.0), (3.0, 1.0, 0.0)]
        cases = []
        for idx in (0, len(base) - 1, 1):        # first, last, interior
            for coord in range(3):
                for val in (nan, inf, -inf, big):
                    pts = [list(p) for p in base]
                    pts[idx][coord] = val
                    cases.append(("pt%d[%d]=%r" % (idx, coord, val),
                                  [tuple(p) for p in pts]))
        for desc, pts in cases:
            with self.subTest(case=desc):
                with self.assertRaises(RuntimeError) as ctx:
                    sm.create_curve_interp_points(pts)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
                # Non-default parameterization takes the same validation path.
                with self.assertRaises(RuntimeError) as ctx2:
                    sm.create_curve_interp_points(
                        pts, parameterization=sm.CurveParameterization.CENTRIPETAL)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx2.exception))

    def test_create_curve_interp_points_unknown_parameterization(self):
        # An above-range CurveParameterization (constructed from an integer
        # outside the three defined values) must raise
        # RuntimeError(SM_ERR_INVALID_INPUT). pybind11 lets CurveParameterization(999)
        # succeed, and the kernel would otherwise silently map it to UNIFORM, so
        # the SM_API boundary rejects unknown values instead. (pybind11 rejects
        # negative values at construction, so below-range is covered by the C++
        # tests via raw casts.)
        pts = [(0, 0, 0), (1, 1, 0), (2, 0, 0), (3, 1, 0)]
        for bad in (3, 999):
            with self.subTest(value=bad):
                with self.assertRaises(RuntimeError) as ctx:
                    sm.create_curve_interp_points(
                        pts, parameterization=sm.CurveParameterization(bad))
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))

    def test_create_curve_interp_points_coincident_points(self):
        # Distance-based modes reject coincident successive points as
        # SM_ERR_INVALID_INPUT; UNIFORM still accepts the same input.
        one_dup = [(0, 0, 0), (1, 1, 0), (1, 1, 0), (3, 1, 0)]
        all_same = [(2, 2, 2)] * 4
        distance_based = (sm.CurveParameterization.CHORD_LENGTH,
                          sm.CurveParameterization.CENTRIPETAL)
        for param in distance_based:
            for desc, pts in (("one_dup", one_dup), ("all_coincident", all_same)):
                with self.subTest(parameterization=param, case=desc):
                    with self.assertRaises(RuntimeError) as ctx:
                        sm.create_curve_interp_points(pts, parameterization=param)
                    self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))

        # UNIFORM must not over-reject the duplicate.
        crv = sm.create_curve_interp_points(
            one_dup, parameterization=sm.CurveParameterization.UNIFORM)
        self.assertIsInstance(crv, sm.Curve)

    def test_evaluate_curve(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        result = sm.evaluate_curve(line, 0.5)
        self.assertIsInstance(result, dict)

    def test_evaluate_continuity(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        ct = sm.evaluate_continuity(line, 0.5)
        self.assertIsInstance(ct, sm.ContinuityType)

    def test_evaluate_continuity_returns_registered_member(self):
        # The kernel reports G2/G3 variants (e.g. C1_G2_G3 inside a circle);
        # every value it can return must map to a named ContinuityType.
        circle = sm.create_circle((0, 0, 0), 5)
        lo, hi = circle.parameter_range()
        ct = sm.evaluate_continuity(circle, 0.5 * (lo + hi))
        self.assertIn(ct.name, sm.ContinuityType.__members__)

    def test_evaluate_continuity_rejects_invalid_inputs(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        lo, hi = line.parameter_range()
        for param in (float("nan"), float("inf"), -float("inf"), lo - 1, hi + 1):
            with self.subTest(parameter=param):
                with self.assertRaisesRegex(RuntimeError, "SM_ERR_INVALID_INPUT"):
                    sm.evaluate_continuity(line, param)
        with self.assertRaisesRegex(RuntimeError, "SM_ERR_INVALID_INPUT"):
            sm.evaluate_continuity(None, 0.5 * (lo + hi))
        for param in (lo, 0.5 * (lo + hi), hi):
            with self.subTest(valid_parameter=param):
                self.assertEqual(sm.evaluate_continuity(line, param), sm.ContinuityType.C1_C2_C3)

    def test_continuity_enum_covers_kernel_values(self):
        # Keep every SmContinuityType value named, including the mixed classes.
        names = (
            "UNDEFINED", "DISCONTINUOUS", "C0", "G1", "G1R", "G1_G2", "G1_G2_G3",
            "C1", "C1_G2", "C1_G2_G3", "C1_C2", "C1_C2_G3", "C1_C2_C3", "C_INFINITY",
        )
        for value, name in enumerate(names):
            with self.subTest(name=name):
                self.assertEqual(int(getattr(sm.ContinuityType, name)), value)
                self.assertEqual(sm.ContinuityType(value).name, name)

    def test_convert_to_lines_and_arcs(self):
        pts = [(0, 0, 0), (3, 5, 0), (6, 2, 0), (10, 4, 0)]
        crv = sm.create_curve(pts)
        segments = sm.convert_to_lines_and_arcs(crv)
        self.assertIsInstance(segments, list)
        # A wiggly (non-linear, non-arc) spline must yield at least one segment.
        self.assertGreater(len(segments), 0)

        # A line, an arc, or a curve below the fit tolerance comes back as one
        # copy; a degenerate curve (a point) raises.
        for crv in (sm.create_line_segment((0, 0, 0), (10, 0, 0)),
                    sm.create_arc((0, 0, 0), 5.0, 0.0, 90.0),
                    sm.create_curve([(0, 0, 0), (1e-4, 0, 0), (2e-4, 1e-4, 0), (3e-4, 0, 0)])):
            self.assertEqual(len(sm.convert_to_lines_and_arcs(crv)), 1)
        with self.assertRaisesRegex(RuntimeError, "SM_ERR_INVALID_INPUT"):
            sm.convert_to_lines_and_arcs(sm.create_curve([(1, 1, 1)] * 4))


# -----------------------------------------------------------------------
#  Surfaces
# -----------------------------------------------------------------------
class TestSurfaces(unittest.TestCase):
    def test_create_surface_from_points(self):
        pts = [
            (0, 0, 0),  (5, 0, 0),  (10, 0, 0),  (15, 0, 0),
            (0, 5, 1),  (5, 5, 2),  (10, 5, 2),  (15, 5, 1),
            (0, 10, 1), (5, 10, 2), (10, 10, 2), (15, 10, 1),
            (0, 15, 0), (5, 15, 0), (10, 15, 0), (15, 15, 0),
        ]
        srf = sm.create_surface_from_points(pts, 4, 4)
        self.assertIsInstance(srf, sm.Surface)

    def test_create_ruled_surface(self):
        arc1 = sm.create_arc((0, 0, 0), 5, 0, 180)
        arc2 = sm.create_arc((0, 0, 10), 5, 0, 180)
        srf = sm.create_ruled_surface(arc1, arc2)
        self.assertIsInstance(srf, sm.Surface)

    def test_create_surface_revolution(self):
        arc = sm.create_arc((5, 0, 0), 2, 0, 180)
        srf = sm.create_surface_revolution(arc, (0, 0, 0), (0, 0, 1), 360)
        self.assertIsInstance(srf, sm.Surface)

    def test_create_surface_from_corner_points(self):
        srf = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        self.assertIsInstance(srf, sm.Surface)

    def test_create_surface_from_ordered_points(self):
        pts = [
            (0, 0, 0), (5, 0, 0), (10, 0, 0), (15, 0, 0),
            (0, 5, 0), (5, 5, 1), (10, 5, 1), (15, 5, 0),
            (0, 10, 0), (5, 10, 1), (10, 10, 1), (15, 10, 0),
            (0, 15, 0), (5, 15, 0), (10, 15, 0), (15, 15, 0),
        ]
        srf = sm.create_surface_from_ordered_points(pts, 4, 4)
        self.assertIsInstance(srf, sm.Surface)

    def test_create_extrude_surface(self):
        arc = sm.create_arc((0, 0, 0), 5, 0, 90)
        srf = sm.create_extrude_surface(arc, (0, 0, 10))
        self.assertIsInstance(srf, sm.Surface)

    def test_create_sweep_surface(self):
        profile = sm.create_arc((0, 0, 0), 2, 0, 180)
        path = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        srf = sm.create_sweep_surface(profile, path)
        self.assertIsInstance(srf, sm.Surface)

    def test_create_skin_surface(self):
        c1 = sm.create_arc((0, 0, 0), 5, 0, 360)
        c2 = sm.create_arc((0, 0, 10), 3, 0, 360)
        srf = sm.create_skin_surface([c1, c2])
        self.assertIsInstance(srf, sm.Surface)

    def test_evaluate_surface_point(self):
        srf = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        pt = sm.evaluate_surface_point(srf, 0.5, 0.5)
        self.assertEqual(len(pt), 3)
        self.assertAlmostEqual(pt[0], 5.0, places=2)
        self.assertAlmostEqual(pt[1], 5.0, places=2)

    def test_evaluate_surface_normal(self):
        srf = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        nm = sm.evaluate_surface_normal(srf, 0.5, 0.5)
        self.assertEqual(len(nm), 3)

    def test_evaluate_surface_derivatives(self):
        srf = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        pt, du, dv = sm.evaluate_surface_derivatives(srf, 0.5, 0.5)
        self.assertEqual(len(pt), 3)
        self.assertEqual(len(du), 3)
        self.assertEqual(len(dv), 3)


# -----------------------------------------------------------------------
#  Booleans
# -----------------------------------------------------------------------
class TestBooleans(unittest.TestCase):
    def _two_boxes(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        return a, b

    def _assert_box_geometry(self, brep, origin=(0, 0, 0), dimensions=(10, 10, 10)):
        self.assertIsInstance(brep, sm.Brep)
        self.assertTrue(brep.is_manifold_solid())
        self.assertEqual(brep.face_count(), 6)
        self.assertEqual(brep.edge_count(), 12)
        self.assertEqual(brep.vertex_count(), 8)
        self._assert_volume(brep, dimensions[0] * dimensions[1] * dimensions[2])
        actual_min, actual_max = brep.bounding_box(tight=True)
        expected_max = tuple(origin[i] + dimensions[i] for i in range(3))
        for actual, expected in zip(actual_min, origin, strict=True):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(actual_max, expected_max, strict=True):
            self.assertAlmostEqual(actual, expected, places=6)

    def _assert_volume(self, brep, expected):
        actual = brep.volume(relative_accuracy=1.0e-4)
        tolerance = max(1.0, abs(expected)) * 2.0e-4
        self.assertLessEqual(abs(actual - expected), tolerance)

    def _assert_planar_area(self, brep, expected):
        face_areas = [face.area(relative_accuracy=1.0e-4) for face in brep.faces()]
        for area in face_areas:
            self.assertTrue(math.isfinite(area))
            self.assertGreater(area, 0.0)
        actual = math.fsum(face_areas)
        tolerance = max(1.0, abs(expected)) * 2.0e-4
        self.assertLessEqual(abs(actual - expected), tolerance)

    def _planar_rectangle(self, center, length=10, width=10):
        return sm.create_planar_faces(sm.create_rectangle(center, length, width))

    def _assert_rectangle_boolean_2d_cases(self, a_spec, b_spec, cases):
        for operation, expected_area, expected_bounds in cases:
            with self.subTest(operation=operation):
                # boolean_2d consumes both operands, so every case needs fresh inputs.
                a = self._planar_rectangle(*a_spec)
                b = self._planar_rectangle(*b_spec)
                result = sm.boolean_2d(a, b, operation=operation)
                self.assertIsInstance(result, sm.Brep)
                self._assert_planar_area(result, expected_area)
                if expected_bounds is None:
                    self.assertEqual(result.face_count(), 0)
                    continue
                self.assertGreater(result.face_count(), 0)
                actual_bounds = result.bounding_box(tight=True)
                for actual_point, expected_point in zip(actual_bounds, expected_bounds, strict=True):
                    for actual, expected in zip(actual_point, expected_point, strict=True):
                        self.assertAlmostEqual(actual, expected, places=6)

    def _assert_merge_invalid_input(self, breps):
        with self.assertRaises(RuntimeError) as raised:
            sm.merge_breps(breps, operation=sm.BooleanOp.UNION)
        message = str(raised.exception)
        self.assertIn("SmApiMergeBreps", message)
        self.assertIn("SM_ERR_INVALID_INPUT", message)

    def test_boolean_union(self):
        a, b = self._two_boxes()
        r = sm.boolean_union(a, b)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 6)

    def test_boolean_difference(self):
        a, b = self._two_boxes()
        r = sm.boolean_difference(a, b)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_boolean_intersection(self):
        a, b = self._two_boxes()
        r = sm.boolean_intersection(a, b)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_boolean_generic(self):
        a, b = self._two_boxes()
        r = sm.boolean(a, b, operation=sm.BooleanOp.UNION)
        self.assertIsInstance(r, sm.Brep)

    def test_merge_breps_empty_raises_invalid_input(self):
        self._assert_merge_invalid_input([])

    def test_merge_breps_singleton_fails_without_consuming_input(self):
        box = sm.create_box((1, 2, 3), 4, 5, 6)
        self._assert_merge_invalid_input([box])
        self._assert_box_geometry(box, origin=(1, 2, 3), dimensions=(4, 5, 6))

    def test_merge_breps_none_entry_fails_without_consuming_inputs(self):
        first = sm.create_box((0, 0, 0), 2, 3, 4)
        second = sm.create_box((10, 0, 0), 3, 4, 5)
        self._assert_merge_invalid_input([first, None, second])
        self._assert_box_geometry(first, dimensions=(2, 3, 4))
        self._assert_box_geometry(second, origin=(10, 0, 0), dimensions=(3, 4, 5))

    def test_merge_breps_duplicate_entry_fails_without_consuming_inputs(self):
        first = sm.create_box((0, 0, 0), 2, 3, 4)
        second = sm.create_box((10, 0, 0), 3, 4, 5)
        self._assert_merge_invalid_input([first, second, first])
        self._assert_box_geometry(first, dimensions=(2, 3, 4))
        self._assert_box_geometry(second, origin=(10, 0, 0), dimensions=(3, 4, 5))

    def test_merge_breps_operation_semantics(self):
        # Use unequal bodies so DIFFERENCE proves the legacy last-to-first
        # evaluation order: second - first has volume 144, not 952.
        # Fresh operands are required because successful calls consume inputs.
        expected_volumes = (
            (sm.BooleanOp.UNION, 1144.0),
            (sm.BooleanOp.INTERSECTION, 48.0),
            (sm.BooleanOp.DIFFERENCE, 144.0),
        )
        for operation, expected_volume in expected_volumes:
            with self.subTest(operation=operation):
                first = sm.create_box((0, 0, 0), 10, 10, 10)
                second = sm.create_box((8, 6, 3), 4, 8, 6)
                result = sm.merge_breps([first, second], operation=operation)
                self.assertIsInstance(result, sm.Brep)
                self.assertTrue(result.is_manifold_solid())
                self._assert_volume(result, expected_volume)
                self.assertGreater(result.face_count(), 0)
                self.assertGreater(result.edge_count(), 0)
                self.assertGreater(result.vertex_count(), 0)

        first = sm.create_box((0, 0, 0), 10, 10, 10)
        second = sm.create_box((8, 6, 3), 4, 8, 6)
        merged = sm.merge_breps([first, second], operation=sm.BooleanOp.MERGE)
        self.assertIsInstance(merged, sm.Brep)
        self.assertFalse(merged.is_manifold_solid())
        self.assertGreater(merged.face_count(), 0)
        self.assertGreater(merged.edge_count(), 0)
        self.assertGreater(merged.vertex_count(), 0)

    def test_merge_breps_three_input_union(self):
        boxes = [
            sm.create_box((0, 0, 0), 10, 10, 10),
            sm.create_box((5, 5, 5), 10, 10, 10),
            sm.create_box((8, 8, 8), 10, 10, 10),
        ]
        result = sm.merge_breps(boxes, operation=sm.BooleanOp.UNION)
        self.assertIsInstance(result, sm.Brep)
        self.assertTrue(result.is_manifold_solid())
        # Inclusion-exclusion: 3*1000 - (125 + 8 + 343) + 8.
        self._assert_volume(result, 2532.0)
        self.assertGreater(result.face_count(), 0)
        self.assertGreater(result.edge_count(), 0)
        self.assertGreater(result.vertex_count(), 0)

    def test_merge_breps_three_input_difference_preserves_legacy_order(self):
        breps = [
            sm.create_box((1, 1, 1.5), 2, 2, 2),
            sm.create_box((8, 6, 3), 4, 8, 6),
            sm.create_box((0, 0, 0), 10, 10, 10),
        ]
        result = sm.merge_breps(breps, operation=sm.BooleanOp.DIFFERENCE)
        self.assertIsInstance(result, sm.Brep)
        self.assertTrue(result.is_manifold_solid())
        # For input [C, B, A], the legacy fold is (A - B) - C = 1000 - 48 - 8.
        self._assert_volume(result, 944.0)
        actual_min, actual_max = result.bounding_box(tight=True)
        for actual, expected in zip(actual_min, (0.0, 0.0, 0.0), strict=True):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(actual_max, (10.0, 10.0, 10.0), strict=True):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertGreater(result.face_count(), 0)
        self.assertGreater(result.edge_count(), 0)
        self.assertGreater(result.vertex_count(), 0)

    def test_boolean_with_options_defaults(self):
        a, b = self._two_boxes()
        r = sm.boolean_with_options(a, b, operation=sm.BooleanOp.DIFFERENCE)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_boolean_with_options_cookie_cutter(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        r = sm.boolean_with_options(a, b, operation=sm.BooleanOp.DIFFERENCE,
                                    cookie_cutter=True)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_boolean_with_options_post_process(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_sphere((5, 5, 5), 4)
        r = sm.boolean_with_options(a, b, operation=sm.BooleanOp.UNION,
                                    post_process=True)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_boolean_with_options_keep_other_brep(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        r = sm.boolean_with_options(a, b, operation=sm.BooleanOp.DIFFERENCE,
                                    keep_other_brep=True)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(b.faces()), 0, "b should still be usable")

    def test_boolean_2d_union(self):
        a = sm.create_planar_circle((0, 0, 0), 10)
        b = sm.create_planar_circle((8, 0, 0), 10)
        r = sm.boolean_2d(a, b, operation=sm.BooleanOp2D.UNION)
        self.assertIsInstance(r, sm.Brep)
        intersection_area = 200 * math.acos(0.4) - 4 * math.sqrt(336)
        self._assert_planar_area(r, 200 * math.pi - intersection_area)

    def test_boolean_2d_noncoplanar_input_reports_invalid_input(self):
        a = self._planar_rectangle((0, 0, 0))
        b = self._planar_rectangle((0, 0, 1))
        with self.assertRaises(RuntimeError) as raised:
            sm.boolean_2d(a, b, operation=sm.BooleanOp2D.UNION)
        message = str(raised.exception)
        self.assertIn("SmApiBoolean2d", message)
        self.assertIn("SM_ERR_INVALID_INPUT", message)
        self.assertEqual(a.face_count(), 1)
        self.assertEqual(b.face_count(), 1)

    def test_boolean_2d_intersection(self):
        a = sm.create_planar_circle((0, 0, 0), 10)
        b = sm.create_planar_circle((8, 0, 0), 10)
        r = sm.boolean_2d(a, b, operation=sm.BooleanOp2D.INTERSECTION)
        self.assertIsInstance(r, sm.Brep)
        intersection_area = 200 * math.acos(0.4) - 4 * math.sqrt(336)
        self._assert_planar_area(r, intersection_area)

    def test_boolean_2d_difference(self):
        a = sm.create_planar_circle((0, 0, 0), 10)
        b = sm.create_planar_circle((8, 0, 0), 10)
        r = sm.boolean_2d(a, b, operation=sm.BooleanOp2D.DIFFERENCE)
        self.assertIsInstance(r, sm.Brep)
        intersection_area = 200 * math.acos(0.4) - 4 * math.sqrt(336)
        self._assert_planar_area(r, 100 * math.pi - intersection_area)

    def test_boolean_2d_exclusive_or(self):
        a = sm.create_planar_circle((0, 0, 0), 10)
        b = sm.create_planar_circle((8, 0, 0), 10)
        r = sm.boolean_2d(a, b, operation=sm.BooleanOp2D.EXCLUSIVE_OR)
        self.assertIsInstance(r, sm.Brep)
        intersection_area = 200 * math.acos(0.4) - 4 * math.sqrt(336)
        self._assert_planar_area(r, 200 * math.pi - 2 * intersection_area)

    def test_boolean_2d_partially_overlapping_rectangle_operation_areas(self):
        cases = (
            (sm.BooleanOp2D.UNION, 150.0, ((-5.0, -5.0, 0.0), (10.0, 5.0, 0.0))),
            (sm.BooleanOp2D.INTERSECTION, 50.0, ((0.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.DIFFERENCE, 50.0, ((-5.0, -5.0, 0.0), (0.0, 5.0, 0.0))),
            (sm.BooleanOp2D.EXCLUSIVE_OR, 100.0, ((-5.0, -5.0, 0.0), (10.0, 5.0, 0.0))),
            (sm.BooleanOp2D.MERGE, 150.0, ((-5.0, -5.0, 0.0), (10.0, 5.0, 0.0))),
        )
        self._assert_rectangle_boolean_2d_cases(((0, 0, 0),), ((5, 0, 0),), cases)

    def test_boolean_2d_disjoint_rectangle_operation_areas(self):
        cases = (
            (sm.BooleanOp2D.UNION, 200.0, ((-5.0, -5.0, 0.0), (20.0, 5.0, 0.0))),
            (sm.BooleanOp2D.INTERSECTION, 0.0, None),
            (sm.BooleanOp2D.DIFFERENCE, 100.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.EXCLUSIVE_OR, 200.0, ((-5.0, -5.0, 0.0), (20.0, 5.0, 0.0))),
            (sm.BooleanOp2D.MERGE, 200.0, ((-5.0, -5.0, 0.0), (20.0, 5.0, 0.0))),
        )
        self._assert_rectangle_boolean_2d_cases(((0, 0, 0),), ((15, 0, 0),), cases)

    def test_boolean_2d_first_rectangle_is_proper_subset_operation_areas(self):
        cases = (
            (sm.BooleanOp2D.UNION, 100.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.INTERSECTION, 16.0, ((-2.0, -2.0, 0.0), (2.0, 2.0, 0.0))),
            (sm.BooleanOp2D.DIFFERENCE, 0.0, None),
            (sm.BooleanOp2D.EXCLUSIVE_OR, 84.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.MERGE, 100.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
        )
        self._assert_rectangle_boolean_2d_cases(((0, 0, 0), 4, 4), ((0, 0, 0),), cases)

    def test_boolean_2d_second_rectangle_is_proper_subset_operation_areas(self):
        cases = (
            (sm.BooleanOp2D.UNION, 100.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.INTERSECTION, 16.0, ((-2.0, -2.0, 0.0), (2.0, 2.0, 0.0))),
            (sm.BooleanOp2D.DIFFERENCE, 84.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.EXCLUSIVE_OR, 84.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
            (sm.BooleanOp2D.MERGE, 100.0, ((-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))),
        )
        self._assert_rectangle_boolean_2d_cases(((0, 0, 0),), ((0, 0, 0), 4, 4), cases)

    def test_boolean_lists_union(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 0, 0), 10, 10, 10)
        breps, surfaces = sm.boolean_lists([a], [], [b], [], operation=1)
        self.assertIsInstance(breps, list)
        self.assertGreater(len(breps), 0)
        self.assertIsInstance(breps[0], sm.Brep)
        self.assertGreater(len(breps[0].faces()), 0)

    def test_boolean_lists_difference(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        breps, surfaces = sm.boolean_lists([a], [], [b], [], operation=3)
        self.assertIsInstance(breps, list)
        self.assertGreater(len(breps), 0)

    def test_evaluate_csg_tree(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 0, 0), 10, 10, 10)
        # Postfix: operand indices 0, 1, then operation 1 (OR/union)
        result = sm.evaluate_csg_tree([a, b], [0, 1, 1])
        self.assertIsInstance(result, list)
        self.assertGreater(len(result), 0)

    def test_boolean_merge(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((10, 0, 0), 10, 10, 10)
        r = sm.boolean_merge(a, b)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_boolean_with_curves(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        brep, edges = sm.boolean_with_curves(a, b, operation=sm.BooleanOp.DIFFERENCE)
        self.assertIsInstance(brep, sm.Brep)
        self.assertIsInstance(edges, list)

    def test_non_manifold_boolean(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        r = sm.non_manifold_boolean(a, b, operation=sm.BooleanOp.UNION)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_piecewise_merge(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        r = sm.piecewise_merge(a, b)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)


# -----------------------------------------------------------------------
#  Sweeps
# -----------------------------------------------------------------------
class TestSweeps(unittest.TestCase):
    _NON_MANIFOLD_SWEEPS = (
        (sm.non_manifold_linear_sweep, ((1, 1, 0), 7)),
        (sm.non_manifold_rotational_sweep, ((0, 0, 0), (0, 0, 1), 30)),
    )

    @staticmethod
    def _counts(brep):
        return (brep.face_count(), brep.edge_count(), brep.vertex_count())

    def test_non_manifold_sweeps_default_region(self):
        # Omitting region_brep used to pass a null region to DoSweep and crash. It now
        # resolves to the swept Brep's own infinite region, so naming that Brep
        # explicitly has to produce exactly the same topology.
        for sweep, args in self._NON_MANIFOLD_SWEEPS:
            with self.subTest(sweep=sweep.__name__):
                results = {}
                for region in ("omitted", "explicit"):
                    body = sm.create_box((-8, -8, -8), 16, 16, 16)
                    faces, edges, verts = self._counts(body)
                    kwargs = {} if region == "omitted" else {"region_brep": body}
                    sweep(body, *args, faces=[], edges=[], vertices=[body.vertices()[0]], **kwargs)
                    results[region] = self._counts(body)
                    self.assertEqual(results[region], (faces, edges + 1, verts + 1))
                self.assertEqual(results["omitted"], results["explicit"])

    # Omitting the selection entirely is the whole-Brep sweep, which takes the branch of
    # DoSweep that deletes and refreshes regions. The rotation axis has to miss the body:
    # revolving a solid through itself fails whether or not a region is supplied, which is
    # unrelated to the default region this test is about.
    _NON_MANIFOLD_WHOLE_BREP_SWEEPS = (
        (sm.non_manifold_linear_sweep, ((1, 1, 0), 7), (16, 30, 16)),
        (sm.non_manifold_rotational_sweep, ((20, 0, 0), (0, 0, 1), 30), (14, 28, 16)),
    )

    def test_non_manifold_whole_brep_sweeps_default_region(self):
        # The whole-Brep branch is the one most sensitive to the default region now being
        # the swept Brep's own infinite region, so cover it as well as a selected sweep.
        for sweep, args, expected in self._NON_MANIFOLD_WHOLE_BREP_SWEEPS:
            with self.subTest(sweep=sweep.__name__):
                results = {}
                for region in ("omitted", "explicit"):
                    body = sm.create_box((-8, -8, -8), 16, 16, 16)
                    kwargs = {} if region == "omitted" else {"region_brep": body}
                    sweep(body, *args, **kwargs)
                    results[region] = self._counts(body)
                    self.assertEqual(results[region], expected)
                self.assertEqual(results["omitted"], results["explicit"])

    def test_non_manifold_sweeps_reject_distinct_region_brep(self):
        # A different region_brep would receive the swept topology while brep_to_sweep is
        # what gets stitched, validated and returned, so the result would be coherently in
        # neither. Covered for a selected face and for a whole-Brep sweep.
        for sweep, args in self._NON_MANIFOLD_SWEEPS:
            for selection in ({"faces": None}, None):
                with self.subTest(sweep=sweep.__name__, whole_brep=selection is None):
                    body = sm.create_box((-8, -8, -8), 16, 16, 16)
                    target = sm.create_box((40, 0, 0), 16, 16, 16)
                    kwargs = {"faces": [body.faces()[0]]} if selection is not None else {}
                    before, target_before = self._counts(body), self._counts(target)
                    with self.assertRaises(ValueError):
                        sweep(body, *args, region_brep=target, **kwargs)
                    self.assertEqual(self._counts(body), before)
                    self.assertEqual(self._counts(target), target_before)

    def test_non_manifold_sweeps_reject_empty_selection(self):
        # Selecting nothing used to sweep nothing and still report success.
        for sweep, args in self._NON_MANIFOLD_SWEEPS:
            for selection in ({"faces": [], "edges": [], "vertices": []}, {"faces": []}):
                with self.subTest(sweep=sweep.__name__, selection=sorted(selection)):
                    body = sm.create_box((-8, -8, -8), 16, 16, 16)
                    before = self._counts(body)
                    with self.assertRaises(ValueError):
                        sweep(body, *args, **selection)
                    self.assertEqual(self._counts(body), before)

    def test_non_manifold_sweeps_reject_null_selection_entries(self):
        # A None entry became a null handle and was dereferenced during the sweep.
        for sweep, args in self._NON_MANIFOLD_SWEEPS:
            for kind in ("faces", "edges", "vertices"):
                with self.subTest(sweep=sweep.__name__, kind=kind):
                    body = sm.create_box((-8, -8, -8), 16, 16, 16)
                    before = self._counts(body)
                    selection = {"faces": [], "edges": [], "vertices": []}
                    selection[kind] = [None]
                    with self.assertRaises(ValueError):
                        sweep(body, *args, **selection)
                    self.assertEqual(self._counts(body), before)

    def test_non_manifold_sweeps_reject_foreign_selection_entries(self):
        # Topology owned by another Brep was swept into brep_to_sweep, silently
        # giving it geometry it never contained.
        for sweep, args in self._NON_MANIFOLD_SWEEPS:
            with self.subTest(sweep=sweep.__name__):
                body = sm.create_box((-8, -8, -8), 16, 16, 16)
                other = sm.create_box((100, 100, 100), 4, 4, 4)
                before, other_before = self._counts(body), self._counts(other)
                with self.assertRaises(RuntimeError):
                    sweep(body, *args, faces=[], edges=[], vertices=[other.vertices()[0]])
                self.assertEqual(self._counts(body), before)
                self.assertEqual(self._counts(other), other_before)

    @staticmethod
    def _curve_snapshot(curve):
        lo, hi = curve.parameter_range()
        return {
            "range": (lo, hi),
            "start_point": curve.evaluate(lo),
            "mid_point": curve.evaluate(0.5 * (lo + hi)),
            "end_point": curve.evaluate(hi),
            "length": curve.length(),
        }

    def _assert_curve_unchanged(self, curve, expected):
        lo, hi = curve.parameter_range()
        self.assertAlmostEqual(lo, expected["range"][0], places=9)
        self.assertAlmostEqual(hi, expected["range"][1], places=9)
        for point_name, parameter in (
                ("start_point", lo),
                ("mid_point", 0.5 * (lo + hi)),
                ("end_point", hi)):
            point = curve.evaluate(parameter)
            self.assertEqual(len(point), len(expected[point_name]))
            for got, want in zip(point, expected[point_name], strict=True):
                self.assertAlmostEqual(got, want, places=6)
        self.assertAlmostEqual(curve.length(), expected["length"], places=6)

    def test_linear_sweep(self):
        circ = sm.create_circle((0, 0, 0), 5)
        r = sm.linear_sweep([circ], (0, 0, 1), 10, cap_ends=True)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_rotational_sweep(self):
        line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
        r = sm.rotational_sweep([line], (0, 0, 0), (0, 0, 1), 360, cap_ends=True)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_draft_sweep(self):
        circ = sm.create_circle((0, 0, 0), 5)
        r = sm.draft_sweep([circ], 10, 5.0,
                           fillet_corners=False, cap_ends=sm.CapEnds.BOTH)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_pipe_sweep(self):
        pts = [(0, 0, 0), (5, 5, 0), (10, 0, 0), (15, 5, 0)]
        path = sm.create_curve(pts)
        r = sm.pipe_sweep(path, radius=1.0, cap_ends=True)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_pipe_sweep_helix_documentation_example(self):
        # Regression for the create_helix -> pipe_sweep example in curves.md:
        #   helix = sm.create_helix((0,0,0), height=10, radius_start=2,
        #                           radius_end=2, turns=5)
        #   spring = sm.pipe_sweep(helix, radius=0.3)
        # The swept spring must be a healthy three-face solid whose two circular
        # caps both report the true area pi*r^2. Previously the start cap had a
        # spurious zero area (reversed-orientation face.area() bug), so the
        # advertised documentation result was wrong. The generic pipe_sweep test
        # above never uses a helix and only checks that the face list is
        # non-empty, so this asserts the full geometric contract.
        R = 2.0            # helix radius
        HEIGHT = 10.0
        TURNS = 5.0
        PIPE_R = 0.3       # pipe (profile) radius
        SENTINEL = 1234567.0  # SM_INFINITE_PARAMETER: finite, so isfinite() misses it

        helix = sm.create_helix((0, 0, 0), height=HEIGHT, radius_start=R,
                                radius_end=R, turns=TURNS)
        spring = sm.pipe_sweep(helix, radius=PIPE_R)
        self.assertIsInstance(spring, sm.Brep)

        # (1) Exactly three faces (swept tube wall + two end caps) and a
        #     watertight manifold solid.
        faces = spring.faces()
        self.assertEqual(len(faces), 3)
        self.assertTrue(spring.is_manifold_solid())

        # (2) The two caps are the single-edge faces; both areas must be
        #     positive, equal, and near pi*r^2 (the start cap must not be zero).
        caps = [f for f in faces if len(f.edges()) == 1]
        self.assertEqual(len(caps), 2)
        expected_cap_area = math.pi * PIPE_R * PIPE_R
        cap_areas = sorted(f.area() for f in caps)
        for a in cap_areas:
            self.assertGreater(a, 0.0, "cap area must not be zero")
            self.assertAlmostEqual(a, expected_cap_area, delta=1e-3)
        self.assertAlmostEqual(cap_areas[0], cap_areas[1], delta=1e-3)

        # (3)+(4) Every face's stored trim domain and its backing-surface
        #     domain must be finite, ordered, and free of the finite
        #     infinite-parameter sentinel (+/-1234567.0).
        def check_domain(dom, what):
            (umin, vmin), (umax, vmax) = dom
            for val in (umin, vmin, umax, vmax):
                self.assertTrue(math.isfinite(val),
                                "%s: non-finite %r" % (what, val))
                self.assertGreater(abs(abs(val) - SENTINEL), 1.0,
                                   "%s: infinite-parameter sentinel %r" % (what, val))
            self.assertLess(umin, umax, "%s: u not ordered" % what)
            self.assertLess(vmin, vmax, "%s: v not ordered" % what)

        for i, f in enumerate(faces):
            check_domain(f.uv_domain(), "face[%d] trim domain" % i)
            check_domain(f.surface().uv_domain(), "face[%d] surface domain" % i)

        # (5) Each face has a finite, tight bounding box inside the
        #     helix-plus-pipe envelope: radially within R+PIPE_R of the +Z axis
        #     and axially within [0, HEIGHT] padded by the pipe radius.
        pad = PIPE_R + 1e-2
        xy_max = R + PIPE_R + 1e-2
        z_lo, z_hi = -pad, HEIGHT + pad
        for i, f in enumerate(faces):
            (bxmin, bymin, bzmin), (bxmax, bymax, bzmax) = f.bounding_box(tight=True)
            for val in (bxmin, bymin, bzmin, bxmax, bymax, bzmax):
                self.assertTrue(math.isfinite(val),
                                "face[%d] bbox non-finite %r" % (i, val))
            self.assertLessEqual(bxmin, bxmax)
            self.assertLessEqual(bymin, bymax)
            self.assertLessEqual(bzmin, bzmax)
            self.assertGreaterEqual(bxmin, -xy_max, "face[%d] bbox x below envelope" % i)
            self.assertLessEqual(bxmax, xy_max, "face[%d] bbox x above envelope" % i)
            self.assertGreaterEqual(bymin, -xy_max, "face[%d] bbox y below envelope" % i)
            self.assertLessEqual(bymax, xy_max, "face[%d] bbox y above envelope" % i)
            self.assertGreaterEqual(bzmin, z_lo, "face[%d] bbox z below envelope" % i)
            self.assertLessEqual(bzmax, z_hi, "face[%d] bbox z above envelope" % i)

    def test_pipe_sweep_low_turn_helix_no_pathological_topology(self):
        # Small-fraction helixes wind steeply (nearly straight). create_helix
        # still produces a valid curve, but such paths are hard to sweep. The
        # contract we lock in here is "no silent bad topology": for every small
        # fraction, in both constant-radius and tapered configurations,
        #   * create_helix either succeeds or raises a clean RuntimeError
        #     (SM_ERR_NOT_WITHIN_TOLERANCE when the tight default tolerance is
        #     unmeetable for the near-straight fit), and
        #   * pipe_sweep either raises a clean RuntimeError (near-straight path
        #     rejected) or returns exactly the same healthy 3-face manifold as a
        #     normal helix -- never the 245-face, V-domain-fragmented solid the
        #     reviewer observed on an earlier build.
        HEIGHT = 10.0
        PIPE_R = 0.05
        SENTINEL = 1234567.0  # SM_INFINITE_PARAMETER: finite, so isfinite() misses it
        # (radius_start, radius_end, turns) -- includes the reviewer's exact
        # cases (tapered 2->3 turns=0.01, constant turns=0.01).
        cases = [
            (2.0, 2.0, 0.01), (2.0, 2.0, 0.05), (2.0, 2.0, 0.25),
            (2.0, 3.0, 0.01), (2.0, 3.0, 0.05), (2.0, 3.0, 0.25),
        ]
        for r0, r1, turns in cases:
            with self.subTest(radius_start=r0, radius_end=r1, turns=turns):
                try:
                    helix = sm.create_helix((0, 0, 0), height=HEIGHT,
                                            radius_start=r0, radius_end=r1,
                                            turns=turns)
                except RuntimeError as exc:
                    # Only the tolerance contract may reject a valid small helix.
                    self.assertIn("SM_ERR_NOT_WITHIN_TOLERANCE", str(exc))
                    continue

                try:
                    solid = sm.pipe_sweep(helix, radius=PIPE_R)
                except RuntimeError:
                    # A near-straight path may be rejected outright -- acceptable,
                    # so long as it is a clean failure and not bad geometry.
                    continue

                # If the sweep succeeded it must be the healthy pipe topology:
                # a three-face manifold with two positive, equal circular caps.
                faces = solid.faces()
                self.assertEqual(len(faces), 3,
                                 "low-turn sweep produced fragmented topology")
                self.assertTrue(solid.is_manifold_solid())
                caps = [f for f in faces if len(f.edges()) == 1]
                self.assertEqual(len(caps), 2)
                expected_cap_area = math.pi * PIPE_R * PIPE_R
                cap_areas = sorted(f.area() for f in caps)
                for a in cap_areas:
                    self.assertGreater(a, 0.0)
                    self.assertAlmostEqual(a, expected_cap_area, delta=1e-4)
                self.assertAlmostEqual(cap_areas[0], cap_areas[1], delta=1e-4)
                # Domains must be finite, ordered, and sentinel-free (no
                # runaway ±1234567 / fragmented V ranges).
                for i, f in enumerate(faces):
                    for what, dom in (("trim", f.uv_domain()),
                                      ("surface", f.surface().uv_domain())):
                        (umin, vmin), (umax, vmax) = dom
                        for val in (umin, vmin, umax, vmax):
                            self.assertTrue(math.isfinite(val))
                            self.assertGreater(abs(abs(val) - SENTINEL), 1.0)
                        self.assertLess(umin, umax)
                        self.assertLess(vmin, vmax)

    def test_taper_extrude(self):
        circ = sm.create_circle((0, 0, 0), 5)
        r = sm.taper_extrude([circ], 10, 80.0, cap_ends=sm.CapEnds.BOTH)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_taper_extrude_rejects_end_caps_kwarg(self):
        circ = sm.create_circle((0, 0, 0), 5)
        with self.assertRaises(TypeError):
            sm.taper_extrude([circ], 10, 80.0, end_caps=sm.CapEnds.BOTH)

    def test_sweep_along_planar_path(self):
        profile = sm.create_circle((0, 0, 0), 1)
        path = sm.create_line_segment((0, 0, 0), (0, 0, 10))
        snapshots = [self._curve_snapshot(curve) for curve in (profile, path)]
        cap_cases = (
            (sm.CapEnds.NONE, False, False),
            (sm.CapEnds.START, True, False),
            (sm.CapEnds.END, False, True),
            (sm.CapEnds.BOTH, True, True),
        )

        for cap_ends, expect_start, expect_end in cap_cases:
            with self.subTest(cap_ends=cap_ends):
                result = sm.sweep_along_planar_path(
                    [profile], [path], move_profile=True, cap_ends=cap_ends)
                self.assertIsInstance(result, sm.Brep)
                self.assertEqual(
                    len(result.faces()), 1 + int(expect_start) + int(expect_end))

                start_cap_count = 0
                end_cap_count = 0
                side_count = 0
                for face in result.faces():
                    bounds_min, bounds_max = face.bounding_box(tight=True)
                    if abs(bounds_max[2] - bounds_min[2]) <= 1.0e-4:
                        cap_z = 0.5 * (bounds_min[2] + bounds_max[2])
                        if abs(cap_z) <= 1.0e-4:
                            start_cap_count += 1
                        elif abs(cap_z - 10.0) <= 1.0e-4:
                            end_cap_count += 1
                        else:
                            self.fail(f"unexpected z-flat face at z={cap_z}")
                    else:
                        side_count += 1

                self.assertEqual(start_cap_count, int(expect_start))
                self.assertEqual(end_cap_count, int(expect_end))
                self.assertEqual(side_count, 1)
                self.assertEqual(
                    result.is_manifold_solid(), expect_start and expect_end)
                volume = result.volume()
                self.assertTrue(math.isfinite(volume))
                if expect_start and expect_end:
                    self.assertAlmostEqual(volume, 10.0 * math.pi,
                                           delta=0.1 * math.pi)
                else:
                    self.assertAlmostEqual(volume, 0.0, delta=1.0e-8)

                for curve, snapshot in zip(
                        (profile, path), snapshots, strict=True):
                    self._assert_curve_unchanged(curve, snapshot)

    def test_connected_planar_path_classifies_bounded_region_as_material(self):
        profile = sm.create_circle((0, 0, 0), 1)
        paths = [
            sm.create_line_segment((0, 0, 0), (0, 0, 10)),
            sm.create_line_segment((0, 0, 10), (5, 0, 15)),
        ]
        curves = [profile, *paths]
        snapshots = [self._curve_snapshot(curve) for curve in curves]

        result = sm.sweep_along_planar_path(
            [profile], paths, move_profile=True,
            cap_ends=sm.CapEnds.BOTH)
        self.assertIsInstance(result, sm.Brep)
        self.assertEqual(len(result.faces()), 4)
        self.assertEqual(len(result.edges()), 5)
        self.assertEqual(len(result.vertices()), 3)
        self.assertTrue(result.is_manifold_solid())
        volume = result.volume()
        self.assertTrue(math.isfinite(volume))
        self.assertGreater(volume, 0.0)
        expected_volume = math.pi * (10.0 + math.sqrt(50.0))
        self.assertAlmostEqual(volume, expected_volume,
                               delta=expected_volume * 0.02)

        for curve, snapshot in zip(curves, snapshots, strict=True):
            self._assert_curve_unchanged(curve, snapshot)

    def test_planar_path_sweep_uses_nested_profile_parity_for_material(self):
        profiles = [
            sm.create_circle((0, 0, 0), 1),
            sm.create_circle((0, 0, 0), 3),
            sm.create_circle((0, 0, 0), 2),
        ]
        path = sm.create_line_segment((0, 0, 0), (0, 0, 10))
        curves = [*profiles, path]
        snapshots = [self._curve_snapshot(curve) for curve in curves]
        cap_cases = (
            (sm.CapEnds.NONE, 3, False),
            (sm.CapEnds.START, 5, False),
            (sm.CapEnds.END, 5, False),
            (sm.CapEnds.BOTH, 7, True),
        )

        for cap_ends, expected_faces, expect_solid in cap_cases:
            with self.subTest(cap_ends=cap_ends):
                result = sm.sweep_along_planar_path(
                    profiles, [path], move_profile=True,
                    cap_ends=cap_ends)
                self.assertEqual(len(result.faces()), expected_faces)
                self.assertEqual(result.is_manifold_solid(), expect_solid)

                volume = result.volume()
                self.assertTrue(math.isfinite(volume))
                if expect_solid:
                    # Even/odd nesting fills the radius-1 disk and the
                    # radius-2-to-3 annulus: area = pi * (1 + 9 - 4).
                    expected_volume = 60.0 * math.pi
                    self.assertAlmostEqual(volume, expected_volume,
                                           delta=expected_volume * 0.02)
                else:
                    self.assertAlmostEqual(volume, 0.0, delta=1.0e-8)

                for curve, snapshot in zip(curves, snapshots, strict=True):
                    self._assert_curve_unchanged(curve, snapshot)

    def test_planar_path_sweep_open_profile_stays_sheet_with_both_caps(self):
        profiles = [
            sm.create_line_segment((-1, -1, 0), (1, -1, 0)),
            sm.create_line_segment((1, -1, 0), (1, 1, 0)),
            sm.create_line_segment((1, 1, 0), (-1, 1, 0)),
        ]
        path = sm.create_line_segment((0, 0, 0), (0, 0, 10))
        curves = [*profiles, path]
        snapshots = [self._curve_snapshot(curve) for curve in curves]

        result = sm.sweep_along_planar_path(
            profiles, [path], move_profile=True, cap_ends=sm.CapEnds.BOTH)
        self.assertEqual(len(result.faces()), 3)
        self.assertFalse(result.is_manifold_solid())
        self.assertAlmostEqual(result.volume(), 0.0, delta=1.0e-8)

        for curve, snapshot in zip(curves, snapshots, strict=True):
            self._assert_curve_unchanged(curve, snapshot)

    def test_planar_path_sweep_tangent_line_arc_line_is_material(self):
        profile = sm.create_circle((0, 0, 0), 3)
        arc_start = (-16.3, 33.0, 62.5)
        arc_end = (-16.3, 37.0, 66.5)
        paths = [
            sm.create_line_segment((-16.3, 3.0, 62.5), arc_start),
            sm.create_canonical_curve(
                [arc_start, (-16.3, 37.0, 62.5), arc_end],
                [0.0, 1.0],
                [3, 3],
                [1.0, math.sqrt(0.5), 1.0],
                degree=2,
                form=sm.BSplineCurveForm.CIRCULAR_ARC,
            ),
            sm.create_line_segment(arc_end, (-16.3, 37.0, 69.5)),
        ]
        curves = [profile, *paths]
        snapshots = [self._curve_snapshot(curve) for curve in curves]

        result = sm.sweep_along_planar_path(
            [profile], paths, move_profile=True, cap_ends=sm.CapEnds.BOTH)
        self.assertEqual(len(result.faces()), 5)
        self.assertTrue(result.is_manifold_solid())
        volume = result.volume()
        self.assertTrue(math.isfinite(volume))
        expected_volume = math.pi * 3.0**2 * (33.0 + 2.0 * math.pi)
        self.assertAlmostEqual(volume, expected_volume,
                               delta=expected_volume * 0.01)

        for curve, snapshot in zip(curves, snapshots, strict=True):
            self._assert_curve_unchanged(curve, snapshot)

    def test_closed_planar_path_uses_mitered_transitions_and_ignores_caps(self):
        profile = sm.create_circle((0, 0, 0), 1)
        path_points = [
            (0, 0, 0),
            (10, 0, 0),
            (10, 10, 0),
            (0, 10, 0),
            (0, 0, 0),
        ]
        paths = [
            sm.create_line_segment(start, end)
            for start, end in zip(path_points[:-1], path_points[1:], strict=True)
        ]
        curves = [profile, *paths]
        snapshots = [self._curve_snapshot(curve) for curve in curves]
        results = []

        for cap_ends in (
                sm.CapEnds.NONE, sm.CapEnds.START,
                sm.CapEnds.END, sm.CapEnds.BOTH):
            with self.subTest(cap_ends=cap_ends):
                result = sm.sweep_along_planar_path(
                    [profile], paths, move_profile=True, cap_ends=cap_ends)
                self.assertEqual(len(result.faces()), 4)
                self.assertTrue(result.is_manifold_solid())
                volume = result.volume()
                self.assertTrue(math.isfinite(volume))
                self.assertAlmostEqual(volume, 40.0 * math.pi,
                                       delta=0.8 * math.pi)
                results.append(volume)

                for curve, snapshot in zip(curves, snapshots, strict=True):
                    self._assert_curve_unchanged(curve, snapshot)

        for volume in results[1:]:
            self.assertAlmostEqual(results[0], volume, delta=1.0e-8)

    def test_linear_sweep_with_repetitions(self):
        line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
        r = sm.linear_sweep_with_repetitions(
            [line], (1, 0, 0), 5, repetitions=2, cap_ends=False)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_rotational_sweep_with_repetitions(self):
        line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
        r = sm.rotational_sweep_with_repetitions(
            [line], (0, 0, 0), (0, 0, 1), 120, repetitions=3, cap_ends=False)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_curve_sweep(self):
        profile = sm.create_circle((0, 0, 0), 2)
        pts = [(0, 0, 0), (5, 5, 0), (10, 0, 0), (15, 5, 0)]
        path = sm.create_curve(pts)
        r = sm.curve_sweep([profile], path)
        self.assertIsInstance(r, sm.Brep)

    def test_curve_sweep_cap_ends(self):
        # Regression: a closed profile swept along a path must, by default,
        # produce a capped manifold solid rather than an open zero-volume
        # shell. cap_ends=False must still yield the open swept shell.
        radius = 2.0
        profile = sm.create_circle((0, 0, 0), radius)
        path = sm.create_curve([(0, 0, 0), (0, 0, 5), (0, 0, 10), (0, 0, 15)])

        solid = sm.curve_sweep([profile], path)  # cap_ends=True (default)
        self.assertIsInstance(solid, sm.Brep)
        self.assertTrue(solid.is_manifold_solid())
        self.assertGreater(solid.volume(), 0.0)
        faces = solid.faces()
        self.assertEqual(len(faces), 3)  # side wall + two planar caps
        # The two circular caps must carry positive area near pi * r^2
        # (spline-approximated circle, so allow a small relative tolerance).
        expected_cap = math.pi * radius * radius
        caps = [f for f in faces if abs(f.area() - expected_cap) < expected_cap * 0.02]
        self.assertEqual(len(caps), 2)

        open_shell = sm.curve_sweep([profile], path, cap_ends=False)
        self.assertIsInstance(open_shell, sm.Brep)
        self.assertFalse(open_shell.is_manifold_solid())


# -----------------------------------------------------------------------
#  Sweep / revolution inputs must survive the call (Pure: inputs unchanged)
# -----------------------------------------------------------------------
class TestSweepInputCurvesUnchanged(unittest.TestCase):
    """Regression for the consuming sweep / revolution kernel paths.

    ``SmPrimitiveCreation::CreateLinearSweep`` / ``CreateRotationalSweep`` /
    ``CreateTaperExtrude`` consume - and sometimes delete - the curves handed
    to them, stripping the internal NURB out of analytic inputs (an
    ``SmCircle`` survives with its NURB gone). The public API and these Python
    bindings document the operations as ``Pure: inputs unchanged``, so the
    wrappers sweep copies. Before the fix, touching the original curve's
    analytic accessors after any of these calls dereferenced freed state and
    crashed the process (SIGSEGV, exit 139)::

        c = sm.create_circle((0, 0, 0), 2)
        sm.linear_sweep([c], (0, 0, 1), 3)
        c.parameter_range()  # -> SIGSEGV before the fix
    """

    def _snapshot(self, curve):
        lo, hi = curve.parameter_range()
        mid = 0.5 * (lo + hi)
        return {
            "range": (lo, hi),
            "mid_point": curve.evaluate(mid),
            "length": curve.length(),
        }

    def _assert_curve_alive(self, curve, snap):
        # The exact accessors named in the bug report: parameter_range(),
        # evaluate(), length(). They must both survive and be unchanged.
        lo, hi = curve.parameter_range()
        self.assertAlmostEqual(lo, snap["range"][0], places=9)
        self.assertAlmostEqual(hi, snap["range"][1], places=9)
        mid = 0.5 * (lo + hi)
        mid_point = curve.evaluate(mid)
        # Guard against zip() silently truncating a shorter result.
        self.assertEqual(len(mid_point), len(snap["mid_point"]))
        for got, want in zip(mid_point, snap["mid_point"]):
            self.assertAlmostEqual(got, want, places=6)
        self.assertAlmostEqual(curve.length(), snap["length"], places=6)

    def test_linear_sweep_capped_preserves_input_circle(self):
        circ = sm.create_circle((0, 0, 0), 2)
        snap = self._snapshot(circ)
        sm.linear_sweep([circ], (0, 0, 1), 3, cap_ends=True)
        self._assert_curve_alive(circ, snap)

    def test_linear_sweep_uncapped_preserves_input_circle(self):
        circ = sm.create_circle((0, 0, 0), 2)
        snap = self._snapshot(circ)
        sm.linear_sweep([circ], (0, 0, 1), 3, cap_ends=False)
        self._assert_curve_alive(circ, snap)

    def test_rotational_sweep_preserves_input_curve(self):
        line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
        snap = self._snapshot(line)
        sm.rotational_sweep([line], (0, 0, 0), (0, 0, 1), 360)
        self._assert_curve_alive(line, snap)

    def test_create_surface_revolution_preserves_input_curve(self):
        arc = sm.create_arc((5, 0, 0), 2, 0, 180)
        snap = self._snapshot(arc)
        sm.create_surface_revolution(arc, (0, 0, 0), (0, 0, 1), 360)
        self._assert_curve_alive(arc, snap)

    def test_draft_sweep_preserves_input_circle(self):
        circ = sm.create_circle((0, 0, 0), 5)
        snap = self._snapshot(circ)
        sm.draft_sweep([circ], 10, 5.0)
        self._assert_curve_alive(circ, snap)

    def test_taper_extrude_preserves_input_circle(self):
        circ = sm.create_circle((0, 0, 0), 5)
        snap = self._snapshot(circ)
        # 90 deg == straight extrusion per the taper_extrude convention.
        sm.taper_extrude([circ], 10, 90.0)
        self._assert_curve_alive(circ, snap)

    def test_linear_sweep_with_repetitions_preserves_input(self):
        line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
        snap = self._snapshot(line)
        sm.linear_sweep_with_repetitions(
            [line], (1, 0, 0), 5, repetitions=2, cap_ends=False)
        self._assert_curve_alive(line, snap)

    def test_rotational_sweep_with_repetitions_preserves_input(self):
        line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
        snap = self._snapshot(line)
        sm.rotational_sweep_with_repetitions(
            [line], (0, 0, 0), (0, 0, 1), 120, repetitions=3, cap_ends=False)
        self._assert_curve_alive(line, snap)

    def test_input_curve_reusable_after_sweep(self):
        # Beyond surviving its accessors, the original must remain a valid
        # modeling input: sweeping the same curve a second time must work.
        circ = sm.create_circle((0, 0, 0), 2)
        first = sm.linear_sweep([circ], (0, 0, 1), 3, cap_ends=True)
        second = sm.linear_sweep([circ], (0, 0, 1), 5, cap_ends=True)
        self.assertIsInstance(first, sm.Brep)
        self.assertIsInstance(second, sm.Brep)
        self.assertGreater(len(second.faces()), 0)


# -----------------------------------------------------------------------
#  Fillets
# -----------------------------------------------------------------------
class TestFillets(unittest.TestCase):
    def test_circular_fillet(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.circular_fillet(box, 1.0)
        self.assertGreater(len(box.faces()), 6)

    def test_chamfer_fillet(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.chamfer_fillet(box, 1.0)
        self.assertGreater(len(box.faces()), 6)

    def test_fillet_edges(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edges = box.edges()[:4]
        sm.fillet_edges(box, edges, radius=1.0, xsect_type=1, continuity=1)
        self.assertGreater(len(box.faces()), 6)

    def test_variable_radius_fillet(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edges = box.edges()[:1]
        sm.variable_radius_fillet(box, edges, start_radius=0.5, end_radius=2.0,
                                  xsect_type=1, continuity=1)
        self.assertGreater(len(box.faces()), 6)

    def test_remove_fillet(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.circular_fillet(box, 1.0)
        faces_filleted = len(box.faces())
        self.assertGreater(faces_filleted, 6)
        sm.remove_fillet(box)
        self.assertIsInstance(box, sm.Brep)

    def test_fillet_edges_per_edge(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edges = box.edges()[:2]
        sm.fillet_edges_per_edge(box, edges, [1.0, 2.0],
                                [sm.FilletXSect.CIRCULAR, sm.FilletXSect.CIRCULAR])
        self.assertGreater(len(box.faces()), 6)

    def test_surface_surface_fillet(self):
        srf1 = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        srf2 = sm.create_surface_from_corner_points(
            (5, 0, -5), (5, 0, 5), (5, 10, -5), (5, 10, 5))
        r = sm.surface_surface_fillet(srf1, srf2, radius1=2.0, radius2=2.0, tolerance=0.01)
        self.assertIsInstance(r, sm.Brep)

    def test_fillet_preview(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edges = box.edges()[:2]
        surfaces = sm.fillet_preview(box, edges, radius=1.0)
        self.assertIsInstance(surfaces, list)


# -----------------------------------------------------------------------
#  Offset / Shell
# -----------------------------------------------------------------------
class TestOffsetShell(unittest.TestCase):
    @staticmethod
    def _owner_snapshot(brep):
        faces = tuple(brep.faces())
        return (
            (brep.face_count(), brep.edge_count(), brep.vertex_count()),
            brep.bounding_box(tight=True),
            brep.volume(relative_accuracy=1.0e-4),
            faces,
            tuple(hash(face) for face in faces),
            tuple((face.bounding_box(tight=True), face.area(), face.brep()) for face in faces),
        )

    def test_offset_brep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        r = sm.offset_brep(box, 1.0)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_shell_brep_preserves_source_across_modes(self):
        expected_results = {
            (1.0, True, True): ((11, 24, 16), 584.0, ((-1.0, -1.0, -1.0), (11.0, 11.0, 10.0))),
            (1.0, False, True): ((11, 24, 16), 584.0, ((-1.0, -1.0, -1.0), (11.0, 11.0, 10.0))),
            (1.0, True, False): ((12, 24, 16), 728.0, ((-1.0, -1.0, -1.0), (11.0, 11.0, 11.0))),
            (1.0, False, False): ((12, 24, 16), 728.0, ((-1.0, -1.0, -1.0), (11.0, 11.0, 11.0))),
            (-1.0, True, True): ((11, 24, 16), 424.0, ((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))),
            (-1.0, False, True): ((11, 24, 16), 424.0, ((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))),
            (-1.0, True, False): ((12, 24, 16), 488.0, ((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))),
            (-1.0, False, False): ((12, 24, 16), 488.0, ((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))),
        }

        for (distance, create_solid, select_face), (counts, volume, bounds) in expected_results.items():
            with self.subTest(distance=distance, create_solid=create_solid, select_face=select_face):
                box = sm.create_box((0, 0, 0), 10, 10, 10)
                source_before = self._owner_snapshot(box)
                top_face = next(
                    face for face in source_before[3]
                    if face.bounding_box(tight=True) == ((0.0, 0.0, 10.0), (10.0, 10.0, 10.0))
                )

                result = sm.shell_brep(
                    box,
                    distance,
                    create_solid=create_solid,
                    faces_to_shell=[top_face] if select_face else [],
                )

                self.assertIsInstance(result, sm.Brep)
                self.assertIsNot(result, box)
                self.assertNotEqual(result, box)
                self.assertEqual(self._owner_snapshot(box), source_before)
                self.assertEqual(top_face.brep(), box)
                self.assertEqual((result.face_count(), result.edge_count(), result.vertex_count()), counts)
                self.assertTrue(result.is_manifold_solid())
                self.assertAlmostEqual(result.volume(relative_accuracy=1.0e-4), volume, delta=2.0e-3)
                actual_bounds = result.bounding_box(tight=True)
                for actual_point, expected_point in zip(actual_bounds, bounds, strict=True):
                    for actual_coordinate, expected_coordinate in zip(actual_point, expected_point, strict=True):
                        self.assertAlmostEqual(actual_coordinate, expected_coordinate, delta=1.0e-9)

                sm.translate(result, (20.0, 0.0, 0.0))
                self.assertEqual(self._owner_snapshot(box), source_before)
                self.assertEqual(top_face.brep(), box)

    def test_shell_brep_cylinder_volume_honors_requested_accuracy(self):
        radius = 31.0
        height = 86.0
        wall = 3.0
        cases = (
            (wall, math.pi * ((radius + wall) ** 2 * (height + wall) - radius**2 * height)),
            (-wall, math.pi * (radius**2 * height - (radius - wall) ** 2 * (height - wall))),
        )

        for distance, expected_volume in cases:
            with self.subTest(distance=distance):
                cylinder = sm.create_cylinder((0.0, 0.0, 0.0), radius, height)
                result = sm.shell_brep(
                    cylinder,
                    distance,
                    create_solid=False,
                    faces_to_shell=[
                        max(cylinder.faces(), key=lambda face: face.bounding_box(tight=True)[0][2])
                    ],
                )

                self.assertTrue(result.is_manifold_solid())
                actual_volume = result.volume(relative_accuracy=1.0e-8)
                relative_error = abs(actual_volume - expected_volume) / expected_volume
                self.assertLessEqual(relative_error, 1.0e-5)

    def test_shell_brep_failure_preserves_source(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        source_before = self._owner_snapshot(box)

        with self.assertRaises(RuntimeError):
            sm.shell_brep(box, 0.0, faces_to_shell=[source_before[3][0]])

        self.assertEqual(self._owner_snapshot(box), source_before)

    def test_shell_brep_rejects_foreign_face(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        other = sm.create_box((20, 0, 0), 10, 10, 10)
        source_before = self._owner_snapshot(box)
        other_before = self._owner_snapshot(other)

        with self.assertRaises(RuntimeError):
            sm.shell_brep(box, 1.0, faces_to_shell=[other_before[3][0]])

        self.assertEqual(self._owner_snapshot(box), source_before)
        self.assertEqual(self._owner_snapshot(other), other_before)


# -----------------------------------------------------------------------
#  Stitching
# -----------------------------------------------------------------------
class TestStitching(unittest.TestCase):
    def test_stitch_into_solid(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        box, is_solid, stitched, vert_gap, edge_gap = sm.stitch_into_solid(box)
        self.assertIsInstance(box, sm.Brep)
        self.assertIsInstance(is_solid, bool)
        self.assertIsInstance(stitched, int)

    def test_stitch_into_shell(self):
        pln = sm.create_plane((0, 0, 0), 10, 10)
        pln, well_formed, stitched, vert_gap, edge_gap = sm.stitch_into_shell(pln)
        self.assertIsInstance(pln, sm.Brep)
        self.assertIsInstance(well_formed, bool)
        self.assertIsInstance(stitched, int)

    def test_stitch_into_shell_mode_is_echoed(self):
        # The second element echoes the shell_is_well_formed input, not a result.
        pln = sm.create_plane((0, 0, 0), 10, 10)
        _, mode, _, _, _ = sm.stitch_into_shell(pln)
        self.assertIs(mode, False)
        pln = sm.create_plane((0, 0, 0), 10, 10)
        _, mode, _, _, _ = sm.stitch_into_shell(pln, shell_is_well_formed=True)
        self.assertIs(mode, True)

    def test_unify_normals(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        faces = box.faces()
        box, flipped = sm.unify_normals(box, faces[0], num_samples=10)
        self.assertIsInstance(box, sm.Brep)
        self.assertIsInstance(flipped, list)

    def test_stitch_advanced(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        box, stitched, lamina, vert_gap, edge_gap = sm.stitch_advanced(box, 0.01)
        self.assertIsInstance(box, sm.Brep)
        self.assertIsInstance(stitched, int)
        self.assertIsInstance(lamina, int)


# -----------------------------------------------------------------------
#  Intersectors
# -----------------------------------------------------------------------
class TestIntersectors(unittest.TestCase):
    def test_intersect_breps(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((5, 5, 5), 10, 10, 10)
        curves, points = sm.intersect_breps(a, b)
        self.assertIsInstance(curves, list)
        self.assertIsInstance(points, list)
        self.assertGreater(len(curves), 0)

    def test_intersect_brep_with_plane(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        curves = sm.intersect_brep_with_plane(box, (0, 0, 5), (0, 0, 1))
        self.assertIsInstance(curves, list)
        self.assertGreater(len(curves), 0)

    def test_intersect_curves(self):
        c1 = sm.create_line_segment((0, 0, 0), (10, 10, 0))
        c2 = sm.create_line_segment((0, 10, 0), (10, 0, 0))
        pts, params1, params2 = sm.intersect_curves(c1, c2)
        self.assertIsInstance(pts, list)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(pts[0][0], 5.0, places=2)
        self.assertAlmostEqual(pts[0][1], 5.0, places=2)

    def test_intersect_surfaces(self):
        srf1 = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        srf2 = sm.create_surface_from_corner_points(
            (5, 0, -5), (5, 0, 5), (5, 10, -5), (5, 10, 5))
        curves = sm.intersect_surfaces(srf1, srf2)
        self.assertIsInstance(curves, list)
        self.assertGreater(len(curves), 0)

    def test_intersect_curve_surface(self):
        srf = sm.create_surface_from_corner_points(
            (0, 0, 0), (10, 0, 0), (0, 10, 0), (10, 10, 0))
        line = sm.create_line_segment((5, 5, -5), (5, 5, 5))
        curves, pts = sm.intersect_curve_surface(line, srf)
        self.assertIsInstance(curves, list)
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts) + len(curves), 0)

    def test_intersect_curve_brep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        line = sm.create_line_segment((5, 5, -5), (5, 5, 15))
        curves, pts = sm.intersect_curve_brep(line, box)
        self.assertIsInstance(curves, list)
        self.assertIsInstance(pts, list)
        self.assertGreaterEqual(len(pts), 2)


# -----------------------------------------------------------------------
#  Transforms
# -----------------------------------------------------------------------
class TestTransforms(unittest.TestCase):
    def assertPointAlmostEqual(self, actual, expected, places=8):
        for got, wanted in zip(actual, expected, strict=True):
            self.assertAlmostEqual(got, wanted, places=places)

    def assertRootTransformContract(self, target, point, validate=None):
        expected = point()

        translation = (1.0, 2.0, 3.0)
        self.assertIs(sm.translate(target, translation), target)
        expected = tuple(expected[i] + translation[i] for i in range(3))
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

        self.assertIs(sm.rotate(target, (0, 0, 0), (0, 0, 1), 90.0), target)
        expected = (-expected[1], expected[0], expected[2])
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

        scale = (2.0, 3.0, 4.0)
        self.assertIs(sm.scale(target, scale), target)
        expected = tuple(expected[i] * scale[i] for i in range(3))
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

        ref_point = (1.0, -2.0, 3.0)
        scale = (0.5, 0.5, 0.5)
        self.assertIs(sm.scale_about_point(target, ref_point, scale), target)
        expected = tuple(ref_point[i] + scale[i] * (expected[i] - ref_point[i]) for i in range(3))
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

        translation = (-4.0, 5.0, 6.0)
        placement = sm.Axis2Placement(translation, (1, 0, 0), (0, 1, 0))
        self.assertIs(sm.transform(target, placement), target)
        expected = tuple(expected[i] + translation[i] for i in range(3))
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

    def assertNegativeScaleContract(self, target, point, validate=None):
        scale = (-1.0, 2.0, 3.0)
        expected = point()

        self.assertIs(sm.scale(target, scale), target)
        expected = tuple(expected[i] * scale[i] for i in range(3))
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

        ref_point = (10.0, 20.0, 30.0)
        self.assertIs(sm.scale_about_point(target, ref_point, scale), target)
        expected = tuple(
            ref_point[i] + scale[i] * (expected[i] - ref_point[i])
            for i in range(3))
        self.assertPointAlmostEqual(point(), expected)
        if validate is not None:
            validate()

    def assertPolyBrepNormalsMatchFaces(self, mesh):
        arrays = mesh.to_mesh_arrays()
        points = arrays["points"]
        vertex_normals = arrays["normals"]
        face_normals = arrays["face_normals"]
        incident_normals = [[] for _ in points]

        polygons = []
        cursor = 0
        while cursor < len(arrays["faces"]):
            count = arrays["faces"][cursor]
            polygons.append(arrays["faces"][cursor + 1:cursor + 1 + count])
            cursor += count + 1
        self.assertEqual(cursor, len(arrays["faces"]))
        self.assertEqual(len(polygons), len(face_normals))

        def normalized(vector):
            length = math.sqrt(sum(component * component for component in vector))
            self.assertGreater(length, 1e-8)
            return tuple(component / length for component in vector)

        for indices, face_normal in zip(polygons, face_normals, strict=True):
            self.assertPointAlmostEqual(normalized(face_normal), face_normal, places=6)

            p0, p1, p2 = (points[index] for index in indices[:3])
            edge1 = tuple(p1[i] - p0[i] for i in range(3))
            edge2 = tuple(p2[i] - p0[i] for i in range(3))
            winding_normal = normalized((
                edge1[1] * edge2[2] - edge1[2] * edge2[1],
                edge1[2] * edge2[0] - edge1[0] * edge2[2],
                edge1[0] * edge2[1] - edge1[1] * edge2[0],
            ))
            self.assertGreater(sum(winding_normal[i] * face_normal[i] for i in range(3)), 0.999)

            for index in set(indices):
                if not any(
                    sum(existing[i] * face_normal[i] for i in range(3)) > 0.999999
                    for existing in incident_normals[index]
                ):
                    incident_normals[index].append(face_normal)

        for actual, incident in zip(vertex_normals, incident_normals, strict=True):
            expected = normalized(tuple(sum(normal[i] for normal in incident) for i in range(3)))
            self.assertPointAlmostEqual(actual, expected, places=6)

    def test_translate(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.translate(box, (5, 5, 5))
        closest, dist = sm.brep_closest_point(box, (5, 5, 5))
        self.assertAlmostEqual(dist, 0.0, places=5)

    def test_brep_root_transform_contract(self):
        box = sm.create_box((1, 2, 3), 2, 3, 4)
        vertex = box.vertices()[0]
        self.assertRootTransformContract(box, vertex.point)

    def test_rotate(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.rotate(box, (5, 5, 5), (0, 0, 1), 90)
        self.assertEqual(len(box.faces()), 6)

    def test_scale(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.scale(box, (2, 2, 2))
        vol = sm.compute_volume(box)
        self.assertAlmostEqual(vol, 8000.0, delta=1.0)

    def test_reflected_brep_preserves_material_orientation(self):
        # Wrappers left in reference cycles by earlier tests can sit at addresses
        # this test's new geometry reuses; collect them so they cannot block scale.
        gc.collect()
        for scale in ((-1, 1, 1), (1, -1, 1), (1, 1, -1),
                      (-1, -1, 1), (-1, -1, -1), (-2, 3, 4)):
            with self.subTest(scale=scale):
                box = sm.create_box((-4, -3, -2), 8, 6, 4)
                sm.scale(box, scale)
                self.assertTrue(box.is_manifold_solid())
                self.assertAlmostEqual(box.volume(), 192 * abs(math.prod(scale)), delta=0.01)
                inner = sm.create_box((-1, -1, -1), 2, 2, 2)
                overlap = sm.boolean_intersection(box, inner)
                self.assertTrue(overlap.is_manifold_solid())
                self.assertAlmostEqual(overlap.volume(), 8, delta=0.01)

    def test_curve_translate_and_rotate(self):
        curve = sm.create_circle((0, 0, 0), 1.0)
        self.assertIsInstance(curve, sm.Object)

        self.assertIs(sm.translate(curve, (1, 2, 3)), curve)
        for got, expected in zip(curve.center(), (1.0, 2.0, 3.0), strict=True):
            self.assertAlmostEqual(got, expected, places=8)

        self.assertIs(sm.rotate(curve, (0, 0, 0), (0, 1, 0), 45.0), curve)
        expected_normal = (math.sqrt(0.5), 0.0, math.sqrt(0.5))
        for got, expected in zip(curve.normal(), expected_normal, strict=True):
            self.assertAlmostEqual(got, expected, places=8)

    def test_curve_scale_and_placement_transform(self):
        curve = sm.create_circle((2, 0, 0), 1.0)

        self.assertIs(sm.scale(curve, (2, 2, 2)), curve)
        self.assertAlmostEqual(curve.center()[0], 4.0, places=8)
        self.assertAlmostEqual(curve.radius(), 2.0, places=8)

        self.assertIs(sm.scale_about_point(curve, (1, 0, 0), (0.5, 0.5, 0.5)), curve)
        self.assertAlmostEqual(curve.center()[0], 2.5, places=8)
        self.assertAlmostEqual(curve.radius(), 1.0, places=8)

        placement = sm.Axis2Placement((1, 2, 3), (1, 0, 0), (0, 1, 0))
        self.assertIs(sm.transform(curve, placement), curve)
        for got, expected in zip(curve.center(), (3.5, 2.0, 3.0), strict=True):
            self.assertAlmostEqual(got, expected, places=8)

        line = sm.create_line_segment((2, 4, 6), (3, 5, 7))
        self.assertIs(sm.scale_about_point(line, (1, 1, 1), (2, 3, 4)), line)
        for got, expected in zip(line.start_point(), (3, 10, 21), strict=True):
            self.assertAlmostEqual(got, expected, places=8)
        for got, expected in zip(line.end_point(), (5, 13, 25), strict=True):
            self.assertAlmostEqual(got, expected, places=8)

    def test_negative_scale_supported_for_exact_roots(self):
        box = sm.create_box((1, 2, 3), 2, 3, 4)
        vertex = box.vertices()[0]
        self.assertNegativeScaleContract(
            box, vertex.point, lambda: self.assertTrue(box.is_manifold_solid()))

        line = sm.create_line_segment((1, 2, 3), (4, 5, 6))
        self.assertNegativeScaleContract(line, line.start_point)

        spline = sm.create_curve([(1, 2, 3), (2, 3, 4), (3, 5, 5), (4, 5, 6)])
        lo, hi = spline.parameter_range()
        mid = 0.5 * (lo + hi)
        self.assertNegativeScaleContract(spline, lambda: spline.evaluate(mid))

    def test_curve_transform_dispatch_and_analytic_scale_rejection(self):
        line = sm.create_line_segment((0, 0, 0), (2, 0, 0))
        placement = sm.Axis2Placement((1, 2, 3), (1, 0, 0), (0, 1, 0))
        self.assertIs(sm.transform(line, placement), line)
        self.assertEqual(line.start_point(), (1.0, 2.0, 3.0))
        self.assertEqual(line.end_point(), (3.0, 2.0, 3.0))

        spline = sm.create_curve([(0, 0, 0), (1, 0, 0), (2, 1, 0), (3, 1, 0)])
        before = spline.evaluate(0.5 * sum(spline.parameter_range()))
        self.assertIs(sm.translate(spline, (4, 5, 6)), spline)
        after = spline.evaluate(0.5 * sum(spline.parameter_range()))
        for got, expected in zip(after, (before[0] + 4, before[1] + 5, before[2] + 6), strict=True):
            self.assertAlmostEqual(got, expected, places=8)

        circle = sm.create_circle((2, 3, 4), 5.0)
        before_center = circle.center()
        before_radius = circle.radius()
        scales = (
            ((2, 1, 1), True),
            ((0, 0, 0), False),
            ((-1, -1, -1), True),
            ((math.inf, 1, 1), False),
            ((math.nan, 1, 1), False),
        )
        for scale, expects_representation_message in scales:
            with self.subTest(scale=scale):
                with self.assertRaises(RuntimeError) as raised:
                    sm.scale_about_point(circle, (1, 1, 1), scale)
                message = str(raised.exception)
                self.assertIn("SM_ERR_INVALID_INPUT", message)
                if expects_representation_message:
                    self.assertIn("standalone Curve", message)
                    self.assertIn("positive, uniform scale factors", message)
                    self.assertIn("does not replace standalone geometry automatically", message)
                    self.assertIn("generic B-spline Curve", message)
                self.assertEqual(circle.center(), before_center)
                self.assertEqual(circle.radius(), before_radius)

    def test_standalone_surface_transform_contract(self):
        surface = sm.create_surface_from_corner_points(
            (0, 0, 0), (2, 0, 0), (0, 2, 0), (2, 2, 0))
        self.assertIsInstance(surface, sm.Object)
        (u_min, v_min), (u_max, v_max) = surface.uv_domain()
        uv_mid = (0.5 * (u_min + u_max), 0.5 * (v_min + v_max))

        self.assertRootTransformContract(surface, lambda: surface.evaluate(*uv_mid))

    def test_analytic_surface_rejects_unsupported_scale_without_mutation(self):
        profile = sm.create_arc((5, 0, 0), 2, 0, 180)
        surface = sm.create_surface_revolution(profile, (0, 0, 0), (0, 0, 1), 360)
        (u_min, v_min), (u_max, v_max) = surface.uv_domain()
        uv_mid = (0.5 * (u_min + u_max), 0.5 * (v_min + v_max))
        before = surface.evaluate(*uv_mid)

        scales = (
            ((2, 3, 4), True),
            ((0, 0, 0), False),
            ((-1, -1, -1), True),
            ((math.inf, 1, 1), False),
            ((math.nan, 1, 1), False),
        )
        for scale, expects_representation_message in scales:
            operations = (
                ("scale", lambda: sm.scale(surface, scale)),
                ("scale_about_point", lambda: sm.scale_about_point(surface, (1, 2, 3), scale)),
            )
            for name, operation in operations:
                with self.subTest(operation=name, scale=scale):
                    with self.assertRaises(RuntimeError) as raised:
                        operation()
                    message = str(raised.exception)
                    self.assertIn("SM_ERR_INVALID_INPUT", message)
                    if expects_representation_message:
                        self.assertIn("standalone Surface", message)
                        self.assertIn("positive, uniform scale factors", message)
                        self.assertIn("does not replace standalone geometry automatically", message)
                        self.assertIn("generic B-spline Surface", message)
                    self.assertPointAlmostEqual(surface.evaluate(*uv_mid), before)

    def test_polybrep_transform_contract(self):
        source = sm.create_box((-1, -1, -1), 2, 2, 2)
        sm.rotate(source, (0, 0, 0), (0, 0, 1), 37)
        sm.rotate(source, (0, 0, 0), (0, 1, 0), 23)
        mesh = sm.tessellate(source, curve_angle_tolerance_deg=0.0)
        self.assertIsInstance(mesh, sm.Object)
        vertex = mesh.get_vertices()[0]

        self.assertRootTransformContract(
            mesh,
            vertex.get_point,
            lambda: self.assertPolyBrepNormalsMatchFaces(mesh))

        for scale in ((1, 0, 1), (-1, 1, 1), (math.inf, 1, 1), (math.nan, 1, 1)):
            operations = (
                ("scale", lambda: sm.scale(mesh, scale)),
                ("scale_about_point", lambda: sm.scale_about_point(mesh, (1, 2, 3), scale)),
            )
            for name, operation in operations:
                with self.subTest(operation=name, scale=scale):
                    before = mesh.to_mesh_arrays()
                    with self.assertRaisesRegex(RuntimeError, "SM_ERR_INVALID_INPUT"):
                        operation()
                    self.assertEqual(mesh.to_mesh_arrays(), before)

        # Large model scales must not underflow normal normalization.
        self.assertIs(sm.scale(mesh, (1e15, 1e15, 1e15)), mesh)
        self.assertPolyBrepNormalsMatchFaces(mesh)

    def test_edge_owned_curve_transform_is_rejected_without_damaging_brep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        curve = edge.curve()
        before_bounds = box.bounding_box()
        before_volume = box.volume()
        before_length = edge.length()
        before_vertices = tuple(vertex.point() for vertex in edge.vertices())

        placement = sm.Axis2Placement((1, 2, 3), (1, 0, 0), (0, 1, 0))
        operations = (
            ("translate", lambda: sm.translate(curve, (1, 2, 3))),
            ("rotate", lambda: sm.rotate(curve, (0, 0, 0), (0, 0, 1), 45)),
            ("scale", lambda: sm.scale(curve, (2, 2, 2))),
            ("scale_about_point", lambda: sm.scale_about_point(curve, (1, 1, 1), (2, 2, 2))),
            ("transform", lambda: sm.transform(curve, placement)),
        )
        for name, operation in operations:
            with self.subTest(operation=name):
                with self.assertRaises(RuntimeError) as raised:
                    operation()
                message = str(raised.exception)
                self.assertIn("SM_ERR_INVALID_INPUT", message)
                self.assertIn("Brep-owned Curve", message)
                self.assertIn("transform the owning Brep", message)

        self.assertEqual(box.bounding_box(), before_bounds)
        self.assertAlmostEqual(box.volume(), before_volume, places=8)
        self.assertAlmostEqual(edge.length(), before_length, places=8)
        self.assertEqual(tuple(vertex.point() for vertex in edge.vertices()), before_vertices)

    def test_face_owned_surface_transform_is_rejected_without_damaging_brep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        surface = face.surface()
        (u_min, v_min), (u_max, v_max) = face.uv_domain()
        uv_mid = (0.5 * (u_min + u_max), 0.5 * (v_min + v_max))
        before_bounds = box.bounding_box()
        before_volume = box.volume()
        before_area = face.area()
        before_surface_point = surface.evaluate(*uv_mid)
        before_vertices = tuple(vertex.point() for vertex in face.vertices())

        placement = sm.Axis2Placement((1, 2, 3), (1, 0, 0), (0, 1, 0))
        operations = (
            ("translate", lambda: sm.translate(surface, (1, 2, 3))),
            ("rotate", lambda: sm.rotate(surface, (0, 0, 0), (0, 0, 1), 45)),
            ("scale", lambda: sm.scale(surface, (2, 2, 2))),
            ("scale_about_point", lambda: sm.scale_about_point(surface, (1, 1, 1), (2, 2, 2))),
            ("transform", lambda: sm.transform(surface, placement)),
        )
        for name, operation in operations:
            with self.subTest(operation=name):
                with self.assertRaises(RuntimeError) as raised:
                    operation()
                message = str(raised.exception)
                self.assertIn("SM_ERR_INVALID_INPUT", message)
                self.assertIn("Brep-owned Surface", message)
                self.assertIn("transform the owning Brep", message)

        self.assertEqual(box.bounding_box(), before_bounds)
        self.assertAlmostEqual(box.volume(), before_volume, places=8)
        self.assertAlmostEqual(face.area(), before_area, places=8)
        self.assertPointAlmostEqual(surface.evaluate(*uv_mid), before_surface_point)
        self.assertEqual(tuple(vertex.point() for vertex in face.vertices()), before_vertices)


# -----------------------------------------------------------------------
#  Borrowed Curve / Surface handles across Brep representation changes
# -----------------------------------------------------------------------
class TestBorrowedGeometryHandleLifetime(unittest.TestCase):
    """Crash regressions run in children so a dangling pointer cannot kill the suite."""

    _BORROWED_GEOMETRY_SCRIPT = r"""
import gc
import math
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm


def close(actual, expected, tolerance=1.0e-7):
    if isinstance(actual, (tuple, list)):
        if len(actual) != len(expected):
            raise AssertionError("length mismatch: %r != %r" % (actual, expected))
        for got, wanted in zip(actual, expected):
            close(got, wanted, tolerance)
        return
    if not math.isclose(actual, expected, rel_tol=tolerance, abs_tol=tolerance):
        raise AssertionError("value mismatch: %r != %r" % (actual, expected))


def brep_snapshot(brep):
    return {
        "bounds": brep.bounding_box(),
        "volume": brep.volume(1.0e-6),
        "counts": (brep.face_count(), brep.edge_count(), brep.vertex_count()),
        "manifold": brep.is_manifold_solid(),
    }


def require_snapshot(brep, snapshot):
    close(brep.bounding_box(), snapshot["bounds"])
    close(brep.volume(1.0e-6), snapshot["volume"])
    if (brep.face_count(), brep.edge_count(), brep.vertex_count()) != snapshot["counts"]:
        raise AssertionError("topology counts changed during rejected operation")
    if brep.is_manifold_solid() != snapshot["manifold"]:
        raise AssertionError("manifold state changed during rejected operation")


def scaled_bounds(bounds, scale, reference):
    minimum, maximum = bounds
    result_minimum = []
    result_maximum = []
    for axis in range(3):
        a = reference[axis] + scale[axis] * (minimum[axis] - reference[axis])
        b = reference[axis] + scale[axis] * (maximum[axis] - reference[axis])
        result_minimum.append(min(a, b))
        result_maximum.append(max(a, b))
    return tuple(result_minimum), tuple(result_maximum)


case = sys.argv[1]
parts = case.split(":")
operation = parts[0]
reference = (4.0, -3.0, 2.0)


if operation == "consumed_invalid_scale":
    scale_operation = parts[1]
    first = sm.create_box((0, 0, 0), 2, 2, 2)
    second = sm.create_box((1, 0, 0), 2, 2, 2)
    result = sm.boolean_union(first, second)
    if result is first:
        consumed = second
    elif result is second:
        consumed = first
    else:
        raise AssertionError("boolean result did not reuse either input wrapper")

    # Disturb the freed allocation so a binding-side virtual type check is
    # likely to expose the stale access. The operation itself remains invalid
    # and must be rejected before accessing the consumed target.
    for index in range(10):
        sm.create_box((10 + 3 * index, 0, 0), 1, 1, 1)

    expected_api = {
        "scale": "SmApiScale",
        "scale_about_point": "SmApiScaleByPt",
    }.get(scale_operation)
    if expected_api is None:
        raise AssertionError("unknown consumed invalid-scale operation: %s" % scale_operation)

    try:
        if scale_operation == "scale":
            sm.scale(consumed, (0, 1, 1))
        else:
            sm.scale_about_point(consumed, reference, (0, 1, 1))
    except RuntimeError as error:
        message = str(error)
        for expected in (expected_api, "SM_ERR_INVALID_INPUT"):
            if expected not in message:
                raise AssertionError("missing %r in error: %s" % (expected, message))
    else:
        raise AssertionError("invalid scale unexpectedly succeeded")

    print("borrowed geometry case passed: %s" % case)
    sys.exit(0)


if operation == "pointer_preserving":
    brep = sm.create_box((1, 2, 3), 2, 3, 4)
    face = brep.faces()[0]
    edge = brep.edges()[0]
    surface = face.surface()
    curve = edge.curve()
    (u_min, v_min), (u_max, v_max) = surface.uv_domain()
    uv = (0.5 * (u_min + u_max), 0.5 * (v_min + v_max))
    t_min, t_max = curve.parameter_range()
    t = 0.5 * (t_min + t_max)
    surface_point = surface.evaluate(*uv)
    curve_point = curve.evaluate(t)

    def require_points(expected_surface, expected_curve):
        close(surface.evaluate(*uv), expected_surface)
        close(curve.evaluate(t), expected_curve)
        if face.surface() is not surface:
            raise AssertionError("pointer-preserving transform replaced the Surface")
        if edge.curve() is not curve:
            raise AssertionError("pointer-preserving transform replaced the Curve")

    translation = (4.0, -2.0, 1.0)
    sm.translate(brep, translation)
    surface_point = tuple(surface_point[i] + translation[i] for i in range(3))
    curve_point = tuple(curve_point[i] + translation[i] for i in range(3))
    require_points(surface_point, curve_point)

    sm.rotate(brep, (0, 0, 0), (0, 0, 1), 90)
    surface_point = (-surface_point[1], surface_point[0], surface_point[2])
    curve_point = (-curve_point[1], curve_point[0], curve_point[2])
    require_points(surface_point, curve_point)

    placement_translation = (3.0, 5.0, -1.0)
    placement = sm.Axis2Placement(placement_translation, (1, 0, 0), (0, 1, 0))
    sm.transform(brep, placement)
    surface_point = tuple(surface_point[i] + placement_translation[i] for i in range(3))
    curve_point = tuple(curve_point[i] + placement_translation[i] for i in range(3))
    require_points(surface_point, curve_point)

    sm.scale(brep, (2, 2, 2))
    surface_point = tuple(2 * value for value in surface_point)
    curve_point = tuple(2 * value for value in curve_point)
    require_points(surface_point, curve_point)

    print("borrowed geometry case passed: %s" % case)
    sys.exit(0)


if operation == "invalid_scale":
    scale_operation = parts[1]
    scale = tuple(float(value) for value in parts[2].split(","))
    brep = sm.create_box((1, 2, 3), 2, 3, 4)
    face = brep.faces()[0]
    edge = brep.edges()[0]
    surface = face.surface()
    curve = edge.curve()
    surface_query = surface.uv_domain()
    curve_query = curve.parameter_range()
    before = brep_snapshot(brep)

    expected_api = {
        "scale": "SmApiScale",
        "scale_about_point": "SmApiScaleByPt",
    }.get(scale_operation)
    if expected_api is None:
        raise AssertionError("unknown invalid-scale operation: %s" % scale_operation)

    try:
        if scale_operation == "scale":
            sm.scale(brep, scale)
        else:
            sm.scale_about_point(brep, reference, scale)
    except RuntimeError as error:
        message = str(error)
        for expected in (expected_api, "SM_ERR_INVALID_INPUT"):
            if expected not in message:
                raise AssertionError("missing %r in error: %s" % (expected, message))
        for unexpected in ("borrowed Python handle", "reacquire", "copy the Brep"):
            if unexpected in message:
                raise AssertionError("invalid scale used borrowed-handle rejection: %s" % message)
    else:
        raise AssertionError("invalid scale unexpectedly succeeded")

    require_snapshot(brep, before)
    close(surface.uv_domain(), surface_query)
    close(curve.parameter_range(), curve_query)
    if face.surface() is not surface or edge.curve() is not curve:
        raise AssertionError("invalid scale replaced borrowed geometry")

    print("borrowed geometry case passed: %s" % case)
    sys.exit(0)


fixture = parts[1]
scale = tuple(float(value) for value in parts[2].split(",")) if len(parts) == 3 else None

surface_owner = None
edge_owner = None
surface = None
curve = None

if fixture in ("box", "box_surface"):
    brep = sm.create_box((1, 2, 3), 2, 3, 4)
    surface_owner = brep.faces()[0]
    surface = surface_owner.surface()
    if fixture == "box":
        edge_owner = brep.edges()[0]
        curve = edge_owner.curve()
elif fixture == "sphere":
    brep = sm.create_sphere((1, 2, 3), 2)
    surface_owner = brep.faces()[0]
    surface = surface_owner.surface()
elif fixture == "cylinder_circle":
    brep = sm.create_cylinder((0, 0, 0), 2, 5)
    for candidate_owner in brep.edges():
        candidate = candidate_owner.curve()
        if isinstance(candidate, sm.Circle):
            edge_owner = candidate_owner
            curve = candidate
            break
        candidate = None
    candidate = None
    if curve is None:
        raise AssertionError("cylinder did not expose a circular edge")
else:
    raise AssertionError("unknown fixture: %s" % fixture)

surface_alias = surface
curve_alias = curve
surface_query = surface.uv_domain() if surface is not None else None
curve_query = curve.parameter_range() if curve is not None else None
before = brep_snapshot(brep)


def run_operation():
    if operation == "turn_to_nurbs":
        sm.turn_to_nurbs(brep)
    elif operation == "scale":
        sm.scale(brep, scale)
    elif operation == "scale_about_point":
        sm.scale_about_point(brep, reference, scale)
    else:
        raise AssertionError("unknown operation: %s" % operation)


def query_live_aliases():
    if surface is not None:
        close(surface.uv_domain(), surface_query)
    if surface_alias is not None:
        close(surface_alias.uv_domain(), surface_query)
    if curve is not None:
        close(curve.parameter_range(), curve_query)
    if curve_alias is not None:
        close(curve_alias.parameter_range(), curve_query)


def require_rejected():
    try:
        run_operation()
    except RuntimeError as error:
        message = str(error)
        expected_api = {
            "turn_to_nurbs": "SmApiTurnToNurbs",
            "scale": "SmApiScale",
            "scale_about_point": "SmApiScaleByPt",
        }[operation]
        for expected in (
            expected_api,
            "SM_ERR_INVALID_INPUT",
            "Edge.curve()",
            "Face.surface()",
            "reacquire",
            "copy the Brep",
        ):
            if expected not in message:
                raise AssertionError("missing %r in error: %s" % (expected, message))
    else:
        # Exercise the aliases before failing. If a future regression deletes
        # their geometry, only this child process receives the resulting signal.
        query_live_aliases()
        raise AssertionError("representation-changing operation unexpectedly succeeded")

    require_snapshot(brep, before)
    query_live_aliases()


# Two Python aliases to each wrapper must keep the preflight active until the
# final reference is released. For the box, releasing Surface aliases first
# also proves the retained Curve independently blocks the operation.
require_rejected()
if surface is not None:
    surface = None
    gc.collect()
    require_rejected()
    surface_alias = None
    gc.collect()
if curve is not None:
    require_rejected()
    curve = None
    gc.collect()
    require_rejected()
    curve_alias = None
    gc.collect()

run_operation()

if operation == "turn_to_nurbs":
    # Conversion is geometrically exact, but the NURBS integration route and
    # recomputed bounding-box tolerance need not reproduce analytic values bit
    # for bit.
    close(brep.bounding_box(), before["bounds"], 1.0e-4)
    close(brep.volume(1.0e-6), before["volume"], 1.0e-6)
else:
    origin = (0.0, 0.0, 0.0) if operation == "scale" else reference
    close(brep.bounding_box(), scaled_bounds(before["bounds"], scale, origin), 1.0e-4)
    # Analytic-to-NURBS conversion can vary by a few parts per million across
    # architectures even when the represented geometry is unchanged.
    close(brep.volume(1.0e-6),
          before["volume"] * abs(scale[0] * scale[1] * scale[2]),
          1.0e-5)

if not brep.is_manifold_solid():
    raise AssertionError("successful operation did not preserve a manifold solid")
if (brep.face_count(), brep.edge_count(), brep.vertex_count()) != before["counts"]:
    raise AssertionError("successful representation change altered topology counts")

reacquired_surface = surface_owner.surface() if surface_owner is not None else None
reacquired_curve = edge_owner.curve() if edge_owner is not None else None
if reacquired_surface is not None:
    reacquired_surface.uv_domain()
if reacquired_curve is not None:
    reacquired_curve.parameter_range()
    if fixture in ("box", "cylinder_circle") and isinstance(reacquired_curve, (sm.Line, sm.Circle)):
        raise AssertionError("analytic edge curve was not converted to base NURBS")

if operation == "turn_to_nurbs":
    # A second conversion is a pointer-preserving no-op, so live reacquired
    # handles must not cause a false rejection or become invalid.
    sm.turn_to_nurbs(brep)
else:
    # The first scale converted the geometry. Repeating the same scale with
    # live base-NURBS handles must mutate those objects in place.
    run_operation()

if reacquired_surface is not None:
    if surface_owner.surface() is not reacquired_surface:
        raise AssertionError("base NURBS surface pointer changed")
    reacquired_surface.uv_domain()
if reacquired_curve is not None:
    if edge_owner.curve() is not reacquired_curve:
        raise AssertionError("base NURBS curve pointer changed")
    reacquired_curve.parameter_range()

print("borrowed geometry case passed: %s" % sys.argv[1])
"""

    _CASES = (
        "turn_to_nurbs:box",
        "turn_to_nurbs:box_surface",
        "turn_to_nurbs:sphere",
        "turn_to_nurbs:cylinder_circle",
        "scale:box:2,3,4",
        "scale:box:-2,-2,-2",
        "scale:box:-1,2,3",
        "scale_about_point:box:2,3,4",
        "scale_about_point:box:-2,-2,-2",
        "scale_about_point:box:-1,2,3",
        "scale:sphere:2,3,4",
        "scale_about_point:sphere:-1,2,3",
    )

    _INVALID_SCALE_CASES = (
        "invalid_scale:scale:0,1,1",
        "invalid_scale:scale:nan,1,1",
        "invalid_scale:scale:inf,1,1",
        "invalid_scale:scale_about_point:0,1,1",
        "invalid_scale:scale_about_point:nan,1,1",
        "invalid_scale:scale_about_point:inf,1,1",
    )

    _CONSUMED_INVALID_SCALE_CASES = (
        "consumed_invalid_scale:scale",
        "consumed_invalid_scale:scale_about_point",
    )

    def _assert_borrowed_geometry_case(self, case):
        try:
            completed = subprocess.run(
                [sys.executable, "-c", self._BORROWED_GEOMETRY_SCRIPT, case],
                capture_output=True,
                text=True,
                timeout=20,
            )
        except subprocess.TimeoutExpired:
            self.fail("borrowed geometry lifetime case timed out: %s" % case)

        self.assertEqual(
            completed.returncode,
            0,
            msg="case %s child output:\n%s\n%s" % (case, completed.stdout, completed.stderr),
        )
        self.assertIn("borrowed geometry case passed: %s" % case, completed.stdout)

    # Topology wrappers (Face/Edge/Vertex) of a Brep the kernel deletes stay
    # registered at addresses that new geometry can reuse. They are not
    # borrowed Surface/Curve handles, so they must not block scale or
    # turn_to_nurbs.
    _STALE_TOPOLOGY_SCRIPT = r"""
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm

def refused(operation):
    try:
        operation()
    except RuntimeError as exc:
        if "borrowed Python handle" not in str(exc):
            raise
        return 1
    return 0

# Address reuse depends on the allocator's state, so run many rounds. On a build that
# counts any wrapper type (Linux, 2026-10-05), 1 or 5 rounds gave no refusal, 20 gave one
# and 200 gave 17 in every run.
scale_refusals = nurbs_refusals = 0
stale = []
for _ in range(200):
    kept = sm.create_box((0, 0, 0), 4, 4, 4)
    consumed = sm.create_box((1, 1, 1), 4, 4, 4)
    stale += list(consumed.faces()) + list(consumed.edges()) + list(consumed.vertices())
    sm.boolean_union(kept, consumed)
    box = sm.create_box((-4, -3, -2), 8, 6, 4)
    scale_refusals += refused(lambda: sm.scale(box, (-1, 1, 1)))
    nurbs_box = sm.create_box((-4, -3, -2), 8, 6, 4)
    nurbs_refusals += refused(lambda: sm.turn_to_nurbs(nurbs_box))
print("stale topology refusals: scale=%d turn_to_nurbs=%d" % (scale_refusals, nurbs_refusals))
"""

    def test_stale_topology_wrappers_do_not_block_nurbs_conversion(self):
        completed = subprocess.run(
            [sys.executable, "-c", self._STALE_TOPOLOGY_SCRIPT],
            capture_output=True,
            text=True,
            timeout=120,
        )
        self.assertEqual(
            completed.returncode,
            0,
            msg="child output:\n%s\n%s" % (completed.stdout, completed.stderr),
        )
        self.assertIn("stale topology refusals: scale=0 turn_to_nurbs=0", completed.stdout)

    def test_representation_changes_reject_live_borrowed_geometry(self):
        for case in self._CASES:
            with self.subTest(case=case):
                self._assert_borrowed_geometry_case(case)

    def test_pointer_preserving_brep_transforms_keep_borrowed_geometry_live(self):
        self._assert_borrowed_geometry_case("pointer_preserving:box")

    def test_copy_can_change_representation_while_original_borrow_is_live(self):
        source = sm.create_box((1, 2, 3), 2, 3, 4)
        source_surface = source.faces()[0].surface()
        source_domain = source_surface.uv_domain()
        source_bounds = source.bounding_box()

        transformed = source.copy()
        self.assertIs(sm.scale(transformed, (2, 3, 4)), transformed)

        self.assertEqual(source.bounding_box(), source_bounds)
        self.assertEqual(source_surface.uv_domain(), source_domain)
        self.assertNotEqual(transformed.bounding_box(), source_bounds)

    def test_invalid_scale_keeps_borrowed_geometry_and_brep_unchanged(self):
        for case in self._INVALID_SCALE_CASES:
            with self.subTest(case=case):
                self._assert_borrowed_geometry_case(case)

    def test_invalid_scale_is_rejected_before_accessing_consumed_target(self):
        for case in self._CONSUMED_INVALID_SCALE_CASES:
            with self.subTest(case=case):
                self._assert_borrowed_geometry_case(case)


# -----------------------------------------------------------------------
#  Queries
# -----------------------------------------------------------------------
class TestQueries(unittest.TestCase):
    def test_brep_closest_point(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        closest, dist = sm.brep_closest_point(box, (5, 5, 15))
        self.assertAlmostEqual(closest.z, 10.0, places=3)
        self.assertAlmostEqual(dist, 5.0, places=3)

    def test_curve_closest_point(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        closest, param, dist = sm.curve_closest_point(line, (5, 3, 0))
        self.assertAlmostEqual(closest.x, 5.0, places=3)
        self.assertAlmostEqual(closest.y, 0.0, places=3)
        self.assertAlmostEqual(dist, 3.0, places=3)

    def test_surface_closest_point(self):
        pts = [
            (0, 0, 0),  (5, 0, 0),  (10, 0, 0),  (15, 0, 0),
            (0, 5, 0),  (5, 5, 0),  (10, 5, 0),  (15, 5, 0),
            (0, 10, 0), (5, 10, 0), (10, 10, 0), (15, 10, 0),
            (0, 15, 0), (5, 15, 0), (10, 15, 0), (15, 15, 0),
        ]
        srf = sm.create_surface_from_points(pts, 4, 4)
        closest, uvs, dist = sm.surface_closest_point(srf, (5, 5, 3))
        self.assertEqual(len(uvs), 1)
        self.assertAlmostEqual(closest.z, 0.0, places=2)
        self.assertAlmostEqual(dist, 3.0, places=2)
        evaluated = srf.evaluate(uvs[0].x, uvs[0].y)
        self.assertLess(math.dist(evaluated, (closest.x, closest.y, closest.z)), 1.0e-8)

    def test_surface_closest_point_returns_periodic_seam_aliases(self):
        cylinder = sm.create_cylinder((0, 0, 0), 5, 10)
        surface = next(
            face.surface()
            for face in cylinder.faces()
            if face.surface().is_periodic_u()
        )
        (umin, vmin), (umax, vmax) = surface.uv_domain()
        query = surface.evaluate(umin, 0.5 * (vmin + vmax))

        closest, uvs, distance = sm.surface_closest_point(surface, query)

        tolerance = 1.0e-8
        closest_tuple = (closest.x, closest.y, closest.z)
        self.assertLessEqual(distance, tolerance)
        self.assertGreaterEqual(len(uvs), 2)
        self.assertTrue(any(abs(uv.x - umin) <= tolerance for uv in uvs))
        self.assertTrue(any(abs(uv.x - umax) <= tolerance for uv in uvs))
        for uv in uvs:
            self.assertLessEqual(
                math.dist(surface.evaluate(uv.x, uv.y), closest_tuple),
                tolerance,
            )

    def test_surface_closest_point_returns_double_periodic_aliases(self):
        torus = sm.create_torus((0, 0, 0), 10, 3)
        surface = torus.faces()[0].surface()
        (umin, vmin), (umax, vmax) = surface.uv_domain()
        query = surface.evaluate(umin, vmin)

        closest, uvs, distance = sm.surface_closest_point(surface, query)

        tolerance = 1.0e-8
        closest_tuple = (closest.x, closest.y, closest.z)
        expected_uvs = {
            (umin, vmin),
            (umin, vmax),
            (umax, vmin),
            (umax, vmax),
        }
        self.assertLessEqual(distance, tolerance)
        self.assertEqual(len(uvs), len(expected_uvs))
        for expected_u, expected_v in expected_uvs:
            self.assertTrue(any(
                abs(uv.x - expected_u) <= tolerance
                and abs(uv.y - expected_v) <= tolerance
                for uv in uvs
            ))
        for uv in uvs:
            self.assertLessEqual(
                math.dist(surface.evaluate(uv.x, uv.y), closest_tuple),
                tolerance,
            )

    def test_surface_closest_point_uvs_represent_only_selected_point(self):
        sphere = sm.create_sphere((0, 0, 0), 5)
        surface = sphere.faces()[0].surface()

        closest, uvs, distance = sm.surface_closest_point(surface, (0, 0, 0))

        tolerance = 1.0e-8
        closest_tuple = (closest.x, closest.y, closest.z)
        self.assertAlmostEqual(distance, 5.0, places=8)
        self.assertGreater(len(uvs), 0)
        for uv in uvs:
            self.assertLessEqual(
                math.dist(surface.evaluate(uv.x, uv.y), closest_tuple),
                tolerance,
            )

    def test_surface_closest_point_ignores_owning_face_trims(self):
        annulus = sm.create_planar_faces([
            sm.create_circle((0, 0, 0), 5),
            sm.create_circle((0, 0, 0), 2),
        ])
        face = annulus.faces()[0]
        surface = face.surface()

        closest, uvs, distance = sm.surface_closest_point(surface, (0, 0, 0))

        tolerance = 1.0e-8
        self.assertLessEqual(distance, tolerance)
        self.assertLessEqual(
            math.dist((closest.x, closest.y, closest.z), (0, 0, 0)),
            tolerance,
        )
        self.assertGreater(len(uvs), 0)
        self.assertTrue(all(
            face.classify_uv(uv.x, uv.y) == "outside"
            for uv in uvs
        ))

    def test_surface_closest_point_uses_natural_domain(self):
        # Regression: analytic fillet surfaces have distinct
        # STEP and natural/NURBS UV domains.  Passing STEP bounds to the
        # natural-domain solvers can move a point already on the surface to a
        # domain boundary and report a nonzero distance.
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        origin = (0.0, 0.0, 0.0)
        corner = min(box.vertices(), key=lambda vertex: math.dist(vertex.point(), origin))
        sm.fillet_edges(box, corner.edges(), radius=1.0, xsect_type=1, continuity=1)
        tolerance = 1.0e-7
        corner_faces = []
        for item in box.faces():
            _, bbox_max = item.bounding_box(tight=True)
            if all(coordinate <= 1.0 + tolerance for coordinate in bbox_max):
                corner_faces.append(item)
        self.assertEqual(len(corner_faces), 1)
        face = corner_faces[0]

        surface = face.surface()
        point = face.interior_point()
        (umin, vmin), (umax, vmax) = surface.uv_domain()

        closest, uvs, distance = sm.surface_closest_point(surface, point)
        closest_tuple = (closest.x, closest.y, closest.z)
        self.assertLessEqual(distance, tolerance)
        self.assertGreater(len(uvs), 0)
        self.assertLessEqual(math.dist(closest_tuple, point), tolerance)
        self.assertTrue(
            any(face.classify_uv(uv.x, uv.y) == "inside" for uv in uvs)
        )
        for uv in uvs:
            self.assertGreaterEqual(uv.x, umin - tolerance)
            self.assertLessEqual(uv.x, umax + tolerance)
            self.assertGreaterEqual(uv.y, vmin - tolerance)
            self.assertLessEqual(uv.y, vmax + tolerance)
            self.assertLessEqual(
                math.dist(surface.evaluate(uv.x, uv.y), closest_tuple),
                tolerance,
            )
        self.assertAlmostEqual(math.dist(point, closest_tuple), distance, places=8)

    def test_is_manifold_solid(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertTrue(sm.is_manifold_solid(box))

    def test_is_manifold_solid_false(self):
        pln = sm.create_plane((0, 0, 0), 10, 10)
        self.assertFalse(sm.is_manifold_solid(pln))

    def test_material_census_simple_solid(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertEqual(sm.material_census(box),
                         {"solid_count": 1, "void_count": 0})

    def test_material_census_method_matches_function(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertEqual(box.material_census(), sm.material_census(box))

    def test_material_census_submodule_alias(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertEqual(sm.queries.material_census(box),
                         {"solid_count": 1, "void_count": 0})

    def test_material_census_hollow_has_void(self):
        # A cube with a fully-interior cube removed leaves one solid shell
        # around one enclosed void cavity.
        outer = sm.create_box((0, 0, 0), 10, 10, 10)
        inner = sm.create_box((3, 3, 3), 4, 4, 4)  # 3..7 in each axis, interior
        hollow = sm.boolean_difference(outer, inner)
        self.assertEqual(sm.material_census(hollow),
                         {"solid_count": 1, "void_count": 1})
        # 10^3 minus 4^3 = 936; the cavity is subtracted from the volume.
        self.assertAlmostEqual(hollow.volume(relative_accuracy=1.0e-4),
                               936.0, delta=0.5)

    def test_material_census_disjoint_solids(self):
        # Two non-overlapping boxes -> two disjoint material regions.
        a = sm.create_box((0, 0, 0), 2, 2, 2)
        far = sm.create_box((20, 0, 0), 2, 2, 2)
        combined = sm.boolean_union(a, far)
        self.assertEqual(sm.material_census(combined),
                         {"solid_count": 2, "void_count": 0})

    def test_material_census_nested_solid_in_cavity(self):
        # The nested case gap #8 targets: a solid, a void cavity inside it,
        # and a second solid floating inside that cavity -> 2 solids, 1 void,
        # counted unambiguously because SMLib stores each as its own region.
        outer = sm.create_box((0, 0, 0), 10, 10, 10)
        inner = sm.create_box((3, 3, 3), 4, 4, 4)  # cavity spans 3..7
        hollow = sm.boolean_difference(outer, inner)
        nested = sm.create_box((4, 4, 4), 2, 2, 2)  # 4..6, inside the cavity
        result = sm.merge_breps([hollow, nested], operation=sm.BooleanOp.UNION)
        self.assertEqual(sm.material_census(result),
                         {"solid_count": 2, "void_count": 1})

    def test_compute_volume(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        vol = sm.compute_volume(box)
        self.assertAlmostEqual(vol, 1000.0, delta=0.1)

    def test_compute_volume_rejects_non_finite_accuracy(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for accuracy in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(relative_accuracy=accuracy):
                with self.assertRaises(RuntimeError) as error:
                    sm.compute_volume(box, relative_accuracy=accuracy)
                self.assertIn("SM_ERR_INVALID_INPUT", str(error.exception))

    def test_compute_volume_sphere(self):
        sph = sm.create_sphere((0, 0, 0), 5)
        vol = sm.compute_volume(sph)
        expected = (4.0 / 3.0) * math.pi * 125
        self.assertAlmostEqual(vol, expected, delta=1.0)

    def test_brep_relationship_separate(self):
        a = sm.create_box((0, 0, 0), 1, 1, 1)
        b = sm.create_box((4, 0, 0), 1, 1, 1)
        r = sm.brep_relationship(a, b)
        self.assertEqual(r["relationship"], "separate")
        self.assertAlmostEqual(r["distance"], 3.0, delta=1.0e-4)

    def test_brep_relationship_touching(self):
        a = sm.create_box((0, 0, 0), 1, 1, 1)
        b = sm.create_box((1, 0, 0), 1, 1, 1)
        r = sm.brep_relationship(a, b)
        self.assertEqual(r["relationship"], "touching")
        self.assertEqual(r["distance"], 0.0)

    def test_brep_relationship_interpenetrating(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((8, 3, 3), 4, 4, 4)  # straddles the x=10 wall
        r = sm.brep_relationship(a, b)
        self.assertEqual(r["relationship"], "interpenetrating")
        self.assertEqual(r["distance"], 0.0)

    def test_brep_relationship_a_contains_b(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((3, 3, 3), 2, 2, 2)  # fully inside, no contact
        r = sm.brep_relationship(a, b)
        self.assertEqual(r["relationship"], "a_contains_b")
        self.assertAlmostEqual(r["distance"], 3.0, delta=1.0e-4)  # wall clearance to nearest cavity wall

    def test_brep_relationship_b_contains_a(self):
        a = sm.create_box((3, 3, 3), 2, 2, 2)
        b = sm.create_box((0, 0, 0), 10, 10, 10)
        r = sm.brep_relationship(a, b)
        self.assertEqual(r["relationship"], "b_contains_a")

    def test_brep_relationship_method_matches_free_function(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((3, 3, 3), 2, 2, 2)
        self.assertEqual(a.relationship_to(b), sm.brep_relationship(a, b))

    def test_brep_distance_disjoint(self):
        a = sm.create_box((0, 0, 0), 1, 1, 1)
        b = sm.create_box((4, 0, 0), 1, 1, 1)
        self.assertAlmostEqual(sm.brep_distance(a, b), 3.0, delta=1.0e-4)

    def test_brep_distance_symmetric(self):
        a = sm.create_box((0, 0, 0), 1, 1, 1)
        b = sm.create_box((4, 0, 0), 1, 1, 1)
        self.assertAlmostEqual(sm.brep_distance(a, b), sm.brep_distance(b, a), delta=1.0e-9)

    def test_brep_distance_touching_is_zero(self):
        a = sm.create_box((0, 0, 0), 1, 1, 1)
        b = sm.create_box((1, 0, 0), 1, 1, 1)
        self.assertAlmostEqual(sm.brep_distance(a, b), 0.0, delta=1.0e-4)

    def test_brep_distance_interpenetrating_is_zero(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((8, 3, 3), 4, 4, 4)
        self.assertAlmostEqual(sm.brep_distance(a, b), 0.0, delta=1.0e-4)

    def test_brep_distance_enclosed_is_wall_clearance(self):
        # Surface-gap semantics: a fully-contained body reports the wall
        # clearance (gap to the nearest cavity wall), not 0. [3,5]^3 inside
        # [0,10]^3 -> 3.0. The relationship label says it is containment.
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        b = sm.create_box((3, 3, 3), 2, 2, 2)
        self.assertAlmostEqual(sm.brep_distance(a, b), 3.0, delta=1.0e-4)

    def test_brep_distance_matches_relationship_distance(self):
        a = sm.create_box((0, 0, 0), 10, 10, 10)
        for b in (sm.create_box((20, 0, 0), 1, 1, 1),   # separate
                  sm.create_box((3, 3, 3), 2, 2, 2)):    # enclosed
            self.assertEqual(sm.brep_distance(a, b), sm.brep_relationship(a, b)["distance"])

    def test_brep_distance_method_matches_free_function(self):
        a = sm.create_box((0, 0, 0), 1, 1, 1)
        b = sm.create_box((4, 0, 0), 1, 1, 1)
        self.assertAlmostEqual(a.distance_to(b), sm.brep_distance(a, b), delta=1.0e-9)

    def test_mass_properties_box(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mp = box.mass_properties(relative_accuracy=1.0e-4)
        self.assertAlmostEqual(mp["volume"], 1000.0, delta=0.5)
        self.assertAlmostEqual(mp["area"], 600.0, delta=0.5)
        self.assertAlmostEqual(mp["mass"], 1000.0, delta=0.5)  # density 1.0
        cx, cy, cz = mp["centroid"]
        self.assertAlmostEqual(cx, 5.0, places=3)
        self.assertAlmostEqual(cy, 5.0, places=3)
        self.assertAlmostEqual(cz, 5.0, places=3)
        # Solid cube of side a about its centroid: Ixx = V * (a^2 + a^2) / 12.
        expected_moi = 1000.0 * (100.0 + 100.0) / 12.0
        for moment in mp["moments_of_inertia"]:
            self.assertAlmostEqual(moment, expected_moi, delta=expected_moi * 1.0e-2)
        for product in mp["products_of_inertia"]:
            self.assertAlmostEqual(product, 0.0, delta=5.0)  # ~0 by symmetry

    def test_mass_properties_density_scales_mass(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        reference = box.mass_properties(density=1.0)
        scaled = box.mass_properties(density=3.0)
        self.assertAlmostEqual(scaled["mass"], 3.0 * scaled["volume"], delta=1.0e-3)
        self.assertAlmostEqual(scaled["mass"], 3.0 * reference["mass"], delta=1.0e-3)

        # The inertia tensor is density-weighted (physical mass moments), so
        # each moment/product of inertia must scale linearly with density on
        # the same factor as the mass; this catches any divergence between the
        # mass and inertia scaling paths.
        for scaled_moment, reference_moment in zip(
                scaled["moments_of_inertia"],
                reference["moments_of_inertia"], strict=True):
            self.assertAlmostEqual(scaled_moment, 3.0 * reference_moment,
                                   delta=3.0 * abs(reference_moment) * 1.0e-2 + 1.0e-6)
        for scaled_product, reference_product in zip(
                scaled["products_of_inertia"],
                reference["products_of_inertia"], strict=True):
            self.assertAlmostEqual(scaled_product, 3.0 * reference_product,
                                   delta=3.0 * abs(reference_product) * 1.0e-2 + 5.0)

    def test_mass_properties_invalid_density_raises(self):
        # Non-positive / non-finite densities are rejected (the kernel flags
        # neither), as are sub-SM_EFF_ZERO values it would floor to 1.0.
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for bad in (0.0, -1.0, -1e-9, float("nan"),
                    float("inf"), float("-inf"), 5e-13, 1e-13):
            with self.subTest(density=bad):
                with self.assertRaises(RuntimeError) as ctx:
                    box.mass_properties(density=bad)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
        # The exact boundary SM_EFF_ZERO (1e-12) and any larger density are
        # honoured (the kernel floors only strictly-smaller values).
        for ok in (1e-12, 1e-9):
            with self.subTest(density=ok):
                mp = box.mass_properties(density=ok)
                self.assertAlmostEqual(mp["mass"], ok * mp["volume"], delta=ok)

    def test_mass_properties_invalid_accuracy_raises(self):
        # Non-finite accuracy is rejected before the clamp (NaN would slip past
        # both comparisons); finite out-of-range values still clamp and succeed.
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(relative_accuracy=bad):
                with self.assertRaises(RuntimeError) as ctx:
                    box.mass_properties(relative_accuracy=bad)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))
        for ok in (1e-9, 10.0):
            with self.subTest(relative_accuracy=ok):
                mp = box.mass_properties(relative_accuracy=ok)
                self.assertAlmostEqual(mp["volume"], 1000.0, delta=1.0)

    def test_brep_area_box(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertAlmostEqual(box.area(), 600.0, delta=1.0e-6)

    def test_brep_area_curved_matches_faces_and_mass_properties(self):
        radius = 5.0
        cases = (
            ("sphere", sm.create_sphere((0, 0, 0), radius), 4.0 * math.pi * radius**2),
            ("torus", sm.create_torus((0, 0, 0), radius, 1.0), 4.0 * math.pi**2 * radius),
        )
        for name, brep, expected in cases:
            with self.subTest(shape=name):
                area = brep.area(relative_accuracy=1.0e-4)
                self.assertLessEqual(abs(area - expected) / expected, 1.0e-3)
                face_sum = sum(face.area(relative_accuracy=1.0e-4) for face in brep.faces())
                self.assertEqual(area, face_sum)
                mp_area = brep.mass_properties(relative_accuracy=1.0e-4)["area"]
                self.assertLessEqual(abs(area - mp_area) / expected, 1.0e-3)

    def test_brep_area_sheet_body(self):
        # mass_properties() rejects a sheet body; area() does not need a solid.
        radius = 5.0
        disk = sm.create_planar_circle((0, 0, 0), radius)
        with self.assertRaises(RuntimeError):
            disk.mass_properties()
        expected = math.pi * radius**2
        self.assertLessEqual(abs(disk.area() - expected) / expected, 1.0e-3)

    def test_brep_area_rejects_non_finite_accuracy(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(relative_accuracy=bad):
                with self.assertRaises(RuntimeError) as ctx:
                    box.area(relative_accuracy=bad)
                self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))

    def test_mass_properties_non_solid_raises(self):
        # A sheet body is rejected rather than returning a plausible all-zero
        # result with the bbox midpoint as its "centroid".
        disk = sm.create_planar_circle((0, 0, 0), 5.0)
        self.assertFalse(disk.is_manifold_solid())
        with self.assertRaises(RuntimeError) as ctx:
            disk.mass_properties()
        self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))

    def test_mass_properties_centroid_offset_from_bbox_midpoint(self):
        # The centroid is recovered from the Pass-1 static moment (sMoments[4]).
        # For any axis-aligned box the centroid equals the bounding-box
        # midpoint, so that static moment is identically zero and the recovery
        # formula would pass even if the wrong array slot were read. Use an
        # L-shaped union of two different-sized boxes whose centroid is
        # displaced from the bounding-box midpoint, giving a non-zero static
        # moment that actually exercises the formula.
        base  = sm.create_box((0, 0, 0), 20, 10, 10)  # x[0,20] y[0,10] z[0,10]
        tower = sm.create_box((0, 0, 5), 10, 10, 15)  # x[0,10] y[0,10] z[5,20]
        solid = sm.boolean_union(base, tower)

        # Inclusion-exclusion over the two boxes (intersection is the
        # x[0,10] y[0,10] z[5,10] block, V=500, centroid (5,5,7.5)):
        #   V        = 2000 + 1500 - 500 = 3000
        #   centroid = (25000, 15000, 25000) / 3000 = (25/3, 5, 25/3)
        mp = solid.mass_properties(relative_accuracy=1.0e-4)
        self.assertAlmostEqual(mp["volume"], 3000.0, delta=1.0)
        cx, cy, cz = mp["centroid"]
        self.assertAlmostEqual(cx, 25.0 / 3.0, delta=0.05)
        self.assertAlmostEqual(cy, 5.0, delta=0.05)
        self.assertAlmostEqual(cz, 25.0 / 3.0, delta=0.05)

        # Guard the premise: the centroid must be displaced from the
        # bounding-box midpoint (10, 5, 10) in x and z, so a wrong slot in
        # place of the static moment [4] would produce a different centroid.
        (mnx, _, mnz), (mxx, _, mxz) = solid.bounding_box()
        self.assertGreater(abs(cx - 0.5 * (mnx + mxx)), 1.0)
        self.assertGreater(abs(cz - 0.5 * (mnz + mxz)), 1.0)

        # The static moment [4] is density-weighted, so the centroid must be
        # recovered by dividing by mass (not volume). With a non-zero offset
        # any density must yield the same geometric centroid; dividing by
        # volume instead would scale the offset by the density.
        for density in (0.5, 3.0):
            with self.subTest(density=density):
                mpd = solid.mass_properties(relative_accuracy=1.0e-4,
                                            density=density)
                dcx, dcy, dcz = mpd["centroid"]
                self.assertAlmostEqual(dcx, 25.0 / 3.0, delta=0.05)
                self.assertAlmostEqual(dcy, 5.0, delta=0.05)
                self.assertAlmostEqual(dcz, 25.0 / 3.0, delta=0.05)

    def test_mass_properties_origin_parallel_axis(self):
        # An explicit origin returns moments/products about that point in a
        # single pass; they must satisfy the parallel-axis theorem relative to
        # the centroidal (default) result, and the reported centroid must not
        # depend on the origin.
        box = sm.create_box((0, 0, 0), 2, 4, 6)
        density = 3.0
        centroidal = box.mass_properties(density=density)
        about_o = box.mass_properties(density=density, origin=(0, 0, 0))

        for got, exp in zip(about_o["centroid"], centroidal["centroid"]):
            self.assertAlmostEqual(got, exp, places=3)

        m = centroidal["mass"]
        cx, cy, cz = centroidal["centroid"]
        moi_c = centroidal["moments_of_inertia"]
        poi_c = centroidal["products_of_inertia"]
        # I(O) = I(centroid) + m * (parallel-axis shift)
        exp_moi = (moi_c[0] + m * (cy * cy + cz * cz),
                   moi_c[1] + m * (cx * cx + cz * cz),
                   moi_c[2] + m * (cx * cx + cy * cy))
        exp_poi = (poi_c[0] + m * cy * cz,   # Iyz
                   poi_c[1] + m * cz * cx,   # Izx
                   poi_c[2] + m * cx * cy)   # Ixy
        for got, exp in zip(about_o["moments_of_inertia"], exp_moi):
            self.assertAlmostEqual(got, exp, delta=abs(exp) * 1.0e-3 + 1.0e-6)
        for got, exp in zip(about_o["products_of_inertia"], exp_poi):
            self.assertAlmostEqual(got, exp, delta=abs(exp) * 1.0e-3 + 1.0e-6)

        # Passing the centroid as the origin reproduces the centroidal default.
        about_c = box.mass_properties(density=density,
                                      origin=centroidal["centroid"])
        for got, exp in zip(about_c["moments_of_inertia"], moi_c):
            self.assertAlmostEqual(got, exp, delta=abs(exp) * 1.0e-3 + 1.0e-6)


# -----------------------------------------------------------------------
#  Topology accessors and geometric properties on Brep / Face / Edge /
#  Vertex (method-form bindings; flat sm.queries.* aliases are tested
#  indirectly above).
# -----------------------------------------------------------------------
class TestTopologyAccessors(unittest.TestCase):
    def assertPointAlmostEqual(self, actual, expected, places=3):
        for a, b in zip(actual, expected, strict=True):
            self.assertAlmostEqual(a, b, places=places)

    def _unit(self, vector):
        length = math.sqrt(sum(component * component for component in vector))
        self.assertGreater(length, 0.0)
        return tuple(component / length for component in vector)

    def _ray_from_box_center_through_point(self, brep, point, distance=20.0):
        center = brep.center()
        outward = self._unit(tuple(point[i] - center[i] for i in range(3)))
        origin = tuple(point[i] + distance * outward[i] for i in range(3))
        direction = tuple(-component for component in outward)
        return origin, direction

    def _assert_pick_hit_shape(self, hit, kind):
        self.assertEqual(hit["kind"], kind)
        self.assertEqual(hit["topology_kind"], kind)
        self.assertIn("topology_index", hit)
        self.assertIn("topology", hit)
        self.assertIn("point", hit)
        self.assertIn("parameters", hit)
        self.assertIn("line_parameter", hit)
        self.assertIn("ray_depth", hit)
        self.assertIn("normal", hit)
        self.assertGreaterEqual(hit["ray_depth"], -1.0e-6)

    def _assert_boundary_loop_closed(self, loop):
        self.assertGreater(len(loop), 0)
        oriented_vertices = []
        for edge, forward in loop:
            self.assertIsInstance(edge, sm.Edge)
            self.assertIsInstance(forward, bool)
            start, end = edge.vertices()
            oriented_vertices.append((start, end) if forward else (end, start))
        for index, (_, end) in enumerate(oriented_vertices):
            next_start, _ = oriented_vertices[(index + 1) % len(oriented_vertices)]
            self.assertIs(end, next_start)

    def _boundary_loop_signed_area(self, face, loop):
        points = []
        for edge, forward in loop:
            start, end = edge.vertices()
            points.append((start if forward else end).point())

        area_vector = [0.0, 0.0, 0.0]
        for point, next_point in zip(points, points[1:] + points[:1], strict=True):
            area_vector[0] += point[1] * next_point[2] - point[2] * next_point[1]
            area_vector[1] += point[2] * next_point[0] - point[0] * next_point[2]
            area_vector[2] += point[0] * next_point[1] - point[1] * next_point[0]

        (umin, vmin), (umax, vmax) = face.uv_domain()
        normal = face.normal_at_uv(0.5 * (umin + umax), 0.5 * (vmin + vmax))
        return 0.5 * sum(area_vector[i] * normal[i] for i in range(3))

    # -- Brep methods --------------------------------------------------
    def test_brep_bounding_box(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mn, mx = box.bounding_box()
        self.assertAlmostEqual(mn[0], 0.0, places=3)
        self.assertAlmostEqual(mx[2], 10.0, places=3)

    def test_brep_center(self):
        box = sm.create_box((1, 2, 3), 10, 10, 10)
        c = box.center()
        self.assertAlmostEqual(c[0], 6.0, places=3)
        self.assertAlmostEqual(c[1], 7.0, places=3)
        self.assertAlmostEqual(c[2], 8.0, places=3)

    def test_brep_volume_method(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertAlmostEqual(box.volume(), 1000.0, delta=0.1)

    def test_brep_is_manifold_solid_method(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertTrue(box.is_manifold_solid())

    def test_brep_counts(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertEqual(box.face_count(), 6)
        self.assertEqual(box.edge_count(), 12)
        self.assertEqual(box.vertex_count(), 8)

    def test_brep_copy_is_independent(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        clone = box.copy()
        sm.translate(box, (100, 0, 0))
        # Clone is unaffected by the in-place translate.
        clone_min = clone.bounding_box()[0]
        self.assertAlmostEqual(clone_min[0], 0.0, places=3)

    def test_brep_copy_module_is_independent(self):
        for copier in (copy.copy, copy.deepcopy):
            with self.subTest(copier=copier.__name__):
                box = sm.create_box((0, 0, 0), 10, 10, 10)
                clone = copier(box)
                self.assertIsInstance(clone, sm.Brep)
                self.assertIsNot(clone, box)
                sm.translate(box, (100, 0, 0))
                self.assertAlmostEqual(clone.bounding_box()[0][0], 0.0, places=3)
        # deepcopy keeps shared references shared.
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        first, second = copy.deepcopy([box, box])
        self.assertIs(first, second)
        self.assertIsNot(first, box)

    # -- Face methods --------------------------------------------------
    def test_face_bounding_box(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        mn, mx = face.bounding_box()
        self.assertEqual(len(mn), 3)
        self.assertEqual(len(mx), 3)

    def test_face_area(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        # All six faces of a 10x10x10 box have area 100.
        for face in box.faces():
            self.assertAlmostEqual(face.area(), 100.0, delta=0.1)

    def test_face_interior_point(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for face in box.faces():
            pt = face.interior_point()
            self.assertEqual(len(pt), 3)
            # The interior point must lie within the face's (loose) box.
            mn, mx = face.bounding_box()
            for axis in range(3):
                self.assertGreaterEqual(pt[axis], mn[axis] - 1.0e-4)
                self.assertLessEqual(pt[axis], mx[axis] + 1.0e-4)
            # Stronger guarantee than bbox containment: project the returned
            # 3D point back onto the surface and confirm the trim logic itself
            # classifies that UV as strictly inside the face.
            _, uvs, _ = sm.surface_closest_point(face.surface(), pt)
            self.assertTrue(
                any(face.classify_uv(uv.x, uv.y) == "inside" for uv in uvs)
            )

    def test_face_classify_uv(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        (umin, vmin), (umax, vmax) = face.uv_domain()
        u_mid, v_mid = 0.5 * (umin + umax), 0.5 * (vmin + vmax)
        # Centre of the trim domain is inside the face.
        self.assertEqual(face.classify_uv(u_mid, v_mid), "inside")
        # Domain edge and corner should classify as boundary / vertex,
        # locking down the SM_PC_EDGE and SM_PC_VERTEX string mappings.
        self.assertEqual(face.classify_uv(umin, v_mid), "boundary")
        self.assertEqual(face.classify_uv(umin, vmin), "vertex")
        # Well outside the trim domain is outside the face.
        du, dv = (umax - umin), (vmax - vmin)
        self.assertEqual(face.classify_uv(umax + du, vmax + dv), "outside")

    def test_face_closest_point(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for face in box.faces():
            # Querying interior_point() itself must return it at ~0 distance.
            pt = face.interior_point()
            closest, dist = face.closest_point(pt)
            self.assertLess(dist, 1.0e-6)
            for a, b in zip(closest, pt, strict=True):
                self.assertAlmostEqual(a, b, places=4)
            # Trim-awareness: even far off to one side, the result must
            # stay within the trim (never "outside"; a diagonal offset
            # commonly lands on a corner, i.e. "vertex").
            _, mx = face.bounding_box()
            far_side = tuple(m + 50.0 for m in mx)
            closest_edge, _dist_edge = face.closest_point(far_side)
            _, uvs, _ = sm.surface_closest_point(face.surface(), closest_edge)
            self.assertTrue(any(
                face.classify_uv(uv.x, uv.y) in ("inside", "boundary", "vertex")
                for uv in uvs
            ))

    def test_face_centroid(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for face in box.faces():
            c = face.centroid()
            self.assertEqual(len(c), 3)
            mn, mx = face.bounding_box()
            for axis in range(3):
                self.assertGreaterEqual(c[axis], mn[axis] - 1.0e-4)
                self.assertLessEqual(c[axis], mx[axis] + 1.0e-4)
            # A box face is a square, so its centroid is its bbox midpoint.
            for axis in range(3):
                self.assertAlmostEqual(c[axis], 0.5 * (mn[axis] + mx[axis]), places=3)

    def test_face_centroid_on_sphere_faces_with_poles(self):
        # Wedge over azimuth [0, a]: curved-face centroid is
        # r*pi/(4a) * (sin a, 1 - cos a, 0).
        radius = 5.0
        full = sm.create_sphere((0, 0, 0), radius).faces()[0]
        for got in full.centroid(relative_accuracy=1.0e-4):
            self.assertAlmostEqual(got, 0.0, delta=1.0e-3 * radius)
        for end_deg in (90.0, 180.0, 270.0):
            with self.subTest(end_deg=end_deg):
                wedge = sm.create_partial_sphere((0, 0, 0), radius, 0.0, end_deg)
                curved = max(wedge.faces(), key=lambda face: face.area())
                a = math.radians(end_deg)
                scale = radius * math.pi / (4.0 * a)
                expected = (scale * math.sin(a), scale * (1.0 - math.cos(a)), 0.0)
                got = curved.centroid(relative_accuracy=1.0e-4)
                for axis in range(3):
                    self.assertAlmostEqual(got[axis], expected[axis], delta=1.0e-3 * radius)

    def test_face_area_reversed_orientation_circular_caps(self):
        # Regression: face.area() previously returned 0.0 for a reversed-
        # orientation planar cap face (one of the two circular caps of a
        # cylinder / pipe).  The precise-property integrator aborted on the
        # first, over-coarse Runge-Kutta step for one traversal direction and
        # silently dropped the whole boundary loop.  Both caps must report the
        # true area pi*r^2, independent of orientation.
        def cap_areas(brep):
            caps = [f for f in brep.faces() if len(f.edges()) == 1]
            return sorted(f.area() for f in caps)

        r = 0.5
        cyl = sm.create_cylinder((0, 0, 0), r, 4.0)
        caps = cap_areas(cyl)
        self.assertEqual(len(caps), 2)
        expected = math.pi * r * r
        for a in caps:
            self.assertGreater(a, 0.0, "cap area must not be zero")
            self.assertLessEqual(abs(a - expected), 1e-3 * expected)
        # The two caps of a straight cylinder are congruent, but the
        # correctly-oriented cap integrates with the historical span/2 initial
        # step while the reversed cap falls back to a finer step, so their
        # numeric areas can differ by up to twice the requested accuracy.
        self.assertLessEqual(abs(caps[0] - caps[1]), 2e-3 * expected)

        # Same defect surfaced through pipe_sweep along straight and curved
        # paths -- exercise both, with a tilted (non-axis-aligned) cap.
        pr = 0.3
        for path in (
            sm.create_line_segment((0, 0, 0), (0, 0, 5)),
            sm.create_arc((0, 0, 0), 5.0, 0.0, 120.0),
        ):
            caps = cap_areas(sm.pipe_sweep(path, radius=pr))
            self.assertEqual(len(caps), 2)
            for a in caps:
                self.assertGreater(a, 0.0, "pipe cap area must not be zero")
                self.assertAlmostEqual(a, math.pi * pr * pr, delta=1e-3)

    def test_face_area_reversed_caps_scaled_deterministic(self):
        # Contract across scales/accuracies: each cap area call must either
        # return a positive value near pi*r^2 or raise -- never succeed with a
        # silently dropped (zero/partial) boundary.
        for r in (0.5, 50000.0):
            expected = math.pi * r * r
            cyl = sm.create_cylinder((0, 0, 0), r, 4.0 * r)
            caps = [f for f in cyl.faces() if len(f.edges()) == 1]
            self.assertEqual(len(caps), 2)
            for accuracy in (1.0e-3, 1.0e-4):
                for f in caps:
                    with self.subTest(r=r, accuracy=accuracy):
                        try:
                            a = f.area(relative_accuracy=accuracy)
                        except RuntimeError:
                            continue  # deterministic failure is acceptable
                        # 1% band: "clearly pi*r^2, not zero", not an exact bound.
                        self.assertGreater(a, 0.0, "cap area must not be zero")
                        self.assertLessEqual(abs(a - expected), 1.0e-2 * expected,
                                             "cap area not close to pi*r^2")

    def test_mass_properties_unaffected_by_cap_fix(self):
        # Keeping span/2 as the first step leaves already-converged integrations
        # unchanged: exact box area/volume, and a radius-5 sphere still within
        # the requested 0.1% (an /8-first schedule drifted it to +0.115%).
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertAlmostEqual(box.volume(), 1000.0, delta=1e-6)
        self.assertAlmostEqual(sum(f.area() for f in box.faces()), 600.0,
                               delta=1e-6)

        r = 5.0
        analytic = 4.0 / 3.0 * math.pi * r ** 3
        sphere_vol = sm.create_sphere((0, 0, 0), r).volume(relative_accuracy=1.0e-3)
        self.assertLessEqual(abs(sphere_vol - analytic), 1.0e-3 * analytic,
                             "sphere volume outside requested 0.1% accuracy")

    def test_face_surface(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        srf = box.faces()[0].surface()
        self.assertIsInstance(srf, sm.Surface)

    def test_face_uv_domain(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        (umin, vmin), (umax, vmax) = box.faces()[0].uv_domain()
        self.assertLess(umin, umax)
        self.assertLess(vmin, vmax)

    def test_face_topology_accessors(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        # A box face: four edges and four vertices on the outer loop.
        self.assertEqual(len(face.edges()), 4)
        self.assertEqual(len(face.vertices()), 4)
        self.assertEqual(len(face.outer_loop_edges()), 4)
        loops = face.boundary_loops()
        outer = face.boundary_outer_loop()
        self.assertEqual([len(loop) for loop in loops], [4])
        self._assert_boundary_loop_closed(outer)
        for (outer_edge, outer_forward), (loop_edge, loop_forward) in zip(
                outer, loops[0], strict=True):
            self.assertIs(outer_edge, loop_edge)
            self.assertEqual(outer_forward, loop_forward)

    def test_face_boundary_loops_order_orientation_and_outer_selection(self):
        outer = sm.create_planar_faces(sm.create_rectangle((0, 0, 0), 10, 10))
        inner = sm.create_planar_faces(sm.create_rectangle((0, 0, 0), 4, 4))
        annulus = sm.boolean_2d(outer, inner, operation=sm.BooleanOp2D.DIFFERENCE)
        self.assertEqual(annulus.face_count(), 1)

        face = annulus.faces()[0]
        loops = face.boundary_loops()
        self.assertEqual([len(loop) for loop in loops], [4, 4])
        self.assertEqual(len(face.edges()), 8)
        self.assertEqual(len(face.outer_loop_edges()), 4)

        owner_edges = annulus.edges()
        for loop in loops:
            self._assert_boundary_loop_closed(loop)
            for edge, _ in loop:
                self.assertTrue(any(edge is owner_edge for owner_edge in owner_edges))

        outer_loop = face.boundary_outer_loop()
        self.assertEqual(len(outer_loop), 4)
        for (outer_edge, outer_forward), (loop_edge, loop_forward) in zip(
                outer_loop, loops[0], strict=True):
            self.assertIs(outer_edge, loop_edge)
            self.assertEqual(outer_forward, loop_forward)

        signed_areas = [self._boundary_loop_signed_area(face, loop) for loop in loops]
        self.assertAlmostEqual(signed_areas[0], 100.0, places=6)
        self.assertAlmostEqual(signed_areas[1], -16.0, places=6)

    def test_face_boundary_loop_preserves_periodic_seam_occurrences(self):
        cylinder = sm.create_cylinder((0, 0, 0), 2, 5)
        side_faces = [face for face in cylinder.faces() if len(face.edges()) == 3]
        self.assertEqual(len(side_faces), 1)

        side = side_faces[0]
        loops = side.boundary_loops()
        self.assertEqual(len(loops), 1)
        loop = loops[0]
        self.assertEqual(len(loop), 4)
        self._assert_boundary_loop_closed(loop)

        occurrence_groups = []
        for edge, forward in loop:
            for grouped_edge, directions in occurrence_groups:
                if edge is grouped_edge:
                    directions.append(forward)
                    break
            else:
                occurrence_groups.append((edge, [forward]))

        self.assertEqual(sorted(len(directions) for _, directions in occurrence_groups),
                         [1, 1, 2])
        seam_directions = next(directions for _, directions in occurrence_groups
                               if len(directions) == 2)
        self.assertEqual(set(seam_directions), {False, True})

    def test_face_point_at_uv(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        (umin, vmin), (umax, vmax) = face.uv_domain()
        u_mid = 0.5 * (umin + umax)
        v_mid = 0.5 * (vmin + vmax)
        pt = face.point_at_uv(u_mid, v_mid)
        # Mid-uv must lie inside the face's bounding box.
        mn, mx = face.bounding_box()
        self.assertGreaterEqual(pt[0], mn[0] - 1e-6)
        self.assertLessEqual(pt[0], mx[0] + 1e-6)

    def test_face_normal_at_uv_unit_length(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        (umin, vmin), (umax, vmax) = face.uv_domain()
        n = face.normal_at_uv(0.5 * (umin + umax), 0.5 * (vmin + vmax))
        mag = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
        self.assertAlmostEqual(mag, 1.0, places=3)

    def test_face_outward_normal_at_uv_points_away_from_box_center(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        center = box.center()
        for face in box.faces():
            (umin, vmin), (umax, vmax) = face.uv_domain()
            u_mid = 0.5 * (umin + umax)
            v_mid = 0.5 * (vmin + vmax)
            point = face.point_at_uv(u_mid, v_mid)
            normal = face.outward_normal_at_uv(u_mid, v_mid)
            mag = math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2)
            self.assertAlmostEqual(mag, 1.0, places=3)
            away = (
                point[0] - center[0],
                point[1] - center[1],
                point[2] - center[2],
            )
            dot = away[0] * normal[0] + away[1] * normal[1] + away[2] * normal[2]
            self.assertGreater(dot, 0.0)

    def test_face_brep_round_trip(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertIs(box.faces()[0].brep(), box)

    # -- Edge methods --------------------------------------------------
    def test_edge_length(self):
        box = sm.create_box((0, 0, 0), 10, 7, 3)
        # Box has 12 edges, all axis-aligned with lengths 10, 7, or 3.
        lengths = sorted({round(e.length(), 3) for e in box.edges()})
        self.assertEqual(lengths, [3.0, 7.0, 10.0])

    def test_edge_curve(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        crv = box.edges()[0].curve()
        self.assertIsInstance(crv, sm.Curve)

    def test_edge_endpoints(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        v_start = edge.start_vertex()
        v_end = edge.end_vertex()
        self.assertIsInstance(v_start, sm.Vertex)
        self.assertIsInstance(v_end, sm.Vertex)
        # vertices() returns the same pair as a tuple.
        vs, ve = edge.vertices()
        self.assertIs(vs, v_start)
        self.assertIs(ve, v_end)
        # other_vertex flips the endpoint.
        self.assertIs(edge.other_vertex(v_start), v_end)
        self.assertIs(edge.other_vertex(v_end), v_start)

    def test_edge_faces(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        # Every edge of a closed manifold solid is shared by exactly two faces.
        for edge in box.edges():
            self.assertEqual(len(edge.faces()), 2)

    def test_edge_point_at_endpoints(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        # Evaluating at an endpoint vertex should match the vertex position.
        v_start = edge.start_vertex().point()
        # Use the curve's parameter at the start vertex via closest_point.
        closest, t, _ = sm.curve_closest_point(edge.curve(), v_start)
        pt = edge.point_at(t)
        for a, b in zip(pt, v_start):
            self.assertAlmostEqual(a, b, places=3)

    def test_edge_parameter_range(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        t_min, t_max = edge.parameter_range()
        self.assertLess(t_min, t_max)
        # Sub-range of the curve's full natural range.
        c_min, c_max = edge.curve().parameter_range()
        self.assertGreaterEqual(t_min, c_min - 1.0e-9)
        self.assertLessEqual(t_max, c_max + 1.0e-9)

    def test_edge_classify_parameter(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        t_min, t_max = edge.parameter_range()
        t_mid = 0.5 * (t_min + t_max)
        self.assertEqual(edge.classify_parameter(t_mid), "inside")
        self.assertEqual(edge.classify_parameter(t_min), "endpoint")
        self.assertEqual(edge.classify_parameter(t_max), "endpoint")
        span = t_max - t_min
        self.assertEqual(edge.classify_parameter(t_min - span), "outside")
        self.assertEqual(edge.classify_parameter(t_max + span), "outside")

    def test_edge_tangent_at(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        t_min, t_max = edge.parameter_range()
        t_mid = 0.5 * (t_min + t_max)
        tangent = edge.tangent_at(t_mid)
        self.assertEqual(len(tangent), 3)
        length = math.sqrt(sum(c * c for c in tangent))
        self.assertAlmostEqual(length, 1.0, places=6)
        # Box edges are axis-aligned: tangent is a signed unit basis vector.
        near_unit = sum(1 for c in tangent if abs(abs(c) - 1.0) < 1e-6)
        near_zero = sum(1 for c in tangent if abs(c) < 1e-6)
        self.assertEqual(near_unit, 1)
        self.assertEqual(near_zero, 2)

    def test_edge_closest_point(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        v_start = edge.start_vertex().point()
        # Querying the start vertex itself must return ~itself, ~0 distance.
        closest, param, dist = edge.closest_point(v_start)
        self.assertLess(dist, 1.0e-6)
        for a, b in zip(closest, v_start, strict=True):
            self.assertAlmostEqual(a, b, places=4)
        t_min, t_max = edge.parameter_range()
        self.assertIn(edge.classify_parameter(param), ("inside", "endpoint"))
        self.assertGreaterEqual(param, t_min - 1.0e-6)
        self.assertLessEqual(param, t_max + 1.0e-6)

    def test_edge_brep_round_trip(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertIs(box.edges()[0].brep(), box)

    # -- Vertex methods ------------------------------------------------
    def test_vertex_point(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        coords = sorted(v.point() for v in box.vertices())
        # Box has corners at every combination of {0, 10}.
        for c in coords:
            for x in c:
                self.assertIn(round(x, 3), (0.0, 10.0))

    def test_vertex_edges(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        # Every corner of a manifold box is incident on exactly 3 edges.
        for v in box.vertices():
            self.assertEqual(len(v.edges()), 3)

    def test_vertex_faces(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        for v in box.vertices():
            self.assertEqual(len(v.faces()), 3)

    def test_vertex_brep_round_trip(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertIs(box.vertices()[0].brep(), box)

    # -- Repr (smoke test) ---------------------------------------------
    def test_repr_smoke(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertIn("Brep", repr(box))
        self.assertIn("Face", repr(box.faces()[0]))
        self.assertIn("Edge", repr(box.edges()[0]))
        self.assertIn("Vertex", repr(box.vertices()[0]))


# -----------------------------------------------------------------------
#  Brep operations (cut, project)
# -----------------------------------------------------------------------
class TestBrepOps(unittest.TestCase):
    def test_cut(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.cut(box, (0, 0, 5), (0, 0, 1))
        faces_after = len(box.faces())
        self.assertGreater(faces_after, 0)

    def test_project_brep_onto_plane(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        curves = sm.project_brep_onto_plane(box, (0, 0, 0), (0, 0, 1))
        self.assertIsInstance(curves, list)
        self.assertGreater(len(curves), 0)

    def test_create_silhouette_curves(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        curves = sm.create_silhouette_curves(box, (0, 0, 20), (0, 0, 1))
        self.assertIsInstance(curves, list)


# -----------------------------------------------------------------------
#  Trimmed surfaces
# -----------------------------------------------------------------------
class TestTrimmedSurfaces(unittest.TestCase):
    def test_create_planar_faces(self):
        circ = sm.create_circle((0, 0, 0), 5)
        r = sm.create_planar_faces([circ])
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_create_planar_faces_failure_keeps_curves(self):
        curves = [sm.create_line_segment((0, 0, 0), (1, 0, 0)),
                  sm.create_line_segment((1, 0, 0), (0, 1, 0))]
        lengths = [curve.length() for curve in curves]
        with self.assertRaises(RuntimeError):
            sm.create_planar_faces(curves)  # An open loop cannot bound a face.
        self.assertEqual([curve.length() for curve in curves], lengths)

        # Repair the rejected loop and reuse the original curves successfully.
        curves.append(sm.create_line_segment((0, 1, 0), (0, 0, 0)))
        result = sm.create_planar_faces(curves)
        self.assertEqual(result.face_count(), 1)
        self.assertAlmostEqual(result.faces()[0].area(), 0.5)

    def test_trim_project_parallel(self):
        srf = sm.create_surface_from_corner_points(
            (0, 0, 0), (25, 0, 0), (0, 25, 0), (25, 25, 0))
        line = sm.create_line_segment((-5, 12.5, 10), (30, 12.5, 10))
        r = sm.trim_project_parallel(
            srf, line, (0, 0, -1), (12.5, 5, 0), sm.TrimType.KEEP_POINT)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_trim_project_parallel_rejects_curve_as_target(self):
        target = sm.create_circle((0, 0, 0), 5)
        projector = sm.create_curve(
            [(-5, 0, 10), (-2, 0, 10), (2, 0, 10), (5, 0, 10)])
        target_center = target.center()
        with self.assertRaises(RuntimeError):
            sm.trim_project_parallel(
                target, projector, (0, 0, -1), (0, 0, 0), sm.TrimType.KEEP_POINT)
        self.assertEqual(target.center(), target_center)

    def test_trim_project_parallel_rejects_brep_owned_surface(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        surface = face.surface()
        (umin, vmin), (umax, vmax) = face.uv_domain()
        uv_mid = (0.5 * (umin + umax), 0.5 * (vmin + vmax))
        projector = sm.create_curve(
            [(-5, 5, 15), (2, 5, 15), (9, 5, 15), (15, 5, 15)])

        box_snapshot = (box.bounding_box(), box.volume(), len(box.faces()))
        face_snapshot = (face.area(), tuple(vertex.point() for vertex in face.vertices()))
        surface_point = surface.evaluate(*uv_mid)

        with self.assertRaisesRegex(RuntimeError, "SM_ERR_INVALID_INPUT"):
            sm.trim_project_parallel(
                surface, projector, (0, 0, -1), (5, 5, 0), sm.TrimType.KEEP_POINT)

        self.assertEqual(box.bounding_box(), box_snapshot[0])
        self.assertAlmostEqual(box.volume(), box_snapshot[1], places=8)
        self.assertEqual(len(box.faces()), box_snapshot[2])
        self.assertAlmostEqual(face.area(), face_snapshot[0], places=8)
        self.assertEqual(tuple(vertex.point() for vertex in face.vertices()), face_snapshot[1])
        for actual, expected in zip(surface.evaluate(*uv_mid), surface_point, strict=True):
            self.assertAlmostEqual(actual, expected, places=8)


# -----------------------------------------------------------------------
#  Heal
# -----------------------------------------------------------------------
class TestHeal(unittest.TestCase):
    def test_heal_brep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sm.heal_brep(box)
        self.assertEqual(len(box.faces()), 6)


# -----------------------------------------------------------------------
#  Closest point (all solutions)
# -----------------------------------------------------------------------
class TestClosestPointAll(unittest.TestCase):
    def test_brep_closest_point_all(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        pts, dists = sm.brep_closest_point_all(box, (5, 5, 15))
        self.assertIsInstance(pts, list)
        self.assertIsInstance(dists, list)
        self.assertGreater(len(pts), 0)

    def test_curve_closest_point_all(self):
        circ = sm.create_circle((0, 0, 0), 5)
        pts, params, dists = sm.curve_closest_point_all(circ, (0, 0, 3))
        self.assertIsInstance(pts, list)
        self.assertGreater(len(pts), 0)

# -----------------------------------------------------------------------
#  Enums (new)
# -----------------------------------------------------------------------
class TestNewEnums(unittest.TestCase):
    def test_trim_type(self):
        self.assertIsNotNone(sm.TrimType.KEEP_POINT)
        self.assertIsNotNone(sm.TrimType.DELETE_POINT)
        self.assertIsNotNone(sm.TrimType.SPLIT)

    def test_continuity_type(self):
        self.assertIsNotNone(sm.ContinuityType.C0)
        self.assertIsNotNone(sm.ContinuityType.G1)
        self.assertIsNotNone(sm.ContinuityType.C1)
        self.assertIsNotNone(sm.ContinuityType.C_INFINITY)


# -----------------------------------------------------------------------
#  USD round-trip
# -----------------------------------------------------------------------
class TestUsd(unittest.TestCase):
    _IMPORT_BREPS_SCRIPT = """
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm

try:
    sm.usd.import_breps(sys.argv[1], heal=False)
except RuntimeError as error:
    print("RuntimeError: %s" % error)
else:
    raise AssertionError("malformed BrepArray unexpectedly imported")
"""

    _TESSELLATE_FILE_SCRIPT = """
import os
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

import _omni_solid as sm

sm.usd.tessellate_file(sys.argv[1], sys.argv[2])
"""

    def _face_range_attrs(self, path):
        with open(path) as fh:
            lines = [line.strip() for line in fh if "face:range" in line]
        self.assertTrue(lines, "export did not author face:range")
        return "\n".join(lines)

    def _face_range_values(self, face_range_text):
        payload = face_range_text.split("=", 1)[1]
        number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
        return [float(value) for value in re.findall(number, payload)]

    def _face_range_faces(self, face_range_text):
        values = self._face_range_values(face_range_text)
        self.assertEqual(len(values) % 4, 0)
        return [values[i:i + 4] for i in range(0, len(values), 4)]

    def _has_infinite_face_bound(self, values):
        return any(abs(value) == 1234567.0 for value in values)

    def _inject_unbounded_first_face_range(self, usda_text):
        pattern = r"(uniform double2\[\] face:range = \[)\([^)]+\), \([^)]+\)"
        replacement = r"\1(-1234567, -1234567), (1234567, 1234567)"
        injected, count = re.subn(pattern, replacement, usda_text, count=1)
        self.assertEqual(count, 1)
        return injected

    def _invalidate_only_sphere_radius(self, usda_text):
        pattern = r"(uniform double\[\] brep:surface:sphere:radius = \[)[^\]]+(\])"
        replacement = r"\g<1>0\2"
        invalidated, count = re.subn(pattern, replacement, usda_text, count=1)
        self.assertEqual(count, 1)
        return invalidated

    def _brep_api_schemas(self, usda_text):
        pattern = re.compile(r'(?m)^(?P<indent>[ \t]*)prepend apiSchemas = \[(?P<tokens>[^\]\r\n]*)\][ \t]*$')
        matches = [
            match for match in pattern.finditer(usda_text)
            if "BrepSurfaceSphereAPI" in match.group("tokens")
        ]
        self.assertEqual(len(matches), 1, "expected one analytic-sphere BrepArray apiSchemas list")
        match = matches[0]
        return match, re.findall(r'"([^"]+)"', match.group("tokens"))

    def _omit_brep_api_schemas(self, usda_text, omitted_schemas):
        match, schemas = self._brep_api_schemas(usda_text)
        self.assertTrue(omitted_schemas)
        self.assertTrue(omitted_schemas.issubset(schemas))
        remaining = [schema for schema in schemas if schema not in omitted_schemas]
        list_operator = "prepend " if remaining else ""
        replacement = '%s%sapiSchemas = [%s]' % (
            match.group("indent"),
            list_operator,
            ", ".join('"%s"' % schema for schema in remaining),
        )
        return usda_text[:match.start()] + replacement + usda_text[match.end():]

    def _assert_import_breps_rejected_without_crash(self, path):
        try:
            completed = subprocess.run(
                [sys.executable, "-c", self._IMPORT_BREPS_SCRIPT, path],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.fail("USD import timed out while rejecting a malformed BrepArray")

        self.assertEqual(
            completed.returncode,
            0,
            msg="child output:\n%s\n%s" % (completed.stdout, completed.stderr),
        )
        output = "%s\n%s" % (completed.stdout, completed.stderr)
        self.assertIn("RuntimeError:", completed.stdout)
        self.assertIn("SmApiUsdImportBreps", completed.stdout)
        self.assertIn("SM_ERR_INVALID_INPUT", completed.stdout, output)
        self.assertRegex(output, r"(?i)malformed BrepArray")

    def _assert_face_ranges_close(self, expected, actual):
        self.assertEqual(len(expected), len(actual))
        for expected_face, actual_face in zip(expected, actual):
            self.assertEqual(len(expected_face), len(actual_face))
            for expected_value, actual_value in zip(expected_face, actual_face):
                self.assertAlmostEqual(expected_value, actual_value, places=12)

    def test_ensure_plugin_registered(self):
        # Idempotent: returns None and is safe to call repeatedly. Also a
        # precondition for the round-trip tests below (they self-register too).
        self.assertIsNone(sm.usd.ensure_plugin_registered())
        self.assertIsNone(sm.usd.ensure_plugin_registered())

    def test_export_import_single_brep(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(box, path)
            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps), 1)
            self.assertEqual(len(breps[0].faces()), 6)
        finally:
            os.unlink(path)

    def test_export_import_multiple_breps(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sph = sm.create_sphere((20, 0, 0), 5)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_breps([box, sph], path)
            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps), 2)

            report = sm.usd.import_breps(path, allow_partial=True)
            self.assertIsInstance(report, sm.usd.BrepImportReport)
            self.assertTrue(report.complete)
            self.assertEqual(len(report.breps), 2)
            self.assertEqual(len(report.items), 2)
            self.assertIsInstance(report.items[0], sm.usd.BrepImportResult)
            self.assertEqual(report.failures, [])
            self.assertEqual([item.prim_path for item in report.items], ["/World/Brep0"] * 2)
            self.assertEqual([item.packed_index for item in report.items], [0, 1])
            self.assertTrue(all(item.ok for item in report.items))
            self.assertEqual({item.status_name for item in report.items}, {"SM_SUCCESS"})
        finally:
            os.unlink(path)

    def test_import_rejects_mixed_valid_and_conversion_invalid_breps(self):
        box = sm.create_box((0, 0, 0), 2, 3, 4)
        sphere = sm.create_sphere((10, 0, 0), 5)
        trailing_box = sm.create_box((20, 0, 0), 1, 2, 3)
        with tempfile.TemporaryDirectory(prefix="smpy_mixed_brep_import_") as tmpdir:
            path = os.path.join(tmpdir, "mixed_valid_invalid.usda")
            sm.usd.export_breps([box, sphere, trailing_box], path, export_uv_curves=False)

            with open(path, encoding="utf-8") as file:
                source_usda = file.read()
            with open(path, "w", encoding="utf-8", newline="") as file:
                file.write(self._invalidate_only_sphere_radius(source_usda))

            cases = (
                ("import_breps", "SmApiUsdImportBreps", lambda: sm.usd.import_breps(path, heal=False)),
                ("import_brep", "SmApiUsdImportBrep", lambda: sm.usd.import_brep(path, "/World/Brep0", heal=False)),
            )
            for name, api_name, import_call in cases:
                with self.subTest(importer=name):
                    with self.assertRaises(RuntimeError) as raised:
                        import_call()
                    message = str(raised.exception)
                    self.assertIn(api_name, message)
                    self.assertIn("/World/Brep0", message)
                    self.assertIn("packed Brep index 1", message)

            with self.assertRaises(TypeError):
                sm.usd.import_breps(path, False, True)

            report = sm.usd.import_breps(path, heal=False, allow_partial=True)
            partial_breps = report.breps
            items = report.items
            failures = report.failures

            self.assertIsInstance(report, sm.usd.BrepImportReport)
            self.assertFalse(report.complete)
            self.assertEqual(len(partial_breps), 2)
            self.assertEqual(len(items), 3)
            self.assertEqual(len(failures), 1)
            self.assertEqual(failures, [item for item in items if not item.ok])
            self.assertEqual(len(set(items + report.items)), 3)
            self.assertEqual({item: i for i, item in enumerate(items)}[failures[0]], 1)
            self.assertNotEqual(items[0], None)
            self.assertFalse(items[0] == object())
            self.assertEqual([item.prim_path for item in items], ["/World/Brep0"] * 3)
            self.assertEqual([item.packed_index for item in items], [0, 1, 2])
            self.assertEqual([item.ok for item in items], [True, False, True])
            self.assertIs(items[0].brep, partial_breps[0])
            self.assertIsNone(items[1].brep)
            self.assertIs(items[2].brep, partial_breps[1])
            self.assertEqual(items[0].status_code, items[2].status_code)
            self.assertNotEqual(items[1].status_code, items[0].status_code)
            self.assertEqual(items[0].status_name, "SM_SUCCESS")
            self.assertRegex(items[1].status_name, r"^SM_ERR")
            self.assertTrue(items[1].message)
            self.assertEqual(failures[0].prim_path, "/World/Brep0")
            self.assertEqual(failures[0].packed_index, 1)
            self.assertFalse(failures[0].ok)
            self.assertIsNone(failures[0].brep)
            self._assert_center(partial_breps[0], (1, 1.5, 2))
            self._assert_center(partial_breps[1], (20.5, 1, 1.5))

    def _tessellate_moved_reference(self, tmpdir, extra_prims=""):
        """A box BrepArray referenced into /World/Moved (translated by 100 in x); output in a subdirectory."""
        sm.usd.export_brep(sm.create_box((0, 0, 0), 1, 1, 1), os.path.join(tmpdir, "box.usda"))
        scene = os.path.join(tmpdir, "scene.usda")
        with open(scene, "w", encoding="utf-8") as file:
            file.write('#usda 1.0\n(\n    defaultPrim = "World"\n)\n\ndef Xform "World"\n{\n'
                       '    def Xform "Moved" (\n        prepend references = @./box.usda@</World>\n    )\n    {\n'
                       '        double3 xformOp:translate = (100, 0, 0)\n'
                       '        uniform token[] xformOpOrder = ["xformOp:translate"]\n    }\n'
                       + extra_prims + '}\n')
        output = os.path.join(tmpdir, "out", "scene_mesh.usda")
        os.makedirs(os.path.dirname(output))
        results = sm.usd.tessellate_file(scene, output)
        with open(output, encoding="utf-8") as file:
            return results, output, file.read()

    def test_tessellate_file_keeps_relative_references_resolvable(self):
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            _, output, usda = self._tessellate_moved_reference(tmpdir)
            self.assertIn("@../box.usda@", usda)
            self.assertEqual(len(sm.usd.import_breps(output, heal=False)), 1)
            # Rewriting the paths must not make USD look for them from the input directory.
            completed = subprocess.run(
                [sys.executable, "-c", self._TESSELLATE_FILE_SCRIPT, os.path.join(tmpdir, "scene.usda"), output],
                capture_output=True, text=True, timeout=60)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertNotIn("Could not open asset", completed.stderr)

    def test_tessellate_file_places_mesh_at_its_brep(self):
        # The mesh points are Brep-local, so the mesh carries the Brep's world transform.
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            results, _, usda = self._tessellate_moved_reference(tmpdir)
            self.assertTrue(results[0].ok)
            mesh = usda[usda.index('def Mesh "tess_0_0"'):]
            self.assertRegex(mesh[:mesh.index("\n    }")], r"matrix4d xformOp:transform = .*\(100, 0, 0, 1\)")

    def test_tessellate_file_rejects_singular_mesh_parent(self):
        # A default prim scaled to zero in z has no inverse, so no mesh transform can place a Brep.
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            sm.usd.export_brep(sm.create_box((0, 0, 0), 1, 1, 1), os.path.join(tmpdir, "box.usda"))
            scene = os.path.join(tmpdir, "scene.usda")
            with open(scene, "w", encoding="utf-8") as file:
                file.write('#usda 1.0\n(\n    defaultPrim = "World"\n)\n\ndef Xform "World"\n{\n'
                           '    double3 xformOp:scale = (1, 1, 0)\n'
                           '    uniform token[] xformOpOrder = ["xformOp:scale"]\n'
                           '    def Xform "Part" (\n        prepend references = @./box.usda@</World>\n    )\n    {\n    }\n}\n')
            output = os.path.join(tmpdir, "out.usda")
            with self.assertRaises(RuntimeError) as raised:
                sm.usd.tessellate_file(scene, output)
            self.assertFalse(os.path.exists(output))
            # The message names the failed Brep and why, not just the status.
            self.assertIn("/World/Part/Brep0[0]", str(raised.exception))
            self.assertIn("singular transform", str(raised.exception))

    def test_tessellate_file_keeps_inherited_visibility_and_purpose(self):
        # The mesh is not under the Brep's ancestors, so it carries what it inherited from them.
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            sm.usd.export_brep(sm.create_box((0, 0, 0), 1, 1, 1), os.path.join(tmpdir, "box.usda"))
            scene = os.path.join(tmpdir, "scene.usda")
            with open(scene, "w", encoding="utf-8") as file:
                file.write('#usda 1.0\n(\n    defaultPrim = "World"\n)\n\ndef Xform "World"\n{\n'
                           '    def Xform "Hidden"\n    {\n'
                           '        token visibility = "invisible"\n        uniform token purpose = "guide"\n'
                           '        def Xform "Part" (\n            prepend references = @./box.usda@</World>\n'
                           '        )\n        {\n        }\n    }\n}\n')
            output = os.path.join(tmpdir, "out.usda")
            self.assertTrue(sm.usd.tessellate_file(scene, output)[0].ok)
            with open(output, encoding="utf-8") as file:
                usda = file.read()
            mesh = usda[usda.index('def Mesh "tess_0_0"'):]
            mesh = mesh[:mesh.index("\n    }")]
            self.assertIn('token visibility = "invisible"', mesh)
            self.assertIn('uniform token purpose = "guide"', mesh)

    def test_tessellate_file_does_not_overwrite_existing_prims(self):
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            results, _, usda = self._tessellate_moved_reference(tmpdir, '    def Xform "tess_0_0"\n    {\n    }\n')
            self.assertTrue(results[0].mesh_path.endswith("/tess_0_0_1"))
            self.assertIn('def Xform "tess_0_0"', usda)

    @staticmethod
    def _mesh_material(usda, mesh_name):
        mesh = usda[usda.index('def Mesh "%s"' % mesh_name):]
        match = re.search(r"rel material:binding = <([^>]*)>", mesh[:mesh.index("\n    }")])
        return match.group(1) if match else None

    def test_tessellate_file_binds_each_brep_material(self):
        # Each Brep of the BrepArray has its own "brep" GeomSubset material.
        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            output = os.path.join(tmpdir, "out.usda")
            results = sm.usd.tessellate_file(
                os.path.join(repo, "TestFiles", "brep_validator", "CubeBrepArray.usda"), output)
            self.assertEqual([r.ok for r in results], [True, True])
            with open(output, encoding="utf-8") as file:
                usda = file.read()
            self.assertEqual(self._mesh_material(usda, "tess_0_0"), "/World/Looks/ABS_Hard_Leather_Brown")
            self.assertEqual(self._mesh_material(usda, "tess_0_1"), "/World/Looks/ABS_Hard_Leather_Deep_Green")

    def test_tessellate_file_binds_the_brep_array_material(self):
        # The material is bound to the whole BrepArray; the mesh is not under it, so it is bound directly.
        material = ('    def "Looks"\n    {\n        def Material "Red"\n        {\n        }\n    }\n'
                    '    def Xform "Part" (\n        prepend references = @./box.usda@</World>\n    )\n    {\n'
                    '        over "Brep0" (\n            prepend apiSchemas = ["MaterialBindingAPI"]\n        )\n'
                    '        {\n            rel material:binding = </World/Looks/Red>\n        }\n    }\n')
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            results, _, usda = self._tessellate_moved_reference(tmpdir, material)
            self.assertEqual([r.ok for r in results], [True, True])
            self.assertEqual(self._mesh_material(usda, "tess_0_0"), None)  # /World/Moved has no material
            self.assertEqual(self._mesh_material(usda, "tess_1_0"), "/World/Looks/Red")

    def test_tessellate_file_writes_a_mesh_per_brep(self):
        box = sm.create_box((0, 0, 0), 2, 2, 2)
        sphere = sm.create_sphere((10, 0, 0), 3)
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            source = os.path.join(tmpdir, "breps.usda")
            output = os.path.join(tmpdir, "meshes.usda")
            sm.usd.export_breps([box, sphere], source)
            imported = sm.usd.import_breps(source, heal=False)

            results = sm.usd.tessellate_file(source, output)
            self.assertEqual([r.packed_index for r in results], [0, 1])
            for result, brep in zip(results, imported):
                self.assertTrue(result.ok, result.message)
                self.assertEqual(result.failed_face_count, 0)
                self.assertTrue(result.mesh_path.endswith("/tess_0_%d" % result.packed_index))
                expected = len(sm.tessellate(brep).to_mesh_arrays()["points"])
                self.assertEqual(result.point_count, expected)
            with open(output, encoding="utf-8") as file:
                usda = file.read()
            self.assertIn('def Mesh "tess_0_0"', usda)
            self.assertIn('def Mesh "tess_0_1"', usda)
            self.assertIn("def BrepArray", usda)  # original content kept

            finer = sm.usd.tessellate_file(source, output, chord_height_tolerance=0.01)
            self.assertGreater(finer[1].point_count, results[1].point_count)

    def test_tessellate_file_reports_failed_and_partial_breps(self):
        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            output = os.path.join(tmpdir, "out.usda")
            # The second Brep's face arrays are the wrong size: it fails, the first still meshes.
            results = sm.usd.tessellate_file(
                os.path.join(repo, "TestFiles", "brep_validator", "Test_BA_120_Wrong_face_array_size.usda"), output)
            self.assertEqual([r.ok for r in results], [True, False])
            self.assertEqual(results[1].packed_index, 1)
            self.assertIsNone(results[1].mesh_path)
            self.assertTrue(results[1].message)
            # Two faces fail to tessellate; the partial mesh is kept.
            (partial,) = sm.usd.tessellate_file(
                os.path.join(repo, "TestFiles", "usd_TestFiles", "RevitWallHealerCrash.usda"), output)
            self.assertTrue(partial.ok)
            self.assertEqual(partial.failed_face_count, 2)
            self.assertIsNotNone(partial.mesh_path)

    def test_tessellate_file_raises_without_brep_arrays(self):
        with tempfile.TemporaryDirectory(prefix="smpy_tessellate_file_") as tmpdir:
            with self.assertRaises(RuntimeError):
                sm.usd.tessellate_file(os.path.join(tmpdir, "missing.usda"), os.path.join(tmpdir, "out.usda"))
            meshes = os.path.join(tmpdir, "meshes.usda")
            sm.usd.export_mesh(sm.tessellate(sm.create_box((0, 0, 0), 1, 1, 1)), meshes)
            with self.assertRaises(RuntimeError) as ctx:
                sm.usd.tessellate_file(meshes, os.path.join(tmpdir, "out.usda"))
            self.assertIn("SM_ERR_INVALID_INPUT", str(ctx.exception))

    def test_import_rejects_missing_required_geometry_apis_without_crash(self):
        sphere = sm.create_sphere((0, 0, 0), 5)
        box = sm.create_box((20, 0, 0), 2, 4, 6)
        with tempfile.TemporaryDirectory(prefix="smpy_missing_brep_api_") as tmpdir:
            source_path = os.path.join(tmpdir, "sphere.usda")
            sm.usd.export_brep(sphere, source_path, export_uv_curves=False)
            sm.usd.append_breps([box], source_path, export_uv_curves=False)

            with open(source_path, encoding="utf-8") as file:
                source_usda = file.read()

            _, applied_schemas = self._brep_api_schemas(source_usda)
            required_schemas = {schema for schema in applied_schemas if schema.startswith("Brep")}
            self.assertIn("BrepSurfaceSphereAPI", required_schemas)
            self.assertNotIn("BrepCurveUvNurbAPI", required_schemas)
            self.assertGreater(len(required_schemas), 1)
            self.assertEqual(len(sm.usd.import_breps(source_path, heal=False)), 2)

            cases = {
                "one_required_api_omitted": {"BrepSurfaceSphereAPI"},
                "all_required_apis_omitted": set(required_schemas),
            }
            for name, omitted_schemas in cases.items():
                with self.subTest(case=name):
                    malformed_path = os.path.join(tmpdir, "%s.usda" % name)
                    malformed_usda = self._omit_brep_api_schemas(source_usda, omitted_schemas)
                    with open(malformed_path, "w", encoding="utf-8", newline="") as file:
                        file.write(malformed_usda)

                    self._assert_import_breps_rejected_without_crash(malformed_path)

                    report = sm.usd.import_breps(malformed_path, heal=False, allow_partial=True)
                    self.assertFalse(report.complete)
                    self.assertEqual(len(report.breps), 1)
                    self.assertEqual(len(report.items), 2)
                    self.assertEqual(len(report.failures), 1)

                    prim_failure, imported_box = report.items
                    self.assertEqual(prim_failure.prim_path, "/World/Brep0")
                    self.assertIsNone(prim_failure.packed_index)
                    self.assertIsNone(prim_failure.brep)
                    self.assertFalse(prim_failure.ok)
                    self.assertRegex(prim_failure.status_name, r"^SM_ERR")
                    self.assertTrue(prim_failure.message)
                    self.assertEqual(imported_box.prim_path, "/World/Brep1")
                    self.assertEqual(imported_box.packed_index, 0)
                    self.assertTrue(imported_box.ok)
                    self.assertEqual(imported_box.status_name, "SM_SUCCESS")
                    self.assertIs(imported_box.brep, report.breps[0])
                    self._assert_center(report.breps[0], (21, 2, 3))

    def test_copy_and_export_preserve_nurbs_representation(self):
        factories = [
            ("box", lambda: sm.create_box((0, 0, 0), 4, 5, 6)),
            ("cylinder", lambda: sm.create_cylinder((0, 0, 0), 3, 8)),
            ("cone", lambda: sm.create_cone((0, 0, 0), 3, 1, 8)),
            ("sphere", lambda: sm.create_sphere((0, 0, 0), 3)),
            ("torus", lambda: sm.create_torus((0, 0, 0), 5, 1)),
        ]
        for shape, factory in factories:
            brep = factory()
            sm.turn_to_nurbs(brep)
            domains = [face.uv_domain() for face in brep.faces()]
            sources = [brep, brep.copy(), sm.brep_copy(brep), copy.copy(brep), copy.deepcopy(brep)]
            with tempfile.TemporaryDirectory(prefix="smpy_nurbs_copy_") as tmpdir:
                for index, candidate in enumerate(sources):
                    with self.subTest(shape=shape, copy=index):
                        self.assertEqual([face.uv_domain() for face in candidate.faces()], domains)
                        path = os.path.join(tmpdir, "%d.usda" % index)
                        sm.usd.export_brep(candidate, path)
                        with open(path, encoding="utf-8") as file:
                            text = file.read()
                        types = re.findall(r'face:surfaceType = \[([^\]]+)\]', text)
                        self.assertEqual(len(types), 1)
                        self.assertEqual(re.findall(r'"([^"]+)"', types[0]),
                                         ["BrepSurfaceNurbAPI"] * candidate.face_count())
                path = os.path.join(tmpdir, "batch.usda")
                sm.usd.export_breps(sources[:2], path)
                sm.usd.append_breps(sources[2:], path)
                with open(path, encoding="utf-8") as file:
                    text = file.read()
                types = re.findall(r'face:surfaceType = \[([^\]]+)\]', text)
                self.assertEqual(len(types), 2)
                self.assertEqual([token for group in types for token in re.findall(r'"([^"]+)"', group)],
                                 ["BrepSurfaceNurbAPI"] * (len(sources) * brep.face_count()))
                self.assertEqual([face.uv_domain() for face in brep.faces()], domains)

    def test_export_rejects_null_breps(self):
        with tempfile.TemporaryDirectory(prefix="smpy_null_export_") as tmpdir:
            path = os.path.join(tmpdir, "breps.usda")
            brep = sm.create_box((0, 0, 0), 1, 2, 3)
            with self.assertRaises(RuntimeError):
                sm.usd.export_brep(None, path)
            with self.assertRaises(RuntimeError):
                sm.usd.export_breps([brep, None], path)
            sm.usd.export_brep(brep, path)
            with self.assertRaises(RuntimeError):
                sm.usd.append_breps([brep, None], path)

    def test_export_preserves_scaled_unbounded_plane_range(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "plane.usda")
            sm.usd.export_brep(sm.create_box((0, 0, 0), 10, 10, 10), path)
            with open(path) as file:
                text = self._inject_unbounded_first_face_range(file.read())
            with open(path, "w") as file:
                file.write(text)
            brep = sm.usd.import_breps(path, heal=True)[0]
            sm.scale(brep, (2, 2, 2))
            sm.usd.export_brep(brep, path, shrink_face_domains=False)
            ranges = self._face_range_faces(self._face_range_attrs(path))
            self.assertEqual(ranges[0], [-1234567, -1234567, 1234567, 1234567])
            imported = sm.usd.import_breps(path, heal=True)[0]
            sm.usd.export_brep(imported, path, shrink_face_domains=False)
            self.assertEqual(self._face_range_faces(self._face_range_attrs(path))[0], ranges[0])

    def test_export_bounds_unbounded_face_ranges_by_default(self):
        cases = [
            ("plane", sm.create_box((0, 0, 0), 10, 10, 10)),
            ("cylinder", sm.create_cylinder((0, 0, 0), 5, 10)),
        ]
        for name, source_brep in cases:
            with self.subTest(surface=name):
                with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
                    source_path = f.name
                with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
                    unbounded_path = f.name
                with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
                    verbatim_path = f.name
                with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
                    default_path = f.name
                with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
                    verbatim_after_path = f.name
                try:
                    sm.usd.export_brep(
                        source_brep,
                        source_path,
                        export_uv_curves=False,
                        shrink_face_domains=False,
                    )

                    with open(source_path) as fh:
                        source_usda = fh.read()
                    with open(unbounded_path, "w") as fh:
                        fh.write(self._inject_unbounded_first_face_range(source_usda))

                    brep = sm.usd.import_breps(unbounded_path)[0]

                    sm.usd.export_brep(
                        brep,
                        verbatim_path,
                        export_uv_curves=False,
                        shrink_face_domains=False,
                    )
                    verbatim = self._face_range_attrs(verbatim_path)

                    sm.usd.export_brep(brep, default_path, export_uv_curves=False)
                    bounded = self._face_range_attrs(default_path)
                    self.assertNotIn("1234567", bounded)

                    sm.usd.export_brep(
                        brep,
                        verbatim_after_path,
                        export_uv_curves=False,
                        shrink_face_domains=False,
                    )
                    self.assertEqual(verbatim, self._face_range_attrs(verbatim_after_path))

                    source_faces = self._face_range_faces(self._face_range_attrs(source_path))
                    verbatim_faces = self._face_range_faces(verbatim)
                    bounded_faces = self._face_range_faces(bounded)
                    self.assertEqual(len(verbatim_faces), len(bounded_faces))

                    unbounded_face_indices = [
                        index for index, face_range in enumerate(verbatim_faces)
                        if self._has_infinite_face_bound(face_range)
                    ]
                    self.assertEqual([0], unbounded_face_indices)

                    for index, verbatim_face in enumerate(verbatim_faces):
                        bounded_face = bounded_faces[index]
                        if index in unbounded_face_indices:
                            self.assertFalse(self._has_infinite_face_bound(bounded_face))
                            self.assertNotEqual(verbatim_face, bounded_face)
                        else:
                            self.assertEqual(verbatim_face, bounded_face)

                    self._assert_face_ranges_close([source_faces[0]], [bounded_faces[0]])
                finally:
                    os.unlink(source_path)
                    os.unlink(unbounded_path)
                    os.unlink(verbatim_path)
                    os.unlink(default_path)
                    os.unlink(verbatim_after_path)

    def test_append_breps(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(box, path)
            sph = sm.create_sphere((20, 0, 0), 5)
            sm.usd.append_breps([sph], path)
            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps), 2)
        finally:
            os.unlink(path)

    def test_import_single_prim(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(box, path)
            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps), 1)
        finally:
            os.unlink(path)

    def test_cylinder_roundtrip_preserves_volume_and_tessellation(self):
        cylinder = sm.create_cylinder((0, 0, 0), 5, 10)
        expected_volume = cylinder.volume()
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "cylinder.usda")
            sm.usd.export_brep(cylinder, path)
            for heal in (False, True):
                with self.subTest(heal=heal):
                    imported = sm.usd.import_breps(path, heal=heal)[0]
                    self.assertEqual(imported.face_count(), 3)
                    self.assertTrue(imported.is_manifold_solid())
                    self.assertAlmostEqual(imported.volume(), expected_volume, delta=0.01)
                    self.assertGreater(len(sm.tessellate(imported).get_faces()), 0)

    def test_roundtrip_preserves_topology(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        sph = sm.create_sphere((5, 5, 5), 3)
        result = sm.boolean_difference(box, sph)
        orig_faces = len(result.faces())
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(result, path)
            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps[0].faces()), orig_faces)
        finally:
            os.unlink(path)

    def _assert_center(self, brep, expected, places=4):
        c = brep.center()
        for got, want in zip(c, expected):
            self.assertAlmostEqual(got, want, places=places)

    def test_import_applies_prim_transform(self):
        # A translate authored directly on the BrepArray prim must be baked
        # into the imported geometry (placement was previously dropped).
        box = sm.create_box((0, 0, 0), 10, 10, 10)  # center (5, 5, 5)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(box, path)
            with open(path) as fh:
                txt = fh.read()
            txt = _inject_xform_ops(txt, "Brep0", _TRANSLATE_OPS(100, 0, 0))
            with open(path, "w") as fh:
                fh.write(txt)

            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps), 1)
            self._assert_center(breps[0], (105, 5, 5))
        finally:
            os.unlink(path)

    def test_import_applies_ancestor_transform(self):
        # World placement must accumulate the ancestor Xform (/World) with a
        # transform on the BrepArray prim itself.
        box = sm.create_box((0, 0, 0), 10, 10, 10)  # center (5, 5, 5)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(box, path)
            with open(path) as fh:
                txt = fh.read()
            txt = _inject_xform_ops(txt, "World", _TRANSLATE_OPS(100, 0, 0))
            txt = _inject_xform_ops(txt, "Brep0", _TRANSLATE_OPS(0, 50, 0))
            with open(path, "w") as fh:
                fh.write(txt)

            breps = sm.usd.import_breps(path)
            self.assertEqual(len(breps), 1)
            self._assert_center(breps[0], (105, 55, 5))
        finally:
            os.unlink(path)

    def test_import_single_prim_applies_transform(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)  # center (5, 5, 5)
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            sm.usd.export_brep(box, path)
            with open(path) as fh:
                txt = fh.read()
            txt = _inject_xform_ops(txt, "Brep0", _TRANSLATE_OPS(0, 0, 30))
            with open(path, "w") as fh:
                fh.write(txt)

            brep = sm.usd.import_brep(path, "/World/Brep0")
            self._assert_center(brep, (5, 5, 35))
        finally:
            os.unlink(path)


# -----------------------------------------------------------------------
#  Error handling
# -----------------------------------------------------------------------
class TestErrors(unittest.TestCase):
    def test_api_invalid_input_status_codes(self):
        cases = (
            (sm.stitch_brep, (None,)),
            (sm.remove_fillet, (None,)),
            (sm.remove_curve_knots, (None,)),
            (sm.make_curves_compatible, ([],)),
            (sm.create_skin_surface, ([],)),
            (sm.create_pyramid, ((0, 0, 0), 0.0, 1.0)),
            (sm.curve_closest_point, (None, (0, 0, 0))),
            (sm.curve_closest_point_all, (None, (0, 0, 0))),
            (sm.surface_closest_point, (None, (0, 0, 0))),
            (sm.brep_closest_point, (None, (0, 0, 0))),
            (sm.intersect_surfaces, (None, None)),
            (sm.intersect_breps, (None, None)),
        )
        for operation, args in cases:
            with self.subTest(operation=operation.__name__):
                with self.assertRaisesRegex(RuntimeError, r"failed: SM_ERR_INVALID_INPUT \(1007\)"):
                    operation(*args)

    def test_zero_rotation_axis_reports_invalid_input(self):
        box = sm.create_box((0, 0, 0), 1, 1, 1)
        before = box.bounding_box()
        with self.assertRaisesRegex(RuntimeError, r"failed: SM_ERR_INVALID_INPUT \(1007\)"):
            sm.rotate(box, (0, 0, 0), (0, 0, 0), 30.0)
        self.assertEqual(box.bounding_box(), before)

    def test_bad_tuple_size(self):
        with self.assertRaises(Exception):
            sm.create_box((0, 0), 10, 10, 10)

    def test_usd_import_nonexistent(self):
        with self.assertRaises(RuntimeError):
            sm.usd.import_breps("/nonexistent/file.usda")

    def test_usd_partial_import_nonexistent(self):
        with self.assertRaises(RuntimeError) as raised:
            sm.usd.import_breps("/nonexistent/file.usda", allow_partial=True)
        self.assertIn("SmApiUsdImportBreps", str(raised.exception))

    def test_runtime_error_includes_c_entry_point(self):
        """A failing kernel call should name the SmApi* function in the
        message so the caller can find the wrapped entry point in the
        source tree without inspecting C++ traces."""
        try:
            sm.usd.import_breps("/nonexistent/file.usda")
        except RuntimeError as exc:
            self.assertIn("SmApiUsdImportBreps", str(exc))
        else:
            self.fail("expected RuntimeError")

    def test_runtime_error_includes_symbolic_status_name(self):
        """The exception message should carry the SM_* identifier (e.g.
        ``SM_ERR``, ``SM_ERR_INVALID_INPUT``) -- not just the integer code,
        which forces grep into source/SMLib/inc/SmMessages.h."""
        try:
            sm.usd.import_breps("/nonexistent/file.usda")
        except RuntimeError as exc:
            msg = str(exc)
            self.assertRegex(msg, r"\bSM_(SUCCESS|ERR\w*)\b",
                             "expected an SM_* identifier in: " + msg)
            self.assertNotRegex(msg, r"^SMLib error code: \d+$",
                                "old opaque format leaked through")
        else:
            self.fail("expected RuntimeError")

    def test_runtime_error_does_not_accumulate_between_calls(self):
        """Each CHECK_STATUS clears the per-thread kernel trail before the
        wrapped call.  Two consecutive failures must produce stand-alone
        messages, not a cumulative one that grows with every attempt."""
        msgs = []
        for _ in range(3):
            try:
                sm.usd.import_breps("/nonexistent/file.usda")
            except RuntimeError as exc:
                msgs.append(str(exc))
        self.assertEqual(len(msgs), 3)
        # All three messages should be the same length to within a small
        # margin -- if the trail accumulated, later messages would grow
        # unbounded.
        lengths = [len(m) for m in msgs]
        spread = max(lengths) - min(lengths)
        self.assertLess(spread, 64,
                        "trail appears to accumulate across calls: " + repr(msgs))


# -----------------------------------------------------------------------
#  Discoverability: every function-only binder is reachable both flat
#  (sm.<name>) and via its category submodule (sm.<cat>.<name>).
# -----------------------------------------------------------------------
class TestDiscoverability(unittest.TestCase):
    EXPECTED_SUBMODULES = (
        "primitives", "curves", "surfaces", "booleans", "sweeps",
        "fillets", "offset", "io", "brep_ops", "stitching", "intersect",
        "transforms", "queries", "cutting", "heal", "poly", "usd",
    )

    # Pick one representative function per category that we expect to be
    # present in both the flat namespace and the submodule.
    SAMPLE_BINDINGS = (
        ("primitives", "create_box"),
        ("curves", "create_line_segment"),
        ("surfaces", "create_ruled_surface"),
        ("booleans", "boolean_union"),
        ("sweeps", "linear_sweep"),
        ("fillets", "circular_fillet"),
        ("offset", "shell_brep"),
        ("io", "read_brep_from_file"),
        ("brep_ops", "turn_to_nurbs"),
        ("stitching", "stitch_brep"),
        ("intersect", "intersect_breps"),
        ("transforms", "translate"),
        ("queries", "compute_volume"),
        ("cutting", "cut"),
        ("heal", "heal_brep"),
        ("poly", "poly_boolean_union"),
    )

    def test_submodules_exist(self):
        for name in self.EXPECTED_SUBMODULES:
            self.assertTrue(hasattr(sm, name), f"sm.{name} missing")

    def test_flat_and_submodule_alias_same_callable(self):
        for category, fname in self.SAMPLE_BINDINGS:
            if fname is None:
                continue
            sub = getattr(sm, category)
            self.assertTrue(hasattr(sm, fname),
                            f"flat sm.{fname} missing")
            self.assertTrue(hasattr(sub, fname),
                            f"sm.{category}.{fname} missing")
            # Both spellings must be callable; they wrap the same C++
            # lambda, but pybind11 makes them distinct PyCFunction objects,
            # so identity is not required -- just both being callable.
            self.assertTrue(callable(getattr(sm, fname)))
            self.assertTrue(callable(getattr(sub, fname)))

    def test_submodule_call_works(self):
        # End-to-end: a real call via the submodule produces a Brep.
        b = sm.primitives.create_box((0, 0, 0), 1, 1, 1)
        self.assertIsNotNone(b)

    def test_module_docstring_lists_categories(self):
        doc = sm.__doc__ or ""
        self.assertIn("API by category", doc)
        for name in ("primitives", "curves", "booleans", "sweeps", "fillets"):
            self.assertIn(name, doc)


# -----------------------------------------------------------------------
#  Curve methods (parameter_range, evaluate, length, is_closed, ...)
# -----------------------------------------------------------------------
class TestCurveMethods(unittest.TestCase):
    def test_line_parameter_range(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        t_min, t_max = line.parameter_range()
        self.assertLess(t_min, t_max)

    def test_line_endpoints_match_constructor(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        sx, sy, sz = line.start_point()
        ex, ey, ez = line.end_point()
        self.assertAlmostEqual(sx, 0.0, places=6)
        self.assertAlmostEqual(sy, 0.0, places=6)
        self.assertAlmostEqual(sz, 0.0, places=6)
        self.assertAlmostEqual(ex, 10.0, places=6)
        self.assertAlmostEqual(ey, 0.0, places=6)
        self.assertAlmostEqual(ez, 0.0, places=6)

    def test_line_evaluate_midpoint(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        t_min, t_max = line.parameter_range()
        mid = line.evaluate(0.5 * (t_min + t_max))
        self.assertAlmostEqual(mid[0], 5.0, places=6)
        self.assertAlmostEqual(mid[1], 0.0, places=6)
        self.assertAlmostEqual(mid[2], 0.0, places=6)

    def test_line_length(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.assertAlmostEqual(line.length(), 10.0, places=4)

    def test_circle_length(self):
        circ = sm.create_circle((0, 0, 0), 5)
        self.assertAlmostEqual(circ.length(), 2.0 * math.pi * 5, delta=1e-3)

    def test_circle_is_closed(self):
        circ = sm.create_circle((0, 0, 0), 5)
        self.assertTrue(circ.is_closed())

    def test_line_is_not_closed(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.assertFalse(line.is_closed())

    def test_line_is_planar(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.assertTrue(line.is_planar())

    def test_circle_is_planar(self):
        circ = sm.create_circle((0, 0, 0), 5)
        self.assertTrue(circ.is_planar())

    def test_curve_repr(self):
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.assertIn("Curve", repr(line))

    def test_evaluate_curve_returns_populated_dict(self):
        # Regression: prior to the SmApiEvaluateCurve binding fix, this
        # call returned an empty dict because the binding passed nullptr
        # for every output slot.
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        result = sm.evaluate_curve(line, 0.5)
        self.assertIn("point", result)
        self.assertIn("tangent", result)
        self.assertIn("curvature", result)
        self.assertEqual(len(result["point"]), 3)


# -----------------------------------------------------------------------
#  Surface methods (uv_domain, evaluate, normal, derivatives, ...)
# -----------------------------------------------------------------------
class TestSurfaceMethods(unittest.TestCase):
    def _flat_surface(self):
        # 4x4 grid of points all at z=0 -> a planar surface.
        pts = [
            (0, 0, 0),  (5, 0, 0),  (10, 0, 0),  (15, 0, 0),
            (0, 5, 0),  (5, 5, 0),  (10, 5, 0),  (15, 5, 0),
            (0, 10, 0), (5, 10, 0), (10, 10, 0), (15, 10, 0),
            (0, 15, 0), (5, 15, 0), (10, 15, 0), (15, 15, 0),
        ]
        return sm.create_surface_from_points(pts, 4, 4)

    def test_u_v_domain(self):
        srf = self._flat_surface()
        u_min, u_max = srf.u_domain()
        v_min, v_max = srf.v_domain()
        self.assertLess(u_min, u_max)
        self.assertLess(v_min, v_max)

    def test_uv_domain_consistent(self):
        srf = self._flat_surface()
        (umin, vmin), (umax, vmax) = srf.uv_domain()
        u_min, u_max = srf.u_domain()
        v_min, v_max = srf.v_domain()
        self.assertAlmostEqual(umin, u_min)
        self.assertAlmostEqual(umax, u_max)
        self.assertAlmostEqual(vmin, v_min)
        self.assertAlmostEqual(vmax, v_max)

    def test_evaluate_returns_3tuple(self):
        srf = self._flat_surface()
        u_min, u_max = srf.u_domain()
        v_min, v_max = srf.v_domain()
        u = 0.5 * (u_min + u_max)
        v = 0.5 * (v_min + v_max)
        pt = srf.evaluate(u, v)
        self.assertEqual(len(pt), 3)
        self.assertAlmostEqual(pt[2], 0.0, places=4)

    def test_normal_is_unit(self):
        srf = self._flat_surface()
        u_min, u_max = srf.u_domain()
        v_min, v_max = srf.v_domain()
        nx, ny, nz = srf.normal(0.5 * (u_min + u_max), 0.5 * (v_min + v_max))
        mag = math.sqrt(nx * nx + ny * ny + nz * nz)
        self.assertAlmostEqual(mag, 1.0, places=4)

    def test_derivatives_shape(self):
        srf = self._flat_surface()
        u_min, u_max = srf.u_domain()
        v_min, v_max = srf.v_domain()
        pt, du, dv = srf.derivatives(
            0.5 * (u_min + u_max), 0.5 * (v_min + v_max))
        self.assertEqual(len(pt), 3)
        self.assertEqual(len(du), 3)
        self.assertEqual(len(dv), 3)

    def test_planar_surface_is_planar(self):
        srf = self._flat_surface()
        self.assertTrue(srf.is_planar(1e-3))

    def test_is_periodic_u_v_returns_bool(self):
        srf = self._flat_surface()
        # Flat NURBS patch is not periodic in either direction, but the
        # only contract here is that the method returns a Python bool.
        self.assertIsInstance(srf.is_periodic_u(), bool)
        self.assertIsInstance(srf.is_periodic_v(), bool)

    def test_surface_repr(self):
        srf = self._flat_surface()
        self.assertIn("Surface", repr(srf))


class TestCapEndsAndOffsetType(unittest.TestCase):
    """Binding-only enums that replaced magic ints in sweeps and offsets."""

    # ---- value mapping: must match the kernel's documented contract ----

    def test_cap_ends_values(self):
        self.assertEqual(int(sm.CapEnds.NONE),  0)
        self.assertEqual(int(sm.CapEnds.START), 1)
        self.assertEqual(int(sm.CapEnds.END),   2)
        self.assertEqual(int(sm.CapEnds.BOTH),  3)

    def test_offset_type_values(self):
        self.assertEqual(int(sm.OffsetType.LEFT),  1)
        self.assertEqual(int(sm.OffsetType.RIGHT), 2)
        self.assertEqual(int(sm.OffsetType.BOTH),  3)

    # ---- not exported into module namespace (qualified access only) ----

    def test_enum_members_not_module_level(self):
        for n in ("LEFT", "RIGHT", "START", "END"):
            self.assertFalse(hasattr(sm, n),
                             f"{n} should be qualified, not module-level")

    # ---- passing a plain int is rejected ----

    def test_draft_sweep_rejects_int_cap_ends(self):
        circ = sm.create_circle((0, 0, 0), 5)
        with self.assertRaises(TypeError):
            sm.draft_sweep([circ], 10, 5.0, cap_ends=3)

    def test_create_offset_profile_rejects_int_offset_type(self):
        sq = [
            sm.create_line_segment((0, 0, 0), (10, 0, 0)),
            sm.create_line_segment((10, 0, 0), (10, 10, 0)),
            sm.create_line_segment((10, 10, 0), (0, 10, 0)),
            sm.create_line_segment((0, 10, 0), (0, 0, 0)),
        ]
        with self.assertRaises(TypeError):
            sm.create_offset_profile(sq, 1.0, offset_type=3)

    # ---- defaults are the BOTH variant ----

    def test_draft_sweep_default_caps_both_ends(self):
        circ = sm.create_circle((0, 0, 0), 5)
        r = sm.draft_sweep([circ], 10, 5.0)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_taper_extrude_default_caps_both_ends(self):
        circ = sm.create_circle((0, 0, 0), 5)
        r = sm.taper_extrude([circ], 10, 80.0)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    def test_sweep_along_planar_path_default_caps_both_ends(self):
        profile = sm.create_circle((0, 0, 0), 2)
        path = sm.create_line_segment((0, 0, 0), (20, 0, 0))
        r = sm.sweep_along_planar_path([profile], [path])
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)

    # ---- create_offset_profile end-to-end with the enum ----

    def test_create_offset_profile_band_with_enum(self):
        sq = [
            sm.create_line_segment((0, 0, 0), (10, 0, 0)),
            sm.create_line_segment((10, 0, 0), (10, 10, 0)),
            sm.create_line_segment((10, 10, 0), (0, 10, 0)),
            sm.create_line_segment((0, 10, 0), (0, 0, 0)),
        ]
        r = sm.create_offset_profile(sq, 1.0, offset_type=sm.OffsetType.BOTH)
        self.assertIsInstance(r, sm.Brep)
        self.assertGreater(len(r.faces()), 0)


class TestDegenerateRadiiHardening(unittest.TestCase):
    """Primitive CreateCanonical must not crash on degenerate radii, but must still accept
    valid-but-unusual representations (inside-out sphere, spindle torus)."""

    def test_spindle_torus_minor_ge_major_is_valid(self):
        # minor radius > major radius is a valid self-intersecting ("spindle") torus for some trimmed
        # domains; CreateCanonical must not reject it. create_torus(origin, radius_major, radius_minor).
        tor = sm.create_torus((0, 0, 0), 3, 10)
        self.assertIsInstance(tor, sm.Brep)
        self.assertGreater(len(tor.faces()), 0)

    def test_inside_out_sphere_negative_radius_is_valid(self):
        # A negative radius is a valid "inside-out" sphere (reversed parameterization); it must not be
        # rejected up front.
        sph = sm.create_sphere((0, 0, 0), -5)
        self.assertIsInstance(sph, sm.Brep)
        self.assertGreater(len(sph.faces()), 0)

    def test_zero_radius_sphere_fails_gracefully(self):
        # A zero radius is genuinely degenerate: fail cleanly rather than crash (the original bug was a
        # NULL-NURB dereference in release builds).
        with self.assertRaises(RuntimeError):
            sm.create_sphere((0, 0, 0), 0.0)

    def test_zero_minor_radius_torus_fails_gracefully(self):
        with self.assertRaises(RuntimeError):
            sm.create_torus((0, 0, 0), 10, 0.0)


if __name__ == "__main__":
    unittest.main()
