# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Data-sanity coverage is not a certificate of kernel geometry validity."""

import math
import sys
from pathlib import Path

TEST_ROOT = Path(__file__).resolve().parent
if str(TEST_ROOT) not in sys.path:
    sys.path.insert(0, str(TEST_ROOT))
try:
    from .utils.base_test_case import BrepValidatorBaseTestCase
except ImportError:
    from utils.base_test_case import BrepValidatorBaseTestCase

from brep_validator import BrepValidator  # noqa: E402
from pxr import Sdf, Usd, Vt  # noqa: E402


class BrepValidatorBoundaryTestCase(BrepValidatorBaseTestCase):
    PERIODIC_AXES = (
        ("Cylinder", 0, "BA.562"),
        ("Cone", 0, "BA.563"),
        ("Sphere", 0, "BA.560"),
        ("Torus", 0, "BA.564"),
        ("Torus", 1, "BA.565"),
    )

    def _open_fixture(self, name):
        stage = Usd.Stage.Open(str(self.test_files_dir / name))
        self.assertIsNotNone(stage)
        prim = next(p for p in stage.TraverseAll() if p.GetTypeName() == "BrepArray")
        return stage, prim

    @staticmethod
    def _issues(prim):
        checker = BrepValidator()
        checker.CheckPrim(prim)
        return checker.GetIssues()

    @staticmethod
    def _codes(issues):
        return {issue.requirement.code for issue in issues if issue.requirement is not None}

    def _analytic_face(self, surface, axis=0, start=0.0, span=2.0 * math.pi):
        # These fixtures isolate authored data. Their simplified boundary is NOT
        # a valid geometric face; passing CheckPrim must not imply otherwise.
        stage, prim = self._open_fixture("coverage_limits/shifted_full_period_face.usda")
        api = f"BrepSurface{surface}API"
        prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit([
            "BrepPointAPI:vertexPoint", "BrepCurve3dLineAPI:edge3dLine", api,
        ]))
        prim.GetAttribute("face:surfaceType").Set(Vt.TokenArray([api]))
        for name in prim.GetPropertyNames():
            if name.startswith("brep:surface:cylinder:"):
                prim.RemoveProperty(name)
        prefix = f"brep:surface:{surface.lower()}:"
        prim.CreateAttribute(prefix + ("center" if surface == "Sphere" else "origin"),
                             Sdf.ValueTypeNames.Point3dArray).Set(Vt.Vec3dArray([(0, 0, 0)]))
        prim.CreateAttribute(prefix + "axis", Sdf.ValueTypeNames.Vector3dArray).Set(Vt.Vec3dArray([(0, 0, 1)]))
        prim.CreateAttribute(prefix + "refDirection", Sdf.ValueTypeNames.Vector3dArray).Set(
            Vt.Vec3dArray([(1, 0, 0)]))
        radii = {"majorRadius": 1.0, "minorRadius": 0.5} if surface == "Torus" else {"radius": 1.0}
        if surface == "Cone":
            radii["semiAngle"] = 0.25
        for name, value in radii.items():
            prim.CreateAttribute(prefix + name, Sdf.ValueTypeNames.DoubleArray).Set(Vt.DoubleArray([value]))
        lower, upper = [0.0, 0.0], [1.0, 1.0]
        lower[axis], upper[axis] = start, start + span
        prim.GetAttribute("face:range").Set(Vt.Vec2dArray([tuple(lower), tuple(upper)]))
        return stage, prim

    def test_nurbs_endpoint_agreement_is_left_to_kernel(self):
        stage, prim = self._open_fixture("Cube.usda")
        self.assertEqual(self._issues(prim), [])
        # Break endpoint-to-vertex agreement without breaking the packed data.
        attr = prim.GetAttribute("brep:edge3dNurb:curve3d:nurb:controlVertices")
        points = list(attr.Get())
        points[0] = (1, 0, 0.75)
        attr.Set(Vt.Vec3dArray(points))
        weights = prim.GetAttribute("brep:edge3dNurb:curve3d:nurb:weights")
        values = list(weights.Get())
        values[0] = 2.0
        weights.Set(Vt.DoubleArray(values))
        self.assertEqual(self._issues(prim), [])

    def test_nurbs_structural_failures_are_still_reported(self):
        cases = (
            ("order", lambda values: [3] + values[1:], "BA.340"),
            ("weights", lambda values: values[:-1], "BA.345"),
            ("weights", lambda values: [0.0] + values[1:], "BA.350"),
            ("knots", lambda values: values[:-1], "BA.355"),
            ("knots", lambda values: [0.5] + values[1:], "BA.360"),
        )
        for name, mutate, code in cases:
            with self.subTest(attribute=name, code=code):
                stage, prim = self._open_fixture("Cube.usda")
                attr = prim.GetAttribute("brep:edge3dNurb:curve3d:nurb:" + name)
                attr.Set(mutate(list(attr.Get())))
                self.assertIn(code, self._codes(self._issues(prim)))

    def test_seam_geometry_is_not_inferred_from_repeated_edge_indices(self):
        for name in ("cylinder_without_repeated_edge.usda", "cylinder_with_vertex_loop.usda"):
            with self.subTest(fixture=name):
                stage, prim = self._open_fixture("coverage_limits/" + name)
                self.assertEqual(self._issues(prim), [])

    def test_topology_index_errors_are_still_reported(self):
        stage, prim = self._open_fixture("Cube.usda")
        attr = prim.GetAttribute("edgeuse:edgeIndex")
        indices = list(attr.Get())
        indices[0] = 999
        attr.Set(Vt.UIntArray(indices))
        self.assertIn("BA.205", self._codes(self._issues(prim)))

    def test_malformed_edge_range_does_not_abort_diagnostics(self):
        cases = (
            (Sdf.ValueTypeNames.StringArray, Vt.StringArray(["0", "bad", "0", "7"]), True),
            (Sdf.ValueTypeNames.Double2Array, Vt.Vec2dArray([(0, 0)] * 4), False),
        )
        for first_curve, second_curve in (("Circle", "Ellipse"), ("Ellipse", "Circle")):
            for value_type, values, has_later_period_error in cases:
                with self.subTest(first_curve=first_curve, value_type=value_type):
                    # Deliberately incomplete data: exercise CheckPrim's diagnostics,
                    # not endpoint geometry evaluation (which requires vertex positions).
                    stage = Usd.Stage.CreateInMemory()
                    prim = stage.DefinePrim("/Brep", "BrepArray")
                    prim.CreateAttribute("edge:curveType", Sdf.ValueTypeNames.TokenArray).Set(Vt.TokenArray([
                        f"BrepCurve3d{first_curve}API", f"BrepCurve3d{second_curve}API",
                    ]))
                    prim.CreateAttribute("edge:vertexIndices", Sdf.ValueTypeNames.Int2Array).Set(
                        Vt.Vec2iArray([(0, 1), (1, 0)]))
                    # Raw authoring bypasses the registered schema's double[] type.
                    spec = Sdf.AttributeSpec(stage.GetRootLayer().GetPrimAtPath("/Brep"),
                                             "edge:range", value_type)
                    spec.default = values
                    prim.CreateAttribute("brep:intersectTol3d", Sdf.ValueTypeNames.DoubleArray).Set(
                        Vt.DoubleArray([math.nan]))

                    issues = self._issues(prim)
                    codes = self._codes(issues)
                    self.assertIn("BA.235", codes)  # Malformed range was reported.
                    self.assertIn("BA.237", codes)  # Authored type is not double[].
                    self.assertIn("BA.660", codes)  # Later nonfinite-data check still ran.
                    period_issues = [issue for issue in issues if issue.requirement.code == "BA.630"]
                    self.assertEqual(len(period_issues), int(has_later_period_error))
                    if has_later_period_error:
                        self.assertIn(f"{second_curve} edge #1", period_issues[0].message)

    def test_numeric_edge_primary_period_checks_are_unchanged(self):
        for curve in ("Circle", "Ellipse"):
            for param_max in (math.pi, 2 * math.pi, -1.0, 2 * math.pi + 1.0):
                with self.subTest(curve=curve, param_max=param_max):
                    stage = Usd.Stage.CreateInMemory()
                    prim = stage.DefinePrim("/Brep", "BrepArray")
                    prim.CreateAttribute("edge:curveType", Sdf.ValueTypeNames.TokenArray).Set(
                        Vt.TokenArray([f"BrepCurve3d{curve}API"]))
                    prim.CreateAttribute("edge:range", Sdf.ValueTypeNames.DoubleArray).Set(
                        Vt.DoubleArray([param_max - 1.0, param_max]))
                    checker = BrepValidator()
                    checker._validate_edge_angular_range_primary_period(prim)
                    codes = self._codes(checker.GetIssues())
                    self.assertEqual(codes, {"BA.630"} if param_max < 0 or param_max > 2 * math.pi else set())

    def test_periodic_face_domains_allow_shifted_full_and_partial_periods(self):
        # Proposal: a periodic range's LENGTH is <= period; no required origin.
        for surface, axis, _ in self.PERIODIC_AXES:
            for start in (-4 * math.pi, -math.pi, 0.0, math.pi, 4 * math.pi):
                for span in (math.pi, 2 * math.pi - 2e-6, 2 * math.pi):
                    with self.subTest(surface=surface, axis=axis, start=start, span=span):
                        stage, prim = self._analytic_face(surface, axis, start, span)
                        self.assertEqual(self._issues(prim), [])

    def test_existing_shifted_partial_period_fixture_is_not_rejected(self):
        stage, prim = self._open_fixture("coverage_limits/shifted_partial_period_face.usda")
        self.assertEqual(self._issues(prim), [])

    def test_shifted_domains_wider_than_one_period_are_rejected(self):
        for surface, axis, code in self.PERIODIC_AXES:
            for start in (-4 * math.pi, 0.0, 4 * math.pi):
                with self.subTest(surface=surface, axis=axis, start=start):
                    stage, prim = self._analytic_face(surface, axis, start, 2 * math.pi + 2e-6)
                    self.assertEqual(self._codes(self._issues(prim)), {code})

    def test_torus_allows_both_periods_shifted_independently(self):
        stage, prim = self._analytic_face("Torus")
        prim.GetAttribute("face:range").Set(Vt.Vec2dArray([
            (-math.pi, 3 * math.pi), (math.pi, 5 * math.pi),
        ]))
        self.assertEqual(self._issues(prim), [])

    def test_invalid_face_ranges_are_still_reported(self):
        for axis in (0, 1):
            for span in (0.0, -1.0):
                with self.subTest(axis=axis, span=span):
                    stage, prim = self._analytic_face("Torus", axis, 0.0, span)
                    self.assertIn("BA.155" if axis == 0 else "BA.160", self._codes(self._issues(prim)))
            for start in (math.nan, math.inf, -math.inf):
                with self.subTest(axis=axis, start=start):
                    stage, prim = self._analytic_face("Torus", axis, start, 1.0)
                    self.assertIn("BA.660", self._codes(self._issues(prim)))

    def test_sphere_latitude_bounds_are_not_periodic(self):
        for lower, upper in ((-math.pi, 0.0), (0.0, math.pi)):
            with self.subTest(lower=lower, upper=upper):
                stage, prim = self._analytic_face("Sphere")
                prim.GetAttribute("face:range").Set(Vt.Vec2dArray([
                    (-math.pi, lower), (math.pi, upper),
                ]))
                self.assertIn("BA.561", self._codes(self._issues(prim)))
