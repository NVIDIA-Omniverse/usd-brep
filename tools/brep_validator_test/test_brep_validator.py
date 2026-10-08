# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import math
import re
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
from pxr import Gf, Sdf, Usd, Vt  # noqa: E402


_UNAUTHORED = object()


def _expected_requirement(test_file: Path) -> str | None:
    match = re.search(r"Test_(BA_\d{3})_", test_file.name)
    return match.group(1).replace("_", ".") if match else None


class BrepValidatorFixtureTestCase(BrepValidatorBaseTestCase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        print("Running BREP validator tests", flush=True)

    def _open_cube_prim(self):
        stage = Usd.Stage.Open(str(self.test_files_dir / "Cube.usda"))
        self.assertIsNotNone(stage)
        return stage, stage.GetPrimAtPath("/World/Cube")

    def _check_prim(self, prim):
        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        checker.CheckPrim(prim)
        return checker.GetIssues()

    def _check_required_geometry_apis(self, prim):
        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        checker._validate_required_geometry_apis(prim)
        return checker.GetIssues()

    def _check_curve_uv_data(self, prim):
        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        checker._validate_curveUv_data(prim)
        return checker.GetIssues()

    def _check_point_positions(self, prim):
        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        checker._validate_point_position(prim)
        return checker.GetIssues()

    def _check_shell_point_containment(self, prim):
        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        checker._validate_shell_point_containment(prim)
        return checker.GetIssues()

    @staticmethod
    def _make_curve_uv_prim(vertex_counts, weights=_UNAUTHORED):
        stage = Usd.Stage.CreateInMemory()
        brep_prim = stage.DefinePrim("/Brep", "BrepArray")
        brep_prim.SetMetadata(
            "apiSchemas",
            Sdf.TokenListOp.CreateExplicit(["BrepCurveUvNurbAPI"]),
        )

        orders = [0 if vertex_count == 0 else 2 for vertex_count in vertex_counts]
        control_vertex_count = sum(vertex_counts)
        knots = []
        for order, vertex_count in zip(orders, vertex_counts):
            knots.extend(float(index) for index in range(order + vertex_count))

        brep_prim.CreateAttribute("edgeuse:edgeIndex", Sdf.ValueTypeNames.UIntArray).Set(
            Vt.UIntArray(range(len(vertex_counts)))
        )
        brep_prim.CreateAttribute("brep:curveUv:nurb:order", Sdf.ValueTypeNames.UIntArray).Set(
            Vt.UIntArray(orders)
        )
        brep_prim.CreateAttribute("brep:curveUv:nurb:vertexCount", Sdf.ValueTypeNames.UIntArray).Set(
            Vt.UIntArray(vertex_counts)
        )
        brep_prim.CreateAttribute(
            "brep:curveUv:nurb:controlVertices",
            Sdf.ValueTypeNames.Double2Array,
        ).Set(Vt.Vec2dArray([Gf.Vec2d(float(index), 0.0) for index in range(control_vertex_count)]))
        brep_prim.CreateAttribute("brep:curveUv:nurb:knots", Sdf.ValueTypeNames.DoubleArray).Set(
            Vt.DoubleArray(knots)
        )
        if weights is not _UNAUTHORED:
            brep_prim.CreateAttribute("brep:curveUv:nurb:weights", Sdf.ValueTypeNames.DoubleArray).Set(
                Vt.DoubleArray(weights)
            )

        return stage, brep_prim

    @staticmethod
    def _requirement_messages(issues, requirement_code):
        return [
            issue.message
            for issue in issues
            if getattr(issue, "requirement", None) is not None
            and getattr(issue.requirement, "code", None) == requirement_code
        ]

    def test_expected_invalid_fixtures_report_expected_requirement(self):
        expected_invalid_files = sorted(
            path
            for path in self.test_files_dir.glob("Test_BA_*.usda")
        )
        self.assertTrue(expected_invalid_files, "No expected-invalid files were found")

        for usd_file in expected_invalid_files:
            with self.subTest(fixture=usd_file.name):
                print(f"Expected-invalid: {usd_file.name}", flush=True)
                expected_requirement = _expected_requirement(usd_file)
                self.assertIsNotNone(expected_requirement, f"Could not determine expected requirement for {usd_file.name}")

                issues = self._collect_issues(usd_file)
                issue_codes = {
                    issue.requirement.code
                    for issue in issues
                    if getattr(issue, "requirement", None) is not None and getattr(issue.requirement, "code", None)
                }

                self.assertIn(
                    expected_requirement,
                    issue_codes,
                    f"{usd_file.name} did not report expected requirement {expected_requirement}; got {sorted(issue_codes)}",
                )

        print(f"Checked {len(expected_invalid_files)} expected-invalid files", flush=True)

    def test_expected_valid_fixtures_validate_without_runtime_errors(self):
        all_fixtures = {
            *self.test_files_dir.glob("*.usda"),
            *self.test_files_dir.glob("*.usd"),
            *self.test_files_dir.glob("*.usdc"),
        }
        expected_valid_files = sorted(path for path in all_fixtures if not path.name.startswith("Test_BA_"))
        self.assertTrue(expected_valid_files, "No expected-valid files were found")
        checked_expected_valid_files = [path for path in expected_valid_files if self._has_brep_prims(path)]

        for usd_file in checked_expected_valid_files:
            with self.subTest(fixture=usd_file.name):
                print(f"Expected-valid: {usd_file.name}", flush=True)
                issues = self._collect_issues(usd_file)
                self.assertIsInstance(issues, list)

        print(
            f"Checked {len(checked_expected_valid_files)} expected-valid files with BrepArray prims "
            f"({len(expected_valid_files) - len(checked_expected_valid_files)} skipped without BrepArray prims)",
            flush=True,
        )

    def _stage_asymmetric_triangle_loop(self):
        """Build a three-edge loop whose 3D edges, orientation tokens and stored UV trims
        all agree with the documented contract: UV trims run in edgeuse traversal order.

        Triangle v0(0,0,0) -> v1(1,0,0) -> v2(0,1,0) -> v0. Edge 1 is deliberately stored
        backwards, as v2->v1, so traversing it v1->v2 needs an ``opposite`` edgeuse. That
        asymmetry is the point: a two-edge loop stays closed when every stored trim is
        reversed, so it cannot tell a correct implementation from a uniformly reversing
        one. Here both that error and re-applying orientation to the opposite use break
        closure.
        """
        stage = Usd.Stage.Open(str(self.test_files_dir / "Test_BA_763_UV_trim_curves_do_not_close_loop.usda"))
        prim = next(p for p in stage.TraverseAll() if p.GetTypeName() == "BrepArray")

        prim.GetAttribute("face:loopCount").Set(Vt.UIntArray([1]))
        prim.GetAttribute("loop:edgeuseCount").Set(Vt.UIntArray([3]))
        prim.GetAttribute("edgeuse:edgeIndex").Set(Vt.UIntArray([0, 1, 2]))
        # Edge 1 runs v2->v1, against the traversal, hence "opposite".
        prim.GetAttribute("edgeuse:orientationType").Set(Vt.TokenArray(["same", "opposite", "same"]))
        prim.GetAttribute("edge:vertexIndices").Set(Vt.Vec2iArray([(0, 1), (2, 1), (2, 0)]))
        prim.GetAttribute("edge:curveType").Set(
            Vt.TokenArray(["BrepCurve3dLineAPI", "BrepCurve3dLineAPI", "BrepCurve3dLineAPI"])
        )
        prim.GetAttribute("edge:range").Set(Vt.DoubleArray([0, 1, 0, math.sqrt(2.0), 0, 1]))
        prim.GetAttribute("vertex:pointType").Set(
            Vt.TokenArray(["BrepPointAPI", "BrepPointAPI", "BrepPointAPI"])
        )
        prim.GetAttribute("brep:vertexPoint:point:position").Set(
            Vt.Vec3dArray([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
        )
        prim.GetAttribute("brep:edge3dLine:curve3d:line:origin").Set(
            Vt.Vec3dArray([(0, 0, 0), (0, 1, 0), (0, 1, 0)])
        )
        r2 = 1.0 / math.sqrt(2.0)
        prim.GetAttribute("brep:edge3dLine:curve3d:line:direction").Set(
            Vt.Vec3dArray([(1, 0, 0), (r2, -r2, 0), (0, -1, 0)])
        )
        prim.GetAttribute("brep:curveUv:nurb:order").Set(Vt.UIntArray([2, 2, 2]))
        prim.GetAttribute("brep:curveUv:nurb:vertexCount").Set(Vt.UIntArray([2, 2, 2]))
        prim.GetAttribute("brep:curveUv:nurb:knots").Set(
            Vt.DoubleArray([0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1])
        )
        prim.GetAttribute("brep:curveUv:nurb:weights").Set(Vt.DoubleArray([1] * 6))
        # Return the stage too: it owns the prim, and letting it fall out of scope
        # expires the handle before the caller can use it.
        return stage, prim

    def test_uv_loop_closure_uses_stored_traversal_direction(self):
        # Trims in traversal order: v0->v1, v1->v2 (opposite use of the v2->v1 edge), v2->v0.
        traversal_order = [(0, 0), (1, 0), (1, 0), (0, 1), (0, 1), (0, 0)]
        # Every trim reversed. A two-edge loop survives this; an asymmetric one must not.
        all_reversed = [(1, 0), (0, 0), (0, 1), (1, 0), (0, 0), (0, 1)]
        # The opposite use stored against traversal, i.e. following its 3D edge v2->v1.
        # Re-applying orientation to a correct trim produces exactly this, so the single
        # case covers both that consumer bug and the edge-directed authoring convention.
        edge_directed_opposite_use = [(0, 0), (1, 0), (0, 1), (1, 0), (0, 1), (0, 0)]
        # A plain gap, to keep a straightforward positive detection.
        real_gap = [(0, 0), (1, 0), (1, 0), (0, 1), (0, 1), (0.25, 0.25)]

        for label, curves, expect_issue in (
            ("traversal order", traversal_order, False),
            ("all trims reversed", all_reversed, True),
            ("opposite use stored edge-directed", edge_directed_opposite_use, True),
            ("real gap", real_gap, True),
        ):
            with self.subTest(case=label):
                _stage, prim = self._stage_asymmetric_triangle_loop()
                prim.GetAttribute("brep:curveUv:nurb:controlVertices").Set(Vt.Vec2dArray(curves))
                checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
                checker._validate_uv_loop_closure(prim)
                self.assertEqual(
                    bool(self._requirement_messages(checker.GetIssues(), "BA.763")),
                    expect_issue,
                    f"{label}: unexpected BA.763 result",
                )

    def test_sparse_uv_curve_records_are_valid(self):
        stage = Usd.Stage.Open(str(self.test_files_dir / "Test_BA_763_UV_trim_curves_do_not_close_loop.usda"))
        self.assertIsNotNone(stage)
        brep_prim = next(prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray")

        # Keep the first curve and replace the second with the documented no-curve sentinel.
        brep_prim.GetAttribute("brep:curveUv:nurb:order").Set(Vt.UIntArray([2, 0]))
        brep_prim.GetAttribute("brep:curveUv:nurb:vertexCount").Set(Vt.UIntArray([2, 0]))
        brep_prim.GetAttribute("brep:curveUv:nurb:controlVertices").Set(
            Vt.Vec2dArray([Gf.Vec2d(0.0, 0.0), Gf.Vec2d(1.0, 0.0)])
        )
        brep_prim.GetAttribute("brep:curveUv:nurb:knots").Set(Vt.DoubleArray([0.0, 0.0, 1.0, 1.0]))
        brep_prim.GetAttribute("brep:curveUv:nurb:weights").Set(Vt.DoubleArray([1.0, 1.0]))

        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        checker.CheckPrim(brep_prim)
        issue_codes = {
            issue.requirement.code
            for issue in checker.GetIssues()
            if getattr(issue, "requirement", None) is not None and getattr(issue.requirement, "code", None)
        }
        sparse_uv_requirements = {
            "BA.375",
            "BA.380",
            "BA.385",
            "BA.390",
            "BA.395",
            "BA.400",
            "BA.405",
            "BA.410",
            "BA.415",
            "BA.416",
            "BA.590",
            "BA.591",
            "BA.750",
            "BA.763",
            "BA.764",
        }
        self.assertFalse(
            issue_codes & sparse_uv_requirements,
            f"Sparse UV sentinel produced unexpected requirements: {sorted(issue_codes & sparse_uv_requirements)}",
        )

    def test_uv_curve_weights_match_packed_control_vertex_count(self):
        cases = (
            ("missing", [2], _UNAUTHORED, 0, True),
            ("authored empty", [2], [], 0, True),
            ("short", [2], [1.0], 1, True),
            ("exact", [2], [1.0, 1.0], 2, False),
            ("long", [2], [1.0, 1.0, 1.0], 3, True),
            ("packed short", [2, 3], [1.0] * 4, 4, True),
            ("packed exact", [2, 3], [1.0] * 5, 5, False),
            ("packed long", [2, 3], [1.0] * 6, 6, True),
            ("missing sentinel weights", [0, 0], _UNAUTHORED, 0, False),
            ("mixed sentinel", [2, 0], [1.0, 1.0], 2, False),
        )

        for name, vertex_counts, weights, actual_count, should_fail in cases:
            with self.subTest(case=name):
                _stage, brep_prim = self._make_curve_uv_prim(vertex_counts, weights)
                messages = self._requirement_messages(self._check_curve_uv_data(brep_prim), "BA.405")

                if should_fail:
                    self.assertEqual(1, len(messages), messages)
                    self.assertIn(f"Expected size {sum(vertex_counts)}, but got {actual_count}", messages[0])
                else:
                    self.assertEqual([], messages)

    def test_absent_uv_curves_do_not_require_weights(self):
        stage = Usd.Stage.CreateInMemory()
        brep_prim = stage.DefinePrim("/Brep", "BrepArray")
        brep_prim.SetMetadata(
            "apiSchemas",
            Sdf.TokenListOp.CreateExplicit(["BrepCurveUvNurbAPI"]),
        )

        messages = self._requirement_messages(self._check_curve_uv_data(brep_prim), "BA.405")

        self.assertEqual([], messages)

    def test_uv_curve_weight_positivity_is_independent_of_cardinality(self):
        _stage, brep_prim = self._make_curve_uv_prim([2], [0.0, 1.0])
        issues = self._check_curve_uv_data(brep_prim)

        self.assertEqual([], self._requirement_messages(issues, "BA.405"))
        self.assertEqual(1, len(self._requirement_messages(issues, "BA.410")))

    def test_one_missing_required_geometry_api_is_reported(self):
        _stage, brep_prim = self._open_cube_prim()
        remaining_schemas = [
            schema for schema in brep_prim.GetAppliedSchemas() if schema != "BrepSurfaceNurbAPI"
        ]
        brep_prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(remaining_schemas))

        messages = self._requirement_messages(self._check_prim(brep_prim), "BA.583")

        self.assertEqual(1, len(messages), messages)
        self.assertIn("face:surfaceType", messages[0])
        self.assertIn("BrepSurfaceNurbAPI", messages[0])

    def test_all_missing_required_geometry_apis_are_reported(self):
        _stage, brep_prim = self._open_cube_prim()
        brep_prim.ClearMetadata("apiSchemas")

        messages = self._requirement_messages(self._check_prim(brep_prim), "BA.583")

        self.assertEqual(3, len(messages), messages)
        for expected_api in (
            "BrepPointAPI:vertexPoint",
            "BrepCurve3dNurbAPI:edge3dNurb",
            "BrepSurfaceNurbAPI",
        ):
            self.assertTrue(any(expected_api in message for message in messages), messages)

    def test_absent_geometry_api_without_occurrence_is_accepted(self):
        stage = Usd.Stage.CreateInMemory()
        brep_prim = stage.DefinePrim("/Brep", "BrepArray")

        messages = self._requirement_messages(self._check_required_geometry_apis(brep_prim), "BA.583")

        self.assertEqual([], messages)

    def test_pcurve_api_is_optional_without_authored_uv_data(self):
        _stage, brep_prim = self._open_cube_prim()
        self.assertNotIn("BrepCurveUvNurbAPI", brep_prim.GetAppliedSchemas())
        self.assertTrue(brep_prim.GetAttribute("edgeuse:edgeIndex").Get())

        messages = self._requirement_messages(self._check_prim(brep_prim), "BA.583")

        self.assertEqual([], messages)

    def test_ignored_shell_point_tokens_do_not_create_geometry_occurrences(self):
        for shell_kind, faceuse_count, wireedge_count in (
            ("face", 1, 0),
            ("wire", 0, 1),
        ):
            with self.subTest(shell=shell_kind):
                stage = Usd.Stage.CreateInMemory()
                brep_prim = stage.DefinePrim("/Brep", "BrepArray")
                brep_prim.GetAttribute("shell:pointType").Set(Vt.TokenArray(["BrepPointAPI"]))
                brep_prim.GetAttribute("shell:faceuseCount").Set(Vt.UIntArray([faceuse_count]))
                brep_prim.GetAttribute("shell:wireEdgeCount").Set(Vt.UIntArray([wireedge_count]))

                self.assertEqual(
                    [],
                    self._requirement_messages(self._check_point_positions(brep_prim), "BA.325"),
                )
                self.assertEqual(
                    [],
                    self._requirement_messages(self._check_required_geometry_apis(brep_prim), "BA.583"),
                )

    def test_true_point_shell_requires_exactly_one_position_occurrence(self):
        stage = Usd.Stage.CreateInMemory()
        brep_prim = stage.DefinePrim("/Brep", "BrepArray")
        brep_prim.GetAttribute("shell:pointType").Set(Vt.TokenArray(["BrepPointAPI"]))
        brep_prim.GetAttribute("shell:faceuseCount").Set(Vt.UIntArray([0]))
        brep_prim.GetAttribute("shell:wireEdgeCount").Set(Vt.UIntArray([0]))

        messages = self._requirement_messages(self._check_point_positions(brep_prim), "BA.325")
        self.assertEqual(1, len(messages), messages)

        brep_prim.CreateAttribute(
            "brep:shellPoint:point:position",
            Sdf.ValueTypeNames.Point3dArray,
        ).Set(Vt.Vec3dArray([(1.0, 2.0, 3.0)]))
        self.assertEqual(
            [],
            self._requirement_messages(self._check_point_positions(brep_prim), "BA.325"),
        )

    def test_ignored_shell_point_token_does_not_shift_later_brep_position_span(self):
        stage = Usd.Stage.CreateInMemory()
        brep_prim = stage.DefinePrim("/Brep", "BrepArray")

        # Brep 0 has a face shell whose point token is ignored. Brep 1 has the
        # sole point-shell occurrence and therefore owns position[0].
        brep_prim.GetAttribute("brep:regionCount").Set(Vt.UIntArray([1, 1]))
        brep_prim.GetAttribute("region:shellCount").Set(Vt.UIntArray([1, 1]))
        brep_prim.GetAttribute("shell:faceuseCount").Set(Vt.UIntArray([1, 0]))
        brep_prim.GetAttribute("shell:wireEdgeCount").Set(Vt.UIntArray([0, 0]))
        brep_prim.GetAttribute("shell:pointType").Set(
            Vt.TokenArray(["BrepPointAPI", "BrepPointAPI"])
        )
        brep_prim.GetAttribute("brep:extent").Set(
            Vt.Vec3dArray([(-1, -1, -1), (1, 1, 1), (9, 9, 9), (11, 11, 11)])
        )
        shell_positions = brep_prim.CreateAttribute(
            "brep:shellPoint:point:position",
            Sdf.ValueTypeNames.Point3dArray,
        )
        shell_positions.Set(Vt.Vec3dArray([(10, 10, 10)]))

        self.assertEqual(
            [],
            self._requirement_messages(self._check_point_positions(brep_prim), "BA.325"),
        )
        self.assertEqual(
            [],
            self._requirement_messages(self._check_shell_point_containment(brep_prim), "BA.710"),
        )

        shell_positions.Set(Vt.Vec3dArray([(12, 10, 10)]))
        messages = self._requirement_messages(
            self._check_shell_point_containment(brep_prim),
            "BA.710",
        )
        self.assertEqual(1, len(messages), messages)
        self.assertIn("brep #1", messages[0])

    def test_authored_pcurve_data_requires_pcurve_api(self):
        stage = Usd.Stage.CreateInMemory()
        brep_prim = stage.DefinePrim("/Brep", "BrepArray")
        brep_prim.CreateAttribute("brep:curveUv:nurb:order", Sdf.ValueTypeNames.UIntArray).Set(
            Vt.UIntArray([2])
        )
        brep_prim.CreateAttribute("brep:curveUv:nurb:vertexCount", Sdf.ValueTypeNames.UIntArray).Set(
            Vt.UIntArray([2])
        )

        messages = self._requirement_messages(self._check_prim(brep_prim), "BA.583")

        self.assertEqual(1, len(messages), messages)
        self.assertIn("BrepCurveUvNurbAPI", messages[0])

        brep_prim.SetMetadata(
            "apiSchemas",
            Sdf.TokenListOp.CreateExplicit(["BrepCurveUvNurbAPI"]),
        )
        self.assertEqual(
            [],
            self._requirement_messages(self._check_prim(brep_prim), "BA.583"),
        )

    def test_every_geometry_token_maps_to_its_required_api(self):
        cases = (
            ("vertex:pointType", "BrepPointAPI", "BrepPointAPI:vertexPoint"),
            ("shell:pointType", "BrepPointAPI", "BrepPointAPI:shellPoint"),
            ("edge:curveType", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI:edge3dNurb"),
            ("edge:curveType", "BrepCurve3dLineAPI", "BrepCurve3dLineAPI:edge3dLine"),
            ("edge:curveType", "BrepCurve3dCircleAPI", "BrepCurve3dCircleAPI:edge3dCircle"),
            ("edge:curveType", "BrepCurve3dEllipseAPI", "BrepCurve3dEllipseAPI:edge3dEllipse"),
            (
                "wireEdge:curveType",
                "BrepCurve3dNurbAPI",
                "BrepCurve3dNurbAPI:wireEdge3dNurb",
            ),
            (
                "wireEdge:curveType",
                "BrepCurve3dLineAPI",
                "BrepCurve3dLineAPI:wireEdge3dLine",
            ),
            (
                "wireEdge:curveType",
                "BrepCurve3dCircleAPI",
                "BrepCurve3dCircleAPI:wireEdge3dCircle",
            ),
            (
                "wireEdge:curveType",
                "BrepCurve3dEllipseAPI",
                "BrepCurve3dEllipseAPI:wireEdge3dEllipse",
            ),
            ("face:surfaceType", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"),
            ("face:surfaceType", "BrepSurfacePlaneAPI", "BrepSurfacePlaneAPI"),
            ("face:surfaceType", "BrepSurfaceCylinderAPI", "BrepSurfaceCylinderAPI"),
            ("face:surfaceType", "BrepSurfaceConeAPI", "BrepSurfaceConeAPI"),
            ("face:surfaceType", "BrepSurfaceSphereAPI", "BrepSurfaceSphereAPI"),
            ("face:surfaceType", "BrepSurfaceTorusAPI", "BrepSurfaceTorusAPI"),
        )

        for attribute_name, token, expected_api in cases:
            with self.subTest(attribute=attribute_name, token=token):
                stage = Usd.Stage.CreateInMemory()
                brep_prim = stage.DefinePrim("/Brep", "BrepArray")
                brep_prim.GetAttribute(attribute_name).Set(Vt.TokenArray([token]))
                if attribute_name == "shell:pointType":
                    brep_prim.GetAttribute("shell:faceuseCount").Set(Vt.UIntArray([0]))
                    brep_prim.GetAttribute("shell:wireEdgeCount").Set(Vt.UIntArray([0]))

                messages = self._requirement_messages(
                    self._check_required_geometry_apis(brep_prim),
                    "BA.583",
                )

                self.assertEqual(1, len(messages), messages)
                self.assertIn(expected_api, messages[0])

                brep_prim.SetMetadata(
                    "apiSchemas",
                    Sdf.TokenListOp.CreateExplicit([expected_api]),
                )
                self.assertEqual(
                    [],
                    self._requirement_messages(self._check_required_geometry_apis(brep_prim), "BA.583"),
                )
