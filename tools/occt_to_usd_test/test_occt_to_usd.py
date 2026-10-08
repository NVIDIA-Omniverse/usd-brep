# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for the standalone OCCT `.brep` -> USD BrepArray converter (`occt_to_usd`).

Runs the `occt_to_usd` CLI on a corpus of `.brep` fixtures, then validates each emitted BrepArray
with the same `BrepValidator` the `brep_validator_test` suite uses, asserting zero failed checks.

The committed corpus additionally runs a SMLib round-trip self-consistency check: each emitted
BrepArray is re-imported into an `SmBrep` and run through `AssertValid` (via
``usd_test_app --assert-valid``). This catches "clearly wrong" output (broken topology/geometry)
that still satisfies the USD schema. The check runs in a subprocess so an importer crash surfaces
as a failing test rather than killing the runner.

Two suites keep shareable and unshareable data isolated:

  * ``OcctBrepCorpusTestCase`` -- a small, self-authored corpus committed under
    ``TestFiles/occt_breps`` (analytic primitives, NURBS, periodic NURBS, extrusion, revolution).
    These shapes are authored independently, so they carry no third-party data license.

  * ``OcctBrepExternalTestCase`` -- proprietary / third-party parts (e.g. the OpenCASCADE
    distribution samples) that cannot be shared. They live in the git-ignored
    ``TestFiles/occt_breps_external`` directory (or wherever ``OCCT_BREP_EXTERNAL_DIR`` points), so
    they are never committed. The suite skips when no such files are present.

The USD-runtime bootstrap and the `BrepValidator` itself are reused from the `brep_validator_test`
harness (its ``utils.base_test_case``).
"""

import atexit
import math
import os
from collections import Counter
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

# Reuse the brep_validator test harness: it bootstraps the USD runtime, installs
# usd-validation-nvidia, and exposes BrepValidatorBaseTestCase (with _collect_issues).
_THIS_DIR = Path(__file__).resolve().parent
_VALIDATOR_TEST_DIR = _THIS_DIR.parent / "brep_validator_test"
if str(_VALIDATOR_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(_VALIDATOR_TEST_DIR))

from utils.base_test_case import REPO_ROOT, BrepValidatorBaseTestCase  # noqa: E402
from pxr import Gf, Usd  # noqa: E402


class _OcctBrepTestSummary:
    """Sample-level result accounting; unittest only counts the two test methods."""

    def __init__(self):
        self.successes = []
        self.failures = []
        self.skips = []
        self.suite_skips = []

    @staticmethod
    def _label(suite: str, sample: Path) -> str:
        return f"{suite}/{sample.name}"

    @staticmethod
    def _short(reason: object, limit: int = 500) -> str:
        text = str(reason).strip() or "(no reason reported)"
        return text if len(text) <= limit else text[: limit - 3] + "..."

    def record_success(self, suite: str, sample: Path):
        self.successes.append(self._label(suite, sample))

    def record_failure(self, suite: str, sample: Path, reason: object):
        self.failures.append((self._label(suite, sample), self._short(reason)))

    def record_skip(self, suite: str, sample: Path, reason: object):
        self.skips.append((self._label(suite, sample), self._short(reason)))

    def record_suite_skip(self, suite: str, reason: object):
        self.suite_skips.append((suite, self._short(reason)))

    def print(self):
        sample_count = len(self.successes) + len(self.failures) + len(self.skips)
        if sample_count == 0 and not self.suite_skips:
            return

        print("\nOCCT BRep import sample summary:", file=sys.stderr)
        print(f"  sample runs: {sample_count}", file=sys.stderr)
        print(f"  successful: {len(self.successes)}", file=sys.stderr)
        print(f"  failed: {len(self.failures)}", file=sys.stderr)
        print(f"  skipped: {len(self.skips)}", file=sys.stderr)

        if self.failures:
            print("  failure details:", file=sys.stderr)
            for label, reason in self.failures:
                print(f"    - {label}: {reason}", file=sys.stderr)

        if self.skips:
            print("  skip reasons:", file=sys.stderr)
            for reason, count in Counter(reason for _, reason in self.skips).items():
                print(f"    - {count} sample(s): {reason}", file=sys.stderr)

        if self.suite_skips:
            print("  suite skips:", file=sys.stderr)
            for suite, reason in self.suite_skips:
                print(f"    - {suite}: {reason}", file=sys.stderr)


_SUMMARY = _OcctBrepTestSummary()
atexit.register(_SUMMARY.print)


def _built_binary(name: str, env_override: str) -> Path:
    """Locate a built binary by name: env override, else the built path (release preferred)."""
    override = os.environ.get(env_override)
    if override:
        return Path(override)
    candidates = sorted((REPO_ROOT / "_build").glob(f"*/*/{name}"))
    release = [c for c in candidates if c.parent.name == "release"]
    chosen = release or candidates
    return chosen[0] if chosen else (REPO_ROOT / "_build" / "linux-x86_64" / "release" / name)


def _tool_path() -> Path:
    """Locate the occt_to_usd CLI."""
    return _built_binary("occt_to_usd", "OCCT_TO_USD")


def _usd_test_app_path() -> Path:
    """Locate the usd_test_app binary (provides the --assert-valid round-trip check)."""
    return _built_binary("usd_test_app", "USD_TEST_APP")


class _OcctBrepConvertMixin(BrepValidatorBaseTestCase):
    """Shared convert + validate machinery; concrete suites supply the input directory."""

    inputs_dir: Path = None
    # Per-subprocess wall-clock cap so a hung binary fails the test instead of stalling the whole job.
    _timeout_sec = int(os.environ.get("OCCT_BREP_TEST_TIMEOUT_SEC", "300"))

    def setUp(self):
        self.tool = _tool_path()
        if not self.tool.is_file():
            self.skipTest(f"occt_to_usd not built at {self.tool}")
        self._tmp = tempfile.mkdtemp(prefix="occt_brep_test_")
        self._opened_stages = []
        self.addCleanup(shutil.rmtree, self._tmp, ignore_errors=True)
        self.addCleanup(self._opened_stages.clear)

    def _discover(self):
        if not self.inputs_dir or not self.inputs_dir.is_dir():
            return []
        return sorted(p for p in self.inputs_dir.rglob("*.brep") if p.is_file())

    @staticmethod
    def _is_non_brep_topology_error(result: subprocess.CompletedProcess) -> bool:
        return (
            result.returncode != 0
            and "no Solid/Shell/Face component found" in (result.stderr or "")
        )

    def _run_convert(self, brep_path: Path):
        rel = brep_path.relative_to(self.inputs_dir)
        out_path = Path(self._tmp) / rel.with_suffix(".usda")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [str(self.tool), str(brep_path), str(out_path)],
            capture_output=True,
            text=True,
            env=os.environ,
            timeout=self._timeout_sec,
        )
        return result, out_path

    def _convert(self, brep_path: Path) -> Path:
        result, out_path = self._run_convert(brep_path)
        self.assertEqual(result.returncode, 0, msg=f"occt_to_usd failed for {brep_path}: {result.stderr.strip()}")
        self.assertTrue(out_path.is_file(), msg=f"no output produced for {brep_path}")
        return out_path

    def _validate_schema(self, out_path: Path, brep_path: Path):
        """USD-level validation: run BrepValidator on the emitted BrepArray."""
        issues = self._collect_issues(out_path)
        if issues:
            detail = "; ".join(
                f"{getattr(getattr(i, 'requirement', None), 'code', '?')}: {getattr(i, 'message', i)}" for i in issues
            )
            self.fail(f"BrepValidator reported {len(issues)} failed check(s) for {brep_path.name}: {detail}")

    def _assert_valid_ranges(self, out_path: Path, brep_path: Path):
        """Importer-authored ranges must be finite, non-empty, and bounded for line edges."""
        stage = Usd.Stage.Open(str(out_path))
        self.assertIsNotNone(stage, f"Failed to open USD stage: {out_path}")
        brep_prims = [prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]
        self.assertTrue(brep_prims, f"No BrepArray prims found in {out_path}")

        for prim in brep_prims:
            edge_ranges = prim.GetAttribute("edge:range").Get()
            edge_curve_types = prim.GetAttribute("edge:curveType").Get()
            face_ranges = prim.GetAttribute("face:range").Get()
            extent = prim.GetAttribute("brep:extent").Get() or prim.GetAttribute("extent").Get()

            self.assertIsNotNone(edge_ranges, msg=f"missing edge:range for {brep_path.name}")
            self.assertIsNotNone(edge_curve_types, msg=f"missing edge:curveType for {brep_path.name}")
            self.assertIsNotNone(face_ranges, msg=f"missing face:range for {brep_path.name}")
            self.assertEqual(
                len(edge_ranges),
                2 * len(edge_curve_types),
                msg=f"edge range/type size mismatch for {brep_path.name}",
            )
            self.assertEqual(len(face_ranges) % 2, 0, msg=f"odd face:range pair count for {brep_path.name}")

            diag = 1.0
            if extent and len(extent) >= 2:
                diag = max(1.0, (extent[1] - extent[0]).GetLength())

            for i, edge_type in enumerate(edge_curve_types):
                lo = float(edge_ranges[2 * i])
                hi = float(edge_ranges[2 * i + 1])
                self.assertTrue(
                    math.isfinite(lo) and math.isfinite(hi) and hi > lo,
                    msg=f"bad edge range {i} for {brep_path.name}: [{lo}, {hi}]",
                )
                if str(edge_type) == "BrepCurve3dLineAPI":
                    self.assertLessEqual(
                        hi - lo,
                        10.0 * diag,
                        msg=f"unbounded line edge range {i} for {brep_path.name}: [{lo}, {hi}]",
                    )

            periodic_surface_types = {
                "BrepSurfaceCylinderAPI",
                "BrepSurfaceConeAPI",
                "BrepSurfaceSphereAPI",
                "BrepSurfaceTorusAPI",
            }
            face_surface_types = prim.GetAttribute("face:surfaceType").Get()
            self.assertEqual(
                len(face_ranges),
                2 * len(face_surface_types),
                msg=f"face range/type size mismatch for {brep_path.name}",
            )

            for i in range(0, len(face_ranges), 2):
                lo = face_ranges[i]
                hi = face_ranges[i + 1]
                ok = (
                    math.isfinite(float(lo[0]))
                    and math.isfinite(float(lo[1]))
                    and math.isfinite(float(hi[0]))
                    and math.isfinite(float(hi[1]))
                    and hi[0] > lo[0]
                    and hi[1] > lo[1]
                )
                self.assertTrue(ok, msg=f"bad face range {i // 2} for {brep_path.name}: [{lo}, {hi}]")
                if str(face_surface_types[i // 2]) in periodic_surface_types:
                    self.assertLessEqual(
                        float(hi[0]) - float(lo[0]),
                        2.0 * math.pi + 1.0e-12,
                        msg=f"overfull analytic U range {i // 2} for {brep_path.name}: [{lo[0]}, {hi[0]}]",
                    )

            self._assert_cylinder_ranges_cover_boundary_vertices(prim, brep_path)
            self._assert_planar_outer_loops_first(prim, brep_path)

    @staticmethod
    def _evaluate_nurb_curve(control_points, weights, knots, order, parameter):
        """Evaluate one rational B-spline using its full knot vector."""
        degree = order - 1
        last_control = len(control_points) - 1
        parameter = min(max(parameter, knots[degree]), knots[last_control + 1])

        if parameter >= knots[last_control + 1]:
            span = last_control
        else:
            span = degree
            while span < last_control and parameter >= knots[span + 1]:
                span += 1

        work = []
        for index in range(span - degree, span + 1):
            weight = weights[index]
            point = control_points[index]
            work.append(
                [
                    float(point[0]) * weight,
                    float(point[1]) * weight,
                    float(point[2]) * weight,
                    weight,
                ]
            )

        for level in range(1, degree + 1):
            for local_index in range(degree, level - 1, -1):
                knot_index = span - degree + local_index
                denominator = knots[knot_index + degree - level + 1] - knots[knot_index]
                alpha = 0.0 if denominator == 0.0 else (parameter - knots[knot_index]) / denominator
                work[local_index] = [
                    (1.0 - alpha) * work[local_index - 1][component] + alpha * work[local_index][component]
                    for component in range(4)
                ]

        result = work[degree]
        if result[3] == 0.0:
            return Gf.Vec3d(result[0], result[1], result[2])
        return Gf.Vec3d(result[0] / result[3], result[1] / result[3], result[2] / result[3])

    def _sample_edge_curves(self, prim):
        """Return 25 geometry samples per edge, matching the importer's boundary-domain sampling."""
        edge_types = [str(value) for value in prim.GetAttribute("edge:curveType").Get()]
        edge_ranges = prim.GetAttribute("edge:range").Get()

        def values(attribute_name):
            return list(prim.GetAttribute(attribute_name).Get() or [])

        circle_centers = values("brep:edge3dCircle:curve3d:circle:center")
        circle_axes = values("brep:edge3dCircle:curve3d:circle:axis")
        circle_refs = values("brep:edge3dCircle:curve3d:circle:refDirection")
        circle_radii = values("brep:edge3dCircle:curve3d:circle:radius")
        ellipse_centers = values("brep:edge3dEllipse:curve3d:ellipse:center")
        ellipse_axes = values("brep:edge3dEllipse:curve3d:ellipse:axis")
        ellipse_refs = values("brep:edge3dEllipse:curve3d:ellipse:refDirection")
        ellipse_x_radii = values("brep:edge3dEllipse:curve3d:ellipse:xRadius")
        ellipse_y_radii = values("brep:edge3dEllipse:curve3d:ellipse:yRadius")
        line_origins = values("brep:edge3dLine:curve3d:line:origin")
        line_directions = values("brep:edge3dLine:curve3d:line:direction")
        nurb_control_points = values("brep:edge3dNurb:curve3d:nurb:controlVertices")
        nurb_vertex_counts = values("brep:edge3dNurb:curve3d:nurb:vertexCount")
        nurb_orders = values("brep:edge3dNurb:curve3d:nurb:order")
        nurb_knots = values("brep:edge3dNurb:curve3d:nurb:knots")
        nurb_weights = values("brep:edge3dNurb:curve3d:nurb:weights")

        type_indices = Counter()
        nurb_control_offset = 0
        nurb_knot_offset = 0
        edge_samples = []
        for edge_index, edge_type in enumerate(edge_types):
            curve_index = type_indices[edge_type]
            type_indices[edge_type] += 1
            start = float(edge_ranges[2 * edge_index])
            end = float(edge_ranges[2 * edge_index + 1])
            parameters = [start + (end - start) * sample / 24.0 for sample in range(25)]

            if edge_type == "BrepCurve3dLineAPI":
                origin = Gf.Vec3d(line_origins[curve_index])
                direction = Gf.Vec3d(line_directions[curve_index])
                samples = [origin + parameter * direction for parameter in parameters]
            elif edge_type in ("BrepCurve3dCircleAPI", "BrepCurve3dEllipseAPI"):
                if edge_type == "BrepCurve3dCircleAPI":
                    center = Gf.Vec3d(circle_centers[curve_index])
                    axis = Gf.Vec3d(circle_axes[curve_index])
                    ref = Gf.Vec3d(circle_refs[curve_index])
                    x_radius = y_radius = float(circle_radii[curve_index])
                else:
                    center = Gf.Vec3d(ellipse_centers[curve_index])
                    axis = Gf.Vec3d(ellipse_axes[curve_index])
                    ref = Gf.Vec3d(ellipse_refs[curve_index])
                    x_radius = float(ellipse_x_radii[curve_index])
                    y_radius = float(ellipse_y_radii[curve_index])
                axis.Normalize()
                ref.Normalize()
                side = Gf.Cross(axis, ref)
                side.Normalize()
                samples = [
                    center + x_radius * math.cos(parameter) * ref + y_radius * math.sin(parameter) * side
                    for parameter in parameters
                ]
            elif edge_type == "BrepCurve3dNurbAPI":
                vertex_count = int(nurb_vertex_counts[curve_index])
                order = int(nurb_orders[curve_index])
                knot_count = vertex_count + order
                control_points = nurb_control_points[nurb_control_offset : nurb_control_offset + vertex_count]
                weights = [
                    float(value) for value in nurb_weights[nurb_control_offset : nurb_control_offset + vertex_count]
                ]
                knots = [float(value) for value in nurb_knots[nurb_knot_offset : nurb_knot_offset + knot_count]]
                samples = [
                    self._evaluate_nurb_curve(control_points, weights, knots, order, parameter)
                    for parameter in parameters
                ]
                nurb_control_offset += vertex_count
                nurb_knot_offset += knot_count
            else:
                samples = []
            edge_samples.append(samples)

        return edge_samples

    def _assert_planar_outer_loops_first(self, prim, brep_path: Path):
        """Planar faces with an unambiguous outer wire must stage that loop first."""
        face_types = prim.GetAttribute("face:surfaceType").Get()
        face_loop_counts = prim.GetAttribute("face:loopCount").Get()
        loop_edgeuse_counts = prim.GetAttribute("loop:edgeuseCount").Get()
        edgeuse_edge_indices = prim.GetAttribute("edgeuse:edgeIndex").Get()
        edge_vertex_indices = prim.GetAttribute("edge:vertexIndices").Get()
        vertex_points = prim.GetAttribute("brep:vertexPoint:point:position").Get()
        plane_origins = prim.GetAttribute("brep:surface:plane:origin").Get()
        plane_axes = prim.GetAttribute("brep:surface:plane:axis").Get()
        plane_refs = prim.GetAttribute("brep:surface:plane:refDirection").Get()

        required = (
            face_types,
            face_loop_counts,
            loop_edgeuse_counts,
            edgeuse_edge_indices,
            edge_vertex_indices,
            vertex_points,
        )
        if not all(value is not None for value in required):
            return

        plane_origins = plane_origins or []
        plane_axes = plane_axes or []
        plane_refs = plane_refs or []
        edge_samples = self._sample_edge_curves(prim)
        loop_index = 0
        edgeuse_index = 0
        plane_index = 0

        for face_index, surface_type in enumerate(face_types):
            face_loops = []
            for _ in range(int(face_loop_counts[face_index])):
                edgeuse_count = int(loop_edgeuse_counts[loop_index])
                face_loops.append(
                    [int(index) for index in edgeuse_edge_indices[edgeuse_index : edgeuse_index + edgeuse_count]]
                )
                edgeuse_index += edgeuse_count
                loop_index += 1

            if str(surface_type) != "BrepSurfacePlaneAPI":
                continue
            self.assertLess(
                plane_index,
                min(len(plane_origins), len(plane_axes), len(plane_refs)),
                msg=f"missing plane data for {brep_path.name}",
            )

            if len(face_loops) > 1:
                origin = Gf.Vec3d(plane_origins[plane_index])
                axis = Gf.Vec3d(plane_axes[plane_index])
                u_dir = Gf.Vec3d(plane_refs[plane_index])
                self.assertGreater(axis.Normalize(), 0.0, msg=f"zero plane axis on face {face_index}")
                self.assertGreater(u_dir.Normalize(), 0.0, msg=f"zero plane ref direction on face {face_index}")
                v_dir = Gf.Cross(axis, u_dir)
                self.assertGreater(v_dir.Normalize(), 0.0, msg=f"degenerate plane frame on face {face_index}")

                loop_areas = []
                for edge_indices in face_loops:
                    points = [
                        point
                        for edge_index in edge_indices
                        for point in edge_samples[edge_index]
                    ] + [
                        Gf.Vec3d(vertex_points[vertex_index])
                        for edge_index in edge_indices
                        for vertex_index in edge_vertex_indices[edge_index]
                    ]
                    if not points:
                        loop_areas.append(0.0)
                        continue
                    uv = [
                        (Gf.Dot(point - origin, u_dir), Gf.Dot(point - origin, v_dir))
                        for point in points
                    ]
                    u_values = [value[0] for value in uv]
                    v_values = [value[1] for value in uv]
                    loop_areas.append((max(u_values) - min(u_values)) * (max(v_values) - min(v_values)))

                largest_area = max(loop_areas)
                area_tol = 1.0e-9 * max(1.0, largest_area)
                largest_loops = [
                    index for index, area in enumerate(loop_areas) if abs(area - largest_area) <= area_tol
                ]
                if largest_area > area_tol and len(largest_loops) == 1:
                    self.assertEqual(
                        largest_loops[0],
                        0,
                        msg=(
                            f"planar face {face_index} outer loop is not first for {brep_path.name}: "
                            f"projected loop areas={loop_areas}"
                        ),
                    )

            plane_index += 1

    def _assert_cylinder_ranges_cover_boundary_vertices(self, prim, brep_path: Path):
        """Cylinder face domains must contain their topological boundary in the authored Ax2 frame."""
        face_types = prim.GetAttribute("face:surfaceType").Get()
        if not face_types or not any(str(surface_type) == "BrepSurfaceCylinderAPI" for surface_type in face_types):
            return

        face_ranges = prim.GetAttribute("face:range").Get()
        face_loop_counts = prim.GetAttribute("face:loopCount").Get()
        loop_edgeuse_counts = prim.GetAttribute("loop:edgeuseCount").Get()
        edgeuse_edge_indices = prim.GetAttribute("edgeuse:edgeIndex").Get()
        edge_vertex_indices = prim.GetAttribute("edge:vertexIndices").Get()
        vertex_points = prim.GetAttribute("brep:vertexPoint:point:position").Get()
        cylinder_origins = prim.GetAttribute("brep:surface:cylinder:origin").Get()
        cylinder_axes = prim.GetAttribute("brep:surface:cylinder:axis").Get()
        cylinder_refs = prim.GetAttribute("brep:surface:cylinder:refDirection").Get()
        cylinder_radii = prim.GetAttribute("brep:surface:cylinder:radius").Get()
        brep_tolerances = prim.GetAttribute("brep:intersectTol3d").Get()

        required = (
            face_ranges,
            face_loop_counts,
            loop_edgeuse_counts,
            edgeuse_edge_indices,
            edge_vertex_indices,
            vertex_points,
            cylinder_origins,
            cylinder_axes,
            cylinder_refs,
            cylinder_radii,
            brep_tolerances,
        )
        self.assertTrue(
            all(value is not None for value in required),
            msg=f"missing cylinder topology data for {brep_path.name}",
        )
        self.assertGreater(
            len(brep_tolerances),
            0,
            msg=f"empty brep:intersectTol3d data for {brep_path.name}",
        )

        loop_index = 0
        edgeuse_index = 0
        cylinder_index = 0
        period = 2.0 * math.pi
        brep_tolerance = max(float(value) for value in brep_tolerances)
        for face_index, surface_type in enumerate(face_types):
            face_edge_indices = []
            for _ in range(face_loop_counts[face_index]):
                edgeuse_count = loop_edgeuse_counts[loop_index]
                face_edge_indices.extend(edgeuse_edge_indices[edgeuse_index : edgeuse_index + edgeuse_count])
                edgeuse_index += edgeuse_count
                loop_index += 1

            if str(surface_type) != "BrepSurfaceCylinderAPI":
                continue

            origin = Gf.Vec3d(cylinder_origins[cylinder_index])
            axis = Gf.Vec3d(cylinder_axes[cylinder_index])
            ref = Gf.Vec3d(cylinder_refs[cylinder_index])
            self.assertGreater(
                axis.Normalize(), 0.0, msg=f"zero cylinder axis on face {face_index} of {brep_path.name}"
            )
            self.assertGreater(
                ref.Normalize(), 0.0, msg=f"zero cylinder ref direction on face {face_index} of {brep_path.name}"
            )
            side = Gf.Cross(axis, ref)
            self.assertGreater(
                side.Normalize(), 0.0, msg=f"degenerate cylinder frame on face {face_index} of {brep_path.name}"
            )

            uv_min = face_ranges[2 * face_index]
            uv_max = face_ranges[2 * face_index + 1]
            u_min, v_min = float(uv_min[0]), float(uv_min[1])
            u_max, v_max = float(uv_max[0]), float(uv_max[1])
            radius = abs(float(cylinder_radii[cylinder_index]))
            angle_tol = max(1.0e-6, brep_tolerance / max(radius, 1.0e-12))
            axial_tol = max(1.0e-6 * max(1.0, abs(v_min), abs(v_max)), brep_tolerance)
            vertex_indices = sorted(
                {
                    vertex_index
                    for edge_index in face_edge_indices
                    for vertex_index in edge_vertex_indices[edge_index]
                }
            )
            for vertex_index in vertex_indices:
                delta = Gf.Vec3d(vertex_points[vertex_index]) - origin
                axial = Gf.Dot(delta, axis)
                radial = delta - axial * axis
                angle = math.atan2(Gf.Dot(radial, side), Gf.Dot(radial, ref))
                if angle < 0.0:
                    angle += period

                angle_in_range = any(
                    u_min - angle_tol <= angle + shift * period <= u_max + angle_tol for shift in (-1, 0, 1)
                )
                self.assertTrue(
                    angle_in_range and v_min - axial_tol <= axial <= v_max + axial_tol,
                    msg=(
                        f"cylinder face {face_index} boundary vertex {vertex_index} outside face:range "
                        f"for {brep_path.name}: angle={angle}, axial={axial}, range=[{uv_min}, {uv_max}], "
                        f"brep tolerance={brep_tolerance}"
                    ),
                )
            cylinder_index += 1

        self.assertEqual(
            cylinder_index,
            len(cylinder_origins),
            msg=f"cylinder face/geometry count mismatch for {brep_path.name}",
        )

    def _assert_valid(self, out_path: Path, brep_path: Path):
        """SMLib-level validation: re-import the BrepArray to an SmBrep and run AssertValid.

        Catches "clearly wrong" output (broken topology/geometry) that still satisfies the USD
        schema. Runs in a subprocess, so an importer crash surfaces as a non-zero return code
        rather than killing the test runner.
        """
        app = _usd_test_app_path()
        if not app.is_file():
            self.skipTest(f"usd_test_app not built at {app}")
        result = subprocess.run(
            [str(app), "--assert-valid", str(out_path)],
            capture_output=True,
            text=True,
            env=os.environ,
            timeout=self._timeout_sec,
        )
        if result.returncode != 0:
            detail = (result.stdout.strip() + " " + result.stderr.strip()).strip()
            self.fail(
                f"AssertValid round-trip failed for {brep_path.name} "
                f"(rc={result.returncode}): {detail}"
            )

    def _convert_and_validate(self, brep_path: Path):
        out_path = self._convert(brep_path)
        self._validate_schema(out_path, brep_path)
        self._assert_valid_ranges(out_path, brep_path)

    def _single_brep_prim(self, out_path: Path):
        stage = Usd.Stage.Open(str(out_path))
        self.assertIsNotNone(stage, f"Failed to open USD stage: {out_path}")
        brep_prims = [prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]
        self.assertEqual(len(brep_prims), 1, msg=f"expected one BrepArray in {out_path}")
        self._opened_stages.append(stage)
        return brep_prims[0]

    def _assert_single_quad_topology(self, prim, brep_path: Path):
        self.assertEqual(list(prim.GetAttribute("face:loopCount").Get()), [1], msg=brep_path.name)
        self.assertEqual(list(prim.GetAttribute("loop:edgeuseCount").Get()), [4], msg=brep_path.name)
        edge_indices = [int(value) for value in prim.GetAttribute("edgeuse:edgeIndex").Get()]
        self.assertEqual(len(edge_indices), 4, msg=brep_path.name)
        self.assertEqual(len(set(edge_indices)), 4, msg=brep_path.name)


class OcctBrepCorpusTestCase(_OcctBrepConvertMixin):
    """Self-authored, shareable corpus committed under TestFiles/occt_breps."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.inputs_dir = cls.repo_root / "TestFiles" / "occt_breps"

    def test_corpus(self):
        samples = self._discover()
        self.assertTrue(samples, msg=f"no .brep fixtures found in {self.inputs_dir}")
        for sample in samples:
            with self.subTest(sample=sample.name):
                try:
                    out_path = self._convert(sample)
                    self._validate_schema(out_path, sample)
                    self._assert_valid_ranges(out_path, sample)
                    # SMLib round-trip self-consistency check (committed corpus only).
                    self._assert_valid(out_path, sample)
                except unittest.SkipTest as exc:
                    _SUMMARY.record_skip("corpus", sample, exc)
                    raise
                except Exception as exc:
                    _SUMMARY.record_failure("corpus", sample, exc)
                    raise
                else:
                    _SUMMARY.record_success("corpus", sample)

    def test_source_maximum_tolerance(self):
        """The Brep-wide USD tolerance is the maximum tolerance stored in the OCCT topology."""
        brep_path = self.inputs_dir / "box.brep"
        prim = self._single_brep_prim(self._convert(brep_path))
        tolerances = prim.GetAttribute("brep:intersectTol3d").Get()
        self.assertEqual(len(tolerances), 1)
        self.assertAlmostEqual(float(tolerances[0]), 1.0e-7, delta=1.0e-15)

    def test_source_pcurves_are_preserved_when_parameterizations_match(self):
        """Compatible source pcurves remain exact UV NURBS records, including sparse sets."""
        for filename in ("box.brep", "bspline_surface.brep", "cylinder_patch.brep", "plate_with_hole.brep"):
            brep_path = self.inputs_dir / filename
            with self.subTest(sample=filename):
                prim = self._single_brep_prim(self._convert(brep_path))
                self.assertIn("BrepCurveUvNurbAPI", [str(schema) for schema in prim.GetAppliedSchemas()])
                edgeuse_count = len(prim.GetAttribute("edgeuse:edgeIndex").Get())
                orders = [int(value) for value in prim.GetAttribute("brep:curveUv:nurb:order").Get()]
                vertex_counts = [int(value) for value in prim.GetAttribute("brep:curveUv:nurb:vertexCount").Get()]
                self.assertEqual(len(orders), edgeuse_count)
                self.assertEqual(len(vertex_counts), edgeuse_count)
                self.assertTrue(all(order >= 2 for order in orders))
                self.assertTrue(all(count >= order for count, order in zip(vertex_counts, orders)))

        # Missing source pcurves are represented per edgeuse and do not discard compatible curves
        # from the same face or BRep.
        sparse_path = self.inputs_dir / "box_sparse_pcurves.brep"
        sparse_prim = self._single_brep_prim(self._convert(sparse_path))
        self.assertIn("BrepCurveUvNurbAPI", [str(schema) for schema in sparse_prim.GetAppliedSchemas()])
        sparse_orders = [int(value) for value in sparse_prim.GetAttribute("brep:curveUv:nurb:order").Get()]
        sparse_counts = [int(value) for value in sparse_prim.GetAttribute("brep:curveUv:nurb:vertexCount").Get()]
        self.assertEqual(len(sparse_orders), len(sparse_prim.GetAttribute("edgeuse:edgeIndex").Get()))
        self.assertEqual(len(sparse_counts), len(sparse_orders))
        self.assertEqual(sum(order == 0 and count == 0 for order, count in zip(sparse_orders, sparse_counts)), 1)
        self.assertTrue(
            all(
                (order == 0 and count == 0) or (order >= 2 and count >= order)
                for order, count in zip(sparse_orders, sparse_counts)
            )
        )

        # The first trim of this fixture is a degree-2 B-spline with natural domain [-1, 2], used
        # only over [0, 1]. Its exact extracted control polygon must not leak the basis interval.
        bspline_prim = self._single_brep_prim(self._convert(self.inputs_dir / "bspline_surface.brep"))
        self.assertEqual(list(bspline_prim.GetAttribute("brep:curveUv:nurb:order").Get()), [3, 2, 2, 2])
        bspline_cvs = bspline_prim.GetAttribute("brep:curveUv:nurb:controlVertices").Get()
        expected_cvs = ((0.0, 1.0), (0.0, 0.5), (0.0, 0.0))
        for actual, expected in zip(bspline_cvs[:3], expected_cvs):
            self.assertAlmostEqual(float(actual[0]), expected[0], places=12)
            self.assertAlmostEqual(float(actual[1]), expected[1], places=12)

        # The plate's circular hole is preserved as an exact rational quadratic, not a polyline.
        plate_prim = self._single_brep_prim(self._convert(self.inputs_dir / "plate_with_hole.brep"))
        self.assertIn(3, [int(value) for value in plate_prim.GetAttribute("brep:curveUv:nurb:order").Get()])
        weights = [float(value) for value in plate_prim.GetAttribute("brep:curveUv:nurb:weights").Get()]
        self.assertTrue(any(abs(weight - 1.0) > 1.0e-12 for weight in weights))

        # If no compatible pcurve survives, omit the applied schema entirely.
        for filename in ("cylinder_u_seam_no_pcurves.brep", "extrusion.brep", "plane_inconsistent_pcurves.brep"):
            brep_path = self.inputs_dir / filename
            with self.subTest(sample=filename):
                prim = self._single_brep_prim(self._convert(brep_path))
                self.assertNotIn("BrepCurveUvNurbAPI", [str(schema) for schema in prim.GetAppliedSchemas()])

    def test_periodic_seam_representations(self):
        """Partial U crossings relocate analytic seams without changing their explicit topology."""
        expected = {
            "cylinder_u_seam.brep": ("BrepSurfaceCylinderAPI", "cylinder", (0.0, 3.0)),
            "cylinder_u_seam_no_pcurves.brep": ("BrepSurfaceCylinderAPI", "cylinder", (0.0, 3.0)),
            "cone_u_seam.brep": ("BrepSurfaceConeAPI", "cone", (0.0, 2.0)),
            "sphere_u_seam.brep": (
                "BrepSurfaceSphereAPI",
                "sphere",
                (-math.pi / 6.0, math.pi / 6.0),
            ),
            "torus_u_seam.brep": ("BrepSurfaceTorusAPI", "torus", (0.0, math.pi / 2.0)),
        }
        expected_ref = (math.sqrt(0.5), -math.sqrt(0.5), 0.0)

        for filename, (surface_type, token, expected_v) in expected.items():
            brep_path = self.inputs_dir / filename
            with self.subTest(sample=filename):
                prim = self._single_brep_prim(self._convert(brep_path))
                self.assertEqual(list(prim.GetAttribute("face:surfaceType").Get()), [surface_type])
                uv_range = prim.GetAttribute("face:range").Get()
                self.assertEqual(len(uv_range), 2)
                self.assertAlmostEqual(float(uv_range[0][0]), 0.0, places=6)
                self.assertAlmostEqual(float(uv_range[1][0]), math.pi / 2.0, places=6)
                self.assertAlmostEqual(float(uv_range[0][1]), expected_v[0], places=6)
                self.assertAlmostEqual(float(uv_range[1][1]), expected_v[1], places=6)

                refs = prim.GetAttribute(f"brep:surface:{token}:refDirection").Get()
                self.assertEqual(len(refs), 1)
                for component, value in enumerate(expected_ref):
                    self.assertAlmostEqual(float(refs[0][component]), value, places=6)
                self._assert_single_quad_topology(prim, brep_path)

    def test_partial_sphere_with_full_uv_extent_preserves_explicit_trim(self):
        """A partial sphere spanning both poles and a full U period is not a natural whole sphere."""
        brep_path = self.inputs_dir / "sphere_partial_full_uv_extent.brep"
        prim = self._single_brep_prim(self._convert(brep_path))

        self.assertEqual(list(prim.GetAttribute("face:surfaceType").Get()), ["BrepSurfaceSphereAPI"])
        self.assertEqual(list(prim.GetAttribute("face:trimType").Get()), ["general"])
        self.assertEqual(list(prim.GetAttribute("face:loopCount").Get()), [1])

        edge_indices = [int(value) for value in prim.GetAttribute("edgeuse:edgeIndex").Get()]
        edge_counts = Counter(edge_indices)
        self.assertEqual(len(edge_indices), 5)
        self.assertEqual(sorted(edge_counts.values()), [1, 1, 1, 2])

        # The source pcurves meet only modulo the sphere period and at its singular poles. BA.763
        # requires literal UV endpoint equality, so omit this loop's pcurves and let SMLib rebuild
        # them instead of authoring an invalid USD loop.
        self.assertNotIn("BrepCurveUvNurbAPI", [str(schema) for schema in prim.GetAppliedSchemas()])

    def test_rectangular_trim_requires_all_four_face_range_sides(self):
        """Four isoparametric pcurves are not rectangular unless each bounds one side of face:range."""
        source_path = self.inputs_dir / "cylinder_patch.brep"
        source = source_path.read_text(encoding="utf-8")
        self.assertEqual(source.count("1 2.356194490192345 0 0 1"), 1)

        valid_prim = self._single_brep_prim(self._convert(source_path))
        self.assertEqual(list(valid_prim.GetAttribute("face:trimType").Get()), ["rectangular"])

        malformed_path = Path(self._tmp) / "cylinder_patch_interior_iso_side.brep"
        malformed_path.write_text(
            source.replace("1 2.356194490192345 0 0 1", "1 1.5707963267948966 0 0 1"),
            encoding="utf-8",
        )
        malformed_usd = malformed_path.with_suffix(".usda")
        result = subprocess.run(
            [str(self.tool), str(malformed_path), str(malformed_usd)],
            capture_output=True,
            text=True,
            env=os.environ,
            timeout=self._timeout_sec,
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr.strip())
        malformed_prim = self._single_brep_prim(malformed_usd)
        self.assertEqual(list(malformed_prim.GetAttribute("face:trimType").Get()), ["general"])

    def test_reversed_outer_shell_orientation_is_composed(self):
        """A reversed shell reference flips every faceuse sense relative to a forward outer shell."""
        source = (self.inputs_dir / "box.brep").read_text(encoding="utf-8")
        forward_marker = "\nSo\n\n1100000\n+2 0 *\n\n+1 0 "
        reversed_marker = "\nSo\n\n1100000\n-2 0 *\n\n+1 0 "
        self.assertEqual(source.count(forward_marker), 1)

        reversed_path = Path(self._tmp) / "box_reversed_outer_shell.brep"
        reversed_path.write_text(source.replace(forward_marker, reversed_marker), encoding="utf-8")
        reversed_usd = reversed_path.with_suffix(".usda")
        result = subprocess.run(
            [str(self.tool), str(reversed_path), str(reversed_usd)],
            capture_output=True,
            text=True,
            env=os.environ,
            timeout=self._timeout_sec,
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr.strip())

        forward_prim = self._single_brep_prim(self._convert(self.inputs_dir / "box.brep"))
        reversed_prim = self._single_brep_prim(reversed_usd)
        forward = [str(value) for value in forward_prim.GetAttribute("faceuse:orientationType").Get()]
        reversed_values = [str(value) for value in reversed_prim.GetAttribute("faceuse:orientationType").Get()]
        expected = ["same" if value == "opposite" else "opposite" for value in forward]
        self.assertEqual(reversed_values, expected)

    def test_torus_v_seam_lowers_to_unwrapped_nurb(self):
        """A torus V seam cannot move analytically, so lower only that face to an exact NURBS patch."""
        brep_path = self.inputs_dir / "torus_v_seam.brep"
        prim = self._single_brep_prim(self._convert(brep_path))
        self.assertEqual(list(prim.GetAttribute("face:surfaceType").Get()), ["BrepSurfaceNurbAPI"])

        uv_range = prim.GetAttribute("face:range").Get()
        self.assertAlmostEqual(float(uv_range[0][0]), 0.0, places=6)
        self.assertAlmostEqual(float(uv_range[1][0]), math.pi / 2.0, places=6)
        self.assertAlmostEqual(float(uv_range[0][1]), 7.0 * math.pi / 4.0, places=6)
        self.assertAlmostEqual(float(uv_range[1][1]), 9.0 * math.pi / 4.0, places=6)

        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:uOrder").Get()), [3])
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:vOrder").Get()), [3])
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:uVertexCount").Get()), [3])
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:vVertexCount").Get()), [3])
        self._assert_single_quad_topology(prim, brep_path)

    def test_multi_turn_cylinder_lowers_to_unwrapped_nurb(self):
        """A two-revolution strip must stay unwrapped instead of being clamped to one period."""
        brep_path = self.inputs_dir / "cylinder_two_turn.brep"
        prim = self._single_brep_prim(self._convert(brep_path))
        self.assertEqual(list(prim.GetAttribute("face:surfaceType").Get()), ["BrepSurfaceNurbAPI"])

        uv_range = prim.GetAttribute("face:range").Get()
        self.assertAlmostEqual(float(uv_range[0][0]), 0.0, places=6)
        self.assertAlmostEqual(float(uv_range[1][0]), 4.0 * math.pi, places=6)
        self.assertAlmostEqual(float(uv_range[0][1]), 0.0, places=6)
        self.assertAlmostEqual(float(uv_range[1][1]), 0.3, places=6)
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:uOrder").Get()), [3])
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:vOrder").Get()), [2])
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:uVertexCount").Get()), [17])
        self.assertEqual(list(prim.GetAttribute("brep:surface:nurb:vVertexCount").Get()), [2])

        curve_types = [str(value) for value in prim.GetAttribute("edge:curveType").Get()]
        self.assertEqual(curve_types, ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"])
        edge_ranges = prim.GetAttribute("edge:range").Get()
        self.assertAlmostEqual(float(edge_ranges[0]), 0.0, places=6)
        self.assertAlmostEqual(float(edge_ranges[1]), 4.0 * math.pi, places=6)
        self.assertAlmostEqual(float(edge_ranges[2]), 0.0, places=6)
        self.assertAlmostEqual(float(edge_ranges[3]), 4.0 * math.pi, places=6)
        self.assertEqual(list(prim.GetAttribute("face:loopCount").Get()), [1])
        self.assertEqual(list(prim.GetAttribute("loop:edgeuseCount").Get()), [2])
        self.assertEqual(list(prim.GetAttribute("edgeuse:edgeIndex").Get()), [0, 1])

    def test_genuine_full_period_faces_remain_analytic(self):
        """Natural one-period faces retain analytic surfaces and repeated seam-edge topology."""
        for filename, surface_type in (
            ("cylinder.brep", "BrepSurfaceCylinderAPI"),
            ("torus.brep", "BrepSurfaceTorusAPI"),
        ):
            brep_path = self.inputs_dir / filename
            with self.subTest(sample=filename):
                prim = self._single_brep_prim(self._convert(brep_path))
                surface_types = [str(value) for value in prim.GetAttribute("face:surfaceType").Get()]
                self.assertIn(surface_type, surface_types)
                self.assertNotIn("BrepSurfaceNurbAPI", surface_types)
                if filename == "torus.brep":
                    uv_range = prim.GetAttribute("face:range").Get()
                    self.assertAlmostEqual(float(uv_range[0][1]), 0.0, places=6)
                    self.assertAlmostEqual(float(uv_range[1][1]), 2.0 * math.pi, places=6)
                edge_indices = [int(value) for value in prim.GetAttribute("edgeuse:edgeIndex").Get()]
                self.assertLess(len(set(edge_indices)), len(edge_indices), msg=filename)

    def test_near_full_period_ranges_snap_from_both_sides(self):
        """Nominal periods slightly below or above 2*pi snap both endpoints to [0, 2*pi]."""
        source = (self.inputs_dir / "cylinder.brep").read_text(encoding="utf-8")
        period_literals = ("6.2831853071795862", "6.28318530717959")
        for literal in period_literals:
            self.assertIn(literal, source)

        period = 2.0 * math.pi
        for label, delta in (("under", -5.0e-7), ("over", 5.0e-7)):
            with self.subTest(side=label):
                modified = source
                replacement = repr(period + delta)
                for literal in period_literals:
                    modified = modified.replace(literal, replacement)

                brep_path = Path(self._tmp) / f"cylinder_near_period_{label}.brep"
                usd_path = brep_path.with_suffix(".usda")
                brep_path.write_text(modified, encoding="utf-8")
                result = subprocess.run(
                    [str(self.tool), str(brep_path), str(usd_path)],
                    capture_output=True,
                    text=True,
                    env=os.environ,
                    timeout=self._timeout_sec,
                )
                self.assertEqual(result.returncode, 0, msg=result.stderr.strip())
                self._validate_schema(usd_path, brep_path)

                prim = self._single_brep_prim(usd_path)
                face_types = [str(value) for value in prim.GetAttribute("face:surfaceType").Get()]
                face_ranges = prim.GetAttribute("face:range").Get()
                cylinder_face = face_types.index("BrepSurfaceCylinderAPI")
                self.assertAlmostEqual(float(face_ranges[2 * cylinder_face][0]), 0.0, places=14)
                self.assertAlmostEqual(float(face_ranges[2 * cylinder_face + 1][0]), period, places=14)

                edge_types = [str(value) for value in prim.GetAttribute("edge:curveType").Get()]
                edge_ranges = prim.GetAttribute("edge:range").Get()
                for edge_index, edge_type in enumerate(edge_types):
                    if edge_type == "BrepCurve3dCircleAPI":
                        self.assertAlmostEqual(float(edge_ranges[2 * edge_index]), 0.0, places=14)
                        self.assertAlmostEqual(float(edge_ranges[2 * edge_index + 1]), period, places=14)


class OcctBrepExternalTestCase(_OcctBrepConvertMixin):
    """Proprietary / third-party parts kept out of git; skips when none are present."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        override = os.environ.get("OCCT_BREP_EXTERNAL_DIR")
        cls.inputs_dir = Path(override) if override else (cls.repo_root / "TestFiles" / "occt_breps_external")

    def test_external_samples(self):
        samples = self._discover()
        if not samples:
            reason = (
                f"no external .brep samples in {self.inputs_dir} "
                "(populate it or set OCCT_BREP_EXTERNAL_DIR; see its README)"
            )
            _SUMMARY.record_suite_skip("external", reason)
            self.skipTest(reason)
        for sample in samples:
            with self.subTest(sample=sample.name):
                try:
                    result, out_path = self._run_convert(sample)
                    if self._is_non_brep_topology_error(result):
                        self.skipTest("no Solid/Shell/Face topology for BrepArray import")
                    self.assertEqual(result.returncode, 0, msg=f"occt_to_usd failed for {sample}: {result.stderr.strip()}")
                    self.assertTrue(out_path.is_file(), msg=f"no output produced for {sample}")
                    self._validate_schema(out_path, sample)
                    self._assert_valid_ranges(out_path, sample)
                except unittest.SkipTest as exc:
                    _SUMMARY.record_skip("external", sample, exc)
                    raise
                except Exception as exc:
                    _SUMMARY.record_failure("external", sample, exc)
                    raise
                else:
                    _SUMMARY.record_success("external", sample)


if __name__ == "__main__":
    unittest.main()
