# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the dev-only _smlib_dev Python bindings."""

from __future__ import annotations

import math
import os
import sys
import tempfile
import unittest

# Python 3.8+ on Windows does not search PATH for an extension module's dependent
# DLLs, so the USD libraries that _omni_solid links against (staged outside the
# module's own directory) have to be registered explicitly. Mirrors test_omni_solid.py.
if sys.platform == "win32":
    for p in os.environ.get("PATH", "").split(";"):
        if p and os.path.isdir(p):
            os.add_dll_directory(p)

import _omni_solid as sm

try:
    import _smlib_dev as smdev
except ImportError:
    smdev = None


def _gfx_output_code_enabled() -> bool:
    """True when this SMLib build can fill SmGfxArraySet from kernel Draw()."""
    if smdev is None or not smdev.tests.available():
        return True
    flags = smdev.tests.debug_runtime_info().get("compile_flags") or {}
    return bool(flags.get("SM_GFX_OUTPUT_CODE", True))


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
class TestSmlibDevSurface(unittest.TestCase):
    def test_stable_api_does_not_expose_dev_symbols(self):
        self.assertFalse(hasattr(sm, "extract_draw_batches"))
        self.assertFalse(hasattr(sm, "assert_valid"))
        self.assertFalse(hasattr(sm, "dump"))
        self.assertFalse(hasattr(sm, "Loopuse"))
        self.assertFalse(hasattr(sm, "Edgeuse"))
        self.assertFalse(hasattr(sm, "Loop"))
        self.assertFalse(hasattr(sm, "draw"))
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        self.assertFalse(hasattr(edge, "edgeuses"))
        self.assertFalse(hasattr(box, "topology_pick_ray"))

    def test_dev_imports_after_omni_solid(self):
        self.assertIsNotNone(smdev)
        self.assertTrue(hasattr(smdev, "topology_pick_ray"))
        self.assertTrue(hasattr(smdev, "assert_valid"))
        self.assertTrue(hasattr(smdev, "assert_valid_reports"))
        self.assertTrue(hasattr(smdev, "dump"))
        self.assertTrue(hasattr(smdev, "user_test"))
        self.assertTrue(hasattr(smdev, "user_test_example"))
        self.assertTrue(hasattr(smdev.draw, "extract_draw_batches"))


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
@unittest.skipUnless(
    _gfx_output_code_enabled(),
    "SMLib built without SM_GFX_OUTPUT_CODE",
)
class TestDrawExtraction(unittest.TestCase):
    def setUp(self):
        self.box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.face = self.box.faces()[0]
        self.edge = self.box.edges()[0]
        self.vertex = self.box.vertices()[0]
        self.edgeuse = smdev.edgeuses(self.edge)[0]
        self.loop = smdev.loops(self.edge)[0]
        self.loopuse = smdev.loopuses(self.edge)[0]
        self.curve = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.surface = self.face.surface()

    def _assert_batches(self, obj, object_type: str, variant: str = "default"):
        batches = smdev.draw.extract_draw_batches(obj, variant=variant)
        self.assertIsInstance(batches, list)
        self.assertGreater(len(batches), 0, object_type)
        for batch in batches:
            self.assertIn(batch["primitive"], {"point", "line", "simple_mesh", "index_mesh"})
            self.assertIn("positions", batch)
            self.assertIsInstance(batch["positions"], list)
            self.assertIn("color", batch)
            self.assertEqual(len(batch["color"]), 3)
            self.assertIn("point_size", batch)
            self.assertIn("line_width", batch)
            self.assertIn("stipple", batch)
            self.assertIn("dashed", batch)
            self.assertEqual(batch["metadata"]["object_type"], object_type)
            self.assertEqual(batch["metadata"]["draw_variant"], variant)
        return batches

    def test_extract_draw_batches_for_core_objects(self):
        cases = [
            (self.box, "Brep"),
            (self.face, "Face"),
            (self.edge, "Edge"),
            (self.vertex, "Vertex"),
            (self.edgeuse, "Edgeuse"),
            (self.loop, "Loop"),
            (self.loopuse, "Loopuse"),
            (self.curve, "Curve"),
            (self.surface, "Surface"),
        ]
        for obj, object_type in cases:
            with self.subTest(object_type=object_type):
                self._assert_batches(obj, object_type)

    def test_draw_uv_variant_for_face_and_surface(self):
        face_batches = self._assert_batches(self.face, "Face", variant="draw_uv")
        surface_batches = self._assert_batches(self.surface, "Surface", variant="draw_uv")
        self.assertIn("line", {batch["primitive"] for batch in face_batches})
        self.assertIn("line", {batch["primitive"] for batch in surface_batches})

    def test_unsupported_draw_variant_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "variants|default"):
            smdev.draw.extract_draw_batches(self.face, variant="facets_smlib")
        with self.assertRaisesRegex(ValueError, "default"):
            smdev.draw.extract_draw_batches(self.edge, variant="draw_uv")

    def test_flat_and_draw_submodule_alias(self):
        flat = smdev.extract_draw_batches(self.edge)[0]["metadata"]["object_type"]
        sub = smdev.draw.extract_draw_batches(self.edge)[0]["metadata"]["object_type"]
        self.assertEqual(flat, "Edge")
        self.assertEqual(sub, "Edge")


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
@unittest.skipIf(
    _gfx_output_code_enabled(),
    "SM_GFX_OUTPUT_CODE is enabled",
)
class TestDrawExtractionUnavailable(unittest.TestCase):
    def test_extract_draw_batches_raises_without_gfx_output(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with self.assertRaisesRegex(RuntimeError, "SM_GFX_OUTPUT_CODE"):
            smdev.draw.extract_draw_batches(box)
        with self.assertRaisesRegex(RuntimeError, "SM_GFX_OUTPUT_CODE"):
            smdev.extract_draw_batches(box)


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
class TestDevTopology(unittest.TestCase):
    @staticmethod
    def _point_almost_equal(a, b, places: int = 3) -> None:
        for i in range(3):
            unittest.TestCase().assertAlmostEqual(a[i], b[i], places=places)

    def test_topology_pick_ray_returns_face_hits(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        hits = smdev.topology_pick_ray(box, (5, 5, 30), (0, 0, -1), 0.05)
        face_hits = [hit for hit in hits if hit["kind"] == "Face"]
        self.assertGreaterEqual(len(face_hits), 2)

    def test_poly_faces_retain_original_brep_face_indices(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        mesh = sm.tessellate(box, chord_height_tolerance=0.1, curve_angle_tolerance_deg=0.0)
        indices = [smdev.poly_face_original_face_index(face) for face in mesh.get_faces()]
        self.assertEqual(set(indices), set(range(box.face_count())))
        self.assertNotIn(-1, indices)

    def test_face_loops_and_edgeuse_accessors(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        face = box.faces()[0]
        loops = smdev.face_loops(face)
        self.assertEqual(len(loops), 1)
        loop = loops[0]
        self.assertEqual(len(loop.edgeuses()), 4)
        self.assertIsInstance(loop.loopuse(), smdev.Loopuse)
        uv = loop.edgeuses()[0].uv_point_at_normalized(0.5)
        self.assertEqual(len(uv), 2)
        self.assertTrue(all(math.isfinite(value) for value in uv))

    def test_edge_topology_free_functions(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        faces = edge.faces()
        edgeuses = smdev.edgeuses(edge)
        self.assertGreaterEqual(len(edgeuses), len(faces))
        self.assertTrue(all(isinstance(edgeuse, smdev.Edgeuse) for edgeuse in edgeuses))
        for face in faces:
            self.assertIn(smdev.edgeuse_of_face(edge, face), edgeuses)
        loopuses = smdev.loopuses(edge)
        self.assertGreaterEqual(len(loopuses), 1)
        self.assertTrue(all(isinstance(loopuse, smdev.Loopuse) for loopuse in loopuses))

    def test_edgeuse_point_at_normalized_endpoints(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        edge = box.edges()[0]
        edgeuse = smdev.edgeuse_of_face(edge, edge.faces()[0])
        p0 = edgeuse.point_at_normalized(0.0)
        p1 = edgeuse.point_at_normalized(1.0)
        start = edge.start_vertex().point()
        end = edge.end_vertex().point()
        same_direction = all(abs(a - b) < 1e-3 for a, b in zip(p0, start)) and all(
            abs(a - b) < 1e-3 for a, b in zip(p1, end)
        )
        opposite_direction = all(abs(a - b) < 1e-3 for a, b in zip(p0, end)) and all(
            abs(a - b) < 1e-3 for a, b in zip(p1, start)
        )
        self.assertTrue(same_direction or opposite_direction)


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
class TestAssertValid(unittest.TestCase):
    def test_assert_valid_passes_on_box_and_topology(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        self.assertTrue(smdev.assert_valid(box))
        self.assertTrue(smdev.assert_valid(box, level=0, walk=False))
        self.assertTrue(smdev.assert_valid(box.faces()[0]))
        self.assertTrue(smdev.assert_valid(box.edges()[0]))
        self.assertTrue(smdev.assert_valid(box.vertices()[0]))
        self.assertTrue(smdev.assert_valid(smdev.edgeuses(box.edges()[0])[0]))
        self.assertTrue(smdev.assert_valid(smdev.loops(box.edges()[0])[0]))
        self.assertTrue(smdev.assert_valid(smdev.loopuses(box.edges()[0])[0]))
        self.assertTrue(smdev.assert_valid(box.faces()[0].surface()))
        line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
        self.assertTrue(smdev.assert_valid(line))

    def test_assert_valid_passes_after_reflected_scale(self):
        # An odd number of negative factors reverses handedness and rewires edgeuses and
        # faceuse shell ownership; walk the full topology to check those links directly.
        for scale in ((-1, 1, 1), (-1, -1, -1), (-2, 3, 4)):
            with self.subTest(scale=scale):
                box = sm.create_box((-4, -3, -2), 8, 6, 4)
                sm.scale(box, scale)
                ok, reports = smdev.assert_valid_reports(box, level=2, walk=True)
                self.assertTrue(ok, reports)

    def test_assert_valid_reports_returns_empty_list_when_passing(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        ok, reports = smdev.assert_valid_reports(box)
        self.assertTrue(ok)
        self.assertEqual(reports, [])
        ok, reports = smdev.assert_valid_reports(box.faces()[0], level=0, walk=False)
        self.assertTrue(ok)
        self.assertIsInstance(reports, list)

    def test_assert_valid_reports_failures_in_every_build_config(self):
        # A zero-thickness box builds in the kernel, but its infinite region is
        # wrong. The old sm_AssertValid binding reported this only in debug
        # SMLib. create_box rejects the zero height, so build it kernel-direct.
        flat = smdev.kernel_create_box((0, 0, 0), 10, 10, 0.0)
        self.assertFalse(smdev.assert_valid(flat))
        ok, reports = smdev.assert_valid_reports(flat)
        self.assertFalse(ok)
        self.assertTrue(reports)
        self.assertIn("FAILED", reports[0])
        self.assertIn("SmBrep", reports[0])

    def test_assert_valid_rejects_bad_inputs(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        with self.assertRaises(TypeError):
            smdev.assert_valid(None)
        with self.assertRaises(TypeError):
            smdev.assert_valid(12)
        with self.assertRaises(ValueError):
            smdev.assert_valid(box, level=9)


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
class TestDump(unittest.TestCase):
    @staticmethod
    def _capture_c_output(func) -> str:
        tmp = tempfile.TemporaryFile()
        saved_out = os.dup(1)
        saved_err = os.dup(2)
        try:
            os.dup2(tmp.fileno(), 1)
            os.dup2(tmp.fileno(), 2)
            func()
        finally:
            os.dup2(saved_out, 1)
            os.dup2(saved_err, 2)
            os.close(saved_out)
            os.close(saved_err)
        tmp.seek(0)
        try:
            return tmp.read().decode("utf-8", "replace")
        finally:
            tmp.close()

    def test_dump_writes_kernel_text_for_box_and_topology(self):
        box = sm.create_box((0, 0, 0), 10, 10, 10)
        brep_text = self._capture_c_output(lambda: smdev.dump(box))
        face_text = self._capture_c_output(lambda: smdev.dump(box.faces()[0]))
        edge_text = self._capture_c_output(lambda: smdev.dump(box.edges()[0]))
        self._capture_c_output(lambda: smdev.dump(box.vertices()[0]))
        self._capture_c_output(lambda: smdev.dump(smdev.edgeuses(box.edges()[0])[0]))
        self._capture_c_output(lambda: smdev.dump(smdev.loops(box.edges()[0])[0]))
        self._capture_c_output(lambda: smdev.dump(smdev.loopuses(box.edges()[0])[0]))
        self._capture_c_output(lambda: smdev.dump(box.faces()[0].surface()))
        self._capture_c_output(lambda: smdev.dump(sm.create_line_segment((0, 0, 0), (10, 0, 0))))

        if not brep_text.strip():
            # Release SMLib (and Windows debug) intentionally do not echo Dump().
            return

        self.assertIn("MANIFOLD Brep", brep_text)
        self.assertIn("Faces =", brep_text)
        self.assertIn("Edges =", brep_text)
        self.assertIn("Begin SmFace::Dump()", face_text)
        self.assertIn("Begin SmEdge::Dump()", edge_text)

    def test_dump_rejects_bad_inputs(self):
        with self.assertRaises(TypeError):
            smdev.dump(None)
        with self.assertRaises(TypeError):
            smdev.dump(12)


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
class TestDevUserTests(unittest.TestCase):
    def test_user_test_example_matches_python_test_2(self):
        box = smdev.user_test_example()
        self.assertEqual(box.face_count(), 6)
        self.assertEqual(box.edge_count(), 12)
        self.assertEqual(box.vertex_count(), 8)
        self.assertTrue(smdev.assert_valid(box))

    def test_user_test_stub_returns_none(self):
        self.assertIsNone(smdev.user_test())


@unittest.skipIf(smdev is None, "_smlib_dev dev module is not built")
class TestDevTestsFacade(unittest.TestCase):
    def test_tests_submodule_reports_availability(self):
        self.assertTrue(hasattr(smdev.tests, "available"))
        self.assertTrue(hasattr(smdev.tests, "list_prog_test_suites"))


if __name__ == "__main__":
    unittest.main()
