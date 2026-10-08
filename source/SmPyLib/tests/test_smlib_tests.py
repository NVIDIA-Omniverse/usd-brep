# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the dev-only _smlib_tests Python bindings."""

from __future__ import annotations

import os
import unittest

try:
    import _smlib_tests
except ImportError:
    _smlib_tests = None


EXPECTED_PROG_TEST_SUITES = (
    "topology",
    "booleans",
    "offset",
    "fillets",
    "local-ops",
    "sweeps",
    "primitives",
    "tessellation",
    "unit",
    "trimmed-surfaces",
    "brep-import",
    "ssi-analytic",
    "ssi-advanced",
    "cci-advanced",
    "section-advanced",
    "silhouette-advanced",
    "stitch",
)

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
PROG_TEST_WORKING_DIRECTORY = os.path.join(REPO, "tests", "prog_test")


@unittest.skipIf(_smlib_tests is None, "_smlib_tests dev module is not built")
class TestSmlibTestsBinding(unittest.TestCase):
    def test_list_prog_test_suites(self):
        self.assertEqual(tuple(_smlib_tests.list_prog_test_suites()), EXPECTED_PROG_TEST_SUITES)

    def test_run_unknown_suite_rejects_cleanly(self):
        with self.assertRaises(ValueError):
            _smlib_tests.run_prog_test_suite(
                "__missing_suite__",
                working_directory=PROG_TEST_WORKING_DIRECTORY,
                do_graphics=False,
            )

    def test_debug_runtime_info_returns_loaded_library_paths_and_flags(self):
        info = _smlib_tests.debug_runtime_info()

        self.assertIn("compile_flags", info)
        self.assertIn("libraries", info)
        self.assertIn("do_graphics", info)
        self.assertIsInstance(info["compile_flags"], dict)
        self.assertIsInstance(info["libraries"], dict)
        self.assertIsInstance(info["do_graphics"], bool)
        for flag in ("SM_DEBUG_CODE", "SM_GFX_CODE", "SM_GFX_OUTPUT_CODE", "SM_GRAPHICS_CALLBACKS"):
            self.assertIn(flag, info["compile_flags"])
            self.assertIsInstance(info["compile_flags"][flag], bool)
        for library in ("prog_test", "smlib"):
            self.assertIn(library, info["libraries"])
            self.assertIsInstance(info["libraries"][library].get("path", ""), str)

    def test_run_suite_returns_structured_result(self):
        result = _smlib_tests.run_prog_test_suite(
            "stitch",
            working_directory=PROG_TEST_WORKING_DIRECTORY,
            do_graphics=False,
        )

        expected_keys = {
            "suite_name",
            "status",
            "status_name",
            "ok",
            "elapsed_seconds",
            "log",
            "draw_events",
        }
        self.assertEqual(set(result), expected_keys)
        self.assertEqual(result["suite_name"], "stitch")
        self.assertIsInstance(result["status"], int)
        self.assertIsInstance(result["status_name"], str)
        self.assertIsInstance(result["ok"], bool)
        self.assertIsInstance(result["elapsed_seconds"], float)
        self.assertIsInstance(result["log"], str)
        self.assertIsInstance(result["draw_events"], list)


if __name__ == "__main__":
    unittest.main()
