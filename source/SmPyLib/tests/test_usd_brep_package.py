#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Qualification for the `usd_brep` package that wraps the compiled module.

`usd_brep` is the supported import. These tests cover what the package adds on
top of `_omni_solid`: the re-export, the submodule attributes, and the
import-time environment setup.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

# In a build tree OpenUSD comes from target-deps, and Windows ignores PATH when
# resolving an extension's imports. Add those directories only -- adding all of
# PATH picks up any other OpenUSD installed on the machine.
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    _config = os.environ.get("BREP_TEST_CONFIG", "release")
    _usd = Path(__file__).resolve().parents[3] / "_build" / "target-deps" / "usd" / _config
    for _dir in (_usd / "lib", _usd / "bin"):
        if _dir.is_dir():
            os.add_dll_directory(str(_dir))

import usd_brep  # noqa: E402

# Packaged inside usd_brep in a wheel, beside it in the build tree and packman.
_CORE = sys.modules.get("usd_brep._omni_solid") or sys.modules["_omni_solid"]


class TestUsdBrepPackage(unittest.TestCase):

    def test_reexports_the_compiled_module(self):
        """Names bound by the extension are reachable through the package."""
        self.assertIs(usd_brep.Brep, _CORE.Brep)
        self.assertIs(usd_brep.primitives.create_sphere, _CORE.primitives.create_sphere)

    def test_submodules_are_attributes(self):
        """pybind submodules are attributes, not importable modules."""
        for name in ("primitives", "curves", "surfaces", "booleans", "sweeps", "usd"):
            self.assertTrue(hasattr(usd_brep, name), f"missing submodule attribute: {name}")

    def test_submodules_are_importable(self):
        """The vendored `usd` resource directory must not shadow the submodule."""
        import usd_brep.usd

        self.assertIs(usd_brep.usd, sys.modules["usd_brep.usd"])
        self.assertTrue(hasattr(usd_brep.usd, "export_brep"))

    def test_all_covers_the_public_surface(self):
        """__all__ is derived from the extension and excludes private names."""
        self.assertIn("primitives", usd_brep.__all__)
        self.assertIn("Brep", usd_brep.__all__)
        self.assertFalse([n for n in usd_brep.__all__ if n.startswith("_")])

    def test_sets_the_schema_plugin_path(self):
        """A shipped schema is pointed at by the package, not just by the caller."""
        here = Path(usd_brep.__file__).parent
        # Beside the package in a wheel, at the package root in the packman layout.
        shipped = [p / "omniSolid" / "resources" for p in (here, here.parent.parent)]
        found = [p for p in shipped if p.is_dir()]
        if found:
            self.assertEqual(Path(os.environ["OMNISOLID_PLUGIN_PATH"]), found[0])
        else:
            # Build tree: the schema is outside the package, so the caller supplies
            # the path -- tests.sh does.
            self.assertTrue(os.environ.get("OMNISOLID_PLUGIN_PATH"))

    def test_modelling_through_the_package(self):
        """A caller can do real work using only the package name."""
        sphere = usd_brep.primitives.create_sphere((0.0, 0.0, 0.0), 1.0)
        self.assertEqual(sphere.face_count(), 1)

    def test_usd_round_trip(self):
        """The schema resolves far enough to write and read back a BrepArray."""
        usd_brep.usd.ensure_plugin_registered()
        sphere = usd_brep.primitives.create_sphere((0.0, 0.0, 0.0), 1.0)
        with tempfile.TemporaryDirectory() as tmp:
            stage = str(Path(tmp) / "sphere.usda")
            usd_brep.usd.export_brep(sphere, stage)
            self.assertEqual(len(usd_brep.usd.import_breps(stage)), 1)


if __name__ == "__main__":
    unittest.main()
