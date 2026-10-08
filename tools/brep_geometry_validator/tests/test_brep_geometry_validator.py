# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from pxr import Sdf, Usd


class TestBrepGeometryValidator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = os.environ["BREP_GEOMETRY_VALIDATOR"]
        cls.repo = Path(__file__).resolve().parents[3]
        cls.cube = cls.repo / "TestFiles/brep_validator/CubeBrepArray.usda"

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.work = Path(self.temp_dir.name)

    def tearDown(self):
        self.temp_dir.cleanup()

    def run_validator(self, path, *extra_args, timeout=30):
        return subprocess.run(
            [self.binary, "--level", "0", "--no-healer", "--quiet", *extra_args, str(path)],
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )

    def add_cube_reference(self, stage, path, instanceable=True, active=True):
        prim = stage.DefinePrim(path)
        prim.GetReferences().AddReference(str(self.cube), Sdf.Path("/World/brepArray"))
        prim.SetInstanceable(instanceable)
        prim.SetActive(active)
        return prim

    def assert_brep_count(self, result, expected):
        self.assertEqual(result.returncode, 0, result.stderr)
        match = re.search(r"BREP_GEOMETRY .* breps=(\d+)", result.stdout)
        self.assertIsNotNone(match, result.stdout)
        self.assertEqual(int(match.group(1)), expected)

    def test_instance_root_imports_components(self):
        stage = Usd.Stage.CreateNew(str(self.work / "instance_root.usda"))
        self.add_cube_reference(stage, "/Root")
        stage.GetRootLayer().Save()
        self.assert_brep_count(self.run_validator(stage.GetRootLayer().realPath), 2)

    def test_shared_nested_and_inactive_instances(self):
        shared_model = Usd.Stage.CreateNew(str(self.work / "shared_model.usda"))
        shared_model.DefinePrim("/Model", "Xform")
        for index in range(3):
            self.add_cube_reference(shared_model, f"/Model/Part{index}", instanceable=False)
        shared_model.GetRootLayer().Save()

        shared = Usd.Stage.CreateNew(str(self.work / "shared.usda"))
        for index in range(100):
            prim = shared.DefinePrim(f"/Instance{index:03d}")
            prim.GetReferences().AddReference(shared_model.GetRootLayer().realPath, Sdf.Path("/Model"))
            prim.SetInstanceable(True)
        shared.GetRootLayer().Save()
        self.assert_brep_count(self.run_validator(shared.GetRootLayer().realPath), 6)

        nested_model = Usd.Stage.CreateNew(str(self.work / "nested_model.usda"))
        nested_model.DefinePrim("/Model", "Xform")
        self.add_cube_reference(nested_model, "/Model/Part")
        nested_model.GetRootLayer().Save()
        nested = Usd.Stage.CreateNew(str(self.work / "nested.usda"))
        outer = nested.DefinePrim("/Assembly")
        outer.GetReferences().AddReference(nested_model.GetRootLayer().realPath, Sdf.Path("/Model"))
        outer.SetInstanceable(True)
        nested.GetRootLayer().Save()
        self.assert_brep_count(self.run_validator(nested.GetRootLayer().realPath), 2)

        inactive_model = Usd.Stage.CreateNew(str(self.work / "inactive_model.usda"))
        inactive_model.DefinePrim("/Model", "Xform")
        self.add_cube_reference(inactive_model, "/Model/Active", instanceable=False)
        self.add_cube_reference(inactive_model, "/Model/Inactive", active=False)
        inactive_model.GetRootLayer().Save()
        inactive = Usd.Stage.CreateNew(str(self.work / "inactive.usda"))
        active_outer = inactive.DefinePrim("/Active")
        active_outer.GetReferences().AddReference(inactive_model.GetRootLayer().realPath, Sdf.Path("/Model"))
        active_outer.SetInstanceable(True)
        inactive_outer = inactive.DefinePrim("/Inactive")
        inactive_outer.GetReferences().AddReference(inactive_model.GetRootLayer().realPath, Sdf.Path("/Model"))
        inactive_outer.SetInstanceable(True)
        inactive_outer.SetActive(False)
        inactive.GetRootLayer().Save()
        self.assert_brep_count(self.run_validator(inactive.GetRootLayer().realPath), 2)

    def test_mixed_component_import_is_hard_error(self):
        output = self.work / "mixed.usda"
        source = Usd.Stage.Open(str(self.cube))
        # Flatten: the cube references ../usd_TestFiles/Materials.usda, which a temp copy cannot reach.
        source.Flatten().Export(str(output))
        stage = Usd.Stage.Open(str(output))
        order = stage.GetPrimAtPath("/World/brepArray").GetAttribute(
            "brep:edge3dNurb:curve3d:nurb:order"
        )
        values = list(order.Get())
        values[12] = 0
        order.Set(values)
        stage.GetRootLayer().Save()

        result = self.run_validator(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("import failed", result.stderr)
        self.assertRegex(result.stdout, r"breps=0 invalid=0")
        self.assertIn("(with hard errors)", result.stdout)

    def make_empty_model(self, path):
        stage = Usd.Stage.CreateNew(str(path))
        stage.DefinePrim("/Model", "Xform")
        stage.DefinePrim("/Model/Z", "BrepArray")
        stage.DefinePrim("/Model/A", "BrepArray")
        stage.GetRootLayer().Save()
        return stage

    def make_ordering_stage(self, path, model, add_decoy):
        stage = Usd.Stage.CreateNew(str(path))
        if add_decoy:
            decoy_model = Usd.Stage.CreateNew(str(self.work / "decoy.usda"))
            decoy_model.DefinePrim("/Model", "Xform")
            decoy_model.DefinePrim("/Model/Child", "Xform")
            decoy_model.GetRootLayer().Save()
            decoy = stage.DefinePrim("/A_Decoy")
            decoy.GetReferences().AddReference(decoy_model.GetRootLayer().realPath, Sdf.Path("/Model"))
            decoy.SetInstanceable(True)
        for name in (("Z", "M") if not add_decoy else ("M", "Z")):
            prim = stage.DefinePrim(f"/{name}")
            prim.GetReferences().AddReference(model.GetRootLayer().realPath, Sdf.Path("/Model"))
            prim.SetInstanceable(True)
        stage.GetRootLayer().Save()
        return stage

    def test_stable_paths_ignore_generated_prototype_order(self):
        model = self.make_empty_model(self.work / "empty_model.usda")
        first = self.make_ordering_stage(self.work / "first.usda", model, False)
        second = self.make_ordering_stage(self.work / "second.usda", model, True)

        def candidate_prototype(stage):
            return next(
                str(prototype.GetPath())
                for prototype in stage.GetPrototypes()
                if any(prim.GetTypeName() == "BrepArray" for prim in Usd.PrimRange(prototype))
            )

        self.assertNotEqual(candidate_prototype(first), candidate_prototype(second))

        def failed_paths(result):
            return [
                line.rsplit(" ", 1)[-1]
                for line in result.stderr.splitlines()
                if "import failed" in line or "import produced missing Breps" in line
            ]

        expected = ["/M/A", "/M/Z"]
        self.assertEqual(failed_paths(self.run_validator(first.GetRootLayer().realPath)), expected)
        self.assertEqual(failed_paths(self.run_validator(second.GetRootLayer().realPath)), expected)

    def test_discovery_does_not_expand_instance_proxies(self):
        model = Usd.Stage.CreateNew(str(self.work / "large_model.usda"))
        model.DefinePrim("/Model", "Xform")
        for index in range(100):
            model.DefinePrim(f"/Model/Child{index:03d}", "Xform")
        model.GetRootLayer().Save()

        assembly = Usd.Stage.CreateNew(str(self.work / "large_assembly.usda"))
        for index in range(100):
            prim = assembly.DefinePrim(f"/Instance{index:03d}")
            prim.GetReferences().AddReference(model.GetRootLayer().realPath, Sdf.Path("/Model"))
            prim.SetInstanceable(True)
        assembly.GetRootLayer().Save()

        result = self.run_validator(assembly.GetRootLayer().realPath, "--discovery-stats")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("no BrepArray prims found", result.stderr)
        self.assertRegex(result.stdout, r"BREP_GEOMETRY_DISCOVERY .* visited=201")


if __name__ == "__main__":
    unittest.main()
