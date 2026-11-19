# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import contextlib
import io
import os
import tempfile
import unittest
from pathlib import Path

if os.name == "nt":
    dll_directories = [
        os.add_dll_directory(os.path.abspath(p)) for p in os.environ["PATH"].split(os.pathsep) if os.path.isdir(p)
    ]

import _omni_solid as sm
from pxr import Plug, Sdf, Usd
from usd_brep import _validate_geometry_cli as validate_geometry_cli


class TestValidateGeometryFile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = Path(__file__).resolve().parents[3]
        Plug.Registry().RegisterPlugins(str(cls.repo / "_build/schema/omniSolid/resources"))
        cls.files = cls.repo / "TestFiles/usd_validator_TestFiles"

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)

    def test_valid_file(self):
        (result,) = sm.usd.validate_geometry_file(str(self.files / "CubeBrepArray.usda"))
        self.assertTrue(result.ok, result.message)
        self.assertEqual((result.prim_path, result.failures, result.message), ("/World/brepArray", [], ""))

    def test_invalid_brep_is_a_result_not_an_error(self):
        # Healing repairs this file's reversed edge range, so it is validated unhealed.
        (result,) = sm.usd.validate_geometry_file(str(self.files / "Test_BA_235_Invalid_edge_range_order.usda"), level=0)
        self.assertFalse(result.ok)
        self.assertTrue(result.failures)

    def test_instances_are_validated_once(self):
        stage = Usd.Stage.CreateNew(str(self.work / "instances.usda"))
        for name in ("B", "A"):
            prim = stage.DefinePrim("/" + name)
            prim.GetReferences().AddReference(str(self.files / "CubeBrepArray.usda"), Sdf.Path("/World"))
            prim.SetInstanceable(True)
        stage.GetRootLayer().Save()
        results = sm.usd.validate_geometry_file(str(self.work / "instances.usda"), level=0)
        self.assertEqual([r.prim_path for r in results], ["/A/brepArray"])

    def test_overridden_reference_is_validated_separately(self):
        stage = Usd.Stage.CreateNew(str(self.work / "override.usda"))
        for name in ("A", "B"):
            stage.DefinePrim("/" + name).GetReferences().AddReference(str(self.files / "CubeBrepArray.usda"), Sdf.Path("/World"))
        points = stage.GetAttributeAtPath("/A/brepArray.brep:vertexPoint:point:position")
        points.Set(type(points.Get())([(50.0, 50.0, 50.0)] + list(points.Get())[1:]))
        stage.GetRootLayer().Save()
        results = {r.prim_path: r.ok for r in sm.usd.validate_geometry_file(str(self.work / "override.usda"))}
        self.assertEqual(results, {"/A/brepArray": False, "/B/brepArray": True})

    def test_prim_without_geometry_is_reported(self):
        stage = Usd.Stage.CreateNew(str(self.work / "empty.usda"))
        stage.DefinePrim("/A").GetReferences().AddReference(str(self.files / "CubeBrepArray.usda"), Sdf.Path("/World"))
        stage.DefinePrim("/Empty", "BrepArray")
        stage.GetRootLayer().Save()
        results = {r.prim_path: r.ok for r in sm.usd.validate_geometry_file(str(self.work / "empty.usda"))}
        self.assertEqual(results, {"/A/brepArray": True, "/Empty": False})

    def test_instances_without_geometry_are_reported_once(self):
        proto = Sdf.Layer.CreateNew(str(self.work / "proto.usda"))
        proto.ImportFromString('#usda 1.0\ndef Xform "World"\n{\n    def BrepArray "b"\n    {\n    }\n}\n')
        proto.Save()
        stage = Usd.Stage.CreateNew(str(self.work / "empty_instances.usda"))
        for name in ("A", "B"):
            prim = stage.DefinePrim("/" + name)
            prim.GetReferences().AddReference(str(self.work / "proto.usda"), Sdf.Path("/World"))
            prim.SetInstanceable(True)
        stage.GetRootLayer().Save()
        results = sm.usd.validate_geometry_file(str(self.work / "empty_instances.usda"))
        self.assertEqual([(r.prim_path, r.ok) for r in results], [("/A/b", False)])

    def test_untyped_prim_with_region_count_is_reported(self):
        stage = Usd.Stage.CreateNew(str(self.work / "untyped.usda"))
        stage.DefinePrim("/A").GetReferences().AddReference(str(self.files / "CubeBrepArray.usda"), Sdf.Path("/World"))
        region_count = stage.DefinePrim("/X", "Xform").CreateAttribute("brep:regionCount", Sdf.ValueTypeNames.UIntArray)
        self.assertTrue(region_count.Set([1]))
        stage.GetRootLayer().Save()
        results = {r.prim_path: r.ok for r in sm.usd.validate_geometry_file(str(self.work / "untyped.usda"))}
        self.assertEqual(results, {"/A/brepArray": True, "/X": False})

    def test_inactive_and_abstract_prims_are_not_validated(self):
        stage = Usd.Stage.CreateNew(str(self.work / "inactive.usda"))
        for path in ("/A", "/Inactive"):
            stage.DefinePrim(path).GetReferences().AddReference(str(self.files / "CubeBrepArray.usda"), Sdf.Path("/World"))
        self.assertTrue(stage.GetPrimAtPath("/Inactive").SetActive(False))
        stage.CreateClassPrim("/_class").GetReferences().AddReference(str(self.files / "CubeBrepArray.usda"), Sdf.Path("/World"))
        stage.GetRootLayer().Save()
        results = [r.prim_path for r in sm.usd.validate_geometry_file(str(self.work / "inactive.usda"))]
        self.assertEqual(results, ["/A/brepArray"])

    def test_command_exit_status(self):
        cube = str(self.files / "CubeBrepArray.usda")
        invalid = str(self.files / "Test_BA_235_Invalid_edge_range_order.usda")
        for argv, status, text in (([cube], 0, "OK"), ([invalid], 1, "FAIL"), ([str(self.work / "missing.usda")], 2, "")):
            with self.subTest(argv=argv), contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(validate_geometry_cli.main(argv), status)
                self.assertIn(text, out.getvalue())

    def test_errors(self):
        stage = Usd.Stage.CreateNew(str(self.work / "none.usda"))
        stage.DefinePrim("/World", "Xform")
        stage.GetRootLayer().Save()
        cube = str(self.files / "CubeBrepArray.usda")
        for path, level in ((self.work / "missing.usda", 2), (self.work / "none.usda", 2), (cube, 3)):
            with self.subTest(path=path, level=level), self.assertRaises(RuntimeError):
                sm.usd.validate_geometry_file(str(path), level=level)


if __name__ == "__main__":
    unittest.main()
