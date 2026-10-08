# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path

if os.name == "nt":
    dll_directories = [
        os.add_dll_directory(os.path.abspath(p)) for p in os.environ["PATH"].split(os.pathsep) if os.path.isdir(p)
    ]

import _omni_solid as sm
from pxr import Plug, Sdf, Usd, UsdGeom, UsdShade, UsdUtils


class TestUsdBrepHeal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = Path(__file__).resolve().parents[3]
        Plug.Registry().RegisterPlugins(str(cls.repo / "_build/schema/omniSolid/resources"))
        cls.cube = cls.repo / "TestFiles/brep_validator/CubeBrepArray.usda"

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)

    def run_heal(self, source, output, threads=0):
        try:
            return SimpleNamespace(returncode=0, results=sm.usd.heal_file(str(source), str(output), threads=threads), message="")
        except RuntimeError as error:
            return SimpleNamespace(returncode=1, results=None, message=str(error))

    def test_references_instances_transforms_and_materials(self):
        model = Usd.Stage.CreateNew(str(self.work / "model.usda"))
        model.DefinePrim("/Model", "Xform")
        prim = model.DefinePrim("/Model/Part")
        self.assertTrue(prim.GetReferences().AddReference(str(self.cube), "/World"))
        mat = UsdShade.Material.Define(model, "/Model/Looks/Red")
        self.assertTrue(UsdShade.MaterialBindingAPI.Apply(model.GetPrimAtPath("/Model/Part/brepArray")).Bind(mat))
        self.assertTrue(model.GetRootLayer().Save())
        source = Usd.Stage.CreateNew(str(self.work / "input.usda"))
        self.assertTrue(UsdGeom.SetStageUpAxis(source, UsdGeom.Tokens.z))
        self.assertTrue(UsdGeom.SetStageMetersPerUnit(source, 0.001))
        source.SetDefaultPrim(source.DefinePrim("/World", "Xform"))
        for name, x in [("A", 0), ("B", 30)]:
            instance = source.DefinePrim("/World/" + name, "Xform")
            self.assertTrue(instance.GetReferences().AddReference(str(self.work / "model.usda"), "/Model"))
            self.assertTrue(instance.SetInstanceable(True))
            self.assertTrue(UsdGeom.Xformable(instance).AddTranslateOp().Set((x, 2, 3)))
        self.assertTrue(source.GetRootLayer().Save())
        output = self.work / "elsewhere" / "healed.usdc"
        output.parent.mkdir()
        result = self.run_heal(self.work / "input.usda", output, threads=2)
        self.assertEqual(result.returncode, 0, result.message)
        self.assertEqual([r.brep_count for r in result.results], [2])
        moved = self.work / "moved"
        output.parent.rename(moved)
        output = moved / output.name
        healed = Usd.Stage.Open(str(output))
        self.assertTrue(healed.GetPrimAtPath("/World/A").HasAuthoredReferences())
        for layer in healed.GetUsedLayers():
            if not layer.anonymous and layer != healed.GetRootLayer():
                self.assertEqual(Path(layer.realPath).parent, output.parent / "healed_layers")
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(healed), 0.001)
        self.assertEqual(UsdGeom.GetStageUpAxis(healed), UsdGeom.Tokens.z)
        self.assertEqual(str(healed.GetDefaultPrim().GetPath()), "/World")
        for name in ("A", "B"):
            path = "/World/" + name
            a, b = source.GetPrimAtPath(path), healed.GetPrimAtPath(path)
            self.assertTrue(b.IsInstance())
            self.assertEqual(
                UsdGeom.Xformable(a).ComputeLocalToWorldTransform(0),
                UsdGeom.Xformable(b).ComputeLocalToWorldTransform(0),
            )
            part = healed.GetPrimAtPath(path + "/Part/brepArray")
            self.assertEqual(part.GetTypeName(), "BrepArray")
            self.assertEqual(part.GetCustomDataByKey("source"), "SMLib")
            self.assertTrue(UsdShade.MaterialBindingAPI(part).GetDirectBindingRel().GetTargets())
        self.assertEqual(
            healed.GetPrimAtPath("/World/A").GetPrototype(), healed.GetPrimAtPath("/World/B").GetPrototype()
        )
        # A second heal re-imports the written geometry and exercises the serialized schema.
        again = self.run_heal(output, self.work / "again.usdc", threads=1)
        self.assertEqual(again.returncode, 0, again.message)

    def test_internal_references_heal_definition_once(self):
        source = Usd.Stage.Open(str(self.cube))
        layer = Sdf.Layer.CreateAnonymous()
        layer.TransferContent(source.GetRootLayer())
        UsdUtils.ModifyAssetPaths(layer, source.GetRootLayer().ComputeAbsolutePath)
        stage = Usd.Stage.Open(layer)
        for name in ("A", "B"):
            instance = stage.DefinePrim("/" + name, "Xform")
            self.assertTrue(instance.GetReferences().AddInternalReference("/World"))
            self.assertTrue(instance.SetInstanceable(True))
        path = self.work / "internal.usda"
        self.assertTrue(layer.Export(str(path)))
        original = path.read_bytes()
        output = self.work / "healed.usdc"
        result = self.run_heal(path, output)
        self.assertEqual(result.returncode, 0, result.message)
        self.assertEqual([r.brep_count for r in result.results], [2])
        healed = Usd.Stage.Open(str(output))
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(healed.GetPrimAtPath("/A").GetPrototype(), healed.GetPrimAtPath("/B").GetPrototype())
        self.assertEqual(
            healed.GetPrimAtPath("/A").GetMetadata("references").GetAddedOrExplicitItems()[0].primPath,
            Sdf.Path("/World"),
        )
        self.assertEqual(healed.GetPrimAtPath("/A/brepArray").GetCustomDataByKey("source"), "SMLib")
        self.assertEqual(len([p for p in healed.TraverseAll() if p.GetTypeName() == "BrepArray"]), 1)

    def test_per_body_material_subsets_survive(self):
        source = Usd.Stage.Open(str(self.cube))
        original = {}
        for child in source.GetPrimAtPath("/World/brepArray").GetChildren():
            subset = UsdGeom.Subset(child)
            if subset and subset.GetElementTypeAttr().Get() == "brep":
                targets = child.GetRelationship("material:binding").GetTargets()
                original[tuple(subset.GetIndicesAttr().Get())] = [str(t) for t in targets]
        # The asset binds a different material to each of the two bodies.
        self.assertEqual(len(original), 2)
        output = self.work / "subsets.usdc"
        result = self.run_heal(self.cube, output)
        self.assertEqual(result.returncode, 0, result.message)
        healed = Usd.Stage.Open(str(output))
        actual = {}
        for child in healed.GetPrimAtPath("/World/brepArray").GetChildren():
            subset = UsdGeom.Subset(child)
            if subset and subset.GetElementTypeAttr().Get() == "brep":
                targets = child.GetRelationship("material:binding").GetTargets()
                actual[tuple(subset.GetIndicesAttr().Get())] = [str(t) for t in targets]
        self.assertEqual(actual, original)

    def prepare_cube_copy(self):
        stage = Usd.Stage.Open(str(self.cube))
        layer = Sdf.Layer.CreateAnonymous()
        layer.TransferContent(stage.GetRootLayer())
        UsdUtils.ModifyAssetPaths(layer, stage.GetRootLayer().ComputeAbsolutePath)
        return Usd.Stage.Open(layer), layer

    def test_shared_material_round_trips_as_array_binding(self):
        stage, layer = self.prepare_cube_copy()
        # Drop the per-body subsets and bind one material to the BrepArray itself, so the
        # binding lives only in the array-level relationship.
        for child in list(stage.GetPrimAtPath("/World/brepArray").GetChildren()):
            self.assertTrue(stage.RemovePrim(child.GetPath()))
        material = "/World/Looks/ABS_Hard_Leather_Brown"
        self.assertTrue(
            stage.GetPrimAtPath("/World/brepArray").CreateRelationship("material:binding").SetTargets([material])
        )
        path = self.work / "shared.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "shared-healed.usda"
        result = self.run_heal(path, output)
        self.assertEqual(result.returncode, 0, result.message)
        healed = Usd.Stage.Open(str(output))
        prim = healed.GetPrimAtPath("/World/brepArray")
        targets = prim.GetRelationship("material:binding").GetTargets()
        self.assertEqual([str(t) for t in targets], [material])

    def test_composed_material_subsets_are_rejected(self):
        # Subsets are rebuilt in the geometry's defining layer. Overrides at a
        # consuming site cannot keep addressing the old subset paths/indices.
        for composition in ("sublayer", "reference", "instance", "internal_reference"):
            for element_type in ("face", "brep"):
                for edit in ("binding", "indices", "new_subset", "deactivate"):
                    with self.subTest(composition=composition, element_type=element_type, edit=edit):
                        case = self.work / f"{composition}-{element_type}-{edit}"
                        case.mkdir()
                        stage, layer = self.prepare_cube_copy()
                        array = stage.GetPrimAtPath("/World/brepArray")
                        for child in list(array.GetChildren()):
                            self.assertTrue(stage.RemovePrim(child.GetPath()))
                        red = UsdShade.Material.Define(stage, "/World/Looks/Red")
                        UsdShade.Material.Define(stage, "/World/Looks/Green")
                        subset = UsdGeom.Subset.Define(stage, "/World/brepArray/redFaces")
                        self.assertTrue(subset.CreateElementTypeAttr(element_type))
                        self.assertTrue(subset.CreateFamilyNameAttr("materialBind"))
                        self.assertTrue(subset.CreateIndicesAttr([0]))
                        self.assertTrue(UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(red))
                        base = case / "base.usda"
                        self.assertTrue(layer.Export(str(base)))

                        source = case / "input.usda"
                        if composition == "internal_reference":
                            self.assertTrue(layer.Export(str(source)))
                            root = Usd.Stage.Open(str(source))
                            self.assertTrue(root.DefinePrim("/Model").GetReferences().AddInternalReference("/World"))
                        else:
                            root = Usd.Stage.CreateNew(str(source))
                            if composition == "sublayer":
                                root.GetRootLayer().subLayerPaths.append(str(base))
                            else:
                                self.assertTrue(
                                    root.DefinePrim("/Model").GetReferences().AddReference(str(base), "/World")
                                )
                        model_path = "/World" if composition == "sublayer" else "/Model"
                        subset_path = model_path + "/brepArray/redFaces"
                        if edit == "new_subset":
                            subset_path = model_path + "/brepArray/greenFaces"
                            added = UsdGeom.Subset.Define(root, subset_path)
                            self.assertTrue(added.CreateElementTypeAttr(element_type))
                            self.assertTrue(added.CreateFamilyNameAttr("materialBind"))
                            self.assertTrue(added.CreateIndicesAttr([1]))
                        override = root.OverridePrim(subset_path)
                        if edit == "indices":
                            self.assertTrue(UsdGeom.Subset(override).GetIndicesAttr().Set([1]))
                        elif edit == "deactivate":
                            self.assertTrue(override.SetActive(False))
                        else:
                            green = UsdShade.Material(root.GetPrimAtPath(model_path + "/Looks/Green"))
                            self.assertTrue(UsdShade.MaterialBindingAPI.Apply(override).Bind(green))
                            bound = UsdShade.MaterialBindingAPI(override).ComputeBoundMaterial()[0]
                            self.assertEqual(bound.GetPath(), green.GetPath())
                        self.assertTrue(root.GetRootLayer().Save())
                        if composition == "instance":
                            # The override lives in the referenced model, below
                            # an instance root in the top-level composition.
                            instance_path = case / "instance.usda"
                            instances = Usd.Stage.CreateNew(str(instance_path))
                            instance = instances.DefinePrim("/Instance", "Xform")
                            self.assertTrue(instance.GetReferences().AddReference(str(source), "/Model"))
                            self.assertTrue(instance.SetInstanceable(True))
                            self.assertTrue(instances.GetRootLayer().Save())
                            source = instance_path
                        before = {path: path.read_bytes() for path in case.glob("*.usda")}
                        output = case / "healed.usda"
                        result = self.run_heal(source, output)
                        self.assertNotEqual(result.returncode, 0, result.message)
                        self.assertIn("SM_ERR_INVALID_INPUT", result.message)
                        self.assertIn("material subset composed outside the BRep definition", result.message)
                        self.assertFalse(output.exists())
                        self.assertFalse((case / "healed_layers").exists())
                        for path, contents in before.items():
                            self.assertEqual(path.read_bytes(), contents)

    def test_package_relative_geometry_layers_round_trip(self):
        # Author the package explicitly, with no external asset dependencies.
        # This also avoids package-builder temporary files outside the test dir.
        stage, layer = self.prepare_cube_copy()
        self.assertTrue(stage.RemovePrim("/World/Looks"))
        for child in list(stage.GetPrimAtPath("/World/brepArray").GetChildren()):
            self.assertTrue(stage.RemovePrim(child.GetPath()))
        geometry_dir = self.work / "geometry"
        geometry_dir.mkdir()
        geometry = geometry_dir / "part.usdc"
        self.assertTrue(layer.Export(str(geometry)))
        root_path = self.work / "scene.usda"
        root = Usd.Stage.CreateNew(str(root_path))
        self.assertTrue(root.DefinePrim("/Model").GetReferences().AddReference("geometry/part.usdc", "/World"))
        self.assertTrue(root.GetRootLayer().Save())
        package = self.work / "scene.usdz"
        with Sdf.ZipFileWriter.CreateNew(str(package)) as writer:
            self.assertEqual(writer.AddFile(str(root_path), "scene.usda"), "scene.usda")
            self.assertEqual(writer.AddFile(str(geometry), "geometry/part.usdc"), "geometry/part.usdc")
        original = package.read_bytes()
        source = Usd.Stage.Open(str(package))
        self.assertFalse(source.GetCompositionErrors())
        wrapper_path = self.work / "wrapper.usda"
        wrapper = Usd.Stage.CreateNew(str(wrapper_path))
        self.assertTrue(wrapper.DefinePrim("/Wrapped").GetReferences().AddReference("scene.usdz", "/Model"))
        self.assertTrue(wrapper.GetRootLayer().Save())
        for input_path, model_path in ((package, "/Model"), (wrapper_path, "/Wrapped")):
            with self.subTest(input=input_path.name):
                output_dir = self.work / (input_path.stem + "-output")
                output_dir.mkdir()
                output = output_dir / "healed.usda"
                result = self.run_heal(input_path, output)
                self.assertEqual(result.returncode, 0, result.message)
                self.assertEqual([r.brep_count for r in result.results], [2])
                self.assertEqual(package.read_bytes(), original)
                moved = self.work / (input_path.stem + "-moved")
                output_dir.rename(moved)
                healed = Usd.Stage.Open(str(moved / output.name))
                self.assertFalse(healed.GetCompositionErrors())
                array = healed.GetPrimAtPath(model_path + "/brepArray")
                self.assertEqual(array.GetTypeName(), "BrepArray")
                self.assertEqual(array.GetCustomDataByKey("source"), "SMLib")
                self.assertEqual(len(array.GetAttribute("brep:regionCount").Get()), 2)
                for dependency in healed.GetUsedLayers():
                    if dependency.anonymous or dependency == healed.GetRootLayer():
                        continue
                    self.assertEqual(Path(dependency.realPath).parent, moved / "healed_layers")
                    self.assertIn(Path(dependency.realPath).suffix, (".usdc", ".usda"))
                # Re-import the serialized geometry, not just the scene description.
                again = self.run_heal(moved / output.name, self.work / (input_path.stem + "-again.usda"))
                self.assertEqual(again.returncode, 0, again.message)
                self.assertEqual([r.brep_count for r in again.results], [2])

    def test_external_unselected_variant_keeps_original_reference(self):
        for name in ("a", "b"):
            stage, layer = self.prepare_cube_copy()
            stage.GetPrimAtPath("/World/brepArray").SetCustomDataByKey("source", "unhealed")
            self.assertTrue(layer.Export(str(self.work / f"{name}.usda")))
        source = self.work / "variants.usda"
        stage = Usd.Stage.CreateNew(str(source))
        model = stage.DefinePrim("/Model", "Xform")
        variants = model.GetVariantSets().AddVariantSet("shape")
        for name in ("a", "b"):
            self.assertTrue(variants.AddVariant(name))
            self.assertTrue(variants.SetVariantSelection(name))
            with variants.GetVariantEditContext():
                self.assertTrue(model.GetReferences().AddReference(f"{name}.usda", "/World"))
        self.assertTrue(variants.SetVariantSelection("a"))
        self.assertTrue(stage.GetRootLayer().Save())
        unselected = self.work / "b.usda"
        before = unselected.read_bytes()
        output = self.work / "variants-healed.usda"
        result = self.run_heal(source, output)
        self.assertEqual(result.returncode, 0, result.message)
        self.assertEqual(len(result.results), 1)
        self.assertEqual(Path(result.results[0].layer), self.work / "a.usda")
        healed = Usd.Stage.Open(str(output))
        self.assertFalse(healed.GetCompositionErrors())
        self.assertEqual(healed.GetPrimAtPath("/Model/brepArray").GetCustomDataByKey("source"), "SMLib")
        self.assertTrue(healed.GetPrimAtPath("/Model").GetVariantSets().GetVariantSet("shape").SetVariantSelection("b"))
        self.assertFalse(healed.GetCompositionErrors())
        array = healed.GetPrimAtPath("/Model/brepArray")
        self.assertEqual(array.GetCustomDataByKey("source"), "unhealed")
        stack = array.GetAttribute("face:loopCount").GetPropertyStack()
        self.assertEqual(Path(stack[0].layer.realPath), unselected)
        self.assertEqual(unselected.read_bytes(), before)

    def test_array_binding_is_rebuilt_as_usdshade_authors_it(self):
        # CreateRelationship authors custom = true; the healer writes UsdShade's custom = false, uniform,
        # whether the writer authors the binding (no subsets) or the healer restores it (per-body subsets).
        material = "/World/Looks/ABS_Hard_Leather_Brown"
        for subsets in (False, True):
            with self.subTest(subsets=subsets):
                stage, layer = self.prepare_cube_copy()
                array = stage.GetPrimAtPath("/World/brepArray")
                if not subsets:
                    for child in list(array.GetChildren()):
                        self.assertTrue(stage.RemovePrim(child.GetPath()))
                binding = array.CreateRelationship("material:binding")
                self.assertTrue(binding.SetTargets([material]))
                self.assertTrue(binding.SetMetadata("variability", Sdf.VariabilityVarying))
                path = self.work / f"custom-{subsets}.usda"
                self.assertTrue(layer.Export(str(path)))
                output = self.work / f"custom-{subsets}-healed.usda"
                result = self.run_heal(path, output)
                self.assertEqual(result.returncode, 0, result.message)
                healed_stage = Usd.Stage.Open(str(output))
                healed = healed_stage.GetPrimAtPath("/World/brepArray").GetRelationship("material:binding")
                self.assertEqual([str(t) for t in healed.GetTargets()], [material])
                self.assertFalse(healed.IsCustom())
                self.assertEqual(healed.GetMetadata("variability"), Sdf.VariabilityUniform)

    def test_binding_strength_is_rejected(self):
        # material:binding is rebuilt with its targets only, which would drop bindMaterialAs.
        for where in ("array", "subset"):
            with self.subTest(where=where):
                stage, layer = self.prepare_cube_copy()
                array = stage.GetPrimAtPath("/World/brepArray")
                prim = array if where == "array" else array.GetChildren()[0]
                if where == "array":
                    self.assertTrue(
                        prim.CreateRelationship("material:binding").SetTargets(["/World/Looks/ABS_Hard_Leather_Brown"])
                    )
                binding = prim.GetRelationship("material:binding")
                self.assertTrue(binding.SetMetadata("bindMaterialAs", "strongerThanDescendants"))
                path = self.work / f"strength-{where}.usda"
                self.assertTrue(layer.Export(str(path)))
                output = self.work / f"strength-{where}-healed.usda"
                result = self.run_heal(path, output)
                self.assertNotEqual(result.returncode, 0, result.message)
                self.assertIn("metadata the healer cannot preserve", result.message)
                self.assertFalse(output.exists())

    def test_unbound_geometry_gets_no_material_binding(self):
        stage, layer = self.prepare_cube_copy()
        # No per-body subsets and no array-level binding: nothing must be authored.
        for child in list(stage.GetPrimAtPath("/World/brepArray").GetChildren()):
            self.assertTrue(stage.RemovePrim(child.GetPath()))
        path = self.work / "unbound.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "unbound-healed.usda"
        result = self.run_heal(path, output)
        self.assertEqual(result.returncode, 0, result.message)
        healed = Usd.Stage.Open(str(output))
        prim = healed.GetPrimAtPath("/World/brepArray")
        # An empty-target relationship reads back as a binding to nothing.
        self.assertFalse(prim.GetRelationship("material:binding").HasAuthoredTargets())

    def test_face_material_subset_stays_within_its_body(self):
        stage = Usd.Stage.Open(str(self.cube))
        layer = Sdf.Layer.CreateAnonymous()
        layer.TransferContent(stage.GetRootLayer())
        UsdUtils.ModifyAssetPaths(layer, stage.GetRootLayer().ComputeAbsolutePath)
        stage = Usd.Stage.Open(layer)
        # Two cubes of six faces each: bind a face subset to the first body alone.
        # Face-material indices are global to the BrepArray, so the importer sees them
        # while converting the second body too, where subtracting that body's face start
        # underflows and produces a wild index instead of an out-of-range one.
        subset = UsdGeom.Subset.Define(stage, "/World/brepArray/faceSubset")
        self.assertTrue(subset.GetElementTypeAttr().Set("face"))
        self.assertTrue(subset.GetFamilyNameAttr().Set("materialBind"))
        self.assertTrue(subset.GetIndicesAttr().Set(list(range(0, 6))))
        self.assertTrue(
            subset.GetPrim()
            .CreateRelationship("material:binding")
            .SetTargets(["/World/Looks/ABS_Hard_Leather_Deep_Green"])
        )
        path = self.work / "facemats.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "facemats-healed.usda"
        result = self.run_heal(path, output)
        self.assertEqual(result.returncode, 0, result.message)
        healed = Usd.Stage.Open(str(output))
        faces = [
            UsdGeom.Subset(child)
            for child in healed.GetPrimAtPath("/World/brepArray").GetChildren()
            if UsdGeom.Subset(child) and UsdGeom.Subset(child).GetElementTypeAttr().Get() == "face"
        ]
        self.assertEqual(len(faces), 1)
        indices = sorted(faces[0].GetIndicesAttr().Get())
        # Only the first body's faces may carry the binding. Without the ownership
        # check the second body indexed far past its face array and this segfaulted.
        self.assertEqual(indices, list(range(0, 6)))

    def test_empty_asset_array_entries_survive(self):
        stage = Usd.Stage.Open(str(self.cube))
        layer = Sdf.Layer.CreateAnonymous()
        layer.TransferContent(stage.GetRootLayer())
        UsdUtils.ModifyAssetPaths(layer, stage.GetRootLayer().ComputeAbsolutePath)
        stage = Usd.Stage.Open(layer)
        prim = stage.DefinePrim("/World/Assets", "Scope")
        paths = [Sdf.AssetPath("a.usda"), Sdf.AssetPath(""), Sdf.AssetPath("b.usda")]
        attr = prim.CreateAttribute("customAssets", Sdf.ValueTypeNames.AssetArray)
        self.assertTrue(attr.Set(paths))
        self.assertTrue(attr.Set(paths, 1.0))
        path = self.work / "assets.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "assets-healed.usda"
        result = self.run_heal(path, output)
        self.assertEqual(result.returncode, 0, result.message)
        healed = Usd.Stage.Open(str(output))
        healed_attr = healed.GetPrimAtPath("/World/Assets").GetAttribute("customAssets")
        # Length and index correspondence must survive both asset-path rewrite passes, in the
        # default value and in time samples.
        for time in (Usd.TimeCode.Default(), 1.0):
            actual = healed_attr.Get(time)
            self.assertEqual(len(actual), 3, time)
            self.assertEqual(actual[1].path, "", time)
            self.assertTrue(actual[0].path.endswith("a.usda"), time)
            self.assertTrue(actual[2].path.endswith("b.usda"), time)

    def test_url_asset_paths_survive_in_dependency_layers(self):
        # Anchored like file paths, "https://host/x" became "https:/host/x".
        urls = ["https://example.com/Materials/Leather.mdl", "omniverse://server/Library/Metal.mdl"]
        looks = Usd.Stage.CreateNew(str(self.work / "looks.usda"))
        shader = looks.DefinePrim("/Looks/Leather/Shader", "Shader")
        self.assertTrue(shader.CreateAttribute("info:mdl:sourceAsset", Sdf.ValueTypeNames.Asset).Set(urls[0]))
        self.assertTrue(shader.CreateAttribute("customAssets", Sdf.ValueTypeNames.AssetArray).Set(urls))
        self.assertTrue(looks.GetRootLayer().Save())
        source = Usd.Stage.CreateNew(str(self.work / "input.usda"))
        source.DefinePrim("/World", "Xform")
        self.assertTrue(source.DefinePrim("/World/Part").GetReferences().AddReference(str(self.cube), "/World"))
        self.assertTrue(source.DefinePrim("/World/Looks").GetReferences().AddReference("./looks.usda", "/Looks"))
        self.assertTrue(source.GetRootLayer().Save())
        output = self.work / "healed.usda"
        script = self.repo / "tools/scripts/usd_brep_heal.py"
        command = [sys.executable, "-c", self._RUN_SCRIPT, str(script), str(self.work / "input.usda"), str(output)]
        healed = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(healed.returncode, 0, healed.stderr)
        # Pointing the layers at their healed copies must not recompose the scene over paths
        # that do not exist yet.
        self.assertNotIn("Could not open asset", healed.stderr)
        stage = Usd.Stage.Open(str(output))
        shader = stage.GetPrimAtPath("/World/Looks/Leather/Shader")
        self.assertEqual(shader.GetAttribute("info:mdl:sourceAsset").Get().path, urls[0])
        self.assertEqual([path.path for path in shader.GetAttribute("customAssets").Get()], urls)

    def test_partial_body_failure_does_not_publish_output(self):
        stage = Usd.Stage.Open(str(self.cube))
        layer = Sdf.Layer.CreateAnonymous()
        layer.TransferContent(stage.GetRootLayer())
        UsdUtils.ModifyAssetPaths(layer, stage.GetRootLayer().ComputeAbsolutePath)
        # Give the second body's nurb patches an invalid order, so surface construction
        # fails for that body alone. The file-level operation must not publish
        # the successfully imported first body as if the whole array healed.
        spec = layer.GetAttributeAtPath("/World/brepArray.brep:surface:nurb:uOrder")
        self.assertIsNotNone(spec)
        orders = list(spec.default)
        # Two cubes, six nurb patches each: the second body owns the trailing half.
        self.assertEqual(len(orders), 12)
        spec.default = orders[:6] + [0] * 6
        path = self.work / "partial.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "partial-healed.usdc"
        result = self.run_heal(path, output)
        self.assertNotEqual(result.returncode, 0, result.message)
        self.assertFalse(output.exists())

    def test_variant_authored_brep_is_rejected(self):
        stage, layer = self.prepare_cube_copy()
        # Move the geometry under a variant, so the authored brep:* opinions live at a
        # variant-decorated spec path that Write() could not reproduce.
        model = stage.DefinePrim("/Model", "Xform")
        variants = model.GetVariantSets().AddVariantSet("shape")
        variants.AddVariant("a")
        variants.SetVariantSelection("a")
        with variants.GetVariantEditContext():
            stage.DefinePrim("/Model/Part").GetReferences().AddInternalReference("/World/brepArray")
            stage.GetPrimAtPath("/Model/Part").CreateAttribute("brep:regionCount", Sdf.ValueTypeNames.UIntArray).Set(
                [2, 2]
            )
        path = self.work / "variant.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "variant-healed.usda"
        result = self.run_heal(path, output)
        self.assertNotEqual(result.returncode, 0, result.message)
        self.assertIn("authored inside a variant", result.message)
        self.assertFalse(output.exists())

    def test_untyped_stronger_override_is_rejected(self):
        # A weaker layer holds the typed definition; the root layer contributes a brep:*
        # opinion through an `over` that never reauthors typeName.
        stage, layer = self.prepare_cube_copy()
        weak = self.work / "weak.usda"
        self.assertTrue(layer.Export(str(weak)))
        root = Usd.Stage.CreateNew(str(self.work / "strong.usda"))
        root.GetRootLayer().subLayerPaths.append(str(weak))
        over = root.OverridePrim("/World/brepArray")
        self.assertEqual(over.GetTypeName(), "BrepArray")
        self.assertTrue(over.CreateAttribute("brep:regionCount", Sdf.ValueTypeNames.UIntArray).Set([2, 2]))
        self.assertTrue(root.GetRootLayer().Save())
        # The override must be authored as an `over`, not a typed def.
        spec = root.GetRootLayer().GetPrimAtPath("/World/brepArray")
        self.assertEqual(spec.typeName, "")
        output = self.work / "strong-healed.usda"
        result = self.run_heal(self.work / "strong.usda", output)
        self.assertNotEqual(result.returncode, 0, result.message)
        self.assertIn("composed across layers", result.message)
        self.assertFalse(output.exists())

    def test_unprefixed_topology_override_is_rejected(self):
        # Topology attributes are authored without the brep: prefix, so a guard keyed on
        # that prefix alone would miss a stronger layer overriding them.
        stage, layer = self.prepare_cube_copy()
        weak = self.work / "topo_weak.usda"
        self.assertTrue(layer.Export(str(weak)))
        root = Usd.Stage.CreateNew(str(self.work / "topo_strong.usda"))
        root.GetRootLayer().subLayerPaths.append(str(weak))
        over = root.OverridePrim("/World/brepArray")
        self.assertTrue(over.CreateAttribute("face:loopCount", Sdf.ValueTypeNames.UIntArray).Set([1] * 12))
        self.assertTrue(root.GetRootLayer().Save())
        output = self.work / "topo-healed.usda"
        result = self.run_heal(self.work / "topo_strong.usda", output)
        self.assertNotEqual(result.returncode, 0, result.message)
        self.assertIn("composed across layers", result.message)
        self.assertFalse(output.exists())

    def test_out_of_range_subset_index_is_rejected(self):
        stage, layer = self.prepare_cube_copy()
        # Two cubes of six faces each, so face 99 belongs to no body. Conversion skips
        # indices outside the body being converted, which would silently drop this one.
        subset = UsdGeom.Subset.Define(stage, "/World/brepArray/strayFaces")
        self.assertTrue(subset.GetElementTypeAttr().Set("face"))
        self.assertTrue(subset.GetFamilyNameAttr().Set("materialBind"))
        self.assertTrue(subset.GetIndicesAttr().Set([99]))
        self.assertTrue(
            subset.GetPrim().CreateRelationship("material:binding").SetTargets(["/World/Looks/ABS_Hard_Leather_Brown"])
        )
        path = self.work / "stray.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "stray-healed.usda"
        result = self.run_heal(path, output)
        self.assertNotEqual(result.returncode, 0, result.message)
        self.assertIn("is outside the BrepArray", result.message)
        self.assertFalse(output.exists())

    def test_purpose_only_subset_binding_is_rejected(self):
        stage, layer = self.prepare_cube_copy()
        # Passes the family check, but the converter reads only the unqualified
        # material:binding, so this subset would be removed without a replacement.
        subset = UsdGeom.Subset.Define(stage, "/World/brepArray/previewOnly")
        self.assertTrue(subset.GetElementTypeAttr().Set("brep"))
        self.assertTrue(subset.GetFamilyNameAttr().Set("materialBind"))
        self.assertTrue(subset.GetIndicesAttr().Set([0]))
        self.assertTrue(
            subset.GetPrim()
            .CreateRelationship("material:binding:preview")
            .SetTargets(["/World/Looks/ABS_Hard_Leather_Brown"])
        )
        path = self.work / "preview.usda"
        self.assertTrue(layer.Export(str(path)))
        output = self.work / "preview-healed.usda"
        result = self.run_heal(path, output)
        self.assertNotEqual(result.returncode, 0, result.message)
        self.assertIn("no material:binding the healer can preserve", result.message)
        self.assertFalse(output.exists())

    def test_invalid_geometry_does_not_publish_output(self):
        stage = Usd.Stage.CreateNew(str(self.work / "bad.usda"))
        stage.DefinePrim("/Bad", "BrepArray")
        self.assertTrue(stage.GetRootLayer().Save())
        output = self.work / "bad-output.usdc"
        result = self.run_heal(self.work / "bad.usda", output)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(output.exists())

    def test_scene_without_brep_array_is_rejected(self):
        stage = Usd.Stage.CreateNew(str(self.work / "no-brep.usda"))
        stage.DefinePrim("/World", "Xform")
        self.assertTrue(stage.GetRootLayer().Save())
        output = self.work / "no-brep-healed.usda"
        result = self.run_heal(self.work / "no-brep.usda", output)
        self.assertIn("no BrepArray prims to heal", result.message)
        self.assertFalse(output.exists())

    def test_existing_output_is_unchanged(self):
        output = self.work / "existing.usdc"
        output.write_bytes(b"keep this file")
        result = self.run_heal(self.cube, output)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output.read_bytes(), b"keep this file")

    # Runs the script as __main__ in a child Python. In the build tree, Windows finds _omni_solid's DLLs only
    # through os.add_dll_directory (usd_brep adds its own directories only in the packaged layout), so the
    # child adds the PATH directories first, as this module does above.
    _RUN_SCRIPT = """
import os
import runpy
import sys

dll_directories = []
if sys.platform == "win32":
    for path in os.environ.get("PATH", "").split(";"):
        if path and os.path.isdir(path):
            dll_directories.append(os.add_dll_directory(path))

sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
"""

    def test_command_line_tool(self):
        script = self.repo / "tools/scripts/usd_brep_heal.py"
        output = self.work / "healed.usda"
        command = [sys.executable, "-c", self._RUN_SCRIPT, str(script), str(self.cube), str(output)]
        healed = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(healed.returncode, 0, healed.stderr)
        self.assertIn("Brep(s) healed", healed.stdout)
        self.assertTrue(output.is_file())
        refused = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(refused.returncode, 1)
        self.assertIn("output already exists", refused.stderr)
