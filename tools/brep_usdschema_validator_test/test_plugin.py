# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import importlib.machinery
import multiprocessing
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

TEST_ROOT = Path(__file__).resolve().parent
if str(TEST_ROOT) not in sys.path:
    sys.path.insert(0, str(TEST_ROOT))
try:
    from .utils.base_test_case import BrepUsdSchemaValidatorBaseTestCase
except ImportError:
    from utils.base_test_case import BrepUsdSchemaValidatorBaseTestCase

from brep_usdschema_validator import BrepUsdSchemaValidator, plugin, validate  # noqa: E402
from usd_validation_nvidia import CategoryRuleRegistry, ValidationEngine, is_multiprocess_safe  # noqa: E402


class BrepUsdSchemaValidatorPluginTestCase(BrepUsdSchemaValidatorBaseTestCase):
    def tearDown(self):
        plugin.unregister_all()

    def test_plugin_registers_and_unregisters_brep_usdschema_validator(self):
        registry = CategoryRuleRegistry()
        plugin.unregister_all()
        self.assertIsNone(registry.get_category(BrepUsdSchemaValidator))

        plugin.BrepUsdSchemaValidatorPlugin().on_startup()
        plugin.register_all()  # repeat registration is a no-op
        self.assertEqual(registry.get_category(BrepUsdSchemaValidator), "Omni:Geometry")

        plugin.BrepUsdSchemaValidatorPlugin().on_shutdown()
        self.assertIsNone(registry.get_category(BrepUsdSchemaValidator))

    def test_bundled_schema_is_found_in_the_wheel_layout(self):
        with tempfile.TemporaryDirectory() as root:
            resources = Path(root) / "usd_brep" / "omniSolid" / "resources"
            resources.mkdir(parents=True)
            (resources / "plugInfo.json").write_text("{}")
            spec = importlib.machinery.ModuleSpec("usd_brep", None, is_package=True)
            spec.submodule_search_locations = [str(Path(root) / "usd_brep")]
            with mock.patch.object(validate.importlib.util, "find_spec", return_value=spec):
                self.assertEqual(validate._bundled_schema_path(), resources)

    def test_invalid_plugin_path_falls_back_to_the_bundled_schema(self):
        # A missing path, and an existing directory with no plugInfo.json, both register nothing.
        with tempfile.TemporaryDirectory() as root:
            bundled = Path(root) / "bundled"
            bundled.mkdir()
            (bundled / "plugInfo.json").write_text("{}")
            not_a_schema = Path(root) / "not_a_schema"
            not_a_schema.mkdir()
            for env_path in (Path(root) / "missing", not_a_schema):
                with self.subTest(env_path=env_path.name), \
                        mock.patch.dict(validate.os.environ, {"OMNISOLID_PLUGIN_PATH": str(env_path)}), \
                        mock.patch.object(validate, "_bundled_schema_path", return_value=bundled), \
                        mock.patch.object(validate, "Plug") as plug:
                    self.assertTrue(validate.register_omnisolid_schema())
                    plug.Registry.return_value.RegisterPlugins.assert_called_once_with(str(bundled.resolve()))

    def test_missing_explicit_plugin_path_is_reported_not_replaced(self):
        with tempfile.TemporaryDirectory() as root:
            missing = str(Path(root) / "missing")
            with mock.patch.dict(validate.os.environ, {"OMNISOLID_PLUGIN_PATH": root}), \
                    mock.patch.object(validate, "_bundled_schema_path", return_value=Path(root)), \
                    mock.patch.object(validate, "Plug") as plug:
                self.assertFalse(validate.register_omnisolid_schema(missing))
                plug.Registry.return_value.RegisterPlugins.assert_not_called()
                with self.assertRaises(FileNotFoundError):
                    validate.usdschema_validate_file("unused.usda", omnisolid_plugin_path=missing)

    def test_process_pool_reports_the_same_issues(self):
        # CheckPrim is marked multiprocess-safe, so the engine pickles the rule to its worker processes.
        self.assertTrue(is_multiprocess_safe(BrepUsdSchemaValidator.CheckPrim))
        # spawn, as on Windows and macOS: a forked worker would inherit the schema and hide its absence.
        multiprocessing.set_start_method("spawn", force=True)
        usd_file = str(self.test_files_dir / "Test_BA_763_UV_trim_curves_do_not_close_loop.usda")

        def issues(processes):
            engine = ValidationEngine(init_rules=False, processes=processes)
            engine.enable_rule(BrepUsdSchemaValidator)
            return sorted((str(i.severity), str(i.message), str(i.at)) for i in engine.validate(usd_file).issues())

        inline = issues(0)
        self.assertTrue(inline)
        self.assertEqual(issues(2), inline)


if __name__ == "__main__":
    unittest.main()
