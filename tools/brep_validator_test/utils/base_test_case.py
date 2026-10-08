# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import os
import subprocess
import sys
import unittest
from pathlib import Path


def _prepend_env_path(var_name: str, value: Path) -> None:
    current = os.environ.get(var_name, "")
    value_str = str(value)
    os.environ[var_name] = f"{value_str}{os.pathsep}{current}" if current else value_str


def _ensure_usd_validation_nvidia() -> None:
    # brep_validator needs `omni.capabilities` and `usd_validation_nvidia`, both
    # shipped by the usd-validation-nvidia pip package. Install it into the
    # bundled interpreter on first run (so it imports without PYTHONPATH tweaks),
    # mirroring scene-optimizer-core's test runner.
    if importlib.util.find_spec("usd_validation_nvidia") is not None:
        return
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--quiet",
            "--disable-pip-version-check",
            "usd-validation-nvidia>=1.22.0,<2",
        ],
        check=True,
    )


def _find_package_root(start: Path) -> Path:
    current = start.resolve()
    if current.is_file():
        current = current.parent

    for candidate in [current, *current.parents]:
        has_validator = (candidate / "brep_validator").is_dir() or (candidate / "tools" / "brep_validator").is_dir()
        has_fixtures = (candidate / "TestFiles" / "brep_validator").is_dir()
        if has_validator and has_fixtures:
            return candidate
    raise RuntimeError(f"Could not locate brep_validator package root from {start}")


def bootstrap_environment() -> Path:
    repo_root = _find_package_root(Path(__file__).resolve())
    tools_root = repo_root / "tools" if (repo_root / "tools" / "brep_validator").exists() else repo_root
    usd_root = repo_root / "_build" / "target-deps" / "usd" / "release"
    python_root = repo_root / "_build" / "target-deps" / "python"
    omnisolid_root = repo_root / "_build" / "schema" / "omniSolid" / "resources"

    # tools_root holds the brep_validator package so `from brep_validator import ...` resolves.
    sys.path.insert(0, str(tools_root))
    sys.path.insert(0, str(usd_root / "lib" / "python"))

    _prepend_env_path("PYTHONPATH", tools_root)
    _prepend_env_path("PYTHONPATH", usd_root / "lib" / "python")
    _prepend_env_path("PATH", usd_root / "bin")
    _prepend_env_path("PATH", usd_root / "lib")
    _prepend_env_path("LD_LIBRARY_PATH", usd_root / "lib")
    _prepend_env_path("LD_LIBRARY_PATH", python_root / "lib")

    os.environ["USD_ROOT"] = str(usd_root)
    if omnisolid_root.exists():
        os.environ["OMNISOLID_PLUGIN_PATH"] = str(omnisolid_root)

    _ensure_usd_validation_nvidia()

    return repo_root


REPO_ROOT = bootstrap_environment()

from pxr import Plug, Usd  # noqa: E402

from brep_validator import BrepValidator  # noqa: E402


class BrepValidatorBaseTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        omnisolid_path = os.environ.get("OMNISOLID_PLUGIN_PATH")
        if omnisolid_path:
            Plug.Registry().RegisterPlugins(str(Path(omnisolid_path).resolve()))

        cls.repo_root = REPO_ROOT
        cls.test_files_dir = cls.repo_root / "TestFiles" / "brep_validator"

    def _get_brep_prims(self, usd_file: Path):
        stage = Usd.Stage.Open(str(usd_file))
        self.assertIsNotNone(stage, f"Failed to open USD stage: {usd_file}")
        return [prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]

    def _has_brep_prims(self, usd_file: Path) -> bool:
        return bool(self._get_brep_prims(usd_file))

    def _collect_issues(self, usd_file: Path):
        stage = Usd.Stage.Open(str(usd_file))
        self.assertIsNotNone(stage, f"Failed to open USD stage: {usd_file}")

        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
        brep_prims = [prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]
        self.assertTrue(brep_prims, f"No BrepArray prims found in {usd_file}")

        for prim in brep_prims:
            checker.CheckPrim(prim)

        return checker.GetIssues()
