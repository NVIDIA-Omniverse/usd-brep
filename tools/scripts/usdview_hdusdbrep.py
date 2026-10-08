#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Launch usdview with the hdUsdBrep Hydra plugin from a release build or a release package.

Usage:
    python tools/scripts/usdview_hdusdbrep.py [usdview arguments] <asset.usda>

In a source checkout, uses the bundled Python and USD from _build/target-deps.
In an extracted release package, uses the package's bundled OpenUSD (extraLibs/, usdpy/)
with the Python running this script; the package bundles no interpreter.
usdview itself needs the USD package's Python requirements (PySide6 and PyOpenGL)
installed for that Python.
"""

import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# A release package keeps this script at the same depth but has no _build tree.
IS_PACKAGE = not (ROOT / "_build").is_dir() and (ROOT / "omniSolid" / "resources").is_dir()

# The package bundles pxr but not OpenUSD's usdview script, which only runs this.
USDVIEW_MAIN = "import pxr.Usdviewq as Usdviewq; Usdviewq.Launcher().Run()"


def release_paths():
    """Return (build, usd, python, schema) directories of the release build."""
    if IS_PACKAGE:
        # See repo.toml: our libraries and plugins in lib/, OpenUSD's in extraLibs/,
        # our Python modules in python/ and OpenUSD's pxr in usdpy/.
        return ROOT / "lib", ROOT / "extraLibs", ROOT / "python", ROOT / "omniSolid/resources"
    machine = platform.machine().lower()
    architecture = "aarch64" if machine in ("arm64", "aarch64") else machine
    build_platform = "windows-x86_64" if sys.platform == "win32" else f"linux-{architecture}"
    return (
        ROOT / "_build" / build_platform / "release",
        ROOT / "_build/target-deps/usd/release",
        ROOT / "_build/target-deps/python",
        ROOT / "_build/schema/omniSolid/resources",
    )


def usdview_environment():
    """Return (environment, python, usdview) for running usdview with hdUsdBrep."""
    build, usd, python, schema = release_paths()
    env = dict(os.environ)

    def prepend(name, paths):
        env[name] = os.pathsep.join([str(path) for path in paths] + ([env[name]] if env.get(name) else []))

    if IS_PACKAGE:
        # usd is extraLibs/ and python is the package's module directory; the interpreter
        # is the caller's.
        prepend("PATH" if sys.platform == "win32" else "LD_LIBRARY_PATH", [build, usd])
        prepend("PYTHONPATH", [python, ROOT / "usdpy"])
    elif sys.platform == "win32":
        prepend("PATH", [build, usd / "bin", usd / "lib", python])
    else:
        prepend("LD_LIBRARY_PATH", [build, usd / "lib", python / "lib"])
    if not IS_PACKAGE:
        prepend("PYTHONPATH", [build, usd / "lib/python"])
    prepend("PXR_PLUGINPATH_NAME", [schema, build / "plugins/hdUsdBrep"])
    env["USDIMAGINGGL_ENGINE_ENABLE_SCENE_INDEX"] = "1"
    # USD 25.11 needs this to expand the procedurals; otherwise usdview shows only bounds.
    env["HDGP_INCLUDE_DEFAULT_RESOLVER"] = "1"
    if sys.platform.startswith("linux"):
        # usdview shows its window before adding the QOpenGLWidget viewport, which makes Qt destroy and
        # recreate the window. Compositing through OpenGL from the start avoids that close-and-reopen.
        env.setdefault("QT_WIDGETS_RHI", "1")
        env.setdefault("QT_WIDGETS_RHI_BACKEND", "opengl")
    if IS_PACKAGE:
        return env, Path(sys.executable), None  # usdview runs from the bundled pxr; see main()
    interpreter = python / "python.exe" if sys.platform == "win32" else python / "bin/python3"
    return env, interpreter, usd / "bin/usdview"


def main():
    if sys.platform.startswith("linux") and platform.machine().lower() in ("arm64", "aarch64"):
        sys.exit("usdview is not yet supported on linux-aarch64.")
    env, interpreter, usdview = usdview_environment()
    plugin = Path(env["PXR_PLUGINPATH_NAME"].split(os.pathsep)[1]) / "plugInfo.json"
    if IS_PACKAGE and not plugin.exists():
        sys.exit("This package has no hdUsdBrep plugin (its OpenUSD has no imaging), so it cannot run usdview.")
    for required in [interpreter, plugin] + ([usdview] if usdview else []):
        if not required.exists():
            sys.exit(f"{required} not found; build the release configuration first.")
    # -P: with -c, Python would search the current directory first, so a pxr/ there (say, in an asset's
    # folder) would run instead of the bundled one.
    command = [str(interpreter), "-P", "-c", USDVIEW_MAIN] if usdview is None else [str(interpreter), str(usdview)]
    return subprocess.call(command + sys.argv[1:], env=env)


if __name__ == "__main__":
    sys.exit(main())
