# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""BRep solid modeling with OpenUSD interop."""

import importlib.util
import os
import sys
import types
from pathlib import Path

_HERE = Path(__file__).parent

# Not PATH: that loads whichever OpenUSD is installed system-wide.
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    _dll_dirs = [_HERE, _HERE.parent, _HERE.parent.parent / "lib"]
    # The packman layout (lib/ and omniSolid/ beside python/) bundles OpenUSD in extraLibs/.
    # Without it, use the OpenUSD whose pxr this Python imports, as pxr itself does:
    # PXR_USD_WINDOWS_DLL_PATH if set, else the lib/ and bin/ beside pxr.
    if (_HERE.parent.parent / "lib").is_dir() and (_HERE.parent.parent / "omniSolid").is_dir():
        if (_HERE.parent.parent / "extraLibs").is_dir():
            _dll_dirs.append(_HERE.parent.parent / "extraLibs")
        elif os.environ.get("PXR_USD_WINDOWS_DLL_PATH"):
            _dll_dirs += [Path(p) for p in os.environ["PXR_USD_WINDOWS_DLL_PATH"].split(os.pathsep) if p]
        else:
            _pxr = importlib.util.find_spec("pxr")
            if _pxr is not None and _pxr.submodule_search_locations:
                _pxr_dir = Path(next(iter(_pxr.submodule_search_locations)))  # <usd>/lib/python/pxr
                _dll_dirs += [_pxr_dir.parents[1], _pxr_dir.parents[2] / "bin"]
    for _libs in _dll_dirs:
        if _libs.is_dir():
            os.add_dll_directory(str(_libs))

# Windows can report the wheel's DLL path as \\?\C:\..., and OpenUSD's Windows glob reads
# that ? as a wildcard, so it finds none of usd/*/resources/; name the folder directly.
if sys.platform == "win32" and (_HERE / "usd" / "plugInfo.json").is_file():
    _plugin_paths = [str(_HERE / "usd"), os.environ.get("PXR_PLUGINPATH_NAME", "")]
    os.environ["PXR_PLUGINPATH_NAME"] = os.pathsep.join(p for p in _plugin_paths if p)

# Beside this file in a wheel, at the package root in the packman layout.
for _resources in (_HERE, _HERE.parent.parent):
    _schema = _resources / "omniSolid" / "resources"
    if _schema.is_dir():
        os.environ.setdefault("OMNISOLID_PLUGIN_PATH", str(_schema))
        break

# The module is packaged inside this package (wheel) or beside it on PYTHONPATH
# (build tree, packman). find_spec picks the layout without importing, so a real
# load failure still raises its own error.
if importlib.util.find_spec(f"{__name__}._omni_solid") is not None:
    from ._omni_solid import *  # noqa: F403
    from . import _omni_solid as _core
else:
    from _omni_solid import *  # noqa: F403
    import _omni_solid as _core

__all__ = [_name for _name in dir(_core) if not _name.startswith("_")]

# Register the binding submodules, otherwise the vendored `usd` resource
# directory shadows the `usd` submodule as a namespace package.
for _name in __all__:
    _attr = getattr(_core, _name)
    if isinstance(_attr, types.ModuleType):
        sys.modules[f"{__name__}.{_name}"] = _attr

# Register now: OpenUSD ignores schema plugins registered after a stage is opened.
if hasattr(_core, "usd") and os.path.isdir(os.environ.get("OMNISOLID_PLUGIN_PATH", "")):
    try:
        _core.usd.ensure_plugin_registered()
    except RuntimeError:
        pass  # reported again where the plugin is needed
