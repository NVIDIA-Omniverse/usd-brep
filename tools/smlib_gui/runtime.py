# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Runtime bootstrapping for the SMLib GUI."""

from __future__ import annotations

import os
import sys


PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(PACKAGE_DIR, "..", ".."))
SCRIPT_DIR = os.path.join(REPO, "tools", "scripts")

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import view_usd  # noqa: E402

view_usd._ensure_usd_runtime_env()

_CONFIG = view_usd._pick_config(REPO)
_PLATFORM = view_usd._pick_platform()
_BUILD_BIN = os.path.join(REPO, "_build", _PLATFORM, _CONFIG)
if os.path.isdir(_BUILD_BIN) and _BUILD_BIN not in sys.path:
    sys.path.insert(0, _BUILD_BIN)

try:
    import _omni_solid as sm  # noqa: E402
except ImportError:
    sm = None

try:
    import _smlib_dev as smdev  # noqa: E402
except ImportError:
    smdev = None

# pyvistaqt reaches Qt through qtpy; make it use the same binding as this package.
os.environ["QT_API"] = "pyside6"

import numpy as np  # noqa: E402
import pyvista as pv  # noqa: E402
from PySide6 import QtCore, QtGui, QtWidgets  # noqa: E402
from pyvistaqt import QtInteractor  # noqa: E402

__all__ = [
    "QtCore",
    "QtGui",
    "QtInteractor",
    "QtWidgets",
    "REPO",
    "SCRIPT_DIR",
    "np",
    "pv",
    "sm",
    "smdev",
]
