#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Launch the SMLib Python GUI.

When run as a script, this defaults to debug SMLib and USD binaries so kernel
``Dump()`` text and C++ breakpoints work. Pass ``--release`` to use the release
build instead. Importing this module (smoke tests, re-exports) still auto-picks
an existing build via ``view_usd._pick_config``.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

_REEXEC_ENV = "SMLIB_GUI_REEXEC"


def _pick_platform() -> str:
    if sys.platform == "win32":
        return "windows-x86_64"
    if sys.platform == "darwin":
        return "macos-universal"
    machine = platform.machine().lower()
    return "linux-aarch64" if machine in ("aarch64", "arm64") else "linux-x86_64"


def _prepend_env_path(name: str, paths: list[str]) -> None:
    existing = os.environ.get(name)
    filtered = [path for path in paths if path and os.path.isdir(path)]
    os.environ[name] = os.pathsep.join(filtered + ([existing] if existing else []))


def _apply_build_env(config: str) -> str:
    plat = _pick_platform()
    build_bin = os.path.join(REPO, "_build", plat, config)
    usd_lib = os.path.join(REPO, "_build", "target-deps", "usd", config, "lib")
    usd_bin = os.path.join(REPO, "_build", "target-deps", "usd", config, "bin")
    python_lib = os.path.join(REPO, "_build", "target-deps", "python", "lib")
    python_root = os.path.join(REPO, "_build", "target-deps", "python")
    plugin = os.path.join(REPO, "_build", "schema", "omniSolid", "resources")

    _prepend_env_path("PYTHONPATH", [build_bin, REPO])
    if sys.platform == "win32":
        _prepend_env_path("PATH", [build_bin, usd_lib, usd_bin, python_root])
        if hasattr(os, "add_dll_directory"):
            for directory in (build_bin, usd_lib, usd_bin, python_root):
                if os.path.isdir(directory):
                    try:
                        os.add_dll_directory(directory)
                    except (OSError, FileNotFoundError):
                        pass
    elif sys.platform == "darwin":
        _prepend_env_path("DYLD_LIBRARY_PATH", [build_bin, usd_lib, python_lib])
    else:
        _prepend_env_path("LD_LIBRARY_PATH", [build_bin, usd_lib, python_lib])

    if os.path.isdir(plugin):
        os.environ.setdefault("OMNISOLID_PLUGIN_PATH", plugin)

    os.environ["SMLIB_BUILD_CONFIG"] = config
    return build_bin


def _parse_launch_args(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Launch the SMLib Python GUI (debug SMLib/USD binaries by default)."
    )
    parser.add_argument(
        "--release",
        action="store_true",
        help="use release SMLib/USD binaries instead of the default debug build",
    )
    parser.add_argument(
        "--wait-for-lldb",
        action="store_true",
        help="print the PID and wait for Enter before importing the GUI",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify native module loading and exit without opening the GUI",
    )
    return parser.parse_known_args(argv)


def _import_native(name: str):
    try:
        return __import__(name)
    except ImportError as exc:
        print(f"smlib_gui: {name} unavailable: {exc}", flush=True)
        return None


def _print_native_module(name: str, module: object | None) -> None:
    if module is None:
        return
    print(f"smlib_gui: {name}={getattr(module, '__file__', '?')}", flush=True)


def _kernel_io_startup_note(config: str, runtime_info: dict | None) -> str:
    """One-time Dump / AssertValid I/O note for the launching terminal."""
    debug = config == "debug"
    flags = (runtime_info or {}).get("compile_flags") or {}
    if "SM_DEBUG_CODE" in flags:
        debug = bool(flags["SM_DEBUG_CODE"])
    if debug:
        return (
            "smlib_gui: debug SMLib echoes Dump() to stderr; AssertValid reports include file/line."
        )
    return (
        "smlib_gui: release SMLib does not echo Dump(); AssertValid still runs (no file/line labels)."
    )


def launch(argv: list[str] | None = None) -> int:
    args, remaining = _parse_launch_args(argv)
    config = "release" if args.release else "debug"
    build_bin = _apply_build_env(config)

    if not os.environ.get(_REEXEC_ENV):
        os.environ[_REEXEC_ENV] = "1"
        relaunch_args = list(argv) if argv is not None else list(sys.argv[1:])
        os.execv(sys.executable, [sys.executable, os.path.abspath(__file__), *relaunch_args])

    sys.argv = [sys.argv[0], *remaining]
    if build_bin not in sys.path:
        sys.path.insert(0, build_bin)
    if REPO not in sys.path:
        sys.path.insert(0, REPO)

    print(f"smlib_gui: config={config} pid={os.getpid()}", flush=True)
    print(f"smlib_gui: build_bin={build_bin}", flush=True)

    omni_solid = _import_native("_omni_solid")
    smlib_dev = _import_native("_smlib_dev")
    smlib_tests = _import_native("_smlib_tests")
    _print_native_module("_omni_solid", omni_solid)
    _print_native_module("_smlib_dev", smlib_dev)
    _print_native_module("_smlib_tests", smlib_tests)
    if omni_solid is None and config == "debug":
        print(
            "smlib_gui: debug binaries not found; build debug or pass --release",
            file=sys.stderr,
            flush=True,
        )
    runtime_info = None
    if smlib_tests is not None and hasattr(smlib_tests, "debug_runtime_info"):
        runtime_info = smlib_tests.debug_runtime_info()
        print(
            "smlib_gui: runtime=" + json.dumps(runtime_info, indent=2, sort_keys=True),
            flush=True,
        )
    print(_kernel_io_startup_note(config, runtime_info), flush=True)

    if args.check:
        def _loaded_from_config(name: str, module: object | None) -> bool:
            if module is None:
                print(f"smlib_gui: {name} did not load", file=sys.stderr, flush=True)
                return False
            loaded = os.path.normcase(os.path.realpath(getattr(module, "__file__", "") or ""))
            expected = os.path.normcase(os.path.realpath(build_bin))
            loaded = loaded.replace("\\", "/")
            expected = expected.rstrip("/\\").replace("\\", "/")
            if not loaded.startswith(expected + "/"):
                print(
                    f"smlib_gui: expected {config} {name} in {expected}, loaded {loaded}",
                    file=sys.stderr,
                    flush=True,
                )
                return False
            return True

        ok = _loaded_from_config("_omni_solid", omni_solid)
        ok = _loaded_from_config("_smlib_dev", smlib_dev) and ok
        return 0 if ok else 1

    if args.wait_for_lldb:
        input("Attach CodeLLDB to this Python process, then press Enter to open the GUI...")

    from tools.smlib_gui.app import main as gui_main  # noqa: PLC0415

    return int(gui_main())


if __name__ == "__main__":
    raise SystemExit(launch())
else:
    from tools.smlib_gui.app import MainWindow, main  # noqa: E402
    from tools.smlib_gui.history import SourceHistory  # noqa: E402
    from tools.smlib_gui.headless import (  # noqa: E402
        DEFAULT_VIEW_NAMES,
        HeadlessRenderResult,
        HeadlessRenderSession,
        HeadlessRenderSettings,
        SUPPORTED_VIEW_NAMES,
        discover_model_files,
        output_directory_for,
        write_contact_sheet,
    )
    from tools.smlib_gui.inspection import (  # noqa: E402
        append_mesh_summary as _append_mesh_summary,
        append_query as _append_query,
        face_stream_cell_count as _face_stream_cell_count,
        format_value as _format_value,
        object_summary_text,
    )
    from tools.smlib_gui.model import (  # noqa: E402
        ActiveObject,
        OBJECT_COLORS as _OBJECT_COLORS,
        legacy_parts_from_objects as _legacy_parts_from_objects,
        smlib_object_kind as _smlib_object_kind,
    )
    from tools.smlib_gui.operations.file_io import (  # noqa: E402
        LOAD_FILE_FILTER,
        SAVE_FILE_FILTER,
        SUPPORTED_LOAD_EXTENSIONS,
        occt_brep_io_available,
        brep_usd_io_available,
        ensure_extension_for_filter,
        export_native_brep,
        export_occt_brep,
        export_usd_objects,
        extension_for_filter,
        file_kind_for_path,
        import_file_objects,
        import_native_brep_object,
        import_occt_brep_objects,
        import_usd_objects,
        native_brep_file_is_ascii,
        native_brep_load_unavailable_reason,
    )
    from tools.smlib_gui.operations.booleans import (  # noqa: E402
        apply_boolean_operation,
        boolean_source_line,
    )
    from tools.smlib_gui.operations.modify import (  # noqa: E402
        ModifyResult,
        apply_modify_operation,
    )
    from tools.smlib_gui.operations.overlays import (  # noqa: E402
        BoxOverlay,
        CurveSampleOverlay,
        LineOverlay,
        NormalOverlay,
        PickOverlay,
        PointOverlay,
        PolylineOverlay,
        SectionOverlay,
        SurfaceSampleOverlay,
        TopologyOverlay,
        brep_edge_highlight_from_edge,
        brep_edgeuse_highlight,
        brep_edge_highlight,
        brep_edge_polylines,
        brep_face_highlight_from_face,
        brep_face_highlight,
        brep_face_normals,
        brep_face_surface_overlay,
        brep_loopuse_highlight,
        brep_vertex_highlight,
        make_overlay_object,
        sample_curve_overlay,
        sample_surface_overlay,
        section_overlay,
    )
    from tools.smlib_gui.operations.picking import (  # noqa: E402
        TopologyPick,
        topology_hits_for_ray,
    )
    from tools.smlib_gui.operations.test_runner import (  # noqa: E402
        DEBUG_GUIDANCE,
        PROG_TEST_SUITES,
        SMLIB_TESTS_UNAVAILABLE_HINT,
        draw_events_to_overlays,
        list_prog_test_suites,
        run_prog_test_suite,
        tests_available,
        unavailable_reason,
    )
    from tools.smlib_gui.prompts import DEFAULT_SCRIPT, prompt_to_source  # noqa: E402
    from tools.smlib_gui.runtime import (  # noqa: E402
        QtCore,
        QtGui,
        QtInteractor,
        QtWidgets,
        np,
        pv,
        sm,
        smdev,
    )
    from tools.smlib_gui.scripting import (  # noqa: E402
        CONSUMING_OPS as _CONSUMING_OPS,
        _split_args,
        consumed_variable_names as _consumed_variable_names,
        execute_script,
    )
    from tools.smlib_gui.settings import (  # noqa: E402
        DISPLAY_MODE_MESH_WIREFRAME,
        DISPLAY_MODE_OPTIONS,
        DISPLAY_MODE_SHADED,
        DISPLAY_MODE_SHADED_MESH_WIREFRAME,
        DISPLAY_MODE_SHADED_WIREFRAME,
        DISPLAY_MODE_WIREFRAME,
        DisplaySettings,
        TessellationSettings,
    )
    from tools.smlib_gui.tessellation import (  # noqa: E402
        objects_to_pyvista,
        objects_to_pyvista_batches,
        tessellate_objects,
    )
    from tools.smlib_gui.viewport import CameraState, ViewportController  # noqa: E402
    from tools.smlib_gui.widgets import PasteablePlainTextEdit as _PasteablePlainTextEdit  # noqa: E402

    __all__ = [
        "ActiveObject",
        "BoxOverlay",
        "CameraState",
        "CurveSampleOverlay",
        "DEFAULT_SCRIPT",
        "DEFAULT_VIEW_NAMES",
        "DEBUG_GUIDANCE",
        "DISPLAY_MODE_MESH_WIREFRAME",
        "DISPLAY_MODE_OPTIONS",
        "DISPLAY_MODE_SHADED",
        "DISPLAY_MODE_SHADED_MESH_WIREFRAME",
        "DISPLAY_MODE_SHADED_WIREFRAME",
        "DISPLAY_MODE_WIREFRAME",
        "DisplaySettings",
        "HeadlessRenderResult",
        "HeadlessRenderSession",
        "HeadlessRenderSettings",
        "MainWindow",
        "QtCore",
        "QtGui",
        "QtInteractor",
        "QtWidgets",
        "REPO",
        "TessellationSettings",
        "ViewportController",
        "_CONSUMING_OPS",
        "_OBJECT_COLORS",
        "_PasteablePlainTextEdit",
        "_append_mesh_summary",
        "_append_query",
        "_consumed_variable_names",
        "_face_stream_cell_count",
        "_format_value",
        "_legacy_parts_from_objects",
        "_smlib_object_kind",
        "_split_args",
        "apply_boolean_operation",
        "apply_modify_operation",
        "brep_usd_io_available",
        "boolean_source_line",
        "brep_edge_highlight_from_edge",
        "brep_edgeuse_highlight",
        "brep_edge_highlight",
        "brep_edge_polylines",
        "brep_face_highlight_from_face",
        "brep_face_highlight",
        "brep_face_normals",
        "brep_face_surface_overlay",
        "brep_loopuse_highlight",
        "brep_vertex_highlight",
        "export_native_brep",
        "export_occt_brep",
        "export_usd_objects",
        "execute_script",
        "discover_model_files",
        "import_native_brep_object",
        "import_file_objects",
        "import_occt_brep_objects",
        "import_usd_objects",
        "LineOverlay",
        "LOAD_FILE_FILTER",
        "main",
        "make_overlay_object",
        "ModifyResult",
        "NormalOverlay",
        "native_brep_load_unavailable_reason",
        "native_brep_file_is_ascii",
        "np",
        "occt_brep_io_available",
        "object_summary_text",
        "objects_to_pyvista",
        "objects_to_pyvista_batches",
        "output_directory_for",
        "PickOverlay",
        "PointOverlay",
        "PolylineOverlay",
        "PROG_TEST_SUITES",
        "prompt_to_source",
        "pv",
        "sample_curve_overlay",
        "sample_surface_overlay",
        "section_overlay",
        "SectionOverlay",
        "SAVE_FILE_FILTER",
        "SUPPORTED_LOAD_EXTENSIONS",
        "SUPPORTED_VIEW_NAMES",
        "sm",
        "smdev",
        "SourceHistory",
        "SurfaceSampleOverlay",
        "SMLIB_TESTS_UNAVAILABLE_HINT",
        "tessellate_objects",
        "TopologyOverlay",
        "TopologyPick",
        "topology_hits_for_ray",
        "draw_events_to_overlays",
        "ensure_extension_for_filter",
        "extension_for_filter",
        "file_kind_for_path",
        "list_prog_test_suites",
        "run_prog_test_suite",
        "tests_available",
        "unavailable_reason",
        "write_contact_sheet",
    ]
