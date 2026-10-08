#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Smoke test for ``tools/scripts/smlib_gui.py``.

This intentionally drives the existing GUI pipeline instead of duplicating its
geometry or viewport setup. It is a developer check for importability, window
construction, script execution, tessellation, and PyVista actor creation.

Usage:
    uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py
    uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py --show --timeout-ms 2000
    source ~/.bashrc && pyenv exec python tools/scripts/smlib_gui_smoke.py
    source ~/.bashrc && pyenv exec python tools/scripts/smlib_gui_smoke.py --show --timeout-ms 2000
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import logging
import os
import subprocess
import sys
import tempfile
import textwrap
import traceback
from pathlib import Path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

logger = logging.getLogger(__name__)

if "--show" not in sys.argv:
    os.environ.pop("DISPLAY", None)
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
    os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "smlib_gui_matplotlib"))
    os.environ.setdefault("MESA_SHADER_CACHE_DIR", os.path.join(tempfile.gettempdir(), "smlib_gui_mesa"))
    os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)
    os.makedirs(os.environ["MESA_SHADER_CACHE_DIR"], exist_ok=True)

import smlib_gui  # noqa: E402

from tools.smlib_gui.operations import file_io as gui_file_io  # noqa: E402
from tools.smlib_gui.operations import test_runner as gui_test_runner  # noqa: E402
from tools.smlib_gui.app import LEFT_PANEL_COLLAPSED_WIDTH, LEFT_PANEL_EXPANDED_WIDTH  # noqa: E402
from tools.smlib_gui import user_test_catalog as gui_user_tests  # noqa: E402
from tools.smlib_gui.settings import (  # noqa: E402
    COLOR_MODE_BREP_ORIENTATION,
    COLOR_MODE_MATERIAL,
    COLOR_MODE_OPTIONS,
    COLOR_MODE_VTK_BACKFACES,
    DISPLAY_MODES_WITH_BREP_WIREFRAME,
    DISPLAY_MODES_WITH_SHADED_SURFACE,
    NORMALS_MODE_MESH_FACE,
    NORMALS_MODE_MESH_VERTEX,
    NORMALS_MODE_OFF,
    TOPOLOGY_COUNTS_BREP,
    TOPOLOGY_COUNTS_BREP_DETAIL,
    TOPOLOGY_COUNTS_MESH,
)
from tools.smlib_gui.tess_presets import chord_tolerance_for_index  # noqa: E402

SMOKE_SOURCE = textwrap.dedent("""\
    box = sm.create_box((-5, -5, 0), 10, 10, 10)
    sm.fillet_edges(box, box.edges(), radius=1.0)
    sphere = sm.create_sphere((0, 0, 6), 4.0)
    """)

PICK_SOURCE = "pick_box = sm.create_box((0, 0, 0), 10, 10, 10)\n"
DEBUG_OVERLAY_COLOR = (1.0, 0.0, 0.0)


class SmokeFailure(RuntimeError):
    """Raised when the GUI initializes but does not reach the expected state."""


def _renderer_actor_count(plotter) -> int:
    """Return a best-effort actor count across PyVista renderer variants."""
    renderers = getattr(plotter, "renderers", None)
    if renderers is None:
        renderer_list = [getattr(plotter, "renderer", None)]
    else:
        try:
            renderer_list = list(renderers)
        except TypeError:
            renderer_list = [getattr(plotter, "renderer", None)]

    count = 0
    for renderer in renderer_list:
        if renderer is None:
            continue
        actors = getattr(renderer, "actors", None)
        if actors is None:
            actors = getattr(renderer, "_actors", None)
        if actors is None:
            continue
        try:
            count += len(actors)
        except TypeError:
            pass
    return count


def _mesh_counts(objects: list[object]) -> tuple[int, int]:
    total_points = 0
    total_faces = 0
    for obj in objects:
        mesh = obj.get("mesh") if isinstance(obj, dict) else getattr(obj, "mesh", None)
        if not mesh:
            continue
        total_points += len(mesh.get("points", ()))
        faces = mesh.get("faces", ())
        total_faces += len(faces)
    return total_points, total_faces


def _surface_mesh_actors(plotter) -> list:
    """Return surface mesh actors from the primary renderer."""
    renderer = getattr(plotter, "renderer", None)
    if renderer is None:
        return []
    actors = getattr(renderer, "actors", None) or getattr(renderer, "_actors", None) or {}
    try:
        return list(actors.values())
    except TypeError:
        return []


def _actor_backface_color(actor) -> tuple[float, float, float] | None:
    back = actor.GetBackfaceProperty() if hasattr(actor, "GetBackfaceProperty") else None
    if back is None:
        return None
    return tuple(float(component) for component in back.GetColor())


def _camera_position_focal(plotter) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    camera = plotter.camera
    position = tuple(float(value) for value in camera.GetPosition())
    focal = tuple(float(value) for value in camera.GetFocalPoint())
    return position, focal


def _camera_view_up(plotter) -> tuple[float, float, float]:
    return tuple(float(value) for value in plotter.camera.GetViewUp())


def _run_split_args_smoke() -> None:
    """Pin _split_args depth and quote handling for tricky argument strings."""
    cases = [
        ("(0, 0, 0), 10, [1, 2]", ["(0, 0, 0)", " 10", " [1, 2]"]),
        ('f(g(1, 2), {"k": 3}), 4', ['f(g(1, 2), {"k": 3})', " 4"]),
        ('"a\\",b", 2', ['"a\\",b"', " 2"]),
        ('"""a,b""", 2', ['"""a,b"""', " 2"]),
        ("'a,b', \"c,d\"", ["'a,b'", ' "c,d"']),
        ("1, 2,", ["1", " 2"]),
        ("", []),
    ]
    for source, expected in cases:
        actual = smlib_gui._split_args(source)
        if actual != expected:
            raise SmokeFailure(f"_split_args({source!r}) returned {actual!r}, expected {expected!r}")


def _run_user_test_catalog_smoke() -> None:
    """Pin Test.cpp-style numbered catalog parse, empty detection, and rewrite."""
    cases, error = gui_user_tests.load_user_tests()
    if error:
        raise SmokeFailure(f"default user_test.py failed to load: {error}")
    numbers = {case.number for case in cases}
    if 0 not in numbers or 1 not in numbers:
        raise SmokeFailure(f"default user_test.py is missing numbered cases: {sorted(numbers)}")
    empty = gui_user_tests.case_by_number(cases, 0)
    if empty is None or not gui_user_tests.script_is_empty(empty.source):
        raise SmokeFailure("user test 0 should be an empty placeholder")
    boxed = gui_user_tests.case_by_number(cases, 1)
    if boxed is None or "create_box" not in boxed.source or "boolean_union" not in boxed.source:
        raise SmokeFailure("user test 1 should string together sm.create_box and sm.boolean_union")
    dumped = gui_user_tests.case_by_number(cases, 2)
    if dumped is None or "smdev.dump" not in dumped.source or "smdev.assert_valid" not in dumped.source:
        raise SmokeFailure("user test 2 should string together smdev dump/assert_valid commands")
    kernel_example = gui_user_tests.case_by_number(cases, 3)
    if kernel_example is None or "smdev.user_test_example(" not in kernel_example.source:
        raise SmokeFailure("user test 3 should call smdev.user_test_example")
    kernel_stub = gui_user_tests.case_by_number(cases, 4)
    if kernel_stub is None or "smdev.user_test(" not in kernel_stub.source:
        raise SmokeFailure("user test 4 should call smdev.user_test")
    if not gui_user_tests.script_is_empty("pass") or not gui_user_tests.script_is_empty("# comment\n..."):
        raise SmokeFailure("script_is_empty did not treat pass/comments as empty")
    if gui_user_tests.script_is_empty("box = sm.create_box((0, 0, 0), 1, 1, 1)"):
        raise SmokeFailure("script_is_empty treated a create_box script as empty")

    original = Path(gui_user_tests.DEFAULT_USER_TEST_PATH).read_text(encoding="utf-8")
    updated = gui_user_tests.replace_user_test(
        original,
        99,
        "box = sm.create_box((0, 0, 0), 1, 1, 1)\nsmdev.dump(box)\n",
        title="smoke rewrite",
    )
    rewritten = gui_user_tests.parse_user_tests(updated)
    added = gui_user_tests.case_by_number(rewritten, 99)
    if added is None or added.title != "smoke rewrite":
        raise SmokeFailure("replace_user_test did not append a titled test_99")
    if "create_box" not in added.source or "smdev.dump" not in added.source:
        raise SmokeFailure("replace_user_test did not preserve sm/smdev body source")
    still_one = gui_user_tests.case_by_number(rewritten, 1)
    if still_one is None or still_one.source.strip() != boxed.source.strip():
        raise SmokeFailure("replace_user_test mutated an unrelated numbered case")
    replaced = gui_user_tests.replace_user_test(updated, 99, "sph = sm.create_sphere((0, 0, 0), 2.0)\n")
    replaced_case = gui_user_tests.case_by_number(gui_user_tests.parse_user_tests(replaced), 99)
    if replaced_case is None or "create_sphere" not in replaced_case.source:
        raise SmokeFailure("replace_user_test did not update an existing numbered case")
    if replaced_case.title != "smoke rewrite":
        raise SmokeFailure("replace_user_test did not preserve the existing docstring title")
    try:
        gui_user_tests.replace_user_test(original, 1, "if True\n")
    except SyntaxError:
        pass
    else:
        raise SmokeFailure("replace_user_test did not reject invalid rewritten catalog syntax")
    with tempfile.TemporaryDirectory(prefix="smlib_gui_user_test_save_") as tmpdir:
        catalog_path = os.path.join(tmpdir, "user_test.py")
        Path(catalog_path).write_text(original, encoding="utf-8")
        save_error = gui_user_tests.save_user_test(1, "if True\n", path=catalog_path)
        if not save_error:
            raise SmokeFailure("save_user_test wrote invalid catalog syntax")
        if Path(catalog_path).read_text(encoding="utf-8") != original:
            raise SmokeFailure("save_user_test overwrote the catalog after a syntax error")


def _run_user_test_gui_smoke(window, app) -> None:
    """Play a numbered user test through the GUI editor and history replay."""
    if getattr(window, "_source_edit", None) is None:
        raise SmokeFailure("user-test source editor was not created")
    window._set_left_panel_collapsed(False, persist=False)
    app.processEvents()
    if window._user_test_error:
        raise SmokeFailure(f"user tests failed to load: {window._user_test_error}")
    if window._user_test_combo.count() < 2:
        raise SmokeFailure("user-test combo did not list numbered cases")

    window._set_user_test_number(0)
    app.processEvents()
    if not gui_user_tests.script_is_empty(window._source_edit.toPlainText()):
        raise SmokeFailure("selecting user test 0 did not load the empty placeholder")
    if not window._on_run_user_test():
        raise SmokeFailure(f"empty user test 0 failed; status={window._status_bar.text()!r}")
    if window._objects:
        raise SmokeFailure("empty user test 0 should leave the scene unchanged")

    window._set_user_test_number(1)
    app.processEvents()
    source = window._source_edit.toPlainText()
    if "create_box" not in source:
        raise SmokeFailure("selecting user test 1 did not load sm.create_box source")
    if not window._on_run_user_test():
        raise SmokeFailure(f"user test 1 playback failed; status={window._status_bar.text()!r}")
    if not window._objects:
        raise SmokeFailure("user test 1 produced no active objects")
    played_count = len(window._objects)

    window._set_user_test_number(0)
    app.processEvents()
    if not window._on_run_user_test():
        raise SmokeFailure(f"replaying empty user test 0 failed; status={window._status_bar.text()!r}")
    if len(window._objects) != played_count:
        raise SmokeFailure("empty user test 0 cleared objects created by user test 1")

    if window._source_history.current() is None:
        raise SmokeFailure("user test playback did not record source history")
    window._on_replay_source_history()
    app.processEvents()
    if not window._objects:
        raise SmokeFailure("source history replay did not restore user-test objects")

    edited = "edited_box = sm.create_box((1, 2, 3), 4, 5, 6)\n"
    window._source_edit.setPlainText(edited)
    app.processEvents()
    if window._user_test_overrides.get(window._current_user_test_number()) != edited:
        raise SmokeFailure("editing user-test source did not keep an in-memory override")
    if not window._on_run_user_test():
        raise SmokeFailure(f"edited user-test playback failed; status={window._status_bar.text()!r}")
    names = {obj.name for obj in window._objects}
    if "edited_box" not in names:
        raise SmokeFailure(f"edited user test did not execute editor contents; objects={names}")

    with tempfile.TemporaryDirectory(prefix="smlib_gui_user_test_") as tmpdir:
        catalog_path = os.path.join(tmpdir, "user_test.py")
        Path(catalog_path).write_text(
            Path(gui_user_tests.DEFAULT_USER_TEST_PATH).read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        previous_path = window._user_test_path
        window._user_test_path = catalog_path
        try:
            window._reload_user_tests()
            window._set_user_test_number(4)
            window._source_edit.setPlainText("saved_box = sm.create_box((0, 0, 0), 2, 2, 2)\n")
            if not window._on_save_user_test():
                raise SmokeFailure(f"saving user test 4 failed; status={window._status_bar.text()!r}")
            saved_cases, saved_error = gui_user_tests.load_user_tests(catalog_path)
            if saved_error:
                raise SmokeFailure(f"saved user_test.py failed to reload: {saved_error}")
            saved = gui_user_tests.case_by_number(saved_cases, 4)
            if saved is None or "saved_box" not in saved.source:
                raise SmokeFailure("Save did not write test_4 into the catalog file")
        finally:
            window._user_test_path = previous_path
            window._reload_user_tests()

    window._set_user_test_number(4)
    app.processEvents()
    if "smdev.user_test(" not in window._source_edit.toPlainText():
        raise SmokeFailure("selecting user test 4 did not load smdev.user_test source")
    if not window._on_run_user_test():
        raise SmokeFailure(f"user test 4 stub failed; status={window._status_bar.text()!r}")

    if not window._execute_and_display("example_box = smdev.user_test_example()\n"):
        raise SmokeFailure(
            f"smdev.user_test_example playback failed; status={window._status_bar.text()!r}"
        )
    example_names = {obj.name for obj in window._objects}
    if "example_box" not in example_names:
        raise SmokeFailure(
            f"user_test_example produced no tessellatable Brep; objects={example_names}"
        )

    # A standalone Curve has no display path. Drive it through _on_run_user_test, which posts
    # its own success text over the status-bar summary, and confirm the reason still reaches
    # the user through the object inspector rather than leaving an unexplained empty viewport.
    window._source_edit.setPlainText("stray_curve = sm.create_line_segment((0, 0, 0), (10, 0, 0))\n")
    app.processEvents()
    if not window._on_run_user_test():
        raise SmokeFailure(
            f"standalone curve playback failed; status={window._status_bar.text()!r}"
        )
    curve_objects = [obj for obj in window._objects if obj.kind == "Curve"]
    if not curve_objects:
        raise SmokeFailure(
            f"expected a standalone Curve; objects={[(o.name, o.kind) for o in window._objects]}"
        )
    if "no display path" not in curve_objects[0].mesh_error:
        raise SmokeFailure(
            f"standalone Curve carries no display diagnostic; mesh_error={curve_objects[0].mesh_error!r}"
        )
    curve_summary = window._object_info.toPlainText()
    if "no display path" not in curve_summary:
        raise SmokeFailure(
            f"object inspector omits the display diagnostic; summary={curve_summary!r}"
        )


def _assert_left_panel_width(window, expected: int, *, collapsed: bool) -> None:
    panel = window._left_panel
    if panel.minimumWidth() != expected or panel.maximumWidth() != expected:
        raise SmokeFailure(
            "left user-test panel width mismatch; "
            f"collapsed={collapsed} expected={expected} "
            f"min={panel.minimumWidth()} max={panel.maximumWidth()}"
        )
    if window._left_panel_collapsed != collapsed:
        raise SmokeFailure(
            f"left user-test panel collapsed flag mismatch; expected={collapsed} "
            f"actual={window._left_panel_collapsed}"
        )
    if window._left_panel_body.isHidden() != collapsed:
        raise SmokeFailure(
            "left user-test panel body visibility mismatch; "
            f"collapsed={collapsed} hidden={window._left_panel_body.isHidden()}"
        )
    if window._user_test_title.isHidden() != collapsed:
        raise SmokeFailure(
            "left user-test title visibility mismatch; "
            f"collapsed={collapsed} hidden={window._user_test_title.isHidden()}"
        )
    if window._left_panel_spine_label.isHidden() == collapsed:
        raise SmokeFailure(
            "collapsed left-panel spine label visibility mismatch; "
            f"collapsed={collapsed} hidden={window._left_panel_spine_label.isHidden()}"
        )
    if window._left_panel_spine_label.text() != "User Tests":
        raise SmokeFailure(
            "collapsed left-panel spine label text mismatch; "
            f"text={window._left_panel_spine_label.text()!r}"
        )


def _run_left_panel_collapse_smoke(window, app) -> None:
    """Collapse the user-test strip by width without using a right-panel QGroupBox."""
    if getattr(window, "_left_panel", None) is None:
        raise SmokeFailure("left user-test panel was not created")
    if getattr(window, "_left_panel_body", None) is None:
        raise SmokeFailure("left user-test panel body was not created")
    if getattr(window, "_left_panel_toggle", None) is None:
        raise SmokeFailure("left user-test panel toggle was not created")
    if getattr(window, "_left_panel_spine_label", None) is None:
        raise SmokeFailure("left user-test spine label was not created")

    original = window._saved_left_panel_collapsed()
    try:
        window._set_left_panel_collapsed(False, persist=False)
        app.processEvents()
        _assert_left_panel_width(window, LEFT_PANEL_EXPANDED_WIDTH, collapsed=False)

        window._set_left_panel_collapsed(True, persist=False)
        app.processEvents()
        _assert_left_panel_width(window, LEFT_PANEL_COLLAPSED_WIDTH, collapsed=True)
        if window._source_edit.isVisibleTo(window._left_panel):
            raise SmokeFailure("source editor remained shown after collapsing the left panel")

        window._left_panel_toggle.click()
        app.processEvents()
        _assert_left_panel_width(window, LEFT_PANEL_EXPANDED_WIDTH, collapsed=False)
        if not window._source_edit.isVisibleTo(window._left_panel):
            raise SmokeFailure("source editor stayed hidden after expanding the left panel")
    finally:
        window._set_left_panel_collapsed(False, persist=False)
        window._persist_left_panel_collapsed(original)


def _run_mesh_batch_smoke() -> None:
    color = (0.3, 0.6, 0.95)
    objects = []
    for object_id, x_offset in ((101, 0.0), (202, 2.0)):
        points = smlib_gui.np.asarray(
            [
                (x_offset, 0.0, 0.0),
                (x_offset + 1.0, 0.0, 0.0),
                (x_offset, 1.0, 0.0),
            ],
            dtype=smlib_gui.np.float32,
        )
        objects.append(
            smlib_gui.ActiveObject(
                object_id=object_id,
                name=f"batch_{object_id}",
                kind="PolyBrep",
                handle=object(),
                color=color,
                mesh={
                    "points": points,
                    "normals": smlib_gui.np.tile(
                        smlib_gui.np.asarray((0.0, 0.0, 1.0), dtype=smlib_gui.np.float32),
                        (3, 1),
                    ),
                    "faces": smlib_gui.np.asarray((3, 0, 1, 2), dtype=smlib_gui.np.int32),
                },
            )
        )

    batches = smlib_gui.objects_to_pyvista_batches(objects)
    if len(batches) != 1:
        raise SmokeFailure(f"same-color display meshes were not batched: {len(batches)}")
    batch = batches[0]
    if batch.n_points != 6 or batch.n_cells != 2:
        raise SmokeFailure(f"batched display geometry mismatch: points={batch.n_points} cells={batch.n_cells}")
    object_ids = set(int(value) for value in batch.cell_data["ObjectID"])
    if object_ids != {101, 202}:
        raise SmokeFailure(f"batched display object IDs were not preserved: {object_ids}")


def _button_texts(window) -> list[str]:
    return [button.text() for button in window.findChildren(smlib_gui.QtWidgets.QPushButton)]


def _enabled_topology_buttons(window) -> set[str]:
    return {key for key, button in window._topology_action_buttons.items() if button.isEnabled()}


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = sum(value * value for value in vector) ** 0.5
    if length <= 0.0:
        return (0.0, 0.0, 1.0)
    return tuple(value / length for value in vector)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _topology_ray_from_point(obj, point: tuple[float, float, float]):
    center = obj.handle.center()
    mn, mx = obj.handle.bounding_box()
    diagonal = ((mx[0] - mn[0]) ** 2 + (mx[1] - mn[1]) ** 2 + (mx[2] - mn[2]) ** 2) ** 0.5
    outward = _normalize(
        (
            point[0] - center[0],
            point[1] - center[1],
            point[2] - center[2],
        )
    )
    origin = (
        point[0] + outward[0] * max(diagonal, 1.0),
        point[1] + outward[1] * max(diagonal, 1.0),
        point[2] + outward[2] * max(diagonal, 1.0),
    )
    direction = (-outward[0], -outward[1], -outward[2])
    return origin, direction


def _assert_topology_buttons(window, expected: set[str]) -> None:
    actual = _enabled_topology_buttons(window)
    if actual != expected:
        raise SmokeFailure(f"topology action buttons mismatch; expected={expected} actual={actual}")


def _assert_dump_wrote(window, action) -> None:
    with contextlib.redirect_stdout(io.StringIO()):
        result = action()
    if result is None:
        raise SmokeFailure(f"Dump failed; status={window._status_bar.text()!r}")
    text = window._object_info.toPlainText()
    if not text.startswith("Dump:"):
        raise SmokeFailure(f"Dump inspector missing target label: {text!r}")
    if "smdev.dump(" not in window._current_source:
        raise SmokeFailure("Dump did not record replayable source")


def _assert_valid_passed(window, action) -> None:
    with contextlib.redirect_stdout(io.StringIO()):
        result = action()
    if result is None or not getattr(result, "ok", False):
        raise SmokeFailure(f"AssertValid failed; status={window._status_bar.text()!r}")
    text = window._object_info.toPlainText()
    if not text.startswith("AssertValid:"):
        raise SmokeFailure(f"AssertValid inspector missing target label: {text!r}")
    if "smdev.assert_valid_reports(" not in window._current_source:
        raise SmokeFailure("AssertValid did not record replayable source")


def _kernel_draw_available() -> bool:
    import _smlib_dev as smdev

    if not smdev.tests.available():
        return True
    flags = smdev.tests.debug_runtime_info().get("compile_flags", {})
    return bool(flags.get("SM_GFX_OUTPUT_CODE", True))


def _assert_overlay_metadata(overlay, topology_kind: str) -> None:
    if overlay is None or overlay.kind not in {"Topology", "DisplayList"}:
        raise SmokeFailure(f"{topology_kind} draw did not create a topology/display-list overlay")
    metadata = getattr(overlay.handle, "metadata", {})
    if metadata.get("topology_kind") != topology_kind:
        raise SmokeFailure(f"{topology_kind} overlay metadata mismatch: {metadata}")
    if tuple(float(component) for component in overlay.color) != DEBUG_OVERLAY_COLOR:
        raise SmokeFailure(f"{topology_kind} overlay is not red: {overlay.color}")


def _assert_draw_cycle(first_overlay, second_overlay, topology_kind: str, expected_count: int) -> None:
    _assert_overlay_metadata(first_overlay, topology_kind)
    _assert_overlay_metadata(second_overlay, topology_kind)
    first_metadata = getattr(first_overlay.handle, "metadata", {})
    second_metadata = getattr(second_overlay.handle, "metadata", {})
    if first_metadata.get("candidate_count") != expected_count:
        raise SmokeFailure(f"{topology_kind} first cycle count mismatch: {first_metadata}")
    if second_metadata.get("candidate_count") != expected_count:
        raise SmokeFailure(f"{topology_kind} second cycle count mismatch: {second_metadata}")
    if expected_count > 1:
        if first_metadata.get("candidate_index") == second_metadata.get("candidate_index"):
            raise SmokeFailure(f"{topology_kind} repeated draw did not cycle: {first_metadata} {second_metadata}")


def _assert_transient_replaced(window, first_overlay, second_overlay, topology_kind: str) -> None:
    if any(obj is first_overlay for obj in window._objects):
        raise SmokeFailure(f"{topology_kind} first transient overlay was not removed")
    if not any(obj is second_overlay for obj in window._objects):
        raise SmokeFailure(f"{topology_kind} replacement transient overlay is not active")
    active = [
        obj
        for obj in window._objects
        if obj.kind in {"Topology", "DisplayList"}
        and getattr(obj.handle, "metadata", {}).get("topology_kind") == topology_kind
    ]
    if len(active) != 1 or active[0] is not second_overlay:
        raise SmokeFailure(f"{topology_kind} active transient overlays mismatch: {active}")


def _assert_no_active_topology_overlays(window, topology_kinds: set[str]) -> None:
    active = [
        obj
        for obj in window._objects
        if obj.kind in {"Topology", "DisplayList"}
        and getattr(obj.handle, "metadata", {}).get("topology_kind") in topology_kinds
    ]
    if active:
        raise SmokeFailure(f"unexpected active transient topology overlays: {active}")


def _run_usd_schema_discovery_smoke() -> None:
    previous_plugin_path = os.environ.get("OMNISOLID_PLUGIN_PATH")
    try:
        with tempfile.TemporaryDirectory(prefix="smlib_gui_schema_") as tmpdir:
            invalid_resource_dir = os.path.join(tmpdir, "omniSolid", "resources")
            os.makedirs(invalid_resource_dir, exist_ok=True)
            with open(os.path.join(invalid_resource_dir, "generatedSchema.usda"), "w", encoding="utf-8") as file:
                file.write("version https://git-lfs.github.com/spec/v1\n")

            os.environ["OMNISOLID_PLUGIN_PATH"] = invalid_resource_dir
            resolved = gui_file_io.omnisolid_plugin_path()
            if not resolved:
                raise SmokeFailure("OmniSolid schema discovery did not recover from an invalid env path")
            if os.path.abspath(resolved) == os.path.abspath(invalid_resource_dir):
                raise SmokeFailure("OmniSolid schema discovery accepted an invalid generatedSchema.usda")
            if not gui_file_io._valid_omnisolid_resource_dir(resolved):
                raise SmokeFailure(f"OmniSolid schema discovery returned an invalid path: {resolved}")
            if os.environ.get("OMNISOLID_PLUGIN_PATH") != resolved:
                raise SmokeFailure("OmniSolid schema discovery did not update OMNISOLID_PLUGIN_PATH")
    finally:
        if previous_plugin_path is None:
            os.environ.pop("OMNISOLID_PLUGIN_PATH", None)
        else:
            os.environ["OMNISOLID_PLUGIN_PATH"] = previous_plugin_path


def _run_in_process_test_smoke(window, app) -> None:
    expected_suites = (
        "topology",
        "booleans",
        "offset",
        "fillets",
        "local-ops",
        "sweeps",
        "primitives",
        "tessellation",
        "unit",
        "trimmed-surfaces",
        "brep-import",
        "ssi-analytic",
        "ssi-advanced",
        "cci-advanced",
        "section-advanced",
        "silhouette-advanced",
        "stitch",
    )
    if tuple(smlib_gui.PROG_TEST_SUITES) != expected_suites:
        raise SmokeFailure(f"prog_test suite manifest mismatch: {smlib_gui.PROG_TEST_SUITES}")

    combo_suites = [window._test_suite_combo.itemData(index) for index in range(window._test_suite_combo.count())]
    if tuple(combo_suites) != expected_suites:
        raise SmokeFailure(f"Tests suite combo mismatch: {combo_suites}")
    if "Run In GUI" not in _button_texts(window):
        raise SmokeFailure("Tests group Run In GUI button is missing")

    class FakeSmlibTests:
        def __init__(self):
            self.calls: list[tuple[str, str, bool]] = []

        def list_prog_test_suites(self):
            return list(expected_suites)

        def run_prog_test_suite(self, name, working_directory="", do_graphics=False):
            if window._run_test_btn.isEnabled() or window._test_suite_combo.isEnabled():
                raise SmokeFailure("Tests run controls were not disabled while a suite was active")
            if name not in expected_suites:
                raise ValueError(f"Unknown suite: {name}")
            self.calls.append((name, working_directory, bool(do_graphics)))
            return {
                "suite_name": name,
                "status": 1000,
                "status_name": "SM_SUCCESS",
                "ok": True,
                "elapsed_seconds": 0.012,
                "log": "fake booleans log",
                "draw_events": [
                    {
                        "type": "display_list",
                        "name": "fake_prog_test_draw",
                        "batches": [
                            {
                                "primitive": "line",
                                "positions": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
                                "line_width": 2.0,
                                "color": (1.0, 0.0, 0.0),
                                "connected": True,
                            }
                        ],
                        "metadata": {"source": "prog_test"},
                    }
                ],
            }

    class FakeTestsFacade:
        def __init__(self, backend: FakeSmlibTests):
            self._backend = backend
            self._available = True

        def available(self) -> bool:
            return self._available

        def list_prog_test_suites(self):
            return self._backend.list_prog_test_suites()

        def run_prog_test_suite(self, name, working_directory="", do_graphics=False):
            return self._backend.run_prog_test_suite(name, working_directory, do_graphics)

    class FakeSmdev:
        def __init__(self, tests: FakeTestsFacade):
            self.tests = tests

    original_module = gui_test_runner._smlib_tests
    original_error = gui_test_runner._import_error
    original_smdev = gui_test_runner.smdev
    fake_module = FakeSmlibTests()
    fake_tests = FakeTestsFacade(fake_module)
    fake_smdev = FakeSmdev(fake_tests)
    booleans_index = next(
        (
            index
            for index in range(window._test_suite_combo.count())
            if window._test_suite_combo.itemData(index) == "booleans"
        ),
        -1,
    )
    if booleans_index < 0:
        raise SmokeFailure("booleans suite is not selectable in the Tests group")

    try:
        gui_test_runner._smlib_tests = fake_module
        gui_test_runner._import_error = None
        gui_test_runner.smdev = fake_smdev
        window._test_suite_combo.setCurrentIndex(booleans_index)
        window._refresh_test_panel()
        app.processEvents()
        if not window._run_test_btn.isEnabled():
            raise SmokeFailure("Run In GUI is disabled when prog_test runner is available")
        result = window._on_run_test_suite()
        app.processEvents()
        if result is None or not result.get("ok"):
            raise SmokeFailure(f"fake prog_test run did not return a passing result: {window._test_log.toPlainText()}")
        if fake_module.calls != [("booleans", gui_test_runner.PROG_TEST_WORKING_DIRECTORY, True)]:
            raise SmokeFailure(f"fake prog_test call mismatch: {fake_module.calls}")
        log_text = window._test_log.toPlainText()
        for needle in ("Suite booleans passed", "fake booleans log", "Draw events displayed: 1"):
            if needle not in log_text:
                raise SmokeFailure(f"Tests run log missing {needle!r}: {log_text}")
        if not any(obj.kind == "DisplayList" and obj.name == "fake_prog_test_draw" for obj in window._objects):
            raise SmokeFailure("Tests draw event did not create a display-list overlay")

        fake_tests._available = False
        gui_test_runner._smlib_tests = None
        gui_test_runner._import_error = ImportError("smoke missing _smlib_tests")
        gui_test_runner.smdev = None
        window._refresh_test_panel()
        app.processEvents()
        if window._run_test_btn.isEnabled():
            raise SmokeFailure("Run In GUI is enabled when prog_test runner is unavailable")
        missing_log = window._test_log.toPlainText()
        if "In-process prog_test runner unavailable" not in missing_log:
            raise SmokeFailure(f"Missing _smlib_tests state was not shown: {missing_log}")
    finally:
        gui_test_runner._smlib_tests = original_module
        gui_test_runner._import_error = original_error
        gui_test_runner.smdev = original_smdev
        window._refresh_test_panel()


def _run_topology_pick_smoke(window, app) -> None:
    if "Pick" in _button_texts(window):
        raise SmokeFailure("debug Pick button should not be exposed")
    removed_debug_buttons = {"Normal", "Curve", "Surface", "Face", "Edge", "Vertex"}
    exposed_removed = removed_debug_buttons.intersection(_button_texts(window))
    if exposed_removed:
        raise SmokeFailure(f"removed debug buttons are still exposed: {sorted(exposed_removed)}")

    if not window._execute_and_display(PICK_SOURCE):
        raise SmokeFailure(f"pick smoke source failed; status={window._status_bar.text()!r}")
    app.processEvents()
    if len(window._objects) != 1 or window._objects[0].kind != "Brep":
        raise SmokeFailure("pick smoke source did not create one BRep")
    obj = window._objects[0]
    if smlib_gui.smdev is None:
        raise SmokeFailure("_smlib_dev is not available for topology pick smoke")
    _assert_valid_passed(window, window._on_assert_valid_object)
    _, mx = obj.handle.bounding_box()
    center = obj.handle.center()

    face_ray_origin = (center[0], center[1], mx[2] + 20.0)
    face_ray_direction = (0.0, 0.0, -1.0)
    face_hits = smlib_gui.topology_hits_for_ray(window._visible_objects(), face_ray_origin, face_ray_direction)
    if len(face_hits) < 2:
        raise SmokeFailure(f"topology ray should hit at least two box faces; hits={face_hits}")
    face_selection = window._select_topology_from_ray(
        face_ray_origin,
        face_ray_direction,
        screen_pos=(50.0, 50.0),
    )
    if face_selection is None or face_selection.topology_kind != "Face":
        raise SmokeFailure(f"face topology pick failed: {face_selection}")
    _assert_topology_buttons(window, {"face", "surface", "loopuse", "dump", "assert_valid"})
    _assert_dump_wrote(window, window._on_dump_topology)
    _assert_valid_passed(window, window._on_assert_valid_topology)
    face_loopuse_count = len(window._loopuse_candidates_for_selection(face_selection))
    if face_loopuse_count <= 0:
        raise SmokeFailure("face selection did not expose attached loopuse candidates")
    if not window._viewport._topology_pick_marker_actors:
        raise SmokeFailure("topology pick did not draw a selected-location marker")

    deeper_selection = window._select_topology_from_ray(
        face_ray_origin,
        face_ray_direction,
        screen_pos=(52.0, 51.0),
    )
    if deeper_selection is None or deeper_selection.ray_depth <= face_selection.ray_depth:
        raise SmokeFailure("same-location topology pick did not cycle to a deeper hit")

    window._set_topology_selection(face_selection)
    kernel_draw = _kernel_draw_available()
    face_overlay = window._add_picked_face_overlay()
    surface_overlay = window._add_picked_surface_overlay()
    _assert_overlay_metadata(face_overlay, "face")
    if kernel_draw:
        _assert_overlay_metadata(surface_overlay, "surface")
        if surface_overlay.kind != "DisplayList":
            raise SmokeFailure(f"draw surface did not use kernel display-list batches: {surface_overlay.kind}")
        if surface_overlay.handle.metadata.get("draw_variant") != "draw_uv":
            raise SmokeFailure(f"surface overlay did not use DrawUV: {surface_overlay.handle.metadata}")
    else:
        if surface_overlay.kind != "SurfaceSample":
            raise SmokeFailure(f"draw surface did not fall back to a sampled surface: {surface_overlay.kind}")
        if "SM_GFX_OUTPUT_CODE" not in window._status_bar.text():
            raise SmokeFailure(
                f"missing kernel Draw unavailable status: {window._status_bar.text()!r}"
            )
    if surface_overlay.handle.metadata.get("face_index") != face_selection.face_index:
        raise SmokeFailure("surface overlay did not record the picked face index")
    if window._viewport._batch_color({"color": (0.0, 0.0, 0.0)}, surface_overlay.color) != DEBUG_OVERLAY_COLOR:
        raise SmokeFailure("display-list renderer did not override black kernel batch color")
    face_loopuse_overlay = window._add_picked_loopuse_overlay()
    face_loopuse_overlay_2 = window._add_picked_loopuse_overlay()
    _assert_overlay_metadata(face_loopuse_overlay, "loopuse")
    _assert_draw_cycle(face_loopuse_overlay, face_loopuse_overlay_2, "loopuse", face_loopuse_count)
    _assert_transient_replaced(window, face_loopuse_overlay, face_loopuse_overlay_2, "loopuse")
    if face_loopuse_overlay.handle.metadata.get("selection_kind") != "Face":
        raise SmokeFailure(f"face loopuse overlay source mismatch: {face_loopuse_overlay.handle.metadata}")

    edge = obj.handle.edges()[0]
    start, end = edge.vertices()
    edge_midpoint = tuple(0.5 * (a + b) for a, b in zip(start.point(), end.point()))
    edge_ray_origin, edge_ray_direction = _topology_ray_from_point(obj, edge_midpoint)
    edge_direction = _normalize(tuple(b - a for a, b in zip(start.point(), end.point())))
    offset_axis = _normalize(_cross(edge_ray_direction, edge_direction))
    edge_ray_origin = tuple(origin + 0.15 * offset for origin, offset in zip(edge_ray_origin, offset_axis))
    edge_selection = window._select_topology_from_ray(
        edge_ray_origin,
        edge_ray_direction,
        screen_pos=(200.0, 50.0),
    )
    if edge_selection is None or edge_selection.topology_kind != "Edge":
        raise SmokeFailure(f"edge topology pick failed: {edge_selection}")
    _assert_topology_buttons(window, {"edge", "curve", "edgeuse", "loopuse", "dump", "assert_valid"})
    if not edge_selection.edgeuses:
        raise SmokeFailure("edge selection did not record edgeuse candidates")
    if not edge_selection.loopuses:
        raise SmokeFailure("edge selection did not record loopuse candidates")

    edge_overlay = window._add_picked_edge_overlay()
    curve_overlay = window._add_picked_curve_overlay()
    edgeuse_overlay = window._add_picked_edgeuse_overlay()
    edgeuse_overlay_2 = window._add_picked_edgeuse_overlay()
    loopuse_overlay = window._add_picked_loopuse_overlay()
    loopuse_overlay_2 = window._add_picked_loopuse_overlay()
    _assert_overlay_metadata(edge_overlay, "edge")
    if kernel_draw:
        _assert_overlay_metadata(curve_overlay, "curve")
        if curve_overlay.kind != "DisplayList":
            raise SmokeFailure(f"draw curve did not use kernel display-list batches: {curve_overlay.kind}")
    else:
        if curve_overlay.kind != "CurveSample":
            raise SmokeFailure(f"draw curve did not fall back to a sampled curve: {curve_overlay.kind}")
        if curve_overlay.handle.metadata.get("topology_kind") != "curve":
            raise SmokeFailure(f"curve overlay metadata mismatch: {curve_overlay.handle.metadata}")
    _assert_overlay_metadata(edgeuse_overlay, "edgeuse")
    _assert_overlay_metadata(loopuse_overlay, "loopuse")
    if curve_overlay.handle.metadata.get("curve_source") != "edge":
        raise SmokeFailure(f"curve overlay did not record edge curve source: {curve_overlay.handle.metadata}")
    _assert_draw_cycle(edgeuse_overlay, edgeuse_overlay_2, "edgeuse", len(edge_selection.edgeuses))
    _assert_draw_cycle(loopuse_overlay, loopuse_overlay_2, "loopuse", len(edge_selection.loopuses))
    _assert_transient_replaced(window, edgeuse_overlay, edgeuse_overlay_2, "edgeuse")
    _assert_transient_replaced(window, loopuse_overlay, loopuse_overlay_2, "loopuse")
    if kernel_draw:
        if edgeuse_overlay.kind != "DisplayList":
            raise SmokeFailure(f"edgeuse draw did not use kernel display-list batches: {edgeuse_overlay.kind}")
        if getattr(edgeuse_overlay.handle, "metadata", {}).get("source") != "kernel_draw":
            raise SmokeFailure(f"edgeuse display-list source mismatch: {edgeuse_overlay.handle.metadata}")
        if len(getattr(edgeuse_overlay.handle, "batches", [])) <= 1:
            raise SmokeFailure("edgeuse display-list did not include richer kernel draw markers")
    else:
        if edgeuse_overlay.kind != "Topology":
            raise SmokeFailure(f"edgeuse draw did not fall back to a sampled highlight: {edgeuse_overlay.kind}")
        if "SM_GFX_OUTPUT_CODE" not in window._status_bar.text():
            raise SmokeFailure(
                f"missing kernel Draw unavailable status: {window._status_bar.text()!r}"
            )
    if "direction_start" not in edgeuse_overlay.handle.metadata:
        raise SmokeFailure("edgeuse overlay did not include direction marker metadata")

    vertex_point = obj.handle.vertices()[0].point()
    vertex_ray_origin, vertex_ray_direction = _topology_ray_from_point(obj, vertex_point)
    vertex_selection = window._select_topology_from_ray(
        vertex_ray_origin,
        vertex_ray_direction,
        screen_pos=(350.0, 50.0),
    )
    if vertex_selection is None or vertex_selection.topology_kind != "Vertex":
        raise SmokeFailure(f"vertex topology pick failed: {vertex_selection}")
    _assert_no_active_topology_overlays(window, {"edgeuse", "loopuse"})
    _assert_topology_buttons(window, {"edge", "edgeuse", "dump", "assert_valid"})
    vertex_edge_count = len(window._edge_candidates_for_selection(vertex_selection))
    vertex_edgeuse_count = len(window._edgeuse_candidates_for_selection(vertex_selection))
    if vertex_edge_count <= 0 or vertex_edgeuse_count <= 0:
        raise SmokeFailure("vertex selection did not expose attached edge/edgeuse candidates")
    vertex_edge_overlay = window._add_picked_edge_overlay()
    vertex_edgeuse_overlay = window._add_picked_edgeuse_overlay()
    vertex_edgeuse_overlay_2 = window._add_picked_edgeuse_overlay()
    _assert_overlay_metadata(vertex_edge_overlay, "edge")
    _assert_overlay_metadata(vertex_edgeuse_overlay, "edgeuse")
    _assert_draw_cycle(vertex_edgeuse_overlay, vertex_edgeuse_overlay_2, "edgeuse", vertex_edgeuse_count)
    _assert_transient_replaced(window, vertex_edgeuse_overlay, vertex_edgeuse_overlay_2, "edgeuse")
    if vertex_edge_overlay.handle.metadata.get("selection_kind") != "Vertex":
        raise SmokeFailure(f"vertex edge overlay source mismatch: {vertex_edge_overlay.handle.metadata}")
    if vertex_edgeuse_overlay.handle.metadata.get("selection_kind") != "Vertex":
        raise SmokeFailure(f"vertex edgeuse overlay source mismatch: {vertex_edgeuse_overlay.handle.metadata}")

    if window._clear_debug_overlays() < 4:
        raise SmokeFailure("topology pick smoke did not clear created overlays")


def run_smoke(show: bool, timeout_ms: int) -> None:
    """Run the GUI smoke test and raise ``SmokeFailure`` on validation misses."""
    if smlib_gui.sm is None:
        raise SmokeFailure("_omni_solid did not import; build the repo or use a Python 3.12 environment")

    try:
        import vtk  # noqa: PLC0415

        vtk.vtkObject.GlobalWarningDisplayOff()
    except ImportError:
        pass
    except Exception:
        logger.exception("VTK initialization failed while disabling global warnings")
        raise

    smlib_gui.pv.global_theme.allow_empty_mesh = True
    _run_usd_schema_discovery_smoke()
    _run_split_args_smoke()
    _run_user_test_catalog_smoke()
    _run_mesh_batch_smoke()

    app = smlib_gui.QtWidgets.QApplication.instance()
    if app is None:
        app = smlib_gui.QtWidgets.QApplication(sys.argv[:1])

    window = smlib_gui.MainWindow()
    try:
        minimum_hint = window.minimumSizeHint()
        if minimum_hint.height() >= window.size().height():
            raise SmokeFailure(
                "main window layout minimum height prevents vertical resizing; "
                f"minimum={minimum_hint.height()} initial={window.size().height()}"
            )

        panel_groups = [group for group in window.findChildren(smlib_gui.QtWidgets.QGroupBox) if group.isCheckable()]
        if len(panel_groups) < 8:
            raise SmokeFailure(f"expected collapsible right-panel groups; found={len(panel_groups)}")
        panel_titles = [group.title() for group in panel_groups]
        if panel_titles.count("View") != 1 or "Display" in panel_titles:
            raise SmokeFailure(f"display controls were not consolidated into View: {panel_titles}")
        if "Command Chain" in panel_titles:
            raise SmokeFailure(f"command chain panel should be removed: {panel_titles}")
        expanded_titles = {group.title() for group in panel_groups if group.isChecked()}
        if expanded_titles != {"File", "View"}:
            raise SmokeFailure(f"expected only File and View expanded at startup; expanded={sorted(expanded_titles)}")
        collapsed = [group for group in panel_groups if not group.isChecked()]
        if not collapsed or any(group.maximumHeight() > 64 for group in collapsed):
            raise SmokeFailure("startup-collapsed right-panel groups did not shrink")
        sample_group = collapsed[0]
        sample_group.setChecked(True)
        app.processEvents()
        if not sample_group.isChecked() or sample_group.maximumHeight() <= 64:
            raise SmokeFailure("expanding a right-panel group did not restore its contents")
        sample_group.setChecked(False)
        app.processEvents()
        if sample_group.isChecked() or sample_group.maximumHeight() > 64:
            raise SmokeFailure("collapsing a right-panel group did not shrink its contents")

        _run_left_panel_collapse_smoke(window, app)
        _run_user_test_gui_smoke(window, app)

        if show:
            window.show()
            app.processEvents()

        _run_in_process_test_smoke(window, app)

        window._execute_and_display(SMOKE_SOURCE)
        app.processEvents()

        if not window._objects:
            raise SmokeFailure(f"script produced no active objects; status={window._status_bar.text()!r}")
        if window._object_list.count() != len(window._objects):
            raise SmokeFailure(
                f"object list mismatch; list={window._object_list.count()} registry={len(window._objects)}"
            )
        if not window._object_info.toPlainText().strip():
            raise SmokeFailure("object inspector did not populate after script execution")

        object_count = len(window._objects)
        if object_count < 2:
            raise SmokeFailure("smoke source should produce at least two objects for selected-object booleans")
        total_points, total_faces = _mesh_counts(window._objects)
        if total_points <= 0 or total_faces <= 0:
            raise SmokeFailure(f"tessellation produced an empty mesh; points={total_points}, face_stream={total_faces}")
        for obj in window._objects:
            if not obj.mesh:
                continue
            for key in ("points", "normals", "faces", "face_normals"):
                if not isinstance(obj.mesh.get(key), smlib_gui.np.ndarray):
                    raise SmokeFailure(f"{obj.name} mesh field {key!r} is not NumPy-backed")

        actor_count = _renderer_actor_count(window._plotter)
        if actor_count <= 0:
            raise SmokeFailure("PyVista plotter has no renderer actors after display")
        if window._hud.left_actor is None or window._hud.right_actor is None:
            raise SmokeFailure("viewport HUD did not create left/right text overlays")
        starting_display_mode = window._display_settings.display_mode
        window._cycle_display_mode_hotkey()
        app.processEvents()
        if window._display_settings.display_mode == starting_display_mode:
            raise SmokeFailure("display mode hotkey did not cycle to a new mode")
        starting_color_mode = window._display_settings.color_mode
        if starting_color_mode != COLOR_MODE_BREP_ORIENTATION:
            raise SmokeFailure(f"default color mode expected BRep orientation, got {starting_color_mode!r}")
        window._toggle_color_mode_hotkey()
        app.processEvents()
        if window._display_settings.color_mode == starting_color_mode:
            raise SmokeFailure("color mode hotkey did not toggle")
        if window._display_settings.color_mode != COLOR_MODE_VTK_BACKFACES:
            raise SmokeFailure(
                f"color mode hotkey expected VTK backfaces next, got " f"{window._display_settings.color_mode!r}"
            )
        window._toggle_color_mode_hotkey()
        app.processEvents()
        if window._display_settings.color_mode != COLOR_MODE_MATERIAL:
            raise SmokeFailure(
                f"color mode hotkey expected Material next, got " f"{window._display_settings.color_mode!r}"
            )
        window._toggle_color_mode_hotkey()
        app.processEvents()
        if window._display_settings.color_mode != COLOR_MODE_BREP_ORIENTATION:
            raise SmokeFailure(
                f"color mode hotkey did not cycle back to BRep orientation, got "
                f"{window._display_settings.color_mode!r}"
            )
        for index in range(window._display_mode_combo.count()):
            if window._display_mode_combo.itemData(index) == smlib_gui.DISPLAY_MODE_SHADED_WIREFRAME:
                window._display_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        visible_objects = [obj for obj in window._objects if obj.visible]
        display_mode = window._display_settings.display_mode
        visible_brep_colors = {obj.color for obj in visible_objects if obj.kind == "Brep"}
        if display_mode not in DISPLAY_MODES_WITH_BREP_WIREFRAME:
            expected_edge_actors = 0
        elif display_mode in DISPLAY_MODES_WITH_SHADED_SURFACE:
            # Shaded modes recolor every BRep edge, collapsing them into one batch.
            expected_edge_actors = min(len(visible_brep_colors), 1)
        else:
            expected_edge_actors = len(visible_brep_colors)
        expected_actor_limit = (
            len(
                smlib_gui.objects_to_pyvista_batches(
                    visible_objects,
                    orient_to_face_normals=(window._display_settings.color_mode == COLOR_MODE_BREP_ORIENTATION),
                )
            )
            + expected_edge_actors
            + 3
        )
        # Re-read after the display/color mode cycling above; the earlier count
        # predates those redraws.
        actor_count = _renderer_actor_count(window._plotter)
        if actor_count > expected_actor_limit:
            raise SmokeFailure(
                "display rendering created per-object actors; "
                f"actors={actor_count} expected_at_most={expected_actor_limit}"
            )

        expected_display_labels = [label for label, _mode in smlib_gui.DISPLAY_MODE_OPTIONS]
        actual_display_labels = [
            window._display_mode_combo.itemText(i) for i in range(window._display_mode_combo.count())
        ]
        if actual_display_labels != expected_display_labels:
            raise SmokeFailure(f"display mode options mismatch: {actual_display_labels}")
        display_calls = []
        original_display_meshes = window._display_meshes

        def _record_display_mode(meshes):
            display_calls.append((window._display_settings.display_mode, len(meshes)))
            return 0

        try:
            window._display_meshes = _record_display_mode
            for index, (_label, mode) in enumerate(smlib_gui.DISPLAY_MODE_OPTIONS):
                window._display_mode_combo.setCurrentIndex(index)
                app.processEvents()
                if window._display_settings.display_mode != mode:
                    raise SmokeFailure(f"display mode did not update to {mode}")
        finally:
            window._display_meshes = original_display_meshes
        seen_modes = [mode for mode, _mesh_count in display_calls]
        expected_modes = [mode for _label, mode in smlib_gui.DISPLAY_MODE_OPTIONS[1:]]
        if seen_modes != expected_modes:
            raise SmokeFailure(f"display mode changes did not redraw all modes: {seen_modes}")

        _run_topology_pick_smoke(window, app)
        window._execute_and_display(SMOKE_SOURCE)
        app.processEvents()
        if len(window._objects) != object_count:
            raise SmokeFailure("script restore after topology picking did not recreate the original object count")

        with tempfile.TemporaryDirectory(prefix="smlib_gui_smoke_") as tmpdir:
            native_path = os.path.join(tmpdir, "part")
            native_file = native_path + ".smb"
            if not window._save_file(native_path, "SMLib BRep (*.smb)"):
                raise SmokeFailure(f"native BRep save failed; status={window._status_bar.text()!r}")
            if not os.path.isfile(native_file) or os.path.getsize(native_file) <= 0:
                raise SmokeFailure("native BRep save did not create a non-empty file")
            if not smlib_gui.native_brep_file_is_ascii(native_file):
                raise SmokeFailure("native BRep save should default to ASCII format")
            loaded_native = window._load_file(native_file)
            app.processEvents()
            if len(loaded_native) != 1 or loaded_native[0].kind != "Brep":
                raise SmokeFailure(f"native BRep load failed; status={window._status_bar.text()!r}")

            binary_native_file = os.path.join(tmpdir, "part_binary.smb")
            loaded_native[0].handle.write_to_file(binary_native_file, ascii=False)
            if smlib_gui.native_brep_file_is_ascii(binary_native_file):
                raise SmokeFailure("binary native BRep test file was detected as ASCII")
            loaded_binary_native = window._load_file(binary_native_file)
            app.processEvents()
            if len(loaded_binary_native) != 1 or loaded_binary_native[0].kind != "Brep":
                raise SmokeFailure(f"binary native BRep load failed; status={window._status_bar.text()!r}")

            legacy_header_file = os.path.join(tmpdir, "legacy_header.smb")
            with open(legacy_header_file, "w", encoding="utf-8") as file:
                file.write("//Brep Starts\n")
            if not smlib_gui.native_brep_file_is_ascii(legacy_header_file):
                raise SmokeFailure("legacy native BRep header was not detected as ASCII")

            usd_path = os.path.join(tmpdir, "part")
            usd_file = usd_path + ".usda"
            usd_roundtrip = smlib_gui.brep_usd_io_available()
            if usd_roundtrip:
                if not window._save_file(usd_path, "USD ASCII (*.usda)"):
                    raise SmokeFailure(f"USD export failed; status={window._status_bar.text()!r}")
                if not os.path.isfile(usd_file) or os.path.getsize(usd_file) <= 0:
                    raise SmokeFailure("USD export did not create a non-empty file")

                imported = window._load_file(usd_file)
                app.processEvents()
                if not imported or not window._objects:
                    raise SmokeFailure(f"USD import produced no active objects; status={window._status_bar.text()!r}")
                if window._object_list.count() != len(window._objects):
                    raise SmokeFailure("object list mismatch after USD import")

                if not smlib_gui.occt_brep_io_available():
                    raise SmokeFailure("OCCT BRep converters are unavailable after a full GUI/USD setup")
                corpus_occt_file = os.path.join(smlib_gui.REPO, "TestFiles", "occt_breps", "box.brep")
                loaded_occt = window._load_file(corpus_occt_file)
                app.processEvents()
                if len(loaded_occt) != 1 or loaded_occt[0].kind != "Brep":
                    raise SmokeFailure(f"OCCT BRep load failed; status={window._status_bar.text()!r}")

                occt_path = os.path.join(tmpdir, "part_occt")
                occt_file = occt_path + ".brep"
                if not window._save_file(occt_path, "OpenCASCADE BRep (*.brep)"):
                    raise SmokeFailure(f"OCCT BRep save failed; status={window._status_bar.text()!r}")
                if not os.path.isfile(occt_file) or os.path.getsize(occt_file) <= 0:
                    raise SmokeFailure("OCCT BRep save did not create a non-empty file")
                reloaded_occt = window._load_file(occt_file)
                app.processEvents()
                if len(reloaded_occt) != 1 or reloaded_occt[0].kind != "Brep":
                    raise SmokeFailure(f"OCCT BRep reload failed; status={window._status_bar.text()!r}")
            elif window._save_file(usd_path, "USD ASCII (*.usda)"):
                raise SmokeFailure("USD export unexpectedly succeeded without OmniSolid schema resources")

            nested_dir = Path(tmpdir) / "nested"
            nested_dir.mkdir()
            nested_native_file = nested_dir / "part.smb"
            loaded_native[0].handle.write_to_file(str(nested_native_file), ascii=True)
            nested_native_file_copy = nested_dir / "part_copy.smb"
            loaded_native[0].handle.write_to_file(str(nested_native_file_copy), ascii=True)
            try:
                (nested_dir / "part_link.smb").symlink_to(nested_native_file)
            except (OSError, NotImplementedError):
                pass  # Symlink creation can be unprivileged-denied on Windows.
            discovered = smlib_gui.discover_model_files(Path(tmpdir))
            if nested_native_file.resolve() not in discovered or nested_native_file_copy.resolve() not in discovered:
                raise SmokeFailure("headless recursive discovery missed a nested native BRep")
            # The symlink resolves onto part.smb, so discovery must report two files.
            nested_discovered = smlib_gui.discover_model_files(nested_dir)
            if len(nested_discovered) != 2 or len(set(nested_discovered)) != 2:
                raise SmokeFailure(f"headless discovery did not deduplicate inputs: {nested_discovered}")
            mapped_output = smlib_gui.output_directory_for(
                nested_native_file,
                Path(tmpdir),
                Path(tmpdir) / "renders",
            )
            if mapped_output != (Path(tmpdir) / "renders" / "nested").resolve():
                raise SmokeFailure(f"headless output hierarchy mismatch: {mapped_output}")
            adjacent_output = smlib_gui.output_directory_for(
                nested_native_file,
                Path(tmpdir),
                None,
            )
            if adjacent_output != nested_dir.resolve():
                raise SmokeFailure(f"headless adjacent output mismatch: {adjacent_output}")

            render_dir = Path(tmpdir) / "headless"
            render_report = render_dir / "worker.json"
            render_script = Path(SCRIPT_DIR) / "smlib_gui_render.py"
            # Keep the per-file worker timeout below the outer timeout so a hung
            # worker is reported by the coordinator instead of killing it here.
            render_timeout = 120
            worker_timeout = 45
            render_result = subprocess.run(
                [
                    sys.executable,
                    str(render_script),
                    str(nested_dir),
                    "--output-dir",
                    str(render_dir),
                    "--report",
                    str(render_report),
                    "--jobs",
                    "2",
                    "--worker-timeout",
                    str(worker_timeout),
                    "--width",
                    "320",
                    "--height",
                    "320",
                ],
                capture_output=True,
                text=True,
                env=os.environ.copy(),
                timeout=render_timeout,
            )
            if render_result.returncode != 0:
                detail = (render_result.stderr or render_result.stdout).strip()
                raise SmokeFailure(f"headless render worker failed: {detail}")
            render_summary = json.loads(render_report.read_text(encoding="utf-8"))
            rendered_files = render_summary.get("files", [])
            if render_summary.get("jobs") != 2 or len(rendered_files) != 2:
                raise SmokeFailure(f"parallel headless renderer omitted results: {render_summary}")
            for rendered in rendered_files:
                if rendered.get("status") != "rendered":
                    raise SmokeFailure(f"parallel headless render failed: {rendered}")
                if rendered.get("views"):
                    raise SmokeFailure(f"headless renderer kept named views by default: {rendered}")
                contact_path = Path(rendered.get("contact_sheet", ""))
                if not contact_path.is_file() or contact_path.stat().st_size <= 0:
                    raise SmokeFailure("headless renderer did not create the contact sheet")
                contact = smlib_gui.QtGui.QImage(str(contact_path))
                if contact.isNull() or (contact.width(), contact.height()) != (640, 640):
                    raise SmokeFailure("headless renderer wrote an invalid contact sheet")

            views_dir = Path(tmpdir) / "headless_views"
            views_report = views_dir / "report.json"
            views_result = subprocess.run(
                [
                    sys.executable,
                    str(render_script),
                    str(nested_native_file),
                    "--output-dir",
                    str(views_dir),
                    "--report",
                    str(views_report),
                    "--save-views",
                    "--worker-timeout",
                    str(worker_timeout),
                    "--width",
                    "320",
                    "--height",
                    "320",
                ],
                capture_output=True,
                text=True,
                env=os.environ.copy(),
                timeout=render_timeout,
            )
            if views_result.returncode != 0:
                detail = (views_result.stderr or views_result.stdout).strip()
                raise SmokeFailure(f"headless --save-views render failed: {detail}")
            views_summary = json.loads(views_report.read_text(encoding="utf-8"))
            views_files = views_summary.get("files", [])
            if len(views_files) != 1 or views_files[0].get("status") != "rendered":
                raise SmokeFailure(f"headless --save-views render is incomplete: {views_summary}")
            saved_views = views_files[0].get("views", {})
            if len(saved_views) != len(smlib_gui.DEFAULT_VIEW_NAMES):
                raise SmokeFailure(f"--save-views omitted named views: {views_files[0]}")
            for view_path in (Path(path) for path in saved_views.values()):
                if not view_path.is_file() or view_path.stat().st_size <= 0:
                    raise SmokeFailure(f"--save-views did not keep a named view: {view_path}")
                image = smlib_gui.QtGui.QImage(str(view_path))
                if image.isNull() or (image.width(), image.height()) != (320, 320):
                    raise SmokeFailure(f"--save-views wrote an invalid named view: {view_path}")
                samples = {
                    image.pixel(0, 0),
                    image.pixel(image.width() // 2, image.height() // 2),
                    image.pixel(image.width() - 1, image.height() - 1),
                }
                if len(samples) < 2:
                    raise SmokeFailure(f"--save-views wrote a blank named view: {view_path}")

        part_file = os.path.join(
            smlib_gui.REPO,
            "TestFiles",
            "bz_TestFiles",
            "Regressions2009",
            "10.Wilson",
            "SmallBox.smp",
        )
        if os.path.isfile(part_file):
            loaded_part = window._load_file(part_file)
            app.processEvents()
            if len(loaded_part) != 1 or loaded_part[0].kind != "Brep":
                raise SmokeFailure(f"native part load failed; status={window._status_bar.text()!r}")
        else:
            print(f"Skipping unavailable native part fixture: {part_file}")

        window._execute_and_display(SMOKE_SOURCE)
        app.processEvents()
        if len(window._objects) != object_count:
            raise SmokeFailure("script restore after file I/O did not recreate the original object count")

        window._max_edge_spin.setValue(4.0)
        window._max_aspect_spin.setValue(5.0)
        app.processEvents()
        if window._tessellation_settings.max_edge_length != 4.0:
            raise SmokeFailure("max edge tessellation control did not update settings")
        if window._tessellation_settings.max_aspect_ratio != 5.0:
            raise SmokeFailure("max aspect tessellation control did not update settings")
        total_points, total_faces = _mesh_counts(window._objects)
        if total_points <= 0 or total_faces <= 0:
            raise SmokeFailure("retessellation after max edge/aspect controls produced an empty mesh")

        actor_count_before_inspection = _renderer_actor_count(window._plotter)
        for index in range(window._normals_mode_combo.count()):
            if window._normals_mode_combo.itemData(index) == NORMALS_MODE_MESH_VERTEX:
                window._normals_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        if window._display_settings.normals_mode != NORMALS_MODE_MESH_VERTEX:
            raise SmokeFailure("mesh vertex normals mode did not update display settings")
        if _renderer_actor_count(window._plotter) <= actor_count_before_inspection:
            raise SmokeFailure("mesh vertex normals overlay did not add viewport actors")
        for index in range(window._normals_mode_combo.count()):
            if window._normals_mode_combo.itemData(index) == NORMALS_MODE_MESH_FACE:
                window._normals_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        if window._display_settings.normals_mode != NORMALS_MODE_MESH_FACE:
            raise SmokeFailure("mesh face normals mode did not update display settings")
        window._edge_vertices_check.setChecked(True)
        app.processEvents()
        if not window._display_settings.show_vertices:
            raise SmokeFailure("vertices overlay toggle did not update display settings")
        window._cycle_topology_counts_hotkey()
        app.processEvents()
        if window._display_settings.topology_counts_mode != TOPOLOGY_COUNTS_BREP_DETAIL:
            raise SmokeFailure("topology counts hotkey did not advance to BRep detail")
        window._cycle_topology_counts_hotkey()
        app.processEvents()
        if window._display_settings.topology_counts_mode != TOPOLOGY_COUNTS_MESH:
            raise SmokeFailure("topology counts hotkey did not advance to mesh counts")
        window._cycle_topology_counts_hotkey()
        app.processEvents()
        if window._display_settings.topology_counts_mode != TOPOLOGY_COUNTS_BREP:
            raise SmokeFailure("topology counts hotkey did not wrap back to BRep F/E/V")
        for index in range(window._normals_mode_combo.count()):
            if window._normals_mode_combo.itemData(index) == NORMALS_MODE_OFF:
                window._normals_mode_combo.setCurrentIndex(index)
                break
        window._edge_vertices_check.setChecked(False)
        app.processEvents()

        bbox_diag = window._bbox_diag
        if bbox_diag <= 0:
            raise SmokeFailure("scene bbox diagonal was not computed")
        window._cycle_chord_tolerance_hotkey()
        app.processEvents()
        expected_chord = chord_tolerance_for_index(window._chord_preset_index, bbox_diag)
        if abs(window._tessellation_settings.chord_height_tolerance - expected_chord) > 0.001:
            raise SmokeFailure(
                "chord preset cycle did not update tessellation settings: "
                f"{window._tessellation_settings.chord_height_tolerance} vs {expected_chord}"
            )

        window._on_view_front()
        front_position, front_focal = _camera_position_focal(window._plotter)
        if front_position[1] >= front_focal[1]:
            raise SmokeFailure(
                f"CAD front view expected camera on -Y side; position={front_position}, focal={front_focal}"
            )
        for view_action in (window._on_view_top, window._on_view_bottom):
            view_action()
            position, focal = _camera_position_focal(window._plotter)
            view_up = _camera_view_up(window._plotter)
            direction = [position[i] - focal[i] for i in range(3)]
            length = sum(component * component for component in direction) ** 0.5
            along_view = sum((direction[i] / length) * view_up[i] for i in range(3)) if length > 0 else 1.0
            # A view-up parallel to the view direction makes VTK discard it.
            if abs(along_view) > 0.001:
                raise SmokeFailure(
                    f"{view_action.__name__} produced a degenerate view-up: "
                    f"up={view_up}, position={position}, focal={focal}"
                )
        window._on_view_right()
        window._on_view_isometric()
        window._on_view_back()
        back_position, back_focal = _camera_position_focal(window._plotter)
        if back_position[1] <= back_focal[1]:
            raise SmokeFailure(
                f"CAD back view expected camera on +Y side; position={back_position}, focal={back_focal}"
            )
        for index in range(window._display_mode_combo.count()):
            if window._display_mode_combo.itemData(index) == smlib_gui.DISPLAY_MODE_SHADED_WIREFRAME:
                window._display_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        expected_color_labels = [label for label, _mode in COLOR_MODE_OPTIONS]
        actual_color_labels = [window._color_mode_combo.itemText(i) for i in range(window._color_mode_combo.count())]
        if actual_color_labels != expected_color_labels:
            raise SmokeFailure(f"color mode options mismatch: {actual_color_labels}")
        for index in range(window._color_mode_combo.count()):
            if window._color_mode_combo.itemData(index) == COLOR_MODE_BREP_ORIENTATION:
                window._color_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()

        def _require_red_backfaces(mode_name: str) -> None:
            colors = [_actor_backface_color(actor) for actor in _surface_mesh_actors(window._plotter)]
            colors = [color for color in colors if color is not None]
            if not colors:
                raise SmokeFailure(f"{mode_name} did not produce mesh actors with backfaces")
            if not any(color[0] > 0.7 and color[1] < 0.3 for color in colors):
                raise SmokeFailure(f"{mode_name} did not use red backfaces: {colors}")

        _require_red_backfaces("BRep orientation color mode")
        for index in range(window._color_mode_combo.count()):
            if window._color_mode_combo.itemData(index) == COLOR_MODE_MATERIAL:
                window._color_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        material_pairs = []
        for actor in _surface_mesh_actors(window._plotter):
            backface = _actor_backface_color(actor)
            if backface is None:
                continue
            front = tuple(float(value) for value in actor.GetProperty().GetColor())
            material_pairs.append((front, backface))
        if not material_pairs:
            raise SmokeFailure("material color mode did not produce mesh actors with backfaces")
        for front, backface in material_pairs:
            if any(abs(front[i] - backface[i]) > 0.02 for i in range(3)):
                raise SmokeFailure(f"material color mode backface did not match front color: {front} vs {backface}")
        for index in range(window._color_mode_combo.count()):
            if window._color_mode_combo.itemData(index) == COLOR_MODE_VTK_BACKFACES:
                window._color_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        _require_red_backfaces("VTK backfaces color mode")
        for index in range(window._color_mode_combo.count()):
            if window._color_mode_combo.itemData(index) == COLOR_MODE_BREP_ORIENTATION:
                window._color_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        for index in range(window._display_mode_combo.count()):
            if window._display_mode_combo.itemData(index) == smlib_gui.DISPLAY_MODE_SHADED_WIREFRAME:
                window._display_mode_combo.setCurrentIndex(index)
                break
        app.processEvents()
        window._on_fit_view()
        window._on_save_view()
        if window._viewport.saved_camera is None:
            raise SmokeFailure("save view action did not capture a camera state")
        window._on_view_top()
        window._on_restore_view()
        window._parallel_projection_check.setChecked(True)
        app.processEvents()
        if not window._viewport.is_parallel_projection():
            raise SmokeFailure("parallel projection toggle did not update the PyVista camera")
        window._parallel_projection_check.setChecked(False)
        app.processEvents()

        window._object_list.clearSelection()
        window._object_list.setCurrentRow(0)
        window._object_list.item(0).setSelected(True)
        window._object_list.item(1).setSelected(True)
        result = window._apply_boolean_operation("union")
        app.processEvents()
        if result is None:
            raise SmokeFailure(f"selected-object boolean failed; status={window._status_bar.text()!r}")
        if len(window._objects) != object_count - 1:
            raise SmokeFailure("selected-object boolean did not consume two inputs and add one result")
        if result.kind != "Brep" or window._selected_object() is not result:
            raise SmokeFailure("selected-object boolean did not select the BRep result")
        if "sm.boolean_union(" not in window._current_source:
            raise SmokeFailure("selected-object boolean did not append reproducible source")

        actor_count_before_overlays = _renderer_actor_count(window._plotter)
        overlays = [
            window._add_selected_box_overlay(),
            window._add_selected_center_overlay(),
            window._add_selected_section_overlay(),
            window._add_debug_line("smoke_line", (-5, -5, 0), (5, 5, 10)),
            window._add_debug_polyline("smoke_polyline", [(-4, 0, 0), (0, 4, 4), (4, 0, 8)]),
        ]
        app.processEvents()
        if any(overlay is None for overlay in overlays):
            raise SmokeFailure(f"debug overlay creation failed; status={window._status_bar.text()!r}")
        if window._object_list.count() != len(window._objects):
            raise SmokeFailure("object list mismatch after adding debug overlays")
        overlay_kinds = {overlay.kind for overlay in overlays if overlay is not None}
        expected_overlay_kinds = {
            "Box",
            "Point",
            "Section",
            "Line",
            "Polyline",
        }
        if not expected_overlay_kinds.issubset(overlay_kinds):
            raise SmokeFailure(f"unexpected debug overlay kinds: {overlay_kinds}")
        section_overlay = next(overlay for overlay in overlays if overlay is not None and overlay.kind == "Section")
        window._set_selected_object_id(section_overlay.object_id)
        if "segments:" not in window._object_info.toPlainText():
            raise SmokeFailure("section overlay inspector did not include section metadata")
        if _renderer_actor_count(window._plotter) <= actor_count_before_overlays:
            raise SmokeFailure("debug overlays did not add viewport actors")

        removed_overlays = window._clear_debug_overlays()
        app.processEvents()
        if removed_overlays < len(overlays):
            raise SmokeFailure("clear debug overlays did not remove every overlay")
        if any(obj.kind in expected_overlay_kinds for obj in window._objects):
            raise SmokeFailure("debug overlay registry entries remained after clear")

        mass_result = window._apply_modify_operation("mass_properties")
        if mass_result is None or "Modify: mass_properties" not in window._object_info.toPlainText():
            raise SmokeFailure(f"mass properties action failed; status={window._status_bar.text()!r}")
        _assert_valid_passed(window, window._on_assert_valid_object)
        if "smdev.assert_valid_reports(" not in window._current_source:
            raise SmokeFailure("AssertValid did not record replayable source")
        heal_result = window._apply_modify_operation("heal")
        if heal_result is None or "sm.heal_brep(" not in window._current_source:
            raise SmokeFailure(f"heal action failed; status={window._status_bar.text()!r}")
        stitch_result = window._apply_modify_operation("stitch_solid")
        if stitch_result is None or "sm.stitch_into_solid(" not in window._current_source:
            raise SmokeFailure(f"stitch solid action failed; status={window._status_bar.text()!r}")
        shell_result = window._apply_modify_operation("stitch_shell")
        if shell_result is None or "sm.stitch_into_shell(" not in window._current_source:
            raise SmokeFailure(f"stitch shell action failed; status={window._status_bar.text()!r}")
        normal_result = window._apply_modify_operation("unify_normals")
        if normal_result is None or "sm.unify_normals(" not in window._current_source:
            raise SmokeFailure(f"unify normals action failed; status={window._status_bar.text()!r}")

        _assert_dump_wrote(window, window._on_dump_object)
        dump_text = window._object_info.toPlainText()
        if "Dump:" not in dump_text:
            raise SmokeFailure("object dump did not record the kernel Dump target")

        if show:
            # A local loop, not app.quit(): Qt 6's quit() closes the window, which closes the
            # plotter the steps below still use.
            loop = smlib_gui.QtCore.QEventLoop()
            smlib_gui.QtCore.QTimer.singleShot(max(0, timeout_ms), loop.quit)
            loop.exec()

        window._on_delete_object()
        if window._objects or window._parts or window._object_list.count():
            raise SmokeFailure("delete action did not clear the active object registry")

        print(
            "smlib_gui smoke ok: "
            f"objects={object_count} points={total_points} face_stream={total_faces} actors={actor_count}"
        )
    finally:
        window.close()
        app.processEvents()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true", help="show the GUI briefly before exiting")
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=1000,
        help="milliseconds to keep the GUI event loop alive when --show is used",
    )
    args = parser.parse_args(argv)

    try:
        run_smoke(show=args.show, timeout_ms=args.timeout_ms)
    except Exception:
        traceback.print_exc()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
