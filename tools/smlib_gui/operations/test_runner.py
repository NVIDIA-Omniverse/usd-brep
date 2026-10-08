# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""In-process prog_test runner integration for the SMLib GUI."""

from __future__ import annotations

from collections.abc import Iterable
import os
from typing import Any

from ..runtime import REPO, smdev
from .overlays import (
    BoxOverlay,
    DrawBatchOverlay,
    LineOverlay,
    NormalOverlay,
    PointOverlay,
    PolylineOverlay,
)


PROG_TEST_SUITES = (
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

SMLIB_TESTS_UNAVAILABLE_HINT = (
    "In-process prog_test runner unavailable. Build the _smlib_dev and _smlib_tests "
    "dev modules and run tools/scripts/smlib_gui.py from the repo Python environment."
)

DEBUG_GUIDANCE = (
    "Debug: launch tools/scripts/smlib_gui.py (debug binaries by default; "
    "--wait-for-lldb to attach, --release for release), set C++ breakpoints, "
    "select a prog_test suite, then click Run In GUI."
)

PROG_TEST_WORKING_DIRECTORY = os.path.join(REPO, "tests", "prog_test")

try:
    import _smlib_tests  # type: ignore
except ImportError as exc:
    _smlib_tests = None  # type: ignore
    _import_error: ImportError | None = exc
else:
    _import_error = None


def tests_available() -> bool:
    if smdev is not None:
        try:
            if smdev.tests.available():
                return True
        except Exception:
            pass
    return _smlib_tests is not None


def unavailable_reason() -> str:
    if _import_error is None:
        return ""
    return f"{SMLIB_TESTS_UNAVAILABLE_HINT}\nImport error: {_import_error}"


def list_prog_test_suites() -> tuple[str, ...]:
    if smdev is not None:
        try:
            if smdev.tests.available():
                suites = tuple(str(name) for name in smdev.tests.list_prog_test_suites())
                return suites or PROG_TEST_SUITES
        except Exception:
            pass
    if _smlib_tests is None:
        return PROG_TEST_SUITES
    suites = tuple(str(name) for name in _smlib_tests.list_prog_test_suites())
    return suites or PROG_TEST_SUITES


def run_prog_test_suite(
    name: str,
    working_directory: str = PROG_TEST_WORKING_DIRECTORY,
    do_graphics: bool = True,
) -> dict[str, Any]:
    if smdev is not None:
        try:
            if smdev.tests.available():
                result = smdev.tests.run_prog_test_suite(
                    str(name),
                    working_directory=working_directory,
                    do_graphics=bool(do_graphics),
                )
                return normalize_prog_test_result(result, name)
        except Exception as exc:
            if _smlib_tests is None:
                raise RuntimeError(unavailable_reason()) from exc
    if _smlib_tests is None:
        raise RuntimeError(unavailable_reason())
    result = _smlib_tests.run_prog_test_suite(
        str(name),
        working_directory=working_directory,
        do_graphics=bool(do_graphics),
    )
    return normalize_prog_test_result(result, name)


def normalize_prog_test_result(result: Any, requested_name: str) -> dict[str, Any]:
    data = dict(result)
    status = int(data.get("status", 0))
    log = data.get("log", "")
    draw_events = data.get("draw_events", ())
    return {
        "suite_name": str(data.get("suite_name") or requested_name),
        "status": status,
        "status_name": str(data.get("status_name") or status),
        "ok": bool(data.get("ok", False)),
        "elapsed_seconds": float(data.get("elapsed_seconds", 0.0)),
        "log": str(log),
        "draw_events": list(draw_events) if draw_events is not None else [],
    }


def draw_events_to_overlays(events: Iterable[dict[str, Any]]) -> list[tuple[str, str, object]]:
    overlays: list[tuple[str, str, object]] = []
    for index, event in enumerate(events, start=1):
        converted = draw_event_to_overlay(event, index)
        if converted is not None:
            overlays.append(converted)
    return overlays


def draw_event_to_overlay(event: dict[str, Any], index: int = 1) -> tuple[str, str, object] | None:
    event_type = str(event.get("type") or event.get("kind") or "").lower()
    name = str(event.get("name") or f"prog_test_{event_type or 'draw'}_{index}")
    metadata = dict(event.get("metadata") or {})
    metadata.setdefault("source", "prog_test")

    if event_type == "point":
        return name, "Point", PointOverlay(_point3(event.get("point", (0.0, 0.0, 0.0))))
    if event_type == "line":
        return (
            name,
            "Line",
            LineOverlay(
                _point3(event.get("start", (0.0, 0.0, 0.0))),
                _point3(event.get("end", (0.0, 0.0, 0.0))),
            ),
        )
    if event_type == "polyline":
        points = [_point3(point) for point in event.get("points", ())]
        if points:
            return name, "Polyline", PolylineOverlay(points)
    if event_type == "box":
        return (
            name,
            "Box",
            BoxOverlay(
                _point3(event.get("minimum", (0.0, 0.0, 0.0))),
                _point3(event.get("maximum", (0.0, 0.0, 0.0))),
            ),
        )
    if event_type == "normal":
        return (
            name,
            "Normal",
            NormalOverlay(
                _point3(event.get("origin", (0.0, 0.0, 0.0))),
                _point3(event.get("vector", (0.0, 0.0, 1.0))),
                float(event.get("scale", 1.0)),
            ),
        )
    if event_type in {"display_list", "draw_batch", "draw_batches"}:
        batches = list(event.get("batches", ()))
        if batches:
            return name, "DisplayList", DrawBatchOverlay(batches=batches, metadata=metadata)
    return None


def _point3(value: object) -> tuple[float, float, float]:
    try:
        seq = tuple(value)  # type: ignore[arg-type]
    except TypeError:
        seq = (0.0, 0.0, 0.0)
    if len(seq) < 3:
        seq = (*seq, 0.0, 0.0, 0.0)
    return (float(seq[0]), float(seq[1]), float(seq[2]))
