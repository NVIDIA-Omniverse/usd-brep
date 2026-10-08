#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Audit OCCT BRep tessellations through the offscreen SMLib GUI pipeline.

Each input is loaded through ``MainWindow._load_file`` so conversion, SMLib
ingest, tessellation, PyVista conversion, and viewport rendering match the
interactive application. The audit writes one four-view contact sheet per
fixture plus a JSON report containing geometry and mesh diagnostics.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import traceback


SCRIPT_DIR = Path(__file__).resolve().parent
REPO = SCRIPT_DIR.parent.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

os.environ.pop("DISPLAY", None)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "smlib_gui_matplotlib"))
os.environ.setdefault("MESA_SHADER_CACHE_DIR", str(Path(tempfile.gettempdir()) / "smlib_gui_mesa"))
Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
Path(os.environ["MESA_SHADER_CACHE_DIR"]).mkdir(parents=True, exist_ok=True)

import smlib_gui  # noqa: E402


VIEWS = smlib_gui.DEFAULT_VIEW_NAMES
FACELESS_TOPOLOGY_MESSAGE = "no Solid/Shell/Face component found"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=REPO / "TestFiles" / "occt_breps",
        help="Directory recursively searched for .brep fixtures.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(tempfile.gettempdir()) / "smlib_gui_occt_audit",
        help="Directory for screenshots and report.json.",
    )
    parser.add_argument("--chord", type=float, default=0.05)
    parser.add_argument("--angle", type=float, default=15.0)
    parser.add_argument("--max-edge", type=float, default=0.0)
    parser.add_argument("--max-aspect", type=float, default=0.0)
    parser.add_argument(
        "--worker-timeout",
        type=int,
        default=int(os.environ.get("OCCT_BREP_AUDIT_TIMEOUT_SEC", "300")),
        help="Maximum seconds allowed for one isolated fixture worker.",
    )
    parser.add_argument(
        "--only",
        nargs="*",
        default=(),
        help="Optional fixture-relative paths, basenames, or stems to audit.",
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--report-path", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args()


def _fixture_key(path: Path, input_dir: Path) -> str:
    return path.relative_to(input_dir).as_posix()


def _select_fixtures(fixtures: list[Path], input_dir: Path, requested) -> list[Path]:
    if not requested:
        return fixtures

    keys = {path: _fixture_key(path, input_dir) for path in fixtures}
    available_keys = set(keys.values())
    exact_keys = set()
    legacy_requests = set()
    for value in requested:
        normalized = str(value).replace("\\", "/")
        if normalized in available_keys:
            exact_keys.add(normalized)
        else:
            legacy_requests.add(normalized.lower())

    return [
        path
        for path in fixtures
        if keys[path] in exact_keys
        or keys[path].lower() in legacy_requests
        or path.name.lower() in legacy_requests
        or path.stem.lower() in legacy_requests
    ]


def _json_value(value):
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def _face_stream_metrics(flat_faces, point_count: int) -> dict:
    values = [int(value) for value in flat_faces]
    offset = 0
    cell_count = 0
    triangle_count = 0
    invalid_cells = 0
    malformed = False
    while offset < len(values):
        vertex_count = values[offset]
        end = offset + 1 + vertex_count
        if vertex_count < 3 or end > len(values):
            invalid_cells += 1
            malformed = True
            break
        indices = values[offset + 1:end]
        if any(index < 0 or index >= point_count for index in indices):
            invalid_cells += 1
        cell_count += 1
        triangle_count += max(0, vertex_count - 2)
        offset = end
    if offset != len(values) and not malformed:
        invalid_cells += 1
    return {
        "face_stream_values": len(values),
        "cells": cell_count,
        "triangles_after_fan": triangle_count,
        "invalid_cells": invalid_cells,
    }


def _mesh_metrics(obj) -> dict:
    mesh = obj.mesh or {}
    points = smlib_gui.np.asarray(mesh.get("points", ()), dtype=float)
    flat_faces = mesh.get("faces", ())
    result = {
        "points": int(len(points)),
        **_face_stream_metrics(flat_faces, len(points)),
        "finite_points": bool(points.size and smlib_gui.np.isfinite(points).all()),
    }
    if len(points) == 0 or len(flat_faces) == 0:
        return result

    poly = smlib_gui.objects_to_pyvista([obj])[0]
    triangulated = poly.triangulate()
    bounds = [float(value) for value in poly.bounds]
    diagonal = math.sqrt(
        (bounds[1] - bounds[0]) ** 2
        + (bounds[3] - bounds[2]) ** 2
        + (bounds[5] - bounds[4]) ** 2
    )
    weld_tolerance = max(diagonal, 1.0) * 1.0e-9
    cleaned = triangulated.clean(tolerance=weld_tolerance, absolute=True)

    cell_sizes = triangulated.compute_cell_sizes(length=False, area=True, volume=False)
    areas = smlib_gui.np.asarray(cell_sizes.cell_data.get("Area", ()), dtype=float)
    area_scale = max(diagonal * diagonal, 1.0)
    degenerate_area = 1.0e-15 * area_scale
    boundary = cleaned.extract_feature_edges(
        boundary_edges=True,
        non_manifold_edges=False,
        feature_edges=False,
        manifold_edges=False,
    )
    non_manifold = cleaned.extract_feature_edges(
        boundary_edges=False,
        non_manifold_edges=True,
        feature_edges=False,
        manifold_edges=False,
    )
    connected = cleaned.connectivity()
    region_ids = smlib_gui.np.asarray(connected.cell_data.get("RegionId", ()), dtype=int)

    result.update(
        {
            "pyvista_cells": int(poly.n_cells),
            "triangulated_cells": int(triangulated.n_cells),
            "cleaned_points": int(cleaned.n_points),
            "cleaned_cells": int(cleaned.n_cells),
            "bounds": bounds,
            "diagonal": diagonal,
            "area": float(areas.sum()) if areas.size else 0.0,
            "degenerate_triangles": int((areas <= degenerate_area).sum()) if areas.size else 0,
            "boundary_edges_after_weld": int(boundary.n_cells),
            "non_manifold_edges_after_weld": int(non_manifold.n_cells),
            "connected_regions": int(region_ids.max() + 1) if region_ids.size else 0,
        }
    )

    poly_brep = obj.display_handle
    try:
        result["poly_manifold"] = bool(poly_brep.is_manifold_solid())
    except Exception as exc:
        result["poly_manifold_error"] = str(exc)
    try:
        result["poly_tolerance"] = float(poly_brep.get_tolerance())
    except Exception as exc:
        result["poly_tolerance_error"] = str(exc)
    try:
        properties = poly_brep.compute_properties((0.0, 0.0, 0.0))
        result["poly_area"] = float(properties["area"])
        result["poly_volume"] = float(properties["volume"])
    except Exception as exc:  # Keep rendering even when a diagnostic query fails.
        result["poly_properties_error"] = str(exc)
    return result


def _brep_metrics(obj) -> dict:
    brep = obj.handle
    metrics = {}
    queries = {
        "faces": brep.face_count,
        "edges": brep.edge_count,
        "vertices": brep.vertex_count,
        "manifold": brep.is_manifold_solid,
        "volume": brep.volume,
        "bounds": brep.bounding_box,
        "center": brep.center,
    }
    query_errors = []
    for name, query in queries.items():
        try:
            metrics[name] = _json_value(query())
        except Exception as exc:
            query_errors.append({"query": name, "error": str(exc)})

    face_areas = []
    face_errors = []
    try:
        faces = brep.faces()
    except Exception as exc:
        faces = ()
        query_errors.append({"query": "faces", "error": str(exc)})
    for index, face in enumerate(faces):
        try:
            face_areas.append(float(face.area()))
        except Exception as exc:  # Keep the other faces and rendered evidence.
            face_areas.append(None)
            face_errors.append({"face": index, "error": str(exc)})
    finite_areas = [area for area in face_areas if area is not None and math.isfinite(area)]
    return {
        **metrics,
        "query_errors": query_errors,
        "face_areas": face_areas,
        "face_area_sum": sum(finite_areas),
        "face_area_errors": face_errors,
    }


def _red_pixel_fraction(image) -> float:
    pixels = smlib_gui.np.asarray(image)
    if pixels.ndim != 3 or pixels.shape[2] < 3 or pixels.size == 0:
        return 0.0
    rgb = pixels[:, :, :3].astype(float)
    red = (rgb[:, :, 0] > 90.0) & (rgb[:, :, 0] > 1.45 * rgb[:, :, 1]) & (
        rgb[:, :, 0] > 1.45 * rgb[:, :, 2]
    )
    return float(red.mean())


def _diagnose(brep: dict, mesh: dict, red_fractions: dict) -> list[str]:
    issues = []
    if mesh.get("points", 0) == 0 or mesh.get("cells", 0) == 0:
        issues.append("empty tessellation")
        return issues
    if not mesh.get("finite_points", False):
        issues.append("non-finite tessellation points")
    if mesh.get("invalid_cells", 0):
        issues.append(f"{mesh['invalid_cells']} invalid mesh cell record(s)")
    if mesh.get("degenerate_triangles", 0):
        issues.append(f"{mesh['degenerate_triangles']} degenerate triangle(s)")
    if brep.get("face_area_errors"):
        issues.append(f"area query failed for {len(brep['face_area_errors'])} face(s)")

    source_area = float(brep.get("face_area_sum", 0.0))
    mesh_area = float(mesh.get("area", 0.0))
    if source_area > 0.0:
        area_ratio = mesh_area / source_area
        mesh["area_ratio_to_brep"] = area_ratio
        if not 0.90 <= area_ratio <= 1.10:
            issues.append(f"mesh/BRep area ratio is {area_ratio:.4g}")

    if "manifold" in brep and "poly_manifold" in mesh and bool(brep["manifold"]) != bool(mesh["poly_manifold"]):
        issues.append(
            f"manifold classification changed from {brep.get('manifold')} to {mesh.get('poly_manifold')}"
        )
    if brep.get("manifold") and mesh.get("boundary_edges_after_weld", 0):
        issues.append(f"closed source has {mesh['boundary_edges_after_weld']} welded mesh boundary edge(s)")
    if brep.get("manifold") and mesh.get("non_manifold_edges_after_weld", 0):
        issues.append(
            f"closed source has {mesh['non_manifold_edges_after_weld']} non-manifold mesh edge(s)"
        )

    source_volume = abs(float(brep.get("volume", 0.0)))
    mesh_volume = abs(float(mesh.get("poly_volume", 0.0)))
    if source_volume > 0.0 and mesh_volume > 0.0:
        volume_ratio = mesh_volume / source_volume
        mesh["volume_ratio_to_brep"] = volume_ratio
        if not 0.90 <= volume_ratio <= 1.10:
            issues.append(f"mesh/BRep volume ratio is {volume_ratio:.4g}")

    max_red = max(red_fractions.values(), default=0.0)
    if brep.get("manifold") and max_red > 0.005:
        issues.append(f"backface-colored pixels reach {100.0 * max_red:.2f}% in one view")
    return issues


def _audit_one(window, app, renderer, path: Path, fixture_key: str, output_dir: Path) -> dict:
    started = time.monotonic()
    entry = {"file": fixture_key, "path": str(path), "status": "error", "issues": []}
    try:
        loaded = window._load_file(str(path))
        app.processEvents()
        entry["gui_status"] = window._status_bar.text()
        if not loaded:
            entry["issues"] = [entry["gui_status"] or "GUI load returned no objects"]
            if FACELESS_TOPOLOGY_MESSAGE in entry["issues"][0]:
                entry["status"] = "skip"
            return entry

        entry["objects"] = []
        for obj in loaded:
            object_entry = {"name": obj.name, "kind": obj.kind}
            if obj.kind == "Brep":
                object_entry["brep"] = _brep_metrics(obj)
            if obj.mesh:
                object_entry["mesh"] = _mesh_metrics(obj)
            else:
                object_entry["mesh"] = {}
            entry["objects"].append(object_entry)

        relative_path = Path(fixture_key)
        fixture_output_dir = output_dir / relative_path.parent
        fixture_output_dir.mkdir(parents=True, exist_ok=True)
        rendered = renderer.render_objects(
            path,
            window._visible_objects(),
            fixture_output_dir,
            output_prefix=relative_path.stem,
        )
        red_fractions = {
            view_name: _red_pixel_fraction(image)
            for view_name, image in rendered.screenshots.items()
        }
        entry["contact_sheet"] = str(rendered.contact_sheet)
        entry["red_pixel_fraction"] = red_fractions

        for object_entry in entry["objects"]:
            if object_entry["kind"] != "Brep":
                continue
            issues = _diagnose(object_entry["brep"], object_entry["mesh"], red_fractions)
            entry["issues"].extend(f"{object_entry['name']}: {issue}" for issue in issues)
        entry["status"] = "review" if entry["issues"] else "pass"
    except Exception as exc:
        entry["issues"] = [str(exc)]
        entry["traceback"] = traceback.format_exc()
    finally:
        entry["elapsed_seconds"] = time.monotonic() - started
    return entry


def _base_report(args: argparse.Namespace, input_dir: Path, output_dir: Path, fixture_count: int) -> dict:
    return {
        "input_dir": str(input_dir),
        "output_dir": str(output_dir),
        "settings": {
            "chord_height_tolerance": args.chord,
            "angle_tolerance_deg": args.angle,
            "max_edge_length": args.max_edge,
            "max_aspect_ratio": args.max_aspect,
        },
        "fixture_count": fixture_count,
        "fixtures": [],
    }


def _write_overview_sheets(report: dict, output_dir: Path) -> list[str]:
    """Compose nine fixture contact sheets per image for visual triage."""
    entries = [entry for entry in report["fixtures"] if Path(entry.get("contact_sheet", "")).is_file()]
    if not entries:
        return []

    app = smlib_gui.QtWidgets.QApplication.instance()
    if app is None:
        app = smlib_gui.QtWidgets.QApplication(sys.argv[:1])
    tile_width = 600
    image_height = 600
    label_height = 42
    columns = 3
    rows = 3
    paths = []
    for page_index, start in enumerate(range(0, len(entries), columns * rows), start=1):
        page_entries = entries[start:start + columns * rows]
        canvas = smlib_gui.QtGui.QImage(
            columns * tile_width,
            rows * (label_height + image_height),
            smlib_gui.QtGui.QImage.Format_RGB32,
        )
        canvas.fill(smlib_gui.QtGui.QColor("#171923"))
        painter = smlib_gui.QtGui.QPainter(canvas)
        painter.setPen(smlib_gui.QtGui.QColor("white"))
        font = painter.font()
        font.setPointSize(13)
        painter.setFont(font)
        for index, entry in enumerate(page_entries):
            x = (index % columns) * tile_width
            y = (index // columns) * (label_height + image_height)
            status = entry.get("status", "unknown")
            painter.drawText(x + 10, y + 28, f"{entry['file']} [{status}]")
            image = smlib_gui.QtGui.QImage(entry["contact_sheet"])
            scaled = image.scaled(
                tile_width,
                image_height,
                smlib_gui.QtCore.Qt.KeepAspectRatio,
                smlib_gui.QtCore.Qt.SmoothTransformation,
            )
            painter.drawImage(x + (tile_width - scaled.width()) // 2, y + label_height, scaled)
        painter.end()
        path = output_dir / f"overview_{page_index}.png"
        if not canvas.save(str(path)):
            raise RuntimeError(f"Could not save overview sheet: {path}")
        paths.append(str(path))
    return paths


def _run_isolated(args: argparse.Namespace, fixtures: list[Path], input_dir: Path,
                  output_dir: Path) -> int:
    """Run one GUI/VTK process per fixture and combine the worker reports."""
    worker_dir = output_dir / "worker_reports"
    worker_dir.mkdir(parents=True, exist_ok=True)
    report = _base_report(args, input_dir, output_dir, len(fixtures))
    report_path = output_dir / "report.json"

    for index, path in enumerate(fixtures, start=1):
        fixture_key = _fixture_key(path, input_dir)
        worker_report = worker_dir / Path(fixture_key).with_suffix(".json")
        worker_report.parent.mkdir(parents=True, exist_ok=True)
        worker_report.unlink(missing_ok=True)
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--worker",
            "--input-dir",
            str(input_dir),
            "--output-dir",
            str(output_dir),
            "--only",
            fixture_key,
            "--chord",
            str(args.chord),
            "--angle",
            str(args.angle),
            "--max-edge",
            str(args.max_edge),
            "--max-aspect",
            str(args.max_aspect),
            "--report-path",
            str(worker_report),
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                env=os.environ.copy(),
                timeout=max(1, args.worker_timeout),
            )
        except subprocess.TimeoutExpired as exc:
            detail = ((exc.stderr or exc.stdout or "") if isinstance(exc.stderr or exc.stdout, str) else "").strip()
            entry = {
                "file": fixture_key,
                "path": str(path),
                "status": "error",
                "issues": [
                    f"headless GUI worker timed out after {args.worker_timeout} seconds"
                    + (f": {detail[-1000:]}" if detail else "")
                ],
            }
            report["fixtures"].append(entry)
            report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(
                f"[{index:02d}/{len(fixtures):02d}] {fixture_key}: error - {entry['issues'][0]}",
                flush=True,
            )
            continue
        entry = None
        if worker_report.is_file():
            try:
                worker_data = json.loads(worker_report.read_text(encoding="utf-8"))
                if worker_data.get("fixtures"):
                    entry = worker_data["fixtures"][0]
            except Exception:
                entry = None
        if entry is None:
            detail = (result.stderr or result.stdout or "").strip()
            entry = {
                "file": fixture_key,
                "path": str(path),
                "status": "error",
                "issues": [
                    f"headless GUI worker exited with status {result.returncode}"
                    + (f": {detail[-1000:]}" if detail else "")
                ],
            }
        report["fixtures"].append(entry)
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        summary = "; ".join(entry.get("issues", ())[:2]) if entry.get("issues") else "no automatic findings"
        print(f"[{index:02d}/{len(fixtures):02d}] {fixture_key}: {entry['status']} - {summary}", flush=True)

    report["overview_sheets"] = _write_overview_sheets(report, output_dir)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failures = sum(entry["status"] == "error" for entry in report["fixtures"])
    reviews = sum(entry["status"] == "review" for entry in report["fixtures"])
    skips = sum(entry["status"] == "skip" for entry in report["fixtures"])
    print(
        f"Audit complete: {len(report['fixtures'])} fixture(s), {failures} error(s), "
        f"{reviews} automatic review candidate(s), {skips} skip(s); report={report_path}"
    )
    return 1 if failures else 0


def main() -> int:
    args = _parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    fixtures = sorted(input_dir.rglob("*.brep"))
    fixtures = _select_fixtures(fixtures, input_dir, args.only)
    if not fixtures:
        raise RuntimeError(f"No matching .brep fixtures found under {input_dir}")
    if not args.worker and len(fixtures) > 1:
        return _run_isolated(args, fixtures, input_dir, output_dir)
    if smlib_gui.sm is None:
        raise RuntimeError("_omni_solid did not import; build the repo with a Python 3.12 environment")

    app = smlib_gui.QtWidgets.QApplication.instance()
    if app is None:
        app = smlib_gui.QtWidgets.QApplication(sys.argv[:1])
    window = smlib_gui.MainWindow()
    window._chord_spin.setValue(args.chord)
    window._angle_spin.setValue(args.angle)
    window._max_edge_spin.setValue(args.max_edge)
    window._max_aspect_spin.setValue(args.max_aspect)
    app.processEvents()
    render_settings = smlib_gui.HeadlessRenderSettings(
        chord_height_tolerance=args.chord,
        angle_tolerance_deg=args.angle,
        max_edge_length=args.max_edge,
        max_aspect_ratio=args.max_aspect,
        view_names=VIEWS,
    )

    report = _base_report(args, input_dir, output_dir, len(fixtures))
    report_path = args.report_path.resolve() if args.report_path else output_dir / "report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with smlib_gui.HeadlessRenderSession(render_settings) as renderer:
            for index, path in enumerate(fixtures, start=1):
                fixture_key = _fixture_key(path, input_dir)
                entry = _audit_one(window, app, renderer, path, fixture_key, output_dir)
                report["fixtures"].append(entry)
                summary = "; ".join(entry["issues"][:2]) if entry["issues"] else "no automatic findings"
                print(
                    f"[{index:02d}/{len(fixtures):02d}] {fixture_key}: "
                    f"{entry['status']} - {summary}",
                    flush=True,
                )
                report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    finally:
        window.close()

    failures = sum(entry["status"] == "error" for entry in report["fixtures"])
    reviews = sum(entry["status"] == "review" for entry in report["fixtures"])
    skips = sum(entry["status"] == "skip" for entry in report["fixtures"])
    print(
        f"Audit complete: {len(report['fixtures'])} fixture(s), {failures} error(s), "
        f"{reviews} automatic review candidate(s), {skips} skip(s); report={report_path}"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
