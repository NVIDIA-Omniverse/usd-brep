#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Recursively render SMLib GUI model files without opening a window."""

from __future__ import annotations

import argparse
from concurrent.futures import CancelledError, Future, ThreadPoolExecutor, as_completed
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback


SCRIPT_PATH = Path(__file__).resolve()
REPO = SCRIPT_PATH.parent.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

# These values must be set before importing Qt or PyVista through smlib_gui.
os.environ.pop("DISPLAY", None)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("PYVISTA_OFF_SCREEN", "true")
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "smlib_gui_matplotlib"))
os.environ.setdefault("MESA_SHADER_CACHE_DIR", str(Path(tempfile.gettempdir()) / "smlib_gui_mesa"))
Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
Path(os.environ["MESA_SHADER_CACHE_DIR"]).mkdir(parents=True, exist_ok=True)

from tools.smlib_gui.headless import (  # noqa: E402
    DEFAULT_VIEW_NAMES,
    HeadlessRenderSession,
    HeadlessRenderSettings,
    SUPPORTED_LOAD_EXTENSIONS,
    SUPPORTED_VIEW_NAMES,
    discover_model_files,
    output_directory_for,
)


_CANCELLED = threading.Event()
_LIVE_WORKERS: set[subprocess.Popen] = set()
_LIVE_WORKERS_LOCK = threading.Lock()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input",
        type=Path,
        help="A supported model file or a directory to search recursively.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output root. By default, renders are written beside each input file.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="JSON report path. Defaults to smlib_gui_render_report.json under the output root.",
    )
    parser.add_argument("--chord", type=float, default=0.05, help="Chord-height tessellation tolerance.")
    parser.add_argument("--angle", type=float, default=25.0, help="Angular tessellation tolerance in degrees.")
    parser.add_argument("--max-edge", type=float, default=0.0, help="Maximum tessellation edge length.")
    parser.add_argument("--max-aspect", type=float, default=0.0, help="Maximum tessellation triangle aspect ratio.")
    parser.add_argument("--width", type=int, default=900, help="Width of each named-view image.")
    parser.add_argument("--height", type=int, default=900, help="Height of each named-view image.")
    parser.add_argument(
        "--view",
        dest="views",
        action="append",
        choices=sorted(SUPPORTED_VIEW_NAMES),
        help="View to render; repeat for multiple views. The default renders all four views.",
    )
    parser.add_argument(
        "--save-views",
        action="store_true",
        help="Keep the individual named-view images in addition to the composite.",
    )
    parser.add_argument("--parallel", action="store_true", help="Use parallel instead of perspective projection.")
    parser.add_argument("--no-brep-edges", action="store_true", help="Do not overlay exact BRep edges.")
    parser.add_argument(
        "--worker-timeout",
        type=int,
        default=int(os.environ.get("SMLIB_GUI_RENDER_TIMEOUT_SEC", "300")),
        help="Maximum seconds allowed for each isolated model worker.",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=1,
        help="Number of model files to render concurrently (default: 1).",
    )
    parser.add_argument(
        "--no-isolation",
        action="store_true",
        help="Render in this process instead of isolating each model in a worker.",
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--worker-report", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--worker-staging-dir", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be at least 1")
    if args.no_isolation and args.jobs != 1:
        parser.error("--jobs greater than 1 requires isolated workers")
    return args


def _settings_from_args(args: argparse.Namespace) -> HeadlessRenderSettings:
    return HeadlessRenderSettings(
        chord_height_tolerance=args.chord,
        angle_tolerance_deg=args.angle,
        max_edge_length=args.max_edge,
        max_aspect_ratio=args.max_aspect,
        view_names=tuple(args.views or DEFAULT_VIEW_NAMES),
        window_size=(args.width, args.height),
        parallel_projection=args.parallel,
        show_brep_edges=not args.no_brep_edges,
    )


def _publish(path: Path, output_dir: Path) -> Path:
    """Move one finished image out of the staging directory."""
    destination = output_dir / path.name
    destination.unlink(missing_ok=True)
    shutil.move(str(path), str(destination))
    return destination


def _object_entry(obj) -> dict:
    entry = {
        "name": obj.name,
        "kind": obj.kind,
        "mesh_points": len((obj.mesh or {}).get("points", ())),
        "mesh_face_values": len((obj.mesh or {}).get("faces", ())),
    }
    if getattr(obj, "mesh_error", ""):
        entry["mesh_error"] = obj.mesh_error
    return entry


def _render_one(source: Path, output_dir: Path,
                settings: HeadlessRenderSettings,
                save_views: bool = False,
                staging_root: Path | None = None) -> dict:
    started = time.monotonic()
    entry = {
        "source": str(source),
        "output_dir": str(output_dir),
        "status": "error",
        "error": "",
    }
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        # Render into staging and publish only once every view and the contact
        # sheet exist, so a failed or killed render leaves no partial images.
        staging = tempfile.mkdtemp(
            prefix="render_",
            dir=str(staging_root) if staging_root is not None else None,
        )
        staging_dir = Path(staging)
        try:
            with HeadlessRenderSession(settings) as renderer:
                result = renderer.render_file(source, staging_dir)
            views: dict[str, Path] = {}
            if save_views:
                for name, path in result.view_paths.items():
                    views[name] = _publish(path, output_dir)
            contact_sheet = _publish(result.contact_sheet, output_dir)
        finally:
            shutil.rmtree(staging_dir, ignore_errors=True)
        entry.update(
            {
                "status": "rendered",
                "objects": [_object_entry(obj) for obj in result.objects],
                "views": {name: str(path) for name, path in views.items()},
                "contact_sheet": str(contact_sheet),
            }
        )
    except Exception as exc:
        entry["error"] = str(exc)
        entry["traceback"] = traceback.format_exc()
    finally:
        entry["elapsed_seconds"] = time.monotonic() - started
    return entry


def _worker_command(args: argparse.Namespace, source: Path, output_dir: Path,
                    worker_report: Path) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT_PATH),
        str(source),
        "--output-dir",
        str(output_dir),
        "--worker",
        "--worker-report",
        str(worker_report),
        "--worker-staging-dir",
        str(worker_report.parent),
        "--chord",
        str(args.chord),
        "--angle",
        str(args.angle),
        "--max-edge",
        str(args.max_edge),
        "--max-aspect",
        str(args.max_aspect),
        "--width",
        str(args.width),
        "--height",
        str(args.height),
    ]
    for view_name in args.views or DEFAULT_VIEW_NAMES:
        command.extend(("--view", view_name))
    if args.parallel:
        command.append("--parallel")
    if args.no_brep_edges:
        command.append("--no-brep-edges")
    if args.save_views:
        command.append("--save-views")
    return command


def _worker_failure(source: Path, output_dir: Path, message: str) -> dict:
    return {
        "source": str(source),
        "output_dir": str(output_dir),
        "status": "error",
        "error": message,
    }


def _terminate_worker(process: subprocess.Popen) -> None:
    """Stop one worker and any native child process it started."""
    if process.poll() is not None:
        return
    try:
        if os.name == "nt":
            process.terminate()
        else:
            os.killpg(process.pid, signal.SIGTERM)
    except OSError:
        return
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        if os.name == "nt":
            process.kill()
        else:
            os.killpg(process.pid, signal.SIGKILL)
    except OSError:
        pass


def _cancel_pending(futures: dict) -> None:
    """Refuse further work and stop every worker already running."""
    _CANCELLED.set()
    for future in futures:
        future.cancel()
    with _LIVE_WORKERS_LOCK:
        running = list(_LIVE_WORKERS)
    for process in running:
        _terminate_worker(process)


def _run_isolated(args: argparse.Namespace, source: Path,
                  output_dir: Path, worker_report: Path) -> dict:
    if _CANCELLED.is_set():
        return _worker_failure(source, output_dir, "Headless render cancelled before this file started")
    worker_report.unlink(missing_ok=True)
    command = _worker_command(args, source, output_dir, worker_report)
    timeout = max(1, args.worker_timeout)
    try:
        # A separate session lets a timeout or Ctrl-C kill the whole worker
        # process group instead of orphaning native children.
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=os.environ.copy(),
            start_new_session=True,
        )
    except OSError as exc:
        return _worker_failure(source, output_dir, f"Could not start headless render worker: {exc}")

    with _LIVE_WORKERS_LOCK:
        _LIVE_WORKERS.add(process)
    try:
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            _terminate_worker(process)
            stdout, stderr = process.communicate()
            detail = (stderr or stdout or "").strip()
            message = f"Headless render worker timed out after {timeout} seconds"
            if detail:
                message += f": {detail[-1000:]}"
            return _worker_failure(source, output_dir, message)
    finally:
        with _LIVE_WORKERS_LOCK:
            _LIVE_WORKERS.discard(process)

    if worker_report.is_file():
        try:
            entry = json.loads(worker_report.read_text(encoding="utf-8"))
            if isinstance(entry, dict):
                return entry
        except (OSError, json.JSONDecodeError):
            pass

    if _CANCELLED.is_set():
        return _worker_failure(source, output_dir, "Headless render worker cancelled")
    detail = (stderr or stdout or "").strip()
    message = f"Headless render worker exited with status {process.returncode}"
    if detail:
        message += f": {detail[-1000:]}"
    return _worker_failure(source, output_dir, message)


def _default_report_path(input_path: Path, output_root: Path | None) -> Path:
    if output_root is not None:
        directory = output_root
    elif input_path.is_dir():
        directory = input_path
    else:
        directory = input_path.parent
    return directory / "smlib_gui_render_report.json"


def _write_report(report: dict, report_path: Path) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def _run_worker(args: argparse.Namespace, settings: HeadlessRenderSettings) -> int:
    source = args.input.expanduser().resolve()
    output_dir = (args.output_dir or source.parent).expanduser().resolve()
    staging_root = (
        args.worker_staging_dir.expanduser().resolve()
        if args.worker_staging_dir is not None
        else None
    )
    entry = _render_one(
        source,
        output_dir,
        settings,
        save_views=args.save_views,
        staging_root=staging_root,
    )
    if args.worker_report is not None:
        _write_report(entry, args.worker_report.expanduser().resolve())
    return 0 if entry["status"] == "rendered" else 1


def _print_result(index: int, total: int, source: Path, input_path: Path,
                  entry: dict) -> None:
    try:
        display_name = source.relative_to(input_path).as_posix()
    except ValueError:
        display_name = source.name
    if input_path.is_file():
        display_name = source.name
    detail = entry.get("contact_sheet") or entry.get("error", "")
    print(
        f"[{index}/{total}] {display_name}: {entry['status']}"
        + (f" - {detail}" if detail else ""),
        flush=True,
    )


def _render_sequential(args: argparse.Namespace, sources: list[Path], input_path: Path,
                       output_root: Path | None, settings: HeadlessRenderSettings,
                       report: dict, report_path: Path, staging_root: Path) -> bool:
    """Render every source in this process; return True if interrupted."""
    for index, source in enumerate(sources, start=1):
        output_dir = output_directory_for(source, input_path, output_root)
        try:
            entry = _render_one(
                source,
                output_dir,
                settings,
                save_views=args.save_views,
                staging_root=staging_root,
            )
        except KeyboardInterrupt:
            return True
        report["files"].append(entry)
        _write_report(report, report_path)
        _print_result(index, len(sources), source, input_path, entry)
    return False


def _render_parallel(args: argparse.Namespace, sources: list[Path], input_path: Path,
                     output_root: Path | None, report: dict, report_path: Path,
                     staging_root: Path) -> bool:
    """Render sources in isolated workers; return True if interrupted."""
    entries: list[dict | None] = [None] * len(sources)
    interrupted = False
    executor = ThreadPoolExecutor(max_workers=min(args.jobs, len(sources)))
    futures: dict[Future, tuple[int, Path, Path]] = {}
    try:
        for index, source in enumerate(sources, start=1):
            output_dir = output_directory_for(source, input_path, output_root)
            output_dir.mkdir(parents=True, exist_ok=True)
            worker_report = staging_root / f"worker_{index:06d}.json"
            future = executor.submit(
                _run_isolated,
                args,
                source,
                output_dir,
                worker_report,
            )
            futures[future] = (index, source, output_dir)

        for future in as_completed(futures):
            index, source, output_dir = futures[future]
            try:
                entry = future.result()
            except CancelledError:
                continue
            except Exception as exc:
                entry = _worker_failure(source, output_dir, str(exc))
            entries[index - 1] = entry
            report["files"] = [item for item in entries if item is not None]
            _write_report(report, report_path)
            _print_result(index, len(sources), source, input_path, entry)
    except KeyboardInterrupt:
        interrupted = True
        _cancel_pending(futures)
    finally:
        executor.shutdown(wait=True, cancel_futures=True)
    report["files"] = [item for item in entries if item is not None]
    _write_report(report, report_path)
    return interrupted


def main() -> int:
    args = _parse_args()
    settings = _settings_from_args(args)
    if args.worker:
        return _run_worker(args, settings)

    # Each coordinator run starts uncancelled, so an interrupted earlier run in
    # this process cannot short-circuit the workers dispatched below.
    _CANCELLED.clear()
    input_path = args.input.expanduser().resolve()
    output_root = args.output_dir.expanduser().resolve() if args.output_dir else None
    try:
        sources = discover_model_files(input_path)
    except RuntimeError as exc:
        print(f"Render error: {exc}", file=sys.stderr)
        return 2
    if not sources:
        supported = ", ".join(sorted(SUPPORTED_LOAD_EXTENSIONS))
        print(f"Render error: no supported files found under {input_path} ({supported})", file=sys.stderr)
        return 2

    report_path = (
        args.report.expanduser().resolve()
        if args.report
        else _default_report_path(input_path, output_root)
    )
    report = {
        "input": str(input_path),
        "output_root": str(output_root) if output_root else None,
        "supported_extensions": sorted(SUPPORTED_LOAD_EXTENSIONS),
        "settings": asdict(settings),
        "jobs": args.jobs,
        "file_count": len(sources),
        "files": [],
    }

    with tempfile.TemporaryDirectory(prefix="smlib_gui_render_workers_") as tmpdir:
        staging_root = Path(tmpdir)
        if args.no_isolation:
            interrupted = _render_sequential(
                args, sources, input_path, output_root, settings,
                report, report_path, staging_root,
            )
        else:
            interrupted = _render_parallel(
                args, sources, input_path, output_root,
                report, report_path, staging_root,
            )

    failures = sum(entry.get("status") != "rendered" for entry in report["files"])
    succeeded = len(report["files"]) - failures
    if interrupted:
        print(
            f"Render cancelled: {succeeded}/{len(sources)} succeeded before the interrupt; "
            f"report={report_path}",
            file=sys.stderr,
        )
        return 130
    print(
        f"Render complete: {succeeded}/{len(sources)} succeeded, "
        f"{failures} failed; report={report_path}"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
