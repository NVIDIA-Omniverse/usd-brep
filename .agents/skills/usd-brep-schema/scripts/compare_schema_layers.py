#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compare two USDA schema layers after normalization through pxr.Sdf."""

from __future__ import annotations

import argparse
import difflib
import sys
from pathlib import Path


def _import_sdf(usd_python_path: Path | None):
    search_paths: list[Path] = []
    if usd_python_path is not None:
        search_paths.append(usd_python_path)

    repo_root = Path(__file__).resolve().parents[4]
    search_paths.append(repo_root / "_build" / "target-deps" / "usd" / "release" / "lib" / "python")

    if usd_python_path is not None and usd_python_path.is_dir():
        explicit_path = str(usd_python_path)
        if explicit_path in sys.path:
            sys.path.remove(explicit_path)
        sys.path.insert(0, explicit_path)

    try:
        from pxr import Sdf

        return Sdf
    except (ImportError, OSError) as initial_error:
        for search_path in search_paths:
            if search_path.is_dir() and str(search_path) not in sys.path:
                sys.path.insert(0, str(search_path))
            try:
                from pxr import Sdf

                return Sdf
            except (ImportError, OSError):
                continue

        searched = ", ".join(str(path) for path in search_paths)
        raise RuntimeError(
            "Could not import pxr.Sdf. Configure the repository USD environment or pass "
            f"--usd-python-path. Searched: {searched}"
        ) from initial_error


def _normalized_layer_text(sdf, path: Path) -> str:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"Schema layer not found: {resolved}")

    layer = sdf.Layer.FindOrOpen(str(resolved))
    if layer is None:
        raise RuntimeError(f"pxr.Sdf could not open schema layer: {resolved}")
    return layer.ExportToString()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare two USDA schema layers after pxr.Sdf normalization. "
            "Comments and formatting are ignored; authored declarations and metadata are compared."
        )
    )
    parser.add_argument("canonical", type=Path, help="Canonical schema.usda path")
    parser.add_argument("candidate", type=Path, help="Candidate schema.usda path")
    parser.add_argument(
        "--usd-python-path",
        type=Path,
        help="Directory containing the pxr Python package, such as USD_ROOT/lib/python",
    )
    parser.add_argument("--quiet", action="store_true", help="Suppress success output")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        sdf = _import_sdf(args.usd_python_path)
        canonical = _normalized_layer_text(sdf, args.canonical)
        candidate = _normalized_layer_text(sdf, args.candidate)
    except (FileNotFoundError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if canonical == candidate:
        if not args.quiet:
            print("Schema layers match after pxr.Sdf normalization.")
        return 0

    diff = difflib.unified_diff(
        canonical.splitlines(keepends=True),
        candidate.splitlines(keepends=True),
        fromfile=str(args.canonical),
        tofile=str(args.candidate),
    )
    sys.stdout.writelines(diff)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
