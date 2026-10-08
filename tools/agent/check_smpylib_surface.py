#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Conservative static checks for the `_omni_solid` Python binding surface."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "source" / "SmPyLib" / "src"
STUB_DIR = REPO_ROOT / "source" / "SmPyLib" / "stubs" / "usd_brep"
MAIN_STUB = STUB_DIR / "__init__.pyi"
USD_STUB = STUB_DIR / "usd.pyi"

MODULE_BIND_RE = re.compile(r"\bm\.def\s*\(\s*\"([A-Za-z_][A-Za-z0-9_]*)\"", re.DOTALL)
STUB_FUNCTION_RE = re.compile(r"^def\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", re.MULTILINE)

IGNORE_SOURCE_FILES = {
    "SmPyMain.cpp",
    "SmPyCommon.h",
    "SmPyError.cpp",
    "SmPyError.h",
    "SmPyEnums.cpp",
    "SmPyTypes.cpp",
    "SmPyPolyBrep.cpp",
}


@dataclass(frozen=True)
class Binding:
    name: str
    file: Path
    line: int
    has_wraps: bool
    is_usd: bool


def line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def parse_bindings(path: Path) -> list[Binding]:
    text = path.read_text(encoding="utf-8")
    matches = list(MODULE_BIND_RE.finditer(text))
    bindings: list[Binding] = []
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        block = text[match.start() : end]
        bindings.append(
            Binding(
                name=match.group(1),
                file=path,
                line=line_number(text, match.start()),
                has_wraps="Wraps:" in block,
                is_usd=path.name == "SmPyUsd.cpp",
            )
        )
    return bindings


def parse_stub_functions(path: Path) -> set[str]:
    return set(STUB_FUNCTION_RE.findall(path.read_text(encoding="utf-8")))


def collect_bindings() -> list[Binding]:
    bindings: list[Binding] = []
    for path in sorted(SRC_DIR.glob("SmPy*.cpp")):
        if path.name in IGNORE_SOURCE_FILES:
            continue
        bindings.extend(parse_bindings(path))
    return bindings


def format_binding(binding: Binding) -> str:
    rel = binding.file.relative_to(REPO_ROOT)
    return f"{rel}:{binding.line}: {binding.name}"


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--allow-missing-wraps",
        action="store_true",
        help="report missing Wraps footers as warnings instead of errors",
    )
    args = parser.parse_args(argv)

    main_stub_functions = parse_stub_functions(MAIN_STUB)
    usd_stub_functions = parse_stub_functions(USD_STUB)
    bindings = collect_bindings()

    main_bindings = {binding.name: binding for binding in bindings if not binding.is_usd}
    usd_bindings = {binding.name: binding for binding in bindings if binding.is_usd}

    missing_main_stubs = sorted(set(main_bindings) - main_stub_functions)
    missing_usd_stubs = sorted(set(usd_bindings) - usd_stub_functions)
    stale_main_stubs = sorted(main_stub_functions - set(main_bindings))
    stale_usd_stubs = sorted(usd_stub_functions - set(usd_bindings))
    missing_wraps = [binding for binding in bindings if not binding.has_wraps]

    errors: list[str] = []
    warnings: list[str] = []

    if missing_main_stubs:
        errors.append("bindings missing from __init__.pyi: " + ", ".join(missing_main_stubs))
    if missing_usd_stubs:
        errors.append("USD bindings missing from usd.pyi: " + ", ".join(missing_usd_stubs))
    if stale_main_stubs:
        warnings.append("stub functions without module binding: " + ", ".join(stale_main_stubs))
    if stale_usd_stubs:
        warnings.append("USD stub functions without module binding: " + ", ".join(stale_usd_stubs))
    if missing_wraps:
        message = "bindings missing Wraps footer:\n" + "\n".join(f"  - {format_binding(item)}" for item in missing_wraps)
        if args.allow_missing_wraps:
            warnings.append(message)
        else:
            errors.append(message)

    print(f"checked {len(main_bindings)} module bindings against {MAIN_STUB.relative_to(REPO_ROOT)}")
    print(f"checked {len(usd_bindings)} USD bindings against {USD_STUB.relative_to(REPO_ROOT)}")

    if warnings:
        print("\nwarnings:")
        for warning in warnings:
            print(f"- {warning}")

    if errors:
        print("\nerrors:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("\nSmPyLib binding surface checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
