#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Suggest SMLib validation commands from changed paths."""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Recommendation:
    name: str
    first: tuple[str, ...]
    broader: tuple[str, ...]


OCCT_IMPORT_TEST = (
    'env OCCT_TO_USD="$PWD/_build/linux-x86_64/release/occt_to_usd" '
    'USD_TEST_APP="$PWD/_build/linux-x86_64/release/usd_test_app" '
    'OMNISOLID_PLUGIN_PATH="$PWD/_build/schema/omniSolid/resources" '
    'LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:'
    '$PWD/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}" '
    'PYTHONPATH="$PWD/_build/linux-x86_64/release:${PYTHONPATH:-}" '
    './_build/target-deps/python/bin/python3 -m unittest discover '
    '-s tools/occt_to_usd_test -p "test_*.py"'
)

OCCT_EXPORT_TEST = (
    'env USD_TO_OCCT="$PWD/_build/linux-x86_64/release/usd_to_occt" '
    'OCCT_TO_USD="$PWD/_build/linux-x86_64/release/occt_to_usd" '
    'USD_TEST_APP="$PWD/_build/linux-x86_64/release/usd_test_app" '
    'OMNISOLID_PLUGIN_PATH="$PWD/_build/schema/omniSolid/resources" '
    'LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:'
    '$PWD/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}" '
    'PYTHONPATH="$PWD/_build/linux-x86_64/release:${PYTHONPATH:-}" '
    './_build/target-deps/python/bin/python3 -m unittest discover '
    '-s tools/usd_to_occt_test -p "test_*.py"'
)


RECOMMENDATIONS = (
    (
        "prog_test focused suites",
        (
            "tests/prog_test/",
        ),
        Recommendation(
            "prog_test focused suites",
            (
                "make --directory=_compiler/gmake2 prog_test_app config=release_x86_64 -j22 --output-sync",
                "./tests/prog_test/prog_test.sh linux-x86_64 --list-suites",
            ),
            ("./tests/prog_test/prog_test.sh linux-x86_64",),
        ),
    ),
    (
        "SmPyLib / Python bindings",
        (
            "source/SmPyLib/",
            "source/SmPyDevLib/",
        ),
        Recommendation(
            "SmPyLib / Python bindings",
            ("./repo.sh build", "tools/agent/run_smpylib_tests.sh"),
            ("./repo.sh test",),
        ),
    ),
    (
        "SM_API",
        (
            "source/SM_API/",
        ),
        Recommendation(
            "SM_API",
            (
                "./repo.sh build",
                "run a focused Python repro or API-level test for the touched operation",
            ),
            ("./tests/SM_API_test/SM_API_test.sh linux-x86_64", "./repo.sh test"),
        ),
    ),
    (
        "SMLib Python GUI",
        (
            "tools/smlib_gui/",
            "tools/scripts/smlib_gui",
        ),
        Recommendation(
            "SMLib Python GUI",
            ("uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py",),
            (
                "uv run --locked --group gui python tools/scripts/smlib_gui_render.py INPUT "
                "--output-dir /tmp/smlib_gui_renders",
            ),
        ),
    ),
    (
        "OCCT BRep import",
        (
            "source/OCCT_BREP_IMPORT/",
            "tools/occt_to_usd_test/",
        ),
        Recommendation(
            "OCCT BRep import",
            ("./repo.sh build", OCCT_IMPORT_TEST),
            (
                OCCT_EXPORT_TEST,
                "./tests/usd_test/usd_test.sh linux-x86_64",
                "uv run --locked --group gui python tools/scripts/smlib_gui_occt_audit.py",
            ),
        ),
    ),
    (
        "OCCT BRep export",
        (
            "source/OCCT_BREP_EXPORT/",
            "tools/usd_to_occt_test/",
        ),
        Recommendation(
            "OCCT BRep export",
            ("./repo.sh build", OCCT_EXPORT_TEST),
            (OCCT_IMPORT_TEST, "./tests/usd_test/usd_test.sh linux-x86_64"),
        ),
    ),
    (
        "USD bridge",
        (
            "source/SM_API_USD/",
            "source/BREP_SM_USD/",
            "source/BREP_USD_DATA/",
            "source/schema/omniSolid/",
            "tests/SM_API_USD_test/",
            "tests/usd_test/",
        ),
        Recommendation(
            "USD bridge",
            ("./repo.sh build", "run a focused USD import/export repro"),
            (
                "./tests/SM_API_USD_test/SM_API_USD_test.sh linux-x86_64",
                "./tests/usd_test/usd_test.sh linux-x86_64",
            ),
        ),
    ),
    (
        "SMLib kernel",
        (
            "source/SMLib/",
        ),
        Recommendation(
            "SMLib kernel",
            ("./repo.sh build", "run a minimal repro through SM_API or _omni_solid"),
            ("run the relevant C++ suite", "./repo.sh test"),
        ),
    ),
    (
        "Agent docs / tooling",
        (
            ".agents/",
            ".claude",
            "CLAUDE.md",
            "AGENTS.md",
            "tools/agent/",
        ),
        Recommendation(
            "Agent docs / tooling",
            ("rg \"\\.claude\" AGENTS.md .agents source/SmPyLib",),
            (),
        ),
    ),
    (
        "Build / packaging",
        (
            "premake",
            "repo.toml",
            "repo.sh",
            "repo.bat",
            "build.sh",
            "build.bat",
            "tests.sh",
            "tests.bat",
            "tools/CI/",
        ),
        Recommendation(
            "Build / packaging",
            ("./repo.sh build",),
            ("./repo.sh test",),
        ),
    ),
)


PROG_TEST_SUITE_RULES = (
    (("merge", "boolean", "classif"), ("booleans",)),
    (("fillet", "blend", "chamfer"), ("fillets", "local-ops")),
    (("offset", "shell"), ("offset",)),
    (("sweep",), ("sweeps",)),
    (("primitive", "cone", "cylinder", "sphere", "torus"), ("primitives",)),
    (("tess", "poly", "raytracer", "grid"), ("tessellation",)),
    (
        ("trim", "topolog", "brep", "face", "edge", "vertex", "loop", "heal"),
        ("topology", "trimmed-surfaces", "brep-import"),
    ),
    (("surfaceintersector", "ssi"), ("ssi-analytic", "ssi-advanced")),
    (("curve", "cci"), ("cci-advanced",)),
    (("section",), ("section-advanced",)),
    (("silhouette",), ("silhouette-advanced",)),
)


def changed_paths_from_git() -> list[str]:
    commands = (
        ("git", "diff", "--name-only", "HEAD", "--"),
        ("git", "diff", "--name-only", "--"),
        (
            "git",
            "-c",
            "filter.lfs.process=",
            "-c",
            "filter.lfs.clean=cat",
            "-c",
            "filter.lfs.required=false",
            "diff",
            "--name-only",
            "HEAD",
            "--",
        ),
        (
            "git",
            "-c",
            "filter.lfs.process=",
            "-c",
            "filter.lfs.clean=cat",
            "-c",
            "filter.lfs.required=false",
            "diff",
            "--name-only",
            "--",
        ),
    )
    paths: list[str] = []
    for command in commands:
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode == 0:
            paths.extend(line.strip() for line in result.stdout.splitlines() if line.strip())
            break

    untracked = subprocess.run(
        ("git", "ls-files", "--others", "--exclude-standard"),
        text=True,
        capture_output=True,
        check=False,
    )
    if untracked.returncode == 0:
        paths.extend(line.strip() for line in untracked.stdout.splitlines() if line.strip())

    return list(dict.fromkeys(paths))


def classify(path: str) -> list[Recommendation]:
    matches: list[Recommendation] = []
    for _label, prefixes, recommendation in RECOMMENDATIONS:
        for prefix in prefixes:
            if prefix.endswith("/"):
                if path.startswith(prefix):
                    matches.append(recommendation)
                    break
            elif prefix in {"premake"}:
                if Path(path).name.startswith(prefix):
                    matches.append(recommendation)
                    break
            elif path == prefix or path.startswith(prefix):
                matches.append(recommendation)
                break
    return matches


def focused_prog_test_suites(path: str) -> list[str]:
    if not path.startswith("source/SMLib/"):
        return []

    lowered = Path(path).name.lower()
    suites: list[str] = []
    for needles, rule_suites in PROG_TEST_SUITE_RULES:
        if any(needle in lowered for needle in needles):
            suites.extend(rule_suites)
    return suites


def unique(items: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            result.append(item)
    return result


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paths", nargs="*", help="Changed paths to classify instead of reading git diff")
    args = parser.parse_args(argv)

    paths = args.paths if args.paths is not None else changed_paths_from_git()
    paths = [path for path in paths if path]

    if not paths:
        print("No changed paths found. Use --paths to classify explicit files.")
        return 0

    matched: list[Recommendation] = []
    unknown: list[str] = []
    focused_suite_commands: list[str] = []
    for path in paths:
        path_matches = classify(path)
        if path_matches:
            matched.extend(path_matches)
        else:
            unknown.append(path)
        focused_suite_commands.extend(
            f"./tests/prog_test/prog_test.sh linux-x86_64 --suite {suite}"
            for suite in focused_prog_test_suites(path)
        )

    focused_suite_commands = unique(focused_suite_commands)
    if focused_suite_commands:
        matched.append(
            Recommendation(
                "Focused prog_test suites",
                tuple(focused_suite_commands),
                ("./tests/prog_test/prog_test.sh linux-x86_64",),
            )
        )

    matched_names = unique([item.name for item in matched])
    first = unique([command for item in matched for command in item.first])
    broader = unique([command for item in matched for command in item.broader])

    print("Changed paths:")
    for path in paths:
        print(f"  - {path}")

    if matched_names:
        print("\nMatched areas:")
        for name in matched_names:
            print(f"  - {name}")

    if unknown:
        print("\nUnclassified paths:")
        for path in unknown:
            print(f"  - {path}")
        if not first:
            first.append("./repo.sh build")

    print("\nFirst validation:")
    if first:
        for command in first:
            print(f"  - {command}")
    else:
        print("  - no validation required")

    print("\nBroader validation:")
    if broader:
        for command in broader:
            print(f"  - {command}")
    else:
        print("  - none")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
