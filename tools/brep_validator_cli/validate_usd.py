# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

#
# Run the asset validator (full rule set) and BrepValidator on a single .usda/.usdc/.usd file.
#
# Thin CLI wrapper over the importable brep_validator.validate_file() API so the run/pass-fail logic
# lives in one place. Usage (from repo root; use the wrapper so Python and deps are set up):
#   tools\brep_validator_cli\validate_usd.bat path\to\file.usda   (Windows)
#   tools/brep_validator_cli/validate_usd.sh path/to/file.usda    (Linux)

import argparse
import sys
from pathlib import Path

# Locate the brep_validator module for both the smlib repo layout (this file is
# tools/brep_validator_cli/validate_usd.py, with tools/brep_validator alongside) and the packaged
# layout (brep_validator_cli and brep_validator are siblings under the package root).
_THIS_FILE = Path(__file__).resolve()
_module_root_candidates = [_THIS_FILE.parents[2] / "tools", _THIS_FILE.parents[1]]
_MODULE_ROOT = next(
    (root for root in _module_root_candidates if (root / "brep_validator").is_dir()),
    _module_root_candidates[0],
)
if str(_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(_MODULE_ROOT))

try:
    from brep_validator import validate_file
except ImportError as e:
    print("ERROR: could not import the brep_validator API.", file=sys.stderr)
    print(
        "  Run from repo root: tools\\brep_validator_cli\\validate_usd.bat <path> (Windows) or "
        "tools/brep_validator_cli/validate_usd.sh <path> (Linux).",
        file=sys.stderr,
    )
    print(f"  Details: {e}", file=sys.stderr)
    raise SystemExit(1) from e


def main():
    parser = argparse.ArgumentParser(
        description="Run BrepValidator and the USD asset validator on a single USD file."
    )
    parser.add_argument("usd_file", type=Path, help="Path to .usd, .usda, or .usdc file")
    parser.add_argument(
        "--brep-only",
        action="store_true",
        help="Run only the BrepArray pass; skip the full asset-validator pass",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=None,
        metavar="N",
        help="Parallel worker count for the BrepArray pass (default: half of CPU cores; use 1 for serial)",
    )
    args = parser.parse_args()

    path = args.usd_file.expanduser().resolve()
    if not path.is_file():
        print(f"Not a file: {path}", file=sys.stderr)
        return 2
    if path.suffix.lower() not in (".usda", ".usdc", ".usd"):
        print(f"Expected .usda, .usdc, or .usd: {path}", file=sys.stderr)
        return 2
    if args.workers is not None and args.workers < 1:
        print("ERROR: --workers must be a positive integer", file=sys.stderr)
        return 2

    result = validate_file(
        path.as_posix(),
        workers=args.workers,
        brep_only=args.brep_only,
    )

    if result.brep_array_count == 0:
        print(f"[BrepValidator] No BrepArray prims in {result.identifier}, skipping BA rules", flush=True)
    if result.messages:
        print(f"\n[Validator] {result.failure_count} issue(s) in {result.identifier}:", flush=True)
        for message in result.messages:
            print(f"  {message}", flush=True)
        if result.issue_summary:
            print(f"[Validator] Issue summary ({result.failure_count} total):", flush=True)
            for issue_type, count in sorted(
                result.issue_summary.items(), key=lambda item: (-item[1], item[0])
            ):
                print(f"  {issue_type}: {count}", flush=True)
    else:
        print("[Validator] No issues found. Passed.", flush=True)

    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
