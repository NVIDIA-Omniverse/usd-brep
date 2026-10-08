# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import sys
from pathlib import Path

# Ensure the core library (tools/brep_validator) is importable when running from this folder.
sys.path.append(str(Path(__file__).resolve().parents[1]))

from brep_validator import BrepValidator

try:
    from pxr import Usd, Plug
except ImportError as e:
    print(
        "ERROR: Failed to import required `pxr` modules (`Usd`, `Plug`). "
        "Ensure USD is installed and setup_env has been run. "
        "See `Environment Setup` https://openusd.org/release/tut_usd_tutorials.html"
    )
    print(f"Details: {e}")
    raise SystemExit("Fix the environment variables and retry.") from e


def register_omnisolid_plugins() -> None:
    """
    Ensure the OmniSolid schema plugins are registered before traversing the stage.
    Without this, BrepArray prims report an empty type name and are impossible to find.
    """
    env_path = Path(os.environ.get("OMNISOLID_PLUGIN_PATH", "")).expanduser()
    if env_path and env_path.exists():
        Plug.Registry().RegisterPlugins(str(env_path.resolve()))
    else:
        print(
            "WARNING: OMNISOLID_PLUGIN_PATH is not set or does not exist. "
            "BrepArray prim discovery may fail without a valid omniSolid plugin path."
        )


# Register plugins once on import so every CLI execution can see BrepArray prims.
register_omnisolid_plugins()

def validate_brep_arrays(file_path: str):
    """
    Loads a USD file, identifies BrepArray prims, and validates them using BrepValidator.
    :param file_path: Path to the USD file containing BrepArray schema data.
    """

    # Check if the file exists
    if not Path(file_path).exists():
        raise FileNotFoundError(f"USD file not found: {file_path}")

    # Load the USD stage
    stage = Usd.Stage.Open(file_path)
    if not stage:
        raise RuntimeError(f"Failed to open USD file: {file_path}")

    # Instantiate BrepValidator
    try:
        checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
    except Exception as e:
        raise RuntimeError(f"Failed to initialize BrepValidator: {e}") from e

    brep_arrays_found = 0
    prim_range = Usd.PrimRange.Stage(stage, predicate=Usd.PrimAllPrimsPredicate)

    for prim in prim_range:
        if prim.GetTypeName() != "BrepArray":
            continue

        brep_arrays_found += 1
        print(f"Validating BrepArray at prim path: {prim.GetPath()}")
        try:
            checker.CheckPrim(prim)
        except (ValueError, RuntimeError, AttributeError) as e:
            import traceback

            print(f"Error validating prim {prim.GetPath()}: {e}")
            print("Full traceback:")
            traceback.print_exc()
            continue
        except Exception as e:
            import traceback

            print(f"Unexpected error validating prim {prim.GetPath()}: {type(e).__name__}: {e}")
            print("Full traceback:")
            traceback.print_exc()
            continue
        print("Validation complete.")


    if brep_arrays_found == 0:
        print("No BrepArray prims found in the USD file.")

    issues = checker.GetIssues()
    if not issues:
        print("Validation passed: No issues found.")

    for issue in issues:
        print(f"[{issue.severity}] {issue.message} (Rule: {issue.rule.__name__})")
        if issue.suggestion:
            print(f"  Suggestion: {issue.suggestion.message}")
        if issue.requirement:
            print(f"  Requirement: {issue.requirement.code}")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description=(
            "Validate USD files containing BrepArray prims. "
            "\n\nThis script identifies and validates BrepArray prims in a given USD file using the BrepValidator tool.\n"
            "Usage:\n"
            "  python brep_array_handler.py <path_to_usd_file>\n"
            "\nExample:\n"
            "  python brep_array_handler.py ./example_file.usd"
        ),
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "usd_file_path",
        type=str,
        help="Path to the USD file to validate (must contain BrepArray prims).",
    )
    args = parser.parse_args()

    # Get the user-supplied USD file path
    usd_file_path = args.usd_file_path

    try:
        validate_brep_arrays(usd_file_path)
    except RuntimeError as e:
        print(f"Error: {e}")

# Ensure the script runs when executed directly
if __name__ == "__main__":
    main()
