<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# brep_validator_cli

Internal CLI entry point for running the BrepArray validator. This is for developer use; production consumers should use the `brep_validator` library directly.

## Usage

From the repository root on Linux:

```bash
source tools/brep_validator_cli/setup_env.sh
python tools/brep_validator_cli/brep_array_handler.py "path/to/your/file.usda"

# Quick smoke test
python tools/brep_validator_cli/brep_array_handler.py --help
```

From the repository root in Windows PowerShell:

```powershell
.\tools\brep_validator_cli\setup_env.ps1
python tools/brep_validator_cli/brep_array_handler.py "path\to\your\file.usda"
```

The setup script prefers the bundled Python in `_build/target-deps/python` (matching USD) and sets `LD_LIBRARY_PATH`/`Path` so libpython is found. If you use a different Python, ensure its lib directory is on `LD_LIBRARY_PATH`. If `OMNISOLID_PLUGIN_PATH` is unset, the setup script sets it to `_build/schema/omniSolid/resources` when that directory exists (run `./prebuild.sh` to generate it).

### Full asset validator + BrepValidator (validate_usd.py)

Runs the full omni.asset_validator rule set plus BrepValidator on a single USD file. **Use the wrapper scripts** so the repo’s target-deps Python, USD, and OMNISOLID_PLUGIN_PATH are set automatically. The validator itself (`usd_validation_nvidia`) must be installed into that Python with `pip install "usd-validation-nvidia>=1.22.0,<2"`.

From the repository root on Linux:

```bash
tools/brep_validator_cli/validate_usd.sh path/to/file.usda
```

From the repository root on Windows:

```bat
tools\brep_validator_cli\validate_usd.bat path\to\file.usda
```

On success the script prints `[Validator] No issues found. Passed.` and exits 0; on validation failures it prints each issue and exits 1.

If you see "pxr not found", run the repo build so that `_build/target-deps/usd` and `_build/target-deps/python` exist. If you see "omni.asset_validator not found" / "usd_validation_nvidia not found", install it into the repo Python with `pip install "usd-validation-nvidia>=1.22.0,<2"`.

## Environment

For **brep_array_handler.py**, run `setup_env.ps1` or `setup_env.sh` first, or set:

- `PYTHONPATH`: Must include `source/` (the script prepends this when run in place). For full USD/pxr support, include USD `lib/python` (setup_env does this).
- `OMNISOLID_PLUGIN_PATH`: Required for BrepArray discovery (setup_env sets it to `_build/schema/omniSolid/resources` if unset).
- USD binaries/libs on PATH/LD_LIBRARY_PATH (setup_env scripts set these).

For **validate_usd.py**, use the wrapper scripts (`validate_usd.bat` / `validate_usd.sh`); they set PYTHONPATH, PATH, `OMNISOLID_PLUGIN_PATH`, and run with `_build/target-deps/python` so no separate setup_env is needed. The validator is resolved from the Python environment (`pip install "usd-validation-nvidia>=1.22.0,<2"`). If you run `validate_usd.py` directly, use a Python that can import both `pxr` and `usd_validation_nvidia` (e.g. after running setup_env).

## Notes
- The CLI wraps the library’s `BrepValidator`; validation output is printed to stdout.
- The CLI ships in the release package as `brep_validator_cli/`. The `validate_usd` wrappers expect a repo checkout, so from a release package run `validate_usd.py` directly, with Python 3.12 and `usd-validation-nvidia` installed (`pip install "usd-validation-nvidia>=1.22.0,<2"`). From the package root, on Linux:

  ```bash
  LD_LIBRARY_PATH="$PWD/lib:$PWD/extraLibs" PYTHONPATH="$PWD/usdpy" \
      python3 brep_validator_cli/validate_usd.py path/to/file.usda
  ```

  On Windows, put `lib` and `extraLibs` on `PATH` instead of `LD_LIBRARY_PATH`.

