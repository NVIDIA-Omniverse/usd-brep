---
name: usd-brep-test-selection
description: "Choose validation for SMLib changes or CI failures; map changed files to focused build/test commands. Do not use for implementation design."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Test Selection

Use this skill whenever validation scope is unclear.

## Purpose

Select the smallest credible build, test, or sanity-check path for an SMLib change, then decide when broader validation is justified.

## Prerequisites

- Inspect changed files or the failing CI job before selecting tests.
- Know whether the task is docs-only, Python binding, `SM_API`, kernel, USD, OCCT translation, GUI/headless rendering, build, or tooling.
- Use implementation/debug skills first when the code behavior is not understood.

## Example Requests

```text
Use $usd-brep-test-selection to choose tests for a change in source/SmPyLib/src/SmPyBooleans.cpp.
Use $usd-brep-test-selection to triage a failing prog_test booleans CI job.
Use $usd-brep-test-selection to decide whether .agents-only edits need kernel tests.
```

## First Inspect Impact

Run:

```bash
git status --short
git diff --name-only
```

Classify touched files by layer before choosing tests.

Convenience helper:

```bash
python3 tools/agent/suggest_tests.py
python3 tools/agent/suggest_tests.py --paths source/SmPyLib/src/SmPyBooleans.cpp
```

## Test Map

| Changed area | First validation | Broader validation |
|---|---|---|
| `source/SmPyLib/` (stable bindings) | `./repo.sh build`, then `tools/agent/run_smpylib_tests.sh` | `./repo.sh test` |
| `source/SmPyDevLib/`, `stubs/smlib_dev/` | `./repo.sh build`, then `tools/agent/run_smpylib_tests.sh --target test_smlib_dev` | `./repo.sh test` plus `tools/scripts/smlib_gui_smoke.py` when pick/draw/tests change |
| `tools/smlib_gui/`, `tools/scripts/smlib_gui*.py` | `uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py` | Run `smlib_gui_render.py --jobs N` on a focused recursive fixture tree; add SmPyLib tests if bindings changed |
| `source/SM_API/` or `source/SM_API/inc/` | `./repo.sh build`, focused Python repro or API-level test for the operation | `./tests/SM_API_test/SM_API_test.sh linux-x86_64`, then `./repo.sh test` |
| `source/SMLib/` | `./repo.sh build`, minimal repro through `SM_API` or `_omni_solid`, then a focused `prog_test` suite when mapped below | Relevant C++ suite plus `./repo.sh test` |
| `tests/prog_test/` | `make --directory=_compiler/gmake2 prog_test_app config=release_x86_64 -j22 --output-sync`, then focused `prog_test` suites | `./tests/prog_test/prog_test.sh linux-x86_64` |
| `source/SM_API_USD/`, `source/BREP_SM_USD/`, `source/BREP_USD_DATA/`, `source/schema/omniSolid/`, USD test harnesses | `./repo.sh build`, focused USD import/export repro | `./tests/SM_API_USD_test/SM_API_USD_test.sh linux-x86_64` and `./tests/usd_test/usd_test.sh linux-x86_64` |
| `source/OCCT_BREP_IMPORT/`, `tools/occt_to_usd_test/` | `./repo.sh build`, then the focused importer command below | Exporter round trip, `./tests/usd_test/usd_test.sh linux-x86_64`, and OCCT headless corpus audit |
| `source/OCCT_BREP_EXPORT/`, `tools/usd_to_occt_test/` | `./repo.sh build`, then the focused exporter command below | Importer suite and `./tests/usd_test/usd_test.sh linux-x86_64` |
| `.agents/`, `AGENTS.md`, docs only | markdown/path sanity checks; no kernel tests | none required |
| `tools/agent/` | run/lint the changed script or a targeted tool check | relevant script validation or CI check |
| Build files, packaging, `premake*`, `repo.toml` | `./repo.sh build` | `./repo.sh test` |

## SmPyLib Focus Command

Use the helper when available:

```bash
tools/agent/run_smpylib_tests.sh
tools/agent/run_smpylib_tests.sh --target test_omni_solid.TestBooleans
tools/agent/run_smpylib_tests.sh --target test_smlib_dev
```

If `_omni_solid` / `_smlib_dev` exist under `_build/linux-x86_64/release`, use this local command:

```bash
OMNISOLID_PLUGIN_PATH="$PWD/_build/schema/omniSolid/resources" \
LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:$PWD/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}" \
PYTHONPATH="$PWD/_build/linux-x86_64/release" \
./_build/target-deps/python/bin/python3 -m unittest discover -s source/SmPyLib/tests -p "test_*.py" -v
```

For a single class, add the test directory to `PYTHONPATH` and use a target such as:

```bash
OMNISOLID_PLUGIN_PATH="$PWD/_build/schema/omniSolid/resources" \
LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:$PWD/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}" \
PYTHONPATH="$PWD/_build/linux-x86_64/release:$PWD/source/SmPyLib/tests" \
./_build/target-deps/python/bin/python3 -m unittest test_omni_solid.TestBooleans -v
```

## OCCT Converter Focus Commands

The converter subprocesses need the built schema plugin and shared libraries. Mirror the hard-gate environment in `tests.sh`:

```bash
platform=linux-x86_64
export OMNISOLID_PLUGIN_PATH="$PWD/_build/schema/omniSolid/resources"
export LD_LIBRARY_PATH="$PWD/_build/$platform/release:$PWD/_build/target-deps/usd/release/lib:$PWD/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$PWD/_build/$platform/release:${PYTHONPATH:-}"
export OCCT_TO_USD="$PWD/_build/$platform/release/occt_to_usd"
export USD_TEST_APP="$PWD/_build/$platform/release/usd_test_app"
./_build/target-deps/python/bin/python3 -m unittest discover \
  -s tools/occt_to_usd_test -p "test_*.py"
```

For exporter round trips, retain that environment and run:

```bash
export USD_TO_OCCT="$PWD/_build/$platform/release/usd_to_occt"
./_build/target-deps/python/bin/python3 -m unittest discover \
  -s tools/usd_to_occt_test -p "test_*.py"
```

Use `./tests.sh linux-x86_64` for the complete canonical gate. Use the headless OCCT audit only after these data-flow checks; rendered screenshots do not replace schema, `AssertValid`, or topology round-trip assertions.

## prog_test Focus Commands

`prog_test_app` supports focused C++ kernel regression suites:

```bash
./tests/prog_test/prog_test.sh linux-x86_64 --list-suites
./tests/prog_test/prog_test.sh linux-x86_64 --suite booleans
./tests/prog_test/prog_test.sh linux-x86_64 --suite fillets
./tests/prog_test/prog_test.sh linux-x86_64 --suite tessellation
```

For changes to `prog_test` itself, do not rely on process success alone. The
historical validation signal is the generated debug output logs. Build the debug
app, run the default suite from `tests/prog_test` on trunk and the branch, then
compare the generated `OutputFiles/progTest_*_long.txt` and `progTest_*.txt`
logs. Treat pointer addresses, wall-clock timings, and tiny last-digit floating
point drift as volatile; added, removed, reordered, or renamed test output is a
real behavior change unless it was intended. Full debug logs are noisy enough
that broad geometry, polygon-output, or warning/message ordering diffs need a
control check: rerun the same branch or compare focused suites before
attributing the difference to the code change.

Use these defaults for obvious `source/SMLib/` changes:

| Change area | Focused suite |
|---|---|
| merge, booleans, classification | `booleans` |
| fillet, blend, chamfer | `fillets`, then `local-ops` |
| offset, shell | `offset` |
| sweeps | `sweeps` |
| primitive creation, analytic primitives | `primitives` |
| tessellation, polybrep, polygon operations | `tessellation` |
| topology, trimming, BRep import/heal | `topology`, `trimmed-surfaces`, `brep-import` |
| SSI / surface intersectors | `ssi-analytic`, `ssi-advanced` |
| CCI / curve intersectors | `cci-advanced` |
| section / silhouette | `section-advanced`, `silhouette-advanced` |

## Troubleshooting

- If a focused test cannot import `_omni_solid`, run `./repo.sh build` or report the missing build output.
- If `prog_test` output differs broadly, rerun a focused suite or compare against a same-branch control before assigning causality.
- If CI failed outside the changed layer, inspect logs before expanding to full `./repo.sh test`.
- If an OCCT visual audit fails, locate the first incorrect data-flow stage before selecting kernel tessellation tests.
- If docs or `.agents/` changed only, prefer path/format validation and report that kernel tests were not run.

## Limitations

- Do not use this skill to design the code change itself.
- Do not run broad tests by default when a focused validation covers the changed behavior.

## Selection Rules

- Start with the smallest test that exercises the changed behavior.
- Add broader tests when the change touches shared ownership, tolerance handling, public API signatures, build files, or object lifetime.
- For OCCT translator changes, test import and export together whenever the modified contract has a symmetric representation.
- If a test cannot run because the build output is missing, run `./repo.sh build` before declaring the test blocked.
- Always report tests that were not run and why.
