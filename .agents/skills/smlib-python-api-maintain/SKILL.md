---
name: smlib-python-api-maintain
description: "Add or change stable `_omni_solid` Python bindings in `source/SmPyLib`, including pybind11, docstrings, stubs, ownership, mutation semantics, and tests. For debugger-only `_smlib_dev` bindings, use `smlib-python-dev-api-maintain`. Do not use for using the existing Python API without binding changes (use `smlib-python-api-use`) or for designing the underlying C `SM_API` wrapper (use `smlib-sm-api-maintain`)."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Python API Maintenance

Use this skill to author or review **stable** Python-facing API changes on `_omni_solid`. For using the existing Python API without changing bindings, use `smlib-python-api-use`. For debugger topology, Draw extraction, or `prog_test` facades, use `smlib-python-dev-api-maintain`.

## Purpose

Keep `_omni_solid` bindings, docstrings, stubs, tests, and operation docs synchronized with the underlying `SM_API` contract. Do not add debugger-only symbols (`Loop`, `Loopuse`, `Edgeuse`, `topology_pick_ray`, `draw.*`, `tests.*`) to this module.

## Prerequisites

- Confirm the C API contract is stable; use `smlib-sm-api-maintain` first if the `SM_API` wrapper is still changing.
- Have build output available or be prepared to run `./repo.sh build` before Python tests.
- Use `usd-brep-test-selection` before broad validation.

## Open First

1. `AGENTS.md` for repo scope and ignored folders.
2. `source/SmPyLib/DOCSTRING_STYLE.md` before editing docstrings.
3. `.agents/operations/INDEX.md`, then the operation guide matching the binding category.
4. The matching public header under `source/SM_API/inc/` or `source/SM_API_USD/inc/`.
5. `source/SmPyLib/src/SmPyCommon.h` and `source/SmPyLib/src/SmPyMain.cpp` for shared registration patterns.
6. The matching binding file under `source/SmPyLib/src/SmPy*.cpp`.
7. `source/SmPyLib/stubs/usd_brep/__init__.pyi` and, for USD bindings, `source/SmPyLib/stubs/usd_brep/usd.pyi`.
8. `source/SmPyLib/tests/test_omni_solid.py`.

## Authoring Rules

- Register function-only bindings through `BoundModule` so the API appears both flat (`sm.create_box`) and in the category submodule (`sm.primitives.create_box`), unless the existing local pattern says otherwise.
- Keep type, enum, USD, `PolyBrep`, and tessellation registration consistent with `SmPyMain.cpp` and `SmPyCommon.h`.
- Use `CHECK_STATUS(...)` for kernel calls that return `SmStatus` or `SmApiStatus`.
- Return kernel-owned handles with the existing local return policy pattern; Python does not own `Brep`, `Face`, `Edge`, `Vertex`, `Curve`, `Surface`, or `PolyBrep`.
- If a binding needs `Loop`, `Loopuse`, `Edgeuse`, topology picking, or kernel `Draw()`, stop and use `smlib-python-dev-api-maintain` instead.
- Preserve the documented mutation contract: pure producers leave inputs unchanged, consuming operations invalidate named inputs, mutators return the same handle or `(handle, ...diagnostics)`.
- Keep mutator diagnostic tuple returns handle-first.
- For every Python-facing function change, update the stub, docstring, and tests unless the change is internal only.
- Keep docstrings in the documented section order and end with `Wraps: SmApi...`.
- Update the relevant `.agents/operations/` guide if the Python behavior, defaults, or mutation semantics change.

## Example Requests

```text
Use $smlib-python-api-maintain to expose a new SmApiBrep function in _omni_solid with stubs and tests.
Use $smlib-python-api-maintain to fix a binding/docstring/stub mismatch for tessellate.
Use $smlib-python-api-maintain to review mutation semantics for a Python mutator that returns diagnostics.
```

## Common Change Flow

1. Locate the API category in `.agents/operations/INDEX.md`.
2. Read the C header comments for parameter and return contracts.
3. Confirm ownership and mutation behavior from the existing binding and operation guide.
4. Edit the binding, stub, and tests as one unit.
5. Add a small Python test that checks type, topology, mutation/consumption behavior, and a representative numeric result.
6. Run `python3 tools/agent/check_smpylib_surface.py` to catch binding/stub/docstring drift.
7. If a `RuntimeError` appears, switch to `smlib-kernel-operations`.

## Troubleshooting

- If a new symbol imports but is missing from stubs, rerun surface checks and update `__init__.pyi` with the exact signature.
- If a docstring contradicts behavior, trust the C header and operation guide, then update binding docs and tests together.
- If `CHECK_STATUS` raises `RuntimeError`, switch to `smlib-kernel-operations` unless the status handling itself is wrong.
- If tests fail to import `_omni_solid`, build first and use the environment pattern below.

## Limitations

- Do not use for pure Python scripts or examples that call existing APIs; use `smlib-python-api-use`.
- Do not expose kernel-owned pointers with new lifetime rules unless neighboring binding patterns support them.

## Validation

Prefer the narrowest credible validation first:

```bash
./repo.sh build
```

If `_omni_solid` is already built, use the environment pattern from `tests.sh` to run SmPyLib tests:

```bash
tools/agent/run_smpylib_tests.sh
tools/agent/run_smpylib_tests.sh --target test_omni_solid.TestBooleans
```

Fallback explicit command:

```bash
OMNISOLID_PLUGIN_PATH="$PWD/_build/schema/omniSolid/resources" \
LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:$PWD/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}" \
PYTHONPATH="$PWD/_build/linux-x86_64/release" \
./_build/target-deps/python/bin/python3 -m unittest discover -s source/SmPyLib/tests -p "test_*.py" -v
```

For broad confidence, run:

```bash
./repo.sh test
```
