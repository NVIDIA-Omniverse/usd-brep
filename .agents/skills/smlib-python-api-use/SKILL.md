---
name: smlib-python-api-use
description: "Use existing `_omni_solid` Python APIs without changing bindings; covers operation choice, mutation semantics, diagnostics, and RuntimeError triage. Do not use for adding or modifying bindings, stubs, or docstrings (use `smlib-python-api-maintain`) or for kernel-level operation failures (use `smlib-kernel-operations`)."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Python API Use

Use this skill to use the existing `_omni_solid` Python API correctly from scripts, tests, notebooks, and examples.

## Purpose

Write, review, or debug Python code that calls existing `_omni_solid` functions while respecting operation docs, stubs, ownership, mutation, and error-triage contracts. For debugger topology pick, loopuse/edgeuse walks, Draw overlays, or in-process `prog_test`, import `_smlib_dev as smdev` (after `_omni_solid`) and see `.agents/operations/dev_topology.md`.

## Prerequisites

- Have an installed or built `_omni_solid` module when running code.
- Know whether the task is API use; switch to `smlib-python-api-maintain` for binding, stub, or docstring edits.
- Use `usd-brep-test-selection` when changing files or choosing validation.

## Open First

1. `AGENTS.md` for repo conventions, ignored folders, and Python quick start.
2. `.agents/operations/INDEX.md`, then the operation guide matching the task.
3. `.agents/docs/errors.md` when a call raises `RuntimeError`.
4. `source/SmPyLib/stubs/usd_brep/__init__.pyi` and `source/SmPyLib/stubs/usd_brep/usd.pyi` when exact Python signatures or return types matter. For debugger APIs, also `source/SmPyDevLib/stubs/smlib_dev/__init__.pyi`.
5. `source/SmPyLib/tests/test_omni_solid.py` for executable examples.

## Example Requests

```text
Use $smlib-python-api-use to write a Python repro for a boolean failure without reusing consumed inputs.
Use $smlib-python-api-use to review whether this tessellate script handles PolyBrep mesh arrays correctly.
Use $smlib-python-api-use to round-trip a BRep through native `.smb` file I/O.
Use $smlib-python-api-use to triage a RuntimeError from drop_curve_to_surface.
```

## Usage Rules

- Prefer the operation guide over source when choosing parameters, tolerance order, or workflow.
- Treat points and vectors as 3-tuples. Use keyword-only parameters where the guide marks them, especially enum operations.
- Honor the docstring mutation line: pure producers leave inputs unchanged, consuming operations invalidate named inputs, and mutators return the same handle for chaining.
- Call `brep.copy()` before mutators or consumers when the original must survive.
- Choose diagnostic cost deliberately: counts and ordinary `bounding_box()` are lightweight; `volume()` integrates geometry, and `info()` includes a volume query. Do not put them or tight bounds in a hot loop without measuring their cost. See the [Boolean/bounds guidance](../../operations/booleans.md#separate-boolean-cost-from-optional-bounds-queries).
- For mesh output, use `tessellate()` -> `PolyBrep` -> `to_mesh_arrays()` and the mesh/USD guidance in `.agents/docs/generate_asset.md`.
- Use native SMLib BRep file I/O for `.smb` round-trips: save with
  `brep.write_to_file(path, ascii=True)`, load with
  `sm.read_brep_from_file(path)` or `sm.Brep.read_from_file(path)`. Treat load
  as a pure producer; it auto-detects ASCII versus binary native files, returns
  a new `Brep`, and does not mutate existing handles. Pass `ascii=True` or
  `ascii=False` only when intentionally validating a known file format.
- Keep exact BRep booleans (`boolean_union`, `boolean_difference`, `boolean_intersection`) distinct from polygon mesh booleans (`poly_boolean_*`).

## Debug Flow

1. Build a minimal Python repro with analytic primitives and deterministic dimensions.
2. Record input and output invariants with query methods before changing tolerances or geometry.
3. For `RuntimeError`, extract the wrapped entry (usually an `SmApi*` function), status, and earliest
   or most specific useful `Kernel trace:` entry. If no trace is present, use the binding-site
   fallback location.
4. Switch to `smlib-kernel-operations` when diagnosis requires `source/SM_API/` or `source/SMLib/`.

## Troubleshooting

- If import fails, confirm the build and environment variables before changing code.
- If `read_brep_from_file` is absent, rebuild `_omni_solid` or switch to `smlib-python-api-maintain` to check the binding/stub state.
- If a result is invalid, record query invariants before altering tolerances or geometry.
- If a function is absent or the stub disagrees with runtime behavior, switch to `smlib-python-api-maintain`.
- If `RuntimeError` points into `source/SMLib`, switch to `smlib-kernel-operations` after extracting status and trace.

## Limitations

- Do not edit pybind11 bindings, stubs, or docstrings under this skill.
- Do not inspect consumed Python handles after a consuming operation; copy first.

## Validation

For Python-only usage advice, a focused runnable snippet or existing test reference is enough. If files change, use `usd-brep-test-selection` to choose validation.
