---
name: smlib-kernel-operations
description: "Diagnose failed SMLib operations across `_omni_solid`, `SM_API`, and `source/SMLib` when errors, invalid topology, or unexpected geometry appear. Use for general or cross-cutting kernel failures; prefer the operation-specific skill (`smlib-kernel-booleans`, `smlib-kernel-fillets`, `smlib-kernel-tessellation`, `smlib-kernel-drop-curve`, `smlib-kernel-healing`) when the failure clearly belongs to one of those, and do not use for API/binding maintenance."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Operations

Use this skill for failing booleans, fillets, sweeps, offsets, tessellation, stitching, trimming, intersections, and other geometry operations.

## Purpose

Build a reproducible diagnosis path from a user-visible failure through Python, `SM_API`, and the kernel layer, then hand off to a specialist skill when the operation family is clear.

## Prerequisites

- Start with the exact exception, bad output, or invalid topology symptom.
- Have or create a minimal `_omni_solid` repro unless the failure is already isolated below `SM_API`.
- Use `usd-brep-test-selection` once changed files or the failing operation family are known.
- For ownership, context, cache, or incorporated NLib questions, read
  [the lifetime/context/cache reference](references/lifetime-context-cache.md).
- For offset or shell internals, read
  [the offset/shell reference](references/offset-shell-internals.md).
- For BRep healing or stale healer properties, use
  [the healing skill](../smlib-kernel-healing/SKILL.md).

## Workflow

1. Decode the error and identify the public API layer involved.
2. Reduce to a minimal deterministic repro.
3. Record cheap invariants before changing tolerances or geometry.
4. Inspect operation docs, then `SM_API`, then kernel source in that order.
5. Convert a confirmed bug into a focused regression test.

## Start With The Error

- If there is a `_omni_solid RuntimeError`, open `.agents/docs/errors.md` before grepping source.
- Extract the wrapped entry (usually an `SmApi*` function), symbolic `SM_ERR_*` status, numeric
  code, and earliest or most specific useful `Kernel trace:` entry. In a simple `SER()` unwind, the
  first entry is often the deepest site.
- Open `.agents/operations/INDEX.md`, then the matching operation guide.
- Read the matching public header in `source/SM_API/inc/` to confirm parameter contracts and output ownership.
- If the problem is clearly in a specialty area, use the matching specialist skill after the first repro: booleans, filleting, tessellation, drop curve, or BRep healing.

## Build A Minimal Repro

Create or reduce a Python repro using `_omni_solid` unless the failure is clearly below the C API. Prefer analytic primitives and deterministic dimensions.

For each input and output, record cheap invariants:

- `info()`, `face_count()`, `edge_count()`, `vertex_count()`
- `bounding_box()`, `center()`
- `volume()` and `is_manifold_solid()` for closed solids
- `tessellate()` plus `PolyBrep.is_manifold_solid()` when mesh output matters

Respect consumption semantics: do not inspect a consumed input after a consuming operation. Copy first when a before/after comparison is needed.

## Diagnosis Order

1. Confirm the Python call matches the binding contract and units.
2. Check the operation guide for tolerance order, known degeneracies, and recommended alternatives.
3. Check the `SM_API` wrapper for status handling, argument conversion, and output ownership.
4. Inspect kernel code under `source/SMLib/` only after the public API layer is understood.
5. When source search is needed, start with the wrapped `SmApi*` name, the status symbol, and the deepest kernel trace function/file.

## Interpret BRep Validation Diagnostics

- Separate structural validity, defining-geometry agreement, derived UV-trim correctness, and
  geometric quality. A bad trim or UV-based orientation report alone does not prove bad topology.
  Verify topology and 3D endpoint pairing, then isolate trim regeneration; use the
  [drop-curve skill](../smlib-kernel-drop-curve/SKILL.md) for that path.
- Read the failing predicate and `SmAssertArray` details, including for `ValidatePointers`, which
  also emits geometry/tolerance reports. Continue deeper checks only after establishing that the
  required graph links are safe to traverse.
- Record actual gap distance, units, threshold, affected edge length, and gap/length ratio. Keep
  stored tolerances distinct from measured gaps. A gap comparable to the edge length suggests an
  orientation/endpoint mismatch; compare normal and swapped endpoint pairing, also considering
  the endpoint chord for curved edges.
- When diagnosing an operation, compare input geometry or an earlier model state with the result
  if available. A pre-existing gap does not establish that the operation introduced a defect.
  Assess geometric-quality findings against the intended operation's requirements, and do not
  change tolerances or topology solely to silence reports.

See the canonical BRep reference for
[UV trim semantics](../smlib-brep-model/references/brep-format.md#uv-trim-curves) and
[gap/tolerance interpretation](../smlib-brep-model/references/brep-format.md#interpreting-gap-and-tolerance-reports).

## Source Anchors

- Python bindings: `source/SmPyLib/src/SmPy*.cpp`, `source/SmPyLib/src/SmPyCommon.h`.
- Public API wrappers: `source/SM_API/inc/`, `source/SM_API/src/`.
- Kernel internals: `source/SMLib/inc/`, `source/SMLib/src/`.
- Lifetime, context, cache, and incorporated NLib internals:
  [references/lifetime-context-cache.md](references/lifetime-context-cache.md).
- Offset/shell implementation stages:
  [references/offset-shell-internals.md](references/offset-shell-internals.md).
- Error decoder: `.agents/docs/errors.md`.
- Operation guides: `.agents/operations/`.

## Common Failure Axes

- Degenerate geometry: zero dimensions, coincident faces, tangent overlaps, poles, tiny sliver faces.
- Tolerance mismatch: modeling tolerance too tight/loose for the feature scale.
- Topology mismatch: non-manifold input passed to a solid operation.
- Lifetime or mutation error: consumed handle reused, mutator expected to return a new handle, or missing copy before mutation.
- Binding conversion error: wrong enum, tuple arity, degree/radian mismatch, or missing stub/doc update.

## Example Requests

```text
Use $smlib-kernel-operations to diagnose a RuntimeError from boolean_union on two touching solids.
Use $smlib-kernel-operations to reduce a sweep that returns a non-manifold result.
```

## Troubleshooting

- If no `Kernel trace:` is present, use the wrapped entry and binding-site fallback location to
  choose the first source anchor.
- If a repro is flaky, remove random geometry, normalize scale, and record topology counts at each step.
- If a specialist skill applies, use it after the first minimal repro and error decode rather than continuing broadly.

## Limitations

- Do not use as a substitute for operation-specific skills once the failure is clearly boolean, fillet, tessellation, drop-curve, or BRep healing.
- Do not debug consumed Python handles after a consuming operation; copy first.

## Validation

- Keep the minimal repro and convert it to a focused test when the behavior should remain fixed.
- Run the focused test first, then the suite selected by `usd-brep-test-selection`.
- Use `python3 tools/agent/suggest_tests.py --paths <changed paths...>` when validation scope is unclear.
- Report any operation guide update that would have prevented the confusion.
