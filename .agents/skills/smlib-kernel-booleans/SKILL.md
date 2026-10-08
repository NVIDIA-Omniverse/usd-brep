---
name: smlib-kernel-booleans
description: "Debug solid BRep booleans in SMLib, including entrypoints, dispatch, consumption, degeneracies, and `IntersectInsertRelate()`; not mesh booleans."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Booleans

Use this skill for solid model booleans only. For polygon mesh booleans, use the operation guides directly or create a separate mesh-specific profile.

## Purpose

Diagnose and change exact solid BRep boolean behavior from Python and `SM_API` down through `SmMerge` and `SmTopologyIntersector::IntersectInsertRelate()`.

## Prerequisites

- Confirm the inputs are solid BReps, not `PolyBrep` polygon meshes.
- Have or create a minimal repro that records both operands before any consuming Python call.
- Use `usd-brep-test-selection` to choose focused boolean tests after diagnosis.

## Open First

1. `AGENTS.md` for architecture and search scope.
2. `.agents/operations/booleans.md` for Python/API behavior, options, and consumption semantics.
3. `.agents/docs/errors.md` for `RuntimeError` or `SM_ERR_*` diagnosis.
4. `.agents/operations/queries.md` for manifold checks, topology traversal, bounding boxes, and volume.
5. Do not start from `.agents/operations/polygon_booleans.md` unless the user explicitly says the inputs are `PolyBrep` meshes.

## Source Anchors

- Python: `source/SmPyLib/src/SmPyBooleans.cpp`, `source/SmPyLib/stubs/usd_brep/__init__.pyi`.
- Public API: `source/SM_API/inc/SmApiBrep.h`, `source/SM_API/src/SmApiBrep.cpp`.
- Kernel boolean executive: `source/SMLib/inc/SmMerge.h`, `source/SMLib/src/SmMerge.cpp`.
- Intersect/insert/relate flow: `source/SMLib/inc/SmTopologyIntersector.h`, `source/SMLib/src/SmTopologyIntersector.cpp` (`SmTopologyIntersector::IntersectInsertRelate()`).
- Topology and BRep structure: `source/SMLib/inc/SmBrep.h`, `source/SMLib/src/SmBrep.cpp`.

## Solid Boolean Flow

1. Python calls consume input `Brep` handles and return a new `Brep`, unless an option explicitly preserves a tool.
2. `SmApiBrep.cpp` dispatches to `SmMerge`, selecting `ManifoldBoolean()` when both inputs are manifold solids and `NonManifoldBoolean()` otherwise.
3. The core manifold path calls `m_vTI.IntersectInsertRelate()`, which performs intersection discovery, topology insertion, and relation/classification setup.
4. `SmMerge` then deletes or keeps faces by operation (`UNION`, `DIFFERENCE`, `INTERSECTION`, `MERGE`) and runs post-processing.
5. CSG/list variants still operate on BReps and should be diagnosed with the same solid-input validity checks.

## Workflow

1. Reproduce the failure with the smallest two solids and the same operation order.
2. Record operand validity, overlap/touch relationship, and whether the path should be manifold or non-manifold.
3. Trace Python -> `SmApiBrep.cpp` -> `SmMerge` -> `IntersectInsertRelate()` only after the public contract is clear.
4. Convert the repro into a focused Python or `prog_test` case when the behavior should remain fixed.

## Diagnosis Rules

- First confirm the API call order, especially `boolean_difference(A, B)` where B is the tool.
- Do not inspect consumed Python inputs after a consuming call. Copy first for before/after comparisons.
- Record both inputs: `info()`, manifold status, volume, bounds, face/edge/vertex counts, and expected overlap/touching relationship.
- Look for coincident, tangent, nearly tangent, tiny, sliver, or zero-thickness intersections before changing the algorithm.
- Keep solid booleans distinct from `SmApiPolygons.h`, `SmPolyIntersector`, `SmPolyMerge`, and `poly_boolean_*`.

## Example Requests

```text
Use $smlib-kernel-booleans to debug boolean_difference when the tool just touches a box face.
Use $smlib-kernel-booleans to trace an IntersectInsertRelate classification failure in a manifold union.
```

## Troubleshooting

- If the result is empty or inverted, recheck operand order, operation enum, and face deletion rules.
- If intersection insertion fails, inspect tangencies, coincident faces, and model scale before changing classification.
- If a Python repro crashes after a boolean, verify no consumed input handle is reused.

## Limitations

- Do not use for polygon mesh boolean algorithms or `poly_boolean_*`.
- Do not treat non-manifold dispatch as a bug until input manifold status is measured.

## Validation

Use `usd-brep-test-selection`. Obvious focused coverage is a minimal Python boolean repro, SmPyLib boolean tests, and `prog_test` suite `booleans` for kernel changes.
