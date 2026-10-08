---
name: smlib-kernel-fillets
description: "Use when diagnosing or fixing SMLib fillet, chamfer, blend, or fillet-removal failures involving selected edges/corners, radius feasibility, continuity/cross-section settings, solver or corner handling, and topology insertion. Do not use for generic booleans unless the failure occurs inside a fillet/chamfer/blend path."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Fillets

Use this skill when edge blends, chamfers, variable-radius fillets, fillet removal, previews, or local blend topology fail or produce invalid geometry.

## Purpose

Diagnose and change fillet/chamfer/blend behavior from Python and `SM_API` through `SmFilletExecutive`, solver selection, corner handling, topology insertion, and post-processing.

## Prerequisites

- Know the target edges or vertices and verify they come from the live `Brep` being mutated.
- Record radius, cross-section, continuity, and nearby face sizes before kernel debugging.
- Use `usd-brep-test-selection` for focused fillet and local operation validation.

## Open First

1. `AGENTS.md` for architecture and search scope.
2. `.agents/operations/fillets.md` for exposed operations and parameter semantics.
3. [The fillet architecture reference](references/architecture.md) for solver, generator, corner,
   and topology-insertion internals.
4. `.agents/docs/errors.md` for `RuntimeError` or `SM_ERR_*` diagnosis.
5. `.agents/operations/queries.md` for edge, vertex, face, and topology selection helpers.

## Source Anchors

- Python: `source/SmPyLib/src/SmPyFillets.cpp`, `source/SmPyLib/stubs/usd_brep/__init__.pyi`.
- Simple all-edge APIs: `source/SM_API/inc/SmApiFillets.h`, `source/SM_API/src/SmApiFillets.cpp`.
- Selective/per-edge APIs: `source/SM_API/inc/SmApiBrep.h`, `source/SM_API/src/SmApiBrep.cpp`.
- Executive and solvers: `source/SMLib/inc/SmFilletExecutive.h`, `source/SMLib/src/SmFilletExecutive.cpp`, `source/SMLib/inc/SmFilletSolver.h`, `source/SMLib/src/SmFilletSolver.cpp`, `source/SMLib/inc/SmFilletStandardSolver.h`, `source/SMLib/src/SmFilletStandardSolver.cpp`.
- Corners and topology: `source/SMLib/inc/SmFilletCorner.h`, `source/SMLib/src/SmFilletCorner.cpp`, `source/SMLib/inc/SmFilletGeom.h`, `source/SMLib/src/SmFilletGeom.cpp`, `source/SMLib/src/SmFilletIntersector.cpp`.
- Removal: `source/SMLib/inc/SmFilletRemoval.h`, `source/SMLib/src/SmFilletRemoval.cpp`.

## Workflow

1. Reproduce on the smallest solid that preserves the selected edge/corner neighborhood.
2. Capture topology and selection handles before mutation.
3. Verify geometric feasibility before stepping through solver code.
4. Trace executive -> solver -> corner/intersector -> topology insertion when feasibility is confirmed.
5. Add a focused repro or `prog_test` case for any fixed failure mode.

## Diagnosis Rules

- Verify selected `Edge` and `Vertex` handles come from the same live `Brep` being mutated.
- Check radius against adjacent face dimensions before kernel debugging. Too-large radii and tangent/near-coincident faces are first-class failure causes.
- Distinguish cross-section type (`LINEAR`, `CIRCULAR`, `BLEND`) from continuity (`G1`, `G2`, `G3`) and variable radius laws.
- Record topology before and after: edge/face counts, manifold status, bounding box, and failed edge identifiers when possible.
- Treat filleting as a topology insertion and post-processing operation; boolean-style merge/intersection failures can surface inside `SmFilletExecutive`.
- Preserve mutator semantics: Python fillet functions modify the `Brep` and return the same handle unless the local binding documents otherwise.

## Example Requests

```text
Use $smlib-kernel-fillets to reduce a variable-radius fillet failure on selected box edges.
Why does a chamfer succeed on one corner but fail on an adjacent tangent edge?
Debug sm.fillet_edges raising RuntimeError after selecting edges from a copied Brep.
Decide whether a circular fillet radius is too large for the adjacent face spans.
Trace a G2 blend through SmFilletExecutive corner handling.
Investigate invalid topology after a nominally successful edge fillet.
Diagnose a fillet removal path that leaves non-manifold local topology.
```

## Troubleshooting

- If every selected edge fails, check handle ownership and mutator semantics first.
- If only small or tangent features fail, compare radius against adjacent face span and intersection tolerance.
- If topology becomes invalid after a nominal success, inspect corner insertion and boolean-style cleanup paths.

## Limitations

- Do not use for generic boolean failures unless they occur inside a fillet/chamfer/blend path.
- Do not assume a solver bug until selected topology and radius feasibility are confirmed.

## Validation

Use `usd-brep-test-selection`. Obvious focused coverage is a minimal Python fillet repro, SmPyLib fillet tests, and `prog_test` suites `fillets` then `local-ops` for kernel changes.
