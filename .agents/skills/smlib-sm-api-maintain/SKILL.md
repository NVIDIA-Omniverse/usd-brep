---
name: smlib-sm-api-maintain
description: "Use when adding, changing, or debugging an SmApi wrapper in source/SM_API or source/SM_API_USD, including declarations, status propagation, ownership, context selection, and kernel dispatch. Do not use for Python-only bindings, existing-API usage, or kernel algorithm fixes; use the corresponding Python or kernel skill."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib SM_API Maintenance

Use this skill to author the C-style C++ wrapper layer that Python and downstream integrations call
through.

## Purpose

Design and update the stable `SM_API` contract that sits between Python/integrations and the C++ kernel. Keep public declarations, wrapper implementation, status behavior, ownership, and operation docs aligned.

## Prerequisites

- Work from a local SMLib checkout with `AGENTS.md` available.
- Know whether the task is adding a new public API, changing an existing contract, or only diagnosing wrapper behavior.
- Use `usd-brep-test-selection` before running expensive builds or tests.

## Open First

1. `AGENTS.md` for architecture, style, and search scope.
2. `.agents/operations/INDEX.md`, then the operation guide for the API family.
3. The public header in `source/SM_API/inc/` or `source/SM_API_USD/inc/`.
4. The matching implementation in `source/SM_API/src/` or `source/SM_API_USD/src/`.
5. The kernel header and source that actually implement the operation under `source/SMLib/inc/` and `source/SMLib/src/`.
6. `smlib-python-api-maintain` if the public API needs Python exposure.

## API Rules

- Use `SmApi*` names, `SMAPI_EXPORT`, and `SmApiStatus` in public headers.
- Follow nearby wrapper style for comments, parameter annotations, `SmApiGetOrCreateContext()`, `SER(...)`, `SM_ASSERT_VALID`, and debug drawing blocks.
- Keep `SmContext` an implementation detail: SM_API provides a process-global default through
  `SmApiGetOrCreateContext()`, while selected wrappers use an input object's context or a dedicated
  output context. Ordinary API/Python users should not allocate or configure one.
- Keep outputs constructed with the selected context association consistent with neighboring
  wrappers. Context association is non-owning, so separately document caller ownership, transfer,
  and deletion obligations.
- Trace ownership explicitly across the kernel call, C wrapper, and Python policy. Python commonly
  uses `py::nodelete`, so garbage collection does not establish kernel reclamation semantics.
- Treat BRep-owned face surfaces and edge curves as borrowed. Do not independently delete or mutate
  them through a wrapper.
- Set output pointers/references deterministically before returning on failure when local patterns do so.
- Do not hide kernel preconditions in Python. Encode them in the C header comment and operation guide.
- If the wrapper consumes, mutates, or preserves inputs in a way Python exposes, update the binding, stub, docstring, tests, and operation guide together.

## Source Anchors

- BRep operations: `source/SM_API/inc/SmApiBrep.h`, `source/SM_API/src/SmApiBrep.cpp`.
- Curves: `source/SM_API/inc/SmApiCurves.h`, `source/SM_API/src/SmApiCurves.cpp`.
- Fillets: `source/SM_API/inc/SmApiFillets.h`, `source/SM_API/src/SmApiFillets.cpp`.
- Queries: `source/SM_API/inc/SmApiQueries.h`, `source/SM_API/src/SmApiQueries.cpp`.
- USD: `source/SM_API_USD/inc/SmApiUsd.h`, `source/SM_API_USD/src/SmApiUsd.cpp`.

## Change Flow

1. Map the user request to an existing operation family and nearby `SmApi*` wrapper.
2. Confirm the kernel call's ownership, context requirements, valid input topology, and failure statuses.
3. Add or change the public declaration and implementation together.
4. Add Python exposure only after the C API contract is stable.
5. Add a focused test or repro at the lowest layer that can observe the behavior.

## Example Requests

```text
Use $smlib-sm-api-maintain to add an SmApi curve projection wrapper and document output ownership.
Use $smlib-sm-api-maintain to review why a wrapper returns success with a null output pointer.
Which context should a new SmApi tessellation wrapper use for its output?
Trace an SM_ERR from an SmApi wrapper to the owning kernel operation without changing the kernel.
Change an SmApi ownership contract and identify every header, implementation, test, and doc update.
Add a source/SM_API_USD wrapper while preserving status propagation and output initialization.
Review whether a borrowed face surface can be returned safely through an existing SmApi function.
Use $smlib-sm-api-maintain to expose an existing kernel operation without adding Python bindings.
```

## Troubleshooting

- For unexpected `SM_ERR_*`, verify wrapper argument validation, `SER(...)` propagation, and output initialization before changing kernel code.
- For ownership confusion, compare neighboring wrappers and public header comments, then update the operation guide if Python will expose the behavior.
- For binding/test failures after an API change, switch to `smlib-python-api-maintain`.

## Limitations

- Do not use this skill for Python-only usage questions; use `smlib-python-api-use`.
- Do not invent a new high-level API until an existing kernel entrypoint and ownership model are understood.

## Validation

Run `./repo.sh build` for API changes. Use `usd-brep-test-selection` for focused operation tests and broader validation.
