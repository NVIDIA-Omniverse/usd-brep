---
name: smlib-sm-api-use
description: "Use native C++ SmApi* calls safely: operation choice, ownership, errors, and Boolean/query performance. Not wrapper or kernel changes."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Native API Use

## Purpose

Build, review, and diagnose native C++ callers of existing `SmApi*` wrappers.
Respect each entry point's ownership and status contract, and distinguish
modeling cost from optional queries. Use tested compositions rather than
adding kernel behavior to solve a caller-side workflow problem.

## When to Use

Use this skill for native API integration, operand lifetime reviews, status
triage, or profiling an application's sequence of API calls. Example requests:

- "Build and run the native Boolean/bounds example."
- "Our repeated-cutter loop is slow; separate Boolean time from query time."
- "Review who deletes each operand after SmApiBooleanDifference succeeds."
- "A native SmApi call failed; which outputs and operands can I still use?"
- "Choose ordinary or tight bounds for viewport framing of a Brep."
- "Add correctness tests for this SM_API composition without fixed face counts."

For Python callers, use `smlib-python-api-use`; its exceptions and handle
invalidation are not the native pointer/status contract.

For a caller using kernel `SmSurface::EvaluateSTEP` rather than a `SmApi*`
wrapper, read the
[STEP derivative example and migration guide](../../../Examples/SMLib/README.md#step-surface-derivatives).
Angular STEP derivatives are already per degree; do not apply the old
`pi/180` compensation again. Ordinary `SmApiEvaluateSurface*` calls use the
surface's ordinary parameters, not STEP degrees. This route is usage guidance,
not authorization to change an evaluator or add a wrapper.

## Prerequisites

- A source checkout or matching SMLib SDK headers and libraries. Source headers
  are in `source/SM_API/inc` and `source/SMLib/inc`; packaged headers are in
  `include/SM_API` and `include/SMLib`.
- A C++17 toolchain for the example. Link `SM_API` and `SMLib`, and configure
  their runtime dependencies. The repository build handles this; independent
  builds and other platforms are covered by the
  [example README](../../../Examples/SM_API/README.md#build-and-run-from-a-source-checkout).
- Initialize the default API context with `SmApiCreateContext()` before
  constructing objects in the single-threaded example. Keep objects' contexts
  alive through their destruction; this example is not a threading recipe.

## Workflow

1. Identify the operation and required outputs. Read the matching
   [operation guide](../../operations/INDEX.md), then its public header for the
   exact C++ overload, statuses, and ownership. Do not translate Python handle
   behavior directly into raw-pointer cleanup.
2. Establish input ownership and output initialization before calling. For the
   example's `SmApiBooleanDifference`, success deletes B and returns modified
   A; failure leaves both allocated, possibly modified. Release B's owner only
   on success and keep A's existing owner. Other entry points may differ.
3. Check every returned `SmApiStatus` against `SM_SUCCESS`, not zero. On error,
   record the call and status, clean up according to its contract, and stop
   before using unassigned outputs. Do not retry possibly modified operands.
4. If profiling, time construction, Boolean calls, bounds, tessellation, and
   property integration separately. Skip unneeded queries. Prefer ordinary
   `SmApiBrepBoundingBox(..., FALSE, ...)` for framing or broad-phase culling;
   request tight bounds only when the tighter fit is needed. Bounds queries
   do not improve the Boolean result or change its modeling tolerance.
5. Validate the required geometry outside the operation timers. Use expected
   volume, bounds, and manifold status for the plate example. Do not demand
   fixed topology partitions or timing ratios; these observations are not a
   complete validity proof.

## Runnable Example

The canonical code is
[`Examples/SM_API/boolean_bounds.cpp`](../../../Examples/SM_API/boolean_bounds.cpp).
It builds a plate with sixteen through-holes under three caller policies:
no per-cut bounds, ordinary bounds, and tight bounds. It demonstrates operand
cleanup and checks API statuses, analytic volume, manifold status, bounds,
and unchanged observables after querying. Adapt that code to the consumer's
geometry; do not copy its timings into a speed guarantee.

From the repository root on Linux:

```bash
./repo.sh build -r --target sm_api_boolean_bounds_example
LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:${LD_LIBRARY_PATH:-}" \
    ./_build/linux-x86_64/release/sm_api_boolean_bounds_example
```

Expected: three timing summaries ending in `checks passed`, and exit status 0.
Nonzero exit status means an API call or correctness check failed; the message
identifies it. Ordinary and tight boxes need not coincide. No-query mode still
runs untimed final qualification queries.

## Validation

`SM_API_test` compiles and runs the same example source. On Linux:

```bash
./repo.sh build -r --target SM_API_test_app
./tests/SM_API_test/SM_API_test.sh linux-x86_64
```

The suite must report success, including `TestSmBooleanBoundsExample`. For new
compositions, add the runnable code to the appropriate `Examples/` category,
index it in that README, and exercise its correctness claims in the owning
suite. Use `usd-brep-test-selection` for other changed layers; report commands
and results actually obtained, not just suggested validation.

## Troubleshooting

- **Missing executable, headers, or shared libraries:** verify the build target,
  platform/configuration, include paths, and runtime library search path in the
  example README. Resolve setup failures before diagnosing the geometry.
- **Non-success status:** consult the selected header and status definitions in
  `SmMessages.h`; reduce to a deterministic primitive-based repro. Preserve the
  original inputs with copies when needed, not by reusing failed operands.
- **Double deletion or stale pointer:** check whether an operand was consumed,
  whether the result aliases it, and whether an owner still holds a deleted
  pointer. Follow the overload's contract; changing modeling tolerances will
  not repair an ownership error.
- **Slow operation or unexpected bounds:** separate query time first. Ordinary
  boxes can be loose and tolerance-padded; neither tighter boxes nor finer
  property integration automatically improves the constructed solid.
- **Correct caller, persistent geometry failure:** hand off the minimal repro,
  status, parameters, and failed invariants to the operation-specific kernel
  skill. Do not silently replace this usage task with kernel changes.

## Limitations

- This skill uses existing native wrappers. Wrapper changes belong to
  `smlib-sm-api-maintain`; kernel algorithm changes need a kernel skill and
  explicit task scope.
- The current runnable example qualifies one manifold, through-hole plate
  construction, not every Boolean configuration or every API entry point.
- Do not generalize its ownership rules to Boolean2D or other overloads, its
  timings to other workloads, or its single-threaded context setup to workers.
