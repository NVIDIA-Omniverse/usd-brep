<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SM_API Examples

Native C++ examples using the public `SmApi*` wrappers. Kernel headers are
needed for their argument types and caller-owned object destruction; these
examples do not call kernel modeling algorithms directly.

## Boolean Construction and Bounding-Box Cost

[`boolean_bounds.cpp`](boolean_bounds.cpp) subtracts sixteen disjoint cylindrical
cutters from a plate. It repeats the construction with no per-cut bounds query,
ordinary bounds (`bTight=FALSE`), and tight bounds (`bTight=TRUE`). Each run starts
from fresh geometry.

The Boolean call does not require a caller-side bounding-box query. If the
consumer does not need bounds, omit that query. For framing or broad-phase
culling, prefer ordinary bounds; request tight bounds when their tighter fit
is needed. These are read-only geometry queries, although they may populate
internal caches. Changing the bounds mode does not change Boolean parameters
or modeling tolerance.

### Build and Run from a Source Checkout

From the repository root on Linux:

```bash
./repo.sh build -r --target sm_api_boolean_bounds_example
LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:${LD_LIBRARY_PATH:-}" \
    ./_build/linux-x86_64/release/sm_api_boolean_bounds_example
```

Use the matching platform directory on other supported platforms. On Windows,
build with `repo.bat build -r --target sm_api_boolean_bounds_example` and run
`_build\windows-x86_64\release\sm_api_boolean_bounds_example.exe` with the USD
dependency `release\bin` directory on `PATH`.

The package also includes the example sources under `Examples/SM_API` and the
executable under `bin`. Run it with the package's normal shared-library setup.
To compile independently, build `boolean_bounds.cpp` as C++17 with
`SM_BOOLEAN_BOUNDS_EXAMPLE_STANDALONE` defined, include the `SM_API` and `SMLib`
headers, and link `SM_API` and `SMLib` plus their normal runtime dependencies.

### Ownership and Failures

The example is single-threaded and initializes the default API context before
creating objects. For the `SmApiBooleanDifference` overload used here:

- Success modifies and returns the primary operand and deletes the cutter.
  The primary `unique_ptr` keeps ownership; the cutter owner releases its
  already-deleted pointer without deleting it again.
- Failure leaves both operands allocated, possibly modified. The example
  reports the failing status and lets their owners destroy them. It does not
  retry failed operands or inspect an unassigned result.
- Every status is checked against `SM_SUCCESS` (not zero).

Do not infer these rules for other overloads, Boolean2D, or Python handles;
consult the selected entry point's ownership contract.

### What the Example Qualifies

Each variant must produce a manifold solid with the analytic expected volume
`12 * 12 * 1.5 - 16 * pi * 0.75^2 * 1.5` (relative tolerance `1e-6`). Tight
bounds must match `(0, 0, 0)` to `(12, 12, 1.5)` within `1e-4` length units,
allowing modeling-tolerance padding; ordinary bounds must enclose that plate
within the same tolerance. All bounds must be finite and ordered.
Additional queries on each completed solid must
leave its volume and face/edge counts unchanged. Counts are compared only
before/after querying the same object, not against a fixed partition.

Ordinary and tight bounds need not coincide, especially for curved or trimmed
models. Matching these observables is not a general proof that two Breps are
identical.

Reported timings separate Boolean calls from per-cut bounds queries and
exclude primitive creation, correctness checks, output, and cleanup. They are
a small diagnostic experiment, not an unbiased benchmark or a guaranteed
speedup: allocation, caches, run order, geometry, and hardware affect results.
No-query mode still runs untimed final qualification queries.

The owning regression suite compiles and runs the same source via
[`TestSmBooleanBoundsExample`](../../tests/SM_API_test/src/TestSmBooleanBoundsExample.cpp).
It checks correctness, never a timing ratio. To run the suite on Linux:

```bash
./repo.sh build -r --target SM_API_test_app
./tests/SM_API_test/SM_API_test.sh linux-x86_64
```
