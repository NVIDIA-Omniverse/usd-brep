<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Error Handling

**Source:** `source/SMLib/inc/SmMessages.h`, `source/SmPyLib/src/SmPyError.{h,cpp}`
**Python:** kernel-status failures routed through `CHECK_STATUS` raise `RuntimeError`.

## Overview

Most binding operations that consume an `SmStatus` use `CHECK_STATUS`, which raises
`RuntimeError`. Exceptions and non-status validation paths exist. Normal `SmPyRaise` messages have
one of two shapes:

```text
<wrapped-entry> failed: <SM_ERR_NAME> (<code>). Kernel trace: <TRACE_ERR_NAME> [at <file>:<line>] [(<message>)]; ...

<wrapped-entry> failed: <SM_ERR_NAME> (<code>) at <binding-file>:<binding-line>
```

- `<wrapped-entry>` is the leading token extracted from the stringified `CHECK_STATUS` expression.
  It is commonly an `SmApi*` entry point such as `SmApiCreatePipeSweep`, but can be a binding helper
  or a direct kernel method such as `self.EvaluatePoint`.
- `<SM_ERR_NAME>` is the symbolic identifier (e.g. `SM_ERR_INVALID_INPUT`); see the table below.
- `<code>` is the raw integer for backwards compatibility with old logs.
- When present, the `Kernel trace` segment lists captured non-success kernel callback events in
  emission order, separated by semicolons, with a source basename and callback-reported line when
  supplied. In a simple nested `SER()` unwind, the first entry is often the deepest site.
- With no captured trail, the second shape omits `Kernel trace:` and reports the binding call site.
- The trail has a soft 4096-byte truncation threshold per call, not a hard cap. The entry that
  crosses the threshold is appended whole and a trailing ` ...[truncated]` marker is added, so the
  message can run slightly past 4096 bytes; every subsequent entry is then dropped. A truncated
  trail means those later entries were dropped, not that the kernel stopped reporting.

The Python frame above the `RuntimeError` in the traceback already shows which binding the caller invoked (`pipe_sweep`, `boolean_union`, …); the C++ message is intentionally focused on *what went wrong inside SMLib*.

## SM_API status policy

Use the following convention for `SmApi*` wrapper status handling:

- `SM_SUCCESS` when the operation succeeded.
- `SM_ERR_INVALID_INPUT` when a wrapper's argument checks reject the call: a NULL required
  argument, a NaN or out-of-range value, a parameter outside a curve or surface domain, too few
  items, or mismatched counts. Each function defines its own accepted inputs; this is not a
  promise that every wrapper validates every possible bad input.
- The kernel's own status when a kernel step fails, so a specific code such as
  `SM_ERR_NOT_CONVERGING` reaches the caller instead of the generic `SM_ERR`.
- `SM_ERR` for failure without a more specific code, or for an empty result where the individual
  API treats that as failure (for example, closest-point queries and surface/surface intersection).
  Other intersection APIs can succeed with empty collections; this policy does not change those
  no-result contracts.

Some legacy wrappers still return `SM_ERR` or `SM_ERR_NULL_POINTER` for bad input.
The status alone does not guarantee rollback, unchanged inputs, or empty outputs. In particular,
`SM_ERR_INVALID_INPUT` can also be propagated from a kernel step after work has begun. Consult the
individual operation's ownership and failure contract before reusing inputs or consuming outputs.

## Reading a Failure

Example (the case that motivated this doc — pipe-sweeping a cable along a tight bend), wrapped here
for readability; the real message is a single line:

```text
RuntimeError: SmApiCreatePipeSweep failed: SM_ERR (1001). Kernel trace:
  SM_ERR at SmCurve.cpp:<line> (non-G1 path);
  SM_ERR at SmPrimitiveCreation.cpp:<line> (approx tol exceeded)
```

Order of investigation:

1. **Symbolic name.** `SM_ERR` is the catch-all "operation failed". Look up the row in the table below for typical causes for *this specific operation*.
2. **Wrapped entry.** `SmApiCreatePipeSweep` is the common operation-wrapper case: search `source/SM_API/inc/SmApiBrep.h` for parameter docs and `source/SM_API/src/SmApiBrep.cpp` for the implementation. For a helper or direct method, search its reported identifier in `source/SmPyLib/` or `source/SMLib/`.
3. **Kernel callback trail.** Inspect the earliest and most specific entry first. In a simple
   `SER()` propagation chain, `SmCurve.cpp` is often the originating constraint, but the formatter
   does not mechanically designate a root cause. Parenthesized text is the callback message.

## Status Code Table

Mirrors `source/SMLib/inc/SmMessages.h`. When fixing a failure, treat the `Typical cause` column as a starting hypothesis, not a guarantee.

| Code | Identifier | Meaning | Typical cause |
|---|---|---|---|
| 1000 | `SM_SUCCESS` | OK | not an error |
| 1001 | `SM_ERR` | Generic "operation failed", or nothing found | The kernel ran out of strategies. Look at the *kernel trace* — it points at the specific check that gave up. Common in sweeps with self-intersecting paths, booleans on degenerate input, fillets where radius exceeds local geometry. |
| 1002 | `SM_ERR_OUT_OF_MEMORY` | Allocation failure | Genuine OOM, or a runaway algorithm that started building an unbounded result (very large boolean trees, fine tessellation on a huge model). |
| 1003 | `SM_ERR_UNKNOWN` | Unclassified internal error | Treat like `SM_ERR`; the kernel didn't tag it. |
| 1004 | `SM_ERR_FATAL` | Unrecoverable kernel state | The context may be corrupt. Re-run from scratch; if reproducible, file a bug with the input. |
| 1005 | `SM_ERR_ASSERT_FAILURE` | Internal `SM_ASSERT` tripped | A kernel invariant was violated. Almost always indicates malformed input that bypassed the public-API validators. File a bug with the input. |
| 1006 | `SM_ERR_NULL_POINTER` | Required pointer was NULL | A kernel step detected a NULL pointer, or a legacy wrapper rejected a NULL argument. This is not general detection of stale handles. |
| 1007 | `SM_ERR_INVALID_INPUT` | Invalid argument | NULL required arguments, negative dimensions, a radius of zero, a sweep angle of zero degrees, an empty curve list, a zero-length plane normal or projection/sweep direction, etc. Check the binding docstring for per-arg constraints. |
| 1008 | `SM_ERR_NON_NULL_OUTPUT_POINTER` | Output pointer was already non-NULL | Internal API contract violation; not normally reachable from Python. |
| 1010 | `SM_ERR_ASSERTVALID_FAILURE` | `AssertValid()` on a topology object failed | Topology corruption — usually from feeding the kernel hand-built `BrepArray` data that didn't pass through `stitch_*`. Re-stitch and retry. |
| 1020 | `SM_ERR_OUTSIDE_OF_DOMAIN` | Parameter not in [u_min, u_max] / [v_min, v_max] | Curve / surface evaluation at a parameter the kernel doesn't own. Clamp to the domain or call `evaluate_continuity` first. |
| 1021 | `SM_ERR_NOT_WITHIN_TOLERANCE` | Geometry didn't converge to the requested tolerance | Often raised by tessellation, healing, intersection, and projection. Try a looser tolerance (`chord_height_tolerance`, `tol3d`); inspect the geometry and operation diagnostics before retrying.  |
| 1022 | `SM_ERR_NOT_CONVERGING` | Iterative solver diverged | Surface-surface intersection on tangent surfaces, fillet propagation across high-curvature transitions, drop-point on near-degenerate surfaces. Perturb input or relax tolerance. |
| 1023 | `SM_ERR_WARNING` | Non-fatal warning emitted | Filtered out of the trail; you only see this if a non-default callback re-raises it. |
| 1024 | `SM_ERR_MESSAGE` | Informational | Filtered out of the trail. |
| 1025 | `SM_ERR_AXIS_INSIDE_FACE` | Rotational sweep axis crosses the profile face | Move `base_point` or rotate `axis` so the profile lies entirely on one side. |
| 1026 | `SM_ERR_LICENSE_EXPIRED` | License check failed | Refresh the license; not normally hit in the open-source build. |
| 1027 | `SM_ERR_BAD_INTERSECTIONS` | Intersector produced inconsistent topology | Surfaces touch tangentially, or intersection curves are below tolerance. Inspect `intersect_surfaces` results or pre-trim; this binding has no tolerance parameter. |
| 1028 | `SM_ERR_BAD_COINCIDENT_VERTICES` | Two vertices that should match are too far apart, or two that shouldn't are too close | Run `heal_brep` to merge near-coincident vertices; common after import from low-precision CAD formats. |
| 1030 | `SM_ERR_BAD_SURFACE_POINT` | Drop-point / closest-point query produced an unusable surface position | Surface is degenerate at the result, or the query point is too far from the surface. |
| 1032 | `SM_ERR_BAD_TANGENT_DROP` | Tangent-direction projection failed | Path tangent flipped or went to zero (`G0` joint in a `G1`-required path). |
| 1034 | `SM_ERR_DEGENERATE_SURFACE` | Surface has a zero-area patch or singular pole | Sweep / boolean / fillet hit a singular pole (e.g. `create_sphere` poles). Try `create_sphere_no_pole` or rebuild the patch. |
| 1036 | `SM_ERR_BAD_FIND_DEGEN_PARAM` | Domain-parameter search at a degeneracy didn't converge | Same family as `SM_ERR_DEGENERATE_SURFACE`; usually means the operation is right at a pole. Reposition input. |
| 1038 | `SM_ERR_METHOD_FAILURE_QUITING` | Algorithm voluntarily aborted | The kernel decided continuing would produce garbage. Specific to the algorithm — see the trace. |
| 1040 | `SM_ERR_NOTYET_HEAL_FACE` | Face couldn't be healed automatically | Inspect or rebuild the offending face before retrying; `heal_brep` exposes no tolerance parameter. |
| 1041 | `SM_ERR_TESS_FAIL_FACE` | Tessellation failed on a specific face | Defined for compatibility, but no current production path emits this status. Diagnose the actual returned status and tessellation diagnostics instead. |

## Operation-Specific Guidance

### pipe_sweep and Cables

`pipe_sweep` raises `SM_ERR` (1001) more often than any of the typed codes because the kernel collapses several distinct path/profile failures into the generic bucket. Read the kernel trace; the most common causes are:

- **Radius ≥ path's minimum radius of curvature.** The pipe self-intersects on the inside of a bend. Reduce radius, or smooth the path with `create_curve_approx_points` (looser tolerance) so its curvature stays below `1/radius`.
- **Path is non-G1.** Polylines built with `create_line_segment` and joined have `G0` corners; sweeping a circle through them produces a discontinuous surface. Build the path with `create_curve(points)` (interpolating) or `create_curve_approx_points` to get a `G1` spline.
- **Path has zero or near-zero length.** Two consecutive control points coincident, or a one-point "curve". Validate the path before sweeping.
- **Path is closed and self-intersecting.** A figure-eight cable path produces an invalid pipe even at small radius.

A working cable example:

```python
import _omni_solid as sm

# Smooth path — interpolate, don't polyline.
path = sm.create_curve([(0, 0, 0), (10, 5, 0), (20, 0, 5), (30, 5, 5)])

# Radius small enough for the bends.  If unsure, halve until pipe_sweep succeeds.
cable = sm.pipe_sweep(radius=0.5, path=path, cap_ends=True)
```

### Booleans

`SM_ERR` from `SmApiBoolean*` is most often:
- Operands share a face but the shared face's edges don't match within tolerance — inspect and, when appropriate, heal both inputs. `boolean_with_options` exposes behavior flags, not a `tol3d` parameter.
- One operand was already consumed by a previous boolean (booleans take ownership of inputs). Build fresh copies via primitive constructors.
- Co-planar surfaces with opposite orientation produce ambiguous regions; pre-stitch with `stitch_into_solid`.

### Fillets

`SM_ERR` from `SmApiCircularFillet` / `SmApiFilletEdges`:
- `radius` exceeds the local geometry (the fillet would eat a neighboring face entirely). Reduce the radius and test a fresh BRep copy. `fillet_preview` takes a chosen radius and returns surfaces; it does not compute a maximum admissible radius.
- The selection contains stale or foreign edges, or unsupported local topology. Selected edges need not form one connected chain; isolate failing edge neighborhoods and corner interactions.

### Tessellation

`SM_ERR_TESS_FAIL_FACE` (1041) is defined but has no current production emitter. Start from the
actual returned status and any available trace or failed-face diagnostics; a face index is not
guaranteed. Isolate the failing geometry and compare explicit tolerances appropriate to its scale.
Inspect input validity before healing, then re-tessellate at the original settings. See
[the tessellation guide](../operations/tessellation.md).

<a name="usd-import--export"></a>

### USD Import and Export

Any `RuntimeError` from `usd.import_breps` / `usd.import_meshes` / `usd.export_brep`:
- Bad path: file doesn't exist, or path is to a directory.
- Stage parses but contains no `BrepArray` (for `import_breps`) or no `UsdGeomMesh` (for `import_meshes`). The bindings raise rather than returning an empty list — see the docstring.
- Schema mismatch on import: the `BrepArray` was written by an incompatible version. `usd.import_meshes` can read separately authored `UsdGeomMesh` prims, but does not convert an unreadable `BrepArray` to meshes.

## Catching Errors in Python

Phase-1 only enriches the *message*. There is no typed `SmlibError` subclass yet:

```python
import _omni_solid as sm

try:
    pipe = sm.pipe_sweep(radius=2.0, path=path)
except RuntimeError as exc:
    msg = str(exc)
    if "SM_ERR_INVALID_INPUT" in msg:
        ...      # caller-side validation problem
    elif "SmApiCreatePipeSweep" in msg:
        ...      # specifically a pipe_sweep failure
    raise
```

If you find yourself frequently dispatching on `SM_ERR_*` substrings, that's the signal to file an issue requesting a typed `SmlibError(RuntimeError)` subclass with `status_name` / `c_call` / `kernel_trace` attributes (phase 2 of the agent-error initiative).

## When the Trace Is Empty

A `RuntimeError` with no `Kernel trace:` segment uses the binding-site fallback shape:

```text
<wrapped-entry> failed: <SM_ERR_NAME> (<code>) at <binding-file>:<binding-line>
```

It means no kernel error-callback events were captured. This is expected when callback emission is
compiled out, as in non-`SM_DEBUG_CODE` definitions of `SER()`, and can also result from a direct
status return or thread-local callback behavior. Trail storage is thread-local; on threaded kernel
builds callback registration is also thread-local, while the current installer runs once, so calls
or kernel work on other threads may have no captured trail.

The numeric status and extracted expression token remain available. Known statuses use the symbolic
names in `kStatusTable`; unknown values use `SM_ERR_???`. The fallback location is in the binding,
not necessarily in `source/SM_API/`.

## Source Map

| File | Role |
|---|---|
| `source/SMLib/inc/SmMessages.h` | Defines every `SM_*` code. Add new codes here. |
| `source/SmPyLib/src/SmPyError.{h,cpp}` | Status-name table, thread-local trail, `SmPyRaise`, callback installer. |
| `source/SmPyLib/src/SmPyCommon.h` | `CHECK_STATUS` macro — single chokepoint that captures `#expr` and forwards to `SmPyRaise`. |
| `source/SmPyLib/src/SmPyMain.cpp` | Calls `install_sm_error_callback()` at module init. |
