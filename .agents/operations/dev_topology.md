<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Developer Topology and Draw (`_smlib_dev`)

**Module:** `import _smlib_dev as smdev` (import `_omni_solid` first)
**Source:** `source/SmPyDevLib/src/SmPyDevMain.cpp`, `SmPyDevTopology.cpp`, `SmPyDevAssertValid.cpp`, `SmPyDevDump.cpp`, `SmPyViewportDraw.cpp`, `SmPyDevTests.cpp`, `SmPyDevUserTests.cpp`
**C API / kernel:** `SmTopologySolver.h`, `SmDraw`, `SmApiCreateBlendPrimitive`, `_smlib_tests`

## Overview

Debugger-only Python surface for oriented trim topology (`Loop`, `Loopuse`, `Edgeuse`), exact viewport topology picking, kernel `AssertValid` reports, kernel `Dump()` text, `Draw()` batch extraction, blend primitives between edgeuses, and in-process `prog_test` execution. These symbols were removed from stable `_omni_solid` so modeling scripts and asset pipelines stay lean.

```python
import _omni_solid as sm
import _smlib_dev as smdev

box = sm.create_box((0, 0, 0), 10, 10, 10)
edge = box.edges()[0]

# Free-function topology walks (preferred on stable handles).
edgeuses = smdev.edgeuses(edge)
loops = smdev.loops(edge)
loopuses = smdev.loopuses(edge)
eu = smdev.edgeuse_of_face(edge, box.faces()[0])

# Ray pick (viewport debugger).
hits = smdev.topology_pick_ray(box, origin, direction, tolerance)

# Kernel Draw extraction for overlays.
batches = smdev.draw.extract_draw_batches(edgeuses[0])

# Kernel AssertValid on the Brep or selected topology (runs in debug and release).
ok = smdev.assert_valid(box)              # or box.faces()[0], an Edgeuse, ...
assert ok
ok, reports = smdev.assert_valid_reports(box)  # same checks; caller formats failures

# Kernel Dump() echoes to stderr in debug SMLib builds (launch the GUI without --release).
smdev.dump(box)
smdev.dump(box.faces()[0])

# In-process tests (GUI test runner).
smdev.tests.run_suite("booleans")

# Kernel-direct C++ (edit SmPyDevUserTests.cpp, rebuild; GUI test_3 / test_4).
box = smdev.user_test_example()
result = smdev.user_test()
```

## Topology Pick

| Function | Purpose |
|---|---|
| `topology_pick_ray(brep, ray_point, ray_direction, tolerance, line_padding=0.5)` | Exact topology picker for viewport rays. Clips the ray to the Brep bounding box, builds a finite `SmLine`, and calls `SmTopologySolver::BrepCurveSolve` with `SM_SO_INTERSECT` / `SM_SR_ALL`. Returns raw `Vertex`, `Edge`, and `Face` hit records for caller-side priority and cycling. |

Used by `tools/smlib_gui/operations/picking.py` for right-click viewport selection.

## AssertValid

| Function | Purpose |
|---|---|
| `assert_valid(object, level=2, walk=True)` | Call `SmObject::AssertValid` on a `Brep`, `Face`, `Edge`, `Vertex`, `Loop`, `Loopuse`, `Edgeuse`, `Curve`, or `Surface`. Returns `True` when checks passed, and writes failed reports to `sys.stderr`. Checks run in **both** debug and release builds. Default `level=2` / `walk=True` is expensive on a large Brep. |
| `assert_valid_reports(object, level=2, walk=True)` | Same checks, returning `(ok, reports)` where `reports` is a list of formatted report lines (empty when `ok`). Writes nothing to stderr, so the caller owns presentation. |

Both deliberately avoid `sm_AssertValid`, which returns `TRUE` without testing anything outside `SM_DEBUG_CODE`, and format the `SmAssertArray` in the binding rather than calling `SmAssertArray::Dump`, whose `smos_WriteBuffer` output is debug-only. Report text is therefore identical in both configurations, except for the originating file/line labels, which `SmAssertReport` only stores in debug SMLib.

Used by `tools/smlib_gui` **AssertValid** on the selected active object and on the current topology pick; the inspector shows the formatted reports.

## Dump

| Function | Purpose |
|---|---|
| `dump(object, abbreviated=False)` | Call kernel `Dump()` on a `Brep`, `Face`, `Edge`, `Vertex`, `Loop`, `Loopuse`, `Edgeuse`, `Curve`, or `Surface`. Returns `None`. Debug SMLib echoes dump text to stderr (the launching terminal); release builds intentionally do not echo Dump(). `abbreviated=True` requests the shorter dump on types that support it; ignored for `Brep` and `Edgeuse`. |

Used by `tools/smlib_gui` **Dump** on the selected active object and on the current topology pick. The inspector records the dump target; it does not reprint kernel text. The GUI launcher prints once at startup whether this SMLib echoes `Dump()`.

## Free-Function Topology Walks

These take stable `Edge` / `Face` handles from `_omni_solid` and return dev types or lists:

| Function | Returns | Purpose |
|---|---|---|
| `edgeuses(edge)` | `list[Edgeuse]` | All oriented edgeuses referencing the edge, in radial-list order. |
| `loopuses(edge)` | `list[Loopuse]` | Unique loopuses containing the edge. |
| `loops(edge)` | `list[Loop]` | Unique loops containing the edge. |
| `edgeuse_of_face(edge, face)` | `Edgeuse \| None` | Oriented edgeuse on `face`; `None` if not incident. |
| `loop_of_face(edge, face)` | `Loop \| None` | Loop on `face` containing the edge; `None` if not incident. |
| `face_loops(face)` | `list[Loop]` | Canonical face traversal; all trim loops, with the outer loop first when present. |
| `poly_face_original_face_index(poly_face)` | `int` | Source BRep face index retained by a tessellation polygon; `-1` when unavailable. |

Stable `_omni_solid` still exposes `face.edges()`, `face.vertices()`, `edge.faces()`, etc. Only oriented loopuse/edgeuse traversal lives here.

## Dev Types: Loop, Loopuse, Edgeuse

| Type | Key methods |
|---|---|
| `Loop` | `.edgeuses()`, `.edges()`, `.vertices()`, `.loopuse()`, `.loopuses()`, `.face()`, `.brep()`, `.is_outer_loop()` |
| `Loopuse` | `.edgeuses()`, `.edges()`, `.loop()`, `.face()`, `.brep()`, `.other_loopuse()`, `.orientation()`, `.is_edge_loopuse()`, `.is_vertex_loopuse()` |
| `Edgeuse` | `.edge()`, `.face()`, `.loopuse()`, `.loop()`, `.mate()`, `.cw_edgeuse()`, `.ccw_edgeuse()`, `.edge_parameter(t)`, `.point_at_normalized(t)`, `.uv_point_at_normalized(t)`, `.orientation()`, `.brep()` |

## Draw Submodule (`smdev.draw`)

| Function | Purpose |
|---|---|
| `extract_draw_batches(topology_or_brep, ...)` | Run kernel `Draw()` on a `Brep`, `Face`, `Edge`, `Vertex`, `Loop`, `Loopuse`, `Edgeuse`, `Curve`, or `Surface` and return line/point batches for PyVista overlays. Requires `SM_GFX_OUTPUT_CODE`. A build that defines `SM_NO_GFX_OUTPUT_CODE` raises `RuntimeError` instead of returning an empty list. |
| `viewport_draw_*` | Lower-level helpers used by the GUI overlay pipeline. |

## Tests Facade (`smdev.tests`)

Thin Python facade over `_smlib_tests` for in-process `prog_test` execution from the GUI test runner. Prefer this over importing `_smlib_tests` directly in workbench code.

## Kernel-Direct User Tests

C++ scratch pad for calling SMLib without wrapping each kernel entry in `_smlib_dev`. Edit `source/SmPyDevLib/src/SmPyDevUserTests.cpp` and rebuild; Python Reload is not enough.

| Function | Purpose |
|---|---|
| `kernel_create_box(origin, length, width, height)` | Kernel-direct `SmPrimitiveCreation::CreateBox` without SM_API validation. Degenerate dimensions (e.g. zero height) build an invalid Brep, for exercising `assert_valid`. |
| `user_test_example()` | Kernel-direct analog of GUI `user_test.py` test_2. GUI **test_3** calls `result = smdev.user_test_example()`. Returns the box Brep. |
| `user_test()` | Ad-hoc stub. GUI **test_4** calls `result = smdev.user_test()`. Return a Brep for tessellation, or `None`. |

## GUI Integration

- `tools/smlib_gui/runtime.py` exports `smdev` alongside `sm`.
- Picking: `tools/smlib_gui/operations/picking.py` → `smdev.topology_pick_ray`, `smdev.edgeuses`, etc.
- Overlays: `tools/smlib_gui/operations/overlays.py` → `smdev.draw.extract_draw_batches`. When the kernel is built without `SM_GFX_OUTPUT_CODE`, the workbench reports that debug Draw overlays are unavailable and uses sampled highlights. Tessellated model display is unchanged.
- AssertValid: `tools/smlib_gui/operations/assert_valid.py` → `smdev.assert_valid`.
- Dump: `tools/smlib_gui/operations/dump.py` → `smdev.dump`.
- Test runner: `tools/smlib_gui/operations/test_runner.py` → `smdev.tests.*`.
- Kernel-direct user tests: `tools/smlib_gui/user_test.py` test_3 → `smdev.user_test_example()`; test_4 → `smdev.user_test()`.

## See Also

- **[queries](queries.md)** — stable BRep/Face/Edge/Vertex queries on `_omni_solid`
- **`smlib-python-dev-api-maintain`** skill — binding/stub/test maintenance for this module
- **`usd-brep-tools-python-gui`** skill — workbench port and smoke validation
