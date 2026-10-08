<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD BRep — Solid Modeling Library

## What It Is

SMLib is a NURBS-based boundary representation (BRep) solid modeling kernel. It represents solids as collections of topological entities (shells, faces, edges, vertices) bound to exact geometric definitions (NURBS surfaces, B-spline curves, analytic primitives). This gives mathematically exact shape definitions that are resolution-independent, unlike polygon meshes.

The library is structured in layers:

- **SMLib** — the low-level kernel. Owns all geometry and topology classes (`SmBrep`, `SmFace`, `SmEdge`, `SmSurface`, `SmBSplineCurve`, etc.) and the algorithms that operate on them. This is a C++ library with an internal `SmContext` that manages memory.
- **SM_API** — a high-level C-style C++ API that wraps common SMLib operations into simple function calls. Functions follow the `SmApi*` naming convention. This is the primary API for users who want to create geometry, perform booleans, sweeps, fillets, tessellation, etc.
- **SM_API_USD** — a USD integration layer that provides import/export of BRep data to/from OpenUSD files (.usda/.usdc/.usd). BRep geometry is stored as a custom `BrepArray` schema alongside standard USD prims.
- **BREP_SM_USD / BREP_USD_DATA** — internal libraries for the actual BRep↔USD data conversion.
- **SmPyLib** (`_omni_solid`) — Python bindings via pybind11 that expose SM_API and SM_API_USD as a Python module, including tessellation to **`PolyBrep`** and polygon mesh booleans.

## Key Concepts

### BRep (Boundary Representation)
A solid is defined by its boundary surfaces. An `SmBrep` contains one or more `SmShell`s, each made of `SmFace`s (trimmed surfaces), connected along `SmEdge`s, meeting at `SmVertex` points. This topological structure tracks adjacency — you can ask which faces share an edge, which edges bound a face, etc.

### NURBS Geometry
The kernel supports analytic, NURBS, and other curve/surface subclasses. NURBS can represent a sphere exactly. Booleans and fillets operate on supported geometry types without requiring every input to be converted to NURBS; numerical solving and approximation use operation-specific tolerances.

### Tessellation
Converting exact BRep geometry to polygonal meshes for rendering. Controlled by chord-height tolerance (max distance from polygon to true surface), angle tolerance, max edge length, and max aspect ratio.

### Context
SM_API provides a process-global default `SmContext`; `SmApiGetOrCreateContext()` creates it lazily,
and the Python bindings initialize it automatically. Selected operations instead use an input
object's context or create a dedicated output context.

## Repository Layout

```
source/
  SMLib/          — core kernel (geometry, topology, algorithms)
  SM_API/         — high-level C API (SmApiPrimitives, SmApiBrep, SmApiCurves, etc.)
  SM_API_USD/     — USD import/export API
  BREP_SM_USD/    — internal BRep↔USD converter
  BREP_USD_DATA/  — USD BRep schema data classes
  SmPyLib/        — Python bindings (_omni_solid module)
  *_test/         — test projects
```

## SM_API Header Organization

| Header | Purpose |
|---|---|
| `SmApiTypes.h` | Basic types, export macros |
| `SmApiGeneral.h` | Context creation, draw, transform, scale, rotate, translate |
| `SmApiPrimitives.h` | Primitive creation: box, sphere, cone, cylinder, torus, pyramid, partial primitives, skins, blends |
| `SmApiBrep.h` | Boolean operations, sweeps, tessellation, cut, project, stitch, fillet, offset/shell |
| `SmApiCurves.h` | Curve creation: line, circle, arc, rectangle, polygon, B-spline from points, offset, evaluate, knot removal |
| `SmApiSurfaces.h` | Surface creation: from control points, ordered/random points, extrude, sweep, ruled, revolution, skin, offset, evaluate |
| `SmApiFillets.h` | Simple chamfer/circular/remove fillet on all edges |
| `SmApiPolygons.h` | Polygon (SmPolyBrep) boolean operations, manifold check, volume |
| `SmApiIntersectors.h` | Curve-curve, surface-surface, face-face, brep-brep, brep-plane, curve-surface/face/brep intersections |
| `SmApiTrimmedSurfaces.h` | Trim surface with 3D curves, project-and-trim, create planar faces from curves |
| `SmApiQueries.h` | Closest-point queries on curves, surfaces, breps; topology queries (get edges/faces, find edge/face by point) |
| `SmApiImportExport.h` | Native SMLib Brep and part-container file I/O |
| `SmApiHeal.h` | BRep healing |

## Python Bindings (SmPyLib / `_omni_solid`)

The Python module `_omni_solid` wraps SM_API with Pythonic conventions:

- Points and vectors are passed as 3-tuples: `(x, y, z)`
- Return types are opaque handles (`Brep`, `PolyBrep`, `Curve`, `Surface`, etc.)
- Kernel-status failures routed through `CHECK_STATUS` raise `RuntimeError` containing a wrapped
  entry and status, followed by either a kernel trace or a binding-site fallback — see
  [`errors.md`](errors.md) for both forms and per-code guidance
- USD functions live in the `_omni_solid.usd` submodule
- Tessellated meshes (`PolyBrep`) support topology queries, bounding box, mass properties, plane section, and ray tests — see `.agents/operations/polybrep.md`

### Quick Reference

```python
import _omni_solid as sm

# Primitives
box = sm.create_box((0,0,0), 10, 10, 10)
sph = sm.create_sphere((0,0,0), 5)
cyl = sm.create_cylinder((0,0,0), 5, 10)

# Booleans
result = sm.boolean_union(box.copy(), sph.copy())
result = sm.boolean_difference(box.copy(), sph.copy())
result = sm.boolean_intersection(box.copy(), sph.copy())
result = sm.boolean(box.copy(), sph.copy(), operation=sm.BooleanOp.UNION)

# Tessellation (exact BRep → mesh)
mesh = sm.tessellate(box, chord_height_tolerance=0.0, curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0)

# Topology queries
faces = box.faces()
edges = box.edges()
verts = box.vertices()

# USD round-trip
sm.usd.export_brep(box, "out.usda")
breps = sm.usd.import_breps("out.usda")
```

## Error Handling

- SM_API functions return `SmApiStatus` (== `long`). `SM_SUCCESS` indicates success.
- Python bindings check the status and raise `RuntimeError` on failure.
- Common error sources: null input pointers, degenerate geometry (zero-length edges, zero-area faces), topological inconsistencies.

## Common Patterns

### Mutation Conventions

Common docstring contracts for BRep operations are listed below. Consult the individual operation
for exceptions and ownership details:

- **`Pure: returns new <type>; inputs unchanged.`** — read-only producer. The result is a fresh handle; inputs survive untouched. Examples: `create_box`, `create_sphere`, `linear_sweep`, `offset_brep`, `shell_brep`, `tessellate`.
- **`Consumes: <args>; returns new Brep.`** — ownership of the named inputs transfers to the operation; use only the returned Brep afterward. The result can reuse the modified primary operand, so a fresh allocation is not guaranteed. Examples: `boolean_union`, `boolean_difference`, `merge_breps`.
- **`Mutates: <arg>; returns the same handle for chaining.`** — modifies the first argument in place *and* returns it, so calls compose: `box = sm.fillet_edges(sm.translate(box, (0, 0, 5)), edges, radius=0.5)`. Examples: `fillet_edges`, `cut`, `translate`, `rotate`, `heal_brep`. The mutator variant `Mutates: <arg>; returns (handle, ...diagnostics) for chaining.` returns a tuple with the handle first plus a few report values (e.g. `stitch_into_solid`, `unify_normals`); destructure as `brep, *diag = sm.stitch_into_solid(brep, ...)`.

Names alone do not determine mutation: non-manifold sweeps mutate their source, and
`create_blend_primitive` inserts into a supplied BRep. Check the individual operation contract.

When you must keep the original through a mutator, copy it first: `keep = brep.copy(); sm.cut(keep, ...)`. The `Brep.copy()` method (and the flat `sm.queries.brep_copy`) is the canonical entry point. The full spelling, with examples, lives in the Python Quick Start in `AGENTS.md`.

### Creating Solids from Profiles
1. Create 2D curves (circle, rectangle, spline from points)
2. Sweep them: `linear_sweep`, `rotational_sweep`, `draft_sweep`, `pipe_sweep`
3. Optionally boolean with other solids

### Modifying Solids
1. Cut with planes (`cut`)
2. Fillet edges (`circular_fillet`, `fillet_edges`, `variable_radius_fillet`)
3. Shell (`shell_brep`) or offset (`offset_brep`)
4. Boolean combine with other shapes

### Querying
1. `is_manifold_solid` — check if a BRep is a valid closed solid
2. `compute_volume` — get enclosed volume (requires manifold solid)
3. `brep_closest_point` — find nearest point on BRep surface
4. `intersect_breps` / `intersect_brep_with_plane` — get intersection curves

### USD Pipeline
1. Create BRep geometry using python API
2. Export to USD: `sm.usd.export_brep(brep, "file.usda")` (subject-first)
3. Append to existing USD files: `sm.usd.append_breps([brep1, brep2], "file.usda")` (subject-first)

### End-to-end 3D assets

[generate_asset](generate_asset.md) — canonical pipeline for producing a visualizable 3D asset: SMLib creation → SMLib tessellation → mesh-only USD. Follow when building or generating a complete 3D asset (robot, vehicle, part, etc.).
