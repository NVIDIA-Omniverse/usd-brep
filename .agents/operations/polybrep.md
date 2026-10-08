<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Polygon BRep (`PolyBrep`)

**Source:** `source/SMLib/inc/SmPoly.h` (core), `source/SmPyLib/src/SmPyPolyBrep.cpp` (bindings)
**Python:** `_omni_solid.PolyBrep`, `PolyFace`, `PolyEdge`, `PolyVertex`; module helpers `poly_boolean`, `poly_boolean_union`, `poly_boolean_difference`, `poly_boolean_intersection`, `poly_boolean_merge`, `poly_brep_is_manifold_via_api`, `poly_brep_volume_via_api`

## Overview

A **`PolyBrep`** is SMLib’s polygon mesh representation: triangulated (or polygonal) topology with 3D points. You obtain one by **tessellating** an exact `Brep`, **importing** USD meshes (`usd.import_mesh` / `usd.import_meshes`), or as the **result of polygon booleans**. It is the right type for rendering-oriented data, mesh analysis, and approximate set operations on discrete geometry.

This guide documents the **Python** surface for inspecting and measuring a `PolyBrep`. C API equivalents for mesh booleans and volume checks live in `SmApiPolygons.h`; tessellation entry points are in `SmApiBrep.h` (`SmApiTessellate`). File I/O for meshes is covered in **[usd_import_export](usd_import_export.md)**.

## Types

| Python type | Role |
|---|---|
| `PolyBrep` | Mesh body: faces, edges, vertices, and point positions |
| `PolyFace` | One polygonal face; `get_normal()` returns a shading normal |
| `PolyEdge` | Directed edge between two `PolyVertex` instances |
| `PolyVertex` | Mesh vertex position via `get_point()` |

## Construction (see also)

| Workflow | Entry point |
|---|---|
| BRep → mesh | `sm.tessellate(brep, ...)` — see **[tessellation](tessellation.md)** |
| Mesh ↔ USD | `sm.usd.import_mesh`, `sm.usd.export_mesh`, … — see **[usd_import_export](usd_import_export.md)** |
| Mesh ∪ / ∩ / − | `sm.poly_boolean*` — see **[polygon_booleans](polygon_booleans.md)** |

## Topology and points

### get_faces / get_vertices / get_edges
Return lists of `PolyFace`, `PolyVertex`, and `PolyEdge` handles. Order is kernel-defined; use for traversal, not stable indexing across edits.

### get_points
Returns vertex positions as a list of `(x, y, z)` tuples in mesh order (consistent with topology).

### to_mesh_arrays / to_numpy_arrays

Points and polygon indices preserve the original mesh connectivity. Normals are
averaged per vertex by default. Pass `face_varying_normals=True` to either method
for one source-face normal per polygon corner, in polygon order without the
counts in `faces`. Use USD `faceVarying` interpolation with this output.
Missing or degenerate normals fall back to the vertex average, then `(0, 0, 1)`.

`to_mesh_arrays()` returns Python lists for compatibility. `to_numpy_arrays()`
returns the same `points`, `normals`, `faces`, and `face_normals` schema in
binding-owned contiguous NumPy buffers: coordinates and normals are `float32`,
while the flat PyVista polygon stream is `int32`. Prefer the NumPy form for
rendering and bulk processing because it avoids per-value Python objects and a
subsequent list-to-array copy.

### PolyVertex.get_point
Coordinates of a single vertex as `(x, y, z)`.

### PolyFace.get_normal
Evaluated face normal `(nx, ny, nz)`.

### PolyEdge.get_start_vertex / get_end_vertex
Incident vertices along the edge direction.

## Bounding box and tolerance

### calculate_bounding_box
Returns `(min_xyz, max_xyz)` as nested 3-tuples. The box is expanded by the mesh tolerance.

### get_tolerance
Mesh tolerance value used internally (e.g. for bbox and classification).

### copy
Deep-copy the mesh into a fresh `PolyBrep` with independent topology (wraps `SmApiPolyBrepCopy`, mirroring `Brep.copy`). Mutating or consuming the result — e.g. a `poly_boolean*`, which consumes its operands — does not affect the source, so `copy()` first when you need to keep the original.

## Mass properties and area

### is_manifold_solid
`True` if the mesh is classified as a closed manifold solid (kernel predicate).

### compute_area_centroid
Returns `(area, (cx, cy, cz))` for the mesh surface.

### mass_properties(density=1.0, origin=None)
Mesh counterpart of `Brep.mass_properties` (wraps `SmApiPolyBrepComputeMassProperties`), with the same keys and conventions: `area`, `volume`, `mass`, `centroid`, `moments_of_inertia` `(Ixx, Iyy, Izz)` and `products_of_inertia` `(Iyz, Izx, Ixy)` as raw positive integrals. `origin=None` gives centroidal inertia (two passes); an explicit `origin` is a single pass about that point. Results are exact for the mesh, so there is no `relative_accuracy`; accuracy is that of the tessellation. The result does not depend on winding direction: a closed mesh wound inside-out (for example, as imported from USD) gives the same result, whereas `poly_brep_volume_via_api` returns a negative signed volume for it. Raises `RuntimeError` (`SM_ERR_INVALID_INPUT`) for a mesh that is not a closed manifold solid, an invalid density, or a non-finite origin.

When a mesh already exists (for example, one built for rendering), this is a cheap preview of the exact `Brep.mass_properties`, most of all on models with many trimmed faces. On simple curved primitives the exact integration is already cheap, so the mesh path gains little.

### compute_properties(origin)
Raw kernel output about the given `origin` (3-tuple): a dict with `area`, `volume`, `barycenter`, `moments_of_inertia`, `products_of_inertia`. Despite the key name, `moments_of_inertia` holds the per-axis **second moments** (`∫x² dV, ∫y² dV, ∫z² dV`), and `products_of_inertia` is `(∫xy, ∫yz, ∫zx) dV`; neither is density-weighted. Prefer `mass_properties` for true moments of inertia.

## Sectioning and rays

### section_plane(plane_origin, plane_normal, zone_tolerance)
Intersects the mesh with a plane. Returns a list of segment endpoints: `((x0,y0,z0), (x1,y1,z1)), ...`.

### ray_intersection(ray_point, ray_direction, ray_tolerance, ray_stepoff)
Ray and point-in-mesh style queries. `ray_direction` may be `None` for point-in-mesh style tests. Each hit is a dict with keys such as `t`, `point`, `normal`, and optionally `face` (a `PolyFace`).

## SM_API wrappers (`poly_brep_*_via_api`)

`poly_brep_is_manifold_via_api` and `poly_brep_volume_via_api` call `SmApiPolyBrepIsManifoldSolid` and `SmApiPolyBrepComputeVolume`. Prefer `PolyBrep.is_manifold_solid` and `compute_properties` / workflow-specific APIs when they meet your needs; the `*_via_api` names exist for parity with the C API.

## Example

```python
import _omni_solid as sm

solid = sm.create_box((0, 0, 0), 10, 10, 10)
mesh = sm.tessellate(solid, chord_height_tolerance=0.0, curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0)

assert mesh.is_manifold_solid()
mn, mx = mesh.calculate_bounding_box()
pts = mesh.get_points()
faces = mesh.get_faces()
```
