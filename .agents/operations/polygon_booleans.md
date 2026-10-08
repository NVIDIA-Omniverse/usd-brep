<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Polygon Boolean Operations

**Source:** `source/SM_API/inc/SmApiPolygons.h`
**Python:** `_omni_solid.poly_boolean`, `poly_boolean_union`, `poly_boolean_difference`, `poly_boolean_intersection`, `poly_boolean_merge`, `PolyBooleanOp` (enum)

## Overview

Polygon booleans operate on `SmPolyBrep` objects (Python **`PolyBrep`**) rather than exact NURBS BReps. They are useful when working with imported mesh data or after tessellation.

The operations mirror the BRep booleans but work on triangulated geometry. Results are approximate (limited by mesh resolution) but can handle arbitrarily complex input meshes.

Obtain meshes via **`tessellate`** (see **[tessellation](tessellation.md)**), **USD mesh import** (see **[usd_import_export](usd_import_export.md)**), or a previous polygon boolean. Inspect results with **[polybrep](polybrep.md)**.

## Operations

All polygon boolean operations follow the same pattern as BRep booleans:

| Function | Operation | Description |
|---|---|---|
| `SmApiPolyBooleanUnion` | Union | Combine two meshes |
| `SmApiPolyBooleanDifference` | Difference | Subtract mesh B from mesh A |
| `SmApiPolyBooleanIntersection` | Intersection | Keep only the overlapping region |
| `SmApiPolyBooleanMerge` | Merge | Combine topology without removing faces |
| `SmApiPolyBoolean` | Generic | Specify operation via enum |

### Parameters (all variants)

| Parameter | Type | Description |
|---|---|---|
| `pPolyBrep1` | SmPolyBrep* | Primary mesh (A) |
| `pPolyBrep2` | SmPolyBrep* | Tool mesh (B) |
| `rpResult` | SmPolyBrep*& | Output mesh |

For the generic `SmApiPolyBoolean`:
| `lOperation` | SmPolyBooleanOperationType | Operation type enum |

### Additional Queries

- **`SmApiPolyBrepIsManifoldSolid`** — check if a polygon BRep is manifold
- **`SmApiPolyBrepComputeVolume`** — compute enclosed volume of a polygon BRep
- **`SmApiPolyBrepComputeMassProperties`** — area, volume, mass, centroid, and moments / products of inertia of a closed polygon BRep; mesh counterpart of `SmApiBrepComputeMassProperties`

In Python, the same checks are available as **`poly_brep_is_manifold_via_api`** and **`poly_brep_volume_via_api`**, or via **`PolyBrep.is_manifold_solid`** and **`PolyBrep.mass_properties`** (see **[polybrep](polybrep.md)**).

## Python usage

```python
import _omni_solid as sm

a = sm.tessellate(sm.create_box((0, 0, 0), 10, 10, 10), chord_height_tolerance=0.0, curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0)
b = sm.tessellate(sm.create_box((5, 0, 0), 10, 10, 10), chord_height_tolerance=0.0, curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0)

u = sm.poly_boolean_union(a, b)
d = sm.poly_boolean_difference(a, b)
i = sm.poly_boolean_intersection(a, b)
m = sm.poly_boolean_merge(a, b)

# Generic form — `PolyBooleanOp` exposes the same five operations as the named helpers
r = sm.poly_boolean(a, b, sm.PolyBooleanOp.UNION)
r = sm.poly_boolean(a, b, sm.PolyBooleanOp.PARTIAL_MERGE)
```

**`PolyBooleanOp`** (Python): `UNION`, `INTERSECTION`, `DIFFERENCE`, `MERGE`, `PARTIAL_MERGE`. The C enum `SmPolyBooleanOperationType` is larger; only these five values are exposed on `_omni_solid.PolyBooleanOp`.

Each operation returns a new **`PolyBrep`** result (or raises on failure).

## When to Use Polygon vs BRep Booleans

| Scenario | Use |
|---|---|
| Creating geometry from primitives/sweeps | BRep booleans (exact) |
| Working with imported mesh data | Polygon booleans |
| Post-tessellation operations | Polygon booleans |
| Need exact surface definitions downstream | BRep booleans |
| Performance on very complex geometry | Polygon booleans (often faster) |

## Design Notes

- Polygon booleans work on discrete triangle meshes. The result quality depends on mesh density at the intersection curves.
- Input meshes should be manifold (closed) for best results.
- The intersection curves are approximated by following triangle edges, so finer meshes produce smoother intersections.
- For highest quality, work in the BRep domain (exact NURBS) and only tessellate at the end.
