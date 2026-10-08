<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Surface Creation and Operations

**Source:** `source/SM_API/inc/SmApiSurfaces.h`, `source/SM_API/inc/SmApiTrimmedSurfaces.h`
**Python:** `_omni_solid.create_surface_from_points`, `create_surface_from_corner_points`, `create_surface_from_ordered_points`, `create_surface_from_random_points`, `create_ruled_surface`, `create_surface_revolution`, `create_offset_surface`, `create_extrude_surface`, `create_sweep_surface`, `create_skin_surface`, `create_planar_faces`

## Overview

Surfaces are standalone NURBS surface objects, distinct from faces within a BRep. They are used as building blocks — you can create surfaces from control points, curves, or other surfaces, then combine them into BReps via trimming and stitching operations.

Surfaces have a (u, v) parameter space. Points on the surface are evaluated by providing (u, v) coordinates. Each surface has well-defined partial derivatives, normals, and curvature.

## Operations

### create_surface_from_points
Create a B-spline surface from a grid of control points.

| Parameter | Type | Description |
|---|---|---|
| `control_points` | list[tuple] | Control points in row-major order: `P[row][col] = pts[row * cols + col]` |
| `rows` | int | Number of rows in the control point grid |
| `cols` | int | Number of columns in the control point grid |

**Design notes:**
- The control points form a grid that defines the shape of the surface. The surface approximates (but does not necessarily interpolate) the control points.
- Minimum grid: 2×2 (bilinear patch). Typical: 4×4 or larger for smooth surfaces.
- Higher grid resolution = more control points = more flexibility in shape but more data to manage.
- Points are in row-major order: the first `cols` points are row 0, next `cols` are row 1, etc.

```python
pts = [
    (0, 0, 0),  (5, 0, 0),  (10, 0, 0),  (15, 0, 0),
    (0, 5, 1),  (5, 5, 3),  (10, 5, 3),  (15, 5, 1),
    (0, 10, 1), (5, 10, 3), (10, 10, 3), (15, 10, 1),
    (0, 15, 0), (5, 15, 0), (10, 15, 0), (15, 15, 0),
]
srf = sm.create_surface_from_points(pts, rows=4, cols=4)
```

### create_ruled_surface
Create a surface that linearly interpolates between two curves.

| Parameter | Type | Description |
|---|---|---|
| `curve1` | BSplineCurve | First boundary curve |
| `curve2` | BSplineCurve | Second boundary curve |

**Design notes:**
- The surface is formed by connecting corresponding points on the two curves with straight lines.
- The two curves should have compatible parameterizations for best results. Use the C API `SmApiMakeCurvesCompatible` if needed.
- Useful for creating connecting surfaces between two profiles.

```python
arc1 = sm.create_arc((0, 0, 0), 5.0, 0, 180)
arc2 = sm.create_arc((0, 0, 10), 8.0, 0, 180)
srf = sm.create_ruled_surface(arc1, arc2)
```

### create_surface_revolution
Create a surface of revolution by rotating a curve around an axis.

| Parameter | Type | Description |
|---|---|---|
| `curve` | BSplineCurve | Generatrix curve to revolve |
| `origin` | tuple | Point on the rotation axis |
| `axis` | tuple | Direction of the rotation axis |
| `angle_deg` | float | Revolution angle in degrees (360 = full revolution) |

**Design notes:**
- The curve is rotated around the specified axis. Each point on the curve traces a circle.
- For a full surface of revolution, use `angle_deg=360`.
- The curve should not cross the rotation axis (produces self-intersection).
- This produces a standalone surface, not a solid. For a solid of revolution, use `rotational_sweep` instead.

```python
arc = sm.create_arc((5, 0, 0), 2.0, 0, 180)
srf = sm.create_surface_revolution(arc, origin=(0, 0, 0), axis=(0, 0, 1), angle_deg=360)
```

### create_offset_surface
Create a surface offset from an existing surface by a distance.

| Parameter | Type | Description |
|---|---|---|
| `surface` | Surface | Source surface |
| `distance` | float | Offset distance (positive = along normal, negative = opposite) |

**Design notes:**
- The offset surface is displaced along the surface normal at every point.
- Positive distance offsets in the normal direction; negative offsets in the opposite direction.
- Self-intersections can occur when the offset distance exceeds the minimum radius of curvature.
- An offset that collapses the surface (a sphere's radius, a torus's minor radius, or both cone radii shrinking to zero) produces no surface and raises `RuntimeError`.

```python
srf = sm.create_surface_from_points(pts, 4, 4)
offset_srf = sm.create_offset_surface(srf, distance=1.0)
```

### create_planar_faces
Build a `Brep` made of planar faces from one or more closed planar curve loops.

| Parameter | Type | Description |
|---|---|---|
| `curves` | list[Curve] | Curves forming one or more closed planar loops. All curves must be coplanar. |

**Returns:** `Brep` — a sheet-body BRep whose faces are the planar regions bounded by the input loops.

**Design notes:**
- The plane of the result is inferred from the input curves; do not pass a plane normal.
- The input curves are consumed on success (the new faces use them); do not reuse them afterwards. On failure, the curves remain owned by the caller.
- Multiple closed loops on the same plane produce one face per loop (or with-holes faces if loops nest), all packed into a single `Brep`.
- Returns a sheet body (open shell), not a manifold solid. Use `linear_sweep` or stitching to extrude/close into a solid.
- Wraps `SmApiCreatePlanarFaces` from `SmApiTrimmedSurfaces.h`. Lives with the surface creators rather than the trimming ops because it is a creation primitive — it constructs a Brep from scratch given curve loops.

```python
loop = [
    sm.create_line_segment((0, 0, 0), (10, 0, 0)),
    sm.create_line_segment((10, 0, 0), (10, 10, 0)),
    sm.create_line_segment((10, 10, 0), (0, 10, 0)),
    sm.create_line_segment((0, 10, 0), (0, 0, 0)),
]
sheet = sm.create_planar_faces(loop)
```

## Surface vs. Face vs. BRep

| Entity | What it is | Use for |
|---|---|---|
| `Surface` | Raw NURBS surface (untrimmed, infinite in parameter space) | Geometric building block |
| `Face` | Trimmed surface within a BRep (bounded by edge loops) | Part of a solid boundary |
| `Brep` | Collection of faces forming a solid boundary | Solid modeling operations |

Surfaces are turned into faces/BReps via trimming operations (`SmApiTrimSurfaceWith3dCurves`) or by being part of a sweep/boolean result.

- `create_surface_from_corner_points` — bilinear surface from 4 corner points
- `create_surface_from_ordered_points` — surface fit to ordered point grid
- `create_surface_from_random_points` — surface fit to unordered point cloud
- `create_extrude_surface` — extrusion surface from curve and vector
- `create_sweep_surface` — general sweep surface from generation curve along path curve
- `create_skin_surface` — lofted surface through multiple profile curves
