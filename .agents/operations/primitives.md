<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Primitive Creation

**Source:** `source/SM_API/inc/SmApiPrimitives.h`
**Python:** `_omni_solid.create_box`, `create_sphere`, `create_cone`, `create_cylinder`, `create_torus`, `create_plane`, `create_planar_circle`, `create_pyramid`, `create_cylindrical_box`, `create_partial_sphere`, `create_sphere_no_pole`, `create_partial_cone`, `create_cone_elliptic_ends`, `create_partial_cylinder`, `create_partial_torus`, `create_swung_primitive`, `create_skin_primitive`, `create_skin_from_faces`

## Overview

Primitives are the starting building blocks for solid modeling. Each function creates an `SmBrep` with exact NURBS surface geometry and proper boundary topology — these are not polygon approximations. The resulting BReps are valid manifold solids (except `create_plane` and `create_planar_circle` which produce single-face sheet bodies).

All primitives are positioned relative to an origin point with axes aligned to the global coordinate system. For arbitrary placement, create at origin and then use `translate` / `rotate` / `scale`.

## Operations

### create_box
Creates an axis-aligned box (rectangular parallelepiped) with 6 planar faces, 12 edges, 8 vertices.

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Position of the minimum-coordinate corner |
| `length` | float | Extent along X axis |
| `width` | float | Extent along Y axis |
| `height` | float | Extent along Z axis |

**Design notes:**
- The origin is the corner at `(minX, minY, minZ)`, not the center.
- All dimensions must be positive.
- Good starting shape for boolean operations — boxes are topologically simple and booleans with them are robust.

```python
box = sm.create_box((0, 0, 0), 10, 20, 5)
```

### create_sphere
Creates a sphere with exact NURBS representation. The sphere is a closed manifold solid.

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Center of the sphere |
| `radius` | float | Radius (non-zero; a negative radius gives an inside-out sphere) |

**Design notes:**
- Standard NURBS sphere has poles (degenerate points at top and bottom). For a pole-free variant, use `create_sphere_no_pole`.
- Spheres have seam edges where the periodic NURBS surface wraps. These are topological artifacts, not defects.

```python
sph = sm.create_sphere((0, 0, 0), 5.0)
```

### create_cylinder
Creates a capped cylinder aligned along the Z axis.

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Center of the base circle |
| `radius` | float | Radius of the circular cross-section |
| `height` | float | Height along Z axis |

```python
cyl = sm.create_cylinder((0, 0, 0), 3.0, 10.0)
```

### create_cone
Creates a truncated cone (frustum). Set `radius_top = 0` for a pointed cone.

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Center of the base circle |
| `radius_base` | float | Radius at the base (Z = 0) |
| `radius_top` | float | Radius at the top (Z = height). Use 0 for a pointed cone. |
| `height` | float | Height along Z axis |

**Design notes:**
- With `radius_top > 0` you get a frustum (truncated cone), useful for tapered walls and funnels.
- `create_cone_elliptic_ends` creates cones where each end is tilted to produce elliptic cross-sections.

```python
cone = sm.create_cone((0, 0, 0), radius_base=5.0, radius_top=2.0, height=10.0)
pointed = sm.create_cone((0, 0, 0), radius_base=5.0, radius_top=0.0, height=10.0)
```

### create_torus
Creates a torus centered at origin with the major circle in the XY plane.

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Center of the torus |
| `radius_major` | float | Distance from center to the tube center |
| `radius_minor` | float | Radius of the tube cross-section |

**Design notes:**
- `radius_major` must be greater than `radius_minor` for a standard torus. When equal, you get a self-intersecting horn torus. When `radius_minor > radius_major`, you get a spindle torus.
- The torus lies in the XY plane with the tube circling around Z.

```python
torus = sm.create_torus((0, 0, 0), radius_major=10.0, radius_minor=3.0)
```

### create_plane
Creates a single rectangular planar face (sheet body, not a solid).

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Corner at (minU, minV) |
| `length` | float | Extent along X axis |
| `width` | float | Extent along Y axis |

**Design notes:**
- This is NOT a manifold solid — it has 1 face, 4 edges, 4 vertices.
- `is_manifold_solid()` returns False.
- Useful for construction geometry, section cutting, or as a target for projection.

```python
plane = sm.create_plane((0, 0, 0), 20.0, 20.0)
```

### create_planar_circle
Creates a circular planar face (sheet body).

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Center of the circular face |
| `radius` | float | Radius of the circle |

```python
disk = sm.create_planar_circle((0, 0, 0), 5.0)
```

### create_pyramid
Creates a pyramid with a square base and an apex point.

| Parameter | Type | Description |
|---|---|---|
| `origin` | Vec3 / tuple | Center of the base |
| `length` | float | Length and width of the square base |
| `height` | float | Height from base to apex |

```python
pyr = sm.create_pyramid((0, 0, 0), 10.0, 15.0)
```

## Additional Primitives (now in Python)

- **`create_partial_sphere(origin, radius, start_angle_deg, end_angle_deg)`** — sphere sector between start/end angles
- **`create_sphere_no_pole(center, radius)`** — sphere without degenerate polar points
- **`create_partial_cone(origin, radius_base, radius_top, height, start_angle_deg, end_angle_deg)`** — conical sector
- **`create_partial_cylinder(origin, radius, height, start_angle_deg, end_angle_deg)`** — cylindrical sector
- **`create_partial_torus(origin, radius_major, radius_minor, start_angle_deg, end_angle_deg)`** — toroidal sector
- **`create_cylindrical_box(origin, length, inside_radius, outside_radius, start_angle_deg, end_angle_deg)`** — wedge-shaped solid in cylindrical coordinates
- **`create_swung_primitive(xy_curves, xz_curves, scale=1.0)`** — surface of revolution from XY/XZ profiles
- **`create_blend_primitive(brep, edge1, face1, edge2, face2, same_dir_curves=True)`** — G2 blend surface between two boundary edges on existing faces (uses stable `Edge` / `Face` handles)
- **`create_skin_primitive(curves_per_profile, curves, degree=1, cap_ends=True)`** — pure loft from ordered
  cross-section profiles; input curves remain unchanged. With `cap_ends=True`, only the first and last profiles
  are capped; intermediate profiles guide the loft without creating transverse faces.
- **`create_skin_from_faces(faces, degree=1)`** — loft between boundary loops of existing faces

### create_blend_primitive

Creates a continuity-controlled blend patch between two boundary edges on existing faces and inserts the resulting face into `brep`.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | Solid that will own the new blend face |
| `edge1` | Edge | First boundary edge on `face1` |
| `face1` | Face | Target surface at the start of the blend |
| `edge2` | Edge | Second boundary edge on `face2` |
| `face2` | Face | Target surface at the end of the blend |
| `same_dir_curves` | bool | `True` (default): blend start→start of the edge curves; `False`: start→end |

Obtain edges and faces from stable topology walks (`brep.faces()`, `face.edges()`, `edge.faces()`). Distinct from edge-rounding `fillet_*` operations.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
face_a = box.faces()[0]
edge_a = face_a.edges()[0]
# ... select a second face/edge pair on the same or merged brep ...
# blend_face = sm.create_blend_primitive(box, edge_a, face_a, edge_b, face_b)
```

## Choosing the Right Primitive

| Goal | Primitive | Notes |
|---|---|---|
| Rectangular solid | `create_box` | Most robust for booleans |
| Round solid | `create_sphere` / `create_cylinder` | Use sphere for balls, cylinder for tubes/posts |
| Tapered tube | `create_cone` | Set both radii for frustum |
| Ring / donut | `create_torus` | Major/minor radius ratio controls shape |
| Pointed solid | `create_pyramid` or `create_cone(radius_top=0)` | Pyramid = square base, cone = circular base |
| Flat reference | `create_plane` / `create_planar_circle` | For section cuts, projections |
| Complex profile | Use curves + sweep | See sweeps.md |
