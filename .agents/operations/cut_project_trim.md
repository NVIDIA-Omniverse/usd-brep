<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Cut, Project, and Trim Operations

**Source:** `source/SM_API/inc/SmApiBrep.h`, `source/SM_API/inc/SmApiTrimmedSurfaces.h`
**Python:** `_omni_solid.cut`, `project_brep_onto_plane`, `project_and_trim`, `create_silhouette_curves`, `project_curve`, `trim_surface_with_3d_curves`, `trim_project_parallel`

## Overview

These operations modify BRep geometry by cutting, projecting, or trimming — removing material along planes or curves, or creating 2D projections.

## Operations

### cut
Cut a BRep with a plane, keeping the material on the positive side of the plane normal.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to cut (modified in-place) |
| `plane_point` | tuple | Any point on the cutting plane |
| `plane_normal` | tuple | Normal of the cutting plane (material on this side is kept) |

**Design notes:**
- The cut keeps the half-space in the direction of `plane_normal`. Flip the normal to keep the other half.
- A new planar face is created at the cut surface.
- The BRep is modified in-place — the removed material is gone.
- The plane must actually intersect the BRep; if the plane doesn't cut through the body, the BRep is unchanged.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
# Cut at Z=5, keep the bottom half (normal points down)
sm.cut(box, plane_point=(0, 0, 5), plane_normal=(0, 0, -1))
# box is now the lower half, 10x10x5

# Cut at Z=5, keep the top half (normal points up)
box2 = sm.create_box((0, 0, 0), 10, 10, 10)
sm.cut(box2, plane_point=(0, 0, 5), plane_normal=(0, 0, 1))
```

### project_brep_onto_plane
Project the edges of a BRep onto a plane, producing 2D projection curves.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to project |
| `plane_point` | tuple | Point on the projection plane |
| `plane_normal` | tuple | Normal of the projection plane (projection direction) |

**Returns:** list of projected curves on the plane.

**Design notes:**
- Projects the BRep's visible edges onto the plane along the plane normal direction.
- Returns curve outlines — useful for creating 2D drawings, silhouettes, or shadow projections.
- This is a non-destructive query — the BRep is unchanged.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
curves = sm.project_brep_onto_plane(box, (0, 0, 0), (0, 0, 1))
# Returns the rectangular outline of the box projected onto Z=0
```

## All Cut/Trim/Project Operations (now in Python)

### project_and_trim
Project a curve onto a BRep and use it to trim (cut) the BRep, keeping the side specified by a reference point.

```python
sm.project_and_trim(brep, curve, (0, 0, 1), (5, 0, 0))
```

### create_silhouette_curves
Compute silhouette curves of a BRep as seen from a viewing plane.

```python
curves = sm.create_silhouette_curves(brep, plane_point=(0, 0, 0), plane_normal=(0, 0, 1))
```

### project_curve

Project a curve onto a Brep along a non-zero direction. Returns new curves;
the source curve and Brep are not consumed. A kernel failure or a projection
that produces no curves raises `RuntimeError`.

For native callers, `SmApiProjectCurve` appends caller-owned curves to its
output array only on success. On failure, the array's existing curves and
their ordering are unchanged.

### trim_surface_with_3d_curves
Trim a standalone surface with 3D curves to produce a BRep with trimmed faces. The surface and the curves are consumed by the call, even if it fails (on failure they may already be deleted); do not reuse them afterwards.

```python
brep = sm.trim_surface_with_3d_curves(surface, curves_per_loop, curves)
```

### trim_project_parallel
Project a curve onto the faces of a `Brep` or standalone `Surface` and trim, with options for keeping/deleting sides or splitting.

- **Success:** the input is consumed and the result is returned as a new `Brep`.
- **Failure, Brep input:** the call raises; the Brep stays with the caller but may be partially trimmed.
- **Failure, standalone Surface input:** the call raises. If the kernel trim itself fails, the surface stays attached to an internal Brep that is not freed.
- **Rejected inputs:** a standalone surface with a zero `projection_dir`, or a surface owned by a Brep face (transform or trim the owning Brep instead). The call raises before trimming and the input is untouched.

```python
trimmed = sm.trim_project_parallel(
    obj, curve,
    projection_dir=(0, 0, 1),
    ref_point=(5, 0, 0),
    trim_type=sm.TrimType.KEEP_POINT,
)
```

| Trim Type | Behavior |
|---|---|
| `TrimType.KEEP_POINT` | Keep the side containing the reference point |
| `TrimType.DELETE_POINT` | Delete the side containing the reference point |
| `TrimType.SPLIT` | Keep both sides as separate faces |

## Common Patterns

### Slice a Body at Multiple Heights
```python
box = sm.create_box((0, 0, 0), 10, 10, 20)
# Get cross-sections at Z=5, 10, 15
for z in [5, 10, 15]:
    curves = sm.intersect_brep_with_plane(box, (0, 0, z), (0, 0, 1))
    print(f"Z={z}: {len(curves)} cross-section curves")
```

### Create a Half-Model
```python
body = sm.create_sphere((0, 0, 0), 10)
sm.cut(body, (0, 0, 0), (1, 0, 0))  # keep X > 0 half
```
