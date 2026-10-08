<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Transform Operations

**Source:** `source/SM_API/inc/SmApiGeneral.h`
**Python:** `_omni_solid.translate`, `rotate`, `scale`, `scale_about_point`, `transform`

## Overview

Python transform operations reposition and resize modeling objects in 3D space. They modify the target in-place and return the same handle.

Every entry point accepts the four independent modeling `Object` families: `Brep`, `PolyBrep`, standalone `Curve`, and standalone `Surface`. Unsupported internal `SmObject` types return `SM_ERR_INVALID_INPUT` from SM_API and raise `RuntimeError` in Python. The previous `object_translate` / `object_rotate` / `object_scale` wrappers have been removed and `object_scale_about_point` / `transform_object` have been renamed to `scale_about_point` / `transform` for symmetry.

Geometry returned from `edge.curve()` or `face.surface()` is owned by its parent Brep and cannot be transformed independently without invalidating topology-to-geometry relationships. Transform the owning Brep instead. Direct transforms of owned Curves and Surfaces are rejected without modifying either object.

Those Curve and Surface handles are borrowed views. If scaling a Brep would replace analytic geometry with NURBS, the Python binding rejects the scale before mutation while any affected borrowed view remains live. Release all such handles, perform the scale, and reacquire them from the mutated Brep. To keep handles into the original model, copy the Brep and scale the copy instead. Rigid transforms, positive uniform scale, and transforms of already-base-NURBS geometry preserve the underlying geometry pointers and remain valid with borrowed views live.

## Operations

### translate
Move any modeling object by a displacement vector.

| Parameter | Type | Description |
|---|---|---|
| `target` | Object | Independent `Brep`, `PolyBrep`, `Curve`, or `Surface` (modified in-place) |
| `translation` | tuple | Displacement vector (dx, dy, dz) |

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sm.translate(box, (5, 0, 0))  # move 5 units along X
```

### rotate
Rotate any modeling object around an axis.

| Parameter | Type | Description |
|---|---|---|
| `target` | Object | Independent `Brep`, `PolyBrep`, `Curve`, or `Surface` (modified in-place) |
| `ref_point` | tuple | Point on the rotation axis |
| `axis` | tuple | Rotation axis direction |
| `angle_deg` | float | Rotation angle in degrees (positive = counter-clockwise when looking along axis) |

**Design notes:**
- Rotation follows the right-hand rule.
- `ref_point` is a point the axis passes through. The object rotates around the line defined by `ref_point` + t * `axis`.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
# Rotate 45 degrees around Z axis through the box center
sm.rotate(box, ref_point=(5, 5, 5), axis=(0, 0, 1), angle_deg=45)
```

### scale
Scale any modeling object by per-axis factors about the world origin.

| Parameter | Type | Description |
|---|---|---|
| `target` | Object | Independent `Brep`, `PolyBrep`, `Curve`, or `Surface` (modified in-place) |
| `scale` | tuple | Scale factors (sx, sy, sz) |

**Design notes:**
- Scale factors must be finite and have magnitude greater than the kernel's effective-zero tolerance.
- Non-uniform scaling is supported for Breps, PolyBreps, generic B-spline Curves and Surfaces, and lines. Analytic geometry inside a `Brep` is converted to NURBS when needed.
- This API permits only positive, uniform scale factors for other specialized standalone Curve and Surface representations. Unsupported scaling returns an error before modifying the object.
- PolyBrep scale factors must be positive and greater than the kernel's effective-zero tolerance. Degenerate and mirrored PolyBreps need separate topology-orientation handling, so zero, near-zero, and negative factors are rejected without modification.
- Scaling is relative to the origin. To scale relative to a different center, translate the center to origin first, scale, then translate back.
- An odd number of negative scale factors reverses handedness and mirrors Breps and supported
  standalone exact geometry; an even number preserves handedness
  (`(-1, -1, 1)` is a half turn about Z, not a mirror). PolyBreps reject negative factors entirely.
- Brep reflections preserve material orientation. Volume scales by `abs(sx * sy * sz)`.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sm.scale(box, (2, 2, 2))  # double the size
vol = sm.compute_volume(box)  # 8000.0 (8x original)

# Non-uniform scale: stretch along Z
sm.scale(box, (1, 1, 3))
```

## Common Patterns

### Position a Primitive
Primitives are created at their natural origin. Move them to the desired location:

```python
# Create a cylinder and move it to position
cyl = sm.create_cylinder((0, 0, 0), 2, 10)
sm.translate(cyl, (20, 15, 0))
sm.rotate(cyl, ref_point=(20, 15, 5), axis=(1, 0, 0), angle_deg=90)
```

### Scale Around a Center Point
```python
box = sm.create_box((5, 5, 5), 10, 10, 10)
center = (10, 10, 10)
sm.translate(box, (-center[0], -center[1], -center[2]))
sm.scale(box, (2, 2, 2))
sm.translate(box, center)
```

### Create a Pattern
```python
import copy
parts = []
base = sm.create_box((0, 0, 0), 5, 5, 5)
for i in range(4):
    part = sm.create_box((0, 0, 0), 5, 5, 5)
    sm.rotate(part, (0, 0, 0), (0, 0, 1), 90 * i)
    sm.translate(part, (10, 0, 0))
    parts.append(part)
result = sm.merge_breps(parts, operation=sm.BooleanOp.UNION)
```

- **`scale_about_point(target, ref_point, scale)`** — wraps `SmApiScaleByPt`. Scales about an arbitrary reference point (translates to origin, scales, translates back) without manual translate/scale chains.
- **`transform(target, placement)`** — wraps `SmApiTransform`. Applies a combined rotation + translation expressed as an `Axis2Placement`. Use when you have a single rigid transform to apply.
