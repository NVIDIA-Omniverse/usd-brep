<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Offset and Shell Operations

**Source:** `source/SM_API/inc/SmApiBrep.h`, `source/SM_API/inc/SmApiPrimitives.h`
**Python:** `_omni_solid.offset_brep`, `shell_brep`, `create_offset_profile`

## Overview

Offset and shell operations create new surfaces at a uniform distance from the original. **Offset** creates a copy of the entire BRep displaced by a distance. **Shell** hollows out a solid by removing selected faces and offsetting the remaining faces inward. **Offset profile** takes a closed loop of planar curves and creates an offset region as a sheet body.

These operations are fundamental for creating wall thicknesses in parts, generating inner/outer surfaces for mold design, creating clearance envelopes, and producing 2D offset regions from curve loops.

## Operations

### offset_brep
Create a new BRep whose surfaces are uniformly offset from the original.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | Source BRep |
| `distance` | float | — | Offset distance (positive = outward along normals, negative = inward) |
| `extended_offset` | bool | `True` | Extend and intersect at convex edges (sharp outer corners) |
| `self_intersection` | bool | `True` | Handle self-intersections in the offset |

**Design notes:**
- Positive distance offsets outward (makes the shape larger). Negative offsets inward (makes it smaller).
- `extended_offset=True` handles convex edges by extending adjacent offset surfaces until they intersect, producing sharp corners. Set `False` for smoother transitions but potential gaps.
- `self_intersection=True` detects and resolves self-intersections that can occur when offsetting concave regions by a distance larger than the local curvature radius.
- The offset distance should be smaller than the minimum radius of curvature of any concave region. Larger offsets may produce invalid geometry.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
bigger = sm.offset_brep(box, distance=1.0)       # slightly larger box
smaller = sm.offset_brep(box, distance=-1.0)     # slightly smaller box
```

### shell_brep
Create a signed-thickness wall by removing selected faces and offsetting the
remaining boundary.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | Source BRep |
| `distance` | float | — | Signed wall thickness: positive expands outward; negative creates an inward wall |
| `extended_offset` | bool | `True` | Extend/intersect at convex edges |
| `self_intersection` | bool | `True` | Handle self-intersections |
| `create_solid` | bool | `True` | For sheet-body inputs, create ruled surfaces between lamina edges |
| `faces_to_shell` | list[Face] | `[]` | Faces to remove (open faces). Empty creates a closed wall. |

**Design notes:**
- The source BRep and selected faces remain unchanged.
- Selected faces become openings; an empty selection creates a closed wall.
- Positive distance expands outward; negative distance offsets inward.
- Solids use Boolean difference. `create_solid=True` joins sheet-body lamina
  edges with ruled surfaces.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
top_face = box.faces()[0]  # remove the top face
shell = sm.shell_brep(box, distance=1.0, faces_to_shell=[top_face])
# Result: hollow box with 1.0 wall thickness, open on top
```

### create_offset_profile
Create an offset region from a closed loop of planar curves.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `curves` | list[Curve] | — | Planar curves forming at least one closed loop. All curves must lie in a single plane. |
| `distance` | float | — | Offset distance |
| `offset_type` | `OffsetType` | `OffsetType.BOTH` | Offset side: `LEFT`, `RIGHT`, or `BOTH` |
| `round_corners` | bool | `True` | Round expanding corners (`False` = extend them) |
| `shell_result` | bool | `False` | Return a shell result (only valid for `LEFT` or `RIGHT`, not `BOTH`) |

**Design notes:**
- Input curves must be coplanar and form at least one closed loop.
- The input array and every `Curve` are borrowed and remain caller-owned and
  unchanged on success or failure. This includes curves borrowed from an
  existing BRep through `Edge.curve()`; the source BRep remains unchanged.
- `offset_type` is an `OffsetType` enum. `BOTH` (default) produces a band region on both sides of the input loop. `LEFT` / `RIGHT` produce a one-sided offset.
- `shell_result=True` returns a shell topology rather than a face region; only meaningful when `offset_type` is `LEFT` or `RIGHT`.
- This is a curve-input operation that produces a new BRep, distinct from `offset_brep` which offsets an existing BRep's surfaces.

```python
# Build a closed square loop
seg1 = sm.create_line_segment((0, 0, 0), (10, 0, 0))
seg2 = sm.create_line_segment((10, 0, 0), (10, 10, 0))
seg3 = sm.create_line_segment((10, 10, 0), (0, 10, 0))
seg4 = sm.create_line_segment((0, 10, 0), (0, 0, 0))
profile = sm.create_offset_profile([seg1, seg2, seg3, seg4], distance=1.0)
```

## Choosing Parameters

### Offset Distance
- Must be positive and smaller than the minimum feature size of the BRep.
- For shelling, the wall thickness should be reasonable relative to the part size. A 10×10×10 box with wall thickness > 5.0 would produce degenerate inner geometry.
- For convex shapes (sphere, box), almost any offset distance works. For concave shapes, the distance is limited by the minimum curvature radius.

### Extended Offset
- `True` (default) is appropriate for mechanical parts with sharp edges.
- `False` may be needed for organic shapes where you want smooth offset behavior.

### Self-Intersection Handling
- Leave `True` unless you know the offset cannot self-intersect. Self-intersection detection adds cost but prevents invalid geometry.

## Common Patterns

### Create a Hollow Container
```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
# Get the top face (Z=10 face) — check which face index
faces = box.faces()
container = sm.shell_brep(box, distance=0.5, faces_to_shell=[faces[0]])
```

### Create Inner and Outer Envelopes
```python
part = sm.create_sphere((0, 0, 0), 10.0)
outer_envelope = sm.offset_brep(part, distance=2.0)
inner_envelope = sm.offset_brep(part, distance=-2.0)
```

### Build a 2D Offset Band from a Curve Loop
```python
loop = [seg1, seg2, seg3, seg4]               # closed planar loop
band = sm.create_offset_profile(loop, distance=1.0, offset_type=sm.OffsetType.BOTH)
```
