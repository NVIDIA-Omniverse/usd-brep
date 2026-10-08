<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Intersection Operations

**Source:** `source/SM_API/inc/SmApiIntersectors.h`
**Python:** `_omni_solid.intersect_breps`, `intersect_brep_with_plane`, `intersect_curves`, `intersect_surfaces`, `intersect_faces`, `intersect_curve_surface`, `intersect_curve_brep`, `intersect_curve_face`

## Overview

Intersection operations compute the geometric intersections between objects — where curves cross, surfaces meet, or solids overlap. The results can contain intersection curves, points, or both. These are purely query operations — they do not modify the input geometry.

Intersection is also the mathematical foundation for booleans, but these standalone intersector functions return the intersection geometry without performing any boolean classification.

## Operations

### intersect_breps
Compute the intersection curves and points between two BReps.

| Parameter | Type | Description |
|---|---|---|
| `brep1` | Brep | First BRep |
| `brep2` | Brep | Second BRep |

**Returns:** tuple of `(curves, points)` where `curves` is a list of intersection curves and `points` is a list of intersection points.

**Design notes:**
- Returns the curves where the surfaces of the two BReps intersect.
- Points are returned for tangential or degenerate intersections.
- The input BReps are NOT consumed (unlike booleans) — they remain usable.
- Useful for interference checking: if the intersection produces curves, the bodies overlap.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sph = sm.create_sphere((5, 5, 5), 4)
curves, points = sm.intersect_breps(box, sph)
print(f"Found {len(curves)} intersection curves, {len(points)} intersection points")
```

### intersect_brep_with_plane
Compute section curves where a plane cuts through a BRep.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to section |
| `plane_point` | tuple | Any point on the cutting plane |
| `plane_normal` | tuple | Normal vector of the cutting plane |

**Returns:** list of section curves.

**Design notes:**
- The plane is defined by a point and a normal vector. The normal doesn't need to be normalized.
- Returns closed curves for each connected cross-section loop.
- This is a non-destructive query — the BRep is unchanged. For actually cutting the BRep, use `cut()`.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
# Horizontal section at mid-height
curves = sm.intersect_brep_with_plane(box, (0, 0, 5), (0, 0, 1))
# Returns a closed rectangular curve (the cross-section outline)
```

## Use Cases

| Goal | Function | Notes |
|---|---|---|
| Interference check | `intersect_breps` | Non-empty result = bodies overlap |
| Cross-section view | `intersect_brep_with_plane` | Get outlines at a given plane |
| Find contact curves | `intersect_breps` | Where two parts touch |
| Slice analysis | `intersect_brep_with_plane` at multiple Z | Stack of cross-sections |

## Additional Intersectors (now in Python)

- **`intersect_curves(curve1, curve2)`** — returns (points, params_on_curve1, params_on_curve2)
- **`intersect_surfaces(surface1, surface2)`** — returns intersection curves
- **`intersect_curve_surface(curve, surface)`** — returns (curves, points) where a curve meets a surface
- **`intersect_curve_brep(curve, brep)`** — returns (curves, points) where a curve passes through a BRep
- **`intersect_curve_face(curve, face)`** — returns (curves, points) where a curve intersects a trimmed face

### Curve-Curve Intersection Details
`intersect_curves` returns:
- 3D intersection points as list of tuples
- Parameter values on curve 1 at each intersection
- Parameter values on curve 2 at each intersection

This is useful for finding where profiles cross, computing trim points, and building intersection graphs.

- **`intersect_faces(face1, face2)`** — trimmed face-versus-face intersection. Returns `(curves, points)` clipped to both faces' trimming loops. Requires `Face` handles obtained from topology traversal (e.g. `brep.faces()`).
