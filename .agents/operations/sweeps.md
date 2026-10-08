<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Sweep Operations

**Source:** `source/SM_API/inc/SmApiBrep.h`
**Python:** `_omni_solid.linear_sweep`, `rotational_sweep`, `draft_sweep`, `pipe_sweep`, `taper_extrude`, `sweep_along_planar_path`, `linear_sweep_with_repetitions`, `rotational_sweep_with_repetitions`, `curve_sweep`, `curve_sweep_from_faces`, `curve_sweep_from_edges`, `non_manifold_linear_sweep`, `non_manifold_rotational_sweep`

## Overview

Sweeps create solid bodies by moving a 2D profile through space. They are the primary way to create shapes that aren't simple primitives. The profile is typically a set of curves (circle, rectangle, spline) and the sweep path can be linear, rotational, or along an arbitrary curve.

All sweep functions produce `SmBrep` objects with exact NURBS geometry. The resulting surfaces inherit the mathematical properties of the profile curves and sweep motion.

## Operations

### linear_sweep
Extrude curves along a direction vector to create a solid.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `curves` | list[Curve] | — | Profile curves to sweep (must form a closed loop for solid) |
| `direction` | tuple | — | Sweep direction vector |
| `distance` | float | — | How far to sweep |
| `cap_ends` | bool | `True` | Close the ends to make a solid |

**Design notes:**
- Curves should form a closed planar loop for a manifold solid result. Open curves produce sheet bodies.
- The direction vector defines the sweep direction; it does not need to be a unit vector. The actual sweep distance is `distance`, not the vector magnitude.
- `cap_ends=True` creates planar cap faces at both ends. Set `False` for an open shell.

```python
circ = sm.create_circle((0, 0, 0), 5.0)
tube = sm.linear_sweep([circ], direction=(0, 0, 1), distance=20.0, cap_ends=True)
```

### rotational_sweep
Revolve curves around an axis to create a solid of revolution.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `curves` | list[Curve] | — | Profile curves to revolve |
| `base_point` | tuple | — | A point on the rotation axis |
| `axis` | tuple | — | Direction of the rotation axis |
| `angle_deg` | float | — | Angle of revolution in degrees (360 = full revolution) |
| `cap_ends` | bool | `True` | Close the ends |

**Design notes:**
- For a full solid of revolution (vase, wine glass), use `angle_deg=360`.
- For partial revolutions, set angle < 360 and `cap_ends=True` to close the wedge.
- Profile curves should NOT cross the rotation axis (produces self-intersection).
- The profile should be on one side of the axis. Distance from axis becomes the radius at that point.

```python
# Create a simple profile line and revolve to make a cylinder
line = sm.create_line_segment((5, 0, 0), (5, 0, 10))
cyl = sm.rotational_sweep([line], base_point=(0, 0, 0), axis=(0, 0, 1),
                           angle_deg=360, cap_ends=True)

# Revolve a curved profile to make a vase shape
pts = [(3, 0, 0), (5, 0, 3), (4, 0, 6), (5, 0, 9), (3, 0, 12)]
profile = sm.create_curve(pts)
vase = sm.rotational_sweep([profile], base_point=(0, 0, 0), axis=(0, 0, 1),
                            angle_deg=360, cap_ends=True)
```

### draft_sweep
Extrude curves with a draft angle (taper). Creates walls that angle inward or outward.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `curves` | list[Curve] | — | Profile curves (must be planar and closed) |
| `height` | float | — | Extrusion height |
| `angle_deg` | float | — | Draft angle in degrees (positive = taper inward) |
| `fillet_corners` | bool | `False` | `True` = fillet expanding corners, `False` = linear extension |
| `cap_ends` | `CapEnds` | `CapEnds.BOTH` | Which ends to cap: `NONE`, `START` (profile end), `END` (drafted end), `BOTH` |

**Design notes:**
- Draft angle controls the taper: `0°` = straight extrusion (same as `linear_sweep`), positive angles taper inward (narrowing), angles toward 90° produce nearly vertical walls.
- `fillet_corners=True` produces smooth rounded corners where adjacent drafted faces meet. `False` extends the faces linearly until they intersect (sharp corners).
- `cap_ends` is a `CapEnds` enum. `BOTH` (default) caps both ends, `START` caps only the original profile end, `END` caps only the drafted (offset) end, `NONE` leaves it open.

```python
circ = sm.create_circle((0, 0, 0), 5.0)
tapered = sm.draft_sweep([circ], height=10.0, angle_deg=5.0,
                          fillet_corners=False, cap_ends=sm.CapEnds.BOTH)
```

### pipe_sweep
Sweep a circular cross-section along a path curve to create a pipe/tube.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `path` | Curve | — | Path curve to sweep along (must be a B-spline curve). Subject — passed positionally. |
| `radius` | float | — | Pipe radius. **Keyword-only.** |
| `cap_ends` | bool | `True` | Close the pipe ends. **Keyword-only.** |

**Design notes:**
- The path curve must be a `BSplineCurve`. Create it with `create_curve(points)`.
- The pipe radius should be smaller than the minimum curvature radius of the path, otherwise self-intersection occurs.
- For variable-radius pipes, use the C API `SmApiCreateCurveSweep` with scale curves.

```python
path_pts = [(0, 0, 0), (5, 5, 0), (10, 0, 5), (15, 5, 5)]
path = sm.create_curve(path_pts)
pipe = sm.pipe_sweep(path, radius=1.0, cap_ends=True)
```

### taper_extrude
Extrude coplanar curves with draft angle and optional fillet corners. Similar to `draft_sweep` but with explicit control over inner/outer taper direction.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `curves` | list[Curve] | — | Coplanar curves to extrude |
| `height` | float | — | Extrusion height (positive or negative for direction) |
| `draft_angle_deg` | float | — | Draft angle: 0–90 = inward taper, 90–180 = outward taper |
| `cap_ends` | `CapEnds` | `CapEnds.BOTH` | Which ends to cap (same enum as `draft_sweep.cap_ends`) |
| `fillet_corner` | bool | `False` | Fillet or linear extension at corners |

**Design notes:**
- Draft angle interpretation: 0° = maximum inward taper, 90° = straight extrusion (no taper), 180° = maximum outward taper.
- Height can be negative to extrude in the opposite direction.
- `cap_ends` is the same `CapEnds` enum used by `draft_sweep`; `START` is the profile end, `END` is the drafted end.

```python
circ = sm.create_circle((0, 0, 0), 5.0)
r = sm.taper_extrude([circ], height=10.0, draft_angle_deg=80.0,
                     cap_ends=sm.CapEnds.BOTH)
```

## Choosing the Right Sweep

| Goal | Sweep Type | Notes |
|---|---|---|
| Straight extrusion | `linear_sweep` | Simplest; profile moves in a straight line |
| Solid of revolution | `rotational_sweep` with 360° | Vases, wheels, axisymmetric parts |
| Partial revolution | `rotational_sweep` with < 360° | Curved walls, sectors |
| Tapered extrusion | `draft_sweep` or `taper_extrude` | Molded parts with draft angles |
| Tube along path | `pipe_sweep` | Pipes, cables, tubes |
| OCP-style spherical corners on a rectangular pipe | [public composition example](../../Examples/PyAPI/round_corner_pipe.py) | Planar sweep + primitives + exact Booleans |
| Complex sweep | `curve_sweep` | Variable cross-section along path |

## Advanced Sweeps (now in Python)

- **`sweep_along_planar_path(profile_curves, path_curves, move_profile=True, cap_ends=CapEnds.BOTH)`** — sweep profile boundary curves along an ordered, connected planar path. See [Planar-path profile and material semantics](#planar-path-profile-and-material-semantics).
- **`curve_sweep(profile_curves, path_curve, scale_reference=None, scale_curve=None, *, cap_ends=True)`** — full sweep with optional scale curves for variable cross-sections. With `cap_ends=True` (default) a closed profile produces a manifold solid; set `cap_ends=False` for the open swept shell.
- **`linear_sweep_with_repetitions(curves, direction, distance, *, repetitions, cap_ends=True)`** — repeated end-to-end linear sweeps for patterns
- **`rotational_sweep_with_repetitions(curves, base_point, axis, angle_deg, *, repetitions, cap_ends=True)`** — repeated end-to-end rotational sweeps
- **`curve_sweep_from_faces(faces, path_curve, scale_reference=None, scale_curve=None, *, cap_ends=True)`** — sweep face edge loops along a path
- **`curve_sweep_from_edges(edges, path_curve, scale_reference=None, scale_curve=None, *, cap_ends=True)`** — sweep edges along a path
- **`non_manifold_linear_sweep(brep_to_sweep, direction, distance, *, region_brep=None, do_stitching=True, do_merge=False, faces=None, edges=None, vertices=None)`** — sweep specific faces/edges/vertices of a Brep along a vector, with stitching and merge
- **`non_manifold_rotational_sweep(brep_to_sweep, base_point, axis, angle_deg, *, region_brep=None, do_stitching=True, do_merge=False, faces=None, edges=None, vertices=None)`** — same, rotational

The three `curve_sweep` variants borrow their profile, path, and optional scale
curves. They create working copies and leave every input unchanged on success or
failure.

### Non-manifold sweep selection and region

The `faces`, `edges` and `vertices` arguments are selected as a group, not
individually:

| `faces`, `edges`, `vertices` | Result |
|---|---|
| All `None` | Whole Brep is swept |
| At least one non-empty sequence | Only the listed topology is swept; a `None` argument means ignore that kind, not "all of them" |
| At least one sequence supplied, but every supplied sequence empty | `ValueError` — previously this reported success while sweeping nothing |

Every entry must be live topology owned by `brep_to_sweep`. A `None` entry
raises `ValueError`; a handle belonging to another Brep is rejected by
`SM_API`, because sweeping it mutates `brep_to_sweep` with foreign topology.

`region_brep=None` sweeps into `brep_to_sweep`'s own infinite region, which is
what the C++ and Python signatures have always documented.

`region_brep` must be `None` or `brep_to_sweep` itself. Any other Brep raises
`ValueError` from Python and `SM_ERR_INVALID_INPUT` from `SM_API`: the sweep
helpers would build the new topology there while the stitch/orient pass,
validation and the Python return value all use `brep_to_sweep`, leaving the
result coherently in neither. `SmTopologySweep::DoSweep` still accepts a
distinct Brep for kernel callers willing to handle that split.

### Planar-path profile and material semantics

`sweep_along_planar_path` sweeps boundary curves. The profile curves must be
coplanar; they may form open wires or one or more simple closed loops. Open
wires produce sheet geometry and cannot form endpoint caps. Closed loops can
bound caps and use even/odd geometric containment: outermost areas are
material, and each nesting level alternates void and material. Loop order and
winding do not select which nested areas are material.

| Profile and path | `cap_ends` | Result |
|---|---|---|
| Open profile wires | Any | Sheet or open-shell geometry |
| Closed loops, open path | `NONE` | Sides open at both endpoints; no enclosed material |
| Closed loops, open path | `START` or `END` | One endpoint capped; open shell with no enclosed material |
| Closed loops, open path | `BOTH` | Nested solid when assembly produces a closed manifold boundary |
| Closed loops, closed path | Any | No endpoint caps; nested solid when assembly produces a closed manifold boundary |

For example, sweeping three concentric loops fills the inner core and the band
between the middle and outer loops; the band between the inner and middle
loops is void. This is the three-dimensional continuation of the containment
rule used to construct the planar cap faces.

`cap_ends` requests endpoint faces; it does not by itself prove that a solid
can be constructed. Self-intersection, invalid profile loops, or a corner
transition that cannot be assembled may fail. When SM_API reports a failure,
its result is null. A completed manifold boundary with solid intent is
classified from the infinite void region inward instead of being returned as
an all-void, zero-volume body.

Tangent-discontinuous path joins use sharp, mitered transitions. This API does
not insert rounded transition faces or expose a corner-transition selector.
Author rounded motion explicitly with tangent-connected arc or spline path
curves. Material classification does not change or add transition geometry.

### Composed OCP-style round-corner pipe

The bounded closed-rectangle `RoundCorner` outcome used by the CAD SDK does
not require another kernel sweep mode. The runnable
[round-corner pipe example](../../Examples/PyAPI/round_corner_pipe.py)
composes it entirely from stable public operations:

1. Create the circular profile and historical mitered result with
   `sweep_along_planar_path`.
2. At each orthogonal vertex, linearly sweep a rectangular profile to make an
   oriented box spanning the outside quadrant.
3. Compute `box - sphere`, where the sphere is centered at the path vertex and
   has the pipe radius.
4. Subtract that cutter from the accumulating mitered pipe.
5. Require a manifold result with finite positive volume.

The checked-in qualification covers the 10-by-10, radius-1 reference in both
path windings and two rigid orientations. Its fine volume matches the OCP
reference `124.51916300794949` within `2e-5`, and its tight bounds match the
expected pipe envelope. The test intentionally checks geometric outcomes, not
a particular face partition.

This spherical corner is not a conventional centerline fillet. A
tangent-connected path arc creates a toroidal elbow with an independent bend
radius; use an authored arc or spline when that is the intended motion. The
reference composition is deliberately limited to a sufficiently clear,
four-leg rectangular path. Its Boolean calls consume only temporary boxes,
spheres, cutters, and intermediate pipe results; the input path curves remain
unchanged.
