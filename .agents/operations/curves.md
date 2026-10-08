<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Curve Creation and Operations

**Source:** `source/SM_API/inc/SmApiCurves.h`
**Python:** `_omni_solid.create_line_segment`, `create_line`, `create_circle`, `create_arc`, `create_helix`, `create_ellipse`, `create_rectangle`, `create_regular_polygon`, `create_curve`, `create_canonical_curve`, `create_curve_approx_points`, `create_curve_interp_points`, `offset_curve`, `evaluate_curve`, `evaluate_continuity`, `drop_curve_to_surface`, `convert_to_lines_and_arcs`, `project_curve_to_surface`, `lift_uv_curve`, `make_curves_compatible`, `order_curves`, `remove_curve_knots`

## Overview

Curves are the fundamental building blocks for profiles, paths, and trimming operations. They are used as inputs to sweeps (to define the profile being extruded or revolved), as construction geometry for trimming surfaces, and as paths for pipe sweeps and curve sweeps.

All curves in SMLib are represented as B-spline curves (or their special-case subclasses: lines, circles, ellipses). The Python bindings return opaque `Curve` handles that can be passed to sweep and other operations.

## Operations

### create_line_segment
Create a straight line segment between two points.

| Parameter | Type | Description |
|---|---|---|
| `start` | tuple | Start point (x, y, z) |
| `end` | tuple | End point (x, y, z) |

```python
line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
```

### create_circle
Create an analytic circle in the XY plane.

| Parameter | Type | Description |
|---|---|---|
| `center` | tuple | Center point (x, y, z) |
| `radius` | float | Circle radius |

**Design notes:**
- The circle lies in the XY plane (Z = center.z). For circles in other planes, you'd need the C API with axis placement.
- Returns an analytic `Circle`. It **is** a `BSplineCurve` (usable anywhere a curve is accepted, e.g. as a sweep profile) and additionally exposes analytic accessors: `center()`, `radius()`, `normal()`, `x_axis()`, `y_axis()` (for the XY-plane circle, `x_axis`=+X, `y_axis`=+Y, `normal`=+Z).
- `center` must be finite and `radius` finite and positive; otherwise `RuntimeError` (`SM_ERR_INVALID_INPUT`) is raised.

```python
circ = sm.create_circle((0, 0, 0), 5.0)
circ.center(), circ.radius(), circ.normal()   # (0,0,0), 5.0, (0,0,1)
# Use as sweep profile:
solid = sm.linear_sweep([circ], (0, 0, 1), 10.0)
```

### create_arc
Create a circular arc in the XY plane.

| Parameter | Type | Description |
|---|---|---|
| `center` | tuple | Center point (x, y, z) |
| `radius` | float | Arc radius |
| `start_angle_deg` | float | Start angle from +X axis (degrees) |
| `end_angle_deg` | float | End angle from +X axis (degrees) |

**Design notes:**
- Angles are measured counter-clockwise from the +X axis in the XY plane.
- For a semicircle: `start_angle_deg=0, end_angle_deg=180`.
- Returns an open B-spline curve.

```python
arc = sm.create_arc((0, 0, 0), 5.0, start_angle_deg=0, end_angle_deg=180)
```

### create_helix
Create a B-spline helix about the +Z axis.

| Parameter | Type | Description |
|---|---|---|
| `origin` | tuple | Base point (x, y, z), must be finite; the helix winds about the +Z axis through it, spanning Z = `origin.z` to `origin.z + height` |
| `height` | float | Axial length along +Z; range `[1e-7, 1e18]` |
| `radius_start` | float | Radius at the base, Z = `origin.z`; range `[1e-7, 1e18]` (zero rejected) |
| `radius_end` | float | Radius at the top, Z = `origin.z + height`; range `[1e-7, 1e18]` (zero rejected) |
| `turns` | float | Number of 360° revolutions over the height (fractional allowed, but see design notes for sweeping low-turn helixes); range `[1e-7, 124.25]` |
| `right_handed` | bool | Winding direction about +Z (default `True`); keyword-only |
| `tolerance` | float | **Guaranteed** max 3D deviation of the returned curve from the true helix (default `1e-4`); range `[1e-6, 1e18]`; keyword-only |

**Design notes:**
- Built about the world +Z axis at `origin`; transform the result to orient it along another axis.
- Equal radii give a constant-radius helix; unequal radii a conical (tapered) helix.
- Returns a `BSplineCurve`. Use it as a `pipe_sweep` path for springs, threads, and coils.
- **Low-turn helixes wind steeply.** Below ~1 turn the path is nearly straight, so a tight `tolerance` may be unmeetable (raising `SM_ERR_NOT_WITHIN_TOLERANCE` — loosen it) and `pipe_sweep` may reject the path outright. Prefer `turns >= 1` for a reliable sweep path. When the sweep does succeed it always yields a clean solid (a manifold pipe with two circular caps); it never returns degenerate or fragmented topology.
- All inputs are validated up front: a non-finite `origin`, out-of-range argument, `NaN`, infinity, non-positive value, or a `turns` count past the fitter's control-point limit (`(NL_CCPLIM-6)/8 ≈ 124.25`) raises `RuntimeError` (`SM_ERR_INVALID_INPUT`) and returns no curve.
- `tolerance` is a **hard contract**. After fitting, the true maximum 3D deviation from the analytic helix is measured (dense sampling, each curve point paired with the helix point at the same axial height). If it exceeds `tolerance` — e.g. many `turns` at a tight `tolerance` that overruns the fitter's span cap — the call raises `RuntimeError` (`SM_ERR_NOT_WITHIN_TOLERANCE`) and returns no curve rather than an out-of-tolerance approximation.

```python
helix = sm.create_helix((0, 0, 0), height=10.0, radius_start=2.0, radius_end=2.0, turns=5.0)
spring = sm.pipe_sweep(helix, radius=0.3)
```

### create_curve
Create a cubic B-spline curve from a sequence of control points.

| Parameter | Type | Description |
|---|---|---|
| `points` | list[tuple] | Ordered control points (at least 4) |

**Design notes:**
- The points are control points: the curve approximates them but does not necessarily pass through them.
- At least 4 points are required; fewer raise `RuntimeError` (`SM_ERR_INVALID_INPUT`).
- The curve is always cubic (degree 3) with knots spanning `[0, 1]`; the interior knots are not evenly spaced.
- For precise control over knots, weights, and degree, use `create_canonical_curve`.
- For an approximating curve (doesn't pass exactly through points), use `create_curve_approx_points`.

```python
pts = [(0, 0, 0), (3, 5, 0), (6, 2, 0), (10, 4, 0)]
crv = sm.create_curve(pts)
# Use as a pipe path:
pipe = sm.pipe_sweep(crv, radius=1.0)
```

### create_curve_interp_points
Create a B-spline curve that passes exactly through a sequence of points.

| Parameter | Type | Description |
|---|---|---|
| `points` | list[tuple] | Ordered points to interpolate (≥ 4, all finite) |
| `parameterization` | `CurveParameterization` | Knot spacing (keyword-only, default `UNIFORM`) |

**Design notes:**
- The curve interpolates (passes through) every point, on parameter interval `[0, 1]`.
- Requires at least **4** finite points (degree-3 fit). Fewer points or a non-finite coordinate raises `RuntimeError` (`SM_ERR_INVALID_INPUT`).
- `parameterization` controls knot spacing: `UNIFORM` (even), `CHORD_LENGTH` (by inter-point distance), or `CENTRIPETAL` (by sqrt of distance — reduces overshoot/looping on sharp turns or uneven spacing).
- For an approximating (non-interpolating) fit use `create_curve_approx_points`; for full knot/weight/degree control use `create_canonical_curve`.

```python
pts = [(0, 0, 0), (2, 3, 0), (5, 1, 0), (8, 4, 0), (10, 0, 0)]
crv = sm.create_curve_interp_points(pts, parameterization=sm.CurveParameterization.CENTRIPETAL)
```

### offset_curve
Create a curve offset from an existing curve by a specified distance.

| Parameter | Type | Description |
|---|---|---|
| `curve` | BSplineCurve | Source curve to offset |
| `distance` | float | Offset distance (positive = right, negative = left when looking along curve direction) |

**Design notes:**
- The offset direction is determined by the sign: positive offsets to the right of the curve direction, negative to the left.
- Self-intersecting offsets (when distance exceeds minimum curvature radius) may produce unexpected results.
- The offset is taken in the curve's own plane. A straight curve has no plane of its own, so it is offset in the XY plane (normal +Z), or in the XZ plane (normal +Y) if it is parallel to Z.

```python
arc = sm.create_arc((0, 0, 0), 5.0, 0, 180)
outer = sm.offset_curve(arc, 2.0)
inner = sm.offset_curve(arc, -2.0)
```

### SmApiRemoveCurveKnots / remove_curve_knots

Remove B-spline knots using the curve's effective approximation tolerance as the allowed
shape deviation. This simplifies the curve representation.

| Parameter | Type | Description |
|---|---|---|
| `curve` | Curve | Curve to simplify in place |

**Returns:** the same curve handle for chaining.

**Design notes:**

- Available as `sm.remove_curve_knots(curve)` and `sm.curves.remove_curve_knots(curve)`.
- Calls `SmBSplineCurve::RemoveKnots`; non-B-spline curve types are unchanged.
- The API passes `SmTol::GetApproxTol3d(curve)`, resolved from owning topology or context,
  to the kernel and propagates failures. It exposes no tolerance argument or removed-knot count.

## Building Profiles for Sweeps

### Closed Loop from Multiple Curves
Sweep operations expect curves that form a closed loop. Combine line segments and arcs:

```python
# Rectangle profile — two options:
# Option 1: use create_rectangle (returns list of B-spline segments)
segs = sm.create_rectangle((5, 2.5, 0), 10, 5)
# Option 2: build manually from line segments
lines = [
    sm.create_line_segment((0, 0, 0), (10, 0, 0)),
    sm.create_line_segment((10, 0, 0), (10, 5, 0)),
    sm.create_line_segment((10, 5, 0), (0, 5, 0)),
    sm.create_line_segment((0, 5, 0), (0, 0, 0)),
]
solid = sm.linear_sweep(lines, (0, 0, 1), 20.0)
```

### Profile Design Tips
- Curves in a profile should connect end-to-end (head-to-tail) to form a closed loop.
- Gaps between curve endpoints will cause sweep failures.
- All profile curves should be coplanar for extrusion sweeps.
- For rotational sweeps, the profile should be entirely on one side of the rotation axis.

## Surface-Related Curve Operations

### project_curve_to_surface
Project a 3D curve onto a surface, returning the resulting 3D on-surface curves.

| Parameter | Type | Description |
|---|---|---|
| `curve` | BSplineCurve | 3D curve to project |
| `surface` | Surface | Target surface |

**Returns:** list of 3D curves lying on the surface.

```python
line = sm.create_line_segment((0, 0, 5), (10, 10, 5))
srf = sm.create_surface_from_corner_points((0,0,0), (10,0,0), (0,10,0), (10,10,0))
projected = sm.project_curve_to_surface(line, srf)
```

### lift_uv_curve
Lift a 2D UV-space curve onto a surface, producing a 3D curve.

| Parameter | Type | Description |
|---|---|---|
| `uv_curve` | BSplineCurve | 2D curve in surface parameter space |
| `surface` | Surface | Surface on which to evaluate |

**Returns:** list of 3D curves on the surface.

### make_curves_compatible
Reparameterize a set of curves to have compatible knot vectors. Required before skinning.

| Parameter | Type | Description |
|---|---|---|
| `curves` | list[BSplineCurve] | Curves to make compatible (modified in-place; a failure can leave some already modified) |

### order_curves
Reorder coplanar curves into loop sequence (e.g. for trimming loops). Curves are not reversed and
per-curve orientation is not reported, so consecutive curves need not meet head to tail. Open or
non-planar input fails.

| Parameter | Type | Description |
|---|---|---|
| `curves` | list[BSplineCurve] | Curves to reorder |

**Returns:** reordered list of curves.

- **`create_canonical_curve`** — full B-spline specification with knots, weights, degree, and curve form. See the docstring on the Python function for the parameter list.
