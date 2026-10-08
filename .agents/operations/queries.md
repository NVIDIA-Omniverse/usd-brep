<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Query Operations

**Source:** `source/SM_API/inc/SmApiQueries.h`, `source/SM_API/inc/SmApiSurfaces.h`, `source/SM_API/inc/SmApiCurves.h`
**Python (flat):** `brep_closest_point`, `brep_closest_point_all`, `curve_closest_point`, `curve_closest_point_all`, `surface_closest_point`, `is_manifold_solid`, `material_census`, `compute_volume`, `brep_relationship`, `brep_distance`, `get_closest_point`, `get_edges_via_api`, `get_faces_via_api`, `find_edge`, `find_face`, `evaluate_curve`, `evaluate_surface_point`, `evaluate_surface_normal`, `evaluate_surface_derivatives`, `brep_bounding_box`, `brep_copy`, `face_bounding_box`, `face_area`, `edge_bounding_box`, `edge_length`, `vertex_point`
**Python (methods):** `Brep.{info, bounding_box, center, volume, mass_properties, distance_to, relationship_to, is_manifold_solid, material_census, copy, face_count, edge_count, vertex_count, faces, edges, vertices}`; `Face.{bounding_box, area, surface, uv_domain, edges, vertices, outer_loop_edges, boundary_loops, boundary_outer_loop, point_at_uv, normal_at_uv, outward_normal_at_uv, interior_point, classify_uv, closest_point, centroid, brep}`; `Edge.{bounding_box, length, curve, start_vertex, end_vertex, vertices, other_vertex, faces, point_at, closest_point, parameter_range, classify_parameter, tangent_at, brep}`; `Vertex.{point, edges, faces, brep}`; `Curve.{parameter_range, start_point, end_point, evaluate, length, is_closed, is_planar}`; `Surface.{u_domain, v_domain, uv_domain, evaluate, normal, derivatives, is_planar, is_periodic_u, is_periodic_v}`

**Developer topology (separate module):** stable face-boundary queries return ordered edge occurrences and direction bits without exposing use topology. Direct loopuse/edgeuse objects, `topology_pick_ray`, and kernel Draw extraction remain on `_smlib_dev` — see **[dev_topology](dev_topology.md)**.

## Overview

Query operations extract information from geometry without modifying it. They are essential for validation (is the solid watertight?), measurement (what's the volume?), and spatial analysis (where is the closest point?).

## Operations

### is_manifold_solid
Check whether a BRep forms a valid closed solid (every edge is shared by exactly 2 faces).

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to check |

**Returns:** `True` if the BRep is a manifold solid, `False` otherwise.

**Design notes:**
- A manifold solid has no free (lamina) edges — every edge connects exactly 2 faces.
- Sheet bodies (planes, open shells) return `False`.
- Use this to validate results after booleans, sweeps, or stitching.
- A `False` result doesn't necessarily mean the geometry is wrong — it might be an intentional open shell.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
assert sm.is_manifold_solid(box)  # True

plane = sm.create_plane((0, 0, 0), 10, 10)
assert not sm.is_manifold_solid(plane)  # False (sheet body)
```

### material_census
Count a BRep's material (solid) regions and enclosed void cavities, resolving nested shells unambiguously.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to query (not modified) |

**Returns:** dict `{'solid_count': int, 'void_count': int}` — material (solid) regions, and enclosed void cavities (the infinite region is never counted).

**Design notes:**
- SMLib stores each connected region separately, so nesting is explicit: a solid with a cavity holding a nested solid is `solid_count == 2`, `void_count == 1`.
- `solid_count` counts material **regions**: for a manifold BRep these are the separate solid bodies; a non-manifold BRep can have adjoining solid regions, counted individually.
- Read-only; counts reflect the BRep's current void flags (`SmRegion::IsVoid()`), which construction and boolean operations set and a correct import preserves. Only a malformed BRep with unset flags needs them re-established (`FindAndSetInfiniteRegion` + `SetRegionIsVoidFlagsForNestedSolids`).
- Cavity count is independent of manifold-ness: a hollow cube is a valid manifold solid (`is_manifold_solid` is `True`) and separately has `void_count == 1`.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
assert sm.material_census(box) == {"solid_count": 1, "void_count": 0}

# Cube with a fully-interior cube removed: one solid shell, one void cavity.
hollow = sm.boolean_difference(sm.create_box((0, 0, 0), 10, 10, 10),
                               sm.create_box((3, 3, 3), 4, 4, 4))
assert hollow.material_census() == {"solid_count": 1, "void_count": 1}
```

### compute_volume
Compute the enclosed volume of a manifold solid.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | BRep to measure |
| `relative_accuracy` | float | `0.001` | Accuracy between 1.0e-1 (coarse) and 1.0e-8 (fine) |

**Returns:** volume as a float (in cubic world units).

**Design notes:**
- Requires a manifold solid (closed volume). Undefined for open shells.
- `relative_accuracy` controls the precision of the numerical integration. Smaller values = more accurate but slower.
- For simple shapes (boxes, spheres), even coarse accuracy gives exact results. For complex shapes with many curved faces, tighter accuracy may be needed.
- Volume is in cubic world units. A 10×10×10 box has volume 1000.0.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
vol = sm.compute_volume(box)  # 1000.0

sph = sm.create_sphere((0, 0, 0), 5)
vol = sm.compute_volume(sph)  # ~523.6 (4/3 * pi * 125)
```

### brep_relationship

Classify how two Breps are arranged, with a distance.

**Python:** `sm.brep_relationship(a, b)` or `a.relationship_to(b)`

**Returns:** `dict` — `{"relationship": str, "distance": float}`. `relationship`
is one of:

- `"separate"` — disjoint; `distance` = surface gap.
- `"touching"` — boundaries meet, interiors do not; `distance` = 0.
- `"interpenetrating"` — interiors overlap; `distance` = 0.
- `"a_contains_b"` — `b` fully inside `a`, no contact; `distance` = wall clearance.
- `"b_contains_a"` — `a` fully inside `b`, no contact; `distance` = wall clearance.

Overlap/containment is decided by classifying sample points — each Brep's
vertices plus one interior point per face — against the other (via
`SmBrep::Point3DClassify`, distinguishing solid material from the exterior/void
region). An overlap that contains none of those sampled points (a shallow or
sliver interpenetration) can still read as `"touching"`; the face-interior
samples make this rare, and it is exact for typical CAD interference.
`brep_distance` returns just this dict's `distance` number.

```python
a = sm.create_box((0, 0, 0), 10, 10, 10)
b = sm.create_box((3, 3, 3), 2, 2, 2)
sm.brep_relationship(a, b)   # {"relationship": "a_contains_b", "distance": 3.0}
```

Wraps `SmApiBrepRelationship`.

### brep_distance

Clearance between two Breps.

**Python:** `sm.brep_distance(a, b)` or `a.distance_to(b)`

**Returns:** `float` — the gap between the two boundary surfaces when the Breps
are **disjoint** — the **wall clearance** when one is nested inside the other —
and `0.0` when they touch or interpenetrate.

Equals the `distance` field of `brep_relationship` (same underlying
computation); call `brep_relationship` when the number needs the arrangement
(separate vs containment) to be meaningful.

```python
a = sm.create_box((0, 0, 0), 1, 1, 1)
b = sm.create_box((4, 0, 0), 1, 1, 1)
sm.brep_distance(a, b)   # 3.0 (disjoint gap)

big    = sm.create_box((0, 0, 0), 10, 10, 10)
inside = sm.create_box((3, 3, 3), 2, 2, 2)   # fully enclosed
sm.brep_distance(big, inside)   # 3.0 (wall clearance to the nearest cavity wall)
```

Wraps `SmApiBrepDistance` -> `SmApiBrepRelationship`.

### brep_closest_point
Find the closest point on a BRep surface to a given point.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to query |
| `point` | tuple | Query point (x, y, z) |

**Returns:** tuple of `(closest_point, distance)` where `closest_point` is a `Vec3`.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
closest, dist = sm.brep_closest_point(box, (5, 5, 15))
# closest.z ≈ 10.0 (top face), dist ≈ 5.0
```

### curve_closest_point
Find the closest point on a curve to a given point.

| Parameter | Type | Description |
|---|---|---|
| `curve` | Curve | Curve to query |
| `point` | tuple | Query point |

**Returns:** tuple of `(closest_point, parameter, distance)` where `parameter` is the curve parameter at the closest point.

```python
line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
closest, param, dist = sm.curve_closest_point(line, (5, 3, 0))
# closest = (5, 0, 0), dist = 3.0
```

### surface_closest_point
Find the closest point on a surface to a given point.

| Parameter | Type | Description |
|---|---|---|
| `surface` | Surface | Surface to query |
| `point` | tuple | Query point |

**Returns:** tuple of `(closest_point, uvs, distance)`. `closest_point` is one globally closest 3D point. `uvs` is a nonempty `list[Vec2]` containing the discrete natural/NURBS parameter representations reported for that selected point, including equivalent representations on periodic seams. Every returned UV evaluates to `closest_point` within solver tolerance.

This function selects one spatial minimum. It does not guarantee discovery of every other spatial point at the same minimum distance or every preimage introduced by a folded or self-overlapping surface.

```python
pts = [(0,0,0), (5,0,0), (10,0,0), (15,0,0),
       (0,5,0), (5,5,0), (10,5,0), (15,5,0),
       (0,10,0),(5,10,0),(10,10,0),(15,10,0),
       (0,15,0),(5,15,0),(10,15,0),(15,15,0)]
srf = sm.create_surface_from_points(pts, 4, 4)
closest, uvs, dist = sm.surface_closest_point(srf, (5, 5, 3))
# closest.z ≈ 0.0, dist ≈ 3.0
# Every UV in uvs evaluates to this same closest point.
```

## Closest-Point Collections (now in Python)

The BRep and curve "All" variants request every solution within solver tolerance of the global
minimum:

- **`brep_closest_point_all(brep, point)`** — globally closest points on a BRep
- **`curve_closest_point_all(curve, point)`** — globally closest points on a curve

Each returns aligned lists:

- `brep_closest_point_all` returns `(points, distances)`.
- `curve_closest_point_all` returns `(points, parameters, distances)`.

## Topology and Spatial Queries

### Face boundary loops

Use `face.boundary_loops()` to reconstruct every trim loop from ordered edge
occurrences. It returns `list[list[tuple[Edge, bool]]]`, with the outer loop
first. Each tuple is `(edge, forward)`: `True` traverses from
`edge.start_vertex()` to `edge.end_vertex()`, and `False` traverses in the
reverse direction. `face.boundary_outer_loop()` returns only the first loop.

The order is SMLib's native upward-faceuse order. Relative to the underlying
surface's parameterization, the outer loop is counterclockwise and holes are
clockwise. The starting edge of a closed loop is unspecified. Repeated edge
occurrences are retained, which is required for periodic seams; a vertex-only
inner loop appears as an empty list.

The nested lists and direction flags are snapshots. Each `Edge` is borrowed
from the face's owning Brep and must not be used after that Brep is consumed,
deleted, or changed by a topology-modifying operation. Query the face again
after topology changes.

```python
for loop in face.boundary_loops():
    for edge, forward in loop:
        start, end = edge.vertices()
        if not forward:
            start, end = end, start
        # ``end`` is the next occurrence's oriented ``start``.
```

Wraps `SmApiFaceGetBoundaryLoops`.

### get_closest_point
Find the closest point on a brep to a given point. Same computation as `brep_closest_point` (the same kernel point solve with the same options); the only difference is that the point comes back as a plain tuple. Kept for compatibility.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to query |
| `point` | tuple | Query point (x, y, z) |

**Returns:** tuple of `(closest_point, distance)`.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
closest, dist = sm.get_closest_point(box, (5, 5, 15))
```

### get_edges_via_api
Retrieve all edges from a BRep using the SmAPI helper. Functionally equivalent to the class method `brep.edges()`. The `_via_api` suffix disambiguates the SmAPI entry point from the class accessor and from `PolyBrep.get_edges` on tessellated meshes.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to query |

**Returns:** list of Edge objects.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
edges = sm.get_edges_via_api(box)
print(f"Box has {len(edges)} edges")  # 12
# Equivalent: edges = box.edges()
```

### get_faces_via_api
Retrieve all faces from a BRep using the SmAPI helper. Functionally equivalent to the class method `brep.faces()`.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to query |

**Returns:** list of Face objects.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
faces = sm.get_faces_via_api(box)
print(f"Box has {len(faces)} faces")  # 6
# Equivalent: faces = box.faces()
```

### find_edge
Find the edge in a brep closest to a given 3D point.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to search |
| `point` | tuple | Point on or near edge (x, y, z) |

**Returns:** tuple of `(edge_index, parameter)` where `edge_index` is the index into the brep's edge list and `parameter` is the curve parameter on that edge.

There is no distance limit: a point far from every edge still returns the nearest edge. Check the distance if the point may be off the Brep.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
edge_idx, param = sm.find_edge(box, (5, 0, 0))
```

### find_face
Find the face in a brep closest to a given 3D point.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to search |
| `point` | tuple | Point on or near face (x, y, z) |

**Returns:** Face object closest to the point.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
face = sm.find_face(box, (5, 5, 10))
```

## Surface Evaluation

Evaluate a standalone `Surface` at a parametric `(u, v)` point. These are read-only queries that do not modify the surface.

### Native STEP evaluator versus ordinary surface evaluation

The `SmApiEvaluateSurface*` wrappers and Python surface methods below use the
ordinary surface parameterization. They do not call the kernel's
`EvaluateSTEP`, whose angular parameters and derivatives are degree-based.
For native integrations using that entry point, see the runnable
[STEP derivative example and migration guide](../../Examples/SMLib/README.md#step-surface-derivatives).
It covers derivative indexing, removal of old caller-side scaling, and explicit
STEP-to-NURBS UV conversion. Do not apply that migration to ordinary surface
evaluation or assume a NURBS parameter is an angle.

### evaluate_surface_point
Evaluate the 3D position of a surface at a `(u, v)` parameter.

| Parameter | Type | Description |
|---|---|---|
| `surface` | Surface | Surface to evaluate |
| `u` | float | Parametric U coordinate |
| `v` | float | Parametric V coordinate |

**Returns:** `(x, y, z)` tuple — the 3D point on the surface.

```python
pts = [(0,0,0), (5,0,0), (10,0,0), (15,0,0),
       (0,5,0), (5,5,5), (10,5,5), (15,5,0),
       (0,10,0),(5,10,5),(10,10,5),(15,10,0),
       (0,15,0),(5,15,0),(10,15,0),(15,15,0)]
srf = sm.create_surface_from_points(pts, 4, 4)
p = sm.evaluate_surface_point(srf, 0.5, 0.5)  # (~7.5, ~7.5, ~3.75)
```

### evaluate_surface_normal
Evaluate the surface normal at a `(u, v)` parameter.

| Parameter | Type | Description |
|---|---|---|
| `surface` | Surface | Surface to evaluate |
| `u` | float | Parametric U coordinate |
| `v` | float | Parametric V coordinate |

**Returns:** `(nx, ny, nz)` tuple — the unit normal vector.

### evaluate_surface_derivatives
Evaluate the surface point and its first partial derivatives at `(u, v)`.

| Parameter | Type | Description |
|---|---|---|
| `surface` | Surface | Surface to evaluate |
| `u` | float | Parametric U coordinate |
| `v` | float | Parametric V coordinate |

**Returns:** nested tuple `((px, py, pz), (dux, duy, duz), (dvx, dvy, dvz))` — the 3D point, the partial derivative with respect to U, and the partial derivative with respect to V.

```python
p, du, dv = sm.evaluate_surface_derivatives(srf, 0.5, 0.5)
```

The cross product `du × dv` (normalized) is the surface normal at that point.

## Geometric Properties (bbox / area / length / point / copy)

These queries return read-only geometric or topological properties of a `Brep` / `Face` / `Edge` / `Vertex`. Each is exposed two ways: as a flat function under `sm.queries.*` (or top-level `sm.*`) and as a method on the type class. The method form is the primary user-facing surface; the flat form exists for symmetry with the rest of the queries family.

### Brep properties

| Method | Flat alias | Purpose |
|---|---|---|
| `brep.bounding_box(tight=False)` | `brep_bounding_box(brep, tight)` | Axis-aligned bounding box as `((min_x,min_y,min_z), (max_x,max_y,max_z))`. `tight=True` samples each face's surface for a minimal box (expensive); default returns the loose vertex/edge/face union. |
| `brep.volume(relative_accuracy=1e-3)` | `compute_volume(brep, relative_accuracy)` | Enclosed volume in cubic modeling units. Closed manifold solid required. Finite `relative_accuracy` is clamped to `[1e-8, 1e-1]`; non-finite values raise `RuntimeError`. |
| `brep.area(relative_accuracy=1e-3)` | — | Total surface area in square modeling units: the sum of `face.area()` over every face, with the same `relative_accuracy` clamp. Works on sheet bodies and open shells, which `mass_properties()` rejects. Non-finite `relative_accuracy` raises `RuntimeError`. Wraps `SmApiBrepComputeArea`. |
| `brep.mass_properties(relative_accuracy=1e-3, density=1.0, origin=None)` | — | Dict of mass properties for a closed manifold solid, computed directly from the exact BRep geometry by adaptive numerical integration to `relative_accuracy` (clamped to `[1e-4, 1e-1]`): `area`, `volume`, `mass` (`density * volume`), `centroid` (mass centre of gravity `(x, y, z)`, origin-independent), and **mass** `moments_of_inertia` `(Ixx, Iyy, Izz)` / `products_of_inertia` `(Iyz, Izx, Ixy)` taken about axes through `origin` (parallel to world axes). `origin=None` (default) returns **centroidal** inertia (recovers the centroid, then integrates about it — two passes); pass an explicit `(x,y,z)` for a single pass about that point. Products are the raw positive integrals `(∫yz dm, ∫zx dm, ∫xy dm)`; an inertia tensor's off-diagonals are their negatives. `density` must be `>= 1e-12`; a non-solid, an attributed Brep (`SM_AI_MASS_PROPERTIES`), or a non-finite `density` / `relative_accuracy` / `origin` raises `RuntimeError`. The C `SmApiBrepComputeMassProperties` takes the origin explicitly and runs a single `ComputePreciseProperties` pass; `SmApiBrepComputeVolume` shares that core. |
| `brep.is_manifold_solid()` | `is_manifold_solid(brep)` | `True` if every edge is shared by exactly two faces. |
| `brep.copy()` | `brep_copy(brep)` | Deep-copy into a fresh handle, preserving geometry types: NURBS are not promoted to analytics. Mutating the result does not affect the source — use this before in-place mutators (`fillet_edges`, `cut`, `translate`, …) when the original must be preserved. `copy.copy(brep)` and `copy.deepcopy(brep)` give the same deep copy (so does `PolyBrep`). |
| `brep.center()` | — | Centre of the loose bounding box as `(x, y, z)`. Convenience for a rough anchor; for the mass centre of gravity use `mass_properties()['centroid']`. |
| `brep.face_count()`, `.edge_count()`, `.vertex_count()` | — | Cheap counts; equivalent to `len(brep.faces())` etc. |
| `repr(brep)` | — | `<Brep faces=N edges=M vertices=K>`. |

### Face properties

`Face.area()` and `Brep.area()` use area-only precise numerical integration:
they do not compute volume or moment integrals. The area error control and
failure reporting are retained; this is not a tessellation approximation.
`Face.centroid()` requests area and area first moments in the same pass.
`mass_properties()` and `Brep.volume()` still use full-property integration.

| Method | Flat alias | Purpose |
|---|---|---|
| `face.bounding_box(tight=False)` | `face_bounding_box(face, tight)` | Axis-aligned bounding box of the face. `tight=True` samples within the trim boundary (expensive). |
| `face.area(relative_accuracy=1e-3)` | `face_area(face, relative_accuracy)` | Surface area within the trim boundary, in square modeling units. Always positive — a geometric, not signed, quantity. `relative_accuracy` clamped to `[1e-4, 1e-1]`. |

### Edge properties

| Method | Flat alias | Purpose |
|---|---|---|
| `edge.bounding_box(tight=False)` | `edge_bounding_box(edge, tight)` | Axis-aligned bounding box of the edge curve over its parametric interval. |
| `edge.length(desired_accuracy=1e-6)` | `edge_length(edge, desired_accuracy)` | 3D arc length over the edge's parametric interval, in modeling units. |

### Vertex properties

| Method | Flat alias | Purpose |
|---|---|---|
| `vertex.point()` | `vertex_point(vertex)` | 3D position as `(x, y, z)`. |

## Topology Accessors (walk the BRep graph)

The kernel's radial-edge data structure links every face, edge, and vertex to its neighbours. These methods expose those links directly so Python code can walk the graph in either direction without re-querying the parent Brep.

### Face

| Method | Returns | Purpose |
|---|---|---|
| `face.surface()` | `Surface` | Borrowed view of the underlying NURBS / analytic surface. Owned by the Brep — do not delete. Release before a representation-changing `turn_to_nurbs` or scale, then reacquire from the mutated face. |
| `face.uv_domain()` | `((u_min,v_min),(u_max,v_max))` | Trim domain on the parent surface (the rectangular UV box that bounds the outer loop). |
| `face.edges()` | `list[Edge]` | Every edge bounding this face (across all loops). |
| `face.vertices()` | `list[Vertex]` | Every vertex on this face's boundary (across all loops). |
| `face.outer_loop_edges()` | `list[Edge]` | Edges of the outer loop only. |
| `face.point_at_uv(u, v)` | `(x,y,z)` | 3D position on the surface at parameter `(u, v)`. Evaluation is allowed outside `uv_domain()` but the result may not lie on the trimmed face. |
| `face.normal_at_uv(u, v)` | `(nx,ny,nz)` | Unit *surface* normal at `(u, v)` — the cross product of the UV partials produced by `SmApiEvaluateSurfaceNormal`. **Does not** apply the face's orientation: for faces whose orientation is `SM_OT_OPPOSITE` relative to their surface this points *inward*. Negate the vector when a consistently outward face-oriented normal is required. |
| `face.outward_normal_at_uv(u, v)` | `(nx,ny,nz)` | Unit face-oriented normal at `(u, v)`, flipped by the same Brep infinite-region orientation test used by SMLib display faceting. |
| `face.interior_point()` | `(x,y,z)` | A 3D point evaluated on the owning surface at a UV strictly inside the trim boundary (interior to the outer loop and outside any holes). If no such point can be evaluated, the call raises `RuntimeError`. Seed point for ray tests / classification / picking. Wraps `SmApiFaceInternalPoint`. |
| `face.classify_uv(u, v)` | `str` | Trim-aware classification of a UV parameter: `"inside"`, `"outside"` (outside the outer loop **or** inside a hole), `"boundary"` (on an edge), or `"vertex"`. `(u, v)` is a parameter on the parent `surface()`. Wraps `SmApiFaceClassifyUV`. |
| `face.closest_point(point)` | `((x,y,z), dist)` | Closest point on the *trimmed* face to `point`. Unlike a plain surface closest point, always lies within (or on) the trim boundary — scopes an `SmShape` to this one face and runs `SmTopologySolver::ShapePointSolve`. Wraps `SmApiFaceClosestPoint`. |
| `face.centroid(relative_accuracy=1e-3)` | `(x,y,z)` | Area centroid of the trimmed face. Shares the `SmFace::ComputePreciseProperties` integration `area()` already runs. `relative_accuracy` clamped to `[1e-4, 1e-1]`, same as `area()`. Wraps `SmApiFaceComputeCentroid`. |
| `face.brep()` | `Brep` | Parent Brep. |

### Edge

| Method | Returns | Purpose |
|---|---|---|
| `edge.curve()` | `Curve` | Borrowed view of the underlying 3D curve. The edge uses a sub-interval of this curve's full parameter range. Release before a representation-changing `turn_to_nurbs` or scale, then reacquire from the mutated edge. |
| `edge.start_vertex()` / `end_vertex()` | `Vertex` | Endpoints (low / high parameter end). |
| `edge.vertices()` | `(Vertex, Vertex)` | Both endpoints in one call. |
| `edge.other_vertex(v)` | `Vertex \| None` | Whichever endpoint is *not* `v`. Returns `None` if `v` is not an endpoint. Useful for walking an edge string: `next = edge.other_vertex(prev)`. |
| `edge.faces()` | `list[Face]` | All faces sharing this edge. Manifold edge in a closed solid → exactly two; lamina edge → one; non-manifold → three or more. |
| `edge.point_at(t)` | `(x,y,z)` | 3D position on the edge's curve at parameter `t`. |
| `edge.closest_point(point)` | `((x,y,z), t, dist)` | Closest point on this edge to `point`, clamped to `parameter_range()` (never on a part of the curve outside the edge's trim interval). Wraps `SmApiEdgeClosestPoint`. |
| `edge.parameter_range()` | `(t_min, t_max)` | This edge's own parametric interval on its curve — distinct from `curve().parameter_range()`, the curve's full natural range. Wraps `SmApiEdgeParameterRange` / `SmEdge::GetInterval()`. |
| `edge.classify_parameter(t)` | `str` | Classify `t` against `parameter_range()`: `"inside"`, `"endpoint"` (within tolerance of either end), or `"outside"`. Wraps `SmApiEdgeClassifyParameter`. |
| `edge.tangent_at(t)` | `(x,y,z)` | Unit tangent vector of the edge's curve at parameter `t`. Wraps `SmApiEdgeTangent`. |
| `edge.brep()` | `Brep` | Parent Brep. |

### Vertex

| Method | Returns | Purpose |
|---|---|---|
| `vertex.edges()` | `list[Edge]` | All edges incident on this vertex. Empty for an isolated shell vertex. |
| `vertex.faces()` | `list[Face]` | All faces incident on this vertex (via its edges). |
| `vertex.brep()` | `Brep` | Parent Brep. |

For oriented loop/loopuse/edgeuse traversal and viewport topology picking, use `_smlib_dev` — see **[dev_topology](dev_topology.md)**.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)

# Whole-Brep properties.
(mn, mx) = box.bounding_box()                # ((0,0,0), (10,10,10))
c        = box.center()                      # (5.0, 5.0, 5.0)
v        = box.volume()                      # 1000.0
ok       = box.is_manifold_solid()           # True
print(f"{box.face_count()} faces, {box.edge_count()} edges, {box.vertex_count()} vertices")

# Per-face: geometry, parameter domain, topology, eval.
f0 = box.faces()[0]
area     = f0.area()                         # ~100.0
srf      = f0.surface()                      # NURBS surface backing this face
(u0,v0),(u1,v1) = f0.uv_domain()
mid      = f0.point_at_uv(0.5*(u0+u1), 0.5*(v0+v1))
n        = f0.normal_at_uv(0.5*(u0+u1), 0.5*(v0+v1))
edges    = f0.outer_loop_edges()             # 4 edges of the outer trim

# Per-edge: walk to neighbours.
e0 = edges[0]
v_start, v_end = e0.vertices()
faces_sharing  = e0.faces()                  # 2 for a manifold edge
crv            = e0.curve()
L              = e0.length()                 # ~10.0

# Walk an edge string by chaining other_vertex.
next_v = e0.other_vertex(v_start)            # == v_end

# Per-vertex: position and incidence.
v0_pt  = v_start.point()                     # (x, y, z)
v0_e   = v_start.edges()                     # 3 incident edges per box corner
v0_f   = v_start.faces()                     # 3 incident faces per box corner

# Round-trip back to the parent Brep.
assert e0.brep() is box

# Copy-then-mutate pattern, since most Brep ops mutate in place:
trimmed = box.copy()
sm.cut(trimmed, (0, 0, 5), (0, 0, 1))        # trimmed truncated; box unchanged
```

**When to prefer the method form vs. flat form**

- Use the method form (`brep.bounding_box()`) for routine code — it reads naturally and self-documents the receiver type.
- Use the flat form (`sm.queries.brep_bounding_box(b)`) when you're writing higher-order code that takes the function as a value, or when matching the existing flat style of older bindings.

## Surface and Curve Properties

Standalone `Surface` and `Curve` handles (the kind returned by `sm.create_surface_*`, `sm.create_*_curve` / `create_circle` / `create_line_segment` / …, `face.surface()`, `edge.curve()`, …) carry their own parametric-domain queries. None of these have flat aliases — they are method-only because the receiver is always one of these geometric types and the symmetric `Brep` queries already cover the topological side. The exception is the position+derivative evaluators, which are also exposed as the flat `sm.evaluate_surface_*` / `sm.evaluate_curve` functions.

### Curve methods

| Method | Purpose |
|---|---|
| `curve.parameter_range()` | Natural parameter range as `(t_min, t_max)`. `evaluate(t)` is well-defined for `t` in this interval. |
| `curve.start_point()` | 3D position at `t_min`. |
| `curve.end_point()` | 3D position at `t_max`. |
| `curve.evaluate(t)` | 3D position at parameter `t` as `(x, y, z)`. For first/second derivatives use the flat `sm.evaluate_curve(curve, t)` (returns `{point, tangent, curvature}`). |
| `curve.length(desired_accuracy=1e-6)` | 3D arc length over the natural parameter range. |
| `curve.is_closed(tolerance=0.0)` | `True` if `start_point()` and `end_point()` coincide within `tolerance`. |
| `curve.is_planar(tolerance=SM_EFF_ZERO)` | `True` if the curve lies within `tolerance` of a best-fit plane. Lines and circles are trivially planar. |
| `repr(curve)` | `<Curve t=[t_min, t_max]>`. |

### Surface methods

| Method | Purpose |
|---|---|
| `surface.u_domain()` | Natural U range as `(u_min, u_max)`. |
| `surface.v_domain()` | Natural V range as `(v_min, v_max)`. |
| `surface.uv_domain()` | Combined `((u_min, v_min), (u_max, v_max))`. The natural domain is the surface's intrinsic parametric extent; on a Brep face this may be wider than the face's trim domain (see `Face.uv_domain`). |
| `surface.evaluate(u, v)` | 3D position at `(u, v)` as `(x, y, z)`. Wraps `SmApiEvaluateSurfacePoint`. |
| `surface.normal(u, v)` | Unit surface normal as `(nx, ny, nz)`, computed as `du × dv` and normalized. **Sign follows the surface's intrinsic orientation; on a `Face` with reversed orientation (`SM_OT_OPPOSITE`) the value may point inward and the caller must negate it for an outward face normal.** Wraps `SmApiEvaluateSurfaceNormal`. |
| `surface.derivatives(u, v)` | Position and first U/V partials at `(u, v)`. Returns `(point, du, dv)` as a tuple of three 3-tuples. Wraps `SmApiEvaluateSurfaceDerivatives`. |
| `surface.is_planar(tolerance=SM_EFF_ZERO)` | `True` if the surface lies within `tolerance` of a best-fit plane. Plane primitives are trivially planar. |
| `surface.is_periodic_u()` | `True` if the U direction wraps (e.g. longitude on a sphere/cylinder). |
| `surface.is_periodic_v()` | `True` if the V direction wraps. |
| `repr(surface)` | `<Surface u=[u_min, u_max] v=[v_min, v_max]>`. |

```python
line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
t_min, t_max = line.parameter_range()
mid = line.evaluate(0.5 * (t_min + t_max))   # (5.0, 0.0, 0.0)
print(line.length())                         # 10.0
print(line.is_closed(), line.is_planar())    # False True

circ = sm.create_circle((0, 0, 0), 5)
print(circ.length())                         # ~31.4159
print(circ.is_closed())                      # True

srf = sm.create_surface_from_points(pts, 4, 4)
(umin, vmin), (umax, vmax) = srf.uv_domain()
u, v = 0.5 * (umin + umax), 0.5 * (vmin + vmax)
print(srf.evaluate(u, v))                    # (x, y, z)
print(srf.normal(u, v))                      # unit (nx, ny, nz)
pt, du, dv = srf.derivatives(u, v)
print(srf.is_planar(1e-3))                   # True for a flat patch

# Walking from a Brep face out to the underlying surface and querying it.
box  = sm.create_box((0, 0, 0), 10, 10, 10)
face = box.faces()[0]
srf  = face.surface()                        # Surface, owned by the Brep
print(srf.is_planar(1e-3))                   # True (box face)
```

**Relationship to the flat `evaluate_*` family.** `Surface.evaluate` / `.normal` / `.derivatives` are thin wrappers over the same `SmApiEvaluateSurface*` entry points used by `sm.evaluate_surface_point`, `sm.evaluate_surface_normal`, and `sm.evaluate_surface_derivatives`. `Curve.evaluate` returns position only; for first/second derivatives use the flat `sm.evaluate_curve` (which returns a `{point, tangent, curvature}` dict). Use whichever reads better at the call site — the methods are preferred for routine code.

## Polygon-Mesh Counterparts

For tessellated `PolyBrep` meshes the equivalent queries are bound under different names and live in **[polybrep](polybrep.md)**:

- **`poly_brep_is_manifold_via_api(mesh)`** — manifold check on a polygon BRep (wraps `SmApiPolyBrepIsManifoldSolid`).
- **`poly_brep_volume_via_api(mesh)`** — enclosed volume of a polygon BRep (wraps `SmApiPolyBrepComputeVolume`).

## Common Validation Patterns

### Check After Boolean
```python
result = sm.boolean_union(box, sph)
if not sm.is_manifold_solid(result):
    print("Warning: boolean result is not a valid solid")
```

### Measure Before/After Offset
```python
before = sm.compute_volume(brep)
offset = sm.offset_brep(brep, 1.0)
after = sm.compute_volume(offset)
print(f"Volume increased by {after - before:.2f}")
```

### Distance Check
```python
point = (15, 15, 15)
closest, dist = sm.brep_closest_point(brep, point)
if dist < tolerance:
    print("Point is on or near the surface")
```
