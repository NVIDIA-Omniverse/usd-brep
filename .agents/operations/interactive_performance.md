<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

<a name="preview-vs-exact-api-selection--cache-reuse"></a>

# Preview Versus Exact: API Selection and Cache Reuse

**Source:** `SmApiQueries.h`, `SmApiBrep.h`, `SmApiGeneral.h`
**Python:** `brep.mass_properties`, `brep.volume`, `brep.bounding_box`, `tessellate`, `translate` / `rotate` / `transform`, `boolean_difference`

**Scope:** choosing a cheap preview path vs. an exact final path, and reusing
cached results, for an interactive edit loop. It does **not** cover the kernel
cost of tessellation itself — usually the dominant cost in an interactive
workload — beyond the quality knobs below; see **[tessellation](tessellation.md)**
and the [Known gaps](#known-gaps-feature-requests-not-missing-docs) section for
that. Context lifetime, caching, and concurrency are out of scope here too.

Work backward from product intent to the right API + quality knob. Most "why is
this slow?" cases are a final-quality path called on every preview, or a
whole-solid result accumulated from per-entity calls. Pick your intent, then
read the note.

| Intent | Use | Not |
|---|---|---|
| Final whole-solid volume / area / centroid / inertia | one `brep.mass_properties()` (fine accuracy) | a per-face loop (can't yield solid volume) |
| Volume only | `brep.volume()` (one integration pass) | `brep.mass_properties()` with default `origin` (two passes) |
| Cheap preview volume | PolyBrep mesh volume (already have it) | exact integration every preview |
| Faster exact preview property | coarse `relative_accuracy` (~`1e-1`) | the fine default every preview |
| Preview bounds | `brep.bounding_box()` (loose) | `bounding_box(tight=True)` |
| Instanced / patterned copies | reuse cached mesh + properties under the rigid transform | re-tessellate / re-integrate per placement |
| Model-relative mesh density | negative `max_edge_length` | one global absolute chord tolerance |
| Repeated solid cutters | sequential `boolean_difference` loop | `boolean_lists` (see note) |

## Properties: cut cost with accuracy and the mesh, not an API swap

The property cost is per-face numerical integration. Consolidating per-face
calls into one `brep.mass_properties()` does **not** reduce it. Prefer
`mass_properties()` for correctness/simplicity — it returns area, volume,
centroid, and inertia consistently in one call, and a `face.area()` loop cannot
yield solid volume — and reserve fine accuracy for final output.

With the default `origin=None`, Python `mass_properties()` integrates **twice**
(once about the bounding-box midpoint to find the centroid, then again about
that centroid so inertia is centroidal). If you only need volume, call
`brep.volume()` (one pass); if you need the rest but not centroidal inertia,
pass an explicit `origin` to get a single pass. The levers that actually cut
interactive-preview cost:

- **Coarse `relative_accuracy`** — `1e-1` is much cheaper than the `1e-3` default.
- **Mesh-derived volume/area** — read it off the `PolyBrep` you already
  tessellated for rendering (~100× cheaper than exact integration, ~0.1% error
  in the measured case). Valid only for a **closed, manifold** mesh; check first
  (open or partial tessellations give a meaningless volume).
- **Instance reuse** — see below; a pattern pays for exact properties once.
- **Defer** exact properties until final output.

## Tessellation quality: preview vs final

`tessellate` rebuilds the whole `PolyBrep` each call (no incremental path — see
Known gaps). Knobs:

- `chord_height_tolerance` — **absolute world units**; scale it yourself by
  model size for scale-relative quality.
- `max_edge_length` — an edge-length subdivision target, not a hard cap on final
  triangle edges (they can come out somewhat longer). A **negative** value sets
  the target to `0.025 * |value| * bbox-diagonal`, so density tracks model size;
  `0` disables.
- `curve_angle_tolerance_deg` / `surface_angle_tolerance_deg`, `max_aspect_ratio`.

A global chord tolerance safe for large models is often far too fine for models
with many small curved faces — make preview quality depend on model scale.

## Instancing: reuse under rigid transforms

`translate` and `rotate` are always rigid (and `transform` when its placement is
orthonormal — a `SetFrom4x4` placement can encode scale/shear and is **not**
rigid). Rigid motions preserve geometry pointers and topology, so compute once
and reposition per placement: transform a cached mesh's points/normals by the
same matrix instead of re-tessellating, and reuse mass properties
(area/volume/mass and inertia magnitudes are invariant; the centroid moves, and
a *centroidal* inertia tensor rotates — a tensor about a fixed origin also needs
the parallel-axis term). A representation-changing scale is different — see
**[transforms](transforms.md)**.

## Repeated solid cutters

Measure the surrounding queries separately from the Boolean itself. A tight
bounding-box query after every cut can dominate caller-side latency without
changing the solid. Omit unneeded queries, or use ordinary bounds when a loose
fit suffices. The [native Boolean/bounds example](../../Examples/SM_API/README.md)
demonstrates all three policies with correctness checks and separate timers.

A sequential `boolean_difference` (or `boolean_union`) loop is the simplest way
to apply repeated cutters. For non-manifold operands it switches to the
non-manifold entry point, which runs the same boolean engine with non-manifold
handling enabled — not a slower algorithm. The batch `boolean_lists` first copies
all operands and stitches them into combined Breps, so for manifold solids it is
**not** faster than the loop (it measured slower); use it for mixed sheet/solid
lists, not as a speed optimization. `evaluate_csg_tree` runs the same boolean
engine at every node, with no copy/stitch and **no** non-manifold fallback — it
is a convenience for expressing a mixed-operation tree, not a speedup, and it is
not a route for sheet operands. See **[booleans](booleans.md)**.

## Known gaps (feature requests, not missing docs)

- **Incremental / cached re-tessellation** — no "re-tessellate only changed
  faces" path.
- **Cancellation / progress** — long operations can't be cancelled or report progress.
- **Batched projection / closest-point** — per-query only.
- **Cross-edit topology identity** — `faces()` / `edges()` order is stable per
  Brep instance but not preserved across topology edits.

## See Also

- **[queries](queries.md)** · **[tessellation](tessellation.md)** · **[transforms](transforms.md)** · **[booleans](booleans.md)**
