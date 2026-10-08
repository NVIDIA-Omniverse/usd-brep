<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Stitching Operations

**Source:** `source/SM_API/inc/SmApiBrep.h`
**Python:** `_omni_solid.stitch_into_solid`, `stitch_into_shell`, `unify_normals`, `stitch_advanced`, `stitch_brep`, `stitch_brep_with_face`, `simple_face_stitch`

## Overview

Stitching joins separate faces in a BRep by identifying and gluing edges that are geometrically close. It converts a collection of loose faces (imported from CAD, for instance) into a topologically connected solid or shell.

This is essential when importing geometry from file formats (STEP, IGES) where faces arrive as disconnected entities. Stitching recovers the topological connectivity.

## Operations

### stitch_into_solid
Attempt to stitch all lamina (unconnected) edges in a BRep to produce a manifold solid.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to stitch (modified in-place) |

**Returns:** tuple of `(is_solid, stitched_edges, max_vertex_gap, max_edge_gap)`
- `is_solid` — True if the result is a manifold solid (all edges shared by exactly 2 faces)
- `stitched_edges` — number of edge pairs that were glued
- `max_vertex_gap` — largest vertex-to-vertex distance that was bridged
- `max_edge_gap` — largest edge-to-edge distance that was bridged

**Design notes:**
- Stitching uses the kernel's default tolerance to determine which edges are "close enough" to be glued.
- If the result is not solid (`is_solid=False`), some edges remain unstitched. Check the gap values — they tell you how far apart the unstitched edges are.
- Common reasons for failure: gaps too large, overlapping faces, non-manifold topology (3+ faces sharing an edge).

```python
# After importing geometry with separate faces:
brep = # ... imported BRep with loose faces
is_solid, stitched, vert_gap, edge_gap = sm.stitch_into_solid(brep)
if is_solid:
    print(f"Successfully stitched {stitched} edges into solid")
else:
    print(f"Not solid. Max gaps: vertex={vert_gap:.6f}, edge={edge_gap:.6f}")
```

### stitch_into_shell
Stitch edges up to a maximum gap distance. Less restrictive than `stitch_into_solid` — the result need not be a solid.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | BRep to stitch (modified in-place) |
| `max_stitching_ratio` | float | `1.0` | Largest gap to close, as an absolute distance in model units (not a ratio) |
| `shell_is_well_formed` | bool (keyword-only) | `False` | Stitching mode: `True` only for a shell known to be well formed; `False` glues edges/vertices and ignores other problems |

**Returns:** tuple of `(brep, shell_is_well_formed, stitched_edges, max_vertex_gap, max_edge_gap)`
- `shell_is_well_formed` — echoes the argument; it is not computed from the result (use `is_manifold_solid` to check the result)

**Design notes:**
- `max_stitching_ratio` is an absolute distance: stitching starts at 1/100 of it and widens in steps. Increase it if `stitch_into_solid` fails but you know the geometry should be connected.
- Use this when the geometry isn't expected to be a solid (e.g., a surface patch collection) or when edge gaps are larger than the default tolerance.

```python
brep, _, stitched, vert_gap, edge_gap = sm.stitch_into_shell(brep, max_stitching_ratio=2.0)
```

### unify_normals
Orient all face normals consistently within a BRep, using a reference face as the starting orientation.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | BRep to orient |
| `start_face` | Face | — | Reference face (its normal direction is kept) |
| `num_samples` | int | `100` | Number of ray samples for inside/outside classification |

**Returns:** list of faces whose normals were flipped.

**Design notes:**
- After stitching, face normals may be inconsistent (some pointing inward, some outward). This function propagates normal orientation from the `start_face` to all connected faces.
- `num_samples` controls the accuracy of inside/outside classification. Higher = more reliable but slower. 100 is usually sufficient.
- Choose a `start_face` whose normal direction you're confident about (e.g., a face you know should point outward).

```python
faces = brep.faces()
flipped = sm.unify_normals(brep, start_face=faces[0], num_samples=100)
print(f"Flipped {len(flipped)} faces to unify normals")
```

## Typical Stitching Workflow

1. **Import geometry** — faces arrive as disconnected entities
2. **Stitch** — `stitch_into_solid(brep)` to join edges
3. **Check result** — if `is_solid` is False, try `stitch_into_shell` with increased tolerance
4. **Unify normals** — `unify_normals(brep, faces[0])` to ensure consistent orientation
5. **Validate** — `is_manifold_solid(brep)` should return True for a valid solid

## Advanced Stitching (now in Python)

### stitch_advanced
Full control over stitching behavior:

```python
result = sm.stitch_advanced(brep, tol_3d=0.01,
    squeeze_small_edges=True, split_edges_with_vertices=True,
    making_manifold_solid=True, remove_laminar_slivers=True)
# Returns (stitched_edges_count, lamina_edges_count, gap_stats)
```

| Parameter | Description |
|---|---|
| `tol_3d` | 3D tolerance for stitching (explicit distance) |
| `squeeze_small_edges` | Remove edges shorter than tolerance |
| `split_edges_with_vertices` | Split edges at nearby vertices for better matching |
| `making_manifold_solid` | Only glue lamina edge pairs (skip non-manifold) |
| `remove_laminar_slivers` | Remove sliver faces before stitching |

- **`stitch_brep(brep)`** — one-call whole-Brep stitch with default tolerances and no diagnostic return values (wraps `SmApiStitch`).
- **`stitch_brep_with_face(brep, face)`** — stitch using a single face's lamina edges as the seed (wraps the `SmApiStitch` Brep+Face overload).
- **`simple_face_stitch(brep, faces_to_keep, faces_to_delete)`** — glue paired faces (delete onto keep) and report `(stitched_edges, max_vertex_gap, max_edge_gap)` (wraps `SmApiSimpleFaceStitch`). Requires `Face` handles obtained from topology traversal.
