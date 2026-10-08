<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Fillet and Chamfer Operations

**Source:** `source/SM_API/inc/SmApiFillets.h`, `source/SM_API/inc/SmApiBrep.h`
**Python:** `_omni_solid.circular_fillet`, `chamfer_fillet`, `fillet_edges`, `fillet_edges_per_edge`, `variable_radius_fillet`, `remove_fillet`, `surface_surface_fillet`, `fillet_preview`, `set_bevel_corners`

## Overview

Fillets and chamfers smooth or bevel the sharp edges of a solid. They modify the BRep in-place by replacing edges with new surface patches. The choice of fillet type affects both the appearance and functional properties of the resulting shape.

- **Circular fillet** — replaces an edge with a smooth circular arc cross-section (constant radius blend)
- **Chamfer** — replaces an edge with a flat cut (linear cross-section)
- **Blend curve** — higher-continuity smooth transition (G2/G3)

Fillets can be applied to all edges at once (simple API) or selectively to specific edges with per-edge control.

## Operations

### circular_fillet
Apply a circular fillet to ALL edges of a BRep.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to fillet (modified in-place) |
| `radius` | float | Fillet radius |

**Design notes:**
- Applies to every edge in the BRep. For selective filleting, use `fillet_edges`.
- The radius must be smaller than the minimum face dimension adjacent to each edge. If the radius is too large, the fillet computation will fail.
- Modifies the BRep in-place — new faces are created for each fillet surface.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sm.circular_fillet(box, radius=1.0)
# box now has >6 faces due to fillet surfaces
```

### chamfer_fillet
Apply a chamfer (flat bevel) to ALL edges of a BRep.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to chamfer (modified in-place) |
| `radius` | float | Chamfer width: the straight-line distance across the chamfer face |

**Design notes:**
- Same interface as `circular_fillet` but produces flat cuts instead of rounded fillets.
- `radius` is the chamfer width, not the setback along each face. On a 90° edge each face is cut back by `radius / √2` (0.707 for `radius=1.0`).

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sm.chamfer_fillet(box, radius=1.0)
```

### fillet_edges
Apply a fillet to selected edges with control over cross-section type and continuity.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | BRep to fillet (modified in-place) |
| `edges` | list[Edge] | — | Specific edges to fillet |
| `radius` | float | — | Fillet radius. **Keyword-only.** |
| `xsect_type` | int | `1` | Cross-section: 0=linear, 1=circular, 2=blend curve. **Keyword-only.** |
| `continuity` | int | `1` | For blend type: 1=G1, 2=G2, 3=G3. **Keyword-only.** |
| `thumbweight` | float | `1.0` | Controls blend shape (1.0 = default). **Keyword-only.** |

**Design notes:**
- Get edges from `brep.edges()`. You can select a subset.
- Cross-section types:
  - `0` (LINEAR) = linear cross-section with the selected radius solver; this does not by itself select the constant-distance chamfer solver used by `chamfer_fillet`
  - `1` (CIRCULAR) = circular fillet (standard smooth round)
  - `2` (BLEND) = blend curve with controllable continuity (G1/G2/G3)
- G1 = tangent continuity (smooth but may have curvature discontinuity). G2 = curvature continuity (smoothest common choice). G3 = third-order continuity (for high-quality Class-A surfaces).
- `thumbweight` controls the fullness of the blend. Values > 1.0 make a fuller blend, < 1.0 make it thinner. Only applies to blend type (xsect_type=2).

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
top_edges = box.edges()[:4]  # first 4 edges
sm.fillet_edges(box, top_edges, radius=2.0, xsect_type=1, continuity=1)
```

### variable_radius_fillet
Apply a fillet with varying radius from start to end of each edge.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | BRep to fillet (modified in-place) |
| `edges` | list[Edge] | — | Edges to fillet |
| `start_radius` | float | — | Radius at the start of the edge. **Keyword-only.** |
| `end_radius` | float | — | Radius at the end of the edge. **Keyword-only.** |
| `xsect_type` | int | `1` | 0=linear, 1=circular, 2=blend. **Keyword-only.** |
| `continuity` | int | `1` | For blend type: 1=G1, 2=G2, 3=G3. **Keyword-only.** |

**Design notes:**
- The radius varies linearly from `start_radius` to `end_radius` along each selected edge.
- Useful for aesthetically shaped transitions or functional requirements where a fillet needs to taper.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
edges = box.edges()[:1]
sm.variable_radius_fillet(box, edges, start_radius=0.5, end_radius=2.0)
```

## Choosing Fillet Parameters

### Radius Selection
- **Rule of thumb:** radius should be ≤ 1/3 of the smallest adjacent face dimension. Larger radii risk overlap between adjacent fillets.
- For a 10×10×10 box, `radius=3.0` is a reasonable maximum. At `radius=5.0`, adjacent fillets will collide.
- Start small and increase. It's easier to recover from a small fillet than debug a failed large one.

### Cross-Section Type
| Type | When to Use |
|---|---|
| Linear (0) | Flat cross-section using the selected fillet solver; use `chamfer_fillet` for the all-edge constant-distance chamfer operation |
| Circular (1) | Default choice — smooth, predictable, matches physical rounding |
| Blend (2) | High-quality surfaces, automotive/industrial design, Class-A surfaces |

### Continuity
| Level | Name | Visual | Use Case |
|---|---|---|---|
| G1 | Tangent | No visible edge, but reflection lines break | General mechanical parts |
| G2 | Curvature | Smooth reflections | Consumer products, visible surfaces |
| G3 | Third-order | Perfectly smooth highlight lines | Automotive Class-A surfaces |

**Three-edge corner patches guarantee only G1.** At a vertex where three filleted
edges meet, `continuity=2` or `3` still applies to blend cross-sections along the
selected edges, but G2/G3 continuity is not guaranteed across the corner-patch
boundary.

## Additional Fillet Functions (now in Python)

- **`fillet_edges_per_edge(brep, edges, radii, xsect_types, *, continuity=1, thumbweight=1.0)`** — per-edge radius values and cross-section types
- **`surface_surface_fillet(surface1, surface2, *, radius1, radius2, tolerance, xsect_type=1, mirror=False, complement=False)`** — fillet between two surfaces
- **`fillet_preview(brep, edges, *, radius)`** — preview fillet surfaces without modifying the BRep
- **`remove_fillet(brep)`** — remove fillets from a BRep (defeaturing)
- **`set_bevel_corners(brep, *, edges, vertices, radius)`** — fillet `edges` with a constant radius in place, beveling the named corner `vertices` instead of rounding them. It performs the fillet itself. Each vertex must be a corner where filleted edges meet; otherwise it raises (`SM_ERR_NULL_POINTER`). Takes `Edge` and `Vertex` handles.
