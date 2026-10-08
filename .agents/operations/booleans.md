<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Boolean Operations

**Source:** `source/SM_API/inc/SmApiBrep.h`
**Python:** `_omni_solid.boolean`, `boolean_union`, `boolean_difference`, `boolean_intersection`, `boolean_merge`, `merge_breps`, `boolean_with_options`, `boolean_with_curves`, `boolean_2d`, `boolean_lists`, `evaluate_csg_tree`, `non_manifold_boolean`, `piecewise_merge`

## Overview

Boolean operations combine two or more solid bodies by computing their surface intersections, classifying regions as inside/outside, and building a new BRep from the retained faces. SMLib uses exact NURBS surface-surface intersection, so results maintain exact geometry — no approximation is introduced.

The four boolean operations work on the principle of set theory:
- **Union** — material from A *or* B (additive, like welding)
- **Difference** — material from A that is *not* in B (subtractive, like drilling)
- **Intersection** — material in both A *and* B (keeps only the overlap)
- **Merge** — combines topology without removing any faces (for non-manifold assembly)

## Operations

### boolean / boolean_union / boolean_difference / boolean_intersection
Perform a boolean on two BReps. The convenience variants (`boolean_union`, etc.) are equivalent to `boolean(a, b, operation=op)`.

| Parameter | Type | Description |
|---|---|---|
| `brep1` | Brep | Primary body (A). For difference, this is the body being cut. |
| `brep2` | Brep | Tool body (B). For difference, this is the cutting shape. |
| `operation` | BooleanOp | `UNION`, `DIFFERENCE`, `INTERSECTION`, or `MERGE`. **Keyword-only** — must be passed as `operation=...` so the enum can never be misordered relative to the operands. |

**Returns:** the resulting `Brep`, which can reuse the modified primary operand. Input ownership transfers to the operation; use only the result afterward, or pass copies to preserve the originals.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sph = sm.create_sphere((5, 5, 5), 4)

union   = sm.boolean_union(box.copy(), sph.copy())         # box + sphere
diff    = sm.boolean_difference(box.copy(), sph.copy())    # box with sphere carved out
inter   = sm.boolean_intersection(box.copy(), sph.copy())  # only the overlapping region
generic = sm.boolean(box.copy(), sph.copy(), operation=sm.BooleanOp.UNION)
```

### merge_breps
Boolean-combine an array of BReps in one call.

| Parameter | Type | Description |
|---|---|---|
| `breps` | list[Brep] | Two or more unique, non-null bodies to combine |
| `operation` | BooleanOp | Boolean operation type. **Keyword-only.** |

```python
parts = [
    sm.create_box((0, 0, 0), 10, 10, 10),
    sm.create_box((5, 0, 0), 10, 10, 10),
    sm.create_cylinder((15, 5, 0), 3, 10),
]
result = sm.merge_breps(parts, operation=sm.BooleanOp.UNION)
```

Operands retain the kernel's longstanding last-to-first evaluation order, so
`DIFFERENCE` on `[a, b, c]` computes `(c - b) - a`. Lists with fewer than two
entries, null entries, and duplicate handles are rejected without consuming any
input.

## Choosing the Right Operation

| Goal | Operation | Example |
|---|---|---|
| Add material | `UNION` | Weld two parts together |
| Cut a hole | `DIFFERENCE` | Drill hole: `boolean_difference(block, cylinder)` |
| Find overlap | `INTERSECTION` | Interference check between parts |
| Keep everything | `MERGE` | Assembly where parts touch but don't penetrate |
| Combine many parts | `merge_breps` | Union of N bodies at once |

## Design Notes

### Separate Boolean cost from optional bounds queries

A caller-side bounding-box query is not required before or after each Boolean.
If bounds are needed for framing or broad-phase culling, start with
`brep.bounding_box(tight=False)` / `SmApiBrepBoundingBox(..., FALSE, ...)`.
Use tight bounds only when the tighter fit is needed: they can be substantially
more expensive and do not improve the constructed solid. Ordinary and tight
boxes are not guaranteed to coincide.

The [native Boolean/bounds example](../../Examples/SM_API/README.md) times the
calls separately and checks a perforated plate's expected volume, bounds, and
manifold status. Its correctness checks run in `SM_API_test`; timing ratios
are not test requirements. See also the
[native API usage skill](../skills/smlib-sm-api-use/SKILL.md).

### Input Requirements
- Both BReps should be valid manifold solids for best results. Sheet bodies (open shells) may work for some operations but can produce unexpected topology.
- The two bodies should overlap or at least touch. If they're disjoint, union produces a multi-shell BRep, and intersection produces nothing.
- Python input ownership transfers to the operation; use only the returned handle afterward. The result may reuse the modified primary operand. Native callers must follow the selected `SmApi*` entry point's ownership contract.

### Difference Order Matters
`boolean_difference(A, B)` removes B from A. Swapping A and B gives a completely different result. Think of B as the "tool" that cuts into the "workpiece" A.

### Robustness Tips
- Avoid configurations where surfaces are tangent or coincident — these are topologically degenerate and harder for the intersector. Offset one body by a tiny amount if needed.
- Very thin features near the intersection boundary can cause issues. If a feature is thinner than the modeling tolerance, the boolean may fail or produce unexpected topology.
- For complex multi-body operations, build up incrementally and check intermediate results with `is_manifold_solid()`.

## Advanced Boolean Options

### boolean_with_options
Provides fine control over boolean behavior.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep1` | Brep | | Primary body (A) |
| `brep2` | Brep | | Tool body (B) |
| `operation` | BooleanOp | | Boolean operation type. **Keyword-only** (along with all flags below). |
| `cookie_cutter` | bool | False | Only remove faces from BrepA (BrepB acts as a cookie cutter). Keyword-only. |
| `imprinting` | bool | False | Imprint intersection curves on BrepA without removing material. Keyword-only. |
| `post_process` | bool | True | Clean up redundant topological edges/vertices after boolean. Keyword-only. |
| `imprint_and_classify` | bool | False | Imprint and mark faces for deletion without actually deleting. Keyword-only. |
| `keep_other_brep` | bool | False | Don't consume BrepB (allows reuse as a tool). Keyword-only. |

```python
result = sm.boolean_with_options(box, cutter, operation=sm.BooleanOp.DIFFERENCE,
                                cookie_cutter=True, post_process=True)
```

### boolean_2d
2D boolean operations for planar BReps.

| Parameter | Type | Description |
|---|---|---|
| `brep1` | Brep | Primary planar body |
| `brep2` | Brep | Second planar body |
| `operation` | BooleanOp2D | `UNION`, `INTERSECTION`, `DIFFERENCE`, `EXCLUSIVE_OR`, or `MERGE`. **Keyword-only.** |

```python
circle_a = sm.create_planar_circle((0, 0, 0), 10)
circle_b = sm.create_planar_circle((5, 0, 0), 10)
result = sm.boolean_2d(circle_a, circle_b, operation=sm.BooleanOp2D.UNION)
```

### boolean_lists
Combine lists of BReps and surfaces in one operation. Returns a tuple of `(result_breps, result_surfaces)`.

| Parameter | Type | Description |
|---|---|---|
| `breps1` | list[Brep] | First list of breps |
| `surfaces1` | list[Surface] | First list of surfaces (for planar 2D) |
| `breps2` | list[Brep] | Second list of breps |
| `surfaces2` | list[Surface] | Second list of surfaces (for planar 2D) |
| `operation` | int | 0=AND, 1=OR, 2=XOR, 3=A-B, 4=B-A. **Keyword-only.** |

```python
breps, surfaces = sm.boolean_lists([box1], [], [box2], [], operation=1)  # OR/union
```

### evaluate_csg_tree
Express complex CSG operations as a postfix tree.

| Parameter | Type | Description |
|---|---|---|
| `breps` | list[Brep] | Array of input breps. **Mutated in place** — the result Brep ends up at `breps[0]` and the returned list is just a copy of the rewritten array. |
| `postfix_tree` | list[int] | CSG tree in postfix notation. Operand entries are non-negative indices into `breps`; operator entries encode `BooleanOp` values. |

```python
result_breps = sm.evaluate_csg_tree([box, sphere, cylinder], postfix_ops)
result = result_breps[0]
```
