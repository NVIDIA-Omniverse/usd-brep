<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD BrepArray Schema Contract

## Contents

1. [Authority and notation](#authority-and-notation)
2. [Schema inventory](#schema-inventory)
3. [Packed topology contract](#packed-topology-contract)
4. [Geometry packing](#geometry-packing)
5. [NURBS contract](#nurbs-contract)
6. [Analytic geometry contract](#analytic-geometry-contract)
7. [Topology semantics](#topology-semantics)
8. [Derived invariants](#derived-invariants)
9. [What the schema does not decide](#what-the-schema-does-not-decide)

## Authority and Notation

Use `source/schema/omniSolid/resources/schema.usda` as the authority. That layer was last changed by `solidmodeling` commit `9889a1fc579671a229c917988469593007c7266f` on 2026-06-05, and this reference was rechecked on 2026-07-14. Recheck current source before relying on it.

Labels used below:

- **S**: explicitly stated in the schema declaration, metadata, or documentation.
- **D**: necessarily derived from one or more S rules.
- **U**: not decided by the available schema.

If the authoritative layer does not state a rule, classify it as **U** until an approved source or schema change resolves it.

## Schema Inventory

`BrepArray` is a concrete typed schema inheriting `Gprim`. Its declared arrays are `uniform`. The schema is codeless (`skipCodeGeneration = true`), so runtime behavior depends on plugin metadata and string-based schema APIs rather than generated wrapper classes.

Applied API inventory:

| API schema | Kind | Allowed instance names | Authored API token / property prefix |
|---|---|---|---|
| `BrepPointAPI` | multiple-apply | `vertexPoint`, `shellPoint` | `BrepPointAPI:<instance>` / `brep:<instance>:point:*` |
| `BrepCurve3dNurbAPI` | multiple-apply | `edge3dNurb`, `wireEdge3dNurb` | `BrepCurve3dNurbAPI:<instance>` / `brep:<instance>:curve3d:nurb:*` |
| `BrepCurve3dLineAPI` | multiple-apply | `edge3dLine`, `wireEdge3dLine` | `BrepCurve3dLineAPI:<instance>` / `brep:<instance>:curve3d:line:*` |
| `BrepCurve3dCircleAPI` | multiple-apply | `edge3dCircle`, `wireEdge3dCircle` | `BrepCurve3dCircleAPI:<instance>` / `brep:<instance>:curve3d:circle:*` |
| `BrepCurve3dEllipseAPI` | multiple-apply | `edge3dEllipse`, `wireEdge3dEllipse` | `BrepCurve3dEllipseAPI:<instance>` / `brep:<instance>:curve3d:ellipse:*` |
| `BrepCurveUvNurbAPI` | single-apply | none | `BrepCurveUvNurbAPI` / `brep:curveUv:nurb:*` |
| `BrepSurfaceNurbAPI` | single-apply | none | `BrepSurfaceNurbAPI` / `brep:surface:nurb:*` |
| `BrepSurfacePlaneAPI` | single-apply | none | `BrepSurfacePlaneAPI` / `brep:surface:plane:*` |
| `BrepSurfaceCylinderAPI` | single-apply | none | `BrepSurfaceCylinderAPI` / `brep:surface:cylinder:*` |
| `BrepSurfaceConeAPI` | single-apply | none | `BrepSurfaceConeAPI` / `brep:surface:cone:*` |
| `BrepSurfaceSphereAPI` | single-apply | none | `BrepSurfaceSphereAPI` / `brep:surface:sphere:*` |
| `BrepSurfaceTorusAPI` | single-apply | none | `BrepSurfaceTorusAPI` / `brep:surface:torus:*` |

All APIs declare `apiSchemaCanOnlyApplyTo = ["OmniSolidBrepArray"]`, the registered internal type corresponding to authored type name `BrepArray`. An authored `apiSchemas` token does not by itself prove valid applicability, correct registration, complete data, or semantic consistency.

## Packed Topology Contract

Let these counts describe the flattened arrays:

| Symbol | Meaning | Schema-defined count relationship |
|---|---|---|
| `B` | Breps | `len(brep:intersectTol3d) = len(brep:regionCount) = B`; `len(brep:extent) = 2B` |
| `R` | Regions | `R = sum(brep:regionCount)` |
| `S` | Shells | `S = sum(region:shellCount)` |
| `FU` | Faceuses | `FU = sum(shell:faceuseCount)` |
| `F` | Faces | `FU = 2F`; face arrays have `F` entries except `face:range`, which has `2F` |
| `L` | Loops | `L = sum(face:loopCount)` |
| `EU` | stored one-sided edge-to-face connections | `EU = sum(loop:edgeuseCount)` |
| `E` | Edges | edge arrays have `E` entries except flat `edge:range`, which has `2E` |
| `W` | Wire edges | `W = sum(shell:wireEdgeCount)`; flat `wireEdge:range` has `2W` |
| `V` | Vertices | `len(vertex:pointType) = V` |

**S: Brep arrays**

- `brep:intersectTol3d` is `double[]`; each entry is the maximum distance at which two objects intersect and the minimum distance at which two points are distinct.
- `brep:extent` is `double3[]`; consecutive `{XYZmin, XYZmax}` entries bound each Brep.
- `brep:regionCount` is `uint[]`.

`brep:extent` and the inherited `UsdGeomBoundable` `extent` property serve different scopes:

- `brep:extent` is a uniform, double-precision range per packed Brep. Prefer it for
  BRep-specific partitioning, per-Brep containment checks, and reconstruction.
- `extent` is the standard prim-wide, local-space `float3[]` range inherited through `Gprim`.
  Prefer it for general USD bounding-box caches, traversal, imaging, and culling; it cannot
  identify the bounds of an individual Brep in the array.
- **Authoring recommendation, not an explicit BrepArray requirement:** provide both and derive
  `extent` as a float-precision enclosure of the union of all `brep:extent` ranges. This is what
  `source/BREP_USD_DATA/src/UsdBrepWrite.cpp` currently writes.
- **Current implementation policy:** if the properties disagree, report the inconsistency rather
  than silently choosing one globally. The BrepArray schema defines the per-Brep array and inherits
  standard `extent` semantics, but it does not explicitly state the cross-property union or
  containment rule. The validator's `BA.040`-`BA.050` checks currently enforce containment;
  classify them as implementation policy unless that relationship is added to the authoritative
  schema.

**S: Region arrays**

- `region:shellCount` is `uint[]`.
- `region:type` is `token[]` with `solidRegion` or `voidRegion`.
- Regions are packed by Brep; the first region for each Brep is the infinite region.

**S: Shell arrays**

- `shell:faceuseCount` and `shell:wireEdgeCount` are `uint[]`.
- `shell:pointType` is `token[]` with `BrepPointAPI` or `none`.
- Shells are packed by region. The first shell is the outer shell; later shells are inner shells.
- `shell:pointType` matters only when both counts are zero.

**S: Faceuse and face arrays**

- `faceuse:faceIndex` is `uint[]` and indexes the face arrays.
- `faceuse:orientationType` is `same` or `opposite` relative to the surface normal.
- Faceuses are packed by shell. The schema states that their count is twice the face count.
- `face:loopCount` is `uint[]`.
- `face:surfaceType` is one of the six surface API tokens in the schema.
- `face:trimType` is `rectangular` or `general`.
- `face:range` is `double2[]` with consecutive `{UVmin, UVmax}` entries per face.
- Faces have no topology-use packing order, but objects for each Brep must remain consecutive.

**S: Loop and edgeuse arrays**

- `loop:edgeuseCount` and `loop:vertexIndex` are `uint[]`.
- Loops are packed by face. The first loop is outer; later loops are inner.
- Edgeuses in a loop are head-to-tail connected.
- Use `loop:vertexIndex` only when `loop:edgeuseCount == 0`; it may be shared with edge or wire-edge vertices.
- `edgeuse:edgeIndex` and `edgeuse:nextRadialEUIndex` are `uint[]`.
- `edgeuse:orientationType` is `same` or `opposite` and relates the UV trim direction to the 3D edge-curve direction.
- `edgeuse:thisRadialEntryType` is `topEntry` or `bottomEntry` and defines entry/exit order through the mated top/bottom face sides.
- Edgeuses are packed by loop. `nextRadialEUIndex` describes a right-hand-rule radial traversal around the owning edge.

**S: Edge, wire-edge, and vertex arrays**

- `edge:curveType` and `wireEdge:curveType` accept the NURB, line, circle, or ellipse curve API tokens.
- `edge:range` and `wireEdge:range` are flat `double[]` arrays of consecutive parameter-bound pairs.
- `edge:vertexIndices` and `wireEdge:vertexIndices` are `int2[]`; the schema notes that values are logically unsigned because USD has no `uint2`.
- The start vertex equals the curve evaluated at the first range bound; the end vertex equals evaluation at the second range bound.
- Wire edges are packed by shell. Edges and vertices have no use-based order, but each Brep's objects remain consecutive.
- `vertex:pointType` currently accepts only `BrepPointAPI`.

## Geometry Packing

Geometry arrays are associated arrays, not arrays indexed directly by topology index. Map them by stable occurrence order:

1. Scan the owning type-token array in order.
2. Count only occurrences of the selected token.
3. Map occurrence `k` to geometry entry `k` for that API and, for multiple-apply APIs, that instance.

Examples:

- The third `BrepSurfaceCylinderAPI` occurrence in `face:surfaceType` maps to entry 2 in every `brep:surface:cylinder:*` array.
- The second NURB edge maps to entry 1 in `brep:edge3dNurb:curve3d:nurb:order`, while its flat control-point and knot slices use prefix sums of the per-curve counts.
- Edge and wire-edge geometry use separate multiple-apply instances and separate occurrence counts.
- `brep:vertexPoint:point:position` is packed in vertex occurrence order; `brep:shellPoint:point:position` is packed in point-shell occurrence order.
- UV NURB curve data is packed in edgeuse order when `BrepCurveUvNurbAPI` is applied.

All objects belonging to one Brep must be consecutive in the flattened topology. Therefore, prefix sums define each Brep partition, and references must resolve inside the owning Brep partition as a derived invariant.

Exact geometry property inventory:

| Geometry | Property names and USD types |
|---|---|
| Point, instance `vertexPoint` or `shellPoint` | `brep:<instance>:point:position` (`point3d[]`) |
| 3D NURB curve, instance `edge3dNurb` or `wireEdge3dNurb` | `brep:<instance>:curve3d:nurb:controlVertices` (`point3d[]`), `vertexCount` (`uint[]`), `order` (`uint[]`), `knots` (`double[]`), `weights` (`double[]`) |
| Line, instance `edge3dLine` or `wireEdge3dLine` | `brep:<instance>:curve3d:line:origin` (`point3d[]`), `direction` (`vector3d[]`) |
| Circle, instance `edge3dCircle` or `wireEdge3dCircle` | `brep:<instance>:curve3d:circle:center` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`), `radius` (`double[]`) |
| Ellipse, instance `edge3dEllipse` or `wireEdge3dEllipse` | `brep:<instance>:curve3d:ellipse:center` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`), `xRadius` and `yRadius` (`double[]`) |
| UV NURB curve | `brep:curveUv:nurb:controlVertices` (`double2[]`), `vertexCount` and `order` (`uint[]`), `knots` and `weights` (`double[]`) |
| NURB surface | `brep:surface:nurb:controlVertices` (`point3d[]`), `uVertexCount`, `vVertexCount`, `uOrder`, `vOrder` (`uint[]`), `uKnots`, `vKnots`, `weights` (`double[]`) |
| Sphere | `brep:surface:sphere:center` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`), `radius` (`double[]`) |
| Plane | `brep:surface:plane:origin` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`) |
| Cylinder | `brep:surface:cylinder:origin` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`), `radius` (`double[]`) |
| Cone | `brep:surface:cone:origin` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`), `radius` and `semiAngle` (`double[]`) |
| Torus | `brep:surface:torus:origin` (`point3d[]`), `axis` and `refDirection` (`vector3d[]`), `majorRadius` and `minorRadius` (`double[]`) |

## NURBS Contract

**S: 3D and UV curves**

- `order = degree + 1`.
- `order` is positive and `order <= vertexCount`.
- Each curve knot slice has `vertexCount + order` entries in non-decreasing order, including multiplicity.
- Control-vertex and weight slices each have `vertexCount` entries.
- Every weight is positive.
- Control points are not pre-weighted.
- 3D curve control points use `point3d`; UV curve control points use `double2`.

**S: Surfaces**

- Each surface has positive `uOrder` and `vOrder` and corresponding U/V vertex counts.
- Each U knot slice has `uVertexCount + uOrder` entries; each V slice has `vVertexCount + vOrder`.
- Knot slices are non-decreasing.
- Each surface has `uVertexCount * vVertexCount` `point3d` control vertices and positive weights.
- Control vertices are row-major with U as rows and V as columns.
- Points are not pre-weighted.

The schema permits order 1 because it says only "positive"; a minimum order of 2 is additional policy unless the schema changes.

## Analytic Geometry Contract

Let `Z = axis`, `X = refDirection`, and `Y = Z x X`. Where present, `axis`, `refDirection`, and line `direction` are unit vectors; `axis` and `refDirection` are orthogonal.

| Type | Schema equation and parameter units | Explicit scalar constraints |
|---|---|---|
| Line | `C(t) = origin + t * direction`; unit-speed linear `t` | direction is unit length |
| Circle | `C(t) = center + r cos(t) X + r sin(t) Y`; `t` in radians | `r > 0` |
| Ellipse | `C(t) = center + xRadius cos(t) X + yRadius sin(t) Y`; `t` in radians | both radii `> 0`; no `xRadius >= yRadius` rule |
| Plane | `S(u,v) = origin + u X + v Y`; both parameters linear and unbounded | implicit U/V scale is 1 |
| Sphere | `S(u,v) = center + r[cos(v)cos(u)X + cos(v)sin(u)Y + sin(v)Z]`; both angles in radians | `r > 0`; outward normal |
| Cylinder | `S(u,v) = origin + r cos(u)X + r sin(u)Y + vZ`; U radians, V linear | `r > 0`; outward normal |
| Cone | `R(v)=r+v tan(a)` and `S(u,v)=origin+R(v)cos(u)X+R(v)sin(u)Y+vZ`; U and `a` radians, V linear | `r >= 0`; `a` in `(-pi/2, pi/2)`; zero is explicitly a cylinder degeneration; negative angles are permitted by the stated range |
| Torus | `S(u,v)=origin+(R+r cos(v))cos(u)X+(R+r cos(v))sin(u)Y+r sin(v)Z`; U/V radians | major and minor radii `> 0`; no stated major/minor ordering |

`face:range` supplies surface domains; `edge:range` and `wireEdge:range` supply curve domains. The schema equations use STEP parameterization. In particular:

- Do not use degrees for angular bounds.
- Do not preserve an SMLib line speed or plane UV scale as a hidden schema parameter; absorb it into ranges during translation.
- Do not store SMLib swap-UV or inside-out implementation flags as if they were schema attributes.

## Topology Semantics

The class documentation motivates closed Breps as watertight partitions of space, but this is not
a blanket closure requirement. `BrepArray` also supports open face shells (sheet models), wire
shells, and point shells. Watertightness applies when a shell is intended to separate regions; the
schema does not fully formalize shell closure, how that intent is declared, or how watertightness
is validated. The topology documentation explicitly states:

- A face has two faceuses, represented by the flattened faceuse count relation.
- A rectangular trim has an outer loop made of four isoparametric UV trim curves.
- Loop edgeuses are connected head-to-tail.
- An edge's endpoint vertices equal curve evaluation at the two stored range bounds.
- An edgeuse record represents a mated top/bottom pair for one side of an edge-to-face connection.
- Seam and strut edges can connect to the same face twice.
- Radial traversal orders edge-to-face connections around an edge by the right-hand rule.

The prose does not fully formalize all manifold, degeneracy, orientation-pairing, seam, or region-classification rules. Do not silently fill those gaps from a particular kernel.

## Derived Invariants

These are strong logical consequences of the explicit representation, but label them as derived when reporting:

- All count equations in [Packed topology contract](#packed-topology-contract) must agree.
- Per-Brep prefix-sum partitions cannot overlap, and references cannot cross into another Brep's partition.
- Each face must be represented by its two faceuses; therefore each valid face index is expected twice when the representation is complete.
- Following a well-formed radial `nextRadialEUIndex` relation must stay on the same edge and close into a cycle.
- Geometry array counts must equal type-token occurrence counts for the corresponding API/instance.
- Evaluating analytic or NURB edge geometry at stored range bounds must agree with referenced vertex positions under the applicable Brep intersection tolerance.
- Applied API declarations, type-token occurrences, and authored geometry arrays must agree; otherwise the representation cannot be interpreted unambiguously.

## What the Schema Does Not Decide

Do not call these schema requirements without an additional approved source:

- Whether every declared property must be authored on an empty or placeholder `BrepArray` prim.
- A NURBS minimum order of 2 rather than the stated positive-order rule.
- A fixed numerical epsilon for unit length, orthogonality, positivity, or containment independent of `brep:intersectTol3d`.
- A primary angular interval such as `[0, 2*pi]`, alignment of full-period ranges to that interval, or a maximum one-period span.
- A positive-only cone semi-angle. The current schema explicitly permits negative and zero values inside `(-pi/2, pi/2)`.
- Control-point containment inside the geometric Brep extent. A control hull can exceed the evaluated shape's bounds.
- A required heuristic distance between an analytic origin/center and a trimmed Brep extent.
- A specific seam-edge heuristic for every full-period analytic face.
- Material `GeomSubset` partition policy beyond inherited USD behavior.
- Complete manifold, degeneracy, or healing rules not stated by the current layer. Its legacy proposal reference does not establish a current requirement.

When one of these matters, report the schema gap and identify the validator, consumer, translator, or project policy that supplies the extra behavior.
