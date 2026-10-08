<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib BRep Format and Validity Contract

This document describes the in-memory SMLib boundary-representation model rooted at `SmBrep`,
the flattened `SmBrepData` transfer form, and the validity assumptions implemented by the kernel.
It is a source-oriented reference for answering format questions, reviewing malformed topology,
writing constructors and tests, and translating between SMLib and USD `BrepArray`.

Source names below are navigation aids, not a pinned baseline. Verify every named symbol and
executable predicate in the current checkout.

## Contents

1. [Scope and authority](#scope-and-authority)
2. [Validity vocabulary](#validity-vocabulary)
3. [Representation overview](#representation-overview)
4. [Common ownership and list rules](#common-ownership-and-list-rules)
5. [Regions and shells](#regions-and-shells)
6. [Faces, faceuses, loops, and loopuses](#faces-faceuses-loops-and-loopuses)
7. [Edges and edgeuses](#edges-and-edgeuses)
8. [Vertices and vertexuses](#vertices-and-vertexuses)
9. [Orientation and traversal](#orientation-and-traversal)
10. [Geometry, domains, and tolerances](#geometry-domains-and-tolerances)
11. [Permitted topology forms](#permitted-topology-forms)
12. [Flattened `SmBrepData`](#flattened-smbrepdata)
13. [Construction and mutation](#construction-and-mutation)
14. [Validation model](#validation-model)
15. [Known validator gaps and hazards](#known-validator-gaps-and-hazards)
16. [Worked structural examples](#worked-structural-examples)
17. [SMLib and USD BrepArray](#smlib-and-usd-breparray)
18. [Translation and normalization rules](#translation-and-normalization-rules)
19. [Testing checklist](#testing-checklist)
20. [Source index](#source-index)

## Scope and authority

There is no separate normative SMLib BRep specification in this repository. The contract must be
reconstructed from several source layers:

1. **Runtime model:** public topology and geometry headers under `source/SMLib/inc/` define object
   roles, links, orientations, ownership, and query semantics.
2. **Construction and mutation:** constructors and Euler-style editing operations establish the
   graph invariants that valid completed objects are expected to preserve.
3. **Validation:** `SmBrep::ValidatePointers`, `SmBrep::AssertValid`, and each topology type's
   `AssertValid` encode structural and geometric acceptance rules.
4. **Algorithms and tests:** traversal, classification, healing, stitching, and translation code
   expose assumptions not always asserted directly.
5. **Transfer implementations:** `SmBrepData` and USD conversion describe serializations of the
   runtime model. They are not, by themselves, the definition of runtime validity.

When these disagree, report the disagreement. Do not silently redefine SMLib validity to match a
serializer or a single validator predicate. The strongest practical evidence is a rule consistently
established by the type model and constructors and consumed by kernel algorithms. A validator check
is strong implementation evidence, but can contain bugs, conditional legacy behavior, or incomplete
coverage.

This document does **not** describe `SmPolyBrep`, the stable `_omni_solid` handle surface, or the USD
schema as if they were the same format.

## Validity vocabulary

Use these labels when explaining a rule:

| Label | Meaning |
|---|---|
| **Model invariant** | Required by the runtime ownership/incidence model and relied on broadly. |
| **Geometric invariant** | Required agreement between topology and geometry, usually within a named tolerance. |
| **Constructor convention** | State produced by standard constructors; may be narrower than every representable valid graph. |
| **Validator enforcement** | Predicate currently checked by `ValidatePointers` or an `AssertValid` implementation. |
| **Conditional enforcement** | Check depends on test level, build macros, lazy data, or topology classification. |
| **Validator gap** | Intended/model rule is missing, disabled, defective, or unsafe to exercise on arbitrary corruption. |
| **Translation policy** | Choice made by `BREP_SM_USD`; not an inherent SMLib validity rule. |

For a compliance answer, state both the rule and how it is enforced. "`AssertValid` passes" is not a
complete definition of correctness.

## Representation overview

### Runtime object graph

SMLib uses explicit *use objects* to represent incidence. A geometric/topological entity is stored
once, while each incidence into a higher-dimensional owner is represented separately.

```text
SmBrep
  region circular list -> SmRegion
    shell circular list -> SmShell
      one directly owned use kind, selected by shell type:
        SmFaceuse list      (surface shell; may additionally contain wire edges)
        connected SmEdgeuse (wire shell)
        one SmVertexuse     (vertex shell)

  global face list -> SmFace -> one SmSurface
                         |-> primary SmFaceuse (SAME)
                         `-> mate SmFaceuse    (OPPOSITE)
                               each belongs to a shell and owns ordered SmLoopuses

  logical SmLoop -> primary SmLoopuse + mate SmLoopuse
                      each is either an edge loop or a one-vertex loop

  global edge list -> SmEdge -> one 3D SmCurve + parameter interval
                         `-> circular radial list of individual SmEdgeuses

  global vertex list -> SmVertex -> one 3D point
                           `-> circular list of SmVertexuses
```

The graph is deliberately not a simple containment tree:

- A `SmFace` logically owns a pair of `SmFaceuse` objects, but each faceuse is list-owned by a
  `SmShell`.
- A `SmLoop` logically owns a pair of `SmLoopuse` objects, but each loopuse is list-owned by the
  corresponding `SmFaceuse`.
- A `SmEdge` owns all individual edgeuses radially. Each loop edgeuse also belongs to a loopuse;
  each wire edgeuse belongs directly to a shell.
- A `SmVertex` owns every vertexuse. Each vertexuse points to the shell, loopuse, or edgeuse whose
  incidence it represents.

This is why pointer reciprocity, list ownership, and graph traversal are part of validity, not mere
implementation details.

### Cardinality summary

| Entity | Required related objects in a completed graph |
|---|---|
| `SmBrep` | One infinite region infrastructure object; zero or more other regions; global lists for faces, edges, vertices. |
| `SmRegion` | Zero or more shells for the infinite region; at least one shell for every non-infinite region in a non-empty multi-region BRep. |
| `SmFace` | Exactly one surface and exactly two mated faceuses. |
| `SmLoop` | Exactly two mated loopuses, one under each mated faceuse. |
| Edge loopuse | A non-empty circular chain of edgeuses. |
| Vertex loopuse | Exactly one vertexuse; its mate loopuse has the paired vertexuse on the same vertex. |
| `SmEdge` | One curve, one nonzero modeled interval, one or more logical uses represented by an even number of individual edgeuses. |
| `SmVertex` | One point and one or more vertexuses in ordinary attached topology. |

Sources: `SmBrep.h`, `SmRegion.h`, `SmShell.h`, `SmFace.h`, `SmLoop.h`, `SmEdge.h`,
`SmVertex.h`, and `SmBrep::ValidatePointers` in `source/SMLib/src/SmBrep.cpp`.

## Common ownership and list rules

### Context identity

Every attached topology object and owned geometry object belongs to the same `SmContext` as the
containing BRep. Context equality is checked at the BRep, shell, face, edge, and use layers. An object
allocated in another context is not made valid merely by assigning its pointer into the graph.

Sources: `SmBrep::AssertValid` in `SmBrep.cpp`, `SmFace::AssertValid` in `SmFace.cpp`,
`SmEdge::AssertValid` in `SmEdge.cpp`, and the use validators.

### Circular intrusive lists

Most topology lists are circular, doubly linked intrusive lists:

- `m_pListOwner` identifies the list owner.
- `m_pNext` and `m_pLast` are reciprocal peers.
- Walking `next` from the head must return to the head after exactly the owner's recorded count.
- An object without a list owner must not retain peer links.
- The owner's `m_pList == NULL` state must agree with a zero list size.

`SmTopology::AssertValid` checks membership, reciprocal links, marks, context, and owner/count
agreement in `SmTopology.cpp`. `SmOwningTopology::AssertValid` checks child-list closure, size,
backpointers, and context in `SmOwningTopology.cpp`.

The BRep's global face, edge, and vertex lists use internal list-head objects and are not all treated
as ordinary ownership lists by every generic assertion. Use the type-specific traversal and list
checks as well.

### Primary pointers are semantic

Several list heads are not arbitrary:

- The first BRep region is the infinite region.
- For a non-infinite region, its first shell is its outer shell.
- A face's `m_pFU` is the `SM_OT_SAME` faceuse.
- A loop's `m_pLU` is the loopuse under the primary/upward faceuse.
- A faceuse's first loopuse is the outer loopuse.
- An edge's `m_pList` is the primary `SM_OT_SAME` edgeuse.

Reordering these objects can change meaning even if the same set of pointers remains reachable.

## Regions and shells

### Regions

An `SmRegion` represents a connected volume of space. The first region is the unbounded/infinite
region. Other regions represent bounded volumes; `m_bIsVoidFlag` records whether a bounded region is
classified as void rather than material.

The BRep-level rules are:

1. `FindInfiniteRegion()` must find the object stored in `m_pInfiniteRegion`.
2. That object must be first in the region list.
3. More than one region implies at least one face.
4. In a non-empty multi-region BRep, every non-infinite region has at least one shell.
5. The first shell of each non-infinite region is its outer shell; it must be a closed faceuse shell.
6. Later shells are inner shells and must classify geometrically inside the outer shell.
7. Every shell of the infinite region is an inner shell because that region has no outer shell.
8. If multiple regions exist, the infinite region must contain at least one closed faceuse shell.
9. All regions must be connected through faceuse adjacency; a traversal from the infinite region must
   recover exactly the BRep's region list.

These are enforced in `SmBrep::AssertValid` in `SmBrep.cpp`.

An otherwise empty BRep is special-cased out of most region/shell containment tests in
`SmBrep.cpp`, but it still needs valid root/list infrastructure and its infinite region. Treat this
as a validator-permitted construction state, not as a useful solid.

### Shell type is exclusive at the direct-owner level

`SmShell::m_tShellType` identifies its highest-dimensional directly owned incidence kind:

| Shell type | Direct list head | Meaning | Additional content |
|---|---|---|---|
| `SmFaceuse_TYPE` | `SmFaceuse` | Connected surface boundary component | May also contain wire edges; no shell vertex. |
| `SmEdgeuse_TYPE` | `SmEdgeuse` | Connected wire graph with no faces | No faceuses and no shell vertex. |
| `SmVertexuse_TYPE` | `SmVertexuse` | Isolated point shell | Exactly one shell vertexuse; no faceuses or wire edges. |

`SmShell::AssertValid` in `SmShell.cpp` dispatches on this type.

A faceuse shell with embedded wire edges is legal. The presence of wires does not change it into a
wire shell. A wire-only shell cannot contain faceuses. A vertex shell is exactly one isolated point
incidence, not a bag of disconnected vertices.

### Shell connectivity

- A wire shell's direct edgeuse is a traversal seed for one connected wire component. The edgeuses
  found through connectivity must equal all wire edgeuses that point back to that shell.
- A faceuse shell's faceuse list, topology traversal, and all faceuses with a shell backpointer must
  identify the same set.
- The first faceuse in a faceuse shell must contain a first loopuse, and that loopuse must be an edge
  loop. This provides a traversal seed in `SmShell.cpp`.
- Across every radial face edge, the radial edgeuse must belong to the same shell. `SmShell::AssertValid`
  checks faceuse backpointers (tests 24–26). 

### What `SmShell::IsClosed` means

`IsClosed()` applies to faceuse shells. The implementation in `SmShell.cpp` considers a shell closed
when faceuse mates cross to another shell, thereby separating regions. This is a
region-boundary/topological notion, not merely "every edge has two incident faces." Non-manifold fins
and higher radial valence can exist. Do not infer regular manifold-solid status from `IsClosed()`
alone; use the relevant manifold/solid query and validation.

## Faces, faceuses, loops, and loopuses

### Face and surface

A completed `SmFace` has:

- one non-null `SmSurface` owned by that face;
- a nonzero modeled UV domain contained in the surface's natural domain;
- one primary/upward `SmFaceuse` with orientation `SM_OT_SAME`;
- one mutual mate faceuse with orientation `SM_OT_OPPOSITE`;
- both faceuses in faceuse shells, possibly the same shell for an open sheet or different shells for
  a region boundary.

The surface owner pointer must point back to the face. A surface already owned elsewhere must be
copied before attachment; `SmBrepConstructor::SetFaceSurface` implements that behavior in
`SmBrepData.cpp`.

At full validation, face geometry must not degenerate to a point or curve at the tested tolerance.
Surface poles/singularities in the modeled area must be represented by vertex topology rather than
being classified as an ordinary face or edge location. Face edges must lie on the surface within the
intersection/tolerance model.

### Faceuse pair

The two faceuses are two oriented sides of one face, not two faces:

- They point to the same `SmFace`.
- Their mate pointers are mutual.
- Their orientations are opposite (`SAME` and `OPPOSITE`).
- Each belongs to a shell.
- Their loopuse lists contain corresponding sides of the same logical loops.

`SmFaceuse::AssertValid` in `SmFaceuse.cpp` mainly verifies generic list/context relationships; the
deeper pair rules live in `SmBrep::ValidatePointers`.
`SmFace::AssertValid` calls both faceuses at `LEVEL_0`.

### Loop pair and ordering

Each logical `SmLoop` is represented by two mated loopuses, one in each faceuse. The primary
loopuse is under the primary faceuse. Both sides have the same outer/inner orientation classification:

- The first loopuse in each faceuse is the outer loop and has `SM_OT_SAME`.
- Every later loopuse is inner and has `SM_OT_OPPOSITE`.
- The primary loopuse of a logical loop belongs to the primary faceuse; its mate belongs to the mate
  faceuse.
- Corresponding loopuses have the same number of edgeuses and reference the same edges in the
  paired/reverse traversal relationship.

Outer/inner orientation and loopuse mate pointers are explicit in `SmBrep::ValidatePointers` in
`SmBrep.cpp`. Paired loopuse edge counts, edge-set agreement, and reverse-order alignment are owned
by `SmLoop::AssertValid` tests 3–6 in `SmLoop.cpp`.

### Face-level loop relationships

Current `SmFace::AssertValid` adds three partial cross-loop checks:

- Test 37 (`LEVEL_0`) classifies one midpoint UV sample from each inner loop against the outer and
  other inner loops. A missing sample or `SM_POC_UNKNOWN` result is inconclusive, and passing does
  not prove whole-loop containment; a loop that crosses the outer boundary can be missed.
- Test 38 (`LEVEL_2`) intersects usable UV trim curves between loop pairs. Trims that cannot be
  built and curve-pair intersection failures are inconclusive, and passing does not establish
  topological disjointness.
- Test 39 (`LEVEL_0`) rejects a vertex shared by two different loops. Reusing a vertex within one
  loop, as in a slit or figure-eight boundary, is not rejected. This is the shared-topology check
  that complements test 38.

These checks are implemented and registered in `SmFace.cpp`.

### Edge loops

An edge loopuse points to a non-empty circular chain of individual edgeuses. For each edgeuse:

- `CCW` and `CW` neighbor pointers are reciprocal.
- In the loop's traversal orientation, this edgeuse's start vertex equals its clockwise neighbor's
  end vertex, and its end vertex equals its counterclockwise neighbor's start vertex.
- The connection uses the same `SmVertex` object, not merely two coincident vertex points, except
  where a specific lamina path relaxes a check.
- Loop closure is also checked geometrically through `SmTrimmingTools::CheckLoop` (from
  `SmLoop::AssertValid`) and 3D endpoint gaps against `XSectTol3d` on the primary loopuse
  (`SmLoopuse::AssertValid` test 5). 

An outer face loop cannot be a vertex loop. The first loop of each face side must be an edge loop.

### Vertex loops

A vertex loop is a legitimate inner-loop representation for a face singularity or collapsed
boundary:

- Each of the two loopuses has type `SmVertexuse_TYPE`.
- Each points to exactly one vertexuse.
- Both vertexuses belong to the same `SmVertex` and point back to their respective mated loopuses.
- The two loopuses belong to the same logical loop and mated faceuses of the same face.
- It carries no edgeuses.
- It cannot be the first/outer loop under the current validity contract.

`SmVertex::CheckPointers` checks this four-way relationship in `SmVertex.cpp`.
`SmBrepConstructor::StartAndEndSingleVertexLoop` creates it in `SmBrepData.cpp`.
That helper does not itself require a pre-existing outer edge loop: if called first, `StartLoop`
marks the new loop outer and the helper returns a graph that `ValidatePointers` rejects. API
availability is therefore not evidence that every call order is valid.

## Edges and edgeuses

### Edge and curve

A completed `SmEdge` has:

- one non-null three-dimensional `SmCurve`, owned by the edge;
- a nonzero modeled interval contained in the curve's natural interval;
- one start and one end vertex index/reference, which may identify the same vertex for a closed edge;
- an even, nonzero number of individual edgeuses in a circular radial list;
- a primary edgeuse with `SM_OT_SAME` orientation.

The approximate curve length over the modeled interval must not be smaller than the edge tolerance.
Vertex cardinality is 1 (closed) or 2 (open), including wires; start/end curve points must agree
with the vertex points within `edgeTol + vertexTol` (`SmEdge::AssertValid` tests 15–17). At full
validation, the edge curve must lie on every incident face surface within the intersection
tolerance. Sources: `SmEdge::AssertValid` in `SmEdge.cpp` and `SmBrep::ValidatePointers` in
`SmBrep.cpp`.

### One logical face-edge incidence has two runtime edgeuses

For a loop edge, the two sides of the face require a mated pair:

```text
Face F
  Faceuse F+ / Loopuse L+ -> Edgeuse EU+
  Faceuse F- / Loopuse L- -> Edgeuse EU- = mate(EU+)
```

The pair belongs to the same logical loop and face but runs in opposite directions. This pair is one
logical face-edge occurrence. It is important not to count it as two independent BrepArray edgeuse
records; `SmBrepData` and USD both collapse the pair.

### Radial list order

For an edge with individual runtime edgeuses, the circular `next` order is:

```text
[primary, mate(primary), radial(mate), mate(radial), ..., radial(primary)]
```

Equivalently:

- Pairs at `[0,1]`, `[2,3]`, ... are mates belonging to the same face.
- Pairs across `[1,2]`, `[3,4]`, ..., `[last,0]` are radial partners bounding sectors in the same
  shell.
- Orientations alternate. Even entries match the primary orientation; odd entries oppose it.
- If an edgeuse has the primary orientation, its mate is `next` and its radial is `last`; for the
  opposite orientation, those directions reverse.

The model is documented in `SmEdgeuse.h`; `ValidatePointers` makes the ordering assumption explicit
in `SmBrep.cpp`.

### Edgeuse state

Every individual `SmEdgeuse` has:

- an edge backpointer;
- an orientation relative to the edge;
- a mated edgeuse and a radial edgeuse, both reciprocal;
- a starting `SmVertexuse` of type `SmEdgeuse_TYPE`;
- either a containing loopuse (`SmLoopuse_TYPE`) with reciprocal CW/CCW neighbors, or a containing
  shell (`SmShell_TYPE`) for a wire edgeuse;
- optionally, a two-dimensional UV trim B-spline curve.

Start/end interpretation follows orientation:

| Orientation | Start vertex/parameter | End vertex/parameter |
|---|---|---|
| `SM_OT_SAME` | Edge start / interval minimum | Edge end / interval maximum |
| `SM_OT_OPPOSITE` | Edge end / interval maximum | Edge start / interval minimum |

### UV trim curves

An edgeuse UV trim curve is **optional, derived data**, not a defining characteristic of the BRep.
The boundary is defined by topology (including use orientations), the edge's 3D curve and interval,
and the face surface. SMLib can derive the UV representation lazily from that information. These
edgeuse trims are distinct from a UV curve used internally to define an `SmCrvOnSurf` 3D edge curve;
the latter is part of the edge's defining geometry.

When an edgeuse UV trim is present:

1. The curve is two-dimensional and owned by an edgeuse.
2. It is stored only once per mate pair, on the `SM_OT_SAME` edgeuse.
3. Its direction agrees with the 3D edge curve's minimum-to-maximum direction.
4. Its natural interval equals the edge's modeled interval within the parameter comparison policy.
5. That interval is compatible with the 3D curve interval.
6. Its endpoint and sampled values lie in or very near the face UV domain.
7. Lifting it through the surface agrees with the 3D edge and endpoint vertices within tolerances.
8. It must not make an improper jump across a periodic seam or introduce a control-polygon dogleg
   near a pole.

Sources: `SmEdgeuse.h`, `SmEdgeuse::AssertValid` in `SmEdgeuse.cpp`, and `ValidatePointers` in
`SmBrep.cpp`.

Absence of a UV curve is not intrinsically invalid. Consumers may generate one. Distinguish that
from an authored "zero-order NURBS" sentinel in USD, which is not the same runtime state.

A wrong UV curve can make loop-orientation, UV closure, or seam checks fail on correctly constructed
topology. Check edgeuse orientations, vertex identity, loop order, edge intervals, and 3D endpoint
pairing independently. For a controlled diagnosis, preserve those and the face/edge geometry while
rebuilding only the suspect edgeuse trims. Record tolerance/cache side effects separately. If this
resolves the failure with the defining data unchanged, investigate trim generation or seam-side
assignment rather than changing topology to fit the bad trim.

`SmEdgeuse::GetOrCreateUVTrimCurve` and `SmFace::CreateUVTrimCurves` reuse existing trims; simply
calling them may preserve the defect. Use the supported `RebuildUVTrimCurve` or
`DeleteUVTrimCurve` followed by regeneration on a diagnostic copy. At a periodic seam both UV sides
can lift to the same 3D curve. At a pole, distinct UV endpoints can also represent the same vertex,
so neighboring vertexuse/trim UVs alone may not select the correct seam side. Compare the oriented
inward binormal with the surface cross-boundary derivative at a nonsingular interior edge point.
See `SmEdgeuse::CreateUVTrimCurve` in `SmEdgeuse.cpp` and `sm_RedistributeSeamUVTrimCurves` in
`SmBrep.cpp`. A correct 3D lift alone does not prove correct seam-side assignment.

## Vertices and vertexuses

### Vertex

An `SmVertex` stores one 3D point and owns a circular list of every incidence at that point. Ordinary
topological connectivity requires use objects to share the same vertex object; coincident points in
distinct vertices are generally rejected as coincident topology rather than treated as connected.

At full validation, the point is checked against incident edges and surfaces and against the local
intersection/tolerance model. `SmVertex::AssertValid` also calls `CheckPointers` and validates all
vertexuses in `SmVertex.cpp`.

### Vertexuse kinds

`SmVertexuse::m_tVertexuseType` selects the object addressed by its union-like pointer:

| Vertexuse type | Target | Meaning |
|---|---|---|
| `SmShell_TYPE` | `SmShell` | The sole incidence of an isolated vertex shell. |
| `SmLoopuse_TYPE` | `SmLoopuse` | One side of a single-vertex loop. |
| `SmEdgeuse_TYPE` | `SmEdgeuse` | The starting vertex incidence of an edgeuse. |

The target must point back to the vertexuse, and the vertexuse must be in the owning vertex's list.
`SmVertex::CheckPointers` enumerates shell-vertex, vertex-loop, wire-edge, and loop-edge cases.

### Sector rules and a coverage caveat

Vertex sectors are used to reason about the ordering of incident face geometry. Missing periodic seam
topology is checked by `SmVertexuse::AssertValid` test 5 (`IsSectorMissingSeam`). 
Loopuse-type and non-wire edgeuse-type vertexuses must connect to a
faceuse (test 6). A broader sector gap/orientation assertion is currently commented out in
`SmVertexuse.cpp` because it produced cascading reports. Thus a validator pass does not prove every
intended vertex-sector condition.

## Orientation and traversal

`SmOrientType` is contextual; see `SmTypes.h`:

| Object | `SM_OT_SAME` | `SM_OT_OPPOSITE` |
|---|---|---|
| Faceuse | Oriented normal equals the surface natural normal. | Oriented normal opposes it. |
| Loopuse | Outer loop, conventionally CCW relative to the relevant oriented surface side. | Inner loop, conventionally CW. |
| Edgeuse | Traverses the edge from interval minimum/start vertex to maximum/end vertex. | Traverses maximum/end to minimum/start. |

Do not apply the loopuse meaning to edgeuses or the faceuse meaning to loops. The same enum encodes
different relative orientations.

The primary faceuse is always `SAME`; its mate is `OPPOSITE`. Both loopuse sides of an outer loop are
classified `SAME`, and both sides of an inner loop are `OPPOSITE`. Individual mated edgeuses have
opposite orientations.

`SmBrepConstructor::StartFace`, `StartLoop`, and `StartEdge` establish these conventions in
`SmBrepData.cpp`.

## Geometry, domains, and tolerances

### Geometry ownership and supported runtime types

Runtime SMLib is more general than USD BrepArray's analytic/NURBS schema subset:

- A face can own any valid `SmSurface` subclass accepted by the kernel, including B-spline, plane,
  cone, cylinder, sphere, torus, revolution, extrusion, blend, curve-bounded, offset, and STEP
  surfaces. See `SmSurfTypes.h` and `SmSurface.h`.
- An edge can own any valid three-dimensional `SmCurve` subclass, including B-spline, line, conic,
  circle, ellipse, parabola, hyperbola, composite, Hermite, offset, projected, iso, and curve-on-
  surface forms. See `SmCurveTypes.h` and `SmCurve.h`.
- An edgeuse may own an optional two-dimensional `SmBSplineCurve` trim curve.

Each face uniquely owns its surface. Each edge uniquely owns its 3D curve. Each mated edgeuse pair
owns at most one UV curve through its `SAME` member.

### Domains

- Preserve `+/-SM_INFINITE_PARAMETER` markers when scaling parameter bounds. Use sentinel-aware
  helpers such as `SmExtent2d::Scale()`, which checks `SM_IS_INFINITE()` before scaling each endpoint.
  Direct multiplication can turn an unbounded domain into an apparently finite one. This applies
  throughout the kernel, not just during import/export.
- `SmFace::m_vUVDomain` is the modeled rectangular domain and must be nonzero and contained in the
  surface natural domain.
- `SmEdge::m_vInterval` is the modeled curve interval and must be nonzero and contained in the curve
  natural interval.
- A UV trim curve, when present, uses the same parameter interval as its 3D edge. It is not generally
  reparameterized to `[0,1]`.
- Periodic analytic SMLib STEP-style parameterizations commonly use degrees; the USD schema uses
  radians. This is a translation issue, not permission to mix units inside one SMLib object.

### Tolerance roles

Use the context/tolerance APIs rather than assuming every legacy scalar member is active:

- **Zone tolerance (`ZoneTol3d`)** is the working local coincidence tolerance and the default for
  newly created topology.
- **Intersection tolerance (`XSectTol3d`)** is used for intersection, loop closure, distinctness,
  and several topology/geometry agreement tests.
- **Approximation tolerance (`ApproxTol3d`)** controls geometric approximation, including fallback
  conversion to NURBS.
- Vertex and edge objects can carry local effective tolerances/gap caches. A BRep tolerance is a
  lower/default scale, not necessarily the exact tolerance of every member.

Build macros such as `SM_USE_NEWTOL`, `SM_USE_OLDTOL`, and legacy defect guards
change which stored fields and checks compile. Prefer `SmTol::GetZoneTol3d`,
`SmTol::GetXSectTol3d`, and `SmTol::GetApproxTol3d` in new reasoning and code.

### Connected geometry may agree within tolerance

Topology connectivity is exact by object identity; geometry connectivity is tolerance-aware:

- vertex-to-edge endpoint gap <= the effective vertex threshold;
- vertex-to-face gap <= the effective vertex threshold;
- UV-trim-to-vertex gap <= the effective vertex threshold when that cache is available;
- edge-to-surface and edge-to-lifted-UV gap <= the effective edge threshold;
- loop endpoint/closure gap <= the intersection tolerance used by the loop validator.

`SmBrep::ValidateAndUpdateTolerances` in `SmBrep.cpp` documents and checks these relationships. The
check reports vertex gaps against half the stored vertex tolerance in its final assertions, while
its earlier violation/update decision compares against the full stored tolerance. Preserve this
implementation detail when diagnosing boundary cases; do not turn it into a universal mathematical
definition without checking the active tolerance configuration.

### Interpreting gap and tolerance reports

An `AssertValid` tolerance report is evidence about geometric quality at the tested threshold, not
by itself evidence of incorrect topology or a failed modeling operation. A BRep can have known gaps
larger than its working tolerance regardless of how it was created. Report structural validity,
derived-trim correctness, and geometric quality separately; assess each finding against the
requirements of the intended operation.

For each gap/tolerance finding:

- Identify the entities, units, gap kind (vertex/edge, vertex/face, edge/face, lifted trim, or loop
  closure), measured distance, and the actual threshold used by the predicate. A reported stored
  tolerance is not a measured gap. For example, legacy `ValidatePointers` tests 59 and 82 compare
  edge/vertex tolerances with `1000 * brepTolerance`; their values are tolerances.
- Compare a measured gap with the affected edge's 3D length over its modeled interval. A gap
  comparable to that length is a strong reason to investigate reversed orientation or mismatched
  endpoints. Check both normal and swapped endpoint pairings and the endpoint chord: for a strongly
  curved or closed edge, arc length alone can hide a reversal. The ratio is diagnostic context,
  not a universal acceptance threshold or automatic instruction to reverse an edge.
- When input geometry or an earlier model state is available, compare it with the result to
  determine whether the gap was pre-existing or introduced by the operation under investigation.
  Check derived trims separately. A small gap/length ratio does not erase a tolerance violation,
  and a pre-existing gap does not establish a new construction or modification defect.
- Preserve the diagnostic and its significance for the requested operation. Do not inflate
  tolerances, heal geometry, or reverse uses merely to make validation pass.

`ValidatePointers` includes geometry and tolerance checks as well as structural checks. Its name
and return status alone do not identify the cause. After establishing safe graph traversal,
continue deeper diagnosis when the remaining reports only concern geometric quality; missing or
inconsistent required links still block safe traversal.

### Validation can refresh state

`SmBrep::AssertValid` calls `ValidateAndUpdateTolerances(TRUE, ...)`. The root comment in
`SmBrep.cpp` describes the intent as check-only and says only cached data may change. However, the
tolerance routine explicitly notes that surface-cache side effects can update edge tolerances even
in check-only mode. It also increments/locks marks and refreshes lazy gap data. Do not call it on an
object concurrently or assume byte-for-byte immutability.

## Permitted topology forms

### Open sheet and closed solid boundaries

- An open face sheet may place both faceuses in the same faceuse shell. Boundary edges are commonly
  lamina edges.
- A closed region boundary places opposing faceuses in shells on opposite sides of the boundary.
  Multiple regions are connected through those faceuse mates.
- A valid closed boundary is not necessarily a regular two-manifold; SMLib can represent fins and
  higher radial valence.

### Edge classifications

The current query predicates in `SmEdge.cpp` classify common radial forms:

| Form | Runtime edgeuse pattern | Meaning |
|---|---|---|
| Closed edge | Start and end reference the same `SmVertex` | A periodic/closed curve segment represented by one endpoint vertex. |
| Wire edge | Primary edgeuse belongs directly to a shell | No face/loop incidence. |
| Lamina edge | Exactly two loop edgeuses | One logical face-boundary occurrence, typical sheet boundary. |
| Manifold edge | Exactly four loop edgeuses | Two logical face occurrences; may involve two faces or a periodic seam using one face twice. |
| Spine edge | Six or more loop edgeuses | Three or more logical face occurrences, a supported non-manifold radial form. |
| Strut edge | `IsStrut` returns true for fewer than two distinct endpoint vertices; otherwise it checks whether an endpoint is incident only to this edge | The early return classifies every closed edge as a strut regardless of other incident edges, a predicate quirk broader than the dangling-end interpretation. Not the same as a wire edge. |
| Embedded edge | Four individual edgeuses connect to one face and the edge is not its seam | An edge embedded in a face, such as a crack; the edge may also connect to other faces. |

Sources: `SmEdge::IsClosed`, `IsWire`, `IsLamina`, `IsManifold`, `IsSpine`, `IsStrut`, and
`IsEmbedded` in `SmEdge.cpp`.

Do not treat "manifold edge" as proof that the entire BRep is a manifold solid.
Likewise, these query classifications describe incidence patterns; each object must still satisfy
the full pointer, orientation, shell, loop, geometry, and tolerance contract.

### Periodic seam edges

A seam is explicit topology on a surface closed in U or V. `SmEdge::IsSeam` looks for two logical
incidences to the same surface at distinct UV locations; the common manifold seam has four individual
runtime edgeuses. The same face can therefore appear twice around one edge while remaining a valid
manifold radial pattern; see `SmEdge.cpp`.

Seam topology is required where a modeled face boundary crosses a periodic surface closure. Full
face validation checks for missing seams, boundaries crossing seams, and near-miss seam placement
(Face tests 29, 31, 36). Per-sector missing-seam topology is owned by Vertexuse test 5.
`SmLoopuse::IsMissingSeamAtVertices` remains for construction (`SmBrep::MakeEdgeInFaceWithVU`) and
is not an `AssertValid` owner. A periodic surface alone does not imply that every edge on it is a
seam.

### Wire graphs

Wire edges use mated edgeuses whose owner is a shell rather than loopuses. A wire shell represents
one connected component and may branch through shared vertices. Faceuse shells may also carry wire
edges as lower-dimensional embedded content. Use `SmBrep::MakeWireEdge` or
`MakeWireEdgeVertex` in `SmBrep.cpp` instead of manufacturing the use graph manually.

### Vertex shells and vertex loops

- A **vertex shell** is a zero-dimensional shell component: one vertex, one shell vertexuse, no
  edges or faces.
- A **vertex loop** is a zero-length inner boundary on a face: two loop vertexuses on the same vertex,
  one per face side.

They are structurally different and map to different USD fields. A vertex can also participate in
wire or loop edges; predicates named `IsWireVertex` versus `HasWireEdge`, or `IsLoopVertex` versus
`HasLoopVertex`, distinguish exclusive from mixed use. Review predicate implementations before
using their names as cardinality guarantees.

### Coincident topology

Distinct topology objects occupying the same geometric locus are generally suspicious. Root and
face validation run coincident vertex/edge checks, including partial edge coincidence at full test
levels. Exact connectivity should share the same entity. Exceptions require a specific topological
reason, not merely matching coordinates.

## Flattened `SmBrepData`

`SmBrepData` is an indexed transfer/serialization representation, not the live intrusive graph. It
is used by database and USD conversion paths. Its records and top-level arrays are defined in
`SmBrepData.h`.

### Array hierarchy

```text
m_vRegions   -> contiguous shell blocks
m_vShells    -> faceuse block, wire-edge start, or vertex index by shell type
m_vFaceuses  -> face index
m_vFaces     -> contiguous loop blocks + surface index/domain
m_vLoops     -> contiguous logical edgeuse records, or one vertex index
m_vEdgeuses  -> logical mated edgeuse-pair records
m_vEdges     -> curve, interval, start/end vertex, primary logical edgeuse
m_vVertices  -> point
```

The records are static data containers. Their pointers such as `m_pFace`, `m_pShell1`, or
`m_pEdgeuse1` are reconstruction aids, not persistent graph identity.

### Shell collapsing

For an ordinary manifold boundary, one `SmShellData` can represent the paired runtime shell sides.
Non-manifold cases may use one data record per runtime shell. This distinction is documented in
`SmBrepData.h`. Do not assume runtime shell count always equals serialized shell record count.

### `SmEUData` collapses a runtime mate pair

One `SmEUData` represents a pair of individual runtime edgeuses. Its signed radial fields identify
both a target record and which member of that target pair is selected:

- non-negative value `i` selects record `i` member 1;
- negative value `-i-1` selects record `i` member 2.

`m_lEUType` can describe a face edge (`0`), loop vertex (`1`), wire edge (`2`), or a vertex at a pole
in an edge loop (`3`). The record I/O retains all four discriminators, but current
`SmBrepData::FromBrep` populates `m_vEdgeuses` with one record per mated face-edge pair and one per
wire edge (`0` and `2`); it stores a vertex loop through `SmLoopData::m_lVertex`, and a shell vertex
through `SmShellData::m_lVertex`. Current USD import follows the same arrangement. The format notes
in `SmBrepData.h` and the extraction logic in `SmBrepData.cpp` establish this distinction.

The broader wording in the bridge crosswalk in `SmuConvert.cpp` mixes runtime use objects,
flattened records, and regenerated import data. Treat it as conversion guidance, not as the current
`FromBrep` array-cardinality contract or proof that loop/shell vertex records must be present.

This representation-level collapse is the direct conceptual precursor to BrepArray's one stored
edgeuse record per logical face-edge occurrence.

## Construction and mutation

### Prefer complete public operations

Do not directly write `m_pNext`, `m_pLast`, `m_pListOwner`, mate, radial, or union-like type/pointer
fields in new code unless implementing a kernel topology primitive that maintains the entire graph.
Use established constructors, `PostInsert`, edge pair insertion, Euler operations, wire/vertex-loop
helpers, and geometry setters.

### `SmBrepConstructor`

`SmBrepConstructor` provides a top-down builder for typical manifold solids and open shells. Its
header in `SmBrepData.h` explicitly warns that it is not well suited to non-manifold construction.
The lifecycle is:

```text
StartBrep
  [StartRegion]
    StartShell
      StartFace
        SetFaceSurface
        StartLoop
          StartEdge / SetEdgeCurve / SetLimits / EndEdge ...
        EndLoop
        [StartAndEndSingleVertexLoop for an inner vertex loop]
      EndFace
    EndShell
  [EndRegion]
EndBrep
```

Important behavior:

- `StartBrep` allocates in the supplied context.
- `StartShell` creates an outer faceuse shell and, for a bounded region, a corresponding inner shell.
- `StartFace` creates both faceuses and installs `SAME`/`OPPOSITE` orientations.
- `StartLoop` makes the first loop outer and all later loops inner, regardless of the caller's
  orientation argument; that argument is retained for edge construction direction.
- `StartEdge` creates a mated edgeuse pair and inserts it into loop and radial structures. Existing
  edges are extended through `SmEdge::AddOrientedEUPair`.
- `EndLoop` connects corner/mate vertexuses and closes the loop.
- `SetLimits` derives/snaps the edge interval from endpoint nodes and the curve natural domain.
- `SetFaceSurface` and `SetEdgeCurve` establish unique geometry ownership and copy/lift geometry as
  needed.
- `EndShell(TRUE)` can stitch coincident boundary entities; stitching is an explicit normalization,
  not implicit in every constructor call.

Implementation: `source/SMLib/src/SmBrepData.cpp`.

### Intermediate states are often invalid

During construction, a face may temporarily lack a surface, a loop may be open, and an edge may lack
its complete radial or vertexuse links. Run full validation only at documented stable boundaries.
When debugging a builder, validate the largest completed subgraph or use narrowly scoped pointer
checks that tolerate the current construction phase.

## Validation model

### Root call sequence

`SmBrep::AssertValid` in `SmBrep.cpp` performs:

1. base `SmSAGObject`/owning-topology checks;
2. `ValidatePointers` presence, structural, and basic geometry checks, passing `eWalkTree` so a later
   `AssertSubTopology` walk can skip `CheckFace`'s `CheckLoop`;
3. immediate failure if `ValidatePointers` fails;
4. `ValidateAndUpdateTolerances(TRUE, ...)` for cached gap/tolerance agreement;
5. BRep-wide region, shell, connectivity, context, and coincident-topology checks;
6. optional `AssertSubTopology` traversal when `SM_WALK` is requested.

`AssertSubTopology` gathers regions, shells, faces, loops, edges, and vertices, then calls each local
validator without recursively walking again. Face, Loop, Edge, and Vertex `SM_NO_WALK` validators
are local-only; children are covered once by this walk (Faceuses from Face, Loopuses from Loop,
Edgeuses from Edge, Vertexuses from Vertex).

A normal completed-graph invocation is:

```cpp
SmAssertArray reports;
SmBoolean valid = pBrep->AssertValid(&reports, SM_LEVEL_2, SM_WALK);
```

Inspect `reports` even when the boolean is the primary gate. If pointer damage is suspected, run a
careful prerequisite preflight before this call; the deep validators are not safe parsers for
arbitrary memory corruption.

### Test levels

- `SM_LEVEL_0` runs the baseline checks and is already substantial.
- `SM_LEVEL_1` adds checks tagged at that level.
- `SM_LEVEL_2` includes all level 0 and level 1 checks and adds expensive geometry, coincidence,
  seam, and placement work. A separate lower-level call is unnecessary for full validation.
- `SM_LEVEL_GIVEN` runs requested assertion indices and does not necessarily execute the ordinary
  base path. Use it for diagnosis, not general certification.

For format verification, use `SM_LEVEL_2` and `SM_WALK` unless cost or a known corrupt graph makes a
staged approach necessary. Capture `SmAssertArray`; a boolean alone loses object, label, threshold,
and observed-value evidence.

### Structural preflight

`ValidatePointers` tests 112-146 (`SM_LIST_1`) are the crash-safety gate. They walk Brep, Regions,
Shells, Faces, Faceuses, Loops, Loopuses, Edges, Edgeuses, Vertices, and Vertexuses and report only
that each required pointer has some non-NULL value. They do not check identity, orientation,
context, or geometry, and they do not call getters that `SM_ASSERT` then dereference (`GetMate`,
`GetRadial`, `GetBrep`, `GetShell`, `GetFace`, `GetLoop`, `GetVertexuse`). Optional UV trim curves
may be NULL. Wire edgeuse CCW/CW pointers may be NULL. Unused unknown shells may have a NULL
owned-use list. Infinite regions may have no shells. Test 146 fails if a loopuse edgeuse CCW walk
does not return to its start before `lMaxLoopEdgeuseWalk` (100000) steps.

These presence tests run first at `SM_LEVEL_0`, independent of the caller's test-request list. The
walk stops at the first missing required pointer (or unclosed CCW walk) and `ValidatePointers`
returns `SM_ERR` before later getters that would crash. After that it remains a useful structural
gate because it checks the graph relationships required for deeper validators to traverse safely:

- contexts and geometry owners;
- face/faceuse/loopuse pairs and ordering;
- edge radial parity, mate/radial reciprocity, loop neighbor reciprocity, and vertexuses;
- shell/region backpointers and faceuse membership;
- edge and vertex tolerance bands in legacy configurations;
- face, edge, and curve domains and basic geometry validation.

When `ValidatePointers` is called with `SM_WALK` (as from `SmBrep::AssertValid`), `CheckFace`
skips `CheckLoop` and still checks outer/inner orientation; `SmLoop::AssertValid` runs `CheckLoop`.
Direct `ValidatePointers()` callers keep the default `SM_NO_WALK` and still run `CheckLoop` inside
`CheckFace`.

Its contract comment in `SmBrep.cpp` is a valuable index, but the executable predicates decide
current behavior. Audit both because they are not perfectly aligned.

For persisted database data, standalone `SmBrepData`, or a USD asset, validate the source container's
counts, ranges, discriminators, and indices before `MakeTopologyFromData`. `AssertValid` evaluates
the reconstructed runtime graph; a pass does not prove that an original byte stream or USD layer
obeyed its own format contract. Validate USD first with `$usd-brep-schema`, then validate the
expanded SMLib graph independently.

### Per-type coverage

| Validator | Principal coverage |
|---|---|
| `SmBrep::ValidatePointers` | Tests 112-146: presence of required topology pointers and a closed loopuse edgeuse CCW walk. Remaining tests: contexts, geometry owners, face/loop/edge relationships, radial parity, shell/region backpointers. |
| `SmTopology::AssertValid` | Owner membership, circular peer links, context, marks, list count. |
| `SmOwningTopology::AssertValid` | Child list head/count/closure, child backpointers, context. |
| `SmShell::AssertValid` | Type-specific content, connectivity, faceuse backpointers.  |
| `SmFace::AssertValid` | Surface/domain, vertex classification onto this face, singularities, coincidence, seam requirements, and degeneracy. Test 37 performs sample-based inner-loop classification and can be inconclusive; test 38 checks usable UV trim intersections and can be inconclusive; test 39 rejects a vertex shared across loops. Faceuses at LEVEL_0. |
| `SmLoop::AssertValid` | Primary loopuse, natural-boundary placement, `CheckLoop`, paired loopuse edge counts/set/order (tests 3–6). |
| `SmLoopuse::AssertValid` | Context, non-empty appropriate content. 3D closure (`IsClosed3d`) on the primary loopuse only. |
| `SmEdge::AssertValid` | Curve/interval, use parity/orientation, length, face agreement, radial angle ordering, vertex count and endpoint gaps (tests 15–17). |
| `SmEdgeuse::AssertValid` | Membership, UV curve, vertex connection. Mate/radial pair and sector tests on `SAME`-oriented manifold/spine uses. |
| `SmVertex::AssertValid` | Incident geometry placement, pointer cases, descendant vertexuses. |
| `SmVertexuse::AssertValid` | Type/target/context, cached gaps, missing seams, non-wire FU connection; broader sector check disabled. |

### A defensible validation result

Report these separately:

1. graph was traversable enough to validate;
2. `ValidatePointers` result, including presence tests 112-146;
3. root `AssertValid` result, test level, and walk mode;
4. local assertion reports by object/type;
5. supplemental checks for known gaps or build-conditional behavior;
6. whether validation refreshed caches or tolerance state;
7. source commit and relevant compile-time tolerance configuration.

## Known validator gaps and hazards

These observations are source-sensitive and must be rechecked before being reported as current
defects.

| Area | Current behavior | Consequence |
|---|---|---|
| Empty wire shell legacy branch | Required-pointer test 119 rejects a non-unknown shell whose owned-use list is null before the later wire-shell checks. A subsequent null-edge branch asserts `pE == NULL` as success and is defective dead legacy code. | Empty wire shells are rejected by test 119, not accepted by the tautological branch. `SmShell::AssertValid` also rejects a null list if reached. |
| Vertex-loop helper ordering | `StartAndEndSingleVertexLoop` permits being called before an outer loop, and `StartLoop` then marks it outer. | The completed graph fails `ValidatePointers`; callers must establish an outer edge loop first. |
| Null edge curve | `SmEdge::AssertValid` reports a null curve, then later calls `m_pCurve->ApproximateLength` without a guaranteed early return. | Arbitrarily corrupted graphs may crash rather than produce a complete report. Preflight manually before deep validation. |
| Vertex sector test | A sector gap/orientation check is commented out in `SmVertexuse::AssertValid`. | Passing validation does not establish every intended sector-order condition. |
| Embedded-loop query | `SmLoop::IsEmbedded` says it should return false on the first non-embedded edge, but that branch and the final return in `SmLoop.cpp` both return true. | The current predicate always returns true; inspect each edge with `SmEdge::IsEmbedded` instead. |
| Edgeuse assertion label | A test comment/sequence and emitted assertion index differ around the local edgeuse checks. | Diagnose by predicate and source location, not label number alone. |
| Lazy/cached geometry | Several checks skip absent optional gap/UV data or create/refresh it while validating. | Coverage can depend on cache state; rerun after forcing required derived data when investigating geometry. |
| Test/build conditionals | Legacy tolerance bounds and some expensive rules are macro- or level-dependent. | Record test level and build configuration. |
| Malformed pointer safety | `ValidatePointers` tests 112-146 report missing required links without following them and stop at the first failure. Later `ValidatePointers` checks and local `AssertValid` paths can still assume non-NULL after that gate, and a non-NULL pointer may still be stale or unmapped. | Call `ValidatePointers` on suspect files before deep validation. A presence pass does not prove neighbor identity; it only means the required slots were non-NULL. |
| `IsWireVertex` naming | Its implementation can return true for an empty edge collection, while `HasWireEdge` expresses a different question. | Do not infer attachment cardinality from the predicate name alone. |

The correct response to a gap is a supplemental check or a focused validator fix, not a claim that
the underlying model allows the malformed state.

## Worked structural examples

These examples show object cardinality and links, not complete geometry construction code.

### One open trimmed face

```text
Regions:  [Rinf]
Shells:   [Ssheet] under Rinf, type Faceuse
Face:     F with Surface P
Faceuses: F+ (SAME) and F- (OPPOSITE), both in Ssheet
Loop:     Louter
Loopuses: L+ under F+, L- under F-, both outer/SAME
Edges:    E0..En-1, each with one mated pair [EU+, EU-]
Vertices: shared cyclic endpoints
```

Each boundary edge is lamina: two individual runtime edgeuses, one logical face-edge occurrence.
The loop chains close on both faceuse sides. This is a valid open sheet form, not a closed solid.

### One manifold solid boundary

```text
Regions:  [Rinf, Rsolid]
For each boundary face F:
  one faceuse belongs to a shell in Rinf
  its mate belongs to the outer shell in Rsolid
At each ordinary boundary edge:
  two logical face occurrences
  four individual edgeuses in alternating mate/radial order
```

The shell under `Rsolid` is first, faceuse-typed, and closed. The corresponding closed shell in
`Rinf` makes the regions mutually traversable. Face orientation decides which faceuse goes to which
shell.

### Wire component

```text
Region:   Rinf
Shell:    Swire, type Edgeuse
Edges:    E0--V1--E1--V2--E2 ...
Uses:     each edge has a mated pair of shell-owned edgeuses
Vertices: endpoint vertexuses point back to those edgeuses
```

All wire edgeuses reached from the shell seed must form one connected component and point back to
`Swire`. A disconnected second component needs another shell.

### Inner vertex loop at a pole

```text
Face F already has outer edge loop L0
Second logical loop Lv is inner/OPPOSITE
  Lv+ -> VU+ -> Vertex V
  Lv- -> VU- -> same Vertex V
No edge or edgeuse belongs to Lv
```

This represents a collapsed inner boundary/singularity. Making it the face's first loop is invalid
under current validity rules, even though the constructor helper does not prevent that call order.

### Periodic seam

```text
One periodic Face F uses Edge E at two distinct UV boundary locations.
E has two logical occurrences on F:
  record/pair A at one side of the UV period
  record/pair B at the other side
Runtime E radial list has four individual edgeuses.
```

Flattening produces two `SmEUData` records, and BrepArray likewise stores two logical edgeuse
records, one for each UV-side occurrence.

The two occurrences share one 3D edge/vertices but have distinct trim placement. They are not
duplicate coincident edges.

## SMLib and USD BrepArray

For USD schema correctness, use the `$usd-brep-schema` skill and the authoritative
`source/schema/omniSolid/resources/schema.usda` layer. This section describes correspondence and
current bridge behavior; it does not add SMLib rules to the USD schema or vice versa.

### Conceptual topology crosswalk

| SMLib runtime | `SmBrepData` | USD BrepArray | Translation note |
|---|---|---|---|
| One `SmBrep` object | One indexed data object | One partition in packed arrays | USD prim can contain many BReps. |
| Ordered regions, first infinite | `m_vRegions` blocks | region counts/types | Preserve order and void classification. |
| Shell direct use kind | shell type + indices | faceuse count, wire-edge count, point type/index | USD can encode faceuses plus wires in one faceuse shell. |
| Two runtime faceuses per face | faceuse records reference one face | faceuse records/counts | Preserve orientations and shell/region incidence. |
| Two runtime loopuses per logical loop | one logical loop record | one loop record | USD stores one side plus counts/vertex sentinel. |
| Two runtime EUs per logical face-edge occurrence | one `SmEUData` pair record | one edgeuse record | Expand/collapse mate pair exactly once. |
| Edge radial circular list | signed next-member fields | next radial EU index + entry type | Signed member selection becomes explicit top/bottom entry. |
| Runtime face and wire edges share one edge class/list | one `m_vEdges` collection | separate edge and wire-edge arrays/APIs | Split on export, recombine on import. |
| All vertices share one class/list | one `m_vVertices` collection | shell points versus general vertices | Split shell vertices from general vertices; regenerate use objects. |
| Optional UV curve pointer | optional paired EU geometry | UV NURBS occurrence | Absence must be represented according to the schema, not invented sentinel semantics. |

The current bridge documents this mapping in `source/BREP_SM_USD/src/SmuConvert.cpp`.

### Runtime uses that USD does not store individually

The current USD representation does not serialize every SMLib use object:

- mate face side and loop side are implied by the logical records;
- wire edgeuses are regenerated from wire-edge records;
- shell vertexuses are regenerated from shell point data;
- vertex-loop vertexuses are regenerated from the loop's vertex index;
- ordinary edge vertexuses are regenerated from edge endpoints and edgeuse orientation.

Consequently, a translator must validate that enough information exists to reconstruct the SMLib
reciprocal graph. Successful array indexing alone is not proof that the expanded runtime graph is
valid.

### Geometry set mismatch

USD BrepArray currently has a smaller named geometry set: NURBS plus selected analytic curves and
surfaces. SMLib accepts additional runtime curve and surface subclasses. Translation therefore has
three cases:

1. preserve an exactly corresponding analytic representation;
2. preserve a B-spline/NURBS representation with correct knot/control-point conventions;
3. approximate an unsupported SMLib type to NURBS under `ApproxTol3d`, recording that the result is
   normalized rather than representation-identical.

The present SMLib-to-USD exporter preserves selected analytic surfaces but exports edge and wire
curves as NURBS in its normal path. Treat that as current implementation policy, not a schema or
SMLib limitation.

## Translation and normalization rules

### SMLib to BrepArray

1. Validate the completed SMLib BRep at full walk/test level and retain diagnostics.
2. Flatten through `SmBrepData::FromBrep`; do not derive packed counts from raw pointer addresses.
3. Preserve region and shell ordering. First region remains infinite; first non-infinite shell remains
   outer.
4. Split the unified SMLib edge list into face edges and wire edges.
5. Split shell vertices/points from ordinary vertices without losing shared endpoint indexing.
6. Emit one USD edgeuse record per logical mated SMLib pair, not per individual runtime edgeuse.
7. Convert signed `SmEUData` radial member selection to the USD next-radial index and top/bottom entry
   token after any culled-record index compaction.
8. Cull/regenerate wire, shell-point, and vertex-loop use records only where their USD parent records
   carry sufficient reconstruction data.
9. Convert SMLib analytic angular domains and cone angles from degrees to schema radians. Do not
   alter NURBS domains merely because analytic domains change units.
10. Preserve supported geometry; derive the export approximation tolerance from the maximum
    effective zone tolerance across the BRep, faces, edges, and vertices: zone tolerance ->
    `SM_ZONE_TO_XSECTTOL3D` -> `SmTol::GetApproxTol3d(sXSectTol3d)`. Use that budget for
    unsupported-type approximation and report representation loss; it is not the ambient context
    approximation tolerance.
11. Preserve face trimming intent. If unbounded SMLib face ranges are replaced by trim-derived finite
    bounds, report that as normalization.
12. Map the relevant SMLib intersection tolerance to per-BRep `brep:intersectTol3d`; do not confuse it
    with approximation tolerance.
13. Validate the resulting asset against the authoritative USD schema independently of SMLib
    validation.

### BrepArray to SMLib

1. Validate schema structure, authored types, packing, occurrence arrays, indices, units, and explicit
   topology rules before allocating a runtime graph.
2. Build `SmBrepData` with local partition-relative indices.
3. Recombine USD face edges and wire edges into the SMLib edge array while retaining shell ranges.
4. Recombine shell points and ordinary vertices into the SMLib vertex array.
5. Expand each stored logical face edgeuse into the two runtime members represented by `SmEUData`.
6. Reconstruct signed radial-member fields from next-radial index and entry type; verify the complete
   cycle and alternating orientation before runtime construction.
7. Generate wire, shell-vertex, loop-vertex, and ordinary edge vertexuses from their parent records.
8. Convert analytic angular ranges from radians to SMLib degrees; leave NURBS parameterization as
   authored unless an explicit conversion requires otherwise.
9. Convert each BRep's authored `brep:intersectTol3d` to zone-tolerance fields on `SmBrepData`,
   reconstructed edge and vertex records, and face records under `SM_USE_OLDTOL`; independently
   derive approximation tolerance. This does not reconfigure `SmContext` defaults.
10. Preserve eligible NURBS-face UV curves. Current import discards authored analytic-face UV
    curves and disables automatic UV-trim creation because stored and runtime parameter spaces
    differ. Missing trims remain null; later consumers may regenerate them through
    `GetOrCreateUVTrimCurve`. Do not treat a schema-invalid zero-order curve as equivalent to absence.
11. Construct topology, run healing only under an explicit import policy, then run full SMLib
    validation. Report healing and any downstream trim generation separately from import.

Current import entry points are `BrepMove_OneUsdBrepToSMLib` and
`BrepMove_OneUsdBrepToSMLibData` in `SmuConvert.cpp`.

### Current bridge normalizations and loss points

| Area | Current behavior to account for |
|---|---|
| Curve representation | SMLib export writes edge/wire curves as NURBS. Existing B-splines bypass approximation; other types go through `ApproximateCurve`. Import can create supported analytic curve types and canonicalizes unclamped flat-knot B-splines through `SmBSplineCurve::CreateCanonical`, including face-edge, wire-edge, and UV curves. |
| Surface representation | Plane, sphere, cylinder, cone, and torus retain analytic types. B-splines bypass approximation, though import may canonicalize unclamped knots. Export attempts to approximate other surfaces to B-spline. A null approximation is logged and skipped after face metadata has been appended; export can still return `SM_SUCCESS` with the packed surface payload absent. This path is not atomic. |
| Angular units | SMLib STEP analytic ranges use degrees; USD equations/ranges use radians. |
| Unbounded faces | Optional export policy can replace an unbounded analytic face range with bounds derived from trims. |
| Trim classification | Import maps USD `general` to SMLib `UNSURE`, while `rectangular` maps to `TRUE`; a round trip can normalize internal trim state. |
| Missing UV curves | Export omits analytic-face UV curves; import discards authored analytic-face UV curves and disables automatic trim creation. Imported trims remain null until a later consumer explicitly or lazily regenerates them with `GetOrCreateUVTrimCurve`. This is representation loss followed by possible downstream regeneration. |
| Healing | Import selects healer behavior based on source metadata/options. A healed result is not a byte- or topology-identical decode. |
| UV absence sentinel | Current export paths have historically emitted zero-valued UV NURBS entries for absence. The authoritative USD schema requires positive order for authored occurrences; audit with `$usd-brep-schema`. |

### Authoring recommendations

- Author the simplest representation that preserves intended exactness and has a direct target
  mapping.
- Keep topology identity separate from geometric coincidence; share indices/objects for real
  connectivity.
- Make periodic seams explicit before translation.
- Preserve loop/shell/region ordering because both formats attach semantics to first entries.
- State every normalization: unit conversion, analytic-to-NURBS approximation, reparameterization,
  trim creation, healing, stitching, or unbounded-domain bounding.
- Test both direction-specific conversions. A round trip can hide two compensating mistakes.
- Compare semantics after round trip: region classification, shell type/connectivity, loop type and
  order, radial cycles, shared vertices, parameter domains, geometry deviation, and tolerance use.

## Testing checklist

### SMLib-only code or validator change

- Construct through public topology APIs or an established internal Euler operation.
- Run `ValidatePointers` on the completed graph.
- Run `AssertValid` once with `SM_LEVEL_2` and `SM_WALK`; this includes the lower-level checks.
- Assert the `SmAssertArray` is empty for positive fixtures, not merely that validation returned.
- Add one focused negative fixture per changed invariant.
- Cover open, closed, and relevant non-manifold radial forms.
- Cover closed edges, periodic seams, vertex loops, wire shells, and vertex shells when affected.
- Exercise lazy UV/gap data both absent and materialized.
- Record build-conditional tolerance behavior.

### Translation change

- Validate the source under its own contract before conversion.
- Validate the target under its own contract after conversion.
- Test one and multiple packed BReps.
- Test face edges and wire edges independently and together in a faceuse shell.
- Test ordinary vertices, shared endpoints, shell points, and vertex loops.
- Test lamina (2 runtime EUs), manifold/seam (4), and spine/non-manifold (6+) radial cycles.
- Test analytic and NURBS domains, including degree/radian boundaries.
- Measure approximation error for unsupported geometry against the selected `ApproxTol3d`.
- Test absent and present UV trim curves without relying on an invalid sentinel.
- Compare first-entry semantics for region, shell, and loop order.
- Report healing or normalization separately from lossless decoding.

## Source index

### Model headers

| Topic | Primary source |
|---|---|
| Root BRep, global lists, validation APIs | `source/SMLib/inc/SmBrep.h` |
| Generic list ownership | `SmTopology.h`, `SmOwningTopology.h` |
| Regions and shells | `SmRegion.h`, `SmShell.h` |
| Faces and faceuses | `SmFace.h`, `SmFaceuse.h` |
| Loops and loopuses | `SmLoop.h`, `SmLoopuse.h` |
| Edges, edgeuses, radial model | `SmEdge.h`, `SmEdgeuse.h` |
| Vertices and incidence kinds | `SmVertex.h`, `SmVertexuse.h` |
| Orientation enum | `SmTypes.h` |
| Curves and curve types | `SmCurve.h`, `SmCurveTypes.h` |
| Surfaces and surface types | `SmSurface.h`, `SmSurfTypes.h` |
| Constructor and flattened records | `SmBrepData.h` |

### Validator and constructor entry points

| Symbol | Source |
|---|---|
| `SmBrep::ValidatePointers` | `source/SMLib/src/SmBrep.cpp` |
| `SmBrep::ValidateAndUpdateTolerances` | `SmBrep.cpp` |
| `SmBrep::AssertValid` | `SmBrep.cpp` |
| `SmBrep::AssertSubTopology` | `SmBrep.cpp` |
| `SmTopology::AssertValid` | `SmTopology.cpp` |
| `SmOwningTopology::AssertValid` | `SmOwningTopology.cpp` |
| `SmShell::AssertValid` | `SmShell.cpp` |
| `SmFace::AssertValid` | `SmFace.cpp` |
| `SmLoop::AssertValid` | `SmLoop.cpp` |
| `SmLoopuse::AssertValid` | `SmLoopuse.cpp` |
| `SmEdge::AssertValid` | `SmEdge.cpp` |
| `SmEdgeuse::AssertValid` | `SmEdgeuse.cpp` |
| `SmVertex::CheckPointers` | `SmVertex.cpp` |
| `SmVertex::AssertValid` | `SmVertex.cpp` |
| `SmVertexuse::AssertValid` | `SmVertexuse.cpp` |
| `SmBrepConstructor` implementation | `SmBrepData.cpp` |

### USD bridge

| Symbol/topic | Source |
|---|---|
| Runtime/data/USD crosswalk comment | `source/BREP_SM_USD/src/SmuConvert.cpp` |
| `BrepAppend_SMLibToUsdBrep` | `SmuConvert.cpp` |
| Import crosswalk comment | `SmuConvert.cpp` |
| `BrepMove_OneUsdBrepToSMLib` | `SmuConvert.cpp` |
| `BrepMove_OneUsdBrepToSMLibData` | `SmuConvert.cpp` |
