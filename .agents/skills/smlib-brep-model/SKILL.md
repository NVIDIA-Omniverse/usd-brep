---
name: smlib-brep-model
description: "Use for internal SMLib SmBrep model questions, validity and AssertValid audits, topology code and tests, or SmBrepData/BrepArray translation. Do not use for ordinary SM_API or Python operation usage."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib BRep Model

Use this skill when working on the kernel's runtime topology model, validators, constructors, or
translation internals. For public operation usage, use `.agents/operations/` and the API-use skills.

## Open First

Read [the canonical BRep model reference](references/brep-format.md) before answering a detailed
format question, changing topology code, auditing validation, or translating `SmBrep` data.

The reference is an index into current source, not a substitute for it. Verify the named headers,
constructors, predicates, and consuming algorithms in the current checkout when the answer affects
correctness.

## Classify Evidence

Use the reference's vocabulary consistently:

- **Model invariant:** established by the type and ownership model and relied on broadly.
- **Geometric invariant:** topology and geometry must agree within named tolerances.
- **Constructor convention:** state produced by standard builders, possibly narrower than every
  representable graph.
- **Validator enforcement:** current executable `ValidatePointers` or `AssertValid` behavior.
- **Conditional enforcement:** depends on test level, build macros, topology class, or lazy data.
- **Validator gap:** an intended rule is missing, disabled, defective, or unsafe on arbitrary
  damage.
- **Translation policy:** behavior of `SmBrepData` or `BREP_SM_USD`, not runtime permission.

When sources disagree, state the conflict. Do not redefine the model around a validator defect or
serializer shortcut.

## Defining Data and Diagnostic Evidence

An edgeuse UV trim curve is optional, derived data. The topology, edge's 3D curve and interval,
and face surface define the boundary. A wrong UV trim or UV-based loop-orientation result alone
cannot establish a topology-construction defect. Check the defining data and isolate trim
regeneration before changing topology; see [UV trim curves](references/brep-format.md#uv-trim-curves).

Tolerance reports describe geometric quality under a particular threshold; they do not by
themselves establish incorrect topology or a failed modeling operation. Distinguish pre-existing
geometry gaps, gaps introduced by construction or modification, and derived-trim errors. Measure
actual gaps, compare them with affected edge lengths, and check both normal and swapped endpoint
pairings for suspected reversals. See
[interpreting gap and tolerance reports](references/brep-format.md#interpreting-gap-and-tolerance-reports).

## Workflow

1. Identify whether the artifact is a live `SmBrep`, flattened `SmBrepData`, persisted data,
   SM_API/Python object, or USD `BrepArray`.
2. Locate the relevant type header and constructor or mutator.
3. Read the root and local validator predicates that cover the relationship.
4. Check consuming algorithms when validity is not asserted directly.
5. State the exact rule, evidence classification, current validator coverage, and any conditional
   or missing enforcement.
6. For “permissible” questions, distinguish valid representation, intermediate/legacy state,
   standard-constructor output, acceptance caused by incomplete validation, and target-format
   limitations.

Useful starting points:

| Question | Start with |
|---|---|
| Regions and shells | `SmBrep.h`, `SmBrep::AssertValid` |
| Face sides, loops, singularities, and seams | `SmFace.h`, local validators, `ValidatePointers` |
| Mate/radial edge topology and UV trims | `SmEdgeuse.h`, `SmEdge::AssertValid`, `SmEdgeuse::AssertValid` |
| Vertex incidence and vertex loops | `SmVertexuse.h`, `SmVertex::CheckPointers` |
| Flattened records | `SmBrepData.h`, `FromBrep`, `MakeTopologyFromData` |
| USD mapping | `SmuConvert.cpp`, then `$usd-brep-schema` |

## Example Requests

```text
Use $smlib-brep-model to explain the mate and radial links for a non-manifold edge.
Use $smlib-brep-model to audit whether AssertValid checks that two face loops share no vertex.
Use $smlib-brep-model to review a vertex-loop constructor change and propose focused tests.
Use $smlib-brep-model with $usd-brep-schema to map seams and wire topology without losing incidence.
```

## Validate a BRep

Treat runtime validation as staged traversal of a trusted in-process graph, not as protection
against hostile serialized data.

1. For serialized input, validate counts, discriminators, and indices under the container's
   contract before constructing runtime pointers.
2. Record build mode, tolerance macros, context, and intended topology form.
3. Run `SmBrep::ValidatePointers` first and capture its `SmAssertArray` reports. It also reports
   geometry/tolerance issues: classify each predicate rather than treating every report as a broken
   pointer. Stop before deep traversal when required links are absent or suspect.
4. Once the graph is traversable, run `SmBrep::AssertValid` once with `SM_LEVEL_2` and `SM_WALK`.
   Level 2 includes all level 0 and level 1 checks; separate lower-level calls are unnecessary.
5. Group findings by root structure, shell connectivity, face/loop trimming, radial edges, vertex
   incidence, and tolerance/geometry agreement.
6. Check the reference's known gaps, level/macro gates, and lazy-cache conditions.
7. Re-run after healing, stitching, trim generation, or cache materialization and report what
   normalization occurred.
8. Report structural validity, derived-trim correctness, and geometric quality separately.
   Interpret gap/tolerance findings in the context of the intended operation rather than treating
   every report as a construction failure.

Validation can refresh marks, gap/surface caches, and tolerance state. Do not assume it is
byte-for-byte read-only or run it concurrently on the same graph and context.

## Audit Validator Fidelity

For each proposed or existing check:

1. Name the intended model relationship.
2. Trace how constructors and topology operations establish it.
3. Compare the assertion label and comment with the executable predicate.
4. Check prerequisite null/index handling and whether failure returns before later dereferences.
5. Identify test-level, build-macro, lazy-cache, and topology-class gates.
6. Check boundary operators and the exact tolerance source.
7. Add a minimal valid fixture and a one-defect invalid fixture.
8. Classify disagreement as validator drift; do not change the model documentation merely to match
   a defective check.

## Develop Topology Code

Use an established operation that owns the reciprocal update: `SmBrepConstructor`, wire or
vertex-loop helpers, `SmEdge::AddOrientedEUPair` through an appropriate topology operation, or the
matching Euler operation.

Preserve context and intrusive-list ownership; mate/radial/CW/CCW reciprocity; orientations; shared
vertex identity; geometry ownership and domains; cache/tolerance invalidation; and region/shell
traversability. Intermediate builder states may be invalid, so validate completed operation
boundaries.

For each behavior change, add:

1. the smallest valid positive fixture;
2. a single-invariant negative fixture when validator behavior changes;
3. `SM_LEVEL_2` coverage, which includes the lower-level checks;
4. round-trip or downstream traversal coverage when representation changes;
5. relevant seam, wire, vertex-loop, radial-edge, and shell forms.

## Work Between SMLib and BrepArray

Load this skill with `$usd-brep-schema`; each representation has independent authority.

1. Write a mapping table for affected topology, geometry, parameters, and tolerances.
2. Validate the source under its own contract.
3. Separate lossless mapping from healing, approximation, generated data, and normalization.
4. Preserve occurrence counts and first-entry semantics while collapsing or expanding paired
   runtime topology.
5. Convert analytic angular units where required without changing NURBS parameter domains.
6. Rebuild and verify reciprocal topology after import.
7. Validate the target independently and test both directions; round-trip success can hide paired
   implementation errors.
8. Report every representation loss or normalization.

Do not promote current translator behavior into either format's contract.

## Boundaries

- Use `$usd-brep-schema` for authoritative USD schema questions.
- Use `smlib-kernel-operations` or its specialist skills for algorithm failures whose topology
  model is already known to be valid.
- Do not expose intrusive topology construction, `SmContext`, or validator internals as public
  SM_API/Python usage guidance.
- Do not treat validator acceptance, serializer behavior, or constructor availability as proof of
  a model invariant.
- Do not hand-edit intrusive links or union-like type/pointer fields without maintaining every
  reciprocal relationship.
- Do not infer topological connection from coincident coordinates.

## Troubleshooting

| Symptom | Response |
|---|---|
| Deep validation fails after a missing prerequisite | Stop deep traversal, preflight owning lists and links, and report the result as inconclusive. |
| `AssertValid` passes despite a source-backed violation | Identify conditional, disabled, defective, or absent coverage; do not weaken the model rule. |
| A round trip changes representation | Measure and report healing, generated trims, unit conversion, or approximation. |
| A cited line no longer contains the predicate | Find the named symbol and re-establish the claim against current source. |

## Validation

Run focused topology tests first, then use `$usd-brep-test-selection` for broader coverage. For
documentation edits, verify links, run `git diff --check`, and recheck every source-sensitive claim
against the current checkout.
