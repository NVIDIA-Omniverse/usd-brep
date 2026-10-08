---
name: smlib-kernel-healing
description: "Use when SMLib BRep healing (heal_brep, HealBrep, or healing during USD import) crashes, reports success but leaves gaps, seams or poles unrepaired, changes a model unexpectedly, or works on stale data after topology edits, and when changing the healer's stages or change tracking. Not for how to call heal_brep, failures not yet traced to healing, or curve knot removal."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Healing

## Purpose

Find the first incorrect boundary in the BRep healer's sequence: target selection, measurement,
classification, mutation, notification, property refresh, or the next consumer. Use this skill for
stale healer state, crashes, and incomplete repairs in `SmHealData` or import healing.

## Prerequisites

- Start with a reproducible BRep or an already isolated kernel path. Record the first incorrect
  topology, geometry, property cache, or problem flag.
- Identify the entrypoint and requested stop stage. Also record `bMakeUVTrimCurves`, the import
  healer version when applicable, and whether the failure occurs in debug, release, or both.
- Read [the healer design reference](references/architecture.md) before changing stage order,
  property ownership, invalidation, or notification handling. Verify its source-sensitive claims
  against the checkout being edited.

## References

- Use [the operation guide](../../operations/heal_import.md) for public operation context.
- For a Python `RuntimeError`, read [the error decoder](../../docs/errors.md).
- Use `smlib-brep-model` when the topology invariant itself is unclear; use
  `smlib-kernel-drop-curve` when the first failure is curve-to-surface projection.

## Diagnostic Workflow

1. Identify the entry: explicit `heal_brep`, `SmBrep::HealBrep`, direct `SmHealData`, or import
   through `MakeTopologyFromData`. Record the requested stop stage, UV-curve option, import
   healer version, and actual stages reached.
2. Find the first disagreement between live topology, cached `*Props`, problem flags/lists, and
   the attempted-repair result. A successful return or a nonempty problem list alone is ambiguous.
3. For topology edits, trace the owning mutator's `Notify` call through
   `SmTopologyChangeCallback::Execute` and `SmHealData::UpdateTargetLists`. Inspect event payload
   roles, removal/addition lists, and the stage used to rebuild properties.
4. For stale state, inspect both the `*Props` lifetime and the topology pointers inside its
   classifications. Reacquire properties after refresh; a surviving face can have new properties.
5. Reproduce with deterministic geometry and exercise the actual mutator plus the healer
   consumer. Synthetic notifications help isolate dispatch but do not establish complete coverage.
6. Report the first failing boundary with its source symbol, observed state, and expected state.
   After a fix, rerun the same input and the next consuming stage. If the requested repair is
   unimplemented or disabled, explain that limitation instead of treating success as a repair.

## Change Rules

- Let the owning topology operation emit a completed edit notification. Maintain tracking lists
  and their debug index arrays through the callback; do not repair missing notifications with
  caller-specific list additions.
- Stop tracking before `UpdateTargetLists`: the system callback slot is not nestable, while
  refresh can recursively heal newly added targets.
- Treat refresh as a point where old property pointers and iterators can become invalid. Keep a
  batch's tracking active through its edits, then refresh; when an algorithm requires per-item
  refresh, recheck queued identities before dereferencing them.
- Do not assume `ChgList` rebuilds caches or that `HasProps` detects changes to dependencies.
- Preserve the distinction between implemented fixes, disabled dispatch, and successful stubs.
  Consult the design reference's incomplete-repair inventory. Keep `Fix_BadLoops` disabled until
  broken-loop classification and repair are validated; helper tests alone do not justify enabling it.
- USD import currently leaves CAD provenance empty, so `heal=True` also heals SMLib-authored USD.
  Provenance semantics and source-based skipping remain undecided; do not infer them from the
  dormant `m_sCADSource` branch.

## Example Requests

```text
Use $smlib-kernel-healing to find why heal_brep crashes on this imported STEP solid.
heal_brep returns success, but the seam gaps and the missing pole loop are still there. Why?
Importing our own exported USD with heal=True changes the topology. What is the healer doing?
After Fix_MoveSeam the next stage reads face properties that no longer match the face.
I added a new topology operation; how do I make the healer track what it removes and adds?
UpdateTargetLists asserts in a debug build after an edge split. Trace the stale entry.
```

## Troubleshooting

| Symptom | Check first |
|---|---|
| A topology edit produces no tracker entry | Read the owning mutator's completed `Notify` call and verify its payload roles. |
| The event is present but `*Props` remain stale | Confirm tracking stopped before `UpdateTargetLists`, then check remove/add membership and the rebuild stage. |
| Healing returns success but the defect remains | Check whether the stage is dispatched, stubbed, or reports repaired history; inspect the live problem flag. |
| Refresh crashes or debug arrays diverge | Look for retained property pointers, dead topology, direct tracker-list edits, or mismatched debug index arrays. |
| A repeated run behaves differently | Reproduce from a fresh BRep and `SmHealData`; populated state can retain or duplicate list entries. Topology hash tables can reorder events, changing outcomes. |

### Errors and recovery

- For a `RuntimeError` naming `SmApiHealBrep`, decode the symbolic status and `Kernel trace:`
  using the error guide. Follow the deepest relevant failing call; distinguish a wrapper argument
  failure from a failure inside a dispatched repair. Preserve the original input and reproduce on
  a copy because healing can mutate state before returning an error.
- For `SM_SUCCESS` with bad output, inspect the requested stage, `m_eDoneHealOp`, and the
  remaining live problem flags. Check the incomplete-repair inventory before investigating a
  notification bug. `SmBrep::HealBrep` and recursive refresh can discard nested status; successful
  outer status alone is insufficient evidence.
- For an assertion or crash in `UpdateTargetLists` or `StopTracking`, first check tracker activity
  and context ownership. Stop tracking before recursive refresh. Then check list/index-array
  alignment and reacquire properties from current targets; never inspect a removed object by
  dereferencing its historical event pointer. Reproduce with debug diagnostics to verify the fix.

## Worked Examples

### “Moving the seam succeeds, but the next seam split crashes.”

Trace `Fix_MoveSeam` → `SmFace::MoveSeam` → completed topology notification → callback lists →
`UpdateTargetLists` → `Fix_SplitEdgesAtSeam`. Record the face identity, old/new property identities,
and property stage at refresh. During normal `Fix_MoveSeam`, catch-up is clamped to the preceding
`SM_HO_CACHE_FACEPROPS_2`; replacement properties need the surface and seam classifications from
that stage. Verify the next consumer uses current properties. An emitted event alone does not prove
that refresh rebuilt the data it needs.

### “Healing returns success, but gaps and missing pole loops remain.”

First distinguish measured tolerance changes from geometric gap closure. `Fix_BadGaps` is a no-op,
and `Fix_BadLoops` is disabled in `HealTgts`. Check which stages actually ran before blaming stale
caches. Report unsupported repairs explicitly. A direct missing-pole helper regression can test
that helper, but does not justify enabling general broken-loop repair.

### “Why does heal=True modify USD that our own kernel exported?”

Follow `UsdBrepRead.cpp` → `m_sCADSource` → `SmuConvert.cpp`'s healer-version assignment →
`MakeTopologyFromData`. The ordinary USD path leaves the source string empty, selecting an older
healer version even for SMLib-authored files. Compare fresh imports with healing off/on and report
the topology/tolerance changes. Explain the current provenance limitation; do not invent a source
marker or skip policy. SMB's stored healer version is a separate input contract.

### “A new topology mutator needs healer tracking.”

Inspect its real `Notify` payload and completion boundary, then trace the callback's remove/add
lists and debug indices. Exercise the mutator followed by refresh and a consumer of the rebuilt
properties. Test surviving identities as well as removed targets: a face can retain its address
while its property object is replaced. Keep target maintenance in the callback mechanism.

### Requests to route elsewhere

“How do I call heal_brep in Python?” uses `smlib-python-api-use` and the operation guide.
“Remove redundant knots from this standalone curve” uses the curve operation/API guidance.
“Projection fails before healing starts” uses `smlib-kernel-drop-curve`. These requests do not
require the healer's stage and invalidation workflow unless evidence localizes a failure there.

## Boundaries

- Use `smlib-python-api-use` for ordinary `heal_brep` calling questions without a kernel defect.
- Start with `smlib-kernel-operations` when the failing operation has not yet been localized to
  healing.

## Validation

For kernel changes, use `usd-brep-test-selection`; start with `prog_test` suite `topology`, adding
`trimmed-surfaces` or `brep-import` when affected. Run debug coverage for notification/index-array
changes and relevant release coverage. Compare generated debug logs as well as exit status.

Useful existing regressions in `tests/prog_test/src/smTopoTest.cpp`:

- `my_test_tracks_changed_objects`, `my_test_tracks_nested_geometry_and_resets`
- `my_test_move_seam_refreshes_face_props`
- `my_test_missing_poles_refreshes_all_faces`, `my_test_squeeze_edge_tracks_removal`

Check repaired geometry/topology, rebuilt property stage and ownership, remaining problem flags,
and repeated-operation behavior. Validate the completed graph using the BRep model skill's
pointer and `AssertValid` guidance. For documentation-only changes, validate skill metadata,
links, and source-sensitive claims; a kernel rebuild is unnecessary.
