<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Healing

**Source:** `source/SM_API/inc/SmApiHeal.h`
**Python:** `_omni_solid.heal_brep`. Also integrated into USD import (`heal` parameter).


**Kernel internals:** [BRep healer design](../skills/smlib-kernel-healing/references/architecture.md)
and [healing skill](../skills/smlib-kernel-healing/SKILL.md) cover stage dependencies, cached
properties, topology notifications, and current repair limitations.

## Overview

BRep healing attempts supported repairs to geometric and topological defects in BRep models. These operations are most commonly needed when receiving geometry from external sources — other CAD systems, file format converters, or procedural generators — where small inconsistencies are common.

## Operations

### SmApiHealBrep / heal_brep

Run the BRep healer on a BRep to attempt supported repairs.

| Parameter | Type | Description |
|---|---|---|
| `brep` | Brep | BRep to heal (modified in-place) |

**Implemented repairs include:**
- Coincident-vertex merging and degenerate face/edge removal
- Tolerance adjustment from measured gaps
- Periodic seam movement and seam-related edge/face splitting
- Selected sheet and region organization repairs

**Limitations:** General geometric gap closure, coincident-edge repair, and missed edge-intersection
repair are unimplemented. Uncontained-edge handling only adjusts cached state. Broken-loop repair
remains disabled because its classification and repair are unreliable. A successful return does not
establish that every defect was repaired; validate the resulting topology and geometry.

**Design notes:**
- USD import healing defaults on in C (`SmApiUsdImportBreps` / `SmApiUsdImportBrep`) and off in
  Python (`heal=False`). When enabled it runs on **every** imported Brep, including SMLib-authored
  USD: the public bridge neither preserves the CAD-source marker on export nor reads it on import,
  so the source is always empty and the healer is never skipped. See  [USD import/export](usd_import_export.md).
- For programmatically created geometry (using SM_API primitives and booleans), healing is usually unnecessary — the kernel produces clean geometry.
- Healing is most valuable for imported geometry from external sources.

## When to Heal

| Source | Heal? | Rationale |
|---|---|---|
| Created with SM_API | Usually no | Kernel-generated geometry is clean |
| Imported from USD (external or known-problem source) | Consider `heal=True` in Python | Attempt supported repairs; healing changes topology and tolerances and does not provide general gap closure |
| Imported from USD (SMLib-authored) | Usually no | Already clean, and healing is not skipped automatically |
| Boolean results | Usually no | Booleans produce valid topology |
| After manual topology edits | Yes | Human edits may introduce inconsistencies |
