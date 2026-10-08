<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Offset and Shell Internals

This maintainer reference describes the current diagnostic stages behind BRep offset and shell
operations. Public behavior and parameters belong in `.agents/operations/offset.md`.

## Pipeline

Trace failures through these stages:

1. Copy or prepare the source topology and classify selected shell openings.
2. Construct offset geometry for each retained face.
3. Resolve convex adjacency by extending/intersecting offset faces or creating fillet-like joins.
4. Resolve concave overlap according to the self-intersection option.
5. Construct edge and vertex caps or joining surfaces.
6. Merge partial topology and select the intended regions/manifold result.
7. For shelling, combine source and offset boundaries and complete the wall through the final
   Boolean/topology stage.

Primary anchors:

- `SmOffsetExecutive::OffsetBrep`
- `SmOffsetExecutive::ShellBrep`
- `SmOffsetExecutive::DoSolidOffset`
- `SmOffsetExecutive::DoExtendedOffset`
- `SmOffsetGeometryCreation.cpp`
- `SmApiOffsetBrepFull`
- `SmApiShellBrepFull`

## Option branches

`extended_offset` controls convex joins:

- `TRUE`: extend and intersect adjacent offset faces to produce sharp joins.
- `FALSE`: create fillet-like joining faces.

`self_intersection` controls concave overlaps:

- `TRUE`: intersect and trim overlapping offset faces.
- `FALSE`: leave those overlaps unresolved.

Do not describe either option as a general validity guarantee. Large offsets can collapse features,
change region topology, or fail because of curvature and nearby geometry.

## Shell-specific diagnosis

Before stepping into geometry creation, verify:

- selected faces belong to the source BRep and are remapped to the private working copy;
- source orientation and signed offset direction agree;
- opening boundaries are complete and joining surfaces can be constructed;
- the final region selection chooses the intended material side;
- temporary copies and Boolean operands follow their documented ownership contracts.

If offset faces are individually correct but the output is empty or non-manifold, inspect merge,
region selection, and final Boolean stages before changing surface-generation tolerances.

## Historical material

The archived shell-offset manual is useful for the stage model, but its capability matrices,
method counts, freeform/self-intersection guarantees, and old `SmOffsetGeometry` class name are not
current contracts. The implementation class is `SmOffsetGeometryCreation`; verify every branch
against current source.
