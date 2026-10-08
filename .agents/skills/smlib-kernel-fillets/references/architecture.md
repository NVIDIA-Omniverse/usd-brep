<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Fillet Kernel Architecture

This is internal diagnostic guidance. Public fillet/chamfer/blend semantics belong in
`.agents/operations/fillets.md`.

## Model

An edge fillet replaces local edge topology with one or more generated surfaces. Keep these axes
separate:

- **Solver/offset constraint:** constant radius, variable radius, or constant distance.
- **Cross-section generator:** linear, circular, or derivative-constrained blend.
- **Continuity:** G1, G2, or G3 constraints for blend sections.
- **Corner strategy:** how several incident fillet rails and surfaces terminate and join.

Chamfer semantics come from composing `SmConstantDistanceFS` with
`SmLinearCrossSectionFSG`. A linear generator is not sufficient by itself:
`SmApiFilletEdges` can combine `SM_FSG_LINEAR` with a constant-radius solver.

Surface-surface filleting solves directly between support surfaces. Topology/edge filleting adds
rail construction, corner resolution, trimming, and insertion into the BRep.

## Diagnostic pipeline

Trace the first incorrect stage:

```text
SmFilletExecutive
  -> selection and parameter setup
  -> fillet solver / offset constraint
  -> cross-section surface generation
  -> rail and surface intersection
  -> corner construction
  -> topology insertion and cleanup
```

Primary anchors:

- `SmFilletExecutive::SetFilletParameters`
- `SmFilletExecutive::SurfaceSurfaceFillet`
- `SmFilletExecutive::DoFilleting`
- `SmFilletIntersector::FlushCurve`
- `SmFilletCorner::Create`
- `SmConstantRadiusFS`
- `SmConstantDistanceFS`
- `SmVariableRadiusFS`
- `SmBlendCurveCrossSectionFSG::CreateSurfaceFromCurves`

## Failure classification

- **Selection:** stale or foreign edge/vertex handles, unsupported local topology.
- **Feasibility:** radius conflicts with face spans, curvature, neighboring features, or another
  fillet.
- **Solver:** rail fails to converge, crosses a singularity, rolls over, or self-intersects.
- **Generator:** cross-section or requested continuity cannot be constructed.
- **Corner:** incident rails cannot be classified or joined by the selected corner strategy.
- **Insertion:** generated geometry is valid but trimming, Boolean-style merging, or topology
  replacement fails.

Do not treat historical corner matrices as public capability guarantees. Use them only as vocabulary
for locating current dispatch, then verify the active implementation.

## Validation

For a fix, test the smallest affected edge neighborhood and record:

- selected topology and local face/edge counts;
- radius law, cross-section, continuity, and thumbweight;
- generated rail/surface count before insertion;
- final validity, manifold status, and changed topology;
- nearby negative cases that should remain rejected.

Retain a surface-surface regression when changing solver/generator behavior and an edge-based
regression when changing corner or topology insertion behavior.
