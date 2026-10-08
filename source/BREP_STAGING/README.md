<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# BREP_STAGING

`BREP_STAGING` owns source-neutral BREP staging records, builders, planners, and UsdBrep writing helpers.

This package must not include source-specific (CAD importer) headers or types. Source-specific extraction should live in its own adapter and pass data into this package through source records.

## Adapter conventions

- Store UV curves in edgeuse traversal order. Reverse edge-directed source trims
  for `opposite` edgeuses; closure checks use stored endpoints without another reversal.
- Missing UV: leave the staging record invalid. Its `(order, vertexCount) = (0, 0)`
  placeholder contributes no CVs, weights or knots; do not invent a trim chord.
- Preserve distinct UV curves for both uses of a seam edge. A loop collapsed to
  a pole uses `loop:edgeuseCount = 0` and `loop:vertexIndex`, not zero-length edges.
  The generic source-record path cannot author that today: `SourceLoopData` carries
  no pole vertex, and `AddSourceLoopForCurrentTraversal` hardcodes
  `uiVertexIndex = 0`. Build such loops as `StagedLoopData` directly until the
  source record and helper carry an explicit vertex mapping.
- See the [schema](../schema/omniSolid/resources/schema.usda) for knot counts and surface CV layout.

## Validation

Gate adapter conformance on the **unhealed** path. Run
[schema checks](../../tools/brep_validator_cli/README.md), then
[geometry validation](../../tools/brep_geometry_validator/src/brep_geometry_validator_main.cpp)
with `--level 2 --no-healer`, then
[smtess_usd](../../tools/smtess_usd/src/smtess_usd_main.cpp) **without** `--heal`,
requiring the expected body count, nonempty meshes and no failed faces.

Healing must not be part of that gate. For a non-SMLib source, import can run
`MakeTopologyFromData` followed by `HealBrep(SM_HO_ALL)`, whose steps merge
coincident vertices, remove degenerate faces and edges, move or split seams and
faces, and repair sheets, tolerances and region state. Those are exactly the
defects a staging adapter introduces, so a healed run can pass on output the
healer rewrote and prove nothing about staging. Loop repair is not in that list:
`Fix_BadLoops` is deliberately disabled because its classification and repair are
unreliable, so loop defects survive healing rather than being corrected.

Run the healed path afterwards as a production-path integration test. When the
source data is intentionally dirty, report the unhealed failure separately rather
than letting healing stand in for adapter correctness.

Diagnose the first failing stage; schema success alone does not establish
geometric validity.
