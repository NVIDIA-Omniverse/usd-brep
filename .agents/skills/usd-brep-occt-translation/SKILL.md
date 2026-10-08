---
name: usd-brep-occt-translation
description: "Debug and modify OpenCASCADE `.brep` import/export through `source/OCCT_BREP_IMPORT`, `source/OCCT_BREP_EXPORT`, USD BrepArray, and SMLib. Use for topology orientation, analytic or NURBS geometry, parameter ranges, periodic seams, pcurves, tolerances, open or mixed components, and OCCT round trips. Do not use for generic BrepArray schema serialization or confirmed kernel tessellation defects."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib OCCT BRep Translation

Use this skill for the exact-geometry and topology bridge between OpenCASCADE `.brep` files and the USD BrepArray representation consumed by SMLib.

## Boundaries

- Import follows `.brep -> OcctFileSource -> OcctStagingDriver -> USD BrepArray -> BREP_SM_USD -> SMLib`.
- Export follows `USD BrepArray -> UsdBrepToOcctModel -> OCCT writer -> .brep`.
- Use `usd-brep-data-author` when the defect is in packed BrepArray serialization, schema metadata, or applied API authoring rather than OCCT semantics.
- Use `smlib-kernel-tessellation` only after confirming that the exact SMLib BRep entering tessellation is correct.

## Open First

1. `source/OCCT_BREP_IMPORT/src/OcctFileSource.cpp` and its header for file parsing, root shapes, and source topology.
2. `source/OCCT_BREP_IMPORT/src/OcctStagingDriver.cpp` for OCCT-to-USD geometry and topology conversion.
3. `source/OCCT_BREP_EXPORT/src/UsdBrepToOcctModel.cpp` and its header for symmetric USD-to-OCCT reconstruction.
4. `tools/occt_to_usd_test/test_occt_to_usd.py` and `tools/usd_to_occt_test/test_usd_to_occt.py` for schema, validity, topology, and round-trip expectations.
5. `source/schema/omniSolid/resources/schema.usda` only when the question concerns an authored BrepArray contract.
6. `tools/scripts/smlib_gui_occt_audit.py` for generating visual corpus screenshots and reports.

## Workflow

1. Reproduce one fixture and identify the first incorrect stage: source OCCT model, staged USD arrays, SMLib ingest, tessellation, or viewport rendering.
2. Inspect source topology and geometry alongside the authored USD attributes. Do not infer translator corruption from a render alone.
3. Preserve topology membership and orientation while converting geometry. Check root/compound/solid/shell/face orientation composition and builder-specific reversal conventions.
4. Validate all aligned arrays and index ranges before changing repair logic. Edge uses must remain in loop order even when geometric edges are shared or repeated.
5. Add a focused fixture or synthetic mutation that exercises the failed contract.
6. Run the importer suite, the exporter round trip when symmetry is relevant, then the headless corpus audit for visual regressions.

## Translation Invariants

- Author finite, ordered edge and face ranges that match the represented geometry and topology. Do not force a lower bound of zero unless the geometry was explicitly reparameterized.
- Shift both ends of an angular interval together. Snap a span to a nominal full period only when it is within tolerance; otherwise preserve its phase and partial span.
- Preserve periodic seam topology, including repeated edge uses and distinct uses of one edge in a loop. A full-period geometric range does not imply a full natural face boundary.
- Treat pcurves as optional input. Ingest usable curves, but do not reject otherwise valid topology because pcurves are absent; SMLib can synthesize UV trim curves.
- Author UV trim curves only in the parameterization of the exported surface. SMLib curves projected to a NURBS approximation are not valid analytic-surface UV data.
- Preserve open shells, free faces, void shells, reversed roots, and mixed components. Do not force every component into a solid or discard topology that is not a closed solid.
- Set `brep:intersectTol3d` to the largest tolerance assigned to imported topology.
- Keep import and export fixes symmetric when they change a shared range, orientation, topology, or tolerance contract.

## Diagnosis Rules

- Missing faces can originate in malformed loops, face ranges, surface parameterization, edge-use orientation, or component membership before tessellation starts.
- Missing pcurves alone are not a defect. Check whether available pcurves are internally consistent before prioritizing them over reconstructable topology.
- Distinguish source-data limitations from translator defects and validator limitations. Record which layer first violates an invariant.
- Avoid automatic healing unless the requested behavior explicitly permits geometry or topology changes; translation should normally preserve the source model.

## Validation

Build before running converter tests:

```bash
./repo.sh build
```

Use the OCCT importer and exporter sections of `tests.sh` as the canonical environment and CI gate. A complete Linux gate is:

```bash
./tests.sh linux-x86_64
```

For visual corpus inspection after correctness tests:

```bash
uv run --locked --group gui python tools/scripts/smlib_gui_occt_audit.py \
  --input-dir TestFiles/occt_breps --output-dir /tmp/smlib_gui_occt_audit
uv run --locked --group gui python tools/scripts/smlib_gui_occt_audit.py \
  --input-dir TestFiles/occt_breps_external --output-dir /tmp/smlib_gui_occt_external_audit
```

Treat screenshots and mesh diagnostics as supporting evidence. The importer schema/`AssertValid` checks and exporter import-export-import round trips remain the correctness gates.

## Example Requests

```text
Use $usd-brep-occt-translation to debug an OCCT ellipse whose USD edge range ignores its vertices.
Use $usd-brep-occt-translation to preserve a reversed open shell through import and export.
Use $usd-brep-occt-translation to investigate a missing face that appears after loading a .brep in SMLib.
```
