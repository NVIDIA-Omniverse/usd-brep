<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD BRep Libraries: BREP_STAGING, BREP_USD_DATA, USD_BREP_UTILS

**Source:** `source/BREP_STAGING/`, `source/BREP_USD_DATA/`, `source/USD_BREP_UTILS/`
**Schema:** `source/schema/omniSolid/resources/` (codeless omniSolid `BrepArray`)

The USD BRep bridge is three **source-neutral** libraries: they contain no modeling-kernel or
CAD-importer types (no OCCT in their public API). A per-source *adapter* extracts
geometry/topology and feeds these libraries through neutral records, so the same USD `BrepArray`
serialization is shared by every importer (OCCT `.brep`, third-party CAD).

## Data flow

```text
<source geometry>  ──(source adapter)──►  neutral staging records
                                              │  BREP_STAGING (BrepStagingState + builders)
                                              ▼
                                   StagedBrepUsdWriter::WriteBrepArrayPrim()
                                              │  builds UsdBrepArrayData
                                              ▼
                                   BREP_USD_DATA  (packed arrays + omniSolid schema)
                                              │  BrepWriteToUsdStage()
                                              ▼
                                   USD  `BrepArray` prim

Read-back / analysis:  USD `BrepArray`  ──►  BREP_USD_DATA (BrepReadFromUsd*)  ──►
                       USD_BREP_UTILS (UsdBrepView iteration, geometry eval, topology queries)
```

## BREP_STAGING — staging model + USD writer

Owns source-neutral BREP staging records, builders, planners, and the USD writer. See
`source/BREP_STAGING/README.md`: this package must not include source-specific (CAD importer) headers;
source-specific extraction lives in an adapter that passes data in through source records.

Key headers (`source/BREP_STAGING/inc/`):
- `BrepStagingState.h` — the staged model: per-face `sUVDomain`, faces/faceuses/loops/edgeuses,
  shells/regions, surfaces/curves/vertices. This is what the writer serializes.
- `BrepStagingBuilder.h` — the generic builder the adapter drives (`AddFace`,
  `AddFaceSurfaceForCurrentFace`, loop/edgeuse/shell/region emission).
- `BrepTopologyBuilder.h`, `BrepEdgeVertexBuilder.h`, `BrepCoedgeBuilder.h`,
  `BrepFaceTraversalPlanner.h`, `BrepNaturalBoundaryBuilder.h`, `BrepExtentBuilder.h`,
  `StagedBrepTransform.h` — topology assembly, natural boundaries, extents, unit/placement transforms.
- `BrepSourceData.h` — the record structs an adapter populates.
- `BrepStatus.h` — `BrepStatus` result codes (`IsBrepStatusSuccess`).
- `BrepUsdWriter.h` — `StagedBrepUsdWriter::WriteBrepArrayPrim(stage, rootPath, source, pError, pOutPath)`
  authors a uniquely named `BrepArray` prim under `rootPath` from the staged model. `source` tags
  `customData["source"]`; optional `pOutPath` returns the created prim path (so callers can attach
  appearance/metadata to the exact prim afterward).

Adapter responsibilities (what the staged model needs to be *valid*):
- Per-face UV domain (`sUVDomain`) as the face's parametric bounding box.
- Consistent orientation across the manifold: loop windings, `faceuse` orientation, and `edgeuse`
  orientation must agree so radial edgeuse partners resolve within the same shell and regions
  (solid/void) classify correctly. Inconsistent orientation passes schema checks but fails geometric
  `AssertValid` (radial/shell/region errors).

## BREP_USD_DATA — packed BrepArray serializer + omniSolid schema

The low-level `BrepArray` data bridge: packed arrays and the codeless omniSolid schema. See the
`usd-brep-data-author` skill for authoring rules.

Key headers (`source/BREP_USD_DATA/inc/`):
- `UsdBrepArrayData.h` — the packed array contract (surfaces, curves, faces, faceuses, loops,
  edgeuses, edges, vertices, shells/regions, plus `face:range`, `edge:range`, …). Layout must stay in
  sync between write and read.
- `UsdBrepBuilder.h` — `UsdBrepData::UsdBrepBuilder`, builds a `UsdBrepArrayData`.
- `UsdBrepWrite.h` — `BrepWriteToUsdStage(arrays, prim)` authors the packed arrays + applied API
  schemas onto a `BrepArray` prim inside an `SdfChangeBlock`.
- `UsdBrepRead.h` — `BrepReadFromUsd*` deserializes back into `UsdBrepArrayData`.
- `UsdBrepTokens.h`, `UsdBrepUtilities.h`, `UsdBrepDiagnostics.h` — tokens, Sdf helpers, logging.
- `UsdBrepHeaders.h` — pxr warning-suppression guard; include before `<pxr/...>`.

Schema/authoring notes:
- Applied/multi-apply API schemas are authored via the prim spec's `apiSchemas` list-op inside the
  change block (`SdfPrimSpecHandle::SetInfo(UsdTokens->apiSchemas, ...)`), not `UsdPrim::ApplyAPI()`.
- Multi-apply instance tokens use `SdfPath::JoinIdentifier(schema, instance)`
  (e.g. `BrepPointAPI:vertexPoint`); readers detect with `HasAPI(schema[, instance])`.
- The codeless omniSolid schema must be registered before the stage is traversed: set
  `OMNISOLID_PLUGIN_PATH` (or `PXR_PLUGINPATH_NAME`) to the built `schema/omniSolid/resources`, and
  re-run `./prebuild.sh` after editing `schema.usda` / `plugInfo.json`.

## USD_BREP_UTILS — kernel-agnostic read-back / query helpers

Consume an authored `BrepArray` without any modeling kernel. Pure representation + query helpers.

Key headers (`source/USD_BREP_UTILS/inc/`):
- `UsdBrepIterator.h` — `UsdBrepView` (lightweight view of one brep) + `UsdBrepIterator`/`UsdBrepRange`
  for range-based iteration over breps and their topology in a `UsdBrepArrayData`.
- `UsdBrepGeometryEval.h` — point evaluation for analytic + NURBS geometry via `UsdBrepView`.
- `UsdBrepTopologyQueries.h` — `UsdBrepTopologyQueries`: lazily built adjacency tables
  (faces-at-edge, edges-of-face, radial edgeuses, vertex adjacency, …).
- `UsdBrepStageUtils.h` — `FindBrepArrayPrims()` and loading BrepArray prims from a stage.
- `UsdBrepPacking.h` — `AppendBrepToArray()` pack/unpack of `UsdBrepArrayData`.
- `UsdBrepKnots.h` — flat↔compact NURBS knot-vector conversions (USD "flat" form ↔ the compact
  (knot, multiplicity) form most kernel file formats use).

## Library dependencies / linking

- `USD_BREP_UTILS` and `BREP_USD_DATA` are shared libraries; `BREP_STAGING` is a static archive that
  depends on `BREP_USD_DATA`. When linking into a consumer, list the static `BREP_STAGING` **before**
  the shared `BREP_USD_DATA` it references.
- Any process that opens a `BrepArray` (write or read) must have the omniSolid schema registered
  (`OMNISOLID_PLUGIN_PATH` → `schema/omniSolid/resources`).

## Gotchas

- `face:range` is authored by the source adapter (`sUVDomain`), not recomputed from pcurves by
  staging. If the adapter can't recover a face's full UV extent (e.g. a planar face whose boundary
  edges lack pcurves), the domain can collapse in one direction and fail the schema `face:range`
  (Vmax > Vmin) rule — project the boundary into the surface frame to recover it.
- Packed-array layout in `BREP_USD_DATA` must match between `UsdBrepWrite.cpp`, `UsdBrepRead.cpp`, and
  `UsdBrepArrayData.h`; never reorder fields on one side only.
- Schema validity (BA.xxx rules) and geometric validity (`AssertValid`: radial edges, shells,
  regions, loop orientation) are independent gates — passing one does not imply the other.

## See also

- `.agents/skills/usd-brep-data-author` — authoring/debugging the BREP_USD_DATA bridge + schema.
- [USD import and export](../operations/usd_import_export.md) — higher-level
  USD import/export operations.
- `source/BREP_STAGING/README.md` — staging package scope/constraints.
