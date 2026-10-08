---
name: usd-brep-data-author
description: "Use when authoring or debugging `source/BREP_USD_DATA` USD BrepArray translation: packed `UsdBrepArrayData` read/write, the omniSolid codeless schema, applied/multi-apply API schemas, `HasAPI` detection, and direct Sdf authoring under `SdfChangeBlock`. Do not use for kernel BRep algorithms, `_omni_solid` Python bindings, OCCT/STEP import, or higher-level USD mesh/asset export."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib USD Brep Data Author

Use this skill when changing `source/BREP_USD_DATA/`, `source/schema/omniSolid/resources/`, or behavior around `BrepArray` codeless schemas, applied API schemas, and USD read/write translation.

## Purpose

Keep the low-level `BrepArray` data bridge correct and self-consistent: packed `UsdBrepArrayData` arrays serialized in `UsdBrepWrite.cpp` and deserialized in `UsdBrepRead.cpp`, the omniSolid codeless schema in `source/schema/omniSolid/resources/`, applied/multi-apply API schema authoring and detection, and the direct Sdf authoring path inside `SdfChangeBlock`.

## When to Use

Reach for this skill when the task matches one of these patterns:

- Adding, removing, or reordering a packed field in `UsdBrepArrayData` and keeping `UsdBrepWrite.cpp` / `UsdBrepRead.cpp` in sync.
- Authoring an applied or multi-apply API schema (e.g. `BrepPointAPI:vertexPoint`) onto a `BrepArray` prim.
- A `HasAPI()` / `HasAPI(schema, instance)` call returns `false` for a Brep codeless schema you expect to be present.
- Editing `schema.usda` or `plugInfo.json` for the omniSolid codeless schema, or fixing `apiSchemaCanOnlyApplyTo` constraints.
- Writing prim spec metadata directly inside `BrepWriteToUsdStage()`'s `SdfChangeBlock`.

### When Not to Use

- Kernel geometry/topology behavior or algorithm bugs → use `smlib-kernel-operations`.
- `_omni_solid` Python bindings, stubs, or mutation semantics → use `smlib-python-api-maintain`.
- OCCT `.brep` import/export translation → use `usd-brep-occt-translation`.
- Tessellation or `UsdGeom.Mesh` asset export → use `smlib-kernel-tessellation` / the asset pipeline.

## Prerequisites

- Local SMLib checkout with `AGENTS.md` available.
- Classify the change first: **data-layout** (packed arrays), **schema** (codeless `schema.usda` / `plugInfo.json`), or **authoring path** (Sdf prim spec vs. live `UsdPrim`). The rules below differ per class.
- A built omniSolid schema plugin, or be ready to run `./prebuild.sh` to (re)generate `plugInfo.json` and `_build/schema/`.
- Use `usd-brep-test-selection` before broad validation.

## Open First

1. `AGENTS.md` for repo scope and ignored folders.
2. `source/BREP_USD_DATA/inc/UsdBrepArrayData.h` for packed array contracts.
3. `source/BREP_USD_DATA/src/UsdBrepWrite.cpp` and `source/BREP_USD_DATA/src/UsdBrepRead.cpp`.
4. `source/BREP_USD_DATA/src/UsdBrepUtilities.cpp` for Sdf authoring helpers.
5. `source/schema/omniSolid/resources/schema.usda` and `plugInfo.json` for codeless schema registration.
6. `usd-brep-test-selection` before broad validation.

## Layer Rules

- `BREP_USD_DATA` is the low-level BrepArray data serializer/deserializer. Prefer local `UsdBrepArrayData` tokens and helpers over adding new USD wrapper abstractions.
- Keep packed array semantics aligned between `UsdBrepWrite.cpp`, `UsdBrepRead.cpp`, and `UsdBrepArrayData.h`.
- Treat `source/schema/omniSolid/resources/schema.usda` as the source schema. Run `./prebuild.sh` after schema changes so `plugInfo.json` and `_build/schema/` are regenerated/copied.
- For codeless schema constraints, `apiSchemaCanOnlyApplyTo` should name the authored USD type token, such as `BrepArray`, not only a generated C++ registry type name.

## Applied API Authoring

- Inside `BrepWriteToUsdStage()`, the writer opens an `SdfChangeBlock` and works directly with `SdfPrimSpecHandle`. In this context, do not use the `USD API` such as `UsdPrim::ApplyAPI()` for writer-side schema authoring.
- When authoring applied API schemas inside the change block, update the prim spec's `UsdTokens->apiSchemas` metadata list-op with `SdfPrimSpecHandle::SetInfo(...)`. This authors the same scene description location without requiring USD prim cache updates before the change block closes.
- Use `UsdPrim::ApplyAPI()` / `CanApplyAPI()` only when operating at the live `UsdPrim` layer outside the direct Sdf authoring path.
- Multi-apply schema tokens in `apiSchemas` use `SdfPath::JoinIdentifier(schema, instance)`, for example `BrepPointAPI:vertexPoint`.

## Applied API Reading

- Reader-side detection should use USD's registered schema view, such as `UsdPrim::HasAPI(schema)` or `UsdPrim::HasAPI(schema, instance)`, after the plugin is registered and the stage is composed.
- Do not add reader fallbacks that inspect raw `apiSchemas` metadata to compensate for unregistered or incorrectly registered schemas. Fix the schema/plugin registration instead.
- If `HasAPI()` fails for a Brep codeless API, inspect `schema.usda` and `plugInfo.json` before changing reader logic.

## Troubleshooting

Match the symptom, fix the cause — do not paper over a registration bug in reader/writer code.

| Symptom | Likely cause | Fix |
|---|---|---|
| `HasAPI()` returns `false` for a schema you just wrote | Plugin not registered, or `schema.usda` / `plugInfo.json` out of sync | Set `OMNISOLID_PLUGIN_PATH=_build/schema/omniSolid/resources`, re-run `./prebuild.sh`, confirm `apiSchemaCanOnlyApplyTo` names the `BrepArray` type token |
| Applied schema missing from the written scene description | Authored via `UsdPrim::ApplyAPI()` inside the `SdfChangeBlock` | Author the `apiSchemas` list-op with `SdfPrimSpecHandle::SetInfo(UsdTokens->apiSchemas, ...)` instead |
| Read-back arrays are wrong length or misaligned | Packed-array semantics drifted between write, read, and header | Re-align token order and per-element counts across `UsdBrepWrite.cpp`, `UsdBrepRead.cpp`, and `UsdBrepArrayData.h` |
| Multi-apply instance not found by reader | Instance token not joined correctly | Build the token with `SdfPath::JoinIdentifier(schema, instance)` (e.g. `BrepPointAPI:vertexPoint`) on both sides |
| Schema edits have no effect at runtime | `_build/schema/` and `plugInfo.json` are stale | Re-run `./prebuild.sh` so the generated schema is regenerated and copied |
| Runtime "omniSolid schema plugin not registered" | `OMNISOLID_PLUGIN_PATH` unset or pointing at a stale/other-platform build | Point it at the current `_build/schema/omniSolid/resources` and rebuild that target; call `_omni_solid.usd.ensure_plugin_registered()` (or `SmApiUsdEnsurePluginRegistered()` in C) at startup to register explicitly and fail fast on a bad path |

When a fix is not obvious, narrow to the layer first (data-layout vs. schema vs. authoring path) using the Prerequisites classification, then re-read the matching `Open First` file before editing.

## Validation

Use the smallest validation that exercises the changed layer:

```bash
./prebuild.sh
./repo.sh build --build-only --release --target BREP_USD_DATA
```

For a fresh runtime check, set `OMNISOLID_PLUGIN_PATH` to `_build/schema/omniSolid/resources`, call `_omni_solid.usd.ensure_plugin_registered()` (raises if the path is wrong), then export and import a simple `_omni_solid` box and assert representative `HasAPI()` calls on the exported `BrepArray` prim.
