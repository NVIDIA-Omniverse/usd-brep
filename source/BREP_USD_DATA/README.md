<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# BREP_USD_DATA

Reads and writes `BrepArray` prims. It is the base layer that every USD BRep converter
builds on, and depends only on OpenUSD, not on SMLib or any other modeling kernel.

A `BrepArray` prim holds many BReps in one set of flat arrays, one array for each
topology and geometry attribute in the [schema](../schema/README.md). Objects refer to each
other by index rather than by pointer. `UsdBrepArrayData` holds those arrays in memory,
one-to-one with the prim's attributes.

| Header | Purpose |
|---|---|
| `UsdBrepArrayData.h` | `UsdBrepArrayData`, the in-memory copy of a `BrepArray` prim; its header comment explains the array layout |
| `UsdBrepRead.h`, `UsdBrepWrite.h` | `BrepReadFromUsdStage` and `BrepWriteToUsdStage` copy between a prim and `UsdBrepArrayData` |
| `UsdBrepBuilder.h` | `UsdBrepBuilder` appends BReps to `UsdBrepArrayData` one region, shell, face, loop, edge and vertex at a time |
| `UsdBrepUtilities.h` | `RegisterOmniSolidResourcesPlugin` registers the schema from `OMNISOLID_PLUGIN_PATH`; call it before opening a stage |
| `UsdBrepTokens.h` | The schema's attribute and value tokens |

Libraries built on it:

- [BREP_STAGING](../BREP_STAGING/README.md) builds `UsdBrepArrayData` from another format's BReps
  (the import side).
- [USD_BREP_UTILS](../USD_BREP_UTILS/README.md) iterates, queries and evaluates BReps in
  `UsdBrepArrayData` (the export side).
- `BREP_SM_USD` converts between `UsdBrepArrayData` and SMLib BReps; it is what
  [SM_API_USD](../SM_API_USD/inc/SmApiUsd.h) uses.
