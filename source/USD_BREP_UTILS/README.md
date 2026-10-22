<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD_BREP_UTILS

Helpers for reading the BReps in `BrepArray` prims without a modeling kernel: finding and
loading prims, walking each BRep's topology, and evaluating its geometry. It builds on
[BREP_USD_DATA](../BREP_USD_DATA/README.md) and depends only on OpenUSD. Exporters to other
formats, such as [usd_to_occt](../../tools/usd_to_occt/README.md), use it.

| Header | Purpose |
|---|---|
| `UsdBrepStageUtils.h` | Find the `BrepArray` prims in a stage, and load or save their `UsdBrepArrayData` |
| `UsdBrepIterator.h` | `UsdBrepRange` iterates over the BReps in a `UsdBrepArrayData`; each `UsdBrepView` gives one BRep's faces, edges and vertices with local indices |
| `UsdBrepTopologyQueries.h` | Adjacency queries on one BRep, such as the faces at an edge or the edges at a vertex |
| `UsdBrepGeometryEval.h` | Evaluate points and normals on a face's NURBS or analytic surface, and its UV domain and periodicity |
| `UsdBrepKnots.h` | Convert between the schema's flat knot vectors and the knot-and-multiplicity form most formats use |
| `UsdBrepPacking.h` | Append one BRep to another `UsdBrepArrayData` |

```cpp
std::vector<std::pair<SdfPath, std::unique_ptr<UsdBrepData::UsdBrepArrayData>>> arrays;
UsdBrep::LoadAllBrepArrays("model.usda", arrays);
for (const auto& [path, data] : arrays)
    for (const auto& brep : UsdBrep::UsdBrepRange(*data))
        ;  // brep.BrepIndex(), its faces, edges and vertices
```

`tests/usd_brep_utils_test` has examples of each.
