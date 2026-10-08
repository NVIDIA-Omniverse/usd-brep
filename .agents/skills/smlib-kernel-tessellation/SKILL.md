---
name: smlib-kernel-tessellation
description: "Debug SMLib BRep tessellation and immediate mesh/USD output, including `tessellate`, `SmApiTessellate`, `SmTess`, `PolyBrep`, and headless visual inspection. Do not use for translator or kernel-topology failures unless the first incorrect stage is tessellation."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Tessellation

Use this skill for exact BRep to polygon mesh conversion and the mesh workflows that immediately depend on it.

## Purpose

Diagnose BRep-to-mesh conversion failures and downstream mesh/USD issues where the first question is whether tessellation, `PolyBrep` topology, or export population introduced the problem.

## Prerequisites

- Have an input BRep, tessellation parameters, model scale, and expected mesh use case.
- Record BRep invariants before inspecting mesh arrays or USD output.
- Use `usd-brep-test-selection` for focused tessellation, mesh, or USD validation.

## Open First

1. `AGENTS.md` for architecture and search scope.
2. `.agents/operations/tessellation.md` for Python/C parameters and tolerance semantics.
3. `.agents/operations/polybrep.md` for inspecting mesh topology and properties.
4. `.agents/operations/usd_import_export.md` when USD mesh import/export is involved.
5. `.agents/docs/errors.md` for `RuntimeError` or `SM_ERR_*` diagnosis.
6. `tools/smlib_gui/headless.py` and `tools/scripts/smlib_gui_render.py` when the symptom is visual or affects a file corpus.

## Source Anchors

- Python: `source/SmPyLib/src/SmPyTessellation.cpp`, `source/SmPyLib/src/SmPyTypes.cpp`, `source/SmPyLib/src/SmPyPolyBrep.cpp`, `source/SmPyLib/stubs/usd_brep/__init__.pyi`.
- API: `source/SM_API/inc/SmApiBrep.h`, `source/SM_API/src/SmApiBrep.cpp` (`SmApiTessellate`).
- Kernel conversion: `source/SMLib/inc/SmBrep.h`, `source/SMLib/src/SmBrep.cpp` (`SmBrep::ConvertToPolyBrep`).
- Tessellator: `source/SMLib/inc/SmTess.h`, `source/SMLib/src/SmTess.cpp`.
- Mesh topology: `source/SMLib/inc/SmPoly.h`, `source/SMLib/src/SmPoly.cpp`.
- Face-specific tessellation paths: `source/SMLib/inc/SmFace.h`, `source/SMLib/src/SmFace.cpp`.
- USD mesh handoff: `source/BREP_SM_USD/inc/SmuTessellate.h`, `source/BREP_SM_USD/src/SmuTessellate.cpp`, `source/BREP_SM_USD/inc/SmuConvert.h`, `source/BREP_SM_USD/src/SmuConvert.cpp`.

## Internal Model

`SmTess::Phase1SetupBrep` tessellates shared BRep edges before face interiors so adjacent faces use
compatible boundary samples. Partial face tessellation includes faces adjacent to selected edges
for the same reason. Describe this as topology-aware, compatible shared-edge tessellation—not an
absolute crack-free guarantee for arbitrary invalid input or tolerance choices.

Curve and surface refinement have separate drivers (`SmCurveTessDriver` and
`SmSurfaceTessDriver`). Internal criteria include minimum 3D edge length and minimum UV-domain edge
ratio in addition to the options exposed through SM_API/Python. `SmViewBasedTessDriver` provides
view-direction and silhouette-sensitive refinement.

Output can become an `SmPolyBrep` or flow through `SmPolygonOutputCallback`, which receives points,
normals, UV data, source surfaces, and source faces. `SmTess` may split or modify its working BRep;
normal public conversion protects the caller by tessellating a temporary copy.

## Workflow

1. Reproduce with one BRep and explicit tessellation options.
2. Check source BRep validity and exact face/loop data before debugging mesh symptoms. For OCCT `.brep` inputs, switch to `usd-brep-occt-translation` if the staged USD or ingested BRep is already wrong.
3. Compare raw `PolyBrep` topology with downstream mesh arrays, USD output, and consistent named-view renders.
4. Trace Python/API -> `SmBrep::ConvertToPolyBrep` -> `SmTess` -> face-specific tessellation only when the source BRep is valid.
5. Add a regression that checks both topology invariants and representative mesh-array shape; use a headless render as supporting evidence for visual regressions.

## Diagnosis Rules

- Separate BRep tessellation failure from downstream `PolyBrep` processing failure.
- Record chord height, angle tolerance, max edge length, max aspect ratio, model scale, and input topology invariants.
- Inspect the source BRep before the mesh: face count, edge count, `is_manifold_solid()`, bounding box, and any analytic/periodic surfaces.
- Validate mesh output with `PolyBrep` topology queries, `is_manifold_solid()`, bounds, face counts, and `to_mesh_arrays()`.
- Treat `SmTess` as allowed to work on a copied/temporary BRep; do not infer input mutation from internal tessellation setup without checking the wrapper contract.
- For USD output, distinguish custom `BrepArray` export from standard `UsdGeomMesh` population.
- A missing rendered face is not sufficient evidence of a tessellator defect. Locate the first stage with a missing face, invalid loop, bad range, empty mesh cell stream, or incorrect actor visibility.

## Example Requests

```text
Use $smlib-kernel-tessellation to debug tessellate returning non-manifold PolyBrep output for a valid solid.
Use $smlib-kernel-tessellation to trace why USD mesh export has missing faces after BRep tessellation.
```

## Troubleshooting

- If mesh arrays are empty, verify the BRep is valid and tessellation parameters are appropriate for model scale.
- If only USD output is wrong, compare `PolyBrep` queries and `to_mesh_arrays()` before editing USD population.
- If smoothing or stitching changes topology, isolate the raw tessellation output first.
- If only imported `.brep` files fail, validate OCCT-to-USD-to-SMLib geometry and topology with `usd-brep-occt-translation` before modifying `SmTess`.

## Limitations

- Do not use for polygon boolean algorithms unless the symptom starts at BRep tessellation output.
- Do not infer input BRep mutation from temporary tessellation internals without checking wrapper behavior.

## Validation

Use `usd-brep-test-selection`. Obvious focused suites are SmPyLib tessellation tests, `prog_test` tessellation, USD mesh import/export tests, or a minimal Python repro that checks mesh arrays and topology.

For repeatable visual evidence across one file or a corpus:

```bash
uv run --locked --group gui python tools/scripts/smlib_gui_render.py INPUT \
  --output-dir /tmp/smlib_tessellation_renders --jobs 4
```

`--jobs N` runs isolated model workers concurrently; choose `N` for available
memory because each worker holds its own imported model, tessellation, and
renderer. Rerun an anomalous or timed-out file alone with `--jobs 1` before
attributing the failure to tessellation. The renderer writes the four-view
contact sheet by default; add `--save-views` only when individual views are
needed.
