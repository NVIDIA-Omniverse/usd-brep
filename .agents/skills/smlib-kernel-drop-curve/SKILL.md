---
name: smlib-kernel-drop-curve
description: "Use when debugging SMLib 3D curve-to-surface UV drop/projection failures, including drop_curve_to_surface, project_curve_to_surface, SmApiDropCurveToSrf, UV/3D segment pairing, seam/domain splits, and trim-boundary crossings. Do not use for standalone curve construction, curve-curve intersection, or unrelated modeling."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Drop Curve

Use this skill when a 3D curve must be projected into surface UV space, a UV curve must lift back to 3D, or failures appear around seams, boundaries, trimming, or paired UV/3D outputs.

## Purpose

Diagnose drop/projection failures where curves must be paired between 3D and surface UV space, especially around periodic seams, domain boundaries, trims, and tolerance-sensitive walking.

## Prerequisites

- Identify the source curve, target surface, expected parameter range, and whether boundary trimming is expected.
- Record `Surface.uv_domain()`, periodicity, and representative start/end points before changing tolerances.
- Use `usd-brep-test-selection` for focused curve or trimming validation.

## Open First

1. `AGENTS.md` for architecture and search scope.
2. `.agents/operations/curves.md` for `drop_curve_to_surface`, `project_curve_to_surface`, and `lift_uv_curve`.
3. `.agents/operations/cut_project_trim.md` when the dropped curve feeds trimming or projection workflows.
4. `.agents/docs/errors.md` for `RuntimeError` or `SM_ERR_*` diagnosis.
5. `.agents/operations/queries.md` for `Surface.uv_domain()`, evaluation, normals, and curve checks.

## Source Anchors

- Python: `source/SmPyLib/src/SmPyCurves.cpp`, `source/SmPyLib/stubs/usd_brep/__init__.pyi`.
- API: `source/SM_API/inc/SmApiCurves.h`, `source/SM_API/src/SmApiCurves.cpp`.
- Surface interface: `source/SMLib/inc/SmSurface.h`, `source/SMLib/src/SmSurface.cpp`.
- Drop operation object: `source/SMLib/inc/SmSurfaceDropCurve.h`, `source/SMLib/src/SmSurfaceDropCurve.cpp`.
- Core algorithm helpers: `source/SMLib/src/SmSurfaceDropLib.cpp`.
- Trim-curve consumers and failure state: `source/SMLib/inc/SmBrep.h`, `source/SMLib/src/SmBrep.cpp`, `source/SMLib/inc/SmEdgeuse.h`.

## Workflow

1. Reproduce with one curve and one surface, preserving both UV and 3D outputs.
2. Decide whether the expected path should stay inside the domain or split across a seam/boundary.
3. Compare `DropCurve` and `DropAndTrimCurve` responsibilities before editing algorithm code.
4. Add regression coverage that checks segment count, UV/3D pairing, and lifted sample points.

## Diagnosis Rules

- Distinguish fixed-tolerance `drop_curve_to_surface` from bbox-derived `project_curve_to_surface`.
- Preserve the output pairing: UV curves and 3D curves are related results, and multiple segments can appear when a curve crosses seams or surface boundaries.
- Record surface `uv_domain()`, periodicity, start/end curve points, curve parameter range, and representative dropped points.
- Check whether failure is from start point containment, seam jump, tangent degeneracy, normal-line distance, or wandering outside the domain.
- Use `DropAndTrimCurve` paths when curves may cross boundaries; use `DropCurve` paths for curves expected to stay inside the domain.
- Treat `SmDropCurveFail` data as useful debug state rather than persistent model data.

## Edgeuse UV Trim Diagnosis

An edgeuse UV trim is optional, derived data. A bad trim can cause UV closure or loop-orientation
failures even when topology and defining 3D geometry are correct. This differs from a UV curve
internal to an `SmCrvOnSurf` that defines the edge's 3D geometry.

1. Check use orientations, loop/vertex connectivity, edge intervals, and 3D endpoint pairing
   independently of the suspect trim. Preserve those and the face/edge geometry during diagnosis.
2. On a diagnostic copy, rebuild only the suspect edgeuse trims and rerun the affected checks.
   `GetOrCreateUVTrimCurve` and `SmFace::CreateUVTrimCurves` reuse existing trims; use
   `RebuildUVTrimCurve` or `DeleteUVTrimCurve` followed by regeneration to test a fresh drop.
   Record cache/tolerance changes separately from topology changes.
3. For seams, verify the UV side as well as the 3D lift. Both seam sides can lift correctly, and
   at poles multiple UVs describe the same vertex. Neighboring vertexuse/trim UVs can therefore
   be ambiguous. Inspect `SmEdgeuse::CreateUVTrimCurve` and `sm_RedistributeSeamUVTrimCurves`,
   including inward-binormal/cross-boundary-derivative tests away from the poles.
4. If regenerating only the trims resolves the defect with defining data unchanged, localize the
   fix to trim creation, caching, or seam-side assignment. Do not reverse valid uses to fit bad UVs.
5. For large 3D discrepancies, distinguish an inherited edge/surface gap from a bad drop. Measure
   the gap relative to edge length and check normal versus swapped endpoints before diagnosing
   orientation. Follow the canonical
   [gap/tolerance guidance](../smlib-brep-model/references/brep-format.md#interpreting-gap-and-tolerance-reports).

See [UV trim semantics](../smlib-brep-model/references/brep-format.md#uv-trim-curves) for ownership,
parameter direction, and validation details.

## Example Requests

```text
Use $smlib-kernel-drop-curve to debug a curve projection that jumps across a periodic seam.
Why does DropAndTrimCurve return two UV segments for one 3D curve?
Reduce a RuntimeError from sm.drop_curve_to_surface on a cylinder face.
Check why project_curve_to_surface returns a UV curve that lifts away from the input curve.
Diagnose a projected trim curve disappearing near a face boundary.
Compare DropCurve and DropAndTrimCurve for a path crossing a surface domain edge.
Add a regression for mismatched UV and 3D dropped segment counts.
```

## Troubleshooting

- If endpoints lift far from the input curve, compare UV domain wrapping and surface normal distance at sampled parameters.
- If a curve disappears near a boundary, check whether `DropAndTrimCurve` is required instead of plain `DropCurve`.
- If UV and 3D segment counts differ, stop and repair pairing logic before downstream trimming.

## Limitations

- Do not use for unrelated curve construction or curve-curve intersection work.
- Do not collapse split outputs unless seam/domain crossing has been ruled out.

## Validation

Use `usd-brep-test-selection`. Obvious focused coverage is a minimal Python curve/surface repro, SmPyLib curve tests, or the focused kernel suite that owns the regression path.
