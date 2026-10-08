---
name: smlib-kernel-solvers
description: "Use for failures inside SmGlobalSolver, SmLocalSolve1d/Nd, surface-point solvers (SmSurfPtGlobalSolver / SmSPGlobalSolver), SmTopologySolver, or SmPolySolver: success without an answer, pruned branches, subdivision/convergence, Jacobian/bounds, tangency/singularity, or duplicate solutions. Do not use for API usage, fillet-specific solving, drop-curve tracing, or failures not localized to a kernel solver."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Solvers

## Purpose

Use this skill for failures rooted in `SmGlobalSolver`, `SmLocalSolve1d`, `SmLocalSolveNd`,
`SmSurfPtGlobalSolver`, `SmSPGlobalSolver`, `SmTopologySolver`, or `SmPolySolver`. Public users
should start from `.agents/operations/`.

## Prerequisites

- Have a minimal failing operation and identify its curve, surface, topology, or polygon entrypoint.
- Record input domains, tolerances, operation type, solution request, and expected result.
- Use a checkout that can run the smallest owning-layer test or `prog_test` case.

## Open First

1. Read [the solver reference](reference.md).
2. Open `SmCoreTypes.h` and identify the exact `SmSolverOperationType` and solution request.
3. Open the owning entrypoint in `SmCurve`, `SmSurface`, `SmTopologySolver`, or `SmPolySolver`.
4. Follow the implementation selected by that entrypoint:
   - for `SmGlobalSolver`, inspect its traversal and concrete `LocalSolve`; if that method
     instantiates `SmLocalSolve1d` or `SmLocalSolveNd`, inspect that solver;
   - for surface-point solving, inspect dispatch in `SmSurface::GlobalPointSolve`: the default
     new-solver path uses `GlobalPointSolve_1` / `SmSurfPtGlobalSolver` in
     `SmSurfacePointSolve.cpp`; ordinary maximize uses `GlobalPointSolve_0` / `SmSPGlobalSolver`
     in `SmSurface.cpp`, as do other operations when the new solvers are disabled, except signed
     directed maximize, which always selects the new path;
   - for polygonal paths, inspect `SmPolySolver::SolveBranch` and its local solve.

## Workflow

1. Verify that the owning entrypoint accepts the operation; enum membership alone does not imply
   support.
2. Record requested solution type, tolerance, target distance, intervals, periodicity, initial
   guess, and optional-vector layout, then verify which inputs the selected implementation consumes.
3. After confirming the entrypoint uses `SmGlobalSolver`, set breakpoints or temporary diagnostics
   in `SolveIt`, `BranchMayContainAnswers`, `AreReadyForLocalSolve`, `Subdivide`, and `LocalSolve`
   to find the first wrong decision.
4. At the local layer, record evaluator residual or solution size where exposed; otherwise record
   the path-specific objective or geometric distance. Then record the returned status,
   `rbFoundSolution`, termination reason, final parameters, boundary hits, and Jacobian state. Treat
   found accuracy and termination state as path-specific evidence, not uniform solver outputs.
5. Classify the first wrong decision: branch pruning, insufficient subdivision, poor initial guess,
   Jacobian failure, bounds handling, convergence rejection, or solution deduplication.
6. For intersections, distinguish a crossing, tangent point, tangent curve, intersection
   singularity, coincident range, and degenerate surface parameterization.
7. Add the smallest regression at the owning layer rather than testing only through a distant
   operation.

## Diagnosis Rules

- Successful `SmStatus` does not prove convergence; inspect the found-solution flag and termination
  reason.
- Desired accuracy is full convergence. Acceptable accuracy can produce
  `SM_TR_FOUND_ANSWER_CLOSE`.
- An unsolvable ND Jacobian initially sets `SM_TR_BAD_JACOBIAN`; final residual checks can
  overwrite it. Inspect the final reason together with the residual and execution path. The 1D
  solver instead substitutes a derivative direction when the derivative is near zero; diagnose the solver family before
  interpreting the termination reason.
- Bounds handling differs by solver. A bounded 1D solve calls `SE(SM_ERR_INVALID_INPUT)` in debug
  when its initial guess lies outside the interval expanded by `SM_EFF_ZERO_SQRT`, then clamps and
  continues. ND passes the initial guess to the evaluator without solver-side clamp or wrap; when
  intervals are supplied, it constrains later guesses.
- `rbNeedsMoreSubdivision` is part of the local/global contract.
- `SM_SR_ALL` filtering is implementation-specific. `SmGlobalSolver::AddSortedSolution` keeps
  min/max answers within tolerance of the best, while `SmPolySolver` can retain older non-global
  extrema because its cleanup pass is disabled.
- Solution identity can include type, object identity, parameter counts and values, and
  solver-specific overrides—not only geometric position.
- In 1D Newton paths, found accuracy is useful, but some Brent success exits return without setting
  either field, leaving the termination reason at its initialized `SM_TR_UNABLE_TO_CONVERGE` and
  `m_dFoundAccuracy` at its initialized `SM_UNDEF_DOUBLE` or a value stale from an earlier
  iteration. In ND paths, do not use `m_dFoundAccuracy`; `SolveIt` never updates its initialized
  `0.0` value, which reads as a plausible converged accuracy rather than an obvious sentinel. An
  initial evaluator failure can return success with `rbFoundSolution` true and
  `SM_TR_UNABLE_TO_CONVERGE`, and final residual checks can replace `SM_TR_BAD_JACOBIAN` with
  `CONVERGED` or `CLOSE`.

## Tangency and Singularity

Treat these as separate diagnostic states:

- **Crossing:** intersection branches cross with independent directions.
- **Tangent point:** isolated touch without a nearby intersection branch.
- **Tangent curve:** contact continues in opposing directions.
- **Intersection singularity:** multiple intersection branches meet.
- **Coincidence:** geometry agrees over a range.
- **Surface singularity:** the parameterization degenerates, such as at a pole.

A surface singularity and an intersection singularity are independent. For surface/surface
classification, duplicate starts, and bidirectional restarts, trace `SmSurfaceIntersector` and
`SmAdvSurfaceIntersector`. Reserve `SmSurfaceTracer` for single-surface drop, projection, and
silhouette tracing.

## Example Requests

```text
Use $smlib-kernel-solvers to explain why a surface/curve solve returns success without a solution.
Use $smlib-kernel-solvers to find where a valid intersection branch is pruned.
Diagnose SM_TR_BAD_JACOBIAN near a surface pole.
Why does an SM_SR_ALL minimum query return only tied global minima?
Trace duplicate surface/surface starts through AddStartPoint and IsPointOnCurve.
Add a regression for a local solve that reaches acceptable but not desired accuracy.
Why does an out-of-range bounded 1D guess emit a debug diagnostic and get clamped before solving?
Use $smlib-kernel-solvers to distinguish branch pruning from local convergence rejection.
```

## Troubleshooting

| Symptom | Next step |
|---|---|
| Success status but no answer | Inspect `rbFoundSolution`, termination reason, and evaluator residual. |
| Expected branch never reaches local solve | Check operation-specific pruning, readiness, and bounding data. |
| Repeated subdivision without convergence | Inspect initial guesses, bounds, Jacobian quality, and `rbNeedsMoreSubdivision`. |
| Surface/surface tangent result disappears or duplicates | Check `SmSurfaceIntersector::AddStartPoint`, `IsPointOnCurve`, restart direction, and direct appends in `SmAdvSurfaceIntersector`. |
| Polygon and analytic paths disagree | Verify each owning entrypoint's supported operation matrix before comparing algorithms. |

## Limitations

- Keep fillet-specific solving in `smlib-kernel-fillets`.
- Keep drop-curve tracing in `smlib-kernel-drop-curve`.
- Do not present solver constructors, evaluator callbacks, or internal tolerances as SM_API/Python
  usage.
- Do not infer support or behavior from the archived manual without checking current dispatch.

## Validation

Cover accepted-close and rejected convergence when tolerance behavior changes. For
tangency/singularity fixes, verify branch count, point classification, continuation directions,
coincident ranges, and solution deduplication.
