<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Solver Reference

This is a maintainer reference for current solver implementation. It adapts useful concepts from
the archived SMLib solver manual but treats current dispatch and source behavior as authoritative.

## Solver layers

| Layer | Responsibility | Primary source |
|---|---|---|
| `SmGlobalSolver` | Traverse spatial trees, prune branches, choose subdivision, invoke local solves, and collect solutions. Storage allows three trees; tree-taking constructors initialize one or two, the default constructor initializes zero, and the documented member invariant says one or two. | `SmGlobalSolver.h/.cpp` |
| `SmLocalSolve1d` | Newton-style one-dimensional solves with bracketed and min/max alternatives | `SmLocalSolve1d.h/.cpp` |
| `SmLocalSolveNd` | Modified Newton-Raphson over bounded or periodic parameter vectors | `SmLocalSolveNd.h/.cpp` |
| `SmTopologySolver` | Adapt global solving to vertices, edges, faces, shapes, and BReps | `SmTopologySolver.h/.cpp` |
| `SmPolySolver` | Solve against polygonal topology with a narrower operation set | `SmPolySolver.h/.cpp` |

## Operation and result types

Start with:

- `SmCoreTypes.h`: `SmSolverOperationType`, solution requests, and sorting keys.
- `SmSolutionArray.h`: node, point, range, and polygon solution records.

Do not infer implementation from the enum alone. Current caveats include:

- `SM_SO_INTERSECT_WIREFRAME` remains in dispatch for selected topology paths but is marked as no
  longer used by ordinary SMLib curve solving.
- `SM_SO_ANGLE_MINIMIZE` is declared but not implemented.
- `SM_SR_FIND_AMBIGUITIES` is currently unused.
- `SM_SR_NODES` semantics belong to the selected implementation. Base
  `SmGlobalSolver::LocalSolve` emits node records; `SmCurve::GlobalCurveSolve` forwards the request,
  but its local curve solve can still emit converged two-parameter geometry. Specialized
  intersectors can consume the request differently.
- `SM_SR_ALL` filtering is also path-specific. `SmGlobalSolver::AddSortedSolution` retains min/max
  answers within tolerance of the best. `SmPolySolver` only rejects a newly arriving worse result;
  cleanup of older results is disabled, so non-global extrema can remain.
- Range solutions represent coincidence or another continuous solution interval.

For curve/surface results, curve parameters precede surface parameters. Confirm parameter ordering
in the owning solution constructor before interpreting raw arrays.

## Current dispatch boundaries

Read the switch in the owning entrypoint:

- `SmCurve::GlobalCurveSolve`: broad curve/curve min/max, normalization, at-distance,
  directed/projected, pivot, rotation, perspective, tangency, and specialized projected or
  perspective intersection variants. Ordinary `SM_SO_INTERSECT` uses `GlobalCurveIntersect`
  instead.
- `SmCurve::GlobalPointSolve`: curve/point solving through `SmCPGlobalSolver`.
- `SmCurve::GlobalPropertyAnalysis`: curve-property solving through `SmCurvePropertyGS`.
- `SmSurface::GlobalCurveSolve`: a smaller raw surface/curve solve set. Surface/curve intersection
  has its own `GlobalCurveIntersect` path; surface/line paths also contain analytic shortcuts.
- `SmSurface::GlobalSurfaceIntersect`: surface/surface intersections through
  `SmAdvSurfaceIntersector`, with numeric tracing and node generation in
  `SmSurfaceIntersector`. This curve-output path is distinct from
  `SmSurface::GlobalSurfaceSolve`, which returns solution records.
- `SmSurface::GlobalSurfaceSolve`: numeric surface/surface solution records through
  `SmSSGlobalSolver`, used by topology and intersector paths; this is distinct from
  `GlobalSurfaceIntersect` curve output.
- `SmSurface::GlobalPointSolve`: after any fast drop path, the new point solvers enabled by default
  normally route every operation except maximize through `GlobalPointSolve_1`. This family is
  implemented in `SmSurfacePointSolve.cpp` and is separate from the
  `SmGlobalSolver`/`SmLocalSolveNd` path. It ignores `cpdOptTargetDistance` and
  `eSolutionRequested`; when it finds an answer, it retains one best result and adds valid
  seam-equivalent copies. Ordinary maximize uses `GlobalPointSolve_0` / `SmSPGlobalSolver`
  in `SmSurface.cpp`. Disabling the new solvers also routes other operations there, except signed
  directed maximize, which always uses the new path.
- `SmTopologySolver`: broad BRep/point, BRep/curve, and BRep/BRep dispatch.
- `SmPolySolver`: independent polygonal traversal for limited min/max and directed/projected
  variants, including `PolyBrepPointSolve` and `PolyBrepRectangleIntersect`. No face/face local
  solver is registered.

The archived operation matrices are incomplete and should not be copied as support guarantees.

## Global traversal

`SmGlobalSolver` follows this conceptual sequence:

```text
SolveIt
  -> SolveBranch
     -> BranchMayContainAnswers
     -> AreReadyForLocalSolve?
        -> yes: LocalSolve
                -> AddSortedSolution (filter, sort, deduplicate, combine ranges)
        -> no: Subdivide
               -> FindBestSubdivisionNode
               -> recurse
```

`SmPolySolver` has a separate traversal:

```text
SolveBranch
  -> BranchMayContainAnswers
  -> LocalSolve
  -> if more subdivision is needed:
       Subdivide -> FindBestSubdivisionNode -> recurse
```

Important behaviors:

- Pruning is operation-specific and tolerance-sensitive.
- Default readiness is usually leaf-based. Bezier-surface and topology solvers have exceptions;
  `SmTopologySolver` can inspect internal spatial-tree nodes.
- Default subdivision favors the largest subdividable bounding box. Some directed, ray, and
  min/max solves change traversal order.
- A local solve may request more subdivision instead of declaring success or failure.
- `AddSortedSolution` incrementally performs best-value filtering, insertion sorting, duplicate
  detection, range combination, and single-result reduction during traversal.
- `AddSortedSolution` stops attempting range combination once the solution count reaches 100;
  `sm_CombineMinMax` also uses a 100-times deviation comparison when deciding whether an
  intersection point should split a range.

Review `SmGlobalSolver.cpp` around `SolveIt`, `BranchMayContainAnswers`,
`AreReadyForLocalSolve`, `FindBestSubdivisionNode`, `Subdivide`, `AddSortedSolution`, and
`IdenticalSolutions`.

## Local convergence

The local solvers distinguish:

- `SM_TR_FOUND_ANSWER_CONVERGED`
- `SM_TR_FOUND_ANSWER_CLOSE`
- `SM_TR_OUT_OF_BOUNDS`
- `SM_TR_BAD_JACOBIAN`
- `SM_TR_UNABLE_TO_CONVERGE`
- `SM_TR_STILL_ITERATING`

Use evaluator residual or solution size where exposed; otherwise use the path-specific objective
or geometric distance. An ND
evaluator must populate `pOptJacobian` itself, either analytically or by explicitly calling the
finite-difference `SmEvalNFunctionsObject::ComputeJacobian` helper; the Newton-Raphson loop does not
call that helper automatically. The ND solver may shorten or negate steps when step cutting is
enabled, and an unsolvable matrix initially sets `SM_TR_BAD_JACOBIAN`. Final residual checks can
overwrite it; inspect the final reason, residual, and execution path together. The 1D solver handles a near-zero
derivative by substituting `+/-1` and continuing.

Diagnostic fields are path-specific:

- 1D Newton paths update found accuracy, but some Brent exits can report a found solution without
  updating either the initialized `SM_TR_UNABLE_TO_CONVERGE` termination reason or
  `m_dFoundAccuracy`, which stays at its initialized `SM_UNDEF_DOUBLE` or holds a value left by an
  earlier iteration. Treat both as evidence only when the path is known to have set them.
- `SmLocalSolveNd::SolveIt` never updates `m_dFoundAccuracy`; it remains initialized to `0.0`, so
  do not treat it as evidence. Note the asymmetry with the 1D solver's `SM_UNDEF_DOUBLE`: `0.0`
  reads as a plausible converged accuracy, so its presence is not a signal that a path ran.
- An ND initial evaluator failure can return `SM_SUCCESS` with `rbFoundSolution` true and
  `SM_TR_UNABLE_TO_CONVERGE`.
- Final ND residual checks can replace an earlier `SM_TR_BAD_JACOBIAN` with
  `SM_TR_FOUND_ANSWER_CONVERGED` or `SM_TR_FOUND_ANSWER_CLOSE`.

Bounds behavior also differs by family. A bounded 1D solve calls `SE(SM_ERR_INVALID_INPUT)` in
debug when its initial guess lies outside the interval expanded by `SM_EFF_ZERO_SQRT`, then clamps
and continues. The ND solver passes its initial guess to the evaluator without solver-side clamp
or wrap; when `m_cpIntervals` is supplied, it constrains later guesses.

## Tangency, coincidence, and singularities

Use `SmSurfaceIntersector.h/.cpp` and `SmAdvSurfaceIntersector.cpp` for current surface/surface
classification, duplicate-start handling, and bidirectional restart behavior. Use
`SmSurfaceTracer.cpp` for single-surface drop, projection, and silhouette tracing. Keep these
distinctions:

- Tangency is not equivalent to an ordinary zero-distance minimum.
- Coincidence produces a range or region rather than one isolated crossing.
- An intersection singularity joins multiple solution branches.
- A surface singularity is a property of the parameterization and can occur independently.
- Tracing can terminate or restart at tangencies and singularities; tangent direction contributes
  to deciding whether two starts represent the same branch.
- Numeric tracing deduplicates starts through `SmSurfaceIntersector::AddStartPoint` and
  `IsPointOnCurve`; `SmAdvSurfaceIntersector` can append directly to its solution array and bypass
  `SmGlobalSolver::IdenticalSolutions`.

Treat archived claims of complete or fast handling as design intent, not guarantees. Require focused
regressions for grazing contact, poles, coincident ranges, and branch nexuses.

## Legacy terminology

- Replace “GSNLib” with SMLib solver terminology.
- Treat NLib as incorporated implementation, not a supported interoperability boundary.
- Ignore archived periodicity array types and operation counts; check current signatures.
- Keep C0/C1 drop-curve details in `smlib-kernel-drop-curve`.
