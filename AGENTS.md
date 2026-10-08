<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

SMLib is a NURBS-based boundary representation (BRep) solid modeling kernel. It represents solids as collections of topological entities (shells, faces, edges, vertices) bound to exact geometric definitions (NURBS surfaces, B-spline curves, analytic primitives). Geometry is mathematically exact and resolution-independent, unlike polygon meshes.

The library is layered: a low-level C++ kernel (`SMLib`), a stable C-style C++ wrapper (`SM_API`), a USD integration layer (`SM_API_USD`), and Python bindings (`SmPyLib` / `_omni_solid`).

<a name="build--test-commands"></a>

## Build and Test Commands

```shell
# Build (Linux; run from the repository root)
./prebuild.sh
./repo.sh build

# Run tests
./repo.sh test

# Format code
./repo.sh format
```

See the [build instructions](README.md#build-it) for prerequisites and Windows/macOS commands.

## Code Style

- **C++:** clang-format enforced (`.clang-format`); C++ standard set per `premake5.lua`
- **Python:** PEP 8, 120 column limit (`.editorconfig`)
- **Indent:** 4 spaces, LF line endings, UTF-8, trailing whitespace trimmed (`.editorconfig`)
- **Batch files:** CRLF; **Shell scripts:** LF (`.gitattributes`)

## Architecture

### Layers

1. **`SMLib`** (`source/SMLib/`) — core C++ kernel. Owns all geometry/topology classes (`SmBrep`, `SmFace`, `SmEdge`, `SmSurface`, `SmBSplineCurve`, …) and the algorithms. Memory is managed by an internal `SmContext`.
2. **`SM_API`** (`source/SM_API/`) — high-level C-style C++ API. Functions follow `SmApi*` naming. Primary surface for booleans, sweeps, fillets, tessellation, etc.
3. **`SM_API_USD`** (`source/SM_API_USD/`) — USD import/export. BRep data is stored as a custom `BrepArray` schema alongside standard USD prims.
4. **`BREP_SM_USD` / `BREP_USD_DATA`** — internal libraries doing the BRep ↔ USD data conversion.
5. **`SmPyLib`** (`source/SmPyLib/`, module `_omni_solid`) — pybind11 bindings exposing `SM_API` and `SM_API_USD` to Python, plus tessellation to `PolyBrep` and polygon-mesh booleans.

### Key Concepts

- **BRep** — a solid is defined by its boundary surfaces (`SmBrep` → `SmShell` → `SmFace` → `SmEdge` → `SmVertex`); topology tracks adjacency.
- **Geometry** — the kernel supports analytic, NURBS, and other curve/surface subclasses. NURBS can represent a sphere exactly; booleans and fillets do not require every input to be converted to NURBS.
- **Tessellation** — converts exact BRep to polygon meshes (`PolyBrep`); controlled by chord-height tolerance, angle tolerance, max edge length, max aspect ratio.
- **Context** — SM_API provides a process-global default `SmContext`, created lazily by
  `SmApiGetOrCreateContext()` and initialized automatically by Python. Selected operations instead
  use an input object's context or a dedicated output context.

### SM_API Header Map (C++ side)

| Header | Purpose |
|---|---|
| `SmApiTypes.h` | Basic types, export macros |
| `SmApiGeneral.h` | Context, draw, transform / scale / rotate / translate |
| `SmApiPrimitives.h` | Box, sphere, cone, cylinder, torus, pyramid, partial primitives, skins, blends |
| `SmApiBrep.h` | Booleans, sweeps, tessellation, cut, project, stitch, fillet, offset/shell |
| `SmApiCurves.h` | Curve creation, offset, evaluate, knot removal |
| `SmApiSurfaces.h` | Surface creation (extrude/sweep/ruled/revolution/skin/offset) |
| `SmApiFillets.h` | Chamfer / circular / remove fillet |
| `SmApiPolygons.h` | `SmPolyBrep` boolean ops, manifold/volume |
| `SmApiIntersectors.h` | Curve, surface, face, BRep intersections |
| `SmApiTrimmedSurfaces.h` | Trim with curves, project-and-trim, planar faces |
| `SmApiQueries.h` | Read-only queries on Brep / Face / Edge / Vertex: closest-point, topology lookups, bounding box, volume, area, length, vertex point, manifold check, deep copy. Surface / Curve evaluators (parametric domain, point, normal, derivatives, length, planarity, periodicity, …) are exposed as methods on `Surface` / `Curve` and wrap `SmApiSurfaces.h` / `SmApiCurves.h` plus a few kernel-direct calls; see `.agents/operations/queries.md`. |
| `SmApiHeal.h` | BRep healing |

## Python Quick Start (`_omni_solid`)

These examples use the source-checkout module `_omni_solid`. For the packaged
`usd_brep` API and its operation submodules, see the [package README](tools/pyproject/README.md).

```python
import _omni_solid as sm

box = sm.create_box((0, 0, 0), 10, 10, 10)
sph = sm.create_sphere((0, 0, 0), 5)

result = sm.boolean_union(box.copy(), sph.copy())
result = sm.boolean_difference(box.copy(), sph.copy())

mesh = sm.tessellate(box, chord_height_tolerance=0.0, curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0)

sm.usd.export_brep(box, "out.usda")

# Quick introspection (one-call summary):
print(box.info())  # {"faces": 6, "edges": 12, "vertices": 8, "manifold": True,
                   #  "volume": 1000.0, "bounds": ((0,0,0),(10,10,10)),
                   #  "center": (5,5,5)}

# Geometric properties (queries) -- methods on the type classes:
mn, mx = box.bounding_box()                      # ((0,0,0), (10,10,10))
print(box.center())                              # (5.0, 5.0, 5.0)
print(box.volume(), box.is_manifold_solid())     # 1000.0 True
print(box.face_count(), box.edge_count(), box.vertex_count())

face0 = box.faces()[0]
print(face0.area(), face0.bounding_box())
print(face0.surface(), face0.uv_domain())        # walk to the parent surface

edge0 = box.edges()[0]
print(edge0.length(), edge0.bounding_box())
v_start, v_end = edge0.vertices()                # walk to endpoints
print(edge0.faces())                             # 2 for a manifold edge

vertex0 = box.vertices()[0]
print(vertex0.point())                           # e.g. (0.0, 0.0, 0.0)
print(vertex0.edges(), vertex0.faces())          # walk to incident edges / faces

# Standalone curves and surfaces carry their own parametric queries:
line = sm.create_line_segment((0, 0, 0), (10, 0, 0))
print(line.parameter_range(), line.length())     # ((0.0, 1.0), 10.0)
print(line.evaluate(0.5))                        # (5.0, 0.0, 0.0)

srf = face0.surface()                            # or sm.create_surface_from_points(...)
(umin, vmin), (umax, vmax) = srf.uv_domain()
u_mid, v_mid = 0.5 * (umin + umax), 0.5 * (vmin + vmax)
print(srf.evaluate(u_mid, v_mid))                # (x, y, z)
print(srf.normal(u_mid, v_mid))                  # unit (nx, ny, nz)
print(srf.is_planar(1e-3), srf.is_periodic_u())  # True False (box face)

# Copy-then-mutate when you need to keep the original (most ops mutate in place,
# but they now return the same handle so you can chain: see below):
trimmed = box.copy()
sm.cut(trimmed, (0, 0, 5), (0, 0, 1))            # trimmed truncated; box unchanged
```

Conventions: points/vectors are 3-tuples; return values are opaque handles (`Brep`, `PolyBrep`, `Curve`, `Surface`, …); errors raise `RuntimeError`; USD lives in the `_omni_solid.usd` submodule.

**Developer API (`_smlib_dev`):** kernel-debugger workflows (loopuses, edgeuses, `topology_pick_ray`, `assert_valid`, `dump`, kernel `Draw()` extraction, `viewport_draw_*`, kernel-direct `user_test` / `user_test_example`, and a `tests` facade over `_smlib_tests`) live in a separate extension module. Import `import _smlib_dev as smdev` alongside `_omni_solid` for the Python GUI and SMLib-internal tooling; do not add these back to the stable `_omni_solid` surface.

**Mutation conventions:** common docstring contracts for BRep operations are listed below.
Consult the individual operation for exceptions and ownership details:

- **`Pure: returns new <type>; inputs unchanged.`** — read-only producer. Example: `offset_brep`, `shell_brep`, `create_box`, `create_sphere`, `linear_sweep`, `tessellate`.
- **`Consumes: <args>; returns new Brep.`** — ownership of the named inputs transfers to the operation; use only the returned Brep afterward. The result can reuse the modified primary operand. Example: `boolean_union`, `boolean_difference`, `merge_breps`.
- **`Mutates: <arg>; returns the same handle for chaining.`** — modifies the first argument *and* returns it, so calls compose: `box = sm.fillet_edges(sm.translate(box, (0,0,5)), edges, radius=0.5)`. Example: `fillet_edges`, `cut`, `translate`, `rotate`, `heal_brep`.
- **`Mutates: <arg>; returns (handle, ...diagnostics) for chaining.`** — same as above plus a tuple of report values; the handle comes *first* so unpacking is forward-compatible. Example: `stitch_into_solid`, `unify_normals`.

Names alone do not determine mutation: non-manifold sweeps mutate their source, and
`create_blend_primitive` inserts into a supplied BRep. Check the individual operation contract.

Use `brep.copy()` first when you need to preserve the original through a mutator.

`Face.surface()` and `Edge.curve()` return borrowed views of Brep-owned geometry. Before a representation-changing
`turn_to_nurbs()` or Brep scale, drop all affected borrowed views and reacquire them afterward. The Python binding
rejects the operation atomically while such a view is live; mutate `brep.copy()` instead when the original view must
remain valid. Pointer-preserving rigid transforms, positive uniform scales, and transforms of base NURBS geometry do
not require reacquisition.

## Operation Reference

Per-operation guides — parameters, common patterns, and gotchas — live in **`.agents/operations/`**. Start here:

- **[.agents/operations/INDEX.md](.agents/operations/INDEX.md)** — router by category, with the Python function names and matching `SmApi*` C headers for each operation.
- Use the existing guide structure when adding a new operation guide, and link it from [.agents/operations/INDEX.md](.agents/operations/INDEX.md).

When asked about a specific operation (e.g. booleans, fillets, tessellation), open the matching guide rather than the source first; the guides encode tuning order, visual diagnosis, and common workflows.

When you (or the user) hit a `RuntimeError` from any `_omni_solid` binding, open **[.agents/docs/errors.md](.agents/docs/errors.md)** before grepping the source. It maps every `SM_ERR_*` code to typical causes and remedies, and explains how to read the `Kernel trace:` segment of the message.

## End-to-End 3D Asset Generation

For text-to-3D / image-to-3D / "build me a \<thing\> in 3D" requests, follow the canonical pipeline in **[.agents/docs/generate_asset.md](.agents/docs/generate_asset.md)**:

```text
SMLib (analytic primitives + NURBS surfaces, sweeps, booleans, fillets)
  → sm.tessellate(brep) → PolyBrep
  → PolyBrep.to_mesh_arrays() → dict: points, normals, faces, face_normals
  → UsdGeom.Mesh prims (z-up, cm scale)
```

Hard rules:

- **Do not round-trip through `BrepArray` USD prims** for asset authoring. Stay in-process via
  `_omni_solid` and only emit `UsdGeom.Mesh` at the end. Round-tripping costs latency and collapses
  per-entity effective tolerances to one per-BRep maximum.
- Start from `tools/scripts/example_generate_asset.py` — it's a runnable reference; copy and adapt.

## Deeper Docs

- **[.agents/docs/OVERVIEW.md](.agents/docs/OVERVIEW.md)** — long-form architecture overview (concepts, layout, full header map, common patterns).
- **[.agents/skills/smlib-brep-model/references/brep-format.md](.agents/skills/smlib-brep-model/references/brep-format.md)** — canonical `SmBrep` runtime/`SmBrepData` structure, validity, validator coverage, special topology, and BrepArray translation contract.
- **[.agents/docs/generate_asset.md](.agents/docs/generate_asset.md)** — full asset-generation playbook.
- **[.agents/docs/errors.md](.agents/docs/errors.md)** — `RuntimeError` decoder: every `SM_ERR_*` code with typical causes and remedies; how to read the kernel trace.
- **[.agents/operations/INDEX.md](.agents/operations/INDEX.md)** — operation router.
- **[source/HYDRA_USD_BREP/README.md](source/HYDRA_USD_BREP/README.md)** — viewing BrepArrays in usdview: the launcher, requirements and display modes.
- **[README.md](README.md)** — top-level repo usage.

## Agent Conventions in This Repo

- Single source of truth for agent docs is **`AGENTS.md`** plus **`.agents/`**.
- When adding a new operation guide, follow the existing guide structure and link it from `.agents/operations/INDEX.md`.
- When a change fixes a bug listed in `KNOWN_ISSUES.md`, remove its entry in the same change.

## Repo-Local Skills

Repo-local agent skills live under **`.agents/skills/`**:

- **[smlib-python-api-use](.agents/skills/smlib-python-api-use/SKILL.md)** — using the existing `_omni_solid` Python API correctly from scripts, examples, tests, and diagnostics.
- **[smlib-sm-api-use](.agents/skills/smlib-sm-api-use/SKILL.md)** — using existing native `SmApi*` wrappers, including ownership/status handling and the runnable Boolean/bounds example.
- **[smlib-python-api-maintain](.agents/skills/smlib-python-api-maintain/SKILL.md)** — stable `_omni_solid` bindings: pybind11, stubs, docstrings, mutation semantics, and binding tests.
- **[smlib-python-dev-api-maintain](.agents/skills/smlib-python-dev-api-maintain/SKILL.md)** — debugger `_smlib_dev` bindings: loopuse/edgeuse topology, topology pick, Draw extraction, blend primitives, and `tests` facade.
- **[smlib-sm-api-maintain](.agents/skills/smlib-sm-api-maintain/SKILL.md)** — high-level `SM_API` functions, public headers, wrapper status handling, context/ownership rules, and kernel entrypoints.
- **[usd-brep-data-author](.agents/skills/usd-brep-data-author/SKILL.md)** — USD BrepArray data author: changing `source/BREP_USD_DATA`, codeless schema metadata, applied API schemas, `UsdBrepRead/Write/Utilities`, and direct Sdf authoring under `SdfChangeBlock`.
- **[usd-brep-schema](.agents/skills/usd-brep-schema/SKILL.md)** — schema-first BrepArray questions, asset compliance checks, validator/implementation drift audits, and schema-driven reader/writer/translator tests.
- **[smlib-brep-model](.agents/skills/smlib-brep-model/SKILL.md)** — source-backed SMLib `SmBrep` model questions, runtime validity and validator audits, topology code/tests, and SMLib/BrepArray translation.
- **[usd-brep-occt-translation](.agents/skills/usd-brep-occt-translation/SKILL.md)** — OCCT `.brep` import/export workflow: exact geometry, parameter ranges, topology orientation, periodic seams, pcurves, tolerances, component preservation, and round-trip validation.
- **[smlib-kernel-operations](.agents/skills/smlib-kernel-operations/SKILL.md)** — diagnosing failed or incorrect geometry operations across `_omni_solid`, `SM_API`, and `source/SMLib`.
- **[smlib-kernel-solvers](.agents/skills/smlib-kernel-solvers/SKILL.md)** — global/local solver traversal, convergence, subdivision, solution identity, tangency, and singularity internals.
- **[smlib-kernel-tessellation](.agents/skills/smlib-kernel-tessellation/SKILL.md)** — tessellation workflow for `_omni_solid.tessellate`, `SmApiTessellate`, `SmBrep::ConvertToPolyBrep`, `SmTess`, `PolyBrep`, and USD mesh output.
- **[smlib-kernel-fillets](.agents/skills/smlib-kernel-fillets/SKILL.md)** — fillet workflow for fillets, chamfers, blends, edge/corner selection, radii, continuity, `SmFilletExecutive`, and solver/topology interactions.
- **[smlib-kernel-healing](.agents/skills/smlib-kernel-healing/SKILL.md)** — BRep healer workflow for `_omni_solid.heal_brep`, `SmApiHealBrep`, `SmHealData` stages, property caches, problem lists, topology-change callbacks, seams, poles, and import healing.
- **[smlib-kernel-booleans](.agents/skills/smlib-kernel-booleans/SKILL.md)** — solid BRep boolean workflow for Python/SM_API booleans, manifold dispatch, CSG/list variants, input degeneracies, and `SmTopologyIntersector::IntersectInsertRelate()`; not polygon booleans.
- **[smlib-kernel-drop-curve](.agents/skills/smlib-kernel-drop-curve/SKILL.md)** — drop-curve workflow for curve-to-surface projection, `SmApiDropCurveToSrf`, `SmSurface::DropCurve`, `SmSurface::DropAndTrimCurve`, seams, domains, tolerances, and UV/3D output pairing.
- **[usd-brep-tools-python-gui](.agents/skills/usd-brep-tools-python-gui/SKILL.md)** — Python GUI workbench (`tools/smlib_gui`): active objects, viewport, picking via `smdev`, overlays, smoke tests.
- **[usd-brep-test-selection](.agents/skills/usd-brep-test-selection/SKILL.md)** — validation helper for choosing focused versus broad validation based on changed files and risk.
- **[text-to-3d-asset-generation](.agents/skills/text-to-3d-asset-generation/SKILL.md)** — text/image-to-3D CAD/USD asset workflow routed through `.agents/docs/generate_asset.md`.

## Folders that are never relevant

These folders should be skipped during search / indexing for any task. Cursor enforces this via `.cursorignore`; other agents (Claude Code, Codex, Copilot) should treat this list as a hard rule unless the user explicitly asks otherwise.

- **Build artifacts** — `_build/`, `_compiler/`, `_repo/`, `_builtpackages/` (multi-GB, regenerated by every build).
- **Binary test data** — the top-level test-fixture directories (binary `.smb` / `.usda` and similar fixtures; tens of MB, never edited by hand). See `.cursorignore` for the authoritative list.
- **Top-level `tests/`** — C++ harness projects, Python roundtrip scripts (`tests/python/`), and related integration tests; skip unless the user asks about them.
- **`.git/`** — repository internals; always skip.

## Task scope hints

Most day-to-day work touches only a subset of `source/`. The mapping below tells the agent where to look first; only step outside if a search in scope comes up empty or the user names a folder explicitly.

- **`SmPyLib` / `_omni_solid` work** — `source/SmPyLib/`, `source/SM_API/inc/`, `source/SM_API_USD/inc/` (only if touching `SmPyUsd.cpp`), `source/SMLib/inc/`, `source/BREP_SM_USD/inc/`, `source/BREP_USD_DATA/inc/`, `source/SmPyLib/tests/`. Stable bindings: `SmPyMain.cpp`, `SmPy*.cpp`, `stubs/omni_solid/`.
- **`SmPyDevLib` / `_smlib_dev` work** — `source/SmPyDevLib/`, `stubs/smlib_dev/`, `tests/test_smlib_dev.py`. Shares `SmPyCommon.h` / `SmPyError` from `source/SmPyLib/src/`. Roundtrip scripts are under `tests/python/` (index-ignored with `tests/`; use explicit paths only when the task needs them).
- **C kernel API (`SM_API`) work** — `source/SM_API/`, `source/SMLib/inc/`, plus the relevant operation guide under `.agents/operations/`.
- **USD bridge work** — `source/SM_API_USD/`, `source/BREP_SM_USD/`, `source/BREP_USD_DATA/`.
- **OCCT `.brep` translator work** — `source/OCCT_BREP_IMPORT/`, `source/OCCT_BREP_EXPORT/`, `tools/occt_to_usd_test/`, `tools/usd_to_occt_test/`; use `usd-brep-occt-translation` before attributing a visual defect to tessellation.
- **Kernel internals** — `source/SMLib/`.

Also **out of scope** for the layers above (don't search unless the user names them): the stand-alone test apps, GUI tools, and internal-only format-bridge components under `source/`; the per-component integration test suites under `tests/` (the `tests/python/` roundtrips are index-ignored with the rest of `tests/`). None are part of the `SmPyLib` / `SM_API` / USD-bridge build closure.
