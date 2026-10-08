<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD Import / Export

**Source:** `source/SM_API_USD/inc/SmApiUsd.h`
**Python (setup):** `usd.ensure_plugin_registered`
**Python (BRep):** `usd.import_breps`, `import_brep`, `export_brep`, `export_breps`, `append_breps`
**Python (mesh):** `usd.import_meshes`, `import_mesh`, `export_mesh`, `export_meshes`, `append_meshes`
**Python (scene tessellation):** `usd.tessellate_file`

## Overview

SM_API_USD provides file-level round-trip between OpenUSD (.usda/.usdc/.usd) and SMLib `SmBrep` / `SmPolyBrep` objects. BRep geometry is stored in USD as custom `BrepArray` prims that encode the full NURBS surface/curve topology alongside standard UsdGeom hierarchy.

This enables a pipeline where:
1. Exact BRep geometry is created/edited in SMLib
2. Exported to USD for interchange, rendering, or scene assembly
3. Re-imported into SMLib for further exact modeling operations

The round-trip can normalize or approximate representation: effective entity tolerances collapse
to one per-BRep value, non-NURBS curves are converted, unclamped NURBS knots may be canonicalized,
and analytic-face UV trims are omitted. Disabling healing does not disable these conversions.
See [the BRep conversion reference](../skills/smlib-brep-model/references/brep-format.md#current-bridge-normalizations-and-loss-points).

### Polygon meshes (`UsdGeomMesh` ↔ `SmPolyBrep`)

SM_API_USD also round-trips **triangle/polygon meshes** as standard `UsdGeomMesh` prims, represented in SMLib as **`SmPolyBrep`** (C) / **`PolyBrep`** (Python). Unlike BRep import, mesh import does **not** run the BRep healer. Import reads the mesh’s `points`, `faceVertexCounts`, and `faceVertexIndices`; export writes equivalent `UsdGeomMesh` attributes. The Python bindings expose this through `usd.import_meshes`, `usd.import_mesh`, `usd.export_mesh`, `usd.export_meshes`, and `usd.append_meshes`; the C API is declared in `SmApiUsd.h`.

For **`PolyBrep`** methods (bbox, mass properties, ray tests, etc.) without file I/O, see **[polybrep](polybrep.md)**. For BRep → mesh conversion in Python, see **[tessellation](tessellation.md)**.

Full signatures, parameters, and examples are in **[Mesh operations](#mesh-operations)** below.

## Operations

### ensure_plugin_registered
Register the omniSolid `BrepArray` schema plugin with USD. Idempotent.

Takes no arguments and returns `None`; raises `RuntimeError` if registration fails (typically `OMNISOLID_PLUGIN_PATH` unset or pointing at a missing/stale `schema/omniSolid/resources` directory).

**Design notes:**
- The import/export operations below already register the plugin on demand, so this is **not** required for a normal round-trip.
- Call it to register explicitly at process startup (so a misconfigured `OMNISOLID_PLUGIN_PATH` fails fast, not on first I/O), or before reading `BrepArray` prims through the raw `pxr` USD API (e.g. `UsdPrim.HasAPI`, schema-typed access) without going through this submodule.

```python
import _omni_solid as sm
sm.usd.ensure_plugin_registered()   # raises RuntimeError if OMNISOLID_PLUGIN_PATH is bad
```

### import_breps
Load all BRep prims from a USD file.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `filename` | str | — | Path to USD file (.usda, .usdc, or .usd) |
| `heal` | bool | `False` | Run the import healer on every imported Brep |
| `allow_partial` | bool | `False` | Retain successful members when others fail; keyword-only |

**Returns:** a list of `Brep` objects when `allow_partial=False`; a `BrepImportReport` when `allow_partial=True`.

**Design notes:**
- Opens the USD stage, finds every prim with a `BrepArray` schema, and converts each to an `SmBrep`.
- Strict import is all-or-nothing. If any packed BRep in any discovered `BrepArray` fails conversion, the call raises `RuntimeError` and returns no newly imported objects.
- Partial import attempts every locatable packed member. A completed scan returns a report instead of raising for member-level or prim-level failures; stage-level and process-level failures such as an unreadable file, out-of-memory condition, or fatal kernel status still raise `RuntimeError`.
- `BrepImportReport.breps` contains successful imports. `items` contains one `BrepImportResult` per attempted packed member in traversal/packed order, plus a prim-level result when malformed `BrepArray` data cannot be decoded. `failures` is the failed subset of `items`, and `complete` is `True` only when every item succeeded.
- Results are read-only, compare by value, and can be used in sets or as dictionary keys.
- Each result exposes `prim_path`, `packed_index`, `brep`, `status_code`, `status_name`, `message`, and `ok`. A prim-level failure uses `packed_index=None`; a successful result's `brep` aliases the corresponding object in `report.breps`.
- `heal=False` (default) disables the optional healer, making conversion effects easier to inspect. It does not guarantee identical topology, per-entity tolerances, or trim representation.
- `heal=True` runs the import healer on every imported Brep, including SMLib-authored data. The
  reader's CAD-source metadata read is currently disabled, so the source string is always empty and
  the healer-version check that would skip SMLib-authored geometry never matches. Healing can change
  tolerances and topology; there is no universal 100x tolerance multiplier.
- In C++, the caller owns newly appended `SmBrep*` objects. Python handles use
  the module's SMLib context lifetime, like other `_omni_solid` BReps.

```python
breps = sm.usd.import_breps("model.usda")
print(f"Loaded {len(breps)} BReps")
for b in breps:
    print(f"  faces={len(b.faces())}, edges={len(b.edges())}")
```

For partial batch import, opt in explicitly and inspect every failure:

```python
report = sm.usd.import_breps("model.usda", allow_partial=True)
for failure in report.failures:
    member = failure.packed_index if failure.packed_index is not None else "whole prim"
    print(f"{failure.prim_path} [{member}]: {failure.message}")

for brep in report.breps:
    print(brep.info())
```

### import_brep
Load a single BRep prim from a USD file by its SdfPath.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `filename` | str | — | Path to USD file |
| `prim_path` | str | — | SdfPath of the target prim (e.g. `"/World/Brep0"`) |
| `heal` | bool | `False` | Run the import healer on every imported Brep (see `import_breps`) |

**Returns:** a single `Brep` object.

**Design notes:**
- Use when you know the specific prim you want. More efficient than importing all and filtering.
- The prim_path must point to a prim that has BrepArray data.
- Every packed BRep in the selected `BrepArray` must convert successfully. If any member fails, the call raises `RuntimeError` instead of returning a partial or null result.

```python
brep = sm.usd.import_brep("model.usda", "/World/Part1")
```

### export_brep
Export a single BRep to a new USD file. Subject-first: `brep` precedes `filename` to match the rest of the modeling API. `export_uv_curves` and `shrink_face_domains` are keyword-only.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `brep` | Brep | — | BRep to export |
| `filename` | str | — | Output USD file path |
| `export_uv_curves` | bool | `True` | Include edgeuse UV trim curves (keyword-only) |
| `shrink_face_domains` | bool | `True` | Replace unbounded `face:range` values with UV-trim-curve bounds in the exported BrepArray; set `False` to skip trim-derived bounding (keyword-only) |

**Design notes:**
- Creates a new USD file (overwrites if exists).
- Export works directly on the input `Brep`, without a defensive copy or NURBS-to-analytic promotion. By default the BrepArray writer looks for unbounded face domains and authors trim-curve-derived bounds into `BrepArray.face:range`; it does not shrink the BRep face domains or trim the surfaces.
- Face-range preparation may attach missing UV trims to the source BRep. These remain for reuse even with `export_uv_curves=False`. Preparation precedes serialization so eligible generated trims can be written on the first export. Export also records the destination USD path as a BRep attribute and may populate geometry caches. Do not operate on the same BRep concurrently; export `brep.copy()` if these side effects must be isolated.
- Face-range bounding is best-effort. If the kernel cannot create UV trim curves for an unbounded face, that face may keep its existing domain.
- When scaling UV domains in converter code, use `SmExtent2d::Scale()` rather than multiplying endpoints directly. It preserves `+/-SM_INFINITE_PARAMETER` sentinels via `SM_IS_INFINITE()`; scaling those markers makes re-import treat unbounded ranges as finite. This matters when absorbing plane UV scale with face-domain shrinking disabled.
- Set `shrink_face_domains=False` to skip trim-derived bounding. NURBS face domains retain their original ranges, including sentinel bounds; analytic face ranges still use the exported surface's parameterization.
- `export_uv_curves=True` exports eligible existing B-spline UV trims only for faces exported as NURBS. Analytic-face UV trims are omitted; import also discards authored analytic-face trims, leaving later consumers to regenerate them as needed.
- Set `export_uv_curves=False` to reduce file size when UV curves aren't needed (e.g., for visualization only — the 3D edge curves are always exported).

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sm.usd.export_brep(box, "box.usda")
```

### export_breps
Export multiple BReps to a new USD file. Subject-first: `breps` precedes `filename`. `export_uv_curves` and `shrink_face_domains` are keyword-only.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `breps` | list[Brep] | — | BReps to export |
| `filename` | str | — | Output USD file path |
| `export_uv_curves` | bool | `True` | Include UV trim curves (keyword-only) |
| `shrink_face_domains` | bool | `True` | Replace unbounded `face:range` values with UV-trim-curve bounds in the exported BrepArray; set `False` to skip trim-derived bounding (keyword-only) |

**Design notes:**
- Same unbounded face-range bounding and input-side effects as `export_brep`, applied directly to each supplied BRep.

```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
sph = sm.create_sphere((20, 0, 0), 5)
sm.usd.export_breps([box, sph], "scene.usda")
```

### append_breps
Append BReps to an existing USD file. Subject-first: `breps` precedes `filename`. `export_uv_curves` and `shrink_face_domains` are keyword-only.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `breps` | list[Brep] | — | BReps to add |
| `filename` | str | — | Existing USD file path |
| `export_uv_curves` | bool | `True` | Include UV trim curves (keyword-only) |
| `shrink_face_domains` | bool | `True` | Replace unbounded `face:range` values with UV-trim-curve bounds in the exported BrepArray; set `False` to skip trim-derived bounding (keyword-only) |

**Design notes:**
- Opens an existing stage, adds new BrepArray prims, and re-saves.
- Use this to incrementally build up a scene without rewriting the entire file.
- Prim names are auto-generated to avoid conflicts with existing prims.
- Same unbounded face-range bounding and input-side effects as `export_brep`, applied only to the supplied BReps being appended.

```python
sm.usd.export_brep(box, "scene.usda")
sm.usd.append_breps([sph, cyl], "scene.usda")
# scene.usda now has 3 BReps
```

## Mesh operations

Polygon mesh APIs use **`SmPolyBrep`** / **`PolyBrep`** only (not `SmBrep` / `Brep`). C API returns **`SmStatus`** (`SM_SUCCESS` or an error such as `SM_ERR`); Python uses **`_omni_solid.usd`** and **raises `RuntimeError`** with the numeric status if the call fails.

**Implementation:** Import calls **`SMU_BrepConvert::CreateSmPolyBrep_FromUsdMesh`** (`SmuConvert.h`). Export and append call **`SMU_BrepConvert::PopulateMeshAttr`** (`SmuTessellate.h`) to author `UsdGeomMesh` specs on the stage layer.

### import_meshes

Load **every active `UsdGeomMesh`** on the stage, in **USD traversal order** (`UsdTraverseInstanceProxies()`), and convert each to an `SmPolyBrep`.

**C++**

```cpp
#include <SmApiUsd.h>
#include <vector>

std::vector<SmPolyBrep*> meshes;
SmStatus st = SmApiUsdImportMeshes("/path/to/file.usda", meshes);
// On success, caller owns each SmPolyBrep* in meshes; delete when done.
```

**Python**

```python
meshes: list[PolyBrep] = sm.usd.import_meshes("model.usda")
```

| Parameter | Type (C++) | Type (Python) | Default | Description |
|---|---|---|---|---|
| `pFileName` / `filename` | `const char*` | `str` | — | Path to USD file (`.usda`, `.usdc`, `.usd`) |
| `rSmPolyBreps` | `std::vector<SmPolyBrep*>&` (output) | *(return value)* | — | All converted meshes |

**Returns:** C: `SmStatus`. Python: `list[PolyBrep]` on success. If the stage contains **no** `UsdGeomMesh` prims, the API returns an error (`SM_ERR`) and Python **raises** (it does not return an empty list).

**Design notes:**
- Traversal visits **all** active mesh prims on the stage (not only under `/World`).
- There is **no** `heal` flag (unlike BRep import). Invalid or non-manifold mesh data may still produce an `SmPolyBrep` or fail depending on kernel rules.
- **Ownership:** C caller must **`delete`** each pointer in `rSmPolyBreps`. Python returns native `PolyBrep` objects owned by the interpreter.

### import_mesh

Load **one** `UsdGeomMesh` prim identified by **`SdfPath` string** (e.g. `"/World/Mesh0"`). The prim at that path must be a **`UsdGeomMesh`**.

**C++**

```cpp
SmPolyBrep* mesh = nullptr;
SmStatus st = SmApiUsdImportMesh("/path/to/file.usda", "/World/Mesh0", mesh);
```

**Python**

```python
m: PolyBrep = sm.usd.import_mesh("model.usda", "/World/Mesh0")
```

| Parameter | Type (C++) | Type (Python) | Default | Description |
|---|---|---|---|---|
| `pFileName` / `filename` | `const char*` | `str` | — | Path to USD file |
| `pPrimPath` / `prim_path` | `const char*` (`SdfPath` text) | `str` | — | Absolute path to the mesh prim (must be `UsdGeomMesh`) |
| `rpSmPolyBrep` | `SmPolyBrep*&` (output) | *(return value)* | — | Resulting mesh |

**Returns:** C: `SmStatus`; output in `rpSmPolyBrep` on success. Python: single `PolyBrep`.

**Design notes:**
- Fails if the path is invalid or the prim is not a `UsdGeomMesh`.
- **Ownership:** C caller owns `*rpSmPolyBrep`.

### export_mesh

Write **one** `SmPolyBrep` to a **new** USD file as **`/World/Mesh0`** (`UsdGeomMesh`).

**C++**

```cpp
SmStatus st = SmApiUsdExportMesh("/path/to/out.usda", pPolyBrep);  // pPolyBrep != nullptr
```

**Python** (subject-first: `mesh` precedes `filename`)

```python
sm.usd.export_mesh(poly_mesh, "out.usda")  # poly_mesh: PolyBrep
```

| Parameter | Type (C++) | Type (Python) | Default | Description |
|---|---|---|---|---|
| `pSmPolyBrep` / `mesh` | `SmPolyBrep*` | `PolyBrep` | — | Mesh to export; must not be `NULL` |
| `pFileName` / `filename` | `const char*` | `str` | — | Output USD path (creates/overwrites file) |

**Returns:** C: `SmStatus`. Python: `None` on success; raises on failure.

**Design notes:**
- Creates a new stage with `/World` xform and a single mesh child **`Mesh0`**.
- Caller **retains** ownership of the `SmPolyBrep` / `PolyBrep`.
- **In-memory mesh mutation:** Unlike BRep export, mesh export does **not** attach SdfPath attributes to each `SmPolyBrep`. While building `UsdGeomMesh` indices, the exporter **may update per-vertex index bookkeeping** on the mesh. Do not assume the `PolyBrep` is unchanged after export; **copy or re-import** first if you need a pristine object for parallel work.

### export_meshes

Write **multiple** `SmPolyBrep` objects to a **new** USD file as **`/World/Mesh0`**, **`/World/Mesh1`**, … in vector order.

**C++**

```cpp
std::vector<SmPolyBrep*> meshes = { /* ... */ };
SmStatus st = SmApiUsdExportMeshes("/path/to/out.usda", meshes);
```

**Python** (subject-first: `meshes` precedes `filename`)

```python
sm.usd.export_meshes([mesh_a, mesh_b], "out.usda")
```

| Parameter | Type (C++) | Type (Python) | Default | Description |
|---|---|---|---|---|
| `rSmPolyBreps` / `meshes` | `std::vector<SmPolyBrep*>&` | `list[PolyBrep]` | — | Meshes to write (must not contain null pointers) |
| `pFileName` / `filename` | `const char*` | `str` | — | Output USD path |

**Returns:** C: `SmStatus`. Python: `None` on success.

**Design notes:**
- Prim names are fixed **`Mesh0`**, **`Mesh1`**, … **`Mesh{N-1}`** under **`/World`** (for `N` meshes).
- Equivalent to calling **`SmApiUsdExportMesh`** for a single mesh (always **`Mesh0`**).
- Same **in-memory mesh mutation** behavior as **`export_mesh`** (see above).

### append_meshes

Append meshes to an **existing** USD file: opens the stage, adds new **`UsdGeomMesh`** prims under **`/World`**, saves.

**C++**

```cpp
std::vector<SmPolyBrep*> more = { /* ... */ };
SmStatus st = SmApiUsdAppendMeshes("/path/to/existing.usda", more);
```

**Python** (subject-first: `meshes` precedes `filename`)

```python
sm.usd.append_meshes([extra_mesh], "scene.usda")
```

| Parameter | Type (C++) | Type (Python) | Default | Description |
|---|---|---|---|---|
| `rSmPolyBreps` / `meshes` | `std::vector<SmPolyBrep*>&` | `list[PolyBrep]` | — | Meshes to append (non-const reference: may update per-vertex index bookkeeping during authoring; does **not** add SdfPath attributes to each mesh, unlike **`append_breps`**) |
| `pFileName` / `filename` | `const char*` | `str` | — | Existing USD file to open and re-save |

**Returns:** C: `SmStatus`. Python: `None` on success.

**Design notes:**
- **Naming:** New prims are named **`Mesh` + decimal index**, where the index **`N`** starts at the **number of `UsdGeomMesh` prims already on the stage** (full-stage traversal count) **before** appending. Example: if the file already has two meshes anywhere on the stage, the next batch becomes **`Mesh2`**, **`Mesh3`**, … under **`/World`**.
- **`/World`** is created if missing (`UsdGeomXform`).
- Does not modify existing **USD** mesh prims on the stage; only **adds** new specs. **Input** `SmPolyBrep` / `PolyBrep` objects use the same **in-memory mutation** semantics as **`export_mesh`** (see above).

## Scene tessellation

### tessellate_file

Tessellate every BrepArray prim in a USD file and write a copy of the whole stage with one **`UsdGeomMesh`** per Brep. The original prims, hierarchy and materials are kept, which `import_breps` + `tessellate` + `export_meshes` does not do.

**C++**

```cpp
std::vector<SmApiUsdMeshTessellationResult> results;
SmStatus st = SmApiUsdTessellateFile("scene.usda", "scene_mesh.usda", SmTessellationParams(), FALSE, 0, results);
```

**Python**

```python
results = sm.usd.tessellate_file("scene.usda", "scene_mesh.usda", chord_height_tolerance=0.01)
failed = [r for r in results if not r.ok]
partial = [r for r in results if r.ok and r.failed_face_count]
```

| Parameter | Default | Description |
|---|---|---|
| `input`, `output` | — | USD file to read, USD file to write |
| `chord_height_tolerance`, `curve_angle_tolerance_deg`, `surface_angle_tolerance_deg`, `max_edge_length`, `max_aspect_ratio` | as `tessellate()` | Tessellation controls (keyword-only) |
| `heal` | `False` | Run the healer while importing |
| `threads` | `0` | Tessellation threads; `0` = automatic |

**Returns:** a list of `MeshTessellationResult`, one per Brep in prim order: `prim_path`, `packed_index` (`None` for a prim that could not be read), `mesh_path` (`None` when no mesh), `ok`, `failed_face_count` (nonzero = partial mesh kept), `point_count`, `status_code` / `status_name`, `message`.

**Design notes:**
- Meshes are named **`tess_<prim>_<brep>`** under the default prim (or **`/Output`**); a name already in use gets a **`_<n>`** suffix, so existing prims are never overwritten. Each mesh is bound to the material of its Brep's brep-element `GeomSubset`, or else to the BrepArray's material.
- Each mesh carries its Brep's world transform (default time), so it lands where the Brep is even under transformed parents; each instance of an instanced Brep gets its own mesh.
- Each mesh also carries its Brep's computed visibility and purpose, since it is not under the Brep's ancestors.
- The meshes are authored on the stage's session layer and merged into the written copy, so the input layer is never modified, even when the caller has it open.
- When no mesh is authored, the Python `RuntimeError` names each failed Brep (up to 10) and why.
- When `output` is in a different directory from `input`, relative asset paths in the root layer (sublayers, references, payloads, asset attributes) are rewritten so they still resolve.
- Prims are tessellated in parallel, each with its own context; a single prim holding many Breps (as `export_breps` writes) is tessellated serially.
- Raises `RuntimeError` when the input cannot be opened, has no BrepArray prims (`SM_ERR_INVALID_INPUT`), no mesh is authored, or the output cannot be written. The output is not written when no mesh is authored.

### heal_file

Heal BrepArray definitions in the layers used by the current USD composition and write the scene with the healed geometry, keeping its layers, references, native instancing, transforms and materials.

```python
results = sm.usd.heal_file("input.usdc", "input_healed.usdc", threads=0)  # 0 = automatic
```

C++: `SmApiUsdHealFile(pInputFile, pOutputFile, iThreadCount, rResults)` in `SmApiUsd.h`.

**Returns:** a list of `BrepArrayHealResult`, one per BrepArray: `prim_path`, `layer` (the layer that defines it), `ok`, `brep_count`, `status_code` / `status_name`, `message`.

**Design notes:**
- Each BrepArray is healed in the layer that defines it. Other layers, including package members, are written as standalone layers to `<output stem>_layers/` beside the output, with references redirected. The output must be a new standalone `.usd`, `.usda`, or `.usdc` file; package output is not supported.
- External layers used only by unselected variants are not enumerated or healed; their references keep pointing at the original assets. Selecting such a variant later does not imply that its geometry was healed.
- All or nothing: if any BrepArray fails, nothing is written, and the `RuntimeError` names each failure.
- Rejected, because the result could not be written back faithfully: BRep geometry authored in a variant or composed from several layers, GeomSubsets other than face or Brep material bindings, and purpose-specific subset material bindings. Material subsets must be authored entirely under the BRep's defining prim spec: overrides or additional subsets in consuming layers or internal-reference sites are rejected before publishing output, since rebuilding subset names/indices would invalidate those opinions. Array-level material overrides remain supported.

## Choosing File Format

| Format | Extension | Characteristics |
|---|---|---|
| ASCII | `.usda` | Human-readable, larger files, good for debugging |
| Binary (Crate) | `.usdc` | Compact, fast loading, not human-readable |
| Auto | `.usd` | Format chosen by USD library based on context |

For development and debugging, use `.usda` so you can inspect the file contents. For production and large models, use `.usdc`.

## The `heal` Parameter

BRep healing fixes common issues after data conversion:
- Tiny gaps between edges that should be connected
- Slightly misaligned vertices at edge endpoints
- Minor surface/curve inconsistencies

The C API (`SmApiUsdImportBreps` / `SmApiUsdImportBrep`) defaults `heal=TRUE`; Python defaults
to `False`. Enabling healing runs the healer on every imported Brep -- including SMLib-authored
data -- and can change topology and tolerances. It does not specify a fixed tolerance multiplier.
Source-selected healing is not currently active on either side of the public bridge. On export,
`BrepAppend_SMLibToUsd()` sets the `source` custom data to `"SMLib"`, but the later
`BrepWriteToUsdStage()` call unconditionally rewrites that key from `m_sCADSource`, which is still
empty on this path, so an exported Brep carries `source == ""`. On import, the matching read in
`BrepReadFromUsdStage()` is commented out, so the source is empty regardless of what the file
holds. The healer-version check that would skip SMLib-authored geometry therefore never matches.

| Scenario | Python `heal=` | Rationale |
|---|---|---|
| Round-tripping your own exports | `False` (default) | Avoids optional healing; conversion normalization still applies |
| Performance-critical batch import | `False` (default) | Skip healing overhead |
| Debugging import issues | `False` (default) | See raw imported state |
| Importing from another CAD system | `True` | Repair conversion gaps; inspect resulting topology and tolerances |

## The `export_uv_curves` Parameter

UV curves describe boundaries in surface parameter space. The exporter retains eligible existing
B-spline UV curves on NURBS faces when enabled. Analytic-face trims are omitted because their
stored and runtime parameter spaces differ; downstream consumers can regenerate missing trims
from the 3D boundary geometry. This option does not guarantee lossless reconstruction.

| Scenario | export_uv_curves= | Rationale |
|---|---|---|
| Round-trip archival | `True` | Retain eligible NURBS-face trim data |
| Interchange with other CAD | `True` | Retain eligible trim data; confirm analytic-face reconstruction support |
| Visualization/rendering only | `False` | 3D edges are sufficient for display; smaller files |
| Large scenes with many BReps | `False` | Significant file size reduction |

## Common Workflows

### Create and Save
```python
import _omni_solid as sm

box = sm.create_box((0, 0, 0), 10, 10, 10)
sph = sm.create_sphere((5, 5, 5), 3)
result = sm.boolean_difference(box, sph)
sm.usd.export_brep(result, "drilled_box.usda")
```

### Load, Modify, Save
```python
breps = sm.usd.import_breps("input.usda")
for b in breps:
    sm.circular_fillet(b, 0.5)
sm.usd.export_breps(breps, "filleted.usda")
```

### Verify Round-Trip Fidelity
```python
box = sm.create_box((0, 0, 0), 10, 10, 10)
orig_faces = len(box.faces())
sm.usd.export_brep(box, "/tmp/test.usda")
breps = sm.usd.import_breps("/tmp/test.usda")
assert len(breps[0].faces()) == orig_faces
```

## C API Details

The C API functions use `std::vector<SmBrep*>` instead of Python lists and return `SmStatus` instead of raising exceptions.

The original three-argument `SmApiUsdImportBreps` symbol remains the strict API. Its five-argument overload accepts an explicit partial-import policy and a result vector:

```cpp
std::vector<SmBrep*> breps;
std::vector<SmApiUsdBrepImportResult> results;
SmStatus st = SmApiUsdImportBreps(
    "/path/to/file.usda", breps, FALSE, TRUE, &results);
```

The result vector is required when partial import is enabled and is cleared at entry. Its non-null `m_pBrep` values are non-owning aliases to pointers appended to `breps`; the caller deletes each BRep through `breps` only. A completed partial scan returns `SM_SUCCESS` even if individual records failed. File, plugin, stage-level, and fatal process-level failures still return an error, leave the BRep vector unchanged, and leave the result vector empty.

C API only (these take OpenUSD C++ types that are not wrapped in the Python module):
- **Primitive conversion** from USD — `SmuConvert.h` provides direct conversion from `UsdGeomCapsule`, `UsdGeomCone`, `UsdGeomCube`, `UsdGeomCylinder`, `UsdGeomSphere` to SmBrep, and from `UsdGeomMesh` to SmPolyBrep.
- **Low-level BrepArray manipulation** — `BrepAppend_SMLibToUsdBrep` and `BrepMove_OneUsdBrepToSMLib` for fine-grained control over the BrepArray prim data.
