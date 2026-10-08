<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

<a name="generate-3d-assets-smlib-stack--usd-mesh"></a>

# Generate 3D Assets

**Purpose:** Canonical pipeline for **text-to-3D** and **image-to-3D** asset generation in this repository — the user describes a \<thing\> (or shows a reference image of one), and the agent produces a 3D asset that is visualized as a USD mesh. Trigger phrasings include "build / create / generate a \<thing\> in 3D", "make me a \<robot / vehicle / object\>", or an attached reference image with "model this". The end result is a usda file with meshes that opens in Omniverse viewers, `tools/scripts/view_usd.py`, or anything else that reads `UsdGeom.Mesh`.

---

<a name="1-the-pipeline-suggested"></a>

## 1. The Suggested Pipeline

```
    [ SMLib stack ]                                     [ USD ]
    ─────────────────────────────────────────────────────────────
    analytic primitives  +  freeform NURBS curves/surfaces
    sweep / extrude / revolve / boolean_* / fillet / ...          build
           │                                                       │
           ▼                                                       │
    sm.tessellate(brep)  →  PolyBrep   (exact → polygons)          │
           │                                                       │
           ▼                                                       │
    PolyBrep.to_mesh_arrays()   →   dictionary of mesh arrays       │
                                                                   ▼
                                                 UsdGeom.Mesh prims (one per part)
                                                 + optional primvars:displayColor
                                                 + SetStageUpAxis(z),
                                                   SetStageMetersPerUnit(0.01)
```

In prose: the SMLib stack builds geometry from **analytic primitives** and **freeform NURBS curves and surfaces**, combined and shaped by operations like `sweep`, `extrude`, `revolve`, `boolean_union` / `boolean_difference` / `boolean_intersection`, and `fillet_edges` → `sm.tessellate(brep)` produces a `PolyBrep` → `PolyBrep.to_mesh_arrays()` returns a dictionary with `points`, `normals`, `faces`, and `face_normals` → those arrays are published as `UsdGeom.Mesh` prims (one per part) on a z-up, centimeter-scale stage, with optional `primvars:displayColor`.

All geometry is built in-process through the SMLib stack — the `SMLib` kernel, the `SM_API`
C-style C++ wrapper, and the `_omni_solid` Python bindings (`source/SmPyLib/`). USD only touches
the pipeline at the **very end** to publish the finished polygon meshes. **No `BrepArray` prims in
the output**.

Description: `SMLib` is the NURBS/BRep kernel; `SM_API` exposes it as a stable C-style C++ API;
`_omni_solid` wraps that API for Python. From the agent's point of view, everything geometric is a
call on `_omni_solid` (imported as `sm`). The `BREP_SM_USD` round-trip through USD `BrepArray`
prims adds latency and normalizes representation: per-entity effective tolerances collapse to one
per-BRep intersection-tolerance value derived from the maximum effective topology zone tolerance;
edge and wire curves are exported as NURBS; selected analytic face surfaces are preserved; and
export attempts to represent other non-B-spline surface types as approximated B-spline surfaces.
For pure asset-authoring there's no reason to involve BRep round-trips at all.

---

## 2. Ground Rules for Any Agent

1. **Start from `tools/scripts/example_generate_asset.py`.** It is a complete, runnable reference implementation of this pipeline — copy and adapt it. Section 5 below reproduces its core for quick reading. Do not start from scratch.
2. **Use any SMLib-stack creation method — pick the simplest one that fits the shape.** The toolkit spans three families:
    - **Analytic primitives** — `create_box`, `create_sphere`, `create_cylinder`, `create_cone`, `create_torus`, `create_partial_torus`, …
    - **Freeform NURBS curves and surfaces** — built from control, interpolation, or approximation points (`create_curve_interp_points`, `create_curve_approx_points`, `create_surface_from_ordered_points`, `create_surface_from_random_points`, `create_surface_from_points`), and wrapped into sheet bodies via ruling or skinning (`create_ruled_surface`, `create_skin_surface`).
    - **Operations that shape or combine them** — sweeps and extrusions (`linear_sweep`, `rotational_sweep`, `pipe_sweep`, `curve_sweep`, `draft_sweep`, `taper_extrude`), revolutions (`create_surface_revolution`), **booleans (`boolean_union`, `boolean_difference`, `boolean_intersection`)**, fillets / chamfers, offset / shell, stitching, trims.

   See section 6 for the full per-function map. Analytic primitives are a good first choice *when they fit* (exact, tessellate cleanly), but a sweep, revolve, or freeform NURBS patch is often cleaner than composing many primitives with booleans.
3. **Build parametrically.** Accept dimensions as Python variables; don't hard-code magic numbers scattered through the script.
4. **One asset, one USDA.** Write a single combined `.usda` with one `UsdGeom.Mesh` prim per logical part, grouped under a root `Xform`.
5. **Do not introduce new dependencies.** Only `_omni_solid`, `pxr` (USD Python), and optionally `numpy` / `PIL` for post-processing. `tools/scripts/view_usd.py` uses `pyvista` for interactive rendering; that's already wired up.
6. **Do not export to USD BReps.** Never call `sm.usd.export_brep` or `sm.usd.export_breps` in this pipeline. 
7. **Do not reimplement geometry code.** No custom fillet logic, no ad-hoc tessellators, no NURBS math. If a primitive or operation is missing from `_omni_solid`, ask the user before writing C++ or adding bindings.

---

## 3. Project Structure

| Path | What it holds |
|---|---|
| `source/SmPyLib/src/SmPy*.cpp` | pybind11 module `_omni_solid` — split per category (`SmPyPrimitives.cpp`, `SmPyBooleans.cpp`, `SmPyFillets.cpp`, `SmPyTessellation.cpp`, `SmPyPolyBrep.cpp`, etc.). Grep `m.def(` across these files to see the full Python surface. `SmPyMain.cpp` is the spine that wires up `bind_*()` calls. |
| `source/SmPyLib/src/SmPyUsd.cpp` | pybind11 submodule `_omni_solid.usd` — USD import/export. Use only the mesh functions (`export_mesh` / `export_meshes`); avoid `export_brep*` for this pipeline. |
| `tools/scripts/view_usd.py` | Stage loader + screenshot renderer + interactive pyvista viewer. Has `_ensure_usd_runtime_env()` and `_pick_config(repo)` helpers that every script should reuse for environment setup. |
| `tools/scripts/example_generate_asset.py` | **Runnable reference.** Minimal end-to-end implementation of this pipeline (base + post + ball + collar). Run it to confirm your environment works, then copy it as the starting point for new assets. |
| `_build/<platform>/<config>/_omni_solid.pyd` (or `.so`) | Built binding. `<platform>` is `windows-x86_64` / `linux-x86_64` / `linux-aarch64` / `macos-universal`. `<config>` is `release` or `debug`. `view_usd._pick_config(repo)` picks one. |
| `.agents/docs/generate_asset.md` | This workflow guide (you are here). |
| `.agents/operations/*.md` | Per-operation references — see section 6 below. |

Create an output folder of your choice (e.g. `_assets_output/` at the repo root) for the generated `.usda` and `.png` files; it does not need to exist ahead of time. Names starting with `_` are automatically gitignored via the `_*/` rule, matching the convention already used by `_build/`. Note that the `scripts/` and `robot_output/` paths in older discussions are also gitignored and should not be assumed to exist.

---

## 4. Runtime Environment Boilerplate

Every script that uses `_omni_solid` needs this header. Copy it verbatim. It sets up USD runtime DLLs, locates the right `_omni_solid.pyd` for the build config, and adds it to `sys.path`.

```python
import os
import sys

# Discover the repo root from this script's own location so the
# boilerplate works on any machine regardless of where the repo was
# cloned or what the top-level folder is called.  This idiom assumes
# the script lives at ``<repo>/tools/scripts/``; adjust the number of
# ``".."`` segments if you place it deeper.  For scripts that live
# *outside* the repo, set the ``SOLIDMODELING_REPO`` environment
# variable to point at the checkout.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO        = os.environ.get(
    "SOLIDMODELING_REPO",
    os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "..")))

TOOLS = os.path.join(REPO, "tools", "scripts")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import view_usd
view_usd._ensure_usd_runtime_env()

_BUILD_BIN = os.path.join(
    REPO, "_build",
    view_usd._pick_platform(),     # windows-x86_64 / macos-universal / linux-x86_64 / linux-aarch64
    view_usd._pick_config(REPO))   # release / debug
if os.path.isdir(_BUILD_BIN) and _BUILD_BIN not in sys.path:
    sys.path.insert(0, _BUILD_BIN)

import _omni_solid as sm
from pxr import Usd, UsdGeom, Sdf, Vt, Gf
```

Three things make this portable:

1. **`REPO` from `__file__`** — no string literal, no per-user path. Any clone (`/home/alice/work/solid`, `D:\src\solidmodeling`, etc.) works unchanged. `tools/scripts/example_generate_asset.py` (lines 45-46) is the canonical example.
2. **`SOLIDMODELING_REPO` env-var escape hatch** — for the rare case where the script lives outside the repo (e.g. an asset author's personal scratch folder), set the variable once and the same boilerplate keeps working.
3. **`view_usd._pick_platform()`** inspects `sys.platform` and `platform.machine()`, so Windows x86-64, macOS, Linux x86-64, and Linux ARM64 (aarch64) all resolve correctly. **`view_usd._pick_config(REPO)`** honors `SMLIB_BUILD_CONFIG=debug|release` when set; otherwise it picks `release` if real build artifacts exist under `_build/<platform>/release/`, else `debug`.

---

## 5. Minimum Working Template

The snippet below is the core of `tools/scripts/example_generate_asset.py`, trimmed for readability. Prefer copying that file directly — it has the complete CLI, argument parsing, optional screenshot, and the platform/config-aware environment setup.

Run commands from the repository root after completing the [source build](../../README.md#build-it).
Install `uv`; the commands use the Python 3.12 environment and rendering dependencies
in [pyproject.toml](../../pyproject.toml), as in the [GUI workflow](../../README.md#run-the-python-gui).
For custom scripts, include the [runtime environment boilerplate](#4-runtime-environment-boilerplate).

On Linux, set the native library paths before starting Python. For a release build
on x86-64 (use `linux-aarch64` for ARM64; replace `release` with `debug` for a debug build):

```bash
export SMLIB_BUILD_CONFIG=release
export LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/$SMLIB_BUILD_CONFIG:$PWD/_build/target-deps/usd/$SMLIB_BUILD_CONFIG/lib:$PWD/_build/target-deps/python/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

To check generation and rendering separately:

```shell
uv run --locked --group gui python tools/scripts/example_generate_asset.py
uv run --locked --group gui python tools/scripts/view_usd.py --screenshot _assets_output/ExampleAsset.png _assets_output/ExampleAsset.usda
```

Expected outputs are `_assets_output/ExampleAsset.usda`, containing the base, post,
ball, and collar meshes, and a rendered PNG. Invoke the viewer directly because the
example's `--screenshot` option can select a different Python interpreter.
Rendering needs a working graphics context; see [GUI troubleshooting](../../tools/smlib_gui/README.md)
if it fails.

<a name="core-pipeline-abridged"></a>

### Abridged Core Pipeline

```python
# 1. <boilerplate from section 4>

# 2. Build parts in the SMLib stack
def build_parts():
    parts = []
    def add(name, brep, color=None, kind=None):
        # kind is a stable semantic role used for optional reference scoring.
        parts.append(dict(name=name, brep=brep, color=color, kind=kind))

    box = sm.create_box((-5, -5, 0), 10, 10, 10)
    sm.fillet_edges(box, box.edges(), radius=1.0)
    add("body", box, (0.3, 0.6, 0.95), kind="body")

    add("ball", sm.create_sphere((0, 0, 12), 2.0),
        (1.0, 0.2, 0.2), kind="accent")
    return parts

# 3. Tessellate in the SMLib stack
def tessellate(parts, **overrides):
    # Library defaults unless overridden, e.g. chord_height_tolerance=...
    for p in parts:
        pb = sm.tessellate(p["brep"], **overrides)
        p["mesh"] = pb.to_mesh_arrays()

# 4. Write a single mesh-only USDA
def write_usda(parts, out_path, root="/Asset"):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        os.remove(out_path)
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    xform = UsdGeom.Xform.Define(stage, root)
    stage.SetDefaultPrim(xform.GetPrim())

    for p in parts:
        m = UsdGeom.Mesh.Define(stage, f"{root}/{p['name']}")
        mesh = p["mesh"]

        pts = mesh["points"]
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*pt) for pt in pts]))

        flat = mesh["faces"]               # [n, i0, i1, ..., n, j0, j1, ...]
        counts, indices = [], []
        i = 0
        while i < len(flat):
            n = int(flat[i]); counts.append(n)
            indices.extend(int(x) for x in flat[i + 1: i + 1 + n])
            i += 1 + n
        m.CreateFaceVertexCountsAttr (Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))

        normals = mesh.get("normals")
        if normals and len(normals) == len(pts):
            m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in normals]))
            m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

        kind = p.get("kind")
        if kind:
            m.GetPrim().CreateAttribute(
                "asset:kind", Sdf.ValueTypeNames.Token, custom=True).Set(kind)

        color = p.get("color")
        if color is not None:
            cp = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray,
                interpolation=UsdGeom.Tokens.constant)
            cp.Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))

    stage.GetRootLayer().Save()

if __name__ == "__main__":
    parts = build_parts()
    tessellate(parts)
    write_usda(parts, os.path.join(REPO, "_assets_output", "MyAsset.usda"))
```

Run your authoring script in the same environment (replace the path placeholder):

```
uv run --locked --group gui python <path/to/your/script.py>
```

### Ways to Write the USD Mesh

Once each part is tessellated (you have a `PolyBrep`, or a dictionary with `points`, `normals`, `faces`, and `face_normals` from `PolyBrep.to_mesh_arrays()`), there are three ways to emit the USD stage. Pick based on how much per-prim control you need.

| Method | Code weight | Control | When to use |
|---|---|---|---|
| **Hand-built via pxr** — `UsdGeom.Mesh.Define(...)` + set `points`, `faceVertexCounts`, `faceVertexIndices`, `normals`, `subdivisionScheme`, `primvars:displayColor` yourself (the template above) | ~30 lines per asset | Full. You choose the prim path (`/<AssetName>/<part>`), normals interpolation, `primvars:displayColor`, UV sets, material bindings, custom primvars, purpose, visibility. | The canonical form for this pipeline. Use when the asset needs per-part names, per-part colors, or USD material hookups — i.e. most real deliverables, and everything that must follow the section-9 output conventions. |
| **`sm.usd.export_meshes([pb0, pb1, ...], "out.usda")`** (or `export_mesh` for a single `PolyBrep`) | 1 line | Low. Prims are written under a fixed `/World` xform as `Mesh0`, `Mesh1`, … in vector order; no `displayColor`, no materials, no custom naming. The exporter also mutates per-vertex index bookkeeping on the input `PolyBrep` — copy first if you need the `PolyBrep` afterward. | Quick debug stages and one-off "does this tessellation look right?" checks where naming and color don't matter. **Not** the canonical output (violates the `/<AssetName>/<part>` convention). |
| **`sm.usd.append_meshes([pb], "scene.usda")`** | 1 line | Same as `export_meshes`, but appends to an existing stage. New prims are named `Mesh<N>`, `Mesh<N+1>`, … starting from the current stage's `UsdGeomMesh` count. `/World` is created if missing. | Layering extra parts on top of an existing USD file, or building an asset incrementally across multiple script runs. Same naming and mutation caveats as `export_meshes`. |

Rationale for keeping the hand-built pxr path as default: section 9 mandates `/<AssetName>/<part_name>` prim paths, a single root `Xform` with `SetDefaultPrim`, z-up, `metersPerUnit=0.01`, and optional `primvars:displayColor`. The `export_meshes` / `append_meshes` helpers hard-code `/World/MeshN`, so they're a shortcut, not the canonical form. Reach for them only when you know the consumer doesn't care about prim naming or color.

---

## 6. Operation Reference Map

When you need to do something beyond the primitives above, consult the matching guide in `.agents/operations/`:

| If the user wants… | Read |
|---|---|
| Basic analytic shapes (box, sphere, cylinder, cone, torus, pyramid, partial revolutions) | `.agents/operations/primitives.md` |
| Round or bevel edges | `.agents/operations/fillets.md` |
| Combine / subtract / intersect solids | `.agents/operations/booleans.md` |
| Extrude / revolve / pipe-sweep profiles | `.agents/operations/sweeps.md` |
| Thin-shell or offset the boundary | `.agents/operations/offset.md` |
| Stitch separate faces into a solid | `.agents/operations/stitching.md` |
| Intersection curves, section planes | `.agents/operations/intersectors.md`, `.agents/operations/cut_project_trim.md` |
| Build custom profiles / paths | `.agents/operations/curves.md` |
| Build custom NURBS surfaces | `.agents/operations/surfaces.md` |
| Translate / rotate / scale | `.agents/operations/transforms.md` |
| Measure volume / manifold-check / closest point | `.agents/operations/queries.md` |
| Convert BRep to mesh (parameters, trade-offs) | `.agents/operations/tessellation.md` |
| Walk / measure a `PolyBrep` | `.agents/operations/polybrep.md` |
| Mesh-level booleans (after tessellation) | `.agents/operations/polygon_booleans.md` |

The `.agents/operations/INDEX.md` has the full map with per-function listings.

---

## 7. Tessellation Parameter Guidance

The API defaults are chord height `0`, surface angle `25°`, curve angle `25°`,
maximum edge length `0`, and maximum aspect ratio `0`. The following are optional
quality overrides from `.agents/operations/tessellation.md`:

| Use | `chord_height_tolerance` | Curve / surface angles (degrees) |
|---|---|---|
| Close-up rendering, CAD | 0.001 | 5 |
| Finer general visualization | 0.01 – 0.05 | 15 |
| Preview / distant LOD | 0.1 | 30 |

World units matter: `0.01` on a 10-cm-scale asset is already quite fine. Halving the chord roughly quadruples triangle count.

---

## 8. Visualization

Among several ways to look at the finished `.usda`: pick based on whether you need a still image (chat / docs), interactive inspection (debugging tessellation or picking geometry), or full-runtime validation.

<a name="screenshot-batch-for-chat--docs"></a>

### Batch Screenshots

```
uv run --locked --group gui python tools/scripts/view_usd.py --screenshot <path/to/MyAsset.png> <path/to/MyAsset.usda>
```

Rationale: off-screen render, one file in → one PNG out. A working graphics context is still required. Best for embedding in chat replies, generating doc figures, and driving a CI visual regression test. `--view front-iso` / `front` / `top` / … picks a canonical camera; see `tools/scripts/view_usd.py --help` for the full option list (tessellation knobs, up-axis, background).

### Interactive PyVista

```
uv run --locked --group gui python tools/scripts/view_usd.py <path/to/MyAsset.usda>
```

Rationale: fastest way to inspect tessellation quality. Opens a PyVista window with a shaded and wireframe overlay and live hotkeys:

| Key | Action |
|---|---|
| `w` | Cycle display mode (shaded, shaded + wireframe, wireframe) |
| `m` | Toggle color mode (front/back orientation or material) |
| `+` / `=`, `-` | Refine / coarsen both tessellation angles |
| `[` / `]` | Refine / coarsen the curve angle only |
| `c` / `l` / `r` | Cycle chord / max-edge / max-aspect presets |
| `a` | Toggle advancing-front tessellation |
| `h` | Toggle the BRep healer |
| `n` | Cycle normals display (off, vertex, face) |
| `e` | Toggle edge-vertex dots |
| `z` | Fly to cursor |
| `f` | Frame the scene |
| `q` | Quit |

Use this while iterating on chord/angle tolerances or diagnosing cracks, gaps, or shading artifacts.

### In Omniverse Viewers

Open the generated `.usda` directly.

Rationale: validates that the asset behaves correctly in the runtime a downstream user is most likely to use — including material previews, lighting rigs, and USD composition (references, variants, payloads). The asset uses z-up and centimeter scale (`metersPerUnit=0.01`); per-part colors are optional and surface through `primvars:displayColor` when present.

---

## 9. Output Conventions

- Output folder: pick a clearly-named directory prefixed with `_`, such as `_assets_output/`, at the repo root. Create it on demand; the leading `_` is caught by the existing `_*/` gitignore rule so these folders are not tracked.
- Filenames: `<AssetName>.usda` and, if a screenshot is generated, `<AssetName>.png` alongside.
- Units: centimeter scale (`metersPerUnit=0.01`), z-up. Override only if the user explicitly asks.
- Root prim path: `/<AssetName>` (e.g. `/Robot`), default prim set to that root xform.
- Per-part prim paths: `/<AssetName>/<part_name>`.
- For reference scoring, give each mesh a stable custom `asset:kind` token describing its semantic role (for example `wheel` or `sensor`); do not use prim paths as semantic identifiers.
- One `UsdGeom.Mesh` per part. No `BrepArray`, no instancing (unless the asset is large enough that instancing is worth the complexity).
- `primvars:displayColor` is optional; omit it when the consumer assigns materials or a default display color.

---

## 10. When to Deviate from This Pipeline

This skill is for **asset authoring** — produce a polygon mesh for display. Use the full BRep round-trip (`sm.usd.export_brep` / `import_brep`) when:

- The consumer needs to modify the BRep geometry downstream (e.g. apply more fillets, booleans, or exact-CAD operations).
- You're validating / debugging the SMLib stack ↔ USD BRep conversion itself.
- The asset is part of a parametric pipeline where downstream tools consume editable BRep geometry.

For those cases, see `.agents/operations/usd_import_export.md`. Treat the round-trip as a tolerance-driven,
potentially approximating conversion, not an identity-preserving serialization. Effective BRep,
face, edge, and vertex zone tolerances collapse to one per-BRep intersection-tolerance value
derived from their maximum. Edge and wire curves are exported as NURBS. Selected analytic face
surfaces are preserved, while B-spline surfaces are exported without approximation and import may
canonicalize unclamped knot vectors; export attempts to approximate other surface types to
B-spline. Existing B-spline UV trims are eligible for export only when UV export is enabled and
the face is exported as NURBS; analytic-face trims are omitted and later regenerated as needed.
See `source/BREP_SM_USD/src/SmuConvert.cpp` and the "Current bridge normalizations and loss points"
section of the BRep format reference.

---

## 11. Quick Self-Check Before Shipping

Positive actions first, prohibitions last — same ordering as the ground rules in section 2.

- [ ] Script starts with the section-4 runtime boilerplate.
- [ ] `sm.tessellate(...)` is called once per part, results stored, USD written in a **single final step**.
- [ ] `UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)` and `SetStageMetersPerUnit(stage, 0.01)` are set.
- [ ] `stage.SetDefaultPrim(...)` is set to the root xform.
- [ ] If the asset needs per-part colors in viewers that honor `primvars:displayColor`, set them (constant interpolation is fine for a single color per part); otherwise meshes need not carry color.
- [ ] A screenshot was generated with `view_usd.py --screenshot` to confirm the asset looks right.
- [ ] Output `.usda` does not contain `BrepArray` prims.
- [ ] Every geometric operation goes through `_omni_solid` — no hand-written math.

---

## 12. Composing Multiple Assets into a Scene

Reference each finished asset under its own `Xform`; do not copy or re-tessellate geometry. Give each asset a cardinal XY `forward_axis`, rotate it about Z to the scene convention (front = `-X`), arrange the row along `+Y`, align every rear face to `X=0`, and ground it at `Z=0`. Relative references keep the scene portable.

```python
import math
import os
from pxr import Usd, UsdGeom, Gf

YAW_TO_NEG_X = {(1, 0, 0): 180, (-1, 0, 0): 0,
                (0, 1, 0): 90, (0, -1, 0): 270}

def aabb(path):
    stage = Usd.Stage.Open(path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=True)
    rng = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedBox()
    return tuple(rng.GetMin()), tuple(rng.GetMax())

def rotated_aabb(mn, mx, yaw):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    xy = [(c*x - s*y, s*x + c*y)
          for x in (mn[0], mx[0]) for y in (mn[1], mx[1])]
    return ((min(x for x, _ in xy), min(y for _, y in xy), mn[2]),
            (max(x for x, _ in xy), max(y for _, y in xy), mx[2]))

def compose(asset_usdas, forward_axes, out_path, spacing_cm=8.0):
    if len(asset_usdas) != len(forward_axes):
        raise ValueError("provide one forward_axis per asset")
    members = []
    for path, forward in zip(asset_usdas, forward_axes):
        yaw = YAW_TO_NEG_X[tuple(forward)]
        mn, mx = rotated_aabb(*aabb(path), yaw)
        members.append((path, yaw, mn, mx, mx[1] - mn[1]))
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    total_width = sum(m[4] for m in members) + spacing_cm * max(0, len(members) - 1)
    cursor = -total_width / 2.0
    scene_dir = os.path.dirname(os.path.abspath(out_path))
    for i, (path, yaw, mn, mx, width) in enumerate(members):
        xf = UsdGeom.Xform.Define(stage, f"/World/Asset_{i:02d}")
        x = UsdGeom.Xformable(xf)
        x.AddTranslateOp().Set(Gf.Vec3d(-mx[0], cursor - mn[1], -mn[2]))
        if yaw:
            x.AddRotateZOp().Set(yaw)
        relative = os.path.relpath(path, scene_dir).replace("\\", "/")
        xf.GetPrim().GetReferences().AddReference(relative)
        cursor += width + spacing_cm
    stage.GetRootLayer().Save()
```

`tools/scripts/view_usd.py --screenshot` can render the composed stage. Add authored camera, lights, or a ground plane only when the deliverable needs them.

---

## 13. Sanity-Checking a Multi-Part or Composed Asset

Before shipping a multi-part asset (or a composed scene), two **cheap bounding-box checks** catch the most common failures without paying for boolean evaluation. Both operate on the per-part `points` arrays you already have from `PolyBrep.to_mesh_arrays()` (or from each mesh prim's world bound), so they cost microseconds.

- **Gross interpenetration.** Compare each candidate or invented part only against the known-good anchor parts. Shrink each box by a small `tol` first so a coincident-face kiss between neighbours is not flagged, and ignore overlaps below a `min_volume` noise floor. Avoid an all-pairs check: intentional overlaps among trusted assembly parts are common.
- **Envelope containment.** When some parts are known-good "anchors" and others are looser (e.g. filled-in detail), take the union AABB of the anchors, expand it by a growth factor (say 30%), and flag any part that pokes outside. This keeps invented detail from ballooning past the real silhouette.

```python
import numpy as np

def sanity_checks(parts, anchor_names, growth=0.30,
                  penetration_tol=0.05, envelope_tol=0.5, min_volume=0.5):
    # parts: [(name, points)]; returns penetrations and envelope breaches.
    anchor_names = set(anchor_names)
    boxes = {}
    for name, points in parts:
        points = np.asarray(points, dtype=np.float64)
        boxes[name] = (points.min(axis=0), points.max(axis=0))
    candidates = set(boxes) - anchor_names

    penetrations = []
    for candidate in sorted(candidates):
        for anchor in sorted(anchor_names & set(boxes)):
            a_mn, a_mx = boxes[candidate]
            b_mn, b_mx = boxes[anchor]
            extent = np.maximum(
                0, np.minimum(a_mx-penetration_tol, b_mx-penetration_tol)
                - np.maximum(a_mn+penetration_tol, b_mn+penetration_tol))
            volume = float(np.prod(extent))
            if volume > min_volume:
                penetrations.append((candidate, anchor, volume))

    anchors = [boxes[name] for name in anchor_names if name in boxes]
    if not anchors:
        return penetrations, []
    env_min = np.min([b[0] for b in anchors], axis=0)
    env_max = np.max([b[1] for b in anchors], axis=0)
    pad = (env_max - env_min) * growth
    env_min, env_max = env_min - pad, env_max + pad
    breaches = []
    for name in sorted(candidates):
        part_min, part_max = boxes[name]
        for axis in range(3):
            if part_min[axis] < env_min[axis] - envelope_tol:
                breaches.append((name, axis, "min"))
            if part_max[axis] > env_max[axis] + envelope_tol:
                breaches.append((name, axis, "max"))
    return penetrations, breaches
```

These are advisory: log the offenders, ask for a revision, or reject — but they are not a substitute for `is_manifold_solid()` / `volume()` checks on the individual BReps (section 6, `.agents/operations/queries.md`).

---

<a name="14-optional-score-a-result-against-a-reference-usd"></a>

## 14. Score a Result Against a Reference USD

This comparison is optional.

When you have a **reference USD** to compare against (a previous good build, or a third-party model), a framework-light three-axis comparator gives a useful first-pass quality number using only bounding boxes and part counts — no Chamfer, scipy, or cv2. Author the `asset:kind` token shown in section 5 and match **by kind, not by prim path**, so naming differences do not sink the score:

- **Geometric** — mean per-pair bbox IoU after greedy nearest-centre pairing within each kind bucket; each unmatched part contributes zero. Keep asset-level bbox IoU as a separate diagnostic rather than counting it again in the overall score.
- **Topological** — per-kind count parity: `min(n_a, n_b) / max(n_a, n_b)`, averaged over kinds (so rare kinds aren't drowned out).
- **Semantic** — the *set* of kinds present, as a Jaccard index (did we build the right roles at all).

```python
RUBRIC_VERSION = "bbox-kind-v1"

def bbox_iou(a_mn, a_mx, b_mn, b_mx):
    inter = 1.0
    for i in range(3):
        d = min(a_mx[i], b_mx[i]) - max(a_mn[i], b_mn[i])
        if d <= 0.0:
            return 0.0
        inter *= d
    va = (a_mx[0]-a_mn[0])*(a_mx[1]-a_mn[1])*(a_mx[2]-a_mn[2])
    vb = (b_mx[0]-b_mn[0])*(b_mx[1]-b_mn[1])*(b_mx[2]-b_mn[2])
    return inter / (va + vb - inter) if (va + vb - inter) > 0 else 0.0
```

To reproduce the score:

1. Inventory `UsdGeom.Mesh` prims and their world AABBs with `UsdGeom.BBoxCache`; multiply coordinates by `UsdGeom.GetStageMetersPerUnit(stage)` before comparing.
2. Read each prim's `asset:kind`. For an untagged reference, resolve prim names through a stable `{canonical_kind: [fnmatch_glob, ...]}` map.
3. Within each kind, sort both inventories by `(prim_name, prim_path)`. Iterate the built list and match each part to the unmatched reference minimizing `(centre_distance, prim_name, prim_path)`.
4. Geometric score is matched IoU sum divided by `max(total_built, total_reference)`; unmatched parts therefore contribute zero. Topological and semantic scores use the formulas above. Any axis with an empty union scores `1.0`; average the three axes and report default-prim bbox IoU separately.

Compare mesh USDs, not non-geometric BRep roots. Keep synonyms stable; do not tune them for one asset. Bump `RUBRIC_VERSION` whenever inventory, matching, or scoring semantics change.

---

## 15. Reference-Driven Modeling Discipline

Treat dimensions and appearance as separate evidence:

1. Prefer authoritative drawings, datasheets, manuals, or URDFs for dimensions. Record the source and a confidence value beside each key dimension in the asset script or a small adjacent JSON file.
2. Use photographs for proportions, color, and silhouette only when no dimensioned source exists. Reject references that are heavily occluded, cropped, too small, or show an ambiguous view.
3. Build from the source constraints before looking at comparator scores. Never back-fit dimensions from the reference USD, and never edit synonyms or the scoring rubric during the run to improve a score.
4. If validation fails, revisit the weakest source-backed constraints once; otherwise report the uncertainty rather than inventing precision.

For image-based validation:

1. Render axis-aligned `front`, `back`, `left`, `right`, and `top` views with `view_usd.py`.
2. Compare each clean reference against every plausible corresponding view (for example front and back when the photograph is ambiguous), and keep the best valid pairing.
3. Extract a foreground mask from alpha or a simple background. Reject the mask if:
   - it covers less than 1% or more than 80% of the image;
   - it has no single dominant component; or
   - the silhouette touches all four edges.
4. Record silhouette IoU and the symmetric contour distance normalized by the image diagonal.
5. Save an overlay so a human can see where the model differs.
