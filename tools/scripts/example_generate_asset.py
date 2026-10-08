#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Minimal reference implementation of the "Generate 3D Asset" pipeline.

Full guide: ``.agents/docs/generate_asset.md``.

Pipeline:

    1. Build a small parametric asset with SMLib analytic primitives
       (box + fillet + sphere + cylinder) via the ``_omni_solid``
       Python bindings.
    2. Tessellate each BRep in SMLib via ``sm.tessellate()``.
    3. Emit a single mesh-only USDA file: one ``UsdGeom.Mesh`` per part,
       grouped under a root ``Xform``. Optional per-part ``primvars:displayColor``.

No ``BrepArray`` prims are ever written. No geometry code is reimplemented.

Usage (Windows PowerShell; adapt separators for POSIX)::

    _build\\target-deps\\python\\python.exe tools\\scripts\\example_generate_asset.py

Options::

    --output <path.usda>        Output USDA path (default: ./_assets_output/ExampleAsset.usda)
    --screenshot <path.png>     Also render a PNG via tools/scripts/view_usd.py
    --chord <float>             Chord-height tolerance for tessellation (default: library default)
    --angle <float>             Angle tolerance in degrees     (default: library default)
"""

import argparse
import os
import subprocess
import sys

# ---------------------------------------------------------------------------
# 1. Runtime environment boilerplate
# ---------------------------------------------------------------------------
#
# Locate the repo root from this script's location, then reuse the helpers
# in ``tools/scripts/view_usd.py`` to set up USD DLL paths and pick the
# right ``_omni_solid.pyd`` (release preferred over debug).

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO        = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", ".."))

if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import view_usd                                        # noqa: E402
view_usd._ensure_usd_runtime_env()

_CONFIG    = view_usd._pick_config(REPO)
_PLATFORM  = view_usd._pick_platform()
_BUILD_BIN = os.path.join(REPO, "_build", _PLATFORM, _CONFIG)
if os.path.isdir(_BUILD_BIN) and _BUILD_BIN not in sys.path:
    sys.path.insert(0, _BUILD_BIN)

import _omni_solid as sm                               # noqa: E402
from pxr import Usd, UsdGeom, Sdf, Vt, Gf              # noqa: E402


# ---------------------------------------------------------------------------
# 2. Build parts in SMLib (exact NURBS / analytic primitives)
# ---------------------------------------------------------------------------

def build_parts():
    """Build a tiny demo asset: a filleted base, a ball on top, and a post.

    Returns a list of ``{name, brep, color?}`` dicts. ``color`` is optional;
    if omitted, no ``displayColor`` primvar is written for that mesh.
    """
    parts = []

    def add(name, brep, color=None):
        parts.append(dict(name=name, brep=brep, color=color))

    base = sm.create_box((-5, -5, 0), 10, 10, 3)
    sm.fillet_edges(base, base.edges(), radius=0.5)
    add("base", base, (0.30, 0.60, 0.95))

    add("post",  sm.create_cylinder((0, 0, 3), 0.8, 6.0), (0.70, 0.70, 0.75))
    add("ball",  sm.create_sphere((0, 0, 10.0), 1.5),      (1.00, 0.20, 0.20))
    add("collar", sm.create_torus((0, 0, 3.2), 1.2, 0.25), (1.00, 0.85, 0.00))

    return parts


# ---------------------------------------------------------------------------
# 3. Tessellate each BRep in SMLib
# ---------------------------------------------------------------------------

def tessellate_parts(parts, **overrides):
    # Library defaults (sm.tessellation_defaults()) unless overridden, e.g.
    # chord_height_tolerance=..., curve_angle_tolerance_deg=...
    for p in parts:
        polybrep = sm.tessellate(p["brep"], **overrides)
        p["mesh"] = polybrep.to_mesh_arrays(face_varying_normals=True)
    return parts


# ---------------------------------------------------------------------------
# 4. Write a single mesh-only USDA (UsdGeom.Mesh only, no BrepArray)
# ---------------------------------------------------------------------------

def write_usda(parts, out_path, root_name="ExampleAsset"):
    out_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(out_dir, exist_ok=True)
    if os.path.exists(out_path):
        os.remove(out_path)

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)

    root_path = f"/{root_name}"
    root = UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(root.GetPrim())

    for p in parts:
        name  = p["name"]
        mesh  = p["mesh"]
        color = p.get("color")

        m = UsdGeom.Mesh.Define(stage, f"{root_path}/{name}")

        pts = mesh["points"]
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*pt) for pt in pts]))

        # ``faces`` is flat: [n, i0, i1, ..., n, j0, j1, ...]
        flat = mesh["faces"]
        counts, indices = [], []
        i = 0
        while i < len(flat):
            n = int(flat[i])
            counts.append(n)
            indices.extend(int(x) for x in flat[i + 1: i + 1 + n])
            i += 1 + n
        m.CreateFaceVertexCountsAttr (Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))

        normals = mesh.get("normals")
        if normals and len(normals) == len(indices):
            m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in normals]))
            m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)

        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

        if color is not None:
            cp = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray,
                interpolation=UsdGeom.Tokens.constant)
            cp.Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))

    stage.GetRootLayer().Save()
    return out_path


# ---------------------------------------------------------------------------
# 5. Optional: screenshot via tools/scripts/view_usd.py
# ---------------------------------------------------------------------------

def render_screenshot(usda_path, png_path):
    python_exe = os.path.join(REPO, "_build", "target-deps", "python",
                              "python.exe" if sys.platform == "win32" else "python")
    if not os.path.isfile(python_exe):
        python_exe = sys.executable
    cmd = [python_exe,
           os.path.join(REPO, "tools", "scripts", "view_usd.py"),
           "--screenshot", png_path,
           usda_path]
    print("  +", " ".join(f'"{c}"' if " " in c else c for c in cmd))
    subprocess.check_call(cmd)
    return png_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--output", default=os.path.join(REPO, "_assets_output",
                                                         "ExampleAsset.usda"),
                        help="Output USDA path.")
    parser.add_argument("--screenshot", default=None,
                        help="If set, also render a PNG to this path.")
    parser.add_argument("--chord", type=float, default=None,
                        help="Tessellation chord-height tolerance (default: library default).")
    parser.add_argument("--angle", type=float, default=None,
                        help="Tessellation curve and surface angle tolerance in degrees "
                             "(default: library default).")
    args = parser.parse_args()

    print(f"[env] config={_CONFIG!r}  build={_BUILD_BIN}")

    print("[1/3] Building parts in SMLib")
    parts = build_parts()
    nf = sum(len(p["brep"].faces())    for p in parts)
    ne = sum(len(p["brep"].edges())    for p in parts)
    nv = sum(len(p["brep"].vertices()) for p in parts)
    print(f"      {len(parts)} parts  ({nf} NURBS faces, {ne} edges, {nv} vertices)")

    overrides = {}
    if args.chord is not None:
        overrides["chord_height_tolerance"] = args.chord
    if args.angle is not None:
        overrides["curve_angle_tolerance_deg"] = args.angle
        overrides["surface_angle_tolerance_deg"] = args.angle
    tess = {**sm.tessellation_defaults(), **overrides}
    print(f"[2/3] Tessellating in SMLib (chord={tess['chord_height_tolerance']}, "
          f"curve angle={tess['curve_angle_tolerance_deg']} deg, "
          f"surface angle={tess['surface_angle_tolerance_deg']} deg)")
    tessellate_parts(parts, **overrides)
    mv = sum(len(p["mesh"]["points"])        for p in parts)
    mf = sum(len(p["mesh"]["face_normals"]) for p in parts)
    print(f"      {mv} mesh verts, {mf} polygons total")

    print(f"[3/3] Writing mesh-only USDA -> {args.output}")
    write_usda(parts, args.output)

    if args.screenshot:
        print(f"[+]   Rendering screenshot -> {args.screenshot}")
        render_screenshot(args.output, args.screenshot)

    print("Done.")


if __name__ == "__main__":
    main()
