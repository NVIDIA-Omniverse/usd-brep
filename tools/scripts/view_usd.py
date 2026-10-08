#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
view_usd.py - Tessellate USD BReps and/or display UsdGeom.Mesh prims interactively.

Workflow:
  1. Loads UsdGeom.Mesh prims from the USD stage
  2. Calls smtess_usd to tessellate BRepArray prims via SmTess
  3. Displays combined meshes with pyvista in shaded + wireframe overlay

Usage:
  python view_usd.py <input.usd> [--surface-angle 25] [--curve-angle 25]

Requirements:
  pip install pyvista numpy
  Build smtess_usd (premake5 + make); solidmodeling build tree for pxr
"""

import argparse
import collections
import os
import platform
import signal
import subprocess
import sys
import time


def _find_repo_root():
    """Return the repo root (two levels up from this script)."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(script_dir, "..", ".."))


def _pick_platform():
    """Return the ``_build/<platform>`` directory name for the current host.

    Mapping:
      * Windows     -> ``windows-x86_64``
      * macOS       -> ``macos-universal``
      * Linux ARM64 -> ``linux-aarch64``  (``platform.machine()`` is
        ``aarch64`` on glibc and ``arm64`` on some distributions)
      * Linux x86   -> ``linux-x86_64``
    """
    if sys.platform == "win32":
        return "windows-x86_64"
    if sys.platform == "darwin":
        return "macos-universal"
    machine = platform.machine().lower()
    return "linux-aarch64" if machine in ("aarch64", "arm64") else "linux-x86_64"


def _pick_config(repo):
    """Return a build config, honoring ``SMLIB_BUILD_CONFIG`` when set.

    Otherwise return 'release' if a release build has actual binaries,
    otherwise 'debug'.

    Probing for the directory alone is not enough: a stale empty `release/`
    folder (e.g. holding only ``compile_commands.json``) is common when the
    user has only ever built debug.  We test for at least one real build
    artifact (the smtess_usd binary or the omniSolid plugins dir).
    """
    forced = os.environ.get("SMLIB_BUILD_CONFIG", "").strip().lower()
    if forced in ("debug", "release"):
        return forced

    plat = _pick_platform()
    sm_exe = "smtess_usd.exe" if sys.platform == "win32" else "smtess_usd"

    def _has_build(cfg):
        bdir = os.path.join(repo, "_build", plat, cfg)
        return os.path.isfile(os.path.join(bdir, sm_exe))

    if _has_build("release"):
        return "release"
    if _has_build("debug"):
        return "debug"
    return "release"


def _ensure_usd_runtime_env():
    """Prepend repo USD bindings to sys.path and linker search path so ``pxr`` loads."""
    repo = _find_repo_root()
    cfg = _pick_config(repo)
    usd_python = os.path.join(
        repo, "_build", "target-deps", "usd", cfg, "lib", "python")
    if os.path.isdir(usd_python):
        ap = os.path.abspath(usd_python)
        if ap not in sys.path:
            sys.path.insert(0, ap)

    if sys.platform == "win32":
        build_dir = os.path.join(repo, "_build", "windows-x86_64", cfg)
        usd_lib = os.path.join(repo, "_build", "target-deps", "usd", cfg, "lib")
        usd_bin = os.path.join(repo, "_build", "target-deps", "usd", cfg, "bin")
        py_dir = os.path.join(repo, "_build", "target-deps", "python")
        parts = [d for d in (build_dir, usd_lib, usd_bin, py_dir) if os.path.isdir(d)]
        if parts:
            existing = os.environ.get("PATH", "")
            os.environ["PATH"] = ";".join(parts) + (";" + existing if existing else "")
        # Python 3.8+ on Windows requires explicit DLL search dirs for native
        # extension modules (e.g. _omni_solid.pyd, pxr's _tf.pyd) — modifying
        # PATH alone is not sufficient.
        if hasattr(os, "add_dll_directory"):
            for d in parts:
                try:
                    os.add_dll_directory(d)
                except (OSError, FileNotFoundError):
                    pass
    elif sys.platform == "darwin":
        plat = "macos-universal"
        build_dir = os.path.join(repo, "_build", plat, cfg)
        usd_lib = os.path.join(repo, "_build", "target-deps", "usd", cfg, "lib")
        parts = [d for d in (build_dir, usd_lib) if os.path.isdir(d)]
        if parts:
            existing = os.environ.get("DYLD_LIBRARY_PATH", "")
            os.environ["DYLD_LIBRARY_PATH"] = (
                ":".join(parts) + (":" + existing if existing else ""))
    else:
        plat = _pick_platform()
        build_dir = os.path.join(repo, "_build", plat, cfg)
        usd_lib = os.path.join(repo, "_build", "target-deps", "usd", cfg, "lib")
        parts = [d for d in (build_dir, usd_lib) if os.path.isdir(d)]
        if parts:
            existing = os.environ.get("LD_LIBRARY_PATH", "")
            os.environ["LD_LIBRARY_PATH"] = (
                ":".join(parts) + (":" + existing if existing else ""))

    plugin = os.path.join(repo, "_build", "schema", "omniSolid", "resources")
    if os.path.isdir(plugin) and not os.environ.get("OMNISOLID_PLUGIN_PATH"):
        os.environ["OMNISOLID_PLUGIN_PATH"] = plugin


_ensure_usd_runtime_env()

try:
    from pxr import Usd, UsdGeom
except ImportError as e:
    repo = _find_repo_root()
    cfg = _pick_config(repo)
    usd_py = os.path.join(
        repo, "_build", "target-deps", "usd", cfg, "lib", "python")
    bundled = os.path.join(repo, "_build", "target-deps", "python", "bin", "python3")
    script = os.path.abspath(__file__)
    # Only when run directly: re-running an importer would replace it with this script.
    if (
        __name__ == "__main__"
        and os.path.isfile(bundled)
        and not os.environ.get("VIEW_USD_INTERNAL_REEXEC")
        and os.path.realpath(sys.executable) != os.path.realpath(bundled)
    ):
        print(
            f"view_usd: re-running with repo Python (pxr needs matching ABI): {bundled}",
            file=sys.stderr,
        )
        os.environ["VIEW_USD_INTERNAL_REEXEC"] = "1"
        os.execv(bundled, [bundled, script] + sys.argv[1:])
    msg = [
        f"USD Python (pxr) failed to import: {e}",
        f"Expected bindings at (exists={os.path.isdir(usd_py)}): {usd_py}",
    ]
    if sys.platform.startswith("linux"):
        libs = [os.path.join(repo, "_build", _pick_platform(), cfg),
                os.path.join(repo, "_build", "target-deps", "usd", cfg, "lib")]
        msg.append(f"Set LD_LIBRARY_PATH before starting Python: {':'.join(libs)}")
    if os.path.isfile(bundled):
        msg.append(
            f"Install deps for that interpreter: "
            f"{bundled} -m pip install numpy pyvista"
        )
    else:
        msg.append(
            "Run ./build.sh so target-deps (USD) is populated, or set PYTHONPATH."
        )
    sys.exit("\n".join(msg))

try:
    import numpy as np
except ImportError:
    bundled = os.path.join(_find_repo_root(), "_build", "target-deps", "python", "bin", "python3")
    if os.path.realpath(sys.executable) == os.path.realpath(bundled):
        sys.exit(
            f"numpy is required. Install into repo Python:\n  {bundled} -m pip install numpy pyvista"
        )
    print("numpy is required.", file=sys.stderr)
    print("Install it with: pip install numpy pyvista", file=sys.stderr)
    sys.exit(1)

try:
    import pyvista as pv
except ImportError:
    bundled = os.path.join(_find_repo_root(), "_build", "target-deps", "python", "bin", "python3")
    if os.path.realpath(sys.executable) == os.path.realpath(bundled):
        sys.exit(
            f"pyvista is required. Install into repo Python:\n  {bundled} -m pip install numpy pyvista"
        )
    print("pyvista is required.", file=sys.stderr)
    print("Install it with: pip install numpy pyvista", file=sys.stderr)
    sys.exit(1)

import vtk

# Traverse instance proxies so UsdGeom.Mesh / BrepArray under USD instances are found.
_USD_TRAVERSE_INSTANCES = Usd.TraverseInstanceProxies()


def _find_smtess_usd():
    """Locate the smtess_usd binary."""
    repo_root = _find_repo_root()
    exe = "smtess_usd.exe" if sys.platform == "win32" else "smtess_usd"
    candidates = [
        os.path.join(repo_root, "_build", "linux-x86_64", "release", exe),
        os.path.join(repo_root, "_build", "linux-x86_64", "debug", exe),
        os.path.join(repo_root, "_build", "linux-aarch64", "release", exe),
        os.path.join(repo_root, "_build", "linux-aarch64", "debug", exe),
        os.path.join(repo_root, "_build", "windows-x86_64", "release", exe),
        os.path.join(repo_root, "_build", "windows-x86_64", "debug", exe),
        os.path.join(repo_root, "_build", "macos-universal", "release", exe),
        os.path.join(repo_root, "_build", "macos-universal", "debug", exe),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def _build_library_path():
    """Build platform-appropriate library search path for the subprocess."""
    repo_root = _find_repo_root()
    cfg = _pick_config(repo_root)
    if sys.platform == "win32":
        build_dir = os.path.join(repo_root, "_build", "windows-x86_64", cfg)
        usd_lib = os.path.join(repo_root, "_build", "target-deps", "usd", cfg, "lib")
        usd_bin = os.path.join(repo_root, "_build", "target-deps", "usd", cfg, "bin")
        python_dir = os.path.join(repo_root, "_build", "target-deps", "python")
        dirs = [d for d in [build_dir, usd_lib, usd_bin, python_dir] if os.path.isdir(d)]
        existing = os.environ.get("PATH", "")
        return "PATH", ";".join(dirs) + (";" + existing if existing else "")
    else:
        plat = _pick_platform()
        build_dir = os.path.join(repo_root, "_build", plat, cfg)
        usd_lib = os.path.join(repo_root, "_build", "target-deps", "usd", cfg, "lib")
        dirs = [d for d in [build_dir, usd_lib] if os.path.isdir(d)]
        var = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
        existing = os.environ.get(var, "")
        return var, ":".join(dirs) + (":" + existing if existing else "")


def _build_omnisolid_plugin_path():
    """Return OMNISOLID_PLUGIN_PATH if not already set."""
    if os.environ.get("OMNISOLID_PLUGIN_PATH"):
        return os.environ["OMNISOLID_PLUGIN_PATH"]
    repo_root = _find_repo_root()
    plugin_path = os.path.join(repo_root, "_build", "schema", "omniSolid", "resources")
    if os.path.isdir(plugin_path):
        return plugin_path
    return ""


def _read_binary_meshes(data):
    """Parse smtess_usd --binary (v2/v3) output.

    Returns ``{"unique": [mesh dict, ...], "instances": [{...}, ...]}``:

    * each *unique* mesh dict carries prototype-local geometry
      (points/normals/faces/face_centers/face_normals/edge_vertex_indices/
      lines/failed_faces) plus its ``proto_index``;
    * each *instance* references a unique prototype by ``proto_index`` and
      supplies a 4x4 world ``xform`` (USD row-vector convention: world = pt * M).

    Instanced geometry is thus stored once and drawn many times. Numeric arrays
    are mapped with ``numpy.frombuffer`` (no per-value Python objects). See
    WriteMeshesBinaryInstanced() in smtess_usd_main.cpp for the byte layout.
    """
    import struct

    if len(data) < 12 or data[:4] != b"SMTB":
        raise ValueError("bad magic (expected 'SMTB')")
    # Native byte order ("=") matches the writer's raw host-endian bytes.
    (version,) = struct.unpack_from("=I", data, 4)
    if version not in (2, 3):
        raise ValueError(f"unsupported binary version {version} (expected 2 or 3)")
    # v3 adds a `lines` array per mesh body (edge-curve polylines from
    # --wireframe); v2 files have no trailing lines field.
    has_lines = version >= 3

    mv = memoryview(data)

    def take_floats(o):
        (n,) = struct.unpack_from("=I", data, o)
        o += 4
        return np.frombuffer(mv, dtype="=f4", count=n, offset=o), o + n * 4

    def read_body(o):
        (name_len,) = struct.unpack_from("=I", data, o)
        o += 4
        name = bytes(mv[o:o + name_len]).decode("utf-8", "replace")
        o += name_len
        (failed,) = struct.unpack_from("=B", data, o)
        o += 1
        points, o = take_floats(o)
        normals, o = take_floats(o)
        (nfaces,) = struct.unpack_from("=I", data, o)
        o += 4
        faces = np.frombuffer(mv, dtype="=i4", count=nfaces, offset=o)
        o += nfaces * 4
        face_centers, o = take_floats(o)
        face_normals, o = take_floats(o)
        (nevi,) = struct.unpack_from("=I", data, o)
        o += 4
        evi = np.frombuffer(mv, dtype="=u4", count=nevi, offset=o)
        o += nevi * 4
        if has_lines:
            (nlines,) = struct.unpack_from("=I", data, o)
            o += 4
            lines = np.frombuffer(mv, dtype="=i4", count=nlines, offset=o)
            o += nlines * 4
        else:
            lines = np.empty(0, dtype="=i4")
        return {
            "name": name,
            "points": points.reshape(-1, 3),
            "normals": normals.reshape(-1, 3),
            "faces": faces,
            "face_centers": face_centers.reshape(-1, 3),
            "face_normals": face_normals.reshape(-1, 3),
            "edge_vertex_indices": evi,
            "lines": lines,
            "failed_faces": bool(failed),
        }, o

    off = 8
    (num_unique,) = struct.unpack_from("=I", data, off)
    off += 4
    unique = []
    for _ in range(num_unique):
        (proto_index,) = struct.unpack_from("=I", data, off)
        off += 4
        body, off = read_body(off)
        body["proto_index"] = proto_index
        unique.append(body)

    (num_inst,) = struct.unpack_from("=I", data, off)
    off += 4
    instances = []
    for _ in range(num_inst):
        (proto_index,) = struct.unpack_from("=I", data, off)
        off += 4
        (name_len,) = struct.unpack_from("=I", data, off)
        off += 4
        name = bytes(mv[off:off + name_len]).decode("utf-8", "replace")
        off += name_len
        xform = np.frombuffer(mv, dtype="=f4", count=16, offset=off).reshape(4, 4).astype(np.float64)
        off += 16 * 4
        instances.append({"name": name, "proto_index": proto_index, "xform": xform})

    return {"unique": unique, "instances": instances}


def tessellate_with_smtess(input_path, surface_angle=25.0, curve_angle=25.0,
                           chord_tolerance=0.0, max_edge_length=0.0,
                           max_aspect_ratio=0.0, smooth=False,
                           advancing_front=False, heal=False, max_breps=0,
                           workers=0, wireframe_only=False, timing_path=None):
    """Call smtess_usd to tessellate and return (mesh_dicts, error_string).

    ``wireframe_only`` samples each Brep's edge curves directly (--wireframe)
    instead of running full surface tessellation -- no SmSurfaceCache
    subdivision, so it stays fast even for breps whose normal tessellation
    is pathologically slow (see docs/vega_osfp_tessellation_failures.md).

    ``timing_path`` writes import/heal and tessellation timing JSONL.

    Returns (list, None) on success, (None, str) on failure.
    """
    tool = _find_smtess_usd()
    if tool is None:
        msg = "smtess_usd binary not found. Build it first."
        print(f"Error: {msg}", file=sys.stderr)
        return None, msg

    # Write the compact binary blob to a temp file via smtess_usd's --output
    # flag rather than capturing it from the pipe.  Capturing large pipe output
    # on Windows is unreliable through subprocess.run.  The binary format lets
    # the reader map the arrays with numpy.frombuffer instead of parsing
    # millions of floats, turning a multi-GB / multi-second step into a small,
    # near-instant one.
    import tempfile
    fd, tmp_out = tempfile.mkstemp(suffix=".smtb", prefix="smtess_")
    os.close(fd)

    cmd = [
        tool, input_path,
        "-o", tmp_out,
        "--surface-angle", str(surface_angle),
        "--curve-angle", str(curve_angle),
        "--chord-tolerance", str(chord_tolerance),
        "--max-edge-length", str(max_edge_length),
        "--max-aspect-ratio", str(max_aspect_ratio),
    ]
    if smooth:
        cmd.append("--smooth")
    if advancing_front:
        cmd.append("--advancing-front")
    if heal:
        cmd.append("--heal")
    if max_breps and max_breps > 0:
        cmd += ["--max-breps", str(max_breps)]
    if workers and workers > 0:
        cmd += ["--workers", str(workers)]
    if wireframe_only:
        cmd.append("--wireframe")
    if timing_path:
        cmd += ["--timing", timing_path]

    env = os.environ.copy()
    lib_var, lib_val = _build_library_path()
    # On Windows, os.environ.copy() yields a case-sensitive dict whose keys
    # may use mixed case (e.g. "Path").  Setting env["PATH"] would add a
    # duplicate instead of overwriting.  Remove any case-variant first.
    for key in list(env.keys()):
        if key.upper() == lib_var.upper():
            del env[key]
    env[lib_var] = lib_val
    omnisolid = _build_omnisolid_plugin_path()
    if omnisolid:
        env["OMNISOLID_PLUGIN_PATH"] = omnisolid

    try:
        try:
            # No timeout: full-scale models can legitimately run for a long time.
            proc = subprocess.run(
                cmd, capture_output=True, text=True, env=env)
        except FileNotFoundError:
            msg = f"cannot execute {tool}"
            print(f"Error: {msg}", file=sys.stderr)
            return None, msg

        if proc.stderr:
            for line in proc.stderr.strip().split("\n"):
                print(f"  [smtess] {line}", file=sys.stderr, flush=True)

        if proc.returncode != 0:
            code = proc.returncode
            if code < 0:
                sig_names = {-11: "SIGSEGV", -6: "SIGABRT", -8: "SIGFPE", -9: "SIGKILL"}
                label = sig_names.get(code, f"signal {-code}")
                msg = f"smtess_usd crashed ({label})"
            else:
                msg = f"smtess_usd failed (exit code {code})"
            print(f"Error: {msg}", file=sys.stderr)
            return None, msg

        try:
            with open(tmp_out, "rb") as f:
                data = f.read()
        except OSError as e:
            msg = f"smtess_usd produced no output file: {e}"
            print(f"Error: {msg}", file=sys.stderr)
            return None, msg

        if not data:
            return None, "smtess_usd produced empty output file"

        try:
            return _read_binary_meshes(data), None
        except Exception as e:
            msg = f"failed to parse smtess_usd binary output: {e}"
            print(f"Error: {msg}", file=sys.stderr)
            return None, msg
    finally:
        try:
            os.remove(tmp_out)
        except OSError:
            pass


MeshEntry = collections.namedtuple(
    "MeshEntry", "name poly kind xform pick_ranges")
MeshEntry.__new__.__defaults__ = (None, None)
MeshEntry.__doc__ = """A renderable mesh with optional transform and pick ranges."""


_BATCH_TARGET_POINTS = 250_000
_BATCH_TARGET_CELLS = 250_000


def _legacy_cells_with_offset(cells, point_offset):
    """Copy legacy VTK cells and offset their point indices."""
    out = np.asarray(cells, dtype=np.int64).copy()
    if len(out) == 0:
        return out, 0

    width = int(out[0]) + 1
    if width > 1 and len(out) % width == 0 and np.all(out[::width] == width - 1):
        shaped = out.reshape(-1, width)
        shaped[:, 1:] += point_offset
        return out, len(shaped)

    count_positions = []
    pos = 0
    while pos < len(out):
        count_positions.append(pos)
        pos += int(out[pos]) + 1
    if pos != len(out):
        raise ValueError("invalid VTK legacy cell array")
    index_mask = np.ones(len(out), dtype=bool)
    index_mask[count_positions] = False
    out[index_mask] += point_offset
    return out, len(count_positions)


class _MeshBatch:
    """Accumulate placed meshes and materialize one world-space PolyData."""

    def __init__(self, failed_faces, line_mesh):
        self.failed_faces = failed_faces
        self.line_mesh = line_mesh
        self.points = []
        self.cells = []
        self.point_normals = []
        self.face_centers = []
        self.face_normals = []
        self.edge_vertex_indices = []
        self.pick_ranges = []
        self.n_points = 0
        self.n_cells = 0
        self.all_point_normals = True
        self.all_face_data = True

    def full(self):
        return (self.n_points >= _BATCH_TARGET_POINTS
                or self.n_cells >= _BATCH_TARGET_CELLS)

    def add(self, mesh_data, name, user_matrix):
        points = np.asarray(mesh_data["points"], dtype=np.float32)
        faces = np.asarray(mesh_data["faces"], dtype=np.int32)
        lines = np.asarray(mesh_data.get("lines", []), dtype=np.int32)
        cells = faces if len(faces) else lines
        placed_cells, n_cells = _legacy_cells_with_offset(cells, self.n_points)
        if len(points) == 0 or n_cells == 0:
            return

        self.points.append(_xform_points(points, user_matrix))
        self.cells.append(placed_cells)

        normals = mesh_data.get("normals")
        if normals is not None and len(normals) == len(points):
            self.point_normals.append(_xform_dirs(normals, user_matrix))
        else:
            self.all_point_normals = False

        fc = mesh_data.get("face_centers")
        fn = mesh_data.get("face_normals")
        if len(faces) and fc is not None and fn is not None and len(fc) == n_cells and len(fn) == n_cells:
            self.face_centers.append(_xform_points(fc, user_matrix))
            self.face_normals.append(_xform_dirs(fn, user_matrix))
        elif len(faces):
            self.all_face_data = False

        evi = mesh_data.get("edge_vertex_indices")
        if evi is not None and len(evi):
            self.edge_vertex_indices.append(
                np.asarray(evi, dtype=np.int64) + self.n_points)

        first_cell = self.n_cells
        self.n_points += len(points)
        self.n_cells += n_cells
        self.pick_ranges.append((first_cell, self.n_cells, name))

    def finish(self, batch_index):
        points = np.vstack(self.points)
        cells = np.concatenate(self.cells)
        poly = (pv.PolyData(points, lines=cells) if self.line_mesh
                else pv.PolyData(points, faces=cells))

        if self.all_point_normals and self.point_normals:
            poly.point_data["Normals"] = np.vstack(self.point_normals)
            poly.GetPointData().SetActiveNormals("Normals")
        if self.all_face_data and self.face_centers:
            poly.cell_data["FaceCenters"] = np.vstack(self.face_centers)
            poly.cell_data["FaceNormals"] = np.vstack(self.face_normals)
        if self.edge_vertex_indices:
            poly.field_data["EdgeVertexIndices"] = np.concatenate(
                self.edge_vertex_indices)
        if self.failed_faces:
            poly.field_data["FailedFaces"] = np.array([1], dtype=np.int8)

        return MeshEntry(
            f"BRepArray batch {batch_index}", poly, "BrepArray", None,
            tuple(self.pick_ranges))


def build_instanced_meshes(payload):
    """Bake instances into bounded world-space mesh batches."""
    if not payload:
        return None
    unique = payload.get("unique") or []
    instances = payload.get("instances") or []

    proto_to_meshes = {}
    for mesh_data in unique:
        if len(mesh_data["points"]) == 0:
            continue
        if len(mesh_data["faces"]) == 0 and len(mesh_data.get("lines", [])) == 0:
            continue
        proto_to_meshes.setdefault(mesh_data["proto_index"], []).append(mesh_data)

    entries = []
    batches = {}

    def flush(key):
        batch = batches.pop(key)
        if batch.n_cells:
            entries.append(batch.finish(len(entries)))

    for inst in instances:
        meshes = proto_to_meshes.get(inst["proto_index"])
        if not meshes:
            continue
        # Convert the USD row-vector matrix to VTK convention.
        user_matrix = np.asarray(inst["xform"], dtype=np.float64).T
        for mesh_data in meshes:
            failed = bool(mesh_data.get("failed_faces"))
            key = (failed, len(mesh_data["faces"]) == 0)
            batch = batches.setdefault(key, _MeshBatch(*key))
            batch.add(
                mesh_data, inst["name"] + mesh_data.get("name", ""), user_matrix)
            if batch.full():
                flush(key)

    for key in list(batches):
        flush(key)
    return entries if entries else None


def _xform_points(pts, user_matrix):
    """Transform local points to world-space float32 coordinates."""
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) == 0:
        return pts.astype(np.float32)
    m = np.asarray(user_matrix, dtype=np.float64)
    return (pts @ m[:3, :3].T + m[:3, 3]).astype(np.float32)


def _xform_dirs(dirs, user_matrix):
    """Transform and normalize directions, including non-uniform scale."""
    dirs = np.asarray(dirs, dtype=np.float64)
    if len(dirs) == 0:
        return dirs.astype(np.float32)
    r = np.asarray(user_matrix, dtype=np.float64)[:3, :3]
    try:
        m = np.linalg.inv(r)
    except np.linalg.LinAlgError:
        m = r.T
    d = dirs @ m
    n = np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-30)
    return (d / n).astype(np.float32)


def _source_mesh_count(entries):
    return sum(len(entry.pick_ranges) if entry.pick_ranges else 1
               for entry in entries)


def _field_rgb(poly, key):
    """Return (r, g, b) from poly.field_data[key], or None if absent."""
    if key not in poly.field_data:
        return None
    v = poly.field_data[key]
    return (float(v[0]), float(v[1]), float(v[2]))


def _resolve_material_color(prim):
    """Best-effort constant diffuse/base color of the prim's bound material.

    Returns an (r, g, b) tuple in [0, 1], or None when no constant color can
    be resolved (no bound material, unresolved MDL asset, connected/textured
    inputs, etc.). Always fails soft -- never raises.
    """
    try:
        from pxr import UsdShade
    except ImportError:
        return None
    try:
        material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not material:
            return None
        # ComputeSurfaceSource returns a tuple (shader, name, context) on newer
        # USD and a bare shader on older releases; accept either.
        src = material.ComputeSurfaceSource()
        shader = src[0] if isinstance(src, tuple) else src
        if not shader or not shader.GetPrim().IsValid():
            return None
        for name in ("diffuseColor", "baseColor",
                     "diffuse_color_constant", "base_color_constant"):
            inp = shader.GetInput(name)
            if inp and not inp.HasConnectedSource():
                val = inp.Get()
                if val is not None:
                    return (float(val[0]), float(val[1]), float(val[2]))
    except Exception as exc:
        # Fail soft: a missing/exotic material must never abort the viewer. Surface
        # the cause only when explicitly debugging color resolution.
        if os.environ.get("VIEW_USD_DEBUG"):
            print(f"view_usd: material color resolution failed for "
                  f"{prim.GetPath()}: {exc}", file=sys.stderr)
    return None


def extract_usd_meshes(stage):
    """Extract UsdGeom.Mesh prims as PyVista PolyData."""
    meshes = []
    for prim in stage.Traverse(_USD_TRAVERSE_INSTANCES):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        fcounts = mesh.GetFaceVertexCountsAttr().Get()
        if not pts or not idx or not fcounts:
            continue

        pts_np = np.array(pts, dtype=np.float32)

        xf = np.array(
            UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
            dtype=np.float64,
        )
        if not np.allclose(xf, np.eye(4)):
            ones = np.ones((len(pts_np), 1), dtype=np.float32)
            # USD Gf matrices are row-vector: world = point * M (translation lives
            # in M's last row). Using xf.T here drops the translation, collapsing
            # placed/instanced meshes to the origin.
            pts_np = (np.hstack([pts_np, ones]) @ xf)[:, :3].astype(np.float32)

        faces = []
        vi = 0
        for c in fcounts:
            n = int(c)
            faces.append(n)
            for k in range(n):
                faces.append(int(idx[vi + k]))
            vi += n

        poly = pv.PolyData(pts_np, faces=np.array(faces, dtype=np.int32))

        normals = mesh.GetNormalsAttr().Get()
        if normals:
            n_arr = np.array(normals, dtype=np.float32)
            if len(n_arr) == len(pts_np):
                if not np.allclose(xf, np.eye(4)):
                    nxf = np.linalg.inv(xf).T
                    n4 = np.hstack(
                        [n_arr.astype(np.float64), np.zeros((len(n_arr), 1), dtype=np.float64)])
                    n_arr = (n4 @ nxf)[:, :3].astype(np.float32)
                    ln = np.linalg.norm(n_arr, axis=1, keepdims=True)
                    ln = np.maximum(ln, 1e-30)
                    n_arr = (n_arr / ln).astype(np.float32)
                poly.point_data["Normals"] = n_arr
                poly.GetPointData().SetActiveNormals("Normals")

        # Honor primvars:displayColor (constant interpolation only -- one color
        # per mesh). Stored as field_data so the renderer can pick it up.
        try:
            disp = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
            if disp and disp.HasValue():
                cols = disp.Get()
                if cols and len(cols) > 0:
                    c = cols[0]
                    poly.field_data["DisplayColor"] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32)
        except Exception:
            pass

        # Bound material color (used by the "material" color mode).
        mat_color = _resolve_material_color(prim)
        if mat_color is not None:
            poly.field_data["MaterialColor"] = np.array(mat_color, dtype=np.float32)

        meshes.append(MeshEntry(prim.GetPath().pathString, poly, prim.GetTypeName(), None))
    return meshes


_DISPLAY_MODES = ["shaded", "shaded+wireframe", "wireframe"]


def _suppress_default_vtk_keys(plotter, key_handlers=None):
    """Replace VTK's CharEvent handler with one that owns custom hotkeys."""
    iren = plotter.iren.interactor
    style = iren.GetInteractorStyle()
    iren.RemoveObservers("CharEvent")
    key_handlers = key_handlers or {}

    def _filtered_char(obj, event):
        key = iren.GetKeySym()
        if key:
            handler = key_handlers.get(key.lower())
            if handler is not None:
                handler()
                return
        style.OnChar()

    iren.AddObserver("CharEvent", _filtered_char)


def display(meshes, show_edges, input_filename="",
            retessellate_fn=None,
            toggle_state=None,
            screenshot_path=None,
            view="iso",
            up_axis="z",
            load_start=None):
    """Display meshes with pyvista.

    If ``screenshot_path`` is set, render off-screen, save PNG, and return
    (no interactive window). Otherwise open the interactive viewer.

    Hotkeys:
        w   - cycle display modes (shaded / shaded+wireframe / wireframe)
        m   - toggle color mode (front/back orientation vs. uniform material)
        a   - toggle advancing front tessellation (re-tessellates)
        e   - toggle edge vertex dots
        n   - cycle normals display (off / vertex / face)
        c   - cycle chord tolerance (off / coarse / medium / fine)
        l   - cycle max edge length (off / coarse / medium / fine / finer)
        r   - cycle max aspect ratio (off / 10 / 5 / 2)
        h   - toggle BRep healer (re-tessellates)
        +/= - increase resolution (halve both angles, re-tessellates)
        -   - decrease resolution (double both angles, re-tessellates)
        [   - refine curve angle only (halve, re-tessellates)
        ]   - coarsen curve angle only (double, re-tessellates)
        z   - fly to click point
        f   - frame scene (fit all objects in view)
    """
    plotter = pv.Plotter(
        off_screen=bool(screenshot_path), window_size=(1920, 1080))
    plotter.set_background("#2E3440", top="#4C566A")

    FRONT_COLOR = "#4878D0"
    BACK_COLOR = "#CC4444"
    HIGHLIGHT_COLOR = "#FFAA00"
    HIGHLIGHT_BACK_COLOR = "#B37700"
    FAILED_COLOR = "#FF0000"  # meshes with SmTess "some faces failed" warnings
    # Fallback for "material" mode when a mesh carries no material/displayColor.
    MATERIAL_DEFAULT_COLOR = "#C8C8C8"

    state = {
        "mode_idx": _DISPLAY_MODES.index("shaded+wireframe" if show_edges else "shaded"),
        "actors": [],
        "actor_styles": [],
        "source_meshes": meshes,
        "meshes": meshes,
        "display_mesh_count": _source_mesh_count(meshes),
        "display_vertex_count": sum(poly.n_points for _, poly, *_ in meshes),
        "display_face_count": sum(poly.n_cells for _, poly, *_ in meshes),
        "hud_actor_left": None,
        "hud_actor_left_green": None,
        "hud_actor_right": None,
        "normals_actor": None,
        "edge_vertex_actor": None,
        "pick_label_actor": None,
        "selected_key": None,
        "selection_actor": None,
        "error_actor": None,
    }

    ts = toggle_state or {}
    _defaults = {
        "mode_idx": state["mode_idx"],
        "color_mode": ts.get("color_mode", "front-back"),
        "normals": ts.get("normals", "off"),
        "edge_vertices": ts.get("edge_vertices", False),
        "surface_angle": ts.get("surface_angle", 25.0),
        "curve_angle": ts.get("curve_angle", 25.0),
        "chord_tolerance": ts.get("chord_tolerance", 0.0),
        "max_edge_length": ts.get("max_edge_length", 0.0),
        "max_aspect_ratio": ts.get("max_aspect_ratio", 0.0),
        "advancing_front": ts.get("advancing_front", False),
        "heal": ts.get("heal", False),
    }

    _MODE_LABELS = {"shaded": "Shaded", "shaded+wireframe": "Shaded+Wire", "wireframe": "Wireframe"}
    _COLOR_MODE_LABELS = {"front-back": "Front/Back", "material": "Material"}

    def _capture_camera_state():
        cam = plotter.camera
        return {
            "position": tuple(cam.GetPosition()),
            "focal_point": tuple(cam.GetFocalPoint()),
            "view_up": tuple(cam.GetViewUp()),
            "parallel_projection": bool(cam.GetParallelProjection()),
            "parallel_scale": cam.GetParallelScale(),
            "view_angle": cam.GetViewAngle(),
            "clipping_range": tuple(cam.GetClippingRange()),
        }

    def _restore_camera_state(camera_state):
        cam = plotter.camera
        cam.SetPosition(*camera_state["position"])
        cam.SetFocalPoint(*camera_state["focal_point"])
        cam.SetViewUp(*camera_state["view_up"])
        cam.SetParallelProjection(camera_state["parallel_projection"])
        cam.SetParallelScale(camera_state["parallel_scale"])
        cam.SetViewAngle(camera_state["view_angle"])
        cam.SetClippingRange(*camera_state["clipping_range"])

    def _update_hud():
        for key in ("hud_actor_left", "hud_actor_left_green", "hud_actor_right"):
            if state[key] is not None:
                plotter.remove_actor(state[key])
                state[key] = None

        mode_label = _MODE_LABELS[_DISPLAY_MODES[state["mode_idx"]]]
        ts = toggle_state or {}
        color_mode = ts.get("color_mode", "front-back")
        color_mode_label = _COLOR_MODE_LABELS.get(color_mode, color_mode)
        normals_mode = ts.get("normals", "off")
        edge_vertices = ts.get("edge_vertices", False)
        advancing_front = ts.get("advancing_front", False)
        heal = ts.get("heal", False)
        surf_angle = ts.get("surface_angle", 25.0)
        curve_angle = ts.get("curve_angle", 25.0)
        chord_tol = ts.get("chord_tolerance", 0.0)
        max_edge = ts.get("max_edge_length", 0.0)
        max_aspect = ts.get("max_aspect_ratio", 0.0)
        surface_count = _source_mesh_count(state["source_meshes"])
        source_vertex_count = sum(poly.n_points for _, poly, *_ in state["source_meshes"])
        source_face_count = sum(poly.n_cells for _, poly, *_ in state["source_meshes"])
        def _fmt(v):
            if v <= 0: return "off"
            if v >= 1.0: return f"{v:.1f}"
            return f"{v:.4g}"

        on, off = "on", "off"
        pad = "  "
        L = 15
        V = 16
        left_entries = [
            ("", False),
            (f"{pad}[SMLib SmTess - crack-free]", False),
            ("", False),
            (f"{pad}{'Display:':<{L}}{mode_label:<{V}} [W]",
             state["mode_idx"] != _defaults["mode_idx"]),
            (f"{pad}{'Color:':<{L}}{color_mode_label:<{V}} [M]",
             color_mode != _defaults["color_mode"]),
            (f"{pad}{'Edge verts:':<{L}}{on if edge_vertices else off:<{V}} [E]",
             edge_vertices != _defaults["edge_vertices"]),
            (f"{pad}{'Normals:':<{L}}{normals_mode:<{V}} [N]",
             normals_mode != _defaults["normals"]),
            ("", False),
            (f"{pad}{'Surf angle:':<{L}}{surf_angle:<5.1f}\u00b0{'':<10s} [+/-]",
             surf_angle != _defaults["surface_angle"]),
            (f"{pad}{'Curve angle:':<{L}}{curve_angle:<5.1f}\u00b0{'':<10s} [\\[/\\]]",
             curve_angle != _defaults["curve_angle"]),
            (f"{pad}{'Chord tol:':<{L}}{_fmt(chord_tol):<{V}} [C]",
             chord_tol != _defaults["chord_tolerance"]),
            (f"{pad}{'Max edge:':<{L}}{_fmt(max_edge):<{V}} [L]",
             max_edge != _defaults["max_edge_length"]),
            (f"{pad}{'Aspect max:':<{L}}{_fmt(max_aspect):<{V}} [R]",
             max_aspect != _defaults["max_aspect_ratio"]),
            (f"{pad}{'Adv. front:':<{L}}{on if advancing_front else off:<{V}} [A]",
             advancing_front != _defaults["advancing_front"]),
            (f"{pad}{'Healer:':<{L}}{on if heal else off:<{V}} [H]",
             heal != _defaults["heal"]),
            ("", False),
            (f"{pad}{'Fly to click pt':<{L + V}} [Z]", False),
            (f"{pad}{'Frame scene':<{L + V}} [F]", False),
        ]

        default_lines = []
        green_lines = []
        has_green = False
        for text, changed in left_entries:
            if text == "":
                default_lines.append("")
                green_lines.append("")
            elif changed:
                has_green = True
                default_lines.append(" ")
                green_lines.append(text)
            else:
                default_lines.append(text)
                green_lines.append(" ")

        stat_label_width = 12
        stat_value_width = 8
        # --wireframe-only meshes have no polygon cells, just line cells (edge
        # curve samples) -- label the count accordingly instead of "Faces".
        face_stat_label = "Segments:" if (toggle_state or {}).get("wireframe_only") else "Faces:"
        right_lines = [
            "",
            f"{pad}{'Meshes:':<{stat_label_width}} {surface_count:>{stat_value_width}d}",
            f"{pad}{'Vertices:':<{stat_label_width}} {source_vertex_count:>{stat_value_width}d}",
            f"{pad}{face_stat_label:<{stat_label_width}} {source_face_count:>{stat_value_width}d}",
        ]

        state["hud_actor_left"] = plotter.add_text(
            "\n".join(default_lines), position="upper_left", font_size=9,
            color="#D8DEE9", font="courier", name="hud_overlay_left")
        if has_green:
            state["hud_actor_left_green"] = plotter.add_text(
                "\n".join(green_lines), position="upper_left", font_size=9,
                color="#50FA7B", font="courier", name="hud_overlay_left_green")
        state["hud_actor_right"] = plotter.add_text(
            "\n".join(right_lines), position="upper_right", font_size=9,
            color="#D8DEE9", font="courier", name="hud_overlay_right")

    def _extract_edge_vertex_points(mesh_list):
        # Cache edge vertices for shared PolyData.
        local_cache = {}
        all_points = []
        for entry in mesh_list:
            poly, xform = entry.poly, entry.xform
            if poly.n_points == 0 or "EdgeVertexIndices" not in poly.field_data:
                continue
            pts = local_cache.get(id(poly))
            if pts is None:
                ids = np.asarray(poly.field_data["EdgeVertexIndices"], dtype=np.int64)
                if len(ids) == 0:
                    continue
                pts = np.asarray(poly.points, dtype=np.float32)[ids]
                local_cache[id(poly)] = pts
            all_points.append(_xform_points(pts, xform) if xform is not None else pts)
        if not all_points:
            return None
        return pv.PolyData(np.vstack(all_points))

    def _update_edge_vertex_overlay(display_meshes):
        if state["edge_vertex_actor"] is not None:
            plotter.remove_actor(state["edge_vertex_actor"])
            state["edge_vertex_actor"] = None
        if not (toggle_state or {}).get("edge_vertices", False):
            return
        edge_points = _extract_edge_vertex_points(display_meshes)
        if edge_points is None or edge_points.n_points == 0:
            return
        state["edge_vertex_actor"] = plotter.add_mesh(
            edge_points, color="#EBCB00", point_size=8,
            render_points_as_spheres=True, reset_camera=False, style="points")

    def _build_normal_tubes(origins, directions, color):
        """Build tube geometry for normal visualization."""
        all_origins = np.vstack(origins) if len(origins) > 1 else origins[0]
        all_dirs = np.vstack(directions) if len(directions) > 1 else directions[0]

        bbox = all_origins.max(axis=0) - all_origins.min(axis=0)
        length = float(np.linalg.norm(bbox)) * 0.04
        radius = length * 0.025

        ends = all_origins + all_dirs * length
        n = len(all_origins)
        lines_pts = np.vstack([all_origins, ends]).astype(np.float32)
        lines = np.empty((n, 3), dtype=np.int32)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(n)
        lines[:, 2] = np.arange(n) + n
        line_mesh = pv.PolyData(lines_pts, lines=lines.ravel())
        tubes = line_mesh.tube(radius=radius, n_sides=8)
        return plotter.add_mesh(tubes, color=color, reset_camera=False, opacity=0.9)

    def _update_normals_overlay(display_meshes):
        # Overlays cannot use the actor user_matrix the mesh path uses: a normal
        # transforms by the inverse-transpose, not as geometry, so an actor
        # matrix points glyphs the wrong way on mirrored or non-uniformly scaled
        # instances (~37 degrees off for diag(1,-1,2)). Hence world-space on CPU.
        if state["normals_actor"] is not None:
            plotter.remove_actor(state["normals_actor"])
            state["normals_actor"] = None

        normals_mode = (toggle_state or {}).get("normals", "off")
        if normals_mode == "off":
            return

        if normals_mode == "vertex":
            origins, dirs = [], []
            for entry in display_meshes:
                poly, xform = entry.poly, entry.xform
                if poly.n_points == 0 or "Normals" not in poly.point_data:
                    continue
                o = np.array(poly.points, dtype=np.float64)
                d = np.array(poly.point_data["Normals"], dtype=np.float64)
                if xform is not None:
                    o = _xform_points(o, xform).astype(np.float64)
                    d = _xform_dirs(d, xform).astype(np.float64)
                origins.append(o)
                dirs.append(d)
            if not origins:
                return
            state["normals_actor"] = _build_normal_tubes(origins, dirs, "#A3BE8C")

        elif normals_mode == "face":
            origins, dirs = [], []
            for entry in display_meshes:
                poly, xform = entry.poly, entry.xform
                if poly.n_cells == 0:
                    continue
                if "FaceCenters" in poly.cell_data and "FaceNormals" in poly.cell_data:
                    o = np.array(poly.cell_data["FaceCenters"], dtype=np.float64)
                    d = np.array(poly.cell_data["FaceNormals"], dtype=np.float64)
                else:
                    o = np.array(poly.cell_centers().points, dtype=np.float64)
                    d = np.array(poly.compute_normals(
                        point_normals=False, cell_normals=True,
                        consistent_normals=False)
                        .cell_data["Normals"], dtype=np.float64)
                if xform is not None:
                    o = _xform_points(o, xform).astype(np.float64)
                    d = _xform_dirs(d, xform).astype(np.float64)
                origins.append(o)
                dirs.append(d)
            if not origins:
                return
            state["normals_actor"] = _build_normal_tubes(origins, dirs, "#EBCB8B")

    def _apply_highlight(idx):
        if idx is None or idx >= len(state["actors"]):
            return
        actor = state["actors"][idx]
        prop = actor.GetProperty()
        prop.SetColor(pv.Color(HIGHLIGHT_COLOR).float_rgb)
        prop.SetAmbient(0.6)
        prop.SetDiffuse(0.5)
        # Recolor the backface too, otherwise the red back-facing surfaces
        # (inward-facing normals) don't visibly change when selected.
        back = actor.GetBackfaceProperty()
        if back is not None:
            back.SetColor(pv.Color(HIGHLIGHT_BACK_COLOR).float_rgb)
            back.SetAmbient(0.6)
            back.SetDiffuse(0.5)

    def _restore_style(idx):
        if idx is None or idx >= len(state["actors"]) or idx >= len(state["actor_styles"]):
            return
        st = state["actor_styles"][idx]
        actor = state["actors"][idx]
        prop = actor.GetProperty()
        prop.SetColor(st["color"])
        prop.SetAmbient(st["ambient"])
        prop.SetDiffuse(st["diffuse"])
        back = actor.GetBackfaceProperty()
        if back is not None:
            back.SetColor(st["back_color"])
            back.SetAmbient(st["back_ambient"])
            back.SetDiffuse(st["back_diffuse"])

    def _clear_selection():
        key = state["selected_key"]
        if key is not None and key[1] is None:
            _restore_style(key[0])
        state["selected_key"] = None
        if state["selection_actor"] is not None:
            plotter.remove_actor(state["selection_actor"])
            state["selection_actor"] = None
        if state["pick_label_actor"] is not None:
            plotter.remove_actor(state["pick_label_actor"])
            state["pick_label_actor"] = None

    def _highlight_cell_range(entry, first_cell, end_cell):
        """Overlay one source mesh selected from a batched render entry."""
        selected = entry.poly.extract_cells(
            np.arange(first_cell, end_cell, dtype=np.int64))
        mode = _DISPLAY_MODES[state["mode_idx"]]
        actor = plotter.add_mesh(
            selected,
            color=HIGHLIGHT_COLOR,
            style="wireframe" if mode == "wireframe" else "surface",
            show_edges=(mode == "shaded+wireframe"),
            edge_color=HIGHLIGHT_BACK_COLOR,
            line_width=1.5,
            reset_camera=False,
            render=False,
        )
        actor.SetPickable(False)
        back = vtk.vtkProperty()
        back.SetColor(pv.Color(HIGHLIGHT_BACK_COLOR).float_rgb)
        back.SetAmbient(0.6)
        back.SetDiffuse(0.5)
        actor.SetBackfaceProperty(back)
        state["selection_actor"] = actor

    def _face_colors(poly, color_mode):
        """Return (front_color, back_color) for a mesh under the given mode."""
        if "FailedFaces" in poly.field_data:
            return FAILED_COLOR, FAILED_COLOR
        if color_mode == "material":
            front = (_field_rgb(poly, "MaterialColor")
                     or _field_rgb(poly, "DisplayColor")
                     or MATERIAL_DEFAULT_COLOR)
            return front, front
        front = _field_rgb(poly, "DisplayColor") or FRONT_COLOR
        return front, BACK_COLOR

    def _apply_face_colors():
        """Recolor existing actors in place (no geometry rebuild)."""
        color_mode = (toggle_state or {}).get("color_mode", "front-back")
        for i, ((_, poly, *_), actor) in enumerate(zip(state["meshes"], state["actors"])):
            front_color, back_color = _face_colors(poly, color_mode)
            rgb_front = pv.Color(front_color).float_rgb
            rgb_back = pv.Color(back_color).float_rgb
            if i < len(state["actor_styles"]):
                state["actor_styles"][i]["color"] = rgb_front
                state["actor_styles"][i]["back_color"] = rgb_back
            front_prop = actor.GetProperty()
            front_prop.SetColor(rgb_front)
            back_prop = actor.GetBackfaceProperty()
            if back_prop is not None:
                back_prop.SetColor(rgb_back)
        key = state["selected_key"]
        if key is not None and key[1] is None:
            _apply_highlight(key[0])

    def _add_meshes(mesh_list):
        _clear_selection()
        state["source_meshes"] = mesh_list
        for actor in state["actors"]:
            plotter.remove_actor(actor)
        state["actors"].clear()
        state["actor_styles"] = []
        if state["normals_actor"] is not None:
            plotter.remove_actor(state["normals_actor"])
            state["normals_actor"] = None
        if state["edge_vertex_actor"] is not None:
            plotter.remove_actor(state["edge_vertex_actor"])
            state["edge_vertex_actor"] = None

        mode = _DISPLAY_MODES[state["mode_idx"]]
        color_mode = (toggle_state or {}).get("color_mode", "front-back")
        display_meshes = mesh_list

        # Native USD meshes may still share PolyData through actor transforms.
        mapper_cache = {}
        color_cache = {}
        for entry in display_meshes:
            poly, xform = entry.poly, entry.xform
            key = id(poly)
            colors = color_cache.get(key)
            if colors is None:
                colors = _face_colors(poly, color_mode)
                color_cache[key] = colors
            front_color, back_color = colors

            mapper = mapper_cache.get(key)
            if mapper is None:
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(poly)
                # Use actor colors instead of normal arrays as scalars.
                mapper.ScalarVisibilityOff()
                mapper_cache[key] = mapper

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            if xform is not None:
                actor.SetUserMatrix(pv.vtkmatrix_from_array(xform))

            front_prop = actor.GetProperty()
            front_prop.SetColor(pv.Color(front_color).float_rgb)
            front_prop.SetAmbient(0.0)
            front_prop.SetDiffuse(1.0)
            front_prop.SetSpecular(0.0)
            front_prop.SetSpecularPower(100.0)
            front_prop.SetInterpolationToPhong()
            front_prop.SetEdgeColor(*pv.Color("#00000040").float_rgb)
            front_prop.SetLineWidth(0.5)
            if mode == "shaded+wireframe":
                front_prop.EdgeVisibilityOn()
            elif mode == "wireframe":
                front_prop.SetRepresentationToWireframe()
                front_prop.EdgeVisibilityOff()

            back_prop = vtk.vtkProperty()
            back_prop.SetColor(pv.Color(back_color).float_rgb)
            back_prop.SetAmbient(0.3)
            back_prop.SetDiffuse(0.7)
            back_prop.SetInterpolationToPhong()
            actor.SetBackfaceProperty(back_prop)

            plotter.renderer.AddActor(actor)
            state["actors"].append(actor)
            state["actor_styles"].append({
                "color": front_prop.GetColor(),
                "ambient": front_prop.GetAmbient(),
                "diffuse": front_prop.GetDiffuse(),
                "back_color": back_prop.GetColor(),
                "back_ambient": back_prop.GetAmbient(),
                "back_diffuse": back_prop.GetDiffuse(),
            })

        state["meshes"] = display_meshes
        state["display_mesh_count"] = _source_mesh_count(display_meshes)
        state["display_vertex_count"] = sum(poly.n_points for _, poly, *_ in display_meshes)
        state["display_face_count"] = sum(poly.n_cells for _, poly, *_ in display_meshes)
        _update_edge_vertex_overlay(display_meshes)
        _update_normals_overlay(display_meshes)

    _add_meshes(meshes)
    _update_hud()

    def cycle_display_mode():
        _clear_selection()
        state["mode_idx"] = (state["mode_idx"] + 1) % len(_DISPLAY_MODES)
        mode = _DISPLAY_MODES[state["mode_idx"]]
        for actor in state["actors"]:
            prop = actor.GetProperty()
            if mode == "wireframe":
                prop.SetRepresentationToWireframe()
                prop.EdgeVisibilityOff()
            elif mode == "shaded+wireframe":
                prop.SetRepresentationToSurface()
                prop.EdgeVisibilityOn()
            else:
                prop.SetRepresentationToSurface()
                prop.EdgeVisibilityOff()
        _update_hud()
        plotter.render()

    def toggle_color_mode():
        ts = toggle_state or {}
        cycle = {"front-back": "material", "material": "front-back"}
        ts["color_mode"] = cycle.get(ts.get("color_mode", "front-back"), "front-back")
        print(f"Color mode: {_COLOR_MODE_LABELS.get(ts['color_mode'], ts['color_mode'])}")
        # Recolor actors in place -- no geometry rebuild (fast on large scenes).
        _apply_face_colors()
        _update_hud()
        plotter.render()

    def toggle_edge_vertices():
        ts = toggle_state or {}
        ts["edge_vertices"] = not ts.get("edge_vertices", False)
        camera_state = _capture_camera_state()
        _add_meshes(state["source_meshes"])
        _restore_camera_state(camera_state)
        _update_hud()
        plotter.render()

    def toggle_normals():
        ts = toggle_state or {}
        cycle = {"off": "vertex", "vertex": "face", "face": "off"}
        ts["normals"] = cycle.get(ts.get("normals", "off"), "off")
        camera_state = _capture_camera_state()
        _add_meshes(state["source_meshes"])
        _restore_camera_state(camera_state)
        _update_hud()
        plotter.render()

    _CHORD_TOL_FRACS = [0, 0.01, 0.001, 0.0001]
    _MAX_EDGE_FRACS = [0, 0.5, 0.2, 0.1, 0.05]
    _MAX_ASPECT_PRESETS = [0, 10.0, 5.0, 2.0]

    def _show_error(msg):
        if state["error_actor"] is not None:
            plotter.remove_actor(state["error_actor"])
            state["error_actor"] = None
        if msg:
            state["error_actor"] = plotter.add_text(
                f"  ERROR: {msg}", position="lower_right", font_size=10,
                color="#FF5555", font="courier", name="error_overlay")

    def _retess_and_refresh():
        if retessellate_fn is None:
            _update_hud()
            plotter.render()
            return
        new_meshes, error = retessellate_fn()
        _show_error(error)
        if new_meshes:
            _add_meshes(new_meshes)
        _update_hud()
        plotter.render()

    def toggle_advancing_front():
        ts = toggle_state or {}
        ts["advancing_front"] = not ts.get("advancing_front", False)
        af = ts["advancing_front"]
        print(f"Advancing front: {'ON' if af else 'OFF'}  \u2014  re-tessellating...")
        _retess_and_refresh()

    def toggle_heal():
        ts = toggle_state or {}
        ts["heal"] = not ts.get("heal", False)
        h = ts["heal"]
        print(f"Healer: {'ON' if h else 'OFF'}  \u2014  re-tessellating...")
        _retess_and_refresh()

    def cycle_chord_tolerance():
        ts = toggle_state or {}
        bbox = ts.get("_bbox_diag", 1.0)
        idx = (ts.get("_chord_idx", 0) + 1) % len(_CHORD_TOL_FRACS)
        ts["_chord_idx"] = idx
        val = _CHORD_TOL_FRACS[idx] * bbox
        ts["chord_tolerance"] = val
        label = "off" if val <= 0 else f"{val:.4g}"
        print(f"Chord tolerance: {label}  \u2014  re-tessellating...")
        _retess_and_refresh()

    def cycle_max_edge_length():
        ts = toggle_state or {}
        bbox = ts.get("_bbox_diag", 1.0)
        idx = (ts.get("_mel_idx", 0) + 1) % len(_MAX_EDGE_FRACS)
        ts["_mel_idx"] = idx
        val = _MAX_EDGE_FRACS[idx] * bbox
        ts["max_edge_length"] = val
        label = "off" if val <= 0 else f"{val:.4g}"
        print(f"Max edge length: {label}  \u2014  re-tessellating...")
        _retess_and_refresh()

    def cycle_max_aspect_ratio():
        ts = toggle_state or {}
        idx = (ts.get("_mar_idx", 0) + 1) % len(_MAX_ASPECT_PRESETS)
        ts["_mar_idx"] = idx
        val = _MAX_ASPECT_PRESETS[idx]
        ts["max_aspect_ratio"] = val
        label = "off" if val <= 0 else f"{val:.1f}"
        print(f"Max aspect ratio: {label}  \u2014  re-tessellating...")
        _retess_and_refresh()

    def increase_resolution():
        ts = toggle_state or {}
        sa = max(1.0, ts.get("surface_angle", 25.0) / 2.0)
        ca = max(1.0, ts.get("curve_angle", 25.0) / 2.0)
        ts["surface_angle"] = sa
        ts["curve_angle"] = ca
        print(f"Surface angle: {sa:.1f}\u00b0, Curve angle: {ca:.1f}\u00b0  \u2014  re-tessellating...")
        _retess_and_refresh()

    def decrease_resolution():
        ts = toggle_state or {}
        sa = min(60.0, ts.get("surface_angle", 25.0) * 2.0)
        ca = min(60.0, ts.get("curve_angle", 25.0) * 2.0)
        ts["surface_angle"] = sa
        ts["curve_angle"] = ca
        print(f"Surface angle: {sa:.1f}\u00b0, Curve angle: {ca:.1f}\u00b0  \u2014  re-tessellating...")
        _retess_and_refresh()

    def refine_curve_angle():
        ts = toggle_state or {}
        ca = max(1.0, ts.get("curve_angle", 25.0) / 2.0)
        ts["curve_angle"] = ca
        print(f"Curve angle: {ca:.1f}\u00b0  \u2014  re-tessellating...")
        _retess_and_refresh()

    def coarsen_curve_angle():
        ts = toggle_state or {}
        ca = min(60.0, ts.get("curve_angle", 25.0) * 2.0)
        ts["curve_angle"] = ca
        print(f"Curve angle: {ca:.1f}\u00b0  \u2014  re-tessellating...")
        _retess_and_refresh()

    def fly_to_cursor():
        plotter.fly_to_mouse_position()

    def frame_scene():
        plotter.reset_camera()
        plotter.render()

    def _on_click(_):
        pos = plotter.iren.interactor.GetEventPosition()
        picker = vtk.vtkCellPicker()
        picker.PickFromListOn()
        for actor in state["actors"]:
            picker.AddPickList(actor)
        picker.Pick(pos[0], pos[1], 0, plotter.renderer)
        picked_actor = picker.GetActor()
        if picked_actor is not None and picked_actor in state["actors"]:
            idx = state["actors"].index(picked_actor)
            entry = state["meshes"][idx]
            cell_id = picker.GetCellId()
            picked_range = next(
                ((first, end, name) for first, end, name in (entry.pick_ranges or ())
                 if first <= cell_id < end),
                None)

            key = (idx, picked_range[0] if picked_range else None)
            if key == state["selected_key"]:
                _clear_selection()
            else:
                _clear_selection()
                state["selected_key"] = key
                if picked_range is None:
                    _apply_highlight(idx)
                    name = entry.name
                else:
                    first_cell, end_cell, name = picked_range
                    _highlight_cell_range(entry, first_cell, end_cell)
                prim_type = entry.kind
                state["pick_label_actor"] = plotter.add_text(
                    f"  {name} ({prim_type})", position="lower_left", font_size=10,
                    color=HIGHLIGHT_COLOR, font="courier", name="pick_label")
        else:
            _clear_selection()
        plotter.render()

    plotter.add_axes()

    # Aim the camera along a canonical axis if requested.  view="iso" leaves
    # the default isometric framing untouched.  Otherwise we look at the
    # scene bounding-box center from the +/- side of one axis, with `up_axis`
    # mapped to the world up vector (Z by default; flip to Y for Y-up scenes).
    if view != "iso":
        bounds = plotter.bounds  # (xmin, xmax, ymin, ymax, zmin, zmax)
        cx = 0.5 * (bounds[0] + bounds[1])
        cy = 0.5 * (bounds[2] + bounds[3])
        cz = 0.5 * (bounds[4] + bounds[5])
        diag = max(
            bounds[1] - bounds[0],
            bounds[3] - bounds[2],
            bounds[5] - bounds[4],
            1e-6,
        )
        # Distance: 2.5x the bbox diagonal axis -- gives some margin around
        # the model and keeps it inside the default view angle.
        dist = 2.5 * diag

        up = (0.0, 0.0, 1.0) if up_axis == "z" else (0.0, 1.0, 0.0)

        # "front" is defined to mean: looking along +Y toward the origin,
        # i.e. the camera sits on the -Y side. (Matches conventional
        # CAD/CAM "front view": -Y is the viewer side.)
        # Three-quarter views: 25 deg azimuth off the canonical front, 15 deg
        # elevation up.  Same distance scale as the orthogonal views.
        import math as _math
        _az = _math.radians(25.0)
        _el = _math.radians(15.0)
        _ce = _math.cos(_el)
        _fx = +_math.sin(_az) * _ce
        _fy = -_math.cos(_az) * _ce
        _fz = +_math.sin(_el)
        offsets = {
            "front":           (0.0, -dist, 0.0),
            "back":            (0.0, +dist, 0.0),
            "left":            (-dist, 0.0, 0.0),
            "right":           (+dist, 0.0, 0.0),
            "top":             (0.0, 0.0, +dist),
            "bottom":          (0.0, 0.0, -dist),
            "front-iso":       (_fx * dist,  _fy * dist, _fz * dist),
            "front-iso-left":  (-_fx * dist, _fy * dist, _fz * dist),
        }
        if view in offsets:
            ox, oy, oz = offsets[view]
            cam = plotter.camera
            cam.SetFocalPoint(cx, cy, cz)
            cam.SetPosition(cx + ox, cy + oy, cz + oz)
            cam.SetViewUp(*up)
            plotter.renderer.ResetCameraClippingRange()
        else:
            plotter.reset_camera()
    else:
        plotter.reset_camera()

    if screenshot_path:
        _render_start = time.perf_counter()
        plotter.render()
        plotter.screenshot(screenshot_path)
        print(f"Render + screenshot: {time.perf_counter() - _render_start:.2f}s", flush=True)
        plotter.close()
        if load_start is not None:
            print(f"Total: {time.perf_counter() - load_start:.2f}s", flush=True)
        return

    plotter.track_click_position(_on_click, side="left")

    _suppress_default_vtk_keys(plotter, {
        "w": cycle_display_mode,
        "m": toggle_color_mode,
        "a": toggle_advancing_front,
        "h": toggle_heal,
        "e": toggle_edge_vertices,
        "n": toggle_normals,
        "c": cycle_chord_tolerance,
        "l": cycle_max_edge_length,
        "r": cycle_max_aspect_ratio,
        "plus": increase_resolution,
        "equal": increase_resolution,
        "minus": decrease_resolution,
        "bracketleft": refine_curve_angle,
        "bracketright": coarsen_curve_angle,
        "z": fly_to_cursor,
        "f": frame_scene,
    })

    if load_start is not None:
        print(f"Total (to viewer ready): {time.perf_counter() - load_start:.2f}s", flush=True)

    title = f"{input_filename} - view_usd" if input_filename else "view_usd"
    plotter.show(title=title)


def main():
    parser = argparse.ArgumentParser(
        description="View USD BReps (SmTess) and/or native UsdGeom.Mesh prims")
    parser.add_argument("input", help="Input USD file or directory containing USD files")
    parser.add_argument("--surface-angle", type=float, default=25.0,
                        help="Surface angular deflection in degrees (default: 25)")
    parser.add_argument("--curve-angle", type=float, default=25.0,
                        help="Curve angular deflection in degrees (default: 25)")
    parser.add_argument("--chord-tolerance", type=float, default=0.0,
                        help="Chord height tolerance (0=off)")
    parser.add_argument("--max-edge-length", type=float, default=0.0,
                        help="Max 3D edge length (0=off)")
    parser.add_argument("--max-aspect-ratio", type=float, default=0.0,
                        help="Max triangle aspect ratio (0=off)")
    parser.add_argument("--smooth", action="store_true",
                        help="Enable smoothing")
    parser.add_argument("--advancing-front", action="store_true",
                        help="Use advancing front tessellation")
    parser.add_argument("--heal", action="store_true",
                        help="Enable BRep healer (off by default)")
    parser.add_argument("--max-breps", type=int, default=0, metavar="N",
                        help="Stop after N BrepArray placements, then display what "
                             "was loaded (0 = all)")
    parser.add_argument("--workers", type=int, default=0, metavar="N",
                        help="Max prototypes tessellated concurrently by smtess_usd "
                             "(0 = auto by hardware; 1 = serial)")
    parser.add_argument("--timing", metavar="PATH",
                        help="Write import/heal and tessellation timing JSONL to PATH; "
                             "view with make_timing_chart.py.")
    parser.add_argument("--no-wireframe", action="store_true",
                        help="Disable wireframe overlay")
    parser.add_argument("--wireframe-only", action="store_true",
                        help="BRep display mode: sample edge curves instead of full surface "
                             "tessellation (smtess_usd --wireframe). No SmSurfaceCache "
                             "subdivision, so it stays fast even for breps whose normal "
                             "tessellation is pathologically slow. Distinct from "
                             "--no-wireframe / the 'w' hotkey, which only change how an "
                             "already-tessellated mesh is drawn.")
    parser.add_argument(
        "--color-mode",
        choices=["front-back", "material"],
        default="front-back",
        help="Initial face coloring: 'front-back' = primvars:displayColor (or blue) "
             "on front faces, red on back for orientation (default); 'material' = "
             "bound material or displayColor on both faces. Toggle at runtime with [M].",
    )
    parser.add_argument("--no-usd-meshes", action="store_true",
                        help="Skip UsdGeom.Mesh prims (BRep tessellation only)")
    parser.add_argument("--no-breps", action="store_true",
                        help="Skip BRepArray tessellation (USD meshes only)")
    parser.add_argument(
        "--screenshot",
        metavar="PATH",
        help="Render off-screen and save PNG (single USD file only; no interactive window)",
    )
    parser.add_argument(
        "--view",
        choices=["iso", "front", "back", "left", "right", "top", "bottom",
                 "front-iso", "front-iso-left"],
        default="iso",
        help="Camera viewpoint. 'front' = camera on -Y side looking toward +Y "
             "(standard CAD front view). 'front-iso' = three-quarter view "
             "from front-right, slightly elevated. 'front-iso-left' = same "
             "but from the left. 'iso' = default isometric framing.",
    )
    parser.add_argument(
        "--up-axis",
        choices=["z", "y"],
        default="z",
        help="World up axis used by --view (default: z)",
    )
    args = parser.parse_args()
    if args.max_breps < 0:
        args.max_breps = 0

    if not os.path.exists(args.input):
        print(f"Error: {args.input} not found", file=sys.stderr)
        sys.exit(1)
    if args.screenshot and os.path.isdir(args.input):
        print(
            "Error: --screenshot requires a single USD file, not a directory.",
            file=sys.stderr,
        )
        sys.exit(1)

    def view_input(input_path):
        load_start = time.perf_counter()
        print(f"Loading input file: {input_path}", flush=True)

        stage_start = time.perf_counter()
        stage = Usd.Stage.Open(input_path)
        if not stage:
            print(f"Error: could not open USD stage: {input_path}", file=sys.stderr)
            return

        # TraverseInstanceProxies expands instances, so BrepArrays are counted once
        # per placement. Unique prototypes are deduped in the same pass, keyed as
        # CollectBrepPlacements() keys them, so both ends report the same split.
        n_meshes = 0
        n_brep_placements = 0
        brep_protos = set()
        for prim in stage.Traverse(_USD_TRAVERSE_INSTANCES):
            if prim.IsA(UsdGeom.Mesh):
                n_meshes += 1
            if prim.GetTypeName() == "BrepArray":
                n_brep_placements += 1
                proto = prim.GetPrimInPrototype() if prim.IsInstanceProxy() else prim
                brep_protos.add(proto.GetPath())
        n_brep_unique = len(brep_protos)
        stage_elapsed = time.perf_counter() - stage_start

        if n_brep_placements != n_brep_unique:
            brep_desc = (f"{n_brep_unique} unique BrepArray prim(s) "
                         f"({n_brep_placements} instance placements)")
        else:
            brep_desc = f"{n_brep_unique} BrepArray prim(s)"
        print(
            f"Stage: {n_meshes} UsdGeom.Mesh prim(s), {brep_desc} "
            f"(open+traverse {stage_elapsed:.2f}s)",
            flush=True,
        )

        toggle_state = {
            "color_mode": args.color_mode,
            "normals": "off",
            "edge_vertices": False,
            "surface_angle": args.surface_angle,
            "curve_angle": args.curve_angle,
            "advancing_front": args.advancing_front,
            "heal": args.heal,
            "chord_tolerance": args.chord_tolerance,
            "max_edge_length": args.max_edge_length,
            "max_aspect_ratio": args.max_aspect_ratio,
            "wireframe_only": args.wireframe_only,
        }

        usd_mesh_parts = []
        if not args.no_usd_meshes and n_meshes > 0:
            usd_mesh_parts = extract_usd_meshes(stage)
            skipped = n_meshes - len(usd_mesh_parts)
            if skipped:
                print(
                    f"  Loaded {len(usd_mesh_parts)} UsdGeom.Mesh object(s) "
                    f"({skipped} prim(s) skipped: empty geometry)",
                    flush=True,
                )
            else:
                print(
                    f"  Loaded {len(usd_mesh_parts)} UsdGeom.Mesh object(s)",
                    flush=True,
                )

        tess_start = time.perf_counter()
        mesh_dicts = None
        error = None
        if not args.no_breps and n_brep_placements > 0:
            mesh_dicts, error = tessellate_with_smtess(
                input_path,
                surface_angle=toggle_state["surface_angle"],
                curve_angle=toggle_state["curve_angle"],
                chord_tolerance=toggle_state["chord_tolerance"],
                max_edge_length=toggle_state["max_edge_length"],
                max_aspect_ratio=toggle_state["max_aspect_ratio"],
                smooth=args.smooth,
                advancing_front=toggle_state["advancing_front"],
                heal=toggle_state["heal"],
                max_breps=args.max_breps,
                workers=args.workers,
                wireframe_only=toggle_state["wireframe_only"],
                timing_path=args.timing,
            )
        tess_elapsed = time.perf_counter() - tess_start

        build_start = time.perf_counter()
        brep_meshes = build_instanced_meshes(mesh_dicts) if mesh_dicts else None
        build_elapsed = time.perf_counter() - build_start

        meshes = []
        if usd_mesh_parts:
            meshes.extend(usd_mesh_parts)
        if brep_meshes:
            meshes.extend(brep_meshes)

        if not meshes:
            msg = f"Skipping {os.path.basename(input_path)}: no meshes to display"
            if error:
                msg += f" ({error})"
            print(msg, file=sys.stderr)
            return

        totals_start = time.perf_counter()
        total_pts = sum(p.n_points for _, p, *_ in meshes)
        total_cells = sum(p.n_cells for _, p, *_ in meshes)
        totals_elapsed = time.perf_counter() - totals_start
        n_brep_loaded = _source_mesh_count(brep_meshes) if brep_meshes else 0
        n_meshes_loaded = len(usd_mesh_parts) + n_brep_loaded
        print(
            f"Viewer mesh preparation finished: {n_meshes_loaded} mesh(es) "
            f"({len(usd_mesh_parts)} UsdGeom.Mesh, {n_brep_loaded} BRep-tessellated), "
            f"{total_pts} points, {total_cells} triangles"
            + (f" (tess {tess_elapsed:.2f}s, instance-build {build_elapsed:.2f}s, "
               f"totals {totals_elapsed:.2f}s)"
               if not args.no_breps and n_brep_placements > 0 else ""),
            flush=True,
        )

        # Scene extent, from each mesh's 8 bounding-box corners. Instanced corners
        # must go to world space anyway -- using only the translation would collapse
        # a single origin-placed part to zero size.
        corner_pts = []
        for entry in meshes:
            poly, xform = entry.poly, entry.xform
            if poly.n_points == 0:
                continue
            # poly.bounds (a plain GetBounds() call) avoids poly.outline(),
            # which builds and validates an actual wireframe mesh just to
            # get 8 corner points.
            xmin, xmax, ymin, ymax, zmin, zmax = poly.bounds
            corners = np.array(
                [[x, y, z] for x in (xmin, xmax) for y in (ymin, ymax) for z in (zmin, zmax)],
                dtype=np.float64)
            corner_pts.append(_xform_points(corners, xform) if xform is not None
                              else corners)
        if corner_pts:
            allc = np.vstack(corner_pts)
            bbox_diag = float(np.linalg.norm(allc.max(axis=0) - allc.min(axis=0)))
        else:
            bbox_diag = 0.0
        toggle_state["_bbox_diag"] = bbox_diag if bbox_diag > 0 else 1.0

        def _retess():
            tess_start = time.perf_counter()
            new_dicts, err = tessellate_with_smtess(
                input_path,
                surface_angle=toggle_state["surface_angle"],
                curve_angle=toggle_state["curve_angle"],
                chord_tolerance=toggle_state["chord_tolerance"],
                max_edge_length=toggle_state["max_edge_length"],
                max_aspect_ratio=toggle_state["max_aspect_ratio"],
                smooth=args.smooth,
                advancing_front=toggle_state["advancing_front"],
                heal=toggle_state["heal"],
                max_breps=args.max_breps,
                workers=args.workers,
                wireframe_only=toggle_state["wireframe_only"],
                timing_path=args.timing,
            )
            if not new_dicts:
                return None, err or "re-tessellation produced no meshes"
            tess_elapsed = time.perf_counter() - tess_start
            build_start = time.perf_counter()
            new_brep = build_instanced_meshes(new_dicts)
            build_elapsed = time.perf_counter() - build_start
            if not new_brep:
                return None, "no valid meshes after loading"
            combined = list(usd_mesh_parts) + new_brep
            total_pts = sum(p.n_points for _, p, *_ in combined)
            total_cells = sum(p.n_cells for _, p, *_ in combined)
            n_new_brep = _source_mesh_count(new_brep)
            print(
                f"Viewer mesh preparation finished: "
                f"{len(usd_mesh_parts) + n_new_brep} mesh(es) "
                f"({len(usd_mesh_parts)} UsdGeom.Mesh, {n_new_brep} BRep-tessellated), "
                f"{total_pts} points, {total_cells} triangles "
                f"(tess {tess_elapsed:.2f}s, instance-build {build_elapsed:.2f}s)",
                flush=True,
            )
            return combined, None

        retess_fn = _retess if (not args.no_breps and n_brep_placements > 0) else None

        display(
            meshes,
            show_edges=not args.no_wireframe,
            input_filename=os.path.basename(input_path),
            retessellate_fn=retess_fn,
            toggle_state=toggle_state,
            screenshot_path=args.screenshot,
            view=args.view,
            up_axis=args.up_axis,
            load_start=load_start,
        )

    if os.path.isdir(args.input):
        files = []
        for name in sorted(os.listdir(args.input)):
            path = os.path.join(args.input, name)
            if os.path.isfile(path) and name.lower().endswith((".usd", ".usda", ".usdc", ".usdz")):
                files.append(path)
        if not files:
            print(f"Error: no USD files found in {args.input}", file=sys.stderr)
            sys.exit(1)
        print(f"Found {len(files)} USD file(s) in {args.input}", flush=True)
        print("Close the viewer window to advance to the next file.", flush=True)
        for idx, path in enumerate(files, start=1):
            print(f"[{idx}/{len(files)}] {os.path.basename(path)}", flush=True)
            view_input(path)
    else:
        view_input(args.input)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    main()
