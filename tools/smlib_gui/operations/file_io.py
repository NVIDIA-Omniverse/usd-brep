# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""File import/export operations for the SMLib GUI."""

from __future__ import annotations

import glob
import os
import platform
import subprocess
import sys
import tempfile

from ..model import ActiveObject, make_active_objects
from ..runtime import REPO, sm


USD_EXTENSIONS = {".usd", ".usda", ".usdc"}
SMB_EXTENSION = ".smb"
SMP_EXTENSION = ".smp"
OCCT_BREP_EXTENSION = ".brep"
SUPPORTED_LOAD_EXTENSIONS = frozenset(
    USD_EXTENSIONS | {SMB_EXTENSION, SMP_EXTENSION, OCCT_BREP_EXTENSION}
)

LOAD_FILE_FILTER = (
    "All supported (*.usd *.usda *.usdc *.smb *.smp *.brep);;"
    "USD ASCII (*.usda);;"
    "USD crate (*.usdc);;"
    "USD files (*.usd);;"
    "SMLib BRep (*.smb);;"
    "SMLib Part (*.smp);;"
    "OpenCASCADE BRep (*.brep);;"
    "All files (*)"
)
SAVE_FILE_FILTER = (
    "USD ASCII (*.usda);;"
    "USD crate (*.usdc);;"
    "USD files (*.usd);;"
    "SMLib BRep (*.smb);;"
    "OpenCASCADE BRep (*.brep);;"
    "All files (*)"
)


def file_kind_for_path(filename: str) -> str:
    """Return the supported GUI file kind for ``filename``."""
    ext = os.path.splitext(filename)[1].lower()
    if ext in USD_EXTENSIONS:
        return "usd"
    if ext == SMB_EXTENSION:
        return "smb"
    if ext == SMP_EXTENSION:
        return "smp"
    if ext == OCCT_BREP_EXTENSION:
        return "occt_brep"
    return ""


def extension_for_filter(selected_filter: str) -> str:
    """Return the default extension implied by a Qt file-dialog filter."""
    normalized = selected_filter.lower()
    if normalized.count("*.") != 1:
        return ""
    if "*.brep" in normalized:
        return ".brep"
    if "*.smb" in normalized:
        return ".smb"
    if "*.smp" in normalized:
        return ".smp"
    if "*.usdc" in normalized:
        return ".usdc"
    if "*.usda" in normalized:
        return ".usda"
    if "*.usd" in normalized:
        return ".usd"
    return ""


def ensure_extension_for_filter(filename: str, selected_filter: str) -> str:
    """Append a selected-filter extension when the dialog path has none."""
    if os.path.splitext(filename)[1]:
        return filename
    ext = extension_for_filter(selected_filter)
    if not ext:
        return filename
    return filename + ext


def native_brep_load_unavailable_reason() -> str:
    return "Native SMLib BRep (*.smb) load is not exposed by this _omni_solid build."


def native_brep_file_is_ascii(filename: str) -> bool:
    """Return whether a native SMLib BRep or part file uses the ASCII format."""
    with open(filename, "rb") as file:
        header = file.read(16)
    if header.startswith(b"Version "):
        return True
    if header.startswith(b"//Brep Starts"):
        return True
    if header.startswith(b"//[Output Summar"):
        return True
    if len(header) >= 2 and header[0] == ord("V"):
        return False
    raise RuntimeError(f"Could not determine native SMLib BRep file format: {filename}")


def native_file_is_part(filename: str) -> bool:
    """Return whether a native SMLib file uses the multi-object part format."""
    if os.path.splitext(filename)[1].lower() == SMP_EXTENSION:
        return True
    with open(filename, "rb") as file:
        return file.read(16).startswith(b"//[Output Summar")


def omnisolid_plugin_path() -> str:
    """Return a valid OmniSolid USD schema plugin resource path, if present."""
    env_value = os.environ.get("OMNISOLID_PLUGIN_PATH", "")
    env_path = os.path.abspath(os.path.expanduser(env_value)) if env_value else ""
    if env_path and _valid_omnisolid_resource_dir(env_path):
        os.environ["OMNISOLID_PLUGIN_PATH"] = env_path
        return env_path

    candidates = [
        os.path.join(REPO, "_build", "schema", "omniSolid", "resources"),
        os.path.join(REPO, "source", "schema", "omniSolid", "resources"),
    ]
    for candidate in candidates:
        if _valid_omnisolid_resource_dir(candidate):
            os.environ["OMNISOLID_PLUGIN_PATH"] = candidate
            return candidate
    return ""


def _valid_omnisolid_resource_dir(path: str) -> bool:
    if not os.path.isdir(path):
        return False
    schema_file = os.path.join(path, "generatedSchema.usda")
    if not os.path.isfile(schema_file):
        return False
    try:
        with open(schema_file, "rb") as file:
            return file.read(8).startswith(b"#usda")
    except OSError:
        return False


def brep_usd_io_available() -> bool:
    """Return whether BRepArray USD operations can locate schema resources."""
    return bool(omnisolid_plugin_path())


def brep_usd_io_unavailable_reason() -> str:
    return (
        "Valid OmniSolid USD schema resources were not found. "
        "Set OMNISOLID_PLUGIN_PATH to an omniSolid/resources directory."
    )


def _host_build_platform() -> str:
    if sys.platform == "win32":
        return "windows-x86_64"
    if sys.platform == "darwin":
        return "macos-universal"
    machine = platform.machine().lower()
    return "linux-aarch64" if machine in ("aarch64", "arm64") else "linux-x86_64"


def _executable_name(name: str) -> str:
    return f"{name}.exe" if sys.platform == "win32" else name


def _built_converter_path(name: str, env_override: str) -> str:
    override = os.environ.get(env_override)
    if override:
        return os.path.abspath(os.path.expanduser(override))

    exe_name = _executable_name(name)
    candidates = sorted(glob.glob(os.path.join(REPO, "_build", "*", "*", exe_name)))
    host_candidates = sorted(
        glob.glob(os.path.join(REPO, "_build", _host_build_platform(), "*", exe_name))
    )
    host_release = [
        path for path in host_candidates if os.path.basename(os.path.dirname(path)).lower() == "release"
    ]
    release = [path for path in candidates if os.path.basename(os.path.dirname(path)).lower() == "release"]
    chosen = host_release or host_candidates or release or candidates
    if chosen:
        return chosen[0]

    return os.path.join(REPO, "_build", _host_build_platform(), "release", exe_name)


def _require_converter(name: str, env_override: str) -> str:
    path = _built_converter_path(name, env_override)
    if os.path.isfile(path) and os.access(path, os.X_OK):
        return path
    raise RuntimeError(
        f"{name} converter is not built at {path}. "
        f"Run ./repo.sh build or set {env_override} to the converter executable."
    )


def occt_brep_io_available() -> bool:
    """Return whether the OCCT BRep converter executables and USD schema are available."""
    try:
        _require_converter("occt_to_usd", "OCCT_TO_USD")
        _require_converter("usd_to_occt", "USD_TO_OCCT")
    except RuntimeError:
        return False
    return brep_usd_io_available()


def _converter_timeout_sec() -> int:
    value = os.environ.get("OCCT_BREP_GUI_TIMEOUT_SEC", os.environ.get("OCCT_BREP_TEST_TIMEOUT_SEC", "300"))
    try:
        return max(1, int(value))
    except ValueError:
        return 300


def _converter_error_message(result: subprocess.CompletedProcess[str]) -> str:
    detail = (result.stderr or result.stdout or "").strip()
    if not detail:
        return f"converter exited with status {result.returncode}"
    return detail if len(detail) <= 2000 else detail[:1997] + "..."


def _run_converter(args: list[str], what: str) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            env=os.environ.copy(),
            timeout=_converter_timeout_sec(),
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"{what} timed out after {_converter_timeout_sec()} seconds") from exc
    if result.returncode != 0:
        raise RuntimeError(f"{what} failed: {_converter_error_message(result)}")
    return result


def _stem_name(filename: str) -> str:
    stem = os.path.splitext(os.path.basename(filename))[0]
    return stem or "imported"


def import_usd_objects(filename: str, starting_id: int = 1,
                       heal: bool = False) -> list[ActiveObject]:
    """Import all exposed USD BRep and mesh objects from a file."""
    if sm is None:
        raise RuntimeError("_omni_solid module not found")

    imported: list[tuple[str, object]] = []
    base_name = _stem_name(filename)

    if brep_usd_io_available():
        breps = sm.usd.import_breps(filename, heal=heal)
        for i, brep in enumerate(breps, start=1):
            imported.append((f"{base_name}_brep{i}", brep))

    try:
        meshes = sm.usd.import_meshes(filename)
    except RuntimeError:
        meshes = []
    for i, mesh in enumerate(meshes, start=1):
        imported.append((f"{base_name}_mesh{i}", mesh))

    if not imported:
        raise RuntimeError(f"No supported SMLib objects found in {filename}")
    return make_active_objects(imported, starting_id=starting_id)


def import_native_brep_object(filename: str, starting_id: int = 1,
                              ascii: bool | None = None) -> list[ActiveObject]:
    """Import a native SMLib single-BRep or multi-object part file."""
    if sm is None:
        raise RuntimeError("_omni_solid module not found")

    read_ascii = native_brep_file_is_ascii(filename) if ascii is None else ascii
    if native_file_is_part(filename):
        read_part = getattr(sm, "read_part_from_file", None)
        if read_part is None:
            raise RuntimeError(
                "Native SMLib part (*.smp) load is not exposed by this _omni_solid build."
            )
        curves, surfaces, _boolean_tree_nodes, breps = read_part(filename, ascii=read_ascii)
        base_name = _stem_name(filename)
        imported = [
            *((f"{base_name}_curve{i}", curve) for i, curve in enumerate(curves, start=1)),
            *((f"{base_name}_surface{i}", surface) for i, surface in enumerate(surfaces, start=1)),
            *((f"{base_name}_brep{i}", brep) for i, brep in enumerate(breps, start=1)),
        ]
        if not imported:
            raise RuntimeError(f"No displayable SMLib objects found in part file {filename}")
        return make_active_objects(imported, starting_id=starting_id)

    read_brep = getattr(sm, "read_brep_from_file", None)
    if read_brep is None:
        raise RuntimeError(native_brep_load_unavailable_reason())
    brep = read_brep(filename, ascii=read_ascii)
    return make_active_objects([(_stem_name(filename), brep)], starting_id=starting_id)


def import_occt_brep_objects(filename: str, starting_id: int = 1,
                             heal: bool = False) -> list[ActiveObject]:
    """Import an OpenCASCADE ``.brep`` file via OCCT BRep -> USD -> SMLib conversion."""
    if sm is None:
        raise RuntimeError("_omni_solid module not found")
    if not brep_usd_io_available():
        raise RuntimeError(brep_usd_io_unavailable_reason())

    converter = _require_converter("occt_to_usd", "OCCT_TO_USD")
    with tempfile.TemporaryDirectory(prefix="smlib_gui_occt_import_") as tmpdir:
        usd_file = os.path.join(tmpdir, _stem_name(filename) + ".usda")
        _run_converter([converter, filename, usd_file], "OCCT BRep import")
        return import_usd_objects(usd_file, starting_id=starting_id, heal=heal)


def import_file_objects(filename: str, starting_id: int = 1,
                        heal: bool = False) -> list[ActiveObject]:
    """Import any model format supported by the GUI."""
    kind = file_kind_for_path(filename)
    if kind == "usd":
        return import_usd_objects(filename, starting_id=starting_id, heal=heal)
    if kind in ("smb", "smp"):
        return import_native_brep_object(filename, starting_id=starting_id)
    if kind == "occt_brep":
        return import_occt_brep_objects(filename, starting_id=starting_id, heal=heal)
    supported = ", ".join(sorted(SUPPORTED_LOAD_EXTENSIONS))
    raise RuntimeError(f"Unsupported file type for {filename}; expected one of: {supported}")


def export_usd_objects(objects: list[ActiveObject], filename: str) -> int:
    """Export all active BRep and PolyBrep objects to a USD file."""
    if sm is None:
        raise RuntimeError("_omni_solid module not found")

    breps = [obj.handle for obj in objects if obj.kind == "Brep"]
    meshes = [obj.handle for obj in objects if obj.kind == "PolyBrep"]
    if not breps and not meshes:
        raise RuntimeError("No BRep or PolyBrep objects are available for USD export")
    if breps and not brep_usd_io_available():
        raise RuntimeError(brep_usd_io_unavailable_reason())

    if breps:
        sm.usd.export_breps(breps, filename)
        if meshes:
            sm.usd.append_meshes(meshes, filename)
    else:
        sm.usd.export_meshes(meshes, filename)
    return len(breps) + len(meshes)


def export_native_brep(obj: ActiveObject, filename: str, ascii: bool = True) -> None:
    """Export one selected BRep through the native BRep write API."""
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep object before saving native BRep data")
    obj.handle.write_to_file(filename, ascii=ascii)


def export_occt_brep(obj: ActiveObject, filename: str) -> None:
    """Export one selected BRep via SMLib -> USD -> OpenCASCADE ASCII ``.brep`` conversion."""
    if sm is None:
        raise RuntimeError("_omni_solid module not found")
    if obj.kind != "Brep":
        raise RuntimeError("Select a BRep object before saving OpenCASCADE BRep data")
    if not brep_usd_io_available():
        raise RuntimeError(brep_usd_io_unavailable_reason())

    converter = _require_converter("usd_to_occt", "USD_TO_OCCT")
    with tempfile.TemporaryDirectory(prefix="smlib_gui_occt_export_") as tmpdir:
        usd_file = os.path.join(tmpdir, _stem_name(filename) + ".usda")
        sm.usd.export_brep(obj.handle, usd_file)
        _run_converter([converter, "-q", usd_file, filename], "OCCT BRep export")

    if not os.path.isfile(filename) or os.path.getsize(filename) <= 0:
        raise RuntimeError(f"OCCT BRep export did not create a non-empty file: {filename}")
