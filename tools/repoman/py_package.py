# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Build a self-contained `usd-brep` wheel from an existing build tree.

The wheel vendors OpenUSD rather than depending on a PyPI distribution of it.
That is safe here because no `pxr` object crosses the pybind11 boundary -- every
USD entry point takes a file path -- so a caller's own OpenUSD and this one never
exchange objects. It is also necessary: no PyPI package ships OpenUSD 25.11.
"""

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
from typing import Callable, Dict, List

import omni.repo.man
from omni.repo.man.deps import get_uv

from wheel_version import wheel_version


# Supplied by the interpreter and the VC++ redistributable. Vendoring python312
# would give the process a second interpreter; the CRT must match the host.
_EXCLUDED_LIBS = {
    "python312.dll",
    "libpython3.12.so.1.0",
    "msvcp140.dll",
    "vcruntime140.dll",
    "vcruntime140_1.dll",
    "ucrtbase.dll",
}


def _dependency_closure(start: str, search_dirs: List[str]) -> List[str]:
    """Every shared library `start` needs, restricted to what we ship.

    Read from the binaries themselves rather than from the premake link lists,
    which cover direct links only and miss the transitive OpenUSD runtime.
    """
    available: Dict[str, str] = {}
    for directory in search_dirs:
        if not os.path.isdir(directory):
            continue
        for name in os.listdir(directory):
            if _is_shared_lib(name) and name.lower() not in available:
                available[name.lower()] = os.path.join(directory, name)

    found: List[str] = []
    seen = set()
    queue = [start]
    while queue:
        for name in _referenced_libs(queue.pop()):
            key = name.lower()
            if key in seen or key not in available or key in _EXCLUDED_LIBS:
                continue
            seen.add(key)
            found.append(available[key])
            queue.append(available[key])
    return found


def _is_shared_lib(name: str) -> bool:
    return name.lower().endswith(".dll") if omni.repo.man.is_windows() else ".so" in name


def _referenced_libs(path: str) -> List[str]:
    if omni.repo.man.is_windows():
        with open(path, "rb") as handle:
            return [m.decode() for m in re.findall(rb"[A-Za-z0-9_.\-]+\.[Dd][Ll][Ll]", handle.read())]
    out = subprocess.run(["objdump", "-p", path], capture_output=True, text=True, check=True).stdout
    return re.findall(r"NEEDED\s+(\S+)", out)


def _patch_elf_binaries(libs: List[str]):
    """Point every binary at its own directory and drop the libpython link.

    The build tree links with `$ORIGIN/../lib`, which suits the packman layout
    (module in `python/`, libraries in `lib/`); the wheel is flat instead.

    OpenUSD is built with Python support, so the stock libraries and our
    first-party ones carry libpython3.12 in DT_NEEDED. Vendoring that would give
    the process a second interpreter, and leaving it unresolved fails the import
    outright -- the symbols come from the running interpreter, so the dependency
    has to go.
    """
    for lib in libs:
        subprocess.run(["patchelf", "--set-rpath", "$ORIGIN", lib], check=True)
        needed = subprocess.run(
            ["patchelf", "--print-needed", lib], capture_output=True, text=True, check=True
        ).stdout.split()
        for entry in needed:
            if entry.startswith("libpython"):
                subprocess.run(["patchelf", "--remove-needed", entry, lib], check=True)


def setup_repo_tool(parser: argparse.ArgumentParser, config: Dict) -> Callable:
    tool_config = config.get("repo_py_package", {})
    if not tool_config.get("enabled", True):
        return None

    parser.description = "Build a self-contained wheel for the usd-brep Python bindings."
    omni.repo.man.add_config_arg(parser)

    def run_repo_tool(options, config: Dict):
        tool_config = config["repo_py_package"]
        staging_dir = omni.repo.man.resolve_tokens(tool_config["staging_dir"])
        install_dir = omni.repo.man.resolve_tokens(tool_config["install_dir"])

        build_dir = omni.repo.man.resolve_tokens("$root/_build/$platform/$config")
        usd_dir = omni.repo.man.resolve_tokens("$root/_build/target-deps/usd/$config")
        schema_dir = omni.repo.man.resolve_tokens("$root/_build/schema/omniSolid")

        # Match the suffix exactly; the build tree also holds .exp/.lib/.pdb.
        suffix = "*.pyd" if omni.repo.man.is_windows() else "*.so"
        modules = glob.glob(f"{build_dir}/_omni_solid{suffix}")
        if len(modules) != 1:
            raise RuntimeError(f"expected exactly one _omni_solid{suffix} in {build_dir}, found {modules}")
        module = modules[0]

        package_dir = os.path.join(staging_dir, "usd_brep")
        if os.path.exists(staging_dir):
            shutil.rmtree(staging_dir)
        os.makedirs(package_dir)

        shutil.copy2(module, package_dir)
        libs = _dependency_closure(module, [build_dir, f"{usd_dir}/lib", f"{usd_dir}/bin"])
        if not libs:
            # Everything below still succeeds; the wheel just fails to import.
            raise RuntimeError(f"no dependent libraries resolved for {os.path.basename(module)}")
        for lib in libs:
            shutil.copy2(lib, package_dir)
        omni.repo.man.logger.info(f"vendored {len(libs)} shared libraries")

        # OpenUSD looks for plugInfo.json relative to the directory holding its
        # libraries, so this tree has to sit beside them.
        shutil.copytree(f"{usd_dir}/lib/usd", os.path.join(package_dir, "usd"))
        shutil.copytree(schema_dir, os.path.join(package_dir, "omniSolid"))

        # The vendored OpenUSD runtime must travel with its full license texts;
        # THIRD_PARTY_NOTICES.md only summarizes them.
        usd_licenses = f"{usd_dir}/PACKAGE-LICENSES"
        if not os.path.isdir(usd_licenses):
            raise RuntimeError(f"OpenUSD package has no PACKAGE-LICENSES directory: {usd_licenses}")
        shutil.copytree(usd_licenses, os.path.join(staging_dir, "PACKAGE-LICENSES"))

        source_dir = omni.repo.man.resolve_tokens("$root/source/SmPyLib")
        shutil.copy2(f"{source_dir}/python/usd_brep/__init__.py", package_dir)
        for stub in glob.glob(f"{source_dir}/stubs/usd_brep/*"):
            shutil.copy2(stub, package_dir)

        # The BrepArray schema rules, as a separate top-level package: importing it must not
        # import usd_brep, which would load the vendored OpenUSD into the validator's process.
        validator_dir = os.path.join(staging_dir, "brep_validator")
        os.makedirs(validator_dir)
        for source in glob.glob(omni.repo.man.resolve_tokens("$root/tools/brep_validator/*.py")):
            shutil.copy2(source, validator_dir)

        if not omni.repo.man.is_windows():
            _patch_elf_binaries([os.path.join(package_dir, os.path.basename(p)) for p in [module] + libs])

        version_file = omni.repo.man.resolve_tokens(f"$root/{config['repo']['folders']['version_file']}")
        with open(version_file) as handle:
            version = wheel_version(handle.read())
        if version.internal:
            omni.repo.man.logger.warning(
                f"VERSION is marked internal; {version.version} must not be published externally"
            )
        _write_project_files(staging_dir, version.version)

        # Build runners have no uv on PATH; repo_man provisions its own.
        uv = str(get_uv())
        os.makedirs(install_dir, exist_ok=True)
        subprocess.run([uv, "build", "--wheel", "--out-dir", install_dir], cwd=staging_dir, check=True)

        wheel = max(glob.glob(f"{install_dir}/*.whl"), key=os.path.getmtime)
        if not omni.repo.man.is_windows():
            wheel = _retag_manylinux(uv, wheel, install_dir)
        _assert_publishable(wheel)
        omni.repo.man.logger.info(f"built {os.path.basename(wheel)}")

    return run_repo_tool


def _write_project_files(staging_dir: str, version: str):
    template = omni.repo.man.resolve_tokens("$root/tools/pyproject/pyproject.toml")
    with open(template) as handle:
        content = handle.read().replace('version = "0.0.0"', f'version = "{version}"')
    with open(os.path.join(staging_dir, "pyproject.toml"), "w") as handle:
        handle.write(content)
    shutil.copy2(omni.repo.man.resolve_tokens("$root/tools/pyproject/setup.py"), staging_dir)
    shutil.copy2(omni.repo.man.resolve_tokens("$root/LICENSE"), staging_dir)
    # The wheel redistributes OpenUSD and its runtime, so it has to carry the same
    # attribution the packman package puts in PACKAGE-LICENSES.
    shutil.copy2(omni.repo.man.resolve_tokens("$root/THIRD_PARTY_NOTICES.md"), staging_dir)
    # The repo README is a developer guide whose relative links do not resolve on
    # the package index, so the wheel carries its own.
    shutil.copy2(omni.repo.man.resolve_tokens("$root/tools/pyproject/README.md"), staging_dir)


def _retag_manylinux(uv: str, wheel: str, install_dir: str) -> str:
    """Replace the bare `linux_*` tag setuptools emits with a manylinux one.

    auditwheel derives the glibc floor from the symbols actually referenced,
    which is stricter than assuming the repo's ABI token. Because the libraries
    are already vendored and rpath'd above it moves nothing and only retags --
    relocating them into `usd_brep.libs` is precisely what would stop OpenUSD
    finding its plugInfo.json tree.
    """
    # --with patchelf: the runners carry 0.14.3 and auditwheel requires >= 0.14.5.
    # Our own patching above is already done, and works with the system copy.
    subprocess.run(
        [uv, "tool", "run", "--with", "patchelf", "auditwheel", "repair", wheel, "-w", install_dir],
        check=True,
    )
    os.remove(wheel)
    return max(glob.glob(f"{install_dir}/*.whl"), key=os.path.getmtime)


def _assert_publishable(wheel: str):
    """Fail here rather than at upload: PyPI rejects bare `linux_*` tags, and a
    missing platform tag means the extension would be served to every platform."""
    name = os.path.basename(wheel)
    if "cp312" not in name:
        raise RuntimeError(f"wheel is not tagged cp312: {name}")
    if omni.repo.man.is_windows():
        if "win_amd64" not in name:
            raise RuntimeError(f"wheel is missing the win_amd64 platform tag: {name}")
    elif "manylinux" not in name:
        raise RuntimeError(f"wheel has a non-manylinux tag; PyPI rejects bare linux_* tags: {name}")
