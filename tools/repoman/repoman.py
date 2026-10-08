# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import io
import contextlib
import shutil
from pathlib import Path

import packmanapi
from repoman_bootstrapper import repoman_bootstrap


THIS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.join(THIS_DIR, "../..")
REPO_DEPS_FILE = os.path.join(REPO_ROOT, "deps/repo-deps.packman.xml")

_MACOS_PYTHON = Path.home() / "Library/Application Support/packman-cache/python/3.12.13-macos-aarch64"


def bootstrap():
    """
    Bootstrap all omni.repo modules.

    Pull with packman from repo.packman.xml and add them all to python sys.path to enable importing.
    """
    with contextlib.redirect_stdout(io.StringIO()):
        deps = packmanapi.pull(REPO_DEPS_FILE)

    for dep_path in deps.values():
        if dep_path not in sys.path:
            sys.path.append(dep_path)

    # Add this repo root, as we are repoman itself!
    sys.path.append(THIS_DIR)
    sys.path.append(REPO_ROOT)


def _platform_target_from_argv(argv):
    for i in range(1, len(argv)):
        arg = argv[i]
        if arg in ("-p", "--platform-target") and i + 1 < len(argv):
            return argv[i + 1]
        if arg.startswith("-p="):
            return arg[3:]
        if arg.startswith("--platform-target="):
            return arg[len("--platform-target=") :]
    return None


def _apply_macos_repo_tool_patches(repo_root):
    """Patch pulled repo_man/repo_build for Apple Silicon macOS builds."""
    root = Path(repo_root)
    patches = (
        (
            "_repo/deps/repo_man/omni/repo/man/guidelines.py",
            """    if arch == "AMD64":
        arch = "x86_64"
    return arch""",
            """    if arch == "AMD64":
        arch = "x86_64"
    if arch == "arm64":
        arch = "aarch64"
    return arch""",
        ),
        (
            "_repo/deps/repo_build/omni/repo/build/build.py",
            """MSVC_PLATFORMS = ["windows-x86_64"]
# TODO: Support building from macos-arm64
MAKE_PLATFORMS = ["linux-x86_64", "linux-aarch64", "macos-x86_64"]""",
            """MSVC_PLATFORMS = ["windows-x86_64"]
MAKE_PLATFORMS = ["linux-x86_64", "linux-aarch64", "macos-x86_64", "macos-aarch64"]""",
        ),
    )

    for rel_path, old, new in patches:
        path = root / rel_path
        if not path.is_file():
            raise RuntimeError(
                f"Repo tool patch failed for {rel_path}: file not found. "
                "Ensure packman deps were pulled and update tools/repoman/repoman.py if paths changed."
            )
        text = path.read_text(encoding="utf-8")
        if new in text:
            continue
        if old not in text:
            raise RuntimeError(
                f"Repo tool patch failed for {rel_path}: expected content not found. "
                "The pinned repo_man/repo_build version may have changed; update tools/repoman/repoman.py."
            )
        path.write_text(text.replace(old, new, 1), encoding="utf-8")


def _ensure_macos_python_deps(repo_root):
    """Symlink host Packman Python when the macos-universal target package is unavailable."""
    host_python = Path(os.environ["SMLIB_PYTHON_ROOT"]) if "SMLIB_PYTHON_ROOT" in os.environ else _MACOS_PYTHON
    if not (host_python / "include/python3.12").is_dir():
        return

    link = Path(repo_root) / "_build/target-deps/python"
    if link.is_symlink():
        try:
            if link.resolve() == host_python.resolve():
                return
        except OSError:
            pass

    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink() or link.is_file():
        link.unlink()
    elif link.is_dir():
        shutil.rmtree(link)
    link.symlink_to(host_python, target_is_directory=True)


if __name__ == "__main__":
    is_macos_build = sys.platform == "darwin" and len(sys.argv) >= 2 and sys.argv[1] == "build"
    if sys.platform == "darwin":
        os.environ.setdefault("UV_SYSTEM_CERTS", "true")
        if is_macos_build and "-h" not in sys.argv and "--help" not in sys.argv:
            if _platform_target_from_argv(sys.argv) != "macos-universal":
                print(
                    "macOS build requires -p macos-universal (see README.md — macOS local development).",
                    file=sys.stderr,
                )
                sys.exit(1)

    repoman_bootstrap()
    bootstrap()
    if is_macos_build:
        _apply_macos_repo_tool_patches(REPO_ROOT)
        _ensure_macos_python_deps(REPO_ROOT)
    import omni.repo.man

    omni.repo.man.main(REPO_ROOT)
