<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Building USD BRep from source

Use `./repo.sh` on Linux and macOS, `./repo.bat` on Windows.

## Requirements

- [**Git**](https://git-scm.com/downloads)
- **Linux:** `sudo apt-get install build-essential`
- **Windows:** Visual Studio and the Windows SDK; see [Windows toolchain](#windows-toolchain).
- **macOS:** Apple Silicon, with limits; see [macOS local development](#macos-local-development).

On Linux and Windows, all other build dependencies (OpenUSD, Python 3.12, pybind11, premake and the `repo_*` tools) are downloaded by Packman when you build. On macOS some are not; see [macOS local development](#macos-local-development).

## Build it

Run prebuild before building a fresh checkout or after deleting `_build`. It stages
the omniSolid schema plugin in `_build/schema/omniSolid/resources`, which USD tools
need at runtime; `repo build` alone does not create it.

```bash
# Linux
./prebuild.sh
./repo.sh build

# macOS
./prebuild.sh -p macos-universal
./repo.sh build -p macos-universal

# Windows
./prebuild.bat
./repo.bat build
```

## Test it

`tests.sh` (`tests.bat` on Windows) runs the full test suite against your build tree; it does not build.

```bash
# Linux / macOS
./tests.sh linux-x86_64              # or linux-aarch64, macos-universal

# Windows
./tests.bat
```

`./repo.sh test` runs the same suite from a package made by `./repo.sh package`.

## Check a Brep's internal consistency

Source builds also include `_smlib_dev`, a developer module that is not in the
PyPI package. `assert_valid` runs the kernel's own consistency checks on a Brep
or any of its topology, in release and debug builds:

```python
import usd_brep
import _smlib_dev

brep = usd_brep.primitives.create_box((0.0, 0.0, 0.0), 1.0, 1.0, 1.0)
print(_smlib_dev.assert_valid(brep))  # True when every check passes
```

Failures are written to `stderr`; `_smlib_dev.assert_valid_reports` returns them
instead. Set `PYTHONPATH` to `_build/<platform>/release` to import both modules.

## Format it

```bash
./repo.sh format
```

## View BReps in usdview

```bash
python tools/scripts/usdview_hdusdbrep.py <asset.usda>
```

This needs a release build. See the [hdUsdBrep README](source/HYDRA_USD_BREP/README.md) for requirements and display modes.

## Run the Python GUI

The GUI runs through [uv](https://docs.astral.sh/uv/), which installs its Python dependencies from `uv.lock`.

```bash
# Debug binaries by default (kernel Dump() echoes to the terminal).
uv run --locked --group gui python tools/scripts/smlib_gui.py

# Release binaries: add --release
uv run --locked --group gui python tools/scripts/smlib_gui.py --release
```

See `tools/smlib_gui/README.md` for smoke-test and troubleshooting commands.

## Windows toolchain

- **Microsoft Visual Studio (2019 or 2022)** with the MSVC C++ toolset:
  - Install from [Visual Studio Downloads](https://visualstudio.microsoft.com/downloads/) (or **Build Tools for Visual Studio**) with the **Desktop development with C++** workload selected. A VS install without the C++ toolset is not enough.
  - The build uses this **host-installed** compiler (`msbuild.link_host_toolchain` in `repo.toml`), not Packman `msvc`.
  - `msbuild.vs_version` is pinned to `vs2019`, so VS 2022 users also need the **MSVC v142 build tools** component.
  - If discovery fails, set `msbuild.vs_path`, `msbuild.vs_version`, or both — see [repo_build toolchains](https://docs.omniverse.nvidia.com/kit/docs/repo_build/latest/docs/toolchains.html). These settings locate an installed toolset; they do not install one.
- **Windows SDK**: install alongside MSVC with the Visual Studio Installer. If discovery fails, set `msbuild.winsdk_path` in `repo.toml`.
- **[Microsoft Visual C++ Redistributable](https://learn.microsoft.com/cpp/windows/latest-supported-vc-redist)**: needed to run the built binaries, which link the VC runtime dynamically. The **Desktop development with C++** workload includes it, so install it separately only on machines that run the binaries without Visual Studio.

## macOS local development

Apple Silicon only: `./repo.sh build -p macos-universal`. The build symlinks the host's Packman Python into `_build/target-deps/python` when available (`SMLIB_PYTHON_ROOT` to override).

**Without OpenUSD (default):** builds the kernel, the C++ tests and the `_smlib_tests` and `_smlib_dev` Python modules. It does not build the USD support, the OCCT converters or the `usd_brep` Python API (`_omni_solid`); `_smlib_dev` builds but imports only in a build with OpenUSD.

```bash
./repo.sh build -p macos-universal -j10
./tests.sh macos-universal
```

**With OpenUSD (optional):** stage a local OpenUSD built for Python 3.12 with `setup-macos-local-deps.sh`.

```bash
export SMLIB_USD_ROOT=~/opt/usd-25.11-py312
./tools/scripts/setup-macos-local-deps.sh
./repo.sh build -p macos-universal -j10
./tests.sh macos-universal
```

Set `PYTHONPATH="$PWD/_build/macos-universal/release"` to import Python extensions. Packman OpenUSD/Python packages are not available on macOS. The macOS premake package (`premake@5.0.0-beta1+nv1-macos-universal`, `deps/host-deps.packman.xml`) is not on the public Packman server either; supply it through a user-level Packman configuration. Avoid `./repo.sh build -x`; delete `_build`, `_compiler`, and `_repo` to clean.
