# USD BRep

Standalone Omniverse repo containing USD BRep (NURBS boundary-representation solid modeling) libraries.

## Requirements

- [**Git**](https://git-scm.com/downloads) and [**Git LFS**](https://git-lfs.com/): Git LFS is required for the large binary fixtures tracked in `.gitattributes` (USD assets, STEP/IGES/JT models).

- **(Windows - C++ Only) Microsoft Visual Studio (2019 or 2022)** with the MSVC C++ toolset:
  - Install from [Visual Studio Downloads](https://visualstudio.microsoft.com/downloads/) (or **Build Tools for Visual Studio**) with the **Desktop development with C++** workload selected. A VS install without the C++ toolset is not enough.
  - The build uses this **host-installed** compiler (`msbuild.link_host_toolchain` in `repo.toml`), not Packman `msvc`.
  - `msbuild.vs_version` is pinned to `vs2019`, so VS 2022 users also need the **MSVC v142 build tools** component.
  - If discovery fails, set `msbuild.vs_path`, `msbuild.vs_version`, or both — see [repo_build toolchains](https://docs.omniverse.nvidia.com/kit/docs/repo_build/latest/docs/toolchains.html). These settings locate an installed toolset; they do not install one.

- **(Windows - C++ Only) Windows SDK**: Install alongside MSVC via the Visual Studio Installer. If discovery fails, set `msbuild.winsdk_path` in `repo.toml`.

- **(Windows) [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/cpp/windows/latest-supported-vc-redist)**: Required to run the built binaries, which link the VC runtime dynamically rather than bundling it. The **Desktop development with C++** workload above ships the redistributable, so a machine set up to build already has it — install it separately only on machines that run the binaries without Visual Studio.

- **(Linux) build-essential**: `sudo apt-get install build-essential`

All other build-time dependencies — OpenUSD, Python 3.12, pybind11, **premake**, and the `repo_*` tools — are pulled automatically via Packman when you build.

## Using this repo

Prerequisites are listed under [Requirements](#requirements). Use `./repo.sh` on Linux and macOS, `./repo.bat` on Windows. macOS: [macOS local development](#macos-local-development).

### Build it

Run prebuild before building a fresh checkout or after deleting `_build`. It stages
the omniSolid schema plugin in `_build/schema/omniSolid/resources`; `repo build`
alone does not create these resources, which USD tools require at runtime.

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

### Test it

```bash
./repo.sh test
./tests.sh linux-x86_64              # or macos-universal
```

### Format it

```bash
./repo.sh format
```

### Run the Python GUI

```bash
# Debug binaries by default (kernel Dump() echoes to the terminal).
uv run --locked --group gui python tools/scripts/smlib_gui.py

# Release binaries: add --release
uv run --locked --group gui python tools/scripts/smlib_gui.py --release
```

See `tools/smlib_gui/README.md` for smoke-test and troubleshooting commands.

### Explore examples

Runnable compositions organized by API layer live under [Examples](Examples/README.md).

## macOS local development

Apple Silicon only: `./repo.sh build -p macos-universal`. Build symlinks host Packman Python into `_build/target-deps/python` when available (`SMLIB_PYTHON_ROOT` to override).

**Python-only (default):** kernel, C++ tests, `_smlib_tests`, and `_smlib_dev` (imports need Tier 1 `_omni_solid`). No OpenUSD, OCCT, or `_omni_solid`.

```bash
./repo.sh build -p macos-universal -j10
./tests.sh macos-universal
```

**Tier 1 (optional):** local OpenUSD 3.12 staged via `setup-macos-local-deps.sh`.

```bash
export SMLIB_USD_ROOT=~/opt/usd-25.11-py312
./tools/scripts/setup-macos-local-deps.sh
./repo.sh build -p macos-universal -j10
./tests.sh macos-universal
```

Set `PYTHONPATH="$PWD/_build/macos-universal/release"` to import Python extensions. Packman OpenUSD/Python packages are not available on macOS. The macOS premake package (`premake@5.0.0-beta1+nv1-macos-universal`, `deps/host-deps.packman.xml`) is not on the public Packman server either; supply it through a user-level Packman configuration. Avoid `./repo.sh build -x`; delete `_build`, `_compiler`, and `_repo` to clean.

## Contributing

This project is currently not accepting contributions.

Issues and bug reports are welcome through the project issue tracker, and security
issues should be reported as described in `SECURITY.md`. See `CONTRIBUTING.md` for
details.

## License

USD BRep is licensed under the Apache License, Version 2.0. See
[LICENSE](LICENSE) for the terms governing use, reproduction, and distribution.

Copyright (c) 2020-2026, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

`repo.sh`, `repo.bat`, and `tools/repoman/repoman_bootstrapper.py` are licensed
under the MIT License instead; see `THIRD_PARTY_NOTICES.md` for the full text.

For third-party notices and bundled components, see `THIRD_PARTY_NOTICES.md`.

## Heal USD BRep assets

`usd.heal_file` heals BrepArray definitions in the layers used by the current USD
composition and writes the scene with the healed geometry, keeping its layers,
references, native instancing, transforms and materials. External layers used only
by unselected variants remain referenced but are not healed. Material subset
opinions outside the BRep's defining prim spec are rejected rather than lost.

```python
import usd_brep
usd_brep.usd.heal_file("input.usdc", "input_healed.usdc")
```
