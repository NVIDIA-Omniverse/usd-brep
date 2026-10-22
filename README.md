# USD BRep

Solid modeling for [OpenUSD](https://openusd.org/). USD BRep creates and edits
boundary-representation (BRep) solids and reads and writes them in USD as
`BrepArray` prims.

BRep faces and edges can be NURBS or analytic geometry. Plane, cylinder, cone, sphere
and torus faces are written to USD as analytic surfaces. Edge curves are written as
NURBS; analytic line, circle and ellipse edges in USD from other tools are read as
analytic curves. Other SMLib types, such as parabolas and surfaces of revolution, are
written as NURBS. See [supported geometry](.agents/operations/usd_import_export.md#supported-geometry)
for the SMLib class names.

## Repo contents

### USD

Libraries and tools that work on BReps stored in USD files:

- **[USD BRep schema](source/schema/README.md) (omniSolid):** defines the `BrepArray` prim, which stores BReps in
  USD with their full topology and NURBS or analytic geometry; see the
  [proposal](proposals/UsdSolid/README.md) and its
  [problem statement](proposals/cad_geometry/README.md).
- **[Import/export](source/SM_API_USD/inc/SmApiUsd.h):** reads `BrepArray` prims into
  SMLib BReps and writes SMLib BReps back to USD.
- **[USD Validator](tools/brep_validator/README.md):** checks that the BReps in a USD file comply with the BRep
  schema.
- **[Healer](source/SM_API_USD/inc/SmApiUsdHeal.h):** heals the BReps across a whole USD scene, preserving layers,
  references, instancing, transforms and materials.
- **[Geometry validator](tools/brep_geometry_validator/README.md):** runs AssertValid (see [Kernel](#kernel)) on every
  `BrepArray` prim in a USD file.
- **[Tessellator](source/SM_API_USD/inc/SmApiUsdTessellate.h):** writes a `UsdGeomMesh` for each BRep in the `BrepArray`
  prims of a USD file.
- **Viewers:**
  - **[hdUsdBrep](source/HYDRA_USD_BREP/README.md):** Hydra plugin for viewing USD BReps in usdview.
  - **[view_usd](tools/scripts/view_usd.py):** shows a USD file's BReps, tessellated, in a PyVista window.
  - **[smlib_gui](tools/smlib_gui/README.md):** desktop app for opening, inspecting and editing USD and SMLib BReps.
- **Converters:**
  - **OpenCASCADE:** [occt_to_usd](tools/occt_to_usd/README.md) and [usd_to_occt](tools/usd_to_occt/README.md)
    convert between OpenCASCADE and USD BReps.
  - **Utility libraries:** [BREP_USD_DATA](source/BREP_USD_DATA/README.md),
    [BREP_STAGING](source/BREP_STAGING/README.md) and [USD_BREP_UTILS](source/USD_BREP_UTILS/README.md)
    support writing USD converters to and from other formats.
- **APIs:** [C++](source/SM_API_USD) and [Python](tools/pyproject/README.md) (USD).

### Kernel

The SMLib kernel, and tools that work on BReps in memory:

- **[Kernel (SMLib)](source/SMLib):** creates and edits BRep solids, with booleans, fillets,
  sweeps and offsets.
- **[Healer](source/SM_API/inc/SmApiHeal.h):** fixes common geometric and topological defects in BReps.
- **[Tessellator](source/SMLib/inc/SmTess.h):** meshes BReps into triangles. The Hydra plugin uses it to draw
  BReps in usdview.
- **Geometry validator (AssertValid):** checks the geometric quality and topological
  consistency of BReps in memory.
- **APIs:** [C++](source/SM_API) and [Python](tools/pyproject/README.md) (SMLib).
- **[Examples](Examples/README.md):** runnable Python and C++ examples for each API layer.

### Agents

Guidance for coding agents, starting from [AGENTS.md](AGENTS.md):

- **[Docs](.agents/docs/OVERVIEW.md):** the library's layers and key concepts, what each
  error code means and how to fix it.
- **[Operations](.agents/operations/INDEX.md):** a guide for each API area, such as booleans,
  fillets, tessellation, healing and USD import/export.
- **[Skills](.agents/skills):** workflows for debugging kernel operations, using and
  maintaining the C++ and Python APIs, working with the USD schema and converters, choosing
  tests and writing bug reports.

## Documentation

[nvidia-omniverse.github.io/usd-brep](https://nvidia-omniverse.github.io/usd-brep/), including the Python and C++ API reference.

## Quick start

The Python package is on PyPI for CPython 3.12 on Linux (x86_64, aarch64) and Windows:

```bash
pip install usd-brep
```

```python
import usd_brep

sphere = usd_brep.primitives.create_sphere((0.0, 0.0, 0.0), 1.0)
box = usd_brep.primitives.create_box((0.0, 0.0, 0.0), 1.0, 1.0, 1.0)
cut = usd_brep.booleans.boolean_difference(sphere, box)  # a sphere missing one octant

usd_brep.usd.export_brep(cut, "cut.usda")
(brep,) = usd_brep.usd.import_breps("cut.usda")
```

To repair the BReps in an existing USD scene, `usd.heal_file` heals the `BrepArray`
geometry in the layers the scene uses and writes the scene with its layers,
references, instancing, transforms and materials kept:

```python
usd_brep.usd.heal_file("input.usdc", "input_healed.usdc")
```

See the [package README](tools/pyproject/README.md) for the API's submodules and
[Examples](Examples/README.md) for runnable examples in Python and C++.

## Issues

- **Known issues:** [KNOWN_ISSUES.md](KNOWN_ISSUES.md), and the [open issues](https://github.com/NVIDIA-Omniverse/usd-brep/issues) on GitHub.
- **Fixed issues:** the "Fixed" sections of the [CHANGELOG](CHANGELOG.md), and the [closed issues](https://github.com/NVIDIA-Omniverse/usd-brep/issues?q=is%3Aissue%20is%3Aclosed) on GitHub.

## Build from source

To build the libraries, tests and tools yourself, see [BUILDING.md](BUILDING.md).
It covers requirements for Linux, Windows and macOS, building, testing, viewing
BReps in usdview and the Python GUI.

## Contributing

This project is currently not accepting contributions.

Issues and bug reports are welcome through the [issue tracker](https://github.com/NVIDIA-Omniverse/usd-brep/issues), and security
issues should be reported as described in `SECURITY.md`. See `CONTRIBUTING.md` for
details.

## License

USD BRep is licensed under the Apache License, Version 2.0. See
[LICENSE](LICENSE) for the terms governing use, reproduction, and distribution.

Copyright (c) 2020-2026, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

`repo.sh`, `repo.bat`, and `tools/repoman/repoman_bootstrapper.py` are licensed
under the MIT License instead; see `THIRD_PARTY_NOTICES.md` for the full text.

For third-party notices and bundled components, see `THIRD_PARTY_NOTICES.md`.
