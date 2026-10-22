<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD BRep schema (omniSolid)

Defines how BReps are stored in USD; the schema is
[omniSolid/resources/schema.usda](omniSolid/resources/schema.usda). It is a codeless schema: OpenUSD reads it from the
plugin files at run time, so it needs no compiled code. The design is described in the
[proposal](../../proposals/UsdSolid/README.md) and its
[problem statement](../../proposals/cad_geometry/README.md).

- **`BrepArray`** is a `UsdGeomGprim` that holds any number of BReps. It stores their
  topology (regions, shells, faces, loops, edges and vertices) as flat arrays, with objects
  referring to each other by index.
- **Geometry API schemas**, applied to a `BrepArray`, hold the geometry its faces and
  edges use:
  - Surfaces: `BrepSurfaceNurbAPI`, `BrepSurfacePlaneAPI`, `BrepSurfaceCylinderAPI`,
    `BrepSurfaceConeAPI`, `BrepSurfaceSphereAPI` and `BrepSurfaceTorusAPI`.
  - 3D curves: `BrepCurve3dNurbAPI`, `BrepCurve3dLineAPI`, `BrepCurve3dCircleAPI` and
    `BrepCurve3dEllipseAPI`.
  - UV trim curves: `BrepCurveUvNurbAPI`.
  - Vertex points: `BrepPointAPI`.

## Files

All in `omniSolid/resources`:

| File | Contents |
|---|---|
| [`schema.usda`](omniSolid/resources/schema.usda) | The schema, with documentation for every attribute. Edit this file. |
| [`generatedSchema.usda`](omniSolid/resources/generatedSchema.usda), [`plugInfo.json`](omniSolid/resources/plugInfo.json) | Generated from `schema.usda` by `usdGenSchema`. |

`prebuild.sh` (`prebuild.bat` on Windows) regenerates the generated files when `schema.usda`
has changed, which needs the Python `jinja2` module; commit the results. It then copies
them to `_build/schema/omniSolid/resources`.

## Registering the schema

OpenUSD must load the plugin before any stage is opened, or `BrepArray` prims can't be read.
The libraries and tools register it from `OMNISOLID_PLUGIN_PATH`: set it to
`_build/schema/omniSolid/resources` in a source build, or `omniSolid/resources` in a release
package. Importing the `usd_brep` Python package sets it to the package's own copy, if it
isn't already set. Adding the folder to `PXR_PLUGINPATH_NAME` also works.

To check that a file complies with the schema, use the
[USD validator](../../tools/brep_validator/README.md).
