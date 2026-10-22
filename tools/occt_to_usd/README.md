<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# occt_to_usd

Converts an OpenCASCADE `.brep` file to a USD file with a `BrepArray` prim. It reads both
ASCII and binary `.brep` files, and does not need OpenCASCADE: it parses the file itself
(`source/OCCT_BREP_IMPORT`) and writes USD through [BREP_STAGING](../../source/BREP_STAGING/README.md)
and [BREP_USD_DATA](../../source/BREP_USD_DATA/README.md).

```bash
occt_to_usd <input.brep> <output.usda|usdc>
```

- Planes, cylinders, cones, spheres, tori, lines, circles and ellipses are written as
  analytic geometry where the schema allows. Other geometry, such as surfaces of revolution
  and extrusion, Bezier, parabolas and hyperbolas, is written as NURBS.
- Solids, shells, faces and compounds are converted, with their location transforms
  applied to the geometry. A file with no faces, such as a lone edge, is rejected.

`occt_dump <file.brep> [more.brep ...]` is built alongside it and prints a summary of each
file's contents: its encoding, the number and type of curves, surfaces and shapes, and
whether the file is well formed.

[usd_to_occt](../usd_to_occt/README.md) converts the other way.

`occt_to_usd` needs the omniSolid schema plugin: set `OMNISOLID_PLUGIN_PATH` to
`<repo>/_build/schema/omniSolid/resources`, or it exits with code 3. These tools are built
from source and are not in the release packages.
