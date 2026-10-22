<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# usd_to_occt

Converts the `BrepArray` prims in a USD file to OpenCASCADE ASCII `.brep` files. It does not
need OpenCASCADE: it reads USD through [USD_BREP_UTILS](../../source/USD_BREP_UTILS/README.md)
and writes the `.brep` format itself (`source/OCCT_BREP_EXPORT`).

```bash
usd_to_occt <input.usd> <output.brep>        # one input
usd_to_occt -o <dir> <input.usd> [more...]   # writes <dir>/<input name>.brep for each input
usd_to_occt --list <input.usd> [more...]     # lists the BrepArray prims without converting
```

A file with several `BrepArray` prims gives one `.brep` per prim, numbered `out_0.brep`,
`out_1.brep` and so on. `-q` hides the line printed for each prim written.

The exit code is 0 when every prim converts and 1 when any prim or input fails. It is 2 when
an input can't be read and nothing was written.

[occt_to_usd](../occt_to_usd/README.md) converts the other way. Like it, `usd_to_occt` needs
`OMNISOLID_PLUGIN_PATH` set to `<repo>/_build/schema/omniSolid/resources`, or it exits with
code 3. It is built from source and is not in the release packages.
