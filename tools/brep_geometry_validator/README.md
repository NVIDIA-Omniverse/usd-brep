<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# brep_geometry_validator

Checks that the BReps in a USD file are geometrically and topologically sound. It imports
each `BrepArray` prim into SMLib and runs `SmBrep::AssertValid` on every BRep, which checks
for problems such as open loops, non-manifold edges and UV trim curves that leave their surface.
Programs can run the same check on one prim with `SmApiUsdGeometryValidate` (`SmApiUsdGeometryValidate.h`, `SM_API_USD`).

It complements the [USD validator](../brep_validator/README.md), which checks that the USD data
complies with the schema. A file can pass the USD validator and still fail here.

## Usage

```bash
brep_geometry_validator <input.usd> [more.usd ...]
```

| Option | Effect |
|---|---|
| `--level <0\|1\|2>` | AssertValid depth: 0 is fastest, 2 (the default) runs every check |
| `--workers <N>` | Validate prims in parallel (0 = automatic, 1 = serial, the default) |
| `--progress` | Print progress to stderr |
| `-q`, `--quiet` | Print only the per-file summary, not each failed check |

For each file it prints one line, followed by a `FAIL` line for each failed check, naming
the prim:

```
BREP_GEOMETRY file=<path> breps=<N> invalid=<M> checksFailed=<K>
```

The exit code is 0 when every BRep is valid, otherwise the number of invalid BReps (at most
250). A file that can't be opened or has no `BrepArray` prims also gives a non-zero code.

The BReps are checked as stored, without healing: healing could repair the defects being looked
for, and the healer can crash on invalid BReps.

## Finding the schema

The tool needs the omniSolid schema plugin. In a release package it finds the schema next to
`bin/`. To run it from a source build, set `OMNISOLID_PLUGIN_PATH`, or it exits with code 3:

```bash
export OMNISOLID_PLUGIN_PATH=<repo>/_build/schema/omniSolid/resources
_build/linux-x86_64/release/brep_geometry_validator model.usda
```
