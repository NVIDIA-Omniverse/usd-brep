<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Examples

This directory contains independently runnable examples organized by the API
layer they exercise. Examples demonstrate how to compose supported operations;
they do not add new API entry points.

## Categories

- [PyAPI](PyAPI/README.md) — public `usd_brep` Python compositions.
- [SM_API](SM_API/README.md) — native examples using the stable C-style C++
  wrapper, including Boolean construction and bounding-box cost.
- [SMLib](SMLib/README.md) — native kernel examples, including STEP surface
  derivative units and parameterization. These are not stable wrapper APIs.

Each example should:

- run independently in an environment containing its required libraries;
- use only the API layer named by its directory;
- state its supported input scope and ownership behavior;
- have its important observable outcomes exercised by the owning test suite;
- be indexed by the README in its category.

For the end-to-end SMLib-to-USD mesh authoring workflow, see the related
[asset-generation script](../tools/scripts/example_generate_asset.py).
