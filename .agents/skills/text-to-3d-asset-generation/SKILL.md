---
name: text-to-3d-asset-generation
description: Use when building a high-fidelity CAD/USD asset from a text asset name, including single assets or compound asset scenes. Do not use for low-level kernel/API work (use the `smlib-*` skills) or for debugging an existing asset's tessellation (use `smlib-kernel-tessellation`).
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Text-to-3D Asset Generation

For text-to-3D, image-to-3D, or compound CAD/USD asset authoring, read and follow
[.agents/docs/generate_asset.md](../../docs/generate_asset.md).

## Workflow

1. Confirm the repository build provides `_omni_solid` and `pxr`.
2. Follow the playbook's reference gathering, parametric modeling, tessellation, USD export, and
   self-check guidance.
3. For multi-part assets or compound scenes, also apply its composition, penetration, and envelope
   checks.
4. When reference material exists, use the documented semantic and visual comparison guidance.
5. Deliver the authoring script, mesh-only USDA, screenshot, and validation summary.

## Validation

Use the playbook's [runtime setup](../../docs/generate_asset.md#5-minimum-working-template),
run your authoring script, then render the resulting asset from the repository root.
Replace `<preview.png>` and `<asset.usda>` with your output paths:

```shell
uv run --locked --group gui python tools/scripts/view_usd.py --screenshot <preview.png> <asset.usda>
```

Inspect the generated screenshot and apply the playbook's [quick self-check](../../docs/generate_asset.md#11-quick-self-check-before-shipping) before reporting completion.

## Boundaries

For kernel, binding, tessellation, or USD BRep issues, use the corresponding operation-specific
skill. Do not conceal an operation failure by hand-editing the generated USD.
