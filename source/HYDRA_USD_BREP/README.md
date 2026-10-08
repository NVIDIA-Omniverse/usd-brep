<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# hdUsdBrep

Hydra imaging plugin that imports USD BrepArray prims into SMLib and exposes
tessellated meshes to Hydra renderers, including Storm in usdview.

## Behavior and limits

- The plugin supplies transient Hydra meshes for viewing; it never writes mesh prims to
  the stage.
- Display needs the Hydra scene index path, the default in USD 25.11. With
  `USDIMAGINGGL_ENGINE_ENABLE_SCENE_INDEX=0` BrepArray prims are not drawn and a warning
  is logged.
- Tessellation uses the shared `SmTessellationDefaults`; settings are not read from
  USD attributes.
- Each BrepArray is tessellated once, when Hydra first reads its meshes, and Hydra
  does this for many BrepArrays in parallel. Peak memory while loading grows with the
  thread count; set `PXR_WORK_THREAD_LIMIT` (e.g. 8) to trade some load speed for a
  lower peak on large assemblies.
- BRep topology and geometry use the schema's `uniform` default values; time-sampled
  BRep geometry is not supported. USD transform and visibility animation are.
- Material subsets are read when the BrepArray is first imaged; editing subset
  membership or bindings afterwards requires reloading the stage. Geometry edits
  repopulate the prim.

## usdview

From the repository root, after a release build:

```sh
python tools/scripts/usdview_hdusdbrep.py /path/to/asset.usda
```

The launcher sets the plugin, library and Python paths for Linux or Windows and runs the
bundled usdview; extra arguments are passed to usdview. usdview needs the USD package's
Python requirements (PySide6 and PyOpenGL) installed for the bundled Python. usdview is
not yet supported on linux-aarch64.

## BRep boundary wireframe

The bundled `hdUsdBrepUsdview` Python plugin, which the launcher makes available, follows
usdview's display-mode selector. **Wireframe** shows BRep boundary curves; **Wireframe on
Surface** shows them over shaded faces; ordinary mesh prims are unaffected. Boundaries are sampled per topological edge at
5 degrees and cached with the surface meshes, so switching modes does not
retessellate; they remain available when surface tessellation fails.

`HdUsdBrepSetDisplayMode` queues a process-wide request and may be called from any
thread. Hosts must call `HdUsdBrepApplyPendingDisplayMode` between renders on the
scene-index creation/update thread, serialized with other scene updates; it applies
changes and sends dirty notifications only for scenes owned by that thread. The
usdview integration assumes one viewer per process. Without it, shaded surfaces with
boundaries hidden remain the default.
