---
name: smlib-python-dev-api-maintain
description: "Add or change `_smlib_dev` Python bindings in `source/SmPyDevLib`, including debugger topology, Draw extraction, viewport draw, and the tests facade."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Python Dev API Maintenance

Use this skill to author or review **developer-only** Python bindings that live in `_smlib_dev`, not in the stable `_omni_solid` module.

## Purpose

Keep `_smlib_dev` bindings, docstrings, stubs, and tests synchronized with kernel-debugger workflows (topology picking, loopuse/edgeuse walks, `Draw()` extraction, in-process `prog_test`).

For stable modeling APIs (`create_box`, booleans, fillets, queries on `Brep`/`Face`/`Edge`/`Vertex`), use `smlib-python-api-maintain` instead.

## Module Boundary

| Module | Audience | Examples |
|---|---|---|
| `_omni_solid` | Stable modeling / asset pipelines | `create_box`, `boolean_union`, `tessellate`, `brep.bounding_box()` |
| `_smlib_dev` | Debugger / GUI / kernel tooling | `topology_pick_ray`, `edgeuses`, `loopuses`, `draw.extract_draw_batches`, `tests.*` |

**Import order:** `import _omni_solid as sm` first, then `import _smlib_dev as smdev`. The dev module reuses stable type handles (`Brep`, `Face`, `Edge`, `Vertex`) and registers only dev-only types (`Loop`, `Loopuse`, `Edgeuse`).

Do **not** add debugger-only symbols back to `_omni_solid`.

## Open First

1. `AGENTS.md` — Developer API (`_smlib_dev`) section.
2. `.agents/operations/dev_topology.md` — topology pick, loopuse/edgeuse walks, Draw extraction.
3. `source/SmPyDevLib/src/SmPyDevMain.cpp` — module entry and registration order.
4. `source/SmPyDevLib/src/SmPyDevCommon.h` — shared dev helpers.
5. Matching binding file under `source/SmPyDevLib/src/`: `SmPyDevTopology.cpp`, `SmPyDevTests.cpp`, `SmPyDevUserTests.cpp`, or `SmPyViewportDraw.cpp` (`bind_dev_draw`).
6. `source/SmPyDevLib/stubs/smlib_dev/__init__.pyi`.
7. `source/SmPyDevLib/tests/test_smlib_dev.py`.
8. `tools/smlib_gui/runtime.py` — how the GUI imports `smdev`.

## Authoring Rules

- Register dev symbols only in `_smlib_dev` under `source/SmPyDevLib/`; keep `source/SmPyLib/` for stable `_omni_solid` only.
- Prefer **free functions** for topology walks on stable types (`smdev.edgeuses(edge)`, `smdev.face_loops(face)`); keep convenience methods on dev types (`Loopuse.edgeuses()`).
- Return kernel-owned handles with the same lifetime rules as stable types; Python does not own `Loop`, `Loopuse`, or `Edgeuse`.
- `bind_dev_draw()` exposes `smdev.draw.extract_draw_batches(...)` and `viewport_draw_*` helpers — not on `_omni_solid`.
- The `tests` submodule is a thin facade over `_smlib_tests`; do not re-export it from `_omni_solid`.
- Update stub, docstring, `test_smlib_dev.py`, and `.agents/operations/dev_topology.md` together when behavior changes.
- GUI code under `tools/smlib_gui/` should call `smdev` for debugger workflows; do not route through `_omni_solid`.

## Common Change Flow

1. Confirm the symbol is debugger-only and does not belong on the stable surface.
2. Edit the matching `SmPyDev*.cpp`, stub, and `test_smlib_dev.py` as one unit.
3. Migrate any `_omni_solid` callers (GUI, smoke tests) to `smdev`.
4. Run `tools/agent/run_smpylib_tests.sh --target test_smlib_dev` and GUI smoke when picking/draw/tests change.

## Example Requests

```text
Use $smlib-python-dev-api-maintain to add a free function for loopuse neighbours on _smlib_dev.
Use $smlib-python-dev-api-maintain to fix topology_pick_ray stub/docstring drift.
Use $smlib-python-dev-api-maintain to expose a new Draw extraction path on smdev.draw.
Use $smlib-python-dev-api-maintain to wire a prog_test suite through smdev.tests.
```

## Validation

```bash
./repo.sh build
tools/agent/run_smpylib_tests.sh --target test_smlib_dev
timeout 30s pyenv exec python tools/scripts/smlib_gui_smoke.py   # when GUI/pick/draw changes
```

## Limitations

- Do not use for stable `_omni_solid` bindings; use `smlib-python-api-maintain`.
- Do not use for GUI layout/UX without binding changes; use `usd-brep-tools-python-gui`.
- Do not use for kernel algorithm fixes; use `smlib-kernel-operations`.
