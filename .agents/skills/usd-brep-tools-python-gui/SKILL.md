---
name: usd-brep-tools-python-gui
description: "Maintain the SMLib Python GUI workbench and headless renderer. Use for `tools/smlib_gui`, active objects, script execution, viewport behavior, file I/O, recursive model rendering, audit scripts, or GUI smoke tests."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Python GUI Maintainer

Use this skill for the Python developer workbench that provides daily debugger workflows for SMLib development.

## Purpose

Maintain the PySide6/PyVista workbench package in `tools/smlib_gui/`, its interactive and headless launchers under `tools/scripts/`, and its smoke validation. This includes active-object registry behavior, script execution/history, viewport rendering, tessellation controls, model file I/O, recursive batch rendering, selected-object operations, debug overlays, topology picking, and in-process `prog_test` integration.

## Prerequisites

- Confirm the task is GUI work, not a kernel algorithm fix or stable Python API design.
- Use existing `_omni_solid` APIs for modeling and queries; use `_smlib_dev` (`smdev`) for debugger topology pick, loopuse/edgeuse walks, Draw overlays, blend primitives, and in-process `prog_test`. Switch to `smlib-python-dev-api-maintain` or `smlib-python-api-maintain` only when a named GUI workflow is blocked by a missing binding.
- Use `usd-brep-test-selection` before adding broad validation beyond the GUI smoke hook.

## Open First

1. `tools/scripts/smlib_gui.py` and `tools/smlib_gui/` for the interactive workbench, numbered User Tests playback (`user_test.py`), tessellation, and viewport display. The script defaults to debug binaries; pass `--release` for release.
2. `tools/smlib_gui/headless.py` and `tools/scripts/smlib_gui_render.py` for reusable offscreen rendering and recursive batch orchestration.
3. `tools/smlib_gui/operations/file_io.py` for the shared supported-format load/save dispatch.
4. `tools/scripts/smlib_gui_smoke.py` for the current GUI and headless validation hook.
5. `tools/scripts/smlib_gui_occt_audit.py` when maintaining OCCT corpus diagnostics rather than the format-agnostic renderer.
6. `tools/smlib_gui/runtime.py` for `sm` / `smdev` import setup.
7. `source/SmPyLib/stubs/usd_brep/__init__.pyi`, `source/SmPyDevLib/stubs/smlib_dev/__init__.pyi`, and `source/SmPyLib/stubs/usd_brep/usd.pyi` for exposed Python API behavior.
8. The relevant `.agents/operations/*` guide and `usd-brep-test-selection` before broad validation.

## When To Use

- Editing `tools/smlib_gui/`, `tools/scripts/smlib_gui*.py`, or future Python GUI and headless-render modules.
- Fixing GUI active-object, selection, history, source regeneration, viewport, tessellation, overlay, file I/O, or operation-panel behavior.
- Adding supported model formats, recursive discovery, output layout, render reports, offscreen views, or corpus audit diagnostics.
- Adding or updating GUI smoke coverage for an existing workbench workflow.

## Not For

- General `_omni_solid` usage examples; use `smlib-python-api-use`.
- Stable or dev binding, stub, or docstring maintenance; use `smlib-python-api-maintain` or `smlib-python-dev-api-maintain`.
- Kernel operation failures outside the GUI surface; use the matching SMLib debug skill.

## Workflow

1. Classify the change as script-runner, active-object model, viewport/display, headless rendering, operation UI, file I/O, picking, debug overlay, or validation.
2. Preserve direct script execution as the primary reproducible debugger path. UI commands should update source/history or leave enough state to reproduce the command.
3. Keep active-object state separate from viewport state. Registry entries should own names, handles, type, metadata, selection, and debug overlays; PyVista actors should be disposable render projections.
4. Respect `_omni_solid` mutation conventions. Pure operations return new handles, consuming booleans invalidate inputs, and mutators return the same handle for chaining.
5. Prefer existing stable query and I/O APIs before adding bindings. Route topology pick, loopuse/edgeuse traversal, Draw extraction, and `prog_test` through `smdev`. Add minimal `_omni_solid` or `_smlib_dev` bindings only when a named debugger workflow is blocked.
6. Keep supported-format dispatch in `tools/smlib_gui/operations/file_io.py` so interactive and headless entry points load models consistently.
7. Keep the generic renderer format-agnostic. Put OCCT-specific mesh metrics and anomaly policy in `smlib_gui_occt_audit.py`.
8. Update `tools/scripts/smlib_gui_smoke.py` when a core workflow needs automated coverage.

## Headless Rendering Rules

- Set `QT_QPA_PLATFORM`, `PYVISTA_OFF_SCREEN`, cache directories, and display overrides before importing Qt, PyVista, or `tools.smlib_gui`.
- Use `HeadlessRenderSession` for reusable rendering behavior and `smlib_gui_render.py` for recursive file discovery, output mapping, reports, and process isolation.
- Preserve source-relative directory structure beneath an explicit output root. With no output root, write each model's renders beside that model.
- Isolate model workers by default. Native OCCT, SMLib, VTK, and Qt object lifetimes can make one failed input corrupt a later render in the same process.
- Use `--jobs N` to render independent input files concurrently in isolated subprocesses. The default is one job; `--no-isolation` cannot be combined with more than one job. Size `N` for available memory because each worker independently loads, tessellates, and renders a model, and treat `--worker-timeout` as a per-file timeout.
- The batch renderer keeps only the four-view `*_contact.png` composite by default. Use `--save-views` only when the four individual named-view PNGs are also needed.
- Render into a staging directory and publish images to the output directory only after the whole file succeeds, so a failed, timed-out, or cancelled render leaves no partial artifacts.
- Keep batch orchestration cancellable. Workers run in their own process group so a timeout or Ctrl-C can kill native children, cancel queued files, and exit non-zero instead of draining the queue.
- Deduplicate discovered inputs by resolved path so a symlink and its target do not produce two workers writing the same output files.
- Do not call global `QApplication.processEvents()` from a reusable offscreen renderer; it can service unrelated live `QtInteractor` instances.
- Emit machine-readable per-file status in addition to screenshots so batch failures are attributable to one source model.

## UI Rules

- Keep debugger workflows dense and explicit. Avoid landing-page or tutorial surfaces inside the workbench.
- Expose view, display, tessellation, selection, and object-inspection controls as persistent workbench controls, not modal-only state.
- Show errors from script execution, tessellation, and rendering in the GUI without losing the source text that produced them.
- Treat debug draw overlays as inspectable registry entries so they can be hidden, deleted, and associated with source objects.

## Troubleshooting

- If `_omni_solid` does not import, build first and check the runtime path setup before changing GUI code.
- If PyVista/Qt fails in headless validation, verify that offscreen environment variables were set before imports and report environment-specific failures separately from GUI logic failures.
- If the first model renders but a later model crashes or disappears, rerun with the default isolated workers before changing geometry or tessellation code.
- If a user test plays back the wrong source, inspect `tools/smlib_gui/user_test.py` and `user_test_catalog.py`, then verify Run / Reload / Replay through the left-panel editor.
- If GUI test_4 does not pick up C++ edits, rebuild `_smlib_dev` (`./repo.sh build`). Python Reload only re-reads `user_test.py`.
- If a selected-object operation uses the wrong object, inspect the active-object registry, selection IDs, and consumed-handle behavior before changing kernel calls.
- If tessellation or overlay rendering fails, record the originating object kind, display handle, and query result before adding fallbacks.

## Limitations

- Do not add new stable `_omni_solid` bindings for debugger convenience; put debugger-only surface on `_smlib_dev` via `smlib-python-dev-api-maintain`.
- Do not hide kernel errors behind GUI-only fallbacks. Preserve the source and diagnostics needed for a minimal repro.

## Validation

Run the smoke hook after GUI behavior changes:

```bash
uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py
```

Exercise recursive rendering after headless discovery, file I/O, output mapping, or render-session changes:

```bash
uv run --locked --group gui python tools/scripts/smlib_gui_render.py INPUT \
  --output-dir /tmp/smlib_gui_renders --jobs 4
```

Repeat a failed or timed-out file with `--jobs 1` before diagnosing shared-resource
pressure as a model or kernel defect.

For docs or skill-only changes, use path/format sanity checks. For operation
surfaces that call new or risky kernel bindings, use `usd-brep-test-selection` to
choose focused Python or kernel validation in addition to the GUI smoke test.

## Example Requests

```text
Use $usd-brep-tools-python-gui to add an active-object inspector to the SMLib GUI.
Use $usd-brep-tools-python-gui to add selected-object boolean commands to the Python workbench.
Use $usd-brep-tools-python-gui to update the smoke runner after adding GUI file import.
Use $usd-brep-tools-python-gui to fix PyVista actor cleanup when deleting debug overlays.
Use $usd-brep-tools-python-gui to make command-sidebar source regeneration preserve string literals.
Use $usd-brep-tools-python-gui to debug topology pick metadata shown after a ray hit.
Use $usd-brep-tools-python-gui to add a persistent tessellation control for max edge length.
Use $usd-brep-tools-python-gui to add a model format to the recursive headless renderer.
```
