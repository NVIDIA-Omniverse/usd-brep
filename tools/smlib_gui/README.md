<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Python GUI

Run the GUI through the repo `uv.lock` so PySide6, PyVista, pyvistaqt, VTK, and
NumPy stay on a known-compatible Python 3.12 stack:

```bash
# Debug SMLib/USD (default). Kernel Dump() echoes to the terminal.
uv run --locked --group gui python tools/scripts/smlib_gui.py

# Release binaries (Dump() is silent).
uv run --locked --group gui python tools/scripts/smlib_gui.py --release

# Pause after native modules load so you can attach CodeLLDB.
uv run --locked --group gui python tools/scripts/smlib_gui.py --wait-for-lldb
```

The interactive launcher looks under `_build/<platform>/debug` by default.
Build debug SMLib first, or pass `--release` if you only have a release tree.

Run the smoke check with the same locked environment:

```bash
uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py
uv run --locked --group gui python tools/scripts/smlib_gui_smoke.py --show --timeout-ms 2000
```

Render every supported model below a directory without opening the GUI:

```bash
uv run --locked --group gui python tools/scripts/smlib_gui_render.py TestFiles/occt_breps
uv run --locked --group gui python tools/scripts/smlib_gui_render.py TestFiles --output-dir /tmp/smlib_renders
```

The renderer recursively discovers `.brep`, `.smb`, `.usd`, `.usda`, and
`.usdc` files. With no output directory, it writes the four-view contact sheet
beside each source. With `--output-dir`, it preserves the input directory
hierarchy below that root. For example, `parts/a/box.smb` produces
`renders/a/box.smb_contact.png`.

Only the `*_contact.png` composite is kept by default. Pass `--save-views` to
also keep the four individual named-view PNGs (`*_isometric.png`,
`*_front.png`, `*_top.png`, `*_right.png`) and to list them under `views` in
the report. Earlier revisions wrote those files unconditionally, so callers
that consumed the individual views need `--save-views` added.

Each input runs in an isolated worker by default, so a native converter or VTK
failure does not stop the remaining directory. Use `--jobs N` to render N files
concurrently, sized for available memory because each worker loads,
tessellates, and renders independently; `--worker-timeout` is per file. The
command writes `smlib_gui_render_report.json` with per-file status; use
`--no-isolation` only when process startup cost matters more than fault
isolation, and note that it cannot be combined with more than one job.
Tessellation, image size, projection, views, and BRep edge overlays are
configurable through `--help`.

Build the repo first if `_omni_solid` or `_smlib_dev` fails to import:

```bash
./repo.sh build
```

The GUI runtime still adds the built SMLib and USD paths at startup. The `uv`
environment only controls third-party Python packages, especially the
Qt/PyVista/VTK combination that is sensitive to version drift.

## User Tests

The left panel runs numbered user tests. The
cases live in `tools/smlib_gui/user_test.py` as `test_N()` functions. Fill a
case with `_omni_solid` (`sm`) and `_smlib_dev` (`smdev`) commands, pick that
number, and press **Run** to play the script back into the viewport.

`sm` and `smdev` are injected at playback; do not import them in the catalog
file. Edit in the GUI or in the catalog file and press **Reload**. **Save**
writes the editor back to `test_N()`. **Replay** / **Prev** / **Next** walk
successful playback history. Ctrl+Return runs the current editor contents
without requiring a restart. Collapse the panel to a thin strip when you need
more viewport space.

Kernel-direct C++ tests live in `source/SmPyDevLib/src/SmPyDevUserTests.cpp`.
Edit `smdev.user_test()`, rebuild `_smlib_dev`, then pick **test_4** and press
**Run**. Assign the returned Brep (`result = smdev.user_test()`) so the GUI
can tessellate it. `smdev.user_test_example()` is the kernel analog of
**test_2**. Python Reload does not pick up C++ edits.
