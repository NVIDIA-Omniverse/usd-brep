<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# prog_test

`prog_test` is the native C++ regression harness for the SMLib kernel. It exercises
geometry, topology, modeling operations, and polygon output using both models
constructed in code and saved regression fixtures. The tests build into the
`prog_test` shared library; `prog_test_app` is its command-line runner. The same
library is available to developer tools through `_smlib_tests`.

This directory covers kernel behavior. The repository's API, USD, Python binding,
and format-converter integration tests have separate harnesses. See the
[repository testing instructions](../../BUILDING.md#test-it) for the broader gate.

## Build and run

Follow the [repository build instructions](../../BUILDING.md#build-it) to configure
a checkout, acquire dependencies, and build the libraries and test application.
On Linux, after generating the makefiles, a focused rebuild is:

```sh
make -C _compiler/gmake2 prog_test_app config=release_x86_64 -j8 --output-sync
make -C _compiler/gmake2 prog_test_app config=debug_x86_64 -j8 --output-sync
```

These targets also build their `prog_test` and `SMLib` dependencies. Source files
are collected by the globs in [premake5.lua](premake5.lua); regenerate the project
files after adding or removing a source file, for example with `./repo.sh build -g`.

Run these commands from the repository root:

```sh
# List the registered suites and their default status.
tests/prog_test/prog_test.sh linux-x86_64 --list-suites

# Run every default suite.
tests/prog_test/prog_test.sh linux-x86_64

# Run one suite, or several suites in the specified order.
tests/prog_test/prog_test.sh linux-x86_64 --suite booleans
tests/prog_test/prog_test.sh linux-x86_64 --suite topology --suite healing

# Stitching is registered but requires explicit selection.
tests/prog_test/prog_test.sh linux-x86_64 --suite stitch
```

The shell launcher selects **release** binaries and changes to this directory so
fixture paths resolve. For macOS, use `macos-universal` instead of `linux-x86_64`
after building that platform. The Windows launcher also selects release binaries
and expects to be invoked from the repository root:

```bat
tests\prog_test\prog_test.bat --list-suites
tests\prog_test\prog_test.bat --suite topology --suite healing
```

To run **debug**, invoke the debug application directly from `tests/prog_test`:

```sh
cd tests/prog_test
../../_build/linux-x86_64/debug/prog_test_app --suite topology --suite healing
# Omit --suite arguments to run all default suites.
```

On Windows, the corresponding executable is
`..\..\_build\windows-x86_64\debug\prog_test_app.exe`. Keep the application and
loaded kernel/test libraries in the same build configuration.

### Command-line selection

- No arguments runs the suites marked as default in the registry, in registry order.
- `--list-suites` prints the available names without executing tests.
- Repeated `--suite <name>` arguments run only those suites, in command-line order.
- The older `--test merge` and `--test stitch` spellings select `booleans` and
  `stitch`, respectively. Prefer `--suite` for new commands.
- Names are case-sensitive. An unknown name, missing argument, or unsupported
  option prints usage and returns a nonzero exit code.

There is no command-line option for an individual numbered regression case or
fillet solver variant. Use the suite's printed case identifiers and source entry
point to narrow a debugger session. The registry is the authoritative list of
selectable tests; a helper or demo being present in a source file does not mean
it runs by default.

## Suite reference

The table follows the order in [the registry](src/prog_test.cpp). All suites are
included in a default run **except `stitch`**. SSI means surface/surface
intersection; CCI means curve/curve intersection.

| Suite | Entry point and primary source | Coverage |
|---|---|---|
| `topology` | `my_test_topology()` in [smTopoTest.cpp](src/smTopoTest.cpp), calling `my_test_faces()` in [smFaceTest.cpp](src/smFaceTest.cpp) | Topology construction and direct edits; shell/wire/loop incidence; pointer and ownership validation; mutation notifications; edge geometry repairs; seam insertion; ProtoEdge combination and overlapping-group reference refresh. The face group checks loop orientation, nesting, containment, seam encounters, vertexuse UV corners, derived trim curves, and UV endpoint gaps. |
| `healing` | `my_test_healing()` in [smHealTest.cpp](src/smHealTest.cpp) | Missing-seam detection and repair; vertex-only closed faces and degenerate boundaries; pole-crossing splits; region/containment preservation; property-cache refresh; continuation after an unsuccessful repair; stability through repeated healing. |
| `booleans` | `my_test_merge()` in [smMergeTest.cpp](src/smMergeTest.cpp) | Primitive and fixture-based Boolean operations, manifold and nonmanifold inputs, multi-body and Boolean-tree cases, shell separation, and Boolean stress cases. |
| `offset` | `my_offset_regression()` in [smOffsetTest.cpp](src/smOffsetTest.cpp) | Solid and surface offsets and shelling. Drives the numbered `my_offset_demo()` and `my_shell_demo()` cases and checks the resulting geometry/topology. |
| `fillets` | `my_test_fillet_const_radius()` in [prog_test.cpp](src/prog_test.cpp), calling `my_test_fillet_regression()` in [smFilletTest.cpp](src/smFilletTest.cpp) | The numbered fillet regression groups, including edge blends and corner construction. The registered suite selects `SM_FS_CONST_RADIUS`; other solver modes accepted by the C++ regression function are not separate CLI suites. |
| `local-ops` | `my_local_op_regression()` in [smFilletTest.cpp](src/smFilletTest.cpp) | The numbered local-operation examples in `my_local_op()`, including fillet and corner operations on existing models. |
| `sweeps` | `my_sweep_regression()` in [smSweepTest.cpp](src/smSweepTest.cpp) | Sweep construction examples and topology/region lifetime during sweeping, including the regression for a region deleted during stitching. |
| `primitives` | `my_primsol_regression()` in [smSweepTest.cpp](src/smSweepTest.cpp) | Primitive solid construction through the primitive demo cases, plus explicit invalid-dimension checks for cylindrical boxes. |
| `tessellation` | `my_test_tess()` in [smTessTest.cpp](src/smTessTest.cpp) | BRep-to-polygon conversion and polygon operations: face/normal tables, polygon solver cases, triangle strips, merging, sectioning/trimming, and measured/quadric decimation. Includes saved mesh fixtures. |
| `unit` | `my_unit_test_suite()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp), primarily using [smCurveTest.cpp](src/smCurveTest.cpp) and [smSurfaceTest.cpp](src/smSurfaceTest.cpp) | Foundational containers and math, analytic and B-spline geometry, curve/surface methods, intersections and numerical solves, curve dropping, projection, and sectioning. See the breakdown below. |
| `trimmed-surfaces` | `my_test_tsurface_regression()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp), with helpers in [smTopoTest.cpp](src/smTopoTest.cpp) | Creating and trimming surfaces with curves, analytic trimmed faces, and geometric queries/intersections on trimmed models. |
| `brep-import` | `my_test_brep_import()` in [smTopoTest.cpp](src/smTopoTest.cpp) | Low-level BRep construction regression through `my_test_MakeFace()`, including face construction from boundary curves and projection/trimming. It does not run a STEP parser or a translator corpus. |
| `ssi-analytic` | `my_test_ssi_analytic()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp) | The `my_ssi_analytic_demo()` cases for intersections between analytic surfaces, with expected solution counts and special configurations. |
| `ssi-advanced` | `my_test_ssi_adv()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp) | Saved surface-pair regressions for extrusion and revolution surfaces and other difficult intersections, including coincidence, tangency, seams, and degenerate solutions. |
| `cci-advanced` | `my_test_cci_adv()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp) | The saved `cci_bug_rhino1.smp` curve-pair regression, checked through the global curve/curve intersector. General CCI coverage is also in `unit`. |
| `section-advanced` | `my_test_sec_adv()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp) | Sectioning transformed torus fixtures in different directions, with expected intersection counts around singular configurations. |
| `silhouette-advanced` | `my_test_sil_adv()` in [smAdvancedTest.cpp](src/smAdvancedTest.cpp) | Silhouette queries on saved trimmed models using different view directions and perspective settings, with expected solution counts. |
| `stitch` | `my_stitching_demo()` in [smMergeTest.cpp](src/smMergeTest.cpp) | Explicit stitching of saved fillet models at different tolerances, reporting stitched/lamina edges and vertex/edge gaps, followed by a Boolean exercise. **Explicit selection only.** |

### What `unit` includes

The historical `unit` name covers a broad collection rather than just isolated
utility functions:

- **Containers and math:** `SmTArray`, `SmMap`, `SmObject`, vectors, matrices,
  axis placements, extents, pseudo-boxes, solution arrays, and polynomials.
- **Curves:** analytic construction, B-spline methods and NLib operations,
  curve/point queries, intersection tolerance, tangency and coincidence,
  self/projected intersections, curve properties, and extrema/directional solves.
- **Surfaces:** analytic and B-spline construction and methods, component
  constructors, curve/surface and surface/point queries and solves,
  surface/surface solves, curve dropping and projection, poles, and sections.

Some geometric helpers are also used by other suites. Source-file ownership and
suite membership are therefore not one-to-one. Historical `if (FALSE)` blocks
and build-conditional paths are not executed coverage merely because they appear
in these files.

## Source organization and adding tests

The suite table identifies the main dispatchers. Other shared pieces are:

| File | Role |
|---|---|
| [src/prog_test.cpp](src/prog_test.cpp) and [inc/prog_test.h](inc/prog_test.h) | Suite registry, exported native runner interfaces, result structure, timing, and CLI-run log setup. |
| [src/prog_test_app.cpp](src/prog_test_app.cpp) | Argument parsing, listing/selecting suites, default-run entry point, and process exit status. |
| [src/smCurveTest.cpp](src/smCurveTest.cpp) / [inc/smCurveTest.h](inc/smCurveTest.h) | Curve, container, and mathematical test implementations and reusable helpers, primarily reached through `unit`. |
| [src/smSurfaceTest.cpp](src/smSurfaceTest.cpp) / [inc/smSurfaceTest.h](inc/smSurfaceTest.h) | Surface construction and query tests, direct curve dropping, projection, and geometry helpers. |
| [src/smTestBrep.cpp](src/smTestBrep.cpp) / [inc/smTestBrep.h](inc/smTestBrep.h) | Analytic fixtures and insertion checks shared by face, topology, and healing regressions. |
| [src/smFeatureTest.cpp](src/smFeatureTest.cpp) / [inc/smFeatureTest.h](inc/smFeatureTest.h) | Defeaturing and feature/fillet rebuilding tests. `my_test_feature()` is compiled but is not registered as a CLI suite or called by the default dispatcher. |
| [src/StdAfx.cpp](src/StdAfx.cpp) / [inc/StdAfx.h](inc/StdAfx.h) | Common compilation includes. |
| [premake5.lua](premake5.lua) | Shared-library and executable projects and platform build settings. |

Choose a test's home by the behavior it verifies. Direct topology edits and
notifications belong in `smTopoTest.cpp`; face classification and derived UV data
belong in `smFaceTest.cpp`; complete repair workflows and repeated-healing checks
belong in `smHealTest.cpp`. Curve/surface algorithms, sweeps, booleans, fillets,
offsets, and tessellation belong with their corresponding suites. Converter
parsing, source fidelity, and corpus acceptance belong to the converter's tests.

Keep helpers local when only one source file uses them. Share fixtures when
several tests need the same input, while keeping each test independent. For
example, the shared torus fixtures let `topology` check direct seam insertion and
`healing` check subsequent repair/stability without depending on a previous run.
The knot-sampling fixtures similarly support separate orientation and
missing-seam checks. Preserve the original coverage and validation when
partitioning a mixed test; the number of test functions need not stay the same.
Healing tests may repeat direct operations to prepare their own inputs, so they
remain independent of the topology suite.

When adding coverage:

1. Add the case and register its call in the appropriate dispatcher. Define its
   expected geometry/topology or other observable result, not just whether an
   operation returned success. Preserve negative controls and their diagnostics.
2. Run every independent test and accumulate its returned status. Preserve the
   first failure and return it from the dispatcher; later successes must not
   erase it. Use `PT_RUN(sResult, test())` from `prog_test.h` to evaluate the
   call once, retain debug diagnostics, and accumulate its status in both build
   modes. Apply it in nested groups and their callers so failures reach the suite
   runner; `SE(test())` alone discards the status. Keep
   `SER(...)` inside a case when failed setup makes subsequent operations unsafe.
3. Give the case a recognizable log message and make its fixture ownership and
   cleanup explicit. Avoid relying on retained state from another test.
4. For a new selectable suite, add its entry to `gProgSuites` in `prog_test.cpp`,
   choose its default status deliberately, and update this reference and
   [the Python registry test](../../source/SmPyLib/tests/test_smlib_tests.py),
   [the GUI fallback list](../../tools/smlib_gui/operations/test_runner.py), and
   [the GUI smoke expectation](../../tools/scripts/smlib_gui_smoke.py).
   Update [test-selection rules](../../tools/agent/suggest_tests.py) when relevant.
5. Regenerate build files if necessary, build, and run the affected suites. For
   dispatcher/order changes, also compare a default debug run with a control.

For example, each independent call in a dispatcher follows this pattern:

```cpp
SmStatus sResult = SM_SUCCESS;

PT_RUN(sResult, my_test_first());
PT_RUN(sResult, my_test_second());
return sResult;
```

`my_test_faces()` is an internal group called by `my_test_topology()`. It needs no
separate registry entry or `PT_EXPORT` because its caller is in the same shared
library. Use the exported interfaces in `prog_test.h` for external native callers.

## Fixtures, logs, and interpreting results

The CLI expects `../../TestFiles/pt_TestFiles` relative to its working directory
and checks that directory before running any selected suite. Fixtures include
native `.smb` BReps, `.smp` parts, and mesh files, with subdirectories such as
`Solids`, `Fillet`, `Shell`, `Tess`, and `Defeature`. Ensure the checkout's large
fixtures are materialized as described in the repository prerequisites. A wrong
working directory or missing fixture tree can fail a run before its first test.

The application enables long and thin logs and creates `OutputFiles/` here when
needed. Typical filenames are:

```text
OutputFiles/progTest_<month>_<day>_<hour>_<minute>_long.txt
OutputFiles/progTest_<month>_<day>_<hour>_<minute>.txt
```

Builds with `APPEND_GIT_BRANCH` add a branch suffix. An additional `_DebugLog.txt`
can be produced when the debug-log output flag is enabled by a caller; the CLI
does not enable that flag itself. The long log carries detailed kernel output;
the thin log uses condensed messages where the kernel supplies them.

Names have minute-level resolution and files are opened for writing. Archive logs
before another run with the same timestamp, and run comparisons serially in a
checkout. Suites also share process state, output settings, and relative paths.

The CLI returns zero when all selected **suite return statuses** succeed, and
nonzero for a reported suite failure or invalid invocation. It continues to later
suites after a returned failure. `PROG_TEST SUCCESS` is not a guarantee that the
logs contain no assertion, error, or validity reports: many historical tests use
assertions or `SE`, some ignore returned statuses, and some deliberately exercise
invalid inputs. `SE` diagnostics and many assertions are debug-only; release
output is not an equivalent diagnostic audit.

For a regression assessment, record the revision, configuration, selected suites,
and case identifiers. Inspect the debug logs as well as return codes. Compare
against the same entry point and configuration on the control revision. Normalize
addresses and timing, and inspect small numerical drift in context; investigate
changed test entries, geometry counts, or diagnostics. Preserve expected negative
controls. For BRep validity reports, distinguish structural defects from measured
gaps/tolerances and derived-UV problems.

The [known issues](../../KNOWN_ISSUES.md#shell_brep-can-return-a-result-that-is-not-a-solid)
include a ShellBrep result that is not a solid (`problemShell4.smb`), whose test
expects a placeholder face count. Confirm a surprising change with an isolated or
same-revision control before attributing it to a patch. Keep current defect
tracking in `KNOWN_ISSUES.md` rather than turning this README into a passing/failing
fixture ledger.

## Python and developer-tool access

The developer extension `_smlib_tests` discovers suites from the same C++ registry.
In a Python environment that can load the built extension, run this from the
repository root:

```python
from pathlib import Path
import _smlib_tests

print(_smlib_tests.list_prog_test_suites())
result = _smlib_tests.run_prog_test_suite(
    "healing",
    working_directory=str(Path("tests/prog_test").resolve()),
    do_graphics=False,
)
print(result["suite_name"], result["status_name"], result["ok"])
```

The result contains `suite_name`, numeric `status`, `status_name`, `ok`,
`elapsed_seconds`, `log`, and `draw_events`. `ok` reflects the suite return status,
with the same diagnostic limitations as the CLI. Currently `log` is always empty:
stdout/stderr capture is not implemented. This binding temporarily disables the
CLI-style file-output flags, restores working-directory/output/graphics settings
after the call, and serializes its suite calls. Use the CLI for the canonical
long/thin log comparison.

Graphics-enabled runs can collect drawing events when the corresponding kernel
callbacks are available. `debug_runtime_info()` reports loaded library paths,
compile flags, and graphics state, which helps verify the build actually in use.
The binding implementation is
[SmPyTestsMain.cpp](../../source/SmPyLib/tests_binding/SmPyTestsMain.cpp); the
[Python tests](../../source/SmPyLib/tests/test_smlib_tests.py) check discovery,
invalid suite selection, runtime information, and the structured result.
