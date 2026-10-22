---
name: usd-brep-bug-report
description: "Verify a suspected USD BRep or SMLib bug, distinguish API misuse and known limitations, and prepare a minimal reproduction and issue draft using this repository alone. Use when reporting a bug, checking whether unexpected behavior is a bug, or reducing a failure for maintainers."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD BRep Bug Report

Establish what failed, check the API contract, and give a maintainer a case they
can run from a clean checkout. A reporter does not need to identify the kernel
root cause or propose a fix. Keep observations separate from hypotheses.

The reproduction must use this repository's own APIs, tools, and documented
dependencies. It must not require outside tools, applications, third-party
wrappers, or derivative works. The documented compiler, Python, USD, and other
build or validation dependencies are allowed.

## 1. Check known problems

- Read [KNOWN_ISSUES.md](../../../KNOWN_ISSUES.md) for the tested revision and the
  [current public version](https://github.com/NVIDIA-Omniverse/usd-brep/blob/main/KNOWN_ISSUES.md).
- Search the [public issue tracker](https://github.com/NVIDIA-Omniverse/usd-brep/issues),
  including open and closed issues, for the operation/API name, error, and symptom.
  Read likely matches; a similar symptom alone does not establish a duplicate.
- Record relevant links, affected versions, workarounds, and whether a fix is in
  the tested build. Recheck these sources for each report; do not copy a permanent
  list of known bugs into the skill. If a source is unavailable, record that gap.

## 2. Establish the expected behavior

Read [AGENTS.md](../../../AGENTS.md), the relevant guide from the
[operation index](../../operations/INDEX.md), and the API documentation or examples
for the tested revision. For a `RuntimeError`, use the
[error guide](../../docs/errors.md) to interpret the status and kernel trace.

Check argument ranges, units, model scale, tolerances, required topology, object
ownership, and mutation/consumption rules. In particular, confirm that selected
edges and faces belong to the intended body, and that the caller does not reuse
handles after their owner is consumed or replaced. Confirm the supported API and
build are actually loaded; record module/library paths when version mixing is possible.

State an expected result supported by the API contract, an analytic construction,
or a specific invariant. An external application's result alone is not proof.
An operation failure alone does not establish a bug, and an error code alone
does not establish misuse. Invalid arguments may correctly be rejected while
still exposing a separate defect, such as corruption of another body.

## 3. Locate the first incorrect result

**Validate the input immediately before the failing operation**, at a completed
modeling boundary, and retain the diagnostic output. Use both validation layers
when USD BRep assets and live SMLib objects are involved:

- **Local USD BRep asset validator:** validate the original USD input before
  importing or processing it. From the repository root on Linux, run:

  ```bash
  tools/brep_validator_cli/validate_usd.sh path/to/input.usda
  ```

  On Windows, use `tools/brep_validator_cli/validate_usd.bat`. Follow the
  [validator CLI setup instructions](../../../tools/brep_validator_cli/README.md)
  for this checkout's Python, USD, schema plugin, and validator dependencies.
  Capture the command, exit status, affected prim paths, and complete findings,
  including `BA.*` codes. Missing dependencies or undiscovered BrepArray prims
  are not a validation pass. This checks asset data and selected schema rules;
  it does not replace kernel geometry validation.

- **SMLib AssertValid:** validate every live input BRep immediately before the
  operation, including after import when applicable. With this repo's Python
  developer bindings, import `_omni_solid` before `_smlib_dev` and capture:

  ```python
  import _omni_solid as sm
  import _smlib_dev as smdev

  # input_brep is the body built or loaded by the reproduction.
  ok, reports = smdev.assert_valid_reports(input_brep, level=2, walk=True)
  print("Input AssertValid:", ok)
  for report in reports:
      print(report)
  ```

  `smdev.assert_valid(input_brep, level=2, walk=True)` is also available and
  writes failures to stderr. In native C++, call
  `pInputBrep->AssertValid(&sReports, SM_LEVEL_2, SM_WALK)` with a
  `SmAssertArray sReports`; retain the Boolean and assertion reports. Use the
  [developer topology guide](../../operations/dev_topology.md#assertvalid) for
  details. These Python bindings and direct C++ methods run in debug and release;
  do not substitute the `sm_AssertValid` convenience helper, which skips checks
  outside `SM_DEBUG_CODE`.

For an operation performed entirely on in-memory geometry, the USD asset check
is not applicable; do not add an export/import round trip just to validate it.
If a required validator is unavailable, record input validity as unverified.
If the input already fails validation, identify the earlier operation that made
it invalid or document the invalid-input handling being tested. Do not attribute
that input defect to the later call, or automatically blame the caller when an
earlier repository operation produced it.

Inspect the output immediately afterward with the same applicable validators
and a check specific to the symptom: a missing face, wrong bounds, unexpected
mutation, open edges, or geometric deviation. Observe ownership rules when
checking surviving inputs. Separate modeling, tessellation, USD conversion, and
display failures. A success status, plausible volume, manifoldness, or clean
validator report alone does not establish that the requested operation is correct.
Preserve the original failing run if added diagnostics change the behavior.

## 4. Reduce to a reproduction contained in the repo

- Use a small script calling this repo's Python bindings, an existing repo tool,
  or a focused C++ test in its existing harness. Keep the relevant public API
  call in the reproduction; do not require a kernel rewrite to report a binding bug.
- Generate geometry directly with repo APIs where possible. Otherwise supply
  the smallest necessary input fixtures alongside the reproduction, including
  referenced USD layers or other required data, and use relative paths.
- Remove external applications and derivative packages, even wrappers described
  as one-to-one bindings. A maintainer must not need another project, service,
  private file path, CAD tool, or a reporter's interactive session to run the case.
- Give exact setup/build and run commands from the repository root, using the
  documented dependencies. Keep production code unchanged; include any small
  harness patch needed to run the case. Screenshots may supplement the case,
  but cannot replace executable steps and observable output.
- Remove unrelated operations while retaining the failure and its input checks.
  Keep a concrete expected-versus-actual check suitable for a regression test.

## 5. Verify repeatability

Run the final reproduction in a fresh process using an unmodified revision,
apart from the supplied reproduction or test harness changes. Record the commit
or release, OS/architecture, build configuration, runtime/compiler versions,
relevant parameters, and complete status/trace output. If possible, compare with
current main or a known-good version and state exactly which revisions were tested.

For intermittent failures, report the number of attempts and each outcome with
fixed input and seeds. For hangs, use a bounded run and record elapsed time and
the timeout; distinguish a killed process from a returned error. Keep healing,
tolerance changes, and other workarounds as separate experiments. Do not silently
repair the input, weaken the expected check, or claim a run that was not performed.

## 6. Prepare the report

Use the [report template](assets/bug-report.md). State the current classification:
API usage problem, documented limitation, known bug, suspected new bug, or
unresolved. Explain the evidence and any remaining uncertainty. When a matching
issue exists, prepare additional evidence for it rather than a duplicate report.
An uncertain diagnosis can still be useful if its observations are reproducible.

Include the validation results from step 3, runnable files and exact commands,
expected and actual behavior, environment, repeatability, and related issues.
Do not demand a proposed implementation fix. Follow [SECURITY.md](../../../SECURITY.md)
for security reports. Prepare a draft; post or update an issue only when the user
has authorized that external action.
