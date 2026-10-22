<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bug report template

Replace the prompts with observed facts. Mark a check as not applicable or not
run, with a reason, when appropriate. Keep hypotheses labeled as hypotheses.

## Summary and classification

Name the operation, triggering condition, and observed failure. State whether
this is a suspected new bug, known bug, API usage problem, documented limitation,
or unresolved, and why.

## Expected behavior

Describe the expected result and its basis in the API documentation, analytic
geometry, or a specific invariant. Include the parameter and ownership checks
used to rule out misuse.

## Actual behavior

Include the exact status, exception, kernel trace, crash, timeout, or incorrect
result. State the first operation whose output differs from the expectation.

## Environment

- Repository commit or official release; reproduction/harness changes, if any:
- OS and architecture; debug/release build; relevant compiler/runtime versions:
- Loaded module/library paths; relevant units, tolerances, settings, and seeds:
- Other revisions tested and their outcomes:

## Reproduction

Supply the minimal script or test and all required fixtures. Give exact setup,
build, and run commands from the repository root. Use only this repo's APIs,
tools, and documented dependencies; no outside applications or derivative works.
Include the expected-versus-actual check and its output.

## Validation before and after the operation

- Original USD input: local asset-validator command, exit status, prim paths,
  and findings, including `BA.*` codes; or why no USD input is involved.
- Each live input BRep immediately before the call: SMLib `AssertValid` result,
  level/walk settings, and complete assertion reports.
- Output and surviving inputs: applicable validator results and the check that
  demonstrates this specific defect. If the operation crashes, say so.
- Any validation not run, unavailable dependencies, or effect of instrumentation:

## Repeatability and related reports

Record fresh-process attempts and outcomes, elapsed time/timeouts where relevant,
and any separate workaround experiments. Link relevant `KNOWN_ISSUES.md` entries
and open or closed issues; explain whether this adds evidence to an existing
report. State remaining uncertainty without presenting a proposed cause as fact.
