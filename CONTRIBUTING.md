<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Contributing

USD BRep is currently not accepting external code contributions. Issues and
bug reports are welcome through the project issue tracker. Security issues
should be reported using `SECURITY.md`, not public issues.

This guide records the development expectations for maintainers and approved
contributors.

## Getting Started

Install the prerequisites listed in `README.md`, then build and test from the
repository root:

```bash
./repo.sh build
./repo.sh test
```

On Windows, use the equivalent batch wrapper:

```bat
repo.bat build
repo.bat test
```

The repo tooling fetches USD, Python, Premake, and other build dependencies via
Packman. A host C++ toolchain is still required.

## Before Submitting Changes

Run the formatter used by CI:

```bash
./repo.sh format
```

For code changes, run the relevant focused tests first, then run the full suite
when the change affects shared behavior:

```bash
./repo.sh test
```

If you change Python bindings, include tests under `source/SmPyLib/tests/`.
If you change C++ kernel, SM_API, or USD bridge behavior, include focused
coverage near the affected layer where practical.

## Code Style

- C++ uses the repository `.clang-format` configuration.
- Python follows PEP 8 with the repository 120-column line limit.
- Keep changes scoped. Avoid unrelated refactors, generated-file churn, and
  formatting-only edits mixed into behavioral changes.
- Prefer existing helper APIs and local patterns over new abstractions.

## Operations And Bindings

SMLib operations span the C++ kernel, the C-style C++ `SM_API`, the USD bridge,
and the `_omni_solid` Python bindings. When adding or changing an operation:

1. Keep the implementation in the appropriate layer (`source/SMLib/`,
   `source/SM_API/`, `source/SM_API_USD/`, or `source/SmPyLib/`).
2. Preserve the documented mutation semantics for Python bindings.
3. Add or update focused tests for the affected API surface.
4. Update the relevant operation guide under `.agents/operations/`.
5. Update public documentation or type stubs when user-visible behavior changes.

See `.agents/operations/INDEX.md` and `AGENTS.md` for the operation map,
layering notes, and binding conventions.

## Documentation

Keep docs in sync with behavior. When adding or changing an operation, update
the relevant operation guide and any public documentation that describes its
arguments, defaults, mutation semantics, or safety constraints.

`KNOWN_ISSUES.md` lists known bugs that users can hit. When a change fixes one,
remove its entry in the same change.

## Community Standards

Follow `CODE_OF_CONDUCT.md` in all project interactions. Report vulnerabilities
according to `SECURITY.md`.
