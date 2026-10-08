---
name: usd-brep-schema
description: "Use for questions, asset compliance checks, validator drift audits, and code or tests involving the proposed OmniSolid USD BrepArray schema."
---

<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# USD BrepArray Schema

## Purpose

Use the authoritative `source/schema/omniSolid/resources/schema.usda` schema to reason about BrepArray. Keep schema-defined correctness separate from implementation policy.

Use this skill to answer schema questions with source references, verify authored USD assets, audit schema and validator drift, and develop BrepArray readers, writers, validators, tests, and translators. Its scope is the proposed OmniSolid USD representation; runtime SMLib BRep rules must be established separately.

## Workflow

Use this sequence for every task, then continue with the matching question, asset, drift, or development section.

1. Work from the `solidmodeling` repository root and record its commit and working tree state. Do not pull or switch branches over local work.
2. Read `source/schema/omniSolid/resources/schema.usda` as the normative source.
3. Treat design documents and comments outside that layer only as rationale. If they conflict with `schema.usda`, follow `schema.usda` and report the conflict.
4. Treat generated schema metadata, serializers, utilities, validators, examples, and tests as implementations that may drift.
5. The schema docstring still mentions a "Solid Models USD Proposal" and future rule migration, but that statement may be stale. Do not treat it as evidence that another current normative source exists. If the authoritative layer does not decide a question, mark it underspecified and flag the docstring for clarification.

Classify every claimed rule:

- **Explicit schema requirement**: stated by a declaration, `allowedTokens`, API metadata, or documentation in `schema.usda`.
- **Derived invariant**: follows necessarily from explicit sizes, ordering, indexing, or equations. Show the derivation.
- **Implementation policy**: useful restriction enforced by code but not required by the schema.
- **Schema/implementation contradiction**: implementation accepts what the schema forbids or rejects what it permits.
- **Underspecified**: the available schema does not decide the question.

Read [references/schema-contract.md](references/schema-contract.md) before answering detailed schema questions, judging an asset, or writing BrepArray code.

## Requirements

- Read access to the `solidmodeling` checkout containing the authoritative schema and its consumers.
- Python 3 and `pxr.Sdf` to run the bundled schema comparison script. Pass `--usd-python-path` when `pxr` is not on the ambient Python path.
- Built `solidmodeling` USD and validator dependencies only when running the validator CLI; source-only schema questions do not require a build.

## Answer Schema Questions

1. Identify the relevant prim, applied API, topology stratum, or geometry type.
2. Trace the answer to `schema.usda`; use the contract reference as an index, not as a replacement for checking current source.
3. State whether the answer is explicit, derived, or underspecified.
4. Include exact property/token names and packed-array occurrence rules where they affect the answer.
5. Distinguish USD registration or type recognition from semantic validity.
6. Call out version/commit sensitivity when the authoritative source differs from the skill baseline.

Do not use the validator's `BA.xxx` text as evidence that a rule is in the schema. First map it back to a schema clause.

## Verify an Asset

Read [references/validation-and-drift.md](references/validation-and-drift.md), then use this order:

1. Confirm the layer opens and identify every authored `BrepArray` prim, including inactive or otherwise non-default prims when relevant.
2. Confirm the OmniSolid plugin is registered and the intended applied API schemas are recognized. Treat "no BrepArray prims found" as inconclusive, never as a pass.
3. Inspect raw `Sdf` property specs when authored types or authorship matter; composed `UsdAttribute` views can inherit schema types and hide a bad raw declaration.
4. Run `tools/brep_validator_cli/validate_usd.sh <asset>` from `solidmodeling` when its built dependencies are available.
5. Reclassify every validator issue against the authoritative schema: schema violation, derived-invariant violation, policy finding, contradiction, or underspecified.
6. Manually cover explicit schema clauses not exercised by the validator.
7. Report the exact scope checked. Do not claim full compliance from parsing, schema recognition, a zero-exit validator run, or round-trip success alone.

Use these result labels:

- `schema violation`
- `derived invariant violation`
- `implementation-policy finding`
- `validator drift`
- `inconclusive: schema underspecified`
- `inconclusive: environment or coverage gap`

## Audit Schema Drift

Compare the authoritative source with a generated, installed, or user-supplied candidate schema when one is relevant. The bundled utility uses `pxr.Sdf` normalization, so comments and formatting do not create false differences while declarations, metadata, allowed tokens, and documentation remain significant.

```bash
python3 .agents/skills/usd-brep-schema/scripts/compare_schema_layers.py \
  source/schema/omniSolid/resources/schema.usda \
  path/to/candidate-schema.usda
```

For validator drift:

1. Start at `BrepArrayRequirements` and the corresponding `_validate_*` implementation in `tools/brep_validator/brep_validator.py`.
2. Cite the authoritative schema clause that supports the check.
3. If multiple clauses imply the check, write the derivation.
4. If no clause supports it, label it policy rather than compliance.
5. If it contradicts the schema, create a minimal schema-permitted fixture that exposes the rejection, or a schema-forbidden fixture that exposes acceptance.
6. Test boundary values named by the schema, not values copied from the implementation.
7. Keep policy diagnostics useful, but do not present them as schema violations.

Never change `schema.usda` merely to rationalize existing validator behavior. Make changes to the authoritative file only with an explicit design decision.

## Available Scripts

Invoke repository scripts directly from the shell; do not assume a provider-specific script runner.

| Script | Purpose | Arguments |
|---|---|---|
| `.agents/skills/usd-brep-schema/scripts/compare_schema_layers.py` | Compare authoritative and candidate USDA schema layers after `pxr.Sdf` normalization; return `0` for a match, `1` for a difference, and `2` for an input or environment error. | `<canonical> <candidate> [--usd-python-path PATH] [--quiet]` |

When an agent host provides a `run_script()` helper, the schema-drift validation step can use this
host-adapter pseudocode. Adapt the call signature to the host; the direct shell invocation above
remains authoritative.

```python
run_script(
    script=".agents/skills/usd-brep-schema/scripts/compare_schema_layers.py",
    args=[
        "source/schema/omniSolid/resources/schema.usda",
        "path/to/candidate-schema.usda",
    ],
)
```

## Develop Code and Tests

Choose the owning layer before editing:

| Work | Start here |
|---|---|
| Authoritative schema or API metadata | `source/schema/omniSolid/resources/schema.usda` |
| Schema packaging/registration | `source/schema/omniSolid/` |
| Packed data read/write | `source/BREP_USD_DATA/` and `$usd-brep-data-author` |
| Kernel-to-USD conversion | `source/BREP_SM_USD/` |
| Staged authoring | `source/BREP_STAGING/` |
| Schema-level iterators/evaluation/topology | `source/USD_BREP_UTILS/` |
| Compliance validator | `tools/brep_validator/` |
| Validator fixtures/tests | `TestFiles/brep_validator/`, `tools/brep_validator_test/` |

For each behavior change:

1. Name the schema clause and classification in the design or test rationale.
2. Preserve the packed occurrence mapping between type-token arrays and geometry arrays.
3. Add a positive boundary test for allowed data and a negative test for each explicit prohibition.
4. Add a drift test when validator or consumer behavior previously disagreed with the schema.
5. Test raw authoring, composed reading, and round-trip behavior separately when each is in scope.
6. Use per-Brep `brep:intersectTol3d` for topology/geometry agreement where the schema assigns it meaning; document any independent numerical epsilon as implementation policy.

## Example Requests

```text
Use $usd-brep-schema to explain whether one edge may have more than two edge uses and cite the governing schema properties.
Use $usd-brep-schema to check this USDA fixture and separate schema violations from validator policy findings.
Use $usd-brep-schema to determine whether BA.590 is supported by schema.usda or represents validator drift.
Use $usd-brep-schema to design positive, negative, and round-trip tests for a BrepArray NURBS writer.
```

## Limitations

- A compliance result is inconclusive when the authoritative schema revision, required USD plugin, or relevant authored data cannot be inspected.
- The schema cannot decide behavior it leaves underspecified; do not replace missing requirements with validator behavior or examples.
- The bundled comparison script detects schema-layer drift, not semantic coverage gaps in validators, readers, writers, or translators.
- This skill does not define runtime SMLib topology or geometry validity; establish that contract separately and state any normalization needed between representations.

## Guardrails

- Do not treat example assets as compliant fixtures without checking them against the authoritative schema.
- Do not treat schema registration or API application tests as semantic validation of BrepArray data.
- Do not treat a validator test as a positive conformance test unless it asserts that the issue set is empty.
- Do not equate generated schema data, `plugInfo.json`, or C++ tokens with the authoritative source without comparing them.
- Do not silently normalize radians to degrees, pre-weight NURBS control points, reorder ellipse radii, or add hidden line/plane scale factors.
- Do not collapse policy warnings into schema errors in user-facing reports.

## Troubleshooting

| Symptom | Likely cause | Resolution |
|---|---|---|
| `Could not import pxr.Sdf` | The USD Python package is not on the active Python path. | Pass `--usd-python-path <USD_ROOT>/lib/python` or enter the repository USD environment. |
| The validator reports no BrepArray prims | The plugin is not registered, the traversal omitted relevant prims, or the asset contains none. | Verify plugin registration and inspect raw `Sdf` prim specs; report the result as inconclusive until discovery is complete. |
| Schema comparison reports a diff | The checkouts differ semantically or represent different revisions. | Record both commits, inspect the normalized unified diff, and classify the drift before changing either schema copy. |
| Validator success conflicts with manual review | The validator has a coverage gap or reads a composed view that hides raw authorship. | Cite the uncovered schema clause, add a minimal fixture, and report validator drift separately from asset compliance. |

## Validation

For skill-only edits, run:

```bash
python3 .agents/skills/usd-brep-schema/scripts/compare_schema_layers.py \
  source/schema/omniSolid/resources/schema.usda \
  source/schema/omniSolid/resources/schema.usda
git diff --check
```

Then exercise at least one question, one asset-check scenario, and one validator/code drift scenario against current source.
