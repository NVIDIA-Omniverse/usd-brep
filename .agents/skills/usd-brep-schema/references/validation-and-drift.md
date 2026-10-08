<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# BrepArray Validation and Drift

## Contents

1. [Validation model](#validation-model)
2. [Preflight](#preflight)
3. [Compare schema layers](#compare-schema-layers)
4. [Validate an asset](#validate-an-asset)
5. [Classify findings](#classify-findings)
6. [Audit validator rules](#audit-validator-rules)
7. [Known audit hotspots](#known-audit-hotspots)
8. [Develop tests](#develop-tests)
9. [Report results](#report-results)

## Validation Model

Keep these gates separate:

1. **Layer syntax**: USD can parse the layer.
2. **Schema registration**: the plugin recognizes `BrepArray` and its applied APIs.
3. **Authored structural contract**: property types, allowed tokens, counts, packing, and indices follow `schema.usda`.
4. **Geometry/topology contract**: equations, endpoint agreement, loop/radial semantics, and watertight representation follow explicit or derived rules.
5. **Implementation policy**: the asset satisfies additional consumer or validator restrictions.
6. **Round-trip behavior**: a selected reader/writer preserves intended data.

Passing one gate does not imply the next. Report each gate exercised.

## Preflight

Before judging an asset or implementation:

```bash
git status --short --branch
git rev-parse HEAD
git log -1 --format='%H %cs %s' -- source/schema/omniSolid/resources/schema.usda
```

Confirm `source/schema/omniSolid/resources/schema.usda` exists at the checked-out revision. If network freshness is required, fetch only after checking local work.

In `solidmodeling`, the validator wrappers expect built USD and OmniSolid dependencies. `tools/brep_validator_cli/validate_usd.sh` configures the usual repository paths. A missing plugin can make a real BrepArray appear untyped or undiscoverable.

## Compare Schema Layers

Use the bundled comparison before trusting a generated, installed, or user-supplied candidate copy:

```bash
python3 .agents/skills/usd-brep-schema/scripts/compare_schema_layers.py \
  source/schema/omniSolid/resources/schema.usda \
  path/to/candidate-schema.usda
```

The script opens both layers with `pxr.Sdf` and compares normalized exports. It intentionally ignores comments and formatting but includes declarations, property types, variability, metadata, allowed tokens, custom data, and documentation. A documentation-only semantic change is therefore visible.

If `pxr` is unavailable, build/setup the repository USD environment or pass `--usd-python-path <usd-lib-python>`.

Do not compare only generated `plugInfo.json`: it cannot capture all property documentation or semantic equations.

## Validate an Asset

Run the exit-code-bearing wrapper:

```bash
tools/brep_validator_cli/validate_usd.sh path/to/asset.usda
```

Interpret outcomes carefully:

- Exit 0 with at least one discovered BrepArray and no messages means the current validator found no issue in its implemented coverage. It is not proof of complete schema compliance.
- Exit 1 means one or more asset-validator or BrepValidator findings. Reclassify them against the schema.
- "No BrepArray prims" is inconclusive. Diagnose plugin registration, population masks, inactive prims, variants, payloads, and actual type names.
- A crash or dependency error is an environment/coverage failure, not an asset failure.

Use `tools/brep_validator_cli/brep_array_handler.py` for interactive diagnostics if useful, but do not use it as a pass/fail gate: it prints issues and does not provide the same reliable nonzero result contract.

For authored type checks, inspect `Sdf.Layer` prim/property specs. A registered schema can supply the expected type through composition, making a malformed authored property appear correctly typed through `UsdAttribute.GetTypeName()`.

For every BrepArray prim:

1. Inventory `apiSchemas`, topology arrays, and geometry arrays.
2. Compute count/prefix-sum partitions from the authoritative contract.
3. Check type-token occurrence mapping for every geometry API and instance.
4. Validate index ranges inside the owning Brep partition.
5. Check NURBS slice lengths, order, knots, weights, and surface layout.
6. Check analytic frames, radii, units, equations, and range-to-vertex agreement.
7. Check explicit loop, faceuse, and radial semantics.
8. Record schema gaps rather than substituting kernel assumptions.

## Classify Findings

Use this evidence test:

| Classification | Evidence required |
|---|---|
| Schema violation | Direct declaration, allowed token, API metadata, or documentation from the current authoritative `schema.usda` |
| Derived invariant violation | A short derivation from named explicit clauses |
| Implementation-policy finding | Owning consumer/validator policy with no claim that schema requires it |
| Validator drift | Minimal example plus schema clause showing the validator contradicts or misses the contract |
| Underspecified | Explanation of which schema rule or approved design decision is absent |

Do not cite a `BA.xxx` description as schema evidence. The requirement enum is part of the validator implementation.

## Audit Validator Rules

The validator source is `tools/brep_validator/brep_validator.py`. Its current README is useful orientation but can lag the code; the implementation currently contains rules beyond the README's older BA.000-BA.555 summary.

For each audited rule:

1. Read the enum description.
2. Read every call site and helper used to enforce it.
3. Identify exact authored/composed data read by the check.
4. Map the check to `schema.usda` or mark it derived/policy.
5. Compare boundary operators exactly: `>`, `>=`, `<`, `<=`, tolerance offsets, and accepted tokens.
6. Determine whether the check uses per-Brep `brep:intersectTol3d` or an unrelated constant.
7. Determine whether malformed prerequisite data causes a false pass, false cascade, or exception.
8. Add a focused fixture whose only intended distinction is the audited rule.
9. Assert both the expected issue code for invalid data and an empty issue set for schema-valid data.

Audit readers/writers similarly: compare field names, types, occurrence order, prefix-sum slicing, units, and applied API tokens in both directions.

## Known Audit Hotspots

These are observations against the 2026-07-14 source baseline. Recheck current code before reporting them as current.

| Area | Schema contract | Implementation behavior to audit |
|---|---|---|
| Cone semi-angle (`BA.515`) | `(-pi/2, pi/2)`; zero explicitly degenerates to a cylinder | Validator description and check require `(0, pi/2)`, rejecting schema-permitted zero and negative values |
| NURBS order (`BA.590`, `BA.651`) | Positive order and order `<= vertexCount` | Validator adds minimum order 2 |
| UV NURBS zero sentinel (`BA.380`, `BA.590`) | Every authored UV NURBS order is positive and bounded by `vertexCount` | Validator special-cases `order == 0` and `vertexCount == 0` as "no UV trim curve," accepting values the schema does not define |
| Authorship (`BA.005`, `BA.070`, etc.) | Properties are declared, but the schema does not mark all as required on an empty typed prim | Validator requires broad authorship; treat this as policy unless authoritative schema documentation requires the data |
| Per-Brep versus prim extent (`BA.040`-`BA.050`) | `brep:extent` stores one double-precision range per Brep; inherited `extent` is the standard float-precision range for the whole prim. The schema does not explicitly relate them. | `UsdBrepWrite.cpp` authors `extent` as the union of individual BRep boxes, and the validator requires every `brep:extent` to be contained by it. Treat that relationship as implementation policy. |
| Positive values | Schema says strictly positive | Several checks compare against fixed `NUMERICAL_TOLERANCE`, rejecting small positive values; decide whether this is numerical policy |
| Angular domains (`BA.560`-`BA.571`, `BA.630`-`BA.631`, `BA.762`, `BA.765`) | Equations use radians; no primary-period or alignment rule is stated | Validator adds one-period, primary-range, and full-period alignment policies |
| Control-point/origin containment (`BA.365`, `BA.465`, `BA.620`, `BA.750`) | Extent and parameter domains are defined, but these specific containment heuristics are not | Validator may reject valid control hulls, analytic centers, or UV control points outside evaluated bounds |
| Minimum topology (`BA.700`-`BA.702`) | Complete BRep prose implies topology, but an empty prim is schema-registerable and requiredness is not formalized | Validator enforces non-empty region/shell content |
| GeomSubset policy (`BA.680`-`BA.682`) | Not declared by BrepArray-specific schema properties | Validator adds inherited/USD consumer policy |
| Type validation | Raw authored type is a schema concern | Registered schema composition can mask a bad raw declaration unless `Sdf` specs are inspected |

The current `tools/brep_validator_test/test_brep_validator.py` expected-valid loop asserts only that the result is a list, not that it is empty. Do not cite it as evidence that positive fixtures pass cleanly until that assertion is strengthened.

## Develop Tests

Create minimal fixtures under `TestFiles/brep_validator/` when changing validator behavior. Keep one intended contract distinction per fixture.

For an explicit schema rule, cover:

- Lowest and highest allowed boundary values.
- One value immediately outside each boundary.
- Correct and incorrect raw USD property types.
- Missing/applied API combinations where required for interpretation.
- Single-Brep and multi-Brep packing when partition logic is involved.
- Both edge and wire-edge multiple-apply instances for curve geometry.

For drift fixes, pair fixtures:

- `schema-valid`: demonstrates the validator must accept data permitted by the schema.
- `schema-invalid`: demonstrates the validator must reject data prohibited by the schema.

Do not name a fixture "valid" if the test only verifies that validation did not crash. Assert the issue list is empty, or explicitly scope which unrelated issues are tolerated and why.

Build the repository first so `_build/target-deps` contains USD and OmniSolid, then run the focused
validator tests.

Windows PowerShell:

```powershell
.\build.bat
python -m unittest tools.brep_validator_test.test_brep_validator -v
```

Linux/WSL:

```bash
./build.sh
python3 -m unittest tools.brep_validator_test.test_brep_validator -v
```

Use the repository's configured Python/USD environment when the system interpreter cannot import `pxr` or `usd_validation_nvidia`.

For authoritative schema changes, use the platform build above and add semantic fixture coverage;
schema registration alone is insufficient.

## Report Results

Use a compact structure:

```text
Authoritative schema: <commit and path>
Asset/implementation: <identifier and prim paths>
Checks run: <schema comparison, validator, manual clauses, tests>

Schema violations:
- <property/index and source clause>

Derived invariant violations:
- <finding and derivation>

Implementation-policy findings:
- <finding and owning policy>

Validator drift:
- <rule, schema contradiction, reproducer>

Inconclusive areas:
- <missing schema rule, design decision, environment, or coverage>
```

State "no violations found in the checked contract" rather than unqualified "fully compliant" when an authoritative schema rule or required geometric proof is unavailable.
