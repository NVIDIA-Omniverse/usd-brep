<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# brep_validator_test

Test harness for the BrepArray validator library.

## Contents

- `test_brep_validator.py`: `unittest` suite (via `utils/base_test_case.py`).
- `test_validation_boundary.py`: Regression coverage for the data-sanity/kernel
  boundary and shifted periodic face ranges. Positive cases assert an empty issue
  set; malformed packing, indices, and ranges must still produce diagnostics.
- `generate_test_files.py`: Generates requirement-specific USD fixtures under `TestFiles/brep_validator`.

## Running the suite

```bash
python -m unittest discover -s tools/brep_validator_test -p "test_*.py"
```

`utils/base_test_case.py` finds USD and the omniSolid schema in `_build/` itself, and pip-installs `usd-validation-nvidia` into the active Python if it is not importable.

## Notes

- `TestFiles/brep_validator/coverage_limits/` contains deliberately simplified
  or geometrically invalid models used by `test_validation_boundary.py`. These
  moved out of the expected-invalid corpus when BA.761 / BA.762 / BA.765 were
  retired. Passing the Python validator does **not** make these valid kernel
  models; do not use them as positive geometric-conformance fixtures.
