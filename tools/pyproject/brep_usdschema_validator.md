# brep_usdschema_validator

Checks that a USD file's BrepArray prims follow the BrepArray schema (the `BA.xxx` rules), through
the NVIDIA USD asset validator. It does not check the geometry of the Breps themselves.

It needs the validation extra:

```bash
pip install "usd-brep[validation]"
```

The extra is not available on Linux aarch64, where PyPI has no `usd-core`.

## Command line

The rules register with the standard asset validator, so they run with it:

```bash
nvidia_usd_validate model.usda
```

## From Python

Run this from a script file: `usdschema_validate_file()` starts worker processes that import the main module,
so the call must sit under an `if __name__ == "__main__":` guard and not run from `python -c` or stdin.

```python
import brep_usdschema_validator


def main():
    result = brep_usdschema_validator.usdschema_validate_file("model.usda", brep_only=True)
    if not result:
        print(result.issue_summary)  # failure count by issue, e.g. {"BA.260": 1, "BA.265": 2}
        for message in result.messages:
            print(message)


if __name__ == "__main__":
    main()
```

`usdschema_validate_file()` returns a `UsdSchemaValidationResult` with `passed`, `failure_count`,
`brep_array_count`, `messages` and `issue_summary`. By default it also runs the standard asset
validator rules, whose findings are not specific to BrepArray; `brep_only=True` runs only the
BrepArray rules. To register the rules with the validator without running a file, call
`brep_usdschema_validator.register_all()`.

The rules run in the validator's OpenUSD and do not import `usd_brep`. They use the omniSolid
schema bundled in this wheel unless `OMNISOLID_PLUGIN_PATH` points to another copy.

## More

The rule catalogue and development notes are in the repository:
<https://github.com/NVIDIA-Omniverse/usd-brep/blob/main/tools/brep_usdschema_validator/README.md>
