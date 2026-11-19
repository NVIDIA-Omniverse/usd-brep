# USD BRep

Boundary representation (BRep) solid modeling, with native import and export of
BRep data to and from [OpenUSD](https://openusd.org/).

```bash
pip install usd-brep
```

```python
import usd_brep

sphere = usd_brep.primitives.create_sphere((0.0, 0.0, 0.0), 1.0)
box = usd_brep.primitives.create_box((0.0, 0.0, 0.0), 1.0, 1.0, 1.0)
cut = usd_brep.booleans.boolean_difference(sphere, box)  # a sphere missing one octant

usd_brep.usd.export_brep(cut, "cut.usda")
(brep,) = usd_brep.usd.import_breps("cut.usda")
```

Operations are grouped into submodule attributes — `primitives`, `curves`,
`surfaces`, `booleans`, `sweeps`, `fillets`, `offset`, `io`, `brep_ops`,
`stitching`, `usd` and more.

## Requirements

- CPython 3.12, 64-bit
- Windows or Linux (x86_64, aarch64)
- On Windows, the
  [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/cpp/windows/latest-supported-vc-redist)

## OpenUSD

The wheel bundles the OpenUSD runtime it was built against (25.11), so no
separate install is needed and nothing is imported into the `pxr` namespace.
Stages are exchanged as files rather than as Python objects, so no OpenUSD
object ever crosses between this package and your own.

It can share a process with a *different* OpenUSD version, which has its own
versioned symbol namespace: `usd-core` 26.8 alongside this package is tested and
works on Linux. Loading another build of **the same version** in one process is
not supported -- the namespaces coincide, so the dynamic loader may bind either
copy's symbols. `usd-core` 25.11 on PyPI is such a build: do not install it in
the same environment, because importing both crashes.

## Validation

`pip install "usd-brep[validation]"` adds
[usd-validation-nvidia](https://pypi.org/project/usd-validation-nvidia/), with PyPI
`usd-core` as its OpenUSD, and registers the BrepArray schema rules (`BrepUsdSchemaValidator`)
with it, so they run with the standard rules:

```bash
nvidia_usd_validate model.usda
```

The `validation` extra is not available on Linux aarch64, where PyPI has no `usd-core`.

The rules run in the validator's OpenUSD and do not import `usd_brep`. They use the
omniSolid schema bundled in this wheel unless `OMNISOLID_PLUGIN_PATH` points to another
copy. To run them without the plugin system, call `brep_usdschema_validator.register_all()`, or
use `brep_usdschema_validator.usdschema_validate_file()`.

## Healing

`usd-brep-heal input.usda output.usda` heals the BrepArray prims in a scene and writes the
result to a new file; it runs `usd_brep.usd.heal_file`.

## Geometry validation

`usd_brep.usd.validate_geometry_file` runs the kernel's `SmBrep::AssertValid` on the Breps
of each BrepArray prim (geometry and topology); prims that share instanced geometry are checked once. It does not check the BrepArray schema rules
that `brep_usdschema_validator` checks. An invalid Brep is returned as a result (`ok` is `False`), not raised.

```python
import usd_brep.usd

for result in usd_brep.usd.validate_geometry_file("model.usda"):
    print(result.prim_path, result.ok)
    for failure in result.failures:
        print(" ", failure)
```

`usd-brep-validate-geometry model.usda [--level N]` does the same from a shell. It prints a line
per prim and exits 0 when every Brep is valid, 1 when any is invalid and 2 when the file cannot be checked.

## License

Apache-2.0. See the LICENSE file included in the distribution.
