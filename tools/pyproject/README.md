# USD BRep

Boundary representation (BRep) solid modeling, with native import and export of
BRep data to and from [OpenUSD](https://openusd.org/).

```bash
pip install usd-brep
```

```python
import usd_brep

sphere = usd_brep.primitives.create_sphere((0.0, 0.0, 0.0), 1.0)
box = usd_brep.primitives.create_box((-0.5, -0.5, -0.5), 1.0, 1.0, 1.0)
cut = usd_brep.booleans.subtract(sphere, box)

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
`usd-core` as its OpenUSD, and registers the BrepArray schema rules (`BrepValidator`)
with it, so they run with the standard rules:

```bash
nvidia_usd_validate model.usda
```

The `validation` extra is not available on Linux aarch64, where PyPI has no `usd-core`.

The rules run in the validator's OpenUSD and do not import `usd_brep`. They use the
omniSolid schema bundled in this wheel unless `OMNISOLID_PLUGIN_PATH` points to another
copy. To run them without the plugin system, call `brep_validator.register_all()`, or
use `brep_validator.validate_file()`.

## License

Apache-2.0. See the LICENSE file included in the distribution.
