<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SMLib Kernel Examples

These examples call the native C++ kernel directly. They are not additions to
the stable `SmApi*` wrappers or the Python API.

## STEP Surface Derivatives

[`step_derivatives.cpp`](step_derivatives.cpp) demonstrates the derivative-unit
correction in 0.19 for native integrations. It constructs
five surfaces, uses the returned derivatives without caller-side rescaling,
and checks them against finite differences in the input parameters. It needs
a build containing the correction; an older build fails the checks rather
than silently applying a compatibility workaround.

### Build and Run

From the repository root on Linux:

```bash
./repo.sh build -r --target smlib_step_derivatives_example
LD_LIBRARY_PATH="$PWD/_build/linux-x86_64/release:$PWD/_build/target-deps/usd/release/lib:${LD_LIBRARY_PATH:-}" \
    ./_build/linux-x86_64/release/smlib_step_derivatives_example
```

On Windows, build with
`repo.bat build -r --target smlib_step_derivatives_example` and run
`_build\windows-x86_64\release\smlib_step_derivatives_example.exe` with the
USD dependency `release\bin` directory on `PATH`. Use the matching platform
directory for other supported platforms.

The package contains the source under `Examples/SMLib` and the executable in
`bin`. To compile independently, build `step_derivatives.cpp` as C++17 with
`SM_STEP_DERIVATIVES_EXAMPLE_STANDALONE` defined, include the `SMLib` headers,
and link `SMLib` with its normal runtime dependencies.

Expected: five surface summaries followed by `STEP derivative checks passed`,
with exit status 0. A nonzero exit reports the failed call or numerical check.
For the radius-10 cylinder, the important values are:

```text
|Du| = 0.174533, |Dv| = 1
expected |Du| = 10*pi/180 = 0.174533
applying the old pi/180 workaround AGAIN would give 0.00304617 (wrong)
```

### Parameters and Units

`EvaluateSTEP` returns derivatives with respect to the parameters passed to
that call. SMLib's angular STEP surface parameters are **degrees**. This is
the kernel evaluator's convention, not a claim about the angle unit in every
STEP file or another kernel's evaluator.

| Surface | STEP U | STEP V |
|---|---|---|
| Cylinder | Angle in degrees | Axial distance in modeling units |
| Cone | Angle in degrees | Axial distance in modeling units (not slant distance) |
| Sphere | Longitude in degrees | Latitude in degrees |
| Torus | Major angle in degrees | Minor angle in degrees |
| General surface of revolution | Rotation angle in degrees | Generator curve's STEP parameter |

The example's general revolution uses a cubic B-spline generator on `[0,1]`;
its V is neither an angle nor arc length. Do not apply angular scaling just
because the surface is periodic or is a surface of revolution.

For a radius-10 cylinder, changing U by one degree moves approximately
`10*pi/180` length units along the circumference. Thus `|Du| = 10*pi/180`,
not 10. V is axial distance, so `|Dv| = 1`.

### Calling and Indexing the Evaluator

For these surface implementations, request equal U/V orders, no higher than
3. They populate the triangular set `i+j <= order`, even though storage is
rectangular. With order 2:

```cpp
SmVector3d d[3][3]; // Full rectangle; do not read the unused entries.
SmStatus status = surface->EvaluateSTEP(
    stepUV, 2, 2, TRUE, TRUE, TRUE, &d[0][0]);
if (status != SM_SUCCESS)
{
    // Report failure; do not use the outputs.
    return status;
}
const SmVector3d& point = d[0][0];
const SmVector3d& du    = d[1][0]; // Already per degree if U is angular.
const SmVector3d& dv    = d[0][1]; // Uses V's own units.
const SmVector3d& duu   = d[2][0];
const SmVector3d& duv   = d[1][1];
const SmVector3d& dvv   = d[0][2];
```

For flat storage the index is `i * (highestVDeriv + 1) + j`. In particular,
`(1,1)` requests point and first partials, **not** the mixed derivative Duv;
request `(2,2)` for Duv. Do not read, rescale, or serialize unused slots as
though they were derivative results.

### Migrating an Existing Caller

Previously, these angular evaluators accepted degrees but returned angular
partials at per-radian scale. A caller might compensate by multiplying a
first angular derivative by `pi/180`, or a derivative of angular order `n`
by `(pi/180)^n`. Remove that compensation when adopting the corrected kernel.
Do not remove conversions elsewhere merely because they contain `pi/180`.

For example, with cylinder U in degrees and V linear, Du and Duv are already
per degree, while Duu is per degree squared. For a sphere or torus, Duv is
per degree squared because both axes are angular. Linear or generator
parameters do not acquire a degree factor.

If your application deliberately differentiates with respect to **radians**,
convert its angular input to degrees before the call and convert the returned
partials back using the chain rule: multiply by `(180/pi)^n` for the number
of angular differentiations. That is a different contract from the removed
workaround. Renormalizing a tangent can hide a unit error; compare derivative
magnitudes or finite differences, not just tangent directions.

### STEP Parameters Are Not NURBS Parameters

Ordinary `SmSurface::Evaluate` / `EvaluatePoint`, the `SmApiEvaluateSurface*`
wrappers, and Python `Surface.evaluate` / `derivatives` use the ordinary
surface parameterization, not this STEP evaluator. On the analytic surfaces
here, that is the underlying NURBS parameterization. Do not pass degree-valued
STEP UVs to them or change Python callers to use degrees as part of this fix.

The example calls `ConvertUVFromSTEPToNURBS`, then checks that ordinary
`EvaluatePoint` returns the same 3D point. The parameter mapping need not be
linear and can swap axes. Matching positions does **not** mean the derivatives
are interchangeable: converting derivatives between these parameterizations
requires the mapping's chain rule, not a universal `pi/180` multiplier.

### Ownership, Scope, and Validation

The single-threaded example owns a local `SmContext` and destroys its surfaces
before the context. The revolution factory adopts the generator once it has
created the surface; the example transfers its owner at that point, before
checking the final factory status. Evaluation does not change the geometry.
No files, USD stage, tessellation, or consumer assets are needed.

The checks cover first, second, and third derivatives, including mixed
partials, at smooth interior samples on all five surface types. First
derivatives are central differences of positions; higher orders are central
differences of the preceding derivative. A separate analytic cylinder check
anchors the units. These are fixture-specific checks, not a general numerical
differentiation algorithm: they avoid seams, poles, endpoints, and out-of-domain
clamping, and do not certify behavior there.

The same source is compiled and run by
[`TestSmSTEPDerivativesExample`](../../tests/SM_API_test/src/TestSmSTEPDerivativesExample.cpp)
alongside the existing `TestSmSTEPDerivativeUnits` regression:

```bash
./repo.sh build -r --target SM_API_test_app
./tests/SM_API_test/SM_API_test.sh linux-x86_64
```

The suite must report success for both tests. Source contracts live in
[`SmSurface.h`](../../source/SMLib/inc/SmSurface.h) and the relevant analytic
surface headers; this example adds no evaluator or API behavior.
