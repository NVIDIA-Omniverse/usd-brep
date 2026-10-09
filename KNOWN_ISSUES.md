<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Known issues

Known bugs in USD BRep.

## Modeling

### `remove_fillet` removes nothing and reports success

`remove_fillet` (`SmApiRemoveFillet`) returns success without removing the fillet.

**Workaround:** none. Check the result after the call, for example its face count or `volume()`.

### `heal_brep` can report success when a repair failed

`heal_brep` (`SmApiHealBrep`) does not report failures from its repair stages, including seam insertion.

**Workaround:** check the healed Brep, for example with `is_manifold_solid()`, or with `bin/brep_geometry_validator` after exporting it to USD.

### Healing closed surfaces with missing or coincident seams

`heal_brep`, and the healing run on import, can mis-repair faces on periodic and closed surfaces such as tori, spheres and cylinders. These faces mostly come from other CAD systems.

- A missing seam on a torus face is not detected when the face boundary lies partly along the other seam, or when the face has only interior holes.
- Inserting the first seam on an uncut torus splits it into two faces.
- A sphere bounded by two coincident seam edges is treated as collapsed and deleted.
- A cylindrical face with a pocket can be left with two outer loops after its seam is repaired.
- A torus whose one seam edge is used by two loops is treated as collapsed.
- A face with a missing seam can be skipped by seam insertion, because its loops are classified too early.
- Rebuilding a closed face inside a hollow solid can merge the cavity into the solid.
- A boundary edge that passes through a surface pole is not split at the pole.
- An edge-to-face gap near a seam can be measured on the wrong side of the surface, so healing tightens the edge tolerance too far.
- A face in which one degenerate edge is used by two of its loops is left with invalid topology.

**Workaround:** none. Check healed Breps as described above.

### `drop_curve_to_surface` near a surface pole

Near a surface pole, for example the apex of a cone, dropping a curve onto a surface can miss a seam that the curve starts on or runs along within tolerance, and so classify the curve's boundary incorrectly.

## Tessellation

### Inside-out triangles next to fillets

On solids filleted with `circular_fillet`, `tessellate()` can wind some triangles against their face's normal, so they render inside out. Seen on cylinder caps and on cones next to fillets.

**Workaround:** a `surface_angle_tolerance_deg` of 15° avoided every case found.

## Measurement

### Area, volume and mass properties of faces bounded by circles

Face area, Brep area and volume, and mass properties (`Face.area()`, `Brep.area()`, `Brep.volume()` and the matching `SmApi*` functions) can be inaccurate for faces whose trim curves have interior knots, such as rational circles. Measured relative area errors were up to about 1.4% at the default `relative_accuracy` of 1e-3, and up to 20% at 1e-1.

**Workaround:** use a smaller `relative_accuracy`. The error fell as the accuracy was tightened in every measured case.

## USD import and validation

### Malformed BrepArray topology can crash import

Reading a BrepArray whose topology counts or indices disagree with its array sizes can read out of bounds and crash the process. Such files fail BrepArray validation.

**Workaround:** validate files before importing them, for example with `brep_validator_cli/validate_usd.sh --brep-only` or `nvidia_usd_validate`.

### BrepArrays with UV trim curves from other exporters

Some BrepArrays written by other exporters, such as OCCT-based UsdSolid exporters, carry UV trim curves that import does not convert correctly. Their faces can tessellate incorrectly, or the Brep can be skipped. BrepArrays written by USD BRep are not affected.

### An orphan edge makes the validator report unrelated failures

When a BrepArray contains an edge that no edgeuse refers to (BA.582), the BrepArray validator can no longer tell which edges and vertices belong to which Brep. It then reports unrelated failures, such as BA.175, BA.205, BA.225, BA.265, BA.310, BA.320, BA.365 and BA.465, for later Breps.

**Workaround:** fix the BA.582 failure first, then validate again.

### The validator scripts in the release package expect a source checkout

`brep_validator_cli/validate_usd.sh` and `brep_validator_cli/validate_directory.sh` look for a source build (`_build/` and `tools/packman/`). Run from the release package, `validate_usd.sh` exits with code 127, and `validate_directory.sh` exits with code 1 and no output.

**Workaround:** run `python brep_validator_cli/validate_usd.py <file>` with a Python 3.12 that can import `pxr` and `usd_validation_nvidia`, or install `usd-brep[validation]` and run `nvidia_usd_validate`.
