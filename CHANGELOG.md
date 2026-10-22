# Changelog

## [1.0.1] - 2026-10-21
### Added
- `SmApiUsdGeometryValidate.h`: run `SmBrep::AssertValid` on one BrepArray prim without changing the stage; `brep_geometry_validator` uses it.
- `SmApiUsdHeal.h`: find, heal and write BrepArray prims on an open USD stage (`SmApiUsdFindBrepArraySites`, `SmApiUsdHealBrepArray`, `SmApiUsdWriteHealedBrepArray`). `usd.heal_file` is built on them. Healing is best effort: a failed heal step or an unfixed problem is not reported.

### Changed
- `tessellate()` is faster; its output is unchanged.
- `volume`, `mass_properties`, `face_area`, `Face.area` and `Face.centroid` (`SmApiBrepComputeVolume`, `SmApiBrepComputeMassProperties`, `SmApiFaceComputeArea`, `SmApiFaceComputeCentroid`) no longer clamp `relative_accuracy`, which was limited to [1e-8, 1e-1] for `volume` and [1e-4, 1e-1] for the others. A non-finite accuracy fails with `SM_ERR_INVALID_INPUT`.
- `usd.heal_file` rejects BRep geometry composed from several prim specs in one layer, such as an internal reference that overrides the geometry it references. It used to keep the override over the healed geometry.
- Linux aarch64 builds no longer fuse multiply-adds (`-ffp-contract=off`), which made their results differ from x86-64 and Windows: a boolean on marginal input could give a different result on aarch64, including a result with no solid material. ([GitHub #12](https://github.com/NVIDIA-Omniverse/usd-brep/issues/12))
- `create_cylinder` and `SMU_BrepConvert::CreateSmBrep_FromUsdCylinder` build their side face on an `SmCylinder` instead of a cone with semi-angle 0, so USD export writes `BrepSurfaceCylinderAPI`. The geometry is unchanged. ([GitHub #14](https://github.com/NVIDIA-Omniverse/usd-brep/issues/14))
- `brep_geometry_validator` no longer heals BReps on import, and its `--no-healer` option is removed. Healing could repair the defects being checked for, and the healer can crash on invalid BReps.

### Fixed
#### Booleans
- A union of a part with its mirrored copy returned the first operand, and their difference failed, because flat NURBS planes were not recognised as planar. Fillets on NURBS boxes are also more accurate. ([GitHub #1](https://github.com/NVIDIA-Omniverse/usd-brep/issues/1))
- Booleans that combined two overlapping intersection edges lost some edge definitions and later read the deleted edge.
- Booleans of three or more equal-radius cylinders whose axes cross at one point, such as a tricylinder, failed with `SM_ERR` or returned a solid missing a whole arm.
- Booleans wrote to freed memory while cleaning up.
- A Boolean union whose edge overlap lay on two coincident face pairs could crash or give different results from run to run.
- Booleans that glued coincident vertices freed curves still in use, which crashed fillet removal on an imported part.
- A Boolean that deleted a face with a one-vertex loop crashed, for example an intersection removing a sphere cut that touches a flat face.
- Booleans that glued coincident vertices kept a deleted vertex in an internal map; no test reaches this.
- Booleans between a cone and an `SmCylinder`, such as two crossing cylinders from `linear_sweep`, used general NURBS intersection instead of the exact cone intersector and could lose a whole cylinder.
- Booleans and merges could give an edge a tolerance far larger than its real gap. When dropping a curve onto a surface took a second attempt, the drop distance from the abandoned first attempt was reported with the curve the second attempt made, and the edge tolerance was raised to twice that. On a ring cut by a wrapped cutter this gave an edge a tolerance of 2.4 in a DIFFERENCE result, and with slightly different rounding, a merge tolerance of 16 that made the next DIFFERENCE return no solid. ([GitHub #12](https://github.com/NVIDIA-Omniverse/usd-brep/issues/12))
- Booleans could crash after closing a gap in an intersection loop deleted a vertex: the vertex stayed on the list cleaned up at the end of the Boolean. `shell_brep` crashed this way when the wall was as thick as a box with an open top face, for example a 20 x 30 x 5 box at distance -5.
- Booleans could leave an edge whose 3D and UV curves ended away from its vertices, for example a thin curved panel cut by an elliptic cylinder across the panel boundary (4.7e-3 off). A split point is now accepted only when it lies within the tolerance of the edge curve.

#### Sweeps and extrusions
- `linear_sweep` (`SmApiCreateLinearSweep`) could return an open, zero-volume body, or hang, for a non-unit direction, a circle swept just off its axis or a negative distance. The direction is now normalised, a negative distance sweeps against it, and a zero direction fails with `SM_ERR_INVALID_INPUT`. ([GitHub #3](https://github.com/NVIDIA-Omniverse/usd-brep/issues/3))
- `draft_sweep` (`SmApiCreateDraftSweep`) crashed on a profile with a repeated edge.
- `taper_extrude` (`SmApiCreateTaperExtrude`) crashed when an open chain of curves ended where a closed curve starts.
- `linear_sweep_with_repetitions` (`SmApiCreateLinearSweepWithRepetitions`) failed with `SM_ERR` when `cap_ends=True` (the default) and `repetitions` was more than 1, and swept only the first profile face.
- `create_skin_from_faces` (`SmApiCreateSkinFromFaces`) aborted the process with a stack overflow (`*** stack smashing detected ***`) when asked for a loft of degree 18 or more through 19 or more faces. It now raises `RuntimeError` (`SM_ERR_INVALID_INPUT`) for any degree outside 1 to 17, however many faces are given; degree 0 used to fail with `SM_ERR`.

#### Fillets
- `fillet_edges`, `fillet_edges_per_edge`, `variable_radius_fillet`, `fillet_preview` and `set_bevel_corners` given an edge of another Brep modified that Brep before failing. They now reject foreign, null or stale edges with `SM_ERR_INVALID_INPUT`. ([GitHub #2](https://github.com/NVIDIA-Omniverse/usd-brep/issues/2))
- `SmVertex::GetVertexuseOfFace` and `SmVertex::GetLoopOfFace` ignored their face argument and returned the Vertexuse or Loop of whichever face came first; the face test had been commented out with the composite-face code. As a result `SmBrep::DeleteFilletFace` failed for every fillet, including one filleted edge of a box.
- `remove_fillet` (`SmApiRemoveFillet`) returned success when it removed nothing. It now returns an error (Python: `RuntimeError`) when any fillet face could not be removed; removed fillets stay removed. Fillets that meet at a rounded corner ([GitHub #6](https://github.com/NVIDIA-Omniverse/usd-brep/issues/6)) are removed with their corner faces, by extending the faces around them.
- `remove_fillet` treated every rectangular face, including the planar faces of a plain box, as a fillet candidate. A fillet face must now meet its neighbours tangentially.
- `circular_fillet` (`SmApiCircularFillet`) with a radius below about 2e-4 ran for minutes and used gigabytes of memory, for example with radius 0.0001 on a box of side 0.001. The 1e-5 padding on each patch's bounding box kept the small fillet surfaces' intersection caches from meeting their chord height tolerance, so they subdivided to the smallest allowed patch. ([GitHub #8](https://github.com/NVIDIA-Omniverse/usd-brep/issues/8))
- `circular_fillet`, `chamfer_fillet`, `fillet_edges`, `fillet_edges_per_edge`, `variable_radius_fillet` and `set_bevel_corners` (and their `SmApi` functions) now reject a radius below 8 times the BRep tolerance (8e-5 by default) with `SM_ERR_INVALID_INPUT`, leaving the BRep unchanged. Below that they could hang, crash, fail after modifying the BRep, or, for `set_bevel_corners` and `variable_radius_fillet`, return a non-manifold solid of zero volume. ([GitHub #8](https://github.com/NVIDIA-Omniverse/usd-brep/issues/8))

#### Offsets
- `shell_brep` with extended offset read faces it had already deleted, for example when thickening a half-cylinder sheet inward past its radius.
- `shell_brep` (`SmApiShellBrepFull`) crashed when an inward shell exactly closed the cavity, for example a 20 x 30 x 10 box with its top face open and distance -10. When the merge failed to insert an intersection edge after the new edge had taken the intersection curve, it deleted the curve anyway. It now returns an error and leaves the input Brep unchanged. ([GitHub #13](https://github.com/NVIDIA-Omniverse/usd-brep/issues/13))
- `shell_brep` with extended offset (the default) could give different results for the same input in different processes, or after other operations in the same process, because it offset the edges in an order set by memory addresses. It now uses an order set by the Brep. Some inputs still give a result that is not a solid, for example the source repository's `problemShell4.smb` test part shelled outward by 5.08 with face 2 open. ([GitHub #10](https://github.com/NVIDIA-Omniverse/usd-brep/issues/10), [#21](https://github.com/NVIDIA-Omniverse/usd-brep/issues/21))
- `shell_brep` left a gap where an open face met a fillet tangentially, such as the open top of a box with filleted top edges, giving the wrong volume or an error. The wall is now closed there. ([GitHub #10](https://github.com/NVIDIA-Omniverse/usd-brep/issues/10), [#21](https://github.com/NVIDIA-Omniverse/usd-brep/issues/21))

#### Healing and topology
- `heal_brep` (`SmApiHealBrep`) missed missing seams on some torus faces, and inserting the first seam split an uncut torus in two. A seam that is a curve on the surface (`SmCrvOnSurf`) now keeps the face whole; one given as an independent 3D circle or B-spline can still split it.
- The `SmCurveClassification` discontinuity intersectors filed some boundary solutions under the wrong side, and put the W planes at V parameters.
- `SmBrep::DeleteFace` on the last face of a shell with a wire attached through a vertex-only loop left an invalid empty shell. The wire is now kept.
- `SmProjectedCurve::MakeOrthoNormal` failed for every non-zero axis, and for a rotation did not make the aux vector perpendicular to the axis.
- `heal_brep` now splits a boundary edge where it crosses a surface pole without a vertex. A crossing near the edge's endpoint can still be skipped.
- `SmVertexuse::ComputeUVSector` counted a face and loop that are both reversed as one reversal, not two, because `?:` binds less tightly than `+`.
- Healing missed some coincident vertex pairs and left them unmerged.
- The healer's edge-length histogram (`SM_HISTOGRAM_EDGELEN_INDEX`) converted a negative number to `ULONG` for edges shorter than 1, which is undefined behaviour; on aarch64 all those edges landed in one bucket.
- Healing read freed memory once its vertex search grew past its initial capacity.
- `heal_brep` could leave a cylindrical face with a pocket with two outer loops, and could merge a hollow solid's cavity into the solid. Spheres and tori bounded only by vertex loops are now repaired in place. A face with edge loops and a missing seam can still be skipped by seam insertion.
- `heal_brep` on a Brep with two regions could leave the wrong one as the exterior and still report success, for example on inward-facing STEP box shells.
- `SmBrep::RefreshGaps` and `SmBrep::RefreshTols` could report success when a gap refresh failed: `RefreshGaps` returned only the status of the last edgeuse and the last vertexuse it refreshed, and `RefreshTols` ignored it. Both now return `SM_ERR` if any refresh fails. No library operation calls them, so this affects only C++ code that calls them directly.

#### Geometry
- The `SmBSplineSurface` component constructor lost V knots when a surface had more V knots than U knots.
- `SmBSplineCurve::ReverseParameterization` and `SmBSplineSurface::Reverse` gave a different curve or surface when the knot vector was not clamped at both ends. Clamped knot vectors are unaffected.
- `SmBSplineSurface::TrimWithDomain` never checked that snapping the upper V trim boundary to a knot left the surface at least as large as before.
- `SmBSplineCurve::RemoveOneKnot` failed and discarded a valid result on parameter ranges over about 1e5, so `SmBSplineCurve::CreateExtendedCurve` with `SM_CT_CINFINITY` returned `SM_ERR`.
- `sm_TotalCoincidenceChecker`, used by cone and cylinder intersection, never found two equal lines running in opposite directions coincident.
- Reading a B-spline surface or curve of degree 18 or more from a native SMLib file now fails with `SM_ERR_INVALID_INPUT`. The read used to succeed, and evaluating the surface, or a surface extruded or swept from the curve, then aborted the process with a stack overflow (`*** stack smashing detected ***`). Degrees below 1 are rejected too; a negative degree used to crash the reader.
- `SmCone::IsNurbSurfaceCone` reported a NURBS surface as a cone when it failed to make one of its iso-parametric curves.
- Reading a B-spline surface or curve from a native SMLib file now fails with `SM_ERR_INVALID_INPUT` when a control point count is negative, below degree + 1 or too large to allocate, when the rational flag is neither 0 nor 1, or when the file ends, or holds text that is not a number, inside the B-spline data. Negative and huge counts used to crash or hang the reader or throw `std::bad_alloc`; a count below degree + 1 read, and every evaluation of the result failed; and the reader carried on past the end of a truncated file, which hung on ASCII files and crashed on binary ones. A failed read also leaked its NURB and the new curve or surface: 37 MB per read for a 1101 x 1101 surface with a bad rational flag.

#### Measurement
- `mass_properties`, `volume` and `Face.centroid` (`SmApiBrepComputeMassProperties`, `SmApiBrepComputeVolume`, `SmApiFaceComputeCentroid`) controlled only the area's integration error, so volume, centroid and inertia could miss the requested accuracy by far more: at the default 1e-3, a thin half shell's volume was 4.5e-2 off. They now control every property's error, and take longer. `mass_properties` about an `origin` far from the solid is also accurate now. ([GitHub #19](https://github.com/NVIDIA-Omniverse/usd-brep/issues/19))
- Face area, volume and mass properties could miss the requested accuracy on faces trimmed by curves with interior knots, such as circles: a half-disk's area was 20% off at `relative_accuracy=1e-1`. Each trim-curve knot span is now integrated separately. ([GitHub #11](https://github.com/NVIDIA-Omniverse/usd-brep/issues/11))
- `brep_relationship` (`SmApiBrepRelationship`) and `brep_distance` gave wrong results for cylinders and cones. `create_cone` returned its Brep in edit mode, in which distances were measured to a cap's whole plane, so cones 0.025 apart were reported touching at distance 0. Overlaps were also missed when no vertex or face sample lay inside the other body, and a shared boundary could count as an overlap. Edge points are now sampled too, and boundary points count as neither inside nor outside. An overlap that contains no sampled point is still reported as touching, for example a radius-1 sphere sunk 0.2 into some side faces of a box. ([GitHub #16](https://github.com/NVIDIA-Omniverse/usd-brep/issues/16))

#### Tessellation
- In the 1.0.0 wheel, `tessellate()` failures raised `ModuleNotFoundError` instead of `TessellationError`.

#### USD import and validation
- The BrepArray validator (`brep_validator`, rule BA.515) rejected cone semi-angles of 0 or below, although the schema allows (-pi/2, pi/2). Every exported `create_cylinder` (a cone with semi-angle 0) and every cone narrowing along its axis failed validation. It now accepts that range and rejects NaN. ([GitHub #14](https://github.com/NVIDIA-Omniverse/usd-brep/issues/14))
- Importing a USD BrepArray now fails with an error naming the array when a NURBS edge, wire edge or face surface needs more control vertices or knots than its packed array holds from its start index, or has fewer control vertices than its order. The import used to read past the end of the array: a short control vertex array imported values from beyond it (a box's volume came out 18 instead of 24), and a vertex count of 4,000,000,000, or 65536 x 65536 surface control vertices (a product that wrapped to 0), crashed it. UV trim curves already had this check.
- Importing a USD BrepArray with a NURBS surface or curve of order 19 or more now fails with an error naming the order. The import used to succeed, and evaluating the surface, or a surface extruded or swept from the curve, then aborted the process with a stack overflow (`*** stack smashing detected ***`); `brep_geometry_validator` aborted the same way. Orders 0 and 1 are rejected too, and a very large order no longer reads past the end of the knot arrays. A UV trim curve of an unsupported order is discarded and rebuilt from its 3D edge. A malformed UV trim record, such as order 0 with control vertices, no longer shifts the trims after it; they used to be read from the wrong place, which could give the imported solid the wrong shape.
- `SmBrep::AssertValid` at `SM_LEVEL_2` leaked memory on every call for a brep with faces on periodic surfaces: about 3.5 KB for a cylinder and 12.5 KB for a torus. `SmFace::HasSeamProblem` copied each seam curve into its output instead of handing it over, and abandoned the original. ([GitHub #9](https://github.com/NVIDIA-Omniverse/usd-brep/issues/9))

#### USD export
- USD export wrote analytic faces whose UV parameterisation had been swapped, such as a hole wall after a fillet, with loops that did not match their USD surface, giving an invalid BrepArray. A swapped face that shares its surface with another face is still exported the old way, with a warning. ([GitHub #4](https://github.com/NVIDIA-Omniverse/usd-brep/issues/4))

#### OCCT `.brep` conversion
- `occt_to_usd`: a planar face's `face:range` now contains the full extent of its B-spline edges. It was taken from 24 samples per edge and could miss an edge's furthest point, which made the face's area wrong after import. ([GitHub #11](https://github.com/NVIDIA-Omniverse/usd-brep/issues/11))
- `occt_to_usd` wrote a face on a periodic B-spline surface or curve partly outside the surface's domain when the face's trims crossed OCCT's seam. SMLib then read the face with its edges a quarter turn or more out of place, and volume and mass properties were wrong. Each periodic direction is now unrolled over the period the face or edge uses.

#### Build
- Windows builds failed when the checkout path contained a parenthesis. Without administrator rights or Developer Mode, they also failed with Visual Studio in its default `C:\Program Files (x86)`.

## [1.0.0] - 2026-10-08
### Added
- Initial public release of USD BRep: NURBS boundary-representation (BRep) solid modeling libraries, with C++ and Python APIs and OpenUSD import and export.
