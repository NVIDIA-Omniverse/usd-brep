<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Tessellation

**Source:** `source/SM_API/inc/SmApiBrep.h` — `SmApiTessellate`
**Python:** `_omni_solid.tessellate` → `PolyBrep` (wraps `SmApiTessellate`)

## Overview

Tessellation converts exact BRep geometry (NURBS surfaces) into polygon meshes (`SmPolyBrep` / Python `PolyBrep`) for rendering, export, or polygon-based operations. The tessellation algorithm subdivides each face into triangles that approximate the underlying surface within specified tolerances.

The output is a **`PolyBrep`** — use it with **[polygon booleans](polygon_booleans.md)**, **[USD mesh I/O](usd_import_export.md)**, or the inspection helpers in **[polybrep](polybrep.md)**.

## Parameters

| Parameter | Type | Description |
|---|---|---|
| `pBrep` | SmBrep* | BRep to tessellate |
| `dChordHeightTol` | double | Max distance from triangle surface to true surface (world units) |
| `dCurveAngleTolDeg` | double | Curve tangent angular tolerance (degrees) |
| `dSurfaceAngleTolDeg` | double | Surface angular tolerance (degrees) |
| `dMaxEdgeLength` | double | Edge-length subdivision target (world units); not a hard cap on final triangle edges |
| `dMaxAspectRatio` | double | Aspect-ratio target for surface subdivision; not a limit on final triangle shape |
| `rpResult` | SmPolyBrep*& | Output polygon BRep |

## How Parameters Control Quality

### Chord Height Tolerance (`dChordHeightTol`)
The most important parameter. Controls the maximum distance between the polygon mesh and the true NURBS surface. Smaller values = finer mesh = better surface approximation.

- **0.001** — very fine (mechanical CAD, close-up rendering)
- **0.01** — moderate (general visualization)
- **0.1** — coarse (distant LOD, preview)

This is in world units, so scale with your scene. A 1-meter object with `dChordHeightTol=0.001` means the mesh never deviates more than 1mm from the true surface.

### Curve Angle Tolerance (`dCurveAngleTolDeg`)

Controls refinement of BRep edge curves, using changes in curve tangent direction.
Smaller positive values produce finer boundary segments. It is independent of
surface refinement, although finer boundaries can also increase face triangle count.

Both angles are in degrees and default to **25.0** across the C++ API, Python and
hdUsdBrep, all sharing `SmTessellationDefaults` (a default-constructed
`SmTessellationParams` uses them). Setting either to **0** disables its angular
criterion, leaving chord-height and edge-length constraints to refine the mesh.

### Surface Angle Tolerance (`dSurfaceAngleTolDeg`)
Controls angular subdivision of surface patches. A patch is split while its surface
turns by more than this many degrees in U or V, measured from the end tangents of its
control polygon (or from surface evaluations when the control net is unsuitable). It
bounds how far the surface turns within a patch; it is not a guaranteed maximum angle
between the normals of adjacent output triangles.

- **5.0** — very fine (smooth highlight lines)
- **15.0** — moderate
- **30.0** — coarse

Lower values produce more triangles in areas of high curvature. This is particularly important for cylindrical and spherical surfaces where chord height alone might allow too-few triangles.

### Max Edge Length (`dMaxEdgeLength`)
Target maximum edge length used when sampling edges and subdividing faces (the kernel's base-polygon side length). It is not a hard cap: final triangle edges can come out somewhat longer. Prevents very large flat triangles on planar or nearly-planar regions.

- Set to 0 or a very large value to disable.
- Useful when you need uniform mesh density for downstream processing (e.g., FEA meshing).

### Max Aspect Ratio (`dMaxAspectRatio`)
Aspect-ratio target for the cells a surface is subdivided into before triangulation. It does not limit the aspect ratio of the final triangles, which can still be long and thin.

- Set to 0 to disable (the default).

## Choosing Parameters by Use Case

The angle column applies to both curve and surface tolerances. These are optional
overrides; the API defaults are chord height `0`, both angles `25`, maximum edge
length `0`, and maximum aspect ratio `0`.

| Use Case | Chord Height | Curve/Surface Angle | Max Edge |
|---|---|---|---|
| High-quality rendering | 0.001 | 5.0 | 0 |
| General visualization | 0.01 | 15.0 | 0 |
| Preview / LOD | 0.1 | 30.0 | 0 |
| FEA mesh | 0.005 | 10.0 | scene_size/50 |
| 3D printing | 0.05 | 15.0 | 0 |

## Python usage

```python
import _omni_solid as sm

brep = sm.create_sphere((0, 0, 0), 5.0)
mesh = sm.tessellate(
    brep,
    chord_height_tolerance=0.0,
    curve_angle_tolerance_deg=25.0, surface_angle_tolerance_deg=25.0,
    max_edge_length=0.0,
    max_aspect_ratio=0.0,
)
# mesh is a PolyBrep — see polybrep.md for topology and measurement APIs
```

`sm.tessellation_defaults()` returns the defaults below as a fresh dict, keyed to
match these keyword arguments. `inspect.signature()` does not work on pybind11
builtins, so this is the only way to read them from Python. Splat it to override
one control and keep the rest:

```python
mesh = sm.tessellate(brep, **{**sm.tessellation_defaults(), "chord_height_tolerance": 0.001})
```

| Parameter | Default | Description |
|---|---|---|
| `brep` | — | `Brep` to tessellate |
| `chord_height_tolerance` | `0.0` | Max distance from mesh to true surface (world units); `0` disables |
| `curve_angle_tolerance_deg` | `25.0` | Curve tangent tolerance; `0` disables the angular criterion |
| `surface_angle_tolerance_deg` | `25.0` | Surface tolerance; `0` disables the angular criterion |
| `max_edge_length` | `0.0` | Edge-length subdivision target (not a hard cap); `0` disables. **Negative** values select an automatic target from model size |
| `max_aspect_ratio` | `0.0` | Subdivision aspect-ratio target, not a limit on final triangles; `0` disables |

Errors raise `RuntimeError` with the underlying `SmApiStatus` on failure.

## C API Usage

```cpp
SmBrep* pBrep = /* ... */;
SmPolyBrep* pPoly = nullptr;
// A default-constructed SmTessellationParams is the standard quality; assign
// only the controls you want to change.
SmTessellationParams sParams;
sParams.dChordHeightTol     = 0.01;
sParams.dCurveAngleTolDeg   = 15.0;
sParams.dSurfaceAngleTolDeg = 15.0;
sParams.dMaxEdgeLength      = 0.0;   // 0 = no limit
sParams.dMaxAspectRatio     = 0.0;   // 0 = disabled

SmApiStatus stat = SmApiTessellate(pBrep, pPoly, sParams);

// Or take the defaults wholesale:
SmApiStatus statDefault = SmApiTessellate(pBrep, pPoly);
```

## Trade-offs

- Tighter tolerances produce more triangles, increasing memory and rendering cost.
- Chord height is the primary quality driver. Angle tolerance adds refinement in curved areas.
- For interactive preview, start coarse and refine. For final output, set tolerances based on the output medium's resolution requirements.
- The relationship between tolerance and triangle count is roughly quadratic: halving chord height tolerance approximately quadruples triangle count.

## Partial output and failure diagnostics

The API always collects per-face failures internally. By default, any failed face
causes an error and the incomplete mesh is discarded. To accept partial output,
C++ callers must set `bAllowPartial=TRUE` and supply `pFailures` or `pReport`; Python callers
must set `allow_partial=True` and either `return_failures=True` or `return_report=True`:

```python
mesh, failures = sm.tessellate(brep, allow_partial=True, return_failures=True)
for failure in failures:
    print(failure["face_index"], failure["stage"], failure["status"])
```

Each entry records the first failure for an original input face: its index in
`GetFaces()` / `faces()` order, stage (`boundary_preparation`, `face_tessellation`,
or `polygon_output`), and numeric kernel status. No face pointers are retained.
Kernel and polygon-output errors fail in either mode. Output errors discard the
whole mesh because a failed face may already have modified shared topology.

C++ `pFailures` is cleared on entry and retains diagnostics on failure. Output
meshes are null on error. Test `!failures.IsEmpty()` in C++ or `bool(failures)`
in Python to determine whether any faces failed. Partial output without detailed
reporting raises `ValueError` in Python; kernel errors raise `RuntimeError`.

Python tessellation errors raise `TessellationError` (a `RuntimeError`) with `failures`,
`status`, and `status_name`, including strict rejection and operation errors. Each face failure
also includes `status_name`. Diagnostics are available without enabling partial output.

Use `return_report=True` to receive `(mesh, report)` with `failures`, `lamina_edge_count`,
`spine_edge_count`, `input_manifold`, `output_manifold`, and `has_mesh`. These manifold flags
check edge incidence only, not full solid validity. Edge counts follow the kernel's PolyEdge
counts (spine edges include each radial member). Statistics describe the generated mesh even
when strict mode rejects it; errors retain them in `TessellationError.report`. They do not
change strict/partial acceptance. `return_report` takes precedence over `return_failures`.

## Boundary polylines

`SmApiTessellateBoundaries(brep, angleToleranceDegrees, points, vertexCounts, edges)`
samples trimmed edges once, using only a finite, positive angle in degrees.
Outputs replace previous values: flat points, per-edge counts and borrowed edges.
Keep the Brep alive and reacquire edges after topology changes.

Missing/failed curves have zero counts and return an error, retaining valid samples.
Invalid inputs clear outputs. Degenerate curves retain one point; `GetBoundaryData`
omits empty and single-point edges from USD lines.

Python: `points, counts, edges = sm.tessellate_boundaries(brep)`.
Keyword options: `angle_tolerance_deg=5.0`, `allow_partial=False`.
Sampling failures raise unless partial output is enabled; invalid arguments always raise.

## USD mesh extraction

Own a tessellated `SmPolyBrep` with `SMU_BrepConvert::TessellatedMeshPtr` and read it
with `GetTessellatedMeshData`, which returns USD arrays and copied material paths that
outlive the source BRep. Keep the source alive until extraction returns. Destroying
the source before the mesh requires `TessellatedMeshPtr`; otherwise keep the source
alive until mesh destruction.

The back-pointer flag is enabled for the call and restored afterwards, so a cached mesh
keeps its own setting; the deleter disables it before destruction, which is what stops
`~SmPolyBrep` from reaching into a freed BRep. One call at a time per mesh.
`GetMeshData` callers are unaffected.
