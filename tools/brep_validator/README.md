<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# BrepArray Validator

## Quick Reference

**System Overview**: Checks USD BrepArray data sanity and selected schema constraints
**Core Class**: `BrepValidator` inherits from `BaseRuleChecker`  
**Requirements Enum**: `BrepArrayRequirements`  
**Main Entry Point**: `CheckPrim(prim: Usd.Prim)`  
**Validation Pattern**: `_AddFailedCheck(requirement=cap.BrepArrayRequirements.BA_XXX, message="...", at=brep_array)`
**Analytic Geometry**: Supports analytic surface types (sphere, plane, cylinder, cone, torus) and analytic curve types (line, circle, ellipse)

**Plugin**: `plugin.BrepValidatorPlugin` registers `BrepValidator` with usd-validation-nvidia (category `Omni:Geometry`). The usd-brep wheel declares it as an `omni.asset_validator` entry point, so after `pip install "usd-brep[validation]"` the rules run under `nvidia_usd_validate`. Elsewhere, call `register_all()` / `unregister_all()`.

## Purpose

The BrepArray Validator checks the structural integrity, data consistency, and selected topological relationships of Boundary Representation (Brep) models stored in USD format. It is the data-sanity layer before kernel import, not a replacement for geometric validation.

**Validation Coverage:**
- **Topology Validation**: Hierarchical relationships between breps, regions, shells, faces, loops, edgeuses, edges, and vertices
- **Geometry Data Validation**: Authored NURBS, analytic, extent, and point data
- **Data Type Validation**: USD schema data type integrity for all 15 topology/geometry strata
- **Data Consistency**: Array sizes, index relationships, and attribute constraints
- **Token Validation**: Enumerated values and type checking
- **Manifold Constraints**: Per-brep topology validation using precomputed partitions
- **NURBS Mathematics**: Order, control points, weights, and knot vector validation

### Validation boundary

The representation contract is the union of the
[schema](../../source/schema/omniSolid/resources/schema.usda) and the
[UsdSolid proposal's rules](../../proposals/UsdSolid/README.md#rules-and-requirements).
Not every rule can be checked without a geometry kernel. A clean Python report
does not certify that the model is geometrically valid; use the kernel-backed
`brep_geometry_validator` / `AssertValid` for that layer of validation.

The following Python checks have been retired; their identifiers are not reused:

- **BA.730**: NURBS endpoint-to-vertex agreement needs kernel curve evaluation
  and tolerance handling. The Python de Boor evaluator for this check is removed.
  Orders, packed array sizes, knots, weights, and vertex indices are still checked.
- **BA.761**: Counting repeated edge indices cannot establish whether a face has
  the required geometric seams. Seam requirements are unchanged, but this heuristic
  is no longer reported as a schema failure. Index and radial-link checks remain.
- **BA.762**, and overlapping face-domain checks **BA.631 / BA.765**: Periodic
  face ranges need not use `[0, 2*pi]`. The proposal requires length no greater
  than one period, not a particular origin. BA.560 / BA.562-BA.565 retain those
  span limits, alongside finite/ordered range checks and the sphere latitude
  bounds. Agreement between a face, its trims, and the surface's chosen domain
  still requires geometric validation.

This is a scoped cleanup, not a completed audit of every existing check. Other
legacy geometric checks, including UV-curve evaluation and the separate BA.630
edge-range policy, are unchanged.

## Current Status

The validator implements the checks summarized below; this is not exhaustive
coverage of the schema/proposal's geometric requirements.

**Recent update:** Analytic surface validation (sphere, plane, cylinder, cone, torus) and analytic curve validation (line, circle, ellipse) added, plus hardened type/sequence guards (including UnregisteredValue handling and integer count coercion); all BA requirement tests now pass.

**Major System Features:**
- **Structural Coverage**: Data type, packing, topology-index, and analytic surface/curve data checks
- **Data Type Integrity**: Comprehensive USD schema data type validation for all topology/geometry strata with clear error reporting
- **Systematic Validation**: Comprehensive CheckPrim method that validates all aspects in logical order: breps → regions → shells → faces → loops → edges → vertices → NURBS geometry → data types
- **Advanced NURBS Validation**: Full mathematical validation of 3D curves, UV curves, and tensor product surfaces including knot vectors, control points, and weights
- **Manifold Topology**: Enhanced per-brep manifold topology validation using precomputed partitions for accurate multi-brep analysis
- **Schema Consistency**: Comprehensive schema validation ensuring geometric data matches topology declarations and USD type requirements

**Advanced NURBS Mathematics:**
- **BA_445/BA_450**: Separate U and V direction knot size requirements for granular validation
- **BA_455/BA_460**: Direction-specific knot ordering validation (non-decreasing constraints)
- **BA_465/BA_470**: Control point containment and schema consistency validation
- **Comprehensive Geometry Validation**: Full validation of 3D curves (BA_330-BA_370), UV curves (BA_375-BA_415), and surfaces (BA_420-BA_470)
- Enhanced mathematical correctness for all NURBS geometric entities

**Implementation Quality:**  
- **Systematic Architecture**: Well-organized validation flow with clear separation of topology, geometry, data type, and consistency checks
- **Robust Error Handling**: Comprehensive error reporting with precise requirement codes, contextual messages, and location information
- **Data Type Validation**: USD schema-level type validation with clear expected vs actual type reporting for all 15 topology/geometry strata
- **Performance Optimization**: Efficient algorithms for large models with minimal memory footprint and graceful handling of malformed data
- **Diagnostics**: Findings use requirement codes and identify affected data

## Code Structure

### Key Classes and Methods
```python
# Core validation class
class BrepValidator(BaseRuleChecker):
    def CheckPrim(self, prim: Usd.Prim) -> None  # Main entry point
    def _AddFailedCheck(self, requirement, message, at) -> None  # Error reporting
    def _validate_array_sizes_and_authored(self, brep_array, attributes, requirement, size=None) -> None
    def _validate_allowed_tokens(self, brep_array, attr_name, allowed_tokens, requirement) -> None

# Requirements enumeration
class BrepArrayRequirements(Requirement, Enum):
    BA_000 = ("BA.000", "brep-array-consistent-sizes", "description...", "url", "Core USD", ("correctness",))
    # ... BA_005, BA_010 through BA_555
```

### Validation Pattern Template
```python
# Standard validation method pattern:
def _validate_COMPONENT_arrays(self, brep_array: Usd.Prim) -> None:
    # 1. Define attributes to check
    attributes_to_check = ["attr1", "attr2", "attr3"]
    
    # 2. Calculate expected size
    expected_size = self._calculate_expected_size(brep_array)
    
    # 3. Validate array sizes and authoring
    self._validate_array_sizes_and_authored(brep_array, attributes_to_check, 
                                           cap.BrepArrayRequirements.BA_XXX, expected_size)
    
    # 4. Validate token constraints
    self._validate_allowed_tokens(brep_array, "attr:type", ["token1", "token2"], 
                                 cap.BrepArrayRequirements.BA_YYY)
```

### Files Overview
- **`brep_validator.py`**: Core validation with BrepValidator class and BrepArrayRequirements enum

### Test Files
Internal test assets live outside the shipped package.

## Prerequisites

### Required Dependencies
- **USD (Universal Scene Description)**: Python bindings (`pxr` module)
- **Omni Asset Validator**: Framework for validation rules (`omni.asset_validator`, `omni.capabilities`). Provided by the `usd-validation-nvidia` pip package; install it into the repo Python with `pip install "usd-validation-nvidia>=1.22.0,<2"`. (The `unittest` suite `tools/brep_validator_test/test_brep_validator.py`, via `utils/base_test_case.py`, installs it automatically on first run.)
- **Distribution**: The library ships as `brep_validator/` in the usd_brep release package and in the usd-brep wheel (`pip install usd-brep[validation]`). It is no longer published as a separate packman package.

### Environment Setup
The validator requires USD and the omniSolid plugins available on the host. There are no packaged setup scripts for the **library**; configure the environment manually using the paths in your checkout. Examples assume the repo root is `/mnt/c/code/solidmodeling` on Linux (WSL) or `C:\code\solidmodeling` on Windows. Adjust if your layout differs.

### Linux (bash/WSL)
```bash
export USD_ROOT="/mnt/c/code/solidmodeling/_build/target-deps/usd/release"
export PYTHONPATH="$USD_ROOT/lib/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$USD_ROOT/lib:${LD_LIBRARY_PATH:-}"
export PATH="$USD_ROOT/bin:${PATH:-}"

# Required: omniSolid plugin resources
export OMNISOLID_PLUGIN_PATH="/mnt/c/code/solidmodeling/_build/schema/omniSolid/resources"

# Asset validator: install via pip into the active Python (one-time)
#   pip install "usd-validation-nvidia>=1.22.0,<2"
```

### Windows (PowerShell)
```powershell
$usdRoot = "C:\code\solidmodeling\_build\target-deps\usd\release"
$env:USD_ROOT  = $usdRoot
$env:PYTHONPATH = "$usdRoot\lib\python;$($env:PYTHONPATH)"
$env:Path       = "$usdRoot\bin;$usdRoot\lib;$($env:Path)"

# Required: omniSolid plugin resources
$env:OMNISOLID_PLUGIN_PATH = "C:\code\solidmodeling\_build\schema\omniSolid\resources"

# Asset validator: install via pip into the active Python (one-time)
#   pip install "usd-validation-nvidia>=1.22.0,<2"
```

Notes:
- Ensure the Python you use can load shared libraries from the USD `lib` directory (hence `LD_LIBRARY_PATH`/`Path` updates).
- The library itself does not hardcode a plugin path; helper scripts may set `OMNISOLID_PLUGIN_PATH` for you.

## Usage Example (library)

```python
import os
from pxr import Usd
from brep_validator import BrepValidator

# OMNISOLID_PLUGIN_PATH is expected to be set externally to the omniSolid plugin resources directory.
stage = Usd.Stage.Open("path/to/your.usda")
prim = stage.GetPrimAtPath("/World/brepArray")

checker = BrepValidator(verbose=True, consumerLevelChecks=["BrepValidator"], assetLevelChecks=["BrepValidator"])
checker.CheckPrim(prim)

for issue in checker.GetIssues():
    requirement = issue.requirement.code if issue.requirement else "n/a"
    print(f"[{issue.severity}] {issue.message} (Requirement: {requirement})")
```

### Supported File Types
- `.usda` (USD ASCII)
- `.usdc` (USD Crate/Binary)
- `.usd` (USD - format determined by content)

### Validation Philosophy
The validator follows a **permissive-for-empty, strict-for-authored** approach:

- **Empty Models**: BrepArray prims with no authored attributes pass validation (useful for placeholders)
- **Partial Models**: Models with some authored attributes validate only what's present
- **Complete Models**: Fully authored models undergo comprehensive validation of all relationships
- **Error Detection**: Any authored data must be structurally correct and consistent

## Validation Requirements

The validator implements requirements organized by topology/geometry:

### Core Data Consistency (BA_000-BA_055) + Data Types
- **Brep Level** (BA_000-BA_055): Array consistency, positive tolerances, extent structure and ordering, extent containment validation, valid type tokens
- **Brep Data Types** (BA_061): USD schema data type validation for brep attributes

### Topological Elements (BA_065-BA_291) + Data Types
```python
# Requirement mapping by topological level:
REGION_REQUIREMENTS = [BA_065, BA_070, BA_075, BA_076]     # region arrays, types, data types
SHELL_REQUIREMENTS = [BA_080, BA_085, BA_090, BA_091]          # shell arrays, types, data types
FACEUSE_REQUIREMENTS = [BA_100, BA_105, BA_110, BA_115, BA_116]  # faceuse arrays, orientation, indexing, data types
FACE_REQUIREMENTS = [BA_120, BA_125, BA_130, BA_135, BA_140, BA_145, BA_150, BA_155, BA_160, BA_161]  # face arrays, UV ranges, data types
LOOP_REQUIREMENTS = [BA_165, BA_170, BA_175, BA_176]       # loop arrays, vertex references, data types
EDGEUSE_REQUIREMENTS = [BA_180, BA_185, BA_190, BA_195, BA_196, BA_200]  # edgeuse arrays, relationships, data types
EDGE_REQUIREMENTS = [BA_205, BA_210, BA_215, BA_225, BA_230, BA_235, BA_237, BA_245]  # edge arrays, manifold constraints, data types
WIREEDGE_REQUIREMENTS = [BA_250, BA_255, BA_260, BA_265, BA_270, BA_275, BA_290, BA_291]  # wire edge validation, data types
```

- **Region Level** (BA_065-BA_076): `region:shellCount`, `region:type` validation + data type validation
- **Shell Level** (BA_080-BA_091): `shell:faceuseCount`, `shell:pointType` + data type validation
- **Faceuse Level** (BA_100-BA_116): `faceuse:faceIndex`, `faceuse:orientationType` validation + data type validation
- **Face Level** (BA_120-BA_161): `face:loopCount`, `face:surfaceType`, UV range validation + data type validation
- **Loop Level** (BA_165-BA_176): `loop:edgeuseCount`, `loop:vertexIndex` validation + data type validation
- **Edgeuse Level** (BA_180-BA_196): `edgeuse:edgeIndex`, `edgeuse:orientationType`, radial relationships + data type validation
- **Edge Level** (BA_205-BA_245): `edge:curveType`, `edge:vertexIndices`, manifold constraints + data type validation (BA_237)
- **WireEdge Level** (BA_250-BA_291): manifold emptiness, curve types, indexing + data type validation

### Geometric Elements (BA_295-BA_471) + Data Types
```python
# Geometric validation requirements by component:
VERTEX_REQUIREMENTS = [BA_295, BA_300, BA_305, BA_310, BA_315, BA_316, BA_320, BA_325, BA_326, BA_327]  # vertex arrays, types, positions, data types
CURVE3D_REQUIREMENTS = [BA_330, BA_335, BA_340, BA_345, BA_350, BA_355, BA_360, BA_365, BA_370, BA_371]  # 3D NURBS curves, data types
CURVEUV_REQUIREMENTS = [BA_375, BA_380, BA_385, BA_390, BA_395, BA_400, BA_405, BA_410, BA_415, BA_416]  # UV curves, data types
SURFACE_REQUIREMENTS = [BA_420, BA_425, BA_430, BA_435, BA_440, BA_445, BA_450, BA_455, BA_460, BA_465, BA_470, BA_471]  # NURBS surfaces, data types

# Comprehensive validation flow in CheckPrim method:
def CheckPrim(self, prim: Usd.Prim) -> None:
    """Complete systematic validation of all BrepArray requirements"""
    brep_offsets = self._compute_brep_offsets(prim)  # Multi-brep partitioning
    
    # Core brep validation (BA_000-BA_055)
    self._validate_brep_extent(prim)      # Extent validation and ordering
    self._validate_brep_tols(prim)        # Tolerance validation
    self._validate_brep_array(prim)       # Array consistency
    
    # Topology validation (BA_065-BA_290) 
    self._validate_region_arrays(prim)    # Region topology
    self._validate_shell_arrays(prim)     # Shell topology
    self._validate_faceuse_arrays(prim, brep_offsets["faces"])  # Faceuse relationships
    self._validate_face_arrays(prim)      # Face topology and UV ranges
    self._validate_face_loop_count_minimum(prim)  # Loop count minimum
    self._validate_face_ranges(prim)      # Face UV range validation
    self._validate_loop_arrays(prim)      # Loop topology
    self._validate_loop_vertex_index(prim, ...)  # Loop vertex index validation
    self._validate_edge_arrays(prim, ...) # Edge validation
    self._validate_edgeuse_arrays(prim, ...)  # Edgeuse validation
    self._validate_wireEdge_arrays(prim, ...)  # Wire edge validation
    
    # Geometry validation (BA_295-BA_471)
    self._validate_vertex_arrays(prim)    # Vertex positions and types
    self._validate_point_position(prim)   # Point position validation
    self._validate_curve3d_nurb_*(prim)   # 3D NURBS curve validation
    self._validate_curve3d_knots(prim)    # 3D curve knot validation
    self._validate_curveUv_data(prim)     # UV NURBS curve validation
    self._validate_surface_*(prim)        # NURBS surface validation
    
    # Analytic surface validation (BA_480-BA_526)
    self._validate_surface_sphere_data(prim)    # Sphere surface validation
    self._validate_surface_plane_data(prim)     # Plane surface validation
    self._validate_surface_cylinder_data(prim)  # Cylinder surface validation
    self._validate_surface_cone_data(prim)      # Cone surface validation
    self._validate_surface_torus_data(prim)     # Torus surface validation
    
    # Analytic curve validation (BA_530-BA_555)
    self._validate_curve3d_circle_data(prim)    # Circle curve validation
    self._validate_curve3d_line_data(prim)      # Line curve validation
    self._validate_curve3d_ellipse_data(prim)   # Ellipse curve validation
    
    # Analytic domain range validation (BA_560-BA_571)
    self._validate_face_range_domain_limits(prim)   # Surface UV domain limits
    self._validate_edge_range_domain_limits(prim)   # Curve parameter domain limits
    
    # Advanced validation
    self._validate_vertex_position_containment(prim)         # Vertex containment
    self._validate_edge3d_nurbs_control_point_containment(prim)  # Edge containment
    self._validate_surface_nurbs_control_point_containment(prim) # Surface containment
    self._validate_schema_consistency(prim)       # Schema consistency validation
    self._validate_nurbs_data_completeness(prim)  # NURBS completeness
    self._validate_nurbs_mathematical_consistency(prim)  # NURBS math consistency
    self._validate_topology_geometry_correspondence(prim)  # Topology-geometry correspondence
    self._validate_attribute_data_types(prim) # USD data type validation (BA_061, BA_076, BA_091, etc.)
```

- **Vertex Arrays** (BA_295-BA_327): `vertex:pointType`, position data validation + data type validation for vertex and point attributes
- **3D NURBS Curves** (BA_330-BA_371): Edge3d curve orders, control vertices, weights, knot vectors, schema consistency + data type validation
- **UV NURBS Curves** (BA_375-BA_416): Parameter space curve geometry, knot validation, schema consistency + data type validation
- **NURBS Surfaces** (BA_420-BA_471): Tensor product surfaces, U/V direction knot vectors, schema consistency + data type validation

### Analytic Surface Elements (BA_480-BA_526)
```python
# Analytic surface validation requirements:
SPHERE_REQUIREMENTS = [BA_480, BA_481, BA_482, BA_483, BA_484, BA_485]   # sphere size, radius, axis, refDirection, orthogonality, schema
PLANE_REQUIREMENTS = [BA_490, BA_491, BA_492, BA_493, BA_495]            # plane size, axis, refDirection, orthogonality, schema
CYLINDER_REQUIREMENTS = [BA_500, BA_501, BA_502, BA_503, BA_504, BA_505] # cylinder size, radius, axis, refDirection, orthogonality, schema
CONE_REQUIREMENTS = [BA_510, BA_511, BA_512, BA_513, BA_514, BA_515, BA_516]  # cone size, radius, axis, refDirection, orthogonality, semiAngle, schema
TORUS_REQUIREMENTS = [BA_520, BA_521, BA_522, BA_523, BA_524, BA_525, BA_526] # torus size, majorRadius, minorRadius, axis, refDirection, orthogonality, schema
```

- **Sphere Surfaces** (BA_480-BA_485): Array sizes, positive radius, unit axis/refDirection, orthogonality, schema consistency
- **Plane Surfaces** (BA_490-BA_495): Array sizes, unit axis/refDirection, orthogonality, schema consistency
- **Cylinder Surfaces** (BA_500-BA_505): Array sizes, positive radius, unit axis/refDirection, orthogonality, schema consistency
- **Cone Surfaces** (BA_510-BA_516): Array sizes, non-negative radius, unit axis/refDirection, orthogonality, semiAngle range, schema consistency
- **Torus Surfaces** (BA_520-BA_526): Array sizes, positive major/minor radii, unit axis/refDirection, orthogonality, schema consistency

### Analytic Curve Elements (BA_530-BA_555)
```python
# Analytic curve validation requirements:
CIRCLE_REQUIREMENTS = [BA_530, BA_531, BA_532, BA_533, BA_534]           # circle size, radius, axis, refDirection, orthogonality
LINE_REQUIREMENTS = [BA_540, BA_541]                                     # line size, direction unit length
ELLIPSE_REQUIREMENTS = [BA_550, BA_551, BA_552, BA_553, BA_554, BA_555]  # ellipse size, radii, axis, refDirection, orthogonality
```

- **Circle Curves** (BA_530-BA_534): Array sizes, positive radius, unit axis/refDirection, orthogonality
- **Line Curves** (BA_540-BA_541): Array sizes, unit direction
- **Ellipse Curves** (BA_550-BA_555): Array sizes, positive xRadius/yRadius, unit axis/refDirection, orthogonality

### Analytic Domain Range Limits (BA_560-BA_571)
```python
# Domain range validation requirements (per STEP ISO 10303-42, IGES, PRC ISO 14739-1):
SPHERE_DOMAIN = [BA_560, BA_561]   # U span <= 2pi, V in [-pi/2, pi/2]
CYLINDER_DOMAIN = [BA_562]         # U span <= 2pi
CONE_DOMAIN = [BA_563]             # U span <= 2pi
TORUS_DOMAIN = [BA_564, BA_565]    # U span <= 2pi, V span <= 2pi
CIRCLE_DOMAIN = [BA_570]           # edge param span <= 2pi
ELLIPSE_DOMAIN = [BA_571]          # edge param span <= 2pi
```

- **Sphere Domain** (BA_560-BA_561): Angular U span ≤ 2π, latitude V bounded to [-π/2, π/2]
- **Cylinder Domain** (BA_562): Angular U span ≤ 2π (V is linear/unbounded)
- **Cone Domain** (BA_563): Angular U span ≤ 2π (V is linear/unbounded)
- **Torus Domain** (BA_564-BA_565): Both U and V angular spans ≤ 2π
- **Circle Edge Domain** (BA_570): Edge parameter span ≤ 2π
- **Ellipse Edge Domain** (BA_571): Edge parameter span ≤ 2π
- **Plane/Line**: No domain limits (linear parameters are unbounded)

## Architecture

### Multi-Brep Support
The validator correctly handles both single-brep and multi-brep USD models by:

1. **Hierarchical Partitioning**: Uses top-down count aggregation for topology elements
2. **Index Usage Analysis**: Analyzes actual index references for geometry elements
3. **Per-Brep Validation**: Ensures indices stay within appropriate brep partitions

### Empty Model Support
The validator gracefully handles empty or minimal BrepArray models by:

1. **Optional Attributes**: No attributes are required to be authored for empty models
2. **Conditional Validation**: Topology and geometry validation only occurs when data is present
3. **Flexible Requirements**: Distinguishes between structural errors and missing optional data

<a name="key-components--implementation-details"></a>

### Key Components and Implementation Details

```python
# Core validation infrastructure:
class BrepConstants:
    NUMERICAL_TOLERANCE = 1e-11
    @staticmethod
    def safe_get_attribute(brep_array, attr_name, default_value=None): # Returns [] if not authored
    
# Partition computation for multi-brep models:
def _compute_brep_offsets(self, brep_array: Usd.Prim) -> dict[str, list[int]]:
    # Returns offsets for: "faces", "loops", "edgeuses", "edges", "vertices"
    
# Standard validation helpers:
def _validate_array_sizes_and_authored(self, brep_array, attributes, requirement, size=None, require_authored=True):
    # Validates: attribute authoring, consistent sizes, expected counts
    
def _validate_allowed_tokens(self, brep_array, attr_name, allowed_tokens, requirement):
    # Token validation with proper error reporting
    
def _validate_surface_sphere_data(...), _validate_surface_plane_data(...), etc.:
    # Validate analytic surface geometry (size, radius, axis, refDirection, orthogonality, schema)

def _validate_curve3d_circle_data(...), _validate_curve3d_line_data(...), _validate_curve3d_ellipse_data(...):
    # Validate analytic curve geometry (size, radius/direction, axis, refDirection, orthogonality)
    
def _validate_indexing_relationships(self, brep_counts, arrays, target_array, partition_array, brep_array, requirement):
    # Multi-brep index validation using partitions
```

**Key Components:**
- **`BrepValidator`**: Main validation class with comprehensive validation methods
- **`BrepArrayRequirements`**: Enum with requirement definitions (BA_000, BA_005, ..., BA_555)
- **`BrepConstants`**: Utility class with `NUMERICAL_TOLERANCE = 1e-11` and safe attribute access
- **`_compute_brep_offsets()`**: Multi-brep partition calculation using hierarchical aggregation
- **`_validate_*_arrays()`**: Component-specific validation methods following consistent patterns
- **`_validate_attribute_data_types()`**: USD schema data type validation for all topology/geometry strata
- **Manifold validation**: Uses precomputed partitions for per-brep topology constraints

## Development Notes

### Data Type Validation
The validator includes comprehensive USD schema data type validation for all topology/geometry strata:

#### Per-Stratum Data Type Requirements
```python
# Data type validation by stratum:
def _validate_attribute_data_types(self, brep_array: Usd.Prim) -> None:
    # BA_061: Brep attributes (int[], double[], double3[], uint[], token[])
    # BA_076: Region attributes (int[], uint[], token[])
    # BA_091: Shell attributes (int[], uint[], token[])
    # BA_116: Faceuse attributes (uint[], token[])
    # BA_161: Face attributes (int[], uint[], token[], double2[])
    # BA_176: Loop attributes (int[], uint[])
    # BA_196: Edgeuse attributes (uint[], token[])
    # BA_237: Edge attributes (int[], token[], int2[], double[])
    # BA_291: WireEdge attributes (int[], token[], int2[], double[])
    # BA_316: Vertex attributes (int[], token[])
    # BA_326: VertexPoint attributes (point3d[])
    # BA_327: ShellPoint attributes (point3d[])
    # BA_371: Edge3dNurb attributes (int[], point3d[], double[])
    # BA_416: CurveUv attributes (int[], double2[], double[])
    # BA_471: Surface attributes (int[], point3d[], double[])
```

**Key Features:**
- **Expected vs Actual Reporting**: Clear error messages showing `Expected 'double[]' but got 'float[]'`
- **Real-World Issue Detection**: Finds actual USD schema violations in production files
- **15 Topology/Geometry Strata**: Complete coverage from brep level to surface NURBS
- **Integration**: Runs automatically as part of standard validation flow

#### Data Type Validation Limitation

**Important**: Data type validation has a known limitation when used with the OmniSolid schema plugins.

When the `omniSolid` schema plugins are registered (required to recognize BrepArray prims), USD automatically normalizes attribute types to match the schema definition. For example:
- This makes data type mismatches invisible to the validator

**Impact:**
- Data type validation tests (BA_061, BA_076, BA_091, BA_116, BA_161, etc.) cannot detect mismatches when schemas are registered
- The validation code remains in place for environments without schema plugins
- Core structural, topology, and NURBS validations are unaffected

**Workaround:** To validate raw data types, use a USD stage without registering the OmniSolid plugins. However, this prevents BrepArray prim recognition.

### Advanced NURBS Validation
The validator includes comprehensive NURBS (Non-Uniform Rational B-Splines) validation for:

#### 3D Curve Validation (BA_330-BA_370)
```python
# NURBS 3D curve validation pattern:
def _validate_curve3d_nurb_order_vertex_count(self, brep_array: Usd.Prim):
    # BA_330: Array size must equal count of "BrepCurve3dNurbAPI" in edge:curveType
    # BA_335: Order values must be positive (> 0)
    # BA_340: Order must be <= vertexCount for each curve
    orders = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get() or []
    vertex_counts = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:vertexCount").Get() or []
    
    for idx, (order, vertex_count) in enumerate(zip(orders, vertex_counts)):
        if order <= 0:  # BA_335 violation
        if order > vertex_count:  # BA_340 violation
```

- **BA_330**: `brep:edge3dNurb:curve3d:nurb:order` size = count of `"BrepCurve3dNurbAPI"` entities
- **BA_335**: Order values must be positive (`order > 0`)
- **BA_340**: Order bounded by vertex count (`order <= vertexCount`)
- **BA_345**: Control vertices and weights array size consistency
- **BA_350**: Weight values must be positive
- **BA_355**: Knot vector size = `order + vertexCount`
- **BA_360**: Knot vector non-decreasing order
- **BA_365**: Edge NURBS control point containment
- **BA_370**: Edge 3D NURBS schema and data consistency

#### UV Curve Validation (BA_375-BA_415)
- **BA_375**: CurveUV array size consistency validation
- **BA_380**: Order validation (positive values)
- **BA_385**: Order bounded by vertex count
- **BA_390**: Control vertex array size correctness
- **BA_395**: Knot vector size validation
- **BA_400**: Knot vector ordering validation
- **BA_405**: Weight array size validation
- **BA_410**: Weight array positivity validation
- **BA_415**: CurveUV NURBS schema consistency
- Parameter space curve geometry and edge curve consistency

#### Surface Validation (BA_420-BA_470)
```python
# NURBS surface validation implementation:
def _validate_surface_knots(self, brep_array: Usd.Prim):
    # Get surface counts and attributes
    surface_type_values = brep_array.GetAttribute("face:surfaceType").Get() or []
    nurb_surface_count = sum(1 for surface_type in surface_type_values if surface_type == "BrepSurfaceNurbAPI")
    
    u_orders = brep_array.GetAttribute("brep:surface:nurb:uOrder").Get() or []
    v_orders = brep_array.GetAttribute("brep:surface:nurb:vOrder").Get() or []
    u_vertex_counts = brep_array.GetAttribute("brep:surface:nurb:uVertexCount").Get() or []
    v_vertex_counts = brep_array.GetAttribute("brep:surface:nurb:vVertexCount").Get() or []
    
    # BA_445: U knot vector size validation
    expected_count_u = u_orders[surface_idx] + u_vertex_counts[surface_idx]
    # BA_450: V knot vector size validation  
    expected_count_v = v_orders[surface_idx] + v_vertex_counts[surface_idx]
    
    # BA_455: U knot ordering (non-decreasing)
    uKnots_slice = uKnots[u_offset:u_offset + expected_count_u]
    if any(y < (x - BrepConstants.NUMERICAL_TOLERANCE) for x, y in zip(uKnots_slice, uKnots_slice[1:])):
        self._AddFailedCheck(requirement=cap.BrepArrayRequirements.BA_455, ...)
        
    # BA_460: V knot ordering (non-decreasing)
    vKnots_slice = vKnots[v_offset:v_offset + expected_count_v]
    if any(y < (x - BrepConstants.NUMERICAL_TOLERANCE) for x, y in zip(vKnots_slice, vKnots_slice[1:])):
        self._AddFailedCheck(requirement=cap.BrepArrayRequirements.BA_460, ...)
```

**Surface Requirements:**
- **BA_420**: Surface array size = count of `"BrepSurfaceNurbAPI"` entities
- **BA_425**: U and V orders positive and bounded
- **BA_430**: Orders less than or equal to vertex counts
- **BA_435**: Control vertices and weights array size consistency
- **BA_440**: Weight values positive for rational surfaces
- **BA_445**: U knot vector size = `uOrder + uVertexCount` per surface
- **BA_450**: V knot vector size = `vOrder + vVertexCount` per surface
- **BA_455**: U knot vector non-decreasing order
- **BA_460**: V knot vector non-decreasing order
- **BA_465**: Surface NURBS control point containment
- **BA_470**: Surface schema consistency

### Performance Considerations
- **Efficient partitioning**: Hierarchical brep offset computation enables fast multi-brep validation
- **Conditional validation**: Smart validation flow that skips unnecessary checks based on data presence
- **Robust error handling**: Graceful handling of malformed data with comprehensive bounds checking
- **Memory efficiency**: Processes large models with complex NURBS geometry without excessive memory usage
- **Optimized algorithms**: Specialized validation algorithms for NURBS mathematics and topology relationships
- **Scalable architecture**: Handles models from simple empty breps to complex multi-brep assemblies

## Troubleshooting

### Common Issues

**Import Error: `pxr` module not found**

Configure the USD/omniSolid environment as described in [Environment Setup](#environment-setup):

- `PYTHONPATH` includes USD `lib/python` (`usd_validation_nvidia` comes from the pip install)
- `PATH`/`LD_LIBRARY_PATH` include USD `bin`/`lib`
- `USD_ROOT` points to the USD install
- `OMNISOLID_PLUGIN_PATH` points to the omniSolid plugin resources

**USD Installation Not Found**

Ensure USD is built in `_build/target-deps/usd/release/`.

**PowerShell Execution Policy Error**

Allow scripts for this session, then dot-source the setup script from the repository root so its environment variables stay set:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
. .\tools\brep_validator_cli\setup_env.ps1
```

### Debug Information
For detailed validation output, the validator provides:
- Specific requirement violations (BA.XXX codes) with standardized terminology
- Array indices where violations occur with precise context
- Contextual information about validation failures with improved clarity
- **Enhanced surface validation**: Direction-specific error reporting for U vs V knot vector issues (BA_445-BA_460)
- **Consistent requirement naming**: All error messages use standardized naming patterns for improved readability

## Contributing

When modifying the validator:

### LLM Agent Development Guidelines

```python
# Template for adding new validation methods:
def _validate_NEW_COMPONENT_arrays(self, brep_array: Usd.Prim) -> None:
    """Validate NEW_COMPONENT attributes for BrepArray."""
    # 1. Define required attributes
    attributes_to_check = ["component:attr1", "component:attr2"]
    
    # 2. Calculate expected size using existing patterns
    expected_size = self._calculate_component_count(brep_array)
    
    # 3. Apply standard validations
    self._validate_array_sizes_and_authored(brep_array, attributes_to_check, 
                                           cap.BrepArrayRequirements.BA_XXX, expected_size)
    
    # 4. Add component-specific validations
    self._validate_allowed_tokens(brep_array, "component:type", 
                                 ["validToken1", "validToken2"], 
                                 cap.BrepArrayRequirements.BA_YYY)

# Requirement definition pattern:
BA_XXX = (
    "BA.XXX",                                    # ID string
    "brep-component-validation-name",            # URL suffix (must start with "brep-")
    "Human-readable requirement description.",    # Description
    "capabilities/visualization/brep/requirements/brep-component-validation-name.html",  # Full URL
    "Core USD",                                  # Category
    ("correctness",),                            # Tags
)
```

**Development Rules:**
1. **Requirement numbering**: BA_000 through BA_555; core topology/geometry use increments of 5, analytic surface/curve requirements use increments of 1
2. **Consistent labeling**: Use correct `cap.BrepArrayRequirements.BA_XXX` in all `_AddFailedCheck()` calls
3. **URL patterns**: All URLs start with `"brep-"` prefix
4. **Data type validation**: Use `_validate_stratum_data_types()` for USD schema type checking
5. **Manifold validation**: Use `_compute_brep_offsets()` partitions for per-brep checks
6. **Error reporting**: `self._AddFailedCheck(requirement=BA_XXX, message="...", at=brep_array)`
7. **Attribute access**: Use `BrepConstants.safe_get_attribute()` for robustness
8. **Numerical tolerance**: Use `BrepConstants.NUMERICAL_TOLERANCE = 1e-10` for comparisons
9. **Array validation**: Follow `_validate_array_sizes_and_authored()` pattern
10. **Token validation**: Use `_validate_allowed_tokens()` for enum constraints
11. **Method naming**: `_validate_COMPONENT_arrays()` pattern for consistency

## References

- [USD Documentation](https://openusd.org/)
- [BrepArray Schema Specification](../../docs/brep-array-schema.md) *(if available)*
- [Asset Validator Framework](https://docs.omniverse.nvidia.com/asset-validator/)
