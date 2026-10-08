# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import math
import threading
from enum import Enum

try:
    from pxr import Sdf, Usd
except ImportError as e:
    msg = (
        "Failed to import required `pxr` modules (`Usd`, `Sdf`). "
        "Ensure USD is on PYTHONPATH/PATH (or LD_LIBRARY_PATH) per the Environment Setup."
    )
    raise ImportError(msg) from e

# Capabilities and Asset Validator (prefer public APIs, guarded fallback to private)
try:
    import omni.capabilities as cap
except ImportError as e:
    raise ImportError(
        'Failed to import `omni.capabilities`. Install the asset validator with: pip install "usd-validation-nvidia>=1.22.0,<2"'
    ) from e

try:
    from usd_validation_nvidia import BaseRuleChecker, Suggestion, register_requirements
except ImportError as e:
    raise ImportError(
        'Failed to import `usd_validation_nvidia`. Install it with: pip install "usd-validation-nvidia>=1.22.0,<2"'
    ) from e

__all__ = ["BrepValidator"]  # Add BrepValidator to the public API of this file

from omni.capabilities import Requirement

# Track one-time OmniSolid registration to avoid repeated work per checker
_omnisolid_registered = False
_managed_event_loop = None


def _ensure_event_loop() -> None:
    """
    Ensure an asyncio event loop exists for contexts where the checker is run
    inside threadpools (common in Asset Validator).
    """
    try:
        import asyncio
    except ImportError:
        # Do not fail validation if asyncio is unavailable
        return

    global _managed_event_loop

    try:
        asyncio.get_running_loop()
        return
    except RuntimeError:
        # No running loop in this thread, so create one explicitly.
        try:
            if _managed_event_loop is None or _managed_event_loop.is_closed():
                _managed_event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(_managed_event_loop)
        except Exception as e:
            print(f"WARNING: Failed to ensure asyncio event loop: {type(e).__name__}: {e}")
    except Exception as e:
        # Unexpected issue: keep going but surface diagnostics
        print(f"WARNING: Failed to ensure asyncio event loop: {type(e).__name__}: {e}")

class BrepArrayRequirements(Requirement, Enum):
    # BA_000: Consistent brep attribute sizes
    BA_000 = (
        "BA.000",
        "brep-array-consistent-sizes",
        "The brep:intersectTol3d, and brep:regionCount attribute sizes must all equal the number of breps. The brep:extent size must equal 2 * number of breps.",
        "capabilities/visualization/brep/requirements/brep-array-consistent-sizes.html",
        "Core USD",
        ("correctness",),
    )
    # BA_005: BREP schema attributes authorship
    BA_005 = (
        "BA.005",
        "brep-schema-attributes-authored",
        "All brep schema attributes (brep:intersectTol3d, brep:extent, brep:regionCount) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/brep-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_010: Positive brep:intersectTol3d values
    BA_010 = (
        "BA.010",
        "brep-intersect-tol3d-positive-values",
        "The brep:intersectTol3d attribute values must be positive.",
        "capabilities/visualization/brep/requirements/brep-intersect-tol3d-positive-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_020: Valid brep:extent structure  
    BA_020 = (
        "BA.020",
        "brep-extent-valid-structure",
        "The brep:extent attribute must contain exactly 2 * number of Breps elements, representing bounding box corner pairs (XYZmin, XYZmax) for each brep.",
        "capabilities/visualization/brep/requirements/brep-extent-valid-structure.html",
        "Core USD", 
        ("correctness",),
    )
    # BA_025: Valid brep:extent X order
    BA_025 = (
        "BA.025",
        "brep-extent-x-valid-order",
        "Each brep:extent bounding box must have Xmin <= Xmax.",
        "capabilities/visualization/brep/requirements/brep-extent-x-valid-order.html",
        "Core USD",
        ("correctness",),
    )
    # BA_030: Valid brep:extent Y order
    BA_030 = (
        "BA.030",
        "brep-extent-y-valid-order",
        "Each brep:extent bounding box must have Ymin <= Ymax.",
        "capabilities/visualization/brep/requirements/brep-extent-y-valid-order.html",
        "Core USD",
        ("correctness",),
    )
    # BA_035: Valid brep:extent Z order
    BA_035 = (
        "BA.035",
        "brep-extent-z-valid-order",
        "Each brep:extent bounding box must have Zmin <= Zmax.",
        "capabilities/visualization/brep/requirements/brep-extent-z-valid-order.html",
        "Core USD",
        ("correctness",),
    )
    # BA_040: Valid brep:extent X containment
    BA_040 = (
        "BA.040",
        "brep-extent-x-containment",
        "Each brep:extent bounding box must be contained within the prim's X extent.",
        "capabilities/visualization/brep/requirements/brep-extent-x-containment.html",
        "Core USD",
        ("correctness",),
    )
    # BA_045: Valid brep:extent Y containment
    BA_045 = (
        "BA.045",
        "brep-extent-y-containment",
        "Each brep:extent bounding box must be contained within the prim's Y extent.",
        "capabilities/visualization/brep/requirements/brep-extent-y-containment.html",
        "Core USD",
        ("correctness",),
    )
    # BA_050: Valid brep:extent Z containment
    BA_050 = (
        "BA.050",
        "brep-extent-z-containment",
        "Each brep:extent bounding box must be contained within the prim's Z extent.",
        "capabilities/visualization/brep/requirements/brep-extent-z-containment.html",
        "Core USD",
        ("correctness",),
    )
    # BA_061: Brep attribute data types
    BA_061 = (
        "BA.061",
        "brep-attribute-data-types",
        "Brep attributes must use correct USD data types: brep:intersectTol3d (double[]), brep:extent (double3[]), brep:regionCount (uint[]).",
        "capabilities/visualization/brep/requirements/brep-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_065: Correct region array sizes
    BA_065 = (
        "BA.065",
        "brep-region-array-size-correctness",
        "The region:shellCount, and region:type attributes must have size equal to the sum of all entries in brep:regionCount.",
        "capabilities/visualization/brep/requirements/brep-region-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_070: Region schema attributes authorship
    BA_070 = (
        "BA.070", 
        "region-schema-attributes-authored",
        "All region schema attributes (region:shellCount, region:type) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/region-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_075: Valid region:type token values
    BA_075 = (
        "BA.075",
        "brep-region-type-valid-tokens",
        "The region:type attribute values must be one of: solidRegion, voidRegion.",
        "capabilities/visualization/brep/requirements/brep-region-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_076: Region attribute data types
    BA_076 = (
        "BA.076",
        "region-attribute-data-types",
        "Region attributes must use correct USD data types: region:shellCount (uint[]), region:type (token[]).",
        "capabilities/visualization/brep/requirements/region-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_080: Correct shell array sizes
    BA_080 = (
        "BA.080",
        "brep-shell-array-size-correctness",
        "The shell:faceuseCount, shell:wireEdgeCount, and shell:pointType attribute sizes must equal the sum of all region:shellCount entry values.",
        "capabilities/visualization/brep/requirements/brep-shell-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_085: Shell schema attributes authorship
    BA_085 = (
        "BA.085",
        "shell-schema-attributes-authored", 
        "All shell schema attributes (shell:faceuseCount, shell:wireEdgeCount, shell:pointType) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/shell-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_090: Valid shell:pointType token values
    BA_090 = (
        "BA.090",
        "brep-shell-type-valid-tokens",
        "The shell:pointType attribute values must be one of: BrepPointAPI, none.",
        "capabilities/visualization/brep/requirements/brep-shell-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_091: Shell attribute data types
    BA_091 = (
        "BA.091",
        "shell-attribute-data-types",
        "Shell attributes must use correct USD data types: shell:faceuseCount (uint[]), shell:wireEdgeCount (uint[]), shell:pointType (token[]).",
        "capabilities/visualization/brep/requirements/shell-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_100: Correct faceuse array sizes
    BA_100 = (
        "BA.100",
        "brep-faceuse-array-size-correctness",
        "The faceuse:faceIndex and faceuse:orientationType attribute sizes must equal the sum of all shell:faceuseCount entry values.",
        "capabilities/visualization/brep/requirements/brep-faceuse-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_105: Faceuse schema attributes authorship
    BA_105 = (
        "BA.105",
        "faceuse-schema-attributes-authored",
        "All faceuse schema attributes (faceuse:faceIndex, faceuse:orientationType) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/faceuse-schema-attributes-authored.html", 
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_110: Valid faceuse:orientationType token values
    BA_110 = (
        "BA.110",
        "brep-faceuse-type-valid-tokens",
        "The faceuse:orientationType attribute values must be one of: same, opposite.",
        "capabilities/visualization/brep/requirements/brep-faceuse-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_115: Valid faceuse:faceIndex index values
    BA_115 = (
        "BA.115",
        "brep-faceuse-faceindex-valid-values",
        "The values in faceuse:faceIndex corresponding to Brep_ii must reference valid face indices corresponding to Brep_ii.",
        "capabilities/visualization/brep/requirements/brep-faceuse-faceindex-valid-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_116: Faceuse attribute data types
    BA_116 = (
        "BA.116",
        "faceuse-attribute-data-types",
        "Faceuse attributes must use correct USD data types: faceuse:faceIndex (uint[]), faceuse:orientationType (token[]).",
        "capabilities/visualization/brep/requirements/faceuse-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_120: Correct face array sizes
    BA_120 = (
        "BA.120",
        "brep-face-array-size-correctness",
        "The face:loopCount, face:surfaceType, face:trimType, and face:range attribute sizes must equal half of the faceuse:faceIndex size.",
        "capabilities/visualization/brep/requirements/brep-face-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_125: Face schema attributes authorship
    BA_125 = (
        "BA.125",
        "face-schema-attributes-authored",
        "All face schema attributes (face:loopCount, face:trimType, face:surfaceType, face:range) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/face-schema-attributes-authored.html",
        "Core USD", 
        ("correctness", "authorship"),
    )
    # BA_130: Valid face:surfaceType token values
    BA_130 = (
        "BA.130",
        "brep-face-surface-type-valid-tokens",
        "The face:surfaceType attribute values must be one of: BrepSurfaceNurbAPI, BrepSurfaceSphereAPI, BrepSurfacePlaneAPI, BrepSurfaceCylinderAPI, BrepSurfaceConeAPI, BrepSurfaceTorusAPI.",
        "capabilities/visualization/brep/requirements/brep-face-surface-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_135: Valid face:trimType token values
    BA_135 = (
        "BA.135",
        "brep-face-trim-type-valid-tokens",
        "The face:trimType attribute values must be one of: rectangular, general.",
        "capabilities/visualization/brep/requirements/brep-face-trim-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_140: Valid face:loopCount minimum values
    BA_140 = (
        "BA.140",
        "brep-face-loop-count-minimum-values",
        "Each entry in face:loopCount must be at least one.",
        "capabilities/visualization/brep/requirements/brep-face-loop-count-minimum-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_145: Valid face:range structure
    BA_145 = (
        "BA.145",
        "brep-face-range-valid-structure",
        "The face:range attribute values must contain exactly 2 elements per range (UV min and max pairs).",
        "capabilities/visualization/brep/requirements/brep-face-range-valid-structure.html",
        "Core USD",
        ("correctness",),
    )
    # BA_150: Valid face:range size
    BA_150 = (
        "BA.150",
        "brep-face-range-size-correctness",
        "The face:range attribute size must equal 2 * number of faces (UV min/max pairs).",
        "capabilities/visualization/brep/requirements/brep-face-range-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_155: Valid face:range U nondegeneracy
    BA_155 = (
        "BA.155",
        "brep-face-range-u-nondegeneracy",
        "The U range values in face:range must be ordered as (Umin, Umax) where Umax > Umin.",
        "capabilities/visualization/brep/requirements/brep-face-range-u-nondegeneracy.html",
        "Core USD",
        ("correctness",),
    )
    # BA_160: Valid face:range V nondegeneracy
    BA_160 = (
        "BA.160",
        "brep-face-range-v-nondegeneracy",
        "The V range values in face:range must be ordered as (Vmin, Vmax) where Vmax > Vmin.",
        "capabilities/visualization/brep/requirements/brep-face-range-v-nondegeneracy.html",
        "Core USD",
        ("correctness",),
    )
    # BA_161: Face attribute data types
    BA_161 = (
        "BA.161",
        "face-attribute-data-types",
        "Face attributes must use correct USD data types: face:loopCount (uint[]), face:trimType (token[]), face:surfaceType (token[]), face:range (double2[]).",
        "capabilities/visualization/brep/requirements/face-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_165: Correct loop array sizes
    BA_165 = (
        "BA.165",
        "brep-loop-array-size-correctness",
        "The loop:edgeuseCount, and loop:vertexIndex attribute sizes must equal the sum of all face:loopCount entry values.",
        "capabilities/visualization/brep/requirements/brep-loop-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_170: Loop schema attributes authorship
    BA_170 = (
        "BA.170",
        "loop-schema-attributes-authored",
        "All loop schema attributes (loop:edgeuseCount, loop:vertexIndex) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/loop-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_175: Valid loop:vertexIndex index values
    BA_175 = (
        "BA.175",
        "brep-loop-vertex-reference-when-no-edges",
        "When loop:edgeuseCount is zero, the corresponding loop:vertexIndex must reference a valid vertex index within the same brep.",
        "capabilities/visualization/brep/requirements/brep-loop-vertex-reference-when-no-edges.html",
        "Core USD",
        ("correctness",),
    )
    # BA_176: Loop attribute data types
    BA_176 = (
        "BA.176",
        "loop-attribute-data-types",
        "Loop attributes must use correct USD data types: loop:edgeuseCount (uint[]), loop:vertexIndex (uint[]).",
        "capabilities/visualization/brep/requirements/loop-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_180: Valid edgeuse array sizes
    BA_180 = (
        "BA.180",
        "brep-edgeuse-array-size-correctness",
        "The edgeuse:edgeIndex, edgeuse:orientationType, edgeuse:nextRadialEUIndex, and edgeuse:thisRadialEntryType attribute sizes must equal the sum of all loop:edgeuseCount entry values.",
        "capabilities/visualization/brep/requirements/brep-edgeuse-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_185: Edgeuse schema attributes authorship
    BA_185 = (
        "BA.185", 
        "edgeuse-schema-attributes-authored",
        "All edgeuse schema attributes (edgeuse:edgeIndex, edgeuse:orientationType, edgeuse:nextRadialEUIndex, edgeuse:thisRadialEntryType) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/edgeuse-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_190: Valid edgeuse:orientationType token values
    BA_190 = (
        "BA.190",
        "brep-edgeuse-orientation-type-valid-tokens",
        "The edgeuse:orientationType attribute values must be one of: same, opposite.",
        "capabilities/visualization/brep/requirements/brep-edgeuse-orientation-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_195: Valid edgeuse:thisRadialEntryType token values
    BA_195 = (
        "BA.195",
        "brep-edgeuse-this-radial-entry-type-valid-tokens",
        "The edgeuse:thisRadialEntryType attribute values must be one of: topEntry, bottomEntry.",
        "capabilities/visualization/brep/requirements/brep-edgeuse-this-radial-entry-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_196: Edgeuse attribute data types
    BA_196 = (
        "BA.196",
        "edgeuse-attribute-data-types",
        "Edgeuse attributes must use correct USD data types: edgeuse:edgeIndex (uint[]), edgeuse:orientationType (token[]), edgeuse:nextRadialEUIndex (uint[]), edgeuse:thisRadialEntryType (token[]).",
        "capabilities/visualization/brep/requirements/edgeuse-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_200: Valid edgeuse:nextRadialEUIndex index values
    BA_200 = (
        "BA.200",
        "brep-edgeuse-nextradialeuindex-valid-values",
        "The values in edgeuse:nextRadialEUIndex corresponding to Brep_ii must reference entries in edgeuse:edgeIndex corresponding to Brep_ii.",
        "capabilities/visualization/brep/requirements/brep-edgeuse-nextradialeuindex-valid-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_205: Valid edgeuse:edgeIndex index values
    BA_205 = (
        "BA.205",
        "brep-edgeuse-edgeindex-valid-values",
        "The values in edgeuse:edgeIndex corresponding to Brep_ii must reference valid edge indices corresponding to Brep_ii.",
        "capabilities/visualization/brep/requirements/brep-edgeuse-edgeindex-valid-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_210: Consistent edge array sizes
    BA_210 = (
        "BA.210",
        "brep-edge-array-size-correctness",
        "The edge:curveType, and edge:vertexIndices attribute sizes must equal the total number of edges across all breps. The edge:range attribute size must equal 2 * total number of edges (flat array with 2 values per edge).",
        "capabilities/visualization/brep/requirements/brep-edge-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_215: Edge schema attributes authorship
    BA_215 = (
        "BA.215",
        "edge-schema-attributes-authored",
        "All edge schema attributes (edge:curveType, edge:vertexIndices, edge:range) must be authored in BrepArray prims.",
        "capabilities/visualization/brep/requirements/edge-schema-attributes-authored.html", 
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_225: Valid edge:vertexIndices index values
    BA_225 = (
        "BA.225",
        "brep-edge-vertex-indices-valid-values",
        "The values in edge:vertexIndices corresponding to Brep_ii must reference valid vertex indices corresponding to Brep_ii.",
        "capabilities/visualization/brep/requirements/brep-edge-vertex-indices-valid-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_230: Valid edge:range per-edge structure
    BA_230 = (
        "BA.230",
        "brep-edge-range-per-edge-structure",
        "The edge:range attribute must contain exactly 2 elements per edge, where each edge has consecutive (min, max) parameter range values.",
        "capabilities/visualization/brep/requirements/brep-edge-range-per-edge-structure.html",
        "Core USD",
        ("correctness",),
    )
    # BA_235: Valid edge:range order
    BA_235 = (
        "BA.235",
        "brep-edge-range-valid-order",
        "Each consecutive pair of values in the edge:range flat array must be ordered as (min, max) where min <= max.",
        "capabilities/visualization/brep/requirements/brep-edge-range-valid-order.html",
        "Core USD",
        ("correctness",),
    )
    # BA_237: Edge attribute data types
    BA_237 = (
        "BA.237",
        "edge-attribute-data-types",
        "Edge attributes must use correct USD data types: edge:curveType (token[]), edge:vertexIndices (int2[]), edge:range (double[]).",
        "capabilities/visualization/brep/requirements/edge-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_245: Edge curve type token validation
    BA_245 = (
        "BA.245",
        "edge-curve-type-valid-tokens",
        "The edge:curveType attribute values must be one of: BrepCurve3dNurbAPI, BrepCurve3dCircleAPI, BrepCurve3dLineAPI, BrepCurve3dEllipseAPI.",
        "capabilities/visualization/brep/requirements/edge-curve-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_250: Correct wireEdge array sizes
    BA_250 = (
        "BA.250",
        "brep-wire-edge-array-size-correctness",
        "The wireEdge:curveType, and wireEdge:vertexIndices attribute sizes must equal the sum of all shell:wireEdgeCount entry values. The wireEdge:range attribute size must equal 2 * sum of all shell:wireEdgeCount entry values (flat array with 2 values per wire edge).",
        "capabilities/visualization/brep/requirements/brep-wire-edge-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_255: WireEdge schema attributes authorship
    BA_255 = (
        "BA.255",
        "wireedge-schema-attributes-authored", 
        "When shell:wireEdgeCount is zero, wireEdge:curveType, wireEdge:vertexIndices, and wireEdge:range may all be omitted or authored together as empty arrays. When any wire edges exist, all three must be authored. If any of the three is authored, all three must be authored.",
        "capabilities/visualization/brep/requirements/wireedge-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_260: Valid wireEdge:curveType token values
    BA_260 = (
        "BA.260",
        "brep-wire-edge-curve-type-valid-tokens",
        "The wireEdge:curveType attribute values must be one of: BrepCurve3dNurbAPI, BrepCurve3dCircleAPI, BrepCurve3dLineAPI, BrepCurve3dEllipseAPI.",
        "capabilities/visualization/brep/requirements/brep-wire-edge-curve-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_265: Valid wireEdge:vertexIndices index values
    BA_265 = (
        "BA.265",
        "brep-wire-edge-vertex-indices-valid-values",
        "The values in wireEdge:vertexIndices corresponding to Brep_ii must reference valid vertex indices corresponding to Brep_ii.",
        "capabilities/visualization/brep/requirements/brep-wire-edge-vertex-indices-valid-values.html",
        "Core USD",
        ("correctness",),
    )
    # BA_270: Valid wireEdge:range per-edge structure
    BA_270 = (
        "BA.270",
        "brep-wire-edge-range-per-edge-structure",
        "The wireEdge:range attribute must contain exactly 2 elements per wire edge, where each wire edge has consecutive (min, max) parameter range values.",
        "capabilities/visualization/brep/requirements/brep-wire-edge-range-per-edge-structure.html",
        "Core USD",
        ("correctness",),
    )
    # BA_275: Valid wireEdge:range order
    BA_275 = (
        "BA.275",
        "brep-wire-edge-range-valid-order",
        "Each consecutive pair of values in the wireEdge:range flat array must be ordered as (min, max) where min <= max.",
        "capabilities/visualization/brep/requirements/brep-wire-edge-range-valid-order.html",
        "Core USD",
        ("correctness",),
    )
    # BA_290: WireEdge 3D NURBS Schema Consistency
    BA_290 = (
        "BA.290",
        "brep-wireEdge3d-nurbs-schema-usage-consistency",
        "When 'BrepCurve3dNurbAPI:wireEdge3dNurb' appears in apiSchemas, all wireEdges with wireEdge:curveType='BrepCurve3dNurbAPI' must have corresponding brep:wireEdge3dNurb:curve3d:nurb NURBS data authored.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurbs-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )
    # BA_291: WireEdge attribute data types
    BA_291 = (
        "BA.291",
        "wireEdge-attribute-data-types",
        "WireEdge attributes must use correct USD data types: wireEdge:curveType (token[]), wireEdge:vertexIndices (int2[]), wireEdge:range (double[]).",
        "capabilities/visualization/brep/requirements/wireEdge-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    
    # BA_295: Consistent vertex array sizes
    BA_295 = (
        "BA.295",
        "brep-vertex-array-size-correctness",
        "The vertex:pointType attribute sizes must equal the total number of vertices across all breps.",
        "capabilities/visualization/brep/requirements/brep-vertex-array-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_300: Vertex schema attributes authorship
    BA_300 = (
        "BA.300",
        "vertex-schema-attributes-authored",
        "All vertex schema attributes (vertex:pointType) must be authored in BrepArray prims.", 
        "capabilities/visualization/brep/requirements/vertex-schema-attributes-authored.html",
        "Core USD",
        ("correctness", "authorship"),
    )
    # BA_305: Vertex Point Schema Consistency
    BA_305 = (
        "BA.305",
        "brep-vertex-point-schema-usage-consistency",
        "When 'BrepPointAPI:vertexPoint' appears in apiSchemas, vertices with vertex:pointType='BrepPointAPI' must have corresponding brep:vertexPoint:point:position data authored.",
        "capabilities/visualization/brep/requirements/brep-vertex-point-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )
    # BA_310: Vertex Position Containment
    BA_310 = (
        "BA.310",
        "brep-vertex-position-extent-containment",
        "All brep:vertexPoint:point:position values must lie within their corresponding brep:extent bounding box, accounting for numerical tolerance.",
        "capabilities/visualization/brep/requirements/brep-vertex-position-extent-containment.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )
    # BA_315: Valid vertex:pointType token values
    BA_315 = (
        "BA.315",
        "brep-vertex-point-type-valid-tokens",
        "The vertex:pointType attribute values must be one of: BrepPointAPI.",
        "capabilities/visualization/brep/requirements/brep-vertex-point-type-valid-tokens.html",
        "Core USD",
        ("correctness",),
    )
    # BA_316: Vertex attribute data types
    BA_316 = (
        "BA.316",
        "vertex-attribute-data-types",
        "Vertex attributes must use correct USD data types: vertex:pointType (token[]).",
        "capabilities/visualization/brep/requirements/vertex-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_320: brep:vertexPoint:point:position size correct
    BA_320 = (
        "BA.320",
        "brep-vertex-point-position-size-correctness",
        "The brep:vertexPoint:point:position attribute size must equal the count of the corresponding vertex:pointType entries.",
        "capabilities/visualization/brep/requirements/brep-vertex-point-position-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_325: brep:shellPoint:point:position size correct
    BA_325 = (
        "BA.325",
        "brep-shell-point-position-size-correctness",
        "The brep:shellPoint:point:position attribute size must equal the number of point shells: shells whose "
        "faceuseCount and wireEdgeCount are both zero and whose pointType is BrepPointAPI.",
        "capabilities/visualization/brep/requirements/brep-shell-point-position-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_326: VertexPoint attribute data types
    BA_326 = (
        "BA.326",
        "vertexPoint-attribute-data-types",
        "VertexPoint attributes must use correct USD data types: brep:vertexPoint:point:position (point3d[]).",
        "capabilities/visualization/brep/requirements/vertexPoint-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_327: ShellPoint attribute data types
    BA_327 = (
        "BA.327",
        "shellPoint-attribute-data-types",
        "ShellPoint attributes must use correct USD data types: brep:shellPoint:point:position (point3d[]).",
        "capabilities/visualization/brep/requirements/shellPoint-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_330: brep:edge3dNurb:curve3d:nurb:order size correct
    BA_330 = (
        "BA.330",
        "brep-curve3d-nurb-order-size-correctness",
        "The brep:edge3dNurb:curve3d:nurb:order attribute size must equal the count of BrepCurve3dNurbAPI entities in edge:curveType.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-order-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_335: brep:edge3dNurb:curve3d:nurb:order positive
    BA_335 = (
        "BA.335",
        "brep-curve3d-nurb-order-positive",
        "The brep:edge3dNurb:curve3d:nurb:order attribute values must be positive.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-order-positive.html",
        "Core USD",
        ("correctness",),
    )
    # BA_340: brep:edge3dNurb:curve3d:nurb:order bounded by brep:edge3dNurb:curve3d:nurb:vertexCount
    BA_340 = (
        "BA.340",
        "brep-curve3d-nurb-order-bounded-by-vertexCount",
        "The brep:edge3dNurb:curve3d:nurb:order attribute must not exceed brep:edge3dNurb:curve3d:nurb:vertexCount.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-order-bounded-by-vertexCount.html",
        "Core USD",
        ("correctness",),
    )
    # BA_345: brep:edge3dNurb:curve3d:nurb:weights and brep:edge3dNurb:curve3d:nurb:controlVertices correct size
    BA_345 = (
        "BA.345",
        "brep-curve3d-nurb-weights-size-correctness",
        "The brep:edge3dNurb:curve3d:nurb:weights and brep:edge3dNurb:curve3d:nurb:controlVertices attribute sizes must equal the sum of all brep:edge3dNurb:curve3d:nurb:vertexCount entry values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-weights-size-correctness.html",
        "Core USD",
        ("correctness",),
    )    
    # BA_350: brep:edge3dNurb:curve3d:nurb:weights positive
    BA_350 = (
        "BA.350",
        "brep-curve3d-nurb-weights-positive",
        "The brep:edge3dNurb:curve3d:nurb:weights attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-weights-positive.html",
        "Core USD",
        ("correctness",),
    )   
    # BA_355: brep:edge3dNurb:curve3d:nurb:knots correct size
    BA_355 = (
        "BA.355",
        "brep-curve3d-nurb-knots-size-correctness",
        "The brep:edge3dNurb:curve3d:nurb:knots attribute must have size equal to the sum of brep:edge3dNurb:curve3d:nurb:order and brep:edge3dNurb:curve3d:nurb:vertexCount.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-knots-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_360: brep:edge3dNurb:curve3d:nurb:knots correct ordering
    BA_360 = (
        "BA.360",
        "brep-curve3d-nurb-knots-ordering",
        "The brep:edge3dNurb:curve3d:nurb:knots attribute values must be non-decreasing for each knot vector.",
        "capabilities/visualization/brep/requirements/brep-curve3d-nurb-knots-authored-and-valid.html",
        "Core USD",
        ("correctness",),
    )
    # BA_365: Edge NURBS Control Point Containment
    BA_365 = (
        "BA.365",
        "brep-edge3d-nurbs-control-point-extent-containment",
        "All brep:edge3dNurb:curve3d:nurb:controlVertices must lie within their corresponding brep:extent bounding box dimensions.",
        "capabilities/visualization/brep/requirements/brep-edge3d-nurbs-control-point-extent-containment.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )
    # BA_370: Edge NURBS Data Completeness
    BA_370 = (
        "BA.370",
        "brep-edge3d-nurbs-schema-data-consistency",
        "For each edge with edge:curveType='BrepCurve3dNurbAPI', the corresponding brep:edge3dNurb:curve3d:nurb data must be complete: order, vertexCount, controlVertices, weights, and knots must all be consistently authored. When 'BrepCurve3dNurbAPI:edge3dNurb' appears in apiSchemas, all such edges must have corresponding NURBS data.",
        "capabilities/visualization/brep/requirements/brep-edge3d-nurbs-schema-data-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )
    # BA_371: Edge3dNurb attribute data types
    BA_371 = (
        "BA.371",
        "edge3dNurb-attribute-data-types",
        "Edge3dNurb attributes must use correct USD data types: brep:edge3dNurb:curve3d:nurb:order (uint[]), brep:edge3dNurb:curve3d:nurb:vertexCount (uint[]), brep:edge3dNurb:curve3d:nurb:controlVertices (point3d[]), brep:edge3dNurb:curve3d:nurb:weights (double[]), brep:edge3dNurb:curve3d:nurb:knots (double[]).",
        "capabilities/visualization/brep/requirements/edge3dNurb-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_375: brep:curveUv:nurb attribute arrays size consistency
    BA_375 = (
        "BA.375",
        "brep-curveUv-size-correctness",
        "The brep:curveUv:nurb:vertexCount and brep:curveUv:nurb:order attribute sizes must equal the edgeuse:edgeIndex array size.",
        "capabilities/visualization/brep/requirements/brep-curveUv-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_380: brep:curveUv:nurb:order positive or paired missing-curve sentinel
    BA_380 = (
        "BA.380",
        "brep-curveUv-order-positive",
        "The brep:curveUv:nurb:order attribute values must be positive for authored curves. An order and corresponding vertexCount may both be zero to indicate that an edgeuse has no UV trim curve.",
        "capabilities/visualization/brep/requirements/brep-curveUv-order-positive.html",   
        "Core USD",
        ("correctness",),
    )
    # BA_385: brep:curveUv:nurb:order bounded by vertexCount
    BA_385 = (
        "BA.385",
        "brep-curveUv-order-bounded-by-vertexCount",
        "The brep:curveUv:nurb:order attribute values must not exceed the corresponding brep:curveUv:nurb:vertexCount attribute values.",
        "capabilities/visualization/brep/requirements/brep-curveUv-order-bounded-by-vertexCount.html",
        "Core USD",
        ("correctness",),
    )
    # BA_390: brep:curveUv:nurb:controlVertices size correct
    BA_390 = (
        "BA.390",
        "brep-curveUv-controlVertices-size-correctness",
        "The brep:curveUv:nurb:controlVertices attribute size must equal the sum of all brep:curveUv:nurb:vertexCount entry values.",
        "capabilities/visualization/brep/requirements/brep-curveUv-controlVertices-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_395: brep:curveUv:nurb:knots size correct
    BA_395 = (
        "BA.395",
        "brep-curveUv-nurb-knots-size-correctness",
        "The brep:curveUv:nurb:knots attribute must have size equal to the sum of brep:curveUv:nurb:order and brep:curveUv:nurb:vertexCount.",
        "capabilities/visualization/brep/requirements/brep-curveUv-nurb-knots-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_400: brep:curveUv:nurb:knots correct ordering
    BA_400 = (
        "BA.400",
        "brep-curveUv-nurb-knots-ordering",
        "The brep:curveUv:nurb:knots attribute values must be non-decreasing for each knot vector.",
        "capabilities/visualization/brep/requirements/brep-curveUv-valid-and-size-consistency.html",
        "Core USD",
        ("correctness",),  
    )
    # BA_405: brep:curveUv:nurb:weights size correct
    BA_405 = (
        "BA.405",
        "brep-curveUv-nurb-weights-size-correctness",
        "The brep:curveUv:nurb:weights attribute size must equal the sum of all brep:curveUv:nurb:vertexCount entry values.",
        "capabilities/visualization/brep/requirements/brep-curveUv-nurb-weights-size-correctness.html",    
        "Core USD",
        ("correctness",),
    )
    # BA_410: brep:curveUv:nurb:weights positive
    BA_410 = (
        "BA.410",
        "brep-curveUv-nurb-weights-positive",
        "The brep:curveUv:nurb:weights attribute values must be positive.",
        "capabilities/visualization/brep/requirements/brep-curveUv-valid-and-size-consistency.html",
        "Core USD", 
        ("correctness",),
    )
    # BA_415: CurveUv NURBS Schema Consistency
    BA_415 = (
        "BA.415",
        "brep-curveUv-nurbs-schema-usage-consistency",
        "When 'BrepCurveUvNurbAPI' appears in apiSchemas, vertexCount and order must carry one record per edgeuse. A (0, 0) record denotes an edgeuse with no UV trim curve and contributes no packed curve data.",
        "capabilities/visualization/brep/requirements/brep-curveUv-nurbs-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )
    # BA_416: CurveUv attribute data types
    BA_416 = (
        "BA.416",
        "curveUv-attribute-data-types",
        "CurveUv attributes must use correct USD data types: brep:curveUv:nurb:order (uint[]), brep:curveUv:nurb:vertexCount (uint[]), brep:curveUv:nurb:controlVertices (double2[]), brep:curveUv:nurb:weights (double[]), brep:curveUv:nurb:knots (double[]).",
        "capabilities/visualization/brep/requirements/curveUv-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_420: brep:surface:nurb attribute arrays size correct
    BA_420 = (
        "BA.420",   
        "brep-surface-nurb-size-correctness",
        "The brep:surface:nurb:uVertexCount, brep:surface:nurb:vVertexCount, brep:surface:nurb:uOrder, and brep:surface:nurb:vOrder attribute sizes must equal the number of face:surfaceType entries with 'BrepSurfaceNurbAPI' values.",
        "capabilities/visualization/brep/requirements/brep-surface-nurb-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_425: brep:surface:nurb orders positive
    BA_425 = (
        "BA.425",
        "brep-surface-orders-positive",
        "The brep:surface:nurb:uOrder and brep:surface:nurb:vOrder attributes must be positive.",
        "capabilities/visualization/brep/requirements/brep-surface-orders-authored-and-valid.html",
        "Core USD",
        ("correctness",),
    )    
    # BA_430: brep:surface:nurb orders less than or equal to vertex counts
    BA_430 = (
        "BA.430",
        "brep-surface-orders-less-than-or-equal-to-vertex-counts",
        "The brep:surface:nurb:uOrder and brep:surface:nurb:vOrder attributes must not exceed the corresponding vertex counts.",
        "capabilities/visualization/brep/requirements/brep-surface-orders-less-than-or-equal-to-vertex-counts.html",
        "Core USD",
        ("correctness",),
    )    
    # BA_435: brep:surface:nurb control vertices and weights size correct
    BA_435 = (
        "BA.435",
        "brep-surface-nurb-control-vertices-and-weights-size-correctness",
        "The brep:surface:nurb:controlVertices and brep:surface:nurb:weights attribute sizes must equal the sum of all (brep:surface:nurb:uVertexCount * brep:surface:nurb:vVertexCount) values.",
        "capabilities/visualization/brep/requirements/brep-surface-nurb-control-vertices-and-weights-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_440: brep:surface:nurb weights positive
    BA_440 = (
        "BA.440",
        "brep-surface-weights-positive",
        "The brep:surface:nurb:weights attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-surface-weights-size-consistent-and-positive.html",
        "Core USD",
        ("correctness",),
    )   
    # BA_445: brep:surface:nurb uKnots size correct
    BA_445 = (
        "BA.445",
        "brep-surface-nurb-uknots-size-correctness",
        "The brep:surface:nurb:uKnots attribute size must equal the sum of all (brep:surface:nurb:uOrder + brep:surface:nurb:uVertexCount) attribute values.",
        "capabilities/visualization/brep/requirements/brep-surface-nurb-uknots-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_450: brep:surface:nurb vKnots size correct
    BA_450 = (
        "BA.450",
        "brep-surface-nurb-vknots-size-correctness",
        "The brep:surface:nurb:vKnots attribute size must equal the sum of all (brep:surface:nurb:vOrder + brep:surface:nurb:vVertexCount) attribute values.",
        "capabilities/visualization/brep/requirements/brep-surface-nurb-vknots-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_455: brep:surface:nurb uKnots non-decreasing
    BA_455 = (
        "BA.455",
        "brep-surface-nurb-uknots-non-decreasing",
        "The brep:surface:nurb:uKnots attribute values must be non-decreasing for each knot vector.",
        "capabilities/visualization/brep/requirements/brep-surface-nurb-uknots-non-decreasing.html",
        "Core USD",
        ("correctness",),
    )
    # BA_460: brep:surface:nurb vKnots non-decreasing
    BA_460 = (
        "BA.460",
        "brep-surface-nurb-vknots-non-decreasing",
        "The brep:surface:nurb:vKnots attribute values must be non-decreasing for each knot vector.",
        "capabilities/visualization/brep/requirements/brep-surface-nurb-vknots-non-decreasing.html",
        "Core USD",
        ("correctness",),
    )
    # BA_465: Surface NURBS Control Point Containment
    BA_465 = (
        "BA.465",
        "brep-surface-nurbs-control-point-extent-containment", 
        "All brep:surface:nurb:controlVertices must lie within their corresponding brep:extent bounding box dimensions.",
        "capabilities/visualization/brep/requirements/brep-surface-nurbs-control-point-extent-containment.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )
    # BA_470: Surface Schema Consistency
    BA_470 = (
        "BA.470",
        "brep-surface-schema-usage-consistency",
        "When 'BrepSurfaceNurbAPI' appears in apiSchemas, all faces with face:surfaceType='BrepSurfaceNurbAPI' must have corresponding brep:surface:nurb NURBS data authored.",
        "capabilities/visualization/brep/requirements/brep-surface-schema-usage-consistency.html", 
        "Core USD",
        ("correctness", "schema-consistency"),
    )
    # BA_471: Surface attribute data types
    BA_471 = (
        "BA.471",
        "surface-attribute-data-types",
        "Surface attributes must use correct USD data types: brep:surface:nurb:uOrder (uint[]), brep:surface:nurb:vOrder (uint[]), brep:surface:nurb:uVertexCount (uint[]), brep:surface:nurb:vVertexCount (uint[]), brep:surface:nurb:controlVertices (point3d[]), brep:surface:nurb:weights (double[]), brep:surface:nurb:uKnots (double[]), brep:surface:nurb:vKnots (double[]).",
        "capabilities/visualization/brep/requirements/surface-attribute-data-types.html",
        "Core USD",
        ("correctness", "data-types"),
    )
    # BA_480: Sphere surface array size correctness
    BA_480 = (
        "BA.480",
        "brep-surface-sphere-size-correctness",
        "The brep:surface:sphere:center, brep:surface:sphere:axis, brep:surface:sphere:refDirection, and brep:surface:sphere:radius attribute sizes must equal the number of face:surfaceType entries with 'BrepSurfaceSphereAPI' values.",
        "capabilities/visualization/brep/requirements/brep-surface-sphere-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    # BA_481: Sphere surface radius positive
    BA_481 = (
        "BA.481",
        "brep-surface-sphere-radius-positive",
        "The brep:surface:sphere:radius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-surface-sphere-radius-positive.html",
        "Core USD",
        ("correctness",),
    )
    # BA_482: Sphere surface axis unit length
    BA_482 = (
        "BA.482",
        "brep-surface-sphere-axis-unit-length",
        "The brep:surface:sphere:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-sphere-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    # BA_483: Sphere surface refDirection unit length
    BA_483 = (
        "BA.483",
        "brep-surface-sphere-refDirection-unit-length",
        "The brep:surface:sphere:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-sphere-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    # BA_484: Sphere surface axis and refDirection orthogonal
    BA_484 = (
        "BA.484",
        "brep-surface-sphere-axis-refDirection-orthogonal",
        "The brep:surface:sphere:axis and brep:surface:sphere:refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-surface-sphere-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )
    # BA_485: Sphere surface schema consistency
    BA_485 = (
        "BA.485",
        "brep-surface-sphere-schema-usage-consistency",
        "When 'BrepSurfaceSphereAPI' appears in apiSchemas, all faces with face:surfaceType='BrepSurfaceSphereAPI' must have corresponding brep:surface:sphere data authored.",
        "capabilities/visualization/brep/requirements/brep-surface-sphere-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- Plane surface requirements (BA_490-BA_495) ----
    BA_490 = (
        "BA.490",
        "brep-surface-plane-size-correctness",
        "The brep:surface:plane:origin, brep:surface:plane:axis, and brep:surface:plane:refDirection attribute sizes must equal the number of face:surfaceType entries with 'BrepSurfacePlaneAPI' values.",
        "capabilities/visualization/brep/requirements/brep-surface-plane-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_491 = (
        "BA.491",
        "brep-surface-plane-axis-unit-length",
        "The brep:surface:plane:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-plane-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_492 = (
        "BA.492",
        "brep-surface-plane-refDirection-unit-length",
        "The brep:surface:plane:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-plane-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_493 = (
        "BA.493",
        "brep-surface-plane-axis-refDirection-orthogonal",
        "The brep:surface:plane:axis and brep:surface:plane:refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-surface-plane-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )
    BA_495 = (
        "BA.495",
        "brep-surface-plane-schema-usage-consistency",
        "When 'BrepSurfacePlaneAPI' appears in apiSchemas, all faces with face:surfaceType='BrepSurfacePlaneAPI' must have corresponding brep:surface:plane data authored.",
        "capabilities/visualization/brep/requirements/brep-surface-plane-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- Cylinder surface requirements (BA_500-BA_505) ----
    BA_500 = (
        "BA.500",
        "brep-surface-cylinder-size-correctness",
        "The brep:surface:cylinder:origin, brep:surface:cylinder:axis, brep:surface:cylinder:refDirection, and brep:surface:cylinder:radius attribute sizes must equal the number of face:surfaceType entries with 'BrepSurfaceCylinderAPI' values.",
        "capabilities/visualization/brep/requirements/brep-surface-cylinder-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_501 = (
        "BA.501",
        "brep-surface-cylinder-radius-positive",
        "The brep:surface:cylinder:radius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-surface-cylinder-radius-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_502 = (
        "BA.502",
        "brep-surface-cylinder-axis-unit-length",
        "The brep:surface:cylinder:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-cylinder-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_503 = (
        "BA.503",
        "brep-surface-cylinder-refDirection-unit-length",
        "The brep:surface:cylinder:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-cylinder-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_504 = (
        "BA.504",
        "brep-surface-cylinder-axis-refDirection-orthogonal",
        "The brep:surface:cylinder:axis and brep:surface:cylinder:refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-surface-cylinder-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )
    BA_505 = (
        "BA.505",
        "brep-surface-cylinder-schema-usage-consistency",
        "When 'BrepSurfaceCylinderAPI' appears in apiSchemas, all faces with face:surfaceType='BrepSurfaceCylinderAPI' must have corresponding brep:surface:cylinder data authored.",
        "capabilities/visualization/brep/requirements/brep-surface-cylinder-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- Cone surface requirements (BA_510-BA_516) ----
    BA_510 = (
        "BA.510",
        "brep-surface-cone-size-correctness",
        "The brep:surface:cone:origin, brep:surface:cone:axis, brep:surface:cone:refDirection, brep:surface:cone:radius, and brep:surface:cone:semiAngle attribute sizes must equal the number of face:surfaceType entries with 'BrepSurfaceConeAPI' values.",
        "capabilities/visualization/brep/requirements/brep-surface-cone-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_511 = (
        "BA.511",
        "brep-surface-cone-radius-non-negative",
        "The brep:surface:cone:radius attribute must contain only non-negative values.",
        "capabilities/visualization/brep/requirements/brep-surface-cone-radius-non-negative.html",
        "Core USD",
        ("correctness",),
    )
    BA_512 = (
        "BA.512",
        "brep-surface-cone-axis-unit-length",
        "The brep:surface:cone:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-cone-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_513 = (
        "BA.513",
        "brep-surface-cone-refDirection-unit-length",
        "The brep:surface:cone:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-cone-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_514 = (
        "BA.514",
        "brep-surface-cone-axis-refDirection-orthogonal",
        "The brep:surface:cone:axis and brep:surface:cone:refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-surface-cone-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )
    BA_515 = (
        "BA.515",
        "brep-surface-cone-semiAngle-valid-range",
        "The brep:surface:cone:semiAngle attribute must contain values in the range (0, pi/2) exclusive.",
        "capabilities/visualization/brep/requirements/brep-surface-cone-semiAngle-valid-range.html",
        "Core USD",
        ("correctness",),
    )
    BA_516 = (
        "BA.516",
        "brep-surface-cone-schema-usage-consistency",
        "When 'BrepSurfaceConeAPI' appears in apiSchemas, all faces with face:surfaceType='BrepSurfaceConeAPI' must have corresponding brep:surface:cone data authored.",
        "capabilities/visualization/brep/requirements/brep-surface-cone-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- Torus surface requirements (BA_520-BA_526) ----
    BA_520 = (
        "BA.520",
        "brep-surface-torus-size-correctness",
        "The brep:surface:torus:origin, brep:surface:torus:axis, brep:surface:torus:refDirection, brep:surface:torus:majorRadius, and brep:surface:torus:minorRadius attribute sizes must equal the number of face:surfaceType entries with 'BrepSurfaceTorusAPI' values.",
        "capabilities/visualization/brep/requirements/brep-surface-torus-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_521 = (
        "BA.521",
        "brep-surface-torus-majorRadius-positive",
        "The brep:surface:torus:majorRadius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-surface-torus-majorRadius-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_522 = (
        "BA.522",
        "brep-surface-torus-minorRadius-positive",
        "The brep:surface:torus:minorRadius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-surface-torus-minorRadius-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_523 = (
        "BA.523",
        "brep-surface-torus-axis-unit-length",
        "The brep:surface:torus:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-torus-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_524 = (
        "BA.524",
        "brep-surface-torus-refDirection-unit-length",
        "The brep:surface:torus:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-surface-torus-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_525 = (
        "BA.525",
        "brep-surface-torus-axis-refDirection-orthogonal",
        "The brep:surface:torus:axis and brep:surface:torus:refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-surface-torus-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )
    BA_526 = (
        "BA.526",
        "brep-surface-torus-schema-usage-consistency",
        "When 'BrepSurfaceTorusAPI' appears in apiSchemas, all faces with face:surfaceType='BrepSurfaceTorusAPI' must have corresponding brep:surface:torus data authored.",
        "capabilities/visualization/brep/requirements/brep-surface-torus-schema-usage-consistency.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- Circle curve requirements (BA_530-BA_535) ----
    BA_530 = (
        "BA.530",
        "brep-curve3d-circle-size-correctness",
        "The brep:*:curve3d:circle:center, axis, refDirection, and radius attribute sizes must equal the number of corresponding curveType entries with 'BrepCurve3dCircleAPI' values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-circle-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_531 = (
        "BA.531",
        "brep-curve3d-circle-radius-positive",
        "The brep:*:curve3d:circle:radius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-circle-radius-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_532 = (
        "BA.532",
        "brep-curve3d-circle-axis-unit-length",
        "The brep:*:curve3d:circle:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-circle-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_533 = (
        "BA.533",
        "brep-curve3d-circle-refDirection-unit-length",
        "The brep:*:curve3d:circle:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-circle-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_534 = (
        "BA.534",
        "brep-curve3d-circle-axis-refDirection-orthogonal",
        "The brep:*:curve3d:circle:axis and refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-circle-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Line curve requirements (BA_540-BA_541) ----
    BA_540 = (
        "BA.540",
        "brep-curve3d-line-size-correctness",
        "The brep:*:curve3d:line:origin and brep:*:curve3d:line:direction attribute sizes must equal the number of corresponding curveType entries with 'BrepCurve3dLineAPI' values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-line-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_541 = (
        "BA.541",
        "brep-curve3d-line-direction-unit-length",
        "The brep:*:curve3d:line:direction attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-line-direction-unit-length.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Ellipse curve requirements (BA_550-BA_555) ----
    BA_550 = (
        "BA.550",
        "brep-curve3d-ellipse-size-correctness",
        "The brep:*:curve3d:ellipse:center, axis, refDirection, xRadius, and yRadius attribute sizes must equal the number of corresponding curveType entries with 'BrepCurve3dEllipseAPI' values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-ellipse-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_551 = (
        "BA.551",
        "brep-curve3d-ellipse-xRadius-positive",
        "The brep:*:curve3d:ellipse:xRadius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-ellipse-xRadius-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_552 = (
        "BA.552",
        "brep-curve3d-ellipse-yRadius-positive",
        "The brep:*:curve3d:ellipse:yRadius attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-curve3d-ellipse-yRadius-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_553 = (
        "BA.553",
        "brep-curve3d-ellipse-axis-unit-length",
        "The brep:*:curve3d:ellipse:axis attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-ellipse-axis-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_554 = (
        "BA.554",
        "brep-curve3d-ellipse-refDirection-unit-length",
        "The brep:*:curve3d:ellipse:refDirection attribute values must be unit vectors (length approximately 1.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-ellipse-refDirection-unit-length.html",
        "Core USD",
        ("correctness",),
    )
    BA_555 = (
        "BA.555",
        "brep-curve3d-ellipse-axis-refDirection-orthogonal",
        "The brep:*:curve3d:ellipse:axis and refDirection vectors must be orthogonal (dot product approximately 0.0).",
        "capabilities/visualization/brep/requirements/brep-curve3d-ellipse-axis-refDirection-orthogonal.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Analytic domain-range requirements (BA_560-BA_571) ----
    # Per STEP (ISO 10303-42), IGES, and PRC (ISO 14739-1), angular surface
    # parameters are periodic with period 2pi, and sphere latitude is bounded
    # to [-pi/2, pi/2].  These rules validate that face:range and edge:range
    # values respect those parameterization limits.

    # Sphere face UV domain
    BA_560 = (
        "BA.560",
        "brep-face-range-sphere-u-span",
        "For faces with surfaceType 'BrepSurfaceSphereAPI', the U domain span (Umax - Umin) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-face-range-sphere-u-span.html",
        "Core USD",
        ("correctness",),
    )
    BA_561 = (
        "BA.561",
        "brep-face-range-sphere-v-bounds",
        "For faces with surfaceType 'BrepSurfaceSphereAPI', the V domain values must lie within [-pi/2, pi/2] (latitude bounds at the poles).",
        "capabilities/visualization/brep/requirements/brep-face-range-sphere-v-bounds.html",
        "Core USD",
        ("correctness",),
    )

    # Cylinder face UV domain
    BA_562 = (
        "BA.562",
        "brep-face-range-cylinder-u-span",
        "For faces with surfaceType 'BrepSurfaceCylinderAPI', the U domain span (Umax - Umin) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-face-range-cylinder-u-span.html",
        "Core USD",
        ("correctness",),
    )

    # Cone face UV domain
    BA_563 = (
        "BA.563",
        "brep-face-range-cone-u-span",
        "For faces with surfaceType 'BrepSurfaceConeAPI', the U domain span (Umax - Umin) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-face-range-cone-u-span.html",
        "Core USD",
        ("correctness",),
    )

    # Torus face UV domain
    BA_564 = (
        "BA.564",
        "brep-face-range-torus-u-span",
        "For faces with surfaceType 'BrepSurfaceTorusAPI', the U domain span (Umax - Umin) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-face-range-torus-u-span.html",
        "Core USD",
        ("correctness",),
    )
    BA_565 = (
        "BA.565",
        "brep-face-range-torus-v-span",
        "For faces with surfaceType 'BrepSurfaceTorusAPI', the V domain span (Vmax - Vmin) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-face-range-torus-v-span.html",
        "Core USD",
        ("correctness",),
    )

    # Circle edge range domain
    BA_570 = (
        "BA.570",
        "brep-edge-range-circle-span",
        "For edges/wireEdges with curveType 'BrepCurve3dCircleAPI', the parameter span (max - min) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-edge-range-circle-span.html",
        "Core USD",
        ("correctness",),
    )

    # Ellipse edge range domain
    BA_571 = (
        "BA.571",
        "brep-edge-range-ellipse-span",
        "For edges/wireEdges with curveType 'BrepCurve3dEllipseAPI', the parameter span (max - min) must not exceed 2*pi (one full revolution).",
        "capabilities/visualization/brep/requirements/brep-edge-range-ellipse-span.html",
        "Core USD",
        ("correctness",),
    )

    # BA_580: Faceuse pairing — every face must be referenced by exactly 2 faceuses
    BA_580 = (
        "BA.580",
        "brep-faceuse-pairing",
        "Every face index in faceuse:faceIndex must appear exactly 2 times, ensuring each face is bounded by exactly one pair of faceuses.",
        "capabilities/visualization/brep/requirements/brep-faceuse-pairing.html",
        "Core USD",
        ("correctness",),
    )

    # BA_581: Radial edgeuse closure — chasing nextRadialEUIndex must form a closed cycle
    BA_581 = (
        "BA.581",
        "brep-radial-edgeuse-closure",
        "Following edgeuse:nextRadialEUIndex from any edgeuse must form a closed circular chain that returns to the starting edgeuse.",
        "capabilities/visualization/brep/requirements/brep-radial-edgeuse-closure.html",
        "Core USD",
        ("correctness",),
    )

    # BA_582: Orphan edge detection — every edge must be referenced by at least one edgeuse
    BA_582 = (
        "BA.582",
        "brep-orphan-edge-detection",
        "Every edge must be referenced by at least one entry in edgeuse:edgeIndex; edges with no referencing edgeuse are orphans.",
        "capabilities/visualization/brep/requirements/brep-orphan-edge-detection.html",
        "Core USD",
        ("correctness",),
    )

    # BA_583: Geometry type tokens and authored UV data require their applied API schemas
    BA_583 = (
        "BA.583",
        "brep-required-geometry-api-applied",
        "Geometry type-token occurrences must have their corresponding applied geometry API schema. "
        "Authored UV NURBS pcurve data requires BrepCurveUvNurbAPI.",
        "capabilities/visualization/brep/requirements/brep-required-geometry-api-applied.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- NURBS non-negative order / vertexCount requirements (BA_590-BA_593) ----

    BA_590 = (
        "BA.590",
        "brep-nurbs-order-positive",
        "NURBS order values (int[]) must be >= 2 (degree >= 1). Applies to edge, wireEdge, curveUv, and surface orders; curveUv may use (order, vertexCount) = (0, 0) for a missing UV trim curve.",
        "capabilities/visualization/brep/requirements/brep-nurbs-order-positive.html",
        "Core USD",
        ("correctness",),
    )

    BA_591 = (
        "BA.591",
        "brep-nurbs-vertex-count-ge-order",
        "NURBS vertexCount values must be >= the corresponding order. Applies to edge, wireEdge, curveUv, and surface vertex counts.",
        "capabilities/visualization/brep/requirements/brep-nurbs-vertex-count-ge-order.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Edge curve endpoint-to-vertex consistency (BA_600-BA_602) ----

    BA_600 = (
        "BA.600",
        "brep-line-edge-endpoint-vertex-consistency",
        "Line edge endpoints evaluated at range[0] and range[1] must match the edge's vertex positions within tolerance.",
        "capabilities/visualization/brep/requirements/brep-line-edge-endpoint-vertex-consistency.html",
        "Core USD",
        ("correctness",),
    )

    BA_601 = (
        "BA.601",
        "brep-circle-edge-endpoint-vertex-consistency",
        "Circle edge endpoints evaluated at range[0] and range[1] must match the edge's vertex positions within tolerance.",
        "capabilities/visualization/brep/requirements/brep-circle-edge-endpoint-vertex-consistency.html",
        "Core USD",
        ("correctness",),
    )

    BA_602 = (
        "BA.602",
        "brep-ellipse-edge-endpoint-vertex-consistency",
        "Ellipse edge endpoints evaluated at range[0] and range[1] must match the edge's vertex positions within tolerance.",
        "capabilities/visualization/brep/requirements/brep-ellipse-edge-endpoint-vertex-consistency.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Circle radius consistency with vertex positions (BA_610) ----

    BA_610 = (
        "BA.610",
        "brep-circle-vertex-radius-consistency",
        "Circle edge endpoint vertices must lie at distance radius from the circle center within tolerance.",
        "capabilities/visualization/brep/requirements/brep-circle-vertex-radius-consistency.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Analytic surface origin/center containment (BA_620) ----

    BA_620 = (
        "BA.620",
        "brep-analytic-surface-origin-containment",
        "Analytic surface origins/centers must lie within a reasonable expansion of the brep extent.",
        "capabilities/visualization/brep/requirements/brep-analytic-surface-origin-containment.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Edge angular range max in primary period (BA_630) ----

    BA_630 = (
        "BA.630",
        "brep-angular-edge-range-max-primary-period",
        "Circle and ellipse edge range max values should fall in the primary period (0, 2*pi].",
        "capabilities/visualization/brep/requirements/brep-angular-edge-range-max-primary-period.html",
        "Core USD",
        ("correctness",),
    )

    # BA_631 retired: periodic face ranges need not start in a primary interval.

    # ---- Cylinder/cone face V-domain ordering (BA_640) ----

    BA_640 = (
        "BA.640",
        "brep-face-v-domain-ordering",
        "For cylinder and cone faces, face:range V-min must be <= V-max.",
        "capabilities/visualization/brep/requirements/brep-face-v-domain-ordering.html",
        "Core USD",
        ("correctness",),
    )

    # ---- WireEdge NURBS geometry parity (BA_650-BA_658) ----

    BA_650 = (
        "BA.650",
        "brep-wireEdge3d-nurb-order-size-correctness",
        "The brep:wireEdge3dNurb:curve3d:nurb:order and vertexCount attribute sizes must equal the count of BrepCurve3dNurbAPI entities in wireEdge:curveType.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-order-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_651 = (
        "BA.651",
        "brep-wireEdge3d-nurb-order-positive",
        "The brep:wireEdge3dNurb:curve3d:nurb:order attribute values must be positive (>= 2).",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-order-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_652 = (
        "BA.652",
        "brep-wireEdge3d-nurb-order-bounded-by-vertexCount",
        "The brep:wireEdge3dNurb:curve3d:nurb:order must not exceed the corresponding vertexCount.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-order-bounded-by-vertexCount.html",
        "Core USD",
        ("correctness",),
    )
    BA_653 = (
        "BA.653",
        "brep-wireEdge3d-nurb-cv-weights-size-correctness",
        "The brep:wireEdge3dNurb:curve3d:nurb:controlVertices and weights sizes must equal the sum of all vertexCount entries.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-cv-weights-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_654 = (
        "BA.654",
        "brep-wireEdge3d-nurb-weights-positive",
        "The brep:wireEdge3dNurb:curve3d:nurb:weights attribute must contain only positive values.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-weights-positive.html",
        "Core USD",
        ("correctness",),
    )
    BA_655 = (
        "BA.655",
        "brep-wireEdge3d-nurb-knots-size-correctness",
        "The brep:wireEdge3dNurb:curve3d:nurb:knots size must equal the sum of (vertexCount + order) for each wireEdge NURBS curve.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-knots-size-correctness.html",
        "Core USD",
        ("correctness",),
    )
    BA_656 = (
        "BA.656",
        "brep-wireEdge3d-nurb-knots-non-decreasing",
        "The brep:wireEdge3dNurb:curve3d:nurb:knots values must be non-decreasing within each knot vector.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-knots-non-decreasing.html",
        "Core USD",
        ("correctness",),
    )
    BA_657 = (
        "BA.657",
        "brep-wireEdge3d-nurb-control-point-extent-containment",
        "All brep:wireEdge3dNurb:curve3d:nurb:controlVertices must lie within their corresponding brep:extent bounding box.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-control-point-extent-containment.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )
    BA_658 = (
        "BA.658",
        "brep-wireEdge3d-nurb-data-completeness",
        "For each wireEdge with wireEdge:curveType='BrepCurve3dNurbAPI', the corresponding NURBS data must be complete: order, vertexCount, controlVertices, weights, and knots must all be authored.",
        "capabilities/visualization/brep/requirements/brep-wireEdge3d-nurb-data-completeness.html",
        "Core USD",
        ("correctness", "schema-consistency"),
    )

    # ---- NaN / Inf sanity check (BA_660) ----

    BA_660 = (
        "BA.660",
        "brep-float-arrays-finite",
        "All floating-point attribute arrays must contain only finite values (no NaN or Inf).",
        "capabilities/visualization/brep/requirements/brep-float-arrays-finite.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Radial edgeuse chain consistency (BA_670) ----

    BA_670 = (
        "BA.670",
        "brep-radial-chain-same-edge",
        "edgeuse:nextRadialEUIndex must partition edgeuses into closed circular chains; all edgeuses in each chain must reference the same edge via edgeuse:edgeIndex, and all edgeuses referencing a given edge must belong to a single chain.",
        "capabilities/visualization/brep/requirements/brep-radial-chain-same-edge.html",
        "Core USD",
        ("correctness",),
    )
    # ---- GeomSubset material binding validation (BA_680-BA_682) ----

    BA_680 = (
        "BA.680",
        "brep-geomsubset-indices-valid-range",
        "GeomSubset indices must be within valid range: [0, numBreps) for elementType='brep', [0, numFaces) for elementType='face'.",
        "capabilities/visualization/brep/requirements/brep-geomsubset-indices-valid-range.html",
        "Core USD",
        ("correctness",),
    )
    BA_681 = (
        "BA.681",
        "brep-geomsubset-indices-non-overlapping",
        "Within the same elementType, no index should appear in more than one GeomSubset.",
        "capabilities/visualization/brep/requirements/brep-geomsubset-indices-non-overlapping.html",
        "Core USD",
        ("correctness",),
    )
    BA_682 = (
        "BA.682",
        "brep-geomsubset-material-binding-target-exists",
        "GeomSubset material:binding relationship targets must reference existing prims on the stage.",
        "capabilities/visualization/brep/requirements/brep-geomsubset-material-binding-target-exists.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Minimum topology counts (BA_700-BA_702) ----

    BA_700 = (
        "BA.700",
        "brep-region-count-minimum",
        "Every brep:regionCount entry must be at least 1.",
        "capabilities/visualization/brep/requirements/brep-region-count-minimum.html",
        "Core USD",
        ("correctness",),
    )
    BA_701 = (
        "BA.701",
        "brep-region-shell-count-minimum",
        "Every region:shellCount entry must be at least 1.",
        "capabilities/visualization/brep/requirements/brep-region-shell-count-minimum.html",
        "Core USD",
        ("correctness",),
    )
    BA_702 = (
        "BA.702",
        "brep-shell-must-have-content",
        "Every shell must have faceuseCount > 0, wireEdgeCount > 0, or pointType == 'BrepPointAPI'.",
        "capabilities/visualization/brep/requirements/brep-shell-must-have-content.html",
        "Core USD",
        ("correctness",),
    )

    # ---- Shell point position containment (BA_710) ----

    BA_710 = (
        "BA.710",
        "brep-shell-point-position-extent-containment",
        "All brep:shellPoint:point:position values must lie within their corresponding brep:extent bounding box.",
        "capabilities/visualization/brep/requirements/brep-shell-point-position-extent-containment.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )

    # ---- Curve/surface type count totals (BA_720-BA_722) ----

    BA_720 = (
        "BA.720",
        "brep-edge-curve-type-exhaustive",
        "The sum of edges across all recognized curveType categories must equal the total edge count.",
        "capabilities/visualization/brep/requirements/brep-edge-curve-type-exhaustive.html",
        "Core USD",
        ("correctness",),
    )
    BA_721 = (
        "BA.721",
        "brep-wireEdge-curve-type-exhaustive",
        "The sum of wireEdges across all recognized curveType categories must equal the total wireEdge count.",
        "capabilities/visualization/brep/requirements/brep-wireEdge-curve-type-exhaustive.html",
        "Core USD",
        ("correctness",),
    )
    BA_722 = (
        "BA.722",
        "brep-face-surface-type-exhaustive",
        "The sum of faces across all recognized surfaceType categories must equal the total face count.",
        "capabilities/visualization/brep/requirements/brep-face-surface-type-exhaustive.html",
        "Core USD",
        ("correctness",),
    )

    # BA_730 retired: NURBS endpoint-to-vertex agreement requires kernel evaluation.

    # ---- UV trim curve domain containment (BA_750) ----

    BA_750 = (
        "BA.750",
        "brep-uv-trim-curve-domain-containment",
        "UV trim curve control vertices should lie within or near the corresponding face's UV range domain.",
        "capabilities/visualization/brep/requirements/brep-uv-trim-curve-domain-containment.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )
    # BA_761 retired: repeated edge indices cannot establish geometric seam validity.
    # BA_762 / BA_765 retired: shifted full/partial periodic face ranges are allowed.
    # Their span limits remain covered by BA_560 / BA_562-BA_565.

    BA_763 = (
        "BA.763",
        "brep-uv-loop-closure",
        "Authored UV trim curves in each loop must connect head-to-tail in parameter space.",
        "capabilities/visualization/brep/requirements/brep-uv-loop-closure.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )

    BA_764 = (
        "BA.764",
        "brep-zero-length-uv-trim-curve",
        "Authored UV trim curves must not collapse to zero length in parameter space.",
        "capabilities/visualization/brep/requirements/brep-zero-length-uv-trim-curve.html",
        "Core USD",
        ("correctness", "geometric-consistency"),
    )

cap.BrepArrayRequirements = BrepArrayRequirements

class BrepConstants:
    NUMERICAL_TOLERANCE = 1e-11

    @staticmethod
    def is_brep_point_shell(
        shell_index: int,
        shell_point_types,
        shell_faceuse_counts,
        shell_wireedge_counts,
    ) -> bool:
        """Return whether a shell contributes one BrepPointAPI occurrence.

        The schema says shell:pointType is meaningful only when both the
        faceuse and wire-edge counts are zero. Keep the bounds checks here so
        malformed parallel arrays are reported by their cardinality rules
        rather than raising while another rule counts packed geometry.
        """
        try:
            return (
                str(shell_point_types[shell_index]) == "BrepPointAPI"
                and int(shell_faceuse_counts[shell_index]) == 0
                and int(shell_wireedge_counts[shell_index]) == 0
            )
        except (IndexError, TypeError, ValueError):
            return False

    @staticmethod
    def isFloatLessThan(point: float, extent: float) -> bool:
        """Returns True if point is less than extent and not within single-precision tolerance."""
        return point < extent and not math.isclose(point, extent, rel_tol=1e-5, abs_tol=1e-6)

    @staticmethod
    def isFloatGreaterThan(point: float, extent: float) -> bool:
        """Returns True if point is greater than extent and not within single-precision tolerance."""
        return point > extent and not math.isclose(point, extent, rel_tol=1e-5, abs_tol=1e-6)

    @staticmethod
    def get_brep_id(brep_region_counts: list, brep_idx: int) -> str:
        """
        Helper method to get the BREP identifier for error messages.
        Returns the array index as a string.
        
        Args:
            brep_region_counts (list): Array used to determine valid brep count
            brep_idx (int): Index in the BREP arrays
            
        Returns:
            str: The brep index as a string
        """
        return str(brep_idx)

    @staticmethod
    def safe_get_attribute(brep_array: Usd.Prim, attr_name: str, default_value=None):
        """
        Safely get a USD attribute value, handling UnregisteredValue cases.
        
        Args:
            brep_array: The USD prim
            attr_name: Name of the attribute to retrieve
            default_value: Default value if attribute is None, invalid, or UnregisteredValue
            
        Returns:
            The attribute value or default_value if invalid
        """
        if default_value is None:
            default_value = []
            
        try:
            attr = brep_array.GetAttribute(attr_name)
            if not attr:
                return default_value
                
            value = attr.Get()
            if value is None:
                return default_value
                
            # Check if it's an UnregisteredValue or other invalid type
            if hasattr(value, '__class__') and 'UnregisteredValue' in str(type(value)):
                return default_value
                
            # Try to get length to verify it's a proper sequence
            try:
                len(value)
                return value
            except (TypeError, AttributeError):
                return default_value
                
        except Exception:
            # Any other exception during attribute access
            return default_value


def _normalize_requirement(req: cap.Requirement, default_version: str = "1.0.0") -> cap.Requirement:
    """
    Ensure requirements carry a semantic version for the asset validator.
    """
    version = default_version
    return cap.Requirement(
        code=req.code,
        version=version,
        display_name=getattr(req, "display_name", None),
        message=getattr(req, "message", None),
        path=getattr(req, "path", None),
        compatibility=getattr(req, "compatibility", None),
        tags=getattr(req, "tags", ()),
        parameters=getattr(req, "parameters", ()),
        examples=getattr(req, "examples", ()),
    )

_BREP_REQUIREMENTS = [_normalize_requirement(r) for r in BrepArrayRequirements]
_BREP_REQUIREMENTS_BY_CODE = {r.code: r for r in _BREP_REQUIREMENTS}

try:
    _register_brep_requirements = register_requirements(*_BREP_REQUIREMENTS)
except (ImportError, RuntimeError) as e:
    # Missing capabilities or runtime registration issue; continue without registration.
    print(f"WARNING: Requirement registration failed; continuing without registration: {type(e).__name__}: {e}")

    def _register_brep_requirements(cls):
        return cls

@_register_brep_requirements
class BrepValidator(BaseRuleChecker):
    """
    Validator for Brep-based geometry objects, fully validating all data members and relationships between attributes.
    """

    def __init__(self, *args, **kwargs):
        _ensure_event_loop()
        global _omnisolid_registered
        if not _omnisolid_registered:
            from .validate import register_omnisolid_schema  # validate imports this module

            if not register_omnisolid_schema():
                print(
                    "WARNING: omniSolid schema not found; BrepArray prim types will not resolve. "
                    "Set OMNISOLID_PLUGIN_PATH to the omniSolid schema resources directory."
                )
            _omnisolid_registered = True
        super().__init__(*args, **kwargs)
        # Thread-local so concurrent CheckPrim calls on one instance (Asset Validator
        # threadpools) cannot cross-contaminate cached offsets between prims.
        self._brep_offsets_tls = threading.local()

    def _clear_brep_offsets_cache(self) -> None:
        self._brep_offsets_tls.cache_prim = None
        self._brep_offsets_tls.cache = None

    def _get_brep_offsets_cache(self, brep_array: Usd.Prim):
        cache_prim = getattr(self._brep_offsets_tls, "cache_prim", None)
        cache = getattr(self._brep_offsets_tls, "cache", None)
        if cache_prim is brep_array and cache is not None:
            return cache
        return None

    @staticmethod
    def _requirement_with_version(requirement):
        """
        Map any Brep requirement to the pre-registered, semver-tagged instance.
        """
        if requirement is None:
            return None
        code = getattr(requirement, "code", None)
        if code and code in _BREP_REQUIREMENTS_BY_CODE:
            return _BREP_REQUIREMENTS_BY_CODE[code]
        # Fallback: enforce a semver string once
        version = getattr(requirement, "version", None)
        if not version or not isinstance(version, str) or not version.strip():
            version = "1.0.0"
        try:
            return cap.Requirement(
                code=code,
                version=version,
                display_name=getattr(requirement, "display_name", None),
                message=getattr(requirement, "message", None),
                path=getattr(requirement, "path", None),
                compatibility=getattr(requirement, "compatibility", None),
                tags=getattr(requirement, "tags", ()),
                parameters=getattr(requirement, "parameters", ()),
                examples=getattr(requirement, "examples", ()),
            )
        except (TypeError, ValueError, AttributeError) as e:
            # Log unexpected requirement creation failures for debugging
            print(f"WARNING: Failed to create normalized requirement: {e}")
            return requirement

    def _AddFailedCheck(self, message=None, at=None, suggestion=None, code=None, requirement=None) -> None:
        requirement = self._requirement_with_version(requirement)
        super()._AddFailedCheck(message=message, at=at, suggestion=suggestion, code=code, requirement=requirement)

    def _AddError(self, message=None, at=None, suggestion=None, code=None, requirement=None) -> None:
        requirement = self._requirement_with_version(requirement)
        super()._AddError(message=message, at=at, suggestion=suggestion, code=code, requirement=requirement)

    def _AddWarning(self, message=None, at=None, suggestion=None, code=None, requirement=None) -> None:
        requirement = self._requirement_with_version(requirement)
        super()._AddWarning(message=message, at=at, suggestion=suggestion, code=code, requirement=requirement)

    def _AddInfo(self, message=None, at=None, code=None, requirement=None) -> None:
        requirement = self._requirement_with_version(requirement)
        super()._AddInfo(message=message, at=at, code=code, requirement=requirement)

    ### Helper Methods ###

    def _validate_array_sizes_and_authored(self, brep_array: Usd.Prim, attributes: list[str], requirement: cap.BrepArrayRequirements, size: int = None, require_authored: bool = True) -> None:
        """
        Validates that all attributes:
        - Have consistent sizes across arrays.
        - Are authored (if require_authored=True).
        """
        sizes = {}
        authored_count = 0
        
        for attr_name in attributes:
            attr = brep_array.GetAttribute(attr_name)
            if not attr or not attr.IsAuthored():
                if require_authored:
                    self._AddFailedCheck(
                        requirement=requirement,
                        message=f"{attr_name} is not authored in BrepArray.",
                        at=brep_array,
                    )
            else:
                _, seq_len = self._get_attr_sequence(
                    brep_array,
                    attr_name,
                    requirement,
                    "a valid USD array",
                )
                authored_count += 1
                sizes[attr_name] = seq_len

        # If no attributes are authored and we don't require them, that's OK for empty models
        if authored_count == 0 and not require_authored:
            return

        # Ensure consistent size
        if len(set(sizes.values())) > 1:
            self._AddFailedCheck(
                requirement=requirement,
                message=f"Inconsistent sizes detected across {attributes} attributes: {sizes}.",
                at=brep_array,
            )
        if size is not None and any(size != s for s in sizes.values()):
                self._AddFailedCheck(
                    requirement=requirement,
                    message=f"Expected size {size} does not match actual sizes {sizes}.",
                    at=brep_array,
                )
    
    def _validate_authorship_only(self, brep_array: Usd.Prim, attributes: list[str], requirement: cap.BrepArrayRequirements) -> None:
        """
        Validates that all attributes are authored, without checking sizes.
        """
        for attr_name in attributes:
            attr = brep_array.GetAttribute(attr_name)
            if not attr.IsAuthored():
                self._AddFailedCheck(
                    requirement=requirement,
                    message=f"{attr_name} is not authored in BrepArray.",
                    at=brep_array,
                )

    def _validate_allowed_tokens(self, brep_array: Usd.Prim, attr_name: str, allowed_tokens: list[str], requirement: cap.BrepArrayRequirements, item_name: str = "item") -> None:
        """
        Validates that the given attribute values are restricted to the allowed tokens.
        Reports specific indices where invalid tokens are found.
        """
        attr = brep_array.GetAttribute(attr_name)
        values = attr.Get()
        if values:
            invalid_indices = []
            for idx, value in enumerate(values):
                if value not in allowed_tokens:
                    invalid_indices.append((idx, value))
            
            if invalid_indices:
                if len(invalid_indices) == 1:
                    idx, value = invalid_indices[0]
                    self._AddFailedCheck(
                        requirement=requirement,
                        message=f"{attr_name}[{idx}] has invalid value '{value}' for {item_name} #{idx}. Allowed values are {allowed_tokens}.",
                        at=brep_array,
                    )
                else:
                    invalid_details = [f"[{idx}]='{value}'" for idx, value in invalid_indices]
                    self._AddFailedCheck(
                        requirement=requirement,
                        message=f"{attr_name} has invalid values at indices: {', '.join(invalid_details)}. Allowed values are {allowed_tokens}.",
                        at=brep_array,
                    )

    def _compute_brep_offsets(self, brep_array: Usd.Prim) -> dict:
        """
        Compute offsets for all hierarchical categories, including vertices, partitioning them by Brep.

        Args:
            brep_array (Usd.Prim): The BrepArray prim from which offsets are computed.

        Returns:
            dict: A dictionary with offsets for each category, partitioned by Brep.

                Example format:
                {
                    "regions": [0, 3, 7],   # Partition boundaries by offsets for regions
                    "shells": [0, 5, 13],  # Partition boundaries for shells
                    "vertices": [0, 10, 25]  # Partition boundaries for vertices
                    # ... other categories
                }
        """
        cached = self._get_brep_offsets_cache(brep_array)
        if cached is not None:
            return cached

        ### Retrieve counts from brep_array ###
        # Counts for regions per Brep
        region_counts = brep_array.GetAttribute("brep:regionCount").Get()
        if region_counts is None:
            region_counts = []

        # Counts for shells per region
        shell_counts_per_region = brep_array.GetAttribute("region:shellCount").Get()
        if shell_counts_per_region is None:
            shell_counts_per_region = []

        # Counts for faceuses per shell
        faceuse_counts_per_shell = brep_array.GetAttribute("shell:faceuseCount").Get()
        if faceuse_counts_per_shell is None:
            faceuse_counts_per_shell = []

        # Counts for faces per faceuse
        loop_counts_per_face = brep_array.GetAttribute("face:loopCount").Get()
        if loop_counts_per_face is None:
            loop_counts_per_face = []

        # Counts for edgeuses per loop
        edgeuse_counts_per_loop = brep_array.GetAttribute("loop:edgeuseCount").Get()
        if edgeuse_counts_per_loop is None:
            edgeuse_counts_per_loop = []

        # Counts for wireedges per shell
        wireedge_counts_per_shell = brep_array.GetAttribute("shell:wireEdgeCount").Get()
        if wireedge_counts_per_shell is None:
            wireedge_counts_per_shell = []

        # Retrieve edge:vertexIndices (list of pairs)
        edge_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "edge:vertexIndices")

        # Retrieve wireEdge:vertexIndices (list of pairs)
        wire_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "wireEdge:vertexIndices")

        # Retrieve loop:vertexIndex (list of single indices)
        loop_vertex_indices = brep_array.GetAttribute("loop:vertexIndex").Get()
        if loop_vertex_indices is None:
            loop_vertex_indices = []

        # Coerce count arrays to integers; fail BA_091 if types are invalid
        def _to_int_list(values, attr_name):
            try:
                return [int(v) for v in (values or [])]
            except (TypeError, ValueError):
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_091,
                    message=f"{attr_name} has invalid data type; expected integers.",
                    at=brep_array,
                )
                return None

        region_counts = _to_int_list(region_counts, "brep:regionCount")
        shell_counts_per_region = _to_int_list(shell_counts_per_region, "region:shellCount")
        faceuse_counts_per_shell = _to_int_list(faceuse_counts_per_shell, "shell:faceuseCount")
        loop_counts_per_face = _to_int_list(loop_counts_per_face, "face:loopCount")
        edgeuse_counts_per_loop = _to_int_list(edgeuse_counts_per_loop, "loop:edgeuseCount")
        wireedge_counts_per_shell = _to_int_list(wireedge_counts_per_shell, "shell:wireEdgeCount")

        if any(
            v is None
            for v in [
                region_counts,
                shell_counts_per_region,
                faceuse_counts_per_shell,
                loop_counts_per_face,
                edgeuse_counts_per_loop,
                wireedge_counts_per_shell,
            ]
        ):
            # We already reported a BA_091-type failure; return a fully populated,
            # empty offsets dict so downstream checks can short-circuit safely.
            return self._cache_brep_offsets(
                brep_array,
                {
                "regions": [],
                "shells": [],
                "faceuses": [],
                "faces": [],
                "loops": [],
                "edgeuses": [],
                "edges": [],
                "wireedges": [],
                "vertices": [],
                "edge_control_vertices": [],
                "surface_control_vertices": [],
                "edge3d_nurbs_curves": [],
                "surface_nurbs": [],
                "curveUv": [],
                },
            )

        ### Helper function to compute offsets cumulatively ###
        def compute_offsets(counts):
            return [0] + [sum(counts[:i + 1]) for i in range(len(counts))]

        ### Compute hierarchical offsets ###
        # Regions per Brep
        region_offsets = compute_offsets(region_counts)

        # Shells per Brep (aggregate `region:shellCount` across regions for each Brep)
        shells = []
        for brep_idx in range(len(region_counts)):
            start = region_offsets[brep_idx]
            end = region_offsets[brep_idx + 1]
            if shell_counts_per_region and start < len(shell_counts_per_region):
                end = min(end, len(shell_counts_per_region))
                shells.append(sum(shell_counts_per_region[start:end]))
            else:
                shells.append(0)
        shell_offsets = compute_offsets(shells)

        # Faceuses per Brep (aggregate `shell:faceuseCount` across shells for each Brep)
        faceuses = []
        for brep_idx in range(len(shells)):
            start = shell_offsets[brep_idx]
            end = shell_offsets[brep_idx + 1]
            if faceuse_counts_per_shell and start < len(faceuse_counts_per_shell):
                end = min(end, len(faceuse_counts_per_shell))
                faceuses.append(sum(faceuse_counts_per_shell[start:end]))
            else:
                faceuses.append(0)
        faceuse_offsets = compute_offsets(faceuses)

        # Faces per Brep (faceuses/2 since each face has 2 faceuses)
        faces = []
        for brep_idx in range(len(faceuses)):
            faces.append(faceuses[brep_idx] // 2)  # Integer division since each face has exactly 2 faceuses
        face_offsets = compute_offsets(faces)

        # Loops per Brep (aggregate `face:loopCount` across faces for each Brep)
        loops = []
        if loop_counts_per_face is not None and len(loop_counts_per_face) > 0:
            for brep_idx in range(len(faces)):
                start = face_offsets[brep_idx]
                end = face_offsets[brep_idx + 1]
                # Ensure we don't slice beyond the array bounds
                max_index = len(loop_counts_per_face)
                safe_start = min(start, max_index)
                safe_end = min(end, max_index)
                
                if safe_start < safe_end:
                    slice_data = loop_counts_per_face[safe_start:safe_end]
                    loops.append(sum(slice_data) if slice_data is not None else 0)
                else:
                    loops.append(0)
        else:
            # If no face loop counts, assume minimal loop structure
            loops = [0] * len(faces) if faces else [0]
        loop_offsets = compute_offsets(loops)

        # Compute edgeuse offsets by properly partitioning top-down through the hierarchy
        # Start from the top: partition regions by brep, then shells by region, etc.
        
        # 6. Now compute edgeuse offsets by aggregating edgeuse counts per loop for each brep
        edgeuses_per_brep = []
        
        for brep_idx in range(len(region_counts)):
            # Get the loop range for this brep
            loop_start = loop_offsets[brep_idx]
            loop_end = loop_offsets[brep_idx + 1]
            
            # Sum edgeuse counts for all loops in this brep
            brep_edgeuse_total = 0
            for loop_idx in range(loop_start, min(loop_end, len(edgeuse_counts_per_loop))):
                brep_edgeuse_total += edgeuse_counts_per_loop[loop_idx]
            
            edgeuses_per_brep.append(brep_edgeuse_total)
        
        # Compute edgeuse offsets from the per-brep counts
        edgeuse_offsets = compute_offsets(edgeuses_per_brep)

        # Wireedges per Brep (aggregate `shell:wireEdgeCount` across shells for each Brep)
        wireedges = []
        for brep_idx in range(len(shells)):
            start = shell_offsets[brep_idx]
            end = shell_offsets[brep_idx + 1]
            # Ensure we don't slice beyond the array bounds
            if wireedge_counts_per_shell is not None:
                max_index = len(wireedge_counts_per_shell)
                safe_start = min(start, max_index)
                safe_end = min(end, max_index)
                
                if safe_start < safe_end:
                    slice_data = wireedge_counts_per_shell[safe_start:safe_end]
                    wireedges.append(sum(slice_data) if slice_data is not None else 0)
                else:
                    wireedges.append(0)
            else:
                wireedges.append(0)
        wireedge_offsets = compute_offsets(wireedges)

        # Determine partitioning: use count aggregation for topology, analyze usage for geometry
        num_breps = len(region_counts)
        
        if num_breps > 1:
            # Use the hierarchical edgeuse offsets (already computed above) to partition edgeuses
            # Then analyze which edges each brep's edgeuses actually reference
            
            edgeuse_edge_indices = brep_array.GetAttribute("edgeuse:edgeIndex").Get() or []
            
            # For each brep, find which edges are used by its edgeuses
            edge_sets_per_brep = []
            for brep_idx in range(num_breps):
                edgeuse_start = edgeuse_offsets[brep_idx]
                edgeuse_end = edgeuse_offsets[brep_idx + 1]
                
                # Get the edgeuses for this brep
                brep_edgeuses = edgeuse_edge_indices[edgeuse_start:edgeuse_end]
                
                # Find unique edge indices used by this brep
                brep_edges = set(brep_edgeuses) if brep_edgeuses else set()
                edge_sets_per_brep.append(brep_edges)
            
            # Create edge offsets based on the edge counts per brep
            edge_counts_per_brep = [len(brep_edges) for brep_edges in edge_sets_per_brep]
            edge_offsets = compute_offsets(edge_counts_per_brep)
            
            # For vertices, analyze which vertices are used by edges, wire edges, and loops for each brep
            edge_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "edge:vertexIndices")
            wire_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "wireEdge:vertexIndices")
            loop_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "loop:vertexIndex")
            loop_edge_counts = brep_array.GetAttribute("loop:edgeuseCount").Get() or []
            
            vertex_counts_per_brep = []
            
            for brep_idx, brep_edges in enumerate(edge_sets_per_brep):
                brep_vertices = set()
                
                # 1. Add vertices from edges that belong to this brep
                for edge_idx in brep_edges:
                    if edge_idx < len(edge_vertex_indices):
                        vertex_pair = edge_vertex_indices[edge_idx]
                        if vertex_pair and len(vertex_pair) == 2:
                            brep_vertices.update(vertex_pair)
                
                # 2. Add vertices from wire edges that belong to this brep's shells
                shell_start = shell_offsets[brep_idx]
                shell_end = shell_offsets[brep_idx + 1]
                
                # For each shell in this brep, get its wire edges
                for shell_idx in range(shell_start, shell_end):
                    if shell_idx < len(wireedge_offsets) - 1:
                        wireedge_start = wireedge_offsets[shell_idx]
                        wireedge_end = wireedge_offsets[shell_idx + 1]
                        
                        for wire_idx in range(wireedge_start, min(wireedge_end, len(wire_vertex_indices))):
                            vertex_pair = wire_vertex_indices[wire_idx]
                            if vertex_pair and len(vertex_pair) == 2:
                                brep_vertices.update(vertex_pair)
                
                # 3. Add vertices from loops that belong to this brep (when edgeuseCount == 0)
                loop_start = loop_offsets[brep_idx]
                loop_end = loop_offsets[brep_idx + 1]
                
                for loop_idx in range(loop_start, min(loop_end, len(loop_vertex_indices), len(loop_edge_counts))):
                    if loop_edge_counts[loop_idx] == 0:  # Only when loop has no edgeuses
                        brep_vertices.add(loop_vertex_indices[loop_idx])
                
                # Count the unique vertices for this brep
                vertex_counts_per_brep.append(len(brep_vertices))
            
            # Create vertex offsets based on the vertex counts per brep
            vertex_offsets = compute_offsets(vertex_counts_per_brep)
            
        else:
            # For single brep, use total array sizes
            edge_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "edge:vertexIndices")
            vertex_point_types = BrepConstants.safe_get_attribute(brep_array, "vertex:pointType") or []
            edgeuse_edge_indices = BrepConstants.safe_get_attribute(brep_array, "edgeuse:edgeIndex")
            
            total_edges = len(edge_vertex_indices)
            total_vertices = len(vertex_point_types)
            total_edgeuses = len(edgeuse_edge_indices)
            
            edge_offsets = [0, total_edges]
            vertex_offsets = [0, total_vertices]
            edgeuse_offsets = [0, total_edgeuses]
        
        # Compute NURBS and geometric data partitions
        edge_control_offsets = self._compute_nurbs_control_vertex_offsets(
            brep_array, "edge", edge_offsets if num_breps > 1 else [0, total_edges]
        )
        surface_control_offsets = self._compute_nurbs_control_vertex_offsets(
            brep_array, "surface", face_offsets if num_breps > 1 else [0, sum(faces) if faces else 0]
        )
        
        # Compute NURBS curve data offsets (edge3d curves)
        edge3d_nurbs_offsets = self._compute_edge3d_nurbs_data_offsets(
            brep_array, edge_offsets if num_breps > 1 else [0, total_edges]
        )
        
        # Compute NURBS surface data offsets
        surface_nurbs_offsets = self._compute_surface_nurbs_data_offsets(
            brep_array, face_offsets if num_breps > 1 else [0, sum(faces) if faces else 0]
        )
        
        # Compute curveUv data offsets
        curveUv_offsets = self._compute_curveUv_data_offsets(
            brep_array, edgeuse_offsets if num_breps > 1 else [0, total_edgeuses]
        )
        
        ### Return computed offsets as a dictionary ###
        return self._cache_brep_offsets(
            brep_array,
            {
            "regions": region_offsets,
            "shells": shell_offsets,
            "faceuses": faceuse_offsets,
            "faces": face_offsets,
            "loops": loop_offsets,
            "edgeuses": edgeuse_offsets,
            "edges": edge_offsets,
            "wireedges": wireedge_offsets,
            "vertices": vertex_offsets,
            "edge_control_vertices": edge_control_offsets,
            "surface_control_vertices": surface_control_offsets,
            "edge3d_nurbs_curves": edge3d_nurbs_offsets,
            "surface_nurbs": surface_nurbs_offsets,
            "curveUv": curveUv_offsets,
            },
        )

    def _cache_brep_offsets(self, brep_array: Usd.Prim, offsets: dict) -> dict:
        self._brep_offsets_tls.cache_prim = brep_array
        self._brep_offsets_tls.cache = offsets
        return offsets

    def _get_edge_intersect_tolerance(
        self,
        brep_array: Usd.Prim,
        edge_idx: int,
        intersect_tols=None,
        brep_region_counts=None,
        edge_brep_indices=None,
    ):
        if intersect_tols is None:
            intersect_tols = BrepConstants.safe_get_attribute(brep_array, "brep:intersectTol3d")
        if not intersect_tols:
            return None, None, None

        if edge_brep_indices is None:
            edge_brep_indices = self._compute_edge_brep_indices(brep_array)

        brep_idx = edge_brep_indices.get(edge_idx)
        if brep_idx is None and len(intersect_tols) == 1:
            brep_idx = 0
        if brep_idx is None or brep_idx >= len(intersect_tols):
            return None, None, None

        try:
            tol = float(intersect_tols[brep_idx])
        except (TypeError, ValueError):
            return None, None, None
        if not math.isfinite(tol) or tol < BrepConstants.NUMERICAL_TOLERANCE:
            return None, None, None

        if brep_region_counts is None:
            brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        return tol, brep_idx, BrepConstants.get_brep_id(brep_region_counts, brep_idx)

    def _compute_edge_brep_indices(self, brep_array: Usd.Prim, edgeuse_offsets=None, edgeuse_edge_indices=None):
        if edgeuse_offsets is None:
            edgeuse_offsets = self._compute_brep_offsets(brep_array).get("edgeuses", [])
        if edgeuse_edge_indices is None:
            edgeuse_edge_indices = BrepConstants.safe_get_attribute(brep_array, "edgeuse:edgeIndex")

        edge_brep_indices = {}
        for brep_idx, (edgeuse_start, edgeuse_end) in enumerate(zip(edgeuse_offsets[:-1], edgeuse_offsets[1:])):
            for edgeuse_idx in range(edgeuse_start, min(edgeuse_end, len(edgeuse_edge_indices))):
                try:
                    edge_idx = int(edgeuse_edge_indices[edgeuse_idx])
                except (TypeError, ValueError):
                    continue
                edge_brep_indices.setdefault(edge_idx, brep_idx)
        return edge_brep_indices

    def _report_unresolved_edge_tolerance(self, brep_array: Usd.Prim, requirement, edge_idx: int, check_label: str):
        self._AddFailedCheck(
            requirement=requirement,
            message=(
                f"{check_label} for edge #{edge_idx} could not be validated because no positive "
                "brep:intersectTol3d value could be resolved for the edge. The tolerance is missing, "
                "invalid, or the edge could not be associated with a BRep."
            ),
            at=brep_array,
        )

    def _compute_nurbs_control_vertex_offsets(self, brep_array: Usd.Prim, nurbs_type: str, entity_offsets: list[int]) -> list[int]:
        """
        Compute control vertex offsets for NURBS curves or surfaces partitioned by brep.
        
        Args:
            brep_array: The BrepArray prim
            nurbs_type: Either "edge" or "surface"  
            entity_offsets: Offsets for edges or faces per brep
            
        Returns:
            List of cumulative offsets for control vertices partitioned by brep
        """
        if nurbs_type == "edge":
            vertex_count_attr = "brep:edge3dNurb:curve3d:nurb:vertexCount"
        elif nurbs_type == "surface":
            u_vertex_count_attr = "brep:surface:nurb:uVertexCount"
            v_vertex_count_attr = "brep:surface:nurb:vVertexCount"
        else:
            return [0]  # Invalid type
            
        # Get NURBS vertex counts
        if nurbs_type == "edge":
            vertex_counts = brep_array.GetAttribute(vertex_count_attr).Get() or []
            if not vertex_counts:
                return [0] * len(entity_offsets)
                
            # Compute cumulative control vertex counts per brep
            control_vertex_counts = []
            for brep_idx in range(len(entity_offsets) - 1):
                edge_start = entity_offsets[brep_idx]
                edge_end = entity_offsets[brep_idx + 1]
                
                # Sum vertex counts for all edges in this brep
                brep_control_vertices = 0
                for edge_idx in range(edge_start, min(edge_end, len(vertex_counts))):
                    brep_control_vertices += vertex_counts[edge_idx]
                    
                control_vertex_counts.append(brep_control_vertices)
                
        else:  # surface
            u_vertex_counts = brep_array.GetAttribute(u_vertex_count_attr).Get() or []
            v_vertex_counts = brep_array.GetAttribute(v_vertex_count_attr).Get() or []
            
            if not u_vertex_counts or not v_vertex_counts:
                return [0] * len(entity_offsets)
                
            # Compute cumulative control vertex counts per brep
            control_vertex_counts = []
            for brep_idx in range(len(entity_offsets) - 1):
                face_start = entity_offsets[brep_idx]
                face_end = entity_offsets[brep_idx + 1]
                
                # Sum vertex counts for all faces in this brep
                brep_control_vertices = 0
                for face_idx in range(face_start, min(face_end, len(u_vertex_counts), len(v_vertex_counts))):
                    brep_control_vertices += u_vertex_counts[face_idx] * v_vertex_counts[face_idx]
                    
                control_vertex_counts.append(brep_control_vertices)
        
        # Compute cumulative offsets
        offsets = [0]
        cumulative = 0
        for count in control_vertex_counts:
            cumulative += count
            offsets.append(cumulative)
            
        return offsets

    def _compute_edge3d_nurbs_data_offsets(self, brep_array: Usd.Prim, edge_offsets: list[int]) -> list[int]:
        """
        Compute offsets for edge3d NURBS curve data partitioned by brep.
        
        Returns:
            List of cumulative offsets for NURBS curves (matching edge counts per brep)
        """
        # For edge3d NURBS, we need to know which edges use NURBS curves
        edge_curve_types = brep_array.GetAttribute("edge:curveType").Get() or []
        
        if not edge_curve_types:
            return [0] * len(edge_offsets)
            
        # Count NURBS curves per brep
        nurbs_curve_counts = []
        for brep_idx in range(len(edge_offsets) - 1):
            edge_start = edge_offsets[brep_idx]
            edge_end = edge_offsets[brep_idx + 1]
            
            # Count edges with NURBS curves in this brep
            nurbs_count = 0
            for edge_idx in range(edge_start, min(edge_end, len(edge_curve_types))):
                if edge_curve_types[edge_idx] == "BrepCurve3dNurbAPI":
                    nurbs_count += 1
                    
            nurbs_curve_counts.append(nurbs_count)
        
        # Compute cumulative offsets
        offsets = [0]
        cumulative = 0
        for count in nurbs_curve_counts:
            cumulative += count
            offsets.append(cumulative)
            
        return offsets

    def _compute_surface_nurbs_data_offsets(self, brep_array: Usd.Prim, face_offsets: list[int]) -> list[int]:
        """
        Compute offsets for surface NURBS data partitioned by brep.
        
        Returns:
            List of cumulative offsets for NURBS surfaces (matching face counts per brep)
        """
        # For surfaces, we need to know which faces use NURBS surfaces
        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        
        if not face_surface_types:
            return [0] * len(face_offsets)
            
        # Count NURBS surfaces per brep
        nurbs_surface_counts = []
        for brep_idx in range(len(face_offsets) - 1):
            face_start = face_offsets[brep_idx]
            face_end = face_offsets[brep_idx + 1]
            
            # Count faces with NURBS surfaces in this brep
            nurbs_count = 0
            for face_idx in range(face_start, min(face_end, len(face_surface_types))):
                if face_surface_types[face_idx] == "BrepSurfaceNurbAPI":
                    nurbs_count += 1
                    
            nurbs_surface_counts.append(nurbs_count)
        
        # Compute cumulative offsets
        offsets = [0]
        cumulative = 0
        for count in nurbs_surface_counts:
            cumulative += count
            offsets.append(cumulative)
            
        return offsets

    def _compute_curveUv_data_offsets(self, brep_array: Usd.Prim, edgeuse_offsets: list[int]) -> list[int]:
        """
        Compute offsets for curveUv data partitioned by brep.
        
        Returns:
            List of cumulative offsets for curveUv data (matching edgeuse counts per brep)
        """
        # CurveUV data maps to edgeuses, so use edgeuse offsets directly
        return edgeuse_offsets

    def _validate_indexing_relationships(
        self,
        counts_array: list[int],
        index_array: dict[str, list[int]],
        target_array: list[int],
        offsets: list[int],
        brep_array: Usd.Prim,
        requirement: cap.BrepArrayRequirements,
    ) -> None:
        """
        Validates that the dependent arrays:
            - Are correctly indexed based on 'counts_array'.
            - Ensure indices belong to the correct partitions of `target_array`, as defined by `offsets`.
        Args:
            counts_array (list[int]): A list of counts corresponding to each Brep.
            index_array (dict[str, list[int]]): A dictionary of dependent arrays containing indices referencing `target_array`.
            target_array (list[int]): The array being partitioned and referenced.
            offsets (list[int]): Array of offsets partitioning `target_array` by Brep. Defines boundaries for each Brep's objects.
            brep_array (Usd.Prim): A USD BrepArray object.
            requirement (cap.BrepArrayRequirements): The validation requirement being checked for compliance.
        """
        # Ensure counts_array is valid
        if counts_array is None or not counts_array:
            return

        # Compute offsets for counts_array (block boundaries for index_array)
        brep_offsets = [0] + [sum(counts_array[:i + 1]) for i in range(len(counts_array))]
        num_breps = len(brep_offsets) - 1
        total_count = sum(counts_array)

        # Validate dependent arrays
        for attr_name, array in index_array.items():
            # Check if array exists
            if not array and total_count > 0:
                self._AddFailedCheck(
                    requirement=requirement,
                    message=f"{attr_name} is missing or not authored, thus the indexing is not valid.",
                    at=brep_array,
                )
                continue

            # Check array size matches total count
            if array is None:
                self._AddFailedCheck(
                    requirement=requirement,
                    message=f"{attr_name} is None, thus the indexing is not valid.",
                    at=brep_array,
                )
                continue
            elif len(array) != total_count:
                self._AddFailedCheck(
                    requirement=requirement,
                    message=f"{attr_name} size {len(array)} does not match expected size {total_count}, thus the indexing is not valid.",
                    at=brep_array,
                )
                continue

            # Validate each block against corresponding offsets partition in `target_array`
            for brep_idx in range(num_breps):
                # Get block for current Brep from index_array
                start_idx = brep_offsets[brep_idx]
                end_idx = brep_offsets[brep_idx + 1]
                block = array[start_idx:end_idx] if array is not None else []

                # Get corresponding partition boundaries for `target_array`
                # Check bounds for offsets array
                if brep_idx >= len(offsets) - 1:
                    self._AddFailedCheck(
                        requirement=requirement,
                        message=f"Insufficient offsets for brep_idx {brep_idx}. Offsets length: {len(offsets)}, expected at least {brep_idx + 2}",
                        at=brep_array,
                    )
                    continue
                
                partition_start = offsets[brep_idx]
                partition_end = offsets[brep_idx + 1]

                # Validate indices within block against target partition
                if block is not None:
                    for idx in block:
                        # Handle Vec2i objects (like wireEdge:vertexIndices) which contain pairs of indices
                        if hasattr(idx, '__len__') and len(idx) == 2:
                            # For Vec2i objects, validate each element separately
                            for i, sub_idx in enumerate(idx):
                                if sub_idx < partition_start or sub_idx >= partition_end:
                                    self._AddFailedCheck(
                                        requirement=requirement,
                                        message=(
                                            f"{attr_name} contains invalid index {sub_idx} (element {i} of pair {idx}) in block #{brep_idx}. "
                                            f"Expected to be in range [{partition_start}, {partition_end})."
                                        ),
                                        at=brep_array,
                                    )
                        else:
                            # Handle single integer indices
                            if idx < partition_start or idx >= partition_end:
                                self._AddFailedCheck(
                                    requirement=requirement,
                                    message=(
                                        f"{attr_name} contains invalid index {idx} in block #{brep_idx}. "
                                        f"Expected to be in range [{partition_start}, {partition_end})."
                                    ),
                                    at=brep_array,
                                )

    ### Brep Validation Methods ###
    ### Validation of array authorship and lengths

    def _validate_brep_array(self, brep_array: Usd.Prim) -> None:
        """
        Validate all top-level BrepArray attributes and relationships, ensure tokens are valid, and that all arrays are consistent in size.
        """
        attributes_to_check = [
            "brep:intersectTol3d",
            "brep:extent",
            "brep:regionCount",
        ]
        
        # BA_005: All brep schema attributes must be authored
        self._validate_authorship_only(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_005)
        
        # BA_000: Validate size consistency for brep attributes (excluding brep:extent which has special size requirement)
        brep_standard_attributes = [attr for attr in attributes_to_check if attr != "brep:extent"]
        self._validate_array_sizes_and_authored(brep_array, brep_standard_attributes, cap.BrepArrayRequirements.BA_000, require_authored=False)
        
        # Special validation for brep:extent size (BA_000)
        brep_extent = brep_array.GetAttribute("brep:extent").Get()
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        if brep_extent is not None and brep_region_counts:
            expected_extent_size = len(brep_region_counts) * 2
            if len(brep_extent) != expected_extent_size:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_000,
                    message=f"brep:extent size ({len(brep_extent)}) does not match expected size ({expected_extent_size}) for {len(brep_region_counts)} Breps.",
                    at=brep_array,
                )

    def _validate_brep_tols(self, brep_array: Usd.Prim) -> None:
        """
        Validate the brep:intersectTol3d attribute, ensuring values are positive.
        """
        intersect_tol = brep_array.GetAttribute("brep:intersectTol3d")
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        tol_values = intersect_tol.Get()
        if tol_values is None:
            tol_values = []
        for brep_idx, tol in enumerate(tol_values):
            if tol is None or tol < BrepConstants.NUMERICAL_TOLERANCE:
                brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_010,
                    message=f"brep:intersectTol3d[{brep_id}] must be a positive value.",
                    at=brep_array,
                )

    def _validate_region_arrays(self, brep_array: Usd.Prim) -> None:
        """
        Validate all Region attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        attributes_to_check = [
            "region:shellCount",
            "region:type",
        ]
        # Validate relationships to regions
        region_counts = brep_array.GetAttribute("brep:regionCount").Get()
        region_count = sum(region_counts) if region_counts else 0

        # BA_070: All region schema attributes must be authored
        self._validate_authorship_only(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_070)
        
        # BA_065: Validate size consistency for region attributes 
        if region_count > 0:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_065, size=region_count, require_authored=False)
        else:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_065, size=region_count, require_authored=False)
        self._validate_allowed_tokens(brep_array, "region:type", ["solidRegion", "voidRegion"], cap.BrepArrayRequirements.BA_075, "region")

    def _validate_shell_arrays(self, brep_array: Usd.Prim) -> None:
        """
        Validate all shell attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        attributes_to_check = [
            "shell:faceuseCount",
            "shell:wireEdgeCount",
            "shell:pointType",
        ]

        shell_counts = brep_array.GetAttribute("region:shellCount").Get()
        shell_count = sum(shell_counts) if shell_counts else 0

        # BA_085: All shell schema attributes must be authored
        self._validate_authorship_only(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_085)
        
        # BA_080: Validate size consistency for shell attributes 
        if shell_count > 0:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_080, size=shell_count, require_authored=False)
        else:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_080, size=shell_count, require_authored=False)
        self._validate_allowed_tokens(brep_array, "shell:pointType", ["BrepPointAPI", "none"], cap.BrepArrayRequirements.BA_090, "shell")

    def _validate_faceuse_arrays(self, brep_array: Usd.Prim, partition_array: list[int]) -> None:
        """
        Validate all faceuse attributes. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        attributes_to_check = [
            "faceuse:faceIndex",
            "faceuse:orientationType",
        ]

        faceuse_counts = brep_array.GetAttribute("shell:faceuseCount").Get() or []
        try:
            faceuse_counts = [int(v) for v in faceuse_counts]
        except (TypeError, ValueError):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_091,
                message="shell:faceuseCount has invalid data type; expected integers.",
                at=brep_array,
            )
            faceuse_counts = []
        faceuse_count = sum(faceuse_counts) if faceuse_counts else 0

        # BA_105: All faceuse schema attributes must be authored
        self._validate_authorship_only(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_105)
        
        # BA_100: Validate size consistency for faceuse attributes 
        if faceuse_count > 0:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_100, size=faceuse_count, require_authored=False)
        else:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_100, size=faceuse_count, require_authored=False)
        self._validate_allowed_tokens(brep_array, "faceuse:orientationType", ["same", "opposite"], cap.BrepArrayRequirements.BA_110, "faceuse")
        
        shell_faceuse_count = faceuse_counts
        faceuse_arrays = {
            "faceuse:faceIndex": brep_array.GetAttribute("faceuse:faceIndex").Get(),
        }
        target_array = BrepConstants.safe_get_attribute(brep_array, "face:loopCount") or []
        
        # Compute brep-level faceuse counts - simplified approach for single brep
        region_counts = brep_array.GetAttribute("brep:regionCount").Get() or []
        
        if len(region_counts) == 1:
            # For single brep, sum all shell faceuse counts
            brep_faceuse_counts = [sum(shell_faceuse_count)] if shell_faceuse_count else [0]
        else:
            # For multiple breps, use the complex aggregation logic
            shell_counts_per_region = brep_array.GetAttribute("region:shellCount").Get() or []
            
            brep_faceuse_counts = []
            shell_idx = 0
            region_idx = 0
            
            for brep_idx in range(len(region_counts)):
                brep_faceuse_total = 0
                region_start = region_idx
                region_end = region_idx + region_counts[brep_idx]
                
                # Sum shell faceuse counts for all regions in this brep
                for r_idx in range(region_start, region_end):
                    if r_idx < len(shell_counts_per_region):
                        shells_in_region = shell_counts_per_region[r_idx]
                        for s_idx in range(shells_in_region):
                            if shell_idx < len(shell_faceuse_count):
                                brep_faceuse_total += shell_faceuse_count[shell_idx]
                                shell_idx += 1
                
                brep_faceuse_counts.append(brep_faceuse_total)
                region_idx = region_end
        
        self._validate_indexing_relationships(brep_faceuse_counts, faceuse_arrays, target_array, partition_array, brep_array, cap.BrepArrayRequirements.BA_115)

    def _validate_face_arrays(self, brep_array: Usd.Prim) -> None:
        """
        Validate all face attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        # Validate standard face attributes (size = number of faces)
        standard_face_attributes = [
            "face:loopCount",
            "face:trimType",
            "face:surfaceType",
        ]

        faceuse_face_indices = brep_array.GetAttribute("faceuse:faceIndex").Get() or []
        expected_faces = len(faceuse_face_indices) // 2 if faceuse_face_indices else 0

        # BA_125: All face schema attributes must be authored
        all_face_attributes = standard_face_attributes + ["face:range"]
        self._validate_authorship_only(brep_array, all_face_attributes, cap.BrepArrayRequirements.BA_125)
        
        # BA_120: Validate size consistency for face attributes
        if expected_faces > 0:
            self._validate_array_sizes_and_authored(brep_array, standard_face_attributes, cap.BrepArrayRequirements.BA_120, size=expected_faces, require_authored=False)
            
            # Validate face:range separately (size = 2 * number of faces)
            face_ranges = brep_array.GetAttribute("face:range").Get() or []
            if len(face_ranges) != expected_faces * 2:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_150,
                    message=f"face:range size mismatch. Expected {expected_faces * 2} elements (2 UV pairs per face), but got {len(face_ranges)}.",
                    at=brep_array,
                )
        else:
            # For empty models, validate that all arrays are consistently empty
            all_face_attributes = standard_face_attributes + ["face:range"]
            self._validate_array_sizes_and_authored(brep_array, all_face_attributes, cap.BrepArrayRequirements.BA_120, size=expected_faces, require_authored=False)

        self._validate_allowed_tokens(brep_array, "face:surfaceType", ["BrepSurfaceNurbAPI", "BrepSurfaceSphereAPI", "BrepSurfacePlaneAPI", "BrepSurfaceCylinderAPI", "BrepSurfaceConeAPI", "BrepSurfaceTorusAPI"], cap.BrepArrayRequirements.BA_130, "face")
        self._validate_allowed_tokens(brep_array, "face:trimType", ["rectangular", "general"], cap.BrepArrayRequirements.BA_135, "face")

    def _validate_loop_arrays(self, brep_array: Usd.Prim) -> None:
        """
        Validate all loop attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        attributes_to_check = [
            "loop:edgeuseCount",
            "loop:vertexIndex",
        ]
        
        face_loop_count_array = brep_array.GetAttribute("face:loopCount").Get() or []
        expected_size = sum(face_loop_count_array) if face_loop_count_array else 0

        # BA_170: All loop schema attributes must be authored
        self._validate_authorship_only(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_170)
        
        # BA_165: Validate size consistency for loop attributes 
        if expected_size > 0:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_165, size=expected_size, require_authored=False)
        else:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_165, size=expected_size, require_authored=False)

        # # Validate loop:vertexIndex when loop:edgeuseCount is zero

        # target_array = BrepConstants.safe_get_attribute(brep_array, "vertex:pointType") or []
        # num_vertices = len(target_array)

        # for edgeuse_count, loop_vertex in zip(brep_array.GetAttribute("loop:edgeuseCount").Get() or [], brep_array.GetAttribute("loop:vertexIndex").Get() or []):
        #     if edgeuse_count == 0 and (loop_vertex is None or loop_vertex < 0 or loop_vertex >= num_vertices):
        #         self._AddFailedCheck(
        #             requirement=cap.BrepArrayRequirements.BA_120,
        #             message=f"loop:vertexIndex must be a valid index when corresponding loop:edgeuseCount is equal to zero. Found loop:edgeuseCount={edgeuse_count} with invalid loop:vertexIndex={loop_vertex}.",
        #             at=brep_array,
        #         )

    def _validate_loop_vertex_index(self, brep_array: Usd.Prim, loop_offsets: list[int], vertex_offsets: list[int]) -> None:
        """
        Validate that all values in loop:vertexIndex for a Brep[ii] fall within the valid vertex range for that Brep
        and ensure that loop:vertexIndex is only valid if loop:edgeuseCount == 0.
        
        Note: The schema documentation states that when loop:edgeuseCount > 0, loop:vertexIndex values are ignored.

        Args:
            brep_array (Usd.Prim): The BrepArray prim.
            loop_offsets (list[int]): Start and end offsets of loop indices for each Brep.
            vertex_offsets (list[int]): Start and end offsets of vertex indices for each Brep.
        """
        # Retrieve required attributes
        loop_vertex_indices = brep_array.GetAttribute("loop:vertexIndex").Get() or []
        loop_edgeuse_counts = brep_array.GetAttribute("loop:edgeuseCount").Get() or []
        vertex_point_types = BrepConstants.safe_get_attribute(brep_array, "vertex:pointType") or []

        # Iterate over each Brep section as defined by the loop and vertex offsets
        for brep_idx, (loop_start, loop_end, vertex_start, vertex_end) in enumerate(
            zip(loop_offsets[:-1], loop_offsets[1:], vertex_offsets[:-1], vertex_offsets[1:])
        ):
            # Valid vertex *index* range for this brep
            if vertex_point_types is not None:
                max_index = len(vertex_point_types)
                safe_vertex_start = min(vertex_start, max_index)
                safe_vertex_end = min(vertex_end, max_index)
            else:
                safe_vertex_start = safe_vertex_end = 0

            # Clamp to available data to avoid IndexError on malformed arrays
            max_loop_count = min(loop_end, len(loop_edgeuse_counts), len(loop_vertex_indices))
            if max_loop_count < loop_end:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_175,
                    message=(
                        f"loop:edgeuseCount/loop:vertexIndex missing data for Brep[{brep_idx}] "
                        f"(expected loops up to {loop_end}, but only {max_loop_count} entries are available)."
                    ),
                    at=brep_array,
                )

            for loop_idx in range(loop_start, max_loop_count):
                edgeuse_count = loop_edgeuse_counts[loop_idx]

                if edgeuse_count == 0:  # Only validate loop:vertexIndex if edgeuseCount == 0
                    vertex_idx = loop_vertex_indices[loop_idx]
                    if vertex_idx < safe_vertex_start or vertex_idx >= safe_vertex_end:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_175,
                            message=(
                                f"Invalid loop:vertexIndex `{vertex_idx}` for Brep[{brep_idx}] when loop:edgeuseCount == 0. "
                                f"Expected to be in the vertex range [{vertex_start}, {vertex_end})."
                            ),
                            at=brep_array,
                        )

    def _validate_edgeuse_arrays(self, brep_array: Usd.Prim, edge_partition_array: list[int], edgeuse_partition_array: list[int]) -> None:
        """
        Validate all edgeuse attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        attributes_to_check = [
            "edgeuse:edgeIndex",
            "edgeuse:orientationType",
            "edgeuse:nextRadialEUIndex",
            "edgeuse:thisRadialEntryType",
        ]

        loop_edgeuse_counts = brep_array.GetAttribute("loop:edgeuseCount").Get() or []
        expected_size = sum(loop_edgeuse_counts) if loop_edgeuse_counts else 0

        
        # BA_180: Validate size consistency for edgeuse attributes 
        if expected_size > 0:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_180, size=expected_size, require_authored=False)
        else:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_180, size=expected_size, require_authored=False)
        self._validate_allowed_tokens(brep_array, "edgeuse:orientationType", ["same", "opposite"], cap.BrepArrayRequirements.BA_190, "edgeuse")
        self._validate_allowed_tokens(brep_array, "edgeuse:thisRadialEntryType", ["topEntry", "bottomEntry"], cap.BrepArrayRequirements.BA_195, "edgeuse")

        # Validate hierarchy between edgeuses and parent loops
        edgeuse_arrays = {
            "edgeuse:edgeIndex": self._get_attr_sequence(
                brep_array,
                "edgeuse:edgeIndex",
                cap.BrepArrayRequirements.BA_185,
                "uint[]",
            )[0],
        }
        target_array, _ = self._get_attr_sequence(
            brep_array,
            "edge:curveType",
            cap.BrepArrayRequirements.BA_215,
            "token[]",
        )
        
        # Compute edgeuse counts per brep from the edgeuse partition array
        brep_edgeuse_counts = []
        for i in range(len(edgeuse_partition_array) - 1):
            brep_edgeuse_counts.append(edgeuse_partition_array[i + 1] - edgeuse_partition_array[i])
        
        # If we have edgeuses but no breps computed, assume single brep
        if len(loop_edgeuse_counts) > 0 and len(brep_edgeuse_counts) == 0:
            brep_edgeuse_counts = [sum(loop_edgeuse_counts)]
        
        self._validate_indexing_relationships(brep_edgeuse_counts, edgeuse_arrays, target_array, edge_partition_array, brep_array, cap.BrepArrayRequirements.BA_205)

        # Validate hierarchy between edgeuses and parent loops
        edgeuse_arrays = {
            "edgeuse:nextRadialEUIndex": self._get_attr_sequence(
                brep_array,
                "edgeuse:nextRadialEUIndex",
                cap.BrepArrayRequirements.BA_185,
                "uint[]",
            )[0],
        }
        target_array, _ = self._get_attr_sequence(
            brep_array,
            "edgeuse:edgeIndex",
            cap.BrepArrayRequirements.BA_185,
            "uint[]",
        )
        self._validate_indexing_relationships(brep_edgeuse_counts, edgeuse_arrays, target_array, edgeuse_partition_array, brep_array, cap.BrepArrayRequirements.BA_200)

    def _validate_edge_arrays(self, brep_array: Usd.Prim, brep_offsets: dict, edge_partition_array: list[int], vertex_partition_array: list[int]) -> None:
        """
        Validate all edge attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        # Validate edge array sizes - edge:range is special (flat array with 2 values per edge)
        edge_standard_attributes = [
            "edge:curveType", 
            "edge:vertexIndices",
        ]

        # Only validate edge attributes if there are edges expected (check if any edge arrays are authored)
        edge_vertex_indices, expected_edges = self._get_attr_sequence(
            brep_array,
            "edge:vertexIndices",
            cap.BrepArrayRequirements.BA_237,
            "int2[]",
        )
        
        
        # BA_210: Validate size consistency among standard edge attributes
        edge_ranges, edge_range_len = self._get_attr_sequence(
            brep_array,
            "edge:range",
            cap.BrepArrayRequirements.BA_237,
            "double[]",
        )

        if expected_edges > 0:
            # Validate standard edge arrays (should all have same size)
            self._validate_array_sizes_and_authored(brep_array, edge_standard_attributes, cap.BrepArrayRequirements.BA_210, require_authored=False)
            
            # BA_230: Validate edge:range per-edge structure (exactly 2 elements per edge)
            if edge_range_len != expected_edges * 2:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_230,
                    message=f"Invalid edge:range per-edge structure. Expected exactly 2 elements per edge ({expected_edges} edges × 2 = {expected_edges * 2} elements), but got {edge_range_len} elements.",
                    at=brep_array,
                )
        else:
            # For empty models, just check if arrays are consistently empty
            all_edge_attributes = edge_standard_attributes + ["edge:range"]
            self._validate_array_sizes_and_authored(brep_array, all_edge_attributes, cap.BrepArrayRequirements.BA_210, require_authored=False)
        self._validate_allowed_tokens(brep_array, "edge:curveType", ["BrepCurve3dNurbAPI", "BrepCurve3dCircleAPI", "BrepCurve3dLineAPI", "BrepCurve3dEllipseAPI"], cap.BrepArrayRequirements.BA_245, "edge")

        if edge_vertex_indices:
            # Get BREP information for enhanced error messaging
            brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
            valid_pairs = []

            # Check for any invalid edge vertex indices and provide BREP context when possible
            for edge_idx, indices in enumerate(edge_vertex_indices):
                try:
                    pair_len = len(indices)
                except TypeError:
                    # Non-iterable value
                    brep_idx = None
                    if edge_partition_array and len(edge_partition_array) > 1:
                        for i in range(len(edge_partition_array) - 1):
                            edge_start = edge_partition_array[i]
                            edge_end = edge_partition_array[i + 1]
                            if edge_start <= edge_idx < edge_end:
                                brep_idx = i
                                break
                    if brep_idx is not None:
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        brep_context = f" (edge #{edge_idx} in brep #{brep_id})"
                    else:
                        brep_context = f" (edge #{edge_idx})"
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_225,
                        message=f"edge:vertexIndices entry is not a 2-element index pair{brep_context}.",
                        at=brep_array,
                    )
                    return  # don't proceed to zip on malformed data

                if pair_len != 2:
                    # Determine which BREP this edge belongs to
                    brep_idx = None
                    if edge_partition_array and len(edge_partition_array) > 1:
                        for i in range(len(edge_partition_array) - 1):
                            edge_start = edge_partition_array[i]
                            edge_end = edge_partition_array[i + 1]
                            if edge_start <= edge_idx < edge_end:
                                brep_idx = i
                                break
                    
                    # Use actual BREP user ID if available, otherwise fall back to index
                    if brep_idx is not None:
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        brep_context = f" (edge #{edge_idx} in brep #{brep_id})"
                    else:
                        brep_context = f" (edge #{edge_idx})"
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_225,
                        message=f"Each entry in edge:vertexIndices must contain exactly two vertex indices{brep_context}.",
                        at=brep_array,
                    )
                    return  # Only report the first violation; avoid further processing

                valid_pairs.append(indices)

            # Validate that all vertex indices in edge:vertexIndices are valid
            target_array, _ = self._get_attr_sequence(
                brep_array,
                "vertex:pointType",
                cap.BrepArrayRequirements.BA_316,
                "int[]",
            )
            if not valid_pairs:
                return
            first_index, second_index = zip(*valid_pairs, strict=True)
            
            # Compute edge counts per brep from the edge partition array
            edge_counts_per_brep = [
                edge_partition_array[i + 1] - edge_partition_array[i]
                for i in range(len(edge_partition_array) - 1)
            ]
            # If we have edges but no per-brep partitioning, assume a single brep
            if not edge_counts_per_brep:
                edge_counts_per_brep = [len(valid_pairs)]
            
            self._validate_indexing_relationships(
                edge_counts_per_brep,
                {"first_vertex": list(first_index), "second_vertex": list(second_index)},
                target_array,
                vertex_partition_array,
                brep_array,
                cap.BrepArrayRequirements.BA_225,
            )
        
        # Validate edge:range ordering (size already validated above)
        # Process pairs from the flat array to validate ordering
        # (Size validation already done above - must be exactly expected_edges * 2)
        
        # Get BREP information for enhanced error messaging
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        for edge_idx in range(0, len(edge_ranges), 2):
            if edge_idx + 1 >= len(edge_ranges):
                break
            edge_range = [edge_ranges[edge_idx], edge_ranges[edge_idx + 1]]
            try:
                edge_min = float(edge_range[0])
                edge_max = float(edge_range[1])
            except (TypeError, ValueError):
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_235,
                    message=f"edge:range contains non-numeric values at indices {edge_idx} and {edge_idx + 1}: {edge_range}.",
                    at=brep_array,
                )
                continue
            
            # Determine which BREP this edge belongs to
            brep_idx = None
            if edge_partition_array and len(edge_partition_array) > 1:
                for i in range(len(edge_partition_array) - 1):
                    edge_start = edge_partition_array[i]
                    edge_end = edge_partition_array[i + 1] 
                    if edge_start <= (edge_idx // 2) < edge_end:
                        brep_idx = i
                        break
            
            # Validate ordering: min <= max
            if edge_max < edge_min - BrepConstants.NUMERICAL_TOLERANCE:
                # Use actual BREP user ID if available, otherwise fall back to index
                if brep_idx is not None:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    brep_context = f" in brep #{brep_id}"
                else:
                    brep_context = ""
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_235,
                    message=f"Invalid edge:range order ({edge_range}) for edge #{edge_idx//2}{brep_context}. "
                            "Ensure the range is specified as (min, max) where min <= max.",
                    at=brep_array,
                )

    def _validate_wireEdge_arrays(self, brep_array: Usd.Prim, vertices_partition: list[int], wireedge_partition: list[int]) -> None:
        """
        Validate all wire edge attributes and relationships. Ensure tokens are valid, and that all arrays are consistent in size.
        """
        # Retrieve the shell:wireEdgeCount attribute
        wire_edge_counts = brep_array.GetAttribute("shell:wireEdgeCount").Get() or []

        # Compute the total by summing the counts
        total_wire_edges = sum(wire_edge_counts) if wire_edge_counts else 0

        wire_edge_attributes = [
            "wireEdge:curveType", 
            "wireEdge:range",
            "wireEdge:vertexIndices",
        ]

        if total_wire_edges == 0:
            # If no wire edges globally, check if any attributes have data when they shouldn't
            for attr_name in wire_edge_attributes:
                attr = brep_array.GetAttribute(attr_name)
                values = attr.Get() or []
                
                if values:  # Check if the attribute contains any data
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_250,
                        message=f"Wire edge attribute {attr_name} should be empty given shell:wireEdgeCount, but contains data: {values}.",
                        at=brep_array,
                    )
        
        wireedge_standard_attributes = [
            "wireEdge:curveType",
            "wireEdge:vertexIndices",
        ]
        all_wireedge_attributes = wireedge_standard_attributes + ["wireEdge:range"]

        # BA_255: Wire-edge topology attributes are all-or-none; required when W > 0
        authored_by_name = {
            attr_name: brep_array.GetAttribute(attr_name).IsAuthored()
            for attr_name in all_wireedge_attributes
        }
        authored_count = sum(authored_by_name.values())
        if authored_count > 0 and authored_count < len(all_wireedge_attributes):
            authored_names = [name for name, is_authored in authored_by_name.items() if is_authored]
            missing_names = [name for name, is_authored in authored_by_name.items() if not is_authored]
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_255,
                message=(
                    "Wire edge topology attributes must be authored together or all omitted. "
                    f"Authored: {authored_names}; not authored: {missing_names}."
                ),
                at=brep_array,
            )
        elif total_wire_edges > 0 and authored_count < len(all_wireedge_attributes):
            for attr_name in all_wireedge_attributes:
                if not authored_by_name[attr_name]:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_255,
                        message=(
                            f"{attr_name} is not authored in BrepArray but shell:wireEdgeCount "
                            f"requires {total_wire_edges} wire edge(s)."
                        ),
                        at=brep_array,
                    )
        
        if total_wire_edges > 0:
            """
            Validate all wire edge attributes and relationships.
            """
            # BA_250: Validate size consistency among standard wireEdge attributes
            self._validate_array_sizes_and_authored(brep_array, wireedge_standard_attributes, cap.BrepArrayRequirements.BA_250, size=total_wire_edges, require_authored=False)
            
            # BA_270: Validate wireEdge:range per-edge structure (exactly 2 elements per wire edge)
            wire_edge_ranges = brep_array.GetAttribute("wireEdge:range").Get() or []
            if len(wire_edge_ranges) != total_wire_edges * 2:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_270,
                    message=f"Invalid wireEdge:range per-edge structure. Expected exactly 2 elements per wire edge ({total_wire_edges} wire edges × 2 = {total_wire_edges * 2} elements), but got {len(wire_edge_ranges)} elements.",
                    at=brep_array,
                )
            self._validate_allowed_tokens(brep_array, "wireEdge:curveType", ["BrepCurve3dNurbAPI", "BrepCurve3dCircleAPI", "BrepCurve3dLineAPI", "BrepCurve3dEllipseAPI"], cap.BrepArrayRequirements.BA_260, "wireEdge")

            # Validate hierarchy between wire edges and parent shells
            wireEdge_counts = brep_array.GetAttribute("shell:wireEdgeCount").Get()
            wireEdge_arrays = {
                "wireEdge:vertexIndices": brep_array.GetAttribute("wireEdge:vertexIndices").Get(),
            }
            target_array = BrepConstants.safe_get_attribute(brep_array, "vertex:pointType") or []
            self._validate_indexing_relationships(wireEdge_counts, wireEdge_arrays, target_array, vertices_partition, brep_array, cap.BrepArrayRequirements.BA_265)
            
            # Validate wireEdge:range ordering (size already validated above)
            wire_edge_ranges = brep_array.GetAttribute("wireEdge:range").Get() or []
            
            # Get BREP information for enhanced error messaging
            brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
            
            # Process pairs from the flat array to validate ordering
            # (Size validation already done above - must be exactly total_wire_edges * 2)
            for wire_edge_idx in range(0, len(wire_edge_ranges), 2):
                if wire_edge_idx + 1 >= len(wire_edge_ranges):
                    break
                wire_edge_range = [wire_edge_ranges[wire_edge_idx], wire_edge_ranges[wire_edge_idx + 1]]
                
                # Determine which BREP this wireEdge belongs to
                brep_idx = None
                if wireedge_partition and len(wireedge_partition) > 1:
                    for i in range(len(wireedge_partition) - 1):
                        wireedge_start = wireedge_partition[i]
                        wireedge_end = wireedge_partition[i + 1] 
                        if wireedge_start <= (wire_edge_idx // 2) < wireedge_end:
                            brep_idx = i
                            break
                
                # Validate ordering: min <= max
                if wire_edge_range[1] < wire_edge_range[0] - BrepConstants.NUMERICAL_TOLERANCE:
                    # Use actual BREP user ID if available, otherwise fall back to index
                    if brep_idx is not None:
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        brep_context = f" in brep #{brep_id}"
                    else:
                        brep_context = ""
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_275,
                        message=f"Invalid wireEdge:range order ({wire_edge_range}) for wireEdge #{wire_edge_idx//2}{brep_context}. "
                                "Ensure the range is specified as (min, max) where min <= max.",
                        at=brep_array,
                    )

    def _validate_vertex_arrays(self, brep_array: Usd.Prim) -> None:
        """
        Validate all vertex attributes and relationships.
        """
        attributes_to_check = [
            "vertex:pointType",
        ]

        # BA_300: All vertex schema attributes must be authored
        self._validate_authorship_only(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_300)
        
        # BA_295: Validate size consistency for vertex attributes
        # Compute expected vertex count from edge:vertexIndices (unique vertices referenced)
        edge_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "edge:vertexIndices") or []
        if edge_vertex_indices:
            all_vertex_indices = set()
            for pair in edge_vertex_indices:
                # Each pair is a Vec2i/int2, access indices via indexing
                try:
                    all_vertex_indices.add(int(pair[0]))
                    all_vertex_indices.add(int(pair[1]))
                except (TypeError, IndexError):
                    pass
            expected_vertices = max(all_vertex_indices) + 1 if all_vertex_indices else 0
        else:
            expected_vertices = 0
        
        if expected_vertices > 0:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_295, size=expected_vertices, require_authored=False)
        else:
            self._validate_array_sizes_and_authored(brep_array, attributes_to_check, cap.BrepArrayRequirements.BA_295, require_authored=False)
        self._validate_allowed_tokens(brep_array, "vertex:pointType", ["BrepPointAPI"], cap.BrepArrayRequirements.BA_315, "vertex")

    ### Validation of Specific Constraints ###

    def _validate_face_loop_count_minimum(self, brep_array: Usd.Prim):
        """
        Validate that all face:loopCount entries are at least one. Validates with brep context.
        """
        loop_counts = brep_array.GetAttribute("face:loopCount").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not loop_counts or not brep_region_counts:
            return

        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        face_offsets = brep_offsets.get("faces", [])

        # Validate that each face has at least one loop, with brep context
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(face_offsets) - 1:
                break
                
            face_start = face_offsets[brep_idx]
            face_end = face_offsets[brep_idx + 1]
            
            # Validate faces belonging to this brep
            for face_idx in range(face_start, min(face_end, len(loop_counts))):
                loop_count = loop_counts[face_idx]
                if loop_count < 1:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_140,
                        message=f"Face #{face_idx} in brep #{brep_id} has loopCount = {loop_count}, but each face must have at least one loop.",
                        at=brep_array,
                    )


    def _validate_radial_relationships(self, brep_array: Usd.Prim):
        """
        Validate radial relationships for `edgeuse:nextRadialEUIndex` and `edgeuse:thisRadialEntryType`. Validates with brep context.
        """
        next_radial_indices = brep_array.GetAttribute("edgeuse:nextRadialEUIndex").Get() or []
        entry_types = brep_array.GetAttribute("edgeuse:thisRadialEntryType").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not next_radial_indices or not brep_region_counts:
            return

        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        edgeuse_offsets = brep_offsets.get("edgeuses", [])

        # Validate radial relationships with brep context
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(edgeuse_offsets) - 1:
                break
                
            edgeuse_start = edgeuse_offsets[brep_idx]
            edgeuse_end = edgeuse_offsets[brep_idx + 1]
            
            # Validate edgeuses belonging to this brep
            for edgeuse_idx in range(edgeuse_start, min(edgeuse_end, len(next_radial_indices))):
                next_idx = next_radial_indices[edgeuse_idx]
                if next_idx >= len(next_radial_indices):  # Index out of range
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_200,
                        message=f"Invalid edgeuse:nextRadialEUIndex at edgeuse #{edgeuse_idx} in brep #{brep_id}: {next_idx}. Index is out of range.",
                        at=brep_array,
                    )

    def _validate_faceuse_pairing(self, brep_array: Usd.Prim):
        """
        Validate that every face is referenced by exactly 2 faceuses in faceuse:faceIndex.
        """
        faceuse_face_indices = brep_array.GetAttribute("faceuse:faceIndex").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not faceuse_face_indices or not brep_region_counts:
            return

        brep_offsets = self._compute_brep_offsets(brep_array)
        face_offsets = brep_offsets.get("faces", [])
        faceuse_offsets = brep_offsets.get("faceuses", [])

        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(faceuse_offsets) - 1 or brep_idx >= len(face_offsets) - 1:
                break

            fu_start = faceuse_offsets[brep_idx]
            fu_end = faceuse_offsets[brep_idx + 1]
            face_start = face_offsets[brep_idx]
            face_end = face_offsets[brep_idx + 1]

            ref_counts: dict[int, int] = {}
            for fu_idx in range(fu_start, min(fu_end, len(faceuse_face_indices))):
                fi = faceuse_face_indices[fu_idx]
                ref_counts[fi] = ref_counts.get(fi, 0) + 1

            brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

            for face_idx in range(face_start, face_end):
                count = ref_counts.get(face_idx, 0)
                if count != 2:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_580,
                        message=f"Face #{face_idx} in brep #{brep_id} is referenced by {count} faceuse(s), expected exactly 2.",
                        at=brep_array,
                    )

    def _validate_radial_edgeuse_closure(self, brep_array: Usd.Prim):
        """
        Validate that chasing edgeuse:nextRadialEUIndex from every edgeuse
        forms a closed circular chain that returns to the starting edgeuse.
        """
        next_radial_indices = brep_array.GetAttribute("edgeuse:nextRadialEUIndex").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not next_radial_indices or not brep_region_counts:
            return

        brep_offsets = self._compute_brep_offsets(brep_array)
        edgeuse_offsets = brep_offsets.get("edgeuses", [])
        total_edgeuses = len(next_radial_indices)

        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(edgeuse_offsets) - 1:
                break

            eu_start = edgeuse_offsets[brep_idx]
            eu_end = edgeuse_offsets[brep_idx + 1]
            brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

            for start_eu in range(eu_start, min(eu_end, total_edgeuses)):
                current = start_eu
                closed = False
                max_steps = eu_end - eu_start

                for _ in range(max_steps):
                    nxt = next_radial_indices[current]
                    if nxt >= total_edgeuses:
                        break
                    if nxt == start_eu:
                        closed = True
                        break
                    current = nxt

                if not closed:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_581,
                        message=(
                            f"Radial chain starting at edgeuse #{start_eu} in brep #{brep_id} "
                            f"does not close within {max_steps} steps."
                        ),
                        at=brep_array,
                    )

    def _validate_orphan_edges(self, brep_array: Usd.Prim):
        """
        Validate that every edge is referenced by at least one edgeuse.

        Uses edge array length for
        partition from _compute_brep_offsets, because the latter undercounts
        edges when orphans are present (it derives edge counts from edgeuse
        references).
        """
        edgeuse_edge_indices = brep_array.GetAttribute("edgeuse:edgeIndex").Get() or []
        edge_curve_types = BrepConstants.safe_get_attribute(brep_array, "edge:curveType") or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not edge_curve_types or not brep_region_counts:
            return

        total_edges = len(edge_curve_types)
        referenced_edges: set[int] = set(edgeuse_edge_indices)

        brep_offsets = self._compute_brep_offsets(brep_array)
        edge_offsets = brep_offsets.get("edges", [])

        for edge_idx in range(total_edges):
            if edge_idx not in referenced_edges:
                brep_idx = None
                for i in range(len(edge_offsets) - 1):
                    if edge_offsets[i] <= edge_idx < edge_offsets[i + 1]:
                        brep_idx = i
                        break

                if brep_idx is not None:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    context = f" in brep #{brep_id}"
                else:
                    context = ""

                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_582,
                    message=f"Edge #{edge_idx}{context} is not referenced by any edgeuse (orphan edge).",
                    at=brep_array,
                )

    ### Validation of geometry relationships ###
    ### Brep level geometric data validation ###

    def _validate_face_ranges(self, brep_array: Usd.Prim):
        """
        Validate face:range structure and nondegeneracy for unified UV ranges.
        Performs per-brep validation with proper error message context.
        """
        face_ranges_val = brep_array.GetAttribute("face:range").Get()
        if self._is_unregistered_value(face_ranges_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_145,
                message="face:range has an unregistered USD type; expected double2[].",
                at=brep_array,
            )
            return
        face_ranges = face_ranges_val or []

        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        # Get brep offsets for per-brep validation
        brep_offsets = self._compute_brep_offsets(brep_array)
        face_offsets = brep_offsets.get("faces", [])
        
        # BA_145: Valid face:range structure validation
        # Check that face:range is properly structured with valid double2[] data
        face_range_attr = brep_array.GetAttribute("face:range")
        if face_range_attr:
            face_range_type = str(face_range_attr.GetTypeName())
            if face_range_type != "double2[]":
                # If face:range is not double2[] type, this violates BA_145
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_145,
                    message=f"Invalid face:range type. Expected 'double2[]' but got '{face_range_type}'. face:range must be double2[] to ensure exactly 2 elements per range (UV min and max pairs).",
                    at=brep_array,
                )
            else:
                # face:range is double2[] - now validate the actual data structure
                for range_idx, face_range in enumerate(face_ranges):
                    # Check that each double2 has valid (non-NaN, non-infinite) components
                    if len(face_range) == 2:  # Should always be 2 for double2, but verify
                        u_val, v_val = face_range[0], face_range[1]
                        if math.isnan(u_val) or math.isnan(v_val):
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_145,
                                message=f"Invalid face:range structure at index {range_idx}. Contains NaN values ({u_val}, {v_val}). face:range must contain valid numeric UV pairs.",
                                at=brep_array,
                            )
                        elif math.isinf(u_val) or math.isinf(v_val):
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_145,
                                message=f"Invalid face:range structure at index {range_idx}. Contains infinite values ({u_val}, {v_val}). face:range must contain finite numeric UV pairs.",
                                at=brep_array,
                            )
                    else:
                        # This should never happen with double2[] but check anyway
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_145,
                            message=f"Invalid face:range structure at index {range_idx}. Expected exactly 2 elements per range (UV pair) but got {len(face_range)} elements.",
                            at=brep_array,
                        )
                # If we get here with double2[] and valid data, BA_145 is satisfied
        else:
            # If face:range attribute doesn't exist, that's a different issue (BA_125)
            # but we still technically fail BA_145 since there's no structure to validate
            pass
        
        # Continue with UV range validation (BA_155, BA_160) regardless of type check
        # face:range is double2[] with size = 2 * number of faces
        # Each face has 2 entries: UVmin=(u_min, v_min) and UVmax=(u_max, v_max)
        for face_idx, face_range in enumerate(face_ranges):
            if face_idx % 2 == 0 and face_idx + 1 < len(face_ranges):
                # Find which brep this face belongs to
                actual_face_idx = face_idx // 2  # Convert range pair index to actual face index
                brep_idx = 0
                local_face_idx = actual_face_idx
                for brep_idx, (start, end) in enumerate(zip(face_offsets[:-1], face_offsets[1:])):
                    if start <= actual_face_idx < end:
                        local_face_idx = actual_face_idx - start
                        break
                
                # This is a UVmin entry, next entry should be UVmax
                uv_min = face_range  # (u_min, v_min)
                uv_max = face_ranges[face_idx + 1]  # (u_max, v_max)
                
                # Validate U range nondegeneracy (BA_155)
                if uv_max[0] < uv_min[0] + BrepConstants.NUMERICAL_TOLERANCE:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_155,
                        message=f"Invalid face:range U values ({uv_min[0]}, {uv_max[0]}) for face #{local_face_idx} in brep #{brep_id}. "
                                "Ensure U range is specified as (Umin, Umax) where Umax > Umin.",
                        at=brep_array,
                    )
                
                # Validate V range nondegeneracy (BA_160)
                if uv_max[1] < uv_min[1] + BrepConstants.NUMERICAL_TOLERANCE:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_160,
                        message=f"Invalid face:range V values ({uv_min[1]}, {uv_max[1]}) for face #{local_face_idx} in brep #{brep_id}. "
                                "Ensure V range is specified as (Vmin, Vmax) where Vmax > Vmin.",
                        at=brep_array,
                    )

    def _validate_brep_extent(self, brep_array: Usd.Prim) -> None:
        """
        Validates the brep:extent attribute which contains bounding box corner pairs (XYZmin, XYZmax) for each brep.
        Ensures proper structure, ordering, and containment within the prim's extent.
        """
        prim_extent = brep_array.GetAttribute("extent").Get()
        min_extent = prim_extent[0] if prim_extent else None
        max_extent = prim_extent[1] if prim_extent else None    

        # Get brep:extent data
        brep_extents = BrepConstants.safe_get_attribute(brep_array, "brep:extent")
        
        # Get number of breps for size validation
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        num_breps = len(brep_region_counts)
        expected_extent_size = num_breps * 2  # 2 points per brep (XYZmin, XYZmax)
        
        # Validate structure (BA_004)
        if len(brep_extents) != expected_extent_size:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_020,
                message=f"Invalid brep:extent structure. Expected {expected_extent_size} elements (2 * {num_breps} Breps), but got {len(brep_extents)}.",
                at=brep_array,
            )
            return  # Can't proceed with validation if size is wrong
            
        # Validate each brep's extent 
        for brep_idx in range(num_breps):
            min_point_idx = brep_idx * 2
            max_point_idx = brep_idx * 2 + 1
            
            try:
                min_point = brep_extents[min_point_idx]
                max_point = brep_extents[max_point_idx]
                
                # Get BREP ID for error messages
                brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                
                # Validate that points are 3D
                if len(min_point) != 3 or len(max_point) != 3:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_020,
                        message=f"Invalid brep:extent structure for Brep #{brep_id}. Each point must have exactly 3 coordinates (XYZ).",
                        at=brep_array,
                    )
                    continue
                    
                # Validate ordering: separate requirement for each dimension
                # X dimension ordering (BA_005)
                if max_point[0] < min_point[0] - BrepConstants.NUMERICAL_TOLERANCE:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_025,
                        message=f"Invalid brep:extent X order for Brep #{brep_id}. "
                                f"Xmin ({min_point[0]}) must be <= Xmax ({max_point[0]}).",
                        at=brep_array,
                    )
                
                # Y dimension ordering (BA_006)
                if max_point[1] < min_point[1] - BrepConstants.NUMERICAL_TOLERANCE:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_030,
                        message=f"Invalid brep:extent Y order for Brep #{brep_id}. "
                                f"Ymin ({min_point[1]}) must be <= Ymax ({max_point[1]}).",
                        at=brep_array,
                    )
                
                # Z dimension ordering (BA_007)
                if max_point[2] < min_point[2] - BrepConstants.NUMERICAL_TOLERANCE:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_035,
                        message=f"Invalid brep:extent Z order for Brep #{brep_id}. "
                                f"Zmin ({min_point[2]}) must be <= Zmax ({max_point[2]}).",
                        at=brep_array,
                    )
                
                # Validate containment: separate requirement for each dimension
                # Use single-precision tolerances since prim extent is float3 while brep:extent is double3
                if min_extent is not None and max_extent is not None:
                    # X dimension containment (BA_008)
                    if (BrepConstants.isFloatLessThan(min_point[0], min_extent[0]) or 
                        BrepConstants.isFloatGreaterThan(max_point[0], max_extent[0])):
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_040,
                            message=f"brep:extent for Brep #{brep_id} is outside the prim's X extent. "
                                    f"Brep range: ({min_point[0]}, {max_point[0]}), Prim range: ({min_extent[0]}, {max_extent[0]}).",
                            at=brep_array,
                        )
                    
                    # Y dimension containment (BA_009)
                    if (BrepConstants.isFloatLessThan(min_point[1], min_extent[1]) or 
                        BrepConstants.isFloatGreaterThan(max_point[1], max_extent[1])):
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_045,
                            message=f"brep:extent for Brep #{brep_id} is outside the prim's Y extent. "
                                    f"Brep range: ({min_point[1]}, {max_point[1]}), Prim range: ({min_extent[1]}, {max_extent[1]}).",
                            at=brep_array,
                        )
                    
                    # Z dimension containment (BA_010)
                    if (BrepConstants.isFloatLessThan(min_point[2], min_extent[2]) or 
                        BrepConstants.isFloatGreaterThan(max_point[2], max_extent[2])):
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_050,
                            message=f"brep:extent for Brep #{brep_id} is outside the prim's Z extent. "
                                    f"Brep range: ({min_point[2]}, {max_point[2]}), Prim range: ({min_extent[2]}, {max_extent[2]}).",
                            at=brep_array,
                        )
                            
            except (TypeError, IndexError, AttributeError) as e:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_020,
                    message=f"Invalid brep:extent data type or structure for Brep #{brep_id}. Cannot access extent points: {e}.",
                    at=brep_array,
                )

    ### Validation of vertex geometry ###

    def _validate_point_position(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `brep:vertexPoint:point:position` attribute
        to ensure they match the vertex count.

        Validates the `brep:shellPoint:point:position` attribute
        to ensure it matches the number of true point-shell occurrences.
        """

        # Ensure the attribute is authored and non-empty
        point_type_attr = brep_array.GetAttribute("vertex:pointType")
        point_type_values = point_type_attr.Get() or []

        # Validate brep:vertexPoint:point:position counts and authorship
        correct_count = sum(1 for point_type in point_type_values if point_type == "BrepPointAPI")
        # only validate if there are any vertex points expected or if there are any authored
        if correct_count > 0 or len(brep_array.GetAttribute("brep:vertexPoint:point:position").Get() or []) > 0:
            self._validate_array_sizes_and_authored(brep_array, ["brep:vertexPoint:point:position"], cap.BrepArrayRequirements.BA_320, correct_count)

        # # Validate brep:vertexPoint:multiPoint:position counts and authorship
        # correct_mp_count = sum(1 for point_type in point_type_values if point_type == "BrepMultiPointAPI")
        # self._validate_array_sizes_and_authored(brep_array, ["brep:vertexPoint:multiPoint:position"], cap.BrepArrayRequirements.BA_440, correct_mp_count)

        # Validate brep:shellPoint:point:position counts and authorship
        shell_point_type_attr = brep_array.GetAttribute("shell:pointType")
        shell_point_type_values = shell_point_type_attr.Get() or []
        shell_faceuse_counts = BrepConstants.safe_get_attribute(brep_array, "shell:faceuseCount")
        shell_wireedge_counts = BrepConstants.safe_get_attribute(brep_array, "shell:wireEdgeCount")

        correct_sp_count = sum(
            1
            for shell_index in range(len(shell_point_type_values))
            if BrepConstants.is_brep_point_shell(
                shell_index,
                shell_point_type_values,
                shell_faceuse_counts,
                shell_wireedge_counts,
            )
        )
        # only validate if there are any shell points expected or if there are any authored
        if correct_sp_count > 0 or len(brep_array.GetAttribute("brep:shellPoint:point:position").Get() or []) > 0:
            self._validate_array_sizes_and_authored(brep_array, ["brep:shellPoint:point:position"], cap.BrepArrayRequirements.BA_325, correct_sp_count)

        # # Validate brep:shellPoint:multiPoint:position counts and authorship
        # correct_smp_count = sum(1 for point_type in shell_point_type_values if point_type == "BrepMultiPointAPI")  
        # self._validate_array_sizes_and_authored(brep_array, ["brep:shellPoint:multiPoint:position"], cap.BrepArrayRequirements.BA_440, correct_smp_count)

    ### Validation of 3d curve geometry ###

    def _validate_curve3d_nurb_order_vertex_count(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `brep:edge3dNurb:curve3d:nurb:order` attribute to ensure it is authored, non-empty, and each value is positive
        and does not exceed the corresponding vertexCount. Validates per brep.
        """
        
        edge_type_attr = brep_array.GetAttribute("edge:curveType")
        edge_type_values = edge_type_attr.Get() or []

        correct_count = sum(1 for edge_type in edge_type_values if edge_type == "BrepCurve3dNurbAPI")

        # only validate if there are any NURB curves expected or if there are any authored
        if correct_count > 0 or len(brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get() or []) > 0:
            # Validate that the order attribute is authored and non-empty
            self._validate_array_sizes_and_authored(
                brep_array=brep_array,
                attributes=["brep:edge3dNurb:curve3d:nurb:order", "brep:edge3dNurb:curve3d:nurb:vertexCount"],
                requirement=cap.BrepArrayRequirements.BA_330,
                size=correct_count,
            )

        orders_val = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get()
        if self._is_unregistered_value(orders_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_330,
                message="brep:edge3dNurb:curve3d:nurb:order has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return
        orders = orders_val or []

        vertex_counts_val = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:vertexCount").Get()
        if self._is_unregistered_value(vertex_counts_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_330,
                message="brep:edge3dNurb:curve3d:nurb:vertexCount has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return
        vertex_counts = vertex_counts_val or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not orders or not vertex_counts or not brep_region_counts:
            return

        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        nurbs_curve_offsets = brep_offsets.get("edge3d_nurbs_curves", [])

        # Validate per brep
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(nurbs_curve_offsets) - 1:
                break
                
            curve_start = nurbs_curve_offsets[brep_idx]
            curve_end = nurbs_curve_offsets[brep_idx + 1]
            
            # Validate NURBS curves belonging to this brep
            for curve_idx in range(curve_start, min(curve_end, len(orders), len(vertex_counts))):
                order = orders[curve_idx]
                vertex_count = vertex_counts[curve_idx]
                
                if order <= 0:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_335,
                        message=f"Invalid brep:edge3dNurb:curve3d:nurb:order {order} for curve3d #{curve_idx} in brep #{brep_id}. brep:edge3dNurb:curve3d:nurb:order must be positive.",
                        at=brep_array,
                    )
                elif order > vertex_count:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_340,
                        message=f"Invalid brep:edge3dNurb:curve3d:nurb:order {order} for curve3d #{curve_idx} in brep #{brep_id}. brep:edge3dNurb:curve3d:nurb:order must not exceed brep:edge3dNurb:curve3d:nurb:vertexCount {vertex_count}.",
                        at=brep_array,
                    )

    def _validate_curve3d_nurb_control_vertices_weights(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `curve3d:nurb:controlVertices` attribute to ensure it is authored, non-empty,
        and matches the expected total count based on curve3d:nurb:vertexCount. Validates per brep.
        """
        vertex_counts_val = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:vertexCount").Get()
        weights_val = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:weights").Get()
        control_vertices_val = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:controlVertices").Get()
        vertex_counts = vertex_counts_val if vertex_counts_val is not None else []
        weights = weights_val if weights_val is not None else []
        control_vertices = control_vertices_val if control_vertices_val is not None else []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if self._is_unregistered_value(vertex_counts):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:vertexCount has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return
        if self._is_unregistered_value(weights):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:weights has an unregistered USD type; expected double[].",
                at=brep_array,
            )
            return

        vertex_counts_len = self._len_or_none(vertex_counts)
        if vertex_counts_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:vertexCount is not a valid sequence; expected uint[].",
                at=brep_array,
            )
            return
        weights_len = self._len_or_none(weights)
        if weights_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_345,
                message="brep:edge3dNurb:curve3d:nurb:weights is not a valid sequence; expected double[].",
                at=brep_array,
            )
            return

        # Validate the size of controlVertices matches the sum of vertexCount
        try:
            expected_size = sum(vertex_counts)
        except (TypeError, ValueError):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:vertexCount contains non-integer data; expected uint[].",
                at=brep_array,
            )
            return

        # only validate if there are any NURB curves expected or if there are any authored
        if expected_size > 0 or len(weights) > 0:
            # Ensure the attribute is authored
            self._validate_array_sizes_and_authored(
                brep_array=brep_array,
                attributes=["brep:edge3dNurb:curve3d:nurb:weights","brep:edge3dNurb:curve3d:nurb:controlVertices"],
                requirement=cap.BrepArrayRequirements.BA_345,
                size=expected_size
            )

        if self._is_unregistered_value(control_vertices):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:controlVertices has an unregistered USD type; expected point3d[].",
                at=brep_array,
            )
            return

        actual_cv_len = self._len_or_none(control_vertices)
        if actual_cv_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_345,
                message=f"brep:edge3dNurb:curve3d:nurb:controlVertices is not a valid sequence; expected size {expected_size}.",
                at=brep_array,
            )
        elif actual_cv_len and actual_cv_len != expected_size:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_345,
                message=f"Invalid size for brep:edge3dNurb:curve3d:nurb:controlVertices. Expected {expected_size}, but got {actual_cv_len}.",
                at=brep_array,
            )

        if not weights or not brep_region_counts:
            return

        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        control_vertex_offsets = brep_offsets.get("edge_control_vertices", [])

        # Validate per brep
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(control_vertex_offsets) - 1:
                break
                
            weight_start = control_vertex_offsets[brep_idx]
            weight_end = control_vertex_offsets[brep_idx + 1]
            
            # Validate weights belonging to this brep
            for weight_idx in range(weight_start, min(weight_end, len(weights))):
                weight = weights[weight_idx]
                if weight < BrepConstants.NUMERICAL_TOLERANCE:  # Allowing for floating point precision issues
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_350,
                        message=f"Invalid weight at index #{weight_idx} in brep #{brep_id}. Weights must be positive.",
                        at=brep_array,
                    )

    def _validate_curve3d_knots(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `brep:edge3dNurb:curve3d:nurb:knots` attribute to ensure it is authored, non-empty,
        and follows the expected rules for knot vector size and ordering. Validates per brep.
        """
        knots_val = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:knots").Get()
        if self._is_unregistered_value(knots_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:knots has an unregistered USD type; expected double[].",
                at=brep_array,
            )
            return
        knots = knots_val or []

        vertex_counts = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:vertexCount").Get() or []
        orders = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if self._is_unregistered_value(vertex_counts):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:vertexCount has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return
        if self._is_unregistered_value(orders):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:order has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return

        vertex_counts_len = self._len_or_none(vertex_counts)
        if vertex_counts_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:vertexCount is not a valid sequence; expected uint[].",
                at=brep_array,
            )
            return
        orders_len = self._len_or_none(orders)
        if orders_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_371,
                message="brep:edge3dNurb:curve3d:nurb:order is not a valid sequence; expected uint[].",
                at=brep_array,
            )
            return

        if not knots or not vertex_counts or not orders or not brep_region_counts:
            return

        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        nurbs_curve_offsets = brep_offsets.get("edge3d_nurbs_curves", [])

        # Validate knot vector size and ordering per brep
        global_offset = 0
        global_curve_idx = 0
        
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(nurbs_curve_offsets) - 1:
                break
                
            curve_start = nurbs_curve_offsets[brep_idx]
            curve_end = nurbs_curve_offsets[brep_idx + 1]
            
            # Validate NURBS curves belonging to this brep
            for local_curve_idx in range(curve_start, min(curve_end, len(vertex_counts), len(orders))):
                vertex_count = vertex_counts[local_curve_idx]
                order = orders[local_curve_idx]
                expected_knot_count = vertex_count + order
                curve_knots = knots[global_offset : global_offset + expected_knot_count]
                try:
                    curve_knots = list(curve_knots or [])
                except TypeError:
                    curve_knots = []
                
                if len(curve_knots) != expected_knot_count:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_355,
                        message=f"Invalid knot count for curve3d #{local_curve_idx} in brep #{brep_id}. Expected {expected_knot_count} knots, but got {len(curve_knots)}.",
                        at=brep_array,
                    )

                if any(y < (x-BrepConstants.NUMERICAL_TOLERANCE) for x, y in zip(curve_knots, curve_knots[1:])): # Allowing for floating point precision issues
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_360,
                        message=f"Invalid knot ordering for curve3d #{local_curve_idx} in brep #{brep_id}. Knots must be non-decreasing.",
                        at=brep_array,
                    )

                global_offset += expected_knot_count

    ### Validation of 2d curve geometry ###

    def _validate_curveUv_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates the optional `brep:curveUv` attributes. These are optional, and the validation
        is performed only if the attributes are authored. Checks include:
        - Consistency of `brep:curveUv:nurb:vertexCount` and related attributes.
        - Slicing of each curve's data members (`nurb:knots`, `nurb:weights`).
        - Correct size and values for all attributes, including non-decreasing knots and positive weights.
        - Per-brep validation with proper error message context.

        Args:
            brep_array (Usd.Prim): The BrepArray prim to validate.
        """
        # List of optional attributes for brep:curveUv
        curveUv_attributes = [
            "brep:curveUv:nurb:vertexCount",
            "brep:curveUv:nurb:order",
        ]

        # Check if any brep:curveUv attributes are authored
        is_curveUv_authored = any([brep_array.GetAttribute(attr).IsAuthored() for attr in curveUv_attributes])

        if not is_curveUv_authored:
            return  # Exit early if no brep:curveUv attributes are authored
        
        correct_count = len(brep_array.GetAttribute("edgeuse:edgeIndex").Get() or [])
        # Validate that all brep:curveUv attributes are authored and non-empty
        self._validate_array_sizes_and_authored(
            brep_array=brep_array,
            attributes=curveUv_attributes,
            requirement=cap.BrepArrayRequirements.BA_375,
            size=correct_count,
        )

        # Get brep offsets for per-brep validation
        brep_offsets = self._compute_brep_offsets(brep_array)
        curveUv_offsets = brep_offsets.get("curveUv", [])
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        #### Validate brep:curveUv:nurb:order and brep:curveUv:nurb:vertexCount ####
        vertex_counts_val = brep_array.GetAttribute("brep:curveUv:nurb:vertexCount").Get()
        orders_val = brep_array.GetAttribute("brep:curveUv:nurb:order").Get()
        control_vertices_val = brep_array.GetAttribute("brep:curveUv:nurb:controlVertices").Get()
        vertex_counts = vertex_counts_val if vertex_counts_val is not None else []
        orders = orders_val if orders_val is not None else []
        control_vertices = control_vertices_val if control_vertices_val is not None else []

        if self._is_unregistered_value(control_vertices_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:controlVertices has an unregistered USD type; expected double2[].",
                at=brep_array,
            )
            return

        if self._is_unregistered_value(vertex_counts):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:vertexCount has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return
        if self._is_unregistered_value(orders):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:order has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return

        vertex_counts_len = self._len_or_none(vertex_counts)
        if vertex_counts_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:vertexCount is not a valid sequence; expected uint[].",
                at=brep_array,
            )
            return
        orders_len = self._len_or_none(orders)
        if orders_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:order is not a valid sequence; expected uint[].",
                at=brep_array,
            )
            return
        
        if orders:
            for brep_idx, (start_idx, end_idx) in enumerate(zip(curveUv_offsets[:-1], curveUv_offsets[1:])):
                for local_curve_idx, curve_idx in enumerate(range(start_idx, end_idx)):
                    if curve_idx >= len(orders) or curve_idx >= len(vertex_counts):
                        continue
                    
                    order = orders[curve_idx]
                    vertex_count = vertex_counts[curve_idx]
                    
                    if order == 0 and vertex_count == 0:
                        # Special case: Both order and vertex_count are zero, indicating no UV trim curve
                        continue
                    elif order <= 0:
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_380,
                            message=f"Invalid brep:curveUv:nurb:order {order} for curveUv #{local_curve_idx} in brep #{brep_id}. "
                                    f"Order must be positive.",
                            at=brep_array,
                        )
                    elif order > vertex_count:
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_385,
                            message=f"Invalid brep:curveUv:nurb:order {order} for curveUv #{local_curve_idx} in brep #{brep_id}. "
                                    f"Order must not exceed vertexCount {vertex_count}.",
                            at=brep_array,
                        )
        
        #### Validate brep:curveUv:nurb:controlVertices ####
        try:
            expected_total_control_vertices = sum(vertex_counts)
        except (TypeError, ValueError):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:vertexCount contains non-integer data; expected uint[].",
                at=brep_array,
            )
            return
        actual_len = self._len_or_none(control_vertices)
        if actual_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_390,
                message=f"brep:curveUv:nurb:controlVertices is not a valid sequence; expected size {expected_total_control_vertices}.",
                at=brep_array,
            )
        elif actual_len != expected_total_control_vertices:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_390,
                message=f"Invalid size for brep:curveUv:nurb:controlVertices. "
                        f"Expected size {expected_total_control_vertices}, but got {actual_len}.",
                at=brep_array,
            )

        #### Validate brep:curveUv:nurb:knots ####
        knots_val = brep_array.GetAttribute("brep:curveUv:nurb:knots").Get()
        if self._is_unregistered_value(knots_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:knots has an unregistered USD type; expected double[].",
                at=brep_array,
            )
            return
        knots = knots_val or []
        if knots:
            offset = 0
            for brep_idx, (start_idx, end_idx) in enumerate(zip(curveUv_offsets[:-1], curveUv_offsets[1:])):
                for local_curve_idx, curve_idx in enumerate(range(start_idx, end_idx)):
                    if curve_idx >= len(vertex_counts) or curve_idx >= len(orders):
                        continue
                    
                    vertex_count = vertex_counts[curve_idx]
                    order = orders[curve_idx]
                    if vertex_count == 0 and order == 0:
                        continue

                    expected_knot_count = vertex_count + order
                    curve_knots = list(knots[offset : offset + expected_knot_count] or [])

                    # Handle case where curve_knots might be None or empty
                    if len(curve_knots) != expected_knot_count:
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_395, # Changed from BA_425
                            message=f"Invalid knot count for curveUv #{local_curve_idx} in brep #{brep_id}. "
                                    f"Expected {expected_knot_count}, but got {len(curve_knots)}.",
                            at=brep_array,
                        )

                    # Validate non-decreasing order of the slice
                    if any(y < (x-BrepConstants.NUMERICAL_TOLERANCE) for x, y in zip(curve_knots, curve_knots[1:])): # Allowing for floating point precision issues
                        brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_400,
                            message=f"Invalid knot ordering for curveUv #{local_curve_idx} in brep #{brep_id}. Knots must be non-decreasing.",
                            at=brep_array,
                        )

                    offset += expected_knot_count

            if offset != len(knots):
                brep_idx = max(len(curveUv_offsets) - 2, 0)
                brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_395,
                    message=f"Invalid packed knot count through brep #{brep_id}. "
                            f"Expected {offset}, but got {len(knots)}.",
                    at=brep_array,
                )

        #### Validate brep:curveUv:nurb:weights ####
        weights_val = brep_array.GetAttribute("brep:curveUv:nurb:weights").Get()
        if self._is_unregistered_value(weights_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_416,
                message="brep:curveUv:nurb:weights has an unregistered USD type; expected double[].",
                at=brep_array,
            )
            return
        weights = weights_val if weights_val is not None else []
        weights_len = self._len_or_none(weights)
        if weights_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_405,
                message=f"brep:curveUv:nurb:weights is not a valid sequence; "
                        f"expected size {expected_total_control_vertices}.",
                at=brep_array,
            )
        elif weights_len != expected_total_control_vertices:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_405,
                message=f"Invalid size for brep:curveUv:nurb:weights. "
                        f"Expected size {expected_total_control_vertices}, but got {weights_len}.",
                at=brep_array,
            )

        if weights_len:
            offset = 0
            for brep_idx, (start_idx, end_idx) in enumerate(zip(curveUv_offsets[:-1], curveUv_offsets[1:])):
                for local_curve_idx, curve_idx in enumerate(range(start_idx, end_idx)):
                    if curve_idx >= len(vertex_counts):
                        continue
                    
                    vertex_count = vertex_counts[curve_idx]
                    curve_weights = list(weights[offset : offset + vertex_count] or [])

                    # Validate positive weights
                    for w_idx, weight in enumerate(curve_weights):
                        if weight < BrepConstants.NUMERICAL_TOLERANCE:  # Allowing for floating point precision issues
                            brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_410, # Changed from BA_435
                                message=f"Invalid weight at index #{w_idx} in brep #{brep_id}. Weights must be positive.",
                                at=brep_array,
                            )

                    offset += vertex_count  # Move to the next curve

    ### Valdation of surface geometry ###

    def _validate_surface_orders_vertex_counts(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `surface:nurb:uOrder` and `surface:nurb:vOrder` attributes to ensure they are positive
        and do not exceed the corresponding vertex counts.

        Validates the `brep:surface:nurb:uVertexCount` and `brep:surface:nurb:vVertexCount` attributes
        to ensure of consistent size. Performs per-brep validation with proper error message context.
        """
        # Validate that uVertexCount and vVertexCount are authored and non-empty

        face_surface_type_vals = brep_array.GetAttribute("face:surfaceType").Get() or [] 

        correct_count = sum(1 for face_type in face_surface_type_vals if face_type == "BrepSurfaceNurbAPI")

        # only validate if there are any NURB surfaces expected or if there are any authored
        if correct_count > 0 or len(brep_array.GetAttribute("brep:surface:nurb:uOrder").Get() or []) > 0:
            self._validate_array_sizes_and_authored(
                brep_array=brep_array,
                attributes=["brep:surface:nurb:uVertexCount", "brep:surface:nurb:vVertexCount","brep:surface:nurb:uOrder", "brep:surface:nurb:vOrder"],
                requirement=cap.BrepArrayRequirements.BA_420, # Changed from BA_435
                size=correct_count,
            )

        # Get brep offsets for per-brep validation
        brep_offsets = self._compute_brep_offsets(brep_array)
        surface_nurbs_offsets = brep_offsets.get("surface_nurbs", [])
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        order_u = brep_array.GetAttribute("brep:surface:nurb:uOrder").Get() or []
        order_v = brep_array.GetAttribute("brep:surface:nurb:vOrder").Get() or []
        vertex_count_u = brep_array.GetAttribute("brep:surface:nurb:uVertexCount").Get() or []
        vertex_count_v = brep_array.GetAttribute("brep:surface:nurb:vVertexCount").Get() or []

        for brep_idx, (start_idx, end_idx) in enumerate(zip(surface_nurbs_offsets[:-1], surface_nurbs_offsets[1:])):
            for local_surface_idx, surface_idx in enumerate(range(start_idx, end_idx)):
                if surface_idx >= len(order_u) or surface_idx >= len(order_v) or surface_idx >= len(vertex_count_u) or surface_idx >= len(vertex_count_v):
                    continue
                
                order_u_val = order_u[surface_idx]
                order_v_val = order_v[surface_idx]
                vertex_u = vertex_count_u[surface_idx]
                vertex_v = vertex_count_v[surface_idx]
                
                if order_u_val <= 0 or order_v_val <= 0:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_425, # Changed from BA_440
                        message=f"Invalid order (U: {order_u_val}, V: {order_v_val}) for surface #{local_surface_idx} in brep #{brep_id}. Orders must be positive.",
                        at=brep_array,
                    )
                    continue

                if order_u_val > vertex_u or order_v_val > vertex_v:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_430, # Changed from BA_445
                        message=f"Invalid order (U: {order_u_val}, V: {order_v_val}) for surface #{local_surface_idx} in brep #{brep_id}. Orders must not exceed vertex counts "
                                f"(U: {vertex_u}, V: {vertex_v}).",
                        at=brep_array,
                    )

    def _validate_surface_control_vertices_weights(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `surface:nurb:controlVertices` attribute to ensure it
        matches the expected total count based on uVertexCount * vVertexCount. Validates per brep.
        """
        vertex_count_u_val = brep_array.GetAttribute("brep:surface:nurb:vVertexCount").Get()
        vertex_count_v_val = brep_array.GetAttribute("brep:surface:nurb:uVertexCount").Get()
        weights_val = brep_array.GetAttribute("brep:surface:nurb:weights").Get()
        control_vertices_val = brep_array.GetAttribute("brep:surface:nurb:controlVertices").Get()

        if self._is_unregistered_value(vertex_count_u_val) or self._is_unregistered_value(vertex_count_v_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_471,
                message="brep:surface:nurb:uVertexCount/vVertexCount has an unregistered USD type; expected uint[].",
                at=brep_array,
            )
            return

        if self._is_unregistered_value(weights_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_471,
                message="brep:surface:nurb:weights has an unregistered USD type; expected double[].",
                at=brep_array,
            )
            return

        weights = weights_val if weights_val is not None else []
        control_vertices = control_vertices_val if control_vertices_val is not None else []
        vertex_count_u = vertex_count_u_val if vertex_count_u_val is not None else []
        vertex_count_v = vertex_count_v_val if vertex_count_v_val is not None else []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        # Validate the total size of controlVertices matches uVertexCount * vVertexCount
        try:
            expected_size = sum(u * v for u, v in zip(vertex_count_u, vertex_count_v, strict=True))
        except ValueError:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_420,
                message="brep:surface:nurb:uVertexCount and vVertexCount have mismatched lengths; expected aligned per-surface counts.",
                at=brep_array,
            )
            return
        except (TypeError, ValueError):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_471,
                message=(
                    "brep:surface:nurb:uVertexCount/vVertexCount contain invalid or non-integer data; "
                    "expected uint[] values to compute control vertex count."
                ),
                at=brep_array,
            )
            return
        
        # only validate if there are any NURB surfaces expected or if there are any authored
        if expected_size > 0 or len(weights) > 0:
            # Validate that weights and controlVertices attributes are authored and non-empty
            self._validate_array_sizes_and_authored(
                brep_array=brep_array,
                attributes=["brep:surface:nurb:weights", "brep:surface:nurb:controlVertices"],
                requirement=cap.BrepArrayRequirements.BA_435, # Changed from BA_450
                size=expected_size,
            )

        if self._is_unregistered_value(control_vertices_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_471,
                message="brep:surface:nurb:controlVertices has an unregistered USD type; expected point3d[].",
                at=brep_array,
            )
            return

        weights_len = self._len_or_none(weights)
        if weights_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_471,
                message="brep:surface:nurb:weights is not a valid sequence; expected double[].",
                at=brep_array,
            )
            return

        actual_cv_len = self._len_or_none(control_vertices)
        if actual_cv_len is None:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_435,
                message=f"brep:surface:nurb:controlVertices is not a valid sequence; expected size {expected_size}.",
                at=brep_array,
            )
        elif expected_size and actual_cv_len != expected_size:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_435,
                message=f"Invalid size for brep:surface:nurb:controlVertices. Expected {expected_size}, but got {actual_cv_len}.",
                at=brep_array,
            )

        if not weights or not brep_region_counts:
            return

        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        surface_control_offsets = brep_offsets.get("surface_control_vertices", [])

        # Validate positive weights per brep
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(surface_control_offsets) - 1:
                break
                
            weight_start = surface_control_offsets[brep_idx]
            weight_end = surface_control_offsets[brep_idx + 1]
            
            # Validate weights belonging to this brep
            for weight_idx in range(weight_start, min(weight_end, len(weights))):
                weight = weights[weight_idx]
                if weight < BrepConstants.NUMERICAL_TOLERANCE:  # Allowing for slight floating-point precision issues
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_440, # Changed from BA_450
                        message=f"Invalid weight at index #{weight_idx} in brep #{brep_id}. Weights in brep:surface:nurb:weights must be positive.",
                        at=brep_array,
                    )

    def _validate_surface_knots(self, brep_array: Usd.Prim) -> None:
        """
        Validates the `brep:surface:nurb:uKnots` and `brep:surface:nurb:vKnots` attributes to ensure:
        - Each slice for a surface matches the expected size (uVertexCount + uOrder for uKnots, vVertexCount + vOrder for vKnots).
        - The values in the knots slice are non-decreasing.
        - Per-brep validation with proper error message context.
        """
        uKnots = brep_array.GetAttribute("brep:surface:nurb:uKnots").Get() or []
        vKnots = brep_array.GetAttribute("brep:surface:nurb:vKnots").Get() or []
        uVertexCount = brep_array.GetAttribute("brep:surface:nurb:uVertexCount").Get() or []
        vVertexCount = brep_array.GetAttribute("brep:surface:nurb:vVertexCount").Get() or []
        uOrder = brep_array.GetAttribute("brep:surface:nurb:uOrder").Get() or []
        vOrder = brep_array.GetAttribute("brep:surface:nurb:vOrder").Get() or []

        # Get brep offsets for per-brep validation
        brep_offsets = self._compute_brep_offsets(brep_array)
        surface_nurbs_offsets = brep_offsets.get("surface_nurbs", [])
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        #### Validate Slicing for Each Surface ####
        u_offset = 0
        v_offset = 0
        for brep_idx, (start_idx, end_idx) in enumerate(zip(surface_nurbs_offsets[:-1], surface_nurbs_offsets[1:])):
            for local_surface_idx, surface_idx in enumerate(range(start_idx, end_idx)):
                if surface_idx >= len(uVertexCount) or surface_idx >= len(vVertexCount) or surface_idx >= len(uOrder) or surface_idx >= len(vOrder):
                    continue
                
                uVertex = uVertexCount[surface_idx]
                vVertex = vVertexCount[surface_idx]
                uOrderVal = uOrder[surface_idx]
                vOrderVal = vOrder[surface_idx]
                
                # Compute expected slice sizes
                expected_count_u = uVertex + uOrderVal
                expected_count_v = vVertex + vOrderVal

                # Slice uKnots and vKnots
                uKnots_slice = uKnots[u_offset : u_offset + expected_count_u]
                vKnots_slice = vKnots[v_offset : v_offset + expected_count_v]
                try:
                    uKnots_slice = list(uKnots_slice or [])
                except TypeError:
                    uKnots_slice = []
                try:
                    vKnots_slice = list(vKnots_slice or [])
                except TypeError:
                    vKnots_slice = []

                # Validation: Ensure correct size
                if len(uKnots_slice) != expected_count_u:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_445, # Changed from BA_455
                        message=f"Invalid knot count for surface #{local_surface_idx} in brep #{brep_id} in U direction. "
                                f"Expected {expected_count_u} knots in brep:surface:nurbs:uKnots for this slice, but got {len(uKnots_slice)}.",
                        at=brep_array,
                    )

                if len(vKnots_slice) != expected_count_v:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_450, # Changed from BA_460
                        message=f"Invalid knot count for surface #{local_surface_idx} in brep #{brep_id} in V direction. "
                                f"Expected {expected_count_v} knots in brep:surface:nurbs:vKnots for this slice, but got {len(vKnots_slice)}.",
                        at=brep_array,
                    )

                # Validation: Ensure non-decreasing order for uKnots slice
                if any(y < (x-BrepConstants.NUMERICAL_TOLERANCE) for x, y in zip(uKnots_slice, uKnots_slice[1:])): # Allowing for floating point precision issues
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_455,
                        message=f"Invalid knot ordering detected in U knot vector for surface #{local_surface_idx} in brep #{brep_id}. "
                                "Values must be non-decreasing.",
                        at=brep_array,
                    )

                # Validation: Ensure non-decreasing order for vKnots slice
                if any(y < (x-BrepConstants.NUMERICAL_TOLERANCE) for x, y in zip(vKnots_slice, vKnots_slice[1:])): # Allowing for floating point precision issues
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_460,
                        message=f"Invalid knot ordering detected in V knot vector for surface #{local_surface_idx} in brep #{brep_id}. "
                                "Values must be non-decreasing.",
                        at=brep_array,
                    )

                # Update offsets for the next surface
                u_offset += expected_count_u
                v_offset += expected_count_v

    ### Additional Validation Methods (BA_310, BA_365, BA_465) ###

    def _validate_vertex_position_containment(self, brep_array: Usd.Prim) -> None:
        """
        BA_310: Validate that all vertex positions lie within their corresponding brep extent bounding boxes.
        """
        vertex_positions = brep_array.GetAttribute("brep:vertexPoint:point:position").Get() or []
        brep_extents = brep_array.GetAttribute("brep:extent").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        if not vertex_positions or not brep_extents or not brep_region_counts:
            return
            
        # Compute brep offsets for vertex partitioning
        brep_offsets = self._compute_brep_offsets(brep_array)
        vertex_offsets = brep_offsets.get("vertices", [])
        
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(brep_extents) // 2:
                break
                
            # Get brep extent (min, max) points
            extent_min = brep_extents[brep_idx * 2]
            extent_max = brep_extents[brep_idx * 2 + 1]
            
            # Get vertex range for this brep
            start_vertex = vertex_offsets[brep_idx] if brep_idx < len(vertex_offsets) else 0
            end_vertex = vertex_offsets[brep_idx + 1] if brep_idx + 1 < len(vertex_offsets) else len(vertex_positions)
            
            # Check each vertex position in this brep
            for vertex_idx in range(start_vertex, min(end_vertex, len(vertex_positions))):
                pos = vertex_positions[vertex_idx]
                
                # Check containment with tolerance
                tol = BrepConstants.NUMERICAL_TOLERANCE
                if (pos[0] < extent_min[0] - tol or pos[0] > extent_max[0] + tol or
                    pos[1] < extent_min[1] - tol or pos[1] > extent_max[1] + tol or
                    pos[2] < extent_min[2] - tol or pos[2] > extent_max[2] + tol):
                    brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    # Calculate distance outside bounds for each axis
                    distances = []
                    for i, axis in enumerate(['X', 'Y', 'Z']):
                        if pos[i] < extent_min[i] - tol:
                            distances.append(f"{axis}: {extent_min[i] - pos[i]:.6g} below min")
                        elif pos[i] > extent_max[i] + tol:
                            distances.append(f"{axis}: {pos[i] - extent_max[i]:.6g} above max")
                    distance_info = ", ".join(distances) if distances else "unknown"
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_310,
                        message=f"Vertex position {pos} for vertex #{vertex_idx} in brep #{brep_id_formatted} lies outside brep extent bounds [{extent_min}, {extent_max}]. Distance outside bounds: {distance_info}.",
                        at=brep_array,
                    )

    def _validate_edge3d_nurbs_control_point_containment(self, brep_array: Usd.Prim) -> None:
        """
        BA_084: Validate that edge3d NURBS control points lie within reasonable expansion of brep extents.
        """
        brep_extents = brep_array.GetAttribute("brep:extent").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        if not brep_extents or not brep_region_counts:
            return
        
        # Check edge control points
        edge_control_vertices = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:controlVertices").Get() or []
        if not edge_control_vertices:
            return
            
        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        edge_control_offsets = brep_offsets.get("edge_control_vertices", [])
        
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(brep_extents) // 2:
                break
                
            extent_min = brep_extents[brep_idx * 2]
            extent_max = brep_extents[brep_idx * 2 + 1]
            
            # Get control point range for this brep
            start_cv = edge_control_offsets[brep_idx] if brep_idx < len(edge_control_offsets) else 0
            end_cv = edge_control_offsets[brep_idx + 1] if brep_idx + 1 < len(edge_control_offsets) else len(edge_control_vertices)
            
            # Check control points belonging to this brep
            tol = BrepConstants.NUMERICAL_TOLERANCE
            for cv_idx in range(start_cv, min(end_cv, len(edge_control_vertices))):
                cv = edge_control_vertices[cv_idx]
                if (cv[0] < extent_min[0] - tol or cv[0] > extent_max[0] + tol or
                    cv[1] < extent_min[1] - tol or cv[1] > extent_max[1] + tol or
                    cv[2] < extent_min[2] - tol or cv[2] > extent_max[2] + tol):
                    brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    # Calculate distance outside bounds for each axis
                    distances = []
                    for i, axis in enumerate(['X', 'Y', 'Z']):
                        if cv[i] < extent_min[i] - tol:
                            distances.append(f"{axis}: {extent_min[i] - cv[i]:.6g} below min")
                        elif cv[i] > extent_max[i] + tol:
                            distances.append(f"{axis}: {cv[i] - extent_max[i]:.6g} above max")
                    distance_info = ", ".join(distances) if distances else "unknown"
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_365,
                        message=f"Edge NURBS control vertex {cv} at index {cv_idx} lies outside reasonable bounds for brep #{brep_id_formatted}. Distance outside bounds: {distance_info}.",
                        at=brep_array,
                    )
                    break  # Report only first violation per brep to avoid spam

    def _validate_surface_sphere_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates brep:surface:sphere attributes for BrepSurfaceSphereAPI:
        - BA_480: Array sizes match number of sphere faces
        - BA_481: Radius values are positive
        - BA_482: Axis vectors are unit length
        - BA_483: RefDirection vectors are unit length
        - BA_484: Axis and refDirection are orthogonal
        """
        import math

        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        sphere_face_count = sum(1 for st in face_surface_types if st == "BrepSurfaceSphereAPI")

        if sphere_face_count == 0:
            return

        sphere_centers = brep_array.GetAttribute("brep:surface:sphere:center").Get() or []
        sphere_axes = brep_array.GetAttribute("brep:surface:sphere:axis").Get() or []
        sphere_ref_dirs = brep_array.GetAttribute("brep:surface:sphere:refDirection").Get() or []
        sphere_radii = brep_array.GetAttribute("brep:surface:sphere:radius").Get() or []

        # BA_480: Array sizes must match sphere face count
        sphere_attrs = {
            "brep:surface:sphere:center": sphere_centers,
            "brep:surface:sphere:axis": sphere_axes,
            "brep:surface:sphere:refDirection": sphere_ref_dirs,
            "brep:surface:sphere:radius": sphere_radii,
        }
        for attr_name, attr_vals in sphere_attrs.items():
            if len(attr_vals) != sphere_face_count:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_480,
                    message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepSurfaceSphereAPI faces ({sphere_face_count}).",
                    at=brep_array,
                )

        # BA_481: Radius must be positive
        for i, radius in enumerate(sphere_radii):
            if radius <= 0.0:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_481,
                    message=f"brep:surface:sphere:radius[{i}] = {radius} is not positive.",
                    at=brep_array,
                )
                break

        # BA_482: Axis must be unit length
        for i, axis in enumerate(sphere_axes):
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_482,
                    message=f"brep:surface:sphere:axis[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_483: RefDirection must be unit length
        for i, ref_dir in enumerate(sphere_ref_dirs):
            length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_483,
                    message=f"brep:surface:sphere:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_484: Axis and refDirection must be orthogonal
        count = min(len(sphere_axes), len(sphere_ref_dirs))
        for i in range(count):
            axis = sphere_axes[i]
            ref_dir = sphere_ref_dirs[i]
            dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
            if abs(dot) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_484,
                    message=f"brep:surface:sphere:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                    at=brep_array,
                )
                break

    def _validate_surface_plane_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates brep:surface:plane attributes for BrepSurfacePlaneAPI:
        - BA_490: Array sizes match number of plane faces
        - BA_491: Axis vectors are unit length
        - BA_492: RefDirection vectors are unit length
        - BA_493: Axis and refDirection are orthogonal
        """
        import math

        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        plane_face_count = sum(1 for st in face_surface_types if st == "BrepSurfacePlaneAPI")

        if plane_face_count == 0:
            return

        plane_origins = brep_array.GetAttribute("brep:surface:plane:origin").Get() or []
        plane_axes = brep_array.GetAttribute("brep:surface:plane:axis").Get() or []
        plane_ref_dirs = brep_array.GetAttribute("brep:surface:plane:refDirection").Get() or []

        # BA_490: Array sizes must match plane face count
        plane_attrs = {
            "brep:surface:plane:origin": plane_origins,
            "brep:surface:plane:axis": plane_axes,
            "brep:surface:plane:refDirection": plane_ref_dirs,
        }
        for attr_name, attr_vals in plane_attrs.items():
            if len(attr_vals) != plane_face_count:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_490,
                    message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepSurfacePlaneAPI faces ({plane_face_count}).",
                    at=brep_array,
                )

        # BA_491: Axis must be unit length
        for i, axis in enumerate(plane_axes):
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_491,
                    message=f"brep:surface:plane:axis[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_492: RefDirection must be unit length
        for i, ref_dir in enumerate(plane_ref_dirs):
            length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_492,
                    message=f"brep:surface:plane:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_493: Axis and refDirection must be orthogonal
        count = min(len(plane_axes), len(plane_ref_dirs))
        for i in range(count):
            axis = plane_axes[i]
            ref_dir = plane_ref_dirs[i]
            dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
            if abs(dot) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_493,
                    message=f"brep:surface:plane:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                    at=brep_array,
                )
                break

    def _validate_surface_cylinder_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates brep:surface:cylinder attributes for BrepSurfaceCylinderAPI:
        - BA_500: Array sizes match number of cylinder faces
        - BA_501: Radius values are positive
        - BA_502: Axis vectors are unit length
        - BA_503: RefDirection vectors are unit length
        - BA_504: Axis and refDirection are orthogonal
        """
        import math

        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        cyl_face_count = sum(1 for st in face_surface_types if st == "BrepSurfaceCylinderAPI")

        if cyl_face_count == 0:
            return

        cyl_origins = brep_array.GetAttribute("brep:surface:cylinder:origin").Get() or []
        cyl_axes = brep_array.GetAttribute("brep:surface:cylinder:axis").Get() or []
        cyl_ref_dirs = brep_array.GetAttribute("brep:surface:cylinder:refDirection").Get() or []
        cyl_radii = brep_array.GetAttribute("brep:surface:cylinder:radius").Get() or []

        # BA_500: Array sizes must match cylinder face count
        cyl_attrs = {
            "brep:surface:cylinder:origin": cyl_origins,
            "brep:surface:cylinder:axis": cyl_axes,
            "brep:surface:cylinder:refDirection": cyl_ref_dirs,
            "brep:surface:cylinder:radius": cyl_radii,
        }
        for attr_name, attr_vals in cyl_attrs.items():
            if len(attr_vals) != cyl_face_count:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_500,
                    message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepSurfaceCylinderAPI faces ({cyl_face_count}).",
                    at=brep_array,
                )

        # BA_501: Radius must be positive
        for i, radius in enumerate(cyl_radii):
            if radius <= 0.0:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_501,
                    message=f"brep:surface:cylinder:radius[{i}] = {radius} is not positive.",
                    at=brep_array,
                )
                break

        # BA_502: Axis must be unit length
        for i, axis in enumerate(cyl_axes):
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_502,
                    message=f"brep:surface:cylinder:axis[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_503: RefDirection must be unit length
        for i, ref_dir in enumerate(cyl_ref_dirs):
            length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_503,
                    message=f"brep:surface:cylinder:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_504: Axis and refDirection must be orthogonal
        count = min(len(cyl_axes), len(cyl_ref_dirs))
        for i in range(count):
            axis = cyl_axes[i]
            ref_dir = cyl_ref_dirs[i]
            dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
            if abs(dot) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_504,
                    message=f"brep:surface:cylinder:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                    at=brep_array,
                )
                break

    def _validate_surface_cone_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates brep:surface:cone attributes for BrepSurfaceConeAPI:
        - BA_510: Array sizes match number of cone faces
        - BA_511: Radius values are non-negative
        - BA_512: Axis vectors are unit length
        - BA_513: RefDirection vectors are unit length
        - BA_514: Axis and refDirection are orthogonal
        - BA_515: SemiAngle in valid range (0, pi/2)
        """
        import math

        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        cone_face_count = sum(1 for st in face_surface_types if st == "BrepSurfaceConeAPI")

        if cone_face_count == 0:
            return

        cone_origins = brep_array.GetAttribute("brep:surface:cone:origin").Get() or []
        cone_axes = brep_array.GetAttribute("brep:surface:cone:axis").Get() or []
        cone_ref_dirs = brep_array.GetAttribute("brep:surface:cone:refDirection").Get() or []
        cone_radii = brep_array.GetAttribute("brep:surface:cone:radius").Get() or []
        cone_semi_angles = brep_array.GetAttribute("brep:surface:cone:semiAngle").Get() or []

        # BA_510: Array sizes must match cone face count
        cone_attrs = {
            "brep:surface:cone:origin": cone_origins,
            "brep:surface:cone:axis": cone_axes,
            "brep:surface:cone:refDirection": cone_ref_dirs,
            "brep:surface:cone:radius": cone_radii,
            "brep:surface:cone:semiAngle": cone_semi_angles,
        }
        for attr_name, attr_vals in cone_attrs.items():
            if len(attr_vals) != cone_face_count:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_510,
                    message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepSurfaceConeAPI faces ({cone_face_count}).",
                    at=brep_array,
                )

        # BA_511: Radius must be non-negative
        for i, radius in enumerate(cone_radii):
            if radius < 0.0:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_511,
                    message=f"brep:surface:cone:radius[{i}] = {radius} is negative.",
                    at=brep_array,
                )
                break

        # BA_512: Axis must be unit length
        for i, axis in enumerate(cone_axes):
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_512,
                    message=f"brep:surface:cone:axis[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_513: RefDirection must be unit length
        for i, ref_dir in enumerate(cone_ref_dirs):
            length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_513,
                    message=f"brep:surface:cone:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_514: Axis and refDirection must be orthogonal
        count = min(len(cone_axes), len(cone_ref_dirs))
        for i in range(count):
            axis = cone_axes[i]
            ref_dir = cone_ref_dirs[i]
            dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
            if abs(dot) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_514,
                    message=f"brep:surface:cone:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                    at=brep_array,
                )
                break

        # BA_515: SemiAngle must be in (0, pi/2) exclusive
        half_pi = math.pi / 2.0
        for i, semi_angle in enumerate(cone_semi_angles):
            if semi_angle <= 0.0 or semi_angle >= half_pi:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_515,
                    message=f"brep:surface:cone:semiAngle[{i}] = {semi_angle} is not in valid range (0, pi/2).",
                    at=brep_array,
                )
                break

    def _validate_surface_torus_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates brep:surface:torus attributes for BrepSurfaceTorusAPI:
        - BA_520: Array sizes match number of torus faces
        - BA_521: MajorRadius values are positive
        - BA_522: MinorRadius values are positive
        - BA_523: Axis vectors are unit length
        - BA_524: RefDirection vectors are unit length
        - BA_525: Axis and refDirection are orthogonal
        """
        import math

        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        torus_face_count = sum(1 for st in face_surface_types if st == "BrepSurfaceTorusAPI")

        if torus_face_count == 0:
            return

        torus_origins = brep_array.GetAttribute("brep:surface:torus:origin").Get() or []
        torus_axes = brep_array.GetAttribute("brep:surface:torus:axis").Get() or []
        torus_ref_dirs = brep_array.GetAttribute("brep:surface:torus:refDirection").Get() or []
        torus_major_radii = brep_array.GetAttribute("brep:surface:torus:majorRadius").Get() or []
        torus_minor_radii = brep_array.GetAttribute("brep:surface:torus:minorRadius").Get() or []

        # BA_520: Array sizes must match torus face count
        torus_attrs = {
            "brep:surface:torus:origin": torus_origins,
            "brep:surface:torus:axis": torus_axes,
            "brep:surface:torus:refDirection": torus_ref_dirs,
            "brep:surface:torus:majorRadius": torus_major_radii,
            "brep:surface:torus:minorRadius": torus_minor_radii,
        }
        for attr_name, attr_vals in torus_attrs.items():
            if len(attr_vals) != torus_face_count:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_520,
                    message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepSurfaceTorusAPI faces ({torus_face_count}).",
                    at=brep_array,
                )

        # BA_521: MajorRadius must be positive
        for i, radius in enumerate(torus_major_radii):
            if radius <= 0.0:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_521,
                    message=f"brep:surface:torus:majorRadius[{i}] = {radius} is not positive.",
                    at=brep_array,
                )
                break

        # BA_522: MinorRadius must be positive
        for i, radius in enumerate(torus_minor_radii):
            if radius <= 0.0:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_522,
                    message=f"brep:surface:torus:minorRadius[{i}] = {radius} is not positive.",
                    at=brep_array,
                )
                break

        # BA_523: Axis must be unit length
        for i, axis in enumerate(torus_axes):
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_523,
                    message=f"brep:surface:torus:axis[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_524: RefDirection must be unit length
        for i, ref_dir in enumerate(torus_ref_dirs):
            length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
            if abs(length - 1.0) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_524,
                    message=f"brep:surface:torus:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                    at=brep_array,
                )
                break

        # BA_525: Axis and refDirection must be orthogonal
        count = min(len(torus_axes), len(torus_ref_dirs))
        for i in range(count):
            axis = torus_axes[i]
            ref_dir = torus_ref_dirs[i]
            dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
            if abs(dot) > 1e-4:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_525,
                    message=f"brep:surface:torus:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                    at=brep_array,
                )
                break

    def _validate_curve3d_circle_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates circle curve attributes for both edge and wireEdge instances.
        - BA_530: Array sizes match count of circle curves
        - BA_531: Radius values are positive
        - BA_532: Axis vectors are unit length
        - BA_533: RefDirection vectors are unit length
        - BA_534: Axis and refDirection are orthogonal
        """
        import math

        for instance_name, curve_type_attr in [("edge3dCircle", "edge:curveType"), ("wireEdge3dCircle", "wireEdge:curveType")]:
            curve_types = brep_array.GetAttribute(curve_type_attr).Get() or []
            circle_count = sum(1 for ct in curve_types if ct == "BrepCurve3dCircleAPI")

            if circle_count == 0:
                continue

            prefix = f"brep:{instance_name}:curve3d:circle"
            centers = brep_array.GetAttribute(f"{prefix}:center").Get() or []
            axes = brep_array.GetAttribute(f"{prefix}:axis").Get() or []
            ref_dirs = brep_array.GetAttribute(f"{prefix}:refDirection").Get() or []
            radii = brep_array.GetAttribute(f"{prefix}:radius").Get() or []

            # BA_530: Array sizes
            circle_attrs = {
                f"{prefix}:center": centers,
                f"{prefix}:axis": axes,
                f"{prefix}:refDirection": ref_dirs,
                f"{prefix}:radius": radii,
            }
            for attr_name, attr_vals in circle_attrs.items():
                if len(attr_vals) != circle_count:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_530,
                        message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepCurve3dCircleAPI entries ({circle_count}) in {curve_type_attr}.",
                        at=brep_array,
                    )

            # BA_531: Radius must be positive
            for i, radius in enumerate(radii):
                if radius <= 0.0:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_531,
                        message=f"{prefix}:radius[{i}] = {radius} is not positive.",
                        at=brep_array,
                    )
                    break

            # BA_532: Axis must be unit length
            for i, axis in enumerate(axes):
                length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                if abs(length - 1.0) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_532,
                        message=f"{prefix}:axis[{i}] has length {length:.6f}, expected 1.0.",
                        at=brep_array,
                    )
                    break

            # BA_533: RefDirection must be unit length
            for i, ref_dir in enumerate(ref_dirs):
                length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
                if abs(length - 1.0) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_533,
                        message=f"{prefix}:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                        at=brep_array,
                    )
                    break

            # BA_534: Axis and refDirection must be orthogonal
            count = min(len(axes), len(ref_dirs))
            for i in range(count):
                axis = axes[i]
                ref_dir = ref_dirs[i]
                dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
                if abs(dot) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_534,
                        message=f"{prefix}:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                        at=brep_array,
                    )
                    break

    def _validate_curve3d_line_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates line curve attributes for both edge and wireEdge instances.
        - BA_540: Array sizes match count of line curves
        - BA_541: Direction vectors are unit length
        """
        import math

        for instance_name, curve_type_attr in [("edge3dLine", "edge:curveType"), ("wireEdge3dLine", "wireEdge:curveType")]:
            curve_types = brep_array.GetAttribute(curve_type_attr).Get() or []
            line_count = sum(1 for ct in curve_types if ct == "BrepCurve3dLineAPI")

            if line_count == 0:
                continue

            prefix = f"brep:{instance_name}:curve3d:line"
            origins = brep_array.GetAttribute(f"{prefix}:origin").Get() or []
            directions = brep_array.GetAttribute(f"{prefix}:direction").Get() or []

            # BA_540: Array sizes
            line_attrs = {
                f"{prefix}:origin": origins,
                f"{prefix}:direction": directions,
            }
            for attr_name, attr_vals in line_attrs.items():
                if len(attr_vals) != line_count:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_540,
                        message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepCurve3dLineAPI entries ({line_count}) in {curve_type_attr}.",
                        at=brep_array,
                    )

            # BA_541: Direction must be unit length
            for i, direction in enumerate(directions):
                length = math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
                if abs(length - 1.0) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_541,
                        message=f"{prefix}:direction[{i}] has length {length:.6f}, expected 1.0.",
                        at=brep_array,
                    )
                    break

    def _validate_curve3d_ellipse_data(self, brep_array: Usd.Prim) -> None:
        """
        Validates ellipse curve attributes for both edge and wireEdge instances.
        - BA_550: Array sizes match count of ellipse curves
        - BA_551: xRadius values are positive
        - BA_552: yRadius values are positive
        - BA_553: Axis vectors are unit length
        - BA_554: RefDirection vectors are unit length
        - BA_555: Axis and refDirection are orthogonal
        """
        import math

        for instance_name, curve_type_attr in [("edge3dEllipse", "edge:curveType"), ("wireEdge3dEllipse", "wireEdge:curveType")]:
            curve_types = brep_array.GetAttribute(curve_type_attr).Get() or []
            ellipse_count = sum(1 for ct in curve_types if ct == "BrepCurve3dEllipseAPI")

            if ellipse_count == 0:
                continue

            prefix = f"brep:{instance_name}:curve3d:ellipse"
            centers = brep_array.GetAttribute(f"{prefix}:center").Get() or []
            axes = brep_array.GetAttribute(f"{prefix}:axis").Get() or []
            ref_dirs = brep_array.GetAttribute(f"{prefix}:refDirection").Get() or []
            x_radius = brep_array.GetAttribute(f"{prefix}:xRadius").Get() or []
            y_radius = brep_array.GetAttribute(f"{prefix}:yRadius").Get() or []

            # BA_550: Array sizes
            ellipse_attrs = {
                f"{prefix}:center": centers,
                f"{prefix}:axis": axes,
                f"{prefix}:refDirection": ref_dirs,
                f"{prefix}:xRadius": x_radius,
                f"{prefix}:yRadius": y_radius,
            }
            for attr_name, attr_vals in ellipse_attrs.items():
                if len(attr_vals) != ellipse_count:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_550,
                        message=f"{attr_name} size ({len(attr_vals)}) does not match number of BrepCurve3dEllipseAPI entries ({ellipse_count}) in {curve_type_attr}.",
                        at=brep_array,
                    )

            # BA_551: xRadius must be positive
            for i, r in enumerate(x_radius):
                if r <= 0.0:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_551,
                        message=f"{prefix}:xRadius[{i}] = {r} is not positive.",
                        at=brep_array,
                    )
                    break

            # BA_552: yRadius must be positive
            for i, r in enumerate(y_radius):
                if r <= 0.0:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_552,
                        message=f"{prefix}:yRadius[{i}] = {r} is not positive.",
                        at=brep_array,
                    )
                    break

            # BA_553: Axis must be unit length
            for i, axis in enumerate(axes):
                length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                if abs(length - 1.0) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_553,
                        message=f"{prefix}:axis[{i}] has length {length:.6f}, expected 1.0.",
                        at=brep_array,
                    )
                    break

            # BA_554: RefDirection must be unit length
            for i, ref_dir in enumerate(ref_dirs):
                length = math.sqrt(ref_dir[0] ** 2 + ref_dir[1] ** 2 + ref_dir[2] ** 2)
                if abs(length - 1.0) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_554,
                        message=f"{prefix}:refDirection[{i}] has length {length:.6f}, expected 1.0.",
                        at=brep_array,
                    )
                    break

            # BA_555: Axis and refDirection must be orthogonal
            count = min(len(axes), len(ref_dirs))
            for i in range(count):
                axis = axes[i]
                ref_dir = ref_dirs[i]
                dot = axis[0] * ref_dir[0] + axis[1] * ref_dir[1] + axis[2] * ref_dir[2]
                if abs(dot) > 1e-4:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_555,
                        message=f"{prefix}:axis[{i}] and refDirection[{i}] are not orthogonal (dot product = {dot:.6f}).",
                        at=brep_array,
                    )
                    break

    def _validate_face_range_domain_limits(self, brep_array: Usd.Prim) -> None:
        """
        Validate authored face:range spans against the UsdSolid proposal's
        Range rule (length <= period) and the schema's analytic parameterization.

        Angular parameters are periodic with period 2*pi, so the span
        (max - min) must not exceed 2*pi; the interval need not start at zero.
        This checks numeric ranges, not geometric seam or trim agreement.
        Sphere latitude is additionally bounded to [-pi/2, pi/2].
        Linear parameters (plane U/V, cylinder V,
        cone V) are unbounded and need no extra check beyond non-degeneracy
        (already covered by BA_155 / BA_160).
        """
        TWO_PI = 2.0 * math.pi
        HALF_PI = math.pi / 2.0
        DOMAIN_TOL = 1e-6

        face_surface_types = BrepConstants.safe_get_attribute(brep_array, "face:surfaceType")
        face_ranges_val = brep_array.GetAttribute("face:range").Get()
        if self._is_unregistered_value(face_ranges_val):
            return
        face_ranges = face_ranges_val or []
        if not face_surface_types or not face_ranges:
            return

        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        brep_offsets = self._compute_brep_offsets(brep_array)
        face_offsets = brep_offsets.get("faces", [])

        num_faces = len(face_surface_types)
        if len(face_ranges) < num_faces * 2:
            return

        for face_idx in range(num_faces):
            uv_min = face_ranges[2 * face_idx]
            uv_max = face_ranges[2 * face_idx + 1]
            u_min, v_min = float(uv_min[0]), float(uv_min[1])
            u_max, v_max = float(uv_max[0]), float(uv_max[1])
            u_span = u_max - u_min
            v_span = v_max - v_min
            stype = str(face_surface_types[face_idx])

            brep_idx = 0
            local_face_idx = face_idx
            for bi, (start, end) in enumerate(zip(face_offsets[:-1], face_offsets[1:])):
                if start <= face_idx < end:
                    brep_idx = bi
                    local_face_idx = face_idx - start
                    break
            brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

            if stype == "BrepSurfaceSphereAPI":
                if u_span > TWO_PI + DOMAIN_TOL:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_560,
                        message=(
                            f"Sphere face #{local_face_idx} in brep #{brep_id} has U span "
                            f"{u_span:.6f} rad which exceeds 2*pi ({TWO_PI:.6f}). "
                            f"U range = [{u_min:.6f}, {u_max:.6f}]."
                        ),
                        at=brep_array,
                    )
                if v_min < -HALF_PI - DOMAIN_TOL or v_max > HALF_PI + DOMAIN_TOL:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_561,
                        message=(
                            f"Sphere face #{local_face_idx} in brep #{brep_id} has V range "
                            f"[{v_min:.6f}, {v_max:.6f}] rad outside the latitude bounds "
                            f"[-pi/2, pi/2] = [{-HALF_PI:.6f}, {HALF_PI:.6f}]."
                        ),
                        at=brep_array,
                    )

            elif stype == "BrepSurfaceCylinderAPI":
                if u_span > TWO_PI + DOMAIN_TOL:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_562,
                        message=(
                            f"Cylinder face #{local_face_idx} in brep #{brep_id} has U span "
                            f"{u_span:.6f} rad which exceeds 2*pi ({TWO_PI:.6f}). "
                            f"U range = [{u_min:.6f}, {u_max:.6f}]."
                        ),
                        at=brep_array,
                    )

            elif stype == "BrepSurfaceConeAPI":
                if u_span > TWO_PI + DOMAIN_TOL:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_563,
                        message=(
                            f"Cone face #{local_face_idx} in brep #{brep_id} has U span "
                            f"{u_span:.6f} rad which exceeds 2*pi ({TWO_PI:.6f}). "
                            f"U range = [{u_min:.6f}, {u_max:.6f}]."
                        ),
                        at=brep_array,
                    )

            elif stype == "BrepSurfaceTorusAPI":
                if u_span > TWO_PI + DOMAIN_TOL:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_564,
                        message=(
                            f"Torus face #{local_face_idx} in brep #{brep_id} has U span "
                            f"{u_span:.6f} rad which exceeds 2*pi ({TWO_PI:.6f}). "
                            f"U range = [{u_min:.6f}, {u_max:.6f}]."
                        ),
                        at=brep_array,
                    )
                if v_span > TWO_PI + DOMAIN_TOL:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_565,
                        message=(
                            f"Torus face #{local_face_idx} in brep #{brep_id} has V span "
                            f"{v_span:.6f} rad which exceeds 2*pi ({TWO_PI:.6f}). "
                            f"V range = [{v_min:.6f}, {v_max:.6f}]."
                        ),
                        at=brep_array,
                    )

    def _validate_edge_range_domain_limits(self, brep_array: Usd.Prim) -> None:
        """
        Validate that edge:range and wireEdge:range values respect the natural
        parameterization limits of analytic curves, per STEP (ISO 10303-42),
        IGES, and PRC (ISO 14739-1).

        Circle and ellipse parameters are angular with period 2*pi, so
        the span (max - min) must not exceed 2*pi.  Line parameters are
        unbounded and need no extra check beyond ordering (BA_235 / BA_275).
        """
        TWO_PI = 2.0 * math.pi
        DOMAIN_TOL = 1e-6

        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        brep_offsets = self._compute_brep_offsets(brep_array)

        for kind, type_attr, range_attr, edge_offsets_key in [
            ("edge", "edge:curveType", "edge:range", "edges"),
            ("wireEdge", "wireEdge:curveType", "wireEdge:range", "wireedges"),
        ]:
            curve_types = BrepConstants.safe_get_attribute(brep_array, type_attr)
            range_vals = brep_array.GetAttribute(range_attr).Get()
            if self._is_unregistered_value(range_vals):
                continue
            ranges = range_vals or []
            if not curve_types or not ranges:
                continue

            edge_offsets = brep_offsets.get(edge_offsets_key, [])
            num_edges = len(curve_types)
            if len(ranges) < num_edges * 2:
                continue

            for edge_idx in range(num_edges):
                try:
                    param_min = float(ranges[2 * edge_idx])
                    param_max = float(ranges[2 * edge_idx + 1])
                except (TypeError, ValueError, IndexError):
                    continue
                span = param_max - param_min
                ctype = str(curve_types[edge_idx])

                brep_idx = 0
                local_edge_idx = edge_idx
                for bi, (start, end) in enumerate(zip(edge_offsets[:-1], edge_offsets[1:])):
                    if start <= edge_idx < end:
                        brep_idx = bi
                        local_edge_idx = edge_idx - start
                        break
                brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

                if ctype == "BrepCurve3dCircleAPI":
                    if span > TWO_PI + DOMAIN_TOL:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_570,
                            message=(
                                f"Circle {kind} #{local_edge_idx} in brep #{brep_id} has "
                                f"parameter span {span:.6f} rad which exceeds 2*pi "
                                f"({TWO_PI:.6f}). Range = [{param_min:.6f}, {param_max:.6f}]."
                            ),
                            at=brep_array,
                        )

                elif ctype == "BrepCurve3dEllipseAPI":
                    if span > TWO_PI + DOMAIN_TOL:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_571,
                            message=(
                                f"Ellipse {kind} #{local_edge_idx} in brep #{brep_id} has "
                                f"parameter span {span:.6f} rad which exceeds 2*pi "
                                f"({TWO_PI:.6f}). Range = [{param_min:.6f}, {param_max:.6f}]."
                            ),
                            at=brep_array,
                        )

    def _validate_surface_nurbs_control_point_containment(self, brep_array: Usd.Prim) -> None:
        """
        BA_085: Validate that surface NURBS control points lie within reasonable expansion of brep extents.
        """
        brep_extents = brep_array.GetAttribute("brep:extent").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        if not brep_extents or not brep_region_counts:
            return
        
        # Check surface control points
        surface_control_vertices = brep_array.GetAttribute("brep:surface:nurb:controlVertices").Get() or []
        if not surface_control_vertices:
            return
            
        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        surface_control_offsets = brep_offsets.get("surface_control_vertices", [])
        
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(brep_extents) // 2:
                break
                
            extent_min = brep_extents[brep_idx * 2]
            extent_max = brep_extents[brep_idx * 2 + 1]
            
            # Get control point range for this brep
            start_cv = surface_control_offsets[brep_idx] if brep_idx < len(surface_control_offsets) else 0
            end_cv = surface_control_offsets[brep_idx + 1] if brep_idx + 1 < len(surface_control_offsets) else len(surface_control_vertices)
            
            # Check control points belonging to this brep
            tol = BrepConstants.NUMERICAL_TOLERANCE
            for cv_idx in range(start_cv, min(end_cv, len(surface_control_vertices))):
                cv = surface_control_vertices[cv_idx]
                if (cv[0] < extent_min[0] - tol or cv[0] > extent_max[0] + tol or
                    cv[1] < extent_min[1] - tol or cv[1] > extent_max[1] + tol or
                    cv[2] < extent_min[2] - tol or cv[2] > extent_max[2] + tol):
                    brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    # Calculate distance outside bounds for each axis
                    distances = []
                    for i, axis in enumerate(['X', 'Y', 'Z']):
                        if cv[i] < extent_min[i] - tol:
                            distances.append(f"{axis}: {extent_min[i] - cv[i]:.6g} below min")
                        elif cv[i] > extent_max[i] + tol:
                            distances.append(f"{axis}: {cv[i] - extent_max[i]:.6g} above max")
                    distance_info = ", ".join(distances) if distances else "unknown"
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_465,
                        message=f"Surface NURBS control vertex {cv} at index {cv_idx} lies outside reasonable bounds for brep #{brep_id_formatted}. Distance outside bounds: {distance_info}.",
                        at=brep_array,
                    )
                    break  # Report only first violation per brep to avoid spam

    def _validate_required_geometry_apis(self, brep_array: Usd.Prim) -> None:
        """Validate the applied APIs required to interpret authored geometry."""
        applied_schemas = set(brep_array.GetAppliedSchemas())

        required_api_uses = []

        def add_token_requirement(attribute_name: str, token: str, api_schema: str) -> None:
            values = BrepConstants.safe_get_attribute(brep_array, attribute_name)
            occurrence_count = sum(1 for value in values if value == token)
            if occurrence_count:
                required_api_uses.append((api_schema, attribute_name, token, occurrence_count))

        token_api_requirements = (
            ("vertex:pointType", "BrepPointAPI", "BrepPointAPI:vertexPoint"),
            ("edge:curveType", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI:edge3dNurb"),
            ("edge:curveType", "BrepCurve3dLineAPI", "BrepCurve3dLineAPI:edge3dLine"),
            ("edge:curveType", "BrepCurve3dCircleAPI", "BrepCurve3dCircleAPI:edge3dCircle"),
            ("edge:curveType", "BrepCurve3dEllipseAPI", "BrepCurve3dEllipseAPI:edge3dEllipse"),
            (
                "wireEdge:curveType",
                "BrepCurve3dNurbAPI",
                "BrepCurve3dNurbAPI:wireEdge3dNurb",
            ),
            (
                "wireEdge:curveType",
                "BrepCurve3dLineAPI",
                "BrepCurve3dLineAPI:wireEdge3dLine",
            ),
            (
                "wireEdge:curveType",
                "BrepCurve3dCircleAPI",
                "BrepCurve3dCircleAPI:wireEdge3dCircle",
            ),
            (
                "wireEdge:curveType",
                "BrepCurve3dEllipseAPI",
                "BrepCurve3dEllipseAPI:wireEdge3dEllipse",
            ),
            ("face:surfaceType", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"),
            ("face:surfaceType", "BrepSurfacePlaneAPI", "BrepSurfacePlaneAPI"),
            ("face:surfaceType", "BrepSurfaceCylinderAPI", "BrepSurfaceCylinderAPI"),
            ("face:surfaceType", "BrepSurfaceConeAPI", "BrepSurfaceConeAPI"),
            ("face:surfaceType", "BrepSurfaceSphereAPI", "BrepSurfaceSphereAPI"),
            ("face:surfaceType", "BrepSurfaceTorusAPI", "BrepSurfaceTorusAPI"),
        )
        for attribute_name, token, api_schema in token_api_requirements:
            add_token_requirement(attribute_name, token, api_schema)

        # shell:pointType is meaningful only for point shells. For face or wire
        # shells the schema explicitly says that the token is ignored.
        shell_point_types = BrepConstants.safe_get_attribute(brep_array, "shell:pointType")
        shell_faceuse_counts = BrepConstants.safe_get_attribute(brep_array, "shell:faceuseCount")
        shell_wireedge_counts = BrepConstants.safe_get_attribute(brep_array, "shell:wireEdgeCount")
        shell_point_count = sum(
            1
            for shell_index in range(len(shell_point_types))
            if BrepConstants.is_brep_point_shell(
                shell_index,
                shell_point_types,
                shell_faceuse_counts,
                shell_wireedge_counts,
            )
        )
        if shell_point_count:
            required_api_uses.append(
                ("BrepPointAPI:shellPoint", "shell:pointType", "BrepPointAPI", shell_point_count)
            )

        for api_schema, attribute_name, token, occurrence_count in required_api_uses:
            if api_schema not in applied_schemas:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_583,
                    message=(
                        f"{attribute_name} contains {occurrence_count} '{token}' occurrence(s), but required "
                        f"applied geometry API '{api_schema}' is absent from apiSchemas."
                    ),
                    at=brep_array,
                )

        # UV pcurves have no topology type-token array. Their data is optional,
        # so edgeuse existence alone must not require BrepCurveUvNurbAPI. A
        # non-zero UV curve record or packed UV geometry does require the API.
        curve_uv_orders = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:order")
        curve_uv_vertex_counts = BrepConstants.safe_get_attribute(
            brep_array,
            "brep:curveUv:nurb:vertexCount",
        )
        has_uv_curve_record = any(value != 0 for value in curve_uv_orders) or any(
            value != 0 for value in curve_uv_vertex_counts
        )
        has_uv_packed_data = any(
            BrepConstants.safe_get_attribute(brep_array, attribute_name)
            for attribute_name in (
                "brep:curveUv:nurb:controlVertices",
                "brep:curveUv:nurb:knots",
                "brep:curveUv:nurb:weights",
            )
        )
        if (
            has_uv_curve_record or has_uv_packed_data
        ) and "BrepCurveUvNurbAPI" not in applied_schemas:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_583,
                message=(
                    "Authored UV NURBS pcurve data requires applied geometry API "
                    "'BrepCurveUvNurbAPI', but it is absent from apiSchemas."
                ),
                at=brep_array,
            )

    def _validate_schema_consistency(self, brep_array: Usd.Prim) -> None:
        """
        Validate that applied API schemas match actual data usage.
        BA_370: Edge 3D NURBS Schema and Data Consistency
        BA_290: WireEdge 3D NURBS Schema Consistency  
        BA_415: CurveUv NURBS Schema Consistency
        BA_470: Surface NURBS Schema Consistency
        BA_305: Vertex Point Schema Consistency
        BA_583: Required Applied Geometry API Consistency
        """
        self._validate_required_geometry_apis(brep_array)

        prim_api_schemas = brep_array.GetAppliedSchemas()
        
        # BA_370: Check BrepCurve3dNurbAPI:edge3dNurb consistency  
        has_edge_curve_api = "BrepCurve3dNurbAPI:edge3dNurb" in prim_api_schemas
        edge_curve_types = brep_array.GetAttribute("edge:curveType").Get() or []
        nurbs_curve_edges = [i for i, ct in enumerate(edge_curve_types) if ct == "BrepCurve3dNurbAPI"]
        
        if nurbs_curve_edges:
            # Check if NURBS data is authored when edges use NURBS
            nurbs_orders = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get() or []
            if not nurbs_orders:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_370,
                    message=f"Found {len(nurbs_curve_edges)} edges with edge:curveType='BrepCurve3dNurbAPI' but no brep:edge3dNurb:curve3d:nurb NURBS data is authored.",
                    at=brep_array,
                )
        
        if has_edge_curve_api and not nurbs_curve_edges:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_370,
                message="BrepCurve3dNurbAPI:edge3dNurb appears in apiSchemas but no edges use edge:curveType='BrepCurve3dNurbAPI'.",
                at=brep_array,
            )
        
        # BA_290: Check BrepCurve3dNurbAPI:wireEdge3dNurb consistency
        has_wireedge_curve_api = "BrepCurve3dNurbAPI:wireEdge3dNurb" in prim_api_schemas
        wireedge_curve_types = brep_array.GetAttribute("wireEdge:curveType").Get() or []
        nurbs_curve_wireedges = [i for i, ct in enumerate(wireedge_curve_types) if ct == "BrepCurve3dNurbAPI"]
        
        if nurbs_curve_wireedges:
            # Check if NURBS data is authored when wireEdges use NURBS
            nurbs_orders = brep_array.GetAttribute("brep:wireEdge3dNurb:curve3d:nurb:order").Get() or []
            if not nurbs_orders:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_290,
                    message=f"Found {len(nurbs_curve_wireedges)} wireEdges with wireEdge:curveType='BrepCurve3dNurbAPI' but no brep:wireEdge3dNurb:curve3d:nurb NURBS data is authored.",
                    at=brep_array,
                )
        
        if has_wireedge_curve_api and not nurbs_curve_wireedges:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_290,
                message="BrepCurve3dNurbAPI:wireEdge3dNurb appears in apiSchemas but no wireEdges use wireEdge:curveType='BrepCurve3dNurbAPI'.",
                at=brep_array,
            )
            
        # BA_415: Check BrepCurveUvNurbAPI consistency
        has_curveUv_api = "BrepCurveUvNurbAPI" in prim_api_schemas
        edgeuse_count = len(brep_array.GetAttribute("edgeuse:edgeIndex").Get() or [])
        
        if edgeuse_count > 0:
            # Check if curveUv NURBS data is authored when edgeuses exist
            curveUv_orders = brep_array.GetAttribute("brep:curveUv:nurb:order").Get() or []
            if has_curveUv_api and not curveUv_orders:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_415,
                    message=f"Found {edgeuse_count} edgeuses but BrepCurveUvNurbAPI is in apiSchemas and no brep:curveUv:nurb NURBS data is authored.",
                    at=brep_array,
                )
        
        if has_curveUv_api and edgeuse_count == 0:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_415,
                message="BrepCurveUvNurbAPI appears in apiSchemas but no edgeuses exist.",
                at=brep_array,
            )
        
        # BA_470: Check BrepSurfaceNurbAPI consistency
        has_surface_api = "BrepSurfaceNurbAPI" in prim_api_schemas
        face_surface_types = brep_array.GetAttribute("face:surfaceType").Get() or []
        nurbs_surface_faces = [i for i, st in enumerate(face_surface_types) if st == "BrepSurfaceNurbAPI"]
        
        if nurbs_surface_faces:
            # Check if NURBS surface data is authored
            nurbs_u_orders = brep_array.GetAttribute("brep:surface:nurb:uOrder").Get() or []
            if not nurbs_u_orders:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_470,
                    message=f"Found {len(nurbs_surface_faces)} faces with face:surfaceType='BrepSurfaceNurbAPI' but no brep:surface:nurb NURBS data is authored.",
                    at=brep_array,
                )
        
        if has_surface_api and not nurbs_surface_faces:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_470,
                message="BrepSurfaceNurbAPI appears in apiSchemas but no faces use face:surfaceType='BrepSurfaceNurbAPI'.",
                at=brep_array,
            )
        
        # BA_485: Check BrepSurfaceSphereAPI consistency
        has_sphere_api = "BrepSurfaceSphereAPI" in prim_api_schemas
        sphere_surface_faces = [i for i, st in enumerate(face_surface_types) if st == "BrepSurfaceSphereAPI"]
        
        if sphere_surface_faces:
            sphere_centers = brep_array.GetAttribute("brep:surface:sphere:center").Get() or []
            if not sphere_centers:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_485,
                    message=f"Found {len(sphere_surface_faces)} faces with face:surfaceType='BrepSurfaceSphereAPI' but no brep:surface:sphere data is authored.",
                    at=brep_array,
                )
        
        if has_sphere_api and not sphere_surface_faces:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_485,
                message="BrepSurfaceSphereAPI appears in apiSchemas but no faces use face:surfaceType='BrepSurfaceSphereAPI'.",
                at=brep_array,
            )

        # BA_495: Check BrepSurfacePlaneAPI consistency
        has_plane_api = "BrepSurfacePlaneAPI" in prim_api_schemas
        plane_surface_faces = [i for i, st in enumerate(face_surface_types) if st == "BrepSurfacePlaneAPI"]

        if plane_surface_faces:
            plane_origins = brep_array.GetAttribute("brep:surface:plane:origin").Get() or []
            if not plane_origins:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_495,
                    message=f"Found {len(plane_surface_faces)} faces with face:surfaceType='BrepSurfacePlaneAPI' but no brep:surface:plane data is authored.",
                    at=brep_array,
                )

        if has_plane_api and not plane_surface_faces:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_495,
                message="BrepSurfacePlaneAPI appears in apiSchemas but no faces use face:surfaceType='BrepSurfacePlaneAPI'.",
                at=brep_array,
            )

        # BA_505: Check BrepSurfaceCylinderAPI consistency
        has_cylinder_api = "BrepSurfaceCylinderAPI" in prim_api_schemas
        cylinder_surface_faces = [i for i, st in enumerate(face_surface_types) if st == "BrepSurfaceCylinderAPI"]

        if cylinder_surface_faces:
            cylinder_origins = brep_array.GetAttribute("brep:surface:cylinder:origin").Get() or []
            if not cylinder_origins:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_505,
                    message=f"Found {len(cylinder_surface_faces)} faces with face:surfaceType='BrepSurfaceCylinderAPI' but no brep:surface:cylinder data is authored.",
                    at=brep_array,
                )

        if has_cylinder_api and not cylinder_surface_faces:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_505,
                message="BrepSurfaceCylinderAPI appears in apiSchemas but no faces use face:surfaceType='BrepSurfaceCylinderAPI'.",
                at=brep_array,
            )

        # BA_516: Check BrepSurfaceConeAPI consistency
        has_cone_api = "BrepSurfaceConeAPI" in prim_api_schemas
        cone_surface_faces = [i for i, st in enumerate(face_surface_types) if st == "BrepSurfaceConeAPI"]

        if cone_surface_faces:
            cone_origins = brep_array.GetAttribute("brep:surface:cone:origin").Get() or []
            if not cone_origins:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_516,
                    message=f"Found {len(cone_surface_faces)} faces with face:surfaceType='BrepSurfaceConeAPI' but no brep:surface:cone data is authored.",
                    at=brep_array,
                )

        if has_cone_api and not cone_surface_faces:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_516,
                message="BrepSurfaceConeAPI appears in apiSchemas but no faces use face:surfaceType='BrepSurfaceConeAPI'.",
                at=brep_array,
            )

        # BA_526: Check BrepSurfaceTorusAPI consistency
        has_torus_api = "BrepSurfaceTorusAPI" in prim_api_schemas
        torus_surface_faces = [i for i, st in enumerate(face_surface_types) if st == "BrepSurfaceTorusAPI"]

        if torus_surface_faces:
            torus_origins = brep_array.GetAttribute("brep:surface:torus:origin").Get() or []
            if not torus_origins:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_526,
                    message=f"Found {len(torus_surface_faces)} faces with face:surfaceType='BrepSurfaceTorusAPI' but no brep:surface:torus data is authored.",
                    at=brep_array,
                )

        if has_torus_api and not torus_surface_faces:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_526,
                message="BrepSurfaceTorusAPI appears in apiSchemas but no faces use face:surfaceType='BrepSurfaceTorusAPI'.",
                at=brep_array,
            )

        # BA_305: Check BrepPointAPI:vertexPoint consistency
        has_point_api = "BrepPointAPI:vertexPoint" in prim_api_schemas
        vertex_point_types = brep_array.GetAttribute("vertex:pointType").Get() or []
        brep_point_vertices = [i for i, pt in enumerate(vertex_point_types) if pt == "BrepPointAPI"]
        
        if brep_point_vertices:
            # Check if vertex point data is authored
            vertex_positions = brep_array.GetAttribute("brep:vertexPoint:point:position").Get() or []
            if not vertex_positions:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_305,
                    message=f"Found {len(brep_point_vertices)} vertices with vertex:pointType='BrepPointAPI' but no brep:vertexPoint:point:position data is authored.",
                    at=brep_array,
                )
        
        if has_point_api and not brep_point_vertices:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_305,
                message="BrepPointAPI:vertexPoint appears in apiSchemas but no vertices use vertex:pointType='BrepPointAPI'.",
                at=brep_array,
            )

    def _validate_nurbs_data_completeness(self, brep_array: Usd.Prim) -> None:
        """
        BA_080: Validate that NURBS curve data is complete and mathematically consistent. Validates per brep.
        """
        edge_curve_types = brep_array.GetAttribute("edge:curveType").Get() or []
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        if not edge_curve_types or not brep_region_counts:
            return
            
        # Get NURBS curve data arrays
        orders = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get() or []
        vertex_counts = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:vertexCount").Get() or []
        control_vertices = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:controlVertices").Get() or []
        weights = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:weights").Get() or []
        knots = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:knots").Get() or []
        
        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        edge_offsets = brep_offsets.get("edges", [])
        
        # Validate NURBS data completeness per brep
        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(edge_offsets) - 1:
                break
                
            edge_start = edge_offsets[brep_idx]
            edge_end = edge_offsets[brep_idx + 1]
            
            # Find NURBS edges in this brep
            brep_nurbs_edges = []
            for edge_idx in range(edge_start, min(edge_end, len(edge_curve_types))):
                if edge_curve_types[edge_idx] == "BrepCurve3dNurbAPI":
                    brep_nurbs_edges.append(edge_idx)
            
            if not brep_nurbs_edges:
                continue  # No NURBS edges in this brep
                
            # Check that all required arrays have sufficient data for this brep's NURBS edges
            required_arrays = {
                "order": orders,
                "vertexCount": vertex_counts, 
                "controlVertices": control_vertices,
                "weights": weights,
                "knots": knots
            }
            
            missing_arrays = [name for name, array in required_arrays.items() if not array]
            if missing_arrays:
                brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_370,
                    message=f"NURBS curve data is incomplete for brep #{brep_id_formatted}. Missing arrays: {missing_arrays}. Required for {len(brep_nurbs_edges)} edges with BrepCurve3dNurbAPI.",
                    at=brep_array,
                )

    def _validate_nurbs_mathematical_consistency(self, brep_array: Usd.Prim) -> None:
        """
        BA_081: Validate NURBS mathematical relationships. Validates per brep.
        """
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        
        if not brep_region_counts:
            return
            
        # Get brep partitioning offsets
        brep_offsets = self._compute_brep_offsets(brep_array)
        nurbs_curve_offsets = brep_offsets.get("edge3d_nurbs_curves", [])
        nurbs_surface_offsets = brep_offsets.get("surface_nurbs", [])
        
        # Validate edge NURBS data per brep
        orders = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:order").Get() or []
        vertex_counts = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:vertexCount").Get() or []
        knots = brep_array.GetAttribute("brep:edge3dNurb:curve3d:nurb:knots").Get() or []
        
        if orders and vertex_counts:
            for brep_idx in range(len(brep_region_counts)):
                if brep_idx >= len(nurbs_curve_offsets) - 1:
                    break
                    
                curve_start = nurbs_curve_offsets[brep_idx]
                curve_end = nurbs_curve_offsets[brep_idx + 1]
                
                # Validate NURBS curves belonging to this brep
                for curve_idx in range(curve_start, min(curve_end, len(orders), len(vertex_counts))):
                    order = orders[curve_idx]
                    vertex_count = vertex_counts[curve_idx]
                    
                    # BA_385: order <= vertexCount
                    if order > vertex_count:
                        brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_340,
                            message=f"Edge NURBS curve #{curve_idx} in brep #{brep_id_formatted}: order ({order}) must be <= vertexCount ({vertex_count}).",
                            at=brep_array,
                        )
        
        # Check knot vector sizes per brep
        if orders and vertex_counts and knots:
            global_knot_offset = 0
            for brep_idx in range(len(brep_region_counts)):
                if brep_idx >= len(nurbs_curve_offsets) - 1:
                    break
                    
                curve_start = nurbs_curve_offsets[brep_idx]
                curve_end = nurbs_curve_offsets[brep_idx + 1]
                
                # Validate knot counts for NURBS curves in this brep
                for curve_idx in range(curve_start, min(curve_end, len(orders), len(vertex_counts))):
                    order = orders[curve_idx]
                    vertex_count = vertex_counts[curve_idx]
                    expected_knot_count = order + vertex_count
                    
                    if global_knot_offset + expected_knot_count > len(knots):
                        brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_355,
                            message=f"Edge NURBS curve #{curve_idx} in brep #{brep_id_formatted}: insufficient knots. Expected {expected_knot_count}, but only {len(knots) - global_knot_offset} remaining.",
                            at=brep_array,
                        )
                    global_knot_offset += expected_knot_count
        
        # Validate surface NURBS data per brep
        u_orders = brep_array.GetAttribute("brep:surface:nurb:uOrder").Get() or []
        v_orders = brep_array.GetAttribute("brep:surface:nurb:vOrder").Get() or []
        u_vertex_counts = brep_array.GetAttribute("brep:surface:nurb:uVertexCount").Get() or []
        v_vertex_counts = brep_array.GetAttribute("brep:surface:nurb:vVertexCount").Get() or []
        
        if u_orders and u_vertex_counts and v_orders and v_vertex_counts:
            for brep_idx in range(len(brep_region_counts)):
                if brep_idx >= len(nurbs_surface_offsets) - 1:
                    break
                    
                surface_start = nurbs_surface_offsets[brep_idx]
                surface_end = nurbs_surface_offsets[brep_idx + 1]
                
                # Validate NURBS surfaces belonging to this brep
                for surface_idx in range(surface_start, min(surface_end, len(u_orders), len(v_orders), len(u_vertex_counts), len(v_vertex_counts))):
                    u_order = u_orders[surface_idx]
                    v_order = v_orders[surface_idx]
                    u_vertex_count = u_vertex_counts[surface_idx]
                    v_vertex_count = v_vertex_counts[surface_idx]
                    
                    # BA_385: order <= vertexCount  
                    if u_order > u_vertex_count:
                        brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_430,
                            message=f"Surface NURBS #{surface_idx} in brep #{brep_id_formatted}: uOrder ({u_order}) must be <= uVertexCount ({u_vertex_count}).",
                            at=brep_array,
                        )
                        
                    if v_order > v_vertex_count:
                        brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_430,
                            message=f"Surface NURBS #{surface_idx} in brep #{brep_id_formatted}: vOrder ({v_order}) must be <= vVertexCount ({v_vertex_count}).",
                            at=brep_array,
                        )

    def _validate_topology_geometry_correspondence(self, brep_array: Usd.Prim) -> None:
        """
        BA_320, BA_225: Validate that topological and geometric arrays have corresponding entries.
        Performs per-brep validation with proper error message context.
        """
        # BA_320: Check that topological arrays have corresponding geometric data
        brep_offsets = self._compute_brep_offsets(brep_array)
        vertex_offsets = brep_offsets.get("vertices", [])
        edge_offsets = brep_offsets.get("edges", [])
        
        # Check vertex correspondence per brep
        vertex_point_types, _ = self._get_attr_sequence(
            brep_array,
            "vertex:pointType",
            cap.BrepArrayRequirements.BA_316,
            "int[]",
        )
        vertex_positions, _ = self._get_attr_sequence(
            brep_array,
            "brep:vertexPoint:point:position",
            cap.BrepArrayRequirements.BA_326,
            "point3d[]",
        )
        vertex_point_types = brep_array.GetAttribute("vertex:pointType").Get() or []
        
        # Check per-brep vertex correspondence
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        position_idx = 0
        for brep_idx, (start_idx, end_idx) in enumerate(zip(vertex_offsets[:-1], vertex_offsets[1:])):
            brep_point_vertices = []
            brep_vertex_count = end_idx - start_idx
            
            for vertex_idx in range(start_idx, end_idx):
                if vertex_idx < len(vertex_point_types) and vertex_point_types[vertex_idx] == "BrepPointAPI":
                    brep_point_vertices.append(vertex_idx)
            
            expected_positions = len(brep_point_vertices)
            if brep_point_vertices and vertex_positions:
                available_positions = len(vertex_positions) - position_idx
                if available_positions < expected_positions:
                    brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_320,
                        message=f"Mismatch between vertex topology ({expected_positions} BrepPointAPI vertices) and geometry ({available_positions} remaining positions) for brep #{brep_id_formatted}.",
                        at=brep_array,
                    )
                position_idx += expected_positions
        
        # BA_225: Check cross-array index consistency per brep
        edge_vertex_indices_val = brep_array.GetAttribute("edge:vertexIndices").Get()
        if self._is_unregistered_value(edge_vertex_indices_val):
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_237,
                message="edge:vertexIndices has an unregistered USD type; expected int2[].",
                at=brep_array,
            )
            return
        edge_vertex_indices = edge_vertex_indices_val or []
        if edge_vertex_indices and vertex_point_types and hasattr(edge_vertex_indices, '__iter__'):
            for brep_idx, (edge_start, edge_end) in enumerate(zip(edge_offsets[:-1], edge_offsets[1:])):
                vertex_start = vertex_offsets[brep_idx] if brep_idx < len(vertex_offsets) else 0
                vertex_end = vertex_offsets[brep_idx + 1] if brep_idx + 1 < len(vertex_offsets) else len(vertex_point_types)
                max_vertex_index = vertex_end - 1
                min_vertex_index = vertex_start
                
                for local_edge_idx, edge_idx in enumerate(range(edge_start, edge_end)):
                    if edge_idx >= len(edge_vertex_indices):
                        continue
                    
                    vertex_pair = edge_vertex_indices[edge_idx]
                    for vertex_index in vertex_pair:
                        if vertex_index < min_vertex_index or vertex_index > max_vertex_index:
                            brep_id_formatted = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_225,
                                message=f"Edge #{local_edge_idx} in brep #{brep_id_formatted} references invalid vertex index {vertex_index}. Valid range for this brep: [{min_vertex_index}, {max_vertex_index}].",
                                at=brep_array,
                            )

    ### Data Type Validation Methods ###

    def _is_unregistered_value(self, value) -> bool:
        if value is None:
            return False
        uv_type = getattr(Sdf, "UnregisteredValue", None)
        try:
            if uv_type is not None and isinstance(value, uv_type):
                return True
        except TypeError:
            return False
        try:
            return "UnregisteredValue" in str(type(value))
        except TypeError:
            return False

    def _len_or_none(self, value):
        try:
            return len(value)
        except (TypeError, AttributeError):
            return None

    def _get_attr_sequence(self, brep_array: Usd.Prim, attr_name: str, requirement, expected_desc: str):
        """
        Safely fetch an attribute value intended to be a sequence.
        - On unregistered type: record a failure and return ([], 0)
        - On non-iterable / len error: record a failure and return ([], 0)
        """
        attr = brep_array.GetAttribute(attr_name)
        if not attr:
            return [], 0
        try:
            if hasattr(attr, "HasAuthoredValue") and not attr.HasAuthoredValue():
                return [], 0
            if hasattr(attr, "IsAuthored") and not attr.IsAuthored():
                return [], 0
        except (RuntimeError, AttributeError):
            return [], 0

        value = attr.Get()
        if self._is_unregistered_value(value):
            self._AddFailedCheck(
                requirement=requirement,
                message=f"{attr_name} has an unregistered USD type; expected {expected_desc}.",
                at=brep_array,
            )
            return [], 0
        try:
            seq_len = len(value)
            return value, seq_len
        except (TypeError, AttributeError):
            self._AddFailedCheck(
                requirement=requirement,
                message=f"{attr_name} is not a valid sequence; expected {expected_desc}.",
                at=brep_array,
            )
            return [], 0

    def _get_authored_type_name(self, attr: Usd.Attribute):
        """
        Return the authored Sdf type token if available (before schema normalization).
        Falls back to None when no authored type is present.
        """
        try:
            specs = attr.GetPropertyStack(Usd.TimeCode.Default())
        except (RuntimeError, AttributeError):
            return None

        for spec in specs:
            try:
                if isinstance(spec, Sdf.AttributeSpec) and spec.typeName:
                    type_name = spec.typeName
                    try:
                        return type_name.GetAsToken()
                    except (RuntimeError, AttributeError):
                        return str(type_name)
            except (RuntimeError, AttributeError):
                continue
        return None

    def _validate_attribute_data_types(self, brep_array: Usd.Prim) -> None:
        """
        Comprehensive data type validation for all BrepArray attributes.
        
        Validates that all attributes use the correct USD data types as specified
        in the BrepArray schema requirements BA_061 through BA_471.
        """
        
        # BA_061: Brep attribute data types
        brep_attributes = {
            "brep:intersectTol3d": "double[]", 
            "brep:extent": "double3[]",
            "brep:regionCount": "uint[]"
        }
        self._validate_stratum_data_types(brep_array, brep_attributes, cap.BrepArrayRequirements.BA_061, "Brep")
        
        # BA_076: Region attribute data types
        region_attributes = {
            "region:shellCount": "uint[]",
            "region:type": "token[]"
        }
        self._validate_stratum_data_types(brep_array, region_attributes, cap.BrepArrayRequirements.BA_076, "Region")
        
        # BA_091: Shell attribute data types
        shell_attributes = {
            "shell:faceuseCount": "uint[]",
            "shell:wireEdgeCount": "uint[]",
            "shell:pointType": "token[]"
        }
        self._validate_stratum_data_types(brep_array, shell_attributes, cap.BrepArrayRequirements.BA_091, "Shell")
        
        # BA_116: Faceuse attribute data types
        faceuse_attributes = {
            "faceuse:faceIndex": "uint[]",
            "faceuse:orientationType": "token[]"
        }
        self._validate_stratum_data_types(brep_array, faceuse_attributes, cap.BrepArrayRequirements.BA_116, "Faceuse")
        
        # BA_161: Face attribute data types
        face_attributes = {
            "face:loopCount": "uint[]",
            "face:trimType": "token[]",
            "face:surfaceType": "token[]",
            "face:range": "double2[]"
        }
        self._validate_stratum_data_types(brep_array, face_attributes, cap.BrepArrayRequirements.BA_161, "Face")
        
        # BA_176: Loop attribute data types
        loop_attributes = {
            "loop:edgeuseCount": "uint[]",
            "loop:vertexIndex": "uint[]"
        }
        self._validate_stratum_data_types(brep_array, loop_attributes, cap.BrepArrayRequirements.BA_176, "Loop")
        
        # BA_196: Edgeuse attribute data types
        edgeuse_attributes = {
            "edgeuse:edgeIndex": "uint[]",
            "edgeuse:orientationType": "token[]",
            "edgeuse:nextRadialEUIndex": "uint[]",
            "edgeuse:thisRadialEntryType": "token[]"
        }
        self._validate_stratum_data_types(brep_array, edgeuse_attributes, cap.BrepArrayRequirements.BA_196, "Edgeuse")
        
        # BA_237: Edge attribute data types
        edge_attributes = {
            "edge:curveType": "token[]",
            "edge:vertexIndices": "int2[]",
            "edge:range": "double[]"
        }
        self._validate_stratum_data_types(brep_array, edge_attributes, cap.BrepArrayRequirements.BA_237, "Edge")
        
        # BA_291: WireEdge attribute data types
        wireEdge_attributes = {
            "wireEdge:curveType": "token[]",
            "wireEdge:vertexIndices": "int2[]",
            "wireEdge:range": "double[]"
        }
        self._validate_stratum_data_types(brep_array, wireEdge_attributes, cap.BrepArrayRequirements.BA_291, "WireEdge")
        
        # BA_316: Vertex attribute data types
        vertex_attributes = {
            "vertex:pointType": "token[]"
        }
        self._validate_stratum_data_types(brep_array, vertex_attributes, cap.BrepArrayRequirements.BA_316, "Vertex")
        
        # BA_326: VertexPoint attribute data types
        vertexPoint_attributes = {
            "brep:vertexPoint:point:position": "point3d[]"
        }
        self._validate_stratum_data_types(brep_array, vertexPoint_attributes, cap.BrepArrayRequirements.BA_326, "VertexPoint")
        
        # BA_327: ShellPoint attribute data types
        shellPoint_attributes = {
            "brep:shellPoint:point:position": "point3d[]"
        }
        self._validate_stratum_data_types(brep_array, shellPoint_attributes, cap.BrepArrayRequirements.BA_327, "ShellPoint")
        
        # BA_371: Edge3dNurb attribute data types
        edge3dNurb_attributes = {
            "brep:edge3dNurb:curve3d:nurb:order": "uint[]",
            "brep:edge3dNurb:curve3d:nurb:vertexCount": "uint[]",
            "brep:edge3dNurb:curve3d:nurb:controlVertices": "point3d[]",
            "brep:edge3dNurb:curve3d:nurb:weights": "double[]",
            "brep:edge3dNurb:curve3d:nurb:knots": "double[]"
        }
        self._validate_stratum_data_types(brep_array, edge3dNurb_attributes, cap.BrepArrayRequirements.BA_371, "Edge3dNurb")
        
        # BA_416: CurveUv attribute data types
        curveUv_attributes = {
            "brep:curveUv:nurb:order": "uint[]",
            "brep:curveUv:nurb:vertexCount": "uint[]",
            "brep:curveUv:nurb:controlVertices": "double2[]",
            "brep:curveUv:nurb:weights": "double[]",
            "brep:curveUv:nurb:knots": "double[]"
        }
        self._validate_stratum_data_types(brep_array, curveUv_attributes, cap.BrepArrayRequirements.BA_416, "CurveUv")
        
        # BA_471: Surface attribute data types
        surface_attributes = {
            "brep:surface:nurb:uOrder": "uint[]",
            "brep:surface:nurb:vOrder": "uint[]",
            "brep:surface:nurb:uVertexCount": "uint[]",
            "brep:surface:nurb:vVertexCount": "uint[]",
            "brep:surface:nurb:controlVertices": "point3d[]",
            "brep:surface:nurb:weights": "double[]",
            "brep:surface:nurb:uKnots": "double[]",
            "brep:surface:nurb:vKnots": "double[]"
        }
        self._validate_stratum_data_types(brep_array, surface_attributes, cap.BrepArrayRequirements.BA_471, "Surface")

    def _validate_stratum_data_types(self, brep_array: Usd.Prim, attributes: dict, requirement, stratum_name: str) -> None:
        """
        Validate data types for a specific topology/geometry stratum.
        
        Checks that the authored type in the USD file matches the type defined
        by the schema.  The hardcoded *attributes* dict is still accepted for
        backwards compatibility (and as a fallback when no schema type is
        available), but the primary comparison is now authored-vs-schema.

        Args:
            brep_array: The BrepArray prim to validate
            attributes: Dictionary mapping attribute names to expected USD type names
            requirement: The BrepArrayRequirements enum value for this stratum
            stratum_name: Human-readable name of the stratum for error messages
        """
        prim_def = brep_array.GetPrimDefinition()
        for attr_name, expected_type in attributes.items():
            attr = brep_array.GetAttribute(attr_name)
            if attr and attr.HasAuthoredValue():
                schema_type = str(attr.GetTypeName())
                authored_type = self._get_authored_type_name(attr)
                actual_type = authored_type or schema_type
                schema_defines_attr = prim_def is not None and attr_name in prim_def.GetPropertyNames()
                expected = schema_type if schema_defines_attr else expected_type
                if actual_type != expected:
                    authored_note = f" Authored type: '{authored_type}'." if authored_type else ""
                    self._AddFailedCheck(
                        requirement=requirement,
                        message=(
                            f"Invalid data type for {attr_name}. Schema expects '{expected}' but got '{actual_type}'."
                            f"{authored_note} {stratum_name} attributes must use correct USD data types for proper schema validation."
                        ),
                        at=brep_array,
                    )

    ### Analytic Curve/Surface Consistency Validation (BA_590 - BA_640) ###

    def _validate_nurbs_order_and_vertex_count_values(self, brep_array: Usd.Prim) -> None:
        """
        BA_590: NURBS order values (int[]) must be >= 2 (degree >= 1).
        BA_591: NURBS vertexCount values must be >= the corresponding order.
        Validates edge, wireEdge, curveUv, and surface NURBS order/vertexCount.
        """
        order_vc_pairs = [
            ("brep:edge3dNurb:curve3d:nurb:order", "brep:edge3dNurb:curve3d:nurb:vertexCount", "edge3dNurb", False),
            ("brep:surface:nurb:uOrder", "brep:surface:nurb:uVertexCount", "surface U", False),
            ("brep:surface:nurb:vOrder", "brep:surface:nurb:vVertexCount", "surface V", False),
            ("brep:curveUv:nurb:order", "brep:curveUv:nurb:vertexCount", "curveUv", True),
        ]

        for order_attr_name, vc_attr_name, label, allows_zero_sentinel in order_vc_pairs:
            order_attr = brep_array.GetAttribute(order_attr_name)
            vc_attr = brep_array.GetAttribute(vc_attr_name)
            if not order_attr or not order_attr.IsAuthored():
                continue
            orders = order_attr.Get()
            vcs = vc_attr.Get() if vc_attr and vc_attr.IsAuthored() else None
            if not orders:
                continue

            for i, order_val in enumerate(orders):
                if allows_zero_sentinel and order_val == 0 and vcs and i < len(vcs) and vcs[i] == 0:
                    continue
                if order_val < 2:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_590,
                        message=f"{order_attr_name}[{i}] = {order_val} is less than 2 (minimum for {label}).",
                        at=brep_array,
                    )
                    break

            if vcs and orders and len(vcs) == len(orders):
                for i in range(len(orders)):
                    if allows_zero_sentinel and orders[i] == 0 and vcs[i] == 0:
                        continue
                    if vcs[i] < orders[i]:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_591,
                            message=f"{vc_attr_name}[{i}] = {vcs[i]} is less than {order_attr_name}[{i}] = {orders[i]} for {label}.",
                            at=brep_array,
                        )
                        break

        # wireEdge NURBS (multi-applied, same attribute names as edge but different instance)
        we_order_attr = brep_array.GetAttribute("brep:wireEdge3dNurb:curve3d:nurb:order")
        we_vc_attr = brep_array.GetAttribute("brep:wireEdge3dNurb:curve3d:nurb:vertexCount")
        if we_order_attr and we_order_attr.IsAuthored():
            we_orders = we_order_attr.Get()
            we_vcs = we_vc_attr.Get() if we_vc_attr and we_vc_attr.IsAuthored() else None
            if we_orders:
                for i, order_val in enumerate(we_orders):
                    if order_val < 2:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_590,
                            message=f"brep:wireEdge3dNurb:curve3d:nurb:order[{i}] = {order_val} is less than 2 (minimum for wireEdge3dNurb).",
                            at=brep_array,
                        )
                        break
                if we_vcs and len(we_vcs) == len(we_orders):
                    for i in range(len(we_orders)):
                        if we_vcs[i] < we_orders[i]:
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_591,
                                message=f"brep:wireEdge3dNurb:curve3d:nurb:vertexCount[{i}] = {we_vcs[i]} is less than order[{i}] = {we_orders[i]} for wireEdge3dNurb.",
                                at=brep_array,
                            )
                            break

    def _validate_edge_curve_endpoint_vertex_consistency(self, brep_array: Usd.Prim) -> None:
        """
        BA_600: Line edge endpoints evaluated at range[0]/range[1] must match vertex positions.
        BA_601: Circle edge endpoints evaluated at range[0]/range[1] must match vertex positions.
        BA_602: Ellipse edge endpoints evaluated at range[0]/range[1] must match vertex positions.
        """
        curve_types = BrepConstants.safe_get_attribute(brep_array, "edge:curveType")
        edge_ranges_val = brep_array.GetAttribute("edge:range").Get()
        if self._is_unregistered_value(edge_ranges_val):
            return
        edge_ranges = edge_ranges_val or []
        edge_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "edge:vertexIndices")
        vertex_positions = BrepConstants.safe_get_attribute(brep_array, "brep:vertexPoint:point:position")
        intersect_tols = BrepConstants.safe_get_attribute(brep_array, "brep:intersectTol3d")

        if not curve_types or not edge_ranges or not edge_vertex_indices or not vertex_positions:
            return

        num_edges = len(curve_types)
        if len(edge_ranges) < num_edges * 2:
            return

        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        brep_offsets = self._compute_brep_offsets(brep_array)
        edge_brep_indices = self._compute_edge_brep_indices(
            brep_array,
            brep_offsets.get("edgeuses", []),
            BrepConstants.safe_get_attribute(brep_array, "edgeuse:edgeIndex"),
        )

        line_origins = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dLine:curve3d:line:origin")
        line_dirs = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dLine:curve3d:line:direction")

        circle_centers = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dCircle:curve3d:circle:center")
        circle_axes = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dCircle:curve3d:circle:axis")
        circle_ref_dirs = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dCircle:curve3d:circle:refDirection")
        circle_radii = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dCircle:curve3d:circle:radius")

        ellipse_centers = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dEllipse:curve3d:ellipse:center")
        ellipse_axes = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dEllipse:curve3d:ellipse:axis")
        ellipse_ref_dirs = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dEllipse:curve3d:ellipse:refDirection")
        ellipse_x_radii = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dEllipse:curve3d:ellipse:xRadius")
        ellipse_y_radii = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dEllipse:curve3d:ellipse:yRadius")

        line_idx = 0
        circle_idx = 0
        ellipse_idx = 0

        for edge_idx in range(num_edges):
            ctype = str(curve_types[edge_idx])
            t_min = float(edge_ranges[2 * edge_idx])
            t_max = float(edge_ranges[2 * edge_idx + 1])
            vi = edge_vertex_indices[edge_idx]
            v0_idx, v1_idx = int(vi[0]), int(vi[1])
            tol, _, brep_id = self._get_edge_intersect_tolerance(
                brep_array, edge_idx, intersect_tols, brep_region_counts, edge_brep_indices
            )

            if v0_idx < 0 or v0_idx >= len(vertex_positions) or v1_idx < 0 or v1_idx >= len(vertex_positions):
                if ctype == "BrepCurve3dLineAPI":
                    line_idx += 1
                elif ctype == "BrepCurve3dCircleAPI":
                    circle_idx += 1
                elif ctype == "BrepCurve3dEllipseAPI":
                    ellipse_idx += 1
                continue

            v0 = vertex_positions[v0_idx]
            v1 = vertex_positions[v1_idx]

            if ctype == "BrepCurve3dLineAPI":
                if line_idx < len(line_origins) and line_idx < len(line_dirs):
                    if tol is None:
                        self._report_unresolved_edge_tolerance(
                            brep_array,
                            cap.BrepArrayRequirements.BA_600,
                            edge_idx,
                            "Line endpoint-to-vertex consistency",
                        )
                        line_idx += 1
                        continue
                    o = line_origins[line_idx]
                    d = line_dirs[line_idx]
                    for t_val, v_pos, endpoint_name in [(t_min, v0, "start"), (t_max, v1, "end")]:
                        px = o[0] + t_val * d[0]
                        py = o[1] + t_val * d[1]
                        pz = o[2] + t_val * d[2]
                        dist = math.sqrt((px - v_pos[0])**2 + (py - v_pos[1])**2 + (pz - v_pos[2])**2)
                        if dist > tol:
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_600,
                                message=(
                                    f"Line edge #{edge_idx} {endpoint_name} point evaluated at t={t_val:.6f} is "
                                    f"({px:.6f}, {py:.6f}, {pz:.6f}), but vertex #{v0_idx if endpoint_name == 'start' else v1_idx} "
                                    f"is at ({v_pos[0]:.6f}, {v_pos[1]:.6f}, {v_pos[2]:.6f}), distance={dist:.6f} "
                                    f"exceeds brep:intersectTol3d[{brep_id}] = {tol}."
                                ),
                                at=brep_array,
                            )
                            break
                line_idx += 1

            elif ctype == "BrepCurve3dCircleAPI":
                if circle_idx < len(circle_centers) and circle_idx < len(circle_axes) and \
                   circle_idx < len(circle_ref_dirs) and circle_idx < len(circle_radii):
                    if tol is None:
                        self._report_unresolved_edge_tolerance(
                            brep_array,
                            cap.BrepArrayRequirements.BA_601,
                            edge_idx,
                            "Circle endpoint-to-vertex consistency",
                        )
                        circle_idx += 1
                        continue
                    center = circle_centers[circle_idx]
                    axis = circle_axes[circle_idx]
                    ref_dir = circle_ref_dirs[circle_idx]
                    radius = float(circle_radii[circle_idx])
                    y_dir = (
                        axis[1]*ref_dir[2] - axis[2]*ref_dir[1],
                        axis[2]*ref_dir[0] - axis[0]*ref_dir[2],
                        axis[0]*ref_dir[1] - axis[1]*ref_dir[0],
                    )
                    for t_val, v_pos, endpoint_name in [(t_min, v0, "start"), (t_max, v1, "end")]:
                        cos_t = math.cos(t_val)
                        sin_t = math.sin(t_val)
                        px = center[0] + radius * (cos_t * ref_dir[0] + sin_t * y_dir[0])
                        py = center[1] + radius * (cos_t * ref_dir[1] + sin_t * y_dir[1])
                        pz = center[2] + radius * (cos_t * ref_dir[2] + sin_t * y_dir[2])
                        dist = math.sqrt((px - v_pos[0])**2 + (py - v_pos[1])**2 + (pz - v_pos[2])**2)
                        if dist > tol:
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_601,
                                message=(
                                    f"Circle edge #{edge_idx} {endpoint_name} point evaluated at t={t_val:.6f} is "
                                    f"({px:.6f}, {py:.6f}, {pz:.6f}), but vertex #{v0_idx if endpoint_name == 'start' else v1_idx} "
                                    f"is at ({v_pos[0]:.6f}, {v_pos[1]:.6f}, {v_pos[2]:.6f}), distance={dist:.6f} "
                                    f"exceeds brep:intersectTol3d[{brep_id}] = {tol}."
                                ),
                                at=brep_array,
                            )
                            break
                circle_idx += 1

            elif ctype == "BrepCurve3dEllipseAPI":
                if ellipse_idx < len(ellipse_centers) and ellipse_idx < len(ellipse_axes) and \
                   ellipse_idx < len(ellipse_ref_dirs) and ellipse_idx < len(ellipse_x_radii) and \
                   ellipse_idx < len(ellipse_y_radii):
                    if tol is None:
                        self._report_unresolved_edge_tolerance(
                            brep_array,
                            cap.BrepArrayRequirements.BA_602,
                            edge_idx,
                            "Ellipse endpoint-to-vertex consistency",
                        )
                        ellipse_idx += 1
                        continue
                    center = ellipse_centers[ellipse_idx]
                    axis = ellipse_axes[ellipse_idx]
                    ref_dir = ellipse_ref_dirs[ellipse_idx]
                    x_r = float(ellipse_x_radii[ellipse_idx])
                    y_r = float(ellipse_y_radii[ellipse_idx])
                    y_dir = (
                        axis[1]*ref_dir[2] - axis[2]*ref_dir[1],
                        axis[2]*ref_dir[0] - axis[0]*ref_dir[2],
                        axis[0]*ref_dir[1] - axis[1]*ref_dir[0],
                    )
                    for t_val, v_pos, endpoint_name in [(t_min, v0, "start"), (t_max, v1, "end")]:
                        cos_t = math.cos(t_val)
                        sin_t = math.sin(t_val)
                        px = center[0] + x_r * cos_t * ref_dir[0] + y_r * sin_t * y_dir[0]
                        py = center[1] + x_r * cos_t * ref_dir[1] + y_r * sin_t * y_dir[1]
                        pz = center[2] + x_r * cos_t * ref_dir[2] + y_r * sin_t * y_dir[2]
                        dist = math.sqrt((px - v_pos[0])**2 + (py - v_pos[1])**2 + (pz - v_pos[2])**2)
                        if dist > tol:
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_602,
                                message=(
                                    f"Ellipse edge #{edge_idx} {endpoint_name} point evaluated at t={t_val:.6f} is "
                                    f"({px:.6f}, {py:.6f}, {pz:.6f}), but vertex #{v0_idx if endpoint_name == 'start' else v1_idx} "
                                    f"is at ({v_pos[0]:.6f}, {v_pos[1]:.6f}, {v_pos[2]:.6f}), distance={dist:.6f} "
                                    f"exceeds brep:intersectTol3d[{brep_id}] = {tol}."
                                ),
                                at=brep_array,
                            )
                            break
                ellipse_idx += 1

    def _validate_circle_vertex_radius_consistency(self, brep_array: Usd.Prim) -> None:
        """
        BA_610: Circle edge endpoint vertices must lie at distance radius from the
        circle center within tolerance.
        """
        curve_types = BrepConstants.safe_get_attribute(brep_array, "edge:curveType")
        edge_vertex_indices = BrepConstants.safe_get_attribute(brep_array, "edge:vertexIndices")
        vertex_positions = BrepConstants.safe_get_attribute(brep_array, "brep:vertexPoint:point:position")
        circle_centers = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dCircle:curve3d:circle:center")
        circle_radii = BrepConstants.safe_get_attribute(brep_array, "brep:edge3dCircle:curve3d:circle:radius")
        intersect_tols = BrepConstants.safe_get_attribute(brep_array, "brep:intersectTol3d")

        if not curve_types or not edge_vertex_indices or not vertex_positions or not circle_centers or not circle_radii:
            return

        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        brep_offsets = self._compute_brep_offsets(brep_array)
        edge_brep_indices = self._compute_edge_brep_indices(
            brep_array,
            brep_offsets.get("edgeuses", []),
            BrepConstants.safe_get_attribute(brep_array, "edgeuse:edgeIndex"),
        )

        circle_idx = 0
        for edge_idx, ctype in enumerate(curve_types):
            if str(ctype) != "BrepCurve3dCircleAPI":
                continue

            if circle_idx >= len(circle_centers) or circle_idx >= len(circle_radii):
                circle_idx += 1
                continue

            if edge_idx >= len(edge_vertex_indices):
                circle_idx += 1
                continue

            center = circle_centers[circle_idx]
            radius = float(circle_radii[circle_idx])
            vi = edge_vertex_indices[edge_idx]
            tol, _, brep_id = self._get_edge_intersect_tolerance(
                brep_array, edge_idx, intersect_tols, brep_region_counts, edge_brep_indices
            )
            if tol is None:
                self._report_unresolved_edge_tolerance(
                    brep_array,
                    cap.BrepArrayRequirements.BA_610,
                    edge_idx,
                    "Circle vertex-radius consistency",
                )
                circle_idx += 1
                continue

            for k, v_idx in enumerate([int(vi[0]), int(vi[1])]):
                if v_idx < 0 or v_idx >= len(vertex_positions):
                    continue
                v = vertex_positions[v_idx]
                dist = math.sqrt((v[0] - center[0])**2 + (v[1] - center[1])**2 + (v[2] - center[2])**2)
                if abs(dist - radius) > tol:
                    endpoint_name = "start" if k == 0 else "end"
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_610,
                        message=(
                            f"Circle edge #{edge_idx} {endpoint_name} vertex #{v_idx} is at distance "
                            f"{dist:.6f} from center ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}), "
                            f"but radius is {radius:.6f}. Difference = {abs(dist - radius):.6f} "
                            f"exceeds brep:intersectTol3d[{brep_id}] = {tol}."
                        ),
                        at=brep_array,
                    )
                    break
            circle_idx += 1

    @staticmethod
    def _surface_face_indices(face_surface_types, target_type):
        """Return list of face indices whose surfaceType matches *target_type*."""
        return [fi for fi, st in enumerate(face_surface_types) if str(st) == target_type]

    @staticmethod
    def _point_inside_expanded_extent(pt, global_min, global_max, margin):
        for c in range(3):
            if pt[c] < global_min[c] - margin or pt[c] > global_max[c] + margin:
                return False
        return True

    def _compute_plane_face_bbox(self, origin, axis, ref_dir, face_range, fi):
        """Return (bbox_min, bbox_max) of a plane face from its parameter range."""
        bx = axis[1] * ref_dir[2] - axis[2] * ref_dir[1]
        by = axis[2] * ref_dir[0] - axis[0] * ref_dir[2]
        bz = axis[0] * ref_dir[1] - axis[1] * ref_dir[0]
        binormal = (bx, by, bz)

        u_min, v_min = float(face_range[fi * 2][0]), float(face_range[fi * 2][1])
        u_max, v_max = float(face_range[fi * 2 + 1][0]), float(face_range[fi * 2 + 1][1])

        bbox_min = [float('inf')] * 3
        bbox_max = [float('-inf')] * 3
        for u, v in [(u_min, v_min), (u_max, v_min), (u_min, v_max), (u_max, v_max)]:
            for c, (o, r, b) in enumerate(zip(origin, ref_dir, binormal)):
                val = float(o) + u * float(r) + v * float(b)
                bbox_min[c] = min(bbox_min[c], val)
                bbox_max[c] = max(bbox_max[c], val)
        return bbox_min, bbox_max

    def _compute_cylinder_face_axial_bbox(self, origin, axis, radius, ref_dir, face_range, fi):
        """Return (bbox_min, bbox_max) of a cylinder face from its parameter range."""
        bx = axis[1] * ref_dir[2] - axis[2] * ref_dir[1]
        by = axis[2] * ref_dir[0] - axis[0] * ref_dir[2]
        bz = axis[0] * ref_dir[1] - axis[1] * ref_dir[0]
        binormal = (bx, by, bz)

        u_min, v_min = float(face_range[fi * 2][0]), float(face_range[fi * 2][1])
        u_max, v_max = float(face_range[fi * 2 + 1][0]), float(face_range[fi * 2 + 1][1])

        bbox_min = [float('inf')] * 3
        bbox_max = [float('-inf')] * 3
        for v in [v_min, v_max]:
            for u in [u_min, u_max, (u_min + u_max) / 2.0]:
                cos_u = math.cos(u)
                sin_u = math.sin(u)
                for c in range(3):
                    val = (float(origin[c])
                           + float(radius) * (cos_u * float(ref_dir[c]) + sin_u * float(binormal[c]))
                           + v * float(axis[c]))
                    bbox_min[c] = min(bbox_min[c], val)
                    bbox_max[c] = max(bbox_max[c], val)
        # Conservatively expand by radius to cover all angular positions
        for c in range(3):
            bbox_min[c] -= float(radius)
            bbox_max[c] += float(radius)
        return bbox_min, bbox_max

    def _compute_cone_face_axial_bbox(self, origin, axis, radius, ref_dir, semi_angle, face_range, fi):
        """Return (bbox_min, bbox_max) of a cone face from its parameter range.

        Cone parameterisation: P(u,v) = origin + v*axis + r(v)*(cos(u)*ref + sin(u)*binormal)
        where r(v) = radius + v*tan(semi_angle).
        """
        bx = axis[1] * ref_dir[2] - axis[2] * ref_dir[1]
        by = axis[2] * ref_dir[0] - axis[0] * ref_dir[2]
        bz = axis[0] * ref_dir[1] - axis[1] * ref_dir[0]
        binormal = (bx, by, bz)

        u_min, v_min = float(face_range[fi * 2][0]), float(face_range[fi * 2][1])
        u_max, v_max = float(face_range[fi * 2 + 1][0]), float(face_range[fi * 2 + 1][1])

        tan_a = math.tan(float(semi_angle))
        rad_base = float(radius)

        bbox_min = [float('inf')] * 3
        bbox_max = [float('-inf')] * 3
        for v in [v_min, v_max]:
            r_v = rad_base + v * tan_a
            for u in [u_min, u_max, (u_min + u_max) / 2.0]:
                cos_u = math.cos(u)
                sin_u = math.sin(u)
                for c in range(3):
                    val = (float(origin[c])
                           + v * float(axis[c])
                           + r_v * (cos_u * float(ref_dir[c]) + sin_u * float(binormal[c])))
                    bbox_min[c] = min(bbox_min[c], val)
                    bbox_max[c] = max(bbox_max[c], val)
        # Conservatively expand by the largest radius to cover all angular positions
        max_r = max(abs(rad_base + v_min * tan_a), abs(rad_base + v_max * tan_a))
        for c in range(3):
            bbox_min[c] -= max_r
            bbox_max[c] += max_r
        return bbox_min, bbox_max

    def _validate_analytic_surface_origin_containment(self, brep_array: Usd.Prim) -> None:
        """
        BA_620: Analytic surface origins/centers must lie within a reasonable
        expansion of the brep extent.

        For planes and cylinders whose origin is far from the face (common when
        the origin sits at the assembly coordinate system), the actual face
        position is computed from origin + face-range * basis vectors.  The
        check only fires when both the raw origin AND the evaluated face
        geometry lie outside the expanded extent.
        """
        brep_extents = brep_array.GetAttribute("brep:extent").Get() or []
        if not brep_extents or len(brep_extents) < 2:
            return

        global_min = [float('inf')] * 3
        global_max = [float('-inf')] * 3
        for i in range(0, len(brep_extents), 2):
            if i + 1 >= len(brep_extents):
                break
            lo = brep_extents[i]
            hi = brep_extents[i + 1]
            for c in range(3):
                global_min[c] = min(global_min[c], float(lo[c]))
                global_max[c] = max(global_max[c], float(hi[c]))

        diag = math.sqrt(sum((global_max[c] - global_min[c])**2 for c in range(3)))
        if diag < 1e-12:
            return
        margin = diag * 2.0

        face_surface_types = BrepConstants.safe_get_attribute(brep_array, "face:surfaceType") or []
        face_ranges_val = brep_array.GetAttribute("face:range").Get()
        face_ranges = None if self._is_unregistered_value(face_ranges_val) else face_ranges_val

        surface_origin_attrs = [
            ("brep:surface:plane:origin", "BrepSurfacePlaneAPI", "Plane"),
            ("brep:surface:cylinder:origin", "BrepSurfaceCylinderAPI", "Cylinder"),
            ("brep:surface:cone:origin", "BrepSurfaceConeAPI", "Cone"),
            ("brep:surface:sphere:center", "BrepSurfaceSphereAPI", "Sphere"),
            ("brep:surface:torus:origin", "BrepSurfaceTorusAPI", "Torus"),
        ]

        for origin_attr_name, surface_type_token, surface_label in surface_origin_attrs:
            origins = BrepConstants.safe_get_attribute(brep_array, origin_attr_name)
            if not origins:
                continue

            face_indices = self._surface_face_indices(face_surface_types, surface_type_token)

            for i, origin in enumerate(origins):
                origin_inside = self._point_inside_expanded_extent(
                    [float(origin[c]) for c in range(3)], global_min, global_max, margin
                )
                if origin_inside:
                    continue

                # Origin is outside -- check if actual face geometry is within extent.
                if face_ranges and i < len(face_indices):
                    fi = face_indices[i]
                    if fi * 2 + 1 < len(face_ranges):
                        face_geo_inside = False
                        if surface_label == "Plane":
                            plane_axes = BrepConstants.safe_get_attribute(brep_array, "brep:surface:plane:axis")
                            plane_refs = BrepConstants.safe_get_attribute(brep_array, "brep:surface:plane:refDirection")
                            if plane_axes and plane_refs and i < len(plane_axes) and i < len(plane_refs):
                                fmin, fmax = self._compute_plane_face_bbox(
                                    origin, plane_axes[i], plane_refs[i], face_ranges, fi
                                )
                                face_geo_inside = all(
                                    fmax[c] >= global_min[c] - margin and fmin[c] <= global_max[c] + margin
                                    for c in range(3)
                                )
                        elif surface_label == "Cylinder":
                            cyl_axes = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cylinder:axis")
                            cyl_refs = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cylinder:refDirection")
                            cyl_radii = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cylinder:radius")
                            if (cyl_axes and cyl_refs and cyl_radii
                                    and i < len(cyl_axes) and i < len(cyl_refs) and i < len(cyl_radii)):
                                fmin, fmax = self._compute_cylinder_face_axial_bbox(
                                    origin, cyl_axes[i], cyl_radii[i], cyl_refs[i], face_ranges, fi
                                )
                                face_geo_inside = all(
                                    fmax[c] >= global_min[c] - margin and fmin[c] <= global_max[c] + margin
                                    for c in range(3)
                                )
                        elif surface_label == "Cone":
                            cone_axes = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cone:axis")
                            cone_refs = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cone:refDirection")
                            cone_radii = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cone:radius")
                            cone_angles = BrepConstants.safe_get_attribute(brep_array, "brep:surface:cone:semiAngle")
                            if (cone_axes and cone_refs and cone_radii and cone_angles
                                    and i < len(cone_axes) and i < len(cone_refs)
                                    and i < len(cone_radii) and i < len(cone_angles)):
                                fmin, fmax = self._compute_cone_face_axial_bbox(
                                    origin, cone_axes[i], cone_radii[i], cone_refs[i],
                                    cone_angles[i], face_ranges, fi
                                )
                                face_geo_inside = all(
                                    fmax[c] >= global_min[c] - margin and fmin[c] <= global_max[c] + margin
                                    for c in range(3)
                                )

                        if face_geo_inside:
                            continue

                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_620,
                    message=(
                        f"{surface_label} surface #{i} origin/center ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}) "
                        f"lies outside the brep extent expanded by {margin:.4f} "
                        f"(extent: [{global_min[0]:.4f}, {global_min[1]:.4f}, {global_min[2]:.4f}] - "
                        f"[{global_max[0]:.4f}, {global_max[1]:.4f}, {global_max[2]:.4f}])."
                    ),
                    at=brep_array,
                )

    def _validate_edge_angular_range_primary_period(self, brep_array: Usd.Prim) -> None:
        """
        BA_630: Circle/ellipse edge range max values should be in (0, 2*pi].
        """
        TWO_PI = 2.0 * math.pi
        PERIOD_TOL = 1e-6

        # BA_630: Edge angular range max
        curve_types = BrepConstants.safe_get_attribute(brep_array, "edge:curveType")
        edge_ranges_val = brep_array.GetAttribute("edge:range").Get()
        if not self._is_unregistered_value(edge_ranges_val):
            edge_ranges = edge_ranges_val or []
            if curve_types and edge_ranges:
                num_edges = len(curve_types)
                if len(edge_ranges) >= num_edges * 2:
                    for edge_idx in range(num_edges):
                        ctype = str(curve_types[edge_idx])
                        if ctype not in ("BrepCurve3dCircleAPI", "BrepCurve3dEllipseAPI"):
                            continue
                        try:
                            param_max = float(edge_ranges[2 * edge_idx + 1])
                        except (TypeError, ValueError):
                            # BA_235 reports malformed ranges; keep checking later edges.
                            continue
                        if param_max < -PERIOD_TOL or param_max > TWO_PI + PERIOD_TOL:
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_630,
                                message=(
                                    f"{ctype.replace('BrepCurve3d', '').replace('API', '')} edge #{edge_idx} range max = "
                                    f"{param_max:.6f} is outside the primary period (0, 2*pi] = (0, {TWO_PI:.6f}]."
                                ),
                                at=brep_array,
                            )

    def _validate_face_v_domain_ordering(self, brep_array: Usd.Prim) -> None:
        """
        BA_640: For cylinder and cone faces, face:range V-min must be <= V-max.
        """
        face_surface_types = BrepConstants.safe_get_attribute(brep_array, "face:surfaceType")
        face_ranges_val = brep_array.GetAttribute("face:range").Get()
        if self._is_unregistered_value(face_ranges_val):
            return
        face_ranges = face_ranges_val or []
        if not face_surface_types or not face_ranges:
            return

        v_ordered_types = {"BrepSurfaceCylinderAPI", "BrepSurfaceConeAPI"}
        num_faces = len(face_surface_types)
        if len(face_ranges) < num_faces * 2:
            return

        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        brep_offsets = self._compute_brep_offsets(brep_array)
        face_offsets = brep_offsets.get("faces", [])

        for face_idx in range(num_faces):
            stype = str(face_surface_types[face_idx])
            if stype not in v_ordered_types:
                continue

            uv_min = face_ranges[2 * face_idx]
            uv_max = face_ranges[2 * face_idx + 1]
            v_min = float(uv_min[1])
            v_max = float(uv_max[1])

            if v_min > v_max:
                brep_idx = 0
                local_face_idx = face_idx
                for bi, (start, end) in enumerate(zip(face_offsets[:-1], face_offsets[1:])):
                    if start <= face_idx < end:
                        brep_idx = bi
                        local_face_idx = face_idx - start
                        break
                brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_640,
                    message=(
                        f"{stype.replace('BrepSurface', '').replace('API', '')} face #{local_face_idx} in brep #{brep_id} "
                        f"has V-min ({v_min:.6f}) > V-max ({v_max:.6f}). "
                        f"V-domain must be ordered (V-min <= V-max)."
                    ),
                    at=brep_array,
                )

    ### WireEdge NURBS Validation (BA_650-BA_658) ###

    def _validate_wireEdge3d_nurbs(self, brep_array: Usd.Prim) -> None:
        """
        BA_650-BA_658: Validate wireEdge 3D NURBS geometry data (parallel to edge NURBS checks).
        """
        wire_curve_types = BrepConstants.safe_get_attribute(brep_array, "wireEdge:curveType")
        if not wire_curve_types:
            return

        nurbs_count = sum(1 for ct in wire_curve_types if str(ct) == "BrepCurve3dNurbAPI")
        if nurbs_count == 0:
            return

        order_vals = BrepConstants.safe_get_attribute(brep_array, "brep:wireEdge3dNurb:curve3d:nurb:order")
        vc_vals = BrepConstants.safe_get_attribute(brep_array, "brep:wireEdge3dNurb:curve3d:nurb:vertexCount")
        cv_vals = BrepConstants.safe_get_attribute(brep_array, "brep:wireEdge3dNurb:curve3d:nurb:controlVertices")
        wt_vals = BrepConstants.safe_get_attribute(brep_array, "brep:wireEdge3dNurb:curve3d:nurb:weights")
        kn_vals = BrepConstants.safe_get_attribute(brep_array, "brep:wireEdge3dNurb:curve3d:nurb:knots")

        has_order = len(order_vals) > 0
        has_vc = len(vc_vals) > 0
        has_cv = len(cv_vals) > 0
        has_wt = len(wt_vals) > 0
        has_kn = len(kn_vals) > 0

        # BA_658: data completeness
        if not all([has_order, has_vc, has_cv, has_wt, has_kn]):
            present = [n for n, v in [("order", has_order), ("vertexCount", has_vc),
                        ("controlVertices", has_cv), ("weights", has_wt), ("knots", has_kn)] if v]
            missing = [n for n, v in [("order", has_order), ("vertexCount", has_vc),
                        ("controlVertices", has_cv), ("weights", has_wt), ("knots", has_kn)] if not v]
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_658,
                message=f"WireEdge NURBS data is incomplete. Present: {present}. Missing: {missing}.",
                at=brep_array,
            )
            return

        # BA_650: order and vertexCount size must match nurbs_count
        if len(order_vals) != nurbs_count:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_650,
                message=f"wireEdge NURBS order size ({len(order_vals)}) != BrepCurve3dNurbAPI count ({nurbs_count}).",
                at=brep_array,
            )
        if len(vc_vals) != nurbs_count:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_650,
                message=f"wireEdge NURBS vertexCount size ({len(vc_vals)}) != BrepCurve3dNurbAPI count ({nurbs_count}).",
                at=brep_array,
            )

        min_len = min(len(order_vals), len(vc_vals))
        expected_cv_count = 0
        expected_knot_count = 0
        for i in range(min_len):
            o = int(order_vals[i])
            vc = int(vc_vals[i])

            # BA_651: order must be >= 2
            if o < 2:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_651,
                    message=f"wireEdge NURBS order[{i}] = {o} is less than 2.",
                    at=brep_array,
                )

            # BA_652: order must not exceed vertexCount
            if o > vc:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_652,
                    message=f"wireEdge NURBS order[{i}] ({o}) exceeds vertexCount[{i}] ({vc}).",
                    at=brep_array,
                )

            expected_cv_count += vc
            expected_knot_count += vc + o

        # BA_653: controlVertices and weights sizes
        if len(cv_vals) != expected_cv_count:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_653,
                message=f"wireEdge NURBS controlVertices size ({len(cv_vals)}) != sum of vertexCounts ({expected_cv_count}).",
                at=brep_array,
            )
        if len(wt_vals) != expected_cv_count:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_653,
                message=f"wireEdge NURBS weights size ({len(wt_vals)}) != sum of vertexCounts ({expected_cv_count}).",
                at=brep_array,
            )

        # BA_654: weights must be positive
        for i, w in enumerate(wt_vals):
            if float(w) <= 0.0:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_654,
                    message=f"wireEdge NURBS weights[{i}] = {w} is not positive.",
                    at=brep_array,
                )
                break

        # BA_655: knots size
        if len(kn_vals) != expected_knot_count:
            self._AddFailedCheck(
                requirement=cap.BrepArrayRequirements.BA_655,
                message=f"wireEdge NURBS knots size ({len(kn_vals)}) != expected ({expected_knot_count}).",
                at=brep_array,
            )

        # BA_656: knots must be non-decreasing per knot vector
        knot_offset = 0
        for i in range(min_len):
            o = int(order_vals[i])
            vc = int(vc_vals[i])
            knot_len = vc + o
            if knot_offset + knot_len > len(kn_vals):
                break
            for j in range(1, knot_len):
                if float(kn_vals[knot_offset + j]) < float(kn_vals[knot_offset + j - 1]):
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_656,
                        message=f"wireEdge NURBS knot vector #{i} is not non-decreasing at position {j}.",
                        at=brep_array,
                    )
                    break
            knot_offset += knot_len

        # BA_657: control point containment within brep extent
        brep_extent = BrepConstants.safe_get_attribute(brep_array, "brep:extent")
        if brep_extent and len(brep_extent) >= 2 and cv_vals:
            brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
            actual_extents = len(brep_extent) // 2
            num_breps = len(brep_region_counts) if brep_region_counts else actual_extents
            num_breps = min(num_breps, actual_extents)
            for cv_idx, cv in enumerate(cv_vals):
                pt = [float(cv[0]), float(cv[1]), float(cv[2])]
                for brep_idx in range(num_breps):
                    ext_min = brep_extent[2 * brep_idx]
                    ext_max = brep_extent[2 * brep_idx + 1]
                    if len(ext_min) < 3 or len(ext_max) < 3:
                        continue
                    if (BrepConstants.isFloatLessThan(pt[0], float(ext_min[0])) or
                            BrepConstants.isFloatGreaterThan(pt[0], float(ext_max[0])) or
                            BrepConstants.isFloatLessThan(pt[1], float(ext_min[1])) or
                            BrepConstants.isFloatGreaterThan(pt[1], float(ext_max[1])) or
                            BrepConstants.isFloatLessThan(pt[2], float(ext_min[2])) or
                            BrepConstants.isFloatGreaterThan(pt[2], float(ext_max[2]))):
                        continue
                    break
                else:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_657,
                        message=f"wireEdge NURBS controlVertices[{cv_idx}] = {pt} is outside all brep extents.",
                        at=brep_array,
                    )
                    break

    ### NaN / Inf Sanity Check (BA_660) ###

    def _validate_float_arrays_finite(self, brep_array: Usd.Prim) -> None:
        """
        BA_660: All floating-point attribute arrays must contain only finite values.
        """
        float_attrs = [
            "brep:intersectTol3d", "brep:extent", "face:range", "edge:range", "wireEdge:range",
            "brep:edge3dNurb:curve3d:nurb:controlVertices",
            "brep:edge3dNurb:curve3d:nurb:knots",
            "brep:edge3dNurb:curve3d:nurb:weights",
            "brep:wireEdge3dNurb:curve3d:nurb:controlVertices",
            "brep:wireEdge3dNurb:curve3d:nurb:knots",
            "brep:wireEdge3dNurb:curve3d:nurb:weights",
            "brep:curveUv:nurb:controlVertices",
            "brep:curveUv:nurb:knots",
            "brep:curveUv:nurb:weights",
            "brep:surface:nurb:controlVertices",
            "brep:surface:nurb:uKnots",
            "brep:surface:nurb:vKnots",
            "brep:surface:nurb:weights",
            "brep:surface:sphere:center", "brep:surface:sphere:axis",
            "brep:surface:sphere:refDirection", "brep:surface:sphere:radius",
            "brep:surface:plane:origin", "brep:surface:plane:axis",
            "brep:surface:plane:refDirection",
            "brep:surface:cylinder:origin", "brep:surface:cylinder:axis",
            "brep:surface:cylinder:refDirection", "brep:surface:cylinder:radius",
            "brep:surface:cone:origin", "brep:surface:cone:axis",
            "brep:surface:cone:refDirection", "brep:surface:cone:radius",
            "brep:surface:cone:semiAngle",
            "brep:surface:torus:origin", "brep:surface:torus:axis",
            "brep:surface:torus:refDirection",
            "brep:surface:torus:majorRadius", "brep:surface:torus:minorRadius",
            "brep:vertexPoint:point:position",
            "brep:shellPoint:point:position",
            "brep:edge3dCircle:curve3d:circle:center",
            "brep:edge3dCircle:curve3d:circle:axis",
            "brep:edge3dCircle:curve3d:circle:refDirection",
            "brep:edge3dCircle:curve3d:circle:radius",
            "brep:edge3dLine:curve3d:line:origin",
            "brep:edge3dLine:curve3d:line:direction",
            "brep:edge3dEllipse:curve3d:ellipse:center",
            "brep:edge3dEllipse:curve3d:ellipse:axis",
            "brep:edge3dEllipse:curve3d:ellipse:refDirection",
            "brep:edge3dEllipse:curve3d:ellipse:xRadius",
            "brep:edge3dEllipse:curve3d:ellipse:yRadius",
        ]

        for attr_name in float_attrs:
            attr = brep_array.GetAttribute(attr_name)
            if not attr or not attr.IsAuthored():
                continue
            values = attr.Get()
            if values is None or self._is_unregistered_value(values):
                continue
            try:
                for i, v in enumerate(values):
                    floats = [float(v)] if not hasattr(v, '__len__') else [float(c) for c in v]
                    for f in floats:
                        if math.isnan(f) or math.isinf(f):
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_660,
                                message=f"{attr_name}[{i}] contains NaN or Inf value.",
                                at=brep_array,
                            )
                            return
            except (TypeError, ValueError):
                continue

    ### Radial Edgeuse Chain Consistency (BA_670) ###

    @staticmethod
    def _decompose_radial_cycles(
        next_radial: list,
        eu_start: int,
        eu_end: int,
    ) -> list[list[int]]:
        """
        Decompose edgeuse:nextRadialEUIndex into disjoint circular chains within
        [eu_start, eu_end). Each edgeuse index appears in exactly one returned cycle.
        """
        assigned: set[int] = set()
        cycles: list[list[int]] = []

        total = len(next_radial)
        safe_end = min(eu_end, total)

        for start_eu in range(eu_start, safe_end):
            if start_eu in assigned:
                continue

            cycle: list[int] = []
            current = start_eu
            while current not in assigned:
                # Defensive bounds check before indexing: out-of-range pointers
                # are reported as BA_670 failures by the caller, so simply stop
                # following the chain here instead of raising an IndexError.
                if current < 0 or current >= total:
                    break
                cycle.append(current)
                assigned.add(current)
                # Non-numeric entries are likewise treated as validation failures
                # upstream; break rather than let the walk abort with an exception.
                try:
                    current = int(next_radial[current])
                except (TypeError, ValueError):
                    break

            cycles.append(cycle)

        return cycles

    def _validate_radial_chain_consistency(self, brep_array: Usd.Prim) -> None:
        """
        BA_670: edgeuse:nextRadialEUIndex forms closed circular chains and edgeuse:edgeIndex
        is constant on each chain; conversely, all edgeuses of an edge share one chain.
        """
        next_radial = BrepConstants.safe_get_attribute(brep_array, "edgeuse:nextRadialEUIndex")
        edge_indices = BrepConstants.safe_get_attribute(brep_array, "edgeuse:edgeIndex")
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not next_radial or not edge_indices or not brep_region_counts:
            return

        total_eu = len(next_radial)
        if len(edge_indices) != total_eu:
            return

        brep_offsets = self._compute_brep_offsets(brep_array)
        edgeuse_offsets = brep_offsets.get("edgeuses", [])

        for brep_idx in range(len(brep_region_counts)):
            if brep_idx >= len(edgeuse_offsets) - 1:
                break

            eu_start = edgeuse_offsets[brep_idx]
            eu_end = edgeuse_offsets[brep_idx + 1]
            brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

            if eu_start >= eu_end:
                continue

            for eu in range(eu_start, min(eu_end, total_eu)):
                try:
                    nxt = int(next_radial[eu])
                except (TypeError, ValueError):
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_670,
                        message=(
                            f"edgeuse:nextRadialEUIndex at edgeuse #{eu} in brep #{brep_id} "
                            f"is not a numeric index: {next_radial[eu]!r}."
                        ),
                        at=brep_array,
                    )
                    return
                if nxt < eu_start or nxt >= eu_end:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_670,
                        message=(
                            f"edgeuse:nextRadialEUIndex at edgeuse #{eu} in brep #{brep_id} "
                            f"references edgeuse #{nxt}, which is outside this brep's edgeuse "
                            f"range [{eu_start}, {eu_end})."
                        ),
                        at=brep_array,
                    )
                    return

            cycles = self._decompose_radial_cycles(next_radial, eu_start, min(eu_end, total_eu))
            eu_cycle_id: dict[int, int] = {}
            for cycle_id, cycle in enumerate(cycles):
                for eu in cycle:
                    eu_cycle_id[eu] = cycle_id

            for cycle_id, cycle in enumerate(cycles):
                cycle_edges = {int(edge_indices[eu]) for eu in cycle}
                if len(cycle_edges) != 1:
                    edge_list = sorted(cycle_edges)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_670,
                        message=(
                            f"Radial chain {cycle} in brep #{brep_id} references multiple edges "
                            f"via edgeuse:edgeIndex ({edge_list}); all edgeuses in a radial chain "
                            f"must share the same edge."
                        ),
                        at=brep_array,
                    )
                    return

            edge_to_cycle_ids: dict[int, set[int]] = {}
            for eu in range(eu_start, min(eu_end, total_eu)):
                edge_id = int(edge_indices[eu])
                edge_to_cycle_ids.setdefault(edge_id, set()).add(eu_cycle_id[eu])

            for edge_id, cycle_ids in edge_to_cycle_ids.items():
                if len(cycle_ids) <= 1:
                    continue

                edgeuses_for_edge = [
                    eu for eu in range(eu_start, min(eu_end, total_eu)) if int(edge_indices[eu]) == edge_id
                ]
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_670,
                    message=(
                        f"Edge #{edge_id} in brep #{brep_id} is referenced by edgeuses "
                        f"{edgeuses_for_edge}, but they fall into {len(cycle_ids)} separate radial "
                        f"chains (edgeuse:nextRadialEUIndex). All edgeuses of an edge must belong "
                        f"to one closed radial chain."
                    ),
                    at=brep_array,
                )
                return

    ### GeomSubset Material Binding Validation (BA_680-BA_682) ###

    def _validate_geomsubset_materials(self, brep_array: Usd.Prim) -> None:
        """
        BA_680: GeomSubset indices must be in valid range.
        BA_681: GeomSubset indices must be non-overlapping within an elementType.
        BA_682: GeomSubset material:binding targets must exist.
        """
        region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount")
        surface_types = BrepConstants.safe_get_attribute(brep_array, "face:surfaceType")
        num_breps = len(region_counts) if region_counts else 0
        num_faces = len(surface_types) if surface_types else 0
        if num_breps == 0:
            brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
            num_breps = len(brep_region_counts) if brep_region_counts else 0
        if num_faces == 0:
            face_loop_counts = BrepConstants.safe_get_attribute(brep_array, "face:loopCount") or []
            num_faces = len(face_loop_counts) if face_loop_counts else 0

        stage = brep_array.GetStage()
        brep_seen_indices = {}
        face_seen_indices = {}

        for child in brep_array.GetAllChildren():
            if child.GetTypeName() != "GeomSubset":
                continue

            element_type_attr = child.GetAttribute("elementType")
            if not element_type_attr:
                continue
            element_type = str(element_type_attr.Get())
            if element_type not in ("brep", "face"):
                continue

            indices_attr = child.GetAttribute("indices")
            if not indices_attr:
                continue
            indices = indices_attr.Get()
            if indices is None:
                continue

            upper_bound = num_breps if element_type == "brep" else num_faces
            seen_map = brep_seen_indices if element_type == "brep" else face_seen_indices
            child_name = child.GetName()

            # BA_680: valid range
            for idx_val in indices:
                idx = int(idx_val)
                if upper_bound > 0 and (idx < 0 or idx >= upper_bound):
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_680,
                        message=(
                            f"GeomSubset '{child_name}' has {element_type} index {idx} "
                            f"outside valid range [0, {upper_bound})."
                        ),
                        at=brep_array,
                    )
                    break

            # BA_681: non-overlapping
            for idx_val in indices:
                idx = int(idx_val)
                if idx in seen_map:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_681,
                        message=(
                            f"GeomSubset '{child_name}': {element_type} index {idx} "
                            f"also appears in subset '{seen_map[idx]}'."
                        ),
                        at=brep_array,
                    )
                    break
                seen_map[idx] = child_name

            # BA_682: material target exists
            if stage:
                mat_rel = child.GetRelationship("material:binding")
                if mat_rel:
                    targets = mat_rel.GetTargets()
                    for target_path in targets:
                        if not stage.GetPrimAtPath(target_path):
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_682,
                                message=(
                                    f"GeomSubset '{child_name}' material:binding target "
                                    f"'{target_path}' does not exist on stage."
                                ),
                                at=brep_array,
                            )

    ### Minimum Topology Counts (BA_700-BA_702) ###

    def _validate_minimum_topology_counts(self, brep_array: Usd.Prim) -> None:
        """
        BA_700: Every brep:regionCount entry >= 1.
        BA_701: Every region:shellCount entry >= 1.
        BA_702: Every shell must have faceuseCount > 0, wireEdgeCount > 0, or pointType == 'BrepPointAPI'.
        """
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []
        region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount")
        shell_counts = BrepConstants.safe_get_attribute(brep_array, "region:shellCount")
        faceuse_counts = BrepConstants.safe_get_attribute(brep_array, "shell:faceuseCount")
        wireedge_counts = BrepConstants.safe_get_attribute(brep_array, "shell:wireEdgeCount")
        shell_point_types = BrepConstants.safe_get_attribute(brep_array, "shell:pointType")

        # BA_700
        if region_counts:
            for brep_idx, rc in enumerate(region_counts):
                if int(rc) < 1:
                    brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_700,
                        message=f"brep:regionCount[{brep_id}] = {rc} is less than 1.",
                        at=brep_array,
                    )

        # BA_701
        if shell_counts:
            for region_idx, sc in enumerate(shell_counts):
                if int(sc) < 1:
                    self._AddFailedCheck(
                        requirement=cap.BrepArrayRequirements.BA_701,
                        message=f"region:shellCount[{region_idx}] = {sc} is less than 1.",
                        at=brep_array,
                    )

        # BA_702
        if faceuse_counts and wireedge_counts:
            num_shells = min(len(faceuse_counts), len(wireedge_counts))
            for shell_idx in range(num_shells):
                try:
                    fu = int(faceuse_counts[shell_idx])
                    we = int(wireedge_counts[shell_idx])
                except (ValueError, TypeError):
                    continue  # bad data type already reported by BA_091
                pt = str(shell_point_types[shell_idx]) if shell_point_types and shell_idx < len(shell_point_types) else "none"
                if fu == 0 and we == 0 and pt != "BrepPointAPI":
                    self._AddWarning(
                        requirement=cap.BrepArrayRequirements.BA_702,
                        message=(
                            f"shell #{shell_idx} has no content: faceuseCount=0, "
                            f"wireEdgeCount=0, pointType='{pt}'."
                        ),
                        at=brep_array,
                    )

    ### Shell Point Position Containment (BA_710) ###

    def _validate_shell_point_containment(self, brep_array: Usd.Prim) -> None:
        """
        BA_710: brep:shellPoint:point:position values must lie within their brep extent.
        """
        shell_positions = BrepConstants.safe_get_attribute(brep_array, "brep:shellPoint:point:position")
        brep_extent = BrepConstants.safe_get_attribute(brep_array, "brep:extent")
        shell_point_types = BrepConstants.safe_get_attribute(brep_array, "shell:pointType")
        shell_faceuse_counts = BrepConstants.safe_get_attribute(brep_array, "shell:faceuseCount")
        shell_wireedge_counts = BrepConstants.safe_get_attribute(brep_array, "shell:wireEdgeCount")
        brep_region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount") or []

        if not shell_positions or not brep_extent or len(brep_extent) < 2:
            return

        region_counts = BrepConstants.safe_get_attribute(brep_array, "brep:regionCount")
        shell_counts = BrepConstants.safe_get_attribute(brep_array, "region:shellCount")
        if not region_counts or not shell_counts:
            return

        num_breps = len(region_counts)
        shell_offset = 0
        sp_idx = 0
        region_offset = 0

        for brep_idx in range(num_breps):
            n_regions = int(region_counts[brep_idx])
            brep_shell_start = shell_offset
            for r in range(n_regions):
                ri = region_offset + r
                if ri < len(shell_counts):
                    shell_offset += int(shell_counts[ri])
            region_offset += n_regions
            brep_shell_end = shell_offset

            if 2 * brep_idx + 1 >= len(brep_extent):
                continue
            ext_min = brep_extent[2 * brep_idx]
            ext_max = brep_extent[2 * brep_idx + 1]
            if ext_min is None or ext_max is None or len(ext_min) < 3 or len(ext_max) < 3:
                continue
            brep_id = BrepConstants.get_brep_id(brep_region_counts, brep_idx)

            for si in range(brep_shell_start, brep_shell_end):
                if BrepConstants.is_brep_point_shell(
                    si,
                    shell_point_types,
                    shell_faceuse_counts,
                    shell_wireedge_counts,
                ):
                    if sp_idx < len(shell_positions):
                        pos = shell_positions[sp_idx]
                        try:
                            if len(pos) < 3:
                                sp_idx += 1
                                continue
                            pt = [float(pos[0]), float(pos[1]), float(pos[2])]
                        except (TypeError, ValueError):
                            sp_idx += 1
                            continue
                        if (BrepConstants.isFloatLessThan(pt[0], float(ext_min[0])) or
                                BrepConstants.isFloatGreaterThan(pt[0], float(ext_max[0])) or
                                BrepConstants.isFloatLessThan(pt[1], float(ext_min[1])) or
                                BrepConstants.isFloatGreaterThan(pt[1], float(ext_max[1])) or
                                BrepConstants.isFloatLessThan(pt[2], float(ext_min[2])) or
                                BrepConstants.isFloatGreaterThan(pt[2], float(ext_max[2]))):
                            self._AddFailedCheck(
                                requirement=cap.BrepArrayRequirements.BA_710,
                                message=(
                                    f"shellPoint:position[{sp_idx}] = {pt} in brep #{brep_id} "
                                    f"is outside brep extent."
                                ),
                                at=brep_array,
                            )
                        sp_idx += 1

    ### Curve/Surface Type Count Totals (BA_720-BA_722) ###

    def _validate_type_count_exhaustive(self, brep_array: Usd.Prim) -> None:
        """
        BA_720: Edge curveType category sum must equal total edge count.
        BA_721: WireEdge curveType category sum must equal total wireEdge count.
        BA_722: Face surfaceType category sum must equal total face count.
        """
        valid_curve_types = {"BrepCurve3dNurbAPI", "BrepCurve3dCircleAPI", "BrepCurve3dLineAPI", "BrepCurve3dEllipseAPI"}
        valid_surface_types = {"BrepSurfaceNurbAPI", "BrepSurfaceSphereAPI", "BrepSurfacePlaneAPI",
                               "BrepSurfaceCylinderAPI", "BrepSurfaceConeAPI", "BrepSurfaceTorusAPI"}

        # BA_720: edges
        edge_types = BrepConstants.safe_get_attribute(brep_array, "edge:curveType")
        if edge_types:
            recognized = sum(1 for ct in edge_types if str(ct) in valid_curve_types)
            if recognized != len(edge_types):
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_720,
                    message=(
                        f"Edge curveType recognized count ({recognized}) != "
                        f"total edge count ({len(edge_types)}). "
                        f"{len(edge_types) - recognized} edges have unrecognized types."
                    ),
                    at=brep_array,
                )

        # BA_721: wireEdges
        wire_types = BrepConstants.safe_get_attribute(brep_array, "wireEdge:curveType")
        if wire_types:
            recognized = sum(1 for ct in wire_types if str(ct) in valid_curve_types)
            if recognized != len(wire_types):
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_721,
                    message=(
                        f"WireEdge curveType recognized count ({recognized}) != "
                        f"total wireEdge count ({len(wire_types)}). "
                        f"{len(wire_types) - recognized} wireEdges have unrecognized types."
                    ),
                    at=brep_array,
                )

        # BA_722: faces
        face_types = BrepConstants.safe_get_attribute(brep_array, "face:surfaceType")
        if face_types:
            recognized = sum(1 for st in face_types if str(st) in valid_surface_types)
            if recognized != len(face_types):
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_722,
                    message=(
                        f"Face surfaceType recognized count ({recognized}) != "
                        f"total face count ({len(face_types)}). "
                        f"{len(face_types) - recognized} faces have unrecognized types."
                    ),
                    at=brep_array,
                )

    @staticmethod
    def _de_boor_evaluate_2d(order, knots, cvs, weights, t):
        """Evaluate a rational 2D B-spline curve at parameter t using de Boor's algorithm."""
        n = len(cvs)
        p = order - 1
        if order < 1 or n < order or len(knots) < n + order:
            return None

        t = max(knots[p], min(t, knots[n]))

        k = p
        for i in range(p, n):
            if knots[i] <= t < knots[i + 1]:
                k = i
                break
        else:
            if math.isclose(t, knots[n], rel_tol=1e-12, abs_tol=1e-14):
                k = n - 1

        d = []
        for j in range(p + 1):
            idx = k - p + j
            if idx < 0 or idx >= n:
                return None
            w = weights[idx]
            d.append([cvs[idx][0] * w, cvs[idx][1] * w, w])

        for r in range(1, p + 1):
            for j in range(p, r - 1, -1):
                left = k - p + j
                right = left + p - r + 1
                if right >= len(knots) or left >= len(knots):
                    return None
                denom = knots[right] - knots[left]
                if abs(denom) < 1e-30:
                    alpha = 0.0
                else:
                    alpha = (t - knots[left]) / denom
                for c in range(3):
                    d[j][c] = (1.0 - alpha) * d[j - 1][c] + alpha * d[j][c]

        w = d[p][2]
        if abs(w) < 1e-30:
            return None
        return [d[p][0] / w, d[p][1] / w]

    ### UV Loop Closure (BA_763) ###

    def _validate_uv_loop_closure(self, brep_array: Usd.Prim) -> None:
        """
        BA_763: When UV trim NURBS are authored, adjacent edgeuse UV
        pcurves in each loop must meet head-to-tail in parameter space.
        """
        UV_CLOSURE_TOL = 1e-6

        order_vals = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:order")
        vc_vals = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:vertexCount")
        cv_vals = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:controlVertices")
        wt_vals = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:weights")
        kn_vals = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:knots")
        face_loop_counts = BrepConstants.safe_get_attribute(brep_array, "face:loopCount")
        loop_edgeuse_counts = BrepConstants.safe_get_attribute(brep_array, "loop:edgeuseCount")
        edgeuse_edge_indices = BrepConstants.safe_get_attribute(brep_array, "edgeuse:edgeIndex")

        if (not order_vals or not vc_vals or not cv_vals or not wt_vals or not kn_vals
                or not face_loop_counts or not loop_edgeuse_counts or not edgeuse_edge_indices):
            return

        edgeuse_count = len(edgeuse_edge_indices)
        if len(order_vals) < edgeuse_count or len(vc_vals) < edgeuse_count:
            return

        curve_endpoints = []
        cv_offset = 0
        knot_offset = 0
        for curve_idx in range(edgeuse_count):
            try:
                order = int(order_vals[curve_idx])
                n_cv = int(vc_vals[curve_idx])
            except (TypeError, ValueError):
                return

            if order == 0 and n_cv == 0:
                curve_endpoints.append(None)
                continue
            if order < 1 or n_cv < order:
                return

            n_knots = n_cv + order
            if cv_offset + n_cv > len(cv_vals) or cv_offset + n_cv > len(wt_vals) or knot_offset + n_knots > len(kn_vals):
                return

            knots = [float(kn_vals[knot_offset + k]) for k in range(n_knots)]
            cvs = [[float(cv_vals[cv_offset + j][0]), float(cv_vals[cv_offset + j][1])] for j in range(n_cv)]
            weights = [float(wt_vals[cv_offset + j]) for j in range(n_cv)]
            t_start = knots[order - 1]
            t_end = knots[n_cv]
            uv_start = self._de_boor_evaluate_2d(order, knots, cvs, weights, t_start)
            uv_end = self._de_boor_evaluate_2d(order, knots, cvs, weights, t_end)
            if uv_start is None or uv_end is None:
                return

            curve_endpoints.append((uv_start, uv_end))
            cv_offset += n_cv
            knot_offset += n_knots

        loop_idx = 0
        edgeuse_offset = 0
        for face_idx, face_loop_count in enumerate(face_loop_counts):
            try:
                n_loops = int(face_loop_count)
            except (TypeError, ValueError):
                return

            for local_loop_idx in range(n_loops):
                if loop_idx >= len(loop_edgeuse_counts):
                    return
                try:
                    n_edgeuses = int(loop_edgeuse_counts[loop_idx])
                except (TypeError, ValueError):
                    return

                loop_start = edgeuse_offset
                loop_end = edgeuse_offset + n_edgeuses
                loop_idx += 1
                edgeuse_offset = loop_end

                if n_edgeuses <= 0:
                    continue
                if loop_end > len(curve_endpoints):
                    return

                endpoints = curve_endpoints[loop_start:loop_end]
                if any(endpoint is None for endpoint in endpoints):
                    continue

                for local_edgeuse_idx in range(n_edgeuses):
                    edgeuse_idx = loop_start + local_edgeuse_idx
                    next_local_edgeuse_idx = (local_edgeuse_idx + 1) % n_edgeuses
                    next_edgeuse_idx = loop_start + next_local_edgeuse_idx
                    uv_end = endpoints[local_edgeuse_idx][1]
                    next_uv_start = endpoints[next_local_edgeuse_idx][0]
                    dist = math.sqrt(
                        (uv_end[0] - next_uv_start[0]) ** 2
                        + (uv_end[1] - next_uv_start[1]) ** 2
                    )
                    if dist > UV_CLOSURE_TOL:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_763,
                            message=(
                                f"Face #{face_idx} loop #{local_loop_idx} edgeuse #{edgeuse_idx} UV endpoint "
                                f"({uv_end[0]:.6f}, {uv_end[1]:.6f}) does not meet next edgeuse "
                                f"#{next_edgeuse_idx} UV start ({next_uv_start[0]:.6f}, {next_uv_start[1]:.6f}); "
                                f"gap {dist:.6f} exceeds tolerance {UV_CLOSURE_TOL}."
                            ),
                            at=brep_array,
                        )

    ### Zero-Length UV Trim Curve Check (BA_764) ###

    def _validate_zero_length_uv_trim_curves(self, brep_array: Usd.Prim) -> None:
        """
        BA_764: Authored UV trim NURBS should have nonzero extent in
        parameter space.

        This schema-side check uses the control polygon extent as a practical
        zero-length heuristic. Malformed UV NURBS array sizing is handled by
        the existing BA_375-BA_416 checks.
        """
        UV_ZERO_LENGTH_TOL = 1e-12

        uv_orders = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:order")
        uv_vc = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:vertexCount")
        uv_cvs = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:controlVertices")

        if not uv_orders or not uv_vc or not uv_cvs:
            return

        cv_offset = 0
        for curve_idx, vertex_count in enumerate(uv_vc):
            if curve_idx >= len(uv_orders):
                return

            try:
                order = int(uv_orders[curve_idx])
                n_cv = int(vertex_count)
            except (TypeError, ValueError):
                return

            if order == 0 and n_cv == 0:
                continue
            if n_cv == 0:
                continue
            if n_cv < 0:
                continue
            if cv_offset + n_cv > len(uv_cvs):
                # Flat UV CV stream is truncated: stop scanning. Using continue here would leave
                # cv_offset unadvanced and let a later smaller vertexCount re-slice the same tail,
                # producing misaligned control vertices and bogus BA_764 hits.
                break

            cvs = uv_cvs[cv_offset : cv_offset + n_cv]
            cv_offset += n_cv

            if order <= 0:
                continue

            try:
                u_vals = [float(cv[0]) for cv in cvs]
                v_vals = [float(cv[1]) for cv in cvs]
            except (TypeError, ValueError, IndexError):
                continue

            u_extent = max(u_vals) - min(u_vals)
            v_extent = max(v_vals) - min(v_vals)
            diagonal = math.sqrt(u_extent ** 2 + v_extent ** 2)
            if diagonal <= UV_ZERO_LENGTH_TOL:
                self._AddFailedCheck(
                    requirement=cap.BrepArrayRequirements.BA_764,
                    message=(
                        f"UV trim curve #{curve_idx} has collapsed control vertices at "
                        f"({u_vals[0]:.6f}, {v_vals[0]:.6f}); control polygon extent "
                        f"{diagonal:.6e} is at or below tolerance {UV_ZERO_LENGTH_TOL:.1e}."
                    ),
                    at=brep_array,
                )

    ### UV Trim Curve Domain Containment (BA_750) ###

    def _validate_uv_trim_curve_domain_containment(self, brep_array: Usd.Prim) -> None:
        """
        BA_750: UV trim curve control vertices should lie within or near the face's UV range.
        """
        uv_vc = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:vertexCount")
        uv_cvs = BrepConstants.safe_get_attribute(brep_array, "brep:curveUv:nurb:controlVertices")
        face_ranges_val = brep_array.GetAttribute("face:range").Get()
        loop_counts = BrepConstants.safe_get_attribute(brep_array, "face:loopCount")
        eu_counts = BrepConstants.safe_get_attribute(brep_array, "loop:edgeuseCount")

        if (not uv_vc or not uv_cvs or face_ranges_val is None
                or self._is_unregistered_value(face_ranges_val)
                or not loop_counts or not eu_counts):
            return

        face_ranges = face_ranges_val
        num_faces = len(loop_counts)
        MARGIN = 0.5

        loop_offset = 0
        eu_offset = 0
        cv_offset = 0

        for face_idx in range(num_faces):
            if face_idx >= len(loop_counts) or 2 * face_idx + 1 >= len(face_ranges):
                break

            uv_min = face_ranges[2 * face_idx]
            uv_max = face_ranges[2 * face_idx + 1]
            u_min, v_min = float(uv_min[0]), float(uv_min[1])
            u_max, v_max = float(uv_max[0]), float(uv_max[1])
            u_span = u_max - u_min
            v_span = v_max - v_min
            u_lo = u_min - MARGIN * max(abs(u_span), 1.0)
            u_hi = u_max + MARGIN * max(abs(u_span), 1.0)
            v_lo = v_min - MARGIN * max(abs(v_span), 1.0)
            v_hi = v_max + MARGIN * max(abs(v_span), 1.0)

            n_loops = int(loop_counts[face_idx])
            face_eu_start = eu_offset
            for lp in range(n_loops):
                lp_idx = loop_offset + lp
                if lp_idx < len(eu_counts):
                    eu_offset += int(eu_counts[lp_idx])
            face_eu_end = eu_offset
            loop_offset += n_loops

            for eu_idx in range(face_eu_start, min(face_eu_end, len(uv_vc))):
                n_cv = int(uv_vc[eu_idx])
                for j in range(n_cv):
                    ci = cv_offset + j
                    if ci >= len(uv_cvs):
                        break
                    u_val = float(uv_cvs[ci][0])
                    v_val = float(uv_cvs[ci][1])
                    if u_val < u_lo or u_val > u_hi or v_val < v_lo or v_val > v_hi:
                        self._AddFailedCheck(
                            requirement=cap.BrepArrayRequirements.BA_750,
                            message=(
                                f"Face #{face_idx} edgeuse #{eu_idx} UV control vertex [{ci}] = "
                                f"({u_val:.6f}, {v_val:.6f}) is far outside face UV domain "
                                f"[{u_min:.4f}..{u_max:.4f}] x [{v_min:.4f}..{v_max:.4f}]."
                            ),
                            at=brep_array,
                        )
                        cv_offset += sum(int(uv_vc[k]) for k in range(eu_idx, min(face_eu_end, len(uv_vc))))
                        return
                cv_offset += n_cv

    ### Entry Point for Validation ###

    def CheckPrim(self, prim: Usd.Prim) -> None:
        """
        Entry point validation method for Brep objects. Executes all validations present in brep_validator.py.
        """
        if prim.GetTypeName() != "BrepArray":
            return

        self._clear_brep_offsets_cache()

        ### Compute Brep Offsets ###
        brep_offsets = self._compute_brep_offsets(prim)

        ### General Brep Attribute Validation ###
        # print("Validating Brep extent ranges...")
        self._validate_brep_extent(prim)
        self._validate_brep_tols(prim)

        # print("Validating Brep array attributes...")
        self._validate_brep_array(prim)

        ### Region Validation ###
        # print("Validating region attributes...")
        self._validate_region_arrays(prim)

        ### Shell Validation ###
        # print("Validating shell attributes...")
        self._validate_shell_arrays(prim)

        ### Face Validation ###
        # print("Validating face attributes...")
        self._validate_faceuse_arrays(prim, brep_offsets["faces"])
        self._validate_face_arrays(prim)
        self._validate_face_loop_count_minimum(prim)
        self._validate_faceuse_pairing(prim)
        self._validate_face_ranges(prim)

        ### Loop Validation ###
        # print("Validating loop attributes...")
        self._validate_loop_arrays(prim)
        self._validate_loop_vertex_index(prim, brep_offsets["loops"], brep_offsets["vertices"])

        ### Edge and Edgeuse Authorship Validation (Always Required) ###
        # BA_215: All edge schema attributes must be authored 
        edge_attributes = ["edge:curveType", "edge:vertexIndices", "edge:range"]
        self._validate_authorship_only(prim, edge_attributes, cap.BrepArrayRequirements.BA_215)
        
        # BA_185: All edgeuse schema attributes must be authored
        edgeuse_attributes = ["edgeuse:edgeIndex", "edgeuse:orientationType", "edgeuse:nextRadialEUIndex", "edgeuse:thisRadialEntryType"]
        self._validate_authorship_only(prim, edgeuse_attributes, cap.BrepArrayRequirements.BA_185)

        ### Edge Validation ###
        # print("Validating edge attributes...")
        # Always validate edge arrays - don't skip due to offset computation issues
        self._validate_edge_arrays(prim, brep_offsets, brep_offsets.get("edges", []), brep_offsets.get("vertices", []))
        
        # Only validate edgeuse relationships if there are edgeuses
        if brep_offsets["edgeuses"] and len(brep_offsets["edgeuses"]) > 1 and brep_offsets["edgeuses"][-1] > 0:
            self._validate_edgeuse_arrays(prim, brep_offsets["edges"], brep_offsets["edgeuses"])
        self._validate_wireEdge_arrays(prim, brep_offsets["vertices"], brep_offsets["wireedges"])
        self._validate_radial_edgeuse_closure(prim)
        self._validate_orphan_edges(prim)

        ### Vertex Validation ###
        # print("Validating vertex attributes...")
        self._validate_vertex_arrays(prim)
        self._validate_point_position(prim)

        ### Curve3D Validation ###
        # print("Validating BrepCurve3dNurbAPI...")
        self._validate_curve3d_nurb_control_vertices_weights(prim)
        self._validate_curve3d_nurb_order_vertex_count(prim)
        self._validate_curve3d_knots(prim)

        ### CurveUV Validation ###
        # print("Validating BrepCurveUvNurbAPI...")
        self._validate_curveUv_data(prim)

        ### Surface Validation ###
        # print("Validating BrepSurfaceNurbAPI...")
        self._validate_surface_control_vertices_weights(prim)
        self._validate_surface_orders_vertex_counts(prim)
        self._validate_surface_knots(prim)

        # print("Validating BrepSurfaceSphereAPI...")
        self._validate_surface_sphere_data(prim)

        # Analytic surface validation
        self._validate_surface_plane_data(prim)
        self._validate_surface_cylinder_data(prim)
        self._validate_surface_cone_data(prim)
        self._validate_surface_torus_data(prim)

        ### Analytic Curve3D Validation ###
        self._validate_curve3d_circle_data(prim)
        self._validate_curve3d_line_data(prim)
        self._validate_curve3d_ellipse_data(prim)

        ### Analytic Domain Range Validation ###
        self._validate_face_range_domain_limits(prim)
        self._validate_edge_range_domain_limits(prim)

        ### Additional Validations (Containment, Schema Consistency, NURBS) ###
        # print("Validating containment and schema consistency...")
        self._validate_vertex_position_containment(prim)
        self._validate_edge3d_nurbs_control_point_containment(prim)
        self._validate_surface_nurbs_control_point_containment(prim)
        self._validate_schema_consistency(prim)
        self._validate_nurbs_data_completeness(prim)
        self._validate_nurbs_mathematical_consistency(prim)
        self._validate_topology_geometry_correspondence(prim)
        
        ### Data Type Validation (BA_061, BA_076, BA_091, BA_116, BA_161, BA_176, BA_196, BA_237, BA_291, BA_316, BA_326, BA_327, BA_371, BA_416, BA_471) ###
        # print("Validating attribute data types...")
        self._validate_attribute_data_types(prim)

        ### NURBS Order/VertexCount Value Validation (BA_590, BA_591) ###
        self._validate_nurbs_order_and_vertex_count_values(prim)

        ### Analytic Curve Endpoint-Vertex Consistency (BA_600, BA_601, BA_602) ###
        self._validate_edge_curve_endpoint_vertex_consistency(prim)

        ### Circle Vertex-Radius Consistency (BA_610) ###
        self._validate_circle_vertex_radius_consistency(prim)

        ### Analytic Surface Origin Containment (BA_620) ###
        self._validate_analytic_surface_origin_containment(prim)

        ### Edge Angular Range Primary Period (BA_630) ###
        self._validate_edge_angular_range_primary_period(prim)

        ### Cylinder/Cone V-Domain Ordering (BA_640) ###
        self._validate_face_v_domain_ordering(prim)

        ### WireEdge NURBS Validation (BA_650-BA_658) ###
        self._validate_wireEdge3d_nurbs(prim)

        ### NaN / Inf Sanity Check (BA_660) ###
        self._validate_float_arrays_finite(prim)

        ### Radial Edgeuse Chain Consistency (BA_670) ###
        self._validate_radial_chain_consistency(prim)

        ### GeomSubset Material Binding Validation (BA_680-BA_682) ###
        self._validate_geomsubset_materials(prim)


        ### Minimum Topology Counts (BA_700-BA_702) ###
        self._validate_minimum_topology_counts(prim)

        ### Shell Point Position Containment (BA_710) ###
        self._validate_shell_point_containment(prim)

        ### Curve/Surface Type Count Totals (BA_720-BA_722) ###
        self._validate_type_count_exhaustive(prim)

        ### UV Loop Closure (BA_763) / Zero-Length UV Trim Curves (BA_764) ###
        self._validate_uv_loop_closure(prim)
        self._validate_zero_length_uv_trim_curves(prim)

        ### UV Trim Curve Domain Containment (BA_750) ###
        self._validate_uv_trim_curve_domain_containment(prim)
