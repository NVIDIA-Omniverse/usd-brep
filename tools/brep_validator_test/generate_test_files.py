#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Script to generate test files for each BrepArray requirement.
Each file will be based on CubeBrepArray.usda but modified to fail a specific requirement.

NOTE: The generated test files in TestFiles/brep_validator/ are currently out of
sync with this script. Some of those files were edited manually and should not be
regenerated without careful review.
"""

import os
import shutil
from pathlib import Path


def find_package_root(start: Path) -> Path:
    current = start.resolve()
    if current.is_file():
        current = current.parent

    for candidate in [current, *current.parents]:
        has_validator = (candidate / "brep_validator").is_dir() or (candidate / "tools" / "brep_validator").is_dir()
        has_fixtures = (candidate / "TestFiles" / "brep_validator").is_dir()
        if has_validator and has_fixtures:
            return candidate
    raise RuntimeError(f"Could not locate brep_validator package root from {start}")


def get_test_files_dir():
    """Get the path to the TestFiles directory."""
    return find_package_root(Path(__file__)) / "TestFiles" / "brep_validator"

def read_base_file():
    """Read the base CubeBrepArray.usda file."""
    base_file = get_test_files_dir() / "CubeBrepArray.usda"
    with open(base_file, 'r') as f:
        return f.readlines()

def write_test_file(filename, content, description):
    """Write a test file with header comment."""
    filepath = get_test_files_dir() / filename
    with open(filepath, 'w') as f:
        f.write(f"#usda 1.0\n")
        f.write(f"# Test file for {filename}\n")
        f.write(f"# {description}\n")
        f.write(f"# This file is designed to fail the specific requirement.\n")
        f.write(f"\n")
        # Skip the first #usda line from content
        for line in content[1:]:
            f.write(line)

def generate_ba_000_test(base_content):
    """BA_000: Inconsistent brep attribute sizes."""
    content = base_content.copy()
    for i, line in enumerate(content):
        # Make brep:regionCount have wrong size (add extra element)
        if 'uniform uint[] brep:regionCount = [2, 2]' in line:
            content[i] = '        uniform uint[] brep:regionCount = [2, 2, 2]\n'  # 3 instead of 2
            break
    return content

def generate_ba_005_test(base_content):
    """BA_005: Missing brep schema attributes."""
    content = base_content.copy()
    # Remove brep:regionCount line entirely (required attribute)
    content = [line for line in content if 'uniform uint[] brep:regionCount' not in line]
    return content

def generate_ba_010_test(base_content):
    """BA_010: Negative brep:intersectTol3d values."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:intersectTol3d = [0.00002, 0.00002]' in line:
            content[i] = '        uniform double[] brep:intersectTol3d = [-0.00001, 0.00002]\n'
            break
    return content

def generate_ba_020_test(base_content):
    """BA_020: Invalid brep:extent structure (wrong size)."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Remove one extent (should have 4 for 2 breps, make it 3)
            content[i] = '        uniform double3[] brep:extent = [(-0.00001, -0.00001, -0.00001), (1.00001, 1.00001, 1.00001), (1.99999, -0.00001, -0.00001)]\n'
            break
    return content

def generate_ba_025_test(base_content):
    """BA_025: Invalid brep:extent X order (Xmax < Xmin)."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Swap X min/max for first brep
            content[i] = '        uniform double3[] brep:extent = [(1.00001, -0.00001, -0.00001), (-0.00001, 1.00001, 1.00001), (1.99999, -0.00001, -0.00001), (3.00001, 1.00001, 1.00001)]\n'
            break
    return content

def generate_ba_030_test(base_content):
    """BA_030: Invalid brep:extent Y order (Ymax < Ymin)."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Swap Y min/max for first brep
            content[i] = '        uniform double3[] brep:extent = [(-0.00001, 1.00001, -0.00001), (1.00001, -0.00001, 1.00001), (1.99999, -0.00001, -0.00001), (3.00001, 1.00001, 1.00001)]\n'
            break
    return content

def generate_ba_035_test(base_content):
    """BA_035: Invalid brep:extent Z order (Zmax < Zmin)."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Swap Z min/max for first brep
            content[i] = '        uniform double3[] brep:extent = [(-0.00001, -0.00001, 1.00001), (1.00001, 1.00001, -0.00001), (1.99999, -0.00001, -0.00001), (3.00001, 1.00001, 1.00001)]\n'
            break
    return content


def generate_ba_075_test(base_content):
    """BA_075: Invalid region:type token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] region:type = ["voidRegion", "solidRegion", "voidRegion", "solidRegion"]' in line:
            content[i] = '        uniform token[] region:type = ["invalidRegion", "solidRegion", "voidRegion", "solidRegion"]\n'
            break
    return content

def generate_ba_090_test(base_content):
    """BA_090: Invalid shell:pointType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] shell:pointType = ["none", "none", "none", "none"]' in line:
            content[i] = '        uniform token[] shell:pointType = ["invalid", "none", "none", "none"]\n'
            break
    return content

def generate_ba_110_test(base_content):
    """BA_110: Invalid faceuse:orientationType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] faceuse:orientationType = [' in line:
            # Replace first token with invalid value
            content[i] = content[i].replace('"same"', '"invalid"', 1)
            break
    return content

def generate_ba_115_test(base_content):
    """BA_115: Invalid faceuse:faceIndex value (out of range)."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] faceuse:faceIndex = [' in line:
            # Replace first index with out-of-range value
            content[i] = content[i].replace('[5,', '[99,', 1)
            break
    return content

def generate_ba_130_test(base_content):
    """BA_130: Invalid face:surfaceType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] face:surfaceType = [' in line:
            content[i] = content[i].replace('"BrepSurfaceNurbAPI"', '"InvalidSurface"', 1)
            break
    return content

def generate_ba_135_test(base_content):
    """BA_135: Invalid face:trimType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] face:trimType = [' in line:
            content[i] = content[i].replace('"general"', '"invalid"', 1)
            break
    return content

def generate_ba_140_test(base_content):
    """BA_140: face:loopCount less than 1."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]' in line:
            content[i] = '        uniform uint[] face:loopCount = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]\n'  # First is 0
            break
    return content

def generate_ba_190_test(base_content):
    """BA_190: Invalid edgeuse:orientationType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] edgeuse:orientationType = [' in line:
            content[i] = content[i].replace('"same"', '"invalid"', 1)
            break
    return content

def generate_ba_195_test(base_content):
    """BA_195: Invalid edgeuse:thisRadialEntryType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] edgeuse:thisRadialEntryType = [' in line:
            content[i] = content[i].replace('"topEntry"', '"invalidEntry"', 1)
            break
    return content

def generate_ba_315_test(base_content):
    """BA_315: Invalid vertex:pointType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] vertex:pointType = [' in line:
            content[i] = content[i].replace('"BrepPointAPI"', '"InvalidPoint"', 1)
            break
    return content

def generate_ba_335_test(base_content):
    """BA_335: Non-positive NURBS order."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2,' in line:
            content[i] = '        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]\n'
            break
    return content

def generate_ba_340_test(base_content):
    """BA_340: NURBS order > vertex count."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2,' in line:
            content[i] = '        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]\n'  # 5 > 2 (vertex count)
            break
    return content

def generate_ba_350_test(base_content):
    """BA_350: Non-positive NURBS weights."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:edge3dNurb:curve3d:nurb:weights = [1, 1,' in line:
            content[i] = content[i].replace('[1,', '[-1,', 1)  # Make first weight negative
            break
    return content

def generate_ba_425_test(base_content):
    """BA_425: Non-positive surface orders."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] brep:surface:nurb:uOrder = [2, 2,' in line:
            content[i] = '        uniform uint[] brep:surface:nurb:uOrder = [0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]\n'
            break
    return content

def generate_ba_440_test(base_content):
    """BA_440: Non-positive surface weights."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:surface:nurb:weights = [1, 1,' in line:
            content[i] = content[i].replace('[1,', '[0,', 1)  # Make first weight 0
            break
    return content

def generate_ba_040_test(base_content):
    """BA_040: brep:extent X outside prim extent."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Make brep extent exceed prim extent
            content[i] = '        uniform double3[] brep:extent = [(-10.0, -0.00001, -0.00001), (1.00001, 1.00001, 1.00001), (1.99999, -0.00001, -0.00001), (3.00001, 1.00001, 1.00001)]\n'
            break
    return content

def generate_ba_045_test(base_content):
    """BA_045: brep:extent Y outside prim extent."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Make brep extent exceed prim extent in Y
            content[i] = '        uniform double3[] brep:extent = [(-0.00001, -10.0, -0.00001), (1.00001, 1.00001, 1.00001), (1.99999, -0.00001, -0.00001), (3.00001, 1.00001, 1.00001)]\n'
            break
    return content

def generate_ba_050_test(base_content):
    """BA_050: brep:extent Z outside prim extent.""" 
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double3[] brep:extent = ' in line:
            # Make brep extent exceed prim extent in Z
            content[i] = '        uniform double3[] brep:extent = [(-0.00001, -0.00001, -10.0), (1.00001, 1.00001, 1.00001), (1.99999, -0.00001, -0.00001), (3.00001, 1.00001, 1.00001)]\n'
            break
    return content

def generate_ba_065_test(base_content):
    """BA_065: Wrong region array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] region:shellCount = [2, 2, 2, 2]' in line:
            content[i] = '        uniform uint[] region:shellCount = [2, 2, 2]\n'  # Too small
            break
    return content

def generate_ba_070_test(base_content):
    """BA_070: Missing region schema attributes."""
    content = base_content.copy()
    # Remove region:shellCount line entirely (required attribute)
    content = [line for line in content if 'uniform uint[] region:shellCount' not in line]
    return content

def generate_ba_080_test(base_content):
    """BA_080: Wrong shell array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] shell:faceuseCount = [6, 6, 6, 6]' in line:
            content[i] = '        uniform uint[] shell:faceuseCount = [6, 6, 6]\n'  # Too small
            break
    return content

def generate_ba_085_test(base_content):
    """BA_085: Missing shell schema attributes."""
    content = base_content.copy()
    # Remove shell:faceuseCount line entirely (required attribute)
    content = [line for line in content if 'uniform uint[] shell:faceuseCount' not in line]
    return content

def generate_ba_100_test(base_content):
    """BA_100: Wrong faceuse array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] faceuse:faceIndex = [' in line:
            # Remove some elements to make wrong size
            content[i] = '        uniform uint[] faceuse:faceIndex = [5, 4, 2, 0, 3]\n'  # Too small
            break
    return content

def generate_ba_105_test(base_content):
    """BA_105: Missing faceuse schema attributes."""
    content = base_content.copy()
    # Remove faceuse:faceIndex line entirely
    content = [line for line in content if 'uniform uint[] faceuse:faceIndex' not in line]
    return content

def generate_ba_120_test(base_content):
    """BA_120: Wrong face array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]' in line:
            content[i] = '        uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1]\n'  # Too small
            break
    return content

def generate_ba_125_test(base_content):
    """BA_125: Missing face schema attributes."""
    content = base_content.copy()
    # Remove face:loopCount line entirely (required attribute)
    content = [line for line in content if 'uniform uint[] face:loopCount' not in line]
    return content

def generate_ba_145_test(base_content):
    """BA_145: Invalid face:range structure."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double2[] face:range = [' in line:
            # Add invalid range with 3 elements instead of 2
            content[i] = '        uniform double2[] face:range = [(0, 0, 1), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1)]\n'
            break
    return content

def generate_ba_150_test(base_content):
    """BA_150: Wrong face:range size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double2[] face:range = [' in line:
            # Remove some elements to make wrong size
            content[i] = '        uniform double2[] face:range = [(0, 0), (1, 1), (0, 0), (1, 1)]\n'  # Too small
            break
    return content

def generate_ba_155_test(base_content):
    """BA_155: Invalid U range order."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double2[] face:range = [' in line:
            # Make Umax < Umin for first face
            content[i] = content[i].replace('(0, 0), (1, 1)', '(1, 0), (0, 1)', 1)
            break
    return content

def generate_ba_160_test(base_content):
    """BA_160: Invalid V range order.""" 
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double2[] face:range = [' in line:
            # Make Vmax < Vmin for first face
            content[i] = content[i].replace('(0, 0), (1, 1)', '(0, 1), (1, 0)', 1)
            break
    return content

def generate_ba_165_test(base_content):
    """BA_165: Wrong loop array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] loop:edgeuseCount = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]' in line:
            content[i] = '        uniform uint[] loop:edgeuseCount = [4, 4, 4, 4]\n'  # Too small
            break
    return content

def generate_ba_170_test(base_content):
    """BA_170: Missing loop schema attributes."""
    content = base_content.copy()
    # Remove loop:edgeuseCount line entirely (required attribute)
    content = [line for line in content if 'uniform uint[] loop:edgeuseCount' not in line]
    return content

def generate_ba_175_test(base_content):
    """BA_175: Invalid loop vertex reference."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] loop:vertexIndex = [9999999' in line:
            content[i] = '        uniform uint[] loop:vertexIndex = [999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999]\n'
            break
    return content

def generate_ba_180_test(base_content):
    """BA_180: Wrong edgeuse array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] edgeuse:edgeIndex = [' in line:
            # Remove some elements to make wrong size
            content[i] = '        uniform uint[] edgeuse:edgeIndex = [2, 11, 4, 7]\n'  # Too small
            break
    return content

def generate_ba_185_test(base_content):
    """BA_185: Missing edgeuse schema attributes."""
    content = base_content.copy()
    # Remove edgeuse:edgeIndex line entirely
    content = [line for line in content if 'uniform uint[] edgeuse:edgeIndex' not in line]
    return content

def generate_ba_200_test(base_content):
    """BA_200: Invalid edgeuse nextRadialEUIndex values."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] edgeuse:nextRadialEUIndex = [' in line:
            # Replace first index with out-of-range value
            content[i] = content[i].replace('[11,', '[999,', 1)
            break
    return content

def generate_ba_205_test(base_content):
    """BA_205: Invalid edgeuse edgeIndex values."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] edgeuse:edgeIndex = [' in line:
            # Replace first index with out-of-range value
            content[i] = content[i].replace('[2,', '[999,', 1)
            break
    return content

def generate_ba_210_test(base_content):
    """BA_210: Wrong edge array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] edge:curveType = [' in line:
            # Truncate curveType array to wrong size
            content[i] = '        uniform token[] edge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]\n'  # Too small
            break
    return content

def generate_ba_215_test(base_content):
    """BA_215: Missing edge schema attributes."""
    content = base_content.copy()
    # Remove edge:curveType line entirely (required attribute)
    content = [line for line in content if 'uniform token[] edge:curveType' not in line]
    return content

def generate_ba_225_test(base_content):
    """BA_225: Invalid edge vertexIndices values."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform int2[] edge:vertexIndices = [' in line:
            # Replace first vertex index with out-of-range value
            content[i] = content[i].replace('(4, 0)', '(999, 0)', 1)
            break
    return content

def generate_ba_230_test(base_content):
    """BA_230: Invalid edge range structure."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] edge:range = [0, 1, 0, 1,' in line:
            # Add invalid range structure
            content[i] = '        uniform double[] edge:range = [0, 1, 0]\n'  # Odd number
            break
    return content

def generate_ba_235_test(base_content):
    """BA_235: Invalid edge range order."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] edge:range = [0, 1, 0, 1,' in line:
            # Make range max < min
            content[i] = content[i].replace('0, 1, 0, 1,', '1, 0, 0, 1,', 1)
            break
    return content

def generate_ba_245_test(base_content):
    """BA_245: Invalid edge curveType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] edge:curveType = [' in line:
            content[i] = content[i].replace('"BrepCurve3dNurbAPI"', '"InvalidCurve"', 1)
            break
    return content

def generate_ba_250_test(base_content):
    """BA_250: Wrong wireEdge array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] wireEdge:curveType = []' in line:
            content[i] = '        uniform token[] wireEdge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]\n'  # Should be empty
            break
    return content

def generate_ba_255_test(base_content):
    """BA_255: Partial wireEdge schema attribute authorship."""
    content = base_content.copy()
    # Remove wireEdge:curveType while other topology attrs remain authored (all-or-none violation)
    content = [line for line in content if 'uniform token[] wireEdge:curveType' not in line]
    return content

def generate_ba_260_test(base_content):
    """BA_260: Invalid wireEdge curveType token."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] wireEdge:curveType = []' in line:
            content[i] = '        uniform token[] wireEdge:curveType = ["InvalidWireCurve"]\n'
            break
    return content

def generate_ba_265_test(base_content):
    """BA_265: Invalid wireEdge vertexIndices values."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform int2[] wireEdge:vertexIndices = []' in line:
            content[i] = '        uniform int2[] wireEdge:vertexIndices = [(999, 1000)]\n'  # Out of range
            break
    return content

def generate_ba_270_test(base_content):
    """BA_270: Invalid wireEdge range structure."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] wireEdge:range = []' in line:
            content[i] = '        uniform double[] wireEdge:range = [0, 1, 0]\n'  # Odd number
            break
    return content

def generate_ba_275_test(base_content):
    """BA_275: Invalid wireEdge range order."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] wireEdge:range = []' in line:
            content[i] = '        uniform double[] wireEdge:range = [1, 0]\n'  # max < min
            break
    return content

def generate_ba_290_test(base_content):
    """BA_290: WireEdge 3D NURBS schema consistency."""
    content = base_content.copy()
    # Add wire edge with inconsistent NURBS data
    for i, line in enumerate(content):
        if 'uniform token[] wireEdge:curveType = []' in line:
            content[i] = '        uniform token[] wireEdge:curveType = ["BrepCurve3dNurbAPI"]\n'
            break
    return content

def generate_ba_295_test(base_content):
    """BA_295: Wrong vertex array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform token[] vertex:pointType = [' in line:
            # Truncate pointType array to wrong size
            content[i] = '        uniform token[] vertex:pointType = ["BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI"]\n'  # Too small
            break
    return content

def generate_ba_300_test(base_content):
    """BA_300: Missing vertex schema attributes."""
    content = base_content.copy()
    # Remove vertex:pointType line entirely (required attribute)
    content = [line for line in content if 'uniform token[] vertex:pointType' not in line]
    return content

def generate_ba_305_test(base_content):
    """BA_305: Vertex point schema consistency."""
    content = base_content.copy()
    # Make vertex point type inconsistent with actual point data
    for i, line in enumerate(content):
        if 'uniform token[] vertex:pointType = [' in line:
            content[i] = content[i].replace('"BrepPointAPI"', '"none"', 1)  # But we have point data
            break
    return content

def generate_ba_310_test(base_content):
    """BA_310: Vertex position containment."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            # Make first vertex position outside extent
            content[i] = content[i].replace('(1, 1, 1)', '(100, 100, 100)', 1)
            break
    return content

def generate_ba_320_test(base_content):
    """BA_320: Wrong vertexPoint position array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            # Remove some positions to make wrong size
            content[i] = '        uniform point3d[] brep:vertexPoint:point:position = [(1, 1, 1), (0, 0, 1)]\n'  # Too small
            break
    return content

def generate_ba_325_test(base_content):
    """BA_325: Wrong shellPoint position array size."""
    content = base_content.copy()
    # Add shell point data when there shouldn't be any
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform point3d[] brep:shellPoint:point:position = [(0, 0, 0)]\n'
            break
    return content

def generate_ba_330_test(base_content):
    """BA_330: Wrong 3D NURBS curve order array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [' in line:
            content[i] = '        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2]\n'  # Too small
            break
    return content

def generate_ba_345_test(base_content):
    """BA_345: Wrong 3D NURBS curve control vertices/weights size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [' in line:
            # Remove some control vertices to make wrong size
            content[i] = '        uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [(1, 0, 1), (1, 1, 1)]\n'
            break
    return content

def generate_ba_355_test(base_content):
    """BA_355: Wrong 3D NURBS curve knot vector size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [' in line:
            content[i] = '        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [0, 0, 1]\n'  # Too small
            break
    return content

def generate_ba_360_test(base_content):
    """BA_360: Wrong 3D NURBS curve knot vector ordering."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [' in line:
            # Make knots non-decreasing
            content[i] = '        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]\n'
            break
    return content

def generate_ba_365_test(base_content):
    """BA_365: Edge NURBS control point containment."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [' in line:
            # Make first control vertex outside extent
            content[i] = content[i].replace('(1, 0, 1)', '(100, 100, 100)', 1)
            break
    return content

def generate_ba_370_test(base_content):
    """BA_370: Edge 3D NURBS schema and data consistency."""
    content = base_content.copy()
    # Make edge curve type inconsistent with NURBS data
    for i, line in enumerate(content):
        if 'uniform token[] edge:curveType = [' in line:
            content[i] = content[i].replace('"BrepCurve3dNurbAPI"', '"line"', 1)  # But we have NURBS data
            break
    return content

def generate_ba_375_test(base_content):
    """BA_375: Wrong UV curve array size."""
    content = base_content.copy()
    # Add UV curve data with wrong size
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:order = [2]\n'  # Wrong size
            break
    return content

def generate_ba_380_test(base_content):
    """BA_380: Non-positive UV curve order."""
    content = base_content.copy()
    # Add UV curve data with non-positive order
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:order = [0]\n'
            break
    return content

def generate_ba_385_test(base_content):
    """BA_385: UV curve order exceeds vertex count."""
    content = base_content.copy()
    # Add UV curve data with order > vertex count
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:order = [5]\n        uniform uint[] brep:curveUv:nurb:vertexCount = [2]\n'  # 5 > 2
            break
    return content

def generate_ba_390_test(base_content):
    """BA_390: Wrong UV curve control vertices size."""
    content = base_content.copy()
    # Add UV curve data with wrong control vertices size
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:vertexCount = [3]\n        uniform double2[] brep:curveUv:nurb:controlVertices = [(0, 0)]\n'  # Size 1 vs expected 3
            break
    return content

def generate_ba_395_test(base_content):
    """BA_395: Wrong UV curve knot vector size."""
    content = base_content.copy()
    # Add UV curve data with wrong knot size
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:order = [2]\n        uniform uint[] brep:curveUv:nurb:vertexCount = [2]\n        uniform double[] brep:curveUv:nurb:knots = [0, 1]\n'  # Size 2 vs expected 4
            break
    return content

def generate_ba_400_test(base_content):
    """BA_400: Wrong UV curve knot ordering."""
    content = base_content.copy()
    # Add UV curve data with wrong knot ordering
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:order = [2]\n        uniform uint[] brep:curveUv:nurb:vertexCount = [2]\n        uniform double[] brep:curveUv:nurb:knots = [1, 0, 1, 0]\n'  # Non-decreasing
            break
    return content

def generate_ba_405_test(base_content):
    """BA_405: Wrong UV curve weights size."""
    content = base_content.copy()
    # Add UV curve data with wrong weights size
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform uint[] brep:curveUv:nurb:vertexCount = [3]\n        uniform double[] brep:curveUv:nurb:weights = [1]\n'  # Size 1 vs expected 3
            break
    return content

def generate_ba_410_test(base_content):
    """BA_410: Non-positive UV curve weights."""
    content = base_content.copy()
    # Add UV curve data with non-positive weights
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform double[] brep:curveUv:nurb:weights = [0, 1]\n'  # First weight is 0
            break
    return content

def generate_ba_415_test(base_content):
    """BA_415: UV curve NURBS schema consistency."""
    content = base_content.copy()
    # Add inconsistent UV curve schema
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content[i] = content[i] + '        uniform double[] brep:curveUv:nurb:weights = [1, 1]\n'  # But no curve type declared
            break
    return content

def generate_ba_420_test(base_content):
    """BA_420: Wrong surface NURBS array size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]' in line:
            content[i] = '        uniform uint[] brep:surface:nurb:uOrder = [2, 2]\n'  # Too small
            break
    return content

def generate_ba_430_test(base_content):
    """BA_430: Surface orders exceed vertex counts."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]' in line:
            content[i] = '        uniform uint[] brep:surface:nurb:uOrder = [5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]\n'  # 5 > 2 (vertex count)
            break
    return content

def generate_ba_435_test(base_content):
    """BA_435: Wrong surface control vertices and weights size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform point3d[] brep:surface:nurb:controlVertices = [' in line:
            # Remove some control vertices to make wrong size
            content[i] = '        uniform point3d[] brep:surface:nurb:controlVertices = [(0, 0, 0), (0, 1, 0)]\n'  # Too small
            break
    return content

def generate_ba_445_test(base_content):
    """BA_445: Wrong surface U knot vector size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:surface:nurb:uKnots = [' in line:
            content[i] = '        uniform double[] brep:surface:nurb:uKnots = [0, 0, 1]\n'  # Too small
            break
    return content

def generate_ba_450_test(base_content):
    """BA_450: Wrong surface V knot vector size."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:surface:nurb:vKnots = [' in line:
            content[i] = '        uniform double[] brep:surface:nurb:vKnots = [0, 0, 1]\n'  # Too small
            break
    return content

def generate_ba_455_test(base_content):
    """BA_455: Wrong surface U knot ordering."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:surface:nurb:uKnots = [' in line:
            # Make knots non-decreasing
            content[i] = '        uniform double[] brep:surface:nurb:uKnots = [1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]\n'
            break
    return content

def generate_ba_460_test(base_content):
    """BA_460: Wrong surface V knot ordering."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform double[] brep:surface:nurb:vKnots = [' in line:
            # Make knots non-decreasing
            content[i] = '        uniform double[] brep:surface:nurb:vKnots = [1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]\n'
            break
    return content

def generate_ba_465_test(base_content):
    """BA_465: Surface NURBS control point containment."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform point3d[] brep:surface:nurb:controlVertices = [' in line:
            # Make first control vertex outside extent
            content[i] = content[i].replace('(0, 0, 0)', '(100, 100, 100)', 1)
            break
    return content

def generate_ba_470_test(base_content):
    """BA_470: Surface schema consistency."""
    content = base_content.copy()
    # Make face surface type inconsistent with NURBS data
    for i, line in enumerate(content):
        if 'uniform token[] face:surfaceType = [' in line:
            content[i] = content[i].replace('"BrepSurfaceNurbAPI"', '"plane"', 1)  # But we have NURBS data
            break
    return content

# Data Type Test Generators (BA_061, BA_076, BA_091, BA_116, BA_161, BA_176, BA_196, BA_237, BA_291, BA_316, BA_326, BA_327, BA_371, BA_416, BA_471)

def generate_ba_061_test(base_content):
    """BA_061: Brep attribute data types."""
    content = base_content.copy()
    # Change brep:regionCount from uint[] to int[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] brep:regionCount = [' in line:
            content[i] = line.replace('uniform uint[] brep:regionCount', 'uniform int[] brep:regionCount')
            break
    return content

def generate_ba_076_test(base_content):
    """BA_076: Region attribute data types."""
    content = base_content.copy()
    # Change region:shellCount from uint[] to int[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] region:shellCount = [' in line:
            content[i] = line.replace('uniform uint[] region:shellCount', 'uniform int[] region:shellCount')
            break
    return content

def generate_ba_091_test(base_content):
    """BA_091: Shell attribute data types."""
    content = base_content.copy()
    # Change shell:faceuseCount from uint[] to double[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] shell:faceuseCount = [' in line:
            content[i] = line.replace('uniform uint[] shell:faceuseCount', 'uniform double[] shell:faceuseCount')
            break
    return content

def generate_ba_116_test(base_content):
    """BA_116: Faceuse attribute data types."""
    content = base_content.copy()
    # Change faceuse:faceIndex from uint[] to float[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] faceuse:faceIndex = [' in line:
            content[i] = line.replace('uniform uint[] faceuse:faceIndex', 'uniform float[] faceuse:faceIndex')
            break
    return content

def generate_ba_161_test(base_content):
    """BA_161: Face attribute data types."""
    content = base_content.copy()
    # Change face:range from double2[] to float2[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform double2[] face:range = [' in line:
            content[i] = line.replace('uniform double2[] face:range', 'uniform float2[] face:range')
            break
    return content

def generate_ba_176_test(base_content):
    """BA_176: Loop attribute data types."""
    content = base_content.copy()
    # Change loop:edgeuseCount from uint[] to string[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] loop:edgeuseCount = [' in line:
            content[i] = line.replace('uniform uint[] loop:edgeuseCount', 'uniform string[] loop:edgeuseCount')
            break
    return content

def generate_ba_196_test(base_content):
    """BA_196: Edgeuse attribute data types."""
    content = base_content.copy()
    # Change edgeuse:nextRadialEUIndex from uint[] to int[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] edgeuse:nextRadialEUIndex = [' in line:
            content[i] = line.replace('uniform uint[] edgeuse:nextRadialEUIndex', 'uniform int[] edgeuse:nextRadialEUIndex')
            break
    return content

def generate_ba_237_test(base_content):
    """BA_237: Edge attribute data types."""
    content = base_content.copy()
    # Change edge:vertexIndices from int2[] to uint2[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform int2[] edge:vertexIndices = [' in line:
            content[i] = line.replace('uniform int2[] edge:vertexIndices', 'uniform uint2[] edge:vertexIndices')
            break
    return content

def generate_ba_291_test(base_content):
    """BA_291: WireEdge attribute data types."""
    content = base_content.copy()
    # Change wireEdge:range from double[] to float[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform double[] wireEdge:range = [' in line:
            content[i] = line.replace('uniform double[] wireEdge:range', 'uniform float[] wireEdge:range')
            break
    return content

def generate_ba_316_test(base_content):
    """BA_316: Vertex attribute data types."""
    content = base_content.copy()
    # Change vertex:pointType from token[] to string[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform token[] vertex:pointType = [' in line:
            content[i] = line.replace('uniform token[] vertex:pointType', 'uniform string[] vertex:pointType')
            break
    return content

def generate_ba_326_test(base_content):
    """BA_326: VertexPoint attribute data types."""
    content = base_content.copy()
    # Change brep:vertexPoint:point:position to an incorrect data type (vector3d[] instead of point3d[])
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            rhs = line.split('=', 1)[1]
            content[i] = f'        uniform vector3d[] brep:vertexPoint:point:position ={rhs}'
            break
    return content

def generate_ba_327_test(base_content):
    """BA_327: ShellPoint attribute data types."""
    content = base_content.copy()
    # Add shellPoint data with wrong data type (vector3d[] instead of point3d[])
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content.insert(i + 1, '        uniform vector3d[] brep:shellPoint:point:position = [(0, 0, 0)]\n')
            break
    return content

def generate_ba_371_test(base_content):
    """BA_371: Edge3dNurb attribute data types."""
    content = base_content.copy()
    # Change brep:edge3dNurb:curve3d:nurb:order from uint[] to int[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [' in line:
            content[i] = line.replace('uniform uint[] brep:edge3dNurb:curve3d:nurb:order', 'uniform int[] brep:edge3dNurb:curve3d:nurb:order')
            break
    return content

def generate_ba_416_test(base_content):
    """BA_416: CurveUv attribute data types."""
    content = base_content.copy()
    # Add CurveUv data with wrong controlVertices type (point2d[] instead of double2[])
    for i, line in enumerate(content):
        if 'brep:vertexPoint:point:position' in line:
            content.insert(i + 1, '        uniform int[] brep:curveUv:nurb:order = [2, 2]\n')
            content.insert(i + 2, '        uniform int[] brep:curveUv:nurb:vertexCount = [2, 2]\n')
            content.insert(i + 3, '        uniform point2d[] brep:curveUv:nurb:controlVertices = [(0, 0), (1, 0)]\n')  # Wrong type
            break
    return content

def generate_ba_471_test(base_content):
    """BA_471: Surface attribute data types."""
    content = base_content.copy()
    # Change brep:surface:nurb:weights from double[] to float[] to trigger data type validation
    for i, line in enumerate(content):
        if 'uniform double[] brep:surface:nurb:weights = [' in line:
            content[i] = line.replace('uniform double[] brep:surface:nurb:weights', 'uniform float[] brep:surface:nurb:weights')
            break
    return content

def generate_ba_580_test(base_content):
    """BA_580: Faceuse pairing - face not referenced by exactly 2 faceuses."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] faceuse:faceIndex = [' in line:
            # In brep 0, change the reference to face 3 (position 4) to face 0.
            # Face 0 ends up with 3 refs, face 3 with only 1.
            content[i] = content[i].replace('0, 3, 1,', '0, 0, 1,', 1)
            break
    return content

def generate_ba_581_test(base_content):
    """BA_581: Radial edgeuse closure - broken radial chain."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] edgeuse:nextRadialEUIndex = [' in line:
            # Break the radial chain 0->11->0 by making index 11 a self-loop (11->11).
            # EU 0 then chases 0->11->11->11... and never returns to 0.
            content[i] = content[i].replace('20, 0, 18', '20, 11, 18', 1)
            break
    return content

def generate_ba_582_test(base_content):
    """BA_582: Orphan edge - edge not referenced by any edgeuse."""
    content = base_content.copy()
    for i, line in enumerate(content):
        if 'uniform uint[] edgeuse:edgeIndex = [' in line:
            # Replace both references to edge 23 (brep 1) with edge 12,
            # making edge 23 an orphan with zero edgeuse references.
            content[i] = content[i].replace('14, 23, 16', '14, 12, 16', 1)
            content[i] = content[i].replace('17, 23]', '17, 12]', 1)
            break
    return content

# Test generators mapping
TEST_GENERATORS = {
    'BA_000': (generate_ba_000_test, "Inconsistent brep attribute sizes"),
    'BA_005': (generate_ba_005_test, "Missing brep schema attributes"),  
    'BA_010': (generate_ba_010_test, "Negative intersectTol3d value"),
    'BA_020': (generate_ba_020_test, "Invalid extent structure - wrong size"),
    'BA_025': (generate_ba_025_test, "Invalid extent X order (Xmax < Xmin)"),
    'BA_030': (generate_ba_030_test, "Invalid extent Y order (Ymax < Ymin)"),
    'BA_035': (generate_ba_035_test, "Invalid extent Z order (Zmax < Zmin)"),
    'BA_040': (generate_ba_040_test, "brep extent X outside prim extent"),
    'BA_045': (generate_ba_045_test, "brep extent Y outside prim extent"),
    'BA_050': (generate_ba_050_test, "brep extent Z outside prim extent"),
    'BA_061': (generate_ba_061_test, "Invalid brep attribute data types"),
    'BA_065': (generate_ba_065_test, "Wrong region array size"),
    'BA_070': (generate_ba_070_test, "Missing region schema attributes"),
    'BA_075': (generate_ba_075_test, "Invalid region type token"),
    'BA_076': (generate_ba_076_test, "Invalid region attribute data types"),
    'BA_080': (generate_ba_080_test, "Wrong shell array size"),
    'BA_085': (generate_ba_085_test, "Missing shell schema attributes"),
    'BA_090': (generate_ba_090_test, "Invalid shell pointType token"),
    'BA_091': (generate_ba_091_test, "Invalid shell attribute data types"),
    'BA_100': (generate_ba_100_test, "Wrong faceuse array size"),
    'BA_105': (generate_ba_105_test, "Missing faceuse schema attributes"),
    'BA_110': (generate_ba_110_test, "Invalid faceuse orientationType token"),
    'BA_115': (generate_ba_115_test, "Invalid faceuse faceIndex - out of range"),
    'BA_116': (generate_ba_116_test, "Invalid faceuse attribute data types"),
    'BA_120': (generate_ba_120_test, "Wrong face array size"),
    'BA_125': (generate_ba_125_test, "Missing face schema attributes"),
    'BA_130': (generate_ba_130_test, "Invalid face surfaceType token"),
    'BA_135': (generate_ba_135_test, "Invalid face trimType token"),
    'BA_140': (generate_ba_140_test, "face loopCount less than 1"),
    'BA_145': (generate_ba_145_test, "Invalid face range structure"),
    'BA_150': (generate_ba_150_test, "Wrong face range size"),
    'BA_155': (generate_ba_155_test, "Invalid U range order"),
    'BA_160': (generate_ba_160_test, "Invalid V range order"),
    'BA_161': (generate_ba_161_test, "Invalid face attribute data types"),
    'BA_165': (generate_ba_165_test, "Wrong loop array size"),
    'BA_170': (generate_ba_170_test, "Missing loop schema attributes"),
    'BA_175': (generate_ba_175_test, "Invalid loop vertex reference"),
    'BA_176': (generate_ba_176_test, "Invalid loop attribute data types"),
    'BA_180': (generate_ba_180_test, "Wrong edgeuse array size"),
    'BA_185': (generate_ba_185_test, "Missing edgeuse schema attributes"),
    'BA_190': (generate_ba_190_test, "Invalid edgeuse orientationType token"),
    'BA_195': (generate_ba_195_test, "Invalid edgeuse thisRadialEntryType token"),
    'BA_196': (generate_ba_196_test, "Invalid edgeuse attribute data types"),
    'BA_200': (generate_ba_200_test, "Invalid edgeuse nextRadialEUIndex values"),
    'BA_205': (generate_ba_205_test, "Invalid edgeuse edgeIndex values"),
    'BA_210': (generate_ba_210_test, "Wrong edge array size"),
    'BA_215': (generate_ba_215_test, "Missing edge schema attributes"),
    'BA_225': (generate_ba_225_test, "Invalid edge vertexIndices values"),
    'BA_230': (generate_ba_230_test, "Invalid edge range structure"),
    'BA_235': (generate_ba_235_test, "Invalid edge range order"),
    'BA_237': (generate_ba_237_test, "Invalid edge attribute data types"),
    'BA_245': (generate_ba_245_test, "Invalid edge curveType token"),
    'BA_250': (generate_ba_250_test, "Wrong wireEdge array size"),
    'BA_255': (generate_ba_255_test, "Partial wireEdge schema attribute authorship"),
    'BA_260': (generate_ba_260_test, "Invalid wireEdge curveType token"),
    'BA_265': (generate_ba_265_test, "Invalid wireEdge vertexIndices values"),
    'BA_270': (generate_ba_270_test, "Invalid wireEdge range structure"),
    'BA_275': (generate_ba_275_test, "Invalid wireEdge range order"),
    'BA_290': (generate_ba_290_test, "WireEdge 3D NURBS schema consistency"),
    'BA_291': (generate_ba_291_test, "Invalid wireEdge attribute data types"),
    'BA_295': (generate_ba_295_test, "Wrong vertex array size"),
    'BA_300': (generate_ba_300_test, "Missing vertex schema attributes"),
    'BA_305': (generate_ba_305_test, "Vertex point schema consistency"),
    'BA_310': (generate_ba_310_test, "Vertex position containment"),
    'BA_315': (generate_ba_315_test, "Invalid vertex pointType token"),
    'BA_316': (generate_ba_316_test, "Invalid vertex attribute data types"),
    'BA_320': (generate_ba_320_test, "Wrong vertexPoint position array size"),
    'BA_325': (generate_ba_325_test, "Wrong shellPoint position array size"),
    'BA_326': (generate_ba_326_test, "Invalid vertexPoint attribute data types"),
    'BA_327': (generate_ba_327_test, "Invalid shellPoint attribute data types"),
    'BA_330': (generate_ba_330_test, "Wrong 3D NURBS curve order array size"),
    'BA_335': (generate_ba_335_test, "Non-positive NURBS curve order"),
    'BA_340': (generate_ba_340_test, "NURBS order exceeds vertex count"),
    'BA_345': (generate_ba_345_test, "Wrong 3D NURBS curve control vertices and weights size"),
    'BA_350': (generate_ba_350_test, "Non-positive NURBS curve weights"),
    'BA_355': (generate_ba_355_test, "Wrong 3D NURBS curve knot vector size"),
    'BA_360': (generate_ba_360_test, "Wrong 3D NURBS curve knot vector ordering"),
    'BA_365': (generate_ba_365_test, "Edge NURBS control point containment"),
    'BA_370': (generate_ba_370_test, "Edge 3D NURBS schema and data consistency"),
    'BA_371': (generate_ba_371_test, "Invalid edge3dNurb attribute data types"),
    'BA_375': (generate_ba_375_test, "Wrong UV curve array size"),
    'BA_380': (generate_ba_380_test, "Non-positive UV curve order"),
    'BA_385': (generate_ba_385_test, "UV curve order exceeds vertex count"),
    'BA_390': (generate_ba_390_test, "Wrong UV curve control vertices size"),
    'BA_395': (generate_ba_395_test, "Wrong UV curve knot vector size"),
    'BA_400': (generate_ba_400_test, "Wrong UV curve knot ordering"),
    'BA_405': (generate_ba_405_test, "Wrong UV curve weights size"),
    'BA_410': (generate_ba_410_test, "Non-positive UV curve weights"),
    'BA_415': (generate_ba_415_test, "UV curve NURBS schema consistency"),
    'BA_416': (generate_ba_416_test, "Invalid curveUv attribute data types"),
    'BA_420': (generate_ba_420_test, "Wrong surface NURBS array size"),
    'BA_425': (generate_ba_425_test, "Non-positive NURBS surface order"),
    'BA_430': (generate_ba_430_test, "Surface orders exceed vertex counts"),
    'BA_435': (generate_ba_435_test, "Wrong surface control vertices and weights size"),
    'BA_440': (generate_ba_440_test, "Non-positive NURBS surface weights"),
    'BA_445': (generate_ba_445_test, "Wrong surface U knot vector size"),
    'BA_450': (generate_ba_450_test, "Wrong surface V knot vector size"),
    'BA_455': (generate_ba_455_test, "Wrong surface U knot ordering"),
    'BA_460': (generate_ba_460_test, "Wrong surface V knot ordering"),
    'BA_465': (generate_ba_465_test, "Surface NURBS control point containment"),
    'BA_470': (generate_ba_470_test, "Surface schema consistency"),
    'BA_471': (generate_ba_471_test, "Invalid surface attribute data types"),
    'BA_580': (generate_ba_580_test, "Faceuse pairing violation"),
    'BA_581': (generate_ba_581_test, "Broken radial edgeuse chain"),
    'BA_582': (generate_ba_582_test, "Orphan edge not referenced by any edgeuse"),
}

def main():
    """Generate all test files."""
    print("Generating BrepArray requirement test files...")
    
    # Read base file
    base_content = read_base_file()
    print(f"Read base file with {len(base_content)} lines")
    
    # Generate test files for implemented generators
    generated_count = 0
    for req_id, (generator_func, description) in TEST_GENERATORS.items():
        try:
            modified_content = generator_func(base_content)
            filename = f"Test_{req_id}_{description.replace(' ', '_').replace('-', '_')}.usda"
            # Clean filename - remove problematic characters
            filename = filename.replace('(', '').replace(')', '').replace(',', '').replace('__', '_')
            filename = filename.replace('<', 'less_than').replace('>', 'greater_than')
            
            write_test_file(filename, modified_content, f"{req_id}: {description}")
            print(f"Generated: {filename}")
            generated_count += 1
        except Exception as e:
            print(f"Error generating {req_id}: {e}")
    
    print(f"\nGenerated {generated_count} test files out of {len(TEST_GENERATORS)} implemented generators")
    print("Files saved to TestFiles/brep_validator/ directory")
    
    # Status about requirement coverage
    if len(TEST_GENERATORS) == 110:
        print(f"\nCOMPLETE: All 110 BrepArray requirements now have test file generators!")
        print("This includes comprehensive test coverage for:")
        print("  • Original 95 requirements (BA_000-BA_470)")
        print("  • 15 new data type validation requirements (BA_061, BA_076, BA_091, BA_116, BA_161, BA_176, BA_196, BA_237, BA_291, BA_316, BA_326, BA_327, BA_371, BA_416, BA_471)")
        print("This provides comprehensive test coverage for the entire BrepArray validation system!")
    else:
        remaining = 110 - len(TEST_GENERATORS)
        print(f"\nNote: {remaining} requirements still need test file generators to be implemented")
        print("This covers the most common validation failure patterns.")

if __name__ == "__main__":
    main()
