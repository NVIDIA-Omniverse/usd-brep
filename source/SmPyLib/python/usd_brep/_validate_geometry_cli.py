# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Check the geometry of the BrepArray prims in a USD file.

Calls usd_brep.usd.validate_geometry_file. Exits 0 when every Brep is valid, 1 when any is
invalid and 2 when the file cannot be checked.

Installed by the wheel as the usd-brep-validate-geometry command.
"""

import argparse
import sys

import usd_brep


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="USD file to check")
    parser.add_argument("--level", type=int, choices=(0, 1, 2), default=2, help="check depth, 0 (fastest) to 2 (default)")
    options = parser.parse_args(argv)
    try:
        results = usd_brep.usd.validate_geometry_file(options.input, level=options.level)
    except RuntimeError as error:
        print(f"validation failed: {error}", file=sys.stderr)
        return 2
    for result in results:
        print(f"{result.prim_path}: {'OK' if result.ok else 'FAIL'}")
        for line in [*result.failures, result.message]:
            if line:
                print(f"  {line}")
    return 0 if all(result.ok for result in results) else 1
