# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Heal BrepArray definitions in the current USD composition's layers.

Calls usd_brep.usd.heal_file. External layers used only by unselected variants
remain referenced but are not healed.

Installed by the wheel as the usd-brep-heal command.
"""

import argparse
import sys

import usd_brep


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="USD file to heal")
    parser.add_argument("output", help="new USD file to write; must not exist")
    parser.add_argument("--threads", type=int, default=0, help="worker threads (default: all cores)")
    options = parser.parse_args()
    try:
        results = usd_brep.usd.heal_file(options.input, options.output, threads=options.threads)
    except RuntimeError as error:  # all or nothing: nothing is written on failure
        sys.exit(f"heal failed: {error}")
    for result in results:
        print(f"{result.prim_path}: {result.brep_count} Brep(s) healed")
    print(f"wrote {options.output}")
