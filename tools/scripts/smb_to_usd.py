#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Wrap a single native .smb BRep file in a standalone USD file.

Useful for isolating one prim extracted from a larger scene (e.g. via
smtess_usd's --dump-slow-threshold/--dump-slow-dir) so it can be opened
directly with view_usd.py without needing the original scene.

Usage (after `source tools/scripts/run_view_env.sh`):
    python3 tools/scripts/smb_to_usd.py input.smb output.usd
"""
import sys

import _omni_solid as sm


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <input.smb> <output.usd>", file=sys.stderr)
        return 1

    brep = sm.Brep.read_from_file(sys.argv[1])
    sm.usd.export_brep(brep, sys.argv[2])
    print(f"wrote {sys.argv[2]} ({len(brep.faces())} faces)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
