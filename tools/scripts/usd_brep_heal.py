#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Heal BrepArray definitions in a USD scene: the usd-brep-heal command (usd_brep._heal_cli).

Needs usd_brep importable: the wheel installed, or a package's python/ and usdpy/ on PYTHONPATH.
"""

from usd_brep._heal_cli import main

if __name__ == "__main__":
    main()
