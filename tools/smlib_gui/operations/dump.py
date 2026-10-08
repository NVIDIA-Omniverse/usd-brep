# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Kernel Dump actions for loaded BReps and picked topology."""

from __future__ import annotations

from dataclasses import dataclass

from ..model import ActiveObject
from ..runtime import smdev

KERNEL_DUMP_KINDS = {"Brep", "Curve", "Surface"}


@dataclass
class DumpResult:
    source_line: str
    status: str
    details: str


def object_can_dump(obj: ActiveObject | None) -> bool:
    return smdev is not None and obj is not None and obj.kind in KERNEL_DUMP_KINDS


def run_dump(handle, *, label: str, source_expr: str, abbreviated: bool = False) -> DumpResult:
    """Run ``smdev.dump``. Debug SMLib echoes kernel ``Dump()`` to stderr."""
    if smdev is None:
        raise RuntimeError("_smlib_dev is not available for Dump")
    smdev.dump(handle, abbreviated=abbreviated)
    source = f"smdev.dump({source_expr})"
    if abbreviated:
        source = f"smdev.dump({source_expr}, abbreviated=True)"
    return DumpResult(
        source_line=source,
        status=f"Invoked Dump() for {label}.",
        details=f"Dump: {label}",
    )


__all__ = [
    "DumpResult",
    "KERNEL_DUMP_KINDS",
    "object_can_dump",
    "run_dump",
]
