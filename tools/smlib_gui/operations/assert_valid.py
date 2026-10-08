# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""AssertValid actions for loaded BReps and picked topology."""

from __future__ import annotations

from dataclasses import dataclass

from ..model import ActiveObject
from ..runtime import smdev
from .picking import TopologyPick

KERNEL_ASSERT_VALID_KINDS = {"Brep", "Curve", "Surface"}


@dataclass
class AssertValidResult:
    ok: bool
    source_line: str
    status: str
    details: str


def object_can_assert_valid(obj: ActiveObject | None) -> bool:
    return smdev is not None and obj is not None and obj.kind in KERNEL_ASSERT_VALID_KINDS


def topology_source_expr(selection: TopologyPick) -> str:
    """Build a replayable expression for the picked Face, Edge, or Vertex."""
    name = selection.object_name
    kind = selection.topology_kind
    index = selection.topology_index
    accessors = {
        "Face": "faces",
        "Edge": "edges",
        "Vertex": "vertices",
    }
    accessor = accessors.get(kind)
    if accessor is None or index < 0:
        return f"{name}  # {kind} {index}"
    return f"{name}.{accessor}()[{index}]"


def run_assert_valid(handle, *, label: str, source_expr: str, level: int = 2, walk: bool = True) -> AssertValidResult:
    """Run ``smdev.assert_valid_reports``, which checks in both debug and release SMLib."""
    if smdev is None:
        raise RuntimeError("_smlib_dev is not available for AssertValid")
    ok, reports = smdev.assert_valid_reports(handle, level=level, walk=walk)
    ok = bool(ok)
    if ok:
        status = f"{label}: AssertValid passed."
    else:
        status = f"{label} failed AssertValid ({len(reports)} report(s))."
    details = f"AssertValid: {label}"
    if reports:
        details = "\n".join([details, *reports])
    return AssertValidResult(
        ok=ok,
        source_line=f"smdev.assert_valid_reports({source_expr}, level={level}, walk={walk})",
        status=status,
        details=details,
    )


__all__ = [
    "AssertValidResult",
    "KERNEL_ASSERT_VALID_KINDS",
    "object_can_assert_valid",
    "run_assert_valid",
    "topology_source_expr",
]
