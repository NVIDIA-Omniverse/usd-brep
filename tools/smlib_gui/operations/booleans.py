# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Selected-object boolean operations for the SMLib GUI."""

from __future__ import annotations

from ..model import ActiveObject, OBJECT_COLORS
from ..runtime import sm


BOOLEAN_LABELS = {
    "union": "Union",
    "difference": "Difference",
    "intersection": "Intersection",
    "merge": "Merge",
    "cookie": "Cookie",
    "imprint": "Imprint",
}

BOOLEAN_DISPATCH = {
    "union": {
        "function": "boolean_union",
        "source": "{result_name} = sm.boolean_union({a_name}, {b_name})",
    },
    "difference": {
        "function": "boolean_difference",
        "source": "{result_name} = sm.boolean_difference({a_name}, {b_name})",
    },
    "intersection": {
        "function": "boolean_intersection",
        "source": "{result_name} = sm.boolean_intersection({a_name}, {b_name})",
    },
    "merge": {
        "function": "boolean_merge",
        "source": "{result_name} = sm.boolean_merge({a_name}, {b_name})",
    },
    "cookie": {
        "function": "boolean_with_options",
        "options": {"operation": "DIFFERENCE", "cookie_cutter": True},
        "source": (
            "{result_name} = sm.boolean_with_options({a_name}, {b_name}, "
            "operation=sm.BooleanOp.DIFFERENCE, cookie_cutter=True)"
        ),
    },
    "imprint": {
        "function": "boolean_with_options",
        "options": {"operation": "DIFFERENCE", "imprinting": True},
        "source": (
            "{result_name} = sm.boolean_with_options({a_name}, {b_name}, "
            "operation=sm.BooleanOp.DIFFERENCE, imprinting=True)"
        ),
    },
}


def _boolean_dispatch_entry(operation: str) -> dict:
    try:
        return BOOLEAN_DISPATCH[operation]
    except KeyError as exc:
        raise ValueError(f"Unknown boolean operation: {operation}") from exc


def boolean_source_line(operation: str, result_name: str, a_name: str, b_name: str) -> str:
    """Return a reproducible source line for a selected-object boolean."""
    return _boolean_dispatch_entry(operation)["source"].format(
        result_name=result_name,
        a_name=a_name,
        b_name=b_name,
    )


def _run_boolean(operation: str, a: ActiveObject, b: ActiveObject):
    entry = _boolean_dispatch_entry(operation)
    kwargs = dict(entry.get("options", {}))
    if "operation" in kwargs:
        kwargs["operation"] = getattr(sm.BooleanOp, kwargs["operation"])
    return getattr(sm, entry["function"])(a.handle, b.handle, **kwargs)


def apply_boolean_operation(objects: list[ActiveObject], selected_ids: list[int],
                            operation: str) -> tuple[list[ActiveObject], ActiveObject, str]:
    """Consume two selected BReps and replace them with the boolean result."""
    obj_by_id = {obj.object_id: obj for obj in objects if obj.kind == "Brep"}
    selected = [obj_by_id[object_id] for object_id in selected_ids if object_id in obj_by_id]
    if len(selected) < 2:
        raise RuntimeError("Select at least two BRep objects for a boolean operation")
    a, b = selected[:2]

    result_handle = _run_boolean(operation, a, b)
    next_id = max((obj.object_id for obj in objects), default=0) + 1
    label = BOOLEAN_LABELS[operation].lower()
    result_name = f"{label}_{a.name}_{b.name}".replace(" ", "_")
    result = ActiveObject(
        object_id=next_id,
        name=result_name,
        kind="Brep",
        handle=result_handle,
        color=OBJECT_COLORS[(next_id - 1) % len(OBJECT_COLORS)],
        selected=True,
    )

    consumed_ids = {a.object_id, b.object_id}
    insert_index = min(i for i, obj in enumerate(objects) if obj.object_id in consumed_ids)
    kept = [obj for obj in objects if obj.object_id not in consumed_ids]
    kept.insert(insert_index, result)
    source_line = boolean_source_line(operation, result_name, a.name, b.name)
    return kept, result, source_line
