# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import re
import traceback

from .model import ActiveObject, make_active_objects, smlib_object_kind
from .runtime import sm, smdev

CONSUMING_OPS = {
    "boolean_union",
    "boolean_difference",
    "boolean_intersection",
    "boolean_merge",
    "boolean",
    "boolean_with_options",
    "boolean_with_curves",
    "boolean_2d",
    "merge_breps",
    "evaluate_csg_tree",
    "non_manifold_boolean",
    "piecewise_merge",
    "boolean_lists",
}


def _split_args(s: str) -> list[str]:
    """Split a comma-separated arg string, respecting parens and brackets."""
    parts = []
    depth = 0
    current = []
    in_quote = False
    quote_char = ""
    escaped = False
    i = 0
    while i < len(s):
        ch = s[i]
        if in_quote:
            if escaped:
                current.append(ch)
                escaped = False
            elif ch == "\\":
                current.append(ch)
                escaped = True
            elif len(quote_char) == 3 and s.startswith(quote_char, i):
                current.append(quote_char)
                in_quote = False
                i += len(quote_char)
                continue
            elif quote_char == ch:
                current.append(ch)
                in_quote = False
            else:
                current.append(ch)
            i += 1
            continue
        if ch in ("'", '"'):
            in_quote = True
            quote_char = ch * 3 if s.startswith(ch * 3, i) else ch
            current.append(quote_char)
            i += len(quote_char)
            continue
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(current))
            current = []
        else:
            current.append(ch)
        i += 1
    if current:
        parts.append("".join(current))
    return parts


def consumed_variable_names(source: str) -> set[str]:
    """Find simple variable names consumed by operations that invalidate inputs."""
    consumed_names: set[str] = set()
    consuming_call_re = re.compile(r"sm\.(\w+)\(")
    pos = 0
    while pos < len(source):
        m = consuming_call_re.search(source, pos)
        if not m:
            break
        if m.group(1) not in CONSUMING_OPS:
            pos = m.end()
            continue
        args_start = m.end()
        depth = 1
        i = args_start
        while i < len(source) and depth > 0:
            if source[i] == "(":
                depth += 1
            elif source[i] == ")":
                depth -= 1
            i += 1
        args_str = source[args_start : i - 1]
        for part in _split_args(args_str):
            tok = part.strip()
            if re.match(r"^[a-zA-Z_]\w*$", tok):
                consumed_names.add(tok)
        pos = i
    return consumed_names


def execute_script(source: str) -> tuple[list[ActiveObject], str]:
    """Execute source as _omni_solid commands and build active objects."""
    if sm is None:
        return [], (
            "_omni_solid module not found. Build the repo first:\n"
            "  ./repo.sh build\n"
            "The GUI will display once the kernel is available."
        )
    # WARNING: exec() runs arbitrary Python with full interpreter access.
    # CPython cannot be meaningfully sandboxed; restricting __builtins__ is
    # trivially bypassed. Only run scripts you trust.
    namespace = {"sm": sm, "__builtins__": __builtins__}
    if smdev is not None:
        namespace["smdev"] = smdev
    try:
        exec(source, namespace)
    except Exception:
        return [], traceback.format_exc()

    consumed_names = consumed_variable_names(source)
    seen_ids: dict[int, str] = {}
    candidates: list[tuple[str, object, str]] = []
    reserved_names = {"sm", "smdev"}
    for name, val in namespace.items():
        if name.startswith("_") or name in reserved_names:
            continue
        kind = smlib_object_kind(val)
        if not kind:
            continue
        oid = id(val)
        if oid in seen_ids:
            old_name = seen_ids[oid]
            candidates = [(n, v, k) for n, v, k in candidates if n != old_name]
        seen_ids[oid] = name
        if name not in consumed_names:
            candidates.append((name, val, kind))

    if not candidates:
        unique_objects = {}
        for name, val in namespace.items():
            if name.startswith("_") or name in reserved_names:
                continue
            kind = smlib_object_kind(val)
            if kind:
                unique_objects[id(val)] = (name, val, kind)
        candidates = list(unique_objects.values())

    objects = make_active_objects([(name, val) for name, val, _kind in candidates])
    return objects, ""
