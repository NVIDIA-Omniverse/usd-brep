# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Prompt-to-source helpers for the SMLib GUI."""

from __future__ import annotations

import textwrap


DEFAULT_SCRIPT = textwrap.dedent("""\
    box = sm.create_box((-5, -5, 0), 10, 10, 10)
    sm.fillet_edges(box, box.edges(), radius=1.0)
    sph = sm.create_sphere((0, 0, 12), 2.0)
    result = sm.boolean_union(box, sph)
""")


def prompt_to_source(prompt: str) -> str:
    """Convert a natural-language prompt into sm.* Python code.

    Compositional: scans for all mentioned primitives, counts ("two",
    "three", ...), modifiers (fillet, round), and boolean operations
    (intersect, union, difference, subtract). A real deployment would
    use an LLM here.
    """
    p = prompt.lower()

    words_to_num = {
        "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
        "2": 2, "3": 3, "4": 4, "5": 5, "6": 6,
    }

    primitives = [
        ("box", ["box", "cube"]),
        ("sphere", ["sphere", "ball"]),
        ("cylinder", ["cylinder", "tube"]),
        ("cone", ["cone"]),
        ("torus", ["torus", "donut", "ring"]),
        ("pipe", ["pipe"]),
    ]

    prim_code = {
        "box": lambda var, org: f"{var} = sm.create_box({org}, 10, 10, 10)",
        "sphere": lambda var, org: f"{var} = sm.create_sphere({org}, 5.0)",
        "cylinder": lambda var, org: f"{var} = sm.create_cylinder({org}, 3.0, 10.0)",
        "cone": lambda var, org: f"{var} = sm.create_cone({org}, 5.0, 0.0, 10.0)",
        "torus": lambda var, org: f"{var} = sm.create_torus({org}, 5.0, 1.5)",
    }

    detected = []
    for prim_key, keywords in primitives:
        for kw in keywords:
            if kw in p:
                detected.append(prim_key)
                break

    if not detected:
        detected = ["box"]

    count = 1
    for word, num in words_to_num.items():
        if word in p:
            count = max(count, num)
            break

    if count > 1 and len(detected) == 1:
        detected = detected * count

    want_fillet = any(w in p for w in ["fillet", "round", "chamfer", "bevel"])
    want_intersect = any(w in p for w in ["intersect", "intersection"])
    want_subtract = any(w in p for w in ["subtract", "difference", "minus", "cut"])
    want_union = any(w in p for w in ["union", "merge", "combine", "join"])
    want_bool = want_intersect or want_subtract or want_union

    if len(detected) > 1 and not want_bool:
        if "corner" in p or "overlap" in p or "intersect" in p:
            want_intersect = True
            want_bool = True

    offsets = [
        (0, 0, 0), (7, 7, 7), (-7, -7, 0), (7, -7, 0),
        (-7, 7, 0), (0, 0, 14),
    ]

    lines = []
    var_names = []
    for i, prim_key in enumerate(detected):
        if prim_key == "pipe":
            var = f"pipe{i + 1}" if len(detected) > 1 else "pipe"
            lines.append(f"path{i + 1} = sm.create_arc((0, 0, 0), 10.0, 0.0, 180.0)")
            lines.append(f"{var} = sm.pipe_sweep(path{i + 1}, radius=1.5, cap_ends=True)")
            var_names.append(var)
            continue

        var = f"{prim_key}{i + 1}" if len(detected) > 1 else prim_key
        ox, oy, oz = offsets[i % len(offsets)]
        org = f"({ox}, {oy}, {oz})"
        code_fn = prim_code.get(prim_key)
        if code_fn:
            lines.append(code_fn(var, org))
        else:
            lines.append(f"{var} = sm.create_box({org}, 10, 10, 10)")
        if want_fillet and prim_key in ("box", "cylinder"):
            lines.append(f"sm.fillet_edges({var}, {var}.edges(), radius=1.0)")
        var_names.append(var)

    if want_bool and len(var_names) >= 2:
        if want_intersect:
            op = "boolean_intersection"
        elif want_subtract:
            op = "boolean_difference"
        else:
            op = "boolean_union"
        acc = var_names[0]
        for i in range(1, len(var_names)):
            result_var = "result" if i == len(var_names) - 1 else f"temp{i}"
            lines.append(f"{result_var} = sm.{op}({acc}, {var_names[i]})")
            acc = result_var

    return "\n".join(lines) + "\n"
