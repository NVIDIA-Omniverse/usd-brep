<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SmPyLib Docstring Style

Conventions for Python-binding docstrings in `source/SmPyLib/src/SmPy*.cpp`.
The goal is consistency across ~20 binding files so Sphinx renders cleanly,
search ranks predictably, and reviewers don't re-litigate wording per PR.

The module-level docstring (in `SmPyMain.cpp`) owns the global conventions
(angles in degrees, Z-up, `RuntimeError` policy, kernel-owned object
lifetime). Per-function docstrings should not repeat them.

---

## Section order

Google-style sections, in this order. Omit any section that has no content.

```text
Summary line (one sentence, imperative mood).

Args:
    name: Description.

Returns:
    Type: description.

Notes:
    Free-form prose. Use for caveats, stability, and behavior that is
    surprising relative to the function name.

See Also:
    other_function: One-line reason to look there.

Wraps: SmApi<C function name>
```

- Summary: one sentence, imperative ("Create a sphere", not "Creates a sphere").
- One blank line between sections.
- No trailing blank line in the docstring.
- `Wraps:` is the last line, no trailing newline.

## Type names in prose

Use the **Python class name** (the second arg to `py::class_<...>(m, "Name")`),
NOT the C++ class name. Sphinx's `autodoc` + `napoleon` will only resolve the
Python name.

| Use in docstrings | Never use |
|---|---|
| `Brep` | `SmBrep` |
| `PolyBrep` | `SmPolyBrep` |
| `Face`, `Edge`, `Vertex` | `SmFace`, `SmEdge`, `SmVertex` |
| `Curve`, `Surface`, `Edgeuse` | `SmCurve`, `SmSurface`, `SmEdgeuse` |

The `Sm` prefix appears in exactly one place: the `Wraps: SmApi*` footer,
which intentionally points at the C function.

## Numeric validation vocabulary

Use the exact phrasing in the right column. Pick by actual kernel semantics,
not by what feels safe.

| Phrase | Meaning | Typical use |
|---|---|---|
| `Must be positive.` | strictly &gt; 0 | dimensions, radii, distances, scale |
| `Must be non-negative.` | &ge; 0 | radii where 0 produces a valid degenerate result (e.g. a cone with `radius_top = 0` collapsing to a tip) |
| `Must be greater than X.` | strictly &gt; X | end-angle vs start-angle, upper bound vs lower bound |

If the kernel accepts 0 but produces a degenerate shape, prefer
`Must be non-negative.` and add a one-line `Notes:` entry describing what 0
gives you (e.g. *"`radius_top = 0` produces a sharp apex (true cone)."*).

## Units and coordinate frame

The module-level docstring fixes the global conventions: angles in degrees,
Z up, sweeps measured in the XY plane from the X axis, distances in modeling
units. **Do not repeat this in every function.**

Mention units per-arg only when:

- the argument deviates (e.g. an arg whose name ends in `_rad`), OR
- the argument is angular and the sweep axis or reference direction is
  non-obvious (then state it explicitly in `Args:` or `Notes:`).

## `Raises:` blocks

Omitted per function. Every binding goes through `CHECK_STATUS`, so every
binding raises `RuntimeError` on a non-success kernel status, with a
message of the form
``<SmApi*> failed: <SM_ERR_NAME> (<code>). Kernel trace: ...``. The
module-level docstring states this once and points at
`.agents/docs/errors.md` for per-code guidance, so per-function
`Raises:` blocks are pure noise.

The exception: if a binding does additional Python-side validation that
raises a *different* exception type (e.g. `ValueError`, `TypeError` from
arity checks), document that one.

## `See Also:`

One line per entry, format: `name: One-line reason.` Restrict to functions
the reader is likely to want next (variants, opposites, the non-partial
version of a partial). Don't dump the whole module index here.

## `Wraps:` footer

Always last, single line, no period:

```text
Wraps: SmApiCreateBox
```

For overloads or multi-step bridges, list them comma-separated:

```text
Wraps: SmApiCreateOffsetProfile, SmApiOffsetCurve
```

## Inline formatting

- Double-backticks for code, literals, parameter values, and Python tuples:
  ``(x, y, z)``, ``True``, ``radius_top = 0``, ``"left"``.
- Italics (`*word*`) for emphasis, used sparingly.
- Don't link to URLs; this is a docstring, not a wiki page.

## Per-arg line wrapping

- One arg per leading line: `    name: First line of description.`
- Continuations indent one extra level (8 spaces total inside the C++
  string literal, so 4 spaces of Python indent after the leading 4):

```text
"    radius_top: Radius at the top (Z = ``height``). Must be non-negative.\n"
"        Use 0 for a true cone (sharp apex).\n"
```

## Worked example

```cpp
m.def("create_sphere", [](py::tuple origin, double radius) {
    SmVector3d o = to_vec(origin);
    SmBrep* r = nullptr;
    CHECK_STATUS(SmApiCreateSphere(o, radius, r));
    return r;
}, py::return_value_policy::reference,
   py::arg("origin"), py::arg("radius"),
   "Create a solid sphere with poles.\n"
   "\n"
   "Args:\n"
   "    origin: Center of the sphere as ``(x, y, z)``.\n"
   "    radius: Radius of the sphere. Must be positive.\n"
   "\n"
   "Returns:\n"
   "    Brep: closed manifold solid.\n"
   "\n"
   "Notes:\n"
   "    The standard NURBS sphere has degenerate poles at top and bottom.\n"
   "    Use ``create_sphere_no_pole`` if downstream operations are sensitive\n"
   "    to pole singularities.\n"
   "\n"
   "See Also:\n"
   "    create_sphere_no_pole: Sphere built without polar singularities.\n"
   "    create_partial_sphere: Azimuthal wedge of a sphere.\n"
   "\n"
   "Wraps: SmApiCreateSphere");
```

## Sourcing docstring content

Where to read each section from when documenting a new binding:

| Docstring section | Source |
|---|---|
| Summary, `Notes:` | `SmApi*.cpp` PURPOSE / NOTES blocks |
| `Args:` per-line | `SmApi*.h` `///< [in/out]:` comments next to each parameter |
| `Returns:` | `SmApi*.h` return / out-param comments |
| Design context | `.agents/operations/*.md` for the relevant category |
| `Wraps:` | the C function name being called inside the lambda |
