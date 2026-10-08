#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression test for the SMLib -> USD -> SMLib -> USD round-trip.

Complements ``roundtrip_smlib_test.py`` by diffing at the *USD* side
instead of the SMLib side. Steps:

  1. Build the BRep in SMLib (fixture).
  2. Export to USDA                       (U1.usda, "before").
  3. Import back to SMLib ``heal=False``.
  4. Export that BRep to USDA again       (U2.usda, "after").
  5. Text-diff U1.usda vs U2.usda:
     - full unified diff written to disk
     - per-prim-scope change counts (tracked via ``def <Type> "<name>"``
       blocks)
     - per-attribute-name change counts for every ``uniform ...`` line
       that changed
     - classification into known-loss vs unexpected

Exit code:

  0  -> U1 and U2 are byte-identical, or all differences are in
        ``KNOWN_LOSSES`` below.
  1  -> an unexpected USD round-trip difference was found.

Usage::

    _build\\target-deps\\python\\python.exe tools\\scripts\\roundtrip_usd_test.py

Options: same as ``roundtrip_smlib_test.py`` (``--out-dir``,
``--geometry``, ``--quiet``, ``--keep``).
"""

import argparse
import difflib
import os
import re
import sys


# ---------------------------------------------------------------------------
# Runtime environment boilerplate (see .agents/docs/generate_asset.md)
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO        = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", ".."))
_TOOLS      = os.path.join(REPO, "tools", "scripts")

if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

import view_usd                                        # noqa: E402
view_usd._ensure_usd_runtime_env()

_CONFIG    = view_usd._pick_config(REPO)
_PLATFORM  = view_usd._pick_platform()
_BUILD_BIN = os.path.join(REPO, "_build", _PLATFORM, _CONFIG)
if os.path.isdir(_BUILD_BIN) and _BUILD_BIN not in sys.path:
    sys.path.insert(0, _BUILD_BIN)

import _omni_solid as sm                               # noqa: E402
from pxr import Usd                                    # noqa: E402


# ---------------------------------------------------------------------------
# Fixtures (kept in sync with roundtrip_smlib_test.py).
# ---------------------------------------------------------------------------

def _filleted_box():
    box = sm.create_box((-7, -5, 6), 14, 10, 16)
    sm.fillet_edges(box, box.edges(), radius=2.0)
    return box


def _plain_box():
    return sm.create_box((0, 0, 0), 10, 10, 10)


def _sphere():
    return sm.create_sphere((0, 0, 0), 5.0)


def _torus():
    return sm.create_torus((0, 0, 0), 5.0, 1.0)


FIXTURES = {
    "filleted_box": _filleted_box,
    "plain_box":    _plain_box,
    "sphere":       _sphere,
    "torus":        _torus,
}


# ---------------------------------------------------------------------------
# Known-loss attribute names: attribute-name -> reason-string.
#
# Empty on purpose -- this test surfaces every real USD round-trip
# mismatch. Promote an entry here once you have root-caused and accepted
# a particular loss.
# ---------------------------------------------------------------------------

KNOWN_LOSSES = {
    # Example (leave commented out):
    # "brep:edge3dNurb:curve3d:nurb:weights":
    #     "floating-point noise in NURBS weight reconstruction",
}


# ---------------------------------------------------------------------------
# USDA line classification
# ---------------------------------------------------------------------------

# ``uniform <type> <attr:name> = ...`` or ``<type> <attr:name> = ...``
_ATTR_RE = re.compile(
    r"^\s*(?:uniform\s+|custom\s+|varying\s+)*"       # optional qualifiers
    r"\S+\s+"                                           # value type token
    r"([A-Za-z_][\w:]*)"                                # attribute name
    r"\s*(?:=|\.)")

# ``def <Type> "<name>"`` possibly followed by "(" or "{" on next lines
_DEF_RE  = re.compile(r'^\s*def\s+(\S+)\s+"([^"]+)"')


def _attribute_name(line):
    m = _ATTR_RE.match(line)
    return m.group(1) if m else None


def _prim_scope_of_each_line(lines):
    """Return a list where ``scopes[i]`` is the slash-joined name stack
    of the prim that encloses ``lines[i]``.

    Brace depth is tracked by counting ``{`` / ``}`` outside of string
    literals (a pragmatic approximation that works on USDA that SMLib's
    writer produces: no multi-line quoted strings with embedded braces).
    """
    scopes = []
    name_stack = []          # list of (name, depth_at_which_pushed)
    depth = 0
    pending_def = None       # name that a subsequent '{' will push
    for line in lines:
        # Record scope BEFORE processing braces on this line so the
        # ``def`` line itself belongs to the parent scope.
        scopes.append("/".join(n for n, _ in name_stack) or "<root>")

        md = _DEF_RE.search(line)
        if md:
            pending_def = md.group(2)

        # Count braces outside of quotes
        in_quote = False
        for ch in line:
            if ch == '"':
                in_quote = not in_quote
                continue
            if in_quote:
                continue
            if ch == "{":
                if pending_def is not None:
                    name_stack.append((pending_def, depth))
                    pending_def = None
                depth += 1
            elif ch == "}":
                depth -= 1
                # pop any name that was scoped at this depth
                while name_stack and name_stack[-1][1] >= depth:
                    name_stack.pop()
        # An unconsumed 'def' (no '{' on same line) stays pending for
        # future lines -- common in USDA pretty-printing.
    return scopes


# ---------------------------------------------------------------------------
# Round-trip
# ---------------------------------------------------------------------------

def run_roundtrip(brep_fn, out_dir, label):
    u1 = os.path.join(out_dir, f"{label}.U1.usda")
    u2 = os.path.join(out_dir, f"{label}.U2.usda")

    print("[1/5] building BRep via fixture", flush=True)
    a_brep = brep_fn()
    af, ae, av = len(a_brep.faces()), len(a_brep.edges()), len(a_brep.vertices())
    print(f"      F/E/V (A) = {af}/{ae}/{av}", flush=True)

    print(f"[2/5] USD export #1 -> {u1}", flush=True)
    if os.path.exists(u1):
        os.remove(u1)
    sm.usd.export_brep(a_brep, u1)
    sz_u1 = os.path.getsize(u1)
    print(f"      wrote {sz_u1} bytes", flush=True)

    print("[3/5] USD import #1 (heal=False)", flush=True)
    stage = Usd.Stage.Open(u1)
    brep_paths = [p.GetPath().pathString for p in stage.Traverse()
                  if p.GetTypeName() == "BrepArray"]
    if len(brep_paths) != 1:
        raise RuntimeError(
            f"Expected exactly 1 BrepArray prim, got {brep_paths}")
    b_brep = sm.usd.import_brep(u1, brep_paths[0], heal=False)
    bf, be, bv = len(b_brep.faces()), len(b_brep.edges()), len(b_brep.vertices())
    match = (af, ae, av) == (bf, be, bv)
    print(f"      F/E/V (B) = {bf}/{be}/{bv}  (match: {match})", flush=True)

    print(f"[4/5] USD export #2 -> {u2}", flush=True)
    if os.path.exists(u2):
        os.remove(u2)
    sm.usd.export_brep(b_brep, u2)
    sz_u2 = os.path.getsize(u2)
    print(f"      wrote {sz_u2} bytes", flush=True)

    print("[5/5] USD files ready for comparison", flush=True)

    return u1, u2, dict(
        a=dict(faces=af, edges=ae, vertices=av, size=sz_u1),
        b=dict(faces=bf, edges=be, vertices=bv, size=sz_u2))


# ---------------------------------------------------------------------------
# Diff and report
# ---------------------------------------------------------------------------

def _full_unified_diff(a_path, b_path):
    with open(a_path, encoding="utf-8", errors="replace") as f:
        a_lines = f.readlines()
    with open(b_path, encoding="utf-8", errors="replace") as f:
        b_lines = f.readlines()
    udiff = list(difflib.unified_diff(
        a_lines, b_lines,
        fromfile=os.path.basename(a_path),
        tofile  =os.path.basename(b_path),
        n=0))
    return udiff, a_lines, b_lines


def _classify_line_diffs(a_lines, b_lines):
    """Classify every actually-changed line between A and B, tagged with
    its prim scope and (where it exists) the attribute name on that
    line.

    Uses ``difflib.SequenceMatcher.get_opcodes`` so that an inserted or
    deleted line shifts the alignment instead of producing spurious
    mismatches for every following line.

    Returns:
      per_scope_counts     : {scope: count}
      per_attribute_counts : {attribute_name: count}
      samples              : list of
          (scope, attr, a_line, b_line, a_lineno, b_lineno, kind)
        where ``kind`` is 'replace', 'delete', or 'insert' and
        ``a_line``/``b_line`` may be '' for insert/delete respectively.
    """
    a_scope = _prim_scope_of_each_line(a_lines)
    b_scope = _prim_scope_of_each_line(b_lines)

    per_scope = {}
    per_attr  = {}
    samples   = []

    def record(scope_a, scope_b, al, bl, a_idx, b_idx, kind):
        scope_label = scope_a if scope_a == scope_b else \
                      (scope_a or scope_b)
        per_scope[scope_label] = per_scope.get(scope_label, 0) + 1
        attr = _attribute_name(al or "") or _attribute_name(bl or "")
        if attr:
            per_attr[attr] = per_attr.get(attr, 0) + 1
            samples.append((
                scope_label, attr,
                (al or "").rstrip("\n"),
                (bl or "").rstrip("\n"),
                (a_idx + 1) if a_idx is not None else None,
                (b_idx + 1) if b_idx is not None else None,
                kind))

    matcher = difflib.SequenceMatcher(a=a_lines, b=b_lines, autojunk=False)
    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == "equal":
            continue
        if tag == "replace":
            k = max(i2 - i1, j2 - j1)
            for di in range(k):
                ai = i1 + di if di < (i2 - i1) else None
                bj = j1 + di if di < (j2 - j1) else None
                al = a_lines[ai] if ai is not None else ""
                bl = b_lines[bj] if bj is not None else ""
                sa = a_scope[ai] if ai is not None else None
                sb = b_scope[bj] if bj is not None else None
                sub = "replace" if (ai is not None and bj is not None) \
                      else ("delete" if ai is not None else "insert")
                record(sa, sb, al, bl, ai, bj, sub)
        elif tag == "delete":
            for ai in range(i1, i2):
                record(a_scope[ai], None, a_lines[ai], "", ai, None, "delete")
        elif tag == "insert":
            for bj in range(j1, j2):
                record(None, b_scope[bj], "", b_lines[bj], None, bj, "insert")

    return per_scope, per_attr, samples


# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------

def _maybe_cleanup_artifacts(args, default_out_dir, paths):
    """Remove ``paths`` (and ``args.out_dir`` if it ends up empty) when:

      * the user did NOT pass ``--keep``, and
      * the run targeted the default out-dir (we never delete from a
        user-specified ``--out-dir``).

    Called only on a passing run; failing runs always keep the artifacts
    so the .usd.diff is available for inspection.
    """
    if args.keep:
        return
    out_dir_abs = os.path.abspath(args.out_dir)
    default_abs = os.path.abspath(default_out_dir)
    if out_dir_abs != default_abs:
        return  # respect user-chosen --out-dir; never touch it

    removed = 0
    for p in paths:
        try:
            if p and os.path.isfile(p):
                os.remove(p)
                removed += 1
        except OSError as e:
            print(f"  cleanup: could not remove {p}: {e}")
    try:
        if os.path.isdir(out_dir_abs) and not os.listdir(out_dir_abs):
            os.rmdir(out_dir_abs)
    except OSError:
        pass

    print()
    print(f"Cleaned up {removed} artifact(s) from {default_abs} "
          "(use --keep to retain).")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    default_out_dir = os.path.join(REPO, "_roundtrip_output")
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out-dir",
                        default=default_out_dir,
                        help="Where to place U1.usda / U2.usda / .diff artifacts.")
    parser.add_argument("--geometry", choices=sorted(FIXTURES.keys()),
                        default="filleted_box",
                        help="Fixture to round-trip (default: filleted_box).")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress per-line diff samples; summary only.")
    parser.add_argument("--keep", action="store_true",
                        help="Retain generated artifacts (U1.usda / U2.usda "
                             "/ .usd.diff) for inspection.  Without this "
                             "flag, PASS runs that wrote into the default "
                             "out-dir auto-clean their artifacts.  FAIL "
                             "runs and runs with a custom --out-dir always "
                             "retain.")
    args = parser.parse_args()

    print(f"[env] config={_CONFIG!r}  build={_BUILD_BIN}", flush=True)
    print(f"[env] fixture={args.geometry!r}  out_dir={args.out_dir}",
          flush=True)

    os.makedirs(args.out_dir, exist_ok=True)

    u1, u2, counts = run_roundtrip(
        FIXTURES[args.geometry], args.out_dir, label=args.geometry)

    print()
    print("Computing unified diff...", flush=True)
    udiff, a_lines, b_lines = _full_unified_diff(u1, u2)
    diff_path = os.path.join(args.out_dir, f"{args.geometry}.usd.diff")
    with open(diff_path, "w", encoding="utf-8") as f:
        f.write("".join(udiff))
    diff_line_count = sum(1 for ln in udiff
                          if ln.startswith(("+", "-"))
                          and not ln.startswith(("+++", "---")))
    print(f"  unified diff lines : {diff_line_count}")
    print(f"  full diff file     : {diff_path}")

    per_scope, per_attr, samples = _classify_line_diffs(a_lines, b_lines)

    print()
    print("Differences by prim scope:")
    if not per_scope:
        print("  (none -- files are byte-identical)")
    else:
        for scope in sorted(per_scope.keys()):
            print(f"  {scope:30s}  {per_scope[scope]} line(s) differ")

    print()
    print("Differences by attribute name:")
    if not per_attr:
        print("  (no attribute-level changes)")
    else:
        for name in sorted(per_attr.keys(),
                           key=lambda k: (-per_attr[k], k)):
            tag = " [KNOWN-LOSS]" if name in KNOWN_LOSSES else ""
            print(f"  {per_attr[name]:4d}  {name}{tag}")

    # Classification
    unexpected_attrs = {k: v for k, v in per_attr.items()
                        if k not in KNOWN_LOSSES}
    known_attrs      = {k: v for k, v in per_attr.items()
                        if k in KNOWN_LOSSES}

    # Scope-only differences (e.g. whitespace/comment changes with no
    # matching attribute line) are counted as unexpected unless the
    # scope has zero attribute-level changes but does have per_scope
    # mismatches.
    scope_only_mismatches = sum(per_scope.values()) - sum(per_attr.values())

    print()
    print("Classification:")
    print(f"  known-loss attribute diffs   : {sum(known_attrs.values())}"
          f" (across {len(known_attrs)} attributes)")
    print(f"  unexpected attribute diffs   : {sum(unexpected_attrs.values())}"
          f" (across {len(unexpected_attrs)} attributes)")
    print(f"  non-attribute line diffs     : {scope_only_mismatches}")

    if not args.quiet and samples:
        print()
        print("Sample mismatches (first 10 unique attributes):")
        shown = 0
        seen_attrs = set()
        for (scope, attr, al, bl, a_ln, b_ln, kind) in samples:
            if attr in seen_attrs:
                continue
            seen_attrs.add(attr)
            loc = []
            if a_ln is not None:
                loc.append(f"A:{a_ln}")
            if b_ln is not None:
                loc.append(f"B:{b_ln}")
            print(f"  [{kind}] {attr}  in {scope}  ({', '.join(loc)})")
            if al:
                print(f"    A: {al[:160]}{'...' if len(al) > 160 else ''}")
            if bl:
                print(f"    B: {bl[:160]}{'...' if len(bl) > 160 else ''}")
            shown += 1
            if shown >= 10:
                break
        if len(seen_attrs) < len(per_attr):
            print(f"  ... ({len(per_attr) - len(seen_attrs)} more attribute(s) differ)")

    # Summary
    print()
    f_match = counts['a']['faces']    == counts['b']['faces']
    e_match = counts['a']['edges']    == counts['b']['edges']
    v_match = counts['a']['vertices'] == counts['b']['vertices']
    print(f"Topology preserved through intermediate SMLib: F={f_match} E={e_match} V={v_match}")
    print(f"USD file sizes U1/U2 = {counts['a']['size']} / {counts['b']['size']} bytes")

    unexpected_total = sum(unexpected_attrs.values()) + scope_only_mismatches
    failed = unexpected_total > 0 or not (f_match and e_match and v_match)

    if not failed:
        _maybe_cleanup_artifacts(
            args, default_out_dir, [u1, u2, diff_path])

    if failed:
        print()
        print("RESULT: FAIL -- unexpected USD round-trip differences detected.")
        return 1
    if known_attrs:
        print()
        print("RESULT: PASS (with known, documented USD round-trip losses).")
    else:
        print()
        print("RESULT: PASS -- USD round-trip is byte-identical.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
