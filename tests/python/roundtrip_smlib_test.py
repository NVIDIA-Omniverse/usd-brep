#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression test for the SMLib -> USD BrepArray -> SMLib round-trip.

Exercises a filleted box (26 faces / 48 edges / 24 vertices) by:

  1. Building the BRep in SMLib.
  2. Dumping an ASCII ``.sm`` file  (A: "before").
  3. Exporting the BRep to a USDA via ``sm.usd.export_brep``.
  4. Re-importing via ``sm.usd.import_brep(..., heal=False)``.
  5. Dumping a second ASCII ``.sm`` file  (B: "after").
  6. Comparing A vs B: every line, every section, with specific
     attention to Face / Edge / Vertex attribute columns.

The script reports *all* differences, classifies each differing line by
which `.sm` section it belongs to, and separates known-loss differences
(per-face ``Tol`` column -- the surface-approximation tolerance that
``BrepArray`` has no attribute to carry) from unexpected ones.

Exit code:

  0  -> no differences at all, or only known-loss differences (currently:
        per-face Tol column).
  1  -> unexpected differences were found (and printed).

Use as a local pre-commit / CI sanity check on any change to
``source/BREP_SM_USD`` (USD BrepArray import/export) or to the SMLib
kernel that might perturb persisted BRep attributes.

Usage::

    _build\\target-deps\\python\\python.exe tools\\scripts\\roundtrip_smlib_test.py

Options::

    --out-dir <path>       Where to put the .sm / .usda / .diff artifacts.
                           Default: <repo>/_roundtrip_output
    --geometry <name>      Which fixture to run. Default: filleted_box
                           (see FIXTURES below).
    --keep                 Retain the per-fixture artifacts (.sm / .usda /
                           .diff) in the default out-dir after a successful
                           run.  Without ``--keep``, those files are deleted
                           on PASS to avoid littering ``_roundtrip_output/``.
                           Artifacts are always retained on FAIL (so the
                           diff is available for inspection) and always
                           retained when ``--out-dir`` was passed
                           explicitly.
    --quiet                Suppress per-line diff output; only print summary.
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
# Fixtures
# ---------------------------------------------------------------------------
#
# Small, fast SMLib BRep constructions that exercise specific persisted
# attributes. Add to this dict when a new piece of the .sm format needs
# round-trip coverage.

def _filleted_box():
    """Canonical fixture -- 26 faces, mix of planar + fillet surfaces."""
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
# .sm ASCII parsing
# ---------------------------------------------------------------------------
#
# Layout (from source/SMLib/src/SmBrepData.cpp). Sections are introduced
# either by a ``//<Name> Data`` line or by the header comment that names
# the per-row columns.

# Face row columns (version > 30):
#   Index, Flags, UserId1, UserId2, Tol, Surface, Outer Loop, # of Loops,
#   Naturally Trimmed
# Followed by a line: UMin, UMax, VMin, VMax
FACE_HDR   = ("//Index, Flags, UserId1, UserId2, Tol, Surface, Outer Loop, "
              "# of Loops, Naturally Trimmed")
FACE_UV    = "//UMin, UMax, VMin, VMax"

# Edge row columns:
#   Index, Flags, UserId1, UserId2, Tol, Curve, Start Vertex, End Vertex,
#   Primary Edgeuse, Param Min, Param Max
EDGE_HDR   = ("//Index, Flags, UserId1, UserId2, Tol, Curve, Start Vertex, "
              "End Vertex, Primary Edgeuse, Param Min, Param Max")

# Vertex row columns:
#   Index, Flags, UserId1, UserId2, Tol, X, Y, Z
VERTEX_HDR = "//Index, Flags, UserId1, UserId2, Tol, X, Y, Z"

FACE_COLS = ["Index", "Flags", "UserId1", "UserId2",
             "Tol", "Surface", "OuterLoop", "NumLoops", "NaturallyTrimmed"]
EDGE_COLS = ["Index", "Flags", "UserId1", "UserId2",
             "Tol", "Curve", "StartVertex", "EndVertex",
             "PrimaryEdgeuse", "ParamMin", "ParamMax"]
VERT_COLS = ["Index", "Flags", "UserId1", "UserId2", "Tol", "X", "Y", "Z"]

# Known-loss columns: (table_name, column_name) -> reason-string.
#
# Entries here are treated as *expected* round-trip differences and do
# not fail the run on their own. Start empty on purpose -- this test
# surfaces every real mismatch, and the caller / reviewer decides which
# ones to whitelist explicitly after understanding them.
KNOWN_LOSSES = {}


def _classify_section(lines):
    """Walk the full .sm file and tag each line with a section name.

    Returns a list of dicts: ``{index, raw, section, kind}`` where
    ``kind`` is one of ``header``, ``blank``, ``row``.

    Section boundaries are detected via the ``//<Name> Data`` comment
    lines emitted by ``SmBrepData::WriteToFile``. Some of those comments
    have a leading space after the ``//`` (e.g. ``// UV-Curve Data``),
    which is handled here by a simple ``endswith('Data')`` check.
    """
    out = []
    section = "<preamble>"
    for i, raw in enumerate(lines):
        stripped = raw.rstrip("\n")
        bare = stripped.strip()
        if not bare:
            kind = "blank"
        elif bare.startswith("//"):
            # A named section boundary.
            if bare.endswith(" Data") or bare.endswith("Data"):
                section = bare.lstrip("/ ").replace(" Data", "").strip()
            kind = "header"
        else:
            kind = "row"
        out.append(dict(index=i, raw=stripped, section=section, kind=kind))
    return out


def _all_floats(toks):
    for t in toks:
        try: float(t)
        except ValueError: return False
    return True


def _is_count_line(toks):
    """True for lines like ``26 Faces In Brep`` / ``48 Edges In Brep``."""
    if len(toks) < 3: return False
    try: int(toks[0])
    except ValueError: return False
    return toks[-1] == "Brep" and toks[-2].lower() == "in"


def _parse_table(classified, section_name, col_names, has_uv_continuation=False):
    """Extract each row of a named table as a dict of column-name -> token.

    ``has_uv_continuation`` is True for the Face table where each data row
    is immediately followed by a ``UMin UMax VMin VMax`` line (those are
    collected into a synthetic ``UVDomain`` column so they participate in
    the comparison).

    Count lines like ``26 Faces In Brep`` and header comments are
    skipped automatically.
    """
    rows = []
    pending_uv_for = None  # index in `rows` awaiting its UV continuation
    for entry in classified:
        if entry["section"] != section_name:
            pending_uv_for = None
            continue
        if entry["kind"] != "row":
            continue
        toks = entry["raw"].split()
        if _is_count_line(toks):
            continue

        # If we're expecting the UV continuation for the previous face
        # row, consume this line as UVDomain when it looks like one (4
        # floats).
        if has_uv_continuation and pending_uv_for is not None \
                and len(toks) == 4 and _all_floats(toks):
            rows[pending_uv_for]["UVDomain"] = " ".join(toks)
            pending_uv_for = None
            continue

        if len(toks) < len(col_names):
            continue
        try:
            int(toks[0])
        except ValueError:
            continue
        row = dict(zip(col_names, toks[:len(col_names)], strict=True))
        rows.append(row)
        if has_uv_continuation:
            pending_uv_for = len(rows) - 1
        else:
            pending_uv_for = None
    return rows


# ---------------------------------------------------------------------------
# Core round-trip
# ---------------------------------------------------------------------------

def dump_sm(brep, path):
    if os.path.exists(path):
        os.remove(path)
    brep.write_to_file(path, ascii=True)
    return os.path.getsize(path)


def run_roundtrip(brep_fn, out_dir, label):
    a_sm = os.path.join(out_dir, f"{label}.A.sm")
    b_sm = os.path.join(out_dir, f"{label}.B.sm")
    usda = os.path.join(out_dir, f"{label}.usda")

    print("[1/5] building BRep via fixture", flush=True)
    a_brep = brep_fn()
    af, ae, av = len(a_brep.faces()), len(a_brep.edges()), len(a_brep.vertices())
    print(f"      F/E/V (A) = {af}/{ae}/{av}", flush=True)

    print(f"[2/5] dumping .sm (A) -> {a_sm}", flush=True)
    sz_a = dump_sm(a_brep, a_sm)
    print(f"      wrote {sz_a} bytes", flush=True)

    print(f"[3/5] USD export -> {usda}", flush=True)
    if os.path.exists(usda):
        os.remove(usda)
    sm.usd.export_brep(a_brep, usda)

    print("[4/5] USD import (heal=False)", flush=True)
    stage = Usd.Stage.Open(usda)
    brep_paths = [p.GetPath().pathString for p in stage.Traverse()
                  if p.GetTypeName() == "BrepArray"]
    if len(brep_paths) != 1:
        raise RuntimeError(
            f"Expected exactly 1 BrepArray prim, got {brep_paths}")
    b_brep = sm.usd.import_brep(usda, brep_paths[0], heal=False)
    bf, be, bv = len(b_brep.faces()), len(b_brep.edges()), len(b_brep.vertices())
    match = (af, ae, av) == (bf, be, bv)
    print(f"      F/E/V (B) = {bf}/{be}/{bv}  (match: {match})", flush=True)

    print(f"[5/5] dumping .sm (B) -> {b_sm}", flush=True)
    sz_b = dump_sm(b_brep, b_sm)
    print(f"      wrote {sz_b} bytes", flush=True)

    return a_sm, b_sm, usda, dict(
        a=dict(faces=af, edges=ae, vertices=av, size=sz_a),
        b=dict(faces=bf, edges=be, vertices=bv, size=sz_b))


# ---------------------------------------------------------------------------
# Diff and report
# ---------------------------------------------------------------------------

def full_unified_diff(a_path, b_path):
    with open(a_path, encoding="utf-8", errors="replace") as f: a_lines = f.readlines()
    with open(b_path, encoding="utf-8", errors="replace") as f: b_lines = f.readlines()
    return list(difflib.unified_diff(
        a_lines, b_lines,
        fromfile=os.path.basename(a_path),
        tofile  =os.path.basename(b_path),
        n=0)), a_lines, b_lines


def diff_by_section(a_classified, b_classified):
    """Return {section_name: (a_changed_lines, b_changed_lines)} listing
    every line that differs between A and B, grouped by section.

    Alignment is by line index; for tables with equal row counts this
    works perfectly. If row counts differ, alignment degrades gracefully
    because `difflib` below will also flag a structural mismatch.
    """
    per_section = {}
    n = min(len(a_classified), len(b_classified))
    for i in range(n):
        a, b = a_classified[i], b_classified[i]
        if a["raw"] != b["raw"]:
            sec = a["section"] if a["section"] == b["section"] else \
                  f"{a['section']} / {b['section']}"
            per_section.setdefault(sec, []).append((i, a["raw"], b["raw"]))
    # Extra trailing lines on either side also count.
    for i in range(n, len(a_classified)):
        sec = a_classified[i]["section"]
        per_section.setdefault(sec, []).append((i, a_classified[i]["raw"], "<EOF>"))
    for i in range(n, len(b_classified)):
        sec = b_classified[i]["section"]
        per_section.setdefault(sec, []).append((i, "<EOF>", b_classified[i]["raw"]))
    return per_section


def diff_table_by_column(a_rows, b_rows, col_names, section_name):
    """Return a list of (row_index, col_name, a_value, b_value) mismatches.

    When row counts disagree, emits a structural entry for each missing
    row so the caller can show "present only in A" / "present only in B".
    """
    mismatches = []
    n = min(len(a_rows), len(b_rows))
    cols_with_uv = col_names + (["UVDomain"] if "UVDomain" in a_rows[0] else [])\
                   if a_rows else col_names
    for i in range(n):
        for c in cols_with_uv:
            av = a_rows[i].get(c, "<missing>")
            bv = b_rows[i].get(c, "<missing>")
            if av != bv:
                mismatches.append((i, c, av, bv))
    for i in range(n, len(a_rows)):
        mismatches.append((i, "<row>", str(a_rows[i]), "<missing in B>"))
    for i in range(n, len(b_rows)):
        mismatches.append((i, "<row>", "<missing in A>", str(b_rows[i])))
    return mismatches


# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------

def _maybe_cleanup_artifacts(args, default_out_dir, paths):
    """Remove ``paths`` (and ``args.out_dir`` if it ends up empty) when:

      * the user did NOT pass ``--keep``, and
      * the run targeted the default out-dir (we never delete from a
        user-specified ``--out-dir``).

    Called only on a passing run; failing runs always keep the artifacts
    so the .diff is available for inspection.
    """
    if args.keep:
        return
    out_dir_abs     = os.path.abspath(args.out_dir)
    default_abs     = os.path.abspath(default_out_dir)
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
    # Drop the default out-dir if it is now empty.
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
                        help="Where to place .sm / .usda / .diff artifacts.")
    parser.add_argument("--geometry", choices=sorted(FIXTURES.keys()),
                        default="filleted_box",
                        help="Fixture to round-trip (default: filleted_box).")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress per-line diff output; summary only.")
    parser.add_argument("--keep", action="store_true",
                        help="Retain generated artifacts (.sm / .usda / "
                             ".diff) for inspection.  Without this flag, "
                             "PASS runs that wrote into the default "
                             "out-dir auto-clean their artifacts.  FAIL "
                             "runs and runs with a custom --out-dir always "
                             "retain.")
    args = parser.parse_args()

    print(f"[env] config={_CONFIG!r}  build={_BUILD_BIN}", flush=True)
    print(f"[env] fixture={args.geometry!r}  out_dir={args.out_dir}",
          flush=True)

    os.makedirs(args.out_dir, exist_ok=True)
    fixture_fn = FIXTURES[args.geometry]

    a_sm, b_sm, usda, counts = run_roundtrip(
        fixture_fn, args.out_dir, label=args.geometry)

    # Full unified diff to a file for the user to open in an editor.
    print()
    print("Computing unified diff...", flush=True)
    udiff, a_lines, b_lines = full_unified_diff(a_sm, b_sm)
    diff_path = os.path.join(args.out_dir, f"{args.geometry}.diff")
    with open(diff_path, "w", encoding="utf-8") as f:
        f.write("".join(udiff))
    diff_line_count = sum(1 for ln in udiff
                          if ln.startswith(("+", "-"))
                          and not ln.startswith(("+++", "---")))
    print(f"  changed lines (unified): {diff_line_count}")
    print(f"  full diff              : {diff_path}")

    # Classify every line of A and B by section, then produce a section map.
    a_cls = _classify_section(a_lines)
    b_cls = _classify_section(b_lines)
    per_section = diff_by_section(a_cls, b_cls)

    print()
    print("Differences by .sm section:")
    if not per_section:
        print("  (none -- files are byte-identical)")
    else:
        for sec in sorted(per_section.keys()):
            print(f"  {sec:30s}  {len(per_section[sec])} line(s) differ")

    # Detailed column-by-column comparison for Face / Edge / Vertex tables.
    a_faces = _parse_table(a_cls, "Face",   FACE_COLS, has_uv_continuation=True)
    b_faces = _parse_table(b_cls, "Face",   FACE_COLS, has_uv_continuation=True)
    a_edges = _parse_table(a_cls, "Edge",   EDGE_COLS)
    b_edges = _parse_table(b_cls, "Edge",   EDGE_COLS)
    a_verts = _parse_table(a_cls, "Vertex", VERT_COLS)
    b_verts = _parse_table(b_cls, "Vertex", VERT_COLS)

    table_rowcounts = {
        "Face":   (len(a_faces), len(b_faces)),
        "Edge":   (len(a_edges), len(b_edges)),
        "Vertex": (len(a_verts), len(b_verts)),
    }
    table_mismatches = {
        "Face":   diff_table_by_column(a_faces, b_faces, FACE_COLS, "Face"),
        "Edge":   diff_table_by_column(a_edges, b_edges, EDGE_COLS, "Edge"),
        "Vertex": diff_table_by_column(a_verts, b_verts, VERT_COLS, "Vertex"),
    }

    print()
    print("Column-by-column differences on persisted tables:")
    for table, items in table_mismatches.items():
        na, nb = table_rowcounts[table]
        rowtxt = f"{na} rows" if na == nb else f"{na} vs {nb} rows"
        if not items:
            print(f"  {table:6s} ({rowtxt}): clean")
            continue
        per_col = {}
        for (_, col, _, _) in items:
            per_col[col] = per_col.get(col, 0) + 1
        cols_line = ", ".join(f"{c}={n}" for c, n in sorted(per_col.items()))
        print(f"  {table:6s} ({rowtxt}): {len(items)} mismatches across columns [{cols_line}]")

    # Classify known-loss vs unexpected.
    unexpected = []
    known      = []
    for table, items in table_mismatches.items():
        for (row, col, av, bv) in items:
            tag = (table, col)
            if tag in KNOWN_LOSSES:
                known.append((table, row, col, av, bv))
            else:
                unexpected.append((table, row, col, av, bv))

    print()
    print("Classification:")
    print(f"  known-loss diffs   : {len(known)}")
    print(f"  unexpected diffs   : {len(unexpected)}")
    if known and not args.quiet:
        print("  known-loss details:")
        seen = set()
        for (table, row, col, av, bv) in known[:5]:
            if (table, col) in seen: continue
            seen.add((table, col))
            reason = KNOWN_LOSSES[(table, col)]
            print(f"    {table}.{col}: {reason}")
            print(f"    e.g. row {row}: A={av} vs B={bv}")
    if unexpected and not args.quiet:
        print("  unexpected details (first 20):")
        for (table, row, col, av, bv) in unexpected[:20]:
            print(f"    {table}[row {row}].{col}: A={av!r}  B={bv!r}")
        if len(unexpected) > 20:
            print(f"    ... ({len(unexpected) - 20} more)")

    # Summary
    print()
    f_match = counts['a']['faces']    == counts['b']['faces']
    e_match = counts['a']['edges']    == counts['b']['edges']
    v_match = counts['a']['vertices'] == counts['b']['vertices']
    print(f"Topology counts preserved: F={f_match} E={e_match} V={v_match}")
    print(f"File sizes A/B = {counts['a']['size']} / {counts['b']['size']} bytes")

    failed = bool(unexpected) or not (f_match and e_match and v_match)

    if not failed:
        _maybe_cleanup_artifacts(
            args, default_out_dir, [a_sm, b_sm, usda, diff_path])

    # Exit code
    if failed:
        print()
        print("RESULT: FAIL -- unexpected round-trip differences detected.")
        return 1
    if known:
        print()
        print("RESULT: PASS (with known, documented per-face Tol losses).")
    else:
        print()
        print("RESULT: PASS -- round-trip is byte-identical.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
