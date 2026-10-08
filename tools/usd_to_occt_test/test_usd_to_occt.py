# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for the standalone USD BrepArray -> OCCT `.brep` exporter (`usd_to_occt`).

There is no committed USD BrepArray corpus authored specifically for the exporter, so these
tests drive it through a **round-trip** seeded by the importer:

    .brep fixture --occt_to_usd--> A.usda --usd_to_occt--> out.brep --occt_to_usd--> B.usda

The exporter (`usd_to_occt`) is the step under test. Re-importing its output and validating the
resulting BrepArray catches exporter bugs that produce a malformed or unparseable `.brep`, or a
`.brep` whose geometry/topology no longer round-trips. Concretely each emitted `B.usda` is:

  * schema-validated with the same `BrepValidator` the importer suite uses (zero failed checks), and
  * checked against `A.usda` for exact region/shell/faceuse topology preservation, and
  * (committed corpus only) run through a SMLib `AssertValid` round-trip via
    ``usd_test_app --assert-valid``, in a subprocess so a crash fails the test rather than the
    runner.

Seeding from the importer keeps the suite self-contained (no hand-authored `.usda` inputs) and
reuses the known-good `.brep` corpus. A bug isolated to the importer can surface here too; when a
case fails, check the importer suite (`occt_to_usd_test`) first to localize it.

Two suites keep shareable and unshareable data isolated, mirroring the importer suite:

  * ``UsdToOcctCorpusRoundTripTestCase`` -- the self-authored corpus committed under
    ``TestFiles/occt_breps``. Carries no third-party data license.

  * ``UsdToOcctExternalRoundTripTestCase`` -- proprietary / third-party parts (e.g. OpenCASCADE
    distribution samples) kept in the git-ignored ``TestFiles/occt_breps_external`` directory (or
    wherever ``OCCT_BREP_EXTERNAL_DIR`` points). Skips when no such files are present.

The USD-runtime bootstrap and the `BrepValidator` are reused from the `brep_validator_test`
harness (its ``utils.base_test_case``).
"""

import atexit
import os
from collections import Counter
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

# Reuse the brep_validator test harness: it bootstraps the USD runtime, installs
# usd-validation-nvidia, and exposes BrepValidatorBaseTestCase (with _collect_issues).
_THIS_DIR = Path(__file__).resolve().parent
_VALIDATOR_TEST_DIR = _THIS_DIR.parent / "brep_validator_test"
if str(_VALIDATOR_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(_VALIDATOR_TEST_DIR))

from utils.base_test_case import REPO_ROOT, BrepValidatorBaseTestCase  # noqa: E402
from pxr import Usd  # noqa: E402


class _UsdToOcctRoundTripSummary:
    """Sample-level result accounting for the corpus test methods."""

    def __init__(self):
        self.successes = []
        self.failures = []
        self.skips = []
        self.suite_skips = []

    @staticmethod
    def _label(suite: str, sample: Path) -> str:
        return f"{suite}/{sample.name}"

    @staticmethod
    def _short(reason: object, limit: int = 500) -> str:
        text = str(reason).strip() or "(no reason reported)"
        return text if len(text) <= limit else text[: limit - 3] + "..."

    def record_success(self, suite: str, sample: Path):
        self.successes.append(self._label(suite, sample))

    def record_failure(self, suite: str, sample: Path, reason: object):
        self.failures.append((self._label(suite, sample), self._short(reason)))

    def record_skip(self, suite: str, sample: Path, reason: object):
        self.skips.append((self._label(suite, sample), self._short(reason)))

    def record_suite_skip(self, suite: str, reason: object):
        self.suite_skips.append((suite, self._short(reason)))

    def print(self):
        sample_count = len(self.successes) + len(self.failures) + len(self.skips)
        if sample_count == 0 and not self.suite_skips:
            return

        print("\nUSD BRep round-trip sample summary:", file=sys.stderr)
        print(f"  sample runs: {sample_count}", file=sys.stderr)
        print(f"  successful: {len(self.successes)}", file=sys.stderr)
        print(f"  failed: {len(self.failures)}", file=sys.stderr)
        print(f"  skipped: {len(self.skips)}", file=sys.stderr)

        if self.failures:
            print("  failure details:", file=sys.stderr)
            for label, reason in self.failures:
                print(f"    - {label}: {reason}", file=sys.stderr)

        if self.skips:
            print("  skip reasons:", file=sys.stderr)
            for reason, count in Counter(reason for _, reason in self.skips).items():
                print(f"    - {count} sample(s): {reason}", file=sys.stderr)

        if self.suite_skips:
            print("  suite skips:", file=sys.stderr)
            for suite, reason in self.suite_skips:
                print(f"    - {suite}: {reason}", file=sys.stderr)


_SUMMARY = _UsdToOcctRoundTripSummary()
atexit.register(_SUMMARY.print)


def _built_binary(name: str, env_override: str) -> Path:
    """Locate a built binary by name: env override, else the built path (release preferred)."""
    override = os.environ.get(env_override)
    if override:
        return Path(override)
    candidates = sorted((REPO_ROOT / "_build").glob(f"*/*/{name}"))
    release = [c for c in candidates if c.parent.name == "release"]
    chosen = release or candidates
    return chosen[0] if chosen else (REPO_ROOT / "_build" / "linux-x86_64" / "release" / name)


def _exporter_path() -> Path:
    """Locate the usd_to_occt CLI (the exporter under test)."""
    return _built_binary("usd_to_occt", "USD_TO_OCCT")


def _importer_path() -> Path:
    """Locate the occt_to_usd CLI (used to seed inputs and to re-import exporter output)."""
    return _built_binary("occt_to_usd", "OCCT_TO_USD")


def _usd_test_app_path() -> Path:
    """Locate the usd_test_app binary (provides the --assert-valid round-trip check)."""
    return _built_binary("usd_test_app", "USD_TEST_APP")


class _UsdToOcctRoundTripMixin(BrepValidatorBaseTestCase):
    """Shared import->export->re-import + validate machinery; suites supply the input directory."""

    inputs_dir: Path = None
    _topology_attributes = (
        "brep:regionCount",
        "region:type",
        "region:shellCount",
        "shell:faceuseCount",
        "faceuse:faceIndex",
        "faceuse:orientationType",
    )
    # Per-subprocess wall-clock cap so a hung binary fails the test instead of stalling the whole job.
    _timeout_sec = int(os.environ.get("OCCT_BREP_TEST_TIMEOUT_SEC", "300"))

    def setUp(self):
        self.exporter = _exporter_path()
        self.importer = _importer_path()
        if not self.exporter.is_file():
            self.skipTest(f"usd_to_occt not built at {self.exporter}")
        if not self.importer.is_file():
            self.skipTest(f"occt_to_usd not built at {self.importer}")
        self._tmp = tempfile.mkdtemp(prefix="usd_to_occt_test_")
        self.addCleanup(shutil.rmtree, self._tmp, ignore_errors=True)

    def _discover(self):
        if not self.inputs_dir or not self.inputs_dir.is_dir():
            return []
        return sorted(p for p in self.inputs_dir.rglob("*.brep") if p.is_file())

    def _run(self, args, what: str, src):
        result = subprocess.run(
            [str(a) for a in args],
            capture_output=True,
            text=True,
            env=os.environ,
            timeout=self._timeout_sec,
        )
        self.assertEqual(
            result.returncode, 0, msg=f"{what} failed for {src}: {result.stderr.strip()}"
        )
        return result

    def _import(self, brep_path: Path, out_name: str) -> Path:
        """Seed step: import a .brep to a USD BrepArray (occt_to_usd)."""
        out_path = Path(self._tmp) / out_name
        out_path.parent.mkdir(parents=True, exist_ok=True)
        self._run([self.importer, brep_path, out_path], "occt_to_usd (seed import)", brep_path)
        self.assertTrue(out_path.is_file(), msg=f"no seed USD produced for {brep_path}")
        return out_path

    def _export(self, usd_path: Path, out_name: str, src: Path) -> Path:
        """Step under test: export a USD BrepArray to OCCT .brep (usd_to_occt)."""
        out_path = Path(self._tmp) / out_name
        self._run([self.exporter, usd_path, out_path], "usd_to_occt (export)", src)
        self.assertTrue(out_path.is_file(), msg=f"usd_to_occt produced no .brep for {src}")
        self.assertGreater(out_path.stat().st_size, 0, msg=f"usd_to_occt wrote an empty .brep for {src}")
        return out_path

    def _validate_schema(self, out_path: Path, src: Path):
        """USD-level validation: run BrepValidator on the re-imported BrepArray."""
        issues = self._collect_issues(out_path)
        if issues:
            detail = "; ".join(
                f"{getattr(getattr(i, 'requirement', None), 'code', '?')}: {getattr(i, 'message', i)}" for i in issues
            )
            self.fail(f"BrepValidator reported {len(issues)} failed check(s) for {src.name}: {detail}")

    def _assert_valid(self, out_path: Path, src: Path):
        """SMLib-level validation: re-import the BrepArray to an SmBrep and run AssertValid."""
        app = _usd_test_app_path()
        if not app.is_file():
            self.skipTest(f"usd_test_app not built at {app}")
        result = subprocess.run(
            [str(app), "--assert-valid", str(out_path)],
            capture_output=True,
            text=True,
            env=os.environ,
            timeout=self._timeout_sec,
        )
        if result.returncode != 0:
            detail = (result.stdout.strip() + " " + result.stderr.strip()).strip()
            self.fail(f"AssertValid round-trip failed for {src.name} (rc={result.returncode}): {detail}")

    def _roundtrip(self, brep_path: Path, *, assert_valid: bool):
        """import (seed) -> export (under test) -> re-import -> validate the re-imported BrepArray."""
        stem = brep_path.stem
        seed_usd = self._import(brep_path, f"{stem}.A.usda")
        exported = self._export(seed_usd, f"{stem}.out.brep", brep_path)
        reimported = self._import_reimport(exported, f"{stem}.B.usda")
        self._validate_schema(reimported, brep_path)
        self._assert_topology_equal(seed_usd, reimported, brep_path)
        if assert_valid:
            self._assert_valid(reimported, brep_path)

    def _import_reimport(self, brep_path: Path, out_name: str) -> Path:
        """Re-import the exporter's output to validate it parses back to a BrepArray."""
        out_path = Path(self._tmp) / out_name
        self._run([self.importer, brep_path, out_path], "occt_to_usd (re-import of export)", brep_path)
        self.assertTrue(out_path.is_file(), msg=f"exporter output did not re-import: {brep_path}")
        return out_path

    def _topology_signature(self, usd_path: Path):
        stage = Usd.Stage.Open(str(usd_path))
        self.assertIsNotNone(stage, msg=f"failed to open {usd_path}")
        brep_prims = [prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]
        self.assertEqual(len(brep_prims), 1, msg=f"expected one BrepArray in {usd_path}")
        prim = brep_prims[0]
        return {
            attribute: [str(value) for value in prim.GetAttribute(attribute).Get()]
            for attribute in self._topology_attributes
        }

    def _attribute_signature(self, usd_path: Path, attributes):
        stage = Usd.Stage.Open(str(usd_path))
        self.assertIsNotNone(stage, msg=f"failed to open {usd_path}")
        brep_prims = [prim for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]
        self.assertEqual(len(brep_prims), 1, msg=f"expected one BrepArray in {usd_path}")
        prim = brep_prims[0]
        signature = {}
        for attribute in attributes:
            value = prim.GetAttribute(attribute).Get()
            signature[attribute] = None if value is None else list(value)
        return signature

    def _assert_topology_roundtrip(self, source: Path, stem: str):
        seed_usd = self._import(source, f"{stem}.A.usda")
        exported = self._export(seed_usd, f"{stem}.out.brep", source)
        reimported = self._import_reimport(exported, f"{stem}.B.usda")
        self._validate_schema(reimported, source)
        self._assert_topology_equal(seed_usd, reimported, source)

    def _assert_topology_equal(self, seed_usd: Path, reimported: Path, source: Path):
        seed = self._topology_signature(seed_usd)
        result = self._topology_signature(reimported)
        for attribute in self._topology_attributes:
            self.assertEqual(
                result[attribute],
                seed[attribute],
                msg=f"{attribute} changed while round-tripping {source.name}",
            )


class UsdToOcctCorpusRoundTripTestCase(_UsdToOcctRoundTripMixin):
    """Self-authored, shareable corpus committed under TestFiles/occt_breps."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.inputs_dir = cls.repo_root / "TestFiles" / "occt_breps"

    def test_pcurve_export_is_unconditional(self):
        source = self.inputs_dir / "cylinder_patch.brep"
        seed_usd = self._import(source, "pcurve_policy.usda")
        exported = self._export(seed_usd, "pcurve_policy.brep", source)

        lines = exported.read_text().splitlines()
        curve2d_line = next(
            (line for line in lines if line.startswith("Curve2ds ")),
            None,
        )
        self.assertIsNotNone(curve2d_line)
        self.assertGreater(int(curve2d_line.split()[1]), 0)

        curve2d_header = lines.index(curve2d_line)
        curve2d_count = int(curve2d_line.split()[1])
        curve2d_records = lines[curve2d_header + 1 : curve2d_header + 1 + curve2d_count]
        self.assertTrue(
            any(record.startswith("7 ") for record in curve2d_records),
            msg="stored UV NURBS curves were replaced by synthesized analytic lines",
        )

        for removed_option in ("--emit-pcurves", "--no-pcurves"):
            result = subprocess.run(
                [str(self.exporter), removed_option, str(seed_usd), str(exported)],
                capture_output=True,
                text=True,
                env=os.environ,
                timeout=self._timeout_sec,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("unknown option", result.stderr)

    def test_seam_pcurves_follow_edgeuse_orientation(self):
        source = self.inputs_dir / "cylinder.brep"
        seed_usd = self._import(source, "seam_orientation.A.usda")
        exported = self._export(seed_usd, "seam_orientation.out.brep", source)
        reimported = self._import_reimport(exported, "seam_orientation.B.usda")

        uv_attributes = (
            "brep:curveUv:nurb:vertexCount",
            "brep:curveUv:nurb:order",
            "brep:curveUv:nurb:controlVertices",
            "brep:curveUv:nurb:knots",
            "brep:curveUv:nurb:weights",
        )
        self.assertEqual(
            self._attribute_signature(reimported, uv_attributes),
            self._attribute_signature(seed_usd, uv_attributes),
            msg="seam UV curves changed sides while round-tripping cylinder.brep",
        )

    def test_container_orientations_roundtrip_through_canonical_export(self):
        box_source = (self.inputs_dir / "box.brep").read_text(encoding="utf-8")
        shell_marker = "\nSo\n\n1100000\n+2 0 *\n\n+1 0 "
        self.assertEqual(box_source.count(shell_marker), 1)

        reversed_outer = Path(self._tmp) / "box_reversed_outer_shell.brep"
        reversed_outer.write_text(
            box_source.replace(shell_marker, "\nSo\n\n1100000\n-2 0 *\n\n+1 0 "),
            encoding="utf-8",
        )
        self._assert_topology_roundtrip(reversed_outer, "reversed_outer")

        # A second, reversed reference to the box shell exercises the cavity-shell slot without
        # requiring another copy of the box's geometry records. This fixture is topology-only; the
        # coincident shells are intentionally not sent through SMLib's geometric AssertValid check.
        inner_shell = Path(self._tmp) / "box_with_reversed_inner_shell.brep"
        inner_shell.write_text(
            box_source.replace(shell_marker, "\nSo\n\n1100000\n+2 0 -2 0 *\n\n+1 0 "),
            encoding="utf-8",
        )
        self._assert_topology_roundtrip(inner_shell, "reversed_inner")

        open_shell_source = box_source.replace("\nSh\n\n0101100\n", "\nSh\n\n0101000\n")
        self.assertNotEqual(open_shell_source, box_source)
        open_shell = Path(self._tmp) / "box_reversed_open_shell.brep"
        open_shell.write_text(open_shell_source.replace("\n+1 0 ", "\n-2 0 "), encoding="utf-8")
        self._assert_topology_roundtrip(open_shell, "reversed_open_shell")

        mixed_marker = "\nSo\n\n1100000\n+2 0 *\n\n+1 0 "
        mixed_components = Path(self._tmp) / "box_with_open_face_component.brep"
        mixed_components.write_text(
            box_source.replace(mixed_marker, "\nCo\n\n1100000\n+2 0 +3 0 *\n\n+1 0 "),
            encoding="utf-8",
        )
        self._assert_topology_roundtrip(mixed_components, "mixed_components")

        face_source = (self.inputs_dir / "plane.brep").read_text(encoding="utf-8")
        root_marker = "\n+1 0 "
        self.assertEqual(face_source.count(root_marker), 1)
        reversed_face = Path(self._tmp) / "plane_reversed_root.brep"
        reversed_face.write_text(face_source.replace(root_marker, "\n-1 0 "), encoding="utf-8")
        self._assert_topology_roundtrip(reversed_face, "reversed_face")

    def test_corpus_roundtrip(self):
        samples = self._discover()
        self.assertTrue(samples, msg=f"no .brep fixtures found in {self.inputs_dir}")
        for sample in samples:
            with self.subTest(sample=sample.name):
                try:
                    self._roundtrip(sample, assert_valid=True)
                except unittest.SkipTest as exc:
                    _SUMMARY.record_skip("corpus", sample, exc)
                    raise
                except Exception as exc:
                    _SUMMARY.record_failure("corpus", sample, exc)
                    raise
                else:
                    _SUMMARY.record_success("corpus", sample)


class UsdToOcctExternalRoundTripTestCase(_UsdToOcctRoundTripMixin):
    """Proprietary / third-party parts kept out of git; skips when none are present."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        override = os.environ.get("OCCT_BREP_EXTERNAL_DIR")
        cls.inputs_dir = Path(override) if override else (cls.repo_root / "TestFiles" / "occt_breps_external")

    def test_external_roundtrip(self):
        samples = self._discover()
        if not samples:
            reason = (
                f"no external .brep samples in {self.inputs_dir} "
                "(populate it or set OCCT_BREP_EXTERNAL_DIR; see its README)"
            )
            _SUMMARY.record_suite_skip("external", reason)
            self.skipTest(reason)
        for sample in samples:
            with self.subTest(sample=sample.name):
                try:
                    self._roundtrip(sample, assert_valid=False)
                except unittest.SkipTest as exc:
                    _SUMMARY.record_skip("external", sample, exc)
                    raise
                except AssertionError as exc:
                    if "no Solid/Shell/Face component found" in str(exc):
                        reason = f"{sample.name} has no Solid/Shell/Face topology for BrepArray import"
                        _SUMMARY.record_skip("external", sample, reason)
                        self.skipTest(reason)
                    _SUMMARY.record_failure("external", sample, exc)
                    raise
                except Exception as exc:
                    _SUMMARY.record_failure("external", sample, exc)
                    raise
                else:
                    _SUMMARY.record_success("external", sample)


if __name__ == "__main__":
    unittest.main()
