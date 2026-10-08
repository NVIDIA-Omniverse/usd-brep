# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Importable USD/BrepArray validation API.

Provides a self-contained entry point that registers the omniSolid schema, runs the USD asset
validator (full rule set) together with the BrepArray schema rules (BrepValidator), and returns a
structured pass/fail result. Consumers can call ``validate_file()`` directly instead of shelling out
to a CLI and hand-rolling plugin/environment setup; the CLI ``validate_usd.py`` is a thin wrapper
around this module so the run/pass-fail logic lives in exactly one place.
"""

import importlib.util
import multiprocessing as mp
import os
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from pxr import Plug, Sdf, Usd
except ImportError as exc:  # pragma: no cover - environment dependent
    raise ImportError(
        "pxr (USD) is required to use brep_validator.validate; ensure USD is available on the "
        "environment (see https://openusd.org/release/tut_usd_tutorials.html)."
    ) from exc

try:
    import usd_validation_nvidia as _asset_validator
except ImportError as exc:  # pragma: no cover - environment dependent
    raise ImportError(
        "usd_validation_nvidia is required to use brep_validator.validate; install it with "
        'pip install "usd-validation-nvidia>=1.22.0,<2".'
    ) from exc

from .brep_validator import BrepValidator

# Asset-validator issue severities that count as a failed validation.
_FAILURE_SEVERITIES = ("FAILURE", "ERROR", "WARNING")
_MP_CONTEXT = mp.get_context("spawn")
_worker_stage = None


@dataclass
class BrepValidationResult:
    """Structured outcome of validating a single USD file."""

    identifier: str
    passed: bool
    failure_count: int
    brep_array_count: int
    messages: List[str] = field(default_factory=list)
    issue_summary: Dict[str, int] = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.passed


def _bundled_schema_path() -> Optional[Path]:
    """Return the omniSolid schema shipped with this package, if there is one.

    In the usd-brep wheel it lives in the ``usd_brep`` package; in the release package it sits
    beside ``brep_validator``. ``find_spec`` locates ``usd_brep`` without importing it, which
    would load the wheel's own OpenUSD into this process.
    """
    candidates = []
    spec = importlib.util.find_spec("usd_brep")
    if spec is not None and spec.submodule_search_locations:
        candidates += [Path(p) / "omniSolid" / "resources" for p in spec.submodule_search_locations]
    candidates.append(Path(__file__).resolve().parents[1] / "omniSolid" / "resources")
    return next((path for path in candidates if (path / "plugInfo.json").is_file()), None)


def register_omnisolid_schema(plugin_path: "Optional[str]" = None) -> bool:
    """Register the codeless omniSolid schema so BrepArray prim types resolve.

    Uses ``plugin_path`` when given, and nothing else, so a wrong path is reported rather than
    replaced. Otherwise registers the first schema found in the ``OMNISOLID_PLUGIN_PATH``
    environment variable and the schema bundled with the usd-brep wheel or release package.
    A path counts only if it is a ``plugInfo.json`` or a directory holding one: any other path
    registers nothing, so it must not stop the fallback. Returns True when one was registered.
    Safe to call repeatedly.
    """
    if plugin_path:
        candidates = (plugin_path,)
    else:
        candidates = (os.environ.get("OMNISOLID_PLUGIN_PATH"), _bundled_schema_path())
    for path in candidates:
        candidate = Path(path).expanduser() if path else None
        if candidate and ((candidate / "plugInfo.json").is_file() or (candidate.name == "plugInfo.json" and candidate.is_file())):
            Plug.Registry().RegisterPlugins(str(candidate.resolve()))
            return True
    return False


def _brep_array_paths(identifier: str) -> List[str]:
    layer = Sdf.Layer.FindOrOpen(identifier)
    if not layer:
        return []
    stage = Usd.Stage.Open(layer)
    if not stage:
        return []
    return [str(prim.GetPath()) for prim in stage.TraverseAll() if prim.GetTypeName() == "BrepArray"]


def _issue_type_key(issue) -> str:
    code = getattr(issue, "code", None)
    if code:
        return str(code)
    requirement = getattr(issue, "requirement", None)
    if requirement is not None:
        code = getattr(requirement, "code", None)
        if code:
            return str(code)
        name = getattr(requirement, "name", None)
        if name:
            return str(name)
    return issue.severity.name


def _format_issue(issue) -> str:
    prim_path = ""
    if issue.at is not None and hasattr(issue.at, "path"):
        prim_path = f" @ {issue.at.path}"
    rule_name = ""
    if issue.requirement is not None and hasattr(issue.requirement, "name"):
        rule_name = getattr(issue.requirement, "name", "") or ""
    if not rule_name and hasattr(issue, "rule"):
        rule_name = str(issue.rule) if issue.rule else ""
    rule_str = f" ({rule_name})" if rule_name else ""
    return f"{issue.severity.name}{rule_str}: {issue.message}{prim_path}"


def _init_brep_worker(identifier: str, omnisolid_plugin_path: Optional[str]) -> None:
    """Open the USD stage once per worker process."""
    global _worker_stage
    register_omnisolid_schema(omnisolid_plugin_path)
    layer = Sdf.Layer.FindOrOpen(identifier)
    _worker_stage = Usd.Stage.Open(layer) if layer else None


def _validate_brep_path(path_str: str) -> List[Tuple[str, str]]:
    global _worker_stage
    if _worker_stage is None:
        message = f"ERROR: Worker stage not initialized for {path_str}"
        return [(message, "ERROR")]

    try:
        prim = _worker_stage.GetPrimAtPath(path_str)
        if not prim.IsValid():
            return []

        checker = BrepValidator(
            verbose=False,
            consumerLevelChecks=["BrepValidator"],
            assetLevelChecks=["BrepValidator"],
        )
        checker.CheckPrim(prim)
        return [
            (_format_issue(issue), _issue_type_key(issue))
            for issue in checker.GetIssues()
            if issue.severity.name in _FAILURE_SEVERITIES
        ]
    except Exception as exc:  # noqa: BLE001 - one bad prim must not abort the whole run
        return [(f"ERROR: BrepValidator raised for {path_str}: {exc}", "ERROR")]


def _default_worker_count() -> int:
    return max(1, (os.cpu_count() or 1) // 2)


def _resolve_worker_count(workers: Optional[int]) -> int:
    if workers is None:
        return _default_worker_count()
    core_count = os.cpu_count() or 1
    return max(1, min(core_count, workers))


def validate_file(
    identifier,
    *,
    omnisolid_plugin_path: "Optional[str]" = None,
    extra_rules: "Optional[Sequence]" = None,
    workers: "Optional[int]" = None,
    brep_only: bool = False,
) -> BrepValidationResult:
    """Validate a single USD file and return a structured pass/fail result.

    Registers the omniSolid schema (see :func:`register_omnisolid_schema`), runs
    the BrepArray schema rules (``BrepValidator``) when BrepArray prims are present, and by default
    the USD asset-validator rule set (plus any ``extra_rules``). Any FAILURE/ERROR/WARNING issue counts as a
    failure.

    BrepArray prims are validated in parallel (default: one worker per two CPU cores).
    Set ``workers=1`` for a single worker process.

    Set ``brep_only=True`` to skip the full asset-validator pass.

    :param identifier: Path (or layer identifier) to a ``.usd``/``.usda``/``.usdc`` file.
    :param omnisolid_plugin_path: omniSolid schema resources dir. When omitted, uses OMNISOLID_PLUGIN_PATH,
        else the schema bundled with the usd-brep wheel or release package.
    :raises FileNotFoundError: ``omnisolid_plugin_path`` is given but holds no ``plugInfo.json``.
    :param extra_rules: Optional additional validator rule classes for the asset-validator pass.
    :param workers: Worker count for the BrepArray pass (default: cpu_count/2).
    :param brep_only: When True, run only the BrepArray pass and skip the asset-validator pass.
    :returns: A :class:`BrepValidationResult` (truthy when validation passed).
    """
    identifier = str(identifier)
    if not register_omnisolid_schema(omnisolid_plugin_path) and omnisolid_plugin_path:
        raise FileNotFoundError(f"omniSolid schema not found at {omnisolid_plugin_path}")

    messages: List[str] = []
    issue_summary: Dict[str, int] = {}
    failure_count = 0
    brep_array_paths = _brep_array_paths(identifier)
    brep_array_count = len(brep_array_paths)

    if brep_array_count > 0:
        worker_count = _resolve_worker_count(workers)
        try:
            with ProcessPoolExecutor(
                max_workers=worker_count,
                mp_context=_MP_CONTEXT,
                initializer=_init_brep_worker,
                initargs=(identifier, omnisolid_plugin_path),
            ) as executor:
                for chunk_issues in executor.map(_validate_brep_path, brep_array_paths, chunksize=1):
                    for message, key in chunk_issues:
                        messages.append(message)
                        issue_summary[key] = issue_summary.get(key, 0) + 1
                        failure_count += 1
        except BrokenProcessPool as exc:
            messages.append(f"ERROR: BrepArray worker pool failed (worker init error?): {exc}")
            issue_summary["ERROR"] = issue_summary.get("ERROR", 0) + 1
            failure_count += 1

    if not brep_only:
        engine = _asset_validator.ValidationEngine()
        for rule in extra_rules or ():
            engine.enable_rule(rule)
        for issue in engine.validate(identifier).issues():
            if issue.severity.name in _FAILURE_SEVERITIES:
                failure_count += 1
                messages.append(_format_issue(issue))
                key = _issue_type_key(issue)
                issue_summary[key] = issue_summary.get(key, 0) + 1

    return BrepValidationResult(
        identifier=identifier,
        passed=failure_count == 0,
        failure_count=failure_count,
        brep_array_count=brep_array_count,
        messages=messages,
        issue_summary=issue_summary,
    )
