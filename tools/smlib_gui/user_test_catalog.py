# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Numbered user-test catalog for the SMLib GUI."""

from __future__ import annotations

import ast
import os
import re
import textwrap
from dataclasses import dataclass

DEFAULT_USER_TEST_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "user_test.py")
TEST_FUNC_RE = re.compile(r"^test_(\d+)$")
SETTINGS_ORG = "NVIDIA"
SETTINGS_APP = "smlib_gui"
SETTINGS_KEY_USER_TEST_NUMBER = "UserTestNumber"
SETTINGS_KEY_USER_TEST_PATH = "UserTestPath"
SETTINGS_KEY_USER_TEST_PANEL_COLLAPSED = "UserTestPanelCollapsed"


@dataclass(frozen=True)
class UserTestCase:
    """One numbered playback case extracted from ``user_test.py``."""

    number: int
    title: str
    docstring: str
    source: str
    lineno: int

    @property
    def label(self) -> str:
        if self.title:
            return f"{self.number}: {self.title}"
        return str(self.number)


def script_is_empty(source: str) -> bool:
    """True when source has no executable statements besides ``pass`` / docs."""
    text = source.strip()
    if not text:
        return True
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return False
    for node in tree.body:
        if isinstance(node, ast.Pass):
            continue
        if isinstance(node, ast.Expr) and _is_string_constant(node.value):
            continue
        if isinstance(node, ast.Expr) and _is_ellipsis(node.value):
            continue
        return False
    return True


def parse_user_tests(source: str) -> list[UserTestCase]:
    """Parse ``def test_N():`` functions into numbered playback cases."""
    tree = ast.parse(source)
    lines = source.splitlines(keepends=True)
    by_number: dict[int, UserTestCase] = {}
    for node in tree.body:
        if not isinstance(node, ast.FunctionDef):
            continue
        match = TEST_FUNC_RE.match(node.name)
        if not match:
            continue
        number = int(match.group(1))
        docstring = ast.get_docstring(node) or ""
        title = docstring.strip().splitlines()[0].strip() if docstring.strip() else ""
        body = _function_body_source(node, lines)
        by_number[number] = UserTestCase(
            number=number,
            title=title,
            docstring=docstring,
            source=body,
            lineno=node.lineno,
        )
    return [by_number[number] for number in sorted(by_number)]


def case_by_number(cases: list[UserTestCase], number: int) -> UserTestCase | None:
    for case in cases:
        if case.number == number:
            return case
    return None


def load_user_tests(path: str | None = None) -> tuple[list[UserTestCase], str]:
    """Read and parse the user-test file. Returns (cases, error)."""
    catalog_path = path or DEFAULT_USER_TEST_PATH
    try:
        text = _read_text(catalog_path)
    except OSError as exc:
        return [], f"Could not read user tests: {exc}"
    try:
        return parse_user_tests(text), ""
    except SyntaxError as exc:
        return [], f"user_test.py syntax error: {exc}"


def save_user_test(
    number: int,
    body: str,
    *,
    path: str | None = None,
    title: str | None = None,
) -> str:
    """Write ``test_N`` into the catalog file. Returns an error string, or empty."""
    catalog_path = path or DEFAULT_USER_TEST_PATH
    try:
        text = _read_text(catalog_path)
    except OSError:
        text = _new_catalog_header()
    try:
        updated = replace_user_test(text, number, body, title=title)
    except SyntaxError as exc:
        return f"Cannot save user test {number}: {exc}"
    try:
        parse_user_tests(updated)
    except SyntaxError as exc:
        return f"Cannot save user test {number}: {exc}"
    try:
        os.makedirs(os.path.dirname(catalog_path) or ".", exist_ok=True)
        with open(catalog_path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(updated)
    except OSError as exc:
        return f"Could not write user tests: {exc}"
    return ""


def replace_user_test(
    file_source: str,
    number: int,
    body: str,
    title: str | None = None,
) -> str:
    """Return ``file_source`` with ``test_N`` replaced or appended."""
    tree = ast.parse(file_source)
    lines = file_source.splitlines(keepends=True)
    target = None
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == f"test_{number}":
            target = node
            break
    docstring = title
    if docstring is None and target is not None:
        docstring = ast.get_docstring(target) or ""
    new_fn = format_test_function(number, body, docstring or "")
    if target is None:
        prefix = file_source if file_source.endswith("\n") else file_source + "\n"
        if prefix.strip():
            updated = prefix.rstrip() + "\n\n" + new_fn
        else:
            updated = new_fn
    else:
        start = target.lineno - 1
        end = target.end_lineno
        suffix = "".join(lines[end:])
        prefix = "".join(lines[:start])
        if suffix and not new_fn.endswith("\n"):
            new_fn += "\n"
        updated = prefix + new_fn + suffix
    ast.parse(updated)
    return updated


def format_test_function(number: int, body: str, docstring: str = "") -> str:
    """Render a ``test_N`` function for the catalog file."""
    cleaned = textwrap.dedent(body).strip("\n")
    if not cleaned:
        cleaned = "pass"
    indented = textwrap.indent(cleaned + "\n", "    ")
    doc = _format_docstring(docstring)
    return f"def test_{number}():\n{doc}{indented}"


def _format_docstring(docstring: str) -> str:
    text = docstring.strip()
    if not text:
        return ""
    if "\n" not in text:
        escaped = text.replace('"""', "'''")
        return f'    """{escaped}"""\n'
    inner = text.replace('"""', "'''")
    return f'    """\n{textwrap.indent(inner, "    ")}\n    """\n'


def _function_body_source(node: ast.FunctionDef, lines: list[str]) -> str:
    body = list(node.body)
    if body and _is_docstring_stmt(body[0]):
        body = body[1:]
    if not body:
        return "pass\n"
    start = body[0].lineno
    end = node.end_lineno
    chunk = "".join(lines[start - 1 : end])
    return textwrap.dedent(chunk)


def _is_docstring_stmt(node: ast.stmt) -> bool:
    return isinstance(node, ast.Expr) and _is_string_constant(node.value)


def _is_string_constant(value: ast.expr) -> bool:
    return isinstance(value, ast.Constant) and isinstance(value.value, str)


def _is_ellipsis(value: ast.expr) -> bool:
    return isinstance(value, ast.Constant) and value.value is Ellipsis


def _read_text(path: str) -> str:
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _new_catalog_header() -> str:
    return (
        "# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. "
        "All rights reserved.\n"
        "# SPDX-License-Identifier: Apache-2.0\n"
        '"""User-managed SMLib GUI tests."""\n\n'
    )


__all__ = [
    "DEFAULT_USER_TEST_PATH",
    "SETTINGS_APP",
    "SETTINGS_KEY_USER_TEST_NUMBER",
    "SETTINGS_KEY_USER_TEST_PANEL_COLLAPSED",
    "SETTINGS_KEY_USER_TEST_PATH",
    "SETTINGS_ORG",
    "UserTestCase",
    "case_by_number",
    "format_test_function",
    "load_user_tests",
    "parse_user_tests",
    "replace_user_test",
    "save_user_test",
    "script_is_empty",
]
