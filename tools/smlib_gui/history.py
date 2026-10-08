# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Source history model for the SMLib GUI."""

from __future__ import annotations


class SourceHistory:
    """Bounded source snapshot history with cursor-style navigation."""

    def __init__(self, max_entries: int = 50):
        self.max_entries = max(1, int(max_entries))
        self._entries: list[str] = []
        self._index = -1

    def __len__(self) -> int:
        return len(self._entries)

    @property
    def index(self) -> int:
        return self._index

    @property
    def position_label(self) -> str:
        if not self._entries:
            return "0/0"
        return f"{self._index + 1}/{len(self._entries)}"

    def add(self, source: str) -> bool:
        """Add a non-empty source snapshot and select it."""
        snapshot = self._normalize(source)
        if not snapshot:
            return False
        if self._index >= 0 and self._entries[self._index] == snapshot:
            return False
        if self._index < len(self._entries) - 1:
            self._entries = self._entries[:self._index + 1]
        if self._entries and self._entries[-1] == snapshot:
            self._index = len(self._entries) - 1
            return False
        self._entries.append(snapshot)
        overflow = len(self._entries) - self.max_entries
        if overflow > 0:
            del self._entries[:overflow]
        self._index = len(self._entries) - 1
        return True

    def current(self) -> str | None:
        if self._index < 0 or self._index >= len(self._entries):
            return None
        return self._entries[self._index]

    def can_previous(self) -> bool:
        return self._index > 0

    def previous(self) -> str | None:
        if not self.can_previous():
            return None
        self._index -= 1
        return self.current()

    def can_next(self) -> bool:
        return 0 <= self._index < len(self._entries) - 1

    def next(self) -> str | None:
        if not self.can_next():
            return None
        self._index += 1
        return self.current()

    @staticmethod
    def _normalize(source: str) -> str:
        if not source or not source.strip():
            return ""
        return source.rstrip() + "\n"
