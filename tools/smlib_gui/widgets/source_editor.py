# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Source editor widgets for the SMLib GUI."""

from __future__ import annotations

from ..runtime import QtWidgets


class PasteablePlainTextEdit(QtWidgets.QPlainTextEdit):
    """QPlainTextEdit that pastes as plain text, stripping rich formatting."""

    def __init__(self, parent=None):
        super().__init__(parent)

    def insertFromMimeData(self, source):
        if source.hasText():
            self.insertPlainText(source.text())
        else:
            super().insertFromMimeData(source)
