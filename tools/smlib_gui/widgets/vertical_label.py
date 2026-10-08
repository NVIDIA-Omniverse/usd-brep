# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Sidebar label that paints its text along the vertical axis."""

from __future__ import annotations

from ..runtime import QtCore, QtGui, QtWidgets


class VerticalLabel(QtWidgets.QWidget):
    """Label that paints text rotated 90 degrees, reading bottom to top."""

    clicked = QtCore.Signal()

    def __init__(self, text: str = "", parent=None):
        super().__init__(parent)
        self._text = text
        self._color = QtGui.QColor("#7ec8e3")
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.MinimumExpanding)
        self.setCursor(QtCore.Qt.PointingHandCursor)

    def text(self) -> str:
        return self._text

    def setText(self, text: str) -> None:
        self._text = text
        self.update()
        self.updateGeometry()

    def sizeHint(self) -> QtCore.QSize:
        metrics = self.fontMetrics()
        return QtCore.QSize(metrics.height() + 4, metrics.horizontalAdvance(self._text) + 12)

    def minimumSizeHint(self) -> QtCore.QSize:
        return self.sizeHint()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit()
            event.accept()
            return
        super().mousePressEvent(event)

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.TextAntialiasing)
        painter.setPen(self._color)
        painter.setFont(self.font())
        painter.translate(0, self.height())
        painter.rotate(-90)
        painter.drawText(
            QtCore.QRect(0, 0, self.height(), self.width()),
            QtCore.Qt.AlignCenter,
            self._text,
        )
