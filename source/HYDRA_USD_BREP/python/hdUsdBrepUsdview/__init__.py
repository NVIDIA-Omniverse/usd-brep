# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Forward usdview's display mode to the SMLib boundary display scene index."""

import ctypes

from pxr import Plug, Tf
from pxr.Usdviewq.common import RenderModes
from pxr.Usdviewq.plugin import PluginContainer
from pxr.Usdviewq.qt import QtCore


class _DisplayUpdates(QtCore.QObject):
    def __init__(self, library, context):
        super().__init__(context.qMainWindow)
        self._context = context
        self._set_mode = library.HdUsdBrepSetDisplayMode
        self._set_mode.argtypes = [ctypes.c_int]
        self._set_mode.restype = None
        self._apply_mode = library.HdUsdBrepApplyPendingDisplayMode
        self._apply_mode.argtypes = []
        self._apply_mode.restype = None
        self._mode = None
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.apply)

    @QtCore.Slot()
    def request(self):
        render_mode = self._context.dataModel.viewSettings.renderMode
        mode = (
            1 if render_mode == RenderModes.WIREFRAME else 2 if render_mode == RenderModes.WIREFRAME_ON_SURFACE else 0
        )
        if mode != self._mode:
            self._mode = mode
            self._set_mode(mode)
            # Apply on the event loop, after the current render/setting callbacks return.
            self._timer.start(0)

    @QtCore.Slot()
    def apply(self):
        self._apply_mode()
        self._context.UpdateViewport()


class BrepDisplayPlugin(PluginContainer):
    def registerPlugins(self, plugRegistry, plugCtx):
        plugin = Plug.Registry().GetPluginWithName("hdUsdBrep")
        plugin.Load()
        self._library = ctypes.CDLL(plugin.path)
        self._updates = _DisplayUpdates(self._library, plugCtx)
        plugCtx.dataModel.viewSettings.signalSettingChanged.connect(self._updates.request)
        self._updates.request()

    def configureView(self, plugRegistry, plugUIBuilder):
        pass


Tf.Type.Define(BrepDisplayPlugin)
