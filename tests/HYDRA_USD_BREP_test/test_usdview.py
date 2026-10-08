# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import time
from pathlib import Path

from OpenGL import GL
from pxr import Plug
from pxr.Usdviewq.common import RenderModes
from pxr.Usdviewq.qt import QtWidgets


def testUsdviewInputFunction(appController):
    view = appController._stageView
    settings = appController._dataModel.viewSettings
    settings.showHUD = False
    settings.showBBoxes = False
    settings.clearColorText = "Black"
    appController._dataModel.selection.clearPrims()
    view.updateView(resetCam=True, forceComputeBBox=True)
    for _ in range(40):
        QtWidgets.QApplication.processEvents()
        view.update()
        time.sleep(0.1)
    view.makeCurrent()
    vendor = GL.glGetString(GL.GL_VENDOR).decode()
    renderer = GL.glGetString(GL.GL_RENDERER).decode()
    print(f"GPU: {vendor}; {renderer}", flush=True)
    assert not any(
        name in renderer.lower() for name in ("llvmpipe", "softpipe", "swrast", "software", "swiftshader")
    ), renderer
    assert view.GetRendererDisplayName(view.GetCurrentRendererId()) == "Storm", "expected Storm"
    plugin = Plug.Registry().GetPluginWithName("hdUsdBrep")
    assert plugin and plugin.isLoaded, "hdUsdBrep did not load"
    assert view._renderer.GetRenderStats().get("topology", 0) > 0, "no rendered topology"
    shot = appController.GrabViewportShot()
    assert not shot.isNull(), "missing viewport image"
    # With HUD and bounds hidden, enough non-black pixels means bodies rendered.
    colored = sum(
        max(shot.pixelColor(x, y).getRgb()[:3]) > 32
        for y in range(0, shot.height(), 4)
        for x in range(0, shot.width(), 4)
    )
    samples = len(range(0, shot.height(), 4)) * len(range(0, shot.width(), 4))
    assert colored > samples * 0.01, "rendered image has no substantial geometry"
    assert shot.save(os.environ["HDUSDBREP_GPU_IMAGE"]), "failed to save viewport image"
    image = Path(os.environ["HDUSDBREP_GPU_IMAGE"])
    shaded = shot
    for mode, suffix in (
        (RenderModes.WIREFRAME, "wireframe"),
        (RenderModes.WIREFRAME_ON_SURFACE, "boundaries"),
        (RenderModes.SMOOTH_SHADED, "restored"),
    ):
        settings.renderMode = mode
        for _ in range(10):
            QtWidgets.QApplication.processEvents()
            view.update()
            time.sleep(0.1)
        shot = appController.GrabViewportShot()
        assert shot.save(str(image.with_name(image.stem + "-" + suffix + image.suffix)))
        if mode == RenderModes.WIREFRAME:
            coverage = sum(
                max(shot.pixelColor(x, y).getRgb()[:3]) > 32 for y in range(shot.height()) for x in range(shot.width())
            )
            shaded_coverage = sum(
                max(shaded.pixelColor(x, y).getRgb()[:3]) > 32
                for y in range(shaded.height())
                for x in range(shaded.width())
            )
            assert (
                shaded_coverage * 0.01 < coverage < shaded_coverage * 0.25
            ), "wireframe must contain visible lines without filled surfaces"
        elif mode == RenderModes.SMOOTH_SHADED:
            assert shot == shaded, "switching back to shaded changed the rendered geometry"
    print("PASS: native-GPU Storm rendering and BRep boundary display modes", flush=True)
