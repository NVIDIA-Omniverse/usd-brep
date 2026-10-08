// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HD_USD_BREP_DISPLAY_SCENE_INDEX_PLUGIN_H
#define HD_USD_BREP_DISPLAY_SCENE_INDEX_PLUGIN_H

#include "HdUsdBrepApi.h"

// Queue a display mode (0 shaded, 1 boundary wireframe, 2 both). Thread-safe; changes nothing yet.
extern "C" HDUSDBREP_API void HdUsdBrepSetDisplayMode(int mode);

// Apply the queued mode between renders on the scene's update thread; other threads' scenes are skipped.
extern "C" HDUSDBREP_API void HdUsdBrepApplyPendingDisplayMode();

#endif
