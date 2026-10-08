// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HD_USD_BREP_TOKENS_H
#define HD_USD_BREP_TOKENS_H

// Keys with no standard Hydra name: the adapter's tessellation data, its per-subset and
// per-Brep material paths, and the marker on the procedural's children.

#include "pxr/pxr.h"
#include "pxr/base/tf/staticTokens.h"

PXR_NAMESPACE_OPEN_SCOPE

#define HD_USD_BREP_TOKENS (usdBrepTessellatorData)(materialPaths)(brepMaterialPath)(usdBrepEdges)

TF_DECLARE_PUBLIC_TOKENS(HdUsdBrepTokens, HD_USD_BREP_TOKENS);

PXR_NAMESPACE_CLOSE_SCOPE

#endif
