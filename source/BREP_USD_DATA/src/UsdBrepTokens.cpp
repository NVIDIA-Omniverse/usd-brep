// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepTokens.cpp
 * PURPOSE: UsdBrepTokens, UsdBrepSolidTokens, UsdBrepCurveTokens, UsdBrepSurfaceTokens definitions
 * ******************************************************************************************************************/

#include "UsdBrepTokens.h"

// Create tokens for UsdBrepArray attribute access (not exported)
PXR_NAMESPACE_OPEN_SCOPE
TF_DEFINE_PUBLIC_TOKENS(UsdBrepSolidTokens, USDBREP_TOKENS);
TF_DEFINE_PUBLIC_TOKENS(UsdBrepCurveTokens, USDBREP_CURVE_TOKENS);
TF_DEFINE_PUBLIC_TOKENS(UsdBrepSurfaceTokens, USDBREP_SURFACE_TOKENS);
PXR_NAMESPACE_CLOSE_SCOPE
