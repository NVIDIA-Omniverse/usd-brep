// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuTokens.cpp
* PURPOSE: definition of tokens used by BREP_SM_USD
**********************************************************************/

#include "SmuTokens.h"

// Python must be included first because it monkeys with macros that cause
// TBB to fail to compile in debug mode if TBB is included before Python

// Create tokens for SM_USD (not exported)
PXR_NAMESPACE_OPEN_SCOPE 
TF_DEFINE_PUBLIC_TOKENS(SmuTessTokens, SMU_TESS_TOKENS);
PXR_NAMESPACE_CLOSE_SCOPE 

