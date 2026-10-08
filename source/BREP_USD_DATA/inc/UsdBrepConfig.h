// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepConfig.h
 * PURPOSE: local config header file for BREP_USD_DATA
 * ******************************************************************************************************************/

#ifndef _USD_BREP_CONFIG_H_
#define _USD_BREP_CONFIG_H_

#if defined(_WIN32)
#    ifdef USDBREP_ENABLE_EXPORTS
#        define USDBREP_EXPORT __declspec(dllexport)
#    else // no USDBREP_ENABLE_EXPORTS
#        define USDBREP_EXPORT __declspec(dllimport)
#    endif // no USDBREP_ENABLE_EXPORTS
#else // no _WIN32
#    ifdef USDBREP_ENABLE_EXPORTS
#        define USDBREP_EXPORT __attribute__((visibility("default")))
#    else
#        define USDBREP_EXPORT
#    endif
#endif // no _WIN32

#endif // no _USD_BREP_CONFIG_H_
