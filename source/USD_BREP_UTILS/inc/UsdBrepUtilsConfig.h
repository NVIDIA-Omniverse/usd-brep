// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepUtilsConfig.h
 * PURPOSE: local config header file for USD_BREP_UTILS
 * ******************************************************************************************************************/

#ifndef _USD_BREP_UTILS_CONFIG_H_
#define _USD_BREP_UTILS_CONFIG_H_

#if defined(_WIN32)
#    ifdef USD_BREP_ENABLE_EXPORTS
#        define USD_BREP_EXPORT __declspec(dllexport)
#    else
#        define USD_BREP_EXPORT __declspec(dllimport)
#    endif
#else
#    ifdef USD_BREP_ENABLE_EXPORTS
#        define USD_BREP_EXPORT __attribute__((visibility("default")))
#    else
#        define USD_BREP_EXPORT
#    endif
#endif

#endif // _USD_BREP_UTILS_CONFIG_H_
