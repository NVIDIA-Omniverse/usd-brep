// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdConfig.h
* PURPOSE: DLL export/import macros for SM_API_USD
**********************************************************************/

#ifndef _SM_API_USD_CONFIG_H_
#define _SM_API_USD_CONFIG_H_

#if defined(_WIN32)
#    ifdef SM_API_USD_EXPORTS
#        define SM_API_USD_EXPORT __declspec(dllexport)
#    else
#        define SM_API_USD_EXPORT __declspec(dllimport)
#    endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#    ifdef SM_API_USD_EXPORTS
#        define SM_API_USD_EXPORT __attribute__((visibility("default")))
#    else
#        define SM_API_USD_EXPORT
#    endif
#else
#    define SM_API_USD_EXPORT
#endif

#endif // _SM_API_USD_CONFIG_H_
