// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************************
* FILE NAME --- SmuConfig.h
* PURPOSE: Header file for local config used by BREP_SM_USD
* **********************************************************************************/

#ifndef _SMU_CONFIG_H_
#define _SMU_CONFIG_H_

#ifdef _WIN32
  #ifdef SMU_ENABLE_EXPORTS
    #define SMU_EXPORT __declspec( dllexport )
    #define SMU_THREAD_LOCAL __declspec(thread)  
  #else // no SMU_ENABLE_EXPORTS
    #define SMU_EXPORT __declspec( dllimport )
    #define SMU_THREAD_LOCAL __declspec(thread)  
  #endif // no SMU_ENABLE_EXPORTS
#else // no _WIN32
  #define SMU_EXPORT
    #define SMU_THREAD_LOCAL thread_local
#endif // no _WIN32

// debug blocks
#ifdef _DEBUG 
# ifndef  SMU_DEBUG_CODE
#  define SMU_DEBUG_CODE 1
# endif 
#endif // _DEBUG

#endif // no _SMU_CONFIG_H_
