// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiTypes.h

PURPOSE: 
    Defines types used throught smlib.

GENERAL NOTES: 

**********************************************************************/

#ifndef __SmApiTypes_H__
#define __SmApiTypes_H__

#ifdef _UNICODE
#include <tchar.h>
#endif

#define SM_TBLOCK_SIZE 1024
#define SM_LARGE_TBLOCK_SIZE 4096

#define SmApiStatus    long


#if defined(_WIN32)
#  ifdef SM_API_EXPORTS
#    define SMAPI_EXPORT __declspec(dllexport)
#  else
#    define SMAPI_EXPORT __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define SMAPI_EXPORT __attribute__((visibility("default")))
#else
#  define SMAPI_EXPORT
#endif


typedef unsigned long ULONG;

#ifdef _UNICODE
#ifndef TCHAR
typedef _TCHAR TCHAR;
#endif
#else  // no _UNICODE
typedef char TCHAR;
 #undef _T
 #define _T(a) a
#endif // no _UNICODE

#endif
