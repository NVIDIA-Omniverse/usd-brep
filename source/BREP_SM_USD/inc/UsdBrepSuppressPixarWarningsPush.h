// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************************
 * FILE NAME --- UsdBrepSuppressPixarWarningsPush.h
 * PURPOSE: Standardize suppressing pixar compile warnings
 * NOTES: Include before pixar header file includes to suppress pixar compile warnings
 * ***********************************************************************************/

// don't add compile guards here - needs to load once in every header file that calls it.
//  pragma warning labeled with a ** are needed in windows to suppress pxr file compile warnings.
//                                ^^ are not needed in windows.

// turn off known compiler warnings for code owned by pixar
#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4100) // Unreferenced formal parameter
#    pragma warning(disable : 4127) // Conditional expression is constant
#    pragma warning(disable : 4201) // Nonstandard extension used: nameless struct/union
#    pragma warning(disable : 4245) // 'initializing': conversion from 'int' to 'const size_t', signed/unsigned mismatch
#    pragma warning(disable : 4251) // class needs to have dll-interface to be used by clients of class
#    pragma warning(disable : 4273) // inconsistent dll linkage
#    ifndef NOMINMAX
#        define NOMINMAX // Make sure nobody #defines min or max
#    endif
#    undef small // defined in rpcndr.h
#elif defined(__GNUC__)
//   This suppresses deprecated header warnings, which is impossible with pragmas.
//   Alternative is to specify -Wno-deprecated build option, but that disables other useful warnings too.
#    ifdef __DEPRECATED
#        define OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        undef __DEPRECATED
#    endif // __DEPRECATED
#endif // _MSC_VER or __GNUC__
