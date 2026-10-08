// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepHeaders.h
 * PURPOSE: Centralized USD header includes with warning suppression
 * NOTES: This header includes all USD headers with proper warning suppression
 *        to avoid cluttering individual source files with Push/Pop includes.
 * ******************************************************************************************************************/

// Suppress Windows warnings for USD headers
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
#endif

// Suppress Linux warnings for USD headers
#if defined(__GNUC__)
//   This suppresses deprecated header warnings, which is impossible with pragmas.
#    ifdef __DEPRECATED
#        define OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        undef __DEPRECATED
#    endif // __DEPRECATED
#endif

// Pixar headers that generate compiler warnings
// Review periodically in new USD releases to check if fixed.
#include <pxr/base/plug/registry.h> // unreferenced formal parameter (Windows C4100)
#include <pxr/base/tf/staticTokens.h> // outdated header hash_set (gcc)
#include <pxr/usd/usd/prim.h> // unreferenced formal parameter (Windows C4100)
#include <pxr/usd/usd/schemaBase.h> // unreferenced formal parameter (Windows C4100)

// Restore Linux warnings
#if defined(__GNUC__)
#    ifdef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        define __DEPRECATED __attribute__((deprecated))
#        undef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#    endif // OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#endif

// Restore Windows warnings
#ifdef _MSC_VER
#    pragma warning(pop)
#endif

// The following headers do not have warnings

// USD Base headers
#include <pxr/base/gf/range3f.h>
#include <pxr/base/plug/plugin.h>

// USD Core headers
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/tokens.h>

// USD Geometry headers
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tokens.h>

// USD Shade headers
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/usdShade/tokens.h>
