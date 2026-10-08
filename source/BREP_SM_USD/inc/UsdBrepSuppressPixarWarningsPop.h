// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************************
 * FILE NAME --- UsdBrepSuppressPixarWarningsPop.h
 * PURPOSE: Standardize suppressing pixar compile warnings
 * PURPOSE: Include after pixar header file includes to restore compile warnings
 * ***********************************************************************************/

// don't add compile guards here - needs to load once in every header file that calls it.

// done including pixar code - restore turned off compile warnings
#ifdef _MSC_VER
#    pragma warning(pop)
#elif defined(__GNUC__)
#    ifdef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#        define __DEPRECATED __attribute__((deprecated))
#        undef OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#    endif // OMNI_USD_SUPPRESS_DEPRECATION_WARNINGS
#endif // _MSC_VER or __GNUC__

// don't add compile guards here - needs to load once in every header file that calls it.
