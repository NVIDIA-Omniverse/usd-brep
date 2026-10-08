// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HD_USD_BREP_API_H
#define HD_USD_BREP_API_H

#include "pxr/base/arch/export.h"

// Exports for the tests and the usdview plugin; HDUSDBREP_EXPORTS is set when building it.
#if defined(HDUSDBREP_EXPORTS)
#    define HDUSDBREP_API ARCH_EXPORT
#else
#    define HDUSDBREP_API ARCH_IMPORT
#endif

#endif // HD_USD_BREP_API_H
