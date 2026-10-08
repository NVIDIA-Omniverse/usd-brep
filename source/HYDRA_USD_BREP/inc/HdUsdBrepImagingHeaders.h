// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HDUSDBREP_USD_IMAGING_HEADERS_H
#define HDUSDBREP_USD_IMAGING_HEADERS_H

// USD 25.11 imaging headers warn on MSVC (C4100 unused parameters, C4324 robin_map
// padding) and warnings are errors, so suppress them around the pxr includes only.
#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4100 4324)
#endif

// MSVC binds warning state at template definition, so core USD must be included here too.
#include "UsdBrepHeaders.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/usdImaging/usdImaging/instanceablePrimAdapter.h"
#include "pxr/usdImaging/usdImaging/dataSourceAttribute.h"
#include "pxr/usdImaging/usdImaging/dataSourceGprim.h"
#include "pxr/usdImaging/usdImaging/indexProxy.h"

#ifdef _MSC_VER
#    pragma warning(pop)
#endif

#endif
