// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepRead.h
 * PURPOSE: UsdBrepArray to UsdBrepArrayData translations
 *
 * CONTAINS:
 *  namespace: UsdBrepData{ // translation functions
 *                           BrepReadFromUsdStage()
 *                         }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_READ_H_
#define _USD_BREP_READ_H_

#include "UsdBrepConfig.h"

// USD includes
#include "UsdBrepHeaders.h"

namespace UsdBrepData
{
PXR_NAMESPACE_USING_DIRECTIVE

class UsdBrepArrayData;

// USD to UsdBrepArrayData. Returns false when required topology data or an
// applied geometry API selected by the BrepArray type tokens is missing.
USDBREP_EXPORT bool BrepReadFromUsdStage(
    const pxr::UsdPrim& crUsdBrepArray, // read from Prim
    UsdBrepArrayData& rArrays
);

} // end namespace UsdBrepData

#endif // no _USD_BREP_READ_H_
