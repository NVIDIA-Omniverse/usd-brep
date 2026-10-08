// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepWrite.h
 * PURPOSE: UsdBrepArrayData to UsdBrepArray translations
 *
 * CONTAINS:
 *  namespace: UsdBrepData{ // translation functions
 *                           BrepWriteToUsdStage()
 *                         }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_WRITE_H_
#define _USD_BREP_WRITE_H_

#include "UsdBrepConfig.h"

// USD includes
#include "UsdBrepHeaders.h"

namespace UsdBrepData
{
PXR_NAMESPACE_USING_DIRECTIVE

class UsdBrepArrayData;

// UsdBrepArrayData to USD
USDBREP_EXPORT bool BrepWriteToUsdStage(const UsdBrepArrayData& rArrays, pxr::UsdPrim& rUsdBrepArray);

} // end namespace UsdBrepData

#endif // no _USD_BREP_WRITE_H_
