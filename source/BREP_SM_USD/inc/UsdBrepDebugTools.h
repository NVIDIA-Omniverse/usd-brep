// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepDebugTools.h
 * PURPOSE: Debugging tools for
            UsdBrepArrayData and UsdBrepArraySpans classes
 * ******************************************************************************************************************/

#ifndef _USD_BREP_DEBUG_TOOLS_H_
#define _USD_BREP_DEBUG_TOOLS_H_

#include "SmuConfig.h"

// USD includes
#include "UsdBrepHeaders.h"

namespace UsdBrepData
{
class UsdBrepArrayData;
class UsdBrepArraySpans;

// prettyPrint UsdBrepArrayData and UsdBrepArraySpans
SMU_EXPORT void Dump_BrepArrayData(const UsdBrepArrayData& rArrays, bool bDumpData = false);
SMU_EXPORT void Dump_BrepArraySpans(const UsdBrepArrayData& rArrays, const UsdBrepArraySpans& rSpans, bool bDumpData = false);

SMU_EXPORT void Dump_PrimProperties(const pxr::UsdPrim& crPrim);
SMU_EXPORT void Dump_PrimSpecProperties(const pxr::SdfPrimSpecHandle& rPrimSpecHandle);
SMU_EXPORT void Dump_PrimProperties_fromPrimSpecHandle(const pxr::SdfPrimSpecHandle& crPrimSpecHandle, pxr::UsdStageRefPtr& rStage);

} // end namespace UsdBrepData

/*********************************************************************************************************************
  outputFiles directory
 ********************************************************************************************************************/

SMU_EXPORT bool usdBrep_CreateOutputFiles();

/*********************************************************************************************************************
  standardized I/O
 ********************************************************************************************************************/

SMU_EXPORT void usdBrep_WriteString(const std::string& string);

#endif // no _USD_BREP_DEBUG_TOOLS_H_
