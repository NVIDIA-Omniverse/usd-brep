// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepStageUtils.h
 * PURPOSE: Utilities for discovering and loading BrepArray prims from a USD stage.
 *
 * CONTAINS:
 *  namespace UsdBrep {
 *      FindBrepArrayPrims()  - find all BrepArray prims in a stage
 *      LoadAllBrepArrays()   - load all BrepArray prims from a USD file
 *      LoadBrepArray()       - load a single BrepArray from a prim
 *      SaveBrepArray()       - write a UsdBrepArrayData to a prim
 *  }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_STAGE_UTILS_H_
#define _USD_BREP_STAGE_UTILS_H_

#include "UsdBrepArrayData.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes
#include "UsdBrepUtilsConfig.h"

#include <pxr/usd/usd/stage.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace UsdBrep
{
PXR_NAMESPACE_USING_DIRECTIVE
using UsdBrepData::UsdBrepArrayData;

// Find all BrepArray prims in a stage by matching the prim type name "BrepArray"
// (the defining marker of a BrepArray prim).
USD_BREP_EXPORT std::vector<UsdPrim> FindBrepArrayPrims(const UsdStageRefPtr& rStage);

// Load a single BrepArray prim into a UsdBrepArrayData.
// Returns true on success.
USD_BREP_EXPORT bool LoadBrepArray(const UsdPrim& rPrim, UsdBrepArrayData& rArrays);

// Open a USD file and load all BrepArray prims found in the stage.
// Results are appended as (SdfPath, unique_ptr<UsdBrepArrayData>) pairs.
// unique_ptr is used because UsdBrepArrayData is non-copyable/non-movable.
// Returns true if the file was opened successfully (even if no BrepArray prims were found).
USD_BREP_EXPORT bool LoadAllBrepArrays(const std::string& sUsdFilePath, std::vector<std::pair<SdfPath, std::unique_ptr<UsdBrepArrayData>>>& rResults);

// Write a UsdBrepArrayData to an existing BrepArray prim.
// The prim must already exist on the stage. Returns true on success.
USD_BREP_EXPORT bool SaveBrepArray(const UsdBrepArrayData& rArrays, UsdPrim& rPrim);

} // end namespace UsdBrep

#endif // _USD_BREP_STAGE_UTILS_H_
