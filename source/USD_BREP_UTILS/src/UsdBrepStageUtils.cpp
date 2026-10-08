// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepStageUtils.h"

#include "UsdBrepRead.h"
#include "UsdBrepTokens.h"
#include "UsdBrepWrite.h"

#include <pxr/usd/usd/primRange.h>

#include <memory>

namespace UsdBrep
{
PXR_NAMESPACE_USING_DIRECTIVE
using namespace UsdBrepData;

/*********************************************************************************************************************
 * FindBrepArrayPrims: traverse the stage and collect prims whose type name is
 * "BrepArray", which is the defining marker of a BrepArray prim.
 ********************************************************************************************************************/
std::vector<UsdPrim> FindBrepArrayPrims(const UsdStageRefPtr& rStage)
{
    std::vector<UsdPrim> results;

    if (!rStage)
    {
        return results;
    }

    for (const UsdPrim& prim : rStage->TraverseAll())
    {
        if (prim.GetTypeName() == TfToken("BrepArray"))
        {
            results.push_back(prim);
        }
    }

    return results;
}

/*********************************************************************************************************************
 * LoadBrepArray: read a single BrepArray prim into a UsdBrepArrayData.
 ********************************************************************************************************************/
bool LoadBrepArray(const UsdPrim& rPrim, UsdBrepArrayData& rArrays)
{
    if (!rPrim)
    {
        return false;
    }

    rArrays.ReSet();
    return BrepReadFromUsdStage(rPrim, rArrays);
}

/*********************************************************************************************************************
 * LoadAllBrepArrays: open a USD file and load every BrepArray prim found.
 * Returns true if the stage was opened successfully.
 ********************************************************************************************************************/
bool LoadAllBrepArrays(const std::string& sUsdFilePath, std::vector<std::pair<SdfPath, std::unique_ptr<UsdBrepArrayData>>>& rResults)
{
    UsdStageRefPtr stage = UsdStage::Open(sUsdFilePath);
    if (!stage)
    {
        return false;
    }

    std::vector<UsdPrim> prims = FindBrepArrayPrims(stage);

    for (const UsdPrim& prim : prims)
    {
        auto pArrays = std::make_unique<UsdBrepArrayData>();
        if (BrepReadFromUsdStage(prim, *pArrays))
        {
            rResults.emplace_back(prim.GetPath(), std::move(pArrays));
        }
    }

    return true;
}

/*********************************************************************************************************************
 * SaveBrepArray: write a UsdBrepArrayData to an existing BrepArray prim.
 ********************************************************************************************************************/
bool SaveBrepArray(const UsdBrepArrayData& rArrays, UsdPrim& rPrim)
{
    if (!rPrim)
    {
        return false;
    }

    return BrepWriteToUsdStage(rArrays, rPrim);
}

} // end namespace UsdBrep
