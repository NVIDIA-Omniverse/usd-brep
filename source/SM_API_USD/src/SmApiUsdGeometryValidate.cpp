// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdGeometryValidate.cpp
* PURPOSE: Import a BrepArray prim's Breps and run SmBrep::AssertValid on each.
**********************************************************************/

#include "SmApiUsdGeometryValidate.h"
#include "SmApiUsd.h"

#include "UsdBrepSuppressPixarWarningsPush.h"
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/gprim.h>
#include "UsdBrepSuppressPixarWarningsPop.h"

#include "SmuConvert.h"
#include "SmAssertArray.h"
#include "SmBrep.h"
#include "SmContext.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <map>
#include <set>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

namespace
{

double SecondsSince(std::chrono::steady_clock::time_point sStart)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - sStart).count();
}

SmStatus Validate(const UsdPrim& crBrepArray, int iLevel, SmApiUsdGeometryResult& rResult)
{
    // One SmContext per prim: deleting Breps while reusing a shared context corrupts
    // kernel marks and caches.
    SmContext sContext;
    std::vector<SmBrep*> vImported;
    auto sStart = std::chrono::steady_clock::now();
    // Don't heal: healing can repair the defects being looked for, and the healer can crash on invalid Breps.
    const SmStatus sStat = SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, UsdGeomGprim(crBrepArray), vImported,
                                                                /*bHealerIsEnabled*/ FALSE);
    rResult.m_dImportSeconds = SecondsSince(sStart);
    std::vector<std::unique_ptr<SmBrep>> vBreps(vImported.begin(), vImported.end());
    if (sStat != SM_SUCCESS)
    {
        rResult.m_sMessage = "import failed, status=" + std::to_string(sStat);
        return SM_ERR;
    }
    if (vBreps.empty() || std::find(vBreps.begin(), vBreps.end(), nullptr) != vBreps.end())
    {
        rResult.m_sMessage = "import produced missing Breps";
        return SM_ERR;
    }

    sStart = std::chrono::steady_clock::now();
    for (std::size_t lBrep = 0; lBrep < vBreps.size(); ++lBrep)
    {
        SmAssertArray sAList;
        if (!vBreps[lBrep]->AssertValid(&sAList, static_cast<SmAssertTestLevel>(iLevel)))
        {
            ++rResult.m_lInvalidBreps;
            for (ULONG jj = 0; jj < sAList.GetSize(); ++jj)
            {
                const SmAssertReport* pRep = sAList[jj];
                if (pRep == nullptr)
                {
                    continue;
                }
                TCHAR sMsg[SM_TBLOCK_SIZE];
                pRep->FormatLogMessage(sMsg, SM_TBLOCK_SIZE);
                rResult.m_sFailures.push_back(
                    { lBrep, smos_FromTChar(pRep->m_sOwnerTypeString ? pRep->m_sOwnerTypeString : _T("?")),
                      smos_FromTChar(pRep->m_pName ? pRep->m_pName : _T("?")), smos_FromTChar(sMsg) });
            }
        }
    }
    rResult.m_lBrepCount = vBreps.size();
    rResult.m_dValidateSeconds = SecondsSince(sStart);
    return SM_SUCCESS;
}

} // namespace

SmStatus SmApiUsdGeometryValidate(const UsdPrim& crBrepArray, int iLevel, SmApiUsdGeometryResult& rResult)
{
    rResult = SmApiUsdGeometryResult();
    if (iLevel < 0 || iLevel > 2)
    {
        return SM_ERR_INVALID_INPUT;
    }
    // Report a kernel exception as a failure, as SmApiUsdHealBrepArray does.
    try
    {
        return Validate(crBrepArray, iLevel, rResult);
    }
    catch (const std::exception& error)
    {
        rResult = SmApiUsdGeometryResult();
        rResult.m_sMessage = error.what();
    }
    catch (...)
    {
        rResult = SmApiUsdGeometryResult();
        rResult.m_sMessage = "unexpected kernel exception";
    }
    return SM_ERR;
}

SmStatus SmApiUsdFindGeometryPrims(const UsdStagePtr& crStage, std::vector<std::pair<SdfPath, UsdPrim>>& rPrims, std::size_t* pVisited)
{
    rPrims.clear();
    if (pVisited)
    {
        *pVisited = 0;
    }
    if (!crStage)
    {
        return SM_ERR_NULL_POINTER;
    }
    std::size_t lVisited = 0;
    std::map<SdfPath, UsdPrim> sBreps;
    std::map<SdfPath, UsdPrim> sPendingPrototypes;
    std::set<SdfPath> sVisitedPrototypes;

    auto isBrep = [](const UsdPrim& sPrim)
    {
        return sPrim.HasAttribute(TfToken("brep:regionCount")) || sPrim.GetTypeName() == "BrepArray";
    };

    // Seed a sorted work queue with active authored instances.
    for (const UsdPrim& sPrim : crStage->Traverse())
    {
        ++lVisited;
        if (!sPrim.IsActive())
        {
            continue;
        }
        if (sPrim.IsInstance())
        {
            sPendingPrototypes.emplace(sPrim.GetPath(), sPrim.GetPrototype());
        }
        if (isBrep(sPrim))
        {
            sBreps.emplace(sPrim.GetPath(), sPrim);
        }
    }

    // The smallest authored path reaches each prototype first, so shared
    // prototype contents are traversed and imported only once.
    while (!sPendingPrototypes.empty())
    {
        const auto sNext = sPendingPrototypes.begin();
        const SdfPath sPrototypeKey = sNext->first;
        const UsdPrim sPrototype = sNext->second;
        sPendingPrototypes.erase(sNext);
        if (!sPrototype || !sVisitedPrototypes.insert(sPrototype.GetPath()).second)
        {
            continue;
        }

        for (const UsdPrim& sPrim : UsdPrimRange(sPrototype))
        {
            ++lVisited;
            if (!sPrim.IsActive())
            {
                continue;
            }
            const SdfPath sRelativePath = sPrim.GetPath().MakeRelativePath(sPrototype.GetPath());
            const SdfPath sStablePath = sPrototypeKey.AppendPath(sRelativePath);
            if (sPrim.IsInstance())
            {
                sPendingPrototypes.emplace(sStablePath, sPrim.GetPrototype());
            }
            if (isBrep(sPrim))
            {
                sBreps.emplace(sStablePath, sPrim);
            }
        }
    }

    rPrims.assign(sBreps.begin(), sBreps.end());
    if (pVisited)
    {
        *pVisited = lVisited;
    }
    return SM_SUCCESS;
}

SmStatus SmApiUsdValidateGeometryFile(const char* pInputFile, int iLevel, std::vector<SmApiUsdGeometryPrimResult>& rResults)
{
    rResults.clear();
    if (!pInputFile)
    {
        return SM_ERR_NULL_POINTER;
    }
    // Kernel messages can hold non-UTF-8 bytes, which Python cannot decode.
    auto ascii = [](std::string s)
    {
        std::replace_if(s.begin(), s.end(), [](char c) { return static_cast<unsigned char>(c) > 127; }, '?');
        return s;
    };
    auto fail = [&](SmStatus sStat, const std::string& sMessage)
    {
        rResults.assign(1, SmApiUsdGeometryPrimResult{ "", {}, sMessage });
        return sStat;
    };
    if (iLevel < 0 || iLevel > 2)
    {
        return fail(SM_ERR_INVALID_INPUT, "level must be 0, 1 or 2");
    }
    if (SmApiUsdEnsurePluginRegistered() != SM_SUCCESS)
    {
        return fail(SM_ERR, "cannot register omniSolid schema");
    }
    const UsdStageRefPtr sStage = UsdStage::Open(pInputFile);
    if (!sStage)
    {
        return fail(SM_ERR, std::string("cannot open input: ") + pInputFile);
    }
    std::vector<std::pair<SdfPath, UsdPrim>> vPrims;
    const SmStatus sFindStat = SmApiUsdFindGeometryPrims(sStage, vPrims);
    if (sFindStat != SM_SUCCESS)
    {
        return fail(sFindStat, std::string("cannot search for BrepArray prims in: ") + pInputFile);
    }
    for (const auto& rFound : vPrims)
    {
        SmApiUsdGeometryResult sResult;
        SmApiUsdGeometryValidate(rFound.second, iLevel, sResult);
        SmApiUsdGeometryPrimResult& rPrim = rResults.emplace_back();
        rPrim.m_sPrimPath = rFound.first.GetString();
        rPrim.m_sMessage = ascii(sResult.m_sMessage);
        for (const SmApiUsdGeometryFailure& rFailure : sResult.m_sFailures)
        {
            rPrim.m_sFailures.push_back(ascii(rFailure.m_sCheck + ": Brep " + std::to_string(rFailure.m_lBrep) + " " +
                                              rFailure.m_sOwner + ": " + rFailure.m_sDetail));
        }
    }
    return rResults.empty() ? fail(SM_ERR_INVALID_INPUT, std::string("no BrepArray prims found in ") + pInputFile)
                            : SM_SUCCESS;
}
