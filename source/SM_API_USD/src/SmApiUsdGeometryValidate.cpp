// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdGeometryValidate.cpp
* PURPOSE: Import a BrepArray prim's Breps and run SmBrep::AssertValid on each.
**********************************************************************/

#include "SmApiUsdGeometryValidate.h"

#include "UsdBrepSuppressPixarWarningsPush.h"
#include <pxr/usd/usdGeom/gprim.h>
#include "UsdBrepSuppressPixarWarningsPop.h"

#include "SmuConvert.h"
#include "SmAssertArray.h"
#include "SmBrep.h"
#include "SmContext.h"

#include <algorithm>
#include <chrono>
#include <memory>
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
