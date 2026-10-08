// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepKnots.h"

#include <algorithm>
#include <cmath>

namespace UsdBrep
{

void CompressFlatKnots(const std::vector<double>& rFlatKnots, std::vector<double>& rOutKnots, std::vector<int>& rOutMults, double dTolerance)
{
    rOutKnots.clear();
    rOutMults.clear();
    for (double dKnot : rFlatKnots)
    {
        if (!rOutKnots.empty() && std::fabs(dKnot - rOutKnots.back()) <= dTolerance)
        {
            ++rOutMults.back();
        }
        else
        {
            rOutKnots.push_back(dKnot);
            rOutMults.push_back(1);
        }
    }
}

int PeriodicExtraPoles(const std::vector<int>& rMults, int iDegree, bool bPeriodic)
{
    if (!bPeriodic || rMults.empty())
    {
        return 0;
    }
    const int iExtra = (iDegree + 1) - rMults.front();
    return iExtra > 0 ? iExtra : 0;
}

pxr::VtArray<double> BuildFlatKnots(const std::vector<double>& rKnots, const std::vector<int>& rMults, int iDegree, bool bPeriodic)
{
    pxr::VtArray<double> sFlat;
    const size_t uiK = std::min(rKnots.size(), rMults.size());
    if (uiK == 0)
    {
        return sFlat;
    }

    int iSumMults = 0;
    for (size_t i = 0; i < uiK; ++i)
    {
        iSumMults += rMults[i] > 0 ? rMults[i] : 0;
    }

    if (!bPeriodic)
    {
        sFlat.reserve(static_cast<size_t>(iSumMults));
        for (size_t i = 0; i < uiK; ++i)
        {
            const int iMult = rMults[i] > 0 ? rMults[i] : 0;
            for (int m = 0; m < iMult; ++m)
            {
                sFlat.push_back(rKnots[i]);
            }
        }
        return sFlat;
    }

    const int iM1 = PeriodicExtraPoles(rMults, iDegree, true);
    const int iLen = iSumMults + 2 * iM1;
    sFlat.assign(static_cast<size_t>(iLen), 0.0);

    // Middle section: the expanded distinct knots, offset by M1.
    int iIdx = iM1;
    for (size_t i = 0; i < uiK; ++i)
    {
        const int iMult = rMults[i] > 0 ? rMults[i] : 0;
        for (int m = 0; m < iMult; ++m)
        {
            sFlat[static_cast<size_t>(iIdx++)] = rKnots[i];
        }
    }

    const double dPeriod = rKnots[uiK - 1] - rKnots[0];

    // Leading section: walk backwards from the second-to-last distinct knot, shifted by -period.
    {
        int m = 1;
        int j = static_cast<int>(uiK) - 2;
        for (int i = iM1 - 1; i >= 0; --i)
        {
            // Clamp before use: j can be driven to -1 by the decrement below when there are
            // fewer distinct interior knots than leading poles, which would index rKnots OOB.
            if (j < 0)
            {
                j = 0;
            }
            sFlat[static_cast<size_t>(i)] = rKnots[static_cast<size_t>(j)] - dPeriod;
            ++m;
            if (j >= 0 && m > rMults[static_cast<size_t>(j)])
            {
                --j;
                m = 1;
            }
        }
    }

    // Trailing section: walk forwards from the second distinct knot, shifted by +period.
    {
        int m = 1;
        int j = 1;
        for (int i = iIdx; i < iLen; ++i)
        {
            // Clamp before use: j can reach uiK via the increment below when there are fewer
            // distinct interior knots than trailing poles, which would index rKnots OOB.
            if (j >= static_cast<int>(uiK))
            {
                j = static_cast<int>(uiK) - 1;
            }
            sFlat[static_cast<size_t>(i)] = rKnots[static_cast<size_t>(j)] + dPeriod;
            ++m;
            if (j < static_cast<int>(uiK) && m > rMults[static_cast<size_t>(j)])
            {
                ++j;
                m = 1;
            }
        }
    }

    return sFlat;
}

} // end namespace UsdBrep
