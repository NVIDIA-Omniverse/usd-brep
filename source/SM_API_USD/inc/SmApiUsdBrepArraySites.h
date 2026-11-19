// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdBrepArraySites.h
* PURPOSE: Find the prim specs that define a stage's BrepArray geometry.
*          Used by SmApiUsdHeal.h.
**********************************************************************/

#ifndef _SM_API_USD_BREP_ARRAY_SITES_H_
#define _SM_API_USD_BREP_ARRAY_SITES_H_

#include "SmApiUsdConfig.h"
#include <SmMessages.h>

#include "UsdBrepHeaders.h" // pxr headers with warning suppression (BREP_USD_DATA)
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <string>
#include <vector>

/// A prim spec that defines BrepArray geometry. Changing it changes every prim that uses it.
struct SmApiUsdBrepArraySite
{
    pxr::SdfLayerHandle        m_sLayer;      ///< layer that holds the spec
    pxr::SdfPath               m_sSpecPath;   ///< path of the spec in m_sLayer
    std::vector<pxr::SdfPath>  m_sPrimPaths;  ///< BrepArray prims that use it, including instance proxies
    std::string                m_sProblem;    ///< why it cannot be rewritten in place; empty if it can
};

/// Find the specs that define the stage's BrepArray prims, and why any cannot be rewritten in place.
/// @return SM_SUCCESS; SM_ERR_NULL_POINTER for a null stage
SM_API_USD_EXPORT SmStatus SmApiUsdFindBrepArraySites
(
    const pxr::UsdStagePtr                  & crStage,
    std::vector<SmApiUsdBrepArraySite>      & rSites
);

#endif // _SM_API_USD_BREP_ARRAY_SITES_H_
