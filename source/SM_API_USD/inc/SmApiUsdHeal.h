// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdHeal.h
* PURPOSE: Heal BrepArray prims on an open USD stage. SmApiUsdHealFile
*          (SmApiUsd.h) is built on these.
**********************************************************************/

#ifndef _SM_API_USD_HEAL_H_
#define _SM_API_USD_HEAL_H_

#include "SmApiUsdConfig.h"
#include <SmMessages.h>

#include "UsdBrepHeaders.h" // pxr headers with warning suppression (BREP_USD_DATA)
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace UsdBrepData
{
class UsdBrepArrayData;
}

/// A prim spec that defines BrepArray geometry. Healing it heals every prim that uses it.
struct SmApiUsdBrepArraySite
{
    pxr::SdfLayerHandle        m_sLayer;      ///< layer that holds the spec
    pxr::SdfPath               m_sSpecPath;   ///< path of the spec in m_sLayer
    std::vector<pxr::SdfPath>  m_sPrimPaths;  ///< BrepArray prims that use it, including instance proxies
    std::string                m_sProblem;    ///< why it cannot be healed in place; empty if it can
};

/// Find the specs that define the stage's BrepArray prims, and why any cannot be healed in place.
/// @return SM_SUCCESS; SM_ERR_NULL_POINTER for a null stage
SM_API_USD_EXPORT SmStatus SmApiUsdFindBrepArraySites
(
    const pxr::UsdStagePtr                  & crStage,
    std::vector<SmApiUsdBrepArraySite>      & rSites
);

/// The healed geometry of one BrepArray.
struct SmApiUsdHealedBrepArray
{
    pxr::SdfPath                                         m_sPrimPath;
    std::string                                          m_sMessage;            ///< why it failed
    std::size_t                                          m_lBrepCount = 0;
    std::shared_ptr<const UsdBrepData::UsdBrepArrayData> m_pArrays;             ///< for SmApiUsdWriteHealedBrepArray; null if it failed
};

/// Heal a BrepArray prim's Breps without changing the stage. Best effort: failed heal steps and
/// unfixed problems are not reported. Thread-safe while the stage is not modified.
/// @return SM_SUCCESS when every Brep imported; SM_ERR otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdHealBrepArray
(
    const pxr::UsdPrim                      & crBrepArray,
    SmApiUsdHealedBrepArray                 & rHealed
);

/// Write healed geometry at the prim's spec in the root layer, which must be the edit target.
/// Write only sites without a problem. rMessage says why it failed.
/// @return SM_SUCCESS; SM_ERR_NULL_POINTER for a null stage; SM_ERR_INVALID_INPUT, writing nothing,
///         for an unsupported scene; SM_ERR if writing fails, possibly part way
SM_API_USD_EXPORT SmStatus SmApiUsdWriteHealedBrepArray
(
    const SmApiUsdHealedBrepArray           & crHealed,
    const pxr::UsdStagePtr                  & crStage,
    std::string                             & rMessage
);

#endif // _SM_API_USD_HEAL_H_
