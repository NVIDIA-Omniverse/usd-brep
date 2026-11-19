// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
/******************************************************************//**
* FILE NAME --- SmApiUsdGeometryValidate.h
* PURPOSE: Check that the Breps in a BrepArray prim are geometrically and
*          topologically sound, as bin/brep_geometry_validator does. This is
*          SMLib's SmBrep::AssertValid, not the BrepArray schema validator.
**********************************************************************/

#ifndef _SM_API_USD_GEOMETRY_VALIDATE_H_
#define _SM_API_USD_GEOMETRY_VALIDATE_H_

#include "SmApiUsdConfig.h"
#include <SmMessages.h>

#include "UsdBrepHeaders.h" // pxr headers with warning suppression (BREP_USD_DATA)
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

/// One failed check.
struct SmApiUsdGeometryFailure
{
    std::size_t  m_lBrep = 0;  ///< index of the Brep in the BrepArray
    std::string  m_sOwner;     ///< type of the object that failed, e.g. SmFace
    std::string  m_sCheck;     ///< name of the check, e.g. UV Domain
    std::string  m_sDetail;    ///< what the check found
};

/// The result of validating one BrepArray prim.
struct SmApiUsdGeometryResult
{
    std::size_t                           m_lBrepCount = 0;        ///< Breps imported
    std::size_t                           m_lInvalidBreps = 0;     ///< Breps that failed a check
    std::vector<SmApiUsdGeometryFailure>  m_sFailures;             ///< in Brep order
    std::string                           m_sMessage;              ///< why the import failed; empty if it did not
    double                                m_dImportSeconds = 0.0;
    double                                m_dValidateSeconds = 0.0;
};

/// Import a BrepArray prim's Breps and run SmBrep::AssertValid on each, without changing the stage.
/// iLevel is the AssertValid depth: 0 is fastest, 2 runs every check. The Breps are not healed: healing
/// can repair the defects being looked for, and the healer can crash on invalid Breps. Several threads can
/// validate different prims at once while the stage is not modified.
/// @return SM_SUCCESS when every Brep imported, whether or not it is valid; SM_ERR_INVALID_INPUT
///         for an iLevel outside 0-2; SM_ERR otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdGeometryValidate
(
    const pxr::UsdPrim                      & crBrepArray,
    int                                       iLevel,
    SmApiUsdGeometryResult                  & rResult
);

/// Find the prims that SmApiUsdGeometryValidate should check: the prims UsdStage::Traverse visits (active, defined,
/// not abstract) that have the BrepArray type or a brep:regionCount attribute, including those inside instance
/// prototypes. The prims of a prototype are listed once, under a stable path made from the smallest authored instance
/// path. Sorted by path.
/// @param pVisited if not null, receives the number of prims visited
/// @return SM_SUCCESS; SM_ERR_NULL_POINTER for a null stage
SM_API_USD_EXPORT SmStatus SmApiUsdFindGeometryPrims
(
    const pxr::UsdStagePtr                                & crStage,
    std::vector<std::pair<pxr::SdfPath, pxr::UsdPrim>>    & rPrims,
    std::size_t                                           * pVisited = nullptr
);

#endif // _SM_API_USD_GEOMETRY_VALIDATE_H_
