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
#include <pxr/usd/usd/prim.h>

#include <cstddef>
#include <string>
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

#endif // _SM_API_USD_GEOMETRY_VALIDATE_H_
