// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepUtilities.h
 * PURPOSE: Header file for USD utilities copied from usdlib
 * ******************************************************************************************************************/

#ifndef _USD_BREP_UTILITIES_H_
#define _USD_BREP_UTILITIES_H_

#include "UsdBrepConfig.h"

// USD includes
#include "UsdBrepHeaders.h"

namespace UsdBrepData
{

// build token name for multiple applied API instance
USDBREP_EXPORT pxr::TfToken GetNameSpacePropertyToken(const pxr::TfToken& rInstanceToken, const pxr::TfToken& rPropName);

// Register the 'omniSolid/resources' plugin found through the input relative path. Registering the
// plugin is required prior to Adding AppliedAPISchemas for the BrepArray shape representations
USDBREP_EXPORT bool RegisterOmniSolidResourcesPlugin();

// Check if the 'omniSolid/resources' plugin is registered with the pxr::PlugRegistry.
// Returns true if registered, false otherwise.
USDBREP_EXPORT bool IsOmniSolidResourcesPluginRegistered();

// Check that OpenUSD knows the BrepArray schemas: false if the plugin was registered after a stage was opened.
// Call it only when reading: like opening a stage, it fixes OpenUSD's schema types for the rest of the process.
USDBREP_EXPORT bool AreOmniSolidSchemasKnown();

// Add the specified single apply schema to primSpec
USDBREP_EXPORT bool AddAppliedSchema(const pxr::SdfPrimSpecHandle& rPrimSpecHandle, const pxr::TfToken& rAppliedSchemaToken);

// Add the specified multiple apply schema to primSpec
USDBREP_EXPORT bool AddMultiAppliedSchema(
    const pxr::SdfPrimSpecHandle& rPrimSpecHandle,
    const pxr::TfToken& rAppliedSchemaToken,
    const pxr::TfToken& rInstanceToken
);

// On prim 'owner' set attribute 'token' with type 'typeName' to value 'value' for singleApplied API
USDBREP_EXPORT pxr::SdfAttributeSpecHandle setAttributeSpec(
    const pxr::SdfPrimSpecHandle& rOwnerPrimSpecHandle,
    const pxr::TfToken& rToken,
    const pxr::SdfValueTypeName& rTypeName,
    const pxr::VtValue& rValue,
    pxr::SdfVariability eVariability = pxr::SdfVariabilityUniform,
    bool bCustom = false
);

// On prim 'owner' set attribute 'token' with type 'typeName' to value 'value' for multipleApplied API
USDBREP_EXPORT pxr::SdfAttributeSpecHandle setMultiAppliedAttributeSpec(
    const pxr::SdfPrimSpecHandle& rOwnerPrimSpecHandle,
    const pxr::TfToken& rInstanceToken,
    const pxr::TfToken& rPropToken,
    const pxr::SdfValueTypeName& rTypeName,
    const pxr::VtValue& rValue,
    pxr::SdfVariability variability = pxr::SdfVariabilityUniform,
    bool custom = false
);

}; // end namespace UsdBrepData

#endif // no _USD_BREP_UTILITIES_H_
