// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepUtilities.cpp
 * PURPOSE: encapsulated USD utilities copied from usdlib
 * ******************************************************************************************************************/

#include "UsdBrepUtilities.h"

#include "UsdBrepDiagnostics.h"
#include "UsdBrepTokens.h"

// USD includes
#include "UsdBrepHeaders.h"

#include <pxr/usd/usd/schemaRegistry.h>

#include <cassert>
#include <filesystem>

/******************************************************************/
/* Below are functions adapted from OpenUSD,                    */
/* so we have access to the Sdf authoring capabilities.         */
/******************************************************************/

/*********************************************************************************************************************
 PURPOSE: (copied from _GetInstanceNamePlaceholder)

 NOTES: return ref to static std::string "__INSTANCE_NAME__"
 ********************************************************************************************************************/
const std::string& usdBrep_GetInstanceNamePlaceholder()
{
    static std::string instanceNamePlaceHolder("__INSTANCE_NAME__");
    return instanceNamePlaceHolder;
}

/*********************************************************************************************************************
 PURPOSE: Finds the first occurrence of the instance name placeholder that is
          fully contained as a single substring between namespace delimiters
          (including the beginning and ends of the name template).

 RETURN: start position of "__INSTANCE_NAME__" substring in nameTemplate if found
         else std::string::npos

 NOTES: (copied from _FindInstanceNamePlaceholder)
 ********************************************************************************************************************/
std::string::size_type usdBrep_FindInstanceNamePlaceholder(const std::string& nameTemplate)
{
    static const std::string::size_type placeholderSize = usdBrep_GetInstanceNamePlaceholder().size();
    std::string::size_type substrStart = 0;
    while (substrStart < nameTemplate.size())
    {
        // The substring ends at the next delimeter (or the end of the name
        // template if no next delimiter is found).
        std::string::size_type substrEnd = nameTemplate.find(':', substrStart);
        if (substrEnd == std::string::npos)
        {
            substrEnd = nameTemplate.size();
        }
        // If the substring is an exact full word match with the instance name
        // placeholder, return the beginning of this substring.
        if (substrEnd - substrStart == placeholderSize &&
            nameTemplate.compare(substrStart, placeholderSize, usdBrep_GetInstanceNamePlaceholder()) == 0)
        {
            return substrStart;
        }
        // Otherwise move to the next substring which starts after the namespace
        // delimiter.
        substrStart = substrEnd + 1;
    }
    return std::string::npos;
} // end usdBrep_FindInstanceNamePlaceholder

/*********************************************************************************************************************
 PURPOSE: build token name for multiple applied API instance

 NOTES: replaces the string "__INSTANCE_NAME__" in rPropertyToken with
        rInstanceToken string value.
        Ex: rInstanceToken = UsdBrepSolidTokens->shellPoint        = "shellPoint"
            rPropertyToken = UsdBrepSolidTokens->brepPointPosition = "brep:__INSTANCE_NAME__:point:position"
            returnToken    = "brep:shellPoint:point:position"
 ********************************************************************************************************************/
pxr::TfToken UsdBrepData::GetNameSpacePropertyToken(
    const pxr::TfToken& rInstanceToken, // in : multiApplied API instance name
                                        //      ex: UsdBrepSolidTokens->shellPoint for "shellPoint"
    const pxr::TfToken& rPropertyToken // in : attribute name from an multiApplied API
                                       //      ex: UsdBrepSolidTokens->brepPointPosition for "brep:__INSTANCE_NAME__:point:position"
)
{
    const std::string::size_type pos = usdBrep_FindInstanceNamePlaceholder(rPropertyToken.GetString());
    if (pos == std::string::npos)
    {
        return rPropertyToken;
    }
    std::string result = rPropertyToken;
    result.replace(pos, usdBrep_GetInstanceNamePlaceholder().size(), rInstanceToken.GetString());
    return pxr::TfToken(result);

} // end UsdBrepData::GetNameSpacePropertyToken

/*********************************************************************************************************************
 PURPOSE: Register 'omniSolid/resources' plugin given a relative path

 NOTES: Store the plugin's path to omnisolid/resources in environment variable OMNISOLID_PLUGIN_PATH
 ********************************************************************************************************************/
bool UsdBrepData::RegisterOmniSolidResourcesPlugin()
{
    // get plugin path from environment variable
    const char* envValue = std::getenv("OMNISOLID_PLUGIN_PATH");
    const std::string sPluginPath = envValue ? envValue : "";

    if (sPluginPath == "")
    {
        USDBREP_ERROR("UsdBrepData::RegisterOmniSolidResourcesPlugin(): Environment variable OMNISOLID_PLUGIN_PATH not set");
        return false;
    }

    // Convert string to path
    std::filesystem::path sPluginDir = std::filesystem::absolute(sPluginPath);

    // Verify the directory exists
    if (!std::filesystem::exists(sPluginDir))
    {
        USDBREP_ERROR("UsdBrepData::RegisterOmniSolidResourcesPlugin(): USD OmniSolid plugin directory %s not found", sPluginDir.string().c_str());
        return false;
    }

    // Register the USD plugins - catch errors
    try
    {
        pxr::PlugPluginPtrVector registeredPlugins = pxr::PlugRegistry::GetInstance().RegisterPlugins(sPluginDir.string());

        if (registeredPlugins.empty())
        {
            // check state - empty registeredPlugins because plugins have already been registered - that's okay
            pxr::PlugPluginPtrVector plugins = pxr::PlugRegistry::GetInstance().GetAllPlugins();
            for (const pxr::PlugPluginPtr& plugin : plugins)
            {
                // get plugin->path and switch from windows to linux path delimiters
                std::string path = plugin->GetPath();
                std::replace(path.begin(), path.end(), '\\', '/');

                // get sPluginDir string and switch from windows to linux path delimiters
                std::string pluginDirStr = sPluginDir.string();
                std::replace(pluginDirStr.begin(), pluginDirStr.end(), '\\', '/');

                if (path.find(pluginDirStr) != std::string::npos)
                {
                    return true;
                }
            }

            // registration failed but not because plugins are already registered - error
            USDBREP_ERROR(
                "%s",
                ("UsdBrepData::RegisterOmniSolidResourcesPlugin(): RegisterPlugins returned empty vector for directory " + sPluginDir.string() +
                 "; the plugins were not registered.")
                    .c_str()
            );
            return false;
        }
    }
    catch (const std::exception& e)
    {
        USDBREP_ERROR("%s", ("UsdBrepData::RegisterOmniSolidResourcesPlugin(): RegisterPlugins failed: " + std::string(e.what())).c_str());
        return false;
    }


    return true;

} // end UsdBrepData::RegisterOmniSolidResourcesPlugin

/*********************************************************************************************************************
 PURPOSE: checks if 'omniSolid/resources' plugin is registered

 NOTES: Use UsdBrepData::RegisterOmniSolidResourcesPlugin to register the plugin at startup.
 ********************************************************************************************************************/
bool UsdBrepData::IsOmniSolidResourcesPluginRegistered()
{
    const std::string partialPath = "omniSolid/resources";
    pxr::PlugPluginPtrVector plugins = pxr::PlugRegistry::GetInstance().GetAllPlugins();
    for (const pxr::PlugPluginPtr& plugin : plugins)
    {
        // Check both the plugin name and the plugin path for the substring
        std::string name = plugin->GetName();
        std::string path = plugin->GetPath();
        std::replace(name.begin(), name.end(), '\\', '/');
        std::replace(path.begin(), path.end(), '\\', '/');

        if (name.find(partialPath) != std::string::npos || path.find(partialPath) != std::string::npos)
        {
            return true;
        }
    }
    return false;

} // end UsdBrepData::IsOmniSolidResourcesPluginRegistered

/*********************************************************************************************************************
 PURPOSE: checks that OpenUSD knows the 'omniSolid/resources' applied-API schemas

 NOTES: FALSE if the plugin was registered after OpenUSD cached its schema types, e.g. when opening a stage.
 ********************************************************************************************************************/
bool UsdBrepData::AreOmniSolidSchemasKnown()
{
    return !pxr::UsdSchemaRegistry::GetTypeFromSchemaTypeName(pxr::UsdBrepSurfaceTokens->brepSurfaceNurbAPI).IsUnknown();

} // end UsdBrepData::AreOmniSolidSchemasKnown

/*********************************************************************************************************************
 PURPOSE: Add the specified single apply schema to primSpec

 NOTES: Assumed: 'omni/solid' plugin registered.
         - if the omni/solid plugin is not registered, this function will return false
           and subsequent AppliedSchema attribute access will fail.
         - Register 'omniSolid/resources' plugin with UsdBrepData::RegisterOmniSolidResourcesPlugin
 ********************************************************************************************************************/
bool UsdBrepData::AddAppliedSchema(const pxr::SdfPrimSpecHandle& rPrimSpecHandle, const pxr::TfToken& rAppliedSchemaToken)
{
    // check state - no input
    if (!rPrimSpecHandle)
    {
        return false;
    }

    // check state - Verify 'omniSolid/resources' is registered
    assert(IsOmniSolidResourcesPluginRegistered());

    // declare _HasItem()
    auto _HasItem = [](const pxr::TfTokenVector& rItems, const pxr::TfToken& rItem)
    {
        return (std::find(rItems.begin(), rItems.end(), rItem) != rItems.end());
    };

    // get listOp() ptr
    pxr::VtValue apiSchemasVal = rPrimSpecHandle->GetInfo(pxr::UsdTokens->apiSchemas);
    if (!apiSchemasVal.IsHolding<pxr::SdfTokenListOp>())
    {
        USDBREP_ERROR("%s", ("Failed to get apiSchemas as SdfTokenListOp for prim " + rPrimSpecHandle->GetPath().GetString()).c_str());
        return false;
    }
    pxr::SdfTokenListOp listOp = apiSchemasVal.Get<pxr::SdfTokenListOp>();

    // explicit listOp branch
    if (listOp.IsExplicit())
    {
        // If the list op is explicit we check if the explicit item to see if
        // our name is already in it. We'll add it to the end of the explicit
        // list if it is not.
        const pxr::TfTokenVector& rItems = listOp.GetExplicitItems();
        if (_HasItem(rItems, rAppliedSchemaToken))
        {
            return true;
        }
        // Use ReplaceOperations to append in place.
        if (!listOp.ReplaceOperations(pxr::SdfListOpTypeExplicit, rItems.size(), 0, { rAppliedSchemaToken }))
        {
            return false;
        }
    }
    else // NonExplicit listOp branch
    {
        // Otherwise our name could be in the append or prepend list (we
        // purposefully ignore the "add" list which is deprecated) so we check
        // both before adding it to the end of prepends.
        const pxr::TfTokenVector& rPreItems = listOp.GetPrependedItems();
        const pxr::TfTokenVector& rAppItems = listOp.GetAppendedItems();

        if (_HasItem(rPreItems, rAppliedSchemaToken) || _HasItem(rAppItems, rAppliedSchemaToken))
        {
            return true;
        }
        // Use ReplaceOperations to append in place.
        if (!listOp.ReplaceOperations(pxr::SdfListOpTypePrepended, rPreItems.size(), 0, { rAppliedSchemaToken }))
        {
            return false;
        }
    }

    // If we got here, we edited the list op, so author it back to the spec.
    rPrimSpecHandle->SetInfo(pxr::UsdTokens->apiSchemas, pxr::VtValue::Take(listOp));
    return true;

} // end UsdBrepData::AddAppliedSchema single applied API

/*********************************************************************************************************************
 PURPOSE: Add the specified multiple apply schema to primSpec

 NOTES:
 ********************************************************************************************************************/
bool UsdBrepData::AddMultiAppliedSchema(
    const pxr::SdfPrimSpecHandle& rPrimSpecHandle,
    const pxr::TfToken& rAppliedSchemaToken,
    const pxr::TfToken& rInstanceToken
)
{
    // The callers of this function will have already validated the schema is
    // mulitple apply either through static asserts or runtime validation
    // _ValidateMultipleApplySchemaType.

    // check state - NonEmpty InstanceToken, rInstanceToken can only be validated at runtime.
    // All API schema functions treat an empty instance for a multiple apply schema as a coding error.
    if (rInstanceToken.IsEmpty())
    {
        return false;
    }

    // check state - Verify 'omniSolid/resources' is registered
    assert(IsOmniSolidResourcesPluginRegistered());

    // The Following is an edited comment from pxr
    //
    // Validate the primSpec to protect against crashes in the
    // Apply functions when called with a null primSpec
    //
    // While we don't typically validate "this" primSpec for public UsdPrim C++ API,
    // for performance reasons, we opt to do so here since AddAppliedSchema isn't
    // performance critical. If AddAppliedSchema becomes performance critical in the
    // future, we may have to move this validation elsewhere if this validation
    // is problematic.
    // if (!primSpec)
    // {
    //     // SE_MSG(SM_ERR, "Invalid primSpec ");
    //     return false;
    // }

    // const pxr::TfToken typeName = pxr::UsdSchemaRegistry::GetSchemaTypeName(schemaType);
    std::string sAppliedInstanceName = pxr::SdfPath::JoinIdentifier(rAppliedSchemaToken, rInstanceToken);
    if (sAppliedInstanceName.empty())
    {
        USDBREP_ERROR(
            "%s",
            ("Failed to create applied instance name for schema " + rAppliedSchemaToken.GetString() + " with instance " + rInstanceToken.GetString())
                .c_str()
        );
        return false;
    }
    pxr::TfToken sAppliedSchemaToken(sAppliedInstanceName);
    return AddAppliedSchema(rPrimSpecHandle, sAppliedSchemaToken);

} // end UsdBrepData::AddAppliedSchema - multiple applied API

/*********************************************************************************************************************
 PURPOSE: Append a value to an owner's target attribute spec

 NOTES:
 ********************************************************************************************************************/
pxr::SdfAttributeSpecHandle UsdBrepData::setAttributeSpec(
    const pxr::SdfPrimSpecHandle& rOwnerPrimSpecHandle,
    const pxr::TfToken& rToken,
    const pxr::SdfValueTypeName& rTypeName,
    const pxr::VtValue& rValue,
    pxr::SdfVariability eVariability,
    bool bCustom
)
{
    // fetch target attribute from Owner
    pxr::SdfPath propertyPath = rOwnerPrimSpecHandle->GetPath().AppendProperty(rToken);
    if (propertyPath.IsEmpty())
    {
        USDBREP_ERROR(
            "%s",
            ("Failed to create property path for token " + rToken.GetString() + " on prim " + rOwnerPrimSpecHandle->GetPath().GetString()).c_str()
        );
        return pxr::SdfAttributeSpecHandle();
    }
    pxr::SdfAttributeSpecHandle attributeSpecHandle = rOwnerPrimSpecHandle->GetAttributeAtPath(propertyPath);

    // when target attribute does not exist - add it to Owner
    if (!attributeSpecHandle)
    {
        attributeSpecHandle = pxr::SdfAttributeSpec::New(rOwnerPrimSpecHandle, rToken, rTypeName, eVariability, bCustom);
        if (!attributeSpecHandle)
        {
            USDBREP_ERROR(
                "%s",
                ("Failed to create new attribute spec for token " + rToken.GetString() + " on prim " + rOwnerPrimSpecHandle->GetPath().GetString())
                    .c_str()
            );
            return pxr::SdfAttributeSpecHandle();
        }
    }

    // set target attribute value
    bool setResult = attributeSpecHandle->SetDefaultValue(rValue);
    if (!setResult)
    {
        USDBREP_ERROR(
            "%s",
            ("Failed to set default value for attribute " + rToken.GetString() + " on prim " + rOwnerPrimSpecHandle->GetPath().GetString()).c_str()
        );
        return pxr::SdfAttributeSpecHandle();
    }

    // all done
    return attributeSpecHandle;

} // end UsdBrepData::setAttributeSpec

/*********************************************************************************************************************
 PURPOSE: Append a value to an owner's target attribute spec for a multiple applied API

 NOTES:
 ********************************************************************************************************************/
pxr::SdfAttributeSpecHandle UsdBrepData::setMultiAppliedAttributeSpec(
    const pxr::SdfPrimSpecHandle& rOwnerPrimSpecHandle,
    const pxr::TfToken& rInstanceToken,
    const pxr::TfToken& rPropertyToken,
    const pxr::SdfValueTypeName& rTypeName,
    const pxr::VtValue& rValue,
    pxr::SdfVariability variability,
    bool custom
)
{
    // Find the first occurrence of the instance name placeholder and replace
    // it with the instance name if found.
    pxr::TfToken nameSpacePropertyToken = GetNameSpacePropertyToken(rInstanceToken, rPropertyToken);


    return UsdBrepData::setAttributeSpec(rOwnerPrimSpecHandle, nameSpacePropertyToken, rTypeName, rValue, variability, custom);

} // end UsdBrepData::setMultiAppliedAttributeSpec
