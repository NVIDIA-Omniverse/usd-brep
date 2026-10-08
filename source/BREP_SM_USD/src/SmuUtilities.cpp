// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuUtilities.cpp
* PURPOSE: Implementation for SMLib to USD conversion utilities
**********************************************************************/

#include "UsdBrepConfig.h"
#include "UsdBrepUtilities.h"
#include "UsdBrepDebugTools.h"

#include "SmuUtilities.h"

// pixar includes
#include "UsdBrepSuppressPixarWarningsPush.h" // turn off compile warnings for problematic pixar include files
#         include <pxr/usd/usdShade/materialBindingAPI.h>
#include "UsdBrepSuppressPixarWarningsPop.h"  // done loading problematic pixar headers - restore compile warnings

// SMLib includes
#include "SmBrep.h"
#include "SmBSplineSurface.h"

#include <sstream>

using namespace SMU_BrepConvert;

/*********************************************************************************************************************/ /**
 PURPOSE:

 NOTES:
 *********************************************************************************************************************/
static std::string FilterUsdName(const std::string& nameIn)
{
    // Don't allow for a blank name
    if (nameIn.size() == 0)
        return "Default";

    std::stringstream sStream;
    sStream << std::hex;
    // Cannot start with a number
    if (nameIn[0] >= '0' && nameIn[0] <= '9')
        sStream << "_";

    // Remove any non alphanumberic data from the string and replace it with the appropriate hex code
    const char* c = nameIn.c_str();
    for (; *c; ++c)
    {
        if (!((*c >= '0' && *c <= '9') || (*c >= 'A' && *c <= 'Z') || (*c >= 'a' && *c <= 'z') || *c == '_'))
            sStream << "_0x" << unsigned(*c) << "_";
        else
            sStream << *c;
    }

    // Convert to wstring
    std::string sString(sStream.str());

    // Convert to string for output
    return std::string(sString.begin(), sString.end());

} // end FilterUsdName

/*******************************************************************//**
PURPOSE: Convenience function to apply an Xform to an SmCurve

NOTES:
***********************************************************************/
SmStatus SMU_BrepConvert::ApplyUsdXformOp
 (SmCurve                   & rCurve,    // in : Curve to which the transform is applied                     
  const pxr::UsdGeomXformOp & crXformOp) // in : Xform to apply to rCurve                                    
{
    // Locals
    SmVector3d sScale;
    SmAxis2Placement sTransform;

    // Get SMLib type transform data
    SMU_BrepConvert::UsdXformToSmTransform(crXformOp, sScale, sTransform);

    // Apply the transformation
    SER(rCurve.Transform(sTransform, &sScale));

    return SM_SUCCESS;
} // end SMU_BrepConvert::ApplyUsdXformOp

/*******************************************************************//**
PURPOSE: Convenience function to apply an Xform to an SmBSplineSurface

NOTES:
***********************************************************************/
SmStatus SMU_BrepConvert::ApplyUsdXformOp
( SmBSplineSurface          & rBSSurface,  // in : Surface to which the transform is applied                     
  const pxr::UsdGeomXformOp & crXformOp)   // in : Xform to apply to rBSSurface                                  
{
    // Locals
    SmVector3d sScale;
    SmAxis2Placement sTransform;

    // Get SMLib type transform data
    SMU_BrepConvert::UsdXformToSmTransform(crXformOp, sScale, sTransform);

    // Apply the transformation
    SER(rBSSurface.Transform(sTransform, &sScale));

    return SM_SUCCESS;
} // end SMU_BrepConvert::ApplyUsdXformOp

/*******************************************************************//**
PURPOSE: Convenience function to apply an Xform to an SmBrep

NOTES:
***********************************************************************/
SmStatus SMU_BrepConvert::ApplyUsdXformOp
( SmBrep                    & rBrep,       // in : Brep to which the transform is applied                        
  const pxr::UsdGeomXformOp & crXformOp)   // in : Xform to apply to rBrep                                    
{
    // Locals
    SmVector3d sScale;
    SmAxis2Placement sTransform;

    // Get SMLib type transform data
    SMU_BrepConvert::UsdXformToSmTransform(crXformOp, sScale, sTransform);

    // Apply the transformation
    SER(rBrep.Transform(sTransform, &sScale));

    return SM_SUCCESS;
} // end SMU_BrepConvert::ApplyUsdXformOp

/*******************************************************************//**
PURPOSE: Creates a GfMatrix4d transformation from an SmScale and SmAxis2Place

NOTES: SMLib follows the convention that the scale is applied before
    the transformation. GfMatrix4d assumes row vectors
***********************************************************************/
SmStatus SMU_BrepConvert::SmTransformToUsdXform
( const SmVector3d       & rScale,      // in : Scaling                                                    
  const SmAxis2Placement & rTransform,  // in : Rotation and translation                                   
        pxr::GfMatrix4d  & r4x4)        // out: Transformation as 4x4 matrix                               
{
    // SMLib Locals
    SmVector3d xAxis, yAxis, zAxis;
    SmPoint3d sOrigin;

    // Get and calculate the normalized vectors
    rTransform.GetCanonical(sOrigin, xAxis, yAxis);
    zAxis = xAxis * yAxis;

    // Set rXformOp to fleshed out 4x4 transform
    // This transform may look transposed because of where the rScale terms show up
    // But - its been tested and its the correct mapping between SMLib and USD transforms
    r4x4.Set(rScale.x * xAxis.x, rScale.y * xAxis.y, rScale.z * xAxis.z, 0.,
             rScale.x * yAxis.x, rScale.y * yAxis.y, rScale.z * yAxis.z, 0.,
             rScale.x * zAxis.x, rScale.y * zAxis.y, rScale.z * zAxis.z, 0., 
             sOrigin.x,          sOrigin.y,          sOrigin.z, 1.);

    return SM_SUCCESS;

} // end SMU_BrepConvert::SmTransformToUsdXform

/*******************************************************************//**
PURPOSE: Sets an XformOp transformation from an SmScale and SmAxis2Place

NOTES: SMLib follows the convention that the scale is applied before
    the transformation. GfMatrix4d assumes row vectors
***********************************************************************/
SmStatus SMU_BrepConvert::SmTransformToUsdXform
( const SmVector3d          & rScale,      // in : Scaling                                                      
  const SmAxis2Placement    & rTransform,  // in : Rotation and translation                                   
        pxr::UsdGeomXformOp & rXformOp)    // out: Transformation as UsdGeomXformOp                           
{
    // SMLib Locals
    pxr::GfMatrix4d sUsd4x4;
    pxr::UsdTimeCode UTC = 0.;

    // make the conversion
    SmTransformToUsdXform(rScale, rTransform, sUsd4x4);

    // Set rXformOp to fleshed out 4x4 transform
    rXformOp.Set(sUsd4x4, UTC);

    return SM_SUCCESS;

} // end SMU_BrepConvert::SmTransformToUsdXform

/*******************************************************************//**
PURPOSE: Creates an SmScale and SmAxis2Placement for scaling and rotation
    from an XformOp

NOTES: SMLib follows the convention that the scale is applied before
    the transformation. GfMatrix4d assumes row vectors
***********************************************************************/
SmStatus SMU_BrepConvert::UsdXformToSmTransform
( const pxr::UsdGeomXformOp & crXformOp,   // in : Xform that will be converted to SMLib standard             
  SmVector3d                & rScale,      // out: Scaling output                                             
  SmAxis2Placement          & rTransform)  // out: Rotation and translation                                    
{
    // Locals
    pxr::GfVec4d x, y, z;
    pxr::UsdTimeCode UTC = 0.;
    pxr::GfMatrix4d xformOpTransform = crXformOp.GetOpTransform(UTC);

    // Get the important entries of the 4x4 transform matrix
    x = xformOpTransform.GetColumn(0);
    y = xformOpTransform.GetColumn(1);
    z = xformOpTransform.GetColumn(2);

    double xScale = xformOpTransform.GetRow3(0).GetLength();
    double yScale = xformOpTransform.GetRow3(1).GetLength();
    double zScale = xformOpTransform.GetRow3(2).GetLength();

    SmPoint3d sOrigin(x[3], y[3], z[3]);
    SmVector3d xAxis(x[0]/xScale, y[0]/yScale, z[0]/zScale);
    SmVector3d yAxis(x[1]/xScale, y[1]/yScale, z[1]/zScale);

    // Set the scaling and transformation entries in SMLib format
    rScale.Set(xScale, yScale, zScale);
    rTransform.SetCanonical( sOrigin,    // origin
                             xAxis,    // xAxis
                             yAxis);   // yAxis
                                       // zAxis implied by xAxis * yAxis
    return SM_SUCCESS;

} // end SMU_BrepConvert::UsdXformToSmTransform

/******************************************************************/
/* Below are functions stripped from OpenUSD, such that we have   */
/* access to the sdf authoring capabilities.                      */
/* This allows us to use the SdfChangeBlock feature.              */
/******************************************************************/

/*******************************************************************//**
PURPOSE: 

NOTES: on failure, returns SdfPrimSpecHandle()
***********************************************************************/
pxr::SdfPrimSpecHandle SMU_BrepConvert::smuGenerateMeshMetadata
 (const pxr::SdfPrimSpecHandle & rParentSpec,
  const std::string              sName,
  std::string                  * pOptDisplayName,
  pxr::GfMatrix4d              & rBrepMat,
  bool                           bDoubleSided,
  ULONG                          uIndex,
  const pxr::SdfPath             materialPath)
{
    static const pxr::TfToken xformOpTransform("xformOp:transform", pxr::TfToken::Immortal);

    std::string sUSDName = FilterUsdName(sName) + "_Mesh";
    std::string sNewName;
    if (uIndex > 0)
    {
        sNewName = sUSDName + "_derived_" + std::to_string(uIndex);
    }
    else
    {
        sNewName = sUSDName;
    }

    pxr::SdfPrimSpecHandle sNewBrepMeshSpecHandle = pxr::SdfPrimSpec::New(rParentSpec, 
                                                                          sNewName, 
                                                                          pxr::SdfSpecifierDef,  
                                                                          "Mesh");
    if (!sNewBrepMeshSpecHandle)
    {
        // return NULL PrimSpecHandle when New failed
        return pxr::SdfPrimSpecHandle();  
    }

    if (pOptDisplayName)
    {
        sNewBrepMeshSpecHandle->GetLayer()->SetField(sNewBrepMeshSpecHandle->GetPath(), 
                                               pxr::SdfFieldKeys->DisplayName, 
                                               *pOptDisplayName + "_" + std::to_string(uIndex));
    }

    pxr::SdfAttributeSpecHandle transformAttr = UsdBrepData::setAttributeSpec(sNewBrepMeshSpecHandle, 
                                    xformOpTransform, 
                                    pxr::SdfValueTypeNames->Matrix4d, 
                                    pxr::VtValue(rBrepMat));
    if (!transformAttr)
    {
        std::string sstring = "Error: Failed to set transform attribute on BrepMesh\n";
        usdBrep_WriteString(sstring);
    }

    pxr::SdfAttributeSpecHandle xformOpOrderAttr = UsdBrepData::setAttributeSpec(sNewBrepMeshSpecHandle, 
                                    pxr::UsdGeomTokens->xformOpOrder, 
                                    pxr::SdfValueTypeNames->TokenArray,
                                   pxr::VtValue(pxr::VtArray<pxr::TfToken>({ xformOpTransform })));
    if (!xformOpOrderAttr)
    {
        std::string sstring = "Error: Failed to set xformOpOrder attribute on BrepMesh\n";
        usdBrep_WriteString(sstring);
    }

    if (materialPath != pxr::SdfPath::EmptyPath())
    {
        bool addSchemaResult = UsdBrepData::AddAppliedSchema
                        (sNewBrepMeshSpecHandle, 
                         pxr::UsdSchemaRegistry::GetSchemaTypeName(pxr::TfType::Find<pxr::UsdShadeMaterialBindingAPI>()));
        if (!addSchemaResult)
        {
            std::string sstring = "Error: Failed to add UsdShadeMaterialBindingAPI schema to BrepMesh\n";
            usdBrep_WriteString(sstring);
        }

        pxr::SdfRelationshipSpecHandle materialBindingHandle = pxr::SdfRelationshipSpec::New(sNewBrepMeshSpecHandle, 
                                                                                             pxr::UsdShadeTokens->materialBinding, 
                                                                                             false);

        materialBindingHandle->GetTargetPathList().ClearEditsAndMakeExplicit();
        materialBindingHandle->GetTargetPathList().Add(materialPath);
    }

    pxr::SdfAttributeSpecHandle doubleSidedAttr = UsdBrepData::setAttributeSpec(sNewBrepMeshSpecHandle, 
                                   pxr::UsdGeomTokens->doubleSided, 
                                   pxr::SdfValueTypeNames->Bool,
                                   pxr::VtValue(bDoubleSided));
    if (!doubleSidedAttr)
    {
        std::string sstring = "Error: Failed to set doubleSided attribute on BrepMesh\n";
        usdBrep_WriteString(sstring);
    }

    pxr::SdfAttributeSpecHandle subdivisionSchemeAttr = UsdBrepData::setAttributeSpec(sNewBrepMeshSpecHandle, 
                                   pxr::UsdGeomTokens->subdivisionScheme, 
                                   pxr::SdfValueTypeNames->Token,
                                   pxr::VtValue(pxr::UsdGeomTokens->none), 
                                   pxr::SdfVariabilityUniform, 
                                   false);
    if (!subdivisionSchemeAttr)
    {
        std::string sstring = "Error: Failed to set subdivisionScheme attribute on BrepMesh\n";
        usdBrep_WriteString(sstring);
    }

    return sNewBrepMeshSpecHandle;

} // end SMU_BrepConvert::smuGenerateMeshMetadata

/*******************************************************************//**
PURPOSE: 

NOTES: on failure, returns SdfPrimSpecHandle()
***********************************************************************/
pxr::SdfPrimSpecHandle SMU_BrepConvert::smuGenerateBrepMetadata
 (const pxr::SdfPrimSpecHandle & rParentSpec,
  const std::string              sName,
  std::string                  * pOptDisplayName,
  pxr::GfMatrix4d              & rBrepMat,
  ULONG                          uIndex,
  const pxr::SdfPath             materialPath)
{
    static const pxr::TfToken xformOpTransform("xformOp:transform", pxr::TfToken::Immortal);

    std::string sUSDName = FilterUsdName(sName) + "_Brep";
    std::string sNewName;

    if (uIndex > 0)
    {
        sNewName = sUSDName + "_derived_" + std::to_string(uIndex);
    }
    else
    {
        sNewName = sUSDName;
    }

    pxr::SdfPrimSpecHandle sNewBrepSpecHandle = pxr::SdfPrimSpec::New(rParentSpec, 
                                                                      sNewName, 
                                                                      pxr::SdfSpecifierDef, 
                                                                      "BrepArray");
    if (!sNewBrepSpecHandle)
    {
        // return NULL PrimSpecHandle when New fails
        return pxr::SdfPrimSpecHandle();
    }

    if (pOptDisplayName)
    {
        sNewBrepSpecHandle->GetLayer()->SetField(
            sNewBrepSpecHandle->GetPath(), pxr::SdfFieldKeys->DisplayName, *pOptDisplayName + "_" + std::to_string(uIndex));
    }

    pxr::SdfAttributeSpecHandle transformAttr = UsdBrepData::setAttributeSpec(sNewBrepSpecHandle, 
                                   xformOpTransform, 
                                   pxr::SdfValueTypeNames->Matrix4d, 
                                   pxr::VtValue(rBrepMat));
    if (!transformAttr)
    {
        std::string sstring = "Error: Failed to set transform attribute on Brep\n";
        usdBrep_WriteString(sstring);
    }

    pxr::SdfAttributeSpecHandle xformOpOrderAttr = UsdBrepData::setAttributeSpec(sNewBrepSpecHandle, 
                                   pxr::UsdGeomTokens->xformOpOrder, 
                                   pxr::SdfValueTypeNames->TokenArray,
                                   pxr::VtValue(pxr::VtArray<pxr::TfToken>({ xformOpTransform })));
    if (!xformOpOrderAttr)
    {
        std::string sstring = "Error: Failed to set xformOpOrder attribute on Brep\n";
        usdBrep_WriteString(sstring);
    }

    if (materialPath != pxr::SdfPath::EmptyPath())
    {
        bool addSchemaResult = UsdBrepData::AddAppliedSchema(sNewBrepSpecHandle,
                                                pxr::UsdSchemaRegistry::GetSchemaTypeName
                                                       (pxr::TfType::Find<pxr::UsdShadeMaterialBindingAPI>()));
        if (!addSchemaResult)
        {
            std::string sstring = "Error: Failed to add UsdShadeMaterialBindingAPI schema to Brep\n";
            usdBrep_WriteString(sstring);
        }

        pxr::SdfRelationshipSpecHandle materialBindingHandle = pxr::SdfRelationshipSpec::New(sNewBrepSpecHandle, 
                                                                                             pxr::UsdShadeTokens->materialBinding, 
                                                                                             false);

        materialBindingHandle->GetTargetPathList().ClearEditsAndMakeExplicit();
        materialBindingHandle->GetTargetPathList().Add(materialPath);
    }

    return sNewBrepSpecHandle;

} // end SMU_BrepConvert::smuGenerateBrepMetadata
