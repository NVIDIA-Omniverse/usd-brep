// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuAttribute.cpp
* PURPOSE: Implementation for SmuAttribute classes
**********************************************************************/

#include "UsdBrepConfig.h"
#include "UsdBrepTokens.h"

#include "UsdBrepUtilities.h"
#include "UsdBrepDebugTools.h"
#include "SmuUtilities.h"
#include "SmuAttribute.h"

// pixar includes
#include "UsdBrepSuppressPixarWarningsPush.h"                 // turn off known compile warnings for code owned by pixar
#         include <pxr/usd/usdShade/tokens.h>  
#include "UsdBrepSuppressPixarWarningsPop.h"                  // done loading pixar code - restore suspended compile warnings


 /*******************************************************************//**
 PURPOSE: Attach Path attributes to a UsdBrepArraySpecHandle
 
 NOTES: primSpec is a SdfPrimSpecHandle to the OmniSolidBrepArray
        rInstanceToken is the BrepAPI instance name
        Only extant attributes are set
 ***********************************************************************/
 SMU_EXPORT bool SmSdfPathAttribute::AttachAttributeToBrepInstance
  (const pxr::SdfPrimSpecHandle & , // in : crUsdBrepArraySpecHandle
   pxr::TfToken & )                 // in : rInstanceToken
 {
     //if (m_lAttributeID == SM_AI_MATERIAL_BINDING)
     //{
     //    SdfRelationshipSpecHandle brepMaterialRel = crUsdBrepArraySpecHandle->GetRelationshipAtPath(crUsdBrepArraySpecHandle->GetPath().AppendProperty(SMU_BrepConvert::GetNamespacedPropertyName(
     //                                     rInstanceToken, UsdBrepSolidTokens::brep_MultipleApplyTemplate_BrepMaterialBinding)));
     //    if (!brepMaterialRel)
     //    {
     //        brepMaterialRel = SdfRelationshipSpec::New( crUsdBrepArraySpecHandle, SMU_BrepConvert::GetNamespacedPropertyName(
     //                rInstanceToken, UsdBrepSolidTokens::brep_MultipleApplyTemplate_BrepMaterialBinding), false);
     //    }
     //    brepMaterialRel->GetTargetPathList().ClearEditsAndMakeExplicit();
     //    brepMaterialRel->GetTargetPathList().Add(GetMaterialPaths()->front());
     //}
     return true;
 } // end SmSdfPathAttribute::AttachAttributeToBrepInstance
 
 /*******************************************************************//**
 PURPOSE: Attach Xform attributes to a UsdBrepArraySpecHandle
 
 NOTES: - Used when moving data from SMLib -> USD
        - primSpec is a SdfPrimSpecHandle to a UsdGeomXformable
        - Only extant attributes are set
 ***********************************************************************/
 SMU_EXPORT bool SmSdfPathAttribute::AttachAttributeToXformable
  (const pxr::SdfPrimSpecHandle& crUsdBrepArraySpecHandle)
 {
     // Set the mesh material binding
     if (m_lAttributeID == SM_AI_MATERIAL_BINDING)
       {
         bool addSchemaResult = UsdBrepData::AddAppliedSchema(crUsdBrepArraySpecHandle, 
                                                 pxr::UsdSchemaRegistry::GetSchemaTypeName(pxr::TfType::Find<pxr::UsdShadeMaterialBindingAPI>()));
         if (!addSchemaResult)
         {
             std::string sstring = "Error: Failed to add UsdShadeMaterialBindingAPI schema to BrepArray\n";
             usdBrep_WriteString(sstring);
         }

         pxr::SdfRelationshipSpecHandle sMaterialBindingHandle = pxr::SdfRelationshipSpec::New(crUsdBrepArraySpecHandle, 
                                                                                              pxr::UsdShadeTokens->materialBinding, 
                                                                                              false);
         if (!sMaterialBindingHandle)
         {
             std::string sstring = "Error: Failed to create material binding relationship spec for BrepArray\n";
             usdBrep_WriteString(sstring);
         }
         else
         {
             sMaterialBindingHandle->GetTargetPathList().ClearEditsAndMakeExplicit();
         }
         if (GetMaterialPaths()->size() > 0 && sMaterialBindingHandle)
         {
             sMaterialBindingHandle->GetTargetPathList().Add(GetMaterialPaths()->front());
         }
       }

     // Set the mesh backpointer to source Brep
     else if (m_lAttributeID == SM_AI_BREP_ARRAY && (GetMaterialPaths()->size() > 0))
       {
         pxr::SdfPath                   sBrepPath(GetMaterialPaths()->front());
         pxr::SdfRelationshipSpecHandle sBrepSourceRelSpecHandle = pxr::SdfRelationshipSpec::New(crUsdBrepArraySpecHandle, 
                                                                                                pxr::TfToken("BrepSource"), 
                                                                                                true);
         sBrepSourceRelSpecHandle->GetTargetPathList().ClearEditsAndMakeExplicit();
         sBrepSourceRelSpecHandle->GetTargetPathList().Add(sBrepPath);
 
         pxr::SdfRelationshipSpecHandle derivedMeshRel = crUsdBrepArraySpecHandle->GetLayer()->GetRelationshipAtPath(sBrepPath);
         if (derivedMeshRel)
           { 
             derivedMeshRel->GetTargetPathList().Add(crUsdBrepArraySpecHandle->GetPath()); 
           }
       }
     return true;
 } // end SmSdfPathAttribute::AttachAttributeToXformable

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
SMU_EXPORT void SmSdfPathAttribute::Dump(void) const
{  
  ULONG lNumPaths = m_sMaterialPaths.empty() ? 0 : m_sMaterialPaths.size() ;

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmSdfPathAttribute[0x%p]: ID[%ld], Number of Values[%d], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              this, 
              m_lAttributeID,
              lNumPaths,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_sprintf( sBuffForFile, _T("SmSdfPathAttribute: ID[%ld], Number of Values[%d], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_lAttributeID,
              lNumPaths,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

// Print all SdfPath strings using sprintf
 if( m_sMaterialPaths.empty()) 
   { smos_sprintf(sBuff, _T("%s"),  _T("\n  No material paths found.")); smos_WriteBuffer(sBuff); }
 else 
   { ULONG ii = 0 ; 
     for (const pxr::SdfPath & path : m_sMaterialPaths)
       {
         std::string pathStr = path.GetString();
         smos_sprintf(sBuff, _T("\n  materialPath[%d]: %s"), ii++, pathStr.c_str());
       }
   }

} // end SmSdfPathAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SMU_EXPORT SmBoolean SmSdfPathAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmSdfPathAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Copy constructor for the UsdXform Attribute

NOTES: 
***********************************************************************/
SMU_EXPORT SmUsdXformAttribute::SmUsdXformAttribute(const SmUsdXformAttribute & crOriginal)
: SmAttribute(crOriginal.m_lAttributeID,
              crOriginal.m_eBehavior)
{
  // deep copies
  m_sTranslate    = crOriginal.m_sTranslate ;
  m_sScale        = crOriginal.m_sScale ;
  m_sRotateXYZ    = crOriginal.m_sRotateXYZ ;
  m_sRotateXZY    = crOriginal.m_sRotateXZY ;
  m_sRotateYXZ    = crOriginal.m_sRotateYXZ ;
  m_sRotateYZX    = crOriginal.m_sRotateYZX ;
  m_sRotateZXY    = crOriginal.m_sRotateZXY ;
  m_sRotateZYX    = crOriginal.m_sRotateZYX ;
  m_dRotateX      = crOriginal.m_dRotateX ;
  m_dRotateY      = crOriginal.m_dRotateY ;
  m_dRotateZ      = crOriginal.m_dRotateZ ;
  m_sOrient       = crOriginal.m_sOrient ;
  m_sTransform    = crOriginal.m_sTransform ;
  m_sXformOpOrder = crOriginal.m_sXformOpOrder ;

} // end SmUsdXformAttribute::SmUsdXformAttribute

 /*******************************************************************//**
 PURPOSE: Attach Xform attributes to a BrepAPI instance
 
 NOTES: primSpec is a SdfPrimSpecHandle to the OmniSolidBrepArray
        tInstanceName is the BrepAPI instance name
        Only extant attributes are set
 ***********************************************************************/
 SMU_EXPORT bool SmUsdXformAttribute::AttachAttributeToBrepInstance
  (const pxr::SdfPrimSpecHandle  & crUsdBrepArraySpecHandle, 
   pxr::TfToken           &rtInstanceName )
 {
  SM_REF2(crUsdBrepArraySpecHandle, rtInstanceName);
  //if (GetXformOpTransform())
  //    SMU_BrepConvert::setAttributeSpec(crUsdBrepArraySpecHandle, tInstanceName, UsdBrepSolidTokens::brep_MultipleApplyTemplate_XformOpTransform,
  //                     SdfValueTypeNames->Matrix4dArray, VtValue(*GetXformOpTransform()), SdfVariabilityUniform, false);
  return true;
 } // end SmUsdXformAttribute::AttachAttributeToBrepInstance
 
 /*******************************************************************//**
 PURPOSE: Attach Xform attributes to a UsdGeomXformable primSpec
 
 NOTES: primSpec is a SdfPrimSpecHandle to a UsdGeomXformable
        Only extant attributes are set
 ***********************************************************************/
 SMU_EXPORT bool SmUsdXformAttribute::AttachAttributeToXformable
  (const pxr::SdfPrimSpecHandle & crPrimSpecHandle)
 {
   SM_REF1(crPrimSpecHandle);
     //if (GetXformOpTransform())
     //    SMU_BrepConvert::setAttributeSpec(crPrimSpecHandle, UsdBrepSolidTokens::XformOpTransform, SdfValueTypeNames->Matrix4dArray,
     //                                      VtValue(*GetXformOpTransform()), SdfVariabilityUniform, false);
     return true;
 } // end SmUsdXformAttribute::AttachAttributeToXformable

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
SMU_EXPORT void SmUsdXformAttribute::Dump(void) const
{  
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf( sBuff, _T("SmUsdXformAttribute[0x%p]: ID[%ld], NumOps[%ld], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              this, m_lAttributeID,
              m_sXformOpOrder.size(),
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_sprintf( sBuffForFile, _T("SmUsdXformAttribute: ID[%ld], NumOps[%ld], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_lAttributeID,
              m_sXformOpOrder.size(),
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmUsdXformAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SMU_EXPORT SmBoolean SmUsdXformAttribute::IsKindOf( SM_TYPE t ) const
{
return ((SmUsdXformAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}
