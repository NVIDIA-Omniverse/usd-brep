// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************/ /**
* FILE NAME --- usd_test.cpp
* PURPOSE: Source file for testing of SMLib / USD interface
**********************************************************************/

#include "usd_test.h"

// BREP_SM_USD
#include "SmuConvert.h"
#include "SmuTessellate.h"
#include "UsdBrepUtilities.h"
#include "SmuUtilities.h"
#include "UsdBrepTokens.h"
#include "SmuTokens.h"
#include "SmuAttribute.h"

// SMLib
#include "SmBrep.h"
#include "SmPoly.h"
#include "SmPrimitiveCreation.h"
#include "SmAttribute.h"
#include "SmSmlibAll.h"
#include "SmMerge.h"
#include "SmCrvOnSurf.h"

// omni_solid_brep_data
#include "UsdBrepArrayData.h"
#include "UsdBrepWrite.h"
#include "UsdBrepRead.h"
#include "UsdBrepDebugTools.h"

#ifdef APPEND_GIT_BRANCH
//#    include "../../../_build/generated/SmGitBranch.h"

#    define Q(x) #x
#    define QUOTE(x) Q(x)
#endif // APPEND_GIT_BRANCH

#ifdef SM_USE_TBB
#include "tbb/parallel_for.h"
#endif // SM_USE_TBB

// turn off compile warnings for problematic pixar include files
#include "UsdBrepSuppressPixarWarningsPush.h"                 // turn off compile warnings for problematic pixar include files
#         include <pxr/usd/usdGeom/xform.h>
#         include <pxr/usd/usd/editContext.h>
#         include <pxr/usd/usd/modelAPI.h>
#         include <pxr/usd/usd/primRange.h>
#         include <pxr/usd/usd/variantSets.h>
#         include <pxr/usd/kind/registry.h>
#         include <pxr/usd/sdf/variantSpec.h>
#         include <pxr/usd/usdShade/materialBindingAPI.h>
#         include <pxr/base/plug/registry.h>
#         include <pxr/base/plug/plugin.h>
#include "UsdBrepSuppressPixarWarningsPop.h"                  // done loading problematic pixar headers - restore compile warnings

#include <string>
#include <cstdio>
#include <cwchar>
#include "codecvt"

using namespace pxr;
using namespace UsdBrepData;

// Maps from UsdPrims to the created SmBreps and Assemblies
SmMapTypeToType<size_t, SmBrep*> sBrepMap;
SmMapTypeToType<size_t, SmAssembly*> sAssemblyMap;

static void DeleteSmBreps(std::vector<SmBrep*>& rBreps)
{
    for (SmBrep* pBrep : rBreps)
    {
        delete pBrep;
    }
    rBreps.clear();
}

class UsdImportMapScope
{
public:
    UsdImportMapScope()
    {
        sBrepMap.RemoveAll();
        sAssemblyMap.RemoveAll();
    }

    ~UsdImportMapScope()
    {
        sBrepMap.RemoveAll();
        sAssemblyMap.RemoveAll();
    }
};

/***********************************************************************
PURPOSE --- translate brep USD file to SMLib array of breps

NOTES ---
***********************************************************************/
SmStatus TranslateUsdToSmlib(
 const std::string sFilename, 
 const SmContext& sContext, 
 SmTArray<SmBrep*>& rBreps
)
{
    if (false == RegisterOmniSolidResourcesPlugin())
    {
        return SM_ERR; // no msg here - already done in RegisterOmniSolidResourcesPlugin()
    }

    // Get SmBreps from input usd file
    return (ImportFromUsd(sFilename, sContext, rBreps));

} // end TranslateUsdToSmlib

/***********************************************************************
PURPOSE --- Recursively convert every BrepArray prim under sRootPrimIn to SmBreps, appending them to rBreps.

NOTES --- Recursion assumes all children of the brep belong to it. That is, any nurbs patch that lives within the
          sDbgUsdBrepArray hierarchy will not be read in separate from the brep read.
          Similarly, nurbsPatches are assumed to not be nested.
          Argument pBrep is a collector of loose geometry.  Any nurbs patches that don't exist within a brep
          Will be turned into SmFaces within pBrep. pBrep is returned as an item in rBreps.
***********************************************************************/
SmStatus GetBrepsFromUsd(UsdPrim& sRootPrimIn, const SmContext& crContext, SmTArray<SmBrep*>& rBreps, SmBrep* pBrep)
{
    // locals
    SmBoolean bHealerIsEnabled = FALSE; // TRUE = healer is enabled. FALSE= disable healer

    // iter every RootPrim child 
    for (auto child : sRootPrimIn.GetAllChildren())
    {
        const UsdPrimTypeInfo& crPrimTypeInfo = child.GetPrimTypeInfo();

        // when the child is a BrepArray
        if (crPrimTypeInfo.GetTypeName() == TfToken("BrepArray"))
        {
            // move USD Breps to SMLib Breps
            std::vector<SmBrep*> sBreps;
            SER(SMU_BrepConvert::BrepMove_UsdToSMLib(
                crContext, (UsdGeomGprim)child, sBreps, bHealerIsEnabled));

            // Transfer the completely converted BrepArray to the output.
            for (SmBrep* pNewBrep : sBreps)
            {
                if (pNewBrep != NULL)
                {
                    // temporary debugging
                    // SmBoolean bIsAnal = pNewBrep->IsAnalytic();
                    // SM_DUMP_AND_ASSERT_VALID(pNewBrep);
                    rBreps.Add(pNewBrep);
                }
            }
            // Note: sBreps objects are transferred to rBreps, so no cleanup needed here
        }

        // recurse looking for descendant USD Brep prims
        SER(GetBrepsFromUsd(child, crContext, rBreps, pBrep));
    }

    return SM_SUCCESS;
} // end GetBrepsFromUsd

/***********************************************************************
PURPOSE ---

NOTES --- Assumes input prim
          * IsA<UsdGeomXform>
          * IsA "assembly" kind
***********************************************************************/
SmStatus BuildSmAssembly(
    UsdPrim* prim,
    const SmContext& crContext,
    const UsdStage& sStage,
    SmTArray<SmBrep*>& rBreps)
{
    // Get the assembly
    SmAssembly* pAssembly = sAssemblyMap.GetValueAt(prim->GetPath().GetHash());
    NER(pAssembly);

    // Locals
    std::string sName;
    std::wstring sWideName;
    TfToken tKind;
    SmVector3d sScale;
    SmAxis2Placement sTransform;

    // for each child populate pAssembly with AssemblyInstance pointing to Brep or SubAssembly
    for (UsdPrim instance : prim->GetChildren())
    {

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if (bDebugMe)
        {
            // Dump prim path
            std::string sstring = TfStringPrintf("\nprim = %s\n", instance.GetPrimPath().GetAsString().c_str());
            usdBrep_WriteString(sstring);
        }
#endif // SM_DEBUG_CODE

        // Get the prim sBrepArrayName
        sName = instance.GetPrimPath().GetName();
        sWideName = std::wstring(sName.begin(), sName.end());

        // Reset the transforms
        sScale.Set(1., 1., 1.);
        sTransform.SetCanonical(
            SmPoint3d(0., 0., 0.), // origin
            SmVector3d(1., 0., 0.), // x axis
            SmVector3d(0., 1., 0.)
        ); // y axis

        // Get the transformations on this instance of the component and cache in sScale and sTransform
        SmVector3d sLocalScale;
        SmAxis2Placement sLocalTransform;
        bool resetXformStack;
        UsdGeomXformable xformable(instance);
        std::vector<UsdGeomXformOp> xforms = xformable.GetOrderedXformOps(&resetXformStack);
        for (UsdGeomXformOp xformOp : xforms)
        {
            SMU_BrepConvert::UsdXformToSmTransform(xformOp, sLocalScale, sLocalTransform);
            sTransform.TransformAxis2Placement(sLocalTransform, sTransform);
            sScale = sScale.Multiply(sLocalScale);
        }

        // Convert each UsdRelationship into an AssemblyInstance pointing to a Brep or Assembly as necessary
        // Even when there is only one UsdRelationship on instance, it goes through this loop twice. Why?
        // Expect only 1 component per instance
        ULONG lComponentCount = 0;
        for (UsdRelationship instanceRel : instance.GetRelationships())
        {
            // Get the component paths
            SdfPathVector componentPaths;
            instanceRel.GetTargets(&componentPaths);

            // Expect one component path the first time through this loop, and no component paths the second time
            for (SdfPath componentPath : componentPaths)
            {
                // Count components. Expect only one component per instance
                lComponentCount++;
                SM_ASSERT(lComponentCount == 1);

                // Get the UsdPrim of the compontent
                UsdPrim sComponentPrim = sStage.GetPrimAtPath(componentPath);
                const UsdPrimTypeInfo& crPrimTypeInfo = sComponentPrim.GetPrimTypeInfo();

#ifdef SM_DEBUG_CODE
                if (bDebugMe)
                {
                    // Dump prim path
                    std::string sstring = TfStringPrintf("\nprim = %s\n", sComponentPrim.GetPrimPath().GetAsString().c_str());
                    usdBrep_WriteString(sstring);
                }
#endif // SM_DEBUG_CODE

                // Create the instance of the brep or Assembly
                if (crPrimTypeInfo.GetTypeName() == TfToken("BrepArray"))
                // if (sComponentPrim.IsA<OmniSolidBrepArray>())
                {
                    // Get the Brep
                    SmBrep* pBrep = sBrepMap.GetValueAt(sComponentPrim.GetPath().GetHash());

                    // If we haven't created the SmBrep yet
                    if (pBrep == NULL)
                    {
                        SM_DBG_WARN(_T("Creating a Brep that should exist already"));

                        std::vector<SmBrep*> sBreps;
                        SER(SMU_BrepConvert::BrepMove_UsdToSMLib(
                            crContext, (UsdGeomGprim)sComponentPrim, sBreps));

                        if (sBreps.size() != 1 || sBreps[0] == NULL)
                        {
                            DeleteSmBreps(sBreps);
                            SER_MSG(SM_ERR, _T("BuildSmAssembly expected exactly one converted Brep"));
                        }

                        pBrep = sBreps[0];

                        // Add a mapping from the sDbgUsdBrepArray -> SmBrep
                        sBrepMap.AddUnique(sComponentPrim.GetPath().GetHash(), pBrep);

                        // Transfer ownership to the import result. The map is
                        // non-owning and remains valid until the caller deletes
                        // the returned Brep.
                        rBreps.Add(pBrep);
                        sBreps.clear();
                    }
                    SM_ASSERT(pBrep != NULL);

                    // Create an AssemblyInstance with the Brep
                    // SmAssemblyInstance* pAssemblyInstance = new (crContext) SmAssemblyInstance(sWideName.c_str(), *pAssembly, pBrep, sTransform,
                    // sScale);
                } // end making an instance of the Brep
                else if (UsdModelAPI(sComponentPrim).GetKind(&tKind) && KindRegistry::IsA(tKind, TfToken("assembly")))
                {
                    // Get assembly from map
                    SmAssembly* pSubAssembly = sAssemblyMap.GetValueAt(sComponentPrim.GetPath().GetHash());
                    SM_ASSERT(pSubAssembly != NULL);

                    // Create an AssemblyInstance with the subAssembly
                    // SmAssemblyInstance* pAssemblyInstance = new (crContext) SmAssemblyInstance(sWideName.c_str(), *pAssembly, pSubAssembly,
                    // sTransform, sScale);
                } // end making an instance of the assembly
#ifdef SM_DEBUG_CODE
                else
                {
                    // Dump prim description
                    std::string sstring = TfStringPrintf("\nignoring prim with description: %s\n", sComponentPrim.GetDescription().c_str());
                    usdBrep_WriteString(sstring);
                }
#endif // SM_DEBUG_CODE
            }
        }
    } // end iteration over all children in input prim

    return SM_SUCCESS;

} // end BuildSmAssembly

/***********************************************************************
PURPOSE --- import Assembly from USD file

NOTES ---
***********************************************************************/
SmStatus ImportFromUsd(
 const std::string& usd_Filename, 
 const SmContext& crContext, 
 SmTArray<SmAssembly*>& rAssemblies, 
 SmTArray<SmBrep*>& rBreps
)
{
    UsdImportMapScope sImportMapScope;
    UsdStageRefPtr sStage = UsdStage::Open(usd_Filename);
    UsdPrimRange sPrimRange = sStage->Traverse();

    // Locals
    SmTArray<UsdPrim> sAssemblyPrims;
    TfToken tKind;

    rAssemblies.ReSet();
    rBreps.ReSet();

    // Create all the Brep and Assembly assets
    for (auto iter = sPrimRange.begin(); iter != sPrimRange.end(); ++iter)
    {
        UsdPrim sPrim = *iter;
        size_t sPrimPathHash = sPrim.GetPath().GetHash();
        const UsdPrimTypeInfo& crPrimTypeInfo = sPrim.GetPrimTypeInfo();

#ifdef SM_DEBUG_CODE
        SmBoolean bDebugMe = FALSE;
        if (bDebugMe)
        {
            // Dump prim path
            std::string sstring = TfStringPrintf("\nprim = %s\n", sPrim.GetPrimPath().GetAsString().c_str());
            usdBrep_WriteString(sstring);
        }
#endif // SM_DEBUG_CODE

        // Set variant to Brep
        if (sPrim.HasVariantSets())
        {
            sPrim.GetVariantSet("modelVariant").SetVariantSelection("UsdBrepArrayFromSmBrep");
            continue;
        }

        // This assumes 1 brep per BrepArray. Needs a more general adjustment
        if (crPrimTypeInfo.GetTypeName() == TfToken("BrepArray"))
        // if (sPrim.IsA<OmniSolidBrepArray>())
        {
            // Get the Brep
            SmBrep* pBrep = sBrepMap.GetValueAt(sPrimPathHash);

            // If we haven't created the SmBrep yet
            if (pBrep == NULL)
            {
                std::vector<SmBrep*> sBreps;
                SER(SMU_BrepConvert::BrepMove_UsdToSMLib(
                    crContext, (UsdGeomGprim)sPrim, sBreps));

                if (sBreps.size() != 1 || sBreps[0] == NULL)
                {
                    DeleteSmBreps(sBreps);
                    SER_MSG(SM_ERR, _T("ImportFromUsd expected exactly one converted Brep"));
                }

                pBrep = sBreps[0];

                // Add a mapping from the sDbgUsdBrepArray -> SmBrep
                sBrepMap.AddUnique(sPrimPathHash, pBrep);

                // Record the output
                rBreps.Add(pBrep);

                // Ownership is transferred to rBreps; sBrepMap is non-owning.
                sBreps.clear();
            }

            // Don't traverse the surfaces and curves
            iter.PruneChildren();
        }
        else if (sPrim.IsA<UsdGeomXform>() && UsdModelAPI(sPrim).GetKind(&tKind) && KindRegistry::IsA(tKind, TfToken("assembly")))
        {
            // Name the assembly
            const std::basic_string<TCHAR> sAssemName = smos_ToTChar(sPrim.GetPrimPath().GetName().c_str());
            SmAssembly* pAssembly = new (crContext) SmAssembly(sAssemName.c_str());

            sAssemblyPrims.Add(sPrim);
            rAssemblies.Add(pAssembly);
            sAssemblyMap.AddUnique(sPrimPathHash, pAssembly);

            iter.PruneChildren();
        }
#ifdef SM_DEBUG_CODE
        else
        {
            // Dump prim description
            std::string sstring = TfStringPrintf("\nignoring prim with description : %s\n", sPrim.GetDescription().c_str());
            usdBrep_WriteString(sstring);
        }
#endif // SM_DEBUG_CODE
    }

    // Populate the assemblies
    for (UsdPrim sPrim : sAssemblyPrims)
    {
        SER(BuildSmAssembly(&sPrim, crContext, *sStage, rBreps));
    }

    // bool reset
    return SM_SUCCESS;

} // end ImportFromUsd

/***********************************************************************
PURPOSE --- import SmBreps from USD file

NOTES ---
***********************************************************************/
SmStatus ImportFromUsd(
 const std::string usd_Filename, 
 const SmContext& crContext, 
 SmTArray<SmBrep*>& rBreps
 )
{

    auto sStage = UsdStage::Open(usd_Filename);
    auto sRootPrimIn = sStage->GetPseudoRoot();
    const UsdPrimTypeInfo& crPrimTypeInfo = sRootPrimIn.GetPrimTypeInfo();

    bool bREeset;
    std::vector<UsdGeomXformOp> XformOps;

    // if (sRootPrimIn.IsA<OmniSolidBrepArray>())
    if (crPrimTypeInfo.GetTypeName() == TfToken("BrepArray"))
    {
        // Get the applied Breps
        std::vector<SmBrep*> sBreps;
        SER(SMU_BrepConvert::BrepMove_UsdToSMLib(
            crContext, (UsdGeomGprim)sRootPrimIn, sBreps));
        for (SmBrep* pNewBrep : sBreps)
        {
            if (pNewBrep == NULL)
                continue;
            rBreps.Add(pNewBrep);
        }
        // Note: sBreps objects are transferred to rBreps, so no cleanup needed here
    }

    do
    {
        if (sRootPrimIn.IsA<UsdGeomXformable>())
        {
            UsdGeomXformable xformable = UsdGeomXformable(sRootPrimIn);
            XformOps = xformable.GetOrderedXformOps(&bREeset);
        }
        SER(GetBrepsFromUsd(sRootPrimIn, crContext, rBreps, NULL));

        // Set up for next iter
        sRootPrimIn = sRootPrimIn.GetNextSibling();
        XformOps.clear();
    } while (sRootPrimIn.IsValid());

    return SM_SUCCESS;
} // end ImportFromUsd

/***********************************************************************
PURPOSE --- export SmAssembly to USD file

NOTES ---
***********************************************************************/
SmStatus ExportToUsd(const std::string& sOutputFilename, SmAssembly& rAssembly, SmBoolean* pbExportMeshVariant)
{
    // TODO Export PolyBreps in Assemblies
    // Write an assembly of Breps in USD from an SmAssembly

    // Gather assembly components
    SmTArray<SmAssemblyInstance*> sInstances;
    SmTArray<SmAssembly*> sAssemblies;
    SmTArray<SmBrep*> sBreps;
    SmTArray<SmPolyBrep*> sPolyBreps;
    SmTArray<SmPolyBrep*> sSmPolyBreps;
    rAssembly.GetComponents(sBreps, sSmPolyBreps, sAssemblies);
    sAssemblies.Add(&rAssembly);
    ULONG lNumBreps = sBreps.GetSize();
    ULONG lNumAssemblies = sAssemblies.GetSize();
    sPolyBreps.SetSize(lNumBreps);

    // Create a new stage from the input filename
    UsdStageRefPtr sStage = UsdStage::CreateNew(sOutputFilename);

    // Create a token for the xformOp, since it isn't available globally
    TfToken xformOpTransform("xformOp:transform");

    // Tessellate all Breps in parallel
    if (pbExportMeshVariant && *pbExportMeshVariant == TRUE)
    {
#ifdef SM_USE_TBB
        static tbb::task_arena sArena;
        sArena.execute(
            [&]()
            {
                tbb::parallel_for(
                    (ULONG)0,
                    lNumBreps,
                    [&](ULONG ii)
                    {
                        SmBrep* pBrep = sBreps[ii];
                        SmPolyBrep* pPolyBrep = NULL;
                        ULONG lNumLaminaEdges = 0;
                        SmBoolean sFailedFaces;

                        // tessellate brep
                        double dCHTol = 0.010; // Max dist3D between geom and tess segment, 0=ignore
                        double dCrvTessAngle = 8.; // 0.0; // Max angDeg between tess tangents
                        double dSrfTessAngle = 8.0; // 0.0 // Max allowed Element ControlPolygon turning
                                                    // ang3d (radians)
                        double dMax3DEdge = 0.0; // 1.0; // Max dist3D between tess pts
                        double dMaxAspect = 0.0; // Max allowed Element BasePolygon aspect ratio3d
                        SmBoolean bAdvancingFront = false;
                        SmBoolean bSmoothResults = false;
                        SmBoolean bPropagateAttribs = false;

                        SmStatus status = pBrep->ConvertToPolyBrep(
                            pPolyBrep,
                            sFailedFaces,
                            lNumLaminaEdges,
                            dCHTol,
                            dCrvTessAngle,
                            dSrfTessAngle,
                            dMax3DEdge,
                            dMaxAspect,
                            bAdvancingFront,
                            bSmoothResults,
                            bPropagateAttribs
                        );

                        sPolyBreps[ii] = pPolyBrep;
                    }
                );
            }
        );
#else // no SM_USE_TBB

        for (ULONG ii = 0; ii < lNumBreps; ++ii)
        {
            SmBrep* pBrep = sBreps[ii];
            SmPolyBrep* pPolyBrep = NULL;
            ULONG lNumLaminaEdges = 0;
            SmBoolean sFailedFaces;

            // tessellate brep
            double dCHTol = 0.010; // Max dist3D between geom and tess segment, 0=ignore
            double dCrvTessAngle = 8.; // 0.0; // Max angDeg between tess tangents
            double dSrfTessAngle = 8.0; // 0.0 // Max allowed Element ControlPolygon turning
                                        // ang3d (radians)
            double dMax3DEdge = 0.0; // 1.0; // Max dist3D between tess pts
            double dMaxAspect = 0.0; // Max allowed Element BasePolygon aspect ratio3d
            SmBoolean bAdvancingFront = false;
            SmBoolean bSmoothResults = false;
            SmBoolean bPropagateAttribs = false;

            pBrep->ConvertToPolyBrep(
                pPolyBrep,
                sFailedFaces,
                lNumLaminaEdges,
                dCHTol,
                dCrvTessAngle,
                dSrfTessAngle,
                dMax3DEdge,
                dMaxAspect,
                bAdvancingFront,
                bSmoothResults,
                bPropagateAttribs
            );

            sPolyBreps[ii] = pPolyBrep;
        }
#endif // no SM_USE_TBB
    }

    // Get root layer
    SdfLayerHandle sRootLayerhandle = sStage->GetRootLayer();

    // Create UsdBrepArrays on the stage
    for (ULONG ii = 0; ii < lNumBreps; ++ii)
    {
        // Create the root of this Brep object
        std::string sBrepArrayName("Brep" + std::to_string(ii));
        SdfPath sBrepArrayPath(sBrepArrayName);

        // Create the model primspec on the root layer
        // GWC:REPLACED  SdfPrimSpecHandle sBrepArrayParentPrimSpecHandle = SdfPrimSpec::New(sRootLayerhandle, sBrepArrayName, SdfSpecifierDef,
        // "Xform");
        pxr::UsdPrim sBrepArrayParentPrim = sStage->DefinePrim(sBrepArrayPath, TfToken("Xform"));
        pxr::SdfPrimSpecHandle sBrepArrayParentPrimSpecHandle = sRootLayerhandle->GetPrimAtPath(sBrepArrayParentPrim.GetPath());

        // Set the brep Kind=component
        sBrepArrayParentPrimSpecHandle->SetKind(KindTokens->component);

        // Set empty xform
        VtArray<TfToken> xformOpList;
        pxr::SdfAttributeSpecHandle xformOpOrderAttr = UsdBrepData::setAttributeSpec(
            sBrepArrayParentPrimSpecHandle,
            UsdGeomTokens->xformOpOrder,
            SdfValueTypeNames->TokenArray,
            VtValue(xformOpList)
        );
        if (!xformOpOrderAttr)
        {
            std::string sstring = "Error: Failed to set xformOpOrder attribute on BrepArray parent\n";
            usdBrep_WriteString(sstring);
        }

        // Set the prototype brep invisible
        pxr::SdfAttributeSpecHandle visibilityAttr = UsdBrepData::setAttributeSpec(
            sBrepArrayParentPrimSpecHandle,
            UsdGeomTokens->visibility,
            SdfValueTypeNames->Token,
            VtValue(UsdGeomTokens->invisible)
        );
        if (!visibilityAttr)
        {
            std::string sstring = "Error: Failed to set visibility attribute on BrepArray parent\n";
            usdBrep_WriteString(sstring);
        }

        // Add a variant set to the brep prim
        SdfVariantSpecHandle sVariantSpecHandle = SdfCreateVariantInLayer(sRootLayerhandle, sBrepArrayPath, "modelVariant", "UsdBrepArrayFromSmBrep");
        sBrepArrayParentPrimSpecHandle->SetVariantSelection("modelVariant", "UsdBrepArrayFromSmBrep");
        {
            std::string sBrepTopoName = "BrepTopo";
            std::vector<SmBrep*> sThisBrep = { sBreps[ii] };
            pxr::UsdPrim sUsdBrepArray = sStage->GetPrimAtPath(sBrepArrayPath);

            SE(SMU_BrepConvert::BrepAppend_SMLibToUsd(sThisBrep, sUsdBrepArray));
        }

        // Create the UsdMesh
        if (pbExportMeshVariant && *pbExportMeshVariant == TRUE)
        {
            // Add the mesh as a variant.  Set the mesh as the default selection so, e.g., composer renders the brep on load
            sVariantSpecHandle = SdfCreateVariantInLayer(sRootLayerhandle, sBrepArrayPath, "modelVariant", "meshFromBrep");
            sBrepArrayParentPrimSpecHandle->SetVariantSelection("modelVariant", "meshFromBrep");

            std::string sMeshName = "mesh";
            SdfPrimSpecHandle sMeshPrimSpecHandle = SdfPrimSpec::New(sVariantSpecHandle->GetPrimSpec(), sMeshName, SdfSpecifierDef, "Mesh");
            SE(SMU_BrepConvert::PopulateMeshAttr(*sPolyBreps[ii], sMeshPrimSpecHandle));
        }
    }

    // Create assemblies on the stage
    for (ULONG ii = 0; ii < lNumAssemblies; ++ii)
    {
        // SMLib Locals
        SmAssembly* pAssembly = sAssemblies[ii];
        pAssembly->GetAssemblyInstances(sInstances);

        // Define a root prim
        std::string sAssemblyName;
        {
            sAssemblyName = "BrepAssembly" + std::to_string(ii);
        }

        // Create the assembly primspec on the root layer
        SdfPrimSpecHandle sAssemblyPrimSpecHandle = SdfPrimSpec::New(sRootLayerhandle, sAssemblyName, SdfSpecifierDef, "Xform");

        // Set the assembly Kind=assembly
        sAssemblyPrimSpecHandle->SetKind(KindTokens->assembly);

        // Set an empty xform
        VtArray<TfToken> xformOpList;
        pxr::SdfAttributeSpecHandle assemblyXformAttr = UsdBrepData::setAttributeSpec(
            sAssemblyPrimSpecHandle,
            UsdGeomTokens->xformOpOrder,
            SdfValueTypeNames->TokenArray,
            VtValue(xformOpList)
        );
        if (!assemblyXformAttr)
        {
            std::string sstring = "Error: Failed to set xformOpOrder attribute on Assembly " + std::to_string(ii) + "\n";
            usdBrep_WriteString(sstring);
        }

        // Set the visibility and default prim
        if (ii == lNumAssemblies - 1)
        {
            pxr::SdfAttributeSpecHandle visibilityAttr = UsdBrepData::setAttributeSpec(
                sAssemblyPrimSpecHandle,
                UsdGeomTokens->visibility,
                SdfValueTypeNames->Token,
                VtValue(UsdGeomTokens->inherited)
            );
            if (!visibilityAttr)
            {
                std::string sstring = "Error: Failed to set visibility attribute on Assembly " + std::to_string(ii) + "\n";
                usdBrep_WriteString(sstring);
            }
            sRootLayerhandle->SetDefaultPrim(sAssemblyPrimSpecHandle->GetNameToken());
        }
        else
        {
            pxr::SdfAttributeSpecHandle visibilityAttr = UsdBrepData::setAttributeSpec(
                sAssemblyPrimSpecHandle,
                UsdGeomTokens->visibility,
                SdfValueTypeNames->Token,
                VtValue(UsdGeomTokens->invisible)
            );
            if (!visibilityAttr)
            {
                std::string sstring = "Error: Failed to set visibility attribute on Assembly " + std::to_string(ii) + "\n";
                usdBrep_WriteString(sstring);
            }
        }
    }

    // Populate the Assembly data
    for (ULONG ii = 0; ii < lNumAssemblies; ++ii)
    {
        // SMLib Locals
        SmAssembly* pAssembly = sAssemblies[ii];
        pAssembly->GetAssemblyInstances(sInstances);

        // Regenerate the path for each assembly
        SdfPath AssemblyPath;
        {
            AssemblyPath = SdfPath("/BrepAssembly" + std::to_string(ii));
        }

        for (ULONG jj = 0; jj < sInstances.GetSize(); ++jj)
        {
            // Loop locals
            SmAssemblyInstance* pInstance = sInstances[jj];
            SmBrep* pBrep = pInstance->GetBrep();
            SmAssembly* pInstanceAssembly = pInstance->GetAssembly();

            std::stringstream ostream;

            if (pBrep)
            {
                ostream << "BrepInstance_" << std::to_string(ii) << "_" << std::to_string(jj) << " \n";
            }
            else // if ( pInstanceAssembly)
            {
                ostream << "AssemblyInstance_" << std::to_string(ii) << "_" << std::to_string(jj) << " \n";
            }

            // convert sRawName to something USD can digest
            std::string sName = ostream.str();

            // Create a new primspec (component) in the assembly primspec
            SdfPrimSpecHandle sComponentPrimSpecHandle = SdfPrimSpec::New(sRootLayerhandle->GetPrimAtPath(AssemblyPath), sName, SdfSpecifierDef);

            // Set up reference and relationship
            SdfRelationshipSpecHandle sRelationshipSpecHandle = SdfRelationshipSpec::New(sComponentPrimSpecHandle, "component");
            SdfReference sComponentRef("", SdfPath());
            SdfPath sComponentRelPath;
            sRelationshipSpecHandle->GetTargetPathList().ClearEditsAndMakeExplicit();

            // Create references to each component
            if (pBrep)
            {
                // Get the brep index
                ULONG lIndex = 0;
                SmBoolean bFoundBrep = sBreps.FindElement(pBrep, lIndex);
                SM_ASSERT(bFoundBrep);

                // set the reference and relationship paths
                sComponentRef.SetPrimPath(SdfPath("/Brep" + std::to_string(lIndex)));
                sComponentRelPath = SdfPath("/Brep" + std::to_string(lIndex) + "/BrepTopo");
            }
            else if (pInstanceAssembly)
            {
                // Get the assembly index
                ULONG lIndex = 0;
                SmBoolean bFoundAssembly = sAssemblies.FindElement(pInstanceAssembly, lIndex);
                SM_ASSERT(bFoundAssembly);

                SdfPath sAssemblyPath;
                {
                    sAssemblyPath = SdfPath("/BrepAssembly" + std::to_string(lIndex));
                }

                // set the reference and relationship paths
                sComponentRef.SetPrimPath(sAssemblyPath);
                sComponentRelPath = sAssemblyPath;
            }

            // Attach the reference and relationship data to the sComponentPrimSpecHandle
            sComponentPrimSpecHandle->GetReferenceList().Add(sComponentRef);
            sRelationshipSpecHandle->GetTargetPathList().Add(sComponentRelPath);

            // Set the Instance xform
            {
                // Get component transform
                SmVector3d sScale = pInstance->GetScale();
                SmAxis2Placement sTransform = pInstance->GetPlacement();

                // set Xform order
                VtArray<TfToken> xformOpList = { xformOpTransform };
                pxr::SdfAttributeSpecHandle componentXformAttr = UsdBrepData::setAttributeSpec(
                    sComponentPrimSpecHandle,
                    UsdGeomTokens->xformOpOrder,
                    SdfValueTypeNames->TokenArray,
                    VtValue(xformOpList)
                );
                if (!componentXformAttr)
                {
                    std::string sstring = "Error: Failed to set xformOpOrder attribute on Component " + std::to_string(ii) + "\n";
                    usdBrep_WriteString(sstring);
                }

                // set Xform transform
                GfMatrix4d transform;
                SMU_BrepConvert::SmTransformToUsdXform(sScale, sTransform, transform);
                pxr::SdfAttributeSpecHandle transformAttr = UsdBrepData::setAttributeSpec(
                    sComponentPrimSpecHandle,
                    xformOpTransform,
                    SdfValueTypeNames->Matrix4d,
                    VtValue(transform)
                );
                if (!transformAttr)
                {
                    std::string sstring = "Error: Failed to set transform attribute on Component " + std::to_string(ii) + "\n";
                    usdBrep_WriteString(sstring);
                }

                // set component visibility
                pxr::SdfAttributeSpecHandle componentVisibilityAttr = UsdBrepData::setAttributeSpec(
                    sComponentPrimSpecHandle,
                    UsdGeomTokens->visibility,
                    SdfValueTypeNames->Token,
                    VtValue(UsdGeomTokens->inherited)
                );
                if (!componentVisibilityAttr)
                {
                    std::string sstring = "Error: Failed to set visibility attribute on Component " + std::to_string(ii) + "\n";
                    usdBrep_WriteString(sstring);
                }
            }
        }
    }

    sStage->Save();

    return SM_SUCCESS;
} // end ExportToUsd

/***********************************************************************
PURPOSE --- tessellate an SmBrep for SMU_BrepConvert::PopulateMeshAttr

NOTES ---  propagates face attributes and copies the Brep's attributes
           (e.g. material binding) to the PolyBrep. Caller owns rpPolyBrep.
           Faces that fail to tessellate are warned about and left out.
***********************************************************************/
SmStatus TessellateBrepForMesh(SmBrep& rBrep, SmPolyBrep*& rpPolyBrep)
{
    rpPolyBrep = NULL;
    ULONG lNumLaminaEdges = 0;
    SmBoolean bFailedFaces = FALSE;
    SER(rBrep.ConvertToPolyBrep(rpPolyBrep, bFailedFaces, lNumLaminaEdges,
        0.0,    // chord height tolerance (0 = ignore)
        10.0,   // curve tessellation angle
        30.0,   // surface tessellation angle
        0.0,    // max edge length (0 = ignore)
        0.0,    // max aspect ratio (0 = ignore)
        FALSE,  // advancing front
        FALSE,  // smooth results
        TRUE)); // propagate attributes
    if (!rpPolyBrep)
    { SER(SM_ERR); }
    if (bFailedFaces)
    { std::fprintf(stderr, "Warning: some faces failed to tessellate; using the partial mesh\n"); }

    SmTArray<SmAttribute*> sAttrs;
    rBrep.GetAttributes(sAttrs);
    for (ULONG ii = 0; ii < sAttrs.GetSize(); ++ii)
    { rpPolyBrep->AddAttribute(sAttrs[ii]->MakeCopy(*rpPolyBrep->GetContext())); }

    return SM_SUCCESS;
} // end TessellateBrepForMesh

/***********************************************************************
PURPOSE --- export SmBreps and SmPolyBreps to USD file

NOTES ---  used by the usd_test regression tests
***********************************************************************/
SmStatus ExportToUsd(const std::string& sOutputFilename, const SmTArray<SmBrep*>& crBreps, const SmTArray<SmPolyBrep*>& crPolyBreps)
{
    // Write a USD Brep
    UsdStageRefPtr sStage = UsdStage::CreateNew(sOutputFilename);

    // Define a root prim
    SdfPath sRootPath = SdfPath("/World");
    UsdGeomXform sRootXform = UsdGeomXform::Define(sStage, sRootPath);
    UsdPrim sRootPrim = sStage->GetPrimAtPath(sRootPath);
    SdfLayerHandle sLayerHandle = sStage->GetRootLayer();
    SdfPrimSpecHandle sRootXformHandle = sLayerHandle->GetPrimAtPath(sRootXform.GetPath());

    ULONG lModelNum = 0;
    // Create UsdGeomBrep
    for (ULONG ii = 0; ii < crBreps.GetSize(); ++ii, ++lModelNum)
    {
        std::string sModelName = "model_" + std::to_string(lModelNum);
        SdfPrimSpecHandle sModelPrimSpecHandle = SdfPrimSpec::New(sRootXformHandle, sModelName, SdfSpecifierDef, "Xform");
        UsdGeomXform sModelXform = UsdGeomXform::Define(sStage, sRootPath.AppendPath(SdfPath(sModelName)));
        UsdPrim sModelPrim = sStage->GetPrimAtPath(sModelPrimSpecHandle->GetPath());

        // Add a variant set to the model prim
        std::string sMeshName = "mesh" + std::to_string(ii);
        std::string sBrepArrayName = "Brep" + std::to_string(ii);
        SdfPath sBrepArrayPath(sBrepArrayName), sMeshPath(sMeshName);
        SdfVariantSpecHandle sVariantSpecHandle = SdfCreateVariantInLayer(
            sModelPrimSpecHandle.GetSpec().GetLayer(),
            sModelPrimSpecHandle->GetPath(),
            "modelVariant",
            "meshFromBrep"
        );
        sModelPrimSpecHandle->SetVariantSelection("modelVariant", "meshFromBrep");

        // Create the UsdMesh
        {
            SdfPrimSpecHandle sMeshPrimSpecHandle = SdfPrimSpec::New(sVariantSpecHandle->GetPrimSpec(), sMeshName, SdfSpecifierDef, "Mesh");
            SmPolyBrep* pPolyBrep = NULL;
            SmObjDelete sCleanPoly;
            if (TessellateBrepForMesh(*crBreps[ii], pPolyBrep) == SM_SUCCESS)
            {
                sCleanPoly.SetObj(pPolyBrep);
                SMU_BrepConvert::PopulateMeshAttr(*pPolyBrep, sMeshPrimSpecHandle);
            }
        }

        // Create the sDbgUsdBrepArray
        sVariantSpecHandle = SdfCreateVariantInLayer(
            sModelPrimSpecHandle.GetSpec().GetLayer(),
            sModelPrimSpecHandle->GetPath(),
            "modelVariant",
            "UsdBrepArrayFromSmBrep"
        );
        sModelPrimSpecHandle->SetVariantSelection("modelVariant", "UsdBrepArrayFromSmBrep");
        {
            std::vector<SmBrep*> sBreps = { crBreps[ii] };
            // GWC:REPLACED  SdfPrimSpecHandle    sUsdBrepArraySpecHandle = SdfPrimSpec::New(sVariantSpecHandle->GetPrimSpec(), sBrepArrayName,
            // SdfSpecifierDef, "BrepArray"); GWC:REPLACED  pxr::UsdPrim         sUsdBrepArray           = sStage->GetPrimAtPath(sBrepArrayPath);
            SdfPath sRootBrepArrayPath = sRootPath.AppendPath(SdfPath(sBrepArrayName));
            pxr::UsdPrim sUsdBrepArray = sStage->DefinePrim(sRootBrepArrayPath, TfToken("BrepArray"));

            SE(SMU_BrepConvert::BrepAppend_SMLibToUsd(sBreps, sUsdBrepArray));
        }
    }

    for (ULONG ii = 0; ii < crPolyBreps.GetSize(); ++ii, ++lModelNum)
    {
        std::string sModelName = "model_" + std::to_string(lModelNum);
        SdfPrimSpecHandle sModelPrimSpecHandle = SdfPrimSpec::New(sRootXformHandle, sModelName, SdfSpecifierDef, "Mesh");
        // UsdGeomXform sModelXform = UsdGeomXform::Define(sStage, sRootPath.AppendPath(SdfPath(sModelName)));
        UsdPrim sModelPrim = sStage->GetPrimAtPath(sModelPrimSpecHandle->GetPath());

        SMU_BrepConvert::PopulateMeshAttr(*crPolyBreps[ii], sModelPrimSpecHandle);
    }

    sStage->Save();

    return SM_SUCCESS;
} // end ExportToUsd
