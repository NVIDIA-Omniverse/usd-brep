// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepRead.cpp
 * PURPOSE: UsdBrepArray to UsdBrepArrayData translations
 * ******************************************************************************************************************/

#include "UsdBrepRead.h"

#include "UsdBrepArrayData.h"
#include "UsdBrepDiagnostics.h"
#include "UsdBrepTokens.h"
#include "UsdBrepUtilities.h"

// USD includes
#include "UsdBrepHeaders.h"

namespace UsdBrepData
{

namespace
{

bool HasTokenOccurrence(const VtTokenArray& values, const TfToken& token)
{
    for (const TfToken& value : values)
    {
        if (value == token)
        {
            return true;
        }
    }

    return false;
}

bool RequireSingleAppliedApi(const UsdPrim& prim, const VtTokenArray& typeValues, const TfToken& typeToken, const TfToken& apiToken)
{
    if (!HasTokenOccurrence(typeValues, typeToken) || prim.HasAPI(apiToken))
    {
        return true;
    }

    USDBREP_ERROR(
        "%s",
        ("Malformed BrepArray " + prim.GetPath().GetString() + ": type token '" + typeToken.GetString() + "' requires applied API '" +
         apiToken.GetString() + "'")
            .c_str()
    );
    return false;
}

bool RequireMultiAppliedApi(const UsdPrim& prim, bool hasOccurrence, const TfToken& typeToken, const TfToken& apiToken, const TfToken& instanceToken)
{
    if (!hasOccurrence || prim.HasAPI(apiToken, instanceToken))
    {
        return true;
    }

    USDBREP_ERROR(
        "%s",
        ("Malformed BrepArray " + prim.GetPath().GetString() + ": type token '" + typeToken.GetString() + "' requires applied API '" +
         apiToken.GetString() + ":" + instanceToken.GetString() + "'")
            .c_str()
    );
    return false;
}

bool ValidateRequiredGeometryApis(const UsdPrim& prim, const UsdBrepArrayData& arrays)
{
    bool valid = true;

    valid &= RequireMultiAppliedApi(
        prim,
        HasTokenOccurrence(arrays.m_sVertexPointTypeArray, UsdBrepSolidTokens->brepPointAPI),
        UsdBrepSolidTokens->brepPointAPI,
        UsdBrepSolidTokens->brepPointAPI,
        UsdBrepSolidTokens->vertexPoint
    );
    valid &= RequireMultiAppliedApi(
        prim,
        arrays.TotalShellVertexCount() > 0,
        UsdBrepSolidTokens->brepPointAPI,
        UsdBrepSolidTokens->brepPointAPI,
        UsdBrepSolidTokens->shellPoint
    );

    struct CurveApiRequirement
    {
        TfToken apiToken;
        TfToken edgeInstance;
        TfToken wireEdgeInstance;
    };
    const CurveApiRequirement curveApis[] = {
        { UsdBrepCurveTokens->brepCurve3dNurbAPI, UsdBrepCurveTokens->edge3dNurb, UsdBrepCurveTokens->wireEdge3dNurb },
        { UsdBrepCurveTokens->brepCurve3dCircleAPI, UsdBrepCurveTokens->edge3dCircle, UsdBrepCurveTokens->wireEdge3dCircle },
        { UsdBrepCurveTokens->brepCurve3dLineAPI, UsdBrepCurveTokens->edge3dLine, UsdBrepCurveTokens->wireEdge3dLine },
        { UsdBrepCurveTokens->brepCurve3dEllipseAPI, UsdBrepCurveTokens->edge3dEllipse, UsdBrepCurveTokens->wireEdge3dEllipse },
    };
    for (const CurveApiRequirement& requirement : curveApis)
    {
        valid &= RequireMultiAppliedApi(
            prim,
            HasTokenOccurrence(arrays.m_sEdgeCurveTypeArray, requirement.apiToken),
            requirement.apiToken,
            requirement.apiToken,
            requirement.edgeInstance
        );
        valid &= RequireMultiAppliedApi(
            prim,
            HasTokenOccurrence(arrays.m_sWireEdgeCurveTypeArray, requirement.apiToken),
            requirement.apiToken,
            requirement.apiToken,
            requirement.wireEdgeInstance
        );
    }

    const TfToken surfaceApis[] = {
        UsdBrepSurfaceTokens->brepSurfaceNurbAPI,     UsdBrepSurfaceTokens->brepSurfaceSphereAPI, UsdBrepSurfaceTokens->brepSurfacePlaneAPI,
        UsdBrepSurfaceTokens->brepSurfaceCylinderAPI, UsdBrepSurfaceTokens->brepSurfaceConeAPI,   UsdBrepSurfaceTokens->brepSurfaceTorusAPI,
    };
    for (const TfToken& apiToken : surfaceApis)
    {
        valid &= RequireSingleAppliedApi(prim, arrays.m_sFaceSurfaceTypeArray, apiToken, apiToken);
    }

    return valid;
}

} // namespace

/*********************************************************************************************************************
 PURPOSE: Utility USD function gets attribute value with
            {UsdPrim, Token}
          from prims and their single applied APIs

 NOTES: Returns false when no attribute returned
 ********************************************************************************************************************/
template <typename T>
bool UsdBrepGetAttribute(const UsdPrim& crPrim, const TfToken& crProp, T& rValues, const std::string& crAttributeName)
{
    if (!crPrim || crProp.IsEmpty())
    {
        USDBREP_ERROR("Invalid prim or empty token in UsdBrepGetAttribute");
        return false;
    }

    UsdAttribute sAttr = crPrim.GetAttribute(crProp);
    if (!sAttr || !sAttr.IsAuthored())
    {
        if (!crAttributeName.empty())
        {
            USDBREP_ERROR("%s", ("Attribute " + crProp.GetString() + " not found on prim " + crPrim.GetPath().GetString()).c_str());
        }
        return false;
    }

    if (!sAttr.Get(&rValues))
    {
        if (!crAttributeName.empty())
        {
            USDBREP_WARN("%s", ("Failed to get " + crAttributeName + " attribute from BrepArray " + crPrim.GetPath().GetString()).c_str());
        }
        return false;
    }

    return true;
} // end UsdBrepGetAttribute<type> from prims and their single applied APIs

/*********************************************************************************************************************
 PURPOSE: Utility USD function to load rValues for a crProp/crInstance combined token
          from the input crPrim

 NOTES: Returns false when no attribute returned
        builds a combined token from crProp and crInstance which
        is then used for attribute access.
        Ex: rInstanceToken = UsdBrepSolidTokens->shellPoint        = "shellPoint"
            rPropertyToken = UsdBrepSolidTokens->brepPointPosition = "brep:__INSTANCE_NAME__:point:position"
            combinedToken  = "brep:shellPoint:point:position"
 ********************************************************************************************************************/
template <typename T>
bool UsdBrepGetMultiAppliedAttribute(
    const UsdPrim& crPrim, // in : source UsdPrim
    const TfToken& crProp, // in : attribute name from an multiApplied API
                           //      ex: UsdBrepSolidTokens->brepPointPosition
    const TfToken& crInstance, // in : multiApplied API instance name
                               //      ex: UsdBrepSolidTokens->shellPoint
    T& rValues, // in : output attribute values
    const std::string& crAttributeName // in : value label, for error reporting only
                                       //      ex: "shell point position"
)
{
    if (!crPrim || crProp.IsEmpty() || crInstance.IsEmpty())
    {
        USDBREP_ERROR("Invalid prim or empty tokens in UsdBrepGetMultiAppliedAttribute");
        return false;
    }

    pxr::TfToken nameSpacedPropName = UsdBrepData::GetNameSpacePropertyToken(crInstance, crProp);
    UsdAttribute sAttr = crPrim.GetAttribute(nameSpacedPropName);

    if (!sAttr || !sAttr.IsAuthored())
    {
        USDBREP_ERROR(
            "%s",
            ("Multi-applied attribute " + crInstance.GetString() + ":" + crProp.GetString() + " not found on prim " + crPrim.GetPath().GetString())
                .c_str()
        );
        return false;
    }

    if (!sAttr.Get(&rValues))
    {
        if (!crAttributeName.empty())
        {
            USDBREP_WARN("%s", ("Failed to get " + crAttributeName + " attribute from BrepArray " + crPrim.GetPath().GetString()).c_str());
        }
        return false;
    }

    return true;
} // end UsdBrepGetMultiAppliedAttribute<type> from prim multiApplied APIs

/*********************************************************************************************************************
 PURPOSE: get primSpec attrib values
          for attribs defined by prim schema or singleApplied APIs

 ********************************************************************************************************************/
template <typename T>
void UsdBrepGetAttribute(const pxr::SdfPrimSpecHandle& crPrimSpecHandle, const TfToken& crProp, T& rValues)
{
    // Check if primSpec handle is valid
    if (!crPrimSpecHandle)
    {
        USDBREP_ERROR("%s", ("Invalid SdfPrimSpecHandle provided to UsdBrepGetAttribute for property " + crProp.GetString()).c_str());
        return;
    }

    // Check if property token is valid
    if (crProp.IsEmpty())
    {
        USDBREP_ERROR("Empty TfToken provided for property in UsdBrepGetAttribute");
        return;
    }

    // check if the field exists
    if (!crPrimSpecHandle->HasField(crProp))
    {
        USDBREP_ERROR("%s", ("Field " + crProp.GetString() + " not found in primSpec " + crPrimSpecHandle->GetPath().GetString()).c_str());
        return;
    }

    // get the value
    rValues = crPrimSpecHandle->GetFieldAs<T>(crProp);

} // end UsdBrepGetAttribute<type> from primSpecs and their singleApplied APIs

/*********************************************************************************************************************
 PURPOSE: get prim attrib values
          for attribs defined by multiApplied APIs

 ********************************************************************************************************************/
template <typename T>
void UsdBrepGetMultiAppliedAttribute(const pxr::SdfPrimSpecHandle& crPrimSpecHandle, const TfToken& crProp, const TfToken& crInstance, T& rValues)
{
    // Check if primSpec handle is valid
    if (!crPrimSpecHandle)
    {
        USDBREP_ERROR("%s", ("Invalid SdfPrimSpecHandle provided to UsdBrepGetMultiAppliedAttribute " + crProp.GetString()).c_str());
        return;
    }

    // Check if property and instance tokens are valid
    if (crProp.IsEmpty())
    {
        USDBREP_ERROR("Empty TfToken provided for property in UsdBrepGetMultiAppliedAttribute");
        return;
    }

    if (crInstance.IsEmpty())
    {
        USDBREP_ERROR("Empty TfToken provided for instance in UsdBrepGetMultiAppliedAttribute");
        return;
    }

    pxr::TfToken nameSpacedPropName = UsdBrepData::GetNameSpacePropertyToken(crInstance, crProp);

    // desired namespaceFeildName = "brep:instance:PropName"
    // expected CrProp strcut     = "brep:PropName"

    // check if the field exists
    if (!crPrimSpecHandle->HasField(nameSpacedPropName))
    {
        USDBREP_ERROR("%s", ("Field " + nameSpacedPropName.GetString() + " not found in primSpec " + crPrimSpecHandle->GetPath().GetString()).c_str());
        return;
    }

    // get the value
    rValues = crPrimSpecHandle->GetFieldAs<T>(nameSpacedPropName);

} // end UsdBrepGetMultiAppliedAttribute<type> from primSpec multiApplied APIs

/*********************************************************************************************************************
 PURPOSE: load UsdBrepArrayData from target BrepArrayGprim

 NOTES:
 USD BrepArray data usage for prim = a OmniSolidBrepArray object
                          primSpec = related PrimSpec on root layer
                          primPath = path to base prim
 1. BrepArray meta data
    a) path  = "PrimPath.source" -> customData(token="source", value=<std::string>) origin CAD system label
    b) path  = "PrimPath.extent" -> attribute (token="extent", value= VtArray<GfVec3f>) BBox (union all Brep BBoxes)

 2. BrepArray Topo and Geom data
    c) paths = "PrimPath.AttribTokenName" -> all BrepArray schema attributes
    d) paths = "PrimPath.AttribTokenName" -> all BrepGeometry applied APIs attributes

 3. Materials for perBrepArray, perBrep, and perFace uses
    e) path = "PrimPath.materal:binding" -> relationship(token:["materialBinding"])
       for BrepArray material default for all Brep materials
    f) path = "PrimPath/subset_ii"       -> prim.subGeom, one per BrepMaterial
     f.1) path = "PrimPath/subset_ii.elementType"
          attribute   (token:["elementType"], value:[token=["brep"])  // "brep" = indices is list of Brep indices
          attribute   (token:["elementType"], value:[token=["face"])  // "face" = indices is list of face indices

     // brep or face indices to be rendered with this materialBinding
     f.2) path = "PrimPath/subset_ii.indices"
          attribute   (token:["indices"],     value:[VtArray<uint32_t>])

     // rendering material path for breps or faces in the indices list
     f.3) path = "PrimPath/subset_ii.material:binding"
          relationship(token:["materialBinding"])

 4. no work - no changes made when crUsdBrepArray has no Breps

 PARAMETERS:
    crUsdBrepArray : in : source UsdBrepArray
    rArrays        : out: UsdBrepArrayData
 ********************************************************************************************************************/
bool BrepReadFromUsdStage(
    const pxr::UsdPrim& crUsdBrepArray, // in : source UsdBrepArray
    UsdBrepArrayData& rArrays //           out: UsdBrepArrayData
)
{
    // locals
    bool bRtn = true;
    std::string sstring;

    // Check if prim is valid
    if (!crUsdBrepArray)
    {
        sstring = "Invalid UsdPrim provided to BrepReadFromUsdStage";
        USDBREP_ERROR("%s", sstring.c_str());
        return false;
    }

    // no work - no Breps in crUsdBrepArray
    if (GetUsdBrepArray_BrepCount(crUsdBrepArray) == 0)
    {
        return false;
    }

    // check state - Verify 'omniSolid/resources' is registered
    if (!IsOmniSolidResourcesPluginRegistered())
    {
        USDBREP_ERROR(
            "%s",
            "BrepReadFromUsdStage: 'omniSolid/resources' schema plugin is not registered "
            "(register it at startup before opening any USD stage; set OMNISOLID_PLUGIN_PATH); "
            "cannot resolve BrepArray applied-API schemas"
        );
        return false;
    }

    // check state - registered too late: the schemas are unknown
    if (!AreOmniSolidSchemasKnown())
    {
        USDBREP_ERROR(
            "%s",
            "BrepReadFromUsdStage: 'omniSolid/resources' schema plugin was registered after a USD stage was opened, "
            "so BrepArray prims cannot be read (register it before opening any USD stage, or set PXR_PLUGINPATH_NAME "
            "to the omniSolid/resources directory)"
        );
        return false;
    }

    // Reset UsdBrepArrayData to clear any existing state before populating
    rArrays.ReSet();

    // begin scope: Move UsdBrepArray::perBrepArray data to UsdBrepArrayData
    {
        // UsdBrepArray.GetPrim() path - to be stored in SmBrep->Attributes built from this UsdBrepArrayData
        rArrays.m_sPrimPath = crUsdBrepArray.GetPath();

        // source string - to be added in case this attribute gets added to the BrepArray Schema
        // rArrays.m_sCADSource = crUsdBrepArray.GetCustomDataByKey(UsdBrepSolidTokens->source).Get<std::string>();

        VtArray<GfVec3f> extentArray;
        bool extentResult = UsdBrepGetAttribute(crUsdBrepArray, UsdGeomTokens->extent, extentArray, "");
        if (extentResult && extentArray.size() >= 2)
        {
            rArrays.m_sBrepArray_BBox = GfRange3f(extentArray[0], extentArray[1]);
        }
        else if (extentResult && extentArray.size() < 2)
        {
            sstring = "Extent array has insufficient elements (" + std::to_string(extentArray.size()) + "), expected at least 2";
            USDBREP_WARN("%s", sstring.c_str());
        }
        else
        {
            sstring = "Failed to get extent attribute for BrepArray " + crUsdBrepArray.GetPath().GetString();
            USDBREP_WARN("%s", sstring.c_str());
        }

        // material properties
        {
            // BrepArray material
            pxr::SdfPathVector sBrepArray_MaterialVector;
            UsdRelationship materialBinding = crUsdBrepArray.GetRelationship(UsdShadeTokens->materialBinding);
            if (materialBinding)
            {
                bool getTargetsResult = materialBinding.GetTargets(&sBrepArray_MaterialVector);
                if (!getTargetsResult)
                {
                    sstring = "Failed to get material binding targets for BrepArray " + crUsdBrepArray.GetPath().GetString();
                    USDBREP_WARN("%s", sstring.c_str());
                }
                else if (sBrepArray_MaterialVector.size() > 0)
                {
                    rArrays.m_sBrepArray_MaterialPath = sBrepArray_MaterialVector[0];
                }
            }

            // PerBrep and perFace materials found in relationships on the list of UsdBrepArray->Children
            UsdPrimSiblingRange sChildren = crUsdBrepArray.GetAllChildren();
            for (auto child = sChildren.begin(); child != sChildren.end(); child++)
            {
                // Check if child prim is valid
                if (!*child)
                {
                    sstring = "Invalid child prim found in BrepArray " + crUsdBrepArray.GetPath().GetString();
                    USDBREP_WARN("%s", sstring.c_str());
                    continue;
                }

                // Only add the data arrays if the geom subset has all of the data defined: elementType, indices,
                // and MatPath
                if (child->IsA<UsdGeomSubset>())
                {
                    // locals
                    TfToken tElementType("none");
                    VtIntArray sIndices;
                    SdfPathVector sMatPaths;

                    // Get the ElementType
                    if (UsdAttribute sElementTypeAttr = child->GetAttribute(UsdGeomTokens->elementType))
                    {
                        bool getResult = sElementTypeAttr.Get(&tElementType);
                        if (!getResult)
                        {
                            sstring = "Failed to get elementType attribute from GeomSubset " + child->GetPath().GetString();
                            USDBREP_WARN("%s", sstring.c_str());
                        }
                    }

                    // skip elementTypes != "face" or "brep"
                    if (tElementType != UsdBrepSolidTokens->brep && tElementType != UsdBrepSolidTokens->face)
                    {
                        continue;
                    }

                    // Get the brep or face indices for this GeomSubset
                    if (UsdAttribute sIndicesAttr = child->GetAttribute(UsdGeomTokens->indices))
                    {
                        bool getResult = sIndicesAttr.Get(&sIndices);
                        if (!getResult)
                        {
                            sstring = "Failed to get indices attribute from GeomSubset " + child->GetPath().GetString();
                            USDBREP_WARN("%s", sstring.c_str());
                            continue;
                        }
                    }
                    else
                    {
                        sstring = "No indices attribute found on GeomSubset " + child->GetPath().GetString();
                        USDBREP_WARN("%s", sstring.c_str());
                        continue;
                    }

                    // Get the Material path for this GeomSubset
                    if (UsdRelationship sMatBinding = child->GetRelationship(UsdShadeTokens->materialBinding))
                    {
                        bool getTargetsResult = sMatBinding.GetTargets(&sMatPaths);
                        if (!getTargetsResult)
                        {
                            sstring = "Failed to get material binding targets from GeomSubset " + child->GetPath().GetString();
                            USDBREP_WARN("%s", sstring.c_str());
                            continue;
                        }
                        if (sMatPaths.size() != 1)
                        {
                            sstring = "GeomSubset " + child->GetPath().GetString() + " has " + std::to_string(sMatPaths.size()) +
                                      " material paths, expected 1";
                            USDBREP_WARN("%s", sstring.c_str());
                        }
                    }
                    else
                    {
                        sstring = "No material binding relationship found on GeomSubset " + child->GetPath().GetString();
                        USDBREP_WARN("%s", sstring.c_str());
                        continue;
                    }

                    // Add the geomSubset material data to appropriate Brep or Face material arrays
                    if (tElementType == UsdBrepSolidTokens->brep)
                    {
                        if (!sMatPaths.empty())
                        {
                            rArrays.m_sBrepMaterial_BrepIndexArray.emplace_back(sIndices);
                            rArrays.m_sBrepMaterial_BrepPathArray.emplace_back(sMatPaths.front());
                        }
                        else
                        {
                            sstring = "Empty material paths for brep GeomSubset " + child->GetPath().GetString();
                            USDBREP_WARN("%s", sstring.c_str());
                        }
                    } // end brep material property branch
                    else // face material property branch
                    {
                        if (!sMatPaths.empty())
                        {
                            rArrays.m_sFaceMaterial_FaceIndexArray.emplace_back(sIndices);
                            rArrays.m_sFaceMaterial_FacePathArray.emplace_back(sMatPaths.front());
                        }
                        else
                        {
                            sstring = "Empty material paths for face GeomSubset " + child->GetPath().GetString();
                            USDBREP_WARN("%s", sstring.c_str());
                        }
                    } // end face material property branch

                } // end if (geomSubset)
            } // end iteration over all children
        } // end scope: material properties
    } // end scope: Move UsdBrepArray BrepArray data to UsdBrepArrayData


    // begin scope: Move UsdBrepArray Brep topology object data to UsdBrepArrayData
    {
        // brep metaData arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->brepIntersectTol3d, rArrays.m_sBrepXSectTol3dArray, "brepIntersectTol3d");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->brepExtent, rArrays.m_sBrepExtentArray, "brepExtent");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->brepRegionCount, rArrays.m_sBrepRegionCountArray, "brepRegionCount");
        // Brep topology object count arrays in USD format

        // region arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->regionShellCount, rArrays.m_sRegionShellCountArray, "regionShellCount");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->regionType, rArrays.m_sRegionTypeArray, "regionType");

        // shell arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->shellFaceuseCount, rArrays.m_sShellFaceuseCountArray, "shellFaceuseCount");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->shellWireEdgeCount, rArrays.m_sShellWireEdgeCountArray, "shellWireEdgeCount");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->shellPointType, rArrays.m_sShellPointTypeArray, "shellPointType");

        // faceuse arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->faceuseFaceIndex, rArrays.m_sFaceuseFaceIndexArray, "faceuseFaceIndex");
        bRtn &= UsdBrepGetAttribute(
            crUsdBrepArray,
            UsdBrepSolidTokens->faceuseOrientationType,
            rArrays.m_sFaceuseOrientationTypeArray,
            "faceuseOrientationType"
        );

        // face arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->faceLoopCount, rArrays.m_sFaceLoopCountArray, "faceLoopCount");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->faceSurfaceType, rArrays.m_sFaceSurfaceTypeArray, "faceSurfaceType");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->faceTrimType, rArrays.m_sFaceTrimTypeArray, "faceTrimType");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->faceRange, rArrays.m_sFaceRangeArray, "faceRange");

        // loop arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->loopEdgeuseCount, rArrays.m_sLoopEdgeuseCountArray, "loopEdgeuseCount");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->loopVertexIndex, rArrays.m_sLoopVertexIndexArray, "loopVertexIndex");

        // edgeuse arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->edgeuseEdgeIndex, rArrays.m_sEdgeuseEdgeIndexArray, "edgeuseEdgeIndex");
        bRtn &= UsdBrepGetAttribute(
            crUsdBrepArray,
            UsdBrepSolidTokens->edgeuseOrientationType,
            rArrays.m_sEdgeuseOrientationTypeArray,
            "edgeuseOrientationType"
        );
        bRtn &= UsdBrepGetAttribute(
            crUsdBrepArray,
            UsdBrepSolidTokens->edgeuseNextRadialEUIndex,
            rArrays.m_sEdgeuseNextRadialEUIndexArray,
            "edgeuseNextRadialEUIndex"
        );
        bRtn &= UsdBrepGetAttribute(
            crUsdBrepArray,
            UsdBrepSolidTokens->edgeuseThisRadialEntryType,
            rArrays.m_sEdgeuseThisRadialEntryTypeArray,
            "edgeuseThisRadialEntryType"
        );

        // edge arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->edgeCurveType, rArrays.m_sEdgeCurveTypeArray, "edgeCurveType");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->edgeRange, rArrays.m_sEdgeRangeArray, "edgeRange");
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->edgeVertexIndices, rArrays.m_sEdgeVertexIndicesArray, "edgeVertexIndices");

        // Wire-edge topology (wireEdge:curveType/range/vertexIndices) is optional as a set:
        // when shell:wireEdgeCount is all zero, producers may omit all three attrs and
        // SmuConvert builds the arrays later. When any wire edges exist, all three must be
        // authored together -- read only if every member is present, so parallel array lengths
        // stay matched; a partially-authored family is skipped (left empty).
        {
            UsdAttribute sWireEdgeCurveTypeAttr = crUsdBrepArray.GetAttribute(UsdBrepSolidTokens->wireEdgeCurveType);
            UsdAttribute sWireEdgeRangeAttr = crUsdBrepArray.GetAttribute(UsdBrepSolidTokens->wireEdgeRange);
            UsdAttribute sWireEdgeVertexIndicesAttr = crUsdBrepArray.GetAttribute(UsdBrepSolidTokens->wireEdgeVertexIndices);
            if (sWireEdgeCurveTypeAttr && sWireEdgeCurveTypeAttr.IsAuthored() && sWireEdgeRangeAttr && sWireEdgeRangeAttr.IsAuthored() &&
                sWireEdgeVertexIndicesAttr && sWireEdgeVertexIndicesAttr.IsAuthored())
            {
                bRtn &= UsdBrepGetAttribute(
                    crUsdBrepArray,
                    UsdBrepSolidTokens->wireEdgeCurveType,
                    rArrays.m_sWireEdgeCurveTypeArray,
                    "wireEdgeCurveType"
                );
                bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->wireEdgeRange, rArrays.m_sWireEdgeRangeArray, "wireEdgeRange");
                bRtn &= UsdBrepGetAttribute(
                    crUsdBrepArray,
                    UsdBrepSolidTokens->wireEdgeVertexIndices,
                    rArrays.m_sWireEdgeVertexIndicesArray,
                    "wireEdgeVertexIndices"
                );
            }
        }

        // vertex arrays in USD format
        bRtn &= UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->vertexPointType, rArrays.m_sVertexPointTypeArray, "vertexPointType");
    } // end scope: Move UsdBrepArray Brep topology object data to UsdBrepArrayData

    if (!bRtn || !ValidateRequiredGeometryApis(crUsdBrepArray, rArrays))
    {
        return false;
    }

    // begin scope: Move UsdBrepArray PerBrep shape object data to UsdBrepArrayData
    {
        // edgeVertex position array in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSolidTokens->brepPointAPI, UsdBrepSolidTokens->vertexPoint))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepSolidTokens->brepPointPosition,
                UsdBrepSolidTokens->vertexPoint,
                rArrays.m_sVertex_PointPositionArray,
                "vertex point position"
            );
        }

        // shellVertex position array in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSolidTokens->brepPointAPI, UsdBrepSolidTokens->shellPoint))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepSolidTokens->brepPointPosition,
                UsdBrepSolidTokens->shellPoint,
                rArrays.m_sShell_PointPositionArray,
                "shell point position"
            );
        }

        // edge Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dNurbAPI, UsdBrepCurveTokens->edge3dNurb))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbControlVertices,
                UsdBrepCurveTokens->edge3dNurb,
                rArrays.m_sEdge_CurveNurb_ControlVerticesArray,
                "edge 3D NURB control vertices"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbVertexCount,
                UsdBrepCurveTokens->edge3dNurb,
                rArrays.m_sEdge_CurveNurb_VertexCountArray,
                "edge 3D NURB vertex count"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbOrder,
                UsdBrepCurveTokens->edge3dNurb,
                rArrays.m_sEdge_CurveNurb_OrderArray,
                "edge 3D NURB order"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbKnots,
                UsdBrepCurveTokens->edge3dNurb,
                rArrays.m_sEdge_CurveNurb_KnotsArray,
                "edge 3D NURB knots"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbWeights,
                UsdBrepCurveTokens->edge3dNurb,
                rArrays.m_sEdge_CurveNurb_WeightsArray,
                "edge 3D NURB weights"
            );
        }

        // edge Circle Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dCircleAPI, UsdBrepCurveTokens->edge3dCircle))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleCenter,
                UsdBrepCurveTokens->edge3dCircle,
                rArrays.m_sEdge_CurveCircle_CenterArray,
                "edge 3D circle center"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleAxis,
                UsdBrepCurveTokens->edge3dCircle,
                rArrays.m_sEdge_CurveCircle_AxisArray,
                "edge 3D circle axis"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleRefDirection,
                UsdBrepCurveTokens->edge3dCircle,
                rArrays.m_sEdge_CurveCircle_RefDirectionArray,
                "edge 3D circle refDirection"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleRadius,
                UsdBrepCurveTokens->edge3dCircle,
                rArrays.m_sEdge_CurveCircle_RadiusArray,
                "edge 3D circle radius"
            );
        }

        // edge Line Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dLineAPI, UsdBrepCurveTokens->edge3dLine))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dLineOrigin,
                UsdBrepCurveTokens->edge3dLine,
                rArrays.m_sEdge_CurveLine_OriginArray,
                "edge 3D line origin"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dLineDirection,
                UsdBrepCurveTokens->edge3dLine,
                rArrays.m_sEdge_CurveLine_DirectionArray,
                "edge 3D line direction"
            );
        }

        // edge Ellipse Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dEllipseAPI, UsdBrepCurveTokens->edge3dEllipse))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseCenter,
                UsdBrepCurveTokens->edge3dEllipse,
                rArrays.m_sEdge_CurveEllipse_CenterArray,
                "edge 3D ellipse center"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseAxis,
                UsdBrepCurveTokens->edge3dEllipse,
                rArrays.m_sEdge_CurveEllipse_AxisArray,
                "edge 3D ellipse axis"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseRefDirection,
                UsdBrepCurveTokens->edge3dEllipse,
                rArrays.m_sEdge_CurveEllipse_RefDirectionArray,
                "edge 3D ellipse refDirection"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseXRadius,
                UsdBrepCurveTokens->edge3dEllipse,
                rArrays.m_sEdge_CurveEllipse_XRadiusArray,
                "edge 3D ellipse XRadius"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseYRadius,
                UsdBrepCurveTokens->edge3dEllipse,
                rArrays.m_sEdge_CurveEllipse_YRadiusArray,
                "edge 3D ellipse YRadius"
            );
        }

        // wireEdge Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dNurbAPI, UsdBrepCurveTokens->wireEdge3dNurb))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbControlVertices,
                UsdBrepCurveTokens->wireEdge3dNurb,
                rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray,
                "wireEdge 3D NURB control vertices"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbVertexCount,
                UsdBrepCurveTokens->wireEdge3dNurb,
                rArrays.m_sWireEdge_CurveNurb_VertexCountArray,
                "wireEdge 3D NURB vertex count"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbOrder,
                UsdBrepCurveTokens->wireEdge3dNurb,
                rArrays.m_sWireEdge_CurveNurb_OrderArray,
                "wireEdge 3D NURB order"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbKnots,
                UsdBrepCurveTokens->wireEdge3dNurb,
                rArrays.m_sWireEdge_CurveNurb_KnotsArray,
                "wireEdge 3D NURB knots"
            );

            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dNurbWeights,
                UsdBrepCurveTokens->wireEdge3dNurb,
                rArrays.m_sWireEdge_CurveNurb_WeightsArray,
                "wireEdge 3D NURB weights"
            );
        }

        // wireEdge Circle Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dCircleAPI, UsdBrepCurveTokens->wireEdge3dCircle))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleCenter,
                UsdBrepCurveTokens->wireEdge3dCircle,
                rArrays.m_sWireEdge_CurveCircle_CenterArray,
                "wireEdge 3D circle center"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleAxis,
                UsdBrepCurveTokens->wireEdge3dCircle,
                rArrays.m_sWireEdge_CurveCircle_AxisArray,
                "wireEdge 3D circle axis"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleRefDirection,
                UsdBrepCurveTokens->wireEdge3dCircle,
                rArrays.m_sWireEdge_CurveCircle_RefDirectionArray,
                "wireEdge 3D circle refDirection"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dCircleRadius,
                UsdBrepCurveTokens->wireEdge3dCircle,
                rArrays.m_sWireEdge_CurveCircle_RadiusArray,
                "wireEdge 3D circle radius"
            );
        }

        // wireEdge Line Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dLineAPI, UsdBrepCurveTokens->wireEdge3dLine))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dLineOrigin,
                UsdBrepCurveTokens->wireEdge3dLine,
                rArrays.m_sWireEdge_CurveLine_OriginArray,
                "wireEdge 3D line origin"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dLineDirection,
                UsdBrepCurveTokens->wireEdge3dLine,
                rArrays.m_sWireEdge_CurveLine_DirectionArray,
                "wireEdge 3D line direction"
            );
        }

        // wireEdge Ellipse Curve3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurve3dEllipseAPI, UsdBrepCurveTokens->wireEdge3dEllipse))
        {
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseCenter,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                rArrays.m_sWireEdge_CurveEllipse_CenterArray,
                "wireEdge 3D ellipse center"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseAxis,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                rArrays.m_sWireEdge_CurveEllipse_AxisArray,
                "wireEdge 3D ellipse axis"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseRefDirection,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                rArrays.m_sWireEdge_CurveEllipse_RefDirectionArray,
                "wireEdge 3D ellipse refDirection"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseXRadius,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                rArrays.m_sWireEdge_CurveEllipse_XRadiusArray,
                "wireEdge 3D ellipse XRadius"
            );
            bRtn &= UsdBrepGetMultiAppliedAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brep3dEllipseYRadius,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                rArrays.m_sWireEdge_CurveEllipse_YRadiusArray,
                "wireEdge 3D ellipse YRadius"
            );
        }

        // edgeuse UVTrimCurve2d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepCurveTokens->brepCurveUvNurbAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brepCurveUvNurbControlVertices,
                rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray,
                "edgeuse curve NURB control vertices"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brepCurveUvNurbOrder,
                rArrays.m_sEdgeuse_CurveNurb_OrderArray,
                "edgeuse curve NURB order"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brepCurveUvNurbVertexCount,
                rArrays.m_sEdgeuse_CurveNurb_VertexCountArray,
                "edgeuse curve NURB vertex count"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brepCurveUvNurbKnots,
                rArrays.m_sEdgeuse_CurveNurb_KnotsArray,
                "edgeuse curve NURB knots"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepCurveTokens->brepCurveUvNurbWeights,
                rArrays.m_sEdgeuse_CurveNurb_WeightsArray,
                "edgeuse curve NURB weights"
            );
        }
        else // make an empty entry UVTrimCurve for every Edgeuse
        {
            rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.assign(rArrays.TotalEdgeuseCount(), 0);
            rArrays.m_sEdgeuse_CurveNurb_OrderArray.assign(rArrays.TotalEdgeuseCount(), 0);
        }

        // face SphereSurface arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSurfaceTokens->brepSurfaceSphereAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceSphereCenter,
                rArrays.m_sFace_SurfaceSphere_CenterArray,
                "face surface sphere center"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceSphereAxis,
                rArrays.m_sFace_SurfaceSphere_AxisArray,
                "face surface sphere axis"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceSphereRefDirection,
                rArrays.m_sFace_SurfaceSphere_RefDirectionArray,
                "face surface sphere refDirection"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceSphereRadius,
                rArrays.m_sFace_SurfaceSphere_RadiusArray,
                "face surface sphere radius"
            );
        }

        // face PlaneSurface arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSurfaceTokens->brepSurfacePlaneAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfacePlaneOrigin,
                rArrays.m_sFace_SurfacePlane_OriginArray,
                "face surface plane origin"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfacePlaneAxis,
                rArrays.m_sFace_SurfacePlane_AxisArray,
                "face surface plane axis"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfacePlaneRefDirection,
                rArrays.m_sFace_SurfacePlane_RefDirectionArray,
                "face surface plane refDirection"
            );
        }

        // face CylinderSurface arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSurfaceTokens->brepSurfaceCylinderAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceCylinderOrigin,
                rArrays.m_sFace_SurfaceCylinder_OriginArray,
                "face surface cylinder origin"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceCylinderAxis,
                rArrays.m_sFace_SurfaceCylinder_AxisArray,
                "face surface cylinder axis"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceCylinderRefDirection,
                rArrays.m_sFace_SurfaceCylinder_RefDirectionArray,
                "face surface cylinder refDirection"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceCylinderRadius,
                rArrays.m_sFace_SurfaceCylinder_RadiusArray,
                "face surface cylinder radius"
            );
        }

        // face ConeSurface arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSurfaceTokens->brepSurfaceConeAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceConeOrigin,
                rArrays.m_sFace_SurfaceCone_OriginArray,
                "face surface cone origin"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceConeAxis,
                rArrays.m_sFace_SurfaceCone_AxisArray,
                "face surface cone axis"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceConeRefDirection,
                rArrays.m_sFace_SurfaceCone_RefDirectionArray,
                "face surface cone refDirection"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceConeRadius,
                rArrays.m_sFace_SurfaceCone_RadiusArray,
                "face surface cone radius"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceConeSemiAngle,
                rArrays.m_sFace_SurfaceCone_SemiAngleArray,
                "face surface cone semiAngle"
            );
        }

        // face TorusSurface arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSurfaceTokens->brepSurfaceTorusAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceTorusOrigin,
                rArrays.m_sFace_SurfaceTorus_OriginArray,
                "face surface torus origin"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceTorusAxis,
                rArrays.m_sFace_SurfaceTorus_AxisArray,
                "face surface torus axis"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceTorusRefDirection,
                rArrays.m_sFace_SurfaceTorus_RefDirectionArray,
                "face surface torus refDirection"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceTorusMajorRadius,
                rArrays.m_sFace_SurfaceTorus_MajorRadiusArray,
                "face surface torus majorRadius"
            );
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceTorusMinorRadius,
                rArrays.m_sFace_SurfaceTorus_MinorRadiusArray,
                "face surface torus minorRadius"
            );
        }

        // face Surface3d arrays in USD format
        if (crUsdBrepArray.HasAPI(UsdBrepSurfaceTokens->brepSurfaceNurbAPI))
        {
            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbControlVertices,
                rArrays.m_sFace_SurfaceNurb_ControlVerticesArray,
                "face surface NURB control vertices"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbUOrder,
                rArrays.m_sFace_SurfaceNurb_UOrderArray,
                "face surface NURB U order"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbVOrder,
                rArrays.m_sFace_SurfaceNurb_VOrderArray,
                "face surface NURB V order"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbUVertexCount,
                rArrays.m_sFace_SurfaceNurb_UVertexCountArray,
                "face surface NURB U vertex count"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbVVertexCount,
                rArrays.m_sFace_SurfaceNurb_VVertexCountArray,
                "face surface NURB V vertex count"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbUKnots,
                rArrays.m_sFace_SurfaceNurb_UKnotsArray,
                "face surface NURB U knots"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbVKnots,
                rArrays.m_sFace_SurfaceNurb_VKnotsArray,
                "face surface NURB V knots"
            );

            bRtn &= UsdBrepGetAttribute(
                crUsdBrepArray,
                UsdBrepSurfaceTokens->brepSurfaceNurbWeights,
                rArrays.m_sFace_SurfaceNurb_WeightsArray,
                "face surface NURB weights"
            );
        }
    } // end scope: Move UsdBrepArray PerBrep shape object data to UsdBrepArrayData

    return bRtn;

} // end UsdBrepData::BrepReadFromUsdStage

} // end namespace UsdBrepData
