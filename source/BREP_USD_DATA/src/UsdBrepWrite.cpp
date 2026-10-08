// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepWrite.cpp
 * PURPOSE: UsdBrepArrayData to UsdBrepArray translations
 * ******************************************************************************************************************/

#include "UsdBrepWrite.h"

#include "UsdBrepArrayData.h"
#include "UsdBrepDiagnostics.h"
#include "UsdBrepTokens.h"
#include "UsdBrepUtilities.h"

// USD includes
#include "UsdBrepHeaders.h"

#include <algorithm>
#include <cassert>

/*********************************************************************************************************************
 PURPOSE: UsdBrepArrayData method implementations

 NOTES:
 ********************************************************************************************************************/
namespace UsdBrepData
{

// clang-format off
/*********************************************************************************************************************
 PURPOSE: move UsdBrepArrayData data into the rUsdBrepArray

 NOTES:
 I. USD BrepArray data usage for prim     = a OmniSolidBrepArray object
                                  primSpec = related PrimSpec on root layer
                                  primPath = path to base prim
    1. BrepArray meta data
       a) path  = "PrimPath.source"          -> customData(token="source", value=<std::string>)     = origin CAD system string label
       b) path  = "PrimPath.extent"          -> attribute (token="extent", value= VtArray<GfVec3f>) = BBox (union of all Brep BBoxes)

    2. BrepArray Topo and Geom data
       c) paths = "PrimPath.AttribTokenName" -> all BrepArray schema attributes
       d) paths = "PrimPath.AttribTokenName" -> all BrepGeometry applied APIs attributes

    3. Materials for perBrepArray, perBrep, and perFace uses
        e) path = "PrimPath.materal:binding" -> relationship(token:["materialBinding"]) for BrepArray material = default for all Brep materials
        f) path = "PrimPath/subset_ii"        -> prim.subGeom, one per BrepMaterial when different Breps use different Materials
        f.1) path = "PrimPath/subset_ii.elementType"     -> attribute   (token:["elementType"], value:[token=["brep"]) 
                                                         -> attribute   (token:["elementType"], value:[token=["face"])
        f.2) path = "PrimPath/subset_ii.indices" -> attribute   (token:["indices"],     value:[VtArray<uint32_t>])
             materialBinding
        f.3) path = "PrimPath/subset_ii.material:binding" -> relationship(token:["materialBinding"])
             rendering material path for breps or faces in the indices list
 ********************************************************************************************************************/
// clang-format on
bool BrepWriteToUsdStage(
    const UsdBrepArrayData& rArrays, // in : UsdBrepArrayData being read
    pxr::UsdPrim& rUsdBrepArray //      out: USDBrep prim
)
{
    // Check if prim is valid
    if (!rUsdBrepArray)
    {
        USDBREP_ERROR("Invalid UsdPrim provided to BrepWriteToUsdStage");
        return false;
    }

    // check state - Verify 'omniSolid/resources' is registered
    assert(IsOmniSolidResourcesPluginRegistered());

    // Get the usd stage
    pxr::UsdStageRefPtr sStage = rUsdBrepArray.GetStage();
    if (!sStage)
    {
        USDBREP_ERROR("rUsdBrepArray has no valid stage.");
        return false;
    }

    // get the path for the UsdBrepArray
    pxr::SdfPath sBrepArrayPath = rUsdBrepArray.GetPath();

    // Get the UsdBrepArray primSpecHandle
    pxr::SdfLayerHandle rootLayer = sStage->GetRootLayer();
    if (!rootLayer)
    {
        USDBREP_ERROR("Failed to get root layer from stage.");
        return false;
    }

    // Check if the path is valid
    if (sBrepArrayPath.IsEmpty())
    {
        USDBREP_ERROR("Empty SdfPath provided to BrepWriteToUsdStage");
        return false;
    }

    pxr::SdfPrimSpecHandle sUsdBrepArraySpecHandle = rootLayer->GetPrimAtPath(sBrepArrayPath);

    // no work - missing or dormant sUsdBrepArraySpecHandle input
    if (!sUsdBrepArraySpecHandle || sUsdBrepArraySpecHandle->IsDormant())
    {
        USDBREP_ERROR("Invalid input. No or dormant sUsdBrepArraySpecHandle input.");
        return false;
    }

    // Open the changeblock - for efficiency, accumulate all change notices to send at once when this ChangeBlock exits scope
    SdfChangeBlock sChangeBlock;

    // begin scope: Move UsdBrepArrayData data to UsdBrepArray
    {
        // UsdBrepArray.GetPrim() path - no work the prim spec path is already a part of its definition

        // BrepArray source string
        sUsdBrepArraySpecHandle->SetCustomData(UsdBrepSolidTokens->source, VtValue(rArrays.m_sCADSource));

        // BBox = stored union of all individual Brep_ii BBoxes - inherited from UsdGeomBoundable
        pxr::SdfAttributeSpecHandle extentAttr = setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdGeomTokens->extent,
            SdfValueTypeNames->Float3Array,
            VtValue(VtArray<GfVec3f>{ rArrays.m_sBrepArray_BBox.GetMin(), rArrays.m_sBrepArray_BBox.GetMax() })
        );

        if (!extentAttr)
        {
            USDBREP_ERROR("Failed to set extent attribute on UsdBrepArray");
            return false;
        }

        // material properties
        size_t lBrepMaterialCount = 0;

        // material path and index arrays must pair up, or assignments would be silently dropped
        auto pairedArrays = [](const char* pWhat, size_t lPaths, size_t lIndexLists)
        {
            if (lPaths != lIndexLists)
            {
                USDBREP_ERROR("%s material path and index arrays disagree (%zu paths, %zu index lists)", pWhat, lPaths, lIndexLists);
            }
            return lPaths == lIndexLists;
        };
        if (!pairedArrays("Brep", rArrays.m_sBrepMaterial_BrepPathArray.size(), rArrays.m_sBrepMaterial_BrepIndexArray.size()) ||
            !pairedArrays("Face", rArrays.m_sFaceMaterial_FacePathArray.size(), rArrays.m_sFaceMaterial_FaceIndexArray.size()))
        {
            return false;
        }
        const size_t lBrepMaterialPairs = rArrays.m_sBrepMaterial_BrepPathArray.size();

        // true when the indices name every Brep (a count match would accept duplicates)
        auto coversEveryBrep = [&rArrays](const VtIntArray& crIndices)
        {
            std::vector<bool> sCovered(rArrays.TotalBrepCount(), false);
            for (int iIndex : crIndices)
            {
                if (iIndex < 0 || static_cast<size_t>(iIndex) >= sCovered.size())
                {
                    return false;
                }
                sCovered[static_cast<size_t>(iIndex)] = true;
            }
            return std::find(sCovered.begin(), sCovered.end(), false) == sCovered.end();
        };
        // one per-Brep material on every Brep; entries without a material path are ignored
        const bool bSharedBrepMaterial = lBrepMaterialPairs == 1 && !rArrays.m_sBrepMaterial_BrepPathArray[0].IsEmpty() &&
                                         coversEveryBrep(rArrays.m_sBrepMaterial_BrepIndexArray[0]);

        // BrepArray binding: the shared material, else the BrepArray material (which also covers
        // Breps outside every subset); none when empty, as a targetless binding binds to nothing
        const SdfPath sArrayMaterialPath = bSharedBrepMaterial ? rArrays.m_sBrepMaterial_BrepPathArray[0] : rArrays.m_sBrepArray_MaterialPath;
        if (!sArrayMaterialPath.IsEmpty())
        {
            // without the applied API, the binding resolves only with legacy bindings allowed
            if (!AddAppliedSchema(sUsdBrepArraySpecHandle, UsdSchemaRegistry::GetSchemaTypeName(TfType::Find<UsdShadeMaterialBindingAPI>())))
            {
                USDBREP_ERROR("Failed to add UsdShadeMaterialBindingAPI schema to the BrepArray");
                return false;
            }
            SdfRelationshipSpecHandle matBinding = SdfRelationshipSpec::New(sUsdBrepArraySpecHandle, UsdShadeTokens->materialBinding, false);
            if (!matBinding)
            {
                USDBREP_ERROR("Failed to create material binding relationship spec");
                return false;
            }
            matBinding->GetTargetPathList().ClearEditsAndMakeExplicit();
            matBinding->GetTargetPathList().Add(sArrayMaterialPath.MakeAbsolutePath(sUsdBrepArraySpecHandle->GetPath()));
        }

        // otherwise one materialBinding GeomSubset per per-Brep material
        if (lBrepMaterialPairs > 0 && !bSharedBrepMaterial)
        {
            // For each BrepMaterial - make a materialBinding geomSubset child of UsdBrepArray with path =
            // "PrimPath/subset_ii"
            //
            //   set sGeomSubsetPrimSpecHandle->attribute  "PrimPath/subset_ii.elementType =
            //                                             ["brep"]specifies indices are Brep indices
            //
            //       sGeomSubsetPrimSpecHandle->attribute  "PrimPath/subset_ii.indices     = VtArray<uint32_t>,
            //             array of BrepIndices using this material binding sGeomSubsetPrimSpecHandle->relationship
            //
            //       "PrimPath/subset_ii.material:binding = relationship set with one Material SdfPath used as the
            //       display material for the brep or faces lists in indices
            for (size_t ii = 0; ii < lBrepMaterialPairs; ++ii)
            {
                // no work for this ii - no Breps or no material
                if (rArrays.m_sBrepMaterial_BrepIndexArray[ii].size() == 0 || rArrays.m_sBrepMaterial_BrepPathArray[ii].IsEmpty())
                {
                    continue;
                }

                // new geomset - ("PrimPath/subset_N"), N counts the subsets written so far, so skipped
                // entries leave no gap for the face subsets below to collide with
                SdfPrimSpecHandle sGeomSubsetPrimSpecHandle = SdfPrimSpec::New(
                    sUsdBrepArraySpecHandle,
                    "subset_" + std::to_string(lBrepMaterialCount),
                    SdfSpecifierDef,
                    "GeomSubset"
                );
                if (!sGeomSubsetPrimSpecHandle)
                {
                    USDBREP_ERROR("%s", ("Failed to create GeomSubset prim spec for subset_" + std::to_string(lBrepMaterialCount)).c_str());
                    return false;
                }
                bool addSchemaResult = AddAppliedSchema(
                    sGeomSubsetPrimSpecHandle,
                    UsdSchemaRegistry::GetSchemaTypeName(TfType::Find<UsdShadeMaterialBindingAPI>())
                );
                if (!addSchemaResult)
                {
                    USDBREP_ERROR("%s", ("Failed to add UsdShadeMaterialBindingAPI schema to subset_" + std::to_string(lBrepMaterialCount)).c_str());
                    return false;
                }

                // geomset_ii->attributes  - ("PrimPath/subset_ii.elementType", "PrimPath/subset_ii.indices")
                pxr::SdfAttributeSpecHandle elementTypeAttr = setAttributeSpec(
                    sGeomSubsetPrimSpecHandle,
                    UsdGeomTokens->elementType,
                    SdfValueTypeNames->Token,
                    VtValue(UsdBrepSolidTokens->brep)
                );

                if (!elementTypeAttr)
                {
                    USDBREP_ERROR("%s", ("Failed to set elementType attribute on subset_" + std::to_string(lBrepMaterialCount)).c_str());
                    return false;
                }
                pxr::SdfAttributeSpecHandle indicesAttr = setAttributeSpec(
                    sGeomSubsetPrimSpecHandle,
                    UsdGeomTokens->indices,
                    SdfValueTypeNames->IntArray,
                    VtValue(rArrays.m_sBrepMaterial_BrepIndexArray[ii])
                );

                if (!indicesAttr)
                {
                    USDBREP_ERROR("%s", ("Failed to set indices attribute on subset_" + std::to_string(lBrepMaterialCount)).c_str());
                    return false;
                }

                // matBinding = geomset_ii->relationship - ("PrimPath/subset_ii.material:binding")
                SdfRelationshipSpecHandle matBinding = SdfRelationshipSpec::New(sGeomSubsetPrimSpecHandle, UsdShadeTokens->materialBinding, false);
                if (!matBinding)
                {
                    USDBREP_ERROR(
                        "%s",
                        ("Failed to create material binding relationship spec for subset_" + std::to_string(lBrepMaterialCount)).c_str()
                    );
                    return false;
                }

                // add sBrepMaterialPaths[ii] to matBinding using matBinding->listOperator
                matBinding->GetTargetPathList().ClearEditsAndMakeExplicit(); // make implied paths explicit
                matBinding->GetTargetPathList().Add(rArrays.m_sBrepMaterial_BrepPathArray[ii].MakeAbsolutePath(sUsdBrepArraySpecHandle->GetPath()));

                // count the Brep materials
                lBrepMaterialCount++;
            } // end iter every BrepMaterial
        } // end different Breps use different materials check

        // when faces are using face materials - build a materialBinding GeomSubset.{elementType, indices,
        // materialBinding} for each material
        if (rArrays.m_sFaceMaterial_FacePathArray.size() > 0)
        {
            // For each FaceMaterial - make a materialBinding sGeomSubsetPrimSpecHandle child of UsdBrepArray
            // with path = "PrimPath/subset_ii"
            //
            //   set sGeomSubsetPrimSpecHandle->attribute    "PrimPath/subset_ii.elementType =
            //           ["face"] specifies indices are Face indices
            //
            //   sGeomSubsetPrimSpecHandle->attribute    "PrimPath/subset_ii.indices = VtArray<uint32_t>, array of
            //       FaceIndices using this material binding sGeomSubsetPrimSpecHandle->relationship
            //       "PrimPath/subset_ii.material:binding = relationship set with one Material SdfPath used as the
            //       display material for the brep or faces lists in indices
            for (size_t ii = 0; ii < rArrays.m_sFaceMaterial_FacePathArray.size(); ++ii)
            {
                // new geomset_ii          - ("PrimPath/subset_(ii + lBrepMaterialCount))
                SdfPrimSpecHandle sGeomSubsetPrimSpecHandle = SdfPrimSpec::New(
                    sUsdBrepArraySpecHandle,
                    "subset_" + std::to_string(ii + lBrepMaterialCount),
                    SdfSpecifierDef,
                    "GeomSubset"
                );

                if (!sGeomSubsetPrimSpecHandle)
                {
                    USDBREP_ERROR("%s", ("Failed to create GeomSubset prim spec for subset_" + std::to_string(ii + lBrepMaterialCount)).c_str());
                    return false;
                }
                bool addSchemaResult = AddAppliedSchema(
                    sGeomSubsetPrimSpecHandle,
                    UsdSchemaRegistry::GetSchemaTypeName(TfType::Find<UsdShadeMaterialBindingAPI>())
                );
                if (!addSchemaResult)
                {
                    USDBREP_ERROR(
                        "%s",
                        ("Failed to add UsdShadeMaterialBindingAPI schema to subset_" + std::to_string(ii + lBrepMaterialCount)).c_str()
                    );
                    return false;
                }

                // geomset_ii->attributes  - ("PrimPath/subset_(ii + lBrepMaterialCount).elementType",
                // "PrimPath/subset_(ii + lBrepMaterialCount).indices")
                pxr::SdfAttributeSpecHandle elementTypeAttr = setAttributeSpec(
                    sGeomSubsetPrimSpecHandle,
                    UsdGeomTokens->elementType,
                    SdfValueTypeNames->Token,
                    VtValue(UsdBrepSolidTokens->face)
                );

                if (!elementTypeAttr)
                {
                    USDBREP_ERROR("%s", ("Failed to set elementType attribute on subset_" + std::to_string(ii + lBrepMaterialCount)).c_str());
                    return false;
                }
                pxr::SdfAttributeSpecHandle indicesAttr = setAttributeSpec(
                    sGeomSubsetPrimSpecHandle,
                    UsdGeomTokens->indices,
                    SdfValueTypeNames->IntArray,
                    VtValue(rArrays.m_sFaceMaterial_FaceIndexArray[ii])
                );

                if (!indicesAttr)
                {
                    USDBREP_ERROR("%s", ("Failed to set indices attribute on subset_" + std::to_string(ii + lBrepMaterialCount)).c_str());
                    return false;
                }

                // matBinding = geomset_ii->relationship - ("PrimPath/subset_(ii +
                // lBrepMaterialCount).material:binding")
                SdfRelationshipSpecHandle matBinding = SdfRelationshipSpec::New(sGeomSubsetPrimSpecHandle, UsdShadeTokens->materialBinding, false);
                if (!matBinding)
                {
                    USDBREP_ERROR(
                        "%s",
                        ("Failed to create material binding relationship spec for subset_" + std::to_string(ii + lBrepMaterialCount)).c_str()
                    );
                    return false;
                }

                // add sBrepMaterialPaths[ii] to matBinding using matBinding->listOperator
                matBinding->GetTargetPathList().ClearEditsAndMakeExplicit(); // make implied paths explicit
                if (!rArrays.m_sFaceMaterial_FacePathArray[ii].IsEmpty())
                {
                    // add per FaceMaterialPath to PathList
                    matBinding->GetTargetPathList().Add(rArrays.m_sFaceMaterial_FacePathArray[ii].MakeAbsolutePath(sUsdBrepArraySpecHandle->GetPath())
                    );
                }

            } // end iter every FaceMaterialPath
        } // end faces are using face materials check
    } // end scope: Move UsdBrepArrayData data to UsdBrepArray

    // begin scope: Move UsdBrepArrayData topology object data to UsdBrepArraySpecHandle
    {
        // move Brep metadata to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->brepIntersectTol3d,
            SdfValueTypeNames->DoubleArray,
            VtValue(rArrays.m_sBrepXSectTol3dArray)
        );

        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->brepExtent,
            SdfValueTypeNames->Double3Array,
            VtValue(rArrays.m_sBrepExtentArray)
        );


        // moveBrep RegionCount to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->brepRegionCount,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sBrepRegionCountArray)
        );

        // move Regions to UsdBrepArraySpecHandle
        setAttributeSpec(sUsdBrepArraySpecHandle, UsdBrepSolidTokens->regionType, SdfValueTypeNames->TokenArray, VtValue(rArrays.m_sRegionTypeArray));
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->regionShellCount,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sRegionShellCountArray)
        );

        // move Shells to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->shellFaceuseCount,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sShellFaceuseCountArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->shellWireEdgeCount,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sShellWireEdgeCountArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->shellPointType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sShellPointTypeArray)
        );

        // move Faceuses to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->faceuseFaceIndex,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sFaceuseFaceIndexArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->faceuseOrientationType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sFaceuseOrientationTypeArray)
        );

        // move Faces to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->faceLoopCount,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sFaceLoopCountArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->faceSurfaceType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sFaceSurfaceTypeArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->faceTrimType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sFaceTrimTypeArray)
        );
        setAttributeSpec(sUsdBrepArraySpecHandle, UsdBrepSolidTokens->faceRange, SdfValueTypeNames->Double2Array, VtValue(rArrays.m_sFaceRangeArray));

        // move Loops to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->loopEdgeuseCount,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sLoopEdgeuseCountArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->loopVertexIndex,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sLoopVertexIndexArray)
        );

        // move Edgeuses to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->edgeuseEdgeIndex,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sEdgeuseEdgeIndexArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->edgeuseOrientationType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sEdgeuseOrientationTypeArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->edgeuseNextRadialEUIndex,
            SdfValueTypeNames->UIntArray,
            VtValue(rArrays.m_sEdgeuseNextRadialEUIndexArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->edgeuseThisRadialEntryType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sEdgeuseThisRadialEntryTypeArray)
        );

        // move Edges to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->edgeCurveType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sEdgeCurveTypeArray)
        );
        setAttributeSpec(sUsdBrepArraySpecHandle, UsdBrepSolidTokens->edgeRange, SdfValueTypeNames->DoubleArray, VtValue(rArrays.m_sEdgeRangeArray));
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->edgeVertexIndices,
            SdfValueTypeNames->Int2Array,
            VtValue(rArrays.m_sEdgeVertexIndicesArray)
        );

        // move WireEdges to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->wireEdgeCurveType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sWireEdgeCurveTypeArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->wireEdgeRange,
            SdfValueTypeNames->DoubleArray,
            VtValue(rArrays.m_sWireEdgeRangeArray)
        );
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->wireEdgeVertexIndices,
            SdfValueTypeNames->Int2Array,
            VtValue(rArrays.m_sWireEdgeVertexIndicesArray)
        );

        // move Vertices to UsdBrepArraySpecHandle
        setAttributeSpec(
            sUsdBrepArraySpecHandle,
            UsdBrepSolidTokens->vertexPointType,
            SdfValueTypeNames->TokenArray,
            VtValue(rArrays.m_sVertexPointTypeArray)
        );

    } // end scope: Move UsdBrepArrayData topology object data to UsdBrepArraySpecHandle

    // begin scope: Move UsdBrepArrayData geometry object data to UsdBrepArraySpecHandle
    {
        // move VertexPoint Positions to UsdBrepArraySpecHandle
        if (rArrays.m_sVertex_PointPositionArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSolidTokens->brepPointAPI, UsdBrepSolidTokens->vertexPoint);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSolidTokens->vertexPoint,
                UsdBrepSolidTokens->brepPointPosition,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sVertex_PointPositionArray)
            );
        }

        // move ShellPoint Positions to UsdBrepArraySpecHandle
        if (rArrays.m_sShell_PointPositionArray.size() > 0)
        {
            // Add ShellPoint Position geometry API to UsdBrepArraySpecHandle
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSolidTokens->brepPointAPI, UsdBrepSolidTokens->shellPoint);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSolidTokens->shellPoint,
                UsdBrepSolidTokens->brepPointPosition,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sShell_PointPositionArray)
            );
        }

        // move Edge 3d CircleCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sEdge_CurveCircle_RadiusArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dCircleAPI, UsdBrepCurveTokens->edge3dCircle);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dCircle,
                UsdBrepCurveTokens->brep3dCircleCenter,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sEdge_CurveCircle_CenterArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dCircle,
                UsdBrepCurveTokens->brep3dCircleAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sEdge_CurveCircle_AxisArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dCircle,
                UsdBrepCurveTokens->brep3dCircleRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sEdge_CurveCircle_RefDirectionArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dCircle,
                UsdBrepCurveTokens->brep3dCircleRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdge_CurveCircle_RadiusArray)
            );
        }

        // move Edge 3d LineCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sEdge_CurveLine_OriginArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dLineAPI, UsdBrepCurveTokens->edge3dLine);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dLine,
                UsdBrepCurveTokens->brep3dLineOrigin,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sEdge_CurveLine_OriginArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dLine,
                UsdBrepCurveTokens->brep3dLineDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sEdge_CurveLine_DirectionArray)
            );
        }

        // move Edge 3d EllipseCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sEdge_CurveEllipse_CenterArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dEllipseAPI, UsdBrepCurveTokens->edge3dEllipse);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseCenter,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sEdge_CurveEllipse_CenterArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sEdge_CurveEllipse_AxisArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sEdge_CurveEllipse_RefDirectionArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseXRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdge_CurveEllipse_XRadiusArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseYRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdge_CurveEllipse_YRadiusArray)
            );
        }

        // move Edge 3d BSplineCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sEdge_CurveNurb_ControlVerticesArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dNurbAPI, UsdBrepCurveTokens->edge3dNurb);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dNurb,
                UsdBrepCurveTokens->brep3dNurbControlVertices,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sEdge_CurveNurb_ControlVerticesArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dNurb,
                UsdBrepCurveTokens->brep3dNurbVertexCount,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sEdge_CurveNurb_VertexCountArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dNurb,
                UsdBrepCurveTokens->brep3dNurbOrder,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sEdge_CurveNurb_OrderArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dNurb,
                UsdBrepCurveTokens->brep3dNurbKnots,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdge_CurveNurb_KnotsArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->edge3dNurb,
                UsdBrepCurveTokens->brep3dNurbWeights,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdge_CurveNurb_WeightsArray)
            );
        }

        // move WireEdge 3d CircleCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sWireEdge_CurveCircle_RadiusArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dCircleAPI, UsdBrepCurveTokens->wireEdge3dCircle);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dCircle,
                UsdBrepCurveTokens->brep3dCircleCenter,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sWireEdge_CurveCircle_CenterArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dCircle,
                UsdBrepCurveTokens->brep3dCircleAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sWireEdge_CurveCircle_AxisArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dCircle,
                UsdBrepCurveTokens->brep3dCircleRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sWireEdge_CurveCircle_RefDirectionArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dCircle,
                UsdBrepCurveTokens->brep3dCircleRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sWireEdge_CurveCircle_RadiusArray)
            );
        }

        // move WireEdge 3d LineCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sWireEdge_CurveLine_OriginArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dLineAPI, UsdBrepCurveTokens->wireEdge3dLine);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dLine,
                UsdBrepCurveTokens->brep3dLineOrigin,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sWireEdge_CurveLine_OriginArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dLine,
                UsdBrepCurveTokens->brep3dLineDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sWireEdge_CurveLine_DirectionArray)
            );
        }

        // move WireEdge 3d EllipseCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sWireEdge_CurveEllipse_CenterArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dEllipseAPI, UsdBrepCurveTokens->wireEdge3dEllipse);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseCenter,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sWireEdge_CurveEllipse_CenterArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sWireEdge_CurveEllipse_AxisArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sWireEdge_CurveEllipse_RefDirectionArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseXRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sWireEdge_CurveEllipse_XRadiusArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dEllipse,
                UsdBrepCurveTokens->brep3dEllipseYRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sWireEdge_CurveEllipse_YRadiusArray)
            );
        }

        // move WireEdge 3d BSplineCurve to UsdBrepArraySpecHandle
        if (rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray.size() > 0)
        {
            AddMultiAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurve3dNurbAPI, UsdBrepCurveTokens->wireEdge3dNurb);

            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dNurb,
                UsdBrepCurveTokens->brep3dNurbControlVertices,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dNurb,
                UsdBrepCurveTokens->brep3dNurbVertexCount,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sWireEdge_CurveNurb_VertexCountArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dNurb,
                UsdBrepCurveTokens->brep3dNurbOrder,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sWireEdge_CurveNurb_OrderArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dNurb,
                UsdBrepCurveTokens->brep3dNurbKnots,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sWireEdge_CurveNurb_KnotsArray)
            );
            setMultiAppliedAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->wireEdge3dNurb,
                UsdBrepCurveTokens->brep3dNurbWeights,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sWireEdge_CurveNurb_WeightsArray)
            );
        }

        // VertexCount and Order carry one record per edgeuse. A (0, 0) record means that
        // edgeuse has no UV trim curve and consumes no packed control vertices, weights,
        // or knots. Apply the schema when at least one real curve exists; an all-zero or
        // empty set carries no UV data and does not need BrepCurveUvNurbAPI.
        bool hasUvCurves = std::any_of(
            rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.begin(),
            rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.end(),
            [](uint32_t v)
            {
                return v != 0;
            }
        );
        if (hasUvCurves)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepCurveTokens->brepCurveUvNurbAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->brepCurveUvNurbControlVertices,
                SdfValueTypeNames->Double2Array,
                VtValue(rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->brepCurveUvNurbVertexCount,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sEdgeuse_CurveNurb_VertexCountArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->brepCurveUvNurbOrder,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sEdgeuse_CurveNurb_OrderArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->brepCurveUvNurbKnots,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdgeuse_CurveNurb_KnotsArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepCurveTokens->brepCurveUvNurbWeights,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sEdgeuse_CurveNurb_WeightsArray)
            );
        }

        // move SphereSurfaces to UsdBrepArraySpecHandle
        if (rArrays.m_sFace_SurfaceSphere_RadiusArray.size() > 0)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSurfaceTokens->brepSurfaceSphereAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceSphereCenter,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sFace_SurfaceSphere_CenterArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceSphereAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceSphere_AxisArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceSphereRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceSphere_RefDirectionArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceSphereRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceSphere_RadiusArray)
            );
        }

        // move PlaneSurfaces to UsdBrepArraySpecHandle
        if (rArrays.m_sFace_SurfacePlane_OriginArray.size() > 0)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSurfaceTokens->brepSurfacePlaneAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfacePlaneOrigin,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sFace_SurfacePlane_OriginArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfacePlaneAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfacePlane_AxisArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfacePlaneRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfacePlane_RefDirectionArray)
            );
        }

        // move CylinderSurfaces to UsdBrepArraySpecHandle
        if (rArrays.m_sFace_SurfaceCylinder_RadiusArray.size() > 0)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSurfaceTokens->brepSurfaceCylinderAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceCylinderOrigin,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sFace_SurfaceCylinder_OriginArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceCylinderAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceCylinder_AxisArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceCylinderRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceCylinder_RefDirectionArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceCylinderRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceCylinder_RadiusArray)
            );
        }

        // move ConeSurfaces to UsdBrepArraySpecHandle
        if (rArrays.m_sFace_SurfaceCone_RadiusArray.size() > 0)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSurfaceTokens->brepSurfaceConeAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceConeOrigin,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sFace_SurfaceCone_OriginArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceConeAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceCone_AxisArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceConeRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceCone_RefDirectionArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceConeRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceCone_RadiusArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceConeSemiAngle,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceCone_SemiAngleArray)
            );
        }

        // move TorusSurfaces to UsdBrepArraySpecHandle
        if (rArrays.m_sFace_SurfaceTorus_MajorRadiusArray.size() > 0)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSurfaceTokens->brepSurfaceTorusAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceTorusOrigin,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sFace_SurfaceTorus_OriginArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceTorusAxis,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceTorus_AxisArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceTorusRefDirection,
                SdfValueTypeNames->Vector3dArray,
                VtValue(rArrays.m_sFace_SurfaceTorus_RefDirectionArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceTorusMajorRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceTorus_MajorRadiusArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceTorusMinorRadius,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceTorus_MinorRadiusArray)
            );
        }

        // move 3dBSplineSurfaces to UsdBrepArraySpecHandle
        if (rArrays.m_sFace_SurfaceNurb_ControlVerticesArray.size() > 0)
        {
            AddAppliedSchema(sUsdBrepArraySpecHandle, UsdBrepSurfaceTokens->brepSurfaceNurbAPI);

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbControlVertices,
                SdfValueTypeNames->Point3dArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_ControlVerticesArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbUKnots,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_UKnotsArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbUOrder,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_UOrderArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbUVertexCount,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_UVertexCountArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbVKnots,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_VKnotsArray)
            );

            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbVOrder,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_VOrderArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbVVertexCount,
                SdfValueTypeNames->UIntArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_VVertexCountArray)
            );
            setAttributeSpec(
                sUsdBrepArraySpecHandle,
                UsdBrepSurfaceTokens->brepSurfaceNurbWeights,
                SdfValueTypeNames->DoubleArray,
                VtValue(rArrays.m_sFace_SurfaceNurb_WeightsArray)
            );
        }
    } // end scope: Move UsdBrepArrayData geometry object data to UsdBrepArraySpecHandle

    // all done
    return (true);
} // end UsdBrepData::BrepWriteToUsdStage

} // end namespace UsdBrepData
