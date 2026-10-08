// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepArrayData.cpp
 * PURPOSE: UsdBrepArrayData, UsdBrepArraySpans methods and global functions
 * ******************************************************************************************************************/

#include "UsdBrepArrayData.h"

#include "UsdBrepDiagnostics.h"
#include "UsdBrepTokens.h"
#include "UsdBrepUtilities.h"

#include <cassert>

// USD includes
#include "UsdBrepHeaders.h"

/*********************************************************************************************************************
 PURPOSE: UsdBrepArrayData method implementations

 ********************************************************************************************************************/
namespace UsdBrepData
{

/*********************************************************************************************************************
 * UsdBrepArrayData Topology Object counts
 * ******************************************************************************************************************/

// total brep count in UsdBrepArrayData
uint32_t UsdBrepArrayData::TotalBrepCount() const
{
    return m_sBrepRegionCountArray.size();
}

// total region count = Sum(Brep_ii->regionCount)
uint32_t UsdBrepArrayData::TotalRegionCount() const
{
    return m_sRegionShellCountArray.size();
}

// total shell count = Sum(Brep_ii->shellCount)
uint32_t UsdBrepArrayData::TotalShellCount() const
{
    return m_sShellFaceuseCountArray.size();
}

// total faceuse count = Sum(Brep_ii->faceuseCount)
uint32_t UsdBrepArrayData::TotalFaceuseCount() const
{
    return m_sFaceuseFaceIndexArray.size();
}

// total face count = Sum(Brep_ii->faceCount)
uint32_t UsdBrepArrayData::TotalFaceCount() const
{
    return m_sFaceLoopCountArray.size();
}

// total loop count = Sum(Brep_ii->loopCount)
uint32_t UsdBrepArrayData::TotalLoopCount() const
{
    return m_sLoopEdgeuseCountArray.size();
}

// total edgeuse count = Sum(Brep_ii->edgeuseCount)
uint32_t UsdBrepArrayData::TotalEdgeuseCount() const
{
    return m_sEdgeuseEdgeIndexArray.size();
}

// total faceEdge count = Sum(Brep_ii->faceEdgeCount)
uint32_t UsdBrepArrayData::TotalEdgeCount() const
{
    return m_sEdgeCurveTypeArray.size();
}

// total wireEdge count = Sum(Brep_ii->wireEdgeCount)
uint32_t UsdBrepArrayData::TotalWireEdgeCount() const
{
    return m_sWireEdgeCurveTypeArray.size();
}

// total vertex count = Sum(Brep_ii->LoopVertexCount + EdgeVertexCount)
uint32_t UsdBrepArrayData::TotalVertexCount() const
{
    return m_sVertexPointTypeArray.size();
}

/*********************************************************************************************************************
 * UsdBrepArrayData Geometry Object counts
 * ******************************************************************************************************************/

uint32_t UsdBrepArrayData::TotalShellVertexPositionCount() const
{
    return m_sShell_PointPositionArray.empty() ? 0 : m_sShell_PointPositionArray.size();
}

uint32_t UsdBrepArrayData::TotalVertexPositionCount() const
{
    return m_sVertex_PointPositionArray.empty() ? 0 : m_sVertex_PointPositionArray.size();
}

uint32_t UsdBrepArrayData::TotalEdgeCurveCount() const
{
    return m_sEdge_CurveNurb_OrderArray.empty() ? 0 : m_sEdge_CurveNurb_OrderArray.size();
}

uint32_t UsdBrepArrayData::TotalEdgeControlVerticesCount() const
{
    return m_sEdge_CurveNurb_ControlVerticesArray.empty() ? 0 : m_sEdge_CurveNurb_ControlVerticesArray.size();
}

uint32_t UsdBrepArrayData::TotalEdgeKnotCount() const
{
    return m_sEdge_CurveNurb_KnotsArray.empty() ? 0 : m_sEdge_CurveNurb_KnotsArray.size();
}

uint32_t UsdBrepArrayData::TotalWireEdgeCurveCount() const
{
    return m_sWireEdge_CurveNurb_OrderArray.empty() ? 0 : m_sWireEdge_CurveNurb_OrderArray.size();
}

uint32_t UsdBrepArrayData::TotalWireEdgeControlVerticesCount() const
{
    return m_sWireEdge_CurveNurb_ControlVerticesArray.empty() ? 0 : m_sWireEdge_CurveNurb_ControlVerticesArray.size();
}

uint32_t UsdBrepArrayData::TotalWireEdgeKnotCount() const
{
    return m_sWireEdge_CurveNurb_KnotsArray.empty() ? 0 : m_sWireEdge_CurveNurb_KnotsArray.size();
}

// count Edgeuse entries with and without Zero entries
uint32_t UsdBrepArrayData::TotalEdgeuseCurveCount() const
{
    return m_sEdgeuse_CurveNurb_VertexCountArray.empty() ? 0 : m_sEdgeuse_CurveNurb_VertexCountArray.size();
}

// Count Edgeuse entries with nonZero entries.
uint32_t UsdBrepArrayData::TotalEdgeuseNonNullCurveCount() const
{
    uint32_t lCnt = 0;

    // UVTrimCurves are optional. When omitted, place '0' entries in
    //   m_sEdgeuse_CurveNurb_VertexCountArray and
    //   m_sEdgeuse_CurveNurb_OrderArray arrays.
    for (size_t ii = 0; ii < m_sEdgeuse_CurveNurb_VertexCountArray.size(); ii++)
    {
        if (m_sEdgeuse_CurveNurb_VertexCountArray[ii] > 0)
        {
            lCnt++;
        }
    }
    return lCnt;
}
uint32_t UsdBrepArrayData::TotalEdgeuseControlVerticesCount() const
{
    return m_sEdgeuse_CurveNurb_ControlVerticesArray.empty() ? 0 : m_sEdgeuse_CurveNurb_ControlVerticesArray.size();
}
uint32_t UsdBrepArrayData::TotalEdgeuseKnotCount() const
{
    return m_sEdgeuse_CurveNurb_KnotsArray.empty() ? 0 : m_sEdgeuse_CurveNurb_KnotsArray.size();
}
uint32_t UsdBrepArrayData::TotalFaceSurfaceCount() const
{
    return m_sFace_SurfaceNurb_UOrderArray.empty() ? 0 : m_sFace_SurfaceNurb_UOrderArray.size();
}
uint32_t UsdBrepArrayData::TotalFaceControlVerticesCount() const
{
    return m_sFace_SurfaceNurb_ControlVerticesArray.empty() ? 0 : m_sFace_SurfaceNurb_ControlVerticesArray.size();
}
uint32_t UsdBrepArrayData::TotalFaceKnot_UCount() const
{
    return m_sFace_SurfaceNurb_UKnotsArray.empty() ? 0 : m_sFace_SurfaceNurb_UKnotsArray.size();
}
uint32_t UsdBrepArrayData::TotalFaceKnot_VCount() const
{
    return m_sFace_SurfaceNurb_VKnotsArray.empty() ? 0 : m_sFace_SurfaceNurb_VKnotsArray.size();
}

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

// Function to safely access array elements, returning 0 if array is empty or index is out of bounds
template <typename ArrayType>
uint32_t GetArraySubscript(const ArrayType& array, size_t index)
{
    if (array.empty())
    {
        USDBREP_ERROR("GetArraySubscript - array is empty");
        return 0U;
    }
    if (index >= array.size())
    {
        USDBREP_ERROR(
            "%s",
            ("GetArraySubscript - index " + std::to_string(index) + " is out of bounds (array size: " + std::to_string(array.size()) + ")").c_str()
        );
        return 0U;
    }
    return static_cast<uint32_t>(array[index]);
} // end GetArraySubscript

// Function to safely access VtArray<GfVec2i> elements, returning 0 if array is empty or indices are out of bounds
uint32_t GetArray2Subscript(const VtArray<GfVec2i>& array, size_t index_i, size_t index_j)
{
    if (array.empty())
    {
        USDBREP_ERROR("GetArray2Subscript - array is empty");
        return 0U;
    }
    if (index_i >= array.size())
    {
        USDBREP_ERROR(
            "%s",
            ("GetArray2Subscript - index_i " + std::to_string(index_i) + " is out of bounds (array size: " + std::to_string(array.size()) + ")")
                .c_str()
        );
        return 0U;
    }
    if (index_j >= 2)
    { // GfVec2i only has 2 elements (0 and 1)
        USDBREP_ERROR(
            "%s",
            ("GetArray2Subscript - index_j " + std::to_string(index_j) + " is out of bounds (GfVec2i only has 2 elements: 0 and 1)").c_str()
        );
        return 0U;
    }
    return static_cast<uint32_t>(array[index_i][index_j]);
} // end GetArray2Subscript

/*********************************************************************************************************************
 PURPOSE: return number of Breps in a UsdBrepArray

 NOTES:
 ********************************************************************************************************************/
uint32_t GetUsdBrepArray_BrepCount(const pxr::UsdPrim& crUsdBrepArray) // in : target UsdBrepArray to Query
{
    // Check if prim is valid
    if (!crUsdBrepArray)
    {
        USDBREP_ERROR("Invalid UsdPrim provided to GetUsdBrepArray_BrepCount");
        return 0;
    }

    uint32_t lBrepCount = 0;
    VtArray<uint32_t> sBrepRegionCountArray;

    if (UsdBrepGetAttribute(crUsdBrepArray, UsdBrepSolidTokens->brepRegionCount, sBrepRegionCountArray, ""))
    {
        lBrepCount = static_cast<uint32_t>(sBrepRegionCountArray.size());
    }

    return (lBrepCount);

} // end GetUsdBrepArray_BrepCount

/*********************************************************************************************************************
 PURPOSE: Return whether a shell is a BrepPoint shell.

 NOTES: shell:pointType is ignored when the shell contains faceuses or wire edges.
        Return false for mismatched/truncated parallel shell arrays.
 ********************************************************************************************************************/
bool UsdBrepArrayData::IsBrepPointShell(uint32_t iShellIndex) const
{
    return iShellIndex < m_sShellFaceuseCountArray.size() && iShellIndex < m_sShellWireEdgeCountArray.size() &&
           iShellIndex < m_sShellPointTypeArray.size() && m_sShellFaceuseCountArray[iShellIndex] == 0 &&
           m_sShellWireEdgeCountArray[iShellIndex] == 0 && m_sShellPointTypeArray[iShellIndex] == UsdBrepSolidTokens->brepPointAPI;
}

/*********************************************************************************************************************
 PURPOSE: Count total number of VertexShells in all Breps in UsdBrepArrayData

 NOTES: return count of shells satisfying the exact BrepPoint-shell occurrence rule.
 ********************************************************************************************************************/
uint32_t UsdBrepArrayData::TotalShellVertexCount() const
{
    uint32_t lVSCnt = 0;

    // for every Shell - count the ones that store a ShellPoint
    for (uint32_t ii = 0; ii < TotalShellCount(); ii++)
    {
        if (IsBrepPointShell(ii))
        {
            lVSCnt++;
        }
    }

    // all done
    return (lVSCnt);

} // end UsdBrepArrayData::TotalShellVertexCount()

/*********************************************************************************************************************
 PURPOSE: Count total number of LoopVerticess in all Breps in UsdBrepArrayData

 NOTES: return count of LoopVertices that store
        a m_sShellPointTypeArray token value != "none"
 ********************************************************************************************************************/
uint32_t UsdBrepArrayData::TotalLoopVertexCount() const
{
    uint32_t lLSCnt = 0;

    // for every Loop - count the ones that store a LoopVertex
    for (uint32_t ii = 0; ii < TotalLoopCount(); ii++)
    {
        if (m_sLoopEdgeuseCountArray[ii] == 0)
        {
            lLSCnt++;
        }
    }

    // all done
    return (lLSCnt);

} // end UsdBrepArrayData::TotalLoopVertexCount()

/*********************************************************************************************************************
 PURPOSE: Map global UsdBrepArrayData indices to local indices

 NOTES: 1. index vals   : An index value stored in one Brep object within one array of data
                          represents a connection to another Brep object which is found in
                          another array of data at the stored index value.
        3. global index : global index values are used in USD BrepArray prim and UsdBrepArrayData objects to access
                          connected-object data in the associated UsdBrepArray and UsdBrepArrayData object arrays.
                          The first entry for the first Brep will be index 0.  The index entries for the second Brep
                          will start at the first Brep counts.
        2. local index  : local index values for Brep_ii are just the global indices offset so that the first
                          object for Brep_ii is index 0.
        4. index mapping: The difference between a global and local index for a common
                          brep_ii object is just an offset value.
 ********************************************************************************************************************/
uint32_t UsdBrepArrayData::MapGlobalToLocal(uint32_t uiGlobalIndex, uint32_t uiGlobalStartIndex)
{
    return (
        uiGlobalIndex == USDBREP_NO_OBJECT_INDEX ? (USDBREP_NO_OBJECT_INDEX) // EdgeuseLoops store USDBREP_NO_OBJECT_INDEX for
                                                                             // VertexLoop VertexIndex.
                                                   :
                                                   (uiGlobalIndex - uiGlobalStartIndex)
    );

} // end UsdBrepData::usdBrep_MapGlobal_ToLocal

/*********************************************************************************************************************
 PURPOSE: Map local UsdBrepArrayData indices to global indices.

 NOTES: 1. index vals   : An index value stored in one Brep object within one array of data
                          represents a connection to another Brep object which is found in
                          another array of data at the stored index value.
        3. global index : global index values are used in USD BrepArray prim and UsdBrepArrayData objects to access
                          connected-object data in the associated UsdBrepArray and UsdBrepArrayData object arrays.
                          The first entry for the first Brep will be index 0.  The index entries for the second Brep
                          will start at the first Brep counts.
        2. local index  : local index values for Brep_ii are just the global indices offset so that the first
                          object for Brep_ii is index 0.
        4. index mapping: The difference between a global and local index for a common
                          brep_ii object is just an offset value.
 ********************************************************************************************************************/
uint32_t UsdBrepArrayData::MapLocalToGlobal(uint32_t uiLocalIndex, uint32_t uiGlobalStartIndex)
{
    return (
        uiLocalIndex == USDBREP_NO_OBJECT_INDEX ? (USDBREP_NO_OBJECT_INDEX) // EdgeuseLoops store USDBREP_NO_OBJECT_INDEX for
                                                                            // VertexLoop VertexIndex.
                                                  :
                                                  (uiLocalIndex + uiGlobalStartIndex)
    );

} // end UsdBrepData::usdBrep_MapLocal_ToGlobal

/*********************************************************************************************************************
 PURPOSE: Clear all count and startIndex values in
          Tgt UsdBrepArraySpans object.

 NOTES: None
 ********************************************************************************************************************/
void UsdBrepArrayData::ReSet()
{
    // clear BrepArray metadata
    m_sPrimPath = pxr::SdfPath::EmptyPath();
    m_sBrepArray_BBox = GfRange3f();
    m_sCADSource = "";
    m_sBrepArray_MaterialPath = pxr::SdfPath::EmptyPath();
    m_sBrepMaterial_BrepPathArray.clear();
    m_sBrepMaterial_BrepIndexArray.clear();
    m_sFaceMaterial_FacePathArray.clear();
    m_sFaceMaterial_FaceIndexArray.clear();

    // clear BrepArray Schema attributes - one value per BrepArray
    m_sBrepXSectTol3dArray.clear();
    m_sBrepExtentArray.clear();
    m_sBrepRegionCountArray.clear();

    // clear BrepArray Topology - one block of values per BrepTopologyObj -
    // lists of Brep topology object values for all the Breps in a BrepArray organized

    // region USD format
    m_sRegionShellCountArray.clear();
    m_sRegionTypeArray.clear();

    // shell USD format
    m_sShellFaceuseCountArray.clear();
    m_sShellWireEdgeCountArray.clear();
    m_sShellPointTypeArray.clear();


    // faceuse USD format
    m_sFaceuseFaceIndexArray.clear();
    m_sFaceuseOrientationTypeArray.clear();

    // face USD format
    m_sFaceLoopCountArray.clear();
    m_sFaceSurfaceTypeArray.clear();

    m_sFaceTrimTypeArray.clear();
    m_sFaceRangeArray.clear();

    // loop USD format
    m_sLoopEdgeuseCountArray.clear();
    m_sLoopVertexIndexArray.clear();

    // edgeuse USD format
    m_sEdgeuseEdgeIndexArray.clear();
    m_sEdgeuseOrientationTypeArray.clear();
    m_sEdgeuseNextRadialEUIndexArray.clear();
    m_sEdgeuseThisRadialEntryTypeArray.clear();

    // edge USD format
    m_sEdgeCurveTypeArray.clear();

    m_sEdgeRangeArray.clear();
    m_sEdgeVertexIndicesArray.clear();

    // wireEdge USD format
    m_sWireEdgeCurveTypeArray.clear();

    m_sWireEdgeRangeArray.clear();
    m_sWireEdgeVertexIndicesArray.clear();

    // vertex USD format
    m_sVertexPointTypeArray.clear();

    // clear geometry data arrays

    // VertexShell geometry
    m_sShell_PointPositionArray.clear();

    // LoopVertex and EdgeVertex geometry
    m_sVertex_PointPositionArray.clear();

    // Edge NurbCurve geom
    m_sEdge_CurveNurb_ControlVerticesArray.clear();
    m_sEdge_CurveNurb_VertexCountArray.clear();
    m_sEdge_CurveNurb_OrderArray.clear();
    m_sEdge_CurveNurb_KnotsArray.clear();
    m_sEdge_CurveNurb_WeightsArray.clear();

    // Edge Circle curve geom
    m_sEdge_CurveCircle_CenterArray.clear();
    m_sEdge_CurveCircle_AxisArray.clear();
    m_sEdge_CurveCircle_RefDirectionArray.clear();
    m_sEdge_CurveCircle_RadiusArray.clear();

    // Edge Line curve geom
    m_sEdge_CurveLine_OriginArray.clear();
    m_sEdge_CurveLine_DirectionArray.clear();

    // WireEdge NurbCurve geom
    m_sWireEdge_CurveNurb_ControlVerticesArray.clear();
    m_sWireEdge_CurveNurb_VertexCountArray.clear();
    m_sWireEdge_CurveNurb_OrderArray.clear();
    m_sWireEdge_CurveNurb_KnotsArray.clear();
    m_sWireEdge_CurveNurb_WeightsArray.clear();

    // WireEdge Circle curve geom
    m_sWireEdge_CurveCircle_CenterArray.clear();
    m_sWireEdge_CurveCircle_AxisArray.clear();
    m_sWireEdge_CurveCircle_RefDirectionArray.clear();
    m_sWireEdge_CurveCircle_RadiusArray.clear();

    // WireEdge Line curve geom
    m_sWireEdge_CurveLine_OriginArray.clear();
    m_sWireEdge_CurveLine_DirectionArray.clear();

    // Edgeuse UVTrimCurve2d NurbCurve geometry
    m_sEdgeuse_CurveNurb_ControlVerticesArray.clear();
    m_sEdgeuse_CurveNurb_VertexCountArray.clear();
    m_sEdgeuse_CurveNurb_OrderArray.clear();
    m_sEdgeuse_CurveNurb_KnotsArray.clear();
    m_sEdgeuse_CurveNurb_WeightsArray.clear();

    // Face NurbSurface geometry
    m_sFace_SurfaceNurb_ControlVerticesArray.clear();
    m_sFace_SurfaceNurb_UVertexCountArray.clear();
    m_sFace_SurfaceNurb_VVertexCountArray.clear();
    m_sFace_SurfaceNurb_UOrderArray.clear();
    m_sFace_SurfaceNurb_VOrderArray.clear();
    m_sFace_SurfaceNurb_UKnotsArray.clear();
    m_sFace_SurfaceNurb_VKnotsArray.clear();
    m_sFace_SurfaceNurb_WeightsArray.clear();

    // Face SphereSurface geometry
    m_sFace_SurfaceSphere_CenterArray.clear();
    m_sFace_SurfaceSphere_AxisArray.clear();
    m_sFace_SurfaceSphere_RefDirectionArray.clear();
    m_sFace_SurfaceSphere_RadiusArray.clear();

    // Face PlaneSurface geometry
    m_sFace_SurfacePlane_OriginArray.clear();
    m_sFace_SurfacePlane_AxisArray.clear();
    m_sFace_SurfacePlane_RefDirectionArray.clear();

    // Face CylinderSurface geometry
    m_sFace_SurfaceCylinder_OriginArray.clear();
    m_sFace_SurfaceCylinder_AxisArray.clear();
    m_sFace_SurfaceCylinder_RefDirectionArray.clear();
    m_sFace_SurfaceCylinder_RadiusArray.clear();

    // Face ConeSurface geometry
    m_sFace_SurfaceCone_OriginArray.clear();
    m_sFace_SurfaceCone_AxisArray.clear();
    m_sFace_SurfaceCone_RefDirectionArray.clear();
    m_sFace_SurfaceCone_RadiusArray.clear();
    m_sFace_SurfaceCone_SemiAngleArray.clear();

    // Face TorusSurface geometry
    m_sFace_SurfaceTorus_OriginArray.clear();
    m_sFace_SurfaceTorus_AxisArray.clear();
    m_sFace_SurfaceTorus_RefDirectionArray.clear();
    m_sFace_SurfaceTorus_MajorRadiusArray.clear();
    m_sFace_SurfaceTorus_MinorRadiusArray.clear();

    // Edge Ellipse curve geom
    m_sEdge_CurveEllipse_CenterArray.clear();
    m_sEdge_CurveEllipse_AxisArray.clear();
    m_sEdge_CurveEllipse_RefDirectionArray.clear();
    m_sEdge_CurveEllipse_XRadiusArray.clear();
    m_sEdge_CurveEllipse_YRadiusArray.clear();

    // WireEdge Ellipse curve geom
    m_sWireEdge_CurveEllipse_CenterArray.clear();
    m_sWireEdge_CurveEllipse_AxisArray.clear();
    m_sWireEdge_CurveEllipse_RefDirectionArray.clear();
    m_sWireEdge_CurveEllipse_XRadiusArray.clear();
    m_sWireEdge_CurveEllipse_YRadiusArray.clear();

} // end UsdBrepArrayData::ReSet

/*********************************************************************************************************************
 PURPOSE: Clear all count and startIndex values in
          Tgt UsdBrepArraySpans object.

 NOTES: None
 ********************************************************************************************************************/
void UsdBrepArraySpans::ClearStartsAndCounts()
{
    m_lBrepIndex = 0;

    // clear all count values
    ClearCounts();

    // Brep_ii topology global start indices - accumulated as sequence of Breps get parsed
    m_lRegionStartIndex = 0; // global region      startIndex for Brep_ii in the UsdBrepArrayData::m_sRegions...arrays
    m_lShellStartIndex = 0; // global shell       startIndex for Brep_ii in the UsdBrepArrayData::m_sShells...  arrays
    m_lShellVertexStartIndex = 0; // global shellVertex startIndex for Brep_ii in the
                                  // UsdBrepArrayData::m_lShellPointPosition_Count array
    m_lFaceuseStartIndex = 0; // global faceuse     startIndex for Brep_ii in the UsdBrepArrayData::m_sFaceuses...
                              // arrays
    m_lFaceStartIndex = 0; // global face        startIndex for Brep_ii in the UsdBrepArrayData::m_sFaces...    arrays
    m_lLoopStartIndex = 0; // global loop        startIndex for Brep_ii in the UsdBrepArrayData::m_sLoops...    arrays
    m_lEdgeuseStartIndex = 0; // global edgeuse     startIndex for Brep_ii in the UsdBrepArrayData::m_sEdgeuses...
                              // arrays
    m_lEdgeStartIndex = 0; // global edge        startIndex for Brep_ii in the UsdBrepArrayData::m_sEdges...    arrays
    m_lWireEdgeStartIndex = 0; // global wireEdge    startIndex for Brep_ii in the UsdBrepArrayData::m_sWireEdges...
                               // arrays
    m_lVertexStartIndex = 0; // global vertex    startIndex for Brep_ii in the UsdBrepArrayData::m_sVertices...arrays

    // BrepData_ii shape global start indices - accumulated as sequence of Breps get parsed
    m_lVertexPointPosition_StartIndex = 0; // global Index
    m_lShellPointPosition_StartIndex = 0; // global Index

    m_lEdgeBSplineCurve3d_StartIndex = 0; // global Index
    m_lEdgeBSplineCurve3d_ControlVerticesStartIndex = 0; // global Index
    m_lEdgeBSplineCurve3d_KnotStartIndex = 0; // global Index

    m_lEdgeCircleCurve3d_StartIndex = 0;
    m_lEdgeLineCurve3d_StartIndex = 0;
    m_lEdgeEllipseCurve3d_StartIndex = 0;

    m_lWireEdgeBSplineCurve3d_StartIndex = 0; // global Index
    m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex = 0; // global Index
    m_lWireEdgeBSplineCurve3d_KnotStartIndex = 0; // global Index

    m_lWireEdgeCircleCurve3d_StartIndex = 0;
    m_lWireEdgeLineCurve3d_StartIndex = 0;
    m_lWireEdgeEllipseCurve3d_StartIndex = 0;

    m_lEdgeuseBSplineCurve2d_StartIndex = 0; // global Index
    m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex = 0; // global Index
    m_lEdgeuseBSplineCurve2d_KnotStartIndex = 0; // global Index

    m_lFaceBSplineSurface_StartIndex = 0; // global Index
    m_lFaceBSplineSurface_ControlVerticesStartIndex = 0; // global Index
    m_lFaceBSplineSurface_UKnotStartIndex = 0; // global Index
    m_lFaceBSplineSurface_VKnotStartIndex = 0; // global Index

    m_lFaceSphereSurface_StartIndex = 0;
    m_lFaceSphereSurface_Count = 0;
    m_lFacePlaneSurface_StartIndex = 0;
    m_lFacePlaneSurface_Count = 0;
    m_lFaceCylinderSurface_StartIndex = 0;
    m_lFaceCylinderSurface_Count = 0;
    m_lFaceConeSurface_StartIndex = 0;
    m_lFaceConeSurface_Count = 0;
    m_lFaceTorusSurface_StartIndex = 0;
    m_lFaceTorusSurface_Count = 0;

    // arrays used to count totalLineCount and totalVertexCount
    m_iProcessedEdge_1stEdgeuses.clear();
    m_bProcessedVertices.clear();

} // end UsdBrepArraySpans::ClearStartsAndCounts

/*********************************************************************************************************************
 PURPOSE: Clear UsdBrepArraySpans count objects

 NOTES: leaves Brep topology object and topology object geometry
        StartIndex values untouched
 ********************************************************************************************************************/
void UsdBrepArraySpans::ClearCounts()
{
    // clear all count values
    m_lRegionCount = 0;
    m_lShellCount = 0;
    m_lShellVertexCount = 0;
    m_lFaceuseCount = 0;
    m_lFaceCount = 0;
    m_lLoopCount = 0;
    m_lLoopVertexCount = 0;
    m_lEdgeuseCount = 0;
    m_lEdgeCount = 0;
    m_lWireEdgeCount = 0;
    m_lVertexCount = 0;

    m_lVertexPointPosition_Count = 0;
    m_lShellPointPosition_Count = 0;

    m_lEdgeBSplineCurve3d_Count = 0;
    m_lEdgeBSplineCurve3d_ControlVerticesCount = 0;
    m_lEdgeBSplineCurve3d_KnotCount = 0;

    m_lEdgeCircleCurve3d_Count = 0;
    m_lEdgeLineCurve3d_Count = 0;
    m_lEdgeEllipseCurve3d_Count = 0;

    m_lWireEdgeBSplineCurve3d_Count = 0;
    m_lWireEdgeBSplineCurve3d_ControlVerticesCount = 0;
    m_lWireEdgeBSplineCurve3d_KnotCount = 0;

    m_lWireEdgeCircleCurve3d_Count = 0;
    m_lWireEdgeLineCurve3d_Count = 0;
    m_lWireEdgeEllipseCurve3d_Count = 0;

    m_lEdgeuseBSplineCurve2d_Count = 0;
    m_lEdgeuseBSplineCurve2d_ControlVerticesCount = 0;
    m_lEdgeuseBSplineCurve2d_KnotCount = 0;

    m_lFaceBSplineSurface_Count = 0;
    m_lFaceBSplineSurface_ControlVerticesCount = 0;
    m_lFaceBSplineSurface_UKnotCount = 0;
    m_lFaceBSplineSurface_VKnotCount = 0;

    m_lFaceSphereSurface_Count = 0;
    m_lFacePlaneSurface_Count = 0;
    m_lFaceCylinderSurface_Count = 0;
    m_lFaceConeSurface_Count = 0;
    m_lFaceTorusSurface_Count = 0;

} // end UsdBrepArraySpans::ClearCounts

/*********************************************************************************************************************
 PURPOSE: increment StartIndex values with count values and clear count values

 NOTES: None
 ********************************************************************************************************************/
void UsdBrepArraySpans::IncrementStartIndices // eff: increment StartIndex values with current counts.
    (bool bSaveCounts) // in : false = clear counts,
                       //      true  = don't, default:[false]

{
    m_lBrepIndex++;

    m_lRegionStartIndex += m_lRegionCount;
    m_lShellStartIndex += m_lShellCount;
    m_lShellVertexStartIndex += m_lShellVertexCount;
    m_lFaceuseStartIndex += m_lFaceuseCount;
    m_lFaceStartIndex += m_lFaceCount;
    m_lLoopStartIndex += m_lLoopCount;
    m_lEdgeuseStartIndex += m_lEdgeuseCount;
    m_lEdgeStartIndex += m_lEdgeCount;
    m_lWireEdgeStartIndex += m_lWireEdgeCount;
    m_lVertexStartIndex += m_lVertexCount;

    m_lVertexPointPosition_StartIndex += m_lVertexPointPosition_Count;
    m_lShellPointPosition_StartIndex += m_lShellPointPosition_Count;

    m_lEdgeBSplineCurve3d_StartIndex += m_lEdgeBSplineCurve3d_Count;
    m_lEdgeBSplineCurve3d_ControlVerticesStartIndex += m_lEdgeBSplineCurve3d_ControlVerticesCount;
    m_lEdgeBSplineCurve3d_KnotStartIndex += m_lEdgeBSplineCurve3d_KnotCount;

    m_lEdgeCircleCurve3d_StartIndex += m_lEdgeCircleCurve3d_Count;
    m_lEdgeLineCurve3d_StartIndex += m_lEdgeLineCurve3d_Count;
    m_lEdgeEllipseCurve3d_StartIndex += m_lEdgeEllipseCurve3d_Count;

    m_lWireEdgeBSplineCurve3d_StartIndex += m_lWireEdgeBSplineCurve3d_Count;
    m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex += m_lWireEdgeBSplineCurve3d_ControlVerticesCount;
    m_lWireEdgeBSplineCurve3d_KnotStartIndex += m_lWireEdgeBSplineCurve3d_KnotCount;

    m_lWireEdgeCircleCurve3d_StartIndex += m_lWireEdgeCircleCurve3d_Count;
    m_lWireEdgeLineCurve3d_StartIndex += m_lWireEdgeLineCurve3d_Count;
    m_lWireEdgeEllipseCurve3d_StartIndex += m_lWireEdgeEllipseCurve3d_Count;

    m_lEdgeuseBSplineCurve2d_StartIndex += m_lEdgeuseBSplineCurve2d_Count;
    m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex += m_lEdgeuseBSplineCurve2d_ControlVerticesCount;
    m_lEdgeuseBSplineCurve2d_KnotStartIndex += m_lEdgeuseBSplineCurve2d_KnotCount;

    m_lFaceBSplineSurface_StartIndex += m_lFaceBSplineSurface_Count;
    m_lFaceBSplineSurface_ControlVerticesStartIndex += m_lFaceBSplineSurface_ControlVerticesCount;
    m_lFaceBSplineSurface_UKnotStartIndex += m_lFaceBSplineSurface_UKnotCount;
    m_lFaceBSplineSurface_VKnotStartIndex += m_lFaceBSplineSurface_VKnotCount;

    m_lFaceSphereSurface_StartIndex += m_lFaceSphereSurface_Count;
    m_lFacePlaneSurface_StartIndex += m_lFacePlaneSurface_Count;
    m_lFaceCylinderSurface_StartIndex += m_lFaceCylinderSurface_Count;
    m_lFaceConeSurface_StartIndex += m_lFaceConeSurface_Count;
    m_lFaceTorusSurface_StartIndex += m_lFaceTorusSurface_Count;

    // when asked - clear counts
    if (bSaveCounts == false)
    {
        ClearCounts();
    }

} // end UsdBrepArraySpans::IncrementStartIndices

/*********************************************************************************************************************
 PURPOSE: Find StartIndices for TgtBrep index

 NOTES:
 ********************************************************************************************************************/
bool UsdBrepArraySpans::SetStartsForNextBrepAdd(const UsdBrepArrayData& rArrays) // in : UsdBrepArrayData to parse
{
    // next Brep index
    m_lBrepIndex = rArrays.TotalBrepCount();

    // pBrepData_ii topology local counts - set to 0 for each Brep and then added up as Brep block is parsed
    m_lRegionCount = 0; //       Brep_iilocal region      count
    m_lShellCount = 0; //        Brep_iilocal shell       count = FaceuseShellCount + ShellVertexCount
    m_lShellVertexCount = 0; //  Brep_iilocal shellVertex count
    m_lFaceuseCount = 0; //      Brep_iilocal faceuse     count
    m_lFaceCount = 0; //         Brep_iilocal face        count
    m_lLoopCount = 0; //         Brep_iilocal loop        count
    m_lLoopVertexCount = 0; //   Brep_iilocal loop        count
    m_lEdgeuseCount = 0; //      Brep_iilocal edgeuse     count
    m_lEdgeCount = 0; //         Brep_iilocal edge        count
    m_lWireEdgeCount = 0; //     Brep_iilocal wireEdge    count
    m_lVertexCount = 0; //       Brep_iilocal vertex      count

    // Brep_ii topology global start indices - accumulated as sequence of Breps get parsed
    m_lRegionStartIndex = rArrays.TotalRegionCount(); //           Brep_ii global region      startIndex
    m_lShellStartIndex = rArrays.TotalShellCount(); //             Brep_ii global shell       startIndex
    m_lShellVertexStartIndex = rArrays.TotalShellVertexCount(); // Brep_ii global shellVertex startIndex
    m_lFaceuseStartIndex = rArrays.TotalFaceuseCount(); //         Brep_ii global faceuse     startIndex
    m_lFaceStartIndex = rArrays.TotalFaceCount(); //               Brep_ii global face        startIndex
    m_lLoopStartIndex = rArrays.TotalLoopCount(); //               Brep_ii global loop        startIndex
    m_lEdgeuseStartIndex = rArrays.TotalEdgeuseCount(); //         Brep_ii global edgeuse     startIndex
    m_lEdgeStartIndex = rArrays.TotalEdgeCount(); //               Brep_ii global edge        startIndex
    m_lWireEdgeStartIndex = rArrays.TotalWireEdgeCount(); //       Brep_ii global wireEdge    startIndex
    m_lVertexStartIndex = rArrays.TotalVertexCount(); //           Brep_ii global vertex      startIndex

    // Brep_ii shape counts - set to 0 for each Brep and then added up as Brep block is parsed
    m_lVertexPointPosition_Count = 0;
    m_lShellPointPosition_Count = 0;

    m_lEdgeBSplineCurve3d_Count = 0;
    m_lEdgeBSplineCurve3d_ControlVerticesCount = 0;
    m_lEdgeBSplineCurve3d_KnotCount = 0;

    m_lEdgeCircleCurve3d_Count = 0;
    m_lEdgeLineCurve3d_Count = 0;

    m_lWireEdgeBSplineCurve3d_Count = 0;
    m_lWireEdgeBSplineCurve3d_ControlVerticesCount = 0;
    m_lWireEdgeBSplineCurve3d_KnotCount = 0;

    m_lWireEdgeCircleCurve3d_Count = 0;
    m_lWireEdgeLineCurve3d_Count = 0;

    m_lEdgeuseBSplineCurve2d_Count = 0;
    m_lEdgeuseBSplineCurve2d_ControlVerticesCount = 0;
    m_lEdgeuseBSplineCurve2d_KnotCount = 0;

    m_lFaceBSplineSurface_Count = 0;
    m_lFaceBSplineSurface_ControlVerticesCount = 0;
    m_lFaceBSplineSurface_UKnotCount = 0;
    m_lFaceBSplineSurface_VKnotCount = 0;

    // Brep_pp shaple global start indices - accumulated as sequence of Breps get parsed
    m_lVertexPointPosition_StartIndex = rArrays.TotalVertexPositionCount();
    m_lShellPointPosition_StartIndex = rArrays.TotalShellVertexPositionCount();

    m_lEdgeBSplineCurve3d_StartIndex = rArrays.TotalEdgeCurveCount();
    m_lEdgeBSplineCurve3d_ControlVerticesStartIndex = rArrays.TotalEdgeControlVerticesCount();
    m_lEdgeBSplineCurve3d_KnotStartIndex = rArrays.TotalEdgeKnotCount();

    m_lEdgeCircleCurve3d_StartIndex = static_cast<uint32_t>(rArrays.m_sEdge_CurveCircle_RadiusArray.size());
    m_lEdgeCircleCurve3d_Count = 0;
    m_lEdgeLineCurve3d_StartIndex = static_cast<uint32_t>(rArrays.m_sEdge_CurveLine_OriginArray.size());
    m_lEdgeLineCurve3d_Count = 0;
    m_lEdgeEllipseCurve3d_StartIndex = static_cast<uint32_t>(rArrays.m_sEdge_CurveEllipse_XRadiusArray.size());
    m_lEdgeEllipseCurve3d_Count = 0;

    m_lWireEdgeBSplineCurve3d_StartIndex = rArrays.TotalWireEdgeCurveCount();
    m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex = rArrays.TotalWireEdgeControlVerticesCount();
    m_lWireEdgeBSplineCurve3d_KnotStartIndex = rArrays.TotalWireEdgeKnotCount();

    m_lWireEdgeCircleCurve3d_StartIndex = static_cast<uint32_t>(rArrays.m_sWireEdge_CurveCircle_RadiusArray.size());
    m_lWireEdgeCircleCurve3d_Count = 0;
    m_lWireEdgeLineCurve3d_StartIndex = static_cast<uint32_t>(rArrays.m_sWireEdge_CurveLine_OriginArray.size());
    m_lWireEdgeLineCurve3d_Count = 0;
    m_lWireEdgeEllipseCurve3d_StartIndex = static_cast<uint32_t>(rArrays.m_sWireEdge_CurveEllipse_XRadiusArray.size());
    m_lWireEdgeEllipseCurve3d_Count = 0;

    m_lEdgeuseBSplineCurve2d_StartIndex = rArrays.TotalEdgeuseCurveCount();
    m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex = rArrays.TotalEdgeuseControlVerticesCount();
    m_lEdgeuseBSplineCurve2d_KnotStartIndex = rArrays.TotalEdgeuseKnotCount();

    m_lFaceBSplineSurface_StartIndex = rArrays.TotalFaceSurfaceCount();
    m_lFaceBSplineSurface_ControlVerticesStartIndex = rArrays.TotalFaceControlVerticesCount();
    m_lFaceBSplineSurface_UKnotStartIndex = rArrays.TotalFaceKnot_UCount();
    m_lFaceBSplineSurface_VKnotStartIndex = rArrays.TotalFaceKnot_VCount();

    m_lFaceSphereSurface_StartIndex = static_cast<uint32_t>(rArrays.m_sFace_SurfaceSphere_RadiusArray.size());
    m_lFaceSphereSurface_Count = 0;
    m_lFacePlaneSurface_StartIndex = static_cast<uint32_t>(rArrays.m_sFace_SurfacePlane_OriginArray.size());
    m_lFacePlaneSurface_Count = 0;
    m_lFaceCylinderSurface_StartIndex = static_cast<uint32_t>(rArrays.m_sFace_SurfaceCylinder_RadiusArray.size());
    m_lFaceCylinderSurface_Count = 0;
    m_lFaceConeSurface_StartIndex = static_cast<uint32_t>(rArrays.m_sFace_SurfaceCone_RadiusArray.size());
    m_lFaceConeSurface_Count = 0;
    m_lFaceTorusSurface_StartIndex = static_cast<uint32_t>(rArrays.m_sFace_SurfaceTorus_MajorRadiusArray.size());
    m_lFaceTorusSurface_Count = 0;

    return true;

} // end UsdBrepArraySpans::SetStartsForNextBrepAdd

/*********************************************************************************************************************
 PURPOSE: Find start index values and optionally the count values
          for TgtBrep index

 NOTES: Walks array data from the start for every Brep before
        the target Brep[iBrepIndex] adding up Brep counts to
        accumulating start index values for iBrepIndex leaving
        the Brep_ii count values at 0.

        When bSetCounts is true walks the arrays one more time to
             also set the Brep_ii count values.

        Save some time and leave the Brep_ii count values at 0 when
        those values are not needed for the next BrepArray call.

Errors: returns false and makes no changes when iBrepIndex is out of range
        else returns true after updating starts and counts as asked
 ********************************************************************************************************************/
bool UsdBrepArraySpans::SetStartsAndCountsForBrepIndex(
    const UsdBrepArrayData& rArrays, // eff: set Brep_ii start
                                     // index and count values by
                                     // walking
                                     //      array data for every
                                     //      Brep up to Brep_ii
    uint32_t iBrepIndex, // in : Tgt Brep_ii index
    bool bSetStarts, // in : true = set Brep_ii start Indices
                     // by walking rArrays data for every
                     // Brep prior to Brep_ii
                     //      false= use StartIndices as is
                     //      assuming they are set for
                     //      Brep_ii. default:[true]
    bool bSetCounts
) // in : true = set Brep_ii counts,
  //      false= set Brep_ii icounts=0,
  //      ready for next move Brep call.
  //      default:[false]
{
    // locals
    uint32_t lBrepCount = rArrays.TotalBrepCount();
    uint32_t lEndIter = bSetCounts ? iBrepIndex + 1 : iBrepIndex;
    uint32_t lStartIter = 0;

    // check input
    if (iBrepIndex >= lBrepCount)
    {
        return false;
    }

    // init starts and counts
    if (bSetStarts == true || iBrepIndex != m_lBrepIndex)
    { // when iBrepIndex is not current, find Start and Count values for iBrepIndex
        lStartIter = 0;
        ClearStartsAndCounts();
    }
    else // iBrepIndex is current. Use current Starts and only build counts for iBrepIndex
    {
        lStartIter = m_lBrepIndex;
        ClearCounts();
    }

    // no work - m_lBrepIndex == target index, and not asked to rebuild starts or counts
    if (bSetStarts == false && bSetCounts == false && iBrepIndex == m_lBrepIndex)
    {
        return true;
    }


    // for every rUsdBrepArray Brep item - count topology objects to acccumulate start index values
    for (size_t ii = lStartIter; ii < lEndIter; ii++)
    {
        // Regions
        m_lRegionCount = GetArraySubscript(rArrays.m_sBrepRegionCountArray, ii);
        for (uint32_t iGlobal = m_lRegionStartIndex, iLocal = 0; iLocal < m_lRegionCount; iGlobal++, iLocal++)
        {
            // accumulate topology counts
            m_lShellCount += GetArraySubscript(rArrays.m_sRegionShellCountArray, iGlobal);
        } // end iter every Region

        // Shells
        for (uint32_t iGlobal = m_lShellStartIndex, iLocal = 0; iLocal < m_lShellCount; iGlobal++, iLocal++)
        {
            if (iGlobal >= rArrays.m_sShellFaceuseCountArray.size() || iGlobal >= rArrays.m_sShellWireEdgeCountArray.size() ||
                iGlobal >= rArrays.m_sShellPointTypeArray.size())
            {
                USDBREP_ERROR("Malformed BrepArray: truncated shell topology data");
                return false;
            }

            // accumulate topology and shape counts
            m_lFaceuseCount += GetArraySubscript(rArrays.m_sShellFaceuseCountArray, iGlobal);
            m_lWireEdgeCount += GetArraySubscript(rArrays.m_sShellWireEdgeCountArray, iGlobal);
            if (rArrays.IsBrepPointShell(iGlobal))
            {
                const uint32_t iPointPosition = m_lShellPointPosition_StartIndex + m_lShellPointPosition_Count;
                if (iPointPosition >= rArrays.m_sShell_PointPositionArray.size())
                {
                    USDBREP_ERROR("Malformed BrepArray: truncated shell-point position data");
                    return false;
                }

                m_lShellVertexCount++;
                m_lShellPointPosition_Count++;
            }
        } // end iter every Shell

        // Shell::ShellPoint3d for shells satisfying IsBrepPointShell(). BrepPointAPI is the only supported point
        // type in the first release; more may be added later.

        // Faceuses

        // FaceCount = FaceuseCount / 2
        m_lFaceCount = m_lFaceuseCount / 2;

        // Faces
        for (uint32_t iGlobal = m_lFaceStartIndex, iLocal = 0; iLocal < m_lFaceCount; iGlobal++, iLocal++)
        {
            // accumulate topology and shape counts
            m_lLoopCount += GetArraySubscript(rArrays.m_sFaceLoopCountArray, iGlobal);

            // accumulate shape counts
            if (rArrays.m_sFaceSurfaceTypeArray.size() > 0 && rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
            {
                // NURBS surface metadata is packed by NURBS-face appearance, not by global face
                // index. Use the NURBS-specific running index for control-point and knot counts.
                uint32_t iNurbIdx = m_lFaceBSplineSurface_StartIndex + m_lFaceBSplineSurface_Count;
                if (iNurbIdx >= rArrays.m_sFace_SurfaceNurb_UVertexCountArray.size() ||
                    iNurbIdx >= rArrays.m_sFace_SurfaceNurb_VVertexCountArray.size() || iNurbIdx >= rArrays.m_sFace_SurfaceNurb_UOrderArray.size() ||
                    iNurbIdx >= rArrays.m_sFace_SurfaceNurb_VOrderArray.size())
                {
                    USDBREP_ERROR("Malformed BrepArray: truncated NURBS surface metadata");
                    return false;
                }

                m_lFaceBSplineSurface_Count++;
                m_lFaceBSplineSurface_ControlVerticesCount += rArrays.m_sFace_SurfaceNurb_UVertexCountArray[iNurbIdx] *
                                                              rArrays.m_sFace_SurfaceNurb_VVertexCountArray[iNurbIdx];
                m_lFaceBSplineSurface_UKnotCount += rArrays.m_sFace_SurfaceNurb_UVertexCountArray[iNurbIdx] +
                                                    rArrays.m_sFace_SurfaceNurb_UOrderArray[iNurbIdx];
                m_lFaceBSplineSurface_VKnotCount += rArrays.m_sFace_SurfaceNurb_VVertexCountArray[iNurbIdx] +
                                                    rArrays.m_sFace_SurfaceNurb_VOrderArray[iNurbIdx];
            }
            else if (rArrays.m_sFaceSurfaceTypeArray.size() > 0 && rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
            {
                m_lFaceSphereSurface_Count++;
            }
            else if (rArrays.m_sFaceSurfaceTypeArray.size() > 0 && rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
            {
                m_lFacePlaneSurface_Count++;
            }
            else if (rArrays.m_sFaceSurfaceTypeArray.size() > 0 && rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
            {
                m_lFaceCylinderSurface_Count++;
            }
            else if (rArrays.m_sFaceSurfaceTypeArray.size() > 0 && rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
            {
                m_lFaceConeSurface_Count++;
            }
            else if (rArrays.m_sFaceSurfaceTypeArray.size() > 0 && rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
            {
                m_lFaceTorusSurface_Count++;
            }

        } // end iter every Face

        // Loops
        for (uint32_t iGlobal = m_lLoopStartIndex, iLocal = 0; iLocal < m_lLoopCount; iGlobal++, iLocal++)
        {
            // accumulate topology and shape counts
            m_lEdgeuseCount += GetArraySubscript(rArrays.m_sLoopEdgeuseCountArray, iGlobal);
            m_lLoopVertexCount += ((!rArrays.m_sLoopEdgeuseCountArray.empty()) && (rArrays.m_sLoopEdgeuseCountArray[iGlobal] == 0)) ?
                                      1U :
                                      0U; // this Loop is a vertexLoop,
                                          // no Edgeuses in it
            m_lVertexCount += ((!rArrays.m_sLoopEdgeuseCountArray.empty()) && (rArrays.m_sLoopEdgeuseCountArray[iGlobal] == 0)) ? 1U :
                                                                                                                                  0U; // this Loop is
                                                                                                                                      // a vertexLoop
        } // end iter every Loop

        // local - edgeCount aide
        VtArray<bool> bEdgeReference(m_lEdgeuseCount, false); // boolean array, sized:[MaxEdgeCount = EdgeuseCount]

        // Edgeuses
        for (uint32_t iGlobal = m_lEdgeuseStartIndex, iLocal = 0; iLocal < m_lEdgeuseCount; iGlobal++, iLocal++)
        {
            // Parse flag to edgeIndex and orientation
            uint32_t iGlobalEdgeuseEdgeIndex = GetArraySubscript(rArrays.m_sEdgeuseEdgeIndexArray, iGlobal);
            uint32_t iLocalEdgeuseEdgeIndex = rArrays.MapGlobalToLocal(iGlobalEdgeuseEdgeIndex, m_lEdgeStartIndex);

            // accumulate topology data counts
            if (bEdgeReference[iLocalEdgeuseEdgeIndex] == false)
            {
                bEdgeReference[iLocalEdgeuseEdgeIndex] = true;
                m_lEdgeCount++;
            }

            // accumulate shape data counts
            // note: this assumes all Edgeuse->UVTrimCurves are BSplineCurves - if this changes the code here will have
            // to branch on the allowed UVTrimCurve types.
            m_lEdgeuseBSplineCurve2d_Count++;
            size_t lSize = rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.size();
            m_lEdgeuseBSplineCurve2d_ControlVerticesCount += lSize > 0 ? rArrays.m_sEdgeuse_CurveNurb_VertexCountArray[iGlobal] : 0;
            m_lEdgeuseBSplineCurve2d_KnotCount += lSize > 0 ? rArrays.m_sEdgeuse_CurveNurb_VertexCountArray[iGlobal] +
                                                                  rArrays.m_sEdgeuse_CurveNurb_OrderArray[iGlobal] :
                                                              0;
        } // end iter every Edgeuse

        // local - vertexCount aide - waited until LoopCount, EdgeCount, and WireEdgeCount were computed to calc a max
        // bVertexReference size
        // boolean array, sized:[MaxVertexCount = LoopVertexCount + 2*EdgeCount + 2*WireEdgeCount]
        VtArray<bool> bVertexReference(m_lLoopCount + 2 * m_lEdgeCount + 2 * m_lWireEdgeCount, false);

        // Count LoopVertex vertices -
        // note: waited until bVertexReference was sized -
        //       this is the only bit of the data parse which runs double (itereate twice over the LoopStartIndex to
        //       LoopCount range). perhaps there is a linear way to count vertices without having to delay the
        //       shellVertex vertex counts.
        for (uint32_t iGlobal = m_lLoopStartIndex, iLocal = 0; iLocal < m_lLoopCount; iGlobal++, iLocal++)
        {
            // accumulate Vertex counts from LoopVertices
            if (rArrays.m_sLoopEdgeuseCountArray[iGlobal] == 0 // this Loop is a vertexLoop
                && bVertexReference[rArrays.m_sLoopVertexIndexArray[iGlobal]] == false) // and its loop vertex hasn't
                                                                                        // been used yet
            {
                bVertexReference[rArrays.m_sLoopVertexIndexArray[iGlobal]] = true;
            }
        } // end iter every Loop

        // Edges
        for (uint32_t iGlobal = m_lEdgeStartIndex, iLocal = 0; iLocal < m_lEdgeCount; iGlobal++, iLocal++)
        {
            // Map global to local vertex indices
            uint32_t iStartVertexLocalIndex = rArrays.MapGlobalToLocal(
                GetArray2Subscript(rArrays.m_sEdgeVertexIndicesArray, iGlobal, 0),
                m_lVertexStartIndex
            );

            uint32_t iEndVertexLocalIndex = rArrays.MapGlobalToLocal(
                GetArray2Subscript(rArrays.m_sEdgeVertexIndicesArray, iGlobal, 1),
                m_lVertexStartIndex
            );

            // accumulate topology data counts
            if (bVertexReference[iStartVertexLocalIndex] == false)
            {
                bVertexReference[iStartVertexLocalIndex] = true;
                m_lVertexCount++;
            }
            if (bVertexReference[iEndVertexLocalIndex] == false)
            {
                bVertexReference[iEndVertexLocalIndex] = true;
                m_lVertexCount++;
            }

            // accumulate shape data counts
            if (rArrays.m_sEdgeCurveTypeArray.size() > 0 && rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dNurbAPI)
            {
                // Use the NURB-specific running index for Order/VertexCount arrays,
                // which only have entries for NURB edges (not circle/line/ellipse edges)
                uint32_t iNurbIdx = m_lEdgeBSplineCurve3d_StartIndex + m_lEdgeBSplineCurve3d_Count;
                m_lEdgeBSplineCurve3d_Count++;
                assert(iNurbIdx < rArrays.m_sEdge_CurveNurb_VertexCountArray.size());
                if (iNurbIdx < rArrays.m_sEdge_CurveNurb_VertexCountArray.size())
                {
                    m_lEdgeBSplineCurve3d_ControlVerticesCount += rArrays.m_sEdge_CurveNurb_VertexCountArray[iNurbIdx];
                    m_lEdgeBSplineCurve3d_KnotCount += rArrays.m_sEdge_CurveNurb_VertexCountArray[iNurbIdx] +
                                                       rArrays.m_sEdge_CurveNurb_OrderArray[iNurbIdx];
                }
            }
            else if (rArrays.m_sEdgeCurveTypeArray.size() > 0 && rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dCircleAPI)
            {
                m_lEdgeCircleCurve3d_Count++;
            }
            else if (rArrays.m_sEdgeCurveTypeArray.size() > 0 && rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dLineAPI)
            {
                m_lEdgeLineCurve3d_Count++;
            }
            else if (rArrays.m_sEdgeCurveTypeArray.size() > 0 && rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
            {
                m_lEdgeEllipseCurve3d_Count++;
            }
        } // end iter every Edge

        // WireEdges
        for (uint32_t iGlobal = m_lWireEdgeStartIndex, iLocal = 0; iLocal < m_lWireEdgeCount; iGlobal++, iLocal++)
        {
            // Map global to local vertex indices
            uint32_t iStartVertexLocalIndex = rArrays.MapGlobalToLocal(
                GetArray2Subscript(rArrays.m_sWireEdgeVertexIndicesArray, iGlobal, 0),
                m_lVertexStartIndex
            );

            uint32_t iEndVertexLocalIndex = rArrays.MapGlobalToLocal(
                GetArray2Subscript(rArrays.m_sWireEdgeVertexIndicesArray, iGlobal, 1),
                m_lVertexStartIndex
            );

            // accumulate topology data counts
            if (bVertexReference[iStartVertexLocalIndex] == false)
            {
                bVertexReference[iStartVertexLocalIndex] = true;
                m_lVertexCount++;
            }
            if (bVertexReference[iEndVertexLocalIndex] == false)
            {
                bVertexReference[iEndVertexLocalIndex] = true;
                m_lVertexCount++;
            }

            // accumulate shape data counts
            if (rArrays.m_sWireEdgeCurveTypeArray.size() > 0 && rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dNurbAPI)
            {
                uint32_t iNurbIdx = m_lWireEdgeBSplineCurve3d_StartIndex + m_lWireEdgeBSplineCurve3d_Count;
                m_lWireEdgeBSplineCurve3d_Count++;
                assert(iNurbIdx < rArrays.m_sWireEdge_CurveNurb_VertexCountArray.size());
                if (iNurbIdx < rArrays.m_sWireEdge_CurveNurb_VertexCountArray.size())
                {
                    m_lWireEdgeBSplineCurve3d_ControlVerticesCount += rArrays.m_sWireEdge_CurveNurb_VertexCountArray[iNurbIdx];
                    m_lWireEdgeBSplineCurve3d_KnotCount += rArrays.m_sWireEdge_CurveNurb_VertexCountArray[iNurbIdx] +
                                                           rArrays.m_sWireEdge_CurveNurb_OrderArray[iNurbIdx];
                }
            }
            else if (rArrays.m_sWireEdgeCurveTypeArray.size() > 0 && rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dCircleAPI)
            {
                m_lWireEdgeCircleCurve3d_Count++;
            }
            else if (rArrays.m_sWireEdgeCurveTypeArray.size() > 0 && rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dLineAPI)
            {
                m_lWireEdgeLineCurve3d_Count++;
            }
            else if (rArrays.m_sWireEdgeCurveTypeArray.size() > 0 && rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
            {
                m_lWireEdgeEllipseCurve3d_Count++;
            }
        } // end iter every WireEdge

        // Vertices
        for (uint32_t iGlobal = m_lVertexStartIndex, iLocal = 0; iLocal < m_lVertexCount; iGlobal++, iLocal++)
        {
            // accumulate shape data counts
            if (rArrays.m_sVertexPointTypeArray.size() > 0 && rArrays.m_sVertexPointTypeArray[iGlobal] == UsdBrepSolidTokens->brepPointAPI)
            {
                m_lVertexPointPosition_Count++;
            }
        } // end iter every Vertex

        // Vertex::Point3ds for types = UsdBrepSolidTokens->brepPointAPI  - for 1st release VertexPoint is the only
        // type, will have more when the MultiVertex is added

        // increment starts and clear counts for next iteration - except on the last iteration when asked for counts
        if ((bSetCounts == false) || (ii < (lEndIter - 1)))
        {
            // increment start indices and clear counts
            IncrementStartIndices(false); // false=clear counts, true=don't,
        }
    } // end iter ii from lStartIter to less than lEndIter

    return true;

} // end UsdBrepArraySpans::SetStartsAndCountsForBrepIndex

} // end namespace UsdBrepData
