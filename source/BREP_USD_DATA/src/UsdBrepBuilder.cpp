// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepBuilder.h"

namespace UsdBrepData
{

UsdBrepBuilder::UsdBrepBuilder(UsdBrepArrayData& rArrays, UsdBrepArraySpans& rSpans) : m_pArrays(&rArrays), m_pSpans(&rSpans)
{
}

UsdBrepArrayData& UsdBrepBuilder::Arrays()
{
    return *m_pArrays;
}

const UsdBrepArrayData& UsdBrepBuilder::Arrays() const
{
    return *m_pArrays;
}

UsdBrepArraySpans& UsdBrepBuilder::Spans()
{
    return *m_pSpans;
}

const UsdBrepArraySpans& UsdBrepBuilder::Spans() const
{
    return *m_pSpans;
}

void UsdBrepBuilder::Reset()
{
    Arrays().ReSet();
    Spans().ClearStartsAndCounts();
}

void UsdBrepBuilder::ReserveRegions(uint32_t uiRegionCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sRegionShellCountArray, uiRegionCount);
    ReserveArray(rArrays.m_sRegionTypeArray, uiRegionCount);
}

void UsdBrepBuilder::ReserveShells(uint32_t uiShellCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sShellFaceuseCountArray, uiShellCount);
    ReserveArray(rArrays.m_sShellWireEdgeCountArray, uiShellCount);
    ReserveArray(rArrays.m_sShellPointTypeArray, uiShellCount);
}

void UsdBrepBuilder::ReserveFaceuses(uint32_t uiFaceuseCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sFaceuseFaceIndexArray, uiFaceuseCount);
    ReserveArray(rArrays.m_sFaceuseOrientationTypeArray, uiFaceuseCount);
}

void UsdBrepBuilder::ReserveFaces(uint32_t uiFaceCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sFaceLoopCountArray, uiFaceCount);
    ReserveArray(rArrays.m_sFaceSurfaceTypeArray, uiFaceCount);
    ReserveArray(rArrays.m_sFaceTrimTypeArray, uiFaceCount);
    ReserveArray(rArrays.m_sFaceRangeArray, static_cast<size_t>(uiFaceCount) * 2);
}

void UsdBrepBuilder::ReserveLoops(uint32_t uiLoopCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sLoopEdgeuseCountArray, uiLoopCount);
    ReserveArray(rArrays.m_sLoopVertexIndexArray, uiLoopCount);
}

void UsdBrepBuilder::ReserveEdgeuses(uint32_t uiEdgeuseCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sEdgeuseEdgeIndexArray, uiEdgeuseCount);
    ReserveArray(rArrays.m_sEdgeuseOrientationTypeArray, uiEdgeuseCount);
    ReserveArray(rArrays.m_sEdgeuseNextRadialEUIndexArray, uiEdgeuseCount);
    ReserveArray(rArrays.m_sEdgeuseThisRadialEntryTypeArray, uiEdgeuseCount);
    // One placeholder UV-curve record is appended per edgeuse.
    ReserveArray(rArrays.m_sEdgeuse_CurveNurb_VertexCountArray, uiEdgeuseCount);
    ReserveArray(rArrays.m_sEdgeuse_CurveNurb_OrderArray, uiEdgeuseCount);
}

void UsdBrepBuilder::ReserveEdges(uint32_t uiEdgeCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sEdgeCurveTypeArray, uiEdgeCount);
    ReserveArray(rArrays.m_sEdgeRangeArray, static_cast<size_t>(uiEdgeCount) * 2);
    ReserveArray(rArrays.m_sEdgeVertexIndicesArray, uiEdgeCount);
}

void UsdBrepBuilder::ReserveWireEdges(uint32_t uiWireEdgeCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sWireEdgeCurveTypeArray, uiWireEdgeCount);
    ReserveArray(rArrays.m_sWireEdgeRangeArray, static_cast<size_t>(uiWireEdgeCount) * 2);
    ReserveArray(rArrays.m_sWireEdgeVertexIndicesArray, uiWireEdgeCount);
}

void UsdBrepBuilder::ReserveVertices(uint32_t uiVertexCount, uint32_t uiVertexPointCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    ReserveArray(rArrays.m_sVertexPointTypeArray, uiVertexCount);
    ReserveArray(rArrays.m_sVertex_PointPositionArray, uiVertexPointCount);
}

bool UsdBrepBuilder::BeginBrep(double dXSectTol3d, const pxr::GfRange3d& rBrepExtent3d, uint32_t uiRegionCount)
{
    UsdBrepArrayData& rArrays = Arrays();
    UsdBrepArraySpans& rSpans = Spans();

    if (rArrays.TotalBrepCount() > 0)
    {
        if (!rSpans.SetStartsForNextBrepAdd(rArrays))
        {
            return false;
        }
    }
    else
    {
        rSpans.ClearStartsAndCounts();
    }

    rSpans.ClearCounts();
    rSpans.m_lBrepIndex = rArrays.TotalBrepCount();
    rSpans.m_lRegionCount = uiRegionCount;

    rArrays.m_sBrepXSectTol3dArray.emplace_back(dXSectTol3d);
    rArrays.m_sBrepExtentArray.emplace_back(rBrepExtent3d.GetMin());
    rArrays.m_sBrepExtentArray.emplace_back(rBrepExtent3d.GetMax());
    rArrays.m_sBrepRegionCountArray.emplace_back(uiRegionCount);

    pxr::GfVec3f sBrepMinF(
        static_cast<float>(rBrepExtent3d.GetMin()[0]),
        static_cast<float>(rBrepExtent3d.GetMin()[1]),
        static_cast<float>(rBrepExtent3d.GetMin()[2])
    );
    pxr::GfVec3f sBrepMaxF(
        static_cast<float>(rBrepExtent3d.GetMax()[0]),
        static_cast<float>(rBrepExtent3d.GetMax()[1]),
        static_cast<float>(rBrepExtent3d.GetMax()[2])
    );
    if (rArrays.m_sBrepArray_BBox.IsEmpty())
    {
        rArrays.m_sBrepArray_BBox = pxr::GfRange3f(sBrepMinF, sBrepMaxF);
    }
    else
    {
        rArrays.m_sBrepArray_BBox.UnionWith(pxr::GfRange3f(sBrepMinF, sBrepMaxF));
    }

    return true;
}

void UsdBrepBuilder::FinishBrep(bool bKeepCounts)
{
    Spans().IncrementStartIndices(bKeepCounts);
}

void UsdBrepBuilder::AppendRegion(uint32_t uiShellCount, const pxr::TfToken& rRegionType)
{
    UsdBrepArrayData& rArrays = Arrays();
    UsdBrepArraySpans& rSpans = Spans();
    rArrays.m_sRegionShellCountArray.emplace_back(uiShellCount);
    rArrays.m_sRegionTypeArray.emplace_back(rRegionType);
    rSpans.m_lShellCount += uiShellCount;
}

void UsdBrepBuilder::AppendShell(uint32_t uiFaceuseCount, uint32_t uiWireEdgeCount, const pxr::TfToken& rShellPointType)
{
    UsdBrepArrayData& rArrays = Arrays();
    UsdBrepArraySpans& rSpans = Spans();
    rArrays.m_sShellFaceuseCountArray.emplace_back(uiFaceuseCount);
    rArrays.m_sShellWireEdgeCountArray.emplace_back(uiWireEdgeCount);
    rArrays.m_sShellPointTypeArray.emplace_back(rShellPointType);
    rSpans.m_lFaceuseCount += uiFaceuseCount;
    rSpans.m_lWireEdgeCount += uiWireEdgeCount;
}

void UsdBrepBuilder::AppendShellPoint(const pxr::GfVec3d& rPoint)
{
    UsdBrepArrayData& rArrays = Arrays();
    UsdBrepArraySpans& rSpans = Spans();
    rArrays.m_sShell_PointPositionArray.emplace_back(rPoint);
    ++rSpans.m_lShellVertexCount;
    ++rSpans.m_lShellPointPosition_Count;
}

void UsdBrepBuilder::AppendFaceuse(uint32_t uiGlobalFaceIndex, const pxr::TfToken& rOrientationType)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFaceuseFaceIndexArray.emplace_back(uiGlobalFaceIndex);
    rArrays.m_sFaceuseOrientationTypeArray.emplace_back(rOrientationType);
}

void UsdBrepBuilder::AppendFace(uint32_t uiLoopCount, const pxr::TfToken& rSurfaceType, const pxr::TfToken& rTrimType, const pxr::GfRange2d& rUVDomain)
{
    UsdBrepArrayData& rArrays = Arrays();
    UsdBrepArraySpans& rSpans = Spans();
    rArrays.m_sFaceLoopCountArray.emplace_back(uiLoopCount);
    rArrays.m_sFaceSurfaceTypeArray.emplace_back(rSurfaceType);
    rArrays.m_sFaceTrimTypeArray.emplace_back(rTrimType);
    rArrays.m_sFaceRangeArray.emplace_back(rUVDomain.GetMin());
    rArrays.m_sFaceRangeArray.emplace_back(rUVDomain.GetMax());
    rSpans.m_lLoopCount += uiLoopCount;
}

void UsdBrepBuilder::AppendLoop(uint32_t uiEdgeuseCount, uint32_t uiGlobalVertexIndex)
{
    UsdBrepArrayData& rArrays = Arrays();
    UsdBrepArraySpans& rSpans = Spans();
    rArrays.m_sLoopEdgeuseCountArray.emplace_back(uiEdgeuseCount);
    rArrays.m_sLoopVertexIndexArray.emplace_back(uiGlobalVertexIndex);
    rSpans.m_lEdgeuseCount += uiEdgeuseCount;
}

void UsdBrepBuilder::AppendEdgeuse(
    uint32_t uiGlobalEdgeIndex,
    const pxr::TfToken& rOrientationType,
    uint32_t uiGlobalNextRadialEUIndex,
    const pxr::TfToken& rThisRadialEntryType
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdgeuseEdgeIndexArray.emplace_back(uiGlobalEdgeIndex);
    rArrays.m_sEdgeuseOrientationTypeArray.emplace_back(rOrientationType);
    rArrays.m_sEdgeuseNextRadialEUIndexArray.emplace_back(uiGlobalNextRadialEUIndex);
    rArrays.m_sEdgeuseThisRadialEntryTypeArray.emplace_back(rThisRadialEntryType);
}

void UsdBrepBuilder::AppendEdge(const pxr::TfToken& rCurveType, const pxr::GfRange1d& rCurveRange, const pxr::GfVec2i& rGlobalVertexIndices)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdgeCurveTypeArray.emplace_back(rCurveType);
    rArrays.m_sEdgeRangeArray.emplace_back(rCurveRange.GetMin());
    rArrays.m_sEdgeRangeArray.emplace_back(rCurveRange.GetMax());
    rArrays.m_sEdgeVertexIndicesArray.emplace_back(rGlobalVertexIndices);
}

void UsdBrepBuilder::AppendWireEdge(const pxr::TfToken& rCurveType, const pxr::GfRange1d& rCurveRange, const pxr::GfVec2i& rGlobalVertexIndices)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sWireEdgeCurveTypeArray.emplace_back(rCurveType);
    rArrays.m_sWireEdgeRangeArray.emplace_back(rCurveRange.GetMin());
    rArrays.m_sWireEdgeRangeArray.emplace_back(rCurveRange.GetMax());
    rArrays.m_sWireEdgeVertexIndicesArray.emplace_back(rGlobalVertexIndices);
}

void UsdBrepBuilder::AppendVertex(const pxr::TfToken& rPointType)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sVertexPointTypeArray.emplace_back(rPointType);
}

void UsdBrepBuilder::AppendVertexPoint(const pxr::GfVec3d& rPoint)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sVertex_PointPositionArray.emplace_back(rPoint);
}

void UsdBrepBuilder::AppendEdgeNurbCurve(
    uint32_t uiVertexCount,
    uint32_t uiOrder,
    const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
    const pxr::VtArray<double>& rWeights,
    const pxr::VtArray<double>& rKnots
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdge_CurveNurb_VertexCountArray.emplace_back(uiVertexCount);
    rArrays.m_sEdge_CurveNurb_OrderArray.emplace_back(uiOrder);
    AppendArray(rArrays.m_sEdge_CurveNurb_ControlVerticesArray, rControlVertices);
    AppendArray(rArrays.m_sEdge_CurveNurb_WeightsArray, rWeights);
    AppendArray(rArrays.m_sEdge_CurveNurb_KnotsArray, rKnots);
}

void UsdBrepBuilder::AppendEdgeCircleCurve(const pxr::GfVec3d& rCenter, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdge_CurveCircle_CenterArray.emplace_back(rCenter);
    rArrays.m_sEdge_CurveCircle_AxisArray.emplace_back(rAxis);
    rArrays.m_sEdge_CurveCircle_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sEdge_CurveCircle_RadiusArray.emplace_back(dRadius);
}

void UsdBrepBuilder::AppendEdgeLineCurve(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rDirection)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdge_CurveLine_OriginArray.emplace_back(rOrigin);
    rArrays.m_sEdge_CurveLine_DirectionArray.emplace_back(rDirection);
}

void UsdBrepBuilder::AppendEdgeEllipseCurve(
    const pxr::GfVec3d& rCenter,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dXRadius,
    double dYRadius
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdge_CurveEllipse_CenterArray.emplace_back(rCenter);
    rArrays.m_sEdge_CurveEllipse_AxisArray.emplace_back(rAxis);
    rArrays.m_sEdge_CurveEllipse_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sEdge_CurveEllipse_XRadiusArray.emplace_back(dXRadius);
    rArrays.m_sEdge_CurveEllipse_YRadiusArray.emplace_back(dYRadius);
}

void UsdBrepBuilder::AppendWireEdgeNurbCurve(
    uint32_t uiVertexCount,
    uint32_t uiOrder,
    const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
    const pxr::VtArray<double>& rWeights,
    const pxr::VtArray<double>& rKnots
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sWireEdge_CurveNurb_VertexCountArray.emplace_back(uiVertexCount);
    rArrays.m_sWireEdge_CurveNurb_OrderArray.emplace_back(uiOrder);
    AppendArray(rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray, rControlVertices);
    AppendArray(rArrays.m_sWireEdge_CurveNurb_WeightsArray, rWeights);
    AppendArray(rArrays.m_sWireEdge_CurveNurb_KnotsArray, rKnots);
}

void UsdBrepBuilder::AppendWireEdgeCircleCurve(
    const pxr::GfVec3d& rCenter,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dRadius
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sWireEdge_CurveCircle_CenterArray.emplace_back(rCenter);
    rArrays.m_sWireEdge_CurveCircle_AxisArray.emplace_back(rAxis);
    rArrays.m_sWireEdge_CurveCircle_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sWireEdge_CurveCircle_RadiusArray.emplace_back(dRadius);
}

void UsdBrepBuilder::AppendWireEdgeLineCurve(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rDirection)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sWireEdge_CurveLine_OriginArray.emplace_back(rOrigin);
    rArrays.m_sWireEdge_CurveLine_DirectionArray.emplace_back(rDirection);
}

void UsdBrepBuilder::AppendWireEdgeEllipseCurve(
    const pxr::GfVec3d& rCenter,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dXRadius,
    double dYRadius
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sWireEdge_CurveEllipse_CenterArray.emplace_back(rCenter);
    rArrays.m_sWireEdge_CurveEllipse_AxisArray.emplace_back(rAxis);
    rArrays.m_sWireEdge_CurveEllipse_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sWireEdge_CurveEllipse_XRadiusArray.emplace_back(dXRadius);
    rArrays.m_sWireEdge_CurveEllipse_YRadiusArray.emplace_back(dYRadius);
}

void UsdBrepBuilder::AppendEdgeuseNurbCurve(
    uint32_t uiVertexCount,
    uint32_t uiOrder,
    const pxr::VtArray<pxr::GfVec2d>& rControlVertices,
    const pxr::VtArray<double>& rWeights,
    const pxr::VtArray<double>& rKnots
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.emplace_back(uiVertexCount);
    rArrays.m_sEdgeuse_CurveNurb_OrderArray.emplace_back(uiOrder);
    AppendArray(rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray, rControlVertices);
    AppendArray(rArrays.m_sEdgeuse_CurveNurb_WeightsArray, rWeights);
    AppendArray(rArrays.m_sEdgeuse_CurveNurb_KnotsArray, rKnots);
}

void UsdBrepBuilder::AppendFaceNurbSurface(
    uint32_t uiUVertexCount,
    uint32_t uiVVertexCount,
    uint32_t uiUOrder,
    uint32_t uiVOrder,
    const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
    const pxr::VtArray<double>& rWeights,
    const pxr::VtArray<double>& rUKnots,
    const pxr::VtArray<double>& rVKnots
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFace_SurfaceNurb_UVertexCountArray.emplace_back(uiUVertexCount);
    rArrays.m_sFace_SurfaceNurb_VVertexCountArray.emplace_back(uiVVertexCount);
    rArrays.m_sFace_SurfaceNurb_UOrderArray.emplace_back(uiUOrder);
    rArrays.m_sFace_SurfaceNurb_VOrderArray.emplace_back(uiVOrder);
    AppendArray(rArrays.m_sFace_SurfaceNurb_ControlVerticesArray, rControlVertices);
    AppendArray(rArrays.m_sFace_SurfaceNurb_WeightsArray, rWeights);
    AppendArray(rArrays.m_sFace_SurfaceNurb_UKnotsArray, rUKnots);
    AppendArray(rArrays.m_sFace_SurfaceNurb_VKnotsArray, rVKnots);
}

void UsdBrepBuilder::AppendFaceSphereSurface(const pxr::GfVec3d& rCenter, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFace_SurfaceSphere_CenterArray.emplace_back(rCenter);
    rArrays.m_sFace_SurfaceSphere_AxisArray.emplace_back(rAxis);
    rArrays.m_sFace_SurfaceSphere_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sFace_SurfaceSphere_RadiusArray.emplace_back(dRadius);
}

void UsdBrepBuilder::AppendFacePlaneSurface(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFace_SurfacePlane_OriginArray.emplace_back(rOrigin);
    rArrays.m_sFace_SurfacePlane_AxisArray.emplace_back(rAxis);
    rArrays.m_sFace_SurfacePlane_RefDirectionArray.emplace_back(rRefDirection);
}

void UsdBrepBuilder::AppendFaceCylinderSurface(
    const pxr::GfVec3d& rOrigin,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dRadius
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFace_SurfaceCylinder_OriginArray.emplace_back(rOrigin);
    rArrays.m_sFace_SurfaceCylinder_AxisArray.emplace_back(rAxis);
    rArrays.m_sFace_SurfaceCylinder_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sFace_SurfaceCylinder_RadiusArray.emplace_back(dRadius);
}

void UsdBrepBuilder::AppendFaceConeSurface(
    const pxr::GfVec3d& rOrigin,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dRadius,
    double dSemiAngle
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFace_SurfaceCone_OriginArray.emplace_back(rOrigin);
    rArrays.m_sFace_SurfaceCone_AxisArray.emplace_back(rAxis);
    rArrays.m_sFace_SurfaceCone_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sFace_SurfaceCone_RadiusArray.emplace_back(dRadius);
    rArrays.m_sFace_SurfaceCone_SemiAngleArray.emplace_back(dSemiAngle);
}

void UsdBrepBuilder::AppendFaceTorusSurface(
    const pxr::GfVec3d& rOrigin,
    const pxr::GfVec3d& rAxis,
    const pxr::GfVec3d& rRefDirection,
    double dMajorRadius,
    double dMinorRadius
)
{
    UsdBrepArrayData& rArrays = Arrays();
    rArrays.m_sFace_SurfaceTorus_OriginArray.emplace_back(rOrigin);
    rArrays.m_sFace_SurfaceTorus_AxisArray.emplace_back(rAxis);
    rArrays.m_sFace_SurfaceTorus_RefDirectionArray.emplace_back(rRefDirection);
    rArrays.m_sFace_SurfaceTorus_MajorRadiusArray.emplace_back(dMajorRadius);
    rArrays.m_sFace_SurfaceTorus_MinorRadiusArray.emplace_back(dMinorRadius);
}

} // namespace UsdBrepData
