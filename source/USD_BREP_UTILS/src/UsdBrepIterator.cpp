// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepIterator.h"

#include "UsdBrepTokens.h"

#include <pxr/base/tf/span.h>

#include <algorithm>
#include <unordered_map>

namespace UsdBrep
{
using namespace UsdBrepData;
PXR_NAMESPACE_USING_DIRECTIVE

namespace
{
template <typename T>
TfSpan<const T> Sub(const VtArray<T>& arr, size_t start, size_t count)
{
    if (count == 0 || start >= arr.size())
    {
        return TfSpan<const T>();
    }
    count = std::min(count, arr.size() - start);
    using D = typename TfSpan<const T>::difference_type;
    return TfMakeConstSpan(arr).subspan(static_cast<D>(start), static_cast<D>(count));
}
} // anonymous namespace

/*********************************************************************************************************************
 * UsdBrepView
 ********************************************************************************************************************/

UsdBrepView::UsdBrepView(const UsdBrepArrayData& rArrays, uint32_t iBrepIndex) : m_rArrays(rArrays)
{
    m_sSpans.SetStartsAndCountsForBrepIndex(rArrays, iBrepIndex, true, true);
}

uint32_t UsdBrepView::BrepRegionCount() const
{
    uint32_t i = BrepIndex();
    return (i < m_rArrays.m_sBrepRegionCountArray.size()) ? m_rArrays.m_sBrepRegionCountArray[i] : 0;
}

bool UsdBrepView::HasBrepXSectTol3d() const
{
    return BrepIndex() < m_rArrays.m_sBrepXSectTol3dArray.size();
}
double UsdBrepView::BrepXSectTol3d() const
{
    return HasBrepXSectTol3d() ? m_rArrays.m_sBrepXSectTol3dArray[BrepIndex()] : 0.0;
}

bool UsdBrepView::HasBrepExtent() const
{
    return BrepIndex() * 2 + 1 < m_rArrays.m_sBrepExtentArray.size();
}
GfVec3d UsdBrepView::BrepExtentMin() const
{
    return HasBrepExtent() ? m_rArrays.m_sBrepExtentArray[BrepIndex() * 2] : GfVec3d(0.0);
}
GfVec3d UsdBrepView::BrepExtentMax() const
{
    return HasBrepExtent() ? m_rArrays.m_sBrepExtentArray[BrepIndex() * 2 + 1] : GfVec3d(0.0);
}

pxr::TfToken UsdBrepView::FaceSurfaceType(uint32_t iLocalFace) const
{
    auto span = FaceSurfaceTypes();
    if (iLocalFace < span.size())
    {
        return span[iLocalFace];
    }
    return pxr::TfToken();
}

pxr::TfToken UsdBrepView::EdgeCurveType(uint32_t iLocalEdge) const
{
    auto span = EdgeCurveTypes();
    if (iLocalEdge < span.size())
    {
        return span[iLocalEdge];
    }
    return pxr::TfToken();
}

uint32_t UsdBrepView::FaceLoopCount(uint32_t iLocalFace) const
{
    auto span = FaceLoopCounts();
    if (iLocalFace < span.size())
    {
        return span[iLocalFace];
    }
    return 0;
}

uint32_t UsdBrepView::FaceuseLocalFaceIndex(uint32_t localFu) const
{
    auto span = FaceuseFaceIndices();
    if (localFu < span.size())
    {
        return span[localFu] - m_sSpans.m_lFaceStartIndex;
    }
    return 0;
}

uint32_t UsdBrepView::EdgeuseLocalEdgeIndex(uint32_t localEu) const
{
    auto span = EdgeuseEdgeIndices();
    if (localEu < span.size())
    {
        return span[localEu] - m_sSpans.m_lEdgeStartIndex;
    }
    return 0;
}

uint32_t UsdBrepView::EdgeuseNextRadialLocalIndex(uint32_t localEu) const
{
    auto span = EdgeuseNextRadialIndices();
    if (localEu < span.size())
    {
        const uint32_t raw = span[localEu];
        // Preserve the "no radial neighbor" sentinel instead of localizing it into a bogus index.
        if (raw == static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX) || raw < m_sSpans.m_lEdgeuseStartIndex)
        {
            return static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
        }
        return raw - m_sSpans.m_lEdgeuseStartIndex;
    }
    return static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
}

GfVec2i UsdBrepView::EdgeLocalVertexIndices(uint32_t localEdge) const
{
    auto span = EdgeVertexIndices();
    if (localEdge < span.size())
    {
        int32_t off = static_cast<int32_t>(m_sSpans.m_lVertexStartIndex);
        return GfVec2i(span[localEdge][0] - off, span[localEdge][1] - off);
    }
    return GfVec2i(0, 0);
}

GfVec2i UsdBrepView::WireEdgeLocalVertexIndices(uint32_t localWireEdge) const
{
    auto span = WireEdgeVertexIndices();
    if (localWireEdge < span.size())
    {
        int32_t off = static_cast<int32_t>(m_sSpans.m_lVertexStartIndex);
        return GfVec2i(span[localWireEdge][0] - off, span[localWireEdge][1] - off);
    }
    return GfVec2i(0, 0);
}

/*********************************************************************************************************************
 * UsdBrepView: type sub-index helpers
 ********************************************************************************************************************/

void UsdBrepView::BuildFaceTypeSubIndices() const
{
    auto types = FaceSurfaceTypes();
    m_vFaceTypeSubIndices.resize(types.size());
    std::unordered_map<TfToken, uint32_t, TfToken::HashFunctor> counts;
    for (size_t i = 0; i < types.size(); ++i)
    {
        m_vFaceTypeSubIndices[i] = counts[types[i]]++;
    }
}

void UsdBrepView::BuildEdgeTypeSubIndices() const
{
    auto types = EdgeCurveTypes();
    m_vEdgeTypeSubIndices.resize(types.size());
    std::unordered_map<TfToken, uint32_t, TfToken::HashFunctor> counts;
    for (size_t i = 0; i < types.size(); ++i)
    {
        m_vEdgeTypeSubIndices[i] = counts[types[i]]++;
    }
}

uint32_t UsdBrepView::FaceTypeSubIndex(uint32_t iLocalFace) const
{
    if (m_vFaceTypeSubIndices.empty() && m_sSpans.m_lFaceCount > 0)
    {
        BuildFaceTypeSubIndices();
    }
    if (iLocalFace >= m_vFaceTypeSubIndices.size())
    {
        return 0;
    }
    return m_vFaceTypeSubIndices[iLocalFace];
}

uint32_t UsdBrepView::EdgeTypeSubIndex(uint32_t iLocalEdge) const
{
    if (m_vEdgeTypeSubIndices.empty() && m_sSpans.m_lEdgeCount > 0)
    {
        BuildEdgeTypeSubIndices();
    }
    if (iLocalEdge >= m_vEdgeTypeSubIndices.size())
    {
        return 0;
    }
    return m_vEdgeTypeSubIndices[iLocalEdge];
}

/*********************************************************************************************************************
 * UsdBrepView: topology spans
 ********************************************************************************************************************/

// Region
TfSpan<const uint32_t> UsdBrepView::RegionShellCounts() const
{
    return Sub(m_rArrays.m_sRegionShellCountArray, m_sSpans.m_lRegionStartIndex, m_sSpans.m_lRegionCount);
}
TfSpan<const TfToken> UsdBrepView::RegionTypes() const
{
    return Sub(m_rArrays.m_sRegionTypeArray, m_sSpans.m_lRegionStartIndex, m_sSpans.m_lRegionCount);
}

// Shell
TfSpan<const uint32_t> UsdBrepView::ShellFaceuseCounts() const
{
    return Sub(m_rArrays.m_sShellFaceuseCountArray, m_sSpans.m_lShellStartIndex, m_sSpans.m_lShellCount);
}
TfSpan<const uint32_t> UsdBrepView::ShellWireEdgeCounts() const
{
    return Sub(m_rArrays.m_sShellWireEdgeCountArray, m_sSpans.m_lShellStartIndex, m_sSpans.m_lShellCount);
}
TfSpan<const TfToken> UsdBrepView::ShellPointTypes() const
{
    return Sub(m_rArrays.m_sShellPointTypeArray, m_sSpans.m_lShellStartIndex, m_sSpans.m_lShellCount);
}

// Faceuse
TfSpan<const uint32_t> UsdBrepView::FaceuseFaceIndices() const
{
    return Sub(m_rArrays.m_sFaceuseFaceIndexArray, m_sSpans.m_lFaceuseStartIndex, m_sSpans.m_lFaceuseCount);
}
TfSpan<const TfToken> UsdBrepView::FaceuseOrientationTypes() const
{
    return Sub(m_rArrays.m_sFaceuseOrientationTypeArray, m_sSpans.m_lFaceuseStartIndex, m_sSpans.m_lFaceuseCount);
}

// Face
TfSpan<const uint32_t> UsdBrepView::FaceLoopCounts() const
{
    return Sub(m_rArrays.m_sFaceLoopCountArray, m_sSpans.m_lFaceStartIndex, m_sSpans.m_lFaceCount);
}
TfSpan<const TfToken> UsdBrepView::FaceSurfaceTypes() const
{
    return Sub(m_rArrays.m_sFaceSurfaceTypeArray, m_sSpans.m_lFaceStartIndex, m_sSpans.m_lFaceCount);
}
TfSpan<const TfToken> UsdBrepView::FaceTrimTypes() const
{
    return Sub(m_rArrays.m_sFaceTrimTypeArray, m_sSpans.m_lFaceStartIndex, m_sSpans.m_lFaceCount);
}
TfSpan<const GfVec2d> UsdBrepView::FaceRanges() const
{
    return Sub(m_rArrays.m_sFaceRangeArray, 2u * m_sSpans.m_lFaceStartIndex, 2u * m_sSpans.m_lFaceCount);
}

// Loop
TfSpan<const uint32_t> UsdBrepView::LoopEdgeuseCounts() const
{
    return Sub(m_rArrays.m_sLoopEdgeuseCountArray, m_sSpans.m_lLoopStartIndex, m_sSpans.m_lLoopCount);
}
TfSpan<const uint32_t> UsdBrepView::LoopVertexIndices() const
{
    return Sub(m_rArrays.m_sLoopVertexIndexArray, m_sSpans.m_lLoopStartIndex, m_sSpans.m_lLoopCount);
}

// Edgeuse
TfSpan<const uint32_t> UsdBrepView::EdgeuseEdgeIndices() const
{
    return Sub(m_rArrays.m_sEdgeuseEdgeIndexArray, m_sSpans.m_lEdgeuseStartIndex, m_sSpans.m_lEdgeuseCount);
}
TfSpan<const TfToken> UsdBrepView::EdgeuseOrientationTypes() const
{
    return Sub(m_rArrays.m_sEdgeuseOrientationTypeArray, m_sSpans.m_lEdgeuseStartIndex, m_sSpans.m_lEdgeuseCount);
}
TfSpan<const uint32_t> UsdBrepView::EdgeuseNextRadialIndices() const
{
    return Sub(m_rArrays.m_sEdgeuseNextRadialEUIndexArray, m_sSpans.m_lEdgeuseStartIndex, m_sSpans.m_lEdgeuseCount);
}
TfSpan<const TfToken> UsdBrepView::EdgeuseRadialEntryTypes() const
{
    return Sub(m_rArrays.m_sEdgeuseThisRadialEntryTypeArray, m_sSpans.m_lEdgeuseStartIndex, m_sSpans.m_lEdgeuseCount);
}

// Edge
TfSpan<const TfToken> UsdBrepView::EdgeCurveTypes() const
{
    return Sub(m_rArrays.m_sEdgeCurveTypeArray, m_sSpans.m_lEdgeStartIndex, m_sSpans.m_lEdgeCount);
}
TfSpan<const double> UsdBrepView::EdgeRanges() const
{
    return Sub(m_rArrays.m_sEdgeRangeArray, 2u * m_sSpans.m_lEdgeStartIndex, 2u * m_sSpans.m_lEdgeCount);
}
TfSpan<const GfVec2i> UsdBrepView::EdgeVertexIndices() const
{
    return Sub(m_rArrays.m_sEdgeVertexIndicesArray, m_sSpans.m_lEdgeStartIndex, m_sSpans.m_lEdgeCount);
}

// WireEdge
TfSpan<const TfToken> UsdBrepView::WireEdgeCurveTypes() const
{
    return Sub(m_rArrays.m_sWireEdgeCurveTypeArray, m_sSpans.m_lWireEdgeStartIndex, m_sSpans.m_lWireEdgeCount);
}
TfSpan<const double> UsdBrepView::WireEdgeRanges() const
{
    return Sub(m_rArrays.m_sWireEdgeRangeArray, 2u * m_sSpans.m_lWireEdgeStartIndex, 2u * m_sSpans.m_lWireEdgeCount);
}
TfSpan<const GfVec2i> UsdBrepView::WireEdgeVertexIndices() const
{
    return Sub(m_rArrays.m_sWireEdgeVertexIndicesArray, m_sSpans.m_lWireEdgeStartIndex, m_sSpans.m_lWireEdgeCount);
}

// Vertex
TfSpan<const TfToken> UsdBrepView::VertexPointTypes() const
{
    return Sub(m_rArrays.m_sVertexPointTypeArray, m_sSpans.m_lVertexStartIndex, m_sSpans.m_lVertexCount);
}
TfSpan<const GfVec3d> UsdBrepView::VertexPositions() const
{
    return Sub(m_rArrays.m_sVertex_PointPositionArray, m_sSpans.m_lVertexPointPosition_StartIndex, m_sSpans.m_lVertexPointPosition_Count);
}
TfSpan<const GfVec3d> UsdBrepView::ShellVertexPositions() const
{
    return Sub(m_rArrays.m_sShell_PointPositionArray, m_sSpans.m_lShellPointPosition_StartIndex, m_sSpans.m_lShellPointPosition_Count);
}

/*********************************************************************************************************************
 * UsdBrepView: face geometry spans
 ********************************************************************************************************************/

// Plane
TfSpan<const GfVec3d> UsdBrepView::PlaneSurfaceOrigins() const
{
    return Sub(m_rArrays.m_sFace_SurfacePlane_OriginArray, m_sSpans.m_lFacePlaneSurface_StartIndex, m_sSpans.m_lFacePlaneSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::PlaneSurfaceAxes() const
{
    return Sub(m_rArrays.m_sFace_SurfacePlane_AxisArray, m_sSpans.m_lFacePlaneSurface_StartIndex, m_sSpans.m_lFacePlaneSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::PlaneSurfaceRefDirections() const
{
    return Sub(m_rArrays.m_sFace_SurfacePlane_RefDirectionArray, m_sSpans.m_lFacePlaneSurface_StartIndex, m_sSpans.m_lFacePlaneSurface_Count);
}

// Cylinder
TfSpan<const GfVec3d> UsdBrepView::CylinderSurfaceOrigins() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCylinder_OriginArray, m_sSpans.m_lFaceCylinderSurface_StartIndex, m_sSpans.m_lFaceCylinderSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::CylinderSurfaceAxes() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCylinder_AxisArray, m_sSpans.m_lFaceCylinderSurface_StartIndex, m_sSpans.m_lFaceCylinderSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::CylinderSurfaceRefDirections() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCylinder_RefDirectionArray, m_sSpans.m_lFaceCylinderSurface_StartIndex, m_sSpans.m_lFaceCylinderSurface_Count);
}
TfSpan<const double> UsdBrepView::CylinderSurfaceRadii() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCylinder_RadiusArray, m_sSpans.m_lFaceCylinderSurface_StartIndex, m_sSpans.m_lFaceCylinderSurface_Count);
}

// Cone
TfSpan<const GfVec3d> UsdBrepView::ConeSurfaceOrigins() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCone_OriginArray, m_sSpans.m_lFaceConeSurface_StartIndex, m_sSpans.m_lFaceConeSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::ConeSurfaceAxes() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCone_AxisArray, m_sSpans.m_lFaceConeSurface_StartIndex, m_sSpans.m_lFaceConeSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::ConeSurfaceRefDirections() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCone_RefDirectionArray, m_sSpans.m_lFaceConeSurface_StartIndex, m_sSpans.m_lFaceConeSurface_Count);
}
TfSpan<const double> UsdBrepView::ConeSurfaceRadii() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCone_RadiusArray, m_sSpans.m_lFaceConeSurface_StartIndex, m_sSpans.m_lFaceConeSurface_Count);
}
TfSpan<const double> UsdBrepView::ConeSurfaceSemiAngles() const
{
    return Sub(m_rArrays.m_sFace_SurfaceCone_SemiAngleArray, m_sSpans.m_lFaceConeSurface_StartIndex, m_sSpans.m_lFaceConeSurface_Count);
}

// Sphere
TfSpan<const GfVec3d> UsdBrepView::SphereSurfaceCenters() const
{
    return Sub(m_rArrays.m_sFace_SurfaceSphere_CenterArray, m_sSpans.m_lFaceSphereSurface_StartIndex, m_sSpans.m_lFaceSphereSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::SphereSurfaceAxes() const
{
    return Sub(m_rArrays.m_sFace_SurfaceSphere_AxisArray, m_sSpans.m_lFaceSphereSurface_StartIndex, m_sSpans.m_lFaceSphereSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::SphereSurfaceRefDirections() const
{
    return Sub(m_rArrays.m_sFace_SurfaceSphere_RefDirectionArray, m_sSpans.m_lFaceSphereSurface_StartIndex, m_sSpans.m_lFaceSphereSurface_Count);
}
TfSpan<const double> UsdBrepView::SphereSurfaceRadii() const
{
    return Sub(m_rArrays.m_sFace_SurfaceSphere_RadiusArray, m_sSpans.m_lFaceSphereSurface_StartIndex, m_sSpans.m_lFaceSphereSurface_Count);
}

// Torus
TfSpan<const GfVec3d> UsdBrepView::TorusSurfaceOrigins() const
{
    return Sub(m_rArrays.m_sFace_SurfaceTorus_OriginArray, m_sSpans.m_lFaceTorusSurface_StartIndex, m_sSpans.m_lFaceTorusSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::TorusSurfaceAxes() const
{
    return Sub(m_rArrays.m_sFace_SurfaceTorus_AxisArray, m_sSpans.m_lFaceTorusSurface_StartIndex, m_sSpans.m_lFaceTorusSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::TorusSurfaceRefDirections() const
{
    return Sub(m_rArrays.m_sFace_SurfaceTorus_RefDirectionArray, m_sSpans.m_lFaceTorusSurface_StartIndex, m_sSpans.m_lFaceTorusSurface_Count);
}
TfSpan<const double> UsdBrepView::TorusSurfaceMajorRadii() const
{
    return Sub(m_rArrays.m_sFace_SurfaceTorus_MajorRadiusArray, m_sSpans.m_lFaceTorusSurface_StartIndex, m_sSpans.m_lFaceTorusSurface_Count);
}
TfSpan<const double> UsdBrepView::TorusSurfaceMinorRadii() const
{
    return Sub(m_rArrays.m_sFace_SurfaceTorus_MinorRadiusArray, m_sSpans.m_lFaceTorusSurface_StartIndex, m_sSpans.m_lFaceTorusSurface_Count);
}

// NURBS surface
TfSpan<const uint32_t> UsdBrepView::NurbsSurfaceUVertexCounts() const
{
    return Sub(m_rArrays.m_sFace_SurfaceNurb_UVertexCountArray, m_sSpans.m_lFaceBSplineSurface_StartIndex, m_sSpans.m_lFaceBSplineSurface_Count);
}
TfSpan<const uint32_t> UsdBrepView::NurbsSurfaceVVertexCounts() const
{
    return Sub(m_rArrays.m_sFace_SurfaceNurb_VVertexCountArray, m_sSpans.m_lFaceBSplineSurface_StartIndex, m_sSpans.m_lFaceBSplineSurface_Count);
}
TfSpan<const uint32_t> UsdBrepView::NurbsSurfaceUOrders() const
{
    return Sub(m_rArrays.m_sFace_SurfaceNurb_UOrderArray, m_sSpans.m_lFaceBSplineSurface_StartIndex, m_sSpans.m_lFaceBSplineSurface_Count);
}
TfSpan<const uint32_t> UsdBrepView::NurbsSurfaceVOrders() const
{
    return Sub(m_rArrays.m_sFace_SurfaceNurb_VOrderArray, m_sSpans.m_lFaceBSplineSurface_StartIndex, m_sSpans.m_lFaceBSplineSurface_Count);
}
TfSpan<const GfVec3d> UsdBrepView::NurbsSurfaceControlVertices() const
{
    return Sub(
        m_rArrays.m_sFace_SurfaceNurb_ControlVerticesArray,
        m_sSpans.m_lFaceBSplineSurface_ControlVerticesStartIndex,
        m_sSpans.m_lFaceBSplineSurface_ControlVerticesCount
    );
}
TfSpan<const double> UsdBrepView::NurbsSurfaceUKnots() const
{
    return Sub(m_rArrays.m_sFace_SurfaceNurb_UKnotsArray, m_sSpans.m_lFaceBSplineSurface_UKnotStartIndex, m_sSpans.m_lFaceBSplineSurface_UKnotCount);
}
TfSpan<const double> UsdBrepView::NurbsSurfaceVKnots() const
{
    return Sub(m_rArrays.m_sFace_SurfaceNurb_VKnotsArray, m_sSpans.m_lFaceBSplineSurface_VKnotStartIndex, m_sSpans.m_lFaceBSplineSurface_VKnotCount);
}
TfSpan<const double> UsdBrepView::NurbsSurfaceWeights() const
{
    return Sub(
        m_rArrays.m_sFace_SurfaceNurb_WeightsArray,
        m_sSpans.m_lFaceBSplineSurface_ControlVerticesStartIndex,
        m_sSpans.m_lFaceBSplineSurface_ControlVerticesCount
    );
}

/*********************************************************************************************************************
 * UsdBrepView: edge geometry spans
 ********************************************************************************************************************/

// Edge NURBS
TfSpan<const uint32_t> UsdBrepView::EdgeNurbsVertexCounts() const
{
    return Sub(m_rArrays.m_sEdge_CurveNurb_VertexCountArray, m_sSpans.m_lEdgeBSplineCurve3d_StartIndex, m_sSpans.m_lEdgeBSplineCurve3d_Count);
}
TfSpan<const uint32_t> UsdBrepView::EdgeNurbsOrders() const
{
    return Sub(m_rArrays.m_sEdge_CurveNurb_OrderArray, m_sSpans.m_lEdgeBSplineCurve3d_StartIndex, m_sSpans.m_lEdgeBSplineCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::EdgeNurbsControlVertices() const
{
    return Sub(
        m_rArrays.m_sEdge_CurveNurb_ControlVerticesArray,
        m_sSpans.m_lEdgeBSplineCurve3d_ControlVerticesStartIndex,
        m_sSpans.m_lEdgeBSplineCurve3d_ControlVerticesCount
    );
}
TfSpan<const double> UsdBrepView::EdgeNurbsKnots() const
{
    return Sub(m_rArrays.m_sEdge_CurveNurb_KnotsArray, m_sSpans.m_lEdgeBSplineCurve3d_KnotStartIndex, m_sSpans.m_lEdgeBSplineCurve3d_KnotCount);
}
TfSpan<const double> UsdBrepView::EdgeNurbsWeights() const
{
    return Sub(
        m_rArrays.m_sEdge_CurveNurb_WeightsArray,
        m_sSpans.m_lEdgeBSplineCurve3d_ControlVerticesStartIndex,
        m_sSpans.m_lEdgeBSplineCurve3d_ControlVerticesCount
    );
}

// Edge Circle
TfSpan<const GfVec3d> UsdBrepView::EdgeCircleCenters() const
{
    return Sub(m_rArrays.m_sEdge_CurveCircle_CenterArray, m_sSpans.m_lEdgeCircleCurve3d_StartIndex, m_sSpans.m_lEdgeCircleCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::EdgeCircleAxes() const
{
    return Sub(m_rArrays.m_sEdge_CurveCircle_AxisArray, m_sSpans.m_lEdgeCircleCurve3d_StartIndex, m_sSpans.m_lEdgeCircleCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::EdgeCircleRefDirections() const
{
    return Sub(m_rArrays.m_sEdge_CurveCircle_RefDirectionArray, m_sSpans.m_lEdgeCircleCurve3d_StartIndex, m_sSpans.m_lEdgeCircleCurve3d_Count);
}
TfSpan<const double> UsdBrepView::EdgeCircleRadii() const
{
    return Sub(m_rArrays.m_sEdge_CurveCircle_RadiusArray, m_sSpans.m_lEdgeCircleCurve3d_StartIndex, m_sSpans.m_lEdgeCircleCurve3d_Count);
}

// Edge Line
TfSpan<const GfVec3d> UsdBrepView::EdgeLineOrigins() const
{
    return Sub(m_rArrays.m_sEdge_CurveLine_OriginArray, m_sSpans.m_lEdgeLineCurve3d_StartIndex, m_sSpans.m_lEdgeLineCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::EdgeLineDirections() const
{
    return Sub(m_rArrays.m_sEdge_CurveLine_DirectionArray, m_sSpans.m_lEdgeLineCurve3d_StartIndex, m_sSpans.m_lEdgeLineCurve3d_Count);
}

// Edge Ellipse
TfSpan<const GfVec3d> UsdBrepView::EdgeEllipseCenters() const
{
    return Sub(m_rArrays.m_sEdge_CurveEllipse_CenterArray, m_sSpans.m_lEdgeEllipseCurve3d_StartIndex, m_sSpans.m_lEdgeEllipseCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::EdgeEllipseAxes() const
{
    return Sub(m_rArrays.m_sEdge_CurveEllipse_AxisArray, m_sSpans.m_lEdgeEllipseCurve3d_StartIndex, m_sSpans.m_lEdgeEllipseCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::EdgeEllipseRefDirections() const
{
    return Sub(m_rArrays.m_sEdge_CurveEllipse_RefDirectionArray, m_sSpans.m_lEdgeEllipseCurve3d_StartIndex, m_sSpans.m_lEdgeEllipseCurve3d_Count);
}
TfSpan<const double> UsdBrepView::EdgeEllipseXRadii() const
{
    return Sub(m_rArrays.m_sEdge_CurveEllipse_XRadiusArray, m_sSpans.m_lEdgeEllipseCurve3d_StartIndex, m_sSpans.m_lEdgeEllipseCurve3d_Count);
}
TfSpan<const double> UsdBrepView::EdgeEllipseYRadii() const
{
    return Sub(m_rArrays.m_sEdge_CurveEllipse_YRadiusArray, m_sSpans.m_lEdgeEllipseCurve3d_StartIndex, m_sSpans.m_lEdgeEllipseCurve3d_Count);
}

/*********************************************************************************************************************
 * UsdBrepView: wire edge geometry spans
 ********************************************************************************************************************/

TfSpan<const uint32_t> UsdBrepView::WireEdgeNurbsVertexCounts() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveNurb_VertexCountArray,
        m_sSpans.m_lWireEdgeBSplineCurve3d_StartIndex,
        m_sSpans.m_lWireEdgeBSplineCurve3d_Count
    );
}
TfSpan<const uint32_t> UsdBrepView::WireEdgeNurbsOrders() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveNurb_OrderArray, m_sSpans.m_lWireEdgeBSplineCurve3d_StartIndex, m_sSpans.m_lWireEdgeBSplineCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::WireEdgeNurbsControlVertices() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray,
        m_sSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex,
        m_sSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesCount
    );
}
TfSpan<const double> UsdBrepView::WireEdgeNurbsKnots() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveNurb_KnotsArray,
        m_sSpans.m_lWireEdgeBSplineCurve3d_KnotStartIndex,
        m_sSpans.m_lWireEdgeBSplineCurve3d_KnotCount
    );
}
TfSpan<const double> UsdBrepView::WireEdgeNurbsWeights() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveNurb_WeightsArray,
        m_sSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex,
        m_sSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesCount
    );
}

TfSpan<const GfVec3d> UsdBrepView::WireEdgeCircleCenters() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveCircle_CenterArray, m_sSpans.m_lWireEdgeCircleCurve3d_StartIndex, m_sSpans.m_lWireEdgeCircleCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::WireEdgeCircleAxes() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveCircle_AxisArray, m_sSpans.m_lWireEdgeCircleCurve3d_StartIndex, m_sSpans.m_lWireEdgeCircleCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::WireEdgeCircleRefDirections() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveCircle_RefDirectionArray,
        m_sSpans.m_lWireEdgeCircleCurve3d_StartIndex,
        m_sSpans.m_lWireEdgeCircleCurve3d_Count
    );
}
TfSpan<const double> UsdBrepView::WireEdgeCircleRadii() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveCircle_RadiusArray, m_sSpans.m_lWireEdgeCircleCurve3d_StartIndex, m_sSpans.m_lWireEdgeCircleCurve3d_Count);
}

TfSpan<const GfVec3d> UsdBrepView::WireEdgeLineOrigins() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveLine_OriginArray, m_sSpans.m_lWireEdgeLineCurve3d_StartIndex, m_sSpans.m_lWireEdgeLineCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::WireEdgeLineDirections() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveLine_DirectionArray, m_sSpans.m_lWireEdgeLineCurve3d_StartIndex, m_sSpans.m_lWireEdgeLineCurve3d_Count);
}

TfSpan<const GfVec3d> UsdBrepView::WireEdgeEllipseCenters() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveEllipse_CenterArray,
        m_sSpans.m_lWireEdgeEllipseCurve3d_StartIndex,
        m_sSpans.m_lWireEdgeEllipseCurve3d_Count
    );
}
TfSpan<const GfVec3d> UsdBrepView::WireEdgeEllipseAxes() const
{
    return Sub(m_rArrays.m_sWireEdge_CurveEllipse_AxisArray, m_sSpans.m_lWireEdgeEllipseCurve3d_StartIndex, m_sSpans.m_lWireEdgeEllipseCurve3d_Count);
}
TfSpan<const GfVec3d> UsdBrepView::WireEdgeEllipseRefDirections() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveEllipse_RefDirectionArray,
        m_sSpans.m_lWireEdgeEllipseCurve3d_StartIndex,
        m_sSpans.m_lWireEdgeEllipseCurve3d_Count
    );
}
TfSpan<const double> UsdBrepView::WireEdgeEllipseXRadii() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveEllipse_XRadiusArray,
        m_sSpans.m_lWireEdgeEllipseCurve3d_StartIndex,
        m_sSpans.m_lWireEdgeEllipseCurve3d_Count
    );
}
TfSpan<const double> UsdBrepView::WireEdgeEllipseYRadii() const
{
    return Sub(
        m_rArrays.m_sWireEdge_CurveEllipse_YRadiusArray,
        m_sSpans.m_lWireEdgeEllipseCurve3d_StartIndex,
        m_sSpans.m_lWireEdgeEllipseCurve3d_Count
    );
}

/*********************************************************************************************************************
 * UsdBrepView: edgeuse UV trim curve spans
 ********************************************************************************************************************/

TfSpan<const uint32_t> UsdBrepView::EdgeuseUVNurbsVertexCounts() const
{
    return Sub(m_rArrays.m_sEdgeuse_CurveNurb_VertexCountArray, m_sSpans.m_lEdgeuseBSplineCurve2d_StartIndex, m_sSpans.m_lEdgeuseBSplineCurve2d_Count);
}
TfSpan<const uint32_t> UsdBrepView::EdgeuseUVNurbsOrders() const
{
    return Sub(m_rArrays.m_sEdgeuse_CurveNurb_OrderArray, m_sSpans.m_lEdgeuseBSplineCurve2d_StartIndex, m_sSpans.m_lEdgeuseBSplineCurve2d_Count);
}
TfSpan<const GfVec2d> UsdBrepView::EdgeuseUVNurbsControlVertices() const
{
    return Sub(
        m_rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray,
        m_sSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex,
        m_sSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesCount
    );
}
TfSpan<const double> UsdBrepView::EdgeuseUVNurbsKnots() const
{
    return Sub(
        m_rArrays.m_sEdgeuse_CurveNurb_KnotsArray,
        m_sSpans.m_lEdgeuseBSplineCurve2d_KnotStartIndex,
        m_sSpans.m_lEdgeuseBSplineCurve2d_KnotCount
    );
}
TfSpan<const double> UsdBrepView::EdgeuseUVNurbsWeights() const
{
    return Sub(
        m_rArrays.m_sEdgeuse_CurveNurb_WeightsArray,
        m_sSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex,
        m_sSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesCount
    );
}

/*********************************************************************************************************************
 * UsdBrepIterator
 ********************************************************************************************************************/

UsdBrepIterator::UsdBrepIterator(const UsdBrepArrayData& rArrays, uint32_t iBrepIndex) : m_rArrays(rArrays), m_iBrepIndex(iBrepIndex)
{
}

UsdBrepView UsdBrepIterator::operator*() const
{
    return UsdBrepView(m_rArrays, m_iBrepIndex);
}

UsdBrepIterator& UsdBrepIterator::operator++()
{
    ++m_iBrepIndex;
    return *this;
}

bool UsdBrepIterator::operator!=(const UsdBrepIterator& rOther) const
{
    return m_iBrepIndex != rOther.m_iBrepIndex;
}

bool UsdBrepIterator::operator==(const UsdBrepIterator& rOther) const
{
    return m_iBrepIndex == rOther.m_iBrepIndex;
}

/*********************************************************************************************************************
 * UsdBrepRange
 ********************************************************************************************************************/

UsdBrepRange::UsdBrepRange(const UsdBrepArrayData& rArrays) : m_rArrays(rArrays)
{
}

UsdBrepIterator UsdBrepRange::begin() const
{
    return UsdBrepIterator(m_rArrays, 0);
}

UsdBrepIterator UsdBrepRange::end() const
{
    return UsdBrepIterator(m_rArrays, m_rArrays.TotalBrepCount());
}

uint32_t UsdBrepRange::size() const
{
    return m_rArrays.TotalBrepCount();
}

} // end namespace UsdBrep
