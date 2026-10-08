// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepPacking.h"

#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes
#include "UsdBrepIterator.h"
#include "UsdBrepTokens.h"

#include <pxr/base/tf/span.h>

namespace UsdBrep
{
PXR_NAMESPACE_USING_DIRECTIVE
using namespace UsdBrepData;

template <typename T>
static void Append(TfSpan<const T> src, VtArray<T>& dest)
{
    if (src.empty())
    {
        return;
    }
    dest.reserve(dest.size() + src.size());
    dest.insert(dest.end(), src.begin(), src.end());
}

/*********************************************************************************************************************
 * AppendBrepToArray: append a single Brep (via UsdBrepView) into rDest.
 * Topology cross-reference indices (faceuse->face, edgeuse->edge,
 * loop->vertex, edgeuse->nextRadialEU, edge->vertex) are converted from
 * local indices to dest-global indices.
 ********************************************************************************************************************/
bool AppendBrepToArray(const UsdBrepView& brep, UsdBrepArrayData& rDest)
{
    // Nothing to append for an empty/degenerate view (e.g. an out-of-range
    // brep index, whose spans are all zero-length).
    if (brep.RegionCount() == 0 && brep.FaceCount() == 0 && brep.WireEdgeCount() == 0 && brep.VertexCount() == 0)
    {
        return false;
    }

    uint32_t destFace = rDest.TotalFaceCount();
    uint32_t destEdge = rDest.TotalEdgeCount();
    uint32_t destVert = rDest.TotalVertexCount();
    uint32_t destEu = rDest.TotalEdgeuseCount();

    // Per-Brep metadata. These arrays are indexed by brep index, so a missing
    // value on one source and a present value on another would shift later
    // entries onto the wrong brep. Refuse to append when that would happen.
    const uint32_t destBrepIndex = rDest.TotalBrepCount();
    const bool hasXSectTol = brep.HasBrepXSectTol3d();
    const bool hasExtent = brep.HasBrepExtent();

    // Validate every per-Brep metadata invariant before mutating rDest so a
    // later failure cannot leave the destination partially appended.
    if (hasXSectTol && rDest.m_sBrepXSectTol3dArray.size() != destBrepIndex)
    {
        return false;
    }
    if (hasExtent && rDest.m_sBrepExtentArray.size() != static_cast<size_t>(destBrepIndex) * 2)
    {
        return false;
    }

    if (hasXSectTol)
    {
        rDest.m_sBrepXSectTol3dArray.push_back(brep.BrepXSectTol3d());
    }
    if (hasExtent)
    {
        rDest.m_sBrepExtentArray.push_back(brep.BrepExtentMin());
        rDest.m_sBrepExtentArray.push_back(brep.BrepExtentMax());
    }
    rDest.m_sBrepRegionCountArray.push_back(brep.BrepRegionCount());

    // Regions
    Append(brep.RegionShellCounts(), rDest.m_sRegionShellCountArray);
    Append(brep.RegionTypes(), rDest.m_sRegionTypeArray);

    // Shells
    Append(brep.ShellFaceuseCounts(), rDest.m_sShellFaceuseCountArray);
    Append(brep.ShellWireEdgeCounts(), rDest.m_sShellWireEdgeCountArray);
    Append(brep.ShellPointTypes(), rDest.m_sShellPointTypeArray);

    // Faceuses (local face index → dest global)
    for (uint32_t i = 0; i < brep.FaceuseCount(); ++i)
    {
        rDest.m_sFaceuseFaceIndexArray.push_back(brep.FaceuseLocalFaceIndex(i) + destFace);
    }
    Append(brep.FaceuseOrientationTypes(), rDest.m_sFaceuseOrientationTypeArray);

    // Faces
    Append(brep.FaceLoopCounts(), rDest.m_sFaceLoopCountArray);
    Append(brep.FaceSurfaceTypes(), rDest.m_sFaceSurfaceTypeArray);
    Append(brep.FaceTrimTypes(), rDest.m_sFaceTrimTypeArray);
    Append(brep.FaceRanges(), rDest.m_sFaceRangeArray);

    // Loops (remap vertex indices, preserving USDBREP_NO_OBJECT_INDEX sentinel)
    Append(brep.LoopEdgeuseCounts(), rDest.m_sLoopEdgeuseCountArray);
    {
        auto loopVtx = brep.LoopVertexIndices();
        for (size_t i = 0; i < loopVtx.size(); ++i)
        {
            uint32_t val = loopVtx[i];
            rDest.m_sLoopVertexIndexArray.push_back(val == static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX) ? val : (val + destVert));
        }
    }

    // Edgeuses (local edge/edgeuse indices → dest global)
    for (uint32_t i = 0; i < brep.EdgeuseCount(); ++i)
    {
        rDest.m_sEdgeuseEdgeIndexArray.push_back(brep.EdgeuseLocalEdgeIndex(i) + destEdge);
    }
    Append(brep.EdgeuseOrientationTypes(), rDest.m_sEdgeuseOrientationTypeArray);
    {
        // Remap next-radial edgeuse indices, preserving the USDBREP_NO_OBJECT_INDEX
        // sentinel (a non-manifold/open edgeuse has no radial neighbour). Read the
        // raw stored value so the sentinel is detected before it is localized.
        auto nextRadialRaw = brep.EdgeuseNextRadialIndices();
        for (uint32_t i = 0; i < brep.EdgeuseCount(); ++i)
        {
            uint32_t raw = (i < nextRadialRaw.size()) ? nextRadialRaw[i] : static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
            if (raw == static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX))
            {
                rDest.m_sEdgeuseNextRadialEUIndexArray.push_back(raw);
            }
            else
            {
                rDest.m_sEdgeuseNextRadialEUIndexArray.push_back(brep.EdgeuseNextRadialLocalIndex(i) + destEu);
            }
        }
    }
    Append(brep.EdgeuseRadialEntryTypes(), rDest.m_sEdgeuseThisRadialEntryTypeArray);

    // Edges (local vertex indices → dest global)
    Append(brep.EdgeCurveTypes(), rDest.m_sEdgeCurveTypeArray);
    Append(brep.EdgeRanges(), rDest.m_sEdgeRangeArray);
    for (uint32_t i = 0; i < brep.EdgeCount(); ++i)
    {
        GfVec2i v = brep.EdgeLocalVertexIndices(i);
        rDest.m_sEdgeVertexIndicesArray.push_back(GfVec2i(v[0] + static_cast<int32_t>(destVert), v[1] + static_cast<int32_t>(destVert)));
    }

    // WireEdges (local vertex indices → dest global)
    Append(brep.WireEdgeCurveTypes(), rDest.m_sWireEdgeCurveTypeArray);
    Append(brep.WireEdgeRanges(), rDest.m_sWireEdgeRangeArray);
    for (uint32_t i = 0; i < brep.WireEdgeCount(); ++i)
    {
        GfVec2i v = brep.WireEdgeLocalVertexIndices(i);
        rDest.m_sWireEdgeVertexIndicesArray.push_back(GfVec2i(v[0] + static_cast<int32_t>(destVert), v[1] + static_cast<int32_t>(destVert)));
    }

    // Vertices
    Append(brep.VertexPointTypes(), rDest.m_sVertexPointTypeArray);

    // Geometry: points
    Append(brep.ShellVertexPositions(), rDest.m_sShell_PointPositionArray);
    Append(brep.VertexPositions(), rDest.m_sVertex_PointPositionArray);

    // Geometry: Edge NURBS curves
    Append(brep.EdgeNurbsVertexCounts(), rDest.m_sEdge_CurveNurb_VertexCountArray);
    Append(brep.EdgeNurbsOrders(), rDest.m_sEdge_CurveNurb_OrderArray);
    Append(brep.EdgeNurbsControlVertices(), rDest.m_sEdge_CurveNurb_ControlVerticesArray);
    Append(brep.EdgeNurbsKnots(), rDest.m_sEdge_CurveNurb_KnotsArray);
    Append(brep.EdgeNurbsWeights(), rDest.m_sEdge_CurveNurb_WeightsArray);

    // Geometry: Edge analytic curves
    Append(brep.EdgeCircleCenters(), rDest.m_sEdge_CurveCircle_CenterArray);
    Append(brep.EdgeCircleAxes(), rDest.m_sEdge_CurveCircle_AxisArray);
    Append(brep.EdgeCircleRefDirections(), rDest.m_sEdge_CurveCircle_RefDirectionArray);
    Append(brep.EdgeCircleRadii(), rDest.m_sEdge_CurveCircle_RadiusArray);
    Append(brep.EdgeLineOrigins(), rDest.m_sEdge_CurveLine_OriginArray);
    Append(brep.EdgeLineDirections(), rDest.m_sEdge_CurveLine_DirectionArray);
    Append(brep.EdgeEllipseCenters(), rDest.m_sEdge_CurveEllipse_CenterArray);
    Append(brep.EdgeEllipseAxes(), rDest.m_sEdge_CurveEllipse_AxisArray);
    Append(brep.EdgeEllipseRefDirections(), rDest.m_sEdge_CurveEllipse_RefDirectionArray);
    Append(brep.EdgeEllipseXRadii(), rDest.m_sEdge_CurveEllipse_XRadiusArray);
    Append(brep.EdgeEllipseYRadii(), rDest.m_sEdge_CurveEllipse_YRadiusArray);

    // Geometry: Edgeuse UV trim curves
    Append(brep.EdgeuseUVNurbsVertexCounts(), rDest.m_sEdgeuse_CurveNurb_VertexCountArray);
    Append(brep.EdgeuseUVNurbsOrders(), rDest.m_sEdgeuse_CurveNurb_OrderArray);
    Append(brep.EdgeuseUVNurbsControlVertices(), rDest.m_sEdgeuse_CurveNurb_ControlVerticesArray);
    Append(brep.EdgeuseUVNurbsKnots(), rDest.m_sEdgeuse_CurveNurb_KnotsArray);
    Append(brep.EdgeuseUVNurbsWeights(), rDest.m_sEdgeuse_CurveNurb_WeightsArray);

    // Geometry: Face NURBS surfaces
    Append(brep.NurbsSurfaceUVertexCounts(), rDest.m_sFace_SurfaceNurb_UVertexCountArray);
    Append(brep.NurbsSurfaceVVertexCounts(), rDest.m_sFace_SurfaceNurb_VVertexCountArray);
    Append(brep.NurbsSurfaceUOrders(), rDest.m_sFace_SurfaceNurb_UOrderArray);
    Append(brep.NurbsSurfaceVOrders(), rDest.m_sFace_SurfaceNurb_VOrderArray);
    Append(brep.NurbsSurfaceControlVertices(), rDest.m_sFace_SurfaceNurb_ControlVerticesArray);
    Append(brep.NurbsSurfaceUKnots(), rDest.m_sFace_SurfaceNurb_UKnotsArray);
    Append(brep.NurbsSurfaceVKnots(), rDest.m_sFace_SurfaceNurb_VKnotsArray);
    Append(brep.NurbsSurfaceWeights(), rDest.m_sFace_SurfaceNurb_WeightsArray);

    // Geometry: Face analytic surfaces
    Append(brep.SphereSurfaceCenters(), rDest.m_sFace_SurfaceSphere_CenterArray);
    Append(brep.SphereSurfaceAxes(), rDest.m_sFace_SurfaceSphere_AxisArray);
    Append(brep.SphereSurfaceRefDirections(), rDest.m_sFace_SurfaceSphere_RefDirectionArray);
    Append(brep.SphereSurfaceRadii(), rDest.m_sFace_SurfaceSphere_RadiusArray);
    Append(brep.PlaneSurfaceOrigins(), rDest.m_sFace_SurfacePlane_OriginArray);
    Append(brep.PlaneSurfaceAxes(), rDest.m_sFace_SurfacePlane_AxisArray);
    Append(brep.PlaneSurfaceRefDirections(), rDest.m_sFace_SurfacePlane_RefDirectionArray);
    Append(brep.CylinderSurfaceOrigins(), rDest.m_sFace_SurfaceCylinder_OriginArray);
    Append(brep.CylinderSurfaceAxes(), rDest.m_sFace_SurfaceCylinder_AxisArray);
    Append(brep.CylinderSurfaceRefDirections(), rDest.m_sFace_SurfaceCylinder_RefDirectionArray);
    Append(brep.CylinderSurfaceRadii(), rDest.m_sFace_SurfaceCylinder_RadiusArray);
    Append(brep.ConeSurfaceOrigins(), rDest.m_sFace_SurfaceCone_OriginArray);
    Append(brep.ConeSurfaceAxes(), rDest.m_sFace_SurfaceCone_AxisArray);
    Append(brep.ConeSurfaceRefDirections(), rDest.m_sFace_SurfaceCone_RefDirectionArray);
    Append(brep.ConeSurfaceRadii(), rDest.m_sFace_SurfaceCone_RadiusArray);
    Append(brep.ConeSurfaceSemiAngles(), rDest.m_sFace_SurfaceCone_SemiAngleArray);
    Append(brep.TorusSurfaceOrigins(), rDest.m_sFace_SurfaceTorus_OriginArray);
    Append(brep.TorusSurfaceAxes(), rDest.m_sFace_SurfaceTorus_AxisArray);
    Append(brep.TorusSurfaceRefDirections(), rDest.m_sFace_SurfaceTorus_RefDirectionArray);
    Append(brep.TorusSurfaceMajorRadii(), rDest.m_sFace_SurfaceTorus_MajorRadiusArray);
    Append(brep.TorusSurfaceMinorRadii(), rDest.m_sFace_SurfaceTorus_MinorRadiusArray);

    // Geometry: WireEdge curves
    Append(brep.WireEdgeNurbsVertexCounts(), rDest.m_sWireEdge_CurveNurb_VertexCountArray);
    Append(brep.WireEdgeNurbsOrders(), rDest.m_sWireEdge_CurveNurb_OrderArray);
    Append(brep.WireEdgeNurbsControlVertices(), rDest.m_sWireEdge_CurveNurb_ControlVerticesArray);
    Append(brep.WireEdgeNurbsKnots(), rDest.m_sWireEdge_CurveNurb_KnotsArray);
    Append(brep.WireEdgeNurbsWeights(), rDest.m_sWireEdge_CurveNurb_WeightsArray);
    Append(brep.WireEdgeCircleCenters(), rDest.m_sWireEdge_CurveCircle_CenterArray);
    Append(brep.WireEdgeCircleAxes(), rDest.m_sWireEdge_CurveCircle_AxisArray);
    Append(brep.WireEdgeCircleRefDirections(), rDest.m_sWireEdge_CurveCircle_RefDirectionArray);
    Append(brep.WireEdgeCircleRadii(), rDest.m_sWireEdge_CurveCircle_RadiusArray);
    Append(brep.WireEdgeLineOrigins(), rDest.m_sWireEdge_CurveLine_OriginArray);
    Append(brep.WireEdgeLineDirections(), rDest.m_sWireEdge_CurveLine_DirectionArray);
    Append(brep.WireEdgeEllipseCenters(), rDest.m_sWireEdge_CurveEllipse_CenterArray);
    Append(brep.WireEdgeEllipseAxes(), rDest.m_sWireEdge_CurveEllipse_AxisArray);
    Append(brep.WireEdgeEllipseRefDirections(), rDest.m_sWireEdge_CurveEllipse_RefDirectionArray);
    Append(brep.WireEdgeEllipseXRadii(), rDest.m_sWireEdge_CurveEllipse_XRadiusArray);
    Append(brep.WireEdgeEllipseYRadii(), rDest.m_sWireEdge_CurveEllipse_YRadiusArray);

    return true;
}

} // end namespace UsdBrep
