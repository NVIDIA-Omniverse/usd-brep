// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepIterator.h
 * PURPOSE: Range-based iteration over Breps and their topology within a UsdBrepArrayData.
 *
 * CONTAINS:
 *  namespace UsdBrep {
 *      UsdBrepView       - lightweight view of one Brep within a UsdBrepArrayData
 *      UsdBrepIterator    - forward iterator over Breps
 *      UsdBrepRange       - range adapter for range-based for loops
 *  }
 *
 * USAGE:
 *      for (const auto& brep : UsdBrepRange(arrays)) {
 *          uint32_t brepIdx = brep.BrepIndex();
 *          for (uint32_t f = 0; f < brep.FaceCount(); ++f) {
 *              TfToken surfType = brep.FaceSurfaceType(f);
 *          }
 *      }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_ITERATOR_H_
#define _USD_BREP_ITERATOR_H_

#include "UsdBrepArrayData.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes
#include "UsdBrepUtilsConfig.h"

#include <pxr/base/tf/span.h>

#include <vector>

namespace UsdBrep
{
using UsdBrepData::UsdBrepArrayData;
using UsdBrepData::UsdBrepArraySpans;
PXR_NAMESPACE_USING_DIRECTIVE

/*********************************************************************************************************************
 * UsdBrepView: a lightweight, non-owning view of one Brep within a UsdBrepArrayData.
 *              Provides direct access to per-Brep topology counts and global index offsets.
 ********************************************************************************************************************/
class USD_BREP_EXPORT UsdBrepView final
{
public:

    UsdBrepView(const UsdBrepArrayData& rArrays, uint32_t iBrepIndex);

    uint32_t BrepIndex() const
    {
        return m_sSpans.m_lBrepIndex;
    }

    // Per-brep metadata
    uint32_t BrepRegionCount() const;
    bool HasBrepXSectTol3d() const;
    double BrepXSectTol3d() const;
    bool HasBrepExtent() const;
    GfVec3d BrepExtentMin() const;
    GfVec3d BrepExtentMax() const;

    // Topology counts for this Brep
    uint32_t RegionCount() const
    {
        return m_sSpans.m_lRegionCount;
    }
    uint32_t ShellCount() const
    {
        return m_sSpans.m_lShellCount;
    }
    uint32_t FaceuseCount() const
    {
        return m_sSpans.m_lFaceuseCount;
    }
    uint32_t FaceCount() const
    {
        return m_sSpans.m_lFaceCount;
    }
    uint32_t LoopCount() const
    {
        return m_sSpans.m_lLoopCount;
    }
    uint32_t EdgeuseCount() const
    {
        return m_sSpans.m_lEdgeuseCount;
    }
    uint32_t EdgeCount() const
    {
        return m_sSpans.m_lEdgeCount;
    }
    uint32_t WireEdgeCount() const
    {
        return m_sSpans.m_lWireEdgeCount;
    }
    uint32_t VertexCount() const
    {
        return m_sSpans.m_lVertexCount;
    }

    // Convenience: surface type for a local face index
    pxr::TfToken FaceSurfaceType(uint32_t iLocalFace) const;

    // Convenience: curve type for a local edge index
    pxr::TfToken EdgeCurveType(uint32_t iLocalEdge) const;

    // Convenience: get loop count for a local face
    uint32_t FaceLoopCount(uint32_t iLocalFace) const;

    // ---- Local-index cross-reference accessors ----
    // These return local indices for topology arrays whose stored values are global.
    uint32_t FaceuseLocalFaceIndex(uint32_t localFu) const;
    uint32_t EdgeuseLocalEdgeIndex(uint32_t localEu) const;
    uint32_t EdgeuseNextRadialLocalIndex(uint32_t localEu) const;
    GfVec2i EdgeLocalVertexIndices(uint32_t localEdge) const;
    GfVec2i WireEdgeLocalVertexIndices(uint32_t localWireEdge) const;

    // ---- Type sub-index helpers ----
    // Position of a local face/edge among same-type faces/edges in this brep.
    // Used to index into geometry spans (e.g., PlaneSurfaceOrigins()[subIdx]).
    // O(1) after the first call (lazily precomputed in a single O(n) pass).
    uint32_t FaceTypeSubIndex(uint32_t iLocalFace) const;
    uint32_t EdgeTypeSubIndex(uint32_t iLocalEdge) const;

    // ---- Topology spans ----

    TfSpan<const uint32_t> RegionShellCounts() const;
    TfSpan<const TfToken> RegionTypes() const;

    TfSpan<const uint32_t> ShellFaceuseCounts() const;
    TfSpan<const uint32_t> ShellWireEdgeCounts() const;
    TfSpan<const TfToken> ShellPointTypes() const;

    TfSpan<const uint32_t> FaceuseFaceIndices() const;
    TfSpan<const TfToken> FaceuseOrientationTypes() const;

    TfSpan<const uint32_t> FaceLoopCounts() const;
    TfSpan<const TfToken> FaceSurfaceTypes() const;
    TfSpan<const TfToken> FaceTrimTypes() const;
    TfSpan<const GfVec2d> FaceRanges() const;

    TfSpan<const uint32_t> LoopEdgeuseCounts() const;
    TfSpan<const uint32_t> LoopVertexIndices() const;

    TfSpan<const uint32_t> EdgeuseEdgeIndices() const;
    TfSpan<const TfToken> EdgeuseOrientationTypes() const;
    TfSpan<const uint32_t> EdgeuseNextRadialIndices() const;
    TfSpan<const TfToken> EdgeuseRadialEntryTypes() const;

    TfSpan<const TfToken> EdgeCurveTypes() const;
    TfSpan<const double> EdgeRanges() const;
    TfSpan<const GfVec2i> EdgeVertexIndices() const;

    TfSpan<const TfToken> WireEdgeCurveTypes() const;
    TfSpan<const double> WireEdgeRanges() const;
    TfSpan<const GfVec2i> WireEdgeVertexIndices() const;

    TfSpan<const TfToken> VertexPointTypes() const;
    TfSpan<const GfVec3d> VertexPositions() const;

    TfSpan<const GfVec3d> ShellVertexPositions() const;

    // ---- Face geometry spans ----

    TfSpan<const GfVec3d> PlaneSurfaceOrigins() const;
    TfSpan<const GfVec3d> PlaneSurfaceAxes() const;
    TfSpan<const GfVec3d> PlaneSurfaceRefDirections() const;

    TfSpan<const GfVec3d> CylinderSurfaceOrigins() const;
    TfSpan<const GfVec3d> CylinderSurfaceAxes() const;
    TfSpan<const GfVec3d> CylinderSurfaceRefDirections() const;
    TfSpan<const double> CylinderSurfaceRadii() const;

    TfSpan<const GfVec3d> ConeSurfaceOrigins() const;
    TfSpan<const GfVec3d> ConeSurfaceAxes() const;
    TfSpan<const GfVec3d> ConeSurfaceRefDirections() const;
    TfSpan<const double> ConeSurfaceRadii() const;
    TfSpan<const double> ConeSurfaceSemiAngles() const;

    TfSpan<const GfVec3d> SphereSurfaceCenters() const;
    TfSpan<const GfVec3d> SphereSurfaceAxes() const;
    TfSpan<const GfVec3d> SphereSurfaceRefDirections() const;
    TfSpan<const double> SphereSurfaceRadii() const;

    TfSpan<const GfVec3d> TorusSurfaceOrigins() const;
    TfSpan<const GfVec3d> TorusSurfaceAxes() const;
    TfSpan<const GfVec3d> TorusSurfaceRefDirections() const;
    TfSpan<const double> TorusSurfaceMajorRadii() const;
    TfSpan<const double> TorusSurfaceMinorRadii() const;

    TfSpan<const uint32_t> NurbsSurfaceUVertexCounts() const;
    TfSpan<const uint32_t> NurbsSurfaceVVertexCounts() const;
    TfSpan<const uint32_t> NurbsSurfaceUOrders() const;
    TfSpan<const uint32_t> NurbsSurfaceVOrders() const;
    TfSpan<const GfVec3d> NurbsSurfaceControlVertices() const;
    TfSpan<const double> NurbsSurfaceUKnots() const;
    TfSpan<const double> NurbsSurfaceVKnots() const;
    TfSpan<const double> NurbsSurfaceWeights() const;

    // ---- Edge geometry spans ----

    TfSpan<const uint32_t> EdgeNurbsVertexCounts() const;
    TfSpan<const uint32_t> EdgeNurbsOrders() const;
    TfSpan<const GfVec3d> EdgeNurbsControlVertices() const;
    TfSpan<const double> EdgeNurbsKnots() const;
    TfSpan<const double> EdgeNurbsWeights() const;

    TfSpan<const GfVec3d> EdgeCircleCenters() const;
    TfSpan<const GfVec3d> EdgeCircleAxes() const;
    TfSpan<const GfVec3d> EdgeCircleRefDirections() const;
    TfSpan<const double> EdgeCircleRadii() const;

    TfSpan<const GfVec3d> EdgeLineOrigins() const;
    TfSpan<const GfVec3d> EdgeLineDirections() const;

    TfSpan<const GfVec3d> EdgeEllipseCenters() const;
    TfSpan<const GfVec3d> EdgeEllipseAxes() const;
    TfSpan<const GfVec3d> EdgeEllipseRefDirections() const;
    TfSpan<const double> EdgeEllipseXRadii() const;
    TfSpan<const double> EdgeEllipseYRadii() const;

    // ---- WireEdge geometry spans ----

    TfSpan<const uint32_t> WireEdgeNurbsVertexCounts() const;
    TfSpan<const uint32_t> WireEdgeNurbsOrders() const;
    TfSpan<const GfVec3d> WireEdgeNurbsControlVertices() const;
    TfSpan<const double> WireEdgeNurbsKnots() const;
    TfSpan<const double> WireEdgeNurbsWeights() const;

    TfSpan<const GfVec3d> WireEdgeCircleCenters() const;
    TfSpan<const GfVec3d> WireEdgeCircleAxes() const;
    TfSpan<const GfVec3d> WireEdgeCircleRefDirections() const;
    TfSpan<const double> WireEdgeCircleRadii() const;

    TfSpan<const GfVec3d> WireEdgeLineOrigins() const;
    TfSpan<const GfVec3d> WireEdgeLineDirections() const;

    TfSpan<const GfVec3d> WireEdgeEllipseCenters() const;
    TfSpan<const GfVec3d> WireEdgeEllipseAxes() const;
    TfSpan<const GfVec3d> WireEdgeEllipseRefDirections() const;
    TfSpan<const double> WireEdgeEllipseXRadii() const;
    TfSpan<const double> WireEdgeEllipseYRadii() const;

    // ---- Edgeuse UV trim curve spans ----

    TfSpan<const uint32_t> EdgeuseUVNurbsVertexCounts() const;
    TfSpan<const uint32_t> EdgeuseUVNurbsOrders() const;
    TfSpan<const GfVec2d> EdgeuseUVNurbsControlVertices() const;
    TfSpan<const double> EdgeuseUVNurbsKnots() const;
    TfSpan<const double> EdgeuseUVNurbsWeights() const;

private:

    void BuildFaceTypeSubIndices() const;
    void BuildEdgeTypeSubIndices() const;

    const UsdBrepArrayData& m_rArrays;
    UsdBrepArraySpans m_sSpans;

    mutable std::vector<uint32_t> m_vFaceTypeSubIndices;
    mutable std::vector<uint32_t> m_vEdgeTypeSubIndices;
};

/*********************************************************************************************************************
 * UsdBrepIterator: forward iterator over Breps in a UsdBrepArrayData
 ********************************************************************************************************************/
class USD_BREP_EXPORT UsdBrepIterator final
{
public:

    UsdBrepIterator(const UsdBrepArrayData& rArrays, uint32_t iBrepIndex);

    UsdBrepView operator*() const;
    UsdBrepIterator& operator++();
    bool operator!=(const UsdBrepIterator& rOther) const;
    bool operator==(const UsdBrepIterator& rOther) const;

private:

    const UsdBrepArrayData& m_rArrays;
    uint32_t m_iBrepIndex;
};

/*********************************************************************************************************************
 * UsdBrepRange: range adapter enabling range-based for over all Breps in a UsdBrepArrayData
 ********************************************************************************************************************/
class USD_BREP_EXPORT UsdBrepRange final
{
public:

    explicit UsdBrepRange(const UsdBrepArrayData& rArrays);

    UsdBrepIterator begin() const;
    UsdBrepIterator end() const;
    uint32_t size() const;

private:

    const UsdBrepArrayData& m_rArrays;
};

} // end namespace UsdBrep

#endif // _USD_BREP_ITERATOR_H_
