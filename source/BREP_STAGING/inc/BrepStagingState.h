// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_STAGING_STATE_H
#define BREP_STAGING_STATE_H

#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/range2d.h>
#include <pxr/base/gf/range3d.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/vt/array.h>

#include <cstdint>
#include <map>
#include <vector>

using SourceObjectId = uintptr_t;

enum class StagedRegionType
{
    Solid,
    Void
};

struct StagedRegionData
{
    uint32_t uiShellCount = 0;
    StagedRegionType eRegionType = StagedRegionType::Void;
};

enum class StagedShellPointType
{
    None,
    VertexPoint
};

struct StagedShellData
{
    uint32_t uiFaceuseCount = 0;
    uint32_t uiWireEdgeCount = 0;
    StagedShellPointType eShellPointType = StagedShellPointType::None;
};

enum class StagedOrientType
{
    NotSet,
    Same,
    Opposite
};

// Matches USDBREP_NO_OBJECT_INDEX in UsdBrepArrayData.h; kept below UINT32_MAX for debug visibility.
static constexpr uint32_t kStagedNoObjectIndex = 9999999U;

struct StagedFaceuseData
{
    uint32_t uiFaceIndex = kStagedNoObjectIndex;
    StagedOrientType eOrientType = StagedOrientType::NotSet;
};

enum class StagedRadialEntryType
{
    NotSet,
    TopSideEntry,
    BottomSideEntry
};

struct StagedEdgeuseData
{
    uint32_t uiEdgeIndex = kStagedNoObjectIndex;
    StagedOrientType eOrientType = StagedOrientType::NotSet;
    uint32_t uiNextRadialEdgeuseIndex = kStagedNoObjectIndex;
    StagedRadialEntryType eEntrySideType = StagedRadialEntryType::NotSet;
};

enum class StagedCurveType
{
    NotSet,
    NoObject,
    BSplineCurve,
    Line,
    Circle,
    Ellipse
};

struct StagedEdgeData
{
    StagedCurveType eCurveType = StagedCurveType::NotSet;
    pxr::GfRange1d sInterval;
    uint32_t uiStartVertexIndex = kStagedNoObjectIndex;
    uint32_t uiEndVertexIndex = kStagedNoObjectIndex;
};

struct StagedCurveData
{
    StagedCurveType eCurveType = StagedCurveType::NotSet;
    pxr::VtArray<pxr::GfVec3d> vNurbControlVertices;
    uint32_t uiNurbVertexCount = 0;
    uint32_t uiNurbOrder = 0;
    pxr::VtArray<double> vNurbKnots;
    pxr::VtArray<double> vNurbWeights;
    pxr::GfVec3d sOrigin;
    pxr::GfVec3d sDirection;
    pxr::GfVec3d sCenter;
    pxr::GfVec3d sAxis;
    pxr::GfVec3d sRefDirection;
    double dRadius = 0.0;
    double dXRadius = 0.0;
    double dYRadius = 0.0;
};

// Parameter-space (UV) trim curve attached to a single edgeuse. Required to disambiguate the two
// uses of a seam edge on a closed/periodic surface (one at U=Umin, one at U=Umax): without it a
// tessellator that recomputes the trim from the shared 3D seam curve maps both uses to the same
// side and collapses the parameter rectangle. Stored as a 2D NURBS (flat clamped knots), mirroring
// the BrepCurveUvNurbAPI schema. An invalid record means "no UV curve" (the placeholder is emitted).
// Store in edgeuse traversal order: opposite edgeuses run against the 3D edge direction.
struct StagedCurveData2d
{
    bool bValid = false;
    uint32_t uiNurbVertexCount = 0;
    uint32_t uiNurbOrder = 0;
    pxr::VtArray<pxr::GfVec2d> vNurbControlVertices;
    pxr::VtArray<double> vNurbKnots;
    pxr::VtArray<double> vNurbWeights;
};

struct StagedLoopData
{
    uint32_t uiEdgeuseCount = 0;
    uint32_t uiVertexIndex = kStagedNoObjectIndex;
};

enum class StagedVertexType
{
    NotSet,
    None,
    VertexPoint
};

struct StagedVertexData
{
    StagedVertexType eVertexType = StagedVertexType::NotSet;
};

enum class StagedSurfaceType
{
    NotSet,
    BSplineSurface,
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus
};

enum class StagedTrimType
{
    NotSet,
    Rectangular,
    General
};

struct StagedFaceData
{
    uint32_t uiLoopCount = 0;
    StagedSurfaceType eSurfaceType = StagedSurfaceType::NotSet;
    StagedTrimType eTrimType = StagedTrimType::NotSet;
    pxr::GfRange2d sUVDomain;
};

struct StagedSurfaceData
{
    StagedSurfaceType eSurfaceType = StagedSurfaceType::NotSet;
    pxr::VtArray<pxr::GfVec3d> vNurbControlVertices;
    uint32_t uiNurbUVertexCount = 0;
    uint32_t uiNurbVVertexCount = 0;
    uint32_t uiNurbUOrder = 0;
    uint32_t uiNurbVOrder = 0;
    pxr::VtArray<double> vNurbUKnots;
    pxr::VtArray<double> vNurbVKnots;
    pxr::VtArray<double> vNurbWeights;
    pxr::GfVec3d sOrigin;
    pxr::GfVec3d sCenter;
    pxr::GfVec3d sAxis;
    pxr::GfVec3d sRefDirection;
    double dRadius = 0.0;
    double dSemiAngle = 0.0;
    double dMajorRadius = 0.0;
    double dMinorRadius = 0.0;
};

struct StagedBrepData
{
    double dTolerance = 0.0;
    pxr::GfRange3d sExtent;
    std::vector<StagedRegionData> vRegions;
    std::vector<StagedShellData> vShells;
    std::vector<StagedFaceuseData> vFaceuses;
    std::vector<StagedEdgeuseData> vEdgeuses;
    std::vector<StagedEdgeData> vEdges;
    std::vector<StagedEdgeData> vWireEdges;
    std::vector<StagedCurveData> vEdgeCurves;
    std::vector<StagedCurveData> vWireEdgeCurves;
    std::vector<StagedLoopData> vLoops;
    std::vector<StagedVertexData> vVertices;
    std::vector<pxr::GfVec3d> vShellPoints;
    std::vector<pxr::GfVec3d> vVertexPoints;
    std::vector<StagedFaceData> vFaces;
    std::vector<StagedSurfaceData> vFaceSurfaces;
    // UV trim curves keyed by staged edgeuse index (sparse: only edgeuses that carry one).
    std::map<uint32_t, StagedCurveData2d> mEdgeuseCurve2d;

    void ClearCollections()
    {
        vRegions.clear();
        vShells.clear();
        vFaceuses.clear();
        vEdgeuses.clear();
        vEdges.clear();
        vWireEdges.clear();
        vEdgeCurves.clear();
        vWireEdgeCurves.clear();
        vLoops.clear();
        vVertices.clear();
        vShellPoints.clear();
        vVertexPoints.clear();
        vFaces.clear();
        vFaceSurfaces.clear();
        mEdgeuseCurve2d.clear();
    }
};

struct StagedEdgeRadialValues
{
    uint32_t uiRadialHead_EUIndex = kStagedNoObjectIndex;
    uint32_t uiRadialTail_EUIndex = kStagedNoObjectIndex;
};

// Boundary merge case classification
enum ImpliedBoundaryCase
{
    BoundaryCase_None = 0, // face does not need boundary work
    BoundaryCase_StandaloneLoop, // Case 1: no loops touch boundary — add standalone CCW outer loop
    BoundaryCase_AugmentCWLoop, // Case 2: one CW loop touches boundary — augment it with boundary edges
    BoundaryCase_CCWLoopIsOuter, // Case 3: one CCW loop touches boundary — it is already the outer loop
    BoundaryCase_MultiCWMerge, // Case 4: multiple CW loops touch boundary — merge into one CCW outer
    BoundaryCase_MixedError // Case 5: mixed CW/CCW touch boundary — error
};

class BrepStagingState
{
public:

    StagedBrepData& Data()
    {
        return m_sData;
    }

    const StagedBrepData& Data() const
    {
        return m_sData;
    }

    void ClearCollections()
    {
        m_sData.ClearCollections();
    }

    void Clear()
    {
        ClearCollections();
        ClearDedupMaps();
        ClearImpliedBoundarySegments();
        ClearNaturalBoundaryVertexPositions();
        ResetImpliedBoundaryState();
    }

    double GetTolerance() const
    {
        return m_sData.dTolerance;
    }

    void SetTolerance(double dTolerance)
    {
        m_sData.dTolerance = dTolerance;
    }

    pxr::GfRange3d GetExtent() const
    {
        return m_sData.sExtent;
    }

    void SetExtent(const pxr::GfVec3d& rMin, const pxr::GfVec3d& rMax)
    {
        m_sData.sExtent.SetMin(rMin);
        m_sData.sExtent.SetMax(rMax);
    }

    uint32_t GetRegionCount() const
    {
        return static_cast<uint32_t>(m_sData.vRegions.size());
    }

    void ResizeRegionRecords(uint32_t uiRegionCount)
    {
        m_sData.vRegions.resize(uiRegionCount);
    }

    StagedRegionData* GetRegionRecord(uint32_t uiRegionIndex)
    {
        return uiRegionIndex < GetRegionCount() ? &m_sData.vRegions[uiRegionIndex] : nullptr;
    }

    const StagedRegionData* GetRegionRecord(uint32_t uiRegionIndex) const
    {
        return uiRegionIndex < GetRegionCount() ? &m_sData.vRegions[uiRegionIndex] : nullptr;
    }

    uint32_t AddRegionRecord(const StagedRegionData& rRegionData)
    {
        m_sData.vRegions.push_back(rRegionData);
        return GetRegionCount() - 1;
    }

    uint32_t GetShellCount() const
    {
        return static_cast<uint32_t>(m_sData.vShells.size());
    }

    void ResizeShellRecords(uint32_t uiShellCount)
    {
        m_sData.vShells.resize(uiShellCount);
    }

    StagedShellData* GetShellRecord(uint32_t uiShellIndex)
    {
        return uiShellIndex < GetShellCount() ? &m_sData.vShells[uiShellIndex] : nullptr;
    }

    const StagedShellData* GetShellRecord(uint32_t uiShellIndex) const
    {
        return uiShellIndex < GetShellCount() ? &m_sData.vShells[uiShellIndex] : nullptr;
    }

    bool TryGetShellPoint(uint32_t uiShellPointIndex, pxr::GfVec3d& rPoint) const
    {
        if (uiShellPointIndex >= m_sData.vShellPoints.size())
        {
            return false;
        }
        rPoint = m_sData.vShellPoints[uiShellPointIndex];
        return true;
    }

    uint32_t GetFaceuseCount() const
    {
        return static_cast<uint32_t>(m_sData.vFaceuses.size());
    }

    void ResizeFaceuseRecords(uint32_t uiFaceuseCount)
    {
        m_sData.vFaceuses.resize(uiFaceuseCount);
    }

    StagedFaceuseData* GetFaceuseRecord(uint32_t uiFaceuseIndex)
    {
        return uiFaceuseIndex < GetFaceuseCount() ? &m_sData.vFaceuses[uiFaceuseIndex] : nullptr;
    }

    const StagedFaceuseData* GetFaceuseRecord(uint32_t uiFaceuseIndex) const
    {
        return uiFaceuseIndex < GetFaceuseCount() ? &m_sData.vFaceuses[uiFaceuseIndex] : nullptr;
    }

    uint32_t GetFaceCount() const
    {
        return static_cast<uint32_t>(m_sData.vFaces.size());
    }

    StagedFaceData* GetFaceRecord(uint32_t uiFaceIndex)
    {
        return uiFaceIndex < GetFaceCount() ? &m_sData.vFaces[uiFaceIndex] : nullptr;
    }

    const StagedFaceData* GetFaceRecord(uint32_t uiFaceIndex) const
    {
        return uiFaceIndex < GetFaceCount() ? &m_sData.vFaces[uiFaceIndex] : nullptr;
    }

    void AppendFaceRecord(const StagedFaceData& rFaceData)
    {
        m_sData.vFaces.push_back(rFaceData);
    }

    uint32_t GetFaceSurfaceCount() const
    {
        return static_cast<uint32_t>(m_sData.vFaceSurfaces.size());
    }

    StagedSurfaceData* GetFaceSurfaceRecord(uint32_t uiSurfaceIndex)
    {
        return uiSurfaceIndex < GetFaceSurfaceCount() ? &m_sData.vFaceSurfaces[uiSurfaceIndex] : nullptr;
    }

    const StagedSurfaceData* GetFaceSurfaceRecord(uint32_t uiSurfaceIndex) const
    {
        return uiSurfaceIndex < GetFaceSurfaceCount() ? &m_sData.vFaceSurfaces[uiSurfaceIndex] : nullptr;
    }

    void AppendFaceSurfaceRecord(const StagedSurfaceData& rSurfaceData)
    {
        m_sData.vFaceSurfaces.push_back(rSurfaceData);
    }

    uint32_t GetLoopCount() const
    {
        return static_cast<uint32_t>(m_sData.vLoops.size());
    }

    StagedLoopData* GetLoopRecord(uint32_t uiLoopIndex)
    {
        return uiLoopIndex < GetLoopCount() ? &m_sData.vLoops[uiLoopIndex] : nullptr;
    }

    const StagedLoopData* GetLoopRecord(uint32_t uiLoopIndex) const
    {
        return uiLoopIndex < GetLoopCount() ? &m_sData.vLoops[uiLoopIndex] : nullptr;
    }

    void AppendLoopRecord(const StagedLoopData& rLoopData)
    {
        m_sData.vLoops.push_back(rLoopData);
    }

    void InsertLoopRecordAt(uint32_t uiIndex, const StagedLoopData& rLoopData)
    {
        m_sData.vLoops.insert(m_sData.vLoops.begin() + uiIndex, rLoopData);
    }

    uint32_t GetEdgeuseCount() const
    {
        return static_cast<uint32_t>(m_sData.vEdgeuses.size());
    }

    void ResizeEdgeuseRecords(uint32_t uiEdgeuseCount)
    {
        m_sData.vEdgeuses.resize(uiEdgeuseCount);
    }

    StagedEdgeuseData* GetEdgeuseRecord(uint32_t uiEdgeuseIndex)
    {
        return uiEdgeuseIndex < GetEdgeuseCount() ? &m_sData.vEdgeuses[uiEdgeuseIndex] : nullptr;
    }

    const StagedEdgeuseData* GetEdgeuseRecord(uint32_t uiEdgeuseIndex) const
    {
        return uiEdgeuseIndex < GetEdgeuseCount() ? &m_sData.vEdgeuses[uiEdgeuseIndex] : nullptr;
    }

    void AppendEdgeuseRecord(const StagedEdgeuseData& rEdgeuseData)
    {
        m_sData.vEdgeuses.push_back(rEdgeuseData);
    }

    void SetEdgeuseCurve2d(uint32_t uiEdgeuseIndex, const StagedCurveData2d& rCurve)
    {
        m_sData.mEdgeuseCurve2d[uiEdgeuseIndex] = rCurve;
    }

    const StagedCurveData2d* GetEdgeuseCurve2d(uint32_t uiEdgeuseIndex) const
    {
        const auto sIter = m_sData.mEdgeuseCurve2d.find(uiEdgeuseIndex);
        return sIter != m_sData.mEdgeuseCurve2d.end() ? &sIter->second : nullptr;
    }

    uint32_t GetVertexCount() const
    {
        return static_cast<uint32_t>(m_sData.vVertices.size());
    }

    const StagedVertexData* GetVertexRecord(uint32_t uiVertexIndex) const
    {
        return uiVertexIndex < GetVertexCount() ? &m_sData.vVertices[uiVertexIndex] : nullptr;
    }

    void AppendVertexRecord(const StagedVertexData& rVertexData)
    {
        m_sData.vVertices.push_back(rVertexData);
    }

    uint32_t GetVertexPointCount() const
    {
        return static_cast<uint32_t>(m_sData.vVertexPoints.size());
    }

    void AppendVertexPointRecord(const pxr::GfVec3d& rPoint)
    {
        m_sData.vVertexPoints.push_back(rPoint);
    }

    const pxr::GfVec3d* GetVertexPointRecord(uint32_t uiVertexIndex) const
    {
        return uiVertexIndex < GetVertexPointCount() ? &m_sData.vVertexPoints[uiVertexIndex] : nullptr;
    }

    pxr::GfVec3d* GetVertexPointRecord(uint32_t uiVertexIndex)
    {
        return uiVertexIndex < GetVertexPointCount() ? &m_sData.vVertexPoints[uiVertexIndex] : nullptr;
    }

    uint32_t GetEdgeCount() const
    {
        return static_cast<uint32_t>(m_sData.vEdges.size());
    }

    StagedEdgeData* GetEdgeRecord(uint32_t uiEdgeIndex)
    {
        return uiEdgeIndex < GetEdgeCount() ? &m_sData.vEdges[uiEdgeIndex] : nullptr;
    }

    const StagedEdgeData* GetEdgeRecord(uint32_t uiEdgeIndex) const
    {
        return uiEdgeIndex < GetEdgeCount() ? &m_sData.vEdges[uiEdgeIndex] : nullptr;
    }

    void AppendEdgeRecord(const StagedEdgeData& rEdgeData)
    {
        m_sData.vEdges.push_back(rEdgeData);
    }

    uint32_t GetWireEdgeCount() const
    {
        return static_cast<uint32_t>(m_sData.vWireEdges.size());
    }

    StagedEdgeData* GetWireEdgeRecord(uint32_t uiEdgeIndex)
    {
        return uiEdgeIndex < GetWireEdgeCount() ? &m_sData.vWireEdges[uiEdgeIndex] : nullptr;
    }

    const StagedEdgeData* GetWireEdgeRecord(uint32_t uiEdgeIndex) const
    {
        return uiEdgeIndex < GetWireEdgeCount() ? &m_sData.vWireEdges[uiEdgeIndex] : nullptr;
    }

    uint32_t GetEdgeCurveCount() const
    {
        return static_cast<uint32_t>(m_sData.vEdgeCurves.size());
    }

    StagedCurveData* GetEdgeCurveRecord(uint32_t uiCurveIndex)
    {
        return uiCurveIndex < GetEdgeCurveCount() ? &m_sData.vEdgeCurves[uiCurveIndex] : nullptr;
    }

    const StagedCurveData* GetEdgeCurveRecord(uint32_t uiCurveIndex) const
    {
        return uiCurveIndex < GetEdgeCurveCount() ? &m_sData.vEdgeCurves[uiCurveIndex] : nullptr;
    }

    void AppendEdgeCurveRecord(const StagedCurveData& rCurveData)
    {
        m_sData.vEdgeCurves.push_back(rCurveData);
    }

    uint32_t GetWireEdgeCurveCount() const
    {
        return static_cast<uint32_t>(m_sData.vWireEdgeCurves.size());
    }

    StagedCurveData* GetWireEdgeCurveRecord(uint32_t uiCurveIndex)
    {
        return uiCurveIndex < m_sData.vWireEdgeCurves.size() ? &m_sData.vWireEdgeCurves[uiCurveIndex] : nullptr;
    }

    const StagedCurveData* GetWireEdgeCurveRecord(uint32_t uiCurveIndex) const
    {
        return uiCurveIndex < m_sData.vWireEdgeCurves.size() ? &m_sData.vWireEdgeCurves[uiCurveIndex] : nullptr;
    }

    void ClearDedupMaps()
    {
        m_sMapSourceVertex_VertexIndex.clear();
        m_sMapSourceEdge_EdgeIndex.clear();
        m_sMapEdgeIndex_RadialEUIndices.clear();
    }

    bool TryGetEdgeIndexForSourceEdge(SourceObjectId uiSourceEdgeId, uint32_t& rEdgeIndex) const
    {
        const auto sEdgeIter = m_sMapSourceEdge_EdgeIndex.find(uiSourceEdgeId);
        if (sEdgeIter == m_sMapSourceEdge_EdgeIndex.end())
        {
            return false;
        }

        rEdgeIndex = sEdgeIter->second;
        return true;
    }

    void RegisterEdgeForSourceEdge(SourceObjectId uiSourceEdgeId, uint32_t uiEdgeIndex)
    {
        m_sMapSourceEdge_EdgeIndex[uiSourceEdgeId] = uiEdgeIndex;
    }

    bool TryGetVertexIndexForSourceVertex(SourceObjectId uiSourceVertexId, uint32_t& rVertexIndex) const
    {
        const auto sVertexIter = m_sMapSourceVertex_VertexIndex.find(uiSourceVertexId);
        if (sVertexIter == m_sMapSourceVertex_VertexIndex.end())
        {
            return false;
        }

        rVertexIndex = sVertexIter->second;
        return true;
    }

    void RegisterVertexForSourceVertex(SourceObjectId uiSourceVertexId, uint32_t uiVertexIndex)
    {
        m_sMapSourceVertex_VertexIndex[uiSourceVertexId] = uiVertexIndex;
    }

    StagedEdgeRadialValues* GetEdgeRadialList(uint32_t uiEdgeIndex)
    {
        auto sEdgeValuesIter = m_sMapEdgeIndex_RadialEUIndices.find(uiEdgeIndex);
        return sEdgeValuesIter != m_sMapEdgeIndex_RadialEUIndices.end() ? &sEdgeValuesIter->second : nullptr;
    }

    const StagedEdgeRadialValues* GetEdgeRadialList(uint32_t uiEdgeIndex) const
    {
        auto sEdgeValuesIter = m_sMapEdgeIndex_RadialEUIndices.find(uiEdgeIndex);
        return sEdgeValuesIter != m_sMapEdgeIndex_RadialEUIndices.end() ? &sEdgeValuesIter->second : nullptr;
    }

    void SetEdgeRadialList(uint32_t uiEdgeIndex, uint32_t uiRadialHeadEdgeuseIndex, uint32_t uiRadialTailEdgeuseIndex)
    {
        m_sMapEdgeIndex_RadialEUIndices[uiEdgeIndex] = StagedEdgeRadialValues{ uiRadialHeadEdgeuseIndex, uiRadialTailEdgeuseIndex };
    }

    bool HasEdgeRadialList(uint32_t uiEdgeIndex) const
    {
        return m_sMapEdgeIndex_RadialEUIndices.find(uiEdgeIndex) != m_sMapEdgeIndex_RadialEUIndices.end();
    }

    void AppendImpliedBoundarySegment(const StagedEdgeuseData& rEdgeuseData)
    {
        m_vImpliedBoundarySegments.push_back(rEdgeuseData);
    }

    void ClearImpliedBoundarySegments()
    {
        m_vImpliedBoundarySegments.clear();
    }

    bool HasImpliedBoundarySegments() const
    {
        return !m_vImpliedBoundarySegments.empty();
    }

    uint32_t GetImpliedBoundarySegmentCount() const
    {
        return static_cast<uint32_t>(m_vImpliedBoundarySegments.size());
    }

    const StagedEdgeuseData* GetImpliedBoundarySegment(uint32_t uiSegmentIndex) const
    {
        return uiSegmentIndex < GetImpliedBoundarySegmentCount() ? &m_vImpliedBoundarySegments[uiSegmentIndex] : nullptr;
    }

    void ClearNaturalBoundaryVertexPositions()
    {
        m_vNaturalBoundaryVertexPositions.clear();
    }

    void AppendNaturalBoundaryVertexPosition(const pxr::GfVec3d& rPosition)
    {
        m_vNaturalBoundaryVertexPositions.push_back(rPosition);
    }

    const std::vector<pxr::GfVec3d>& GetNaturalBoundaryVertexPositions() const
    {
        return m_vNaturalBoundaryVertexPositions;
    }

    void ResetImpliedBoundaryState()
    {
        m_eImpliedBoundaryCase = BoundaryCase_None;
        m_uiAugmentSourceLoopIndex = UINT32_MAX;
        m_uiImpliedBoundaryContactVertexIndex = UINT32_MAX;
    }

    void SetImpliedBoundaryCase(ImpliedBoundaryCase eCase)
    {
        m_eImpliedBoundaryCase = eCase;
    }

    ImpliedBoundaryCase GetImpliedBoundaryCase() const
    {
        return m_eImpliedBoundaryCase;
    }

    void SetImpliedBoundaryAugmentTarget(uint32_t uiSourceLoopIndex, uint32_t uiContactVertexIndex)
    {
        m_uiAugmentSourceLoopIndex = uiSourceLoopIndex;
        m_uiImpliedBoundaryContactVertexIndex = uiContactVertexIndex;
    }

    uint32_t GetImpliedBoundaryAugmentSourceLoopIndex() const
    {
        return m_uiAugmentSourceLoopIndex;
    }

    uint32_t GetImpliedBoundaryContactVertexIndex() const
    {
        return m_uiImpliedBoundaryContactVertexIndex;
    }

private:

    StagedBrepData m_sData;
    std::map<uint32_t, StagedEdgeRadialValues> m_sMapEdgeIndex_RadialEUIndices;
    std::map<SourceObjectId, uint32_t> m_sMapSourceVertex_VertexIndex;
    std::map<SourceObjectId, uint32_t> m_sMapSourceEdge_EdgeIndex;
    std::vector<pxr::GfVec3d> m_vNaturalBoundaryVertexPositions;
    std::vector<StagedEdgeuseData> m_vImpliedBoundarySegments;
    uint32_t m_uiImpliedBoundaryContactVertexIndex = UINT32_MAX;
    ImpliedBoundaryCase m_eImpliedBoundaryCase = BoundaryCase_None;
    uint32_t m_uiAugmentSourceLoopIndex = UINT32_MAX;
};

#endif // BREP_STAGING_STATE_H
