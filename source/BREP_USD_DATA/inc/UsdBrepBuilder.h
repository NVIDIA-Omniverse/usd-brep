// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef _USD_BREP_BUILDER_H_
#define _USD_BREP_BUILDER_H_

#include "UsdBrepArrayData.h"
#include "UsdBrepConfig.h"

#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/range2d.h>
#include <pxr/base/gf/range3d.h>

namespace UsdBrepData
{

class USDBREP_EXPORT UsdBrepBuilder final
{
public:

    UsdBrepBuilder() = default;
    UsdBrepBuilder(UsdBrepArrayData& rArrays, UsdBrepArraySpans& rSpans);
    ~UsdBrepBuilder() noexcept = default;

    UsdBrepBuilder(const UsdBrepBuilder&) = delete;
    UsdBrepBuilder& operator=(const UsdBrepBuilder&) = delete;

    UsdBrepArrayData& Arrays();
    const UsdBrepArrayData& Arrays() const;

    UsdBrepArraySpans& Spans();
    const UsdBrepArraySpans& Spans() const;

    void Reset();

    // Reserve capacity for the fixed-size-per-entity arrays when the totals are known up front. These avoid the
    // incremental reallocations that would otherwise occur while appending entities one at a time. Variable-size
    // geometry arrays (per-type curve/surface data) are intentionally left to grow on demand.
    void ReserveRegions(uint32_t uiRegionCount);
    void ReserveShells(uint32_t uiShellCount);
    void ReserveFaceuses(uint32_t uiFaceuseCount);
    void ReserveFaces(uint32_t uiFaceCount);
    void ReserveLoops(uint32_t uiLoopCount);
    void ReserveEdgeuses(uint32_t uiEdgeuseCount);
    void ReserveEdges(uint32_t uiEdgeCount);
    void ReserveWireEdges(uint32_t uiWireEdgeCount);
    void ReserveVertices(uint32_t uiVertexCount, uint32_t uiVertexPointCount);

    bool BeginBrep(double dXSectTol3d, const pxr::GfRange3d& rBrepExtent3d, uint32_t uiRegionCount);
    void FinishBrep(bool bKeepCounts = false);

    void AppendRegion(uint32_t uiShellCount, const pxr::TfToken& rRegionType);
    void AppendShell(uint32_t uiFaceuseCount, uint32_t uiWireEdgeCount, const pxr::TfToken& rShellPointType);
    void AppendShellPoint(const pxr::GfVec3d& rPoint);
    void AppendFaceuse(uint32_t uiGlobalFaceIndex, const pxr::TfToken& rOrientationType);
    void AppendFace(uint32_t uiLoopCount, const pxr::TfToken& rSurfaceType, const pxr::TfToken& rTrimType, const pxr::GfRange2d& rUVDomain);
    void AppendLoop(uint32_t uiEdgeuseCount, uint32_t uiGlobalVertexIndex);
    void AppendEdgeuse(
        uint32_t uiGlobalEdgeIndex,
        const pxr::TfToken& rOrientationType,
        uint32_t uiGlobalNextRadialEUIndex,
        const pxr::TfToken& rThisRadialEntryType
    );
    void AppendEdge(const pxr::TfToken& rCurveType, const pxr::GfRange1d& rCurveRange, const pxr::GfVec2i& rGlobalVertexIndices);
    void AppendWireEdge(const pxr::TfToken& rCurveType, const pxr::GfRange1d& rCurveRange, const pxr::GfVec2i& rGlobalVertexIndices);
    void AppendVertex(const pxr::TfToken& rPointType);
    void AppendVertexPoint(const pxr::GfVec3d& rPoint);

    void AppendEdgeNurbCurve(
        uint32_t uiVertexCount,
        uint32_t uiOrder,
        const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
        const pxr::VtArray<double>& rWeights,
        const pxr::VtArray<double>& rKnots
    );
    void AppendEdgeCircleCurve(const pxr::GfVec3d& rCenter, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius);
    void AppendEdgeLineCurve(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rDirection);
    void AppendEdgeEllipseCurve(
        const pxr::GfVec3d& rCenter,
        const pxr::GfVec3d& rAxis,
        const pxr::GfVec3d& rRefDirection,
        double dXRadius,
        double dYRadius
    );

    void AppendWireEdgeNurbCurve(
        uint32_t uiVertexCount,
        uint32_t uiOrder,
        const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
        const pxr::VtArray<double>& rWeights,
        const pxr::VtArray<double>& rKnots
    );
    void AppendWireEdgeCircleCurve(const pxr::GfVec3d& rCenter, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius);
    void AppendWireEdgeLineCurve(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rDirection);
    void AppendWireEdgeEllipseCurve(
        const pxr::GfVec3d& rCenter,
        const pxr::GfVec3d& rAxis,
        const pxr::GfVec3d& rRefDirection,
        double dXRadius,
        double dYRadius
    );

    void AppendEdgeuseNurbCurve(
        uint32_t uiVertexCount,
        uint32_t uiOrder,
        const pxr::VtArray<pxr::GfVec2d>& rControlVertices,
        const pxr::VtArray<double>& rWeights,
        const pxr::VtArray<double>& rKnots
    );

    void AppendFaceNurbSurface(
        uint32_t uiUVertexCount,
        uint32_t uiVVertexCount,
        uint32_t uiUOrder,
        uint32_t uiVOrder,
        const pxr::VtArray<pxr::GfVec3d>& rControlVertices,
        const pxr::VtArray<double>& rWeights,
        const pxr::VtArray<double>& rUKnots,
        const pxr::VtArray<double>& rVKnots
    );
    void AppendFaceSphereSurface(const pxr::GfVec3d& rCenter, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius);
    void AppendFacePlaneSurface(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection);
    void AppendFaceCylinderSurface(const pxr::GfVec3d& rOrigin, const pxr::GfVec3d& rAxis, const pxr::GfVec3d& rRefDirection, double dRadius);
    void AppendFaceConeSurface(
        const pxr::GfVec3d& rOrigin,
        const pxr::GfVec3d& rAxis,
        const pxr::GfVec3d& rRefDirection,
        double dRadius,
        double dSemiAngle
    );
    void AppendFaceTorusSurface(
        const pxr::GfVec3d& rOrigin,
        const pxr::GfVec3d& rAxis,
        const pxr::GfVec3d& rRefDirection,
        double dMajorRadius,
        double dMinorRadius
    );

private:

    template <typename T>
    void AppendArray(pxr::VtArray<T>& rTarget, const pxr::VtArray<T>& rSource)
    {
        rTarget.insert(rTarget.end(), rSource.begin(), rSource.end());
    }

    template <typename T>
    static void ReserveArray(pxr::VtArray<T>& rTarget, size_t uiAdditionalCount)
    {
        rTarget.reserve(rTarget.size() + uiAdditionalCount);
    }

    UsdBrepArrayData m_sOwnedArrays;
    UsdBrepArraySpans m_sOwnedSpans;
    UsdBrepArrayData* m_pArrays = &m_sOwnedArrays;
    UsdBrepArraySpans* m_pSpans = &m_sOwnedSpans;
};

} // namespace UsdBrepData

#endif // _USD_BREP_BUILDER_H_
