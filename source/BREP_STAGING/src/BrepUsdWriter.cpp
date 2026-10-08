// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepUsdWriter.h"

#include "UsdBrepArrayData.h"
#include "UsdBrepBuilder.h"
#include "UsdBrepDiagnostics.h"
#include "UsdBrepTokens.h"
#include "UsdBrepUtilities.h"
#include "UsdBrepWrite.h"

#include <pxr/base/gf/range2d.h>
#include <pxr/base/gf/vec2i.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/dictionary.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/usd/prim.h>

#include <algorithm>
#include <cassert>
#include <string>

/****************************************************************************************************************************************************
 * Staged BREP Integration Methods - Using UsdBrep System
 *
 ***************************************************************************************************************************************************/
double StagedBrepUsdWriter::GetStagedBrepTolerance() const
{
    return m_rState.GetTolerance();
}

pxr::GfRange3d StagedBrepUsdWriter::GetStagedBrepExtent() const
{
    return m_rState.GetExtent();
}

uint32_t StagedBrepUsdWriter::GetStagedRegionCount() const
{
    return m_rState.GetRegionCount();
}

StagedRegionType StagedBrepUsdWriter::GetStagedRegionType(uint32_t uiRegionIndex) const
{
    const StagedRegionData* pRegionData = m_rState.GetRegionRecord(uiRegionIndex);
    return pRegionData ? pRegionData->eRegionType : StagedRegionType::Void;
}

uint32_t StagedBrepUsdWriter::GetStagedRegionShellCount(uint32_t uiRegionIndex) const
{
    const StagedRegionData* pRegionData = m_rState.GetRegionRecord(uiRegionIndex);
    return pRegionData ? pRegionData->uiShellCount : 0;
}

uint32_t StagedBrepUsdWriter::GetStagedShellCount() const
{
    return m_rState.GetShellCount();
}

uint32_t StagedBrepUsdWriter::GetStagedShellFaceuseCount(uint32_t uiShellIndex) const
{
    const StagedShellData* pShellData = m_rState.GetShellRecord(uiShellIndex);
    return pShellData ? pShellData->uiFaceuseCount : 0;
}

uint32_t StagedBrepUsdWriter::GetStagedShellWireEdgeCount(uint32_t uiShellIndex) const
{
    const StagedShellData* pShellData = m_rState.GetShellRecord(uiShellIndex);
    return pShellData ? pShellData->uiWireEdgeCount : 0;
}

StagedShellPointType StagedBrepUsdWriter::GetStagedShellPointType(uint32_t uiShellIndex) const
{
    const StagedShellData* pShellData = m_rState.GetShellRecord(uiShellIndex);
    return pShellData ? pShellData->eShellPointType : StagedShellPointType::None;
}

bool StagedBrepUsdWriter::TryGetStagedShellPoint(uint32_t uiShellPointIndex, pxr::GfVec3d& rPoint) const
{
    return m_rState.TryGetShellPoint(uiShellPointIndex, rPoint);
}

uint32_t StagedBrepUsdWriter::GetStagedFaceuseCount() const
{
    return m_rState.GetFaceuseCount();
}

uint32_t StagedBrepUsdWriter::GetStagedFaceuseFaceIndex(uint32_t uiFaceuseIndex) const
{
    const StagedFaceuseData* pFaceuseData = m_rState.GetFaceuseRecord(uiFaceuseIndex);
    return pFaceuseData ? pFaceuseData->uiFaceIndex : static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
}

StagedOrientType StagedBrepUsdWriter::GetStagedFaceuseOrientation(uint32_t uiFaceuseIndex) const
{
    const StagedFaceuseData* pFaceuseData = m_rState.GetFaceuseRecord(uiFaceuseIndex);
    return pFaceuseData ? pFaceuseData->eOrientType : StagedOrientType::NotSet;
}

uint32_t StagedBrepUsdWriter::GetStagedFaceCount() const
{
    return m_rState.GetFaceCount();
}

StagedSurfaceType StagedBrepUsdWriter::GetStagedFaceSurfaceType(uint32_t uiFaceIndex) const
{
    const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiFaceIndex);
    return pFaceData ? pFaceData->eSurfaceType : StagedSurfaceType::NotSet;
}

StagedTrimType StagedBrepUsdWriter::GetStagedFaceTrimType(uint32_t uiFaceIndex) const
{
    const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiFaceIndex);
    return pFaceData ? pFaceData->eTrimType : StagedTrimType::NotSet;
}

uint32_t StagedBrepUsdWriter::GetStagedFaceLoopCount(uint32_t uiFaceIndex) const
{
    const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiFaceIndex);
    return pFaceData ? pFaceData->uiLoopCount : 0;
}

bool StagedBrepUsdWriter::TryGetStagedFaceDomain(uint32_t uiFaceIndex, pxr::GfRange2d& rUVDomain) const
{
    const StagedFaceData* pFaceData = m_rState.GetFaceRecord(uiFaceIndex);
    if (!pFaceData)
    {
        return false;
    }

    rUVDomain = pFaceData->sUVDomain;
    return true;
}

const StagedSurfaceData* StagedBrepUsdWriter::GetStagedFaceSurfaceRecord(uint32_t uiSurfaceIndex) const
{
    return m_rState.GetFaceSurfaceRecord(uiSurfaceIndex);
}

uint32_t StagedBrepUsdWriter::GetStagedLoopCount() const
{
    return m_rState.GetLoopCount();
}

uint32_t StagedBrepUsdWriter::GetStagedLoopEdgeuseCount(uint32_t uiLoopIndex) const
{
    const StagedLoopData* pLoopData = m_rState.GetLoopRecord(uiLoopIndex);
    return pLoopData ? pLoopData->uiEdgeuseCount : 0;
}

uint32_t StagedBrepUsdWriter::GetStagedLoopVertexIndex(uint32_t uiLoopIndex) const
{
    const StagedLoopData* pLoopData = m_rState.GetLoopRecord(uiLoopIndex);
    return pLoopData ? pLoopData->uiVertexIndex : static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
}

uint32_t StagedBrepUsdWriter::GetStagedEdgeuseCount() const
{
    return m_rState.GetEdgeuseCount();
}

StagedEdgeuseData StagedBrepUsdWriter::GetStagedEdgeuse(uint32_t uiEdgeuseIndex) const
{
    const StagedEdgeuseData* pEdgeuseData = m_rState.GetEdgeuseRecord(uiEdgeuseIndex);
    return pEdgeuseData ? *pEdgeuseData : StagedEdgeuseData();
}

const StagedCurveData2d* StagedBrepUsdWriter::GetStagedEdgeuseCurve2d(uint32_t uiEdgeuseIndex) const
{
    return m_rState.GetEdgeuseCurve2d(uiEdgeuseIndex);
}

uint32_t StagedBrepUsdWriter::GetStagedEdgeCount() const
{
    return m_rState.GetEdgeCount();
}

StagedCurveType StagedBrepUsdWriter::GetStagedEdgeCurveType(uint32_t uiEdgeIndex) const
{
    const StagedEdgeData* pEdgeData = m_rState.GetEdgeRecord(uiEdgeIndex);
    return pEdgeData ? pEdgeData->eCurveType : StagedCurveType::NotSet;
}

pxr::GfRange1d StagedBrepUsdWriter::GetStagedEdgeInterval(uint32_t uiEdgeIndex) const
{
    const StagedEdgeData* pEdgeData = m_rState.GetEdgeRecord(uiEdgeIndex);
    return pEdgeData ? pEdgeData->sInterval : pxr::GfRange1d();
}

uint32_t StagedBrepUsdWriter::GetStagedEdgeStartVertexIndex(uint32_t uiEdgeIndex) const
{
    const StagedEdgeData* pEdgeData = m_rState.GetEdgeRecord(uiEdgeIndex);
    return pEdgeData ? pEdgeData->uiStartVertexIndex : static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
}

uint32_t StagedBrepUsdWriter::GetStagedEdgeEndVertexIndex(uint32_t uiEdgeIndex) const
{
    const StagedEdgeData* pEdgeData = m_rState.GetEdgeRecord(uiEdgeIndex);
    return pEdgeData ? pEdgeData->uiEndVertexIndex : static_cast<uint32_t>(USDBREP_NO_OBJECT_INDEX);
}

const StagedCurveData* StagedBrepUsdWriter::GetStagedEdgeCurveRecord(uint32_t uiCurveIndex) const
{
    return m_rState.GetEdgeCurveRecord(uiCurveIndex);
}

uint32_t StagedBrepUsdWriter::GetStagedWireEdgeCount() const
{
    return m_rState.GetWireEdgeCount();
}

const StagedEdgeData* StagedBrepUsdWriter::GetStagedWireEdgeRecord(uint32_t uiEdgeIndex) const
{
    return m_rState.GetWireEdgeRecord(uiEdgeIndex);
}

const StagedCurveData* StagedBrepUsdWriter::GetStagedWireEdgeCurveRecord(uint32_t uiCurveIndex) const
{
    return m_rState.GetWireEdgeCurveRecord(uiCurveIndex);
}

uint32_t StagedBrepUsdWriter::GetStagedVertexCount() const
{
    return m_rState.GetVertexCount();
}

uint32_t StagedBrepUsdWriter::GetStagedVertexPointCount() const
{
    return m_rState.GetVertexPointCount();
}

StagedVertexType StagedBrepUsdWriter::GetStagedVertexType(uint32_t uiVertexIndex) const
{
    const StagedVertexData* pVertexData = m_rState.GetVertexRecord(uiVertexIndex);
    return pVertexData ? pVertexData->eVertexType : StagedVertexType::NotSet;
}

bool StagedBrepUsdWriter::TryGetStagedVertexPoint(uint32_t uiVertexIndex, pxr::GfVec3d& rPoint) const
{
    const pxr::GfVec3d* pPoint = m_rState.GetVertexPointRecord(uiVertexIndex);
    if (!pPoint)
    {
        return false;
    }

    rPoint = *pPoint;
    return true;
}

bool StagedBrepUsdWriter::PopulateDirectUsdBrepMetadataFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{
    return rBuilder.BeginBrep(GetStagedBrepTolerance(), GetStagedBrepExtent(), GetStagedRegionCount());
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepRegionsFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiRegionCount = GetStagedRegionCount();
    rBuilder.ReserveRegions(uiRegionCount);
    for (uint32_t ii = 0; ii < uiRegionCount; ++ii)
    {
        StagedRegionType eRegionType = GetStagedRegionType(ii);
        pxr::TfToken tRegionType = (eRegionType == StagedRegionType::Void)  ? UsdBrepData::UsdBrepSolidTokens->voidRegion :
                                   (eRegionType == StagedRegionType::Solid) ? UsdBrepData::UsdBrepSolidTokens->solidRegion :
                                                                              UsdBrepData::UsdBrepSolidTokens->none;
        rBuilder.AppendRegion(GetStagedRegionShellCount(ii), tRegionType);
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepShellsFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiShellCount = GetStagedShellCount();
    rBuilder.ReserveShells(uiShellCount);
    for (uint32_t ii = 0; ii < uiShellCount; ++ii)
    {
        StagedShellPointType eShellPointType = GetStagedShellPointType(ii);
        pxr::TfToken tShellPointType = (eShellPointType == StagedShellPointType::VertexPoint) ? UsdBrepData::UsdBrepSolidTokens->brepPointAPI :
                                                                                                UsdBrepData::UsdBrepSolidTokens->none;

        rBuilder.AppendShell(GetStagedShellFaceuseCount(ii), GetStagedShellWireEdgeCount(ii), tShellPointType);

        if (tShellPointType == UsdBrepData::UsdBrepSolidTokens->brepPointAPI)
        {
            pxr::GfVec3d sPoint;
            if (!TryGetStagedShellPoint(ii, sPoint))
            {
                return false;
            }
            rBuilder.AppendShellPoint(sPoint);
        }
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepFaceusesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiFaceuseCount = GetStagedFaceuseCount();
    rBuilder.ReserveFaceuses(uiFaceuseCount);
    for (uint32_t ii = 0; ii < uiFaceuseCount; ++ii)
    {
        const uint32_t uiFaceIndex = GetStagedFaceuseFaceIndex(ii);
        if (uiFaceIndex == USDBREP_NO_OBJECT_INDEX)
        {
            return false;
        }

        StagedOrientType eOrientation = GetStagedFaceuseOrientation(ii);
        if (eOrientation == StagedOrientType::NotSet)
        {
            return false;
        }

        pxr::TfToken tOrientationType = eOrientation == StagedOrientType::Same     ? UsdBrepData::UsdBrepSolidTokens->same :
                                        eOrientation == StagedOrientType::Opposite ? UsdBrepData::UsdBrepSolidTokens->opposite :
                                                                                     UsdBrepData::UsdBrepSolidTokens->none;
        rBuilder.AppendFaceuse(rBuilder.Arrays().MapLocalToGlobal(uiFaceIndex, rBuilder.Spans().m_lFaceStartIndex), tOrientationType);
    }

    // Direct UsdBrep export emits paired in/out faceuses for every staged face.
    rBuilder.Spans().m_lFaceCount = rBuilder.Spans().m_lFaceuseCount / 2;
    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepFaceSurfaceFromStagedData(
    UsdBrepData::UsdBrepBuilder& rBuilder,
    uint32_t uiFaceIndex,
    const pxr::TfToken& rSurfaceType
) const
{

    const StagedSurfaceData* pSurface = GetStagedFaceSurfaceRecord(uiFaceIndex);
    if (!pSurface)
    {
        return false;
    }

    if (rSurfaceType == UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
    {
        rBuilder.Spans().m_lFaceSphereSurface_Count++;
        const StagedSurfaceData* pSphere = (pSurface->eSurfaceType == StagedSurfaceType::Sphere) ? pSurface : nullptr;
        if (!pSphere)
        {
            return false;
        }
        rBuilder.AppendFaceSphereSurface(pSphere->sCenter, pSphere->sAxis, pSphere->sRefDirection, pSphere->dRadius);
    }
    else if (rSurfaceType == UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
    {
        rBuilder.Spans().m_lFaceBSplineSurface_Count++;
        const StagedSurfaceData* pSurfaceData = (pSurface->eSurfaceType == StagedSurfaceType::BSplineSurface) ? pSurface : nullptr;
        if (!pSurfaceData)
        {
            return false;
        }
        rBuilder.AppendFaceNurbSurface(
            pSurfaceData->uiNurbUVertexCount,
            pSurfaceData->uiNurbVVertexCount,
            pSurfaceData->uiNurbUOrder,
            pSurfaceData->uiNurbVOrder,
            pSurfaceData->vNurbControlVertices,
            pSurfaceData->vNurbWeights,
            pSurfaceData->vNurbUKnots,
            pSurfaceData->vNurbVKnots
        );
    }
    else if (rSurfaceType == UsdBrepData::UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
    {
        rBuilder.Spans().m_lFacePlaneSurface_Count++;
        const StagedSurfaceData* pPlane = (pSurface->eSurfaceType == StagedSurfaceType::Plane) ? pSurface : nullptr;
        if (!pPlane)
        {
            return false;
        }
        rBuilder.AppendFacePlaneSurface(pPlane->sOrigin, pPlane->sAxis, pPlane->sRefDirection);
    }
    else if (rSurfaceType == UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
    {
        rBuilder.Spans().m_lFaceCylinderSurface_Count++;
        const StagedSurfaceData* pCylinder = (pSurface->eSurfaceType == StagedSurfaceType::Cylinder) ? pSurface : nullptr;
        if (!pCylinder)
        {
            return false;
        }
        rBuilder.AppendFaceCylinderSurface(pCylinder->sOrigin, pCylinder->sAxis, pCylinder->sRefDirection, pCylinder->dRadius);
    }
    else if (rSurfaceType == UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceConeAPI)
    {
        rBuilder.Spans().m_lFaceConeSurface_Count++;
        const StagedSurfaceData* pCone = (pSurface->eSurfaceType == StagedSurfaceType::Cone) ? pSurface : nullptr;
        if (!pCone)
        {
            return false;
        }
        rBuilder.AppendFaceConeSurface(pCone->sOrigin, pCone->sAxis, pCone->sRefDirection, pCone->dRadius, pCone->dSemiAngle);
    }
    else if (rSurfaceType == UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
    {
        rBuilder.Spans().m_lFaceTorusSurface_Count++;
        const StagedSurfaceData* pTorus = (pSurface->eSurfaceType == StagedSurfaceType::Torus) ? pSurface : nullptr;
        if (!pTorus)
        {
            return false;
        }
        rBuilder.AppendFaceTorusSurface(pTorus->sOrigin, pTorus->sAxis, pTorus->sRefDirection, pTorus->dMajorRadius, pTorus->dMinorRadius);
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepFacesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiFaceCount = GetStagedFaceCount();
    rBuilder.ReserveFaces(uiFaceCount);
    for (uint32_t ii = 0; ii < uiFaceCount; ++ii)
    {
        StagedSurfaceType eSurfaceType = GetStagedFaceSurfaceType(ii);
        pxr::TfToken tSurfaceType;
        switch (eSurfaceType)
        {
            case StagedSurfaceType::Plane:
                tSurfaceType = UsdBrepData::UsdBrepSurfaceTokens->brepSurfacePlaneAPI;
                break;
            case StagedSurfaceType::Cylinder:
                tSurfaceType = UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceCylinderAPI;
                break;
            case StagedSurfaceType::Cone:
                tSurfaceType = UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceConeAPI;
                break;
            case StagedSurfaceType::Sphere:
                tSurfaceType = UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceSphereAPI;
                break;
            case StagedSurfaceType::Torus:
                tSurfaceType = UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceTorusAPI;
                break;
            default:
                tSurfaceType = UsdBrepData::UsdBrepSurfaceTokens->brepSurfaceNurbAPI;
                break;
        }

        StagedTrimType eTrimType = GetStagedFaceTrimType(ii);
        pxr::TfToken tTrimType = eTrimType == StagedTrimType::Rectangular ? UsdBrepData::UsdBrepSolidTokens->rectangular :
                                 eTrimType == StagedTrimType::General     ? UsdBrepData::UsdBrepSolidTokens->general :
                                                                            UsdBrepData::UsdBrepSolidTokens->none;

        pxr::GfRange2d sDomain;
        if (!TryGetStagedFaceDomain(ii, sDomain))
        {
            return false;
        }

        rBuilder.AppendFace(GetStagedFaceLoopCount(ii), tSurfaceType, tTrimType, sDomain);
        if (!AppendDirectUsdBrepFaceSurfaceFromStagedData(rBuilder, ii, tSurfaceType))
        {
            return false;
        }
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepLoopsFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    // Tracks which staged vertices have been counted while building the UsdBrep vertex set. Loops (isolated vertices),
    // edges, and wire edges all reference staged vertex indices, every one of which is < GetStagedVertexCount().
    rBuilder.Spans().m_bProcessedVertices.clear();
    rBuilder.Spans().m_bProcessedVertices.resize(GetStagedVertexCount(), false);

    const uint32_t uiLoopCount = GetStagedLoopCount();
    rBuilder.ReserveLoops(uiLoopCount);
    for (uint32_t ii = 0; ii < uiLoopCount; ++ii)
    {
        const uint32_t uiEdgeuseCount = GetStagedLoopEdgeuseCount(ii);
        const uint32_t uiVertexIndex = GetStagedLoopVertexIndex(ii);
        const uint32_t uiGlobalVertexIndex = uiEdgeuseCount > 0 ?
                                                 0 :
                                                 rBuilder.Arrays().MapLocalToGlobal(uiVertexIndex, rBuilder.Spans().m_lVertexStartIndex);

        rBuilder.AppendLoop(uiEdgeuseCount, uiGlobalVertexIndex);

        if (uiEdgeuseCount == 0)
        {
            if (uiVertexIndex >= rBuilder.Spans().m_bProcessedVertices.size())
            {
                return false;
            }
            if (!rBuilder.Spans().m_bProcessedVertices[uiVertexIndex])
            {
                rBuilder.Spans().m_bProcessedVertices[uiVertexIndex] = true;
                rBuilder.Spans().m_lVertexCount++;
            }
        }
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepEdgeusesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    rBuilder.Spans().m_iProcessedEdge_1stEdgeuses.clear();
    rBuilder.Spans().m_iProcessedEdge_1stEdgeuses.resize(std::max(GetStagedEdgeuseCount(), GetStagedEdgeCount()), -1);

    const uint32_t uiEdgeuseCount = GetStagedEdgeuseCount();
    rBuilder.ReserveEdgeuses(uiEdgeuseCount);
    for (uint32_t ii = 0; ii < uiEdgeuseCount; ++ii)
    {
        const StagedEdgeuseData sEdgeuseData = GetStagedEdgeuse(ii);

        pxr::TfToken tOrientationType = sEdgeuseData.eOrientType == StagedOrientType::Same     ? UsdBrepData::UsdBrepSolidTokens->same :
                                        sEdgeuseData.eOrientType == StagedOrientType::Opposite ? UsdBrepData::UsdBrepSolidTokens->opposite :
                                                                                                 UsdBrepData::UsdBrepSolidTokens->none;
        pxr::TfToken tRadialEntryType = sEdgeuseData.eEntrySideType == StagedRadialEntryType::TopSideEntry ?
                                            UsdBrepData::UsdBrepSolidTokens->topEntry :
                                        sEdgeuseData.eEntrySideType == StagedRadialEntryType::BottomSideEntry ?
                                            UsdBrepData::UsdBrepSolidTokens->bottomEntry :
                                            UsdBrepData::UsdBrepSolidTokens->none;

        const uint32_t uiLocalEdgeIndex = sEdgeuseData.uiEdgeIndex;
        if (uiLocalEdgeIndex >= rBuilder.Spans().m_iProcessedEdge_1stEdgeuses.size())
        {
            return false;
        }

        rBuilder.AppendEdgeuse(
            rBuilder.Arrays().MapLocalToGlobal(uiLocalEdgeIndex, rBuilder.Spans().m_lEdgeStartIndex),
            tOrientationType,
            rBuilder.Arrays().MapLocalToGlobal(sEdgeuseData.uiNextRadialEdgeuseIndex, rBuilder.Spans().m_lEdgeuseStartIndex),
            tRadialEntryType
        );

        rBuilder.Spans().m_lEdgeuseBSplineCurve2d_Count++;
        if (rBuilder.Spans().m_iProcessedEdge_1stEdgeuses[uiLocalEdgeIndex] < 0)
        {
            rBuilder.Spans().m_iProcessedEdge_1stEdgeuses[uiLocalEdgeIndex] = static_cast<int32_t>(ii);
            rBuilder.Spans().m_lEdgeCount++;
        }

        // Emit the edgeuse's UV trim curve when one was staged (seam/boundary pcurves on closed/
        // periodic faces); otherwise a placeholder empty record (UsdBrep expects one record per edgeuse).
        const StagedCurveData2d* pUvCurve = GetStagedEdgeuseCurve2d(ii);
        if (pUvCurve && pUvCurve->bValid && pUvCurve->uiNurbVertexCount > 0)
        {
            rBuilder.AppendEdgeuseNurbCurve(
                pUvCurve->uiNurbVertexCount,
                pUvCurve->uiNurbOrder,
                pUvCurve->vNurbControlVertices,
                pUvCurve->vNurbWeights,
                pUvCurve->vNurbKnots
            );
        }
        else
        {
            rBuilder.AppendEdgeuseNurbCurve(0, 0, pxr::VtArray<pxr::GfVec2d>(), pxr::VtArray<double>(), pxr::VtArray<double>());
        }
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepEdgeCurveFromStagedData(
    UsdBrepData::UsdBrepBuilder& rBuilder,
    uint32_t uiEdgeIndex,
    const pxr::TfToken& rCurveType
) const
{

    const StagedCurveData* pCurve = GetStagedEdgeCurveRecord(uiEdgeIndex);
    if (!pCurve)
    {
        return false;
    }

    if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dNurbAPI)
    {
        rBuilder.Spans().m_lEdgeBSplineCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::BSplineCurve)
        {
            return false;
        }
        rBuilder.AppendEdgeNurbCurve(
            pCurve->uiNurbVertexCount,
            pCurve->uiNurbOrder,
            pCurve->vNurbControlVertices,
            pCurve->vNurbWeights,
            pCurve->vNurbKnots
        );
    }
    else if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dCircleAPI)
    {
        rBuilder.Spans().m_lEdgeCircleCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::Circle)
        {
            return false;
        }
        rBuilder.AppendEdgeCircleCurve(pCurve->sCenter, pCurve->sAxis, pCurve->sRefDirection, pCurve->dRadius);
    }
    else if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dLineAPI)
    {
        rBuilder.Spans().m_lEdgeLineCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::Line)
        {
            return false;
        }
        rBuilder.AppendEdgeLineCurve(pCurve->sOrigin, pCurve->sDirection);
    }
    else if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dEllipseAPI)
    {
        rBuilder.Spans().m_lEdgeEllipseCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::Ellipse)
        {
            return false;
        }
        rBuilder.AppendEdgeEllipseCurve(pCurve->sCenter, pCurve->sAxis, pCurve->sRefDirection, pCurve->dXRadius, pCurve->dYRadius);
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepEdgesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiEdgeCount = GetStagedEdgeCount();
    rBuilder.ReserveEdges(uiEdgeCount);
    for (uint32_t ii = 0; ii < uiEdgeCount; ++ii)
    {
        StagedCurveType eCurveType = GetStagedEdgeCurveType(ii);
        pxr::TfToken tCurveType;
        switch (eCurveType)
        {
            case StagedCurveType::Line:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dLineAPI;
                break;
            case StagedCurveType::Circle:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dCircleAPI;
                break;
            case StagedCurveType::Ellipse:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dEllipseAPI;
                break;
            default:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dNurbAPI;
                break;
        }

        const uint32_t uiStartVertexIndex = GetStagedEdgeStartVertexIndex(ii);
        const uint32_t uiEndVertexIndex = GetStagedEdgeEndVertexIndex(ii);
        if (uiStartVertexIndex >= rBuilder.Spans().m_bProcessedVertices.size() || uiEndVertexIndex >= rBuilder.Spans().m_bProcessedVertices.size())
        {
            return false;
        }

        rBuilder.AppendEdge(
            tCurveType,
            GetStagedEdgeInterval(ii),
            pxr::GfVec2i(
                static_cast<int32_t>(rBuilder.Arrays().MapLocalToGlobal(uiStartVertexIndex, rBuilder.Spans().m_lVertexStartIndex)),
                static_cast<int32_t>(rBuilder.Arrays().MapLocalToGlobal(uiEndVertexIndex, rBuilder.Spans().m_lVertexStartIndex))
            )
        );

        if (!rBuilder.Spans().m_bProcessedVertices[uiStartVertexIndex])
        {
            rBuilder.Spans().m_bProcessedVertices[uiStartVertexIndex] = true;
            rBuilder.Spans().m_lVertexCount++;
            rBuilder.Spans().m_lVertexPointPosition_Count++;
        }
        if (!rBuilder.Spans().m_bProcessedVertices[uiEndVertexIndex])
        {
            rBuilder.Spans().m_bProcessedVertices[uiEndVertexIndex] = true;
            rBuilder.Spans().m_lVertexCount++;
            rBuilder.Spans().m_lVertexPointPosition_Count++;
        }

        if (!AppendDirectUsdBrepEdgeCurveFromStagedData(rBuilder, ii, tCurveType))
        {
            return false;
        }
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepWireEdgeCurveFromStagedData(
    UsdBrepData::UsdBrepBuilder& rBuilder,
    uint32_t uiWireEdgeIndex,
    const pxr::TfToken& rCurveType
) const
{

    const StagedCurveData* pCurve = GetStagedWireEdgeCurveRecord(uiWireEdgeIndex);
    if (!pCurve)
    {
        return false;
    }

    if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dNurbAPI)
    {
        rBuilder.Spans().m_lWireEdgeBSplineCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::BSplineCurve)
        {
            return false;
        }
        rBuilder.AppendWireEdgeNurbCurve(
            pCurve->uiNurbVertexCount,
            pCurve->uiNurbOrder,
            pCurve->vNurbControlVertices,
            pCurve->vNurbWeights,
            pCurve->vNurbKnots
        );
    }
    else if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dCircleAPI)
    {
        rBuilder.Spans().m_lWireEdgeCircleCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::Circle)
        {
            return false;
        }
        rBuilder.AppendWireEdgeCircleCurve(pCurve->sCenter, pCurve->sAxis, pCurve->sRefDirection, pCurve->dRadius);
    }
    else if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dLineAPI)
    {
        rBuilder.Spans().m_lWireEdgeLineCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::Line)
        {
            return false;
        }
        rBuilder.AppendWireEdgeLineCurve(pCurve->sOrigin, pCurve->sDirection);
    }
    else if (rCurveType == UsdBrepData::UsdBrepCurveTokens->brepCurve3dEllipseAPI)
    {
        rBuilder.Spans().m_lWireEdgeEllipseCurve3d_Count++;
        if (pCurve->eCurveType != StagedCurveType::Ellipse)
        {
            return false;
        }
        rBuilder.AppendWireEdgeEllipseCurve(pCurve->sCenter, pCurve->sAxis, pCurve->sRefDirection, pCurve->dXRadius, pCurve->dYRadius);
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepWireEdgesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiWireEdgeCount = GetStagedWireEdgeCount();
    rBuilder.ReserveWireEdges(uiWireEdgeCount);
    for (uint32_t ii = 0; ii < uiWireEdgeCount; ++ii)
    {
        const StagedEdgeData* pWireEdge = GetStagedWireEdgeRecord(ii);
        if (!pWireEdge)
        {
            return false;
        }

        pxr::TfToken tCurveType;
        switch (pWireEdge->eCurveType)
        {
            case StagedCurveType::Line:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dLineAPI;
                break;
            case StagedCurveType::Circle:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dCircleAPI;
                break;
            case StagedCurveType::Ellipse:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dEllipseAPI;
                break;
            default:
                tCurveType = UsdBrepData::UsdBrepCurveTokens->brepCurve3dNurbAPI;
                break;
        }

        const uint32_t uiStartVertexIndex = pWireEdge->uiStartVertexIndex;
        const uint32_t uiEndVertexIndex = pWireEdge->uiEndVertexIndex;
        if (uiStartVertexIndex >= rBuilder.Spans().m_bProcessedVertices.size() || uiEndVertexIndex >= rBuilder.Spans().m_bProcessedVertices.size())
        {
            return false;
        }

        rBuilder.AppendWireEdge(
            tCurveType,
            pWireEdge->sInterval,
            pxr::GfVec2i(
                static_cast<int32_t>(rBuilder.Arrays().MapLocalToGlobal(uiStartVertexIndex, rBuilder.Spans().m_lVertexStartIndex)),
                static_cast<int32_t>(rBuilder.Arrays().MapLocalToGlobal(uiEndVertexIndex, rBuilder.Spans().m_lVertexStartIndex))
            )
        );

        if (!rBuilder.Spans().m_bProcessedVertices[uiStartVertexIndex])
        {
            rBuilder.Spans().m_bProcessedVertices[uiStartVertexIndex] = true;
            rBuilder.Spans().m_lVertexCount++;
            rBuilder.Spans().m_lVertexPointPosition_Count++;
        }
        if (!rBuilder.Spans().m_bProcessedVertices[uiEndVertexIndex])
        {
            rBuilder.Spans().m_bProcessedVertices[uiEndVertexIndex] = true;
            rBuilder.Spans().m_lVertexCount++;
            rBuilder.Spans().m_lVertexPointPosition_Count++;
        }

        if (!AppendDirectUsdBrepWireEdgeCurveFromStagedData(rBuilder, ii, tCurveType))
        {
            return false;
        }
    }

    return true;
}

bool StagedBrepUsdWriter::AppendDirectUsdBrepVerticesFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{

    const uint32_t uiVertexCount = GetStagedVertexCount();
    rBuilder.ReserveVertices(uiVertexCount, GetStagedVertexPointCount());
    for (uint32_t ii = 0; ii < uiVertexCount; ++ii)
    {
        StagedVertexType eVertexType = GetStagedVertexType(ii);
        pxr::TfToken tVertexType = (eVertexType == StagedVertexType::VertexPoint) ? UsdBrepData::UsdBrepSolidTokens->brepPointAPI :
                                                                                    UsdBrepData::UsdBrepSolidTokens->none;

        rBuilder.AppendVertex(tVertexType);

        if (tVertexType == UsdBrepData::UsdBrepSolidTokens->brepPointAPI)
        {
            pxr::GfVec3d sPoint;
            if (!TryGetStagedVertexPoint(ii, sPoint))
            {
                return false;
            }
            rBuilder.AppendVertexPoint(sPoint);
        }
    }

    return true;
}

bool StagedBrepUsdWriter::PopulateCurrentStagedBrepDirectlyToUsdBrepBuilder(UsdBrepData::UsdBrepBuilder& rBuilder, std::string* pErrorStage) const
{
    rBuilder.Reset();

    std::string sDirectError;
    if (!PopulateDirectUsdBrepMetadataFromStagedData(rBuilder))
    {
        sDirectError = "metadata";
    }
    else if (!AppendDirectUsdBrepRegionsFromStagedData(rBuilder))
    {
        sDirectError = "regions";
    }
    else if (!AppendDirectUsdBrepShellsFromStagedData(rBuilder))
    {
        sDirectError = "shells";
    }
    else if (!AppendDirectUsdBrepFaceusesFromStagedData(rBuilder))
    {
        sDirectError = "faceuses";
    }
    else if (!AppendDirectUsdBrepFacesFromStagedData(rBuilder))
    {
        sDirectError = "faces";
    }
    else if (!AppendDirectUsdBrepLoopsFromStagedData(rBuilder))
    {
        sDirectError = "loops";
    }
    else if (!AppendDirectUsdBrepEdgeusesFromStagedData(rBuilder))
    {
        sDirectError = "edgeuses";
    }
    else if (!AppendDirectUsdBrepEdgesFromStagedData(rBuilder))
    {
        sDirectError = "edges";
    }
    else if (!AppendDirectUsdBrepWireEdgesFromStagedData(rBuilder))
    {
        sDirectError = "wire edges";
    }
    else if (!AppendDirectUsdBrepVerticesFromStagedData(rBuilder))
    {
        sDirectError = "vertices";
    }

    if (!sDirectError.empty())
    {
        if (pErrorStage)
        {
            *pErrorStage = sDirectError;
        }
        return false;
    }

    return true;
}

BrepStatus StagedBrepUsdWriter::BuildUsdBrepFromStagedData(UsdBrepData::UsdBrepBuilder& rBuilder) const
{
    std::string sDirectError;
    if (!PopulateCurrentStagedBrepDirectlyToUsdBrepBuilder(rBuilder, &sDirectError))
    {
        USDBREP_ERROR("%s", ("Failed to populate direct UsdBrep BREP " + sDirectError).c_str());
        return BrepStatusError;
    }

    return BrepStatusSuccess;
}

BrepStatus StagedBrepUsdWriter::WriteBrepArrayPrim(
    const pxr::UsdStageRefPtr& rStage,
    const pxr::SdfPath& rRootPath,
    const std::string& rSource,
    std::string* pError,
    pxr::SdfPath* pOutPath
) const
{
    // The 'omniSolid/resources' plugin must be registered once at startup (ConvertSession::createAndConfigureStage).
    assert(UsdBrepData::IsOmniSolidResourcesPluginRegistered());

    if (!rStage)
    {
        if (pError)
        {
            *pError = "Invalid USD stage passed to WriteBrepArrayPrim";
        }
        return BrepStatusError;
    }

    // Create the BrepArray prim at the specified root path with a unique name. Multiple brep bodies may share the
    // same parent path, so find the next available index to avoid overwriting a previously written BrepArray prim.
    int iBrepArrayIndex = 0;
    std::string sBrepArrayName;
    pxr::SdfPath sBrepArrayPath;
    do
    {
        sBrepArrayName = "brepArray" + std::to_string(iBrepArrayIndex);
        sBrepArrayPath = rRootPath.AppendPath(pxr::SdfPath(sBrepArrayName));
        iBrepArrayIndex++;
    } while (rStage->GetPrimAtPath(sBrepArrayPath).IsValid());
    pxr::UsdPrim sBrepArrayPrim = rStage->DefinePrim(sBrepArrayPath, pxr::TfToken("BrepArray"));

    // Build the UsdBrep arrays from the staged model, then serialize them to the USD BrepArray prim.
    UsdBrepData::UsdBrepBuilder sUsdBrepBuilder;
    const BrepStatus sBuildStatus = BuildUsdBrepFromStagedData(sUsdBrepBuilder);
    if (!IsBrepStatusSuccess(sBuildStatus))
    {
        if (pError)
        {
            *pError = "Failed to build UsdBrep brep arrays from staged data";
        }
        return sBuildStatus;
    }

    // Tag the prim with its source provenance. This is authored by BrepWriteToUsdStage below as the
    // BrepArray "source" customData; set it here (after the build, which resets the array data) so the
    // value survives instead of being overwritten with an empty string.
    sUsdBrepBuilder.Arrays().m_sCADSource = rSource;

    if (!UsdBrepData::BrepWriteToUsdStage(sUsdBrepBuilder.Arrays(), sBrepArrayPrim))
    {
        if (pError)
        {
            *pError = "Failed to move UsdBrep brep data to USD BrepArray prim";
        }
        return BrepStatusError;
    }

    // Only publish the path once the BrepArray is fully authored.
    if (pOutPath)
    {
        *pOutPath = sBrepArrayPath;
    }

    return BrepStatusSuccess;
}
