// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StagedBrepTransform.h"

#include "BrepStagingState.h"
#include "UsdBrepDiagnostics.h"

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>

#include <cmath>

using namespace pxr;

namespace
{

double GetMatrixColumnLength(const pxr::GfMatrix4d& rTransform, int iCol)
{
    return pxr::GfVec3d(rTransform[0][iCol], rTransform[1][iCol], rTransform[2][iCol]).GetLength();
}

void GetMatrixScales(const pxr::GfMatrix4d& rTransform, double& rdSX, double& rdSY, double& rdSZ)
{
    rdSX = GetMatrixColumnLength(rTransform, 0);
    rdSY = GetMatrixColumnLength(rTransform, 1);
    rdSZ = GetMatrixColumnLength(rTransform, 2);
}

pxr::GfVec3d TransformPoint(const pxr::GfMatrix4d& rTransform, const pxr::GfVec3d& rPoint)
{
    return rTransform.Transform(rPoint);
}

pxr::GfVec3d TransformVectorNoTranslation(const pxr::GfMatrix4d& rTransform, const pxr::GfVec3d& rVector)
{
    return rTransform.TransformDir(rVector);
}

} // namespace


uint32_t StagedBrepTransform::GetStagedFaceSurfaceCount() const
{
    return m_rState.GetFaceSurfaceCount();
}

StagedFaceData* StagedBrepTransform::GetStagedFaceForSurface(uint32_t uiSurfaceIndex)
{
    return m_rState.GetFaceRecord(uiSurfaceIndex);
}

StagedSurfaceData* StagedBrepTransform::GetStagedFaceSurface(uint32_t uiSurfaceIndex)
{
    return m_rState.GetFaceSurfaceRecord(uiSurfaceIndex);
}

uint32_t StagedBrepTransform::GetStagedEdgeCurveSetCount(bool bWireEdgeSet) const
{
    return bWireEdgeSet ? m_rState.GetWireEdgeCurveCount() : m_rState.GetEdgeCurveCount();
}

StagedEdgeData* StagedBrepTransform::GetStagedEdgeCurveSetEdge(bool bWireEdgeSet, uint32_t uiEdgeIndex)
{
    return bWireEdgeSet ? m_rState.GetWireEdgeRecord(uiEdgeIndex) : m_rState.GetEdgeRecord(uiEdgeIndex);
}

StagedCurveData* StagedBrepTransform::GetStagedEdgeCurveSetCurve(bool bWireEdgeSet, uint32_t uiCurveIndex)
{
    return bWireEdgeSet ? m_rState.GetWireEdgeCurveRecord(uiCurveIndex) : m_rState.GetEdgeCurveRecord(uiCurveIndex);
}

void StagedBrepTransform::TransformStagedVertexPoint(
    uint32_t uiVertexIndex,
    bool bApplyScale,
    double dScale,
    bool bHaveTransform,
    const pxr::GfMatrix4d& rTransform
)
{
    pxr::GfVec3d* pPoint = m_rState.GetVertexPointRecord(uiVertexIndex);
    if (!pPoint)
    {
        return;
    }

    pxr::GfVec3d& rPoint = *pPoint;
    if (bApplyScale)
    {
        rPoint *= dScale;
    }

    if (bHaveTransform)
    {
        rPoint = TransformPoint(rTransform, rPoint);
    }
}

void StagedBrepTransform::TransformStagedFaceSurfaces(bool bApplyScale, double dScale, bool bHaveTransform, const pxr::GfMatrix4d& rTransform)
{
    // Transform analytic face surfaces so they match the same world space as NURBS surfaces, vertices, and edge curves.
    double dScaleFromMatrix = 1.0;
    if (bHaveTransform)
    {
        double dSX = 1.0, dSY = 1.0, dSZ = 1.0;
        GetMatrixScales(rTransform, dSX, dSY, dSZ);
        constexpr double dUniformScaleTolerance = 1e-10;
        if (std::fabs(dSX - dSY) > dUniformScaleTolerance || std::fabs(dSY - dSZ) > dUniformScaleTolerance)
        {
            USDBREP_ERROR("Non-uniform matrix scale (sx=%g, sy=%g, sz=%g) is not supported for analytic face surfaces", dSX, dSY, dSZ);
            return;
        }
        dScaleFromMatrix = (dSX + dSY + dSZ) / 3.0; // Averaged scale. Note: elsewhere non-uniform scaling for analytics is treated as an error;
                                                    // a future improvement would convert analytics to BSplines for non-uniform scaling.
        if (dScaleFromMatrix < 1e-12)
        {
            dScaleFromMatrix = 1.0;
        }
    }

    auto transformVectorNoTranslation = [&](const pxr::GfVec3d& rVec) -> pxr::GfVec3d
    {
        return TransformVectorNoTranslation(rTransform, rVec);
    };
    auto scaleRange2d = [&](pxr::GfRange2d& rRange, double dUScale, double dVScale)
    {
        rRange.SetMin(pxr::GfVec2d(rRange.GetMin()[0] * dUScale, rRange.GetMin()[1] * dVScale));
        rRange.SetMax(pxr::GfVec2d(rRange.GetMax()[0] * dUScale, rRange.GetMax()[1] * dVScale));
    };

    for (uint32_t uiSurfaceIndex = 0; uiSurfaceIndex < GetStagedFaceSurfaceCount(); ++uiSurfaceIndex)
    {
        // surface and OwnerFace
        StagedFaceData* pFace = GetStagedFaceForSurface(uiSurfaceIndex);
        StagedSurfaceData* pSurface = GetStagedFaceSurface(uiSurfaceIndex);

        // Plane
        StagedSurfaceData* pPlane = (pSurface && pSurface->eSurfaceType == StagedSurfaceType::Plane) ? pSurface : nullptr;
        if (pPlane)
        {
            if (bApplyScale)
            {
                pPlane->sOrigin *= dScale;
                if (pFace)
                {
                    scaleRange2d(pFace->sUVDomain, dScale, dScale);
                }
            }
            if (bHaveTransform)
            {
                pPlane->sOrigin = TransformPoint(rTransform, pPlane->sOrigin);

                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pPlane->sRefDirection);
                double dRefDirLength = sRefDirTrans.GetLength();

                const pxr::GfVec3d sYDir = pxr::GfCross(pPlane->sAxis, pPlane->sRefDirection);
                pxr::GfVec3d sYDirTrans = transformVectorNoTranslation(sYDir);
                double dYDirLength = sYDirTrans.GetLength();

                if (dRefDirLength > 1e-12)
                {
                    pPlane->sRefDirection = sRefDirTrans.GetNormalized();
                }
                if (dYDirLength > 1e-12)
                {
                    sYDirTrans = sYDirTrans.GetNormalized();
                    pxr::GfVec3d sAxisTrans = pxr::GfCross(sRefDirTrans, sYDirTrans);
                    double dAxisLength = sAxisTrans.GetLength();
                    if (dAxisLength > 1e-12)
                    {
                        pPlane->sAxis = sAxisTrans.GetNormalized();
                    }
                }
                if (pFace)
                {
                    scaleRange2d(pFace->sUVDomain, dRefDirLength, dYDirLength);
                }
            }
            continue;
        } // end Plane check

        // Cylinder
        StagedSurfaceData* pCylinder = (pSurface && pSurface->eSurfaceType == StagedSurfaceType::Cylinder) ? pSurface : nullptr;
        if (pCylinder)
        {
            if (bApplyScale)
            {
                pCylinder->sOrigin *= dScale;
                pCylinder->dRadius *= dScale;
                if (pFace)
                {
                    scaleRange2d(pFace->sUVDomain, 1.0, dScale);
                }
            }
            if (bHaveTransform)
            {
                pCylinder->sOrigin = TransformPoint(rTransform, pCylinder->sOrigin);
                pxr::GfVec3d sAxisTrans = transformVectorNoTranslation(pCylinder->sAxis);
                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pCylinder->sRefDirection);

                double dAxisLength = sAxisTrans.GetLength();
                double dRefDirLength = sRefDirTrans.GetLength();
                if (dAxisLength > 1e-12)
                {
                    pCylinder->sAxis = sAxisTrans.GetNormalized();
                }
                if (dRefDirLength > 1e-12)
                {
                    pCylinder->sRefDirection = sRefDirTrans.GetNormalized();
                }
                pCylinder->dRadius *= dScaleFromMatrix;
                if (pFace)
                {
                    scaleRange2d(pFace->sUVDomain, 1.0, dAxisLength);
                }
            }
            continue;
        } // end Cylinder check

        // Cone
        StagedSurfaceData* pCone = (pSurface && pSurface->eSurfaceType == StagedSurfaceType::Cone) ? pSurface : nullptr;
        if (pCone)
        {
            if (bApplyScale)
            {
                pCone->sOrigin *= dScale;
                pCone->dRadius *= dScale;
                if (pFace)
                {
                    scaleRange2d(pFace->sUVDomain, 1.0, dScale);
                }
            }
            if (bHaveTransform)
            {
                pCone->sOrigin = TransformPoint(rTransform, pCone->sOrigin);
                pxr::GfVec3d sAxisTrans = transformVectorNoTranslation(pCone->sAxis);
                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pCone->sRefDirection);

                double dAxisLength = sAxisTrans.GetLength();
                double dRefDirLength = sRefDirTrans.GetLength();
                if (dAxisLength > 1e-12)
                {
                    pCone->sAxis = sAxisTrans.GetNormalized();
                }
                if (dRefDirLength > 1e-12)
                {
                    pCone->sRefDirection = sRefDirTrans.GetNormalized();
                }
                pCone->dRadius *= dScaleFromMatrix;
                if (pFace)
                {
                    scaleRange2d(pFace->sUVDomain, 1.0, dAxisLength);
                }
            }
            continue;
        } // end Cone check

        // Sphere
        StagedSurfaceData* pSphere = (pSurface && pSurface->eSurfaceType == StagedSurfaceType::Sphere) ? pSurface : nullptr;
        if (pSphere)
        {
            if (bApplyScale)
            {
                pSphere->sCenter *= dScale;
                pSphere->dRadius *= dScale;
            }
            if (bHaveTransform)
            {
                pSphere->sCenter = TransformPoint(rTransform, pSphere->sCenter);
                pxr::GfVec3d sAxisTrans = transformVectorNoTranslation(pSphere->sAxis);
                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pSphere->sRefDirection);

                double dAxisLength = sAxisTrans.GetLength();
                double dRefDirLength = sRefDirTrans.GetLength();
                if (dAxisLength > 1e-12)
                {
                    pSphere->sAxis = sAxisTrans.GetNormalized();
                }
                if (dRefDirLength > 1e-12)
                {
                    pSphere->sRefDirection = sRefDirTrans.GetNormalized();
                }
                pSphere->dRadius *= dScaleFromMatrix;
            }
            continue;
        } // end Sphere check

        // Torus
        StagedSurfaceData* pTorus = (pSurface && pSurface->eSurfaceType == StagedSurfaceType::Torus) ? pSurface : nullptr;
        if (pTorus)
        {
            if (bApplyScale)
            {
                pTorus->sOrigin *= dScale;
                pTorus->dMajorRadius *= dScale;
                pTorus->dMinorRadius *= dScale;
            }
            if (bHaveTransform)
            {
                pTorus->sOrigin = TransformPoint(rTransform, pTorus->sOrigin);
                pxr::GfVec3d sAxisTrans = transformVectorNoTranslation(pTorus->sAxis);
                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pTorus->sRefDirection);

                double dAxisLength = sAxisTrans.GetLength();
                double dRefDirLength = sRefDirTrans.GetLength();
                if (dAxisLength > 1e-12)
                {
                    pTorus->sAxis = sAxisTrans.GetNormalized();
                }
                if (dRefDirLength > 1e-12)
                {
                    pTorus->sRefDirection = sRefDirTrans.GetNormalized();
                }
                pTorus->dMajorRadius *= dScaleFromMatrix;
                pTorus->dMinorRadius *= dScaleFromMatrix;
            }
            continue;
        } // end Torus check

        // BSplineSurface
        StagedSurfaceData* pBSplineSurf = (pSurface && pSurface->eSurfaceType == StagedSurfaceType::BSplineSurface) ? pSurface : nullptr;
        if (pBSplineSurf)
        {
            for (size_t i = 0; i < pBSplineSurf->vNurbControlVertices.size(); ++i)
            {
                pxr::GfVec3d& rPt = pBSplineSurf->vNurbControlVertices[i];
                if (bApplyScale)
                {
                    rPt *= dScale;
                }
                if (bHaveTransform)
                {
                    rPt = TransformPoint(rTransform, rPt);
                }
            } // end iter i, every NURBS surface control vertex
            continue;
        } // end BSplineSurface check
    } // end iter surfIdx, every staged face surface
}

void StagedBrepTransform::TransformStagedEdgeCurveSet(
    bool bWireEdgeSet,
    bool bApplyScale,
    double dScale,
    bool bHaveTransform,
    const pxr::GfMatrix4d& rTransform
)
{
    auto transformVectorNoTranslation = [&](const pxr::GfVec3d& rVec) -> pxr::GfVec3d
    {
        return TransformVectorNoTranslation(rTransform, rVec);
    };
    auto scaleRange1d = [&](pxr::GfRange1d& rRange, double dRangeScale)
    {
        rRange.SetMin(rRange.GetMin() * dRangeScale);
        rRange.SetMax(rRange.GetMax() * dRangeScale);
    };

    for (uint32_t uiEdgeIndex = 0; uiEdgeIndex < GetStagedEdgeCurveSetCount(bWireEdgeSet); ++uiEdgeIndex)
    {
        // curve->OwnerEdge
        StagedEdgeData* pEdge = GetStagedEdgeCurveSetEdge(bWireEdgeSet, uiEdgeIndex);
        StagedCurveData* pCurve = GetStagedEdgeCurveSetCurve(bWireEdgeSet, uiEdgeIndex);
        if (!pCurve)
        {
            continue;
        }

        // line
        if (pCurve->eCurveType == StagedCurveType::Line)
        {
            if (bApplyScale)
            {
                pCurve->sOrigin *= dScale;
                if (pEdge)
                {
                    scaleRange1d(pEdge->sInterval, dScale);
                }
            }
            if (bHaveTransform)
            {
                pCurve->sOrigin = TransformPoint(rTransform, pCurve->sOrigin);

                pxr::GfVec3d sDirTrans = transformVectorNoTranslation(pCurve->sDirection);
                double dDirLength = sDirTrans.GetLength();
                if (dDirLength > 1e-12)
                {
                    pCurve->sDirection = sDirTrans.GetNormalized();
                    if (pEdge)
                    {
                        scaleRange1d(pEdge->sInterval, dDirLength);
                    }
                }
            }
            continue;
        } // end Line check

        // circle
        if (pCurve->eCurveType == StagedCurveType::Circle)
        {
            if (bApplyScale)
            {
                pCurve->sCenter *= dScale;
                pCurve->dRadius *= dScale;
            }
            if (bHaveTransform)
            {
                pCurve->sCenter = TransformPoint(rTransform, pCurve->sCenter);

                const pxr::GfVec3d sYDir = pxr::GfCross(pCurve->sAxis, pCurve->sRefDirection);
                pxr::GfVec3d sAxisTrans = transformVectorNoTranslation(pCurve->sAxis);
                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pCurve->sRefDirection);
                pxr::GfVec3d sYDirTrans = transformVectorNoTranslation(sYDir);

                double dAxisLength = sAxisTrans.GetLength();
                const double dRefDirLength = sRefDirTrans.GetLength();
                const double dYDirLength = sYDirTrans.GetLength();

                if (dAxisLength > 1e-12)
                {
                    pCurve->sAxis = sAxisTrans.GetNormalized();
                }
                if (dRefDirLength > 1e-12)
                {
                    pCurve->sRefDirection = sRefDirTrans.GetNormalized();
                }

                // A non-uniform transform turns a circle into an ellipse. Preserve the existing
                // circle schema by using the average transformed in-plane scale.
                constexpr double dCircleUniformScaleTolerance = 1e-10;
                if (std::fabs(dRefDirLength - dYDirLength) > dCircleUniformScaleTolerance)
                {
                    USDBREP_WARN(
                        "Non-uniform in-plane scale for circle curve index %u (refDirLength=%g, yDirLength=%g); using averaged radius scale",
                        uiEdgeIndex,
                        dRefDirLength,
                        dYDirLength
                    );
                }
                const double dCircleScaleFromMatrix = 0.5 * (dRefDirLength + dYDirLength);
                if (dCircleScaleFromMatrix > 1e-12)
                {
                    pCurve->dRadius *= dCircleScaleFromMatrix;
                }
            }
            continue;
        } // end Circle check

        // ellipse
        if (pCurve->eCurveType == StagedCurveType::Ellipse)
        {
            if (bApplyScale)
            {
                pCurve->sCenter *= dScale;
                pCurve->dXRadius *= dScale;
                pCurve->dYRadius *= dScale;
            }
            if (bHaveTransform)
            {
                pCurve->sCenter = TransformPoint(rTransform, pCurve->sCenter);

                const pxr::GfVec3d sYDir = pxr::GfCross(pCurve->sAxis, pCurve->sRefDirection);
                pxr::GfVec3d sRefDirTrans = transformVectorNoTranslation(pCurve->sRefDirection);
                pxr::GfVec3d sYDirTrans = transformVectorNoTranslation(sYDir);

                const double dRefDirLength = sRefDirTrans.GetLength();
                const double dYDirLength = sYDirTrans.GetLength();

                if (dRefDirLength > 1e-12)
                {
                    pCurve->sRefDirection = sRefDirTrans.GetNormalized();
                }
                if (dYDirLength > 1e-12)
                {
                    sYDirTrans = sYDirTrans.GetNormalized();
                    pxr::GfVec3d sAxisTrans = pxr::GfCross(sRefDirTrans, sYDirTrans);
                    double dAxisLength = sAxisTrans.GetLength();
                    if (dAxisLength > 1e-12)
                    {
                        pCurve->sAxis = sAxisTrans.GetNormalized();
                    }
                }
                if (dRefDirLength > 1e-12)
                {
                    pCurve->dXRadius *= dRefDirLength;
                }
                if (dYDirLength > 1e-12)
                {
                    pCurve->dYRadius *= dYDirLength;
                }
            }
            continue;
        } // end Ellipse check

        // BSplineCurve
        if (pCurve->eCurveType == StagedCurveType::BSplineCurve)
        {
            // for every BSplineCurve ControlVertex
            for (size_t i = 0; i < pCurve->vNurbControlVertices.size(); ++i)
            {
                pxr::GfVec3d& rPt = pCurve->vNurbControlVertices[i];
                // Apply scale first
                if (bApplyScale)
                {
                    rPt *= dScale;
                }
                // Then apply transform
                if (bHaveTransform)
                {
                    rPt = TransformPoint(rTransform, rPt);
                }
            } // end iter i, every NURBS curve control vertex
            continue;
        } // end BSplineCurve check
    } // end iter staged edge curve index
}
