// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef STAGED_BREP_TRANSFORM_H
#define STAGED_BREP_TRANSFORM_H

#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/matrix4d.h>

#include <cstdint>

class BrepStagingState;
struct StagedCurveData;
struct StagedEdgeData;
struct StagedFaceData;
struct StagedSurfaceData;

class StagedBrepTransform
{
public:

    explicit StagedBrepTransform(BrepStagingState& rState) : m_rState(rState)
    {
    }

    void TransformStagedVertexPoint(uint32_t uiVertexIndex, bool bApplyScale, double dScale, bool bHaveTransform, const pxr::GfMatrix4d& rTransform);
    void TransformStagedFaceSurfaces(bool bApplyScale, double dScale, bool bHaveTransform, const pxr::GfMatrix4d& rTransform);
    void TransformStagedEdgeCurveSet(bool bWireEdgeSet, bool bApplyScale, double dScale, bool bHaveTransform, const pxr::GfMatrix4d& rTransform);

private:

    uint32_t GetStagedFaceSurfaceCount() const;
    StagedFaceData* GetStagedFaceForSurface(uint32_t uiSurfaceIndex);
    StagedSurfaceData* GetStagedFaceSurface(uint32_t uiSurfaceIndex);
    uint32_t GetStagedEdgeCurveSetCount(bool bWireEdgeSet) const;
    StagedEdgeData* GetStagedEdgeCurveSetEdge(bool bWireEdgeSet, uint32_t uiEdgeIndex);
    StagedCurveData* GetStagedEdgeCurveSetCurve(bool bWireEdgeSet, uint32_t uiCurveIndex);

    BrepStagingState& m_rState;
};

#endif // STAGED_BREP_TRANSFORM_H
