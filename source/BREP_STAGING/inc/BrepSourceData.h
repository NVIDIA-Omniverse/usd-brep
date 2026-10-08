// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_SOURCE_DATA_H
#define BREP_SOURCE_DATA_H

#include "BrepStagingState.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/range2d.h>
#include <pxr/base/gf/vec3d.h>

#include <cstdint>
#include <functional>
#include <vector>

struct SourceBrepData
{
    double dTolerance = 0.0;
    uint32_t uiConnectedComponentCount = 0;
    uint32_t uiInfiniteRegionFaceuseCount = 0;
};

struct SourceConnectedComponentTopologyReserveData
{
    uint32_t uiShellBlockSize = 0;
    uint32_t uiFaceuseBlockSize = 0;
    uint32_t uiShellPairedSpacingIncrement = 0;
    uint32_t uiFaceusePairedSpacingIncrement = 0;
};

struct SourceShellData
{
    uint32_t uiFaceCount = 0;
    bool bClosed = false;
};

struct SourceFaceData
{
    uint32_t uiLoopCount = 0;
    uint32_t uiOuterLoopIndex = kStagedNoObjectIndex;
    bool bSameOrientationWithShell = true;
    bool bClosedSurface = false;
    bool bAllLoopsCW = false;
    uint32_t uiCCWLoopCount = 0;
    bool bHasImpliedNaturalBoundary = false;
    pxr::GfRange2d sDomain;
};

struct SourceFaceOuterLoopData
{
    uint32_t uiLoopCount = 0;
    uint32_t uiOuterLoopIndex = kStagedNoObjectIndex;
    uint32_t uiOuterLoopEdgeuseCount = 0;
    bool bHasImpliedNaturalBoundary = false;
    bool bExplicitOuterLoopOutOfRange = false;
};

struct SourceLoopData
{
    uint32_t uiCoedgeCount = 0;
    bool bSameOrientationWithSurface = true;
};

struct SourceCoedgeData
{
    uint32_t uiSourceEdgeIndex = kStagedNoObjectIndex;
    bool bSameOrientationWithLoop = true;
};

struct SourceVertexData
{
    uint32_t uiSourceVertexIndex = kStagedNoObjectIndex;
    pxr::GfVec3d sPoint;
    bool bHasPoint = false;
};

struct SourceCurveData
{
    StagedCurveData sCurveData;
    pxr::GfRange1d sInterval;
};

struct SourceEdgeData
{
    uint32_t uiSourceEdgeIndex = kStagedNoObjectIndex;
    uint32_t uiStartSourceVertexIndex = kStagedNoObjectIndex;
    uint32_t uiEndSourceVertexIndex = kStagedNoObjectIndex;
    SourceCurveData sCurve;
};

struct SourceSurfaceData
{
    StagedSurfaceData sSurfaceData;
    pxr::GfRange2d sDomain;
};

struct SourceLoopBoundaryContactData
{
    uint32_t uiLoopIndex = kStagedNoObjectIndex;
    uint32_t uiCoedgeCount = 0;
    bool bClockwiseWithSurface = false;
    bool bTouchesNaturalBoundary = false;
    uint32_t uiFirstContactStagedVertexIndex = kStagedNoObjectIndex;
};

struct SourceFaceBoundaryContactData
{
    std::vector<SourceLoopBoundaryContactData> vLoops;
};

struct SourceNaturalBoundaryData
{
    StagedSurfaceData sSurfaceData;
    pxr::GfRange2d sDomain;
    std::vector<pxr::GfVec3d> vBoundaryVertexPositions;
};

using SourceNaturalBoundaryVertexResolver = std::function<uint32_t(const pxr::GfVec3d& rPoint)>;

#endif // BREP_SOURCE_DATA_H
