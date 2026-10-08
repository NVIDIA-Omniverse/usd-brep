// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_EXTENT_BUILDER_H
#define BREP_EXTENT_BUILDER_H

#include "BrepStagingState.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/vec3d.h>

class StagedBrepExtentBuilder
{
public:

    explicit StagedBrepExtentBuilder(BrepStagingState& rState);

    void UpdateExtent();

private:

    void ExpandExtentWithStagedEdgeCurveSet(bool bWireEdgeSet, pxr::GfVec3d& rExtentMin, pxr::GfVec3d& rExtentMax, bool& bHasGeometry);
    void ExpandExtentWithStagedFaceSurfaces(pxr::GfVec3d& rExtentMin, pxr::GfVec3d& rExtentMax, bool& bHasGeometry);

    BrepStagingState& m_rState;
};

#endif // BREP_EXTENT_BUILDER_H
