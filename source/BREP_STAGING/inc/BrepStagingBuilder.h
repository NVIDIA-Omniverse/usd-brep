// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_STAGING_BUILDER_H
#define BREP_STAGING_BUILDER_H

#include "BrepSourceData.h"

class GenericBrepStagingBuilder
{
public:

    explicit GenericBrepStagingBuilder(BrepStagingState& rState);

    void ResetForSourceBrep(const SourceBrepData& rBrepData);
    uint32_t AddFace(const SourceFaceData& rFaceData, uint32_t uiLoopCount, StagedTrimType eTrimType);
    uint32_t AddFaceSurface(const SourceSurfaceData& rSurfaceData);
    uint32_t AddFaceSurfaceForCurrentFace(const SourceSurfaceData& rSurfaceData, StagedTrimType eTrimType);
    void SetCurrentFaceTrimAndDomain(StagedTrimType eTrimType, const pxr::GfRange2d& rUVDomain);
    void SetCurrentFaceLoopCount(uint32_t uiLoopCount);
    void AddLoopForCurrentTraversal(
        const StagedLoopData& rLoopData,
        bool bIsOuterLoop,
        uint32_t uiCurrentLoopTraversalCount,
        bool bHasStagedImpliedNaturalBoundary
    );
    void AddSourceLoopForCurrentTraversal(
        const SourceLoopData& rLoopData,
        bool bIsAugmentedOuterLoop,
        uint32_t uiAugmentedOuterLoopEdgeuseCount,
        bool bIsOuterLoop,
        uint32_t uiCurrentLoopTraversalCount,
        bool bHasStagedImpliedNaturalBoundary
    );

private:

    StagedFaceData* GetCurrentFace();

    BrepStagingState& m_rState;
};

#endif // BREP_STAGING_BUILDER_H
