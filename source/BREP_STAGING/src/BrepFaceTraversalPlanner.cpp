// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BrepFaceTraversalPlanner.h"

#include "UsdBrepDiagnostics.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/vec3d.h>

#include <cstdint>

using namespace pxr;

BrepStatus PlanFaceOuterLoop(
    uint32_t uiLoopCount,
    uint32_t uiSourceOuterLoopIndex,
    bool bClosedSurface,
    bool bAllLoopsCW,
    uint32_t uiCCWLoopCount,
    const std::vector<bool>& vLoopEffectivelyCCW,
    const std::vector<uint32_t>& vLoopCoedgeCount,
    SourceFaceOuterLoopData& rOuterLoopData
)
{
    rOuterLoopData = SourceFaceOuterLoopData();
    rOuterLoopData.uiLoopCount = uiLoopCount;
    rOuterLoopData.uiOuterLoopIndex = uiSourceOuterLoopIndex;

    // Contract: the caller must supply per-loop data for every loop. Treat an
    // undersized vector as a hard error rather than silently degrading to an
    // implied-natural-boundary fallback, which would mask the missing data.
    if (vLoopEffectivelyCCW.size() < uiLoopCount || vLoopCoedgeCount.size() < uiLoopCount)
    {
        return BrepStatusError;
    }

    // index < loopCount: source named an explicit outer loop. Trust it; ignore orientation.
    if (uiSourceOuterLoopIndex < uiLoopCount)
    {
        rOuterLoopData.uiOuterLoopEdgeuseCount = vLoopCoedgeCount[uiSourceOuterLoopIndex];
        return BrepStatusSuccess;
    }

    // index == loopCount: source declares "no outer loop" → implied natural boundary. Trust it.
    if (uiSourceOuterLoopIndex == uiLoopCount)
    {
        rOuterLoopData.bHasImpliedNaturalBoundary = true;
        return BrepStatusSuccess;
    }

    // index > loopCount: unknown/malformed. Orientation fallback only — never invent a boundary
    // from the bad index alone.
    if (bClosedSurface && bAllLoopsCW)
    {
        rOuterLoopData.bHasImpliedNaturalBoundary = true;
        return BrepStatusSuccess;
    }

    if (uiCCWLoopCount > 0)
    {
        for (uint32_t ii = 0; ii < uiLoopCount; ii++)
        {
            if (vLoopEffectivelyCCW[ii])
            {
                rOuterLoopData.uiOuterLoopIndex = ii;
                rOuterLoopData.uiOuterLoopEdgeuseCount = vLoopCoedgeCount[ii];
                return BrepStatusSuccess;
            }
        }
    }

    rOuterLoopData.bExplicitOuterLoopOutOfRange = true;
    return BrepStatusSuccess;
}

StagedFaceTraversalPlanner::StagedFaceTraversalPlanner(BrepStagingState& rState, const SourceFaceData& rSourceFaceData, uint32_t uiFaceIndex)
    : m_rState(rState), m_rSourceFaceData(rSourceFaceData), m_uiFaceIndex(uiFaceIndex)
{
}


void StagedFaceTraversalPlanner::ResetImpliedBoundaryState()
{
    m_rState.ResetImpliedBoundaryState();
    m_rState.ClearImpliedBoundarySegments();
}

void StagedFaceTraversalPlanner::SetImpliedBoundaryCase(uint32_t uiTouchingCWCount, uint32_t uiTouchingCCWCount)
{
    const uint32_t uiTotalTouching = uiTouchingCWCount + uiTouchingCCWCount;
    ImpliedBoundaryCase eCase = (uiTotalTouching == 0)                            ? BoundaryCase_StandaloneLoop :
                                (uiTotalTouching == 1 && uiTouchingCWCount == 1)  ? BoundaryCase_AugmentCWLoop :
                                (uiTotalTouching == 1 && uiTouchingCCWCount == 1) ? BoundaryCase_CCWLoopIsOuter :
                                (uiTotalTouching > 1 && uiTouchingCCWCount == 0)  ? BoundaryCase_MultiCWMerge :
                                                                                    BoundaryCase_MixedError;
    m_rState.SetImpliedBoundaryCase(eCase);
}

bool StagedFaceTraversalPlanner::IsImpliedBoundaryCase(ImpliedBoundaryCase eCase) const
{
    return m_rState.GetImpliedBoundaryCase() == eCase;
}

bool StagedFaceTraversalPlanner::NeedsImpliedBoundaryEdges() const
{
    return IsImpliedBoundaryCase(BoundaryCase_StandaloneLoop) || IsImpliedBoundaryCase(BoundaryCase_AugmentCWLoop);
}

void StagedFaceTraversalPlanner::SetImpliedBoundaryAugmentTarget(uint32_t uiSourceLoopIndex, uint32_t uiContactVertexIndex)
{
    m_rState.SetImpliedBoundaryAugmentTarget(
        IsImpliedBoundaryCase(BoundaryCase_AugmentCWLoop) ? uiSourceLoopIndex : UINT32_MAX,
        IsImpliedBoundaryCase(BoundaryCase_AugmentCWLoop) ? uiContactVertexIndex : UINT32_MAX
    );
}

void StagedFaceTraversalPlanner::SetCurrentFaceLoopCount(uint32_t uiLoopCount)
{
    const uint32_t uiFaceCount = m_rState.GetFaceCount();
    StagedFaceData* pFaceData = uiFaceCount > 0 ? m_rState.GetFaceRecord(uiFaceCount - 1) : nullptr;
    if (!pFaceData)
    {
        return;
    }

    pFaceData->uiLoopCount = uiLoopCount;
}

BrepStatus StagedFaceTraversalPlanner::PlanCurrentFace(
    uint32_t& rLoopCount,
    const SourceFaceBoundaryContactData& rBoundaryContactData,
    StagedFacePlanningResult& rPlanningResult
)
{
    // For faces with an implied outer loop, create and add a synthetic loop to the staged BREP data.
    //   Add StagedLoopData     to m_sStagingState.Data().vLoops        - just one
    //       StagedVertexData   to m_sStagingState.Data().vVertices     - one for every loop EdgeEnd - EdgeStart connection
    //       StagedEdgeData     to m_sStagingState.Data().vEdges        - one for every lamina or seam edge
    //       StagedEdgeuseData  to m_sStagingState.Data().vEdgeuses     - one for every lamina edge, two for every seam edge
    //       StagedCurveData    to m_sStagingState.Data().vEdgeCurves - one for every lamina or seam edge curve
    //
    // So far, the types of faces with no loops include:
    //   1) standalone rectangular faces
    //   2) "Wireframe faces" from autocad dwg files
    //   3) closed surfaces with no loops or only innerLoops
    // Calling addImpliedNaturalBoundaryAsLoop(), will stop #2 from crashing. It is not a complete solution for #2 because
    //   wireframes can have discontinuities. The source adapter should eventually provide loops for every
    //   face AND to split wireframe faces at discontinuities. Alternatively, the source can stop sending "wireframe faces"
    //   as brep surfaces. Instead they should be meshes or breps with multiple faces.
    // Classify how the implied outer loop should be built
    ResetImpliedBoundaryState();
    rPlanningResult.uiLoopCount = rLoopCount;

    if (m_rSourceFaceData.bHasImpliedNaturalBoundary)
    {
        // locals
        uint32_t uiTouchingCWCount = 0;
        uint32_t uiTouchingCCWCount = 0;
        uint32_t uiTouchingSourceLoopIndex = UINT32_MAX;
        uint32_t uiTouchingSourceLoopCoedgeCount = 0;
        uint32_t uiFirstContactStagedVertexIndex = UINT32_MAX;

        // for every source loop - find if loop vertices touch the surface natural boundaries
        for (const SourceLoopBoundaryContactData& rLoopContactData : rBoundaryContactData.vLoops)
        {
            if (!rLoopContactData.bTouchesNaturalBoundary)
            {
                continue;
            }

            if (rLoopContactData.bClockwiseWithSurface)
            {
                uiTouchingCWCount++;
            }
            else
            {
                uiTouchingCCWCount++;
            }
            uiTouchingSourceLoopIndex = rLoopContactData.uiLoopIndex;
            uiTouchingSourceLoopCoedgeCount = rLoopContactData.uiCoedgeCount;
            if (uiFirstContactStagedVertexIndex == UINT32_MAX)
            {
                uiFirstContactStagedVertexIndex = rLoopContactData.uiFirstContactStagedVertexIndex;
            }
        } // end iter every source loop - find natural boundary contact

        SetImpliedBoundaryCase(uiTouchingCWCount, uiTouchingCCWCount);

        SetImpliedBoundaryAugmentTarget(uiTouchingSourceLoopIndex, uiFirstContactStagedVertexIndex);
        rPlanningResult.uiSelectedBoundarySourceLoopCoedgeCount = uiTouchingSourceLoopCoedgeCount;

        // when needed - extract Implied BoundaryEdges for later use and adjust staged loop count for case BoundaryCase_StandaloneLoop
        if (NeedsImpliedBoundaryEdges())
        {
            rPlanningResult.bExtractImpliedBoundaryEdges = true;
        } // end ned to extractImplied boundary edges check


        // execute based on case
        switch (m_rState.GetImpliedBoundaryCase())
        {
            case BoundaryCase_StandaloneLoop:
            {
                rPlanningResult.bAddStandaloneImpliedBoundaryLoop = true;
                rPlanningResult.bMarkStandaloneImpliedLoopProcessed = true;
                rLoopCount++; // total loop count for the face
                SetCurrentFaceLoopCount(rLoopCount);
                rPlanningResult.bSetStandaloneImpliedOuterLoopEdgeuseCount = true;
                rPlanningResult.bLoopCountChanged = true;
                rPlanningResult.uiLoopCount = rLoopCount;


            } // end BoundaryCase_StandaloneLoop case
            break;

            case BoundaryCase_AugmentCWLoop:
            {
                // flag the shell so subsequent faces negate their prcSameFaceOrient
                rPlanningResult.bMarkShellHasAugmentedFace = true;

                // fix faceuse orientations for faces already processed before this augmented face
                // PRC set their orientations based on the wrong model interpretation (complement)
                rPlanningResult.bFlipPriorFaceuseOrientationsForAugmentedFace = true;

                // note: NaturalBoundaryBuilder::AddImpliedNaturalBoundaryToLoop() is run later in visitLeave(Loop) after all
                //       source coedges have been processed into staged edgeuses and assigned staged index values.

                // extractImpliedBoundaryEdges() already called above and populated staged boundary segments.
                // compute augmented outer loop EU count = PRC coedge count + boundary segment count
                rPlanningResult.bSetAugmentedOuterLoopForCurrentFace = true;
            } // end BoundaryCase_AugmentCWLoop case
            break;

            case BoundaryCase_CCWLoopIsOuter:
            {
                // the CCW loop is already the outer loop — no boundary edges to add.
                // Reset m_bNaturalBoundaried_Implied so downstream code treats the CCW loop as an explicit outer loop
                // with proper slot pre-allocation and placement.
                rPlanningResult.bSetCCWLoopAsExplicitOuterLoop = true;
                rPlanningResult.uiCCWLoopExplicitOuterSourceLoopIndex = uiTouchingSourceLoopIndex;

                rLoopCount = m_rSourceFaceData.uiLoopCount;
                SetCurrentFaceLoopCount(rLoopCount);
                rPlanningResult.bLoopCountChanged = true;
                rPlanningResult.uiLoopCount = rLoopCount;
            } // end BoundaryCase_CCWLoopIsOuter case
            break;

            case BoundaryCase_MultiCWMerge:
            {
                // TODO: In the future, multiple CW loops touching the natural boundary at different
                // vertices must be merged into a single CCW outer loop. Not yet implemented.
                USDBREP_ERROR("BoundaryCase_MultiCWMerge is not yet implemented for face index %u", m_uiFaceIndex);
                return BrepStatusError;
            } // end BoundaryCase_MultiCWMerge case
            break;

            case BoundaryCase_MixedError:
            {
                return BrepStatusError;
            } // end BoundaryCase_MixedError case
            break;

            case BoundaryCase_None:
            default:
                break;
        } // end implied-boundary case switch

    } // end m_bNaturalBoundaried_Implied check

    return BrepStatusSuccess;
}
