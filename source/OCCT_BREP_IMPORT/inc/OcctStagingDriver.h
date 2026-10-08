// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_STAGING_DRIVER_H
#define OCCT_BREP_IMPORT_OCCT_STAGING_DRIVER_H

#include "BrepStagingState.h"
#include "OcctFileSource.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/range1d.h>
#include <pxr/base/gf/range2d.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

class StagedBrepTopologyBuilder;

namespace occt
{

class OcctStagingDriver
{
public:

    OcctStagingDriver(const OcctFileSource& rSource, BrepStagingState& rState);

    // Populate the staging state from the source. Returns false (with rError) on failure.
    bool Run(std::string& rError);

private:

    struct ConnexReserve
    {
        uint32_t uiShellBlockSize = 0;
        uint32_t uiFaceuseBlockSize = 0;
        uint32_t uiShellPairedSpacingIncrement = 0;
        uint32_t uiFaceusePairedSpacingIncrement = 0;
    };

    StagedBrepTopologyBuilder MakeTopologyBuilder();

    uint32_t InfiniteRegionFaceuseCount() const;
    ConnexReserve ComputeConnexReserve(const OcctFileSource::Solid& rSolid) const;
    uint32_t ParentRegionIndexForCurrentShell(bool bFirstShell) const;

    bool DriveSolid(const OcctFileSource::Solid& rSolid, std::string& rError);
    bool DriveShell(const OcctFileSource::Shell& rShell, bool bFirstShell, std::string& rError);
    bool DriveFace(const OcctFileSource::Shell& rShell, const OcctFileSource::Face& rFace, bool bFirstShell, std::string& rError);
    bool StageEdgeForCoedge(const OcctFileSource::Coedge& rCoedge, uint32_t uiEdgeuseIndex, uint32_t& rEdgeIndex, std::string& rError);
    uint32_t StageVertex(int iVertexStorageIndex, const pxr::GfMatrix4d& rVertexLoc);
    uint32_t FindOrAddVertexAtPoint(const pxr::GfVec3d& rPoint);

    // Maps a face's native OCCT pcurve parameter coordinates to the (possibly shifted/rescaled) UV
    // domain we emit for that face, so staged edgeuse UV curves agree with face:range.
    struct FaceUvCurveContext
    {
        bool bParameterizationCompatible = false; // source pcurves map affinely to the authored face
        int iSurface = 0; // OCCT surface index the pcurves live on (== face surface)
        double dUScale = 1.0; // multiplicative U reflection for an indirect analytic Ax3; 1 otherwise
        double dUAdd = 0.0; // additive U shift applied to face:range (whole 2*pi periods)
        double dVScale = 1.0; // plane reflection or cone slant->axial rescale; 1 otherwise
        pxr::GfRange2d sDomain; // authored face:range used to reject individually incompatible pcurves
    };

    struct PeriodicBoundaryCoedge
    {
        bool bSynthetic = false;
        OcctFileSource::Coedge sSourceCoedge;
        StagedCurveData sSyntheticCurve;
        pxr::GfRange1d sSyntheticInterval;
        uint32_t uiSyntheticStartVertexIndex = kStagedNoObjectIndex;
        uint32_t uiSyntheticEndVertexIndex = kStagedNoObjectIndex;
        uint32_t uiSyntheticGroupIndex = kStagedNoObjectIndex;
        uint32_t uiSyntheticEdgeIndex = kStagedNoObjectIndex;
        bool bSameOrientationWithEdge = true;
    };

    struct PeriodicBoundaryLoop
    {
        std::vector<PeriodicBoundaryCoedge> vCoedges;
    };

    struct PeriodicBoundaryRepair
    {
        bool bEnabled = false;
        uint32_t uiOuterLoopIndex = 0;
        pxr::GfRange2d sDomain;
        std::vector<PeriodicBoundaryLoop> vLoops;
    };

    bool TryBuildPeriodicBoundaryRepair(
        const OcctFileSource::Face& rFace,
        const StagedSurfaceData& rSurfaceData,
        const pxr::GfRange2d& rPCurveBox,
        PeriodicBoundaryRepair& rRepair
    );
    bool TryBuildAnalyticSinglyPeriodicRepair(
        const OcctFileSource::Face& rFace,
        const StagedSurfaceData& rSurfaceData,
        const pxr::GfRange2d& rPCurveBox,
        PeriodicBoundaryRepair& rRepair
    );
    bool TryBuildNurbsSinglyPeriodicRepair(
        const OcctFileSource::Face& rFace,
        const StagedSurfaceData& rSurfaceData,
        const pxr::GfRange2d& rPCurveBox,
        PeriodicBoundaryRepair& rRepair
    );
    bool TryBuildDoublyPeriodicBoundaryRepair(
        const OcctFileSource::Face& rFace,
        const StagedSurfaceData& rSurfaceData,
        const pxr::GfRange2d& rPCurveBox,
        PeriodicBoundaryRepair& rRepair
    );
    bool StagePeriodicBoundaryRepair(const PeriodicBoundaryRepair& rRepair, const FaceUvCurveContext& rUvContext, std::string& rError);

    // When the source and authored surface parameterizations are compatible, resolve the coedge's
    // pcurve, convert it to a 2D NURBS, and store it on the staged edgeuse.
    void StageEdgeuseUvCurve(
        const OcctFileSource::Coedge& rCoedge,
        uint32_t uiEdgeuseIndex,
        bool bStagedEdgeuseMatchesSourceOrient,
        const FaceUvCurveContext& rContext
    );

    const OcctFileSource& m_rSource;
    BrepStagingState& m_rState;

    // Staged topology cursor (mirrors the A3DVisitorBrep member set).
    uint32_t m_uiConnexCount = 0;
    uint32_t m_uiSolidRegion_TgtIndex = 0;
    uint32_t m_uiShell_TgtIndex = 0;
    uint32_t m_uiShell_PairedSpacing = 0;
    uint32_t m_uiInfiniteFaceuse_TgtIndex = 0;
    uint32_t m_uiFaceuse_TgtIndex = 0;
    uint32_t m_uiFaceuse_PairedSpacing = 0;

    // Per-face outer-loop edgeuse cursor.
    uint32_t m_uiOuterLoop_EdgeusesTgtIndex = 0;
    uint32_t m_uiOuterLoop_EdgeuseCount = 0;

    // Location-aware dedup: the same TShape under different accumulated locations is a distinct
    // world instance, so edges/vertices are keyed by (storage index, accumulated transform).
    std::unordered_map<std::string, uint32_t> m_mEdgeIndexByKey;
    std::unordered_map<std::string, uint32_t> m_mVertexIndexByKey;
};

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_STAGING_DRIVER_H
