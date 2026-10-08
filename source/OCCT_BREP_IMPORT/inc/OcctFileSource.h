// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_FILE_SOURCE_H
#define OCCT_BREP_IMPORT_OCCT_FILE_SOURCE_H

#include "OcctFileRecords.h"
#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/range2d.h>
#include <pxr/base/gf/vec3d.h>

#include <cstdint>
#include <string>
#include <vector>

namespace occt
{

class OcctFileSource
{
public:

    struct Coedge
    {
        int iEdgeStorageIndex = 0; // 1-based index into SOcctModel::sShapes
        bool bSameOrientationWithLoop = true; // edge-in-wire orientation converted to SMLib edgeuse orientation
        pxr::GfMatrix4d mEdgeLoc = pxr::GfMatrix4d(1.0); // accumulated location down to the edge
    };

    struct Loop
    {
        bool bSameOrientationWithSurface = true; // wire-in-face orientation == Forward
        std::vector<Coedge> vCoedges;
    };

    struct Face
    {
        int iFaceStorageIndex = 0;
        int iSurfaceIndex = 0; // 1-based into SOcctModel::sSurfaces (0 == none)
        int iSurfaceLocation = 0;
        bool bSameOrientationWithShell = true; // face-in-shell orientation == Forward
        bool bNaturalRestriction = false; // boundary is the surface's natural param range (no wires)
        uint32_t uiOuterLoopIndex = 0;
        pxr::GfMatrix4d mFaceLoc = pxr::GfMatrix4d(1.0); // accumulated location down to the face
        std::vector<Loop> vLoops;
    };

    struct Shell
    {
        int iShellStorageIndex = 0;
        bool bClosed = false;
        bool bSameOrientationWithSolid = true; // effective shell orientation after container composition
        std::vector<Face> vFaces;
    };

    struct Solid
    {
        int iSolidStorageIndex = 0;
        std::vector<Shell> vShells;
    };

    // A coedge's 2D parametric (pcurve) representation on a given face surface.
    struct CoedgePCurve
    {
        const SCurve2d* pCurve2d = nullptr; // null when the coedge has no pcurve on the surface
        double dFirst = 0.0;
        double dLast = 0.0;
    };

    // Resolved geometry for one edge.
    struct EdgeGeometry
    {
        const SCurve3d* pCurve3d = nullptr; // null if the edge has no 3D curve (e.g. degenerate)
        int iCurveLocation = 0;
        double dFirst = 0.0;
        double dLast = 0.0;
        int iStartVertexStorageIndex = 0; // Forward vertex (0 == none)
        int iEndVertexStorageIndex = 0; // Reversed vertex
        int iStartVertexLocation = 0; // edge->start-vertex reference location
        int iEndVertexLocation = 0; // edge->end-vertex reference location
        bool bDegenerated = false;
    };

    explicit OcctFileSource(const SOcctModel& rModel);

    // Resolve the root shape into the nested solid view. Returns false (with rError) when the model
    // is not a single supported manifold solid.
    bool Build(std::string& rError);

    double Tolerance() const
    {
        return m_dTolerance;
    }

    const std::vector<Solid>& Solids() const
    {
        return m_vSolids;
    }

    const SOcctModel& Model() const
    {
        return m_rModel;
    }

    // Geometry / vertex accessors keyed by 1-based storage index.
    EdgeGeometry ResolveEdge(int iEdgeStorageIndex) const;
    pxr::GfVec3d VertexPoint(int iVertexStorageIndex) const;
    double VertexTolerance(int iVertexStorageIndex) const;

    // Bounding UV box of a face's parametric domain, derived from its edges' 2D pcurves on the
    // face surface. Returns an empty range when no pcurves are available (caller should fall back).
    pxr::GfRange2d ComputeFaceUVDomain(const Face& rFace) const;

    // Bounding UV box of a single loop, from its edges' 2D pcurves on surface iSurface. Empty when
    // the loop has no usable pcurves on that surface.
    pxr::GfRange2d ComputeLoopUVRange(const Loop& rLoop, int iSurface) const;

    // The 2D pcurve a coedge uses on face surface iSurface. For a seam edge (one edge carrying two
    // pcurves on the same closed surface), the two wire uses land on opposite sides (U=Umin / U=Umax)
    // of the parameter rectangle. Returns false (pCurve2d == null) when no pcurve on iSurface is found.
    bool ResolveCoedgePCurve(const Coedge& rCoedge, int iSurface, CoedgePCurve& rOut) const;

    // Prepare a loop for BrepArray staging: demote OCCT degenerated pole edges (same start/end
    // vertex) to vertex singularities by removing their coedges, then order the remaining coedges as a
    // connected cycle. Raw file loops keep their pole coedges until face-domain detection has used their
    // pcurves, so this is called by the staging driver rather than BuildLoop.
    void PrepareCoedgesForStaging(std::vector<Coedge>& rCoedges) const;

private:

    const SShapeRecord* Shape(int iStorageIndex) const;
    // Recursively gather connected components from a shape reference: Solid -> shells; bare Shell ->
    // single-shell component; bare Face -> synthetic open single-face shell; Compound/CompSolid ->
    // recurse into children. Each component is appended to m_vSolids (a "Solid" here models a
    // connected component, which may be an open sheet when its shell is not closed).
    bool AddComponentsFromRef(const SSubShapeRef& rRef, const pxr::GfMatrix4d& rAccLoc, bool bParentReversed, std::string& rError);
    bool BuildSolid(int iSolidStorageIndex, const pxr::GfMatrix4d& rAccLoc, bool bSolidReversed, Solid& rSolid, std::string& rError);
    bool BuildShell(const SSubShapeRef& rShellRef, const pxr::GfMatrix4d& rAccLoc, bool bParentReversed, Shell& rShell, std::string& rError);
    bool BuildFace(const SSubShapeRef& rFaceRef, const pxr::GfMatrix4d& rAccLoc, bool bParentReversed, Face& rFace, std::string& rError);
    bool BuildLoop(const SSubShapeRef& rWireRef, const pxr::GfMatrix4d& rAccLoc, Loop& rLoop, std::string& rError);
    // Match BRepTools_WireExplorer-style connection order for consumers that walk loop edgeuses
    // sequentially. The serialized TShape child list is not always already in that order; repeated
    // seam edges are handled when vertex connectivity makes the next coedge unambiguous.
    void OrderCoedgesIntoConnectedChain(std::vector<Coedge>& rCoedges) const;

    const SOcctModel& m_rModel;
    double m_dTolerance = 0.0;
    std::vector<Solid> m_vSolids;
};

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_FILE_SOURCE_H
