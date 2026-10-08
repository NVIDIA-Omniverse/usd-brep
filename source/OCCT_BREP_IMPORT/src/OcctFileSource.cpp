// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OcctFileSource.h"

#include "OcctAsciiParser.h" // ShapeTypeName
#include "OcctGeometryConvert.h" // ResolveLocation

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace occt
{

namespace
{

bool ComposeReversedOrientation(EOrientation eOrientation, bool bParentReversed, bool& rReversed, std::string& rError)
{
    if (eOrientation == EOrientation::Forward)
    {
        rReversed = bParentReversed;
        return true;
    }
    if (eOrientation == EOrientation::Reversed)
    {
        rReversed = !bParentReversed;
        return true;
    }
    rError = "component has unsupported INTERNAL/EXTERNAL orientation";
    return false;
}

// Evaluate a 2D pcurve at parameter t (parametric/UV space; no location applies to pcurves).
// Trimmed/Offset wrappers are unwrapped to their basis (the offset magnitude is ignored for the
// purpose of bounding the parametric domain).
SPnt2 EvalCurve2d(const SCurve2d& rCurve, double t)
{
    const SCurve2d* pCurve = &rCurve;
    while (pCurve && (pCurve->eType == ECurveType::Trimmed || pCurve->eType == ECurveType::Offset) && pCurve->pBasis)
    {
        pCurve = pCurve->pBasis.get();
    }
    if (!pCurve)
    {
        return SPnt2{};
    }

    switch (pCurve->eType)
    {
        case ECurveType::Line:
            return SPnt2{ pCurve->sLocation.dX + t * pCurve->sXAxis.dX, pCurve->sLocation.dY + t * pCurve->sXAxis.dY };
        case ECurveType::Circle:
        {
            const double c = std::cos(t);
            const double s = std::sin(t);
            return SPnt2{ pCurve->sLocation.dX + pCurve->dRadius * (c * pCurve->sXAxis.dX + s * pCurve->sYAxis.dX),
                          pCurve->sLocation.dY + pCurve->dRadius * (c * pCurve->sXAxis.dY + s * pCurve->sYAxis.dY) };
        }
        case ECurveType::Ellipse:
        {
            const double c = std::cos(t);
            const double s = std::sin(t);
            return SPnt2{ pCurve->sLocation.dX + pCurve->dRadius * c * pCurve->sXAxis.dX + pCurve->dMinorRadius * s * pCurve->sYAxis.dX,
                          pCurve->sLocation.dY + pCurve->dRadius * c * pCurve->sXAxis.dY + pCurve->dMinorRadius * s * pCurve->sYAxis.dY };
        }
        default:
            // Bezier/BSpline/conics we don't evaluate exactly: fall back to the control-pole hull,
            // handled by the caller via the poles list. Return the first pole if present.
            if (!pCurve->sPoles.empty())
            {
                return pCurve->sPoles.front();
            }
            return SPnt2{ pCurve->sLocation.dX, pCurve->sLocation.dY };
    }
}

void AccumulatePCurveExtent(const SCurve2d& rCurve, double dFirst, double dLast, pxr::GfRange2d& rRange)
{
    // For free-form curves, trim the source NURBS to the representation interval before using its
    // positive-weight control hull. Folding in the untrimmed basis poles can spuriously enlarge a
    // face domain when an edge uses only a sub-interval of that basis curve.
    const SCurve2d* pBasis = &rCurve;
    while (pBasis && (pBasis->eType == ECurveType::Trimmed || pBasis->eType == ECurveType::Offset) && pBasis->pBasis)
    {
        pBasis = pBasis->pBasis.get();
    }
    if (pBasis && (pBasis->eType == ECurveType::Bezier || pBasis->eType == ECurveType::BSpline))
    {
        StagedCurveData2d sTrimmed;
        if (ConvertCurve2dToNurb(rCurve, pxr::GfRange1d(dFirst, dLast), pxr::GfVec2d(1.0, 1.0), pxr::GfVec2d(0.0, 0.0), sTrimmed))
        {
            for (const pxr::GfVec2d& rPole : sTrimmed.vNurbControlVertices)
            {
                rRange.UnionWith(rPole);
            }
            return;
        }
        for (const SPnt2& rPole : pBasis->sPoles)
        {
            rRange.UnionWith(pxr::GfVec2d(rPole.dX, rPole.dY));
        }
        return;
    }

    constexpr int kSamples = 8;
    for (int ii = 0; ii <= kSamples; ++ii)
    {
        const double t = dFirst + (dLast - dFirst) * (static_cast<double>(ii) / static_cast<double>(kSamples));
        const SPnt2 sP = EvalCurve2d(rCurve, t);
        rRange.UnionWith(pxr::GfVec2d(sP.dX, sP.dY));
    }
}

} // namespace

OcctFileSource::OcctFileSource(const SOcctModel& rModel) : m_rModel(rModel)
{
    for (const SShapeRecord& rShape : m_rModel.sShapes)
    {
        double dTolerance = 0.0;
        switch (rShape.eType)
        {
            case EShapeType::Vertex:
                dTolerance = rShape.sVertex.dTolerance;
                break;
            case EShapeType::Edge:
                dTolerance = rShape.sEdge.dTolerance;
                break;
            case EShapeType::Face:
                dTolerance = rShape.sFace.dTolerance;
                break;
            default:
                continue;
        }
        if (std::isfinite(dTolerance) && dTolerance >= 0.0)
        {
            m_dTolerance = std::max(m_dTolerance, dTolerance);
        }
    }

    // BrepArray requires a usable Brep-wide tolerance even if a malformed source has none.
    if (m_dTolerance <= 0.0)
    {
        m_dTolerance = 1.0e-6;
    }
}

const SShapeRecord* OcctFileSource::Shape(int iStorageIndex) const
{
    if (iStorageIndex < 1 || static_cast<size_t>(iStorageIndex) > m_rModel.sShapes.size())
    {
        return nullptr;
    }
    return &m_rModel.sShapes[static_cast<size_t>(iStorageIndex - 1)];
}

bool OcctFileSource::Build(std::string& rError)
{
    m_vSolids.clear();

    if (!m_rModel.bHasRoot)
    {
        rError = "model has no root shape";
        return false;
    }

    const SShapeRecord* pRoot = Shape(m_rModel.sRoot.iStorageIndex);
    if (!pRoot)
    {
        rError = "root shape storage index out of range";
        return false;
    }

    // Gather connected components from the root (Solid / Shell / Face / Compound / CompSolid).
    // The root reference itself carries a location; AddComponentsFromRef applies it.
    if (!AddComponentsFromRef(m_rModel.sRoot, pxr::GfMatrix4d(1.0), false, rError))
    {
        return false;
    }

    if (m_vSolids.empty())
    {
        rError = std::string("no Solid/Shell/Face component found under the root shape (type '") + ShapeTypeName(pRoot->eType) + "')";
        return false;
    }
    return true;
}

bool OcctFileSource::AddComponentsFromRef(const SSubShapeRef& rRef, const pxr::GfMatrix4d& rAccLoc, bool bParentReversed, std::string& rError)
{
    const SShapeRecord* pShape = Shape(rRef.iStorageIndex);
    if (!pShape)
    {
        rError = "component reference storage index out of range";
        return false;
    }

    switch (pShape->eType)
    {
        case EShapeType::Solid:
        {
            // BuildSolid expects the solid's already-accumulated location (incl. its own ref loc).
            const pxr::GfMatrix4d sSolidLoc = ResolveLocation(m_rModel, rRef.iLocation) * rAccLoc;
            bool bSolidReversed = false;
            if (!ComposeReversedOrientation(rRef.eOrientation, bParentReversed, bSolidReversed, rError))
            {
                return false;
            }
            Solid sSolid;
            if (!BuildSolid(rRef.iStorageIndex, sSolidLoc, bSolidReversed, sSolid, rError))
            {
                return false;
            }
            m_vSolids.push_back(std::move(sSolid));
            return true;
        }
        case EShapeType::Shell:
        {
            // Bare shell (sheet body or open boundary): one component, one shell. BuildShell applies
            // the shell ref's own location onto rAccLoc.
            Solid sComponent;
            Shell sShell;
            if (!BuildShell(rRef, rAccLoc, bParentReversed, sShell, rError))
            {
                return false;
            }
            sComponent.vShells.push_back(std::move(sShell));
            m_vSolids.push_back(std::move(sComponent));
            return true;
        }
        case EShapeType::Face:
        {
            // Bare face: synthesize an open single-face shell. BuildFace applies the face ref's loc.
            Solid sComponent;
            Shell sShell;
            sShell.bClosed = false;
            Face sFace;
            if (!BuildFace(rRef, rAccLoc, bParentReversed, sFace, rError))
            {
                return false;
            }
            sShell.vFaces.push_back(std::move(sFace));
            sComponent.vShells.push_back(std::move(sShell));
            m_vSolids.push_back(std::move(sComponent));
            return true;
        }
        case EShapeType::Compound:
        case EShapeType::CompSolid:
        {
            // Recurse into children, folding in the compound's own reference location. A malformed
            // child (e.g. a degenerate solid with no shell) is skipped rather than aborting the whole
            // assembly; Build() reports an error only if no component is produced at all.
            const pxr::GfMatrix4d sChildAcc = ResolveLocation(m_rModel, rRef.iLocation) * rAccLoc;
            bool bCompoundReversed = false;
            if (!ComposeReversedOrientation(rRef.eOrientation, bParentReversed, bCompoundReversed, rError))
            {
                return false;
            }
            for (const SSubShapeRef& rChildRef : pShape->sSubShapes)
            {
                const SShapeRecord* pChild = Shape(rChildRef.iStorageIndex);
                if (!pChild)
                {
                    continue;
                }
                // Only descend into shape kinds that can yield faces.
                if (pChild->eType == EShapeType::Solid || pChild->eType == EShapeType::Shell || pChild->eType == EShapeType::Face ||
                    pChild->eType == EShapeType::Compound || pChild->eType == EShapeType::CompSolid)
                {
                    std::string sChildError;
                    AddComponentsFromRef(rChildRef, sChildAcc, bCompoundReversed, sChildError);
                }
            }
            return true;
        }
        default:
            // Wires/edges/vertices as a root or compound child carry no faces — skip silently inside
            // a compound; at the very top this leaves m_vSolids empty and Build() reports it.
            return true;
    }
}

bool OcctFileSource::BuildSolid(int iSolidStorageIndex, const pxr::GfMatrix4d& rAccLoc, bool bSolidReversed, Solid& rSolid, std::string& rError)
{
    const SShapeRecord* pSolid = Shape(iSolidStorageIndex);
    if (!pSolid)
    {
        rError = "solid storage index out of range";
        return false;
    }

    rSolid.iSolidStorageIndex = iSolidStorageIndex;
    for (const SSubShapeRef& rShellRef : pSolid->sSubShapes)
    {
        const SShapeRecord* pShell = Shape(rShellRef.iStorageIndex);
        if (!pShell || pShell->eType != EShapeType::Shell)
        {
            continue;
        }
        Shell sShell;
        if (!BuildShell(rShellRef, rAccLoc, bSolidReversed, sShell, rError))
        {
            return false;
        }
        rSolid.vShells.push_back(std::move(sShell));
    }

    if (rSolid.vShells.empty())
    {
        rError = "solid has no shells";
        return false;
    }
    return true;
}

bool OcctFileSource::BuildShell(const SSubShapeRef& rShellRef, const pxr::GfMatrix4d& rAccLoc, bool bParentReversed, Shell& rShell, std::string& rError)
{
    const SShapeRecord* pShell = Shape(rShellRef.iStorageIndex);
    if (!pShell)
    {
        rError = "shell storage index out of range";
        return false;
    }

    rShell.iShellStorageIndex = rShellRef.iStorageIndex;
    rShell.bClosed = pShell->bClosed;
    bool bShellReversed = false;
    if (!ComposeReversedOrientation(rShellRef.eOrientation, bParentReversed, bShellReversed, rError))
    {
        return false;
    }
    rShell.bSameOrientationWithSolid = !bShellReversed;

    const pxr::GfMatrix4d sShellLoc = ResolveLocation(m_rModel, rShellRef.iLocation) * rAccLoc;

    for (const SSubShapeRef& rFaceRef : pShell->sSubShapes)
    {
        const SShapeRecord* pFace = Shape(rFaceRef.iStorageIndex);
        if (!pFace || pFace->eType != EShapeType::Face)
        {
            continue;
        }
        Face sFace;
        if (!BuildFace(rFaceRef, sShellLoc, false, sFace, rError))
        {
            return false;
        }
        rShell.vFaces.push_back(std::move(sFace));
    }

    if (rShell.vFaces.empty())
    {
        rError = "shell has no faces";
        return false;
    }
    return true;
}

bool OcctFileSource::BuildFace(const SSubShapeRef& rFaceRef, const pxr::GfMatrix4d& rAccLoc, bool bParentReversed, Face& rFace, std::string& rError)
{
    const SShapeRecord* pFace = Shape(rFaceRef.iStorageIndex);
    if (!pFace)
    {
        rError = "face storage index out of range";
        return false;
    }

    rFace.iFaceStorageIndex = rFaceRef.iStorageIndex;
    rFace.iSurfaceIndex = pFace->sFace.iSurface;
    rFace.iSurfaceLocation = pFace->sFace.iLocation;
    rFace.bNaturalRestriction = pFace->sFace.bNaturalRestriction;
    bool bFaceReversed = false;
    if (!ComposeReversedOrientation(rFaceRef.eOrientation, bParentReversed, bFaceReversed, rError))
    {
        return false;
    }
    rFace.bSameOrientationWithShell = !bFaceReversed;
    rFace.mFaceLoc = ResolveLocation(m_rModel, rFaceRef.iLocation) * rAccLoc;

    for (const SSubShapeRef& rWireRef : pFace->sSubShapes)
    {
        const SShapeRecord* pWire = Shape(rWireRef.iStorageIndex);
        if (!pWire || pWire->eType != EShapeType::Wire)
        {
            continue;
        }
        Loop sLoop;
        if (!BuildLoop(rWireRef, rFace.mFaceLoc, sLoop, rError))
        {
            return false;
        }
        rFace.vLoops.push_back(std::move(sLoop));
    }

    // A natural-restriction face legitimately has no wires: the surface's natural parameter range is
    // the boundary. Such faces are driven through the implied-natural-boundary path in OcctStagingDriver.
    if (rFace.vLoops.empty() && !rFace.bNaturalRestriction)
    {
        rError = "face has no loops";
        return false;
    }

    // The file does not flag the outer wire. OCCT writes it first, so default to loop 0 (and faces
    // with a single loop, e.g. all box faces, are unambiguous). For a multi-loop face (a face with
    // holes), confirm geometrically instead of trusting write order: the outer loop's parametric
    // bounding box encloses every hole, so it has the largest UV-domain area. Falls back to loop 0
    // when any loop lacks usable pcurves (no 2D representation to measure).
    rFace.uiOuterLoopIndex = 0;
    if (rFace.vLoops.size() > 1)
    {
        double dBestArea = -1.0;
        uint32_t uiBestLoop = 0;
        bool bAllLoopsMeasurable = true;
        for (size_t iiLoop = 0; iiLoop < rFace.vLoops.size(); ++iiLoop)
        {
            const pxr::GfRange2d sLoopRange = ComputeLoopUVRange(rFace.vLoops[iiLoop], rFace.iSurfaceIndex);
            if (sLoopRange.IsEmpty())
            {
                bAllLoopsMeasurable = false;
                break;
            }
            const pxr::GfVec2d sSize = sLoopRange.GetSize();
            const double dArea = sSize[0] * sSize[1];
            if (dArea > dBestArea)
            {
                dBestArea = dArea;
                uiBestLoop = static_cast<uint32_t>(iiLoop);
            }
        }
        if (bAllLoopsMeasurable)
        {
            rFace.uiOuterLoopIndex = uiBestLoop;
        }
    }
    return true;
}

bool OcctFileSource::BuildLoop(const SSubShapeRef& rWireRef, const pxr::GfMatrix4d& rAccLoc, Loop& rLoop, std::string& rError)
{
    const SShapeRecord* pWire = Shape(rWireRef.iStorageIndex);
    if (!pWire)
    {
        rError = "wire storage index out of range";
        return false;
    }

    rLoop.bSameOrientationWithSurface = (rWireRef.eOrientation == EOrientation::Forward);

    const pxr::GfMatrix4d sWireLoc = ResolveLocation(m_rModel, rWireRef.iLocation) * rAccLoc;

    for (const SSubShapeRef& rEdgeRef : pWire->sSubShapes)
    {
        const SShapeRecord* pEdge = Shape(rEdgeRef.iStorageIndex);
        if (!pEdge || pEdge->eType != EShapeType::Edge)
        {
            continue;
        }
        Coedge sCoedge;
        sCoedge.iEdgeStorageIndex = rEdgeRef.iStorageIndex;
        if (rEdgeRef.eOrientation != EOrientation::Forward && rEdgeRef.eOrientation != EOrientation::Reversed)
        {
            rError = "wire edge has unsupported INTERNAL/EXTERNAL orientation";
            return false;
        }
        sCoedge.bSameOrientationWithLoop = (rEdgeRef.eOrientation == EOrientation::Forward);
        sCoedge.mEdgeLoc = ResolveLocation(m_rModel, rEdgeRef.iLocation) * sWireLoc;
        rLoop.vCoedges.push_back(sCoedge);
    }

    if (rLoop.vCoedges.empty())
    {
        rError = "wire has no edges";
        return false;
    }

    OrderCoedgesIntoConnectedChain(rLoop.vCoedges);
    return true;
}

void OcctFileSource::PrepareCoedgesForStaging(std::vector<Coedge>& rCoedges) const
{
    std::vector<Coedge> vKept;
    vKept.reserve(rCoedges.size());
    bool bDroppedPole = false;
    for (const Coedge& rCoedge : rCoedges)
    {
        const EdgeGeometry sEdge = ResolveEdge(rCoedge.iEdgeStorageIndex);
        if (sEdge.bDegenerated && sEdge.iStartVertexStorageIndex != 0 && sEdge.iStartVertexStorageIndex == sEdge.iEndVertexStorageIndex)
        {
            bDroppedPole = true;
            continue;
        }
        vKept.push_back(rCoedge);
    }
    if (bDroppedPole)
    {
        rCoedges.swap(vKept);
    }

    OrderCoedgesIntoConnectedChain(rCoedges);
}

void OcctFileSource::OrderCoedgesIntoConnectedChain(std::vector<Coedge>& rCoedges) const
{
    const size_t uiCount = rCoedges.size();
    if (uiCount < 3)
    {
        return; // a 1- or 2-edge loop may be a closed single curve or a degenerate sliver.
    }

    // Directed boundary endpoints (start -> end vertex storage indices) implied by each coedge's
    // edge curve direction (Forward vertex -> Reversed vertex) and the coedge's own orientation.
    std::vector<int> vStart(uiCount, 0);
    std::vector<int> vEnd(uiCount, 0);
    for (size_t i = 0; i < uiCount; ++i)
    {
        const EdgeGeometry sEdge = ResolveEdge(rCoedges[i].iEdgeStorageIndex);
        int iV0 = sEdge.iStartVertexStorageIndex;
        int iV1 = sEdge.iEndVertexStorageIndex;
        if (iV0 == 0 || iV1 == 0 || sEdge.bDegenerated)
        {
            return; // unresolved endpoints or a degenerate/pole edge: preserve authored order.
        }
        if (!rCoedges[i].bSameOrientationWithLoop)
        {
            std::swap(iV0, iV1);
        }
        vStart[i] = iV0;
        vEnd[i] = iV1;
    }

    // Order the directed coedges as an Eulerian loop, anchored at the first authored coedge.
    // This keeps repeated seam-edge cases deterministic without recursive backtracking.
    std::unordered_map<int, int> mDegreeBalance;
    std::unordered_map<int, std::vector<size_t>> mOutgoing;
    mDegreeBalance.reserve(uiCount * 2);
    mOutgoing.reserve(uiCount);
    for (size_t i = 0; i < uiCount; ++i)
    {
        ++mDegreeBalance[vStart[i]];
        --mDegreeBalance[vEnd[i]];
        if (i != 0)
        {
            mOutgoing[vStart[i]].push_back(i);
        }
    }
    for (const auto& rEntry : mDegreeBalance)
    {
        if (rEntry.second != 0)
        {
            return; // not a closed directed loop; preserve authored order.
        }
    }
    for (auto& rEntry : mOutgoing)
    {
        std::reverse(rEntry.second.begin(), rEntry.second.end());
    }

    std::vector<int> vVertexStack;
    std::vector<size_t> vEdgeStack;
    std::vector<size_t> vTailReverse;
    vVertexStack.reserve(uiCount);
    vEdgeStack.reserve(uiCount);
    vTailReverse.reserve(uiCount - 1);
    vVertexStack.push_back(vEnd[0]);

    while (!vVertexStack.empty())
    {
        const int iVertex = vVertexStack.back();
        auto sIt = mOutgoing.find(iVertex);
        if (sIt != mOutgoing.end() && !sIt->second.empty())
        {
            const size_t uiEdge = sIt->second.back();
            sIt->second.pop_back();
            vEdgeStack.push_back(uiEdge);
            vVertexStack.push_back(vEnd[uiEdge]);
            continue;
        }

        vVertexStack.pop_back();
        if (!vEdgeStack.empty())
        {
            vTailReverse.push_back(vEdgeStack.back());
            vEdgeStack.pop_back();
        }
    }

    if (vTailReverse.size() + 1 != uiCount)
    {
        return; // disconnected or otherwise not orderable from the first coedge.
    }

    std::vector<size_t> vOrder;
    vOrder.reserve(uiCount);
    vOrder.push_back(0);
    for (auto it = vTailReverse.rbegin(); it != vTailReverse.rend(); ++it)
    {
        vOrder.push_back(*it);
    }
    for (size_t i = 1; i < vOrder.size(); ++i)
    {
        if (vEnd[vOrder[i - 1]] != vStart[vOrder[i]])
        {
            return;
        }
    }
    if (vEnd[vOrder.back()] != vStart[vOrder.front()])
    {
        return;
    }

    std::vector<Coedge> vChain;
    vChain.reserve(uiCount);
    for (size_t uiIndex : vOrder)
    {
        vChain.push_back(rCoedges[uiIndex]);
    }
    rCoedges.swap(vChain);
}

OcctFileSource::EdgeGeometry OcctFileSource::ResolveEdge(int iEdgeStorageIndex) const
{
    EdgeGeometry sResult;
    const SShapeRecord* pEdge = Shape(iEdgeStorageIndex);
    if (!pEdge || pEdge->eType != EShapeType::Edge)
    {
        return sResult;
    }

    sResult.bDegenerated = pEdge->sEdge.bDegenerated;

    // 3D curve representation (kind == 1).
    for (const SCurveRepr& rRepr : pEdge->sEdge.sReprs)
    {
        if (rRepr.iKind == 1 && rRepr.iCurve3d >= 1 && static_cast<size_t>(rRepr.iCurve3d) <= m_rModel.sCurves.size())
        {
            sResult.pCurve3d = &m_rModel.sCurves[static_cast<size_t>(rRepr.iCurve3d - 1)];
            sResult.iCurveLocation = rRepr.iLocation;
            sResult.dFirst = rRepr.dFirst;
            sResult.dLast = rRepr.dLast;
            break;
        }
    }

    // Start/end vertices: edge sub-shapes are vertices; Forward == start, Reversed == end.
    for (const SSubShapeRef& rRef : pEdge->sSubShapes)
    {
        const SShapeRecord* pVertex = Shape(rRef.iStorageIndex);
        if (!pVertex || pVertex->eType != EShapeType::Vertex)
        {
            continue;
        }
        if (rRef.eOrientation == EOrientation::Reversed)
        {
            sResult.iEndVertexStorageIndex = rRef.iStorageIndex;
            sResult.iEndVertexLocation = rRef.iLocation;
        }
        else
        {
            sResult.iStartVertexStorageIndex = rRef.iStorageIndex;
            sResult.iStartVertexLocation = rRef.iLocation;
        }
    }

    return sResult;
}

pxr::GfVec3d OcctFileSource::VertexPoint(int iVertexStorageIndex) const
{
    const SShapeRecord* pVertex = Shape(iVertexStorageIndex);
    if (!pVertex || pVertex->eType != EShapeType::Vertex)
    {
        return pxr::GfVec3d(0.0, 0.0, 0.0);
    }
    const SPnt3& rPnt = pVertex->sVertex.sPoint;
    return pxr::GfVec3d(rPnt.dX, rPnt.dY, rPnt.dZ);
}

pxr::GfRange2d OcctFileSource::ComputeLoopUVRange(const Loop& rLoop, int iSurface) const
{
    pxr::GfRange2d sRange; // empty: min=+inf, max=-inf

    for (const Coedge& rCoedge : rLoop.vCoedges)
    {
        const SShapeRecord* pEdge = Shape(rCoedge.iEdgeStorageIndex);
        if (!pEdge || pEdge->eType != EShapeType::Edge)
        {
            continue;
        }

        for (const SCurveRepr& rRepr : pEdge->sEdge.sReprs)
        {
            const auto accumulatePCurve = [&](int iPCurveIndex)
            {
                if (iPCurveIndex < 1 || static_cast<size_t>(iPCurveIndex) > m_rModel.sCurve2ds.size())
                {
                    return;
                }

                const SCurve2d& rCurve2d = m_rModel.sCurve2ds[static_cast<size_t>(iPCurveIndex - 1)];
                AccumulatePCurveExtent(rCurve2d, rRepr.dFirst, rRepr.dLast, sRange);
            };

            // kind 2: pcurve on a surface; kind 3: two pcurves on one closed surface.
            if (rRepr.iKind == 2 && rRepr.iSurface == iSurface)
            {
                accumulatePCurve(rRepr.iPCurve);
            }
            else if (rRepr.iKind == 3 && rRepr.iSurface == iSurface)
            {
                accumulatePCurve(rRepr.iPCurve);
                accumulatePCurve(rRepr.iPCurve2);
            }
            else if (rRepr.iKind == 3 && rRepr.iSurface2 == iSurface)
            {
                accumulatePCurve(rRepr.iPCurve2);
            }
        }
    }

    return sRange;
}

bool OcctFileSource::ResolveCoedgePCurve(const Coedge& rCoedge, int iSurface, CoedgePCurve& rOut) const
{
    rOut = CoedgePCurve{};

    const SShapeRecord* pEdge = Shape(rCoedge.iEdgeStorageIndex);
    if (!pEdge || pEdge->eType != EShapeType::Edge)
    {
        return false;
    }

    // A seam edge stores two pcurves on the same closed surface (kind 3). Pick by the coedge's edge
    // orientation so the two uses land on opposite sides of the parameter rectangle. Non-seam edges
    // (kind 2, or kind 3 sharing the surface with a neighbor face) resolve to their single pcurve.
    for (const SCurveRepr& rRepr : pEdge->sEdge.sReprs)
    {
        int iPCurveIndex = 0;
        if (rRepr.iKind == 2 && rRepr.iSurface == iSurface)
        {
            iPCurveIndex = rRepr.iPCurve;
        }
        else if (rRepr.iKind == 3 && rRepr.iSurface == iSurface && rRepr.iPCurve2 >= 1)
        {
            // Seam edge: one edge carrying two pcurves on the same closed surface (kind 3 stores both
            // under a single surface index). The two wire uses have opposite orientation, so select
            // opposite pcurves to keep the uses on opposite sides (U=Umin / U=Umax) of the parameter
            // rectangle.
            iPCurveIndex = rCoedge.bSameOrientationWithLoop ? rRepr.iPCurve : rRepr.iPCurve2;
        }
        else if (rRepr.iKind == 3 && rRepr.iSurface == iSurface)
        {
            iPCurveIndex = rRepr.iPCurve;
        }
        else if (rRepr.iKind == 3 && rRepr.iSurface2 == iSurface)
        {
            iPCurveIndex = rRepr.iPCurve2;
        }

        if (iPCurveIndex < 1 || static_cast<size_t>(iPCurveIndex) > m_rModel.sCurve2ds.size())
        {
            continue;
        }

        rOut.pCurve2d = &m_rModel.sCurve2ds[static_cast<size_t>(iPCurveIndex - 1)];
        rOut.dFirst = rRepr.dFirst;
        rOut.dLast = rRepr.dLast;
        return true;
    }

    return false;
}

pxr::GfRange2d OcctFileSource::ComputeFaceUVDomain(const Face& rFace) const
{
    pxr::GfRange2d sRange; // empty: min=+inf, max=-inf
    for (const Loop& rLoop : rFace.vLoops)
    {
        sRange.UnionWith(ComputeLoopUVRange(rLoop, rFace.iSurfaceIndex));
    }
    return sRange;
}

double OcctFileSource::VertexTolerance(int iVertexStorageIndex) const
{
    const SShapeRecord* pVertex = Shape(iVertexStorageIndex);
    if (!pVertex || pVertex->eType != EShapeType::Vertex)
    {
        return 0.0;
    }
    return pVertex->sVertex.dTolerance;
}

} // namespace occt
