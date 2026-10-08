// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "UsdBrepTopologyQueries.h"

#include "UsdBrepTokens.h"

#include <algorithm>
#include <unordered_set>

namespace UsdBrep
{
using namespace UsdBrepData;

UsdBrepTopologyQueries::UsdBrepTopologyQueries(const UsdBrepView& brep) : m_rBrep(brep)
{
}

/*********************************************************************************************************************
 * Build edge -> edgeuse index map (all indices local).
 ********************************************************************************************************************/
void UsdBrepTopologyQueries::BuildEdgeToEdgeuseMap() const
{
    if (m_bEdgeToEdgeuseBuilt)
    {
        return;
    }

    for (uint32_t eu = 0; eu < m_rBrep.EdgeuseCount(); ++eu)
    {
        uint32_t localEdge = m_rBrep.EdgeuseLocalEdgeIndex(eu);
        m_mEdgeToEdgeuses[localEdge].push_back(eu);
    }

    m_bEdgeToEdgeuseBuilt = true;
}

/*********************************************************************************************************************
 * Build edgeuse -> face map (all indices local).
 ********************************************************************************************************************/
void UsdBrepTopologyQueries::BuildEdgeuseToFaceMap() const
{
    if (m_bEdgeuseToFaceBuilt)
    {
        return;
    }

    auto faceLoopCounts = m_rBrep.FaceLoopCounts();
    auto loopEuCounts = m_rBrep.LoopEdgeuseCounts();

    uint32_t loopCursor = 0;
    uint32_t euCursor = 0;

    // Guard against inconsistent counts (malformed data): never index past the
    // span sizes, otherwise the cursors could walk out of bounds.
    const uint32_t faceCount = std::min<uint32_t>(m_rBrep.FaceCount(), static_cast<uint32_t>(faceLoopCounts.size()));
    for (uint32_t f = 0; f < faceCount; ++f)
    {
        uint32_t loopCount = faceLoopCounts[f];
        for (uint32_t l = 0; l < loopCount && loopCursor < loopEuCounts.size(); ++l)
        {
            uint32_t euCount = loopEuCounts[loopCursor];
            for (uint32_t e = 0; e < euCount; ++e)
            {
                m_mEdgeuseToFace[euCursor] = f;
                ++euCursor;
            }
            ++loopCursor;
        }
    }

    m_bEdgeuseToFaceBuilt = true;
}

/*********************************************************************************************************************
 * Build vertex -> edge map (all indices local).
 ********************************************************************************************************************/
void UsdBrepTopologyQueries::BuildVertexToEdgeMap() const
{
    if (m_bVertexToEdgeBuilt)
    {
        return;
    }

    uint32_t totalEdges = m_rBrep.EdgeCount();
    for (uint32_t e = 0; e < totalEdges; ++e)
    {
        GfVec2i verts = m_rBrep.EdgeLocalVertexIndices(e);
        uint32_t startV = static_cast<uint32_t>(verts[0]);
        uint32_t endV = static_cast<uint32_t>(verts[1]);
        m_mVertexToEdges[startV].push_back(e);
        if (endV != startV)
        {
            m_mVertexToEdges[endV].push_back(e);
        }
    }

    uint32_t totalWireEdges = m_rBrep.WireEdgeCount();
    for (uint32_t w = 0; w < totalWireEdges; ++w)
    {
        GfVec2i verts = m_rBrep.WireEdgeLocalVertexIndices(w);
        uint32_t startV = static_cast<uint32_t>(verts[0]);
        uint32_t endV = static_cast<uint32_t>(verts[1]);
        m_mVertexToEdges[startV].push_back(w + totalEdges);
        if (endV != startV)
        {
            m_mVertexToEdges[endV].push_back(w + totalEdges);
        }
    }

    m_bVertexToEdgeBuilt = true;
}

/*********************************************************************************************************************
 * GetFacesAdjacentToEdge
 ********************************************************************************************************************/
std::vector<uint32_t> UsdBrepTopologyQueries::GetFacesAdjacentToEdge(uint32_t iLocalEdge) const
{
    BuildEdgeToEdgeuseMap();
    BuildEdgeuseToFaceMap();

    std::unordered_set<uint32_t> faceSet;
    auto it = m_mEdgeToEdgeuses.find(iLocalEdge);
    if (it != m_mEdgeToEdgeuses.end())
    {
        for (uint32_t euIdx : it->second)
        {
            auto fit = m_mEdgeuseToFace.find(euIdx);
            if (fit != m_mEdgeuseToFace.end())
            {
                faceSet.insert(fit->second);
            }
        }
    }

    return std::vector<uint32_t>(faceSet.begin(), faceSet.end());
}

/*********************************************************************************************************************
 * GetEdgesBoundingFace
 ********************************************************************************************************************/
std::vector<uint32_t> UsdBrepTopologyQueries::GetEdgesBoundingFace(uint32_t iLocalFace) const
{
    auto faceLoopCounts = m_rBrep.FaceLoopCounts();
    auto loopEuCounts = m_rBrep.LoopEdgeuseCounts();

    if (iLocalFace >= faceLoopCounts.size())
    {
        return {};
    }

    uint32_t loopStart = 0;
    for (uint32_t f = 0; f < iLocalFace; ++f)
    {
        loopStart += faceLoopCounts[f];
    }

    uint32_t euStart = 0;
    for (uint32_t l = 0; l < loopStart && l < loopEuCounts.size(); ++l)
    {
        euStart += loopEuCounts[l];
    }

    std::unordered_set<uint32_t> edgeSet;
    uint32_t loopCount = faceLoopCounts[iLocalFace];
    for (uint32_t li = 0; li < loopCount; ++li)
    {
        uint32_t loopIdx = loopStart + li;
        if (loopIdx >= loopEuCounts.size())
        {
            break;
        }
        uint32_t euCount = loopEuCounts[loopIdx];
        for (uint32_t ei = 0; ei < euCount; ++ei)
        {
            uint32_t localEu = euStart + ei;
            if (localEu < m_rBrep.EdgeuseCount())
            {
                edgeSet.insert(m_rBrep.EdgeuseLocalEdgeIndex(localEu));
            }
        }
        euStart += euCount;
    }

    return std::vector<uint32_t>(edgeSet.begin(), edgeSet.end());
}

/*********************************************************************************************************************
 * AreFacesAdjacent
 ********************************************************************************************************************/
bool UsdBrepTopologyQueries::AreFacesAdjacent(uint32_t iLocalFace1, uint32_t iLocalFace2) const
{
    auto edges1 = GetEdgesBoundingFace(iLocalFace1);
    auto edges2 = GetEdgesBoundingFace(iLocalFace2);

    std::unordered_set<uint32_t> set1(edges1.begin(), edges1.end());
    for (uint32_t e : edges2)
    {
        if (set1.count(e))
        {
            return true;
        }
    }
    return false;
}

/*********************************************************************************************************************
 * GetEdgesAtVertex
 ********************************************************************************************************************/
std::vector<uint32_t> UsdBrepTopologyQueries::GetEdgesAtVertex(uint32_t iLocalVertex) const
{
    BuildVertexToEdgeMap();
    auto it = m_mVertexToEdges.find(iLocalVertex);
    if (it != m_mVertexToEdges.end())
    {
        return it->second;
    }
    return {};
}

/*********************************************************************************************************************
 * GetFacesAtVertex
 ********************************************************************************************************************/
std::vector<uint32_t> UsdBrepTopologyQueries::GetFacesAtVertex(uint32_t iLocalVertex) const
{
    auto edges = GetEdgesAtVertex(iLocalVertex);
    std::unordered_set<uint32_t> faceSet;

    uint32_t totalEdges = m_rBrep.EdgeCount();
    for (uint32_t e : edges)
    {
        if (e < totalEdges)
        {
            auto faces = GetFacesAdjacentToEdge(e);
            faceSet.insert(faces.begin(), faces.end());
        }
    }

    return std::vector<uint32_t>(faceSet.begin(), faceSet.end());
}

/*********************************************************************************************************************
 * GetRadialEdgeuses
 ********************************************************************************************************************/
std::vector<uint32_t> UsdBrepTopologyQueries::GetRadialEdgeuses(uint32_t iLocalEdgeuse) const
{
    std::vector<uint32_t> result;

    if (iLocalEdgeuse >= m_rBrep.EdgeuseCount())
    {
        return result;
    }

    result.push_back(iLocalEdgeuse);
    uint32_t current = m_rBrep.EdgeuseNextRadialLocalIndex(iLocalEdgeuse);

    uint32_t maxIter = m_rBrep.EdgeuseCount();
    uint32_t iter = 0;

    while (current != iLocalEdgeuse && iter < maxIter)
    {
        if (current >= m_rBrep.EdgeuseCount())
        {
            break;
        }
        result.push_back(current);
        current = m_rBrep.EdgeuseNextRadialLocalIndex(current);
        ++iter;
    }

    return result;
}

/*********************************************************************************************************************
 * GetFaceForEdgeuse
 ********************************************************************************************************************/
uint32_t UsdBrepTopologyQueries::GetFaceForEdgeuse(uint32_t iLocalEdgeuse) const
{
    BuildEdgeuseToFaceMap();
    auto it = m_mEdgeuseToFace.find(iLocalEdgeuse);
    if (it != m_mEdgeuseToFace.end())
    {
        return it->second;
    }
    return UINT32_MAX;
}

} // end namespace UsdBrep
