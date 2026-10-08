// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepTopologyQueries.h
 * PURPOSE: Adjacency and connectivity queries on a single Brep's topology via UsdBrepView.
 *
 * CONTAINS:
 *  namespace UsdBrep {
 *      UsdBrepTopologyQueries - lazily built adjacency tables with query methods
 *  }
 *
 * NOTES:
 *   Adjacency tables are built on first query and cached. The object holds a const
 *   reference to the UsdBrepView (which is non-copyable) and must not outlive it;
 *   in particular, do not construct this from a temporary view such as the one
 *   yielded by range-based iteration (`UsdBrepTopologyQueries q(*it)`). Bind the
 *   view to a named variable first.
 *   All indices are local to the Brep (not global UsdBrepArrayData indices).
 *
 *   Not thread-safe: even the const query methods lazily build and mutate the
 *   internal adjacency caches, so a single instance must not be queried from
 *   multiple threads concurrently. Use one instance per thread.
 * ******************************************************************************************************************/

#ifndef _USD_BREP_TOPOLOGY_QUERIES_H_
#define _USD_BREP_TOPOLOGY_QUERIES_H_

#include "UsdBrepIterator.h"
#include "UsdBrepUtilsConfig.h"

#include <unordered_map>
#include <vector>

namespace UsdBrep
{

class USD_BREP_EXPORT UsdBrepTopologyQueries final
{
public:

    explicit UsdBrepTopologyQueries(const UsdBrepView& brep);

    ~UsdBrepTopologyQueries() noexcept = default;

    UsdBrepTopologyQueries(const UsdBrepTopologyQueries&) = delete;
    UsdBrepTopologyQueries& operator=(const UsdBrepTopologyQueries&) = delete;

    // ---- Edge-Face adjacency ----

    // All local face indices whose faceuses reference edge iLocalEdge.
    std::vector<uint32_t> GetFacesAdjacentToEdge(uint32_t iLocalEdge) const;

    // All local edge indices referenced by edgeuses belonging to loops of face iLocalFace.
    std::vector<uint32_t> GetEdgesBoundingFace(uint32_t iLocalFace) const;

    // True if the two faces share at least one common edge.
    bool AreFacesAdjacent(uint32_t iLocalFace1, uint32_t iLocalFace2) const;

    // ---- Vertex connectivity ----

    // All local edge indices that reference iLocalVertex as start or end vertex.
    std::vector<uint32_t> GetEdgesAtVertex(uint32_t iLocalVertex) const;

    // All local face indices that reference iLocalVertex
    // (through edges that bound those faces).
    std::vector<uint32_t> GetFacesAtVertex(uint32_t iLocalVertex) const;

    // ---- Radial edge traversal ----

    // Collect the circular list of local edgeuse indices around an edge,
    // starting from iLocalEdgeuse and chasing NextRadialEUIndex.
    std::vector<uint32_t> GetRadialEdgeuses(uint32_t iLocalEdgeuse) const;

    // ---- Edgeuse-to-face mapping ----

    // Return the local face index that owns a given local edgeuse index.
    uint32_t GetFaceForEdgeuse(uint32_t iLocalEdgeuse) const;

private:

    void BuildEdgeToEdgeuseMap() const;
    void BuildEdgeuseToFaceMap() const;
    void BuildVertexToEdgeMap() const;

    const UsdBrepView& m_rBrep;

    // Lazily populated caches (all indices are local)
    mutable bool m_bEdgeToEdgeuseBuilt = false;
    mutable std::unordered_map<uint32_t, std::vector<uint32_t>> m_mEdgeToEdgeuses;

    mutable bool m_bEdgeuseToFaceBuilt = false;
    mutable std::unordered_map<uint32_t, uint32_t> m_mEdgeuseToFace;

    mutable bool m_bVertexToEdgeBuilt = false;
    mutable std::unordered_map<uint32_t, std::vector<uint32_t>> m_mVertexToEdges;
};

} // end namespace UsdBrep

#endif // _USD_BREP_TOPOLOGY_QUERIES_H_
