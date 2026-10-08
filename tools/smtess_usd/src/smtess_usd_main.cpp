// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "SmBrep.h"
#include "SmEdge.h"
#include "SmCurve.h"
#include "SmFace.h"
#include "SmPoly.h"
#include "SmContext.h"
#include "UsdBrepArrayData.h"
#include "UsdBrepUtilities.h"
#include "SmuConvert.h"

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>

#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#if __has_include(<tbb/parallel_pipeline.h>)
#include <tbb/parallel_pipeline.h>
#define SMLIB_TBB_FILTER_SERIAL_IN_ORDER tbb::filter_mode::serial_in_order
#define SMLIB_TBB_FILTER_PARALLEL tbb::filter_mode::parallel
#define SMLIB_TBB_FILTER_SERIAL_OUT_OF_ORDER tbb::filter_mode::serial_out_of_order
#elif __has_include(<tbb/pipeline.h>)
#include <tbb/pipeline.h>
#define SMLIB_TBB_FILTER_SERIAL_IN_ORDER tbb::filter::serial_in_order
#define SMLIB_TBB_FILTER_PARALLEL tbb::filter::parallel
#define SMLIB_TBB_FILTER_SERIAL_OUT_OF_ORDER tbb::filter::serial_out_of_order
#else
#error "TBB pipeline header not found (expected tbb/parallel_pipeline.h or tbb/pipeline.h)"
#endif

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

struct MeshOutput
{
    std::string name;
    std::vector<float> points;         // x,y,z triples
    std::vector<int32_t> faces;        // pyvista format: [n, i0, i1, ..., n, ...]
    std::vector<float> normals;        // x,y,z triples (per face-vertex; split at hard edges)
    std::vector<float> face_centers;   // x,y,z triples (per-face centroid)
    std::vector<float> face_normals;   // x,y,z triples (per-face normal)
    std::vector<uint32_t> edge_vertex_indices; // indices of vertices on BRep edges/vertices
    std::vector<int32_t> lines;        // pyvista format: [n, i0, i1, ..., n, ...] (--wireframe only)
    bool failedFaces = false;          // true if SmTess reported failed faces for this Brep
};

static bool ExtractMesh(SmPolyBrep& polyBrep, const std::string& name, MeshOutput& out)
{
    SmTArray<SmPolyVertex*> polyVerts;
    SmTArray<SmPolyFace*> polyFaces;
    polyBrep.GetPolyVertices(polyVerts);
    polyBrep.GetPolyFaces(polyFaces);

    ULONG nVerts = polyVerts.GetSize();
    ULONG nFaces = polyFaces.GetSize();
    if (nVerts == 0 || nFaces == 0)
        return false;

    out.name = name;
    out.points.reserve(nVerts * 3);
    out.normals.reserve(nVerts * 3);
    out.faces.reserve(nFaces * 4);
    out.face_centers.reserve(nFaces * 3);
    out.face_normals.reserve(nFaces * 3);

    // Split vertices so each surface patch keeps its own normal at shared boundary
    // vertices: smooth within a patch, hard across patch boundaries (matches typical CAD
    // tessellator output). A welded SmPolyVertex stores one normal per incident face in parallel
    // arrays (GetFacesRef()/GetNormalsRef(), keyed by the original SmFace), so we emit a
    // distinct output vertex per (SmPolyVertex, normal-index) pair and pick the normal
    // belonging to the face currently being emitted. Averaging these (the old behavior)
    // rounded off hard edges (e.g. box corners shaded as if smooth).
    std::map<std::pair<SmPolyVertex*, int>, uint32_t> splitIdx;
    std::vector<SmPolyVertex*> splitPv;  // parallel to output vertices, for edge-vertex tagging

    auto averagedNormal = [](SmPolyVertex* pv, double& nx, double& ny, double& nz) {
        const SmTArray<SmVector3d>& norms = pv->GetNormalsRef();
        nx = ny = nz = 0;
        for (ULONG j = 0; j < norms.GetSize(); ++j) { nx += norms[j].x; ny += norms[j].y; nz += norms[j].z; }
        double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-30) { nx /= len; ny /= len; nz /= len; }
        else { nx = 0; ny = 0; nz = 1; }
    };

    // Emit (or reuse) the output vertex for SmPolyVertex pv using normal index k
    // (k < 0 or out of range => fall back to the averaged normal).
    auto emitVertex = [&](SmPolyVertex* pv, int k) -> uint32_t {
        auto key = std::make_pair(pv, k);
        auto it = splitIdx.find(key);
        if (it != splitIdx.end())
            return it->second;

        uint32_t idx = static_cast<uint32_t>(out.points.size() / 3);
        const SmPoint3d& pt = pv->GetPoint();
        out.points.push_back(static_cast<float>(pt.x));
        out.points.push_back(static_cast<float>(pt.y));
        out.points.push_back(static_cast<float>(pt.z));

        double nx, ny, nz;
        const SmTArray<SmVector3d>& norms = pv->GetNormalsRef();
        if (k >= 0 && static_cast<ULONG>(k) < norms.GetSize())
        {
            nx = norms[k].x; ny = norms[k].y; nz = norms[k].z;
            double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 1e-30) { nx /= len; ny /= len; nz /= len; }
            else averagedNormal(pv, nx, ny, nz);
        }
        else
        {
            averagedNormal(pv, nx, ny, nz);
        }
        out.normals.push_back(static_cast<float>(nx));
        out.normals.push_back(static_cast<float>(ny));
        out.normals.push_back(static_cast<float>(nz));

        splitIdx[key] = idx;
        splitPv.push_back(pv);
        return idx;
    };

    for (ULONG i = 0; i < nFaces; ++i)
    {
        SmPolyFace* pf = polyFaces[i];
        SmPolyLoop* pLoop = pf->GetOuterPolyLoop();
        if (!pLoop)
            continue;

        SmTArray<SmPolyEdge*> edges;
        pLoop->GetPolyEdges(edges);
        ULONG nEdges = edges.GetSize();
        if (nEdges == 0)
            continue;

        // The original BRep face (surface) this poly face came from; used to pick the
        // matching per-face normal entry on each shared vertex.
        SmFace* pOrigFace = pf->GetOriginalFace();

        std::vector<int32_t> faceVerts;
        faceVerts.reserve(nEdges);
        double cx = 0, cy = 0, cz = 0;
        for (ULONG j = 0; j < nEdges; ++j)
        {
            SmPolyVertex* pv = edges[j]->GetStartPolyVertex();
            int k = -1;
            if (pOrigFace)
            {
                SmTArray<SmTopology*>& vfaces = pv->GetFacesRef();
                for (ULONG f = 0; f < vfaces.GetSize(); ++f)
                {
                    if (vfaces[f] == static_cast<SmTopology*>(pOrigFace)) { k = static_cast<int>(f); break; }
                }
            }
            faceVerts.push_back(static_cast<int32_t>(emitVertex(pv, k)));

            const SmPoint3d& pt = pv->GetPoint();
            cx += pt.x; cy += pt.y; cz += pt.z;
        }
        cx /= nEdges; cy /= nEdges; cz /= nEdges;
        out.faces.push_back(static_cast<int32_t>(nEdges));
        out.faces.insert(out.faces.end(), faceVerts.begin(), faceVerts.end());
        out.face_centers.push_back(static_cast<float>(cx));
        out.face_centers.push_back(static_cast<float>(cy));
        out.face_centers.push_back(static_cast<float>(cz));

        SmVector3d fn = pf->GetNormal(TRUE, FALSE);
        double flen = std::sqrt(fn.x * fn.x + fn.y * fn.y + fn.z * fn.z);
        if (flen > 1e-30) { fn.x /= flen; fn.y /= flen; fn.z /= flen; }
        out.face_normals.push_back(static_cast<float>(fn.x));
        out.face_normals.push_back(static_cast<float>(fn.y));
        out.face_normals.push_back(static_cast<float>(fn.z));
    }

    for (size_t i = 0; i < splitPv.size(); ++i)
    {
        SmPolyVertex* pv = splitPv[i];
        if (pv->GetOriginalVertex() != NULL || pv->GetOriginalEdgeuse() != NULL)
            out.edge_vertex_indices.push_back(static_cast<uint32_t>(i));
    }

    return true;
}

// Sample each of the Brep's unique edges directly as a curve polyline --
// SmEdge::GetCurve()->EvaluatePoint(), no SmSurfaceCache subdivision at all.
// This is what --wireframe uses instead of SmBrep::ConvertToPolyBrep, so it
// stays cheap (O(edges * samplesPerEdge)) even for breps whose full surface
// tessellation is pathologically slow (see docs/vega_osfp_tessellation_failures.md).
static bool ExtractWireframe(SmBrep& brep, const std::string& name, int samplesPerEdge,
                             MeshOutput& out)
{
    out.name = name;
    SmTArray<SmEdge*> sEdges;
    brep.GetEdges(sEdges);
    if (sEdges.GetSize() == 0) return false;

    const int nSamples = std::max(2, samplesPerEdge);
    for (ULONG ei = 0; ei < sEdges.GetSize(); ++ei)
    {
        SmEdge* pEdge = sEdges[ei];
        SmCurve* pCurve = pEdge ? pEdge->GetCurve() : nullptr;
        if (!pCurve) continue;

        const SmExtent1d sInterval = pEdge->GetInterval();
        const double tMin = sInterval.GetMin();
        const double tMax = sInterval.GetMax();
        const int32_t firstIdx = static_cast<int32_t>(out.points.size() / 3);

        for (int si = 0; si < nSamples; ++si)
        {
            const double t = tMin + (tMax - tMin) * (double(si) / double(nSamples - 1));
            SmPoint3d pt(0.0, 0.0, 0.0);
            pCurve->EvaluatePoint(t, pt);
            out.points.push_back(static_cast<float>(pt.x));
            out.points.push_back(static_cast<float>(pt.y));
            out.points.push_back(static_cast<float>(pt.z));
        }

        out.lines.push_back(nSamples);
        for (int si = 0; si < nSamples; ++si)
        {
            out.lines.push_back(firstIdx + si);
        }
    }
    return !out.lines.empty();
}

// Faces are stored pyvista-style ([n, i0..in-1] runs), so counting them means
// walking the run lengths rather than dividing by a fixed stride.
static size_t CountFaces(const MeshOutput& mesh)
{
    size_t n = 0;
    for (size_t k = 0; k < mesh.faces.size();) { k += mesh.faces[k] + 1; ++n; }
    return n;
}

// A unique BRep to tessellate once: the prim whose BrepArray geometry is
// imported (a prototype prim for instanced parts, or a non-instanced prim).
struct BrepProto
{
    UsdPrim prim;
    std::string path;   // prototype/prim path, used to name the unique mesh
};

// A placement of a (possibly instanced) BRep in the scene: which unique
// prototype it draws, its world transform, and its scene path (for naming).
struct BrepPlacement
{
    size_t protoIndex = 0;
    GfMatrix4d worldXform;
    std::string name;
};

// Collect BRep placements, deduplicating instanced geometry: proxies sharing a
// prototype prim (GetPrimInPrototype) map to one BrepProto, so identical
// geometry is tessellated once. maxBreps (0 = all) caps placements.
static void CollectBrepPlacements(const UsdPrim& root, size_t maxBreps,
                                  std::vector<BrepProto>& rProtos,
                                  std::vector<BrepPlacement>& rPlacements)
{
    std::unordered_map<std::string, size_t> keyToIndex;
    UsdPrimRange range(root, UsdTraverseInstanceProxies());
    for (const UsdPrim& prim : range)
    {
        if (prim.GetPrimTypeInfo().GetTypeName() != TfToken("BrepArray"))
            continue;
        // Prototype *definitions* are drawn through their instance proxies, not
        // directly (IsInPrototype is the authoritative, naming-agnostic test).
        if (prim.IsInPrototype())
            continue;
        if (maxBreps != 0 && rPlacements.size() >= maxBreps)
            break;

        const UsdPrim protoPrim = prim.IsInstanceProxy() ? prim.GetPrimInPrototype() : prim;
        const std::string key = protoPrim.GetPath().GetString();

        size_t protoIndex;
        auto it = keyToIndex.find(key);
        if (it == keyToIndex.end())
        {
            protoIndex = rProtos.size();
            keyToIndex.emplace(key, protoIndex);
            rProtos.push_back({protoPrim, key});
        }
        else
        {
            protoIndex = it->second;
        }

        GfMatrix4d xform(1.0);
        if (prim.IsA<UsdGeomImageable>())
            xform = UsdGeomImageable(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default());

        rPlacements.push_back({protoIndex, xform, prim.GetPath().GetString()});
    }
}

// Binary v3: unique prototype-local meshes plus an instance table, so
// instanced geometry is stored once and drawn many times. Meshes are written
// as they are tessellated, so the unique count is not known up front:
// writeHeader() reserves it, patchUniqueCount() seeks back to fill it in.
// Layout after the 'SMTB' magic:
//   uint32 version = 3
//   uint32 num_unique_meshes            (patched at the end)
//   repeated: uint32 proto_index; <mesh body>                 (LOCAL space)
//   uint32 num_instances
//   repeated: uint32 proto_index; uint32 name_len; char name[];
//             float xform[16]  (row-major, USD row-vector convention: world = pt * M)
// <mesh body>: name(local suffix), failed, points, normals, faces,
// face_centers, face_normals, edge_vertex_indices, lines (v3+; empty for v2 data).
struct BinWriter
{
    FILE* fp = nullptr;
    bool ok = true;
    long countPos = -1;  // file offset of the num_unique_meshes field

    void bytes(const void* p, size_t elemSize, size_t count)
    {
        if (!ok || count == 0) return;
        if (fwrite(p, elemSize, count, fp) != count) ok = false;
    }
    void u32(uint32_t v) { bytes(&v, sizeof(uint32_t), 1); }
    void floats(const std::vector<float>& v)
    {
        u32(static_cast<uint32_t>(v.size()));
        bytes(v.data(), sizeof(float), v.size());
    }
    void writeHeader()
    {
        const char magic[4] = {'S', 'M', 'T', 'B'};
        bytes(magic, 1, 4);
        u32(3u);                 // version
        countPos = ftell(fp);    // remember where the count goes
        // -1 on a non-seekable stream: patchUniqueCount can't backfill the
        // count, so fail fast instead of emitting corrupt output.
        if (countPos < 0) { ok = false; return; }
        u32(0u);                 // placeholder num_unique_meshes
    }
    // Stream one prototype mesh (proto_index tag + body).
    void writeUniqueMesh(uint32_t protoIndex, const MeshOutput& m)
    {
        u32(protoIndex);
        u32(static_cast<uint32_t>(m.name.size()));
        bytes(m.name.data(), 1, m.name.size());
        const uint8_t failed = m.failedFaces ? 1u : 0u;
        bytes(&failed, 1, 1);
        floats(m.points);
        floats(m.normals);
        u32(static_cast<uint32_t>(m.faces.size()));
        bytes(m.faces.data(), sizeof(int32_t), m.faces.size());
        floats(m.face_centers);
        floats(m.face_normals);
        u32(static_cast<uint32_t>(m.edge_vertex_indices.size()));
        bytes(m.edge_vertex_indices.data(), sizeof(uint32_t), m.edge_vertex_indices.size());
        u32(static_cast<uint32_t>(m.lines.size()));
        bytes(m.lines.data(), sizeof(int32_t), m.lines.size());
    }
    void patchUniqueCount(uint32_t count)
    {
        if (!ok || countPos < 0) return;
        if (fflush(fp) != 0) { ok = false; return; }
        if (fseek(fp, countPos, SEEK_SET) != 0) { ok = false; return; }
        if (fwrite(&count, sizeof(uint32_t), 1, fp) != 1) { ok = false; return; }
        if (fseek(fp, 0, SEEK_END) != 0) { ok = false; return; }
    }
    void writeInstances(const std::vector<BrepPlacement>& placements)
    {
        u32(static_cast<uint32_t>(placements.size()));
        for (const BrepPlacement& pl : placements)
        {
            u32(static_cast<uint32_t>(pl.protoIndex));
            u32(static_cast<uint32_t>(pl.name.size()));
            bytes(pl.name.data(), 1, pl.name.size());
            float x[16];
            const double* d = pl.worldXform.GetArray();  // 16 doubles, row-major
            for (int k = 0; k < 16; ++k) x[k] = static_cast<float>(d[k]);
            bytes(x, sizeof(float), 16);
        }
    }
};

static void PrintUsage(const char* prog)
{
    fprintf(stderr,
        "Usage: %s <input.usd> [options]\n"
        "\n"
        "Tessellate USD BRep geometry with SmTess and write a binary (.smtb) mesh file.\n"
        "\n"
        "Options:\n"
        "  --chord-tolerance <val>   Max chord height deviation (default: 0, ignore)\n"
        "  --curve-angle <val>       Max curve angular deflection in degrees (default: 25)\n"
        "  --surface-angle <val>     Max surface angular deflection in degrees (default: 25)\n"
        "  --max-edge-length <val>   Max 3D edge length (default: 0, ignore)\n"
        "  --max-aspect-ratio <val>  Max triangle aspect ratio (default: 0, ignore)\n"
        "  --smooth                  Enable smoothing\n"
        "  --advancing-front         Use advancing front tessellation\n"
        "  --heal                    Enable BRep healer (default: off)\n"
        "  --max-breps <n>           Stop after n BrepArray placements (0 = all)\n"
        "  --workers <n>             Max prototypes in flight (default: auto; 1 = serial)\n"
        "  --wireframe               Sample edge curves instead of ConvertToPolyBrep -- fast\n"
        "                            (no SmSurfaceCache subdivision) even for breps whose\n"
        "                            full tessellation is pathologically slow\n"
        "  --wireframe-samples <n>   Points sampled per edge in --wireframe (default 12)\n"
        "  --timing-csv <file>       Write per-brep ConvertToPolyBrep wall time (seconds),\n"
        "                            sorted slowest first, as prim_path,brep_index,seconds\n"
        "  --timing <file>           Write import/heal and tessellation timing as JSONL\n"
        "                            (same schema as brep_geometry_validator --timing);\n"
        "                            view with make_timing_chart.py\n"
        "  --dump-slow-dir <dir>     Write a native .smb to this pre-existing directory for every\n"
        "                            brep whose ConvertToPolyBrep wall time is >= --dump-slow-threshold\n"
        "  --dump-slow-threshold <s> Minimum wall time in seconds for --dump-slow-dir (default: 0,\n"
        "                            i.e. dump every brep)\n"
        "  -o, --output <file>       Output file (required)\n"
        "  -h, --help                Print this help\n",
        prog);
}

static bool ParseDouble(const char* s, double& out)
{
    char* end = nullptr;
    out = std::strtod(s, &end);
    return end && end != s && *end == '\0' && std::isfinite(out);
}

int main(int argc, char** argv)
{
    std::string inputPath;
    std::string outputPath;
    double chordTol = 0.0;
    double curveAngle = 25.0;
    double surfaceAngle = 25.0;
    double maxEdgeLen = 0.0;
    double maxAspect = 0.0;
    bool smooth = false;
    bool advancingFront = false;
    bool heal = false;
    int maxBreps = 0;   // 0 = no limit; otherwise stop after this many placements
    int nWorkers = 0;   // --workers N: fixed pipeline token count (0 = auto by hardware; 1 = serial)
    bool wireframeOnly = false;       // --wireframe: sample edge curves instead of ConvertToPolyBrep
    int wireframeSamples = 12;        // --wireframe-samples <n>: points per edge (>= 2)
    std::string timingCsvPath;  // --timing-csv <path>: per-brep ConvertToPolyBrep wall time, sorted slowest-first
    std::string threadTimingPath;  // --timing <path>: per-brep worker-thread start/end timestamps (JSONL)
    double dumpSlowThreshold = 0.0;   // --dump-slow-threshold <secs>: write out breps at/above this time
    std::string dumpSlowDir;          // --dump-slow-dir <dir>: pre-existing directory for the above (native .smb)

    for (int i = 1; i < argc; ++i)
    {
        const char* arg = argv[i];
        if (std::strcmp(arg, "-h") == 0 || std::strcmp(arg, "--help") == 0)
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else if (std::strcmp(arg, "--smooth") == 0)
        {
            smooth = true;
        }
        else if (std::strcmp(arg, "--advancing-front") == 0)
        {
            advancingFront = true;
        }
        else if (std::strcmp(arg, "--heal") == 0)
        {
            heal = true;
        }
        else if (std::strcmp(arg, "--binary") == 0)
        {
            // Deprecated no-op: binary is now the only output format. Still
            // accepted so existing callers (e.g. view_usd) keep working.
        }
        else if (std::strcmp(arg, "--workers") == 0 && i + 1 < argc)
        {
            nWorkers = std::atoi(argv[++i]);
            if (nWorkers < 1) { nWorkers = 1; }
        }
        else if (std::strcmp(arg, "--wireframe") == 0)
        {
            wireframeOnly = true;
        }
        else if (std::strcmp(arg, "--wireframe-samples") == 0 && i + 1 < argc)
        {
            const char* sVal = argv[++i];
            char* end = nullptr;
            long parsed = std::strtol(sVal, &end, 10);
            if (!end || end == sVal || *end != '\0' || parsed < 2 || parsed > INT32_MAX)
            {
                fprintf(stderr, "Bad value for --wireframe-samples\n");
                return 1;
            }
            wireframeSamples = static_cast<int>(parsed);
        }
        else if (std::strcmp(arg, "--timing-csv") == 0 && i + 1 < argc)
        {
            timingCsvPath = argv[++i];
        }
        else if (std::strcmp(arg, "--timing") == 0 && i + 1 < argc)
        {
            threadTimingPath = argv[++i];
        }
        else if (std::strcmp(arg, "--dump-slow-threshold") == 0 && i + 1 < argc)
        {
            if (!ParseDouble(argv[++i], dumpSlowThreshold) || dumpSlowThreshold < 0.0)
            {
                fprintf(stderr, "Bad value for --dump-slow-threshold\n");
                return 1;
            }
        }
        else if (std::strcmp(arg, "--dump-slow-dir") == 0 && i + 1 < argc)
        {
            dumpSlowDir = argv[++i];
        }
        else if (std::strcmp(arg, "--max-breps") == 0 && i + 1 < argc)
        {
            maxBreps = std::atoi(argv[++i]);
            if (maxBreps < 0)
            {
                maxBreps = 0;
            }
        }
        else if ((std::strcmp(arg, "-o") == 0 || std::strcmp(arg, "--output") == 0) && i + 1 < argc)
        {
            outputPath = argv[++i];
        }
        else if (std::strcmp(arg, "--chord-tolerance") == 0 && i + 1 < argc)
        {
            if (!ParseDouble(argv[++i], chordTol)) { fprintf(stderr, "Bad value for --chord-tolerance\n"); return 1; }
        }
        else if (std::strcmp(arg, "--curve-angle") == 0 && i + 1 < argc)
        {
            if (!ParseDouble(argv[++i], curveAngle)) { fprintf(stderr, "Bad value for --curve-angle\n"); return 1; }
        }
        else if (std::strcmp(arg, "--surface-angle") == 0 && i + 1 < argc)
        {
            if (!ParseDouble(argv[++i], surfaceAngle)) { fprintf(stderr, "Bad value for --surface-angle\n"); return 1; }
        }
        else if (std::strcmp(arg, "--max-edge-length") == 0 && i + 1 < argc)
        {
            if (!ParseDouble(argv[++i], maxEdgeLen)) { fprintf(stderr, "Bad value for --max-edge-length\n"); return 1; }
        }
        else if (std::strcmp(arg, "--max-aspect-ratio") == 0 && i + 1 < argc)
        {
            if (!ParseDouble(argv[++i], maxAspect)) { fprintf(stderr, "Bad value for --max-aspect-ratio\n"); return 1; }
        }
        else if (arg[0] == '-')
        {
            fprintf(stderr, "Unknown option: %s\n", arg);
            PrintUsage(argv[0]);
            return 1;
        }
        else
        {
            inputPath = arg;
        }
    }

    if (inputPath.empty())
    {
        PrintUsage(argv[0]);
        return 1;
    }

    // The output is a binary blob and needs a seekable file: the header's mesh
    // count is backfilled after streaming, and stdout is text-mode on Windows.
    if (outputPath.empty())
    {
        fprintf(stderr, "Error: an output file is required (-o/--output).\n");
        return 1;
    }

    const bool requestedTiming = !timingCsvPath.empty();
    const bool requestedThreadTiming = !threadTimingPath.empty();
    const bool requestedDumpSlow = !dumpSlowDir.empty();
    const bool wantTiming = requestedTiming && !wireframeOnly;
    const bool wantThreadTiming = requestedThreadTiming && !wireframeOnly;
    const bool wantDumpSlow = requestedDumpSlow && !wireframeOnly;

    if (wireframeOnly && (requestedTiming || requestedThreadTiming || requestedDumpSlow))
    {
        fprintf(stderr, "Warning: --timing-csv/--timing/--dump-slow-dir have no effect with "
                         "--wireframe (no ConvertToPolyBrep call to time)\n");
    }

    UsdBrepData::RegisterOmniSolidResourcesPlugin();

    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    auto stage = UsdStage::Open(inputPath);
    if (!stage)
    {
        fprintf(stderr, "Error: failed to open USD stage %s\n", inputPath.c_str());
        return 1;
    }

    std::vector<BrepProto> protos;
    std::vector<BrepPlacement> placements;
    CollectBrepPlacements(stage->GetPseudoRoot(),
                          maxBreps > 0 ? static_cast<size_t>(maxBreps) : 0,
                          protos, placements);

    if (placements.empty())
    {
        fprintf(stderr, "Error: no BrepArray prims found in %s\n", inputPath.c_str());
        return 1;
    }

    fprintf(stderr,
            "Found %zu BrepArray placement(s), %zu unique prototype(s)%s in %s\n",
            placements.size(), protos.size(),
            maxBreps > 0 ? " [capped by --max-breps]" : "", inputPath.c_str());

    // Open the output up front so each mesh streams to the file as it is
    // tessellated rather than accumulating the deduped scene in memory.
    FILE* fp = fopen(outputPath.c_str(), "wb");
    if (!fp)
    {
        fprintf(stderr, "Error: cannot open %s\n", outputPath.c_str());
        return 1;
    }
    // Streamed writes run in the serial output filter, so default 4 KB buffering
    // would put a write() per 4 KB on the one stage that cannot parallelize.
    setvbuf(fp, nullptr, _IOFBF, 4 << 20);

    BinWriter bw;
    bw.fp = fp;
    bw.writeHeader();  // magic + version + placeholder unique-mesh count
    if (!bw.ok)        // fail fast rather than tessellating into a dead file
    {
        fprintf(stderr, "Error: failed to write binary output header\n");
        fclose(fp);
        return 1;
    }

    // Tessellate each unique prototype once, prototype-local, through a software
    // pipeline: serial input (USD reads stay serial), parallel ConvertToPolyBrep,
    // serial output. The live-token count bounds peak memory. Each mesh streams
    // straight to the file, tagged with its proto_index so out-of-order
    // completion is fine.
    const size_t protoCount = protos.size();

    // Streamed-mesh accounting (binary path).
    size_t numUnique = 0, uniquePts = 0, uniqueFaces = 0;

    struct WorkItem
    {
        size_t index = 0;
        SmContext* context = nullptr;
        std::vector<SmBrep*> breps;
        std::vector<SmPolyBrep*> polyBreps;
        std::vector<char> failed;
        std::vector<double> tessSecs;  // per-brep ConvertToPolyBrep wall time (--timing-csv)
        std::vector<double> tessStartSecs;  // per-brep start offset from pipelineStart (--timing)
        double importHealStartSecs = 0.0;
        double importHealEndSecs = 0.0;
        int timingThreadIdx = -1;
        std::vector<MeshOutput> wireframeMeshes;  // per-brep edge-curve polylines (--wireframe)
        std::string warn;
    };

    // Populated single-threaded in the serial-out-of-order filter below.
    struct TimingEntry { double secs; std::string primPath; size_t brepIndex; };
    std::vector<TimingEntry> timingEntries;
    struct ThreadTimingEntry
    {
        int thread;
        double healStartSecs, healEndSecs, tessStartSecs, tessEndSecs;
        std::string primPath;
        size_t brepIndex;
    };
    std::vector<ThreadTimingEntry> threadTimingEntries;
    const Clock::time_point pipelineStart = Clock::now();

    size_t nextIdx = 0;
    size_t doneCount = 0;
    const unsigned hw = std::thread::hardware_concurrency();
    // Max prototypes in flight through the parallel filter. Default auto-sizes to
    // the hardware; --workers N overrides it (N=1 => strictly serial cross-brep
    // processing, useful for reproducible debugging). A single monster BRep can
    // tessellate to multiple GB, so this also bounds peak memory.
    const size_t nTokens =
        (nWorkers > 0) ? (size_t)nWorkers : std::max<size_t>(64, hw ? size_t(2) * hw : 128);

    tbb::parallel_pipeline(
        nTokens,
        tbb::make_filter<void, WorkItem*>(
            SMLIB_TBB_FILTER_SERIAL_IN_ORDER,
            [&](tbb::flow_control& fc) -> WorkItem*
            {
                // Stop feeding once done, or as soon as a streamed binary write
                // fails -- no point tessellating into an unrecoverable file.
                if (nextIdx >= protoCount || !bw.ok)
                {
                    fc.stop();
                    return nullptr;
                }
                WorkItem* item = new WorkItem();
                item->index = nextIdx++;
                return item;  // USD import + heal happen in the parallel filter
            })
        & tbb::make_filter<WorkItem*, WorkItem*>(
            SMLIB_TBB_FILTER_PARALLEL,
            [&](WorkItem* item) -> WorkItem*
            {
                // Import and heal here in the parallel filter so both scale across cores.
                // Heal via BrepMove_UsdToSMLib's import-integrated healer -- the path the
                // parallel brep_geometry_validator uses safely -- not SmBrep::HealBrep,
                // whose extra steps corrupt shared state under concurrency.
                item->context = new SmContext();
                const Clock::time_point importHealT0 = Clock::now();
                if (wantThreadTiming)
                    item->timingThreadIdx = tbb::this_task_arena::current_thread_index();
                try
                {
                    const SmStatus importStatus = SMU_BrepConvert::BrepMove_UsdToSMLib(
                        *item->context, (UsdGeomGprim)protos[item->index].prim,
                        item->breps, heal ? TRUE : FALSE);
                    if (importStatus != SM_SUCCESS)
                    {
                        for (SmBrep* b : item->breps) delete b;
                        item->breps.clear();
                        item->warn += "Warning: import failed for " +
                                      protos[item->index].path + "\n";
                    }
                }
                catch (...)
                {
                    for (SmBrep* b : item->breps) delete b;
                    item->breps.clear();
                    // Report as a warning like tessellation failures below; dropping it silently
                    // makes parts vanish from the viewer with no diagnostic.
                    item->warn += "Warning: import failed for " +
                                  protos[item->index].path + "\n";
                }
                if (wantThreadTiming)
                {
                    item->importHealStartSecs =
                        std::chrono::duration<double>(importHealT0 - pipelineStart).count();
                    item->importHealEndSecs =
                        std::chrono::duration<double>(Clock::now() - pipelineStart).count();
                }

                std::vector<SmBrep*>& sBreps = item->breps;
                item->polyBreps.assign(sBreps.size(), nullptr);
                item->failed.assign(sBreps.size(), 0);
                const std::string& primPath = protos[item->index].path;

                if (wireframeOnly)
                {
                    item->wireframeMeshes.resize(sBreps.size());
                    for (size_t bi = 0; bi < sBreps.size(); ++bi)
                    {
                        SmBrep* pBrep = sBreps[bi];
                        if (!pBrep) continue;
                        const std::string localName =
                            (sBreps.size() > 1) ? ("/Brep_" + std::to_string(bi)) : std::string();
                        if (!ExtractWireframe(*pBrep, localName, wireframeSamples,
                                              item->wireframeMeshes[bi]))
                        {
                            item->warn += "Warning: no edges for " + primPath +
                                          " Brep " + std::to_string(bi) + "\n";
                        }
                    }
                    return item;
                }

                item->tessSecs.assign(sBreps.size(), 0.0);
                item->tessStartSecs.assign(sBreps.size(), -1.0);
                for (size_t bi = 0; bi < sBreps.size(); ++bi)
                {
                    SmBrep* pBrep = sBreps[bi];
                    if (!pBrep) continue;

                    SmPolyBrep* pPolyBrep = nullptr;
                    SmBoolean failedFaces = FALSE;
                    ULONG numLamina = 0;
                    const Clock::time_point tessT0 = Clock::now();
                    SmStatus status = pBrep->ConvertToPolyBrep(
                        pPolyBrep, failedFaces, numLamina,
                        chordTol, curveAngle, surfaceAngle,
                        maxEdgeLen, maxAspect,
                        advancingFront ? TRUE : FALSE,
                        smooth ? TRUE : FALSE, FALSE);
                    item->tessSecs[bi] =
                        std::chrono::duration<double>(Clock::now() - tessT0).count();
                    if (wantThreadTiming)
                    {
                        item->tessStartSecs[bi] =
                            std::chrono::duration<double>(tessT0 - pipelineStart).count();
                    }

                    if (wantDumpSlow && item->tessSecs[bi] >= dumpSlowThreshold)
                    {
                        // Native binary so the exact input geometry can be reloaded directly.
                        std::string sName = primPath;
                        for (char& c : sName) { if (c == '/') { c = '_'; } }
                        char sSecsBuf[32];
                        std::snprintf(sSecsBuf, sizeof(sSecsBuf), "%.1fs", item->tessSecs[bi]);
                        const std::string sOutPath = dumpSlowDir + "/" + sName + "_brep" +
                                                      std::to_string(bi) + "_" + sSecsBuf + ".smb";
#ifdef UNICODE
                        std::wstring wOutPath(sOutPath.begin(), sOutPath.end());
                        SmStatus dumpStatus = pBrep->WriteToFile(wOutPath.c_str(), SM_BINARY);
#else
                        SmStatus dumpStatus = pBrep->WriteToFile(sOutPath.c_str(), SM_BINARY);
#endif
                        if (dumpStatus != SM_SUCCESS)
                        {
                            fprintf(stderr, "Warning: --dump-slow-dir: failed to write %s\n",
                                    sOutPath.c_str());
                        }
                    }

                    if (status != SM_SUCCESS || !pPolyBrep)
                    {
                        item->warn += "Warning: tessellation failed for " + primPath +
                                      " Brep " + std::to_string(bi) + "\n";
                        continue;
                    }
                    if (failedFaces)
                    {
                        item->warn += "Warning: some faces failed for " + primPath +
                                      " Brep " + std::to_string(bi) + "\n";
                        item->failed[bi] = 1;
                    }
                    item->polyBreps[bi] = pPolyBrep;
                }
                return item;
            })
        & tbb::make_filter<WorkItem*, void>(
            SMLIB_TBB_FILTER_SERIAL_OUT_OF_ORDER,
            [&](WorkItem* item)
            {
                if (!item->warn.empty())
                    fputs(item->warn.c_str(), stderr);

                if (wantTiming)
                {
                    const std::string& primPath = protos[item->index].path;
                    for (size_t bi = 0; bi < item->tessSecs.size(); ++bi)
                    {
                        timingEntries.push_back({item->tessSecs[bi], primPath, bi});
                    }
                }

                if (wantThreadTiming)
                {
                    const std::string& primPath = protos[item->index].path;
                    bool includeImportHeal = true;
                    for (size_t bi = 0; bi < item->tessSecs.size(); ++bi)
                    {
                        if (item->tessStartSecs[bi] < 0.0) continue;
                        threadTimingEntries.push_back({
                            item->timingThreadIdx,
                            includeImportHeal ? item->importHealStartSecs : -1.0,
                            includeImportHeal ? item->importHealEndSecs : -1.0,
                            item->tessStartSecs[bi],
                            item->tessStartSecs[bi] + item->tessSecs[bi],
                            primPath, bi});
                        includeImportHeal = false;
                    }
                }

                std::vector<SmBrep*>& sBreps = item->breps;
                for (size_t bi = 0; bi < sBreps.size(); ++bi)
                {
                    if (wireframeOnly && !item->wireframeMeshes[bi].lines.empty())
                    {
                        MeshOutput& mesh = item->wireframeMeshes[bi];
                        uniquePts += mesh.points.size() / 3;
                        bw.writeUniqueMesh(static_cast<uint32_t>(item->index), mesh);
                        ++numUnique;
                    }
                    else if (item->polyBreps[bi])
                    {
                        // Local-space mesh; name holds only the per-brep suffix,
                        // the placement path is prepended when instanced.
                        const std::string localName =
                            (sBreps.size() > 1) ? ("/Brep_" + std::to_string(bi)) : std::string();
                        MeshOutput mesh;
                        if (ExtractMesh(*item->polyBreps[bi], localName, mesh))
                        {
                            mesh.failedFaces = (item->failed[bi] != 0);
                            // Stream straight to the file; do not retain.
                            uniquePts += mesh.points.size() / 3;
                            uniqueFaces += CountFaces(mesh);
                            bw.writeUniqueMesh(static_cast<uint32_t>(item->index), mesh);
                            ++numUnique;
                        }
                        delete item->polyBreps[bi];
                    }
                    if (sBreps[bi]) delete sBreps[bi];
                }
                delete item->context;

                ++doneCount;
                if (doneCount % 512 == 0 || doneCount == protoCount)
                {
                    fprintf(stderr, "Tessellated %zu/%zu unique prototype(s)...\n",
                            doneCount, protoCount);
                    fflush(stderr);
                }
                delete item;
            }));

    double elapsedSecs = std::chrono::duration<double>(Clock::now() - t0).count();

    bool writeOk = true;
    size_t nUnique = numUnique, nPts = uniquePts, nFaces = uniqueFaces;

    // Unique meshes were streamed during tessellation; backfill the count
    // and append the instance table (store once, draw many).
    bw.patchUniqueCount(static_cast<uint32_t>(numUnique));
    bw.writeInstances(placements);
    writeOk = bw.ok;

    if (nUnique == 0)
    {
        fprintf(stderr, "Error: no meshes produced\n");
        fclose(fp);
        return 1;
    }
    fprintf(stderr,
            "Tessellation: %zu unique mesh(es), %zu instance(s), %zu unique vertices, "
            "%zu unique faces (%.2fs)\n",
            nUnique, placements.size(), nPts, nFaces, elapsedSecs);

    if (wantTiming)
    {
        std::sort(timingEntries.begin(), timingEntries.end(),
                  [](const TimingEntry& a, const TimingEntry& b) { return a.secs > b.secs; });
        FILE* tfp = std::fopen(timingCsvPath.c_str(), "w");
        if (!tfp)
        {
            fprintf(stderr, "Warning: cannot write --timing-csv %s\n", timingCsvPath.c_str());
        }
        else
        {
            fputs("prim_path,brep_index,seconds\n", tfp);
            for (const TimingEntry& e : timingEntries)
            {
                fprintf(tfp, "%s,%zu,%.4f\n", e.primPath.c_str(), e.brepIndex, e.secs);
            }
            fclose(tfp);
            fprintf(stderr, "Wrote %zu per-brep timing(s) to %s\n",
                    timingEntries.size(), timingCsvPath.c_str());
        }
    }

    if (wantThreadTiming)
    {
        FILE* ttfp = std::fopen(threadTimingPath.c_str(), "w");
        if (!ttfp)
        {
            fprintf(stderr, "Warning: cannot write --timing %s\n", threadTimingPath.c_str());
        }
        else
        {
            for (const ThreadTimingEntry& e : threadTimingEntries)
            {
                // Escape stray '"'/'\\' in prim paths.
                std::string escapedPath;
                escapedPath.reserve(e.primPath.size());
                for (char c : e.primPath)
                {
                    if (c == '"' || c == '\\') escapedPath += '\\';
                    escapedPath += c;
                }
                const long long healStartUs =
                    e.healStartSecs < 0.0 ? -1 : (long long)(e.healStartSecs * 1e6);
                const long long healEndUs =
                    e.healEndSecs < 0.0 ? -1 : (long long)(e.healEndSecs * 1e6);
                const long long tessStartUs = (long long)(e.tessStartSecs * 1e6);
                const long long tessEndUs = (long long)(e.tessEndSecs * 1e6);
                fprintf(ttfp, "{\"t\":%d,\"hs\":%lld,\"he\":%lld,\"vs\":%lld,\"ve\":%lld,"
                              "\"prim\":\"%s\"}\n",
                        e.thread, healStartUs, healEndUs, tessStartUs, tessEndUs,
                        escapedPath.c_str());
            }
            fclose(ttfp);
            fprintf(stderr, "Wrote %zu per-brep thread timing(s) to %s\n",
                    threadTimingEntries.size(), threadTimingPath.c_str());
        }
    }

    // Surface deferred stdio write errors before declaring success: a failed
    // fwrite may only be visible via ferror, and a flush/close can still fail
    // on a full disk. Check ferror + fflush while fp is valid, then propagate a
    // failing fclose too, so a truncated output file is reported as an error
    // instead of silently returning 0.
    if (fflush(fp) != 0 || ferror(fp))
        writeOk = false;

    if (fclose(fp) != 0)
        writeOk = false;

    if (!writeOk)
    {
        fprintf(stderr, "Error: failed to write output (short write / disk full?)\n");
        return 1;
    }

    return 0;
}
