// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- TestSmApiUsd.cpp
* PURPOSE: Test cases for SM_API_USD import/export functions.
**********************************************************************/

#include "SM_API_USD_test.h"

#include <SmBrep.h>
#include <SmEdge.h>
#include <SmEdgeuse.h>
#include <SmFace.h>
#include <SmSurface.h>
#include <SmTrimmingTools.h>
#include <SmBSplineCurve.h>
#include <SmPoly.h>
#include <SmApiPrimitives.h>
#include <SmApiBrep.h>
#include <SmApiUsd.h>
#include <SmApiUsdTessellate.h>
#include <SmApiGeneral.h>
#include <UsdBrepTokens.h>

#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformable.h>

// Load kernel declarations before Windows macros introduced by USD.
#include <UsdBrepHeaders.h>
#include <SmuTessellate.h>
#include <SmuAttribute.h>

#include <cmath>
#include <future>
#include <memory>
#include <vector>
#include <cstdio>
#include <string>

static const std::string kOutputDir = "../SM_API_USD_test/OutputFiles/";
static const std::string kTestFilesDir = "../../TestFiles/usd_TestFiles/";

static SmStatus TestExportRetainsGeneratedTrims()
{
    for (bool nurbs : {false, true})
    for (bool bound : {false, true})
    for (bool exportUV : {false, true})
    {
        SmBrep* raw = nullptr;
        SER(SmApiCreateBox(SmVector3d(0.0, 0.0, 0.0), 4.0, 5.0, 6.0, raw));
        std::unique_ptr<SmBrep> brep(raw);
        if (nurbs)
            SER(SmApiTurnToNurbs(raw));
        SmTArray<SmFace*> faces;
        raw->GetFaces(faces);
        // Simulate a supplied unbounded face range, including on NURBS where
        // the exporter must bound the range to the finite supporting surface.
        const SmExtent2d unbounded(SmPoint2d(-SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER),
                                   SmPoint2d(SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER));
        faces[0]->SetUVDomain(unbounded);
        SmTrimmingTools::ClearUVTrimCurves(raw);
        SmTArray<SmEdgeuse*> edgeuses;
        raw->GetEdgeuses(edgeuses);
        for (ULONG ii = 0; ii < edgeuses.GetSize(); ++ii)
            if (edgeuses[ii]->GetUVTrimCurve() != nullptr)
                return SM_ERR;

        SmSurface* surface = faces[0]->GetSurface();
        const std::string suffix = std::to_string(nurbs) + std::to_string(bound) + std::to_string(exportUV);
        const std::string path = kOutputDir + "retained_trims_" + suffix + ".usda";
        SER(SmApiUsdExportBrep(path.c_str(), raw, exportUV, bound));

        // No geometry replacement or source face-domain shrinking.
        if (faces[0]->GetSurface() != surface || faces[0]->GetUVDomain().IsBounded() ||
            raw->FindAttribute(SM_AI_BREP_ARRAY) == nullptr)
            return SM_ERR;
        std::vector<SmBSplineCurve*> trims;
        ULONG trimCount = 0;
        for (ULONG ii = 0; ii < edgeuses.GetSize(); ++ii)
        {
            SmBSplineCurve* trim = edgeuses[ii]->GetUVTrimCurve();
            trims.push_back(trim);
            if (trim != nullptr)
                ++trimCount;
        }
        if ((trimCount > 0) != bound)
            return SM_ERR;

        pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(path);
        if (!stage)
            return SM_ERR;
        pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath("/World/Brep0"));
        pxr::VtVec2dArray ranges;
        if (!prim.GetAttribute(pxr::TfToken("face:range")).Get(&ranges) || ranges.empty())
            return SM_ERR;
        // Analytic exports use the surface's STEP domain (already bounded for
        // this box), whereas NURBS exports use the prepared native face domain.
        if (nurbs && (std::abs(ranges[0][0]) != SM_INFINITE_PARAMETER) != bound)
            return SM_ERR;
        pxr::VtUIntArray counts;
        prim.GetAttribute(pxr::TfToken("brep:curveUv:nurb:vertexCount")).Get(&counts);
        bool hasExportedTrim = false;
        for (unsigned int count : counts)
            hasExportedTrim = hasExportedTrim || count > 0;
        if (hasExportedTrim != (nurbs && bound && exportUV))
            return SM_ERR; // must appear on the first export, not only the second

        const std::string secondPath = kOutputDir + "retained_trims_again_" + suffix + ".usda";
        SER(SmApiUsdExportBrep(secondPath.c_str(), raw, exportUV, bound));
        for (ULONG ii = 0; ii < edgeuses.GetSize(); ++ii)
            if (edgeuses[ii]->GetUVTrimCurve() != trims[ii])
                return SM_ERR; // reuse rather than regenerate the cached curves
    }
    return SM_SUCCESS;
}

static SmStatus TestBoundaryPointCurve()
{
    SmPoint3d origin(0.0, 0.0, 0.0);
    SmBrep* box = nullptr;
    SER(SmApiCreateBox(origin, 10.0, 10.0, 10.0, box));
    std::unique_ptr<SmBrep> ownedBox(box);
    SmBSplineCurve* pointCurve = nullptr;
    SER(SmBSplineCurve::CreatePointCurve(*box->GetContext(), origin, pointCurve));
    SmTArray<SmEdge*> edges;
    box->GetEdges(edges);
    // ReplaceCurve, not SetCurve: SetCurve is an internal pointer setter that leaves
    // the curve's owner unset, so ~SmEdge would not free it and the edge/curve
    // ownership invariant would be malformed for the rest of the test.
    SER(box->ReplaceCurve(edges[0], pointCurve));

    SmTArray<SmPoint3d> points;
    SmTArray<ULONG> counts;
    SER(SmApiTessellateBoundaries(box, 5.0, points, counts, edges));
    pxr::VtArray<pxr::GfVec3f> usdPoints;
    pxr::VtIntArray usdCounts;
    SER(SMU_BrepConvert::GetBoundaryData(box, 5.0, usdPoints, usdCounts));
    if (counts.GetSize() != 12 || counts[0] != 1 || points.GetSize() != 23 ||
        usdCounts.size() != 11 || usdPoints.size() != 22)
        return SM_ERR;
    for (size_t i = 0; i < usdPoints.size(); ++i)
    {
        const SmPoint3d& p = points[static_cast<ULONG>(i) + 1];
        if (usdPoints[i] != pxr::GfVec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)))
            return SM_ERR;
    }
    return SM_SUCCESS;
}

static SmStatus TestTessellatedMeshData()
{
    SmPoint3d origin(0.0, 0.0, 0.0);
    SmBrep* box = nullptr;
    SER(SmApiCreateBox(origin, 10.0, 10.0, 10.0, box));
    std::unique_ptr<SmBrep> ownedBox(box);
    SmTArray<SmFace*> faces;
    box->GetFaces(faces);
    // Two bound faces -> two subsets. The second binding carries no paths, which is what
    // produces an empty SdfPath in the output (a face with no binding gets no subset at all).
    const pxr::SdfPath material("/Materials/Test");
    faces[0]->AddAttribute(new (*box->GetContext()) SmSdfPathAttribute(
        SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, pxr::SdfPathVector{material}));
    faces[1]->AddAttribute(new (*box->GetContext()) SmSdfPathAttribute(
        SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, pxr::SdfPathVector{}));

    SmPolyBrep* rawMesh = nullptr;
    SmBoolean failedFaces = FALSE;
    ULONG laminaEdges = 0;
    const SmStatus status = box->ConvertToPolyBrep(rawMesh, failedFaces, laminaEdges,
        0.0, 25.0, 25.0, 0.0, 0.0, FALSE, FALSE, TRUE);
    SMU_BrepConvert::TessellatedMeshPtr mesh(rawMesh);
    SER(status);
    if (!mesh || failedFaces)
        return SM_ERR;

    pxr::VtArray<pxr::GfVec3f> extent, points, normals;
    pxr::VtIntArray counts, indices, normalIndices;
    pxr::TfToken interpolation, subdivision;
    pxr::VtArray<pxr::VtIntArray> subsets;
    pxr::VtArray<pxr::SdfPath> materials;
    SER(SMU_BrepConvert::GetTessellatedMeshData(*mesh, extent, points, counts, indices,
        normals, normalIndices, interpolation, subdivision, subsets, materials));
    // Entered disabled, so must stay disabled.
    if (mesh->GetOKBackPtrs() || points.empty() || normals.empty() ||
        normalIndices.size() != indices.size() || materials.size() != 2 || subsets.size() != 2 ||
        subsets[0].empty() || subsets[1].empty())
        return SM_ERR;

    // Subset order follows an unordered_map, so match by value: one path bound, one empty.
    size_t boundCount = 0, emptyCount = 0;
    for (const pxr::SdfPath& path : materials)
    {
        if (path == material)
            ++boundCount;
        else if (path.IsEmpty())
            ++emptyCount;
    }
    if (boundCount != 1 || emptyCount != 1)
        return SM_ERR;

    // Entered enabled (as SmTess caches are), so must stay enabled.
    mesh->SetOKBackPtrs(TRUE);
    SER(SMU_BrepConvert::GetTessellatedMeshData(*mesh, extent, points, counts, indices,
        normals, normalIndices, interpolation, subdivision, subsets, materials));
    if (!mesh->GetOKBackPtrs())
        return SM_ERR;

    // Delete in both orders with the flag left on. The deleter disables it first, so
    // ~SmPolyBrep never reads the internal tessellation copy's vertices, which
    // ConvertToPolyBrep already destroyed. Copied paths still survive.
    ownedBox.reset();
    mesh.reset();
    for (const pxr::SdfPath& path : materials)
    {
        if (path == material)
            return SM_SUCCESS;
    }
    return SM_ERR;
}

// Regression: geomSubset indices name emitted USD faces. GetMeshData skips PolyFaces with
// no outer loop, so indexing the unfiltered PolyFace array runs past the USD face array
// once any face is dropped. Strip the first face's loop and bind the last face, so the
// recorded index lands exactly one past the end unless the emitted index is used.
static SmStatus TestSubsetIndicesSkipLooplessFaces()
{
    SmPoint3d origin(0.0, 0.0, 0.0);
    SmBrep* box = nullptr;
    SER(SmApiCreateBox(origin, 10.0, 10.0, 10.0, box));
    std::unique_ptr<SmBrep> ownedBox(box);

    SmPolyBrep* rawMesh = nullptr;
    SmBoolean failedFaces = FALSE;
    ULONG laminaEdges = 0;
    // Propagation off so the binding can be attached after tessellation, which is what
    // lets the bound face be positioned last.
    SER(box->ConvertToPolyBrep(rawMesh, failedFaces, laminaEdges,
        0.0, 25.0, 25.0, 0.0, 0.0, FALSE, FALSE, FALSE));
    SMU_BrepConvert::TessellatedMeshPtr mesh(rawMesh);
    if (!mesh || failedFaces)
        return SM_ERR;

    SmTArray<SmPolyFace*> polyFaces;
    mesh->GetPolyFaces(polyFaces);
    const ULONG lFacesBefore = polyFaces.GetSize();
    if (lFacesBefore < 2)
        return SM_ERR;

    // Bind the last PolyFace's source face: its index is the one that overruns.
    SmFace* pLastSourceFace = polyFaces[lFacesBefore - 1]->GetOriginalFace();
    if (pLastSourceFace == nullptr)
        return SM_ERR;
    const pxr::SdfPath material("/Materials/Test");
    pLastSourceFace->AddAttribute(new (*box->GetContext()) SmSdfPathAttribute(
        SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, pxr::SdfPathVector{material}));

    // Drop the first face's outer loop: that is what GetMeshData skips.
    SmPolyLoop* pOuterLoop = polyFaces[0]->GetOuterPolyLoop();
    if (pOuterLoop == nullptr)
        return SM_ERR;
    SER(polyFaces[0]->RemovePolyLoop(pOuterLoop, NULL));
    if (polyFaces[0]->GetOuterPolyLoop() != nullptr)
        return SM_ERR;

    pxr::VtArray<pxr::GfVec3f> extent, points, normals;
    pxr::VtIntArray counts, indices, normalIndices;
    pxr::TfToken interpolation, subdivision;
    pxr::VtArray<pxr::VtIntArray> subsets;
    pxr::VtArray<pxr::SdfPath> materials;
    SER(SMU_BrepConvert::GetTessellatedMeshData(*mesh, extent, points, counts, indices,
        normals, normalIndices, interpolation, subdivision, subsets, materials));

    // One face was skipped, so the emitted array is one shorter than the PolyFace array.
    if (counts.size() != lFacesBefore - 1 || subsets.empty())
        return SM_ERR;

    for (const pxr::VtIntArray& subset : subsets)
    {
        for (int faceIndex : subset)
        {
            if (faceIndex < 0 || static_cast<size_t>(faceIndex) >= counts.size())
                return SM_ERR; // index names a face the mesh never emitted
        }
    }
    return SM_SUCCESS;
}

// Regression: the mesh-owned attribute lookup is tried first, so the source-face fallback
// only runs when the mesh carries no binding of its own. bPropagateAttribs does not achieve
// that (it is only honoured under SM_INDEXING, and copies regardless here), so bind the
// source face after tessellation. The path must resolve through the live source face and
// the copy must outlive that source.
static SmStatus TestSourceOwnedMaterialFallback()
{
    SmPoint3d origin(0.0, 0.0, 0.0);
    SmBrep* box = nullptr;
    SER(SmApiCreateBox(origin, 10.0, 10.0, 10.0, box));
    std::unique_ptr<SmBrep> ownedBox(box);

    SmPolyBrep* rawMesh = nullptr;
    SmBoolean failedFaces = FALSE;
    ULONG laminaEdges = 0;
    SER(box->ConvertToPolyBrep(rawMesh, failedFaces, laminaEdges,
        0.0, 25.0, 25.0, 0.0, 0.0, FALSE, FALSE, FALSE));
    SMU_BrepConvert::TessellatedMeshPtr mesh(rawMesh);
    if (!mesh || failedFaces)
        return SM_ERR;

    SmTArray<SmPolyFace*> polyFaces;
    mesh->GetPolyFaces(polyFaces);
    if (polyFaces.GetSize() == 0)
        return SM_ERR;

    // Bound after tessellation, so no PolyFace can hold a copy.
    SmFace* pSourceFace = polyFaces[0]->GetOriginalFace();
    if (pSourceFace == nullptr)
        return SM_ERR;
    const pxr::SdfPath material("/Materials/Test");
    pSourceFace->AddAttribute(new (*box->GetContext()) SmSdfPathAttribute(
        SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, pxr::SdfPathVector{material}));

    for (ULONG ii = 0; ii < polyFaces.GetSize(); ++ii)
    {
        if (polyFaces[ii]->FindAttribute(SM_AI_MATERIAL_BINDING) != NULL)
            return SM_ERR; // a mesh-owned copy would bypass the fallback
    }

    pxr::VtArray<pxr::GfVec3f> extent, points, normals;
    pxr::VtIntArray counts, indices, normalIndices;
    pxr::TfToken interpolation, subdivision;
    pxr::VtArray<pxr::VtIntArray> subsets;
    pxr::VtArray<pxr::SdfPath> materials;
    SER(SMU_BrepConvert::GetTessellatedMeshData(*mesh, extent, points, counts, indices,
        normals, normalIndices, interpolation, subdivision, subsets, materials));

    // Resolved only through the live source face.
    if (materials.size() != 1 || subsets.size() != 1 || materials[0] != material)
        return SM_ERR;

    // The path is deep-copied, so it outlives the source it was read from.
    ownedBox.reset();
    if (materials[0] != material)
        return SM_ERR;
    return SM_SUCCESS;
}

// Regression: an empty mesh must not publish the bounding box's initial negative-volume
// state as an extent. Success with empty arrays is the contract.
static SmStatus TestEmptyMeshExtraction()
{
    SmContext sContext;
    SmPolyBrep* pEmptyMesh = new (sContext) SmPolyBrep(SM_ZONE_TOL_3D);
    SMU_BrepConvert::TessellatedMeshPtr mesh(pEmptyMesh);
    if (!mesh)
        return SM_ERR;

    SmTArray<SmPolyVertex*> polyVertices;
    mesh->GetPolyVertices(polyVertices);
    if (polyVertices.GetSize() != 0)
        return SM_ERR;

    pxr::VtArray<pxr::GfVec3f> extent, points, normals;
    pxr::VtIntArray counts, indices, normalIndices;
    pxr::TfToken interpolation, subdivision;
    pxr::VtArray<pxr::VtIntArray> subsets;
    pxr::VtArray<pxr::SdfPath> materials;
    SER(SMU_BrepConvert::GetTessellatedMeshData(*mesh, extent, points, counts, indices,
        normals, normalIndices, interpolation, subdivision, subsets, materials));

    if (!extent.empty() || !points.empty() || !counts.empty() || !indices.empty() ||
        !subsets.empty() || !materials.empty())
        return SM_ERR;
    return SM_SUCCESS;
}

static void DeleteImportedBreps(std::vector<SmBrep*>& rBreps)
{
    for (size_t ii = 0; ii < rBreps.size(); ii++)
        delete rBreps[ii];
    rBreps.clear();
}

static SmStatus CreateMixedValidAndConversionInvalidUsd(const std::string& rFilename)
{
    SmApiCreateContext();

    std::vector<SmBrep*> spheres;
    for (int ii = 0; ii < 3; ++ii)
    {
        SmVector3d sphereOrigin(10.0 * ii, 0.0, 0.0);
        SmBrep* pSphere = NULL;
        SmStatus stat = SmApiCreateSphere(sphereOrigin, 3.0 + ii, pSphere);
        if (stat != SM_SUCCESS || pSphere == NULL)
        {
            delete pSphere;
            DeleteImportedBreps(spheres);
            return (stat != SM_SUCCESS) ? stat : SM_ERR;
        }
        spheres.push_back(pSphere);
    }

    SmStatus stat = SmApiUsdExportBreps(rFilename.c_str(), spheres, FALSE);
    DeleteImportedBreps(spheres);
    if (stat != SM_SUCCESS)
        return stat;

    pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(rFilename);
    if (!stage)
        return SM_ERR;

    pxr::UsdPrim brepPrim = stage->GetPrimAtPath(pxr::SdfPath("/World/Brep0"));
    pxr::UsdAttribute radiusAttr = brepPrim.GetAttribute(pxr::UsdBrepSurfaceTokens->brepSurfaceSphereRadius);
    pxr::VtArray<double> radii;
    if (!radiusAttr || !radiusAttr.Get(&radii) || radii.size() != 3)
        return SM_ERR;

    // Preserve readable packed-member boundaries while making the middle
    // sphere impossible to construct. Members 0 and 2 remain independently
    // importable so the partial policy must continue after member 1 fails.
    radii[1] = 0.0;
    if (!radiusAttr.Set(radii))
        return SM_ERR;

    stage->Save();
    return SM_SUCCESS;
}

static SmStatus CreateEmptyBrepArrayUsd(const std::string& rFilename)
{
    pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateNew(rFilename);
    if (!stage)
        return SM_ERR;

    stage->DefinePrim(pxr::SdfPath("/World"), pxr::TfToken("Xform"));
    pxr::UsdPrim brepPrim = stage->DefinePrim(pxr::SdfPath("/World/Brep0"), pxr::TfToken("BrepArray"));
    if (!brepPrim)
        return SM_ERR;

    stage->Save();
    return SM_SUCCESS;
}

static SmStatus CreateSingularTransformUsd(const std::string& rFilename)
{
    SmVector3d sphereOrigin(0.0, 0.0, 0.0);
    SmBrep* pSphere = NULL;
    SmStatus stat = SmApiCreateSphere(sphereOrigin, 5.0, pSphere);
    if (stat != SM_SUCCESS || pSphere == NULL)
        return SM_ERR;

    stat = SmApiUsdExportBrep(rFilename.c_str(), pSphere, FALSE);
    delete pSphere;
    if (stat != SM_SUCCESS)
        return stat;

    pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(rFilename);
    pxr::UsdPrim brepPrim = stage ? stage->GetPrimAtPath(pxr::SdfPath("/World/Brep0")) : pxr::UsdPrim();
    if (!brepPrim)
        return SM_ERR;

    // This finite rank-one matrix is valid USD data but collapses 3D geometry.
    // It deterministically exercises failure after Brep conversion, while the
    // world transform is being baked into the staged import result.
    pxr::GfMatrix4d singularTransform(0.0);
    singularTransform[0][0] = 1.0;
    singularTransform[1][0] = 1.0;
    singularTransform[3][3] = 1.0;
    pxr::UsdGeomXformOp transformOp = pxr::UsdGeomXformable(brepPrim).AddTransformOp();
    if (!transformOp || !transformOp.Set(singularTransform))
        return SM_ERR;

    stage->Save();
    return SM_SUCCESS;
}

//*************************************************************************
// TestTessellateBrepArray
//   SmApiUsdTessellateBrepArray results and failure handling.
//*************************************************************************
static SmStatus CheckTessellatedBrep(const SmApiUsdTessellatedBrep& rMesh)
{
    if (rMesh.m_sStatus != SM_SUCCESS || !rMesh.m_sMessage.empty())
        return SM_ERR;
    if (rMesh.m_sPoints.empty() || rMesh.m_sFaceVertexCounts.empty() || rMesh.m_sExtent.size() != 2)
        return SM_ERR;
    size_t lCornerCount = 0;
    for (int iCount : rMesh.m_sFaceVertexCounts)
    {
        if (iCount < 3)
            return SM_ERR;
        lCornerCount += static_cast<size_t>(iCount);
    }
    if (lCornerCount != rMesh.m_sFaceVertexIndices.size() || rMesh.m_sNormalsIndices.size() != lCornerCount)
        return SM_ERR;
    for (int iIndex : rMesh.m_sFaceVertexIndices)
        if (iIndex < 0 || static_cast<size_t>(iIndex) >= rMesh.m_sPoints.size())
            return SM_ERR;
    for (const pxr::GfVec3f& rPoint : rMesh.m_sPoints)
        if (!std::isfinite(rPoint[0]) || !std::isfinite(rPoint[1]) || !std::isfinite(rPoint[2]))
            return SM_ERR;
    for (int iIndex : rMesh.m_sNormalsIndices)
        if (iIndex < 0 || static_cast<size_t>(iIndex) >= rMesh.m_sNormals.size())
            return SM_ERR;
    for (const pxr::GfVec3f& rNormal : rMesh.m_sNormals)
        if (!std::isfinite(rNormal.GetLength()) || std::abs(rNormal.GetLength() - 1.0f) >= 1e-4f)
            return SM_ERR;
    if (rMesh.m_sSubdivisionScheme != pxr::TfToken("none"))
        return SM_ERR;
    return SM_SUCCESS;
}

static SmStatus TestTessellateBrepArray()
{
    // Two boxes in one BrepArray.
    std::vector<SmBrep*> boxes;
    for (int ii = 0; ii < 2; ++ii)
    {
        SmVector3d sOrigin(20.0 * ii, 0.0, 0.0);
        SmBrep* pBox = NULL;
        SmStatus stat = SmApiCreateBox(sOrigin, 10.0, 10.0, 10.0, pBox);
        if (stat != SM_SUCCESS || pBox == NULL)
        {
            delete pBox;
            DeleteImportedBreps(boxes);
            return (stat != SM_SUCCESS) ? stat : SM_ERR;
        }
        boxes.push_back(pBox);
    }
    const std::string sFilename = kOutputDir + "test_tessellate_brep_array.usda";
    SmStatus stat = SmApiUsdExportBreps(sFilename.c_str(), boxes, FALSE);
    DeleteImportedBreps(boxes);
    SER(stat);

    pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(sFilename);
    if (!stage)
        return SM_ERR;
    const pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath("/World/Brep0"));

    // One result per member, in order, with mesh and edges.
    std::vector<SmApiUsdTessellatedBrep> results;
    SER(SmApiUsdTessellateBrepArray(prim, SmTessellationParams(), FALSE, 5.0, results));
    if (results.size() != 2)
        return SM_ERR;
    for (size_t ii = 0; ii < results.size(); ++ii)
    {
        SER(CheckTessellatedBrep(results[ii]));
        if (results[ii].m_iPackedBrepIndex != static_cast<std::int64_t>(ii) || !results[ii].m_sFailedFaces.empty())
            return SM_ERR;
        if (results[ii].m_sBoundaryStatus != SM_SUCCESS || results[ii].m_sBoundaryVertexCounts.size() != 12)
            return SM_ERR;
    }
    if (results[0].m_sExtent[0] == results[1].m_sExtent[0])
        return SM_ERR;  // members must keep their own positions

    // Concurrent calls each use their own context and give identical results.
    std::vector<std::future<SmStatus>> jobs;
    for (int iWorker = 0; iWorker < 4; ++iWorker)
        jobs.push_back(std::async(std::launch::async, [&]() {
            for (int iIteration = 0; iIteration < 8; ++iIteration)
            {
                std::vector<SmApiUsdTessellatedBrep> concurrent;
                if (SmApiUsdTessellateBrepArray(prim, SmTessellationParams(), FALSE, 5.0, concurrent) != SM_SUCCESS
                    || concurrent.size() != results.size())
                    return SM_ERR;
                for (size_t ii = 0; ii < results.size(); ++ii)
                    if (concurrent[ii].m_sStatus != SM_SUCCESS || concurrent[ii].m_sPoints != results[ii].m_sPoints
                        || concurrent[ii].m_sFaceVertexIndices != results[ii].m_sFaceVertexIndices
                        || concurrent[ii].m_sNormals != results[ii].m_sNormals)
                        return SM_ERR;
            }
            return SM_SUCCESS;
        }));
    SmStatus concurrentStatus = SM_SUCCESS;
    for (std::future<SmStatus>& rJob : jobs)
        if (rJob.get() != SM_SUCCESS)
            concurrentStatus = SM_ERR;
    SER(concurrentStatus);

    // A zero boundary angle skips edge sampling but still meshes.
    SER(SmApiUsdTessellateBrepArray(prim, SmTessellationParams(), FALSE, 0.0, results));
    if (results.size() != 2 || !results[0].m_sBoundaryPoints.empty())
        return SM_ERR;
    SER(CheckTessellatedBrep(results[0]));

    // Real partial: a scaled cylinder with out-of-domain side UV trims; only the caps mesh.
    SmBrep* pCylinder = NULL;
    SmVector3d sCylOrigin(0.0, 0.0, 0.0), sStretch(2.0, 1.0, 1.0);
    SER(SmApiCreateCylinder(sCylOrigin, 5.0, 10.0, pCylinder));
    SER(SmApiScale(pCylinder, sStretch));
    const std::string sPartialFilename = kOutputDir + "test_tessellate_brep_array_partial.usda";
    stat = SmApiUsdExportBrep(sPartialFilename.c_str(), pCylinder, TRUE, TRUE);
    delete pCylinder;
    SER(stat);
    pxr::UsdStageRefPtr partialStage = pxr::UsdStage::Open(sPartialFilename);
    pxr::UsdPrim partialPrim = partialStage ? partialStage->GetPrimAtPath(pxr::SdfPath("/World/Brep0")) : pxr::UsdPrim();
    if (!partialPrim || !partialPrim.AddAppliedSchema(pxr::TfToken("BrepCurveUvNurbAPI")))
        return SM_ERR;
    const pxr::GfVec2d corners[4] = { { 1000, 1000 }, { 1010, 1000 }, { 1010, 1010 }, { 1000, 1010 } };
    pxr::VtArray<pxr::GfVec2d> controlVertices;
    for (int ii = 0; ii < 4; ++ii)
        controlVertices.insert(controlVertices.end(), { corners[ii], corners[(ii + 1) % 4] });
    const pxr::VtArray<unsigned int> counts = { 2, 2, 2, 2, 0, 0 };  // side face only
    const auto author = [&](const char* name, const pxr::SdfValueTypeName& type, const pxr::VtValue& value)
    { return partialPrim.CreateAttribute(pxr::TfToken(std::string("brep:curveUv:nurb:") + name), type).Set(value); };
    if (!author("vertexCount", pxr::SdfValueTypeNames->UIntArray, pxr::VtValue(counts))
        || !author("order", pxr::SdfValueTypeNames->UIntArray, pxr::VtValue(counts))
        || !author("knots", pxr::SdfValueTypeNames->DoubleArray, pxr::VtValue(pxr::VtArray<double>{ 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1 }))
        || !author("weights", pxr::SdfValueTypeNames->DoubleArray, pxr::VtValue(pxr::VtArray<double>(8, 1.0)))
        || !author("controlVertices", pxr::SdfValueTypeNames->Double2Array, pxr::VtValue(controlVertices)))
        return SM_ERR;
    SER(SmApiUsdTessellateBrepArray(partialPrim, SmTessellationParams(), FALSE, 5.0, results));
    if (results.size() != 1 || CheckTessellatedBrep(results[0]) != SM_SUCCESS || results[0].m_sBoundaryPoints.empty())
        return SM_ERR;
    if (results[0].m_sFailedFaces.size() != 1 || results[0].m_sFailedFaces[0] != 0)
        return SM_ERR;
    for (const pxr::GfVec3f& rPoint : results[0].m_sPoints)
        if (std::abs(rPoint[2]) > 1e-4f && std::abs(rPoint[2] - 10.0f) > 1e-4f)
            return SM_ERR;  // the failed side face was meshed

    // A member that fails to import does not discard its neighbours.
    const std::string sMixedFilename = kOutputDir + "test_tessellate_brep_array_mixed.usda";
    SER(CreateMixedValidAndConversionInvalidUsd(sMixedFilename));
    pxr::UsdStageRefPtr mixedStage = pxr::UsdStage::Open(sMixedFilename);
    if (!mixedStage)
        return SM_ERR;
    SER(SmApiUsdTessellateBrepArray(mixedStage->GetPrimAtPath(pxr::SdfPath("/World/Brep0")), SmTessellationParams(), FALSE,
                                    5.0, results));
    if (results.size() != 3 || CheckTessellatedBrep(results[0]) != SM_SUCCESS || CheckTessellatedBrep(results[2]) != SM_SUCCESS)
        return SM_ERR;
    if (results[1].m_iPackedBrepIndex != 1 || results[1].m_sStatus == SM_SUCCESS || !results[1].m_sPoints.empty()
        || results[1].m_sMessage.find("Import failed") == std::string::npos)
        return SM_ERR;

    // No Breps: error with one whole-prim result.
    const std::string sEmptyFilename = kOutputDir + "test_tessellate_brep_array_empty.usda";
    SER(CreateEmptyBrepArrayUsd(sEmptyFilename));
    pxr::UsdStageRefPtr emptyStage = pxr::UsdStage::Open(sEmptyFilename);
    if (!emptyStage)
        return SM_ERR;
    if (SmApiUsdTessellateBrepArray(emptyStage->GetPrimAtPath(pxr::SdfPath("/World/Brep0")), SmTessellationParams(), FALSE,
                                    5.0, results) == SM_SUCCESS)
        return SM_ERR;
    if (results.size() != 1 || results[0].m_iPackedBrepIndex >= 0 || results[0].m_sMessage.empty())
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdExportBrep
//   Create a single sphere, export it to USD, verify file was written.
//*************************************************************************
SmStatus TestSmApiUsdExportBrep()
{
    SmApiCreateContext();
    SER(TestExportRetainsGeneratedTrims());
    SER(TestBoundaryPointCurve());
    SER(TestTessellatedMeshData());
    SER(TestSubsetIndicesSkipLooplessFaces());
    SER(TestSourceOwnedMaterialFallback());
    SER(TestEmptyMeshExtraction());
    SER(TestTessellateBrepArray());

    SmVector3d sPosition(0.0, 0.0, 0.0);
    SmBrep* pSphere = NULL;
    SmStatus stat = SmApiCreateSphere(sPosition, 10.0, pSphere);
    if (stat != SM_SUCCESS || pSphere == NULL)
        return SM_ERR;

    std::string sFilename = kOutputDir + "test_export_brep.usda";
    stat = SmApiUsdExportBrep(sFilename.c_str(), pSphere);
    if (stat != SM_SUCCESS)
    {
        delete pSphere;
        return stat;
    }

    // Verify the file exists by re-importing
    std::vector<SmBrep*> imported;
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported);
    if (stat != SM_SUCCESS || imported.empty())
    {
        delete pSphere;
        DeleteImportedBreps(imported);
        return SM_ERR;
    }

    delete pSphere;
    DeleteImportedBreps(imported);
    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdExportBreps
//   Create multiple primitives, export them, verify file was written.
//*************************************************************************
SmStatus TestSmApiUsdExportBreps()
{
    SmApiCreateContext();

    SmVector3d sPosition1(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmStatus stat = SmApiCreateBox(sPosition1, 5.0, 10.0, 15.0, pBox);
    if (stat != SM_SUCCESS || pBox == NULL)
        return SM_ERR;

    SmVector3d sPosition2(20.0, 0.0, 0.0);
    SmBrep* pSphere = NULL;
    stat = SmApiCreateSphere(sPosition2, 8.0, pSphere);
    if (stat != SM_SUCCESS || pSphere == NULL)
    {
        delete pBox;
        delete pSphere;
        return SM_ERR;
    }

    std::vector<SmBrep*> breps = { pBox, pSphere };

    std::string sFilename = kOutputDir + "test_export_breps.usda";
    stat = SmApiUsdExportBreps(sFilename.c_str(), breps);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        delete pSphere;
        return stat;
    }

    // Verify by re-importing
    std::vector<SmBrep*> imported;
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported);
    if (stat != SM_SUCCESS || imported.empty())
    {
        delete pBox;
        delete pSphere;
        DeleteImportedBreps(imported);
        return SM_ERR;
    }

    delete pBox;
    delete pSphere;
    DeleteImportedBreps(imported);
    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdImportBreps
//   Import all BRep prims from an existing test file.
//*************************************************************************
SmStatus TestSmApiUsdImportBreps()
{
    SmApiCreateContext();

    std::string sFilename = kTestFilesDir + "sphere_nurbs.usda";

    SmVector3d sExistingOrigin(-10.0, 0.0, 0.0);
    SmBrep* pExistingBrep = NULL;
    SmStatus stat = SmApiCreateBox(sExistingOrigin, 1.0, 1.0, 1.0, pExistingBrep);
    if (stat != SM_SUCCESS || pExistingBrep == NULL)
        return SM_ERR;

    std::vector<SmBrep*> breps = { pExistingBrep };
    stat = SmApiUsdImportBreps(sFilename.c_str(), breps);
    if (stat != SM_SUCCESS)
    {
        DeleteImportedBreps(breps);
        return stat;
    }

    if (breps.size() != 2 || breps[0] != pExistingBrep || breps[1] == NULL)
    {
        DeleteImportedBreps(breps);
        return SM_ERR;
    }

    DeleteImportedBreps(breps);
    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdImportBrep
//   Import a single BRep prim by path from an existing test file.
//*************************************************************************
SmStatus TestSmApiUsdImportBrep()
{
    std::string sFilename = kTestFilesDir + "sphere_nurbs.usda";
    const char* sPrimPath = "/sphere_analytic_solid_preserveAnalytics_false/tn__Surface1_l8/tn__Surface1_l8/brepArray0";

    SmBrep* pBrep = NULL;
    SmStatus stat = SmApiUsdImportBrep(sFilename.c_str(), sPrimPath, pBrep);
    if (stat != SM_SUCCESS)
    {
        delete pBrep;
        return stat;
    }

    if (pBrep == NULL)
        return SM_ERR;

    delete pBrep;
    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdImportRejectsMixedValidInvalidBreps
//   Strict import rejects a valid-invalid-valid packed BrepArray atomically.
//   Explicit partial import retains both valid members and reports complete
//   member provenance without transferring ownership through the report.
//*************************************************************************
SmStatus TestSmApiUsdImportRejectsMixedValidInvalidBreps()
{
    std::string sFilename = kOutputDir + "test_import_mixed_valid_invalid.usda";
    SmStatus stat = CreateMixedValidAndConversionInvalidUsd(sFilename);
    if (stat != SM_SUCCESS)
        return stat;

    SmVector3d sentinelOrigin(-10.0, 0.0, 0.0);
    SmBrep* pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    using LegacyImportBreps = SmStatus (*)(const char*, std::vector<SmBrep*>&, SmBoolean);
    LegacyImportBreps pLegacyImportBreps = static_cast<LegacyImportBreps>(&SmApiUsdImportBreps);

    std::vector<SmBrep*> imported = { pSentinel };
    stat = pLegacyImportBreps(sFilename.c_str(), imported, FALSE);
    const bool multiImportFailedAtomically =
        stat != SM_SUCCESS && imported.size() == 1 && imported[0] == pSentinel;
    DeleteImportedBreps(imported);
    if (!multiImportFailedAtomically)
    {
        return SM_ERR;
    }

    // Selecting strict mode through the new overload preserves the same
    // rollback contract and does not publish a partial diagnostic report.
    pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    imported = { pSentinel };
    std::vector<SmApiUsdBrepImportResult> strictResults(1);
    strictResults[0].m_sPrimPath = "/stale";
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE, FALSE, &strictResults);
    const bool explicitStrictImportFailedAtomically =
        stat != SM_SUCCESS && imported.size() == 1 && imported[0] == pSentinel &&
        strictResults.empty();
    DeleteImportedBreps(imported);
    if (!explicitStrictImportFailedAtomically)
        return SM_ERR;

    SmBrep* pImported = NULL;
    stat = SmApiUsdImportBrep(sFilename.c_str(), "/World/Brep0", pImported, FALSE);
    if (stat == SM_SUCCESS || pImported != NULL)
    {
        delete pImported;
        return SM_ERR;
    }

    // Partial import without its mandatory report is invalid and must leave
    // the caller's existing output untouched.
    pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    imported = { pSentinel };
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE, TRUE, nullptr);
    const bool missingReportRejected =
        stat == SM_ERR_INVALID_INPUT && imported.size() == 1 && imported[0] == pSentinel;
    DeleteImportedBreps(imported);
    if (!missingReportRejected)
        return SM_ERR;

    // With a report, partial import succeeds and retains members 0 and 2.
    pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    imported = { pSentinel };
    std::vector<SmApiUsdBrepImportResult> results(1);
    results[0].m_sPrimPath = "/stale";
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE, TRUE, &results);

    const std::string expectedPath = "/World/Brep0";
    const bool partialImportSucceeded =
        stat == SM_SUCCESS && imported.size() == 3 && imported[0] == pSentinel &&
        imported[1] != NULL && imported[2] != NULL && imported[1] != imported[2];
    const bool resultProvenanceIsComplete =
        partialImportSucceeded && results.size() == 3 &&
        results[0].m_sPrimPath == expectedPath && results[0].m_iPackedBrepIndex == 0 &&
        results[0].m_sStatus == SM_SUCCESS && results[0].m_pBrep == imported[1] &&
        results[1].m_sPrimPath == expectedPath && results[1].m_iPackedBrepIndex == 1 &&
        results[1].m_sStatus != SM_SUCCESS && results[1].m_pBrep == NULL &&
        results[1].m_sMessage.find(expectedPath) != std::string::npos &&
        results[1].m_sMessage.find("packed Brep index 1") != std::string::npos &&
        results[2].m_sPrimPath == expectedPath && results[2].m_iPackedBrepIndex == 2 &&
        results[2].m_sStatus == SM_SUCCESS && results[2].m_pBrep == imported[2];

    DeleteImportedBreps(imported);
    if (!partialImportSucceeded || !resultProvenanceIsComplete)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdImportRejectsEmptyBrepArray
//   An authored BrepArray with no packed members is invalid, not a successful
//   import of zero objects.
//*************************************************************************
SmStatus TestSmApiUsdImportRejectsEmptyBrepArray()
{
    std::string sFilename = kOutputDir + "test_import_empty_brep_array.usda";
    SmStatus stat = CreateEmptyBrepArrayUsd(sFilename);
    if (stat != SM_SUCCESS)
        return stat;

    SmVector3d sentinelOrigin(-10.0, 0.0, 0.0);
    SmBrep* pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    std::vector<SmBrep*> imported = { pSentinel };
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE);
    const bool multiImportRejectedEmpty =
        stat == SM_ERR_INVALID_INPUT && imported.size() == 1 && imported[0] == pSentinel;
    DeleteImportedBreps(imported);
    if (!multiImportRejectedEmpty)
        return SM_ERR;

    SmBrep* pImported = NULL;
    stat = SmApiUsdImportBrep(sFilename.c_str(), "/World/Brep0", pImported, FALSE);
    if (stat != SM_ERR_INVALID_INPUT || pImported != NULL)
    {
        delete pImported;
        return SM_ERR;
    }

    // Partial mode reports a prim-level failure (no packed member exists to
    // identify) and completes without appending a new Brep.
    pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    imported = { pSentinel };
    std::vector<SmApiUsdBrepImportResult> results(1);
    results[0].m_sPrimPath = "/stale";
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE, TRUE, &results);
    const bool emptyArrayReported =
        stat == SM_SUCCESS && imported.size() == 1 && imported[0] == pSentinel &&
        results.size() == 1 && results[0].m_sPrimPath == "/World/Brep0" &&
        results[0].m_iPackedBrepIndex == -1 && results[0].m_pBrep == NULL &&
        results[0].m_sStatus == SM_ERR_INVALID_INPUT &&
        results[0].m_sMessage.find("BrepArray container import failed") != std::string::npos;
    DeleteImportedBreps(imported);
    if (!emptyArrayReported)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdImportRejectsSingularWorldTransform
//   A failure while baking the USD world transform must not publish or retain
//   a partially transformed Brep through either public import entry point.
//*************************************************************************
SmStatus TestSmApiUsdImportRejectsSingularWorldTransform()
{
    SmApiCreateContext();

    std::string sFilename = kOutputDir + "test_import_singular_world_transform.usda";
    SmStatus stat = CreateSingularTransformUsd(sFilename);
    if (stat != SM_SUCCESS)
        return stat;

    SmVector3d sentinelOrigin(-10.0, 0.0, 0.0);
    SmBrep* pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    std::vector<SmBrep*> imported = { pSentinel };
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE);
    const bool multiImportFailedAtomically =
        stat != SM_SUCCESS && imported.size() == 1 && imported[0] == pSentinel;
    DeleteImportedBreps(imported);
    if (!multiImportFailedAtomically)
        return SM_ERR;

    SmBrep* pImported = NULL;
    stat = SmApiUsdImportBrep(sFilename.c_str(), "/World/Brep0", pImported, FALSE);
    if (stat == SM_SUCCESS || pImported != NULL)
    {
        delete pImported;
        return SM_ERR;
    }

    // Partial mode turns the post-conversion transform failure into a member
    // report. No failed Brep is published or retained by the report.
    pSentinel = NULL;
    stat = SmApiCreateBox(sentinelOrigin, 1.0, 1.0, 1.0, pSentinel);
    if (stat != SM_SUCCESS || pSentinel == NULL)
        return SM_ERR;

    imported = { pSentinel };
    std::vector<SmApiUsdBrepImportResult> results;
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported, FALSE, TRUE, &results);
    const bool transformFailureReported =
        stat == SM_SUCCESS && imported.size() == 1 && imported[0] == pSentinel &&
        results.size() == 1 && results[0].m_sPrimPath == "/World/Brep0" &&
        results[0].m_iPackedBrepIndex == 0 && results[0].m_pBrep == NULL &&
        results[0].m_sStatus != SM_SUCCESS &&
        results[0].m_sMessage.find("World transform failed") != std::string::npos &&
        results[0].m_sMessage.find("packed Brep index 0") != std::string::npos;
    DeleteImportedBreps(imported);
    if (!transformFailureReported)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdRoundtrip
//   Create a box, export to USD, import back, verify the result is valid.
//*************************************************************************
SmStatus TestSmApiUsdRoundtrip()
{
    SmApiCreateContext();

    SmVector3d sPosition(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmStatus stat = SmApiCreateBox(sPosition, 10.0, 10.0, 10.0, pBox);
    if (stat != SM_SUCCESS || pBox == NULL)
        return SM_ERR;

    // Export
    std::string sFilename = kOutputDir + "test_roundtrip.usda";
    stat = SmApiUsdExportBrep(sFilename.c_str(), pBox);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        return stat;
    }

    // Import
    std::vector<SmBrep*> imported;
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        DeleteImportedBreps(imported);
        return stat;
    }

    if (imported.empty())
    {
        delete pBox;
        return SM_ERR;
    }

    SmBrep* pImported = imported[0];
    if (!pImported)
    {
        delete pBox;
        DeleteImportedBreps(imported);
        return SM_ERR;
    }

    if (!pImported->IsManifoldSolid())
    {
        delete pBox;
        DeleteImportedBreps(imported);
        return SM_ERR;
    }

    delete pBox;
    DeleteImportedBreps(imported);
    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdAppendBreps
//   Export a box, then append a sphere to the same file, import and
//   verify both BReps are present.
//*************************************************************************
SmStatus TestSmApiUsdAppendBreps()
{
    SmApiCreateContext();

    // Create and export initial box
    SmVector3d sPosition1(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmStatus stat = SmApiCreateBox(sPosition1, 5.0, 10.0, 15.0, pBox);
    if (stat != SM_SUCCESS || pBox == NULL)
        return SM_ERR;

    std::string sFilename = kOutputDir + "test_append.usda";
    stat = SmApiUsdExportBrep(sFilename.c_str(), pBox);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        return stat;
    }

    // Create sphere and append it
    SmVector3d sPosition2(20.0, 0.0, 0.0);
    SmBrep* pSphere = NULL;
    stat = SmApiCreateSphere(sPosition2, 8.0, pSphere);
    if (stat != SM_SUCCESS || pSphere == NULL)
    {
        delete pBox;
        delete pSphere;
        return SM_ERR;
    }

    std::vector<SmBrep*> appendBreps = { pSphere };
    stat = SmApiUsdAppendBreps(sFilename.c_str(), appendBreps);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        delete pSphere;
        return stat;
    }

    // Import and verify we get breps from both BrepArray prims
    std::vector<SmBrep*> imported;
    stat = SmApiUsdImportBreps(sFilename.c_str(), imported);
    if (stat != SM_SUCCESS)
    {
        delete pBox;
        delete pSphere;
        DeleteImportedBreps(imported);
        return stat;
    }

    if (imported.size() < 2)
    {
        delete pBox;
        delete pSphere;
        DeleteImportedBreps(imported);
        return SM_ERR;
    }

    delete pBox;
    delete pSphere;
    DeleteImportedBreps(imported);
    return SM_SUCCESS;
}

//*************************************************************************
// TestSmApiUsdMeshRoundtrip
//   Tessellate a box to SmPolyBrep, export as UsdGeomMesh, import back.
//*************************************************************************
SmStatus TestSmApiUsdMeshRoundtrip()
{
    SmApiCreateContext();

    SmVector3d origin(0.0, 0.0, 0.0);
    SmBrep* pBox = nullptr;
    SmApiStatus astat = SmApiCreateBox(origin, 10.0, 10.0, 10.0, pBox);
    if (astat != SM_SUCCESS || pBox == nullptr)
        return SM_ERR;

    SmPolyBrep* pPoly = nullptr;
    // Pin explicit tolerances (the known-good values) so this roundtrip mesh does not shift
    // if the tessellation defaults change.
    SmTessellationParams sParams;
    sParams.dChordHeightTol     = 0.1;
    sParams.dCurveAngleTolDeg   = 0.0;
    sParams.dSurfaceAngleTolDeg = 0.0;
    sParams.dMaxEdgeLength      = 0.0;
    sParams.dMaxAspectRatio     = 0.0;
    astat = SmApiTessellate(pBox, pPoly, sParams);
    delete pBox;
    if (astat != SM_SUCCESS || pPoly == nullptr)
    {
        delete pPoly;
        return SM_ERR;
    }

    std::string sFilename = kOutputDir + "test_mesh_roundtrip.usda";
    SmStatus stat = SmApiUsdExportMesh(sFilename.c_str(), pPoly);
    if (stat != SM_SUCCESS)
    {
        delete pPoly;
        return stat;
    }

    delete pPoly;
    pPoly = nullptr;

    SmPolyBrep* pImported = nullptr;
    stat = SmApiUsdImportMesh(sFilename.c_str(), "/World/Mesh0", pImported);
    if (stat != SM_SUCCESS)
    {
        delete pImported;
        return stat;
    }

    if (pImported == nullptr)
        return SM_ERR;

    delete pImported;
    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- SmApiUsdTessellateFile leaves the input layer unchanged even
            when the caller has a stage open on it (USD shares the layer).

NOTES --- The output is written to another directory, so the copied
          material references must still resolve from there.
***********************************************************************/
SmStatus TestSmApiUsdTessellateFileLeavesInputLayer()
{
    if (SmApiUsdEnsurePluginRegistered() != SM_SUCCESS)
        return SM_ERR;
    const std::string sInput = "../../TestFiles/brep_validator/CubeBrepArray.usda";
    const std::string sOutput = kOutputDir + "tessellate_file_held_input.usda";
    pxr::UsdStageRefPtr heldStage = pxr::UsdStage::Open(sInput);
    if (!heldStage)
        return SM_ERR;
    std::string sBefore, sAfter;
    heldStage->GetRootLayer()->ExportToString(&sBefore);

    std::vector<SmApiUsdMeshTessellationResult> results;
    SmStatus stat = SmApiUsdTessellateFile(sInput.c_str(), sOutput.c_str(), SmTessellationParams(), FALSE, 0, results);
    if (stat != SM_SUCCESS)
        return stat;

    heldStage->GetRootLayer()->ExportToString(&sAfter);
    if (sAfter != sBefore)
    {
        printf("FAIL: the caller's input layer was modified\n");
        return SM_ERR;
    }
    for (const pxr::UsdPrim& prim : heldStage->Traverse())
    {
        if (prim.GetName().GetString().rfind("tess_", 0) == 0)
        {
            printf("FAIL: the caller's stage shows %s\n", prim.GetPath().GetText());
            return SM_ERR;
        }
    }

    pxr::UsdStageRefPtr outStage = pxr::UsdStage::Open(sOutput);
    if (!outStage)
        return SM_ERR;
    size_t lMeshes = 0;
    for (const pxr::UsdPrim& prim : outStage->Traverse())
        lMeshes += prim.GetName().GetString().rfind("tess_", 0) == 0 ? 1 : 0;
    if (lMeshes != results.size())
    {
        printf("FAIL: expected %zu tess_ meshes in the output, found %zu\n", results.size(), lMeshes);
        return SM_ERR;
    }
    if (!outStage->GetPrimAtPath(pxr::SdfPath("/World/Looks/ABS_Hard_Leather_Brown/Shader")))
    {
        printf("FAIL: the output does not resolve the referenced material\n");
        return SM_ERR;
    }
    return SM_SUCCESS;
}
