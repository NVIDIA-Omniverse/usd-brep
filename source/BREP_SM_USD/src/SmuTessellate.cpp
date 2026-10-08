// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuTessellate.cpp
* PURPOSE: Implementation for tessellation and conversion of SMLib objects to USD
**********************************************************************/

#include "UsdBrepConfig.h"

// pixar includes
#include "UsdBrepSuppressPixarWarningsPush.h"        // turn off compile warnings for problematic pixar include files
#         include <pxr/usd/usdShade/shader.h>
#include "UsdBrepSuppressPixarWarningsPop.h"         // done loading problematic pixar headers - restore compile warnings

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "UsdBrepUtilities.h"
#include "UsdBrepDebugTools.h"

#include "SmuTessellate.h"
#include "SmuConvert.h"
#include "SmuUtilities.h"
#include "SmuAttribute.h"
#include "SmuTokens.h"

#include <unordered_map>
#include <vector>
#include <cstdio>

// SMLib includes
#include "SmContext.h"
#include "SmBSplineSurface.h"
#include "SmLine.h"
#include "SmPoly.h"
#include "SmBrep.h"
#include "SmFace.h"
#include "nurbs.h"

using namespace pxr;
using namespace SMU_BrepConvert;

// First material path of each subset attribute; an empty path when unbound.
static VtArray<SdfPath> sm_FirstMaterialPaths(const SmTArray<SmSdfPathAttribute*>& crAttrs)
{
    VtArray<SdfPath> sPaths;
    sPaths.reserve(crAttrs.GetSize());
    for (SmSdfPathAttribute* pAttr : crAttrs)
    {
        const SdfPathVector* pPaths = pAttr ? pAttr->GetMaterialPaths() : nullptr;
        sPaths.push_back(pPaths && !pPaths->empty() ? pPaths->front() : SdfPath());
    }
    return sPaths;
}

/*******************************************************************//**
PURPOSE: Sample BRep boundaries as USD linear curves.

NOTES: Omit empty/point edges; retain valid samples on failure.
***********************************************************************/
SmStatus SMU_BrepConvert::GetBoundaryData
(
    SmBrep* pBrep,
    double dAngleTolDeg,
    VtArray<GfVec3f>& rPoints,
    VtIntArray& rVertexCounts
)
{
    rPoints.clear();
    rVertexCounts.clear();
    if (!pBrep)
        return SM_ERR_INVALID_INPUT;
    SmTArray<SmPoint3d> points;
    SmTArray<ULONG> counts;
    SmTArray<SmEdge*> edges;
    const SmStatus status = pBrep->TessellateBoundaries(dAngleTolDeg, points, counts, edges);
    ULONG offset = 0;
    rPoints.reserve(points.GetSize());
    for (ULONG count : counts)
    {
        if (count >= 2)
        {
            rVertexCounts.push_back(static_cast<int>(count));
            for (ULONG i = 0; i < count; ++i)
            {
                const SmPoint3d& p = points[offset + i];
                rPoints.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
            }
        }
        offset += count;
    }
    return status;
}

/*******************************************************************//**
PURPOSE: Get Mesh data from an SmPolyBrep, prepared for authoring UsdGeomMesh

NOTES: rPrimvarNormals and rPrimvarNormalsIndices will have values when
    crSmPolyBrep is a tessellated SmBrep.  They will not have values when,
    e.g., the SmPolyBrep is derived from an STL file.  
***********************************************************************/
SmStatus SMU_BrepConvert::GetMeshData
(
    SmPolyBrep                    & crSmPolyBrep,                 // in : Get mesh data from this SmPolyBrep                               
    VtArray<GfVec3f>              & rExtent,                      // out: Mesh Extent                                                          
    VtArray<GfVec3f>              & rPoints,                      // out: Mesh vertex locations                                               
    VtIntArray                    & rFaceVertexCounts,            // out: Number of vertices in each face of the mesh                         
    VtIntArray                    & rFaceVertexIndices,           // out: Flat list of the index of each vertex of each face in the mesh      
    VtArray<GfVec3f>              & rPrimvarNormals,              // out: Mesh vertex normals                                                 
    VtIntArray                    & rPrimvarNormalsIndices,       // out: Flat list of the index of each normal of each vertex in the mesh    
    TfToken                       & sPrimvarNormalsInterpolation, // out: face varying normals interpolation                                  
    TfToken                       & rSubdivisionScheme,           // out: None                                                                 
    VtArray<VtIntArray>           * pOptGeomSubsetIndices,
    SmTArray<SmSdfPathAttribute*> * pOptSubsetMaterialPaths
)
{
    // SMLib containers
    SmTArray<SmPolyFace*> sPolyFaces;
    SmTArray<SmPolyEdge*> sPolyEdges;
    SmTArray<SmPolyVertex*> sPolyVertices;
    SmTArray<ULONG>         sPolyVertexNormalIndices;
    
    crSmPolyBrep.GetPolyVertices(sPolyVertices);
    ULONG lNumPolyVertices = sPolyVertices.GetSize();
    ULONG lNumPolyEdges;
    ULONG lNumFaces;
    ULONG lUsdVertexIndex = 0;
    SmBoolean bWriteFaceVaryingNormals = crSmPolyBrep.GetOKBackPtrs();
    sPolyVertexNormalIndices.SetSize(lNumPolyVertices);

    // SmPolyVertex::m_vIndices is temporary USD export state. Clear it at entry and
    // on every return so a failed export cannot leak stale indices into a later one.
    struct SmPolyVertexMeshIndexCleaner
    {
        SmTArray<SmPolyVertex*> & m_rPolyVertices;
        ULONG                     m_lNumPolyVertices;

        SmPolyVertexMeshIndexCleaner(SmTArray<SmPolyVertex*> & rPolyVertices, ULONG lNumPolyVertices)
            : m_rPolyVertices(rPolyVertices), m_lNumPolyVertices(lNumPolyVertices)
        { Clear(); }

        ~SmPolyVertexMeshIndexCleaner()
        { Clear(); }

        void Clear()
        {
            for (ULONG ii = 0; ii < m_lNumPolyVertices; ++ii)
            { m_rPolyVertices[ii]->GetIndicesRef().RemoveAll(); }
        }
    } sPolyVertexMeshIndexCleaner(sPolyVertices, lNumPolyVertices);

    // UsdGeomMesh Attribute containers
    VtArray<GfVec3f> sUsdPoints;
    VtArray<GfVec3f> sUsdNormals;
    VtIntArray sUsdNormalsIndices;
    sUsdPoints.reserve(lNumPolyVertices);
    sUsdNormals.reserve(lNumPolyVertices);

    auto DisableFaceVaryingNormals = [&](const char* pReason)
    {
        if (bWriteFaceVaryingNormals)
        {
            std::string sstring = "Warning: Face-varying normals could not be authored for mesh export";
            if (pReason != NULL && pReason[0] != '\0')
            {
                sstring += ": ";
                sstring += pReason;
            }
            sstring += ". Mesh will be exported without primvars:normals.\n";
            usdBrep_WriteString(sstring);
        }
        bWriteFaceVaryingNormals = FALSE;
        lUsdVertexIndex = 0;
        sUsdNormals.clear();
        sUsdNormalsIndices.clear();
    };

    // Calculate the bounding box
    SmExtent3d sBBox;
    for (ULONG ii = 0; ii < lNumPolyVertices; ii++)
    { sBBox.AddPoint3d(sPolyVertices[ii]->GetPoint()); }
    SmVector3d sMin(sBBox.GetMin());
    SmVector3d sMax(sBBox.GetMax());

    // IsInit() is TRUE while the box is still in its initial negative-volume state, i.e. no
    // point was ever added. Publishing it would author an extent of
    // (+SM_BIG_DOUBLE, -SM_BIG_DOUBLE); an empty mesh gets an empty extent instead.
    VtArray<GfVec3f> sUsdExtent;
    if (!sBBox.IsInit())
    {
        sUsdExtent = {
            GfVec3f(static_cast<float>(sMin.x), static_cast<float>(sMin.y), static_cast<float>(sMin.z)),
            GfVec3f(static_cast<float>(sMax.x), static_cast<float>(sMax.y), static_cast<float>(sMax.z)) };
    }

    // Set the vertex based attributes: points and normals
    for (ULONG ii = 0; ii < lNumPolyVertices; ++ii)
    {
        // Loop local
        SmPolyVertex* pPolyVertex = sPolyVertices[ii];
        const SmPoint3d& crPoint = pPolyVertex->GetPoint();

        // Get Faces and Normals on pPolyVertex
        const SmTArray<SmVector3d>  & crNormals       = pPolyVertex->GetNormalsRef();
        const SmTArray<SmTopology*> & crFaces         = pPolyVertex->GetFacesRef();
        //      SmTArray<ULONG>       & rUsdMeshIndices = pPolyVertex->GetIndicesRef();
        lNumFaces = crFaces.GetSize();

        // Record the first normal index for this PolyVertex
        sPolyVertexNormalIndices[ii] = lUsdVertexIndex;

        // For each SmPolyFace->Original_SmFace, create a new normal
        // Only do this if the Brep is still valid.  If the Brep is deleted before export,
        // the Original_SmFace pointers will be stale.
        if (bWriteFaceVaryingNormals)
        {
            if (lNumFaces != crNormals.GetSize())
            {
                DisableFaceVaryingNormals("PolyVertex face/normal table size mismatch");
            }
            else
            {
                for (ULONG jj = 0; jj < lNumFaces; ++jj)
                {
                    SmFace* pFace = SM_CAST_PTR(SmFace, crFaces[jj]);
                    if (pFace == NULL)
                    {
                        DisableFaceVaryingNormals("PolyVertex face/normal table contains non-SmFace entry");
                        break;
                    }

                    // Normalize the normal vector
                    pxr::GfVec3f normalVec((float)crNormals[jj].x, (float)crNormals[jj].y, (float)crNormals[jj].z);
                    normalVec.Normalize();

                    // Set Usd Normal
                    sUsdNormals.push_back(normalVec);

                    // Increment the UsdVertexIndex when we have a valid SmFace.
                    ++lUsdVertexIndex;
                } // end
            }
        } // end if GetOKBackPtrs()

        // Assign index value for later access
        pPolyVertex->GetIndicesRef().Add(ii);

        // Set Usd Point
        sUsdPoints.push_back(pxr::GfVec3f((float)crPoint.x, (float)crPoint.y, (float)crPoint.z));

    } // end setting vertex base attributes

    // Get the PolyFaces
    crSmPolyBrep.GetPolyFaces(sPolyFaces);
    ULONG lNumPolyFaces = sPolyFaces.GetSize();

    // UsdGeomMesh Attribute containers
    VtIntArray sUsdIndices;
    VtIntArray sUsdFaceVertexCounts;
    sUsdIndices.         reserve(3 * lNumPolyFaces);
    sUsdNormalsIndices.  reserve(3 * lNumPolyFaces);
    sUsdFaceVertexCounts.reserve(lNumPolyFaces);

    // GeomSubset face indices: keep heap in std containers (not placement-new into SmPolyBrep context;
    // the old path leaked and corrupted the arena when the brep was freed after export).
    std::unordered_map<SmSdfPathAttribute*, std::vector<uint32_t>> sSubsetMaps;

    // set the face based attributes: vertex counts and indices
    for (ULONG ii = 0; ii < lNumPolyFaces; ++ii)
    {
        // Loop locals
        SmPolyFace* pPolyFace = sPolyFaces[ii];
        SmPolyLoop* pPolyLoop = pPolyFace->GetOuterPolyLoop(); // Only look at outer loop. USD doesn't have inner loop
                                                               // concept
        if (pPolyLoop == NULL)  // JLMCC: Avoids a crash in the Alias connector.
        { continue;}

        // Get PolyEdges to traverse
        pPolyLoop->GetPolyEdges(sPolyEdges);
        lNumPolyEdges = sPolyEdges.GetSize();
        sUsdFaceVertexCounts.push_back((int)lNumPolyEdges);

        SmFace* pOriginalFaceLabel = pPolyFace->GetOriginalFace();
        if (bWriteFaceVaryingNormals && pOriginalFaceLabel == NULL)
        {
            DisableFaceVaryingNormals("PolyFace is missing original SmFace label");
        }
        SmFace* pLiveFace = pPolyFace->GetOKBackPtrs() ? pOriginalFaceLabel : NULL;
        SmSdfPathAttribute* pAttr = SM_CAST_PTR(SmSdfPathAttribute, pPolyFace->FindAttribute(SM_AI_MATERIAL_BINDING));
        if (pAttr == NULL && pLiveFace != NULL)
        {
            pAttr = SM_CAST_PTR(SmSdfPathAttribute, pLiveFace->FindAttribute(SM_AI_MATERIAL_BINDING));
        }
        if (pAttr != NULL)
        {
            // Index the emitted face, not sPolyFaces: faces with no outer loop are skipped
            // above, so ii runs ahead of the USD face array once any face is dropped.
            sSubsetMaps[pAttr].push_back(static_cast<uint32_t>(sUsdFaceVertexCounts.size() - 1));
        }

        // For each PolyEdge, get the PolyVertex->UsdMeshIndex
        for (ULONG jj = 0; jj < lNumPolyEdges; ++jj)
        {
            // Loop locals
            ULONG lIndexOnPolyVertex;
            SmPolyEdge* pEdge = sPolyEdges[jj];
            SmPolyVertex* pPolyVertex = pEdge->GetStartPolyVertex();

            // Get vertex properties
            const SmTArray<SmTopology*>& rFaces          = pPolyVertex->GetFacesRef();
            const SmTArray<ULONG>      & rUsdMeshIndices = pPolyVertex->GetIndicesRef();

            ULONG lNumUsdMeshIndices = rUsdMeshIndices.GetSize();
            SER_MSG(lNumUsdMeshIndices > 0 ? SM_SUCCESS : SM_ERR,
                    _T("Missing USD mesh index on PolyVertex during USD mesh export"));
            const ULONG lUsdMeshIndex = rUsdMeshIndices[lNumUsdMeshIndices - 1];
            ULONG lUsdNormalIndex = 0;

            if (bWriteFaceVaryingNormals)
            {
                // Set the normal of this PolyVertex from the Original-face normal.
                // may be stale if Brep is deleted before export.
                if (rFaces.FindElement(pOriginalFaceLabel, lIndexOnPolyVertex))
                { lUsdNormalIndex = sPolyVertexNormalIndices[lUsdMeshIndex] + lIndexOnPolyVertex; }
                else
                {
                    DisableFaceVaryingNormals("PolyVertex face table does not contain PolyFace original SmFace");
                }
            }

            // Append required mesh indices; append optional normal indices while they remain available.
            sUsdIndices.push_back((int)lUsdMeshIndex);
            if (bWriteFaceVaryingNormals)
            { sUsdNormalsIndices.push_back((int)lUsdNormalIndex); }
        } // end for each PolyEdge
    } // end for each PolyFace: set the face based attributes

    // output results
    rExtent                      = sUsdExtent;
    rPoints                      = sUsdPoints;
    rFaceVertexCounts            = sUsdFaceVertexCounts;
    rFaceVertexIndices           = sUsdIndices;
    rPrimvarNormals              = sUsdNormals;
    rPrimvarNormalsIndices       = sUsdNormalsIndices;
    sPrimvarNormalsInterpolation = UsdGeomTokens->faceVarying;
    rSubdivisionScheme           = UsdGeomTokens->none;

    // output option SubsetGeom indices
    if ( pOptGeomSubsetIndices )
    {
        pOptGeomSubsetIndices->resize(0);
        pOptGeomSubsetIndices->reserve(sSubsetMaps.size());
        SmTArray<SmSdfPathAttribute*> sSmSubsetAttrs;

        for (const auto& kv : sSubsetMaps)
        {
            const std::vector<uint32_t>& g = kv.second;
            VtIntArray sVtGeomSubset;
            sVtGeomSubset.reserve(static_cast<size_t>(g.size()));
            for (uint32_t faceIx : g)
            {
                sVtGeomSubset.push_back(static_cast<int>(faceIx));
            }
            pOptGeomSubsetIndices->emplace_back(sVtGeomSubset);
            sSmSubsetAttrs.Add(kv.first);
        }

        if (pOptSubsetMaterialPaths)
        {
            *pOptSubsetMaterialPaths = sSmSubsetAttrs;
        }
    }
  
    return SM_SUCCESS;

} // end SMU_BrepConvert::GetMeshData

void SMU_BrepConvert::TessellatedMeshDeleter::operator()(SmPolyBrep* pMesh) const
{
    if (pMesh)
        pMesh->SetOKBackPtrs(FALSE);
    delete pMesh;
}

/*******************************************************************//**
PURPOSE: Extract tessellated mesh arrays and owned material paths.

NOTES: Source Brep must remain alive and topologically unmodified; its SmFace pointers
       label the mesh and are dereferenced here. The back-pointer flag is restored on exit.
       rMaterialPaths pairs with rSubsetIndices; an unbound subset gives an empty path.
***********************************************************************/
SmStatus SMU_BrepConvert::GetTessellatedMeshData
(
        SmPolyBrep& rMesh,
        pxr::VtArray<pxr::GfVec3f>& rExtent,
        pxr::VtArray<pxr::GfVec3f>& rPoints,
        pxr::VtIntArray& rFaceVertexCounts,
        pxr::VtIntArray& rFaceVertexIndices,
        pxr::VtArray<pxr::GfVec3f>& rNormals,
        pxr::VtIntArray& rNormalsIndices,
        pxr::TfToken& rNormalsInterpolation,
        pxr::TfToken& rSubdivisionScheme,
        pxr::VtArray<pxr::VtIntArray>& rSubsetIndices,
        pxr::VtArray<pxr::SdfPath>& rMaterialPaths
)
{
    // Clear outputs up front so a failure never leaves stale results.
    rExtent.clear();
    rPoints.clear();
    rFaceVertexCounts.clear();
    rFaceVertexIndices.clear();
    rNormals.clear();
    rNormalsIndices.clear();
    rNormalsInterpolation = pxr::TfToken();
    rSubdivisionScheme = pxr::TfToken();
    rSubsetIndices.clear();
    rMaterialPaths.clear();

    // Restore, not force FALSE: SmTess caches keep back-pointers on purpose.
    struct RestoreBackPointers
    {
        SmPolyBrep& rMesh;
        SmBoolean bPrevOKBackPtrs;
        ~RestoreBackPointers() { rMesh.SetOKBackPtrs(bPrevOKBackPtrs); }
    } sRestoreGuard{rMesh, rMesh.GetOKBackPtrs()};
    rMesh.SetOKBackPtrs(TRUE);

    SmTArray<SmSdfPathAttribute*> sMaterialPaths;
    SER(GetMeshData(rMesh, rExtent, rPoints, rFaceVertexCounts, rFaceVertexIndices,
                   rNormals, rNormalsIndices, rNormalsInterpolation, rSubdivisionScheme,
                   &rSubsetIndices, &sMaterialPaths));

    rMaterialPaths = sm_FirstMaterialPaths(sMaterialPaths);
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Author precomputed mesh arrays (GetTessellatedMeshData) onto a
    UsdGeomMesh prim spec: extent, points, topology, subdivision scheme,
    normals, and one bound GeomSubset per face material.
***********************************************************************/
SMU_EXPORT SmStatus SMU_BrepConvert::PopulateMeshAttr
 (const pxr::VtArray<pxr::GfVec3f>    & crExtent,
  const pxr::VtArray<pxr::GfVec3f>    & crPoints,
  const pxr::VtIntArray               & crFaceVertexCounts,
  const pxr::VtIntArray               & crFaceVertexIndices,
  const pxr::VtArray<pxr::GfVec3f>    & crNormals,
  const pxr::VtIntArray               & crNormalsIndices,
  const pxr::TfToken                  & crNormalsInterpolation,
  const pxr::TfToken                  & crSubdivisionScheme,
  const pxr::VtArray<pxr::VtIntArray> & crSubsetIndices,
  const pxr::VtArray<pxr::SdfPath>    & crMaterialPaths,  // pairs with crSubsetIndices; empty = unbound
  const pxr::SdfPrimSpecHandle        & rPrimSpecHandle)
{
    if (!rPrimSpecHandle || rPrimSpecHandle->IsDormant())
    {
        SER_MSG(SM_ERR_INVALID_INPUT, _T("no or dormant rPrimSpecHandle input."));
    }
    // Check before any write: a rejected call must leave the prim unchanged.
    if (crSubsetIndices.size() != crMaterialPaths.size())
    {
        SER_MSG(SM_ERR_INVALID_INPUT, _T("crSubsetIndices and crMaterialPaths must have the same size."));
    }

    // Start of edits to USD
    SdfChangeBlock changeBlock;

    // set attributes all meshes have
    pxr::SdfAttributeSpecHandle extentAttr = UsdBrepData::setAttributeSpec(rPrimSpecHandle, UsdGeomTokens->extent,              SdfValueTypeNames->Vector3fArray, VtValue(crExtent));
    if (!extentAttr)
    {
        std::string sstring = "Error: Failed to set extent attribute on mesh\n";
        usdBrep_WriteString(sstring);
    }
    
    pxr::SdfAttributeSpecHandle pointsAttr = UsdBrepData::setAttributeSpec(rPrimSpecHandle, UsdGeomTokens->points,              SdfValueTypeNames->Vector3fArray, VtValue(crPoints));
    if (!pointsAttr)
    {
        std::string sstring = "Error: Failed to set points attribute on mesh\n";
        usdBrep_WriteString(sstring);
    }
    
    pxr::SdfAttributeSpecHandle faceVertexCountsAttr = UsdBrepData::setAttributeSpec(rPrimSpecHandle, UsdGeomTokens->faceVertexCounts,    SdfValueTypeNames->IntArray,      VtValue(crFaceVertexCounts));
    if (!faceVertexCountsAttr)
    {
        std::string sstring = "Error: Failed to set faceVertexCounts attribute on mesh\n";
        usdBrep_WriteString(sstring);
    }
    
    pxr::SdfAttributeSpecHandle faceVertexIndicesAttr = UsdBrepData::setAttributeSpec(rPrimSpecHandle, UsdGeomTokens->faceVertexIndices,   SdfValueTypeNames->IntArray,      VtValue(crFaceVertexIndices));
    if (!faceVertexIndicesAttr)
    {
        std::string sstring = "Error: Failed to set faceVertexIndices attribute on mesh\n";
        usdBrep_WriteString(sstring);
    }
    
    pxr::SdfAttributeSpecHandle subdivisionSchemeAttr = UsdBrepData::setAttributeSpec(rPrimSpecHandle, UsdGeomTokens->subdivisionScheme,   SdfValueTypeNames->Token,         VtValue(crSubdivisionScheme));
    if (!subdivisionSchemeAttr)
    {
        std::string sstring = "Error: Failed to set subdivisionScheme attribute on mesh\n";
        usdBrep_WriteString(sstring);
    }

    // set the normals and normal indices for meshes that have them
    if (!crNormals.empty())
    {
        pxr::SdfAttributeSpecHandle normalsIndicesAttr = UsdBrepData::setAttributeSpec(rPrimSpecHandle, SmuTessTokens->primvarNormalsIndices, SdfValueTypeNames->IntArray,      VtValue(crNormalsIndices));
        if (!normalsIndicesAttr)
        {
            std::string sstring = "Error: Failed to set primvarNormalsIndices attribute on mesh\n";
            usdBrep_WriteString(sstring);
        }
        
        SdfAttributeSpecHandle sNormalSpecHandle = UsdBrepData::setAttributeSpec(rPrimSpecHandle, SmuTessTokens->primvarNormals, SdfValueTypeNames->Normal3fArray, VtValue(crNormals));
        if (!sNormalSpecHandle)
        {
            std::string sstring = "Error: Failed to set primvarNormals attribute on mesh\n";
            usdBrep_WriteString(sstring);
        }
        else
        {
            sNormalSpecHandle->SetField(UsdGeomTokens->interpolation, VtValue(crNormalsInterpolation));
        }
    }

    // Create GeomSubsets per Face->material.  
    size_t lNumPaths = crMaterialPaths.size();
    if (lNumPaths == 1 && crMaterialPaths[0].IsEmpty())
    { lNumPaths = 0; }

    for ( ULONG ii = 0; ii < lNumPaths; ++ii)
    {
        std::string sName = "subset_" + std::to_string(ii);
        SdfPrimSpecHandle sSubsetPrimSpecHandle = SdfPrimSpec::New(rPrimSpecHandle, sName, SdfSpecifierDef, "GeomSubset");
        if (!sSubsetPrimSpecHandle)
        {
            std::string sstring = "Error: Failed to create GeomSubset prim spec for subset_" + std::to_string(ii) + "\n";
            usdBrep_WriteString(sstring);
            continue;
        }
        
        pxr::SdfAttributeSpecHandle elementTypeAttr = UsdBrepData::setAttributeSpec(sSubsetPrimSpecHandle, UsdGeomTokens->elementType, SdfValueTypeNames->Token, VtValue(UsdGeomTokens->face));
        if (!elementTypeAttr)
        {
            std::string sstring = "Error: Failed to set elementType attribute for subset_" + std::to_string(ii) + "\n";
            usdBrep_WriteString(sstring);
        }
        
        pxr::SdfAttributeSpecHandle indicesAttr = UsdBrepData::setAttributeSpec(sSubsetPrimSpecHandle, UsdGeomTokens->indices,     SdfValueTypeNames->IntArray, VtValue(crSubsetIndices[ii]));
        if (!indicesAttr)
        {
            std::string sstring = "Error: Failed to set indices attribute for subset_" + std::to_string(ii) + "\n";
            usdBrep_WriteString(sstring);
        }
        
        // Apply material binding
        bool addSchemaResult = UsdBrepData::AddAppliedSchema(sSubsetPrimSpecHandle, UsdSchemaRegistry::GetSchemaTypeName
                                                                         (pxr::TfType::Find<pxr::UsdShadeMaterialBindingAPI>()));
        if (!addSchemaResult)
        {
            std::string sstring = "Error: Failed to add UsdShadeMaterialBindingAPI schema to subset_" + std::to_string(ii) + "\n";
            usdBrep_WriteString(sstring);
        }
        pxr::SdfRelationshipSpecHandle materialBindingHandle =
            pxr::SdfRelationshipSpec::New(sSubsetPrimSpecHandle, UsdShadeTokens->materialBinding, false);
        materialBindingHandle->GetTargetPathList().ClearEditsAndMakeExplicit();
        if (!crMaterialPaths[ii].IsEmpty())
        { materialBindingHandle->GetTargetPathList().Add(crMaterialPaths[ii]); }
    }

    return SM_SUCCESS;

} // End SMU_BrepConvert::PopulateMeshAttr (arrays)

/*******************************************************************//**
PURPOSE: Converts an SmPolyBrep into a UsdGeomMesh

NOTES:
***********************************************************************/
SMU_EXPORT SmStatus SMU_BrepConvert::PopulateMeshAttr
 (SmPolyBrep                   & crSmPolyBrep,    // in : PolyBrep being converted to a UsdGeomMesh      
  const pxr::SdfPrimSpecHandle & rPrimSpecHandle) // in : PrimSpecHandle of the UsdGeomMesh
{
    if (!rPrimSpecHandle || rPrimSpecHandle->IsDormant())
    {
        SER_MSG(SM_ERR_INVALID_INPUT, _T("no or dormant rPrimSpecHandle input."));
    }

    // UsdGeomMesh Attribute containers
    VtArray<GfVec3f>              sUsdPoints;
    VtArray<GfVec3f>              sUsdNormals;
    VtArray<GfVec3f>              sUsdExtent;
    VtIntArray                    sUsdIndices;
    VtIntArray                    sUsdFaceVertexCounts;
    VtIntArray                    sUsdNormalsIndices;
    TfToken                       sNormalsInterpolation;
    TfToken                       sSubdivisionScheme;
    VtArray<VtIntArray>           sGeomSubsets;
    SmTArray<SmSdfPathAttribute*> sSubsetMaterialPaths;

    SmStatus status = GetMeshData(crSmPolyBrep, sUsdExtent, sUsdPoints, sUsdFaceVertexCounts, sUsdIndices, sUsdNormals, sUsdNormalsIndices, sNormalsInterpolation, sSubdivisionScheme, &sGeomSubsets, &sSubsetMaterialPaths);
    if (status != SM_SUCCESS)
    { return status; }

    SER(PopulateMeshAttr(sUsdExtent, sUsdPoints, sUsdFaceVertexCounts, sUsdIndices, sUsdNormals, sUsdNormalsIndices,
                         sNormalsInterpolation, sSubdivisionScheme, sGeomSubsets,
                         sm_FirstMaterialPaths(sSubsetMaterialPaths), rPrimSpecHandle));

    SdfChangeBlock changeBlock;

    // Set the mesh material binding
    SmSdfPathAttribute * pAttr = SM_CAST_PTR(SmSdfPathAttribute, crSmPolyBrep.FindAttribute(SM_AI_MATERIAL_BINDING));
    if (pAttr)
    {
        bool addSchemaResult = UsdBrepData::AddAppliedSchema(rPrimSpecHandle, UsdSchemaRegistry::GetSchemaTypeName
                                                                   (pxr::TfType::Find<pxr::UsdShadeMaterialBindingAPI>()));
        if (!addSchemaResult)
        {
            std::string sstring = "Error: Failed to add UsdShadeMaterialBindingAPI schema to mesh\n";
            usdBrep_WriteString(sstring);
        }
        
        pxr::SdfRelationshipSpecHandle materialBindingHandle =
            pxr::SdfRelationshipSpec::New(rPrimSpecHandle, UsdShadeTokens->materialBinding, false);
        if (!materialBindingHandle)
        {
            std::string sstring = "Error: Failed to create material binding relationship spec for mesh\n";
            usdBrep_WriteString(sstring);
        }
        else
        {
            materialBindingHandle->GetTargetPathList().ClearEditsAndMakeExplicit();
            if (pAttr->GetMaterialPaths()->size() > 0)
            { materialBindingHandle->GetTargetPathList().Add(pAttr->GetMaterialPaths()->front()); }
        }
    }

    // Set the mesh backpointer to source Brep
    pAttr = SM_CAST_PTR(SmSdfPathAttribute, crSmPolyBrep.FindAttribute(SM_AI_BREP_ARRAY));
    if (pAttr && (pAttr->GetMaterialPaths()->size() > 0) )
    {
        SdfPath sBrepPath(pAttr->GetMaterialPaths()->front());
        SdfRelationshipSpecHandle brepSourceRel = SdfRelationshipSpec::New( rPrimSpecHandle, TfToken("BrepSource"), true);
        brepSourceRel->GetTargetPathList().ClearEditsAndMakeExplicit();
        brepSourceRel->GetTargetPathList().Add(sBrepPath);

        SdfRelationshipSpecHandle derivedMeshRel = rPrimSpecHandle->GetLayer()->GetRelationshipAtPath(sBrepPath);
        if (derivedMeshRel)
        { derivedMeshRel->GetTargetPathList().Add(rPrimSpecHandle->GetPath()); }
    }

    // Set the mesh xform ops
    SmUsdXformAttribute * pXformAttr = SM_CAST_PTR(SmUsdXformAttribute, crSmPolyBrep.FindAttribute(SM_AI_XFORM_OP));
    if (pXformAttr)
    {
        pXformAttr->AttachAttributeToXformable(rPrimSpecHandle);
    }

    return SM_SUCCESS;

} // End SMU_BrepConvert::PopulateMeshAttr
