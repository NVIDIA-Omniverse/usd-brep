// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsd.cpp
* PURPOSE: Implementation of the high-level USD import/export API.
**********************************************************************/

#include "SmApiUsd.h"

#include <algorithm>

// USD includes
#include "UsdBrepHeaders.h"
#include <pxr/base/tf/diagnostic.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/gf/matrix4d.h>

// BREP_SM_USD conversion layer
#include "SmuConvert.h"
#include "SmuTessellate.h"

// BREP_USD_DATA layer
#include "UsdBrepArrayData.h"
#include "UsdBrepWrite.h"
#include "UsdBrepRead.h"
#include "UsdBrepUtilities.h"

// SMLib
#include "SmBrep.h"
#include "SmPoly.h"
#include "SmContext.h"
#include "SmMessages.h"

// SM_API
#include "SmApiGeneral.h"

using namespace pxr;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static SmStatus EnsurePluginRegistered()
{
    if (!UsdBrepData::IsOmniSolidResourcesPluginRegistered())
    {
        if (!UsdBrepData::RegisterOmniSolidResourcesPlugin())
            return SM_ERR;
    }
    return SM_SUCCESS;
}

static void DeleteBreps(std::vector<SmBrep*>& rBreps)
{
    for (SmBrep* pBrep : rBreps)
        delete pBrep;
    rBreps.clear();
}

// Public, idempotent registration entrypoint (see SmApiUsd.h). The import/export
// wrappers below already call the static helper on demand; this exposes it so a
// host can register explicitly and fail fast on a bad OMNISOLID_PLUGIN_PATH.
SmStatus SmApiUsdEnsurePluginRegistered()
{
    return EnsurePluginRegistered();
}

// Register, then check the plugin was registered in time to read BrepArrays. Only for readers: the check
// fixes OpenUSD's schema types, which reading a stage does anyway.
static SmStatus EnsurePluginRegisteredForReading()
{
    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    if (!UsdBrepData::AreOmniSolidSchemasKnown())
    {
        ERR_MSG(_T("SmApiUsd: the omniSolid schema plugin was registered after a USD stage was opened, so BrepArray ")
                _T("prims cannot be read. Import usd_brep (or call SmApiUsdEnsurePluginRegistered) before opening ")
                _T("any stage, or set PXR_PLUGINPATH_NAME to the omniSolid/resources directory."));
        return SM_ERR;
    }
    return SM_SUCCESS;
}

static void CollectBrepArrayPrims(const UsdPrim& prim, std::vector<UsdPrim>& rBrepPrims)
{
    for (auto child : prim.GetAllChildren())
    {
        if (child.GetTypeName() == TfToken("BrepArray"))
            rBrepPrims.push_back(child);

        CollectBrepArrayPrims(child, rBrepPrims);
    }
}

// Compute a prim's local-to-world transform, or identity if it is not imageable.
static GfMatrix4d ComputeWorldXform(const UsdPrim& crPrim)
{
    if (crPrim.IsA<UsdGeomImageable>())
        return UsdGeomImageable(crPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
    return GfMatrix4d(1.0);
}

// Bake a USD local-to-world transform into freshly imported SmBreps so they
// land at their authored scene position. BrepArray geometry is stored in the
// prim's local space (placement lives in the prim's xformOps and ancestors),
// so world geometry = local geometry * worldXform; previously this transform
// was dropped, collapsing placed assemblies to their local origins.
//
// USD GfMatrix4d is row-major with the translation in row 3 -- exactly the
// layout SmBrep::Transform(double[4][4]) expects (rows 0..2 are basis vectors,
// row 3 is displacement), so the copy is element-for-element with NO transpose.
// SmBrep::Transform internally converts analytics to NURBS for non-uniform scale.
static SmStatus ApplyWorldXformToBrep(
    const GfMatrix4d& crWorldXform,
    const SdfPath& crPrimPath,
    std::int64_t iPackedBrepIndex,
    SmBrep* pBrep)
{
    if (!pBrep)
    {
        const std::basic_string<TCHAR> sPrimPath = smos_ToTChar(crPrimPath.GetText());
        TCHAR sMessage[SM_TBLOCK_SIZE];
        SM_SPRINTF(
            sMessage,
            _T("SmApiUsd: null imported Brep for prim [%.800s], packed Brep index %lld"),
            sPrimPath.c_str(),
            static_cast<long long>(iPackedBrepIndex));
        ERR_MSG(sMessage);
        return SM_ERR;
    }

    if (crWorldXform == GfMatrix4d(1.0))
        return SM_SUCCESS; // identity (e.g. SMLib-authored stages): nothing to do

    double sMatrix[4][4];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            sMatrix[r][c] = crWorldXform[r][c];

    SmStatus stat = pBrep->Transform(sMatrix);
    if (stat != SM_SUCCESS)
    {
        const std::basic_string<TCHAR> sPrimPath = smos_ToTChar(crPrimPath.GetText());
        TCHAR sMessage[SM_TBLOCK_SIZE];
        SM_SPRINTF(
            sMessage,
            _T("SmApiUsd: world transform failed for prim [%.800s], packed Brep index %lld (status %ld)"),
            sPrimPath.c_str(),
            static_cast<long long>(iPackedBrepIndex),
            static_cast<long>(stat));
        ERR_MSG(sMessage);
        return stat;
    }

    return SM_SUCCESS;
}

static SmStatus ApplyWorldXformToBreps(
    const GfMatrix4d& crWorldXform,
    const SdfPath& crPrimPath,
    std::vector<SmBrep*>& rBreps)
{
    for (size_t iBrep = 0; iBrep < rBreps.size(); ++iBrep)
    {
        SmStatus stat = ApplyWorldXformToBrep(
            crWorldXform, crPrimPath, static_cast<std::int64_t>(iBrep), rBreps[iBrep]);
        if (stat != SM_SUCCESS)
            return stat;
    }

    return SM_SUCCESS;
}

static void DeleteBrepImportResults(std::vector<SMU_BrepConvert::BrepImportResult>& rResults)
{
    for (SMU_BrepConvert::BrepImportResult& rResult : rResults)
    {
        delete rResult.pBrep;
        rResult.pBrep = nullptr;
    }
    rResults.clear();
}

static bool IsFatalBrepImportStatus(SmStatus status)
{
    return status == SM_ERR_OUT_OF_MEMORY ||
           status == SM_ERR_FATAL ||
           status == SM_ERR_ASSERT_FAILURE ||
           status == SM_ERR_LICENSE_EXPIRED;
}

static std::string MakeBrepImportMessage(
    const SdfPath& crPrimPath,
    std::int64_t iPackedBrepIndex,
    SmStatus status,
    const char* pFailure)
{
    std::string sMessage = pFailure;
    sMessage += " for prim [";
    sMessage += crPrimPath.GetString();
    sMessage += "]";
    if (iPackedBrepIndex >= 0)
    {
        sMessage += ", packed Brep index ";
        sMessage += std::to_string(iPackedBrepIndex);
    }
    sMessage += " (status ";
    sMessage += std::to_string(static_cast<long>(status));
    sMessage += ")";
    return sMessage;
}

static void CollectUsdGeomMeshes(const UsdStageRefPtr& stage, std::vector<UsdPrim>& rMeshPrims)
{
    for (const UsdPrim& prim : stage->Traverse(UsdTraverseInstanceProxies()))
    {
        if (!prim.IsValid() || !prim.IsActive())
            continue;
        if (prim.IsA<UsdGeomMesh>())
            rMeshPrims.push_back(prim);
    }
}

/// Validate a prim spec produced by \c SdfPrimSpec::New before authoring mesh attributes.
/// \return SM_SUCCESS if the handle is valid and the spec is not dormant; otherwise logs and returns SM_ERR.
static SmStatus ValidateMeshPrimSpecForExport(
    const SdfPrimSpecHandle& meshSpec,
    const std::string&       meshName,
    const SdfPrimSpecHandle& parentPrimSpec)
{
    if (!meshSpec)
    {
        TF_RUNTIME_ERROR(
            "SmApiUsd: SdfPrimSpec::New failed or returned an invalid handle for UsdGeomMesh '%s' under parent %s",
            meshName.c_str(),
            parentPrimSpec->GetPath().GetString().c_str());
        return SM_ERR;
    }
    if (meshSpec->IsDormant())
    {
        TF_RUNTIME_ERROR(
            "SmApiUsd: UsdGeomMesh prim spec is dormant (mesh='%s', parent=%s)",
            meshName.c_str(),
            parentPrimSpec->GetPath().GetString().c_str());
        return SM_ERR;
    }
    return SM_SUCCESS;
}

// ---------------------------------------------------------------------------
//  USD -> SMLib
// ---------------------------------------------------------------------------

static SmStatus ImportBreps(
    const char* pFileName,
    std::vector<SmBrep*>& rSmBreps,
    SmBoolean bHealerIsEnabled,
    SmBoolean bAllowPartial,
    std::vector<SmApiUsdBrepImportResult>* pImportResults)
{
    if (pImportResults)
        pImportResults->clear();

    // A partial result is useful only if its failures remain visible to the
    // caller. Require the report rather than silently dropping them.
    if (bAllowPartial && !pImportResults)
        return SM_ERR_INVALID_INPUT;

    if (!pFileName)
        return SM_ERR;

    SmStatus regStat = EnsurePluginRegisteredForReading();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::Open(pFileName);
    if (!stage)
        return SM_ERR;

    std::vector<UsdPrim> brepPrims;
    CollectBrepArrayPrims(stage->GetPseudoRoot(), brepPrims);

    if (brepPrims.empty())
        return SM_ERR;

    const SmContext* pContext = SmApiGetOrCreateContext();
    std::vector<SmBrep*> importedBreps;
    std::vector<SmApiUsdBrepImportResult> importResults;

    for (auto& prim : brepPrims)
    {
        UsdGeomGprim gprim(prim);
        std::vector<SMU_BrepConvert::BrepImportResult> conversionResults;
        SmStatus stat = SMU_BrepConvert::BrepMove_UsdToSMLibWithResults(
            *pContext,
            gprim,
            conversionResults,
            bHealerIsEnabled,
            bAllowPartial ? FALSE : TRUE);
        if (stat != SM_SUCCESS || conversionResults.empty())
        {
            DeleteBrepImportResults(conversionResults);
            const SmStatus failureStatus = (stat != SM_SUCCESS) ? stat : SM_ERR;

            if (!bAllowPartial || IsFatalBrepImportStatus(failureStatus))
            {
                DeleteBreps(importedBreps);
                return failureStatus;
            }

            SmApiUsdBrepImportResult sResult;
            sResult.m_sPrimPath = prim.GetPath().GetString();
            sResult.m_sStatus = failureStatus;
            sResult.m_sMessage = MakeBrepImportMessage(
                prim.GetPath(), -1, failureStatus, "BrepArray container import failed");
            importResults.push_back(sResult);
            continue;
        }

        const GfMatrix4d worldXform = ComputeWorldXform(prim);
        for (SMU_BrepConvert::BrepImportResult& rConversionResult : conversionResults)
        {
            SmApiUsdBrepImportResult sResult;
            sResult.m_sPrimPath = prim.GetPath().GetString();
            sResult.m_iPackedBrepIndex = static_cast<std::int64_t>(rConversionResult.iPackedBrepIndex);
            sResult.m_sStatus = rConversionResult.status;

            if (rConversionResult.status != SM_SUCCESS || !rConversionResult.pBrep)
            {
                if (sResult.m_sStatus == SM_SUCCESS)
                    sResult.m_sStatus = SM_ERR;

                delete rConversionResult.pBrep;
                rConversionResult.pBrep = nullptr;
                if (IsFatalBrepImportStatus(sResult.m_sStatus))
                {
                    DeleteBrepImportResults(conversionResults);
                    DeleteBreps(importedBreps);
                    return sResult.m_sStatus;
                }

                sResult.m_sMessage = MakeBrepImportMessage(
                    prim.GetPath(),
                    sResult.m_iPackedBrepIndex,
                    sResult.m_sStatus,
                    rConversionResult.bRemainingMembersSkipped
                        ? "Malformed packed-member boundary; remaining members skipped"
                        : "Packed Brep conversion failed");

                if (!bAllowPartial)
                {
                    DeleteBrepImportResults(conversionResults);
                    DeleteBreps(importedBreps);
                    return sResult.m_sStatus;
                }

                importResults.push_back(sResult);
                continue;
            }

            SmBrep* pBrep = rConversionResult.pBrep;
            rConversionResult.pBrep = nullptr;

            // Place each successful member at its authored world location.
            // Preserve the packed index from conversion in any diagnostics;
            // the compacted success-vector index is not member provenance.
            stat = ApplyWorldXformToBrep(
                worldXform, prim.GetPath(), sResult.m_iPackedBrepIndex, pBrep);
            if (stat != SM_SUCCESS)
            {
                delete pBrep;
                if (IsFatalBrepImportStatus(stat))
                {
                    DeleteBrepImportResults(conversionResults);
                    DeleteBreps(importedBreps);
                    return stat;
                }

                sResult.m_sStatus = stat;
                sResult.m_sMessage = MakeBrepImportMessage(
                    prim.GetPath(),
                    sResult.m_iPackedBrepIndex,
                    stat,
                    "World transform failed");

                if (!bAllowPartial)
                {
                    DeleteBrepImportResults(conversionResults);
                    DeleteBreps(importedBreps);
                    return stat;
                }

                importResults.push_back(sResult);
                continue;
            }

            importedBreps.push_back(pBrep);
            sResult.m_pBrep = pBrep;
            sResult.m_sStatus = SM_SUCCESS;
            importResults.push_back(sResult);
        }

        DeleteBrepImportResults(conversionResults);
    }

    // Publish outputs and aliases only after the selected import policy has
    // completed. Existing caller-owned entries are never part of the report.
    rSmBreps.insert(rSmBreps.end(), importedBreps.begin(), importedBreps.end());
    if (pImportResults)
        *pImportResults = importResults;

    return SM_SUCCESS;
}

SmStatus SmApiUsdImportBreps
(
    const char               * pFileName,
    std::vector<SmBrep*>     & rSmBreps,
    SmBoolean                  bHealerIsEnabled
)
{
    return ImportBreps(pFileName, rSmBreps, bHealerIsEnabled, FALSE, nullptr);
}

SmStatus SmApiUsdImportBreps
(
    const char                                  * pFileName,
    std::vector<SmBrep*>                        & rSmBreps,
    SmBoolean                                     bHealerIsEnabled,
    SmBoolean                                     bAllowPartial,
    std::vector<SmApiUsdBrepImportResult>       * pImportResults
)
{
    return ImportBreps(pFileName, rSmBreps, bHealerIsEnabled, bAllowPartial, pImportResults);
}

SmStatus SmApiUsdImportBrep
(
    const char               * pFileName,
    const char               * pPrimPath,
    SmBrep                  *& rpSmBrep,
    SmBoolean                  bHealerIsEnabled
)
{
    rpSmBrep = nullptr;

    if (!pFileName || !pPrimPath)
        return SM_ERR;

    SmStatus regStat = EnsurePluginRegisteredForReading();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::Open(pFileName);
    if (!stage)
        return SM_ERR;

    SdfPath primPath(pPrimPath);
    UsdPrim prim = stage->GetPrimAtPath(primPath);
    if (!prim.IsValid() || prim.GetTypeName() != TfToken("BrepArray"))
        return SM_ERR;

    const SmContext* pContext = SmApiGetOrCreateContext();
    UsdGeomGprim gprim(prim);
    std::vector<SmBrep*> breps;
    SmStatus stat = SMU_BrepConvert::BrepMove_UsdToSMLib(*pContext, gprim, breps, bHealerIsEnabled);
    const bool hasNullBrep = std::find(breps.begin(), breps.end(), nullptr) != breps.end();
    if (stat != SM_SUCCESS || breps.empty() || hasNullBrep)
    {
        DeleteBreps(breps);
        return (stat != SM_SUCCESS) ? stat : SM_ERR;
    }

    // Place the imported geometry at the prim's scene (world) location.
    stat = ApplyWorldXformToBreps(ComputeWorldXform(prim), prim.GetPath(), breps);
    if (stat != SM_SUCCESS)
    {
        DeleteBreps(breps);
        return stat;
    }

    rpSmBrep = breps[0];

    for (size_t i = 1; i < breps.size(); ++i)
        delete breps[i];

    return SM_SUCCESS;
}

SmStatus SmApiUsdImportMeshes
(
    const char               * pFileName,
    std::vector<SmPolyBrep*> & rSmPolyBreps
)
{
    if (!pFileName)
        return SM_ERR;

    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::Open(pFileName);
    if (!stage)
        return SM_ERR;

    std::vector<UsdPrim> meshPrims;
    CollectUsdGeomMeshes(stage, meshPrims);

    if (meshPrims.empty())
        return SM_ERR;

    const SmContext* pContext = SmApiGetOrCreateContext();

    for (auto& prim : meshPrims)
    {
        UsdGeomMesh meshGeom(prim);
        SmPolyBrep* pPoly = nullptr;
        SmStatus stat = SMU_BrepConvert::CreateSmPolyBrep_FromUsdMesh(*pContext, meshGeom, pPoly);
        if (stat != SM_SUCCESS || pPoly == nullptr)
        {
            for (SmPolyBrep* q : rSmPolyBreps)
                delete q;
            rSmPolyBreps.clear();
            return (stat != SM_SUCCESS) ? stat : SM_ERR;
        }
        rSmPolyBreps.push_back(pPoly);
    }

    return SM_SUCCESS;
}

SmStatus SmApiUsdImportMesh
(
    const char    * pFileName,
    const char    * pPrimPath,
    SmPolyBrep   *& rpSmPolyBrep
)
{
    if (!pFileName || !pPrimPath)
        return SM_ERR;

    rpSmPolyBrep = nullptr;

    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::Open(pFileName);
    if (!stage)
        return SM_ERR;

    SdfPath primPath(pPrimPath);
    UsdPrim prim = stage->GetPrimAtPath(primPath);
    if (!prim.IsValid() || !prim.IsA<UsdGeomMesh>())
        return SM_ERR;

    const SmContext* pContext = SmApiGetOrCreateContext();
    UsdGeomMesh meshGeom(prim);
    return SMU_BrepConvert::CreateSmPolyBrep_FromUsdMesh(*pContext, meshGeom, rpSmPolyBrep);
}

// ---------------------------------------------------------------------------
//  SMLib -> USD
// ---------------------------------------------------------------------------

SmStatus SmApiUsdExportBreps
(
    const char               * pFileName,
    std::vector<SmBrep*>     & rSmBreps,
    SmBoolean                  bExportUVCurves,
    SmBoolean                  bBoundUnboundedFaceRanges
)
{
    if (!pFileName || rSmBreps.empty())
        return SM_ERR;

    for (const SmBrep* pBrep : rSmBreps)
        if (pBrep == nullptr)
            return SM_ERR;

    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::CreateNew(pFileName);
    if (!stage)
        return SM_ERR;

    SdfPath rootPath("/World");
    UsdGeomXform::Define(stage, rootPath);

    SdfPath brepArrayPath = rootPath.AppendChild(TfToken("Brep0"));
    UsdPrim brepArrayPrim = stage->DefinePrim(brepArrayPath, TfToken("BrepArray"));
    if (!brepArrayPrim.IsValid())
        return SM_ERR;

    SmStatus stat = SMU_BrepConvert::BrepAppend_SMLibToUsd(rSmBreps, brepArrayPrim, bExportUVCurves, bBoundUnboundedFaceRanges);
    if (stat != SM_SUCCESS)
        return stat;

    stage->Save();
    return SM_SUCCESS;
}

SmStatus SmApiUsdExportBrep
(
    const char               * pFileName,
    SmBrep                   * pSmBrep,
    SmBoolean                  bExportUVCurves,
    SmBoolean                  bBoundUnboundedFaceRanges
)
{
    if (!pSmBrep)
        return SM_ERR;

    std::vector<SmBrep*> breps = { pSmBrep };
    return SmApiUsdExportBreps(pFileName, breps, bExportUVCurves, bBoundUnboundedFaceRanges);
}

static SmStatus ExportPolyBrepsToNewUsd
(
    const char                 * pFileName,
    std::vector<SmPolyBrep*>   & rSmPolyBreps
)
{
    if (!pFileName || rSmPolyBreps.empty())
        return SM_ERR;

    for (SmPolyBrep* p : rSmPolyBreps)
    {
        if (p == nullptr)
            return SM_ERR;
    }

    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::CreateNew(pFileName);
    if (!stage)
        return SM_ERR;

    SdfPath rootPath("/World");
    UsdGeomXform::Define(stage, rootPath);

    SdfLayerHandle layer = stage->GetRootLayer();
    SdfPrimSpecHandle sRootXformHandle = layer->GetPrimAtPath(rootPath);
    if (!sRootXformHandle)
        return SM_ERR;

    for (size_t i = 0; i < rSmPolyBreps.size(); ++i)
    {
        std::string meshName = "Mesh" + std::to_string(i);
        SdfPrimSpecHandle meshSpec = SdfPrimSpec::New(sRootXformHandle, meshName, SdfSpecifierDef, "Mesh");
        SmStatus meshSpecStat = ValidateMeshPrimSpecForExport(meshSpec, meshName, sRootXformHandle);
        if (meshSpecStat != SM_SUCCESS)
            return meshSpecStat;
        SmStatus st = SMU_BrepConvert::PopulateMeshAttr(*rSmPolyBreps[i], meshSpec);
        if (st != SM_SUCCESS)
            return st;
    }

    stage->Save();
    return SM_SUCCESS;
}

SmStatus SmApiUsdExportMeshes
(
    const char                 * pFileName,
    std::vector<SmPolyBrep*>   & rSmPolyBreps
)
{
    return ExportPolyBrepsToNewUsd(pFileName, rSmPolyBreps);
}

SmStatus SmApiUsdExportMesh
(
    const char    * pFileName,
    SmPolyBrep    * pSmPolyBrep
)
{
    if (!pSmPolyBrep)
        return SM_ERR;

    std::vector<SmPolyBrep*> meshes = { pSmPolyBrep };
    return SmApiUsdExportMeshes(pFileName, meshes);
}

// ---------------------------------------------------------------------------
//  Append to existing stage
// ---------------------------------------------------------------------------

SmStatus SmApiUsdAppendBreps
(
    const char               * pFileName,
    std::vector<SmBrep*>     & rSmBreps,
    SmBoolean                  bExportUVCurves,
    SmBoolean                  bBoundUnboundedFaceRanges
)
{
    if (!pFileName || rSmBreps.empty())
        return SM_ERR;

    for (const SmBrep* pBrep : rSmBreps)
        if (pBrep == nullptr)
            return SM_ERR;

    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::Open(pFileName);
    if (!stage)
        return SM_ERR;

    // Count existing BrepArray prims to generate a unique name
    std::vector<UsdPrim> existing;
    CollectBrepArrayPrims(stage->GetPseudoRoot(), existing);

    SdfPath rootPath("/World");
    if (!stage->GetPrimAtPath(rootPath).IsValid())
        UsdGeomXform::Define(stage, rootPath);

    std::string brepName = "Brep" + std::to_string(existing.size());
    SdfPath brepArrayPath = rootPath.AppendChild(TfToken(brepName));
    UsdPrim brepArrayPrim = stage->DefinePrim(brepArrayPath, TfToken("BrepArray"));
    if (!brepArrayPrim.IsValid())
        return SM_ERR;

    SmStatus stat = SMU_BrepConvert::BrepAppend_SMLibToUsd(rSmBreps, brepArrayPrim, bExportUVCurves, bBoundUnboundedFaceRanges);
    if (stat != SM_SUCCESS)
        return stat;

    stage->Save();
    return SM_SUCCESS;
}

SmStatus SmApiUsdAppendMeshes
(
    const char                 * pFileName,
    std::vector<SmPolyBrep*>   & rSmPolyBreps
)
{
    if (!pFileName || rSmPolyBreps.empty())
        return SM_ERR;

    for (SmPolyBrep* p : rSmPolyBreps)
    {
        if (p == nullptr)
            return SM_ERR;
    }

    SmStatus regStat = EnsurePluginRegistered();
    if (regStat != SM_SUCCESS)
        return regStat;

    UsdStageRefPtr stage = UsdStage::Open(pFileName);
    if (!stage)
        return SM_ERR;

    SdfPath rootPath("/World");
    if (!stage->GetPrimAtPath(rootPath).IsValid())
        UsdGeomXform::Define(stage, rootPath);

    SdfLayerHandle layer = stage->GetRootLayer();
    SdfPrimSpecHandle sRootXformHandle = layer->GetPrimAtPath(rootPath);
    if (!sRootXformHandle)
        return SM_ERR;

    std::vector<UsdPrim> existingMeshes;
    CollectUsdGeomMeshes(stage, existingMeshes);
    const size_t base = existingMeshes.size();

    for (size_t i = 0; i < rSmPolyBreps.size(); ++i)
    {
        std::string meshName = "Mesh" + std::to_string(base + i);
        SdfPrimSpecHandle meshSpec = SdfPrimSpec::New(sRootXformHandle, meshName, SdfSpecifierDef, "Mesh");
        SmStatus meshSpecStat = ValidateMeshPrimSpecForExport(meshSpec, meshName, sRootXformHandle);
        if (meshSpecStat != SM_SUCCESS)
            return meshSpecStat;
        SmStatus st = SMU_BrepConvert::PopulateMeshAttr(*rSmPolyBreps[i], meshSpec);
        if (st != SM_SUCCESS)
            return st;
    }

    stage->Save();
    return SM_SUCCESS;
}
