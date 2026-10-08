// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsd.h
* PURPOSE: High-level API for importing/exporting USD BRep data.
*
*   Provides file-level round-trip between USD (.usda/.usdc/.usd)
*   and SMLib SmBrep / SmPolyBrep objects.  Internally delegates to
*   BREP_SM_USD and BREP_USD_DATA for BRep conversions.
**********************************************************************/

#ifndef _SM_API_USD_H_
#define _SM_API_USD_H_

#include "SmApiUsdConfig.h"
#include <SmApiTessellationParams.h>
#include <SmMessages.h>
#include <SmTypes.h>
#include <SmTArray.h>
#include <cstdint>
#include <string>
#include <vector>

class SmBrep;
class SmPolyBrep;
class SmContext;

/// Outcome of one attempted packed-BRep import.
///
/// Results are emitted in USD traversal and packed-member order. A negative
/// packed index identifies a failure involving the BrepArray prim as a whole,
/// before a particular packed member could be identified.
struct SmApiUsdBrepImportResult
{
    std::string     m_sPrimPath;
    std::int64_t    m_iPackedBrepIndex = -1;
    SmBrep        * m_pBrep = nullptr;
    SmStatus        m_sStatus = SM_ERR;
    std::string     m_sMessage;
};

/// One result per packed Brep from SmApiUsdTessellateFile. A negative packed index
/// marks a whole-prim failure.
struct SmApiUsdMeshTessellationResult
{
    std::string     m_sPrimPath;             ///< source BrepArray prim
    std::int64_t    m_iPackedBrepIndex = -1;
    std::string     m_sMeshPath;             ///< authored UsdGeomMesh; empty when none
    SmStatus        m_sStatus = SM_ERR;      ///< SM_SUCCESS when a mesh was authored
    std::size_t     m_lFailedFaceCount = 0;  ///< nonzero for a partial mesh
    std::size_t     m_lPointCount = 0;
    std::string     m_sMessage;              ///< why the Brep failed or is partial
};

/// One result per BrepArray prim from SmApiUsdHealFile; an empty prim path reports a
/// problem with the file as a whole.
struct SmApiUsdBrepArrayHealResult
{
    std::string     m_sPrimPath;             ///< BrepArray prim, as defined in m_sLayer
    std::string     m_sLayer;                ///< identifier of the layer that defines it
    SmStatus        m_sStatus = SM_ERR;      ///< SM_SUCCESS when healed
    std::size_t     m_lBrepCount = 0;        ///< Breps healed
    std::string     m_sMessage;              ///< why it failed
};

// ---------------------------------------------------------------------------
//  Plugin management
// ---------------------------------------------------------------------------

/// Ensure the omniSolid BrepArray schema plugin is registered with USD.
///
/// Idempotent. The import/export functions below already register the plugin
/// on demand, so this is not required for a normal round-trip. It lets a host
/// register explicitly at startup (failing fast on a bad or unset
/// OMNISOLID_PLUGIN_PATH) or before reading BrepArray prims through the raw USD
/// API. The plugin directory is taken from the OMNISOLID_PLUGIN_PATH
/// environment variable.
///
/// Call it before opening any USD stage: if a stage was opened first, the
/// BrepArray imports fail.
///
/// @return SM_SUCCESS if the plugin is (already) registered, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdEnsurePluginRegistered();

// ---------------------------------------------------------------------------
//  USD -> SMLib
// ---------------------------------------------------------------------------

/// Load all BRep prims from a USD file and convert them to SmBreps.
///
/// Opens the USD stage at \p pFileName, locates every UsdGeomGprim that
/// carries a BrepArray schema, and converts each one into an SmBrep. Newly
/// imported Breps are appended to \p rSmBreps only after the entire stage
/// converts successfully. On failure, any entries already supplied by the
/// caller remain unchanged and no newly created Breps are returned.
///
/// @param pFileName        [in ]: Path to USD file (.usda, .usdc, .usd)
/// @param rSmBreps         [i/o]: Existing entries are preserved; on success,
///                                resulting SmBreps are appended and owned by
///                                the caller
/// @param bHealerIsEnabled [in ]: TRUE = run healer on each converted BRep
/// @return SM_SUCCESS only if every discovered packed BRep converts; error
///         code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdImportBreps
(
    const char               * pFileName,
    std::vector<SmBrep*>     & rSmBreps,
    SmBoolean                  bHealerIsEnabled = TRUE
);

/// Load BRep prims from a USD file using an explicit partial-import policy.
///
/// This overload shares the strict behavior of the three-argument overload
/// when \p bAllowPartial is FALSE: every discovered packed BRep must import,
/// or no newly created BReps are appended. When \p bAllowPartial is TRUE,
/// every locatable packed member is attempted, successful BReps are appended,
/// and member failures are returned through \p pImportResults. A completed
/// partial scan returns SM_SUCCESS even when individual results failed; stage-
/// level and process-level failures such as an unreadable file, exhausted
/// memory, or a fatal kernel status still return an error.
///
/// @param pFileName        [in ]: Path to USD file (.usda, .usdc, .usd)
/// @param rSmBreps         [i/o]: Existing entries are preserved; newly
///                                imported BReps are appended and owned by the
///                                caller
/// @param bHealerIsEnabled [in ]: TRUE = run healer on each converted BRep
/// @param bAllowPartial    [in ]: TRUE = retain successful members when other
///                                members fail; FALSE = all-or-nothing
/// @param pImportResults   [out]: Pure output, cleared at entry. Required when
///                                \p bAllowPartial is TRUE; optional otherwise.
///                                Each nonNULL result BRep is a non-owning alias
///                                to the same pointer appended to \p rSmBreps.
/// @return SM_SUCCESS after a completed scan according to the selected policy;
///         error code for invalid arguments, stage-level failures, or any
///         member failure in strict mode
SM_API_USD_EXPORT SmStatus SmApiUsdImportBreps
(
    const char                                  * pFileName,
    std::vector<SmBrep*>                        & rSmBreps,
    SmBoolean                                     bHealerIsEnabled,
    SmBoolean                                     bAllowPartial,
    std::vector<SmApiUsdBrepImportResult>       * pImportResults
);

/// Load a single BRep prim from a USD file by SdfPath and convert to SmBrep.
/// If the prim contains multiple packed Breps, every member must convert
/// successfully; the first is returned and the remaining members are deleted.
///
/// @param pFileName        [in ]: Path to USD file
/// @param pPrimPath        [in ]: SdfPath string of the target prim (e.g. "/World/Brep0")
/// @param rpSmBrep         [out]: Resulting SmBrep (caller takes ownership),
///                                or NULL on failure
/// @param bHealerIsEnabled [in ]: TRUE = run healer on the converted BRep
/// @return SM_SUCCESS only if every packed BRep converts; error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdImportBrep
(
    const char               * pFileName,
    const char               * pPrimPath,
    SmBrep                  *& rpSmBrep,
    SmBoolean                  bHealerIsEnabled = TRUE
);

// ---------------------------------------------------------------------------
//  UsdGeomMesh <-> SmPolyBrep
// ---------------------------------------------------------------------------

/// Load every active UsdGeomMesh on the stage (traversal order) and convert
/// each to an SmPolyBrep.
///
/// @param pFileName        [in ]: Path to USD file (.usda, .usdc, .usd)
/// @param rSmPolyBreps     [out]: Resulting SmPolyBreps (caller takes ownership)
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdImportMeshes
(
    const char                 * pFileName,
    std::vector<SmPolyBrep*>   & rSmPolyBreps
);

/// Load a single UsdGeomMesh prim by SdfPath and convert to SmPolyBrep.
///
/// @param pFileName        [in ]: Path to USD file
/// @param pPrimPath        [in ]: SdfPath string (e.g. "/World/Mesh0")
/// @param rpSmPolyBrep     [out]: Resulting SmPolyBrep (caller takes ownership)
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdImportMesh
(
    const char                 * pFileName,
    const char                 * pPrimPath,
    SmPolyBrep                *& rpSmPolyBrep
);

// ---------------------------------------------------------------------------
//  SMLib -> USD
// ---------------------------------------------------------------------------

/// Export a vector of SmBreps to a new USD file.
///
/// Creates a new USD stage, converts each SmBrep to a BrepArray prim,
/// and saves the stage to \p pFileName.
/// Export does not copy the Breps or promote NURBS surfaces to analytics.
/// Bounding unbounded ranges may attach missing UV trims to the input Breps;
/// these remain available for reuse even when bExportUVCurves is FALSE.
/// Source face domains and geometry are not shrunk. Geometry caches and the
/// destination-path attribute may be updated. Do not export a Brep concurrently
/// with another operation on it; use SmApiBrepCopy first to isolate side effects.
///
/// @param pFileName        [in ]: Output USD file path
/// @param rSmBreps         [i/o]: SmBreps to export (non-const: SdfPath attributes are added to each SmBrep)
/// @param bExportUVCurves  [in ]: TRUE = include edgeuse UV trim curves
/// @param bBoundUnboundedFaceRanges [in ]: TRUE = use trim-curve bounds for unbounded BrepArray face ranges without modifying the BRep face domains
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdExportBreps
(
    const char               * pFileName,
    std::vector<SmBrep*>     & rSmBreps,
    SmBoolean                  bExportUVCurves = TRUE,
    SmBoolean                  bBoundUnboundedFaceRanges = FALSE
);

/// Export a single SmBrep to a new USD file.
/// Retains generated trims, caches, and destination-path attributes on the input
/// with the same side effects as SmApiUsdExportBreps; does not copy the Brep.
///
/// @param pFileName        [in ]: Output USD file path
/// @param pSmBrep          [i/o]: SmBrep to export; ownership remains with the caller
/// @param bExportUVCurves  [in ]: TRUE = include edgeuse UV trim curves
/// @param bBoundUnboundedFaceRanges [in ]: TRUE = use trim-curve bounds for unbounded BrepArray face ranges without modifying the BRep face domains
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdExportBrep
(
    const char               * pFileName,
    SmBrep                   * pSmBrep,
    SmBoolean                  bExportUVCurves = TRUE,
    SmBoolean                  bBoundUnboundedFaceRanges = FALSE
);

/// Export SmPolyBreps to a new USD file as UsdGeomMesh children under /World.
///
/// @param pFileName        [in ]: Output USD file path
/// @param rSmPolyBreps     [i/o]: SmPolyBreps to export (written as Mesh0, Mesh1, ...; non-const:
///                                per-vertex index bookkeeping may be updated during mesh authoring.
///                                SdfPath attributes are not added to each \c SmPolyBrep, unlike
///                                \ref SmApiUsdExportBreps). Duplicate first if you need an untouched
///                                copy for concurrent use.
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdExportMeshes
(
    const char                 * pFileName,
    std::vector<SmPolyBrep*>   & rSmPolyBreps
);

/// @brief Export a single polygon mesh (\c SmPolyBrep) to a new USD file as \c UsdGeomMesh.
///
/// Creates a new USD stage, writes the mesh as <tt>/World/Mesh0</tt>, and saves
/// the stage to \p pFileName.
///
/// @param pFileName        [in ]: Output USD file path (.usda, .usdc, or .usd); must not be NULL
/// @param pSmPolyBrep      [i/o]: Mesh to export; must not be NULL. Caller retains ownership of \p pSmPolyBrep.
///                                The mesh may be mutated the same way as in \ref SmApiUsdExportMeshes
///                                (see that function’s \p rSmPolyBreps documentation).
/// @return SM_SUCCESS on success, or an error code (e.g. \c SM_ERR) on failure
/// @note Behavior matches \ref SmApiUsdExportMeshes with a single-element vector (always \c Mesh0).
/// @see SmApiUsdExportMeshes
SM_API_USD_EXPORT SmStatus SmApiUsdExportMesh
(
    const char                 * pFileName,
    SmPolyBrep                 * pSmPolyBrep
);

// ---------------------------------------------------------------------------
//  USD <-> SMLib  (append to existing stage)
// ---------------------------------------------------------------------------

/// Append SmBreps to an existing USD file.
///
/// Opens the stage at \p pFileName, appends BrepArray prims converted
/// from \p rSmBreps, and re-saves the stage.
/// Retains generated trims, caches, and destination-path attributes on the inputs
/// with the same side effects as SmApiUsdExportBreps; does not copy the Breps.
///
/// @param pFileName        [in ]: Existing USD file path
/// @param rSmBreps         [i/o]: SmBreps to append (non-const: SdfPath attributes are added to each SmBrep)
/// @param bExportUVCurves  [in ]: TRUE = include edgeuse UV trim curves
/// @param bBoundUnboundedFaceRanges [in ]: TRUE = use trim-curve bounds for unbounded BrepArray face ranges without modifying the BRep face domains
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdAppendBreps
(
    const char               * pFileName,
    std::vector<SmBrep*>     & rSmBreps,
    SmBoolean                  bExportUVCurves = TRUE,
    SmBoolean                  bBoundUnboundedFaceRanges = FALSE
);

/// Append SmPolyBreps as new UsdGeomMesh prims under /World.
///
/// Mesh prims are named Mesh\<N\> where N is the previous UsdGeomMesh count on the stage.
///
/// @param pFileName        [in ]: Existing USD file path
/// @param rSmPolyBreps     [i/o]: SmPolyBreps to append (non-const: per-vertex index bookkeeping may be
///                                updated during mesh authoring, same as \ref SmApiUsdExportMeshes.
///                                SdfPath attributes are not added to each \c SmPolyBrep, unlike
///                                \ref SmApiUsdAppendBreps).
/// @return SM_SUCCESS on success, error code otherwise
SM_API_USD_EXPORT SmStatus SmApiUsdAppendMeshes
(
    const char                 * pFileName,
    std::vector<SmPolyBrep*>   & rSmPolyBreps
);

// ---------------------------------------------------------------------------
//  Scene tessellation
// ---------------------------------------------------------------------------

/// Tessellate every BrepArray prim in a USD file and write a copy of the stage with a
/// UsdGeomMesh per Brep under the default prim (or /Output), named tess_\<prim\>_\<brep\>
/// (_\<n\> added if taken). Each mesh gets its Brep's world transform, visibility and purpose
/// (default time) and material. Relative asset paths are rewritten if the output is in another
/// directory. The input layer is not modified, even if the caller has it open.
///
/// Prims are tessellated in parallel, each through SmApiUsdTessellateBrepArray. Breps
/// fail independently: a failed Brep is reported and skipped, and a partial mesh is
/// kept and reported.
///
/// @param pInputFile     [in ]: USD file with BrepArray prims
/// @param pOutputFile    [in ]: USD file to write; not written when no mesh is authored
/// @param crParams       [in ]: Tessellation quality controls
/// @param bHealerIsEnabled [in ]: TRUE = run the healer while importing
/// @param iThreadCount   [in ]: Tessellation threads; 0 = automatic
/// @param rResults       [out]: One entry per Brep (or whole-prim failure), in prim order
/// @return SM_SUCCESS once the output is written, even if some Breps failed (see
///         rResults); SM_ERR_INVALID_INPUT when the file has no BrepArray prims; SM_ERR
///         when the file cannot be opened, no mesh is authored, or the output cannot be
///         written
SM_API_USD_EXPORT SmStatus SmApiUsdTessellateFile
(
    const char                                      * pInputFile,
    const char                                      * pOutputFile,
    const SmTessellationParams                      & crParams,
    SmBoolean                                         bHealerIsEnabled,
    int                                               iThreadCount,
    std::vector<SmApiUsdMeshTessellationResult>     & rResults
);

/// Heal BrepArray definitions in the layers used by the current USD composition and
/// write the scene to pOutputFile, keeping its layers, references, native instancing,
/// transforms and materials. Each BrepArray is healed in the layer that defines it;
/// other layers (including package members) are written as standalone layers to
/// `<output stem>_layers/` beside the output, with references redirected.
/// External layers used only by unselected variants are not enumerated or healed;
/// their references keep pointing at the original assets.
///
/// All or nothing: if any BrepArray fails, nothing is written. Rejected, since healing
/// could not be written back faithfully: BRep geometry authored in a variant or composed
/// from several layers, GeomSubsets other than face or Brep material bindings, and
/// material subsets with opinions outside the BRep's defining prim spec. This includes
/// overrides or additional subsets in consuming layers or internal-reference sites.
///
/// @param pInputFile     [in ]: USD file with BrepArray prims
/// @param pOutputFile    [in ]: Standalone .usd/.usda/.usdc file to write; must not exist
/// @param iThreadCount   [in ]: Healing threads; 0 = automatic
/// @param rResults       [out]: One entry per BrepArray, plus one with an empty prim path
///                              for a problem with the file as a whole
/// @return SM_SUCCESS once written; SM_ERR_INVALID_INPUT for a scene with no BrepArray or
///         one that cannot be healed faithfully, or an existing output; SM_ERR when a BrepArray fails or a file
///         cannot be read or written
SM_API_USD_EXPORT SmStatus SmApiUsdHealFile
(
    const char                                      * pInputFile,
    const char                                      * pOutputFile,
    int                                               iThreadCount,
    std::vector<SmApiUsdBrepArrayHealResult>        & rResults
);

#endif // _SM_API_USD_H_
