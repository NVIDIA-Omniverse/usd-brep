// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiUsdHeal.cpp
* PURPOSE: Heal BrepArray definitions in the current composition's layers.
**********************************************************************/

#include "SmApiUsd.h"

#include "UsdBrepHeaders.h"
#include "UsdBrepArrayData.h"
#include "UsdBrepWrite.h"
#include "UsdBrepUtilities.h"
#include "UsdBrepSuppressPixarWarningsPush.h"
#include <pxr/base/tf/errorMark.h>
#include <pxr/usd/ar/packageUtils.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdUtils/dependencies.h>
#include "UsdBrepSuppressPixarWarningsPop.h"
#include "SmuConvert.h"
#include "SmBrep.h"
#include "SmContext.h"
#include <tbb/task_arena.h>
#if __has_include(<tbb/parallel_pipeline.h>)
#    include <tbb/parallel_pipeline.h>
#    define HEAL_SERIAL_INPUT tbb::filter_mode::serial_in_order
#    define HEAL_PARALLEL tbb::filter_mode::parallel
#    define HEAL_SERIAL_OUTPUT tbb::filter_mode::serial_out_of_order
#else
#    include <tbb/pipeline.h>
#    define HEAL_SERIAL_INPUT tbb::filter::serial_in_order
#    define HEAL_PARALLEL tbb::filter::parallel
#    define HEAL_SERIAL_OUTPUT tbb::filter::serial_out_of_order
#endif
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
namespace
{
// A scene that cannot be healed faithfully, as opposed to a failure while healing.
struct UnsupportedScene : std::runtime_error
{
    using std::runtime_error::runtime_error;
};
struct WorkItem
{
    size_t index = 0; // into the results
    UsdPrim prim;
    UsdStageRefPtr output;
    std::unique_ptr<UsdBrepData::UsdBrepArrayData> arrays;
    std::string error;
    size_t bodies = 0;
};
// BrepArray geometry and topology span several namespaces: the brep: prefix carries
// surfaces, curves and points, while the topology tables are authored unprefixed. Bare
// `extent` is deliberately excluded - it is a UsdGeomBoundable bounding box hint that
// pipelines legitimately override, not geometry rebuilt from the kernel.
bool IsBrepGeometryProperty(const std::string& name)
{
    static const std::string prefixes[] = { "brep:", "face:", "faceuse:", "edge:", "edgeuse:", "loop:", "shell:", "region:", "vertex:", "wireEdge:" };
    for (const auto& prefix : prefixes)
    {
        if (name.rfind(prefix, 0) == 0)
        {
            return true;
        }
    }
    return false;
}
struct OwnedBreps
{
    std::vector<SmBrep*> values;
    ~OwnedBreps()
    {
        for (auto brep : values)
        {
            delete brep;
        }
    }
};
// material:binding is rebuilt with its targets only, so any other authored metadata on it
// (bindMaterialAs, customData, ...) would be dropped. `custom` and `variability` do not affect which
// material it binds; it is rebuilt with UsdShade's `custom = false`, `uniform`.
bool HasUnpreservedBindingMetadata(const UsdRelationship& binding)
{
    for (const auto& entry : binding.GetAllAuthoredMetadata())
    {
        if (entry.first != SdfFieldKeys->Custom && entry.first != SdfFieldKeys->Variability)
        {
            return true;
        }
    }
    return false;
}

// The writer rebuilds subsets, including their names and face indices. Opinions
// at a consuming site cannot keep addressing the original subsets. Check the
// authoring path as well as the layer: internal references can contribute
// overrides from another site in the same layer. Include instance proxies so
// an instance does not hide material overrides in its referenced model.
void ValidateMaterialSubsetSites(const UsdPrim& prim, const SdfLayerHandle& owner, const SdfPath& ownerPath)
{
    for (auto child : prim.GetFilteredChildren(UsdTraverseInstanceProxies(UsdPrimAllPrimsPredicate)))
    {
        if (!child.IsA<UsdGeomSubset>())
        {
            continue;
        }
        for (auto spec : child.GetPrimStack())
        {
            if (spec->GetLayer() != owner || spec->GetPath().GetParentPath() != ownerPath)
            {
                throw UnsupportedScene("material subset composed outside the BRep definition is unsupported: " +
                                       child.GetPath().GetString() + " (" + spec->GetLayer()->GetIdentifier() + ")");
            }
        }
    }
}

void Heal(WorkItem& item)
{
    try
    {
        // Subset indices are global to the whole BrepArray. One naming an object owned by
        // another body is remapped to that body during conversion, but one outside every
        // body matches nothing and would be dropped without trace.
        VtUIntArray faceLoopCount;
        item.prim.GetAttribute(TfToken("face:loopCount")).Get(&faceLoopCount);
        const size_t faceCount = faceLoopCount.size();
        const size_t brepCount = UsdBrepData::GetUsdBrepArray_BrepCount(item.prim);
        if (auto binding = item.prim.GetRelationship(TfToken("material:binding")); binding && HasUnpreservedBindingMetadata(binding))
        {
            throw std::runtime_error("material:binding carries metadata the healer cannot preserve: " + binding.GetPath().GetString());
        }
        // Healing changes face indices. Only material subsets have a kernel mapping.
        for (auto child : item.prim.GetChildren())
        {
            UsdGeomSubset subset(child);
            if (!subset)
            {
                continue;
            }
            TfToken family;
            subset.GetFamilyNameAttr().Get(&family);
            if (family != TfToken("materialBind") && !family.IsEmpty())
            {
                throw std::runtime_error("cannot remap non-material GeomSubset " + child.GetPath().GetString());
            }
            // Subsets are rebuilt from the converted bindings, and the converter reads only
            // the unqualified material:binding. A subset carrying just a purpose-specific
            // binding would otherwise pass this check and then be removed without a
            // replacement, so require a binding the round trip can actually carry.
            if (!child.GetRelationship(TfToken("material:binding")).HasAuthoredTargets())
            {
                throw std::runtime_error("GeomSubset has no material:binding the healer can preserve: " + child.GetPath().GetString());
            }
            if (HasUnpreservedBindingMetadata(child.GetRelationship(TfToken("material:binding"))))
            {
                throw std::runtime_error("GeomSubset material:binding carries metadata the healer cannot preserve: " + child.GetPath().GetString());
            }
            for (auto relationship : child.GetRelationships())
            {
                const auto& name = relationship.GetName().GetString();
                if (name != "material:binding" && name.compare(0, 16, "material:binding") == 0 && relationship.HasAuthoredTargets())
                {
                    throw std::runtime_error("purpose-specific material binding is not preserved: " + relationship.GetPath().GetString());
                }
            }
            // The converter reads brep and face subsets only; any other elementType is
            // ignored on import and then removed with the rest of the subsets.
            TfToken elementType;
            subset.GetElementTypeAttr().Get(&elementType);
            if (elementType != TfToken("brep") && elementType != TfToken("face"))
            {
                throw std::runtime_error("GeomSubset elementType '" + elementType.GetString() + "' is not preserved: " + child.GetPath().GetString());
            }
            const size_t limit = elementType == TfToken("face") ? faceCount : brepCount;
            VtIntArray indices;
            subset.GetIndicesAttr().Get(&indices);
            for (auto index : indices)
            {
                if (index < 0 || static_cast<size_t>(index) >= limit)
                {
                    throw std::runtime_error(
                        "GeomSubset index " + std::to_string(index) + " is outside the BrepArray: " + child.GetPath().GetString()
                    );
                }
            }
        }
        SmContext context;
        OwnedBreps breps;
        auto status = SMU_BrepConvert::BrepMove_UsdToSMLib(context, UsdGeomGprim(item.prim), breps.values, TRUE);
        if (status != SM_SUCCESS || breps.values.empty() || breps.values.size() != UsdBrepData::GetUsdBrepArray_BrepCount(item.prim))
        {
            throw std::runtime_error("import/heal failed or omitted bodies, status=" + std::to_string(status));
        }
        // Require a complete result even if the importer changes its failure policy.
        for (size_t body = 0; body < breps.values.size(); ++body)
        {
            if (!breps.values[body])
            {
                throw std::runtime_error("import/heal failed for body " + std::to_string(body));
            }
        }
        item.bodies = breps.values.size();
        item.arrays = std::make_unique<UsdBrepData::UsdBrepArrayData>();
        item.arrays->m_sPrimPath = item.prim.GetPath();
        SdfPathVector paths{ item.prim.GetPath() };
        status = SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(breps.values, *item.arrays, &paths, true, false);
        if (status != SM_SUCCESS)
        {
            throw std::runtime_error("serialization failed, status=" + std::to_string(status));
        }
        item.arrays->m_sCADSource = "SMLib";
    }
    catch (const std::exception& error)
    {
        item.error = error.what();
    }
    catch (...)
    {
        item.error = "unexpected kernel exception";
    }
}
void Write(WorkItem& item, const UsdStageRefPtr& stage)
{
    const auto path = item.prim.GetPath();
    auto spec = SdfCreatePrimInLayer(stage->GetRootLayer(), path);
    auto prim = stage->GetPrimAtPath(path);
    if (!spec || !prim)
    {
        throw std::runtime_error("cannot author output prim");
    }
    SdfPathVector originalBinding;
    if (auto binding = prim.GetRelationship(TfToken("material:binding")))
    {
        binding.GetTargets(&originalBinding);
    }
    // The binding is restored below with SetTargets, which authors an explicit list. Reject
    // list-edit opinions it would silently flatten; Heal() has already rejected binding metadata.
    for (auto property : spec->GetProperties())
    {
        if (property->GetName() != "material:binding")
        {
            continue;
        }
        auto binding = TfDynamic_cast<SdfRelationshipSpecHandle>(property);
        if (binding && (!binding->GetTargetPathList().IsExplicit() || !binding->GetCustomData().empty()))
        {
            throw std::runtime_error("material:binding carries list edits or metadata the healer cannot preserve: " + path.GetString());
        }
    }
    // Replace geometry arrays and schemas, retaining xforms, display state and custom metadata.
    std::vector<SdfPropertySpecHandle> properties;
    for (auto property : spec->GetProperties())
    {
        if (property->GetName().compare(0, 5, "brep:") == 0 || property->GetName() == "material:binding")
        {
            properties.push_back(property);
        }
    }
    for (auto property : properties)
    {
        spec->RemoveProperty(property);
    }
    TfTokenVector schemas;
    for (auto schema : prim.GetAppliedSchemas())
    {
        if (schema.GetString().compare(0, 4, "Brep") != 0)
        {
            schemas.push_back(schema);
        }
    }
    SdfTokenListOp list;
    if (!list.SetExplicitItems(schemas))
    {
        throw std::runtime_error("Failed to prepare applied schemas");
    }
    TfErrorMark errors;
    spec->SetInfo(UsdTokens->apiSchemas, VtValue(list));
    if (!errors.IsClean())
    {
        throw std::runtime_error("Failed to restore applied schemas");
    }
    std::vector<SdfPath> subsets;
    for (auto child : prim.GetChildren())
    {
        if (child.IsA<UsdGeomSubset>())
        {
            subsets.push_back(child.GetPath());
        }
    }
    for (auto subset : subsets)
    {
        if (!stage->RemovePrim(subset))
        {
            throw std::runtime_error("Failed to remove material subset");
        }
    }
    if (!UsdBrepData::BrepWriteToUsdStage(*item.arrays, prim))
    {
        throw std::runtime_error("USD write failed");
    }
    if (!originalBinding.empty() && !prim.GetRelationship(TfToken("material:binding")).HasAuthoredTargets())
    {
        if (!prim.CreateRelationship(TfToken("material:binding"), /*custom=*/false).SetTargets(originalBinding))
        {
            throw std::runtime_error("Failed to restore material binding");
        }
    }
}
// UsdUtilsModifyAssetPaths with keepEmptyPathsInArrays. Since OpenUSD 26.08 the flag keeps only
// the entries modify maps to empty: entries already empty return early and are dropped (OpenUSD
// commit ae9e20af22, a regression of issue #3060), where 25.11 keeps every entry. Array lengths
// and index correspondence must survive, so each asset[] value that holds an empty entry is
// rewritten here afterwards, entry by entry, as 25.11 does.
void ModifyAssetPathsKeepingEmpty(const SdfLayerHandle& layer, const UsdUtilsModifyAssetPathFn& modify)
{
    // Each asset[] value with an empty entry: attribute, sample time (none for the default), entries.
    std::vector<std::tuple<SdfPath, std::optional<double>, VtArray<SdfAssetPath>>> withEmpty;
    const auto record = [&withEmpty](const SdfPath& path, std::optional<double> time, const VtValue& value)
    {
        const auto isEmpty = [](const SdfAssetPath& entry)
        {
            return entry.GetAssetPath().empty();
        };
        if (value.IsHolding<VtArray<SdfAssetPath>>())
        {
            const auto& paths = value.UncheckedGet<VtArray<SdfAssetPath>>();
            if (std::any_of(paths.cbegin(), paths.cend(), isEmpty))
            {
                withEmpty.emplace_back(path, time, paths);
            }
        }
    };
    layer->Traverse(
        SdfPath::AbsoluteRootPath(),
        [&](const SdfPath& path)
        {
            record(path, std::nullopt, layer->GetField(path, SdfFieldKeys->Default));
            for (const double time : layer->ListTimeSamplesForPath(path))
            {
                VtValue sample;
                if (layer->QueryTimeSample(path, time, &sample))
                {
                    record(path, time, sample);
                }
            }
        }
    );
    UsdUtilsModifyAssetPaths(layer, modify, true);
    // SetField and SetTimeSample report failures only as Tf errors.
    TfErrorMark errors;
    for (auto& [path, time, paths] : withEmpty)
    {
        for (auto& entry : paths)
        {
            entry = SdfAssetPath(modify(entry.GetAssetPath()));
        }
        if (time)
        {
            layer->SetTimeSample(path, *time, VtValue(paths));
        }
        else
        {
            layer->SetField(path, SdfFieldKeys->Default, VtValue(paths));
        }
    }
    if (!errors.IsClean())
    {
        throw std::runtime_error("cannot restore asset arrays with empty entries");
    }
}
// Heal the composed scene; throws on any failure that leaves nothing to write.
void HealFile(const std::string& inputPath, const std::string& outputPath, int workers,
              std::vector<SmApiUsdBrepArrayHealResult>& rResults)
{
    if (std::filesystem::exists(outputPath))
    {
        throw UnsupportedScene("output already exists: " + outputPath);
    }
    if (!UsdBrepData::RegisterOmniSolidResourcesPlugin())
    {
        throw std::runtime_error("cannot register omniSolid schema");
    }
    if (!UsdBrepData::AreOmniSolidSchemasKnown())
    {
        throw std::runtime_error("omniSolid schema was registered after a USD stage was opened");
    }
    auto source = UsdStage::Open(inputPath);
    if (!source)
    {
        throw std::runtime_error("cannot open input: " + inputPath);
    }
    // Copy layers, not the composed stage: internal references continue to share
    // the authored geometry definition and never create extra prototype copies.
    struct LayerCopy
    {
        SdfLayerHandle original;
        SdfLayerRefPtr copy;
        UsdStageRefPtr input, output;
        std::string destination;
    };
    std::vector<LayerCopy> layers;
    std::map<std::string, std::string> destinations;
    const auto root = source->GetRootLayer();
    SdfLayerRefPtr rootOutput;
    const auto dependencyDir = std::filesystem::path(outputPath).parent_path() / (std::filesystem::path(outputPath).stem().string() + "_layers");
    for (auto layer : source->GetUsedLayers())
    {
        if (layer == source->GetSessionLayer())
        {
            continue;
        }
        // Package members use USD paths such as scene.usdz[geometry/part.usdc],
        // not filesystem paths. Extract the innermost member before taking its
        // basename; otherwise the trailing ']' becomes part of the extension.
        const auto realPath = layer->GetRealPath();
        std::filesystem::path filename(ArIsPackageRelativePath(realPath) ?
                                          ArSplitPackageRelativePathInner(realPath).second : realPath);
        filename = filename.filename();
        if (layer->GetFileFormat()->IsPackage())
        {
            // A referenced package root is copied as an ordinary layer too;
            // SdfLayer::Export cannot write a package through this path.
            filename.replace_extension(".usda");
        }
        const auto destination = layer == root ? outputPath :
                                                 (dependencyDir / (std::to_string(layers.size()) + "_" +
                                                                   filename.string()))
                                                     .string();
        if (layer->IsAnonymous() || std::filesystem::exists(destination))
        {
            throw UnsupportedScene("unsupported anonymous layer or existing dependency output: " + destination);
        }
        destinations[layer->GetIdentifier()] = std::filesystem::absolute(destination).string();
        auto copy = SdfLayer::CreateAnonymous();
        copy->TransferContent(layer);
        if (layer == root)
        {
            rootOutput = copy;
        }
        // Anchor assets to their original location while operating on anonymous copies.
        // URLs are already absolute; the default resolver would treat one as a relative
        // path and collapse its "//", turning https://host/x into https:/host/x.
        ModifyAssetPathsKeepingEmpty(
            copy,
            [&](const std::string& asset)
            {
                return asset.find("://") != std::string::npos ? asset : layer->ComputeAbsolutePath(asset);
            }
        );
        layers.push_back({ layer, copy, UsdStage::Open(layer), UsdStage::Open(copy), destination });
        if (!layers.back().input || !layers.back().output)
        {
            throw std::runtime_error("cannot compose layer " + layer->GetIdentifier());
        }
    }
    // Write() re-authors the healed arrays as a plain prim spec in a single layer. That
    // spec is stronger than any variant and replaces only one layer's opinion, so reject
    // the authoring sites it cannot faithfully reproduce rather than emit a file that
    // composes differently from the geometry that was healed.
    for (const auto& entry : layers)
    {
        std::string variantSite;
        entry.original->Traverse(
            SdfPath::AbsoluteRootPath(),
            [&](const SdfPath& specPath)
            {
                if (variantSite.empty() && specPath.IsPropertyPath() && specPath.ContainsPrimVariantSelection() &&
                    IsBrepGeometryProperty(specPath.GetName()))
                {
                    variantSite = specPath.GetString();
                }
            }
        );
        // Checked per layer rather than over the composition, so a Brep authored inside a
        // variant that is not currently selected is rejected too, instead of being skipped.
        if (!variantSite.empty())
        {
            throw UnsupportedScene("BRep geometry authored inside a variant is unsupported: " + variantSite);
        }
    }
    // The per-layer guard below cannot see an opinion in a layer that consumes the one
    // being healed, and a stronger `over` need not reauthor typeName to contribute one.
    // The composed input stage does see them.
    for (auto prim : source->Traverse(UsdTraverseInstanceProxies(UsdPrimAllPrimsPredicate)))
    {
        if (prim.GetTypeName() != TfToken("BrepArray"))
        {
            continue;
        }
        SdfLayerHandle owner;
        SdfPath ownerPath;
        for (auto attr : prim.GetAttributes())
        {
            if (!IsBrepGeometryProperty(attr.GetName().GetString()))
            {
                continue;
            }
            for (auto property : attr.GetPropertyStack())
            {
                if (!owner)
                {
                    owner = property->GetLayer();
                    ownerPath = property->GetPath().GetPrimPath();
                }
                else if (property->GetLayer() != owner)
                {
                    throw UnsupportedScene("BRep geometry composed across layers is unsupported: " + prim.GetPath().GetString());
                }
            }
        }
        if (owner)
        {
            ValidateMaterialSubsetSites(prim, owner, ownerPath);
        }
    }
    std::vector<WorkItem> jobs;
    for (const auto& layer : layers)
    {
        for (auto prim : layer.input->TraverseAll())
        {
            auto spec = layer.original->GetPrimAtPath(prim.GetPath());
            if (prim.GetTypeName() == TfToken("BrepArray") && spec && spec->GetTypeName() == TfToken("BrepArray"))
            {
                // A definition split across layers cannot safely be healed independently.
                for (auto attr : prim.GetAttributes())
                {
                    if (IsBrepGeometryProperty(attr.GetName().GetString()))
                    {
                        for (auto property : attr.GetPropertyStack())
                        {
                            if (property->GetLayer() != layer.original)
                            {
                                throw UnsupportedScene("BRep geometry composed across layers is unsupported: " + prim.GetPath().GetString());
                            }
                        }
                    }
                }
                ValidateMaterialSubsetSites(prim, layer.original, prim.GetPath());
                WorkItem item;
                item.index = rResults.size();
                item.prim = prim;
                item.output = layer.output;
                jobs.push_back(std::move(item));
                SmApiUsdBrepArrayHealResult& rResult = rResults.emplace_back();
                rResult.m_sPrimPath = prim.GetPath().GetString();
                rResult.m_sLayer = layer.original->GetIdentifier();
            }
        }
    }
    if (jobs.empty())
    {
        throw UnsupportedScene("no BrepArray prims to heal: " + inputPath);
    }
    size_t next = 0, failures = 0;
    tbb::task_arena arena(workers > 0 ? workers : tbb::task_arena::automatic);
    arena.execute(
        [&]
        {
            tbb::parallel_pipeline(
                static_cast<size_t>(arena.max_concurrency()),
                tbb::make_filter<void, WorkItem*>(
                    HEAL_SERIAL_INPUT,
                    [&](tbb::flow_control& control) -> WorkItem*
                    {
                        if (next == jobs.size())
                        {
                            control.stop();
                            return nullptr;
                        }
                        return new WorkItem(std::move(jobs[next++]));
                    }
                ) &
                    tbb::make_filter<WorkItem*, WorkItem*>(
                        HEAL_PARALLEL,
                        [&](WorkItem* item)
                        {
                            Heal(*item);
                            return item;
                        }
                    ) &
                    tbb::make_filter<WorkItem*, void>(
                        HEAL_SERIAL_OUTPUT,
                        [&](WorkItem* raw)
                        {
                            std::unique_ptr<WorkItem> item(raw);
                            if (item->error.empty())
                            {
                                try
                                {
                                    Write(*item, item->output);
                                }
                                catch (const std::exception& error)
                                {
                                    item->error = error.what();
                                }
                            }
                            SmApiUsdBrepArrayHealResult& rResult = rResults[item->index];
                            if (item->error.empty())
                            {
                                rResult.m_sStatus = SM_SUCCESS;
                                rResult.m_lBrepCount = item->bodies;
                            }
                            else
                            {
                                ++failures;
                                rResult.m_sMessage = item->error;
                            }
                        }
                    )
            );
        }
    );
    if (failures)
    {
        throw std::runtime_error(std::to_string(failures) + " BrepArray prim(s) failed; no output saved");
    }
    // Release the stages first: the healed copies' paths are relative to the output, which
    // an anonymous layer cannot resolve, so an open stage would recompose and warn.
    jobs.clear();
    for (auto& layer : layers)
    {
        layer.input = nullptr;
        layer.output = nullptr;
    }
    // Redirect composition dependencies to their healed copies only after all
    // processing succeeds. Other assets retain their anchored original paths.
    for (auto& layer : layers)
    {
        ModifyAssetPathsKeepingEmpty(
            layer.copy,
            [&](const std::string& asset)
            {
                auto dependency = SdfLayer::Find(asset);
                auto found = destinations.find(dependency ? dependency->GetIdentifier() : asset);
                return found == destinations.end() ? asset :
                                                     std::filesystem::path(found->second)
                                                         .lexically_relative(std::filesystem::absolute(layer.destination).parent_path())
                                                         .generic_string();
            }
        );
    }
    std::vector<std::filesystem::path> partialOutputs;
    if (layers.size() > 1)
    {
        if (!std::filesystem::create_directory(dependencyDir))
        {
            throw std::runtime_error("dependency output directory already exists: " + dependencyDir.string());
        }
        partialOutputs.push_back(dependencyDir);
    }
    try
    {
        auto reserveOutput = [&](const std::string& path)
        {
            std::FILE* output = std::fopen(path.c_str(), "wbx");
            if (!output)
            {
                throw std::runtime_error("cannot create output (already exists or inaccessible): " + path);
            }
            std::fclose(output);
            partialOutputs.push_back(path);
        };
        // Publish the root last; reserve each file before exporting or cleaning it up.
        for (const auto& layer : layers)
        {
            if (layer.original != root)
            {
                reserveOutput(layer.destination);
                if (!layer.copy->Export(layer.destination))
                {
                    throw std::runtime_error("failed to save dependency " + layer.destination);
                }
            }
        }
        reserveOutput(outputPath);
        if (!rootOutput->Export(outputPath))
        {
            throw std::runtime_error("failed to save output");
        }
    }
    catch (...)
    {
        // Only paths owned by this run are listed, including a failing export.
        for (auto path = partialOutputs.rbegin(); path != partialOutputs.rend(); ++path)
        {
            std::error_code error;
            std::filesystem::remove(*path, error);
        }
        throw;
    }
}
} // namespace

SmStatus SmApiUsdHealFile
(
    const char                                      * pInputFile,
    const char                                      * pOutputFile,
    int                                               iThreadCount,
    std::vector<SmApiUsdBrepArrayHealResult>        & rResults
)
{
    rResults.clear();
    if (!pInputFile || !pOutputFile)
        return SM_ERR_NULL_POINTER;
    auto fail = [&](SmStatus status, const char* pMessage)
    {
        // Keep the BrepArrays that were healed or failed; report anything else against the file.
        rResults.erase(std::remove_if(rResults.begin(), rResults.end(),
                                      [](const auto& r) { return r.m_sStatus != SM_SUCCESS && r.m_sMessage.empty(); }),
                       rResults.end());
        if (std::none_of(rResults.begin(), rResults.end(), [](const auto& r) { return r.m_sStatus != SM_SUCCESS; }))
            rResults.emplace_back().m_sMessage = pMessage;
        return status;
    };
    try
    {
        HealFile(pInputFile, pOutputFile, iThreadCount, rResults);
        return SM_SUCCESS;
    }
    catch (const UnsupportedScene& error)
    {
        return fail(SM_ERR_INVALID_INPUT, error.what());
    }
    catch (const std::exception& error)
    {
        return fail(SM_ERR, error.what());
    }
}
