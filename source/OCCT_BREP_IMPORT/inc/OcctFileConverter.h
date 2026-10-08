// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_FILE_CONVERTER_H
#define OCCT_BREP_IMPORT_OCCT_FILE_CONVERTER_H

#include "UsdBrepHeaders.h" // pxr warning-suppression guard; keep ahead of <pxr/..> includes

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <string>

namespace occt
{

// Parse ASCII `.brep` text in rOcctText and author a BrepArray prim under rRootPath of rStage.
// Returns false (with rError) on parse/topology/staging/write failure.
bool ConvertOcctTextToUsd(const std::string& rOcctText, const pxr::UsdStageRefPtr& rStage, const pxr::SdfPath& rRootPath, std::string& rError);

// Parse `.brep` bytes in rOcctData (auto-detecting ASCII BRepTools vs binary BinTools format) and
// author a BrepArray prim under rRootPath of rStage. Returns false (with rError) on failure.
bool ConvertOcctDataToUsd(const std::string& rOcctData, const pxr::UsdStageRefPtr& rStage, const pxr::SdfPath& rRootPath, std::string& rError);

// Convenience: read rInputOcctPath, convert, and export to rOutputUsdPath (extension picks the USD
// format, e.g. .usda / .usdc). Returns false (with rError) on any failure.
bool ConvertOcctFileToUsd(const std::string& rInputOcctPath, const std::string& rOutputUsdPath, std::string& rError);

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_FILE_CONVERTER_H
