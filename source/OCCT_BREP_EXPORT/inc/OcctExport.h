// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_EXPORT_OCCT_EXPORT_H
#define OCCT_BREP_EXPORT_OCCT_EXPORT_H

#include "UsdBrepArrayData.h"
#include "UsdBrepToOcctModel.h"

#include <string>

namespace occt
{

// Convert a UsdBrepArrayData into OCCT ASCII `.brep` text.
// Returns false and sets rError on failure (see BuildOcctModel for scope).
bool ExportBrepArrayToText(const UsdBrepData::UsdBrepArrayData& rArrays, std::string& rText, std::string& rError);

// Convert a UsdBrepArrayData and write the resulting `.brep` text to a file.
bool ExportBrepArrayToFile(const UsdBrepData::UsdBrepArrayData& rArrays, const std::string& rPath, std::string& rError);

} // namespace occt

#endif // OCCT_BREP_EXPORT_OCCT_EXPORT_H
