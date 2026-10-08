// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_EXPORT_USD_BREP_TO_OCCT_MODEL_H
#define OCCT_BREP_EXPORT_USD_BREP_TO_OCCT_MODEL_H

#include "OcctFileRecords.h"
#include "UsdBrepArrayData.h"

#include <string>

namespace occt
{

// Build an in-memory OCCT SOcctModel from a UsdBrepArrayData.
//
// Supports analytic and NURBS geometry, best-effort pcurves, closed solids, open shells,
// standalone faces, and mixed-component Breps. Multiple root components are gathered under a
// Compound so nothing is silently dropped.
//
// Returns false (and sets rError) when the data contains unsupported geometry or is structurally
// inconsistent.
bool BuildOcctModel(const UsdBrepData::UsdBrepArrayData& rArrays, SOcctModel& rModel, std::string& rError);

} // namespace occt

#endif // OCCT_BREP_EXPORT_USD_BREP_TO_OCCT_MODEL_H
