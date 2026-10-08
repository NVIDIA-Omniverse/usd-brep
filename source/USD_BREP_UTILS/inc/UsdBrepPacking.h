// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepPacking.h
 * PURPOSE: Pack and unpack operations on UsdBrepArrayData.
 *
 * CONTAINS:
 *  namespace UsdBrep {
 *      AppendBrepToArray()  - append a single Brep (via UsdBrepView) into a UsdBrepArrayData
 *  }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_PACKING_H_
#define _USD_BREP_PACKING_H_

#include "UsdBrepArrayData.h"
#include "UsdBrepIterator.h"
#include "UsdBrepUtilsConfig.h"

namespace UsdBrep
{
using UsdBrepData::UsdBrepArrayData;

// Append a single Brep into rDest.
// Global cross-reference indices are remapped to follow existing data in rDest.
// Returns true when the Brep was appended; false if the source view is empty
// (no regions, faces, wire edges, or vertices) and nothing was appended.
USD_BREP_EXPORT bool AppendBrepToArray(const UsdBrepView& brep, UsdBrepArrayData& rDest);

} // end namespace UsdBrep

#endif // _USD_BREP_PACKING_H_
