// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_ASCII_PARSER_H
#define OCCT_BREP_IMPORT_OCCT_ASCII_PARSER_H

#include "OcctFileRecords.h"

#include <string>

namespace occt
{

// Parse the full contents of a `.brep` (or DBRep_DrawableShape) file held in rText.
// Returns true on success. On failure, returns false and fills rError.
bool ReadOcct(const std::string& rText, SOcctModel& rModel, std::string& rError);

// Human-readable names for logging.
const char* CurveTypeName(ECurveType eType);
const char* SurfaceTypeName(ESurfaceType eType);
const char* ShapeTypeName(EShapeType eType);

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_ASCII_PARSER_H
