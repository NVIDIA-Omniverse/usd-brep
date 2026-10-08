// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_BINARY_PARSER_H
#define OCCT_BREP_IMPORT_OCCT_BINARY_PARSER_H

#include "OcctFileRecords.h"

#include <string>

namespace occt
{

// True when rData looks like an OCCT binary `.brep` (BinTools::Write) file, as opposed to the ASCII
// topology format consumed by ReadOcct. Detection inspects the leading version banner.
bool IsBinaryOcct(const std::string& rData);

// Parse the raw bytes of an OCCT binary `.brep` file (BinTools_ShapeSet format) held in rData into
// the same neutral SOcctModel produced by the ASCII ReadOcct. Returns true on success; on failure
// returns false and fills rError.
//
// Note: BinTools writes integers/reals in the host byte order of the writing machine (no endianness
// marker is stored). This reader assumes little-endian, matching x86-64 producers.
bool ReadOcctBinary(const std::string& rData, SOcctModel& rModel, std::string& rError);

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_BINARY_PARSER_H
