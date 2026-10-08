// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_EXPORT_OCCT_ASCII_WRITER_H
#define OCCT_BREP_EXPORT_OCCT_ASCII_WRITER_H

#include "OcctFileRecords.h"

#include <string>

namespace occt
{

// Serialize an in-memory SOcctModel into OCCT ASCII `.brep` text.
//
// This is the exact inverse of OcctAsciiParser.cpp / ReadOcct: it emits the
// "CASCADE Topology V3" banner, then the Locations / Curve2ds / Curves /
// Polygon3D / PolygonOnTriangulations / Surfaces / Triangulations sections, then
// the TShapes graph (in storage order, child-first) and the trailing root ref.
//
// Sub-shape file tokens are derived from each reference's iStorageIndex using the
// same `token = N - storageIndex + 1` rule the reader inverts, so callers only
// need to populate iStorageIndex.
//
// Real numbers are written at full double precision so a subsequent ReadOcct
// reproduces identical values (the output is not intended to be byte-identical to
// OCCT's own writer, only structurally/semantically equivalent on re-read).
std::string WriteOcct(const SOcctModel& rModel);

} // namespace occt

#endif // OCCT_BREP_EXPORT_OCCT_ASCII_WRITER_H
