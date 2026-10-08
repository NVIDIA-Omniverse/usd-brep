// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OcctExport.h"

#include "OcctAsciiWriter.h"
#include "OcctFileRecords.h"
#include "UsdBrepToOcctModel.h"

#include <fstream>

namespace occt
{

bool ExportBrepArrayToText(const UsdBrepData::UsdBrepArrayData& rArrays, std::string& rText, std::string& rError)
{
    SOcctModel sModel;
    if (!BuildOcctModel(rArrays, sModel, rError))
    {
        return false;
    }
    rText = WriteOcct(sModel);
    return true;
}

bool ExportBrepArrayToFile(const UsdBrepData::UsdBrepArrayData& rArrays, const std::string& rPath, std::string& rError)
{
    std::string sText;
    if (!ExportBrepArrayToText(rArrays, sText, rError))
    {
        return false;
    }

    std::ofstream sFile(rPath, std::ios::binary);
    if (!sFile)
    {
        rError = "could not open output file: " + rPath;
        return false;
    }
    sFile.write(sText.data(), static_cast<std::streamsize>(sText.size()));
    if (!sFile)
    {
        rError = "failed while writing output file: " + rPath;
        return false;
    }
    return true;
}

} // namespace occt
