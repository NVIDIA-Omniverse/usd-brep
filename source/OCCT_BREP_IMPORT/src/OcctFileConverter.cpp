// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OcctFileConverter.h"

#include "BrepStagingState.h"
#include "BrepStatus.h"
#include "BrepUsdWriter.h"
#include "OcctAsciiParser.h"
#include "OcctBinaryParser.h"
#include "OcctFileSource.h"
#include "OcctStagingDriver.h"

#include <fstream>
#include <ios>
#include <string>

namespace occt
{
namespace
{

// Resolve a parsed model into a BrepArray prim under rRootPath of rStage. Shared by the ASCII and
// binary entry points.
bool ConvertModelToUsd(const SOcctModel& rModel, const pxr::UsdStageRefPtr& rStage, const pxr::SdfPath& rRootPath, std::string& rError)
{
    OcctFileSource sSource(rModel);
    if (!sSource.Build(rError))
    {
        rError = "topology resolve failed: " + rError;
        return false;
    }

    BrepStagingState sState;
    OcctStagingDriver sDriver(sSource, sState);
    if (!sDriver.Run(rError))
    {
        rError = "staging failed: " + rError;
        return false;
    }

    std::string sWriteError;
    const BrepStatus sStatus = StagedBrepUsdWriter(sState).WriteBrepArrayPrim(rStage, rRootPath, "OCCT", &sWriteError, nullptr);
    if (!IsBrepStatusSuccess(sStatus))
    {
        rError = "USD write failed: " + sWriteError;
        return false;
    }

    return true;
}

} // namespace

bool ConvertOcctTextToUsd(const std::string& rOcctText, const pxr::UsdStageRefPtr& rStage, const pxr::SdfPath& rRootPath, std::string& rError)
{
    if (!rStage)
    {
        rError = "null USD stage";
        return false;
    }

    SOcctModel sModel;
    if (!ReadOcct(rOcctText, sModel, rError))
    {
        rError = "parse failed: " + rError;
        return false;
    }

    return ConvertModelToUsd(sModel, rStage, rRootPath, rError);
}

bool ConvertOcctDataToUsd(const std::string& rOcctData, const pxr::UsdStageRefPtr& rStage, const pxr::SdfPath& rRootPath, std::string& rError)
{
    if (!rStage)
    {
        rError = "null USD stage";
        return false;
    }

    // Binary `.brep` (BinTools) and ASCII `.brep` (BRepTools) share the `.brep` extension; pick the
    // parser from the leading version banner.
    SOcctModel sModel;
    if (IsBinaryOcct(rOcctData))
    {
        if (!ReadOcctBinary(rOcctData, sModel, rError))
        {
            rError = "binary parse failed: " + rError;
            return false;
        }
    }
    else if (!ReadOcct(rOcctData, sModel, rError))
    {
        rError = "parse failed: " + rError;
        return false;
    }

    return ConvertModelToUsd(sModel, rStage, rRootPath, rError);
}

bool ConvertOcctFileToUsd(const std::string& rInputOcctPath, const std::string& rOutputUsdPath, std::string& rError)
{
    std::ifstream sIn(rInputOcctPath, std::ios::binary);
    if (!sIn)
    {
        rError = "cannot open input file: " + rInputOcctPath;
        return false;
    }
    // Cap the buffered input so a pathological/corrupt file cannot OOM the process. 256 MiB is far
    // above any realistic single .brep part.
    constexpr std::streamoff kMaxBrepBytes = 256ll * 1024ll * 1024ll;
    sIn.seekg(0, std::ios::end);
    const std::streamoff iSize = sIn.tellg();
    if (iSize < 0 || iSize > kMaxBrepBytes)
    {
        rError = "input file is too large or unreadable: " + rInputOcctPath;
        return false;
    }
    sIn.seekg(0, std::ios::beg);
    std::string sData(static_cast<size_t>(iSize), '\0');
    if (iSize > 0 && !sIn.read(&sData[0], static_cast<std::streamsize>(iSize)))
    {
        rError = "failed to read input file: " + rInputOcctPath;
        return false;
    }

    pxr::UsdStageRefPtr sStage = pxr::UsdStage::CreateInMemory();
    if (!sStage)
    {
        rError = "failed to create in-memory USD stage";
        return false;
    }

    if (!ConvertOcctDataToUsd(sData, sStage, pxr::SdfPath("/World"), rError))
    {
        return false;
    }

    if (!sStage->GetRootLayer()->Export(rOutputUsdPath))
    {
        rError = "failed to export USD to: " + rOutputUsdPath;
        return false;
    }

    return true;
}

} // namespace occt
