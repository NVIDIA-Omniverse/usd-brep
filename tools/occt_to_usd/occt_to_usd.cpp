// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "OcctFileConverter.h"
#include "UsdBrepUtilities.h"

#include <cstdio>
#include <string>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::fprintf(stderr, "usage: %s <input.brep> <output.usda|usdc>\n", argv[0]);
        return 2;
    }

    // Register the omniSolid schema plugin before writing BrepArray prims (required on macOS
    // release builds where NDEBUG is not yet defined, and for consistent behavior everywhere).
    if (!UsdBrepData::IsOmniSolidResourcesPluginRegistered())
    {
        if (!UsdBrepData::RegisterOmniSolidResourcesPlugin())
        {
            std::fprintf(
                stderr,
                "ERROR: could not register the omniSolid schema plugin; set OMNISOLID_PLUGIN_PATH "
                "to <build>/schema/omniSolid/resources\n"
            );
            return 3;
        }
    }

    const std::string sInput = argv[1];
    const std::string sOutput = argv[2];

    std::string sError;
    if (!occt::ConvertOcctFileToUsd(sInput, sOutput, sError))
    {
        std::fprintf(stderr, "ERROR: %s\n", sError.c_str());
        return 1;
    }

    std::printf("wrote %s\n", sOutput.c_str());
    return 0;
}
