// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// CLI: read BrepArray prims from USD and write each as an OCCT ASCII `.brep`.
// Mirrors occt_to_usd.cpp from OCCT_BREP_IMPORT.
//
// Forms:
//   usd_to_occt <input.usd> <output.brep>        single input -> output (legacy)
//   usd_to_occt -o <dir> <input.usd> [more...]   batch: outputs named after each input stem
//   usd_to_occt --list <input.usd> [more...]     list BrepArray prims, do not export
//
// When a stage holds multiple BrepArray prims, an index suffix is appended to the output
// name (e.g. out_0.brep, out_1.brep). Output is OCCT ASCII `.brep` (binary is not emitted;
// ASCII is fully consumable by OpenCASCADE).

#include "OcctExport.h"
#include "UsdBrepStageUtils.h"
#include "UsdBrepUtilities.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

struct Counts
{
    int iWrote = 0;
    int iFailed = 0;
    int iPrims = 0;
};

void PrintUsage(const char* zProg)
{
    std::fprintf(
        stderr,
        "usage:\n"
        "  %s <input.usd> <output.brep>        single input -> output\n"
        "  %s -o <dir> <input.usd> [more...]   batch: outputs named after each input\n"
        "  %s --list <input.usd> [more...]     list BrepArray prims, do not export\n"
        "\n"
        "options:\n"
        "  -o, --outdir <dir>   write <stem>.brep into <dir> for each input\n"
        "      --list           enumerate BrepArray prims per input and exit\n"
        "  -q, --quiet          suppress per-prim success lines\n"
        "  -h, --help           show this help\n"
        "\n"
        "Notes: a stage with multiple BrepArray prims yields out_0.brep, out_1.brep, ...\n"
        "       Output is OCCT ASCII .brep (binary .brep is not emitted).\n",
        zProg,
        zProg,
        zProg
    );
}

std::string Stem(const std::string& sPath)
{
    const size_t uiSlash = sPath.find_last_of("/\\");
    const std::string sBase = (uiSlash == std::string::npos) ? sPath : sPath.substr(uiSlash + 1);
    const size_t uiDot = sBase.find_last_of('.');
    return (uiDot == std::string::npos) ? sBase : sBase.substr(0, uiDot);
}

// Load all BrepArray prims from sInput and write them to .brep files based on sOutBase
// (a path ending in .brep). Multiple prims get an index suffix. Updates rCounts.
// Returns false only when the input could not be opened or held no BrepArray prims.
bool ExportInput(const std::string& sInput, const std::string& sOutBase, bool bQuiet, Counts& rCounts)
{
    std::vector<std::pair<PXR_NS::SdfPath, std::unique_ptr<UsdBrepData::UsdBrepArrayData>>> vArrays;
    if (!UsdBrep::LoadAllBrepArrays(sInput, vArrays))
    {
        std::fprintf(stderr, "ERROR: could not open USD file: %s\n", sInput.c_str());
        return false;
    }
    if (vArrays.empty())
    {
        std::fprintf(stderr, "ERROR: no BrepArray prims found in %s\n", sInput.c_str());
        return false;
    }

    const bool bMulti = vArrays.size() > 1;
    for (size_t ii = 0; ii < vArrays.size(); ++ii)
    {
        rCounts.iPrims++;
        std::string sPath = sOutBase;
        if (bMulti)
        {
            const std::string sSuffix = "_" + std::to_string(ii) + ".brep";
            const size_t uiDot = sOutBase.rfind(".brep");
            sPath = (uiDot == std::string::npos) ? (sOutBase + sSuffix) : (sOutBase.substr(0, uiDot) + sSuffix);
        }

        std::string sError;
        if (!occt::ExportBrepArrayToFile(*vArrays[ii].second, sPath, sError))
        {
            std::fprintf(stderr, "ERROR (%s %s): %s\n", sInput.c_str(), vArrays[ii].first.GetText(), sError.c_str());
            rCounts.iFailed++;
            continue;
        }
        rCounts.iWrote++;
        if (!bQuiet)
        {
            std::printf("wrote %s  (%s)\n", sPath.c_str(), vArrays[ii].first.GetText());
        }
    }
    return true;
}

int ListInput(const std::string& sInput)
{
    std::vector<std::pair<PXR_NS::SdfPath, std::unique_ptr<UsdBrepData::UsdBrepArrayData>>> vArrays;
    if (!UsdBrep::LoadAllBrepArrays(sInput, vArrays))
    {
        std::fprintf(stderr, "ERROR: could not open USD file: %s\n", sInput.c_str());
        return 1;
    }
    std::printf("%s: %zu BrepArray prim(s)\n", sInput.c_str(), vArrays.size());
    for (const auto& rEntry : vArrays)
    {
        std::printf("  %s\n", rEntry.first.GetText());
    }
    return vArrays.empty() ? 1 : 0;
}

} // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> vInputs;
    std::string sOutDir;
    std::string sSingleOutput;
    bool bList = false;
    bool bQuiet = false;

    for (int ii = 1; ii < argc; ++ii)
    {
        const std::string sArg = argv[ii];
        if (sArg == "-h" || sArg == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else if (sArg == "--list")
        {
            bList = true;
        }
        else if (sArg == "-q" || sArg == "--quiet")
        {
            bQuiet = true;
        }
        else if (sArg == "-o" || sArg == "--outdir")
        {
            if (ii + 1 >= argc)
            {
                std::fprintf(stderr, "ERROR: %s requires a directory argument\n", sArg.c_str());
                return 2;
            }
            sOutDir = argv[++ii];
        }
        else if (!sArg.empty() && sArg[0] == '-')
        {
            std::fprintf(stderr, "ERROR: unknown option: %s\n", sArg.c_str());
            PrintUsage(argv[0]);
            return 2;
        }
        else
        {
            vInputs.push_back(sArg);
        }
    }

    if (vInputs.empty())
    {
        PrintUsage(argv[0]);
        return 2;
    }

    // Register the omniSolid schema plugin before any stage is opened so BrepArray schema data
    // resolves consistently in --list and export modes.
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

    if (bList)
    {
        int iReturn = 0;
        for (const std::string& sInput : vInputs)
        {
            iReturn |= ListInput(sInput);
        }
        return iReturn ? 1 : 0;
    }

    // Determine output strategy.
    // Legacy form: exactly two positional args, no -o, second ends in .brep -> <in> <out.brep>.
    const bool bLegacy = sOutDir.empty() && vInputs.size() == 2 && vInputs[1].size() >= 5 && vInputs[1].rfind(".brep") == vInputs[1].size() - 5;
    if (bLegacy)
    {
        sSingleOutput = vInputs[1];
        vInputs.resize(1);
    }
    else if (sOutDir.empty())
    {
        std::fprintf(stderr, "ERROR: multiple inputs require -o <dir> (or use <input.usd> <output.brep>)\n");
        PrintUsage(argv[0]);
        return 2;
    }

    Counts sCounts;
    bool bAnyInputError = false;
    for (const std::string& sInput : vInputs)
    {
        std::string sOutBase;
        if (bLegacy)
        {
            sOutBase = sSingleOutput;
        }
        else
        {
            sOutBase = sOutDir;
            if (!sOutBase.empty() && sOutBase.back() != '/' && sOutBase.back() != '\\')
            {
                sOutBase += '/';
            }
            sOutBase += Stem(sInput) + ".brep";
        }
        if (!ExportInput(sInput, sOutBase, bQuiet, sCounts))
        {
            bAnyInputError = true;
        }
    }

    std::printf("summary: %zu input(s), %d BrepArray prim(s): %d wrote, %d failed\n", vInputs.size(), sCounts.iPrims, sCounts.iWrote, sCounts.iFailed);

    if (bAnyInputError && sCounts.iWrote == 0)
    {
        return 2;
    }
    return (sCounts.iFailed > 0 || bAnyInputError) ? 1 : 0;
}
