// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// brep_geometry_validator - standalone SMLib geometry/topology validator for USD BrepArray prims.
//
// Imports every unique BrepArray prim found in the input USD file(s) into a real SmBrep and runs
// SmBrep::AssertValid on it. This is the kernel-level "is the built solid actually sound"
// check (closing loops, manifold edges, on-surface UV trim curves, ...). It is a sibling of the
// schema-level brep_validator (the BA.xxx checks): the schema validator checks the USD data is
// well-formed, this one checks the built solid is geometrically/topologically sound.
//
// Forms:
//   brep_geometry_validator <input.usd> [more.usd ...]
//
// Options:
//   --level <0|1|2>   AssertValid test depth: 0 fastest, 2 = all tests (default 2)
//   --no-healer       skip SMLib healer during USD import (faster)
//   --progress        print import/validation progress to stderr
//   --workers <N>     parallel prim validation (0 = auto, 1 = serial, default 1)
//   -q, --quiet       suppress per-failure detail lines (keep the per-file summary)
//   -h, --help        show this help
//
// Output: one machine-parseable summary line per file
//   BREP_GEOMETRY file=<path> breps=<N> invalid=<M> checksFailed=<K>
// plus (unless --quiet) a FAIL[j] line per failed check (includes the BrepArray prim path).
//
// Exit code: 0 when every brep in every input is valid; otherwise the number of invalid
// breps (capped at 250). Hard errors (no input, open/import failure, zero breps) also yield
// a non-zero code, so callers can treat "rc != 0" as "not clean".

#include "SmuConvert.h"
#include "UsdBrepUtilities.h"

#include "SmAssertArray.h"
#include "SmBrep.h"
#include "SmContext.h"

#include "UsdBrepSuppressPixarWarningsPush.h"
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/base/arch/systemInfo.h>
#include "UsdBrepSuppressPixarWarningsPop.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <thread>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

PXR_NAMESPACE_USING_DIRECTIVE

namespace
{

// ---- per-prim timing instrumentation (enabled via --timing <file>) ----
// Records, per prim: worker thread index, and heal (import) + validate start/end
// times in microseconds relative to g_timingT0. Written out serially after the
// parallel loop, so no locking is needed on the hot path beyond thread-id assignment.
std::chrono::steady_clock::time_point g_timingT0;
std::mutex g_threadIdMutex;
std::map<std::thread::id, unsigned> g_threadIds;
unsigned g_nextThreadId = 0;

long long TimingElapsedUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now() - g_timingT0)
        .count();
}

unsigned TimingThreadIndex()
{
    const std::thread::id sTid = std::this_thread::get_id();
    std::lock_guard<std::mutex> sLock(g_threadIdMutex);
    auto sIt = g_threadIds.find(sTid);
    if (sIt != g_threadIds.end())
    {
        return sIt->second;
    }
    const unsigned iId = g_nextThreadId++;
    g_threadIds.emplace(sTid, iId);
    return iId;
}

// Incremental (crash-safe) per-prim timing writer: appended as each prim finishes,
// so a mid-run crash (e.g. the healer thread-safety gap under --workers) still leaves
// the completed prims on disk.
std::FILE* g_timingFp = nullptr;
std::mutex g_timingMutex;

void WriteTimingRecord(const SdfPath& rPrim, const struct PrimValidateResult& rResult);

constexpr size_t kProgressInterval = 100;

void PrintUsage(const char* zProg)
{
    std::fprintf(
        stderr,
        "usage:\n"
        "  %s <input.usd> [more.usd ...]\n"
        "\n"
        "Import each BrepArray prim into an SmBrep and run SmBrep::AssertValid.\n"
        "\n"
        "options:\n"
        "  --level <0|1|2>   AssertValid depth: 0 fastest, 2 = all tests (default 2)\n"
        "  --no-healer       skip SMLib healer during USD import (faster)\n"
        "  --progress        print import/validation progress to stderr\n"
        "  --discovery-stats print authored/prototype prims visited during discovery\n"
        "  --workers <N>     parallel prim validation (0 = auto, 1 = serial, default 1)\n"
        "  --timing <file>   write per-prim heal/validate timing (JSONL) for charting\n"
        "  -q, --quiet       suppress per-failure detail lines\n"
        "  -h, --help        show this help\n"
        "\n"
        "Per file prints: BREP_GEOMETRY file=<path> breps=<N> invalid=<M> checksFailed=<K>\n"
        "Exit code: 0 if all breps valid, else the number of invalid breps (capped at 250).\n",
        zProg
    );
}

SmAssertTestLevel ToTestLevel(int iLevel)
{
    switch (iLevel)
    {
        case 0: return SM_LEVEL_0;
        case 1: return SM_LEVEL_1;
        default: return SM_LEVEL_2;
    }
}

// Validate a single SmBrep. Returns true when valid; on failure increments rChecksFailed and
// prints per-check FAIL lines (unless bQuiet). pOptStdoutMutex serializes stdout in parallel runs.
bool ValidateBrep(const SmBrep* pBrep, SmAssertTestLevel eLevel, bool bQuiet, unsigned long& rChecksFailed,
                  const char* zPrimPath, std::mutex* pOptStdoutMutex = nullptr)
{
    if (pBrep == nullptr)
    {
        return false;
    }

    SmAssertArray sAList;
    if (pBrep->AssertValid(&sAList, eLevel))
    {
        return true;
    }

    rChecksFailed += sAList.GetSize();
    if (!bQuiet)
    {
        const char* zPath = (zPrimPath != nullptr && zPrimPath[0] != '\0') ? zPrimPath : "?";
        for (ULONG jj = 0; jj < sAList.GetSize(); ++jj)
        {
            const SmAssertReport* pRep = sAList[jj];
            if (pRep == nullptr)
            {
                continue;
            }
            // Report fields are TCHAR* (wchar_t on Windows UNICODE builds). Pass them as %s
            // arguments via SM_FPRINTF rather than formatting into a buffer and then handing that
            // buffer to a single-arg print macro -- on Windows UNICODE that would treat the buffer
            // as a format string, so a '%' inside any field would be a format-string bug.
            TCHAR sMsg[SM_TBLOCK_SIZE];
            pRep->FormatLogMessage(sMsg, SM_TBLOCK_SIZE);
            const std::basic_string<TCHAR> sPath = smos_ToTChar(zPath);
            std::unique_lock<std::mutex> sStdoutLock;
            if (pOptStdoutMutex != nullptr)
            {
                sStdoutLock = std::unique_lock<std::mutex>(*pOptStdoutMutex);
            }
            SM_FPRINTF(stdout, _T("  FAIL[%lu] path=%s owner=%s name=%s msg=%s\n"),
                       (unsigned long)jj,
                       sPath.c_str(),
                       pRep->m_sOwnerTypeString ? pRep->m_sOwnerTypeString : _T("?"),
                       pRep->m_pName ? pRep->m_pName : _T("?"),
                       sMsg);
        }
    }
    return false;
}

struct PrimValidateResult
{
    bool bImportFailed = false;
    bool bMissingBreps = false;
    int iBreps = 0;
    int iInvalid = 0;
    unsigned long lChecksFailed = 0;
    // timing (us relative to g_timingT0; -1 = did not run)
    unsigned iThreadId = 0;
    long long iHealStartUs = -1, iHealEndUs = -1, iValStartUs = -1, iValEndUs = -1;
};

void WriteTimingRecord(const SdfPath& rPrim, const PrimValidateResult& rResult)
{
    if (g_timingFp == nullptr)
    {
        return;
    }
    std::lock_guard<std::mutex> sLock(g_timingMutex);
    std::fprintf(g_timingFp,
                 "{\"t\":%u,\"hs\":%lld,\"he\":%lld,\"vs\":%lld,\"ve\":%lld,\"prim\":\"%s\"}\n",
                 rResult.iThreadId, rResult.iHealStartUs, rResult.iHealEndUs,
                 rResult.iValStartUs, rResult.iValEndUs, rPrim.GetText());
    std::fflush(g_timingFp);
}

PrimValidateResult ProcessPrim(const UsdGeomGprim& rGprim, const SdfPath& rStablePath,
                               SmAssertTestLevel eLevel, bool bQuiet, SmBoolean bHealerIsEnabled,
                               std::mutex* pStdoutMutex)
{
    PrimValidateResult sResult;
    sResult.iThreadId = TimingThreadIndex();

    // One SmContext per prim: deleting breps while reusing a shared context corrupts
    // kernel marks/caches (SmApiUsdTessellateBrepArray also uses one context per prim).
    SmContext sContext;
    std::vector<SmBrep*> vFromPrim;
    sResult.iHealStartUs = TimingElapsedUs();
    const SmStatus sStat =
        SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, rGprim, vFromPrim, bHealerIsEnabled);
    sResult.iHealEndUs = TimingElapsedUs();
    if (sStat != SM_SUCCESS)
    {
        sResult.bImportFailed = true;
        for (SmBrep* pBrep : vFromPrim)
        {
            delete pBrep;
        }
        return sResult;
    }
    if (vFromPrim.empty() ||
        std::any_of(vFromPrim.begin(), vFromPrim.end(), [](const SmBrep* pBrep) { return pBrep == nullptr; }))
    {
        sResult.bMissingBreps = true;
        for (SmBrep* pBrep : vFromPrim)
        {
            delete pBrep;
        }
        return sResult;
    }

    sResult.iValStartUs = TimingElapsedUs();
    for (SmBrep* pBrep : vFromPrim)
    {
        if (pBrep == nullptr)
        {
            continue;
        }

        ++sResult.iBreps;
        if (!ValidateBrep(pBrep, eLevel, bQuiet, sResult.lChecksFailed,
                          rStablePath.GetText(), pStdoutMutex))
        {
            ++sResult.iInvalid;
        }
        delete pBrep;
    }
    sResult.iValEndUs = TimingElapsedUs();

    return sResult;
}

// pOptProgressMutex serializes stderr in parallel runs; the interval gate runs before the lock so
// only the printing prim contends.
void ReportProgress(bool bProgress, size_t iProcessed, size_t iPrimTotal, int iBrepCount, int iInvalid,
                    std::mutex* pOptProgressMutex = nullptr)
{
    if (!bProgress)
    {
        return;
    }
    if (iProcessed % kProgressInterval != 0 && iProcessed != iPrimTotal)
    {
        return;
    }
    std::unique_lock<std::mutex> sProgressLock;
    if (pOptProgressMutex != nullptr)
    {
        sProgressLock = std::unique_lock<std::mutex>(*pOptProgressMutex);
    }
    std::fprintf(stderr,
                 "[BrepGeometry] Processed %zu/%zu prim(s), %d brep(s), %d invalid so far\n",
                 iProcessed, iPrimTotal, iBrepCount, iInvalid);
    std::fflush(stderr);
}

// Import + validate every BrepArray prim in one file. Returns the number of invalid breps;
// sets rbHardError on open/import problems (which also count as "not clean").
int ValidateFile(const std::string& sInput, SmAssertTestLevel eLevel, bool bQuiet, bool bProgress,
                 bool bDiscoveryStats, SmBoolean bHealerIsEnabled, bool& rbHardError, int iWorkers,
                 const std::string& sTimingFile)
{
    rbHardError = false;

    // Use the non-throwing overload: a permission/stat error on user-supplied paths should be a
    // clean hard error, not an uncaught exception that aborts the run.
    std::error_code sEc;
    if (!std::filesystem::exists(sInput, sEc))
    {
        if (sEc)
        {
            std::fprintf(stderr, "ERROR: failed to access input file %s: %s\n",
                         sInput.c_str(), sEc.message().c_str());
        }
        else
        {
            std::fprintf(stderr, "ERROR: input file not found: %s\n", sInput.c_str());
        }
        std::printf("BREP_GEOMETRY file=%s open=FAIL\n", sInput.c_str());
        rbHardError = true;
        return 0;
    }

    UsdStageRefPtr sStage = UsdStage::Open(sInput);
    if (!sStage)
    {
        std::fprintf(stderr, "ERROR: failed to open USD stage: %s\n", sInput.c_str());
        std::printf("BREP_GEOMETRY file=%s open=FAIL\n", sInput.c_str());
        rbHardError = true;
        return 0;
    }

    using OrderedGprim = std::pair<SdfPath, UsdGeomGprim>;
    std::vector<OrderedGprim> vOrderedBreps;
    std::map<SdfPath, UsdGeomGprim> sOrderedBreps;
    std::map<SdfPath, UsdPrim> sPendingPrototypes;
    std::set<SdfPath> sVisitedPrototypes;
    size_t iDiscoveryPrimCount = 0;

    auto isBrep = [](const UsdPrim& sPrim)
    {
        return sPrim.HasAttribute(TfToken("brep:regionCount")) || sPrim.GetTypeName() == "BrepArray";
    };

    // Seed a sorted work queue with active authored instances.
    for (const UsdPrim& sPrim : sStage->Traverse())
    {
        ++iDiscoveryPrimCount;
        if (!sPrim.IsValid() || !sPrim.IsActive())
        {
            continue;
        }
        if (sPrim.IsInstance())
        {
            sPendingPrototypes.emplace(sPrim.GetPath(), sPrim.GetPrototype());
        }
        if (isBrep(sPrim))
        {
            sOrderedBreps.emplace(sPrim.GetPath(), UsdGeomGprim(sPrim));
        }
    }

    // The smallest authored path reaches each prototype first, so shared
    // prototype contents are traversed and imported only once.
    while (!sPendingPrototypes.empty())
    {
        const auto sNext = sPendingPrototypes.begin();
        const SdfPath sPrototypeKey = sNext->first;
        const UsdPrim sPrototype = sNext->second;
        sPendingPrototypes.erase(sNext);
        if (!sPrototype || !sVisitedPrototypes.insert(sPrototype.GetPath()).second)
        {
            continue;
        }

        for (const UsdPrim& sPrim : UsdPrimRange(sPrototype))
        {
            ++iDiscoveryPrimCount;
            if (!sPrim.IsValid() || !sPrim.IsActive())
            {
                continue;
            }
            const SdfPath sRelativePath = sPrim.GetPath().MakeRelativePath(sPrototype.GetPath());
            const SdfPath sStablePath = sPrototypeKey.AppendPath(sRelativePath);
            if (sPrim.IsInstance())
            {
                sPendingPrototypes.emplace(sStablePath, sPrim.GetPrototype());
            }
            if (isBrep(sPrim))
            {
                sOrderedBreps.emplace(sStablePath, UsdGeomGprim(sPrim));
            }
        }
    }

    vOrderedBreps.assign(sOrderedBreps.begin(), sOrderedBreps.end());

    if (bDiscoveryStats)
    {
        std::printf("BREP_GEOMETRY_DISCOVERY file=%s visited=%zu\n", sInput.c_str(), iDiscoveryPrimCount);
    }

    if (vOrderedBreps.empty())
    {
        // No BrepArray prims at all: treat as a hard error so a silent "nothing to check" cannot
        // be mistaken for success. Keep the invalid<=breps invariant (invalid=0); the hard-error
        // state is carried by rbHardError -> the non-zero process exit code instead.
        std::fprintf(stderr, "ERROR: no BrepArray prims found in %s\n", sInput.c_str());
        std::printf("BREP_GEOMETRY file=%s breps=0 invalid=0 checksFailed=0\n", sInput.c_str());
        rbHardError = true;
        return 0;
    }

    // Opened after the empty-work early return above: g_timingFp is only ever
    // closed further down, so opening it before a possible early return would
    // leak the FILE* for every empty input in a batch.
    if (!sTimingFile.empty())
    {
        g_timingFp = std::fopen(sTimingFile.c_str(), "a");
        if (g_timingFp == nullptr)
        {
            std::fprintf(stderr, "WARNING: could not open timing file %s\n", sTimingFile.c_str());
        }
    }

    if (bProgress)
    {
        std::fprintf(stderr,
                     "[BrepGeometry] Processing %zu BrepArray prim(s) in %s (import, validate, release)...\n",
                     vOrderedBreps.size(), sInput.c_str());
        if (iWorkers != 1)
        {
            std::fprintf(stderr, "[BrepGeometry] Parallel workers: %s\n",
                         iWorkers == 0 ? "auto" : std::to_string(iWorkers).c_str());
        }
        std::fflush(stderr);
    }

    std::vector<PrimValidateResult> vResults(vOrderedBreps.size());
    std::mutex sStdoutMutex;
    const size_t iPrimTotal = vOrderedBreps.size();

    if (iWorkers != 1)
    {
        std::atomic<int> iPrimsDone{0};
        std::atomic<int> iBrepsDone{0};
        std::atomic<int> iInvalidDone{0};
        std::mutex sProgressMutex;

        const int iArenaThreads = (iWorkers == 0) ? tbb::task_arena::automatic : iWorkers;
        tbb::task_arena sArena(iArenaThreads);
        sArena.execute([&]() {
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, vOrderedBreps.size(), 1),
                [&](const tbb::blocked_range<size_t>& rRange) {
                    for (size_t iPrim = rRange.begin(); iPrim != rRange.end(); ++iPrim)
                    {
                        const OrderedGprim& rOrderedBrep = vOrderedBreps[iPrim];
                        vResults[iPrim] = ProcessPrim(rOrderedBrep.second, rOrderedBrep.first,
                                                     eLevel, bQuiet, bHealerIsEnabled, &sStdoutMutex);
                        WriteTimingRecord(rOrderedBrep.first, vResults[iPrim]);

                        if (bProgress)
                        {
                            const PrimValidateResult& rResult = vResults[iPrim];
                            const int iPrimsNow = iPrimsDone.fetch_add(1) + 1;
                            const int iBrepsNow = iBrepsDone.fetch_add(rResult.iBreps) + rResult.iBreps;
                            iInvalidDone.fetch_add(rResult.iInvalid);
                            ReportProgress(bProgress, static_cast<size_t>(iPrimsNow), iPrimTotal,
                                           iBrepsNow, iInvalidDone.load(), &sProgressMutex);
                        }
                    }
                },
                tbb::simple_partitioner{});
        });
    }
    else
    {
        // Each brep is freed before the next prim is processed so large assemblies do not retain every brep in memory.
        int iBrepCount = 0;
        int iInvalid = 0;
        for (size_t iPrim = 0; iPrim < vOrderedBreps.size(); ++iPrim)
        {
            const OrderedGprim& rOrderedBrep = vOrderedBreps[iPrim];
            vResults[iPrim] =
                ProcessPrim(rOrderedBrep.second, rOrderedBrep.first, eLevel, bQuiet, bHealerIsEnabled, nullptr);
            WriteTimingRecord(rOrderedBrep.first, vResults[iPrim]);
            iBrepCount += vResults[iPrim].iBreps;
            iInvalid += vResults[iPrim].iInvalid;
            ReportProgress(bProgress, iPrim + 1, iPrimTotal, iBrepCount, iInvalid);
        }
    }

    int iBrepCount = 0;
    int iInvalid = 0;
    unsigned long lChecksFailed = 0;
    for (size_t iPrim = 0; iPrim < vOrderedBreps.size(); ++iPrim)
    {
        const OrderedGprim& rOrderedBrep = vOrderedBreps[iPrim];
        const SdfPath& sStablePath = rOrderedBrep.first;
        const PrimValidateResult& rResult = vResults[iPrim];
        if (rResult.bImportFailed)
        {
            std::fprintf(stderr, "ERROR: import failed for %s %s\n", sInput.c_str(), sStablePath.GetText());
            rbHardError = true;
        }
        else if (rResult.bMissingBreps)
        {
            std::fprintf(stderr, "ERROR: import produced missing Breps for %s %s\n",
                         sInput.c_str(), sStablePath.GetText());
            rbHardError = true;
        }

        iBrepCount += rResult.iBreps;
        iInvalid += rResult.iInvalid;
        lChecksFailed += rResult.lChecksFailed;
    }

    if (g_timingFp != nullptr)
    {
        std::fclose(g_timingFp);
        g_timingFp = nullptr;
    }

    std::printf("BREP_GEOMETRY file=%s breps=%d invalid=%d checksFailed=%lu\n",
                sInput.c_str(), iBrepCount, iInvalid, lChecksFailed);

    return iInvalid;
}

} // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> vInputs;
    int iLevel = 2;
    bool bQuiet = false;
    bool bProgress = false;
    bool bDiscoveryStats = false;
    int iWorkers = 1;
    std::string sTimingFile;
    SmBoolean bHealerIsEnabled = TRUE;

    for (int ii = 1; ii < argc; ++ii)
    {
        const std::string sArg = argv[ii];
        if (sArg == "-h" || sArg == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else if (sArg == "-q" || sArg == "--quiet")
        {
            bQuiet = true;
        }
        else if (sArg == "--no-healer")
        {
            bHealerIsEnabled = FALSE;
        }
        else if (sArg == "--progress")
        {
            bProgress = true;
        }
        else if (sArg == "--discovery-stats")
        {
            bDiscoveryStats = true;
        }
        else if (sArg == "--workers")
        {
            if (ii + 1 >= argc)
            {
                std::fprintf(stderr, "ERROR: --workers requires a thread count\n");
                return 2;
            }
            const std::string sWorkers = argv[++ii];
            char* pEnd = nullptr;
            const long lWorkers = std::strtol(sWorkers.c_str(), &pEnd, 10);
            if (pEnd == sWorkers.c_str() || *pEnd != '\0' || lWorkers < 0)
            {
                std::fprintf(stderr,
                             "ERROR: --workers expects a non-negative integer (0 = auto), got: %s\n",
                             sWorkers.c_str());
                return 2;
            }
            iWorkers = static_cast<int>(lWorkers);
        }
        else if (sArg == "--level")
        {
            if (ii + 1 >= argc)
            {
                std::fprintf(stderr, "ERROR: --level requires an argument (0, 1, or 2)\n");
                return 2;
            }
            const std::string sLevel = argv[++ii];
            if (sLevel != "0" && sLevel != "1" && sLevel != "2")
            {
                std::fprintf(stderr, "ERROR: --level expects 0, 1, or 2, got: %s\n", sLevel.c_str());
                return 2;
            }
            iLevel = std::atoi(sLevel.c_str());
        }
        else if (sArg == "--timing")
        {
            if (ii + 1 >= argc)
            {
                std::fprintf(stderr, "ERROR: --timing requires a file path\n");
                return 2;
            }
            sTimingFile = argv[++ii];
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

    // Register the omniSolid schema plugin before opening any stage so BrepArray prims resolve.
    if (!UsdBrepData::IsOmniSolidResourcesPluginRegistered())
    {
        // In a release package the schema is in omniSolid/resources, beside bin/.
        const std::filesystem::path packageSchema =
            std::filesystem::path(ArchGetExecutablePath()).parent_path().parent_path() / "omniSolid" / "resources";
        if (!std::getenv("OMNISOLID_PLUGIN_PATH") && std::filesystem::is_directory(packageSchema))
        {
#ifdef _WIN32
            _putenv_s("OMNISOLID_PLUGIN_PATH", packageSchema.string().c_str());
#else
            setenv("OMNISOLID_PLUGIN_PATH", packageSchema.c_str(), 0);
#endif
        }
        if (!UsdBrepData::RegisterOmniSolidResourcesPlugin())
        {
            std::fprintf(
                stderr,
                "ERROR: could not register the omniSolid schema plugin; set OMNISOLID_PLUGIN_PATH "
                "to <build>/schema/omniSolid/resources or <package>/omniSolid/resources\n"
            );
            return 3;
        }
    }

    const SmAssertTestLevel eLevel = ToTestLevel(iLevel);

    g_timingT0 = std::chrono::steady_clock::now();
    if (!sTimingFile.empty())
    {
        std::remove(sTimingFile.c_str());  // truncate: ValidateFile appends per input
    }

    int iTotalInvalid = 0;
    bool bAnyHardError = false;
    for (const std::string& sInput : vInputs)
    {
        bool bHardError = false;
        iTotalInvalid +=
            ValidateFile(sInput, eLevel, bQuiet, bProgress, bDiscoveryStats, bHealerIsEnabled, bHardError, iWorkers,
                         sTimingFile);
        bAnyHardError = bAnyHardError || bHardError;
    }

    std::printf("summary: %zu file(s), %d invalid brep(s)%s\n",
                vInputs.size(), iTotalInvalid, bAnyHardError ? " (with hard errors)" : "");

    if (iTotalInvalid > 0)
    {
        return iTotalInvalid > 250 ? 250 : iTotalInvalid;
    }
    return bAnyHardError ? 1 : 0;
}
