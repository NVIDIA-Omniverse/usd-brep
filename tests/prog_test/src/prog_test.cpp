// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- prog_test.cpp
* PURPOSE --- Main file.
*
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <prog_test.h>
#include <smAdvancedTest.h>
#include <smFilletTest.h>
#include <smMergeTest.h>
#include <smOffsetTest.h>

#include <cstring>
#include <filesystem>

#ifdef APPEND_GIT_BRANCH
#include "../../../_build/generated/SmGitBranch.h"

#    define Q(x) #x
#    define QUOTE(x) Q(x)
#endif

typedef SmStatus (*ProgSuiteFunc)();

struct ProgSuite
{
    const char*   pName;
    const TCHAR*  pBanner;
    const TCHAR*  pFailureName;
    const TCHAR*  pTimeFormat;
    ProgSuiteFunc pFunc;
    SmBoolean     bRunByDefault;
};

static SmStatus my_test_fillet_const_radius()
{
    return my_test_fillet_regression(SM_FS_CONST_RADIUS);
}

static const ProgSuite gProgSuites[] =
{
    { "topology",          _T("**************************************  Testing Topology    ****************************************"),       _T("my_test_topology"),          NULL,                                                          my_test_topology,           TRUE  },
    { "booleans",          _T("**************************************  Testing Booleans    ****************************************"),       _T("my_test_merge"),             _T("**************** Testing Booleans Time %lf secs **********\n"), my_test_merge,              TRUE  },
    { "offset",            _T("*************************************  Testing Offsetting    ***************************************"),       _T("my_offset_regression"),      _T("********** Testing Offsetting Time %lf secs **********\n"),    my_offset_regression,       TRUE  },
    { "fillets",           _T("*************************************  Testing Filleting     ***************************************"),       _T("my_test_fillet_regression"), _T("********** Testing Filleting Time %lf secs **********\n"),     my_test_fillet_const_radius, TRUE },
    { "local-ops",         NULL,                                                                                                             _T("my_local_op_regression"),    NULL,                                                          my_local_op_regression,     TRUE  },
    { "sweeps",            _T("*************************************  Testing Sweeping    *****************************************"),       _T("my_sweep_regression"),       NULL,                                                          my_sweep_regression,        TRUE  },
    { "primitives",        _T("********************************  Testing Primitive Creation    ************************************"),       _T("my_primsol_regression"),     NULL,                                                          my_primsol_regression,      TRUE  },
    { "tessellation",      _T("********************************  Polygon Optimization Library  ************************************"),       _T("my_test_tess"),             NULL,                                                          my_test_tess,              TRUE  },
    { "unit",              _T("***************************************  Run Unit Tests ********************************************"),       _T("my_unit_test_suite"),        NULL,                                                          my_unit_test_suite,         TRUE  },
    { "trimmed-surfaces",  _T("******************************* Trimmed Surface Regression *****************************************"),       _T("my_test_tsurface_regression"), NULL,                                                        my_test_tsurface_regression, TRUE },
    { "brep-import",       _T("******************************* Brep Import Regression *****************************************"),           _T("my_test_brep_import"),      NULL,                                                          my_test_brep_import,        TRUE  },
    { "ssi-analytic",      _T("*********************************    Analytic Surface   ********************************************"),       _T("my_test_ssi_analytic"),      NULL,                                                          my_test_ssi_analytic,       TRUE  },
    { "ssi-advanced",      _T("*****************************   Testing Advanced SSI   *********************************************"),       _T("my_test_ssi_adv"),           NULL,                                                          my_test_ssi_adv,            TRUE  },
    { "cci-advanced",      _T("***************************** Testing Advanced CCI *************************************************"),       _T("my_test_cci_adv"),           NULL,                                                          my_test_cci_adv,            TRUE  },
    { "section-advanced",  _T("***************************** Testing Advanced Sectioning ******************************************"),       _T("my_test_sec_adv"),           NULL,                                                          my_test_sec_adv,            TRUE  },
    { "silhouette-advanced", _T("**************************** Testing Advanced Silhouette *******************************************"),     _T("my_test_sil_adv"),           NULL,                                                          my_test_sil_adv,            TRUE  },
    { "stitch",            _T("**************************************  Testing Stitching   ****************************************"),       _T("my_stitching_demo"),         NULL,                                                          my_stitching_demo,          FALSE }
};

static const int gProgSuiteCount = sizeof(gProgSuites) / sizeof(gProgSuites[0]);

static const ProgSuite* find_prog_suite(const char* pName)
{
    for (int ii = 0; ii < gProgSuiteCount; ++ii)
    {
        if (strcmp(gProgSuites[ii].pName, pName) == 0)
        {
            return &gProgSuites[ii];
        }
    }
    return NULL;
}

static const ProgSuite* find_legacy_test(const char* pName)
{
    if (strcmp(pName, "merge") == 0)
    {
        return find_prog_suite("booleans");
    }
    if (strcmp(pName, "stitch") == 0)
    {
        return find_prog_suite("stitch");
    }
    return NULL;
}

int prog_test_suite_count()
{
    return gProgSuiteCount;
}

const char* prog_test_suite_name(int lIndex)
{
    if (lIndex < 0 || lIndex >= gProgSuiteCount)
    {
        return NULL;
    }
    return gProgSuites[lIndex].pName;
}

SmBoolean prog_test_suite_runs_by_default(int lIndex)
{
    if (lIndex < 0 || lIndex >= gProgSuiteCount)
    {
        return FALSE;
    }
    return gProgSuites[lIndex].bRunByDefault;
}

SmBoolean prog_test_suite_exists(const char* pName)
{
    if (!pName)
    {
        return FALSE;
    }
    return find_prog_suite(pName) != NULL ? TRUE : FALSE;
}

static double prog_test_suites(TCHAR* pFileName,
                               TCHAR* pFileNameThin,
                               TCHAR* pFileNameDebugLog,
                               SmBoolean* bAllTestsOk,
                               const ProgSuite* const* ppSuites,
                               int lSuiteCount);

static SmStatus run_prog_suite(const ProgSuite& rSuite,
                               SmBoolean* bAllTestsOk,
                               ProgTestRunResult* pResult)
{
    TCHAR sBuff[SM_TBLOCK_SIZE];
    if (pResult)
    {
        pResult->pSuiteName = rSuite.pName;
        pResult->eStatus = SM_ERR;
        pResult->bOk = FALSE;
        pResult->dElapsedSeconds = 0.0;
    }

    if (rSuite.pBanner)
    {
        MYPRINTF(_T("\n\n"));
        SM_SPRINTF(sBuff, _T("%s\n"), rSuite.pBanner);
        MYPRINTF(sBuff);
    }

    clock_t start = clock();
    SmStatus stat = rSuite.pFunc();
    if (stat != SM_SUCCESS)
    {
        SM_SPRINTF(sBuff, _T("Failed: %s\n\n"), rSuite.pFailureName);
        MYPRINTF(sBuff);
        if (bAllTestsOk) *bAllTestsOk = false;
    }

    clock_t finish = clock();
    double elapsedSeconds = static_cast<double>(finish - start) / static_cast<double>(CLOCKS_PER_SEC);

    if (rSuite.pTimeFormat)
    {
        MYPRINTF(_T("\n\n"));
        SM_SPRINTF(sBuff, rSuite.pTimeFormat, elapsedSeconds);
        MYPRINTF(sBuff);
    }

    if (pResult)
    {
        pResult->eStatus = stat;
        pResult->bOk = (stat == SM_SUCCESS) ? TRUE : FALSE;
        pResult->dElapsedSeconds = elapsedSeconds;
    }

    return stat;
}

SmStatus prog_test_run_suite_by_name(const char* pName, ProgTestRunResult* pResult)
{
    if (!pName || !pResult)
    {
        return SM_ERR_INVALID_INPUT;
    }

    const ProgSuite* pSuite = find_prog_suite(pName);
    if (!pSuite)
    {
        pResult->pSuiteName = pName;
        pResult->eStatus = SM_ERR_INVALID_INPUT;
        pResult->bOk = FALSE;
        pResult->dElapsedSeconds = 0.0;
        return SM_ERR_INVALID_INPUT;
    }

    SmBoolean bSuiteOk = TRUE;
    SmStatus stat = run_prog_suite(*pSuite, &bSuiteOk, pResult);
    pResult->bOk = (stat == SM_SUCCESS && bSuiteOk) ? TRUE : FALSE;
    return stat;
}

double prog_test_named_suites(TCHAR* pFileName,
                              TCHAR* pFileNameThin,
                              TCHAR* pFileNameDebugLog,
                              SmBoolean* bAllTestsOk,
                              const char* const* ppSuiteNames,
                              int lSuiteCount)
{
    if (lSuiteCount <= 0)
    {
        return prog_test_suites(pFileName, pFileNameThin, pFileNameDebugLog, bAllTestsOk, NULL, 0);
    }

    if (!ppSuiteNames || lSuiteCount > gProgSuiteCount)
    {
        if (bAllTestsOk)
        {
            *bAllTestsOk = FALSE;
        }
        return 0.0;
    }

    const ProgSuite* sSelectedSuites[sizeof(gProgSuites) / sizeof(gProgSuites[0])];
    for (int ii = 0; ii < lSuiteCount; ++ii)
    {
        const ProgSuite* pSuite = find_prog_suite(ppSuiteNames[ii]);
        if (!pSuite)
        {
            pSuite = find_legacy_test(ppSuiteNames[ii]);
        }
        if (!pSuite)
        {
            if (bAllTestsOk)
            {
                *bAllTestsOk = FALSE;
            }
            return 0.0;
        }
        sSelectedSuites[ii] = pSuite;
    }

    return prog_test_suites(pFileName, pFileNameThin, pFileNameDebugLog, bAllTestsOk,
                            sSelectedSuites, lSuiteCount);
}

static double prog_test_suites
 (TCHAR                 * pFileName,         // out:
  TCHAR                 * pFileNameThin,     // out:
  TCHAR                 * pFileNameDebugLog, // out:
  SmBoolean             * bAllTestsOk,       // out:
  const ProgSuite* const* ppSuites,
  int                     lSuiteCount)
{
    if(bAllTestsOk) *bAllTestsOk = true;

    namespace fs = std::filesystem;
    std::error_code sErrCode;

    // Let's first check if the test files exist
    if (!fs::is_directory("../../TestFiles/pt_TestFiles", sErrCode))
    {
        if (bAllTestsOk) *bAllTestsOk = false;
        return 0.0;
    }

    // Now let's check if the OutputFiles Folder exists
    // If not, we can create folder to place output files
    if (smGet_OutputLong() || smGet_OutputThin() || smGet_OutputDebugLog())
    {
        fs::create_directory("../prog_test/OutputFiles", sErrCode);
        if (!fs::is_directory("../prog_test/OutputFiles", sErrCode))
        {
            if (bAllTestsOk) *bAllTestsOk = false;
            return -1.0;
        }
    }

        // Build log filename from local time
        time_t lTime = time(NULL);
        struct tm* pTm = localtime(&lTime);

        TCHAR sTimeStamp[SM_TBLOCK_SIZE];
        SM_SPRINTF(sTimeStamp, _T("%d_%d_%d_%d"), pTm->tm_mon + 1, pTm->tm_mday, pTm->tm_hour, pTm->tm_min);

        if (smGet_OutputLong())
        {
            smos_WStrCpy(pFileName, SM_TBLOCK_SIZE, _T("../prog_test/OutputFiles/"));
            smos_WStrCat(pFileName, _T("progTest"));
            smos_WStrCat(pFileName, _T("_"));

            smos_WStrCat(pFileName, sTimeStamp);
#ifdef APPEND_GIT_BRANCH
#ifdef SM_DEBUG_CODE
            smos_WStrCat(pFileName, _T("_"));
            smos_WStrCat(pFileName, _T(QUOTE(GIT_BRANCH)));
#else
            smos_WStrCat(pFileName, _T("_"));
            smos_WStrCat(pFileName, _T(Q(GIT_BRANCH)));
#endif
#endif
            smos_WStrCat(pFileName, _T("_long.txt"));
        }
        else
        {
            pFileName = NULL;
        }

        if (smGet_OutputThin())
        {
            smos_WStrCpy(pFileNameThin, SM_TBLOCK_SIZE, _T("../prog_test/OutputFiles/"));
            smos_WStrCat(pFileNameThin, _T("progTest"));
            smos_WStrCat(pFileNameThin, _T("_"));

            smos_WStrCat(pFileNameThin, sTimeStamp);
#ifdef APPEND_GIT_BRANCH
#ifdef SM_DEBUG_CODE
            smos_WStrCat(pFileNameThin, _T("_"));
            smos_WStrCat(pFileNameThin, _T(QUOTE(GIT_BRANCH)));
#else
            smos_WStrCat(pFileNameThin, _T("_"));
            smos_WStrCat(pFileNameThin, _T(Q(GIT_BRANCH)));
#endif
#endif
            smos_WStrCat(pFileNameThin, _T(".txt"));
        }
        else
        {
            pFileNameThin = NULL;
        }

        if (smGet_OutputDebugLog())
        {
            smos_WStrCpy(pFileNameDebugLog, SM_TBLOCK_SIZE, _T("../prog_test/OutputFiles/"));
            smos_WStrCat(pFileNameDebugLog, _T("progTest"));
            smos_WStrCat(pFileNameDebugLog, _T("_"));

            smos_WStrCat(pFileNameDebugLog, sTimeStamp);
#ifdef APPEND_GIT_BRANCH
#ifdef SM_DEBUG_CODE
            smos_WStrCat(pFileNameDebugLog, _T("_"));
            smos_WStrCat(pFileNameDebugLog, _T(QUOTE(GIT_BRANCH)));
#else
            smos_WStrCat(pFileNameDebugLog, _T("_"));
            smos_WStrCat(pFileNameDebugLog, _T(Q(GIT_BRANCH)));
#endif
#endif
            smos_WStrCat(pFileNameDebugLog, _T("_DebugLog.txt"));
        }
        else
        {
            pFileNameDebugLog = NULL;
        }

    // create output files and redirect StdOut to that file
    FILE *pStream = smos_DirectStdOutToFile(pFileName, pFileNameThin, pFileNameDebugLog);

    MYPRINTF(_T("\n\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));
    MYPRINTF(_T("****************************************     PROG_TEST      ****************************************\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));
    
    // Just run through the programatic test suite.
    // Note that if you do not have a pool based memory system
    // it may fail because no memory is freed in most of the tests
    // by any other mechanism.
    // It worked fine on my machine without the pooled memory but 
    // I have a lot of memory.

    TCHAR sBuff[SM_TBLOCK_SIZE];
    clock_t startAll = clock();

    if (ppSuites && lSuiteCount > 0)
    {
        for (int ii = 0; ii < lSuiteCount; ++ii)
        {
            run_prog_suite(*ppSuites[ii], bAllTestsOk, NULL);
        }
    }
    else
    {
        for (int ii = 0; ii < gProgSuiteCount; ++ii)
        {
            if (gProgSuites[ii].bRunByDefault)
            {
                run_prog_suite(gProgSuites[ii], bAllTestsOk, NULL);
            }
        }
    }


    clock_t finishAll = clock();

    // Dividing clock() ticks by CLOCKS_PER_SEC yields seconds on every platform
    // (Windows defines it as 1000, Linux/macOS as 1000000), so no per-platform
    // fix-up is needed. Matches the per-suite timing above (elapsedSeconds).
    double dDuration = static_cast<double>(finishAll - startAll) / static_cast<double>(CLOCKS_PER_SEC);

    MYPRINTF(_T("\n\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));
    MYPRINTF(_T("*************************************     END OF PROG-TEST      ************************************\n"));
    SM_SPRINTF(sBuff, _T("*************************************    Time %lf secs   ************************************\n"), dDuration);
    MYPRINTF(sBuff);
    MYPRINTF(_T("****************************************************************************************************\n"));
    MYPRINTF(_T("****************************************************************************************************\n"));

    // restore stdout pStream to window
    smos_RestoreStdOut(pStream);

    // all done
    return dDuration;

} // end prog_test
