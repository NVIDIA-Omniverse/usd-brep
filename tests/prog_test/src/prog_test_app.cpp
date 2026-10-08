// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- prog_test_app.cpp
* PURPOSE --- Command line entry point for prog_test.
*
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <prog_test.h>

#include <cstring>
#include <vector>

static void print_prog_suites()
{
    for (int ii = 0; ii < prog_test_suite_count(); ++ii)
    {
        printf("%s", prog_test_suite_name(ii));
        if (!prog_test_suite_runs_by_default(ii))
        {
            printf(" (not run by default)");
        }
        printf("\n");
    }
}

static int print_usage_and_return(const char* pError)
{
    if (pError)
    {
        printf("%s\n", pError);
    }
    printf("Usage:\n");
    printf("  prog_test_app\n");
    printf("  prog_test_app --list-suites\n");
    printf("  prog_test_app --suite <name> [--suite <name> ...]\n");
    printf("  prog_test_app --test <merge|stitch>\n");
    printf("\nAvailable suites:\n");
    print_prog_suites();
    return 1;
}

static int run_prog_test_from_main(const char* const* ppSuiteNames, int lSuiteCount)
{
    TCHAR sOutputFileName[SM_TBLOCK_SIZE];
    TCHAR sOutputFileNameThin[SM_TBLOCK_SIZE];
    TCHAR sOutputFileNameDebugLog[SM_TBLOCK_SIZE];

    smSet_OutputLong(TRUE);
    smSet_OutputThin(TRUE);

    TCHAR sBuff[SM_TBLOCK_SIZE];
    SmBoolean bAllTestsOk = true;
    double dDuration = 0.0;
    printf("START PROG_TEST\n\n");

    dDuration = prog_test_named_suites(sOutputFileName, sOutputFileNameThin, sOutputFileNameDebugLog,
                                       &bAllTestsOk, ppSuiteNames, lSuiteCount);

    printf("END PROG_TEST\n\n");

    SM_SPRINTF(sBuff, _T("PROG_TEST DURATION: time %.2fs\n\n"), dDuration);
    SM_PRINTF(sBuff);

    if (bAllTestsOk)
    {
        MYPRINTF(_T("PROG_TEST SUCCESS\n"));
    }
    else
    {
        MYPRINTF(_T("PROG_TEST FAILED\n"));
    }

    return (bAllTestsOk) ? 0 : 1;
}

/***********************************************************************
PURPOSE ---  run prog_test from team city.

RETURNS ---
***********************************************************************/
int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        return run_prog_test_from_main(NULL, 0);
    }

    std::vector<const char*> selectedSuites;
    selectedSuites.reserve(static_cast<size_t>(prog_test_suite_count()));

    for (int ii = 1; ii < argc; ++ii)
    {
        if (strcmp(argv[ii], "--list-suites") == 0)
        {
            print_prog_suites();
            return 0;
        }
        if (strcmp(argv[ii], "--suite") == 0)
        {
            if (ii + 1 >= argc)
            {
                return print_usage_and_return("Missing value for --suite.");
            }
            const char* pName = argv[++ii];
            if (!prog_test_suite_exists(pName))
            {
                printf("Unknown suite: %s\n", pName);
                return print_usage_and_return(NULL);
            }
            if (selectedSuites.size() >= static_cast<size_t>(prog_test_suite_count()))
            {
                return print_usage_and_return("Too many suites selected.");
            }
            selectedSuites.push_back(pName);
            continue;
        }
        if (strcmp(argv[ii], "--test") == 0)
        {
            if (ii + 1 >= argc)
            {
                return print_usage_and_return("Missing value for --test.");
            }
            const char* pName = argv[++ii];
            if (strcmp(pName, "merge") == 0)
            {
                pName = "booleans";
            }
            else if (strcmp(pName, "stitch") != 0)
            {
                printf("Unknown test name: %s\n", argv[ii]);
                return print_usage_and_return(NULL);
            }
            if (selectedSuites.size() >= static_cast<size_t>(prog_test_suite_count()))
            {
                return print_usage_and_return("Too many suites selected.");
            }
            selectedSuites.push_back(pName);
            continue;
        }

        printf("Unknown argument: %s\n", argv[ii]);
        return print_usage_and_return(NULL);
    }

    if (selectedSuites.empty())
    {
        return print_usage_and_return("No suites selected.");
    }

    return run_prog_test_from_main(selectedSuites.data(), static_cast<int>(selectedSuites.size()));

} // end main
