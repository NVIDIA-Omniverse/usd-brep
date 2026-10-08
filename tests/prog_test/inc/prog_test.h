// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- prog_test.h
* PURPOSE --- Main Header file 
*
* 
**********************************************************************/
/*___*/

#ifndef __PROG_TEST_H__
#define __PROG_TEST_H__

#ifdef _WIN32
#define PT_EXPORT   __declspec( dllexport )
#elif defined(__GNUC__) && __GNUC__ >= 4
#define PT_EXPORT   __attribute__((visibility("default")))
#else
#define PT_EXPORT
#endif

#include <SmMessages.h>

struct ProgTestRunResult
{
    const char* pSuiteName;
    SmStatus    eStatus;
    SmBoolean   bOk;
    double      dElapsedSeconds;
};

PT_EXPORT int prog_test_suite_count();
PT_EXPORT const char* prog_test_suite_name(int lIndex);
PT_EXPORT SmBoolean prog_test_suite_runs_by_default(int lIndex);
PT_EXPORT SmBoolean prog_test_suite_exists(const char* pName);

PT_EXPORT SmStatus prog_test_run_suite_by_name(const char* pName,
                                               ProgTestRunResult* pResult);

PT_EXPORT double prog_test_named_suites(TCHAR* pFileName,
                                        TCHAR* pFileNameThin,
                                        TCHAR* pFileNameDebugLog,
                                        SmBoolean* bAllTestsOk,
                                        const char* const* ppSuiteNames,
                                        int lSuiteCount);

// include test header files
#include <smCurveTest.h>     
#include <smSurfaceTest.h>      
#include <smSweepTest.h> 
#include <smTopoTest.h>     
#include <smTessTest.h>     
#include <smFeatureTest.h> 


#endif // __PROG_TEST_H__
