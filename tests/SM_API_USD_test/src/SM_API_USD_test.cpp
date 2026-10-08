// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SM_API_USD_test.cpp
* PURPOSE: Test runner for SM_API_USD unit tests.
**********************************************************************/

#include "SM_API_USD_test.h"
#include <SmSmlibAll.h>
#include <SmApiGeneral.h>

#include <cstdio>
#include <cstdlib>

SmStatus run_sm_api_usd(SmBoolean bDoGraphics)
{
    SmStatus stat;
    ULONG nSuccessful = 0, nFailed = 0;
    int iCnt = 0;

    smSet_DoGraphics(bDoGraphics);

    printf("\n\n********** SM_API_USD TESTS ************\n");

    // --- TestSmApiUsdExportBrep ---
    printf("\n\n******************** Start TestSmApiUsdExportBrep ************\n");
    stat = TestSmApiUsdExportBrep();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdExportBrep Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdExportBrep NOT successful ************\n");
    }

    // --- TestSmApiUsdExportBreps ---
    printf("\n\n******************** Start TestSmApiUsdExportBreps ************\n");
    stat = TestSmApiUsdExportBreps();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdExportBreps Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdExportBreps NOT successful ************\n");
    }

    // --- TestSmApiUsdImportBreps ---
    printf("\n\n******************** Start TestSmApiUsdImportBreps ************\n");
    stat = TestSmApiUsdImportBreps();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdImportBreps Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdImportBreps NOT successful ************\n");
    }

    // --- TestSmApiUsdImportBrep ---
    printf("\n\n******************** Start TestSmApiUsdImportBrep ************\n");
    stat = TestSmApiUsdImportBrep();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdImportBrep Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdImportBrep NOT successful ************\n");
    }

    // --- TestSmApiUsdImportRejectsMixedValidInvalidBreps ---
    printf("\n\n******************** Start TestSmApiUsdImportRejectsMixedValidInvalidBreps ************\n");
    stat = TestSmApiUsdImportRejectsMixedValidInvalidBreps();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdImportRejectsMixedValidInvalidBreps Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdImportRejectsMixedValidInvalidBreps NOT successful ************\n");
    }

    // --- TestSmApiUsdImportRejectsEmptyBrepArray ---
    printf("\n\n******************** Start TestSmApiUsdImportRejectsEmptyBrepArray ************\n");
    stat = TestSmApiUsdImportRejectsEmptyBrepArray();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdImportRejectsEmptyBrepArray Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdImportRejectsEmptyBrepArray NOT successful ************\n");
    }

    // --- TestSmApiUsdImportRejectsSingularWorldTransform ---
    printf("\n\n******************** Start TestSmApiUsdImportRejectsSingularWorldTransform ************\n");
    stat = TestSmApiUsdImportRejectsSingularWorldTransform();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdImportRejectsSingularWorldTransform Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdImportRejectsSingularWorldTransform NOT successful ************\n");
    }

    // --- TestSmApiUsdRoundtrip ---
    printf("\n\n******************** Start TestSmApiUsdRoundtrip ************\n");
    stat = TestSmApiUsdRoundtrip();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdRoundtrip Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdRoundtrip NOT successful ************\n");
    }

    // --- TestSmApiUsdAppendBreps ---
    printf("\n\n******************** Start TestSmApiUsdAppendBreps ************\n");
    stat = TestSmApiUsdAppendBreps();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdAppendBreps Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdAppendBreps NOT successful ************\n");
    }

    printf("\n\n******************** Start TestSmApiUsdMeshRoundtrip ************\n");
    stat = TestSmApiUsdMeshRoundtrip();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdMeshRoundtrip Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdMeshRoundtrip NOT successful ************\n");
    }

    // --- TestSmApiUsdTessellateFileLeavesInputLayer ---
    printf("\n\n******************** Start TestSmApiUsdTessellateFileLeavesInputLayer ************\n");
    stat = TestSmApiUsdTessellateFileLeavesInputLayer();
    iCnt++;
    if (stat == SM_SUCCESS) {
        nSuccessful++;
        printf("\n\n******************** End TestSmApiUsdTessellateFileLeavesInputLayer Successful ************\n");
    } else {
        nFailed++;
        printf("\n\n******************** End TestSmApiUsdTessellateFileLeavesInputLayer NOT successful ************\n");
    }

    // --- Summary ---
    printf("\n\n*******************************************************************\n");
    printf("***********************  END OF SM_API_USD TEST  *****************\n");
    printf("*******************************************************************\n\n");
    printf("************************************************\n");
    printf("****************      RESULTS      *************\n");
    printf("************************************************\n");
    printf("**************** %lu Successful    *************\n", nSuccessful);
    printf("**************** %lu Failed        *************\n", nFailed);

    if (nFailed == 0) {
        printf("\n  SM_API_USD Test Final Result = Success: All %d tests are still Working\n", iCnt);
        printf("\n\n************************* End SM_API_USD Test with SUCCESS ************\n");
    } else {
        printf("\n  SM_API_USD Test Final Result = Failure: [%lu of %d] tests failed\n", nFailed, iCnt);
        printf("\n\n************************* End SM_API_USD Test with PROBLEMS ************\n");
    }

    return (nFailed > 0) ? SM_ERR : SM_SUCCESS;
}

#ifdef SM_API_USD_TEST_STANDALONE

#include <csignal>

int main(int /*argc*/, char* /*argv*/[])
{
    // GitLab CI and other non-TTY runners often fully buffer stdout; flush so logs
    // show which sub-test failed instead of losing output on early exit.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    std::signal(SIGSEGV, [](int) { std::_Exit(139); });
    std::signal(SIGABRT, [](int) { std::_Exit(134); });

    printf("\nSM_API_USD Standalone Test Runner\n");
    printf("=================================\n\n");

    SmStatus result = run_sm_api_usd(FALSE);

    printf("\n=================================\n");
    if (result == SM_SUCCESS)
        printf("RESULT: PASS\n");
    else
        printf("RESULT: FAIL (code %ld)\n", result);

    return (result == SM_SUCCESS) ? 0 : 1;
}

#endif // SM_API_USD_TEST_STANDALONE
