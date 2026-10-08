// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SM_API_USD_test.h
* PURPOSE --- Header file for SM_API_USD unit tests
**********************************************************************/

#ifndef SM_API_USD_TEST_H
#define SM_API_USD_TEST_H

#include <SmTypes.h>

#if defined(_WIN32)
#  define API_USD_TEST_EXPORT __declspec( dllexport )
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define API_USD_TEST_EXPORT __attribute__((visibility("default")))
#else
#  define API_USD_TEST_EXPORT
#endif

// TestSmApiUsd -- import/export round-trip tests
API_USD_TEST_EXPORT SmStatus TestSmApiUsdExportBrep();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdExportBreps();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdImportBreps();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdImportBrep();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdImportRejectsMixedValidInvalidBreps();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdImportRejectsEmptyBrepArray();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdImportRejectsSingularWorldTransform();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdRoundtrip();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdAppendBreps();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdMeshRoundtrip();
API_USD_TEST_EXPORT SmStatus TestSmApiUsdTessellateFileLeavesInputLayer();

// Main entry point
API_USD_TEST_EXPORT SmStatus run_sm_api_usd(SmBoolean bDoGraphics = FALSE);

#endif // SM_API_USD_TEST_H
