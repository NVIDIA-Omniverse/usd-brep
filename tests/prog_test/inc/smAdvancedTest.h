// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smadvanced_test.h
* PURPOSE --- 
*
**********************************************************************/
/*___*/

#ifndef __SMADVANCED_TEST_H__
#define __SMADVANCED_TEST_H__

#include <prog_test.h> 

class SmBrep;
class SmCurve;
class SmSurface;

PT_EXPORT SmStatus my_ssi_analytic_demo();

PT_EXPORT SmStatus my_test_tsurface_regression();

PT_EXPORT SmStatus my_test_ssi_adv(void);

PT_EXPORT SmStatus my_test_sil_adv(void);

PT_EXPORT SmStatus my_test_sec_adv();

PT_EXPORT SmStatus my_test_cci_adv(void);

// run all smsurf_test.h and smcurve_test.h unit tests
PT_EXPORT SmStatus my_unit_test_suite() ;

SmStatus my_test_ssi_analytic();

#endif
