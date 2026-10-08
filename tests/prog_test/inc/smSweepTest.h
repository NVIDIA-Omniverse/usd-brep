// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smsweep_test.h
* PURPOSE ---  Header file for sweep tests
*
**********************************************************************/
/*___*/

#ifndef __SMSWEEP_TEST_H__
#define __SMSWEEP_TEST_H__

#include <prog_test.h>

PT_EXPORT SmStatus my_sweep_regression();

PT_EXPORT SmStatus my_sweep_demo(const SmContext &crContext,
                                 SmTArray<SmBrep*> & rPartBreps,
                                 ULONG nTest = 1000);

PT_EXPORT ULONG my_sweep_demo_count(void);

PT_EXPORT SmStatus my_primsol_regression();

/***********************************************************************
PURPOSE --- Run a specific test (nTest) or all tests in turn 
            (nTest omitted or -1)

USAGE NOTES --- 
***********************************************************************/

PT_EXPORT SmStatus my_primsol_demo(SmContext & crContext,
                                   SmTArray<SmBrep*> & rPartBreps,
                                   ULONG nTest = 1000);

PT_EXPORT ULONG my_primsol_demo_count(void);



#endif



