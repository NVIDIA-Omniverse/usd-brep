// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smmerge_test.h
* PURPOSE ---  Header file for merge tests
*
* 
**********************************************************************/
/*___*/
#ifndef __SMMERGE_TESTS_H__
#define __SMMERGE_TESTS_H__

#include <prog_test.h>
#include <SmTopoTypes.h>

PT_EXPORT SmStatus my_test_bbu(SmBrep *cpBrep1,
                               SmBrep *cpBrep2,
                               ULONG lOperation,
                               SmBrep *& rpResult);

PT_EXPORT SmStatus my_test_merge();
PT_EXPORT SmStatus my_mouse_demo();

PT_EXPORT SmStatus my_stitching_demo();

PT_EXPORT SmStatus my_test_BrepPerShell();

PT_EXPORT SmStatus my_test_stress() ;
PT_EXPORT SmStatus my_test_non_manifold() ;

#endif
