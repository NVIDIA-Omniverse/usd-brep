// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smfillet_test.h
* PURPOSE --- Include file for testing the filleting
*
**********************************************************************/
/*___*/

#ifndef SM_FILLET_TEST_H
#define SM_FILLET_TEST_H

#include <prog_test.h>
#include <SmFilletSolver.h>

PT_EXPORT SmStatus my_test_fillet_regression(SmFilletSolverType eSolverType = SM_FS_CONST_RADIUS);

PT_EXPORT SmStatus my_read_fillet_definition_file(const SmContext & crContext, const TCHAR * cInputFileName, SmTArray<SmBrep*> & rBreps);

PT_EXPORT SmStatus my_local_op(const SmContext & crContext, ULONG & rlCount);

PT_EXPORT SmStatus my_local_op_regression();



#endif // SM_FILLET_TEST_H
