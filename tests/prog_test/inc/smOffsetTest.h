// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smfillet_test.h
* PURPOSE --- Include file for testing the filleting
*
**********************************************************************/
/*___*/

#ifndef __SMOFFSET_TEST_H__
#define __SMOFFSET_TEST_H__

#include <prog_test.h>
#include <SmTopoTypes.h>

PT_EXPORT SmStatus my_offset_regression();

PT_EXPORT SmStatus my_test_offset(const SmContext & crContext,
                                  const TCHAR * pFileName,
                                  ULONG   lShellFace,
                                  double dFilletRadius,
                                  double dTolerance,
                                  SmTArray<SmBrep*> & rPartBreps);

PT_EXPORT SmStatus my_offset_demo(const SmContext & crContext,
                                  SmTArray<SmBrep*> & rPartBreps,
                                  ULONG & lCount);

PT_EXPORT SmStatus my_shell_demo(const SmContext & crContext,
                                 SmTArray<SmBrep*> & rPartBreps,
                                 ULONG & lCount);

PT_EXPORT SmStatus ShellBrep(const SmContext & crContext,
                             const TCHAR * filename,
                             double dOffsetDistance, // Offset distance - if
                             // negative do an inset.
                             SmBoolean bExtendCorners,
                             SmBoolean bMergeResults,                 // NotUsed: in : bMergeResults
                             const SmTArray<ULONG> & crFacesToShell,
                             SmTArray<SmBrep*> & rPartBreps) ;

#endif // __SMOFFSET_TEST_H__
