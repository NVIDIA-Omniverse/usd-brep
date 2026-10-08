// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smtess_test.h
* PURPOSE ---  Test tessellation header.
*
**********************************************************************/
/*___*/

#ifndef __SMTESS_TEST_H__
#define __SMTESS_TEST_H__

#include <prog_test.h>
                                          
PT_EXPORT SmStatus my_test_one_tess(SmBrep *pBrep,
                                    double dCHTol,
                                    double dAngleTolDeg,
                                    double dMax3DEdge,
                                    double dMaxAspect,
                                    SmBoolean & rbFailedFaces);

PT_EXPORT SmStatus my_test_tess();

PT_EXPORT SmStatus my_test_ppu_poly(const SmContext & crContext,
    SmPolyBrep *pPolyBrep1,
    SmPolyBrep *pPolyBrep2,
    ULONG lOperation,  // 0 - union, 1 - intersection, 2 - difference, 3 - merge
    SmPolyBrep *& rpResult); // ---	API remeshes an STL input file



SmStatus my_test_decimation_regression();
SmStatus my_poly_boolean_regression();

SmStatus my_test_decimation(const SmContext & crContext,
    const TCHAR     * pFileName,
    double            dTessellationChTol,
    double            dCrvAngleTolDeg,
    double            dSrfAngleTolDeg,
    double            dPercentReduction,
    double            dMaximumReductionError,
    ULONG             lStage,     // 1 - Just Read in Triangles
                                  // 2 - Decimate Triangles,
                                  // 3 - Both read and decimate
    SmBoolean         bQuadricDecimation,
    SmTArray<SmPolyBrep*> & rBreps);
#endif // __SMTESS_TEST_H__
