// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smtopo_test.h
* PURPOSE ---  Header file for topology tests
*
**********************************************************************/
/*___*/

#ifndef __SMTOPO_TEST_H__
#define __SMTOPO_TEST_H__

#include <prog_test.h>

PT_EXPORT SmStatus my_test_tsurf_cyl_creation_fast(const SmContext & crContext, SmBrep *& rpNewBrep, SmBoolean bDoFast = TRUE);

PT_EXPORT SmStatus my_test_topology();

PT_EXPORT SmStatus my_test_analytic_tsurf_creation(const SmContext & crContext);

PT_EXPORT SmStatus my_test_tsurf_creation(const SmContext & crContext,
                                          SmBrep *& rpNewBrep);

PT_EXPORT SmStatus my_test_tsurf_insert_edge();

PT_EXPORT SmStatus my_test_tsurface_curve_classify(const SmBrep *cpBrep,ULONG lNum,ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_tsurface_point_classify(const SmBrep *cpBrep,
                                               long lNumU,
                                               long lNumV,
                                               ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_tsurface_point_extrema(const SmBrep *cpBrep,
                                                  SmSolverOperationType eSolverOperation,
                                                  long lNumX,
                                                  long lNumY,
                                                  long lNumZ,
                                                  const SmVector3d * cpOptVectors,
                                                  ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_tsurface_point_intersect(const SmBrep *cpBrep,
                                                    long lNumU,
                                                    long lNumV,
                                                    ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_tsurface_curve_solve(const SmBrep *cpBrep,
                                                SmSolverOperationType eSolverOp,
                                                long lNumX,
                                                long lNumY,
                                                const SmVector3d * cpOptVectors,
                                                ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_tsurface_tsurface_solve(const SmBrep *cpBrep1, const SmBrep *cpBrep2,
                                                   SmSolverOperationType eSolverOp,
                                                   const SmVector3d * cpOptVectors,
                                                   ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_ssi(const SmBrep * cpBrep1,
                               const SmBrep * cpBrep2,
                               SmApproxTol3d  dApproxTol,
                               ULONG        & rlNumFound);

PT_EXPORT SmStatus my_test_tsurf_projection(const SmBrep * cpBrep,
                                            SmApproxTol3d  dApproxTol,
                                            ULONG          lNumCurves,
                                            ULONG        & rlNumFound);

PT_EXPORT SmStatus my_test_tsurf_section(const SmBrep      * cpBrep,
                                          SmApproxTol3d      dApproxTol,
                                          const SmVector3d & crPlaneNormal,
                                          ULONG              lNumPlanes,
                                          ULONG            & rlNumFound);

PT_EXPORT SmStatus my_test_tsurf_silhouette(const SmBrep     * cpBrep,
                                            SmApproxTol3d      dApproxTol,
                                            const SmVector3d & crPlaneNormal,
                                            SmBoolean          bPerspective,
                                            ULONG            & rlNumFound);

PT_EXPORT SmStatus my_topo_regression_test(SmBrep *pBrep,
                                           ULONG lPointClass,
                                           ULONG lPointMin,
                                           ULONG lPointMax,
                                           ULONG lPointNorm,
                                           ULONG lPointIntersect,
                                           ULONG lCurveClass,
                                           ULONG lCurveSolveInt,
                                           ULONG lCurveSolveMin,
                                           ULONG lCurveSolveMax,
                                           ULONG lCrvProjection,
                                           ULONG lCrvSection);

PT_EXPORT SmStatus my_test_trim_surfaces();

PT_EXPORT SmStatus my_test_MakeFace();

PT_EXPORT SmStatus my_test_brep_import();

#endif

