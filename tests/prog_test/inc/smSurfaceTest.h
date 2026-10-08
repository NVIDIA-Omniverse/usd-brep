// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smsurf_test.h
* PURPOSE --- Header file for surface tests.
*
**********************************************************************/
/*___*/

#ifndef __SMSURF_TEST_H__
#define __SMSURF_TEST_H__

#include <SmSurfTypes.h>
#include <SmCoreTypes.h>

PT_EXPORT SmStatus my_test_analytic_surfaces(void);
PT_EXPORT SmStatus my_test_bsplinesurface(void);
PT_EXPORT SmStatus my_test_csi_analy(void);
PT_EXPORT SmStatus my_test_csi_nurb(void);
PT_EXPORT SmStatus my_test_csi_tol(void);

PT_EXPORT SmStatus my_test_csi_tangency(void);
PT_EXPORT SmStatus my_test_csi_coincidence();
PT_EXPORT SmStatus my_test_cs_nurb(void);
PT_EXPORT SmStatus my_test_drop_curve();
PT_EXPORT SmStatus my_test_ss_nurb(void);

PT_EXPORT SmStatus my_test_surf_methods(void);
PT_EXPORT SmStatus my_test_sp_nurb(void);
PT_EXPORT SmStatus my_test_sp_analy(void);
PT_EXPORT SmStatus my_test_surface_suite(void);
PT_EXPORT SmStatus my_test_section(void);
PT_EXPORT SmStatus my_test_projection(void);
PT_EXPORT SmStatus my_test_surface_interface(void);

PT_EXPORT SmStatus my_test_surface_point_intersect(const SmBSplineSurface & crSurface, 
                                                   const SmExtent2d & crUVDomain,
                                                   long lNumU,    
                                                   long lNumV);

PT_EXPORT SmStatus my_test_surface_point_extrema(const SmBSplineSurface & crSurface, 
                                                 const SmExtent2d & crUVDomain,
                                                 SmSolverOperationType eSolverOperation,
                                                 long lNumX,
                                                 long lNumY,
                                                 long lNumZ,
                                                 ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_planar_section(const SmBSplineSurface & crSurface, 
                                          const SmExtent2d & crUVDomain,
                                          const SmVector3d & crPlaneNormal,
                                          ULONG lNumPlanes,
                                          ULONG & rlNumFound);

PT_EXPORT SmStatus my_test_GlobalCSI(const SmSurface & crSurface,      // in : target surface
                                     const SmCurve   & crCurve,        // in : target curve
                                     double            d3dTol,         // in : intersection tolerance
                                     ULONG           & rlTsectCount);   // out: number of intersection results 


SmStatus my_test_surface_closest_point(const SmBSplineSurface& crSurface, SmPoint3d& point );

#endif
