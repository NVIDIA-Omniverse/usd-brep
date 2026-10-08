// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smcurv_test.h
* PURPOSE --- Header file for test functions.
*
**********************************************************************/
/*___*/

#ifndef __SMCURV_TEST_H__
#define __SMCURV_TEST_H__

#ifndef __SMCURVE_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMPOINT3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

PT_EXPORT SmStatus my_test_analytic_curves(void);
PT_EXPORT SmStatus my_test_object(void);
PT_EXPORT SmStatus my_test_array(void);
PT_EXPORT SmStatus my_test_map(void);
PT_EXPORT SmStatus my_test_vector2d(void);
PT_EXPORT SmStatus my_test_vector3d(void);   

PT_EXPORT SmStatus my_test_axis2placement(void);
PT_EXPORT SmStatus my_test_extent1d(void);
PT_EXPORT SmStatus my_test_extent2d(void);
PT_EXPORT SmStatus my_test_extent3d(void);
PT_EXPORT SmStatus my_test_pseudobox(void);
PT_EXPORT SmStatus my_test_solution(void);

PT_EXPORT SmStatus my_test_bsplinecurve(void);
PT_EXPORT SmStatus my_test_bsplinecurve_nlib(void);
PT_EXPORT SmStatus my_test_bsplinesurface_nlib(void);
PT_EXPORT SmStatus my_test_matrix(void);
PT_EXPORT SmStatus my_test_cci_analy();

PT_EXPORT SmStatus my_test_cci_nurb(void);
PT_EXPORT SmStatus my_test_cci_tol(void);
PT_EXPORT SmStatus my_test_cci_tangent(void);
PT_EXPORT SmStatus my_test_cci_coincidence(void);
PT_EXPORT SmStatus my_test_cci_self(void);

PT_EXPORT SmStatus my_test_cci_projection(void);
PT_EXPORT SmStatus my_test_cc_nurb(void);
PT_EXPORT SmStatus my_test_cc_analy();
PT_EXPORT SmStatus my_test_methods(void);
PT_EXPORT SmStatus my_test_creation(void);

PT_EXPORT SmStatus my_test_cp_nurb(void);
PT_EXPORT SmStatus my_test_cp_analy(void);
PT_EXPORT SmStatus my_test_time(void);
PT_EXPORT SmStatus my_test_polynomial(void);

PT_EXPORT SmStatus my_test_curve_properties(void);
PT_EXPORT SmStatus my_test_crv_angle_min(void);
PT_EXPORT SmStatus my_test_crv_directed_max(void);    
PT_EXPORT SmStatus my_test_crv_directed_min(void);    
PT_EXPORT SmStatus my_test_crv_projected_max(void); 
PT_EXPORT SmStatus my_test_crv_projected_min(void); 
   
PT_EXPORT SmStatus my_test_crv_projected_tangency(void);    
PT_EXPORT SmStatus my_test_crv_signed_angle_min(void);        
PT_EXPORT SmStatus my_test_crv_signed_directed_min(void);
PT_EXPORT SmStatus my_test_signed_pivot_min(void);

PT_EXPORT SmBSplineCurve * my_create_circle(const SmContext & crContext, 
                                            double            dRadius, 
                                            const SmPoint3d & rCenter, 
                                            SmNurbCircleParam eParam,
                                            double dRed, double dGreen, double dBlue,
                                            SmBoolean         b2D=FALSE);

PT_EXPORT SmBSplineCurve * my_create_ellipse(const SmContext & crContext, 
                                             double            dRadius1, 
                                             double            dRadius2, 
                                             const SmPoint3d & rCenter, 
                                             SmNurbCircleParam eParam,
                                             double dRed, double dGreen, double dBlue,
                                             SmBoolean         b2D=FALSE);

PT_EXPORT SmBSplineCurve * my_create_nurb(const SmContext     & crContext,  
                                          const SmPoint3d     & rTranslate, 
                                          SmTArray<SmPoint3d> & rPoints, 
                                          ULONG                 lDegree,
                                          double dRed, double dGreen, double dBlue,
                                          SmBoolean             b2D=FALSE);

PT_EXPORT SmBSplineCurve * my_create_line(const SmContext & crContext,  
                                          const SmPoint3d & rStartPt, 
                                          const SmPoint3d & rEndPt, 
                                          double dRed, double dGreen, double dBlue,
                                          SmBoolean         b2D=FALSE);

PT_EXPORT SmStatus my_test_GlobalCC_intersect(SmCurve & rC1,
                                              SmCurve & rC2, 
                                              ULONG & rlCount);
#endif

