// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*************************************************************************/
/* SrfNurb.h: Basic Function Declarations that act on NL_SURFACE objects */
/*************************************************************************/

#ifndef _SRFNURB_H
#define _SRFNURB_H

/******************/
/* Nurbs routines */
/******************/

/* evaluate all non-zero bivariate rational or non-rational basis functions and derivatives for a parameter */

GW_EXPORT NL_FLAG N_SrfBasisDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_INDEX, 
                                    NL_INDEX, NL_BASISDERIVATIVES&, NL_INDEX *, NL_INDEX * );

/* compute all non-zero bivariate non-rational basis function values and derivatives for a parameter value */
GW_EXPORT NL_FLAG N_SrfNonRatBasisDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_INDEX, 
                                          NL_INDEX, NL_BASISDERIVATIVES&, NL_INDEX *, NL_INDEX * );

/* compute all non-zero bivariate rational basis function values and derivatives for a parameter value */
GW_EXPORT NL_FLAG N_SrfRatBasisDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_INDEX, 
                                       NL_INDEX, NL_BASISDERIVATIVES&, NL_INDEX *, NL_INDEX * );

/* compute one bivariate rational basis function value for a parameter value */
GW_EXPORT NL_FLAG N_SrfRatBasisIEval( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );

/* compute one bivariate rational basis function value and derivatives for a parameter value */
GW_EXPORT NL_FLAG N_SrfRatBasisIDerivs( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, 
                                        NL_INDEX, NL_INDEX, NL_REAL ** );

/* compute surface derivatives of all non-vanishing bivariate rational basis functions for a knot */
GW_EXPORT NL_FLAG N_SrfRatBasisKnotDeriv( NL_SURFACE *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, 
                                          NL_FLAG, NL_REAL **, NL_INDEX * );

/* Surfaces */

GW_EXPORT NL_FLAG N_SrfEvalPtCurvature( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_REAL *, 
                                        NL_REAL *, NL_POINT *, NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_REAL *, NL_REAL *, 
                                        NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_FLAG * );
GW_EXPORT NL_FLAG N_SrfEvalPt( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT * );
GW_EXPORT NL_FLAG N_SrfEvalPtCrvOnSrf( NL_SURFACE *, NL_CURVE *, NL_PARAMETER, NL_POINT * );
GW_EXPORT NL_FLAG N_SrfDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_INDEX, NL_INDEX, NL_POINT ** );
GW_EXPORT NL_FLAG N_SrfGetBBox( NL_SURFACE *, NL_MINMAXBOX * );
GW_EXPORT NL_VOID N_SrfMaxMagnitudePosVectors( NL_SURFACE *, NL_POINT *, NL_REAL * );
GW_EXPORT NL_VOID N_SrfTransform( NL_SURFACE *, NL_RMATRIX * );
GW_EXPORT NL_VOID N_SrfMinMaxWeightPosVectors( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfEvalPtPtDerivNormal( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT *, 
                                            NL_VECTOR *, NL_VECTOR *, NL_VECTOR * );
GW_EXPORT NL_FLAG N_SrfEvalPtPtDerivNormalFast( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT *, 
                                                NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_POINT ** );
GW_EXPORT NL_VOID N_SrfUnclamp( NL_SURFACE *, NL_FLAG );
GW_EXPORT NL_VOID N_SrfUnclampKnotVector( NL_SURFACE *, NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_FLAG );
GW_EXPORT NL_VOID N_SrfScale( NL_SURFACE *, NL_POINT, NL_VECTOR );
GW_EXPORT NL_VOID N_SrfTranslate( NL_SURFACE *, NL_VECTOR );
GW_EXPORT NL_FLAG N_SrfRotateAtPt( NL_SURFACE *, NL_POINT, NL_VECTOR, NL_REAL );
GW_EXPORT NL_FLAG N_SrfProjectOntoPlane( NL_SURFACE *, NL_POINT, NL_VECTOR, NL_VECTOR, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfReverse( NL_SURFACE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfLargeExtendToCrv( NL_SURFACE *, NL_CURVE *, NL_FLAG, NL_FLAG, NL_REAL, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfType( NL_SURFACE *, NL_REAL, NL_CURVE **, NL_CURVE **, NL_POINT *, NL_POINT *, NL_POINT *, NL_POINT *, 
                             NL_VECTOR *, NL_VECTOR *, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL *, NL_FLAG *, NL_FLAG *, NL_STACKS * );
GW_EXPORT NL_VOID N_SrfReparam( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER );
GW_EXPORT NL_FLAG N_SrfFindDegenPatch( NL_SURFACE *, NL_REAL, NL_FLAG, NL_REAL ***, NL_INDEX *, NL_REAL ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfIsClosedSmooth( NL_SURFACE *, NL_INDEX, NL_REAL *, NL_FLAG, NL_FLAG * );
GW_EXPORT NL_FLAG N_SrfEvalPtPtDerivNormalPole( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT *, 
                                                NL_VECTOR *, NL_VECTOR *, NL_VECTOR * );
GW_EXPORT NL_FLAG N_SrfEvalPtNormalAtPole( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT *, NL_VECTOR *, 
                                           NL_VECTOR *, NL_VECTOR *, NL_POINT ** );
GW_EXPORT NL_FLAG N_SrfMaxDiagDistBBox( NL_SURFACE *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfEvalPtDerivCPt( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_VECTOR *, NL_VECTOR * );
GW_EXPORT NL_FLAG N_ConeEllipticEnds( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_SURFACE *, NL_STACKS * );

#endif /* _SRFNURB_H */
