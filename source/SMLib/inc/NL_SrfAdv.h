// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************/
/* SrfAdv.h: Advanced Function Declarations that act on NL_SURFACE objects */
/************************************************************************/

#ifndef _SRFADV_H
#define _SRFADV_H

/* advanced Nurbs routines */

GW_EXPORT NL_FLAG N_SrfEvalPtGrid( NL_SURFACE *, NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_INDEX, NL_FLAG, NL_FLAG, NL_POINT ** );
GW_EXPORT NL_VOID N_SrfScaleWeights( NL_SURFACE *, NL_REAL );

GW_EXPORT NL_FLAG N_SrfEvenSpacedPts( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, NL_INDEX, 
                                     NL_INDEX, NL_REAL, NL_POINT **, NL_PARAMETER *, NL_PARAMETER * );
GW_EXPORT NL_FLAG N_SrfExtendByDist( NL_SURFACE *, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfExtendToCrv( NL_SURFACE *, NL_CURVE *, NL_FLAG, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfDerivKnot( NL_SURFACE *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_FLAG, NL_POINT * );
GW_EXPORT NL_FLAG N_SrfModifyBoundaryCrv( NL_SURFACE *, NL_CURVE *, NL_FLAG, NL_FLAG, NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfEvalPtDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_INDEX, NL_INDEX, NL_CPOINT ** );
GW_EXPORT NL_FLAG N_SrfMaxSecondDeriv( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfEvalPtNormalDeriv( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT ** );
GW_EXPORT NL_FLAG N_SrfOffsetGetMaxDeriv( NL_SURFACE *, NL_REAL, NL_REAL *, NL_REAL *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfGetGridPtsNormals( NL_SURFACE *, NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_INDEX, NL_FLAG, 
                                         NL_FLAG, NL_POINT **, NL_VECTOR ** );
GW_EXPORT NL_FLAG N_SrfGetMaxSecondDeriv( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfOffsetGetFirstSecondDerivs( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_SFUN *, NL_SURFACE *, 
                                                  NL_FLAG, NL_FLAG, NL_POINT ** );
GW_EXPORT NL_FLAG N_SrfEvalPtDerivsUnbounded( NL_SURFACE *, NL_PARAMETER, NL_PARAMETER, NL_INDEX, NL_FLAG, NL_SURFACE *, 
                                             NL_FLAG *, NL_POINT *, NL_POINT **, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfExtendByParamDist( NL_SURFACE *, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

/* evaluate surface rational or non-rational basis functions corresponding to a given index */
GW_EXPORT NL_FLAG N_SrfBasisIEval( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );

/* compute the derivative of one bivariate rational basis function with respect to a knot */
GW_EXPORT NL_FLAG N_SrfRatBasisIKnotDeriv( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, 
                                          NL_FLAG, NL_FLAG, NL_FLAG, NL_REAL * );

GW_EXPORT NL_FLAG N_ReparmCrvsIsectPt( NL_CURVE **, NL_CURVE **, NL_INDEX, NL_INDEX, NL_PARAMETER **, NL_PARAMETER **, 
                                    NL_FLAG, NL_PARAMETER **, NL_PARAMETER **, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CreateSwungSrf( NL_CURVE *, NL_CURVE *, NL_REAL, NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CreateSkinSrf( NL_CURVE **, NL_INDEX, NL_FLAG, NL_FLAG, NL_DEGREE, NL_FLAG, NL_REAL, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CreateSkinSrfApprox( NL_CURVE **, NL_INDEX, NL_FLAG, NL_FLAG, NL_INDEX, NL_DEGREE, NL_FLAG, 
                                        NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSkinSrfParams( NL_CURVE **, NL_INDEX, NL_FLAG, NL_FLAG, NL_DEGREE, NL_FLAG, NL_PARAMETER *, 
                                        NL_KNOTVECTOR *, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSkinSrfApproxParams( NL_CURVE **, NL_INDEX, NL_FLAG, NL_FLAG, NL_INDEX, NL_DEGREE, NL_FLAG, 
                                              NL_PARAMETER *, NL_KNOTVECTOR *, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSkinSrfApproxTol( NL_CURVE **, NL_INDEX, NL_FLAG, NL_FLAG, NL_DEGREE, NL_DEGREE, NL_REAL, 
                                           NL_REAL, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_GetIsoCrvClosestCrv( NL_SURFACE *, NL_CURVE *, NL_FLAG, NL_PARAMETER, NL_REAL, NL_REAL, NL_PARAMETER *, 
                                        NL_INDEX, NL_REAL **, NL_INDEX, NL_PARAMETER *, NL_REAL * );
GW_EXPORT NL_FLAG N_CreateSkinSpine( NL_CURVE **, NL_INDEX, NL_FLAG, NL_CURVE *, NL_PARAMETER *, NL_FLAG, NL_VECTOR, 
                                    NL_INDEX, NL_FLAG, NL_DEGREE, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateTransSweepSrf( NL_CURVE *, NL_CURVE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateBVectors( NL_CURVE *, NL_VECTOR, NL_PARAMETER *, NL_INDEX, NL_VECTOR * );
GW_EXPORT NL_FLAG N_CreateSweepSrf( NL_CURVE *, NL_CURVE *, NL_INDEX, NL_VECTOR, NL_INDEX, NL_CURVE *, NL_DEGREE, NL_FLAG, 
                                   NL_REAL, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateGordonSrf( NL_CURVE **, NL_PARAMETER *, NL_INDEX, NL_CURVE **, NL_PARAMETER *, NL_INDEX, NL_DEGREE, 
                                    NL_DEGREE, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateCoonsSrf( NL_CURVE **, NL_CURVE **, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateCoonsSrfTwist( NL_CURVE **, NL_CURVE **, NL_VECTOR **, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateDataBoundaryDerivs( NL_SURFACE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSkinSrfBoundaryDerivs( NL_CURVE *, NL_VECTOR, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_MergeKnotVectors( NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_DEGREE, NL_REAL *, NL_INDEX *, NL_REAL *, NL_INDEX * );
GW_EXPORT NL_FLAG N_CreateSkinSrfBoundaryContinuity( NL_CURVE **, NL_INDEX, NL_SURFACE **, NL_FLAG, NL_FLAG *, NL_FLAG *, 
                                                    NL_CURVE **, NL_DEGREE, NL_REAL, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, 
                                                    NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSweepScale( NL_CURVE *, NL_CURVE *, NL_VECTOR, NL_CURVE *, NL_CURVE *, NL_FLAG, NL_PARAMETER, 
                                     NL_DEGREE, NL_FLAG, NL_REAL, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SweepCrossTangentCrv( NL_VECTOR *, NL_POINT *, NL_VECTOR *, NL_RMATRIX *, NL_RMATRIX *, NL_CURVE *, 
                                         NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_AdjustDerivSrf( NL_SURFACE *, NL_FLAG, NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_AdjustWeightScale( NL_CPOINT, NL_CPOINT, NL_CPOINT, NL_REAL *, NL_REAL * );
GW_EXPORT NL_FLAG N_ApproxCrossBoundaryDerivs( NL_CURVE *, NL_VECTOR, NL_VECTOR, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateCoonsBoundaryCrvs( NL_CURVE *, NL_CURVE *, NL_CURVE *, NL_CURVE *, NL_CURVE *, NL_CURVE *, 
                                            NL_CURVE *, NL_CURVE *, NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateCrossBoundaryDerivCrv( NL_CURVE *, NL_VECTOR, NL_VECTOR, NL_VECTOR, NL_VECTOR, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FillNSidedHole( NL_CURVE **, NL_CURVE **, NL_INDEX, NL_REAL, NL_POINT *, NL_VECTOR *, NL_FLAG, 
                                   NL_SURFACE ***, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_Get4CornerTwistVectors( NL_CURVE **, NL_CURVE **, NL_POINT *, NL_REAL *, NL_REAL *, NL_VECTOR ** );
GW_EXPORT NL_FLAG N_CrossBoundaryDerivCrv( NL_CURVE *, NL_CURVE *, NL_VECTOR *, NL_REAL *, NL_INDEX, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_Create4CrossBoundaryDerivCrvs( NL_CURVE **, NL_CURVE **, NL_POINT *, NL_INDEX, NL_REAL *, 
                                                  NL_REAL *, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_REAL *, NL_REAL *, 
                                                  NL_CURVE **, NL_CURVE **, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateTensorProductSrf( NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_POINT *, NL_INDEX, 
                                           NL_REAL *, NL_REAL *, NL_INDEX, NL_INDEX, NL_REAL *, NL_REAL *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateCoonsSrfCrossBoundaryDerivs( NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_SURFACE *, 
                                                      NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSkinSrfInterp( NL_CURVE **, NL_INDEX, NL_FLAG, NL_DEGREE, NL_DEGREE, NL_REAL *, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrossBoundaryDerivsVectorField( NL_CURVE *, NL_CURVE *, NL_FLAG, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateDerivField( NL_SURFACE *, NL_FLAG, NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_CURVE *, NL_STACKS * );

#endif /* _SRFADV_H */
