// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvAdv.h: Advanced Function Declarations that act on NL_CURVE objects */
/**********************************************************************/

#ifndef _CRVADV_H
#define _CRVADV_H

/* advanced curve Nurbs functions */

GW_EXPORT NL_FLAG N_CrvEvalPt( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_CPOINT * );
GW_EXPORT NL_FLAG N_CrvDerivsAtKnot( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_CPOINT * );
GW_EXPORT NL_FLAG N_CrvAlign( NL_CURVE *, NL_PARAMETER, NL_POINT, NL_VECTOR, NL_VECTOR, NL_VECTOR );
GW_EXPORT NL_FLAG N_CrvEvalEvenSpacedPts( NL_CURVE *, NL_PARAMETER, NL_PARAMETER, NL_INDEX, NL_REAL, NL_POINT *, NL_PARAMETER * );

GW_EXPORT NL_FLAG N_CrvExtendByDist( NL_CURVE *, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvExtendToPt( NL_CURVE *, NL_POINT, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvModifyEndPt( NL_CURVE *, NL_POINT, NL_FLAG, NL_REAL, NL_FLAG, NL_POINT, NL_FLAG, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvEvalDerivAtKnot( NL_CURVE *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvGetMax2ndDeriv( NL_CURVE *, NL_REAL * );

GW_EXPORT NL_FLAG N_CrvPlanarOffsetGetDeriv( NL_CURVE *, NL_PARAMETER, NL_REAL, NL_VECTOR, NL_FLAG, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvOffsetGetMax2ndDeriv( NL_CURVE *, NL_REAL, NL_VECTOR, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvOffsetGetDeriv( NL_CURVE *, NL_PARAMETER, NL_CFUN *, NL_CURVE *, NL_FLAG, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvExtendByParamDist( NL_CURVE *, NL_REAL, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvEvalUnboundedPtsAndDerivs( NL_CURVE *, NL_PARAMETER, NL_INDEX, NL_FLAG, NL_CURVE *, NL_FLAG *, NL_POINT *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvGetAveragePosMag( NL_CURVE *, NL_REAL * );

/* evaluate curve rational or non-rational basis functions corresponding to a given index */
GW_EXPORT NL_FLAG N_CrvBasisIEval( NL_CURVE *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_REAL * );

/* compute derivatives of all non-vanishing rational basis functions for a knot */
GW_EXPORT NL_FLAG N_CrvRatBasisKnotDeriv( NL_CURVE *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );

/* compute the derivative of one univariate rational basis function with respect to a knot */
GW_EXPORT NL_FLAG N_CrvRatBasisIKnotDeriv( NL_CURVE *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );

GW_EXPORT NL_FLAG N_CrvRemoveAllKnotsArcLen( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveAllKnots( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveAllKnotsConstraints( NL_CURVE *, NL_PARAMETER *, NL_INDEX, NL_FLAG *, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvsMakeCompatibleApprox( NL_CURVE **, NL_INDEX, NL_REAL, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvsMakeCompatibleKnotRemove( NL_CURVE **, NL_INDEX, NL_REAL, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvsMakeCompatibleConstraints( NL_CURVE **, NL_INDEX, NL_REAL, NL_FLAG, NL_FLAG, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvsMakeCompatibleFast( NL_CURVE **, NL_INDEX, NL_REAL, NL_FLAG, NL_FLAG, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveKnotsTangentConstraints( NL_CURVE *, NL_REAL, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRemoveKnotsDerivConstraints( NL_CURVE *, NL_REAL, NL_FLAG, NL_INDEX, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvMakeCompatibleWithSrf( NL_CURVE *, NL_SURFACE *, NL_FLAG, NL_PARAMETER, NL_PARAMETER, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRefineToKnotVector( NL_CURVE *, NL_REAL *, NL_INDEX, NL_CPOINT * );

GW_EXPORT NL_FLAG N_CrvGetMaxFirstDeriv( NL_CURVE *, NL_POINT *, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvGetMaxSecondDeriv( NL_CURVE *, NL_POINT *, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvGetRatSecondDeriv( NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvDotCrv( NL_CURVE *, NL_CURVE *, NL_CFUN *, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvSumDiffCrv( NL_CURVE *, NL_CURVE *, NL_FLAG, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvCrossMultiplyCrv( NL_CURVE *, NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvSumDiffVector( NL_CURVE *, NL_VECTOR, NL_FLAG );
GW_EXPORT NL_FLAG N_CrvNonRatGetFirstDeriv( NL_CURVE *, NL_INDEX, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvRatGetFirstDeriv( NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvNonRatGetDerivCrvsAll( NL_CURVE *, NL_INDEX, NL_CURVE ***, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvGetDerivCrvsAll( NL_CURVE *, NL_INDEX, NL_CURVE ***, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvMakeDerivCrv( NL_CURVE *, NL_INDEX, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvDiffCrvGetMaxChange( NL_CURVE *, NL_CURVE *, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvMoveKnotGetMaxChange( NL_CURVE *, NL_INDEX, NL_REAL, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvNonRatEvalDeriv( NL_CURVE *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_CrvEvalHighDerivsKnot( NL_CURVE *, NL_INDEX, NL_FLAG, NL_INDEX, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvEvalFirstDerivKnot( NL_CURVE *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvEvalSecondDerivKnot( NL_CURVE *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_VOID N_ConstantMultiplyCrv( NL_REAL, NL_CURVE * );
GW_EXPORT NL_VOID N_LinearMultiplyCrv( NL_REAL, NL_REAL, NL_CURVE * );
GW_EXPORT NL_FLAG N_CrvCombine( NL_REAL, NL_CURVE *, NL_REAL, NL_CURVE *, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_VOID N_ConstantMultiplyCrv4d( NL_REAL, NL_CURVE * );

GW_EXPORT NL_FLAG N_CrvReparamRat( NL_CURVE *, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvReparamWeights( NL_CURVE *, NL_REAL, NL_REAL, NL_REAL );
GW_EXPORT NL_FLAG N_SrfReparamFunc( NL_CURVE *, NL_CFUN *, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfReparamMultKnots( NL_CURVE *, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfReparamWeights( NL_CURVE *, NL_REAL, NL_REAL );

#endif /* _CRVADV_H */
