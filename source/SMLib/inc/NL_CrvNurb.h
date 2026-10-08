// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***********************************************************************/
/* CrvNurb.h: NURB Function Declarations that act on NL_CURVE objects  */
/***********************************************************************/

#ifndef _CRVNURB_H
#define _CRVNURB_H

/* evaluate all non-zero curve rational or non-rational basis functions and derivatives for a parameter */

GW_EXPORT NL_FLAG N_CrvBasisDerivs( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_REAL **, NL_INDEX * );

/* compute all non-zero univariate rational basis function values for a parameter value */
GW_EXPORT NL_FLAG N_CrvRatBasisDerivs( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_REAL **, NL_INDEX * );

/* compute one univariate rational basis function value for a parameter value */
GW_EXPORT NL_FLAG N_CrvRatBasisIEval( NL_CURVE *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_REAL * );

/* compute one univariate rational basis function value and derivatives for a parameter value */
GW_EXPORT NL_FLAG N_CrvRatBasisIDerivs( NL_CURVE *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_REAL * );

/* Compute the Greville abscissa correcponding to a control point */
GW_EXPORT NL_FLAG N_CrvGetGrevilleAbscissa( NL_REAL *pdKnotArray, NL_INDEX iKnotCount, NL_DEGREE iDegree, NL_INDEX idx, NL_REAL *pdGrevilleAbs );

GW_EXPORT NL_FLAG N_CrvEvalCurvature( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_REAL *, NL_POINT *, NL_VECTOR *, NL_FLAG, NL_POINT *, NL_VECTOR * );
GW_EXPORT NL_FLAG N_CrvEval( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvGetBBox( NL_CURVE *, NL_MINMAXBOX * );
GW_EXPORT NL_FLAG N_CrvDerivs( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_POINT * );
GW_EXPORT NL_FLAG N_CrvEvalTangent( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_POINT *, NL_VECTOR * );

GW_EXPORT NL_FLAG N_CrvEvalFrenetFrame( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_POINT *, NL_VECTOR *, NL_VECTOR *, NL_VECTOR * );
GW_EXPORT NL_VOID N_CrvTransform( NL_CURVE *, NL_RMATRIX * );
GW_EXPORT NL_VOID N_CrvGetMinMaxWeightsAndPts( NL_CURVE *, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL * );
GW_EXPORT NL_VOID N_CrvUnclamp( NL_CURVE * );
GW_EXPORT NL_VOID N_CrvUnclampKnots( NL_CURVE *, NL_KNOTVECTOR * );

GW_EXPORT NL_FLAG N_CrvDegenFromPt( NL_POINT, NL_INDEX, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvLineFromPtAndVector( NL_POINT, NL_VECTOR, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_VOID N_CrvScale( NL_CURVE *, NL_POINT, NL_VECTOR );
GW_EXPORT NL_VOID N_CrvScaleCPts( NL_CURVE *, NL_REAL );
GW_EXPORT NL_VOID N_CrvTranslate( NL_CURVE *, NL_VECTOR );
GW_EXPORT NL_FLAG N_CrvRotateAboutAxis( NL_CURVE *, NL_POINT, NL_VECTOR, NL_REAL );

GW_EXPORT NL_FLAG N_CrvProjectOntoPlane( NL_CURVE *, NL_POINT, NL_VECTOR, NL_VECTOR, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvReverse( NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvGetBBoxMaxDiagDist( NL_CURVE *, NL_REAL * );
GW_EXPORT NL_VOID N_CrvGetMaxPosVector( NL_CURVE *, NL_POINT *, NL_REAL * );
GW_EXPORT NL_VOID N_CrvGetMinPosVector( NL_CURVE *, NL_POINT *, NL_REAL * );

GW_EXPORT NL_FLAG N_CrvGetG1Segs( NL_CURVE *, NL_REAL, NL_REAL ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvExtendToPtLargeExtensions( NL_CURVE *, NL_POINT, NL_FLAG, NL_REAL, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvGetLinearSegs( NL_CURVE *, NL_REAL, NL_REAL ***, NL_FLAG **, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvGetDegenSegs( NL_CURVE *, NL_REAL, NL_REAL ***, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_VOID N_CrvReparam( NL_CURVE *, NL_PARAMETER, NL_PARAMETER );

GW_EXPORT NL_FLAG N_CrvGetType( NL_CURVE *, NL_REAL, NL_POINT *, NL_VECTOR *, NL_VECTOR *, NL_VECTOR *, NL_REAL *, NL_REAL *, NL_FLAG * );
GW_EXPORT NL_FLAG N_CrvGetCurvatureDeriv( NL_CURVE *, NL_PARAMETER, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvIsClosedContinuity( NL_CURVE *, NL_INDEX, NL_REAL *, NL_FLAG * );
GW_EXPORT NL_FLAG N_CrvLineFrom2Pts( NL_POINT, NL_POINT, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFromCrvTranslation( NL_CURVE *, NL_VECTOR, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_PntCrvFitError( NL_POINT *, NL_INDEX, NL_CURVE *, NL_REAL *, NL_REAL *, NL_REAL *, NL_INDEX * );
GW_EXPORT NL_FLAG N_PntSrfFitError( NL_POINT *, NL_INDEX, NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL * );

GW_EXPORT NL_FLAG N_CrvFitToPt(NL_CURVE *, // in : TgtCrv to edit  
                               NL_POINT,   // in : TgtPt to interpolate  
                               NL_BOOLEAN, // in : TRUE  = use FixParam for PtOnCurve to fit to Point  
                                           //      FALSE = use CrvParam of input CrvPt closest to TgtPt  
                               NL_REAL,    // in : if useFixParam == TRUE, CrvParamPt to fit to Point, else not used  
                               NL_BOOLEAN, // in : TRUE  = use guessParam as NR closest point guess for finding the closest point (runs locally and faster)  
                                           //      FALSE = find closest point without an initial closest point guess (runs globally and slower)  
                               NL_REAL );  // in : if(useFixParam == FALSE && useGuessParam == TRUE) the guess point passed to NR to find closest point on Curve  
                                           //      else not used 
                                           
GW_EXPORT NL_FLAG N_FitCrvApproxPts( NL_POINT *, NL_INDEX, NL_CURVE *, NL_INDEX, NL_INDEX, NL_INDEX *, NL_PARAMETER *, NL_FLAG, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL *, NL_STACKS * );

#endif /* _CRVNURB_H */
