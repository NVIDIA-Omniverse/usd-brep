// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**************************************************************************/
/* BasisBsc.h: Basic Function Declarations that act on NL_KNOTVECTOR objects */
/**************************************************************************/

#ifndef _BASISBSC_H
#define _BASISBSC_H

GW_EXPORT NL_FLAG N_KnotVectorIsParamOutOfBounds( NL_KNOTVECTOR *, NL_PARAMETER, NL_STRING );
GW_EXPORT NL_FLAG N_KnotVectorIsEndParam( NL_KNOTVECTOR *, NL_PARAMETER, const TCHAR * );
GW_EXPORT NL_FLAG N_KnotVectorIsValid( NL_KNOTVECTOR *, NL_DEGREE, NL_STRING );

GW_EXPORT NL_KNOTVECTOR *N_AllocKnotVector( NL_STACKS * );
GW_EXPORT NL_KNOTVECTOR *N_AllocKnotVectorAndArray( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_KNOTVECTOR ** N_Alloc1dArrayKnotVectors( NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_KNOTVECTOR ** N_Alloc1dArrayKnotVectPtrs( NL_INDEX, NL_STACKS * );

GW_EXPORT NL_VOID N_KnotVectorGetKnots( NL_KNOTVECTOR *, NL_INDEX *, NL_REAL ** );

GW_EXPORT NL_VOID N_KnotVectorFromRealArray( NL_KNOTVECTOR *, NL_REAL *, NL_INDEX );
GW_EXPORT NL_VOID N_SetKnotIndex( NL_KNOTVECTOR *, NL_INDEX );

GW_EXPORT NL_VOID N_FreeKnotVector( NL_KNOTVECTOR *, NL_STACKS * );

GW_EXPORT NL_VOID N_KnotsPrint( NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_KnotsCopy( NL_KNOTVECTOR *, NL_KNOTVECTOR **, NL_STACKS * );
GW_EXPORT NL_FLAG N_KnotsCheck( NL_KNOTVECTOR **, NL_INDEX, NL_STRING, NL_STACKS * );
GW_EXPORT NL_FLAG N_KnotsRefine( NL_REAL *, NL_INDEX, NL_DEGREE, NL_REAL *, NL_INDEX, NL_INDEX, NL_INDEX, NL_REAL **, NL_INDEX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_KnotsAdd( NL_REAL *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_REAL **, NL_STACKS * );

/* find the knot span for a parameter */
GW_EXPORT NL_FLAG N_BasisFindSpan( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_INDEX * );

/* find the knot span and multiplicity for a parameter */
GW_EXPORT NL_FLAG N_BasisFindSpanAndMult( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_INDEX *, NL_INDEX * );

/* find the non-zero span count in a knot vector */
GW_EXPORT NL_VOID N_BasisGetSpanCount( NL_KNOTVECTOR *, NL_DEGREE, NL_INDEX * );

/* compute global maximum of a basis function */
GW_EXPORT NL_FLAG N_BasisFindGlobalMax( NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_REAL, NL_REAL *, NL_REAL * );

/* compute local basis function mins and maxs in each knot span */
GW_EXPORT NL_FLAG N_BasisFindAllSpanMaxima( NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_REAL, NL_REAL *, NL_REAL *, NL_PARAMETER * );

/* return all distinct knots and multiplicities of a given knot vector */
GW_EXPORT NL_FLAG N_BasisGetKnotsAndMults( NL_KNOTVECTOR *, NL_REAL **, NL_INDEX **, NL_INDEX *, NL_STACKS * );

/* scale a knot vector to a given  interval */
GW_EXPORT NL_VOID N_BasisReparam( NL_KNOTVECTOR *, NL_DEGREE, NL_INTERVAL );

/* make a pair of knotvectors compatible by merging them (with no scaling) */
GW_EXPORT NL_FLAG N_GetCompatibleKnotArray( NL_KNOTVECTOR **, NL_INDEX, NL_KNOTVECTOR ***, NL_STACKS * );

/* make a set of knotvectors compatible after scaling to a common interval by merging them */
GW_EXPORT NL_FLAG N_GetCompatibleKnotVector( NL_KNOTVECTOR **, NL_INDEX, NL_REAL, NL_KNOTVECTOR ***, NL_STACKS * );

/* make a set of knotvectors compatible after scaling to a common interval by merging them treating knots within a tolerance as the same */
GW_EXPORT NL_FLAG N_GetCompatibleKnotVectorToTol( NL_KNOTVECTOR **, NL_INDEX, NL_REAL, NL_KNOTVECTOR ***, NL_STACKS * );

/* compute the nodes of a given knot vector */
GW_EXPORT NL_VOID N_BasisFindIndexNodeArray( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER * );

/* compute all non-vanishing basis function values for a parameter value */
GW_EXPORT NL_FLAG N_BasisEval( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_REAL *, NL_INDEX * );

/* compute basis function values and derivatives for a parameter value */
GW_EXPORT NL_FLAG N_BasisDerivs( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_REAL **, NL_INDEX * );

/* compute one basis function value for a parameter value */
GW_EXPORT NL_FLAG N_BasisIEval( NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_REAL * );

/* compute one basis function value and its derivatives for a parameter value */
GW_EXPORT NL_FLAG N_BasisIDerivs( NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_REAL * );

GW_EXPORT NL_VOID N_MakeKnotsCompatible( NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_DEGREE, NL_DEGREE, NL_KNOTVECTOR *, NL_KNOTVECTOR * );

#endif /* _BASISBSC_H */
