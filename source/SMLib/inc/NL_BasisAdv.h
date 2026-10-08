// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/********************************************************************************/
/* BasisAdv.h: Advanced Function Declarations that act on NL_KNOTVECTOR objects */
/********************************************************************************/

#ifndef _BASISADV_H
#define _BASISADV_H

/* find smallest & largest non-zero spans in knot vector */

GW_EXPORT NL_VOID N_BasisGetLongestAndShortestSpans( NL_KNOTVECTOR *, NL_DEGREE, NL_REAL *, NL_REAL *);

/* make a pair of knotvectors compatible after scaling to a common interval by merging them */
GW_EXPORT NL_VOID N_GetCompatibleKnotArrayMult( NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_DEGREE, NL_KNOTVECTOR *, NL_KNOTVECTOR * );

/* insert an array of knots into a knot vector */
GW_EXPORT NL_FLAG N_BasisInsertKnots( NL_KNOTVECTOR *, NL_DEGREE, NL_KNOTVECTOR *, NL_KNOTVECTOR * );

/* insert knot to mid-point of longest span */
GW_EXPORT NL_VOID N_BasisSplitLongestSpan( NL_KNOTVECTOR *, NL_DEGREE, NL_INTEGER );

/* insert mid-point knots into a given number of the longest curve spans */
GW_EXPORT NL_FLAG N_BasisSplitNLongestSpans( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_PARAMETER, NL_INDEX, NL_INDEX, NL_INDEX, NL_KNOTVECTOR * );

/* increase the multiplicity of all of a knotvector's internal knots */
GW_EXPORT NL_FLAG N_BasisIncreaseKnotMult( NL_KNOTVECTOR *, NL_DEGREE, NL_INDEX, NL_KNOTVECTOR * );

/* compute node corresponding to a given index */
GW_EXPORT NL_FLAG N_BasisFindIndexNode( NL_KNOTVECTOR *, NL_DEGREE, NL_INDEX, NL_PARAMETER * );

/* find the node span containing a given parameter */
GW_EXPORT NL_FLAG N_BasisFindNodeSpan( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_PARAMETER *, NL_PARAMETER *, NL_INDEX * );

/* computes a new parameter uh so that if uh is inserted into the knot vector, a given parameter u becomes a new node */
GW_EXPORT NL_FLAG N_BasisFindKnotToTurnParamIntoNode( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER, NL_PARAMETER * );

/* compute basis function values for a parameter array */
GW_EXPORT NL_FLAG N_BasisEvalArray( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER *, NL_INDEX, NL_FLAG, NL_REAL **, NL_INDEX * );

/* compute all non-vanishing basis functions and derivatives at given parameter values */
GW_EXPORT NL_FLAG N_BasisDerivsArray( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER *, NL_INDEX, NL_FLAG, NL_INDEX, NL_REAL ***, NL_INDEX * );

/* compute the derivative of one basis function with respect to a knot */
GW_EXPORT NL_FLAG N_BasisIKnotDeriv( NL_KNOTVECTOR *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );

/* compute derivatives of all non-vanishing basis functions for a knot */
GW_EXPORT NL_FLAG N_BasisKnotDerivs( NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );

/* compute bivariate derivatives of all non-vanishing basis functions for a knot */
GW_EXPORT NL_FLAG N_BiBasisKnotDeriv( NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_FLAG, NL_REAL **, NL_INDEX * );

#endif /* _BASISADV_H */
