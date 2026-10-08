// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvNurb.c : NURB Function Definitions that act on NL_CURVE objects */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/* are the curve end points allowed to move?  */
static NL_FLAG ST_findStep(NL_CURVE* crv, NL_REAL BetaMat[4][4], NL_FLAG fixEndPts, NL_INDEX numDataPoints, NL_POINT xPts[],
    NL_REAL xParams[], /* must be initialized to previous-step values */
    NL_REAL alpha, NL_REAL beta, NL_VECTOR ctrlPtMoves[], NL_REAL* errorDist,
    NL_REAL* errorSD, NL_REAL* allErrors, NL_GCPTEMP* crvData, NL_STACKS* pocStacks);

static NL_BOOLEAN ST_doneIterating(NL_INDEX iter, NL_INDEX maxIter, NL_REAL errorSD, NL_REAL errorDist, NL_REAL errorStop);

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine  evaluates all non-vanishing rational  or non rational 
     curve basis functions and their derivatives. It is assumed that the 
     end knots  are  repeated with  multiplicity = degree + 1. A typical 
     calling example is:

       NL_CURVE      cur;
       NL_INDEX      der, spn;
       NL_PARAMETER  u;
       NL_REAL       **ND;
       ...
       (define cur, get u and der);
       ...
       N_CrvBasisDerivs(&cur,u,NL_LEFT,der,ND,&spn);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     der , input  ,  Highest derivatives required
     ND  , output ,  Derivatives  computed at u.  ND[k][i] is  the  k-th 
                     derivative of the basis function  N[j-p+i], where u 
                     is in  {u[j],u[j+1]}. MEMORY FOR ND IS ALLOCATED IN
                     THE CALLING ROUTINE!
     spn , output ,  Index of knot span u is in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvBasisDerivs( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_REAL ** ND, NL_INDEX *spn )
{
    NL_FLAG error = NL_NO;
    NL_DEGREE p;
    NL_KNOTVECTOR *knt;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Compute basis functions */
    if( N_IsCrvRat( cur ) )
    {
        error = N_CrvRatBasisDerivs( cur, u, flg, der, ND, spn );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        N_CrvGetKnotVector( cur, &knt );
        N_CrvGetDegree( cur, &p );

        error = N_BasisDerivs( knt, p, u, flg, der, ND, spn );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */
    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvBasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  all  non-vanishing univariate rational  
     basis  functions  and  their  derivatives  at a  given parameter 
     value. It  is  assumed  that  the knot vector is  clamped, i.e., 
     end knots are repeated with multiplicity = degree + 1. A typical
     calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_INDEX      der, spn;
       NL_REAL       **RD;
       ...
       (define cur, get u and der, and allocate memory for RD);
       ...
       N_CrvRatBasisDerivs(&cur,u,NL_LEFT,der,RD,&spn);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES REQUIRED)
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES REQUIRED) 
     der , input  ,  Highest derivative required
     RD  , output ,  Derivatives  computed  at u.  RD[k][i]  is  the 
                     k-th derivative of the basis function R[j-p+i], 
                     where u is in {u[j],u[j+1]}. MEMORY FOR RD MUST 
                       NL_PRIVATE  NL_STRING  rname = "N_CrvRatBasisDerivs");
BE ALLOCATED IN THE CALLING ROUTINE!
     spn , output ,  Index of knot span u is in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRatBasisDerivs( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_REAL ** RD, NL_INDEX *spn )
{

    NL_PRIVATE NL_STRING rname = _T("N_CrvRatBasisDerivs");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, k, nsp;
    /*        NL_INTEGER     **bin;    */
    NL_REAL ** ND, *d, v, w;
    NL_DEGREE p;
    NL_KNOTVECTOR *knt;
    NL_CPOINT *Pw;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get local notation */
    N_CrvGetCPts( cur, &i, &Pw );
    N_CrvGetKnotVector( cur, &knt );
    N_CrvGetDegree( cur, &p );

    /* Check parameter and order of derivative */
    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( der GT NL_MAXDER )
    {
        N_ErrSet( NL_MXD_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Get derivatives of all basis functions */

    ND = N_AllocReal2dArray( der, p, &S );

    if( ND EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knt, p, u, flg, der, ND, &nsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get derivatives of the denominator */
    d = N_AllocReal1dArray( der, &S );

    if( d EQ NULL )
        NL_QUIT;

    for ( k = 0; k <= der; k++ )
    {
        d[k] = 0.0;

        for ( j = nsp - p; j <= nsp; j++ )
        {
            N_CPtGetW( Pw[j], &w );
            d[k] += w * ND[k][j - nsp + p];
        }
    }

    /* Compute derivatives of rational basis function */

    /*        bin = N_AllocInt2dArray(der,der,&S);    */
    /*        if( bin EQ NULL )  NL_QUIT;       */
    /*                                       */
    /*        N_PascalTriRow(bin,der);             */

    for ( i = 0; i <= p; i++ )
    {
        N_CPtGetW( Pw[nsp - p + i], &w );

        for ( k = 0; k <= der; k++ )
        {
            v = w * ND[k][i];

            for ( j = 1; j <= k; j++ )
            {
                v -= NL_PascalTri[k][j] * d[j] * RD[k - j][i];
            }
            RD[k][i] = v / d[0];
        }
    }

    *spn = nsp;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvRatBasisDerivs */

#endif // NLIB_UNUSED

/*******************************************************************//**

   DESCRIPTION:

     This Nurbs curve routine evaluates the Greville abscissa at a
     given control point.  The Greville abcissa is the parameter
     value that most closely corresponds to a given control point.
     A typical calling example is:

     NL_INDEX  idx = < control point index >
     NL_REAL   *pdKnotArray;
     NL_INDEX  iKnotCount;
     NL_DEGREE iDegree;
     NL_REAL   pdGrevilleAbs;
     ...
     (Get knotArray, knotCount, and degree, perhaps from
     N_CrvGetKnots() and N_CrvGetDegree().)
     ...

     N_CrvGetGrevilleAbscissa( knotArray, knotCount, degree, idx, &dGrevilleAbs );

   ACCESS:
   
     pdKnotArray   , input , knot vector
     iKnotCount    , input , highest index in pdKnotArray
     iDegree       , input , degree of B-Spline
     idx           , input , Index of control point
     pdGrevilleAbs , output, parameter value at control point [idx]

   RETURN CODES:

     NL_NO  : No error
     NL_YES : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetGrevilleAbscissa( NL_REAL *pdKnotArray, NL_INDEX iKnotCount, NL_DEGREE iDegree, NL_INDEX idx, NL_REAL *pdGrevilleAbs )
{
  NL_PRIVATE NL_STRING rname = _T("N_CrvGetGrevilleAbscissa");
  NL_INDEX ii;
  NL_FLAG error = NL_NO;
  *pdGrevilleAbs = 0.0;

  if ( iDegree < 1  OR  idx < 0   ) { NL_ERROR( NL_IND_ERR ); }
  if ( idx + iDegree > iKnotCount ) { NL_ERROR( NL_IND_ERR ); }

  /* The Grevilla abscissa is the average of 'degree' knot values. */
  for ( ii = 1; ii <= iDegree; ii++ )
  {
      *pdGrevilleAbs += pdKnotArray[ idx + ii ];
  }
  *pdGrevilleAbs /= iDegree;

  EXIT:

  return error;

} /* end N_CrvGetGrevilleAbscissa */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  routine evaluates a  univariate rational  basis function 
     at a given parameter value. It is assumed that the knot vector 
     is  clamped, i.e., end  knots are repeated with multiplicity = 
     degree + 1. A typical calling example is:

       NL_CURVE      cur;
       NL_INDEX      i;
       NL_PARAMETER  u;
       NL_REAL       R;
       ...
       (define cur, get i and u);
       ...
       N_CrvRatBasisIEval(&cur,i,u,NL_LEFT,&R);


   ACCESS:
   
     cur , input  ,  NURBS curve
     i   , input  ,  Index of rational basis function (0<=i<=n)
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     R   , output ,  Basis function computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRatBasisIEval( NL_CURVE *cur, NL_INDEX i, NL_PARAMETER u, NL_FLAG flg, NL_REAL *R )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRatBasisIEval");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    NL_REAL *U, *w, den, N;

    NL_DEGREE p;

    NL_KNOTVECTOR *knt;

    NL_CFUN cfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetKnotVector( cur, &knt );
    N_CrvGetDegree( cur, &p );
    N_KnotVectorGetKnots( knt, &m, &U );

    n = m - p - 1;

    /* Check parameter and index */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT n )
        NL_ERROR( NL_IND_ERR );

    /* Special cases for end values */

    if( u EQ U[p] )
    {
        if( i EQ 0 )
            *R = 1.0;
        else
            *R = 0.0;
        NL_OUT;
    }

    if( u EQ U[m - p] )
    {
        if( i EQ n )
            *R = 1.0;
        else
            *R = 0.0;
        NL_OUT;
    }

    /* Check if u is outside the appropriate knot span */

    switch( flg )
    {
        case NL_LEFT:
            if( u LT U[i]OR u GE U[i + p + 1] )
            {
                *R = 0.0;
                NL_OUT;
            }
            break;

        case NL_RIGHT:
            if( u LE U[i]OR u GT U[i + p + 1] )
            {
                *R = 0.0;
                NL_OUT;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Extract denominator */

    N_CFuncInitArrays( &cfn );
    error = N_CrvGetDenomCrvFunc( cur, &cfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( &cfn, &w, &U );

    /* Compute the i-th B-spline */

    error = N_BasisIEval( knt, i, p, u, flg, &N );

    if( error EQ NL_YES )
        NL_OUT;

    /* Evaluate the denominator */

    error = N_CFuncEval( &cfn, u, flg, &den );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute rational basis */

    *R = w[i] * N / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvRatBasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates a  univariate rational  basis function 
     and its  derivatives at a  given parameter value. It is assumed 
     that the  knot vector is  clamped, i.e., end knots are repeated 
     with multiplicity = degree + 1. A typical calling example is:

       NL_CURVE      cur;
       NL_INDEX      i, der;
       NL_PARAMETER  u;
       NL_REAL       *RD;
       ...
       (define cur, get i, der, and allocate memory for RD);
       ...
       N_CrvRatBasisIDerivs(&cur,i,u,NL_LEFT,der,RD);
       

   ACCESS:
   
     cur , input  ,  NURBS curve
     i   , input  ,  Index of rational basis function (0<=i<=n)
     u   , input  ,  Parameter value
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES REQUIRED)
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES REQUIRED) 
     der , input  ,  Highest derivative required
     RD  , output ,  Derivatives  computed  at u; RD[k]  is the k-th
                     derivative. MEMORY  FOR RD MUST BE ALLOCATED IN
                     THE CALLING ROUTINE.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvRatBasisIDerivs */
NL_FLAG N_CrvRatBasisIDerivs( NL_CURVE *cur, NL_INDEX i, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_REAL *RD )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRatBasisIDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX n, j, k;

    NL_REAL *A, *d, *w, v;

    NL_DEGREE p;

    NL_KNOTVECTOR *knt;

    NL_CFUN cfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetArraySizes( cur, &n, &j );
    N_CrvGetDegree( cur, &p );
    N_CrvGetKnotVector( cur, &knt );

    /* Check parameter and index */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT n )
        NL_ERROR( NL_IND_ERR );

    /* Extract denominator */

    N_CFuncInitArrays( &cfn );
    error = N_CrvGetDenomCrvFunc( cur, &cfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( &cfn, &w, &A );

    /* Get derivatives of numerator */

    A = N_AllocReal1dArray( der, &S );

    if( A EQ NULL )
        NL_QUIT;

    error = N_BasisIDerivs( knt, i, p, u, flg, der, A );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get derivatives of denominator */

    d = N_AllocReal1dArray( der, &S );

    if( d EQ NULL )
        NL_QUIT;

    error = N_CFuncDerivs( &cfn, u, flg, der, d );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives of rational basis */

    /* row = N_AllocInt1dArray(der,&S);   */
    /* if( row EQ NULL )  NL_QUIT;  */

    for ( k = 0; k <= der; k++ )
    {
        v = w[i] * A[k];
        /* N_PascalTriIndex(row,k);  gwc - replaced pascal tri computation with a global array */

        for ( j = 1; j <= k; j++ )
        {
            /* v -= row[j]*d[j]*RD[k-j]; */
            v -= NL_PascalTri[k][j] * d[j] * RD[k - j];
        }
        RD[k] = v / d[0];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvRatBasisIDerivs */

/* Curves */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes  the curvature and  optionally the osculating 
     circle of a  curve at a given  parameter. Discontinuous  curves can  
     be handled by passing a NL_LEFT/NL_RIGHT flag. A typical calling example:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_REAL       kap;
       NL_POINT      P, C;
       NL_VECTOR     T, N;
       ...
       (define cur and get u);
       ...
       N_CrvEvalCurvature(&cur,u,NL_LEFT,&kap,&P,&T,NL_YES,&C,&N);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     ufl , input  ,  Flag:
                       NL_LEFT : u is in  [u[j],u[j+1]) (NL_RIGHT  derivatives
                              used)
                       NL_RIGHT: u is  in  (u[j],u[j+1]] (NL_LEFT  derivatives
                              used)
     kap , output ,  Curvature at u. The radius of the osculating circle
                     is rad = 1/kap.
     P   , output ,  Point on the curve at u
     T   , output ,  Unit tangent at u
     cfl , input  ,  Flag:
                       NL_YES: compute osculating circle
                       NL_NO : no osculating circle is needed
     C   , output ,  Center of osculating circle
     N   , output ,  Normal to the plane of the osculating circle


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalCurvature( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG ufl, NL_REAL *kap, NL_POINT *P, NL_VECTOR *T, NL_FLAG cfl, NL_POINT *C, NL_VECTOR *N )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvEvalCurvature");

    NL_FLAG error = NL_NO;

    NL_REAL num, den, rad;

    NL_POINT D[3];

    NL_VECTOR A, B;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get derivatives */

    error = N_CrvDerivs( cur, u, ufl, 2, D );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get curvature */

    N_VectorCross( D[1], D[2], &A );
    N_VectorMagnitude( A, &num );
    N_VectorMagnitude( D[1], &den );

    den = den * den * den;

    if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    *kap = num / den;

    /* Get osculating circle */

    if( cfl EQ NL_YES )
    {
        if( N_FloatOpIsBad( 1.0, *kap, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        rad = 1.0 / (*kap);

        error = N_CrvEvalFrenetFrame( cur, u, ufl, P, T, &B, N );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorPtAlongVector( D[0], rad, B, C );
    }
    else
    {
        N_CopyPt( D[0], P );

        error = N_VectorNormalize( D[1], T, &num );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvEvalCurvature */


/*******************************************************************//**


   DESCRIPTION:

     This routine  computes a point on a NURBS curve by evaluating all
     non-vanishing basis functions and multiplying them by appropriate
     control  points.  Discontinuous  curves can  also be evaluated by
     passing a NL_LEFT/NL_RIGHT flag. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_POINT      C;
       ...
       (define cur and get u);
       ...
       N_CrvEval(&cur,u,NL_LEFT,&C);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     C   , output ,  Point on the curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEval( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_POINT *C )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvEval");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, spn;

    NL_DEGREE p;

    NL_REAL alpha, N[NL_MAXDEG + 1];

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, Cw;

    /* Get local notation */

    knt = cur->knt;
    Pw = cur->pol->Pw;
    p = cur->p;

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines */

    error = N_BasisEval( knt, p, u, flg, N, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the point on the curve */

    Cw.x = 0.0;
    Cw.y = 0.0;
    Cw.z = 0.0;
    Cw.w = 0.0;

    j = spn - p;

    for ( i = j; i <= spn; i++ )
    {
        alpha = N[i - j];
        Cw.x = Cw.x + alpha * Pw[i].x;
        Cw.y = Cw.y + alpha * Pw[i].y;

        if( Pw[i].z NEQ NL_NOZ )
            Cw.z = Cw.z + alpha * Pw[i].z;
        else
            Cw.z = NL_NOZ;

        if( Pw[i].w NEQ NL_NOW )
            Cw.w = Cw.w + alpha * Pw[i].w;
        else
            Cw.w = NL_NOW;
    }

    N_CPtToPtEuclid( Cw, C );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the min-max box of a NURBS curve. A typical
     calling example is:

       NL_CURVE      cur;
       NL_MINMAXBOX  box;
       ...
       (define cur);
       ...
       N_CrvGetBBox(&cur,&box);


   ACCESS:
   
     cur , input  ,  NURBS curve
     box , output ,  Min-max box


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetBBox( NL_CURVE *cur, NL_MINMAXBOX *box )
{

    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    NL_EPOLYGON ppl;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetArraySizes( cur, &n, &m );

    /* Map control polygon */

    error = N_CrvGetEPolygon( cur, 0, n, &ppl, &S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get min-max box */

    N_PolygonGetBBox( &ppl, box );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvGetBBox */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a NURBS curve by evaluating 
     all  non-vanishing  basis functions  and  their derivatives, and 
     multiplying  them by  appropriate control points.  Discontinuous  
     curves  can  also be  handled  by  passing a  NL_LEFT/NL_RIGHT flag. A 
     typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_INDEX      der;
       NL_POINT      *CD;
       ...
       (define cur, get u and der, and allocate memory for CD);
       ...
       N_CrvDerivs(&cur,u,NL_LEFT,der,CD);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES REQUIRED)
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES REQUIRED)
     der , input  ,  Highest derivative required
     CD  , output ,  Derivatives; CD[k] is the kth derivative. MEMORY
                     FOR CD MUST BE ALLOCATED IN THE CALLING ROUTINE.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvDerivs */
NL_FLAG N_CrvDerivs( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_POINT *CD )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, m, r, s1, s2, rk, pk, j1, j2, mder, nsp;

    NL_INTEGER tri[NL_MAXDER + 1][NL_MAXDER + 1];

    NL_DEGREE p;

    NL_REAL *U, BD[NL_MAXDER + 1][NL_MAXDEG + 1], saved, temp, d, *left, w, *right, a[2][NL_MAXDEG + 1], ndu[NL_MAXDEG + 1][NL_MAXDEG + 1], *dd;

    NL_KNOTVECTOR *knt;

    NL_POINT P[NL_MAXDEG + 1];

    NL_CPOINT *Pw;

    /* Assign addresses */

    left = &a[0][0];
    right = &a[1][0];
    dd = &a[0][0];

    /* Get local notation */

    p = cur->p;
    knt = cur->knt;
    m = knt->m;
    U = knt->U;
    Pw = cur->pol->Pw;

    /* Check parameter and order of derivatives */

    if( u LT U[0]OR u GT U[m] )
    {
        N_ErrSet( NL_PAR_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    if( der GT NL_MAXDER )
    {
        N_ErrSet( NL_MXD_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Compute basis function derivatives */

    /* Find the knot span u is in */

    error = N_BasisFindSpan( knt, p, u, flg, &nsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get maximum derivative index and set zero derivatives */

    mder = NL_MIN( p, der );

    for ( k = p + 1; k <= der; k++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            BD[k][j] = 0.0;
        }
    }

    /* Compute the basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= p; j++ )
    {
        left[j] = u - U[nsp + 1 - j];
        right[j] = U[nsp + j] - u;
        saved = 0.0;

        for ( r = 0; r < j; r++ )
        {
            ndu[j][r] = right[r + 1] + left[j - r];
            if( N_FloatOpIsBad( ndu[r][j - 1], ndu[j][r] , NL_DIVISION ) )
               NL_ERROR( NL_NUM_ERR );

            temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the basis functions */

    for ( j = 0; j <= p; j++ )
        BD[0][j] = ndu[j][p];

    /* Compute derivatives */

    for ( r = 0; r <= p; r++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( k = 1; k <= mder; k++ )
        {
            d = 0.0;
            rk = r - k;
            pk = p - k;

            if( r GE k )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (r - 1)LE pk )
                j2 = k - 1;
            else
                j2 = p - r;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if( r LE pk )
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            BD[k][r] = d;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors */

    r = p;

    for ( k = 1; k <= mder; k++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            BD[k][j] *= r;
        }
        r *= (p - k);
    }

    if( N_IsCrvRat( cur ) )
    {
        /* Get derivatives of the denominator */

        for ( k = 0; k <= der; k++ )
        {
            dd[k] = 0.0;

            for ( j = nsp - p; j <= nsp; j++ )
            {
                w = Pw[j].w;
                dd[k] += w * BD[k][j - nsp + p];
            }
        }

        /* Compute derivatives of rational basis function */

        /* Pascal triangle first */

        tri[0][0] = 1;
        tri[1][0] = 1;
        tri[1][1] = 1;

        if( der GT 1 )
        {
            tri[2][0] = 1;
            tri[2][1] = 2;
            tri[2][2] = 1;

            if( der GT 2 )
            {
                for ( k = 3; k <= der; k++ )
                {
                    tri[k][0] = 1;
                    r = k / 2;
                    j2 = 1;

                    for ( j = 1; j <= r; j++ )
                    {
                        j1 = tri[k - 1][j];
                        tri[k][j] = tri[k - 1][j] + j2;
                        tri[k][k - j] = tri[k][j];
                        j2 = j1;
                    }
                    tri[k][k] = 1;
                }
            }
        }

        for ( i = 0; i <= p; i++ )
        {
            w = Pw[nsp - p + i].w;

            for ( k = 0; k <= der; k++ )
            {
                d = w * BD[k][i];

                for ( j = 1; j <= k; j++ )
                {
                    d -= tri[k][j] * dd[j] * BD[k - j][i];
                }
                BD[k][i] = d / dd[0];
            }
        }
    }

    /* Compute derivatives */

    j = nsp - p;

    for ( i = j; i <= nsp; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P[i - j] );
    }

    for ( k = 0; k <= der; k++ )
    {
        CD[k].x = CD[k].y = CD[k].z = 0.0;

        for ( i = 0; i <= p; i++ )
        {
            d = BD[k][i];
            CD[k].x += d * P[i].x;
            CD[k].y += d * P[i].y;
            CD[k].z += d * P[i].z;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvDerivs */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes the point and the unit tangent of a NURBS 
     curve at a given parameter value. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_POINT      P;
       NL_VECTOR     T;
       ...
       (define cur and get u);
       ...
       N_CrvEvalTangent(&cur,u,NL_LEFT,&P,&T);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES ARE NL_USED)
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES ARE NL_USED)
     P   , output ,  Point on the curve
     T   , output ,  Unit tangent


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalTangent( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_POINT *P, NL_VECTOR *T )
{

    NL_FLAG error = NL_NO;

    NL_REAL mag;

    NL_POINT CD[2];

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute curve derivatives */

    error = N_CrvDerivs( cur, u, flg, 1, CD );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get point and unit tangent */

    N_CopyPt( CD[0], P );

    error = N_VectorNormalize( CD[1], T, &mag );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvEvalTangent */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes the Frenet frame of a NURBS curve at a given
     parameter value. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_POINT      P;
       NL_VECTOR     T, N, B;
       ...
       (define cur and get u);
       ...
       N_CrvEvalFrenetFrame(&cur,u,NL_LEFT,&P,&T,&N,&B);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     u     , input  ,  Parameter value 
     flg   , input  ,  Flag:
                         NL_LEFT : u is in [u[j],u[j+1])
                                (NL_RIGHT DERIVATIVES ARE NL_USED)
                         NL_RIGHT: u is in (u[j],u[j+1]]
                                (NL_LEFT DERIVATIVES ARE NL_USED)
     P     , output ,  Point on the curve
     T,N,B , output ,  Unit tangent, normal and binormal


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalFrenetFrame( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_POINT *P, NL_VECTOR *T, NL_VECTOR *N, NL_VECTOR *B )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvEvalFrenetFrame");

    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_REAL mag;

    NL_POINT CD[3];

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute curve derivatives */

    N_CrvGetDegree( cur, &p );

    if( p LE 1 )
        NL_ERROR( NL_DEG_ERR );

    error = N_CrvDerivs( cur, u, flg, 2, CD );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get point and tangent */

    N_CopyPt( CD[0], P );

    error = N_VectorNormalize( CD[1], T, &mag );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get binormal */

    N_VectorCross( CD[1], CD[2], B );
    error = N_VectorNormalizeRef( B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get normal */

    N_VectorCrossRef( B, T, N );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvEvalFrenetFrame */


/*******************************************************************//**


   DESCRIPTION:

     This  routine  transforms  a  NURBS  curve given a general 4x4 
     transformation matrix. The  input data is  destroyed, i.e. the  
     transformation is done IN PLACE. A typical calling example is:

       NL_CURVE    cur;
       NL_RMATRIX  rma;
       ...
       (define cur and rma);
       ...
       N_CrvTransform(&cur,&rma);


   ACCESS:
   
     cur , in/out ,  NURBS curve
     rma , input  ,  4x4 Matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvTransform( NL_CURVE *cur, NL_RMATRIX *rma )
{

    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to 3-D if needed */

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    /* Transform control points */

    for ( i = 0; i <= n; i++ )
    {
        N_TransformCPt( Pw[i], rma, &Pw[i] );
    }
} /* end N_CrvTransform */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the minimum and maximum weights, and the
     minimum and  maximum magnitudes of position vectors of control
     points. A typical calling example is:

       NL_CURVE  cur;
       NL_REAL   wmin, wmax, pmin, pmax;
       ...
       (define cur);
       ...
       N_CrvGetMinMaxWeightsAndPts(&cur,&wmin,&wmax,&pmin,&pmax);

    This is a case where the theory and the practice conflict.
    We have adjusted pmax to be computed from a local origin, which conforms to
    NURBS translation independence.  If you use pmax to scale the tolerance for knot 
    removal in curves/surfaces, it can result in very small tolerances (tol *= w/(1+pmax))
    and no knot removal occurs. Maybe if you want to retain the 'conic'-ness, you would not
    want to remove knots anyway.
 
    Since the input tol is 3D, it should be scaled to the diag of the minmax box.
    Even then, a more useful result is achieved if you set pmax to zero:
  
   ACCESS:
   
     cur       , input  ,  NURBS curve
     wmin,wmax , output ,  Min-max weights
     pmin,pmax , output ,  Min-max position vectors


   RETURN CODES:

     None


   ***********************************************************************/

NL_VOID N_CrvGetMinMaxWeightsAndPts( NL_CURVE *cur, NL_REAL *wmin, NL_REAL *wmax, NL_REAL *pmin, NL_REAL *pmax )
{

    NL_INDEX i, n;

    NL_REAL mag, w;

    NL_CPOINT *Pw;

    NL_POINT P, C;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Compute min-max weights */

    if( N_IsCrvRat( cur ) )
    {
        N_CPtGetW( Pw[0], wmin );
        N_CPtGetW( Pw[0], wmax );

        for ( i = 1; i <= n; i++ )
        {
            N_CPtGetW( Pw[i], &w );

            if( w LT *wmin )
                *wmin = w;

            if( w GT *wmax )
                *wmax = w;
        }
    }
    else
    {
        *wmin = NL_NOW;
        *wmax = NL_NOW;
    }

    /* Compute the euclidean center, as the new origin */
    C.x = C.y = C.z = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P );
        N_Sum2Pts( P, C, &C );
    }
    N_ScalePt( (1.0 / ((NL_REAL)n + (NL_REAL)1)), C, &C );

    /* Compute min-max position vector magnitudes */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P );
        N_Diff2Pts( P, C, &P ); /* changed */
        N_VectorMagnitude( P, &mag );

        if( i EQ 0 || mag LT *pmin )
            *pmin = mag;

        if( i EQ 0 || mag GT *pmax )
            *pmax = mag;
    }

    *pmax = 0.0; /* changed: See header above */
}                /* end N_CrvGetMinMaxWeightsAndPts */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine unclamps a NURBS  curve IN PLACE.  That is, given a
     curve defined  by a  knot vector  that  has its  end  knots with
     multiplicity = p+1, where p is the degree. This routine computes
     a  precise  representation with a  new knot vector that does not
     have multiple knots at the ends. A typical calling example is:

       NL_CURVE  cur;
       ...
       (define cur);
       ...
       N_CrvUnclamp(&cur);


   ACCESS:
   
     cur , in/out ,  NURBS curve to be unclamped


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvUnclamp( NL_CURVE *cur )
{

    NL_INDEX i, j, n, m;

    NL_DEGREE p;

    NL_REAL *U, alf, bet;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Unclamp at the left end */

    for ( i = 0; i <= p - 2; i++ )
    {
        U[p - i - 1] = U[p - i] - U[n - i + 1] + U[n - i];

        for ( j = i; j >= 0; j-- )
        {
            alf = (U[p] - U[p + j - i - 1]) / (U[p + j + 1] - U[p + j - i - 1]);
            bet = alf / (alf - 1.0);
            alf = 1.0 - bet;
            N_Combine2CPts( alf, Pw[j], bet, Pw[j + 1], &Pw[j] );
        }
    }
    U[0] = U[1] - U[n - p + 2] + U[n - p + 1];

    /* Unlamp at the right end */

    for ( i = 0; i <= p - 2; i++ )
    {
        U[n + i + 2] = U[n + i + 1] + U[p + i + 1] - U[p + i];

        for ( j = i; j >= 0; j-- )
        {
            alf = (U[n + 1] - U[n - j]) / (U[n - j + i + 2] - U[n - j]);
            bet = (alf - 1.0) / alf;
            alf = 1.0 - bet;
            N_Combine2CPts( alf, Pw[n - j], bet, Pw[n - j - 1], &Pw[n - j] );
        }
    }
    U[m] = U[m - 1] + U[2 * p] - U[2 * p - 1];

    /* End NURBS and Exit */

    N_EndNurbs( &S );
} /* end N_CrvUnclamp */

/*******************************************************************//**


   DESCRIPTION:

     This routine unclamps a NURBS  curve IN PLACE.  That is, given a
     curve defined  by a  knot vector  that  has its  end  knots with
     multiplicity = p+1, where p is the degree. This routine computes
     a  precise  representation with a  new knot vector that does not
     have multiple  knots at the  ends. The new knots at the ends are
     given in the form of a new knot vector. The old and the new knot
     vectors  must agree on the interior and end knot values as shown
     below:

                  ||||_________|________|________||||  
                       (old clamped knot vector)

         |___|__|____|_________|________|________|____|___|____|     
                      (new unclamped knot vector)

     A typical calling example is:

       NL_CURVE       cur;
       NL_KNOTVECTOR  knt;
       ...
       (define cur and knt);
       ...
       N_CrvUnclampKnots(&cur,&knt);


   ACCESS:
   
     cur , in/out ,  NURBS curve to be unclamped
     knt , input  ,  New knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvUnclampKnots( NL_CURVE *cur, NL_KNOTVECTOR *knt )
{

    NL_INDEX i, j, n, m;

    NL_DEGREE p;

    NL_REAL *U, *UC, alf, bet;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &UC );
    N_KnotVectorGetKnots( knt, &m, &U );

    /* Unclamp at the left end */

    for ( i = 0; i <= p - 2; i++ )
    {
        for ( j = i; j >= 0; j-- )
        {
            alf = (U[p] - U[p + j - i - 1]) / (U[p + j + 1] - U[p + j - i - 1]);
            bet = alf / (alf - 1.0);
            alf = 1.0 - bet;
            N_Combine2CPts( alf, Pw[j], bet, Pw[j + 1], &Pw[j] );
        }
    }

    /* Unlamp at the right end */

    for ( i = 0; i <= p - 2; i++ )
    {
        for ( j = i; j >= 0; j-- )
        {
            alf = (U[n + 1] - U[n - j]) / (U[n - j + i + 2] - U[n - j]);
            bet = (alf - 1.0) / alf;
            alf = 1.0 - bet;
            N_Combine2CPts( alf, Pw[n - j], bet, Pw[n - j - 1], &Pw[n - j] );
        }
    }

    /* Copy knot vector */

    for ( i = 0; i <= m; i++ )
        UC[i] = U[i];

    /* End NURBS and Exit */

    N_EndNurbs( &S );
} /* end N_CrvUnclampKnots */

/*******************************************************************//**


   DESCRIPTION:

     This routine creates a point as a degenerate curve, i.e. for all 
     u values C(u) = P. A typical calling example is:

       NL_POINT   P;
       NL_DEGREE  p;
       NL_INDEX   n;
       NL_CURVE   cur;
       NL_STACKS  S;
       ...
       (get P, p and n);
       ...
       N_CrvInitArrays(&cur);
       N_CrvDegenFromPt(P,n,p,&cur,&S);

     If memory is not available, it is allocated  inside the routine. 
     If it is available, no memory is allocated, however, the routine
     checks for the proper amount by  looking at the  highest indexes 
     in cur's polygon and knot vector objects. This version creates a
     non-rational curve.


   ACCESS:
   
     P   , input  ,  Given point
     n   , input  ,  Highest index required in control point array
     p   , input  ,  Degree required
     cur , output ,  NURBS curve
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvDegenFromPt( NL_POINT P, NL_INDEX n, NL_DEGREE p, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvDegenFromPt");

    NL_FLAG error = NL_NO;

    NL_INDEX i, m;

    NL_REAL *U, uinc;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( p LT 0 OR p GT n OR p GT NL_DMAX )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    m = n + p + 1;

    error = N_CrvSizeArrays( cur, n, p, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get control points */

    for ( i = 0; i <= n; i++ )
    {
        N_PtToCPt( P, &Pw[i] );
    }

    /* Get the knots */

    uinc = 1.0 / ((NL_REAL)n - (NL_REAL)p + (NL_REAL)1);

    for ( i = 0; i <= p; i++ )
        U[i] = 0.0;

    for ( i = 1; i <= n - p; i++ )
        U[i + p] = i * uinc;

    for ( i = n + 1; i <= m; i++ )
        U[i] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvDegenFromPt */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine  creates a line as a non-rational  NURBS curve. The
     line is given by  the start point (P) and  direction vector (V).
     The end point is obtained as P+V. A typical calling example is:

       NL_POINT   P;
       NL_VECTOR  V;
       NL_CURVE   cur;
       NL_STACKS  S;
       ...
       (get P and V);
       ...
       N_CrvInitArrays(&cur);
       N_CrvLineFromPtAndVector(P,V,&cur,&S);

     If memory is not available, it is allocated  inside the routine. 
     If it is available, no memory is allocated, however, the routine
     checks for the proper amount by  looking at the  highest indexes 
     in cur's polygon and knot vector objects.


   ACCESS:
   
     P   , input  ,  Given point
     V   , input  ,  Direction vector
     cur , output ,  NURBS line
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvLineFromPtAndVector( NL_POINT P, NL_VECTOR V, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvLineFromPtAndVector");

    NL_FLAG error = NL_NO;

    NL_REAL *U;

    NL_POINT T;

    NL_CPOINT *Pw;

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, 1, 1, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get control points */

    N_PtToCPt( P, &Pw[0] );
    N_VectorSum( P, V, &T );
    N_PtToCPt( T, &Pw[1] );

    /* Get the knots */

    U[0] = 0.0;
    U[1] = 0.0;
    U[2] = 1.0;
    U[3] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvLineFromPtAndVector */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  scales a  NURBS curve  with respect to a point. A 
     typical calling example is:

       NL_POINT   C;
       NL_VECTOR  f;
       NL_CURVE   cur;
       ...
       (define cur, get C and scaling vector f);
       ...
       N_CrvScale(&cur,C,f);

     The  scaling  is  done  in-place,  i.e.  the  original  curve is 
     destroyed.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     C   , input  ,  Center of scaling
     f   , input  ,  Vector-valued scaling factor


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvScale( NL_CURVE *cur, NL_POINT C, NL_VECTOR f )
{

    NL_INDEX i, n;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to 3-D if needed */

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    /* Get scaled control points */

    for ( i = 0; i <= n; i++ )
    {
        N_ScaleCPtWithPtAndVector( Pw[i], C, f, &Pw[i] );
    }

    /* End NURBS */

    N_EndNurbs( &SL );
} /* end N_CrvScale */

/*******************************************************************//**


   DESCRIPTION:

     This curve routine scales weighted control points of a NURBS curve,
     i.e. if Pw[i]=(w[i]*x[i],w[i]*y[i],w[i]*z[i],w[i]) and  the scaling 
     factor is  f, then the  new control  points are Qw[i]=(w[i]*f*x[i],
     w[i]*f*y[i],w[i]*f*z[i],w[i]*f). A typical calling example is:

       NL_REAL   f;
       NL_CURVE  cur;
       ...
       (define cur and get f);
       ...
       N_CrvScaleCPts(&cur,f);

     THE SCALING IS DONE IN-PLACE, I.E. THE ORIGINAL NL_CURVE IS DESTROYED.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     f   , input  ,  Scaling factor


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvScaleCPts( NL_CURVE *cur, NL_REAL f )
{
    NL_INDEX i, n;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Get scaled control points */

    for ( i = 0; i <= n; i++ )
    {
        N_ScaleCPt( f, Pw[i], &Pw[i] );
    }

    /* End NURBS */

    N_EndNurbs( &SL );
} /* end N_CrvScaleCPts */


/*******************************************************************//**


   DESCRIPTION:

     This routine translates a NURBS curve. A typical calling example
     is as follows:

       NL_VECTOR  T;
       NL_CURVE   cur;
       ...
       (define cur and translation vector T);
       ...
       N_CrvTranslate(&cur,T);

     The  translation is done  in-place,  i.e. the  original  curve is 
     destroyed.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     T   , input  ,  Translation vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvTranslate( NL_CURVE *cur, NL_VECTOR T )
{

    NL_INDEX i, n;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to 3-D if necessary */

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    /* Get translated control points */

    for ( i = 0; i <= n; i++ )
    {
        N_TranslateCPt( Pw[i], T, &Pw[i] );
    }

    /* End NURBS */

    N_EndNurbs( &SL );
} /* end N_CrvTranslate */

/*******************************************************************//**


   DESCRIPTION:

     This routine rotates a NURBS curve about a general axis. A typical 
     calling example is:

       NL_POINT   P;
       NL_VECTOR  V;
       NL_REAL    al;
       NL_CURVE   cur;
       ...
       (define cur and get rotation parameters P, V and al);
       ...
       N_CrvRotateAboutAxis(&cur,P,V,al);

     The  rotation  is  done  in-place,  i.e.  the  original  curve  is 
     destroyed.


   ACCESS:
   
     cur , in/out ,  NURBS curve
     P,V , input  ,  Point and vector of rotation axis
     al  , input  ,  Rotation angle


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRotateAboutAxis( NL_CURVE *cur, NL_POINT P, NL_VECTOR V, NL_REAL al )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, n;

    NL_RMATRIX rma;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Convert to 3-D if needed */

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    /* Get rotation matrix */

    N_InitRealMatrix( &rma );
    error = N_CreateRotationMatrixAboutAxis( P, V, al, &rma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get rotated control points */

    for ( i = 0; i <= n; i++ )
    {
        N_TransformCPt( Pw[i], &rma, &Pw[i] );
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRotateAboutAxis */

/*******************************************************************//**


   DESCRIPTION:

     This curve routine projects a curve onto a  plane. Either parallel
     or  perspective  projection can  be used. If  the output  curve is 
     initialized to the  NULL curve, memory to store new control points 
     and  knots is  allocated. If the  output curve  is the same as the 
     input curve, projection is done in place and the original curve is 
     destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_POINT   O;
       NL_VECTOR  N, E;
       NL_STACKS  SQ;
       ...
       (define curP, get O, N and E);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvProjectOntoPlane(&curP,O,N,E,NL_PARALLEL   ,&curQ,&SQ);
       N_CrvProjectOntoPlane(&curP,O,N,E,NL_PERSPECTIVE,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and  polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     O    , input  ,  Point on the plane of projection
     N    , input  ,  Normal to the plane of projection
     E    , input  ,  Direction of projection  (NL_PARALLEL) or the center 
                      of projection (NL_PERSPECTIVE)
     prj  , input  ,  Flag:
                        NL_PARALLEL   : parallel projection required
                        NL_PERSPECTIVE: perspective projection required
     curQ , output ,  Curve after projection
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvProjectOntoPlane( NL_CURVE *curP, NL_POINT O, NL_VECTOR N, NL_VECTOR E, NL_FLAG prj, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvProjectOntoPlane");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, alf, bet, ne, nop, nep, neo, w;

    NL_POINT P, Q;

    NL_VECTOR OP, NN, EN, EO, EP;

    NL_CPOINT *Pw, *Qw;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Compute projections */

    if( N_IsCrvRat( curP ) )
        rat = NL_YES;

    switch( prj )
    {
        case NL_PARALLEL:

            error = N_VectorNormalize( N, &NN, &ne );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalize( E, &EN, &ne );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDot( NN, EN, &ne );

            if( fabs( ne )LT NL_LTOL )
                NL_ERROR( NL_INP_ERR );

            N_VectorDot( N, E, &ne );

            for ( i = 0; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i], &P );
                N_VectorDiff( O, P, &OP );
                N_VectorDot( N, OP, &nop );
                bet = nop / ne;
                N_Combine2Pts( 1.0, P, bet, E, &Q );

                if( rat )
                {
                    N_CPtGetW( Pw[i], &w );
                    N_Weight( Q, w, &Qw[i] );
                }
                else
                {
                    N_PtToCPt( Q, &Qw[i] );
                }
            }
            break;

        case NL_PERSPECTIVE:

            N_VectorDiff( E, O, &EO );
            N_VectorDot( N, EO, &neo );

            for ( i = 0; i <= n; i++ )
            {
                N_CPtToPtEuclid( Pw[i], &P );
                N_VectorDiff( E, P, &EP );
                N_VectorDot( N, EP, &nep );

                if( N_FloatOpIsBad( neo, nep, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = neo / nep;
                bet = 1.0 - alf;
                N_Combine2Pts( alf, P, bet, E, &Q );

                if( rat )
                {
                    N_CPtGetW( Pw[i], &w );
                    w = w * nep;
                    N_Weight( Q, w, &Qw[i] );
                }
                else
                {
                    N_PtToCPt( Q, &Qw[i] );
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Copy knot vector if new curve is computed */

    if( curP NEQ curQ )
    {
        for ( i = 0; i <= m; i++ )
            UQ[i] = UP[i];
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvProjectOntoPlane */



/*******************************************************************//**


   DESCRIPTION:

     This curve routine reverses a curve, i.e. it computes a  new curve
     that  is traced  out  in  reverse order.  If  the output  curve is 
     initialized to the NULL curve, memory to store new  control points 
     and  knots is  allocated. If the  output curve  is the same as the 
     input curve,  reversal is done in  place and the original curve is 
     destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_STACKS  SQ;
       ...
       (define curP);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvReverse(&curP,&curQ,&SQ);
       N_CrvReverse(&curP,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and  polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve to be reversed
     curQ , output ,  Curve after reversal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReverse( NL_CURVE *curP, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvReverse");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m, k;

    NL_DEGREE p;

    NL_REAL *UP = NULL, *UQ = NULL, c, a;

    NL_CPOINT *Pw = NULL, *Qw = NULL;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* See if memory is needed */

    if( curP NEQ curQ )
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Prepare for reversal */

    c = UP[0] + UP[m];

    if( curP NEQ curQ )
    {
        for ( i = 0; i <= p; i++ )
        {
            UQ[i] = UP[i];
            UQ[n + i + 1] = UP[n + i + 1];
        }
    }

    /* Reverse curve */

    if( curP EQ curQ )
    {
        k = n / 2;

        for ( i = 0; i <= k; i++ )
        {
            N_SwapCPts( &Pw[i], &Pw[n - i] );
        }

        k = m / 2 - p;

        for ( i = 1; i <= k; i++ )
        {
            a = UP[m - p - i];
            UP[m - p - i] = c - UP[p + i];
            UP[p + i] = c - a;
        }
    }
    else
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( Pw[n - i], &Qw[i] );
        }

        k = m - 2 * p - 1;

        for ( i = 1; i <= k; i++ )
        {
            UQ[m - p - i] = c - UP[p + i];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvReverse */



/*******************************************************************//**


   DESCRIPTION:

     This routine computes the largest extent of a NURBS curve which is
     the diagonal of its bounding box. A typical calling example is:

       NL_CURVE  cur;
       NL_REAL   d;
       ...
       (define cur);
       ...
       N_CrvGetBBoxMaxDiagDist(&cur,&d);


   ACCESS:
   
     cur , input  ,  NURBS curve
     d   , output ,  Length of the diagonal of the bounding box


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetBBoxMaxDiagDist( NL_CURVE *cur, NL_REAL *d )
{

    NL_FLAG error = NL_NO;

    NL_MINMAXBOX box;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute diagonal */

    error = N_CrvGetBBox( cur, &box );

    if( error EQ NL_YES )
        NL_OUT;

    N_BBoxGetDiagonal( &box, d );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvGetBBoxMaxDiagDist */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the maximum magnitude of position vectors 
     of curve control points. A typical calling example is:

       NL_CURVE  cur;
       NL_POINT  P;
       NL_REAL   mag;
       ...
       (define cur);
       ...
       N_CrvGetMaxPosVector(&cur,&P,&mag);


   ACCESS:
   
     cur , input  ,  NURBS curve
     P   , output ,  Longest position vector
     mag , output ,  Magnitude of longest position vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvGetMaxPosVector( NL_CURVE *cur, NL_POINT *P, NL_REAL *mag )
{

    NL_INDEX i, n;

    NL_REAL len;

    NL_CPOINT *Pw;

    NL_POINT Q;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Compute maximum position vector */

    N_CPtToPtEuclid( Pw[0], &Q );
    N_PtMagnitude( Q, mag );
    N_CopyPt( Q, P );

    for ( i = 1; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &Q );
        N_PtMagnitude( Q, &len );

        if( len GT *mag )
        {
            N_CopyPt( Q, P );
            *mag = len;
        }
    }
} /* end N_CrvGetMaxPosVector */


#if NLIB_UNUSED


/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine computes the minimum magnitude of position vectors 
     of curve control points. A typical calling example is:
 
       NL_CURVE  cur;
       NL_POINT  P;
       NL_REAL   mag;
       ...
       (define cur);
       ...
       N_CrvGetMinPosVector(&cur,&P,&mag);
 
 
   ACCESS:
   
     cur , input  ,  NURBS curve
     P   , output ,  Shortest position vector
     mag , output ,  Magnitude of shortest position vector
 
 
   RETURN CODES:
 
     None
 
   ***********************************************************************/

NL_VOID N_CrvGetMinPosVector( NL_CURVE *cur, NL_POINT *P, NL_REAL *mag )
{

    NL_INDEX i, n;

    NL_REAL len;

    NL_CPOINT *Pw;

    NL_POINT Q;

    /* Get local notation */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Compute minimum position vector */

    N_CPtToPtEuclid( Pw[0], &Q );
    N_PtMagnitude( Q, mag );
    N_CopyPt( Q, P );

    for ( i = 1; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &Q );
        N_PtMagnitude( Q, &len );

        if( len LT *mag )
        {
            N_CopyPt( Q, P );
            *mag = len;
        }
    }
} /* end N_CrvGetMinPosVector */

#endif // NLIB_UNUSED

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine compares the first derivative vectors from the left 
     and right at the internal knots,  and it returns  the  parameter
     intervals defining segments within which the  first  derivatives
     from  left  and  right  are  non-zero  and  pointing in the same 
     direction. Hence, each curve segment is generally  NL_G1-continuous
     (note: continuity is checked only at knots).  A  typical calling 
     example is:
 
       NL_CURVE   cur;
       NL_REAL    ang;
       NL_REAL    **segs;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (define cur);
       ...
       error = N_CrvGetG1Segs(&cur,ang,&segs,&n,&S);
 
 
   ACCESS:
   
     cur  , input  ,  NURBS curve
     ang  , input  ,  Angular tolerance (degrees).  Two  vectors  are 
                      considered to be in the same direction  if  the
                      angle between them is less  than  or  equal  to
                      ang
     segs , output ,  The parameter values defining the NL_G1-continuous
                      segments. segs[i][0]  and  segs[i][1]  are  the
                      start and end parameters of the i-th  segment ,
                      respectively.  This  array is allocated in this
                      routine
     n    , output ,  High index of segments (there are n+1 segments)
     S    , output ,  Memory stack for segs
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_CrvGetG1Segs( NL_CURVE *cur, NL_REAL ang, NL_REAL *** segs, NL_INDEX *n, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX *ind, m, ii, nn;

    NL_DEGREE p;

    NL_REAL ** sg, *U, angle;

    NL_POINT PL[2], PR[2];

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetDegree( cur, &p );
    N_CrvGetKnots( cur, &m, &U );

    /* Allocate memory for indexes of knots where discontinuities exist */

    ind = N_AllocInt1dArray( m - 2 * p, &SL );

    if( ind EQ NULL )
        NL_QUIT;

    /* Loop thru knots and find discontinuities */

    nn = -1;
    ii = p + 1;

    while( ii LT m - p )
    {
        error = N_CrvDerivs( cur, U[ii], NL_LEFT, 1, PL );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_CrvDerivs( cur, U[ii], NL_RIGHT, 1, PR );

        if( error EQ NL_YES )
            NL_OUT;

        angle = NL_BIGD;
        error = N_VectorsAngle( PL[1], PR[1], &angle );

        if( error EQ NL_YES OR angle GT ang )
        {
            nn += 1;
            ind[nn] = ii;

            if( error EQ NL_YES )
            {
                N_ErrClear();
                error = NL_NO;
            }
        }

        while( U[ii]EQ U[ii + 1] )
            ii += 1;
        ii += 1;
    }

    /* Allocate segs memory and load segs array */

    nn += 1;
    *n = nn;

    sg = N_AllocReal2dArray( nn, 1, S );

    if( sg EQ NULL )
        NL_QUIT;

    *segs = sg;

    sg[0][0] = U[0];
    sg[nn][1] = U[m];

    for ( ii = 0; ii < nn; ii++ )
    {
        sg[ii][1] = sg[ii + 1][0] = U[ind[ii]];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetG1Segs */

#ifdef NLIB_UNUSED

/*******************************************************************************************/
/* N_CrvExtendToPtLargeExtensions: Extend a curve to a point for a large NL_CMAX extension */
/*******************************************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This curve routine extends a Nurbs curve to a point. The start or
     end can be extended.  Continuity can  be  controlled via options.  
     After extension,  the original parameter range still maps to  the 
     original curve (i.e. the parameter range is also extended. This is 
     a special case of N_CrvExtendToPt when large NL_CMAX extensions are required 
     but the extension creates a bad, possibly self-intersecting curve. 
     
     A typical calling example is:

       NL_CURVE     curP, curQ;
       NL_POINT     pt;
       NL_REAL      sFactor
       NL_STACKS    SP, SQ;
       ...
       (define curP, xFactor and choose pt);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvExtendToPt(&curP,pt,NL_START,xFactor,&curQ,&SP,&SQ);
       N_CrvExtendToPt(&curP,pt,NL_END,xFactor,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's  
     knot vector and polygon objects.

     The algorithm is designed to give an improved NL_CMAX extension in the 
     case when NL_CMAX fails but a NL_G1 is acceptable. The process is to build 
     a NL_CMAX extension and a NL_G1 extension and then add two knots to the NL_CMAX
     curve and reset two CPOINTs of the NL_CMAX to those of the NL_G1 extension.

   ACCESS:
   
     curP , input  ,  NURBS curve
     pt   , input  ,  Point to where the curve is extended
     atSorE , input  ,  Flag:
                       NL_START: The curve is extended back from  its  start
                              point
                       NL_END  : The curve is extended forward from its  end
                              point
     xFactor, input  , NL_REAL:  0 < xFactor < 0.2  0.1 is a good choice
                              This contols the max knot spacing of the
                              two knots that are added at the end of the 
                              of the curve
     curQ , output ,  Curve after extension
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvExtendToPtLargeExtensions( NL_CURVE *curP, NL_POINT pt, NL_FLAG atSorE, NL_REAL xFactor, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CrvExtendToPtLargeExtensions"); */
    /* NL_PRIVATE NL_REAL KN_TOL = 1.0e-6; */

    NL_CURVE CrvG1;
    NL_CURVE *CrvCMAX;

    /* We need to get the defining data */
    NL_INDEX nC, mC;
    NL_DEGREE pC;
    NL_REAL *UC;
    NL_CPOINT *PwC;
    NL_INDEX nG, mG;
    NL_DEGREE pG;
    NL_REAL *UG;
    NL_CPOINT *PwG;
    NL_INDEX mp;

    NL_REAL dt, uNew;
    NL_REAL t0, t1, t2;
    NL_REAL tn, tn_1, tn_2;

    NL_FLAG error = NL_NO;
    NL_STACKS SL; /* local for the NL_G1 extension */

    /* Start NURBS */

    N_InitNurbs( &SL );

    N_CrvInitArrays( &CrvG1 );

    /* CrvG1 starts as a copy of curP */
    error = N_CrvCopy( curP, &CrvG1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* do a NL_CMAX extension of curP */
    CrvCMAX = curQ;
    error = N_CrvExtendToPt( curP, pt, atSorE, NL_CMAX, CrvCMAX, SP, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* do a NL_G1 extension of curP */
    error = N_CrvExtendToPt( &CrvG1, pt, atSorE, NL_G1, &CrvG1, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( CrvCMAX, &nC, &PwC, &pC, &mC, &UC );

    N_CrvGetCPtsDegreeAndKnots( &CrvG1, &nG, &PwG, &pG, &mG, &UG );

    if( atSorE EQ NL_START )
    { /* this extension is at the start of U so work around U[p] 
         initially U[p-1] = 0
         add two knots to CrvCMAX between U[p] and U[p+1]
      */
        t0 = UC[pC];     /* start param of Crv */
        t1 = UC[pC + 1]; /* should be start parm of srf before extension */
        t2 = UC[pC + 2];
        dt = t2 - t1; /* length of first span before extension */
 
      /* require dt to be < xFactor*(t1-t0) */

        if( dt > xFactor * (t1 - t0) )
            dt = xFactor * (t1 - t0);

        /* add two knots to CrvCMAX */
        uNew = t1 - 0.5 *dt;
        error = N_CrvInsertKnot( CrvCMAX, uNew, 1, CrvCMAX, SQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        uNew = t1 - dt;
        error = N_CrvInsertKnot( CrvCMAX, uNew, 1, CrvCMAX, SQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        /*
        CrvCMAX now has two more spans at the start of the curve so 
        reset PwC[1] and PwC[2] by using PwG1[1] and PwG1[2]
        */

        /* need to get the defining data again since knots have been added */
        N_CrvGetCPtsDegreeAndKnots( CrvCMAX, &nC, &PwC, &pC, &mC, &UC );

        N_CopyCPt( PwG[1], &PwC[1] );
        N_CopyCPt( PwG[2], &PwC[2] );
    }
    else
    { /* 
      extension is at the end of the curve so work around U[m-p] 
      initially U[m-p-1] = 1
      add two knots to CrvCMAX between U[m-p-1 and U[m-p]
      */
        mp = mC - pC;
        tn = UC[mp];       /* end param */
        tn_1 = UC[mp - 1]; /* should be end parm of cur before extension */
        tn_2 = UC[mp - 2];
        dt = tn_1 - tn_2;  /* length of last span before extension */

        if( dt > xFactor * (tn - tn_1) )
            dt = xFactor * (tn - tn_1);

        uNew = tn_1 + 0.5 *dt;
        error = N_CrvInsertKnot( CrvCMAX, uNew, 1, CrvCMAX, SQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        uNew = tn_1 + dt;
        error = N_CrvInsertKnot( CrvCMAX, uNew, 1, CrvCMAX, SQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
        /*
        CrvCMAX now has two more spans at the end of the curve so 
        reset PwC[n-2] and PwC[n-1] by using PwG1[n-2] and PwG1[n-1]
        */

        /* need to get the defining data again since knots have been added */
        N_CrvGetCPtsDegreeAndKnots( CrvCMAX, &nC, &PwC, &pC, &mC, &UC );

        N_CopyCPt( PwG[nG - 2], &PwC[nC - 2] );
        N_CopyCPt( PwG[nG - 1], &PwC[nC - 1] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvExtendToPtLargeExtensions */

#endif //NLIB_UNUSED

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine determines which segments of a curve are curved, and
     which are straight lines.  It  returns  parameter  intervals that 
     correspond to the linear and curved segments.  A  typical calling 
     example is:
 
       NL_CURVE   cur;
       NL_REAL    tol;
       NL_REAL    **segs;
       NL_FLAG    *linf;
       NL_INDEX   nn;
       NL_STACKS  S;
       ...
       (define cur and choose tol);
       ...
       N_CrvGetLinearSegs(&cur,tol,&segs,&linf,&nn,&S);

     This function assumes that degree of the curve is greater than 1.
     It returns an error (NL_DEG_ERR) if this is not the case.
 
 
   ACCESS:
   
     cur  , input  ,  NURBS curve (degree greater than 1)
     tol  , input  ,  Tolerance measures zero distance in the space in
                      which the curve is defined. Points are consider-
                      ed collinear if all of them are within tol dist-
                      ance of the line defined by their two endpoints,
                      and if the angles defined by them are small.
     segs , output ,  The parameter values  defining  the  linear  and
                      curved segments. segs[i][0]  and  segs[i][1] are
                      the start and end parameters of the i-th segment
                      respectively.  This  array is  allocated in this
                      routine.
     linf , output ,  Flags, indicating  linearity  of  the  segments:
                      linf[i] = NL_YES (NL_NO) indicates the i-th segment is
                      (not) linear.  This  array is  allocated in this
                      routine.
     nn   , output ,  High index of segments (there are nn+1 segments)
     S    , output ,  Memory stack for segs and linf
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_CrvGetLinearSegs( NL_CURVE *cur, NL_REAL tol, NL_REAL *** segs, NL_FLAG ** linf, NL_INDEX *nn, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetLinearSegs");

    NL_FLAG *flg, error = NL_NO;

    NL_INDEX n, m, kn, ii, jj;

    NL_DEGREE p;

    NL_CURVE cseg;

    NL_REAL ** sg, *U, cos;

    NL_CPOINT *Pw;

    NL_VECTOR V1, V2;

    NL_BOOLEAN alloc, line;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( p LT 2 )
        NL_ERROR( NL_DEG_ERR );

    /* Allocate memory and initialize it */

    sg = N_AllocReal2dArray( 0, 1, S );
    flg = N_AllocFlag1dArray( 0, S );

    if( sg EQ NULL OR flg EQ NULL )
        NL_QUIT;

    sg[0][0] = U[0];
    kn = 0;

    N_CrvInitArrays( &cseg );
    alloc = NL_TRUE;

    /* Loop thru knots and find linear/curved segments */

    ii = p;

    while( 1 )
    {
        line = NL_FALSE;

        jj = ii - p + 1;
        N_VectorDiffCPts( Pw[jj], Pw[jj - 1], &V2 );

        NL_INDEX jjStart = jj;
        for ( jj = jjStart; jj < ii; jj++ )
        {
            N_VectorCopy( V2, &V1 );
            N_VectorDiffCPts( Pw[jj + 1], Pw[jj], &V2 );

            error = N_VectorsCosAngle( V1, V2, &cos );

            if( error EQ NL_NO )
                if( cos LT 0.99985 )
                    break; /* 1 degree angle */
        }

        if( jj EQ ii )
        { /* must do point-to-line distance check */
            if( alloc )
            {
                error = N_CrvSizeArrays( &cseg, n, p, m, rname, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                alloc = NL_FALSE;
            }

            error = N_CrvExtractCrvSeg( cur, U[ii], U[ii + 1], &cseg, S, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            line = N_CrvIsLine( &cseg, tol );
            N_CrvSetSizeIndices( &cseg, n, p, m );
        }

        if( line )
        {
            if( ii EQ p )
                flg[kn] = NL_YES;
            else
            {
                if( flg[kn]EQ NL_YES )
                {
                    error = N_CrvExtractCrvSeg( cur, sg[kn][0], U[ii + 1], &cseg, S, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                    line = N_CrvIsLine( &cseg, tol );
                    N_CrvSetSizeIndices( &cseg, n, p, m );
                }

                if( flg[kn]EQ NL_NO OR( NOT line ) )
                {
                    sg[kn][1] = U[ii];
                    error = N_Realloc2dRealArray( &sg, kn, 1, kn + 1, 1, S );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_Realloc1dFlagArray( &flg, kn, kn + 1, S );

                    if( error EQ NL_YES )
                        NL_OUT;
                    kn += 1;
                    sg[kn][0] = U[ii];
                    flg[kn] = NL_YES;
                }
            }
        }
        else
        {
            if( ii EQ p )
                flg[kn] = NL_NO;
            else if( flg[kn]EQ NL_YES )
            {
                sg[kn][1] = U[ii];
                error = N_Realloc2dRealArray( &sg, kn, 1, kn + 1, 1, S );

                if( error EQ NL_YES )
                    NL_OUT;
                error = N_Realloc1dFlagArray( &flg, kn, kn + 1, S );

                if( error EQ NL_YES )
                    NL_OUT;
                kn += 1;
                sg[kn][0] = U[ii];
                flg[kn] = NL_NO;
            }
        }

        ii += 1; /* go to the next span */

        if( U[ii]GE U[m] )
            break;

        while( U[ii]EQ U[ii + 1] )
            ii += 1;
    }

    sg[kn][1] = U[m];

    *nn = kn;
    *segs = sg;
    *linf = flg;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetLinearSegs */


/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This curve routine finds all degenerate segments of a curve. The
     segment [u1,u2] is defined to  be  degenerate if its image curve
     segment between C(u1) and C(u2) has length less  than some given
     tolerance. A typical calling example is:
 
       NL_CURVE   cur;
       NL_REAL    tol;
       NL_REAL    **segs;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (define cur and choose tol);
       ...
       N_CrvGetDegenSegs(&cur,tol,&segs,&n,&S);
 
 
   ACCESS:
   
     cur  , input  ,  NURBS curve
     tol  , input  ,  Tolerance.  A  segment is degenerate if its arc
                      length is less than tol. 
     segs , output ,  The parameter values  defining  the  degenerate
                      segments. segs[i][0]  and  segs[i][1]  are  the
                      start and end parameters of the i-th degenerate
                      segment, respectively. This  array is allocated
                      in this routine (only if n >= 0)
     n    , output ,  High index of segments (n+1 segments). If there
                      are no degenerate segments, n = -1  and segs is
                      not allocated.
     S    , output ,  Memory stack for segs
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_CrvGetDegenSegs( NL_CURVE *cur, NL_REAL tol, NL_REAL *** segs, NL_INDEX *n, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX nn, m, ii; 

    NL_DEGREE p;

    NL_REAL ** sg = NULL, *U, len, d1, d2;

    NL_POINT Ps, Pe, Pm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetDegree( cur, &p );
    N_CrvGetKnots( cur, &m, &U );

    /* Loop thru segments and find the degeneracies */

    nn = -1;
    len = 0.0;
    ii = p;
    error = N_CrvEval( cur, U[ii], NL_LEFT, &Ps );

    while( 1 )
    {
        error = N_CrvEval( cur, U[ii + 1], NL_LEFT, &Pe );

        N_DistPtPt( Ps, Pe, &d1 );

        if( d1 LE tol )
        {
            if( m LE 2 *p + 1 )
            {
                error = N_CrvEval( cur, 0.5 *( U[ii] + U[ii + 1] ), NL_LEFT, &Pm );
                N_DistPtPt( Ps, Pm, &d2 );
                N_DistPtPt( Pm, Pe, &d1 );

                if( d2 + d1 GT tol )
                    break;
            }

            error = N_CrvArcLength( cur, U[ii], U[ii + 1], tol, NL_ABSOLUTE, &d1 );

            if( error EQ NL_YES )
                NL_OUT;

            if( d1 GT tol )
                len = 0.0;
            else
            {
                if( len GT 0.0 AND len + d1 LE tol )
                {
                    sg[nn][1] = U[ii + 1];
                    len += d1;
                }
                else
                {
                    len = d1;
                    nn += 1;

                    if( nn EQ 0 )
                    {
                        sg = N_AllocReal2dArray( 0, 1, S );

                        if( sg EQ NULL )
                            NL_QUIT;
                    }
                    else
                    {
                        error = N_Realloc2dRealArray( &sg, nn - 1, 1, nn, 1, S );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    sg[nn][0] = U[ii];
                    sg[nn][1] = U[ii + 1];
                }
            }
        }
        else
            len = 0.0;

        if( ii GE m - p - 1 )
            break;

        /* prepare for next pass thru loop */

        N_CopyPt( Pe, &Ps );
        ii += 1;

        while( U[ii]EQ U[ii + 1] )
            ii += 1;
    }

    *n = nn;

    if( nn GE 0 )
        *segs = sg;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetDegenSegs */

/*******************************************************************//**


   DESCRIPTION:

     This curve routine scales a curve's knot vector to a given inter-
     val, [ul,ur]. This does not change the curve geometry.  The oper-
     ation is done in place. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  ul, ur;
       ...
       (define cur, and choose ul,ur);
       ...

       N_CrvReparam(&cur,ul,ur);


   ACCESS:
   
     cur  , in/out ,  NURBS curve whose knot vector is to be rescaled
     ul   , input  ,  Start parameter of curve after scaling
     ur   , input  ,  End parameter of curve after scaling


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvReparam( NL_CURVE *cur, NL_PARAMETER ul, NL_PARAMETER ur )
{

    NL_INTERVAL I;

    /* Make new parameter interval and call the utility */

    N_CreateInterval( &I, ul, ur );

    N_CrvReparamToInterval( cur, I );
} /* end N_CrvReparam */

/*******************************************************************//**


   DESCRIPTION:

     This routine  determines the type of a given curve.  Lines, circles
     and free-form curves in 2d and 3d are recognized. A typical calling example:

       NL_CURVE   cur;
       NL_REAL    tol, r, al;
       NL_POINT   AA;
       NL_VECTOR  BB, XX, YY;
       NL_FLAG    typ;
       ...
       (define cur and get tol);
       ...
       N_CrvGetType(&cur,tol,&AA,&BB,&XX,&YY,&r,&al,&typ);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     tol   , input  ,  Tolerance;
                         line  : points do not deviate  from a line more 
                                 than tol
                         circle: radii  of  osculating  circles  do  not
                                 differ more than tol,  and the curve is
                                 planar to within this tolerance
     AA    , output ,  Point;
                         line  : start point
                         circle: center of circle
     BB    , output ,  Vector;
                         line  : direction vector
                         circle: unit normal to circle's plane
     XX,YY , output ,  Local coordinate system;
                         line  : undefined
                         circle: unit perpendicular axes in the plane of
                                 circle centered at the center
     r     , output ,  Radius;
                         line  : undefined
                         circle: radius of circle
     al    , output ,  Sweep angle;
                         line  : undefined
                         circle: angle of arc
     typ   , output ,  Curve type:
                         NL_NLINE    : line
                         NL_NCIRCLE  : circular arc
                         NL_NFREEFORM: spline curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR
   NOTE:
     N_CrvGetType uses curvature values to determine circularity.
     Actually, there is often alot of numerical noise in such
     calculations. If this is a source of too much error, then
     implementing a new strategy based on purely geometric
     computations would probably improve the situation. For
     example, using functions such as N_IsPtSet1dCircular and N_CreateCircFrom2dPts.

     Use NL_FLAG  N_CrvGetBBoxMaxDiagDist( NL_CURVE *cur, NL_REAL *d ) to get d to scale tol
     ( suggested use tol < 0.004*d )
   ***********************************************************************/
NL_FLAG N_CrvGetType   /* rtn: 0 = OK, 1 = error                                    */
 ( NL_CURVE  * cur,    /* in : Target Curve                                         */
   NL_REAL     tol,    /* in : max dist to Line or max variation in radius          */
   NL_POINT  * AA,     /* out: line start point or circle center                    */                        
   NL_VECTOR * BB,     /* out: line dir vector  or circle plane normal              */                             
   NL_VECTOR * XX,     /* out: line undefined   or circle x axis                    */                        
   NL_VECTOR * YY,     /* out: line undefined   or circle y axis                    */                        
   NL_REAL   * r,      /* out: line undefined   or circle radius                    */                        
   NL_REAL   * al,     /* out: line undefined   or circle angle or arc in degrees   */                                         
   NL_FLAG   * typ )   /* out: NL_NLINE        - curve is within tol of line        */
                       /*      NL_NCIRCLE      - curve is within tol of circle      */
                       /*      NL_NFREEFORM    - curve is neither a line or circle  */                                      
{
    NL_FLAG lin = NL_YES ; /* assume a line, tested and changed later */
    
    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_INDEX i, j, k, kl, kh, kr, n, m;

    NL_REAL *U, *rd, d, kap, ra, fac, t, tl, tr, ti, dab, minrad, maxrad, dab2, tol2 ; 
    
    NL_BOOLEAN bIsClosed ;

    NL_POINT *C, Ca, A, B, Q, R, *P;

    NL_VECTOR *N, T, V, Na;

    NL_CPOINT *Pw;

    NL_PLANE pln;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals */

    *typ = NL_NFREEFORM;

    N_CrvGetCPtsDegreeAndKnots(  cur,  /* in : target curve                                                    */
                               & n,    /* in : Highest index in control point array                            */
                               & Pw,   /* out: array of control points                                         */
                               & p,    /* out: curve degree                                                    */
                               & m,    /* out: Highest index in KnotArray                                      */
                               & U );  /* out: array of knots (multiple knots are represented multiple times ) */
    /* Check if line */

    N_CPtToPtEuclid( Pw[0], &A );  /* A = 1st control point in euclidean coordinates */
    N_CPtToPtEuclid( Pw[n], &B );  /* B = last control point in euclidean coordinates */

    N_VectorDir( A, B, &T );
    N_VectorMagnitude( T, &dab );

    if( dab LE NL_MTOL AND dab LE NL_PTOL )
        lin = NL_NO;  /* closed curves are not lines */

    /* when control point count is larger than 2, check for colinearity */
    else if( n GT 1 ) /* n == 1 implies a line */
    {
        dab2 = dab * dab;

        kh = 2 * p;
        tl = U[p];
        i = p + 1;

        while( i LT m ) /* loop and use points in curve segments */
        {
            while( i LT m AND U[i]EQ U[i + 1] )
                i++;
            tr = U[i];
            ti = (tr - tl) / kh;

            if( i GE m )
                kh -= 1;

            for ( k = 1; k <= kh; k++ )
            {
                if( k EQ kh AND i LT m )
                    t = tr;
                else
                    t = tl + k * ti;

                error = N_CrvEval( cur, t, NL_LEFT, &Q );

                if( error EQ NL_YES )
                {
                    N_ErrClear();
                    error = NL_NO;
                    NL_OUT;
                }

                /* project Q onto line [A,B] to get R */

                N_VectorDir( A, Q, &V );
                N_VectorDot( V, T, &d );

                d /= dab2;

                if( d LT 0.0 OR d GT 1.0 )
                {
                    lin = NL_NO;
                    break;
                }

                N_VectorPtAlongVector( A, d, T, &R );

                N_DistPtPt( Q, R, &d ); /* projection distance */

                if( d GT tol )
                {
                    lin = NL_NO;
                    break;
                }

                N_DistPtPt( Q, A, &d ); /* this distance should not be  */

                if( d GT dab )          /* more than length of line seg */
                {
                    lin = NL_NO;
                    break;
                }
            }

            if( lin EQ NL_NO )
                break;

            tl = tr;
            i++;
        }
    } /* end more than 2 control points so need to test for linearity check */

    /* arrive here when lin is set */ 

    if( lin EQ NL_YES )
    {
        N_CopyPt( A, AA );
        N_CopyPt( T, BB );

        *typ = NL_NLINE;

        /* all done */ 
        NL_OUT;
    }

    /* arrive here when curve is not a line */ 

    /* Check if circle */
    N_DistPtPt( A, B, &d );  /* A = first control point in euclidean coordinates */
                             /* B = last  control point in euclidean coordinates */
    bIsClosed = (d LT tol) ? NL_TRUE : NL_FALSE ;

                            /* i = generic index - it means different things at different times - now highest index of curve samples */
    i = 2 * n * (p + 1);    /* n = curve max control point index */                                     
                            /* p = curve degree */
    rd = N_AllocReal1dArray( i, &SL );

    if( rd EQ NULL )
        NL_QUIT;

    C = N_AllocPt1dArray( 3 * (i + 1), &SL );  /* alloc C, N, and P arrays */ 
                                               /* C = array of curve sample osculating circle centers */
    if( C EQ NULL )
        NL_QUIT;

    N = &C[i + 1];  /* N = array of curve sample osculating plane normals */
    P = &N[i + 1];  /* P = array of curve sample positions */

    ra = 0.0;     /* ra = radius average */
    kh = 2 * p;   /* kh = number of samples per knot span (one extra for 1st span) */
    j = -1;       /* j  = accumulative sample index into the C, N, and P arrays  */
    tl = U[p];    /* tl = param to left of current knot span */
    i = p + 1;    /* i  = left knot index of current knot span */

    tol2 = 2.0 * tol;
    maxrad = 0.0;
    minrad = NL_BIGD;

    N_CopyPt( NL_ZERO, &Ca );  /* Ca = average of all sample osculating circle centers */
    N_CopyPt( NL_ZERO, &Na );  /* Na = average of all sample osculating plane normals */

    /* for all knot spans */
    while( i LT m ) /* i less than knot greatest index */
    {
        if( i EQ p + 1 )   /* for the 1st span only */
            kl = 0;        /* sample the span start value */
        else
            kl = 1;        /* else skip the span start value (it's already sampled as the last span's end sample */

        /* increment i to last knot of current knot span */
        while( i LT m AND U[i]EQ U[i + 1] )  /* set i to index of last of multiple knots - no change for single knots */
            i++;

        if( bIsClosed && i == m) 
            kr = kh-1 ;          /* don't sample the span end value for the last span on a closed curve */
        else
            kr = kh ;            /* else sample the span end value for all other spans */

        tr = U[i];             /* tr = param to right of current knot span */
        ti = (tr - tl) / kh;   /* ti = param increment between sample points */

        /* for every sample point in this knot span */
        for ( k = kl; k <= kr; k++ )  /* for ever sample in this knot span (one extra for 1st span) */
        {
            if( k EQ kh )   /* t = target param value in current span */
                t = tr;     /* for last sample - set t = span right param value */
            else
                t = tl + k * ti;  /* for other samples - increment t evenly through the span */ 
            j++;            /* increment index into C, N, and P arrays */
            
            /* eval curve curvature, position, and osculating circle at param = t */
            error = N_CrvEvalCurvature( cur,       /* in : target curve                           */
                                        t,         /* in : target param                           */
                                        NL_LEFT,   /* in : pick span side to eval if at knot bdry */
                                       &kap,       /* out: curvature at param t, radius = 1/kap   */
                                       &P[j],      /* out: position at param t                    */
                                       &T,         /* out: unitTangent at param t                 */
                                        NL_YES,    /* out: NL_YES = compute osculating circle     */
                                       &C[j],      /* out: osculating center                      */
                                       &N[j] );    /* out: osculating plane normal                */

            if( error EQ NL_YES )
            {
                N_ErrClear();
                error = NL_NO;
                NL_OUT;
            }

            if( N_FloatOpIsBad( 1.0, kap, NL_DIVISION ) )
                NL_OUT;

            /* The following code (16 lines) was commented out in v8.5.8 */
            /* This caused a regression See [B261] */
            rd[j] = 1.0 / kap;
                  
            /* accumulate the sample radius value into a sum of radius value */
            ra += rd[j];
            
            /* save min and max radius values */
            if( minrad GT rd[j] )
                minrad = rd[j];
             
            if( maxrad LT rd[j] )
                maxrad = rd[j];
        
            /* quit - when tolerances are exceeded */
            if( maxrad - minrad GT tol2 )
                NL_OUT;

            /* accumulate osculating circle center and plane normal sums */
            N_Sum2Pts( Ca, C[j], &Ca );
            N_Sum2Pts( Na, N[j], &Na );

        } /* end iter every sample point in this knot span */

        /* move the next span left side param value to this span's right side param */
        tl = tr;

        /* increment the knot index to the first knot value to the right of the next span */
        i++;
    } /* end while iterating all knot spans in curve */

    /* turn the sums into averages by dividing by the total sample count */
    fac = 1.0 / ((NL_REAL)j + (NL_REAL)1);
    /*      ra = fac * ra;  */            /* radius sum to radius average */

    N_ScalePt( fac, Ca, &Ca );  /* circle center sum to circle center average */
    N_ScalePt( fac, Na, &Na );  /* circle plane normal sum to circle plane normal average */

    /* define the average osculating circle plane */
    N_CreatePlanePtNormal( &pln, Ca, Na );

    /* get the radius from the sample positions and not the average of the curvature of radii */
    ra     = 0.0 ;
    maxrad = 0.0;
    minrad = NL_BIGD;

    /* for every sample point */
    for ( i = 0; i <= j; i++ )
    {
        /* proj sample point to plane */
        error = N_ProjectPtPlane( pln, P[i], &Q );

        if( error EQ NL_YES )
            NL_OUT;

        /* radial distance */
        N_DistPtPt( P[i], Ca, &d );

        rd[i] = d ;

        /* accumulate the sample radius vaule into a sum of radius value */
        ra += rd[i];

        /* save min and max radius values */
        if( minrad GT rd[i] )
            minrad = rd[i];

        if( maxrad LT rd[i] )
            maxrad = rd[i];

        /* proj distance */
        N_DistPtPt( P[i], Q, &d );

        /* if any sample point projects to plane by more than tolerance - not a circle */
        if( d GT tol )
            NL_OUT;
    } /* end iter every sample point */

    /* turn the radius sum into averages by dividing by the total sample count */
    fac = 1.0 / ((NL_REAL)j + (NL_REAL)1);
    ra = fac * ra;              /* radius sum to radius average */

    /* if any sample radius varies from the average by more than tol - not a circle */
    if( fabs( maxrad - ra ) GT tol )
        NL_OUT;
    if( fabs( ra - minrad ) GT tol )
        NL_OUT;

    /* Let circle X axis = Normalized(1stControlPoint - CircleCenter) */
    N_CopyPt( Ca, AA );
    N_CopyPt( Na, BB );
    N_Diff2Pts( A, Ca, &T );  /* A = 1st control point in euclidean coordinates */

    error = N_VectorNormalize( T, XX, &d );

    if( error EQ NL_YES )
        NL_OUT;

    /* Let circle Y axis = CrossProduct(OsculatingPlaneNormal, CircleXAxis) */
    N_VectorCross( Na, *XX, YY );
    /* N_DistPtPt( A, B, &d );  */

    /* when 1st and last control points are colocated */
    if( bIsClosed )
    {
        *al = 360.0; /* set circle sweep = 360 degrees */
    }
    else /* not a closed circle */
    {
        /* vector fom circle center to last control point */
        N_Diff2Pts( B, Ca, &T );
        
        /* angleDeg [ 0, 180] between circle start vector and end vector */
        error = N_VectorsAngle( *XX, T, &fac );

        if( error EQ NL_YES )
            NL_OUT;

        /* when appropriate - change angles from 0 to 180 to 0 to 360 deg */
        N_VectorDot( *YY, T, &d );

        /* when EndVec is in 2nd half of circle - map 0 to 180 to 180 to 360 */
        if( d LT 0.0 )
            *al = 360.0-fac;
        else /* else EndVec is in 1st half of circle */
            *al = fac;
    } /* end not a closed circle */

    /* set output */
    *r = ra;
    *typ = NL_NCIRCLE;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetType */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivative of the curvature of a curve at 
     a given  parameter. Discontinuous  curves can be handled by passing 
     a NL_LEFT/NL_RIGHT flag. A typical calling example:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_REAL       kpp;
       ...
       (define cur and get u);
       ...
       N_CrvGetCurvatureDeriv(&cur,u,NL_LEFT,&kpp);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     ufl , input  ,  Flag:
                       NL_LEFT : u is in  [u[j],u[j+1]) (NL_RIGHT  derivatives
                              used)
                       NL_RIGHT: u is  in  (u[j],u[j+1]] (NL_LEFT  derivatives
                              used)
     kpp , output ,  Curvature derivative at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetCurvatureDeriv( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG ufl, NL_REAL *kpp )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetCurvatureDeriv");

    NL_FLAG error = NL_NO;

    NL_REAL num, den, f1, f2;

    NL_POINT D[4];

    NL_VECTOR A, Z;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get derivatives and binormal */

    error = N_CrvDerivs( cur, u, ufl, 3, D );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCross( D[1], D[2], &Z );
    N_VectorMagnitude( Z, &f1 );

    if( f1 LT NL_MTOL )
    {
        *kpp = 0.0;
        NL_OUT;
    }

    error = N_VectorNormalizeRef( &Z );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get curvature derivative */

    N_VectorMagnitude( D[1], &den );
    N_VectorCross( D[1], D[3], &A );
    N_VectorDot( A, Z, &f2 );

    f1 = den * den;
    num = f1 * f2;

    N_VectorDot( D[1], D[2], &f1 );
    N_VectorCross( D[1], D[2], &A );
    N_VectorDot( A, Z, &f2 );

    num = num - 3.0 *f1 * f2;
    den = den * den * den * den * den;

    if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    *kpp = num / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvGetCurvatureDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This routine determines if a curve is Gn smoothly closed to  within
     specified tolerances.  By definition,  Gn continuity requires  Gn-1
     continuity, and hence, G0,...,Gn continuity is checked in this rou-
     tine.  Currently,  this  routine  can  determine only G0, NL_G1 and NL_G2 
     continuity. A typical calling example is:

       NL_CURVE    cur;
       NL_FLAG     gnflg;
       NL_REAL     tols[3];
       ...
       (define cur and set tols);
       ...
       N_CrvIsClosedContinuity(&cur,1,tols,&gnflg);


   ACCESS:
   
     cur   , input  ,  NURBS curve
     n     , input  ,  The level of G-continuity to  check  for (n=0,1,2
                       must hold)
     tols  , input  ,  An array containing the tolerances:
                        [0]: distance tolerance for G0 closure
                        [1]: angular tolerance (degrees) for  NL_G1 closure
                             (required only if n>0)
                        [2]: curvature tolerance for NL_G2 closure (requir-
                             ed only if n>1)
     gnflg , output ,  Flag: 
                        NL_NO : curve not Gn continuously closed
                        NL_YES: curve is Gn closed 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsClosedContinuity( NL_CURVE *cur, NL_INDEX n, NL_REAL *tols, NL_FLAG *gnflg )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvIsClosedContinuity");

    NL_FLAG error = NL_NO;

    NL_POINT D1[3], D2[3];

    NL_VECTOR V;

    NL_REAL kap1, kap2, dd, dn, us, ue;

    /* Check for bad input data */

    if( n LT 0 OR n GT 2 )
        NL_ERROR( NL_INP_ERR );

    *gnflg = NL_NO;

    /* Check continuity */

    N_CrvGetParamBounds( cur, &us, &ue );

    error = N_CrvDerivs( cur, us, NL_LEFT, n, D1 );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvDerivs( cur, ue, NL_RIGHT, n, D2 );

    if( error EQ NL_YES )
        NL_OUT;

    /* check G0 */

    N_DistPtPt( D1[0], D2[0], &dd );

    if( dd GT tols[0] )
        NL_OUT;

    /* check NL_G1 */

    if( n GT 0 )
    {
        error = N_VectorsAngle( D1[1], D2[1], &dd );

        if( error EQ NL_YES )
            NL_OUT;

        else if( dd GT tols[1] )
            NL_OUT;
    }

    /* check NL_G2 */

    if( n GT 1 )
    {
        N_VectorCross( D1[1], D1[2], &V );
        N_VectorMagnitude( V, &dn );
        N_VectorMagnitude( D1[1], &dd );
        dd = dd * dd * dd;

        if( N_FloatOpIsBad( dn, dd, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        kap1 = dn / dd;

        N_VectorCross( D2[1], D2[2], &V );
        N_VectorMagnitude( V, &dn );
        N_VectorMagnitude( D2[1], &dd );
        dd = dd * dd * dd;

        if( N_FloatOpIsBad( dn, dd, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        kap2 = dn / dd;

        if( fabs( kap1 - kap2 )GT tols[2] )
            NL_OUT;
    }

    /* passed all the tests */

    *gnflg = NL_YES;

    /* End NURBS and Exit */
    EXIT:

    return (error);
} /* end N_CrvIsClosedContinuity */

/*******************************************************************//**


   DESCRIPTION:

     This routine  creates a line as a non-rational  NURBS curve. The
     line is given by its start and end points. A typical calling ex-
     ample is:

       NL_POINT   P, Q;
       NL_CURVE   cur;
       NL_STACKS  S;
       ...
       (get P and Q);
       ...
       N_CrvInitArrays(&cur);
       N_CrvLineFrom2Pts(P,Q,&cur,&S);

     If memory is not available, it is allocated  inside the routine. 
     If it is available, no memory is allocated, however, the routine
     checks for the proper amount by  looking at the  highest indexes 
     in cur's polygon and knot vector objects.


   ACCESS:
   
     P   , input  ,  Start point
     Q   , input  ,  End point
     cur , output ,  NURBS line
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvLineFrom2Pts( NL_POINT P, NL_POINT Q, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvLineFrom2Pts");

    NL_FLAG error = NL_NO;

    NL_REAL *U;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, 1, 1, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get control points */

    N_PtToCPt( P, &Pw[0] );
    N_PtToCPt( Q, &Pw[1] );

    /* Get the knots */

    U[0] = 0.0;
    U[1] = 0.0;
    U[2] = 1.0;
    U[3] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvLineFrom2Pts */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine translates a NURBS curve to obtain a new curve, i.e.
     the translation is not in place. A typical calling example is:

       NL_VECTOR  T;
       NL_CURVE   curP, curQ;
       NL_STACKS  SQ;
       ...
       (define curP and translation vector T);
       ...
       N_CrvInitArrays(&curQ)
       N_CrvFromCrvTranslation(&curP,T,&curQ,&SQ);

     If memory is  available, curQ is not  initialized and the routine
     assumes that  memory allocation has been done. However, it checks  
     for the proper amount by looking at the highest indexes in curQ's  
     knot vector and  polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     T    , input  ,  Translation vector
     curQ , output ,  Translated curve
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFromCrvTranslation( NL_CURVE *curP, NL_VECTOR T, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFromCrvTranslation");

    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_INDEX i, n, m;

    NL_REAL *UQ, *UP;

    NL_CPOINT *Pw, *Qw;

    /* Check for memory */

    if( NOT N_CrvIs3d( curP ) )
        N_Crv2dTo3d( curP );

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );

    /* Get translated control points and the knots */

    for ( i = 0; i <= n; i++ )
        N_TranslateCPt( Pw[i], T, &Qw[i] );

    for ( i = 0; i <= m; i++ )
        UQ[i] = UP[i];

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvFromCrvTranslation */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine computes the 'average' error for the points to the 
     closest point on the curve. The 'average' is the square root of the
     average sum of distances squared. The points should be sequential 
     (not random) and fairly evenly spaced (for Newton's method  to work).

     The maximum distance error and the parameter are returned. 
 
       NL_POINT   *P;
       NL_INDEX   m;
       NL_CURVE   cur;
       NL_REAL    *Tdist;
       NL_REAL    *Tdmax;
       NL_REAL    *U


       ...
(define array P, and cur, and return Tdist, Tdmax, U);
 
   ACCESS:
   
     P   , input  ,  Points
     m   , input  ,  Highest index in P
     cur , input  ,  Curve fitted to points
    Tdist, output ,  Average distance, point(s) to curve
    Tdmax, output .  Maximum distance, point to curve
    U    , output ,  Parameter value in curve for worst point 
    maxIdx, output,  index of worst point
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_PntCrvFitError( NL_POINT *P, NL_INDEX m, NL_CURVE *cur, NL_REAL *Tdist, NL_REAL *Tdmax, NL_REAL *U, NL_INDEX *maxIdx )
{
    /* Evaluate goodness of fit of curve to points */
    NL_FLAG error;
    NL_POINT Q0, Q1;
    NL_REAL Tsumdsq, Tdsq, Tmax;
    NL_INDEX idxMax;
    NL_PARAMETER u0, u1, ul, ur, udmax = 0.0;
    NL_REAL Udelta;
    NL_INDEX i;
    NL_REAL alpha = 0.3; /* smoothing for Udelta */

    NL_CURVEJET xPOC;   /* For the data points, projected to the curve. */
    NL_GCPTEMP crvData; /* Temp data for NLib curve inversion routines. */

    NL_BOOLEAN ok;
    NL_STACKS localStacks;

    /* init output */
    Tsumdsq = *Tdist = 0.0;
    Tmax = *Tdmax = -1.0;
    *U = -1;
    idxMax = *maxIdx = -1;

    /* Set up the CurveJet, and the temp data for the curve */
    N_crvJetInit( &xPOC, cur );
    N_CrvProjectionInitArrays( &crvData );
    error = NL_NO;

    if( m <= 0 )
        NL_OUT;
    N_CrvGetParamBounds( cur, &ul, &ur );
    Udelta = (ur - ul) / m;

    u1 = ul - Udelta;

    for ( i = 0; i <= m; i++ )
    {
        /* next guess starts at previous found + delta */
        u0 = u1 + Udelta;

        if( u0 > ur )
            u0 = ur;
        Q0 = P[i];

/* find closest point Q,u1 on curve to P iterating from u0 */
#if 0

        error = N_CrvClosestPt( cur, Q0, u0, Tol, toc, &u1, &Q1 );

        if( error )
            NL_OUT;
#else

        N_crvJetSetParam( &xPOC, u0 );
        ok = N_crvJetRelax( &xPOC, &Q0, &crvData, &localStacks );

        if( !ok )
            NL_QUIT;

        u1 = xPOC.param;
        N_crvJetPosCopy( &xPOC, &Q1 );

#endif

        Tdsq = (Q0.x - Q1.x) * (Q0.x - Q1.x) + (Q0.y - Q1.y) * (Q0.y - Q1.y);

        if( Tdsq > Tmax )
        {
            Tmax = Tdsq;
            udmax = u1;
            idxMax = i;
        }
        Tsumdsq += Tdsq;

        /* exponentially smooth the Udelta */
        Udelta = alpha * (u1 - u0) + (1.0 - alpha) * Udelta;
    }

    /* Average deviation ('average' here is sqrt of ave sum of sqs)
       first and last points are 'on' */
    *Tdist = sqrt( Tsumdsq / m );
    *Tdmax = sqrt( Tmax );
    *U = udmax;
    *maxIdx = idxMax;

    EXIT:

    return (error);
} /* end N_PntCrvFitError  */

/*******************************************************************//**


   DESCRIPTION:

   This routine checks the fit of a set of points to a NURBS surface.

   A typical calling example is as follows:

     NL_POINT   *dataPoints;
     NL_INDEX   dataPointCount;
     NL_SURFACE *srfPtr;
     NL_REAL    aveError;
     NL_REAL    maxError;
     NL_REAL    *allErrors;
     ...
     (get dataPoints and srfPtr, allocate maxErrors if desired)
     ...
     N_PntSrfFitError( dataPoints, dataPointCount, srfPtr,
       &aveError, &maxError, allErrors );


  ACCESS:

   dataPoints    , input ,  array of data points to be checked
   dataPointCount, input ,  highest index in dataPoints
   srfPtr        , input ,  approximating surface
   aveError      , output , average distance of points to the surface
   maxError      , output , maximum distance of points to the surface
   allErrors     , output , distances for each data point, or NULL


  RETURN CODES:

   0 : No error
   1 : Error saved in NL_ERROR


  ***********************************************************************/

NL_FLAG N_PntSrfFitError( NL_POINT xPts [], NL_INDEX xPtsCount,                /* Highest index in xPts */
NL_SURFACE *srfPtr, NL_REAL *aveError, NL_REAL *maxError, NL_REAL allErrors [] /*  May be null.  */
)
{
    NL_FLAG error = NL_NO;
    NL_BOOLEAN ok;

    NL_SURFJET sJet; /* data Point On Surface */
    NL_REAL thisError;

    NL_REAL *xParams_u;
    NL_REAL *xParams_v;
    NL_POINT *projPts;
    NL_INDEX projCount;

    NL_INDEX k;
    NL_INDEX numCounted;

    NL_STACKS localStacks;

    N_InitNurbs( &localStacks );

    /* Project all data points to the surface,
       to get guesses for the uv parameters */

    /* Note: for the initial projection to the surface, we're currently
     * using N_SrfProjectPts.  It's not exactly what we want, but seems to be
     * the closest thing in NLib.
     * For one thing, it says it requires the surface and points to be
     * "simple", which means generally flat, able to be projected to a
     * plane without overlap.  On preliminary tests, however, it seems
     * to work well enough on a closed surface.  We should write something
     * analogous to the curve routine N_CrvClosestPtMultiple.
     */
    error = N_SrfProjectPts( srfPtr, xPts, xPtsCount, NL_NO, NL_NO, 0.001, &projPts, &xParams_u, &xParams_v, &projCount, &localStacks );

    if( error == NL_YES )
        NL_OUT;

    N_surfJetInit( &sJet, srfPtr );

    *aveError = *maxError = 0;
    numCounted = 0;

    for ( k = 0; k <= xPtsCount; k++ )
    {
        /* find foot on the surface for this data point */
        N_surfJetSetParam( &sJet, xParams_u[k], xParams_v[k] );

        ok = N_surfJetRelax( &sJet, &( xPts[k] ) );

        if( ok != NL_TRUE )
        {
            error = NL_YES;

            if( allErrors != NULL )
                allErrors[k] = -1;
            continue; /* for now, just skip this point. */
        }

        N_DistPtPt( xPts[k], *( N_surfJetPos( &sJet ) ), &thisError );

        *aveError += thisError;
        numCounted++;

        if( thisError > *maxError )
            *maxError = thisError;

        if( allErrors != NULL )
            allErrors[k] = thisError;
    }

    if( numCounted > 0 )
        *aveError /= numCounted;

    EXIT:
    N_EndNurbs( &localStacks );

    return error;
} /* end N_PntSrfFitError */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This curve routine modifies a curve so that it interpolates a
     given point, optionally at a specified parameter value.  
     A typical calling example is:

       NL_CURVE   crv;
       NL_POINT   pt;
       NL_BOOLEAN useFixParam, useGuessParam;
       NL_REAL    fixParam, guessParam;
       ...
(specify useFixParam, useGuessParam, fixParam and guessParam)
       ...
       N_CrvFitToPt(&crv, pt,
         useFixParam, fixParam, useGuessParam, guessParam);

     Note, useFixParam has priority over useGuessParam: if useFixParam
     is true, then guessParam is ignored.


   ACCESS:
   
     crvPtr         , input ,  NURBS curve
     pt             , input ,  Point to be interpolated
     useFixParam    , input ,  If true, interpolate at fixParam,
                               otherwise at closest point on curve
     fixParam       , input ,  Interpolate at fixParam, if useFixParam
     useGuessParam  , input ,  If true, use guessParam as closest - point guess
                               Ignored if useFixParam is true.
     guessParam     , input ,  Guess parameter to use, if useGuessParam


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CrvFitToPt
 (NL_CURVE  * crvPtr,        // in : TgtCrv to edit
  NL_POINT    pt,            // in : TgtPt to interpolate
  NL_BOOLEAN  useFixParam,   // in : TRUE  = use FixParam for PtOnCurve to fit to Point
                             //      FALSE = use CrvParam of input CrvPt closest to TgtPt
  NL_REAL     fixParam,      // in : if useFixParam == TRUE, CrvParamPt to fit to Point, else not used
  NL_BOOLEAN  useGuessParam, // in : TRUE  = use guessParam as NR closest point guess for finding the closest point (runs locally and faster)
                             //      FALSE = find closest point without an initial closest point guess (runs globally and slower)
  NL_REAL     guessParam )   // in : if(useFixParam == FALSE && useGuessParam == TRUE) the guess point passed to NR to find closest point on Curve
                             //      else not used
{
    NL_CURVEJET poc;
    NL_VECTOR disp;
    NL_CPOINT dispCPt;
    NL_REAL sumSquares;

    NL_INDEX numCtrlPts, numKnots;
    NL_DEGREE deg;
    NL_CPOINT *ctrlPts;
    NL_REAL *knots;
    NL_KNOTVECTOR *knotVecPtr;
    NL_INDEX span;

    NL_REAL *basisFcns;

    NL_INDEX cPtIdx, basisIdx;

    NL_STACKS S;

    NL_BOOLEAN ok;
    NL_FLAG error = NL_NO;

    /* Start: init stacks and curveJet */

    N_InitNurbs( &S );

    N_crvJetInit( &poc, crvPtr );

    /* Find the corresponding point on the curve.  */

    if( useFixParam )
        N_crvJetSetParam( &poc, fixParam );

    else if( useGuessParam )
        N_crvJetSetParam( &poc, guessParam );

    if( !useFixParam )
    {
        ok = N_crvJetRelax( &poc, &pt, NULL, NULL );

        if( ok != NL_TRUE )
            NL_QUIT;
    }

    /* Find the displacement by which we have to move the curve */
    N_Diff2Pts( pt, *( N_crvJetPos( &poc ) ), &disp );

    /* Check whether we're already there */
    N_VectorDotRef( &disp, &disp, &sumSquares ); /* dot product */

    if( sumSquares < NL_MTOL * NL_MTOL )
        NL_OUT;

    /*  Ok, get to work.  Get curve details.  */
    N_CrvGetCPtsDegreeAndKnots( crvPtr, &numCtrlPts, &ctrlPts, &deg, &numKnots, &knots );
    /*  We'll need the curve's knot vector. */
    N_CrvGetKnotVector( crvPtr, &knotVecPtr );

    /*  Get the values of the basis functions at this parameter value. */
    basisFcns = N_AllocReal1dArray( deg, &S ); /* allocates deg + 1 */
    N_BasisEval( knotVecPtr, deg, poc.param, NL_LEFT, basisFcns, &span );

    /* Here's the trick.  We need to move the involved control points
    * such that the sum of the movements weighted by their basis functions
    * equals disp.  One solution would be to move them all by disp, but we
    * can localize the change better than that.  We will make all of the
    * point moves parallel to disp, so it becomes a scalar problem, just
    * a matter of dividing up disp among the weighted control point moves.
    * Control points with larger basis fucntions
    * have greater influence on the location of the curve.  To minimize
    * the overall effect on the curve, points with larger basis functions
    * should be moved farther.  At one extreme, if a point has a zero
    * basis function, it should not be moved at all.  Let's make the
    * size of the point move proportional to the basis function: say
    * b_i is twice b_j, then move cpt_i twice as far as cpt_j; thus
    * the movement of cpt_i will have four times the effect of cpt_j.
    * To make this add up, doing the math, we get that the distance to
    * move cpt_i is  disp * b_i /(sum of b_j squared).
    */

    sumSquares = 0;

    for ( basisIdx = 0; basisIdx <= deg; basisIdx++ )
    {
        sumSquares += basisFcns[basisIdx] * basisFcns[basisIdx];
    }

    /* convert displacement vector to NL_CPOINT */
    N_PtToCPt( disp, &dispCPt );

    for ( cPtIdx = span - deg; cPtIdx <= span; cPtIdx++ )
    {
        basisIdx = cPtIdx - span + deg;

        /* Add the fraction of disp to each involved control point */
        N_VectorBlendCPt( basisFcns[basisIdx] / sumSquares, dispCPt, &( ctrlPts[cPtIdx] ) );
    }

    EXIT:

    N_EndNurbs( &S );

    return error;
} /* end N_CrvFitToPt */

/*******************************************************************//**


    DESCRIPTION:

      This fitting routine computes a cubic NURBS curve approximating a
      given set of points, with specified points interpolated at specified
      parameter values.  Parameter values of the non-interpolated data
      points are not required.  Optionally, knot values to be used for
      the knot vector for the curve may be supplied.  Note, this is not
      a NL_KNOTVECTOR, it is just a list of increasing knot values, with
      no duplicates.  Also, if no knot vector is provided, the number of
      control points in the resulting curve may optionally be specified.
      If both knots and a control point count are specified, the knots
      will be used and the control point count ignored.
      Also, if there are fixed points, the number of control points
      specified by knots or control point count may not be exact, due
      to splitting up the points among the segments.

      Note: The resulting curve is suitable as input to
      N_FitCrvApproxPts().

      The resulting curve will be parameterized on the domain [0,1].
      The input fixParams must be within this domain, and in increasing order.

      A typical calling example is as follows:

        NL_POINT   dataPoints[];
        NL_INDEX   dataPointCount;
        NL_INDEX   fixIndices[];
        NL_PARAMETER fixParams[];
        NL_INDEX   fixCount;
        NL_REAL    givenKnots[];
        NL_INDEX   givenKnotCount;
        NL_INDEX   ctrlPtCount;
        NL_CURVE   crv;
        NL_STACKS  curveStackPtr;
        ...
        (get dataPoints, set up fixParams and fixIndices, and givenKnots)
        N_CrvInitArrays( &crv );
        ...
        N_FitCrvApproxPts( dataPoints, dataPointCount,
            fixIndices, fixParams, fixCount,
            ctrlPtCount, givenKnots, givenKnotCount,
            &crv, &curveStacks );


    ACCESS:

      dataPoints    , input ,  array of data points
      dataPointCount, input ,  highest index in dataPoints
      fixIndices    , input ,  indices in dataPoints of positions
                               to be interpolated
      fixParams     , input ,  parameters corresponding to the points
                               to be interpolated
      fixCount      , input ,  highest index in fixIndices and fixParams
      givenKnots    , input ,  knot values to use as knot vector, or Null.
                               Note, not a NL_KNOTVECTOR; no duplicates
      givenKnotCount, input ,  highest index in givenKnots
      ctrlPtCount   , input ,  highest index in control polygon of new curve,
                               if >= 2 and no knots are given.
      crv           , output ,  approximating curve
      curveStackptr , input/output ,  ptr to crv's stack


    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

    ***********************************************************************/

NL_FLAG N_ApproxCrvToPts( NL_POINT xPts [], NL_INDEX xPtCount, NL_INDEX fixIndices [], /* indices into xPts of fixed points */
NL_PARAMETER fixParams [], NL_INDEX fixCount, NL_REAL givenKnots [], NL_INDEX givenKnotCount, NL_INDEX ctrlPtCount, NL_CURVE *crvPtr, NL_STACKS *curveStackPtr )
{
    /* Programming note: variable names "...Count" mean "highest index
     * in array", which is one less than the number of items.
     * "num..." would be the actual count.
     */

    NL_FLAG error;
    NL_FLAG flg;

    NL_INDEX numSegs;
    NL_INDEX segNum, i;

    NL_INDEX dataStart, dataEnd;
    NL_INDEX startIdx, endIdx;
    NL_INDEX segDataCount;

    /* Data for the new curve */
    NL_INDEX numCtrlPts;
    NL_CPOINT *ctrlPts;
    NL_INDEX mainKnotCount;
    NL_REAL *mainKnots;
    NL_DEGREE deg = 3;      /* We create a cubic. */

    NL_INDEX crvPtIdx, crvKnotIdx;

    NL_CURVE ** segCurves; /* a curve for each segment */

    /* segment curve data */
    NL_INDEX segCPtCount, segKnotCount;
    NL_CPOINT *segCtrlPts;
    NL_REAL *segKnots;

    NL_LINESEG lineSeg1, lineSeg2;
 
    NL_POINT intPt, intPt2;
    NL_POINT tmpPt1, tmpPt2;
    NL_PARAMETER ta, tb;

    /* Used if givenKnots are passed in: */
    NL_INDEX ktIdx0 = 0, ktIdx1 = 0;
    NL_KNOTVECTOR segKnotVec;

    NL_VECTOR Ds, De;
    NL_POINT CD[2];
    NL_CURVE CubicCur;
    NL_POINT *Pset4;

    NL_STACKS localStacks;

    /* Start of executable code */

    N_InitNurbs( &localStacks );

    /* Input curve must be null. */
    error = NL_YES;

    /* initialize line segments */
    tmpPt1.x = 0.0; tmpPt1.y = 0.0; tmpPt1.z = 0.0;
    tmpPt2.x = 0.0; tmpPt2.y = 0.0; tmpPt2.z = 0.0;
    N_CreateLinePtPt( &lineSeg1, tmpPt1, tmpPt2, NL_UNBOUNDED );
    N_CreateLinePtPt( &lineSeg1, tmpPt1, tmpPt2, NL_UNBOUNDED );

    if( N_CrvAreArraysNULL( crvPtr ) == NL_FALSE )
        return error;

    if( fixCount < -1 )
        fixCount = -1; /* no fixes */

    numSegs = fixCount + 2;

    /* Allocate and initialize an array of CURVEs */
    segCurves = N_AllocArrayCrvPtrsAndData( numSegs - 1, NL_YES, &localStacks );

    numCtrlPts = 0;

    dataEnd = 0;

    segKnots = NULL;

    if( givenKnots != NULL && givenKnotCount > 1 )
    {
        ktIdx0 = ktIdx1 = 0;
        /* big enough for any subset: */
        segKnots = N_AllocReal1dArray( givenKnotCount + 2 * deg, &localStacks );
    }

    /* Create an approximating curve for each segment (between fixes). */

    for ( segNum = 0; segNum < numSegs; segNum++ )
    {
        dataStart = dataEnd;
        dataEnd = (segNum < numSegs - 1) ? fixIndices[segNum] : xPtCount;

        /* Create a curve out of this segment, interpolate the ends.*/
        N_CrvInitArrays( &CubicCur );

        if( segNum == 0 )
        {
            /* Use cubic Bezier to get start and end tangent directions
               for the whole curve  */
            Pset4 = &xPts[dataStart];
            error = N_FitCrvInterp( Pset4, 3, 3, NL_CHORDLENGTH, &CubicCur, &localStacks );
            N_CrvDerivs( &CubicCur, 0.0, NL_LEFT, 1, CD );
            N_VectorCopy( CD[1], &Ds );
        }
        else
        {
            /* Interior tangents are not critical, just use F-mill type thing */
            N_Diff2Pts( xPts[dataStart + 1], xPts[dataStart - 1], &Ds );
        }

        if( segNum == numSegs - 1 )
        {
            /* cubic Bezier for tangent directions at end  */
            Pset4 = &xPts[dataEnd - 3];
            error = N_FitCrvInterp( Pset4, 3, 3, NL_CHORDLENGTH, &CubicCur, &localStacks );
            N_CrvDerivs( &CubicCur, 1.0, NL_LEFT, 1, CD );
            N_VectorCopy( CD[1], &De );
        }
        else
        {
            /* Interior tangents: F-mill */
            N_Diff2Pts( xPts[dataEnd + 1], xPts[dataEnd - 1], &De );
        }

        /* least-squares NURBS curve approximation, given a span-count */

        if( numSegs <= 1 )
            segDataCount = xPtCount;

        else if( segNum == 0 )
            segDataCount = fixIndices[0];

        else if( segNum == numSegs - 1 )
            segDataCount = xPtCount - fixIndices[segNum - 1];

        else
            segDataCount = fixIndices[segNum] - fixIndices[segNum - 1];

        /* parameter interval for this segment:*/
        ta = (segNum == 0) ? 0.0 : fixParams[segNum - 1];
        tb = (segNum == numSegs - 1) ? 1.0 : fixParams[segNum];

        /* If knots are provided, use N_FitCrvLstSqEnds(), else use N_FitCrvApproxTangents(). */

        if( segKnots != NULL )
        {
            ktIdx0 = ktIdx1;

            /* find end knot: ktIdx1 */
            if( segNum == numSegs - 1 )
                ktIdx1 = givenKnotCount;
            else
            {
                while( givenKnots[ktIdx1] < tb - NL_PTOL && ktIdx1 <= givenKnotCount )
                    ktIdx1++;

                if( ktIdx1 > givenKnotCount )
                    NL_QUIT;                          /* input error */
            }
            segKnotCount = ktIdx1 - ktIdx0 + 2 * deg; /* highest index */

            for ( i = 0; i <= segKnotCount; i++ )
            {
                segKnots[i] = i <= deg ? givenKnots[ktIdx0] : i < segKnotCount - deg ? givenKnots[ktIdx0 + i - deg] : givenKnots[ktIdx1];
            }

            N_KnotVectorFromRealArray( &segKnotVec, segKnots, segKnotCount );

            error = N_FitCrvLstSqEnds( &( xPts[dataStart] ), segDataCount, &Ds, &De, NL_TANGENT, &segKnotVec, deg, segCurves[segNum], &localStacks );

            segCPtCount = segKnotCount - deg - 1;
        }
        else
        {
            if( ctrlPtCount > 1 )
            {
                /* prorate this segment's control point count by how many
                   data points are in this segment */
                NL_REAL ratio = (NL_REAL)segDataCount / (NL_REAL)xPtCount;
                segCPtCount = (NL_INDEX)( ratio * ctrlPtCount ) + 1;
            }
            else
            {
                /* Calculation of segCPtCount given segDataCount is from */
                /* Maron, "Numerical Analysis:  A Practical Approach",   */
                /* McMillan, 1982, Sec. 5.3D.  */
                /* (Explicit casts to silence warning messages.) */

                segCPtCount = (int)( 2.0 *sqrt( (NL_REAL)segDataCount ) - 2.0 );

                /* but for our application, this works better in practice: */
                segCPtCount *= 2;
            }

            if( segCPtCount < deg + 1 )
                segCPtCount = deg + 1;

            if( segCPtCount > segDataCount - deg - 1 )
                segCPtCount = segDataCount - deg - 1;

            error = N_FitCrvApproxTangents( &( xPts[dataStart] ), segDataCount, segCPtCount, deg, &Ds, &De, NL_TANGENT, NL_CHORDLENGTH, segCurves[segNum], &localStacks );
        }

        if( error != NL_NO )
            NL_OUT;

        /* Reparameterize this curve to the appropriate interval */
        N_CrvReparam( segCurves[segNum], ta, tb );

        /* accumulate total point count */
        numCtrlPts += segCPtCount + 1;
    }

    /* Set up the main curve */
    /* At each fixed point, the two end points of each curve (4 total) */
    /*   will be replaced by a single point, so: */
    numCtrlPts -= (fixCount + 1) * 3;

    ctrlPts = N_AllocCPt1dArray( numCtrlPts - 1, curveStackPtr );

    mainKnotCount = numCtrlPts + deg;
    mainKnots = N_AllocReal1dArray( mainKnotCount, curveStackPtr );

    crvPtIdx = 0;   /* index into final ctrl pt array */
    crvKnotIdx = 0; /* index into final mainKnot array */

    for ( segNum = 0; segNum < numSegs; segNum++ )
    {
        /* get curve data for this segment curve */
        N_CrvGetCPtsDegreeAndKnots( segCurves[segNum], &segCPtCount, &segCtrlPts, &deg, &segKnotCount, &segKnots );

        if( segNum > 0 )
        {

            /*
             * Fill in the transition from the previous seg to this one.
             * Here is the ctrl-pt manipulation thing that removes the triple
             * (actually quadruple) knot between the two cubic segments.
             * We replace the two end points of each curve with a single point,
             * and put in a single knot, equal to the multiple end knots of
             * both curves.  The only trick is what that single point should be.
             * If the curves happened to meet with true NL_C2 continuity, then
             * the penultimate legs of the two control polygons would intersect,
             * and that intersection point would be used, and the shapes and
             * parameterizations of both curves would be preserved exactly.
             * They're not NL_C2 of course, so we can't expect such nice behavior.
             * A close approximation is provided by finding the closest approach
             * of the intersection of legs: say, the midpoint of the closest
             * approach on each leg.  Sometimes however, such as if there's
             * an inflection point nearby, the intersection could be miles
             * away.  In that case, we take the common end point, drop it to
             * both of the penultimate legs, and average those.  This will
             * give a close approximation to the point at the given parameter,
             * close enough to be successfully treated by N_CrvFitToPt().
             */

            /* convert from NL_CPOINT to NL_POINT */
            tmpPt1.x = segCtrlPts[1].x; /* 2nd & 3rd pts */
            tmpPt1.y = segCtrlPts[1].y;
            tmpPt1.z = segCtrlPts[1].z;
            tmpPt2.x = segCtrlPts[2].x;
            tmpPt2.y = segCtrlPts[2].y;
            tmpPt2.z = segCtrlPts[2].z;

            N_CreateLinePtPt( &lineSeg2, tmpPt1, tmpPt2, NL_UNBOUNDED );

            /* The int pt returned by N_IsectLineLine is the midpoint of the     */
            /* closest approach on both segs, which is just what we want. */

            error = N_IsectLineLine( lineSeg1, lineSeg2, &intPt, &ta, &tb, &flg );

            /* Check for int pt far away from segments */
            if( ta < -3 || ta > 4 )
                flg = NL_FALSE;

            if( tb < -3 || tb > 4 )
                flg = NL_FALSE;

            /* Problem intersecting lines?  Then drop end pt to both legs. */
            if( flg != NL_TRUE || error != NL_NO )
            {
                tmpPt1.x = segCtrlPts[0].x;
                tmpPt1.y = segCtrlPts[0].y;
                tmpPt1.z = segCtrlPts[0].z;

                error = N_ProjectPtLine( lineSeg1, tmpPt1, &intPt, &ta, &flg );

                if( error != NL_NO )
                    NL_QUIT;

                error = N_ProjectPtLine( lineSeg2, tmpPt1, &intPt2, &ta, &flg );

                if( error != NL_NO )
                    NL_QUIT;

                /* midpoint: */
                N_Combine2Pts( 0.5, intPt, 0.5, intPt2, &intPt );
            }

            /* Dump in this point */
            N_PtToCPt( intPt, &( ctrlPts[crvPtIdx] ) ); /* convert to NL_CPOINT */
            crvPtIdx++;
        }

        /* Dump the control points into the main list */
        startIdx = (segNum == 0) ? 0 : 2;
        endIdx = (segNum == numSegs - 1) ? segCPtCount : segCPtCount - 2;

        for ( i = startIdx; i <= endIdx; i++ )
        {
            ctrlPts[crvPtIdx] = segCtrlPts[i];
            crvPtIdx++;
        }

        /* Dump in the knots */
        startIdx = (segNum == 0) ? 0 : deg + 1;
        endIdx = (segNum == numSegs - 1) ? segKnotCount : segKnotCount - deg;

        for ( i = startIdx; i <= endIdx; i++ )
        {
            mainKnots[crvKnotIdx] = segKnots[i];
            crvKnotIdx++;
        }

        /* Set this up for next time */
        i = segCPtCount - 2; /* 2nd & 3rd to last point */
        /* convert from NL_CPOINT to NL_POINT */
        tmpPt1.x = segCtrlPts[i].x;
        tmpPt1.y = segCtrlPts[i].y;
        tmpPt1.z = segCtrlPts[i].z;
        tmpPt2.x = segCtrlPts[i + 1].x;
        tmpPt2.y = segCtrlPts[i + 1].y;
        tmpPt2.z = segCtrlPts[i + 1].z;

        N_CreateLinePtPt( &lineSeg1, tmpPt1, tmpPt2, NL_UNBOUNDED );
    }

    /* Ok, the curve data is all set up, create the curve. */

    /* Fill in the curve structure with ctrl pts and knots */
    error = N_CrvFromCPtsAndKnots( crvPtr, ctrlPts, crvPtIdx - 1, deg, mainKnots, crvKnotIdx - 1, curveStackPtr );

    /* Now make the curve actually interpolate each point */
    for ( i = 0; i <= fixCount; i++ )
    {
        N_CrvFitToPt( crvPtr, xPts[fixIndices[i]], NL_TRUE, fixParams[i], NL_FALSE, 0 );
    }

    EXIT:
    N_EndNurbs( &localStacks );

    return error;
} /* end N_ApproxCrvToPts */

/**********************************************************************/
/* N_FitCrvApproxPts: Curve approximation to points with         */
/*                         optional fixed points.                     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS curve approximating a given
     set of points.  Parameter values of the input points are not
     required.  Moreover, the points need not be ordered IF A STARTING
     NL_CURVE IS PROVIDED; if no starting curve is provided, the points are
     assumed to be ordered.  An input curve (non rational) may be provided
     as an initial guess.  If the input curve is null (N_iscurn), a starting
     curve will be created.  If the curve is null, two pieces of optional
     information may be passed in with the curve: the degree, and/or a
     knot vector.  If the curve's degree is >= 1, that degree will be
     used, otherwise a cubic curve is created.  If the curve's knot
     vector is not null, that knot vector will be used, and this will
     also determine the number of control points.  If the knot vector
     is null, and if controlPointCount is > 1, then the curve will be
     created with controlPointCount+1 control points, otherwise the
     number of control points will be based on the number of data points.
     
     Certain points on the curve may be specified as fixed, and the
     algorithm will not allow them to move.  The points are specified
     as parameter values.  A list of data points corresponding to the
     fixed parameter values may optionally be passed in; it is specified
     by an array of indices into the data point array.  A separate
     argument specifies whether the end points and end tangents should
     be fixed.

     An optional argument may be passed in, to return the error for
     each data point.  The values are the perpendicular distance from
     the data point to the curve.  If not NULL, this array must be
     allocated by the caller, to the same size as dataPoints.

     Constants for regularization terms may be passed in, if the curve
     is cubic (which is the default if no curve is passed in).
     Regularization helps to maintain a fair curve, and is used because
     in theory,the very "best" curve for the data, in terms of minimizing
     the distance to each data point, would be full of kinks and loops.
     The first term "alpha" tends to minimize the arc length of the
     resulting curve, keeping the curve short (to avoid big loops), and
     the other term "beta" minimizes curvature, keeping the curve "stiff"
     (to avoid wiggles).

     These regularization term values are passed in as a starting value
     and an ending value, because they work best when they decrease as
     the iteration proceeds.  If values are passed in negative, default
     values will be used.  Smaller values will result in a better fit,
     but the curve might develop kinks or loops.  The values should not
     be large: the current defaults are alpha_0 = alpha_1 = 0 (i.e., no
     alpha correction), and beta_0 = 0.0001, beta_1 = 0.000008.

     A typical calling example is as follows:

       NL_POINT   dataPoints[];
       NL_INDEX   dataPointCount;
       NL_CURVE   *crv;
       NL_INDEX   controlPointCount;
       NL_INDEX   numFixes;
       NL_INDEX   fixIndices[];
       NL_PARAMETER fixParams[];
       NL_FLAG    fixEndPts;
       NL_REAL    alpha_0, alpha_1;
       NL_REAL    beta_0, beta_1;
       NL_REAL    allErrors[];
       NL_STACKS  curveStacks;
       ...
       (get dataPoints, crv, set up fixIndices, fixParams)
       ...
       N_FitCrvApproxPts( dataPoints, dataPointCount, crv,
           controlPointCount,
           numFixes, fixIndices, fixParams, fixEndPts,
           alpha_0, alpha_1, beta_0, beta_1,
           allErrors, &curveStacks );


    ACCESS:

     dataPoints    , input ,  array of data points
     dataPointCount, input ,  highest index in dataPoints
     crv           , input/output ,  approximating curve
     controlPointCount , input , if crv is a null curve (N_CrvAreArraysNULL), and this
                                 argument is >= 1, then this will be the
                                 highest index in the control point array
                                 of the curve, which will be created here.
     controlDegree , input , if crv is a null curve (N_CrvAreArraysNULL), and this
                             argument is >= 1, then this will be the degree
                             of the curve, which will be created here.
     numFixes      , input ,  count of fixed points on crv
     fixIndices    , input ,  if not null, indices into the dataPoints array
                              of points to be interpolated
     fixParams     , input ,  if not null, parameters at which the curve
                              should not move
                              If both fixIndices and fixParams are provided,
                              then the points indicated by fixIndices will
                              correspond to the parameters in fixParams.
     fixEndPts     , input ,  0: no constraints;
                              1: fix the positions of the curve's end points;
                              2: fix end positions and tangent directions
                              3: fix end positions and first derivatives
     alpha_0, alpha_1, input, start and end values of alpha regularization term,
                              use default values if negative.
     beta_0,  beta_1 , input, start and end values of beta regularization term,
                              use default values if negative.
     allErrors     , output , error values for each data point, or NULL
     curveStacks   , input/output ,  crv's stack


    RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

    Possible Enhancements:
    - The stopping criteria ( ST_doneIterating() ) is pretty rudimentary,
      but it is giving good results, particularly with the use of
      bestErrorSD and bestErrorDist, and doesn't take too long.
    - If the end points are not fixed (fixEndPts <= 0), then the end
      points are free to move independently.  If the input curve is
      closed, it may be desireable to add the ability to keep the
      curve closed, without holding the end points fixed, just keeping
      them coincident.


    ***********************************************************************/
NL_FLAG N_FitCrvApproxPts
 (NL_POINT xPts [],                // in : TgtPts to approx with a curve
  NL_INDEX xPtsCount,              // in : highest index in TgtPts array:[PtsIndx = No. of Points-1]
  NL_CURVE *crvPtr,                // i/o: Ptr to Approximating curve being built, 
                                   //      NULL    = make a new Curve with (CptsMaxIndx + 1) Cpts
                                   //      NotNULL = Move InputCrv's existing CPts to make the best Approx Crv possible.
  NL_INDEX controlPointCount,      // in : When PtrToCurve = NULL, highest OutCurve ControlPoint [CPtsIndx = No. of CPts -1]
  NL_INDEX numFixes,               // in : No. of Points on OutCurve that are to be interpolated
  NL_INDEX fixIndices [],          // in : NotNULL = indices of Pts in XPts array to be treated as fixed, NULL to ignore.
  NL_PARAMETER fixParams [],       // in : NotNULL = Crv params to interpolate the fixed points, NULL to ignore.
  NL_FLAG fixEndPoints,            // in : 0 = No Constraints
                                   //      1 = Constrained Curve EndPts
                                   //      2 = Constrained Curve EndPts and EndTangent directions (Unconstrained speeds)
                                   //      3 = Constrained Curve EndPts and 1st Derivs            (constrained speeds)
  NL_REAL alpha_0,                 // in : start value of alpha (resist stretch) regularization term - neg values = use defaults
  NL_REAL alpha_1,                 // in : end value of alpha (resist stretch) regularization term   - neg values = use defaults
  NL_REAL beta_0,                  // in : start value of beta (resist bending) regularization term - neg values = use defaults
  NL_REAL beta_1,                  // in : end value of beta resist bending) regularization term   - neg values = use defaults
  NL_REAL allErrors [],            // out: error values for each point, NULL to ignore
  NL_STACKS *curveStacks )         // i/o: crv's stack
{
    NL_PRIVATE NL_STRING rname = _T("N_FitCrvApproxPts");
    static NL_REAL BetaMat[4][4]; /* See "Integrating Arc Length" at end of function */

    static constexpr NL_INDEX MAXITER = 100;
    NL_FLAG error = NL_NO;

    NL_INDEX numDataPoints = xPtsCount + 1; /* actual point count */

    NL_INDEX i, iter;

    /* error terms: dist squared, and their Squared Error term. */
    NL_REAL errorDist, errorSD;
    NL_REAL errorStop;

    NL_GCPTEMP  crvData;       /* Temp data for NLib curve inversion routines. */
    NL_INDEX    numCtrlPts;
    NL_CPOINT * ctrlPts;
    NL_DEGREE   deg;
    NL_INDEX    nKnots;
    NL_REAL   * knots;
    NL_POINT  * fixedPts = NULL;

    NL_CPOINT * bestCtrlPts;   /* Save the best results, in case we diverge. */
    NL_REAL     bestErrorSD = NL_BIGD;
    NL_REAL   * bestAllErrors = NULL;
    NL_BOOLEAN  ok;
    NL_STACKS   localStacks;
    NL_REAL   * xParams;       /* Parameters on the curve of data points (dropped) */
    NL_VECTOR * ctrlPtMoves;   /* calculated on each iteration */
    NL_REAL     damping;
    NL_CURVEJET xPOC;          /* For the data points, projected to the curve. */
    NL_REAL     alpha, beta;
    NL_REAL     iterFrac;

    /* Start of executable code */

    N_InitNurbs( &localStacks );

    /* init BetaMat: See "Integrating Arc Length" at end of file */
    BetaMat[0][0] = 2;
    BetaMat[0][1] = -3;
    BetaMat[0][2] = 0;
    BetaMat[0][3] = 1;
    BetaMat[1][0] = -3;
    BetaMat[1][1] = 6;
    BetaMat[1][2] = -3;
    BetaMat[1][3] = 0;
    BetaMat[2][0] = 0;
    BetaMat[2][1] = -3;
    BetaMat[2][2] = 6;
    BetaMat[2][3] = -3;
    BetaMat[3][0] = 1;
    BetaMat[3][1] = 0;
    BetaMat[3][2] = -3;
    BetaMat[3][3] = 2;

    /* init regularization terms */
    if( alpha_0 < 0 )
        alpha_0 = 0;

    if( alpha_1 < 0 )
        alpha_1 = 0;

    if( beta_0 < 0 )
        beta_0 = 0.0001;

    if( beta_1 < 0 )
        beta_1 = 0.000008;

    /* Check whether the caller provided a starting curve, or whether
     * we should figure one.
     */
    if( N_CrvAreArraysNULL( crvPtr ) == NL_TRUE )
    {
        /* Get some curve info */
        NL_CPOLYGON *cPoly;
        NL_KNOTVECTOR *knotVec;
        nKnots = -1;
        knots = NULL;

        N_CrvDetachPolygonKnot( crvPtr, &cPoly, &deg, &knotVec );

        if( knotVec != NULL )
            N_KnotVectorGetKnots( knotVec, &nKnots, &knots );

        /* figure degree */
        if( deg < 1 )
            deg = 3; /* default cubic */

        /* figure number of control points */
        if( knotVec != NULL && nKnots > 0 )
            numCtrlPts = nKnots - deg; /* actual count, not index */

        else if( controlPointCount >= 1 )
            numCtrlPts = controlPointCount + 1;

        else
        {
            /* Calculation of numCtrlPts given numDataPoints is from Maron, */
            /* "Numerical Analysis:  A Practical Approach", McMillan, 1982, */
            /*  Sec. 5.3D.  */
            /* (Explicit casts to silence warning messages.) */

            numCtrlPts = (int)( 2.0 *sqrt( (NL_REAL)numDataPoints ) - 2.0 );

            /* But in our application, this works better: */
            numCtrlPts *= 2;

            if( fixEndPoints >= 3 )
                numCtrlPts += 4; /* Four points are fixed. */

            else if( fixEndPoints >= 1 )
                numCtrlPts += 2; /* Only the interior points are unknowns */
        }

        if( numCtrlPts < deg + 1 )
            numCtrlPts = deg + 1;

        if( numCtrlPts > numDataPoints - deg - 1 )
        {
            if( knotVec != NULL || controlPointCount >= 1 )
            {
                NL_QUIT; /* Input error: too many ctrl pts indicated */
            }
            else
                numCtrlPts = numDataPoints - deg - 1;
        }

        if( deg == 3 )
        {
            /* this takes a list of knots, not NL_KNOTVECTOR: */
            NL_REAL *knotArray = (knots == NULL) ? NULL : knots + deg;
            error = N_ApproxCrvToPts( xPts, numDataPoints - 1, fixIndices, fixParams, numFixes - 1, knotArray, nKnots - 2 * deg, controlPointCount, crvPtr, curveStacks );
        }
        else
        {
            error = N_FitCrvApproxLstSq( xPts, numDataPoints - 1, numCtrlPts - 1, deg, NL_UNIFORM, crvPtr, curveStacks );

            if( error != NL_NO )
                NL_OUT;

            if( numFixes > 0 && fixIndices != NULL )
            {
                for ( i = 0; i < numFixes; i++ )
                {
                    N_CrvFitToPt( crvPtr, xPts[fixIndices[i]], NL_FALSE, 0, NL_FALSE, 0 );
                }
            }
        }

        if( error != NL_NO )
            NL_OUT;
    } // end CrvPtr is NULL check - so we have to build a curve with good guesses for CPt and knot counts

    /* get curve control points */
    N_CrvGetCPtsDegreeAndKnots( crvPtr, &numCtrlPts, &ctrlPts, &deg, &nKnots, &knots );

    /* Rename these variables as actual counts instead of "highest index":  */
    numCtrlPts++;
    nKnots++;

    /* We should have a curve at this point. */
    if( numCtrlPts < 2 )
        return NL_YES;

    xParams = N_AllocReal1dArray( numDataPoints - 1, &localStacks );

    if( xParams == NULL )
        NL_ERROR( NL_MEM_ERR );

    ctrlPtMoves = N_AllocPt1dArray( numCtrlPts - 1, &localStacks );

    if( ctrlPtMoves == NULL )
        NL_ERROR( NL_MEM_ERR );

    if( allErrors != NULL )
    {
        bestAllErrors = N_AllocReal1dArray( numDataPoints - 1, &localStacks );
    }

    /* Set up positions for fixed points */
    if( numFixes > 0 )
    {
        fixedPts = N_AllocPt1dArray( numFixes - 1, &localStacks );

        for ( i = 0; i < numFixes; i++ )
        {
            if( fixIndices != NULL )
                fixedPts[i] = xPts[fixIndices[i]];
            else
                N_CrvEval( crvPtr, fixParams[i], NL_LEFT, &( fixedPts[i] ) );
        }
    }

    /* Set up the temp data for the curve */
    N_CrvProjectionInitArrays( &crvData );

    bestCtrlPts = N_AllocCPt1dArray( numCtrlPts - 1, &localStacks );

    if( bestCtrlPts == NULL )
        NL_ERROR( NL_MEM_ERR );

    /*  init POC before use  */
    N_crvJetInit( &xPOC, crvPtr );

    /* We have to get initial parameter values for each data point. */
    for ( i = 0; i < numDataPoints; i++ )
    {
        N_crvJetReset( &xPOC );
        ok = N_crvJetRelax( &xPOC, &( xPts[i] ), &crvData, &localStacks );

        if( !ok ) {
        /*  Failure?  Perhaps exclude this point?
         *  No, if this fails on the initial curve, there's a real problem.
         *  Note that if the point is off the end of the curve, it will
         *  still return success; failure will probably happen only if
         *  the curve has a kink or something.  Moreover, this algorithm
         *  isn't likely to fix such a curve.  So we really need a better
         *  curve to start with.
         *  If the relaxation failed, there is a good chance that it will
         *  return its best effort, i.e., a parameter value near the
         *  solution, so things could well smooth out as we proceed.
         *  But if it's a real problem, something will probably fail
         *  properly down the line, or we just return a bad curve.
         *  So there's no reason not just to press on.
         */
        }

        xParams[i] = xPOC.param;
    }

    errorStop = NL_MTOL * numDataPoints; /* pretty tight: each pt within tol? */

    errorSD = errorDist = 2 * errorStop; /* just to get it started. */

    iter = 0;

    // itereate until Pt set is approximated
    while( !ST_doneIterating( iter, MAXITER, errorSD, errorDist, errorStop ) )
    {
        iter++;

        /* Set up alpha and beta: decrease as we proceed. */
        iterFrac = ((NL_REAL)iter - (NL_REAL)1) / 40.0;

        if( iterFrac > 1.0 )
            iterFrac = 1.0;
        alpha = alpha_0 + iterFrac * (alpha_1 - alpha_0);
        beta  = beta_0  + iterFrac * (beta_1 - beta_0);

        error = ST_findStep(crvPtr, 
                            BetaMat, 
                            fixEndPoints, 
                            numDataPoints, 
                            xPts, 
                            xParams, 
                            alpha,           /* Updated at each step */
                            beta,            /* Updated at each step */
                            ctrlPtMoves, 
                            &errorDist, 
                            &errorSD, 
                            allErrors, 
                            &crvData, 
                            &localStacks );   /* error measures returned */
        
        if( error != NL_NO )
            break; /* ST_findStep() will have set NL_ERROR */

        /*  Note, we're using errorSD to check progress, but we're returning
         *  errorDist.  That makes sense, because errorSD is what we're
         *  minimizing, and errorDist is what the caller is interested in.
         */
        if( errorSD < bestErrorSD )
        {
            bestErrorSD = errorSD;
            /* bestErrorDist = errorDist; */

            for ( i = 0; i < numCtrlPts; i++ )
                bestCtrlPts[i] = ctrlPts[i];

            if( allErrors != NULL && bestAllErrors != NULL )
            {
                for ( i = 0; i < numDataPoints; i++ )
                    bestAllErrors[i] = allErrors[i];
            }
        }

        /* Now update the control points according to the point moves: */
        damping = (iter < 40) ? (NL_REAL)iter / 40.0 : 1.0;

        for ( i = 0; i < numCtrlPts; i++ )
        {
            ctrlPts[i].x += damping * ctrlPtMoves[i].x;
            ctrlPts[i].y += damping * ctrlPtMoves[i].y;
            ctrlPts[i].z += damping * ctrlPtMoves[i].z;
        }

        /* Make it go through each fixed point.
         * This call should minimize control point moves to achieve that.
         */
        for ( i = 0; i < numFixes; i++ )
        {
            N_CrvFitToPt( crvPtr, fixedPts[i], NL_TRUE, fixParams[i], NL_FALSE, 0 );
        }
    }  // end itereation to move control points to fit curve to TgtPts

    /* Done iterating.  See whether we ended up diverging */

    if( errorSD > bestErrorSD )
    {
        for ( i = 0; i < numCtrlPts; i++ )
            ctrlPts[i] = bestCtrlPts[i];

        if( allErrors != NULL && bestAllErrors != NULL )
        {
            for ( i = 0; i < numDataPoints; i++ )
                allErrors[i] = bestAllErrors[i];
        }
    }

    EXIT:
    /*  Clean up  */
    N_EndNurbs( &localStacks );

    return error;

} // end N_FitCrvApproxPts()

/* ----------------------------------------------------------------- */
/*  End of main routine.  Helper routines and notes follow.          */
/* ----------------------------------------------------------------- */
NL_FLAG ST_findStep              // in : 
 (NL_CURVE   * cur,              // in : 
  NL_REAL      BetaMat[4][4],    // in : 
  NL_FLAG      fixEndPts,        // in : 
  NL_INDEX     numDataPoints,    // in : 
  NL_POINT     xPts [],          // in : 
  NL_REAL      xParams [],       // in : 
  NL_REAL      alpha,            // in :     /* must be initialized to previous-step values */
  NL_REAL      beta,             // in : 
  NL_VECTOR    ctrlPtMoves [],   // in : 
  NL_REAL    * errorDist,        // in : 
  NL_REAL    * errorSD,          // in : 
  NL_REAL      allErrors [],     // in : 
  NL_GCPTEMP * crvData,          // in : 
  NL_STACKS  * pocStacks )       // in :    /* Allocated in caller */
{
    NL_PRIVATE NL_STRING rname = _T("ST_findStep");

    /* Note: this will also work in 2 dimensions.
     * But it's not necessary; it works fine on planar data
     * (e.g., all z == 0), with dimension == 3.
     */
    int dim = 3;

    NL_REAL thisTerm;

    NL_INDEX i, j, k;
    NL_INDEX rowPtIdx, colPtJdx; /* For indexing "pointwise": bands of xyz. */
    NL_INDEX rowCoord, colCoord; /* For indexing within an xyz band. */
    NL_INDEX ctrlPtIdx;          /* For indexing into the control point array */
    NL_REAL d_k;
    NL_REAL sdTerm;              /*  the Squared Distance term,  d / ( d - rho )  */
    NL_REAL dotP;
    NL_REAL dotTan, dotNorm, dotOther;
    NL_REAL kappa_k;
    NL_REAL bMatTerm;

    NL_CURVEJET xPOC; /* data Point On Curve */

    NL_BOOLEAN retval;
    NL_VECTOR  ptDiff, kappaVec;
    NL_VECTOR  unitTan, unitNorm, unitOther;
    NL_REAL    uTan[3], uNorm[3], uOther[3];

    NL_INDEX      span, basisIdx, basisJdx;
    NL_KNOTVECTOR knotVec;
    NL_REAL     * basisFcns;

    NL_RMATRIX    rMat; /* The matrices */
    NL_REAL    ** AMat;
    NL_REAL     * bMat;

    NL_INDEX * idxArray; /* Pivoting array for matrix solvers */

    NL_STACKS  localStacks;

    NL_RMATRIX crvMat;  /* to help with the AMat terms: explanation below. */
    NL_REAL ** CMat;

    NL_INDEX numVars;
    NL_FLAG  error;

    /* curve data */
    NL_INDEX    numCtrlPts, nKnots;
    NL_DEGREE   deg;
    NL_CPOINT * ctrlPts;
    NL_REAL   * knots;

    NL_INDEX    numVarPts; /* perhaps a subset of numCtrlPts */

    NL_INDEX    numSpans, spn;

    NL_VECTOR   BVecPt;

    NL_VECTOR startTan, endTan; /* used if fixEndPts == 2 */

    /* For alpha correction: */
    NL_INDEX ktIdx;
    NL_REAL alphaMat[4][4];
    NL_REAL del, del1, del2;
    NL_REAL i00, i01, i02, i11, i12, i22;

    /* Init */
    N_InitNurbs( &localStacks );

    /* get curve data */
    N_CrvGetCPtsDegreeAndKnots( cur, &numCtrlPts, &ctrlPts, &deg, &nKnots, &knots );
    numCtrlPts++; /* (They return "highest index".) */
    nKnots++;

    /* For later: */
    knotVec.m = nKnots - 1;
    knotVec.U = knots;

    numVarPts = numCtrlPts;

    if( fixEndPts >= 1 )
        numVarPts -= 2;

    if( fixEndPts >= 3 )
        numVarPts -= 2;  /* Fix two points at each end */

    if( fixEndPts == 2 ) /* preserve these directions */
    {
        startTan.x = ctrlPts[1].x - ctrlPts[0].x;
        startTan.y = ctrlPts[1].y - ctrlPts[0].y;
        startTan.z = ctrlPts[1].z - ctrlPts[0].z;
        k = numCtrlPts - 1;
        endTan.x = ctrlPts[k].x - ctrlPts[k - 1].x;
        endTan.y = ctrlPts[k].y - ctrlPts[k - 1].y;
        endTan.z = ctrlPts[k].z - ctrlPts[k - 1].z;
    }

    N_crvJetInit( &xPOC, cur );

    basisFcns = N_AllocReal1dArray( deg, &localStacks );

    if( basisFcns == NULL )
        NL_ERROR( NL_MEM_ERR );

    /* Set up the A and b matrices, and init them to zero. */

    numVars = numVarPts * dim;

    error = N_SetRealMatrix( &rMat, numVars - 1, numVars - 1, NL_MT_FULL, numVars - 1, &localStacks );

    if( error != NL_NO )
        NL_OUT;

    N_GetRealMatrixPtr( &rMat, &AMat );

    bMat = N_AllocReal1dArray( numVars - 1, &localStacks );

    if( bMat == NULL )
        NL_ERROR( NL_MEM_ERR );

    for ( i = 0; i < numVars; i++ )
    {
        bMat[i] = 0;

        for ( j = 0; j < numVars; j++ )
            AMat[i][j] = 0;
    }

    /* Now the loop over the data points. */

    *errorDist = *errorSD = 0;

    for ( k = 0; k < numDataPoints; k++ )
    {
        /* find foot on cur for this data point */

        N_crvJetSetParam( &xPOC, xParams[k] ); /* should be a good guess */
        retval = N_crvJetRelax( &xPOC, &( xPts[k] ), crvData, pocStacks );

        if( retval != NL_TRUE )
            continue; /* for now, just skip this point. */

        /* For next iteration: */
        xParams[k] = xPOC.param;

        /* Find d and rho at this data point.
         * In order to check whether they have the same sign,
         * we'll need both of the actual vectors:
         * ptDiff, from foot to data pt,
         * and kappaVec, the curve's curvature vector at foot.
         */
        N_Diff2Pts( xPts[k], *( N_crvJetPos( &xPOC ) ), &ptDiff );

        N_crvJetCurvature( &xPOC, &kappaVec );

        N_VectorMagnitudeRef( &ptDiff, &d_k ); /* vector magnitude */
        N_VectorMagnitudeRef( &kappaVec, &kappa_k );

        *errorDist += d_k;

        if( allErrors != NULL )
            allErrors[k] = d_k;

        /* Get unit normal.  In the papers, this is e3, local z-axis. */
        if( d_k > NL_ZCTL )
            N_VectorScaleRef( &ptDiff, 1 / d_k, &unitNorm );

        else if( kappa_k > NL_ZCTL )
            N_VectorScaleRef( &kappaVec, -1 / kappa_k, &unitNorm );

        else
            unitNorm = NL_ZERO;

        /* From one of the papers, "Geometry of the squared distance
         * function to curves and surfaces" by Pottmann and Hofer
         * (T.R. No. 90 at TU Wien, Jan 2002), we divide rho by the
         * cosine of the angle between e3, which is ptDiff, and the
         * curvature vector, kappaVec.  Also, on pp. 8, 9 and 10,
         * they say that in practice, one can just take the absolute
         * values of d_k and rho, and make everything positive, and add.
         * This does indeed work better than setting d_k to zero
         * if it has the same sign as rho, as suggested in the
         * Wang-Pottmann-Liu paper.  So instead of all the sign
         * wrangling, just make everything positive and add.
         *
         * For space curves, replace rho by rho_1, which is
         * rho / cos(theta), where theta is the angle between
         * ptDiff and kappaVec.  After a bit of algebra, their SD term
         * d_k / ( d_k + rho_1 ) comes out to dotP / ( 1 + dotP ),
         * where dotP is the absolute value of ( ptDiff <dot> kappaVec ).
         */

        N_VectorDotRef( &ptDiff, &kappaVec, &dotP );

        if( dotP < 0 )
            dotP = -dotP;
        sdTerm = dotP / (dotP + 1);

#if NLIB_REMOVE
        /* (Leave out the sign wrangling: ) */
        /* But the way we have our vectors set up
         * (both point away from the curve):
         */

        cosTheta_times_kappa = -cosTheta_times_kappa;

        kappa_k = cosTheta_times_kappa; /* Correction for 3d. */
        /* Now check whether data point k is on the same side as
         * the center of curvature.  Negative if on opposite sides.
         */
        if( cosTheta_times_kappa > 0 )
            d_k = -d_k;

        /*
         *  The formula -- Eqn. 10, p. 7 of Wang-Pottmann-Liu -- would
         *  prefer d_k to be negative (taking rho_k to be positive always).
         *  In fact, if d_k is positive and >= rho_k, that means that the
         *  data point is beyond the center of curvature, which renders this
         *  sort of meaningless.  (It's also pretty much impossible: it would
         *  have had to relax to a relative maximum, instead of finding some
         *  closer point.)
         *  If d_k is positive and less than rho_k ( 0 < d < rho ) then the
         *  problem is that their term d / (d-rho) will be negative, and the
         *  whole error function could be negative as well.  In that case,
         *  they say -- rather arbitrarily it seems -- to ignore that term.
         *  This results in treating the curve as if it were linear: the same
         *  as if rho were infinite.  The term then reduces to what they called
         *  the Tangent Distance function, e_TD, which they observed to have
         *  fast but non-robust convergence.
         *
         *  Bottom line: just do this, it takes care of everything:
         */

        if( d_k > 0 )
            d_k = 0;
        * /

#endif // NLIB_REMOVE

        /*  We're going to need the values of the basis functions for the
         *  curve at this parameter value.
         */

        N_BasisEval( &knotVec, deg, xPOC.param, NL_LEFT, basisFcns, &span );

        N_crvJetDer1Copy( &xPOC, &unitTan );
        N_VectorNormalizeRef( &unitTan );

        /* Here's where their method actually happens.
         * (Note: leaving off a factor of 2.)
         *
         * Their basis vector e1 is unitTan, e3 is unitNorm,
         * and e2 is e3 <cross> e1, which we'll call unitOther.
         * (unitOther is sort of like the binormal, but not really.)
         * Then the x_i are e_i <dot> (C-X+D), i = 1,2,3.
         * (Note C-X is ptDiff, backwards.)
         *
         * We calculate ptDiff <dot> unit[Norm,Tan,Other] as the
         * part with no variables in it: no D, so it's for the B-vector.
         * Those are called dotNorm, dotTan, dotOther.
         */
        N_VectorCrossRef( &unitNorm, &unitTan, &unitOther );

        N_VectorDotRef( &ptDiff, &unitTan, &dotTan ); /* dot product */
        N_VectorDotRef( &ptDiff, &unitNorm, &dotNorm );
        N_VectorDotRef( &ptDiff, &unitOther, &dotOther );

        /* because ptDiff = X-P, they use P-X. */
        dotTan = -dotTan;
        dotNorm = -dotNorm;
        dotOther = -dotOther;

        /* cumulative error */
        *errorSD += sdTerm * dotTan * dotTan + dotOther * dotOther + dotNorm * dotNorm;

        /* NOTE: see comment "Setting up the A-matrix" at end of file. */

        error = N_SetRealMatrix( &crvMat, 2, 2, NL_MT_FULL, 2, &localStacks );
        N_GetRealMatrixPtr( &crvMat, &CMat );

        CMat[0][0] = sdTerm * unitTan.x * unitTan.x + unitOther.x * unitOther.x + unitNorm.x * unitNorm.x;
        CMat[0][1] = sdTerm * unitTan.x * unitTan.y + unitOther.x * unitOther.y + unitNorm.x * unitNorm.y;
        CMat[0][2] = sdTerm * unitTan.x * unitTan.z + unitOther.x * unitOther.z + unitNorm.x * unitNorm.z;
        CMat[1][0] = CMat[0][1];
        CMat[1][1] = sdTerm * unitTan.y * unitTan.y + unitOther.y * unitOther.y + unitNorm.y * unitNorm.y;
        CMat[1][2] = sdTerm * unitTan.y * unitTan.z + unitOther.y * unitOther.z + unitNorm.y * unitNorm.z;
        CMat[2][0] = CMat[0][2];
        CMat[2][1] = CMat[1][2];
        CMat[2][2] = sdTerm * unitTan.z * unitTan.z + unitOther.z * unitOther.z + unitNorm.z * unitNorm.z;

        /*  (Say 3-dimensional cubic, for this discussion.)
         *  We're filling in a 12 x 12 square within AMat (see comment
         *  "Setting up the A-Matrix").  We'll do it in chunks of 3x3:
         *  the outer iteration will be on the number of involved
         *  control points (4x4), and within that we'll do the 3x3.
         */

        /* Can't index ( [i] ) into VECTORs, so do this: */
        uTan[0] = unitTan.x;
        uNorm[0] = unitNorm.x;
        uOther[0] = unitOther.x;
        uTan[1] = unitTan.y;
        uNorm[1] = unitNorm.y;
        uOther[1] = unitOther.y;
        uTan[2] = unitTan.z;
        uNorm[2] = unitNorm.z;
        uOther[2] = unitOther.z;

        /* Note: only (deg+1) of the basis fcns are non-zero.  */
        /* Indexing here is taken from N_CrvEval.  */

        for ( i = span - deg; i <= span; i++ )
        {
            rowPtIdx = i;

            basisIdx = i - span + deg;

            if( fixEndPts >= 1 )
            {
                if( span == deg && basisIdx == 0 )
                {
                    /* first span, first basis fcn: means first ctrl point. */
                    continue;
                }

                if( span == nKnots - deg - 2 && basisIdx == deg )
                {
                    /* last span, last basis fcn: means last ctrl point. */
                    continue;
                }

                /*  shift down one. */
                rowPtIdx--;
            }

            if( fixEndPts >= 3 ) /* Also check 1st points in from ends */
            {
                if( (span == deg && basisIdx == 1) || (span == deg + 1 && basisIdx == 0) )
                {
                    /* means second ctrl point. */
                    continue;
                }

                if( (span == nKnots - deg - 2 && basisIdx == deg - 1) || (span == nKnots - deg - 3 && basisIdx == deg) )
                {
                    /* next to last ctrl point. */
                    continue;
                }

                /*  shift down another one. */
                rowPtIdx--;
            }

            /* Moving across the row [rowPtIdx*dimension + rowCoord],
             * which is three rows, x, y and z:
             */

            for ( rowCoord = 0; rowCoord < dim; rowCoord++ )
            {
                /* B-vector term  ( RHS ) */

                bMatTerm = basisFcns[basisIdx] * (sdTerm * dotTan * uTan[rowCoord] + dotOther * uOther[rowCoord] + dotNorm * uNorm[rowCoord]);

                /* Note: Use -= for b vec, because term is negative. */
                bMat[rowPtIdx * dim + rowCoord] -= bMatTerm;

                /* If the end points are fixed, then those point moves are
                 * no longer variables, so their terms end up on the RHS
                 * instead of in AMat.  But, the term to be added here
                 * would be Bi*Bj * N.x * ( D0 dot N ) etc.
                 * But D0 is zero, so just leaving it out of AMat is all we do.
                 *  if ( fixEndPts )
                 *  {
                 *      endPointTerm = ....
                 *      bMat[rowPtIdx].x -= endPointTerm;
                 *  }
                 */

                /* Row of A-matrix terms */

                for ( j = span - deg; j <= span; j++ )
                {
                    colPtJdx = j;

                    basisJdx = j - span + deg;

                    if( fixEndPts >= 1 )
                    {
                        if( span == deg && basisJdx == 0 )
                            continue;

                        if( span == nKnots - deg - 2 && basisJdx == deg )
                            continue;

                        colPtJdx--; /* shift down one */
                    }

                    if( fixEndPts >= 3 )
                    {
                        if( (span == deg && basisJdx == 1) || (span == deg + 1 && basisJdx == 0) )
                        {
                            /* means second ctrl point. */
                            continue;
                        }

                        if( (span == nKnots - deg - 2 && basisJdx == deg - 1) || (span == nKnots - deg - 3 && basisJdx == deg) )
                        {
                            /* next to last ctrl point. */
                            continue;
                        }

                        /*  shift down another one. */
                        colPtJdx--;
                    }

                    /* Index through the dimensions, for the three columns. */
                    for ( colCoord = 0; colCoord < dim; colCoord++ )
                    {
                        thisTerm = basisFcns[basisJdx] * basisFcns[basisIdx] * CMat[rowCoord][colCoord];

                        AMat[rowPtIdx * dim + rowCoord][colPtJdx * dim + colCoord] += thisTerm;
                    }
                }
            }
        }
    } /* End of loop over all data points, to set up matrices. */

    /*  Now add in the regularization terms. */  
    /* NOTE: see comment "Regularization Terms" at end of file. */
    if( alpha > 0 && deg == 3 ) /* This is specific to cubics. */
    {
        /* Minimize the (square of the) first derivative.
         * Integrate over each span.
         * NOTE: see comment "Integrating Arc Length" at end of file.
        */
        numSpans = numCtrlPts - deg;

        for ( spn = 0; spn < numSpans; spn++ )
        {
            ctrlPtIdx = spn;

            /* For the first deriv, the 4x4 is different for each span. */
            ktIdx = spn + deg - 1;

            /*  Calculate the integrals of the products of the 2nd degree
             *  basis functions over [knots[i+1], knots[i+2]]
             *  i(ij) is the integral of N2_i*N2_j.
             *  NOTE: see comment "Integrating Arc Length" at end of file.
             */

            del = knots[ktIdx + 2] - knots[ktIdx + 1];
            del1 = del / (knots[ktIdx + 2] - knots[ktIdx + 0]);
            del2 = del / (knots[ktIdx + 3] - knots[ktIdx + 1]);
            i00 = (del * del1 * del1) / 5;
            i01 = del * del1 * (10 - 6 * del1 - del2) / 30;
            i02 = (del * del1 * del2) / 30;
            i11 = del * (15 - 10 * del1 - 10 * del2 + 3 * del1 * del1 + del1 * del2 + 3 * del2 * del2) / 15;
            i12 = del * del2 * (10 - del1 - 6 * del2) / 30;
            i22 = (del * del2 * del2) / 5;

            alphaMat[0][0] = i00;
            alphaMat[0][1] = alphaMat[1][0] = i01 - i00;
            alphaMat[0][2] = alphaMat[2][0] = i02 - i01;
            alphaMat[0][3] = alphaMat[3][0] = -i02;
            alphaMat[1][1] = i00 - 2 * i01 + i11;
            alphaMat[1][2] = alphaMat[2][1] = i01 - i02 - i11 + i12;
            alphaMat[1][3] = alphaMat[3][1] = i02 - i12;
            alphaMat[2][2] = i11 - 2 * i12 + i22;
            alphaMat[2][3] = alphaMat[3][2] = i12 - i22;
            alphaMat[3][3] = i22;

            rowPtIdx = ctrlPtIdx;

            if( fixEndPts >= 1 )
                rowPtIdx--;

            if( fixEndPts >= 3 )
                rowPtIdx--;

            /* Add this matrix into the A matrix.  For the B vector, subtract
               this matrix times the column vector of control points.
             */

            for ( i = 0; i <= deg; i++ )
            {
                BVecPt.x = BVecPt.y = BVecPt.z = 0; /* For the B-vector */

                for ( j = 0; j <= deg; j++ )
                {
                    if( (rowPtIdx + i) >= 0 && (rowPtIdx + i) < numVarPts && (rowPtIdx + j) >= 0 && (rowPtIdx + j) < numVarPts )
                    {
                        for ( rowCoord = 0; rowCoord < dim; rowCoord++ )
                        {
                            AMat[(rowPtIdx + i) * dim + rowCoord][(rowPtIdx + j) * dim + rowCoord] += alpha * alphaMat[i][j];
                        }
                    }

                    BVecPt.x += alphaMat[i][j] * ctrlPts[ctrlPtIdx + j].x;
                    BVecPt.y += alphaMat[i][j] * ctrlPts[ctrlPtIdx + j].y;
                    BVecPt.z += alphaMat[i][j] * ctrlPts[ctrlPtIdx + j].z;
                }

                if( (rowPtIdx + i) >= 0 && (rowPtIdx + i) < numVarPts )
                {
                    bMat[(rowPtIdx + i) * dim + 0] -= alpha * BVecPt.x;
                    bMat[(rowPtIdx + i) * dim + 1] -= alpha * BVecPt.y;
                    bMat[(rowPtIdx + i) * dim + 2] -= alpha * BVecPt.z;
                }
            }
        }
    } /* end of alpha correction */

    /* beta correction: */

    if( beta > 0 && deg == 3 ) /* This is specific to cubics. */
    {
        /* Minimize the (square of the) second derivative.
         * Integrate over each span.
         * NOTE: see comment "Integrating Arc Length" at end of file.
        */
        numSpans = numCtrlPts - deg;

        for ( spn = 0; spn < numSpans; spn++ )
        {
            ctrlPtIdx = spn;

            rowPtIdx = ctrlPtIdx;

            if( fixEndPts >= 1 )
                rowPtIdx--;

            if( fixEndPts >= 3 )
                rowPtIdx--;

            /* For second order, the matrix is independent of the knot vector:
             *     2  -3   0   1
             *    -3   6  -3   0
             *     0  -3   6  -3
             *     1   0  -3   2
             * Add this into the A matrix.  For the B vector, subtract
             * this matrix times the column vector of control points.
             */

            for ( i = 0; i <= deg; i++ )
            {
                BVecPt.x = BVecPt.y = BVecPt.z = 0; /* For the B-vector */

                for ( j = 0; j <= deg; j++ )
                {
                    if( (rowPtIdx + i) >= 0 && (rowPtIdx + i) < numVarPts && (rowPtIdx + j) >= 0 && (rowPtIdx + j) < numVarPts )
                    {
                        for ( rowCoord = 0; rowCoord < dim; rowCoord++ )
                        {
                            AMat[(rowPtIdx + i) * dim + rowCoord][(rowPtIdx + j) * dim + rowCoord] += beta * BetaMat[i][j];
                        }
                    }

                    BVecPt.x += BetaMat[i][j] * ctrlPts[ctrlPtIdx + j].x;
                    BVecPt.y += BetaMat[i][j] * ctrlPts[ctrlPtIdx + j].y;
                    BVecPt.z += BetaMat[i][j] * ctrlPts[ctrlPtIdx + j].z;
                }

                if( (rowPtIdx + i) >= 0 && (rowPtIdx + i) < numVarPts )
                {
                    bMat[(rowPtIdx + i) * dim + 0] -= beta * BVecPt.x;
                    bMat[(rowPtIdx + i) * dim + 1] -= beta * BVecPt.y;
                    bMat[(rowPtIdx + i) * dim + 2] -= beta * BVecPt.z;
                }
            }
        }
    } /* end of beta correction */

    /*  Ok, now we've got A and b, solve Ax = b for x,
     *  which is the displacements.
     *
     *  NLib doesn't appear to have a direct matrix solver;
     *  use N_RealMatrixLuDecomposePivot and N_RealMatrixForBackPivot, as in N_RealMatrixLstSqSolve:
     */

    idxArray = N_AllocInt1dArray( numVars, &localStacks );

    if( idxArray == NULL )
        NL_ERROR( NL_MEM_ERR );

    error = N_RealMatrixLuDecomposePivot( &rMat, idxArray ); /* do L/U decomp on AMat */

    if( error != NL_NO )
        NL_ERROR( NL_SEQ_ERR );

    error = N_RealMatrixForBackPivot( &rMat, idxArray, bMat ); /* Solve L/U decomposed mat */

    if( error != NL_NO )
        NL_ERROR( NL_SEQ_ERR );

    /* Load up the results */
    j = 0;

    if( fixEndPts >= 1 )
    {
        j = 1;
        ctrlPtMoves[0] = NL_ZERO;
        ctrlPtMoves[numCtrlPts - 1] = NL_ZERO;
    }

    if( fixEndPts >= 3 )
    {
        j = 2;
        ctrlPtMoves[1] = NL_ZERO;
        ctrlPtMoves[numCtrlPts - 2] = NL_ZERO;
    }

    for ( i = 0; i < numVarPts; i++ )
    {
        ctrlPtMoves[j].x = bMat[i * dim + 0];
        ctrlPtMoves[j].y = bMat[i * dim + 1];

        if( dim > 2 )
            ctrlPtMoves[j].z = bMat[i * dim + 2];
        else
            ctrlPtMoves[j].z = 0;

        j++;
    }

    if( fixEndPts == 2 )
    {
        /* Line up the penultimate point moves with the initial vectors */
        /* (Note, if we're here, the end points are fixed.) */

        N_VectorDotRef( &( ctrlPtMoves[1] ), &startTan, &dotTan ); /* dot product */
        N_VectorDotRef( &startTan, &startTan, &dotNorm );

        if( dotNorm > fabs( dotTan *NL_ZCTL ) )
        {
            thisTerm = dotTan / dotNorm;
            N_ScalePt( thisTerm, startTan, &( ctrlPtMoves[1] ) );
        }

        N_VectorDotRef( &( ctrlPtMoves[numCtrlPts - 2] ), &endTan, &dotTan );
        N_VectorDotRef( &endTan, &endTan, &dotNorm );

        if( dotNorm > fabs( dotTan *NL_ZCTL ) )
        {
            thisTerm = dotTan / dotNorm;
            N_ScalePt( thisTerm, endTan, &( ctrlPtMoves[numCtrlPts - 2] ) );
        }
    }

    EXIT:

    N_EndNurbs( &localStacks );

    return error;

} // end ST_findStep()


static NL_BOOLEAN ST_doneIterating( NL_INDEX iter, NL_INDEX maxIter, NL_REAL errorSD, NL_REAL errorDist, NL_REAL errorStop )
{
    static NL_REAL errors[100]; /* Can't just say 'MAXITER': compiler. */

    /* whether to check converging, or just continue to MAXITER: */
    static NL_FLAG checkIter = NL_NO;

    NL_BOOLEAN prevConverging, thisConverging;

    if( iter >= maxIter )
        return NL_TRUE;

    if( errorDist < errorStop )
        return NL_TRUE;

    errors[iter] = errorSD;

    /* Check converging (diverging).  */
    /* Check two consecutive steps, because sometimes it will bump up
       just a bit, and then continue downward. */

    if( checkIter == NL_YES && iter >= 3 )
    {
        prevConverging = (errors[iter - 1] >= errors[iter - 2]) || (errors[iter - 1] / errors[iter - 2] > 0.998);

        if( prevConverging )
        {
            thisConverging = (errors[iter] >= errors[iter - 1]) || (errors[iter] / errors[iter - 1] > 0.999);

            if( thisConverging )
                return NL_TRUE;
        }
    }

    return NL_FALSE;
}

#if NLIB_UNUSED

/**********************************************************************/
/* Comments on the algorithms.  Moved here to keep the code cleaner.  */
/**********************************************************************/

/*
        Setting up the A-matrix
        -----------------------

   First, for readability in this discussion, we'll assume it's
   three-dimensional cubic.  If not, then change each "four" to "degree+1"
   and each "12" to "(degree+1)*dimension".

   Each entry of the A matrix is basis functions times dot-product-like
   products of components of the tangent and normal vectors.  Since each
   coordinate of the point-shift vectors is an independent unknown, we do
   it all coordinate by coordinate.  And since it's cubic, at any point,
   only four control point (shifts) are affected.  (Indicated by the
   'span' argument from N_BasisEval.)  So for a 3-dimensional cubic, this
   loop will increment 12 consecutive entries in the b-vector (four
   points, three coords), and a 12 x 12 square in the A matrix.

   There are only nine combinations of the dot-product-like products
   for the 144 entries of A.  If s == sdTerm, N is unitNorm, etc:
  s*Tx*Tx + Ox*Ox + Nx*Nx;
  s*Tx*Ty + Ox*Oy + Nx*Ny;  s*Tx*Tz + Ox*Oz + Nx*Nz
  s*Ty*Tx + Oy*Ox + Ny*Nx;
  s*Ty*Ty + Oy*Oy + Ny*Ny;  s*Ty*Tz + Oy*Oz + Ny*Nz
  s*Tz*Tx + Oz*Ox + Nz*Nx;
  s*Tz*Ty + Oz*Oy + Nz*Ny;  s*Tz*Tz + Oz*Oz + Nz*Nz
   Since it's symmetrical, there are actually only six different ones.
   We'll calculate those before the loop.  If we call them CMat[i][j]
   (since they come from the Curve evaluation), the entries of A will be:

  B0*B0*C00 B0*B0*C01 B0*B0*C02  B0*B1*C00 B0*B1*C01 B0*B1*C02  B0*B2*C00  ...
  B0*B0*C10 B0*B0*C11 B0*B0*C12  B0*B1*C10 B0*B1*C11 B0*B1*C12  B0*B2*C10  ...
  B0*B0*C20 B0*B0*C21 B0*B0*C22  B0*B1*C20 B0*B1*C21 B0*B1*C22  B0*B2*C20  ...

  B1*B0*C00 B1*B0*C01 B1*B0*C02  B1*B1*C00 B1*B1*C01 B1*B1*C02  B1*B2*C00  ...
  B1*B0*C10 B1*B0*C11 B1*B0*C12  B1*B1*C10 B1*B1*C11 B1*B1*C12  B1*B2*C10  ...
  B1*B0*C20 B1*B0*C21 B1*B0*C22  B1*B1*C20 B1*B1*C21 B1*B1*C22  B1*B2*C20  ...

  B2*B0*C00 B2*B0*C01 B2*B0*C02  B2*B1*C00 B2*B1*C01 B2*B1*C02  B2*B2*C00  ...
  ...

   where B0, B1, B2 and B3 are the cubic basis functions.

   Note that we have to do each coordinate separately, because the
   coordinates mix together (e.g., s*Ty*Tx + Oy*Ox + Ny*Nx).  This means
   that we can't do it as in N_FitCrvApproxLstSq, where the size of the matrix is
   related to the number of variable control points, and the matrix
   solvers (N_RealMatrixLuDecompose and N_RealMatrixForBack) work with CPOINTs in the x- and
   B-vectors.  We must use a [3*numPpoints] square matrix, and
   use N_RealMatrixLuDecomposePivot and N_RealMatrixForBackPivot, as in N_RealMatrixLstSqSolve.


        Regularization Terms
        --------------------

  We're minimizing the distances from the data points to the curve.
  This just means that we're making a curve that goes close to each point;
  it doesn't say anything about what else the curve does.  The curve may,
  and often does, develop kinks or loops, or fly far away and come back.
  The purpose of the regularization terms is to prevent that, and keep
  the curve close to the data points.

  Two regularization terms are used.  One minimizes the arc length of
  the curve, and the other minimizes curvature.  They are weighted by
  coefficients alpha and beta, respectively.  In practice, it works best
  if alpha and beta decrease as the iteration proceeds.  Therefore,
  starting and ending values of both may be passed into the routine.
  If the passed-in values are negative, default values will be used.

  Writing out the expression for the arc length squared of a cubic
  B-Spline from t_i to t_i+1, and differentiating it w.r.t. the
  four involved control points gives a system of equations whose
  coefficients are the integrals (from t_i to t_i+1) of products
  of quadratic basis functions.  If "ij" means the integral from
  t_i to t_i+1 of N^3_i * N^3_j, then the system is:

   (   00      01-00        02-01       -02  )   ( d_0 )   ( b_0 )
   ( 01-00   00-2*01+11   01-02-11+12  02-12 ) * ( d_1 ) = ( b_1 )
   ( 02-01   01-02-11+12  00-2*01+11   12-22 )   ( d_2 )   ( b_2 )
   (  -02      02-12        12-22        22  )   ( d_3 )   ( b_3 )

  The modified control points are the sum of the current control points
  plus the dispalcements: p_i = c_i + d_i.  The d_i are the unknowns,
  and the c_i go into the right-hand side, negated.  Therefore the b_i
  are the product of the i-th row of the A-matrix times the existing
  control points, negated.


        Integrating Arc Length
        ----------------------

  We had to come up with an explicit expression for the arc length
  (squared) of the curve, in terms of the control points, so that we
  could differentiate the expressions with respect to the control
  points, set those derivatives to zero, and incorporate them into
  the least-squares matrices.  The derivative of a cubic curve is
  a constant times the quadratic basis functions times the first-order
  control-point differences.  Writing out the integral of the square
  of the first derivative over a single span gives a sum of products
  of two control points times the integrals of products of two of the
  quadratic B-Spline basis functions.  Four control points and four
  knots values are involved on each span.  Differentiating this expression
  with respect to the control points gives a linear system in the
  control points, as desired, whose coefficients are the integrals
  of products of the basis functions.  

  Integrating products of quadratic basis functions resulted in
  fifth-degree expressions in the knot values.  Some came out nice,
  for example, the integral from t_i to t_i+1 of N^2_0 times itself
  is   ( t_i+1 - t_i )^3  /  5*( t_i+1 - t_i-1 )^2.  The mixed terms
  got very complex, but ultimately simplified down to the following.

  Say D = ( t_i+1 - t_i ),
     D1 = D / ( t_i+1 - T_i-1 ), and
     D2 = D / ( t_i+2 - T_i ), then:

   int(0*0) = 1/5  D * D1 * D1
   int(0*1) = 1/30 D * D1 * ( 10 - 6*D1 - D2 )
   int(0*2) = 1/30 D * D1 * D2
   int(1*1) = 1/15 D * ( 15 - 10*D1 - 10*D2 + 3*D1*d1 + D1*D2 + 3*D2*D2 )
   int(1*2) = 1/30 D * D2 * ( 10 - D1 - 6*D2 )
   int(2*2) = 1/5  D * D2 * D2

  The integrals for the second derivative squared were much easier,
  because the second derivative of the cubic curve is linear, so the
  integrals are cubic.  In particular, only t_i+1 and t_i are involved,
  and int(0*0) == int(1*1) = D/3, and the mixed int(0*1) = D/6. 
  Combining these with the control points was more involved, because
  the coefficients are second-order point differences (p2 - 2*p1 + p0),
  but all in all the matrix came out to be independent of the knot vector:
    2  -3   0   1
   -3   6  -3   0
    0  -3   6  -3
    1   0  -3   2
  This matrix is kept as BetaMat.

  These corrections are added in span by span, with each span covering
  four control-points rows and columns.

  The only downside to all this is that the calculations are specific
  to cubics.  Other degrees would have to be done separately.  Currently,
  if the curve is not cubic, these corrections cannot be applied.

*/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a common knot vector for global
     approximation of several curves, with optional parameter fixes.
     A typical calling example is:

       NL_EPOLYGON    **pointLists;
       NL_INDEX       listCount, cPtCount;
       NL_FLAG        knotType;
       NL_INDEX       fixCount;
       NL_INDEX       *fixIndices;
       NL_REAL        *fixParams;
       NL_DEGREE      deg;
       NL_KNOTVECTOR  knotVec;
       NL_STACKS      stks;
       ...
       (set up pointLists, fixIndices and fixParams, choose cPtCount
       and deg, and define knotVec);
       ...
       N_CalcKnotVectorFromPtArrays(pointLists, listCount, cPtCount, deg, knotType,
               fixCount, fixIndices, fixParams, &knotVec, &stks);

     MEMORY FOR knotVec IS ALLOCATED INSIDE THIS ROUTINE, in stks.

     Note: if cPtCount is passed in, the resulting knot vector is not
     guaranteed to result in a curve with exactly cPtCount+1 control
     points, due to prorating the control point counts among segments
     between parameter fixes.  It will be within one or two however.


   ACCESS:
   
     pointLists   , input  ,  ptr to lists of points to be approximated
     listCount    , input  ,  Highest index in pointLists
     cPtCount     , input  ,  Highest index of control point array of 
                              approximating curve, if > deg (or if > 4,
                              if deg < 1); otherwise chosen by this routine
     deg          , input  ,  Degree of approximating curve if > 0,
                              else chosen by this routine (as 3).
     knotType     , input  ,  Parameterization type:
                                NL_UNIFORM     : Uniform parametrization
                                NL_CHORDLENGTH : Chord length parametrization
                                NL_CENTRIPETAL : Centripetal parametrization
     fixCount     , input  ,  Highest index in fixIndices and fixParams
     fixIndices   , input  ,  Index into each member of pointLists of the
                              data point to be interpolated
     fixParams    , input  ,  Parameter values for fixes
     knotVec      , output ,  Knot vector
     stks         , input  ,  Stacks for knotVec


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* Forward declaration of helper routine */
static NL_FLAG N_CalcKnotVectorOneSegment( NL_EPOLYGON * pointLists [], NL_INDEX listCount, NL_INDEX cPtCount, NL_DEGREE deg, NL_FLAG knotType, NL_KNOTVECTOR * knotVec, NL_STACKS * stks );

/* The main routine */

NL_FLAG N_CalcKnotVectorFromPtArrays( NL_EPOLYGON *pointLists [], /* array of pointers */
                                      NL_INDEX listCount, NL_INDEX cPtCount, NL_DEGREE deg, NL_FLAG knotType, NL_INDEX fixCount, NL_INDEX fixIndices [], 
                                      NL_REAL fixParams [], NL_KNOTVECTOR *knotVec, NL_STACKS *stks )
{

    NL_FLAG error = NL_NO;

    NL_INDEX numSegs, segNum;

    NL_INDEX i, k, k0, k1;
    NL_INDEX ktIdx;

    NL_REAL t0, t1;

    NL_INDEX knotCount; /* For the final knotVec */
    NL_REAL *knots;

    NL_INDEX dataCount;

    NL_INDEX dataPtIdx0, dataPtIdx1;

    NL_INDEX tmpIdx;
    NL_POINT *tmpPtPtr;

    NL_INDEX curveNum;
    NL_INDEX segDataCount;
    NL_POINT *segDataPts;
    NL_INTERVAL segDomain;

    NL_INDEX segCPtCount;
    NL_INDEX segKnotCount;
    NL_REAL *segKnots;
    NL_REAL xPtRatio;

    NL_EPOLYGON ** segPolys;
    NL_KNOTVECTOR ** knotVecArray;

    NL_STACKS localStacks;

    /* Start NURBS */
    N_InitNurbs( &localStacks );

    /* Initialize */

    if( deg < 1 )
        deg = 3;

    numSegs = fixCount + 2; /* actual count, not highest index */

    if( numSegs < 2 )
    {
        return N_CalcKnotVectorOneSegment( pointLists, listCount, cPtCount, deg, knotType, knotVec, stks );
    }

    /* Generate a common knot vector for each segment. */

    knotVecArray = N_Alloc1dArrayKnotVectPtrs( numSegs - 1, &localStacks );

    dataCount = pointLists[0]->n; /* Highest index in each point list */

    /* Create all the EPOLYGONs ahead of time. */
    /* For their point arrays, we'll just point right into pointLists. */
    segPolys = N_Alloc1dPtPolygonPtrs( listCount, &localStacks );

    for ( i = 0; i <= listCount; i++ )
    {
        segPolys[i] = N_AllocPtPolygonStruct( &localStacks );

        if( segPolys[i] == NULL )
            NL_QUIT;
    }

    dataPtIdx1 = 0;
    t1 = 0;
    knotCount = 0;

    for ( segNum = 0; segNum < numSegs; segNum++ )
    {
        dataPtIdx0 = dataPtIdx1;
        dataPtIdx1 = (segNum < numSegs - 1) ? fixIndices[segNum] : dataCount;

        segDataCount = dataPtIdx1 - dataPtIdx0;

        /* Set up point list for this segment, for each curve. */
        for ( curveNum = 0; curveNum <= listCount; curveNum++ )
        {
            N_EPolygonGetPts( pointLists[curveNum], &tmpIdx, &tmpPtPtr );

            /* while we're here, check valid input */
            if( tmpIdx != dataCount )
                NL_QUIT;

            /* Point right into the main lists:  */
            segDataPts = tmpPtPtr + dataPtIdx0;
            N_EPolygonFromPts( segPolys[curveNum], segDataCount, segDataPts );
        }

        /* set ctrl pt count */
        segCPtCount = cPtCount;

        if( cPtCount > 0 )
        {
            /* Prorate the number of desired control points by the number of */
            /* data points in this segment */
            xPtRatio = (NL_REAL)segDataCount / (NL_REAL)dataCount;
            segCPtCount = (int)( xPtRatio * cPtCount + 2 );

            if( segNum > 0 && segNum < numSegs - 1 )
                segCPtCount++; /* an extra one for each interior breakpoint */
        }

        /* These haven't been initialized yet */
        knotVecArray[segNum] = N_AllocKnotVector( &localStacks );

        error = N_CalcKnotVectorOneSegment( segPolys, listCount, segCPtCount, deg, knotType, knotVecArray[segNum], &localStacks );

        if( error == NL_YES )
            NL_QUIT;

        /* scale knot vector to this interval */

        t0 = t1;
        t1 = (segNum >= numSegs - 1) ? 1 : fixParams[segNum];

        N_CreateInterval( &segDomain, t0, t1 );
        N_BasisReparam( knotVecArray[segNum], deg, segDomain );

        knotCount += knotVecArray[segNum]->m + 1; /* actual count, not index */
    }

    /* Now we've got a common knot vector for each segment. */
    /* Combine the segments into one.     */
    /* They are parameterized end to end. */

    /* Each internal fix has 2(deg+1) knots, we want 1 knot */
    /* remove (2*deg+1) for each internal fix */
    knotCount -= (2 * deg + 1) * (fixCount + 1);
    /* and turn it back into an index, not a count */
    knotCount--;

    knots = N_AllocReal1dArray( knotCount, stks );

    if( knots == NULL )
        NL_QUIT;

    ktIdx = 0; /* index into combined knot vector */

    for ( segNum = 0; segNum < numSegs; segNum++ )
    {
        N_KnotVectorGetKnots( knotVecArray[segNum], &segKnotCount, &segKnots );

        k0 = (segNum == 0) ? 0 : deg + 1;
        k1 = (segNum < numSegs - 1) ? segKnotCount - deg : segKnotCount;

        for ( k = k0; k <= k1; k++ )
            knots[ktIdx++] = segKnots[k];
    }

    /* Put it all together */
    N_KnotVectorFromRealArray( knotVec, knots, knotCount );

    /* End NURBS */

    EXIT:

    N_EndNurbs( &localStacks );

    return (error);
} /* end N_CalcKnotVectorFromPtArrays */

/*******************************************************************//**

   DESCRIPTION:

     Make a surface by sweeping a g1 curve built in the 
     xy-plane along a spiral around the positive Z axis starting
     at the origin (0,0,0).  

     Height is the length of the spiral.

     NumTurns is the number of revolutions, with each turn = 360 degrees.

     RorL specifies a right or left handed spiral.
        The spiral is a right hand spiral if RorL = 1 and 
        a left hand spiral if RorL = 0.

     RadiusTaper = amount the helix radius increases (or decreases)
        from the helix base to the helix top.  All points
        on a curve being swept along a decreasing helix must start
        out far enough from the origin so that their radius along the
        length of the helix remains positive.
      
     NumPoints when greater than 0 specifies the number of points 
        used to approximate the helix running through
        every point on the input curve. When NumPoints is 0 
        then NumPoints is calculated for the first point on
        the input curve to generate an approximation to a helix
        for the given tol and the calculated NumPoints is then
        used for the remainder of the points on the curve.  
        The max number of interpolation points allowed in any one 
        helix curve is limited by NL_CCPLIM which is currently set to 1000.

       NL_CURVE       *Crv
       NL_REAL        height;
       NL_REAL        RadiusTaper;
       NL_REAL        NumTurns;
       NL_INDEX       RorL;
       NL_INDEX       *NumPoints;
       NL_REAL        *tol
       NL_SURFACE     *Srf;
       NL_STACKS&	   S;
       ....
       ....
       get height, radius, taper, NumTurns, RorL, deg
       set NumPoints and tol
       ...
       N_CrvInitArrays(&Crv);
       ....get curve to sweep in X,Z plane
       N_SrfInitArrays(&Srf);
       N_CrvApproxSpiral(&Crv. height, radius, taper, NumTurns, 
            RorL, *NumPoints, *tol, &Srf, &S);

   ACCESS:
   
     *Crv        input     NL_G1 Curve built in the xy-plane to sweep along helix
     height      input     The height (>0) of the spiral along the z axis 
     RadiusTaper input     amount helix increases (decreases) from bot to top.
     NumTurns    input     Number of turns (1 = 360 degrees) in the spiral
     RorL        input     Right or left spiral direction
     *NumPoints  input     if zero, the routine will compute and 
                            Crv will be a fit to the tolerance tol
                 output    number of points required for this fit
                            max allowed is determined by NL_CCPLIM
     *tol        input     Cubic Crv is an approximation to within tol
                            tol > 10*NL_MTOL
                
     Srf         output    Output Surface
     S           input     Crv's memory stack

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSpiralSrfTaper( NL_CURVE *Crv,       /* in : curve in x-y plane to be swept (not modified)                   */
                                NL_REAL height,      /* in : height of helix sweep                                           */
                                NL_REAL RadiusTaper, /* in : amount helix increases (decreases) from bot to top.             */
                                NL_REAL NumTurns,    /* in : 1 turn = 360 degrees                                            */
                                NL_INDEX RorL,       /* in : 1 = right hand spiral                                           */
                                                     /*      0 = left hand spiral                                            */
                                NL_INDEX *numPoints, /* i/o: 0 = fit helix sweep to tolerance                                */
                                                     /*      else number of control points in sweep direction.               */
                                                     /*      Set to number of sweep direction control points on exit.        */
                                NL_REAL *tol,        /* i/o: tol of swept helix curves from true helix, range: tol >= 1.0E-6 */
                                NL_SURFACE *Srf,     /* out: Output surface                                                  */
                                NL_STACKS *S )       /* in : Crv's memory stack                                              */
{
    NL_STACKS SL;

    NL_CURVE CrvH;
    NL_FLAG error = NL_NO;

    NL_REAL w;
    NL_REAL rad0, rad1;
    NL_KNOTVECTOR *hKnt;
    NL_BOOLEAN rat = 0;

    /* get knot data off section curve */
    NL_CPOINT *cPw, *hPw, ** Qw, *Cp;
    NL_REAL *cU, *hU, *UQ, *VQ, RotAng;
    NL_DEGREE p, q;
    NL_INDEX i, ii, mU, n, m;
    NL_VECTOR ZAxis, Origin;
    NL_KNOTVECTOR *knu, *knv;

    N_InitNurbs( &SL );
    N_CrvInitArrays( &CrvH );
    N_CrvGetCPtsDegreeAndKnots( Crv, &n, &cPw, &p, &mU, &cU );

    if( cPw[0].w != NL_NOW )
        rat = 1;

    /* for each control point in section curve */
    for ( ii = 0; ii <= n; ii++ )
    {
        /* the helix */
        w = (rat) ? cPw[ii].w : 1.0; /* weight of Pw from section curve  */
        /* rad0 = radius0 + cPw.x/w ;  */
        rad0 = sqrt( cPw[ii].x / w * cPw[ii].x / w + cPw[ii].y / w * cPw[ii].y / w );
        rad1 = rad0 + RadiusTaper;

        if( rad0 < 0.0 )
        {
            error = 1;
            NL_OUT;
        }

        if( rad1 < 0.0 )
        {
            error = 1;
            NL_OUT;
        }

        /* make the spiral with start on x axis. */
        error = N_CrvApproxSpiral( height, rad0, rad1, NumTurns, RorL, numPoints, tol, &CrvH, &SL );

        if( error )
            NL_OUT;

        /* get m value to size surface arrays */
        m = *numPoints + 1; /* must now be constant for all helixes */

        /* Rotate CrvH to coeff point                                */
        /* gwc note: CurvePoint z values are ignored here assuming   */
        /*           they are zero.  We could make up a rotate and   */
        /*           translate a helix rule (many are possible) to   */
        /*           handle non planar curves.                       */
        RotAng = atan2( cPw[ii].y / w, cPw[ii].x / w );
        Origin.x = 0.0;
        ZAxis.x = 0.0;
        Origin.y = 0.0;
        ZAxis.y = 0.0;
        Origin.z = 0.0;
        ZAxis.z = 1.0;
        N_CrvRotateAboutAxis( &CrvH, Origin, ZAxis, RotAng );

        if( ii == 0 )
        {
            /* Build surface */

            q = 3; /* degree from helix = cubic */
            N_SrfSizeArrays( Srf, n, m, p, q, n + p + 1, m + q + 1, _T("N_CreateSpiralSrfTaper"), S );
            N_SrfGetCPtsKnotVectorAndKnots( Srf, &Qw, &knu, &knv, &UQ, &VQ );

            /* get knot data off section curve */
            for ( i = 0; i <= n + p + 1; i++ )
                UQ[i] = cU[i];

            /* get knot data off helix */
            N_CrvGetCPtsKnotVectorAndKnots( &CrvH, &hPw, &hKnt, &hU );

            for ( i = 0; i <= m + q + 1; i++ )
                VQ[i] = hU[i];
        }
        else
        { /* get control points from helix  */
            N_CrvGetCPtsKnotVectorAndKnots( &CrvH, &hPw, &hKnt, &hU );
        }

        /* load  coeffs from helix */
        for ( i = 0; i <= m; i++ )
        {
            Cp = &( Srf->net->Pw[ii][i] );
            Cp->x = hPw[i].x * w;
            Cp->y = hPw[i].y * w;
            Cp->z = hPw[i].z * w;
            Cp->w = (rat) ? w : NL_NOW;
        }
    }

    /* End NURBS and Exit */
    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSpiralSrfTaper */

/**********************************************************************/
/*      Local (private) routine                                       */
/**********************************************************************/

static NL_FLAG N_CalcKnotVectorOneSegment( NL_EPOLYGON *pointLists [], /* array of pointers */
                                           NL_INDEX listCount, NL_INDEX cPtCount, NL_DEGREE deg, 
                                           NL_FLAG knotType, NL_KNOTVECTOR *knotVec, NL_STACKS *stks )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, k;
    NL_INDEX minPtCount, maxPtCount;
    NL_INDEX knotCount;

    NL_INDEX dataCount;
    NL_POINT *dataPtr;
    NL_REAL *dataParams;

    NL_REAL fact, sum;
    NL_REAL *knots;

    NL_KNOTVECTOR ** ktVecArray;

    NL_STACKS localStacks;

    /* Start NURBS */
    N_InitNurbs( &localStacks );

    /* Initialize */

    if( deg < 1 )
        deg = 3;

    /* Find smallest and largest numbers of data points in the lists */
    minPtCount = maxPtCount = pointLists[0]->n;

    for ( i = 1; i <= listCount; i++ )
    {
        if( pointLists[i]->n < minPtCount )
            minPtCount = pointLists[i]->n;

        if( pointLists[i]->n > maxPtCount )
            maxPtCount = pointLists[i]->n;
    }

    if( cPtCount < 0 )
    {
        /* Calculate a control point count. */
        /* This works well in practice, for approximating points: */
        /* From Maron, "Numerical Analysis:  A Practical Approach", */
        /* McMillan, 1982, Sec. 5.3D.  */
        /* (Explicit casts to silence warning messages.) */

        cPtCount = (int)( 2.0 *sqrt( (NL_REAL)minPtCount ) - 2.0 );
        cPtCount *= 2;

        if( cPtCount > minPtCount - deg - 1 )
            cPtCount = minPtCount - deg - 1;

        if( cPtCount < deg )
            cPtCount = deg;
    }

    if( cPtCount < deg )
        NL_QUIT; /* input error */

    knotCount = cPtCount + deg + 1;

    knots = N_AllocReal1dArray( knotCount, stks );

    if( knots == NULL )
        NL_QUIT;

    /* Easy one first */
    if( knotType EQ NL_UNIFORM )
    {
        /* Clamped ends */
        for ( i = 0; i <= deg; i++ )
        {
            knots[i] = 0.0;
            knots[knotCount - i] = 1.0;
        }

        sum = cPtCount + (NL_REAL)1 - (NL_REAL)deg;
        fact = 1.0 / sum;

        for ( i = deg + 1; i < knotCount - deg; i++ )
            knots[i] = i * fact;

        NL_OUT;
    }

    /* Generate a knot vector for each point set, all with the
     * same number of knots
     */
    ktVecArray = N_Alloc1dArrayKnotVectors( knotCount, listCount, &localStacks );

    dataParams = N_AllocReal1dArray( maxPtCount, &localStacks );

    if( dataParams == NULL )
        NL_QUIT;

    for ( i = 0; i <= listCount; i++ )
    {
        N_EPolygonGetPts( pointLists[i], &dataCount, &dataPtr );
        N_FitCalcCrvParamValues( (NL_VOID *)dataPtr, dataCount, NL_EPOINT, knotType, dataParams );
        N_FitCalcKnotVectorCrvApprox( dataParams, dataCount, cPtCount, deg, ktVecArray[i] );
    }

    /* The knot vectors are all the same size, and presumably pretty similar. */
    /* Use the average value of each knot.  */

    for ( k = 0; k <= knotCount; k++ )
    {
        sum = 0;

        for ( i = 0; i <= listCount; i++ )
        {
            sum += ktVecArray[i]->U[k];
        }
        knots[k] = sum / ((NL_REAL)listCount + (NL_REAL)1);
    }

    /* Assemble the NL_KNOTVECTOR and return. */
    N_KnotVectorFromRealArray( knotVec, knots, knotCount );

    /* End NURBS */

    EXIT:

    N_EndNurbs( &localStacks );

    return (error);
} /* end N_CalcKnotVectorOneSegment */
#endif // NLIB_UNUSED
