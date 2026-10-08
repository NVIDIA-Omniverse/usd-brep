// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* CrvPoly.c : Function Definitions that act on NL_CURVE objects as polynomials */
/*****************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>


#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point on a polynomial curve, given in power 
     basis form, using Horner's method. A typical calling example is:

       NL_CURVE      cpl;
       NL_PARAMETER  u;
       NL_POINT      C;
       ...
       (define cpl and get u);
       ....
       N_CrvPowerBasisEvalPt(&cpl,u,&C);

     The curve structure NL_CURVE is used to define the polynomial segment; 
     the control points are the vector coefficients, and only two knots, 
     U[0]  and U[1], are  used to  define the  interval  over  which the 
     polynomial is defined.  



   ACCESS:
   
     cpl , input  ,  Polynomial curve
     u   , input  ,  Parameter value 
     C   , output ,  Point on the curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvPowerBasisEvalPt( NL_CURVE *cpl, NL_PARAMETER u, NL_POINT *C )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPowerBasisEvalPt");

    NL_FLAG error = NL_NO;

    NL_INDEX n;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *aw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPts( cpl, &n, &aw );
    N_CrvGetKnotVector( cpl, &knt );

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the point on the curve */

    N_PowerBasisCrvEvalPts( aw, n, u, C );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvPowerBasisEvalPt */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine  reparametrizes a  polynomial curve  given in power 
     basis form. If the output curve is initilized to the NULL curve,
     memory is allocated  locally. Otherwise it is  checked if enough 
     memory is passed in. A typical calling example is:

       NL_CURVE      cplA, cplB;
       NL_PARAMETER  a, b;
       NL_STACKS     SA, SB;
       ...
       (define cplA, get a and b);
       ....
       N_CrvInitArrays(&cplB);
       N_CrvPowerBasisReparam(&cplA,a,b,&cplB,&SA,&SB);
       N_CrvPowerBasisReparam(&cplA,a,b,&cplA,&SA,&SA);

     The  curve  structure NL_CURVE  is used to  define  the  polynomial 
     segment; the  control points  are the  vector  coefficients, and 
     only two knots, U[0]  and U[1], are  used to define the interval  
     over which the polynomial is defined.  



   ACCESS:
   
     cplA , input  ,  Polynomial curve
     a,b  , input  ,  New parameter bounds
     cplB , output ,  Reparametrized curve
     SA   , input  ,  cplA's stack
     SB   , input  ,  cplB's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvPowerBasisReparam( NL_CURVE *cplA, NL_PARAMETER a, NL_PARAMETER b, NL_CURVE *cplB, NL_STACKS *SA, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPowerBasisReparam");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n;

    NL_DEGREE p;

    NL_REAL *A, *B;

    NL_CPOINT *aw, *bw;

    NL_RMATRIX irm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cplA, &n, &aw, &p, &i, &A );

    /* Check if memory is needed */

    if( cplA EQ cplB )
    {
        bw = N_AllocCPt1dArray( n, SA );

        if( bw EQ NULL )
            NL_QUIT;

        B = A;
    }
    else
    {
        error = N_CrvSizeArrays( cplB, n, p, 1, rname, SB );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( cplB, &bw, &B );
    }

    /* Get reparametrization matrix */

    N_InitRealMatrix( &irm );
    error = N_BezInverseReparamMatrix( p, A[0], A[1], a, b, &irm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Reparametrize curve */

    error = N_RealMatrixMultiplyCPtArray( &irm, aw, bw );

    if( error EQ NL_YES )
        NL_OUT;

    B[0] = a;
    B[1] = b;

    /* If in-place reparametrization, kill old coefficients */

    if( cplA EQ cplB )
    {
        N_CrvSetCPtsAndKnots( cplA, bw, B );
        N_FreeCPt1dArray( aw, SA );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvPowerBasisReparam */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a power basis curve. The curve
     may be rational or nonrational. A typical calling example is:

       NL_CURVE      cpl;
       NL_PARAMETER  u;
       NL_INDEX      dr;
       NL_POINT      *CD;
       ...
       (define cpl, get u, dr, and allocate memory for CD);
       ...
       N_CrvPowerBasisEvalDerivs(&cpl,u,dr,CD);

     The curve structure NL_CURVE is used to define the power basis  curve;
     the control points are the vector coefficients, and only two knots,
     U[0] and U[1], are used to define the interval over which the curve
     is defined. 


   ACCESS:
   
     cpl , input  ,  power basis curve
     u   , input  ,  Parameter value 
     dr  , input  ,  Highest derivatives required
     CD  , output ,  Derivatives;  CD[k] is the k-th derivative.  MEMORY
                     FOR CD MUST BE ALLOCATED IN THE CALLING ROUTINE 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvPowerBasisEvalDerivs( NL_CURVE *cpl, NL_PARAMETER u, NL_INDEX dr, NL_POINT *CD )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPowerBasisEvalDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX r, ii, jj, kk, maxd, tmp1, tmp2, bin[NL_MAXDER + 1][NL_MAXDER + 1], xyp;

    NL_DEGREE p;

    NL_REAL up[NL_MAXDEG + 1], w00;

    NL_CPOINT *Pw, CwDers[NL_MAXDER + 1];

    NL_POINT v;

    /* Get local notation */

    p = cpl->p;
    Pw = cpl->pol->Pw;

    if( Pw[0].z EQ NL_NOZ )
        xyp = 1;
    else
        xyp = 0;

    /* Check for parameters out of bounds and dr too big */

    if( u LT cpl->knt->U[0] )
        NL_ERROR( NL_INP_ERR );

    if( u GT cpl->knt->U[1] )
        NL_ERROR( NL_INP_ERR );

    if( dr GT NL_MAXDER )
        NL_ERROR( NL_INP_ERR );

    /* Initialize derivatives to 0 */

    for ( ii = 0; ii <= dr; ii++ )
    {
        CD[ii].x = 0.0;
        CD[ii].y = 0.0;
        CD[ii].z = 0.0;
    }

    /* Now compute all derivatives; treat rational/nonrational */
    /* separately for efficiency                               */

    maxd = NL_MIN( dr, p );

    if( Pw[0].w EQ NL_NOW )
    { /* nonrational */
        up[0] = 1.0;

        for ( ii = 1; ii <= p; ii++ )
            up[ii] = u * up[ii - 1];

        for ( ii = 0; ii <= maxd; ii++ )
        {
            if( ii GT 0 )
            {
                for ( jj = p; jj >= ii; jj-- )
                    up[jj] = jj * up[jj - 1];
            }

            for ( kk = ii; kk <= p; kk++ )
            {
                CD[ii].x += up[kk] * Pw[kk].x;
                CD[ii].y += up[kk] * Pw[kk].y;

                if( xyp EQ 0 )
                    CD[ii].z += up[kk] * Pw[kk].z;
            }
        }
    }
    else
    { /* rational */
        /* Initialize derivatives of Cw(u) to 0 */

        for ( ii = 0; ii <= dr; ii++ )
        {
            CwDers[ii].x = 0.0;
            CwDers[ii].y = 0.0;
            CwDers[ii].z = 0.0;
            CwDers[ii].w = 0.0;
        }

        /* compute derivatives of Cw(u) */

        up[0] = 1.0;

        for ( ii = 1; ii <= p; ii++ )
            up[ii] = u * up[ii - 1];

        for ( ii = 0; ii <= maxd; ii++ )
        {
            if( ii GT 0 )
            {
                for ( jj = p; jj >= ii; jj-- )
                    up[jj] = jj * up[jj - 1];
            }

            for ( kk = ii; kk <= p; kk++ )
            {
                CwDers[ii].x += up[kk] * Pw[kk].x;
                CwDers[ii].y += up[kk] * Pw[kk].y;

                if( xyp EQ 0 )
                    CwDers[ii].z += up[kk] * Pw[kk].z;
                CwDers[ii].w += up[kk] * Pw[kk].w;
            }
        }

        /* Get binomial coefficients */

        bin[0][0] = 1;
        bin[1][0] = 1;
        bin[1][1] = 1;

        for ( kk = 2; kk <= dr; kk++ )
        {
            bin[kk][0] = 1;
            r = kk / 2;
            tmp2 = 1;

            for ( jj = 1; jj <= r; jj++ )
            {
                tmp1 = bin[kk - 1][jj];
                bin[kk][jj] = bin[kk - 1][jj] + tmp2;
                bin[kk][kk - jj] = bin[kk][jj];
                tmp2 = tmp1;
            }

            bin[kk][kk] = 1;
        }

        /* Compute rational derivs from nonrational derivs */

        w00 = CwDers[0].w;

        if ( N_FloatOpIsBad( 1.0, w00, NL_DIVISION ) )
          { NL_ERROR( NL_CON_ERR ); }

        for ( kk = 0; kk <= dr; kk++ )
        {
            v.x = CwDers[kk].x;
            v.y = CwDers[kk].y;
            v.z = CwDers[kk].z;

            for ( ii = 1; ii <= kk; ii++ )
                N_VectorBlendPt( -(bin[kk][ii] * CwDers[ii].w), CD[kk - ii], &v );

            CD[kk].x = v.x / w00;
            CD[kk].y = v.y / w00;
            CD[kk].z = v.z / w00;
        }
    }

    /* Deallocate memory and Exit */

    EXIT:

    return (error);
} /* end N_CrvPowerBasisEvalDerivs */
#endif // NLIB_UNUSED
