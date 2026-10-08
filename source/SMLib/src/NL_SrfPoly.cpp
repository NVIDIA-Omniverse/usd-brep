// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************************/
/* SrfPoly.c : Function Definitions that act on NL_SURFACE objects as polynomials */
/**********************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a  point on a polynomial  surface given in 
     power  basis  form  using  Horner's  method. A  typical  calling 
     example is as follows:

       NL_SURFACE    spl;
       NL_PARAMETER  u, v;
       NL_POINT      S;
       ...
       (define spl, get u and v);
       ....
       N_SrfPowerBasisEvalPt(&spl,u,v,&S);

     The surface structure NL_SURFACE is  used to  define the polynomial 
     patch; the control points are the  vector coefficients, and only 
     four knots, U[0], U[1], V[0] and  V[1], are  used to  define the  
     rectangle  over  which the surface polynomial is defined.  



   ACCESS:
   
     spl , input  ,  Polynomial surface
     u,v , input  ,  Parameter values 
     S   , output ,  Point on the surface


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfPowerBasisEvalPt( NL_SURFACE *spl, NL_PARAMETER u, NL_PARAMETER v, NL_POINT *S )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfPowerBasisEvalPt"); */

    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** aw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPts( spl, &n, &m, &aw );
    N_SrfGetKnotVectors( spl, &knu, &knv );

    /* Allow param to lie outside domain
    error = N_KnotVectorIsParamOutOfBounds(knu,u,rname);
    if (error EQ NL_YES)
    NL_OUT;
    
      error = N_KnotVectorIsParamOutOfBounds(knv,v,rname);
      if (error EQ NL_YES)
      NL_OUT;
    */

    /* Compute the point on the surface */

    N_PowerBasisSrfEvalPts( aw, n, m, u, v, S );

    /* End NURBS */

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfPowerBasisEvalPt */

/*******************************************************************//**


   DESCRIPTION:

     This routine reparametrizes a polynomial surface  given in power 
     basis  form. If  the output  surface is  initilized to  the NULL 
     surface, memory is  allocated  locally. Otherwise it is  checked 
     if enough memory is passed in. A typical calling example is:

       NL_SURFACE    splA, splB;
       NL_PARAMETER  a, b, c, d;
       NL_STACKS     SA, SB;
       ...
       (define splA, get a, b, c and d);
       ....
       N_SrfInitArrays(&splB);
       N_SrfPowerBasisReparam(&splA,a,b,c,d,&splB,&SA,&SB);
       N_SrfPowerBasisReparam(&splA,a,b,c,d,&splA,&SA,&SA);

     The surface structure NL_SURFACE is used to define  the  polynomial 
     patch; the control points are the vector coefficients, and  only 
     four  knots, U[0], U[1], V[0]  and V[1]  are  used to define the 
     rectangle over which the polynomial is defined.  



   ACCESS:
   
     splA , input  ,  Polynomial surface
     a,b  , input  ,  New parameter bounds in u-direction
     c,d  , input  ,  New parameter bounds in v-direction
     splB , output ,  Reparametrized surface
     SA   , input  ,  splA's stack
     SB   , input  ,  splB's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfPowerBasisReparam( NL_SURFACE *splA, NL_PARAMETER a, NL_PARAMETER b, NL_PARAMETER c, NL_PARAMETER d, NL_SURFACE *splB, NL_STACKS *SA, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfPowerBasisReparam");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p, q;

    NL_REAL *AU, *AV, *BU, *BV;

    NL_CPOINT ** aw, ** bw;

    NL_RMATRIX iru, irv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( splA, &n, &m, &aw, &p, &q, &i, &i, &AU, &AV );

    /* Check if memory is needed */

    if( splA EQ splB )
    {
        bw = N_AllocCPt2dArray( n, m, SA );

        if( bw EQ NULL )
            NL_QUIT;

        BU = AU;
        BV = AV;
    }
    else
    {
        error = N_SrfSizeArrays( splB, n, m, p, q, 1, 1, rname, SB );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( splB, &bw, &BU, &BV );
    }

    /* Get reparametrization matrices */

    N_InitRealMatrix( &iru );
    error = N_BezInverseReparamMatrix( p, AU[0], AU[1], a, b, &iru, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &irv );
    error = N_BezInverseReparamMatrix( q, AV[0], AV[1], c, d, &irv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Reparametrize surface */

    error = N_RealMatrixMultiplyRealMatrixTranspose( &iru, aw, &irv, bw );

    if( error EQ NL_YES )
        NL_OUT;

    BU[0] = a;
    BU[1] = b;
    BV[0] = c;
    BV[1] = d;

    /* If in-place reparametrization, kill old coefficients */

    if( splA EQ splB )
    {
        N_SrfSetCPtsAndKnots( splA, bw, BU, BV );
        N_FreeCPt2dArray( aw, SA );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfPowerBasisReparam */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a power basis polynomial surf-
     ace. The surface may be rational or nonrational.  A typical calling
     example is:

       NL_SURFACE    spl;
       NL_PARAMETER  u, v;
       NL_INDEX      udr, vdr;
       NL_POINT      **SD;
       ...
       (define spl, get u, v, udr, vdr, and allocate memory for SD);
       ...
       N_SrfPowerBasisEvalDerivs(&spl,u,v,NL_TRUE,udr,vdr,SD);

     The surface structure NL_SURFACE is used to  define the power basis
     patch; the control points are the  vector coefficients, and only 
     four knots, U[0], U[1], V[0] and  V[1], are  used to  define the  
     rectangle  over  which the surface patch is defined. 


   ACCESS:
   
     spl     , input  ,  power basis surface
     u,v     , input  ,  Parameter values 
     mfl     , input  ,  Flag: 
                           NL_TRUE : compute  upper  half  only  of the 
                                  derivative matrix
                           NL_FALSE: compute full derivative matrix
                         (APPLICABLE ONLY IF udr=vdr!)
     udr,vdr , input  ,  Highest derivatives required
     SD      , output ,  Derivatives;   SD[k][l]   is    the   (k,l)-th 
                         derivative.  MEMORY  FOR SD  MUST BE ALLOCATED 
                         IN THE CALLING ROUTINE TO HOLD SD[udr][vdr].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfPowerBasisEvalDerivs( NL_SURFACE *spl, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_POINT ** SD )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfPowerBasisEvalDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX r, ii, jj, kk, k1 = 0, k2, ll, maxvd, maxud, tmp1, tmp2, bin[NL_MAXDER + 1][NL_MAXDER + 1];

    NL_DEGREE p, q;

    NL_REAL up[NL_MAXDEG + 1], *vp, w00;

    NL_CPOINT temp[NL_MAXDEG + 1], ** Pw, CwDers[NL_MAXDER + 1][NL_MAXDER + 1];

    NL_POINT v1, v2;

    /* Get local notation */

    p = spl->p;
    q = spl->q;

    Pw = spl->net->Pw;

    /* Check for parameters out of bounds and udr,vdr too big */

    /* we want to allow evaluation outside domain
    if (u LT spl->knu->U[0])
    NL_ERROR(NL_INP_ERR);
    if (u GT spl->knu->U[1])
    NL_ERROR(NL_INP_ERR);
    if (v LT spl->knv->U[0])
    NL_ERROR(NL_INP_ERR);
    if (v GT spl->knv->U[1])
    NL_ERROR(NL_INP_ERR);
    */

    if( udr GT NL_MAXDER OR vdr GT NL_MAXDER )
        NL_ERROR( NL_INP_ERR );

    vp = &up[p + 1];

    /* Compute derivative vectors of v-powers */

    vp[0] = 1.0;

    for ( ii = 1; ii <= q; ii++ )
        vp[ii] = v * vp[ii - 1];

    maxvd = NL_MIN( vdr, q );
    k2 = 0;

    for ( ii = 1; ii <= maxvd; ii++ )
    {
        k1 = k2;
        k2 += (q + 1);

        for ( jj = ii; jj <= q; jj++ )
            vp[k2 + jj] = jj * vp[k1 + jj - 1];
    }

    /* Initialize derivatives to 0 */

    for ( ii = 0; ii <= udr; ii++ )
        for ( jj = 0; jj <= vdr; jj++ )
        {
            SD[ii][jj].x = 0.0;
            SD[ii][jj].y = 0.0;
            SD[ii][jj].z = 0.0;
        }

    /* Now compute all derivatives; treat rational/nonrational */
    /* separately for efficiency                               */

    if( Pw[0][0].w EQ NL_NOW )
    { /* nonrational */
        up[0] = 1.0;

        for ( ii = 1; ii <= p; ii++ )
            up[ii] = u * up[ii - 1];

        maxud = NL_MIN( udr, p );

        for ( ii = 0; ii <= maxud; ii++ )
        {
            if( ii GT 0 )
            {
                for ( jj = p; jj >= ii; jj-- )
                    up[jj] = jj * up[jj - 1];
            }

            for ( jj = 0; jj <= q; jj++ )
            {
                temp[jj].x = 0.0;
                temp[jj].y = 0.0;
                temp[jj].z = 0.0;

                for ( kk = ii; kk <= p; kk++ )
                {
                    temp[jj].x += up[kk] * Pw[kk][jj].x;
                    temp[jj].y += up[kk] * Pw[kk][jj].y;
                    temp[jj].z += up[kk] * Pw[kk][jj].z;
                }
            }

            k2 = maxvd;

            if( udr EQ vdr AND mfl EQ NL_TRUE )
                k2 = NL_MIN( k2, udr - ii );

            for ( k1 = 0, jj = 0; jj <= k2; k1 += (q + 1), jj++ )
                for ( kk = jj; kk <= q; kk++ )
                {
                    SD[ii][jj].x += vp[k1 + kk] * temp[kk].x;
                    SD[ii][jj].y += vp[k1 + kk] * temp[kk].y;
                    SD[ii][jj].z += vp[k1 + kk] * temp[kk].z;
                }
        }
    }
    else
    { /* rational */
        /* Initialize derivatives of Cw(u) to 0 */

        for ( ii = 0; ii <= udr; ii++ )
            for ( jj = 0; jj <= vdr; jj++ )
            {
                CwDers[ii][jj].x = 0.0;
                CwDers[ii][jj].y = 0.0;
                CwDers[ii][jj].z = 0.0;
                CwDers[ii][jj].w = 0.0;
            }

        /* compute derivatives of Cw(u) */

        up[0] = 1.0;

        for ( ii = 1; ii <= p; ii++ )
            up[ii] = u * up[ii - 1];

        maxud = NL_MIN( udr, p );

        for ( ii = 0; ii <= maxud; ii++ )
        {
            if( ii GT 0 )
            {
                for ( jj = p; jj >= ii; jj-- )
                    up[jj] = jj * up[jj - 1];
            }

            for ( jj = 0; jj <= q; jj++ )
            {
                temp[jj].x = 0.0;
                temp[jj].y = 0.0;
                temp[jj].z = 0.0;
                temp[jj].w = 0.0;

                for ( kk = ii; kk <= p; kk++ )
                {
                    temp[jj].x += up[kk] * Pw[kk][jj].x;
                    temp[jj].y += up[kk] * Pw[kk][jj].y;
                    temp[jj].z += up[kk] * Pw[kk][jj].z;
                    temp[jj].w += up[kk] * Pw[kk][jj].w;
                }
            }

            k2 = maxvd;

            if( udr EQ vdr AND mfl EQ NL_TRUE )
                k2 = NL_MIN( k2, udr - ii );

            for ( k1 = 0, jj = 0; jj <= k2; k1 += (q + 1), jj++ )
                for ( kk = jj; kk <= q; kk++ )
                {
                    CwDers[ii][jj].x += vp[k1 + kk] * temp[kk].x;
                    CwDers[ii][jj].y += vp[k1 + kk] * temp[kk].y;
                    CwDers[ii][jj].z += vp[k1 + kk] * temp[kk].z;
                    CwDers[ii][jj].w += vp[k1 + kk] * temp[kk].w;
                }
        }

        /* Get binomial coefficients */

        ii = NL_MAX( udr, vdr );

        bin[0][0] = 1;
        bin[1][0] = 1;
        bin[1][1] = 1;

        for ( kk = 2; kk <= ii; kk++ )
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

        w00 = CwDers[0][0].w;

        if ( N_FloatOpIsBad( 1.0, w00, NL_DIVISION ) )
          { NL_ERROR( NL_CON_ERR ); }

        for ( kk = 0; kk <= udr; kk++ )
        {
            k2 = vdr;

            if( mfl EQ NL_TRUE AND k1 EQ k2 )
                k2 = udr - kk;

            for ( ll = 0; ll <= k2; ll++ )
            {
                v1.x = CwDers[kk][ll].x;
                v1.y = CwDers[kk][ll].y;
                v1.z = CwDers[kk][ll].z;

                for ( jj = 1; jj <= ll; jj++ )
                    N_VectorBlendPt( -(bin[ll][jj] * CwDers[0][jj].w), SD[kk][ll - jj], &v1 );

                for ( ii = 1; ii <= kk; ii++ )
                {
                    N_VectorBlendPt( -(bin[kk][ii] * CwDers[ii][0].w), SD[kk - ii][ll], &v1 );
                    N_CopyPt( NL_ZERO, &v2 );

                    for ( jj = 1; jj <= ll; jj++ )
                        N_VectorBlendPt( bin[ll][jj] * CwDers[ii][jj].w, SD[kk - ii][ll - jj], &v2 );
                    N_VectorBlendPt( -bin[kk][ii], v2, &v1 );
                }

                SD[kk][ll].x = v1.x / w00;
                SD[kk][ll].y = v1.y / w00;
                SD[kk][ll].z = v1.z / w00;
            }
        }
    }

    /* Deallocate memory and Exit */

    EXIT:

    return (error);
} /* end N_SrfPowerBasisEvalDerivs */
