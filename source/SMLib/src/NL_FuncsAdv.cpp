// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***************************************************************************/
/* FuncsAdv.c : Advanced Function Definitions that act on NL_VFUN objects  */
/***************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */


#if NLIB_UNUSED



/**********************************************************************/
/* N_CrvFuncEvalRatBasis: Evaluate rational basis function given as curve function */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine evaluates a rational basis function given as a curve
     function, i.e. the  curve function  represents the denominator of
     the form

                     w_{i} N_{i,p}  
          R_{i} = -------------------
                  sum_j w_{j} N_{j,p}

     A typical calling example is:

       NL_CFUN       cfn;
       NL_INDEX      i;
       NL_PARAMETER  u;
       NL_REAL       R;
       ...
       (define cfn; get i and u);
       ...
       N_CrvFuncEvalRatBasis(&cfn,i,u,NL_LEFT,&R);


   ACCESS:
   
     cfn , input  ,  Curve  function (the denomonator of  the rational 
                     form)
     i   , input  ,  Index of rational basis function
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     R   , output ,  Basis function evaluated at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncEvalRatBasis( NL_CFUN *cfn, NL_INDEX i, NL_PARAMETER u, NL_FLAG flg, NL_REAL *R )
{
    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_REAL *w, *tmp, num, den;

    NL_KNOTVECTOR *knt;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvFuncCntrlValKnots( cfn, &w, &tmp );
    N_CFuncGetDegree( cfn, &p );
    N_CFuncGetKnotVector( cfn, &knt );

    /* Get the numerator */

    error = N_BasisIEval( knt, i, p, u, flg, &num );

    if( error EQ NL_YES )
        NL_OUT;

    num = w[i] * num;

    /* Get the denominator */

    error = N_CFuncEval( cfn, u, flg, &den );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute basis function */

    *R = num / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvFuncEvalRatBasis */

/**********************************************************************/
/* N_CrvFuncEvalDerivsAtKnot: Compute derivatives of a curve function wrt a knot       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a  curve function with respect
     to a knot. A typical calling example is:

       NL_CFUN       cfn;
       NL_INDEX      k;
       NL_PARAMETER  u;
       NL_REAL       *fd;
       ...
       (define cfn, get u, and k);
       ...
       N_CrvFuncEvalDerivsAtKnot(&cfn,k,u,NL_LEFT,NL_RIGHT,fd);


   ACCESS:
   
     cfn , input  ,  Curve function
     k   , input  ,  Index of knot, i.e. the derivative  with respect to 
                     u_k is computed
     u   , input  ,  Parameter value 
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k-1)
                       NL_RIGHT: right  derivative.  NL_INDEX  k  MUST SATISFY
                              u_(k) != u_(k+1)
     flp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     fd  , output ,  Function derivative


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncEvalDerivsAtKnot( NL_CFUN *cfn, NL_INDEX k, NL_PARAMETER u, NL_FLAG flk, NL_FLAG flp, NL_REAL *fd )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_DEGREE p;

    NL_REAL *Nk, *fu;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvFuncCntrlVal( cfn, &i, &fu );
    N_CFuncGetDegree( cfn, &p );
    N_CFuncGetKnotVector( cfn, &knt );

    /* Compute derivatives */

    Nk = N_AllocReal1dArray( p + 1, &SL );

    if( Nk EQ NULL )
        NL_QUIT;

    error = N_BasisKnotDerivs( knt, k, p, u, flk, flp, Nk );

    if( error EQ NL_YES )
        NL_OUT;

    *fd = 0.0;

    for ( i = k - p - 1; i <= k; i++ )
    {
        *fd += fu[i] * Nk[i - k + p + 1];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncEvalDerivsAtKnot */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_CrvFuncEvalInvertPt: Curve function value inversion using Newton's method     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine finds the parameter u of a NURBS 
     curve function corresponding to a given value f. The routine uses  
     Newton iteration with a start parameter passed in. The  iteration 
     limit "itl" is also passed in. A typical calling example is:

       NL_CFUN       cfn;
       NL_REAL       f, tol;
       NL_INTEGER    itl;
       NL_PARAMETER  u0, u;
       ...
       (define cfn, get f, u0, itl and tol);
       ...
       N_CrvFuncEvalInvertPt(&cfn,f,u0,tol,itl,&u);

     If convergence is reached, the parameter u is returned. Otherwise 
     the predefined NL_CON_ERR (convergence error) is returned.


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     f   , input  ,  Function value to be inverted
     u0  , input  ,  Guess parameter
     tol , input  ,  Tolerance
     itl , input  ,  Maximum number of iterations allowed
     u   , output ,  Parameter corresponding to f, i.e. f = h(u)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncEvalInvertPt( NL_CFUN *cfn, NL_REAL f, NL_PARAMETER u0, NL_REAL tol, NL_INTEGER itl, NL_PARAMETER *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncEvalInvertPt");

    NL_FLAG error = NL_NO;

    NL_INDEX k, n, m;

    NL_DEGREE p;

    NL_REAL *U, *tmp, fd[2], num, a, b, alf;

    NL_PARAMETER uold, unew;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CFuncGetData( cfn, &n, &tmp, &p, &m, &U );

    /* Check end values */

    error = N_CFuncEval( cfn, U[0], NL_LEFT, &a );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CFuncEval( cfn, U[m], NL_LEFT, &b );

    if( error EQ NL_YES )
        NL_OUT;

    if( fabs( f - a )LT tol )
    {
        *u = U[0];
        NL_OUT;
    }

    if( fabs( b - f )LT tol )
    {
        *u = U[m];
        NL_OUT;
    }

    if( f LT a OR f GT b )
        NL_ERROR( NL_INP_ERR );

    /* Treat p = 1 as special case */

    if( p EQ 1 )
    {
        for ( k = p; k <= n; k++ )
        {
            error = N_CFuncEval( cfn, U[k + 1], NL_LEFT, &b );

            if( error EQ NL_YES )
                NL_OUT;

            if( f LT b AND f GE a )
            {
                alf = (f - a) / (b - a);
                *u = (1.0 - alf) * U[k] + alf * U[k + 1];

                break;
            }
            a = b;
        }

        NL_OUT;
    }

    /* Perform Newton interations till convergence reached */

    k = 0;
    unew = u0;

    while( k LE itl )
    {
        /* Get derivatives */

        error = N_CFuncDerivs( cfn, unew, NL_LEFT, 1, fd );

        if( error EQ NL_YES )
            NL_OUT;

        /* Check convergence */

        num = fd[0] - f;

        if( fabs( num )LE tol )
            break;

        /* No convergence, compute new parameter */

        if( N_FloatOpIsBad( num, fd[1], NL_DIVISION ) )
            NL_ERROR( NL_CON_ERR );

        uold = unew;
        unew = uold - num / fd[1];

        /* Check parameter range */

        if( unew LT U[0] )
            unew = U[0];

        if( unew GT U[m] )
            unew = U[m];

        /* Check parameter change */

        if( fabs( (unew - uold) * fd[1] )LE tol )
            break;

        k++;
    }

    /* If no convergence, quit. Else output parameter */

    if( k GT itl )
        NL_ERROR( NL_CON_ERR );

    *u = unew;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncEvalInvertPt */


/**********************************************************************/
/* N_MapKnotsBetweenCrvFuncAndKnotVector: Map knot vectors onto each other                         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine maps two knot vectors onto each
     other.  More precisely, given curve function f(s) and knot vector
     U, this routine maps the function value f(s_i) of each distinct knot
     s_i in the curve function onto U, and inverse maps (using f^{-1}(s))
     each knot u_i back onto S, where S is the knot vector of the curve
     function. (Note, the function values of the curve function are
     parameter values of U.)  The output are two new knot vectors to
     be used to refine the respective curve or curve function.
     A typical calling example:

       NL_CFUN        cfn;
       NL_KNOTVECTOR  knu, kxu, kxs;
       NL_DEGREE      pu;
       ...
       (define cfn, knu and get pu; get memory for kxu and kxs);
       ...
       N_MapKnotsBetweenCrvFuncAndKnotVector(&cfn,&knu,pu,&kxu,&kxs);

     The  inverse  mapping  uses  Newton  iterations. If convergence is 
     reached, the knot vectors kxu and kxs are  returned. Otherwise the 
     the predefined NL_CON_ERR (convergence error) is  returned. After the
     mappings, the two  knot vectors, S and U, have  the same number of 
     spans that  are images  of  each  other under  f(u) and f^{-1}(u), 
     respectively. IT  IS  ASSUMED  THAT MEMORY  FOR  kxu  AND  kxs  IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     knu , input  ,  Knot vector to which knots of cfn must be mapped,
                     and  whose knots are inverse mapped to cfn's knot
                     vector
     pu  , input  ,  Degree of knu
     kxu , output ,  Knot vector of new knots to be inserted into knu
     kxs , output ,  Knot vector  of knots to  be inserted  into  cfn's 
                     knot vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MapKnotsBetweenCrvFuncAndKnotVector( NL_CFUN *cfn, NL_KNOTVECTOR *knu, NL_DEGREE knuDeg, NL_KNOTVECTOR *kxu, NL_KNOTVECTOR *kxs )
{

    NL_PRIVATE NL_STRING rname = _T("N_MapKnotsBetweenCrvFuncAndKnotVector");
    NL_PRIVATE NL_INTEGER maxIter    = 10;
    NL_PRIVATE NL_INTEGER numPerSpan = 10;

    NL_FLAG error = NL_NO;

    NL_INDEX ii, k, knuKtCnt, cfnKtCnt, mxu, mxs, cfnValCnt, knuValCnt, spn, it, numSpans;

    NL_INTEGER num;

    NL_REAL *knuKtArr, *cfnKtArr, *XU, *XS, *tmpReal, u, u0, um, s = 0.0, sl, sr, s0, ds, dds, tmp;

    NL_DEGREE cfnDeg;

    NL_KNOTVECTOR *kns;

    NL_STACKS SL;

    NL_REAL pTol;

    NL_REAL distLo, distHi, dDelta;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CFuncGetData( cfn, &cfnValCnt, &tmpReal, &cfnDeg, &cfnKtCnt, &cfnKtArr );
    N_CrvFuncCntrlValKnotVector( cfn, &tmpReal, &kns, &cfnKtArr );

    N_KnotVectorGetKnots( knu, &knuKtCnt, &knuKtArr );

    /* Tolerance: make it 1/1000 of the average span size. [B82] */
    N_BasisGetSpanCount( knu, knuDeg, &numSpans );

    pTol = ( knuKtArr[knuKtCnt] - knuKtArr[0] ) / ( numSpans * 1000.0 );

    /* Check end values */

    error = N_CFuncEval( cfn, cfnKtArr[0], NL_LEFT, &u0 );

    if( error EQ NL_YES )
      { NL_OUT; }

    error = N_CFuncEval( cfn, cfnKtArr[cfnKtCnt], NL_LEFT, &um );

    if( error EQ NL_YES )
      { NL_OUT; }

    /* dont be too quick to fail this routine */
    if( fabs( knuKtArr[0] - u0 ) GT pTol )
      { NL_ERROR( NL_INP_ERR ); }

    if( fabs( knuKtArr[knuKtCnt] - um ) GT pTol )
      { NL_ERROR( NL_INP_ERR ); }

    /* Map cfnKtArr's distinct internal knots to knuKtArr */
    /* This loop fills in kxu: mxu and XU.  */

    N_KnotVectorGetKnots( kxu, &mxu, &XU );

    mxu = -1;

    if( cfnValCnt GT cfnDeg )
    {
        ii = cfnDeg + 1;

        while( ii LE cfnValCnt )
        {
            /* Find the next non-zero knot span. */
            while( ii LE cfnValCnt AND cfnKtArr[ii] EQ cfnKtArr[ii+1] )
              { ii++; }

            error = N_CFuncEval( cfn, cfnKtArr[ii], NL_LEFT, &u );

            if( error EQ NL_YES )
              { NL_OUT; }

            error = N_BasisFindSpan( knu, knuDeg, u, NL_LEFT, &spn );

            if( error EQ NL_YES )
              { NL_OUT; }

            /* Check for too close to a knot in knuKtArr. */
            distLo = fabs( knuKtArr[spn  ] - u );
            distHi = fabs( knuKtArr[spn+1] - u );

            if ( distLo LT pTol OR distHi LT pTol )
            {
               /* Note: we don't have the luxury of just skipping knots here:
                * on output, kxu and kxs (here, XU and XS) must have the same
                * number of knot spans.
                * We'll have to tweak the parameter value slightly.  We don't
                * want it extremely close to the existing knot, and we don't
                * want to put it right at the existing knot because that would
                * introduce a multiple knot which could have unwanted effects
                * on the curve.  So move it a small distance away from the knot;
                * this will have an effectively imperceptible influence on the curve.
                * Put it 0.01 of the span width into the span.  That's a pretty
                * small ratio for adjacent knot spans, but in practice it works
                * just fine. [B82 B135]
                */
                dDelta = ( knuKtArr[spn+1] - knuKtArr[spn] ) * 0.01;
                if ( distLo LT distHi )
                  { u = knuKtArr[spn  ] + dDelta; }
                else
                  { u = knuKtArr[spn+1] - dDelta; }
            }

            /* Make sure that it's bigger than the previous knot.   */
            /* This can happen if we applied the above knot-pushing */
            /* trick on this iteration, or a previous one. [B168]   */
            /* In this case it works just to swap the knot values.  */

            if ( mxu >= 0 AND u < XU[mxu] )
            {
                tmp = XU[mxu];
                XU[mxu] = u;
                u = tmp;
            }

            /* Record the mapped parameter. */
            XU[++mxu] = u;

            ii++;

        } /* end while ii LE cfnValCnt */
    }

    /* Inverse map knuKtArr's distinct internal knots to cfnKtArr */
    /* This loop fills in kxs: mxs and XS.  */

    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    knuValCnt = knuKtCnt - knuDeg - 1;

    mxs = -1;

    if( knuValCnt GT knuDeg )
    {
        /* Estimate number of sampling points */

        N_BasisGetLongestAndShortestSpans( knu, knuDeg, &u0, &um );

        num = (NL_INTEGER)( numPerSpan * (knuKtArr[knuKtCnt] - knuKtArr[0]) / u0 );
        ds = (cfnKtArr[cfnKtCnt] - cfnKtArr[0]) / num;

        /* Inverse map each distinct u-knot */

        sr = cfnKtArr[0];
        ii = knuDeg + 1;

        while( ii LE knuValCnt )
        {
            /* Find the next non-zero knot span. */
            while( ii LE knuValCnt AND knuKtArr[ii] EQ knuKtArr[ii+1] )
              { ii++; }

            /* Bracket the guess parameter.
             * Loop on sr: keep adding ds to it until
             * the cfn function value at sr exceeds knuKtArr[ii].
             */

            while( sr LE cfnKtArr[cfnKtCnt] )
            {
                sr = sr + ds;

                if( sr GT cfnKtArr[cfnKtCnt] )
                  { sr =  cfnKtArr[cfnKtCnt]; }

                error = N_CFuncEval( cfn, sr, NL_LEFT, &u );

                if( error EQ NL_YES )
                  { NL_OUT; }

                if( u GT knuKtArr[ii] )
                  { break; }
            }

            /* Now f(sr) > the knot [ii], and f(sr-ds) was less,
             * So sr-ds and sr bracket the knot.
             */
            sl = sr - ds;

            /* Start Newton the midpoint of [sl,sr]. */

            it = 0;

            while( it LT maxIter )
            {
                s0 = 0.5 *( sl + sr );

                error = N_CrvFuncEvalInvertPt( cfn, knuKtArr[ii], s0, pTol, maxIter, &s );

                if( error EQ NL_NO )
                {
                    /* Convergence -> output knot */

                    error = N_BasisFindSpan( kns, cfnDeg, s, NL_LEFT, &spn );

                    if( error EQ NL_YES )
                      { NL_OUT; }

                    /* Check for too close to a knot in cfnKtArr. */
                    distLo = fabs( cfnKtArr[spn  ] - s );
                    distHi = fabs( cfnKtArr[spn+1] - s );

                    if ( distLo LT pTol OR distHi LT pTol )
                    {
                       /* We can't just skip knots here: see note above.  [B82 B135] */

                       dDelta = ( cfnKtArr[spn+1] - cfnKtArr[spn] ) * 0.01;
                       if ( distLo LT distHi )
                         { s = cfnKtArr[spn  ] + dDelta; }
                       else
                         { s = cfnKtArr[spn+1] - dDelta; }

                        /* But make sure that it's bigger than the previous knot. */
                        if ( mxs GE 0 )
                        {
                            if ( s LT XS[mxs] + pTol )
                            {
                                dDelta = ( cfnKtArr[spn+1] - XS[mxs] ) * 0.01;
                                if ( distLo LE pTol )
                                  { s = cfnKtArr[spn  ] + dDelta; }
                                else
                                  { s = cfnKtArr[spn+1] - dDelta; }
                            }
                        }
                    }

                    /* But make sure that it's bigger than the previous knot. */
                    /* This can happen if we applied the above knot-pushing   */
                    /* trick on the previous iteration. [B168]                */
                    /* In this case it works just to swap the knot values.    */

                    if ( mxs >= 0 AND s < XS[mxs] )
                    {
                        tmp = XS[mxs];
                        XS[mxs] = s;
                        s = tmp;
                    }

                    /* Record the mapped parameter. */
                    XS[++mxs] = s;
                    break;
                }
                else
                {
                    /* No convergence -> refine [sl,sr] */

                    dds = (sr - sl) / numPerSpan;

                    for ( k = 0; k <= numPerSpan; k++ )
                    {
                        sr = sl + k * dds;

                        error = N_CFuncEval( cfn, sr, NL_LEFT, &u );

                        if( error EQ NL_YES )
                          { NL_OUT; }

                        if( u GT knuKtArr[ii] )
                          { break; }
                    }
                    sl = sr - dds;
                }
                it++;

            } /* End iter on it, while convergence is not reached. */


            if( it GT maxIter )
              { NL_ERROR( NL_CON_ERR ); }

            sr = s;
            ii++;

        } /* end while ii LE knuValCnt */
    }

    /* Define output knot vectors */

    N_SetKnotIndex( kxu, mxu );
    N_SetKnotIndex( kxs, mxs );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_MapKnotsBetweenCrvFuncAndKnotVector */


/**********************************************************************/
/* N_CRVFUNCREFINE: Refine a NURBS curve function with a given knot vector   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools routine refines a NURBS curve function with a given knot 
     vector. It is assumed that the  new knot vector "fits" into the old 
     one, i.e. U[0] < X[0] <=...<= X[r] < U[m] holds where U[0],...,U[m] 
     are the old knots and X[0],...,X[r] are the new ones. If the output 
     function is initialized to NULL, memory to store new control values 
     and knots is  allocated. If the output function is  the same as the 
     input function, knot refinement is done in place  and the  original 
     function is destroyed. A typical calling example is:

       NL_CFUN        cfnP, cfnQ;
       NL_KNOTVECTOR  knx;
       NL_STACKS      SP, SQ;
       ...
       (define cfnP and knx);
       ...
       N_CFuncInitArrays(&cfnQ);
       N_CrvFuncRefine(&cfnP,&knx,&cfnQ,&SP,&SQ);
       N_CrvFuncRefine(&cfnP,&knx,&cfnP,&SP,&SP);

     If  memory is  available,  cfnQ is not  initialized and the routine
     assumes  that memory  allocation has  been done. However, it checks  
     for the proper  amount  by looking at the highest indexes in cfnQ's  
     knot vector and polygon objects.


   ACCESS:
   
     cfnP , input  ,  NURBS curve function
     knx  , input  ,  New knot vector
     cfnQ , output ,  Curve function after knot refinement
     SP   , input  ,  cfnP's stack
     SQ   , input  ,  cfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncRefine( NL_CFUN *cfnP, NL_KNOTVECTOR *knx, NL_CFUN *cfnQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncRefine");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, a, b, n, m, r, t;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *X, *fp, *fq, alf, oma;

    NL_KNOTVECTOR *knt;

    NL_CFUN cfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CFuncGetData( cfnP, &n, &fp, &p, &m, &UP );
    N_CFuncGetKnotVector( cfnP, &knt );
    N_KnotVectorGetKnots( knx, &r, &X );

    /* Check input parameters  */

    if( r LT 0 )
        NL_ERROR( NL_INP_ERR );

    error = N_KnotVectorIsEndParam( knt, X[0], rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsEndParam( knt, X[r], rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed */

    if( cfnP EQ cfnQ )
    {
        cfnA = *cfnP;

        error = N_AllocCFuncArrays( cfnP, n + r + 1, p, m + r + 1, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnP, &fq, &UQ );
    }
    else
    {
        error = N_CFuncSizeArrays( cfnQ, n + r + 1, p, m + r + 1, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnQ, &fq, &UQ );
    }

    /* Find knot spans */

    error = N_BasisFindSpan( knt, p, X[0], NL_LEFT, &a );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpan( knt, p, X[r], NL_LEFT, &b );

    if( error EQ NL_YES )
        NL_OUT;
    b++;

    /* Initialize output knot vector */

    for ( j = 0; j <= a; j++ )
        UQ[j] = UP[j];

    for ( j = b + p; j <= m; j++ )
        UQ[j + r + 1] = UP[j];

    /* Save unaltered control values */

    for ( j = 0; j <= a - p; j++ )
        fq[j] = fp[j];

    for ( j = b - 1; j <= n; j++ )
        fq[j + r + 1] = fp[j];

    /* Now refine the knot vector */

    i = b + p - 1;
    k = b + p + r;

    for ( j = r; j >= 0; j-- )
    {
        while( X[j]LE UP[i]AND i GT a )
        {
            fq[k - p - 1] = fp[i - p - 1];
            UQ[k] = UP[i];
            k--;
            i--;
        }

        fq[k - p - 1] = fq[k - p];

        for ( l = 1; l <= p; l++ )
        {
            t = k - p + l;
            alf = UQ[k + l] - X[j];

            if( fabs( alf )LE 0.0 )
            {
                fq[t - 1] = fq[t];
            }
            else
            {
                alf = alf / (UQ[k + l] - UP[i - p + l]);
                oma = 1.0 - alf;
                fq[t - 1] = oma * fq[t] + alf * fq[t - 1];
            }
        }
        UQ[k] = X[j];
        k--;
    }

    /* If refinement is in place, kill old curve function */

    if( cfnP EQ cfnQ )
        N_FreeCrvFunc( &cfnA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncRefine */

#if NLIB_UNUSED

/**********************************************************************/
/* N_CrvFuncDegreeElevate: Elevate the degree of a curve function                   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine elevates the  degree of a  curve function  from 
     any degree to any higher degree (below the allowed maximum). If the
     output function is initialized to NULL, memory is allocated. If the  
     output function is the same as the input function, degree elevation 
     is done in place and the  original function is destroyed. A typical 
     calling example is:

       NL_CFUN    cfnP, cfnQ;
       NL_INDEX   t;
       NL_STACKS  SP, SQ;
       ...
       (define cfnP, get t);
       ...
       N_CFuncInitArrays(&cfnQ);
       N_CrvFuncDegreeElevate(&cfnP,t,&cfnQ,&SP,&SQ);
       N_CrvFuncDegreeElevate(&cfnP,t,&cfnP,&SP,&SP);

     If memory is  available, cfnQ  is not  initialized  and the routine
     assumes  that memory  allocation has been done.  However, it checks  
     for the proper  amount by looking  at the highest indexes in cfnQ's  
     knot vector and control value objects.

   ACCESS:
   
     cfnP , input  ,  Curve function
     t    , input  ,  Increment (0<=t<=original_degree+NL_DMAX)
     cfnQ , output ,  Function after degree elevation
     SP   , input  ,  cfnP's stack
     SQ   , input  ,  cfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncDegreeElevate( NL_CFUN *cfnP, NL_INDEX t, NL_CFUN *cfnQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncDegreeElevate");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, np, mp, nq, mq, mlt, r, s, a, b, kind, find, first, last, oldr, save, lbz, rbz;

    NL_DEGREE p, q;

    NL_REAL *fp, *fq, *fb, *fn, *fd, *UP, *UQ, *alfs, *omas, alf, oma, bet, omb, gam, omg, num, den;

    NL_RMATRIX dm;

    NL_KNOTVECTOR *knt;

    NL_CFUN cfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CFuncGetData( cfnP, &np, &fp, &p, &mp, &UP );
    N_CFuncGetKnotVector( cfnP, &knt );

    /* Check error */

    if( t LT 0 OR p + t GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    error = N_KnotVectorIsValid( knt, p, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed */

    N_BasisGetSpanCount( knt, p, &s );

    nq = np + t * s;
    q = (NL_DEGREE)( p + t );
    mq = mp + t * (s + 1);

    if( cfnP EQ cfnQ )
    {
        cfnA = *cfnP;

        error = N_AllocCFuncArrays( cfnP, nq, q, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnP, &fq, &UQ );
    }
    else
    {
        error = N_CFuncSizeArrays( cfnQ, nq, q, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnQ, &fq, &UQ );
    }

    /* Get local memory */

    fb = N_AllocReal1dArray( p, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    fn = N_AllocReal1dArray( p, &SL );

    if( fn EQ NULL )
        NL_QUIT;

    fd = N_AllocReal1dArray( q, &SL );

    if( fd EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    a = p;
    b = p + 1;
    r = -1;
    find = 1;
    kind = q + 1;
    fq[0] = fp[0];

    for ( i = 0; i <= q; i++ )
        UQ[i] = UP[a];

    for ( i = 0; i <= p; i++ )
        fb[i] = fp[i];

    /* Get degree elevation matrix */

    N_InitRealMatrix( &dm );
    error = N_BezGetDegreeElevationMatrix( p, t, &dm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*************************************************************/
    /* Loop through the knot vector and do the following:        */
    /*   (1) Extract the i-th Bezier segment.                    */
    /*   (2) Degree elevate the segment.                         */
    /*   (3) Remove the knot between the i-th and the (i-1)-th   */
    /*       segment.                                            */
    /*************************************************************/

    while( b LT mp )
    {
        /* Get multiplicity of the knot */

        i = b;

        while( b LT mp AND UP[b]EQ UP[b + 1] )
            b++;
        mlt = b - i + 1;

        /* Insert knot to get Bezier segment */

        oldr = r;
        r = p - mlt;

        if( oldr GT 0 )
            lbz = (oldr + 2) / 2;
        else
            lbz = 1;

        if( r GT 0 )
            rbz = q - (r + 1) / 2;
        else
            rbz = q;

        if( r GT 0 )
        {
            /* Save the alphas */

            num = UP[b] - UP[a];

            for ( k = p; k > mlt; k-- )
            {
                alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
            }

            /* Now insert the knot */

            for ( j = 1; j <= r; j++ )
            {
                save = r - j;
                s = mlt + j;

                for ( k = p; k >= s; k-- )
                {
                    fb[k] = alfs[k - s] * fb[k] + omas[k - s] * fb[k - 1];
                }
                fn[save] = fb[p];
            }
        } /* End of insert knot */

        /* Now degree elevate Bezier segment */

        error = N_BezFuncDegreeElevate( fb, p, t, &dm, lbz, q, fd );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot UP[a] */

        if( oldr GT 1 )
        {
            first = kind - 2;
            last = kind;
            den = UP[b] - UP[a];
            bet = (UP[b] - UQ[kind - 1]) / den;
            omb = 1.0 - bet;

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT find )
                    {
                        alf = (UP[b] - UQ[i]) / (UP[a] - UQ[i]);
                        oma = 1.0 - alf;
                        fq[i] = alf * fq[i] + oma * fq[i - 1];
                    }

                    if( j GE lbz )
                    {
                        if( (j - k)LE( kind - q + oldr ) )
                        {
                            gam = (UP[b] - UQ[j - k]) / den;
                            omg = 1.0 - gam;
                            fd[l] = gam * fd[l] + omg * fd[l + 1];
                        }
                        else
                        {
                            fd[l] = bet * fd[l] + omb * fd[l + 1];
                        }
                    }
                    i++;
                    j--;
                    l--;
                }
                first--;
                last++;
            }
        } /* End of removing knot */

        /* Load knot vector and control values */

        if( a NEQ p )
        {
            for ( i = 0; i < q - oldr; i++ )
            {
                UQ[kind] = UP[a];
                kind++;
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            fq[find] = fd[i];
            find++;
        }

        /* Initialize for next pass through */

        if( b LT mp )
        {
            for ( i = 0; i < r; i++ )
                fb[i] = fn[i];

            for ( i = r; i <= p; i++ )
                fb[i] = fp[b - p + i];

            a = b;
            b++;
        }
        else
        {
            /* Get the end knot */

            for ( i = 0; i <= q; i++ )
                UQ[kind + i] = UP[b];
        }
    } /* End of while loop */

    /* If degree elevation is in place, kill old function */

    if( cfnP EQ cfnQ )
        N_FreeCrvFunc( &cfnA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end toocfe */

/**********************************************************************/
/* N_CRVFUNCINSERTKNOT: Insert a knot into a function                            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine  inserts a new knot  into a curve function. The 
     new knot must be an interior knot and the sum of the multiplicities
     of  the old  and the  new knots  must be  less than or equal to the 
     degree. If the output  function is  initialized to NULL,  memory to 
     store new  control values  and knots  is  allocated. If  the output 
     function is the same as the input function, knot  insertion is done 
     in place and the original function is destroyed. A typical  calling 
     example is: 

       NL_CFUN       cfnP, cfnQ;
       NL_PARAMETER  u;
       NL_INDEX      r;
       NL_STACKS     SP, SQ;
       ...
       (define cfnP, get u and r);
       ...
       N_CFuncInitArrays(&cfnQ);
       N_CrvFuncInsertKnot(&cfnP,u,r,&cfnQ,&SP,&SQ);
       N_CrvFuncInsertKnot(&cfnP,u,r,&cfnP,&SP,&SP);

     If memory is  available, cfnQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in cfnQ's  
     knot vector and  polygon objects.


   ACCESS:
   
     cfnP , input  ,  Function
     u    , input  ,  New knot
     r    , input  ,  Number of times u is to be inserted
     cfnQ , output ,  Function after knot inserion
     SP   , input  ,  cfnP's stack
     SQ   , input  ,  cfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncInsertKnot( NL_CFUN *cfnP, NL_PARAMETER u, NL_INDEX r, NL_CFUN *cfnQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncInsertKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, spn, mlt;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *fp, *fq, *fr, alf, oma;

    NL_KNOTVECTOR *knt;

    NL_CFUN cfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CFuncGetData( cfnP, &n, &fp, &p, &m, &UP );
    N_CFuncGetKnotVector( cfnP, &knt );

    /* Check parameter and get knot span */

    error = N_KnotVectorIsEndParam( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpanAndMult( knt, p, u, NL_LEFT, &spn, &mlt );

    if( error EQ NL_YES )
        NL_OUT;

    if( (mlt + r)GT p OR r LT 0 )
        NL_ERROR( NL_PAR_ERR );

    k = spn - p;

    /* See if memory is needed */

    if( cfnP EQ cfnQ )
    {
        cfnA = *cfnP;

        error = N_AllocCFuncArrays( cfnP, n + r, p, m + r, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnP, &fq, &UQ );
    }
    else
    {
        error = N_CFuncSizeArrays( cfnQ, n + r, p, m + r, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnQ, &fq, &UQ );
    }

    /* Get auxiliary control values */

    fr = N_AllocReal1dArray( p, &SL );

    if( fr EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p - mlt; i++ )
        fr[i] = fp[k + i];

    /* Save unaltered control values */

    for ( i = 0; i <= k; i++ )
        fq[i] = fp[i];

    for ( i = spn - mlt; i <= n; i++ )
        fq[i + r] = fp[i];

    /* Now insert the knot */

    for ( i = 1; i <= r; i++ )
    {
        k = spn - p + i;

        for ( j = 0; j <= p - i - mlt; j++ )
        {
            alf = (u - UP[k + j]) / (UP[spn + j + 1] - UP[k + j]);
            oma = 1.0 - alf;
            fr[j] = alf * fr[j + 1] + oma * fr[j];
        }
        fq[k] = fr[0];
        fq[spn + r - i - mlt] = fr[p - i - mlt];
    }

    /* Load the remaining control values */

    for ( i = k + 1; i < spn - mlt; i++ )
        fq[i] = fr[i - k];

    /* Load the knot vector */

    for ( i = 0; i <= spn; i++ )
        UQ[i] = UP[i];

    for ( i = 1; i <= r; i++ )
        UQ[i + spn] = u;

    for ( i = spn + 1; i <= m; i++ )
        UQ[i + r] = UP[i];

    /* If insertion is in place, kill old function */

    if( cfnP EQ cfnQ )
        N_FreeCrvFunc( &cfnA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncInsertKnot */

/**********************************************************************/
/* N_CRVFUNCDECOMPOSE: Decompose a curve function into Bezier pieces            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  decomposes a curve  function  into its Bezier 
     constituents  without   using  knot   refinement.   Each  piece  is 
     represented as a NURBS function even though the segments are Bezier
     functions. MEMORY TO STORE THE OUTPUT FUNCTIONS IS ALLOCATED INSIDE 
     THE ROUTINE. A typical calling example is:

       NL_CFUN    cfnP, **cfnQ;
       NL_INDEX   k;
       NL_STACKS  SQ;
       ...
       (define cfnP);
       ...
       N_CrvFuncDecompose(&cfnP,&cfnQ,&k,&SQ);

     cfnQ[i], 0<=i<=k, is a pointer to the i-th function.


   ACCESS:
   
     cfnP , input  ,  Function to be decomposed
     cfnQ , output ,  Array of Bezier functions
     k    , output ,  Highest index in cfnQ
     SQ   , input  ,  cfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncDecompose( NL_CFUN *cfnP, NL_CFUN *** cfnQ, NL_INDEX *k, NL_STACKS *SQ )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s, nsp, mlt, is, ie, iq, save;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *alfs, *omas, *fp, *fq, *nq = NULL, num;

    NL_KNOTVECTOR *knt;

    NL_CFUN ** cfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CFuncGetData( cfnP, &n, &fp, &p, &m, &UP );
    N_CFuncGetKnotVector( cfnP, &knt );

    /* Get number of segments and allocate memory */

    N_BasisGetSpanCount( knt, p, &nsp );

    cfnA = N_AllocCrvFuncArray( p, p, 2 * p + 1, nsp - 1, SQ );

    if( cfnA EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    is = p;
    ie = p + 1;
    iq = -1;

    /* skip over first knot over-multiplicity  ( bad data) */
    while( ie LT m AND UP[ie - 1]EQ UP[ie] )
    {
        is++;
        ie++;
    }

    N_CrvFuncCntrlVal( cfnA[0], &i, &fq );

    for ( i = 0; i <= p; i++ )
        fq[i] = fp[i + is - p];

    /* Loop through the knot vector and extract each segment */

    while( ie LT m )
    {
        /* Initialize */

        iq = iq + 1;

        if( iq >= nsp )
            NL_QUIT;

        N_CrvFuncCntrlValKnots( cfnA[iq], &fq, &UQ );

        if( iq LT nsp - 1 )
            N_CrvFuncCntrlVal( cfnA[iq + 1], &i, &nq );

        /* Get knot multiplicity */

        i = ie;

        while( ie LT m AND UP[ie]EQ UP[ie + 1] )
            ie++;
        mlt = ie - i + 1;
        r = p - mlt;

        /* Insert the knot */

        if( mlt LT p )
        {
            num = UP[ie] - UP[is];

            for ( i = p; i > mlt; i-- )
            {
                alfs[i - mlt - 1] = num / (UP[is + i] - UP[is]);
                omas[i - mlt - 1] = 1.0 - alfs[i - mlt - 1];
            }

            for ( i = 1; i <= r; i++ )
            {
                s = mlt + i;
                save = r - i;

                for ( j = p; j >= s; j-- )
                {
                    fq[j] = alfs[j - s] * fq[j] + omas[j - s] * fq[j - 1];
                }

                if( ie LT m )
                    nq[save] = fq[p];
            }
        }

        /* Get knot vector */

        for ( i = 0; i <= p; i++ )
        {
            UQ[i] = UP[is];
            UQ[i + p + 1] = UP[ie];
        }

        /* Segment completed - prepare for next piece */

        if( ie LT m )
        {
            for ( i = r; i <= p; i++ )
                nq[i] = fp[ie - p + i];
        }
        is = ie;
        ie = ie + 1;
    }

    *k = nsp - 1;
    *cfnQ = cfnA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncDecompose */

#endif // NLIB_UNUSED

/* Advanced NL_CFUN and NL_CVALUE Symbolic operators */

/**********************************************************************/
/* N_CrvFuncMultiplyCrvFunc: Compute the product of two univariate B-spline functions */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the  product of two curve
     functions. The knot vectors of the  functions are  rescaled to the
     unit span [0,1], however, the input functions do not change either 
     parametrically or geometrically. A typical calling example is:

       NL_CFUN    cfnF, cfnG, cfnH;
       NL_STACKS  SH;
       ...
       (define cfnF and cfnG);
       ...
       N_CFuncInitArrays(&cfnH);
       N_CrvFuncMultiplyCrvFunc(&cfnF,&cfnG,&cfnH,&SH);

     If memory is  available, cfnH  is not initialized and  the routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in cfnH's  
     knot  vector and control  value objects. THE STORAGE OF THE OUTPUT 
     NL_FUNCTION IS COMPACTED, I.E. THE MEMORY PASSED IN IS DESTROYED.


   ACCESS:
   
     cfnF , in/out ,  Function f (ITS KNOT NL_VECTOR IS RESCALED)
     cfnG , in/out ,  Function g (ITS KNOT NL_VECTOR IS RESCALED)
     cfnH , output ,  Product function f*g
     SH   , input  ,  cfnH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncMultiplyCrvFunc( NL_CFUN *cfnF, NL_CFUN *cfnG, NL_CFUN *cfnH, NL_STACKS *SH )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncMultiplyCrvFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, nf, mr, ng, ms, nh, mt, mlr, mls, mir, mis, mi, rr, rs, s, ar, as, br, bs, mxr, mxs, ixr, ixs, t, kind, find, first, last, rem, save, nsr, nss, lbz, rbz;

    NL_DEGREE p, q, pq;

    NL_REAL *R, *S, *T, *XR, *XS, *f, *g, *h, *fb, *fn, *gb, *gn, *fg, *alfr, *omar, *alfs, *omas, alf, oma, bet, omb, gam, omg, num, den;

    NL_KNOTVECTOR *knr, *kns, *kxr, *kxs;

    NL_CFUN cfnFR, cfnGR;

    NL_RMATRIX pm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get new degree */

    N_CFuncGetDegree( cfnF, &p );
    N_CFuncGetDegree( cfnG, &q );

    pq = p + q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Handle special case p=0 or q=0 */

    if( p EQ 0 OR q EQ 0 )
    {
        N_CFuncGetArraySizes( cfnF, &i, &j );
        N_CFuncGetArraySizes( cfnG, &k, &l );

        if( i NEQ p OR k NEQ q )
            NL_ERROR( NL_INP_ERR );

        error = N_CFuncSizeArrays( cfnH, pq, pq, pq + pq + 1, rname, SH );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnF, &f, &R );
        N_CrvFuncCntrlValKnots( cfnG, &g, &S );
        N_CrvFuncCntrlValKnots( cfnH, &h, &T );

        if( p EQ 0 )
        {
            for ( j = 0; j <= q; j++ )
            {
                h[j] = f[0] * g[j];
                T[j] = S[j];
                T[q + j + 1] = S[q + j + 1];
            }
        }
        else
        {
            for ( i = 0; i <= p; i++ )
            {
                h[i] = g[0] * f[i];
                T[i] = R[i];
                T[p + i + 1] = R[p + i + 1];
            }
        }

        NL_OUT;
    }

    /* Merge knots */

    N_CFuncGetKnotVector( cfnF, &knr );
    N_CFuncGetKnotVector( cfnG, &kns );

    N_BasisGetSpanCount( knr, p, &nsr );
    N_BasisGetSpanCount( kns, q, &nss );

    kxr = N_AllocKnotVectorAndArray( nss, &SL );

    if( kxr EQ NULL )
        NL_QUIT;

    kxs = N_AllocKnotVectorAndArray( nsr, &SL );

    if( kxs EQ NULL )
        NL_QUIT;

    N_MakeKnotsCompatible( knr, kns, p, q, kxr, kxs );

    /* Refine functions */

    N_KnotVectorGetKnots( kxr, &mxr, &XR );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( mxr GE 0 )
    {
        N_CFuncInitArrays( &cfnFR );
        error = N_CrvFuncRefine( cfnF, kxr, &cfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CFuncGetData( &cfnFR, &nf, &f, &p, &mr, &R );
        N_CFuncGetKnotVector( &cfnFR, &knr );
        N_BasisGetSpanCount( knr, p, &nsr );
    }
    else
    {
        N_CFuncGetData( cfnF, &nf, &f, &p, &mr, &R );
    }

    if( mxs GE 0 )
    {
        N_CFuncInitArrays( &cfnGR );
        error = N_CrvFuncRefine( cfnG, kxs, &cfnGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CFuncGetData( &cfnGR, &ng, &g, &q, &ms, &S );
        N_CFuncGetKnotVector( &cfnGR, &kns );
        N_BasisGetSpanCount( kns, q, &nss );
    }
    else
    {
        N_CFuncGetData( cfnG, &ng, &g, &q, &ms, &S );
    }

    if( nsr NEQ nss )
        NL_ERROR( NL_KNT_ERR );

    /* See if memory is needed */

    nh = nsr * pq;
    mt = nh + pq + 1;

    error = N_CFuncSizeArrays( cfnH, nh, pq, mt, rname, SH );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( cfnH, &h, &T );

    /* Get local memory */

    fb = N_AllocReal1dArray( p, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    fn = N_AllocReal1dArray( p, &SL );

    if( fn EQ NULL )
        NL_QUIT;

    gb = N_AllocReal1dArray( q, &SL );

    if( gb EQ NULL )
        NL_QUIT;

    gn = N_AllocReal1dArray( q, &SL );

    if( gn EQ NULL )
        NL_QUIT;

    fg = N_AllocReal1dArray( pq, &SL );

    if( fg EQ NULL )
        NL_QUIT;

    alfr = N_AllocReal1dArray( p, &SL );

    if( alfr EQ NULL )
        NL_QUIT;

    omar = N_AllocReal1dArray( p, &SL );

    if( omar EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( q, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( q, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    ar = p;
    br = p + 1;
    as = q;
    bs = q + 1;
    rr = -1;
    rs = -1;
    ixr = 0;
    ixs = 0;

    find = 1;
    kind = pq + 1;
    rem = -1;

    for ( i = 0; i <= pq; i++ )
        T[i] = S[as];

    for ( i = 0; i <= p; i++ )
        fb[i] = f[i];

    for ( j = 0; j <= q; j++ )
        gb[j] = g[j];

    h[0] = f[0] * g[0];

    N_InitRealMatrix( &pm );
    error = N_BezFuncMultiplyBezCrvMatrix( p, q, &pm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /***************************************************************/
    /* Loop through the knot vectors and do the following:         */
    /*   (1) Extract the i-th Bezier functions.                    */
    /*   (2) Compute their product.                                */
    /*   (3) Remove the knot(s) between the i-th and the (i-1)-th  */
    /*       segments.                                             */
    /***************************************************************/

    while( br LT mr AND bs LT ms )
    {
        /* Get multiplicities of the knots */

        i = br;

        while( br LT mr AND R[br]EQ R[br + 1] )
            br++;
        mlr = br - i + 1;
        mir = mlr;

        j = bs;

        while( bs LT ms AND S[bs]EQ S[bs + 1] )
            bs++;
        mls = bs - j + 1;
        mis = mls;

        /* Check multiplicities of internal knots */

        if( br LT mr AND bs LT ms )
        {
            if( mlr GT p OR mls GT q )
                NL_ERROR( NL_KNT_ERR );
        }

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mir EQ 1 AND ixr LE mxr )
        {
            if( fabs( R[br] - XR[ixr] )LT NL_PTOL )
            {
                mir = 0;
                ixr++;
            }
        }

        if( mis EQ 1 AND ixs LE mxs )
        {
            if( fabs( S[bs] - XS[ixs] )LT NL_PTOL )
            {
                mis = 0;
                ixs++;
            }
        }

        if( mis EQ 0 )
            mi = q + mir;

        else if( mir EQ 0 )
            mi = p + mis;

        else
            mi = NL_MAX( q + mir, p + mis );

        /* Insert knot to get Bezier functions */

        rr = p - mlr;
        rs = q - mls;
        t = pq - mi;

        if( rem GT 0 )
            lbz = (rem + 2) / 2;
        else
            lbz = 1;

        if( t GT 0 )
            rbz = pq - (t + 1) / 2;
        else
            rbz = pq;

        if( rr GT 0 )
        {
            num = R[br] - R[ar];

            for ( k = p; k > mlr; k-- )
            {
                alfr[k - mlr - 1] = num / (R[ar + k] - R[ar]);
                omar[k - mlr - 1] = 1.0 - alfr[k - mlr - 1];
            }

            for ( j = 1; j <= rr; j++ )
            {
                save = rr - j;
                s = mlr + j;

                for ( k = p; k >= s; k-- )
                {
                    fb[k] = alfr[k - s] * fb[k] + omar[k - s] * fb[k - 1];
                }
                fn[save] = fb[p];
            }
        }

        if( rs GT 0 )
        {
            num = S[bs] - S[as];

            for ( k = q; k > mls; k-- )
            {
                alfs[k - mls - 1] = num / (S[as + k] - S[as]);
                omas[k - mls - 1] = 1.0 - alfs[k - mls - 1];
            }

            for ( j = 1; j <= rs; j++ )
            {
                save = rs - j;
                s = mls + j;

                for ( k = q; k >= s; k-- )
                {
                    gb[k] = alfs[k - s] * gb[k] + omas[k - s] * gb[k - 1];
                }
                gn[save] = gb[q];
            }
        }

        /* Compute product of Bezier functions */

        error = N_BezFuncMultiplyBezFunc( fb, p, gb, q, &pm, lbz, pq, fg );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot R[ar] = S[as] */

        if( rem GT 1 )
        {
            first = kind - 2;
            last = kind;
            den = S[bs] - S[as];
            bet = (S[bs] - T[kind - 1]) / den;
            omb = 1.0 - bet;

            for ( k = 1; k < rem; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT find )
                    {
                        alf = (S[bs] - T[i]) / (S[as] - T[i]);
                        oma = 1.0 - alf;
                        h[i] = alf * h[i] + oma * h[i - 1];
                    }

                    if( j GE lbz )
                    {
                        if( j - k LE kind - pq + rem )
                        {
                            gam = (S[bs] - T[j - k]) / den;
                            omg = 1.0 - gam;
                            fg[l] = gam * fg[l] + omg * fg[l + 1];
                        }
                        else
                        {
                            fg[l] = bet * fg[l] + omb * fg[l + 1];
                        }
                    }
                    i++;
                    j--;
                    l--;
                }
                first--;
                last++;
            }
        }

        /* Load knots and control values */

        if( ar NEQ p AND as NEQ q )
        {
            for ( i = 0; i < pq - rem; i++ )
            {
                T[kind] = S[as];
                kind++;
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            h[find] = fg[i];
            find++;
        }

        /* Initialize for next pass through */

        if( br LT mr AND bs LT ms )
        {
            for ( i = 0; i < rr; i++ )
                fb[i] = fn[i];

            for ( i = rr; i <= p; i++ )
                fb[i] = f[br - p + i];

            for ( j = 0; j < rs; j++ )
                gb[j] = gn[j];

            for ( j = rs; j <= q; j++ )
                gb[j] = g[bs - q + j];

            ar = br;
            br++;
            rem = pq - mi;
            as = bs;
            bs++;
        }
        else
        {
            /* Get end knots */

            for ( i = 0; i <= pq; i++ )
            {
                T[kind] = S[bs];
                kind++;
            }
        }
    }

    /* Compact output function */

    if( find LT nh + 1 )
    {
        N_CFuncSetSizeIndices( cfnH, find - 1, pq, kind - 1 );

        error = N_CrvFuncCompact( cfnH, SH );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncMultiplyCrvFunc */

/**********************************************************************/
/* N_CrvFuncMultiplyCrv: Compute the product of a function and a curve            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a B-spline
     function and a NURBS curve. The knot vectors of both entities  are 
     rescaled to the unit span  [0,1]. This does not change them either 
     parametrically or geometrically. A typical calling example is:

       NL_CFUN    cfnF;
       NL_CURVE   curG, curH;
       NL_STACKS  SH;
       ...
       (define cfnF and curG);
       ...
       N_CrvInitArrays(&curH);
       N_CrvFuncMultiplyCrv(&cfnF,&curG,&curH,&SH);

     If memory is  available, curH  is not initialized and  the routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curH's  
     knot vector and control  point objects. THE STORAGE  OF THE OUTPUT 
     NL_CURVE IS COMPACTED, I.E. THE MEMORY PASSED IN IS DESTROYED.


   ACCESS:
   
     cfnF , in/out ,  Function (ITS KNOT NL_VECTOR IS RESCALED)
     curG , in/out ,  Curve (ITS KNOT NL_VECTOR IS RESCALED)
     curH , output ,  Product of function and curve
     SH   , input  ,  curH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncMultiplyCrv( NL_CFUN *cfnF, NL_CURVE *curG, NL_CURVE *curH, NL_STACKS *SH )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncMultiplyCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, nf, mr, ng, ms, nh, mt, mlr, mls, mir, mis, mi, rr, rs, s, ar, as, br, bs, mxr, mxs, ixr, ixs, t, kind, cind, first, last, rem, save, nsr, nss, lbz, rbz;

    NL_DEGREE p, q, pq;

    NL_REAL *R, *S, *T, *XR, *XS, *f, *fb, *fn, *alfr, *omar, *alfs, *omas, alf, oma, bet, omb, gam, omg, num, den;

    NL_CPOINT *Gw, *Hw, *GBw, *GNw, *FGw;

    NL_KNOTVECTOR *knr, *kns, *kxr, *kxs;

    NL_CFUN cfnFR;

    NL_CURVE curGR;

    NL_RMATRIX pm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get new degree */

    N_CFuncGetDegree( cfnF, &p );
    N_CrvGetDegree( curG, &q );

    pq = p + q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Handle special case p=0 or q=0 */

    if( p EQ 0 OR q EQ 0 )
    {
        N_CFuncGetArraySizes( cfnF, &i, &j );
        N_CrvGetArraySizes( curG, &k, &l );

        if( i NEQ p OR k NEQ q )
            NL_ERROR( NL_INP_ERR );

        error = N_CrvSizeArrays( curH, pq, pq, pq + pq + 1, rname, SH );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnF, &f, &R );
        N_CrvGetCPtsAndKnots( curG, &Gw, &S );
        N_CrvGetCPtsAndKnots( curH, &Hw, &T );

        if( p EQ 0 )
        {
            for ( j = 0; j <= q; j++ )
            {
                N_ScaleCPtXYZ( f[0], Gw[j], &Hw[j] );
                T[j] = S[j];
                T[q + j + 1] = S[q + j + 1];
            }
        }
        else
        {
            for ( i = 0; i <= p; i++ )
            {
                N_ScaleCPtXYZ( f[i], Gw[0], &Hw[i] );
                T[i] = R[i];
                T[p + i + 1] = R[p + i + 1];
            }
        }

        NL_OUT;
    }

    /* Merge knots */

    N_CFuncGetKnotVector( cfnF, &knr );
    N_CrvGetKnotVector( curG, &kns );

    N_BasisGetSpanCount( knr, p, &nsr );
    N_BasisGetSpanCount( kns, q, &nss );

    kxr = N_AllocKnotVectorAndArray( nss, &SL );

    if( kxr EQ NULL )
        NL_QUIT;

    kxs = N_AllocKnotVectorAndArray( nsr, &SL );

    if( kxs EQ NULL )
        NL_QUIT;

    N_MakeKnotsCompatible( knr, kns, p, q, kxr, kxs );

    /* Refine entities */

    N_KnotVectorGetKnots( kxr, &mxr, &XR );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( mxr GE 0 )
    {
        N_CFuncInitArrays( &cfnFR );
        error = N_CrvFuncRefine( cfnF, kxr, &cfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CFuncGetData( &cfnFR, &nf, &f, &p, &mr, &R );
        N_CFuncGetKnotVector( &cfnFR, &knr );
        N_BasisGetSpanCount( knr, p, &nsr );
    }
    else
    {
        N_CFuncGetData( cfnF, &nf, &f, &p, &mr, &R );
    }

    if( mxs GE 0 )
    {
        N_CrvInitArrays( &curGR );
        error = N_CrvRefine( curG, kxs, &curGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsDegreeAndKnots( &curGR, &ng, &Gw, &q, &ms, &S );
        N_CrvGetKnotVector( &curGR, &kns );
        N_BasisGetSpanCount( kns, q, &nss );
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( curG, &ng, &Gw, &q, &ms, &S );
    }

    if( nsr NEQ nss )
        NL_ERROR( NL_KNT_ERR );

    /* See if memory is needed */

    nh = nsr * pq;
    mt = nh + pq + 1;

    error = N_CrvSizeArrays( curH, nh, pq, mt, rname, SH );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curH, &Hw, &T );

    /* Get local memory */

    fb = N_AllocReal1dArray( p, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    fn = N_AllocReal1dArray( p, &SL );

    if( fn EQ NULL )
        NL_QUIT;

    GBw = N_AllocCPt1dArray( q, &SL );

    if( GBw EQ NULL )
        NL_QUIT;

    GNw = N_AllocCPt1dArray( q, &SL );

    if( GNw EQ NULL )
        NL_QUIT;

    FGw = N_AllocCPt1dArray( pq, &SL );

    if( FGw EQ NULL )
        NL_QUIT;

    alfr = N_AllocReal1dArray( p, &SL );

    if( alfr EQ NULL )
        NL_QUIT;

    omar = N_AllocReal1dArray( p, &SL );

    if( omar EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( q, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( q, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    ar = p;
    br = p + 1;
    as = q;
    bs = q + 1;
    rr = -1;
    rs = -1;
    ixr = 0;
    ixs = 0;

    cind = 1;
    kind = pq + 1;
    rem = -1;

    for ( i = 0; i <= pq; i++ )
        T[i] = S[as];

    for ( i = 0; i <= p; i++ )
        fb[i] = f[i];

    for ( j = 0; j <= q; j++ )
        N_CopyCPt( Gw[j], &GBw[j] );

    N_ScaleCPtXYZ( f[0], Gw[0], &Hw[0] );

    N_InitRealMatrix( &pm );
    error = N_BezFuncMultiplyBezCrvMatrix( p, q, &pm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /***************************************************************/
    /* Loop through the knot vectors and do the following:         */
    /*   (1) Extract the i-th Bezier entities.                     */
    /*   (2) Compute their product.                                */
    /*   (3) Remove the knot(s) between the i-th and the (i-1)-th  */
    /*       segments.                                             */
    /***************************************************************/

    while( br LT mr AND bs LT ms )
    {
        /* Get multiplicities of the knots */

        i = br;

        while( br LT mr AND R[br]EQ R[br + 1] )
            br++;
        mlr = br - i + 1;
        mir = mlr;

        j = bs;

        while( bs LT ms AND S[bs]EQ S[bs + 1] )
            bs++;
        mls = bs - j + 1;
        mis = mls;

        /* Check multiplicities of internal knots */

        if( br LT mr AND bs LT ms )
        {
            if( mlr GT p OR mls GT q )
                NL_ERROR( NL_KNT_ERR );
        }

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mir EQ 1 AND ixr LE mxr )
        {
            if( fabs( R[br] - XR[ixr] )LT NL_PTOL )
            {
                mir = 0;
                ixr++;
            }
        }

        if( mis EQ 1 AND ixs LE mxs )
        {
            if( fabs( S[bs] - XS[ixs] )LT NL_PTOL )
            {
                mis = 0;
                ixs++;
            }
        }

        if( mis EQ 0 )
            mi = q + mir;

        else if( mir EQ 0 )
            mi = p + mis;

        else
            mi = NL_MAX( q + mir, p + mis );

        /* Insert knot to get Bezier entities */

        rr = p - mlr;
        rs = q - mls;
        t = pq - mi;

        if( rem GT 0 )
            lbz = (rem + 2) / 2;
        else
            lbz = 1;

        if( t GT 0 )
            rbz = pq - (t + 1) / 2;
        else
            rbz = pq;

        if( rr GT 0 )
        {
            num = R[br] - R[ar];

            for ( k = p; k > mlr; k-- )
            {
                alfr[k - mlr - 1] = num / (R[ar + k] - R[ar]);
                omar[k - mlr - 1] = 1.0 - alfr[k - mlr - 1];
            }

            for ( j = 1; j <= rr; j++ )
            {
                save = rr - j;
                s = mlr + j;

                for ( k = p; k >= s; k-- )
                {
                    fb[k] = alfr[k - s] * fb[k] + omar[k - s] * fb[k - 1];
                }
                fn[save] = fb[p];
            }
        }

        if( rs GT 0 )
        {
            num = S[bs] - S[as];

            for ( k = q; k > mls; k-- )
            {
                alfs[k - mls - 1] = num / (S[as + k] - S[as]);
                omas[k - mls - 1] = 1.0 - alfs[k - mls - 1];
            }

            for ( j = 1; j <= rs; j++ )
            {
                save = rs - j;
                s = mls + j;

                for ( k = q; k >= s; k-- )
                {
                    N_Combine2CPts( alfs[k - s], GBw[k], omas[k - s], GBw[k - 1], &GBw[k] );
                }
                N_CopyCPt( GBw[q], &GNw[save] );
            }
        }

        /* Compute product of Bezier entities */

        error = N_BezFuncMultiplyBezCrv( fb, p, GBw, q, &pm, lbz, pq, FGw );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot R[ar] = S[as] */

        if( rem GT 1 )
        {
            first = kind - 2;
            last = kind;
            den = S[bs] - S[as];
            bet = (S[bs] - T[kind - 1]) / den;
            omb = 1.0 - bet;

            for ( k = 1; k < rem; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        alf = (S[bs] - T[i]) / (S[as] - T[i]);
                        oma = 1.0 - alf;
                        N_Combine2CPts( alf, Hw[i], oma, Hw[i - 1], &Hw[i] );
                    }

                    if( j GE lbz )
                    {
                        if( j - k LE kind - pq + rem )
                        {
                            gam = (S[bs] - T[j - k]) / den;
                            omg = 1.0 - gam;
                            N_Combine2CPts( gam, FGw[l], omg, FGw[l + 1], &FGw[l] );
                        }
                        else
                        {
                            N_Combine2CPts( bet, FGw[l], omb, FGw[l + 1], &FGw[l] );
                        }
                    }
                    i++;
                    j--;
                    l--;
                }
                first--;
                last++;
            }
        }

        /* Load knots and control points */

        if( ar NEQ p AND as NEQ q )
        {
            for ( i = 0; i < pq - rem; i++ )
            {
                T[kind] = S[as];
                kind++;
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            N_CopyCPt( FGw[i], &Hw[cind] );
            cind++;
        }

        /* Initialize for next pass through */

        if( br LT mr AND bs LT ms )
        {
            for ( i = 0; i < rr; i++ )
                fb[i] = fn[i];

            for ( i = rr; i <= p; i++ )
                fb[i] = f[br - p + i];

            for ( j = 0; j < rs; j++ )
                N_CopyCPt( GNw[j], &GBw[j] );

            for ( j = rs; j <= q; j++ )
                N_CopyCPt( Gw[bs - q + j], &GBw[j] );

            ar = br;
            br++;
            rem = pq - mi;
            as = bs;
            bs++;
        }
        else
        {
            /* Get end knots */

            for ( i = 0; i <= pq; i++ )
            {
                T[kind] = S[bs];
                kind++;
            }
        }
    }

    /* Compact output curve */

    if( cind LT nh + 1 )
    {
        N_CrvSetSizeIndices( curH, cind - 1, pq, kind - 1 );

        error = N_CrvCompress( curH, SH );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncMultiplyCrv */

#if NLIB_UNUSED

/**********************************************************************/
/* N_CRVFUNCSUMDIFFCRVFUNC: Sum/difference of two curve functions                    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic operators  routine computes the sum/difference  of 
     two curve functions. A typical calling example is:

       NL_CFUN    cfnP, cfnQ, cfnR;
       NL_STACKS  SG;
       ...
       (define cfnP and cfnQ);
       ...
       N_CFuncInitArrays(&cfnR);
       N_CrvFuncSumDiffCrvFunc(&cfnP,&cfnQ,NL_PLUS,NL_YES,&cfnR,&SG);

     If memory is available, cfnR is not  initialized and  the routine 
     assumes that memory  allocation has been done. However, it checks 
     for the proper amount by looking at the highest indexes in cfnR's
     knot vector and control value objects. IF THE FUNCTIONS ARE KNOWN
     TO BE COMPATIBLE, SET THE cmp NL_FLAG "NL_YES" WHICH WILL SKIP A LOT OF
     UNNECESSARY COMPUTATIONS. THIS IS USEFUL IF SYMBOLIC COMPUTATIONS 
     ARE TO BE PERFORMED ON THE  INDIVIDUAL COORDINATE  FUNCTIONS THAT 
     ARE  KNOWN TO BE  COMPATIBLE. BECAUSE OF  COMPATIBILITY, THE KNOT
     VECTORS MUST BE  SCALED TO A  COMMON NL_INTERVAL  ([0,1]). THIS DOES 
     NOT CHANGE THE FUNCTIONS EITHER PARAMETRICALLY OR GEOMETRICALLY.


   ACCESS:
   
     cfnP , in/out ,  Curve function
     cfnQ , in/out ,  Curve function
     opr  , input  ,  Operator flag:
                        NL_PLUS : sum
                        NL_MINUS: difference
     cmp  , input  ,  Flag:
                        NL_YES: functions are compatible
                        NL_NO : make functions compatible
     cfnR , output ,  Sum/difference of cfnP and cfnQ
     SG   , input  ,  cfnR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncSumDiffCrvFunc( NL_CFUN *cfnP, NL_CFUN *cfnQ, NL_FLAG opr, NL_FLAG cmp, NL_CFUN *cfnR, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncSumDiffCrvFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m, nsp, nsq, mxp, mxq;

    NL_DEGREE p, q;

    NL_REAL *fp, *fq, *fr, *UP, *UQ, *UR, *A;

    NL_CFUN cfnPW, cfnQW;

    NL_KNOTVECTOR *knp, *knq, *kxp, *kxq;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute sum/difference */

    if( cmp EQ NL_YES )
    {
        /* Functions are compatible -> get memory for output function */

        N_CFuncGetData( cfnP, &n, &fp, &p, &m, &UP );
        N_CFuncGetData( cfnQ, &n, &fq, &q, &m, &UQ );

        error = N_CFuncSizeArrays( cfnR, n, p, m, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnR, &fr, &UR );
    }
    else
    {
        /* Make functions compatible using auxiliary functions */

        N_CFuncInitArrays( &cfnPW );
        error = N_CrvFuncCopy( cfnP, &cfnPW, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CFuncInitArrays( &cfnQW );
        error = N_CrvFuncCopy( cfnQ, &cfnQW, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get local notation */

        N_CFuncGetData( &cfnPW, &n, &fp, &p, &m, &UP );
        N_CFuncGetData( &cfnQW, &n, &fq, &q, &m, &UQ );
        N_CFuncGetKnotVector( &cfnPW, &knp );
        N_CFuncGetKnotVector( &cfnQW, &knq );

        /* Degree elevate */

        if( p NEQ q )
        {
            if( p LT q )
            {
                error = N_CrvFuncDegreeElevate( &cfnPW, q - p, &cfnPW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CFuncGetData( &cfnPW, &n, &fp, &p, &m, &UP );
                N_CFuncGetKnotVector( &cfnPW, &knp );
            }
            else
            {
                error = N_CrvFuncDegreeElevate( &cfnQW, p - q, &cfnQW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CFuncGetData( &cfnQW, &n, &fq, &q, &m, &UQ );
                N_CFuncGetKnotVector( &cfnQW, &knq );
            }
        }

        /* Refine functions */

        N_BasisGetSpanCount( knp, p, &nsp );
        N_BasisGetSpanCount( knq, q, &nsq );

        kxp = N_AllocKnotVectorAndArray( nsq * q, &SL );

        if( kxp EQ NULL )
            NL_QUIT;

        kxq = N_AllocKnotVectorAndArray( nsp * p, &SL );

        if( kxq EQ NULL )
            NL_QUIT;

        N_GetCompatibleKnotArrayMult( knp, knq, p, kxp, kxq );

        N_KnotVectorGetKnots( kxp, &mxp, &A );
        N_KnotVectorGetKnots( kxq, &mxq, &A );

        if( mxp GE 0 )
        {
            error = N_CrvFuncRefine( &cfnPW, kxp, &cfnPW, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CFuncGetData( &cfnPW, &n, &fp, &p, &m, &UP );
        }

        if( mxq GE 0 )
        {
            error = N_CrvFuncRefine( &cfnQW, kxq, &cfnQW, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CFuncGetData( &cfnQW, &n, &fq, &q, &m, &UQ );
        }

        /* Check memory of output function */

        error = N_CFuncSizeArrays( cfnR, n, p, m, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnR, &fr, &UR );
    }

    /* Compute output function */

    switch( opr )
    {
        case NL_PLUS:
            for ( i = 0; i <= n; i++ )
                fr[i] = fp[i] + fq[i];
            break;

        case NL_MINUS:
            for ( i = 0; i <= n; i++ )
                fr[i] = fp[i] - fq[i];
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= m; i++ )
        UR[i] = UP[i];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncSumDiffCrvFunc */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_CRVFUNCDERIV: Derivative function of a curve function                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators  routine computes the  derivative function 
     of a curve  function. The maximum  derivative allowed  is equal to 
     the degree. A typical calling example is:

       NL_CFUN    cfn, der;
       NL_INDEX   d;
       NL_STACKS  SG;
       ...
       (define cfn; get highest derivative index d);
       ...
       N_CFuncInitArrays(&der);
       N_CrvFuncDeriv(&cfn,d,&der,&SG);


   ACCESS:
   
     cfn  , input  ,  Curve function
     d    , input  ,  Highest derivative required (MUST BE LESS THAN OR
                      EQUAL TO THE NL_FUNCTION NL_DEGREE)
     der  , output ,  Derivative function
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncDeriv( NL_CFUN *cfn, NL_INDEX d, NL_CFUN *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncDeriv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, nd, md;

    NL_DEGREE p, pd;

    NL_REAL *fu, *fd, *ft, *U, *V, alf;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CFuncGetData( cfn, &n, &fu, &p, &m, &U );

    if( d LT 0 OR d GT p )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    nd = n - d;
    pd = (NL_DEGREE)( p - d );
    md = nd + pd + 1;

    error = N_CFuncSizeArrays( der, nd, pd, md, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( der, &fd, &V );

    /* Initialize control values */

    ft = N_AllocReal1dArray( n, &SL );

    if( ft EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
        ft[i] = fu[i];

    /* Compute control values of derivative function */

    for ( k = 1; k <= d; k++ )
    {
        for ( i = 0; i <= n - k; i++ )
        {
            if( U[i + p + 1]EQ U[i + k] )
                NL_ERROR( NL_DER_ERR );

            if( N_FloatOpIsBad( ( (NL_REAL)p - (NL_REAL)k + 1.0 ), U[i + p + 1] - U[i + k], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            alf = ((NL_REAL)p - (NL_REAL)k + 1.0) / (U[i + p + 1] - U[i + k]);
            ft[i] = alf * (ft[i + 1] - ft[i]);
        }
    }

    for ( i = 0; i <= n - d; i++ )
        fd[i] = ft[i];

    /* Compute the knot vector */

    j = -1;

    for ( i = 0; i <= p - d; i++ )
        V[++j] = U[0];

    for ( i = p + 1; i <= n; i++ )
        V[++j] = U[i];

    for ( i = 0; i <= p - d; i++ )
        V[++j] = U[m];

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncDeriv */

#if NLIB_UNUSED

/**********************************************************************/
/* N_CRVFUNCDERIVKNOT: Derivative of function with respect to a knot            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the  derivative of a 
     function  with respect  to a knot. A  typical calling example is:

       CFN     cfn, der;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cfn);
       ...
       N_CFuncInitArrays(&der);
       N_CrvFuncDerivKnot(&cfn,k,NL_LEFT,&der,&SG);

     Since NLIB  does not allow knot  multiplicities  greater than the
     degree, the  function cannot be  differentiated with respect to a 
     knot with  multiplicity equal to the  degree. The  error  NL_KML_ERR 
     is returned if such a knot is found.


   ACCESS:
   
     cfn  , input  ,  Function
     k    , input  ,  Index of  knot, i.e. the derivative with respect 
                      to u_k is computed
     flg  , input  ,  Flag:
                        NL_LEFT : left derivative. NL_INDEX  k  MUST SATISFY
                               u_(k) != u_(k-1)
                        NL_RIGHT: right derivative. NL_INDEX k  MUST SATISFY
                               u_(k) != u_(k+1) 
     der  , output ,  Derivative function
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncDerivKnot( NL_CFUN *cfn, NL_INDEX k, NL_FLAG flg, NL_CFUN *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncDerivKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, mlt;

    NL_DEGREE p;

    NL_REAL *UF, *UD, *fu, *fd;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CFuncGetData( cfn, &n, &fu, &p, &m, &UF );

    if( k LE p OR k GE n + 1 )
        NL_ERROR( NL_INP_ERR );

    switch( flg )
    {
        case NL_LEFT:
            if( UF[k]EQ UF[k - 1] )
                NL_ERROR( NL_INP_ERR );

            i = k;

            while( UF[i]EQ UF[i + 1] )
                i++;
            mlt = i - k + 1;

            if( mlt GE p )
                NL_ERROR( NL_KML_ERR );
            break;

        case NL_RIGHT:
            if( UF[k]EQ UF[k + 1] )
                NL_ERROR( NL_INP_ERR );

            i = k;

            while( UF[i]EQ UF[i - 1] )
                i--;
            mlt = k - i + 1;

            if( mlt GE p )
                NL_ERROR( NL_KML_ERR );
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_CFuncSizeArrays( der, n + 1, p, m + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( der, &fd, &UD );

    /* Compute the knot vector */

    switch( flg )
    {
        case NL_LEFT:

            j = -1;

            for ( i = 0; i <= k + mlt - 1; i++ )
                UD[++j] = UF[i];
            UD[++j] = UF[k];

            for ( i = k + mlt; i <= m; i++ )
                UD[++j] = UF[i];
            break;

        case NL_RIGHT:

            j = -1;

            for ( i = 0; i <= k; i++ )
                UD[++j] = UF[i];
            UD[++j] = UF[k];

            for ( i = k + 1; i <= m; i++ )
                UD[++j] = UF[i];
            break;
    }

    /* Compute control values */

    for ( i = 0; i <= k - p - 1; i++ )
        fd[i] = 0.0;

    for ( i = k - p; i <= k; i++ )
        fd[i] = (fu[i - 1] - fu[i]) / (UF[i + p] - UF[i]);

    for ( i = k + 1; i <= n + 1; i++ )
        fd[i] = 0.0;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncDerivKnot */

/**********************************************************************/
/* N_CRVFUNCHIGHERDERIVKNOT: Higher derivatives of function wrt to a knot             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes  higher derivatives of a 
     function with respect to a knot. A typical calling example is:

       NL_CFUN    cfn, der;
       NL_INDEX   k, r;
       NL_STACKS  SG;
       ...
       (define cfn);
       ...
       N_CFuncInitArrays(&der);
       N_CrvFuncHigherDerivKnot(&cfn,k,NL_LEFT,r,&der,&SG);

     Since NLIB  does not allow knot  multiplicities  greater than the
     degree, the  function cannot be  differentiated with respect to a 
     knot with multiplicity equal to the degree. The error  NL_KML_ERR is 
     returned if such a knot is found.


   ACCESS:
   
     cfn  , input  ,  Function
     k    , input  ,  Index of  knot, i.e. the derivative with respect 
                      to u_k is computed
     flg  , input  ,  Flag:
                        NL_LEFT : left derivative. NL_INDEX  k  MUST SATISFY
                               u_(k) != u_(k-1)
                        NL_RIGHT: right derivative. NL_INDEX k  MUST SATISFY
                               u_(k) != u_(k+1) 
     r    , input  ,  Highest derivative required (1 < r < degree-1)
     der  , output ,  Derivative function
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncHigherDerivKnot( NL_CFUN *cfn, NL_INDEX k, NL_FLAG flg, NL_INDEX r, NL_CFUN *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncHigherDerivKnot");

    NL_FLAG tgl, error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_CFUN cfnA, cfnB;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CFuncGetArraySizes( cfn, &n, &m );
    N_CFuncGetDegree( cfn, &p );

    if( r LT 1 OR r GT p - 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LE p OR k GE n + 1 )
        NL_ERROR( NL_INP_ERR );

    /* Allocate memory */

    error = N_CFuncSizeArrays( der, n + r, p, m + r, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( r GT 1 )
    {
        error = N_AllocCFuncArrays( &cfnA, n + r, p, m + r, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( r GT 2 )
    {
        error = N_AllocCFuncArrays( &cfnB, n + r, p, m + r, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute derivatives */

    if( r EQ 1 )
    {
        error = N_CrvFuncDerivKnot( cfn, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    error = N_CrvFuncDerivKnot( cfn, k, flg, &cfnA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    tgl = NL_YES;

    for ( i = 2; i <= r - 1; i++ )
    {
        if( flg EQ NL_RIGHT )
            k++;

        if( tgl EQ NL_YES )
        {
            N_CFuncSetSizeIndices( &cfnB, n + r, p, m + r );

            error = N_CrvFuncDerivKnot( &cfnA, k, flg, &cfnB, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            tgl = NL_NO;
        }
        else
        {
            N_CFuncSetSizeIndices( &cfnA, n + r, p, m + r );

            error = N_CrvFuncDerivKnot( &cfnB, k, flg, &cfnA, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            tgl = NL_YES;
        }
    }

    if( flg EQ NL_RIGHT )
        k++;

    if( tgl EQ NL_YES )
    {
        error = N_CrvFuncDerivKnot( &cfnA, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_CrvFuncDerivKnot( &cfnB, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncHigherDerivKnot */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_CrvFuncMultiplyCrv4D: Compute the product of a function and a curve in 4-D     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a B-spline
     function and a NURBS curve. The knot vectors of both entities  are 
     rescaled to the unit span  [0,1]. This does not change them either 
     parametrically or geometrically. This version computes the product
     in 4-D. A typical calling example is:

       NL_CFUN    cfnF;
       NL_CURVE   curG, curH;
       NL_STACKS  SH;
       ...
       (define cfnF and curG);
       ...
       N_CrvInitArrays(&curH);
       N_CrvFuncMultiplyCrv4d(&cfnF,&curG,&curH,&SH);

     If memory is  available, curH  is not initialized and  the routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curH's  
     knot vector and control  point objects. THE STORAGE  OF THE OUTPUT 
     NL_CURVE IS COMPACTED, I.E. THE MEMORY PASSED IN IS DESTROYED.


   ACCESS:
   
     cfnF , in/out ,  Function (ITS KNOT NL_VECTOR IS RESCALED)
     curG , in/out ,  Curve (ITS KNOT NL_VECTOR IS RESCALED)
     curH , output ,  Product of function and curve
     SH   , input  ,  curH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncMultiplyCrv4d( NL_CFUN *cfnF, NL_CURVE *curG, NL_CURVE *curH, NL_STACKS *SH )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncMultiplyCrv4D");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, nf, mr, ng, ms, nh, mt, mlr, mls, mir, mis, mi, rr, rs, s, ar, as, br, bs, mxr, mxs, ixr, ixs, t, kind, cind, first, last, rem, save, nsr, nss, lbz, rbz;

    NL_DEGREE p, q, pq;

    NL_REAL *R, *S, *T, *XR, *XS, *f, *fb, *fn, *alfr, *omar, *alfs, *omas, alf, oma, bet, omb, gam, omg, num, den;

    NL_CPOINT *Gw, *Hw, *GBw, *GNw, *FGw;

    NL_KNOTVECTOR *knr, *kns, *kxr, *kxs;

    NL_CFUN cfnFR;

    NL_CURVE curGR;

    NL_RMATRIX pm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get new degree */

    N_CFuncGetDegree( cfnF, &p );
    N_CrvGetDegree( curG, &q );

    pq = p + q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Handle special case p=0 or q=0 */

    if( p EQ 0 OR q EQ 0 )
    {
        N_CFuncGetArraySizes( cfnF, &i, &j );
        N_CrvGetArraySizes( curG, &k, &l );

        if( i NEQ p OR k NEQ q )
            NL_ERROR( NL_INP_ERR );

        error = N_CrvSizeArrays( curH, pq, pq, pq + pq + 1, rname, SH );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( cfnF, &f, &R );
        N_CrvGetCPtsAndKnots( curG, &Gw, &S );
        N_CrvGetCPtsAndKnots( curH, &Hw, &T );

        if( p EQ 0 )
        {
            for ( j = 0; j <= q; j++ )
            {
                N_ScaleCPt( f[0], Gw[j], &Hw[j] );
                T[j] = S[j];
                T[q + j + 1] = S[q + j + 1];
            }
        }
        else
        {
            for ( i = 0; i <= p; i++ )
            {
                N_ScaleCPt( f[i], Gw[0], &Hw[i] );
                T[i] = R[i];
                T[p + i + 1] = R[p + i + 1];
            }
        }

        NL_OUT;
    }

    /* Merge knots */

    N_CFuncGetKnotVector( cfnF, &knr );
    N_CrvGetKnotVector( curG, &kns );

    N_BasisGetSpanCount( knr, p, &nsr );
    N_BasisGetSpanCount( kns, q, &nss );

    kxr = N_AllocKnotVectorAndArray( nss, &SL );

    if( kxr EQ NULL )
        NL_QUIT;

    kxs = N_AllocKnotVectorAndArray( nsr, &SL );

    if( kxs EQ NULL )
        NL_QUIT;

    N_MakeKnotsCompatible( knr, kns, p, q, kxr, kxs );

    /* Refine entities */

    N_KnotVectorGetKnots( kxr, &mxr, &XR );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( mxr GE 0 )
    {
        N_CFuncInitArrays( &cfnFR );
        error = N_CrvFuncRefine( cfnF, kxr, &cfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CFuncGetData( &cfnFR, &nf, &f, &p, &mr, &R );
        N_CFuncGetKnotVector( &cfnFR, &knr );
        N_BasisGetSpanCount( knr, p, &nsr );
    }
    else
    {
        N_CFuncGetData( cfnF, &nf, &f, &p, &mr, &R );
    }

    if( mxs GE 0 )
    {
        N_CrvInitArrays( &curGR );
        error = N_CrvRefine( curG, kxs, &curGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsDegreeAndKnots( &curGR, &ng, &Gw, &q, &ms, &S );
        N_CrvGetKnotVector( &curGR, &kns );
        N_BasisGetSpanCount( kns, q, &nss );
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( curG, &ng, &Gw, &q, &ms, &S );
    }

    if( nsr NEQ nss )
        NL_ERROR( NL_KNT_ERR );

    /* See if memory is needed */

    nh = nsr * pq;
    mt = nh + pq + 1;

    error = N_CrvSizeArrays( curH, nh, pq, mt, rname, SH );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curH, &Hw, &T );

    /* Get local memory */

    fb = N_AllocReal1dArray( p, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    fn = N_AllocReal1dArray( p, &SL );

    if( fn EQ NULL )
        NL_QUIT;

    GBw = N_AllocCPt1dArray( q, &SL );

    if( GBw EQ NULL )
        NL_QUIT;

    GNw = N_AllocCPt1dArray( q, &SL );

    if( GNw EQ NULL )
        NL_QUIT;

    FGw = N_AllocCPt1dArray( pq, &SL );

    if( FGw EQ NULL )
        NL_QUIT;

    alfr = N_AllocReal1dArray( p, &SL );

    if( alfr EQ NULL )
        NL_QUIT;

    omar = N_AllocReal1dArray( p, &SL );

    if( omar EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( q, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( q, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Initialize */

    ar = p;
    br = p + 1;
    as = q;
    bs = q + 1;
    rr = -1;
    rs = -1;
    ixr = 0;
    ixs = 0;

    cind = 1;
    kind = pq + 1;
    rem = -1;

    for ( i = 0; i <= pq; i++ )
        T[i] = S[as];

    for ( i = 0; i <= p; i++ )
        fb[i] = f[i];

    for ( j = 0; j <= q; j++ )
        N_CopyCPt( Gw[j], &GBw[j] );

    N_ScaleCPt( f[0], Gw[0], &Hw[0] );

    N_InitRealMatrix( &pm );
    error = N_BezFuncMultiplyBezCrvMatrix( p, q, &pm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /***************************************************************/
    /* Loop through the knot vectors and do the following:         */
    /*   (1) Extract the i-th Bezier entities.                     */
    /*   (2) Compute their product.                                */
    /*   (3) Remove the knot(s) between the i-th and the (i-1)-th  */
    /*       segments.                                             */
    /***************************************************************/

    while( br LT mr AND bs LT ms )
    {
        /* Get multiplicities of the knots */

        i = br;

        while( br LT mr AND R[br]EQ R[br + 1] )
            br++;
        mlr = br - i + 1;
        mir = mlr;

        j = bs;

        while( bs LT ms AND S[bs]EQ S[bs + 1] )
            bs++;
        mls = bs - j + 1;
        mis = mls;

        /* Check multiplicities of internal knots */

        if( br LT mr AND bs LT ms )
        {
            if( mlr GT p OR mls GT q )
                NL_ERROR( NL_KNT_ERR );
        }

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mir EQ 1 AND ixr LE mxr )
        {
            if( fabs( R[br] - XR[ixr] )LT NL_PTOL )
            {
                mir = 0;
                ixr++;
            }
        }

        if( mis EQ 1 AND ixs LE mxs )
        {
            if( fabs( S[bs] - XS[ixs] )LT NL_PTOL )
            {
                mis = 0;
                ixs++;
            }
        }

        if( mis EQ 0 )
            mi = q + mir;

        else if( mir EQ 0 )
            mi = p + mis;

        else
            mi = NL_MAX( q + mir, p + mis );

        /* Insert knot to get Bezier entities */

        rr = p - mlr;
        rs = q - mls;
        t = pq - mi;

        if( rem GT 0 )
            lbz = (rem + 2) / 2;
        else
            lbz = 1;

        if( t GT 0 )
            rbz = pq - (t + 1) / 2;
        else
            rbz = pq;

        if( rr GT 0 )
        {
            num = R[br] - R[ar];

            for ( k = p; k > mlr; k-- )
            {
                alfr[k - mlr - 1] = num / (R[ar + k] - R[ar]);
                omar[k - mlr - 1] = 1.0 - alfr[k - mlr - 1];
            }

            for ( j = 1; j <= rr; j++ )
            {
                save = rr - j;
                s = mlr + j;

                for ( k = p; k >= s; k-- )
                {
                    fb[k] = alfr[k - s] * fb[k] + omar[k - s] * fb[k - 1];
                }
                fn[save] = fb[p];
            }
        }

        if( rs GT 0 )
        {
            num = S[bs] - S[as];

            for ( k = q; k > mls; k-- )
            {
                alfs[k - mls - 1] = num / (S[as + k] - S[as]);
                omas[k - mls - 1] = 1.0 - alfs[k - mls - 1];
            }

            for ( j = 1; j <= rs; j++ )
            {
                save = rs - j;
                s = mls + j;

                for ( k = q; k >= s; k-- )
                {
                    N_Combine2CPts( alfs[k - s], GBw[k], omas[k - s], GBw[k - 1], &GBw[k] );
                }
                N_CopyCPt( GBw[q], &GNw[save] );
            }
        }

        /* Compute product of Bezier entities */

        error = N_BezFuncMultiplyBezCrv4D( fb, p, GBw, q, &pm, lbz, pq, FGw );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot R[ar] = S[as] */

        if( rem GT 1 )
        {
            first = kind - 2;
            last = kind;
            den = S[bs] - S[as];
            bet = (S[bs] - T[kind - 1]) / den;
            omb = 1.0 - bet;

            for ( k = 1; k < rem; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        alf = (S[bs] - T[i]) / (S[as] - T[i]);
                        oma = 1.0 - alf;
                        N_Combine2CPts( alf, Hw[i], oma, Hw[i - 1], &Hw[i] );
                    }

                    if( j GE lbz )
                    {
                        if( j - k LE kind - pq + rem )
                        {
                            gam = (S[bs] - T[j - k]) / den;
                            omg = 1.0 - gam;
                            N_Combine2CPts( gam, FGw[l], omg, FGw[l + 1], &FGw[l] );
                        }
                        else
                        {
                            N_Combine2CPts( bet, FGw[l], omb, FGw[l + 1], &FGw[l] );
                        }
                    }
                    i++;
                    j--;
                    l--;
                }
                first--;
                last++;
            }
        }

        /* Load knots and control points */

        if( ar NEQ p AND as NEQ q )
        {
            for ( i = 0; i < pq - rem; i++ )
            {
                T[kind] = S[as];
                kind++;
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            N_CopyCPt( FGw[i], &Hw[cind] );
            cind++;
        }

        /* Initialize for next pass through */

        if( br LT mr AND bs LT ms )
        {
            for ( i = 0; i < rr; i++ )
                fb[i] = fn[i];

            for ( i = rr; i <= p; i++ )
                fb[i] = f[br - p + i];

            for ( j = 0; j < rs; j++ )
                N_CopyCPt( GNw[j], &GBw[j] );

            for ( j = rs; j <= q; j++ )
                N_CopyCPt( Gw[bs - q + j], &GBw[j] );

            ar = br;
            br++;
            rem = pq - mi;
            as = bs;
            bs++;
        }
        else
        {
            /* Get end knots */

            for ( i = 0; i <= pq; i++ )
            {
                T[kind] = S[bs];
                kind++;
            }
        }
    }

    /* Compact output curve */

    if( cind LT nh + 1 )
    {
        N_CrvSetSizeIndices( curH, cind - 1, pq, kind - 1 );

        error = N_CrvCompress( curH, SH );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvFuncMultiplyCrv4d */

#if NLIB_UNUSED

/**********************************************************************/
/* N_CrvFuncMultiplyConstant: Product of constant and curve function                   */
/**********************************************************************/

/*******************************************************************//**

   DESCRIPTION:

     This symbolic operators routine computes the product of a constant 
     and a curve function. A typical calling example is:

       NL_REAL  alpha; 
       NL_CFUN  cfn;
       ...
       (define cfn and alpha);
       ...
       N_CrvFuncMultiplyConstant(alpha,&cfn);

     THE  PRODUCT  IS  COMPUTED IN-PLACE, I.E. THE ORIGINAL NL_FUNCTION IS 
     DESTROYED.


   ACCESS:
   
     alpha , input  ,  Scalar
     cfn   , in/out ,  Product of alpha and cfn


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncMultiplyConstant( NL_REAL alpha, NL_CFUN *cfn )
{

    NL_INDEX i, n;

    NL_REAL *fu;

    /* Get locals */

    N_CrvFuncCntrlVal( cfn, &n, &fu );

    /* Compute product */

    for ( i = 0; i <= n; i++ )
        fu[i] = alpha * fu[i];
} /* end N_CrvFuncMultiplyConstant */

/***************************/
/* advanced NL_SFUN functions */
/***************************/

/**********************************************************************/
/* N_SrfFuncEvalRatBasis: Evaluate rat basis function given as a surface function  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine evaluates a rational basis function given as a surface
     function, i.e. the  surface function  represents the denominator of
     the form

                        w_{i,j} N_{i,p} N_{j,q}   
          R_{i,j} = -------------------------------
                    sum_r^s w_{r,s} N_{r,p} N_{s,q}

     A typical calling example is:

       NL_SFUN       sfn;
       NL_INDEX      i, j;
       NL_PARAMETER  u, v;
       NL_REAL       R;
       ...
       (define sfn; get i, j, u and v);
       ...
       N_SrfFuncEvalRatBasis(&sfn,i,j,u,v,NL_LEFT,NL_LEFT,&R);


   ACCESS:
   
     sfn , input  ,  Surface function (the  denomonator of  the rational 
                     form)
     i,j , input  ,  Indexes of rational basis function
     u,v , input  ,  Parameter values 
     ufl , input  ,  Flag:
                       NL_LEFT : u is in [u[k],u[k+1])
                       NL_RIGHT: u is in (u[k],u[k+1]]
     vfl , input  ,  Flag:
                       NL_LEFT : v is in [v[l],v[l+1])
                       NL_RIGHT: v is in (v[l],v[l+1]]
     R   , output ,  Basis function evaluated at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncEvalRatBasis( NL_SFUN *sfn, NL_INDEX i, NL_INDEX j, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_REAL *R )
{

    NL_FLAG error = NL_NO;

    NL_INDEX k, l;

    NL_DEGREE p, q;

    NL_REAL ** w, *tmp, Ni, Nj, num, den;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SFuncGetComponents( sfn, &k, &l, &w, &p, &q, &k, &l, &tmp, &tmp );
    N_SFuncGetKnotVectors( sfn, &knu, &knv );

    /* Get the numerator */

    error = N_BasisIEval( knu, i, p, u, ufl, &Ni );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIEval( knv, j, q, v, vfl, &Nj );

    if( error EQ NL_YES )
        NL_OUT;

    num = w[i][j] * Ni * Nj;

    /* Get the denominator */

    error = N_SrfFuncEvalPt( sfn, u, v, ufl, vfl, &den );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute basis function */

    *R = num / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfFuncEvalRatBasis */

/**********************************************************************/
/* N_SrfFuncDerivFuncAtKnot: Compute derivatives of a surface function wrt a knot     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes  derivatives  of a  surface  function  with 
     respect to a knot. A typical calling example is:

       NL_SFUN       sfn;
       NL_INDEX      k;
       NL_PARAMETER  u, v;
       NL_REAL       *fd;
       ...
       (define sfn; get u, v, and k);
       ...
       N_SrfFuncDerivFuncAtKnot(&sfn,k,u,v,NL_UDIR,NL_LEFT,NL_RIGHT,NL_LEFT,fd);


   ACCESS:
   
     sfn , input  ,  Surface function
     k   , input  ,  Index of knot, i.e. the derivative  with respect to 
                     u_k is computed
     u,v , input  ,  Parameter values
     dir , input  ,  Flag:
                       NL_UDIR: u-derivative required
                       NL_VDIR: v-derivative required
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              t_(k) != t_(k-1)
                       NL_RIGHT: right  derivative.  NL_INDEX  k  MUST SATISFY
                              t_(k) != t_(k+1)
                              (t is either u or v)
     ulp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     vlp , input  ,  Flag:
                       NL_LEFT : v is in [v[j],v[j+1])
                       NL_RIGHT: v is in (v[j],v[j+1]]
     fd  , output ,  Function derivative


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncDerivFuncAtKnot( NL_SFUN *sfn, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG dir, NL_FLAG flk, NL_FLAG ulp, NL_FLAG vlp, NL_REAL *fd )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, spn;

    NL_DEGREE p, q;

    NL_REAL ** Bk, ** fuv;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfFuncCntrlVal( sfn, &i, &j, &fuv );
    N_SFuncGetDegrees( sfn, &p, &q );
    N_SFuncGetKnotVectors( sfn, &knu, &knv );

    /* Compute derivatives */

    Bk = N_AllocReal2dArray( p + 1, q + 1, &SL );

    if( Bk EQ NULL )
        NL_QUIT;

    error = N_BiBasisKnotDeriv( knu, knv, p, q, k, u, v, dir, flk, ulp, vlp, Bk, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    *fd = 0.0;

    if( dir EQ NL_UDIR )
    {
        for ( i = k - p - 1; i <= k; i++ )
        {
            for ( j = spn - q; j <= spn; j++ )
            {
                *fd += fuv[i][j] * Bk[i - k + p + 1][j - spn + q];
            }
        }
    }
    else if( dir EQ NL_VDIR )
    {
        for ( i = spn - p; i <= spn; i++ )
        {
            for ( j = k - q - 1; j <= k; j++ )
            {
                *fd += fuv[i][j] * Bk[i - spn + p][j - k + q + 1];
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncDerivFuncAtKnot */

/**********************************************************************/
/* N_SFuncEvalGrid: Compute a grid of points on a surface function           */
/**********************************************************************/

/*******************************************************************//**

   DESCRIPTION:

     This routine computes a grid of points on a surface function given 
     a set of u- and v-values. A typical calling example is:

       NL_SFUN       sfn;
       NL_PARAMETER  *u, *v;
       NL_INDEX      n, m;
       NL_REAL       **F;
       ...
       (define sfn; get u and v, allocate memory for F);
       ...
       N_SFuncEvalGrid(&sfn,u,v,n,m,NL_LEFT,NL_RIGHT,F);

     MEMORY FOR F MUST BE ALLOCATED IN  THE CALLING  ROUTINE TO HOLD UP
     TO F[n][m].


   ACCESS:
   
     sfn     , input  ,  Function
     u,v     , input  ,  Arrays of INCREASING parameters
     n,m     , input  ,  Highest indexes in u and v
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u or v)
     F       , output ,  Values of the  function.  F[i][j] is  a  value 
                         computed at (u[i],v[j]).


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SFuncEvalGrid( NL_SFUN *sfn, NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX n, NL_INDEX m, NL_FLAG ufl, NL_FLAG vfl, NL_REAL ** F )
{
    NL_PRIVATE NL_STRING rname = _T("N_SFuncEvalGrid");

    NL_FLAG error = NL_NO;

    NL_INDEX *usp, *vsp, i, j, k, l, nu, mv, r, s, a, b;

    NL_DEGREE p, q;

    NL_REAL ** NU, ** NV, ** fuv, *U, *V, T;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SFuncGetComponents( sfn, &nu, &mv, &fuv, &p, &q, &r, &s, &U, &V );
    N_SFuncGetKnotVectors( sfn, &knu, &knv );

    /* Check parameters */

    if( n LT 0 OR m LT 0 )
        NL_ERROR( NL_IND_ERR );

    if( u[0]LT U[0]OR u[n]GT U[r] )
        NL_ERROR( NL_PAR_ERR );

    if( v[0]LT V[0]OR v[m]GT V[s] )
        NL_ERROR( NL_PAR_ERR );

    /* Compute non-vanishing B-splines */

    NU = N_AllocReal2dArray( n, p, &SL );

    if( NU EQ NULL )
        NL_QUIT;

    NV = N_AllocReal2dArray( m, q, &SL );

    if( NV EQ NULL )
        NL_QUIT;

    usp = N_AllocInt1dArray( n, &SL );

    if( usp EQ NULL )
        NL_QUIT;

    vsp = N_AllocInt1dArray( m, &SL );

    if( vsp EQ NULL )
        NL_QUIT;

    error = N_BasisEvalArray( knu, p, u, n, ufl, NU, usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEvalArray( knv, q, v, m, vfl, NV, vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute grid of points on the function */

    for ( k = 0; k <= n; k++ )
    {
        for ( l = 0; l <= m; l++ )
        {
            F[k][l] = 0.0;

            for ( i = 0; i <= p; i++ )
            {
                a = usp[k] - p + i;
                T = 0.0;

                for ( j = 0; j <= q; j++ )
                {
                    b = vsp[l] - q + j;
                    T += NV[l][j] * fuv[a][b];
                }

                F[k][l] += NU[k][i] * T;
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SFuncEvalGrid */

/***************************/
/* advanced NL_SFUN tools     */
/***************************/

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_SRFFUNCREFINE: Refine a surface function with a given knot vector       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine refines a  surface function  with a given knot 
     vector in  either  u- or  v-direction. It is  assumed that the new 
     knot vector fits into the old  ones, i.e. T[0] < X[0] <=...<= X[r] 
     < T[k] holds where T[0],...,T[k] are the old knots in either the U
     or  the V knot  vector, and  X[0],...,X[r] are the new knots to be
     inserted. If the output function is initialized to NULL, memory to 
     store new control values  and  knots is  allocated. If  the output  
     function is the same as the input function, knot refinement  is in  
     place and  the original  function is  destroyed. A typical calling 
     example is:

       NL_SFUN        sfnP, sfnQ;
       NL_KNOTVECTOR  knx;
       NL_STACKS      SP, SQ;
       ...
       (define sfnP and knx);
       ...
       N_SFuncInitArrays(&sfnQ);
       N_SrfFuncRefine(&sfnP,&knx,NL_UDIR,&sfnQ,&SP,&SQ);
       N_SrfFuncRefine(&sfnP,&knx,NL_VDIR,&sfnP,&SP,&SP);

     If memory is  available, sfnQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in sfnQ's 
     knot vector and control value objects.


   ACCESS:
   
     sfnP , input  ,  Surface function
     knx  , input  ,  New knot vector to be inserted
     dir  , input  ,  Flag:
                        NL_UDIR: Refine in u-direction
                        NL_VDIR: Refine in v-direction
     sfnQ , output ,  Function after knot refinement
     SP   , input  ,  sfnP's stack
     SQ   , input  ,  sfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncRefine( NL_SFUN *sfnP, NL_KNOTVECTOR *knx, NL_FLAG dir, NL_SFUN *sfnQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncRefine");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, a, b, k, l, n, m, r, s, xr, t, row, col, ni, mi, ri, si;

    NL_DEGREE p, q;

    NL_REAL ** fp, ** fq, *UP, *VP, *UQ, *VQ, *X, alf, oma;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SFuncGetComponents( sfnP, &n, &m, &fp, &p, &q, &r, &s, &UP, &VP );
    N_SFuncGetKnotVectors( sfnP, &knu, &knv );
    N_KnotVectorGetKnots( knx, &xr, &X );

    /* Check parameters and compute highest indexes */

    if( xr LT 0 )
        NL_ERROR( NL_INP_ERR );

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, X[0], rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_KnotVectorIsEndParam( knu, X[xr], rname );

            if( error EQ NL_YES )
                NL_OUT;

            ni = n + xr + 1;
            mi = m;
            ri = r + xr + 1;
            si = s;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, X[0], rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_KnotVectorIsEndParam( knv, X[xr], rname );

            if( error EQ NL_YES )
                NL_OUT;

            ni = n;
            mi = m + xr + 1;
            ri = r;
            si = s + xr + 1;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( sfnP EQ sfnQ )
    {
        sfnA = *sfnP;

        error = N_AllocSFuncArrays( sfnP, ni, mi, p, q, ri, si, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnP, &fq, &UQ, &VQ );
    }
    else
    {
        error = N_SFuncSizeArrays( sfnQ, ni, mi, p, q, ri, si, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnQ, &fq, &UQ, &VQ );
    }

    /* Refine U knot vector */

    if( dir EQ NL_UDIR )
    {
        /* Find knot span indexes */

        error = N_BasisFindSpan( knu, p, X[0], NL_LEFT, &a );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knu, p, X[xr], NL_LEFT, &b );

        if( error EQ NL_YES )
            NL_OUT;
        b++;

        /* Initialize output knot vectors */

        for ( i = 0; i <= a; i++ )
            UQ[i] = UP[i];

        for ( i = b + p; i <= r; i++ )
            UQ[i + xr + 1] = UP[i];

        for ( j = 0; j <= s; j++ )
            VQ[j] = VP[j];

        /* Save unaltered control values */

        for ( col = 0; col <= m; col++ )
        {
            for ( j = 0; j <= a - p; j++ )
                fq[j][col] = fp[j][col];

            for ( j = b - 1; j <= n; j++ )
                fq[j + xr + 1][col] = fp[j][col];
        }

        /* Now refine the knot vector */

        i = b + p - 1;
        k = b + p + xr;

        for ( j = xr; j >= 0; j-- )
        {
            while( X[j]LE UP[i]AND i GT a )
            {
                for ( col = 0; col <= m; col++ )
                    fq[k - p - 1][col] = fp[i - p - 1][col];
                UQ[k] = UP[i];
                k--;
                i--;
            }

            for ( col = 0; col <= m; col++ )
                fq[k - p - 1][col] = fq[k - p][col];

            for ( l = 1; l <= p; l++ )
            {
                t = k - p + l;
                alf = UQ[k + l] - X[j];

                if( fabs( alf )LE 0.0 )
                {
                    for ( col = 0; col <= m; col++ )
                        fq[t - 1][col] = fq[t][col];
                }
                else
                {
                    alf = alf / (UQ[k + l] - UP[i - p + l]);
                    oma = 1.0 - alf;

                    for ( col = 0; col <= m; col++ )
                    {
                        fq[t - 1][col] = alf * fq[t - 1][col] + oma * fq[t][col];
                    }
                }
            }
            UQ[k] = X[j];
            k--;
        }
    } /* End of NL_UDIR */

    /* Refine V knot vector */

    if( dir EQ NL_VDIR )
    {
        /* Find knot span indexes */

        error = N_BasisFindSpan( knv, q, X[0], NL_LEFT, &a );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisFindSpan( knv, q, X[xr], NL_LEFT, &b );

        if( error EQ NL_YES )
            NL_OUT;
        b++;

        /* Initialize output knot vector */

        for ( j = 0; j <= a; j++ )
            VQ[j] = VP[j];

        for ( j = b + q; j <= s; j++ )
            VQ[j + xr + 1] = VP[j];

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        /* Save unaltered control points */

        for ( row = 0; row <= n; row++ )
        {
            for ( j = 0; j <= a - q; j++ )
                fq[row][j] = fp[row][j];

            for ( j = b - 1; j <= m; j++ )
                fq[row][j + xr + 1] = fp[row][j];
        }

        /* Now refine the knot vector */

        i = b + q - 1;
        k = b + q + xr;

        for ( j = xr; j >= 0; j-- )
        {
            while( X[j]LE VP[i]AND i GT a )
            {
                for ( row = 0; row <= n; row++ )
                    fq[row][k - q - 1] = fp[row][i - q - 1];
                VQ[k] = VP[i];
                k--;
                i--;
            }

            for ( row = 0; row <= n; row++ )
                fq[row][k - q - 1] = fq[row][k - q];

            for ( l = 1; l <= q; l++ )
            {
                t = k - q + l;
                alf = VQ[k + l] - X[j];

                if( fabs( alf )LE 0.0 )
                {
                    for ( row = 0; row <= n; row++ )
                        fq[row][t - 1] = fq[row][t];
                }
                else
                {
                    alf = alf / (VQ[k + l] - VP[i - q + l]);
                    oma = 1.0 - alf;

                    for ( row = 0; row <= n; row++ )
                    {
                        fq[row][t - 1] = alf * fq[row][t - 1] + oma * fq[row][t];
                    }
                }
            }
            VQ[k] = X[j];
            k--;
        }
    } /* End of NL_VDIR */

    /* If refinement is in place, kill old surface */

    if( sfnP EQ sfnQ )
        N_FreeSrfFuncData( &sfnA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncRefine */



/**********************************************************************/
/* N_SRFFUNCREMOVEKNOT: Remove one knot multiple times from a surface function   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes one knot  multiple times from a surface
     function either in  u- or in  v-direction. IT IS  ASSUMED THAT THE 
     KNOT IS PRECIELY REMOVABLE, I.E.  NL_NO NL_ERROR  CHECKING IS PERFORMED. 
     The knot must  be an  interior  knot. If  the  output  function is  
     initialized to NULL, memory to  store new control values and knots  
     is allocated. A typical calling example is:

       NL_SFUN       sfnP, sfnQ;
       NL_PARAMETER  t;
       NL_INDEX      nt;
       NL_STACKS     SQ;
       ...
       (define sfnP; get t and nt);
       ...
       N_SFuncInitArrays(&sfnQ);
       N_SrfFuncRemoveKnot(&sfnP,t,nt,NL_UDIR,&sfnQ,&SQ);
       N_SrfFuncRemoveKnot(&sfnP,t,nt,NL_VDIR,&sfnP,&SQ);

     If memory is  available, sfnQ  is not  initialized and the routine
     assumes that memory allocation  has been done. However, it  checks  
     for the proper amount by looking  at the highest indexes in sfnQ's 
     knot  vector and  control  value  objects. If  sfnP is the same as 
     sfnQ, knot removal is done in place.


   ACCESS:
   
     sfnP , input  ,  Surface function
     t,nt , input  ,  Knot  "t" to be  removed  "nt" times (nt  must be 
                      less  than or  equal to the  multiplicity  of the 
                      knot)
     dir  , input  ,  Flag:
                        NL_UDIR: Remove in u-direction
                        NL_VDIR: Remove in v-direction
     sfnQ , output ,  Function after knot removal
     SQ   , input  ,  sfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncRemoveKnot( NL_SFUN *sfnP, NL_PARAMETER t, NL_INDEX nt, NL_FLAG dir, NL_SFUN *sfnQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncRemoveKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, row, col, ii, jj, first, last, off, n, m, r, s, a, spn, mlt, fout;

    NL_DEGREE p, q;

    NL_REAL ** fp, ** fq, *fr, *UP, *VP, *UQ, *VQ, *alf, *oma, *bet, *omb;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SFuncGetComponents( sfnP, &n, &m, &fp, &p, &q, &r, &s, &UP, &VP );
    N_SFuncGetKnotVectors( sfnP, &knu, &knv );

    /* Check parameters */

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spn, &mlt );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spn, &mlt );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    if( nt GT mlt )
        NL_ERROR( NL_KNT_ERR );

    /* See if memory is needed */

    if( sfnP EQ sfnQ )
    {
        N_SFuncGetKnots( sfnP, &fq, &UQ, &VQ );
    }
    else
    {
        error = N_SFuncSizeArrays( sfnQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnQ, &fq, &UQ, &VQ );
    }

    /* Allocate local memory */

    a = NL_MAX( p, q );

    alf = N_AllocReal1dArray( 2 * a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal1dArray( 2 * a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    bet = N_AllocReal1dArray( 2 * a, &SL );

    if( bet EQ NULL )
        NL_QUIT;

    omb = N_AllocReal1dArray( 2 * a, &SL );

    if( omb EQ NULL )
        NL_QUIT;

    fr = N_AllocReal1dArray( 2 * a, &SL );

    if( fr EQ NULL )
        NL_QUIT;

    /* Initialize */

    if( sfnP NEQ sfnQ )
    {
        for ( row = 0; row <= n; row++ )
        {
            for ( col = 0; col <= m; col++ )
                fq[row][col] = fp[row][col];
        }

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= s; j++ )
            VQ[j] = VP[j];
    }

    /* Remove in u-direction */

    if( dir EQ NL_UDIR )
    {
        fout = (2 * spn - mlt - p) / 2;
        first = spn - p;
        last = spn - mlt;

        for ( k = 0; k < nt; k++ )
        {
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT k )
            {
                alf[i - first] = (UQ[i + p + 1] - UQ[i]) / (t - UQ[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (UQ[j + p - k + 1] - UQ[j - k]) / (UQ[j + p - k + 1] - t);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            /* Remove one knot along each row */

            for ( col = 0; col <= m; col++ )
            {

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                fr[0] = fq[off][col];
                fr[last + 1 - off] = fq[last + 1][col];

                /* Get new control values for one removal step */

                while( (j - i)GT k )
                {
                    fr[ii] = alf[i - first] * fq[i][col] + oma[i - first] * fr[ii - 1];
                    fr[jj] = bet[j - first] * fq[j][col] + omb[j - first] * fr[jj + 1];
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Save new control values */

                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    fq[i][col] = fr[i - off];
                    fq[j][col] = fr[j - off];
                    i++;
                    j--;
                }
            }

            /* Shift down knots */

            for ( l = spn - k; l <= r - k - 1; l++ )
                UQ[l] = UQ[l + 1];

            first--;
            last++;
        }

        /* Shift down control values */

        j = fout;
        i = j;

        for ( l = 1; l < k; l++ )
        {
            if( l % 2 )
                i++;
            else
                j--;
        }

        for ( col = 0; col <= m; col++ )
        {
            a = j;

            for ( l = i + 1; l <= n; l++ )
            {
                fq[a][col] = fq[l][col];
                a++;
            }
        }

        N_SFuncSetSizeIndices( sfnQ, n - nt, m, p, q, r - nt, s );
    }

    /* Remove in v-direction */

    if( dir EQ NL_VDIR )
    {
        fout = (2 * spn - mlt - q) / 2;
        first = spn - q;
        last = spn - mlt;

        /* Remove the knot "nt" times */

        for ( k = 0; k < nt; k++ )
        {
            off = first - 1;

            /* Save some parameters */

            i = first;
            j = last;

            while( (j - i)GT k )
            {
                alf[i - first] = (VQ[i + q + 1] - VQ[i]) / (t - VQ[i]);
                oma[i - first] = 1.0 - alf[i - first];
                bet[j - first] = (VQ[j + q - k + 1] - VQ[j - k]) / (VQ[j + q - k + 1] - t);
                omb[j - first] = 1.0 - bet[j - first];
                i++;
                j--;
            }

            /* Remove one knot along each column */

            for ( row = 0; row <= n; row++ )
            {

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                fr[0] = fq[row][off];
                fr[last + 1 - off] = fq[row][last + 1];

                /* Get new control values for one removal step */

                while( (j - i)GT k )
                {
                    fr[ii] = alf[i - first] * fq[row][i] + oma[i - first] * fr[ii - 1];
                    fr[jj] = bet[j - first] * fq[row][j] + omb[j - first] * fr[jj + 1];
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Save new control values */

                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    fq[row][i] = fr[i - off];
                    fq[row][j] = fr[j - off];
                    i++;
                    j--;
                }
            }

            /* Shift down knots */

            for ( l = spn - k; l <= s - k - 1; l++ )
                VQ[l] = VQ[l + 1];

            first--;
            last++;
        }

        /* Shift down control values */

        j = fout;
        i = j;

        for ( l = 1; l < k; l++ )
        {
            if( l % 2 )
                i++;
            else
                j--;
        }

        for ( row = 0; row <= n; row++ )
        {
            a = j;

            for ( l = i + 1; l <= m; l++ )
            {
                fq[row][a] = fq[row][l];
                a++;
            }
        }

        N_SFuncSetSizeIndices( sfnQ, n, m - nt, p, q, r, s - nt );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncRemoveKnot */

#if NLIB_UNUSED

/**********************************************************************/
/* N_SrfFuncDegreeElevate: Elevate the degree of a surface function                 */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools routine  elevates the degree of a  surface function from 
     any  degree  to  any  higher  degree. If  the  output  function  is  
     initialized to NULL, memory  to store new control  values and knots 
     is  allocated. If the  output function  is  the same  as  the input  
     function, degree elevation is in place and the original function is  
     destroyed. A typical calling example is:

       NL_SFUN    sfnP, sfnQ;
       NL_INDEX   t;
       NL_STACKS  SP, SQ;
       ...
       (define sfnP, get t);
       ...
       N_SFuncInitArrays(&sfnQ);
       N_SrfFuncDegreeElevate(&sfnP,t,NL_UDIR,&sfnQ,&SP,&SQ);
       N_SrfFuncDegreeElevate(&sfnP,t,NL_VDIR,&sfnP,&SP,&SP);

     If memory is  available, sfnQ  is not  initialized and  the routine
     assumes  that memory  allocation  has been done. However, it checks  
     for the proper amount by looking  at the  highest indexes in sfnQ's  
     knot vector and control net objects.


   ACCESS:
   
     sfnP , input  ,  Surface function
     t    , input  ,  Increment (new degree is old_degree+t)
     dir  , input  ,  Flag:
                        NL_UDIR: Elevate in u-direction
                        NL_VDIR: Elevate in v-direction
     sfnQ , output ,  Surface function after degree elevation
     SP   , input  ,  sfnP's stack
     SQ   , input  ,  sfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncDegreeElevate( NL_SFUN *sfnP, NL_INDEX t, NL_FLAG dir, NL_SFUN *sfnQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncDegreeElevate");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, row, col, np, mp, rp, sp, nq, mq, rq, sq, r, s, a, b, mlt, kind, cind, pind = 0, lbz, rbz, bi, bj, di, dj, first, last, oldr, save;

    NL_DEGREE pp, qp, pq, qq;

    NL_REAL ** fp, ** fq, ** fb, ** fn, ** fd, *UP, *VP, *UQ, *VQ, ** ralf, ** roma, ** rbet, ** romb, *alfs, *omas, num, den;

    NL_RMATRIX dm;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SFuncGetComponents( sfnP, &np, &mp, &fp, &pp, &qp, &rp, &sp, &UP, &VP );
    N_SFuncGetKnotVectors( sfnP, &knu, &knv );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:
            if( t LT 0 OR pp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knu, pp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knu, pp, &s );

            nq = np + t * s;
            rq = rp + t * (s + 1);
            pq = (NL_DEGREE)( pp + t );
            mq = mp;
            sq = sp;
            qq = qp;

            bi = pp;
            bj = mp;
            di = pq;
            dj = mp;
            break;

        case NL_VDIR:
            if( t LT 0 OR qp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knv, qp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knv, qp, &s );

            nq = np;
            rq = rp;
            pq = pp;
            mq = mp + t * s;
            sq = sp + t * (s + 1);
            qq = (NL_DEGREE)( qp + t );

            bi = np;
            bj = qp;
            di = np;
            dj = qq;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( sfnP EQ sfnQ )
    {
        sfnA = *sfnP;

        error = N_AllocSFuncArrays( sfnP, nq, mq, pq, qq, rq, sq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnP, &fq, &UQ, &VQ );
    }
    else
    {
        error = N_SFuncSizeArrays( sfnQ, nq, mq, pq, qq, rq, sq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnQ, &fq, &UQ, &VQ );
    }

    /* Allocate local memory */

    a = NL_MAX( pp, qp );

    fb = N_AllocReal2dArray( bi, bj, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    fn = N_AllocReal2dArray( bi, bj, &SL );

    if( fn EQ NULL )
        NL_QUIT;

    fd = N_AllocReal2dArray( di, dj, &SL );

    if( fd EQ NULL )
        NL_QUIT;

    ralf = N_AllocReal2dArray( a, 2 * a, &SL );

    if( ralf EQ NULL )
        NL_QUIT;

    roma = N_AllocReal2dArray( a, 2 * a, &SL );

    if( roma EQ NULL )
        NL_QUIT;

    rbet = N_AllocReal2dArray( a, 2 * a, &SL );

    if( rbet EQ NULL )
        NL_QUIT;

    romb = N_AllocReal2dArray( a, 2 * a, &SL );

    if( romb EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( a, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( a, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Degree elevate in u-direction */

    if( dir EQ NL_UDIR )
    {
        a = pp;
        b = pp + 1;
        r = -1;
        cind = 1;
        kind = pq + 1;

        for ( i = 0; i <= pq; i++ )
            UQ[i] = UP[a];

        for ( j = 0; j <= sp; j++ )
            VQ[j] = VP[j];

        /* Initialize Bezier strip */

        for ( col = 0; col <= mp; col++ )
        {
            fq[0][col] = fp[0][col];

            for ( i = 0; i <= pp; i++ )
                fb[i][col] = fp[i][col];
        }

        /* Get degree elevation matrix */

        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( pp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the i-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the i-th and the (i-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT rp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT rp AND UP[b]EQ UP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = pp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = pq - (r + 1) / 2;
            else
                rbz = pq;

            /* Save some entities */

            if( r GT 0 )
            {
                num = UP[b] - UP[a];

                for ( k = pp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = UP[b] - UP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (UP[b] - UQ[i]) / (UP[a] - UQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (UP[b] - UQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each row, perform curve function degree elevation */

            for ( col = 0; col <= mp; col++ )
            {
                pind = cind;

                /* Insert knot */

                if( r GT 0 )
                {
                    for ( j = 1; j <= r; j++ )
                    {
                        save = r - j;
                        s = mlt + j;

                        for ( k = pp; k >= s; k-- )
                        {
                            fb[k][col] = alfs[k - s] * fb[k][col] + omas[k - s] * fb[k - 1][col];
                        }
                        fn[save][col] = fb[pp][col];
                    }
                } /* End of insert knot */

                /* Now degree elevate Bezier */

                error = N_BezSrfFuncElevateDegree( fb, pp, t, &dm, NL_UDIR, lbz, pq, col, fd );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Remove the knot UP[a] */

                if( oldr GT 1 )
                {
                    first = kind - 2;
                    last = kind;

                    for ( k = 1; k < oldr; k++ )
                    {
                        i = first;
                        j = last;
                        l = j - kind + 1;

                        while( (j - i)GT k )
                        {
                            if( i LT cind )
                            {
                                fq[i][col] = ralf[k][i - first] * fq[i][col] + roma[k][i - first] * fq[i - 1][col];
                            }

                            if( j GE lbz )
                            {
                                fd[l][col] = rbet[k][j - last] * fd[l][col] + romb[k][j - last] * fd[l + 1][col];
                            }
                            i++;
                            j--;
                            l--;
                        }
                        first--;
                        last++;
                    }
                } /* End of removing knot */

                /* Load control values */

                for ( i = lbz; i <= rbz; i++ )
                {
                    fq[pind][col] = fd[i][col];
                    pind++;
                }

                /* Initialize for next pass through */

                if( b LT rp )
                {
                    for ( i = 0; i < r; i++ )
                        fb[i][col] = fn[i][col];

                    for ( i = r; i <= pp; i++ )
                        fb[i][col] = fp[b - pp + i][col];
                }
            } /* End for each row */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ pp )
            {
                for ( i = 0; i < pq - oldr; i++ )
                {
                    UQ[kind] = UP[a];
                    kind++;
                }
            }

            if( b LT rp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( i = 0; i <= pq; i++ )
                    UQ[kind + i] = UP[b];
            }
        } /* End of while */
    }     /* End of NL_UDIR */

    /* Degree elevate in v-direction */

    if( dir EQ NL_VDIR )
    {
        a = qp;
        b = qp + 1;
        r = -1;
        cind = 1;
        kind = qq + 1;

        for ( i = 0; i <= rp; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= qq; j++ )
            VQ[j] = VP[a];

        /* Initialize Bezier strip */

        for ( row = 0; row <= np; row++ )
        {
            fq[row][0] = fp[row][0];

            for ( j = 0; j <= qp; j++ )
                fb[row][j] = fp[row][j];
        }

        /* Get degree elevation matrix */

        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( qp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the j-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the j-th and the (j-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT sp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT sp AND VP[b]EQ VP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = qp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = qq - (r + 1) / 2;
            else
                rbz = qq;

            /* Save some entities */

            if( r GT 0 )
            {
                num = VP[b] - VP[a];

                for ( k = qp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (VP[a + k] - VP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = VP[b] - VP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (VP[b] - VQ[i]) / (VP[a] - VQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (VP[b] - VQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each column, perform curve function elevation */

            for ( row = 0; row <= np; row++ )
            {
                pind = cind;

                /* Insert knot */

                if( r GT 0 )
                {
                    for ( j = 1; j <= r; j++ )
                    {
                        save = r - j;
                        s = mlt + j;

                        for ( k = qp; k >= s; k-- )
                        {
                            fb[row][k] = alfs[k - s] * fb[row][k] + omas[k - s] * fb[row][k - 1];
                        }
                        fn[row][save] = fb[row][qp];
                    }
                } /* End of insert knot */

                /* Now degree elevate Bezier */

                error = N_BezSrfFuncElevateDegree( fb, qp, t, &dm, NL_VDIR, lbz, qq, row, fd );

                if( error EQ NL_YES )
                    NL_OUT;

                /* Remove the knot VP[a] */

                if( oldr GT 1 )
                {
                    first = kind - 2;
                    last = kind;

                    for ( k = 1; k < oldr; k++ )
                    {
                        i = first;
                        j = last;
                        l = j - kind + 1;

                        while( (j - i)GT k )
                        {
                            if( i LT cind )
                            {
                                fq[row][i] = ralf[k][i - first] * fq[row][i] + roma[k][i - first] * fq[row][i - 1];
                            }

                            if( j GE lbz )
                            {
                                fd[row][l] = rbet[k][j - last] * fd[row][l] + romb[k][j - last] * fd[row][l + 1];
                            }
                            i++;
                            j--;
                            l--;
                        }
                        first--;
                        last++;
                    }
                } /* End of removing knot */

                /* Load control points */

                for ( j = lbz; j <= rbz; j++ )
                {
                    fq[row][pind] = fd[row][j];
                    pind++;
                }

                /* Initialize for next pass through */

                if( b LT sp )
                {
                    for ( j = 0; j < r; j++ )
                        fb[row][j] = fn[row][j];

                    for ( j = r; j <= qp; j++ )
                        fb[row][j] = fp[row][b - qp + j];
                }
            } /* End for each column */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ qp )
            {
                for ( j = 0; j < qq - oldr; j++ )
                {
                    VQ[kind] = VP[a];
                    kind++;
                }
            }

            if( b LT sp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( j = 0; j <= qq; j++ )
                    VQ[kind + j] = VP[b];
            }
        } /* End of while */
    }     /* End of NL_VDIR */

    /* If degree elevation is in place, kill old function */

    if( sfnP EQ sfnQ )
        N_FreeSrfFuncData( &sfnA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncDegreeElevate */

/**********************************************************************/
/* N_SRFFUNCINSERTKNOT: Insert a new knot into a bivariate function              */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This tools routine inserts one new knot into a bivariate function 
     either in u- or in v-direction. The  new knot must be an interior 
     knot and the  sum of the  multiplicities of  the old and  the new 
     knots must be less than or equal to the respective degree. If the  
     output function  is  initialized  to  NULL, memory  to  store new 
     control points and knots is allocated. A typical  calling example
     is as follows:

       NL_SFUN       sfnP, sfnQ;
       NL_PARAMETER  t;
       NL_INDEX      mt;
       NL_STACKS     SP, SQ;
       ...
       (define surP, get t and mt);
       ...
       N_SFuncInitArrays(&sfnQ);
       N_SrfFuncInsertKnot(&sfnP,t,mt,NL_UDIR,&sfnQ,&SP,&SQ);
       N_SrfFuncInsertKnot(&sfnP,t,mt,NL_VDIR,&sfnP,&SP,&SP);

     If memory is  available, sfnQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in sfnQ's 
     knot vector and polygon objects. If sfnP is the same as sfnQ, the
     insertion is done in place.


   ACCESS:
   
     sfnP , input  ,  Bivariate function
     t    , input  ,  New knot to be inserted
     mt   , input  ,  Number of times t is to be inserted
     dir  , input  ,  Flag:
                        NL_UDIR: Insert in u-direction
                        NL_VDIR: Insert in v-direction
     sfnQ , output ,  Function after knot inserion
     SP   , input  ,  sfnP's stack
     SQ   , input  ,  sfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncInsertKnot( NL_SFUN *sfnP, NL_PARAMETER t, NL_INDEX mt, NL_FLAG dir, NL_SFUN *sfnQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncInsertKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, spu = 0, mlu = 0, spv = 0, mlv = 0, a, ni, mi, ri, si;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, ** alf, ** oma, ** fp, ** fq, *fr;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SFuncGetComponents( sfnP, &n, &m, &fp, &p, &q, &r, &s, &UP, &VP );
    N_SFuncGetKnotVectors( sfnP, &knu, &knv );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlu + mt)GT p OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            ni = n + mt;
            mi = m;
            ri = r + mt;
            si = s;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlv + mt)GT q OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            ni = n;
            mi = m + mt;
            ri = r;
            si = s + mt;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( sfnP EQ sfnQ )
    {
        sfnA = *sfnP;

        error = N_AllocSFuncArrays( sfnP, ni, mi, p, q, ri, si, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnP, &fq, &UQ, &VQ );
    }
    else
    {
        error = N_SFuncSizeArrays( sfnQ, ni, mi, p, q, ri, si, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnQ, &fq, &UQ, &VQ );
    }

    /* Allocate local memory */

    a = NL_MAX( p, q );

    alf = N_AllocReal2dArray( a, a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal2dArray( a, a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    fr = N_AllocReal1dArray( a, &SL );

    if( fr EQ NULL )
        NL_QUIT;

    /* Insert the knot */

    if( dir EQ NL_UDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= mt; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf[i][j] = (t - UP[a + j]) / (UP[spu + j + 1] - UP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each row do */

        for ( l = 0; l <= m; l++ )
        {
            /* Load auxiliary control values */

            for ( i = 0; i <= p - mlu; i++ )
                fr[i] = fp[spu - p + i][l];

            /* Save unaltered control values */

            for ( i = 0; i <= a; i++ )
                fq[i][l] = fp[i][l];

            for ( i = spu - mlu; i <= n; i++ )
                fq[i + mt][l] = fp[i][l];

            /* Now insert the knot */

            for ( i = 1; i <= mt; i++ )
            {
                a = spu - p + i;

                for ( j = 0; j <= p - i - mlu; j++ )
                {
                    fr[j] = alf[i][j] * fr[j + 1] + oma[i][j] * fr[j];
                }
                fq[a][l] = fr[0];
                fq[spu + mt - i - mlu][l] = fr[p - i - mlu];
            }

            /* Load the remaining control values */

            for ( i = a + 1; i < spu - mlu; i++ )
                fq[i][l] = fr[i - a];
        } /* End for each row */

        /* Load knot vectors */

        for ( i = 0; i <= spu; i++ )
            UQ[i] = UP[i];

        for ( i = 1; i <= mt; i++ )
            UQ[i + spu] = t;

        for ( i = spu + 1; i <= r; i++ )
            UQ[i + mt] = UP[i];

        for ( i = 0; i <= s; i++ )
            VQ[i] = VP[i];
    } /* End of NL_UDIR */

    if( dir EQ NL_VDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= mt; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf[i][j] = (t - VP[a + j]) / (VP[spv + j + 1] - VP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column do */

        for ( k = 0; k <= n; k++ )
        {
            /* Load auxiliary control values */

            for ( i = 0; i <= q - mlv; i++ )
                fr[i] = fp[k][spv - q + i];

            /* Save unaltered control values */

            for ( i = 0; i <= a; i++ )
                fq[k][i] = fp[k][i];

            for ( i = spv - mlv; i <= m; i++ )
                fq[k][i + mt] = fp[k][i];

            /* Now insert the knot */

            for ( i = 1; i <= mt; i++ )
            {
                a = spv - q + i;

                for ( j = 0; j <= q - i - mlv; j++ )
                {
                    fr[j] = alf[i][j] * fr[j + 1] + oma[i][j] * fr[j];
                }
                fq[k][a] = fr[0];
                fq[k][spv + mt - i - mlv] = fr[q - i - mlv];
            }

            /* Load the remaining control values */

            for ( i = a + 1; i < spv - mlv; i++ )
                fq[k][i] = fr[i - a];
        } /* End for each column */

        /* Load knot vectors */

        for ( i = 0; i <= spv; i++ )
            VQ[i] = VP[i];

        for ( i = 1; i <= mt; i++ )
            VQ[i + spv] = t;

        for ( i = spv + 1; i <= s; i++ )
            VQ[i + mt] = VP[i];

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];
    } /* End of NL_VDIR */

    /* If insertion is in place, kill old function */

    if( sfnP EQ sfnQ )
        N_FreeSrfFuncData( &sfnA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncInsertKnot */

/**********************************************************************/
/* N_SRFFUNCDECOMPOSE: Decompose a surface function into Bezier patches         */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  decomposes a surface function into its Bezier 
     constituents   without   using  knot   refinement.  Each  piece  is 
     represented  as a NURBS function even though the patches are Bezier
     functions. MEMORY TO STORE THE OUTPUT FUNCTIONS IS ALLOCATED INSIDE 
     THE ROUTINE. A typical calling example is:

       NL_SFUN    sfnP, ***sfnQ;
       NL_INDEX   kk, ll;
       NL_STACKS  SQ;
       ...
       (define sfnP);
       ...
       N_SrfFuncDecompose(&sfnP,&sfnQ,&kk,&ll,&SQ);

     sfnQ[i][j], 0<=i<=k, 0<=j<=l, is a pointer to the (i,j)th function. 


   ACCESS:
   
     sfnP  , input  ,  Function to be decomposed
     sfnQ  , output ,  2-D array of Bezier surfaces
     kk,ll , output ,  Highest indexes in surQ
     SQ    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncDecompose( NL_SFUN *sfnP, NL_SFUN **** sfnQ, NL_INDEX *kk, NL_INDEX *ll, NL_STACKS *SQ )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, ru, su, rv, sv, nsu, nsv, mlu, mlv, isu, ieu, isv, iev, iq, jq, save;

    NL_DEGREE p, q;

    NL_REAL ** fp, ** fq, ** nq = NULL, ** fb, ** nb, ** tmp, *UP, *VP, *UQ, *VQ, *uals, *vals, *omus, *omvs, num;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN *** sfnA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SFuncGetComponents( sfnP, &n, &m, &fp, &p, &q, &r, &s, &UP, &VP );
    N_SFuncGetKnotVectors( sfnP, &knu, &knv );

    /* Allocate memory */

    N_BasisGetSpanCount( knu, p, &nsu );
    N_BasisGetSpanCount( knv, q, &nsv );

    sfnA = N_Alloc2dArraySrfFunc( p, q, p, q, 2 * p + 1, 2 * q + 1, nsu - 1, nsv - 1, SQ );

    if( sfnA EQ NULL )
        NL_QUIT;

    fb = N_AllocReal2dArray( p, m, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    nb = N_AllocReal2dArray( p, m, &SL );

    if( nb EQ NULL )
        NL_QUIT;

    uals = N_AllocReal1dArray( p, &SL );

    if( uals EQ NULL )
        NL_QUIT;

    omus = N_AllocReal1dArray( p, &SL );

    if( omus EQ NULL )
        NL_QUIT;

    vals = N_AllocReal1dArray( q, &SL );

    if( vals EQ NULL )
        NL_QUIT;

    omvs = N_AllocReal1dArray( q, &SL );

    if( omvs EQ NULL )
        NL_QUIT;

    /* Initialize for u-directional decomposition */

    isu = p;
    ieu = p + 1;
    iq = -1;

    for ( i = 0; i <= p; i++ )
    {
        for ( l = 0; l <= m; l++ )
            fb[i][l] = fp[i][l];
    }

    /* Decompose in u-direction into Bezier strips */

    while( ieu LT r )
    {
        iq = iq + 1;

        /* Get knot multiplicity */

        i = ieu;

        while( ieu LT r AND UP[ieu]EQ UP[ieu + 1] )
            ieu++;
        mlu = ieu - i + 1;
        ru = p - mlu;

        /* Insert the knot */

        if( mlu LT p )
        {
            num = UP[ieu] - UP[isu];

            for ( i = p; i > mlu; i-- )
            {
                uals[i - mlu - 1] = num / (UP[isu + i] - UP[isu]);
                omus[i - mlu - 1] = 1.0 - uals[i - mlu - 1];
            }

            for ( i = 1; i <= ru; i++ )
            {
                su = mlu + i;
                save = ru - i;

                for ( j = p; j >= su; j-- )
                {
                    for ( l = 0; l <= m; l++ )
                    {
                        fb[j][l] = uals[j - su] * fb[j][l] + omus[j - su] * fb[j - 1][l];
                    }
                }

                if( ieu LT r )
                {
                    for ( l = 0; l <= m; l++ )
                        nb[save][l] = fb[p][l];
                }
            }
        }

        /* Bezier strip completed. Initialize for */
        /* v-directional decomposition            */

        isv = q;
        iev = q + 1;
        jq = -1;

        N_SFuncGetKnots( sfnA[iq][0], &fq, &UQ, &VQ );

        for ( j = 0; j <= q; j++ )
        {
            for ( k = 0; k <= p; k++ )
                fq[k][j] = fb[k][j];
        }

        /* Decompose in v-direction into Bezier patches */

        while( iev LT s )
        {
            jq = jq + 1;

            N_SFuncGetKnots( sfnA[iq][jq], &fq, &UQ, &VQ );

            if( jq LT nsv - 1 )
                N_SrfFuncCntrlVal( sfnA[iq][jq + 1], &i, &j, &nq );

            /* Get knot multiplicity */

            i = iev;

            while( iev LT s AND VP[iev]EQ VP[iev + 1] )
                iev++;
            mlv = iev - i + 1;
            rv = q - mlv;

            /* Insert the knot */

            if( mlv LT q )
            {
                num = VP[iev] - VP[isv];

                for ( i = q; i > mlv; i-- )
                {
                    vals[i - mlv - 1] = num / (VP[isv + i] - VP[isv]);
                    omvs[i - mlv - 1] = 1.0 - vals[i - mlv - 1];
                }

                for ( i = 1; i <= rv; i++ )
                {
                    sv = mlv + i;
                    save = rv - i;

                    for ( j = q; j >= sv; j-- )
                    {
                        for ( k = 0; k <= p; k++ )
                        {
                            fq[k][j] = vals[j - sv] * fq[k][j] + omvs[j - sv] * fq[k][j - 1];
                        }
                    }

                    if( iev LT s )
                    {
                        for ( k = 0; k <= p; k++ )
                            nq[k][save] = fq[k][q];
                    }
                }
            }

            /* Get knot vectors */

            for ( i = 0; i <= p; i++ )
            {
                UQ[i] = UP[isu];
                UQ[i + p + 1] = UP[ieu];
            }

            for ( j = 0; j <= q; j++ )
            {
                VQ[j] = VP[isv];
                VQ[j + q + 1] = VP[iev];
            }

            /* Patch completed - prepare for next Bezier patch */

            if( iev LT s )
            {
                for ( i = rv; i <= q; i++ )
                {
                    for ( k = 0; k <= p; k++ )
                        nq[k][i] = fb[k][iev - q + i];
                }
            }

            isv = iev;
            iev = iev + 1;
        }

        /* Bezier strip decomposed - prepare for next strip */

        if( ieu LT r )
        {
            for ( i = ru; i <= p; i++ )
            {
                for ( l = 0; l <= m; l++ )
                    nb[i][l] = fp[ieu - p + i][l];
            }
        }

        isu = ieu;
        ieu = ieu + 1;
        tmp = fb;
        fb = nb;
        nb = tmp;
    }

    *kk = nsu - 1;
    *ll = nsv - 1;
    *sfnQ = sfnA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncDecompose */

/************************************/
/* advanced NL_SFUN Symbolic operators */
/************************************/

/**********************************************************************/
/* N_SrfFuncMultiplyConstant: Product of constant and surface function                 */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a constant 
     and a surface function. A typical calling example is:

       NL_REAL  alpha; 
       NL_SFUN  sfn;
       ...
       (define sfn and alpha);
       ...
       N_SrfFuncMultiplyConstant(alpha,&sfn);

     THE  PRODUCT  IS  COMPUTED IN-PLACE, I.E. THE ORIGINAL NL_FUNCTION IS 
     DESTROYED.


   ACCESS:
   
     alpha , input  ,  Scalar
     sfn   , in/out ,  Product of alpha and sfn


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfFuncMultiplyConstant( NL_REAL alpha, NL_SFUN *sfn )
{

    NL_INDEX i, j, n, m;

    NL_REAL ** fuv;

    /* Get locals */

    N_SrfFuncCntrlVal( sfn, &n, &m, &fuv );

    /* Compute product */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            fuv[i][j] = alpha * fuv[i][j];
    }
} /* end N_SrfFuncMultiplyConstant */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_SrfFuncMultiplySrfFunc: Compute the product of two bivariate B-spline functions  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of two surface
     functions. The knot  vectors of the  functions are  rescaled to the
     unit span [0,1], however, the input  functions do not change either 
     parametrically or geometrically. A typical calling example is:

       NL_SFUN    sfnF, sfnG, sfnH;
       NL_STACKS  SO;
       ...
       (define sfnF and sfnG);
       ...
       N_SFuncInitArrays(&sfnH);
       N_SrfFuncMultiplySrfFunc(&sfnF,&sfnG,&sfnH,&SO);

     If memory is  available, sfnH  is not  initialized and  the routine
     assumes  that memory  allocation has  been done. However, it checks  
     for the proper  amount by looking at the  highest indexes in sfnH's  
     knot  vector and control  value objects. THE  STORAGE OF THE OUTPUT 
     NL_FUNCTION IS COMPACTED, I.E. THE MEMORY PASSED IN IS DESTROYED.


   ACCESS:
   
     sfnF , in/out ,  Function f (ITS KNOT NL_VECTOR IS RESCALED)
     sfnG , in/out ,  Function g (ITS KNOT NL_VECTOR IS RESCALED)
     sfnH , output ,  Product function f*g
     SO   , input  ,  sfnH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncMultiplySrfFunc( NL_SFUN *sfnF, NL_SFUN *sfnG, NL_SFUN *sfnH, NL_STACKS *SO )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncMultiplySrfFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX *mu, *mv, i, j, k, l, nf, mf, rf, sf, ng, mg, rg, sg, nh, mh, rh, sh, nfr, nfs, ngr, ngs, mfx, mfy, mgx, mgy, af, ag, bf, bg, mxfb, myfb, mxgb, mygb, ixfr, iyfs, ixgr, iygs, iu, jv, mlf, mlg, mi, ih, jh, kf, lf, kg, lg, kh, lh;

    NL_DEGREE pf, qf, pg, qg, ph, qh;

    NL_REAL ** f, ** g, ** h, ** fb, ** gb, ** hb, *RF, *SF, *RG, *SG, *RH, *SH, *XF, *YF, *XG, *YG, *XFB, *YFB, *XGB, *YGB, *u, *v;

    NL_KNOTVECTOR *kfr, *kfs, *kgr, *kgs, *kfx, *kfy, *kgx, *kgy, *kfrr, *kfsr, *kgrr, *kgsr, *kfrb, *kfsb, *kgrb, *kgsb;

    NL_SFUN sfnFR, sfnGR;

    NL_RMATRIX pmr, pms;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check degrees */

    N_SFuncGetComponents( sfnF, &nf, &mf, &f, &pf, &qf, &rf, &sf, &RF, &SF );
    N_SFuncGetComponents( sfnG, &ng, &mg, &g, &pg, &qg, &rg, &sg, &RG, &SG );

    if( pf EQ 0 AND nf NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    if( qf EQ 0 AND mf NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    if( pg EQ 0 AND ng NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    if( qg EQ 0 AND mg NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    ph = pf + pg;
    qh = qf + qg;

    if( ph GT NL_DMAX OR qh GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Merge knots */

    N_SFuncGetKnotVectors( sfnF, &kfr, &kfs );
    N_SFuncGetKnotVectors( sfnG, &kgr, &kgs );

    N_BasisGetSpanCount( kfr, pf, &nfr );
    N_BasisGetSpanCount( kfs, qf, &nfs );
    N_BasisGetSpanCount( kgr, pg, &ngr );
    N_BasisGetSpanCount( kgs, qg, &ngs );

    kfx = N_AllocKnotVectorAndArray( ngr, &SL );

    if( kfx EQ NULL )
        NL_QUIT;

    kfy = N_AllocKnotVectorAndArray( ngs, &SL );

    if( kfy EQ NULL )
        NL_QUIT;

    kgx = N_AllocKnotVectorAndArray( nfr, &SL );

    if( kgx EQ NULL )
        NL_QUIT;

    kgy = N_AllocKnotVectorAndArray( nfs, &SL );

    if( kgy EQ NULL )
        NL_QUIT;

    N_MakeKnotsCompatible( kfr, kgr, pf, pg, kfx, kgx );
    N_MakeKnotsCompatible( kfs, kgs, qf, qg, kfy, kgy );

    /* Refine knot vectors */

    N_KnotVectorGetKnots( kfx, &mfx, &XF );
    N_KnotVectorGetKnots( kfy, &mfy, &YF );
    N_KnotVectorGetKnots( kgx, &mgx, &XG );
    N_KnotVectorGetKnots( kgy, &mgy, &YG );

    if( mfx GE 0 )
    {
        kfrr = N_AllocKnotVectorAndArray( rf + mfx + 1, &SL );

        if( kfrr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kfr, pf, kfx, kfrr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kfrr, &rf, &RF );
    }
    else
    {
        N_KnotVectorGetKnots( kfr, &rf, &RF );
    }

    if( mfy GE 0 )
    {
        kfsr = N_AllocKnotVectorAndArray( sf + mfy + 1, &SL );

        if( kfsr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kfs, qf, kfy, kfsr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kfsr, &sf, &SF );
    }
    else
    {
        N_KnotVectorGetKnots( kfs, &sf, &SF );
    }

    if( mgx GE 0 )
    {
        kgrr = N_AllocKnotVectorAndArray( rg + mgx + 1, &SL );

        if( kgrr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kgr, pg, kgx, kgrr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kgrr, &rg, &RG );
    }
    else
    {
        N_KnotVectorGetKnots( kgr, &rg, &RG );
    }

    if( mgy GE 0 )
    {
        kgsr = N_AllocKnotVectorAndArray( sg + mgy + 1, &SL );

        if( kgsr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kgs, qg, kgy, kgsr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kgsr, &sg, &SG );
    }
    else
    {
        N_KnotVectorGetKnots( kgs, &sg, &SG );
    }

    /**************************************/
    /* Get knot vectors for decomposition */
    /**************************************/

    u = N_AllocReal1dArray( nfr + ngr, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( nfs + ngs, &SL );

    if( v EQ NULL )
        NL_QUIT;

    mu = N_AllocInt1dArray( nfr + ngr, &SL );

    if( mu EQ NULL )
        NL_QUIT;

    mv = N_AllocInt1dArray( nfs + ngs, &SL );

    if( mv EQ NULL )
        NL_QUIT;

    kfrb = N_AllocKnotVectorAndArray( (nfr + ngr) * pf, &SL );

    if( kfrb EQ NULL )
        NL_QUIT;

    kfsb = N_AllocKnotVectorAndArray( (nfs + ngs) * qf, &SL );

    if( kfsb EQ NULL )
        NL_QUIT;

    kgrb = N_AllocKnotVectorAndArray( (nfr + ngr) * pg, &SL );

    if( kgrb EQ NULL )
        NL_QUIT;

    kgsb = N_AllocKnotVectorAndArray( (nfs + ngs) * qg, &SL );

    if( kgsb EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( kfrb, &mxfb, &XFB );
    N_KnotVectorGetKnots( kfsb, &myfb, &YFB );
    N_KnotVectorGetKnots( kgrb, &mxgb, &XGB );
    N_KnotVectorGetKnots( kgsb, &mygb, &YGB );

    /* Get U knot vector */

    ixfr = 0;
    ixgr = 0;
    af = pf + 1;
    ag = pg + 1;
    mxfb = -1;
    mxgb = -1;
    iu = -1;

    while( af LT rf - pf AND ag LT rg - pg )
    {
        i = af;

        while( RF[af]EQ RF[af + 1] )
            af++;
        mlf = af - i + 1;

        j = ag;

        while( RG[ag]EQ RG[ag + 1] )
            ag++;
        mlg = ag - j + 1;

        /* Check multiplicities of internal knots */

        if( mlf GT pf OR mlg GT pg )
            NL_ERROR( NL_KNT_ERR );

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mlf EQ 1 AND ixfr LE mfx )
        {
            if( fabs( RF[af] - XF[ixfr] )LT NL_PTOL )
            {
                mlf = 0;
                ixfr++;
            }
        }

        if( mlg EQ 1 AND ixgr LE mgx )
        {
            if( fabs( RG[ag] - XG[ixgr] )LT NL_PTOL )
            {
                mlg = 0;
                ixgr++;
            }
        }

        if( mlg EQ 0 )
            mi = pg + mlf;

        else if( mlf EQ 0 )
            mi = pf + mlg;

        else
            mi = NL_MAX( pg + mlf, pf + mlg );

        for ( i = 1; i <= pf - mlf; i++ )
            XFB[++mxfb] = RF[af];

        for ( j = 1; j <= pg - mlg; j++ )
            XGB[++mxgb] = RG[ag];

        iu++;
        u[iu] = RF[af];
        mu[iu] = ph - mi;

        af++;
        ag++;
    }

    /* Get V knot vector */

    iyfs = 0;
    iygs = 0;
    bf = qf + 1;
    bg = qg + 1;
    myfb = -1;
    mygb = -1;
    jv = -1;

    while( bf LT sf - qf AND bg LT sg - qg )
    {
        i = bf;

        while( SF[bf]EQ SF[bf + 1] )
            bf++;
        mlf = bf - i + 1;

        j = bg;

        while( SG[bg]EQ SG[bg + 1] )
            bg++;
        mlg = bg - j + 1;

        /* Check multiplicities of internal knots */

        if( mlf GT qf OR mlg GT qg )
            NL_ERROR( NL_KNT_ERR );

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mlf EQ 1 AND iyfs LE mfy )
        {
            if( fabs( SF[bf] - YF[iyfs] )LT NL_PTOL )
            {
                mlf = 0;
                iyfs++;
            }
        }

        if( mlg EQ 1 AND iygs LE mgy )
        {
            if( fabs( SG[bg] - YG[iygs] )LT NL_PTOL )
            {
                mlg = 0;
                iygs++;
            }
        }

        if( mlg EQ 0 )
            mi = qg + mlf;

        else if( mlf EQ 0 )
            mi = qf + mlg;

        else
            mi = NL_MAX( qg + mlf, qf + mlg );

        for ( i = 1; i <= qf - mlf; i++ )
            YFB[++myfb] = SF[bf];

        for ( j = 1; j <= qg - mlg; j++ )
            YGB[++mygb] = SG[bg];

        jv++;
        v[jv] = SF[bf];
        mv[jv] = qh - mi;

        bf++;
        bg++;
    }

    N_SetKnotIndex( kfrb, mxfb );
    N_SetKnotIndex( kfsb, myfb );
    N_SetKnotIndex( kgrb, mxgb );
    N_SetKnotIndex( kgsb, mygb );

    /* Refine functions */

    N_SFuncInitArrays( &sfnFR );
    error = N_SrfFuncCopy( sfnF, &sfnFR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &sfnGR );
    error = N_SrfFuncCopy( sfnG, &sfnGR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( mxfb GE 0 )
    {
        error = N_SrfFuncRefine( &sfnFR, kfrb, NL_UDIR, &sfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( myfb GE 0 )
    {
        error = N_SrfFuncRefine( &sfnFR, kfsb, NL_VDIR, &sfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( mxgb GE 0 )
    {
        error = N_SrfFuncRefine( &sfnGR, kgrb, NL_UDIR, &sfnGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( mygb GE 0 )
    {
        error = N_SrfFuncRefine( &sfnGR, kgsb, NL_VDIR, &sfnGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SFuncGetComponents( &sfnFR, &nf, &mf, &f, &pf, &qf, &rf, &sf, &RF, &SF );
    N_SFuncGetComponents( &sfnGR, &ng, &mg, &g, &pg, &qg, &rg, &sg, &RG, &SG );

    /* See if memory is needed */

    if( pf EQ 0 )
        ih = 1;
    else
        ih = nf / pf;

    if( qf EQ 0 )
        jh = 1;
    else
        jh = mf / qf;

    nh = ih * ph;
    mh = jh * qh;
    rh = nh + ph + 1;
    sh = mh + qh + 1;

    error = N_SFuncSizeArrays( sfnH, nh, mh, ph, qh, rh, sh, rname, SO );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( sfnH, &h, &RH, &SH );

    /* Compute product surface */

    fb = N_AllocReal2dArray( pf, qf, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    gb = N_AllocReal2dArray( pg, qg, &SL );

    if( gb EQ NULL )
        NL_QUIT;

    hb = N_AllocReal2dArray( ph, qh, &SL );

    if( hb EQ NULL )
        NL_QUIT;

    N_InitRealMatrix( &pmr );
    error = N_BezFuncMultiplyBezCrvMatrix( pf, pg, &pmr, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &pms );
    error = N_BezFuncMultiplyBezCrvMatrix( qf, qg, &pms, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i < ih; i++ )
    {
        kf = i * pf;
        kg = i * pg;
        kh = i * ph;

        for ( j = 0; j < jh; j++ )
        {
            lf = j * qf;
            lg = j * qg;
            lh = j * qh;

            for ( k = 0; k <= pf; k++ )
            {
                for ( l = 0; l <= qf; l++ )
                    fb[k][l] = f[kf + k][lf + l];
            }

            for ( k = 0; k <= pg; k++ )
            {
                for ( l = 0; l <= qg; l++ )
                    gb[k][l] = g[kg + k][lg + l];
            }

            error = N_BezSrfFuncMultiply( fb, pf, qf, gb, pg, qg, &pmr, &pms, 0, ph, 0, qh, hb );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = 0; k <= ph; k++ )
            {
                for ( l = 0; l <= qh; l++ )
                    h[kh + k][lh + l] = hb[k][l];
            }
        }
    }

    k = -1;
    af = pf + 1;

    for ( i = 0; i <= ph; i++ )
        RH[++k] = RF[0];

    while( af LT rf - pf )
    {
        while( RF[af]EQ RF[af + 1] )
            af++;

        for ( i = 1; i <= ph; i++ )
            RH[++k] = RF[af];

        af++;
    }

    for ( i = 0; i <= ph; i++ )
        RH[++k] = RF[rf];

    l = -1;
    bf = qf + 1;

    for ( j = 0; j <= qh; j++ )
        SH[++l] = SF[0];

    while( bf LT sf - qf )
    {
        while( SF[bf]EQ SF[bf + 1] )
            bf++;

        for ( j = 1; j <= qh; j++ )
            SH[++l] = SF[bf];

        bf++;
    }

    for ( j = 0; j <= qh; j++ )
        SH[++l] = SF[sf];

    /* Remove knots */

    for ( i = 0; i <= iu; i++ )
    {
        if( mu[i]GT 0 )
        {
            error = N_SrfFuncRemoveKnot( sfnH, u[i], mu[i], NL_UDIR, sfnH, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    for ( j = 0; j <= jv; j++ )
    {
        if( mv[j]GT 0 )
        {
            error = N_SrfFuncRemoveKnot( sfnH, v[j], mv[j], NL_VDIR, sfnH, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Compact output function */

    N_SFuncGetArraySizes( sfnH, &i, &j, &k, &l );

    if( i LT nh OR j LT mh )
    {
        error = N_SrfFuncCompact( sfnH, SO );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncMultiplySrfFunc */


/**********************************************************************/
/* N_SrfFuncMultiplySrf: Compute the product of bivariate function and surface    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators routine  computes the product of a surface
     function and a surface. The  knot vectors are  rescaled to the unit 
     span  [0,1],  however, the  input  entities  do not  change  either 
     parametrically or geometrically. A typical calling example is:

       NL_SFUN     sfnF;
       NL_SURFACE  surG, surH;
       NL_STACKS   SO;
       ...
       (define sfnF and surG);
       ...
       N_SrfInitArrays(&surH);
       N_SrfFuncMultiplySrf(&sfnF,&surG,&surH,&SO);

     If memory is  available, surH  is not  initialized and  the routine
     assumes  that memory  allocation has  been done. However, it checks  
     for the proper  amount by looking at the  highest indexes in surH's  
     knot  vector and  control net  objects. THE  STORAGE OF  THE OUTPUT 
     NL_SURFACE IS COMPACTED, I.E. THE MEMORY PASSED IN IS DESTROYED.


   ACCESS:
   
     sfnF , in/out ,  Function f (ITS KNOT NL_VECTOR IS RESCALED)
     surG , in/out ,  Surface S (ITS KNOT NL_VECTOR IS RESCALED)
     surH , output ,  Product surface f*S
     SO   , input  ,  surH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncMultiplySrf( NL_SFUN *sfnF, NL_SURFACE *surG, NL_SURFACE *surH, NL_STACKS *SO )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncMultiplySrf");

    NL_FLAG error = NL_NO;

    NL_INDEX *mu, *mv, i, j, k, l, nf, mf, rf, sf, ng, mg, rg, sg, nh, mh, rh, sh, nfr, nfs, ngr, ngs, mfx, mfy, mgx, mgy, af, ag, bf, bg, mxfb, myfb, mxgb, mygb, ixfr, iyfs, ixgr, iygs, iu, jv, mlf, mlg, mi, ih, jh, kf, lf, kg, lg, kh, lh;

    NL_DEGREE pf, qf, pg, qg, ph, qh;

    NL_REAL ** f, ** fb, *RF, *SF, *RG, *SG, *RH, *SH, *XF, *YF, *XG, *YG, *XFB, *YFB, *XGB, *YGB, *u, *v;

    NL_CPOINT ** Gw, ** Hw, ** GBw, ** HBw;

    NL_KNOTVECTOR *kfr, *kfs, *kgr, *kgs, *kfx, *kfy, *kgx, *kgy, *kfrr, *kfsr, *kgrr, *kgsr, *kfrb, *kfsb, *kgrb, *kgsb;

    NL_SFUN sfnFR;

    NL_SURFACE surGR;

    NL_RMATRIX pmr, pms;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check degrees */

    N_SFuncGetComponents( sfnF, &nf, &mf, &f, &pf, &qf, &rf, &sf, &RF, &SF );
    N_SrfGetCPtsDegreesAndKnots( surG, &ng, &mg, &Gw, &pg, &qg, &rg, &sg, &RG, &SG );

    if( pf EQ 0 AND nf NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    if( qf EQ 0 AND mf NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    if( pg EQ 0 AND ng NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    if( qg EQ 0 AND mg NEQ 0 )
        NL_ERROR( NL_INP_ERR );

    ph = pf + pg;
    qh = qf + qg;

    if( ph GT NL_DMAX OR qh GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Merge knots */

    N_SFuncGetKnotVectors( sfnF, &kfr, &kfs );
    N_SrfGetKnotVectors( surG, &kgr, &kgs );

    N_BasisGetSpanCount( kfr, pf, &nfr );
    N_BasisGetSpanCount( kfs, qf, &nfs );
    N_BasisGetSpanCount( kgr, pg, &ngr );
    N_BasisGetSpanCount( kgs, qg, &ngs );

    kfx = N_AllocKnotVectorAndArray( ngr, &SL );

    if( kfx EQ NULL )
        NL_QUIT;

    kfy = N_AllocKnotVectorAndArray( ngs, &SL );

    if( kfy EQ NULL )
        NL_QUIT;

    kgx = N_AllocKnotVectorAndArray( nfr, &SL );

    if( kgx EQ NULL )
        NL_QUIT;

    kgy = N_AllocKnotVectorAndArray( nfs, &SL );

    if( kgy EQ NULL )
        NL_QUIT;

    N_MakeKnotsCompatible( kfr, kgr, pf, pg, kfx, kgx );
    N_MakeKnotsCompatible( kfs, kgs, qf, qg, kfy, kgy );

    /* Refine knot vectors */

    N_KnotVectorGetKnots( kfx, &mfx, &XF );
    N_KnotVectorGetKnots( kfy, &mfy, &YF );
    N_KnotVectorGetKnots( kgx, &mgx, &XG );
    N_KnotVectorGetKnots( kgy, &mgy, &YG );

    if( mfx GE 0 )
    {
        kfrr = N_AllocKnotVectorAndArray( rf + mfx + 1, &SL );

        if( kfrr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kfr, pf, kfx, kfrr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kfrr, &rf, &RF );
    }
    else
    {
        N_KnotVectorGetKnots( kfr, &rf, &RF );
    }

    if( mfy GE 0 )
    {
        kfsr = N_AllocKnotVectorAndArray( sf + mfy + 1, &SL );

        if( kfsr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kfs, qf, kfy, kfsr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kfsr, &sf, &SF );
    }
    else
    {
        N_KnotVectorGetKnots( kfs, &sf, &SF );
    }

    if( mgx GE 0 )
    {
        kgrr = N_AllocKnotVectorAndArray( rg + mgx + 1, &SL );

        if( kgrr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kgr, pg, kgx, kgrr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kgrr, &rg, &RG );
    }
    else
    {
        N_KnotVectorGetKnots( kgr, &rg, &RG );
    }

    if( mgy GE 0 )
    {
        kgsr = N_AllocKnotVectorAndArray( sg + mgy + 1, &SL );

        if( kgsr EQ NULL )
            NL_QUIT;

        error = N_BasisInsertKnots( kgs, qg, kgy, kgsr );

        if( error EQ NL_YES )
            NL_OUT;

        N_KnotVectorGetKnots( kgsr, &sg, &SG );
    }
    else
    {
        N_KnotVectorGetKnots( kgs, &sg, &SG );
    }

    /**************************************/
    /* Get knot vectors for decomposition */
    /**************************************/

    u = N_AllocReal1dArray( nfr + ngr, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( nfs + ngs, &SL );

    if( v EQ NULL )
        NL_QUIT;

    mu = N_AllocInt1dArray( nfr + ngr, &SL );

    if( mu EQ NULL )
        NL_QUIT;

    mv = N_AllocInt1dArray( nfs + ngs, &SL );

    if( mv EQ NULL )
        NL_QUIT;

    kfrb = N_AllocKnotVectorAndArray( (nfr + ngr) * pf, &SL );

    if( kfrb EQ NULL )
        NL_QUIT;

    kfsb = N_AllocKnotVectorAndArray( (nfs + ngs) * qf, &SL );

    if( kfsb EQ NULL )
        NL_QUIT;

    kgrb = N_AllocKnotVectorAndArray( (nfr + ngr) * pg, &SL );

    if( kgrb EQ NULL )
        NL_QUIT;

    kgsb = N_AllocKnotVectorAndArray( (nfs + ngs) * qg, &SL );

    if( kgsb EQ NULL )
        NL_QUIT;

    N_KnotVectorGetKnots( kfrb, &mxfb, &XFB );
    N_KnotVectorGetKnots( kfsb, &myfb, &YFB );
    N_KnotVectorGetKnots( kgrb, &mxgb, &XGB );
    N_KnotVectorGetKnots( kgsb, &mygb, &YGB );

    /* Get U knot vector */

    ixfr = 0;
    ixgr = 0;
    af = pf + 1;
    ag = pg + 1;
    mxfb = -1;
    mxgb = -1;
    iu = -1;

    while( af LT rf - pf AND ag LT rg - pg )
    {
        i = af;

        while( RF[af]EQ RF[af + 1] )
            af++;
        mlf = af - i + 1;

        j = ag;

        while( RG[ag]EQ RG[ag + 1] )
            ag++;
        mlg = ag - j + 1;

        /* Check multiplicities of internal knots */

        if( mlf GT pf OR mlg GT pg )
            NL_ERROR( NL_KNT_ERR );

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mlf EQ 1 AND ixfr LE mfx )
        {
            if( fabs( RF[af] - XF[ixfr] )LT NL_PTOL )
            {
                mlf = 0;
                ixfr++;
            }
        }

        if( mlg EQ 1 AND ixgr LE mgx )
        {
            if( fabs( RG[ag] - XG[ixgr] )LT NL_PTOL )
            {
                mlg = 0;
                ixgr++;
            }
        }

        if( mlg EQ 0 )
            mi = pg + mlf;

        else if( mlf EQ 0 )
            mi = pf + mlg;

        else
            mi = NL_MAX( pg + mlf, pf + mlg );

        for ( i = 1; i <= pf - mlf; i++ )
            XFB[++mxfb] = RF[af];

        for ( j = 1; j <= pg - mlg; j++ )
            XGB[++mxgb] = RG[ag];

        iu++;
        u[iu] = RF[af];
        mu[iu] = ph - mi;

        af++;
        ag++;
    }

    /* Get V knot vector */

    iyfs = 0;
    iygs = 0;
    bf = qf + 1;
    bg = qg + 1;
    myfb = -1;
    mygb = -1;
    jv = -1;

    while( bf LT sf - qf AND bg LT sg - qg )
    {
        i = bf;

        while( SF[bf]EQ SF[bf + 1] )
            bf++;
        mlf = bf - i + 1;

        j = bg;

        while( SG[bg]EQ SG[bg + 1] )
            bg++;
        mlg = bg - j + 1;

        /* Check multiplicities of internal knots */

        if( mlf GT qf OR mlg GT qg )
            NL_ERROR( NL_KNT_ERR );

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( mlf EQ 1 AND iyfs LE mfy )
        {
            if( fabs( SF[bf] - YF[iyfs] )LT NL_PTOL )
            {
                mlf = 0;
                iyfs++;
            }
        }

        if( mlg EQ 1 AND iygs LE mgy )
        {
            if( fabs( SG[bg] - YG[iygs] )LT NL_PTOL )
            {
                mlg = 0;
                iygs++;
            }
        }

        if( mlg EQ 0 )
            mi = qg + mlf;

        else if( mlf EQ 0 )
            mi = qf + mlg;

        else
            mi = NL_MAX( qg + mlf, qf + mlg );

        for ( i = 1; i <= qf - mlf; i++ )
            YFB[++myfb] = SF[bf];

        for ( j = 1; j <= qg - mlg; j++ )
            YGB[++mygb] = SG[bg];

        jv++;
        v[jv] = SF[bf];
        mv[jv] = qh - mi;

        bf++;
        bg++;
    }

    N_SetKnotIndex( kfrb, mxfb );
    N_SetKnotIndex( kfsb, myfb );
    N_SetKnotIndex( kgrb, mxgb );
    N_SetKnotIndex( kgsb, mygb );

    /* Refine entities */

    N_SFuncInitArrays( &sfnFR );
    error = N_SrfFuncCopy( sfnF, &sfnFR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &surGR );
    error = N_SrfCopy( surG, &surGR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( mxfb GE 0 )
    {
        error = N_SrfFuncRefine( &sfnFR, kfrb, NL_UDIR, &sfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( myfb GE 0 )
    {
        error = N_SrfFuncRefine( &sfnFR, kfsb, NL_VDIR, &sfnFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( mxgb GE 0 )
    {
        error = N_SrfInsertKnots( &surGR, kgrb, NL_UDIR, &surGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( mygb GE 0 )
    {
        error = N_SrfInsertKnots( &surGR, kgsb, NL_VDIR, &surGR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SFuncGetComponents( &sfnFR, &nf, &mf, &f, &pf, &qf, &rf, &sf, &RF, &SF );
    N_SrfGetCPtsDegreesAndKnots( &surGR, &ng, &mg, &Gw, &pg, &qg, &rg, &sg, &RG, &SG );

    /* See if memory is needed */

    if( pf EQ 0 )
        ih = 1;
    else
        ih = nf / pf;

    if( qf EQ 0 )
        jh = 1;
    else
        jh = mf / qf;

    nh = ih * ph;
    mh = jh * qh;
    rh = nh + ph + 1;
    sh = mh + qh + 1;

    error = N_SrfSizeArrays( surH, nh, mh, ph, qh, rh, sh, rname, SO );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( surH, &Hw, &RH, &SH );

    /* Compute product surface */

    fb = N_AllocReal2dArray( pf, qf, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    GBw = N_AllocCPt2dArray( pg, qg, &SL );

    if( GBw EQ NULL )
        NL_QUIT;

    HBw = N_AllocCPt2dArray( ph, qh, &SL );

    if( HBw EQ NULL )
        NL_QUIT;

    N_InitRealMatrix( &pmr );
    error = N_BezFuncMultiplyBezCrvMatrix( pf, pg, &pmr, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &pms );
    error = N_BezFuncMultiplyBezCrvMatrix( qf, qg, &pms, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i < ih; i++ )
    {
        kf = i * pf;
        kg = i * pg;
        kh = i * ph;

        for ( j = 0; j < jh; j++ )
        {
            lf = j * qf;
            lg = j * qg;
            lh = j * qh;

            for ( k = 0; k <= pf; k++ )
            {
                for ( l = 0; l <= qf; l++ )
                    fb[k][l] = f[kf + k][lf + l];
            }

            for ( k = 0; k <= pg; k++ )
            {
                for ( l = 0; l <= qg; l++ )
                    N_CopyCPt( Gw[kg + k][lg + l], &GBw[k][l] );
            }

            error = N_BezSrfMultiplySrfFunc( fb, pf, qf, GBw, pg, qg, &pmr, &pms, 0, ph, 0, qh, HBw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = 0; k <= ph; k++ )
            {
                for ( l = 0; l <= qh; l++ )
                    N_CopyCPt( HBw[k][l], &Hw[kh + k][lh + l] );
            }
        }
    }

    k = -1;
    af = pf + 1;

    for ( i = 0; i <= ph; i++ )
        RH[++k] = RF[0];

    while( af LT rf - pf )
    {
        while( RF[af]EQ RF[af + 1] )
            af++;

        for ( i = 1; i <= ph; i++ )
            RH[++k] = RF[af];

        af++;
    }

    for ( i = 0; i <= ph; i++ )
        RH[++k] = RF[rf];

    l = -1;
    bf = qf + 1;

    for ( j = 0; j <= qh; j++ )
        SH[++l] = SF[0];

    while( bf LT sf - qf )
    {
        while( SF[bf]EQ SF[bf + 1] )
            bf++;

        for ( j = 1; j <= qh; j++ )
            SH[++l] = SF[bf];

        bf++;
    }

    for ( j = 0; j <= qh; j++ )
        SH[++l] = SF[sf];

    /* Remove knots */

    for ( i = 0; i <= iu; i++ )
    {
        if( mu[i]GT 0 )
        {
            error = N_SrfRemoveKnotMultiple( surH, u[i], mu[i], NL_UDIR, surH, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    for ( j = 0; j <= jv; j++ )
    {
        if( mv[j]GT 0 )
        {
            error = N_SrfRemoveKnotMultiple( surH, v[j], mv[j], NL_VDIR, surH, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* Compact output surface */

    N_SrfGetArraySizes( surH, &i, &j, &k, &l );

    if( i LT nh OR j LT mh )
    {
        error = N_SrfCompress( surH, SO );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncMultiplySrf */

#if NLIB_UNUSED

/**********************************************************************/
/* N_SRFFUNCSUMDIFFSRFFUNC: Sum/difference of two surface functions                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic operators  routine computes the sum/difference  of 
     two surface functions. A typical calling example is:

       NL_SFUN    sfnP, sfnQ, sfnR;
       NL_STACKS  SG;
       ...
       (define sfnP and sfnQ);
       ...
       N_SFuncInitArrays(&sfnR);
       N_SrfFuncSumDiffSrfFunc(&sfnP,&sfnQ,NL_PLUS,NL_YES,&sfnR,&SG);

     If memory is available, sfnR is not  initialized and  the routine 
     assumes that memory  allocation has been done. However, it checks 
     for the proper amount by looking at the highest indexes in sfnR's
     knot vector and control value objects. IF THE FUNCTIONS ARE KNOWN
     TO BE COMPATIBLE, SET THE cmp NL_FLAG "NL_YES" WHICH WILL SKIP A LOT OF
     UNNECESSARY COMPUTATIONS. THIS IS USEFUL IF SYMBOLIC COMPUTATIONS 
     ARE TO BE PERFORMED ON THE  INDIVIDUAL COORDINATE  FUNCTIONS THAT 
     ARE  KNOWN TO BE  COMPATIBLE. BECAUSE OF  COMPATIBILITY, THE KNOT
     VECTORS MUST BE  SCALED TO A  COMMON NL_INTERVAL  ([0,1]). THIS DOES 
     NOT CHANGE THE FUNCTIONS EITHER PARAMETRICALLY OR GEOMETRICALLY.


   ACCESS:
   
     sfnP , in/out ,  Surface function
     sfnQ , in/out ,  Surface function
     opr  , input  ,  Operator flag:
                        NL_PLUS : sum
                        NL_MINUS: difference
     cmp  , input  ,  Flag:
                        NL_YES: functions are compatible
                        NL_NO : make functions compatible
     sfnR , output ,  sfnP +- sfnQ
     SG   , input  ,  sfnR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncSumDiffSrfFunc( NL_SFUN *sfnP, NL_SFUN *sfnQ, NL_FLAG opr, NL_FLAG cmp, NL_SFUN *sfnR, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncSumDiffSrfFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s, nup, nvp, nuq, nvq, mxp, myp, mxq, myq;

    NL_DEGREE pp, qp, pq, qq;

    NL_REAL ** fp, ** fq, ** fr, *UP, *VP, *UQ, *VQ, *UR, *VR, *A;

    NL_SFUN sfnPW, sfnQW;

    NL_KNOTVECTOR *kup, *kvp, *kuq, *kvq, *kxp, *kyp, *kxq, *kyq;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute sum/difference */

    if( cmp EQ NL_YES )
    {
        /* Functions are compatible -> get memory for output function */

        N_SFuncGetComponents( sfnP, &n, &m, &fp, &pp, &qp, &r, &s, &UP, &VP );
        N_SFuncGetComponents( sfnQ, &n, &m, &fq, &pq, &qq, &r, &s, &UQ, &VQ );

        error = N_SFuncSizeArrays( sfnR, n, m, pp, qp, r, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnR, &fr, &UR, &VR );
    }
    else
    {
        /* Make functions compatible using auxiliary functions */

        N_SFuncInitArrays( &sfnPW );
        error = N_SrfFuncCopy( sfnP, &sfnPW, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncInitArrays( &sfnQW );
        error = N_SrfFuncCopy( sfnQ, &sfnQW, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get local notation */

        N_SFuncGetComponents( &sfnPW, &n, &m, &fp, &pp, &qp, &r, &s, &UP, &VP );
        N_SFuncGetComponents( &sfnQW, &n, &m, &fq, &pq, &qq, &r, &s, &UQ, &VQ );
        N_SFuncGetKnotVectors( &sfnPW, &kup, &kvp );
        N_SFuncGetKnotVectors( &sfnQW, &kuq, &kvq );

        /* Degree elevate */

        if( pp NEQ pq )
        {
            if( pp LT pq )
            {
                error = N_SrfFuncDegreeElevate( &sfnPW, pq - pp, NL_UDIR, &sfnPW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SFuncGetComponents( &sfnPW, &n, &m, &fp, &pp, &qp, &r, &s, &UP, &VP );
                N_SFuncGetKnotVectors( &sfnPW, &kup, &kvp );
            }
            else
            {
                error = N_SrfFuncDegreeElevate( &sfnQW, pp - pq, NL_UDIR, &sfnQW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SFuncGetComponents( &sfnQW, &n, &m, &fq, &pq, &qq, &r, &s, &UQ, &VQ );
                N_SFuncGetKnotVectors( &sfnQW, &kuq, &kvq );
            }
        }

        if( qp NEQ qq )
        {
            if( qp LT qq )
            {
                error = N_SrfFuncDegreeElevate( &sfnPW, qq - qp, NL_VDIR, &sfnPW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SFuncGetComponents( &sfnPW, &n, &m, &fp, &pp, &qp, &r, &s, &UP, &VP );
                N_SFuncGetKnotVectors( &sfnPW, &kup, &kvp );
            }
            else
            {
                error = N_SrfFuncDegreeElevate( &sfnQW, qp - qq, NL_VDIR, &sfnQW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SFuncGetComponents( &sfnQW, &n, &m, &fq, &pq, &qq, &r, &s, &UQ, &VQ );
                N_SFuncGetKnotVectors( &sfnQW, &kuq, &kvq );
            }
        }

        /* Refine functions */

        N_BasisGetSpanCount( kup, pp, &nup );
        N_BasisGetSpanCount( kvp, qp, &nvp );
        N_BasisGetSpanCount( kuq, pq, &nuq );
        N_BasisGetSpanCount( kvq, qq, &nvq );

        kxp = N_AllocKnotVectorAndArray( nuq * pq, &SL );

        if( kxp EQ NULL )
            NL_QUIT;

        kyp = N_AllocKnotVectorAndArray( nvq * qq, &SL );

        if( kyp EQ NULL )
            NL_QUIT;

        kxq = N_AllocKnotVectorAndArray( nup * pp, &SL );

        if( kxq EQ NULL )
            NL_QUIT;

        kyq = N_AllocKnotVectorAndArray( nvp * qp, &SL );

        if( kyq EQ NULL )
            NL_QUIT;

        N_GetCompatibleKnotArrayMult( kup, kuq, pp, kxp, kxq );
        N_GetCompatibleKnotArrayMult( kvp, kvq, qp, kyp, kyq );

        N_KnotVectorGetKnots( kxp, &mxp, &A );
        N_KnotVectorGetKnots( kyp, &myp, &A );
        N_KnotVectorGetKnots( kxq, &mxq, &A );
        N_KnotVectorGetKnots( kyq, &myq, &A );

        if( mxp GE 0 )
        {
            error = N_SrfFuncRefine( &sfnPW, kxp, NL_UDIR, &sfnPW, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SFuncGetComponents( &sfnPW, &n, &m, &fp, &pp, &qp, &r, &s, &UP, &VP );
        }

        if( myp GE 0 )
        {
            error = N_SrfFuncRefine( &sfnPW, kyp, NL_VDIR, &sfnPW, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SFuncGetComponents( &sfnPW, &n, &m, &fp, &pp, &qp, &r, &s, &UP, &VP );
        }

        if( mxq GE 0 )
        {
            error = N_SrfFuncRefine( &sfnQW, kxq, NL_UDIR, &sfnQW, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SFuncGetComponents( &sfnQW, &n, &m, &fq, &pq, &qq, &r, &s, &UQ, &VQ );
        }

        if( myq GE 0 )
        {
            error = N_SrfFuncRefine( &sfnQW, kyq, NL_VDIR, &sfnQW, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SFuncGetComponents( &sfnQW, &n, &m, &fq, &pq, &qq, &r, &s, &UQ, &VQ );
        }

        /* Check memory of output function */

        error = N_SFuncSizeArrays( sfnR, n, m, pp, qp, r, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( sfnR, &fr, &UR, &VR );
    }

    /* Compute output function */

    switch( opr )
    {
        case NL_PLUS:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                    fr[i][j] = fp[i][j] + fq[i][j];
            }
            break;

        case NL_MINUS:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                    fr[i][j] = fp[i][j] - fq[i][j];
            }
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    for ( i = 0; i <= r; i++ )
        UR[i] = UP[i];

    for ( j = 0; j <= s; j++ )
        VR[j] = VP[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncSumDiffSrfFunc */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_SrfFuncDerivFunc: Derivative function of bivariate B-spline function       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the derivative function of 
     a bivariate  B-spline function. The  maximum derivative  allowed is 
     equal to the degree. A typical calling example is:

       NL_SFUN    sfn, der;
       NL_INDEX   du, dv;
       NL_STACKS  SG;
       ...
       (define sfn; get highest derivative indexes du and dv);
       ...
       N_SFuncInitArrays(&der);
       N_SrfFuncDerivFunc(&sfn,du,dv,&der,&SG);


   ACCESS:
   
     sfn   , input  ,  Bivariate B-spline function
     du,dv , input  ,  Highest  derivatives  required (MUST BE LESS THAN 
                       OR EQUAL TO THE DEGREES)
     der   , output ,  Derivative function
     SG    , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncDerivFunc( NL_SFUN *sfn, NL_INDEX du, NL_INDEX dv, NL_SFUN *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncDerivFunc");

    NL_FLAG error = NL_NO, uder = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, nd, md;

    NL_DEGREE p, q, pd, qd;

    NL_REAL ** f, ** fd, ** fu, ** fv, *U, *V, *UD, *VD, alf, bet;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_SFuncGetComponents( sfn, &n, &m, &f, &p, &q, &r, &s, &U, &V );

    if( du LT 0 OR du GT p )
        NL_ERROR( NL_INP_ERR );

    if( dv LT 0 OR dv GT q )
        NL_ERROR( NL_INP_ERR );

    /* No derivatives required */

    if( du EQ 0 AND dv EQ 0 )
    {
        error = N_SrfFuncCopy( sfn, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* See if memory is needed */

    nd = n - du;
    md = m - dv;
    pd = (NL_DEGREE)( p - du );
    qd = (NL_DEGREE)( q - dv );

    error = N_SFuncSizeArrays( der, nd, md, pd, qd, nd + pd + 1, md + qd + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( der, &fd, &UD, &VD );

    /* Initialize control values */

    fu = N_AllocReal2dArray( n, m, &SL );

    if( fu EQ NULL )
        NL_QUIT;

    fv = N_AllocReal2dArray( n, m, &SL );

    if( fv EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            fu[i][j] = f[i][j];
            fv[i][j] = f[i][j];
        }
    }

    /* Compute control values of derivative surface */

    nd = n;
    md = m;
    k = 1;
    l = 1;

    while( k LE du OR l LE dv )
    {
        if( k LE du )
        {
            nd--;
            uder = NL_YES;

            for ( i = 0; i <= nd; i++ )
            {
                if( U[i + p + 1]EQ U[i + k] )
                    NL_ERROR( NL_DER_ERR );

                if( N_FloatOpIsBad( ( (NL_REAL)p - (NL_REAL)k + 1.0 ), U[i + p + 1] - U[i + k], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = ((NL_REAL)p - (NL_REAL)k + 1.0) / (U[i + p + 1] - U[i + k]);

                for ( j = 0; j <= md; j++ )
                {
                    if( l LE dv + 1 )
                        fu[i][j] = alf * (fv[i + 1][j] - fv[i][j]);
                    else
                        fu[i][j] = alf * (fu[i + 1][j] - fu[i][j]);
                }
            }
        }

        if( l LE dv )
        {
            md--;
            uder = NL_NO;

            for ( j = 0; j <= md; j++ )
            {
                if( V[j + q + 1]EQ V[j + l] )
                    NL_ERROR( NL_DER_ERR );

                if( N_FloatOpIsBad( ( (NL_REAL)q - (NL_REAL)l + 1.0 ), V[j + q + 1] - V[j + l], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                bet = ((NL_REAL)q - (NL_REAL)l + 1.0) / (V[j + q + 1] - V[j + l]);

                for ( i = 0; i <= nd; i++ )
                {
                    if( k LE du )
                        fv[i][j] = bet * (fu[i][j + 1] - fu[i][j]);
                    else
                        fv[i][j] = bet * (fv[i][j + 1] - fv[i][j]);
                }
            }
        }

        k++;
        l++;
    }

    if( uder EQ NL_YES )
    {
        for ( i = 0; i <= nd; i++ )
        {
            for ( j = 0; j <= md; j++ )
                fd[i][j] = fu[i][j];
        }
    }
    else
    {
        for ( i = 0; i <= nd; i++ )
        {
            for ( j = 0; j <= md; j++ )
                fd[i][j] = fv[i][j];
        }
    }

    /* Compute knot vectors */

    k = -1;

    for ( i = 0; i <= p - du; i++ )
        UD[++k] = U[0];

    for ( i = p + 1; i <= n; i++ )
        UD[++k] = U[i];

    for ( i = 0; i <= p - du; i++ )
        UD[++k] = U[r];

    l = -1;

    for ( j = 0; j <= q - dv; j++ )
        VD[++l] = V[0];

    for ( j = q + 1; j <= m; j++ )
        VD[++l] = V[j];

    for ( j = 0; j <= q - dv; j++ )
        VD[++l] = V[s];

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncDerivFunc */

#if NLIB_UNUSED

/**********************************************************************/
/* N_SrfFuncFuncDerivFuncAtKnot: Derivative of surface function with respect to a knot    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the  derivative  of  a 
     surface function with respect to a knot. A typical calling example:

       NL_SFUN    sfn, der;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define sfn);
       ...
       N_SFuncInitArrays(&der);
       N_SrfFuncFuncDerivFuncAtKnot(&sfn,k,NL_LEFT,NL_UDIR,&der,&SG);

     Since NLIB  does not  allow  knot  multiplicities  greater than the
     degree, the  function cannot  be  differentiated  with respect to a 
     knot with multiplicity  equal  to  the  degree. The  error  NL_KML_ERR  
     is returned if such a knot is found.


   ACCESS:
   
     sfn  , input  ,  Bivariate function
     k    , input  ,  Index of  knot, i.e.  the derivative  with respect 
                      to t_k is computed (t is either u or v)
     flg  , input  ,  Flag:
                        NL_LEFT : left  derivative. NL_INDEX  k  MUST  SATISFY
                               t_(k) != t_(k-1)
                        NL_RIGHT: right  derivative. NL_INDEX  k  MUST SATISFY
                               t_(k) != t_(k+1)
                      (t is either u or v)
     dir  , input  ,  Flag:
                        NL_UDIR: derivative in u-direction required
                        NL_VDIR: derivative in v-direction required
     der  , output ,  Derivative function
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncFuncDerivFuncAtKnot( NL_SFUN *sfn, NL_INDEX k, NL_FLAG flg, NL_FLAG dir, NL_SFUN *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncFuncDerivFuncAtKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s, mlt;

    NL_DEGREE p, q;

    NL_REAL *UF, *VF, *UD, *VD, ** ft, ** fd, *fac;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_SFuncGetComponents( sfn, &n, &m, &ft, &p, &q, &r, &s, &UF, &VF );

    switch( dir )
    {
        case NL_UDIR:
            if( k LE p OR k GE n + 1 )
                NL_ERROR( NL_INP_ERR );

            switch( flg )
            {
                case NL_LEFT:
                    if( UF[k]EQ UF[k - 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( UF[i]EQ UF[i + 1] )
                        i++;
                    mlt = i - k + 1;

                    if( mlt GE p )
                        NL_ERROR( NL_KML_ERR );
                    break;

                case NL_RIGHT:
                    if( UF[k]EQ UF[k + 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( UF[i]EQ UF[i - 1] )
                        i--;
                    mlt = k - i + 1;

                    if( mlt GE p )
                        NL_ERROR( NL_KML_ERR );
                    break;

                default:
                    NL_ERROR( NL_CAL_ERR );
            }
            break;

        case NL_VDIR:
            if( k LE q OR k GE m + 1 )
                NL_ERROR( NL_INP_ERR );

            switch( flg )
            {
                case NL_LEFT:
                    if( VF[k]EQ VF[k - 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( VF[i]EQ VF[i + 1] )
                        i++;
                    mlt = i - k + 1;

                    if( mlt GE q )
                        NL_ERROR( NL_KML_ERR );
                    break;

                case NL_RIGHT:
                    if( VF[k]EQ VF[k + 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( VF[i]EQ VF[i - 1] )
                        i--;
                    mlt = k - i + 1;

                    if( mlt GE q )
                        NL_ERROR( NL_KML_ERR );
                    break;

                default:
                    NL_ERROR( NL_CAL_ERR );
            }
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( dir EQ NL_UDIR )
    {
        error = N_SFuncSizeArrays( der, n + 1, m, p, q, r + 1, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( dir EQ NL_VDIR )
    {
        error = N_SFuncSizeArrays( der, n, m + 1, p, q, r, s + 1, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SFuncGetKnots( der, &fd, &UD, &VD );

    /* Compute the knot vector */

    switch( dir )
    {
        case NL_UDIR:
            switch( flg )
            {
                case NL_LEFT:

                    j = -1;

                    for ( i = 0; i <= k + mlt - 1; i++ )
                        UD[++j] = UF[i];
                    UD[++j] = UF[k];

                    for ( i = k + mlt; i <= r; i++ )
                        UD[++j] = UF[i];
                    break;

                case NL_RIGHT:

                    j = -1;

                    for ( i = 0; i <= k; i++ )
                        UD[++j] = UF[i];
                    UD[++j] = UF[k];

                    for ( i = k + 1; i <= r; i++ )
                        UD[++j] = UF[i];
                    break;
            }

            for ( j = 0; j <= s; j++ )
                VD[j] = VF[j];
            break;

        case NL_VDIR:
            switch( flg )
            {
                case NL_LEFT:

                    i = -1;

                    for ( j = 0; j <= k + mlt - 1; j++ )
                        VD[++i] = VF[j];
                    VD[++i] = VF[k];

                    for ( j = k + mlt; j <= s; j++ )
                        VD[++i] = VF[j];
                    break;

                case NL_RIGHT:

                    i = -1;

                    for ( j = 0; j <= k; j++ )
                        VD[++i] = VF[j];
                    VD[++i] = VF[k];

                    for ( j = k + 1; j <= s; j++ )
                        VD[++i] = VF[j];
                    break;
            }

            for ( i = 0; i <= r; i++ )
                UD[i] = UF[i];
            break;
    }

    /* Compute control points */

    fac = N_AllocReal1dArray( NL_MAX( p, q ), &SL );

    if( fac EQ NULL )
        NL_QUIT;

    if( dir EQ NL_UDIR )
    {
        for ( i = k - p; i <= k; i++ )
            fac[i - k + p] = 1.0 / (UF[i + p] - UF[i]);
    }

    if( dir EQ NL_VDIR )
    {
        for ( j = k - q; j <= k; j++ )
            fac[j - k + q] = 1.0 / (VF[j + q] - VF[j]);
    }

    switch( dir )
    {
        case NL_UDIR:
            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= k - p - 1; i++ )
                    fd[i][j] = 0.0;

                for ( i = k - p; i <= k; i++ )
                {
                    fd[i][j] = fac[i - k + p] * (ft[i - 1][j] - ft[i][j]);
                }

                for ( i = k + 1; i <= n + 1; i++ )
                    fd[i][j] = 0.0;
            }
            break;

        case NL_VDIR:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= k - q - 1; j++ )
                    fd[i][j] = 0.0;

                for ( j = k - q; j <= k; j++ )
                {
                    fd[i][j] = fac[j - k + q] * (ft[i][j - 1] - ft[i][j]);
                }

                for ( j = k + 1; j <= m + 1; j++ )
                    fd[i][j] = 0.0;
            }
            break;
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFuncFuncDerivFuncAtKnot */

/***********************/
/* NL_VFUN Error routines */
/***********************/

/*******************************************************************//**


   DESCRIPTION:

     This error  routine checks  if a volume  function  structure has  
     sufficient  memory to  store control  values and knots. A typical 
     calling example is:

       NL_VFUN    vfn;
       NL_INDEX   mf, nf, of, rk, sk, tk;
       NL_STRING  rname;
       ...
       (define vfn, get nf, mf, of, rk, sk, tk and rname);
       ...
       N_VolumeIsFuncSized(&vfn,mf,nf,of,rk,sk,tk,rname);


   ACCESS:
   
     vfn       , input ,  Surface function
     mf,nf,of  , input ,  Expected highest indexes in control value array
     rk,sk,tk  , input ,  Expected highest indexes in knot vector arrays
     rname     , input ,  Routine name where error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG   N_SrfIsFuncSized */
NL_FLAG N_VolumeIsFuncSized( NL_VFUN *vfn, NL_INDEX mf, NL_INDEX nf, NL_INDEX of, NL_INDEX rk, NL_INDEX sk, NL_INDEX tk, NL_STRING rname )
{

    NL_FLAG error = NL_NO;

    NL_INDEX m, n, o, ir, is, it;

    /* Get local notation */

    N_VFuncGetArraySizes( vfn, &m, &n, &o, &ir, &is, &it );

    /* Check storage */

    if( m LT mf OR n LT nf OR o LT of OR ir LT rk OR is LT sk OR it LT tk )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_VolumeIsFuncSized */

/***************************/
/* NL_VVALUE Utility routines */
/***************************/

/**********************************************************************/
/* N_AllocVValue: Allocate memory for a volume value structure            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a volume
     value  structure. Proper  error check is  performed in case 
     memory allocation fails. A typical calling example is:

       NL_VVALUE  *vvl;
       NL_STACKS  S;
       ...
       vvl = N_AllocVValue(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     vvl  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_VVALUE  *N_AllocSValue */
NL_VVALUE *N_AllocVValue( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocVValue");

    NL_VVALUE *vvl;

    NL_VVLNODE *vvd;

    /* Allocate memory for the structure */

    vvl = (NL_VVALUE *)N_Malloc( sizeof( NL_VVALUE ) );

    if( vvl EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    vvd = (NL_VVLNODE *)N_Malloc( sizeof( NL_VVLNODE ) );

    if( vvd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( vvl );
        vvl = NULL;
        return NULL;
    }

    vvd->ptr = vvl;
    vvd->next = S->vvl;
    S->vvl = vvd;

    /* Exit */

    return vvl;
} /* end N_AllocVValue */

/**********************************************************************/
/* N_AllocVValueAndArray: Allocate memory to store volume value object            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to store a volume value object. 
     Proper error check is performed in case memory allocation fails. 
     A typical calling example is:

       NL_VVALUE  *vvs;
       NL_INDEX   m,n,o;
       NL_STACKS  S;
       ...
       (get m, n, and o);
       ...
       vvs = N_AllocVValueAndArray(m,n,o,&S);


   ACCESS:
   
     m,n,o , input  ,  Highest indexes in control value array
     S     , input  ,  Memory stack pointer


   RETURN CODES:

     vvs  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_VVALUE  *N_AllocSValueAndArray */
NL_VVALUE *N_AllocVValueAndArray( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S )
{

    NL_REAL *** fuvw;

    NL_VVALUE *vvs;

    /* Allocate memory */

    vvs = N_AllocVValue( S );

    if( vvs EQ NULL )
        return NULL;

    fuvw = N_AllocReal3dArray( m, n, o, S );

    if( fuvw EQ NULL )
        return NULL;

    /* Build the object */

    N_VValueFromArray( vvs, fuvw, m, n, o );

    /* Exit */

    return vvs;
} /* end N_AllocVValueAndArray */

/**********************************************************************/
/* N_FreeVValue: Release memory that stores control value structure       */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores  members of a 
     control value structure. Given a control value pointer, the routine
     searches  for the  pointer  on the  memory  stack. It it  is found, 
     memory is deallocated. If not, the  routine does nothing. A typical
     calling example is:

       NL_VVALUE  *vvl;
       NL_STACKS   S;
       ...
       N_FreeVValue(vvl,&S);


   ACCESS:
   
     vvl , input  ,  Control value pointer 
     S   , input  ,  vvl's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeSValue */
NL_VOID N_FreeVValue( NL_VVALUE *vvl, NL_STACKS *S )
{

    NL_VVLNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->vvl NEQ NULL )
    {
        prev = S->vvl;
        curr = S->vvl;

        while( curr NEQ NULL AND curr->ptr NEQ vvl )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->vvl = NULL;
            }
            else /* More than one node */
            {
                S->vvl = S->vvl->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
        }
    }
} /* end N_FreeVValue */

/**********************************************************************/
/* N_VValueFromArray: Make volume function control value structure            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper  pointer  assignments to define a
     volume  function  control  value  object  from a  set of  function 
     values. Memory for the  control value structure is allocated in the  
     calling routine; only the pointer is passed down. A typical calling  
     example is:

       NL_VVALUE  vvl;
       NL_REAL    ***fuvw;
       NL_INDEX   m, n, o;
       ...
       (allocate memory for fuvw);
       ...
       N_VValueFromArray(&vvl,fuvw,n,m);


   ACCESS:
   
     vvl   , in/out ,  Control value structure
     fuvw  , input  ,  Function values
     m,n,o , input  ,  Highest indexes in fuvw


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_SValueFromArray */
NL_VOID N_VValueFromArray( NL_VVALUE *vvl, NL_REAL *** fuvw, NL_INDEX m, NL_INDEX n, NL_INDEX o )
{

    vvl->m = m;
    vvl->n = n;
    vvl->o = o;
    vvl->fuvw = fuvw;
} /* end N_VValueFromArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine makes  proper pointer  assignments to define
     a  volume function  object  from  control  value and  knot vector 
     objects. Memory is allocated in the calling routine; only pointers 
     are passed down. A typical calling example is:

       NL_VFUN        vfn;
       NL_VVALUE      vvl;
       NL_DEGREE      p, q, r;
       NL_KNOTVECTOR  knu, knv, knw;
       ...
       (define vvl, knu, knv, and knw);
       ...
       N_SFuncFromKnotVectors(&vfn,&vvl,p,q,r,&knu,&knv,&knw);


   ACCESS:
   
     vfn         , in/out ,  NURBS volume function
     vvl         , input  ,  Control value object
     p,q,r       , input  ,  Degrees
     knu,knv,knw , input  ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncFromKnotVectors( NL_VFUN *vfn, NL_VVALUE *vvl, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv, NL_KNOTVECTOR *knw )
{

    vfn->vvl = vvl;
    vfn->p = p;
    vfn->q = q;
    vfn->r = r;
    vfn->knu = knu;
    vfn->knv = knv;
    vfn->knw = knw;
} /* end N_VFuncFromKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  generates a volume  function object  given function 
     values  and knots. It  allocates memory  to store the control value  
     and knot  vector  objects, and makes  proper pointer assignments to 
     create the volume function object. Memory for the volume function  
     structure is allocated  in the  calling routine. A  typical calling 
     example is:

       NL_VFUN    vfn;
       NL_REAL    **fuvw, *U, *V, *W;
       NL_INDEX   m, n, o, ir, is, it;
       NL_DEGREE  p, q, r;
       NL_STACKS  S;
       ...
       (allocate memory for fuvw, U, V, and W);
       ...
       N_SFuncFromKnots(&vfn,fuvw,m,n,o,p,q,r,U,V,W,ir,is,it,&S);


   ACCESS:
   
     vfn      , in/out ,  NURBS volume function
     fuvw     , input  ,  Function values
     m,n,o    , input  ,  Highest indexes in fuvw
     p,q,r    , input  ,  Degrees
     U,V,W    , input  ,  Knot vectors
     ir,is,it , input  ,  Highest indexes in U, V and W respectively
     S        , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VFuncFromKnots( NL_VFUN *vfn, NL_REAL *** fuvw, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_REAL *U, NL_REAL *V, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S )
{

    NL_VVALUE *vvl;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Allocate memory */

    vvl = N_AllocVValue( S );

    if( vvl EQ NULL )
        return (1);

    knu = N_AllocKnotVector( S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVector( S );

    if( knv EQ NULL )
        return (1);

    knw = N_AllocKnotVector( S );

    if( knw EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_VValueFromArray( vvl, fuvw, m, n, o );
    N_KnotVectorFromRealArray( knu, U, ir );
    N_KnotVectorFromRealArray( knv, V, is );
    N_KnotVectorFromRealArray( knw, V, it );
    N_VFuncFromKnotVectors( vfn, vvl, p, q, r, knu, knv, knw );

    /* Exit */

    return (0);
} /* end N_VFuncFromKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if memory is needed to store a volume
     function. If  the  function  is  initiaized  to  NULL,  memory  is  
     allocated. If  not,  the  routine  checkes  if  enough  memory  is 
     available. A typical calling example is:

       NL_VFUN     vfn;
       NL_INDEX    m, n, o, ir, is, it;
       NL_DEGREE   p, q, r;
       NL_STRING   rname;
       NL_STACKS   S;
       ...
       (get m, n, o, p, q, r, ir, is, it, and rname);
       ...
       N_VFuncSizeArrays(&vfn,m,n,o,p,q,r,ir,is,it,rname,&S);

     IT IS  ASSUMED THAT MEMORY TO STORE THE NL_SURFACE NL_FUNCTION STRUCTURE 
     ITSELF IS ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     vfn                  , in/out ,  Volume function to be created
     m,n,o,p,q,r,ir,is,it , input  ,  Usual volume parameters
     rname                , input  ,  Routine name
     S                    , input  ,  vfn's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VFuncSizeArrays( NL_VFUN *vfn, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STRING rname, NL_STACKS *S )
{

    NL_FLAG error;

    /* See if memory is needed */

    if( N_VFuncAreArraysNULL( vfn ) )
    {
        error = N_AllocVFuncArrays( vfn, m, n, o, p, q, r, ir, is, it, S );

        if( error EQ 1 )
            return (1);
    }
    else
    {
        error = N_VolumeIsFuncSized( vfn, m, n, o, ir, is, it, rname );

        if( error EQ 1 )
            return (1);

        N_VFuncSetSizeIndices( vfn, m, n, o, p, q, r, ir, is, it );
    }

    /* Exit */

    return (0);
} /* end N_VFuncSizeArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the highest indexes in a volume function 
     object. A typical calling example is:

       NL_VFUN   vfn;
       NL_INDEX  m, n, o, ir, is, it;
       ...
       N_VFuncGetArraySizes(&vfn,&m,&n,&o,&ir,&is,&it);


   ACCESS:
   
     vfn      , input  ,  Volume function
     m,n,o    , output ,  Highest indexes in fuv
     ir,is,it , output ,  Highest indexes in U and V


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncGetArraySizes( NL_VFUN *vfn, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_INDEX *ir, NL_INDEX *is, NL_INDEX *it )
{

    *m = vfn->vvl->m;
    *n = vfn->vvl->n;
    *o = vfn->vvl->o;
    *ir = vfn->knu->m;
    *is = vfn->knv->m;
    *it = vfn->knw->m;
} /* end N_VFuncGetArraySizes */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets parameters to complete volume function
     definition. A typical calling example is:

       NL_VFUN    vfn;
       NL_INDEX   m, n, o, ir, is, it;
       NL_DEGREE  p, q, r;
       ...
       N_VFuncSetSizeIndices(&vfn,m,n,o,p,q,r,ir,is,it);


   ACCESS:
   
     vfn      , in/out ,  Volume function
     m,n,o    , input  ,  Highest indexes in control value array
     p,q,r    , input  ,  Degrees
     ir,is,it , input  ,  Highest indexes in knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncSetSizeIndices( NL_VFUN *vfn, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it )
{

    vfn->vvl->m = m;
    vfn->vvl->n = n;
    vfn->vvl->o = o;
    vfn->p = p;
    vfn->q = q;
    vfn->r = r;
    vfn->knu->m = ir;
    vfn->knv->m = is;
    vfn->knw->m = it;
} /* end N_VFuncSetSizeIndices */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine breaks a volume function object down to its 
     components, i.e. indexes, control  value array and knot vectors. A  
     typical calling example is:

       NL_VFUN    vfn;
       NL_INDEX   m, n, o, ir, is, it;
       NL_DEGREE  p, q, r;
       NL_REAL    ***fuvw, *U, *V, *W;
       ...
       N_VFuncGetAll(&vfn,&m,&n,&0,&fuvw,&p,&q,&r,&ir,&is,&it,&U,&V,&W);


   ACCESS:
   
     vfn      , input  ,  NURBS volume function
     m,n,o    , output ,  Highest indexes in fuvw
     fuvw     , output ,  Control values
     p,q,r    , output ,  Degrees
     ir,is,it , output ,  Highest indexes in U and V, respectively
     U,V,W    , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncGetComponents( NL_VFUN *vfn, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_REAL **** fuvw, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r, NL_INDEX *ir, NL_INDEX *is, NL_INDEX *it, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W )
{

    *m = vfn->vvl->m;
    *n = vfn->vvl->n;
    *o = vfn->vvl->o;
    *fuvw = vfn->vvl->fuvw;
    *p = vfn->p;
    *q = vfn->q;
    *r = vfn->r;
    *ir = vfn->knu->m;
    *is = vfn->knv->m;
    *it = vfn->knw->m;
    *U = vfn->knu->U;
    *V = vfn->knv->U;
    *W = vfn->knw->U;
} /* end N_VFuncGetComponents */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control values and knots from a volume 
     function object. A typical calling example is:

       NL_VFUN  vfn;
       NL_REAL  ***fuvw, *U, *V, *W;
       ...
       N_VFuncGetKnots(&vfn,&fuvw,&U,&V,&W);


   ACCESS:
   
     vfn   , input  ,  NURBS volume function
     fuvw  , output ,  Control value array
     U,V,W , output ,  Knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncGetKnots( NL_VFUN *vfn, NL_REAL **** fuvw, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W )
{

    *fuvw = vfn->vvl->fuvw;
    *U = vfn->knu->U;
    *V = vfn->knv->U;
    *W = vfn->knw->U;
} /* end N_VFuncGetKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the degree of a volume function. A 
     typical calling example is:

       NL_VFUN    vfn;
       NL_DEGREE  *p, *q, *r;
       ...
       N_VFuncGetDegrees(&vfn,&p,&q,&r);


   ACCESS:
   
     vfn   , input  ,  NURBS volume function
     p,q,r , output ,  Degrees


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncGetDegrees( NL_VFUN *vfn, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r )
{

    *p = vfn->p;
    *q = vfn->q;
    *r = vfn->r;
} /* end N_VFuncGetDegrees */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine initializes a volume function structure by  
     setting pointers to NULL  and the degrees  to -1. It is  used to  
     check if memory allocation is needed, ie if the volume function 
     is initialized to NULL, memory is allocated to hold the  control 
     value and knot vectors objects.  Otherwise  it is  assumed  that 
     memory has already been allocated. A typical calling example is:

       NL_VFUN  vfn;
       ...
       N_VFuncInitArrays(&vfn);
     

   ACCESS:
   
     vfn , in/out ,  Volume function


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncInitArrays( NL_VFUN *vfn )
{

    vfn->vvl = NULL;
    vfn->p = -1;
    vfn->q = -1;
    vfn->r = -1;
    vfn->knu = NULL;
    vfn->knv = NULL;
    vfn->knw = NULL;
} /* end N_VFuncInitArrays */

/**********************************************************************/
/* N_VFuncAreArraysNULL: Is volume function initialized to NULL?                 */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a volume function is initialized to 
     the NULL  function. If yes,  NL_TRUE is returned. Otherwise,  NL_FALSE is  
     returned. This  routine  is  used  to  check  if  memory allocation 
     is needed. A typical calling example is:

       NL_VFUN  vfn;
       ...
       if( N_VFuncAreArraysNULL(&vfn) )  --> allocate memory;
     

   ACCESS:
   
     vfn , input ,  Volume function


   RETURN CODES:

     NL_TRUE:  Function is initialized to NULL (need memory)
     NL_FALSE: Function is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_VFuncAreArraysNULL( NL_VFUN *vfn )
{

    NL_DEGREE p, q, r;

    NL_VVALUE *vvl;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Get local notation */

    N_VFuncGetArrayAndKnotVectors( vfn, &vvl, &p, &q, &r, &knu, &knv, &knw );

    /* Check initialization */

    if( vvl EQ NULL OR p EQ - 1 OR q EQ - 1 OR r EQ - 1 OR knu EQ NULL OR knv EQ NULL OR knw EQ NULL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_VFuncAreArraysNULL */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the knot vector objects from a volume 
     function. A typical calling example is:

       NL_VFUN        vfn;
       NL_KNOTVECTOR  *knu, *knv, *knw;
       ...
       N_VFuncGetKnotVectors(&vfn,&knu,&knv,&knw);


   ACCESS:
   
     vfn         , input  ,  NURBS volume function
     knu,knv,knw , output ,  Knot vector objects


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncGetKnotVectors( NL_VFUN *vfn, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw )
{

    *knu = vfn->knu;
    *knv = vfn->knv;
    *knw = vfn->knw;
} /* end N_VFuncGetKnotVectors */

/**********************************************************************/
/* N_VFuncGetArrayAndKnotVectors: Get control value and knot vector objects from NL_VFUN      */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This utility routine detaches the control value and the knot vector
     objects from a given volume function. A typical calling example:

       NL_VFUN        vfn;
       NL_VVALUE      *vvl;
       NL_DEGREE      p, q, r;
       NL_KNOTVECTOR  *knu, *knv, *knw;
       ...
       N_VFuncGetArrayAndKnotVectors(&vfn,&vvl,&p,&q,&r,&knu,&knv,&knw);


   ACCESS:
   
     vfn         , input  ,  Volume function
     vvl         , output ,  Control value
     p,q,r       , output ,  Degrees
     knu,knv,knw , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VFuncGetArrayAndKnotVectors( NL_VFUN *vfn, NL_VVALUE ** vvl, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw )
{

    *vvl = vfn->vvl;
    *p = vfn->p;
    *q = vfn->q;
    *r = vfn->r;
    *knu = vfn->knu;
    *knv = vfn->knv;
    *knw = vfn->knw;
} /* end N_VFuncGetArrayAndKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     Given a volume function object, this routine allocates memory to 
     store control points and knots. A typical calling example is:

       NL_VFUN    vfn;
       NL_STACKS  S;
       ...
       N_AllocVFuncArrays(&vfn,m,n,o,p,q,r,ir,is,it,&S);

     where  <m,p,ir>,  <n,q,is> and <o,r,it> are the usual volume parameters. 
     Since  the  declaration  "NL_VFUN  vfn"  defines  the data  type and 
     allocates memory,  memory is needed to store only the control net 
     and knot vector objects.


   ACCESS:
   
     vfn      , in/out ,  Volume function structure
     m,n,o    , input  ,  Highest indexes in control value array
     p,q,r    , input  ,  Degrees of the function
     ir,is,it , input  ,  Highest indexes in knot vector arrays
     S        , input  ,  vfn's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllocVFuncArrays( NL_VFUN *vfn, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S )
{

    NL_VVALUE *vvl;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Allocate memory */

    vvl = N_AllocVValueAndArray( m, n, o, S );

    if( vvl EQ NULL )
        return (1);

    knu = N_AllocKnotVectorAndArray( ir, S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVectorAndArray( is, S );

    if( knv EQ NULL )
        return (1);

    knw = N_AllocKnotVectorAndArray( it, S );

    if( knw EQ NULL )
        return (1);

    /* Build function structure */

    N_VFuncFromKnotVectors( vfn, vvl, p, q, r, knu, knv, knw );

    /* Exit */

    return (0);
} /* end N_AllocVFuncArrays */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point on a volume function by evaluating 
     all  non-vanishing   basis  functions  and  multiplying  them  by 
     appropriate values. Discontinuous functions can also be evaluated 
     by passing NL_LEFT/NL_RIGHT flags. A typical calling example is:

       NL_VFUN       vfn;
       NL_PARAMETER  u, v, w;
       NL_REAL       F;
       ...
       (define vfn, get u, v, and w);
       ...
       N_VFuncEval(&vfn,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,&F);


   ACCESS:
   
     vfn         , input  ,  Volume function
     u,v,w       , input  ,  Parameter values 
     ufl,vfl,wfl , input  ,  Flags:
                               NL_LEFT : u is in [u[j],u[j+1])
                               NL_RIGHT: u is in (u[j],u[j+1]]
     F           , output ,  Function value


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VFuncEval( NL_VFUN *vfn, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_REAL *F )
{
    NL_PRIVATE NL_STRING rname = _T("N_VFuncEval");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, usp, vsp, wsp, iu, jv;

    NL_DEGREE p, q, r;

    NL_REAL *** fuvw, *NU, *NV, *NW;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_STACKS S;

    NL_REAL tu[NL_MAXDEG + 1], tuv[NL_MAXDEG + 1][NL_MAXDEG + 1];

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_VFuncGetKnots( vfn, &fuvw, &NU, &NV, &NW );
    N_VFuncGetDegrees( vfn, &p, &q, &r );
    N_VFuncGetKnotVectors( vfn, &knu, &knv, &knw );

    /* Check parameters */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knw, w, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines */

    NU = N_AllocReal1dArray( p, &S );

    if( NU EQ NULL )
        NL_QUIT;

    NV = N_AllocReal1dArray( q, &S );

    if( NV EQ NULL )
        NL_QUIT;

    NW = N_AllocReal1dArray( r, &S );

    if( NV EQ NULL )
        NL_QUIT;

    error = N_BasisEval( knu, p, u, ufl, NU, &usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEval( knv, q, v, vfl, NV, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEval( knw, r, w, wfl, NW, &wsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the function value                               */
    /* F = Sum_i(Sum_j(Sum_k(   fuvw[usp-p+i][vsp-q+j][wsp-r+k] */
    /*                       *  NU[i] * NV[j] * NW[k] )         */

    for ( i = 0; i <= p; i++ )
    {
        iu = usp - p + i;

        for ( j = 0; j <= q; j++ )
        {
            jv = vsp - q + j;
            tuv[i][j] = 0.0;

            for ( k = 0; k <= r; k++ )
            {
                tuv[i][j] += fuvw[iu][jv][wsp - r + k] * NW[k];
            }
        }
    }

    for ( i = 0; i <= p; i++ )
    {
        iu = usp - p + i;
        tu[i] = 0.0;

        for ( j = 0; j <= q; j++ )
        {
            tu[i] += NV[j] * tuv[i][j];
        }
    }
    *F = 0.0;

    for ( i = 0; i <= p; i++ )
    {
        *F += NU[i] * tu[i];
    }

    /*        tu = N_AllocReal1dArray(p,&S);         */
    /*        if( tu EQ NULL )  NL_QUIT;                   */
    /*                                                  */
    /*        for( i=0; i<=p; i++ )                     */
    /*        {                                         */
    /*          iu    = usp-p+i;                        */
    /*          tu[i] = 0.0;                            */
    /*          for( j=0; j<=q; j++ )                   */
    /*          {                                       */
    /*            tu[i] += NV[j]*fuvw[iu][vsp-q+j];     */
    /*          }                                       */
    /*        }                                         */
    /*                                                  */
    /*        *F = 0.0;                                 */
    /*        for( i=0; i<=p; i++ )                     */
    /*        {                                         */
    /*          *F += NU[i]*tu[i];                      */
    /*        }                                         */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VFuncEval */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes  all derivatives  of a  volume function by 
     evaluating all non-vanishing basis functions and their derivatives, 
     and evaluating  the derivative volume. Discontinuous functions can 
     also be  handled by passing  NL_LEFT/NL_RIGHT flags.  Memory to store the
     derivatives  must be  allocated in  the calling  routine. A typical 
     calling example is:

       NL_VFUN       vfn;
       NL_PARAMETER  u, v, w;
       NL_INDEX      udr, vdr, wdr;
       NL_REAL       ***FD;
       ...
       (define vfn, get u, v, w, udr vdr, wdr, and allocate memory for FD);
       ...
       N_VFuncDerivs(&vfn,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,udr,vdr,wdr,FD);


   ACCESS:
   
     vfn         , input  ,  Volume function
     u,v,w       , input  ,  Parameter values 
     ufl,vfl,wfl , input  ,  Flags:
                               NL_LEFT : t is in [t[j],t[j+1])
                                      (NL_RIGHT DERIVATIVES REQUIRED)
                               NL_RIGHT: t is in (t[j],t[j+1]]
                                      (NL_LEFT DERIVATIVES REQUIRED)
                               (t is either u or v)
     udr,vdr,wdr , input  ,  Highest derivatives required
     FD          , output ,  Derivatives; FD[l][m][n] is  the  l-th  derivative  
                             in the u-direction, the m-th derivative  in  
                             the  v-direction, and the n-th derivative in the
                             w-direction. FD MUST HAVE ROOM TO STORE UP 
                             TO FD[udr][vdr][wdr].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VFuncDerivs( NL_VFUN *vfn, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL *** FD )
{
    NL_PRIVATE NL_STRING rname = _T("N_VFuncDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, m, n, iu, jv, usp, vsp, wsp, dru, drv, drw;

    NL_DEGREE p, q, r;

    NL_REAL *** fuvw, ** DU, ** DV, ** DW, *tmpU;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_STACKS S;

    NL_REAL tu[NL_MAXDEG + 1], tuv[NL_MAXDEG + 1][NL_MAXDEG + 1];

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_VFuncGetKnots( vfn, &fuvw, &tmpU, &tmpU, &tmpU );
    N_VFuncGetDegrees( vfn, &p, &q, &r );
    N_VFuncGetKnotVectors( vfn, &knu, &knv, &knw );

    /* Check parameters */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knw, w, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines and their derivatives */

    dru = NL_MIN( udr, p );
    DU = N_AllocReal2dArray( dru, p, &S );

    if( DU EQ NULL )
        NL_QUIT;

    drv = NL_MIN( vdr, q );
    DV = N_AllocReal2dArray( drv, q, &S );

    if( DV EQ NULL )
        NL_QUIT;

    drw = NL_MIN( wdr, r );
    DW = N_AllocReal2dArray( drw, r, &S );

    if( DW EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knu, p, u, ufl, dru, DU, &usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisDerivs( knv, q, v, vfl, drv, DV, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisDerivs( knw, r, w, wfl, drw, DW, &wsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize derivative matrix */

    for ( l = 0; l <= udr; l++ )
    {
        for ( m = 0; m <= vdr; m++ )
        {
            for ( n = 0; n <= wdr; n++ )
            {
                FD[l][m][n] = 0.0;
            }
        }
    }

    /* Compute derivatives                                                */
    /* FD[l][m][n] = Sum_i(Sum_j(Sum_k(   fuvw[usp-p+i][vsp-q+j][wsp-r+k] */
    /*                                 *  NU[l][i] * NV[m][j] * NW[n][k]) */

    for ( n = 0; n <= drw; n++ )
    {
        for ( i = 0; i <= p; i++ )
        {
            iu = usp - p + i;

            for ( j = 0; j <= q; j++ )
            {
                jv = vsp - q + j;
                tuv[i][j] = 0.0;

                for ( k = 0; k <= r; k++ )
                {
                    tuv[i][j] += fuvw[iu][jv][wsp - r + k] * DW[n][k];
                }
            }
        }

        for ( m = 0; m <= drv; m++ )
        {
            for ( i = 0; i <= p; i++ )
            {
                iu = usp - p + i;
                tu[i] = 0.0;

                for ( j = 0; j <= q; j++ )
                {
                    tu[i] += DV[m][j] * tuv[i][j];
                }
            }

            for ( l = 0; l <= dru; l++ )
            {
                for ( i = 0; i <= p; i++ )
                {
                    FD[l][m][n] += DU[l][i] * tu[i];
                }
            }
        }
    }

    /*        tu = N_AllocReal1dArray(p,&S);                */
    /*        if( tu EQ NULL )  NL_QUIT;                          */
    /*                                                         */
    /*        for( m=0; m<=drv; m++ )                          */
    /*        {                                                */
    /*          for( i=0; i<=p; i++ )                          */
    /*          {                                              */
    /*            tmp   = usp-p+i;                             */
    /*            tu[i] = 0.0;                                 */
    /*            for( j=0; j<=q; j++ )                        */
    /*            {                                            */
    /*              tu[i] += DV[m][j]*fuvw[tmp][vsp-q+j];      */
    /*            }                                            */
    /*          }                                              */
    /*                                                         */
    /*          for( l=0; l<=dru; l++ )                        */
    /*          {                                              */
    /*            for( i=0; i<=p; i++ )                        */
    /*            {                                            */
    /*              FD[l][m] += DU[l][i]*tu[i];                */
    /*            }                                            */
    /*          }                                              */
    /*        }                                                */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VFuncDerivs */
#endif // NLIB_UNUSED
