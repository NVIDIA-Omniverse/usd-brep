// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************************/
/* SrfSymbol.c : Symbolic Function Definitions that act on NL_SURFACE objects */
/******************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */
#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */
#include <NL_FuncsAdv.h>    /* Advanced NL_CFUN, NL_CVALUE, NL_SFUN, NL_SVALUE, NL_VFUN, and NL_VVALUE functions */


#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes an upper bound on the 
     first derivative of a surface. A typical calling example is:

       NL_SURFACE  sur;
       NL_POINT    DM;
       NL_REAL     mag; 
       ...
       (define sur);
       ...
       N_SrfMaxFirstDeriv(&sur,NL_UDIR,&DM,&mag);


   ACCESS:
   
     sur , input  ,  NURBS surface
     dir , input  ,  Flag:
                       NL_UDIR: maximum of Su is computed
                       NL_VDIR: maximum of Sv is computed
     DM  , output ,  Maximum first derivative vector
     mag , output ,  Magnitude of DM


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfMaxFirstDeriv( NL_SURFACE *sur, NL_FLAG dir, NL_POINT *DM, NL_REAL *mag )
{
    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INDEX i, j, k, l, m, n, r, s;

    NL_REAL len;

    NL_POINT D;

    NL_SURFACE *** bez, *der;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and allocate memory */

    N_SrfGetDegrees( sur, &p, &q );

    *mag = 0.0;
    N_CopyPt( NL_ZERO, DM );

    if( (p LT 1 AND dir EQ NL_UDIR)OR( q LT 1 AND dir EQ NL_VDIR ) )
        NL_OUT;

    p = 2 * p;
    q = 2 * q;
    n = p;
    m = q;
    r = n + p + 1;
    s = m + q + 1;

    der = N_AllocSrfAndArrays( n, m, p, q, r, s, &SL );

    if( der EQ NULL )
        NL_QUIT;

    /* Get Bezier pieces */

    error = N_SrfDecomposeToBez( sur, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            error = N_FirstDerivSrfRatSrf( bez[i][j], dir, der, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfMaxMagnitudePosVectors( der, &D, &len );

            if( len GT *mag )
            {
                *mag = len;
                N_CopyPt( D, DM );
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfMaxFirstDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes an upper bound on the 
     second derivative of a surface. A typical calling example is:

       NL_SURFACE  sur;
       NL_POINT    DM;
       NL_REAL     mag; 
       ...
       (define sur);
       ...
       N_SrfMax2ndDeriv(&sur,NL_UVDIR,&DM,&mag);


   ACCESS:
   
     sur , input  ,  NURBS surface
     dir , input  ,  Flag:
                       NL_UDIR : maximum of Suu is computed
                       NL_UVDIR: maximum of Suv is computed
                       NL_VDIR : maximum of Svv is computed
     DM  , output ,  Maximum second derivative vector
     mag , output ,  Magnitude of DM


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfMax2ndDeriv( NL_SURFACE *sur, NL_FLAG dir, NL_POINT *DM, NL_REAL *mag )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfMax2ndDeriv");

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INDEX i, j, k, l, m, n, r, s;

    NL_REAL len;

    NL_POINT D;

    NL_SURFACE *** bez, *der;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and allocate memory */

    N_SrfGetDegrees( sur, &p, &q );

    switch( dir )
    {
        case NL_UDIR:
            break;

        case NL_UVDIR:
            break;

        case NL_VDIR:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    *mag = 0.0;
    N_CopyPt( NL_ZERO, DM );

    if( p LT 2 AND dir EQ NL_UDIR )
        NL_OUT;

    if( q LT 2 AND dir EQ NL_VDIR )
        NL_OUT;

    if( ((p LT 1)OR( q LT 1 ))AND dir EQ NL_UVDIR )
        NL_OUT;

    p = 3 * p;
    q = 3 * q;
    n = p;
    m = q;
    r = n + p + 1;
    s = m + q + 1;

    der = N_AllocSrfAndArrays( n, m, p, q, r, s, &SL );

    if( der EQ NULL )
        NL_QUIT;

    /* Get Bezier pieces */

    error = N_SrfDecomposeToBez( sur, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            switch( dir )
            {
                case NL_UDIR:
                    error = N_SecondDerivSrfRatSrf( bez[i][j], der, &SL );
                    break;

                case NL_UVDIR:
                    error = N_MixedPartialDerivSrfRatSrf_UV( bez[i][j], der, &SL );
                    break;

                case NL_VDIR:
                    error = N_SecondDerivSrfRatSrf_VV( bez[i][j], der, &SL );
                    break;
            }

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfMaxMagnitudePosVectors( der, &D, &len );

            if( len GT *mag )
            {
                *mag = len;
                N_CopyPt( D, DM );
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfMax2ndDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators  routine  computes the dot product of two 
     surfaces. The knot  vectors are  rescaled to the unit span  [0,1],  
     however, the input entities do not change either parametrically or 
     geometrically. A typical calling example is:

       NL_SURFACE  surF, surG;
       NL_SFUN     num, den;
       NL_STACKS   SO;
       ...
       (define surF and surG);
       ...
       N_SFuncInitArrays(&num);
       N_SFuncInitArrays(&den);
       N_DotProductTwoSrfs(&surF,&surG,&num,&den,&SO);

     If memory is  available, num and den are not  initialized and  the 
     routine assumes that memory  allocation has been done. However, it 
     checks for the proper amount by looking at the  highest indexes in 
     num's  and  den's  knot  vectors and  control  value  objects. THE  
     STORAGE OF THE OUTPUT FUNCTIONS IS COMPACTED, IE THE MEMORY PASSED 
     IN IS DESTROYED.


   ACCESS:
   
     surF , in/out ,  Surface (ITS KNOT NL_VECTOR IS RESCALED)
     surG , in/out ,  Surface (ITS KNOT NL_VECTOR IS RESCALED)
     num  , output ,  Numerator of the dot product
     den  , output ,  Denominator of the dot product (if rational)
     SO   , input  ,  num's and den's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_DotProductTwoSrfs( NL_SURFACE *surF, NL_SURFACE *surG, NL_SFUN *num, NL_SFUN *den, NL_STACKS *SO )
{
    NL_PRIVATE NL_STRING rname = _T("N_DotProductTwoSrfs");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX *mu, *mv, i, j, k, l, nf, mf, rf, sf, ng, mg, rg, sg, nh, mh, rh, sh, nfr, nfs, ngr, ngs, mfx, mfy, mgx, mgy, af, ag, bf, bg, mxfb, myfb, mxgb, mygb, ixfr, iyfs, ixgr, iygs, iu, jv, mlf, mlg, mi, ih, jh, kf, lf, kg, lg, kh, lh;

    NL_DEGREE pf, qf, pg, qg, ph, qh;

    NL_REAL ** fn, ** fd = NULL, ** fnb, ** fdb = NULL, *RF, *SF, *RG, *SG, *RN, *SN, *RD = NULL, *SD = NULL, *XF, *YF, *XG, *YG, *XFB, *YFB, *XGB, *YGB, *u, *v;

    NL_CPOINT ** Fw, ** Gw, ** FBw, ** GBw;

    NL_KNOTVECTOR *kfr, *kfs, *kgr, *kgs, *kfx, *kfy, *kgx, *kgy, *kfrr, *kfsr, *kgrr, *kgsr, *kfrb, *kfsb, *kgrb, *kgsb;

    NL_SURFACE surFR, surGR;

    NL_RMATRIX pmr, pms;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals, check degrees and set rational flag */

    N_SrfGetCPtsDegreesAndKnots( surF, &nf, &mf, &Fw, &pf, &qf, &rf, &sf, &RF, &SF );
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

    if( N_IsSrfRat( surF ) )
        rat = NL_YES;

    else if( N_IsSrfRat( surG ) )
        rat = NL_YES;

    /* Merge knots */

    N_SrfGetKnotVectors( surF, &kfr, &kfs );
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

    N_SrfInitArrays( &surFR );
    error = N_SrfCopy( surF, &surFR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &surGR );
    error = N_SrfCopy( surG, &surGR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( mxfb GE 0 )
    {
        error = N_SrfInsertKnots( &surFR, kfrb, NL_UDIR, &surFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( myfb GE 0 )
    {
        error = N_SrfInsertKnots( &surFR, kfsb, NL_VDIR, &surFR, &SL, &SL );

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

    N_SrfGetCPtsDegreesAndKnots( &surFR, &nf, &mf, &Fw, &pf, &qf, &rf, &sf, &RF, &SF );
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

    error = N_SFuncSizeArrays( num, nh, mh, ph, qh, rh, sh, rname, SO );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( num, &fn, &RN, &SN );

    if( rat EQ NL_YES )
    {
        error = N_SFuncSizeArrays( den, nh, mh, ph, qh, rh, sh, rname, SO );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( den, &fd, &RD, &SD );
    }

    /* Compute dot product */

    FBw = N_AllocCPt2dArray( pf, qf, &SL );

    if( FBw EQ NULL )
        NL_QUIT;

    GBw = N_AllocCPt2dArray( pg, qg, &SL );

    if( GBw EQ NULL )
        NL_QUIT;

    fnb = N_AllocReal2dArray( ph, qh, &SL );

    if( fnb EQ NULL )
        NL_QUIT;

    if( rat EQ NL_YES )
    {
        fdb = N_AllocReal2dArray( ph, qh, &SL );

        if( fdb EQ NULL )
            NL_QUIT;
    }

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
                    N_CopyCPt( Fw[kf + k][lf + l], &FBw[k][l] );
            }

            for ( k = 0; k <= pg; k++ )
            {
                for ( l = 0; l <= qg; l++ )
                    N_CopyCPt( Gw[kg + k][lg + l], &GBw[k][l] );
            }

            error = N_BezSrfDotProduct( FBw, pf, qf, GBw, pg, qg, &pmr, &pms, 0, ph, 0, qh, fnb, fdb );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = 0; k <= ph; k++ )
            {
                for ( l = 0; l <= qh; l++ )
                    fn[kh + k][lh + l] = fnb[k][l];
            }

            if( rat EQ NL_YES )
            {
                for ( k = 0; k <= ph; k++ )
                {
                    for ( l = 0; l <= qh; l++ )
                        fd[kh + k][lh + l] = fdb[k][l];
                }
            }
        }
    }

    k = -1;
    af = pf + 1;

    for ( i = 0; i <= ph; i++ )
        RN[++k] = RF[0];

    while( af LT rf - pf )
    {
        while( RF[af]EQ RF[af + 1] )
            af++;

        for ( i = 1; i <= ph; i++ )
            RN[++k] = RF[af];

        af++;
    }

    for ( i = 0; i <= ph; i++ )
        RN[++k] = RF[rf];

    l = -1;
    bf = qf + 1;

    for ( j = 0; j <= qh; j++ )
        SN[++l] = SF[0];

    while( bf LT sf - qf )
    {
        while( SF[bf]EQ SF[bf + 1] )
            bf++;

        for ( j = 1; j <= qh; j++ )
            SN[++l] = SF[bf];

        bf++;
    }

    for ( j = 0; j <= qh; j++ )
        SN[++l] = SF[sf];

    if( rat EQ NL_YES )
    {
        k = -1;
        af = pf + 1;

        for ( i = 0; i <= ph; i++ )
            RD[++k] = RF[0];

        while( af LT rf - pf )
        {
            while( RF[af]EQ RF[af + 1] )
                af++;

            for ( i = 1; i <= ph; i++ )
                RD[++k] = RF[af];

            af++;
        }

        for ( i = 0; i <= ph; i++ )
            RD[++k] = RF[rf];

        l = -1;
        bf = qf + 1;

        for ( j = 0; j <= qh; j++ )
            SD[++l] = SF[0];

        while( bf LT sf - qf )
        {
            while( SF[bf]EQ SF[bf + 1] )
                bf++;

            for ( j = 1; j <= qh; j++ )
                SD[++l] = SF[bf];

            bf++;
        }

        for ( j = 0; j <= qh; j++ )
            SD[++l] = SF[sf];
    }

    /* Remove knots */

    for ( i = 0; i <= iu; i++ )
    {
        if( mu[i]GT 0 )
        {
            error = N_SrfFuncRemoveKnot( num, u[i], mu[i], NL_UDIR, num, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    for ( j = 0; j <= jv; j++ )
    {
        if( mv[j]GT 0 )
        {
            error = N_SrfFuncRemoveKnot( num, v[j], mv[j], NL_VDIR, num, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    if( rat EQ NL_YES )
    {
        for ( i = 0; i <= iu; i++ )
        {
            if( mu[i]GT 0 )
            {
                error = N_SrfFuncRemoveKnot( den, u[i], mu[i], NL_UDIR, den, SO );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }

        for ( j = 0; j <= jv; j++ )
        {
            if( mv[j]GT 0 )
            {
                error = N_SrfFuncRemoveKnot( den, v[j], mv[j], NL_VDIR, den, SO );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    /* Compact output functions */

    N_SFuncGetArraySizes( num, &i, &j, &k, &l );

    if( i LT nh OR j LT mh )
    {
        error = N_SrfFuncCompact( num, SO );

        if( error EQ NL_YES )
            NL_OUT;

        if( rat EQ NL_YES )
        {
            error = N_SrfFuncCompact( den, SO );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_DotProductTwoSrfs */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the cross product of two 
     NURBS surfaces. The knot  vectors are  rescaled to the unit span,  
     however, the input  entities do not  change either parametrically 
     or geometrically. A typical calling example is:

       NL_SURFACE  surF, surG, surH;
       NL_STACKS   SO;
       ...
       (define surF and surG);
       ...
       N_SrfInitArrays(&surH);
       N_CrossProductTwoSrfs(&surF,&surG,&surH,&SO);

     If memory is available, surH is not  initialized and  the routine 
     assumes that memory  allocation has been done. However, it checks 
     for the proper amount by looking at the highest indexes in surH's
     knot  vector  and  control  polygon  objects. THE  STORAGE OF THE 
     OUTPUT NL_SURFACE IS  COMPACTED, THAT  IS, THE MEMORY  PASSED  IN IS 
     DESTROYED.


   ACCESS:
   
     surF , input  ,  Surface (ITS KNOT NL_VECTOR IS RESCALED)
     surG , input  ,  Surface (ITS KNOT NL_VECTOR IS RESCALED)
     surH , output ,  Cross product of surF and surG
     SH   , input  ,  surH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrossProductTwoSrfs( NL_SURFACE *surF, NL_SURFACE *surG, NL_SURFACE *surH, NL_STACKS *SO )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrossProductTwoSrfs");

    NL_FLAG error = NL_NO;

    NL_INDEX *mu, *mv, i, j, k, l, nf, mf, rf, sf, ng, mg, rg, sg, nh, mh, rh, sh, nfr, nfs, ngr, ngs, mfx, mfy, mgx, mgy, af, ag, bf, bg, mxfb, myfb, mxgb, mygb, ixfr, iyfs, ixgr, iygs, iu, jv, mlf, mlg, mi, ih, jh, kf, lf, kg, lg, kh, lh;

    NL_DEGREE pf, qf, pg, qg, ph, qh;

    NL_REAL *RF, *SF, *RG, *SG, *RH, *SH, *XF, *YF, *XG, *YG, *XFB, *YFB, *XGB, *YGB, *u, *v;

    NL_CPOINT ** Fw, ** Gw, ** Hw, ** FBw, ** GBw, ** FGw;

    NL_KNOTVECTOR *kfr, *kfs, *kgr, *kgs, *kfx, *kfy, *kgx, *kgy, *kfrr, *kfsr, *kgrr, *kgsr, *kfrb, *kfsb, *kgrb, *kgsb;

    NL_SURFACE surFR, surGR;

    NL_RMATRIX pmr, pms;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals, check degrees and set rational flag */

    N_SrfGetCPtsDegreesAndKnots( surF, &nf, &mf, &Fw, &pf, &qf, &rf, &sf, &RF, &SF );
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

    N_SrfGetKnotVectors( surF, &kfr, &kfs );
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

    N_SrfInitArrays( &surFR );
    error = N_SrfCopy( surF, &surFR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &surGR );
    error = N_SrfCopy( surG, &surGR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( mxfb GE 0 )
    {
        error = N_SrfInsertKnots( &surFR, kfrb, NL_UDIR, &surFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( myfb GE 0 )
    {
        error = N_SrfInsertKnots( &surFR, kfsb, NL_VDIR, &surFR, &SL, &SL );

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

    N_SrfGetCPtsDegreesAndKnots( &surFR, &nf, &mf, &Fw, &pf, &qf, &rf, &sf, &RF, &SF );
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

    /* Compute cross product */

    FBw = N_AllocCPt2dArray( pf, qf, &SL );

    if( FBw EQ NULL )
        NL_QUIT;

    GBw = N_AllocCPt2dArray( pg, qg, &SL );

    if( GBw EQ NULL )
        NL_QUIT;

    FGw = N_AllocCPt2dArray( ph, qh, &SL );

    if( FGw EQ NULL )
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
                    N_CopyCPt( Fw[kf + k][lf + l], &FBw[k][l] );
            }

            for ( k = 0; k <= pg; k++ )
            {
                for ( l = 0; l <= qg; l++ )
                    N_CopyCPt( Gw[kg + k][lg + l], &GBw[k][l] );
            }

            error = N_BezSrfCrossProduct( FBw, pf, qf, GBw, pg, qg, &pmr, &pms, 0, ph, 0, qh, FGw );

            if( error EQ NL_YES )
                NL_OUT;

            for ( k = 0; k <= ph; k++ )
            {
                for ( l = 0; l <= qh; l++ )
                    N_CopyCPt( FGw[k][l], &Hw[kh + k][lh + l] );
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
} /* end N_CrossProductTwoSrfs */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the sum/difference  of 
     two NURBS surfaces. The result is obtained by  using other symbolic 
     operators to compute the following expression:

          N_p       N_q   N_p*D_q +/- N_q*D_p   N_r
          ---  +/-  --- = ------------------- = ---
          D_p       D_q         D_p*D_q         D_r

     If the  surfaces are  non-rational, the  computation  simplifies to 
     N_p +/- N_q. A typical calling example is:

       NL_SURFACE  surP, surQ, surR;
       NL_STACKS   SG;
       ...
       (define surP and surQ);
       ...
       N_SrfInitArrays(&surR);
       N_SumDiffTwoSrfs(&surP,&surQ,NL_PLUS,&surR,&SG);

     If memory is available, surR  is not  initialized  and  the routine 
     assumes that  memory  allocation  has been done. However, it checks 
     for the proper  amount by looking  at the highest indexes in surR's
     knot vector and control polygon objects.


   ACCESS:
   
     surP , input  ,  Surface
     surQ , input  ,  Surface
     opr  , input  ,  Operator flag:
                        NL_PLUS : sum
                        NL_MINUS: difference
     surR , output ,  Sum/difference of surP and surQ
     SG   , input  ,  surR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SumDiffTwoSrfs( NL_SURFACE *surP, NL_SURFACE *surQ, NL_FLAG opr, NL_SURFACE *surR, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SumDiffTwoSrfs");

    NL_FLAG error = NL_NO, ratP = NL_NO, ratQ = NL_NO, rat = NL_NO;

    NL_INDEX i, j, n, m, r, s, nup, nvp, nuq, nvq, mxp, myp, mxq, myq;

    NL_DEGREE pp, qp, pq, qq;

    NL_REAL *UP, *VP, *UQ, *VQ, *UR, *VR, *A;

    NL_CPOINT ** Pw, ** Qw, ** Rw;

    NL_SURFACE numP, numQ, num, tmp, surPW, surQW;

    NL_SFUN denP, denQ, den;

    NL_KNOTVECTOR *kup, *kvp, *kuq, *kvq, *kxp, *kyp, *kxq, *kyq;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get rational flags */

    if( N_IsSrfRat( surP ) )
        ratP = NL_YES;

    if( N_IsSrfRat( surQ ) )
        ratQ = NL_YES;

    if( ratP EQ NL_YES OR ratQ EQ NL_YES )
        rat = NL_YES;

    /* Compute sum/difference */

    switch( rat )
    {
        case NL_YES:

            /* Make non-rational surface rational */

            if( ratP EQ NL_NO )
                N_SrfNonRatToRat( surP );

            if( ratQ EQ NL_NO )
                N_SrfNonRatToRat( surQ );

            /* Extract numerators and denominators */

            N_SrfInitArrays( &numP );
            N_SFuncInitArrays( &denP );
            error = N_SrfGetNumAndDen( surP, &numP, &denP, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfInitArrays( &numQ );
            N_SFuncInitArrays( &denQ );
            error = N_SrfGetNumAndDen( surQ, &numQ, &denQ, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Compute product of denominators */

            N_SFuncInitArrays( &den );
            error = N_SrfFuncMultiplySrfFunc( &denP, &denQ, &den, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Compute terms in the numerator */

            N_SrfInitArrays( &num );
            error = N_SrfFuncMultiplySrf( &denQ, &numP, &num, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfInitArrays( &tmp );
            error = N_SrfFuncMultiplySrf( &denP, &numQ, &tmp, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Add terms in the numerator */

            N_SrfGetCPts( &num, &n, &m, &Pw );
            N_SrfGetCPts( &tmp, &n, &m, &Qw );

            switch( opr )
            {
                case NL_PLUS:
                    for ( i = 0; i <= n; i++ )
                    {
                        for ( j = 0; j <= m; j++ )
                            N_Sum2CPts( Pw[i][j], Qw[i][j], &Pw[i][j] );
                    }
                    break;

                case NL_MINUS:
                    for ( i = 0; i <= n; i++ )
                    {
                        for ( j = 0; j <= m; j++ )
                            N_Diff2CPts( Pw[i][j], Qw[i][j], &Pw[i][j] );
                    }
                    break;

                default:

                    NL_ERROR( NL_CAL_ERR );
            }

            /* Create output surface and reset rationality of input surfaces */

            error = N_CreateSrfFromNumAndDen( &num, &den, surR, SG );

            if( error EQ NL_YES )
                NL_OUT;

            if( ratP EQ NL_NO )
                N_SrfRatToNonRat( surP );

            if( ratQ EQ NL_NO )
                N_SrfRatToNonRat( surQ );
            break;

        case NL_NO:

            /* Create working surfaces */

            N_SrfInitArrays( &surPW );
            error = N_SrfCopy( surP, &surPW, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfInitArrays( &surQW );
            error = N_SrfCopy( surQ, &surQW, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Get local notation */

            N_SrfGetCPtsDegreesAndKnots( &surPW, &n, &m, &Pw, &pp, &qp, &r, &s, &UP, &VP );
            N_SrfGetCPtsDegreesAndKnots( &surQW, &n, &m, &Qw, &pq, &qq, &r, &s, &UQ, &VQ );
            N_SrfGetKnotVectors( &surPW, &kup, &kvp );
            N_SrfGetKnotVectors( &surQW, &kuq, &kvq );

            /* Degree elevate */

            if( pp NEQ pq )
            {
                if( pp LT pq )
                {
                    error = N_SrfElevateDegree( &surPW, pq - pp, NL_UDIR, &surPW, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_SrfGetCPtsDegreesAndKnots( &surPW, &n, &m, &Pw, &pp, &qp, &r, &s, &UP, &VP );
                    N_SrfGetKnotVectors( &surPW, &kup, &kvp );
                }
                else
                {
                    error = N_SrfElevateDegree( &surQW, pp - pq, NL_UDIR, &surQW, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_SrfGetCPtsDegreesAndKnots( &surQW, &n, &m, &Qw, &pq, &qq, &r, &s, &UQ, &VQ );
                    N_SrfGetKnotVectors( &surQW, &kuq, &kvq );
                }
            }

            if( qp NEQ qq )
            {
                if( qp LT qq )
                {
                    error = N_SrfElevateDegree( &surPW, qq - qp, NL_VDIR, &surPW, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_SrfGetCPtsDegreesAndKnots( &surPW, &n, &m, &Pw, &pp, &qp, &r, &s, &UP, &VP );
                    N_SrfGetKnotVectors( &surPW, &kup, &kvp );
                }
                else
                {
                    error = N_SrfElevateDegree( &surQW, qp - qq, NL_VDIR, &surQW, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_SrfGetCPtsDegreesAndKnots( &surQW, &n, &m, &Qw, &pq, &qq, &r, &s, &UQ, &VQ );
                    N_SrfGetKnotVectors( &surQW, &kuq, &kvq );
                }
            }

            /* Refine surfaces */

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
                error = N_SrfInsertKnots( &surPW, kxp, NL_UDIR, &surPW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SrfGetCPtsDegreesAndKnots( &surPW, &n, &m, &Pw, &pp, &qp, &r, &s, &UP, &VP );
            }

            if( myp GE 0 )
            {
                error = N_SrfInsertKnots( &surPW, kyp, NL_VDIR, &surPW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SrfGetCPtsDegreesAndKnots( &surPW, &n, &m, &Pw, &pp, &qp, &r, &s, &UP, &VP );
            }

            if( mxq GE 0 )
            {
                error = N_SrfInsertKnots( &surQW, kxq, NL_UDIR, &surQW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SrfGetCPtsDegreesAndKnots( &surQW, &n, &m, &Qw, &pq, &qq, &r, &s, &UQ, &VQ );
            }

            if( myq GE 0 )
            {
                error = N_SrfInsertKnots( &surQW, kyq, NL_VDIR, &surQW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_SrfGetCPtsDegreesAndKnots( &surQW, &n, &m, &Qw, &pq, &qq, &r, &s, &UQ, &VQ );
            }

            /* Check memory of output surface */

            error = N_SrfSizeArrays( surR, n, m, pp, qp, r, s, rname, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetCPtsAndKnots( surR, &Rw, &UR, &VR );

            /* Compute output surface */

            switch( opr )
            {
                case NL_PLUS:
                    for ( i = 0; i <= n; i++ )
                    {
                        for ( j = 0; j <= m; j++ )
                            N_Sum2CPts( Pw[i][j], Qw[i][j], &Rw[i][j] );
                    }
                    break;

                case NL_MINUS:
                    for ( i = 0; i <= n; i++ )
                    {
                        for ( j = 0; j <= m; j++ )
                            N_Diff2CPts( Pw[i][j], Qw[i][j], &Rw[i][j] );
                    }
                    break;

                default:

                    NL_ERROR( NL_CAL_ERR );
            }

            for ( i = 0; i <= r; i++ )
                UR[i] = UP[i];

            for ( j = 0; j <= s; j++ )
                VR[j] = VP[j];
            break;

        default:

            NL_ERROR( NL_INP_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SumDiffTwoSrfs */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a constant 
     and a surface. A typical calling example is:

       NL_REAL     alpha; 
       NL_SURFACE  sur;
       ...
       (define sur and alpha);
       ...
       N_ConstantMultiplySrf(alpha,&sur);

     THE  PRODUCT  IS  COMPUTED IN-PLACE,  I.E. THE ORIGINAL NL_SURFACE IS 
     DESTROYED.


   ACCESS:
   
     alpha , input  ,  Scalar
     sur   , in/out ,  Product of alpha and sur


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ConstantMultiplySrf( NL_REAL alpha, NL_SURFACE *sur )
{
    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get locals */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Compute product */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            N_ScaleCPtXYZ( alpha, Pw[i][j], &Pw[i][j] );
    }
} /* end N_ConstantMultiplySrf */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators  routine computes the  combination of two
     surfaces, ie it computes alpha*surP +- beta*surQ = surR. A typical 
     calling example is:

       NL_REAL     alpha, beta; 
       NL_SURFACE  surP, surQ, surR;
       NL_STACKS   SG;
       ...
       (define surP and surQ; get alpha and beta);
       ...
       N_CombineTwoSrfs(alpha,&surP,beta,&surQ,NL_PLUS,&surR,&SG);


   ACCESS:
   
     alpha , input  ,  Scalar
     surP  , input  ,  NURBS surface
     beta  , input  ,  Scalar
     surQ  , input  ,  NURBS surface
     opr   , input  ,  Flag:
                         NL_PLUS : sum
                         NL_MINUS: difference
     surR  , output ,  Combination alpha*surP +- beta*surQ
     SG    , input  ,  surR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CombineTwoSrfs( NL_REAL alpha, NL_SURFACE *surP, NL_REAL beta, NL_SURFACE *surQ, NL_FLAG opr, NL_SURFACE *surR, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_SURFACE surPW, surQW;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Scale surfaces */

    N_SrfInitArrays( &surPW );
    error = N_SrfCopy( surP, &surPW, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &surQW );
    error = N_SrfCopy( surQ, &surQW, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_ConstantMultiplySrf( alpha, &surPW );
    N_ConstantMultiplySrf( beta, &surQW );

    /* Compute sum/difference */

    error = N_SumDiffTwoSrfs( &surPW, &surQW, opr, surR, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CombineTwoSrfs */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the sum/difference of a 
     NURBS surface and a vector. A typical calling example is:

       NL_SURFACE  sur;
       NL_VECTOR   K;
       ...
       (define sur and get K);
       ...
       N_SumDiffSrfVector(&sur,K,NL_PLUS);

     THE  COMPUTATION IS DONE  IN-PLACE, I.E. THE ORIGINAL NL_SURFACE IS 
     DESTROYED.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     K   , input  ,  Vector
     opr , input  ,  Operator flag:
                       NL_PLUS : sum
                       NL_MINUS: difference


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SumDiffSrfVector( NL_SURFACE *sur, NL_VECTOR K, NL_FLAG opr )
{
    NL_PRIVATE NL_STRING rname = _T("N_SumDiffSrfVector");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL w;

    NL_CPOINT ** Pw;

    NL_VECTOR L;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute sum/difference */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    if( N_IsSrfRat( sur ) )
    {
        switch( opr )
        {
            case NL_PLUS:
                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                    {
                        N_CPtGetW( Pw[i][j], &w );
                        N_VectorScale( K, w, &L );
                        N_SumCPtAndPt( Pw[i][j], L, &Pw[i][j] );
                    }
                }
                break;

            case NL_MINUS:
                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                    {
                        N_CPtGetW( Pw[i][j], &w );
                        N_VectorScale( K, w, &L );
                        N_DiffCPtPt( Pw[i][j], L, &Pw[i][j] );
                    }
                }
                break;

            default:

                NL_ERROR( NL_CAL_ERR );
        }
    }
    else
    {
        switch( opr )
        {
            case NL_PLUS:
                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                        N_SumCPtAndPt( Pw[i][j], K, &Pw[i][j] );
                }
                break;

            case NL_MINUS:
                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                        N_DiffCPtPt( Pw[i][j], K, &Pw[i][j] );
                }
                break;

            default:

                NL_ERROR( NL_CAL_ERR );
        }
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SumDiffSrfVector */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the derivative surface of 
     a NON-RATIONAL surface. The maximum derivative allowed is equal to 
     the degree. A typical calling example is:

       NL_SURFACE  sur, der;
       NL_INDEX    du, dv;
       NL_STACKS   SG;
       ...
       (define sur; get highest derivative indexes du and dv);
       ...
       N_SrfInitArrays(&der);
       N_FirstDerivSrfNonRatSrf(&sur,du,dv,&der,&SG);


   ACCESS:
   
     sur   , input  ,  NON-RATIONAL surface
     du,dv , input  ,  Highest derivatives  required (MUST BE LESS THAN 
                       OR EQUAL TO THE DEGREES)
     der   , output ,  Derivative surface
     SG    , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FirstDerivSrfNonRatSrf( NL_SURFACE *sur, NL_INDEX du, NL_INDEX dv, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FirstDerivSrfNonRatSrf");

    NL_FLAG error = NL_NO, uder = NL_NO;

    NL_INDEX i, j, k, l, n, m, r, s, nd, md;

    NL_DEGREE p, q, pd, qd;

    NL_REAL *U, *V, *UD, *VD, alf, bet;

    NL_CPOINT ** Pw, ** Dw, ** PUw, ** PVw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    if( du LT 0 OR du GT p )
        NL_ERROR( NL_INP_ERR );

    if( dv LT 0 OR dv GT q )
        NL_ERROR( NL_INP_ERR );

    if( N_IsSrfRat( sur ) )
        NL_ERROR( NL_INP_ERR );

    /* No derivatives required */

    if( du EQ 0 AND dv EQ 0 )
    {
        error = N_SrfCopy( sur, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* See if memory is needed */

    nd = n - du;
    md = m - dv;
    pd = (NL_DEGREE)(p - du);
    qd = (NL_DEGREE)(q - dv);

    error = N_SrfSizeArrays( der, nd, md, pd, qd, nd + pd + 1, md + qd + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( der, &Dw, &UD, &VD );

    /* Initialize control points */

    PUw = N_AllocCPt2dArray( n, m, &SL );

    if( PUw EQ NULL )
        NL_QUIT;

    PVw = N_AllocCPt2dArray( n, m, &SL );

    if( PVw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CopyCPt( Pw[i][j], &PUw[i][j] );
            N_CopyCPt( Pw[i][j], &PVw[i][j] );
        }
    }

    /* Compute control points of derivative surface */

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

                if( N_FloatOpIsBad( (NL_REAL)p - (NL_REAL)k + 1.0, U[i + p + 1] - U[i + k], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                alf = ((NL_REAL)p - (NL_REAL)k + 1.0) / (U[i + p + 1] - U[i + k]);

                for ( j = 0; j <= md; j++ )
                {
                    if( l LE dv + 1 )
                    {
                        N_Combine2CPts( alf, PVw[i + 1][j], -alf, PVw[i][j], &PUw[i][j] );
                    }
                    else
                    {
                        N_Combine2CPts( alf, PUw[i + 1][j], -alf, PUw[i][j], &PUw[i][j] );
                    }
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

                if( N_FloatOpIsBad( (NL_REAL)q - (NL_REAL)l + 1.0, V[j + q + 1] - V[j + l], NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                bet = ((NL_REAL)q - (NL_REAL)l + 1.0) / (V[j + q + 1] - V[j + l]);

                for ( i = 0; i <= nd; i++ )
                {
                    if( k LE du )
                    {
                        N_Combine2CPts( bet, PUw[i][j + 1], -bet, PUw[i][j], &PVw[i][j] );
                    }
                    else
                    {
                        N_Combine2CPts( bet, PVw[i][j + 1], -bet, PVw[i][j], &PVw[i][j] );
                    }
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
                N_CopyCPt( PUw[i][j], &Dw[i][j] );
        }
    }
    else
    {
        for ( i = 0; i <= nd; i++ )
        {
            for ( j = 0; j <= md; j++ )
                N_CopyCPt( PVw[i][j], &Dw[i][j] );
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
} /* end N_FirstDerivSrfNonRatSrf */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes  the  first derivative 
     surface of a  rational surface in either the u- or the v-direction. 
     The result is obtained by using other symbolic operators to compute 
     the following expression:

         ( N )'   N_a*D - N*D_a
         ( - )  = -------------
         ( D )         D*D     

     where  N_a and D_a denote the partial derivatives with respect to u 
     or v. A typical calling example is:

       NL_SURFACE  sur, der;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&der);
       N_FirstDerivSrfRatSrf(&sur,NL_UDIR,&der,&SG);


   ACCESS:
   
     sur , input  ,  NURBS surface
     dir , input  ,  Flag:
                       NL_UDIR: compute u-partial
                       NL_VDIR: compute v-partial
     der , output ,  Derivative of sur
     SG  , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FirstDerivSrfRatSrf( NL_SURFACE *sur, NL_FLAG dir, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_FirstDerivSrfRatSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX nsp, du, dv, mx;

    NL_DEGREE p, q;

    NL_REAL *X;

    NL_SURFACE n, np, npd, ndp, num;

    NL_SFUN d, dp, den;

    NL_KNOTVECTOR *knu = NULL, *knv = NULL, *knx = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get derivative indexes */

    switch( dir )
    {
        case NL_UDIR:
            du = 1;
            dv = 0;
            break;

        case NL_VDIR:
            du = 0;
            dv = 1;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Check if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_FirstDerivSrfNonRatSrf( sur, du, dv, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_SrfInitArrays( &n );
    N_SFuncInitArrays( &d );
    error = N_SrfGetNumAndDen( sur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output surface */

    N_SFuncInitArrays( &den );
    error = N_SrfFuncMultiplySrfFunc( &d, &d, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnotVectors( &den, &knu, &knv );
    N_SFuncGetDegrees( &den, &p, &q );

    switch( dir )
    {
        case NL_UDIR:

            N_BasisGetSpanCount( knu, p, &nsp );

            knx = N_AllocKnotVectorAndArray( nsp, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            error = N_BasisIncreaseKnotMult( knu, p, 1, knx );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_VDIR:

            N_BasisGetSpanCount( knv, q, &nsp );

            knx = N_AllocKnotVectorAndArray( nsp, &SL );

            if( knx EQ NULL )
                NL_QUIT;

            error = N_BasisIncreaseKnotMult( knv, q, 1, knx );

            if( error EQ NL_YES )
                NL_OUT;
            break;
    }

    N_KnotVectorGetKnots( knx, &mx, &X );

    if( mx GE 0 )
    {
        error = N_SrfFuncRefine( &den, knx, dir, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute terms in the numerator */

    N_SrfInitArrays( &np );
    error = N_FirstDerivSrfNonRatSrf( &n, du, dv, &np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &dp );
    error = N_SrfFuncDerivFunc( &d, du, dv, &dp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &npd );
    error = N_SrfFuncMultiplySrf( &d, &np, &npd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ndp );
    error = N_SrfFuncMultiplySrf( &dp, &n, &ndp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_SrfInitArrays( &num );
    error = N_SumDiffTwoSrfs( &npd, &ndp, NL_MINUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Degree elevate numerator */

    error = N_SrfElevateDegree( &num, 1, dir, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output surface */

    error = N_CreateSrfFromNumAndDen( &num, &den, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_FirstDerivSrfRatSrf */


/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators routine  computes the  (k,l)-th derivative 
     surface of a NURBS surface. If the surface is non rational, control 
     point differencing is applied. If it is rational, then the (i,j)-th  
     derivative surface is obtained  by  differentiating  the (i,j-1)-th 
     derivative  surface in  v-direction or  the  (i-1,j)-th  derivative 
     surface in u-direction. A typical calling example is:

       NL_SURFACE  sur, der;
       NL_INDEX    k, l;
       NL_STACKS   SG;
       ...
       (define sur; get k and l);
       ...
       N_SrfInitArrays(&der);
       N_KDerivSrf(&sur,k,l,&der,&SG);

     IT IS SUGGESTED TO  ALLOW THE ROUTINE TO  ALLOCATE  MEMORY FOR  der
     INTERNALLY. NL_MAXIMUM DERIVATIVES ARE DEGREES NL_MINUS  NL_MAXIMUM INTERNAL
     KNOT MULTIPLICITIES.


   ACCESS:
   
     sur , input  ,  NURBS surface
     k,l , input  ,  Highest derivatives (MUST BE LESS THAN THE SURFACE 
                     DEGREES FOR  NON-RATIONAL AND DEGREES MINUS ONE FOR
                     RATIONAL)
     der , output ,  Derivative surface
     SG  , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_KDerivSrf( NL_SURFACE *sur, NL_INDEX k, NL_INDEX l, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_KDerivSrf");

    NL_FLAG error = NL_NO, derA; 

    NL_INDEX i, j;

    NL_DEGREE p, q;

    NL_SURFACE surA, surB, surC;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_FirstDerivSrfNonRatSrf( sur, k, l, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Compute derivatives */

    N_SrfGetDegrees( sur, &p, &q );

    if( k LT 0 OR k GT p - 1 )
        NL_ERROR( NL_INP_ERR );

    if( l LT 0 OR l GT q - 1 )
        NL_ERROR( NL_INP_ERR );

    N_SrfInitArrays( &surA );
    error = N_SrfCopy( sur, &surA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &surB );
    error = N_SrfCopy( sur, &surB, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    derA = NL_YES;
    i = 1;
    j = 1;

    while( i LE k OR j LE l )
    {
        if( i LE k )
        {
            if( j LE l + 1 )
            {
                N_SrfInitArrays( &surB );
                error = N_FirstDerivSrfRatSrf( &surA, NL_UDIR, &surB, SG );

                if( error EQ NL_YES )
                    NL_OUT;
                N_FreeSrf( &surA, &SL );
            }
            else
            {
                N_SrfInitArrays( &surC );
                error = N_SrfCopy( &surB, &surC, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                N_FreeSrf( &surB, &SL );

                N_SrfInitArrays( &surB );
                error = N_FirstDerivSrfRatSrf( &surC, NL_UDIR, &surB, SG );

                if( error EQ NL_YES )
                    NL_OUT;
                N_FreeSrf( &surC, &SL );
            }
            derA = NL_NO;
        }

        if( j LE l )
        {
            if( i LE k )
            {
                N_SrfInitArrays( &surA );
                error = N_FirstDerivSrfRatSrf( &surB, NL_VDIR, &surA, SG );

                if( error EQ NL_YES )
                    NL_OUT;
                N_FreeSrf( &surB, &SL );
            }
            else
            {
                N_SrfInitArrays( &surC );
                error = N_SrfCopy( &surA, &surC, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                N_FreeSrf( &surA, &SL );

                N_SrfInitArrays( &surA );
                error = N_FirstDerivSrfRatSrf( &surC, NL_VDIR, &surA, SG );

                if( error EQ NL_YES )
                    NL_OUT;
                N_FreeSrf( &surC, &SL );
            }
            derA = NL_YES;
        }

        i++;
        j++;
    }

    if( derA EQ NL_YES )
    {
        error = N_SrfCopy( &surA, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_SrfCopy( &surB, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_KDerivSrf */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes all derivative surfaces of 
     a  NON-RATIONAL  surface up to  specified  derivatives. The maximum  
     derivatives  allowed are  equal to  the degrees. A  typical calling 
     example is:

       NL_SURFACE  sur, ***der;
       NL_INDEX    du, dv;
       NL_STACKS   SG;
       ...
       (define sur; get highest derivative indexes du and dv);
       ...
       N_AllDerivSrfNonRatSrf(&sur,du,dv,&der,&SG);

     der[0][0], der[1][0],..., der[du][dv] are  pointers to the (0,0)th,
     ..., (du,dv)th derivative surfaces. MEMORY TO  STORE THESE SURFACES 
     IS ALLOCTED  INSIDE THE  ROUTINE. NL_MAXIMUM  DERIVATIVES  ARE DEGREES  
     NL_MINUS THE NL_MAXIMUM INTERNAL KNOT MULTIPLICITIES.


   ACCESS:
   
     sur   , input  ,  NON-RATIONAL surface
     du,dv , input  ,  Highest derivatives  required (MUST BE  LESS THAN 
                       OR EQUAL TO THE DEGREES)
     der   , output ,  Derivative surfaces
     SG    , input  ,  ders' stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllDerivSrfNonRatSrf( NL_SURFACE *sur, NL_INDEX du, NL_INDEX dv, NL_SURFACE **** der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllDerivSrfNonRatSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX k, l;

    NL_DEGREE p, q;

    NL_SURFACE *** surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_SrfGetDegrees( sur, &p, &q );

    if( du LT 0 OR du GT p )
        NL_ERROR( NL_INP_ERR );

    if( dv LT 0 OR dv GT q )
        NL_ERROR( NL_INP_ERR );

    if( N_IsSrfRat( sur ) )
        NL_ERROR( NL_INP_ERR );

    /* Get derivative surfaces */

    surA = N_Alloc2dArraySrfPtrs( du, dv, SG );

    if( surA EQ NULL )
        NL_QUIT;

    for ( k = 0; k <= du; k++ )
    {
        for ( l = 0; l <= dv; l++ )
        {
            surA[k][l] = N_AllocSrf( SG );

            if( surA[k][l]EQ NULL )
                NL_QUIT;
        }
    }

    N_SrfInitArrays( surA[0][0] );
    error = N_SrfCopy( sur, surA[0][0], SG );

    if( error EQ NL_YES )
        NL_OUT;

    for ( k = 0; k <= du; k++ )
    {
        if( k GT 0 )
        {
            N_SrfInitArrays( surA[k][0] );
            error = N_FirstDerivSrfNonRatSrf( surA[k - 1][0], 1, 0, surA[k][0], SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( l = 0; l <= dv; l++ )
        {
            if( l GT 0 )
            {
                N_SrfInitArrays( surA[k][l] );
                error = N_FirstDerivSrfNonRatSrf( surA[k][l - 1], 0, 1, surA[k][l], SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    *der = surA;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_AllDerivSrfNonRatSrf */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes all derivative surfaces of 
     a NURBS  surface up to  specified derivatives. Maximum  derivatives  
     allowed are  equal to  the degrees  for non-rational, and  equal to 
     degrees minus one for rational. A typical calling example is:

       NL_SURFACE  sur, ***der;
       NL_INDEX    du, dv;
       NL_STACKS   SG;
       ...
       (define sur; get highest derivative indexes du and dv);
       ...
       N_AllDerivSrfNurbsSrf(&sur,du,dv,&der,&SG);

     der[0][0], der[1][0],..., der[du][dv] are  pointers to the (0,0)th,
     ..., (du,dv)th derivative surfaces. MEMORY TO  STORE THESE SURFACES 
     IS ALLOCTED  INSIDE THE  ROUTINE. 


   ACCESS:
   
     sur   , input  ,  NURBS surface
     du,dv , input  ,  Highest derivatives  required (MUST BE  LESS THAN 
                       OR EQUAL  TO THE  DEGREES  FOR  NON-RATIONAL, AND 
                       DEGREES NL_MINUS ONE FOR RATIONAL)
     der   , output ,  Derivative surfaces
     SG    , input  ,  ders' stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllDerivSrfNurbsSrf( NL_SURFACE *sur, NL_INDEX du, NL_INDEX dv, NL_SURFACE **** der, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX k, l;

    NL_SURFACE *** surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_AllDerivSrfNonRatSrf( sur, du, dv, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Get derivative surfaces */

    surA = N_Alloc2dArraySrfPtrs( du, dv, SG );

    if( surA EQ NULL )
        NL_QUIT;

    for ( k = 0; k <= du; k++ )
    {
        for ( l = 0; l <= dv; l++ )
        {
            surA[k][l] = N_AllocSrf( SG );

            if( surA[k][l]EQ NULL )
                NL_QUIT;
        }
    }

    N_SrfInitArrays( surA[0][0] );
    error = N_SrfCopy( sur, surA[0][0], SG );

    if( error EQ NL_YES )
        NL_OUT;

    for ( k = 0; k <= du; k++ )
    {
        if( k GT 0 )
        {
            N_SrfInitArrays( surA[k][0] );
            error = N_FirstDerivSrfRatSrf( surA[k - 1][0], NL_UDIR, surA[k][0], SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        for ( l = 0; l <= dv; l++ )
        {
            if( l GT 0 )
            {
                N_SrfInitArrays( surA[k][l] );
                error = N_FirstDerivSrfRatSrf( surA[k][l - 1], NL_VDIR, surA[k][l], SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    *der = surA;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_AllDerivSrfNurbsSrf */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators routine computes  the normal surface of a 
     NURBS surface. A typical calling example is:

       NL_SURFACE  sur, nor;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&nor);
       N_NormalSrfNurbsSrf_UU(&sur,&nor,&SG);

     THE  DEGREES  OF THE  NORMAL  NL_SURFACE ARE  (2*p-1,2*q-1)  FOR  NON 
     RATIONAL,  AND  (4*p,4*q)  FOR  RATIONAL,  WHERE  p AND  q ARE THE 
     ORIGINAL NL_SURFACE DEGREES.


   ACCESS:
   
     sur  , input  ,  NURBS surface
     nor  , output ,  Normal surface
     SG   , input  ,  nor's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_NormalSrfNurbsSrf_UU( NL_SURFACE *sur, NL_SURFACE *nor, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_SURFACE Su, Sv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get partial derivative surfaces */

    N_SrfInitArrays( &Su );
    error = N_FirstDerivSrfRatSrf( sur, NL_UDIR, &Su, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &Sv );
    error = N_FirstDerivSrfRatSrf( sur, NL_VDIR, &Sv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get cross product surfaces */

    error = N_CrossProductTwoSrfs( &Su, &Sv, nor, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_NormalSrfNurbsSrf_UU */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the  second derivative 
     surface  (Suu) of a  rational  surface. The  result is  obtained by  
     using other symbolic operators to compute the following expression:

         ( N )     D*D*N_uu- 2*D*D_u*N_u+N*(2*D_u*D_u-D*D_uu)
         ( - )   = ------------------------------------------
         ( D )uu                    D*D*D     

     A typical calling example is:

       NL_SURFACE  sur, Suu;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&Suu);
       N_SecondDerivSrfRatSrf(&sur,&Suu,&SG);


   ACCESS:
   
     sur , input  ,  NURBS surface
     Suu , output ,  Derivative of sur
     SG  , input  ,  Suu's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SecondDerivSrfRatSrf( NL_SURFACE *sur, NL_SURFACE *Suu, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX nup, rx;

    NL_DEGREE p, q;

    NL_REAL *X;

    NL_SURFACE n, nu, nuu, ddnuu, ddunu, dduun, nsum, num;

    NL_SFUN d, dd, du, duu, ddu, dudu, dduu, dsum, den;

    NL_KNOTVECTOR *knu, *knv, *knx;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_FirstDerivSrfNonRatSrf( sur, 2, 0, Suu, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_SrfInitArrays( &n );
    N_SFuncInitArrays( &d );
    error = N_SrfGetNumAndDen( sur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output surface */

    N_SFuncInitArrays( &dd );
    error = N_SrfFuncMultiplySrfFunc( &d, &d, &dd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &den );
    error = N_SrfFuncMultiplySrfFunc( &d, &dd, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnotVectors( &den, &knu, &knv );
    N_SFuncGetDegrees( &den, &p, &q );
    N_BasisGetSpanCount( knu, p, &nup );

    knx = N_AllocKnotVectorAndArray( 2 * nup, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knu, p, 2, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &rx, &X );

    if( rx GE 0 )
    {
        error = N_SrfFuncRefine( &den, knx, NL_UDIR, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute terms in the numerator */

    /*--------- DDNuu ------------*/

    N_SrfInitArrays( &nuu );
    error = N_FirstDerivSrfNonRatSrf( &n, 2, 0, &nuu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ddnuu );
    error = N_SrfFuncMultiplySrf( &dd, &nuu, &ddnuu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*--------- 2DDuNu ------------*/

    N_SFuncInitArrays( &du );
    error = N_SrfFuncDerivFunc( &d, 1, 0, &du, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &ddu );
    error = N_SrfFuncMultiplySrfFunc( &d, &du, &ddu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfFuncMultiplyConstant( 2.0, &ddu );

    N_SrfInitArrays( &nu );
    error = N_FirstDerivSrfNonRatSrf( &n, 1, 0, &nu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ddunu );
    error = N_SrfFuncMultiplySrf( &ddu, &nu, &ddunu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnotVectors( &ddunu, &knu, &knv );
    N_SrfGetDegrees( &ddunu, &p, &q );
    N_BasisGetSpanCount( knu, p, &nup );

    knx = N_AllocKnotVectorAndArray( nup, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knu, p, 1, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &rx, &X );

    if( rx GE 0 )
    {
        error = N_SrfInsertKnots( &ddunu, knx, NL_UDIR, &ddunu, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /*--------- N(2DuDu-DDuu) ------------*/

    N_SFuncInitArrays( &dudu );
    error = N_SrfFuncMultiplySrfFunc( &du, &du, &dudu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfFuncMultiplyConstant( 2.0, &dudu );

    N_SFuncGetKnotVectors( &dudu, &knu, &knv );
    N_SFuncGetDegrees( &dudu, &p, &q );
    N_BasisGetSpanCount( knu, p, &nup );

    knx = N_AllocKnotVectorAndArray( nup, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knu, p, 1, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &rx, &X );

    if( rx GE 0 )
    {
        error = N_SrfFuncRefine( &dudu, knx, NL_UDIR, &dudu, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SFuncInitArrays( &duu );
    error = N_SrfFuncDerivFunc( &d, 2, 0, &duu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &dduu );
    error = N_SrfFuncMultiplySrfFunc( &d, &duu, &dduu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &dsum );
    error = N_SrfFuncSumDiffSrfFunc( &dudu, &dduu, NL_MINUS, NL_YES, &dsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &dduun );
    error = N_SrfFuncMultiplySrf( &dsum, &n, &dduun, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_SrfInitArrays( &nsum );
    error = N_SumDiffTwoSrfs( &ddnuu, &ddunu, NL_MINUS, &nsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &num );
    error = N_SumDiffTwoSrfs( &nsum, &dduun, NL_PLUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Degree elevate numerator */

    error = N_SrfElevateDegree( &num, 2, NL_UDIR, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output surface */

    error = N_CreateSrfFromNumAndDen( &num, &den, Suu, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SecondDerivSrfRatSrf */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine  computes the mixed partial deriva-
     tive surface (Suv) of a rational surface. The result is obtained by  
     using other symbolic operators to compute the following expression:

         ( N )     D*D*N_uv-D*(N_u*D_v+N_v*D_u+N*D_uv)+2*N*D_u*D_v
         ( - )   = -----------------------------------------------
         ( D )uv                        D*D*D     

     A typical calling example is:

       NL_SURFACE  sur, Suv;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&Suv);
       N_MixedPartialDerivSrfRatSrf_UV(&sur,&Suv,&SG);


   ACCESS:
   
     sur , input  ,  NURBS surface
     Suv , output ,  Derivative of sur
     SG  , input  ,  Suv's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MixedPartialDerivSrfRatSrf_UV( NL_SURFACE *sur, NL_SURFACE *Suv, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX nup, nvq, rx, sy;

    NL_DEGREE p, q;

    NL_REAL *X, *Y;

    NL_SURFACE n, nu, nv, nuv, ddnuv, ddana, dudvn, nudv, nvdu, nduv, sum1, sum2, sum3, num;

    NL_SFUN d, dd, du, dv, duv, dudv, den;

    NL_KNOTVECTOR *knu, *knv, *knx, *kny;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_FirstDerivSrfNonRatSrf( sur, 1, 1, Suv, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_SrfInitArrays( &n );
    N_SFuncInitArrays( &d );
    error = N_SrfGetNumAndDen( sur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output surface */

    N_SFuncInitArrays( &dd );
    error = N_SrfFuncMultiplySrfFunc( &d, &d, &dd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &den );
    error = N_SrfFuncMultiplySrfFunc( &d, &dd, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnotVectors( &den, &knu, &knv );
    N_SFuncGetDegrees( &den, &p, &q );
    N_BasisGetSpanCount( knu, p, &nup );
    N_BasisGetSpanCount( knv, q, &nvq );

    knx = N_AllocKnotVectorAndArray( nup, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    kny = N_AllocKnotVectorAndArray( nvq, &SL );

    if( kny EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knu, p, 1, knx );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIncreaseKnotMult( knv, q, 1, kny );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &rx, &X );
    N_KnotVectorGetKnots( kny, &sy, &Y );

    if( rx GE 0 )
    {
        error = N_SrfFuncRefine( &den, knx, NL_UDIR, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( sy GE 0 )
    {
        error = N_SrfFuncRefine( &den, kny, NL_VDIR, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute various derivatives */

    N_SrfInitArrays( &nu );
    N_SFuncInitArrays( &du );
    error = N_FirstDerivSrfNonRatSrf( &n, 1, 0, &nu, &SL );
    error = N_SrfFuncDerivFunc( &d, 1, 0, &du, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &nv );
    N_SFuncInitArrays( &dv );
    error = N_FirstDerivSrfNonRatSrf( &n, 0, 1, &nv, &SL );
    error = N_SrfFuncDerivFunc( &d, 0, 1, &dv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &nuv );
    N_SFuncInitArrays( &duv );
    error = N_FirstDerivSrfNonRatSrf( &n, 1, 1, &nuv, &SL );
    error = N_SrfFuncDerivFunc( &d, 1, 1, &duv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute terms in the numerator */

    /*--------- DDNuv ------------*/

    N_SrfInitArrays( &ddnuv );
    error = N_SrfFuncMultiplySrf( &dd, &nuv, &ddnuv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*----- D(NuDv+NvDu+NDuv) -----*/

    N_SrfInitArrays( &nudv );
    error = N_SrfFuncMultiplySrf( &dv, &nu, &nudv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &nvdu );
    error = N_SrfFuncMultiplySrf( &du, &nv, &nvdu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &nduv );
    error = N_SrfFuncMultiplySrf( &duv, &n, &nduv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &sum1 );
    error = N_SumDiffTwoSrfs( &nudv, &nvdu, NL_PLUS, &sum1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &sum2 );
    error = N_SumDiffTwoSrfs( &sum1, &nduv, NL_PLUS, &sum2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ddana );
    error = N_SrfFuncMultiplySrf( &d, &sum2, &ddana, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*--------- 2NDuDv ------------*/

    N_SFuncInitArrays( &dudv );
    error = N_SrfFuncMultiplySrfFunc( &du, &dv, &dudv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &dudvn );
    error = N_SrfFuncMultiplySrf( &dudv, &n, &dudvn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_ConstantMultiplySrf( 2.0, &dudvn );

    /* Add terms in the numerator */

    N_SrfInitArrays( &sum3 );
    error = N_SumDiffTwoSrfs( &ddnuv, &ddana, NL_MINUS, &sum3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &num );
    error = N_SumDiffTwoSrfs( &sum3, &dudvn, NL_PLUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Degree elevate numerator */

    error = N_SrfElevateDegree( &num, 1, NL_UDIR, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfElevateDegree( &num, 1, NL_VDIR, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output surface */

    error = N_CreateSrfFromNumAndDen( &num, &den, Suv, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_MixedPartialDerivSrfRatSrf_UV */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the  second derivative 
     surface  (Svv) of a  rational  surface. The  result is  obtained by  
     using other symbolic operators to compute the following expression:

         ( N )     D*D*N_vv- 2*D*D_v*N_v+N*(2*D_v*D_v-D*D_vv)
         ( - )   = ------------------------------------------
         ( D )vv                    D*D*D     

     A typical calling example is:

       NL_SURFACE  sur, Svv;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&Svv);
       N_SecondDerivSrfRatSrf_VV(&sur,&Svv,&SG);


   ACCESS:
   
     sur , input  ,  NURBS surface
     Svv , output ,  Derivative of sur
     SG  , input  ,  Svv's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SecondDerivSrfRatSrf_VV( NL_SURFACE *sur, NL_SURFACE *Svv, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX nvq, sy;

    NL_DEGREE p, q;

    NL_REAL *Y;

    NL_SURFACE n, nv, nvv, ddnvv, ddvnv, ddvvn, nsum, num;

    NL_SFUN d, dd, dv, dvv, ddv, dvdv, ddvv, dsum, den;

    NL_KNOTVECTOR *knu, *knv, *kny;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_FirstDerivSrfNonRatSrf( sur, 0, 2, Svv, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_SrfInitArrays( &n );
    N_SFuncInitArrays( &d );
    error = N_SrfGetNumAndDen( sur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output surface */

    N_SFuncInitArrays( &dd );
    error = N_SrfFuncMultiplySrfFunc( &d, &d, &dd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &den );
    error = N_SrfFuncMultiplySrfFunc( &d, &dd, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnotVectors( &den, &knu, &knv );
    N_SFuncGetDegrees( &den, &p, &q );
    N_BasisGetSpanCount( knv, q, &nvq );

    kny = N_AllocKnotVectorAndArray( 2 * nvq, &SL );

    if( kny EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knv, q, 2, kny );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( kny, &sy, &Y );

    if( sy GE 0 )
    {
        error = N_SrfFuncRefine( &den, kny, NL_VDIR, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute terms in the numerator */

    /*--------- DDNvv ------------*/

    N_SrfInitArrays( &nvv );
    error = N_FirstDerivSrfNonRatSrf( &n, 0, 2, &nvv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ddnvv );
    error = N_SrfFuncMultiplySrf( &dd, &nvv, &ddnvv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*--------- 2DDvNv ------------*/

    N_SFuncInitArrays( &dv );
    error = N_SrfFuncDerivFunc( &d, 0, 1, &dv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &ddv );
    error = N_SrfFuncMultiplySrfFunc( &d, &dv, &ddv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfFuncMultiplyConstant( 2.0, &ddv );

    N_SrfInitArrays( &nv );
    error = N_FirstDerivSrfNonRatSrf( &n, 0, 1, &nv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ddvnv );
    error = N_SrfFuncMultiplySrf( &ddv, &nv, &ddvnv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnotVectors( &ddvnv, &knu, &knv );
    N_SrfGetDegrees( &ddvnv, &p, &q );
    N_BasisGetSpanCount( knv, q, &nvq );

    kny = N_AllocKnotVectorAndArray( nvq, &SL );

    if( kny EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knv, q, 1, kny );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( kny, &sy, &Y );

    if( sy GE 0 )
    {
        error = N_SrfInsertKnots( &ddvnv, kny, NL_VDIR, &ddvnv, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /*--------- N(2DuDu-DDuu) ------------*/

    N_SFuncInitArrays( &dvdv );
    error = N_SrfFuncMultiplySrfFunc( &dv, &dv, &dvdv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfFuncMultiplyConstant( 2.0, &dvdv );

    N_SFuncGetKnotVectors( &dvdv, &knu, &knv );
    N_SFuncGetDegrees( &dvdv, &p, &q );
    N_BasisGetSpanCount( knv, q, &nvq );

    kny = N_AllocKnotVectorAndArray( nvq, &SL );

    if( kny EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knv, q, 1, kny );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( kny, &sy, &Y );

    if( sy GE 0 )
    {
        error = N_SrfFuncRefine( &dvdv, kny, NL_VDIR, &dvdv, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SFuncInitArrays( &dvv );
    error = N_SrfFuncDerivFunc( &d, 0, 2, &dvv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &ddvv );
    error = N_SrfFuncMultiplySrfFunc( &d, &dvv, &ddvv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &dsum );
    error = N_SrfFuncSumDiffSrfFunc( &dvdv, &ddvv, NL_MINUS, NL_YES, &dsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ddvvn );
    error = N_SrfFuncMultiplySrf( &dsum, &n, &ddvvn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_SrfInitArrays( &nsum );
    error = N_SumDiffTwoSrfs( &ddnvv, &ddvnv, NL_MINUS, &nsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &num );
    error = N_SumDiffTwoSrfs( &nsum, &ddvvn, NL_PLUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Degree elevate numerator */

    error = N_SrfElevateDegree( &num, 2, NL_VDIR, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output surface */

    error = N_CreateSrfFromNumAndDen( &num, &den, Svv, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SecondDerivSrfRatSrf_VV */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators  routine  computes  the  cross-boundary 
     derivative curve of a NON-RATIONAL surface in 3-D, or a RATIONAL 
     surface in 4-D. A typical calling example is:

       NL_CURVE    cur;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_CrvInitArrays(&cur);
       N_CrossBoundDerivCrvNonRatSrf(&sur,NL_LEFT,&cur,&SG);


   ACCESS:
   
     sur  , input  ,  NON-RATIONAL surface
     side , input  ,  Flag:
                        NL_LEFT  : derivative across u=umin
                        NL_RIGHT : derivative across u=umax
                        NL_BOTTOM: derivative across v=vmin
                        NL_TOP   : derivative across v=vmax
     cur  , output ,  Cross-boundary derivative
     SG   , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrossBoundDerivCrvNonRatSrf( NL_SURFACE *sur, NL_FLAG side, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrossBoundDerivCrvNonRatSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *US, *VS, *UC = NULL, fac;

    NL_CPOINT ** Pw, *Cw = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &US, &VS );

    /* See if memory is needed */

    if( side EQ NL_LEFT OR side EQ NL_RIGHT )
    {
        error = N_CrvSizeArrays( cur, m, q, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsDegreeAndKnots( cur, &m, &Cw, &q, &s, &UC );
    }
    else if( side EQ NL_BOTTOM OR side EQ NL_TOP )
    {
        /* allocate cur memory as needed */
        error = N_CrvSizeArrays( cur, n, p, r, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        /* get cur internal values */
        N_CrvGetCPtsDegreeAndKnots( cur, &n, &Cw, &p, &r, &UC );
    }

    /* Compute cross-derivative curves */

    switch( side )
    {
        case NL_LEFT:

            fac = p / (US[p + 1] - US[1]);

            for ( j = 0; j <= m; j++ )
            {
                N_Diff2CPts( Pw[1][j], Pw[0][j], &Cw[j] );
                N_ScaleCPt( fac, Cw[j], &Cw[j] );
            }

            for ( j = 0; j <= s; j++ )
                UC[j] = VS[j];
            break;

        case NL_RIGHT:

            fac = p / (US[n + p] - US[n]);

            for ( j = 0; j <= m; j++ )
            {
                N_Diff2CPts( Pw[n][j], Pw[n - 1][j], &Cw[j] );
                N_ScaleCPt( fac, Cw[j], &Cw[j] );
            }

            for ( j = 0; j <= s; j++ )
                UC[j] = VS[j];
            break;

        case NL_BOTTOM:

            fac = q / (VS[q + 1] - VS[1]);

            for ( i = 0; i <= n; i++ )
            {
                N_Diff2CPts( Pw[i][1], Pw[i][0], &Cw[i] );
                N_ScaleCPt( fac, Cw[i], &Cw[i] );
            }

            for ( i = 0; i <= r; i++ )
                UC[i] = US[i];
            break;

        case NL_TOP:

            fac = q / (VS[m + q] - VS[m]);

            for ( i = 0; i <= n; i++ )
            {
                /* let Cw[i] = Pw[i][m] - Pw[i][m-1] */
                N_Diff2CPts( Pw[i][m], Pw[i][m - 1], &Cw[i] );

                /* let Cw[i] = fac * Cw[i] */
                N_ScaleCPt( fac, Cw[i], &Cw[i] );
            }

            for ( i = 0; i <= r; i++ )
                UC[i] = US[i];
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrossBoundDerivCrvNonRatSrf */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This  symbolic  operators  routine  computes   the  cross-boundary 
     derivative curve of a NURBS surface. A typical calling example is:
 
       NL_CURVE    cur;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_CrvInitArrays(&cur);
       N_CrossBoundDerivCrvNurbsSrf(&sur,NL_LEFT,&cur,&SG);
 
 
   ACCESS:
   
     sur  , input  ,  NURBS surface
     side , input  ,  Flag:
                        NL_LEFT  : derivative across u=umin
                        NL_RIGHT : derivative across u=umax
                        NL_BOTTOM: derivative across v=vmin
                        NL_TOP   : derivative across v=vmax
     cur  , output ,  Cross-boundary derivative
     SG   , input  ,  cur's stack
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_CrossBoundDerivCrvNurbsSrf( NL_SURFACE *sur, NL_FLAG side, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrossBoundDerivCrvNurbsSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, ns, ms, r, s, cp = 0, kn = 0;

    NL_DEGREE p, q, dg = 0;

    NL_REAL ** w, *U, *V, *Una, *Ud, *Un, *Uda, *daw, *dw, fac;
    NL_REAL tmpWeight;

    NL_CPOINT ** Pw, *Naw, *Nw;

    NL_SURFACE num;

    NL_CURVE na, n, nad, nda, cn;

    NL_SFUN den;

    NL_CFUN d, da, cd;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals, numerator and denominator */
    N_SrfGetCPtsDegreesAndKnots( sur, &ns, &ms, &Pw, &p, &q, &r, &s, &U, &V );

    /* See if surface is rational */

    /* Note: SensAble Technologies [B Denker]
    sur has only two rows of control points filled in,
    the first-derivative band along one edge.
    N_IsSrfRat() checks the [0][0] point, which may or may not be filled in.
    If not filled in, it returns True (rational), which leads to a crash.
    if( NOT N_IsSrfRat(sur) ) */

    i = 0;
    j = 0;

    if( side EQ NL_RIGHT )
        i = ns;

    if( side EQ NL_TOP )
        j = ms;
    N_CPtGetW( Pw[i][j], &tmpWeight );

    if( tmpWeight EQ NL_NOW ) /* not rational */
    {
        error = N_CrossBoundDerivCrvNonRatSrf( sur, side, cur, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    N_SrfInitArrays( &num );
    N_SFuncInitArrays( &den );
    error = N_SrfGetNumAndDen( sur, &num, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPts( &num, &ns, &ms, &Pw );
    N_SrfFuncCntrlVal( &den, &ns, &ms, &w );

    /* Get curve memory */
    if( side EQ NL_LEFT OR side EQ NL_RIGHT )
    {
        cp = ms;
        dg = q;
        kn = s;
    }
    else if( side EQ NL_BOTTOM OR side EQ NL_TOP )
    {
        cp = ns;
        dg = p;
        kn = r;
    }
    error = N_AllocCrvArrays( &na, cp, dg, kn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocCrvArrays( &n, cp, dg, kn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocCFuncArrays( &da, cp, dg, kn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_AllocCFuncArrays( &d, cp, dg, kn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPts( &na, &cp, &Naw );
    N_CrvGetCPts( &n, &cp, &Nw );
    N_CrvFuncCntrlVal( &da, &cp, &daw );
    N_CrvFuncCntrlVal( &d, &cp, &dw );

    N_CrvGetKnots( &na, &kn, &Una );
    N_CrvGetKnots( &n, &kn, &Un );
    N_CFuncGetKnots( &da, &kn, &Uda );
    N_CFuncGetKnots( &d, &kn, &Ud );

    /* Compute cross-derivative curves */

    switch( side )
    {
        case NL_LEFT:

            fac = p / (U[p + 1] - U[1]);

            for ( j = 0; j <= ms; j++ )
            {
                N_Diff2CPts( Pw[1][j], Pw[0][j], &Naw[j] );
                N_ScaleCPt( fac, Naw[j], &Naw[j] );

                dw[j] = w[0][j];

                N_CopyCPt( Pw[0][j], &Nw[j] );

                daw[j] = fac * (w[1][j] - w[0][j]);
            }

            for ( j = 0; j <= s; j++ )
                Una[j] = Un[j] = Uda[j] = Ud[j] = V[j];
            break;

        case NL_RIGHT:

            fac = p / (U[ns + p] - U[ns]);

            for ( j = 0; j <= ms; j++ )
            {
                N_Diff2CPts( Pw[ns][j], Pw[ns - 1][j], &Naw[j] );
                N_ScaleCPt( fac, Naw[j], &Naw[j] );

                dw[j] = w[ns][j];

                N_CopyCPt( Pw[ns][j], &Nw[j] );

                daw[j] = fac * (w[ns][j] - w[ns - 1][j]);
            }

            for ( j = 0; j <= s; j++ )
                Una[j] = Un[j] = Uda[j] = Ud[j] = V[j];
            break;

        case NL_BOTTOM:

            fac = q / (V[q + 1] - V[1]);

            for ( i = 0; i <= ns; i++ )
            {
                N_Diff2CPts( Pw[i][1], Pw[i][0], &Naw[i] );
                N_ScaleCPt( fac, Naw[i], &Naw[i] );

                dw[i] = w[i][0];

                N_CopyCPt( Pw[i][0], &Nw[i] );

                daw[i] = fac * (w[i][1] - w[i][0]);
            }

            for ( i = 0; i <= r; i++ )
                Una[i] = Un[i] = Uda[i] = Ud[i] = U[i];
            break;

        case NL_TOP:

            fac = q / (V[ms + q] - V[ms]);

            for ( i = 0; i <= ns; i++ )
            {
                N_Diff2CPts( Pw[i][ms], Pw[i][ms - 1], &Naw[i] );
                N_ScaleCPt( fac, Naw[i], &Naw[i] );

                dw[i] = w[i][ms];

                N_CopyCPt( Pw[i][ms], &Nw[i] );

                daw[i] = fac * (w[i][ms] - w[i][ms - 1]);
            }

            for ( i = 0; i <= r; i++ )
                Una[i] = Un[i] = Uda[i] = Ud[i] = U[i];
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    /* Compute numerator and denominator of cross-boundary */

    N_CrvInitArrays( &nad );
    error = N_CrvFuncMultiplyCrv( &d, &na, &nad, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &nda );
    error = N_CrvFuncMultiplyCrv( &da, &n, &nda, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &cn );
    error = N_CrvSumDiffCrv( &nad, &nda, NL_MINUS, &cn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &cd );
    error = N_CrvFuncMultiplyCrvFunc( &d, &d, &cd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CreateCrvFromNumAndDenom( &cn, &cd, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrossBoundDerivCrvNurbsSrf */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes an upper bound on the 
     difference of two surfaces. That is, it computes

        | S_1(u,v) - S_2(u,v) | < bound

     A typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_REAL     bnd; 
       ...
       (define surP and surQ);
       ...
       N_MaxDiffTwoSrfs(&surP,&surQ,&bnd);


   ACCESS:
   
     surP , input  ,  NURBS surface
     surQ , input  ,  NURBS surface
     bnd  , output ,  Bound of |surP-surQ|


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MaxDiffTwoSrfs( NL_SURFACE *surP, NL_SURFACE *surQ, NL_REAL *bnd )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l;

    NL_REAL len;

    NL_POINT P;

    NL_SURFACE *** bez, surR;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get difference of two surfaces */

    N_SrfInitArrays( &surR );
    error = N_SumDiffTwoSrfs( surP, surQ, NL_MINUS, &surR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier pieces */

    error = N_SrfDecomposeToBez( &surR, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute bound */

    *bnd = 0.0;

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            N_SrfMaxMagnitudePosVectors( bez[i][j], &P, &len );

            if( len GT *bnd )
                *bnd = len;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_MaxDiffTwoSrfs */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators routine  computes an  upper  bound on the 
     change of a surface obtained by  moving a knot. A  typical calling 
     example is:

       NL_SURFACE  sur;
       NL_INDEX    k, l;
       NL_REAL     du, dv, bnd; 
       ...
       (define sur, get k, l, du and dv);
       ...
       N_SrfMaxChangeMovingKnot(&sur,k,l,du,dv,NL_UDIR,&bnd);
 

   ACCESS:
   
     sur , input  ,  NURBS curve
     k,l , input  ,  Indeces of knots to be moved
     du  , input  ,  Distance u_k to be moved:
                       du > 0.0: the right  knot of a  multiple knot is 
                                  moved to the right by du
                       du < 0.0: the  left  knot of a  multiple knot is 
                                  moved to the left by du
     dv  , input  ,  Distance v_l to be moved:
                       dv > 0.0: the right  knot of a  multiple knot is 
                                  moved to the right by dv
                       dv < 0.0: the  left  knot of a  multiple knot is 
                                  moved to the left by dv
     dir , input  ,  Flag:
                       NL_UDIR : move the u-knot
                       NL_VDIR : move the v-knot
                       NL_UVDIR: move both u- and v-knot
     bnd , output ,  Bound on surface change


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfMaxChangeMovingKnot( NL_SURFACE *sur, NL_INDEX k, NL_INDEX l, NL_REAL du, NL_REAL dv, NL_FLAG dir, NL_REAL *bnd )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfMaxChangeMovingKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    N_SrfGetArraySizes( sur, &n, &m, &r, &s );
    N_SrfGetKnots( sur, &r, &s, &U, &V );
    N_SrfGetDegrees( sur, &p, &q );

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        if( k LE p OR k GE n + 1 )
            NL_ERROR( NL_INP_ERR );

        if( du GT 0.0 )
        {
            if( U[k]EQ U[k + 1] )
                NL_ERROR( NL_INP_ERR );

            if( U[k] + du GE U[k + 1] )
                NL_ERROR( NL_INP_ERR );
        }
        else
        {
            if( U[k]EQ U[k - 1] )
                NL_ERROR( NL_INP_ERR );

            if( U[k] + du LE U[k - 1] )
                NL_ERROR( NL_INP_ERR );
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        if( l LE q OR l GE m + 1 )
            NL_ERROR( NL_INP_ERR );

        if( dv GT 0.0 )
        {
            if( V[l]EQ V[l + 1] )
                NL_ERROR( NL_INP_ERR );

            if( V[l] + dv GE V[l + 1] )
                NL_ERROR( NL_INP_ERR );
        }
        else
        {
            if( V[l]EQ V[l - 1] )
                NL_ERROR( NL_INP_ERR );

            if( V[l] + dv LE V[l - 1] )
                NL_ERROR( NL_INP_ERR );
        }
    }

    /* Copy surface and move knot */

    N_SrfInitArrays( &surA );
    error = N_SrfCopy( sur, &surA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetKnots( &surA, &r, &s, &U, &V );

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
        U[k] = U[k] + du;

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
        V[l] = V[l] + dv;

    /* Compute bound */

    error = N_MaxDiffTwoSrfs( sur, &surA, bnd );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfMaxChangeMovingKnot */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators  routine computes the  derivative of a NON-
     RATIONAL surface with respect to a knot. A typical calling example:

       NL_SURFACE  sur, der;
       NL_INDEX    k;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&der);
       N_DerivSrfNonRatSrfKnot(&sur,k,NL_LEFT,NL_UDIR,&der,&SG);

     Since NLIB  does not  allow  knot  multiplicities  greater than the
     degree, the surface cannot be differentiated with respect to a knot
     with  multiplicity  equal  to  the  degree. The  error  NL_KML_ERR  is 
     returned if such a knot is found.


   ACCESS:
   
     sur  , input  ,  NON-RATIONAL surface
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
     der  , output ,  Derivative surface
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_DerivSrfNonRatSrfKnot( NL_SURFACE *sur, NL_INDEX k, NL_FLAG flg, NL_FLAG dir, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_DerivSrfNonRatSrfKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s, mlt;

    NL_DEGREE p, q;

    NL_REAL *US, *VS, *UD, *VD, *fac;

    NL_CPOINT ** Pw, ** Dw, Z;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &US, &VS );

    if( N_IsSrfRat( sur ) )
        NL_ERROR( NL_INP_ERR );

    switch( dir )
    {
        case NL_UDIR:
            if( k LE p OR k GE n + 1 )
                NL_ERROR( NL_INP_ERR );

            switch( flg )
            {
                case NL_LEFT:
                    if( US[k]EQ US[k - 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( US[i]EQ US[i + 1] )
                        i++;
                    mlt = i - k + 1;

                    if( mlt GE p )
                        NL_ERROR( NL_KML_ERR );
                    break;

                case NL_RIGHT:
                    if( US[k]EQ US[k + 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( US[i]EQ US[i - 1] )
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
                    if( VS[k]EQ VS[k - 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( VS[i]EQ VS[i + 1] )
                        i++;
                    mlt = i - k + 1;

                    if( mlt GE q )
                        NL_ERROR( NL_KML_ERR );
                    break;

                case NL_RIGHT:
                    if( VS[k]EQ VS[k + 1] )
                        NL_ERROR( NL_INP_ERR );

                    i = k;

                    while( VS[i]EQ VS[i - 1] )
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
        error = N_SrfSizeArrays( der, n + 1, m, p, q, r + 1, s, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( dir EQ NL_VDIR )
    {
        error = N_SrfSizeArrays( der, n, m + 1, p, q, r, s + 1, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SrfGetCPtsAndKnots( der, &Dw, &UD, &VD );

    /* Compute the knot vector */

    switch( dir )
    {
        case NL_UDIR:
            switch( flg )
            {
                case NL_LEFT:

                    j = -1;

                    for ( i = 0; i <= k + mlt - 1; i++ )
                        UD[++j] = US[i];
                    UD[++j] = US[k];

                    for ( i = k + mlt; i <= r; i++ )
                        UD[++j] = US[i];
                    break;

                case NL_RIGHT:

                    j = -1;

                    for ( i = 0; i <= k; i++ )
                        UD[++j] = US[i];
                    UD[++j] = US[k];

                    for ( i = k + 1; i <= r; i++ )
                        UD[++j] = US[i];
                    break;
            }

            for ( j = 0; j <= s; j++ )
                VD[j] = VS[j];
            break;

        case NL_VDIR:
            switch( flg )
            {
                case NL_LEFT:

                    i = -1;

                    for ( j = 0; j <= k + mlt - 1; j++ )
                        VD[++i] = VS[j];
                    VD[++i] = VS[k];

                    for ( j = k + mlt; j <= s; j++ )
                        VD[++i] = VS[j];
                    break;

                case NL_RIGHT:

                    i = -1;

                    for ( j = 0; j <= k; j++ )
                        VD[++i] = VS[j];
                    VD[++i] = VS[k];

                    for ( j = k + 1; j <= s; j++ )
                        VD[++i] = VS[j];
                    break;
            }

            for ( i = 0; i <= r; i++ )
                UD[i] = US[i];
            break;
    }

    /* Compute control points */

    fac = N_AllocReal1dArray( NL_MAX( p, q ), &SL );

    if( fac EQ NULL )
        NL_QUIT;

    if( dir EQ NL_UDIR )
    {
        for ( i = k - p; i <= k; i++ )
            fac[i - k + p] = 1.0 / (US[i + p] - US[i]);
    }

    if( dir EQ NL_VDIR )
    {
        for ( j = k - q; j <= k; j++ )
            fac[j - k + q] = 1.0 / (VS[j + q] - VS[j]);
    }

    N_CPtFromWxWyWz( 0.0, 0.0, 0.0, NL_NOW, &Z );

    switch( dir )
    {
        case NL_UDIR:
            for ( j = 0; j <= m; j++ )
            {
                for ( i = 0; i <= k - p - 1; i++ )
                    N_CopyCPt( Z, &Dw[i][j] );

                for ( i = k - p; i <= k; i++ )
                {
                    N_Diff2CPts( Pw[i - 1][j], Pw[i][j], &Dw[i][j] );
                    N_ScaleCPt( fac[i - k + p], Dw[i][j], &Dw[i][j] );
                }

                for ( i = k + 1; i <= n + 1; i++ )
                    N_CopyCPt( Z, &Dw[i][j] );
            }
            break;

        case NL_VDIR:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= k - q - 1; j++ )
                    N_CopyCPt( Z, &Dw[i][j] );

                for ( j = k - q; j <= k; j++ )
                {
                    N_Diff2CPts( Pw[i][j - 1], Pw[i][j], &Dw[i][j] );
                    N_ScaleCPt( fac[j - k + q], Dw[i][j], &Dw[i][j] );
                }

                for ( j = k + 1; j <= m + 1; j++ )
                    N_CopyCPt( Z, &Dw[i][j] );
            }
            break;
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_DerivSrfNonRatSrfKnot */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine computes the  first derivative 
     surface of a rational surface with respect to a knot. The  result 
     is obtained by using  other symbolic  operators  to  compute  the 
     following expression:

         ( N )'   N'*D - N*D'
         ( - )  = -----------
         ( D )       D*D     

     where  N' and  D' are the  derivatives of  the  numerator and the 
     denominator  with  respect  to a kot in  u- or in  v-direction. A 
     typical calling example is:

       NL_SURFACE  sur, der;
       NL_INDEX    k;
       NL_STACKS   SG;
       ...
       (define sur);
       ...
       N_SrfInitArrays(&der);
       N_DerivSrfRatSrfKnot(&sur,k,NL_LEFT,NL_UDIR,&der,&SG);

     Since NLIB  does not allow knot  multiplicities  greater than the
     degree, the  surface cannot  be differentiated  with respect to a 
     knot with  multiplicity  equal to the u- or  v-degree. The  error  
     NL_KML_ERR is returned if such a knot is found.


   ACCESS:
   
     sur  , input  ,  NURBS surface
     k    , input  ,  Index of  knot, i.e. the derivative with respect 
                      to t_k is computed (t is u or v)
     flg  , input  ,  Flag:
                        NL_LEFT : left derivative. NL_INDEX  k  MUST SATISFY
                               t_(k) != t_(k-1)
                        NL_RIGHT: right derivative. NL_INDEX k  MUST SATISFY
                               t_(k) != t_(k+1)
     dir  , input  ,  Flag:
                        NL_UDIR: derivative in u-direction required
                        NL_VDIR: derivative in v-direction required
     der  , output ,  Derivative surface
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_DerivSrfRatSrfKnot( NL_SURFACE *sur, NL_INDEX k, NL_FLAG flg, NL_FLAG dir, NL_SURFACE *der, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX r, s;

    NL_REAL *U, *V;

    NL_SURFACE n, np, npd, ndp, num;

    NL_SFUN d, dp, den;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsSrfRat( sur ) )
    {
        error = N_DerivSrfNonRatSrfKnot( sur, k, flg, dir, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_SrfInitArrays( &n );
    N_SFuncInitArrays( &d );
    error = N_SrfGetNumAndDen( sur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output surface */

    N_SrfGetKnots( sur, &r, &s, &U, &V );
    N_SFuncInitArrays( &den );

    error = N_SrfFuncMultiplySrfFunc( &d, &d, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( dir EQ NL_UDIR )
    {
        error = N_SrfFuncInsertKnot( &den, U[k], 1, NL_UDIR, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( dir EQ NL_VDIR )
    {
        error = N_SrfFuncInsertKnot( &den, V[k], 1, NL_VDIR, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute the numerator */

    N_SrfInitArrays( &np );
    error = N_DerivSrfNonRatSrfKnot( &n, k, flg, dir, &np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &dp );
    error = N_SrfFuncFuncDerivFuncAtKnot( &d, k, flg, dir, &dp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &npd );
    error = N_SrfFuncMultiplySrf( &d, &np, &npd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &ndp );
    error = N_SrfFuncMultiplySrf( &dp, &n, &ndp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &num );
    error = N_SumDiffTwoSrfs( &npd, &ndp, NL_MINUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output surface */

    error = N_CreateSrfFromNumAndDen( &num, &den, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_DerivSrfRatSrfKnot */

#endif // NLIB_UNUSED
