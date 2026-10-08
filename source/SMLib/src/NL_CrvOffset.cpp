// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvOffset.c: Curve offset routines                               */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

// #include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */
#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
// #include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */
// #include <NL_VolumeAdv.h>   /* Advanced NL_VOLUME functions     */
// #include <NL_FuncsAdv.h>    /* Advanced NL_CFUN, NL_CVALUE, NL_SFUN, NL_SVALUE, NL_VFUN, and NL_VVALUE functions */
// #include <NL_FrameAdv.h>    /* Advanced NL_CPOLYGON, NL_EPOLYGON, NL_CNET, NL_ENET, NL_CMESH, and NL_EMESH functions */
// #include <NL_Tessellate.h>  /* Tessellation functions */
// #include <NL_Spiral.h>      /* Spiral functions */

#if NLIB_UNUSED



static NL_FLAG ST_CrvOffsetApprox( NL_PARAMETER, NL_POINT * );

/**********************************************************************/
/* N_CRVOFFSETFUNC: Functional offset of Nlib curve using point sampling     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation  routine computes an approximation of the offset 
     curve of a NL_G1 continuous  Nlib  curve  using functional offsetting.
     That is, given a curve  C(u), a  distance function d(u),  and a di-
     rection curve V(u), this routine  computes an approximation of  the
     curve

       C_o(u) = C(u) + d(u)*V(u)

     This function does not check for loops/cusps or self intersections.
     A typical calling  example is:

       NL_CURVE   curP, curV, curQ;
       NL_CFUN    cfnd;
       NL_DEGREE  deg;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (get curP, cfnd, curV, tol and deg);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvOffsetFuncVariableDir(&curP,&cfnd,&curV,deg,NL_INHERITED,tol,&curQ,&SG);

     curQ MUST BE INITIALIZED TO NULL FOR THIS NL_FUNCTION,  i.e. MEMORY IS
     ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     curP , input  ,  NL_G1 curve whose offset is to be computed
     cfnd , input  ,  Distance function
     curV , input  ,  Direction curve
     deg  , input  ,  Degree of the approximating curve
     par  , input  ,  Flag:
                       NL_UNIFORM    : uniform parametrization wanted
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    curP
     tol  , input  ,  Tolerance of approximation
     curQ , output ,  Offset curve
     SG   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_CrvOffsetFuncVariableDir( NL_CURVE *curP, NL_CFUN *cfnd, NL_CURVE *curV, NL_DEGREE deg, NL_FLAG par, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvOffsetFuncVariableDir");

    NL_FLAG twod = NL_NO, error = NL_NO;

    NL_INDEX *nu, i, j, k, l, n, mp, mq, mu, mt;

    NL_DEGREE p, q, pm;

    NL_REAL *UP, *UQ, *u, Muu, ul, ur, uinc, f1, f2, del, num, gro, exp, tlk, duu, d, t;

    NL_POINT *P, OD[3], Q;

    NL_VECTOR V;

    NL_KNOTVECTOR ** knt, ** knx, *knu;

    NL_INTERVAL I;

    NL_CURVE ** cbz, ** vbz, curB, curD;

    NL_CFUN ** fbz, cfnl;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    if( NOT N_CrvIs3d( curP ) )
        twod = NL_YES;

    /* Merge knots */

    N_CrvGetKnots( curP, &mp, &UP );
    f1 = fabs( UP[mp] - UP[0] );

    N_CFuncGetKnots( cfnd, &mp, &UP );
    f2 = fabs( UP[mp] - UP[0] );

    if( f2 LT f1 )
        f1 = f2;

    N_CrvGetKnots( curV, &mp, &UP );
    f2 = fabs( UP[mp] - UP[0] );

    if( f2 LT f1 )
        f1 = f2;

    tlk = f1 * NL_PTOL;

    knt = N_Alloc1dArrayKnotVectPtrs( 2, &SL );

    if( knt EQ NULL )
        NL_QUIT;

    N_CrvGetKnotVector( curP, &knt[0] );
    N_CFuncGetKnotVector( cfnd, &knt[1] );
    N_CrvGetKnotVector( curV, &knt[2] );

    error = N_GetCompatibleKnotVector( knt, 2, tlk, &knx, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Refine input entities */

    N_CrvInitArrays( &curB );
    error = N_CrvCopy( curP, &curB, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &cfnl );
    error = N_CrvFuncCopy( cfnd, &cfnl, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &curD );
    error = N_CrvCopy( curV, &curD, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx[0], &mp, &UP );

    if( mp GE 0 )
    {
        error = N_CrvRefine( &curB, knx[0], &curB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( knx[1], &mp, &UP );

    if( mp GE 0 )
    {
        error = N_CrvFuncRefine( &cfnl, knx[1], &cfnl, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( knx[2], &mp, &UP );

    if( mp GE 0 )
    {
        error = N_CrvRefine( &curD, knx[2], &curD, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_CrvGetKnots( &curB, &mp, &UP );
    N_CrvGetDegree( &curB, &p );

    pm = p;
    N_CFuncGetDegree( &cfnl, &q );

    if( q GT pm )
        pm = q;

    N_CrvGetDegree( &curD, &q );

    if( q GT pm )
        pm = q;

    /* Initialize */

    if( deg EQ 1 )
        exp = 0.5;
    else
        exp = 0.34;

    if( N_FloatOpIsBad( 1.0, tol, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    f1 = 1.0 / tol;
    f1 = pow( f1, exp );
    f2 = ( 2.0 * (NL_REAL)deg );

    /* Get Bezier segments */

    error = N_CrvDecomposeBez( &curB, &cbz, &i, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvFuncDecompose( &cfnl, &fbz, &j, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDecomposeBez( &curD, &vbz, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( i NEQ j OR j NEQ k )
        NL_ERROR( NL_KNT_ERR );

    /* Get number of sampling points */

    nu = N_AllocInt1dArray( k, &SL );

    if( nu EQ NULL )
        NL_QUIT;

    mu = 0;
    mt = 10 * pm;
    uinc = 1.0 / mt;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvReparamToInterval( cbz[i], NL_UNITSPAN );
        N_CrvFuncReparam( fbz[i], NL_UNITSPAN );
        N_CrvReparamToInterval( vbz[i], NL_UNITSPAN );

        Muu = 0.0;

        for ( j = 0; j <= mt; j++ )
        {
            if( j EQ 0 )
                t = 0.0;

            else if( j EQ mt )
                t = 1.0;

            else
                t = j * uinc;

            error = N_CrvOffsetGetDeriv( cbz[i], t, fbz[i], vbz[i], NL_LEFT, OD );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtMagnitude( OD[2], &duu );

            if( duu GT Muu )
                Muu = duu;
        }

        num = 0.125 *Muu;
        gro = NL_MAX( f2, sqrt( num ) );
        del = f1 * gro;
        del = ceil( del );
        nu[i] = (NL_INDEX)del;
        mu += nu[i];
    }

    /* Compute parameters */

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    ul = UP[p];
    i = p + 1;
    j = 0;
    u[0] = ul;
    l = 0;

    while( i LT mp )
    {
        while( i LT mp AND UP[i]EQ UP[i + 1] )
            i++;
        ur = UP[i];

        uinc = (ur - ul) / nu[j];

        for ( k = 1; k < nu[j]; k++ )
            u[++l] = ul + k * uinc;
        u[++l] = ur;

        ul = ur;
        i++;
        j++;
    }

    /* Sample curve */

    P = N_AllocPt1dArray( mu, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= mu; i++ )
    {
        error = N_CrvEval( &curB, u[i], NL_LEFT, &Q );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CFuncEval( &cfnl, u[i], NL_LEFT, &d );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvEval( &curD, u[i], NL_LEFT, &V );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorPtAlongVector( Q, d, V, &P[i] );
    }

    /* Get knot vector */

    knu = N_AllocKnotVectorAndArray( mu + deg + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)P, mu, NL_EPOINT, par, u );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_FitCrvCalcKnotVector( u, mu, deg, knu );

    /* Interpolate points */

    N_CrvInitArrays( curQ );
    error = N_FitCrvInterpGivenParams( (NL_VOID *)P, mu, NL_EPOINT, u, knu, deg, curQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( twod EQ NL_YES )
        N_Crv3dTo2d( curQ );

    /* Remove knots */

    error = N_CrvRemoveKnots( curQ, tol, curQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output curve */

    N_CrvGetArraySizes( curQ, &n, &i );

    if( n LT mu )
    {
        error = N_CrvCompress( curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Rescale knot vector if necessary */

    if( par NEQ NL_INHERITED )
    {
        N_CrvGetKnots( curQ, &mq, &UQ );

        if( UP[0]NEQ UQ[0]OR UP[mp]NEQ UQ[mq] )
        {
            N_CreateInterval( &I, UP[0], UP[mp] );
            N_CrvReparamToInterval( curQ, I );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvOffsetFuncConstantDir: Functional offset of curve at constant direction        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation  routine computes an approximation of the offset 
     curve of a NL_G1 continuous  Nlib  curve  using functional offsetting.
     That is, given a curve  C(u), a  distance function d(u),  and a 
     constant direction normal, this routine  computes an approximation 
     of  the curve

       C_o(u) = C(u) + d(u)*(T x norm)
       where T is unitised C'(u)

     This function does not check for loops/cusps or self intersections.
     A typical calling  example is:

       NL_CURVE   curP,curQ;
       NL_VECTOR  norm;
       NL_CFUN    cfnd;
       NL_DEGREE  deg;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (get curP, cfnd, norm, tol and deg);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvOffsetFuncConstantDir(&curP,&cfnd,norm,deg,NL_INHERITED,tol,&curQ,&SG);

     curQ MUST BE INITIALIZED TO NULL FOR THIS NL_FUNCTION,  i.e. MEMORY IS
     ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     curP , input  ,  NL_G1 curve whose offset is to be computed
     cfnd , input  ,  Distance function
     norm , input  ,  Normal direction for offset *offset = tangent x norm
     deg  , input  ,  Degree of the approximating curve
     par  , input  ,  Flag:
                       NL_UNIFORM    : uniform parametrization wanted
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    curP
     tol  , input  ,  Tolerance of approximation
     curQ , output ,  Offset curve
     SG   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   NOTE: This is a simplified version of N_CrvOffsetFuncVariableDir where the direction 
   of the offset direction is given by a constant normal x curve tangent
   and the magnitude by a distance function cfnd(u),


   ***********************************************************************/

NL_FLAG N_CrvOffsetFuncConstantDir( NL_CURVE *curP, NL_CFUN *cfnd, NL_VECTOR norm, NL_DEGREE deg, NL_FLAG par, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvOffsetFuncConstantDir");

    NL_FLAG twod = NL_NO, error = NL_NO;

    NL_INDEX *nu, i, j, k, l, n, mp, mq, mu, mt;

    NL_DEGREE p, q, pm;

    NL_REAL *UP, *UQ, *u, Muu, ul, ur, uinc, f1, f2, del, num, gro, exp, tlk, duu, d, t, mag;

    NL_POINT *P, OD[3], CD[2];

    NL_VECTOR T, N;

    NL_KNOTVECTOR ** knt, ** knx, *knu;

    NL_INTERVAL I;

    NL_CURVE ** cbz, curB, curD;

    NL_CFUN ** fbz, cfnl;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    if( NOT N_CrvIs3d( curP ) )
        twod = NL_YES;

    /* Merge knots */

    N_CrvGetKnots( curP, &mp, &UP );
    f1 = fabs( UP[mp] - UP[0] );

    N_CFuncGetKnots( cfnd, &mp, &UP );
    f2 = fabs( UP[mp] - UP[0] );

    if( f2 LT f1 )
        f1 = f2;

    tlk = f1 * NL_PTOL;

    knt = N_Alloc1dArrayKnotVectPtrs( 2, &SL );

    if( knt EQ NULL )
        NL_QUIT;

    N_CrvGetKnotVector( curP, &knt[0] );
    N_CFuncGetKnotVector( cfnd, &knt[1] );

    error = N_GetCompatibleKnotVector( knt, 1, tlk, &knx, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Refine input entities */

    N_CrvInitArrays( &curB );
    error = N_CrvCopy( curP, &curB, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &cfnl );
    error = N_CrvFuncCopy( cfnd, &cfnl, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx[0], &mp, &UP );

    if( mp GE 0 )
    {
        error = N_CrvRefine( &curB, knx[0], &curB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( knx[1], &mp, &UP );

    if( mp GE 0 )
    {
        error = N_CrvFuncRefine( &cfnl, knx[1], &cfnl, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_CrvGetKnots( &curB, &mp, &UP );
    N_CrvGetDegree( &curB, &p );

    pm = p;
    N_CFuncGetDegree( &cfnl, &q );

    if( q GT pm )
        pm = q;

    N_CrvGetDegree( &curD, &q );

    if( q GT pm )
        pm = q;

    /* Initialize */

    if( deg EQ 1 )
        exp = 0.5;
    else
        exp = 0.34;

    if( N_FloatOpIsBad( 1.0, tol, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    f1 = 1.0 / tol;
    f1 = pow( f1, exp );
    f2 = ((NL_REAL)2 * (NL_REAL)deg );

    /* Get Bezier segments */

    error = N_CrvDecomposeBez( &curB, &cbz, &i, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvFuncDecompose( &cfnl, &fbz, &j, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( i NEQ j )
        NL_ERROR( NL_KNT_ERR );

    /* Get number of sampling points */

    nu = N_AllocInt1dArray( i, &SL );

    if( nu EQ NULL )
        NL_QUIT;

    mu = 0;
    mt = 10 * pm;
    uinc = 1.0 / mt;
    k = j;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvReparamToInterval( cbz[i], NL_UNITSPAN );
        N_CrvFuncReparam( fbz[i], NL_UNITSPAN );

        Muu = 0.0;

        for ( j = 0; j <= mt; j++ )
        {
            if( j EQ 0 )
                t = 0.0;

            else if( j EQ mt )
                t = 1.0;

            else
                t = j * uinc;

            error = N_CrvDerivs( cbz[i], t, NL_LEFT, 2, OD );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtMagnitude( OD[2], &duu );

            if( duu GT Muu )
                Muu = duu;
        }

        num = 0.125 *Muu;
        gro = NL_MAX( f2, sqrt( num ) );
        del = f1 * gro;
        del = ceil( del );
        nu[i] = (NL_INDEX)del;
        mu += nu[i];
    }

    /* Compute parameters */

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    ul = UP[p];
    i = p + 1;
    j = 0;
    u[0] = ul;
    l = 0;

    while( i LT mp )
    {
        while( i LT mp AND UP[i]EQ UP[i + 1] )
            i++;
        ur = UP[i];

        uinc = (ur - ul) / nu[j];

        for ( k = 1; k < nu[j]; k++ )
            u[++l] = ul + k * uinc;
        u[++l] = ur;

        ul = ur;
        i++;
        j++;
    }

    /* Sample curve */

    P = N_AllocPt1dArray( mu, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= mu; i++ )
    {
        /* Get tangent */
        error = N_CrvDerivs( &curB, u[i], NL_LEFT, 1, CD );

        if( error EQ NL_YES )
            NL_OUT;

        /* Normalise */
        error = N_VectorNormalize( CD[1], &T, &mag );

        if( error EQ NL_YES )
            NL_OUT;

        /* Cross product */
        N_VectorCrossRef( &norm, &T, &N );

        N_VectorNormalize( N, &T, &mag );

        error = N_CFuncEval( &cfnl, u[i], NL_LEFT, &d );

        if( error EQ NL_YES )
            NL_OUT;

        /* Create point on crv */
        N_VectorPtAlongVector( CD[0], d, T, &P[i] );
    }

    /* Get knot vector */

    knu = N_AllocKnotVectorAndArray( mu + deg + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)P, mu, NL_EPOINT, par, u );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_FitCrvCalcKnotVector( u, mu, deg, knu );

    /* Interpolate points */

    N_CrvInitArrays( curQ );
    error = N_FitCrvInterpGivenParams( (NL_VOID *)P, mu, NL_EPOINT, u, knu, deg, curQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( twod EQ NL_YES )
        N_Crv3dTo2d( curQ );

    /* Remove knots */

    error = N_CrvRemoveKnots( curQ, tol, curQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output curve */

    N_CrvGetArraySizes( curQ, &n, &i );

    if( n LT mu )
    {
        error = N_CrvCompress( curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Rescale knot vector if necessary */

    if( par NEQ NL_INHERITED )
    {
        N_CrvGetKnots( curQ, &mq, &UQ );

        if( UP[0]NEQ UQ[0]OR UP[mp]NEQ UQ[mq] )
        {
            N_CreateInterval( &I, UP[0], UP[mp] );
            N_CrvReparamToInterval( curQ, I );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CrvOffsetApprox: Approximate offset of Nlib curve with nonrational curve  */
/**********************************************************************/

NL_PRIVATE_TLS NL_CURVE *gloC;
NL_PRIVATE_TLS NL_VECTOR gloN;
NL_PRIVATE_TLS NL_REAL glod;
/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a NURBS curve, Q(v) , approxi-
     mating the offset of a planar,  NL_G1 continuous Nlib curve, C(u).  A
     global method is used, consisting of interpolation, least  squares
     approximation, and knot removal. This function does not check  for
     loops, cusps or self-intersections in Q(v). Q(v)  is  at least  NL_C1
     continuous and nonrational.  Various  options  exist for the para-
     meterization of Q(v). A typical calling example is as follows:

       NL_DEGREE     p;
       NL_REAL       Eg, Ep, ptol, dist;
       NL_VECTOR     Nm;
       NL_CURVE      curQ;
       NL_STACKS     SQ;
       ...
       (define function:  NL_FLAG  evnF (NL_PARAMETER u, NL_PARAMETER *v);
       (compute Nm);
       (choose p, dist, Eg, Ep, and ptol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvOffsetApprox(curP,Nm,dist,p,NL_NO,NL_FUNCTION,evnF,Eg,Ep,ptol,&curQ,&SQ);

       curQ must be initialized to NULL for this function.


   ACCESS:
   
     curP , input  ,  Curve whose offset is to be approximated. Assumed
                      to be NL_G1
     Nm   , input  ,  This vector must be normal to the  plane  of def-
                      inition of curP. The offset curve is defined  by: 
                      Q(v) = C(u) + dist*(Nm x T(u)) ,  where  T(u)  is
                      parallel to  C(u)'s  tangent at u, and  Nm x T(u)
                      is normalized to unit length.
     dist , input  ,  The offset distance (may be negative or positive)
     p    , input  ,  Degree of  the  approximating curve, Q(v).  p > 1
                      must hold. p = 3 is recommended
     tans , input  ,  Flag:
                       NL_YES: maintain the precise end tangent directions 
                       NL_NO : do not require precise end tangent  direct-
                            ions 
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(u)
                       NL_FUNCTION   : parameterization defined by evnF
     evnF , input  ,  Reparameterization function, v = f(u). Used  only
                      if par = NL_FUNCTION
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from Q(v) to
                      the true offset of C(u) to not be greater than Eg
     Ep   , input  ,  Parametric error tolerance.  For  par = NL_INHERITED
                      and  par = NL_FUNCTION ,  this  function  attemps to 
                      maintain  |true offset of C(u) - Q(v)| <= Ep  for
                      corresponding  u and v.  Ep  is also used for the
                      par = NL_CHORDLENGTH  and  par = NL_CENTRIPETAL  cases, 
                      but it's meaning is less precise. 
     ptol , input  ,  Point coincidence tolerance.  Two points are con-
                      sidered to be equal if the  distance between them
                      is less than or equal to ptol.  ptol < Eg  should
                      hold
     curQ , output ,  Approximating curve (initialized to NULL)
     SQ   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_CrvOffsetApprox( NL_CURVE *curP, NL_VECTOR Nm, NL_REAL dist, NL_DEGREE p, NL_FLAG tans, NL_FLAG par, NL_FLAG(*evnF)( NL_PARAMETER, NL_PARAMETER * ), NL_REAL Eg, NL_REAL Ep, NL_REAL ptol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvOffsetApprox");

    NL_FLAG error = NL_NO;

    NL_INDEX m;

    NL_PARAMETER *up, *uq;

    NL_POINT *P, D[4];

    NL_VECTOR *Ts, *Te;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if( p LT 2 )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvAreArraysNULL( curQ ) )
        NL_ERROR( NL_INP_ERR );

    /* Assign globals */

    gloC = curP;
    N_CopyPt( Nm, &gloN );
    glod = dist;

    /* Evaluate a set of sample points from input curve */

    error = N_GetPtsForCrvApprox( curP, ST_CrvOffsetApprox, par, 0.9 *Eg, ptol, &P, &m, &up, &uq, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Supply end tangents if required */

    Ts = NULL;
    Te = NULL;

    if( tans )
    {
        error = N_CrvDerivs( curP, up[0], NL_LEFT, 1, &D[0] );

        if( error EQ NL_YES )
            NL_OUT;
        Ts = &D[1];
        error = N_CrvDerivs( curP, up[m], NL_RIGHT, 1, &D[2] );

        if( error EQ NL_YES )
            NL_OUT;
        Te = &D[3];
    }

    /* Now fit the point set with a NL_C1 Nurbs curve */

    error = N_ApproxProcCrvWithCrvFit( P, m, ST_CrvOffsetApprox, Ts, Te, up, par, uq, evnF, p, Eg, Ep, ptol, curQ, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_CRVOFFSET: Offset of Nlib curve                                     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes a NURBS curve, Q(v) , approxi-
     mating the offset of a planar,  NL_G1 continuous Nlib curve, C(u), or
     precisely  offsetting it  if it is a  LINE or a  CIRCLE.  A global 
     method  is  used,  consisting  of  interpolation,  least   squares 
     approximation, and knot removal. This function does not check  for
     loops, cusps or self-intersections in Q(v). Q(v)  is  at least  NL_C1
     continuous and nonrational.  Various  options  exist for the para-
     meterization of Q(v). A typical calling example is as follows:

       NL_DEGREE     p;
       NL_REAL       Eg, Ep, ptol, dist;
       NL_VECTOR     Nm;
       NL_CURVE      curQ;
       NL_STACKS     SQ;
       ...
       (define function:  NL_FLAG  evnF (NL_PARAMETER u, NL_PARAMETER *v);
       (compute Nm);
       (choose p, dist, Eg, Ep, and ptol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvOffset(curP,Nm,dist,p,NL_NO,NL_FUNCTION,evnF,Eg,Ep,ptol,&curQ,&SQ);

       curQ must be initialized to NULL for this function. 


   ACCESS:
   
     curP , input  ,  Curve whose offset is to be  computed. Assumed to
                      be NL_G1
     Nm   , input  ,  This vector must be normal to the  plane  of def-
                      inition of curP. The offset curve is defined  by: 
                      Q(v) = C(u) + dist*(Nm x T(u)) ,  where  T(u)  is
                      parallel to  C(u)'s  tangent at u, and  Nm x T(u)
                      is normalized to unit length.
     dist , input  ,  The offset distance (may be negative or positive)
     p    , input  ,  Degree of  the  approximating curve, Q(v).  p > 1
                      must  hold. p = 3 is  recommended. If  curP  is a 
                      circle and its  degree is  2, 4 or 5, the  offset
                      circle has the same degree. Otherwise, a degree 2
                      circle is computed.
     tans , input  ,  Flag:
                       NL_YES: maintain the precise end tangent directions 
                       NL_NO : do not require precise end tangent  direct-
                            ions 
     par  , input  ,  Flag:
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    C(u)
                       NL_FUNCTION   : parameterization defined by evnF
     evnF , input  ,  Reparameterization function, v = f(u). Used  only
                      if par = NL_FUNCTION
     Eg   , input  ,  Geometric error tolerance. This function attempts
                      to bound the perpendicular distance  from Q(v) to
                      the true offset of C(u) to not be greater than Eg
     Ep   , input  ,  Parametric error tolerance.  For  par = NL_INHERITED
                      and  par = NL_FUNCTION ,  this  function  attemps to 
                      maintain  |true offset of C(u) - Q(v)| <= Ep  for
                      corresponding  u and v.  Ep  is also used for the
                      par = NL_CHORDLENGTH  and  par = NL_CENTRIPETAL  cases, 
                      but it's meaning is less precise. 
     ptol , input  ,  Point coincidence tolerance.  Two points are con-
                      sidered to be equal if the  distance between them
                      is less than or equal to ptol.  ptol < Eg  should
                      hold
     curQ , output ,  Offset curve (initialized to NULL)
     SQ   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_CrvOffset( NL_CURVE *curP, NL_VECTOR Nm, NL_REAL dist, NL_DEGREE p, NL_FLAG tans, NL_FLAG par, NL_FLAG(*evnF)( NL_PARAMETER, NL_PARAMETER * ), NL_REAL Eg, NL_REAL Ep, NL_REAL ptol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_FLAG typ, ctp, error = NL_NO;

    NL_DEGREE deg;

    NL_REAL r, al;

    NL_POINT P;

    NL_VECTOR V, X, Y, D;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Get curve type */

    error = N_CrvGetType     /* rtn: 0 = OK, 1 = error                                    */
                  ( curP,    /* in : Target Curve                                         */
                    ptol,    /* in : max dist to Line or max variation in radius          */
                    &P,      /* out: line start point or circle center                    */
                    &V,      /* out: line dir vector  or circle plane normal              */
                    &X,      /* out: line undefined   or circle x axis                    */
                    &Y,      /* out: line undefined   or circle y axis                    */
                    &r,      /* out: line undefined   or circle radius                    */
                    &al,     /* out: line undefined   or circle angle or arc in degrees   */
                    &typ );  /* out: NL_NLINE        - curve is within tol of line        */
                             /*      NL_NCIRCLE      - curve is within tol of circle      */
                             /*      NL_NFREEFORM    - curve is neither a line or circle  */

    if( error EQ NL_YES )
        NL_OUT;

    /* Switch to different curve types */

    switch( typ )
    {
        case NL_NLINE:

            N_VectorCross( Nm, V, &D );

            error = N_VectorNormalizeRef( &D );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorBlendPt( dist, D, &P );

            error = N_CrvLineFromPtAndVector( P, V, curQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_NCIRCLE:

            N_CrvGetDegree( curP, &deg );

            ctp = NL_QUADRATIC;

            if( deg EQ 4 )
                ctp = NL_QUARTIC;

            else if( deg EQ 5 )
                ctp = NL_QUINTIC;

            error = N_CreateCircArc( P, X, Y, r + dist, 0.0, al, ctp, curQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_NFREEFORM:

            error = N_CrvOffsetApprox( curP, Nm, dist, p, tans, par, evnF, Eg, Ep, ptol, curQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_CrvOffsetPtSampling: Offset of Nlib curve using point sampling                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes an approximation of the offset 
     curve of a planar, NL_G1 continuous Nlib curve. If the curve  happens
     to  be a  LINE or a  CIRCLE, its  offset is  computed precisely. A 
     global sampling-based method is used, consisting of point sampling
     interpolation, and knot removal. This function does not check  for
     loops, cusps or  self-intersections. A typical calling  example is 
     as follows:

       NL_CURVE   curP, curQ;
       NL_DEGREE  deg;
       NL_REAL    d, tol;
       NL_VECTOR  B;
       NL_STACKS  SG;
       ...
       (get curP, B, d and tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvOffsetPtSampling(curP,B,d,deg,NL_NO,NL_INHERITED,tol,&curQ,&SG);

     curQ MUST BE INITIALIZED TO NULL FOR THIS NL_FUNCTION, i.e. MEMORY IS
     ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     curP , input  ,  NL_G1 curve whose offset is to be computed
     B    , input  ,  Vector normal  (binormal of curP) to the plane of 
                      curP. The curve is offset along B x T, where T is
                      the tangent computed at various parameter values
     dist , input  ,  Offset distance (may be negative or positive)
     deg  , input  ,  Degree of  the  approximating curve.  If curP  is 
                      a precise circle, deg = circle degree
     tans , input  ,  Flag:
                       NL_YES: maintain the precise end tangent directions 
                       NL_NO : do not require precise end tangent  direct-
                            ions 
     par  , input  ,  Flag:
                       NL_UNIFORM    : uniform parametrization wanted
                       NL_CHORDLENGTH: chordlength parameterization wanted
                       NL_CENTRIPETAL: centripetal parameterization wanted
                       NL_INHERITED  : parameterization   inherited   from
                                    curP
     tol  , input  ,  Tolerance of approximation
     curQ , output ,  Offset curve
     SG   , input  ,  curQ's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_CrvOffsetPtSampling( NL_CURVE *curP, NL_VECTOR B, NL_REAL d, NL_DEGREE deg, NL_FLAG tan, NL_FLAG par, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvOffsetPtSampling");

    NL_FLAG twod = NL_NO, error = NL_NO, typ, ctp;

    NL_INDEX *nu, i, j, k, l, n, mp, mq, mu, der;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *u, Muu, ul, ur, uinc, f1, f2, del, num, gro, exp, r, al;

    NL_POINT *P, CD[2], Q;

    NL_VECTOR *Vs, *Ve, Ts, Te, N, X, Y, V, D;

    NL_KNOTVECTOR *knu;

    NL_INTERVAL I;

    NL_CURVE ** bez;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Get curve type */

    error = N_CrvGetType        /* rtn: 0 = OK, 1 = error                                    */
                  ( curP,       /* in : Target Curve                                         */
                    NL_MTOL,    /* in : max dist to Line or max variation in radius          */
                    &Q,         /* out: line start point or circle center                    */
                    &V,         /* out: line dir vector  or circle plane normal              */
                    &X,         /* out: line undefined   or circle x axis                    */
                    &Y,         /* out: line undefined   or circle y axis                    */
                    &r,         /* out: line undefined   or circle radius                    */
                    &al,        /* out: line undefined   or circle angle or arc in degrees   */
                    &typ );     /* out: NL_NLINE        - curve is within tol of line        */
                                /*      NL_NCIRCLE      - curve is within tol of circle      */
                                /*      NL_NFREEFORM    - curve is neither a line or circle  */

    if( error EQ NL_YES )
        NL_OUT;

    if( NOT N_CrvIs3d( curP ) )
        twod = NL_YES;

    /* Offset line */

    if( typ EQ NL_NLINE )
    {
        N_VectorCross( B, V, &D );

        error = N_VectorNormalizeRef( &D );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorBlendPt( d, D, &Q );

        error = N_CrvLineFromPtAndVector( Q, V, curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;

        if( twod EQ NL_YES )
            N_Crv3dTo2d( curQ );

        NL_OUT;
    }

    /* Offset circle */

    if( typ EQ NL_NCIRCLE )
    {
        ctp = NL_QUADRATIC;

        if( deg EQ 4 )
            ctp = NL_QUARTIC;

        else if( deg EQ 5 )
            ctp = NL_QUINTIC;

        error = N_CreateCircArc( Q,      /* in : circle center                                                 */
                                 X,      /* in : circle X axis                                                 */
                                 Y,      /* in : circle Y axis (circle normal plane Normal = cross(X,Y)        */
                                 r + d,  /* in : circle radius                                                 */
                                 0.0,    /* in : StartAngle degrees                                            */
                                 al,     /* in : EndAngle degrees                                              */
                                 ctp,    /* in : NL_QUADRATIC = deg 2 circle with double internal knots        */
                                         /*      NL_QUARTIC   = deg 4 circle with quadruple internal knots     */
                                         /*                     and better parameterization than NL_QUADRATIC  */
                                         /*      NL_QUINTIC   = deg 5 circle no internal knots                 */
                                 curQ,   /* out: The circle curve                                              */
                                 SG );   /* in : new object stack                                              */

        if( error EQ NL_YES )
            NL_OUT;

        if( twod EQ NL_YES )
            N_Crv3dTo2d( curQ );

        NL_OUT;
    }

    /* Offset free-form curve */

    N_CrvGetKnots( curP, &mp, &UP );
    N_CrvGetDegree( curP, &p );

    if( NOT N_CrvIs3d( curP ) )
        twod = NL_YES;

    /* Initialize */

    if( deg EQ 1 )
        exp = 0.5;
    else
        exp = 0.34;

    if( N_FloatOpIsBad( 1.0, tol, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    f1 = 1.0 / tol;
    f1 = pow( f1, exp );
    f2 = ( 2.0 * (NL_REAL)deg );

    error = N_VectorNormalizeRef( &B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier segments */

    error = N_CrvDecomposeBez( curP, &bez, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get number of sampling points */

    nu = N_AllocInt1dArray( k, &SL );

    if( nu EQ NULL )
        NL_QUIT;

    mu = 0;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvReparamToInterval( bez[i], NL_UNITSPAN );

        error = N_CrvOffsetGetMax2ndDeriv( bez[i], d, B, &Muu );

        if( error EQ NL_YES )
            NL_OUT;

        num = 0.125 *Muu;
        gro = NL_MAX( f2, sqrt( num ) );
        del = f1 * gro;
        del = ceil( del );
        nu[i] = (NL_INDEX)del;
        mu += nu[i];
    }

    /* Compute parameters */

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    ul = UP[p];
    i = p + 1;
    j = 0;
    u[0] = ul;
    l = 0;

    while( i LT mp )
    {
        while( i LT mp AND UP[i]EQ UP[i + 1] )
            i++;
        ur = UP[i];

        uinc = (ur - ul) / nu[j];

        for ( k = 1; k < nu[j]; k++ )
            u[++l] = ul + k * uinc;
        u[++l] = ur;

        ul = ur;
        i++;
        j++;
    }

    /* Sample curve */

    P = N_AllocPt1dArray( mu, &SL );

    if( P EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= mu; i++ )
    {
        error = N_CrvEvalTangent( curP, u[i], NL_LEFT, &Q, &Ts );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCross( B, Ts, &N );
        N_VectorPtAlongVector( Q, d, N, &P[i] );
    }

    /* Get knot vector */

    if( tan EQ NL_YES )
        j = mu + deg + 3;
    else
        j = mu + deg + 1;

    knu = N_AllocKnotVectorAndArray( j, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)P, mu, NL_EPOINT, par, u );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( tan EQ NL_YES )
        N_FitCalcKnotVectorEndDerivs( u, mu, deg, knu );
    else
        N_FitCrvCalcKnotVector( u, mu, deg, knu );

    /* Interpolate points */

    N_CrvInitArrays( curQ );

    if( tan EQ NL_YES )
    {
        /* Get end tangents */

        error = N_CrvDerivs( curP, UP[0], NL_LEFT, 1, CD );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( CD[1], &Ts );

        error = N_CrvDerivs( curP, UP[mp], NL_LEFT, 1, CD );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCopy( CD[1], &Te );

        if( par NEQ NL_INHERITED )
        {
            /* Adjust magnitudes */

            N_DistPolygon( P, mu, &del );

            error = N_VectorNormalizeRef( &Ts );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalizeRef( &Te );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorScale( Ts, del, &Ts );
            N_VectorScale( Te, del, &Te );
        }

        Vs = &Ts;
        Ve = &Te;

        error = N_FitCrvKnotsAndDerivs( (NL_VOID *)P, mu, NL_EPOINT, u, knu, deg, (NL_VOID *)Vs, (NL_VOID *)Ve, curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_FitCrvInterpGivenParams( (NL_VOID *)P, mu, NL_EPOINT, u, knu, deg, curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( twod EQ NL_YES )
        N_Crv3dTo2d( curQ );

    /* Remove knots */

    if( tan EQ NL_YES AND deg EQ 2 )
        der = 1;
    else
        der = 0;

    error = N_CrvRemoveKnotsDerivConstraints( curQ, tol, NL_BOTH, der, curQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output curve */

    N_CrvGetArraySizes( curQ, &n, &i );

    if( n LT mu )
    {
        error = N_CrvCompress( curQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Rescale knot vector if necessary */

    if( par NEQ NL_INHERITED )
    {
        N_CrvGetKnots( curQ, &mq, &UQ );

        if( UP[0]NEQ UQ[0]OR UP[mp]NEQ UQ[mq] )
        {
            N_CreateInterval( &I, UP[0], UP[mp] );
            N_CrvReparamToInterval( curQ, I );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

/*****************   Function  ST_CrvOffsetApprox()  ******************/

NL_FLAG ST_CrvOffsetApprox( NL_PARAMETER u, NL_POINT *P )
{
    NL_FLAG error = NL_NO;

    NL_POINT De[2], R;

    error = N_CrvDerivs( gloC, u, NL_LEFT, 1, De );

    if( error EQ NL_NO )
    {
        N_VectorCross( gloN, De[1], &R );
        error = N_VectorNormalizeRef( &R );

        if( error EQ NL_NO )
            N_Combine2Pts( 1.0, De[0], glod, R, P );
    }

    return (error);
}
#endif // NLIB_UNUSED
