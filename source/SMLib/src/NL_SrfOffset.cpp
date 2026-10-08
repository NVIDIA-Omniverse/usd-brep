// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfOffset.c: Surface offset routines                               */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */


#if NLIB_UNUSED

/**********************************************************************/
/* N_SRFOFFSETFUNC: Functional offset of Nlib surface using point sampling   */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes an approximation of the offset
     surface of a  given NL_G1 continuous  Nlib  surface using  functional 
     offseting. That  is, given a  surface  S(u,v), a distance function
     d(u,v), and a direction surface  V(u,v), this routine computes an
     approximation of the surface

       S_o(u,v) = S(u,v) + d(u,v)*V(u,v)/|V(u,v)|      


     Note: V(u,v) should produce a unit vector for all u,v. Just in case
     it does not, we normalise the result.

     This routine does not  check for offset anomalies  such as ridges,
     self intersections, etc. A typical calling example is:

       NL_SURFACE  surP, surV, surQ;
       NL_SFUN     sfnd;
       NL_DEGREE   pp, qq;
       NL_REAL     tol;
       NL_STACKS   SG;
       ...
       (define surP, sfnd and surV; get tol, pp and qq);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfOffsetFunc(&surP,&sfnd,&surV,pp,qq,NL_INHERITED,tol,&surQ,&SG);

     MEMORY TO STORE THE  OUTPUT NL_SURFACE  surQ  IS ALLOCATED INSIDE THE
     ROUTINE! IT IS  ASSUMED THAT surP IS AT LEAST NL_G1 CONTINUOUS AND IS
     A VALID Nlib NL_SURFACE!


   ACCESS:
   
     surP  , input  ,  NURBS surface
     sfnd  , input  ,  Distance function
     surV  , input  ,  Direction surface
     pp,qq , input  ,  Degrees of approximating surface.
     par   , input  ,  Flag:
                         NL_UNIFORM    : uniform parametrization
                         NL_CHORDLENGTH: chordlength parametrization
                         NL_CENTRIPETAL: centipetal parametrization
                         NL_INHERITED  : surP's parametrization 
     tol   , input  ,  Tolerance of approximation
     surQ  , output ,  Approximating surface
     SG    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfOffsetFunc( NL_SURFACE *surP, NL_SFUN *sfnd, NL_SURFACE *surV, NL_DEGREE pp, NL_DEGREE qq, NL_FLAG par, NL_REAL tol, NL_SURFACE *surQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfOffsetFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX ** np, *nu, *nv, i, j, k, l, n, m, rp, sp, rq, sq, mu, mv, a, b, ma, mb;

    NL_DEGREE p, q, pq, pm, qm;

    NL_REAL ** d, *UP, *VP, *UQ, *VQ, *u, *v, Muu, Muv, Mvv, ul, ur, vl, vr, uinc, vinc, f1, f2, del, num, gro, exp, utk, vtk, duu, duv, dvv, mag;

    NL_POINT ** P, ** OD;

    NL_VECTOR ** V, VN;

    NL_KNOTVECTOR ** ktu, ** ktv, ** kxu, ** kxv, *knu, *knv;

    NL_RECTANGLE R;

    NL_SURFACE *** sbz, *** vbz, surB, surD;

    NL_SFUN *** fbz, sfnl;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Merge knots */

    N_SrfGetKnots( surP, &rp, &sp, &UP, &VP );
    f1 = UP[rp] - UP[0];
    f2 = VP[sp] - VP[0];

    N_SrfFuncGetKnots( sfnd, &rp, &sp, &UP, &VP );

    if( UP[rp] - UP[0]LT f1 )
        f1 = UP[rp] - UP[0];

    if( VP[sp] - VP[0]LT f2 )
        f2 = VP[sp] - VP[0];

    N_SrfGetKnots( surV, &rp, &sp, &UP, &VP );

    if( UP[rp] - UP[0]LT f1 )
        f1 = UP[rp] - UP[0];

    if( VP[sp] - VP[0]LT f2 )
        f2 = VP[sp] - VP[0];

    utk = f1 * NL_PTOL;
    vtk = f2 * NL_PTOL;

    ktu = N_Alloc1dArrayKnotVectPtrs( 2, &SL );

    if( ktu EQ NULL )
        NL_QUIT;

    ktv = N_Alloc1dArrayKnotVectPtrs( 2, &SL );

    if( ktv EQ NULL )
        NL_QUIT;

    N_SrfGetKnotVectors( surP, &ktu[0], &ktv[0] );
    N_SFuncGetKnotVectors( sfnd, &ktu[1], &ktv[1] );
    N_SrfGetKnotVectors( surV, &ktu[2], &ktv[2] );

    error = N_GetCompatibleKnotVector( ktu, 2, utk, &kxu, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_GetCompatibleKnotVector( ktv, 2, vtk, &kxv, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Refine input entities */

    N_SrfInitArrays( &surB );
    error = N_SrfCopy( surP, &surB, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncInitArrays( &sfnl );
    error = N_SrfFuncCopy( sfnd, &sfnl, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &surD );
    error = N_SrfCopy( surV, &surD, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( kxu[0], &rp, &UP );

    if( rp GE 0 )
    {
        error = N_SrfInsertKnots( &surB, kxu[0], NL_UDIR, &surB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( kxv[0], &sp, &VP );

    if( sp GE 0 )
    {
        error = N_SrfInsertKnots( &surB, kxv[0], NL_VDIR, &surB, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( kxu[1], &rp, &UP );

    if( rp GE 0 )
    {
        error = N_SrfFuncRefine( &sfnl, kxu[1], NL_UDIR, &sfnl, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( kxv[1], &sp, &VP );

    if( sp GE 0 )
    {
        error = N_SrfFuncRefine( &sfnl, kxv[1], NL_VDIR, &sfnl, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( kxu[2], &rp, &UP );

    if( rp GE 0 )
    {
        error = N_SrfInsertKnots( &surD, kxu[2], NL_UDIR, &surD, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_KnotVectorGetKnots( kxv[2], &sp, &VP );

    if( sp GE 0 )
    {
        error = N_SrfInsertKnots( &surD, kxv[2], NL_VDIR, &surD, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_SrfGetKnots( &surB, &rp, &sp, &UP, &VP );

    pm = qm = 0;

    N_SFuncGetDegrees( &sfnl, &p, &q );

    if( p GT pm )
        pm = p;

    if( q GT qm )
        qm = q;

    N_SrfGetDegrees( &surD, &p, &q );

    if( p GT pm )
        pm = p;

    if( q GT qm )
        qm = q;

    N_SrfGetDegrees( &surB, &p, &q );

    if( p GT pm )
        pm = p;

    if( q GT qm )
        qm = q;

    /* Initialize */

    if( pp EQ 1 OR qq EQ 1 )
        exp = 0.5;
    else
        exp = 0.3;

    if( N_FloatOpIsBad( 1.0, tol, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    f1 = 1.0 / tol;
    f1 = pow( f1, exp );
    pq = NL_MAX( pp, qq );
    f2 = 2.0 * (NL_REAL)pq;

    /* Get Bezier patches */

    error = N_SrfDecomposeToBez( &surB, &sbz, &i, &j, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfFuncDecompose( &sfnl, &fbz, &n, &m, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfDecomposeToBez( &surD, &vbz, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( i NEQ n OR n NEQ k )
        NL_ERROR( NL_KNT_ERR );

    if( j NEQ m OR m NEQ l )
        NL_ERROR( NL_KNT_ERR );

    /* Allocate memory */

    np = N_AllocInt2dArray( k, l, &SL );

    if( np EQ NULL )
        NL_QUIT;

    nu = N_AllocInt1dArray( k, &SL );

    if( nu EQ NULL )
        NL_QUIT;

    nv = N_AllocInt1dArray( l, &SL );

    if( nv EQ NULL )
        NL_QUIT;

    OD = N_AllocPt2dArray( 2, 2, &SL );

    if( OD EQ NULL )
        NL_QUIT;

    /* Get number of sampling points */

    ma = 2 * pm;
    mb = 2 * qm;
    uinc = 1.0 / ma;
    vinc = 1.0 / mb;

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            N_SrfReparamToInterval( sbz[i][j], NL_UNITSQUARE, NL_UVDIR );
            N_SrfFuncScale( fbz[i][j], NL_UNITSQUARE, NL_UVDIR );
            N_SrfReparamToInterval( vbz[i][j], NL_UNITSQUARE, NL_UVDIR );

            Muu = Muv = Mvv = 0.0;

            for ( a = 0; a <= ma; a++ )
            {
                if( a EQ 0 )
                    ul = 0.0;

                else if( a EQ ma )
                    ul = 1.0;

                else
                    ul = a * uinc;

                for ( b = 0; b <= mb; b++ )
                {
                    if( b EQ 0 )
                        vl = 0.0;

                    else if( b EQ mb )
                        vl = 1.0;

                    else
                        vl = b * vinc;

                    error = N_SrfOffsetGetFirstSecondDerivs( sbz[i][j], ul, vl, fbz[i][j], vbz[i][j], NL_LEFT, NL_LEFT, OD );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_PtMagnitude( OD[2][0], &duu );
                    N_PtMagnitude( OD[1][1], &duv );
                    N_PtMagnitude( OD[0][2], &dvv );

                    if( duu GT Muu )
                        Muu = duu;

                    if( duv GT Muv )
                        Muv = duv;

                    if( dvv GT Mvv )
                        Mvv = dvv;
                }
            }

            num = 0.125 *(Muu + Mvv) + 0.25 *Muv;
            gro = NL_MAX( f2, sqrt( num ) );
            del = f1 * gro;
            del = ceil( del );
            np[i][j] = (NL_INDEX)del;
        }
    }

    /* Get maximum number of points to be sampled */
    /* Maybe gets too many points, erring on being
    too safe */

    mu = 0;

    for ( i = 0; i <= k; i++ )
    {
        nu[i] = np[i][0];

        for ( j = 1; j <= l; j++ )
        {
            if( np[i][j]GT nu[i] )
                nu[i] = np[i][j];
        }
        mu += nu[i];
    }

    mv = 0;

    for ( j = 0; j <= l; j++ )
    {
        nv[j] = np[0][j];

        for ( i = 1; i <= k; i++ )
        {
            if( np[i][j]GT nv[j] )
                nv[j] = np[i][j];
        }
        mv += nv[j];
    }

    /* Compute parameters */

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mv, &SL );

    if( v EQ NULL )
        NL_QUIT;

    ul = UP[p];
    i = p + 1;
    j = 0;
    u[0] = ul;
    l = 0;

    while( i LT rp )
    {
        while( i LT rp AND UP[i]EQ UP[i + 1] )
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

    vl = VP[q];
    i = q + 1;
    j = 0;
    v[0] = vl;
    l = 0;

    while( i LT sp )
    {
        while( i LT sp AND VP[i]EQ VP[i + 1] )
            i++;
        vr = VP[i];

        vinc = (vr - vl) / nv[j];

        for ( k = 1; k < nv[j]; k++ )
            v[++l] = vl + k * vinc;
        v[++l] = vr;

        vl = vr;
        i++;
        j++;
    }

    /* Sample surface */

    P = N_AllocPt2dArray( mu, mv, &SL );

    if( P EQ NULL )
        NL_QUIT;

    d = N_AllocReal2dArray( mu, mv, &SL );

    if( d EQ NULL )
        NL_QUIT;

    V = N_AllocPt2dArray( mu, mv, &SL );

    if( V EQ NULL )
        NL_QUIT;

    error = N_SrfEvalPtGrid( &surB, u, v, mu, mv, NL_LEFT, NL_LEFT, P );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SFuncEvalGrid( &sfnl, u, v, mu, mv, NL_LEFT, NL_LEFT, d );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfEvalPtGrid( &surD, u, v, mu, mv, NL_LEFT, NL_LEFT, V );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= mv; j++ )
        {
            /* Note: surD(u,v) should produce a unit vector for all u,v. Just in case
                it does not, we normalise the result */
            N_VectorNormalize( V[i][j], &VN, &mag );
            N_VectorPtAlongVector( P[i][j], d[i][j], VN, &P[i][j] );
        }
    }

    /* Get knot vectors */

    knu = N_AllocKnotVectorAndArray( mu + pp + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    knv = N_AllocKnotVectorAndArray( mv + qq + 1, &SL );

    if( knv EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        error = N_FitCalcSrfParamValues( (NL_VOID ** )P, mu, mv, NL_EPOINT, par, u, v );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_FitCrvCalcKnotVector( u, mu, pp, knu );
    N_FitCrvCalcKnotVector( v, mv, qq, knv );

    /* Interpolate points */

    N_SrfInitArrays( surQ );
    error = N_FitSrfToPtsKnots( (NL_VOID ** )P, mu, mv, NL_EPOINT, u, v, knu, knv, pp, qq, surQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Remove knots */
    /* Very slow on large mu, mv */

    error = N_SrfRemoveAllKnots( surQ, tol, NL_UVDIR, surQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output surface */

    N_SrfGetArraySizes( surQ, &n, &m, &i, &i );

    if( n LT mu OR m LT mv )
    {
        error = N_SrfCompress( surQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Rescale knot vector if necessary */

    if( par NEQ NL_INHERITED )
    {
        N_SrfGetKnots( surQ, &rq, &sq, &UQ, &VQ );

        if( UP[0]NEQ UQ[0]OR UP[rp]NEQ UQ[rq]OR VP[0]NEQ VQ[0]OR VP[sp]NEQ VQ[sq] )
        {
            N_CreateRectangle( &R, UP[0], UP[rp], VP[0], VP[sp] );

            if( UP[0]EQ UQ[0]AND UP[rp]EQ UQ[rq] )
                N_SrfReparamToInterval( surQ, R, NL_VDIR );

            else if( VP[0]EQ VQ[0]AND VP[sp]EQ VQ[sq] )
                N_SrfReparamToInterval( surQ, R, NL_UDIR );

            else
                N_SrfReparamToInterval( surQ, R, NL_UVDIR );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_SRFOFFSET: Offset of Nlib surface using point sampling              */
/**********************************************************************/

NL_PRIVATE NL_REAL slm = 1000.0;
NL_PRIVATE NL_INDEX totalmax = 10000;

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine computes an approximation to the offset
     surface of a  given NL_G1 continuous Nlib  surface. If the surface is
     a NL_PLANE, SPHERE, TORUS, CYLINDER or a CONE, the offset is computed
     precisely.  Surfaces of  revolution  and  extruded  surfaces  with 
     planar generator curves are computed by  offsetting  the generator
     curve and  revolving/extruding it.  Free - form offsets are computed
     by a process consisting of point  sampling, interpolation and knot 
     removal. A typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_DEGREE   pp, qq;
       NL_REAL     d, eps, tol;
       NL_STACKS   SG;
       ...
       (define surP, get eps, tol, pp and qq);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfOffset(&surP, d, pp, qq, NL_INHERITED, eps, tol, &surQ, &SG);

     MEMORY TO STORE THE  OUTPUT NL_SURFACE  surQ  IS ALLOCATED INSIDE THE
     ROUTINE! IT IS  ASSUMED THAT surP IS AT LEAST NL_G1 CONTINUOUS AND IS
     A VALID Nlib NL_SURFACE!


   ACCESS:
   
     surP  , input  ,  NURBS surface
     d     , input  ,  Offset distance
     pp, qq , input ,  Degrees of approximating surface.
     par   , input  ,  Flag:
                         NL_UNIFORM    : uniform parametrization
                         NL_CHORDLENGTH: chordlength parametrization
                         NL_CENTRIPETAL: centipetal parametrization
                         NL_INHERITED  : surP's parametrization 
                       This flag is not used for the special  cases  of
                       cone, cylinder, sphere, torus,  and  surface  of
                       revolution
     eps   , input  ,  Tolerance to check surface type (see below)
     tol   , input  ,  Tolerance of approximation 
     surQ  , output ,  Approximating surface
     SG    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   NOTE: eps needs to be as large as the application will allow.
   Eps should be larger than tol (suggest 10*tol)
   A small eps will bypass special surfaces (surfaces of rev, or 
   extruded surfaces)

   ***********************************************************************/

NL_FLAG N_SrfOffset( NL_SURFACE *surP, NL_REAL d, NL_DEGREE pp, NL_DEGREE qq, NL_FLAG par, NL_REAL eps, NL_REAL tol, NL_SURFACE *surQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfOffset");

    NL_FLAG stp, flt, dir, rev, error = NL_NO;

    NL_INDEX ** np, *nu, *nv, i, j, ii, jj, k, l, n, m, rp, sp, rq, sq, mu, mv, nnn, mmm;

    NL_DEGREE p, q, pq;

    NL_REAL *UP, *VP, *UQ, *VQ, *u, *v, Muu, Muv, Mvv, ul, ur, vl, vr, uinc, vinc, f1, f2, del, num, gro, exp, r1, r2, ac, al, h, per, ddd, du, dv, dot;

    NL_POINT ** P, ** SD, AA, BB, CC, DD, Pnt;

    NL_CPOINT ** Pw;

    NL_VECTOR ** N, NN, V1, V2, V3, cross, Ta, Np, Nq, B, C;

    NL_KNOTVECTOR *knu, *knv;

    NL_RECTANGLE R;

    NL_CURVE *curA, *curB, curP;

    NL_SURFACE *** bez;

    NL_PARAMETER u0, u1, v0, v1;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    if( surQ == NULL )
        NL_OUT;

    /* Get surface type */

    error = N_SrfType( surP, eps, &curA, &curB, &AA, &BB, &CC, &DD, &V1, &V2, &r1, &r2, &ac, &al, &h, &stp, &dir, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( N_PtsAreEqual( CC, NL_ZERO ) )
        flt = NL_NO;
    else
        flt = NL_YES;

    /* Offset planar surface */

    if( stp EQ NL_NPLANE )
    {
        N_VectorDiff( BB, AA, &V1 );
        N_VectorDiff( DD, AA, &V2 );
        N_VectorCross( V1, V2, &NN );

        error = N_VectorNormalizeRef( &NN ); /* normalize */

        if (error EQ NL_YES) /* Assumed error is due to coincident BB,AA or DD,AA */
        {
            N_VectorDiff( CC, AA, &V3 ); /* Replacement for degenerate vector*/
            N_VectorCross( V3, V2, &NN ); /* Guess that V1 was degenerate */

            error = N_VectorNormalizeRef( &NN ); /* normalize */

            if (error EQ NL_YES) /* Check if that solved the error */
            {
                N_VectorCross(V1, V3, &NN); /* If not, replace degenerate V2 */

                error = N_VectorNormalizeRef(&NN); /* normalize */
            }
        }

        if( error EQ NL_YES )
            NL_OUT;
        N_SrfInitArrays( surQ );

        if( par == NL_INHERITED )
        {
            N_VectorScaleRef( &NN, d, &NN );     /* Scale normal by distance */

            error = N_SrfCopy( surP, surQ, SG ); /* copy */

            if( error EQ NL_YES )
                NL_OUT;
            N_SrfTranslate( surQ, NN ); /* Translate by scaled normal */
        }
        else
        { /* build new surface param on 0-1 */
            N_VectorPtAlongVector( AA, d, NN, &AA );
            N_VectorPtAlongVector( BB, d, NN, &BB );
            N_VectorPtAlongVector( CC, d, NN, &CC );
            N_VectorPtAlongVector( DD, d, NN, &DD );

            error = N_CreateSrfCornerPts( AA, BB, DD, CC, surQ, SG );
        }

        NL_OUT;
    }

    /* Offset surface of revolution */

    rev = NL_NO;

    if( stp EQ NL_NSPHERE )
        rev = NL_YES;

    else if( stp EQ NL_NTORUS )
        rev = NL_YES;

    else if( stp EQ NL_NCONE )
        rev = NL_YES;

    else if( stp EQ NL_NCYLINDER )
        rev = NL_YES;

    else if( stp EQ NL_NREVOLUTION )
        rev = NL_YES;

    /* Allocate memory for derivatives */
    SD = N_AllocPt2dArray( 1, 1, &SL );

    if( SD EQ NULL )
        NL_QUIT;

    if( rev EQ NL_YES AND flt EQ NL_YES )
    {
        if( stp EQ NL_NCONE OR stp EQ NL_NCYLINDER )
            pq = 1;
        else
            pq = (dir EQ NL_UDIR) ? qq : pp;

        N_SrfGetParameterBounds( surP, &u0, &u1, &v0, &v1 );
        error = N_SrfEvalPtPtDerivNormalFast( surP, u0, v0, NL_RIGHT, NL_RIGHT, &Pnt, &B, &C, &Np, SD );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetParamBounds( curA, &u0, &u1 );
        N_CrvEvalTangent( curA, u0, NL_RIGHT, &Pnt, &Ta );

        N_VectorCross( CC, Ta, &cross );
        N_VectorDot( Np, cross, &dot );

        if( dot < 0.0 )
            N_VectorScale( CC, -1.0, &CC );

        N_CrvInitArrays( &curP );
        error = N_CrvOffsetPtSampling( curA, CC, d, pq, NL_NO, par, tol, &curP, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CreateRevolvedSrf( &curP, AA, BB, al, NL_QUADRATIC, surQ, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetParameterBounds( surQ, &u0, &u1, &v0, &v1 );
        error = N_SrfEvalPtPtDerivNormalFast( surQ, u0, v0, NL_RIGHT, NL_RIGHT, &Pnt, &B, &C, &Nq, SD );
        N_VectorDot( Np, Nq, &dot );

        if( dot < 0.0 )
        {
            error = N_SwapUV( surQ, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }

        NL_OUT;
    }

    /* Offset extruded surface */

    if( stp EQ NL_NEXTRUSION )
    {
        error = N_CrvIsPlanar( curA, eps, &DD, &NN, &flt, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorCross( AA, NN, &V1 );
        N_VectorMagnitude( V1, &f1 );

        if( flt EQ NL_TRUE AND f1 LT NL_MTOL )
        {
            if( dir EQ NL_UDIR )
                pq = qq;
            else
                pq = pp;

            N_SrfGetParameterBounds( surP, &u0, &u1, &v0, &v1 );
            error = N_SrfEvalPtPtDerivNormalFast( surP, u0, v0, NL_RIGHT, NL_RIGHT, &Pnt, &B, &C, &Np, SD );
            N_CrvGetParamBounds( curA, &u0, &u1 );
            N_CrvEvalTangent( curA, u0, NL_RIGHT, &Pnt, &Ta );
            N_VectorCross( NN, Ta, &cross );
            N_VectorDot( Np, cross, &dot );

            if( dot < 0.0 )
                N_VectorScale( NN, -1.0, &NN );

            N_CrvInitArrays( &curP );
            error = N_CrvOffsetPtSampling( curA, NN, d, pq, NL_NO, par, tol, &curP, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CreateSrfExtrudeCrv( &curP, AA, r1, dir, surQ, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_SrfGetParameterBounds( surQ, &u0, &u1, &v0, &v1 );
            error = N_SrfEvalPtPtDerivNormalFast( surQ, u0, v0, NL_RIGHT, NL_RIGHT, &Pnt, &B, &C, &Nq, SD );
            N_VectorDot( Np, Nq, &dot );

            if( dot < 0.0 )
            {
                error = N_SwapUV( surQ, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            NL_OUT;
        }
    }

    /* Offset general surface */
    N_SrfGetKnots( surP, &rp, &sp, &UP, &VP );
    N_SrfGetDegrees( surP, &p, &q );

    /* set output surface control point size */
    /* these are values for N_FitSrfLstSqApprox only (see below) */
    nnn = rp + 1;
    mmm = sp + 1;

    if( pp EQ 1 OR qq EQ 1 )
        exp = 0.5;
    else
        exp = 0.34;

    if( N_FloatOpIsBad( 1.0, tol, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    f1 = 1.0 / tol;
    f1 = pow( f1, exp );
    pq = NL_MAX( pp, qq );
    f2 = 2.0 * (NL_REAL)pq;

    du = 0.00005 *(UP[rp] - UP[0]);
    dv = 0.00005 *(VP[sp] - VP[0]);

    /* Get Bezier patches */

    error = N_SrfDecomposeToBez( surP, &bez, &k, &l, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Allocate memory */

    np = N_AllocInt2dArray( k, l, &SL );

    if( np EQ NULL )
        NL_QUIT;

    nu = N_AllocInt1dArray( k, &SL );

    if( nu EQ NULL )
        NL_QUIT;

    nv = N_AllocInt1dArray( l, &SL );

    if( nv EQ NULL )
        NL_QUIT;

    N = N_AllocPt2dArray( p, q, &SL );

    if( N EQ NULL )
        NL_QUIT;

    /* Get number of sampling points */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            N_SrfGetParameterBounds( bez[i][j], &ul, &ur, &vl, &vr );

            if( ur - ul LE du OR vr - vl LE dv )
            {
                np[i][j] = 1;
                continue;
            }

            N_SrfReparamToInterval( bez[i][j], NL_UNITSQUARE, NL_UVDIR );

            error = N_SrfOffsetGetMaxDeriv( bez[i][j], d, &Muu, &Muv, &Mvv );

            if( error EQ NL_YES )
                NL_OUT;

            num = 0.125 *(Muu + Mvv) + 0.25 *Muv;
            gro = NL_MAX( f2, sqrt( num ) );
            del = f1 * gro;
            del = ceil( del );

            if( del GE slm )
            {
                np[i][j] = (NL_INDEX)slm;

                N_SrfGetCPts( bez[i][j], &mu, &mv, &Pw );

                uinc = 1.0 / p;
                vinc = 1.0 / q;

                for ( ii = 0; ii <= p; ii++ )
                {
                    ul = ii * uinc;

                    if( ul GT 1.0 )
                        ul = 1.0;

                    for ( jj = 0; jj <= q; jj++ )
                    {
                        vl = jj * vinc;

                        if( vl GT 1.0 )
                            vl = 1.0;

                        error = N_SrfEvalPtPtDerivNormalFast( bez[i][j], ul, vl, NL_LEFT, NL_LEFT, &AA, &BB, &CC, &N[ii][jj], SD );

                        if( error EQ NL_YES )
                        {
                            error = N_SrfEvalPtNormalAtPole( bez[i][j], ul, vl, NL_LEFT, NL_LEFT, &AA, &BB, &CC, &N[ii][jj], SD );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }

                for ( ii = 0; ii <= p; ii++ )
                {
                    for ( jj = 0; jj <= q; jj++ )
                    {
                        N_CPtToPtAndW( Pw[ii][jj], &AA, &h );
                        N_VectorPtAlongVector( AA, d, N[ii][jj], &AA );
                        N_Weight( AA, h, &Pw[ii][jj] );
                    }
                }

                error = N_SrfGetAverageLen( bez[i][j], &ac, &r1, &r2 );

                if( error EQ NL_YES )
                    NL_OUT;

                del = sqrt( ac / tol );
                del = ceil( del );
                jj = (NL_INDEX)del;

                if( jj LT np[i][j] )
                    np[i][j] = jj;
            }
            else
                np[i][j] = (NL_INDEX)del;
        }
    }

    /* Get maximum number of points to be sampled */

    mu = 0;

    for ( i = 0; i <= k; i++ )
    {
        nu[i] = np[i][0];

        for ( j = 1; j <= l; j++ )
        {
            if( np[i][j]GT nu[i] )
                nu[i] = np[i][j];
        }
        mu += nu[i];
    }

    mv = 0;

    for ( j = 0; j <= l; j++ )
    {
        nv[j] = np[0][j];

        for ( i = 1; i <= k; i++ )
        {
            if( np[i][j]GT nv[j] )
                nv[j] = np[i][j];
        }
        mv += nv[j];
    }

    if( (mu + 1)*(mv + 1)GT totalmax )
    {
        ddd = ((NL_REAL)mu + 1.0) * ((NL_REAL)mv + 1.0);
        per = ((NL_REAL)totalmax) / ddd;
        per = sqrt( per );

        mu = 0;

        for ( i = 0; i <= k; i++ )
        {
            ddd = (NL_REAL)(nu[i]);
            nu[i] = (NL_INDEX)(ddd * per);

            if( nu[i]LT p )
                nu[i] = p;
            mu += nu[i];
        }

        mv = 0;

        for ( j = 0; j <= l; j++ )
        {
            ddd = (NL_REAL)(nv[j]);
            nv[j] = (NL_INDEX)(ddd * per);

            if( nv[j]LT q )
                nv[j] = q;
            mv += nv[j];
        }
    }

	if (mu < 0 || mv < 0)
		NL_QUIT;

	/* Adjust output surface control point size */
    /* these are values for N_FitSrfLstSqApprox only (see below) */
	if( nnn > mu ) 
		nnn = mu;

	if( mmm > mv ) 
		mmm = mv;

    /* Compute parameters */

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    v = N_AllocReal1dArray( mv, &SL );

    if( v EQ NULL )
        NL_QUIT;

    ul = UP[p];
    i = p + 1;
    j = 0;
    u[0] = ul;
    l = 0;

    while( i LT rp )
    {
        while( i LT rp AND UP[i]EQ UP[i + 1] )
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

    vl = VP[q];
    i = q + 1;
    j = 0;
    v[0] = vl;
    l = 0;

    while( i LT sp )
    {
        while( i LT sp AND VP[i]EQ VP[i + 1] )
            i++;
        vr = VP[i];

        vinc = (vr - vl) / nv[j];

        for ( k = 1; k < nv[j]; k++ )
            v[++l] = vl + k * vinc;
        v[++l] = vr;

        vl = vr;
        i++;
        j++;
    }

    /* Sample surface */

    P = N_AllocPt2dArray( mu, mv, &SL );

    if( P EQ NULL )
        NL_QUIT;

    N = N_AllocPt2dArray( mu, mv, &SL );

    if( N EQ NULL )
        NL_QUIT;

    error = N_SrfGetGridPtsNormals( surP, u, v, mu, mv, NL_LEFT, NL_LEFT, P, N );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= mv; j++ )
        {
            N_VectorPtAlongVector( P[i][j], d, N[i][j], &P[i][j] );
        }
    }

    N_FreePt2dArray( N, &SL );

    /* Get knot vectors */

    knu = N_AllocKnotVectorAndArray( mu + pp + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    knv = N_AllocKnotVectorAndArray( mv + qq + 1, &SL );

    if( knv EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        error = N_FitCalcSrfParamValues( (NL_VOID ** )P, mu, mv, NL_EPOINT, par, u, v );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_FitCrvCalcKnotVector( u, mu, pp, knu );
    N_FitCrvCalcKnotVector( v, mv, qq, knv );

    /* Interpolate points */

    N_SrfInitArrays( surQ );

    if( par EQ NL_INHERITED )
        error = N_FitSrfToPtsKnots( (NL_VOID ** )P, mu, mv, NL_EPOINT, u, v, knu, knv, pp, qq, surQ, SG );
    else
        error = N_FitSrfLstSqApprox( P, mu, mv, nnn, mmm, 3, 3, par, surQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_FreePt2dArray( P, &SL );

    /* Remove knots */

    error = N_SrfRemoveAllKnots( surQ, tol, NL_UVDIR, surQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output surface */

    N_SrfGetArraySizes( surQ, &n, &m, &i, &i );

    if( n LT mu OR m LT mv )
    {
        error = N_SrfCompress( surQ, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Rescale knot vector if necessary */

    if( par NEQ NL_INHERITED )
    {
        N_SrfGetKnots( surQ, &rq, &sq, &UQ, &VQ );

        if( UP[0]NEQ UQ[0]OR UP[rp]NEQ UQ[rq]OR VP[0]NEQ VQ[0]OR VP[sp]NEQ VQ[sq] )
        {
            N_CreateRectangle( &R, UP[0], UP[rp], VP[0], VP[sp] );

            if( UP[0]EQ UQ[0]AND UP[rp]EQ UQ[rq] )
                N_SrfReparamToInterval( surQ, R, NL_VDIR );

            else if( VP[0]EQ VQ[0]AND VP[sp]EQ VQ[sq] )
                N_SrfReparamToInterval( surQ, R, NL_UDIR );

            else
                N_SrfReparamToInterval( surQ, R, NL_UVDIR );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
