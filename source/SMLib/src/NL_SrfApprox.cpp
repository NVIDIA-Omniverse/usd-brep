// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfApprox.c: Surface approximation routines                        */
/**********************************************************************/

#include "StdAfx.h"


#include <nurbs.h>
#include <NL_Globals.h>
#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */



static NL_REAL ST_GetDist(NL_REAL);
static NL_FLAG ST_GetPoint(NL_CURVE**, NL_CURVE**, NL_CFUN**, NL_CFUN**, NL_REAL, NL_REAL, NL_POINT*,
    NL_REAL (*)[2][2], NL_REAL, NL_REAL, NL_REAL, NL_REAL);

#if NLIB_UNUSED

/**********************************************************************/
/* N_ApproxConeWithSrf: Approximate cylinder/cone surface/patch                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a NON-RATIONAL approximation of a cylinder or 
     a cone or a patch  of them  by extruding  an approximate  circle or 
     circular arc. The degree  of the  approximation, the  continuity of 
     the approximating circle, as well as the error of approximation can
     be controlled. If the output surface is initialized to NULL, memory 
     to  store  new control  points  and  knots is  allocated. A typical 
     calling example is:

       NL_POINT    C;
       NL_VECTOR   X, Y;
       NL_DEGREE   p;
       NL_INDEX    cnt;
       NL_REAL     rb, rt, as, ae, h, tol;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get C, X, Y, rb, rt, as, ae, h, p, cnt and tol);
       ...
       N_SrfInitArrays(&sur);
       N_ApproxConeWithSrf(C,X,Y,as,ae,rb,rt,h,p,cnt,tol,&sur,&SG);

     If  memory is  available, sur  is not  initialized and the  routine
     assumes that  memory allocation has  been done. However, it  checks  
     for the proper  amount by looking at  the highest  indexes in sur's 
     knot vector and polygon objects. S(u,0) and S(u,1) are the base and
     top circles.


   ACCESS:
   
     C,X,Y , input  ,  Center of  bottom circle  and orthogonal  axes of 
                       bottom plane
     as,ae , input  ,  Start and end sweep angles of base circle
                       0 <= as < ae <= 360 must hold
     rb,rt , input  ,  Bottom and top radii
     h     , input  ,  Height of cylinder/cone
     p     , input  ,  Degree of sur in the direction of the circles
     cnt   , input  ,  Level of continuity of the  circle approximation,
                       i.e. the degree p non-rational  curve is at least
                       cnt continuous. cnt < p must hold!
     tol   , input  ,  Error tolerance; the approximate surface does not
                       deviate from the precise surface more than tol!
     sur   , output ,  NURBS cylinder/cone surface/patch
     SG    , input  ,  sur's stack stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxConeWithSrf( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL as, NL_REAL ae, NL_REAL rb, NL_REAL rt, NL_REAL h, NL_DEGREE p, NL_INDEX cnt, NL_REAL tol, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxConeWithSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, nc;

    NL_REAL *U, *UQ, *VQ, fac;

    NL_POINT D;

    NL_VECTOR Z, V, F;

    NL_CPOINT *Pw, ** Qw;

    NL_CURVE curM;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get some vectors */

    if( rb LE NL_MTOL AND rt LE NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    N_VectorCross( X, Y, &Z );

    error = N_VectorNormalizeRef( &Z );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorPtAlongVector( C, h, Z, &D );

    /* Get maximum circle */

    nc = NL_MAX( 2, p - 1 );

    N_CrvInitArrays( &curM );

    if( rb GT rt )
    {
        error = N_ApproxArcWithCrv( C, X, Y, rb, as, ae, p, cnt, nc, tol, NL_ABSOLUTE, &curM, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_ApproxArcWithCrv( D, X, Y, rt, as, ae, p, cnt, nc, tol, NL_ABSOLUTE, &curM, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_CrvGetCPtsDegreeAndKnots( &curM, &n, &Pw, &p, &m, &U );

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, n, 1, p, 1, m, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Get control points */

    if( rb GT rt )
    {
        for ( i = 0; i <= n; i++ )
            N_CopyCPt( Pw[i], &Qw[i][0] );

        if( rt LT NL_MTOL )
        {
            for ( i = 0; i <= n; i++ )
                N_Weight( D, 1.0, &Qw[i][1] );
        }
        else
        {
            fac = rt / rb;

            N_VectorDiff( D, C, &V );
            N_VectorCreate( fac, fac, fac, &F );
            N_CrvTranslate( &curM, V );
            N_CrvScale( &curM, D, F );

            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Pw[i], &Qw[i][1] );
        }
    }
    else
    {
        for ( i = 0; i <= n; i++ )
            N_CopyCPt( Pw[i], &Qw[i][1] );

        if( rb LT NL_MTOL )
        {
            for ( i = 0; i <= n; i++ )
                N_Weight( C, 1.0, &Qw[i][0] );
        }
        else
        {
            fac = rb / rt;

            N_VectorDiff( C, D, &V );
            N_VectorCreate( fac, fac, fac, &F );
            N_CrvTranslate( &curM, V );
            N_CrvScale( &curM, C, F );

            for ( i = 0; i <= n; i++ )
                N_CopyCPt( Pw[i], &Qw[i][0] );
        }
    }

    N_SrfRatToNonRat( sur );

    /* Get the knots */

    for ( i = 0; i <= m; i++ )
        UQ[i] = U[i];

    for ( j = 0; j <= 1; j++ )
    {
        VQ[j] = 0.0;
        VQ[2 + j] = 1.0;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_ApproxNurbsWithNonRatSrf: Approximate NURBS surface with non-rational surface      */
/**********************************************************************/

NL_PRIVATE NL_REAL slm = 1000.0;

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine approximates a given NURBS surface with
     a non-rational surface of  selected degrees. The method is global,
     i.e. the approximating  surface does not deviate from the origonal
     surface more than the tolerance anywhere on the surface. A typical 
     calling example is:

       NL_SURFACE  surP, surQ;
       NL_DEGREE   pp, qq;
       NL_REAL     tol;
       NL_STACKS   SG;
       ...
       (define surP, get tol, pp and qq);
       ...
       N_SrfInitArrays(&surQ);
       N_ApproxNurbsWithNonRatSrf(&surP,tol,pp,qq,NL_INHERITED,&surQ,&SG);

     MEMORY TO STORE THE  OUTPUT NL_SURFACE  surQ  IS ALLOCATED INSIDE THE
     ROUTINE! IT IS  ASSUMED THAT surP IS AT LEAST NL_G1 CONTINUOUS AND IS
     A VALID Nlib NL_SURFACE! IF THE  NL_SURFACE IS A  RULED NL_SURFACE, IT WILL 
     BE APPROXIMATED AS A  RULED NL_SURFACE IF THE PARAMETRIZATION NL_FLAG IS
     SET TO NL_INHERITED.


   ACCESS:
   
     surP  , input  ,  NURBS surface
     tol   , input  ,  Error tolerance
     pp,qq , input  ,  Degrees of approximating surface.
     par   , input  ,  Flag:
                         NL_UNIFORM    : uniform parametrization
                         NL_CHORDLENGTH: chordlength parametrization
                         NL_CENTRIPETAL: centipetal parametrization
                         NL_INHERITED  : surP's parametrization 
     surQ  , output ,  Approximating surface
     SG    , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxNurbsWithNonRatSrf( NL_SURFACE *surP, NL_REAL tol, NL_DEGREE pp, NL_DEGREE qq, NL_FLAG par, NL_SURFACE *surQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxNurbsWithNonRatSrf");

    NL_FLAG url = NL_NO, vrl = NL_NO, error = NL_NO;

    NL_INDEX ** np, *nu, *nv, i, j, k = 0, l = 0, n, m, rp, sp, rq, sq, mu = 0, mv = 0, jj;

    NL_DEGREE p, q, pq;

    NL_REAL *UP, *VP, *UQ, *VQ, *u, *v, Muu, Muv, Mvv, ul, ur, vl, vr, uinc, vinc, f1, f2, del, num, gro, exp = 0.0, r1, r2, ac;

    NL_POINT ** P, *Q;

    NL_KNOTVECTOR *knu, *knv;

    NL_RECTANGLE R;

    NL_CURVE ** cbz, curB[2], curI[2];

    NL_SURFACE *** sbz;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetKnots( surP, &rp, &sp, &UP, &VP );
    N_SrfGetDegrees( surP, &p, &q );
    N_SrfGetArraySizes( surP, &n, &m, &i, &i );

    /* Initialize */

    if( n EQ 1 AND p EQ 1 )
        url = NL_YES;

    if( m EQ 1 AND q EQ 1 )
        vrl = NL_YES;

    if( url EQ NL_YES OR vrl EQ NL_YES )
    {
        if( url EQ NL_YES )
        {
            if( qq EQ 1 )
                exp = 0.5;
            else
                exp = 0.34;
        }
        else if( vrl EQ NL_YES )
        {
            if( pp EQ 1 )
                exp = 0.5;
            else
                exp = 0.34;
        }
    }
    else
    {
        if( pp EQ 1 OR qq EQ 1 )
            exp = 0.5;
        else
            exp = 0.34;
    }

    if( N_FloatOpIsBad( 1.0, tol, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    f1 = 1.0 / tol;
    f1 = pow( f1, exp );
    pq = NL_MAX( pp, qq );
    f2 = (NL_REAL)pq;

    /* Special case out ruled surfaes */

    if( par EQ NL_INHERITED AND( url EQ NL_YES OR vrl EQ NL_YES ) )
    {
        N_CreateRectangle( &R, UP[0], UP[rp], VP[0], VP[sp] );

        /****************************/
        /* Ruled in the u-direction */
        /****************************/

        if( url EQ NL_YES )
        {
            np = N_AllocInt2dArray( 1, m, &SL );

            if( np EQ NULL )
                NL_QUIT;

            nv = N_AllocInt1dArray( m, &SL );

            if( nv EQ NULL )
                NL_QUIT;

            /* Extract boundaries */

            N_CrvInitArrays( &curB[0] );
            error = N_SrfExtractIsoCrv( surP, UP[0], NL_VDIR, &curB[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &curB[1] );
            error = N_SrfExtractIsoCrv( surP, UP[rp], NL_VDIR, &curB[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Sample boundaries */

            for ( i = 0; i <= 1; i++ )
            {
                error = N_CrvDecomposeBez( &curB[i], &cbz, &l, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( j = 0; j <= l; j++ )
                {
                    N_CrvReparamToInterval( cbz[j], NL_UNITSPAN );

                    error = N_CrvGetMax2ndDeriv( cbz[j], &Mvv );

                    if( error EQ NL_YES )
                        NL_OUT;

                    num = 0.125 *Mvv;
                    gro = NL_MAX( f2, sqrt( num ) );
                    del = f1 * gro;
                    del = ceil( del );
                    np[i][j] = (NL_INDEX)del;
                }
            }

            /* Get maximum number of points */

            mv = 0;

            for ( j = 0; j <= l; j++ )
            {
                nv[j] = np[0][j];

                if( np[1][j]GT nv[j] )
                    nv[j] = np[1][j];

                mv += nv[j];
            }

            /* Get parameters and knot vector */

            v = N_AllocReal1dArray( mv, &SL );

            if( v EQ NULL )
                NL_QUIT;

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

            knv = N_AllocKnotVectorAndArray( mv + qq + 1, &SL );

            if( knv EQ NULL )
                NL_QUIT;

            N_FitCrvCalcKnotVector( v, mv, qq, knv );

            /* Sample curves and fit */

            Q = N_AllocPt1dArray( mv, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            for ( i = 0; i <= 1; i++ )
            {
                for ( j = 0; j <= mv; j++ )
                {
                    error = N_CrvEval( &curB[i], v[j], NL_LEFT, &Q[j] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvInitArrays( &curI[i] );
                error = N_FitCrvInterpGivenParams( (NL_VOID *)Q, mv, NL_EPOINT, v, knv, qq, &curI[i], &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* Get ruled surface */

            N_SrfInitArrays( surQ );
            error = N_CreateRuledSrf( &curI[0], &curI[1], NL_UDIR, surQ, &SL, SG );

            if( error EQ NL_YES )
                NL_OUT;

            /* Set parameter domain back to original */

            N_SrfReparamToInterval( surQ, R, NL_UDIR );

            /* Raise degree if necessary */

            if( pp GT 1 )
            {
                error = N_SrfElevateDegree( surQ, pp - 1, NL_UDIR, surQ, SG, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }

        /****************************/
        /* Ruled in the v-direction */
        /****************************/

        if( vrl EQ NL_YES )
        {
            np = N_AllocInt2dArray( n, 1, &SL );

            if( np EQ NULL )
                NL_QUIT;

            nu = N_AllocInt1dArray( n, &SL );

            if( nu EQ NULL )
                NL_QUIT;

            /* Extract boundaries */

            N_CrvInitArrays( &curB[0] );
            error = N_SrfExtractIsoCrv( surP, VP[0], NL_UDIR, &curB[0], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &curB[1] );
            error = N_SrfExtractIsoCrv( surP, VP[sp], NL_UDIR, &curB[1], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Sample boundaries */

            for ( j = 0; j <= 1; j++ )
            {
                error = N_CrvDecomposeBez( &curB[j], &cbz, &k, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                for ( i = 0; i <= k; i++ )
                {
                    N_CrvReparamToInterval( cbz[i], NL_UNITSPAN );

                    error = N_CrvGetMax2ndDeriv( cbz[i], &Muu );

                    if( error EQ NL_YES )
                        NL_OUT;

                    num = 0.125 *Muu;
                    gro = NL_MAX( f2, sqrt( num ) );
                    del = f1 * gro;
                    del = ceil( del );
                    np[i][j] = (NL_INDEX)del;
                }
            }

            /* Get maximum number of points */

            mu = 0;

            for ( i = 0; i <= k; i++ )
            {
                nu[i] = np[i][0];

                if( np[i][1]GT nu[i] )
                    nu[i] = np[i][1];

                mu += nu[i];
            }

            /* Get parameters and knot vector */

            u = N_AllocReal1dArray( mu, &SL );

            if( u EQ NULL )
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

            knu = N_AllocKnotVectorAndArray( mu + pp + 1, &SL );

            if( knu EQ NULL )
                NL_QUIT;

            N_FitCrvCalcKnotVector( u, mu, pp, knu );

            /* Sample curves and fit */

            Q = N_AllocPt1dArray( mu, &SL );

            if( Q EQ NULL )
                NL_QUIT;

            for ( j = 0; j <= 1; j++ )
            {
                for ( i = 0; i <= mu; i++ )
                {
                    error = N_CrvEval( &curB[j], u[i], NL_LEFT, &Q[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_CrvInitArrays( &curI[j] );
                error = N_FitCrvInterpGivenParams( (NL_VOID *)Q, mu, NL_EPOINT, u, knu, pp, &curI[j], &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            /* Get ruled surface */

            N_SrfInitArrays( surQ );
            error = N_CreateRuledSrf( &curI[0], &curI[1], NL_VDIR, surQ, &SL, SG );

            if( error EQ NL_YES )
                NL_OUT;

            /* Set parameter domain back to original */

            N_SrfReparamToInterval( surQ, R, NL_VDIR );

            /* Raise degree if necessary */

            if( qq GT 1 )
            {
                error = N_SrfElevateDegree( surQ, qq - 1, NL_VDIR, surQ, SG, SG );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }
    else
    {
        /************************/
        /* Surface is not ruled */
        /************************/

        /* Get Bezier patches */

        error = N_SrfDecomposeToBez( surP, &sbz, &k, &l, &SL );

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

        /* Sample surface */

        for ( i = 0; i <= k; i++ )
        {
            for ( j = 0; j <= l; j++ )
            {
                N_SrfReparamToInterval( sbz[i][j], NL_UNITSQUARE, NL_UVDIR );

                error = N_SrfMaxSecondDeriv( sbz[i][j], &Muu, &Muv, &Mvv );

                if( error EQ NL_YES )
                    NL_OUT;

                num = 0.125 *( Muu + Mvv ) + 0.25 *Muv;
                gro = NL_MAX( f2, sqrt( num ) );
                del = f1 * gro;
                del = ceil( del );
                np[i][j] = (NL_INDEX)del;

                if( del GE slm )
                {
                    error = N_SrfGetAverageLen( sbz[i][j], &ac, &r1, &r2 );

                    if( error EQ NL_YES )
                        NL_OUT;

                    del = sqrt( ac / tol );
                    del = ceil( del );
                    jj = (NL_INDEX)del;

                    if( jj LT np[i][j] )
                        np[i][j] = jj;
                }
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

        error = N_SrfEvalPtGrid( surP, u, v, mu, mv, NL_LEFT, NL_LEFT, P );

        if( error EQ NL_YES )
            NL_OUT;

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
    }

    /* Remove knots */

    error = N_SrfRemoveAllKnots( surQ, tol, NL_UVDIR, surQ, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output surface */

    N_SrfGetArraySizes( surQ, &n, &m, &i, &i );

    if( par EQ NL_INHERITED AND( url EQ NL_YES OR vrl EQ NL_YES ) ) {
    /* skip the N_SrfCompress check, everything should have been created
        properly in the first place. */
    }
    else if( n LT mu OR m LT mv )
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

#if NLIB_UNUSED

/**********************************************************************/
/* N_ApproxRevolvedSrfWithSrf: Approximate surface of revolution                        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine  approximates a surface of revolution by
     approximately revolving an arbitrary  NON-RATIONAL curve  around an 
     arbitrary  axis. The degree, the continuity as well as the error of
     the approximate revolution can be controlled. If the output surface 
     is initialized to NULL, memory to store control points and knots is 
     allocated. A typical calling example is:

       NL_CURVE    cur;
       NL_POINT    S; 
       NL_VECTOR   T;
       NL_REAL     al, tol;
       NL_INDEX    cnt;
       NL_DEGREE   p;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get cur, S, T, al, tol, p and cnt)
       ...
       N_SrfInitArrays(&sur);
       N_ApproxRevolvedSrfWithSrf(&cur,S,T,al,p,cnt,tol,&sur,&SG);

     If  memory is  available, sur  is not  initialized and the  routine
     assumes that  memory allocation has been  done. However, it  checks  
     for the proper  amount by looking at the  highest  indexes in sur's 
     knot vector  and  polygon  objects. 


   ACCESS:
   
     cur , input  ,  NURBS  curve to  be  revolved. THIS  MUST BE  A NON 
                     RATIONAL NL_CURVE!
     S,T , input  ,  Start  point  and  direction   vector  of  axis  of 
                     revolution
     al  , input  ,  Angle of revolution (0<al<=360), measured clockwise
                     looking in the direction of T.
     p   , input  ,  Degree of sur in the direction of revolution
     cnt , input  ,  Level of continuity of the approximate  revolution,
                     i.e. the approximation to the  revolution circle is
                     at least cnt continuous. cnt < p must hold!
     tol , input  ,  Error tolerance; the approximate surface of revolu-
                     tion does not deviate from the precise surface more
                     than tol!
     sur , output ,  Approximate NURBS surface of revolution
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxRevolvedSrfWithSrf( NL_CURVE *cur, NL_POINT S, NL_VECTOR T, NL_REAL al, NL_DEGREE p, NL_INDEX cnt, NL_REAL tol, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxRevolvedSrfWithSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jm = 0, n, m, r, s, nc;

    NL_DEGREE q;

    NL_REAL *U, *V, *UQ, *VQ, *rad, rmax, t, fac;

    NL_POINT *C, P, B;

    NL_VECTOR *X, *Y, D, F;

    NL_CPOINT *Cw, *Pw, ** Qw;

    NL_CURVE curC, curM;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check the profile curve */

    if( N_IsCrvRat( cur ) )
        NL_ERROR( NL_INP_ERR );

    if( al LE 0.0 OR al GT 360.0 )
        NL_ERROR( NL_INP_ERR );

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    N_CrvGetCPtsDegreeAndKnots( cur, &m, &Pw, &q, &s, &V );

    /* Project all control points to the axis */

    C = N_AllocPt1dArray( 3 * (m + 1), &SL );

    if( C EQ NULL )
        NL_QUIT;

    X = &C[m + 1];
    Y = &X[m + 1];

    rad = N_AllocReal1dArray( m, &SL );

    if( rad EQ NULL )
        NL_QUIT;

    N_VectorSum( S, T, &B );

    rmax = -1.0;

    for ( j = 0; j <= m; j++ )
    {
        N_CPtToPtEuclid( Pw[j], &P );

        error = N_ProjectPtLineParam( P, S, B, &C[j], &t );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDiff( P, C[j], &X[j] );
        N_VectorCross( T, X[j], &Y[j] );
        N_VectorMagnitude( X[j], &rad[j] );

        if( rad[j]GT NL_MTOL AND rad[j]GT rmax )
        {
            rmax = rad[j];
            jm = j;
        }
    }

    if( rmax LT 0.0 )
        NL_ERROR( NL_INP_ERR );

    /* Get maximum circle */

    nc = NL_MAX( 2, p - 1 );

    N_CrvInitArrays( &curM );
    error = N_ApproxArcWithCrv( C[jm], X[jm], Y[jm], rad[jm], 0.0, al, p, cnt, nc, tol, NL_ABSOLUTE, &curM, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed for the output */

    N_CrvGetCPtsDegreeAndKnots( &curM, &n, &Cw, &p, &r, &U );

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Compute surface control points */

    error = N_AllocCrvArrays( &curC, n, p, r, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    for ( j = 0; j <= m; j++ )
    {
        N_CPtToPtEuclid( Pw[j], &P );

        if( rad[j]LT NL_MTOL )
        {
            for ( i = 0; i <= n; i++ )
                N_Weight( C[j], 1.0, &Qw[i][j] );
        }
        else if( j EQ jm )
        {
            N_CrvGetCPts( &curM, &n, &Cw );
            N_Weight( P, 1.0, &Qw[0][j] );

            for ( i = 1; i <= n; i++ )
                N_CopyCPt( Cw[i], &Qw[i][j] );

            if( al EQ 360.0 )
                N_Weight( P, 1.0, &Qw[n][j] );
        }
        else
        {
            fac = rad[j] / rmax;

            N_VectorDiff( C[j], C[jm], &D );
            N_VectorCreate( fac, fac, fac, &F );

            error = N_CrvFromCrvTranslation( &curM, D, &curC, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvScale( &curC, C[j], F );

            N_CrvGetCPts( &curC, &n, &Cw );
            N_Weight( P, 1.0, &Qw[0][j] );

            for ( i = 1; i <= n; i++ )
                N_CopyCPt( Cw[i], &Qw[i][j] );

            if( al EQ 360.0 )
                N_Weight( P, 1.0, &Qw[n][j] );
        }
    }

    N_SrfRatToNonRat( sur );

    /* Get the knots */

    for ( i = 0; i <= r; i++ )
        UQ[i] = U[i];

    for ( j = 0; j <= s; j++ )
        VQ[j] = V[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_ApproxSubSrfWithSrf: Approximate subsurface bounded by 4 curves               */
/**********************************************************************/

NL_PRIVATE_TLS NL_POINT Gpnt;
NL_PRIVATE_TLS NL_CURVE *Gcur;
NL_PRIVATE_TLS NL_SURFACE *Gsur;
NL_PRIVATE_TLS NL_REAL G_us, G_ue, G_vs, G_ve;
/*******************************************************************//**


   DESCRIPTION:

     This approximation  routine  approximates a subsurface bounded by 4
     curves. That is, given a surface and 4 curves lying on the  surface
     and forming the 4 boundaries of a subsurface, this function creates
     a surface which approximates the implied subsurface on its interior
     and has the 4 curves as its boundaries.  The  approximating subsur-
     face is defined on parameter domain  0 <= s,t <= 1,  and  functions
     u = f(s,t) and v = g(s,t) may also be returned which relate the s,t
     domain to the u,v domain of the original surface.  No  error toler-
     ance is input,  however,  accuracy  is  influenced by the number of
     knots in the 3D boundary curves,  and hence,  accuracy  can  be in-
     creased by inserting knots into the 4  3D  boundary  curves  before
     calling this function.  The  2D  u,v-domain curves corresponding to 
     the 4  3D  curves (and forming the bounded domain of the subsurface 
     in the u,v-domain)  must also be input to this routine.  The output
     surface and surface functions must be initialized to NULL.  A  typ-
     ical calling example is:

       NL_CURVE    **curS3d, **curT3d, **curS2d, **curT2d;
       NL_SURFACE  sur, subsur;
       NL_SFUN     ufn, vgn;
       NL_STACKS   SG;
       ...
       (get 3D and 2D boundary curves);
       ...
       N_SrfInitArrays(&subsur);
       N_SFuncInitArrays(&ufn);
       N_SFuncInitArrays(&vgn);
       N_ApproxSubSrfWithSrf(curS3d,curT3d,curS2d,curT2d,&sur,&subsur,&ufn,&vgn,&SG);

       or

       N_SrfInitArrays(&subsur);
       N_ApproxSubSrfWithSrf(curS3d,curT3d,curS2d,curT2d,&sur,&subsur,NULL,NULL,&SG);

    ALL 2D AND 3D BOUNDARY CURVES MUST BE NONRATIONAL.  


   ACCESS:
   
     curS3d , input  ,  3D boundary curves at t=0 and t=1.  Need  not be
                        compatible.
     curT3d , input  ,  3D boundary curves at s=0 and s=1.  Need  not be
                        compatible.
     curS2d , input  ,  2D curves in the u,v domain of sur which corres-
                        pond to curS3d. Need not be compatible.
     curT2d , input  ,  2D curves in the u,v domain of sur which corres-
                        pond to curT3d. Need not be compatible.
     sur    , input  ,  Surface containing the subsurface.
     subsur , output ,  Approximating  subsurface.  Its  boundaries  are 
                        precisely the curves of curS3d and curT3d, de-
                        fined on the domain 0<=s,t<=1.
     ufn    , output ,  If:
                          != NULL : Function  u = f(s,t) 
                          == NULL : No output
     vgn    , output ,  If:
                          != NULL : Function  v = g(s,t)
                          == NULL : No output
     SG     , input  ,  subsur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxSubSrfWithSrf( NL_CURVE ** curS3d, NL_CURVE ** curT3d, NL_CURVE ** curS2d, NL_CURVE ** curT2d, NL_SURFACE *sur, NL_SURFACE *subsur, NL_SFUN *ufn, NL_SFUN *vgn, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxSubSrfWithSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, usam, vsam, ns, nt, ms, mt, nn, mm;

    NL_DEGREE p, q, pq1, pq2;

    NL_REAL u, v, d1 = 0.0, d2 = 0.0, d3 = 0.0, ds, dt, *st, *f1, fb, xa, xb, xc, *s, *t, ** Fu, ** Fv, top, toc, d4, uvtop;
    NL_REAL corners[2][2][2];
    NL_REAL us, ue, vs, ve;

    NL_KNOTVECTOR *knt;

    NL_POINT ** pnts, ** SD, uvpnt, Q, R;

    NL_SURFACE sur1;

    NL_CURVE ** scur2, ** scur3, ** tcur2, ** tcur3;

    NL_CFUN ** sfn2, ** tfn2;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Error checks */

    if( (ufn EQ NULL AND vgn NEQ NULL)OR( vgn EQ NULL AND ufn NEQ NULL ) )
        NL_ERROR( NL_INP_ERR );

    /* Get u,v bounds of surface */

    N_SrfGetParameterBounds( sur, &us, &ue, &vs, &ve );
    G_us = us;
    G_ue = ue;
    G_vs = vs;
    G_ve = ve;

    /* Make copies of all input curves, and define all on [0,1]. */
    /* Also ensure all weights = NL_NOW.                           */

    scur2 = N_AllocArrayCrvPtrsAndData( 7, NL_YES, &SL );

    if( scur2 EQ NULL )
        NL_QUIT;

    scur3 = &scur2[2];
    tcur2 = &scur3[2];
    tcur3 = &tcur2[2];

    for ( ii = 0; ii <= 1; ii++ )
    {
        error = N_CrvCopy( curS2d[ii], scur2[ii], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetParamBounds( scur2[ii], &d1, &d2 );

        if( d1 NEQ 0.0 OR d2 NEQ 1.0 )
            N_CrvReparam( scur2[ii], 0.0, 1.0 );

        N_CrvRatToNonRat( scur2[ii] );

        error = N_CrvCopy( curS3d[ii], scur3[ii], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetParamBounds( scur3[ii], &d1, &d2 );

        if( d1 NEQ 0.0 OR d2 NEQ 1.0 )
            N_CrvReparam( scur3[ii], 0.0, 1.0 );

        N_CrvRatToNonRat( scur3[ii] );

        error = N_CrvCopy( curT2d[ii], tcur2[ii], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetParamBounds( tcur2[ii], &d1, &d2 );

        if( d1 NEQ 0.0 OR d2 NEQ 1.0 )
            N_CrvReparam( tcur2[ii], 0.0, 1.0 );

        N_CrvRatToNonRat( tcur2[ii] );

        error = N_CrvCopy( curT3d[ii], tcur3[ii], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetParamBounds( tcur3[ii], &d1, &d2 );

        if( d1 NEQ 0.0 OR d2 NEQ 1.0 )
            N_CrvReparam( tcur3[ii], 0.0, 1.0 );

        N_CrvRatToNonRat( tcur3[ii] );
    }

    /* Compute s,t -> u,v parameter correspondence on boundaries. */
    /* Also load up boundary fit points in the process.           */

    Gsur = sur;

    N_CrvGetDegree( scur3[0], &pq1 );
    N_CrvGetDegree( scur3[1], &pq2 );
    p = NL_MAX( pq1, pq2 );

    if( p LT 2 )
    {
        error = N_CrvElevateDegree( scur3[0], 1, scur3[0], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        p = 2;
    }

    N_CrvGetDegree( tcur3[0], &pq1 );
    N_CrvGetDegree( tcur3[1], &pq2 );
    q = NL_MAX( pq1, pq2 );

    if( q LT 2 )
    {
        error = N_CrvElevateDegree( tcur3[0], 1, tcur3[0], &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
        q = 2;
    }

    N_CrvGetArraySizes( scur3[0], &ii, &kk );
    N_CrvGetArraySizes( scur3[1], &jj, &kk );

    kk = NL_MAX( ii, jj );

    if( kk LT 10 )
        ii = 8;

    else if( kk LT 20 )
        ii = 17;

    else
        ii = 28;

    usam = NL_MAX( p + 4, ii );

    N_CrvGetArraySizes( tcur3[0], &ii, &kk );
    N_CrvGetArraySizes( tcur3[1], &jj, &kk );

    kk = NL_MAX( ii, jj );

    if( kk LT 10 )
        ii = 8;

    else if( kk LT 20 )
        ii = 17;

    else
        ii = 28;

    vsam = NL_MAX( q + 4, ii );

    pnts = N_AllocPt2dArray( usam, vsam, &SL );

    if( pnts EQ NULL )
        NL_QUIT;

    sfn2 = N_AllocCrvFuncArray( usam, 2, usam + 3, 1, &SL );

    if( sfn2 EQ NULL )
        NL_QUIT;

    f1 = N_AllocReal1dArray( NL_MAX( usam, vsam ), &SL );

    if( f1 EQ NULL )
        NL_QUIT;

    s = N_AllocReal1dArray( usam, &SL );

    if( s EQ NULL )
        NL_QUIT;

    ds = 1.0 / usam;

    s[0] = f1[0] = 0.0;
    s[usam] = f1[usam] = 1.0;

    for ( ii = 1; ii < usam; ii++ )
        s[ii] = ii * ds;

    for ( ii = 0; ii <= 1; ii++ )
    {
        if( ii EQ 0 )
        {
            d1 = d2 = 0.0;

            for ( jj = 0; jj <= usam; jj++ )
            {
                error = N_CrvEval( scur3[0], s[jj], NL_LEFT, &pnts[jj][0] );

                if( error EQ NL_YES )
                    NL_OUT;

                if( jj GT 0 )
                {
                    N_DistPtPt( pnts[jj][0], pnts[jj - 1][0], &d3 );
                    d1 += d3;
                }

                error = N_CrvEval( scur3[1], s[jj], NL_LEFT, &pnts[jj][vsam] );

                if( error EQ NL_YES )
                    NL_OUT;

                if( jj GT 0 )
                {
                    N_DistPtPt( pnts[jj][vsam], pnts[jj - 1][vsam], &d3 );
                    d2 += d3;
                }
            }

            if( d1 GT d2 )
                kk = 0;
            else
                kk = 1;
        }
        else
            kk = (kk + 1) % 2;

        Gcur = scur2[kk];

        for ( jj = 1; jj < usam; jj++ )
        {
            N_CopyPt( pnts[jj][kk * vsam], &Gpnt );

            xb = s[jj];
            fb = ST_GetDist( xb );

            xa = xc = xb;

            do
            {
                xa -= 0.02;

                if( xa LT 0.0 )
                    xa = 0.0;
                d3 = ST_GetDist( xa );
            } while ( xa GT 0.0 AND d3 LE fb );

            if( d3 LE fb )
                NL_ERROR( NL_NUM_ERR );

            do
            {
                xc += 0.02;

                if( xc GT 1.0 )
                    xc = 1.0;
                d3 = ST_GetDist( xc );
            } while ( xc LT 1.0 AND d3 LE fb );

            if( d3 LE fb )
                NL_ERROR( NL_NUM_ERR );

            N_FuncFindMinima( xa, xb, xc, fb, ST_GetDist, 6.e-8, NL_MTOL, &f1[jj], &d3 );
        }

        error = N_FitFuncInterpGivenParams( f1, usam, 2, s, sfn2[kk], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( NL_MIN( d1, d2 )LE NL_MTOL )
        { /* handle 0-length curves */
            jj = (kk + 1) % 2;

            error = N_CrvFuncCopy( sfn2[kk], sfn2[jj], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            break;
        }
    }

    tfn2 = N_AllocCrvFuncArray( vsam, 2, vsam + 3, 1, &SL );

    if( tfn2 EQ NULL )
        NL_QUIT;

    t = N_AllocReal1dArray( vsam, &SL );

    if( t EQ NULL )
        NL_QUIT;

    dt = 1.0 / vsam;

    t[0] = f1[0] = 0.0;
    t[vsam] = f1[vsam] = 1.0;

    for ( ii = 1; ii < vsam; ii++ )
        t[ii] = ii * dt;

    for ( ii = 0; ii <= 1; ii++ )
    {
        if( ii EQ 0 )
        {
            d1 = d2 = 0.0;

            for ( jj = 0; jj <= vsam; jj++ )
            {
                error = N_CrvEval( tcur3[0], t[jj], NL_LEFT, &pnts[0][jj] );

                if( error EQ NL_YES )
                    NL_OUT;

                if( jj GT 0 )
                    N_DistPtPt( pnts[0][jj], pnts[0][jj - 1], &d3 );
                d1 += d3;

                error = N_CrvEval( tcur3[1], t[jj], NL_LEFT, &pnts[usam][jj] );

                if( error EQ NL_YES )
                    NL_OUT;

                if( jj GT 0 )
                    N_DistPtPt( pnts[usam][jj], pnts[usam][jj - 1], &d3 );
                d2 += d3;
            }

            if( d1 GT d2 )
                kk = 0;
            else
                kk = 1;
        }
        else
            kk = (kk + 1) % 2;

        Gcur = tcur2[kk];

        for ( jj = 1; jj < vsam; jj++ )
        {
            N_CopyPt( pnts[kk * usam][jj], &Gpnt );

            xb = t[jj];
            fb = ST_GetDist( xb );

            xa = xc = xb;

            do
            {
                xa -= 0.02;

                if( xa LT 0.0 )
                    xa = 0.0;
                d3 = ST_GetDist( xa );
            } while ( xa GT 0.0 AND d3 LE fb );

            if( d3 LE fb )
                NL_ERROR( NL_NUM_ERR );

            do
            {
                xc += 0.02;

                if( xc GT 1.0 )
                    xc = 1.0;
                d3 = ST_GetDist( xc );
            } while ( xc LT 1.0 AND d3 LE fb );

            if( d3 LE fb )
                NL_ERROR( NL_NUM_ERR );

            N_FuncFindMinima( xa, xb, xc, fb, ST_GetDist, 6.e-8, NL_MTOL, &f1[jj], &d3 );
        }

        error = N_FitFuncInterpGivenParams( f1, vsam, 2, t, tfn2[kk], &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( NL_MIN( d1, d2 )LE NL_MTOL )
        { /* handle 0-length curves */
            jj = (kk + 1) % 2;

            error = N_CrvFuncCopy( tfn2[kk], tfn2[jj], &SL );

            if( error EQ NL_YES )
                NL_OUT;

            break;
        }
    }

    /* Load corner u,v points for local function */

    error = N_CrvEval( scur2[0], 0.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &corners[0][0][0], &corners[0][0][1], &d1 );
    error = N_CrvEval( tcur2[0], 0.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &u, &v, &d1 );
    corners[0][0][0] = 0.5 *( u + corners[0][0][0] );
    corners[0][0][1] = 0.5 *( v + corners[0][0][1] );

    error = N_CrvEval( scur2[0], 1.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &corners[1][0][0], &corners[1][0][1], &d1 );
    error = N_CrvEval( tcur2[1], 0.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &u, &v, &d1 );
    corners[1][0][0] = 0.5 *( u + corners[1][0][0] );
    corners[1][0][1] = 0.5 *( v + corners[1][0][1] );

    error = N_CrvEval( scur2[1], 0.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &corners[0][1][0], &corners[0][1][1], &d1 );
    error = N_CrvEval( tcur2[0], 1.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &u, &v, &d1 );
    corners[0][1][0] = 0.5 *( u + corners[0][1][0] );
    corners[0][1][1] = 0.5 *( v + corners[0][1][1] );

    error = N_CrvEval( scur2[1], 1.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &corners[1][1][0], &corners[1][1][1], &d1 );
    error = N_CrvEval( tcur2[1], 1.0, NL_LEFT, &uvpnt );

    if( error EQ NL_YES )
        NL_OUT;
    N_PtToXYZ( uvpnt, &u, &v, &d1 );
    corners[1][1][0] = 0.5 *( u + corners[1][1][0] );
    corners[1][1][1] = 0.5 *( v + corners[1][1][1] );

    for ( ii = 0; ii <= 1; ii++ )
        for ( jj = 0; jj <= 1; jj++ )
        {
            if( corners[ii][jj][0]LT us )
                corners[ii][jj][0] = us;

            if( corners[ii][jj][0]GT ue )
                corners[ii][jj][0] = ue;

            if( corners[ii][jj][1]LT vs )
                corners[ii][jj][1] = vs;

            if( corners[ii][jj][1]GT ve )
                corners[ii][jj][1] = ve;
        }

    /* Load remaining interior fit points */

    for ( ii = 1; ii < usam; ii++ )
        for ( jj = 1; jj < vsam; jj++ )
        {
            error = ST_GetPoint( scur2, tcur2, sfn2, tfn2, s[ii], t[jj], &uvpnt, corners, us, ue, vs, ve );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtToXYZ( uvpnt, &u, &v, &d1 );
            error = N_SrfEvalPt( sur, u, v, NL_LEFT, NL_LEFT, &pnts[ii][jj] );

            if( error EQ NL_YES )
                NL_OUT;
        }

    /* Make the 3D boundaries compatible, then do initial surface fit */

    N_CrvGetArraySizes( scur3[0], &ii, &kk );
    N_CrvGetArraySizes( scur3[1], &jj, &kk );
    kk = NL_MAX( ii, jj );
    nn = NL_MIN( kk, usam - 2 );

    error = N_CrvsMakeCompatible( scur3, 1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetArraySizes( tcur3[0], &ii, &kk );
    N_CrvGetArraySizes( tcur3[1], &jj, &kk );
    kk = NL_MAX( ii, jj );
    mm = NL_MIN( kk, vsam - 2 );

    error = N_CrvsMakeCompatible( tcur3, 1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfInitArrays( &sur1 );

    error = N_FitSrfToPtsAndBoundary( pnts, usam, vsam, s, t, scur3, tcur3, 1, nn, mm, &sur1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now use the initial surface to compute fit points for */
    /* final surface and for fit values for the parameter    */
    /* correspondence functions.                             */

    N_CrvGetArraySizes( scur3[0], &ns, &ii );
    N_CrvGetArraySizes( tcur3[0], &nt, &jj );
    kk = NL_MAX( ii, jj );

    st = N_AllocReal1dArray( 7 * (kk + 1) + 60, &SL );

    if( st EQ NULL )
        NL_QUIT;
    s = &st[kk + 1];
    t = &s[3 * (kk + 1) + 30];

    N_CrvGetKnotVector( scur3[0], &knt );
    N_BasisFindIndexNodeArray( knt, p, st );

    s[0] = 0.0;

    for ( ms = 0, ii = 1; ii <= ns; ii++ )
    {
        s[++ms] = 0.5 *( st[ii - 1] + st[ii] );
        s[++ms] = st[ii];
    }

    N_CrvGetKnotVector( tcur3[0], &knt );
    N_BasisFindIndexNodeArray( knt, q, st );

    t[0] = 0.0;

    for ( mt = 0, ii = 1; ii <= nt; ii++ )
    {
        t[++mt] = 0.5 *( st[ii - 1] + st[ii] );
        t[++mt] = st[ii];
    }

    /* Loop and compute points at (s,t) locations */

    pnts = N_AllocPt2dArray( ms, mt, &SL );

    if( pnts EQ NULL )
        NL_QUIT;

    Fu = N_AllocReal2dArray( ms, mt, &SL );
    Fv = N_AllocReal2dArray( ms, mt, &SL );

    if( Fu EQ NULL OR Fv EQ NULL )
        NL_QUIT;

    Fu[0][0] = corners[0][0][0];
    Fv[0][0] = corners[0][0][1];
    Fu[0][mt] = corners[0][1][0];
    Fv[0][mt] = corners[0][1][1];
    Fu[ms][0] = corners[1][0][0];
    Fv[ms][0] = corners[1][0][1];
    Fu[ms][mt] = corners[1][1][0];
    Fv[ms][mt] = corners[1][1][1];

    error = N_CrvEval( scur3[0], 0.0, NL_LEFT, &pnts[0][0] );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( scur3[0], 1.0, NL_LEFT, &pnts[ms][0] );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( tcur3[0], 1.0, NL_LEFT, &pnts[0][mt] );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_CrvEval( tcur3[1], 1.0, NL_LEFT, &pnts[ms][mt] );

    if( error EQ NL_YES )
        NL_OUT;

    top = NL_MTOL;
    toc = 1.e-5;

    N_SrfGetParameterBounds( sur, &d1, &d2, &d3, &d4 );
    d1 = sqrt( (d2 - d1) * (d2 - d1) + (d4 - d3) * (d4 - d3) );
    uvtop = d1 * 1.e-7;

    SD = N_AllocPt2dArray( 2, 2, &SL );

    if( SD EQ NULL )
        NL_OUT;

    for ( ii = 0; ii <= ms; ii++ )
        for ( jj = 0; jj <= mt; jj++ )
        {
            if( ii EQ 0 AND jj EQ 0 )
                continue;

            if( ii EQ ms AND jj EQ 0 )
                continue;

            if( ii EQ 0 AND jj EQ mt )
                continue;

            if( ii EQ ms AND jj EQ mt )
                continue;

            error = N_SrfEvalPt( &sur1, s[ii], t[jj], NL_LEFT, NL_LEFT, &Q );

            if( error EQ NL_YES )
                NL_OUT;

            error = ST_GetPoint( scur2, tcur2, sfn2, tfn2, s[ii], t[jj], &uvpnt, corners, us, ue, vs, ve );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtToXYZ( uvpnt, &u, &v, &d1 );

            error = N_GetClosestPtOnSrf( sur, Q, u, v, top, toc, &u, &v, &R, SD );

            if( error EQ NL_YES )
                NL_OUT;

            if( ii GT 0 AND ii LT ms AND jj GT 0 AND jj LT mt )
            {
                N_CopyPt( R, &Q );
            }
            else
            {
                N_PtFromXYZ( u, v, 0.0, &uvpnt );

                if( ii EQ 0 )
                {
                    error = N_CFuncEval( tfn2[0], t[jj], NL_LEFT, &d1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_CrvClosestPt( tcur2[0], uvpnt, d1, uvtop, toc, &d1, &uvpnt );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( ii EQ ms )
                {
                    error = N_CFuncEval( tfn2[1], t[jj], NL_LEFT, &d1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_CrvClosestPt( tcur2[1], uvpnt, d1, uvtop, toc, &d1, &uvpnt );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( jj EQ 0 )
                {
                    error = N_CFuncEval( sfn2[0], s[ii], NL_LEFT, &d1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_CrvClosestPt( scur2[0], uvpnt, d1, uvtop, toc, &d1, &uvpnt );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( jj EQ mt )
                {
                    error = N_CFuncEval( sfn2[1], s[ii], NL_LEFT, &d1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_CrvClosestPt( scur2[1], uvpnt, d1, uvtop, toc, &d1, &uvpnt );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                N_PtToXYZ( uvpnt, &u, &v, &d1 );
            }

            if( u LT us )
                u = us;

            if( u GT ue )
                u = ue;

            if( v LT vs )
                v = vs;

            if( v GT ve )
                v = ve;

            Fu[ii][jj] = u;
            Fv[ii][jj] = v;
            N_CopyPt( Q, &pnts[ii][jj] );
        }

    /* Now do the fits */

    error = N_FitSrfToPtsAndBoundary( pnts, ms, mt, s, t, scur3, tcur3, 2, 0, 0, subsur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( ufn NEQ NULL AND vgn NEQ NULL )
    {
        if( ms GT 70 )                                /* don't need this many */
        {
            for ( kk = 0, ii = 2; ii <= ms; ii += 2 ) /* ms is an even number */
            {
                s[++kk] = s[ii];

                for ( jj = 0; jj <= mt; jj++ )
                    Fu[kk][jj] = Fu[ii][jj];
            }

            ms = kk;
        }

        if( mt GT 70 )                                /* don't need this many */
        {
            for ( kk = 0, ii = 2; ii <= mt; ii += 2 ) /* mt is an even number */
            {
                t[++kk] = t[ii];

                for ( jj = 0; jj <= ms; jj++ )
                    Fv[jj][kk] = Fv[jj][ii];
            }

            mt = kk;
        }

        error = N_FitSrfFuncInterp( Fu, ms, mt, 2, 2, s, t, ufn, SG );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_FitSrfFuncInterp( Fv, ms, mt, 2, 2, s, t, vgn, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#if NLIB_UNUSED

/**********************************************************************/
/* N_ApproxNormalSrfWithSrf: Approximate unit normal surface of Nlib surface          */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This approximation routine approximates the UNIT normal surface of
     a given Nlib surface by a nonrational surface of selected degrees. 
     The  method  is  global, i.e. the  approximating  surface does not 
     deviate  from the  unit normal  surface  more  than the  tolerance 
     anywhere on the surface. A typical calling example is:

       NL_SURFACE  surP, surN;
       NL_DEGREE   pp, qq;
       NL_REAL     tol;
       NL_STACKS   SG;
       ...
       (define surP, get tol, pp and qq);
       ...
       N_SrfInitArrays(&surN);
       N_ApproxNormalSrfWithSrf(&surP,tol,pp,qq,NL_INHERITED,&surN,&SG);

     MEMORY TO STORE THE  OUTPUT NL_SURFACE  surN  IS ALLOCATED INSIDE THE
     ROUTINE! IT IS  ASSUMED THAT surP IS AT LEAST NL_G1 CONTINUOUS AND IS
     A VALID Nlib NL_SURFACE! 


   ACCESS:
   
     surP  , input  ,  NURBS surface
     tol   , input  ,  Error tolerance
     pp,qq , input  ,  Degrees of approximating surface.
     par   , input  ,  Flag:
                         NL_UNIFORM    : uniform parametrization
                         NL_CHORDLENGTH: chordlength parametrization
                         NL_CENTRIPETAL: centipetal parametrization
                         NL_INHERITED  : surP's parametrization 
     surN  , output ,  Approximation of the  UNIT normal  surface, i.e.
                       surN(u,v) agrees with the unit normal of surP at 
                       (u,v) up to tol
     SG    , input  ,  surN's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxNormalSrfWithSrf( NL_SURFACE *surP, NL_REAL tol, NL_DEGREE pp, NL_DEGREE qq, NL_FLAG par, NL_SURFACE *surN, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxNormalSrfWithSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX ** np, *nu, *nv, i, j, k, l, n, m, rp, sp, rq, sq, mu, mv;

    NL_DEGREE p, q, pq;

    NL_REAL *UP, *VP, *UQ, *VQ, *u, *v, Nuu, Nuv, Nvv, ul, ur, vl, vr, uinc, vinc, f1, f2, del, num, gro, exp;

    NL_POINT ** P;

    NL_VECTOR ** N;

    NL_KNOTVECTOR *knu, *knv;

    NL_RECTANGLE R;

    NL_SURFACE *** bez;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetKnots( surP, &rp, &sp, &UP, &VP );
    N_SrfGetDegrees( surP, &p, &q );

    /* Initialize */

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

    /* Get number of sampling points per Bezier patch */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            N_SrfReparamToInterval( bez[i][j], NL_UNITSQUARE, NL_UVDIR );

            error = N_SrfGetMaxSecondDeriv( bez[i][j], &Nuu, &Nuv, &Nvv );

            if( error EQ NL_YES )
                NL_OUT;

            num = 0.125 *( Nuu + Nvv ) + 0.25 *Nuv;
            gro = NL_MAX( f2, sqrt( num ) );
            del = f1 * gro;
            del = ceil( del );
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

    /* Get knot vectors */

    knu = N_AllocKnotVectorAndArray( mu + pp + 1, &SL );

    if( knu EQ NULL )
        NL_QUIT;

    knv = N_AllocKnotVectorAndArray( mv + qq + 1, &SL );

    if( knv EQ NULL )
        NL_QUIT;

    if( par NEQ NL_INHERITED )
    {
        error = N_FitCalcSrfParamValues( (NL_VOID ** )N, mu, mv, NL_EPOINT, par, u, v );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_FitCrvCalcKnotVector( u, mu, pp, knu );
    N_FitCrvCalcKnotVector( v, mv, qq, knv );

    /* Interpolate normals */

    N_SrfInitArrays( surN );
    error = N_FitSrfToPtsKnots( (NL_VOID ** )N, mu, mv, NL_EPOINT, u, v, knu, knv, pp, qq, surN, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Remove knots */

    error = N_SrfRemoveAllKnots( surN, tol, NL_UVDIR, surN, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compact output surface */

    N_SrfGetArraySizes( surN, &n, &m, &i, &i );

    if( n LT mu OR m LT mv )
    {
        error = N_SrfCompress( surN, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Rescale knot vector if necessary */

    if( par NEQ NL_INHERITED )
    {
        N_SrfGetKnots( surN, &rq, &sq, &UQ, &VQ );

        if( UP[0]NEQ UQ[0]OR UP[rp]NEQ UQ[rq]OR VP[0]NEQ VQ[0]OR VP[sp]NEQ VQ[sq] )
        {
            N_CreateRectangle( &R, UP[0], UP[rp], VP[0], VP[sp] );

            if( UP[0]EQ UQ[0]AND UP[rp]EQ UQ[rq] )
                N_SrfReparamToInterval( surN, R, NL_VDIR );

            else if( VP[0]EQ VQ[0]AND VP[sp]EQ VQ[sq] )
                N_SrfReparamToInterval( surN, R, NL_UDIR );

            else
                N_SrfReparamToInterval( surN, R, NL_UVDIR );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_ApproxSphereWithSrf: Approximation of a NURBS spherical patch                 */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes a  NON-RATIONAL  approximation  of a  NURBS 
     spherical patch. It first approximates the profile circle, and then 
     approximately  revolves  it  around  the  z-axis. The  degrees, the  
     continuities, and the error of the approximation can be controlled. 
     If the output surface is initialized to NULL, memory to  store  new 
     control points and knots is allocated. A typical calling example:

       NL_POINT    C;
       NL_DEGREE   p, q;
       NL_INDEX    cu, cv;
       NL_REAL     r, as, ae, al, tol;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get C, r, as, ae, al, p, q, cu, cv and tol);
       ...
       N_SrfInitArrays(&sur);
       N_ApproxSphereWithSrf(C,r,as,ae,al,p,q,cu,cv,tol,&sur,&S);

     If memory is  available, sur  is  not  initialized and the  routine
     assumes that  memory allocation has  been done. However, it  checks  
     for the proper  amount by looking at  the highest  indexes in sur's 
     knot vector and polygon objects.


   ACCESS:
   
     C     , input  ,  Center of sphere
     r     , input  ,  Radius of sphere
     as,ae , input  ,  Start and end angles of circular arc  sweep. MUST 
                       SATISFY 0 <= as < ae <= 180. as=0  corresponds to 
                       the point C+r*Z , and ae=180 to the  point C-r*Z.
     al    , input  ,  Angle of revolution from the Y axis (al > 0). 
     p,q   , input  ,  Degrees of the u- and v-directional circles:
                         p: degree in the direction of revolution
                         q: degree of the profile cicle
     cu,cv , input  ,  Continuities  of the u- and  v-circles;  
                       cu less than p and cv less than q must hold!
     tol   , input  ,  Error  tolerance; sur  does not  deviate anywhere 
                       from the precise sphere more than tol!
     sur   , output ,  NURBS sphere/spherical patch
     SG    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxSphereWithSrf( NL_POINT C, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL al, NL_DEGREE p, NL_DEGREE q, NL_INDEX cu, NL_INDEX cv, NL_REAL tol, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxSphereWithSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX nc;

    NL_REAL epu, epv, ang, fac;

    NL_CURVE curP;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check angles */

    if( as GE ae )
        NL_ERROR( NL_CAL_ERR );

    if( as LT 0.0 OR ae GT 180.0 )
        NL_ERROR( NL_CAL_ERR );

    ang = ae - as;

    if( ang LE 0.0 )
        NL_ERROR( NL_INP_ERR );

    fac = ang / (ang + al);
    epu = (1.0 - fac) * tol;
    epv = fac * tol;

    /* Get profile circle */

    nc = NL_MAX( 2, p - 1 );

    N_CrvInitArrays( &curP );
    error = N_ApproxArcWithCrv( C, NL_UNITZ, NL_UNITY, r, as, ae, q, cv, nc, epv, NL_ABSOLUTE, &curP, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Revolve circle/arc to get sphere/patch */

    error = N_ApproxRevolvedSrfWithSrf( &curP, C, NL_UNITZ, al, p, cu, epu, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* N_ApproxTorusWithSrf: Approximation of a NURBS toroidal patch                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes a  NON-RATIONAL  approximation  of a  NURBS 
     toroidal  patch. It first approximates the profile circle, and then 
     approximately  revolves it  around a  given axis. The  degrees, the  
     continuities, and the error of the approximation can be controlled. 
     If the output surface is initialized to NULL, memory to  store  new 
     control points and knots is allocated. A typical calling example:

       NL_POINT    S, C;
       NL_VECTOR   T;
       NL_DEGREE   p, q;
       NL_INDEX    cu, cv;
       NL_REAL     r, as, ae, al, tol;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get S, T, C, r, as, ae, al, p, q, cu, cv and tol);
       ...
       N_SrfInitArrays(&sur);
       N_ApproxTorusWithSrf(S,T,C,r,as,ae,al,p,q,cu,cv,tol,&sur,&S);

     If memory is  available, sur  is  not  initialized and the  routine
     assumes that  memory allocation has  been done. However, it  checks  
     for the proper  amount by looking at  the highest  indexes in sur's 
     knot vector and polygon objects.


   ACCESS:
   
     S,T   , input  ,  Point and direction of axis of revolution
     C     , input  ,  Center of circle  (or arc) to be  revolved  about 
                       the axis. Circle is in a  local coordinate system
                       (X,T), where  X points  from the  axis to the arc 
                       center, C.
     r     , input  ,  Radius of circle.
     as,ae , input  ,  Start and end angles of circular arc (measured in 
                       (X,T)). 0 <= as < ae <= 360 must hold
     al    , input  ,  Angle of revolution clockwise about the axis.
                       0 < al <= 360 must hold
     p,q   , input  ,  Degrees of the u- and v-directional circles:
                         p: degree in the direction of revolution
                         q: degree of the profile cicle
     cu,cv , input  ,  Continuities  of the u- and  v-circles;  
                       cu less than p and cv less than q must hold!
     tol   , input  ,  Error  tolerance; sur  does not  deviate anywhere 
                       from the precise torus more than tol!
     sur   , output ,  NURBS toroidal patch
     SG    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ApproxTorusWithSrf( NL_POINT S, NL_VECTOR T, NL_POINT C, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL al, NL_DEGREE p, NL_DEGREE q, NL_INDEX cu, NL_INDEX cv, NL_REAL tol, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_ApproxTorusWithSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX nc;

    NL_REAL d, t, epu, epv;

    NL_POINT P;

    NL_VECTOR X, B;

    NL_CURVE curP;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check angles */

    if( as GE ae )
        NL_ERROR( NL_CAL_ERR );

    if( as LT 0.0 OR ae GT 360.0 )
        NL_ERROR( NL_CAL_ERR );

    if( al LE 0.0 OR al GT 360.0 )
        NL_ERROR( NL_CAL_ERR );

    /* Get profile circle */

    N_VectorSum( S, T, &B );

    error = N_ProjectPtLineParam( C, S, B, &P, &t );

    if( error EQ NL_YES )
        NL_OUT;

    N_DistPtPt( C, P, &d );

    if( d LT NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    N_VectorDiff( C, P, &X );

    epu = 0.5 *tol;
    epv = 0.5 *tol;

    nc = NL_MAX( 2, p - 1 );

    N_CrvInitArrays( &curP );
    error = N_ApproxArcWithCrv( C, X, T, r, as, ae, q, cv, nc, epv, NL_ABSOLUTE, &curP, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Revolve circle/arc to get torus/patch */

    error = N_ApproxRevolvedSrfWithSrf( &curP, S, T, al, p, cu, epu, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif // NLIB_UNUSED

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

NL_REAL ST_GetDist( NL_REAL par )
{
    NL_FLAG error = NL_NO;

    NL_POINT P;

    NL_REAL u, v, z, dd;

    error = N_CrvEval( Gcur, par, NL_LEFT, &P );

    if( error EQ NL_YES )
        return (-1.0);

    N_PtToXYZ( P, &u, &v, &z );

    if( u LT G_us )
        u = G_us;

    if( u GT G_ue )
        u = G_ue;

    if( v LT G_vs )
        v = G_vs;

    if( v GT G_ve )
        v = G_ve;

    error = N_SrfEvalPt( Gsur, u, v, NL_LEFT, NL_LEFT, &P );

    if( error EQ NL_YES )
        return (-1.0);

    N_DistPtPt( P, Gpnt, &dd );

    return (dd);
}

/* --------------------------------------------------------------- */
/* --------------------------------------------------------------- */

NL_FLAG ST_GetPoint( NL_CURVE ** scur2, NL_CURVE ** tcur2, NL_CFUN ** sfn2, NL_CFUN ** tfn2, NL_REAL s, NL_REAL t, NL_POINT *uvP,
    NL_REAL corners[2][2][2], NL_REAL us, NL_REAL ue, NL_REAL vs, NL_REAL ve )
{
    NL_FLAG error = NL_NO;

    NL_POINT PP;

    NL_INDEX ij;

    NL_REAL uv[4][2], d4, s1, t1, s2, t2, ss, tt, temp[2];

    error = N_CFuncEval( sfn2[0], s, NL_LEFT, &s1 );

    if( error EQ NL_YES )
        return (error);
    error = N_CrvEval( scur2[0], s1, NL_LEFT, &PP );

    if( error EQ NL_YES )
        return (error);
    N_PtToXYZ( PP, &uv[0][0], &uv[0][1], &d4 );

    error = N_CFuncEval( sfn2[1], s, NL_LEFT, &s2 );

    if( error EQ NL_YES )
        return (error);
    error = N_CrvEval( scur2[1], s2, NL_LEFT, &PP );

    if( error EQ NL_YES )
        return (error);
    N_PtToXYZ( PP, &uv[1][0], &uv[1][1], &d4 );

    error = N_CFuncEval( tfn2[0], t, NL_LEFT, &t1 );

    if( error EQ NL_YES )
        return (error);
    error = N_CrvEval( tcur2[0], t1, NL_LEFT, &PP );

    if( error EQ NL_YES )
        return (error);
    N_PtToXYZ( PP, &uv[2][0], &uv[2][1], &d4 );

    error = N_CFuncEval( tfn2[1], t, NL_LEFT, &t2 );

    if( error EQ NL_YES )
        return (error);
    error = N_CrvEval( tcur2[1], t2, NL_LEFT, &PP );

    if( error EQ NL_YES )
        return (error);
    N_PtToXYZ( PP, &uv[3][0], &uv[3][1], &d4 );

    ss = 0.5 *( s1 + s2 );
    tt = 0.5 *( t1 + t2 );

    s1 = 1.0 - ss;
    t1 = 1.0 - tt;

    /* Boolean sum */

    for ( ij = 0; ij <= 1; ij++ )
    {
        temp[ij] = s1 * uv[2][ij] + ss * uv[3][ij] + t1 * uv[0][ij] + tt * uv[1][ij] - s1 * t1 * corners[0][0][ij] - ss * t1 * corners[1][0][ij] - s1 * tt * corners[0][1][ij] - ss * tt * corners[1][1][ij];
    }

    if( temp[0]LT us )
        temp[0] = us;

    if( temp[0]GT ue )
        temp[0] = ue;

    if( temp[1]LT vs )
        temp[1] = vs;

    if( temp[1]GT ve )
        temp[1] = ve;

    N_PtFromXYZ( temp[0], temp[1], 0.0, uvP );

    return (NL_NO);
}
