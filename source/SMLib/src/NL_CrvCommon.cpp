// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/********************************************************************************/
/* CrvCommon.c : Common Curve Function Definitions that act on NL_CURVE objects */
/********************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/*******************************************************************//**

    DESCRIPTION: See N_CreateQuadraticArcTol with added real at=0.01 

***********************************************************************/
NL_FLAG N_CreateQuadraticArc( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_CURVE *cur, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CreateQuadraticArc"); */

    NL_REAL at = 0.01;

    NL_FLAG error = NL_NO;

    error = N_CreateQuadraticArcTol( C, X, Y, r, as, ae, at, cur, SG );

    return (error);
} /* end N_CreateQuadraticArc */


/*******************************************************************//**


   DESCRIPTION:

     This conic routine  creates a quadratic  NURBS circle or circular 
     arc. If the output curve is initialized to NULL, memory to  store 
     new  control  points  and  knots is  allocated. A typical calling  
     example is:

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r, as, ae, at;
       NL_STACKS  SG;
       ...
       (get C, X, Y, r, as and ae);
       ...
       N_CrvInitArrays(&cur);
       N_CreateQuadraticArc(C,X,Y,r,as,ae,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.


   ACCESS:
   
     C,X,Y  , input  ,  Center and orthogonal axes of circle
     r      , input  ,  Radius of circle
     as,ae  , input  ,  Start and end angles in degrees
     at     , input  ,  Angle tolerance for snapping ends in degrees
     cur    , output ,  NURBS circle/circular arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

***********************************************************************/

NL_FLAG N_CreateQuadraticArcTol( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL at, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateQuadraticArcTol");

    NL_FLAG ints, error = NL_NO;

    NL_INDEX i, j, n, m, narcs;

    NL_DEGREE p;

    NL_REAL *U, theta, dtheta, angle, dangle, alf, bet, cosa, sina, w1;

    NL_REAL AngleTolDeg = at;

    NL_POINT P0, P1, P2;

    NL_VECTOR T0, T2;

    NL_LINESEG l0, l2;

    NL_CPOINT *Pw;

    /* Get number of arcs */

    n = m = p = 0;
    N_CrvGetConicData( NL_QUADRATIC, as, &ae, &n, &p, &m );

    narcs = n / 2;
    theta = ae - as;
    dtheta = theta / narcs;

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, n, 2, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get start point */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    angle = (NL_PI * dtheta) / 360.0;
    w1 = cos( angle );

    angle = (NL_PI * as) / 180.0;
    cosa = cos( angle );
    sina = sin( angle );
    alf = r * cosa;
    bet = r * sina;

    N_TranslateSum2Pts( C, alf, X, bet, Y, &P0 );
    N_Combine2Pts( -sina, X, cosa, Y, &T0 );
    N_Weight( P0, 1.0, &Pw[0] );

    /* variable initializede in SMLib version - not used in this version
    N_CopyCPt(P0, PStart);
    */

    /* Compute segments */

    dangle = (NL_PI * dtheta) / 180.0;
    j = 0;

    for ( i = 1; i <= narcs; i++ )
    {
        angle = angle + dangle;
        cosa = cos( angle );
        sina = sin( angle );
        alf = r * cosa;
        bet = r * sina;

        N_TranslateSum2Pts( C, alf, X, bet, Y, &P2 );
        N_Combine2Pts( -sina, X, cosa, Y, &T2 );

        N_CreateLineStartDirVector( &l0, P0, T0, NL_UNBOUNDED );
        N_CreateLineStartDirVector( &l2, P2, T2, NL_UNBOUNDED );

        error = N_IsectLineLine( l0, l2, &P1, &alf, &bet, &ints );

        /* if lines fail to intersect */
        if( ints EQ NL_FALSE )
        {
            /* RMB mod to allow for very flat segments */
            NL_REAL dist;
            N_DistPtPt( P0, P2, &dist );

            /* even if very small, better a point-curve than a null */
            /* if (dist LE NL_MTOL) NL_ERROR(NL_INP_ERR); */

            /* this segment is very flat, use mid point for P1 */
            N_Combine2Pts( 0.5, P0, 0.5, P2, &P1 );
        }

        else if( error )
            NL_ERROR( NL_INP_ERR );

        N_Weight( P1, w1, &Pw[j + 1] );
        N_Weight( P2, 1.0, &Pw[j + 2] );

#if 0
        /* check made in SMLib version not made here */
        /* If have complete circle match end points */
        /* if (i == narcs && angle > 2.0*NL_PI - 1.0e-8 && theta > 359.9) */

        if( i == narcs && theta > 359.9 )
        {
            N_Weight( PStart, 1.0, &Pw[j + 2] );
        }

#endif

        if( i LT narcs )
        {
            N_CopyPt( P2, &P0 );
            N_CopyPt( T2, &T0 );
        }

        j += 2;
    }

    /* Ensure closure if theta = 360 degrees */
    if( theta GT 360.0 - AngleTolDeg )
        N_CopyCPt( Pw[0], &Pw[n] );

    /* Get knot vectors */

    for ( i = 0; i <= 2; i++ )
        U[i] = 0.0;

    switch( narcs )
    {
        case 2:
            U[3] = 0.5;
            U[4] = 0.5;
            break;

        case 3:
            U[3] = 1.0 / 3.0;
            U[4] = 1.0 / 3.0;
            U[5] = 2.0 / 3.0;
            U[6] = 2.0 / 3.0;
            break;

        case 4:
            U[3] = 0.25;
            U[4] = 0.25;
            U[5] = 0.50;
            U[6] = 0.50;
            U[7] = 0.75;
            U[8] = 0.75;
            break;
    }

    for ( i = 0; i <= 2; i++ )
        U[n + i + 1] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CreateQuadraticArcTol */


/*******************************************************************//**

    DESCRIPTION: See N_CreateQuarticArcTol with added real at=0.01 

***********************************************************************/
NL_FLAG N_CreateQuarticArc( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_CURVE *cur, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CreateQuarticArc"); */

    NL_REAL at = 0.01;

    NL_FLAG error = NL_NO;

    error = N_CreateQuarticArcTol( C, X, Y, r, as, ae, at, cur, SG );

    return (error);
} /* end N_CreateQuarticArc*/

/*******************************************************************//**


   DESCRIPTION:

     This conic routine  creates a quartic NURBS circle or circular arc.
     If the output curve is initialized to NULL,  memory  to  store  new 
     control  points  and  knots is allocated. A typical calling example
     is:

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r, as, ae, at;
       NL_STACKS  SG;
       ...
       (get C, X, Y, r, as and ae);
       ...
       N_CrvInitArrays(&cur);
       N_CreateQuarticArc(C,X,Y,r,as,ae,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.


   ACCESS:
   
     C,X,Y  , input  ,  Center and orthogonal axes of circle
     r      , input  ,  Radius of circle
     as,ae  , input  ,  Start and end angles in degrees
     at     , input  ,  Angle tolerance in degrees
     cur    , output ,  NURBS circle/circular arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateQuarticArcTol( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL at, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateQuarticArcTol");

    NL_FLAG ints, error = NL_NO;

    NL_INDEX i, j, n, m, narcs;

    NL_DEGREE p;

    NL_REAL *U, theta, dtheta, angle, dangle, alf, bet, disc, wf, Fw, cosa, sina, w1, x2, a, b, c, d, ang, cs, u1 = 0.0, u2 = 0.0, v1, v2;

    NL_REAL AngleTolDeg = at;

    NL_POINT P0, P1, P2;

    NL_VECTOR T0, T2;

    NL_LINESEG l0, l2;

    NL_CPOINT *Pw, Qw;

    /* Get number of arcs */

    n = m = p = 0;
    N_CrvGetConicData( NL_QUARTIC, as, &ae, &n, &p, &m );

    narcs = n / 4;
    theta = ae - as;
    dtheta = theta / narcs;

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, n, 4, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get start point */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    dtheta = theta / narcs;
    angle = (NL_PI * dtheta) / 180.0;
    x2 = cos( angle );
    angle = 0.5 *angle;
    w1 = cos( angle );

    angle = (NL_PI * as) / 180.0;
    cosa = cos( angle );
    sina = sin( angle );
    alf = r * cosa;
    bet = r * sina;

    N_TranslateSum2Pts( C, alf, X, bet, Y, &P0 );
    N_Combine2Pts( -sina, X, cosa, Y, &T0 );
    N_Weight( P0, 1.0, &Pw[0] );

    /* Compute quadratic segments and reparameterize them to quartics */

    /* First determine the 2 parameters of the Bezier quadratic   */
    /* circle (u in [0,1]) which map to the points at 1/4 and 3/4 */
    /* of the sweep dtheta. The two parameters are u1 and u2.     */

    for ( i = 0; i < 2; i++ )
    {
        if( i EQ 0 )
            ang = (0.25 *NL_PI * dtheta) / 180.0;
        else
            ang = (0.75 *NL_PI * dtheta) / 180.0;
        cs = cos( ang );

        a = 1.0 - 2.0 *w1 + x2 - 2.0 *cs * (1.0 - w1); /* quadratic equation */
        b = 2.0 *( w1 - 1.0 ) + 2.0 *cs * (1.0 - w1);
        c = 1.0 - cs;

        disc = b * b - 4.0 *a * c;

        if( disc LT 0.0 )
            NL_ERROR( NL_NUM_ERR );
        disc = sqrt( disc ); /* solve quadratic equation */
        v1 = (-b + disc) / (2.0 *a);
        v2 = (-b - disc) / (2.0 *a);

        if( v1 GE 0.0 AND v1 LE 1.0 )
            a = v1;

        else if( v2 GE 0.0 AND v2 LE 1.0 )
            a = v2;

        else
            NL_ERROR( NL_NUM_ERR );

        if( i EQ 0 )
            u1 = a;
        else
            u2 = a;
    }

    /* Use u1 and u2 to get the rational quadratic Bezier */
    /* reparameterization function. Must solve 2x2 system */
    /* of linear equations to get middle control point    */
    /* (1D) and middle weight.                            */

    a = u1 - u2;
    wf = -(8.0 + 10.0 *a) / (6.0 *a);
    Fw = (u1 * (10.0 + 6.0 *wf) - 1.0) / 6.0;

    d = (2.0 *Fw * Fw) / 3.0;
    c = (1.0 - 6.0 *Fw + 4.0 *wf * Fw - 4.0 *Fw * Fw) / 3.0;
    b = (-6.0 *wf + 2.0 *wf * wf - 4.0 *wf * Fw + 6.0 *Fw + 2.0 *Fw * Fw) / 3.0;
    a = wf - Fw;

    /* Now compute all the quadratic segments and reparameterize them. */
    /* This means we compute the three quadratic control points, and   */
    /* then we compute the three middle control points of the quartic. */

    dangle = (NL_PI * dtheta) / 180.0;
    j = 0;

    for ( i = 1; i <= narcs; i++ )
    {
        angle = angle + dangle;
        cosa = cos( angle );
        sina = sin( angle );
        alf = r * cosa;
        bet = r * sina;

        N_TranslateSum2Pts( C, alf, X, bet, Y, &P2 );
        N_Combine2Pts( -sina, X, cosa, Y, &T2 );

        N_CreateLineStartDirVector( &l0, P0, T0, NL_UNBOUNDED );
        N_CreateLineStartDirVector( &l2, P2, T2, NL_UNBOUNDED );

        error = N_IsectLineLine( l0, l2, &P1, &alf, &bet, &ints );

        if( error EQ NL_YES )
            NL_OUT;

        if( ints EQ NL_FALSE )
            NL_ERROR( NL_INP_ERR );

        N_Weight( P1, w1, &Qw );
        N_Weight( P2, 1.0, &Pw[j + 4] );

        /* The quadratic segment is Pw[j], Qw, and Pw[j+4] */

        N_Combine2CPts( a, Pw[j], Fw, Qw, &Pw[j + 1] );
        N_Combine4CPts( b, Pw[j], c, Qw, d, Pw[j + 4], 2.0, Pw[j + 1], &Pw[j + 2] );
        N_Combine2CPts( a, Qw, Fw, Pw[j + 4], &Pw[j + 3] );

        if( i LT narcs )
        {
            N_CopyPt( P2, &P0 );
            N_CopyPt( T2, &T0 );
        }

        j += 4;
    }

    /* Ensure closure if theta = 360 degrees */

    if( theta GT 360.0 - AngleTolDeg )
        N_CopyCPt( Pw[0], &Pw[n] );

    /* Get knot vectors */

    for ( i = 0; i <= 4; i++ )
        U[i] = 0.0;

    switch( narcs )
    {
        case 2:
            for ( i = 5; i <= 8; i++ )
                U[i] = 0.5;
            break;

        case 3:
            for ( i = 5; i <= 8; i++ )
                U[i] = 1.0 / 3.0;

            for ( i = 9; i <= 12; i++ )
                U[i] = 2.0 / 3.0;
            break;
    }

    for ( i = 0; i <= 4; i++ )
        U[n + i + 1] = 1.0;

    /* End NURBS */

    EXIT:

    return (error);
} /* end N_CreateQuarticArcTol */


/*******************************************************************//**

    DESCRIPTION: See N_CreateQuinticArcTol with added real at=0.01 

***********************************************************************/
NL_FLAG N_CreateQuinticArc( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_CURVE *cur, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CreateQuinticArc"); */

    NL_REAL at = 0.01;

    NL_FLAG error = NL_NO;

    error = N_CreateQuinticArcTol( C, X, Y, r, as, ae, at, cur, SG );

    return (error);
} /* end N_CreateQuinticArc*/


/*******************************************************************//**


   DESCRIPTION:

     This conic routine creates a quintic NURBS circle or circular arc
     with  no interior  knots used, i.e. the  arc is a  quintic Bezier 
     curve. If  the output  curve is  initialized  to NULL, memory  to 
     store  new  control  points  and  knots is  allocated. A  typical 
     calling example is:

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r, as, ae, at;
       NL_STACKS  SG;
       ...
       (get C, X, Y, r, as and ae);
       ...
       N_CrvInitArrays(&cur);
       N_CreateQuinticArc(C,X,Y,r,as,ae,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.


   ACCESS:
   
     C,X,Y  , input  ,  Center and orthogonal axes of circle
     r      , input  ,  Radius of circle
     as,ae  , input  ,  Start and end angles in degrees
     at     , input  ,  Angle tolerance in degrees
     cur    , output ,  Quintic NURBS circle/circular arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateQuinticArcTol( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL at, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateQuinticArcTol");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *U, angle, dangle, alf, bet, cosa, sina, am;

    NL_REAL AngleTolDeg = at;

    NL_PARAMETER us, ue, u0;

    NL_POINT Ps, Pe, Pm, A;

    NL_VECTOR T, S;

    NL_CPOINT *Pw;

    /* Get number of arcs */

    n = m = p = 0;
    N_CrvGetConicData( NL_QUINTIC, as, &ae, &n, &p, &m );

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, n, p, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get start, end and mid points */

    am = 0.5 *( as + ae );
    am += 180.0; /* ?? */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    angle = (NL_PI * as) / 180.0;
    alf = r * cos( angle );
    bet = r * sin( angle );

    N_TranslateSum2Pts( C, alf, X, bet, Y, &Ps );

    angle = (NL_PI * ae) / 180.0;
    alf = r * cos( angle );
    bet = r * sin( angle );

    N_TranslateSum2Pts( C, alf, X, bet, Y, &Pe );

    angle = (NL_PI * am) / 180.0;
    cosa = cos( angle );
    sina = sin( angle );
    alf = r * cosa;
    bet = r * sina;

    N_TranslateSum2Pts( C, alf, X, bet, Y, &Pm );
    N_Combine2Pts( -sina, X, cosa, Y, &T );
    N_Combine2Pts( -cosa, X, -sina, Y, &S );

    error = N_VectorNormalizeRef( &T );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get quintic full circle */

    N_Weight( Pm, 5.0, &Pw[0] );
    N_Weight( Pm, 5.0, &Pw[5] );

    N_Combine2Pts( 1.0, Pm, 4.0 *r, T, &A );
    N_Weight( A, 1.0, &Pw[1] );

    N_Combine2Pts( 1.0, Pm, -4.0 *r, T, &A );
    N_Weight( A, 1.0, &Pw[4] );

    N_TranslateSum2Pts( C, 3.0 *r, S, 2.0 *r, T, &A );
    N_Weight( A, 1.0, &Pw[2] );

    N_TranslateSum2Pts( C, 3.0 *r, S, -2.0 *r, T, &A );
    N_Weight( A, 1.0, &Pw[3] );

    for ( i = 0; i <= 5; i++ )
    {
        U[i] = 0.0;
        U[i + 6] = 1.0;
    }

    /* Extract curve segment */

    if( (ae - as)GT 360.0 - AngleTolDeg )
    {
        N_CopyCPt( Pw[0], &Pw[5] );
        NL_OUT;
    }

    dangle = as - am;
    dangle = 360.0 + dangle;

    /* in NMTversion last line is
    if (dangle LT 0.0)  dangle = 360.0+dangle;
    */

    u0 = dangle / 360.0;
    error = N_CrvClosestPt( cur, Ps, u0, NL_MTOL, NL_MTOL, &us, &A );

    if( error EQ NL_YES )
        NL_OUT;

    dangle = ae - am;

    if( dangle LT 0.0 )
        dangle = 360.0 + dangle;

    /* in NMTversion last line is
    if (dangle LT 0.0)  dangle = 360.0+dangle;
    */

    u0 = dangle / 360.0;
    error = N_CrvClosestPt( cur, Pe, u0, NL_MTOL, NL_MTOL, &ue, &A );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvExtractCrvSeg( cur, us, ue, cur, SG, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* N_CrvReparamToInterval(cur,NL_UNITSPAN); */

    /* in SMLib version last line is not called */

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CreateQuinticArcTol */


/*******************************************************************//**

    DESCRIPTION: See N_CreateCircArcTol with added real at=0.01 

***********************************************************************/
NL_FLAG N_CreateCircArc
 ( NL_POINT    C,     /* in : circle center                                                 */
   NL_VECTOR   X,     /* in : circle X axis                                                 */
   NL_VECTOR   Y,     /* in : circle Y axis (circle normal plane Normal = cross(X,Y)        */
   NL_REAL     r,     /* in : circle radius                                                 */
   NL_REAL     as,    /* in : StartAngle degrees                                            */
   NL_REAL     ae,    /* in : EndAngle degrees                                              */
   NL_FLAG     ctp,   /* in : NL_QUADRATIC = deg 2 circle with double internal knots        */
                      /*      NL_QUARTIC   = deg 4 circle with quadruple internal knots     */
                      /*                     and better parameterization than NL_QUADRATIC  */
                      /*      NL_QUINTIC   = deg 5 circle no internal knots                 */
   NL_CURVE  * cur,   /* out: The circle curve                                              */
   NL_STACKS * SG )   /* in : new object stack                                              */
{                     
    /* NL_PRIVATE NL_STRING rname = _T("N_CreateCircArc"); */

    NL_REAL at = 0.01;

    NL_FLAG error = NL_NO;

    error = N_CreateCircArcTol( C, X, Y, r, as, ae, at, ctp, cur, SG );

    return (error);
} /* end N_CreateCircArc */



/*******************************************************************//**


   DESCRIPTION:

     This conic routine creates a NURBS circle or circular arc. If the 
     output curve is initialized to NULL, memory to store new  control  
     points and knots is allocated. In this version NL_QUADRATIC, NL_QUARTIC
     or NL_QUINTIC circles are allowed. A typical calling example is: 

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r, as, ae, at;
       NL_STACKS  SG;
       ...
       (get C, X, Y, r, as and ae);
       ...
       N_CrvInitArrays(&cur);
       N_CreateCircArc(C,X,Y,r,as,ae,NL_QUINTIC,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.


   ACCESS:
   
     C,X,Y  , input  ,  Center and orthogonal axes of circle
     r      , input  ,  Radius of circle
     as,ae  , input  ,  Start and end angles in degrees
     at     , input  ,  Angle Tolerance in degrees
     ctp    , input  ,  Flag:
                          NL_QUADRATIC: degree 2  circle  is created  with
                                     double internal knots
                          NL_QUARTIC  : degree 4 circle is created, possi-
                                     ibly with quadruple internal knots
                                     (but better parameterization  than
                                     NL_QUADRATIC)
                          NL_QUINTIC  : degree 5  circle  is created  with 
                                     no internal knots
     cur    , output ,  NURBS circle/circular arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CreateCircArcTol
 ( NL_POINT    C,     /* in : circle center                                                 */
   NL_VECTOR   X,     /* in : circle X axis                                                 */
   NL_VECTOR   Y,     /* in : circle Y axis (circle normal plane Normal = cross(X,Y)        */
   NL_REAL     r,     /* in : circle radius                                                 */
   NL_REAL     as,    /* in : StartAngle degrees                                            */
   NL_REAL     ae,    /* in : EndAngle degrees                                              */
   NL_REAL     at,    /* in : Angle Tolerance in degrees                                    */
   NL_FLAG     ctp,   /* in : NL_QUADRATIC = deg 2 circle with double internal knots        */
                      /*      NL_QUARTIC   = deg 4 circle with quadruple internal knots     */
                      /*                     and better parameterization than NL_QUADRATIC  */
                      /*      NL_QUINTIC   = deg 5 circle no internal knots                 */
   NL_CURVE  * cur,   /* out: The circle curve                                              */
   NL_STACKS * SG )   /* in : new object stack                                              */
{                     
    NL_PRIVATE NL_STRING rname = _T("N_CreateCircArcTol");

    NL_FLAG error = NL_NO;

    switch( ctp )
    {
        case NL_QUADRATIC:

            error = N_CreateQuadraticArcTol( C, X, Y, r, as, ae, at, cur, SG );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_QUARTIC:

            error = N_CreateQuarticArcTol( C, X, Y, r, as, ae, at, cur, SG );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_QUINTIC:

            error = N_CreateQuinticArcTol( C, X, Y, r, as, ae, at, cur, SG );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateCircArcTol */


/*******************************************************************//**


   DESCRIPTION:

     This conic routine computes circle weights for a particular sweep
     angle. NL_QUADRATIC  and  NL_QUINTIC  circles are considered. A typical 
     calling example is:

       NL_INDEX  k;
       NL_REAL   as, ae, w[9];
       ...
       (get as and ae);
       ...
       N_CalcCircWeights(as,ae,NL_QUADRATIC,w,&k);

     This  routine  is  used   mainly  for  constructing  surfaces  of 
     revolution or in routines where knowledge of circle weights is of
     importance.


   ACCESS:
   
     as,ae  , input  ,  Start and end angles
     ctp    , input  ,  Flag:
                          NL_QUADRATIC: degree 2 circle is used
                          NL_QUINTIC  : degree 5 circle is used
     w      , output ,  Circle/circular arc weights
     k      , output ,  Highest index in w array


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CalcCircWeights( NL_REAL as, NL_REAL ae, NL_FLAG ctp, NL_REAL *w, NL_INDEX *k )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, n;

    NL_CPOINT *Aw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Create unit circle */

    N_CrvInitArrays( &curA );

    error = N_CreateCircArc( NL_ZERO, NL_UNITX, NL_UNITY, 1.0, as, ae, ctp, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Extract weights */

    N_CrvGetCPts( &curA, &n, &Aw );

    for ( i = 0; i <= n; i++ )
    {
        N_CPtGetW( Aw[i], &w[i] );
    }

    *k = n;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CalcCircWeights */


/*******************************************************************//**

    DESCRIPTION: See N_CreateEllipticalArcAngTypeTol 
        with added flag atp=0 and real at = 0.01.

***********************************************************************/
NL_FLAG N_CreateEllipticalArc( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r1, NL_REAL r2, NL_REAL as, NL_REAL ae, NL_FLAG ctp, NL_CURVE *cur, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CreateEllipticalArcAngType"); */
    NL_FLAG error, atp;
    NL_REAL at = 0.01;
    atp = 0;
    error = N_CreateEllipticalArcAngTypeTol( C, X, Y, r1, r2, as, ae, at, atp, ctp, cur, SG );
    return error;
} /* end N_CreateEllipticalArc */



/*******************************************************************//**

    DESCRIPTION: See N_CreateEllipticalArcAngTypeTol with added flag atp=0.
        If tolerance at <= 0.0, tolerance passed is 0.01.

***********************************************************************/
NL_FLAG N_CreateEllipticalArcTol( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r1, NL_REAL r2, NL_REAL as, NL_REAL ae, NL_REAL at, NL_FLAG ctp, NL_CURVE *cur, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CreateEllipticalArcAngType"); */
    NL_FLAG error, atp;
    NL_REAL aTol;

    if ( at <= 0.0 )
        aTol = 0.01;
    else
        aTol = at;

    atp = 0;
    error = N_CreateEllipticalArcAngTypeTol( C, X, Y, r1, r2, as, ae, aTol, atp, ctp, cur, SG );

    return error;
} /* end N_CreateEllipticalArcTol */


/*******************************************************************//**

    DESCRIPTION: See N_CreateEllipticalArcAngTypeTol with added real at=0.01 

***********************************************************************/

// ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_CreateEllipticalArcAngType( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r1, NL_REAL r2, NL_REAL as, NL_REAL ae, NL_FLAG atp, NL_FLAG ctp, NL_CURVE *cur, NL_STACKS *SG )
{
    /* NL_PRIVATE NL_STRING rname = _T( "N_CreateEllipticalArcAngType" ); */

    NL_FLAG error = NL_NO;

    NL_REAL at = 0.01;

    error = N_CreateEllipticalArcAngTypeTol( C, X, Y, r1, r2, as, ae, at, atp, ctp, cur, SG );

    return ( error );
} /* end N_CreateEllipticalArcAngType */


/*******************************************************************//**


   DESCRIPTION:

     This conic routine creates a NURBS ellipse or elliptical  arc. If 
     the output  curve is  initialized  to NULL, memory to  store  new 
     control points and knots is allocated. A typical calling  example
     is as follows:

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r1, r2, as, ae, at;
       NL_FLAG    atp
       NL_STACKS  SG;
       ...
       (get C, X, Y, r1, r2, as and ae);
       ...
       N_CrvInitArrays(&cur);
       N_CreateEllipticalArcAngType(C, X, Y, r1, r2, as, ae, atp, NL_QUADRATIC, &cur, &SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.
     Note: The start and end angles can be measured from the center to
     the point on the circle of the major axis (atp = 0), or to the point
     on the ellipse (atp = 1). If you drop a perpendicular from the point
     on the circle, it will intercept a point on the ellipse, but the
     two angles are not the same.


   ACCESS:
   
     C, X, Y , input  ,  Center and orthogonal axes of ellipse
     r1, r2  , input  ,  Major and minor radii
     as, ae  , input  ,  Start and end angles. |as - ae| <= 360                         
     at      , input  ,  Angle tolerance in degrees
     atp     , input  ,  angletype : 
                           0 - Angle from point on circle(based on C, r1)
                           1 - Angle from point on ellipse
     ctp    , input  ,  Flag:
                          NL_QUADRATIC: degree 2  ellipse is created  with
                                     double internal knots
                          NL_QUARTIC  : degree 4 ellipse is created, poss-
                                     ibly with quadruple internal knots
                                     (but better parameterization  than
                                     NL_QUADRATIC)
                          NL_QUINTIC  : degree 5 ellipse  is created  with 
                                     no internal knots
     cur    , output ,  NURBS ellipse/elliptical arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateEllipticalArcAngTypeTol( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r1, NL_REAL r2, NL_REAL as, NL_REAL ae, NL_REAL at, NL_FLAG atp, NL_FLAG ctp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateEllipticalArcAngTypeTol");

    NL_FLAG prj, error = NL_NO;

    NL_INDEX i, n;

    NL_REAL angle, alf, bet, t, fac, omf, ang[2], apnt[2], ap, w, tmp_angle, e;

    NL_POINT P, Q;

    NL_LINESEG lx;

    NL_CPOINT *Pw;

    /* Normalize vectors */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreateLineStartDirVector( &lx, C, X, NL_UNBOUNDED );

    fac = r1 / r2;
    omf = 1.0 - fac;

    /* Get start and end points */
    while( ae LE as )
        ae = 360.0 + ae;

    while( (ae - as)GE 360.1 )
        ae = ae - 360.0;

    if( (ae - as)GT 360.0 )
        ae = as + 360.0;

    ang[0] = as;
    ang[1] = ae;

    for ( i = 0; i < 2; i++ ) /* start, end angle */
    {
        angle = (NL_PI * ang[i]) / 180.0;

        if( atp == 0 )
        { /* point on circle at angle - projected to ellipse */
            alf = r1 * cos( angle );
            bet = r2 * sin( angle ); /* bet = r2/r1 * r1*sin( angle ); Point on circle projected to ellipse */
        }
        else
        { /* point on ellipse at angle
          The equation of the ellipse:   x*x/a*a + y*y/b*b = 1 
          for a point on the ellipse   tan(angle) = y/x and let e = tan(angle)
          then  x*x = a*a*b*b/(a*a*e*e + b*b)                   a = r1
                  x = a*b/sqrt(a*a*e*e + b*b) and               b = r2
                a*a*y*y = a*a*b*b - b*b*x*x                     x = alf
                  y*y   = (a*a - x*x)*b*b/a*a                   y = bet
                    y   = b/a * sqrt(a*a - x*x)
          using this, solve for x=alf and y=bet */
            e = tan( angle );
            alf = r1 * r2 / sqrt( r2 * r2 + r1 * r1 * e * e );

            if( cos( angle )LT 0.0 )
                alf = -alf;
            bet = r1 * r1 - alf * alf; /*Temp use of bet*/
            if ( bet < 1.0e-12)
                bet = 0.0;
            else
                bet = (r2 / r1) * sqrt( bet);

            if( sin( angle )LT 0.0 )
                bet = -bet;
        }

        /* getting polar angle of vector */
        tmp_angle = atan2(bet / r2, alf / r1);

        ap = (180.0 *tmp_angle) / NL_PI;

        /* angle snapping up to NL_PTOL degrees */
        if ( fabs(ap) LE NL_PTOL )  
          ap = 0.0;

        if( ap LT 0.0 )
            ap = 360.0 + ap;
        apnt[i] = ap;
    }

    /* Compute circle/circular arc */
    error = N_CreateCircArcTol( C, X, Y, r1, apnt[0], apnt[1], at, ctp, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPts( cur, &n, &Pw );

    /* Apply affine transformation to get ellipse control points */
    fac = r2 / r1;
    omf = 1.0 - fac;

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P );
        N_CPtGetW( Pw[i], &w );

        error = N_ProjectPtLine( lx, P, &Q, &t, &prj );

        if( error EQ NL_YES )
            NL_OUT;

        if( prj EQ NL_FALSE )
            NL_ERROR( NL_INP_ERR );

        N_Combine2Pts( fac, P, omf, Q, &P );
        N_Weight( P, w, &Pw[i] );
    }

    /* End NURBS and Exit */
    EXIT:

    return (error);
} /* end N_CreateEllipticalArcAngTypeTol */



/*******************************************************************//**


   DESCRIPTION:

     This conic routine creates a NURBS  conic arc. The arc is defined 
     by  its end  points, end tangents  and one arbitrary point on the 
     arc. If the output curve is initialized to  NULL, memory to store 
     new  control  points  and knots is  allocated. A  typical calling  
     example is:

       NL_POINT   P0, P2, P;
       NL_VECTOR  T0, T2;
       NL_CURVE   cur;
       NL_STACKS  SG;
       ...
       (get P0, T0, P2, T2 and input P);
       ...
       N_CrvInitArrays(&cur);
       N_CreateConicArc(P0,T0,P2,T2,P,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.
     If there is an error, such as tangents failing to intersect, return
     a conic between start,end points with mid point between them  
     and w = 1.0


   ACCESS:
   
     P0,T0  , input  ,  Start point and tangent 
     P2,T2  , input  ,  End point and tangent 
     P      , input  ,  Given point on the arc
     cur    , output ,  NURBS conic arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_CreateConicArc( NL_POINT P0, NL_VECTOR T0, NL_POINT P2, NL_VECTOR T2, NL_POINT P, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateConicArc");

    NL_FLAG one_arc = NL_NO, two_arcs = NL_NO, four_arcs = NL_NO, error = NL_NO;

    NL_INDEX i, n = 0, m;

    NL_REAL *U, wp, wq, wr, wq1, wq2, wr1, wr2, dot, m1, m2, cosa;

    NL_POINT PP[3], Q[3], R[3], Q1[3], Q2[3], R1[3], R2[3], P1;

    NL_CPOINT *Pw;

    /* Create one arc first */

    error = N_BezConicArcFromPtsAndTangents( P0, T0, P2, T2, P, &P1, &wp );

    if( error EQ NL_YES ) /* unable to intersect tangents etc so make a conic line */
    {
      /* do not do this:    NL_OUT; but return a conic 'line' */
      P1.x = (P0.x + P2.x)/2.0; P1.y = (P0.y + P2.y)/2.0; P1.z = (P0.z + P2.z)/2.0;
      wp = 1.0;
      error = NL_NO;
    }

    if( wp LE - 1.0 )
        NL_ERROR( NL_WEI_ERR );

    /* Classify by the number of arcs */

    if( wp GE 1.0 )
    {
        one_arc = NL_YES;
    }
    else
    {
        N_VectorDot( T0, T2, &dot );
        N_VectorMagnitude( T0, &m1 );
        N_VectorMagnitude( T2, &m2 );
        cosa = -dot / (m1 * m2);

        if( wp GE 0.0 AND cosa LE 0.5 )
            one_arc = NL_YES;

        else if( wp EQ 0.0 AND cosa GT( 1.0 - NL_PTOL ) )
            two_arcs = NL_YES;

        else if( wp GE 0.0 AND cosa GT 0.5 )
            two_arcs = NL_YES;

        else if( wp LT 0.0 AND cosa GE 0.0 )
            two_arcs = NL_YES;

        else if( wp LT 0.0 AND cosa LT 0.0 )
            four_arcs = NL_YES;
    }

    if( one_arc )
        n = 2;

    else if( two_arcs )
        n = 4;

    else if( four_arcs )
        n = 8;

    m = n + 3;

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, n, 2, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get end points and end knots */
    N_Weight( P0, 1.0, &Pw[0] );
    N_Weight( P2, 1.0, &Pw[n] );

    for ( i = 0; i <= 2; i++ )
    {
        U[i] = 0.0;
        U[n + i + 1] = 1.0;
    }

    /* Compute conic arcs: one arc */
    if( one_arc EQ NL_YES )
    {
        N_Weight( P1, wp, &Pw[1] );

        NL_OUT;
    }

    /* Two arcs */
    N_CopyPt( P0, &PP[0] );
    N_CopyPt( P1, &PP[1] );
    N_CopyPt( P2, &PP[2] );

    N_BezConicSplit( PP, wp, Q, &wq, R, &wr );

    if( two_arcs EQ NL_YES )
    {
        N_Weight( Q[1], wq, &Pw[1] );
        N_Weight( Q[2], 1.0, &Pw[2] );
        N_Weight( R[1], wr, &Pw[3] );

        U[3] = 0.5;
        U[4] = 0.5;

        NL_OUT;
    }

    /* Four arcs */

    if( four_arcs EQ NL_YES )
    {
        N_BezConicSplit( Q, wq, Q1, &wq1, Q2, &wq2 );
        N_BezConicSplit( R, wr, R1, &wr1, R2, &wr2 );

        N_Weight( Q1[1], wq1, &Pw[1] );
        N_Weight( Q1[2], 1.0, &Pw[2] );
        N_Weight( Q2[1], wq2, &Pw[3] );
        N_Weight( Q2[2], 1.0, &Pw[4] );
        N_Weight( R1[1], wr1, &Pw[5] );
        N_Weight( R1[2], 1.0, &Pw[6] );
        N_Weight( R2[1], wr2, &Pw[7] );

        U[3] = 0.25;
        U[4] = 0.25;
        U[5] = 0.50;
        U[6] = 0.50;
        U[7] = 0.75;
        U[8] = 0.75;

        NL_OUT;
    }

    /* Exit */
    EXIT:

    return (error);
} /* end N_CreateConicArc */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This conic routine computes the conic shape invariance factor of
     a NURBS conic, k = (w0*w2)/(w1*w1), where w0, w1, and w2 are the
     weights from the 1st 3 control points. For non-rational curves
     k = 1.0.  A typical calling example is:
    
       NL_CURVE  cur;
       NL_REAL   k;
       ...
       (define cur);
       ...
       N_CalcConicShapeFactor(&cur,&k);

     IT IS ASSUMED THAT cur REPRESENTS ONE SEGMENT OF A CONIC. 


   ACCESS:
   
     cur  , input  ,  NURBS conic arc
     k    , output ,  Shape invariance factor
                      k > 1.0 + tol := NL_ELLIPSE
                      k < 1.0 - tol := NL_HYPERBOLA
                      else          := NL_PARABOLA
                      see N_ConicGetType

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CalcConicShapeFactor( NL_CURVE *cur, NL_REAL *k )
{
    NL_PRIVATE NL_STRING rname = _T("N_CalcConicShapeFactor");

    NL_FLAG error = NL_NO;

    NL_REAL *dum, w0, w1, w2;

    NL_CPOINT *Pw;

    /* Check curve weights */

    error = N_CrvWeightsAreValid( cur, rname );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &dum );

    /* Get conic shape factor */

    *k = 1.0;

    if( N_IsCrvRat( cur ) )
    {
        N_CPtGetW( Pw[0], &w0 );
        N_CPtGetW( Pw[1], &w1 );
        N_CPtGetW( Pw[2], &w2 );

        *k = (w0 * w2) / (w1 * w1);
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CalcConicShapeFactor */

/*******************************************************************//**


   DESCRIPTION:

     This conic  routine  computes  the implicit  equation of a  conic 
     curve. The implicit form is given as

       c[0]*x*x + c[1]*y*y + c[2]*x*y + c[3]*x + c[4]*y + c[5] = 0

     A typical calling example is:

       NL_CURVE  cur;
       NL_REAL   c[6];
       ...
       (define cur);
       ...
       N_CalcConicImplicitEq(&cur,c);

     II IS ASSUMED THAT  MEMORY FOR THE  COEFFICIENTS IS  ALLOCATED IN 
     THE  CALLING  ROUTINE. IT  IS ALSO  ASSUMED THAT  cur IS A  CONIC 
     NL_CURVE LYING IN THE X-Y NL_PLANE. IT HAS  ONLY ONE  SEGMENT AND IS OF 
     NL_DEGREE TWO.


   ACCESS:
   
     cur , input  ,  NURBS conic arc
     c   , output ,  Coefficients  of implicit  equation (must hold up 
                     to c[5])


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CalcConicImplicitEq( NL_CURVE *cur, NL_REAL *c )
{
    NL_PRIVATE NL_STRING rname = _T("N_CalcConicImplicitEq");

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL k, h0, h1, h2, h3, g1, s1, s2, x[3], y[3], z;

    NL_POINT P[3];

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute some constants */

    error = N_CalcConicShapeFactor( cur, &k );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPts( cur, &i, &Pw );

    for ( i = 0; i <= 2; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &P[i] );
        N_PtToXYZ( P[i], &x[i], &y[i], &z );

        if( z NEQ 0.0 AND z NEQ NL_NOZ )
            NL_ERROR( NL_INP_ERR );
    }

    h0 = x[0] - x[1];
    h1 = x[2] - x[1];
    h2 = y[0] - y[1];
    h3 = y[2] - y[1];
    g1 = h2 * h1 - h0 * h3;
    s1 = h0 - h1;
    s2 = h3 - h2;
    k = 1.0 / k;

    /* Get coefficients of implicit equation */

    c[0] = s2 * s2 + 4.0 *k * h2 * h3;
    c[1] = s1 * s1 + 4.0 *k * h0 * h1;
    c[2] = 2.0 *( s1 * s2 - 2.0 *k * (h0 * h3 + h1 * h2) );
    c[3] = 2.0 *( g1 * s2 - c[0] * x[1] - c[2] * y[1] );
    c[4] = 2.0 *( g1 * s1 - c[1] * y[1] - c[2] * x[1] );
    c[5] = c[0] * x[1] * x[1] + c[1] * y[1] * y[1] + 2.0 *c[2] * x[1] * y[1] - 2.0 *g1 * (s1 * y[1] + s2 * x[1]) + g1 * g1;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CalcConicImplicitEq */

/*******************************************************************//**


   DESCRIPTION:

     This  conic routine determines the type of a NURBS conic arc. The
     classification is accurate up to NL_PTOL specified in "globals.h". A 
     typical calling example is:

       NL_CURVE  cur;
       NL_FLAG   type;
       ...
       (define cur);
       ...
       N_ConicGetType(&cur,&type);

     IT IS ASSUMED THAT cur REPRESENTS ONE SEGMENT OF A CONIC. 


   ACCESS:
   
     cur   , input  ,  NURBS conic arc
     type  , output ,  Type of conic:
                         NL_ELLIPSE
                         NL_HYPERBOLA
                         NL_PARABOLA


   METHOD:
      get conic shape factor, k = (w0*w2)/(w1*w1)
      where w0, w1, and w2 are the weights of the first three control points
      and select type by k value as:

         k > 1.0 + tol := NL_ELLIPSE,
         k < 1.0 - tol := NL_HYPERBOLA,
         else          := NL_PARABOLA.

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ConicGetType( NL_CURVE *cur, NL_FLAG *type )
{
    NL_FLAG error = NL_NO;

    NL_REAL k;

    /* Get conic shape factor */

    error = N_CalcConicShapeFactor( cur, &k );

    if( error EQ NL_YES )
        NL_OUT;

    /* Classify conic */

    if( k GT( 1.0 + NL_PTOL ) )
        *type = NL_ELLIPSE;

    else if( k GE( 1.0 - NL_PTOL ) )
        *type = NL_PARABOLA;

    else
        *type = NL_HYPERBOLA;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_ConicGetType */

/*******************************************************************//**


   DESCRIPTION:

     This conic routine computes the geometric definition of a conic
     arc. The different conics are defined as follows:

       NL_PARABOLA  (P): vertex, axis and focus 
       NL_ELLIPSE   (E): center, major and minor axes, and radii
       NL_HYPERBOLA (H): center, major and minor axes, and radii
       
     A typical calling example is:

       NL_CURVE   cur;
       NL_POINT   C;
       NL_VECTOR  U, V;
       NL_REAL    r1, r2;
       ...
       (define cur);
       ...
       N_ConicCalcGeomDef(&cur,&C,&U,&V,&r1,&r2);

     It is assumed that the  conic is not degenerate, i.e. no special 
     cases are checked for.


   ACCESS:
   
     cur   , input  ,  NURBS conic arc
     C     , output ,  P  : vertex
                       E/H: center
     U     , output ,  P  : unit axis
                       E/H: unit major axis
     V     , output ,  P  : focus
                       E/H: unit minor axis
     r1,r2 , output ,  P  : undefined
                       E/H: axis radii


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ConicCalcGeomDef( NL_CURVE *cur, NL_POINT *C, NL_VECTOR *U, NL_VECTOR *V, NL_REAL *r1, NL_REAL *r2 )
{
    NL_PRIVATE NL_STRING rname = _T("N_ConicCalcGeomDef");

    NL_FLAG type, error = NL_NO;

    NL_REAL *dum, k, eps, alf, bet, gam, del, zet, eta, la1, la2, rho, a, b, c, xh, yh, x0, y0, dis;

    NL_POINT P[3], A;

    NL_VECTOR S, T;

    NL_CPOINT *Pw;

    /* Compute shape invariance, k = (w0*w2)/(w1*w1) */
    /* k > 1 = NL_ELLIPSE    */
    /* k < 1 = NL_HYPERBOLA  */
    /* k = 1 = NL_PARABOLA   */
    error = N_CalcConicShapeFactor( cur, &k );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get some constants */
    N_CrvGetCPtsAndKnots( cur, &Pw, &dum );

    N_CPtToPtEuclid( Pw[0], &P[0] ); /* P0, P1, P2 = 1st 3 control Points */
    N_CPtToPtEuclid( Pw[1], &P[1] );
    N_CPtToPtEuclid( Pw[2], &P[2] );

    N_VectorDiff( P[0], P[1], &S ); /* S = vector from P1 to P0 */
    N_VectorDiff( P[2], P[1], &T ); /* T = vector from P2 to p0 */
    N_VectorDot( S, S, &alf );      /* alf = S*S                */
    N_VectorDot( S, T, &bet );      /* bet = S*T                */
    N_VectorDot( T, T, &gam );      /* gam = T*T                */

    /* Get type of conic */
    error = N_ConicGetType( cur, &type );

    if( error EQ NL_YES )
        NL_OUT;

    /* Consider the parabola first */
    if( type EQ NL_PARABOLA )
    {
        zet = alf + gam + 2.0 *bet;
        a = 1.0 / sqrt( zet );
        N_Combine2Pts( a, S, a, T, U );

        error = N_VectorNormalizeRef( U );

        if( error EQ NL_YES )
            NL_OUT;

        a = (gam + bet) / zet;
        b = (alf + bet) / zet;
        N_TranslateSum2Pts( P[1], a * a, S, b * b, T, C );

        a = gam / zet;
        b = alf / zet;
        N_TranslateSum2Pts( P[1], a, S, b, T, V );

        NL_OUT;
    }

    /* Ellipse or hyperbola */
    eps = 0.5 *k / (k - 1.0); /* gwc: potential divide by zero here? */

    N_TranslateSum2Pts( P[1], eps, S, eps, T, C );

    eta = alf + gam - 2.0 *bet;
    del = alf * gam - bet * bet;

    a = 2.0 *del;
    b = -(k * eta + 4.0 *bet);
    c = 2.0 *( k - 1.0 );
    dis = fabs( b * b - 4.0 *a * c );
    la1 = (-b + sqrt( dis )) / (2.0 *a);
    la2 = (-b - sqrt( dis )) / (2.0 *a);

    if( la1 GT la2 )
        N_SwapReals( &la1, &la2 );

    if( N_FloatOpIsBad( eps, la1, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( eps / la1 LT 0.0 )
        NL_ERROR( NL_NUM_ERR );

    *r1 = sqrt( eps / la1 );

    if( type EQ NL_ELLIPSE )
    {
        *r2 = sqrt( eps / la2 );

        if( fabs( *r1 - *r2 )LT NL_MTOL ) /* circle */
        {
            N_VectorCopy( NL_UNITX, U );
            N_VectorCopy( NL_UNITY, V );
            NL_OUT;
        }
    }
    else
    {
        if( -eps / la2 LT 0.0 )
            NL_ERROR( NL_NUM_ERR );
        *r2 = sqrt( -eps / la2 );
    }

    if( (0.5 *k - gam *la1)GT( 0.5 *k - alf *la1 ) )
    {
        xh = 0.5 *k - gam * la1;
        yh = bet * la1 - 0.5 *k + 1.0;
    }
    else
    {
        xh = bet * la1 - 0.5 *k + 1.0;
        yh = 0.5 *k - alf * la1;
    }

    rho = alf * xh * xh + 2.0 *bet * xh * yh + gam * yh * yh;

    if( N_FloatOpIsBad( xh, rho, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( yh, rho, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    x0 = xh / rho;
    y0 = yh / rho;

    a = *r1 * x0;
    b = *r1 * y0;
    N_TranslateSum2Pts( *C, a, S, b, T, &A );
    N_VectorDiff( *C, A, U );
    N_VectorCross( T, S, &A );
    N_VectorCross( *U, A, V );

    error = N_VectorNormalizeRef( U );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( V );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_ConicCalcGeomDef */

/*******************************************************************//**


   DESCRIPTION:

     This conic routine creates a NURBS cubic semi-circle. The repre-
     sentation does not use internal knots, i.e. the curve is a cubic
     rational  Bezier circle. If the  output curve is  initialized to 
     NULL, memory to store new control points and knots is allocated. 
     A typical calling example is:

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r, as;
       NL_STACKS  SG;
       ...
       (get C, X, Y, r and as);
       ...
       N_CrvInitArrays(&cur);
       N_CreateCubicSemiCircle(C,X,Y,r,as,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.


   ACCESS:
   
     C,X,Y  , input  ,  Center and orthogonal axes of circle
     r      , input  ,  Radius of circle
     as     , input  ,  Start angle measured to the positive X
     cur    , output ,  NURBS cubic semi-circle
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCubicSemiCircle( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCubicSemiCircle");

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL *U, angle, alf, bet, cosa, sina, onethird;

    NL_POINT P0, P1, P2, P3;

    NL_VECTOR T;

    NL_CPOINT *Pw;

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, 3, 3, 7, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get start and end points, and start tangent */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    angle = (NL_PI * as) / 180.0;
    cosa = cos( angle );
    sina = sin( angle );
    alf = r * cosa;
    bet = r * sina;

    N_TranslateSum2Pts( C, alf, X, bet, Y, &P0 );
    N_Weight( P0, 1.0, &Pw[0] );
    N_Combine2Pts( 2.0, C, -1.0, P0, &P3 );
    N_Weight( P3, 1.0, &Pw[3] );

    N_Combine2Pts( -sina, X, cosa, Y, &T );

    /* Compute middle control points */

    error = N_VectorNormalizeRef( &T );

    if( error EQ NL_YES )
        NL_OUT;

    onethird = 1.0 / 3.0;
    N_Combine2Pts( 1.0, P0, 2.0 *r, T, &P1 );
    N_Combine2Pts( 1.0, P3, 2.0 *r, T, &P2 );

    N_Weight( P1, onethird, &Pw[1] );
    N_Weight( P2, onethird, &Pw[2] );

    /* Get knot vector */

    for ( i = 0; i <= 3; i++ )
    {
        U[i] = 0.0;
        U[i + 4] = 1.0;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateCubicSemiCircle */

/*******************************************************************//**


   DESCRIPTION:

     This conic routine creates a NURBS circular arc of  less  than  or
     equal to 180 degrees sweep angle.  No interior knots  are  used (a
     Bezier curve). Degree can be cubic or quartic. If the output curve
     is initialized to NULL,  memory to store new  control  points  and 
     knots is allocated. A typical calling example is: 

       NL_POINT   C;
       NL_VECTOR  X, Y;
       NL_REAL    r, as, ae;
       NL_STACKS  SG;
       ...
       (get C, X, Y, r, as and ae);
       ...
       N_CrvInitArrays(&cur);
       N_CreateBoundedCircArc(C,X,Y,r,as,ae,NL_QUARTIC,&cur,&SG);

     If memory is  available, cur is  not initialized and  the routine 
     assumes that memory allocation has been done. However,  it checks  
     for the proper amount by looking  at the highest indexes in cur's  
     knot vector and polygon objects.


   ACCESS:
   
     C,X,Y  , input  ,  Center and orthogonal axes of circle
     r      , input  ,  Radius of circle
     as,ae  , input  ,  Start and end angles (total sweep must  not  be
                        greater than 180 degrees)
     ctp    , input  ,  Flag:
                          NL_CUBIC   : degree 3 circle is created  with no
                                    internal knots
                          NL_QUARTIC : degree 4 circle is created  with no
                                    internal knots  (better parameteri-
                                    zation than NL_CUBIC)
     cur    , output ,  NURBS circular arc
     SG     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateBoundedCircArc( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL r, NL_REAL as, NL_REAL ae, NL_FLAG ctp, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateBoundedCircArc");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, n;

    NL_REAL *U, theta, si, co, dd, a, b, c, d, disc, u1 = 0.0, u2 = 0.0, v1, v2, w1, x2, wf, Fw;

    NL_CPOINT *Pw;

    NL_POINT P0, Pn;

    NL_VECTOR T0, Tn;

    /* adjust angles if necessary, and check for input error */

    if( ae LE as )
        ae = 360.0 + ae;
    theta = ae - as;

    if( theta GT 180.0 )
        NL_ERROR( NL_INP_ERR );

    /* Ensure axes have unit length */

    error = N_VectorNormalizeRef( &X );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &Y );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed */

    if( ctp EQ NL_CUBIC )
        n = 3;
    else
        n = 4;

    error = N_CrvSizeArrays( cur, n, (NL_DEGREE)n, 2 * n + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;
    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Load the first and last control points and get tangent vectors */

    as = as * (NL_PI / 180.0);
    si = sin( as );
    co = cos( as );

    N_TranslateSum2Pts( C, r * co, X, r * si, Y, &P0 );
    N_Weight( P0, 1.0, &Pw[0] );

    N_Combine2Pts( -si, X, co, Y, &T0 );

    ae = ae * (NL_PI / 180.0);
    si = sin( ae );
    co = cos( ae );

    N_TranslateSum2Pts( C, r * co, X, r * si, Y, &Pn );
    N_Weight( Pn, 1.0, &Pw[n] );

    N_Combine2Pts( -si, X, co, Y, &Tn );

    /* Now create circular arc (interior control points and knots) */

    if( ctp EQ NL_CUBIC )
    {
        theta = theta * (NL_PI / 360.0);
        si = sin( theta );
        co = cos( theta );

        dd = (2.0 *r * si) / (2.0 *co + 1.0);

        N_VectorPtAlongVector( P0, dd, T0, &P0 );
        N_VectorPtAlongVector( Pn, -dd, Tn, &Pn );

        dd = (2.0 *co + 1.0) / 3.0; /* weight */

        N_Weight( P0, dd, &Pw[1] );
        N_Weight( Pn, dd, &Pw[2] );

        /* load knots */

        for ( ii = 0; ii <= 3; ii++ )
        {
            U[ii] = 0.0;
            U[ii + 4] = 1.0;
        }
    }
    else
    {
        theta = theta * (NL_PI / 180.0);
        w1 = cos( 0.5 *theta );
        x2 = cos( theta );
        dd = r * sin( 0.5 *theta );

        a = w1 * P0.x + dd * T0.x;
        b = w1 * P0.y + dd * T0.y;
        c = w1 * P0.z + dd * T0.z;

        N_CPtFromWxWyWz( a, b, c, w1, &Pw[2] );

        /* Compute quadratic segment and reparameterize it to quartic */

        /* First determine the 2 parameters of the Bezier quadratic   */
        /* circle (u in [0,1]) which map to the points at 1/4 and 3/4 */
        /* of the sweep angle. The two parameters are u1 and u2.      */

        for ( ii = 0; ii < 2; ii++ )
        {
            if( ii EQ 0 )
                co = cos( 0.25 *theta );
            else
                co = cos( 0.75 *theta );

            a = 1.0 - 2.0 *w1 + x2 - 2.0 *co * (1.0 - w1); /* quadratic equation */
            b = 2.0 *( w1 - 1.0 ) + 2.0 *co * (1.0 - w1);
            c = 1.0 - co;

            disc = b * b - 4.0 *a * c;

            if( disc LT 0.0 )
                NL_ERROR( NL_NUM_ERR );
            disc = sqrt( disc ); /* solve quadratic equation */
            v1 = (-b + disc) / (2.0 *a);
            v2 = (-b - disc) / (2.0 *a);

            if( v1 GE 0.0 AND v1 LE 1.0 )
                a = v1;

            else if( v2 GE 0.0 AND v2 LE 1.0 )
                a = v2;

            else
                NL_ERROR( NL_NUM_ERR );

            if( ii EQ 0 )
                u1 = a;
            else
                u2 = a;
        }

        /* Use u1 and u2 to get the rational quadratic Bezier */
        /* reparameterization function. Must solve 2x2 system */
        /* of linear equations to get middle control point    */
        /* (1D) and middle weight.                            */

        a = u1 - u2;
        wf = -(8.0 + 10.0 *a) / (6.0 *a);
        Fw = (u1 * (10.0 + 6.0 *wf) - 1.0) / 6.0;

        d = (2.0 *Fw * Fw) / 3.0;
        c = (1.0 - 6.0 *Fw + 4.0 *wf * Fw - 4.0 *Fw * Fw) / 3.0;
        b = (-6.0 *wf + 2.0 *wf * wf - 4.0 *wf * Fw + 6.0 *Fw + 2.0 *Fw * Fw) / 3.0;
        a = wf - Fw;

        /* Now compute the quadratic segment and reparameterize it. This */
        /* means we compute the three quadratic control points, and then */
        /* we compute the three middle control points of the quartic.    */

        /* The quadratic segment is already in: Pw[0], Pw[2], and Pw[4]  */

        N_Combine2CPts( a, Pw[0], Fw, Pw[2], &Pw[1] );
        N_Combine2CPts( a, Pw[2], Fw, Pw[4], &Pw[3] );
        N_Combine4CPts( b, Pw[0], c, Pw[2], d, Pw[4], 2.0, Pw[1], &Pw[2] );

        /* load knots */

        for ( ii = 0; ii <= 4; ii++ )
        {
            U[ii] = 0.0;
            U[ii + 5] = 1.0;
        }
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateBoundedCircArc */
#endif // NLIB_UNUSED
