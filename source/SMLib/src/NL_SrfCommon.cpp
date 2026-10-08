// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************************/
/* SrfCommon.c : Common Surface Function Definitions that act on NL_SURFACE objects */
/************************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>


/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a bilinear surface given the
     four corner points. The surface is represented as a  non-rational
     NURBS  surface. If  the  output  surface is  initialized to NULL, 
     memory to store new  control  points  and knots  is  allocated. A 
     typical calling example is:

       NL_SURFACE  sur;
       NL_POINT    P00, P10, P01, P11;
       NL_STACKS   SG;
       ...
       (define corner points P00, P10, P01 and P11);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSrfCornerPts(P00,P10,P01,P11,&sur,&SG);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and  polygon objects. If  the four  points lie in the 
     same plane, sur represents a planar surface.


   ACCESS:
   
     P00,P10,P01,P11 , input  ,  Corner points
     sur             , output ,  Bilinear NURBS surface
     SG              , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSrfCornerPts( NL_POINT P00, NL_POINT P10, NL_POINT P01, NL_POINT P11, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSrfCornerPts");

    NL_FLAG error = NL_NO;

    NL_REAL *U, *V;

    NL_CPOINT ** Pw;

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, 1, 1, 1, 1, 3, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Pw, &U, &V );

    /* Compute control points */

    N_PtToCPt( P00, &Pw[0][0] );
    N_PtToCPt( P10, &Pw[1][0] );
    N_PtToCPt( P01, &Pw[0][1] );
    N_PtToCPt( P11, &Pw[1][1] );

    /* Get the knots */

    U[0] = 0.0;
    U[1] = 0.0;
    U[2] = 1.0;
    U[3] = 1.0;

    V[0] = 0.0;
    V[1] = 0.0;
    V[2] = 1.0;
    V[3] = 1.0;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CreateSrfCornerPts */

/*******************************************************************//**


   DESCRIPTION:

     This  common surfaces routine  creates a generalized  cylinder by 
     extruding an  arbitrary NURBS curve in a given  direction a given  
     distance. If the output surface is initialized to NULL, memory to 
     store  new  control  points  and  knots  is  allocated. A typical 
     calling example is:

       NL_SURFACE  sur;
       NL_CURVE    cur;
       NL_VECTOR   W;
       NL_REAL     d;
       NL_STACKS   SG;
       ...
       (define cur, and get W and d);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSrfExtrudeCrv(&cur,W,d,NL_UDIR,&sur,&SG);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector  and  polygon  objects. The  extrusion can be done in
     either u- or in v-direction.


   ACCESS:
   
     cur , input  ,  NURBS curve to be extruded
     W   , input  ,  Direction vector of extrusion
     d   , input  ,  Distance of extrusion
     dir , input  ,  Flag:
                       NL_UDIR: extruded surface is  linear in u-direction 
                       NL_VDIR: extruded surface is  linear in v-direction 
     sur , output ,  Extruded NURBS surface
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSrfExtrudeCrv( NL_CURVE *cur, NL_VECTOR W, NL_REAL d, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSrfExtrudeCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, ni, mi, ri, si;

    NL_DEGREE p, pi, qi;

    NL_REAL *U, *UQ, *VQ, w;

    NL_POINT A;

    NL_CPOINT *Pw, ** Qw;

    /* Get local notations */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    /* Get highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            ni = 1;
            mi = n;
            pi = 1;
            qi = p;
            ri = 3;
            si = m;
            break;

        case NL_VDIR:

            ni = n;
            mi = 1;
            pi = p;
            qi = 1;
            ri = m;
            si = 3;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, ni, mi, pi, qi, ri, si, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Prepare for extrusion */

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    error = N_VectorNormalizeRef( &W );

    if( error EQ NL_YES )
        NL_OUT;

    /* Extrude in u-direction */

    if( dir EQ NL_UDIR )
    {
        /* Compute control points */

        for ( j = 0; j <= n; j++ )
        {
            N_CopyCPt( Pw[j], &Qw[0][j] );
            N_CPtToPtAndW( Pw[j], &A, &w );
            N_VectorPtAlongVector( A, d, W, &A );

            if( N_IsCrvRat( cur ) )
                N_Weight( A, w, &Qw[1][j] );
            else
                N_PtToCPt( A, &Qw[1][j] );
        }

        /* Get the knots */

        for ( i = 0; i <= 1; i++ )
        {
            UQ[i] = 0.0;
            UQ[2 + i] = 1.0;
        }

        for ( j = 0; j <= m; j++ )
            VQ[j] = U[j];
    }

    /* Extrude in v-direction */

    if( dir EQ NL_VDIR )
    {
        /* Compute control points */

        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( Pw[i], &Qw[i][0] );
            N_CPtToPtAndW( Pw[i], &A, &w );
            N_VectorPtAlongVector( A, d, W, &A );

            if( N_IsCrvRat( cur ) )
                N_Weight( A, w, &Qw[i][1] );
            else
                N_PtToCPt( A, &Qw[i][1] );
        }

        /* Get the knots */

        for ( i = 0; i <= m; i++ )
            UQ[i] = U[i];

        for ( j = 0; j <= 1; j++ )
        {
            VQ[j] = 0.0;
            VQ[2 + j] = 1.0;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CreateSrfExtrudeCrv */



/*******************************************************************//**


   DESCRIPTION:

     This  common  surfaces  routine creates a  cylinder or a cone or a
     patch of them by extruding  a  circle/circular  arc. It allows the 
     user to choose  between a  quadratic circle  (with internal knots)
     and a  quintic  circle  (with  no  internal  knots). If the output 
     surface is initialized to NULL, memory to store new control points  
     and knots is allocated. A typical calling example is:

       NL_POINT    C;
       NL_VECTOR   X, Y;
       NL_REAL     rb, rt, as, ae, h;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get C, X, Y, rb, rt, as, ae, and h);
       ...
       N_SrfInitArrays(&sur);
       N_CreateCylCone(C,X,Y,rb,rt,as,ae,h,NL_QUINTIC,NL_UDIR,&sur,&SG);

     If  memory is  available, sur  is not  initialized and the routine
     assumes that  memory allocation has been done. However, it  checks  
     for the proper  amount by looking at the highest  indexes in sur's 
     knot vector  and  polygon  objects. The  extrusion  can be done in
     either u- or in v-direction.


   ACCESS:
   
     C,X,Y , input  ,  Center of  botton  circle  and  orthogonal  axes 
                       defining bottom plane
     rb,rt , input  ,  Bottom and top radii
     as,ae , input  ,  Start and end sweep angles of base circle
     h     , input  ,  Height of cylinder/cone
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: Use   quadratic   circle  (internal 
                                    knots)
                         NL_QUINTIC  : Use   quintic  circle  (no internal
                                    knots)
     dir   , input  ,  Flag:
                         NL_UDIR: base circle is a u-curve
                         NL_VDIR: base circle is a v-curve
     sur   , output ,  NURBS cylinder/cone surface/patch
     SG    , input  ,  sur's stack stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCylCone( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL rb, NL_REAL rt, NL_REAL as, NL_REAL ae, NL_REAL h, NL_FLAG ctp, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCylCone");
    NL_FLAG error = NL_NO;
    /*
    This good idea causes all sorts of problems
    error =  N_CreateEllipticalCylCone(C, X, Y, rb, rb, rt, rt,
    as, ae, h, ctp, dir, sur, SG);
    
      return error;
} */

    NL_INDEX i, j, n = 0, m = 0, ni, mi, ri, si;

    NL_DEGREE p = 0, pi, qi;

    NL_REAL *U = NULL, *UQ, *VQ, w;

    NL_POINT A;

    NL_VECTOR Z;

    NL_CPOINT *Bw = NULL, *Tw = NULL, ** Qw;

    NL_CURVE curB, curT;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get bottom and top circles */

    if( rb LE NL_MTOL AND rt LE NL_MTOL )
        NL_ERROR( NL_CAL_ERR );

    N_VectorCross( X, Y, &Z );

    error = N_VectorNormalizeRef( &Z );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorPtAlongVector( C, h, Z, &A );

    N_CrvInitArrays( &curB );
    N_CrvInitArrays( &curT );

    if( rb GT NL_MTOL )
    {
        error = N_CreateCircArc( C, X, Y, rb, as, ae, ctp, &curB, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( rt GT NL_MTOL )
    {
        error = N_CreateCircArc( A, X, Y, rt, as, ae, ctp, &curT, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get local notations */

    if( rb GT NL_MTOL )
        N_CrvGetCPtsDegreeAndKnots( &curB, &n, &Bw, &p, &m, &U );

    if( rt GT NL_MTOL )
        N_CrvGetCPtsDegreeAndKnots( &curT, &n, &Tw, &p, &m, &U );

    /* Get highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            ni = n;
            mi = 1;
            pi = p;
            qi = 1;
            ri = m;
            si = 3;
            break;

        case NL_VDIR:

            ni = 1;
            mi = n;
            pi = 1;
            qi = p;
            ri = 3;
            si = m;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, ni, mi, pi, qi, ri, si, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Extrude the circle as a u-curve */

    if( dir EQ NL_UDIR )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( rb GT NL_MTOL )
            {
                N_CopyCPt( Bw[i], &Qw[i][0] );
            }
            else
            {
                N_CPtGetW( Tw[i], &w );
                N_Weight( C, w, &Qw[i][0] );
            }

            if( rt GT NL_MTOL )
            {
                N_CopyCPt( Tw[i], &Qw[i][1] );
            }
            else
            {
                N_CPtGetW( Bw[i], &w );
                N_Weight( A, w, &Qw[i][1] );
            }
        }

        /* Get the knots */

        for ( i = 0; i <= m; i++ )
            UQ[i] = U[i];

        for ( j = 0; j <= 1; j++ )
        {
            VQ[j] = 0.0;
            VQ[2 + j] = 1.0;
        }
    }

    /* Extrude the circle as a v-curve */

    if( dir EQ NL_VDIR )
    {
        for ( j = 0; j <= n; j++ )
        {
            if( rb GT NL_MTOL )
            {
                N_CopyCPt( Bw[j], &Qw[0][j] );
            }
            else
            {
                N_CPtGetW( Tw[j], &w );
                N_Weight( C, w, &Qw[0][j] );
            }

            if( rt GT NL_MTOL )
            {
                N_CopyCPt( Tw[j], &Qw[1][j] );
            }
            else
            {
                N_CPtGetW( Bw[j], &w );
                N_Weight( A, w, &Qw[1][j] );
            }
        }

        /* Get the knots */

        for ( i = 0; i <= 1; i++ )
        {
            UQ[i] = 0.0;
            UQ[2 + i] = 1.0;
        }

        for ( j = 0; j <= m; j++ )
            VQ[j] = U[j];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCylCone */

#ifdef NLIB_UNUSED

/**********************************************************************/
/* N_CREATEELLIPTICALCYLCONE: Create a elliptic/cone surface/patch                     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This  common  surfaces  routine creates an elliptic cylinder or a cone or a
     patch of them by extruding  an elliptic  arc. It allows the 
     user to choose  between a  quadratic circle  (with internal knots)
     and a  quintic  circle  (with  no  internal  knots). If the output 
     surface is initialized to NULL, memory to store new control points  
     and knots is allocated. A typical calling example is:

       NL_POINT    C;
       NL_VECTOR   X, Y;
       NL_REAL     rb0, rb1, rt0, rt1, as, ae, h;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get C, X, Y, rb0, rb1, rt0, rt1, as, ae, and h);
       ...
       N_SrfInitArrays(&sur);
       N_CreateEllipticalCylCone(C,X,Y,rb0,rb1,rt0,rt1,as,ae,h,NL_QUINTIC,NL_UDIR,&sur,&SG);

     If  memory is  available, sur  is not  initialized and the routine
     assumes that  memory allocation has been done. However, it  checks  
     for the proper  amount by looking at the highest  indexes in sur's 
     knot vector  and  polygon  objects. The  extrusion  can be done in
     either u- or in v-direction.


   ACCESS:
   
     C,X,Y , input  ,  Center of  botton  circle  and  orthogonal  axes 
                       defining bottom plane
     rb0, rb1 input  , Bottom elliptic radii
     rt0, rt1 input  , Top elliptic radii
     as,ae , input  ,  Start and end sweep angles of base circle
     h     , input  ,  Height of cylinder/cone
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: Use   quadratic   circle  (internal 
                                    knots)
                         NL_QUINTIC  : Use   quintic  circle  (no internal
                                    knots)
     dir   , input  ,  Flag:
                         NL_UDIR: base circle is a u-curve
                         NL_VDIR: base circle is a v-curve
     sur   , output ,  NURBS cylinder/cone surface/patch
     SG    , input  ,  sur's stack stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateEllipticalCylCone( NL_POINT C, NL_VECTOR X, NL_VECTOR Y, NL_REAL rb0, NL_REAL rb1, NL_REAL rt0, NL_REAL rt1, NL_REAL as, NL_REAL ae, NL_REAL h, NL_FLAG ctp, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateEllipticalCylCone");

    NL_FLAG error = NL_NO, atp;

    NL_INDEX i, j, n = 0, m = 0, ni, mi, ri, si;

    NL_DEGREE p = 0, pi, qi;

    NL_REAL *U = NULL, *UQ, *VQ, w;

    NL_POINT A;

    NL_VECTOR Z;

    NL_CPOINT *Bw = NULL, *Tw = NULL, ** Qw;

    NL_CURVE curB, curT;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get bottom and top ellipses */

    if( rb0 LE NL_MTOL AND rt0 LE NL_MTOL )
        NL_ERROR( NL_CAL_ERR );

    N_VectorCross( X, Y, &Z );

    error = N_VectorNormalizeRef( &Z );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorPtAlongVector( C, h, Z, &A );

    N_CrvInitArrays( &curB );
    N_CrvInitArrays( &curT );

    if( rb0 GT NL_MTOL && rb1 GT NL_MTOL )
    {
        atp = 0;
        error = N_CreateEllipticalArcAngType( C, X, Y, rb0, rb1, as, ae, atp, ctp, &curB, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( rt0 GT NL_MTOL && rt1 GT NL_MTOL )
    {
        atp = 0;
        error = N_CreateEllipticalArcAngType( A, X, Y, rt0, rt1, as, ae, atp, ctp, &curT, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get local notations */

    if( rb0 GT NL_MTOL && rb1 GT NL_MTOL )
        N_CrvGetCPtsDegreeAndKnots( &curB, &n, &Bw, &p, &m, &U );

    if( rt0 GT NL_MTOL && rt1 GT NL_MTOL )
        N_CrvGetCPtsDegreeAndKnots( &curT, &n, &Tw, &p, &m, &U );

    /* Get highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            ni = n;
            mi = 1;
            pi = p;
            qi = 1;
            ri = m;
            si = 3;
            break;

        case NL_VDIR:

            ni = 1;
            mi = n;
            pi = 1;
            qi = p;
            ri = 3;
            si = m;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, ni, mi, pi, qi, ri, si, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Extrude the ellipse as a u-curve */

    if( dir EQ NL_UDIR )
    {
        for ( i = 0; i <= n; i++ )
        {
            if( rb0 GT NL_MTOL && rb1 GT NL_MTOL )
            {
                N_CopyCPt( Bw[i], &Qw[i][0] );
            }
            else
            {
                N_CPtGetW( Tw[i], &w );
                N_Weight( C, w, &Qw[i][0] );
            }

            if( rt0 GT NL_MTOL && rt1 GT NL_MTOL )
            {
                N_CopyCPt( Tw[i], &Qw[i][1] );
            }
            else
            {
                N_CPtGetW( Bw[i], &w );
                N_Weight( A, w, &Qw[i][1] );
            }
        }

        /* Get the knots */

        for ( i = 0; i <= m; i++ )
            UQ[i] = U[i];

        for ( j = 0; j <= 1; j++ )
        {
            VQ[j] = 0.0;
            VQ[2 + j] = 1.0;
        }
    }

    /* Extrude the ellipse as a v-curve */

    if( dir EQ NL_VDIR )
    {
        for ( j = 0; j <= n; j++ )
        {
            if( rb0 GT NL_MTOL && rb1 GT NL_MTOL )
            {
                N_CopyCPt( Bw[j], &Qw[0][j] );
            }
            else
            {
                N_CPtGetW( Tw[j], &w );
                N_Weight( C, w, &Qw[0][j] );
            }

            if( rt0 GT NL_MTOL && rt1 GT NL_MTOL )
            {
                N_CopyCPt( Tw[j], &Qw[1][j] );
            }
            else
            {
                N_CPtGetW( Bw[j], &w );
                N_Weight( A, w, &Qw[1][j] );
            }
        }

        /* Get the knots */

        for ( i = 0; i <= 1; i++ )
        {
            UQ[i] = 0.0;
            UQ[2 + i] = 1.0;
        }

        for ( j = 0; j <= m; j++ )
            VQ[j] = U[j];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateEllipticalCylCone */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a ruled surface between two
     arbitrary curves. If the output surface is  initialized to NULL, 
     memory  to store  new control  points and knots  is allocated. A 
     typical calling example is:

       NL_SURFACE  sur;
       NL_CURVE    curA, curB;
       NL_STACKS   S;
       ...
       (define curA and curB);
       ...
       N_SrfInitArrays(&sur);
       N_CreateRuledSrf(&curA,&curB,NL_UDIR,&sur,&S,&S);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon objects. The ruling can be done in either 
     u- or in v-direction. BOTH CURVES MUST BELONG TO THE SAME MEMORY
     STACK!


   ACCESS:
   
     curA , in/out ,  NURBS  curve to form  surface boundary S(0,v) or
                      S(u,0)
     curB , in/out ,  NURBS curve to  form surface  boundary S(1,v) or
                      S(u,1)
     dir  , input  ,  Flag:
                        NL_UDIR: ruled surface is linear in u-direction 
                        NL_VDIR: ruled surface is linear in v-direction 
     sur  , output ,  Ruled NURBS surface
     SC   , input  ,  Stack of curA and curB
     SS   , input  ,  Stack of sur


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateRuledSrf( NL_CURVE *curA, NL_CURVE *curB, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateRuledSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, ni, mi, ri, si;

    NL_DEGREE p, pi, qi;

    NL_REAL *U, *UQ, *VQ;

    NL_CPOINT *Aw, *Bw, ** Qw;

    NL_CURVE *cur[2];

    NL_BOOLEAN A3d, B3d;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make boundary curves compatible */

    cur[0] = curA;
    cur[1] = curB;

    error = N_CrvsMakeCompatible( cur, 1, SC );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsDegreeAndKnots( curA, &n, &Aw, &p, &m, &U );
    N_CrvGetCPts( curB, &n, &Bw );

    /* Get highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            ni = 1;
            mi = n;
            pi = 1;
            qi = p;
            ri = 3;
            si = m;
            break;

        case NL_VDIR:

            ni = n;
            mi = 1;
            pi = p;
            qi = 1;
            ri = m;
            si = 3;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, ni, mi, pi, qi, ri, si, rname, SS );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Get dimensionality of curves (surface should be 3D */

    A3d = N_CrvIs3d( curA );
    B3d = N_CrvIs3d( curB );

    /* Rule in u-direction */

    if( dir EQ NL_UDIR )
    {
        /* Compute control points */

        for ( j = 0; j <= n; j++ )
        {
            N_CopyCPt( Aw[j], &Qw[0][j] );
            N_CopyCPt( Bw[j], &Qw[1][j] );

            if( NOT A3d )
                N_CPtSetZ( 0.0, &Qw[0][j] );

            if( NOT B3d )
                N_CPtSetZ( 0.0, &Qw[1][j] );
        }

        /* Get the knots */

        for ( i = 0; i <= 1; i++ )
        {
            UQ[i] = 0.0;
            UQ[2 + i] = 1.0;
        }

        for ( j = 0; j <= m; j++ )
            VQ[j] = U[j];
    }

    /* Rule in v-direction */

    if( dir EQ NL_VDIR )
    {
        /* Compute control points */

        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( Aw[i], &Qw[i][0] );
            N_CopyCPt( Bw[i], &Qw[i][1] );

            if( NOT A3d )
                N_CPtSetZ( 0.0, &Qw[i][0] );

            if( NOT B3d )
                N_CPtSetZ( 0.0, &Qw[i][1] );
        }

        /* Get the knots */

        for ( i = 0; i <= m; i++ )
            UQ[i] = U[i];

        for ( j = 0; j <= 1; j++ )
        {
            VQ[j] = 0.0;
            VQ[2 + j] = 1.0;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateRuledSrf */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This common  surfaces routine  creates a ruled surface between two
     NURBS  curves. The result is  obtained by evaluating the following 
     expression:

                          N_p(v)      N_q(v)
           S(u,v) = (1-u) ------  + u ------
                          D_p(v)      D_q(v)

     If the  curves  are  non-rational, the  computation  simplifies to 
     (1-u)N_p(v) + uN_q(v). A typical calling example is:

       NL_CURVE    curP, curQ;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (define curP and curQ);
       ...
       N_SrfInitArrays(&sur);
       N_CreateRuledSrfFromBoundaryCrvs(&curP,&curQ,NL_UDIR,&sur,&SG);

     If memory is  available, sur  is not  initialized and  the routine 
     assumes that memory  allocation has  been done. However, it checks 
     for the  proper amount by  looking at the highest indexes in sur's
     knot vector and control net objects.


   ACCESS:
   
     curP , input  ,  Boundary curve (S(0,v) or S(u,0)) 
     curQ , input  ,  Boundary curve (S(1,v) or S(u,1)) 
     dir  , input  ,  Flag:
                        NL_UDIR: create ruling in the u-direction
                        NL_VDIR: create ruling in the v-direction
     sur  , output ,  Ruled surface
     SG   , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateRuledSrfFromBoundaryCrvs( NL_CURVE *curP, NL_CURVE *curQ, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateRuledSrfFromBoundaryCrvs");

    NL_FLAG error = NL_NO, ratP = NL_NO, ratQ = NL_NO, rat = NL_NO;

    NL_INDEX i, j, n, m, ns, ms, rs, ss, nsp, nsq, mxp, mxq;

    NL_DEGREE p, q, ps, qs;

    NL_REAL *UP, *UQ, *U, *V;

    NL_CPOINT *Pw, *Qw, ** Sw;

    NL_CURVE numP, numQ, numPQ, numQP, curPW, curQW;

    NL_CFUN denP, denQ, den;

    NL_KNOTVECTOR *knp, *knq, *kxp, *kxq;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get rational flags */

    if( N_IsCrvRat( curP ) )
        ratP = NL_YES;

    if( N_IsCrvRat( curQ ) )
        ratQ = NL_YES;

    if( ratP EQ NL_YES OR ratQ EQ NL_YES )
        rat = NL_YES;

    /* Make boundary curves compatible */

    switch( rat )
    {
        case NL_YES:

            /* Make non-rational curve rational */

            if( ratP EQ NL_NO )
                N_CrvNonRatToRat( curP );

            if( ratQ EQ NL_NO )
                N_CrvNonRatToRat( curQ );

            /* Extract numerators and denominators */

            N_CrvInitArrays( &numP );
            N_CFuncInitArrays( &denP );
            error = N_CrvGetNumAndDenom( curP, &numP, &denP, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &numQ );
            N_CFuncInitArrays( &denQ );
            error = N_CrvGetNumAndDenom( curQ, &numQ, &denQ, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Compute product of denominators */

            N_CFuncInitArrays( &den );
            error = N_CrvFuncMultiplyCrvFunc( &denP, &denQ, &den, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Compute terms in the numerator */

            N_CrvInitArrays( &numPQ );
            error = N_CrvFuncMultiplyCrv( &denQ, &numP, &numPQ, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &numQP );
            error = N_CrvFuncMultiplyCrv( &denP, &numQ, &numQP, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Create curves */

            N_CrvInitArrays( &curPW );
            error = N_CreateCrvFromNumAndDenom( &numPQ, &den, &curPW, SG );

            if( error EQ NL_YES )
                NL_OUT;
            N_CrvGetCPtsDegreeAndKnots( &curPW, &n, &Pw, &p, &m, &UP );

            N_CrvInitArrays( &curQW );
            error = N_CreateCrvFromNumAndDenom( &numQP, &den, &curQW, SG );

            if( error EQ NL_YES )
                NL_OUT;
            N_CrvGetCPtsDegreeAndKnots( &curQW, &n, &Qw, &p, &m, &UQ );

            if( ratP EQ NL_NO )
                N_CrvRatToNonRat( curP );

            if( ratQ EQ NL_NO )
                N_CrvRatToNonRat( curQ );
            break;

        case NL_NO:

            /* Create working curves */

            N_CrvInitArrays( &curPW );
            error = N_CrvCopy( curP, &curPW, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &curQW );
            error = N_CrvCopy( curQ, &curQW, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Get local notation */

            N_CrvGetCPtsDegreeAndKnots( &curPW, &n, &Pw, &p, &m, &UP );
            N_CrvGetCPtsDegreeAndKnots( &curQW, &n, &Qw, &q, &m, &UQ );
            N_CrvGetKnotVector( &curPW, &knp );
            N_CrvGetKnotVector( &curQW, &knq );

            /* Degree elevate */

            if( p NEQ q )
            {
                if( p LT q )
                {
                    error = N_CrvElevateDegree( &curPW, q - p, &curPW, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CrvGetCPtsDegreeAndKnots( &curPW, &n, &Pw, &p, &m, &UP );
                    N_CrvGetKnotVector( &curPW, &knp );
                }
                else
                {
                    error = N_CrvElevateDegree( &curQW, p - q, &curQW, &SL, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    N_CrvGetCPtsDegreeAndKnots( &curQW, &n, &Qw, &q, &m, &UQ );
                    N_CrvGetKnotVector( &curQW, &knq );
                }
            }

            /* Refine curves */

            N_BasisGetSpanCount( knp, p, &nsp );
            N_BasisGetSpanCount( knq, q, &nsq );

            kxp = N_AllocKnotVectorAndArray( nsq * q, &SL );

            if( kxp EQ NULL )
                NL_QUIT;

            kxq = N_AllocKnotVectorAndArray( nsp * p, &SL );

            if( kxq EQ NULL )
                NL_QUIT;

            N_GetCompatibleKnotArrayMult( knp, knq, p, kxp, kxq );

            N_KnotVectorGetKnots( kxp, &mxp, &U );
            N_KnotVectorGetKnots( kxq, &mxq, &V );

            if( mxp GE 0 )
            {
                error = N_CrvRefine( &curPW, kxp, &curPW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPtsDegreeAndKnots( &curPW, &n, &Pw, &p, &m, &UP );
            }

            if( mxq GE 0 )
            {
                error = N_CrvRefine( &curQW, kxq, &curQW, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                N_CrvGetCPtsDegreeAndKnots( &curQW, &n, &Qw, &q, &m, &UQ );
            }
            break;

        default:

            NL_ERROR( NL_INP_ERR );
    }

    /* Compute surface indexes */

    switch( dir )
    {
        case NL_UDIR:

            ns = 1;
            ps = 1;
            rs = 3;
            ms = n;
            qs = p;
            ss = m;
            break;

        case NL_VDIR:

            ns = n;
            ps = p;
            rs = m;
            ms = 1;
            qs = 1;
            ss = 3;
            break;

        default:

            NL_ERROR( NL_INP_ERR );
    }

    /* Check if surface memory is needed */

    error = N_SrfSizeArrays( sur, ns, ms, ps, qs, rs, ss, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &U, &V );

    /* Create ruled surface */

    switch( dir )
    {
        case NL_UDIR:
            for ( j = 0; j <= n; j++ )
            {
                N_CopyCPt( Pw[j], &Sw[0][j] );
                N_CopyCPt( Qw[j], &Sw[1][j] );
            }

            for ( i = 0; i <= 1; i++ )
            {
                U[i] = 0.0;
                U[2 + i] = 1.0;
            }

            for ( j = 0; j <= m; j++ )
                V[j] = UP[j];
            break;

        case NL_VDIR:
            for ( i = 0; i <= n; i++ )
            {
                N_CopyCPt( Pw[i], &Sw[i][0] );
                N_CopyCPt( Qw[i], &Sw[i][1] );
            }

            for ( i = 0; i <= m; i++ )
                U[i] = UP[i];

            for ( j = 0; j <= 1; j++ )
            {
                V[j] = 0.0;
                V[2 + j] = 1.0;
            }
            break;

        default:

            NL_ERROR( NL_INP_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateRuledSrfFromBoundaryCrvs */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This common  surfaces  routine  creates a surface of revolution by
     revolving an arbitrary curve  around an arbitrary  axis. Quadratic
     or quintic  circles  are  used  to  represent  iso-curves  in  the 
     direction  of  revolution. Qudratic circles  may result in  double
     knots in the  u-knot  vector.  Quintic  circles  give  no internal 
     knots.  If the output  surface  is initialized to NULL,  memory to
     store new control points and knots is allocated. A typical calling
     example is:

       NL_CURVE    cur;
       NL_POINT    S; 
       NL_VECTOR   T;
       NL_REAL     al;
       NL_SURFACE  sur;
       NL_STACKS   SG;
       ...
       (get cur, S, T and al)
       ...
       N_SrfInitArrays(&sur);
       N_CreateRevolvedSrf(&cur,S,T,al,NL_QUINTIC,&sur,&SG);

     If  memory is  available, sur  is not  initialized and the routine
     assumes that  memory allocation has been done. However, it  checks  
     for the proper  amount by looking at the highest  indexes in sur's 
     knot vector  and  polygon  objects. 


   ACCESS:
   
     cur , input  ,  NURBS curve to be revolved
     S,T , input  ,  Start  point  and  direction  vector  of  axis  of 
                     revolution
     al  , input  ,  Angle of revolution (al > 0),  measured  clockwise
                     looking in the direction of T.
     ctp , input  ,  Flag:
                       NL_QUADRATIC: rotation iso-curves are quadratic
                       NL_QUINTIC  : rotation iso-curves are quintic
     sur , output ,  NURBS surface of revolution
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateRevolvedSrf( NL_CURVE *cur, NL_POINT S, NL_VECTOR T, NL_REAL al, NL_FLAG ctp, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateRevolvedSrf");

    NL_FLAG prj, allrzero, error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *UQ, *VQ, wi[9], t, rad, wp;

    NL_POINT A, C;

    NL_VECTOR X, Y;

    NL_CPOINT *Pw, *Aw, ** Qw;

    NL_CURVE curA;

    NL_LINESEG lst;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get number of arcs */

    n = m = p = 0;

    N_CrvGetConicData( ctp, 0.0, &al, &n, &p, &r );

    /* Get local notations */

    N_CrvGetCPtsDegreeAndKnots( cur, &m, &Pw, &q, &s, &V );

    /* Make profile curve 3-D rational */

    if( NOT N_IsCrvRat( cur ) )
        N_CrvNonRatToRat( cur );

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    /* Get memory for the circle */

    error = N_AllocCrvArrays( &curA, n, p, r, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* See if memory is needed for the output */

    error = N_SrfSizeArrays( sur, n, m, p, q, r, s, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

    /* Extract circle weights */

    error = N_CalcCircWeights( 0.0, al, ctp, wi, &i );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute surface control points */

    N_CreateLineStartDirVector( &lst, S, T, NL_UNBOUNDED );

    allrzero = NL_TRUE;

    for ( j = 0; j <= m; j++ )
    {
        N_CPtToPtEuclid( Pw[j], &A );
        N_CPtGetW( Pw[j], &wp );

        error = N_ProjectPtLine( lst, A, &C, &t, &prj );

        if( error EQ NL_YES )
            NL_OUT;

        if( prj EQ NL_FALSE )
            NL_ERROR( NL_INP_ERR );

        N_VectorDiff( A, C, &X );
        N_VectorMagnitude( X, &rad );

        if( rad LT NL_MTOL )
        {
            for ( i = 0; i <= n; i++ )
            {
                N_Weight( C, wi[i] * wp, &Qw[i][j] );
            }
            continue;
        }

        allrzero = NL_FALSE;

        N_VectorCross( T, X, &Y );

        error = N_CreateCircArc( C, X, Y, rad, 0.0, al, ctp, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPts( &curA, &i, &Aw );

        for ( i = 0; i <= n; i++ )
        {
            N_ScaleCPt( wp, Aw[i], &Qw[i][j] );
        }
    }

    if( allrzero EQ NL_TRUE )
        NL_ERROR( NL_CAL_ERR );

    /* Get the knots */

    N_CrvGetKnots( &curA, &i, &U );

    for ( i = 0; i <= r; i++ )
        UQ[i] = U[i];

    for ( j = 0; j <= s; j++ )
        VQ[j] = V[j];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateRevolvedSrf */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a NURBS sphere or  spherical
     patch. Either  degree 2 or  degree 5  curves can be  used in both
     directions,  e.g.  the  degree  (5x5)  sphere  is defined with no 
     internal knots.  The sphere is created by revolving the  circular 
     arc clockwise about the Z axis. If the output surface is initial-
     ized to NULL, memory  to  store  new control  points and knots is
     allocated. A typical calling example is:

       NL_POINT    C;
       NL_REAL     r, as, ae, al;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get r, as, ae and al);
       ...
       N_SrfInitArrays(&sur);
       N_CreateSphere(C,r,as,ae,al,NL_QUINTIC,NL_QUADRATIC,&sur,&S);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon objects.


   ACCESS:
   
     C     , input  ,  Center of sphere
     r     , input  ,  Radius of sphere
     as,ae , input  ,  Start and  end angles of  circular  arc  sweep. 
                       MUST SATISFY 0 <= as < ae <= 180. as=0  corres-
                       ponds to the point  C + r*Z , and ae=180 to the
                       point C - r*Z.
     al    , input  ,  Angle of revolution from the Y axis (al > 0). 
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: profile circle is quadratic
                         NL_QUINTIC  : profile circle is quintic
     rtp   , input  ,  Flag:
                         NL_QUADRATIC: revolution iso-curve is quadratic
                         NL_QUINTIC  : revolution iso-curve is quintic
     sur   , output ,  NURBS sphere/spherical patch
     SG    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateSphere( NL_POINT C, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL al, NL_FLAG ctp, NL_FLAG rtp, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateSphere");

    NL_FLAG error = NL_NO;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check angles */

    if( as GE ae )
        NL_ERROR( NL_CAL_ERR );

    if( as LT 0.0 OR ae GT 180.0 )
        NL_ERROR( NL_CAL_ERR );

    /* Get circle to be revolved */

    N_CrvInitArrays( &curA );
    error = N_CreateCircArc( C, NL_UNITZ, NL_UNITY, r, as, ae, ctp, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Revolve circle/arc to get sphere/patch */

    error = N_CreateRevolvedSrf( &curA, C, NL_UNITZ, al, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateSphere */

/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a  NURBS  torus  or toroidal
     patch. Either  degree 2 or  degree 5  curves can be  used in both
     directions,  e.g.  the  degree  (5x5)  torus  is  defined with no 
     internal  knots.  If the output  surface is  initialized to NULL, 
     memory  to  store  new control  points and knots  is allocated. A 
     typical calling example is:

       NL_POINT    S, C;
       NL_VECTOR   T;
       NL_REAL     r, as, ae, al;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get S, T, C, r, as, ae and al);
       ...
       N_SrfInitArrays(&sur);
       N_CreateTorus(S,T,C,r,as,ae,al,NL_QUINTIC,NL_QUADRATIC,&sur,&S);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and control net objects.


   ACCESS:
   
     S,T   , input  ,  Point and direction of axis of revolution
     C     , input  ,  Center of circle (or arc) to be revolved  about
                       the axis. Circle is in a local coordinate  sys-
                       tem (X,T), where X points from the axis  toward
                       the arc center, C.
     r     , input  ,  Radius of circle.
     as,ae , input  ,  Start and end angles of circular arc  (measured
                       in (X,T)).
     al    , input  ,  Angle of revolution clockwise about the axis.
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: profile circle is quadratic
                         NL_QUINTIC  : profile circle is quintic
     rtp   , input  ,  Flag:
                         NL_QUADRATIC: revolution iso-curve is quadratic
                         NL_QUINTIC  : revolution iso-curve is quintic
     sur   , output ,  NURBS torus/toroidal patch
     SG    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTorus( NL_POINT S, NL_VECTOR T, NL_POINT C, NL_REAL r, NL_REAL as, NL_REAL ae, NL_REAL al, NL_FLAG ctp, NL_FLAG rtp, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTorus");

    NL_FLAG prj, error = NL_NO;

    NL_REAL d;

    NL_POINT A;

    NL_VECTOR X;

    NL_LINESEG lst;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get circle to be revolved */

    N_CreateLineStartDirVector( &lst, S, T, NL_UNBOUNDED );

    error = N_ProjectPtLine( lst, C, &A, &d, &prj );

    if( error EQ NL_YES )
        NL_OUT;

    if( prj EQ NL_FALSE )
        NL_ERROR( NL_INP_ERR );

    N_VectorDiff( C, A, &X );
    N_VectorMagnitude( X, &d );

    if( d LT NL_MTOL )
        NL_ERROR( NL_CAL_ERR );

    N_CrvInitArrays( &curA );
    error = N_CreateCircArc( C, X, T, r, as, ae, ctp, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Revolve circle/arc to get torus/patch */

    error = N_CreateRevolvedSrf( &curA, S, T, al, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateTorus */

/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a NURBS representation of an
     elliptic  paraboloid in  the standard  position, i.e. the axis of 
     symmetry is the z axis and the surface passes through the origin. 
     The paraboloid's equation is

             x^2    y^2
             ---- + ---- = 2*z
             rx^2   ry^2

     where rx and ry are the radii of the ellipse lying in the z = 1/2
     plane. If the output surface is  initialized to the NULL surface, 
     memory  to  store  new control  points and knots  is allocated. A 
     typical calling example is:

       NL_REAL     rx, ry, h, al;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get rx, ry, h and al);
       ...
       N_SrfInitArrays(&sur);
       N_CreateParaboloid(rx,ry,h,al,NL_QUINTIC,&sur,&S);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon objects. A full paraboloid is  created if
     the rotation angle is 360 degrees. Otherwise a patch is computed.


   ACCESS:
   
     rx,ry , input  ,  Radii of ellipse in the z=1/2 plane
     h     , input  ,  Height of the paraboloid
     al    , input  ,  Angle of rotation
     rtp   , input  ,  Flag:
                         NL_QUADRATIC: revolution iso-curve is quadratic
                         NL_QUINTIC  : revolution iso-curve is quintic
     sur   , output ,  NURBS elliptic paraboloid
     SG    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateParaboloid( NL_REAL rx, NL_REAL ry, NL_REAL h, NL_REAL al, NL_FLAG rtp, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateParaboloid");

    NL_FLAG error = NL_NO;

    NL_REAL *U, x;

    NL_VECTOR f;

    NL_CPOINT *Pw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( rx LT NL_MTOL OR ry LT NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    /* Get parabolic curve to be revolved */

    error = N_AllocCrvArrays( &curA, 2, 2, 5, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( &curA, &Pw, &U );

    x = rx * sqrt( 2.0 *h );

    N_CPtFromWxWyWz( 0.0, 0.0, 0.0, 1.0, &Pw[0] );
    N_CPtFromWxWyWz( 0.5 *x, 0.0, 0.0, 1.0, &Pw[1] );
    N_CPtFromWxWyWz( x, 0.0, h, 1.0, &Pw[2] );

    U[0] = U[1] = U[2] = 0.0;
    U[3] = U[4] = U[5] = 1.0;

    /* Revolve parabola to get rotation paraboloid */

    error = N_CreateRevolvedSrf( &curA, NL_ZERO, NL_UNITZ, al, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Scale paraboloid if necesary */

    if( fabs( rx - ry )GT NL_MTOL )
    {
        N_VectorCreate( 1.0, ry / rx, 1.0, &f );
        N_SrfScale( sur, NL_ZERO, f );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateParaboloid */

/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a NURBS representation of an
     ellipsoid/patch  in the standard position, i.e. the  center is at 
     the  origin  and  the  three  principal  axes  are the x-, y- and 
     z-axis. The ellipsoid's equation is

             x^2    y^2    z^2
             ---- + ---- + ---- = 1
             rx^2   ry^2   rz^2

     where rx, ry  and rz  are the  radii of the ellipses lying in the
     three principal planes. If the  output surface is  initialized to 
     the NULL surface, memory to  store  new control  points and knots  
     is allocated. A typical calling example is:

       NL_REAL     rx, ry, rz, as, ae, al;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get rx, ry, rz, as, ae and al);
       ...
       N_SrfInitArrays(&sur);
       N_CreateEllipsoid(rx,ry,rz,as,ae,al,NL_QUINTIC,NL_QUADRATIC,&sur,&S);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot  vector  and  polygon objects.  The ellipsoid  (or patch) is 
     created  by  revolving a  circular arc  in the YZ plane clockwise 
     about the Z axis,  then scaling by  (rx,ry,rz).  A full ellipsoid
     is created if as=0 and ae=180 (half-circle in the YZ plane),  and 
     al=360 (Revolution about Z). Otherwise a patch is computed.


   ACCESS:
   
     rx,ry,rz , input  ,  Radii of ellipses along the  three principal
                          axes
     as,ae    , input  ,  Start and end angles of elliptical arc in YZ
                          plane. MUST SATISFY 0 <= as < ae <= 180.
     al       , input  ,  Angle of revolution (al > 0).
     ctp      , input  ,  Flag:
                           NL_QUADRATIC: profile ellipse is quadratic
                           NL_QUINTIC  : profile ellipse is quintic
     rtp      , input  ,  Flag:
                            NL_QUADRATIC: revolution iso-curve is quadratic
                            NL_QUINTIC  : revolution iso-curve is quintic
     sur      , output ,  NURBS ellipsoid
     SG       , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateEllipsoid( NL_REAL rx, NL_REAL ry, NL_REAL rz, NL_REAL as, NL_REAL ae, NL_REAL al, NL_FLAG ctp, NL_FLAG rtp, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateEllipsoid");

    NL_FLAG error = NL_NO;

    NL_VECTOR f;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( rx LT NL_MTOL OR ry LT NL_MTOL OR rz LT NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    /* Get unit sphere */

    error = N_CreateSphere( NL_ZERO, 1.0, as, ae, al, ctp, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Scale ellipsoid */

    N_VectorCreate( rx, ry, rz, &f );
    N_SrfScale( sur, NL_ZERO, f );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateEllipsoid */

/*******************************************************************//**


   DESCRIPTION:

     This common surfaces routine creates a NURBS representation  of a
     hyperboloid of one sheet in the  standard position, i.e. the axis 
     of symmetry is the z axis and the center of the surface is at the 
     origin. The hyperboloid's equation is

             x^2    y^2    z^2
             ---- + ---- - ---- = 1
             rx^2   ry^2   rz^2

     where rx and ry are the radii of the  ellipse lying in  the z = 0
     plane, and the rz is one of the radii  of hyperbolae lying in the
     x = 0 or y = 0  planes. If the  output surface is  initialized to 
     the NULL surface, memory to store new control points and knots is 
     allocated. A typical calling example is:

       NL_REAL     rx, ry, rz, h, al;
       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (get rx, ry, rz, h and al);
       ...
       N_SrfInitArrays(&sur);
       N_CreateHyperboloid(rx,ry,rz,h,al,NL_QUINTIC,&sur,&S);

     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon objects. A full hyperboloid is created if
     the rotation angle is 360 degrees (full means that the surface is
     clamped only by  the  z = h and z = -h planes). Otherwise a patch 
     is computed.


   ACCESS:
   
     rx,ry,rz , input  ,  Radii  of ellipses  and hyperbolae  lying in 
                          the principal planes
     h        , input  ,  Height of the hyperboloid
     al       , input  ,  Angle of rotation
     rtp      , input  ,  Flag:
                            NL_QUADRATIC: revolution iso-curve is quadratic
                            NL_QUINTIC  : revolution iso-curve is quintic
     sur      , output ,  NURBS hyperboloid of one sheet
     SG       , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateHyperboloid( NL_REAL rx, NL_REAL ry, NL_REAL rz, NL_REAL h, NL_REAL al, NL_FLAG rtp, NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateHyperboloid");

    NL_FLAG error = NL_NO;

    NL_REAL *U, xh, xt, h2, rz2, d1, d2, s, w;

    NL_VECTOR f;

    NL_POINT P, S, M;

    NL_CPOINT *Pw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( rx LT NL_MTOL OR ry LT NL_MTOL OR rz LT NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    /* Get hyperbolic curve to be revolved */

    error = N_AllocCrvArrays( &curA, 2, 2, 5, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( &curA, &Pw, &U );

    h2 = h * h;
    rz2 = rz * rz;
    xh = rx * sqrt( 1.0 + h2 / rz2 );
    xt = (xh * xh * rz2 - rx * rx * h2) / (xh * rz2);

    N_VectorCreate( xh, 0.0, 0.0, &M );
    N_VectorCreate( rx, 0.0, 0.0, &S );
    N_VectorCreate( xt, 0.0, 0.0, &P );

    N_DistPtPt( S, M, &d1 );
    N_DistPtPt( P, M, &d2 );

    s = d1 / d2;
    w = s / (1.0 - s);

    N_CPtFromWxWyWz( xh, 0.0, h, 1.0, &Pw[0] );
    N_CPtFromWxWyWz( xt * w, 0.0, 0.0, w, &Pw[1] );
    N_CPtFromWxWyWz( xh, 0.0, -h, 1.0, &Pw[2] );

    U[0] = U[1] = U[2] = 0.0;
    U[3] = U[4] = U[5] = 1.0;

    /* Revolve parabola to get rotation paraboloid */

    error = N_CreateRevolvedSrf( &curA, NL_ZERO, NL_UNITZ, al, rtp, sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Scale paraboloid if necesary */

    if( fabs( rx - ry )GT NL_MTOL )
    {
        N_VectorCreate( 1.0, ry / rx, 1.0, &f );
        N_SrfScale( sur, NL_ZERO, f );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateHyperboloid */

/*******************************************************************//**
 

   DESCRIPTION:
 
     This common surfaces routine creates a ruled  surface  (generalized
     cone) between an arbitrary curve and a point. If the output surface
     is  initialized to NULL,  memory  to store  new control  points and
     knots is allocated. A typical calling example is:
 
       NL_SURFACE  sur;
       NL_CURVE    cur;
       NL_POINT    pnt;
       NL_STACKS   S;
       ...
       (define cur and pnt);
       ...
       N_SrfInitArrays(&sur);
       N_CreateRuledSrfBetweenCrvAndPt(&cur,pnt,NL_UDIR,&sur,&S,&S);
 
     If memory is  available, sur  is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest  indexes in sur's 
     knot vector and polygon objects. The ruling can be done in either
     u- or in v-direction.
  
     The point (pnt) should not lie on the curve (crv). 
 

   ACCESS:
   
     cur , input  ,  NURBS curve to form surface  boundary  S(0,v)  or
                     S(u,0)
     pnt , input  ,  Point (apex of generalized cone)
     dir , input  ,  Flag:
                        NL_UDIR: ruled surface is linear in u-direction 
                        NL_VDIR: ruled surface is linear in v-direction 
     sur , output ,  Ruled NURBS surface (generalized cone)
     SC  , input  ,  Stack of cur
     SS  , input  ,  Stack of sur
 

   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_CreateRuledSrfBetweenCrvAndPt( NL_CURVE *cur, NL_POINT pnt, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{
    NL_STACKS SL;

    NL_FLAG error = NL_NO;

    NL_CURVE cur1;

    NL_CPOINT *Pw;

    NL_INDEX i, n;

    NL_REAL x, y, z, w, ww;

    /* start NURBS (initializes memory stacks) */

    N_InitNurbs( &SL );

    /* Copy curve */

    N_CrvInitArrays( &cur1 );

    error = N_CrvCopy( cur, &cur1, SC );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPts( &cur1, &n, &Pw );

    /* Get coordinates of point pnt */

    N_PtToXYZ( pnt, &x, &y, &z );

    /* Replace Euclidean coordinates of cur1's control points */
    /* with those of point pnt.                               */

    for ( i = 0; i <= n; i++ )
    {
        N_CPtGetW( Pw[i], &ww );

        w = ww;

        if( w EQ NL_NOW )
            w = 1.0;

        N_CPtFromWxWyWz( w * x, w * y, w * z, ww, &Pw[i] );
    }

    error = N_CreateRuledSrf( cur, &cur1, dir, sur, SC, SS );

    if( error EQ NL_YES )
        NL_OUT;

    /* Deallocate cur1's memory from SC stack */

    N_FreeCrv( &cur1, SC );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateRuledSrfBetweenCrvAndPt */
#endif // NLIB_UNUSED
