// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* Tessellate.c : Tessellation Routines under NLib                    */
/**********************************************************************/

#include "StdAfx.h"

#ifdef USE_NLIB_TESS

#include <nurbs.h>
#include <NL_Globals.h>

/* Prototypes only referenced within Tessellate.c */

static NL_FLAG ST_TessCheckTrim( NL_SURFACE *, NL_CURVE *, NL_PARAMETER, NL_PARAMETER, NL_REAL * );
static NL_FLAG ST_TessPolygonizeTrimCrv( NL_SURFACE *, NL_REAL, NL_REAL, NL_CURVE *, NL_REAL, NL_PARAMETER **, NL_PARAMETER **, 
                                         NL_INDEX *, NL_STACKS * );
static NL_VOID ST_TessFindBBoxIndices( NL_INDEX **, NL_FLAG *, NL_FLAG *, NL_INDEX, NL_INDEX, NL_INDEX *, NL_INDEX * );
static NL_FLAG ST_TessIsInsidePolygon( NL_REAL *, NL_REAL *, NL_INDEX, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, 
                                       NL_REAL, NL_REAL, NL_REAL, NL_REAL );
static NL_FLAG ST_TessIsInsideTrim( NL_REAL ***, NL_REAL ***, NL_INDEX *, NL_INDEX **, NL_INDEX, NL_REAL **, NL_REAL **, 
                                    NL_REAL **, NL_REAL **, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL );
static NL_FLAG ST_TessIsInsideLoop( NL_REAL **, NL_REAL **, NL_INDEX *, NL_INDEX, NL_REAL *, NL_REAL *, NL_REAL *, 
                                   NL_REAL *, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL );
static NL_BOOLEAN ST_TessLineXLine2d( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, 
                                     NL_REAL, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL * );
static NL_FLAG ST_IsectLineWith2dPolygon( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_REAL *,
                                         NL_REAL *, NL_INDEX, NL_REAL, NL_REAL, NL_REAL, NL_REAL,
                                         NL_REAL, NL_REAL, NL_REAL, NL_REAL,
                                         NL_REAL, NL_REAL **, NL_REAL **, NL_INDEX **, NL_FLAG **,
                                         NL_INDEX *, NL_STACKS * );
static NL_FLAG ST_TessSubdivideSrf( NL_SURFACE *, NL_REAL, NL_PARAMETER ***, NL_PARAMETER ***, NL_INDEX *, 
                                   NL_INDEX **, NL_INDEX, NL_REAL **, NL_REAL **, NL_REAL **, NL_REAL **, 
                                   NL_REAL, NL_REAL, NL_REAL, 
                                   NL_REAL, NL_PARAMETER **, NL_PARAMETER **, NL_FLAG **, NL_FLAG **, NL_INDEX *, 
                                   NL_INDEX ***, NL_FLAG ***, NL_INDEX *, NL_INDEX *, NL_STACKS *, NL_STACKS * );
static NL_FLAG ST_TessBBoxXTrim( NL_PARAMETER **, NL_PARAMETER **, NL_INDEX *, NL_INDEX **, NL_INDEX, NL_INDEX, 
                                NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_FLAG, NL_PARAMETER **, NL_PARAMETER **, 
                                NL_INDEX *, NL_INDEX, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL *,
                                NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_INDEX **, 
                                NL_INDEX *, NL_INDEX *, NL_INDEX, NL_INDEX ***, NL_INDEX **, NL_INDEX **, NL_INDEX *, 
                                NL_STACKS *, NL_STACKS * );
static NL_FLAG ST_TessRegionOfRectangles( NL_PARAMETER **, NL_PARAMETER **, NL_INDEX *, NL_FLAG *, NL_FLAG *, 
                                         NL_INDEX **, NL_FLAG **, NL_INDEX, NL_INDEX, NL_PARAMETER ***, NL_PARAMETER ***, 
                                         NL_INDEX *, NL_INDEX **, NL_INDEX, NL_REAL **, NL_REAL **, NL_REAL **, 
                                         NL_REAL **, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_INDEX ***, NL_INDEX **, NL_STACKS * );

static NL_FLAG ST_TessIsOnBoundary( NL_CURVE *, NL_REAL, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, 
                                   NL_FLAG *, NL_FLAG *, NL_FLAG * );
static NL_VOID ST_TessInitArray( NL_INDEX *, NL_INDEX, NL_INDEX, NL_INTEGER );
static NL_FLAG ST_TessCosine( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL * );
static NL_FLAG ST_TessSideTest( NL_REAL, NL_REAL, NL_REAL, NL_REAL );
static NL_FLAG ST_TessEdgeTouch( NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_FLAG * );
static NL_BOOLEAN ST_TessPtInSpan( NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_REAL );
static NL_BOOLEAN ST_TessIsEdgeOnBoundary( NL_INDEX, NL_INDEX, NL_INDEX **, NL_INDEX *, NL_INDEX );
static NL_BOOLEAN ST_TessLineXPolygon( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_PARAMETER *, 
                                      NL_PARAMETER *, NL_INDEX **, NL_INDEX *, NL_INDEX, NL_REAL );
static NL_FLAG ST_TessSideTest2d( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL );
static NL_VOID ST_TessOverlap( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG * );
static NL_FLAG ST_ReallocArrays( NL_REAL **, NL_REAL **, NL_INDEX **, NL_FLAG **, NL_INDEX *, NL_INDEX, NL_STACKS * );
static NL_FLAG ST_TessAppendPt( NL_PARAMETER **, NL_PARAMETER **, NL_INDEX *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, 
                               NL_INDEX *, NL_STACKS * );

static NL_FLAG ST_TessBuffer( NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_REAL, 
                             NL_INDEX **, NL_INDEX *, NL_INDEX *, NL_INDEX, NL_FLAG *, NL_INDEX *, NL_STACKS * );

static NL_FLAG ST_TessFindUnused( NL_INDEX *, NL_FLAG *, NL_FLAG *, NL_INDEX *, NL_INDEX *, NL_INDEX, NL_INDEX *, 
                                 NL_FLAG *, NL_FLAG *, NL_INDEX *, NL_INDEX *, NL_INDEX, NL_INDEX *, NL_FLAG *, 
                                 NL_FLAG *, NL_INDEX *, NL_INDEX *, NL_INDEX, NL_INDEX *, NL_FLAG *, NL_FLAG *, 
                                 NL_INDEX *, NL_INDEX *, NL_INDEX, NL_INDEX *, NL_INDEX *, NL_FLAG *, NL_INDEX *, NL_INDEX * );

static NL_FLAG ST_TessAppendPolygonVertex( NL_INDEX **, NL_INDEX *, NL_INDEX, NL_INDEX, NL_INDEX *, NL_STACKS * );

static NL_BOOLEAN ST_TessIsPtInBBox( NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL );

static NL_BOOLEAN ST_TessIsPtBBoxCorner( NL_FLAG, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX );

static NL_FLAG ST_IsPtInPolygon( NL_REAL, NL_REAL, NL_REAL *, NL_REAL *, NL_INDEX, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL );

static NL_BOOLEAN ST_IsPtInPoly( NL_PARAMETER *, NL_PARAMETER *, NL_INDEX *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_REAL );

static NL_FLAG ST_TessFindCrossingPt( NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_FLAG *, NL_INDEX, NL_INDEX *, 
                                     NL_INDEX *, NL_INDEX *, NL_FLAG *, NL_INDEX, NL_INDEX *, NL_INDEX *, 
                                     NL_INDEX *, NL_FLAG *, NL_INDEX, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_FLAG *, 
                                     NL_INDEX, NL_INDEX, NL_INDEX *, NL_INDEX, NL_INDEX, NL_FLAG *, NL_INDEX *, NL_INDEX * );

/**********************************************************************/
/* N_TessTrimmedSrf: Tessellate a trimmed NURBS surface               */
/**********************************************************************/

/* THIS IS USER CALLABLE
 
 
   DESCRIPTION:
 
     This  tesselation  routine  triangulates a  trimmed  NURBS surface. 
     The trimming is by NURBS curves lying in the parameter space of the
     surface. The trimmed domain consists of disjoint regions bounded by
     outer and inner loops. The following restrictions apply:
 
       1. All outer and inner loops must be 2d curves.
       2. All loops are disjoint, i.e. they cannot  intersect or contain
          one another.
       3. No loop may intersect itself except at its endpoints(closed).
       4. Outer loops are oriented COUNTERCLOCKWISE, whereas inner loops
          are oriented CLOCKWISE.
       5. Each inner loop is contained in exactly one outer loop.
       6. Curve  segments  making up an  outer or an inner  loop must be 
          joined with at least C0 continuity, i.e. no  gaps are allowed.
 
     The following notations are used:
 
       m   : highest index of outer loops.
       ho  : ho[i] is the  highest index of  segments in  the i - th outer 
             loop.
       hi  : hi[i] is the highest index of inner loops in the i - th outer
             loop.
       hs  : hs[i][j] is the highest index of segments in the j - th inner
             loop of the i - th outer loop.
       cuo : cuo[i][j]  points to  the  j - th  segment of  the i - th outer 
             loop; 0 <= i <= m, 0 <= j <= ho[i].
       cui : cui[i][j][k] points to the  k - th segment  in the j - th inner
             loop  of  the   i - th  outer   loop;  0 <= i <= m,  0 <= j <= hi[j], 
             0 <= k <= hs[i][j].
 
     To help  set up  these  pointers, the  memory  allocation  routines 
     N_AllocArrayCrvPtrs,  N_AllocArrayRealCrvPtrs  and N_AllocArrayTripleCrvPtrs  can be  used. As an example, the 
     following code fragment sets up all curve pointers to represent the
     inner loops:
 
       cui = N_AllocArrayTripleCrvPtrs(m, &S);
       if (cui EQ NULL)
       NL_QUIT;
 
       for (i = 0; i <= m; i++)
   {
   NL_PRIVATE  NL_STRING  rname = "N_TessTrimmedSrf");
   cui[i] = N_AllocArrayRealCrvPtrs(hi[i], &S);
   if (cui[i] EQ NULL)
   NL_QUIT;
   
     for (j = 0; j <= hi[i]; j++)
     {
     cui[i][j] = N_AllocArrayCrvPtrs(hs[i][j], &S);
     if (cui[i][j] EQ NULL)
     NL_QUIT;
     
       for (k = 0; k <= hs[i][j]; k++)
       {
       cui[i][j][k] = make this point to the
       k - th segment of the
       j - th inner loop in the
       i - th outer loop;
       }
       }
       }
 
     A typical calling example is as follows:
 
       NL_PARAMETER  *u, *v;
       NL_INDEX      **DT, *hd, *ho, *hi, **hs, m, n;
       NL_REAL       epc, eps, tol;
       NL_CURVE      ***cuo, ****cui;
       NL_SURFACE    sur;
       NL_STACKS     SG;
       ...
       (define trimming curves);
       ...
       N_TessTrimmedSrf(&sur, cuo, cui, m, ho, hi, hs, epc, eps, tol,
                &u, &v, &n, &DT, &hd, &SG);
 
     MEMORY TO STORE THE OUTPUT  NL_POINTS  u AND  v AND THE  TRIANGULATION 
     DATA  STRUCTURE DT  AND hd  IS ALLOCATED  INSIDE  THE  ROUTINE. THE 
     POLYGONIZATION TOLERANCE SHOULD BE SET SO THAT THE  POLYGONS DO NOT
     INTERSECT.
 
 
   ACCESS:
   
     sur  , in/out ,  NURBS surface(knot vectors are rescaled)
     cuo  , input  ,  Outer trimming loops(You MUST have some)
     cui  , input  ,  Inner trimming loops
     m    , input  ,  Highest index of outer trimming loops
     ho   , input  ,  Array of highest indexes of outer loop segments
     hi   , input  ,  Array of highest indexes of inner loops
     hs   , input  ,  Array  of  highest  indexes of  segments of  inner 
                      loops
     epc  , input  ,  Tolerance  to   polygonize  trimming   curves;  no 
                      straight edge  deviates from  any of the  trimming 
                      curves more that epc
     eps  , input  ,  Tessellation tolerance; no  triangle deviates from 
                      the surface more than eps
     tol  , input  ,  Parameter space tolerance; two parameters are con-
                      sidered the same if they differ less than tol.
                      Value must be small compared to eps
                      A typical value for tol is 1.0e-7
     u, v  , output ,  Parameters  corresponding   to  the   vertices  of 
                      triangles
     n    , output ,  Highest index in(u, v)
     DT   , output ,  Point lists of triangulation, i.e. DT[i] points to
                      a list of indexes representing  points surrounding
                      the point(u[i], v[i]). The indexes  are listed in
                      COUNTERCLOCKWISE  order. If  DT[i][0]  is  -1, the
                      point(u[i], v[i]) is out of the domain.
     hd   , output ,  hd[i] is the highest index of array DT[i]
     SG   , input  ,  Memory stack for u, v, DT and hd
 
     
   NOTE:
 
     The  data structure  DT records  topological  information via index 
     lists. Assume that we have five points representing the vertices of 
     four triangles:
                                2 x-----------x 5
       1 | 2 3 4 5 2              | \       / |
       2 | 3 1 5                  |   \ x /   |          
       3 | 4 1 2                  |   / 1 \   |
       4 | 5 1 3                  | /       \ |
       5 | 2 1 4                3 x-----------x 4
 
     The interpretation of this data structure is as follows:
 
       * The indexes in each list are listed counterclockwise.
       * Point 1 is an internal  point  because the  first and  the last 
         indexes are the same. Points 2 - 5 are all external points.
       * All edges incident at, say, point 3 are:
           <3, 4>  <3, 1>  <3, 2>
       * All triangles incident at, say, point 5 are:
           <5, 2, 1>  <5, 1, 4>
       * All triangles incident on, say, edge <1, 4> are:
           - Go to the list of 1 and find point 4 -> <1, 4, 5>
           - Go to the list of 4 and find point 1 -> <4, 1, 3>
 
     Although   this  data   structure  does   not  store   neightboring 
     information explicitly, simple procedures can be  written to answer
     all data base queries one might have.
                       
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_TessTrimmedSrf( NL_SURFACE *sur, NL_CURVE *** cuo, NL_CURVE **** cui, NL_INDEX m, NL_INDEX *ho, NL_INDEX *hi, NL_INDEX ** hs, NL_REAL epc, NL_REAL eps, NL_REAL tol, NL_PARAMETER ** u, NL_PARAMETER ** v, NL_INDEX *n, NL_INDEX *** DT, NL_INDEX ** hd, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TessTrimmedSrf");

    NL_FLAG ** SM = NULL, *R, *T, side, scor, ecor, ucl = NL_NO, vcl = NL_NO, uin = NL_NO, vin = NL_NO, error = NL_NO;

    NL_INDEX ** TM = NULL, ** hp, *ht, i, j, k, l, r = 0, s = 0, a, b, np, nt, nc, nl, ni, is, ks, nxy, kxy, kk, kkk, nnn;

    NL_REAL *** x, *** y, ** UL, ** UR, ** VB, ** VT, *ut, *vt, *ul, *vl, d1, d2, tmp, xl, xr, yb, yt, off, ax, ay, bx, by, z, lu, lv, usl, vsl, len, d3, d4;

    NL_CPOINT *Pw, *Qw;
    NL_POINT A, B;
    NL_CURVE ** seg;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* check paramater space is small compared to 3D tol */
    if( tol > eps * 0.001 )
        tol = 0.001 *eps;

    /* Get some constants */
    N_SrfGetParameterBounds( sur, &xl, &xr, &yb, &yt );

    error = N_TessSrfArea( sur, &tmp, &lu, &lv );

    if( error EQ NL_YES )
        NL_OUT;

    /* No longer using globals: */
    /* NL_TES_AREA = tmp; */
    /* NL_TES_LENU = lu;  */
    /* NL_TES_LENV = lv;  */

    if( tmp LE NL_MTOL * NL_MTOL )
        NL_ERROR( NL_INP_ERR );

    if( N_FloatOpIsBad( lu, xr - xl, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( lv, yt - yb, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    usl = lu / (xr - xl);
    vsl = lv / (yt - yb);

    off = 30.0 *tol;

    nxy = 20; /* initial size of x[][] and y[][] arrays */

    /**********************************/
    /* Polygonize trimming curves     */
    /**********************************/

    /* Allocate memory for various arrays */
    ht = N_AllocInt1dArray( m, &SL );

    if( ht EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= m; i++ )
        ht[i] = hi[i] + 1;

    hp = N_AllocIntPtr1dArray( m, &SL );

    if( hp EQ NULL )
        NL_QUIT;

    x = N_AllocRealPtr3dArray( m, &SL );
    y = N_AllocRealPtr3dArray( m, &SL );

    if( x EQ NULL )
        NL_QUIT;

    if( y EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= m; i++ )
    {
        hp[i] = N_AllocInt1dArray( ht[i], &SL );

        if( hp[i]EQ NULL )
            NL_QUIT;

        x[i] = N_AllocRealPtr1dArray( ht[i], &SL );
        y[i] = N_AllocRealPtr1dArray( ht[i], &SL );

        if( x[i]EQ NULL )
            NL_QUIT;

        if( y[i]EQ NULL )
            NL_QUIT;

        for ( j = 0; j <= ht[i]; j++ )
        {
            x[i][j] = N_AllocReal1dArray( nxy + 1, &SL );
            y[i][j] = N_AllocReal1dArray( nxy + 1, &SL );

            if( x[i][j]EQ NULL )
                NL_QUIT;

            if( y[i][j]EQ NULL )
                NL_QUIT;
        }
    }

    /* For each outer loop, polygonize inner and outer loops */

    for ( i = 0; i <= m; i++ )
    {
        /* Initialize first point of polygon */

        N_CrvGetCPts( cuo[i][0], &nc, &Pw );
        N_CPtToPtEuclid( Pw[0], &A );
        N_PtToXYZ( A, &x[i][0][0], &y[i][0][0], &z );

        error = N_CrvDecomposeContinuity( cuo[i][0], &seg, &is, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( is GT 0 ) /* eliminate degenerate segment at start */
        {
            N_CrvGetCPts( seg[0], &nnn, &Qw );
            N_DistCPolygon( Qw, nnn, &len );

            if( len LE 0.1 *tol )
            {
                for ( j = 1; j <= is; j++ )
                    seg[j - 1] = seg[j];
                is -= 1;
            }
        }

        if( ST_TessIsOnBoundary( seg[0], tol, xl, xr, yb, yt, &side, &scor, &ecor ) )
        {
            switch( side )
            {
                case NL_LEFT:
                    if( scor EQ NL_UPPERLEFT )
                    {
                        x[i][0][0] -= off;
                        y[i][0][0] += off;
                    }
                    break;

                case NL_BOTTOM:
                    if( scor EQ NL_LOWERLEFT )
                    {
                        x[i][0][0] -= off;
                        y[i][0][0] -= off;
                    }
                    break;

                case NL_RIGHT:
                    if( scor EQ NL_LOWERRIGHT )
                    {
                        x[i][0][0] += off;
                        y[i][0][0] -= off;
                    }
                    break;

                case NL_TOP:
                    if( scor EQ NL_UPPERRIGHT )
                    {
                        x[i][0][0] += off;
                        y[i][0][0] += off;
                    }
                    break;
            }
        }

        /* Polygonize each outer segment */

        np = 0;
        kxy = nxy; /* current size of x[][] and y[][] arrays */

        for ( j = 0; j <= ho[i]; j++ )
        {
            if( j GT 0 )
            {
                error = N_CrvDecomposeContinuity( cuo[i][j], &seg, &is, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            for ( ks = 0; ks <= is; ks++ )
            {
                if( ST_TessIsOnBoundary( seg[ks], tol, xl, xr, yb, yt, &side, &scor, &ecor ) )
                {
                    /* Segment lies along the boundary */

                    N_CrvGetCPts( seg[ks], &nc, &Pw );

                    N_CPtToPtEuclid( Pw[0], &A );
                    N_PtToXYZ( A, &ax, &ay, &z );
                    N_CPtToPtEuclid( Pw[nc], &B );
                    N_PtToXYZ( B, &bx, &by, &z );

                    if( np + 3 GT kxy )
                    {
                        kk = 3 * (is - ks + 1);
                        error = N_Realloc1dRealArray( &x[i][0], np, kxy + kk + 1, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;
                        error = N_Realloc1dRealArray( &y[i][0], np, kxy + kk + 1, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;
                        kxy += kk;
                    }

                    switch( side )
                    {
                        case NL_LEFT:
                            if( scor EQ NL_UPPERLEFT AND ecor EQ NL_LOWERLEFT )
                            {
                                if( x[i][0][np]GE xl - tol )
                                {
                                    x[i][0][np + 1] = ax - off;
                                    y[i][0][np + 1] = ay;
                                    np++;
                                }
                                x[i][0][np + 1] = bx - off;
                                y[i][0][np + 1] = by - off;
                                np++;
                            }
                            else if( scor EQ NL_UPPERLEFT )
                            {
                                if( x[i][0][np]GE xl - tol )
                                {
                                    x[i][0][np + 1] = ax - off;
                                    y[i][0][np + 1] = ay;
                                    np++;
                                }
                                x[i][0][np + 1] = bx - off;
                                y[i][0][np + 1] = by;
                                x[i][0][np + 2] = bx;
                                y[i][0][np + 2] = by;
                                np += 2;
                            }
                            else if( ecor EQ NL_LOWERLEFT )
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax - off AND y[i][0][np - 1]EQ ay )
                                        np -= 2;
                                x[i][0][np + 1] = ax - off;
                                y[i][0][np + 1] = ay;
                                x[i][0][np + 2] = bx - off;
                                y[i][0][np + 2] = by - off;
                                np += 2;
                            }
                            else
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax - off AND y[i][0][np - 1]EQ ay )
                                        np -= 2;
                                x[i][0][np + 1] = ax - off;
                                y[i][0][np + 1] = ay;
                                x[i][0][np + 2] = bx - off;
                                y[i][0][np + 2] = by;
                                x[i][0][np + 3] = bx;
                                y[i][0][np + 3] = by;
                                np += 3;
                            }
                            break;

                        case NL_BOTTOM:
                            if( scor EQ NL_LOWERLEFT AND ecor EQ NL_LOWERRIGHT )
                            {
                                if( y[i][0][np]GE yb - tol )
                                {
                                    x[i][0][np + 1] = ax;
                                    y[i][0][np + 1] = ay - off;
                                    np++;
                                }
                                x[i][0][np + 1] = bx + off;
                                y[i][0][np + 1] = by - off;
                                np++;
                            }
                            else if( scor EQ NL_LOWERLEFT )
                            {
                                if( y[i][0][np]GE yb - tol )
                                {
                                    x[i][0][np + 1] = ax;
                                    y[i][0][np + 1] = ay - off;
                                    np++;
                                }
                                x[i][0][np + 1] = bx;
                                y[i][0][np + 1] = by - off;
                                x[i][0][np + 2] = bx;
                                y[i][0][np + 2] = by;
                                np += 2;
                            }
                            else if( ecor EQ NL_LOWERRIGHT )
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax AND y[i][0][np - 1]EQ ay - off )
                                        np -= 2;
                                x[i][0][np + 1] = ax;
                                y[i][0][np + 1] = ay - off;
                                x[i][0][np + 2] = bx + off;
                                y[i][0][np + 2] = by - off;
                                np += 2;
                            }
                            else
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax AND y[i][0][np - 1]EQ ay - off )
                                        np -= 2;
                                x[i][0][np + 1] = ax;
                                y[i][0][np + 1] = ay - off;
                                x[i][0][np + 2] = bx;
                                y[i][0][np + 2] = by - off;
                                x[i][0][np + 3] = bx;
                                y[i][0][np + 3] = by;
                                np += 3;
                            }
                            break;

                        case NL_RIGHT:
                            if( scor EQ NL_LOWERRIGHT AND ecor EQ NL_UPPERRIGHT )
                            {
                                if( x[i][0][np]LE xr + tol )
                                {
                                    x[i][0][np + 1] = ax + off;
                                    y[i][0][np + 1] = ay;
                                    np++;
                                }
                                x[i][0][np + 1] = bx + off;
                                y[i][0][np + 1] = by + off;
                                np++;
                            }
                            else if( scor EQ NL_LOWERRIGHT )
                            {
                                if( x[i][0][np]LE xr + tol )
                                {
                                    x[i][0][np + 1] = ax + off;
                                    y[i][0][np + 1] = ay;
                                    np++;
                                }
                                x[i][0][np + 1] = bx + off;
                                y[i][0][np + 1] = by;
                                x[i][0][np + 2] = bx;
                                y[i][0][np + 2] = by;
                                np += 2;
                            }
                            else if( ecor EQ NL_UPPERRIGHT )
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax + off AND y[i][0][np - 1]EQ ay )
                                        np -= 2;
                                x[i][0][np + 1] = ax + off;
                                y[i][0][np + 1] = ay;
                                x[i][0][np + 2] = bx + off;
                                y[i][0][np + 2] = by + off;
                                np += 2;
                            }
                            else
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax + off AND y[i][0][np - 1]EQ ay )
                                        np -= 2;
                                x[i][0][np + 1] = ax + off;
                                y[i][0][np + 1] = ay;
                                x[i][0][np + 2] = bx + off;
                                y[i][0][np + 2] = by;
                                x[i][0][np + 3] = bx;
                                y[i][0][np + 3] = by;
                                np += 3;
                            }
                            break;

                        case NL_TOP:
                            if( scor EQ NL_UPPERRIGHT AND ecor EQ NL_UPPERLEFT )
                            {
                                if( y[i][0][np]LE yt + tol )
                                {
                                    x[i][0][np + 1] = ax;
                                    y[i][0][np + 1] = ay + off;
                                    np++;
                                }
                                x[i][0][np + 1] = bx - off;
                                y[i][0][np + 1] = by + off;
                                np++;
                            }
                            else if( scor EQ NL_UPPERRIGHT )
                            {
                                if( y[i][0][np]LE yt + tol )
                                {
                                    x[i][0][np + 1] = ax;
                                    y[i][0][np + 1] = ay + off;
                                    np++;
                                }
                                x[i][0][np + 1] = bx;
                                y[i][0][np + 1] = by + off;
                                x[i][0][np + 2] = bx;
                                y[i][0][np + 2] = by;
                                np += 2;
                            }
                            else if( ecor EQ NL_UPPERLEFT )
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax AND y[i][0][np - 1]EQ ay + off )
                                        np -= 2;
                                x[i][0][np + 1] = ax;
                                y[i][0][np + 1] = ay + off;
                                x[i][0][np + 2] = bx - off;
                                y[i][0][np + 2] = by + off;
                                np += 2;
                            }
                            else
                            {
                                if( np GT 0 )
                                    if( x[i][0][np - 1]EQ ax AND y[i][0][np - 1]EQ ay + off )
                                        np -= 2;
                                x[i][0][np + 1] = ax;
                                y[i][0][np + 1] = ay + off;
                                x[i][0][np + 2] = bx;
                                y[i][0][np + 2] = by + off;
                                x[i][0][np + 3] = bx;
                                y[i][0][np + 3] = by;
                                np += 3;
                            }
                            break;
                    }
                }
                else  /* else: Tess is Not on boundary. */
                {
                    /* Polygonize segment */

                    error = ST_TessPolygonizeTrimCrv( sur, lu, lv, seg[ks], epc, &ut, &vt, &nt, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( nt LT 0 )
                        NL_ERROR( NL_GEO_ERR );

                    if( nt EQ 0 )
                        if( ks EQ is )
                        {
                            if( np LT 3 )
                                NL_ERROR( NL_GEO_ERR );
                            N_CrvGetParamBounds( seg[ks], &d1, &d2 );
                            N_CrvEval( seg[ks], d2, NL_LEFT, &A );
                            N_PtToXYZ( A, &ut[1], &vt[1], &z );
                            np -= 1;
                            nt = 1;
                        }

                    if( np + nt GT kxy )
                    {
                        kk = nt * (is - ks + 1);
                        error = N_Realloc1dRealArray( &x[i][0], np, kxy + kk + 1, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;
                        error = N_Realloc1dRealArray( &y[i][0], np, kxy + kk + 1, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;
                        kxy += kk;
                    }

                    for ( k = 1; k <= nt; k++ )
                    {
                        x[i][0][np + k] = ut[k];
                        y[i][0][np + k] = vt[k];
                    }

                    np += nt;

                    N_FreeReal1dArray( ut, &SL );
                    N_FreeReal1dArray( vt, &SL );
                }
            }
        }

        if( np LE 2 )
            NL_ERROR( NL_TOL_ERR );

        /* Check for closure of outer loop */

        d1 = x[i][0][0] - x[i][0][np];
        d2 = y[i][0][0] - y[i][0][np];

        if( sqrt( d1 *d1 + d2 *d2 )GT tol )
        {                             /* close it up */
            np += 1;
            x[i][0][np] = x[i][0][0]; /* memory already there */
            y[i][0][np] = y[i][0][0];
        }

        hp[i][0] = np;

        /* Now polygonize all inner loops residing inside the i - th outer loop*/

        for ( j = 0; j <= hi[i]; j++ )
        {
            np = 0;
            kxy = nxy; /* current size of x[][] and y[][] arrays */

            N_CrvGetCPts( cui[i][j][0], &nc, &Pw );
            N_CPtToPtEuclid( Pw[0], &A );
            N_PtToXYZ( A, &x[i][j + 1][0], &y[i][j + 1][0], &z );

            for ( k = 0; k <= hs[i][j]; k++ )
            {
                error = ST_TessPolygonizeTrimCrv( sur, lu, lv, cui[i][j][k], epc, &ut, &vt, &nt, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                if( nt LE 0 )
                    NL_ERROR( NL_GEO_ERR );

                if( np + nt GT kxy )
                {
                    kk = nt * (hs[i][j] - k + 1);
                    error = N_Realloc1dRealArray( &x[i][j + 1], np, kxy + kk, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_Realloc1dRealArray( &y[i][j + 1], np, kxy + kk, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                    kxy += kk;
                }

                for ( l = 1; l <= nt; l++ )
                {
                    x[i][j + 1][np + l] = ut[l];
                    y[i][j + 1][np + l] = vt[l];
                }

                np += nt;

                N_FreeReal1dArray( ut, &SL );
                N_FreeReal1dArray( vt, &SL );
            }

            if( np LE 2 )
                NL_ERROR( NL_TOL_ERR );

            hp[i][j + 1] = np;
        }
    }

    /* Validate outer loop polygons */

    for ( i = 0; i <= m; i++ )
    {
        for ( k = 0; k <= hp[i][0]; k++ )
        {
            if( fabs( x[i][0][k] - xl )LE 7.0 *tol )
                x[i][0][k] = xl;

            if( fabs( x[i][0][k] - xr )LE 7.0 *tol )
                x[i][0][k] = xr;

            if( fabs( y[i][0][k] - yb )LE 7.0 *tol )
                y[i][0][k] = yb;

            if( fabs( y[i][0][k] - yt )LE 7.0 *tol )
                y[i][0][k] = yt;
        }

        for ( k = 1; k < hp[i][0]; k++ )
        {
            if( x[i][0][k]EQ xl )
                if( x[i][0][k - 1]LT xl )
                {
                    kk = k + 1;

                    while( kk LT hp[i][0]AND x[i][0][kk]EQ xl )
                        kk += 1;

                    if( x[i][0][kk]LT xl )
                    {
                        d1 = 0.5 *(x[i][0][k - 1] + x[i][0][kk]);

                        for ( kkk = k; kkk < kk; kkk++ )
                            x[i][0][kkk] = d1;
                        k = kk;
                    }
                }
        }

        for ( k = 1; k < hp[i][0]; k++ )
        {
            if( x[i][0][k]EQ xr )
                if( x[i][0][k - 1]GT xr )
                {
                    kk = k + 1;

                    while( kk LT hp[i][0]AND x[i][0][kk]EQ xr )
                        kk += 1;

                    if( x[i][0][kk]GT xr )
                    {
                        d1 = 0.5 *(x[i][0][k - 1] + x[i][0][kk]);

                        for ( kkk = k; kkk < kk; kkk++ )
                            x[i][0][kkk] = d1;
                        k = kk;
                    }
                }
        }

        for ( k = 1; k < hp[i][0]; k++ )
        {
            if( y[i][0][k]EQ yb )
                if( y[i][0][k - 1]LT yb )
                {
                    kk = k + 1;

                    while( kk LT hp[i][0]AND y[i][0][kk]EQ yb )
                        kk += 1;

                    if( y[i][0][kk]LT yb )
                    {
                        d1 = 0.5 *(y[i][0][k - 1] + y[i][0][kk]);

                        for ( kkk = k; kkk < kk; kkk++ )
                            y[i][0][kkk] = d1;
                        k = kk;
                    }
                }
        }

        for ( k = 1; k < hp[i][0]; k++ )
        {
            if( y[i][0][k]EQ yt )
                if( y[i][0][k - 1]GT yt )
                {
                    kk = k + 1;

                    while( kk LT hp[i][0]AND y[i][0][kk]EQ yt )
                        kk += 1;

                    if( y[i][0][kk]GT yt )
                    {
                        d1 = 0.5 *(y[i][0][k - 1] + y[i][0][kk]);

                        for ( kkk = k; kkk < kk; kkk++ )
                            y[i][0][kkk] = d1;
                        k = kk;
                    }
                }
        }
    }

    /**************************************************/
    /* Rescale the polygons and subdivide the surface */
    /**************************************************/

    UL = N_AllocRealPtr1dArray( m, &SL );
    UR = N_AllocRealPtr1dArray( m, &SL );

    if( UL EQ NULL )
        NL_OUT;

    if( UR EQ NULL )
        NL_OUT;

    VB = N_AllocRealPtr1dArray( m, &SL );
    VT = N_AllocRealPtr1dArray( m, &SL );

    if( VB EQ NULL )
        NL_OUT;

    if( VT EQ NULL )
        NL_OUT;

    for ( i = 0; i <= m; i++ )
    {
        UL[i] = N_AllocReal1dArray( ht[i], &SL );
        UR[i] = N_AllocReal1dArray( ht[i], &SL );

        if( UL[i]EQ NULL )
            NL_OUT;

        if( UR[i]EQ NULL )
            NL_OUT;

        VB[i] = N_AllocReal1dArray( ht[i], &SL );
        VT[i] = N_AllocReal1dArray( ht[i], &SL );

        if( VB[i]EQ NULL )
            NL_OUT;

        if( VT[i]EQ NULL )
            NL_OUT;
    }

    tol *= NL_MAX( lu, lv );
    d1 = lu + 0.001 *tol;
    d2 = lv + 0.001 *tol;

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= ht[i]; j++ )
        {
            UL[i][j] = VB[i][j] = NL_BIGD;
            UR[i][j] = VT[i][j] = -NL_BIGD;

            for ( k = 0; k <= hp[i][j]; k++ )
            {
                x[i][j][k] = usl * (x[i][j][k] - xl);
                y[i][j][k] = vsl * (y[i][j][k] - yb);

                if( x[i][j][k]LT 0.0 )
                    x[i][j][k] = -3.1 *tol;

                if( x[i][j][k]GT d1 )
                    x[i][j][k] = lu + 3.1 *tol;

                if( y[i][j][k]LT 0.0 )
                    y[i][j][k] = -3.1 *tol;

                if( y[i][j][k]GT d2 )
                    y[i][j][k] = lv + 3.1 *tol;

                if( x[i][j][k]LT UL[i][j] )
                    UL[i][j] = x[i][j][k];

                if( x[i][j][k]GT UR[i][j] )
                    UR[i][j] = x[i][j][k];

                if( y[i][j][k]LT VB[i][j] )
                    VB[i][j] = y[i][j][k];

                if( y[i][j][k]GT VT[i][j] )
                    VT[i][j] = y[i][j][k];
            }
        }
    }

    error = ST_TessSubdivideSrf( sur, eps, x, y, ht, hp, m, UL, UR, VB, VT, tmp, lu, lv, tol, u, v, &R, &T, n, &TM, &SM, &r, &s, &SL, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /***********************************************************************/
    /* For closed surfaces, match subdivision points along common boundary */
    /***********************************************************************/

    if( N_SrfIsClosed( sur, NL_UDIR ) )
        ucl = NL_YES;

    if( N_SrfIsClosed( sur, NL_VDIR ) )
        vcl = NL_YES;

    if( ucl EQ NL_YES OR vcl EQ NL_YES )
    {
        /* See how many points to add */

        ul = *u;
        vl = *v;
        nl = *n;
        ni = 0;

        if( ucl EQ NL_YES )
        {
            for ( j = 0; j <= s; j++ )
            {
                a = TM[0][j];
                b = TM[r][j];

                if( (a LT 0 AND b GE 0)OR( a GE 0 AND b LT 0 ) )
                {
                    ni++;
                    uin = NL_YES;
                }
            }
        }

        if( vcl EQ NL_YES )
        {
            for ( i = 0; i <= r; i++ )
            {
                a = TM[i][0];
                b = TM[i][s];

                if( (a LT 0 AND b GE 0)OR( a GE 0 AND b LT 0 ) )
                {
                    ni++;
                    vin = NL_YES;
                }
            }
        }

        if( ni GT 0 )
        {
            /* Reallocate memory */

            error = N_Realloc1dRealArray( &ul, nl, nl + ni, SG );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &vl, nl, nl + ni, SG );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dFlagArray( &R, nl, nl + ni, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dFlagArray( &T, nl, nl + ni, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Insert new points */

            if( ucl EQ NL_YES AND uin EQ NL_YES )
            {
                for ( j = 0; j <= s; j++ )
                {
                    a = TM[0][j];
                    b = TM[r][j];

                    if( a LT 0 AND b GE 0 )
                    {
                        TM[0][j] = nl + 1;
                        ul[nl + 1] = 0.0;
                        vl[nl + 1] = vl[b];
                        SM[0][j] = NL_NOSTATUS;
                        R[nl + 1] = NL_NO;
                        T[nl + 1] = NL_NO;

                        nl++;
                    }

                    if( a GE 0 AND b LT 0 )
                    {
                        TM[r][j] = nl + 1;
                        ul[nl + 1] = lu;
                        vl[nl + 1] = vl[a];
                        SM[r][j] = NL_NOSTATUS;
                        R[nl + 1] = NL_NO;
                        T[nl + 1] = NL_NO;

                        nl++;
                    }
                }
            }

            if( vcl EQ NL_YES AND vin EQ NL_YES )
            {
                for ( i = 0; i <= r; i++ )
                {
                    a = TM[i][0];
                    b = TM[i][s];

                    if( a LT 0 AND b GE 0 )
                    {
                        TM[i][0] = nl + 1;
                        ul[nl + 1] = ul[b];
                        vl[nl + 1] = 0.0;
                        SM[i][0] = NL_NOSTATUS;
                        R[nl + 1] = NL_NO;
                        T[nl + 1] = NL_NO;

                        nl++;
                    }

                    if( a GE 0 AND b LT 0 )
                    {
                        TM[i][s] = nl + 1;
                        ul[nl + 1] = ul[a];
                        vl[nl + 1] = lv;
                        SM[i][s] = NL_NOSTATUS;
                        R[nl + 1] = NL_NO;
                        T[nl + 1] = NL_NO;

                        nl++;
                    }
                }
            }
        }

        *u = ul;
        *v = vl;
        *n = nl;
    }

    /* Now triangulate the subdivided domain */

    N_SrfGetParameterBounds( sur, &xl, &xr, &yb, &yt ); /* ST_IsectLineWith2dPolygon needs these globals */
    /* No longer using globals: */
    /* NL_TES_UMIN = xl; */
    /* NL_TES_UMAX = xr; */
    /* NL_TES_VMIN = yb; */
    /* NL_TES_VMAX = yt; */

    error = ST_TessRegionOfRectangles( u, v, n, R, T, TM, SM, r, s, x, y, ht, hp, m, UL, UR, VB, VT, xl, xr, yb, yt, tol, DT, hd, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Check that tessellation points not out of valid region */

    if( m EQ 0 )
    {
        tmp = 1.2 *tol;

        d1 = UL[0][0] - tmp;
        d2 = UR[0][0] + tmp;
        d3 = VB[0][0] - tmp;
        d4 = VT[0][0] + tmp;

        j = *n;
        ul = *u;
        vl = *v;

        for ( i = 0; i <= j; i++ )
            if( ( *DT)[i][0]NEQ - 1 )
            {
                if( ul[i]LT d1 OR ul[i]GT d2 )
                    NL_ERROR( NL_GEO_ERR );

                if( vl[i]LT d3 OR vl[i]GT d4 )
                    NL_ERROR( NL_GEO_ERR );
            }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* ST_TessIsOnBoundary: Check if segment lies along boundary          */
/**********************************************************************/

static NL_FLAG ST_TessIsOnBoundary( NL_CURVE *cur, NL_REAL tol, NL_PARAMETER ul, NL_PARAMETER ur, NL_PARAMETER vb, NL_PARAMETER vt, NL_FLAG *side, NL_FLAG *scor, NL_FLAG *ecor )
{
    NL_FLAG bnd = NL_NO;

    NL_INDEX n;

    NL_REAL ax, ay, bx, by, z;

    NL_CPOINT *Pw;

    NL_POINT A, B;

    if( NOT N_CrvIsLine( cur, tol ) )
        NL_OUT;

    N_CrvGetCPts( cur, &n, &Pw );
    N_CPtToPtEuclid( Pw[0], &A );
    N_CPtToPtEuclid( Pw[n], &B );
    N_PtToXYZ( A, &ax, &ay, &z );
    N_PtToXYZ( B, &bx, &by, &z );

    *scor = NL_NOCORNER;
    *ecor = NL_NOCORNER;

    if( fabs( ax - ul )LT tol AND fabs( bx - ul )LT tol )
    {
        bnd = NL_YES;
        *side = NL_LEFT;

        if( fabs( ay - vt )LT tol )
            *scor = NL_UPPERLEFT;

        if( fabs( by - vb )LT tol )
            *ecor = NL_LOWERLEFT;
    }

    if( fabs( ax - ur )LT tol AND fabs( bx - ur )LT tol )
    {
        bnd = NL_YES;
        *side = NL_RIGHT;

        if( fabs( ay - vb )LT tol )
            *scor = NL_LOWERRIGHT;

        if( fabs( by - vt )LT tol )
            *ecor = NL_UPPERRIGHT;
    }

    if( fabs( ay - vb )LT tol AND fabs( by - vb )LT tol )
    {
        bnd = NL_YES;
        *side = NL_BOTTOM;

        if( fabs( ax - ul )LT tol )
            *scor = NL_LOWERLEFT;

        if( fabs( bx - ur )LT tol )
            *ecor = NL_LOWERRIGHT;
    }

    if( fabs( ay - vt )LT tol AND fabs( by - vt )LT tol )
    {
        bnd = NL_YES;
        *side = NL_TOP;

        if( fabs( ax - ur )LT tol )
            *scor = NL_UPPERRIGHT;

        if( fabs( bx - ul )LT tol )
            *ecor = NL_UPPERLEFT;
    }

    EXIT:

    return (bnd);
}

/**********************************************************************/
/* N_TessGetTriangleEdges: Return all edges forming edges in a triangulation        */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and  returns all  edges  that  form edges  of  triangles. A typical 
     calling example is:
 
       NL_INDEX   **DT, *hd, *alf, *bet, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd);
       ...
       N_TessGetTriangleEdges(DT, hd, m, &alf, &bet, &n, &SG);
 
     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., bet[n] IS ALLOCATED 
     INSIDE THE ROUTINE.
 
 
   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     hd  , input  ,  hd[i] is the highest index in the list DT[i]
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     alf , output ,  Indexes of start points of all edges
     bet , output ,  Indexes of end points of all edges
     n   , output ,  Highest index in alf and bet
     SG  , input  ,  Memory stack for alf and bet
                       
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
***********************************************************************/

NL_FLAG N_TessGetTriangleEdges( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX m, NL_INDEX ** alf, NL_INDEX ** bet, NL_INDEX *n, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *al, *bl, *ao, *bo, i, j, r, t;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Allocate local memory */

    al = N_AllocInt1dArray( 4 * m, &SL );

    if( al EQ NULL )
        NL_QUIT;

    bl = N_AllocInt1dArray( 4 * m, &SL );

    if( bl EQ NULL )
        NL_QUIT;

    /* Find all edges */

    t = -1;

    for ( i = 0; i <= m; i++ )
    {
        if( DT[i][0]NEQ - 1 )
        {
            for ( r = 0; r <= hd[i]; r++ )
            {
                j = DT[i][r];

                if( i LT j )
                {
                    t++;
                    al[t] = i;
                    bl[t] = j;
                }
            }
        }
    }

    /* Output edges */

    ao = N_AllocInt1dArray( t, SG );

    if( ao EQ NULL )
        NL_QUIT;

    bo = N_AllocInt1dArray( t, SG );

    if( bo EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= t; i++ )
    {
        ao[i] = al[i];
        bo[i] = bl[i];
    }

    *alf = ao;
    *bet = bo;
    *n = t;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
/**********************************************************************/
/* N_TessGetTriangleVertices: Return all points forming vertices in a triangulation    */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and  returns all  points that form vertices of triangles. A typical 
     calling example is:

       NL_INDEX   **DT, *alf, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd);
       ...
       N_TessGetTriangleVertices(DT, m, &alf, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., alf[n] IS ALLOCATED 
     INSIDE THE ROUTINE.


   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     alf , output ,  Indexes of all points
     n   , output ,  Highest index in alf
     SG  , input  ,  Memory stack for alf
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessGetTriangleVertices( NL_INDEX ** DT, NL_INDEX m, NL_INDEX ** alf, NL_INDEX *n, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *al, i, j;

    /* Get number of points */

    j = -1;

    for ( i = 0; i <= m; i++ )
    {
        if( DT[i][0]NEQ - 1 )
            j++;
    }

    /* Allocate memory and output points */

    al = N_AllocInt1dArray( j, SG );

    if( al EQ NULL )
        NL_QUIT;

    j = -1;

    for ( i = 0; i <= m; i++ )
    {
        if( DT[i][0]NEQ - 1 )
            al[++j] = i;
    }

    *alf = al;
    *n = j;

    /* Exit */

    EXIT:

    return (error);
}

/**********************************************************************/
/* N_TessGetTriangles: Return all triangles in a surface triangulation          */
/**********************************************************************/
/* THIS IS USER CALLABLE

   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and returns all triangles. A typical calling example is:

       NL_INDEX   **DT, *hd, *alf, *bet, *gam, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd);
       ...
       N_TessGetTriangles(DT, hd, m, &alf, &bet, &gam, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., gam[n] IS ALLOCATED 
     INSIDE THE ROUTINE. INDEXES IN EACH RETURNED TRIANGLE ARE LISTED IN
     COUNTERCLOCKWISE ORDER.


   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     hd  , input  ,  hd[i] is the highest index in the list DT[i]
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     alf , output ,  Indexes of first points
     bet , output ,  Indexes of second points
     gam , output ,  Indexes of third points
     n   , output ,  Highest index in alf, bet and gam
     SG  , input  ,  Memory stack for alf, bet and gam
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessGetTriangles( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX m, NL_INDEX ** alf, NL_INDEX ** bet, NL_INDEX ** gam, NL_INDEX *n, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *al, *bl, *gl, *ao, *bo, *go, i, j, k, r, t;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Allocate local memory */

    k = 2 * m + 1;
    al = N_AllocInt1dArray( 3 * k, &SL );

    if( al EQ NULL )
        NL_QUIT;

    bl = &al[k];
    gl = &bl[k];

    /* Find all triangles */

    t = -1;

    for ( i = 0; i <= m; i++ )
    {
        if( DT[i][0]NEQ - 1 )
        {
            for ( r = 0; r < hd[i]; r++ )
            {
                j = DT[i][r];
                k = DT[i][r + 1];

                if( i LT j AND i LT k )
                {
                    t++;
                    al[t] = i;
                    bl[t] = j;
                    gl[t] = k;
                }
            }
        }
    }

    /* Output triangles */

    ao = N_AllocInt1dArray( t, SG );

    if( ao EQ NULL )
        NL_QUIT;

    bo = N_AllocInt1dArray( t, SG );

    if( bo EQ NULL )
        NL_QUIT;

    go = N_AllocInt1dArray( t, SG );

    if( go EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= t; i++ )
    {
        ao[i] = al[i];
        bo[i] = bl[i];
        go[i] = gl[i];
    }

    *alf = ao;
    *bet = bo;
    *gam = go;
    *n = t;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
/**********************************************************************/
/* N_TESSISINSIDELOOP: Check if rectangle is inside a trimming loop             */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     Given a trimmed parametric domain, this routine checks if a given
     rectangle is  entirely IN or NL_OUT, or it intersects the region, or 
     some of the trimming polygons are entirely in the rectangle. This
     version checks the box with respect to one set of trimming curves
     bounded by one outer loop. A typical calling example is:

       NL_REAL   **u, **v, *UL, *UR, *VB, *VT, xl, xr, yb, yt, tol;
       NL_INDEX  *h, n;
       ...
(get u,..., tol);
       ...
       if (ST_TessIsInsideLoop(u, v, h, n, UL, UR, VB, VT, xl, xr, yb, yt, tol) EQ NL_INP)  
           --> handle "in" rectangle

     ALL TRIMMING  POLYGONS MUST BE  CLOSED LOOPS! THE OUTER LOOP MUST 
     BE ORIENTED COUNTERCLOCKWISE  WHEREAS ALL THE INNER  ONES MUST BE
     ORIENTED CLOCKWISE.


   ACCESS:
   
     u, v         , input ,  Vertices of trimming polygons; u[i], v[i]
                            are  pointers to the  i - th polygon. u[0],
                            v[0] ARE POINTERS TO THE OUTER LOOP!
     h           , input ,  h[i] is  the  highest index  in the array
                            pointed to by u[i] and v[i]
     n           , input ,  Highest index in(u, v) and  h, i.e. there
                            are n + 1 trimming curves/polygons
     UL, UR, VB, VT , input ,  Bounding boxes of(u[i], v[i])
     xl, xr, yb, yt , input ,  Given rectangle
     tol         , input ,  Parameter space tolerance
                       

   RETURN CODES:

     NL_ONANDOVER: Some of the trimming polygons are entirely  inside the
                rectangle and some others intersect it
     NL_OVER     : Some of the trimming polygons are entirely  inside the 
                rectangle
     NL_INP      : Rectangle is inside the trimming region
     NL_ONP      : Rectangle intersects one of the polygons
     NL_OUTP     : Rectangle is out of the trimming region

   ***********************************************************************/

static NL_FLAG ST_TessIsInsideLoop( NL_REAL ** u, NL_REAL ** v, NL_INDEX *h, NL_INDEX n, NL_REAL *UL, NL_REAL *UR, NL_REAL *VB, NL_REAL *VT, NL_REAL xl, NL_REAL xr, NL_REAL yb, NL_REAL yt, NL_REAL tol )
{
    NL_FLAG status, on, over, bstat;

    NL_INDEX i;

    /* Check box with respect to the loop */

    on = over = NL_NO;

    status = ST_TessIsInsidePolygon( u[0], v[0], h[0], UL[0], UR[0], VB[0], VT[0], xl, xr, yb, yt, tol );

    if( status EQ NL_OUTP )
    {
        bstat = NL_OUTP;
        NL_OUT;
    }

    if( status EQ NL_ONP )
        on = NL_YES;

    if( status EQ NL_OVER )
        over = NL_YES;

    for ( i = 1; i <= n; i++ )
    {
        status = ST_TessIsInsidePolygon( u[i], v[i], h[i], UL[i], UR[i], VB[i], VT[i], xl, xr, yb, yt, tol );

        if( status EQ NL_INP )
        {
            bstat = NL_OUTP;
            NL_OUT;
        }

        if( status EQ NL_ONP )
            on = NL_YES;

        if( status EQ NL_OVER )
            over = NL_YES;
    }

    /* Determine box status */

    if( on EQ NL_YES )
    {
        if( over EQ NL_YES )
            bstat = NL_ONANDOVER;
        else
            bstat = NL_ONP;
    }

    else if( over EQ NL_YES )
        bstat = NL_OVER;

    else
        bstat = NL_INP;

    EXIT:

    return (bstat);
}

/**********************************************************************/
/* ST_TessIsInsidePolygon: Check if rectangle is in a given polgon                  */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     Given a closed  polygon and a rectangle, this routine checks if the
     rectangle is  entirely IN  or NL_OUT, or it intersects the polygon, or
     the polygon is entirely in the rectangle. A typical calling example 
     is:

       NL_REAL   *u, *v, UL, UR, VB, VT, xl, xr, yb, yt, tol;
       NL_INDEX  n;
       ...
(get u,..., tol);
       ...
       if (ST_TessIsInsidePolygon(u, v, n, UL, UR, VB, VT, xl, xr, yb, yt, tol) EQ NL_INP)  
           --> handle "in" rectangle


   ACCESS:
   
     u, v         , input ,  Vertices of polygon
     n           , input ,  Highest index in(u, v)
     UL, UR, VB, VT , input ,  Bounding box of(u, v)
     xl, xr, yb, yt , input ,  Given rectangle
     tol         , input ,  Parameter space tolerance
                       

   RETURN CODES:

     NL_OVER: Polygon is entirely inside the rectangle
     NL_INP : Rectangle is in the polygon
     NL_ONP : Rectangle intersects the polygon
     NL_OUTP: Rectangle is out of the polygon

   ***********************************************************************/

static NL_FLAG ST_TessIsInsidePolygon( NL_REAL *u, NL_REAL *v, NL_INDEX n, NL_REAL UL, NL_REAL UR, NL_REAL VB, NL_REAL VT, NL_REAL xl, NL_REAL xr, NL_REAL yb, NL_REAL yt, NL_REAL tol )
{
    NL_FLAG its, ovr, lb, rb, lt, rt;

    NL_INDEX i, klb, krb, klt, krt;

    NL_REAL xb, xt, xlm, xlp, xrm, xrp, ybm, ybp, ytm, ytp;

    /* Quick box test */

    xlm = xl - tol;
    xlp = xl + tol;
    xrm = xr - tol;
    xrp = xr + tol;
    ybm = yb - tol;
    ybp = yb + tol;
    ytm = yt - tol;
    ytp = yt + tol;

    if( xrp LT UL - tol OR xlm GT UR + tol OR ytp LT VB - tol OR ybm GT VT + tol )
    {
        return NL_OUTP;
    }

    /* Check overlapping */

    ovr = NL_YES;

    for ( i = 0; i <= n; i++ )
    {
        if( u[i]LT xlp OR u[i]GT xrm OR v[i]LT ybp OR v[i]GT ytm )
            ovr = NL_NO;
    }

    if( ovr EQ NL_YES )
        return NL_OVER;

    /* Check each vertex with respect to the polygon */

    klb = 0;
    krb = 0;
    klt = 0;
    krt = 0;
    lb = 5;
    rb = 5;
    lt = 5;
    rt = 5;

    for ( i = 1; i <= n; i++ )
    {
        /* Check vertical and horizontal line segments */

        if( fabs( v[i - 1] - yb )LT tol AND fabs( v[i] - yb )LT tol )
        {
            if( ((xl GT u[i - 1] - tol)AND( xl LT u[i] + tol ))OR( (xl GT u[i] - tol)AND( xl LT u[i - 1] + tol ) ) )
                lb = NL_ONP;

            if( ((xr GT u[i - 1] - tol)AND( xr LT u[i] + tol ))OR( (xr GT u[i] - tol)AND( xr LT u[i - 1] + tol ) ) )
                rb = NL_ONP;
        }

        if( fabs( u[i - 1] - xr )LT tol AND fabs( u[i] - xr )LT tol )
        {
            if( ((yb GT v[i - 1] - tol)AND( yb LT v[i] + tol ))OR( (yb GT v[i] - tol)AND( yb LT v[i - 1] + tol ) ) )
                rb = NL_ONP;

            if( ((yt GT v[i - 1] - tol)AND( yt LT v[i] + tol ))OR( (yt GT v[i] - tol)AND( yt LT v[i - 1] + tol ) ) )
                rt = NL_ONP;
        }

        if( fabs( v[i - 1] - yt )LT tol AND fabs( v[i] - yt )LT tol )
        {
            if( ((xl GT u[i - 1] - tol)AND( xl LT u[i] + tol ))OR( (xl GT u[i] - tol)AND( xl LT u[i - 1] + tol ) ) )
                lt = NL_ONP;

            if( ((xr GT u[i - 1] - tol)AND( xr LT u[i] + tol ))OR( (xr GT u[i] - tol)AND( xr LT u[i - 1] + tol ) ) )
                rt = NL_ONP;
        }

        if( fabs( u[i - 1] - xl )LT tol AND fabs( u[i] - xl )LT tol )
        {
            if( ((yb GT v[i - 1] - tol)AND( yb LT v[i] + tol ))OR( (yb GT v[i] - tol)AND( yb LT v[i - 1] + tol ) ) )
                lb = NL_ONP;

            if( ((yt GT v[i - 1] - tol)AND( yt LT v[i] + tol ))OR( (yt GT v[i] - tol)AND( yt LT v[i - 1] + tol ) ) )
                lt = NL_ONP;
        }

        /* Check in/out status by ray shooting */

        if( lb NEQ NL_ONP OR rb NEQ NL_ONP )
        {
            if( ((v[i]GT ybp)AND( (v[i - 1]LT ybm)OR( fabs( v[i - 1] - yb )LT tol ) ))OR( (v[i - 1]GT ybp)AND( (v[i]LT ybm)OR( fabs( v[i] - yb )LT tol ) ) ) )
            {
                xb = u[i] + ((yb - v[i]) * (u[i] - u[i - 1])) / (v[i] - v[i - 1]);

                if( lb NEQ NL_ONP )
                {
                    if( fabs( xb - xl )LT tol )
                        lb = NL_ONP;

                    else if( xb GT xl )
                        klb++;
                }

                if( rb NEQ NL_ONP )
                {
                    if( fabs( xb - xr )LT tol )
                        rb = NL_ONP;

                    else if( xb GT xr )
                        krb++;
                }
            }
        }

        if( lt NEQ NL_ONP OR rt NEQ NL_ONP )
        {
            if( ((v[i]GT ytp)AND( (v[i - 1]LT ytm)OR( fabs( v[i - 1] - yt )LT tol ) ))OR( (v[i - 1]GT ytp)AND( (v[i]LT ytm)OR( fabs( v[i] - yt )LT tol ) ) ) )
            {
                xt = u[i] + ((yt - v[i]) * (u[i] - u[i - 1])) / (v[i] - v[i - 1]);

                if( lt NEQ NL_ONP )
                {
                    if( fabs( xt - xl )LT tol )
                        lt = NL_ONP;

                    else if( xt GT xl )
                        klt++;
                }

                if( rt NEQ NL_ONP )
                {
                    if( fabs( xt - xr )LT tol )
                        rt = NL_ONP;

                    else if( xt GT xr )
                        krt++;
                }
            }
        }
    }

    /* Define status of corners */

    if( lb NEQ NL_ONP )
    {
        if( klb % 2 )
            lb = NL_INP;
        else
            lb = NL_OUTP;
    }

    if( rb NEQ NL_ONP )
    {
        if( krb % 2 )
            rb = NL_INP;
        else
            rb = NL_OUTP;
    }

    if( lt NEQ NL_ONP )
    {
        if( klt % 2 )
            lt = NL_INP;
        else
            lt = NL_OUTP;
    }

    if( rt NEQ NL_ONP )
    {
        if( krt % 2 )
            rt = NL_INP;
        else
            rt = NL_OUTP;
    }

    if( lb EQ NL_ONP OR rb EQ NL_ONP OR lt EQ NL_ONP OR rt EQ NL_ONP )
        return NL_ONP;

    /* Check intersection */

    its = NL_NO;

    for ( i = 1; i <= n; i++ )
    {
        /* See if segment might intersect box */

        if( NL_MAX( u[i - 1], u[i] ) + tol LT xlm OR NL_MIN( u[i - 1], u[i] ) - tol GT xrp )
            continue;

        if( NL_MAX( v[i - 1], v[i] ) + tol LT ybm OR NL_MIN( v[i - 1], v[i] ) - tol GT ytp )
            continue;

        /* See if segment straddles the sides of the box */

        if( ((u[i]GT xlm)AND( u[i - 1]LT xlp ))OR( (u[i - 1]GT xlm)AND( u[i]LT xlp ) ) )
        {
            if( fabs( u[i - 1] - xl )LT tol AND( v[i - 1]LT ytp AND v[i - 1]GT ybm ) )
            {
                its = NL_YES;
                break;
            }
            else if( fabs( u[i] - xl )LT tol AND( v[i]LT ytp AND v[i]GT ybm ) )
            {
                its = NL_YES;
                break;
            }
            else if( N_LineSegs2dAreIntersecting( u[i - 1], v[i - 1], u[i], v[i], xl, yb, xl, yt, tol ) )
            {
                its = NL_YES;
                break;
            }
        }

        if( ((u[i]GT xrm)AND( u[i - 1]LT xrp ))OR( (u[i - 1]GT xrm)AND( u[i]LT xrp ) ) )
        {
            if( (fabs( u[i - 1] - xr )LT tol)AND( (v[i - 1]LT ytp)AND( v[i - 1]GT ybm ) ) )
            {
                its = NL_YES;
                break;
            }
            else if( (fabs( u[i] - xr )LT tol)AND( (v[i]LT ytp)AND( v[i]GT ybm ) ) )
            {
                its = NL_YES;
                break;
            }
            else if( N_LineSegs2dAreIntersecting( u[i - 1], v[i - 1], u[i], v[i], xr, yb, xr, yt, tol ) )
            {
                its = NL_YES;
                break;
            }
        }

        if( ((v[i]GT ybm)AND( v[i - 1]LT ybp ))OR( (v[i - 1]GT ybm)AND( v[i]LT ybp ) ) )
        {
            if( (fabs( v[i - 1] - yb )LT tol)AND( (u[i - 1]LT xrp)AND( u[i - 1]GT xlm ) ) )
            {
                its = NL_YES;
                break;
            }
            else if( (fabs( v[i] - yb )LT tol)AND( (u[i]LT xrp)AND( u[i]GT xlm ) ) )
            {
                its = NL_YES;
                break;
            }
            else if( N_LineSegs2dAreIntersecting( u[i - 1], v[i - 1], u[i], v[i], xl, yb, xr, yb, tol ) )
            {
                its = NL_YES;
                break;
            }
        }

        if( ((v[i]GT ytm)AND( v[i - 1]LT ytp ))OR( (v[i - 1]GT ytm)AND( v[i]LT ytp ) ) )
        {
            if( (fabs( v[i - 1] - yt )LT tol)AND( u[i - 1]LT xrp AND u[i - 1]GT xlm ) )
            {
                its = NL_YES;
                break;
            }
            else if( (fabs( v[i] - yt )LT tol)AND( u[i]LT xrp AND u[i]GT xlm ) )
            {
                its = NL_YES;
                break;
            }
            else if( N_LineSegs2dAreIntersecting( u[i - 1], v[i - 1], u[i], v[i], xl, yt, xr, yt, tol ) )
            {
                its = NL_YES;
                break;
            }
        }
    }

    if( its EQ NL_YES )
    {
        return NL_ONP;
    }
    else if( lb EQ NL_INP AND rb EQ NL_INP AND lt EQ NL_INP AND rt EQ NL_INP )
    {
        return NL_INP;
    }
    else
    {
        return NL_OUTP;
    }
}

/***********************************************************************/
/* N_TESSISINSIDETRIM: Check if rectangle is inside the trimmed region */
/***********************************************************************/

/*******************************************************************//**

   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     Given a trimmed parametric domain, this routine checks if a given
     rectangle is  entirely IN or NL_OUT, or it intersects the region, or 
     some of the trimming  polygons are entirely  in the  rectangle. A 
     typical calling example is:

       NL_REAL   ***u, ***v, **UL, **UR, **VB, **VT, xl, xr, yb, yt, tol;
       NL_INDEX  **hp, *ht, n;
       ...
(get u,..., tol);
       ...
       if (ST_TessIsInsideTrim(u, v, ht, hp, n, UL, UR, VB, VT, xl, xr, yb, yt, tol) EQ NL_INP)  
           --> handle "in" rectangle

     ALL TRIMMING  POLYGONS MUST BE CLOSED LOOPS! THE OUTER LOOPS MUST 
     BE ORIENTED COUNTERCLOCKWISE  WHEREAS ALL THE INNER  ONES MUST BE
     ORIENTED CLOCKWISE. 


   ACCESS:
   
     u, v         , input ,  Vertices  of  trimming  polygons; u[i][j], 
                            v[i][j] are  pointers to the  j - th polygon
                            in the i - th  loop. For  j = 0  one  gets the
                            outer loop, whereas  for j = 1,..., ht[i] the
                            inner loops are obtained.
     ht          , input ,  ht[i] is the  highest index of polygons in 
                            the i - th outer loop.
     hp          , input ,  hp[i][j] is  the highest index of vertices
                            in the j - th  polygon  residing in the i - th
                            loop
     n           , input ,  Highest  index in  ht, i.e. there  are n + 1 
                            trimming loops
     UL, UR, VB, VT , input ,  Bounding boxes of(u[i][j], v[i][j])
     xl, xr, yb, yt , input ,  Given rectangle
     tol         , input ,  Parameter space tolerance
                       

   RETURN CODES:

     NL_ONANDOVER: Some  trimming   polygons  are   entirely  inside  the 
                rectangle and some others intersect it
     NL_OVER     : Some of the trimming polygons are entirely  inside the 
                rectangle
     NL_INP      : Rectangle is inside the trimming region
     NL_ONP      : Rectangle intersects one of the polygons
     NL_OUTP     : Rectangle is out of the trimming region

   ***********************************************************************/

static NL_FLAG ST_TessIsInsideTrim( NL_REAL *** u, NL_REAL *** v, NL_INDEX *ht, NL_INDEX ** hp, NL_INDEX n, NL_REAL ** UL, NL_REAL ** UR, NL_REAL ** VB, NL_REAL ** VT, NL_REAL xl, NL_REAL xr, NL_REAL yb, NL_REAL yt, NL_REAL tol )
{
    NL_FLAG status, on, over, in, bstat;

    NL_INDEX i, j;

    /* Check box with respect to all loops */

    in = on = over = NL_NO;

    for ( i = 0; i <= n; i++ )
    {
        status = ST_TessIsInsidePolygon( u[i][0], v[i][0], hp[i][0], UL[i][0], UR[i][0], VB[i][0], VT[i][0], xl, xr, yb, yt, tol );

        if( status EQ NL_INP )
            in = NL_YES;

        else if( status EQ NL_ONP )
            on = NL_YES;

        else if( status EQ NL_OVER )
            over = NL_YES;

        for ( j = 1; j <= ht[i]; j++ )
        {
            status = ST_TessIsInsidePolygon( u[i][j], v[i][j], hp[i][j], UL[i][j], UR[i][j], VB[i][j], VT[i][j], xl, xr, yb, yt, tol );

            if( status EQ NL_INP )
            {
                bstat = NL_OUTP;
                NL_OUT;
            }

            if( status EQ NL_ONP )
                on = NL_YES;

            else if( status EQ NL_OVER )
                over = NL_YES;
        }

        if( in EQ NL_YES )
            break;
    }

    /* Determine box status */

    if( on EQ NL_YES )
    {
        if( over EQ NL_YES )
            bstat = NL_ONANDOVER;
        else
            bstat = NL_ONP;
    }

    else if( over EQ NL_YES )
        bstat = NL_OVER;

    else if( in EQ NL_YES )
        bstat = NL_INP;

    else
        bstat = NL_OUTP;

    EXIT:

    return (bstat);
}
/**********************************************************************/
/* N_TESSISONEDGE: Find all triangles incident on a given edge              */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and  finds  all  triangles  incident  on a  given  edge. A  typical 
     calling example is:

       NL_INDEX   **DT, *hd, *alf, *bet, *gam, k, l, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd, and indexes k and l);
       ...
       N_TessIsOnEdge(DT, hd, m, k, l, &alf, &bet, &gam, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., gam[n] IS ALLOCATED 
     INSIDE THE ROUTINE. INDEXES IN EACH RETURNED TRIANGLE ARE LISTED IN
     COUNTERCLOCKWISE ORDER.


   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     hd  , input  ,  hd[i] is the highest index in the list DT[i]
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     k, l , input  ,  Indexes of given edge
     alf , output ,  Indexes of first points of triangles
     bet , output ,  Indexes of second points of triangles
     bet , output ,  Indexes of third points of triangles. The  returned 
                     triangles  are  <alf[0], bet[0], gam[0]>,..., < alf[n],
                     bet[n], gam[n]>. THERE ARE 1 OR 2 TRIANGLES.
     n   , output ,  Highest index in alf, bet and gam
     SG  , input  ,  Memory stack for alf, bet and gam
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessIsOnEdge( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_INDEX ** alf, NL_INDEX ** bet, NL_INDEX ** gam, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TessIsOnEdge");

    NL_FLAG error = NL_NO;

    NL_INDEX *al, *be, *ga, i, j, a, b, c;

    /* Check input */

    if( m LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR k GT m )
        NL_ERROR( NL_IND_ERR );

    if( l LT 0 OR l GT m )
        NL_ERROR( NL_IND_ERR );

    if( DT[k][0]EQ - 1 OR DT[l][0]EQ - 1 )
    {
        *n = -1;
        NL_OUT;
    }

    /* Get number of triangles */

    a = b = c = -1;

    j = -1;

    for ( i = 0; i <= hd[k]; i++ )
    {
        if( DT[k][i]EQ l )
        {
            j = i;
            break;
        }
    }

    if( j EQ - 1 )
        NL_ERROR( NL_IND_ERR );

    if( j LT hd[k] )
    {
        a = DT[k][j + 1];
        c++;
    }

    j = -1;

    for ( i = 0; i <= hd[l]; i++ )
    {
        if( DT[l][i]EQ k )
        {
            j = i;
            break;
        }
    }

    if( j EQ - 1 )
        NL_ERROR( NL_IND_ERR );

    if( j LT hd[l] )
    {
        b = DT[l][j + 1];
        c++;
    }

    if( c EQ - 1 )
        NL_ERROR( NL_IND_ERR );

    /* Allocate memory and output indexes */

    al = N_AllocInt1dArray( c, SG );

    if( al EQ NULL )
        NL_QUIT;

    be = N_AllocInt1dArray( c, SG );

    if( be EQ NULL )
        NL_QUIT;

    ga = N_AllocInt1dArray( c, SG );

    if( ga EQ NULL )
        NL_QUIT;

    c = -1;

    if( a GT - 1 )
    {
        c++;
        al[c] = k;
        be[c] = l;
        ga[c] = a;
    }

    if( b GT - 1 )
    {
        c++;
        al[c] = l;
        be[c] = k;
        ga[c] = b;
    }

    *alf = al;
    *bet = be;
    *gam = ga;
    *n = c;

    /* Exit */

    EXIT:

    return (error);
}
/**********************************************************************/
/* ST_TessFindBBoxIndices: Find box indexes from topology matrix                    */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     Given the toplogy matrix TM, a pair of indexes(i, j), this routine
     finds another pair(k, l) so that the indexes TM[i][j] and TM[k][l]
     represent the lower left and the upper right indexes of a subdivi-
     sion rectangle. A typical calling example is:

       NL_FLAG   *R, *T;
       NL_INDEX  **TM, i, j, k, l;
       ...
(get TM, R and T);
       ...
       ST_TessFindBBoxIndices(TM, R, T, i, j, &k, &l);


   ACCESS:
   
     TM  , input  ,  Topology matrix
     R, T , input  ,  Right and top pointers
     i, j , input  ,  Indexes of lower left corner
     k, l , output ,  Indexes of upper right corner
                       

   RETURN CODES:

     None

   ***********************************************************************/

static NL_VOID ST_TessFindBBoxIndices( NL_INDEX ** TM, NL_FLAG *R, NL_FLAG *T, NL_INDEX i, NL_INDEX j, NL_INDEX *k, NL_INDEX *l )
{
    NL_INDEX a, b, c;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if box exists at(i, j) */

    *k = -1;
    *l = -1;
    c = TM[i][j];

    if( c LT 0 )
        NL_OUT;

    if( R[c]EQ NL_NO OR T[c]EQ NL_NO )
        NL_OUT;

    /* Locate indexes */

    a = i + 1;
    b = j;

    while( TM[a][b]LT 0 OR T[TM[a][b]]EQ NL_NO )
        a++;
    *k = a;

    a = i;
    b = j + 1;

    while( TM[a][b]LT 0 OR R[TM[a][b]]EQ NL_NO )
        b++;
    *l = b;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );
}
/**********************************************************************/
/* N_TessEdgesFromPt: Find all edges incident at a given point                 */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and finds  all edges  that are incident at a given point. A typical 
     calling example is:

       NL_INDEX   **DT, *hd, *alf, *bet, k, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd, and index k);
       ...
       N_TessEdgesFromPt(DT, hd, m, k, &alf, &bet, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., bet[n] IS ALLOCATED 
     INSIDE THE ROUTINE.


   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     hd  , input  ,  hd[i] is the highest index in the list DT[i]
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     k   , input  ,  Index of given point
     alf , output ,  Indexes  of start points of edges
     bet , output ,  Indexes of end points of edges. The  returned edges 
                     are  <alf[0], bet[0]>,..., < alf[n], bet[n]>. THEY  ARE
                     LISTED IN COUNTERCLOCKWISE ORDER.
     n   , output ,  Highest index in alf and bet
     SG  , input  ,  Memory stack for alf and bet
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessEdgesFromPt( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX m, NL_INDEX k, NL_INDEX ** alf, NL_INDEX ** bet, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TessEdgesFromPt");

    NL_FLAG error = NL_NO;

    NL_INDEX *al, *be, i;

    /* Check input */

    if( m LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR k GT m )
        NL_ERROR( NL_IND_ERR );

    if( DT[k][0]EQ - 1 )
    {
        *n = -1;
        NL_OUT;
    }

    /* Get all edges */

    al = N_AllocInt1dArray( hd[k], SG );

    if( al EQ NULL )
        NL_QUIT;

    be = N_AllocInt1dArray( hd[k], SG );

    if( be EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= hd[k]; i++ )
    {
        al[i] = k;
        be[i] = DT[k][i];
    }

    *alf = al;
    *bet = be;
    *n = hd[k];

    /* Exit */

    EXIT:

    return (error);
}
/**********************************************************************/
/* N_TessPtsAdjPt: Find all points neighboring a given point                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and finds all points that are in the neighborhood of a given point. 
     A typical calling example is:

       NL_INDEX   **DT, *hd, *alf, k, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd, and index k);
       ...
       N_TessPtsAdjPt(DT, hd, m, k, &alf, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., alf[n] IS ALLOCATED 
     INSIDE THE ROUTINE.


   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     hd  , input  ,  hd[i] is the highest index in the list DT[i]
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     k   , input  ,  Index of given point
     alf , output ,  Indexes  of  points neighboring point k. NL_POINTS ARE
                     LISTED IN COUNTERCLOCKWISE ORDER.
     n   , output ,  Highest index in alf
     SG  , input  ,  Memory stack for alf
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessPtsAdjPt( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX m, NL_INDEX k, NL_INDEX ** alf, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TessPtsAdjPt");

    NL_FLAG error = NL_NO;

    NL_INDEX *al, i;

    /* Check input */

    if( m LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR k GT m )
        NL_ERROR( NL_IND_ERR );

    if( DT[k][0]EQ - 1 )
    {
        *n = -1;
        NL_OUT;
    }

    /* Get all neighboring points */

    al = N_AllocInt1dArray( hd[k], SG );

    if( al EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= hd[k]; i++ )
        al[i] = DT[k][i];

    *alf = al;
    *n = hd[k];

    /* Exit */

    EXIT:

    return (error);
}
/**********************************************************************/
/* N_TessTrianglesAdjPt: Find all triangles incident at a given point             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and  finds all  triangles  incident  at a  given  point. A  typical 
     calling example is:

       NL_INDEX   **DT, *hd, *alf, *bet, *gam, k, m, n;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd, and index k);
       ...
       N_TessTrianglesAdjPt(DT, hd, m, k, &alf, &bet, &gam, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., gam[n] IS ALLOCATED 
     INSIDE THE ROUTINE.


   ACCESS:
   
     DT  , input  ,  Index lists of triangulation
     hd  , input  ,  hd[i] is the highest index in the list DT[i]
     m   , input  ,  Highest index in hd and DT, ie there are m + 1 points 
                     in the triangulation
     k   , input  ,  Index of given point
     alf , output ,  Indexes of first points of triangles
     bet , output ,  Indexes of second points of triangles
     bet , output ,  Indexes of third points of triangles. The  returned 
                     triangles  are  <alf[0], bet[0], gam[0]>,..., < alf[n],
                     bet[n], gam[n]>. THEY ARE LISTED IN COUNTERCLOCKWISE 
                     ORDER.
     n   , output ,  Highest index in alf, bet and gam
     SG  , input  ,  Memory stack for alf, bet and gam
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessTrianglesAdjPt( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX m, NL_INDEX k, NL_INDEX ** alf, NL_INDEX ** bet, NL_INDEX ** gam, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TessTrianglesAdjPt");

    NL_FLAG error = NL_NO;

    NL_INDEX *al, *be, *ga, i;

    /* Check input */

    if( m LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR k GT m )
        NL_ERROR( NL_IND_ERR );

    if( DT[k][0]EQ - 1 )
    {
        *n = -1;
        NL_OUT;
    }

    /* Get all triangles */

    al = N_AllocInt1dArray( hd[k] - 1, SG );

    if( al EQ NULL )
        NL_QUIT;

    be = N_AllocInt1dArray( hd[k] - 1, SG );

    if( be EQ NULL )
        NL_QUIT;

    ga = N_AllocInt1dArray( hd[k] - 1, SG );

    if( ga EQ NULL )
        NL_QUIT;

    for ( i = 0; i < hd[k]; i++ )
    {
        al[i] = k;
        be[i] = DT[k][i];
        ga[i] = DT[k][i + 1];
    }

    *alf = al;
    *bet = be;
    *gam = ga;
    *n = hd[k] - 1;

    /* Exit */

    EXIT:

    return (error);
}
/**********************************************************************/
/* N_TESSTRIANGLESADJTRIANGLE: Find all triangles neighboring a given triangle          */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  browses the  database of a triangulation 
     and  finds  all triangles neighboring  a given triangle. A  typical 
     calling example is:

       NL_INDEX   **DT, *hd, *alf, *bet, *gam, k, l, m, n, t;
       NL_STACKS  SG;
       ...
(get triangulation DT and hd, and indexes k, l and m);
       ...
       N_TessTrianglesAdjTriangle(DT, hd, t, k, l, m, &alf, &bet, &gam, &n, &SG);

     MEMORY TO  STORE THE  NL_POINT  INDEXES alf[0],..., gam[n] IS ALLOCATED 
     INSIDE THE ROUTINE. INDEXES IN EACH RETURNED TRIANGLE ARE LISTED IN
     COUNTERCLOCKWISE ORDER.


   ACCESS:
   
     DT    , input  ,  Index lists of triangulation
     hd    , input  ,  hd[i] is the highest index in the list DT[i]
     t     , input  ,  Highest index in hd and  DT, i.e.  there are  t + 1 
                       points in the triangulation
     k, l, m , input  ,  Vertex indexes of given triangle
     alf   , output ,  Indexes of first points of triangles
     bet   , output ,  Indexes of second points of triangles
     bet   , output ,  Indexes  of   third   points of   triangles.  The 
                       returned  triangles  are  <alf[0], bet[0], gam[0]>,
                       ..., < alf[n], bet[n], gam[n]>. THERE  ARE 1, 2  OR 3
                       TRIANGLES.
     n     , output ,  Highest index in alf, bet and gam
     SG    , input  ,  Memory stack for alf, bet and gam
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessTrianglesAdjTriangle( NL_INDEX ** DT, NL_INDEX *hd, NL_INDEX t, NL_INDEX k, NL_INDEX l, NL_INDEX m, NL_INDEX ** alf, NL_INDEX ** bet, NL_INDEX ** gam, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TessTrianglesAdjTriangle");

    NL_FLAG error = NL_NO;

    NL_INDEX *al, *be, *ga, a1, a2 = 0, a3 = 0, b1, b2 = 0, b3 = 0, c1, c2 = 0, c3 = 0, d, r;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( t LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LT 0 OR k GT t )
        NL_ERROR( NL_IND_ERR );

    if( l LT 0 OR l GT t )
        NL_ERROR( NL_IND_ERR );

    if( m LT 0 OR m GT t )
        NL_ERROR( NL_IND_ERR );

    if( DT[k][0]EQ - 1 OR DT[l][0]EQ - 1 OR DT[m][0]EQ - 1 )
    {
        *n = -1;
        NL_OUT;
    }

    /* For each edge find neighboring triangles */

    a1 = b1 = c1 = d = -1;

    error = N_TessIsOnEdge( DT, hd, t, k, l, &al, &be, &ga, &r, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( r EQ 1 )
    {
        if( ga[0]NEQ m )
            r = 0;
        else
            r = 1;
        a1 = al[r];
        a2 = be[r];
        a3 = ga[r];
        d++;
    }

    error = N_TessIsOnEdge( DT, hd, t, l, m, &al, &be, &ga, &r, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( r EQ 1 )
    {
        if( ga[0]NEQ k )
            r = 0;
        else
            r = 1;
        b1 = al[r];
        b2 = be[r];
        b3 = ga[r];
        d++;
    }

    error = N_TessIsOnEdge( DT, hd, t, m, k, &al, &be, &ga, &r, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    if( r EQ 1 )
    {
        if( ga[0]NEQ l )
            r = 0;
        else
            r = 1;
        c1 = al[r];
        c2 = be[r];
        c3 = ga[r];
        d++;
    }

    if( d EQ - 1 )
    {
        *n = -1;
        NL_OUT;
    }

    /* Allocate memory and output indexes */

    al = N_AllocInt1dArray( d, SG );

    if( al EQ NULL )
        NL_QUIT;

    be = N_AllocInt1dArray( d, SG );

    if( be EQ NULL )
        NL_QUIT;

    ga = N_AllocInt1dArray( d, SG );

    if( ga EQ NULL )
        NL_QUIT;

    d = -1;

    if( a1 GT - 1 )
    {
        d++;
        al[d] = a1;
        be[d] = a2;
        ga[d] = a3;
    }

    if( b1 GT - 1 )
    {
        d++;
        al[d] = b1;
        be[d] = b2;
        ga[d] = b3;
    }

    if( c1 GT - 1 )
    {
        d++;
        al[d] = c1;
        be[d] = c2;
        ga[d] = c3;
    }

    *alf = al;
    *bet = be;
    *gam = ga;
    *n = d;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/***************************************************************************************/
/* ST_TessRegionOfRectangles: Triangulate a trimmed region bounded by many outer loops */
/***************************************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation routine  triangulates a trimmed  region  that has 
     been subdivided  into a set of  rectangles. Given the rectangles in 
     the parameter domain, this  routine first triangulates each trimmed 
     rectangle, then merges these triangulations into a triangulation of 
     the entire region. A typical calling example is:

       NL_FLAG       **SM, *R, *T;
       NL_PARAMETER  ***x, ***y, *u, *v;
       NL_INDEX      **DT, **TM, **hp, *ht, *hd, n, m, r, s;
       NL_REAL       **UL, **UR, **VB, **VT, tol;
       NL_STACKS     SG;
       ...
(define TM,..., s);
       ...
       ST_TessRegionOfRectangles(&u, &v, &n, R, T, TM, SM, r, s, x, y, ht, hp, m, UL, UR, VB, VT, tol,
                &DT, &hd, &SG);

     MEMORY  TO  STORE  THE  TRIANGULATION  DT[n][hd[i]]  AND  hd[i]  IS 
     ALLOCATED INSIDE THE ROUTINE.


   ACCESS:
   
     u, v   , in/out ,  Parameters corresponding to the  vertices of  the
                       rectangles
     pVtxCount , in/out ,  Highest index in(u, v)
     R, T   , input  ,  Right and top flags:
                         NL_YES: there is a right/top neighbor
                         NL_NO : there is no right/top neighbor
     TM    , input  ,  Topology matrix:
                         TM[i][j] >= 0 :(u[i], v[j]) is in the set
                         TM[i][j] <  0 : no point is at this location
     SM    , input  ,  Status matrix:
     SM[i][j] = { NL_ONANDOVER, NL_OVER, NL_INP,  NL_ONP, NL_OUTP}:
                                      NL_PRIVATE  NL_STRING  rname = "ST_TessRegionOfRectangles");
                                      rectangle  anchored at  u[i], v[j] is
                                      classified as listed
                         SM[i][j] = NL_NOSTATUS: no point at  this location
     r, s   , input  ,  Highest indexes in TM and SM
     x, y   , input  ,  Trimming polygons(ALL MUST BE CLOSED LOOPS!)
     ht    , input  ,  ht[i] is  the  highest  index of  polygons in the 
                       i - th loop
     hp    , input  ,  hp[i][j] is the highest index of  vertices in the
                       j - th polygon residing inside the i - th loop
     m     , input  ,  Highest index in ht, i.e. there are m + 1  trimming 
                       loops
     UL, UR , input  ,  Left and right bounds of trimming curves
     VB, VT , input  ,  Bottom and top bounds of trimming curves
     srfUMin, srfUMax, srfVMin, srfVMax , input , parameter bounds of surface
     tol   , input  ,  Parameter space tolerance
     DT    , output ,  Point lists of triangulation, ie  DT[i] points to
                       a list of indexes representing points surrounding
                       the point(u[i], v[i]). The indexes are listed in
                       COUNTERCLOCKWISE  order. If  DT[i][0]  is -1, the
                       point(u[i], v[i]) is out of the domain.
     hd    , output ,  hd[i] is the highest index of array DT[i]
     SG    , input  ,  DT's and h's memory stack
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

static NL_FLAG ST_TessRegionOfRectangles(
    NL_PARAMETER ** u, NL_PARAMETER ** v, NL_INDEX *pVtxCount,
    NL_FLAG *R, NL_FLAG *T,
    NL_INDEX ** TM, NL_FLAG ** SM,
    NL_INDEX r, NL_INDEX s,
    NL_PARAMETER *** x, NL_PARAMETER *** y,
    NL_INDEX *ht, NL_INDEX ** hp, NL_INDEX m,
    NL_REAL ** UL, NL_REAL ** UR, NL_REAL ** VB, NL_REAL ** VT,
    NL_REAL srfUMin, NL_REAL srfUMax, NL_REAL srfVMin, NL_REAL srfVMax,
    NL_REAL tol,
    NL_INDEX *** DT, NL_INDEX ** hd,
    NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("ST_TessRegionOfRectangles");

    NL_FLAG *used, touch, inner, dupl, error = NL_NO;

    NL_BOOLEAN lsd, rsd;

    NL_INDEX *** DL, ** DO, ** HI, ** p, *g, *ins, *LI, *LID, *S_A, *S_B, *ho, *buf, *idup, *ld, bi, bs, bn, i, j, k, l, top, lia, lib, lig, lis, ind = 0, hid, a, b, c, t, alf, bet, gam, outer, ns, np, sin, pin, join, way, left, start, nl = 0, nh, ni, dpl, lo = 0, prev, curr, next, lp, count;

    NL_REAL *ul, *vl, cmin, csa, xs, ys, xe, ye, xf, yf, xl, xr, yb, yt, xes, yes, xef, yef, xsf, ysf;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Allocate memory and initialize */

    ns = 30;
    np = 10;
    sin = 20;
    pin = 5;
    top = -1;
    nh = *pVtxCount;
    bi = -1;
    bs = 75;
    bn = 30;
    ul = *u;
    vl = *v;

    c = 1 + (NL_MAX( r, s ) / 2);

    for ( a = 0; a <= m; a++ )
    {
        for ( b = 0; b <= ht[a]; b++ )
            nh += (hp[a][b] * c);
    }

    buf = N_AllocInt1dArray( bs, &SL );

    if( buf EQ NULL )
        NL_QUIT;

    DL = N_AllocIntPtr2dArray( 3, nh, &SL );

    if( DL EQ NULL )
        NL_QUIT;

    HI = N_AllocInt2dArray( 3, nh, &SL );

    if( HI EQ NULL )
        NL_QUIT;

    LI = N_AllocInt1dArray( nh, &SL );

    if( LI EQ NULL )
        NL_QUIT;

    used = N_AllocFlag1dArray( nh, &SL );

    if( used EQ NULL )
        NL_QUIT;

    S_A = N_AllocInt1dArray( ns, &SL );

    if( S_A EQ NULL )
        NL_QUIT;

    S_B = N_AllocInt1dArray( ns, &SL );

    if( S_B EQ NULL )
        NL_QUIT;

    for ( b = 0; b <= nh; b++ )
    {
        LI[b] = -1;
        used[b] = NL_NO;

        for ( a = 0; a <= 3; a++ )
        {
            DL[a][b] = NULL;
            HI[a][b] = -1;
        }
    }

    /**************************************/
    /* For all rectangles in all loops do */
    /**************************************/

    for ( lp = 0; lp <= m; lp++ )
    {
        for ( i = 0; i < r; i++ )
        {
            for ( j = 0; j < s; j++ )
            {
                /* Find opposite corner */

                ST_TessFindBBoxIndices( TM, R, T, i, j, &k, &l );

                if( k GE 0 AND l GE 0 AND SM[i][j]NEQ NL_OUTP )
                {
                    /* See if box is inside the current outer loop */

                    if( m GT 0 )
                    {
                        xl = ul[TM[i][j]];
                        xr = ul[TM[k][l]];
                        yb = vl[TM[i][j]];
                        yt = vl[TM[k][l]];

                        if( ST_TessIsInsideLoop( x[lp], y[lp], hp[lp], ht[lp], UL[lp], UR[lp], VB[lp], VT[lp], xl, xr, yb, yt, tol )EQ NL_OUTP )
                            continue;
                    }

                    /* Get trimming polygons */

                    error = ST_TessBBoxXTrim(
                        u, v, pVtxCount,
                        TM, r, s, i, j, k, l,
                        SM[i][j], x[lp], y[lp], hp[lp], ht[lp],
                        UL[lp], UR[lp], VB[lp], VT[lp],
                        srfUMin, srfUMax, srfVMin, srfVMax,
                        tol,
                        &buf, &bi, &bs, bn, &p, &g, &ins, &t, &SL, SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    ul = *u;
                    vl = *v;
                    nl = *pVtxCount;

                    if( t EQ - 1 )
                        continue;

                    if( nl GT nh )
                    {
                        a = 2 * (nl - nh);
                        b = nh / 2;
                        ni = NL_MAX( a, b );

                        error = N_Realloc2dIntPtrArray( &DL, 3, nh, 3, nh + ni, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_Realloc2dIntArray( &HI, 3, nh, 3, nh + ni, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_Realloc1dIntArray( &LI, nh, nh + ni, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        error = N_Realloc1dFlagArray( &used, nh, nh + ni, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        for ( b = nh + 1; b <= nh + ni; b++ )
                        {
                            used[b] = NL_NO;
                            LI[b] = -1;

                            for ( a = 0; a <= 3; a++ )
                            {
                                DL[a][b] = NULL;
                                HI[a][b] = -1;
                            }
                        }

                        nh += ni;
                    }

                    /* Postprocess polygons */

                    for ( a = 0; a <= t; a++ )
                    {
                        if( p[a][0]EQ p[a][g[a]] )
                            g[a]--;

                        /* remove repeated points to avoid an infinite loop later on*/

                        for ( b = 0; b < g[a]; b++ )
                        {
                            if( p[a][b]EQ p[a][b + 1] )
                            {
                                for ( c = b + 1; c < g[a]; c++ )
                                {
                                    p[a][c] = p[a][c + 1];
                                }
                                g[a]--;
                            }
                        }
                    }

                    inner = NL_NO;
                    dpl = 0;

                    for ( a = 0; a <= t; a++ )
                    {
                        if( ins[a]GE 0 )
                            inner = NL_YES;

                        if( g[a]GT dpl )
                            dpl = g[a];
                    }

                    idup = N_AllocInt1dArray( 3 * (dpl + 1), &SL );

                    if( idup EQ NULL )
                        NL_QUIT;

                    LID = &idup[dpl + 1];
                    ld = &LID[dpl + 1];

                    /**************************************/
                    /* For all disjoint outer polygons do */
                    /**************************************/

                    outer = 0;

                    while( outer LE t AND ins[outer]EQ - 1 )
                    {
                        if( g[outer]LE 1 )
                            NL_ERROR( NL_GEO_ERR );

                        /* Allocate memory for outer polygon vertex lists and initialize */

                        b = g[outer];

                        for ( a = 0; a <= b; a++ )
                        {
                            bet = p[outer][a];
                            used[bet] = NL_NO;

                            if( bet LT 0 OR bet GT nl )
                                NL_ERROR( NL_GEO_ERR );

                            for ( c = 0; c <= 3; c++ )
                            {
                                if( DL[c][bet]EQ NULL )
                                {
                                    DL[c][bet] = N_AllocInt1dArray( np, &SL );

                                    if( DL[c][bet]EQ NULL )
                                        NL_QUIT;

                                    ST_TessInitArray( DL[c][bet], 0, np, 0 );

                                    DL[c][bet][np] = NL_EIA;
                                    LI[bet] = c;
                                    break;
                                }
                            }

                            if( a EQ 0 )
                                alf = p[outer][b];
                            else
                                alf = p[outer][a - 1];

                            if( a EQ b )
                                gam = p[outer][0];
                            else
                                gam = p[outer][a + 1];

                            if( alf LT 0 OR alf GT nl )
                                NL_ERROR( NL_GEO_ERR );

                            if( gam LT 0 OR gam GT nl )
                                NL_ERROR( NL_GEO_ERR );

                            if( c GT 3 )
                                NL_ERROR( NL_MEM_ERR );

                            DL[c][bet][0] = gam;
                            DL[c][bet][1] = alf;
                            HI[c][bet] += 2;
                        }

                        /* Allocate memory for inner pol vertex lists and initialize */

                        if( inner EQ NL_YES )
                        {
                            for ( a = 0; a <= t; a++ )
                            {
                                if( ins[a]EQ outer )
                                {
                                    for ( b = 0; b <= g[a]; b++ )
                                    {
                                        bet = p[a][b];
                                        used[bet] = NL_NO;

                                        if( bet LT 0 OR bet GT nl )
                                            NL_ERROR( NL_GEO_ERR );

                                        for ( c = 0; c <= 3; c++ )
                                        {
                                            if( DL[c][bet]EQ NULL )
                                            {
                                                DL[c][bet] = N_AllocInt1dArray( np, &SL );

                                                if( DL[c][bet]EQ NULL )
                                                    NL_QUIT;

                                                ST_TessInitArray( DL[c][bet], 0, np, 0 );

                                                DL[c][bet][np] = NL_EIA;
                                                LI[bet] = c;
                                                break;
                                            }
                                        }

                                        if( b EQ 0 )
                                            alf = p[a][g[a]];
                                        else
                                            alf = p[a][b - 1];

                                        if( b EQ g[a] )
                                            gam = p[a][0];
                                        else
                                            gam = p[a][b + 1];

                                        if( alf LT 0 OR alf GT nl )
                                            NL_ERROR( NL_GEO_ERR );

                                        if( gam LT 0 OR gam GT nl )
                                            NL_ERROR( NL_GEO_ERR );

                                        if( c GT 3 )
                                            NL_ERROR( NL_MEM_ERR );

                                        DL[c][bet][0] = gam;
                                        DL[c][bet][1] = alf;
                                        HI[c][bet] += 2;
                                    }
                                }
                            }
                        }

                        /* Buffer indexes of multiple points */

                        dpl = -1;

                        for ( a = 0; a < g[outer]; a++ )
                        {
                            for ( b = a + 1; b <= g[outer]; b++ )
                            {
                                if( p[outer][a]EQ p[outer][b] )
                                {
                                    dpl++;
                                    idup[dpl] = p[outer][a];
                                    ld[dpl] = a;
                                    LID[dpl] = LI[p[outer][a]];
                                }
                            }
                        }

                        /* Put outer polygon vertices on the stack */

                        if( top + g[outer] + 2 GT ns )
                        {
                            error = N_Realloc1dIntArray( &S_A, ns, ns + g[outer] + 2, &SL );

                            if( error EQ NL_YES )
                                NL_OUT;

                            error = N_Realloc1dIntArray( &S_B, ns, ns + g[outer] + 2, &SL );

                            if( error EQ NL_YES )
                                NL_OUT;

                            ns += (g[outer] + 2);
                        }

                        for ( b = g[outer]; b >= 0; b-- )
                        {
                            alf = p[outer][b];

                            if( b EQ g[outer] )
                                bet = p[outer][0];
                            else
                                bet = p[outer][b + 1];

                            S_A[top + 1] = alf;
                            S_B[top + 1] = bet;
                            top++;
                        }

                        /* Put all inner polygon vertices on the stack */

                        if( inner EQ NL_YES )
                        {
                            for ( a = 0; a <= t; a++ )
                            {
                                if( ins[a]EQ outer )
                                {
                                    if( top + g[a] + 2 GT ns )
                                    {
                                        error = N_Realloc1dIntArray( &S_A, ns, ns + g[a] + 2, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        error = N_Realloc1dIntArray( &S_B, ns, ns + g[a] + 2, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ns += (g[a] + 2);
                                    }

                                    for ( b = 0; b <= g[a]; b++ )
                                    {
                                        alf = p[a][b];

                                        if( b EQ g[a] )
                                            bet = p[a][0];
                                        else
                                            bet = p[a][b + 1];

                                        S_A[top + 1] = alf;
                                        S_B[top + 1] = bet;
                                        top++;
                                    }
                                }
                            }
                        }

                        /*******************************/
                        /* While stack is not empty do */
                        /*******************************/

                        used[S_A[top]] = NL_YES;
                        used[S_B[top]] = NL_YES;
                        count = -1;

                        while( top GE 0 )
                        {
                            /* Get current edge -> pop stack */

                            alf = S_A[top];
                            bet = S_B[top];
                            xs = ul[alf];
                            ys = vl[alf];
                            xe = ul[bet];
                            ye = vl[bet];
                            xes = xe - xs;
                            yes = ye - ys;

                            /* Search for the third point */

                            cmin = NL_BIGD;
                            gam = -1;

                            for ( a = 0; a <= t; a++ )
                            {
                                for ( b = 0; b <= g[a]; b++ )
                                {
                                    ind = p[a][b];

                                    if( ind EQ alf OR ind EQ bet )
                                        continue;

                                    xf = ul[ind];
                                    yf = vl[ind];
                                    xsf = xs - xf;
                                    ysf = ys - yf;

                                    if( ST_TessSideTest( xsf, ysf, xes, yes )EQ NL_RIGHT )
                                        continue;

                                    xef = xe - xf;
                                    yef = ye - yf;

                                    error = ST_TessCosine( xsf, ysf, xef, yef, &csa );

                                    if( error EQ NL_YES )
                                        NL_OUT;

                                    if( csa GE cmin )
                                        continue;

                                    if( ST_TessLineXPolygon( xs, ys, xf, yf, -xsf, -ysf, ul, vl, p, g, t, tol )EQ NL_YES )
                                        continue;

                                    if( ST_TessLineXPolygon( xf, yf, xe, ye, xef, yef, ul, vl, p, g, t, tol )EQ NL_YES )
                                        continue;

                                    cmin = csa;
                                    gam = ind;
                                }
                            }

                            if( gam LT 0 OR gam GT nl )
                                NL_ERROR( NL_GEO_ERR );

                            /**********************/
                            /* Update point lists */
                            /**********************/

                            /* Determine touch case */

                            touch = ST_TessEdgeTouch( alf, bet, gam, S_A, S_B, &top, used );

                            lia = LI[alf];
                            lib = LI[bet];
                            lig = LI[gam];

                            if( lia LT 0 OR lia GT 3 )
                                NL_ERROR( NL_MEM_ERR );

                            if( lib LT 0 OR lib GT 3 )
                                NL_ERROR( NL_MEM_ERR );

                            if( lig LT 0 OR lig GT 3 )
                                NL_ERROR( NL_MEM_ERR );

                            /* Handle multiple points */

                            if( dpl GE 0 )
                            {
                                dupl = NL_NO;

                                for ( a = 0; a <= dpl; a++ )
                                {
                                    ind = idup[a];

                                    if( alf EQ ind OR bet EQ ind OR gam EQ ind )
                                    {
                                        dupl = NL_YES;
                                        c = LID[a];
                                        lo = ld[a];
                                        break;
                                    }
                                }

                                if( dupl EQ NL_YES )
                                {
                                    b = g[outer];

                                    if( lo EQ 0 )
                                        prev = p[outer][b];
                                    else
                                        prev = p[outer][lo - 1];
                                    curr = p[outer][lo];

                                    if( lo EQ b )
                                        next = p[outer][0];
                                    else
                                        next = p[outer][lo + 1];

                                    if( alf EQ ind )
                                    {
                                        lsd = ST_TessPtInSpan( ul, vl, prev, curr, next, bet, tol );
                                        rsd = ST_TessPtInSpan( ul, vl, prev, curr, next, gam, tol );

                                        if( lsd AND rsd )
                                            lia = c - 1;
                                        else
                                            lia = c;

                                        if( lia LT 0 OR lia GT 3 )
                                            NL_ERROR( NL_MEM_ERR );
                                    }
                                    else if( bet EQ ind )
                                    {
                                        lsd = ST_TessPtInSpan( ul, vl, prev, curr, next, alf, tol );
                                        rsd = ST_TessPtInSpan( ul, vl, prev, curr, next, gam, tol );

                                        if( lsd AND rsd )
                                            lib = c - 1;
                                        else
                                            lib = c;

                                        if( lib LT 0 OR lib GT 3 )
                                            NL_ERROR( NL_MEM_ERR );
                                    }
                                    else if( gam EQ ind )
                                    {
                                        lsd = ST_TessPtInSpan( ul, vl, prev, curr, next, alf, tol );
                                        rsd = ST_TessPtInSpan( ul, vl, prev, curr, next, bet, tol );

                                        if( lsd AND rsd )
                                            lig = c - 1;
                                        else
                                            lig = c;

                                        if( lig LT 0 OR lig GT 3 )
                                            NL_ERROR( NL_MEM_ERR );
                                    }
                                }
                            }

                            used[gam] = NL_YES;

                            switch( touch )
                            {
                                case NL_TWOTOUCH:

                                    /* Skip to the next edge */

                                    count = -1;

                                    top--;
                                    continue;
                                    break;

                                case NL_NOTOUCH:

                                    /* Put edge to the bottom of the stack */

                                    count += 1;

                                    if( count GT top + 3 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = top; a >= 1; a-- )
                                    {
                                        S_A[a] = S_A[a - 1];
                                        S_B[a] = S_B[a - 1];
                                    }
                                    S_A[0] = alf;
                                    S_B[0] = bet;

                                    continue;
                                    break;

                                case NL_REGULAR:

                                    /* Add gam to alf's list after bet */

                                    count = -1;

                                    HI[lia][alf]++;
                                    hid = HI[lia][alf];

                                    if( DL[lia][alf][hid]EQ NL_EIA )
                                    {
                                        error = N_Realloc1dIntArray( &DL[lia][alf], hid, hid + pin, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ST_TessInitArray( DL[lia][alf], hid, hid + pin, 0 );

                                        DL[lia][alf][hid + pin] = NL_EIA;
                                    }

                                    b = -1;

                                    for ( a = 0; a < hid; a++ )
                                    {
                                        if( DL[lia][alf][a]EQ bet )
                                        {
                                            b = a;
                                            break;
                                        }
                                    }

                                    if( b EQ - 1 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = hid; a >= b + 2; a-- )
                                        DL[lia][alf][a] = DL[lia][alf][a - 1];
                                    DL[lia][alf][b + 1] = gam;

                                    /* Add gam to bet's list before alf */

                                    HI[lib][bet]++;
                                    hid = HI[lib][bet];

                                    if( DL[lib][bet][hid]EQ NL_EIA )
                                    {
                                        error = N_Realloc1dIntArray( &DL[lib][bet], hid, hid + pin, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ST_TessInitArray( DL[lib][bet], hid, hid + pin, 0 );

                                        DL[lib][bet][hid + pin] = NL_EIA;
                                    }

                                    b = -1;

                                    for ( a = 0; a < hid; a++ )
                                    {
                                        if( DL[lib][bet][a]EQ alf )
                                        {
                                            b = a;
                                            break;
                                        }
                                    }

                                    if( b EQ - 1 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = hid; a >= b + 1; a-- )
                                        DL[lib][bet][a] = DL[lib][bet][a - 1];
                                    DL[lib][bet][b] = gam;

                                    /* Append alf and bet to gam's list */

                                    HI[lig][gam] += 2;
                                    DL[lig][gam][3] = DL[lig][gam][1];
                                    DL[lig][gam][1] = alf;
                                    DL[lig][gam][2] = bet;

                                    break;

                                case NL_RIGHTTOUCH:

                                    /* Add gam to bet's list before alf */

                                    count = -1;

                                    HI[lib][bet]++;
                                    hid = HI[lib][bet];

                                    if( DL[lib][bet][hid]EQ NL_EIA )
                                    {
                                        error = N_Realloc1dIntArray( &DL[lib][bet], hid, hid + pin, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ST_TessInitArray( DL[lib][bet], hid, hid + pin, 0 );

                                        DL[lib][bet][hid + pin] = NL_EIA;
                                    }

                                    b = -1;

                                    for ( a = 0; a < hid; a++ )
                                    {
                                        if( DL[lib][bet][a]EQ alf )
                                        {
                                            b = a;
                                            break;
                                        }
                                    }

                                    if( b EQ - 1 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = hid; a >= b + 1; a-- )
                                        DL[lib][bet][a] = DL[lib][bet][a - 1];
                                    DL[lib][bet][b] = gam;

                                    /* Add bet to gam's list after alf */

                                    HI[lig][gam]++;
                                    hid = HI[lig][gam];

                                    if( DL[lig][gam][hid]EQ NL_EIA )
                                    {
                                        error = N_Realloc1dIntArray( &DL[lig][gam], hid, hid + pin, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ST_TessInitArray( DL[lig][gam], hid, hid + pin, 0 );

                                        DL[lig][gam][hid + pin] = NL_EIA;
                                    }

                                    b = -1;

                                    for ( a = 0; a < hid; a++ )
                                    {
                                        if( DL[lig][gam][a]EQ alf )
                                        {
                                            b = a;
                                            break;
                                        }
                                    }

                                    if( b EQ - 1 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = hid; a >= b + 2; a-- )
                                        DL[lig][gam][a] = DL[lig][gam][a - 1];
                                    DL[lig][gam][b + 1] = bet;

                                    break;

                                case NL_LEFTTOUCH:

                                    /* Add gam to alf's list after bet */

                                    count = -1;

                                    HI[lia][alf]++;
                                    hid = HI[lia][alf];

                                    if( DL[lia][alf][hid]EQ NL_EIA )
                                    {
                                        error = N_Realloc1dIntArray( &DL[lia][alf], hid, hid + pin, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ST_TessInitArray( DL[lia][alf], hid, hid + pin, 0 );

                                        DL[lia][alf][hid + pin] = NL_EIA;
                                    }

                                    b = -1;

                                    for ( a = 0; a < hid; a++ )
                                    {
                                        if( DL[lia][alf][a]EQ bet )
                                        {
                                            b = a;
                                            break;
                                        }
                                    }

                                    if( b EQ - 1 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = hid; a >= b + 2; a-- )
                                        DL[lia][alf][a] = DL[lia][alf][a - 1];
                                    DL[lia][alf][b + 1] = gam;

                                    /* Add alf to gam's list before bet */

                                    HI[lig][gam]++;
                                    hid = HI[lig][gam];

                                    if( DL[lig][gam][hid]EQ NL_EIA )
                                    {
                                        error = N_Realloc1dIntArray( &DL[lig][gam], hid, hid + pin, &SL );

                                        if( error EQ NL_YES )
                                            NL_OUT;

                                        ST_TessInitArray( DL[lig][gam], hid, hid + pin, 0 );

                                        DL[lig][gam][hid + pin] = NL_EIA;
                                    }

                                    b = -1;

                                    for ( a = 0; a < hid; a++ )
                                    {
                                        if( DL[lig][gam][a]EQ bet )
                                        {
                                            b = a;
                                            break;
                                        }
                                    }

                                    if( b EQ - 1 )
                                        NL_ERROR( NL_GEO_ERR );

                                    for ( a = hid; a >= b + 1; a-- )
                                        DL[lig][gam][a] = DL[lig][gam][a - 1];
                                    DL[lig][gam][b] = alf;

                                    break;
                            }

                            /* Put non - boundary edges on the stack */

                            top--;

                            if( top + 2 GT ns )
                            {
                                error = N_Realloc1dIntArray( &S_A, ns, ns + sin, &SL );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                error = N_Realloc1dIntArray( &S_B, ns, ns + sin, &SL );

                                if( error EQ NL_YES )
                                    NL_OUT;

                                ns += sin;
                            }

                            if( ST_TessIsEdgeOnBoundary( alf, gam, p, g, t )EQ NL_NO AND touch NEQ NL_RIGHTTOUCH )
                            {
                                S_A[top + 1] = alf;
                                S_B[top + 1] = gam;
                                top++;
                            }

                            if( ST_TessIsEdgeOnBoundary( gam, bet, p, g, t )EQ NL_NO AND touch NEQ NL_LEFTTOUCH )
                            {
                                S_A[top + 1] = gam;
                                S_B[top + 1] = bet;
                                top++;
                            }
                        } /* End of inner while */

                        outer++;
                    }     /* End of outer while */

                    /* Kill temporary memory */

                    for ( a = 0; a <= t; a++ )
                        N_FreeInt1dArray( p[a], &SL );

                    N_FreeIntPtr1dArray( p, &SL );
                    N_FreeInt1dArray( g, &SL );
                    N_FreeInt1dArray( idup, &SL );
                }
            }
        }
    }

    /* Triangulation of each rectangle is finished -> merge triangulations */

    DO = N_AllocIntPtr1dArray( nl, SG );

    if( DO EQ NULL )
        NL_QUIT;

    ho = N_AllocInt1dArray( nl, SG );

    if( ho EQ NULL )
        NL_QUIT;

    for ( ind = 0; ind <= nl; ind++ )
    {
        /* Get highest index */

        ho[ind] = 0;
        way = 0;

        for ( lis = 0; lis <= 3; lis++ )
        {
            if( HI[lis][ind]GE 0 )
            {
                ho[ind] += HI[lis][ind];
                way++;
            }
        }

        /* Merge lists */

        DO[ind] = N_AllocInt1dArray( ho[ind], SG );

        if( DO[ind]EQ NULL )
            NL_QUIT;

        if( way EQ 0 )
        {
            DO[ind][0] = -1;
            continue;
        }

        /* See if lists match up */

        start = -1;

        for ( a = 0; a < way; a++ )
        {
            left = way - 1;
            hid = HI[a][ind];
            join = DL[a][ind][hid];

            while( left GE 1 )
            {
                for ( b = 0; b < way; b++ )
                {
                    if( a EQ b )
                        continue;

                    if( DL[b][ind][0]EQ join )
                        break;
                }

                if( b EQ way )
                    break;
                hid = HI[b][ind];
                join = DL[b][ind][hid];
                left--;
            }

            if( b LT way )
            {
                start = a;
                break;
            }
        }

        /* If no match, copy list */

        if( start EQ - 1 )
        {
            if( way GT 1 )
                NL_ERROR( NL_GEO_ERR );

            hid = HI[0][ind];

            for ( b = 0; b <= hid; b++ )
                DO[ind][b] = DL[0][ind][b];
        }
        else
        {
            /* Match found -> hook up lists */

            left = way - 1;
            hid = HI[start][ind];
            join = DL[start][ind][hid];
            t = -1;

            for ( b = 0; b <= hid; b++ )
                DO[ind][++t] = DL[start][ind][b];

            while( left GE 1 )
            {
                for ( a = 0; a < way; a++ )
                {
                    if( DL[a][ind][0]EQ join )
                        break;
                }

                hid = HI[a][ind];
                join = DL[a][ind][hid];

                for ( b = 1; b <= hid; b++ )
                    DO[ind][++t] = DL[a][ind][b];

                left--;
            }
        }
    }

    *DT = DO;
    *hd = ho;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* ST_TessInitArray: Initialize DL array                              */
/**********************************************************************/

static NL_VOID ST_TessInitArray( NL_INDEX *DL, NL_INDEX s, NL_INDEX e, NL_INTEGER v )
{
    NL_INDEX i;

    for ( i = s; i <= e; i++ )
        DL[i] = v;
}

/**********************************************************************/
/* ST_TessSideTest: Side test for 2d points and lines                  */
/**********************************************************************/

static NL_FLAG ST_TessSideTest( NL_REAL xsf, NL_REAL ysf, NL_REAL xes, NL_REAL yes )
{
    NL_REAL crs;

    crs = xes * ysf - yes * xsf;

    if( crs GE 0.0 )
        return NL_RIGHT;
    else
        return NL_LEFT;
}

/**********************************************************************/
/* ST_TessPtInSpan: Point in line span test                                  */
/**********************************************************************/

static NL_BOOLEAN ST_TessPtInSpan( NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX p, NL_INDEX c, NL_INDEX n, NL_INDEX k, NL_REAL tol )
{
    NL_FLAG sp, sn;

    NL_REAL vx, vy, wx, wy, crs;

    wx = u[k] - u[c];
    wy = v[k] - v[c];

    vx = u[p] - u[c];
    vy = v[p] - v[c];
    crs = vx * wy - vy * wx;

    if( crs LE tol )
        sp = NL_RIGHT;
    else
        sp = NL_LEFT;

    vx = u[n] - u[c];
    vy = v[n] - v[c];
    crs = vx * wy - vy * wx;

    if( crs GE - tol )
        sn = NL_LEFT;
    else
        sn = NL_RIGHT;

    if( sp EQ NL_RIGHT AND sn EQ NL_LEFT )
        return NL_YES;
    else
        return NL_NO;
}

/**********************************************************************/
/* ST_TessCosine: Compute cosine of angle                             */
/**********************************************************************/

static NL_FLAG ST_TessCosine( NL_REAL u12, NL_REAL v12, NL_REAL u32, NL_REAL v32, NL_REAL *c )
{
    NL_PRIVATE NL_STRING rname = _T("ST_TessCosine");

    NL_FLAG error = NL_NO;

    NL_REAL a, b, ab, den;

    ab = u32 * u12 + v32 * v12;
    a = u32 * u32 + v32 * v32;
    b = u12 * u12 + v12 * v12;

    if( ab LT 0.0 )
        ab = -(ab * ab);
    else
        ab = ab * ab;

    den = a * b;

    if( N_FloatOpIsBad( ab, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    *c = ab / den;

    EXIT:

    return (error);
}

/**********************************************************************/
/* ST_TessIsEdgeOnBoundary: Check if edge is boundary edge            */
/**********************************************************************/

static NL_BOOLEAN ST_TessIsEdgeOnBoundary( NL_INDEX r, NL_INDEX s, NL_INDEX ** p, NL_INDEX *g, NL_INDEX t )
{
    NL_INDEX i, j, k, l;

    for ( i = 0; i <= t; i++ )
    {
        for ( j = 0; j <= g[i]; j++ )
        {
            if( j EQ g[i] )
                k = 0;
            else
                k = j + 1;

            if( j EQ 0 )
                l = g[i];
            else
                l = j - 1;

            if( r EQ p[i][j] )
                if( s EQ p[i][k]OR s EQ p[i][l] )
                    return NL_YES;
        }
    }

    return NL_NO;
}

/**********************************************************************/
/* ST_TessEdgeTouch: Determine edge touch cases                       */
/**********************************************************************/

static NL_FLAG ST_TessEdgeTouch( NL_INDEX alf, NL_INDEX bet, NL_INDEX gam, NL_INDEX *S_A, NL_INDEX *S_B, NL_INDEX *top, NL_FLAG *used )
{
    NL_FLAG touch = 0;

    NL_INDEX i, ag, gb;

    /* Check for left touch */

    ag = -1;

    for ( i = 0; i <= ( *top); i++ )
    {
        if( (alf EQ S_A[i]AND gam EQ S_B[i])OR( alf EQ S_B[i]AND gam EQ S_A[i] ) )
        {
            ag = i;
            break;
        }
    }

    if( ag GE 0 )
    {
        for ( i = ag + 1; i <= ( *top); i++ )
        {
            S_A[i - 1] = S_A[i];
            S_B[i - 1] = S_B[i];
        }
        ( *top)--;
    }

    /* Check for right touch */

    gb = -1;

    for ( i = 0; i <= ( *top); i++ )
    {
        if( (gam EQ S_A[i]AND bet EQ S_B[i])OR( gam EQ S_B[i]AND bet EQ S_A[i] ) )
        {
            gb = i;
            break;
        }
    }

    if( gb GE 0 )
    {
        for ( i = gb + 1; i <= ( *top); i++ )
        {
            S_A[i - 1] = S_A[i];
            S_B[i - 1] = S_B[i];
        }
        ( *top)--;
    }

    /* Classify touch */

    if( ag GE 0 OR gb GE 0 )
    {
        if( ag GE 0 AND gb GE 0 )
            touch = NL_TWOTOUCH;

        else if( ag GE 0 )
            touch = NL_RIGHTTOUCH;

        else if( gb GE 0 )
            touch = NL_LEFTTOUCH;
    }

    else if( used[gam]EQ NL_YES )
        touch = NL_NOTOUCH;

    else
        touch = NL_REGULAR;

    return touch;
}

/**********************************************************************/
/* ST_TessLineXPolygon: Check if line intersects polygon                         */
/**********************************************************************/

static NL_BOOLEAN ST_TessLineXPolygon( NL_REAL x1, NL_REAL y1, NL_REAL x2, NL_REAL y2, NL_REAL x12, NL_REAL y12, NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX ** p, NL_INDEX *g, NL_INDEX t, NL_REAL tol )
{
    NL_INDEX i, j, k, a;

    NL_REAL x3, y3, x4, y4, cr3, cr4, x43, y43, x31, y31, mnxp, mnyp, mxxm, mxym, x41, y41, den, denp, denm, numa, numb, mag12, tol2;

    mag12 = sqrt( x12 * x12 + y12 * y12 );

    if( mag12 LE 0.01 *tol )
        return NL_NO;

    tol2 = 2.0 *tol * mag12;

    mnxp = NL_MIN( x1, x2 ) + tol;
    mnyp = NL_MIN( y1, y2 ) + tol;
    mxxm = NL_MAX( x1, x2 ) - tol;
    mxym = NL_MAX( y1, y2 ) - tol;

    for ( i = 0; i <= t; i++ )
    {
        for ( j = 0; j <= g[i]; j++ )
        {
            a = p[i][j];
            x3 = u[a];
            y3 = v[a];

            if( j EQ g[i] )
                k = 0;
            else
                k = j + 1;
            a = p[i][k];
            x4 = u[a];
            y4 = v[a];

            if( NL_MAX( x3, x4 ) - tol LT mnxp )
                continue;

            if( mxxm LT NL_MIN( x3, x4 ) + tol )
                continue;

            if( NL_MAX( y3, y4 ) - tol LT mnyp )
                continue;

            if( mxym LT NL_MIN( y3, y4 ) + tol )
                continue;

            x31 = x1 - x3;
            y31 = y1 - y3;
            x41 = x1 - x4;
            y41 = y1 - y4;

            cr3 = x12 * y31 - y12 * x31;
            cr4 = x12 * y41 - y12 * x41;

            if( cr3 * cr4 GT 0.0 )
                if( fabs( cr3 )GT tol2 AND fabs( cr4 )GT tol2 )
                    continue;

            x43 = x3 - x4;
            y43 = y3 - y4;
            den = y12 * x43 - x12 * y43;

            if( fabs( den )LT NL_LTOL ) 
                continue;

            numb = cr3;

            if( den GT 0.0 )
            {
                denp = den - tol;

                if( numb LT tol OR numb GT denp )
                    continue;

                numa = y43 * x31 - x43 * y31;

                if( numa LT tol OR numa GT denp )
                    continue;
            }
            else
            {
                denm = den + tol;

                if( numb GT - tol OR numb LT denm )
                    continue;

                numa = y43 * x31 - x43 * y31;

                if( numa GT - tol OR numa LT denm )
                    continue;
            }

            return NL_YES;
        }
    }

    return NL_NO;
}
/**********************************************************************/
/* NL_TessSrfArea: Estimate surface area for tessellation                   */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     see N_TessTrimmedSrf.     
     This tesselation routine estimates surface area to allocate memory
     for  various  arrays for  surface  tesselation. A typical  calling 
     example is:

       NL_SURFACE  sur;
       NL_REAL     A, lu, lv;
       ...
(define sur);
       ...
       N_TessSrfArea(&sur, &A, &lu, &lv);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     A     , output ,  Estimated surface area
     lu, lv , output ,  Average lengths in u- and v - directions 
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TessSrfArea( NL_SURFACE *sur, NL_REAL *A, NL_REAL *lu, NL_REAL *lv )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL len, maxu, maxv, sumu, sumv, d;

    NL_POINT *P;

    NL_CPOINT ** Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and allocate memory */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    P = N_AllocPt1dArray( NL_MAX( n, m ), &SL );

    if( P EQ NULL )
        NL_QUIT;

    /* Get maximum polygon lengths in u- and v - directions */

    maxu = sumu = 0.0;

    for ( j = 0; j <= m; j++ )
    {
        N_CPtToPtEuclid( Pw[0][j], &P[0] );

        len = 0.0;

        for ( i = 1; i <= n; i++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &P[i] );
            N_DistPtPt( P[i], P[i - 1], &d );

            len += d;
        }
        sumu += len;

        if( len GT maxu )
            maxu = len;
    }

    maxv = sumv = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        N_CPtToPtEuclid( Pw[i][0], &P[0] );

        len = 0.0;

        for ( j = 1; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &P[j] );
            N_DistPtPt( P[j], P[j - 1], &d );

            len += d;
        }
        sumv += len;

        if( len GT maxv )
            maxv = len;
    }

    /* Estimate area and length */

    *A = maxu * maxv;
    *lu = sumu / ((NL_REAL)m + 1.0);
    *lv = sumv / ((NL_REAL)n + 1.0);

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
/**********************************************************************/
/* ST_IsectLineWith2dPolygon: Intersect a line and a polygon in 2d            */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This tessellation  routine  intersects a  directed(horizontal or 
     vertical) line  segment with a closed  polygon in 2 - D. All special  
     cases, eg vertex intersection and line overlap, are accounted for. 
     A typical calling example is:

       NL_FLAG    *itp;
       NL_INDEX   *l, n, m; 
       NL_REAL    *x, *y, *u, *v, xs, ys, xe, ye, XL, XR, YB, YT;
       NL_REAL    srfUMin, srfUMax, srfVMin, srfVMax, tol;
       NL_STACKS  SG
       ...
(get x,..., tol);
       ...
       ST_IsectLineWith2dPolygon(xs, ys, xe, ye, NL_LEFT, x, y, n, XL, XR, YB, YT, tol, &u, &v, &l, &itp,
                &m, SG);

     THE POLYGON MUST BE CLOSED AND PROPERLY ORIENTED(EITHER CLOCKWISE
     OR COUNTERCLOCKWISE).


   ACCESS:
   
     xs, ys, xe, ye , input  ,  Start and end  coordinates of line segment
     flg         , input  ,  NL_FLAG:
                               NL_LEFT  : vertical line on the left
                               NL_RIGHT : vertical line on the right
                               NL_BOTTOM: horizontal line on the bottom
                               NL_TOP   : horizontal line on the top
     x, y         , input  ,  Coordinates of the vertices of the polygon
     n           , input  ,  Highest index in(x, y)
     XL, XR, YB, YT , input  ,  Bounding box of polygon
     srfUMin, srfUMax, srfVMin, srfVMax , input , parameter bounds of surface
     tol         , input  ,  Parameter space tolerance
     u, v         , output ,  Coordinates of intersection points
     l           , output ,  Leg index:(u[i], v[i]) lies on the polygon
                             leg(x[l[i]], y[l[i]])
     itp         , output ,  Intersection type:
                               NL_INP   : polygon enters half space
                               NL_OUTP  : polygon leaves half space
                               NL_TURNP : polygon turns on the line
                               NL_TOUCHP: polgon touches the line from the 
                                       right or  polygon  leg  overlaps 
                                       line
     m           , output ,  Highest index in(u, v), l and itp
     SG          , input  ,  Stack of output entities


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

static NL_FLAG ST_IsectLineWith2dPolygon( NL_REAL xs, NL_REAL ys, NL_REAL xe, NL_REAL ye,
    NL_FLAG flg,
    NL_REAL *x, NL_REAL *y, NL_INDEX n,
    NL_REAL XL, NL_REAL XR, NL_REAL YB, NL_REAL YT,
    NL_REAL srfUMin, NL_REAL srfUMax, NL_REAL srfVMin, NL_REAL srfVMax,
    NL_REAL tol,
    NL_REAL ** u, NL_REAL ** v, NL_INDEX ** l,
    NL_FLAG ** itp,
    NL_INDEX *m, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("ST_IsectLineWith2dPolygon");

    NL_FLAG *itl, *ito, sa, sb, sc = 0, sd = 0, ita, output, error = NL_NO;

    NL_INDEX *ll, *lo, i, j, k, max, np, ne;

    NL_REAL *ul, *vl, *uo, *vo, fac, a, b, c, d, xi, yi, xa, ya, xp, yp, xn, yn, lengtha, lengthb;

    NL_BOOLEAN boo;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Allocate memory */

    fac = NL_MAX( fabs( xe - xs ), fabs( ye - ys ) ) / NL_MAX( XR - XL, YT - YB );
    ne = (NL_INDEX)(NL_MIN( n, fac * n )) + 5;

    ul = N_AllocReal1dArray( ne, &SL );

    if( ul EQ NULL )
        NL_QUIT;

    vl = N_AllocReal1dArray( ne, &SL );

    if( vl EQ NULL )
        NL_QUIT;

    ll = N_AllocInt1dArray( ne, &SL );

    if( ll EQ NULL )
        NL_QUIT;

    itl = N_AllocFlag1dArray( ne, &SL );

    if( itl EQ NULL )
        NL_QUIT;

    /* Get all intersections */

    i = 0;
    max = n - 1;
    np = -1;

    xp = xe + ys - ye;
    yp = ye + xe - xs;
    xn = xs + ye - ys;
    yn = ys + xs - xe;

    lengthb = sqrt( (xe - xs) * (xe - xs) + (ye - ys) * (ye - ys) );

    while( i LE max )
    {
        /* See if current leg intersects line */

        boo = ST_TessLineXLine2d( x[i], y[i], x[i + 1], y[i + 1], xs, ys, xe, ye, tol, &xi, &yi, &a, &b );

        if( boo )
        {
            b = b * lengthb; /* modify b from normalized to true length */

            /* don't allow intersections at corners of domain that are */
            /* supposed to be outside                                  */

            if( b LT - 0.01 *tol OR b - lengthb GT 0.01 *tol )
            {
                if( flg EQ NL_BOTTOM )
                {
                    if( xs EQ srfUMin AND b LT tol )
                        boo = NL_FALSE;

                    if( xe EQ srfUMax AND b GT lengthb )
                        boo = NL_FALSE;
                }
                else if( flg EQ NL_TOP )
                {
                    if( xs EQ srfUMax AND b LT tol )
                        boo = NL_FALSE;

                    if( xe EQ srfUMin AND b GT lengthb )
                        boo = NL_FALSE;
                }
                else if( flg EQ NL_LEFT )
                {
                    if( ys EQ srfVMax AND b LT tol )
                        boo = NL_FALSE;

                    if( ye EQ srfVMin AND b GT lengthb )
                        boo = NL_FALSE;
                }
                else
                {
                    if( ys EQ srfVMin AND b LT tol )
                        boo = NL_FALSE;

                    if( ye EQ srfVMax AND b GT lengthb )
                        boo = NL_FALSE;
                }
            }
        }

        if( boo )
        {
            /* Classify intersection types */

            lengtha = sqrt( (x[i] - x[i + 1]) * (x[i] - x[i + 1]) + (y[i] - y[i + 1]) * (y[i] - y[i + 1]) );

            a = a * lengtha; /* modify a from normalized to true length */

            if( a GE tol AND a LE lengtha - tol )
            {
                /* Through intersection */

                sa = ST_TessSideTest2d( xs, ys, xe, ye, x[i + 1], y[i + 1] );

                output = NL_YES;

                if( b LT tol )
                {
                    if( sa EQ NL_LEFT )
                    {
                        sc = ST_TessSideTest2d( xs, ys, xn, yn, x[i + 1], y[i + 1] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;
                    }

                    if( sa EQ NL_RIGHT )
                    {
                        sc = ST_TessSideTest2d( xs, ys, xn, yn, x[i], y[i] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;
                    }

                    ST_TessOverlap( x[i + 1], y[i + 1], xs, ys, xe, ye, tol, flg, NL_START, &output );
                }
                else if( b GT lengthb - tol )
                {
                    if( sa EQ NL_LEFT )
                    {
                        sc = ST_TessSideTest2d( xe, ye, xp, yp, x[i + 1], y[i + 1] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;
                    }

                    if( sa EQ NL_RIGHT )
                    {
                        sc = ST_TessSideTest2d( xe, ye, xp, yp, x[i], y[i] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;
                    }

                    ST_TessOverlap( x[i + 1], y[i + 1], xs, ys, xe, ye, tol, flg, NL_END, &output );
                }

                if( output EQ NL_YES )
                {
                    np++;

                    error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    ul[np] = xi;
                    vl[np] = yi;
                    ll[np] = i % n;

                    if( sa EQ NL_LEFT )
                        itl[np] = NL_INP;
                    else
                        itl[np] = NL_OUTP;
                }

                i++;
            } /* end case: through intersection */
            else if( a LT tol AND i EQ 0 )
            {
                /* Corner or overlap intersection at the beginning */

                if( ST_TessLineXLine2d( x[n - 1], y[n - 1], x[0], y[0], xs, ys, xe, ye, tol, &xa, &ya, &c, &d ) )
                {
                    /* Intersects previous segment -> corner intersection */

                    sa = ST_TessSideTest2d( xs, ys, xe, ye, x[1], y[1] );
                    sb = ST_TessSideTest2d( xs, ys, xe, ye, x[n - 1], y[n - 1] );

                    output = NL_YES;

                    if( b LT tol )
                    {
                        sc = ST_TessSideTest2d( xs, ys, xn, yn, x[1], y[1] );
                        sd = ST_TessSideTest2d( xs, ys, xn, yn, x[n - 1], y[n - 1] );

                        if( sc EQ NL_RIGHT AND sd EQ NL_RIGHT )
                            output = NL_NO;
                        ST_TessOverlap( x[1], y[1], xs, ys, xe, ye, tol, flg, NL_START, &output );
                    }
                    else if( b GT lengthb - tol )
                    {
                        sc = ST_TessSideTest2d( xe, ye, xp, yp, x[1], y[1] );
                        sd = ST_TessSideTest2d( xe, ye, xp, yp, x[n - 1], y[n - 1] );

                        if( sc EQ NL_RIGHT AND sd EQ NL_RIGHT )
                            output = NL_NO;
                        ST_TessOverlap( x[1], y[1], xs, ys, xe, ye, tol, flg, NL_END, &output );
                    }

                    if( output EQ NL_YES )
                    {
                        np++;

                        error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ul[np] = xi;
                        vl[np] = yi;
                        ll[np] = 0;

                        if( sa EQ NL_RIGHT AND sb EQ NL_RIGHT )
                        {
                            itl[np] = NL_TOUCHP;
                        }
                        else if( b GE tol AND b LE lengthb - tol )
                        {
                            if( sa EQ NL_LEFT AND sb EQ NL_LEFT )
                                itl[np] = NL_TURNP;

                            else if( sa EQ NL_LEFT )
                                itl[np] = NL_INP;

                            else if( sb EQ NL_LEFT )
                                itl[np] = NL_OUTP;
                        }
                        else
                        {
                            if( sa EQ sb )
                            {
                                if( sc EQ NL_LEFT AND sd EQ NL_RIGHT )
                                    itl[np] = NL_INP;

                                else if( sc EQ NL_RIGHT AND sd EQ NL_LEFT )
                                    itl[np] = NL_OUTP;

                                else if( sc EQ NL_LEFT AND sd EQ NL_LEFT )
                                    itl[np] = NL_TURNP;
                            }
                            else
                            {
                                if( sa EQ NL_LEFT AND sc EQ NL_LEFT )
                                    itl[np] = NL_INP;
                                else
                                    itl[np] = NL_OUTP;
                            }
                        }
                    }
                } /* end case: intersect previous segment(corner) */
                else
                {
                    /* Does not intersect previous segment -> overlap case */

                    sa = ST_TessSideTest2d( xs, ys, xe, ye, x[1], y[1] );

                    output = NL_YES;

                    if( b LT tol )
                    {
                        sc = ST_TessSideTest2d( xs, ys, xn, yn, x[1], y[1] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;

                        ST_TessOverlap( x[1], y[1], xs, ys, xe, ye, tol, flg, NL_START, &output );
                    }
                    else if( b GT lengthb - tol )
                    {
                        sc = ST_TessSideTest2d( xe, ye, xp, yp, x[1], y[1] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;

                        ST_TessOverlap( x[1], y[1], xs, ys, xe, ye, tol, flg, NL_END, &output );
                    }

                    if( output EQ NL_YES )
                    {
                        np++;

                        error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ul[np] = xi;
                        vl[np] = yi;
                        ll[np] = 0;

                        if( sa EQ NL_LEFT )
                            itl[np] = NL_INP;

                        else if( sa EQ NL_RIGHT )
                            itl[np] = NL_TOUCHP;
                    }
                } /* end case: does not intx previous seg(overlap) */

                i++;
                max = n - 2;
            }
            else if( a LT tol AND i NEQ 0 )
            {
                /* Overlap case not at the beginning */

                sa = ST_TessSideTest2d( xs, ys, xe, ye, x[i + 1], y[i + 1] );

                output = NL_YES;

                if( b LT tol )
                {
                    sc = ST_TessSideTest2d( xs, ys, xn, yn, x[i + 1], y[i + 1] );

                    if( sc EQ NL_RIGHT )
                        output = NL_NO;

                    ST_TessOverlap( x[i + 1], y[i + 1], xs, ys, xe, ye, tol, flg, NL_START, &output );
                }
                else if( b GT lengthb - tol )
                {
                    sc = ST_TessSideTest2d( xe, ye, xp, yp, x[i + 1], y[i + 1] );

                    if( sc EQ NL_RIGHT )
                        output = NL_NO;

                    ST_TessOverlap( x[i + 1], y[i + 1], xs, ys, xe, ye, tol, flg, NL_END, &output );
                }

                if( output EQ NL_YES )
                {
                    np++;

                    error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    ul[np] = xi;
                    vl[np] = yi;
                    ll[np] = i % n;

                    if( sa EQ NL_LEFT )
                        itl[np] = NL_INP;

                    else if( sa EQ NL_RIGHT )
                        itl[np] = NL_TOUCHP;
                }

                i++;
            } /* end case: overlap not at beginning */
            else if( a GT lengtha - tol )
            {
                /* Corner or overlap not at the beginning */

                if( i LT n - 1 AND ST_TessLineXLine2d( x[i + 1], y[i + 1], x[i + 2], y[i + 2], xs, ys, xe, ye, tol, &xa, &ya, &c, &d ) )
                {
                    /* Intersects next segment -> corner intersection */

                    sa = ST_TessSideTest2d( xs, ys, xe, ye, x[i], y[i] );
                    sb = ST_TessSideTest2d( xs, ys, xe, ye, x[i + 2], y[i + 2] );

                    output = NL_YES;

                    if( b LT tol )
                    {
                        sc = ST_TessSideTest2d( xs, ys, xn, yn, x[i], y[i] );
                        sd = ST_TessSideTest2d( xs, ys, xn, yn, x[i + 2], y[i + 2] );

                        if( sc EQ NL_RIGHT AND sd EQ NL_RIGHT )
                            output = NL_NO;
                        ST_TessOverlap( x[i + 2], y[i + 2], xs, ys, xe, ye, tol, flg, NL_START, &output );
                    }
                    else if( b GT lengthb - tol )
                    {
                        sc = ST_TessSideTest2d( xe, ye, xp, yp, x[i], y[i] );
                        sd = ST_TessSideTest2d( xe, ye, xp, yp, x[i + 2], y[i + 2] );

                        if( sc EQ NL_RIGHT AND sd EQ NL_RIGHT )
                            output = NL_NO;
                        ST_TessOverlap( x[i + 2], y[i + 2], xs, ys, xe, ye, tol, flg, NL_END, &output );
                    }

                    if( output EQ NL_YES )
                    {
                        np++;

                        error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ul[np] = xi;
                        vl[np] = yi;
                        ll[np] = (i + 1) % n;

                        if( sa EQ NL_RIGHT AND sb EQ NL_RIGHT )
                        {
                            itl[np] = NL_TOUCHP;
                        }
                        else if( b GE tol AND b LE lengthb - tol )
                        {
                            if( sa EQ NL_LEFT AND sb EQ NL_LEFT )
                                itl[np] = NL_TURNP;

                            else if( sa EQ NL_LEFT )
                                itl[np] = NL_OUTP;

                            else if( sb EQ NL_LEFT )
                                itl[np] = NL_INP;
                        }
                        else
                        {
                            if( sa EQ sb )
                            {
                                if( sc EQ NL_LEFT AND sd EQ NL_RIGHT )
                                    itl[np] = NL_OUTP;

                                else if( sc EQ NL_RIGHT AND sd EQ NL_LEFT )
                                    itl[np] = NL_INP;

                                else if( sc EQ NL_LEFT AND sd EQ NL_LEFT )
                                    itl[np] = NL_TURNP;
                            }
                            else
                            {
                                if( sb EQ NL_LEFT AND sd EQ NL_LEFT )
                                    itl[np] = NL_INP;
                                else
                                    itl[np] = NL_OUTP;
                            }
                        }
                    }
                } /* end case: intx next segment(corner) */
                else
                {
                    /* Does not intersect next segment -> overlap case */

                    sa = ST_TessSideTest2d( xs, ys, xe, ye, x[i], y[i] );

                    output = NL_YES;

                    if( b LT tol )
                    {
                        sc = ST_TessSideTest2d( xs, ys, xn, yn, x[i], y[i] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;

                        ST_TessOverlap( x[i], y[i], xs, ys, xe, ye, tol, flg, NL_START, &output );
                    }
                    else if( b GT lengthb - tol )
                    {
                        sc = ST_TessSideTest2d( xe, ye, xp, yp, x[i], y[i] );

                        if( sc EQ NL_RIGHT )
                            output = NL_NO;

                        ST_TessOverlap( x[i], y[i], xs, ys, xe, ye, tol, flg, NL_END, &output );
                    }

                    if( output EQ NL_YES )
                    {
                        np++;

                        error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ul[np] = xi;
                        vl[np] = yi;
                        ll[np] = (i + 1) % n;

                        if( sa EQ NL_LEFT )
                            itl[np] = NL_OUTP;

                        if( sa EQ NL_RIGHT )
                            itl[np] = NL_TOUCHP;
                    }
                } /* end case: does not intx next segment(overlap) */

                i += 2;
            }
        }
        else
        {
            /* No intersection -> check overlap case */

            output = NL_NO;

            switch( flg )
            {
                case NL_LEFT:
                    if( fabs( x[i] - xs )LT tol )
                    {
                        if( ye LT y[i]AND y[i]LT ys )
                            output = NL_YES;
                    }
                    break;

                case NL_BOTTOM:
                    if( fabs( y[i] - ys )LT tol )
                    {
                        if( xs LT x[i]AND x[i]LT xe )
                            output = NL_YES;
                    }
                    break;

                case NL_RIGHT:
                    if( fabs( x[i] - xs )LT tol )
                    {
                        if( ys LT y[i]AND y[i]LT ye )
                            output = NL_YES;
                    }
                    break;

                case NL_TOP:
                    if( fabs( y[i] - ys )LT tol )
                    {
                        if( xe LT x[i]AND x[i]LT xs )
                            output = NL_YES;
                    }
                    break;
            }

            if( output EQ NL_YES )
            {
                np++;

                error = ST_ReallocArrays( &ul, &vl, &ll, &itl, &ne, np, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                ul[np] = x[i];
                vl[np] = y[i];
                ll[np] = i;

                if( ST_TessLineXLine2d( x[n - 1], y[n - 1], x[0], y[0], xs, ys, xe, ye, tol, &xa, &ya, &c, &d ) )
                {
                    sa = ST_TessSideTest2d( xs, ys, xe, ye, x[n - 1], y[n - 1] );

                    if( sa EQ NL_LEFT )
                        itl[np] = NL_OUTP;
                    else
                        itl[np] = NL_TOUCHP;

                    max = n - 2;
                }
                else
                {
                    itl[np] = NL_TOUCHP;
                }
            }

            i++;
        } /* end case: no intx, check overlap case */
    }

    /* Sort intersection points */

    switch( flg )
    {
        case NL_LEFT: /* Sort by decreasing y - value */
            for ( i = 1; i <= np; i++ )
            {
                b = vl[i];
                k = ll[i];
                ita = itl[i];
                j = i;

                while( j GE 1 AND vl[j - 1]LT b )
                {
                    vl[j] = vl[j - 1];
                    ll[j] = ll[j - 1];
                    itl[j] = itl[j - 1];
                    j--;
                }
                vl[j] = b;
                ll[j] = k;
                itl[j] = ita;
            }

            for ( i = 0; i <= np; i++ )
                ul[i] = xs;
            break;

        case NL_BOTTOM: /* Sort by increasing x - value */
            for ( i = 1; i <= np; i++ )
            {
                a = ul[i];
                k = ll[i];
                ita = itl[i];
                j = i;

                while( j GE 1 AND ul[j - 1]GT a )
                {
                    ul[j] = ul[j - 1];
                    ll[j] = ll[j - 1];
                    itl[j] = itl[j - 1];
                    j--;
                }
                ul[j] = a;
                ll[j] = k;
                itl[j] = ita;
            }

            for ( i = 0; i <= np; i++ )
                vl[i] = ys;
            break;

        case NL_RIGHT: /* Sort by increasing y - value */
            for ( i = 1; i <= np; i++ )
            {
                b = vl[i];
                k = ll[i];
                ita = itl[i];
                j = i;

                while( j GE 1 AND vl[j - 1]GT b )
                {
                    vl[j] = vl[j - 1];
                    ll[j] = ll[j - 1];
                    itl[j] = itl[j - 1];
                    j--;
                }
                vl[j] = b;
                ll[j] = k;
                itl[j] = ita;
            }

            for ( i = 0; i <= np; i++ )
                ul[i] = xs;
            break;

        case NL_TOP: /* Sort by decreasing x - value */
            for ( i = 1; i <= np; i++ )
            {
                a = ul[i];
                k = ll[i];
                ita = itl[i];
                j = i;

                while( j GE 1 AND ul[j - 1]LT a )
                {
                    ul[j] = ul[j - 1];
                    ll[j] = ll[j - 1];
                    itl[j] = itl[j - 1];
                    j--;
                }
                ul[j] = a;
                ll[j] = k;
                itl[j] = ita;
            }

            for ( i = 0; i <= np; i++ )
                vl[i] = ys;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Create output */

    if( np GE 0 )
    {
        uo = N_AllocReal1dArray( np, SG );

        if( uo EQ NULL )
            NL_QUIT;

        vo = N_AllocReal1dArray( np, SG );

        if( vo EQ NULL )
            NL_QUIT;

        lo = N_AllocInt1dArray( np, SG );

        if( lo EQ NULL )
            NL_QUIT;

        ito = N_AllocFlag1dArray( np, SG );

        if( ito EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= np; i++ )
        {
            uo[i] = ul[i];
            vo[i] = vl[i];
            lo[i] = ll[i];
            ito[i] = itl[i];
        }

        *u = uo;
        *v = vo;
        *l = lo;
        *itp = ito;
    }

    *m = np;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* ST_TessSideTest2d: Side test for 2d points and lines                */
/**********************************************************************/

static NL_FLAG ST_TessSideTest2d( NL_REAL xs, NL_REAL ys, NL_REAL xe, NL_REAL ye, NL_REAL x, NL_REAL y )
{
    NL_REAL vx, vy, wx, wy, crs;

    vx = xe - xs;
    vy = ye - ys;
    wx = x - xs;
    wy = y - ys;

    crs = vx * wy - vy * wx;

    if( crs GE 0.0 )
        return NL_LEFT;
    else
        return NL_RIGHT;
}

/**********************************************************************/
/* ST_TessOverlap: Overlap test                                       */
/**********************************************************************/

static NL_VOID ST_TessOverlap( NL_REAL x, NL_REAL y, NL_REAL xs, NL_REAL ys, NL_REAL xe, NL_REAL ye, NL_REAL tol, NL_FLAG flg, NL_FLAG pos, NL_FLAG *output )
{
    if( pos EQ NL_START )
    {
        switch( flg )
        {
            case NL_LEFT:
                if( fabs( y - ys )LT tol )
                    *output = NL_NO;
                break;

            case NL_BOTTOM:
                if( fabs( x - xs )LT tol )
                    *output = NL_NO;
                break;

            case NL_RIGHT:
                if( fabs( y - ys )LT tol )
                    *output = NL_NO;
                break;

            case NL_TOP:
                if( fabs( x - xs )LT tol )
                    *output = NL_NO;
                break;
        }
    }

    if( pos EQ NL_END )
    {
        switch( flg )
        {
            case NL_LEFT:
                if( fabs( y - ye )LT tol )
                    *output = NL_NO;
                break;

            case NL_BOTTOM:
                if( fabs( x - xe )LT tol )
                    *output = NL_NO;
                break;

            case NL_RIGHT:
                if( fabs( y - ye )LT tol )
                    *output = NL_NO;
                break;

            case NL_TOP:
                if( fabs( x - xe )LT tol )
                    *output = NL_NO;
                break;
        }
    }
}

/**********************************************************************/
/* ST_ReallocArrays: Reallocate memory for local arrays               */
/**********************************************************************/

NL_FLAG ST_ReallocArrays( NL_REAL ** u, NL_REAL ** v, NL_INDEX ** l, NL_FLAG ** f, NL_INDEX *ne, NL_INDEX np, NL_STACKS *S )
{
    NL_FLAG *fl, error = NL_NO;

    NL_INDEX *ll, nl;

    NL_REAL *ul, *vl;

    ul = *u;
    vl = *v;
    ll = *l;
    fl = *f;
    nl = *ne;

    if( np GT nl )
    {
        error = N_Realloc1dRealArray( &ul, nl, nl + nl, S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_Realloc1dRealArray( &vl, nl, nl + nl, S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_Realloc1dIntArray( &ll, nl, nl + nl, S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_Realloc1dFlagArray( &fl, nl, nl + nl, S );

        if( error EQ NL_YES )
            NL_OUT;

        nl += nl;
    }

    *u = ul;
    *v = vl;
    *l = ll;
    *f = fl;
    *ne = nl;

    EXIT:

    return (error);
}
/**********************************************************************/
/* N_TESSLINEXLINE2D: Intersect two 2d line segments                  */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This tessellation routine intersects two 2d line segments 

           <P1(x1, y1), P2(x2, y2)>  and  <P3(x3, y3), P4(x4, y4)>

     A typical calling example is:

       NL_REAL  x1, y1, x2, y2, x3, y3, x4, y4, x, y, tol, alf, bet;
       ...
(get x1,..., tol);
       ...
       if (ST_TessLineXLine2d(x1, y1, x2, y2, x3, y3, x4, y4, tol, &x, &y, &alf, &bet))
       --> 
           intersection stored in(x, y);

     COINCIDENT LINE SEGMENTS ARE CONSIDERED NON - INTERSECTING.
     

   ACCESS:
   
     x1, y1, x2, y2 , input  ,  Coordinates of  first  line's  end points
     x3, y3, x4, y4 , input  ,  Coordinates of  second line's end  points
     tol         , input  ,  Parameter tolerance
     x, y         , output ,  Coordinates of intersection point
     alf, bet     , output ,  Parameters corresponding to the intersec-
                             tion point:
                              (x, y) =(x1, y1) + alf*(x2 - x1, y2 - y1)
                                     =(x3, y3) + bet*(x4 - x3, y4 - y3)


   RETURN CODES:

     NL_YES: Segments intersect
     NL_NO : Segments DO NOT intersect

   ***********************************************************************/

static NL_BOOLEAN ST_TessLineXLine2d( NL_REAL x1, NL_REAL y1, NL_REAL x2, NL_REAL y2, NL_REAL x3, NL_REAL y3, NL_REAL x4, NL_REAL y4, NL_REAL tol, NL_REAL *x, NL_REAL *y, NL_REAL *alf, NL_REAL *bet )
{
    NL_REAL ax, ay, bx, by, cx, cy, den, numa, numb, xa, ya, xb, yb, d, lengtha, lengthb;

    /* Do bounding box check */

    if( NL_MAX( x3, x4 ) + tol LE NL_MIN( x1, x2 ) - tol )
        return NL_NO;

    if( NL_MAX( x1, x2 ) + tol LE NL_MIN( x3, x4 ) - tol )
        return NL_NO;

    if( NL_MAX( y3, y4 ) + tol LE NL_MIN( y1, y2 ) - tol )
        return NL_NO;

    if( NL_MAX( y1, y2 ) + tol LE NL_MIN( y3, y4 ) - tol )
        return NL_NO;

    /* Check intersection */

    ax = x2 - x1;
    ay = y2 - y1;
    bx = x3 - x4;
    by = y3 - y4;
    cx = x1 - x3;
    cy = y1 - y3;

    den = ay * bx - ax * by;

    if( fabs( den )LT NL_LTOL )
        return NL_NO;

    numa = by * cx - bx * cy;
    numb = ax * cy - ay * cx;

    if( N_FloatOpIsBad( numa, den, NL_DIVISION ) )
        return NL_NO;
    *alf = numa / den;
    d = 0.0;
    lengtha = sqrt( ax * ax + ay * ay );

    if( ( *alf)LT 0.0 )
        d = -( *alf) * lengtha;

    else if( ( *alf)GT 1.0 )
        d = (( *alf)-1.0)*lengtha;

    if( d GT tol )
        return NL_NO;

    if( N_FloatOpIsBad( numb, den, NL_DIVISION ) )
        return NL_NO;
    *bet = numb / den;
    d = 0.0;
    lengthb = sqrt( bx * bx + by * by );

    if( ( *bet)LT 0.0 )
        d = -( *bet) * lengthb;

    else if( ( *bet)GT 1.0 )
        d = (( *bet)-1.0)*lengthb;

    if( d GT tol )
        return NL_NO;

    xa = x1 + ( *alf) * ax;
    ya = y1 + ( *alf) * ay;
    xb = x3 - ( *bet) * bx;
    yb = y3 - ( *bet) * by;

    *x = 0.5 *(xa + xb);
    *y = 0.5 *(ya + yb);

    return NL_YES;
}
/**********************************************************************/
/* N_TESSCHECKTRIM: Error check for trimming curve polygonization            */
/**********************************************************************/
/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This tesselation  routine checks the  deviation of a trimming curve
     segment from the  chord joining the  end points. The current imple-
     mentation uses a  4 - point cubic  to  approximate the segment and to 
     check the chordal deviation. A typical calling example is:

       NL_SURFACE    sur;
       NL_CURVE      trm;
       NL_PARAMETER  ts, te;
       NL_REAL       dev;
       ...
(define sur, trm, and segment parameters ts and te);
       ...
       ST_TessCheckTrim(&sur, &trm, ts, te, &dev);


   ACCESS:
   
     sur   , input  ,  Trimmed surface
     trm   , input  ,  Trimming curve in sur's parameter domain
     ts, te , input  ,  Start and  end parameters specifying a segment of
                       the trimming curve
     dev   , output ,  Maximum  deviation of the segment  from the chord
                       sur(trm(ts)_x, trm(ts)_y), sur(trm(te)_x, trm(te)_y)
                       

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

static NL_FLAG ST_TessCheckTrim( NL_SURFACE *sur, NL_CURVE *trm, NL_PARAMETER ts, NL_PARAMETER te, NL_REAL *dev )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL t[4], dist, u, v, z, um = 0.0, vm = 0.0, f18, f38;

    NL_POINT Q[4], R, P;

    NL_CPOINT Pw[4];

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Sample segment */

    t[0] = ts;
    t[1] = (2.0 *ts + te) / 3.0;
    t[2] = (ts + 2.0 *te) / 3.0;
    t[3] = te;

    for ( i = 0; i <= 3; i++ )
    {
        error = N_CrvEval( trm, t[i], NL_LEFT, &P );

        if( error EQ NL_YES )
            NL_OUT;

        N_PtToXYZ( P, &u, &v, &z );
        N_ClampSrfAtParams( sur, &u, &v );

        error = N_SrfEvalPt( sur, u, v, NL_LEFT, NL_LEFT, &Q[i] );

        if( error EQ NL_YES )
            NL_OUT;

        if( i EQ 0 )
        {
            um = u;
            vm = v;
        }
        else if( i EQ 3 )
        {
            um = 0.5 *(u + um);
            vm = 0.5 *(v + vm);
        }
    }

    /* Get 4 - point cubic and check deviation */

    error = N_BezInterp4Pts( Q, Pw );

    if( error EQ NL_YES )
        NL_OUT;

    N_CPtToPtEuclid( Pw[0], &Q[0] );
    N_CPtToPtEuclid( Pw[3], &Q[3] );

    *dev = 0.0;

    for ( i = 1; i <= 2; i++ )
    {
        N_CPtToPtEuclid( Pw[i], &Q[i] );
        N_DistPtLineSeg( Q[i], Q[0], Q[3], &dist );

        if( dist GT *dev )
            *dev = dist;
    }

    /*  Check 3D image of midpoint of uv - chord against  */
    /*  midpoint of 3D chord                            */

    error = N_SrfEvalPt( sur, um, vm, NL_LEFT, NL_LEFT, &P );

    if( error EQ NL_YES )
        NL_OUT;

    f18 = 1. / 8.;
    f38 = 3. / 8.;

    N_Combine4Pts( f18, Q[0], f38, Q[1], f38, Q[2], f18, Q[3], &R );

    N_DistPtPt( P, R, &dist );

    if( dist GT *dev )
        *dev = dist;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* ST_TessBBoxXTrim: Intersect box with trimming polygons             */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine is for internal use only: see N_TessTrimmedSrf.
     This tessellation  routine intersects a box with trimming polygons.
     The  output is a  set of  directed  polygons so  that when marching
     along the  polygons, the valid  material is on  the left. A typical 
     calling example is:

       NL_INDEX      **TM, **p, *h, *g, *buf, *ins, bi, bs, bn, rtm, stm,
                  n, m, i, j, k, l, t;
       NL_PARAMETER  *u, *v, **x, **y
       NL_REAL       *UL, *UR, *VB, *VT, uMin, uMax, vMin, vMax, tol;
       NL_STACKS     SP, SG
       ...
(get x,..., tol);
       ...
       ST_TessBBoxXTrim(&u, &v, &n, TM, rtm, stm, i, j, k, l, NL_INP, x, y, h, m, UL, UR, VB, VT, tol,
                &buf, &bi, &bs, bn, &p, &g, &ins, &t, &SP, &SG);

     EACH POLYGON MUST BE CLOSED AND PROPERLY ORIENTED(EITHER CLOCKWISE
     OR COUNTERCLOCKWISE).


   ACCESS:
   
     u, v         , in/out ,  Points  in  parameter  space  created  with 
                             subdivision. This list is updated as points
                             of  polygon intersections and  vertices are
                             added
     pVtxCount           , in/out ,  Highest index in(u, v)
     TM          , input  ,  Topology matrix(see ST_TessSubdivideSrf)
     rtm, stm     , input  ,  Highest indexes in TM
     i, j, k, l     , input  ,  Indexes  of  lower  left  and  upper  right 
                             corners of box
     sta         , input  ,  Status of box:
                               NL_ONANDOVER: box  intersects  polygons  and 
                                          contains others
                               NL_OVER     : box contains polygons
                               NL_INP      : box  is  inside  the  trimming 
                                          domain
                               NL_ONP      : box intersects polygons
                               NL_OUTP     : box is outside the domain
     x, y         , input  ,  Coordinates of the vertices of the polygons
                             x[0], y[0] MUST BE THE OUTER BOUNDARY
     h           , input  ,  h[i] is the  highest index in(x[i], y[i]),
                             pointers to the i - th polygon
     m           , input  ,  Highest index in x[i], y[i] and  h[i], i.e.
                             there are(m + 1) trimming polygons
     UL, UR, UB, UT , input  ,  Bounding boxes of polygons
     srfUMin, srfUMax, srfVMin, srfVMax , input , parameter bounds of surface
     tol         , input  ,  Parameter space tolerance
     buf         , in/out ,  Index buffer
     bi, bs       , in/out ,  Buffer index and size
     bn          , input  ,  Buffer increment
     p           , output ,  Output polygons, ie  index arrays of points
     g           , output ,  Highest indexes in p[i], the pointer to the
                             i - th index array
     ins         , output ,  Inner status of polygon:
                               ins[i] =  k: p[i] is inner polygon and is
                                            contained in p[k]
                               ins[i] = -1: p[i] is outer(boundary) pol
     t           , output ,  Highest index in p and g[i], i.e. there are
                               (t + 1) output polygons
     SP          , input  ,  Stack of buf, p, g and ins
     SG          , input  ,  Stack of u and v


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

static NL_FLAG ST_TessBBoxXTrim( NL_PARAMETER ** u, NL_PARAMETER ** v, NL_INDEX *pVtxCount,
    NL_INDEX ** TM,
    NL_INDEX rtm, NL_INDEX stm,
    NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_INDEX l,
    NL_FLAG sta,
    NL_PARAMETER ** x, NL_PARAMETER ** y,
    NL_INDEX *h, NL_INDEX m,
    NL_REAL *UL, NL_REAL *UR, NL_REAL *VB, NL_REAL *VT,
    NL_REAL srfUMin, NL_REAL srfUMax, NL_REAL srfVMin, NL_REAL srfVMax,
    NL_REAL tol,
    NL_INDEX ** buf, NL_INDEX *bi, NL_INDEX *bs, NL_INDEX bn,
    NL_INDEX *** p, NL_INDEX ** g,
    NL_INDEX ** ins, NL_INDEX *t,
    NL_STACKS *SP, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("ST_TessBBoxXTrim");

    NL_FLAG ** itl, ** itr, ** itb, ** itt, *pfl, *L_S, *L_U, *B_S, *B_U, *R_S, *R_U, *T_S, *T_U, its, moved, side, pst, cross, goin, ploop, sturn, pinpol, fnd, error = NL_NO;

    NL_INDEX ** pl = NULL, *gl = NULL, ** po, *go, *inl = NULL, *ino, *L_I, *L_P, *L_L, *B_I, *B_P, *B_L, *ip, *R_I, *R_P, *R_L, *T_I, *T_P, *T_L, ** ll, ** lr, ** lb, ** lt, *ml, *mr, *mb, *mt, *rh, r, s = 0, sh, ri, a, b, c, i00, i11, sin, pin = 0, kl, kb, kr, kt, sp, pi, ipu, oh, li, nl, nh, ni, cri, sti, im, count;

    NL_REAL ** xl, ** yl, ** xr, ** yr, ** xb, ** yb, ** xt, ** yt, *ul, *vl, *A, xmin, xmax, ymin, ymax, fac, ba, crx, cry, bl, br, bb, bt, blt, brt, bbt, btt, mx, my;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize some variables */

    nh = *pVtxCount;
    nl = nh;
    ul = *u;
    vl = *v;
    i00 = TM[i][j];
    /* i10 = TM[k][j]; */
    /* i01 = TM[i][l]; */
    i11 = TM[k][l];
    bl = ul[i00];
    br = ul[i11];
    bb = vl[i00];
    bt = vl[i11];
    blt = bl + tol;
    brt = br - tol;
    bbt = bb + tol;
    btt = bt - tol;

    ni = -1;

    for ( a = 0; a <= m; a++ )
        ni += h[a] + 1;
    ni *= 2;

    /* Box is entirely inside the domain */

    if( sta EQ NL_INP )
    {
        /* Count the number of points */

        r = -1;

        for ( b = l; b > j; b-- )
            if( TM[i][b]GE 0 )
                r++;

        for ( a = i; a < k; a++ )
            if( TM[a][j]GE 0 )
                r++;

        for ( b = j; b < l; b++ )
            if( TM[k][b]GE 0 )
                r++;

        for ( a = k; a > i; a-- )
            if( TM[a][l]GE 0 )
                r++;

        /* Allocate memory */

        po = N_AllocIntPtr1dArray( 0, SP );

        if( po EQ NULL )
            NL_QUIT;

        po[0] = N_AllocInt1dArray( r, SP );

        if( po[0]EQ NULL )
            NL_QUIT;

        go = N_AllocInt1dArray( 0, SP );

        if( go EQ NULL )
            NL_QUIT;

        ino = N_AllocInt1dArray( 0, SP );

        if( ino EQ NULL )
            NL_QUIT;

        /* Output polygon */

        r = -1;

        for ( b = l; b > j; b-- )
            if( TM[i][b]GE 0 )
                po[0][++r] = TM[i][b];

        for ( a = i; a < k; a++ )
            if( TM[a][j]GE 0 )
                po[0][++r] = TM[a][j];

        for ( b = j; b < l; b++ )
            if( TM[k][b]GE 0 )
                po[0][++r] = TM[k][b];

        for ( a = k; a > i; a-- )
            if( TM[a][l]GE 0 )
                po[0][++r] = TM[a][l];

        go[0] = r;
        ino[0] = -1;

        *p = po;
        *g = go;
        *ins = ino;
        *t = 0;

        NL_OUT;
    }

    /* Box contains trimming polygons */

    if( sta EQ NL_OVER )
    {
        /* Get the number of polygons totally inside the box */

        pfl = N_AllocFlag1dArray( m, &SL );

        if( pfl EQ NULL )
            NL_QUIT;

        s = -1;

        for ( a = 0; a <= m; a++ )
        {
            if( rtm EQ 1 AND stm EQ 1 )
            { /* special case only 1 rectangle: 
               every polygon in */
                s++;
                pfl[a] = NL_YES;
            }
            else if( UL[a]GE blt AND UR[a]LE brt AND VB[a]GE bbt AND VT[a]LE btt )
            {
                s++;
                pfl[a] = NL_YES;
            }
            else
                pfl[a] = NL_NO;
        }

        if( pfl[0]NEQ NL_YES )
            s++;

        /* Allocate memory */

        po = N_AllocIntPtr1dArray( s, SP );

        if( po EQ NULL )
            NL_QUIT;

        go = N_AllocInt1dArray( s, SP );

        if( go EQ NULL )
            NL_QUIT;

        ino = N_AllocInt1dArray( s, SP );

        if( ino EQ NULL )
            NL_QUIT;

        /* Output the boundary first */

        s = -1;

        if( pfl[0]NEQ NL_YES )
        {
            r = -1;
            s++;

            for ( b = l; b > j; b-- )
                if( TM[i][b]GE 0 )
                    r++;

            for ( a = i; a < k; a++ )
                if( TM[a][j]GE 0 )
                    r++;

            for ( b = j; b < l; b++ )
                if( TM[k][b]GE 0 )
                    r++;

            for ( a = k; a > i; a-- )
                if( TM[a][l]GE 0 )
                    r++;

            po[s] = N_AllocInt1dArray( r, SP );

            if( po[s]EQ NULL )
                NL_QUIT;

            r = -1;

            for ( b = l; b > j; b-- )
                if( TM[i][b]GE 0 )
                    po[s][++r] = TM[i][b];

            for ( a = i; a < k; a++ )
                if( TM[a][j]GE 0 )
                    po[s][++r] = TM[a][j];

            for ( b = j; b < l; b++ )
                if( TM[k][b]GE 0 )
                    po[s][++r] = TM[k][b];

            for ( a = k; a > i; a-- )
                if( TM[a][l]GE 0 )
                    po[s][++r] = TM[a][l];

            go[s] = r;
            ino[s] = -1;
        }

        /* Output the inner loops next */

        for ( a = 0; a <= m; a++ )
        {
            if( pfl[a]EQ NL_YES )
            {
                s++;

                po[s] = N_AllocInt1dArray( h[a] - 1, SP );

                if( po[s]EQ NULL )
                    NL_QUIT;

                for ( r = 0; r < h[a]; r++ )
                {
                    error = ST_TessAppendPt( &ul, &vl, &nh, ni, x[a][r], y[a][r], &nl, SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    po[s][r] = nl;
                }

                go[s] = h[a] - 1;

                if( a EQ 0 )
                    ino[s] = -1;
                else
                    ino[s] = 0;
            }
        }

        /* Create output data */

        *p = po;
        *u = ul;
        *g = go;
        *v = vl;
        *ins = ino;
        *t = s;
        *pVtxCount = nl;

        NL_OUT;
    }

    /* Box intersects polygons and may contain some others */

    if( sta EQ NL_ONP OR sta EQ NL_ONANDOVER )
    {
        /* Estimate the highest index in each polygon */

        A = N_AllocReal1dArray( m, &SL );

        if( A EQ NULL )
            NL_QUIT;

        pfl = N_AllocFlag1dArray( m, &SL );

        if( pfl EQ NULL )
            NL_QUIT;

        s = 0;
        im = 2 * (k - i) + 2 * (l - j) + 1;
        ba = (br - bl) * (bt - bb);

        for ( a = 0; a <= m; a++ )
        {
            fac = 1.0 / ((UR[a] - UL[a]) * (VT[a] - VB[a]));
            N_BBox2dCalcOverlap( bl, br, bb, bt, UL[a], UR[a], VB[a], VT[a], tol, &A[a] );

            if( A[a]GE 0.0 )
            {
                b = (NL_INDEX)(((ba - A[a]) / ba) * im + 1);
                r = (NL_INDEX)(A[a] * fac * h[a] + 1);

                if( NL_MAX( r, b )GT s )
                    s = NL_MAX( r, b );
            }
        }

        ri = s + 1;

        /* Get intersection with each side */

        xl = N_AllocRealPtr1dArray( 8 * (m + 1), &SL );

        if( xl EQ NULL )
            NL_QUIT;

        yl = &xl[m + 1];
        xr = &yl[m + 1];
        yr = &xr[m + 1];
        xb = &yr[m + 1];
        yb = &xb[m + 1];
        xt = &yb[m + 1];
        yt = &xt[m + 1];

        ml = N_AllocInt1dArray( 4 * (m + 1), &SL );

        if( ml EQ NULL )
            NL_QUIT;

        mr = &ml[m + 1];
        mb = &mr[m + 1];
        mt = &mb[m + 1];

        ll = N_AllocIntPtr1dArray( 4 * (m + 1), &SL );

        if( ll EQ NULL )
            NL_QUIT;

        lr = &ll[m + 1];
        lb = &lr[m + 1];
        lt = &lb[m + 1];

        itl = N_AllocFlagPtr1dArray( 4 * (m + 1), &SL );

        if( itl EQ NULL )
            NL_QUIT;

        itr = &itl[m + 1];
        itb = &itr[m + 1];
        itt = &itb[m + 1];

        kl = kb = kr = kt = -1;

        for ( a = 0; a <= m; a++ )
        {
            ml[a] = mb[a] = mr[a] = mt[a] = -1; /* high index of intx points */

            if( A[a]GT - tol )
            {
                error = ST_IsectLineWith2dPolygon( bl, bt, bl, bb, NL_LEFT, x[a], y[a], h[a], UL[a], UR[a], VB[a], VT[a], srfUMin, srfUMax, srfVMin, srfVMax, tol, &xl[a], &yl[a], &ll[a], &itl[a], &ml[a], &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = ST_IsectLineWith2dPolygon( bl, bb, br, bb, NL_BOTTOM, x[a], y[a], h[a], UL[a], UR[a], VB[a], VT[a], srfUMin, srfUMax, srfVMin, srfVMax, tol, &xb[a], &yb[a], &lb[a], &itb[a], &mb[a], &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = ST_IsectLineWith2dPolygon( br, bb, br, bt, NL_RIGHT, x[a], y[a], h[a], UL[a], UR[a], VB[a], VT[a], srfUMin, srfUMax, srfVMin, srfVMax, tol, &xr[a], &yr[a], &lr[a], &itr[a], &mr[a], &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                error = ST_IsectLineWith2dPolygon( br, bt, bl, bt, NL_TOP, x[a], y[a], h[a], UL[a], UR[a], VB[a], VT[a], srfUMin, srfUMax, srfVMin, srfVMax, tol, &xt[a], &yt[a], &lt[a], &itt[a], &mt[a], &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                kl += ml[a] + 1;
                kb += mb[a] + 1;
                kr += mr[a] + 1;
                kt += mt[a] + 1;
            }
        }

        /* Intx points are in:(xl, yl) , (xb, yb) , (xr, yr) , (xt, yt) */
        /* The points are sorted based on direction of the side; i.e. */
        /* yl[0][0] > yl[0][1] */

        /* Merge lists of intx and box points */

        kl += l - j + 1;
        kb += k - i + 1;
        kr += l - j + 1;
        kt += k - i + 1;

        L_I = N_AllocInt1dArray( 3 * (kl + kb + kr + kt + 4), &SL );

        if( L_I EQ NULL )
            NL_QUIT;

        L_P = &L_I[kl + 1];
        L_L = &L_P[kl + 1];
        B_I = &L_L[kl + 1];
        B_P = &B_I[kb + 1];
        B_L = &B_P[kb + 1];
        R_I = &B_L[kb + 1];
        R_P = &R_I[kr + 1];
        R_L = &R_P[kr + 1];
        T_I = &R_L[kr + 1];
        T_P = &T_I[kt + 1];
        T_L = &T_P[kt + 1];

        L_S = N_AllocFlag1dArray( 2 * (kl + kb + kr + kt + 4), &SL );

        if( L_S EQ NULL )
            NL_QUIT;

        L_U = &L_S[kl + 1];
        B_S = &L_U[kl + 1];
        B_U = &B_S[kb + 1];
        R_S = &B_U[kb + 1];
        R_U = &R_S[kr + 1];
        T_S = &R_U[kr + 1];
        T_U = &T_S[kt + 1];

        ip = N_AllocInt1dArray( m, &SL );

        if( ip EQ NULL )
            NL_QUIT;

        /**************/
        /* Merge NL_LEFT */
        /**************/

        /* Find the highest intersection */

        for ( a = 0; a <= m; a++ )
            ip[a] = 0;

        pi = -1;
        ymax = bb - tol;

        for ( a = 0; a <= m; a++ )
        {
            if( ml[a]GE 0 )
            {
                if( yl[a][ip[a]]GT ymax )
                {
                    ymax = yl[a][ip[a]];
                    pi = a;
                }
            }
        }

        /* If no intersection, output box points */

        if( pi EQ - 1 )
        {
            kl = -1;

            for ( b = l; b >= j; b-- )
            {
                if( TM[i][b]GE 0 )
                {
                    kl++;
                    L_I[kl] = TM[i][b];
                    L_P[kl] = -1;
                    L_L[kl] = -1;
                    L_S[kl] = NL_NOSTATUS;
                    L_U[kl] = NL_NOTUSED;
                }
            }
        }
        else
        {
            /* Merge intersection points with box points */

            sp = l;
            kl = -1;
            its = NL_YES;

            while( sp GE j )
            {
                if( TM[i][sp]LT 0 )
                {
                    sp--;
                    continue;
                }

                kl++;

                if( its EQ NL_YES AND pi LT 0 )
                    NL_ERROR( NL_GEO_ERR );

                if( its EQ NL_YES AND fabs( vl[TM[i][sp]] - yl[pi][ip[pi]] )LT tol )
                { /* use intx point */
                    L_I[kl] = TM[i][sp];
                    L_P[kl] = pi;
                    L_L[kl] = ll[pi][ip[pi]];
                    L_S[kl] = itl[pi][ip[pi]];
                    L_U[kl] = NL_NOTUSED;

                    sp--;
                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }
                else if( its EQ NL_NO OR( its EQ NL_YES AND vl[TM[i][sp]]GT yl[pi][ip[pi]] ) )
                { /* box point */
                    L_I[kl] = TM[i][sp];
                    L_P[kl] = -1;
                    L_L[kl] = -1;
                    L_S[kl] = NL_NOSTATUS;
                    L_U[kl] = NL_NOTUSED;

                    sp--;
                    moved = NL_NO; /* stay at same intx point */
                }
                else
                {
                    fnd = NL_NO;

                    if( i NEQ 0 )
                    { /* look for intx point in buffer */
                        error = ST_TessBuffer( ul, vl, nl, xl[pi][ip[pi]], yl[pi][ip[pi]], tol, buf, bi, bs, bn, &fnd, &pin, SP );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    if( fnd EQ NL_NO )
                    { /* append this intx point to list */
                        error = ST_TessAppendPt( &ul, &vl, &nh, ni, xl[pi][ip[pi]], yl[pi][ip[pi]], &nl, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        pin = nl;
                    }

                    L_I[kl] = pin; /* add intx point to list */
                    L_P[kl] = pi;
                    L_L[kl] = ll[pi][ip[pi]];
                    L_S[kl] = itl[pi][ip[pi]];
                    L_U[kl] = NL_NOTUSED;

                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }

                if( moved EQ NL_YES )
                {
                    pi = -1;
                    ymax = bb - tol;
                    its = NL_NO;

                    for ( a = 0; a <= m; a++ )
                    {
                        if( ml[a]GE 0 AND ip[a]LE ml[a] )
                        {
                            if( yl[a][ip[a]]GT ymax )
                            {
                                ymax = yl[a][ip[a]];
                                pi = a;
                            }

                            its = NL_YES;
                        }
                    }
                }
            }
        }

        /****************/
        /* Merge NL_BOTTOM */
        /****************/

        /* Find the leftmost intersection */

        for ( a = 0; a <= m; a++ )
            ip[a] = 0;

        pi = -1;
        xmin = br + tol;

        for ( a = 0; a <= m; a++ )
        {
            if( mb[a]GE 0 )
            {
                if( xb[a][ip[a]]LT xmin )
                {
                    xmin = xb[a][ip[a]];
                    pi = a;
                }
            }
        }

        /* If no intersection, output box points */

        if( pi EQ - 1 )
        {
            kb = -1;

            for ( a = i; a <= k; a++ )
            {
                if( TM[a][j]GE 0 )
                {
                    kb++;
                    B_I[kb] = TM[a][j];
                    B_P[kb] = -1;
                    B_L[kb] = -1;
                    B_S[kb] = NL_NOSTATUS;
                    B_U[kb] = NL_NOTUSED;
                }
            }
        }
        else
        {
            /* Merge intersection points with box points */

            sp = i;
            kb = -1;
            its = NL_YES;

            while( sp LE k )
            {
                if( TM[sp][j]LT 0 )
                {
                    sp++;
                    continue;
                }

                kb++;

                if( its EQ NL_YES AND pi LT 0 )
                    NL_ERROR( NL_GEO_ERR );

                if( its EQ NL_YES AND fabs( ul[TM[sp][j]] - xb[pi][ip[pi]] )LT tol )
                { /* use intx point */
                    B_I[kb] = TM[sp][j];
                    B_P[kb] = pi;
                    B_L[kb] = lb[pi][ip[pi]];
                    B_S[kb] = itb[pi][ip[pi]];
                    B_U[kb] = NL_NOTUSED;

                    sp++;
                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }
                else if( its EQ NL_NO OR( its EQ NL_YES AND ul[TM[sp][j]]LT xb[pi][ip[pi]] ) )
                { /* box point */
                    B_I[kb] = TM[sp][j];
                    B_P[kb] = -1;
                    B_L[kb] = -1;
                    B_S[kb] = NL_NOSTATUS;
                    B_U[kb] = NL_NOTUSED;

                    sp++;
                    moved = NL_NO; /* stay at same intx point */
                }
                else
                {
                    fnd = NL_NO;

                    if( j NEQ 0 )
                    { /* look for intx point in buffer */
                        error = ST_TessBuffer( ul, vl, nl, xb[pi][ip[pi]], yb[pi][ip[pi]], tol, buf, bi, bs, bn, &fnd, &pin, SP );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    if( fnd EQ NL_NO )
                    { /* append this intx point to list */
                        error = ST_TessAppendPt( &ul, &vl, &nh, ni, xb[pi][ip[pi]], yb[pi][ip[pi]], &nl, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        pin = nl;
                    }

                    B_I[kb] = pin; /* add intx point to list */
                    B_P[kb] = pi;
                    B_L[kb] = lb[pi][ip[pi]];
                    B_S[kb] = itb[pi][ip[pi]];
                    B_U[kb] = NL_NOTUSED;

                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }

                if( moved EQ NL_YES )
                {
                    pi = -1;
                    xmin = br + tol;
                    its = NL_NO;

                    for ( a = 0; a <= m; a++ )
                    {
                        if( mb[a]GE 0 AND ip[a]LE mb[a] )
                        {
                            if( xb[a][ip[a]]LT xmin )
                            {
                                xmin = xb[a][ip[a]];
                                pi = a;
                            }

                            its = NL_YES;
                        }
                    }
                }
            }
        }

        /***************/
        /* Merge NL_RIGHT */
        /***************/

        /* Find the lowest intersection */

        for ( a = 0; a <= m; a++ )
            ip[a] = 0;

        pi = -1;
        ymin = bt + tol;

        for ( a = 0; a <= m; a++ )
        {
            if( mr[a]GE 0 )
            {
                if( yr[a][ip[a]]LT ymin )
                {
                    ymin = yr[a][ip[a]];
                    pi = a;
                }
            }
        }

        /* If no intersection, output box points */

        if( pi EQ - 1 )
        {
            kr = -1;

            for ( b = j; b <= l; b++ )
            {
                if( TM[k][b]GE 0 )
                {
                    kr++;
                    R_I[kr] = TM[k][b];
                    R_P[kr] = -1;
                    R_L[kr] = -1;
                    R_S[kr] = NL_NOSTATUS;
                    R_U[kr] = NL_NOTUSED;
                }
            }
        }
        else
        {
            /* Merge intersection points with box points */

            sp = j;
            kr = -1;
            its = NL_YES;

            while( sp LE l )
            {
                if( TM[k][sp]LT 0 )
                {
                    sp++;
                    continue;
                }

                kr++;

                if( its EQ NL_YES AND pi LT 0 )
                    NL_ERROR( NL_GEO_ERR );

                if( its EQ NL_YES AND fabs( vl[TM[k][sp]] - yr[pi][ip[pi]] )LT tol )
                { /* use intx point */
                    R_I[kr] = TM[k][sp];
                    R_P[kr] = pi;
                    R_L[kr] = lr[pi][ip[pi]];
                    R_S[kr] = itr[pi][ip[pi]];
                    R_U[kr] = NL_NOTUSED;

                    sp++;
                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }
                else if( its EQ NL_NO OR( its EQ NL_YES AND vl[TM[k][sp]]LT yr[pi][ip[pi]] ) )
                { /* box point */
                    R_I[kr] = TM[k][sp];
                    R_P[kr] = -1;
                    R_L[kr] = -1;
                    R_S[kr] = NL_NOSTATUS;
                    R_U[kr] = NL_NOTUSED;

                    sp++;
                    moved = NL_NO; /* stay at same intx point */
                }
                else
                {
                    fnd = NL_NO;

                    if( k NEQ rtm )
                    { /* look for intx point in buffer */
                        error = ST_TessBuffer( ul, vl, nl, xr[pi][ip[pi]], yr[pi][ip[pi]], tol, buf, bi, bs, bn, &fnd, &pin, SP );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    if( fnd EQ NL_NO )
                    { /* append this intx point to list */
                        error = ST_TessAppendPt( &ul, &vl, &nh, ni, xr[pi][ip[pi]], yr[pi][ip[pi]], &nl, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        pin = nl;
                    }

                    R_I[kr] = pin; /* add intx point to list */
                    R_P[kr] = pi;
                    R_L[kr] = lr[pi][ip[pi]];
                    R_S[kr] = itr[pi][ip[pi]];
                    R_U[kr] = NL_NOTUSED;

                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }

                if( moved EQ NL_YES )
                {
                    pi = -1;
                    ymin = bt + tol;
                    its = NL_NO;

                    for ( a = 0; a <= m; a++ )
                    {
                        if( mr[a]GE 0 AND ip[a]LE mr[a] )
                        {
                            if( yr[a][ip[a]]LT ymin )
                            {
                                ymin = yr[a][ip[a]];
                                pi = a;
                            }

                            its = NL_YES;
                        }
                    }
                }
            }
        }

        /*************/
        /* Merge NL_TOP */
        /*************/

        /* Find the rightmost intersection */

        for ( a = 0; a <= m; a++ )
            ip[a] = 0;

        pi = -1;
        xmax = bl - tol;

        for ( a = 0; a <= m; a++ )
        {
            if( mt[a]GE 0 )
            {
                if( xt[a][ip[a]]GT xmax )
                {
                    xmax = xt[a][ip[a]];
                    pi = a;
                }
            }
        }

        /* If no intersection, output box points */

        if( pi EQ - 1 )
        {
            kt = -1;

            for ( a = k; a >= i; a-- )
            {
                if( TM[a][l]GE 0 )
                {
                    kt++;
                    T_I[kt] = TM[a][l];
                    T_P[kt] = -1;
                    T_L[kt] = -1;
                    T_S[kt] = NL_NOSTATUS;
                    T_U[kt] = NL_NOTUSED;
                }
            }
        }
        else
        {
            /* Merge intersection points with box points */

            sp = k;
            kt = -1;
            its = NL_YES;

            while( sp GE i )
            {
                if( TM[sp][l]LT 0 )
                {
                    sp--;
                    continue;
                }

                kt++;

                if( its EQ NL_YES AND pi LT 0 )
                    NL_ERROR( NL_GEO_ERR );

                if( its EQ NL_YES AND fabs( ul[TM[sp][l]] - xt[pi][ip[pi]] )LT tol )
                { /* use intx point */
                    T_I[kt] = TM[sp][l];
                    T_P[kt] = pi;
                    T_L[kt] = lt[pi][ip[pi]];
                    T_S[kt] = itt[pi][ip[pi]];
                    T_U[kt] = NL_NOTUSED;

                    sp--;
                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }
                else if( its EQ NL_NO OR( its EQ NL_YES AND ul[TM[sp][l]]GT xt[pi][ip[pi]] ) )
                { /* box point */
                    T_I[kt] = TM[sp][l];
                    T_P[kt] = -1;
                    T_L[kt] = -1;
                    T_S[kt] = NL_NOSTATUS;
                    T_U[kt] = NL_NOTUSED;

                    sp--;
                    moved = NL_NO; /* stay at same intx point */
                }
                else
                {
                    fnd = NL_NO;

                    if( l NEQ stm )
                    { /* look for intx point in buffer */
                        error = ST_TessBuffer( ul, vl, nl, xt[pi][ip[pi]], yt[pi][ip[pi]], tol, buf, bi, bs, bn, &fnd, &pin, SP );

                        if( error EQ NL_YES )
                            NL_OUT;
                    }

                    if( fnd EQ NL_NO )
                    { /* append this intx point to list */
                        error = ST_TessAppendPt( &ul, &vl, &nh, ni, xt[pi][ip[pi]], yt[pi][ip[pi]], &nl, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        pin = nl;
                    }

                    T_I[kt] = pin; /* add intx point to list */
                    T_P[kt] = pi;
                    T_L[kt] = lt[pi][ip[pi]];
                    T_S[kt] = itt[pi][ip[pi]];
                    T_U[kt] = NL_NOTUSED;

                    ip[pi]++;
                    moved = NL_YES; /* go to next intx point */
                }

                if( moved EQ NL_YES )
                {
                    pi = -1;
                    xmax = bl - tol;
                    its = NL_NO;

                    for ( a = 0; a <= m; a++ )
                    {
                        if( mt[a]GE 0 AND ip[a]LE mt[a] )
                        {
                            if( xt[a][ip[a]]GT xmax )
                            {
                                xmax = xt[a][ip[a]];
                                pi = a;
                            }

                            its = NL_YES;
                        }
                    }
                }
            }
        }

        /************************************************************/
        /* All lists are created. Now trace lists and form polygons */
        /************************************************************/

        /* Find number of output polygons */

        sh = -1;

        for ( b = 0; b <= kl; b++ )
            if( L_S[b]EQ NL_INP OR L_S[b]EQ NL_TURNP )
                sh++;

        for ( a = 0; a <= kb; a++ )
            if( B_S[a]EQ NL_INP OR B_S[a]EQ NL_TURNP )
                sh++;

        for ( b = 0; b <= kr; b++ )
            if( R_S[b]EQ NL_INP OR R_S[b]EQ NL_TURNP )
                sh++;

        for ( a = 0; a <= kt; a++ )
            if( T_S[a]EQ NL_INP OR T_S[a]EQ NL_TURNP )
                sh++;

        /* Avoid touch cases */

        if( sh EQ - 1 )
        {
            /* Check if polygons are inside the box */

            pfl = N_AllocFlag1dArray( m, &SL );

            if( pfl EQ NULL )
                NL_QUIT;

            s = -1;

            for ( a = 0; a <= m; a++ )
            {
                if( UL[a]GE blt AND UR[a]LE brt AND VB[a]GE bbt AND VT[a]LE btt )
                {
                    s++;
                    pfl[a] = NL_YES;
                }
                else
                    pfl[a] = NL_NO;
            }

            if( s GT - 1 AND s LT m )
                c = s + 1;
            else
                c = s;

            if( s GE 0 )
            {
                po = N_AllocIntPtr1dArray( c, SP );

                if( po EQ NULL )
                    NL_QUIT;

                go = N_AllocInt1dArray( c, SP );

                if( go EQ NULL )
                    NL_QUIT;

                ino = N_AllocInt1dArray( c, SP );

                if( ino EQ NULL )
                    NL_QUIT;

                c = -1;

                if( pfl[0]NEQ NL_YES )
                {
                    r = -1;
                    c++;

                    for ( b = l; b > j; b-- )
                        if( TM[i][b]GE 0 )
                            r++;

                    for ( a = i; a < k; a++ )
                        if( TM[a][j]GE 0 )
                            r++;

                    for ( b = j; b < l; b++ )
                        if( TM[k][b]GE 0 )
                            r++;

                    for ( a = k; a > i; a-- )
                        if( TM[a][l]GE 0 )
                            r++;

                    po[c] = N_AllocInt1dArray( r, SP );

                    if( po[c]EQ NULL )
                        NL_QUIT;

                    r = -1;

                    for ( b = l; b > j; b-- )
                        if( TM[i][b]GE 0 )
                            po[c][++r] = TM[i][b];

                    for ( a = i; a < k; a++ )
                        if( TM[a][j]GE 0 )
                            po[c][++r] = TM[a][j];

                    for ( b = j; b < l; b++ )
                        if( TM[k][b]GE 0 )
                            po[c][++r] = TM[k][b];

                    for ( a = k; a > i; a-- )
                        if( TM[a][l]GE 0 )
                            po[c][++r] = TM[a][l];

                    go[c] = r;
                    ino[c] = -1;
                }

                for ( a = 0; a <= m; a++ )
                {
                    if( pfl[a]EQ NL_YES )
                    {
                        c++;

                        po[c] = N_AllocInt1dArray( h[a] - 1, SP );

                        if( po[c]EQ NULL )
                            NL_QUIT;

                        for ( r = 0; r < h[a]; r++ )
                        {
                            error = ST_TessAppendPt( &ul, &vl, &nh, ni, x[a][r], y[a][r], &nl, SG );

                            if( error EQ NL_YES )
                                NL_OUT;

                            po[c][r] = nl;
                        }

                        go[c] = h[a] - 1;

                        if( a EQ 0 )
                            ino[c] = -1;
                        else
                            ino[c] = 0;
                    }
                }

                *p = po;
                *u = ul;
                *g = go;
                *v = vl;
                *ins = ino;
                *t = c;
                *pVtxCount = nl;

                NL_OUT;
            }

            /* Check if box is inside the polygon */

            mx = 0.5 *(bl + br);
            my = 0.5 *(bb + bt);

            pinpol = NL_INP;

            for ( a = 0; a <= m; a++ )
            {
                pst = ST_IsPtInPolygon( mx, my, x[a], y[a], h[a], UL[a], UR[a], VB[a], VT[a], tol );

                if( a EQ 0 AND pst EQ NL_OUTP )
                {
                    pinpol = NL_OUTP;
                    break;
                }

                if( a GT 0 AND pst EQ NL_INP )
                {
                    pinpol = NL_OUTP;
                    break;
                }
            }

            if( pinpol EQ NL_OUTP )
            {
                *t = -1;
                *u = ul;
                *v = vl;
                *pVtxCount = nl;

                NL_OUT;
            }

            /* Switch to NL_INP case */

            r = kl + kb + kr + kt - 1;

            po = N_AllocIntPtr1dArray( 0, SP );

            if( po EQ NULL )
                NL_QUIT;

            po[0] = N_AllocInt1dArray( r, SP );

            if( po[0]EQ NULL )
                NL_QUIT;

            go = N_AllocInt1dArray( 0, SP );

            if( go EQ NULL )
                NL_QUIT;

            ino = N_AllocInt1dArray( 0, SP );

            if( ino EQ NULL )
                NL_QUIT;

            r = -1;

            for ( b = 0; b < kl; b++ )
                po[0][++r] = L_I[b];

            for ( a = 0; a < kb; a++ )
                po[0][++r] = B_I[a];

            for ( b = 0; b < kr; b++ )
                po[0][++r] = R_I[b];

            for ( a = 0; a < kt; a++ )
                po[0][++r] = T_I[a];

            go[0] = r;
            ino[0] = -1;

            *p = po;
            *u = ul;
            *g = go;
            *v = vl;
            *ins = ino;
            *t = 0;
            *pVtxCount = nl;

            NL_OUT;
        }

        /* Adjust number of NL_INP or NL_TURNP */

        ipu = sh + 1;

        if( (T_S[kt]EQ NL_INP AND L_S[0]EQ NL_INP)OR( T_S[kt]EQ NL_TURNP AND L_S[0]EQ NL_TURNP )OR( T_S[kt]EQ NL_INP AND L_S[0]EQ NL_TURNP )OR( T_S[kt]EQ NL_TURNP AND L_S[0]EQ NL_INP ) )
            ipu--;

        if( (L_S[kl]EQ NL_INP AND B_S[0]EQ NL_INP)OR( L_S[kl]EQ NL_TURNP AND B_S[0]EQ NL_TURNP )OR( L_S[kl]EQ NL_INP AND B_S[0]EQ NL_TURNP )OR( L_S[kl]EQ NL_TURNP AND B_S[0]EQ NL_INP ) )
            ipu--;

        if( (B_S[kb]EQ NL_INP AND R_S[0]EQ NL_INP)OR( B_S[kb]EQ NL_TURNP AND R_S[0]EQ NL_TURNP )OR( B_S[kb]EQ NL_INP AND R_S[0]EQ NL_TURNP )OR( B_S[kb]EQ NL_TURNP AND R_S[0]EQ NL_INP ) )
            ipu--;

        if( (R_S[kr]EQ NL_INP AND T_S[0]EQ NL_INP)OR( R_S[kr]EQ NL_TURNP AND T_S[0]EQ NL_TURNP )OR( R_S[kr]EQ NL_INP AND T_S[0]EQ NL_TURNP )OR( R_S[kr]EQ NL_TURNP AND T_S[0]EQ NL_INP ) )
            ipu--;

        /* Consider the NL_ONANDOVER case */

        oh = 0;

        if( sta EQ NL_ONANDOVER )
        {
            for ( a = 0; a <= m; a++ )
            {
                if( UL[a]GE blt AND UR[a]LE brt AND VB[a]GE bbt AND VT[a]LE btt )
                {
                    oh++;
                    pfl[a] = NL_YES;
                }
                else
                    pfl[a] = NL_NO;
            }
        }

        /* Allocate memory for output polygons */

        pl = N_AllocIntPtr1dArray( sh + oh + 1, &SL );

        if( pl EQ NULL )
            NL_QUIT;

        gl = N_AllocInt1dArray( sh + oh + 1, &SL );

        if( gl EQ NULL )
            NL_QUIT;

        inl = N_AllocInt1dArray( sh + oh + 1, &SL );

        if( inl EQ NULL )
            NL_QUIT;

        rh = N_AllocInt1dArray( sh + oh + 1, &SL );

        if( rh EQ NULL )
            NL_QUIT;

        for ( a = 0; a <= sh; a++ )
        {
            pl[a] = N_AllocInt1dArray( ri, &SL );

            if( pl[a]EQ NULL )
                NL_QUIT;

            rh[a] = ri;
        }

        /***************************************************************/
        /* While not all input points are used, create output polygons */
        /***************************************************************/

        s = -1; /* high index of output polygons */

        while( ipu GT 0 )
        {
            /* Find an unused NL_INP or NL_TURNP */

            error = ST_TessFindUnused( L_I, L_S, L_U, L_P, L_L, kl, B_I, B_S, B_U, B_P, B_L, kb, R_I, R_S, R_U, R_P, R_L, kr, T_I, T_S, T_U, T_P, T_L, kt, &pi, &li, &side, &sin, &pin );

            if( error EQ NL_YES )
                NL_ERROR( NL_GEO_ERR );

            switch( side )
            {
                case NL_LEFT:
                    L_U[sin] = NL_USED;

                    if( sin EQ kl )
                        B_U[0] = NL_USED;
                    break;

                case NL_BOTTOM:
                    B_U[sin] = NL_USED;

                    if( sin EQ kb )
                        R_U[0] = NL_USED;
                    break;

                case NL_RIGHT:
                    R_U[sin] = NL_USED;

                    if( sin EQ kr )
                        T_U[0] = NL_USED;
                    break;

                case NL_TOP:
                    T_U[sin] = NL_USED;

                    if( sin EQ kt )
                        L_U[0] = NL_USED;
                    break;
            }
            ipu--;

            /* Get start point */

            pst = NL_NOSTATUS;
            cross = NL_YES;
            s++;
            r = -1;    /* high index in pl[s] array(r + 1 distinct pts in output polgon) */
            cri = pin; /* current index */
            sti = pin; /* start index(IN- or TURN- point) */
            ploop = NL_YES;
            sturn = NL_NO;

            error = ST_TessAppendPolygonVertex( &pl[s], &rh[s], ri, pin, &r, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            a = (li + 1) % h[pi];
            crx = x[pi][a]; /* coord's of current trim polygon point */
            cry = y[pi][a]; /* may be corner or intx point once in inner loop */
            count = -1;

            while( cri NEQ sti OR ploop EQ NL_YES )
            {
                count += 1;

                if( count GT 4 * h[pi] )
                    NL_ERROR( NL_GEO_ERR );

                /* Case 1: 
            point is inside the box -> polygon point */

                if( ST_TessIsPtInBBox( crx, cry, bl, br, bb, bt, tol ) )
                {
                    error = ST_TessAppendPt( &ul, &vl, &nh, ni, crx, cry, &nl, SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    error = ST_TessAppendPolygonVertex( &pl[s], &rh[s], ri, nl, &r, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    li = (li + 1) % h[pi];
                    a = (li + 1) % h[pi];
                    cri = nl;

                    crx = x[pi][a];
                    cry = y[pi][a];
                }
                else

                /* Point is not in box -> cross side */

                if( cross EQ NL_YES )
                {
                    error = ST_TessFindCrossingPt( L_I, L_P, L_L, L_S, kl, B_I, B_P, B_L, B_S, kb, R_I, R_P, R_L, R_S, kr, T_I, T_P, T_L, T_S, kt, pi, h, li, cri, &side, &sin, &pin );

                    if( error EQ NL_YES )
                        NL_ERROR( NL_GEO_ERR );

                    switch( side )
                    {
                        case NL_LEFT:
                            pst = L_S[sin];
                            break;

                        case NL_BOTTOM:
                            pst = B_S[sin];
                            break;

                        case NL_RIGHT:
                            pst = R_S[sin];
                            break;

                        case NL_TOP:
                            pst = T_S[sin];
                            break;
                    }

                    crx = ul[pin];
                    cry = vl[pin];

                    if( pst EQ NL_TURNP AND pi EQ 0 )
                    {
                        switch( side )
                        {
                            case NL_LEFT:
                                L_U[sin] = NL_USED;
                                break;

                            case NL_BOTTOM:
                                B_U[sin] = NL_USED;
                                break;

                            case NL_RIGHT:
                                R_U[sin] = NL_USED;
                                break;

                            case NL_TOP:
                                T_U[sin] = NL_USED;
                                break;
                        }

                        ipu--;
                    }

                    cross = NL_NO;
                    cri = pin;

                    error = ST_TessAppendPolygonVertex( &pl[s], &rh[s], ri, pin, &r, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( ploop EQ NL_YES )
                    {
                        if( cri NEQ sti OR pst EQ NL_TURNP )
                            ploop = NL_NO;
                    }
                }
                else

                /* Point is on the side */

                if( NOT ST_TessIsPtBBoxCorner( side, sin, kl, kb, kr, kt )OR( pst EQ NL_TURNP AND pi EQ 0 )OR( pst EQ NL_TURNP AND sturn EQ NL_YES )OR pst EQ NL_INP )
                {
                    /* Point is NL_TURNP point or NL_INP -> go inside or go along the side */

                    goin = NL_NO;

                    if( pst EQ NL_INP )
                        goin = NL_YES;

                    else if( pst EQ NL_TURNP AND pi EQ 0 )
                        goin = NL_YES;

                    else if( pst EQ NL_TURNP AND sturn EQ NL_YES )
                        goin = NL_YES;

                    if( goin EQ NL_YES )
                    {
                        switch( side )
                        {
                            case NL_LEFT:
                                pi = L_P[sin];
                                li = L_L[sin];

                                if( cri NEQ sti )
                                    L_U[sin] = NL_USED;
                                break;

                            case NL_BOTTOM:
                                pi = B_P[sin];
                                li = B_L[sin];

                                if( cri NEQ sti )
                                    B_U[sin] = NL_USED;
                                break;

                            case NL_RIGHT:
                                pi = R_P[sin];
                                li = R_L[sin];

                                if( cri NEQ sti )
                                    R_U[sin] = NL_USED;
                                break;

                            case NL_TOP:
                                pi = T_P[sin];
                                li = T_L[sin];

                                if( cri NEQ sti )
                                    T_U[sin] = NL_USED;
                                break;
                        }

                        if( cri NEQ sti )
                        {
                            if( pst NEQ NL_TURNP OR pi NEQ 0 )
                                ipu--;
                        }

                        a = (li + 1) % h[pi];

                        crx = x[pi][a];
                        cry = y[pi][a];

                        cross = NL_YES;
                        pst = NL_NOSTATUS;
                        sturn = NL_NO;
                    }
                    else
                    {
                        /* Point is side point -> march along side */

                        switch( side )
                        {
                            case NL_LEFT:
                                cri = L_I[sin + 1];
                                pst = L_S[sin + 1];
                                break;

                            case NL_BOTTOM:
                                cri = B_I[sin + 1];
                                pst = B_S[sin + 1];
                                break;

                            case NL_RIGHT:
                                cri = R_I[sin + 1];
                                pst = R_S[sin + 1];
                                break;

                            case NL_TOP:
                                cri = T_I[sin + 1];
                                pst = T_S[sin + 1];
                                break;
                        }

                        crx = ul[cri];
                        cry = vl[cri];
                        sin++;

                        if( pst EQ NL_TURNP )
                            sturn = NL_YES;

                        if( cri NEQ sti )
                        {
                            error = ST_TessAppendPolygonVertex( &pl[s], &rh[s], ri, cri, &r, &SL );

                            if( error EQ NL_YES )
                                NL_OUT;

                            ploop = NL_NO;
                        }
                    }
                }
                else

                /* Point is corner point -> change direction */

                if( ST_TessIsPtBBoxCorner( side, sin, kl, kb, kr, kt ) )
                {
                    switch( side )
                    {
                        case NL_LEFT:
                            if( sin EQ 0 )
                            {
                                side = NL_LEFT;
                                cri = L_I[1];
                                pst = L_S[1];
                            }
                            else
                            {
                                side = NL_BOTTOM;
                                cri = B_I[1];
                                pst = B_S[1];
                            }
                            break;

                        case NL_BOTTOM:
                            if( sin EQ 0 )
                            {
                                side = NL_BOTTOM;
                                cri = B_I[1];
                                pst = B_S[1];
                            }
                            else
                            {
                                side = NL_RIGHT;
                                cri = R_I[1];
                                pst = R_S[1];
                            }
                            break;

                        case NL_RIGHT:
                            if( sin EQ 0 )
                            {
                                side = NL_RIGHT;
                                cri = R_I[1];
                                pst = R_S[1];
                            }
                            else
                            {
                                side = NL_TOP;
                                cri = T_I[1];
                                pst = T_S[1];
                            }
                            break;

                        case NL_TOP:
                            if( sin EQ 0 )
                            {
                                side = NL_TOP;
                                cri = T_I[1];
                                pst = T_S[1];
                            }
                            else
                            {
                                side = NL_LEFT;
                                cri = L_I[1];
                                pst = L_S[1];
                            }
                            break;
                    }

                    sin = 1;
                    crx = ul[cri];
                    cry = vl[cri];

                    if( pst EQ NL_TURNP )
                        sturn = NL_YES;

                    if( cri NEQ sti )
                    {
                        error = ST_TessAppendPolygonVertex( &pl[s], &rh[s], ri, cri, &r, &SL );

                        if( error EQ NL_YES )
                            NL_OUT;

                        ploop = NL_NO;
                    }
                }
            } /* End of inner while */

            gl[s] = r;
            inl[s] = -1;
        } /* End of outer while */

        /* Add polygons that are inside the box */

        if( sta EQ NL_ONANDOVER )
        {
            /* Append trimming polygon vertices to the polygon list */

            c = s;

            for ( a = 0; a <= m; a++ )
            {
                if( pfl[a]EQ NL_YES )
                {
                    s++;

                    pl[s] = N_AllocInt1dArray( h[a] - 1, &SL );

                    if( pl[s]EQ NULL )
                        NL_QUIT;

                    for ( r = 0; r < h[a]; r++ )
                    {
                        error = ST_TessAppendPt( &ul, &vl, &nh, ni, x[a][r], y[a][r], &nl, SG );

                        if( error EQ NL_YES )
                            NL_OUT;

                        pl[s][r] = nl;
                    }

                    gl[s] = h[a] - 1;

                    pin = pl[s][0];
                    mx = ul[pin];
                    my = vl[pin];

                    inl[s] = -1;

                    for ( b = 0; b <= c; b++ )
                    {
                        if( ST_IsPtInPoly( ul, vl, pl[b], gl[b], mx, my, tol )EQ NL_YES )
                            inl[s] = b;
                    }
                }
            }
        }
    } /* End of NL_ONP case */

    /* Elminate degenerate polygons and create output */

    r = -1;

    for ( a = 0; a <= s; a++ )
    {
        if( gl[a]GT 1 )
            r++;
    }

    if( r EQ - 1 )
    {
        *u = ul;
        *v = vl;
        *pVtxCount = nl;
        *t = -1;
        NL_OUT;
    }

    po = N_AllocIntPtr1dArray( r, SP );

    if( po EQ NULL )
        NL_QUIT;

    go = N_AllocInt1dArray( r, SP );

    if( go EQ NULL )
        NL_QUIT;

    ino = N_AllocInt1dArray( r, SP );

    if( ino EQ NULL )
        NL_QUIT;

    r = -1;

    for ( a = 0; a <= s; a++ )
    {
        if( gl[a]GT 1 )
        {
            r++;
            go[r] = gl[a];
            ino[r] = inl[a];

            po[r] = N_AllocInt1dArray( go[r], SP );

            if( po[r]EQ NULL )
                NL_QUIT;

            for ( b = 0; b <= go[r]; b++ )
                po[r][b] = pl[a][b];
        }
    }

    *p = po;
    *u = ul;
    *g = go;
    *v = vl;
    *ins = ino;
    *t = r;
    *pVtxCount = nl;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

/**********************************************************************/
/* ST_TessAppendPt: Append point to the output list                          */
/**********************************************************************/

NL_FLAG ST_TessAppendPt( NL_PARAMETER ** u, NL_PARAMETER ** v, NL_INDEX *nh, NL_INDEX ni, NL_PARAMETER x, NL_PARAMETER y, NL_INDEX *n, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INDEX mh, m;

    NL_REAL *ul, *vl;

    mh = *nh;
    m = *n;
    ul = *u;
    vl = *v;

    m++;

    if( m GT mh )
    {
        error = N_Realloc1dRealArray( &ul, mh, mh + ni, S );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_Realloc1dRealArray( &vl, mh, mh + ni, S );

        if( error EQ NL_YES )
            NL_OUT;

        mh += ni;
    }

    ul[m] = x;
    vl[m] = y;

    *nh = mh;
    *n = m;
    *u = ul;
    *v = vl;

    EXIT:

    return (error);
}

/*************************************************************************/
/* ST_TessBuffer: Buffer intersection point or check if computed already */
/*************************************************************************/

NL_FLAG ST_TessBuffer( NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX n, NL_PARAMETER x, NL_PARAMETER y, NL_REAL tol, NL_INDEX ** buf, NL_INDEX *bi, NL_INDEX *bs, NL_INDEX bn, NL_FLAG *fnd, NL_INDEX *k, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *bfl, bli, bls, i, j, f;

    NL_REAL d2, t2;

    bfl = *buf;
    bli = *bi;
    bls = *bs;
    t2 = tol * tol;

    f = -1;

    for ( i = 0; i <= bli; i++ )
    {
        j = bfl[i];
        d2 = (x - u[j]) * (x - u[j]) + (y - v[j]) * (y - v[j]);

        if( d2 LT t2 )
        {
            *k = j;
            f = i;
            break;
        }
    }

    if( f GE 0 )
    { /* delete point from buffer, then exit */
        for ( i = f + 1; i <= bli; i++ )
            bfl[i - 1] = bfl[i];
        bli--;

        *buf = bfl;
        *bi = bli;
        *fnd = NL_YES;
        NL_OUT;
    }

    /* add point to buffer */

    bli++;

    if( bli GT bls )
    {
        error = N_Realloc1dIntArray( &bfl, bls, bls + bn, S );

        if( error EQ NL_YES )
            NL_OUT;

        bls += bn;
    }

    bfl[bli] = n + 1;

    *buf = bfl;
    *bi = bli;
    *bs = bls;
    *fnd = NL_NO;

    EXIT:

    return (error);
}

/**********************************************************************/
/* ST_TessFindUnused: Find not used NL_INP or NL_TURNP                */
/**********************************************************************/

NL_FLAG ST_TessFindUnused( NL_INDEX *li, NL_FLAG *ls, NL_FLAG *lu, NL_INDEX *lp, NL_INDEX *ll, NL_INDEX kl, NL_INDEX *bi, NL_FLAG *bs, NL_FLAG *bu, NL_INDEX *bp, NL_INDEX *bl, NL_INDEX kb, NL_INDEX *ri, NL_FLAG *rs, NL_FLAG *ru, NL_INDEX *rp, NL_INDEX *rl, NL_INDEX kr, NL_INDEX *ti, NL_FLAG *ts, NL_FLAG *tu, NL_INDEX *tp, NL_INDEX *tl, NL_INDEX kt, NL_INDEX *p, NL_INDEX *l, NL_FLAG *side, NL_INDEX *sin, NL_INDEX *pin )
{
    NL_FLAG found = NL_NO;

    NL_INDEX r;

    /* Search for an unused NL_INP first */

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kl; r++ )
        {
            if( ls[r]EQ NL_INP AND lu[r]EQ NL_NOTUSED )
            {
                *p = lp[r];
                *l = ll[r];
                *sin = r;
                *pin = li[r];
                *side = NL_LEFT;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kb; r++ )
        {
            if( bs[r]EQ NL_INP AND bu[r]EQ NL_NOTUSED )
            {
                *p = bp[r];
                *l = bl[r];
                *sin = r;
                *pin = bi[r];
                *side = NL_BOTTOM;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kr; r++ )
        {
            if( rs[r]EQ NL_INP AND ru[r]EQ NL_NOTUSED )
            {
                *p = rp[r];
                *l = rl[r];
                *sin = r;
                *pin = ri[r];
                *side = NL_RIGHT;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kt; r++ )
        {
            if( ts[r]EQ NL_INP AND tu[r]EQ NL_NOTUSED )
            {
                *p = tp[r];
                *l = tl[r];
                *sin = r;
                *pin = ti[r];
                *side = NL_TOP;
                found = NL_YES;
                break;
            }
        }
    }

    /* Now search for an unused NL_TURNP */

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kl; r++ )
        {
            if( ls[r]EQ NL_TURNP AND lu[r]EQ NL_NOTUSED )
            {
                *p = lp[r];
                *l = ll[r];
                *sin = r;
                *pin = li[r];
                *side = NL_LEFT;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kb; r++ )
        {
            if( bs[r]EQ NL_TURNP AND bu[r]EQ NL_NOTUSED )
            {
                *p = bp[r];
                *l = bl[r];
                *sin = r;
                *pin = bi[r];
                *side = NL_BOTTOM;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kr; r++ )
        {
            if( rs[r]EQ NL_TURNP AND ru[r]EQ NL_NOTUSED )
            {
                *p = rp[r];
                *l = rl[r];
                *sin = r;
                *pin = ri[r];
                *side = NL_RIGHT;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kt; r++ )
        {
            if( ts[r]EQ NL_TURNP AND tu[r]EQ NL_NOTUSED )
            {
                *p = tp[r];
                *l = tl[r];
                *sin = r;
                *pin = ti[r];
                *side = NL_TOP;
                found = NL_YES;
                break;
            }
        }
    }

    if( found EQ NL_YES )
        return NL_NO;
    else
        return NL_YES;
}

/**********************************************************************/
/* ST_TessAppendPolygonVertex: Append polygon vertex to the output list                 */
/**********************************************************************/

NL_FLAG ST_TessAppendPolygonVertex( NL_INDEX ** p, NL_INDEX *rh, NL_INDEX ri, NL_INDEX i, NL_INDEX *r, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *q, sh, s;

    sh = *rh;
    s = *r;
    q = *p;

    s++;

    if( s GT sh )
    {
        error = N_Realloc1dIntArray( &q, sh, sh + ri, S );

        if( error EQ NL_YES )
            NL_OUT;

        sh += ri;
    }

    q[s] = i;

    *rh = sh;
    *r = s;
    *p = q;

    EXIT:

    return (error);
}

/**********************************************************************/
/* ST_TessIsPtInBBox: Is point inside a box?                                   */
/**********************************************************************/

static NL_BOOLEAN ST_TessIsPtInBBox( NL_REAL x, NL_REAL y, NL_REAL xl, NL_REAL xr, NL_REAL yb, NL_REAL yt, NL_REAL tol )
{
    if( x LT xl + tol OR x GT xr - tol )
        return NL_NO;

    if( y LT yb + tol OR y GT yt - tol )
        return NL_NO;

    return NL_YES;
}

/**********************************************************************/
/* ST_TessIsPtBBoxCorner: Does point lie at the corner of the box?                 */
/**********************************************************************/

static NL_BOOLEAN ST_TessIsPtBBoxCorner( NL_FLAG side, NL_INDEX i, NL_INDEX kl, NL_INDEX kb, NL_INDEX kr, NL_INDEX kt )
{
    switch( side )
    {
        case NL_LEFT:
            if( i EQ 0 OR i EQ kl )
                return NL_YES;
            break;

        case NL_BOTTOM:
            if( i EQ 0 OR i EQ kb )
                return NL_YES;
            break;

        case NL_RIGHT:
            if( i EQ 0 OR i EQ kr )
                return NL_YES;
            break;

        case NL_TOP:
            if( i EQ 0 OR i EQ kt )
                return NL_YES;
            break;
    }

    return NL_NO;
}

/**********************************************************************/
/* ST_IsPtInPolygon: Does point lie inside a polygon?                         */
/**********************************************************************/

NL_FLAG ST_IsPtInPolygon( NL_REAL xp, NL_REAL yp, NL_REAL *x, NL_REAL *y, NL_INDEX n, NL_REAL XL, NL_REAL XR, NL_REAL YB, NL_REAL YT, NL_REAL tol )
{
    NL_INDEX i, k;

    NL_REAL xi, yt;

    if( xp LT XL - tol OR xp GT XR + tol OR yp LT YB - tol OR yp GT YT + tol )
    {
        return NL_OUTP;
    }

    k = 0;
    yt = yp + tol;

    for ( i = 1; i <= n; i++ )
    {
        if( (y[i]GT yp AND y[i - 1]LT yt)OR( y[i - 1]GT yp AND y[i]LT yt ) )
        {
            xi = x[i] + ((yp - y[i]) * (x[i] - x[i - 1])) / (y[i] - y[i - 1]);

            if( xi GT xp )
                k++;
        }
    }

    if( k % 2 )
        return NL_INP;
    else
        return NL_OUTP;
}

/**********************************************************************/
/* ST_IsPtInPoly: Point in polygon test using index array                  */
/**********************************************************************/

static NL_BOOLEAN ST_IsPtInPoly( NL_PARAMETER *u, NL_PARAMETER *v, NL_INDEX *p, NL_INDEX n, NL_PARAMETER x, NL_PARAMETER y, NL_REAL tol )
{
    NL_INDEX i, k, j, j1;

    NL_REAL xi, yt;

    k = 0;
    yt = y + tol;

    for ( i = 1; i <= n + 1; i++ )
    {
        if( i EQ n + 1 )
            j = p[0];
        else
            j = p[i];
        j1 = p[i - 1];

        if( (v[j]GT y AND v[j1]LT yt)OR( v[j1]GT y AND v[j]LT yt ) )
        {
            xi = u[j] + ((y - v[j]) * (u[j] - u[j1])) / (v[j] - v[j1]);

            if( xi GT x )
                k++;
        }
    }

    if( k % 2 )
        return NL_YES;
    else
        return NL_NO;
}

/**********************************************************************/
/* ST_TessFindCrossingPt: Find crossing point                                      */
/**********************************************************************/

NL_FLAG ST_TessFindCrossingPt( NL_INDEX *li, NL_INDEX *lp, NL_INDEX *ll, NL_FLAG *ls, NL_INDEX kl, NL_INDEX *bi, NL_INDEX *bp, NL_INDEX *bl, NL_FLAG *bs, NL_INDEX kb, NL_INDEX *ri, NL_INDEX *rp, NL_INDEX *rl, NL_FLAG *rs, NL_INDEX kr, NL_INDEX *ti, NL_INDEX *tp, NL_INDEX *tl, NL_FLAG *ts, NL_INDEX kt, NL_INDEX p, NL_INDEX *h, NL_INDEX l, NL_INDEX s, NL_FLAG *side, NL_INDEX *sin, NL_INDEX *pin )
{
    NL_FLAG ps[4], sd[4], found = NL_NO, same;

    NL_INDEX r, pi[4], si[4], n, m;

    /* First try to find on current leg */

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kl; r++ )
        {
            if( lp[r]EQ p AND ll[r]EQ l )
            {
                if( li[r]NEQ s )
                {
                    *sin = r;
                    *pin = li[r];
                    *side = NL_LEFT;
                    found = NL_YES;
                    break;
                }
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kb; r++ )
        {
            if( bp[r]EQ p AND bl[r]EQ l )
            {
                if( bi[r]NEQ s )
                {
                    *sin = r;
                    *pin = bi[r];
                    *side = NL_BOTTOM;
                    found = NL_YES;
                    break;
                }
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kr; r++ )
        {
            if( rp[r]EQ p AND rl[r]EQ l )
            {
                if( ri[r]NEQ s )
                {
                    *sin = r;
                    *pin = ri[r];
                    *side = NL_RIGHT;
                    found = NL_YES;
                    break;
                }
            }
        }
    }

    if( found EQ NL_NO )
    {
        for ( r = 0; r <= kt; r++ )
        {
            if( tp[r]EQ p AND tl[r]EQ l )
            {
                if( ti[r]NEQ s )
                {
                    *sin = r;
                    *pin = ti[r];
                    *side = NL_TOP;
                    found = NL_YES;
                    break;
                }
            }
        }
    }

    /* Now try the next leg to account for turning points */

    if( found EQ NL_NO )
    {
        n = -1;

        for ( r = 0; r <= kl; r++ )
        {
            if( lp[r]EQ p AND ll[r]EQ( l + 1 ) % h[p] )
            {
                if( li[r]NEQ s )
                {
                    n++;
                    si[n] = r;
                    pi[n] = li[r];
                    sd[n] = NL_LEFT;
                    ps[n] = ls[r];
                }
            }
        }

        for ( r = 0; r <= kb; r++ )
        {
            if( bp[r]EQ p AND bl[r]EQ( l + 1 ) % h[p] )
            {
                if( bi[r]NEQ s )
                {
                    n++;
                    si[n] = r;
                    pi[n] = bi[r];
                    sd[n] = NL_BOTTOM;
                    ps[n] = bs[r];
                }
            }
        }

        for ( r = 0; r <= kr; r++ )
        {
            if( rp[r]EQ p AND rl[r]EQ( l + 1 ) % h[p] )
            {
                if( ri[r]NEQ s )
                {
                    n++;
                    si[n] = r;
                    pi[n] = ri[r];
                    sd[n] = NL_RIGHT;
                    ps[n] = rs[r];
                }
            }
        }

        for ( r = 0; r <= kt; r++ )
        {
            if( tp[r]EQ p AND tl[r]EQ( l + 1 ) % h[p] )
            {
                if( ti[r]NEQ s )
                {
                    n++;
                    si[n] = r;
                    pi[n] = ti[r];
                    sd[n] = NL_TOP;
                    ps[n] = ts[r];
                }
            }
        }

        if( n GE 0 )
            found = NL_YES;
        else
            found = NL_NO;

        if( found EQ NL_YES )
        {
            same = NL_YES;

            for ( r = 0; r < n; r++ )
            {
                if( pi[r]NEQ pi[r + 1] )
                {
                    same = NL_NO;
                    break;
                }
            }

            if( n EQ 0 OR same EQ NL_YES )
            {
                *sin = si[0];
                *pin = pi[0];
                *side = sd[0];
            }
            else
            {
                m = -1;

                for ( r = 0; r <= n; r++ )
                    if( ps[r]EQ NL_TURNP )
                    {
                        m = r;
                        break;
                    }

                if( m EQ - 1 )
                {
                    found = NL_NO;
                }
                else
                {
                    *sin = si[m];
                    *pin = pi[m];
                    *side = sd[m];
                }
            }
        }
    }

    if( found EQ NL_YES )
        return NL_NO;
    else
        return NL_YES;
}

/**********************************************************************/
/* ST_TessPolygonizeTrimCrv: Polygonize trimming curve                                */
/**********************************************************************/
NL_PRIVATE NL_REAL prc = 0.01;

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine is for internal use only: see N_TessTrimmedSrf.
     This tesselation routine polygonizes a trimming  curve given by a
     NURBS  curve in the  surface's parameter domain. More  precisely,
     given a model space tolerance eps, the routine computes a polygon
     that does not deviate from the 3D trimming curve more  than eps.
     A typical calling example is:
 
       NL_SURFACE  sur;
       NL_CURVE    trm;
       NL_REAL     *u, *v, lu, lv, eps;
       NL_INDEX    n;
       NL_STACKS   SG;
       ...
(define sur, trm, and get eps);
       ...
       ST_TessPolygonizeTrimCrv(&sur, &trm, eps, &u, &v, &n, &SG);
 
     MEMORY TO  STORE THE NL_PARAMETERS u[0],..., v[n] IS ALLOCATED INSIDE 
     THE ROUTINE. THIS ROUTINE  ACCOUNTS FOR VERY TINY TRIMMING CURVES
     BY OVERWRITING THE INPUT TOLERANCE TO APPROXIMATELY 1% OF THE NL_MIN
     NL_MAX BOX  OF THE  TRIMMING  NL_CURVE IN  THE NL_PARAMETER  SPACE. IF THE 
     INPUT TOLERANCE IS SMALLER THAN THIS VALUE, IT IS TAKEN. HOWEVER,
     IF IT IS LARGER, IT WILL BE REPLACED.
 
 
   ACCESS:
   
     sur  , input  ,  Trimmed surface
     lu,lv, input  ,  Average lengths of surface in u and v
     trm  , input  ,  Trimming curve in sur's parameter domain
     eps  , input  ,  Maximum  deviation  between  the trimming  curve 
                      and any of the polygon legs
     u, v  , output ,  Parameters  corresponding  to  the approximating 
                      polygon
     n    , output ,  Highest index in(u, v)
     SG   , input  , (u, v)'s memory stack
                       
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR

    NOTE:  The trimming curve must be g1 or better. A crease causes a 
    failure. Also the surface should be parameterised nicely in uv
          (ie ratio of u to v equivalent to chordlength)
    Also too large a value of eps will exit - out.
    so:    eps = 0.1;
           n =0;
           while (n <1) 
       {
       NL_PRIVATE  NL_STRING  rname = "ST_TessPolygonizeTrimCrv");
       eps = eps*0.1;
       error =  ST_TessPolygonizeTrimCrv(&sur, &Cur, eps, &u, &v, &n, &S);
       }
 
   ***********************************************************************/

static NL_FLAG ST_TessPolygonizeTrimCrv( NL_SURFACE *sur, NL_REAL lu, NL_REAL lv, NL_CURVE *trm, NL_REAL eps, NL_PARAMETER ** u, NL_PARAMETER ** v, NL_INDEX *n, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("ST_TessPolygonizeTrimCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k, l, m, r, is, ie, nh, nhg, nn, mm, count;

    NL_DEGREE p;

    NL_REAL *T, *U, *us, *vs, len, t, dt, dtg, dist = 0.0, eplh, ts, te, dot, z, xl, xr, yb, yt, fac, epl, epa, mindt, d1, d2;

    NL_POINT Ps, Pe, C;

    NL_CPOINT ** Pw, *Cw;

    NL_VECTOR V1, V2;

    NL_CURVE ** cur;

    NL_BOOLEAN next_seg;

    NL_MINMAXBOX box;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    if( eps LT NL_MTOL )
        NL_ERROR( NL_TOL_ERR );

    N_CrvGetKnots( trm, &m, &T );
    N_CrvGetDegree( trm, &p );
    epl = eps;

    /* Treat degree 1 curve and rectangular plane as special case */

    if( p EQ 1 )
    {
        N_SrfGetCPts( sur, &nn, &mm, &Pw );

        if( nn EQ 1 AND mm EQ 1 )
        {
            N_VectorDiffCPts( Pw[0][0], Pw[1][0], &V1 );
            N_VectorDiffCPts( Pw[0][0], Pw[0][1], &V2 );
            N_VectorDot( V1, V2, &dot );
            N_VectorDot( V1, V1, &d1 );
            N_VectorDot( V2, V2, &d2 );

            d1 = sqrt( d1 * d2 );

            if( NOT( N_FloatOpIsBad( dot, d1, NL_DIVISION ) ) )
            {
                d2 = dot / d1;           /* cosine */

                if( fabs( d2 )LT 0.002 ) /* consider them perpendicular */
                {                        /* just load the control points of the trim curve */
                    N_CrvGetCPts( trm, &nn, &Cw );
                    us = N_AllocReal1dArray( nn, SG );

                    if( us EQ NULL )
                        NL_QUIT;
                    vs = N_AllocReal1dArray( nn, SG );

                    if( vs EQ NULL )
                        NL_QUIT;

                    for ( i = 0; i <= nn; i++ )
                    {
                        N_CPtToPtEuclid( Cw[i], &C );
                        N_PtToXYZ( C, &us[i], &vs[i], &z );
                    }

                    *u = us;
                    *v = vs;
                    *n = nn;
                    NL_OUT;
                }
            }
        }
    }

    /* Check trimming tolerance */

    *n = -1;

    /* No longer using globals: */
    /* A = NL_TES_AREA; */
    /* lu = NL_TES_LENU;   */
    /* lv = NL_TES_LENV;   */

    error = N_CrvGetBBox( trm, &box );

    if( error EQ NL_YES )
        NL_OUT;
    N_BBoxGetDiagonal( &box, &len );

    N_SrfGetParameterBounds( sur, &xl, &xr, &yb, &yt );

    fac = NL_MAX( (xr - xl), (yb - yt) ) / NL_MAX( lu, lv );
    epa = prc * len;

    if( epl * fac GT epa )
        epl = epa / fac;

    if( epl LT NL_MTOL )
        epl = NL_MTOL;

    eplh = epl / 2.0;

    /* Decompose 2d trimming curve into continuous segments */

    error = N_CrvDecomposeContinuity( trm, &cur, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Estimate the number of vertices */

    is = p;
    ie = p + 1;
    t = T[0];
    len = 0.0;

    error = N_SrfEvalPtCrvOnSrf( sur, trm, t, &Ps );

    if( error EQ NL_YES )
        NL_OUT;

    while( ie LT m )
    {
        while( ie LT m AND T[ie]EQ T[ie + 1] )
            ie++;

        dt = (T[ie] - T[is]) / p;

        for ( i = 1; i <= p; i++ )
        {
            t += dt;

            if( t GT T[m] )
                t = T[m];

            error = N_SrfEvalPtCrvOnSrf( sur, trm, t, &Pe );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( Ps, Pe, &dist );
            len += dist;
            N_CopyPt( Pe, &Ps );
        }

        is = ie;
        ie++;
    }

    nh = NL_MAX( 1, (NL_INDEX)(len / sqrt( epl )) );
    dtg = (T[m] - T[0]) / nh;

    if( nh LT 10 )
        nh *= 3;
    else
        nh *= 2;
    nhg = 2 * nh;

    dt = dtg;

    us = N_AllocReal1dArray( nh, SG );

    if( us EQ NULL )
        NL_QUIT;

    vs = N_AllocReal1dArray( nh, SG );

    if( vs EQ NULL )
        NL_QUIT;

    /* For each curve segment compute polygonal approximation */

    next_seg = NL_FALSE;

    ts = T[0];
    l = 0;

    error = N_CrvEval( trm, ts, NL_LEFT, &C );

    if( error EQ NL_YES )
        NL_OUT;

    N_PtToXYZ( C, &us[0], &vs[0], &z );
    N_ClampSrfAtParams( sur, &us[0], &vs[0] );

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetKnots( cur[i], &r, &U );
        mindt = dtg;

        if( i GT 0 )
            if( dist / eplh LT 0.001 AND dt / dtg LT 0.001 )
                dt = dtg;

        te = ts + dtg;

        if( te GT U[r] )
            te = U[r];

        while( te LE U[r]AND ts LT te )
        {
            /* Step ahead if guess point is too close */

            do
            {
                error = ST_TessCheckTrim( sur, cur[i], ts, te, &dist );

                if( error EQ NL_YES )
                {
                    if( te LT U[r] )
                        dist = 0.0;

                    else if( ts GT U[0] )
                        NL_OUT;

                    else
                    {
                        error = N_SrfEvalPtCrvOnSrf( sur, cur[i], ts, &Ps );
                        error = N_SrfEvalPtCrvOnSrf( sur, cur[i], te, &Pe );
                        N_DistPtPt( Ps, Pe, &dist );

                        if( dist GT epl )
                            NL_ERROR( NL_NUM_ERR );
                        error = NL_NO;

                        if( k EQ 0 )
                        {
                            *u = us;
                            *v = vs;
                            *n = l;
                            NL_OUT;
                        }

                        if( i EQ k )
                            dist = 0.5 *(epl + eplh);
                        else
                        {
                            next_seg = NL_TRUE;
                            break;
                        }
                    }
                }

                if( dist GE eplh OR te EQ U[r] )
                    break;

                te += dt;

                if( te GT U[r] )
                    te = U[r];
            } while ( 1 );

            /* Find point so that epl/2 <= |Ps, Pe| < epl */

            dt = te - ts;

            if( dt LT mindt )
                mindt = dt;

            count = 0;

            while( NOT next_seg )
            {
                count += 1;

                if( dist LT epl OR count GT 5000 )
                {
                    if( dist GE eplh OR te EQ U[r]OR count GT 5000 )
                    {
                        l++;

                        if( l GT nh )
                        {
                            error = N_Realloc1dRealArray( &us, nh, nh + nhg, SG );

                            if( error EQ NL_YES )
                                NL_OUT;

                            error = N_Realloc1dRealArray( &vs, nh, nh + nhg, SG );

                            if( error EQ NL_YES )
                                NL_OUT;

                            nh += nhg;
                        }

                        if( U[r] - te LE 0.01 *mindt )
                            te = U[r];

                        error = N_CrvEval( trm, te, NL_LEFT, &C );

                        if( error EQ NL_YES )
                            NL_OUT;

                        N_PtToXYZ( C, &us[l], &vs[l], &z );
                        N_ClampSrfAtParams( sur, &us[l], &vs[l] );

                        if( te LT U[r] )
                        {
                            if( fabs( us[l] - us[l - 1] )LT NL_PTOL AND fabs( vs[l] - vs[l - 1] )LT NL_PTOL )
                                l -= 1;
                        }

                        if( ts EQ U[0]AND te EQ U[r] )
                        {
                            if( fabs( us[l] - us[l - 1] )LT NL_PTOL AND fabs( vs[l] - vs[l - 1] )LT NL_PTOL )
                            {
                                if( k EQ 0 )
                                {
                                    *u = us;
                                    *v = vs;
                                    *n = l - 1;
                                    NL_OUT;
                                }
                                l -= 1;

                                if( i EQ k )
                                {
                                    us[l] = us[l + 1];
                                    vs[l] = vs[l + 1];
                                }
                                break;
                            }
                        }

                        dt = te - ts;

                        if( dt LT mindt )
                            mindt = dt;
                        break;
                    }
                    else
                    {
                        dt /= 2.0;

                        if( dt LT mindt )
                            mindt = dt;
                        te += dt;

                        if( te GT U[r] )
                            te = U[r];
                    }
                }
                else
                {
                    dt /= 2.0;

                    if( dt LT mindt )
                        mindt = dt;
                    te -= dt;

                    if( te LT U[0] )
                        te = U[0];
                }

                error = ST_TessCheckTrim( sur, cur[i], ts, te, &dist );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            ts = te;
            te += dt;

            if( te GT U[r] )
                te = U[r];
            next_seg = NL_FALSE;
        }
    }

    /* Compact output arrays */

    if( l LT nh )
    {
        error = N_Realloc1dRealArray( &us, nh, l, SG );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_Realloc1dRealArray( &vs, nh, l, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    *u = us;
    *v = vs;
    *n = l;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
/**********************************************************************/
/* ST_TessSubdivideSrf: Subdivide surface into a set of rectangles               */
/**********************************************************************/
/*******************************************************************//**
  
   DESCRIPTION:
 
     This routine is for internal use only: see N_TessTrimmedSrf.
     This  tesselation  routine  subdivides  a  surface  into a  set  of 
     rectangles so that the surface does not deviate from any  rectangle
     more than a given tolerance. The  corners of the rectangles as well
     as topology  information regarding  the connectivity  of points are
     output. A typical calling example is:
 
       NL_FLAG       **SM, *R, *T;
       NL_SURFACE    sur;
       NL_PARAMETER  ***x, ***y, *u, *v;
       NL_REAL       **UL, **UR, **VB, **VT, eps, A, lu, lv, tol;
       NL_INDEX      **TM, **hp, *ht, m, n, r, s;
       NL_STACKS     ST, SG;
       ...
              (define sur, get eps and trimming data);
       ...
       ST_TessSubdivideSrf(&sur, eps, x, y, ht, hp, m, UL, UR, VB, VT, A, lu, lv, tol, &u, &v, &R, &T, &n, &TM,
                &SM, &r, &s, &ST, &SG);
 
     MEMORY TO STORE  THE NL_PARAMETERS  u[0],..., v[n], THE FLAGS R[0],..., 
     T[n] AND THE MATRICES TM[r][s] AND SM[r][s] IS ALLOCATED INSIDE THE 
     ROUTINE.
 
 
   ACCESS:
   
     sur   , in/out ,  NURBS surface(knot vectors are rescaled)
     eps   , input  ,  Maximum  deviation  between the  surface and  any
                       rectangular approximation(model space tolerance)
     x, y   , input  ,  Trimming polygons(ALL MUST BE CLOSED LOOPS!)
     ht    , input  ,  ht[i] is  the  highest  index of  polygons in the 
                       i - th loop
     hp    , input  ,  hp[i][j] is the highest index of  vertices in the
                       j - th polygon residing inside the i - th loop
     m     , input  ,  Highest index in ht, i.e. there are m + 1  trimming 
                       loops
     UL, UR , input  ,  Left and right bounds of trimming curves
     VB, VT , input  ,  Bottom and top bounds of trimming curves
     A     , input  ,  Approximate area of the surface
     lu, lv, input  ,  Average lengths of surface in u and v
     tol   , input  ,  Parameter space tolerance
     u, v   , output ,  Parameters corresponding  to the  vertices of the
                       rectangles
     R, T   , output ,  Right and top flags:
                         NL_YES: there is a right/top neighbor
                         NL_NO : there is no right/top neighbor
     n     , output ,  Highest index in(u, v), R and T
     TM    , output ,  Topology matrix:
                         TM[i][j] >= 0 :(u[i], v[j]) is in the set
                         TM[i][j] <  0 : no point is at this location
     SM    , output ,  Status matrix:
     SM[i][j] = { NL_ONANDOVER, NL_OVER, NL_INP, NL_ONP, NL_OUTP} :
                                      NL_PRIVATE  NL_STRING  rname = "ST_TessSubdivideSrf");
                                    rectangle anchored at(u[i], v[j]) is
                                    classified as listed
                         SM[i][j] = NL_NOSTATUS: no point  at this location
     r, s   , output ,  Highest indexes in TM and SM
     ST    , input  ,  Memory stack for R, T, TM and SM
     SG    , input  ,  Memory stack for u and v
                       
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

static NL_FLAG ST_TessSubdivideSrf( NL_SURFACE *sur,
    NL_REAL eps,
    NL_PARAMETER *** x, NL_PARAMETER *** y,
    NL_INDEX *ht, NL_INDEX ** hp, NL_INDEX m,
    NL_REAL ** UL, NL_REAL ** UR, NL_REAL ** VB, NL_REAL ** VT,
    NL_REAL A, NL_REAL lu, NL_REAL lv, 
    NL_REAL tol,
    NL_PARAMETER ** u, NL_PARAMETER ** v,
    NL_FLAG ** R, NL_FLAG ** T,
    NL_INDEX *n,
    NL_INDEX *** TM, NL_FLAG *** SM,
    NL_INDEX *r, NL_INDEX *s,
    NL_STACKS *ST, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("ST_TessSubdivideSrf");

    NL_FLAG ** SMP, *RP, *TP, *RO, *TO, *S_RS, *rs, flt, dir, status, bstat, error = NL_NO;

    NL_INDEX ** TMP, *S_LB, *S_RB, *S_LT, *S_RT, i, j, k, l, nh, nuv, ns, top, a, b, c, d, np, mu, mv, lb, rb, lt, rt, ii, jj, minmax = 0;

    NL_DEGREE p, q;

    NL_REAL ** HU, ** HV, *up, *vp, *us, *vs, *uo, *vo, fac, d1, d2, tolsq, dist, ul, ur, vb, vt, um, vm, dm = 0.0, ctol, closest;

    NL_RECTANGLE rct;

    NL_SURFACE surA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Rescale knot rectangle */

    if( eps LT NL_MTOL )
        NL_ERROR( NL_TOL_ERR );

    /* No longer using globals: */
    /* A  = NL_TES_AREA; */
    /* lu = NL_TES_LENU; */
    /* lv = NL_TES_LENV; */

    N_CreateRectangle( &rct, 0.0, lu, 0.0, lv );
    N_SrfReparamToInterval( sur, rct, NL_UVDIR );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &a, &b, &c, &d );
    N_SrfGetDegrees( sur, &p, &q );

    /* Estimate various indexes */

    fac = A / eps;

    if( fac GT 500000.0 )
        fac = 500000.0;
    nuv = NL_MAX( 6, (NL_INDEX)(0.25 *fac) );

    if( nuv LT 20000 )
        nuv *= 2;
    nh = NL_MAX( 20, (NL_INDEX)(4.0 *sqrt( fac )) );
    ns = NL_MAX( 4, (NL_INDEX)(4.0 *log( fac )) );
    ns *= 2;

    ctol = 20.0 *tol;

    /**********************************************/
    /* Decompose surface into a set of rectangles */
    /**********************************************/

    /*(1) Allocate memory */

    up = N_AllocReal1dArray( nuv, &SL );

    if( up EQ NULL )
        NL_QUIT;

    vp = N_AllocReal1dArray( nuv, &SL );

    if( vp EQ NULL )
        NL_QUIT;

    RP = N_AllocFlag1dArray( nuv, &SL );

    if( RP EQ NULL )
        NL_QUIT;

    TP = N_AllocFlag1dArray( nuv, &SL );

    if( TP EQ NULL )
        NL_QUIT;

    rs = N_AllocFlag1dArray( nuv, &SL );

    if( rs EQ NULL )
        NL_QUIT;

    S_LB = N_AllocInt1dArray( ns, &SL );

    if( S_LB EQ NULL )
        NL_QUIT;

    S_RB = N_AllocInt1dArray( ns, &SL );

    if( S_RB EQ NULL )
        NL_QUIT;

    S_LT = N_AllocInt1dArray( ns, &SL );

    if( S_LT EQ NULL )
        NL_QUIT;

    S_RT = N_AllocInt1dArray( ns, &SL );

    if( S_RT EQ NULL )
        NL_QUIT;

    S_RS = N_AllocFlag1dArray( ns, &SL );

    if( S_RS EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= nuv; i++ )
        rs[i] = NL_NOSTATUS;

    error = N_AllocSrfArrays( &surA, a, b, p, q, c, d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*(2) Initialize stacks and output point set */

    status = ST_TessIsInsideTrim( x, y, ht, hp, m, UL, UR, VB, VT, 0.0, lu, 0.0, lv, tol );

    S_LB[0] = 0;
    S_RB[0] = 1;
    S_LT[0] = 2;
    S_RT[0] = 3;

    S_RS[0] = status;
    rs[0] = status;

    up[0] = 0.0;
    vp[0] = 0.0;
    up[1] = lu;
    vp[1] = 0.0;
    up[2] = 0.0;
    vp[2] = lv;
    up[3] = lu;
    vp[3] = lv;

    RP[0] = NL_YES;
    TP[0] = NL_YES;
    RP[1] = NL_NO;
    TP[1] = NL_YES;
    RP[2] = NL_YES;
    TP[2] = NL_NO;
    RP[3] = NL_NO;
    TP[3] = NL_NO;

    top = 0;
    np = 3;

    /*(3) Subdivide while stack is not empty */

    while( top GE 0 )
    {
        /*(3.1) Pop rectangle stacks */

        lb = S_LB[top];
        rb = S_RB[top];
        lt = S_LT[top];
        rt = S_RT[top];
        ul = up[lb];
        ur = up[rt];
        vb = vp[lb];
        vt = vp[rt];

        if( ur - ul LT NL_PTOL OR vt - vb LT NL_PTOL )
            NL_ERROR( NL_PAR_ERR );

        bstat = S_RS[top];
        top--;

        /*(3.2) Extract surface */

        if( bstat EQ NL_OUTP )
            continue;

        N_SrfSetSizeIndices( &surA, a, b, p, q, c, d );
        error = N_SrfExtractPatch( sur, ul, ur, vb, vt, &surA, SG, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*(3.3) Check flatness */

        error = N_SrfIsFlat( &surA, eps, &flt, &dir );

        if( flt EQ NL_YES )
            continue;

        /*(3.4) Surface must be subdivided */

        /*(3.4.1) Collect subdivision points */

        if( np + 2 GT nuv )
        {
            error = N_Realloc1dRealArray( &up, nuv, nuv + nuv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dRealArray( &vp, nuv, nuv + nuv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dFlagArray( &RP, nuv, nuv + nuv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dFlagArray( &TP, nuv, nuv + nuv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dFlagArray( &rs, nuv, nuv + nuv, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            for ( i = nuv + 1; i <= nuv + nuv; i++ )
                rs[i] = NL_NOSTATUS;

            nuv += nuv;
        }

        um = 0.5 *(ul + ur);
        vm = 0.5 *(vb + vt);

        if( dir EQ NL_UDIR )
        {
            if( vt - vm GT ctol )
            {
                closest = lv;                          /* don't subdivide too  */

                for ( ii = 0; ii <= m; ii++ )          /* close to boundary of */
                    for ( jj = 0; jj <= ht[ii]; jj++ ) /* trim curve boxes     */
                    {
                        if( fabs( vm - VB[ii][jj] )LT closest )
                        {
                            dm = VB[ii][jj];
                            closest = fabs( vm - dm );
                            minmax = 1;
                        }

                        if( fabs( vm - VT[ii][jj] )LT closest )
                        {
                            dm = VT[ii][jj];
                            closest = fabs( vm - dm );
                            minmax = 2;
                        }
                    }

                if( closest LT ctol )
                {
                    if( minmax EQ 1 )
                    {
                        if( dm - ctol GT vb )
                            vm = dm - ctol;
                    }
                    else
                    {
                        if( dm + ctol LT vt )
                            vm = dm + ctol;
                    }
                }
            }

            up[np + 1] = ul;
            vp[np + 1] = vm;
            up[np + 2] = ur;
            vp[np + 2] = vm;

            RP[np + 1] = NL_YES;
            TP[np + 1] = NL_YES;
            RP[np + 2] = NL_NO;
            TP[np + 2] = NL_YES;
        }
        else
        {
            if( ur - um GT ctol )
            {
                closest = lu;                          /* don't subdivide too  */

                for ( ii = 0; ii <= m; ii++ )          /* close to boundary of */
                    for ( jj = 0; jj <= ht[ii]; jj++ ) /* trim curve boxes     */
                    {
                        if( fabs( um - UL[ii][jj] )LT closest )
                        {
                            dm = UL[ii][jj];
                            closest = fabs( um - dm );
                            minmax = 1;
                        }

                        if( fabs( um - UR[ii][jj] )LT closest )
                        {
                            dm = UR[ii][jj];
                            closest = fabs( um - dm );
                            minmax = 2;
                        }
                    }

                if( closest LT ctol )
                {
                    if( minmax EQ 1 )
                    {
                        if( dm - ctol GT ul )
                            um = dm - ctol;
                    }
                    else
                    {
                        if( dm + ctol LT ur )
                            um = dm + ctol;
                    }
                }
            }

            up[np + 1] = um;
            vp[np + 1] = vb;
            up[np + 2] = um;
            vp[np + 2] = vt;

            RP[np + 1] = NL_YES;
            TP[np + 1] = NL_YES;
            RP[np + 2] = NL_YES;
            TP[np + 2] = NL_NO;
        }

        /*(3.4.2) Put new parameter squares on the stack */

        if( top + 2 GT ns )
        {
            error = N_Realloc1dIntArray( &S_LB, ns, ns + ns, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dIntArray( &S_RB, ns, ns + ns, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dIntArray( &S_LT, ns, ns + ns, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dIntArray( &S_RT, ns, ns + ns, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_Realloc1dFlagArray( &S_RS, ns, ns + ns, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            ns += ns;
        }

        if( dir EQ NL_UDIR )
        {
            S_LB[top + 1] = lb;
            S_RB[top + 1] = rb;
            S_LT[top + 1] = np + 1;
            S_RT[top + 1] = np + 2;
            S_LB[top + 2] = np + 1;
            S_RB[top + 2] = np + 2;
            S_LT[top + 2] = lt;
            S_RT[top + 2] = rt;
        }
        else
        {
            S_LB[top + 1] = lb;
            S_RB[top + 1] = np + 1;
            S_LT[top + 1] = lt;
            S_RT[top + 1] = np + 2;
            S_LB[top + 2] = np + 1;
            S_RB[top + 2] = rb;
            S_LT[top + 2] = np + 2;
            S_RT[top + 2] = rt;
        }

        /*(3.4.3) Classify new rectangles */

        if( bstat EQ NL_INP )
        {
            S_RS[top + 1] = NL_INP;
            S_RS[top + 2] = NL_INP;
            rs[lb] = NL_INP;
            rs[np + 1] = NL_INP;
        }
        else
        {
            if( dir EQ NL_UDIR )
            {
                status = ST_TessIsInsideTrim( x, y, ht, hp, m, UL, UR, VB, VT, ul, ur, vb, vm, tol );

                S_RS[top + 1] = status;
                rs[lb] = status;

                status = ST_TessIsInsideTrim( x, y, ht, hp, m, UL, UR, VB, VT, ul, ur, vm, vt, tol );

                S_RS[top + 2] = status;
                rs[np + 1] = status;
            }
            else
            {
                status = ST_TessIsInsideTrim( x, y, ht, hp, m, UL, UR, VB, VT, ul, um, vb, vt, tol );

                S_RS[top + 1] = status;
                rs[lb] = status;

                status = ST_TessIsInsideTrim( x, y, ht, hp, m, UL, UR, VB, VT, um, ur, vb, vt, tol );

                S_RS[top + 2] = status;
                rs[np + 1] = status;
            }
        }

        np += 2;
        top += 2;
    }

    /* Hash vertices of subdivision rectangles into hash tables */

    error = N_InitRealHash( &HU, nh, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_InitRealHash( &HV, nh, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    mu = mv = -1;

    for ( k = 0; k <= np; k++ )
    {
        error = N_UpdateRealHash( HU, nh, 0.0, up[k], lu, &mu, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_UpdateRealHash( HV, nh, 0.0, vp[k], lv, &mv, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Retrieve subdivision parameters */

    us = N_AllocReal1dArray( mu, &SL );

    if( us EQ NULL )
        NL_QUIT;

    vs = N_AllocReal1dArray( mv, &SL );

    if( vs EQ NULL )
        NL_QUIT;

    TMP = N_AllocInt2dArray( mu, mv, ST );

    if( TMP EQ NULL )
        NL_QUIT;

    SMP = N_AllocFlag2dArray( mu, mv, ST );

    if( SMP EQ NULL )
        NL_QUIT;

    N_RetrieveRealHash( HU, nh, us );
    N_RetrieveRealHash( HV, nh, vs );

    /* Build topology matrix */

    uo = N_AllocReal1dArray( np, SG );

    if( uo EQ NULL )
        NL_QUIT;

    vo = N_AllocReal1dArray( np, SG );

    if( vo EQ NULL )
        NL_QUIT;

    RO = N_AllocFlag1dArray( np, ST );

    if( RO EQ NULL )
        NL_QUIT;

    TO = N_AllocFlag1dArray( np, ST );

    if( TO EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= mu; i++ )
    {
        for ( j = 0; j <= mv; j++ )
        {
            TMP[i][j] = -1;
            SMP[i][j] = NL_NOSTATUS;
        }
    }

    a = -1;

    for ( k = 0; k <= np; k++ )
    {
        N_SearchInsertReal( us, mu, up[k], &i );
        N_SearchInsertReal( vs, mv, vp[k], &j );

        if( i LT 0 OR j LT 0 )
            NL_ERROR( NL_NUM_ERR );

        l = TMP[i][j];

        if( l GE 0 )
        {
            if( RO[l]EQ NL_NO AND RP[k]EQ NL_YES )
                RO[l] = NL_YES;

            if( TO[l]EQ NL_NO AND TP[k]EQ NL_YES )
                TO[l] = NL_YES;
        }
        else
        {
            a++;
            TMP[i][j] = a;
            uo[a] = up[k];
            vo[a] = vp[k];
            RO[a] = RP[k];
            TO[a] = TP[k];
        }

        if( rs[k]NEQ NL_NOSTATUS )
            SMP[i][j] = rs[k];
    }

    /* Compact output */

    error = N_Realloc1dRealArray( &uo, np, a, SG );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_Realloc1dRealArray( &vo, np, a, SG );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_Realloc1dFlagArray( &RO, np, a, ST );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_Realloc1dFlagArray( &TO, np, a, ST );

    if( error EQ NL_YES )
        NL_OUT;

    /* Handle 1 rectangle as special case(NL_OVER) */

    if( a EQ 3 )
    {
        SMP[0][0] = NL_OVER;
        tolsq = tol * tol;

        /* Ensure that all polygon vertices are within bounds */

        for ( i = 0; i <= m; i++ )
            for ( j = 0; j <= ht[i]; j++ )
                for ( k = 0; k <= hp[i][j]; k++ )
                {
                    if( x[i][j][k]LT 0.0 )
                        x[i][j][k] = 0.0;

                    else if( x[i][j][k]GT lu )
                        x[i][j][k] = lu;

                    if( y[i][j][k]LT 0.0 )
                        y[i][j][k] = 0.0;

                    else if( y[i][j][k]GT lv )
                        y[i][j][k] = lv;

                    /* check for and eliminate duplicates */

                    if( k GT 0 AND hp[i][j]GT 3 )
                    {
                        d1 = x[i][j][k] - x[i][j][k - 1];
                        d2 = y[i][j][k] - y[i][j][k - 1];
                        dist = d1 * d1 + d2 * d2;

                        if( dist LE tolsq )
                        {
                            for ( l = k; l < hp[i][j]; l++ )
                            {
                                x[i][j][l] = x[i][j][l + 1];
                                y[i][j][l] = y[i][j][l + 1];
                            }
                            hp[i][j] -= 1;
                            k -= 1;
                        }
                    }
                }
    }

    /* Create the output */

    *u = uo;
    *v = vo;
    *R = RO;
    *T = TO;
    *n = a;
    *TM = TMP;
    *SM = SMP;
    *r = mu;
    *s = mv;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}

#endif  // NLIB_UNUSED
