// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* NL_Iges.c : NLib Math Function Definitions                                  */
/*****************************************************************************/

#include "StdAfx.h"



#include <nurbs.h>
#include <NL_Globals.h>

/* file local declarations */
#define IND(I, J, K) (16*(I) + 4*(J) + (K))

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an Nlib NURBS curve from the NL_IGES  Entity
     126 NURBS curve data. The resulting Nlib NURBS curve will be clamp-
     ed and have no internal knots of multiplicity greater than  degree.
     A typical calling example is as follows:

       NL_INDEX      k;
       NL_DEGREE     m;
       NL_PARAMETER  v[2];
       NL_REAL       *t, *w;
       NL_POINT      *P;
       NL_CURVE      cur;
       NL_STACKS     S;
       ...
       (allocate and load arrays)
       ...

       N_CrvInitArrays(&cur);
       N_Iges126Crv(k,m,t,w,P,v,&cur,&S);

     2d curves are identified by setting the z coordinate values of all control points
     to the magic value, NL_NOZ.  Output curve control point z coordinates that are
     generated from input control point z coordinates == NL_NOZ are set to NL_NOZ.
     
     When building a 3d curve on the z=0.0 plane, just set all the z coordinate
     values to 0.0.  That curve can be converted to a 2d curve with a call to
     N_Crv3dTo2d which sets all control point z coordinate values to NL_NOZ.

     If memory is  available, cur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  cur's  
     knot vector and polygon objects. NOTICE that the four PROPS  flags,
     the Form Number, and the normal vector are not required.


   ACCESS:

     k   , input  ,  Upper index of sum (k+1 control points)
     m   , input  ,  Degree of the basis functions
     t   , input  ,  The k+m+2 knots
     w   , input  ,  The k+1 weights
     P   , input  ,  The k+1 Euclidean control points (unweighted)
     v   , input  ,  The bounding parameters.
     cur , output ,  Nlib NURBS curve
     S   , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_Iges126Crv( NL_INDEX k, NL_DEGREE m, NL_REAL *t, NL_REAL *w, NL_POINT *P, NL_PARAMETER v[2], NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges126Crv");

    NL_FLAG error = NL_NO, rat = NL_NO; /* , xyp = NL_YES; */

    NL_INDEX ii, nn, mm;

    NL_DEGREE pp;

    NL_REAL *U, xx, yy, zz, ww;

    NL_CPOINT *Pw;

    /* Get Nlib notation */

    pp = m;
    nn = k;
    mm = nn + pp + 1;

    /* Check for various errors: degree, weights, knots */

    if( pp LT 1 OR pp GT NL_DMAX )
    {
        N_ErrSet( NL_DEG_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    for ( ii = 0; ii <= nn; ii++ ) /* also set rational flag */
    {
        if( w[ii]LT NL_WMIN OR w[ii]GT NL_WMAX )
        {
            N_ErrSet( NL_WEI_ERR, rname );
            error = NL_YES;
            NL_OUT;
        }

        if( w[ii]NEQ w[0] )
            rat = NL_YES;
    }

    error = N_IgesValidate( t, mm, pp, v );

    if( error EQ NL_YES )
        NL_OUT;

    /* Make a NURBS curve in the Nlib structures (may not */
    /* be a valid Nlib NURBS curve yet. */

    error = N_CrvSizeArrays( cur, nn, pp, mm, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsDegreeAndKnots( cur, &nn, &Pw, &pp, &mm, &U );

    for ( ii = 0; ii <= mm; ii++ ) /* load the knots */
        U[ii] = t[ii];

    for ( ii = 0; ii <= nn; ii++ ) /* Weight and load the control points */
    {                              /* Also check for xy-planar curve     */
        N_PtToXYZ( P[ii], &xx, &yy, &zz );

        if( rat == NL_YES )
        {
            ww = w[ii];
            xx = ww * xx;
            yy = ww * yy;
            zz = (zz == NL_NOZ) ? zz : ww * zz;
        }
        else
            ww = NL_NOW;
        /* if ( zz NEQ 0.0 )  xyp = NL_NO; */
        N_CPtFromWxWyWz( xx, yy, zz, ww, &Pw[ii] );
    }

    /* No: a 3d curve in the xy plane is not necessarily a 2d curve.
    * If the caller wants a 2d curve, that can be done separately: N_Crv3dTo2d().
    * if ( xyp EQ NL_YES )
    *  {
    *   for ( ii=0; ii <= nn; ii++ )  
    *     N_CPtSetZ(NL_NOZ,&Pw[ii]);
    *  }
    */

    /* Now clamp it to the NL_IGES bounds */

    error = N_IgesClampCrv( cur, v, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Sanitize the interior knots */

    error = N_IgesCorrectCrvKnots( cur, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* It is now a valid Nlib NURBS curve */

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_Iges126Crv */



/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine clamps an NL_IGES Entity 126 NURBS  curve  to  its
     valid parameter bounds. This function assumes that the curve  has
     been loaded into Nlib structure, and: 
       (1) m = n+p+1
       (2) U[i] <= U[i+1] for all i
       (3) U[p] <= v[0] < v[1] <= U[m-p]  (v[0],v[1] are valid bounds)
     Clamping is done in place. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  v[2];
       NL_STACKS     S;
       ...
       (define cur, and load bounds in v);
       ...
       N_IgesClampCrv(&cur,v,&S);


   ACCESS:
   
     cur  , in/out ,  NURBS curve
     v    , input  ,  Bounds for clamping
     S    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IgesClampCrv( NL_CURVE *cur, NL_PARAMETER v[2], NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, ll, lk, lr, n, m, span1 = 0, mult1, span2 = 0, mult2, is, ie;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, alf, oma, left;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE curA;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &UP );

    /* Find knot spans and set multiplicities of bounds (as knots) */

    mult1 = 0;

    for ( i = 0; i <= m; i++ )
    {
        if( UP[i]EQ v[0] )
            mult1 += 1;

        if( UP[i]GT v[0] )
        {
            span1 = i - 1;
            break;
        }
    }
    mult2 = 0;

    for ( i = m; i >= 0; i-- )
    {
        if( UP[i]EQ v[1] )
            mult2 += 1;

        if( UP[i]LT v[1] )
        {
            span2 = i + mult2;
            break;
        }
    }

    /* Handle case that junk is in first and last knots */

    if( span1 GE p AND mult1 EQ span1 )
    {
        UP[0] = v[0];
        mult1 += 1;
    }

    if( span2 EQ m - 1 AND mult2 GE p )
    {
        UP[m] = v[1];
        mult2 += 1;
        span2 += 1;
    }

    /* Check if it is already properly clamped (at ends) */

    if( span1 GE p AND mult1 EQ span1 + 1 AND span2 EQ m AND mult2 GE p + 1 )
        NL_OUT; /* nothing to do */

    /* Get new indices */

    is = span1 - p;
    ie = span2 - mult2;

    n = ie - is;
    m = span2 - span1 - mult2 + 2 * p + 1;

    if( n < 0 )
        { error = NL_YES; NL_OUT; }

    /* Get new memory for control points and knots */

    curA = *cur;
    error = N_AllocCrvArrays( cur, n, p, m, S );

    if( error EQ NL_YES )
        NL_OUT;
    N_CrvGetCPtsAndKnots( cur, &Qw, &UQ );

    /* Get initial control points */

    for ( i = is; i <= ie; i++ )
        N_CopyCPt( Pw[i], &Qw[i - is] );

    /* Insert the left knot */

    ll = span1 - p;

    for ( i = 1; i <= p - mult1; i++ )
    {
        for ( j = 0; j <= p - i - mult1; j++ )
        {
            left = UP[ll + i + j];
            alf = (v[0] - left) / (UP[span1 + j + 1] - left);
            oma = 1.0 - alf;
            N_Combine2CPts( alf, Qw[j + 1], oma, Qw[j], &Qw[j] );
        }
    }

    /* Insert the right knot */

    lr = span2 - p;
    lk = n - p + mult2;

    for ( i = 1; i <= p - mult2; i++ )
    {
        for ( j = p - i - mult2; j >= 0; j-- )
        {
            k = lk + i + j;
            left = UP[lr + i + j];

            if( left LT v[0] )
                left = v[0];

            alf = (v[1] - left) / (UP[span2 + j + 1] - left);
            oma = 1.0 - alf;
            N_Combine2CPts( alf, Qw[k], oma, Qw[k - 1], &Qw[k] );
        }
    }

    /* Load the knot vector */

    j = -1;

    for ( i = 0; i <= p; i++ )
        UQ[++j] = v[0];

    for ( i = span1 + 1; i <= span2 - mult2; i++ )
        UQ[++j] = UP[i];

    for ( i = 0; i <= p; i++ )
        UQ[++j] = v[1];

    /* Kill old curve */

    N_FreeCrv( &curA, S );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_IgesClampCrv */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine sanitizes, validates, and corrects knot multipli-
     cities of an NL_IGES 126 NURBS curve that has been put into Nlib  form
     and clamped. In particular, it ensures that:
       (1) end knot multiplicities are not greater than p+1
       (2) internal knot multiplicities are not greater than p  ( hence,
           the curve is at least C0 continuous )
     The sanitizing is done in place. A typical calling example is:

       NL_CURVE    cur;
       NL_STACKS   S;
       ...
       (define cur);
       ...
       N_IgesCorrectCrvKnots(&cur,&S);


   ACCESS:
   
     cur  , in/out ,  NURBS curve
     S    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IgesCorrectCrvKnots( NL_CURVE *cur, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, k1, k2, k3, n, m, nn, mm, mult;

    NL_DEGREE p;

    NL_PARAMETER us, ue;

    NL_REAL d, *UP;

    NL_CPOINT *Pw;

    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &UP );
    nn = n;
    mm = m;

    /* Eliminate internal multiplicities greater than p */

    us = UP[0];
    ue = UP[m];
    ii = p + 1;

    while( UP[ii]EQ UP[ii - 1] )
        ii += 1;

    while( UP[ii]LT ue )
    {
        /* get multiplicity of UP[ii] */

        mult = 1;

        while( UP[ii]EQ UP[ii + mult] )
            mult += 1;

        if( mult GT p )         /* reduce to multiplicity p */
        {
            k1 = ii - 1;        /* first control point to replace */
            k2 = k1 + mult - p; /* last  control point to replace */
            k3 = k2 - k1;

            for ( jj = k1 + 1; jj <= k2; jj++ )
                N_Sum2CPts( Pw[k1], Pw[jj], &Pw[k1] );
            d = 1.0 / ((NL_REAL)k3 + 1.0);
            N_ScaleCPt( d, Pw[k1], &Pw[k1] );
            k1 += 1;

            for ( jj = k2 + 1; jj <= nn; jj++ )
                N_CopyCPt( Pw[jj], &Pw[k1++] );
            nn -= k3;
            k1 = ii + p;

            for ( jj = ii + mult; jj <= mm; jj++ )
                UP[k1++] = UP[jj];
            mm -= k3;
            mult = p;
        }

        ii = ii + mult;
    }

    /* Validate that UP[0] multiplicity not greater than p+1 */

    mult = 0;

    while( us EQ UP[p + mult + 1] )
        mult += 1;

    if( mult GT 0 )
    {
        k1 = p + 1;

        for ( jj = p + mult + 1; jj <= mm; jj++ )
            UP[k1++] = UP[jj];
        mm -= mult;
        k1 = 1;

        for ( jj = mult + 1; jj <= nn; jj++ )
            N_CopyCPt( Pw[jj], &Pw[k1++] );
        nn -= mult;
    }

    /* Validate that UP[mm] multiplicity not greater than p+1 */

    mult = 0;

    while( ue EQ UP[mm - p - mult - 1] )
        mult += 1;

    if( mult GT 0 )
    {
        mm -= mult;
        N_CopyCPt( Pw[nn], &Pw[nn - mult] );
        nn -= mult;
    }

    /* Compact the control point and knot arrays if required */

    if( nn LT n )
    {
        N_CrvDetachPolygonKnot( cur, &pol, &p, &knt );
        N_KnotVectorFromRealArray( knt, UP, mm );
        N_CPolygonFromCPts( pol, Pw, nn );
        error = N_CrvCompress( cur, S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_IgesCorrectCrvKnots */



/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine validates the NL_IGES Entities  126  and  128  NURBS
     curve and surface knots and bounds data. In particular,  it  checks
     that:
       (1) the knots are non-decreasing
       (2) U[p] <= v[0] < v[1] <= U[m-p]
     A typical calling example is as follows:

       NL_INDEX      m;
       NL_DEGREE     p;
       NL_PARAMETER  v[2];
       NL_REAL       *U
       ...
       (load knots into U)
       ...

       N_IgesValidate(U,m,p,v);


   ACCESS:

     U   , input  ,  The knots
     m   , input  ,  High index of U
     p   , input  ,  Degree
     v   , input  ,  The bounding parameters


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_IgesValidate( NL_REAL *U, NL_INDEX m, NL_DEGREE p, NL_PARAMETER v[2] )
{
    NL_PRIVATE NL_STRING rname = _T("N_IgesValidate");

    NL_FLAG error = NL_NO;

    NL_INDEX ii;

    if( U[p]GT v[0]OR v[0]GE v[1]OR U[m - p]LT v[1] )
    {
        N_ErrSet( NL_KNT_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    for ( ii = 0; ii < m; ii++ )
        if( U[ii]GT U[ii + 1] )
        {
            N_ErrSet( NL_KNT_ERR, rname );
            error = NL_YES;
            NL_OUT;
        }

    /* Exit */

    EXIT:

    return (error);
} /* end N_IgesValidate */


/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an Nlib NURBS surface from the NL_IGES  Ent-
     ity 128 NURBS surface data.  The  resulting Nlib NURBS surface will 
     be clamped  and have no internal knots of multiplicity greater than
     degree. A typical calling example is as follows:

       NL_INDEX      k1, k2;
       NL_DEGREE     m1, m2;
       NL_PARAMETER  u[2], v[2];
       NL_REAL       *s, *t, **w;
       NL_POINT      **P;
       NL_SURFACE    sur;
       NL_STACKS     SG;
       ...
       (allocate and load arrays)
       ...

       N_SrfInitArrays(&sur);
       N_Iges128Srf(k1,k2,m1,m2,s,t,w,P,u,v,&sur,&SG);

     If memory is  available, sur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  sur's  
     knot vector and control net objects.  NOTICE  that  the  five PROPS 
     flags and the Form Number are not required


   ACCESS:

     k1  , input  ,  Upper index of first sum ( k1+1 control  points  in 
                     the s-direction)
     k2  , input  ,  Upper index of second sum ( k2+1 control points  in 
                     the t-direction)
     m1  , input  ,  Degree of the s-basis functions
     m2  , input  ,  Degree of the t-basis functions
     s   , input  ,  The k1+m1+2 s-knots
     t   , input  ,  The k2+m2+2 t-knots
     w   , input  ,  The (k1+1)x(k2+1) weights.  w[i][j] with 0<=i<=k1 ,
                     0<=j<=k2
     P   , input  ,  The  (k1+1)x(k2+1)  Euclidean control points  ( not
                     weighted). P[i][j] with 0<=i<=k1 , 0<=j<=k2
     u,v , input  ,  The bounding parameters
     sur , output ,  Nlib NURBS surface
     SG  , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_Iges128Srf( NL_INDEX k1, NL_INDEX k2, NL_DEGREE m1, NL_DEGREE m2, NL_REAL *s, NL_REAL *t, NL_REAL ** w, NL_POINT ** P, NL_PARAMETER u[2], NL_PARAMETER v[2], NL_SURFACE *sur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges128Srf");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX ii, jj, nn, mm, rr, ss;

    NL_DEGREE pp, qq;

    NL_REAL *U, *V, xx, yy, zz, ww;

    NL_CPOINT ** Pw;

    /* Get Nlib notation */

    pp = m1;
    qq = m2;
    nn = k1;
    mm = k2;
    rr = nn + pp + 1;
    ss = mm + qq + 1;

    /* Check for various errors: degree, weights, knots */

    if( pp LT 1 OR pp GT NL_DMAX OR qq LT 1 OR qq GT NL_DMAX )
    {
        N_ErrSet( NL_DEG_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    for ( ii = 0; ii <= nn; ii++ ) /* also set rational flag */
        for ( jj = 0; jj <= mm; jj++ )
        {
            if( w[ii][jj]LT NL_WMIN OR w[ii][jj]GT NL_WMAX )
            {
                N_ErrSet( NL_WEI_ERR, rname );
                error = NL_YES;
                NL_OUT;
            }

            if( w[ii][jj]NEQ w[0][0] )
                rat = NL_YES;
        }

    error = N_IgesValidate( s, rr, pp, u );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_IgesValidate( t, ss, qq, v );

    if( error EQ NL_YES )
        NL_OUT;

    /* Make a NURBS surface in the Nlib structures (may */
    /* not be a valid Nlib NURBS surface yet. */

    error = N_SrfSizeArrays( sur, nn, mm, pp, qq, rr, ss, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsDegreesAndKnots( sur, &nn, &mm, &Pw, &pp, &qq, &rr, &ss, &U, &V );

    /* load the knots */

    for ( ii = 0; ii <= rr; ii++ )
        U[ii] = s[ii];

    for ( ii = 0; ii <= ss; ii++ )
        V[ii] = t[ii];

    for ( ii = 0; ii <= nn; ii++ ) /* Weight and load the control points */
        for ( jj = 0; jj <= mm; jj++ )
        {
            N_PtToXYZ( P[ii][jj], &xx, &yy, &zz );

            if( rat == NL_YES )
            {
                ww = w[ii][jj];
                xx = ww * xx;
                yy = ww * yy;
                zz = ww * zz;
            }
            else
                ww = NL_NOW;
            N_CPtFromWxWyWz( xx, yy, zz, ww, &Pw[ii][jj] );
        }

    /* Now clamp it to the NL_IGES bounds */

    error = N_IgesClampSrf( sur, u, v, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Sanitize the interior knots */

    error = N_IgesCorrectSrfKnots( sur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* It is now a valid Nlib NURBS surface */

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_Iges128Srf */


/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine clamps an NL_IGES Entity 128 NURBS surface  to  its
     valid parameter bounds. This function assumes that the surface has
     been loaded into Nlib structure, and:
       (1) r = n+p+1  and  s = m+q+1
       (2) U[i] <= U[i+1]  and  V[j] <= V[j+1]  for 0<=i<=r , 0<=j<=s
       (3) U[p] <= ub[0] < ub[1] <= U[r-p]  (ub[0],ub[1] are bounds)
       (4) V[q] <= vb[0] < vb[1] <= V[s-q]  (vb[0],vb[1] are bounds)
     Clamping is done in place. A typical calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  ub[2], vb[2];
       NL_STACKS     S;
       ...
       (define sur, and load bounds in ub and vb);
       ...
       N_IgesClampSrf(&sur,ub,vb,&S);


   ACCESS:
   
     sur  , in/out ,  NURBS surface
     ub   , input  ,  u-bounds for clamping
     vb   , input  ,  v-bounds for clamping
     S    , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IgesClampSrf( NL_SURFACE *sur, NL_PARAMETER ub[2], NL_PARAMETER vb[2], NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ii, i, j, k, ll, lk, lr, span1 = 0, mult1, span2 = 0, mult2, is, ie, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *UP, *UQ, *VP, *VQ, alf, oma, left;

    NL_CPOINT ** Pw, ** Qw;

    NL_SURFACE surA;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );

    /* First clamp in u, then in v */

    /* Find knot spans and set multiplicities of bounds (as knots) */

    mult1 = 0;

    for ( i = 0; i <= r; i++ )
    {
        if( UP[i]EQ ub[0] )
            mult1 += 1;

        if( UP[i]GT ub[0] )
        {
            span1 = i - 1;
            break;
        }
    }
    mult2 = 0;

    for ( i = r; i >= 0; i-- )
    {
        if( UP[i]EQ ub[1] )
            mult2 += 1;

        if( UP[i]LT ub[1] )
        {
            span2 = i + mult2;
            break;
        }
    }

    /* Handle case that junk is in first and last knots */

    if( span1 GE p AND mult1 EQ span1 )
    {
        UP[0] = ub[0];
        mult1 += 1;
    }

    if( span2 EQ r - 1 AND mult2 GE p )
    {
        UP[r] = ub[1];
        mult2 += 1;
        span2 += 1;
    }

    /* Check if it is already properly clamped (at ends) */

    if( mult1 NEQ span1 + 1 OR span2 NEQ r OR mult2 LT p + 1 )
    { /* must clamp in u */

        /* Get new indices */

        is = span1 - p;
        ie = span2 - mult2;

        n = ie - is;
        r = span2 - span1 - mult2 + 2 * p + 1;

        if( n < 0 )
            { error = NL_YES; NL_OUT; }

        /* Get new memory for control points and knots */

        surA = *sur;
        error = N_AllocSrfArrays( sur, n, m, p, q, r, s, S );

        if( error EQ NL_YES )
            NL_OUT;
        N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

        /* Get initial control points */

        for ( ii = 0; ii <= m; ii++ )
            for ( i = is; i <= ie; i++ )
                N_CopyCPt( Pw[i][ii], &Qw[i - is][ii] );

        /* Insert the left knot */

        ll = span1 - p;

        for ( i = 1; i <= p - mult1; i++ )
        {
            for ( j = 0; j <= p - i - mult1; j++ )
            {
                left = UP[ll + i + j];
                alf = (ub[0] - left) / (UP[span1 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= m; ii++ )
                    N_Combine2CPts( alf, Qw[j + 1][ii], oma, Qw[j][ii], &Qw[j][ii] );
            }
        }

        /* Insert the right knot */

        lr = span2 - p;
        lk = n - p + mult2;

        for ( i = 1; i <= p - mult2; i++ )
        {
            for ( j = p - i - mult2; j >= 0; j-- )
            {
                k = lk + i + j;
                left = UP[lr + i + j];

                if( left LT ub[0] )
                    left = ub[0];

                alf = (ub[1] - left) / (UP[span2 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= m; ii++ )
                    N_Combine2CPts( alf, Qw[k][ii], oma, Qw[k - 1][ii], &Qw[k][ii] );
            }
        }

        /* Load the knot vectors */

        j = -1;

        for ( i = 0; i <= p; i++ )
            UQ[++j] = ub[0];

        for ( i = span1 + 1; i <= span2 - mult2; i++ )
            UQ[++j] = UP[i];

        for ( i = 0; i <= p; i++ )
            UQ[++j] = ub[1];

        for ( i = 0; i <= s; i++ )
            VQ[i] = VP[i];

        /* Kill old surface and get new values for local notation */

        N_FreeSrf( &surA, S );
        N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    } /* end of u-clamping */

    /* Now check if we need to clamp in v */

    /* Find knot spans and set multiplicities of bounds (as knots) */

    mult1 = 0;

    for ( i = 0; i <= s; i++ )
    {
        if( VP[i]EQ vb[0] )
            mult1 += 1;

        if( VP[i]GT vb[0] )
        {
            span1 = i - 1;
            break;
        }
    }
    mult2 = 0;

    for ( i = s; i >= 0; i-- )
    {
        if( VP[i]EQ vb[1] )
            mult2 += 1;

        if( VP[i]LT vb[1] )
        {
            span2 = i + mult2;
            break;
        }
    }

    /* Handle case that junk is in first and last knots */

    if( span1 GE q AND mult1 EQ span1 )
    {
        VP[0] = vb[0];
        mult1 += 1;
    }

    if( span2 EQ s - 1 AND mult2 GE q )
    {
        VP[s] = vb[1];
        mult2 += 1;
        span2 += 1;
    }

    /* Check if it is already properly clamped (at ends) */

    if( mult1 NEQ span1 + 1 OR span2 NEQ s OR mult2 LT q + 1 )
    { /* must clamp in v */

        /* Get new indices */

        is = span1 - q;
        ie = span2 - mult2;

        m = ie - is;
        s = span2 - span1 - mult2 + 2 * q + 1;

        if( m < 0 )
            { error = NL_YES; NL_OUT; }

        /* Get new memory for control points and knots */

        surA = *sur;
        error = N_AllocSrfArrays( sur, n, m, p, q, r, s, S );

        if( error EQ NL_YES )
            NL_OUT;
        N_SrfGetCPtsAndKnots( sur, &Qw, &UQ, &VQ );

        /* Get initial control points */

        for ( ii = 0; ii <= n; ii++ )
            for ( i = is; i <= ie; i++ )
                N_CopyCPt( Pw[ii][i], &Qw[ii][i - is] );

        /* Insert the left knot */

        ll = span1 - q;

        for ( i = 1; i <= q - mult1; i++ )
        {
            for ( j = 0; j <= q - i - mult1; j++ )
            {
                left = VP[ll + i + j];
                alf = (vb[0] - left) / (VP[span1 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= n; ii++ )
                    N_Combine2CPts( alf, Qw[ii][j + 1], oma, Qw[ii][j], &Qw[ii][j] );
            }
        }

        /* Insert the right knot */

        lr = span2 - q;
        lk = m - q + mult2;

        for ( i = 1; i <= q - mult2; i++ )
        {
            for ( j = q - i - mult2; j >= 0; j-- )
            {
                k = lk + i + j;
                left = VP[lr + i + j];

                if( left LT vb[0] )
                    left = vb[0];

                alf = (vb[1] - left) / (VP[span2 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= n; ii++ )
                    N_Combine2CPts( alf, Qw[ii][k], oma, Qw[ii][k - 1], &Qw[ii][k] );
            }
        }

        /* Load the knot vectors */

        j = -1;

        for ( i = 0; i <= q; i++ )
            VQ[++j] = vb[0];

        for ( i = span1 + 1; i <= span2 - mult2; i++ )
            VQ[++j] = VP[i];

        for ( i = 0; i <= q; i++ )
            VQ[++j] = vb[1];

        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        /* Kill old surface */

        N_FreeSrf( &surA, S );
    } /* end of v-clamping */

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_IgesClampSrf */



/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine sanitizes, validates, and corrects knot multipli-
     cities of an NL_IGES 128 NURBS surface that has  been  put  into  Nlib
     form and clamped. In particular, it ensures that:
       (1) end knot multiplicities are not greater than p+1 (q+1)
       (2) internal knot multiplicities are not greater than p or q (and
           hence, the surface is at least C0 continuous)
     The sanitizing is done in place. A typical calling example is:

       NL_SURFACE  sur;
       NL_STACKS   S;
       ...
       (define sur);
       ...
       N_IgesCorrectSrfKnots(&sur,&S);


   ACCESS:
   
     sur  , in/out ,  NURBS surface
     S    , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IgesCorrectSrfKnots( NL_SURFACE *sur, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, k1, k2, k3, k4, n, m, nn, mm, r, rr, s, ss, mult;

    NL_DEGREE p, q;

    NL_PARAMETER us, ue, vs, ve;

    NL_REAL d, *UP, *VP;

    NL_CPOINT ** Pw;

    NL_CNET *net;

    NL_KNOTVECTOR *knu, *knv;

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );
    nn = n;
    mm = m;
    rr = r;
    ss = s;

    /* First the u-direction, then the v-direction */

    /* Eliminate internal multiplicities greater than p */

    us = UP[0];
    ue = UP[r];
    ii = p + 1;

    while( UP[ii]EQ UP[ii - 1] )
        ii += 1;

    while( UP[ii]LT ue )
    {
        /* get multiplicity of UP[ii] */

        mult = 1;

        while( UP[ii]EQ UP[ii + mult] )
            mult += 1;

        if( mult GT p )         /* reduce to multiplicity p */
        {
            k1 = ii - 1;        /* first control point to replace */
            k2 = k1 + mult - p; /* last  control point to replace */
            k3 = k2 - k1;
            d = 1.0 / ((NL_REAL)k3 + 1.0);

            for ( kk = 0; kk <= mm; kk++ )
            {
                for ( jj = k1 + 1; jj <= k2; jj++ )
                    N_Sum2CPts( Pw[k1][kk], Pw[jj][kk], &Pw[k1][kk] );
                N_ScaleCPt( d, Pw[k1][kk], &Pw[k1][kk] );
                k4 = k1 + 1;

                for ( jj = k2 + 1; jj <= nn; jj++ )
                    N_CopyCPt( Pw[jj][kk], &Pw[k4++][kk] );
            }

            nn -= k3;
            k1 = ii + p;

            for ( jj = ii + mult; jj <= rr; jj++ )
                UP[k1++] = UP[jj];
            rr -= k3;
            mult = p;
        }

        ii = ii + mult;
    }

    /* Validate that UP[0] multiplicity not greater than p+1 */

    mult = 0;

    while( us EQ UP[p + mult + 1] )
        mult += 1;

    if( mult GT 0 )
    {
        k1 = p + 1;

        for ( jj = p + mult + 1; jj <= rr; jj++ )
            UP[k1++] = UP[jj];
        rr -= mult;

        for ( kk = 0; kk <= mm; kk++ )
        {
            k1 = 1;

            for ( jj = mult + 1; jj <= nn; jj++ )
                N_CopyCPt( Pw[jj][kk], &Pw[k1++][kk] );
        }
        nn -= mult;
    }

    /* Validate that UP[rr] multiplicity not greater than p+1 */

    mult = 0;

    while( ue EQ UP[rr - p - mult - 1] )
        mult += 1;

    if( mult GT 0 )
    {
        rr -= mult;

        for ( kk = 0; kk <= mm; kk++ )
            N_CopyCPt( Pw[nn][kk], &Pw[nn - mult][kk] );
        nn -= mult;
    }

    /* Now the v-direction */

    /* Eliminate internal multiplicities greater than q */

    vs = VP[0];
    ve = VP[s];
    ii = q + 1;

    while( VP[ii]EQ VP[ii - 1] )
        ii += 1;

    while( VP[ii]LT ve )
    {
        /* get multiplicity of VP[ii] */

        mult = 1;

        while( VP[ii]EQ VP[ii + mult] )
            mult += 1;

        if( mult GT q )         /* reduce to multiplicity q */
        {
            k1 = ii - 1;        /* first control point to replace */
            k2 = k1 + mult - q; /* last  control point to replace */
            k3 = k2 - k1;
            d = 1.0 / ((NL_REAL)k3 + 1.0);

            for ( kk = 0; kk <= nn; kk++ )
            {
                for ( jj = k1 + 1; jj <= k2; jj++ )
                    N_Sum2CPts( Pw[kk][k1], Pw[kk][jj], &Pw[kk][k1] );
                N_ScaleCPt( d, Pw[kk][k1], &Pw[kk][k1] );
                k4 = k1 + 1;

                for ( jj = k2 + 1; jj <= mm; jj++ )
                    N_CopyCPt( Pw[kk][jj], &Pw[kk][k4++] );
            }

            mm -= k3;
            k1 = ii + q;

            for ( jj = ii + mult; jj <= ss; jj++ )
                VP[k1++] = VP[jj];
            ss -= k3;
            mult = q;
        }

        ii = ii + mult;
    }

    /* Validate that VP[0] multiplicity not greater than q+1 */

    mult = 0;

    while( vs EQ VP[q + mult + 1] )
        mult += 1;

    if( mult GT 0 )
    {
        k1 = q + 1;

        for ( jj = q + mult + 1; jj <= ss; jj++ )
            VP[k1++] = VP[jj];
        ss -= mult;

        for ( kk = 0; kk <= nn; kk++ )
        {
            k1 = 1;

            for ( jj = mult + 1; jj <= mm; jj++ )
                N_CopyCPt( Pw[kk][jj], &Pw[kk][k1++] );
        }
        mm -= mult;
    }

    /* Validate that VP[ss] multiplicity not greater than q+1 */

    mult = 0;

    while( ve EQ VP[ss - q - mult - 1] )
        mult += 1;

    if( mult GT 0 )
    {
        ss -= mult;

        for ( kk = 0; kk <= nn; kk++ )
            N_CopyCPt( Pw[kk][mm], &Pw[kk][mm - mult] );
        mm -= mult;
    }

    /* Compact the control point and knot arrays if required */

    if( nn LT n OR mm LT m )
    {
        N_SrfGetNetAndKnotVectors( sur, &net, &p, &q, &knu, &knv );
        N_KnotVectorFromRealArray( knu, UP, rr );
        N_KnotVectorFromRealArray( knv, VP, ss );
        N_CNetFromCPts( net, Pw, nn, mm );
        error = N_SrfCompress( sur, S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_IgesCorrectSrfKnots */


/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates a nonrational Nlib  B-spline  curve  from
     the  NL_IGES Entity 112 parameteric spline curve data.  The  resulting
     B-spline curve has knots of multiplicity equal  to  degree  at  all 
     interior  breakpoints.  Knot removal may  be  applied to  eliminate
     some of  these  multiplicities  (based on  the  continuity  of  the 
     spline). A typical calling example is as follows:

       NL_INDEX      n;
       NL_PARAMETER  *t;
       NL_REAL       ***polys;
       NL_CURVE      cur;
       NL_STACKS     S;
       ...
       (load the polynomial coefficients into array polys)
       ...

       N_CrvInitArrays(&cur);
       N_Iges112CrvNonRat(n,t,polys,&cur,&S);

     If memory is  available, cur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  cur's  
     knot vector and polygon objects.


   ACCESS:

     n     , input  ,  There are n polynomial segments (n+1 breakpoints)
     t     , input  ,  The n+1 breakpoints
     polys , input  ,  The polynomial  coefficients.  polys[i][j][k]  is
                       the  coefficient of the k-th power  term  of  the 
                       j-th coordinate of the i-th polynomial; 0<=k<=3 ,
                       0<=j<=2 , 0<=i<=n-1
     cur   , output ,  B-spline curve
     S     , input  ,  cur's memory stack

     2d curves are identified by setting the z coordinate values of all control points
     to the magic value, NL_NOZ.  This function only outputs 3d curves.
     
     When building a 2d curve place it on the z=0.0 plane, just set all the z coordinate
     values to 0.0.  That curve can be converted to a 2d curve with a call to
     N_Crv3dTo2d which sets all control point z coordinate values to NL_NOZ.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges112CrvNonRat( NL_INDEX n, NL_PARAMETER *t, NL_REAL *** polys, NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges112CrvNonRat");

    NL_FLAG error = NL_NO; /* , xyp = NL_YES; */

    NL_INDEX i, j, k, kk, nn, mm;

    NL_DEGREE pp;

    NL_REAL *U, xx, yy, zz, ww, d, gam;

    NL_CPOINT *Pw, *Aw;

    NL_RMATRIX ipm;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* First check for proper breakpoints */

    for ( i = 0; i < n; i++ )
        if( t[i]GE t[i + 1] )
        {
            N_ErrSet( NL_CUR_ERR, rname );
            error = NL_YES;
            NL_OUT;
        }

    /* Now determine degree */

    for ( i = 3; i > 1; i-- )
    {
        for ( j = 0; j < n; j++ )
        {
            for ( k = 0; k < 3; k++ )
                if( polys[j][k][i]NEQ 0.0 )
                    break;

            if( k LT 3 )
                break;
        }

        if( j LT n )
            break;
    }

    pp = (NL_DEGREE)i;

    /*  Get nn,mm and check for memory in curve structure */

    nn = n * pp;
    mm = (n + 1) * pp + 1;

    error = N_CrvSizeArrays( cur, nn, pp, mm, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsDegreeAndKnots( cur, &nn, &Pw, &pp, &mm, &U );

    /* Get inverse of power basis conversion matrix */

    N_InitRealMatrix( &ipm );
    error = N_BezInversePowerMatrix( pp, &ipm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get control point memory for one polynomial segment */

    Aw = N_AllocCPt1dArray( pp, &SL );

    if( Aw EQ NULL )
        NL_QUIT;

    /* Now loop and convert each polynomial segment to Bezier, */
    /* and piece the Beziers together with pp-ful knots        */

    for ( i = 0; i <= pp; i++ )
        U[i] = t[0];
    ww = NL_NOW;
    k = 0;
    kk = pp + 1;

    for ( i = 0; i < n; i++ )
    {
        d = t[i + 1] - t[i]; /* factor to reparameterize segments to [0,1] */

        gam = 1.0;

        for ( j = 0; j <= pp; j++ )
        {
            xx = gam * polys[i][0][j];
            yy = gam * polys[i][1][j];
            zz = gam * polys[i][2][j];
            /* if (zz NEQ 0.0)  xyp = NL_NO; */
            N_CPtFromWxWyWz( xx, yy, zz, ww, &Aw[j] );
            gam = d * gam;
        }

        error = N_RealMatrixMultiplyCPtArray( &ipm, Aw, &Pw[k] );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j < pp; j++ )
            U[kk++] = t[i + 1];
        k += pp;
    }

    U[kk] = t[n];

    /* Set z-coordinates if it is xy-planar */

    /* No: a 3d curve in the xy plane is not necessarily a 2d curve.
    * If the caller wants a 2d curve, that can be done separately: N_Crv3dTo2d().
    * if ( xyp EQ NL_YES )
    * {
    *   for ( i=0; i<=nn; i++ )
    *     N_CPtSetZ(NL_NOZ,&Pw[i]);
    * }
    */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges112CrvNonRat */

#ifdef NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an Nlib Nurbs full circle or circular arc
     from the NL_IGES Entity 100 data. The circle is in a plane parallel to
     the xy-plane. The parameter domain of the Nurbs curve is that given
     in the NL_IGES Manual, [start angle , end angle].  A  typical  calling 
     example is as follows:

       NL_REAL       zt, x1, y1, x2, y2, x3, y3, tol;
       NL_CURVE      cur;
       NL_STACKS     S;
       ...
       (choose tol)

       N_CrvInitArrays(&cur);
       N_Iges100Arc(zt,x1,y1,x2,y2,x3,y3,NL_QUADRATIC,tol,&cur,&S);

     If memory is  available, cur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  cur's  
     knot vector and polygon objects.


   ACCESS:

     zt    , input  ,  The displacement of the circular arc from the xy-
                       plane
     x1,y1 , input  ,  Center of the circular arc
     x2,y2 , input  ,  Start point of the arc
     x3,y3 , input  ,  End point of the arc
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: degree 2 circle is created, possibly
                                    with double internal knots
                         NL_QUARTIC  : degree 4 circle is created, possibly
                                    with quadruple internal  knots,  but
                                    better  parameterization  than   the 
                                    quadratic
                         NL_QUINTIC  : degree 5 circle is created  with  no
                                    internal knots     
     tol   , input  ,  If the radius of the circle is less than tol, the
                       circle is not created (error)
     cur   , output ,  NURBS circle/circular arc
     S     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_Iges100Arc( NL_REAL zt, NL_REAL x1, NL_REAL y1, NL_REAL x2, NL_REAL y2, NL_REAL x3, NL_REAL y3, NL_FLAG ctp, NL_REAL tol, NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges100Arc");

    NL_FLAG error = NL_NO;

    NL_REAL d2, d3, rad, sang, eang, sr, er, w;

    NL_INDEX n;

    NL_CPOINT *Pw;

    NL_POINT cen;

    NL_VECTOR X, Y, v2, v3;

    NL_INTERVAL I;

    NL_DEGREE p;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Create the local coordinate system for the circle */

    N_PtFromXYZ( x1, y1, zt, &cen );

    N_VectorCreate( 1.0, 0.0, 0.0, &X );
    N_VectorCreate( 0.0, 1.0, 0.0, &Y );

    /* Compute the radius */

    N_VectorCreate( x2 - x1, y2 - y1, 0.0, &v2 );
    N_VectorCreate( x3 - x1, y3 - y1, 0.0, &v3 );

    N_VectorMagnitude( v2, &d2 );
    N_VectorMagnitude( v3, &d3 );
    rad = 0.5 *( d3 + d2 );

    if( rad LT tol )
    {
        N_ErrSet( NL_INP_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Compute the start and end angles */

    error = N_VectorsCosAngle( v2, X, &d2 );

    if( error EQ NL_YES )
        NL_OUT;
    sr = acos( d2 );

    if( y2 - y1 LT 0.0 )
        sr = 2.0 *NL_PI - sr;
    sang = (180.0 *sr) / NL_PI;

    d2 = x3 - x2;
    d3 = y3 - y2;

    if( sqrt( d2 * d2 + d3 * d3 )LT tol )
        er = sr + 2.0 *NL_PI;
    else
    {
        error = N_VectorsCosAngle( v3, X, &d3 );

        if( error EQ NL_YES )
            NL_OUT;
        er = acos( d3 );

        if( y3 - y1 LT 0.0 )
            er = 2.0 *NL_PI - er;

        if( er LT sr )
            er += 2.0 *NL_PI;
    }
    eang = (180.0 *er) / NL_PI;

    /* Now create the Nurbs circular arc */

    error = N_CreateCircArc( cen, X, Y, rad, sang, eang, ctp, cur, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now reset the start/end control points for accuracy */

    N_CrvGetCPts( cur, &n, &Pw );
    N_CPtGetW( Pw[0], &w );
    N_CPtFromWxWyWz( w * x2, w * y2, w * zt, w, &Pw[0] );
    N_CPtGetW( Pw[n], &w );
    N_CPtFromWxWyWz( w * x3, w * y3, w * zt, w, &Pw[n] );

    /* Rescale knots to [sr,er] */

    N_CrvGetDegree( cur, &p );
    N_CrvGetKnotVector( cur, &knt );
    N_CreateInterval( &I, sr, er );
    N_BasisReparam( knt, p, I );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges100Arc */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine  creates a bounded Nlib Nurbs line from the  NL_IGES
     Entity 110 data. A typical calling example is as follows:

       NL_REAL       x1, y1, z1, x2, y2, z2;
       NL_CURVE      cur;
       NL_STACKS     S;
       ...

       N_CrvInitArrays(&cur);
       N_Iges100Line(x1,y1,z1,x2,y2,z2,&cur,&S);

     If memory is  available, cur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  cur's  
     knot vector and polygon objects.


   ACCESS:

     x1,y1,z1 , input  ,  Start point of the line
     x2,y2,z2 , input  ,  End point of the line
     cur      , output ,  NURBS line
     S        , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

// ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges100Line( NL_REAL x1, NL_REAL y1, NL_REAL z1, NL_REAL x2, NL_REAL y2, NL_REAL z2, NL_CURVE *cur, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_POINT P, Q;

    NL_VECTOR V;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Create start point and vector */

    N_PtFromXYZ( x1, y1, z1, &P );
    N_PtFromXYZ( x2, y2, z2, &Q );
    N_VectorDir( P, Q, &V );

    /* Create Nlib line */

    error = N_CrvLineFromPtAndVector( P, V, cur, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges100Line */


/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an Nlib 4x4 transformation matrix from the
     NL_IGES Entity 124 transformation matrix data.  The transformation  can
     be applied to a curve or surface by calling  N_CrvTransform  or  N_SrfTransform,
     respectively. A typical calling example is as follows:

       NL_REAL       R[3][3];
       NL_VECTOR     T;
       NL_RMATRIX    rma;
       NL_STACKS     S;
       ...
       (load R and T)
       ...

       N_InitRealMatrix(&rma);
       N_Iges124Matrix(R,T,&rma,&S);

     If rma is initialized to the NULL matrix, memory is allocated in
     the  routine. If not, it is  assumed that  memory allocation has 
     been done. However, the routine checks for  the proper amount by
     looking  at  the  matrix  indexes (which  are  assumed to be set 
     correctly).  


   ACCESS:

     R   , input  ,  The 3x3 rotation portion of the matrix entity
     T   , input  ,  The translation vector
     rma , output ,  The Nlib 4x4 transformation matrix
     S   , input  ,  rma's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges124Matrix( NL_REAL R[3][3], NL_VECTOR T, NL_RMATRIX *rma, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges124Matrix");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj;

    NL_REAL ** RM;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check/get memory for the transformation matrix */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 4, rname, S );

    if( error EQ NL_YES )
        NL_OUT;
    N_GetRealMatrixPtr( rma, &RM );

    /* Load the rotation matrix and translation vector */

    RM[3][3] = 1.0;

    RM[0][3] = T.x;
    RM[1][3] = T.y;
    RM[2][3] = T.z;

    for ( ii = 0; ii < 3; ii++ )
    {
        RM[3][ii] = 0.0;

        for ( jj = 0; jj < 3; jj++ )
            RM[ii][jj] = R[ii][jj];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges124Matrix */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an Nlib Nurbs composite curve  from  n+1
     constituent curves. It can be used to convert the NL_IGES  Entity 102
     composite curve to Nlib Nurbs form (after all  constituent  curves
     have been converted). A typical calling example is as follows:

       NL_CURVE      **curT, curP;
       NL_REAL       tol;
       NL_STACKS     ST, SP;
       ...
       (Convert all constituent curves to Nlib Nurbs form, in curT)
       (Set tol)
       ...

       N_CrvInitArrays(&curP);
       N_Iges102CompositeCrv(n,curT,NL_IGES,tol,&curP,&ST,&SP);

     If memory is  available, curP is not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking at the highest indexes  in  curP's  
     knot vector and polygon objects.  ALL CONSTITUENT CURVES MUST BE ON
     THE SAME STACK "ST".


   ACCESS:

     n    , input  ,  High index of curves in curT (n+1 curves)
     curT , in/out ,  The constituent curves  (array of curve pointers).
                      These curves may be modified in place
     par  , input  ,  Flag:
                        NL_IGES       : Parameterize the composite curve as
                                     in  NL_IGES  Entity  102  (accumulated 
                                     length of the n+1 parameter ranges,
                                     not normalized)
                        NL_CHORDLENGTH: Parameterize  the  composite  curve
                                     according to  the  approximate  arc
                                     length of  its  constituent  curves
                                     (normalized)
     tol  , input  ,  Two points on the curve are considered the same if
                      they are within this distance of one another
     curP , output ,  Nlib NURBS composite curve
     ST   , input  ,  curT's memory stack
     SP   , input  ,  curP's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_Iges102CompositeCrv( NL_INDEX n, NL_CURVE ** curT, NL_FLAG par, NL_REAL tol, NL_CURVE *curP, NL_STACKS *ST, NL_STACKS *SP )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges102CompositeCrv");

    NL_PRIVATE NL_REAL eps = 1.0e-03;

    NL_FLAG error = NL_NO, *discard;

    NL_BOOLEAN ratl, dim3;

    NL_INDEX ii, jj, kk, ll, nn, nt, mt;

    NL_DEGREE p, max_p;

    NL_INTERVAL range;

    NL_PARAMETER ul, ur;

    NL_REAL *lens, wf, *UP, *UT, wx, wy, wz, w, wxl = 0.0, wyl = 0.0, wzl = 0.0, wl = 0.0, length;

    NL_CPOINT *Pw, *Tw;

    NL_CPOLYGON *pol;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* May need to discard some degenerate curves */

    discard = N_AllocFlag1dArray( n, &SL );

    if( discard EQ NULL )
        NL_QUIT;

    for ( ii = 0; ii <= n; ii++ )
        discard[ii] = NL_NO;

    /* Determine the parameterization and set the discard flag */
    /* Also determine maximum degree in the process            */

    max_p = 1;
    jj = 0;

    if( par EQ NL_IGES )
    {
        range.ul = 0.0;

        for ( ii = 0; ii <= n; ii++ )
        {
            N_CrvDetachPolygonKnot( curT[ii], &pol, &p, &knt );

            if( p GT max_p )
                max_p = p;
            N_CrvGetParamBounds( curT[ii], &ul, &ur );

            if( ur - ul EQ 0.0 ) /* only when precisely zero */
            {
                discard[ii] = NL_YES;
                jj += 1;
            }
            else
            {
                range.ur = range.ul + (ur - ul);
                N_BasisReparam( knt, p, range );
                range.ul = range.ur;
            }
        }
    }
    else
    {
        lens = N_AllocReal1dArray( n, &SL );

        length = 0.0;

        for ( ii = 0; ii <= n; ii++ )
        {
            N_CrvGetParamBounds( curT[ii], &ul, &ur );
            error = N_CrvArcLength( curT[ii], ul, ur, eps, NL_RELATIVE, &lens[ii] );

            if( error EQ NL_YES )
                NL_OUT;
            length += lens[ii];
        }

        range.ul = 0.0;

        for ( ii = 0; ii <= n; ii++ )
        {
            N_CrvDetachPolygonKnot( curT[ii], &pol, &p, &knt );

            if( p GT max_p )
                max_p = p;

            if( lens[ii]LE tol )
            {
                discard[ii] = NL_YES;
                jj += 1;
            }
            else
            {
                if( ii LT n )
                    range.ur = range.ul + lens[ii] / length;
                else
                    range.ur = 1.0;
                N_BasisReparam( knt, p, range );
                range.ul = range.ur;
            }
        }
    }

    if( jj GT n )
    {
        N_ErrSet( NL_INP_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Now elevate degrees of constiuent curves if required */

    for ( ii = 0; ii <= n; ii++ )
    {
        if( discard[ii]EQ NL_NO )
        {
            N_CrvGetDegree( curT[ii], &p );

            if( p LT max_p )
            {
                error = N_CrvElevateDegree( curT[ii], max_p - p, curT[ii], ST, ST );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    /* Determine dimensionality, rationality, and number of */
    /* control points of composite curve                    */

    ratl = NL_FALSE;
    dim3 = NL_FALSE;
    nn = 0;

    for ( ii = 0; ii <= n; ii++ )
    {
        if( discard[ii]EQ NL_NO )
        {
            dim3 = (dim3 OR N_CrvIs3d( curT[ii] ));
            ratl = (ratl OR N_IsCrvRat( curT[ii] ));
            N_CrvGetArraySizes( curT[ii], &jj, &kk );
            nn += jj;
        }
    }

    /* Get rid of all NL_NOW weights and NL_NOZ coord's; */
    /* it simplifies matters below                   */

    for ( ii = 0; ii <= n; ii++ )
    {
        if( discard[ii]EQ NL_NO )
        {
            N_CrvGetCPts( curT[ii], &nt, &Tw );

            for ( jj = 0; jj <= nt; jj++ )
            {
                N_CPtToWxWyWz( Tw[jj], &wx, &wy, &wz, &w );

                if( w EQ NL_NOW )
                    N_CPtSetW( 1.0, &Tw[jj] );

                if( wz EQ NL_NOZ )
                    N_CPtSetZ( 0.0, &Tw[jj] );
            }
        }
    }

    /* Check curve memory and get local notation */

    error = N_CrvSizeArrays( curP, nn, max_p, nn + max_p + 1, rname, SP );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsDegreeAndKnots( curP, &nn, &Pw, &max_p, &kk, &UP );

    /* Load knots and control points into curP. Load knots with   */
    /* multiplicity max_p at joints, and average start/end points */

    jj = 1; /* index into Pw array */

    for ( ii = 0; ii <= n; ii++ )
    {
        if( discard[ii]EQ NL_YES )
            continue;

        N_CrvGetCPtsDegreeAndKnots( curT[ii], &nt, &Tw, &p, &mt, &UT );
        N_CPtToWxWyWz( Tw[0], &wx, &wy, &wz, &w );

        if( jj EQ 1 )
        {
            for ( kk = 0; kk <= max_p; kk++ )
                UP[kk] = UT[kk];
            wf = 1.0; /* weight factor */
            N_CPtFromWxWyWz( wx, wy, wz, w, &Pw[0] );
        }
        else
        {
            wf = wl / w;
            jj -= 1; /* average control points */
            N_CPtFromWxWyWz( 0.5 *( wxl + wf * wx ), 0.5 *( wyl + wf * wy ), 0.5 *( wzl + wf * wz ), wl, &Pw[jj] );
            jj += 1;
        }

        for ( ll = 1; ll <= nt; ll++ )
        {
            N_CPtToWxWyWz( Tw[ll], &wx, &wy, &wz, &w );
            N_CPtFromWxWyWz( wf * wx, wf * wy, wf * wz, wf * w, &Pw[jj] );
            jj += 1;
        }

        for ( ll = max_p + 1; ll < mt; ll++ )
            UP[kk++] = UT[ll];

        if( ii NEQ n )
        {
            N_CPtToWxWyWz( Pw[jj - 1], &wxl, &wyl, &wzl, &wl );
        }
    }

    UP[kk] = UP[kk - 1];

    /* Set first/last control points precisely if in degenerate segments */

    if( par EQ NL_CHORDLENGTH )
    {
        if( discard[0]EQ NL_YES )
        {
            N_CPtGetW( Pw[0], &w );
            N_CrvGetCPts( curT[0], &nt, &Tw );
            N_CPtToXYZ( Tw[0], &wx, &wy, &wz ); /* Euclidean coords */
            N_CPtFromWxWyWz( w * wx, w * wy, w * wz, w, &Pw[0] );
        }

        if( discard[n]EQ NL_YES )
        {
            jj -= 1;
            N_CPtGetW( Pw[jj], &w );
            N_CrvGetCPts( curT[n], &nt, &Tw );
            N_CPtToXYZ( Tw[nt], &wx, &wy, &wz ); /* Euclidean coords */
            N_CPtFromWxWyWz( w * wx, w * wy, w * wz, w, &Pw[jj] );
        }
    }

    /* Now reset NL_NOZ and NL_NOW if applicable */

    if( !ratl OR!dim3 )
    {
        for ( ii = 0; ii <= nn; ii++ )
        {
            N_CPtToWxWyWz( Pw[ii], &wx, &wy, &wz, &w );

            if( !ratl )
                N_CPtSetW( NL_NOW, &Pw[ii] );

            if( !dim3 )
                N_CPtSetZ( NL_NOZ, &Pw[ii] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges102CompositeCrv */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine  creates a bounded  Nlib  Nurbs  plane  from  the 
     NL_IGES Entity 108, Form 1 data (bounded plane). The minmax box of the
     bounding curve is used to compute the bounds of the untrimmed NURBS
     plane. A typical calling example is as follows:

       NL_REAL       a, b, c, d;
       NL_MINMAXBOX  box;
       NL_SURFACE    sur;
       NL_STACKS     S;
       ...
       (get minmax box of bounding curve)
       ...

       N_SrfInitArrays(&sur);
       N_Iges108Plane(a,b,c,d,&box,&sur,&S);

     If memory is  available, sur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  sur's  
     knot vectors and control net objects.


   ACCESS:

     a,b  , input  ,  The coefficients of  the  implicit plane equation:
     c,d              a*x + b*y + c*z = d  ( (a,b,c) need not be normal-
                      ized)
     box  , input  ,  Minmax box of  the  bounding curve.  This box con-
                      tains the area on the plane which is bounded.
     sur  , output ,  NURBS plane
     S    , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges108Plane( NL_REAL a, NL_REAL b, NL_REAL c, NL_REAL d, NL_MINMAXBOX *box, NL_SURFACE *sur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges108Plane");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, k1;

    NL_POINT cen, pts[8];

    NL_DEGREE p;

    NL_REAL dd, num, den, x, y, z, xl, xr, yb, yt, zn, zf, *U, *V, dist;

    NL_CPOINT ** Pw;

    NL_VECTOR xx, yy, zz;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check/allocate memory for the surface structure */

    error = N_SrfSizeArrays( sur, 1, 1, 1, 1, 3, 3, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsDegreesAndKnots( sur, &ii, &ii, &Pw, &p, &p, &ii, &ii, &U, &V );

    /* Load knot vectors */

    for ( ii = 0; ii < 2; ii++ )
    {
        U[ii] = 0.0;
        U[ii + 2] = 1.0;
        V[ii] = 0.0;
        V[ii + 2] = 1.0;
    }

    /* Project 8 corner points of minmax box to plane */

    den = a * a + b * b + c * c;
    N_GetBBoxData( box, &xl, &xr, &yb, &yt, &zn, &zf );
    k1 = 0;

    for ( ii = 0; ii < 2; ii++ )
    {
        if( ii EQ 0 )
            x = xl;
        else
            x = xr;

        for ( jj = 0; jj < 2; jj++ )
        {
            if( jj EQ 0 )
                y = yb;
            else
                y = yt;

            for ( kk = 0; kk < 2; kk++ )
            {
                if( kk EQ 0 )
                    z = zn;
                else
                    z = zf;
                num = d - (x * a + y * b + z * c);

                if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );
                dd = num / den;
                N_PtFromXYZ( x + dd * a, y + dd * b, z + dd * c, &pts[k1] );
                k1 += 1;
            }
        }
    }

    /*  Get the center of gravity of the 8 points  */

    N_CopyPt( pts[0], &cen );

    for ( ii = 1; ii < 8; ii++ )
        N_Sum2Pts( cen, pts[ii], &cen );
    N_ScalePt( 1.0 / 8.0, cen, &cen );

    /* Get the max distance of the 8 points from the center */

    dist = 0.0;

    for ( ii = 0; ii < 8; ii++ )
    {
        N_DistPtPt( cen, pts[ii], &dd );

        if( dd GT dist )
        {
            dist = dd;
            k1 = ii;
        }
    }

    /* Get the local coordinate system of the Nurbs plane */

    N_VectorDir( cen, pts[k1], &xx );
    den = sqrt( den );
    N_VectorCreate( a / den, b / den, c / den, &zz );
    N_VectorCross( zz, xx, &yy );
    N_VectorScale( xx, 1.02, &xx ); /* extend a bit past the bounds */
    N_VectorScale( yy, 1.02, &yy );

    /* Now compute and load the control points */

    N_TranslateSum2Pts( cen, -1.0, xx, -1.0, yy, &pts[0] );
    N_PtToCPt( pts[0], &Pw[0][0] );
    N_TranslateSum2Pts( cen, 1.0, xx, -1.0, yy, &pts[0] );
    N_PtToCPt( pts[0], &Pw[1][0] );
    N_TranslateSum2Pts( cen, -1.0, xx, 1.0, yy, &pts[0] );
    N_PtToCPt( pts[0], &Pw[0][1] );
    N_TranslateSum2Pts( cen, 1.0, xx, 1.0, yy, &pts[0] );
    N_PtToCPt( pts[0], &Pw[1][1] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges108Plane */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an  Nlib  extruded  surface from  the NL_IGES
     Entity 122 extruded surface data (TABCYL). A typical calling example
     is as follows:

       NL_POINT      P;
       NL_CURVE      cur;
       NL_SURFACE    sur;
       NL_STACKS     S;
       ...
       (convert the directrix curve to Nlib form, cur)
       ...

       N_SrfInitArrays(&sur);
       N_Iges122ExtrudedSrf(&cur,P,&sur,&S);

     If memory is  available, sur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  sur's  
     knot vector and control net objects.


   ACCESS:

     cur , input  ,  The directrix curve
     P   , input  ,  Terminate point of the generatrix line segment
     sur , output ,  Nlib extruded surface
     S   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges122ExtrudedSrf( NL_CURVE *cur, NL_POINT P, NL_SURFACE *sur, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_POINT Q;

    NL_VECTOR W;

    NL_PARAMETER ul, ur;

    NL_REAL d;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Get extrude vector and distance */

    N_CrvGetParamBounds( cur, &ul, &ur );
    error = N_CrvEval( cur, ul, NL_LEFT, &Q );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDir( Q, P, &W );
    N_DistPtPt( Q, P, &d );

    /* Now create the extruded surface */

    error = N_CreateSrfExtrudeCrv( cur, W, d, NL_VDIR, sur, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges122ExtrudedSrf */

#ifdef NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an  Nlib  ruled  surface  from  the  NL_IGES
     Entity 118, Form 1 ruled surface data. This function  modifies  the
     original rail curves. If either or both of the rail curves are rat-
     ional, the parameterization in the v-direction may  not  be  linear 
     (as specified in NL_IGES). This routine assumes the case (DIRFLG=0). A
     typical calling example is as follows:

       NL_CURVE      cur1, cur2;
       NL_SURFACE    sur;
       NL_STACKS     SC, SS;
       ...
       (convert the rail curves to Nlib form, cur1 and cur2)
       ...

       N_SrfInitArrays(&sur);
       N_Iges118RuledSrf(&cur1,&cur2,&sur,&SC,&SS);

     If memory is  available, sur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  sur's  
     knot vector and control net objects. NL_BOTH CURVES MUST BELONG TO THE
     SAME MEMORY STACK


   ACCESS:

     cur1 , input  ,  The first rail curve. This curve will be modified
     cur2 , input  ,  The second rail curve. This curve will be modified
     sur  , output ,  Nlib ruled surface
     SC   , input  ,  Stack of cur1 and cur2
     SS   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_Iges118RuledSrf( NL_CURVE *cur1, NL_CURVE *cur2, NL_SURFACE *sur, NL_STACKS *SC, NL_STACKS *SS )
{

    NL_FLAG error = NL_NO;

    /* Call N_CreateRuledSrf directly, nothing else to do */

    error = N_CreateRuledSrf( cur1, cur2, NL_VDIR, sur, SC, SS );

    if( error EQ NL_YES )
        NL_OUT;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Iges118RuledSrf */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates a nonrational Nlib  B-spline surface from
     the NL_IGES Entity 114 parameteric spline surface data.  The resulting
     B-spline surface has knots of multiplicity equal to u- and v-degree
     at all interior u- and v-breakpoints,  respectively.  Knot  removal
     may be applied to  eliminate some of these multiplicities (based on
     the  continuity  of  the  spline).  A typical calling example is as 
     follows:

       NL_INDEX      m, n;
       NL_PARAMETER  *tu, *tv;
       NL_REAL       ***polys;
       NL_SURFACE    sur;
       NL_STACKS     S;
       ...
       (load the polynomial coefficients into array polys)
       ...

       N_SrfInitArrays(&sur);
       N_Iges114NonRatSrf(m,n,tu,tv,polys,&sur,&S);

     If memory is  available, sur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  sur's  
     knot vector and control net objects.


   ACCESS:

     m     , input  ,  There are m polynomial patches  (m+1 breakpoints)
                       in the u-direction
     n     , input  ,  There are n polynomial patches  (n+1 breakpoints)
                       in the v-direction
     tu    , input  ,  The m+1 u-breakpoints
     tv    , input  ,  The n+1 v-breakpoints
     polys , input  ,  The  polynomial coefficients.  polys[i][j][k] are
                       the coefficients of the (i,j)-th patch: 0<=i<=m-1
                       0<=j<=n-1 and 0<=k<=47 (48 coefficients for  each
                       patch). Notice that the non-required (n+1)-th row
                       and (m+1)-th column  of  patches must not be con-
                       tained in this array 
     sur   , output ,  B-spline surface
     S     , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges114NonRatSrf( NL_INDEX m, NL_INDEX n, NL_PARAMETER *tu, NL_PARAMETER *tv, NL_REAL *** polys, NL_SURFACE *sur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges114NonRatSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, ii, jj, nn, mm, rr, ss, ku, kv;

    NL_DEGREE pp, qq;

    NL_REAL xx, yy, zz, *U, *V, d, e, vgam, ugam[4];

    NL_CPOINT ** Pw, ** Aw, ** Bw;

    NL_RMATRIX cum, cvm;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* First determine degree in each direction */

    for ( i = 3; i > 1; i-- )
        for ( ii = 0; ii < m; ii++ )
            for ( jj = 0; jj < n; jj++ )
                for ( j = 0; j < 3; j++ )
                    for ( k = 0; k < 4; k++ )
                        if( polys[ii][jj][IND( j, k, i )] != 0.0 )
                            goto uloop;

    uloop:

    pp = (NL_DEGREE)i;

    for ( i = 3; i > 1; i-- )
        for ( ii = 0; ii < m; ii++ )
            for ( jj = 0; jj < n; jj++ )
                for ( j = 0; j < 3; j++ )
                    for ( k = 0; k < 4; k++ )
                        if( polys[ii][jj][IND( j, i, k )] != 0.0 )
                            goto vloop;

    vloop:

    qq = (NL_DEGREE)i;

    /*  Get nn,mm,rr,ss and check for memory in surface structure */

    nn = m * pp;
    rr = nn + pp + 1;
    mm = n * qq;
    ss = mm + qq + 1;

    error = N_SrfSizeArrays( sur, nn, mm, pp, qq, rr, ss, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsDegreesAndKnots( sur, &nn, &mm, &Pw, &pp, &qq, &rr, &ss, &U, &V );

    /* Allocate memory for one power basis/Bezier patch */

    Aw = N_AllocCPt2dArray( pp, qq, &SL );
    Bw = N_AllocCPt2dArray( pp, qq, &SL );

    if( Aw EQ NULL OR Bw EQ NULL )
        NL_QUIT;

    /* Get inverses of power basis conversion matrices */

    N_InitRealMatrix( &cum );
    error = N_BezInversePowerMatrix( pp, &cum, &SL );

    if( error EQ NL_YES )
        NL_OUT;
    N_InitRealMatrix( &cvm );
    error = N_BezInversePowerMatrix( qq, &cvm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Now loop over all patches, reparameterize each one, convert */
    /* each to Bezier, and piece the Beziers together with pp-ful  */
    /* and qq-ful knots                                            */

    for ( i = 0; i <= pp; i++ )
        U[i] = tu[0];

    for ( i = 0; i <= qq; i++ )
        V[i] = tv[0];

    ku = pp + 1;
    kv = qq + 1;

    ugam[0] = 1.0;

    for ( ii = 0; ii < m; ii++ )
    {
        d = tu[ii + 1] - tu[ii];

        for ( k = 1; k <= pp; k++ )
            ugam[k] = d * ugam[k - 1];

        for ( k = 0; k < pp; k++ )
            U[ku++] = tu[ii + 1];

        for ( jj = 0; jj < n; jj++ )
        {
            if( ii EQ 0 )
            {
                for ( k = 0; k < qq; k++ )
                    V[kv++] = tv[jj + 1];
            }

            d = tv[jj + 1] - tv[jj];
            vgam = 1.0;

            for ( i = 0; i <= qq; i++ )
            {
                for ( j = 0; j <= pp; j++ )
                {
                    e = ugam[j] * vgam;
                    xx = e * polys[ii][jj][IND( 0, i, j )];
                    yy = e * polys[ii][jj][IND( 1, i, j )];
                    zz = e * polys[ii][jj][IND( 2, i, j )];
                    N_CPtFromWxWyWz( xx, yy, zz, NL_NOW, &Aw[j][i] );
                }
                vgam = d * vgam;
            }

            error = N_RealMatrixMultiplyRealMatrixTranspose( &cum, Aw, &cvm, Bw );

            if( error EQ NL_YES )
                NL_OUT;

            k = ii * pp;
            l = jj * qq;

            for ( i = 0; i <= pp; i++ )
                for ( j = 0; j <= qq; j++ )
                    N_CopyCPt( Bw[i][j], &Pw[k + i][l + j] );
        }
    }

    U[ku] = tu[m];
    V[kv] = tv[n];

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges114NonRatSrf */


/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an  Nlib  revolved  surface from the NL_IGES
     Entity 120 revolved surface data. The v-direction  is  circular  in
     the Nlib surface (i.e. isocurves at fixed u-values are circular). The
     parameter ranges are those given in NL_IGES; i.e. the u-range is
     inherited from the generatrix curve, and v is in [sa,ta]. A typical
     calling example is as follows:

       NL_POINT      P1, P2;
       NL_REAL       sa, ta;
       NL_CURVE      cur;
       NL_SURFACE    sur;
       NL_STACKS     S;
       ...
       (convert the generatrix curve to Nlib form, cur)
       ...

       N_SrfInitArrays(&sur);
       N_Iges120RevolvedSrf(P1,P2,&cur,sa,ta,&sur,&S);

     If memory is  available, sur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  sur's  
     knot vector and control net objects.


   ACCESS:

     P1  , input  ,  First point on the axis line
     P2  , input  ,  Second point on the axis line
     cur , input  ,  The generatrix curve
     sa  , input  ,  The start angle, MEASURED IN DEGREES
     ta  , input  ,  The terminate angle, MEASURED IN DEGREES
                     Note that both angles  are  measured  counterclock-
                     wise while looking  from P2 to P1  (as specified in
                     NL_IGES)
     sur , output ,  Nlib revolved surface
     S   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges120RevolvedSrf( NL_POINT P1, NL_POINT P2, NL_CURVE *cur, NL_REAL sa, NL_REAL ta, NL_SURFACE *sur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges120RevolvedSrf");

    NL_FLAG error = NL_NO;

    NL_CURVE cur2, *curp;

    NL_KNOTVECTOR *knu, *knv;

    NL_REAL ang;

    NL_VECTOR T;

    NL_RMATRIX rma;

    NL_INTERVAL I;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for input error */

    ang = ta - sa;

    if( ang LE 0.0 )
    {
        N_ErrSet( NL_INP_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    else if( ang GT 360.0 )
        ang = 360.0;

    /* Get axis vector */

    N_VectorDir( P1, P2, &T );

    /* If necessary, rotate curve about axis by the angle sa */

    curp = cur;

    if( sa NEQ 0.0 )
    {
        curp = &cur2;
        N_CrvInitArrays( curp );
        error = N_CrvCopy( cur, curp, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_InitRealMatrix( &rma );

        error = N_CreateRotationMatrixAboutAxis( P1, T, sa, &rma, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvTransform( curp, &rma );
    }

    /* Now create revolved surface */

    error = N_CreateRevolvedSrf( curp, P1, T, ang, NL_QUADRATIC, sur, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Swap u,v parameters and scale new v to correspond to NL_IGES */

    error = N_SwapUV( sur, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreateInterval( &I, sa, ta );
    N_SrfGetKnotVectors( sur, &knu, &knv );
    N_BasisReparam( knv, 2, I );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges120RevolvedSrf */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an Nlib degree 1 piecewise  linear  curve
     from the NL_IGES Entity 106, Form 12 data (xyz-coordinates). A typical
     calling example is as follows:

       NL_INDEX      n;
       NL_POINT      *P;
       NL_CURVE      cur;
       NL_STACKS     S;
       ...
       (load n+1 points into P array)
       ...

       N_CrvInitArrays(&cur);
       N_Iges106LinearCrv(n,P,NL_IGES,&cur,&S);

     If memory is  available, cur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  cur's  
     knot vector and polygon objects.


   ACCESS:

     n    , input  ,  There are n+1 points in the P array
     P    , input  ,  The points defining the piecewise linear curve
     par  , input  ,  Flag:
                        NL_IGES       : Parameterize the degree 1 curve  as
                                     in NL_IGES Entity 106 (not normalized)
                        NL_CHORDLENGTH: Parameterize the degree 1 curve ac-
                                     cording to the chord length  of its 
                                     constituent chords (normalized)
     cur  , output ,  NURBS degree 1 curve
     S    , input  ,  cur's memory stack

     2d curves are identified by setting the z coordinate values of all control points
     to the magic value, NL_NOZ.  This function passes through the input Z coordinate
     values to the output curve.  Build a 2d curve by setting input point 
     Z coordinate values to NL_NOZ.  Build a 3d curve on the Z=0 plane by setting all
     the input point Z coordinate value to 0.0.  Convert a 3d curve to 2d with
     N_Crv3dTo2d which sets control point Z coordinate values to NL_NOZ and convert 
     a 2d curve to a 3d curve on the Z=0 plane with a call to N_Crv3dTo2d which
     sets control point Z coordinate values to 0.0.
     
   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/


   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges106LinearCrv( NL_INDEX n, NL_POINT *P, NL_FLAG par, NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges106LinearCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, nn, mm;

    NL_DEGREE pp;

    NL_REAL *U;

    NL_CPOINT *Pw;

    /* Check/allocate memory for the curve structure */

    error = N_CrvSizeArrays( cur, n, 1, n + 2, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsDegreeAndKnots( cur, &nn, &Pw, &pp, &mm, &U );

    /* Determine if curve is xy-planar */

    /* Note: a 3d curve in the xy plane is not necessarily a 2d curve.
    * If the caller wants a 2d curve, that can be done separately: N_Crv3dTo2d()*/

    for ( ii = 0; ii <= nn; ii++ )
    {
        N_PtToCPt( P[ii], &Pw[ii] );
    }

    /* Compute knots */

    if( par EQ NL_IGES )
    {
        for ( ii = 0; ii <= nn; ii++ )
            U[ii + 1] = (NL_REAL)ii;
    }
    else
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)P, nn, NL_EPOINT, NL_CHORDLENGTH, &U[1] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    U[0] = U[1];
    U[nn + 2] = U[nn + 1];

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_Iges106LinearCrv */



/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine creates an  Nlib  Nurbs conic arc or full ellipse
     from the NL_IGES Entity 104 data. The conic is in a plane  parallel to
     the xy-plane.  As per NL_IGES specification,  the  conic  must  be  in 
     standard position,  and  it  must not be a degenerate case (line or 
     point). A typical calling example is as follows:

       NL_REAL       a, b, c, d, e, f, zt, x1, y1, x2, y2;
       NL_CURVE      cur;
       NL_STACKS     S;
       ...

       N_CrvInitArrays(&cur);
       N_Iges104ConicArc(a,b,c,d,e,f,zt,x1,y1,x2,y2,NL_QUADRATIC,&cur,&S);

     If memory is  available, cur is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  cur's  
     knot vector and polygon objects.


   ACCESS:

     a,b,c , input  ,  The coefficients of the implicit conic equation:
     d,e,f             a*x**2 + b*x*y + c*y**2 + d*x + e*y + f = 0
     zt    , input  ,  The displacement of the conic arc from  the  xy-
                       plane
     x1,y1 , input  ,  Start point of the arc
     x2,y2 , input  ,  End point of the arc
     ctp   , input  ,  Flag:
                         NL_QUADRATIC: degree 2 conic is created,  ellipse
                                    may have double internal knots
                         NL_QUARTIC  : degree 4 ellipse is created,  poss-
                                    ibly with quadruple internal knots, 
                                    but  better  parameterization  than
                                    the quadratic;  parabola and hyper-
                                    will be quadratic
                         NL_QUINTIC  : degree 5 ellipse is created with no
                                    internal knots; parabola and hyper-
                                    will be quadratic
     cur   , output ,  NURBS conic
     S     , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_Iges104ConicArc( NL_REAL a, NL_REAL b, NL_REAL c, NL_REAL d, NL_REAL e, NL_REAL f, NL_REAL zt, NL_REAL x1, NL_REAL y1, NL_REAL x2, NL_REAL y2, NL_FLAG ctp, NL_CURVE *cur, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Iges104ConicArc");

    NL_PRIVATE NL_REAL NRTOL = 1.0e-7;

    NL_FLAG error = NL_NO, type;

    NL_REAL x, y, as = 0.0, ae = 0.0, r1 = 0.0, r2 = 0.0, t, h, max;

    NL_POINT P, P0, P2;

    NL_VECTOR T0, T2;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for bad input data (many cases) */

    max = fabs( a );

    if( max LT fabs( b ) )
        max = fabs( b );

    if( max LT fabs( c ) )
        max = fabs( c );

    if( max LT fabs( d ) )
        max = fabs( d );

    if( max LT fabs( e ) )
        max = fabs( e );

    if( max LT fabs( f ) )
        max = fabs( f );

    if( max LT NL_ZCTL )
    {
        N_ErrSet( NL_INP_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    if( fabs( a ) / max LT NRTOL AND fabs( c ) / max LT NRTOL )
    { /* degenerate case; point or line */
        N_ErrSet( NL_INP_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    if( fabs( b ) / max GT NRTOL )
    { /* conic not in standard position */
        N_ErrSet( NL_INP_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Determine type of conic. Also more checks for bad data. */
    /* Creation algorithm depends on conic type. */

    if( fabs( a )LT fabs( c ) )
        h = fabs( a ) / fabs( c );
    else
        h = fabs( c ) / fabs( a );

    if( (fabs( a ) / max LT NRTOL OR fabs( c ) / max LT NRTOL)AND h LT NRTOL )
    { /* it's a parabola */

        type = NL_PARABOLA;

        if( fabs( f ) / max GT NRTOL )
        { /* parabola not in standard position */
            N_ErrSet( NL_INP_ERR, rname );
            error = NL_YES;
            NL_OUT;
        }

        /* Distinguish parabola based on axis */

        if( fabs( a ) / max LT NRTOL )
        {     /* x axis is axis of the parabola */
            if( fabs( d ) / max LT NRTOL OR fabs( e ) / max GT NRTOL )
            { /* parabola not in standard position */
                N_ErrSet( NL_INP_ERR, rname );
                error = NL_YES;
                NL_OUT;
            }

            h = -c / d;
            t = 0.5 *( y1 + y2 );
            N_PtFromXYZ( h * t * t, t, zt, &P );

            if( y1 LT y2 )
            {
                N_VectorCreate( 2.0 *h * y1, 1.0, 0.0, &T0 );
                N_VectorCreate( 2.0 *h * y2, 1.0, 0.0, &T2 );
            }
            else
            {
                N_VectorCreate( -2.0 *h * y1, -1.0, 0.0, &T0 );
                N_VectorCreate( -2.0 *h * y2, -1.0, 0.0, &T2 );
            }
        }
        else
        {     /* y axis is axis of the parabola */
            if( fabs( e ) / max LT NRTOL OR fabs( d ) / max GT NRTOL )
            { /* parabola not in standard position */
                N_ErrSet( NL_INP_ERR, rname );
                error = NL_YES;
                NL_OUT;
            }

            h = -a / e;
            t = 0.5 *( x1 + x2 );
            N_PtFromXYZ( t, h * t * t, zt, &P );

            if( x1 LT x2 )
            {
                N_VectorCreate( 1.0, 2.0 *h * x1, 0.0, &T0 );
                N_VectorCreate( 1.0, 2.0 *h * x2, 0.0, &T2 );
            }
            else
            {
                N_VectorCreate( -1.0, -2.0 *h * x1, 0.0, &T0 );
                N_VectorCreate( -1.0, -2.0 *h * x2, 0.0, &T2 );
            }
        }
    }
    else
    { /* it's an ellipse or a hyperbola */
        if( N_FloatOpIsBad( f, a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        if( N_FloatOpIsBad( f, c, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        if( a *c GT 0.0 )
        { /* it's an ellipse */

            type = NL_ELLIPSE;

            if( fabs( d ) / max GT NRTOL OR fabs( e ) / max GT NRTOL OR fabs( f ) / max LT NRTOL )
            { /* ellipse not in standard position or zero radius */
                N_ErrSet( NL_INP_ERR, rname );
                error = NL_YES;
                NL_OUT;
            }

            N_PtFromXYZ( 0.0, 0.0, zt, &P ); /* P is center of ellipse */
            N_VectorCreate( 1.0, 0.0, 0.0, &T0 );
            N_VectorCreate( 0.0, 1.0, 0.0, &T2 );

            r1 = -f / a;
            r2 = -f / c;

            if( r1 LT 0.0 OR r2 LT 0.0 )
            { /* invalid equation */
                N_ErrSet( NL_INP_ERR, rname );
                error = NL_YES;
                NL_OUT;
            }

            r1 = sqrt( r1 );
            r2 = sqrt( r2 ); /* major and minor radii */

            as = (180.0 / NL_PI) * atan2( y1 / r2, x1 / r1 );
            ae = (180.0 / NL_PI) * atan2( y2 / r2, x2 / r1 );

            if( as LE 0.0 )
                as = as + 360.0;

            while( ae LE as )
                ae = ae + 360.0;
        }
        else
        { /* it's a hyperbola */

            type = NL_HYPERBOLA;

            if( fabs( d ) / max GT NRTOL OR fabs( e ) / max GT NRTOL OR fabs( f ) / max LT NRTOL )
            { /* hyperbola not in standard position or zero radius */
                N_ErrSet( NL_INP_ERR, rname );
                error = NL_YES;
                NL_OUT;
            }

            /* Distinguish hyperbola based on axis */

            r1 = f / a;
            r2 = f / c;

            if( r1 LT 0.0 )
            { /* x axis is transverse axis of the hyperbola */

                y = 0.5 *( y1 + y2 );
                x = sqrt( -(f + c * y * y) / a );

                if( x1 LT 0.0 )
                    x = -x;
                N_PtFromXYZ( x, y, zt, &P );

                y = 1.0;
                x = -(c * y1) / (a * x1);

                if( y1 GT y2 )
                {
                    y = -1.0;
                    x = -x;
                }
                N_VectorCreate( x, y, 0.0, &T0 );

                y = 1.0;
                x = -(c * y2) / (a * x2);

                if( y1 GT y2 )
                {
                    y = -1.0;
                    x = -x;
                }
                N_VectorCreate( x, y, 0.0, &T2 );
            }
            else
            { /* y axis is transverse axis of the hyperbola */

                x = 0.5 *( x1 + x2 );
                y = sqrt( -(f + a * x * x) / c );

                if( y1 LT 0.0 )
                    y = -y;
                N_PtFromXYZ( x, y, zt, &P );

                x = 1.0;
                y = -(a * x1) / (c * y1);

                if( x1 GT x2 )
                {
                    x = -1.0;
                    y = -y;
                }
                N_VectorCreate( x, y, 0.0, &T0 );

                x = 1.0;
                y = -(a * x2) / (c * y2);

                if( x1 GT x2 )
                {
                    x = -1.0;
                    y = -y;
                }
                N_VectorCreate( x, y, 0.0, &T2 );
            }
        }
    }

    /* Now create the Nlib NURBS conic */

    if( type EQ NL_ELLIPSE )
    {
        error = N_CreateEllipticalArcAngType( P, T0, T2, r1, r2, as, ae, 0, ctp, cur, S );
    }
    else
    {
        N_PtFromXYZ( x1, y1, zt, &P0 );
        N_PtFromXYZ( x2, y2, zt, &P2 );
        error = N_CreateConicArc( P0, T0, P2, T2, P, cur, S );
    }

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Iges104ConicArc */

#ifdef NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine is a utility which computes unclamped,  nonperio-
     dic control points from periodic control points.  It can be used to
     convert periodic curves  and  surfaces  to  unclamped,  nonperiodic
     form,  which  can  then be converted (clamped) to Nlib form via the
     routines N_Iges126Crv and N_Iges128Srf. The definition of periodic implies:
       - P0,...,Pn and P0 != Pn (first point not repeated in the set)
       - u0,...,um and m=n+1
       - u0 and um evaluate to the same point
       - u0 appears only once in the set, but um may  have  multiplicity
         between 1 and the degree
     A typical calling example is as follows:

       NL_INDEX      nold, nnew, mult;
       NL_DEGREE     p;
       NL_POINT      *Pold, *Pnew;
       NL_CPOINT     *Pwold, *Pwnew;
       ...
       (allocate arrays Aold, Anew, and load Aold, and compute mult)
       ...

       N_IgesNonPeriodicCPts(nold,p,(NL_VOID *)Pold,NL_EPOINT,mult,&nnew,(NL_VOID *)Pold);
       N_IgesNonPeriodicCPts(nold,p,(NL_VOID *)Pwold,NL_HPOINT,mult,&nnew,(NL_VOID *)Pwnew);

     Either 3D points or 4D (weighted) control points can be  processed
     by this routine. Note that the NL_POINT or NL_CPOINT arrays must be type
     casted to NL_VOID *.  Notice  also that the conversion may be done in
     place by passing the same pointers for old and new.


   ACCESS:

     nold  , input  ,  High index of (periodic) points in Aold
     p     , input  ,  The degree of the b-splines. p >= 2 must hold
     Aold  , in/out ,  Array 3D points or 4D control points (periodic).
                       Must be passed in as a NL_VOID *
     flg   , input  ,  Flag:
                        NL_EPOINT: Aold and Anew are of type NL_POINT
                        NL_HPOINT: Aold and Anew are of type NL_CPOINT
     mult  , input  ,  Multiplicity of the last periodic knot, um, (see
                       above). 1 <= mult <= p must hold
     nnew  , output ,  High index of points in Anew. nnew will have the 
                       value, nold+p-mult+1, upon return
     Anew  , output ,  Array of 3D points or 4D  control  points  (non-
                       periodic and unclamped).  Memory for this  array 
                       must be  passed in (NL_VOID *),  with  high  index, 
                       at least nold+p-mult+1. If Aold = Anew, the con-
                       version is done in place


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_IgesNonPeriodicCPts( NL_INDEX nold, NL_DEGREE p, NL_VOID *Aold, NL_FLAG flg, NL_INDEX mult, NL_INDEX *nnew, NL_VOID *Anew )
{
    NL_PRIVATE NL_STRING rname = _T("N_IgesNonPeriodicCPts");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk;

    NL_POINT *Pold, *Pnew;

    NL_CPOINT *Pwold, *Pwnew;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for errors */

    if( p LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( mult LT 1 OR mult GT p )
        NL_ERROR( NL_INP_ERR );

    /* Load the new points */

    kk = (p - mult + 1) / 2;

    if( flg EQ NL_EPOINT )
    {
        Pold = (NL_POINT *)Aold;
        Pnew = (NL_POINT *)Anew;

        for ( ii = nold; ii >= 0; ii-- )
            N_CopyPt( Pold[ii], &Pnew[ii + kk] );

        for ( ii = 0; ii < kk; ii++ )
            N_CopyPt( Pnew[nold + ii + 1], &Pnew[ii] );

        if( (p + mult) % 2 EQ 1 )
            jj = kk - 1;
        else
            jj = kk;

        for ( ii = 0; ii <= jj; ii++ )
            N_CopyPt( Pnew[ii + kk], &Pnew[nold + kk + ii + 1] );
    }
    else
    {
        Pwold = (NL_CPOINT *)Aold;
        Pwnew = (NL_CPOINT *)Anew;

        for ( ii = nold; ii >= 0; ii-- )
            N_CopyCPt( Pwold[ii], &Pwnew[ii + kk] );

        for ( ii = 0; ii < kk; ii++ )
            N_CopyCPt( Pwnew[nold + ii + 1], &Pwnew[ii] );

        if( (p + mult) % 2 EQ 1 )
            jj = kk - 1;
        else
            jj = kk;

        for ( ii = 0; ii <= jj; ii++ )
            N_CopyCPt( Pwnew[ii + kk], &Pwnew[nold + kk + ii + 1] );
    }

    *nnew = nold + p - mult + 1;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_IgesNonPeriodicCPts */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine is a utility which computes unclamped,  nonperio-
     dic knots from periodic knots.  It can be used to convert periodic,
     closed curves and surfaces to unclamped, nonperiodic form which can 
     then be converted (clamped) to Nlib form via the routines  N_Iges126Crv  
     and N_Iges128Srf. The definition of periodic implies:
       - P0,...,Pn and P0 != Pn (first point not repeated in the set)
       - u0,...,um and m=n+1
       - u0 and um evaluate to the same point
       - u0 appears only once in the set, but um may  have  multiplicity
         between 1 and the degree
     A typical calling example is as follows:

       NL_INDEX      mold, mnew, mult;
       NL_DEGREE     p;
       NL_REAL       *Uold, *Unew;
       ...
       (allocate arrays Uold, Unew, and load Uold, and compute mult)
       ...

       N_IgesNonPeriodicKnots(mold,p,Uold,mult,Uold,&mnew);
       N_IgesNonPeriodicKnots(mold,p,Uold,mult,Unew,&mnew);

     Notice that the conversion may be done in place by passing the same
     pointers for old and new.


   ACCESS:

     mold  , input  ,  High index of (periodic) knots in Uold
     p     , input  ,  The degree of the b-splines. p >= 2 must hold
     Uold  , in/out ,  The input (periodic) knots
     mult  , input  ,  The multiplicity of the last knot in  Uold  (1 <=
                       mult <= p) must hold)
     Unew  , output ,  The output (nonperiodic, unclamped) knots. Memory
                       for this array must be passed in (high index must 
                       be at least mold+2*p-mult+1). If Uold = Unew, the 
                       conversion will be done in place
     mnew  , output ,  High index of knots in Unew


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_IgesNonPeriodicKnots( NL_INDEX mold, NL_DEGREE p, NL_REAL *Uold, NL_INDEX mult, NL_REAL *Unew, NL_INDEX *mnew )
{
    NL_PRIVATE NL_STRING rname = _T("N_IgesNonPeriodicKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, nold;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for errors */

    if( p LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( mult LT 1 OR mult GT p )
        NL_ERROR( NL_INP_ERR );

    /* Compute and load the new knots */

    nold = mold - 1;
    jj = nold + 1;

    for ( ii = jj; ii >= 0; ii-- )
        Unew[ii + p] = Uold[ii];

    for ( ii = 0; ii < p; ii++ )
        Unew[p - ii - 1] = Unew[p - ii] - (Unew[jj + p - ii] - Unew[nold + p - ii]);

    *mnew = mold + 2 * p - mult + 1;
    jj = nold + p;
    kk = p - mult + 1;

    for ( ii = 0; ii < kk; ii++ )
        Unew[jj + ii + 2] = Unew[jj + ii + 1] + (Unew[p + ii + 1] - Unew[p + ii]);

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_IgesNonPeriodicKnots */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine converts a periodic Nurbs curve to an  unclamped,
     nonperiodic curve.  The  resulting curve can then be converted to a 
     valid Nlib curve via the routine N_Iges126Crv. The definition of perio-
     dic implies:
       - P0,...,Pn and P0 != Pn (first point not repeated in the set)
       - u0,...,um and m=n+1
       - u0 and um evaluate to the same point
       - u0 appears only once in the set, but um may  have  multiplicity
         between 1 and the degree      
     A typical calling example is as follows:

       NL_INDEX      no, nn;
       NL_DEGREE     deg;
       NL_REAL       *to, *tn, *wo, *wn;
       NL_POINT      *Po, *Pn;
       NL_STACKS     S;
       ...
       (allocate and load arrays)
       ...

       N_IgesNonPeriodicCrv(no,deg,to,wo,Po,flg,&nn,&tn,&wn,&Pn,&S);
     

   ACCESS:

     no  , input  ,  Upper index of control points (no+1 ctrl pts)
     deg , input  ,  Degree of the basis functions (deg >= 2 must hold)
     to  , input  ,  The knots (see flg)
     wo  , input  ,  The no+1 weights
     Po  , input  ,  The no+1 Euclidean control points (unweighted)
     flg , input  ,  Flag (parent system):
                      NL_SOLIDWORKS: Data is from a SolidWorks NL_IGES 126 En-
                                  tity. On input, there are no+deg+2
                                  knots; on output, nn+deg+2 knots
     nn  , output ,  New upper index of control points and weights (nn+1
                     ctrl pts in Pn and weights in wn)
     tn  , output ,  The new nn+deg+2 knots
     wn  , output ,  The new nn+1 weights
     Pn  , output ,  The new nn+1 Euclidean control points (unweighted)
     S   , input  ,  Memory stack for tn, wn, and Pn

     2d curves are identified by setting the z coordinate values of all control points
     to the magic value, NL_NOZ.  This function only outputs 3d curves; an input
     2d curve will be returned as a 3d curve built on the z=0 plane.

     To convert a 3d curve into a 2d curve call N_Crv3dTo2d which sets all 
     control point z coordinate values to NL_NOZ.

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_IgesNonPeriodicCrv( NL_INDEX no, NL_DEGREE deg, NL_REAL *to, NL_REAL *wo, NL_POINT *Po, NL_FLAG flg, NL_INDEX *nn, NL_REAL ** tn, NL_REAL ** wn, NL_POINT ** Pn, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_IgesNonPeriodicCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, n, m, mult, p_off, k_off;

    NL_REAL *U, *w;

    NL_POINT *P;

    NL_CPOINT *Pw, *Pw_old;

    NL_BOOLEAN rat;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for error */

    if( deg LE 1 )
        NL_ERROR( NL_INP_ERR );

    /* Compute new unclamped, nonperiodic data (method depends */
    /* on parent system)                                       */

    switch( flg )
    {
        case NL_SOLIDWORKS:

            n = no - 1;
            m = n + 1;
            p_off = 1;
            k_off = deg;

            /* compute the multiplicity of the last knot */

            for ( ii = m; ii > 0; ii-- )
                if( to[ii + k_off]NEQ to[ii + k_off - 1] )
                    break;

            mult = m - ii + 1;

            /* compute the nonperiodic, unclamped knots */

            U = N_AllocReal1dArray( m + 2 * deg - mult + 1, S );

            if( U EQ NULL )
                NL_QUIT;

            error = N_IgesNonPeriodicKnots( m, deg, &to[k_off], mult, U, &ii );

            if( error EQ NL_YES )
                NL_OUT;

            /* compute the corresponding control points and weights */

            w = N_AllocReal1dArray( n + deg - mult + 1, S );

            if( w EQ NULL )
                NL_QUIT;

            P = N_AllocPt1dArray( n + deg - mult + 1, S );

            if( P EQ NULL )
                NL_QUIT;

            if( mult EQ deg )
            { /* special case; periodic and nonperiodic cps/wts are same */
                for ( ii = 0; ii <= no; ii++ )
                {
                    w[ii] = wo[ii];
                    N_CopyPt( Po[ii], &P[ii] );
                }

                *nn = no;
            }
            else
            {
                rat = NL_FALSE; /* determine rationality */

                for ( ii = 0; ii < n; ii++ )
                    if( wo[ii + p_off]NEQ wo[ii + 1 + p_off] )
                    {
                        rat = NL_TRUE;
                        break;
                    }

                if( rat )
                {
                    /* Weight the control points */

                    Pw = N_AllocCPt1dArray( n + deg - mult + 1, &SL );

                    if( Pw EQ NULL )
                        NL_QUIT;
                    Pw_old = N_AllocCPt1dArray( n, &SL );

                    if( Pw_old EQ NULL )
                        NL_QUIT;

                    for ( ii = 0; ii <= n; ii++ )
                        N_Weight( Po[ii + p_off], wo[ii + p_off], &Pw_old[ii] );

                    error = N_IgesNonPeriodicCPts( n, deg, (NL_VOID *)Pw_old, NL_HPOINT, mult, nn, (NL_VOID *)Pw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    jj = *nn;

                    for ( ii = 0; ii <= jj; ii++ )
                        N_CPtToPtAndW( Pw[ii], &P[ii], &w[ii] );
                }
                else
                {
                    error = N_IgesNonPeriodicCPts( n, deg, (NL_VOID *) &Po[p_off], NL_EPOINT, mult, nn, (NL_VOID *)P );

                    if( error EQ NL_YES )
                        NL_OUT;

                    jj = *nn;

                    for ( ii = 0; ii <= jj; ii++ )
                        w[ii] = wo[p_off];
                }
            }

            break;

        default:

            /* Due to limited test cases, this modification is at customer request */
            /* without much testing */
            /* n = no - 1; */
            n = no;
            m = n + 1;

            /* compute the multiplicity of the last knot */

            for ( ii = m; ii > 0; ii-- )
                if( to[ii]NEQ to[ii - 1] )
                    break;

            mult = m - ii + 1;

            /* compute the nonperiodic, unclamped knots */
            U = N_AllocReal1dArray( m + 2 * deg - mult + 1, S );
            if( U EQ NULL )
                NL_QUIT;

            error = N_IgesNonPeriodicKnots( m, deg, &to[0], mult, U, &ii );
            if( error EQ NL_YES )
                NL_OUT;

            /* compute the corresponding control points and weights */
            w = N_AllocReal1dArray( n + deg - mult + 1, S );
            if( w EQ NULL )
                NL_QUIT;

            P = N_AllocPt1dArray( n + deg - mult + 1, S );

            if( P EQ NULL )
                NL_QUIT;

            /* Maybe there is another situation where this special case is correct, but not with this condition. Elias. */
	        /* if( mult EQ deg )
	        {  
                special case; periodic and nonperiodic cps/wts are same
		    	for ( ii = 0; ii <= no; ii++ )
		    	{
		    		w[ii] = wo[ii];
		    		N_CopyPt( Po[ii], &P[ii] );
		    	}

		    	*nn = no;
		    }
		    else */
            {
                rat = NL_FALSE; /* determine rationality */

                for ( ii = 0; ii < n; ii++ )
                    if( wo[ii]NEQ wo[ii + 1] )
                    {
                        rat = NL_TRUE;
                        break;
                    }

                if( rat )
                {
                    /* Weight the control points */

                    Pw = N_AllocCPt1dArray( n + deg - mult + 1, &SL );

                    if( Pw EQ NULL )
                        NL_QUIT;
                    Pw_old = N_AllocCPt1dArray( n, &SL );

                    if( Pw_old EQ NULL )
                        NL_QUIT;

                    for ( ii = 0; ii <= n; ii++ )
                        N_Weight( Po[ii], wo[ii], &Pw_old[ii] );

                    error = N_IgesNonPeriodicCPts( n, deg, (NL_VOID *)Pw_old, NL_HPOINT, mult, nn, (NL_VOID *)Pw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    jj = *nn;

                    for ( ii = 0; ii <= jj; ii++ )
                        N_CPtToPtAndW( Pw[ii], &P[ii], &w[ii] );
                }
                else
                {
                    error = N_IgesNonPeriodicCPts( n, deg, (NL_VOID *) &Po[0], NL_EPOINT, mult, nn, (NL_VOID *)P );

                    if( error EQ NL_YES )
                        NL_OUT;

                    jj = *nn;

                    for ( ii = 0; ii <= jj; ii++ )
                        w[ii] = wo[0];
                }
            }

            break;
    }

    /* Assign output */

    *Pn = P;
    *wn = w;
    *tn = U;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_IgesNonPeriodicCrv */

/*******************************************************************//**


   DESCRIPTION:

     This NL_IGES routine converts a periodic Nurbs surface to an  unclamp-
     ed, nonperiodic surface. The resulting surface can then be convert-
     ed to a valid Nlib surface via the routine N_Iges128Srf. The definition 
     of periodic (in the u- or v-direction) implies:
       - P0,...,Pn and P0 != Pn (first point not repeated in the set)
       - u0,...,um and m=n+1
       - u0 and um evaluate to the same point
       - u0 appears only once in the set, but um may  have  multiplicity
         between 1 and the degree      
     A typical calling example is as follows:

       NL_INDEX      no, nn, mo, mn;
       NL_DEGREE     p, q;
       NL_REAL       *uo, *un, *vo, *vn, **wo, **wn;
       NL_POINT      **Po, **Pn;
       NL_STACKS     S;
       ...
       (allocate and load arrays)
       ...

       N_IgesNonPeriodicSrf(no,mo,p,q,uo,vo,wo,Po,dflg,sflg,&nn,&mn,&un,&vn,&wn,
           &Pn,&S);
     

   ACCESS:

     no   , input  ,  High index of control points in u direction
     mo   , input  ,  High index of control points in v direction
     p,q  , input  ,  Degrees of the basis functions  (must be >= 2 in a
                      periodic direction)
     uo   , input  ,  The u-knots (see sflg and dflg)
     vo   , input  ,  The v-knots (see sflg and dflg)
     wo   , input  ,  The (no+1)x(mo+1) weights
     Po   , input  ,  The (no+1)x(mo+1) Euclidean  control  points  (un-
                      weighted)
     dflg , input  ,  Flag (direction):
                       NL_UDIR : surface is periodic in u only  (vo is  not
                              used, and vn is not set)
                       NL_VDIR : surface is periodic in v only  (uo is  not
                              used, and un is not set)
                       NL_UVDIR: surface is periodic in both u and v
     sflg , input  ,  Flag (parent system):
                       NL_SOLIDWORKS: Data is  from  a  SolidWorks NL_IGES 128
                                   Entity.  On input,  there are  no+p+2
                                   u-knots; on output, nn+p+2 u-knots (v
                                   direction is  analogous)
     nn   , output ,  New high index of control points and weights in u-
                      direction
     mn   , output ,  New high index of control points and weights in v-
                      direction
     un   , output ,  The new nn+p+2 u-knots  (not allocated nor set  if
                      dflg = NL_VDIR)
     vn   , output ,  The new mn+q+2 v-knots  (not allocated nor set  if
                      dflg = NL_UDIR)
     wn   , output ,  The new (nn+1)x(mn+1) weights
     Pn   , output ,  The new  (nn+1)x(mn+1)  Euclidean  control  points 
                      (unweighted)
     S    , input  ,  Memory stack for un, vn, wn, and Pn


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_IgesNonPeriodicSrf( NL_INDEX no, NL_INDEX mo, NL_DEGREE p, NL_DEGREE q, NL_REAL *uo, NL_REAL *vo, NL_REAL ** wo, NL_POINT ** Po, NL_FLAG dflg, NL_FLAG sflg, NL_INDEX *nn, NL_INDEX *mn, NL_REAL ** un, NL_REAL ** vn, NL_REAL *** wn, NL_POINT *** Pn, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_IgesNonPeriodicSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, n, m, r, s, u_mult = 0, v_mult = 0, k_off, nv, nu;

    NL_REAL dd, *U, *V, ** w;

    NL_POINT ** P, *Q = NULL;

    NL_CPOINT ** Pw = NULL, *Qw = NULL;

    NL_BOOLEAN rat;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Check for error */

    if( p LE 1 AND( NOT NL_VDIR ) )
        NL_ERROR( NL_INP_ERR );

    if( q LE 1 AND( NOT NL_UDIR ) )
        NL_ERROR( NL_INP_ERR );

    /* Compute new unclamped, nonperiodic data (method depends */
    /* on parent system)                                       */

    switch( sflg )
    {
        case NL_SOLIDWORKS:

            rat = NL_FALSE; /* determine rationality */
            dd = wo[0][0];

            for ( ii = 0; ii <= no; ii++ )
                for ( jj = 0; jj <= mo; jj++ )
                    if( wo[ii][jj]NEQ dd )
                    {
                        rat = NL_TRUE;
                        break;
                    }

            /* Set some indexes and compute new knots */

            if( dflg EQ NL_VDIR )
            {
                n = nu = no;
                *nn = n; /* just set indexes */
            }
            else
            {
                n = no - 1;
                r = n + 1;
                k_off = p;

                /* compute the multiplicity of the last u-knot */

                for ( ii = r; ii > 0; ii-- )
                    if( uo[ii + k_off]NEQ uo[ii + k_off - 1] )
                        break;

                u_mult = r - ii + 1;

                /* compute the nonperiodic, unclamped u-knots */

                U = N_AllocReal1dArray( r + 2 * p - u_mult + 1, S );

                if( U EQ NULL )
                    NL_QUIT;
                *un = U;

                error = N_IgesNonPeriodicKnots( r, p, &uo[k_off], u_mult, U, &ii );

                if( error EQ NL_YES )
                    NL_OUT;

                nu = n + p - u_mult + 1; /* new high index of ctrl pts in u */
                *nn = nu;
            }

            if( dflg EQ NL_UDIR )
            {
                m = nv = mo;
                *mn = m; /* just set indexes */
            }
            else
            {
                m = mo - 1;
                s = m + 1;
                k_off = q;

                /* compute the multiplicity of the last v-knot */

                for ( ii = s; ii > 0; ii-- )
                    if( vo[ii + k_off]NEQ vo[ii + k_off - 1] )
                        break;

                v_mult = s - ii + 1;

                /* compute the nonperiodic, unclamped v-knots */

                V = N_AllocReal1dArray( s + 2 * q - v_mult + 1, S );

                if( V EQ NULL )
                    NL_QUIT;
                *vn = V;

                error = N_IgesNonPeriodicKnots( s, q, &vo[k_off], v_mult, V, &ii );

                if( error EQ NL_YES )
                    NL_OUT;

                nv = m + q - v_mult + 1; /* new high index of ctrl pts in v */
                *mn = nv;
            }

            /* Allocate memory for weights and ctrl pts */

            w = N_AllocReal2dArray( nu, nv, S );

            if( w EQ NULL )
                NL_QUIT;
            *wn = w;

            P = N_AllocPt2dArray( nu, nv, S );

            if( P EQ NULL )
                NL_QUIT;
            *Pn = P;

            jj = NL_MAX( nu, nv );

            if( rat )
            {
                Qw = N_AllocCPt1dArray( jj, &SL );

                if( Qw EQ NULL )
                    NL_QUIT;

                if( dflg EQ NL_UVDIR )
                {
                    Pw = N_AllocCPt2dArray( nu, nv, S );

                    if( Pw EQ NULL )
                        NL_QUIT;
                }
            }
            else
            {
                Q = N_AllocPt1dArray( jj, &SL );

                if( Q EQ NULL )
                    NL_QUIT;
            }

            /* compute the corresponding control points and weights */

            if( rat )
            {
                if( dflg EQ NL_UDIR OR dflg EQ NL_UVDIR )
                {
                    for ( ii = 0; ii <= mo; ii++ )
                    {
                        if( p EQ u_mult )
                            kk = 0;
                        else
                            kk = 1;

                        for ( jj = kk; jj <= no; jj++ ) /* Weight the control points */
                            N_Weight( Po[jj][ii], wo[jj][ii], &Qw[jj - kk] );

                        if( u_mult LT p )
                        {
                            error = N_IgesNonPeriodicCPts( n, p, (NL_VOID *)Qw, NL_HPOINT, u_mult, &jj, (NL_VOID *)Qw );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }

                        if( dflg EQ NL_UDIR )
                        {
                            for ( jj = 0; jj <= nu; jj++ )
                                N_CPtToPtAndW( Qw[jj], &P[jj][ii], &w[jj][ii] );
                        }
                        else
                        {
                            for ( jj = 0; jj <= nu; jj++ )
                                N_CopyCPt( Qw[jj], &Pw[jj][ii] );
                        }
                    }

                    if( dflg EQ NL_UVDIR )
                    {
                        for ( ii = 0; ii <= nu; ii++ ) /* now the v-direction */
                        {
                            if( q EQ v_mult )
                                kk = 0;
                            else
                                kk = 1;

                            for ( jj = kk; jj <= mo; jj++ )
                                N_CopyCPt( Pw[ii][jj], &Qw[jj - kk] );

                            if( v_mult LT q )
                            {
                                error = N_IgesNonPeriodicCPts( m, q, (NL_VOID *)Qw, NL_HPOINT, v_mult, &jj, (NL_VOID *)Qw );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }

                            for ( jj = 0; jj <= nv; jj++ )
                                N_CPtToPtAndW( Qw[jj], &P[jj][ii], &w[jj][ii] );
                        }
                    }
                }
                else
                {
                    for ( ii = 0; ii <= no; ii++ ) /* this is the NL_VDIR case */
                    {
                        if( q EQ v_mult )
                            kk = 0;
                        else
                            kk = 1;

                        for ( jj = kk; jj <= mo; jj++ ) /* Weight the control points */
                            N_Weight( Po[ii][jj], wo[ii][jj], &Qw[jj - kk] );

                        if( v_mult LT q )
                        {
                            error = N_IgesNonPeriodicCPts( m, q, (NL_VOID *)Qw, NL_HPOINT, v_mult, &jj, (NL_VOID *)Qw );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }

                        for ( jj = 0; jj <= nv; jj++ )
                            N_CPtToPtAndW( Qw[jj], &P[ii][jj], &w[ii][jj] );
                    }
                }
            } /* end of rational case */
            else
            {
                if( dflg EQ NL_UDIR OR dflg EQ NL_UVDIR )
                {
                    for ( ii = 0; ii <= mo; ii++ )
                    {
                        if( p EQ u_mult )
                            kk = 0;
                        else
                            kk = 1;

                        for ( jj = kk; jj <= no; jj++ )
                            N_CopyPt( Po[jj][ii], &Q[jj - kk] );

                        if( u_mult LT p )
                        {
                            error = N_IgesNonPeriodicCPts( n, p, (NL_VOID *)Q, NL_EPOINT, u_mult, &jj, (NL_VOID *)Q );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }

                        for ( jj = 0; jj <= nu; jj++ )
                            N_CopyPt( Q[jj], &P[jj][ii] );
                    }

                    if( dflg EQ NL_UVDIR )
                    {
                        for ( ii = 0; ii <= nu; ii++ ) /* now the v-direction */
                        {
                            if( q EQ v_mult )
                                kk = 0;
                            else
                                kk = 1;

                            for ( jj = kk; jj <= mo; jj++ )
                                N_CopyPt( P[ii][jj], &Q[jj - kk] );

                            if( v_mult LT q )
                            {
                                error = N_IgesNonPeriodicCPts( m, q, (NL_VOID *)Q, NL_EPOINT, v_mult, &jj, (NL_VOID *)Q );

                                if( error EQ NL_YES )
                                    NL_OUT;
                            }

                            for ( jj = 0; jj <= nv; jj++ )
                                N_CopyPt( Q[jj], &P[ii][jj] );
                        }
                    }
                }
                else
                {
                    for ( ii = 0; ii <= no; ii++ ) /* this is the NL_VDIR case */
                    {
                        if( q EQ v_mult )
                            kk = 0;
                        else
                            kk = 1;

                        for ( jj = kk; jj <= mo; jj++ )
                            N_CopyPt( Po[ii][jj], &Q[jj - kk] );

                        if( v_mult LT q )
                        {
                            error = N_IgesNonPeriodicCPts( m, q, (NL_VOID *)Q, NL_EPOINT, v_mult, &jj, (NL_VOID *)Q );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }

                        for ( jj = 0; jj <= nv; jj++ )
                            N_CopyPt( Q[jj], &P[ii][jj] );
                    }
                }

                for ( ii = 0; ii <= nu; ii++ )
                    for ( jj = 0; jj <= nv; jj++ )
                        w[ii][jj] = wo[0][0];
            } /* end of nonrational case */

            break;

        default:

            NL_ERROR( NL_INP_ERR );

            break;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_IgesNonPeriodicSrf */
#endif // NLIB_UNUSED
