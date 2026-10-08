// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************************/
/* CrvGeom.c : Geometry Processing Function Definitions that act on NL_CURVE objects */
/**********************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

// ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **
NL_FLAG ST_bezlen( NL_CPOINT *Pw, NL_DEGREE p, NL_REAL ls, NL_REAL *eps, NL_REAL *len, NL_REAL tl, 
                   NL_REAL tr, NL_REAL *ul, NL_REAL *ur, NL_FLAG *fin );


/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine finds  the point Q  on a NURBS
     curve closest to a given point P. The parameter corresponding to Q
     is also  returned. The point  P can be on the curve (inversion) or
     off  the  curve  (projection). The  routine uses  Newton iteration  
     with a  start  parameter  passed in. The  iteration limit NL_ITLIM is 
     defined as a global  parameter  in "globals.h".  The returned  u,Q
     represent the  best solution found,  regardless  of  the  returned
     error value. A typical calling example is:

       NL_CURVE      cur;
       NL_POINT      P, Q;
       NL_REAL       top, toc;
       NL_PARAMETER  u0, u;
       ...
       (define cur, get P, u0, top, toc);
       ...
       N_CrvClosestPt(&cur,P,u0,top,toc,&u,&Q);

     If convergence is not reached, the predefined NL_CON_ERR (convergence
     error) is returned. In any case,  the best values of  u and Q  are 
     returned.


   ACCESS:
   
     cur , input  ,  NURBS curve
     P   , input  ,  Point to be inverted/projected
     u0  , input  ,  Guess parameter
     top , input  ,  Point coincidence tolerance
     toc , input  ,  Zero cosine tolerance
     u   , output ,  Parameter corresponding to Q, i.e. Q = C(u)
     Q   , output ,  Point on the curve closest to P


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvClosestPt( NL_CURVE *cur, NL_POINT P, NL_PARAMETER u0, NL_REAL top, NL_REAL toc, NL_PARAMETER *u, NL_POINT *Q )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvClosestPt");

    NL_FLAG error = NL_NO, closed = NL_NO;

    NL_INDEX k, m, ucount;

    NL_REAL *U, num, den, der, dot = 0.0, dis, best_dis, zco;

    NL_PARAMETER uold, unew;

    NL_POINT D[3];

    NL_VECTOR V;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetKnots( cur, &m, &U );

    /* Perform Newton interations till convergence reached */

    if( N_CrvIsClosed( cur ) )
        closed = NL_YES;

    k = 0;
    unew = u0;
    best_dis = NL_BIGD;

    ucount = 0;

    while( k LT NL_ITLIM )
    {
        /* Get derivatives */

        error = N_CrvDerivs( cur, unew, NL_LEFT, 2, D );

        if( error EQ NL_YES )
            NL_OUT;

        /* Prepare for convergence test */

        N_VectorDiff( D[0], P, &V );
        N_VectorDot( D[1], V, &num );
        N_VectorMagnitude( D[1], &der );

        /* Check #1: point coincidence */

        N_VectorMagnitude( V, &dis );

        if( dis LT best_dis )
        {
            *u = unew;
            N_CopyPt( D[0], Q );
            best_dis = dis;
        }

        if( dis LE top )
            break;

        /* Check #2: zero cosine */

        if( N_FloatOpIsBad( num, der * dis, NL_DIVISION ) )
            NL_ERROR( NL_CON_ERR );

        zco = fabs( num ) / (der * dis);

        if( zco LE toc AND dis EQ best_dis )
            break;

        /* No convergence, compute new parameter */

        N_VectorDot( D[2], V, &dot );
        den = dot + der * der;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_CON_ERR );

        uold = unew;
        unew = uold - num / den;

        /* Check #3: parameter range */

        if( closed )
        {
            if( unew LT U[0] )
            {
                if( (U[0] - unew)LT( 100.0 *( U[m] - U[0] ) ) )
                {
                    while( unew LT U[0] )
                        unew = U[m] - U[0] + unew;
                    ucount = 0;
                }
                else
                {
                    if( ucount GE 2 )
                        NL_ERROR( NL_CON_ERR );
                    unew = uold;
                    ucount += 1;
                }
            }
            else if( unew GT U[m] )
            {
                if( (unew - U[m])LT( 100.0 *( U[m] - U[0] ) ) )
                {
                    while( unew GT U[m] )
                        unew = U[0] - U[m] + unew;
                    ucount = 0;
                }
                else
                {
                    if( ucount GE 2 )
                        NL_ERROR( NL_CON_ERR );
                    unew = uold;
                    ucount += 1;
                }
            }
            else
                ucount = 0;
        }
        else
        {
            if( unew LT U[0] )
                unew = U[0];

            if( unew GT U[m] )
                unew = U[m];
        }

        /* Check #4: parameter change */

        if( fabs( (unew - uold) * der )LE top AND dis EQ best_dis )
            break;

        k++;
    }

    /* If no convergence, error out */

    if( k GE NL_ITLIM )
        NL_ERROR( NL_CON_ERR );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvClosestPt */


/*******************************************************************//**


   DESCRIPTION:

     This  geometry processing  routine  converts a  NURBS curve into 
     piecewise power basis form. MEMORY TO STORE THE OUTPUT CURVES IS 
     ALLOCATED  INSIDE THE  ROUTINE. A typical calling example is: 

       NL_CURVE   cur, **cpl;
       NL_INDEX   i, k;
       NL_CPOINT  *aw;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvNurbsToPiecewise(&cur,NL_NORMALIZED,&cpl,&k,&SG);
       ...
       aw = cpl[i]->pol->Pw;
       ...

     cpl[i], 0<=i<=k, is a pointer to the i-th power basis curve.  The
     power  basis  curve  segment is  defined as a  NL_CURVE object;  the 
     vector  coefficients  aw[i] are  stored in  place of the  control 
     points, and the knots U[0] and U[1]  represent the bounds of  the
     interval over which the curve is defined.


   ACCESS:
   
     cur  , input  ,  NURBS curve to be converted
     pflg , input  ,  Flag, indicating desired parameterization of the
                      power basis segments (U[0],U[1] values):
                       = NL_INHERITED  : inherited knot span bounds
                       = NL_NORMALIZED : [0,1]
     cpl  , output ,  Array of power basis curves
     k    , output ,  Highest index in cpl
     SG   , input  ,  cpl's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvNurbsToPiecewise( NL_CURVE *cur, NL_FLAG pflg, NL_CURVE *** cpl, NL_INDEX *k, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, l, n, m, r, s, nsp, mlt, is, ie, save;

    NL_DEGREE p;

    NL_REAL *U, *A, *alfs, *omas, num;

    NL_KNOTVECTOR *knt;

    NL_RMATRIX pom;

    NL_CPOINT *Pw, *Bw, *Nw, *aw;

    NL_CURVE ** cpa;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur, &knt );

    /* Get number of segments and allocate memory */

    N_BasisGetSpanCount( knt, p, &nsp );

    cpa = N_Alloc1dArrayCrvs( p, p, 1, nsp - 1, SG );

    if( cpa EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    Bw = N_AllocCPt1dArray( p, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt1dArray( p, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    /* Get power basis conversion matrix */

    N_InitRealMatrix( &pom );
    error = N_BezGetPowerConversionMatrix( p, &pom, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize */

    is = p;
    ie = p + 1;
    l = -1;

    /* skip over first knot over-multiplicity  ( bad data) */
    while( ie LT m AND U[ie - 1]EQ U[ie] )
    {
        is++;
        ie++;
    };

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Pw[i + is - p], &Bw[i] );

    /******************************************************/
    /* Loop through the knot vector and do the following: */
    /*  (1) extract Bezier segment,                       */
    /*  (2) convert segment into power basis form.        */
    /******************************************************/

    while( ie LT m )
    {
        /* Initialize */

        l = l + 1;

        if( l >= nsp )
            NL_QUIT;

        N_CrvGetCPtsAndKnots( cpa[l], &aw, &A );

        /* Get knot multiplicity */

        i = ie;

        while( ie LT m AND U[ie]EQ U[ie + 1] )
            ie++;
        mlt = ie - i + 1;
        r = p - mlt;

        /* Insert the knot */

        if( mlt LT p )
        {
            num = U[ie] - U[is];

            for ( i = p; i > mlt; i-- )
            {
                alfs[i - mlt - 1] = num / (U[is + i] - U[is]);
                omas[i - mlt - 1] = 1.0 - alfs[i - mlt - 1];
            }

            for ( i = 1; i <= r; i++ )
            {
                s = mlt + i;
                save = r - i;

                for ( j = p; j >= s; j-- )
                {
                    N_Combine2CPts( alfs[j - s], Bw[j], omas[j - s], Bw[j - 1], &Bw[j] );
                }

                if( ie LT m )
                {
                    N_CopyCPt( Bw[p], &Nw[save] );
                }
            }
        }

        /* Convert segment to Power basis form */

        if( pflg EQ NL_NORMALIZED )
        {
            A[0] = 0.0;
            A[1] = 1.0;
        }
        else
        {
            A[0] = U[is];
            A[1] = U[ie];
        }

        error = N_BezToPowerBasis( Bw, p, &pom, A[0], A[1], aw );

        if( error EQ NL_YES )
            NL_OUT;

        /* Conversion completed - prepare for next segment */

        if( ie LT m )
        {
            for ( i = 0; i < r; i++ )
                N_CopyCPt( Nw[i], &Bw[i] );

            for ( i = r; i <= p; i++ )
                N_CopyCPt( Pw[ie - p + i], &Bw[i] );
        }
        is = ie;
        ie = ie + 1;
    }

    *k = nsp - 1;
    *cpl = cpa;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvNurbsToPiecewise */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine  converts a  piecewise power 
     basis curve to NURBS form. If the output curve is initialized to
     the NULL curve, memory is  allocated. Otherwise it is checked if
     enough memory is passed in. A typical calling example is:

       NL_CURVE   **cpl, cur;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cpl and k);
       ...
       N_CrvInitArrays(&cur);
       N_CrvPiecewiseToNurbs(cpl,k,&cur,&SG);

     cpl[i], 0<=i<=k, is a pointer to the i-th power basis curve. The
     power  basis  curve  segment is  defined as a  NL_CURVE object; the 
     vector  coefficients  aw[i] are  stored in  place of the control 
     points, and the knots U[0] and U[1]  represent the bounds of the
     interval over which the curve is defined. IT IS ASSUMED THAT THE
     NL_CURVE OBJECT cur IS DEFINED IN THE CALLING ROUTINE.


   ACCESS:
   
     cpl , input  ,  Piecewise  power  basis  curve  (array  of curve 
                     pointers)
     k   , input  ,  Highest  index in  cpl  array (number  of pieces
                     minus one)
     cur , output ,  NURBS curve 
     SG  , input  ,  cur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvPiecewiseToNurbs( NL_CURVE ** cpl, NL_INDEX k, NL_CURVE *cur, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPiecewiseToNurbs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, l, n, m, glo_par;

    NL_DEGREE p;

    NL_REAL *A, *U, u0, um, uu;

    NL_RMATRIX ipm;

    NL_CPOINT *aw, *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetDegree( cpl[0], &p );

    n = (k + 1) * p;
    m = (k + 2) * p + 1;

    /* Determine parameterization of curve segments (global or local) */

    N_CrvGetKnots( cpl[0], &j, &U );
    u0 = U[0];
    um = U[1];

    glo_par = 1; /* global */

    for ( i = 1; i <= k; i++ )
    {
        N_CrvGetKnots( cpl[i], &j, &U );

        if( U[0]NEQ um )
        {
            glo_par = 0; /* local */
            break;
        }
        um = U[1];
    }

    /* Check for curve memory */

    error = N_CrvSizeArrays( cur, n, p, m, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Pw, &U );

    /* Get inverse of power basis conversion matrix */

    N_InitRealMatrix( &ipm );
    error = N_BezInversePowerMatrix( p, &ipm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize */

    l = 0;
    m = p;

    for ( i = 0; i <= p; i++ )
        U[i] = u0;

    /********************************************************/
    /* Convert curve to NURBS form as follows:              */
    /*  (1) stitch segments together into piecewise Bezier, */
    /*  (2) remove all removable knots.                     */
    /********************************************************/

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetCPtsAndKnots( cpl[i], &aw, &A );

        error = N_PowerBasisToBez( aw, p, &ipm, A[0], A[1], &Pw[l] );

        if( error EQ NL_YES )
            NL_OUT;

        if( glo_par EQ 1 )
        {
            for ( j = 1; j <= p; j++ )
                U[m + j] = A[1];
        }
        else
        {
            uu = A[1] - A[0];

            for ( j = 1; j <= p; j++ )
                U[m + j] = U[m] + uu;
        }

        l = l + p;
        m = m + p;
    }

    U[m + 1] = U[m];

    /* Remove all removable knots */

    error = N_CrvRemoveKnots( cur, NL_MTOL, cur, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvPiecewiseToNurbs */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine  computes  the  arclength of a 
     segment of a NURBS curve. It decomposes  the curve on the fly into  
     piecewise Bezier, and calls a recursive Bezier routine to  compute 
     the arclength of the Bezier segment. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  ul, ur;
       NL_REAL       tol, len
       ...
       (define cur, get ul, ur and tol);
       ...
       N_CrvArcLength(&cur,ul,ur,tol,NL_RELATIVE,&len);

     The tolerance tol can be  absolute or relative. If it is relative,
     it is scaled by the largest extent of the curve.
     Handles Lines and CircularArcs as special cases for performance.


   ACCESS:
   
     cur   , input  ,  NURBS curve
     ul,ur , input  ,  Parameters corresponding to curve segment
     tol   , input  ,  Tolerance 
     flg   , input  ,  Flag:
                         NL_ABSOLUTE: tol is an absolute tolerance
                         NL_RELATIVE: tol is a relative tolerance, i.e. it
                                   is scaled by the extent of the curve
     len   , output ,  Length of curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CrvArcLength
 (NL_CURVE    *cur,   // in : NURBS curve
  NL_PARAMETER ul,    // in : Start Param of CrvIvl to measure
  NL_PARAMETER ur,    // in : End   Param of CrvIvl to measure 
  NL_REAL      tol,   // in : calc ArcLength to this tolerance
  NL_FLAG      flg,   // in : NL_ABSOLUTE: absolute tolerance
                      //      NL_RELATIVE: relative tolerance scaled by the curve extent
  NL_REAL     *len )  // out: length of Cur +/- tol
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvArcLength");
    NL_PRIVATE NL_REAL ctl = 1.0e-09;

    NL_FLAG typ, error = NL_NO;

    NL_INDEX i, j, k, n, m, mlt, r, s, a, b, nsp, save;

    NL_DEGREE p;

    NL_REAL *U, *alfs, *omas, d, ltol, num, eps, lbz, rad, al, lctl;

    NL_KNOTVECTOR *knt;

    NL_POINT AA;

    NL_VECTOR BB, XX, YY;

    NL_CPOINT *Pw, *Bw, *Nw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Take care of parameter degeneracy */

    if( fabs( ur - ul )LT NL_PTOL )
    {
        *len = 0.0;

        NL_OUT;
    }

    /* Extract segment */

    N_CrvInitArrays( &curA );
    error = N_CrvExtractCrvSeg( cur, ul, ur, &curA, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvGetBBoxMaxDiagDist( &curA, &d );

    if( error EQ NL_YES )
        NL_OUT;

    lctl = d * ctl;

    /* Get line and circle */

    error = N_CrvGetType( &curA, lctl, &AA, &BB, &XX, &YY, &rad, &al, &typ );

    if( error EQ NL_YES )
        NL_OUT;

    if( typ EQ NL_NLINE )
    {
        N_VectorSum( AA, BB, &BB );
        N_DistPtPt( AA, BB, len );

        NL_OUT;
    }

    if( typ EQ NL_NCIRCLE )
    {
        *len = (al / 180.0) * NL_PI * rad;

        NL_OUT;
    }

    /* Adjust tolerance */

    switch( flg )
    {
        case NL_RELATIVE:
            ltol = d * tol;
            break;

        case NL_ABSOLUTE:
            ltol = tol;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Get local notation and prepare for decomposition */

    N_CrvGetCPtsDegreeAndKnots( &curA, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( &curA, &knt );
    N_BasisGetSpanCount( knt, p, &nsp );

    eps = ltol / nsp;
    *len = 0.0;

    /* Get local memory */

    Bw = N_AllocCPt1dArray( p, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt1dArray( p, &SL );

    if( Nw EQ NULL )
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

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Pw[i], &Bw[i] );

    /* Decompose curve and compute length of each piece */

    while( b LT m )
    {
        i = b;

        while( b LT m AND U[b]EQ U[b + 1] )
            b++;

        mlt = b - i + 1;
        r = p - mlt;

        if( r GT 0 )
        {
            num = U[b] - U[a];

            for ( k = p; k > mlt; k-- )
            {
                alfs[k - mlt - 1] = num / (U[a + k] - U[a]);
                omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
            }

            for ( j = 1; j <= r; j++ )
            {
                save = r - j;
                s = mlt + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfs[k - s], Bw[k], omas[k - s], Bw[k - 1], &Bw[k] );
                }
                N_CopyCPt( Bw[p], &Nw[save] );
            }
        }

        /* Compute length of Bezier segment and update total length */

        error = N_BezCalcLength( Bw, p, eps, &lbz );

        if( error EQ NL_YES )
            NL_OUT;

        *len += lbz;

        /* Initialize for next pass through */

        if( b LT m )
        {
            if( r < 0 )
            {
                error = NL_YES;
                NL_OUT;
            }
            /*  it ia an interior knot with full multiplicity if r was still  -1 */
            for ( i = 0; i < r; i++ )
                N_CopyCPt( Nw[i], &Bw[i] );

            for ( i = r; i <= p; i++ )
                N_CopyCPt( Pw[b - p + i], &Bw[i] );

            a = b;
            b++;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvArcLength */


/*******************************************************************//**


   DESCRIPTION:

     This  geometry  processing  routine finds  the point Q  on a NURBS
     curve closest to a given point P. The parameter corresponding to Q
     is also  returned. The point  P can be on the curve (inversion) or
     off  the  curve  (projection). The routine uses a global search to
     find a good start point for Newton iterations.  It  is designed to
     work efficiently for  multiple  calls  with  the  same  curve  but
     different points to project. A typical calling example is:

       NL_CURVE      cur;
       NL_POINT      P, Q;
       NL_REAL       top, toc, tin;
       NL_PARAMETER  u;
       NL_GCPTEMP    Tdata;
       NL_FLAG       Qret;
       NL_STACKS     S;
       ...
       (define cur, get P, top, toc, tin);
       ...
       N_CrvProjectionInitArrays(&Tdata);
       N_CrvClosestPtMultiple(&cur,P,tin,top,toc,&Tdata,&u,&Q,&Qret,&S);

     "Inversion" means P is considered to be on the curve to  within  a
     small tolerance (tin),  and the problem is to find the correspond-
     ing u-value. For "projection", the point P can  be  anywhere,  and 
     the solution Q is simply the point on the curve closest to P.

 
   ACCESS:
   
     cur   , input  ,  NURBS curve
     P     , input  ,  Point to be inverted/projected
     tin   , input  ,  If tin >= 0.0, then it is a bound for inversion,
                       and the problem is assumed to  be  an  inversion
                       problem. A solution Q is returned  only  if  the
                       distance from P to Q is less than  or  equal  to 
                       tin. If tin < 0.0, then the problem  is  assumed 
                       to be one of projection.  Note that tin does not
                       affect the convergence of the Newton process (as
                       do top and toc)
     top   , input  ,  Point coincidence tolerance
     toc   , input  ,  Zero cosine tolerance
     Tdata , in/out ,  Temporary data for speeding up multiple calls to
                       this function  with  different points, P.  Tdata 
                       can have 3 values/states:
                       1: = NULL.   Tdata  is  not  used.  This  option 
                          applies when N_CrvClosestPtMultiple is to  be  called  only
                          once with this curve.
                       2: != NULL but initialized to  NULL  (N_CrvProjectionInitArrays).
                          This applies for the first of multiple  calls
                          with the same curve.
                       3: != NULL and contents not NULL.  This  is  the
                          state after the first of multiple calls.
     u     , output ,  Parameter corresponding to Q, i.e. Q = C(u)
     Q     , output ,  Point on the curve closest to P
     Qret  , output ,  Flag:
                       NL_YES: Q and u are returned.  This flag may be set
                            even in the case that an error is  returned
                            (in which case Q is the best solution found
                            although convergence was not achieved).
                       NL_NO : Q and u are not returned.  This is returned
                            in some error cases, and also for inversion
                            if solutions are not within the tin bound.
     S     , input  ,  Memory stack for the contents of Tdata


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvClosestPtMultiple( NL_CURVE *cur, NL_POINT P, NL_REAL tin, NL_REAL top, NL_REAL toc, NL_GCPTEMP *Tdata, NL_PARAMETER *u, NL_POINT *Q, NL_FLAG *Qret, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO, *cand, error1, error2, error3 = NL_NO;

    NL_INDEX ii, jj, kk, i1, i2, k1, k2, idx1, idx2, kdx1, kdx2, kdx3, nks, nip, n, m, rat, j1, j2;

    NL_INTEGER itl_sav;

    NL_REAL dsq, *U, dp1, dp2, dp3, dg1, dg2, d1, d2, d3, xyz[3], du;

    NL_PARAMETER uu, u1, u2, u3, u11, u22, u33 = 0.0;

    NL_POINT Q1, Q2, Q3 = { 0,0,0 };

    NL_CPOINT *Pw;

    NL_DEGREE p;

    NL_GCPTEMP Ldata, *data;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL, *SS;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    *Qret = NL_NO;

    /* Handle different Tdata options */

    if( Tdata EQ NULL )
    {
        N_CrvProjectionInitArrays( &Ldata );
        data = &Ldata;
    }
    else
        data = Tdata;

    /* If necessary, allocate memory in "data' and load some contents */

    if( data->nks LT 0 )
    {
        N_CrvGetKnotVector( cur, &knt );
        N_BasisGetSpanCount( knt, p, &nks );

        data->nks = nks; /* number of knot spans */

        if( p EQ 1 )
            nip = 1;
        else
        {
            if( N_IsCrvRat( cur ) )
                rat = 1;
            else
                rat = 0;

            if( nks EQ 1 )
                nip = 21 + 4 * (p / 3) + 2 * rat;

            else if( nks LE 4 )
                nip = 13 + 2 *( p / 3 ) + 2 *rat;

            else if( nks LE 10 )
                nip = 7 + 2 *( p / 3 ) + 2 *rat;

            else if( nks LE 18 )
                nip = 5 + 2 *( p / 3 );

            else if( p LE 3 )
                nip = 3;

            else
                nip = 5;
        }

        data->nip = nip; /* number of points interior to each span */

        if( data EQ Tdata )
            SS = SG;     /* allocate memory */
        else
            SS = &SL;

        data->box = N_AllocReal3dArray( nks - 1, 1, 2, SS );
        data->cog = N_AllocPt1dArray( nks - 1, SS );
        data->pflg = N_AllocFlag1dArray( nks - 1, SS );
        data->us = N_AllocReal1dArray( nks - 1, SS );
        data->pts = N_AllocPt1dArray( nks * (nip + 1), SS );

        if( data->box EQ NULL OR data->cog EQ NULL OR data->pflg EQ NULL OR data->us EQ NULL OR data->pts EQ NULL )
            NL_QUIT;

        ii = p; /* now compute and load the curve data */
        jj = 0;
        k2 = p + 1;

        while( jj LT nks AND ii LT m )
        {
            data->us[jj] = U[ii];
            data->pflg[jj] = NL_NO;

            N_CPtToXYZ( Pw[ii - p], &d1, &d2, &d3 );
            data->box[jj][0][0] = data->box[jj][1][0] = d1;
            data->box[jj][0][1] = data->box[jj][1][1] = d2;
            data->box[jj][0][2] = data->box[jj][1][2] = d3;

            for ( kk = 1; kk <= p; kk++ )
            {
                N_CPtToXYZ( Pw[ii - p + kk], &xyz[0], &xyz[1], &xyz[2] );
                d1 += xyz[0];
                d2 += xyz[1];
                d3 += xyz[2];

                for ( k1 = 0; k1 < 3; k1++ )
                {
                    if( xyz[k1]LT data->box[jj][0][k1] )
                        data->box[jj][0][k1] = xyz[k1];

                    else if( xyz[k1]GT data->box[jj][1][k1] )
                        data->box[jj][1][k1] = xyz[k1];
                }
            }

            N_PtFromXYZ( d1 / k2, d2 / k2, d3 / k2, &( data->cog[jj] ) );
            jj += 1;

            ii += 1;

            while( ii LT m )
                if( U[ii]NEQ U[ii + 1] )
                    break;
                else
                    ii++;
        }
    }
    else
    {
        nks = data->nks;
        nip = data->nip;
    }

    /* Allocate flags indicating candidate spans */

    cand = N_AllocFlag1dArray( nks - 1, &SL );

    if( cand EQ NULL )
        NL_QUIT;

    for ( ii = 0; ii < nks; ii++ )
        cand[ii] = NL_YES;

    /* Now do some initial culling, and compute two spans */
    /* likely to be closest to the point P */

    idx1 = idx2 = -3;
    dg1 = dg2 = NL_BIGD;
    N_PtToXYZ( P, &xyz[0], &xyz[1], &xyz[2] );

    for ( ii = 0; ii < nks; ii++ )
    {
        if( tin GE 0.0 )
        {
            for ( jj = 0; jj < 3; jj++ )
                if( xyz[jj]LT data->box[ii][0][jj] - tin OR xyz[jj]GT data->box[ii][1][jj] + tin )
                {
                    cand[ii] = NL_NO; /* P cannot be within tin distance */
                    break;            /* of this span                    */
                }

            if( cand[ii]EQ NL_NO )
                continue;
        }

        N_DistSqPtPt( P, data->cog[ii], &dsq );

        if( dsq LT dg2 )
        {
            if( dsq LT dg1 )
            {
                dg2 = dg1;
                idx2 = idx1;
                dg1 = dsq;
                idx1 = ii;
            }
            else
            {
                dg2 = dsq;
                idx2 = ii;
            }
        }
    }

    if( idx1 LT 0 )
        NL_OUT; /* no solution to the inversion */

    /* Now search the points in the two "closest" spans found */
    /* above to find the closest two points                   */

    dp1 = dp2 = NL_BIGD;
    kdx1 = kdx2 = -3;

    for ( ii = 0; ii < 2; ii++ )
    {
        if( ii EQ 0 ) /* select span */
        {
            kk = idx1;
            k1 = idx1 * (nip + 1); /* search points k1 to k2 in each span */
            k2 = k1 + nip + 1;
        }
        else
        {
            if( idx2 LT 0 )
                continue;
            kk = idx2;
            k1 = idx2 * (nip + 1);
            k2 = k1 + nip + 1;

            if( idx1 EQ idx2 - 1 )
                k1 += 1; /* don't repeat points */

            else         /* from span idx1      */
            if( idx1 EQ idx2 + 1 )
                k2 -= 1;
        }

        if( data->pflg[kk]EQ NL_NO ) /* Points not evaluated yet. */
        {                            /* Must get them first.      */
            i1 = j1 = kk * (nip + 1);
            i2 = j2 = i1 + nip + 1;

            if( kk GT 0 )
                if( data->pflg[kk - 1]EQ NL_YES )
                    j1 += 1;

            if( kk LT nks - 1 )
                if( data->pflg[kk + 1]EQ NL_YES )
                    j2 -= 1;

            u1 = data->us[kk];

            if( kk LT nks - 1 )
                u2 = data->us[kk + 1];
            else
                u2 = U[m];
            du = (u2 - u1) / ((NL_REAL)nip + (NL_REAL)1);

            for ( jj = j1; jj <= j2; jj++ )
            {
                if( jj EQ i2 )
                    uu = u2;
                else
                    uu = u1 + ((NL_REAL)jj - (NL_REAL)i1) * du;
                error = N_CrvEval( cur, uu, NL_LEFT, &( data->pts[jj] ) );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            data->pflg[kk] = NL_YES;
        }

        for ( jj = k1; jj <= k2; jj++ ) /* now find closest 2 points */
        {
            N_DistSqPtPt( P, data->pts[jj], &dsq );

            if( dsq LT dp2 )
            {
                if( dsq LT dp1 )
                {
                    dp2 = dp1;
                    kdx2 = kdx1;
                    dp1 = dsq;
                    kdx1 = jj;
                }
                else
                {
                    dp2 = dsq;
                    kdx2 = jj;
                }
            }
        }

        cand[kk] = NL_NO; /* don't visit this span again */
    }

    /* Now look at the other spans to find the closest three points. */
    /* The third point is not in the original two spans (idx1,idx2). */

    dp3 = NL_BIGD;
    kdx3 = -3;

    for ( ii = 0; ii < nks; ii++ )
    {
        if( cand[ii]EQ NL_NO )
            continue;

        if( tin LT 0.0 )
        {
            d1 = sqrt( dp1 );

            for ( jj = 0; jj < 3; jj++ )
                if( xyz[jj]LT data->box[ii][0][jj] - d1 OR xyz[jj]GT data->box[ii][1][jj] + d1 )
                {
                    cand[ii] = NL_NO; /* This span cannot be the closest */
                    break;
                }

            if( cand[ii]EQ NL_NO )
                continue;
        }

        k1 = ii * (nip + 1);
        k2 = k1 + nip + 1;

        if( data->pflg[ii]EQ NL_NO ) /* Points not evaluated yet. */
        {                            /* Must get them first.      */
            i1 = j1 = k1;
            i2 = j2 = k2;

            if( ii GT 0 )
                if( data->pflg[ii - 1]EQ NL_YES )
                    j1 += 1;

            if( ii LT nks - 1 )
                if( data->pflg[ii + 1]EQ NL_YES )
                    j2 -= 1;

            u1 = data->us[ii];

            if( ii LT nks - 1 )
                u2 = data->us[ii + 1];
            else
                u2 = U[m];
            du = (u2 - u1) / ((NL_REAL)nip + (NL_REAL)1);

            for ( jj = j1; jj <= j2; jj++ )
            {
                if( jj EQ i2 )
                    uu = u2;
                else
                    uu = u1 + ((NL_REAL)jj - (NL_REAL)i1) * du;
                error = N_CrvEval( cur, uu, NL_LEFT, &( data->pts[jj] ) );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            data->pflg[ii] = NL_YES;
        }

        if( ii GT 0 )
            if( cand[ii - 1]EQ NL_NO )
                k1 += 1;

        if( ii LT nks - 1 )
            k2 -= 1;

        for ( jj = k1; jj <= k2; jj++ ) /* now find closest 3 points */
        {
            N_DistSqPtPt( P, data->pts[jj], &dsq );

            if( dsq LT dp3 )
            {
                if( dsq LT dp2 )
                {
                    if( dsq LT dp1 )
                    {
                        dp3 = dp2;
                        kdx3 = kdx2;
                        dp2 = dp1;
                        kdx2 = kdx1;
                        dp1 = dsq;
                        kdx1 = jj;
                    }
                    else
                    {
                        dp3 = dp2;
                        kdx3 = kdx2;
                        dp2 = dsq;
                        kdx2 = jj;
                    }
                }
                else
                {
                    dp3 = dsq;
                    kdx3 = jj;
                }
            }
        }
    }

    /* kdx1 and kdx2 are indices of the closest two points. */
    /* Use 2 Newton iterations to decide which to keep.     */

    jj = nip + 1; /* find start u-values */
    u1 = u2 = uu = U[0] - 10.0;
    u3 = uu;

    for ( ii = 0; ii < nks; ii++ )
    {
        if( kdx1 LE jj AND u1 EQ uu )
        { /* kdx1 is in the ii-th span */
            if( ii LT nks - 1 )
                u22 = data->us[ii + 1];
            else
                u22 = U[m];

            if( kdx1 EQ jj )
                u1 = u22;
            else
            {
                u11 = data->us[ii];
                du = (u22 - u11) / ((NL_REAL)nip + (NL_REAL)1);
                u1 = u11 + ((NL_REAL)kdx1 - (NL_REAL)jj + (NL_REAL)nip + (NL_REAL)1) * du;
            }
        }

        if( kdx2 LE jj AND u2 EQ uu )
        { /* kdx2 is in the ii-th span */
            if( ii LT nks - 1 )
                u22 = data->us[ii + 1];
            else
                u22 = U[m];

            if( kdx2 EQ jj )
                u2 = u22;
            else
            {
                u11 = data->us[ii];
                du = (u22 - u11) / ((NL_REAL)nip + (NL_REAL)1);
                u2 = u11 + ((NL_REAL)kdx2 - (NL_REAL)jj + (NL_REAL)nip + (NL_REAL)1) * du;
            }
        }

        if( kdx3 LE jj AND u3 EQ uu AND kdx3 GT 0 )
        { /* kdx3 is in the ii-th span */
            if( ii LT nks - 1 )
                u22 = data->us[ii + 1];
            else
                u22 = U[m];

            if( kdx3 EQ jj )
                u3 = u22;
            else
            {
                u11 = data->us[ii];
                du = (u22 - u11) / ((NL_REAL)nip + (NL_REAL)1);
                u3 = u11 + ((NL_REAL)kdx3 - (NL_REAL)jj + (NL_REAL)nip + (NL_REAL)1) * du;
            }
        }

        if( u1 GT uu AND u2 GT uu )
            if( u3 GT uu OR kdx3 LT 0 )
                break;
        jj += (nip + 1);
    }

    itl_sav = NL_ITLIM;
    NL_ITLIM = 5;
    error1 = N_CrvClosestPt( cur, P, u1, top, toc, &u11, &Q1 );
    error2 = N_CrvClosestPt( cur, P, u2, top, toc, &u22, &Q2 );
    d3 = NL_BIGD;

    if( kdx3 GT 0 )
    {
        error3 = N_CrvClosestPt( cur, P, u3, top, toc, &u33, &Q3 );
        N_DistSqPtPt( P, Q3, &d3 );
    }
    NL_ITLIM = itl_sav;

    N_DistSqPtPt( P, Q1, &d1 ); /* take the best one */
    N_DistSqPtPt( P, Q2, &d2 );

    if( d3 LT d1 AND d3 LT d2 )
    {
        error = error3;
        *u = u33;
        N_CopyPt( Q3, Q );
    }
    else if( d1 LT d2 )
    {
        error = error1;
        *u = u11;
        N_CopyPt( Q1, Q );
    }
    else
    {
        error = error2;
        *u = u22;
        N_CopyPt( Q2, Q );
    }

    if( error EQ NL_YES ) /* let it converge completely */
        error = N_CrvClosestPt( cur, P, *u, top, toc, u, Q );

    /* Set error and Qret correctly */

    if( tin GE 0.0 )
    {
        N_DistPtPt( P, *Q, &d1 );

        if( d1 LE tin )
        {
            *Qret = NL_YES;
            error = NL_NO;
        }
    }
    else
        *Qret = NL_YES;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvClosestPtMultiple */


/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine computes a piecewise linear approximation to a Nurbs
     curve. An absolute or relative chordal deviation tolerance can be
     specified. Points on the curve and/or their corresponding  param-
     eter values can be returned. A typical calling example is:
 
       NL_CURVE   cur;
       NL_REAL    tol;
       NL_REAL    *pars;
       NL_POINT   *pts;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (define cur and choose tol);
       ...
       N_ApproxCrvWithPolyline(&cur,tol,NL_ABSOLUTE,NL_BOTH,&pts,&pars,&n,&S);
 
 
   ACCESS:
   
     cur  , input  ,  NURBS curve
     tol  , input  ,  Chordal deviation tolerance.  The curve does not
                      deviate from the polygonal approximation by more
                      than tol.  If tflg = NL_RELATIVE, tol is multiplied
                      by the arc length of the curve.
     tflg , input  ,  Tolerance flag:
                       NL_ABSOLUTE : use tol as is
                       NL_RELATIVE : multiply tol by arc length of cur
     rflg , input  ,  Return flag:
                       NL_POINTS     : return just the points of the lin-
                                       ear approximation  (points are  on 
                                       the curve)
                       NL_PARAMETERS : return just the  curve  parameters
                                       corresponding to the points of the
                                       linear approximation
                       NL_BOTH       : return both points and parameters
     pts  , output ,  Points of  the  piecewise  linear  approximation
                      (lying on the curve).  Memory  is  allocated  in
                      this routine. pts should be NULL on entry.
     pars , output ,  Parameters corresponding to the points of the
                      piecewise linear approximation. Memory is alloc-
                      in this routine. pars should be NULL on entry.
     n    , output ,  High index of points and/or parameters
     S    , output ,  Memory stack for pts and/or pars
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_ApproxCrvWithPolyline /* eff: build polyline (pts on crv) to chord height tol with  */
                                /*      at least a point at every original knot param value   */ 
 ( NL_CURVE  * cur,             /* in : target curve                                          */
   NL_REAL     tol,             /* in : max chord height                                      */
   NL_FLAG     tflg,            /* in : NL_ABSOLUTE = use tol as is                           */
                                /*      NL_RELATIVE = multiply tol by arc length of cur       */
   NL_FLAG     rflg,            /* in : NL_POINTS     = output points only                    */
                                /*      NL_PARAMETERS = output params only                    */
                                /*      NL_BOTH       = output both                           */
   NL_POINT ** pts,             /* out: output polyline points                                */
   NL_REAL  ** pars,            /* out: output polyline params                                */
   NL_INDEX  * n,               /* out: highest index in output arrays                        */
   NL_STACKS * S )              /* in : new object stack                                      */
{                                                                                        
    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, nn, top, s_size, p_size;

    NL_CURVE ** bezs;

    NL_DEGREE p;

    NL_CPOINT *Pw, ** cp_stack;

    NL_POINT *pnts = NULL, *lpts, P1, P2, PP, QQ, last_pt;

    NL_VECTOR V12, V1P;

    NL_REAL tolsq, *parms = NULL, *lpars, *U, dd, ** pm_stack = NULL, dot, length;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetDegree( cur, &p );

    /* Handle p = 1 as special case */

    if( p EQ 1 )
    {
        N_CrvGetCPtsDegreeAndKnots( cur,  /* in : target curve                                                    */
                                   &nn,   /* in : Highest index in control point array                            */
                                   &Pw,   /* out: array of control points                                         */
                                   &p,    /* out: curve degree                                                    */
                                   &ii,   /* out: Highest index in KnotArray                                      */
                                   &U );  /* out: array of knots (multiple knots are represented multiple times ) */

        /* when asked - allocate pnts array memory */
        if( rflg NEQ NL_PARAMETERS)
        {
            pnts = N_AllocPt1dArray( nn, S );

            if( pnts EQ NULL )
                NL_QUIT;
            *pts = pnts;
        }

        /* when asked - allocate pars array memory */
        if( rflg NEQ NL_POINTS)
        {
            parms = N_AllocReal1dArray( nn, S );

            if( parms EQ NULL )
                NL_QUIT;
            *pars = parms;
        }

        /* number of control points */
        *n = nn;

        /* init pnts and parms with control points and associated param values */
        for ( ii = 0; ii <= nn; ii++ )
        {
            if( rflg NEQ NL_PARAMETERS )
                N_CPtToPtEuclid( Pw[ii], &pnts[ii] );

            if( rflg NEQ NL_POINTS )
                parms[ii] = U[ii + 1];
        }

        NL_OUT;
    }

    /* Decompose curve into Bezier segments */
    /* - every cur knto span interval is conveted into it's own Bezier curve */ 
    /* - every Bezier start and end control point is a point on cur */
    /* - any curve point represented by a fully multiple knot will be represented as a bezier end point */
    error = N_CrvDecomposeBez( cur,   /* in : target curve                                 */
                              &bezs,  /* out: A Bezier curve for every input span interval */
                              &nn,    /* out: Highest index in Bezier curve array          */
                              &SL );  /* in : stack for new objects                        */

    if( error EQ NL_YES )
        NL_OUT;

    /* Adjust tolerance if necessary */

    if( tflg EQ NL_RELATIVE )
    {
        length = 0.0;

        for ( ii = 0; ii <= nn; ii++ )
        {
            N_CrvGetCPtsDegreeAndKnots( bezs[ii], /* in : target curve                                                    */ 
                                       &jj,       /* in : Highest index in control point array                            */
                                       &Pw,       /* out: array of control points                                         */
                                       &p,        /* out: curve degree                                                    */
                                       &kk,       /* out: Highest index in KnotArray                                      */
                                       &U );      /* out: array of knots (multiple knots are represented multiple times ) */
            error = N_BezCalcLength( Pw, p, 0.01, &dd );

            if( error EQ NL_YES )
                NL_OUT;
            length += dd;
        }

        tol *= length;

    } /* end need to compute RELATIVE tolerance check */ 

    /* Allocate and initialize the stacks of Bezier segments */

    if( nn LT 6 )        /* nn = number of original control points */
        s_size = 100;

    else if( nn LT 15 )
        s_size = 10 *nn;

    else
        s_size = 5 * nn;

    cp_stack = N_AllocCPt2dArray( s_size, p, &SL );

    if( cp_stack EQ NULL )
        NL_QUIT;

    if( rflg NEQ NL_POINTS )
    {
        pm_stack = N_AllocReal2dArray( s_size, 1, &SL );

        if( pm_stack EQ NULL )
            NL_QUIT;
    }

    top = -1;

    /* for every control point */ 
    for ( ii = nn; ii >= 0; ii-- )
    {
        N_CrvGetCPtsDegreeAndKnots( bezs[ii], /* in : target curve                                                    */
                                   &jj,       /* in : Highest index in control point array                            */
                                   &Pw,       /* out: array of control points                                         */
                                   &p,        /* out: curve degree                                                    */
                                   &kk,       /* out: Highest index in KnotArray                                      */
                                   &U );      /* out: array of knots (multiple knots are represented multiple times ) */
        top += 1;

        if( rflg NEQ NL_POINTS )
        {
            pm_stack[top][0] = U[0];
            pm_stack[top][1] = U[kk];
        }

        for ( jj = 0; jj <= p; jj++ )
            N_CopyCPt( Pw[jj], &cp_stack[top][jj] );
    }

    /* Allocate and initialize points and parms arrays */

    p_size = 250;

    if( rflg NEQ NL_PARAMETERS )
    {
        pnts = N_AllocPt1dArray( p_size, &SL );

        if( pnts EQ NULL )
            NL_QUIT;
        N_CPtToPtEuclid( cp_stack[top][0], &pnts[0] );
    }

    if( rflg NEQ NL_POINTS )
    {
        parms = N_AllocReal1dArray( p_size, &SL );

        if( parms EQ NULL )
            NL_QUIT;
        parms[0] = pm_stack[top][0];
    }
    N_CPtToPtEuclid( cp_stack[top][0], &last_pt );

    nn = 0;

    /* Now recursively subdivide the segments until the stack is empty */

    tolsq = tol * tol;

    while( top GE 0 )
    {
        /* check if this segment has converged */

        N_CPtToPtEuclid( cp_stack[top][0], &P1 );
        N_CPtToPtEuclid( cp_stack[top][p], &P2 );
        N_VectorDir( P1, P2, &V12 );
        error = N_VectorNormalize( V12, &V12, &dd );

        if( error EQ NL_NO )
        {
            for ( ii = 1; ii < p; ii++ )
            {
                N_CPtToPtEuclid( cp_stack[top][ii], &PP );
                N_VectorDir( P1, PP, &V1P );
                N_VectorDot( V12, V1P, &dot );
                N_VectorPtAlongVector( P1, dot, V12, &QQ );
                N_DistSqPtPt( PP, QQ, &dd );

                if( dd GT tolsq )
                    break;
            }
        }
        else
        { /* segment is either closed or degenerate */
            N_CPtToPtEuclid( cp_stack[top][(p + 1) / 2], &PP );
            N_DistSqPtPt( PP, P1, &dd );

            if( dd LT tolsq ) {
                ii = p;  /* degenerate */
                if( error ) error = 0;
            }
            else
                ii = -1; /* closed */
        }

        if( ii GE p )
        {                      /* this segment has converged */
            if( nn GE p_size ) /* check if we need more memory */
            {
                p_size += 150;

                if( rflg NEQ NL_PARAMETERS )
                {
                    error = N_Realloc1dPtArray( &pnts, nn, p_size, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                if( rflg NEQ NL_POINTS )
                {
                    error = N_Realloc1dRealArray( &parms, nn, p_size, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
            /* load point and parameter */

            N_DistPtPt( P2, last_pt, &dd );

            if( top EQ 0 )
            {
                if( nn EQ 0 )
                    dd = tol;
                else if( dd LE 0.01 *tol )
                {
                    nn -= 1;
                    dd = tol;
                }
            }

            if( dd GT 0.01 *tol )
            {
                nn += 1;

                if( rflg NEQ NL_PARAMETERS )
                    N_CopyPt( P2, &pnts[nn] );

                if( rflg NEQ NL_POINTS )
                    parms[nn] = pm_stack[top][1];
                N_CopyPt( P2, &last_pt );
            }

            top -= 1; /* pop stack */
        }
        else
        {                       /* must split segment */
            if( top GE s_size ) /* check if we need more memory */
            {
                s_size += 75;
                error = N_Realloc2dCPtArray( &cp_stack, top, p, s_size, p, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                if( rflg NEQ NL_POINTS )
                {
                    error = N_Realloc2dRealArray( &pm_stack, top, 1, s_size, 1, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }

            N_CopyCPt( cp_stack[top][0], &cp_stack[top + 1][0] ); /* split */

            for ( kk = 1; kk <= p; kk++ )
            {
                for ( ii = 0; ii <= p - kk; ii++ )
                    N_Combine2CPts( 0.5, cp_stack[top][ii], 0.5, cp_stack[top][ii + 1], &cp_stack[top][ii] );
                N_CopyCPt( cp_stack[top][0], &cp_stack[top + 1][kk] );
            }

            if( rflg NEQ NL_POINTS )
            {
                pm_stack[top + 1][0] = pm_stack[top][0];
                pm_stack[top][0] = 0.5 *( pm_stack[top][0] + pm_stack[top][1] );
                pm_stack[top + 1][1] = pm_stack[top][0];
            }

            top += 1; /* one more on stack (left on top) */
        }
    }

    /* Load output */

    if( rflg NEQ NL_PARAMETERS )
    {
        lpts = N_AllocPt1dArray( nn, S );

        if( lpts EQ NULL )
            NL_QUIT;

        for ( ii = 0; ii <= nn; ii++ )
            N_CopyPt( pnts[ii], &lpts[ii] );
        *pts = lpts;
    }

    if( rflg NEQ NL_POINTS )
    {
        lpars = N_AllocReal1dArray( nn, S );

        if( lpars EQ NULL )
            NL_QUIT;

        for ( ii = 0; ii <= nn; ii++ )
            lpars[ii] = parms[ii];
        *pars = lpars;
    }

    *n = nn;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_ApproxCrvWithPolyline */

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine computes a point on a curve that 
     is at given percentage of the total arc  length computed from the 
     start point. A typical calling example is:

       NL_CURVE  cur;
       NL_REAL   per, atl, ctl, len, up;
       NL_POINT  P;
       ...
       (define cur, get per, atl and ctl);
       ...
       N_CrvPercentageAlongPt(&cur,per,atl,ctl,NL_RELATIVE,&len,&up,&P);

     The tolerance atl can be absolute or relative. If it is relative,
     it is scaled by the largest extent of the curve.


   ACCESS:
   
     cur , input  ,  NURBS curve
     per , input  ,  Percentage of arc length; 0.0 <= per <= 1.0!
     atl , input  ,  Tolerance for arc length approximation
     ctl , input  ,  Tolerance  for  curve  type   determination  (see f
                     N_CrvGetType for details)
     flg , input  ,  Flag:
                       NL_ABSOLUTE: atl is an absolute tolerance
                       NL_RELATIVE: atl is a relative tolerance, i.e., it
                                 is scaled by the  extent of the curve
     len , in/out ,  Length of curve (may be passed in, if known):
                       len<0.0: length is  not known, must be computed
                                internally
                       len>0.0: it is assumed  that len is the correct 
                                arc length
     up  , output ,  Parameter at which P is computed
     P   , output ,  Point at  per*arc_length  distance  away from the
                     start point


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_CrvPercentageAlongPt( NL_CURVE *cur, NL_REAL per, NL_REAL atl, NL_REAL ctl, NL_FLAG flg, NL_REAL *len, NL_REAL *up, NL_POINT *P )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPercentageAlongPt");

    NL_FLAG fin, typ, error = NL_NO, rev = NL_NO;

    NL_INDEX i, j, k, n, m, mlt, r, s, a, b, nsp, save;

    NL_DEGREE p;

    NL_REAL *U, *alfs, *omas, d, ltol, num, eps, lbz, lp, ld, ll, al, lr, ls, ul, um, ur, us, tl, tr, epl;

    NL_KNOTVECTOR *knt;

    NL_POINT AA;

    NL_VECTOR BB, XX, YY;

    NL_CPOINT *Pw, *Bw, *Nw;

    NL_CURVE curA, curB;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input */

    if( per LT 0.0 OR per GT 1.0 )
        NL_ERROR( NL_INP_ERR );

    /* Adjust tolerance */

    switch( flg )
    {
        case NL_RELATIVE:

            error = N_CrvGetBBoxMaxDiagDist( cur, &d );

            if( error EQ NL_YES )
                NL_OUT;

            ltol = d * atl;
            break;

        case NL_ABSOLUTE:

            ltol = atl;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Get total length if not passed in */

    N_CrvGetKnots( cur, &m, &U );
    N_CrvGetDegree( cur, &p );

    if( *len LT 0.0 )
    {
        error = N_CrvArcLength( cur, U[0], U[m], atl, flg, len );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Do special cases */

    error = N_CrvGetType( cur, ctl, &AA, &BB, &XX, &YY, &d, &al, &typ );

    if( error EQ NL_YES )
        NL_OUT;

    if( typ EQ NL_NLINE )
    {
        if( p EQ 1 )
        {
            *up = U[0] + per * (U[m] - U[0]);

            error = N_CrvEval( cur, *up, NL_LEFT, P );

            if( error EQ NL_YES )
                NL_OUT;

            NL_OUT;
        }
    }

    if( typ EQ NL_NCIRCLE )
    {
        ll = (al / 180.0) * NL_PI * d;
        lp = per * ll;
        al = lp / d;
        ul = d * cos( al );
        ur = d * sin( al );

        N_TranslateSum2Pts( AA, ul, XX, ur, YY, P );

        um = U[0] + (lp / ll) * (U[m] - U[0]);

        error = N_CrvClosestPt( cur, *P, um, NL_MIN( NL_MTOL, ltol ), NL_MIN( NL_MTOL, ltol ), up, &AA );

        if( error NEQ NL_CON_ERR )
            NL_OUT;
    }

    /* Reverse curve if necessary */

    if( per GT 0.5 )
    {
        per = 1.0 - per;
        rev = NL_YES;

        N_CrvInitArrays( &curA );
        error = N_CrvReverse( cur, &curA, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get local notation and prepare for decomposition */

    if( rev EQ NL_YES )
    {
        N_CrvGetCPtsDegreeAndKnots( &curA, &n, &Pw, &p, &m, &U );
        N_CrvGetKnotVector( &curA, &knt );
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
        N_CrvGetKnotVector( cur, &knt );
    }

    N_BasisGetSpanCount( knt, p, &nsp );

    eps = ltol / nsp;
    ll = 0.0;

    /* Get local memory */

    Bw = N_AllocCPt1dArray( p, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt1dArray( p, &SL );

    if( Nw EQ NULL )
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
    lp = per * (*len);

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Pw[i], &Bw[i] );

    /* Decompose curve and compute length of each Bezier piece */

    while( b LT m )
    {
        i = b;

        while( b LT m AND U[b]EQ U[b + 1] )
            b++;

        mlt = b - i + 1;
        r = p - mlt;

        if( r GT 0 )
        {
            num = U[b] - U[a];

            for ( k = p; k > mlt; k-- )
            {
                alfs[k - mlt - 1] = num / (U[a + k] - U[a]);
                omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
            }

            for ( j = 1; j <= r; j++ )
            {
                save = r - j;
                s = mlt + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfs[k - s], Bw[k], omas[k - s], Bw[k - 1], &Bw[k] );
                }
                N_CopyCPt( Bw[p], &Nw[save] );
            }
        }

        /* Compute length of Bezier segment */

        error = N_BezCalcLength( Bw, p, eps, &lbz );

        if( error EQ NL_YES )
            NL_OUT;

        if( (ll + lbz)LT lp )
            ll += lbz;
        else
            break;

        /* Initialize for next pass through */

        if( b LT m )
        {
            for ( i = 0; i < r; i++ )
                N_CopyCPt( Nw[i], &Bw[i] );

            for ( i = r; i <= p; i++ )
                N_CopyCPt( Pw[b - p + i], &Bw[i] );

            a = b;
            b++;
        }
    }

    /* Compute length of leftover piece */

    ld = lp - ll;
    ls = 2.0 *ld;
    epl = 2.0 *ltol;
    tl = 0.0;
    tr = 1.0;
    lr = 0.0;
    fin = NL_NO;
    ul = ur = 1.0;

    error = ST_bezlen( Bw, p, ls, &epl, &lr, tl, tr, &ul, &ur, &fin );

    if( error EQ NL_YES )
        NL_OUT;

    lr = 0.5 *lr;

    /* Extract remaining piece */

    ul = U[a] + ul * (U[b] - U[a]);
    ur = U[a] + ur * (U[b] - U[a]);
    us = ul;
    ld = ld - lr;

    *up = ul;

    if( ld GT ltol )
    {
        N_CrvInitArrays( &curB );

        if( rev EQ NL_YES )
        {
            error = N_CrvExtractCrvSeg( &curA, ul, ur, &curB, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            error = N_CrvExtractCrvSeg( cur, ul, ur, &curB, &SL, &SL );

            if( error EQ NL_YES )
                NL_OUT;
        }

        do
        {
            um = 0.5 *( ul + ur );

            error = N_CrvArcLength( &curB, us, um, ltol, flg, &ll );

            if( error EQ NL_YES )
                NL_OUT;

            if( fabs( ll - ld )LT ltol )
                break;

            else if( ll LT ld )
                ul = um;

            else if( ll GT ld )
                ur = um;
        } while ( 1 );

        *up = um;
    }

    if( rev EQ NL_YES )
        *up = U[m] + U[0]-(*up);

    error = N_CrvEval( cur, *up, NL_LEFT, P );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvPercentageAlongPt */


/*******************************************************************//**


   DESCRIPTION:

     Given a set of points and a curve, this geometry processing routine  
     projects each of them onto the curve and returns the projections as
     well as  the corresponding  parameters. The  projection is  done by 
     decomposing the curve into a polygon and projecting the points onto
     the closest legs of the polygon. A typical calling example is:


       NL_CURVE   cur;
       NL_POINT   *P, *T;
       NL_REAL    *u, tol;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cur, get P and tol);
       ...
       N_CrvProjectPts(&cur,P,k,tol,&T,&u,&SG);

     THIS ROUTINE  IS  DESIGNED TO HANDLE NL_POINTS  LYING ON THE NL_CURVE! IT
     WORKS WITH NL_POINTS THAT ARE OFF  THE NL_CURVE, HOWEVER, THE ACCURACY OF 
     THE RETURNED  NL_PARAMETERS MAY NOT BE  ACCEPTABLE  (SEE N_CRVCLOSESTPTMULTIPLE AS A
     MORE GENERAL ALTERNATIVE). MEMORY FOR T AND u  IS  ALLOCATED INSIDE 
     THE ROUTINE.


   ACCESS:
   
     cur  , input  ,  NURBS curve
     P    , input  ,  Points to be projected
     k    , input  ,  Highest index in array P
     tol  , input  ,  Curve straightness tolerance;  MUST BE A  NL_RELATIVE 
                      TOLERANCE!! 0.1%, I.E. 0.001 IS SUGGESTED.
     T    , output ,  Projection of points P
     u    , output ,  Parameters corresponding to T[i]
     SG   , input  ,  T's and u's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvProjectPts( NL_CURVE *cur, NL_POINT *P, NL_INDEX k, NL_REAL tol, NL_POINT ** T, NL_REAL ** u, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvProjectPts");
    NL_FLAG error = NL_NO;
    NL_INDEX i, j, js, m;
    NL_REAL *t, *ul, alf;
    NL_POINT *Tl, *Q, R;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Decompose curve */
    error = N_ApproxCrvWithPolyline( cur, tol, NL_RELATIVE, NL_BOTH, &Q, &t, &m, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Project points and find parameters */
    ul = N_AllocReal1dArray( k, SG );

    if( ul EQ NULL )
        NL_QUIT;

    Tl = N_AllocPt1dArray( k, SG );

    if( Tl EQ NULL )
        NL_QUIT;

    js = 0;

    for ( i = 0; i <= k; i++ )
    {
        ul[i] = NL_BIGD;

        for ( j = js; j < m; j++ )
        {
            error = N_ProjectPtLineParam( P[i], Q[j], Q[j + 1], &R, &alf );

            if( error EQ NL_YES )
                NL_OUT;

            if( alf GE 0.0 AND alf LE 1.0 )
            {
                ul[i] = t[j] + alf * (t[j + 1] - t[j]);

                N_VectorCopy( R, &Tl[i] );

                js = j;
                break;
            }
        }

        if( ul[i]EQ NL_BIGD )
            NL_ERROR( NL_NUM_ERR );
    }

    *T = Tl;
    *u = ul;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvProjectPts */

/**********************************************************************/
/* ST_BEZLEN: Compute length till exceeds a given limit               */
/**********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG ST_bezlen( NL_CPOINT *Pw, NL_DEGREE p, NL_REAL ls, NL_REAL *eps, NL_REAL *len, NL_REAL tl, NL_REAL tr, NL_REAL *ul, NL_REAL *ur, NL_FLAG *fin )
{
    NL_FLAG error = NL_NO;
    NL_REAL del, epl, epr, UB, LB, tm;
    NL_CPOINT *Qw, *Rw;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get upper and lower bounds */
    if( *fin EQ NL_NO )
    {
        N_DistCPolygon( Pw, p, &UB );
        N_DistCptCpt( Pw[0], Pw[p], &LB );

        /* Compute total length or recurse */

        del = fabs( UB - LB );

        if( del LE *eps )
        {
            if( (*len + UB + LB)GT ls )
            {
                *fin = NL_YES;
                *ul = tl;
                *ur = tr;
            }
            else
            {
                *len += UB + LB;
                *eps = del;
            }
        }
        else
        {
            Qw = N_AllocCPt1dArray( p, &S );

            if( Qw EQ NULL )
                NL_QUIT;

            Rw = N_AllocCPt1dArray( p, &S );

            if( Rw EQ NULL )
                NL_QUIT;

            error = N_BezSplit( Pw, p, 0.5, Qw, Rw );

            if( error EQ NL_YES )
                NL_OUT;

            epl = *eps / 2.0;
            tm = 0.5 *( tl + tr );

            error = ST_bezlen( Qw, p, ls, &epl, len, tl, tm, ul, ur, fin );

            if( error EQ NL_YES )
                NL_OUT;

            if( *fin EQ NL_NO )
            {
                epr = *eps - epl;

                error = ST_bezlen( Rw, p, ls, &epr, len, tm, tr, ul, ur, fin );

                if( error EQ NL_YES )
                    NL_OUT;

                *eps = epl + epr;
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
}
