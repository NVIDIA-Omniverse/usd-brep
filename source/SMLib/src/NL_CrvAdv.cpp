// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvAdv.c : Advanced Function Definitions that act on NL_CURVE objects */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_FuncsAdv.h>    /* Advanced NL_CFUN, NL_CVALUE, NL_SFUN, NL_SVALUE, NL_VFUN, and NL_VVALUE functions */

#include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */



NL_PRIVATE NL_REAL NOREM = 1.0e+25;
NL_PRIVATE NL_REAL KN_TOL = 1.0e-6;

/* ----------------------- */
/* File Local declarations */
/* ----------------------- */
static NL_FLAG ST_curxcdr(NL_CURVE*, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_CURVE*, NL_STACKS*, NL_STACKS*, int);
static NL_FLAG ST_curxcrr(NL_CURVE*, NL_REAL, NL_FLAG, NL_FLAG, NL_CURVE*, NL_STACKS*, NL_STACKS*, int);


NL_PRIVATE NL_REAL cto = 1.0e-05;
#if NLIB_UNUSED

NL_PRIVATE NL_REAL eps = 1.0e-03;

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes a  point on a  NURBS  curve in  homogeneous 
     space  by  evaluating  all  non-vanishing   basis   functions  and 
     multiplying  them by  appropriate  control  points.  Discontinuous  
     curves can also be handled by passing a NL_LEFT/NL_RIGHT flag. A typical 
     calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_CPOINT     Cw;
       ...
       (define cur and get u);
       ...
       N_CrvEvalPt(&cur,u,NL_LEFT,&Cw);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     Cw  , output ,  Point on the curve in homogeneous space


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalPt( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_CPOINT *Cw )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvEvalPt");

    NL_FLAG error = NL_NO;

    NL_INDEX i, spn;

    NL_DEGREE p;

    NL_REAL *N;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &i, &Pw, &p, &i, &N );
    N_CrvGetKnotVector( cur, &knt );

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines */

    N = N_AllocReal1dArray( p, &S );

    if( N EQ NULL )
        NL_QUIT;

    error = N_BasisEval( knt, p, u, flg, N, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the point on the curve */

    N_CopyCPt( NL_CZERO, Cw );

    for ( i = 0; i <= p; i++ )
    {
        N_VectorBlendCPt( N[i], Pw[spn - p + i], Cw );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvEvalPt */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a NURBS curve in homogeneous
     space by evaluating all non-vanishing  basis functions and  their 
     derivatives, and multiplying them by  appropriate control points.  
     Discontinuous curves can also be handled by passing a  NL_LEFT/NL_RIGHT 
     flag. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_INDEX      der;
       NL_CPOINT     *Dw;
       ...
       (define cur, get u and der, and allocate memory for Dw);
       ...
       N_CrvDerivsAtKnot(&cur,u,NL_LEFT,der,Dw);


   ACCESS:
   
     cur , input  ,  NURBS curve
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES REQUIRED)
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES REQUIRED)
     der , input  ,  Highest derivative required
     Dw  , output ,  Derivatives; Dw[k] is the k-th derivative. MEMORY
                     FOR Dw MUST BE  ALLOCATED IN THE CALLING ROUTINE.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvDerivsAtKnot( NL_CURVE *cur, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_CPOINT *Dw )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvDerivsAtKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k, spn;

    NL_DEGREE p;

    NL_REAL *dum, ** BD;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &i, &Pw, &p, &i, &dum );
    N_CrvGetKnotVector( cur, &knt );

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute basis function derivatives */

    BD = N_AllocReal2dArray( der, p, &S );

    if( BD EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knt, p, u, flg, der, BD, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives */

    for ( k = 0; k <= der; k++ )
    {
        N_CopyCPt( NL_CZERO, &Dw[k] );

        for ( i = 0; i <= p; i++ )
        {
            N_VectorBlendCPt( BD[k][i], Pw[spn - p + i], &Dw[k] );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvDerivsAtKnot */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  routine aligns  a NURBS  curve so  that its  Frenet frame,
     computed at a given parameter, is aligned to a coordinate frame.
     The input data is destroyed, ie  the alignment is done IN PLACE. 
     A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_POINT      O;
       NL_VECTOR     X, Y, Z;
       ...
       (define cur and get O,X,Y,Z);
       ...
       N_CrvAlign(&cur,u,O,X,Y,Z);


   ACCESS:
   
     cur   , in/out ,  NURBS curve
     u     , input  ,  Parameter  at  which  Frenet  frame  is  to be 
                       computed
     O     , input  ,  Origin of a local coordinate system
     X,Y,Z , input  ,  Unit coordinate axes of local system


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR     

   ***********************************************************************/

NL_FLAG N_CrvAlign( NL_CURVE *cur, NL_PARAMETER u, NL_POINT O, NL_VECTOR X, NL_VECTOR Y, NL_VECTOR Z )
{
    NL_FLAG error = NL_NO;

    NL_POINT P;

    NL_VECTOR T, N, B;

    NL_RMATRIX rma;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Convert to 3-D if necessary */

    if( NOT N_CrvIs3d( cur ) )
        N_Crv2dTo3d( cur );

    /* Get Frenet frame */

    error = N_CrvEvalFrenetFrame( cur, u, NL_LEFT, &P, &T, &N, &B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get transformation matrix */

    N_InitRealMatrix( &rma );
    error = N_CreateTransformMatrixFromAxes( P, T, N, B, O, X, Y, Z, &rma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Transform curve */

    N_CrvTransform( cur, &rma );

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvAlign */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     Given curve, C(u), this routine computes n+1 points on C(u), which
     are  approximately  evenly  spaced.  Optionally, the corresponding
     u-parameter  values as  well as  the points  may be  returned. The 
     iteration  limit  NL_ITLIM  is  defined  as  a  global  parameter  in 
     "globals.h".  If NL_ITLIM is exceeded,  the currently  best  solution  
     is returned. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u0, u1;
       NL_INDEX      n;
       NL_REAL       tol, *u;
       NL_POINT      *P;
       ...
       (define cur, get u0, u1, tol, n, and get memory for P and u);
       ...
       N_CrvEvalEvenSpacedPts(&cur,u0,u1,n,tol,P   ,u   ); or
       N_CrvEvalEvenSpacedPts(&cur,u0,u1,n,tol,P   ,NULL); or
       N_CrvEvalEvenSpacedPts(&cur,u0,u1,n,tol,NULL,u   ); 


   ACCESS:
   
     cur   , input  ,  NURBS curve
     u0,u1 , input  ,  The n+1 points will be taken at parameter values
                       between u0 and u1.
     n     , input  ,  n+1 points are to be generated ( n > 1 ).
     tol   , input  ,  Let aver be the average distance  between neigh-
                       boring points on the curve,  and  let maxdev  be
                       the  maximum deviation of  any  two  neighboring
                       points  from  the average.  Then  maxdev/aver <=
                       tol.
     P     , output ,  The n+1 approximately evenly spaced points. This 
                       memory must be allocated in the calling routine.
                       if (P == NULL), then this is not used.
     u     , output ,  If (u == NULL),  then  this   is  not  used  (no
                       parameters  returned). If (u != NULL),  then  it
                       is assumed that u is an array of length at least 
                       n+1.  The   parameter   values  of   cur,  which 
                       correspond to the points in  P, are  returned in 
                       this array.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalEvenSpacedPts( NL_CURVE *cur, NL_PARAMETER u0, NL_PARAMETER u1, NL_INDEX n, NL_REAL tol, NL_POINT *P, NL_PARAMETER *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvEvalEvenSpacedPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k, its;

    NL_REAL *t, *oldt, *s, *temp, aver, dt, d, num, den;

    NL_POINT *Q;

    NL_KNOTVECTOR *knt;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetKnotVector( cur, &knt );

    /* Check parameters */

    error = N_KnotVectorIsParamOutOfBounds( knt, u0, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knt, u1, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( n LE 1 )
        NL_ERROR( NL_INP_ERR );

    /* Allocate arrays and initialize the t-array */
    Q = N_AllocPt1dArray( n, &S );

    if( Q EQ NULL )
        NL_QUIT;

    t = N_AllocReal1dArray( n, &S );

    if( t EQ NULL )
        NL_QUIT;

    oldt = N_AllocReal1dArray( n, &S );

    if( oldt EQ NULL )
        NL_QUIT;

    s = N_AllocReal1dArray( n, &S );

    if( s EQ NULL )
        NL_QUIT;

    oldt[0] = u0;
    oldt[n] = u1;
    t[0] = u0;
    t[n] = u1;
    dt = (u1 - u0) / n;

    for ( i = 1; i < n; i++ )
        t[i] = u0 + i * dt;

    /* Initialize start point */
    error = N_CrvEval( cur, u0, NL_LEFT, &Q[0] );

    if( error EQ NL_YES )
        NL_OUT;

    /* Iteratively compute distances, and linearly interpolate */
    /* to improve them                                         */
    s[0] = 0.0;

    for ( its = 1; its <= NL_ITLIM; its++ )
    {
        /* Compute points and distances */

        for ( i = 1; i <= n; i++ )
        {
            error = N_CrvEval( cur, t[i], NL_LEFT, &Q[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( Q[i - 1], Q[i], &d );
            s[i] = s[i - 1] + d;
        }

        aver = s[n] / n;

        /* Compute deviations */

        if( aver > NL_MTOL )
        {
            for ( i = 1; i <= n; i++ )
            {
                d = fabs( s[i] - s[i - 1] - aver );

                /* Note: expensive and unnecessary: use multiplication instead of division.
                **   if( N_FloatOpIsBad( d, aver, NL_DIVISION ) )
                **     NL_ERROR( NL_NUM_ERR );
                **   if( d / aver GT tol )
                */
                if( d GT aver * tol )
                    break;
            }
        }

        /* Convergence              -> Load u array and return. */
        /* Exceeded iteration limit -> Return best result.      */

        if( aver <= NL_MTOL OR i GT n OR its GE NL_ITLIM )
        {
            if( u NEQ NULL )
            {
                for ( k = 0; k <= n; k++ )
                    u[k] = t[k];
            }

            if( P NEQ NULL )
            {
                for ( k = 0; k <= n; k++ )
                {
                    N_CopyPt( Q[k], &P[k] );
                }
            }

            NL_OUT;
        }

        /* Recompute t-values using linear interpolation */

        temp = t;
        t = oldt;
        oldt = temp;

        k = 1;

        for ( i = 1; i < n; i++ )
        {
            d = i * aver;

            while( d GT s[k] )
                k++;
            num = oldt[k] - oldt[k - 1];
            den = s[k] - s[k - 1];

            if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            t[i] = (num / den) * (d - s[k - 1]) + oldt[k - 1];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvEvalEvenSpacedPts */



/*******************************************************************//**


   DESCRIPTION:

     This curve routine extends a Nurbs curve a  given(approximate)
     distance.  The start or end can be extended.  Continuity can  be
     controlled via options.  After extension, the original parameter
     range still maps to the original curve(i.e. the parameter range
     is also extended. A typical calling example is:

       NL_CURVE     curP, curQ;
       NL_REAL      dist;
       NL_STACKS    SP, SQ;
       ...
       (define curP and choose dist);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvExtendByDist(&curP, dist, NL_ABSOLUTE, NL_START, NL_CMAX, &curQ, &SP, &SQ);
       N_CrvExtendByDist(&curP, dist, NL_RELATIVE, NL_END, NL_G1, &curP, &SP, &SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's  
     knot vector and polygon objects. 
     We added limits to the recursion(for badly behaved curves).


   ACCESS:
   
     curP , input  ,  NURBS curve
     dist , input  ,  The arc length of curP is extended approximately by
                      an amount derived from dist.If flg = NL_ABSOLUTE, then 
                      dist is  an absolute distance.  If flg = NL_RELATIVE , 
                      then dist is multiplied by the arc length  of  curP 
                      to yield the desired extension distance.  In either 
                      case,  the  actual  distance  extended  is  only an 
                      approximation
     flg  , input  ,  Flag:
                       NL_ABSOLUTE: dist is an absolute distance
                       NL_RELATIVE: dist is scaled by the curve's arc length
                                 to yield the desired extension distance
     end  , input  ,  Flag:
                       NL_START: The curve is extended back from  its  start
                              point
                       NL_END  : The curve is extended forward from its  end
                              point
     cont , input  ,  Flag:
                       NL_G1  : curP is extended  by a linear segment.  curQ
                             is at least NL_G1(possibly NL_C1) where the  ext-
                             ension joins the original curve
                       NL_G2  : curP is extended by reflection,  yielding  a 
                             NL_G2 continuous extension
                       NL_CMAX: the extension yields infinite(C) continuity
                             at the join point(no knot there)
     curQ , output ,  Curve after extension
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   METHOD:

   ***********************************************************************/

NL_FLAG N_CrvExtendByDist( NL_CURVE *curP, NL_REAL dist, NL_FLAG flg, NL_FLAG end, NL_FLAG cont, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CrvExtendByDist"); */
    return (ST_curxcdr( curP, dist, flg, end, cont, curQ, SP, SQ, 0 ));
} /* end N_CrvExtendByDist */



/*******************************************************************//**
   ***********************************************************************/
NL_FLAG ST_curxcdr( NL_CURVE *curP, NL_REAL dist, NL_FLAG flg, NL_FLAG end, NL_FLAG cont, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ, int depth )
{
    NL_PRIVATE NL_STRING rname = _T("ST_curxcdr");

    NL_FLAG error = NL_NO;

    NL_CURVE curA, curB, *curptr;

    /*        NL_INTEGER     **bin;      */

    NL_INDEX ii, jj, kk, kp, kq, span = 0, mult = 0, n, m, mq, nr, mr;

    NL_DEGREE p;

    NL_CPOINT *Dw, *Pw, *Qw, *Rw;

    NL_POINT P1, P2;

    NL_VECTOR V;

    NL_PLANE pln;

    NL_KNOTVECTOR *knt;

    NL_BOOLEAN rat;

    NL_REAL ldst, *UP, *UQ, d1 = 0.0, du, uu = 0.0, ul, ur, dl, dr, dd, mag, wx, wy, wz, w, *UR, u1, u2, u3, d2;

    NL_STACKS SL;

    /* Limit recursion to 50 levels */
    if( depth > 50 )
        return (1);

    /* Start NURBS - set all stack pointers to NULL */
    N_InitNurbs( &SL );

    /* Get local notation - get curP internal pointers and max-index parameters */
    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* Get extension distance */
    ldst = dist;

    if( flg EQ NL_RELATIVE OR cont EQ NL_G2 )
    {
        /* let d1 = curP arcLength from first to last knot to a relative tol of 0.01 */
        error = N_CrvArcLength( curP, UP[0], UP[m], 0.01, NL_RELATIVE, &d1 );

        if( error EQ NL_YES )
            NL_OUT;

        /* when requested compute absolute distance from relative parameter */
        if( flg EQ NL_RELATIVE )
            ldst *= d1;

        /* check state - shorter than tolerance extension distances */
        if( d1 LE NL_MTOL AND d1 LE NL_PTOL )
            NL_ERROR( NL_INP_ERR );

        /* when extension distance is larger than the curve (for NL_G2 extensions) */
        /* extend the curve in small steps until its the requested size         */
        if( cont EQ NL_G2 )
            if( ldst GT 0.9313 *d1 )
            {
                /* pick an extension distance less than 1/2 of the current curve length */
                dd = 0.4671 *d1;

                /* recurse: extend curve by smaller amount = ldst - dd */
                error = ST_curxcdr( curP, ldst - dd, NL_ABSOLUTE, end, cont, curQ, SP, SQ, depth + 1 );

                if( error EQ NL_YES )
                    NL_OUT;

                /* recurse: extend curve again by dd to make a total extension of ldst */
                error = ST_curxcdr( curQ, dd, NL_ABSOLUTE, end, cont, curQ, SQ, SQ, depth + 1 );
                NL_OUT;
            } /* end large extension distance check */
    }         /* end relative extension distance or g2 extension type check */

    /* If cont == NL_G2, then get the parm value that corresponds    */
    /* roughly to a segment of length = ldst                      */
    /* either [Umin, uu] or [uu, Umax]                            */
    /* set mq = extended curve knot count after segment mirroring */

    /* when cont == NL_G2,                                           */
    /* extend curve by mirroring an end segment of ldst arcLength */
    /* and adding it to the curve end                             */
    if( cont EQ NL_G2 )
    {
        /* get curP->Knt pointer */
        N_CrvGetKnotVector( curP, &knt );

        /* du = expected increase in u range to handle requested increase in arcLength          */
        /* uu = will be param value marking curve segment to be mirrored onto end of this curve */
        /*      init uu to invalid value less than Umin                                         */
        du = (UP[m] - UP[0]) * (ldst / d1);
        uu = UP[0] - 10.0;

        /* for the start end */
        if( end EQ NL_START ) /* binary search */
        {
            /* ur = param value to find that bounds a curve segment with arcLength = ldst */
            /* dr = curve arcLength for curve segment from Umin to ur                     */
            ur = UP[0];
            dr = 0.0;

            /* find param range that maps to desired arcLength extension range */
            while( NL_YES )
            {
                /* ul and dl are last iteration ur and dr values */
                /* ur is incremented by du each iteration        */
                ul = ur;
                dl = dr;
                ur = ur + du;

                /* limit ur to max knot value */
                if( ur GT UP[m] )
                    ur = UP[m];

                /* let dr = curP arcLength from first knot to ur to a relative tol of .005*/
                error = N_CrvArcLength( curP, UP[0], ur, 0.005, NL_RELATIVE, &dr );

                if( error EQ NL_YES )
                    NL_OUT;

                /* check upcoming divide arguments for a math overflow error */
                /* No, just use multiplication instead of division. */
                /* if( N_FloatOpIsBad( fabs( ldst - dr ), ldst, NL_DIVISION ) ) */
                /*    NL_ERROR( NL_NUM_ERR ); */

                /*    when current param range maps to arcLength within 5% of extension distance */
                /* or when param range is maxed out                                              */
            /*  if( fabs( ldst - dr ) / ldst LT 0.05 OR ur EQ UP[m] ) */
                if( fabs( ldst - dr ) LT ldst * 0.05 OR ur EQ UP[m] )
                {
                    /* set uu value */
                    uu = ur;
                    break;
                }

                /* when dr is greater than extension distance (by 5%) - break */
                if( dr GT ldst )
                    break;
            } /* end while increasing uu gueses looking for proper arcLength segement */

            /* when a param range larger than ldst was found (by 5%)                             */
            /* iterate to a param range that maps to a curve segment arcLength within 5% of ldst */
            if( uu LT UP[0] )
            {
                for ( ii = 0; ii < 4; ii++ )
                {
                    /* let uu = expected value to get proper arc length from last iteration values */
                    uu = ul + (ldst - dl) * ((ur - ul) / (dr - dl));

                    /* let dd = store curP arcLength from Umin to uu */
                    error = N_CrvArcLength( curP, UP[0], uu, 0.005, NL_RELATIVE, &dd );

                    if( error EQ NL_YES )
                        NL_OUT;

                    /* when arcLength is ldst or longer - exit */
               /*   if( fabs( ldst - dd ) / ldst LT 0.05 ) */
                    if( fabs( ldst - dd ) LT ldst * 0.05 )
                        break;

                    /* select ur/ul values to get next guess closer to desired distance */
                    if( dd GT ldst )
                    {
                        ur = uu;
                        dr = dd;
                    }
                    else
                    {
                        ul = uu;
                        dl = dd;
                    }
                } /* end iter 4 times */
            }     /* end param range with long arcLength found check */

            /* find knot span for uu and mult of uu if it happens to be a knot value */
            error = N_BasisFindSpanAndMult( knt, p, uu, NL_RIGHT, &span, &mult );

            if( error EQ NL_YES )
                NL_OUT;

            /* when uu is not a knot value                                          */
            /* and  uu is within tol of span's lower knot bound                     */
            /* and  span's length is larger than tol size                           */
            /* then snap uu to tolerance size distance from span's lower knot bound */
            if( (uu NEQ UP[span + 1])AND( uu - UP[span]LT KN_TOL *( UP[m] - UP[0] ) )AND( UP[span] + KN_TOL *( UP[m] - UP[0] )LT UP[span + 1] ) )
            {
                uu = UP[span] + KN_TOL * (UP[m] - UP[0]);
            }

            /* let mq = number of knots in extended curve */
            mq = m + span;
        }    /* end targetEnd == Start branch */
        else /* end targetEnd == End branch */
        {
            /* iterate to find [uu,UMax] curve segment with arcLength >= ldst */
            ul = UP[m];
            dl = 0.0;

            while( NL_YES )
            {
                ur = ul;
                dr = dl;
                ul = ul - du;

                if( ul LT UP[0] )
                    ul = UP[0];

                error = N_CrvArcLength( curP, ul, UP[m], 0.005, NL_RELATIVE, &dl );

                if( error EQ NL_YES )
                    NL_OUT;

                /*  if( N_FloatOpIsBad( fabs( ldst - dl ), ldst, NL_DIVISION ) )
                **      NL_ERROR( NL_NUM_ERR );
                */
            /*  if( fabs( ldst - dl ) / ldst LT 0.05 OR ul EQ UP[0] )  */
                if( fabs( ldst - dl ) LT ldst * 0.05 OR ul EQ UP[0] )
                {
                    uu = ul;
                    break;
                }

                if( dl GT ldst )
                    break;
            }

            /* when a param range with arcLength GT ldst (by 5%) was found */
            /* iterate to param range with arcLength within 5% of ldst     */
            if( uu LT UP[0] )
            {
                for ( ii = 0; ii < 4; ii++ )
                {
                    uu = ur - (ldst - dr) * ((ur - ul) / (dl - dr));
                    error = N_CrvArcLength( curP, uu, UP[m], 0.005, NL_RELATIVE, &dd );

                    if( error EQ NL_YES )
                        NL_OUT;

                    /* [uu,Umax] range is close enough */
                /*  if( fabs( ldst - dd ) / ldst LT 0.05 ) */
                    if( fabs( ldst - dd ) LT ldst * 0.05 )
                        break;

                    /* set ul,dl,ur,dr values for next better guess */
                    if( dd GT ldst )
                    {
                        ul = uu;
                        dl = dd;
                    }
                    else
                    {
                        ur = uu;
                        dr = dd;
                    }
                }
            }

            /* get knot span containing uu and mult if uu happens to be a knot value */
            error = N_BasisFindSpanAndMult( knt, p, uu, NL_LEFT, &span, &mult );

            if( error EQ NL_YES )
                NL_OUT;

            /* when uu is not a knot value                                          */
            /* and  uu is within tol of span's upper knot bound                     */
            /* and  span's length is larger than tol size                           */
            /* then snap uu to tolerance size distance from span's upper knot bound */
            if( (uu NEQ UP[span])AND( UP[span + 1] - uu LT KN_TOL *( UP[m] - UP[0] ) )AND( UP[span + 1] - KN_TOL *( UP[m] - UP[0] )GT UP[span] ) )
            {
                uu = UP[span + 1] - KN_TOL * (UP[m] - UP[0]);
            }

            /* let mq = number of knots in extended curve */
            mq = 2 * m - span - 1;
        } /* end targetEnd == NL_END branch */
    }     /* end cont == NL_G2 branch */
    else if( cont EQ NL_CMAX )
    {
        mult = 1;

        if( end EQ NL_START )
        {
            uu = UP[p + 1];

            while( p + mult + 1 LT m )
            {
                if( uu EQ UP[p + mult + 1] )
                    mult += 1;
                else
                    break;
            }
        }
        else
        {
            uu = UP[m - p - 1];

            while( m - p - mult - 1 GT 0 )
            {
                if( uu EQ UP[m - p - mult - 1] )
                    mult += 1;
                else
                    break;
            }
        }

        /* let mq = number of knots in extended curve */
        mq = m + 2 * p - mult;
    }    /* end if (cont EQ NL_CMAX) branch */
    else /* cont NE NL_G2 nor NL_CMAX branch */
    {    /* let mq = number of knots in extended curve */
        mq = m + p;
    }

    /* CurP is about to be given new memory                      */
    /* save curP object pointers in curA keeping Pw and UP valid */
    curA = *curP;

    /* when extending the curve in place */
    if( curP EQ curQ )
    {
        /* allocate control point and knot memory */
        error = N_AllocCrvArrays( curP, mq - p - 1, p, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        /* get curQ=curP Pw and U pointers to uninitialized memory */
        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else /* extending the curve into new memory */
    {
        /* allocate control point and knot memory */
        error = N_CrvSizeArrays( curQ, mq - p - 1, p, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        /* get curQ Pw and U pointers to uninitialized memory */
        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Compute the new control points and knots defining the extension */

    switch( cont )
    {
        case NL_G1:
            if( end EQ NL_START )
            { /* compute new start control point */
                /* convert 1st two control Points to euclidian coordinates */
                N_CPtToPtEuclid( Pw[0], &P1 );
                N_CPtToPtEuclid( Pw[1], &P2 );

                /* get curve end-tangent unit vector = p1 - p2 */
                N_VectorDir( P2, P1, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error )
                {
                    /* try getting end-tangent unit vector from 2nd and 3rd points */
                    N_CPtToPtEuclid( Pw[2], &P2 );
                    N_VectorDir( P2, P1, &V );
                    error = N_VectorNormalize( V, &V, &mag );
                }

                /* if failed to get end-tangent vector - quit */
                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );

                /* move p1 to extension distance on tangent vector */
                N_VectorBlendPt( ldst, V, &P1 );

                /* get p1 coordinate values */
                N_PtToXYZ( P1, &wx, &wy, &wz );

                /* let weight = weight for symmetric 1st interior point */
                N_CPtGetW( Pw[1], &w );

                /* when needed - make homogeneous coordinates for moved p1 position */
                if( w NEQ NL_NOW )
                {
                    wx *= w;
                    wy *= w;
                    wz *= w;
                }

                /* when working with a NL_NOZ - set new control Point z coordinate to NL_NOZ */
                N_CPtToWxWyWz( Pw[0], &d1, &d1, &dd, &d1 );

                if( dd EQ NL_NOZ )
                    wz = NL_NOZ;

                /* place new coordinate values into new Qw[0] control Point */
                N_CPtFromWxWyWz( wx, wy, wz, w, &Qw[0] );

                /* compute p - 1 new internal ctrl pts */
                dd = (NL_REAL)p;

                for ( ii = 1; ii < p; ii++ )
                { /*(degree elevation)                */
                    d1 = (NL_REAL)ii / dd;

                    /* let Qw[ii] = (1-ii/p)*Qw[0] + (ii/p)*Pw[0] */
                    N_Combine2CPts( 1.0 - d1, Qw[0], d1, Pw[0], &Qw[ii] );
                }

                /* get new start knot increment distance */
                du = ldst * (UP[p + 1] - UP[0]) / (dd * mag);

                /* snap too close new start knots increments to smallest allowed tolerance */
                if( du LT KN_TOL * (UP[m] - UP[0]) )
                    du = KN_TOL * (UP[m] - UP[0]);

                /* get knot value ending curve section to mirror */
                uu = UP[0] - du;

                /* set end knot multiplicity */
                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = uu;

                /* set kk, kq and kp */
                kk = p;     /* old to new control point map; Qw[ii+kk] = Pw[ii] */
                kq = p + 1; /* old to new knot map; UQ[ii+kq] = UP[ii+kp] */
                kp = 1;
            }               /* end (cont EQ NL_G1) AND (end EQ NL_START) branch */
            else            /* (cont EQ NL_G1) AND (end EQ NL_END) branch */
            {               /* compute new end control point */
                /* convert last two control Points to euclidian coordinates */
                N_CPtToPtEuclid( Pw[n - 1], &P1 );
                N_CPtToPtEuclid( Pw[n], &P2 );

                /* get curve end-tangent unit vector = p2 - p1 */
                N_VectorDir( P1, P2, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error )
                {
                    /* try getting end-tangent unit vector from 2nd and 3rd to last points */
                    N_CPtToPtEuclid( Pw[n - 2], &P1 );
                    N_VectorDir( P1, P2, &V );
                    error = N_VectorNormalize( V, &V, &mag );
                }

                /* if failed to get end-tangent vector - quit */
                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );

                /* move p2 to extension distance on tangent vector */
                N_VectorBlendPt( ldst, V, &P2 );

                /* get p2 coordinate values */
                N_PtToXYZ( P2, &wx, &wy, &wz );

                /* let weight = weight for symmetric 1st interior point */
                N_CPtGetW( Pw[n - 1], &w );

                /* when needed - make homogeneous coordinates for moved p1 position */
                if( w NEQ NL_NOW )
                {
                    wx *= w;
                    wy *= w;
                    wz *= w;
                }

                /* when working with a NL_NOZ - set new control Point z coordinate to NL_NOZ */
                N_CPtToWxWyWz( Pw[0], &d1, &d1, &dd, &d1 );

                if( dd EQ NL_NOZ )
                    wz = NL_NOZ;

                /* place new coordinate values into new Qw[n + p] control Point */
                N_CPtFromWxWyWz( wx, wy, wz, w, &Qw[n + p] );

                /* compute p - 1 new internal ctrl pts */
                dd = (NL_REAL)p;

                for ( ii = 1; ii < p; ii++ )
                { /*(degree elevation)                */
                    d1 = (NL_REAL)ii / dd;

                    /* let Qw[n+ii] = (1-ii/p)*Pw[n] + (ii/p)*Qw[n+p] */
                    N_Combine2CPts( 1.0 - d1, Pw[n], d1, Qw[n + p], &Qw[n + ii] );
                }

                /* get new end knot increment distance */
                du = ldst * (UP[m] - UP[m - p - 1]) / (dd * mag); /* get new end knot */

                /* snap too close new start knots incremetns to smallest allowed tolerance */
                if( du LT KN_TOL * (UP[m] - UP[0]) )
                    du = KN_TOL * (UP[m] - UP[0]);

                /* get knot value ending curve section to mirror */
                uu = UP[m] + du;

                /* set end knot multiplicity */
                for ( ii = 0; ii <= p; ii++ )
                    UQ[m + ii] = uu;

                /* set kk, kq and kp */
                kk = 0; /* old to new control point map; Qw[ii+kk] = Pw[ii] */
                kq = 0; /* old to new knot map; UQ[ii+kq] = UP[ii+kp] */
                kp = 0;
            }           /* end (cont EQ NL_G1) AND (end EQ NL_END) branch */

            /* load remaining control points */
            for ( ii = 0; ii <= n; ii++ )
            {
                N_CopyCPt( Pw[ii], &Qw[kk] );
                kk += 1;
            }

            /* load remaining knots */
            for ( ii = 1; ii <= m; ii++ )
            {
                UQ[kq++] = UP[kp++];
            }

            break;

        case NL_G2:

            /* make sure uu value is a p multiple knot in input curve*/
            if( mult LT p ) /* Insert uu to mult = p */
            {
                /* set curB pointers to NULL */
                N_CrvInitArrays( &curB );
                curptr = &curB;

                /* copy original input curA into curB */
                error = N_CrvCopy( &curA, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* insert knot to curB at uu value, p-mult times */
                error = N_CrvInsertKnot( &curB, uu, p - mult, &curB, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* increment end span value to get past inserted knots */
                if( end EQ NL_END )
                    span += (p - mult);
            }
            else                /* uu is a p multiple knot - set curptr to original curve curA */
            {
                curptr = &curA; /* bug? this should be curA? */
            }

            /* get target curve's CPoint index, array, degree, knot index and array values */
            N_CrvGetCPtsDegreeAndKnots( curptr, &nr, &Rw, &p, &mr, &UR );

            /* branch on extension end */
            if( end EQ NL_START )
            {
                /* get Curve end unit-Tangent from 1st two end points */
                N_CPtToPtEuclid( Pw[0], &P1 ); /* get reflection plane */
                N_CPtToPtEuclid( Pw[1], &P2 );
                N_VectorDir( P2, P1, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error EQ NL_YES )
                {
                    /* try using 3rd point to compute unitTangent */
                    N_CPtToPtEuclid( Pw[2], &P2 );
                    N_VectorDir( P2, P1, &V );
                    error = N_VectorNormalize( V, &V, &mag );
                }

                /* failed to compute endTangent - quit */
                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );

                /* define a plane at curve endPoint normal to curve tangent */
                N_CreatePlanePtNormal( &pln, P1, V );

                kk = 0; /* load reflected control points */

                /* for every control point being reflected */
                for ( ii = span; ii > 0; ii-- )
                {
                    /* reflect Rw[ii] into Qw[ii+kk] */
                    N_CptReflect( Rw[ii], pln, &Qw[kk] );
                    kk += 1;
                }

                /* load end-knots for reflected section */
                dd = 2.0 *UP[0];

                for ( ii = 0; ii <= p; ii++ )
                {
                    /* let UQ[endKnots] = UP[0] - du                             */
                    /* where du is knot distance of curve section being mirrored */
                    /*                  dd = 2*UP[0]    */
                    /*                  uu = UP[0] + du */
                    UQ[ii] = dd - uu;
                }

                /* load internal knots for reflected section */
                kq = p + 1;

                for ( ii = span; ii > p; ii-- )
                    UQ[kq++] = dd - UP[ii];
                kp = 1;
            }    /* end extending NL_START of curve branch */
            else /* extending NL_END of curve branch */
            {
                /* get endTangent from last 2 control Points */
                N_CPtToPtEuclid( Pw[n], &P1 ); /* get reflection plane */
                N_CPtToPtEuclid( Pw[n - 1], &P2 );
                N_VectorDir( P2, P1, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error EQ NL_YES )
                {
                    /* try getting endTangent from last 3 control points */
                    N_CPtToPtEuclid( Pw[n - 2], &P2 );
                    N_VectorDir( P2, P1, &V );
                    error = N_VectorNormalize( V, &V, &mag );
                }

                /* failed to get endTangent - quit */
                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );

                /* define plane at curve endPoint normal to curve endTangent */
                N_CreatePlanePtNormal( &pln, P1, V );

                kk = n + 1; /* load reflected control points */

                /* for every control point being mirrored */
                for ( ii = nr - 1; ii >= span - p; ii-- )
                {
                    /* reflect orig curve control Points into target curve */
                    N_CptReflect( Rw[ii], pln, &Qw[kk] );
                    kk += 1;
                }

                kk = 0;
                dd = 2.0 *UP[m];

                /* set mirrored endPoint knots */
                for ( ii = 0; ii <= p; ii++ ) /* load knots for reflected section */
                    UQ[mq - p + ii] = dd - uu;

                /* set mirrored interior knot values */
                kq = mq - p - 1;

                for ( ii = span + 1; ii < mr - p; ii++ )
                    UQ[kq--] = dd - UR[ii];

                kq = 0;
                kp = 0;
            } /* end extending end of Curve branch */

            /* load remaining control points */
            for ( ii = 0; ii <= n; ii++ )
            {
                /* copy Pw ControlPoints into Qw */
                N_CopyCPt( Pw[ii], &Qw[kk] );
                kk += 1;
            }

            /* load remaining knots */
            for ( ii = 1; ii <= m; ii++ )
            {
                UQ[kq++] = UP[kp++];
            }

            break;

        case NL_CMAX:

            kk = p;

            /*               allocate bin[kk+1][kk+1] integer array    */
            /*              bin = N_AllocInt2dArray(kk, kk, &SL);               */
            /*              if (bin EQ NULL)                           */
            /*                NL_QUIT;                                    */
            /*                                                         */
            /*              load bin with pascal triangle values       */
            /*              N_PascalTriRow(bin, kk);                         */

            if( mult LT p ) /* Insert uu to mult = p */
            {
                /* set curB to NULL curve */
                N_CrvInitArrays( &curB );
                curptr = &curB;

                /* copy curA into curB */
                error = N_CrvCopy( &curA, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* insert knot at uu value p times */
                error = N_CrvInsertKnot( &curB, uu, p - mult, &curB, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else /* no need for knot insertion just use curA as the target curve */
            {
                curptr = &curA;
            }

            /* get targetCurve's controlPoint index and array, degree, knot index and array values */
            N_CrvGetCPtsDegreeAndKnots( curptr, &nr, &Rw, &p, &mr, &UR );

            /* get targetCurve's rational state */
            rat = N_IsCrvRat( curptr );

            /* allocate control Point array sized:[p] */
            Dw = N_AllocCPt1dArray( p, &SL );

            if( Dw EQ NULL )
                NL_QUIT;

            /* branch on curve extension end */
            if( end EQ NL_START )
            {
                for ( ii = 0; ii <= nr; ii++ ) /* load control points from curP */
                    N_CopyCPt( Rw[ii], &Qw[ii + p] );

                for ( ii = 1; ii <= mr; ii++ ) /* load knots from curP */
                    UQ[ii + p] = UR[ii];

                dr = p / (UR[p + 1] - UR[0]);

                for ( ii = 1; ii <= p; ii++ ) /* this loop computes derivatives */
                {                             /* from the right                 */
                    N_CopyCPt( Rw[ii], &Dw[ii] );

                    if( ii % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < ii; kk++ ) /* compute the ii - th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * (NL_REAL)NL_PascalTri[ii][kk], Rw[kk], &Dw[ii] );
                        kp *= -1;
                    }

                    N_ScaleCPt( dr, Dw[ii], &Dw[ii] );
                    dr = dr * ((NL_REAL)p - (NL_REAL)ii) / (UR[p + 1] - UR[0]);
                }

                u1 = 0.0;
                d1 = 0.0;                    /* u is span length, d is arc length */
                u2 = UR[p + 1] - UR[0];

                for ( ii = 0; ii < 5; ii++ ) /* iteratively compute new start seg- */
                {                            /* ment until arc length is correct   */
                    for ( jj = 0; jj <= p; jj++ )
                        UQ[jj] = UR[0] - u2;

                    dl = u2 / p;

                    for ( jj = 1; jj <= p; jj++ ) /* this loop computes */
                    {                             /* new control points */
                        if( jj % 2 EQ 0 )
                            kp = -1;
                        else
                            kp = 1;

                        N_CopyCPt( Dw[jj], &Qw[p - jj] );
                        N_ScaleCPt( -kp * dl, Qw[p - jj], &Qw[p - jj] );

                        if( jj LT p )
                            dl = dl * u2 / ((NL_REAL)p - (NL_REAL)jj);

                        for ( kk = 0; kk < jj; kk++ ) /* compute the(p - jj) - th ctrl pt */
                        {
                            N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[jj][kk], Qw[p - kk], &Qw[p - jj] );
                            kp *= -1;
                        }
                    }

                    if( rat ) /* don't create unacceptable weights */
                    {
                        for ( jj = 0; jj < p; jj++ )
                        {
                            N_CPtGetW( Qw[jj], &w );

                            if( w LT NL_WMIN OR w GT NL_WMAX )
                                break;
                        }

                        if( jj LT p )
                        {
                            if( ii EQ 4 OR u2 LE u1 )
                                NL_ERROR( NL_WEI_ERR );
                            u2 = 0.5 *( u1 + u2 );
                            continue;
                        }
                    }

                    /* now check the arc length of the new segment */

                    error = N_CrvArcLength( curQ, UQ[0], UQ[p + 1], 0.005, NL_RELATIVE, &d2 );

                    if( error EQ NL_YES )
                        NL_OUT;

                /*  if( fabs( ldst - d2 ) / ldst LT 0.03 ) */
                    if( fabs( ldst - d2 ) LT ldst * 0.03 )
                        break; /* close enough */

                    if( N_FloatOpIsBad( u2 - u1, d2 - d1, NL_DIVISION ) )
                        break;

                    u3 = u2;
                    u2 = u1 + (ldst - d1) * ((u2 - u1) / (d2 - d1));
                    u1 = u3;
                    d1 = d2;
                }

                /* remove the original start knot */

                error = N_CrvRemoveKnot( curQ, UP[0], p, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }                                  /* end extending curve Start branch */
            else                               /* extend curve End branch */
            {
                for ( ii = 0; ii <= nr; ii++ ) /* load control points from curP */
                    N_CopyCPt( Rw[ii], &Qw[ii] );

                for ( ii = 0; ii < mr; ii++ )  /* load knots from curP */
                    UQ[ii] = UR[ii];

                dl = -p / (UR[mr] - UR[mr - p - 1]);

                for ( ii = 1; ii <= p; ii++ ) /* this loop computes derivatives */
                {                             /* from the left                  */
                    N_CopyCPt( Rw[nr - ii], &Dw[ii] );

                    if( ii % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < ii; kk++ ) /* compute the ii - th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Rw[nr - kk], &Dw[ii] );
                        kp *= -1;
                    }
                    N_ScaleCPt( dl, Dw[ii], &Dw[ii] );
                    dl = -dl * ((NL_REAL)p - (NL_REAL)ii) / (UR[mr] - UR[mr - p - 1]);
                }

                u1 = 0.0;
                d1 = 0.0;                    /* u is span length, d is arc length */
                u2 = UR[mr] - UR[mr - p - 1];

                for ( ii = 0; ii < 5; ii++ ) /* iteratively compute new end seg- */
                {                            /* ment until arc length is correct */
                    for ( jj = 0; jj <= p; jj++ )
                        UQ[mr + jj] = UR[mr] + u2;

                    dr = u2 / p;

                    for ( jj = 1; jj <= p; jj++ ) /* this loop computes */
                    {                             /* new control points */
                        if( jj % 2 EQ 0 )
                            kp = -1;
                        else
                            kp = 1;

                        N_CopyCPt( Dw[jj], &Qw[nr + jj] );
                        N_ScaleCPt( dr, Qw[nr + jj], &Qw[nr + jj] );

                        if( jj LT p )
                            dr = dr * u2 / ((NL_REAL)p - (NL_REAL)jj);

                        for ( kk = 0; kk < jj; kk++ ) /* compute the(nr + jj) - th ctrl pt */
                        {
                            N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[jj][kk], Qw[nr + kk], &Qw[nr + jj] );
                            kp *= -1;
                        }
                    }

                    if( rat ) /* don't create unacceptable weights */
                    {
                        for ( jj = 1; jj <= p; jj++ )
                        {
                            N_CPtGetW( Qw[nr + jj], &w );

                            if( w LT NL_WMIN OR w GT NL_WMAX )
                                break;
                        }

                        if( jj LE p )
                        {
                            if( ii EQ 4 OR u2 LE u1 )
                                NL_ERROR( NL_WEI_ERR );
                            u2 = 0.5 *( u1 + u2 );
                            continue;
                        }
                    }

                    /* now check the arc length of the new segment */

                    error = N_CrvArcLength( curQ, UQ[mq - p - 1], UQ[mq], 0.005, NL_RELATIVE, &d2 );

                    if( error EQ NL_YES )
                        NL_OUT;

                /*  if( fabs( ldst - d2 ) / ldst LT 0.03 ) */
                    if( fabs( ldst - d2 ) LT ldst * 0.03 )
                        break; /* close enough */

                    if( N_FloatOpIsBad( u2 - u1, d2 - d1, NL_DIVISION ) )
                        break;

                    u3 = u2;
                    u2 = u1 + (ldst - d1) * ((u2 - u1) / (d2 - d1));
                    u1 = u3;
                    d1 = d2;
                }

                /* remove the original end knot */

                error = N_CrvRemoveKnot( curQ, UP[m], p, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            } /* end extending curve Start branch */

            if( mult LT p )
            { /* remove the additional instances of uu */
                error = N_CrvRemoveKnot( curQ, uu, p - mult, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            break;
    } /* end switch on cont */

    /* If extension is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* End NURBS and Exit */

    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end ST_curxcdr */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This curve routine extends a Nurbs curve to a point. The start or
     end can be extended.  Continuity can  be  controlled via options.  
     After extension,  the original parameter range still maps to  the 
     original curve (i.e. the parameter range is also extended. A typ-
     ical calling example is:

       NL_CURVE     curP, curQ;
       NL_POINT     pt;
       NL_STACKS    SP, SQ;
       ...
       (define curP and choose pt);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvExtendToPt(&curP,pt,NL_START,NL_CMAX,&curQ,&SP,&SQ);
       N_CrvExtendToPt(&curP,pt,NL_END,NL_G1,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's  
     knot vector and polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     pt   , input  ,  Point to where the curve is extended
     end  , input  ,  Flag:
                       NL_START: The curve is extended back from  its  start
                              point
                       NL_END  : The curve is extended forward from its  end
                              point
     cont , input  ,  Flag:
                       NL_G1  : curP is  extended  by  a  quadratic  segment
                             (unless it is degree 1,  in which case NL_G1 is
                             not attained). curQ is at least NL_G1 where the  
                             extension joins the original curve
                       NL_CMAX: the extension yields (C) continuity degree-1
                             at the join point (a single knot)
     curQ , output ,  Curve after extension
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvExtendToPt( NL_CURVE *curP, NL_POINT pt, NL_FLAG end, NL_FLAG cont, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvExtendToPt");

    NL_FLAG error = NL_NO, parm, boo;

    NL_INDEX mq, n, m, ii, jj, kk = 0, kp, kq, mult = 0, nr, mr;

    NL_CURVE curA, curB, *curptr;

    NL_CPOINT cpt, *Pw, *Qw, *Rw, Dw;

    NL_POINT D[2], QQ, Q[2];

    NL_VECTOR V1, V2, V3;

    NL_LINESEG lsg1, lsg2;

    /*        NL_INTEGER     **bin;                  */

    NL_DEGREE p;

    NL_BOOLEAN rat, c3d;

    NL_REAL t1, t2, mag, uu = 0.0, d1, d2, du, *UP, *UQ, *UR, dr, dl, w, z, fac[4];

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* See if memory is needed */

    curA = *curP;

    if( cont EQ NL_G1 OR p EQ 1 )
        mq = m + p;
    else
    {
        if( end EQ NL_START )
        {
            uu = UP[p + 1]; /* determine multiplicity of first internal knot */
            mult = 1;

            while( p + mult + 1 LT m )
                if( uu EQ UP[p + mult + 1] )
                    mult += 1;
                else
                    break;
        }
        else
        {
            uu = UP[m - p - 1]; /* determine multiplicity of last internal knot */
            mult = 1;

            while( m - p - mult - 1 GT 0 )
                if( uu EQ UP[m - p - mult - 1] )
                    mult += 1;
                else
                    break;
        }

        mq = m + p - mult + p;
    }

    if( curP EQ curQ )
    {
        error = N_AllocCrvArrays( curP, mq - p - 1, p, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, mq - p - 1, p, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Make control point out of the point, pt */
    N_PtToCPt( pt, &cpt );
    N_CPtToWxWyWz( cpt, &d1, &d1, &z, &w );
    rat = N_IsCrvRat( &curA );

    if( rat )
        N_CPtSetW( 1.0, &cpt );
    c3d = N_CrvIs3d( &curA );

    if( NOT c3d )
        if( z EQ 0.0 )
        {
            z = NL_NOZ;
            N_CPtSetZ( z, &cpt );
        }

    /* Handle degree = 1 as a special case */

    if( p EQ 1 )
    {
        if( m LE 3 ) /* decide how to parameterize */
        {
            du = UP[m] - UP[0];
            parm = NL_CHORDLENGTH;
        }
        else
        {
            du = UP[2] - UP[1];
            d1 = KN_TOL * (UP[m] - UP[0]);

            for ( ii = 3; ii < m; ii++ )
                if( fabs( UP[ii] - UP[ii - 1] - du )GT d1 )
                    break;

            if( ii GE m )
                parm = NL_UNIFORM;
            else
                parm = NL_CHORDLENGTH;
        }

        if( end EQ NL_START )
        {
            N_CopyCPt( cpt, &Qw[0] );
            kk = 1;
            kp = 1;
            kq = 2;

            if( parm EQ NL_CHORDLENGTH )
            {
                N_DistCptCpt( Pw[0], Qw[0], &d1 );
                N_DistCptCpt( Pw[0], Pw[1], &d2 );

                if( NOT N_FloatOpIsBad( d1, d2, NL_DIVISION ) )
                    du = du * (d1 / d2);
            }
            UQ[0] = UQ[1] = UP[0] - du;
        }
        else
        {
            N_CopyCPt( cpt, &Qw[n + 1] );
            kk = 0;
            kp = 0;
            kq = 0;
            du = UP[m] - UP[m - 2];

            if( parm EQ NL_CHORDLENGTH )
            {
                N_DistCptCpt( Pw[n], Qw[n + 1], &d1 );
                N_DistCptCpt( Pw[n], Pw[n - 1], &d2 );

                if( NOT N_FloatOpIsBad( d1, d2, NL_DIVISION ) )
                    du = du * (d1 / d2);
            }
            UQ[mq] = UQ[mq - 1] = UP[m] + du;
        }

        for ( ii = 0; ii <= n; ii++ ) /* load remaining control points */
        {
            N_CopyCPt( Pw[ii], &Qw[kk] );

            if( z NEQ NL_NOZ AND NOT c3d )
                N_CPtSetZ( 0.0, &Qw[kk] );
            kk += 1;
        }

        for ( ii = 1; ii <= m; ii++ ) /* load remaining knots */
            UQ[kq++] = UP[kp++];

        /* If extension is in place, kill old curve */

        if( curP EQ curQ )
            N_FreeCrv( &curA, SP );

        NL_OUT;
    }

    /* Compute the new control points and knots defining the extension */

    switch( cont )
    {
        case NL_G1:
            if( end EQ NL_START )
            {
                uu = UP[0];
                error = N_CrvDerivs( &curA, uu, NL_LEFT, 1, D );

                if( error EQ NL_YES )
                    NL_OUT;
                N_VectorMagnitude( D[1], &mag );

                if( mag LE NL_MTOL AND mag LE NL_PTOL )
                    NL_ERROR( NL_GEO_ERR );

                N_CopyCPt( cpt, &Qw[p - 2] ); /* load initial control points */
                kk = p;

                for ( ii = 0; ii <= n; ii++ )
                {
                    N_CopyCPt( Pw[ii], &Qw[kk] );

                    if( z NEQ NL_NOZ AND NOT c3d )
                        N_CPtSetZ( 0.0, &Qw[kk] );
                    kk += 1;
                }

                /* Now get middle control point of quadratic Bezier extension */

                N_CreateLineStartDirVector( &lsg1, D[0], D[1], NL_UNBOUNDED );
                error = N_ProjectPtLine( lsg1, pt, &Q[0], &t1, &boo );

                if( error EQ NL_YES )
                    NL_OUT;
                N_Combine2Pts( 0.5, D[0], 0.5, pt, &QQ );
                N_VectorDir( D[0], pt, &V1 );
                error = N_VectorNormalize( V1, &V1, &d1 );

                if( error EQ NL_YES )
                    NL_OUT;
                N_VectorCross( V1, D[1], &V3 );
                error = N_VectorNormalize( V3, &V3, &d1 );

                if( error EQ NL_YES )
                {
                    N_VectorDot( V1, D[1], &d1 );

                    if( d1 GE 0.0 )
                        NL_ERROR( NL_INP_ERR );
                    error = NL_NO;
                }
                else
                {                                 /* lsg2 is perpendicular bisector */
                    N_VectorCross( V3, V1, &V2 ); /* of chord from D[0] to pt       */
                    N_CreateLineStartDirVector( &lsg2, QQ, V2, NL_UNBOUNDED );
                    error = N_IsectLineLine( lsg1, lsg2, &Q[1], &t2, &d1, &boo );

                    if( error EQ NL_YES OR NOT boo )
                    {
                        t2 = 1.0 + fabs( t1 );
                        error = NL_NO;
                    }
                    t1 = -t1;
                    t2 = -t2;

                    N_DistPtPt( D[0], pt, &d2 );
                    boo = NL_FALSE;

                    if( t1 GT 0.0 AND t2 GT 0.0 )
                    {
                        boo = NL_TRUE;

                        if( t1 LT t2 )
                            kk = 0;
                        else
                            kk = 1;
                    }
                    else if( t1 GT 0.0 )
                    {
                        boo = NL_TRUE;
                        kk = 0;
                    }
                    else if( t2 GT 0.0 )
                    {
                        boo = NL_TRUE;
                        kk = 1;
                    }

                    if( boo )
                    {
                        N_DistPtPt( D[0], Q[kk], &d1 );

                        if( d1 LE NL_MTOL AND d1 LE NL_PTOL AND d1 LT 0.2 *d2 )
                            boo = NL_FALSE;
                    }

                    if( boo )
                        N_CopyPt( Q[kk], &QQ );
                    else
                        N_Combine2Pts( 1.0, D[0], -0.2 *d2 / mag, D[1], &QQ );
                }

                if( rat ) /* make QQ the middle control point */
                {
                    N_CPtGetW( Pw[0], &d1 );
                    d1 = 0.5 *( 1.0 + d1 );
                    N_ScalePt( d1, QQ, &QQ );
                }
                N_PtToCPt( QQ, &Qw[p - 1] );

                if( rat )
                    N_CPtSetW( d1, &Qw[p - 1] );

                if( z EQ NL_NOZ )
                    N_CPtSetZ( z, &Qw[p - 1] );

                if( p GT 2 ) /* degree elevate if necessary */
                {
                    for ( ii = 3; ii <= p; ii++ )
                    {
                        d1 = (NL_REAL)ii;
                        N_CopyCPt( Qw[p - ii + 1], &Qw[p - ii] );

                        for ( kk = 1; kk < ii; kk++ )
                        {
                            d2 = (NL_REAL)kk / d1;
                            N_Combine2CPts( 1.0 - d2, Qw[p - ii + kk + 1], d2, Qw[p - ii + kk], &Qw[p - ii + kk] );
                        }
                    }
                }

                for ( ii = 1; ii <= m; ii++ )
                    UQ[p + ii] = UP[ii]; /* load knots */

                mag /= p;

                if( rat )
                {
                    N_CPtGetW( Qw[p], &d1 );
                    N_CPtGetW( Qw[p - 1], &d2 );
                    mag *= (d1 / d2);
                }
                N_DistCptCpt( Qw[p], Qw[p - 1], &d1 );
                du = d1 / mag;

                if( du LT KN_TOL * (UP[m] - UP[0]) )
                    du = KN_TOL * (UP[m] - UP[0]);

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = UP[0] - du;
            }
            else
            {
                uu = UP[m];
                error = N_CrvDerivs( &curA, uu, NL_RIGHT, 1, D );

                if( error EQ NL_YES )
                    NL_OUT;
                N_VectorMagnitude( D[1], &mag );

                if( mag LE NL_MTOL AND mag LE NL_PTOL )
                    NL_ERROR( NL_GEO_ERR );

                N_CopyCPt( cpt, &Qw[n + 2] ); /* load initial control points */

                for ( ii = 0; ii <= n; ii++ )
                {
                    N_CopyCPt( Pw[ii], &Qw[ii] );

                    if( z NEQ NL_NOZ AND NOT c3d )
                        N_CPtSetZ( 0.0, &Qw[ii] );
                }

                /* Now get middle control point of quadratic Bezier extension */

                N_CreateLineStartDirVector( &lsg1, D[0], D[1], NL_UNBOUNDED );
                error = N_ProjectPtLine( lsg1, pt, &Q[0], &t1, &boo );

                if( error EQ NL_YES )
                    NL_OUT;
                N_Combine2Pts( 0.5, D[0], 0.5, pt, &QQ );
                N_VectorDir( D[0], pt, &V1 );
                error = N_VectorNormalize( V1, &V1, &d1 );

                if( error EQ NL_YES )
                    NL_OUT;
                N_VectorCross( V1, D[1], &V3 );
                error = N_VectorNormalize( V3, &V3, &d1 );

                if( error EQ NL_YES )
                {
                    N_VectorDot( V1, D[1], &d1 );

                    if( d1 LE 0.0 )
                        NL_ERROR( NL_INP_ERR );
                    error = NL_NO;
                }
                else
                {                                 /* lsg2 is perpendicular bisector */
                    N_VectorCross( V3, V1, &V2 ); /* of chord from D[0] to pt       */
                    N_CreateLineStartDirVector( &lsg2, QQ, V2, NL_UNBOUNDED );
                    error = N_IsectLineLine( lsg1, lsg2, &Q[1], &t2, &d1, &boo );

                    if( error EQ NL_YES OR NOT boo )
                    {
                        t2 = -1.0 - fabs( t1 );
                        error = NL_NO;
                    }

                    N_DistPtPt( D[0], pt, &d2 );
                    boo = NL_FALSE;

                    if( t1 GT 0.0 AND t2 GT 0.0 )
                    {
                        boo = NL_TRUE;

                        if( t1 LT t2 )
                            kk = 0;
                        else
                            kk = 1;
                    }
                    else if( t1 GT 0.0 )
                    {
                        boo = NL_TRUE;
                        kk = 0;
                    }
                    else if( t2 GT 0.0 )
                    {
                        boo = NL_TRUE;
                        kk = 1;
                    }

                    if( boo )
                    {
                        N_DistPtPt( D[0], Q[kk], &d1 );

                        if( d1 LE NL_MTOL AND d1 LE NL_PTOL AND d1 LT 0.2 *d2 )
                            boo = NL_FALSE;
                    }

                    if( boo )
                        N_CopyPt( Q[kk], &QQ );
                    else
                        N_Combine2Pts( 1.0, D[0], 0.2 *d2 / mag, D[1], &QQ );
                }

                if( rat ) /* make QQ the middle control point */
                {
                    N_CPtGetW( Pw[n], &d1 );
                    d1 = 0.5 *( 1.0 + d1 );
                    N_ScalePt( d1, QQ, &QQ );
                }
                N_PtToCPt( QQ, &Qw[n + 1] );

                if( rat )
                    N_CPtSetW( d1, &Qw[n + 1] );

                if( z EQ NL_NOZ )
                    N_CPtSetZ( z, &Qw[n + 1] );

                if( p GT 2 ) /* degree elevate if necessary */
                {
                    for ( ii = 3; ii <= p; ii++ )
                    {
                        d1 = (NL_REAL)ii;
                        N_CopyCPt( Qw[n + ii - 1], &Qw[n + ii] );

                        for ( kk = ii - 1; kk > 0; kk-- )
                        {
                            d2 = (NL_REAL)kk / d1;
                            N_Combine2CPts( 1.0 - d2, Qw[n + kk], d2, Qw[n + kk - 1], &Qw[n + kk] );
                        }
                    }
                }

                for ( ii = 0; ii < m; ii++ )
                    UQ[ii] = UP[ii]; /* load knots */

                mag /= p;

                if( rat )
                {
                    N_CPtGetW( Qw[n], &d1 );
                    N_CPtGetW( Qw[n + 1], &d2 );
                    mag *= (d1 / d2);
                }
                N_DistCptCpt( Qw[n], Qw[n + 1], &d1 );
                du = d1 / mag;

                if( du LT KN_TOL * (UP[m] - UP[0]) )
                    du = KN_TOL * (UP[m] - UP[0]);

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii + m] = UP[m] + du;
            }

            break;

        case NL_CMAX:

            kk = p;
            /*            bin = N_AllocInt2dArray(kk,kk,&SL);     */
            /*            if ( bin EQ NULL )  NL_QUIT;      */
            /*            N_PascalTriRow(bin,kk);              */

            fac[0] = 0.9; /* these control span length of extension */
            fac[1] = 0.6;
            fac[2] = 0.4;
            fac[3] = 0.3;

            if( mult LT p ) /* Insert uu to mult=p */
            {
                N_CrvInitArrays( &curB );
                curptr = &curB;
                error = N_CrvCopy( &curA, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                error = N_CrvInsertKnot( &curB, uu, p - mult, &curB, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
                curptr = &curA;

            N_CrvGetCPtsDegreeAndKnots( curptr, &nr, &Rw, &p, &mr, &UR );

            if( end EQ NL_START )
            {
                for ( ii = 0; ii <= nr; ii++ ) /* load control points from curP */
                {
                    N_CopyCPt( Rw[ii], &Qw[ii + p] );

                    if( z NEQ NL_NOZ AND NOT c3d )
                        N_CPtSetZ( 0.0, &Qw[ii + p] );
                }

                for ( ii = 1; ii <= mr; ii++ )   /* load knots from curP */
                    UQ[ii + p] = UR[ii];

                N_CPtToPtEuclid( Rw[0], &Q[0] ); /* get new start knot */
                N_CPtToPtEuclid( Rw[p], &Q[1] );
                N_DistPtPt( pt, Q[0], &d1 );
                N_DistPtPt( Q[0], Q[1], &d2 );
                ii = NL_MIN( p, 5 );
                d1 = fac[ii - 2] * d1 * (UR[p + 1] - UR[0]);

                if( N_FloatOpIsBad( d1, d2, NL_DIVISION ) )
                    NL_ERROR( NL_INP_ERR );
                du = d1 / d2;

                if( du LT KN_TOL * (UP[m] - UP[0]) )
                    du = KN_TOL * (UP[m] - UP[0]);

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = UP[0] - du;

                dl = du / p;
                dr = p / (UR[p + 1] - UR[0]); /* this loop computes derivatives  */

                for ( ii = 1; ii < p; ii++ )  /* from the right and uses them to */
                {                             /* compute the new control points  */
                    N_CopyCPt( Rw[ii], &Dw );

                    if( ii % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < ii; kk++ ) /* compute the ii-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Rw[kk], &Dw );
                        kp *= -1;
                    }
                    N_ScaleCPt( dr, Dw, &Dw );
                    dr = dr * ((NL_REAL)p - (NL_REAL)ii) / (UR[p + 1] - UR[0]);

                    if( ii % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw, &Qw[p - ii] );
                    N_ScaleCPt( -kp * dl, Qw[p - ii], &Qw[p - ii] );
                    dl = dl * du / ((NL_REAL)p - (NL_REAL)ii);

                    for ( kk = 0; kk < ii; kk++ ) /* compute the (p-ii)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Qw[p - kk], &Qw[p - ii] );
                        kp *= -1;
                    }

                    if( z NEQ NL_NOZ AND NOT c3d )
                        N_CPtSetZ( 0.0, &Qw[p - ii] );
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj < p; jj++ )
                    {
                        N_CPtGetW( Qw[jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }

                    N_CPtGetW( Qw[1], &w ); /* set weight[0] = weight[1] */
                    N_ScaleCPt( w, cpt, &cpt );
                }

                N_CopyCPt( cpt, &Qw[0] );

                /* remove the original start knot to multiplicity 1 */

                error = N_CrvRemoveKnot( curQ, UP[0], p - 1, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                for ( ii = 0; ii <= nr; ii++ ) /* load control points from curP */
                {
                    N_CopyCPt( Rw[ii], &Qw[ii] );

                    if( z NEQ NL_NOZ AND NOT c3d )
                        N_CPtSetZ( 0.0, &Qw[ii] );
                }

                for ( ii = 0; ii < mr; ii++ )     /* load knots from curP */
                    UQ[ii] = UR[ii];

                N_CPtToPtEuclid( Rw[nr], &Q[0] ); /* get new end knot */
                N_CPtToPtEuclid( Rw[nr - p], &Q[1] );
                N_DistPtPt( pt, Q[0], &d1 );
                N_DistPtPt( Q[0], Q[1], &d2 );
                ii = NL_MIN( p, 5 );
                d1 = fac[ii - 2] * d1 * (UR[mr] - UR[mr - p - 1]);

                if( N_FloatOpIsBad( d1, d2, NL_DIVISION ) )
                    NL_ERROR( NL_INP_ERR );
                du = d1 / d2;

                if( du LT KN_TOL * (UP[m] - UP[0]) )
                    du = KN_TOL * (UP[m] - UP[0]);

                for ( ii = 0; ii <= p; ii++ )
                    UQ[mr + ii] = UP[m] + du;

                dl = -p / (UR[mr] - UR[mr - p - 1]);
                dr = du / p;                 /* this loop computes derivatives */

                for ( ii = 1; ii < p; ii++ ) /* from the left and uses them to */
                {                            /* compute the new control points */
                    N_CopyCPt( Rw[nr - ii], &Dw );

                    if( ii % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < ii; kk++ ) /* compute the ii-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Rw[nr - kk], &Dw );
                        kp *= -1;
                    }
                    N_ScaleCPt( dl, Dw, &Dw );
                    dl = -dl * ((NL_REAL)p - (NL_REAL)ii) / (UR[mr] - UR[mr - p - 1]);

                    if( ii % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw, &Qw[nr + ii] );
                    N_ScaleCPt( dr, Qw[nr + ii], &Qw[nr + ii] );
                    dr = dr * du / ((NL_REAL)p - (NL_REAL)ii);

                    for ( kk = 0; kk < ii; kk++ ) /* compute the (nr+ii)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Qw[nr + kk], &Qw[nr + ii] );
                        kp *= -1;
                    }

                    if( z NEQ NL_NOZ AND NOT c3d )
                        N_CPtSetZ( 0.0, &Qw[nr + ii] );
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj < p; jj++ )
                    {
                        N_CPtGetW( Qw[nr + jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }

                    N_CPtGetW( Qw[nr + p - 1], &w ); /* set wt[nr+p] = wt[nr+p-1] */
                    N_ScaleCPt( w, cpt, &cpt );
                }

                N_CopyCPt( cpt, &Qw[nr + p] );

                /* remove the original end knot to multiplicity 1 */

                error = N_CrvRemoveKnot( curQ, UP[m], p - 1, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( mult LT p )
            { /* remove the additional instances of uu */
                error = N_CrvRemoveKnot( curQ, uu, p - mult, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            break;
    }

    /* If extension is in place, kill old curve */
    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvExtendToPt */

#endif // NLIB_UNUSED

// GWC -Begin method to be made obsolete as soon as we can update its use in HWNLibSolidLoob::FixUVHoles
/*******************************************************************//**
   DESCRIPTION:

     This curve routine modifies a curve so that an  endpoint  passes
     through a given point.  Either the start or end can be modified.
     Various criteria are available to control the segment  of  curve
     that is affected by the endpoint modification. Continuity can be 
     controlled via options.  The largest deviation between  original 
     and modified curves is at the affected endpoint. A typical call-
     ing example is:

       NL_CURVE     curP, curQ;
       NL_POINT     pt, newend;
       NL_REAL      dp;
       NL_STACKS    SP, SQ;
       ...
       (define curP and choose newend and dp or pt);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvModifyEndPt(&curP,newend,NL_START,dp,NL_RELATIVE,pt,NL_C1,&curQ,&SP,&SQ);
       N_CrvModifyEndPt(&curP,newend,NL_END,dp,NL_NOTUSED,pt,NL_C2,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's  
     knot vector and polygon objects.


   ACCESS:
   
     curP   , input  ,  NURBS curve
     newend , input  ,  New endpoint of curve
     end    , input  ,  Flag:
                         NL_START: The curve is modified so that newend is
                                its new start point
                         NL_END  : The curve is modified so that newend is
                                its new end point
     dp     , input  ,  A distance,  or a parameter value,  or not used
                        (see flg below).  This controls what portion of
                        the curve is modified, inward from the affected
                        endpoint
     flg    , input  ,  Flag:
                         NL_ABSOLUTE: dp is an absolute distance  (approx.
                                   arc length inward)
                         NL_RELATIVE: dp is scaled by the curve's arc 
                                   length to yield the desired distance
                         NL_PARVALUE: dp is a parameter value.  The  curve
                                   will change inward to this  location
                         NL_NOTUSED : dp is not used (pt is used  to  com-
                                   pute a parameter value)
     pt     , input  ,  A point on (or close to) the curve which yields
                        a parameter value (via projection) to bound the
                        portion of curve change inward  (used only if flg
                        = NL_NOTUSED)
     cont   , input  ,  Flag:
                         NL_C1  : The modified portion will join the unmod-
                               ified portion with at least NL_C1 continuity
                               (assuming degree > 1  and  the continuity
                               there was originally at least NL_C1)
                         NL_C2  : The modified portion will join the unmod-
                               ified portion with at least NL_C2 continuity
                               (assuming degree > 2  and  the continuity
                               there was originally at least NL_C2)
                         NL_CMAX: The modified portion will join the unmod-
                               ified portion  with  degree-1  continuity
                               (assuming the continuity there was origi-
                               nally at least degree-1)

     curQ   , output ,  Curve after modification
     SP     , input  ,  curP's stack
     SQ     , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_CrvModifyEndPt
 (NL_CURVE  * curP,    // in : input NURBS curve to modify
  NL_POINT    newend,  // in : New endPoint for curve
  NL_FLAG     end,     // in : NL_START = modify Curve start
                       //      NL_END   = modify Curve end
  NL_REAL     dp,      // in : input val used as specified by flg
  NL_FLAG     flg,     // in : NL_ABSOLUTE: modified region arc length - dp is dist
                       //      NL_RELATIVE: modified reg percent arc length - dp from 0 to 1 
                       //      NL_PARVALUE: modified reg transition at param - dp is param
                       //      NL_NOTUSED : modified reg transition closest to input pt
  NL_POINT    pt,      // in : Pt close to curP definiting modified reg transition pt. used when flg==NL_NOTUSED
  NL_FLAG     cont,    // in : NL_C1  : transition to modified region at least C1
                       //      NL_C2  : transition to modified region at least C2
                       //      NL_CMAX: transition to modified region with degree-1 continuity
  NL_CURVE  * curQ,    // out: Curve after modification
  NL_STACKS * SP,      // in : curP's stack
  NL_STACKS * SQ )     // in : curQ's stack
{
  // locals
  NL_PRIVATE NL_STRING rname = _T("N_CrvModifyEndPt");
  NL_PRIVATE NL_REAL   toc   = 1.0e-6;
  NL_FLAG       error = NL_NO, Qret;
  NL_INDEX      n, m, ii, jj=0, mult, mr;
  NL_DEGREE     p, pq;
  NL_CURVE      curA, curB;
  NL_REAL       *UP, *UQ, *UR, top, dd, ldst, d1, uu, dl, dr, ul, ur, du, x, y, z, w, xq, yq, zq;
  NL_BOOLEAN    rat, c3d;
  NL_POINT      Q;
  NL_CPOINT     *Pw, *Qw, Rw;
  NL_KNOTVECTOR knt;
  NL_STACKS     SL;
  
  /* Start NURBS */
  N_InitNurbs( &SL );
  
  /* Get local notation */
  N_CrvGetCPtsDegreeAndKnots( curP,   /* in : target curve                                                    */
                             &n,      /* in : Highest index in control point array                            */
                             &Pw,     /* out: array of control points                                         */
                             &p,      /* out: curve degree                                                    */
                             &m,      /* out: Highest index in KnotArray                                      */
                             &UP );   /* out: array of knots (multiple knots are represented multiple times ) */
  
  /* Get uu parameter value curve change boundary */
  ldst = dp;
  du = 0.05;

  // switch on flg to pick uu = curve change param boundary value
  switch( flg )
    {
      case NL_PARVALUE: { uu = dp; }
                        break;
  
      case NL_RELATIVE: { if(dp LE 0.0 || dp GE 1.0 )
                            { NL_ERROR( NL_INP_ERR ) ; }
                          error = N_CrvArcLength( curP, UP[0], UP[m], 0.01, NL_RELATIVE, &d1 );
                          
                          if( error EQ NL_YES )
                              NL_OUT;
                          du = ldst;
                          ldst *= d1;
                        } // end case NL_RELATIVE
  
      case NL_ABSOLUTE: { if( ldst LE NL_MTOL AND ldst LE NL_PTOL )
                            { NL_ERROR( NL_INP_ERR ); }
                          
                          /* Get parm value that corresponds to segment of length = ldst */
                          
                          du = du * (UP[m] - UP[0]);
                          uu = UP[0] - 10.0;
                          
                          // start branch
                          if( end EQ NL_START ) /* binary search */
                            {
                              ur = UP[0];
                              dr = 0.0;
                          
                              while( NL_YES )
                                {
                                  ul = ur;
                                  dl = dr;
                                  ur = ur + du;
                          
                                  if( ur GT UP[m] )
                                      ur = UP[m];
                                  error = N_CrvArcLength( curP, UP[0], ur, 0.005, NL_RELATIVE, &dr );
                          
                                  if( error EQ NL_YES )
                                      NL_OUT;
                          
                                  /* Note: expensive and unnecessary: use multiplication instead of division.
                                  ** if( N_FloatOpIsBad( fabs( ldst - dr ), ldst, NL_DIVISION ) )
                                  **     NL_ERROR( NL_NUM_ERR );
                                  */
                                /*  if( fabs( ldst - dr ) / ldst LT 0.05 OR ur EQ UP[m] ) */
                                  if( fabs( ldst - dr ) LT ldst * 0.05 OR ur EQ UP[m] )
                                    {
                                      uu = ur;
                                      break;
                                    }
                          
                                  if( dr GT ldst )
                                      break;
                                }
                          
                              if( uu LT UP[0] )
                                {
                                  for ( ii = 0; ii < 4; ii++ )
                                    {
                                      uu = ul + (ldst - dl) * ((ur - ul) / (dr - dl));
                                      error = N_CrvArcLength( curP, UP[0], uu, 0.005, NL_RELATIVE, &dd );
                          
                                      if( error EQ NL_YES )
                                          NL_OUT;
                          
                                    /*  if( fabs( ldst - dd ) / ldst LT 0.05 ) */
                                      if( fabs( ldst - dd ) LT ldst * 0.05 )
                                          break;
                          
                                      if( dd GT ldst )
                                        {
                                          ur = uu;
                                          dr = dd;
                                        }
                                      else
                                        {
                                          ul = uu;
                                          dl = dd;
                                        }
                                    }
                                }
                            }
                          else
                            {
                              ul = UP[m];
                              dl = 0.0;
                          
                              while( NL_YES )
                                {
                                  ur = ul;
                                  dr = dl;
                                  ul = ul - du;
                          
                                  if( ul LT UP[0] )
                                      ul = UP[0];
                                  error = N_CrvArcLength( curP, ul, UP[m], 0.005, NL_RELATIVE, &dl );
                          
                                  if( error EQ NL_YES )
                                      NL_OUT;
                          
                                  /* if( N_FloatOpIsBad( fabs( ldst - dl ), ldst, NL_DIVISION ) )
                                  **    NL_ERROR( NL_NUM_ERR );
                                  */
                                /*  if( fabs( ldst - dl ) / ldst LT 0.05 OR ul EQ UP[0] ) */
                                  if( fabs( ldst - dl ) / ldst LT 0.05 OR ul EQ UP[0] )
                                    {
                                      uu = ul;
                                      break;
                                    }
                          
                                  if( dl GT ldst )
                                      break;
                                }
                          
                              if( uu LT UP[0] )
                                {
                                  for ( ii = 0; ii < 4; ii++ )
                                    {
                                      uu = ur - (ldst - dr) * ((ur - ul) / (dl - dr));
                                      error = N_CrvArcLength( curP, uu, UP[m], 0.005, NL_RELATIVE, &dd );
                          
                                      if( error EQ NL_YES )
                                          NL_OUT;
                          
                                  /*  if( fabs( ldst - dd ) / ldst LT 0.05 ) */
                                      if( fabs( ldst - dd ) LT ldst * 0.05 )
                                          break;
                          
                                      if( dd GT ldst )
                                        {
                                          ul = uu;
                                          dl = dd;
                                        }
                                      else
                                        {
                                          ur = uu;
                                          dr = dd;
                                        }
                                    }
                                }
                            }
                        } // end case NL_ABSOLUTE
                        break;
  
      case NL_NOTUSED : { top = 0.5 *( NL_MTOL + NL_PTOL ); /* don't know what kind of curve it is */
                         
                          N_CrvClosestPtMultiple( curP, pt, -1.0, top, toc, (NL_GCPTEMP *)NULL, &uu, &Q, &Qret, &SL );
                          
                          if( Qret EQ NL_NO )
                              NL_ERROR( NL_NUM_ERR );
                        } // end case NL_NOTUSED
                        break;

      default         : NL_ERROR( NL_INP_ERR );
    } // end switch on flg to find transition param, uu
  
  /* Now find closest knot */
  dd = NL_BIGD;

  // for every knot (let jj = knot index closest to tgt uu transition param value)
  for ( ii = p; ii <= m - p; ii++ )
    {
      // when current knot is closer
      if( fabs( UP[ii] - uu )LT dd )
        {
          // save its distance and knot index value
          dd = fabs( UP[ii] - uu );
          jj = ii;
        }
    } // end iter knots looking for closest knot to uu

  // snap uu to within tol knot values
  if( dd LT KN_TOL * (UP[m] - UP[0]) )
    { uu = UP[jj]; }

  // move uu off of end param values and set mult if uu happens to be on a knot value
  mult = 0;
  if     ( jj EQ p )     { if( uu EQ UP[0] ) { uu = UP[0] + KN_TOL * (UP[m] - UP[0]); }}
  else if( jj EQ m - p ) { if( uu EQ UP[m] ) { uu = UP[m] - KN_TOL * (UP[m] - UP[0]); }}
  else if( uu EQ UP[jj]) { mult = 1 ; while( uu EQ UP[jj + mult] ) { mult += 1; } }
  
  /* A new curve is formed which is zero to uu, and then transitions      */
  /* to newend-P[n] (or analogous if end == NL_START). This curve is then */
  /* added to the original.                                               */

  // pick initial degree of displacement curve, curQ
  pq =   (cont == NL_C1) ? 2
       : (cont == NL_C2) ? 3
       : p ;

  // limit displacement curve degree to input curve degree
  if( pq GT p )
      pq = p;
  
  /* Copy input curP into curA */
  curA = *curP;

  // when curP is same as curQ - set curP memory for displacement function
  if( curP EQ curQ )
    {
      // size curP memory for displacement curve
      error = N_AllocCrvArrays(curP,         // i/o: set this NURBs crv size members and internal arrays
                               pq + 1,       // in : Highest index in control point array
                               pq,           // in : Degree of the curve
                               2 * (pq + 1), // in : Highest index in knot vector array
                               SP );         // in : cur's stack
  
      if( error EQ NL_YES )
          NL_OUT;

      // let Qw, UQ = CtrlPts and Knots of displacement function in curP
      N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
  else // else set curQ memory for displacement function
    {
      error = N_CrvSizeArrays(curQ,         /* in : NURBS curve to be created    */
                              pq + 1,       /* in : highest control point index  */
                              pq,           /* in : degree                       */
                              2 * (pq + 1), /* in : highest knot index           */
                              rname,        /* in : name of calling routine      */
                              SQ );         /* in : new object stack             */
  
      if( error EQ NL_YES )
          NL_OUT;

      // let Qw, UQ = CtrlPts and Knots of displacement function in curQ
      N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }
  
  /* Compute the new control points and knots defining the extension */

  // set displacement curve transition knot value
  UQ[pq + 1] = uu;

  // set displacement curve end knot value
  for ( ii = 0; ii <= pq; ii++ )
    {
      UQ[ii] = UP[0];
      UQ[ii + pq + 2] = UP[m];
    }

  // Rw = NonRational CPt at origin
  N_CPtFromWxWyWz( 0.0, 0.0, 0.0, NL_NOW, &Rw );

  // make a displacment curve (all cpts 0 except for 1 end with Cpt[end] = CurrentEnd - NewEnd)
  if( end EQ NL_START )
    {
      for ( ii = 1; ii <= pq + 1; ii++ )
          N_CopyCPt( Rw, &Qw[ii] );     // Qw[1],Q[2],..Q[pq+1] = Rw = 0,0,0 cartesian
      N_CPtToPtEuclid( Pw[0], &Q );     // Q = Qcartesian = Pw[0]
      N_VectorDir(Q,                    // Q = Vec(newend - Pw[0] /* in : Ps  of Vec = Pe - Ps */
                  newend,               //                        /* in : Pe  of Vec = Pe - Ps */
                  &Q );                 //                        /* out: Vec of Vec = Pe - Ps */
      N_PtToCPt( Q, &Qw[0] );           // Qw[0] = newend - Pw[0] = displacement at end
    }
  else // end EQ NL_END
    {
      for ( ii = 0; ii <= pq; ii++ )
          N_CopyCPt( Rw, &Qw[ii] );     // Qw[0],Q[1],..Q[pq] = Rw = 0,0,0 cartesian
      N_CPtToPtEuclid( Pw[n], &Q );     // Q = Qcartesian = Pw[n]
      N_VectorDir( Q,                   // Q = Vec(newend - Pw[n] /* in : Ps  of Vec = Pe - Ps */
                   newend,              //                        /* in : Pe  of Vec = Pe - Ps */
                   &Q );                //                        /* out: Vec of Vec = Pe - Ps */
      N_PtToCPt( Q, &Qw[pq + 1] );      // Qw[pq+1] = newend - Pw[n] = displacement at end
    }
  
  /* when needed - elevate displacment curve, curQ, degree to input curve degree (making curves homogeneous) */
  if( pq LT p )
    {
      error = N_CrvElevateDegree( curQ, p - pq, curQ, SQ, SQ );
  
      if( error EQ NL_YES )
          NL_OUT;
    }
  
  /* when needed - add knots to displacement curve, curQ, to make homogenous withinput curve */
  if( n GT p )
    {
      UR = N_AllocReal1dArray( n - p, &SL );
  
      if( UR EQ NULL )
          NL_QUIT;
  
      mr = -1;

      // build knot array to add to curQ displacement curve (knts = orig curP->knts not coincident with displacment curQ knots)
      for ( ii = p + 1; ii < m - p; ii++ )
          if( UP[ii] NEQ uu )
            {
              mr += 1;
              UR[mr] = UP[ii];
            }
          else
            {
              for ( jj = mult; jj > p - pq + 1; jj-- )
                {
                  mr += 1;
                  UR[mr] = uu;
                }
              ii += (mult - 1);
            }

      // insert knt knots into curQ KnotVector
      if( mr GE 0 )
        {
          N_KnotVectorFromRealArray(&knt,  // in : input KNOTVECTOR to build
                                    UR,    // out: set knt->U = U, knot array
                                    mr );  // out: set knt->m = m, highest index in U
          error = N_CrvRefine( curQ,  // in : NURBS curve
                              &knt,   // in : New knot vector
                               curQ,  // out: Curve after adding knots in knx to curP (ok if curP == curA)
                               SQ,    // in : curP's stack
                               SQ );  // in : curQ's stack
  
          if( error EQ NL_YES )
              NL_OUT;
        }
  }
  
  /* Set curB (original curve) correctly and insert uu into */
  /* curB if necessary                                      */
  
  if( p - pq + 1 GT mult )
  {
      N_CrvInitArrays( &curB );
      error = N_CrvInsertKnot( &curA, uu, p - pq + 1 - mult, &curB, &SL, &SL );
  
      if( error EQ NL_YES )
          NL_OUT;
  }
  else
      curB = curA;

  // arrive here when:
  //  curQ = displacement function
  //  curB = original curve
  //  curQ and curB are compatible
  
  /* We now just need to add the control points of curB and curQ. */
  /* Dealing with the NL_NOZ and NL_NOW options are a bit tricky.     */
  
  c3d = N_CrvIs3d( &curB );
  rat = N_IsCrvRat( &curB );
  N_PtToXYZ( newend, &x, &y, &z );
  
  if( z NEQ 0.0 AND( NOT c3d ) )
      c3d = NL_TRUE;
  w = NL_NOW;
  
  N_CrvGetCPts( &curB, &n, &Pw );
  N_CrvGetCPts( curQ, &n, &Qw );
  
  for ( ii = 0; ii <= n; ii++ )
  {
      N_CPtToXYZ( Pw[ii], &x, &y, &z );
      N_CPtToXYZ( Qw[ii], &xq, &yq, &zq );
      x += xq;
      y += yq;
  
      if( c3d )
          z += zq;
      else
          z = NL_NOZ;
  
      if( rat )
      {
          N_CPtGetW( Pw[ii], &w );
          x *= w;
          y *= w;
  
          if( c3d )
              z *= w;
      }
      N_CPtFromWxWyWz( x, y, z, w, &Qw[ii] );
  }
  
  /* If extension is in place, kill old curve */
  if( curP EQ curQ )
      N_FreeCrv( &curA, SP );
  
  /* End NURBS and Exit */
  EXIT:
  N_EndNurbs( &SL );

  // all done
  return (error) ;

} /* end N_CrvModifyEndPt */
// GWC - end of method to be made obsolete as soon as we can update its use in HWNLibSolidLoob::FixUVHoles

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the  derivative of a NURBS curve with respect
     to a knot. A typical calling example is:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_INDEX      k;
       NL_POINT      Ck;
       ...
       (define cur, get u and k);
       ...
       N_CrvEvalDerivAtKnot(&cur,k,u,NL_LEFT,NL_RIGHT,&Ck);


   ACCESS:
   
     cur , input  ,  NURBS curve
     k   , input  ,  Index of knot, i.e. the derivative  with respect to
                     u_k is computed
     u   , input  ,  Parameter value 
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k-1)
                       NL_RIGHT: right  derivative. NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k+1)
     flp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     Ck  , output ,  Derivative computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalDerivAtKnot( NL_CURVE *cur, NL_INDEX k, NL_PARAMETER u, NL_FLAG flk, NL_FLAG flp, NL_POINT *Ck )
{
    NL_FLAG error = NL_NO;
    NL_INDEX i, kk;
    NL_DEGREE p;
    NL_REAL *Bk, *U;
    NL_KNOTVECTOR *knt;
    NL_EPOLYGON ppl;
    NL_POINT *P;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation */
    N_CrvGetDegree( cur, &p );
    N_CrvGetKnotVector( cur, &knt );
    N_CrvGetKnots( cur, &i, &U );

    /* Compute basis function derivatives */

    Bk = N_AllocReal1dArray( p + 1, &SL );

    if( Bk EQ NULL )
        NL_QUIT;

    if( N_IsCrvRat( cur ) )
    {
        error = N_CrvRatBasisKnotDeriv( cur, k, u, flk, flp, Bk );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_BasisKnotDerivs( knt, k, p, u, flk, flp, Bk );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute derivatives */

    kk = k;

    if( N_IsCrvRat( cur ) )
    {
        if( flk EQ NL_LEFT AND u GT U[k] )
            while( U[kk]EQ U[kk + 1] )
                kk++;

        if( flk EQ NL_RIGHT AND u LT U[k] )
            while( U[kk]EQ U[kk - 1] )
                kk--;
    }

    error = N_CrvGetEPolygon( cur, kk - p - 1, kk, &ppl, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_EPolygonGetPts( &ppl, &i, &P );

    N_CopyPt( NL_ZERO, Ck );

    for ( i = 0; i <= p + 1; i++ )
    {
        N_VectorBlendPt( Bk[i], P[i], Ck );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvEvalDerivAtKnot */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes an upper bound on the second derivative of a 
     curve. The approach is based on sampling the Bezier segments at the 
     nodes. A typical calling example is:

       NL_CURVE  cur;
       NL_REAL   Muu;
       ...
       (define cur);
       ...
       N_CrvGetMax2ndDeriv(&cur,&Muu);

     THIS ROUTINE PROVIDES A  REASONABLE APPROXIMATION TO THE NL_MAXIMUM OF
     THE SECOND NL_DERIVATIVE. FOR A PRECISE ALTHOUGH  SIGNIFICANTLY SLOWER
     APPROACH SEE N_CrvGetMaxSecondDeriv.


   ACCESS:
   
     cur , input  ,  NURBS curve
     Muu , output ,  Maximum 2nd derivative magnitude


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetMax2ndDeriv( NL_CURVE *cur, NL_REAL *Muu )
{
    NL_FLAG error = NL_NO;
    NL_DEGREE p;
    NL_INDEX i, k, r, a, al;
    NL_REAL *U, *u, ui, duu, muu;
    NL_POINT *CD;
    NL_CURVE ** bez;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get locals and decompose into Bezier segments */
    N_CrvGetDegree( cur, &p );

    error = N_CrvDecomposeBez( cur, &bez, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */
    CD = N_AllocPt1dArray( 2, &SL );

    if( CD EQ NULL )
        NL_QUIT;

    u = N_AllocReal1dArray( p, &SL );

    if( u EQ NULL )
        NL_QUIT;

    duu = 0.0;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            al = 0;
        else
            al = 1;

        N_CrvGetKnots( bez[i], &r, &U );

        ui = (U[r] - U[0]) / p;
        u[0] = U[0];
        u[p] = U[r];

        for ( a = 1; a <= p - 1; a++ )
            u[a] = U[0] + a * ui;

        for ( a = al; a <= p; a++ )
        {
            error = N_CrvDerivs( bez[i], u[a], NL_LEFT, 2, CD );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtMagnitude( CD[2], &muu );

            if( muu GT duu )
                duu = muu;
        }
    }

    *Muu = duu;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetMax2ndDeriv */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes the 1st and the 2nd derivatives of the offset 
     curve of a given planar NURBS curve. The derivatives are defined in 
     terms of the derivatives of the  base curve, its curvature, and the 
     offset distance. Discontinuous  curves can be handled by  passing a 
     NL_LEFT/NL_RIGHT flag. A typical calling example:

       NL_CURVE      cur;
       NL_PARAMETER  u;
       NL_REAL       d;
       NL_POINT      OD[3];
       NL_VECTOR     N;
       ...
       (define cur; get u and N);
       ...
       N_CrvPlanarOffsetGetDeriv(&cur,u,d,N,NL_LEFT,OD);

     THE  DERIVATIVES ARE  STORED IN  OD[1]  AND  OD[2]. MEMORY  MUST BE 
     ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     cur , input  ,  Planar NURBS curve
     u   , input  ,  Parameter value 
     d   , input  ,  Offset  distance
     N   , input  ,  Normal to the plane of  curve. The  curve is offset 
                     in the direction of  N x T, where T is  the tangent 
                     to cur at u
     ufl , input  ,  Flag:
                       NL_LEFT : u is in  [u[j],u[j+1]) (NL_RIGHT  derivatives
                              used)
                       NL_RIGHT: u is  in  (u[j],u[j+1]] (NL_LEFT  derivatives
                              used)
     OD  , output ,  Offset derivatives at u; OD[1] and OD[2] are  first
                     and second derivatives


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvPlanarOffsetGetDeriv( NL_CURVE *cur, NL_PARAMETER u, NL_REAL d, NL_VECTOR N, NL_FLAG ufl, NL_POINT *OD )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvPlanarOffsetGetDeriv");
    NL_FLAG error = NL_NO;
    NL_REAL num, den, kap, kpp, f1, f2, sgn;
    NL_POINT D[3];
    NL_VECTOR B;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Get derivatives, the curvature and its derivative */
    error = N_CrvDerivs( cur, u, ufl, 2, D );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCross( D[1], D[2], &B );
    N_VectorMagnitude( B, &num );
    N_VectorMagnitude( D[1], &den );

    den = den * den * den;

    if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    kap = num / den;

    N_VectorDot( B, N, &den );

    if( den GT 0.0 )
        sgn = -1.0;
    else
        sgn = 1.0;

    error = N_CrvGetCurvatureDeriv( cur, u, ufl, &kpp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get offset derivatives */
    f1 = 1.0 + sgn * kap * d;
    N_VectorScale( D[1], f1, &OD[1] );
    f2 = sgn * kpp * d;
    N_VectorCombine( f1, D[2], f2, D[1], &OD[2] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvPlanarOffsetGetDeriv */



/*******************************************************************//**


   DESCRIPTION:

     This routine computes an upper bound on the second  derivative of a
     plane offset curve of a  given planar curve. The  approach is based  
     on sampling the Bezier segments. A typical calling example is:

       NL_CURVE   cur;
       NL_REAL    d, Muu;
       NL_VECTOR  N;
       ...
       (define cur; get offset distance d and normal N);
       ...
       N_CrvOffsetGetMax2ndDeriv(&cur,d,N,&Muu);

     THIS ROUTINE PROVIDES A  REASONABLE APPROXIMATION TO THE NL_MAXIMUM OF
     THE SECOND  NL_DERIVATIVE. A  PRECISE  ALTHOUGH  SIGNIFICANTLY  SLOWER
     APPROACH CAN BE OBTAINED BY USING SYMBOLIC OPERATORS.


   ACCESS:
   
     cur , input  ,  Planar NURBS curve
     d   , input  ,  Offset distance
     N   , input  ,  Normal to the plane of cur
     Muu , output ,  Maximum 2nd derivative magnitude


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvOffsetGetMax2ndDeriv( NL_CURVE *cur, NL_REAL d, NL_VECTOR N, NL_REAL *Muu )
{
    NL_FLAG error = NL_NO;
    NL_DEGREE p;
    NL_INDEX i, k, r, a, al, mu;
    NL_REAL *U, *u, ui, duu, muu;
    NL_POINT OD[3];
    NL_CURVE ** bez;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get locals and decompose into Bezier segments */
    N_CrvGetDegree( cur, &p );

    error = N_CrvDecomposeBez( cur, &bez, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */
    duu = 0.0;
    mu = 10 * p;

    u = N_AllocReal1dArray( mu, &SL );

    if( u EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= k; i++ )
    {
        if( i EQ 0 )
            al = 0;
        else
            al = 1;

        N_CrvGetKnots( bez[i], &r, &U );

        ui = (U[r] - U[0]) / mu;
        u[0] = U[0];
        u[mu] = U[r];

        for ( a = 1; a <= mu - 1; a++ )
            u[a] = U[0] + a * ui;

        for ( a = al; a <= mu; a++ )
        {
            error = N_CrvPlanarOffsetGetDeriv( bez[i], u[a], d, N, NL_LEFT, OD );

            if( error EQ NL_YES )
                NL_OUT;

            N_PtMagnitude( OD[2], &muu );

            if( muu GT duu )
                duu = muu;
        }
    }

    *Muu = duu;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvOffsetGetMax2ndDeriv */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the 1st and the 2nd derivatives of the offset 
     curve of a given NURBS curve. The offset is computed as

         C_o(u) = C(u) + d(u)*V(u)

     where C(u) is the original  curve, d(u) is a distance function, and
     V(u) is a direction curve. A typical calling example:

       NL_CURVE      cur, curV;
       NL_CFUN       cfnd
       NL_PARAMETER  u;
       NL_POINT      OD[3];
       ...
       (define cur; get cfnd and curV);
       ...
       N_CrvOffsetGetDeriv(&cur,u,&cfnd,&curV,NL_LEFT,OD);

     THE  DERIVATIVES ARE  STORED IN  OD[1]  AND  OD[2]. MEMORY  MUST BE 
     ALLOCATED IN  THE  CALLING  ROUTINE!  cur,  cfnd AND  curV  MUST BE 
     DEFINED NL_OVER THE SAME NL_PARAMETER SPAN.


   ACCESS:
   
     cur  , input  ,  NURBS curve (not necessarily planar!)
     u    , input  ,  Parameter value 
     cfnd , input  ,  Distance  function; cfnd(u) is the offset distance
                      at u
     curV , input  ,  Direction curve; curV(u) is the direction in which
                      cur is to be offset at u
     ufl  , input  ,  Flag:
                        NL_LEFT : u is in [u[j],u[j+1]) (NL_RIGHT  derivatives
                               used)
                        NL_RIGHT: u is  in (u[j],u[j+1]] (NL_LEFT  derivatives
                               used)
     OD   , output ,  Offset derivatives at u; OD[1] and OD[2] are first
                      and second derivatives


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvOffsetGetDeriv( NL_CURVE *cur, NL_PARAMETER u, NL_CFUN *cfnd, NL_CURVE *curV, NL_FLAG ufl, NL_POINT *OD )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvOffsetGetDeriv");
    NL_FLAG error = NL_NO;
    NL_INDEX mc, mf, mv;
    NL_REAL *UC, *UF, *UV, fd[3];
    NL_POINT CD[3], VD[3];
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get derivatives of base entities */
    N_CrvGetKnots( cur, &mc, &UC );
    N_CFuncGetKnots( cfnd, &mf, &UF );
    N_CrvGetKnots( curV, &mv, &UV );

    if( UC[0]NEQ UF[0]OR UC[0]NEQ UV[0] )
        NL_ERROR( NL_INP_ERR );

    if( UC[mc]NEQ UF[mf]OR UC[mc]NEQ UV[mv] )
        NL_ERROR( NL_INP_ERR );

    error = N_CrvDerivs( cur, u, ufl, 2, CD );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvDerivs( curV, u, ufl, 2, VD );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CFuncDerivs( cfnd, u, ufl, 2, fd );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get offset derivatives */

    N_VectorPtAlongVector( CD[0], fd[0], VD[0], &OD[0] );

    N_VectorPtAlongVector( CD[1], fd[1], VD[0], &OD[1] );
    N_VectorPtAlongVector( OD[1], fd[0], VD[1], &OD[1] );

    fd[1] *= 2.0;
    N_VectorPtAlongVector( CD[2], fd[2], VD[0], &OD[2] );
    N_VectorPtAlongVector( OD[2], fd[1], VD[1], &OD[2] );
    N_VectorPtAlongVector( OD[2], fd[0], VD[2], &OD[2] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvOffsetGetDeriv */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This curve routine extends a Nurbs curve a given parametric dist-
     ance.  The start or end can be extended.  Continuity can be cont-
     rolled via options. After extension, the original parameter range
     still maps to the original curve (i.e. the parameter range is ex-
     tended). A typical calling example is:

       NL_CURVE     curP, curQ;
       NL_REAL      par;
       NL_STACKS    SP, SQ;
       ...
       (define curP and choose par);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvExtendByParamDist(&curP,par,NL_START,NL_CMAX,&curQ,&SP,&SQ);
       N_CrvExtendByParamDist(&curP,par,NL_END,NL_G1,&curP,&SP,&SP);

     If memory is  available, curQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in curQ's
     knot vector and polygon objects.

     We limit to 10 levels of recursion.


   ACCESS:
   
     curP , input  ,  NURBS curve
     par  , input  ,  The parameter value  defining  the  new  extended
                      boundary
     end  , input  ,  Flag:
                       NL_START: The curve is extended back from  its  start
                              point
                       NL_END  : The curve is extended forward from its  end
                              point
     cont , input  ,  Flag:
                       NL_G1  : curP is extended  by a linear segment.  curQ
                             is at least NL_G1 (possibly NL_C1) where the  ext-
                             ension joins the original curve
                       NL_G2  : curP is extended by reflection,  yielding  a 
                             NL_G2 continuous extension
                       NL_CMAX: the extension yields infinite (C) continuity
                             at the join point (no knot there)
     curQ , output ,  Curve after extension
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvExtendByParamDist /* eff: extend curve interval range */
( NL_CURVE *curP,              /* in : curve to be extended */
NL_REAL par,                   /* in : new interval parameter value */
NL_FLAG end,                   /* in : end to extend, oneof: NL_START/NL_END */
NL_FLAG cont,                  /* in : continuity of extension, oneof: NL_G1/NL_G2/NL_CMAX */
NL_CURVE *curQ,                /* out: the extended curve */
NL_STACKS *SP,                 /* in : memory stack for curP */
NL_STACKS *SQ )                /* in : memory stack for new curQ */                             
{
    /* NL_PRIVATE NL_STRING rname = _T("N_CrvExtendByParamDist"); */
    return (ST_curxcrr( curP, par, end, cont, curQ, SP, SQ, 0 ));
} /* end    */



/*******************************************************************//**
***********************************************************************/
NL_FLAG ST_curxcrr (   /* eff: extend curve interval range */
    NL_CURVE *curP,    /* in : curve to be extended */
    NL_REAL par,       /* in : new interval parameter value */
    NL_FLAG end,       /* in : end to extend, oneof: NL_START/NL_END */
    NL_FLAG cont,      /* in : continuity of extension, oneof: NL_G1/NL_G2/NL_CMAX */
    NL_CURVE *curQ,    /* out: the extended curve */
    NL_STACKS *SP,     /* in : memory stack for curP */
    NL_STACKS *SQ,     /* in : memory stack for new curQ */
    int depth          /* in : recursion depth */
)
{
    NL_PRIVATE NL_STRING rname = _T("ST_curxcrr");
    NL_FLAG error = NL_NO;
    NL_CURVE curA, curB;
    NL_CURVE *curptr; /* use: working input curve, equals curB if curP is modified prior to extension, else equals curP  */

    /*        NL_INTEGER     **bin;      */

    NL_INDEX ii, jj = 0, kk, kp, kq, span = 0, mult = 0;
    NL_INDEX nr, mr; /* use: output curve cpt and knt high index values */
    NL_INDEX mq;     /* use: new curQ high index value */
    NL_INDEX n, m;   /* use: curP point and knot high index values */
    NL_DEGREE p;     /* use: curP degree */
    NL_CPOINT *Dw;
    NL_CPOINT *Qw;   /* use: output curve cpt array: from curP when curP = curQ, else from curQ */
    NL_CPOINT *Rw;   /* use: working input curve cpt array */
    NL_CPOINT *Pw;   /* use: curP cpt array */
    NL_POINT P1, P2;
    NL_VECTOR V;
    NL_PLANE pln;
    NL_KNOTVECTOR *knt; /* use: curP knot vector*/
    NL_BOOLEAN rat;
    NL_REAL *UP;        /* use: curP knot array*/
    NL_REAL *UR;        /* use: working input curve knot array*/
    NL_REAL *UQ;        /* use: output curve knot array*/
    NL_REAL d1, uu = 0.0, dl, dr, dd, mag, wx, wy, wz, w, u2;
    NL_STACKS SL;

    /* Limit recursion to 10 levels */
    if( depth > 10 )
        return (1);

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* Get local notation */
    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    N_CrvGetKnotVector( curP, &knt );

    /* Get mq, new high index of knots */

    /* for g2 extensions*/
    if( cont EQ NL_G2 )
    {
        /* Recurse if par is too big */

        if( (end EQ NL_START AND UP[0] - par GT 0.9213 *( UP[m] - UP[0] ))OR( end EQ NL_END AND par - UP[m]GT 0.9213 *( UP[m] - UP[0] ) ) )
        {
            /* use the recursive version with level control */
            if( end EQ NL_START )
                dd = par + 0.4671 *( UP[m] - UP[0] );
            else
                dd = par - 0.4671 *( UP[m] - UP[0] );

            /* extend the curve domain for this step*/
            error = ST_curxcrr( curP, dd, end, cont, curQ, SP, SQ, depth + 1 );

            if( error EQ NL_YES )
                NL_OUT;

            /* now extend the curve for the full extension*/
            error = ST_curxcrr( curQ, par, end, cont, curQ, SQ, SQ, depth + 1 );
            NL_OUT;
        } /* end large extension check*/

        /* find uu param value equal dist from the extending end as par*/
        /* ???Does uu have to be in the curve interval*/
        if( end EQ NL_START )
            uu = 2.0 *UP[0] - par;
        else
            uu = 2.0 *UP[m] - par;

        /* find the knot closest to the uu value*/
        d1 = NL_BIGD;

        for ( ii = p + 1; ii < m - p; ii++ )
        {
            if( fabs( uu - UP[ii] )LT d1 )
            {
                /* let d1 = min dist from uu to existing knot*/
                d1 = fabs( uu - UP[ii] );
                jj = ii;
            }
        } /* end iter every knot*/

        /* when uu is close to knot jj, snap uu to knot jj*/
        /*   where close :== knot_tolerance scaled by interval length*/
        if( d1 LT KN_TOL * (UP[m] - UP[0]) )
            uu = UP[jj];

        /* when extending the start*/
        if( end EQ NL_START )
        {
            /* find the knt span (and multiplicity) containing uu*/
            error = N_BasisFindSpanAndMult( knt, p, uu, NL_RIGHT, &span, &mult );

            if( error EQ NL_YES )
                NL_OUT;

            /* make sure uu is either the span_top or not close to the span_bottom.*/
            /* when     uu NEQ span_top*/
            /*      and uu is close to span_bottom*/
            /*      and span length is greater then close distance*/
            if( uu NEQ UP[span + 1] )
                if( uu - UP[span]LT KN_TOL *( UP[m] - UP[0] ) )
                    if( UP[span] + KN_TOL *( UP[m] - UP[0] )LT UP[span + 1] )
                    {
                        /* let uu = span_bottom + close distance*/
                        uu = UP[span] + KN_TOL * (UP[m] - UP[0]);
                    }

            /* add knots for new spans equal to number of spans below uu*/
            mq = m + span;
        }
        else /* extend the end*/
        {
            /* find the knt span (and multiplicity) containing uu*/
            error = N_BasisFindSpanAndMult( knt, p, uu, NL_LEFT, &span, &mult );

            if( error EQ NL_YES )
                NL_OUT;

            /* make sure uu is either the span_bottom or not close to the span_top.*/
            /* when     uu NEQ span_bottom*/
            /*      and uu is close to span_top*/
            /*      and span length is greater then close distance*/
            if( uu NEQ UP[span] )
                if( UP[span + 1] - uu LT KN_TOL *( UP[m] - UP[0] ) )
                    if( UP[span + 1] - KN_TOL *( UP[m] - UP[0] )GT UP[span] )
                    {
                        /* let uu = span_top - close distance*/
                        uu = UP[span + 1] - KN_TOL * (UP[m] - UP[0]);
                    }

            /* add knots for new spans equal to number of spans above uu*/
            mq = 2 * m - span - 1;
        } /* end extend upper-end branch*/
    }     /* end g1 extension check*/
    else if( cont EQ NL_CMAX )
    {
        mult = 1;

        if( end EQ NL_START )
        {
            uu = UP[p + 1];

            while( p + mult + 1 LT m )
                if( uu EQ UP[p + mult + 1] )
                    mult += 1;
                else
                    break;
        }
        else
        {
            uu = UP[m - p - 1];

            while( m - p - mult - 1 GT 0 )
                if( uu EQ UP[m - p - mult - 1] )
                    mult += 1;
                else
                    break;
        }

        /* mq = */
        mq = m + 2 * p - mult;
    } /* end NL_CMAX extension branch*/
    else
    {
        /* mq = original knot max index + curve degree (room for one new span)*/
        mq = m + p;
    } /* end G0 extension branch */

    /* See if memory is needed */

    curA = *curP;

    /* when curP and CurQ are the same*/
    if( curP EQ curQ )
    {
        /* alloc output curQ memory for mq highest knot index with degree p on SP stack*/
        /*   note: old Pw and UP pointers are no longer stored in curP but in curA*/
        error = N_AllocCrvArrays( curP, mq - p - 1, p, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        /* get output control point and knot vector pointers*/
        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else /* curP and curQ are different*/
    {
        /* set curQ memory when it is NULL or large enough, else signal error*/
        error = N_CrvSizeArrays( curQ, mq - p - 1, p, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        /* get output control point and knot vector pointers*/
        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Compute the new control points and knots defining the extension */

    switch( cont )
    {
        case NL_G1:
            if( end EQ NL_START )
            {
                N_CPtToPtEuclid( Pw[0], &P1 ); /* compute new start control point */
                N_CPtToPtEuclid( Pw[1], &P2 );
                N_VectorDir( P2, P1, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error EQ NL_YES )
                {
                    N_CPtToPtEuclid( Pw[2], &P2 );
                    N_VectorDir( P2, P1, &V );
                    error = N_VectorNormalize( V, &V, &mag );
                }

                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );
                d1 = mag * ((UP[0] - par) / (UP[p + 1] - UP[0]));
                N_VectorBlendPt( d1, V, &P1 );
                N_PtToXYZ( P1, &wx, &wy, &wz );
                N_CPtGetW( Pw[1], &w );

                if( w NEQ NL_NOW )
                {
                    wx *= w;
                    wy *= w;
                    wz *= w;
                }
                N_CPtToWxWyWz( Pw[0], &d1, &d1, &dd, &d1 );

                if( dd EQ NL_NOZ )
                    wz = NL_NOZ;

                N_CPtFromWxWyWz( wx, wy, wz, w, &Qw[0] );

                dd = (NL_REAL)p;

                for ( ii = 1; ii < p; ii++ ) /* compute p-1 new internal ctrl pts */
                {                            /* (degree elevation)                */
                    d1 = (NL_REAL)ii / dd;
                    N_Combine2CPts( 1.0 - d1, Qw[0], d1, Pw[0], &Qw[ii] );
                }
                kk = p;

                for ( ii = 0; ii <= p; ii++ )
                    UQ[ii] = par; /* get new start knot */
                kq = p + 1;
                kp = 1;
            }
            else
            {
                N_CPtToPtEuclid( Pw[n - 1], &P1 ); /* compute new end control point */
                N_CPtToPtEuclid( Pw[n], &P2 );
                N_VectorDir( P1, P2, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error EQ NL_YES )
                {
                    N_CPtToPtEuclid( Pw[n - 2], &P1 );
                    N_VectorDir( P1, P2, &V );
                    error = N_VectorNormalize( V, &V, &mag );
                }

                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );
                d1 = mag * ((par - UP[m]) / (UP[m] - UP[m - p - 1]));
                N_VectorBlendPt( d1, V, &P2 );
                N_PtToXYZ( P2, &wx, &wy, &wz );
                N_CPtGetW( Pw[n - 1], &w );

                if( w NEQ NL_NOW )
                {
                    wx *= w;
                    wy *= w;
                    wz *= w;
                }
                N_CPtToWxWyWz( Pw[0], &d1, &d1, &dd, &d1 );

                if( dd EQ NL_NOZ )
                    wz = NL_NOZ;

                N_CPtFromWxWyWz( wx, wy, wz, w, &Qw[n + p] );

                dd = (NL_REAL)p;

                for ( ii = 1; ii < p; ii++ ) /* compute p-1 new internal ctrl pts */
                {                            /* (degree elevation)                */
                    d1 = (NL_REAL)ii / dd;
                    N_Combine2CPts( 1.0 - d1, Pw[n], d1, Qw[n + p], &Qw[n + ii] );
                }
                kk = 0;

                for ( ii = 0; ii <= p; ii++ )
                    UQ[m + ii] = par; /* get new end knot */
                kq = 0;
                kp = 0;
            }

            for ( ii = 0; ii <= n; ii++ ) /* load remaining control points */
            {
                N_CopyCPt( Pw[ii], &Qw[kk] );
                kk += 1;
            }

            for ( ii = 1; ii <= m; ii++ ) /* load remaining knots */
                UQ[kq++] = UP[kp++];

            break;

        case NL_G2:
            if( mult LT p ) /* Insert uu to mult=p */
            {
                /* init curB to NULL and use it for the working input curve*/
                N_CrvInitArrays( &curB );
                curptr = &curB;

                /* copy curA=curP into curB*/
                error = N_CrvCopy( &curA, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* insert new knots at uu until multiplicity = degree */
                error = N_CrvInsertKnot( &curB, uu, p - mult, &curB, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                /* increment span count by the number of knots inserted*/
                if( end EQ NL_END )
                    span += (p - mult);
            }
            else /* no need to insert knot branch*/
            {
                /* let working input curve equal curA*/
                /* gwc: Bug Fix curptr needs to be curA not curP*/
                /*      curP has already been resized with uninitialized array memory*/
                /* replaced curptr = *curP;  with the next line*/
                curptr = &curA;
            }

            /* locals - working input curve data*/
            N_CrvGetCPtsDegreeAndKnots( curptr, &nr, &Rw, &p, &mr, &UR );

            if( end EQ NL_START )
            {
                N_CPtToPtEuclid( Pw[0], &P1 ); /* get reflection plane */
                N_CPtToPtEuclid( Pw[1], &P2 );
                N_VectorDir( P2, P1, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );
                N_CreatePlanePtNormal( &pln, P1, V );

                kk = 0; /* load reflected control points */

                for ( ii = span; ii > 0; ii-- )
                {
                    N_CptReflect( Rw[ii], pln, &Qw[kk] );
                    kk += 1;
                }

                dd = 2.0 *UP[0];

                for ( ii = 0; ii <= p; ii++ ) /* load knots for reflected section */
                    UQ[ii] = par;
                kq = p + 1;

                for ( ii = span; ii > p; ii-- )
                    UQ[kq++] = dd - UP[ii];
                kp = 1;
            }                                  /* end extend start end check*/
            else                               /* extending curve-end case*/
            {
                N_CPtToPtEuclid( Pw[n], &P1 ); /* get reflection plane defined by end-curve tangent dir */
                N_CPtToPtEuclid( Pw[n - 1], &P2 );

                /* build end-curve normalized tangent vector*/
                N_VectorDir( P2, P1, &V );
                error = N_VectorNormalize( V, &V, &mag );

                if( error EQ NL_YES )
                    NL_ERROR( NL_GEO_ERR );

                /* make reflection plane; thru end-point, normal to tangent dir*/
                N_CreatePlanePtNormal( &pln, P1, V );

                /* 1st index of output-curves new control points*/
                kk = n + 1; /* load reflected control points */

                /* for every new control-point in the output curve*/
                for ( ii = nr - 1; ii >= span - p; ii-- )
                {
                    /* mirror the cpt about the reflection plane*/
                    N_CptReflect( Rw[ii], pln, &Qw[kk] );
                    kk += 1;
                }

                kk = 0;

                /* set last knot values to clamp curve at new end*/
                for ( ii = 0; ii <= p; ii++ ) /* load knots for reflected section */
                    UQ[mq - p + ii] = par;

                /* for every other new knot*/
                dd = 2.0 *UP[m];
                kq = mq - p - 1;

                for ( ii = span + 1; ii < mr - p; ii++ )
                    /* mirror knot values about old_end_knot_value: new_knot = old_end + (old_end - old_knot)*/
                    UQ[kq--] = dd - UR[ii];

                /* set kq and kp to copy original knots beginning with curve-start*/
                kq = 0;
                kp = 0;
            } /* end extend curve-end case*/

            /* copy original cpt from input to output curve*/
            for ( ii = 0; ii <= n; ii++ ) /* load remaining control points */
            {
                N_CopyCPt( Pw[ii], &Qw[kk] );
                kk += 1;
            }

            /* copy original knot values from*/
            for ( ii = 1; ii <= m; ii++ ) /* load remaining knots */
                UQ[kq++] = UP[kp++];

            break;

        case NL_CMAX:

            kk = p;
            /*            bin = N_AllocInt2dArray(kk,kk,&SL);     */
            /*            if ( bin EQ NULL )  NL_QUIT;      */
            /*            N_PascalTriRow(bin,kk);              */

            if( mult LT p ) /* Insert uu to mult=p */
            {
                N_CrvInitArrays( &curB );
                curptr = &curB;
                error = N_CrvCopy( &curA, &curB, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
                error = N_CrvInsertKnot( &curB, uu, p - mult, &curB, &SL, &SL );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
                curptr = &curA;

            N_CrvGetCPtsDegreeAndKnots( curptr, &nr, &Rw, &p, &mr, &UR );

            if (mr + p NEQ mq)
                NL_QUIT;

            rat = N_IsCrvRat( curptr );

            Dw = N_AllocCPt1dArray( p, &SL );

            if( Dw EQ NULL )
                NL_QUIT;

            if( end EQ NL_START )
            {
                for ( ii = 0; ii <= nr; ii++ ) /* load control points from curP */
                    N_CopyCPt( Rw[ii], &Qw[ii + p] );

                for ( ii = 1; ii <= mr; ii++ ) /* load knots from curP */
                    UQ[ii + p] = UR[ii];

                dr = p / (UR[p + 1] - UR[0]);

                for ( ii = 1; ii <= p; ii++ ) /* this loop computes derivatives */
                {                             /* from the right                 */
                    N_CopyCPt( Rw[ii], &Dw[ii] );

                    if( ii % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < ii; kk++ ) /* compute the ii-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Rw[kk], &Dw[ii] );
                        kp *= -1;
                    }
                    N_ScaleCPt( dr, Dw[ii], &Dw[ii] );
                    dr = dr * ((NL_REAL)p - (NL_REAL)ii) / (UR[p + 1] - UR[0]);
                }

                u2 = UR[0] - par;

                for ( jj = 0; jj <= p; jj++ )
                    UQ[jj] = par;

                dl = u2 / p;

                for ( jj = 1; jj <= p; jj++ ) /* this loop computes */
                {                             /* new control points */
                    if( jj % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw[jj], &Qw[p - jj] );
                    N_ScaleCPt( -kp * dl, Qw[p - jj], &Qw[p - jj] );

                    if( jj LT p )
                        dl = dl * u2 / ((NL_REAL)p - (NL_REAL)jj);

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (p-jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[jj][kk], Qw[p - kk], &Qw[p - jj] );
                        kp *= -1;
                    }
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 0; jj < p; jj++ )
                    {
                        N_CPtGetW( Qw[jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }
                }

                /* remove the original start knot */

                error = N_CrvRemoveKnot( curQ, UP[0], p, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }
            else
            {
                for ( ii = 0; ii <= nr; ii++ ) /* load control points from curP */
                    N_CopyCPt( Rw[ii], &Qw[ii] );

                for ( ii = 0; ii <= mr; ii++ )  /* load knots from curP */
                    UQ[ii] = UR[ii];

                dl = -p / (UR[mr] - UR[mr - p - 1]);

                for ( ii = 1; ii <= p; ii++ ) /* this loop computes derivatives */
                {                             /* from the left                  */
                    N_CopyCPt( Rw[nr - ii], &Dw[ii] );

                    if( ii % 2 EQ 0 )
                        kp = 1;
                    else
                        kp = -1;

                    for ( kk = 0; kk < ii; kk++ ) /* compute the ii-th derivative */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[ii][kk], Rw[nr - kk], &Dw[ii] );
                        kp *= -1;
                    }
                    N_ScaleCPt( dl, Dw[ii], &Dw[ii] );
                    dl = -dl * ((NL_REAL)p - (NL_REAL)ii) / (UR[mr] - UR[mr - p - 1]);
                }

                u2 = par - UR[mr];

                for ( jj = 0; jj <= p; jj++ )
                    UQ[mr + jj] = par;

                dr = u2 / p;

                for ( jj = 1; jj <= p; jj++ ) /* this loop computes */
                {                             /* new control points */
                    if( jj % 2 EQ 0 )
                        kp = -1;
                    else
                        kp = 1;
                    N_CopyCPt( Dw[jj], &Qw[nr + jj] );
                    N_ScaleCPt( dr, Qw[nr + jj], &Qw[nr + jj] );

                    if( jj LT p )
                        dr = dr * u2 / ((NL_REAL)p - (NL_REAL)jj);

                    for ( kk = 0; kk < jj; kk++ ) /* compute the (nr+jj)-th ctrl pt */
                    {
                        N_VectorBlendCPt( (NL_REAL)kp * NL_PascalTri[jj][kk], Qw[nr + kk], &Qw[nr + jj] );
                        kp *= -1;
                    }
                }

                if( rat ) /* don't create unacceptable weights */
                {
                    for ( jj = 1; jj <= p; jj++ )
                    {
                        N_CPtGetW( Qw[nr + jj], &w );

                        if( w LT NL_WMIN OR w GT NL_WMAX )
                            NL_ERROR( NL_WEI_ERR );
                    }
                }

                /* remove the original end knot */

                error = N_CrvRemoveKnot( curQ, UP[m], p, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            if( mult LT p )
            { /* remove the additional instances of uu */
                error = N_CrvRemoveKnot( curQ, uu, p - mult, NL_MTOL, &kk, curQ, SQ );

                if( error EQ NL_YES )
                    NL_OUT;
            }

            break;
    } /* end switch on continuity for computing output-curve cpts and knots*/

    /* End NURBS and Exit */

    EXIT:
    /* If extension is in place, kill old curve */
    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    N_EndNurbs( &SL );

    return (error);
} /* end ST_curxcrr */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes points and derivatives of a NURBS curve with-
     out regard to its u-domain bounds.  That is,  if the given u is out
     of bounds,  then the appropriately extended curve is evaluated.  If
     the curve is NL_G1-smoothly  closed,  then it  is  not  extended,  but 
     rather it is treated and evaluated as periodic.  A  typical calling 
     example is:

       NL_CURVE      cur, ecur;
       NL_PARAMETER  u;
       NL_FLAG       gflg;
       NL_INDEX      ndr;
       NL_POINT      *D;
       NL_STACKS     SG;
       ...
       (define cur, get u, ndr, and allocate D);
       ...
       N_CrvInitArrays(&ecur);    (initialize before first call to N_CrvEvalUnboundedPtsAndDerivs)
       ...
       N_CrvEvalUnboundedPtsAndDerivs(&cur,u,ndr,cflg,&ecur,&gflg,D,&SG);     (multiple calls)


   ACCESS:
   
     cur  , input  ,  NURBS curve
     u    , input  ,  Parameter value.  Does  not have to lie within the
                      the u-domain. 
     ndr  , input  ,  Maximum order of the  derivatives to compute. 
     cflg , input  ,  Flag:
                       NL_G1  : cur is  extended  with  tangent  continuity
                             where necessary for evaluation
                       NL_G2  : cur is extended by  reflection,  yielding a
                             NL_G2 (curvature) continuous  extension  where
                             necessary
                       NL_CMAX: the extension yields infinite  C-continuity
                             where necessary
     ecur , in/out ,  The curve's extension is stored  here.  ecur  must
                      be initialized to the NULL object before the first
                      call to N_CrvEvalUnboundedPtsAndDerivs with the curve,  cur.  Upon  sub-
                      sequent calls to N_CrvEvalUnboundedPtsAndDerivs, cur will be extended in
                      ecur if required.  The  calling  routine  must not 
                      modify this curve  until  evaluations of  cur  are
                      finished.  If ecur was used ( can be determined by
                      calling N_CrvAreArraysNULL),  then  it  can  be reused after
                      calls to N_FreeCrv and N_CrvInitArrays
     gflg , in/out ,  Flag:
                       This flag indicates  whether  cur  is  NL_G1-closed. 
                       This flag is computed only if cur requires exten-
                       sion.  The  calling  routine must not modify this 
                       flag until evaluations of cur are finished
     D    , output ,  Point and derivatives:  D[i]  is  the  i-th deriv-
                      ative (D[0] is the curve point). MEMORY FOR D MUST 
                      BE ALLOCATED IN THE CALLING ROUTINE
     SG   , input  ,  Global stack.  cur's stack.  If required,  ecur is 
                      also created on this stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalUnboundedPtsAndDerivs( NL_CURVE *cur, NL_PARAMETER u, NL_INDEX ndr, NL_FLAG cflg, NL_CURVE *ecur, NL_FLAG *gflg, NL_POINT *D, NL_STACKS *SG )
{

    /* NL_PRIVATE NL_STRING rname = _T("N_CrvEvalUnboundedPtsAndDerivs"); */
    NL_PRIVATE NL_REAL g1tol = 0.1; /* g1 angular  tolerance */

    NL_FLAG error = NL_NO;

    NL_CURVE *curptr;

    NL_FLAG g1_flag = 0;

    NL_REAL us, ue, uu, tols[2];

    /* Assign pointer to curve to be evaluated and get u - bounds */

    if( N_CrvAreArraysNULL( ecur ) )
        curptr = cur;
    else
    {
        curptr = ecur;
        g1_flag = *gflg;
    }

    N_CrvGetParamBounds( curptr, &us, &ue );

    /* Check if u is in bounds */

    if( u LT us OR u GT ue )
    { /* must extend or adjust periodic parameter */
        if( curptr EQ cur )
        {
            error = N_CrvCopy( cur, ecur, SG );

            if( error EQ NL_YES )
                NL_OUT;

            tols[0] = NL_MTOL;
            tols[1] = g1tol;

            error = N_CrvIsClosedContinuity( curptr, 1, tols, gflg );

            if( error EQ NL_YES )
                NL_OUT;

            curptr = ecur;
            g1_flag = *gflg;
        }

        /* Now extend or adjust parm */

        if( g1_flag EQ NL_YES )
        {
            while( u LT us )
                u = ue - us + u; /* adjust parm */

            while( u GT ue )
                u = us - ue + u;
        }
        else
        { /* extend */
            if( u LT us )
            {
                do
                {
                    if( us - u LE 0.9 *( ue - us ) )
                        uu = u;
                    else
                        uu = us - (0.9 *( ue - us ));

                    error = N_CrvExtendByParamDist( curptr, uu, NL_START, cflg, curptr, SG, SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    us = uu;
                } while ( u LT us );
            }
            else
            {
                do
                {
                    if( u - ue LE 0.9 *( ue - us ) )
                        uu = u;
                    else
                        uu = ue + (0.9 *( ue - us ));

                    error = N_CrvExtendByParamDist( curptr, uu, NL_END, cflg, curptr, SG, SG );

                    if( error EQ NL_YES )
                        NL_OUT;

                    ue = uu;
                } while ( u GT ue );
            }
        }
    }

    /* Now evaluate */

    if( ndr EQ 0 )
        error = N_CrvEval( curptr, u, NL_LEFT, &D[0] );
    else
        error = N_CrvDerivs( curptr, u, NL_LEFT, ndr, D );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_CrvEvalUnboundedPtsAndDerivs */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes the  average position vector magnitude of  a 
     curve by computing points at the nodes and averaging them. This is
     useful if the curve is a  derivative and the  average magnitude of 
     the derivatives is sought. A typical calling example is:

       NL_CURVE  der;
       NL_REAL   mag;
       ...
       (define der);
       ...
       N_CrvGetAveragePosMag(&der,&mag);


   ACCESS:
   
     der , input  ,  NURBS curve
     mag , output ,  Average magnitude


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetAveragePosMag( NL_CURVE *der, NL_REAL *mag )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_INDEX i, n;

    NL_REAL *t, len, total;

    NL_POINT C;

    NL_KNOTVECTOR *knt;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get the nodes */

    N_CrvGetDegree( der, &p );
    N_CrvGetKnotVector( der, &knt );
    N_CrvGetArraySizes( der, &n, &i );

    t = N_AllocReal1dArray( n, &SL );

    if( t EQ NULL )
        NL_QUIT;

    N_BasisFindIndexNodeArray( knt, p, t );

    /* Get average magnitude */

    total = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        error = N_CrvEval( der, t[i], NL_LEFT, &C );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorMagnitude( C, &len );

        total += len;
    }

    *mag = total / ((NL_REAL)n + (NL_REAL)1);

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetAveragePosMag */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine evaluates the rational or non rational basis function 
     of a curve  corresponding to a given index. It is assumed that the 
     end knots  are repeated with  multiplicity = degree + 1. A typical 
     calling example is:

       NL_CURVE      cur;
       NL_INDEX      k;
       NL_PARAMETER  u;
       NL_REAL       R;
       ...
       (define cur, get k and u);
       ...
       N_CrvBasisIEval(&cur,k,u,NL_LEFT,&R);


   ACCESS:
   
     cur , input  ,  NURBS curve
     k   , input  ,  Index of basis function
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     R   , output ,  Basis function computed at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvBasisIEval */
NL_FLAG N_CrvBasisIEval( NL_CURVE *cur, NL_INDEX k, NL_PARAMETER u, NL_FLAG flg, NL_REAL *R )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_KNOTVECTOR *knt;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute basis function */

    if( N_IsCrvRat( cur ) )
    {
        error = N_CrvRatBasisIEval( cur, k, u, flg, R );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        N_CrvGetKnotVector( cur, &knt );
        N_CrvGetDegree( cur, &p );

        error = N_BasisIEval( knt, k, p, u, flg, R );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvBasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivatives of all non-vanishing rational 
     basis functions with  respect to a  knot. The  multiplicity of  the 
     knot must be less than the degree. A typical calling example is:

       NL_CURVE       cur;
       NL_INDEX       k;
       NL_PARAMETER   u;
       NL_REAL        *Rk;
       ...
       (define cur, get k and u);
       ...
       N_CrvRatBasisKnotDeriv(&cur,k,u,NL_LEFT,NL_RIGHT,Rk);

     MEMORY TO STORE  Rk[0],...,Rk[p+1] MUST BE ALLOCATED IN THE CALLING 
     ROUTINE.


   ACCESS:
   
     cur , input  ,  NURBS curve
     k   , input  ,  Index of knot, i.e. the derivative  with respect to
                     u_k is computed
     u   , input  ,  Parameter value 
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k-1)
                       NL_RIGHT: right  derivative. NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k+1)
     flp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]] 
     Rk  , output ,  Basis  function  derivatives  computed  at  u.  The 
                     values are stored in Rk[0],...,Rk[p+1].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvRatBasisKnotDeriv */
NL_FLAG N_CrvRatBasisKnotDeriv( NL_CURVE *cur, NL_INDEX k, NL_PARAMETER u, NL_FLAG flk, NL_FLAG flp, NL_REAL *Rk )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, kk;

    NL_DEGREE p;

    NL_REAL *U;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get basis functions */

    N_CrvGetDegree( cur, &p );
    N_CrvGetKnots( cur, &i, &U );

    kk = k;

    if( flk EQ NL_LEFT AND u GT U[k] )
        while( U[kk]EQ U[kk + 1] )
            kk++;

    if( flk EQ NL_RIGHT AND u LT U[k] )
        while( U[kk]EQ U[kk - 1] )
            kk--;

    for ( i = kk - p - 1; i <= kk; i++ )
    {
        error = N_CrvRatBasisIKnotDeriv( cur, i, k, u, flk, flp, &Rk[i - kk + p + 1] );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRatBasisKnotDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivative of a univariate rational basis 
     function with respect to a knot. A typical calling example is:

       NL_CURVE      cur;
       NL_INDEX      i, k;
       NL_PARAMETER  u;
       NL_REAL       R;
       ...
       (define cur, get i, and k);
       ...
       N_CrvRatBasisIKnotDeriv(&cur,i,k,u,NL_LEFT,NL_RIGHT,&Rd);
       

   ACCESS:
   
     cur , input  ,  NURBS curve
     i   , input  ,  Index of rational basis function
     k   , input  ,  Index of  knot, i.e. the derivative with respect to 
                     u_k is computed
     u   , input  ,  Parameter value
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative.  NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k-1)
                       NL_RIGHT: right derivative.  NL_INDEX  k  MUST  SATISFY
                              u_(k) != u_(k+1) 
     flp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     Rd  , output ,  Derivative computed  at u


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_CrvRatBasisIKnotDeriv */
NL_FLAG N_CrvRatBasisIKnotDeriv( NL_CURVE *cur, NL_INDEX i, NL_INDEX k, NL_PARAMETER u, NL_FLAG flk, NL_FLAG flp, NL_REAL *Rd )
{

    NL_FLAG error = NL_NO;

    NL_REAL *w, *A, Nk, fd, den, R;

    NL_DEGREE p;

    NL_KNOTVECTOR *knt;

    NL_CFUN cfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvGetDegree( cur, &p );
    N_CrvGetKnotVector( cur, &knt );

    /* Extract denominator */

    N_CFuncInitArrays( &cfn );
    error = N_CrvGetDenomCrvFunc( cur, &cfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( &cfn, &w, &A );

    /* Get denominator */

    error = N_CFuncEval( &cfn, u, flp, &den );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get numerator and derivative */

    error = N_BasisIKnotDeriv( knt, i, k, p, u, flk, flp, &Nk );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvFuncEvalDerivsAtKnot( &cfn, k, u, flk, flp, &fd );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvRatBasisIEval( cur, i, u, flp, &R );

    if( error EQ NL_YES )
        NL_OUT;

    *Rd = (w[i] * Nk - fd * R) / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CrvRatBasisIKnotDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS curve. 
     It reparametrizes  the curve with  respect to the arc length to be 
     able to remove as many knots as possible. This routine is suitable
     for NURBS curves stiched together  from Bezier  pieces with fairly 
     bad (uniform) parametrization. If the  output curve is initialized 
     to  NULL,  memory  to  store  new  control  points  and  knots  is 
     allocated.  If the  output curve  is the  same as the input curve, 
     knot removal  is in place and  the original curve  is destroyed. A 
     typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SQ;
       ...
       (define curP, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveAllKnotsArcLen(&curP,tol,&curQ,&SQ);
       N_CrvRemoveAllKnotsArcLen(&curP,tol,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon  objects. THE ROUTINE USES  0.1%  NL_RELATIVE
     TOLERANCE TO APPROXIMATE ARC LENGTH.


   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance 
     curQ , output ,  Curve after cleaning
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveAllKnotsArcLen( NL_CURVE *curP, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SQ )
{

    /* NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveAllKnotsArcLen"); */

    NL_FLAG error = NL_NO;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Reparametrize and clean */

    if( curP EQ curQ )
    {
        error = N_SrfReparamMultKnots( curP, eps, curP, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvRemoveKnots( curP, tol, curP, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_SrfReparamMultKnots( curP, eps, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvRemoveKnots( curQ, tol, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveAllKnotsArcLen */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS curve. 
     If the  output curve is initialized to NULL, memory  to  store new  
     control points and knots is allocated. If the  output curve is the 
     same as the input  curve, knot  removal is  done in place  and the 
     original curve is destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SQ;
       ...
       (define curP, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveAllKnots(&curP,tol,&curQ,&SQ);
       N_CrvRemoveAllKnots(&curP,tol,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects. THIS  VERSION USES AN  APPROXIMA-
     TION OF THE  PRECISE NL_ERROR  TOLERANCE. THAT  IS, IT IS FASTER THAN
     N_CrvRemoveKnots, HOWEVER, IT REMOVES LESS KNOTS.

   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance to check removability
     curQ , output ,  Curve after knot removal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveAllKnots( NL_CURVE *curP, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveAllKnots");

    NL_FLAG rmf, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, fout, l, ns;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *br, *er, *te, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    ns = n;

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsKnotVectorAndKnots( curP, &Qw, &knt, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsKnotVectorAndKnots( curQ, &Qw, &knt, &UQ );
    }

    /* Get local memory */

    Rw = N_AllocCPt1dArray( 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal1dArray( m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal1dArray( m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal1dArray( m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    /* Initialize */

    if( curP NEQ curQ )
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= m; i++ )
    {
        br[i] = NL_BIGD;
        sr[i] = 0;
        er[i] = 0.0;
    }

    /* Compute knot removal errors for each distinct knot */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND UQ[r]EQ UQ[r + 1] )
            r++;
        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( curP, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        b = br[p + 1];
        s = sr[p + 1];
        r = p + 1;

        for ( i = p + 2; i <= m - p - 1; i++ )
        {
            if( br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot -> finished */

        if( b EQ NL_BIGD OR b EQ NOREM )
            break;

        /* Check error of removal */

        if( (p + s) % 2 )
        {
            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (UQ[r] - UQ[r - k]) / (UQ[r - k + p + 1] - UQ[r - k]);
            bet = (UQ[r] - UQ[r - k + 1]) / (UQ[r - k + p + 2] - UQ[r - k + 1]);
            lam = alf / (alf + bet);
            oml = 1.0 - lam;
        }
        else
        {
            k = (p + s) / 2;
            l = r - k + p;
        }

        rmf = NL_TRUE;

        for ( i = r - k; i <= l; i++ )
        {
            if( UQ[i]NEQ UQ[i + 1] )
            {
                te[i] = er[i] + b;

                if( te[i]GT tol )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( i = r - k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1] )
                    er[i] = te[i];
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;
            i = first;
            j = last;
            ii = 1;
            jj = last - off;

            N_CopyCPt( Qw[off], &Rw[0] );
            N_CopyCPt( Qw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (UQ[i + p + 1] - UQ[i]) / (UQ[r] - UQ[i]);
                oma = 1.0 - alf;
                bet = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Qw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Qw[j], omb, Rw[jj + 1], &Rw[jj] );
                i++;
                j--;
                ii++;
                jj--;
            }

            /* Check for disallowed weights */

            if( rat EQ NL_YES )
            {
                i = first;
                j = last;
                wmin = NL_BIGD;
                wmax = NL_SMAD;

                while( (j - i)GT 0 )
                {
                    N_CPtGetW( Rw[i - off], &wi );
                    N_CPtGetW( Rw[j - off], &wj );

                    if( wi LT wmin )
                        wmin = wi;

                    if( wj LT wmin )
                        wmin = wj;

                    if( wi GT wmax )
                        wmax = wi;

                    if( wj GT wmax )
                        wmax = wj;
                    i++;
                    j--;
                }

                if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                {
                    br[r] = NOREM;
                    continue;
                }
            }

            /* Save new control points */

            if( (p + s) % 2 )
            {
                N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
            }

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                N_CopyCPt( Rw[i - off], &Qw[i] );
                N_CopyCPt( Rw[j - off], &Qw[j] );
                i++;
                j--;
            }

            /* Shift down some parameters */

            if( s EQ 1 )
                er[r - 1] = NL_MAX( er[r - 1], er[r] );

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( i = r + 1; i <= m; i++ )
            {
                br[i - 1] = br[i];
                sr[i - 1] = sr[i];
                er[i - 1] = er[i];
            }

            /* Shift down knots and control points */

            for ( i = r + 1; i <= m; i++ )
                UQ[i - 1] = UQ[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Qw[i], &Qw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( curQ, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( curQ, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        error = N_CrvCompress( curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveAllKnots */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS  curve 
     with  point and  tangent constraints  applied at given parameters.
     If the  output curve is initialized to NULL, memory  to  store new  
     control points and knots is allocated. If the  output curve is the 
     same as the input  curve, knot  removal is  done in place  and the 
     original curve is destroyed. A typical calling example is:

       NL_FLAG       *cst;
       NL_CURVE      curP, curQ;
       NL_PARAMETER  *u;
       NL_INDEX      nu;
       NL_REAL       tol;
       NL_STACKS     SQ;
       ...
       (define curP, u and cst; get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveAllKnotsConstraints(&curP,u,nu,cst,tol,&curQ,&SQ);
       N_CrvRemoveAllKnotsConstraints(&curP,u,nu,cst,tol,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects. THIS  VERSION USES AN  APPROXIMA-
     TION OF THE  PRECISE NL_ERROR  TOLERANCE. THAT  IS, IT IS FASTER THAN
     N_CrvRemoveKnotsParams, HOWEVER, IT REMOVES LESS KNOTS.

   ACCESS:
   
     curP , input  ,  NURBS curve
     u    , input  ,  Parameter values at which constraints are applied
     nu   , input  ,  Highest index in u
     cst  , input  ,  cst[i] means:
                        NL_EPOINT    : constrain C(u[i])
                        NL_DERIVATIVE: constrain C'(u[i])
                        NL_BOTH      : constrain both
     tol  , input  ,  Tolerance to check removability
     curQ , output ,  Curve after knot removal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveAllKnotsConstraints( NL_CURVE *curP, NL_PARAMETER *u, NL_INDEX nu, NL_FLAG *cst, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveAllKnotsConstraints");

    NL_FLAG rmf, rem, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, fout, l, ns, lp, rp, lt, rt, left = 0, right= 0;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *br, *er, *te, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input and get local notation */

    for ( i = 0; i <= nu; i++ )
    {
        switch( cst[i] )
        {
            case NL_EPOINT:
                break;

            case NL_DERIVATIVE:
                break;

            case NL_BOTH:
                break;

            default:
                NL_ERROR( NL_INP_ERR );
        }
    }

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    ns = n;

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsKnotVectorAndKnots( curP, &Qw, &knt, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsKnotVectorAndKnots( curQ, &Qw, &knt, &UQ );
    }

    /* Get local memory */

    Rw = N_AllocCPt1dArray( 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal1dArray( m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal1dArray( m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal1dArray( m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    /* Initialize */

    if( curP NEQ curQ )
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= m; i++ )
    {
        br[i] = NL_BIGD;
        sr[i] = 0;
        er[i] = 0.0;
    }

    /* Compute knot removal errors for each distinct knot */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND UQ[r]EQ UQ[r + 1] )
            r++;
        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( curP, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        b = br[p + 1];
        s = sr[p + 1];
        r = p + 1;

        for ( i = p + 2; i <= m - p - 1; i++ )
        {
            if( br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot -> finished */

        if( b EQ NL_BIGD OR b EQ NOREM )
            break;

        /* Check constraints */

        lp = NL_MAX( p, r - p - 1 );
        rp = NL_MIN( n + 1, r + p - s + 2 );
        lt = NL_MAX( p, r - p );
        rt = NL_MIN( n + 1, r + p - s + 1 );

        rem = NL_YES;

        for ( i = 0; i <= nu; i++ )
        {
            if( cst[i]EQ NL_EPOINT OR cst[i]EQ NL_BOTH )
            {
                left = lp;
                right = rp;
            }

            if( cst[i]EQ NL_DERIVATIVE )
            {
                left = lt;
                right = rt;
            }

            if( u[i]GT UQ[left]AND u[i]LT UQ[right] )
            {
                rem = NL_NO;
                break;
            }
        }

        if( rem EQ NL_NO )
        {
            br[r] = NOREM;
            continue;
        }

        /* Check error of removal */

        if( (p + s) % 2 )
        {
            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (UQ[r] - UQ[r - k]) / (UQ[r - k + p + 1] - UQ[r - k]);
            bet = (UQ[r] - UQ[r - k + 1]) / (UQ[r - k + p + 2] - UQ[r - k + 1]);
            lam = alf / (alf + bet);
            oml = 1.0 - lam;
        }
        else
        {
            k = (p + s) / 2;
            l = r - k + p;
        }

        rmf = NL_TRUE;

        for ( i = r - k; i <= l; i++ )
        {
            if( UQ[i]NEQ UQ[i + 1] )
            {
                te[i] = er[i] + b;

                if( te[i]GT tol )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( i = r - k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1] )
                    er[i] = te[i];
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;
            i = first;
            j = last;
            ii = 1;
            jj = last - off;

            N_CopyCPt( Qw[off], &Rw[0] );
            N_CopyCPt( Qw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (UQ[i + p + 1] - UQ[i]) / (UQ[r] - UQ[i]);
                oma = 1.0 - alf;
                bet = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Qw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Qw[j], omb, Rw[jj + 1], &Rw[jj] );
                i++;
                j--;
                ii++;
                jj--;
            }

            /* Check for disallowed weights */

            if( rat EQ NL_YES )
            {
                i = first;
                j = last;
                wmin = NL_BIGD;
                wmax = NL_SMAD;

                while( (j - i)GT 0 )
                {
                    N_CPtGetW( Rw[i - off], &wi );
                    N_CPtGetW( Rw[j - off], &wj );

                    if( wi LT wmin )
                        wmin = wi;

                    if( wj LT wmin )
                        wmin = wj;

                    if( wi GT wmax )
                        wmax = wi;

                    if( wj GT wmax )
                        wmax = wj;
                    i++;
                    j--;
                }

                if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                {
                    br[r] = NOREM;
                    continue;
                }
            }

            /* Save new control points */

            if( (p + s) % 2 )
            {
                N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
            }

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                N_CopyCPt( Rw[i - off], &Qw[i] );
                N_CopyCPt( Rw[j - off], &Qw[j] );
                i++;
                j--;
            }

            /* Shift down some parameters */

            if( s EQ 1 )
                er[r - 1] = NL_MAX( er[r - 1], er[r] );

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( i = r + 1; i <= m; i++ )
            {
                br[i - 1] = br[i];
                sr[i - 1] = sr[i];
                er[i - 1] = er[i];
            }

            /* Shift down knots and control points */

            for ( i = r + 1; i <= m; i++ )
                UQ[i - 1] = UQ[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Qw[i], &Qw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( curQ, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( curQ, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        error = N_CrvCompress( curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveAllKnotsConstraints */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools routine makes a set of curves compatible using approxi-
     mation. More precisely, first it  makes curves compatible precisely
     followed by removing as many knots as possible. The curves are made 
     compatible in place, ie the  original control points are destroyed. 
     A typical calling example is:

       NL_INDEX   kk;
       NL_CURVE   **cur;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define array of curves and get tol);
       ...
       N_CrvsMakeCompatibleApprox(cur,kk,tol,&SG);

     ALL CURVES MUST BE ON THE SAME STACK "SG"!


   ACCESS:
   
     cur , in/out ,  A set of NURBS curves
     kk  , input  ,  Highest index in cur
     tol , input  ,  Tolerance to check removability. tol = 0.0 COMPUTES
                     PRECISE COMPATIBILITY!
     SG  , input  ,  curs' stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatibleApprox( NL_CURVE ** cur, NL_INDEX kk, NL_REAL tol, NL_STACKS *SG )
{

    /* NL_PRIVATE NL_STRING rname = _T("N_CrvsMakeCompatibleApprox"); */

    NL_FLAG rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, l, ll, ii, jj, first, last, off, n, ns, m, r, s = 0, fout;

    NL_DEGREE p;

    NL_REAL ** br, ** er, ** te, *U, *a, *b, *minl, *maxl, *minr, *maxr, *max, *alfs, *omas, *bets, *ombs, wmin, wmax, pmax, tmp, alf, bet, omb, lam = 0.0, oml = 0.0, lto, bsum, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT ** Rw, *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curves exactly compatible, then remove unwanted knots */

    error = N_CrvsMakeCompatibleKnotTol( cur, kk, NL_PTOL, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( tol LT NL_PTOL )
        NL_OUT;

    /************************************/
    /* Remove as many knots as possible */
    /************************************/

    /* Adjust knot removal tolerance for rational curves */

    N_CrvGetCPtsDegreeAndKnots( cur[kk], &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur[kk], &knt );
    ns = n;

    if( N_IsCrvRat( cur[0] ) )
    {
        N_CrvGetMinMaxWeightsAndPts( cur[0], &wmin, &tmp, &tmp, &pmax );
        alf = (tol * wmin) / (1.0 + pmax);

        for ( ll = 1; ll <= kk; ll++ )
        {
            N_CrvGetMinMaxWeightsAndPts( cur[ll], &wmin, &tmp, &tmp, &pmax );
            bet = (tol * wmin) / (1.0 + pmax);

            if( bet LT alf )
                alf = bet;
        }
        tol = alf;
        rat = NL_YES;
    }

    lto = cto * fabs( U[m] - U[0] );

    /* Get local memory */

    Rw = N_AllocCPt2dArray( kk, 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal2dArray( kk, m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    b = N_AllocReal1dArray( kk, &SL );

    if( b EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( kk, m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( kk, m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    minl = N_AllocReal1dArray( p, &SL );

    if( minl EQ NULL )
        NL_QUIT;

    maxl = N_AllocReal1dArray( p, &SL );

    if( maxl EQ NULL )
        NL_QUIT;

    minr = N_AllocReal1dArray( p, &SL );

    if( minr EQ NULL )
        NL_QUIT;

    maxr = N_AllocReal1dArray( p, &SL );

    if( maxr EQ NULL )
        NL_QUIT;

    max = N_AllocReal1dArray( p + 1, &SL );

    if( max EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( 2 * p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( 2 * p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    bets = N_AllocReal1dArray( 2 * p, &SL );

    if( bets EQ NULL )
        NL_QUIT;

    ombs = N_AllocReal1dArray( 2 * p, &SL );

    if( ombs EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( j = 0; j <= m; j++ )
    {
        sr[j] = 0;

        for ( ll = 0; ll <= kk; ll++ )
        {
            br[ll][j] = NL_BIGD;
            er[ll][j] = 0.0;
        }
    }

    /* Compute knot removal errors for each distinct knot for each curve */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;
        sr[r] = r - i + 1;

        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvRemoveKnotMaxErr( cur[ll], r, sr[r], &br[ll][r] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        r++;
    }

    /* Try to remove each knot from each curve */

    while( NL_TRUE )
    {
        /* Find knot with smallest error sum */

        bsum = NL_BIGD;

        for ( j = p + 1; j <= m - p - 1; j++ )
        {
            if( br[0][j]NEQ NL_BIGD AND br[0][j]NEQ NOREM )
            {
                alf = 0.0;

                for ( ll = 0; ll <= kk; ll++ )
                    alf += br[ll][j];

                if( alf LT bsum )
                {
                    bsum = alf;
                    s = sr[j];
                    r = j;

                    for ( ll = 0; ll <= kk; ll++ )
                        b[ll] = br[ll][j];
                }
            }
        }

        /* If no more removable knot -> finished */

        if( bsum EQ NL_BIGD )
            break;

        /* Check error of removal */

        rmf = NL_TRUE;

        if( (p + s) % 2 )
        {
            /* Compute maximums of basis functions over each span */

            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (U[r] - U[r - k]) / (U[r - k + p + 1] - U[r - k]);
            bet = (U[r] - U[r - k + 1]) / (U[r - k + p + 2] - U[r - k + 1]);
            omb = 1.0 - bet;
            lam = alf / (alf + bet);
            oml = 1.0 - lam;

            error = N_BasisFindAllSpanMaxima( knt, r - k, p, lto, minl, maxl, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                {
                    minl[i] = 0.0;
                    maxl[i] = 1.0;
                }
            }

            error = N_BasisFindAllSpanMaxima( knt, r - k + 1, p, lto, minr, maxr, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                {
                    minr[i] = 0.0;
                    maxr[i] = 1.0;
                }
            }

            max[0] = lam * alf * maxl[0];

            for ( i = 1; i <= p; i++ )
            {
                minl[i] *= lam * alf;
                minr[i - 1] *= oml * omb;
                maxl[i] *= lam * alf;
                maxr[i - 1] *= oml * omb;

                max[i] = NL_MAX( fabs( maxl[i] - minr[i - 1] ), fabs( maxr[i - 1] - minl[i] ) );
            }
            max[p + 1] = oml * omb * maxr[p];
        }
        else
        {
            /* Compute maximum of basis function */

            k = (p + s) / 2;
            l = r - k + p;

            error = N_BasisFindAllSpanMaxima( knt, r - k, p, lto, minl, max, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                    max[i] = 1.0;
            }
        }

        /* Check the error */

        for ( j = r - k; j <= l; j++ )
        {
            if( U[j]NEQ U[j + 1] )
            {
                for ( ll = 0; ll <= kk; ll++ )
                {
                    te[ll][j] = er[ll][j] + max[j - r + k] * b[ll];

                    if( te[ll][j]GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }
                }
            }

            if( rmf EQ NL_FALSE )
                break;
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( j = r - k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                        er[ll][j] = te[ll][j];
                }
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;

            /* Save alphas and betas */

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                alfs[i - first] = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                omas[i - first] = 1.0 - alfs[i - first];
                bets[j - first] = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                ombs[j - first] = 1.0 - bets[j - first];
                i++;
                j--;
            }

            /* Remove the knot from all curves */

            wfl = NL_TRUE;

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Pw[off], &Rw[ll][0] );
                N_CopyCPt( Pw[last + 1], &Rw[ll][last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT 0 )
                {
                    N_Combine2CPts( alfs[i - first], Pw[i], omas[i - first], Rw[ll][ii - 1], &Rw[ll][ii] );
                    N_Combine2CPts( bets[j - first], Pw[j], ombs[j - first], Rw[ll][jj + 1], &Rw[ll][jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Check for disallowed weights */

                if( rat EQ NL_YES )
                {
                    i = first;
                    j = last;
                    wmin = NL_BIGD;
                    wmax = NL_SMAD;

                    while( (j - i)GT 0 )
                    {
                        N_CPtGetW( Rw[ll][i - off], &wi );
                        N_CPtGetW( Rw[ll][j - off], &wj );

                        if( wi LT wmin )
                            wmin = wi;

                        if( wj LT wmin )
                            wmin = wj;

                        if( wi GT wmax )
                            wmax = wi;

                        if( wj GT wmax )
                            wmax = wj;
                        i++;
                        j--;
                    }

                    if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                    {
                        wfl = NL_FALSE;
                        break;
                    }
                }

                if( (p + s) % 2 )
                {
                    N_Combine2CPts( lam, Rw[ll][jj + 1], oml, Rw[ll][ii - 1], &Rw[ll][jj + 1] );
                }
            } /* End for each curve */

            /* Check weights */

            if( wfl EQ NL_FALSE )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    br[ll][r] = NOREM;
                continue;
            }
            else
            {
                /* Save control points */

                for ( ll = 0; ll <= kk; ll++ )
                {
                    N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        N_CopyCPt( Rw[ll][i - off], &Pw[i] );
                        N_CopyCPt( Rw[ll][j - off], &Pw[j] );
                        i++;
                        j--;
                    }
                }
            }

            /* Shift down some parameters */

            if( s EQ 1 )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    er[ll][r - 1] = NL_MAX( er[ll][r - 1], er[ll][r] );
            }

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( j = r + 1; j <= m; j++ )
            {
                sr[j - 1] = sr[j];

                for ( ll = 0; ll <= kk; ll++ )
                {
                    br[ll][j - 1] = br[ll][j];
                    er[ll][j - 1] = er[ll][j];
                }
            }

            /* Shift down knots and control points */

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &U );

                for ( j = r + 1; j <= m; j++ )
                    U[j - 1] = U[j];

                for ( j = fout + 1; j <= n; j++ )
                {
                    N_CopyCPt( Pw[j], &Pw[j - 1] );
                }

                N_CrvSetSizeIndices( cur[ll], n - 1, p, m - 1 );
            }
            n--;
            m--;

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( j = k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                    {
                        if( br[ll][j]NEQ NOREM )
                        {
                            error = N_CrvRemoveKnotMaxErr( cur[ll], j, sr[j], &br[ll][j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }
            }
        }
        else
        {
            /* Knot is not removable */

            for ( ll = 0; ll <= kk; ll++ )
                br[ll][r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvCompress( cur[ll], SG );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvsMakeCompatibleApprox */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  tools routine makes a set of curves compatible using approxi-
     mation. More precisely, first it  makes curves compatible precisely
     followed by removing as many knots as possible. The curves are made 
     compatible in place, ie the  original control points are destroyed. 
     A typical calling example is:

       NL_INDEX   kk;
       NL_CURVE   **cur;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define array of curves and get tol);
       ...
       N_CrvsMakeCompatibleKnotRemove(cur,kk,tol,&SG);

     ALL CURVES  MUST BE  ON  THE  SAME  STACK "SG"! THIS  VERSION  USES 
     APPROXIMATIVE NL_ERROR MEASURES AND HENCE IT IS FASTER THAN  N_CrvsMakeCompatibleApprox. 
     HOWEVER, IT REMOVES SIGNIFICANTLY LESS KNOTS.


   ACCESS:
   
     cur , in/out ,  A set of NURBS curves
     kk  , input  ,  Highest index in cur
     tol , input  ,  Tolerance  to check  removability. tol=0.0 PROVIDES
                     PRECISE COMPATIBILITY!
     SG  , input  ,  curs' stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatibleKnotRemove( NL_CURVE ** cur, NL_INDEX kk, NL_REAL tol, NL_STACKS *SG )
{

    NL_FLAG rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, l, ll, ii, jj, first, last, off, n, ns, m, r, s = 0, fout;

    NL_DEGREE p;

    NL_REAL ** br, ** er, ** te, *U, *a, *b, *alfs, *omas, *bets, *ombs, wmin, wmax, pmax, tmp, alf, bet, lam = 0.0, oml = 0.0, bsum, wi, wj;

    NL_CPOINT ** Rw, *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Make curves exactly compatible, then remove unwanted knots */

    error = N_CrvsMakeCompatibleKnotTol( cur, kk, NL_PTOL, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( tol LT NL_PTOL )
        NL_OUT;

    /************************************/
    /* Remove as many knots as possible */
    /************************************/

    /* Adjust knot removal tolerance for rational curves */

    N_CrvGetCPtsDegreeAndKnots( cur[kk], &n, &Pw, &p, &m, &U );
    ns = n;

    if( N_IsCrvRat( cur[0] ) )
    {
        N_CrvGetMinMaxWeightsAndPts( cur[0], &wmin, &tmp, &tmp, &pmax );
        alf = (tol * wmin) / (1.0 + pmax);

        for ( ll = 1; ll <= kk; ll++ )
        {
            N_CrvGetMinMaxWeightsAndPts( cur[ll], &wmin, &tmp, &tmp, &pmax );
            bet = (tol * wmin) / (1.0 + pmax);

            if( bet LT alf )
                alf = bet;
        }
        tol = alf;
        rat = NL_YES;
    }

    /* Get local memory */

    Rw = N_AllocCPt2dArray( kk, 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal2dArray( kk, m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    b = N_AllocReal1dArray( kk, &SL );

    if( b EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( kk, m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( kk, m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( 2 * p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( 2 * p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    bets = N_AllocReal1dArray( 2 * p, &SL );

    if( bets EQ NULL )
        NL_QUIT;

    ombs = N_AllocReal1dArray( 2 * p, &SL );

    if( ombs EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( j = 0; j <= m; j++ )
    {
        sr[j] = 0;

        for ( ll = 0; ll <= kk; ll++ )
        {
            br[ll][j] = NL_BIGD;
            er[ll][j] = 0.0;
        }
    }

    /* Compute knot removal errors for each distinct knot for each curve */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;
        sr[r] = r - i + 1;

        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvRemoveKnotMaxErr( cur[ll], r, sr[r], &br[ll][r] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        r++;
    }

    /* Try to remove each knot from each curve */

    while( NL_TRUE )
    {
        /* Find knot with smallest error sum */

        bsum = NL_BIGD;

        for ( j = p + 1; j <= m - p - 1; j++ )
        {
            if( br[0][j]NEQ NL_BIGD AND br[0][j]NEQ NOREM )
            {
                alf = 0.0;

                for ( ll = 0; ll <= kk; ll++ )
                    alf += br[ll][j];

                if( alf LT bsum )
                {
                    bsum = alf;
                    s = sr[j];
                    r = j;

                    for ( ll = 0; ll <= kk; ll++ )
                        b[ll] = br[ll][j];
                }
            }
        }

        /* If no more removable knot -> finished */

        if( bsum EQ NL_BIGD )
            break;

        /* Check error of removal */

        if( (p + s) % 2 )
        {
            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (U[r] - U[r - k]) / (U[r - k + p + 1] - U[r - k]);
            bet = (U[r] - U[r - k + 1]) / (U[r - k + p + 2] - U[r - k + 1]);
            lam = alf / (alf + bet);
            oml = 1.0 - lam;
        }
        else
        {
            k = (p + s) / 2;
            l = r - k + p;
        }

        rmf = NL_TRUE;

        for ( j = r - k; j <= l; j++ )
        {
            if( U[j]NEQ U[j + 1] )
            {
                for ( ll = 0; ll <= kk; ll++ )
                {
                    te[ll][j] = er[ll][j] + b[ll];

                    if( te[ll][j]GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }
                }
            }

            if( rmf EQ NL_FALSE )
                break;
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( j = r - k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                        er[ll][j] = te[ll][j];
                }
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;

            /* Save alphas and betas */

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                alfs[i - first] = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                omas[i - first] = 1.0 - alfs[i - first];
                bets[j - first] = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                ombs[j - first] = 1.0 - bets[j - first];
                i++;
                j--;
            }

            /* Remove the knot from all curves */

            wfl = NL_TRUE;

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Pw[off], &Rw[ll][0] );
                N_CopyCPt( Pw[last + 1], &Rw[ll][last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT 0 )
                {
                    N_Combine2CPts( alfs[i - first], Pw[i], omas[i - first], Rw[ll][ii - 1], &Rw[ll][ii] );
                    N_Combine2CPts( bets[j - first], Pw[j], ombs[j - first], Rw[ll][jj + 1], &Rw[ll][jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Check for disallowed weights */

                if( rat EQ NL_YES )
                {
                    i = first;
                    j = last;
                    wmin = NL_BIGD;
                    wmax = NL_SMAD;

                    while( (j - i)GT 0 )
                    {
                        N_CPtGetW( Rw[ll][i - off], &wi );
                        N_CPtGetW( Rw[ll][j - off], &wj );

                        if( wi LT wmin )
                            wmin = wi;

                        if( wj LT wmin )
                            wmin = wj;

                        if( wi GT wmax )
                            wmax = wi;

                        if( wj GT wmax )
                            wmax = wj;
                        i++;
                        j--;
                    }

                    if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                    {
                        wfl = NL_FALSE;
                        break;
                    }
                }

                if( (p + s) % 2 )
                {
                    N_Combine2CPts( lam, Rw[ll][jj + 1], oml, Rw[ll][ii - 1], &Rw[ll][jj + 1] );
                }
            } /* End for each curve */

            /* Check weights */

            if( wfl EQ NL_FALSE )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    br[ll][r] = NOREM;
                continue;
            }
            else
            {
                /* Save control points */

                for ( ll = 0; ll <= kk; ll++ )
                {
                    N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        N_CopyCPt( Rw[ll][i - off], &Pw[i] );
                        N_CopyCPt( Rw[ll][j - off], &Pw[j] );
                        i++;
                        j--;
                    }
                }
            }

            /* Shift down some parameters */

            if( s EQ 1 )
            {
                for ( ll = 0; ll <= kk; ll++ )
                {
                    er[ll][r - 1] = NL_MAX( er[ll][r - 1], er[ll][r] );
                }
            }

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( j = r + 1; j <= m; j++ )
            {
                sr[j - 1] = sr[j];

                for ( ll = 0; ll <= kk; ll++ )
                {
                    br[ll][j - 1] = br[ll][j];
                    er[ll][j - 1] = er[ll][j];
                }
            }

            /* Shift down knots and control points */

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &U );

                for ( j = r + 1; j <= m; j++ )
                    U[j - 1] = U[j];

                for ( j = fout + 1; j <= n; j++ )
                {
                    N_CopyCPt( Pw[j], &Pw[j - 1] );
                }

                N_CrvSetSizeIndices( cur[ll], n - 1, p, m - 1 );
            }
            n--;
            m--;

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( j = k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                    {
                        if( br[ll][j]NEQ NOREM )
                        {
                            error = N_CrvRemoveKnotMaxErr( cur[ll], j, sr[j], &br[ll][j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }
            }
        }
        else
        {
            /* Knot is not removable */

            for ( ll = 0; ll <= kk; ll++ )
                br[ll][r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvCompress( cur[ll], SG );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvsMakeCompatibleKnotRemove */

/*******************************************************************//**


   DESCRIPTION:

     This  tools routine makes a set of curves compatible using approxi-
     mation. More precisely, first it  makes curves compatible precisely
     followed by  removing as  many knots  as  possible. End  derivative 
     constraints or end tangent constraints can be specified. The curves 
     are made compatible in place, i.e. the  original control points are 
     destroyed. A typical calling example is:

       NL_INDEX   kk, der;
       NL_CURVE   **cur;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define array of curves and get tol and der);
       ...
       N_CrvsMakeCompatibleConstraints(cur,kk,tol,NL_BOTH,NL_TANGENT,der,&SG);

     ALL CURVES MUST BE ON THE SAME STACK "SG"!


   ACCESS:
   
     cur , in/out ,  A set of NURBS curves
     kk  , input  ,  Highest index in cur
     tol , input  ,  Tolerance to check removability. tol = 0.0 COMPUTES
                     PRECISE COMPATIBILITY!
     whr , input  ,  Flag:
                       NL_NO   : don't constrain end derivatives
                       NL_START: constrain at the start point
                       NL_END  : constrain at the end point
                       NL_BOTH : constrain at both ends
     tnf , input  ,  Flag:
                       NL_TANGENT   : constrain tangent directions  at  the
                                   specified endpoints  (magnitudes  may
                                   be changed)
                       NL_DERIVATIVE: constrain derivatives at  the  speci-
                                   fied endpoints
     der , input  ,  Highest  derivative  constraint,  i.e. the  routine
                     maintains the 1-st to the  der-th derivative at the
                     specified endpoints. This input is ignored if tnf =
                     NL_TANGENT
     SG  , input  ,  curs' stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatibleConstraints( NL_CURVE ** cur, NL_INDEX kk, NL_REAL tol, NL_FLAG whr, NL_FLAG tnf, NL_INDEX der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvsMakeCompatibleConstraints");

    NL_FLAG rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, l, ll, ii, jj, first, last, off, n, ns, m, r, s = 0, fout;

    NL_DEGREE p;

    NL_REAL ** br, ** er, ** te, *U, *a, *b, *minl, *maxl, *minr, *maxr, *max, *alfs, *omas, *bets, *ombs, wmin, wmax, pmax, tmp, alf, bet, omb, lam = 0.0, oml = 0.0, lto, bsum, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT ** Rw, *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input flags */

    switch( whr )
    {
        case NL_NO:
            break;

        case NL_START:
            break;

        case NL_END:
            break;

        case NL_BOTH:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    switch( tnf )
    {
        case NL_TANGENT:
            break;

        case NL_DERIVATIVE:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    /* Make curves exaCTLY compatible then remove unwanted knots */

    error = N_CrvsMakeCompatibleKnotTol( cur, kk, NL_PTOL, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( tol LT NL_PTOL )
        NL_OUT;

    /************************************/
    /* Remove as many knots as possible */
    /************************************/

    /* Adjust knot removal tolerance for rational curves */

    N_CrvGetCPtsDegreeAndKnots( cur[kk], &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur[kk], &knt );
    ns = n;

    if( N_IsCrvRat( cur[0] ) )
    {
        N_CrvGetMinMaxWeightsAndPts( cur[0], &wmin, &tmp, &tmp, &pmax );
        alf = (tol * wmin) / (1.0 + pmax);

        for ( ll = 1; ll <= kk; ll++ )
        {
            N_CrvGetMinMaxWeightsAndPts( cur[ll], &wmin, &tmp, &tmp, &pmax );
            bet = (tol * wmin) / (1.0 + pmax);

            if( bet LT alf )
                alf = bet;
        }
        tol = alf;
        rat = NL_YES;
    }

    lto = cto * fabs( U[m] - U[0] );

    /* Get local memory */

    Rw = N_AllocCPt2dArray( kk, 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal2dArray( kk, m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    b = N_AllocReal1dArray( kk, &SL );

    if( b EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( kk, m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( kk, m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    minl = N_AllocReal1dArray( p, &SL );

    if( minl EQ NULL )
        NL_QUIT;

    maxl = N_AllocReal1dArray( p, &SL );

    if( maxl EQ NULL )
        NL_QUIT;

    minr = N_AllocReal1dArray( p, &SL );

    if( minr EQ NULL )
        NL_QUIT;

    maxr = N_AllocReal1dArray( p, &SL );

    if( maxr EQ NULL )
        NL_QUIT;

    max = N_AllocReal1dArray( p + 1, &SL );

    if( max EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( 2 * p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( 2 * p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    bets = N_AllocReal1dArray( 2 * p, &SL );

    if( bets EQ NULL )
        NL_QUIT;

    ombs = N_AllocReal1dArray( 2 * p, &SL );

    if( ombs EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( j = 0; j <= m; j++ )
    {
        sr[j] = 0;

        for ( ll = 0; ll <= kk; ll++ )
        {
            br[ll][j] = NL_BIGD;
            er[ll][j] = 0.0;
        }
    }

    /* Compute knot removal errors for each distinct knot for each curve */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;
        sr[r] = r - i + 1;

        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvRemoveKnotMaxErr( cur[ll], r, sr[r], &br[ll][r] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        r++;
    }

    /* Try to remove each knot from each curve */

    while( NL_TRUE )
    {
        /* Find knot with smallest error sum */

        bsum = NL_BIGD;

        for ( j = p + 1; j <= m - p - 1; j++ )
        {
            if( br[0][j]NEQ NL_BIGD AND br[0][j]NEQ NOREM )
            {
                alf = 0.0;

                for ( ll = 0; ll <= kk; ll++ )
                    alf += br[ll][j];

                if( alf LT bsum )
                {
                    bsum = alf;
                    s = sr[j];
                    r = j;

                    for ( ll = 0; ll <= kk; ll++ )
                        b[ll] = br[ll][j];
                }
            }
        }

        /* If no more removable knot -> finished */

        if( bsum EQ NL_BIGD )
            break;

        /* Check constraints */

        if( tnf EQ NL_TANGENT )
        {
            if( p LE 2 )
                der = 1;
            else
                der = 0;
        }

        if( der GE 1 )
        {
            rmf = NL_TRUE;

            if( whr EQ NL_START OR whr EQ NL_BOTH )
            {
                if( r LE( p + der ) )
                    rmf = NL_FALSE;
            }

            if( whr EQ NL_END OR whr EQ NL_BOTH )
            {
                if( r GE( n - der + 1 ) )
                    rmf = NL_FALSE;
            }

            if( rmf EQ NL_FALSE )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    br[ll][r] = NOREM;
                continue;
            }
        }

        /* Check error of removal */

        rmf = NL_TRUE;

        if( (p + s) % 2 )
        {
            /* Compute maximums of basis functions over each span */

            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (U[r] - U[r - k]) / (U[r - k + p + 1] - U[r - k]);
            bet = (U[r] - U[r - k + 1]) / (U[r - k + p + 2] - U[r - k + 1]);
            omb = 1.0 - bet;
            lam = alf / (alf + bet);
            oml = 1.0 - lam;

            error = N_BasisFindAllSpanMaxima( knt, r - k, p, lto, minl, maxl, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                {
                    minl[i] = 0.0;
                    maxl[i] = 1.0;
                }
            }

            error = N_BasisFindAllSpanMaxima( knt, r - k + 1, p, lto, minr, maxr, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                {
                    minr[i] = 0.0;
                    maxr[i] = 1.0;
                }
            }

            max[0] = lam * alf * maxl[0];

            for ( i = 1; i <= p; i++ )
            {
                minl[i] *= lam * alf;
                minr[i - 1] *= oml * omb;
                maxl[i] *= lam * alf;
                maxr[i - 1] *= oml * omb;

                max[i] = NL_MAX( fabs( maxl[i] - minr[i - 1] ), fabs( maxr[i - 1] - minl[i] ) );
            }
            max[p + 1] = oml * omb * maxr[p];
        }
        else
        {
            /* Compute maximum of basis function */

            k = (p + s) / 2;
            l = r - k + p;

            error = N_BasisFindAllSpanMaxima( knt, r - k, p, lto, minl, max, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                    max[i] = 1.0;
            }
        }

        /* Check the error */

        for ( j = r - k; j <= l; j++ )
        {
            if( U[j]NEQ U[j + 1] )
            {
                for ( ll = 0; ll <= kk; ll++ )
                {
                    te[ll][j] = er[ll][j] + max[j - r + k] * b[ll];

                    if( te[ll][j]GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }
                }
            }

            if( rmf EQ NL_FALSE )
                break;
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( j = r - k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                        er[ll][j] = te[ll][j];
                }
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;

            /* Save alphas and betas */

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                alfs[i - first] = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                omas[i - first] = 1.0 - alfs[i - first];
                bets[j - first] = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                ombs[j - first] = 1.0 - bets[j - first];
                i++;
                j--;
            }

            /* Remove the knot from all curves */

            wfl = NL_TRUE;

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Pw[off], &Rw[ll][0] );
                N_CopyCPt( Pw[last + 1], &Rw[ll][last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT 0 )
                {
                    N_Combine2CPts( alfs[i - first], Pw[i], omas[i - first], Rw[ll][ii - 1], &Rw[ll][ii] );
                    N_Combine2CPts( bets[j - first], Pw[j], ombs[j - first], Rw[ll][jj + 1], &Rw[ll][jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Check for disallowed weights */

                if( rat EQ NL_YES )
                {
                    i = first;
                    j = last;
                    wmin = NL_BIGD;
                    wmax = NL_SMAD;

                    while( (j - i)GT 0 )
                    {
                        N_CPtGetW( Rw[ll][i - off], &wi );
                        N_CPtGetW( Rw[ll][j - off], &wj );

                        if( wi LT wmin )
                            wmin = wi;

                        if( wj LT wmin )
                            wmin = wj;

                        if( wi GT wmax )
                            wmax = wi;

                        if( wj GT wmax )
                            wmax = wj;
                        i++;
                        j--;
                    }

                    if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                    {
                        wfl = NL_FALSE;
                        break;
                    }
                }

                if( (p + s) % 2 )
                {
                    N_Combine2CPts( lam, Rw[ll][jj + 1], oml, Rw[ll][ii - 1], &Rw[ll][jj + 1] );
                }
            } /* End for each curve */

            /* Check weights */

            if( wfl EQ NL_FALSE )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    br[ll][r] = NOREM;
                continue;
            }
            else
            {
                /* Save control points */

                for ( ll = 0; ll <= kk; ll++ )
                {
                    N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        N_CopyCPt( Rw[ll][i - off], &Pw[i] );
                        N_CopyCPt( Rw[ll][j - off], &Pw[j] );
                        i++;
                        j--;
                    }
                }
            }

            /* Shift down some parameters */

            if( s EQ 1 )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    er[ll][r - 1] = NL_MAX( er[ll][r - 1], er[ll][r] );
            }

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( j = r + 1; j <= m; j++ )
            {
                sr[j - 1] = sr[j];

                for ( ll = 0; ll <= kk; ll++ )
                {
                    br[ll][j - 1] = br[ll][j];
                    er[ll][j - 1] = er[ll][j];
                }
            }

            /* Shift down knots and control points */

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &U );

                for ( j = r + 1; j <= m; j++ )
                    U[j - 1] = U[j];

                for ( j = fout + 1; j <= n; j++ )
                {
                    N_CopyCPt( Pw[j], &Pw[j - 1] );
                }

                N_CrvSetSizeIndices( cur[ll], n - 1, p, m - 1 );
            }
            n--;
            m--;

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( j = k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                    {
                        if( br[ll][j]NEQ NOREM )
                        {
                            error = N_CrvRemoveKnotMaxErr( cur[ll], j, sr[j], &br[ll][j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }
            }
        }
        else
        {
            /* Knot is not removable */

            for ( ll = 0; ll <= kk; ll++ )
                br[ll][r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvCompress( cur[ll], SG );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvsMakeCompatibleConstraints */

/*******************************************************************//**


   DESCRIPTION:

     This  tools routine makes a set of curves compatible using approxi-
     mation. More precisely, first it  makes curves compatible precisely
     followed by  removing as  many  knots as  possible. End  derivative 
     constraints or end tangent constraints can be specified. The curves 
     are made compatible in place, i.e. the  original control points are 
     destroyed. A typical calling example is:

       NL_INDEX   kk, der;
       NL_CURVE   **cur;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define array of curves and get tol and der);
       ...
       N_CrvsMakeCompatibleFast(cur,kk,tol,NL_BOTH,NL_TANGENT,der,&SG);

     ALL CURVES  MUST BE  ON  THE  SAME  STACK "SG"! THIS  VERSION  USES 
     APPROXIMATIVE NL_ERROR MEASURES AND HENCE IT IS FASTER THAN  N_CrvsMakeCompatibleConstraints. 
     HOWEVER, IT REMOVES SIGNIFICANTLY LESS KNOTS.


   ACCESS:
   
     cur , in/out ,  A set of NURBS curves
     kk  , input  ,  Highest index in cur
     tol , input  ,  Tolerance  to check  removability. tol=0.0 PROVIDES
                     PRECISE COMPATIBILITY!
     whr , input  ,  Flag:
                       NL_NO   : don't constrain end derivatives
                       NL_START: constrain at the start point
                       NL_END  : constrain at the end point
                       NL_BOTH : constrain at both ends
     tnf , input  ,  Flag:
                       NL_TANGENT   : constrain tangent directions  at  the
                                   specified endpoints  (magnitudes  may
                                   be changed)
                       NL_DERIVATIVE: constrain derivatives at  the  speci-
                                   fied endpoints
     der , input  ,  Highest  derivative  constraint,  i.e. the  routine
                     maintains the 1-st to the  der-th derivative at the
                     specified endpoints. This input is ignored if tnf =
                     NL_TANGENT
     SG  , input  ,  curs' stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvsMakeCompatibleFast( NL_CURVE ** cur, NL_INDEX kk, NL_REAL tol, NL_FLAG whr, NL_FLAG tnf, NL_INDEX der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvsMakeCompatibleFast");

    NL_FLAG rmf, wfl, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, l, ll, ii, jj, first, last, off, n, ns, m, r, s = 0, fout;

    NL_DEGREE p;

    NL_REAL ** br, ** er, ** te, *U, *a, *b, *alfs, *omas, *bets, *ombs, wmin, wmax, pmax, tmp, alf, bet, lam = 0.0, oml = 0.0, bsum, wi, wj;

    NL_CPOINT ** Rw, *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check input flags */

    switch( whr )
    {
        case NL_NO:
            break;

        case NL_START:
            break;

        case NL_END:
            break;

        case NL_BOTH:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

    switch( tnf )
    {
        case NL_TANGENT:
            break;

        case NL_DERIVATIVE:
            break;

        default:
            NL_ERROR( NL_INP_ERR );
    }

   /* Make curves exactly acompatible then remove unwanted knots */

    error = N_CrvsMakeCompatibleKnotTol( cur, kk, NL_PTOL, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( tol LT NL_PTOL )
        NL_OUT;

    /************************************/
    /* Remove as many knots as possible */
    /************************************/

    /* Adjust knot removal tolerance for rational curves */

    N_CrvGetCPtsDegreeAndKnots( cur[kk], &n, &Pw, &p, &m, &U );
    ns = n;

    if( N_IsCrvRat( cur[0] ) )
    {
        N_CrvGetMinMaxWeightsAndPts( cur[0], &wmin, &tmp, &tmp, &pmax );
        alf = (tol * wmin) / (1.0 + pmax);

        for ( ll = 1; ll <= kk; ll++ )
        {
            N_CrvGetMinMaxWeightsAndPts( cur[ll], &wmin, &tmp, &tmp, &pmax );
            bet = (tol * wmin) / (1.0 + pmax);

            if( bet LT alf )
                alf = bet;
        }
        tol = alf;
        rat = NL_YES;
    }

    /* Get local memory */

    Rw = N_AllocCPt2dArray( kk, 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal2dArray( kk, m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    b = N_AllocReal1dArray( kk, &SL );

    if( b EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal2dArray( kk, m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal2dArray( kk, m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( 2 * p, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( 2 * p, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    bets = N_AllocReal1dArray( 2 * p, &SL );

    if( bets EQ NULL )
        NL_QUIT;

    ombs = N_AllocReal1dArray( 2 * p, &SL );

    if( ombs EQ NULL )
        NL_QUIT;

    /* Initialize */

    for ( j = 0; j <= m; j++ )
    {
        sr[j] = 0;

        for ( ll = 0; ll <= kk; ll++ )
        {
            br[ll][j] = NL_BIGD;
            er[ll][j] = 0.0;
        }
    }

    /* Compute knot removal errors for each distinct knot for each curve */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND U[r]EQ U[r + 1] )
            r++;

        sr[r] = r - i + 1;

        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvRemoveKnotMaxErr( cur[ll], r, sr[r], &br[ll][r] );

            if( error EQ NL_YES )
                NL_OUT;
        }

        r++;
    }

    /* Try to remove each knot from each curve */

    while( NL_TRUE )
    {
        /* Find knot with smallest error sum */

        bsum = NL_BIGD;

        for ( j = p + 1; j <= m - p - 1; j++ )
        {
            if( br[0][j]NEQ NL_BIGD AND br[0][j]NEQ NOREM )
            {
                alf = 0.0;

                for ( ll = 0; ll <= kk; ll++ )
                    alf += br[ll][j];

                if( alf LT bsum )
                {
                    bsum = alf;
                    s = sr[j];
                    r = j;

                    for ( ll = 0; ll <= kk; ll++ )
                        b[ll] = br[ll][j];
                }
            }
        }

        /* If no more removable knot -> finished */

        if( bsum EQ NL_BIGD )
            break;

        /* Check constraints */

        if( tnf EQ NL_TANGENT )
        {
            if( p LE 2 )
                der = 1;
            else
                der = 0;
        }

        if( der GE 1 )
        {
            rmf = NL_TRUE;

            if( whr EQ NL_START OR whr EQ NL_BOTH )
            {
                if( r LE( p + der ) )
                    rmf = NL_FALSE;
            }

            if( whr EQ NL_END OR whr EQ NL_BOTH )
            {
                if( r GE( n - der + 1 ) )
                    rmf = NL_FALSE;
            }

            if( rmf EQ NL_FALSE )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    br[ll][r] = NOREM;
                continue;
            }
        }

        /* Check error of removal */

        if( (p + s) % 2 )
        {
            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (U[r] - U[r - k]) / (U[r - k + p + 1] - U[r - k]);
            bet = (U[r] - U[r - k + 1]) / (U[r - k + p + 2] - U[r - k + 1]);
            lam = alf / (alf + bet);
            oml = 1.0 - lam;
        }
        else
        {
            k = (p + s) / 2;
            l = r - k + p;
        }

        rmf = NL_TRUE;

        for ( j = r - k; j <= l; j++ )
        {
            if( U[j]NEQ U[j + 1] )
            {
                for ( ll = 0; ll <= kk; ll++ )
                {
                    te[ll][j] = er[ll][j] + b[ll];

                    if( te[ll][j]GT tol )
                    {
                        rmf = NL_FALSE;
                        break;
                    }
                }
            }

            if( rmf EQ NL_FALSE )
                break;
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( j = r - k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                        er[ll][j] = te[ll][j];
                }
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;

            /* Save alphas and betas */

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                alfs[i - first] = (U[i + p + 1] - U[i]) / (U[r] - U[i]);
                omas[i - first] = 1.0 - alfs[i - first];
                bets[j - first] = (U[j + p + 1] - U[j]) / (U[j + p + 1] - U[r]);
                ombs[j - first] = 1.0 - bets[j - first];
                i++;
                j--;
            }

            /* Remove the knot from all curves */

            wfl = NL_TRUE;

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                i = first;
                j = last;
                ii = 1;
                jj = last - off;

                N_CopyCPt( Pw[off], &Rw[ll][0] );
                N_CopyCPt( Pw[last + 1], &Rw[ll][last + 1 - off] );

                /* Get new control points for one removal step */

                while( (j - i)GT 0 )
                {
                    N_Combine2CPts( alfs[i - first], Pw[i], omas[i - first], Rw[ll][ii - 1], &Rw[ll][ii] );
                    N_Combine2CPts( bets[j - first], Pw[j], ombs[j - first], Rw[ll][jj + 1], &Rw[ll][jj] );
                    i++;
                    j--;
                    ii++;
                    jj--;
                }

                /* Check for disallowed weights */

                if( rat EQ NL_YES )
                {
                    i = first;
                    j = last;
                    wmin = NL_BIGD;
                    wmax = NL_SMAD;

                    while( (j - i)GT 0 )
                    {
                        N_CPtGetW( Rw[ll][i - off], &wi );
                        N_CPtGetW( Rw[ll][j - off], &wj );

                        if( wi LT wmin )
                            wmin = wi;

                        if( wj LT wmin )
                            wmin = wj;

                        if( wi GT wmax )
                            wmax = wi;

                        if( wj GT wmax )
                            wmax = wj;
                        i++;
                        j--;
                    }

                    if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                    {
                        wfl = NL_FALSE;
                        break;
                    }
                }

                if( (p + s) % 2 )
                {
                    N_Combine2CPts( lam, Rw[ll][jj + 1], oml, Rw[ll][ii - 1], &Rw[ll][jj + 1] );
                }
            } /* End for each curve */

            /* Check weights */

            if( wfl EQ NL_FALSE )
            {
                for ( ll = 0; ll <= kk; ll++ )
                    br[ll][r] = NOREM;
                continue;
            }
            else
            {
                /* Save control points */

                for ( ll = 0; ll <= kk; ll++ )
                {
                    N_CrvGetCPtsAndKnots( cur[ll], &Pw, &a );

                    i = first;
                    j = last;

                    while( (j - i)GT 0 )
                    {
                        N_CopyCPt( Rw[ll][i - off], &Pw[i] );
                        N_CopyCPt( Rw[ll][j - off], &Pw[j] );
                        i++;
                        j--;
                    }
                }
            }

            /* Shift down some parameters */

            if( s EQ 1 )
            {
                for ( ll = 0; ll <= kk; ll++ )
                {
                    er[ll][r - 1] = NL_MAX( er[ll][r - 1], er[ll][r] );
                }
            }

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( j = r + 1; j <= m; j++ )
            {
                sr[j - 1] = sr[j];

                for ( ll = 0; ll <= kk; ll++ )
                {
                    br[ll][j - 1] = br[ll][j];
                    er[ll][j - 1] = er[ll][j];
                }
            }

            /* Shift down knots and control points */

            for ( ll = 0; ll <= kk; ll++ )
            {
                N_CrvGetCPtsAndKnots( cur[ll], &Pw, &U );

                for ( j = r + 1; j <= m; j++ )
                    U[j - 1] = U[j];

                for ( j = fout + 1; j <= n; j++ )
                {
                    N_CopyCPt( Pw[j], &Pw[j - 1] );
                }

                N_CrvSetSizeIndices( cur[ll], n - 1, p, m - 1 );
            }
            n--;
            m--;

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( j = k; j <= l; j++ )
            {
                if( U[j]NEQ U[j + 1] )
                {
                    for ( ll = 0; ll <= kk; ll++ )
                    {
                        if( br[ll][j]NEQ NOREM )
                        {
                            error = N_CrvRemoveKnotMaxErr( cur[ll], j, sr[j], &br[ll][j] );

                            if( error EQ NL_YES )
                                NL_OUT;
                        }
                    }
                }
            }
        }
        else
        {
            /* Knot is not removable */

            for ( ll = 0; ll <= kk; ll++ )
                br[ll][r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        for ( ll = 0; ll <= kk; ll++ )
        {
            error = N_CrvCompress( cur[ll], SG );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvsMakeCompatibleFast */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable knots from a NURBS  curve
     with  possible  end  tangent  constraints. It  reparametrizes  the 
     curve with  respect to the arc length to be able to remove as many 
     knots as  possible. This  routine is  suitable for  curves stiched 
     together from  Bezier pieces  with fairly bad  parametrization. If 
     the output curve is  initialized to  NULL,  memory  to  store  new  
     control points and knots is allocated. If the output curve  is the  
     same as the input curve, knot removal is in place and the original 
     curve is destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SQ;
       ...
       (define curP, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveKnotsTangentConstraints(&curP,tol,NL_START,&curQ,&SQ);
       N_CrvRemoveKnotsTangentConstraints(&curP,tol,NL_BOTH ,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon  objects. THE ROUTINE USES  0.1%  NL_RELATIVE
     TOLERANCE TO APPROXIMATE ARC LENGTH.


   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance 
     whr  , input  ,  Flag:
                        NL_NO   : don't constrain end tangents
                        NL_START: constrain at the start point
                        NL_END  : constrain at the end point
                        NL_BOTH : constrain at both ends
     curQ , output ,  Curve after cleaning
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveKnotsTangentConstraints( NL_CURVE *curP, NL_REAL tol, NL_FLAG whr, NL_CURVE *curQ, NL_STACKS *SQ )
{

    NL_FLAG error = NL_NO;

    /* Reparametrize and clean */

    if( curP EQ curQ )
    {
        error = N_SrfReparamMultKnots( curP, eps, curP, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvRemoveKnotsDerivConstraints( curP, tol, whr, 1, curP, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_SrfReparamMultKnots( curP, eps, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvRemoveKnotsDerivConstraints( curQ, tol, whr, 1, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvRemoveKnotsTangentConstraints */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This tools routine removes all removable  knots from a NURBS curve
     with possible end derivative  constraints applied. If  the  output 
     curve is initialized to NULL, memory  to  store new control points 
     and knots  is  allocated. If the  output curve is  the same as the 
     input curve, knot removal is done in place  and the original curve 
     is destroyed. A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_INDEX   der;
       NL_REAL    tol;
       NL_STACKS  SQ;
       ...
       (define curP and der, get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvRemoveKnots(&curP,tol,NL_START,der,&curQ,&SQ);
       N_CrvRemoveKnots(&curP,tol,NL_BOTH ,der,&curP,&SQ);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that memory allocation  has been done.  However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects. 

   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Tolerance to check removability
     whr  , input  ,  Flag:
                        NL_NO   : don't constrain end derivatives
                        NL_START: constrain at the start point
                        NL_END  : constrain at the end point
                        NL_BOTH : constrain at both ends
     der  , input  ,  Highest  derivative  constraint, i.e. the routine
                      maintains the  1-st to the  der-th derivatives at 
                      the specified end point(s)    
     curQ , output ,  Curve after knot removal
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRemoveKnotsDerivConstraints( NL_CURVE *curP, NL_REAL tol, NL_FLAG whr, NL_INDEX der, NL_CURVE *curQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRemoveKnotsDerivConstraints");

    NL_FLAG rmf, rat = NL_NO, error = NL_NO;

    NL_INDEX *sr, i, j, k, ii, jj, first, last, off, n, m, r, s, fout, l, ns;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *br, *er, *te, *minl, *maxl, *minr, *maxr, *max, wmin, wmax, pmax, tmp, b, alf, oma, bet, omb, lam = 0.0, oml = 0.0, lto, wi, wj;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );
    ns = n;

    /* Adjust removal tolerance in case of rational curves */

    if( N_IsCrvRat( curP ) )
    {
        N_CrvGetMinMaxWeightsAndPts( curP, &wmin, &tmp, &tmp, &pmax );
        tol = (tol * wmin) / (1.0 + pmax);
        rat = NL_YES;
    }

    lto = cto * fabs( UP[m] - UP[0] );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsKnotVectorAndKnots( curP, &Qw, &knt, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsKnotVectorAndKnots( curQ, &Qw, &knt, &UQ );
    }

    /* Get local memory */

    Rw = N_AllocCPt1dArray( 2 * p, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    br = N_AllocReal1dArray( m, &SL );

    if( br EQ NULL )
        NL_QUIT;

    sr = N_AllocInt1dArray( m, &SL );

    if( sr EQ NULL )
        NL_QUIT;

    er = N_AllocReal1dArray( m, &SL );

    if( er EQ NULL )
        NL_QUIT;

    te = N_AllocReal1dArray( m, &SL );

    if( te EQ NULL )
        NL_QUIT;

    minl = N_AllocReal1dArray( p, &SL );

    if( minl EQ NULL )
        NL_QUIT;

    maxl = N_AllocReal1dArray( p, &SL );

    if( maxl EQ NULL )
        NL_QUIT;

    minr = N_AllocReal1dArray( p, &SL );

    if( minr EQ NULL )
        NL_QUIT;

    maxr = N_AllocReal1dArray( p, &SL );

    if( maxr EQ NULL )
        NL_QUIT;

    max = N_AllocReal1dArray( p + 1, &SL );

    if( max EQ NULL )
        NL_QUIT;

    /* Initialize */

    if( curP NEQ curQ )
    {
        error = N_CrvCopy( curP, curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    for ( i = 0; i <= m; i++ )
    {
        br[i] = NL_BIGD;
        sr[i] = 0;
        er[i] = 0.0;
    }

    /* Compute knot removal errors for each distinct knot */

    r = p + 1;

    while( r LE n )
    {
        i = r;

        while( r LE n AND UQ[r]EQ UQ[r + 1] )
            r++;
        sr[r] = r - i + 1;

        error = N_CrvRemoveKnotMaxErr( curP, r, sr[r], &br[r] );

        if( error EQ NL_YES )
            NL_OUT;

        r++;
    }

    /* Try to remove each knot */

    while( NL_TRUE )
    {
        /* Find knot with smallest error */

        b = br[p + 1];
        s = sr[p + 1];
        r = p + 1;

        for ( i = p + 2; i <= m - p - 1; i++ )
        {
            if( br[i]LT b )
            {
                b = br[i];
                s = sr[i];
                r = i;
            }
        }

        /* If no more removable knot -> finished */

        if( b EQ NL_BIGD OR b EQ NOREM )
            break;

        /* Check constraints */

        rmf = NL_TRUE;

        if( whr EQ NL_START OR whr EQ NL_BOTH )
        {
            if( r LE( p + der ) )
                rmf = NL_FALSE;
        }

        if( whr EQ NL_END OR whr EQ NL_BOTH )
        {
            if( r GE( n - der + 1 ) )
                rmf = NL_FALSE;
        }

        if( rmf EQ NL_FALSE )
        {
            br[r] = NOREM;
            continue;
        }

        /* Check error of removal */

        rmf = NL_TRUE;

        if( (p + s) % 2 )
        {
            /* Compute maximums of basis functions over each span */

            k = (p + s + 1) / 2;
            l = r - k + p + 1;
            alf = (UQ[r] - UQ[r - k]) / (UQ[r - k + p + 1] - UQ[r - k]);
            bet = (UQ[r] - UQ[r - k + 1]) / (UQ[r - k + p + 2] - UQ[r - k + 1]);
            omb = 1.0 - bet;
            lam = alf / (alf + bet);
            oml = 1.0 - lam;

            error = N_BasisFindAllSpanMaxima( knt, r - k, p, lto, minl, maxl, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                {
                    minl[i] = 0.0;
                    maxl[i] = 1.0;
                }
            }

            error = N_BasisFindAllSpanMaxima( knt, r - k + 1, p, lto, minr, maxr, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                {
                    minr[i] = 0.0;
                    maxr[i] = 1.0;
                }
            }

            max[0] = lam * alf * maxl[0];

            for ( i = 1; i <= p; i++ )
            {
                minl[i] *= lam * alf;
                minr[i - 1] *= oml * omb;
                maxl[i] *= lam * alf;
                maxr[i - 1] *= oml * omb;

                max[i] = NL_MAX( fabs( maxl[i] - minr[i - 1] ), fabs( maxr[i - 1] - minl[i] ) );
            }
            max[p + 1] = oml * omb * maxr[p];
        }
        else
        {
            /* Compute maximum of basis function */

            k = (p + s) / 2;
            l = r - k + p;

            error = N_BasisFindAllSpanMaxima( knt, r - k, p, lto, minl, max, &tmp );

            if( error EQ NL_YES )
            {
                for ( i = 0; i <= p; i++ )
                    max[i] = 1.0;
            }
        }

        /* Check the error */

        for ( i = r - k; i <= l; i++ )
        {
            if( UQ[i]NEQ UQ[i + 1] )
            {
                te[i] = er[i] + max[i - r + k] * b;

                if( te[i]GT tol )
                {
                    rmf = NL_FALSE;
                    break;
                }
            }
        }

        /* If error test passed -> update error vector and remove knot */

        if( rmf EQ NL_TRUE )
        {
            for ( i = r - k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1] )
                    er[i] = te[i];
            }

            fout = (2 * r - s - p) / 2;
            first = r - p;
            last = r - s;
            off = first - 1;
            i = first;
            j = last;
            ii = 1;
            jj = last - off;

            N_CopyCPt( Qw[off], &Rw[0] );
            N_CopyCPt( Qw[last + 1], &Rw[last + 1 - off] );

            /* Get new control points for one removal step */

            while( (j - i)GT 0 )
            {
                alf = (UQ[i + p + 1] - UQ[i]) / (UQ[r] - UQ[i]);
                oma = 1.0 - alf;
                bet = (UQ[j + p + 1] - UQ[j]) / (UQ[j + p + 1] - UQ[r]);
                omb = 1.0 - bet;
                N_Combine2CPts( alf, Qw[i], oma, Rw[ii - 1], &Rw[ii] );
                N_Combine2CPts( bet, Qw[j], omb, Rw[jj + 1], &Rw[jj] );
                i++;
                j--;
                ii++;
                jj--;
            }

            /* Check for disallowed weights */

            if( rat EQ NL_YES )
            {
                i = first;
                j = last;
                wmin = NL_BIGD;
                wmax = NL_SMAD;

                while( (j - i)GT 0 )
                {
                    N_CPtGetW( Rw[i - off], &wi );
                    N_CPtGetW( Rw[j - off], &wj );

                    if( wi LT wmin )
                        wmin = wi;

                    if( wj LT wmin )
                        wmin = wj;

                    if( wi GT wmax )
                        wmax = wi;

                    if( wj GT wmax )
                        wmax = wj;
                    i++;
                    j--;
                }

                if( wmin LT NL_WMIN OR wmax GT NL_WMAX )
                {
                    br[r] = NOREM;
                    continue;
                }
            }

            /* Save new control points */

            if( (p + s) % 2 )
            {
                N_Combine2CPts( lam, Rw[jj + 1], oml, Rw[ii - 1], &Rw[jj + 1] );
            }

            i = first;
            j = last;

            while( (j - i)GT 0 )
            {
                N_CopyCPt( Rw[i - off], &Qw[i] );
                N_CopyCPt( Rw[j - off], &Qw[j] );
                i++;
                j--;
            }

            /* Shift down some parameters */

            if( s EQ 1 )
                er[r - 1] = NL_MAX( er[r - 1], er[r] );

            if( s GT 1 )
                sr[r - 1] = sr[r] - 1;

            for ( i = r + 1; i <= m; i++ )
            {
                br[i - 1] = br[i];
                sr[i - 1] = sr[i];
                er[i - 1] = er[i];
            }

            /* Shift down knots and control points */

            for ( i = r + 1; i <= m; i++ )
                UQ[i - 1] = UQ[i];

            for ( i = fout + 1; i <= n; i++ )
                N_CopyCPt( Qw[i], &Qw[i - 1] );

            n--;
            m--;
            N_CrvSetSizeIndices( curQ, n, p, m );

            /* If no more internal knots -> finished */

            if( n EQ p )
                break;

            /* Update error bounds */

            k = NL_MAX( r - p, p + 1 );
            l = NL_MIN( n, r + p - s );

            for ( i = k; i <= l; i++ )
            {
                if( UQ[i]NEQ UQ[i + 1]AND br[i]NEQ NOREM )
                {
                    error = N_CrvRemoveKnotMaxErr( curQ, i, sr[i], &br[i] );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
            }
        }
        else
        {
            /* Knot is not removable */

            br[r] = NOREM;
        }
    } /* End of while loop */

    /* Compact output curve */

    if( n LT ns )
    {
        error = N_CrvCompress( curQ, SQ );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRemoveKnotsDerivConstraints */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine makes a curve compatible with one direction of
     a surface, i.e. it raises the degrees and inserts knots until the
     two have the same degree and are defined over the same knot  vec-
     tor. A typical calling example is:

       NL_CURVE      cur;
       NL_SURFACE    sur;
       NL_PARAMETER  ts, te;
       NL_STACKS     S;
       ...
       (define cur and sur, and choose ts and te);
       ...
       N_CrvMakeCompatibleWithSrf(&cur,&sur,NL_UDIR,ts,te,&S); 

     THE ALGORITHM WORKS IN PLACE, I.E. THE ORIGINAL NL_CURVE AND NL_SURFACE
     ARE DESTROYED!  THE  NL_CURVE  AND  NL_SURFACE  MUST BELONG TO THE SAME 
     STACK!

   ACCESS:
   
     cur   , in/out ,  The curve
     sur   , in/out ,  The surface
     dir   , input  ,  Flag:
                       NL_UDIR: make cur and sur's u-direction compatible
                       NL_VDIR: make cur and sur's v-direction compatible
     ts,te , input  ,  Reparameterize cur and sur's relevant direction
                       to the interval [ts,te]
     S     , input  ,  cur's and sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvMakeCompatibleWithSrf( NL_CURVE *cur, NL_SURFACE *sur, NL_FLAG dir, NL_PARAMETER ts, NL_PARAMETER te, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX nc, mc, ms, nn, mm, rr, ss;

    NL_DEGREE pc, ps, pp, qq;

    NL_REAL *US, *UC, *UU, *VV;

    NL_CPOINT *Cw, ** Sw;

    NL_BOOLEAN ratc, rats;

    NL_KNOTVECTOR *knt[2], ** knx, *dum;

    NL_INTERVAL pint;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &nc, &Cw, &pc, &mc, &UC );
    N_CrvGetKnotVector( cur, &knt[0] );
    N_SrfGetCPtsDegreesAndKnots( sur, &nn, &mm, &Sw, &pp, &qq, &rr, &ss, &UU, &VV );

    /* Make rationality of each compatible */

    ratc = N_IsCrvRat( cur );
    rats = N_IsSrfRat( sur );

    if( ratc AND( NOT rats ) )
        N_SrfNonRatToRat( sur );

    if( rats AND( NOT ratc ) )
        N_CrvNonRatToRat( cur );

    /* Set u/v-directional data */

    if( dir EQ NL_UDIR )
    {
        N_SrfGetKnotVectors( sur, &knt[1], &dum );

        ms = rr;
        ps = pp;
        US = UU;
    }
    else
    {
        N_SrfGetKnotVectors( sur, &dum, &knt[1] );

        ms = ss;
        ps = qq;
        US = VV;
    }

    /* Reparameterize to a common interval */

    N_CreateInterval( &pint, ts, te );

    if( UC[0]NEQ ts OR UC[mc]NEQ te )
        N_BasisReparam( knt[0], pc, pint );

    if( US[0]NEQ ts OR US[ms]NEQ te )
        N_BasisReparam( knt[1], ps, pint );

    /* Degree elevate if necessary */

    if( pc LT ps )
    {
        error = N_CrvElevateDegree( cur, ps - pc, cur, S, S );
        N_CrvGetKnotVector( cur, &knt[0] );
    }
    else if( ps LT pc )
    {
        error = N_SrfElevateDegree( sur, pc - ps, dir, sur, S, S );

        if( dir EQ NL_UDIR )
            N_SrfGetKnotVectors( sur, &knt[1], &dum );
        else
            N_SrfGetKnotVectors( sur, &dum, &knt[1] );
    }

    if( error EQ NL_YES )
        NL_OUT;

    /* Merge knot vectors */
    error = N_GetCompatibleKnotArray( knt, 1, &knx, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Refine curve and/or surface */
    N_KnotVectorGetKnots( knx[0], &mm, &UU );

    if( mm GE 0 )
        error = N_CrvRefine( cur, knx[0], cur, S, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx[1], &mm, &UU );

    if( mm GE 0 )
        error = N_SrfInsertKnots( sur, knx[1], dir, sur, S, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvMakeCompatibleWithSrf */



/*******************************************************************//**


   DESCRIPTION:

     This tools routine refines a NURBS curve with a given knot vector.
     This version of knot refinement computes the  control points only.
     It is assumed that the  new knot vector  "fits" into the  old one, 
     i.e. U[0] < X[0] <=...<= X[r] < U[m] holds where U[0],...,U[m] are
     the  old  knots  and  X[0],...,X[r]  are the  new  ones. A typical 
     calling example is:

       NL_CURVE    cur;
       NL_REAL     *X;
       NL_INDEX    r;
       NL_CPOINT   *Qw;
       ...
       (define cur, X and allocate memory for Qw);
       ...
       N_CrvRefineToKnotVector(&cur,X,r,Qw);

     IT IS  ASSUMED THAT MEMORY TO STORE  THE NL_NEW  CONTROL NL_POINTS Qw IS 
     ALLOCATED IN THE CALLING ROUTINE!!!


   ACCESS:
   
     cur , input  ,  NURBS curve
     X   , input  ,  New knots
     r   , input  ,  Highest index in X
     Qw  , output ,  New  control points. IF n IS  THE HIGHEST NL_INDEX OF
                     CONTROL NL_POINTS IN  cur, THEN  Qw MUST  HAVE MEMORY
                     TO HOLD Qw[0],...,Qw[n+r+1].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRefineToKnotVector( NL_CURVE *cur, NL_REAL *X, NL_INDEX r, NL_CPOINT *Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvRefineToKnotVector");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, a, b, n, m, t;

    NL_DEGREE p;

    NL_REAL *U, *UQ, alf, oma;

    NL_KNOTVECTOR *knt;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CrvGetKnotVector( cur, &knt );

    /* Check input parameters  */

    if( r LT 0 )
        NL_ERROR( NL_INP_ERR );

    if( X[0]LE U[0] )
        NL_ERROR( NL_INP_ERR );

    if( X[r]GE U[m] )
        NL_ERROR( NL_INP_ERR );

    /* Find knot spans */

    error = N_BasisFindSpan( knt, p, X[0], NL_LEFT, &a );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisFindSpan( knt, p, X[r], NL_LEFT, &b );

    if( error EQ NL_YES )
        NL_OUT;
    b++;

    /* Compute new control points */

    UQ = N_AllocReal1dArray( m + r + 1, &SL );

    if( UQ EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= a; j++ )
        UQ[j] = U[j];

    for ( j = b + p; j <= m; j++ )
        UQ[j + r + 1] = U[j];

    for ( j = 0; j <= a - p; j++ )
        N_CopyCPt( Pw[j], &Qw[j] );

    for ( j = b - 1; j <= n; j++ )
        N_CopyCPt( Pw[j], &Qw[j + r + 1] );

    i = b + p - 1;
    k = b + p + r;

    for ( j = r; j >= 0; j-- )
    {
        while( X[j]LE U[i]AND i GT a )
        {
            N_CopyCPt( Pw[i - p - 1], &Qw[k - p - 1] );
            UQ[k] = U[i];
            k--;
            i--;
        }

        N_CopyCPt( Qw[k - p], &Qw[k - p - 1] );

        for ( l = 1; l <= p; l++ )
        {
            t = k - p + l;
            alf = UQ[k + l] - X[j];

            if( fabs( alf )LE 0.0 )
            {
                N_CopyCPt( Qw[t], &Qw[t - 1] );
            }
            else
            {
                alf = alf / (UQ[k + l] - U[i - p + l]);
                oma = 1.0 - alf;
                N_Combine2CPts( alf, Qw[t - 1], oma, Qw[t], &Qw[t - 1] );
            }
        }
        UQ[k] = X[j];
        k--;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRefineToKnotVector */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes an upper bound on the 
     first derivative of a curve. A typical calling example is:

       NL_CURVE  cur;
       NL_POINT  Cp;
       NL_REAL   mag; 
       ...
       (define cur);
       ...
       N_CrvGetMaxFirstDeriv(&cur,&Cp,&mag);


   ACCESS:
   
     cur , input  ,  NURBS curve
     Cp  , output ,  Maximum first derivative vector
     mag , output ,  Magnitude of Cp


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetMaxFirstDeriv( NL_CURVE *cur, NL_POINT *Cp, NL_REAL *mag )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_INDEX i, k, m, n;

    NL_REAL len;

    NL_POINT D;

    NL_CURVE ** bez, *der;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and allocate memory */

    N_CrvGetDegree( cur, &p );

    *mag = 0.0;
    N_CopyPt( NL_ZERO, Cp );

    if( p LT 1 )
        NL_OUT;

    p = 2 * p;
    n = p;
    m = n + p + 1;

    der = N_AllocCrvAndArrays( n, p, m, &SL );

    if( der EQ NULL )
        NL_QUIT;

    /* Get Bezier pieces */

    error = N_CrvDecomposeBez( cur, &bez, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    for ( i = 0; i <= k; i++ )
    {
        error = N_CrvRatGetFirstDeriv( bez[i], der, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetMaxPosVector( der, &D, &len );

        if( len GT *mag )
        {
            *mag = len;
            N_CopyPt( D, Cp );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetMaxFirstDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes an upper bound on the 
     second derivative of a curve. A typical calling example is:

       NL_CURVE  cur;
       NL_POINT  Cpp;
       NL_REAL   mag; 
       ...
       (define cur);
       ...
       N_CrvGetMaxSecondDeriv(&cur,&Cpp,&mag);


   ACCESS:
   
     cur , input  ,  NURBS curve
     Cpp , output ,  Maximum second derivative vector
     mag , output ,  Magnitude of Cpp


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetMaxSecondDeriv( NL_CURVE *cur, NL_POINT *Cpp, NL_REAL *mag )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p;

    NL_INDEX i, k, m, n;

    NL_REAL len;

    NL_POINT D;

    NL_CURVE ** bez, *der;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and allocate memory */

    N_CrvGetDegree( cur, &p );

    *mag = 0.0;
    N_CopyPt( NL_ZERO, Cpp );

    if( p LT 2 )
        NL_OUT;

    p = 3 * p;
    n = p;
    m = n + p + 1;

    der = N_AllocCrvAndArrays( n, p, m, &SL );

    if( der EQ NULL )
        NL_QUIT;

    /* Get Bezier pieces */

    error = N_CrvDecomposeBez( cur, &bez, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute maximum derivative */

    for ( i = 0; i <= k; i++ )
    {
        error = N_CrvGetRatSecondDeriv( bez[i], der, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetMaxPosVector( der, &D, &len );

        if( len GT *mag )
        {
            *mag = len;
            N_CopyPt( D, Cpp );
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetMaxSecondDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the second derivative 
     curve of a rational curve. The result is  obtained by  using other 
     symbolic operators to compute the following expression:

         ( N )''   D*D*N''- 2*D*D'*N'+N*(2*D'*D'-D*D'')
         ( - )   = ------------------------------------
         ( D )                   D*D*D     

     A typical calling example is:

       NL_CURVE   cur, der;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&der);
       N_CrvGetRatSecondDeriv(&cur,&der,&SG);


   ACCESS:
   
     cur , input  ,  NURBS curve 
     der , output ,  Derivative of cur
     SG  , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetRatSecondDeriv( NL_CURVE *cur, NL_CURVE *der, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX nsp, mx;

    NL_DEGREE p;

    NL_REAL *X;

    NL_CURVE n, np, npp, ddnpp, ddpnp, ddppn, nsum, num;

    NL_CFUN d, dd, dp, dpp, ddp, dpdp, ddpp, dsum, den;

    NL_KNOTVECTOR *knd, *knx;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsCrvRat( cur ) )
    {
        error = N_CrvNonRatGetFirstDeriv( cur, 2, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_CrvInitArrays( &n );
    N_CFuncInitArrays( &d );
    error = N_CrvGetNumAndDenom( cur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output curve */

    N_CFuncInitArrays( &dd );
    error = N_CrvFuncMultiplyCrvFunc( &d, &d, &dd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &den );
    error = N_CrvFuncMultiplyCrvFunc( &d, &dd, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncGetKnotVector( &den, &knd );
    N_CFuncGetDegree( &den, &p );
    N_BasisGetSpanCount( knd, p, &nsp );

    knx = N_AllocKnotVectorAndArray( 2 * nsp, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knd, p, 2, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &mx, &X );

    if( mx GE 0 )
    {
        error = N_CrvFuncRefine( &den, knx, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute terms in the numerator */

    /*--------- DDN'' ------------*/

    N_CrvInitArrays( &npp );
    error = N_CrvNonRatGetFirstDeriv( &n, 2, &npp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ddnpp );
    error = N_CrvFuncMultiplyCrv( &dd, &npp, &ddnpp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*--------- 2DD'N' ------------*/

    N_CFuncInitArrays( &dp );
    error = N_CrvFuncDeriv( &d, 1, &dp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &ddp );
    error = N_CrvFuncMultiplyCrvFunc( &d, &dp, &ddp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncMultiplyConstant( 2.0, &ddp );

    N_CrvInitArrays( &np );
    error = N_CrvNonRatGetFirstDeriv( &n, 1, &np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ddpnp );
    error = N_CrvFuncMultiplyCrv( &ddp, &np, &ddpnp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnotVector( &ddpnp, &knd );
    N_CrvGetDegree( &ddpnp, &p );
    N_BasisGetSpanCount( knd, p, &nsp );

    knx = N_AllocKnotVectorAndArray( nsp, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knd, p, 1, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &mx, &X );

    if( mx GE 0 )
    {
        error = N_CrvRefine( &ddpnp, knx, &ddpnp, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /*--------- N(2D'D'-DD'') ------------*/

    N_CFuncInitArrays( &dpdp );
    error = N_CrvFuncMultiplyCrvFunc( &dp, &dp, &dpdp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncMultiplyConstant( 2.0, &dpdp );

    N_CFuncGetKnotVector( &dpdp, &knd );
    N_CFuncGetDegree( &dpdp, &p );
    N_BasisGetSpanCount( knd, p, &nsp );

    knx = N_AllocKnotVectorAndArray( nsp, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knd, p, 1, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &mx, &X );

    if( mx GE 0 )
    {
        error = N_CrvFuncRefine( &dpdp, knx, &dpdp, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    N_CFuncInitArrays( &dpp );
    error = N_CrvFuncDeriv( &d, 2, &dpp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &ddpp );
    error = N_CrvFuncMultiplyCrvFunc( &d, &dpp, &ddpp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &dsum );
    error = N_CrvFuncSumDiffCrvFunc( &dpdp, &ddpp, NL_MINUS, NL_YES, &dsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ddppn );
    error = N_CrvFuncMultiplyCrv( &dsum, &n, &ddppn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_CrvInitArrays( &nsum );
    error = N_CrvSumDiffCrv( &ddnpp, &ddpnp, NL_MINUS, &nsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &num );
    error = N_CrvSumDiffCrv( &nsum, &ddppn, NL_PLUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Degree elevate numerator */

    error = N_CrvElevateDegree( &num, 2, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output curve */

    error = N_CreateCrvFromNumAndDenom( &num, &den, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetRatSecondDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators  routine computes  the dot product of two
     NURBS curves. The dot  product is a rational  function if at least 
     one of the curves is rational. Otherwise, the  product function is 
     non-rational.  The knot vectors of both curves are rescaled to the 
     unit span [0,1]. This  changes the position of the knots, however, 
     it   leaves   the   curves   unaltered  both   parametrically  and  
     geometrically. A typical calling example is:

       NL_CURVE   curF, curG;
       NL_CFUN    num, den;
       NL_STACKS  SG;
       ...
       (define curF and curG);
       ...
       N_CFuncInitArrays(&num);
       N_CFuncInitArrays(&den);
       N_CrvDotCrv(&curF,&curG,&num,&den,&SG);

     If memory is available,  num and  den are not initialized and  the 
     routine assumes that memory  allocation has been done. However, it 
     checks for the proper  amount by looking at the highest indexes in 
     num's and den's knot vector and control value objects. THE STORAGE
     OF THE OUTPUT FUNCTIONS IS COMPACTED, I.E. THE MEMORY PASSED IN IS
     DESTROYED.


   ACCESS:
   
     curF , in/out ,  Curve (ITS KNOT NL_VECTOR IS RESCALED)
     curG , in/out ,  Curve (ITS KNOT NL_VECTOR IS RESCALED)
     num  , output ,  Product of the numerators
     den  , output ,  Product of the denominators (if rational)
     SG   , input  ,  num's and den's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvDotCrv( NL_CURVE *curF, NL_CURVE *curG, NL_CFUN *num, NL_CFUN *den, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvDotCrv");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX i, j, k, l, nf, mr, ng, ms, nh, mt, mlr, mls, mir, mis, mi, rr, rs, s, ar, as, br, bs, mxr, mxs, ixr, ixs, t, kind, lind, nind, dind, first, last, rem, save, nsr, nss, lbz, rbz;

    NL_DEGREE p, q, pq;

    NL_REAL *R, *S, *N, *D = NULL, *XR, *XS, *fn, *fd = NULL, *dot, *dow=NULL, *alfr, *omar, *alfs, *omas, alf, oma, bet, omb, gam, omg, dw, numer, denom;

    NL_CPOINT *Fw, *Gw, *FBw, *GBw, *FNw, *GNw;

    NL_KNOTVECTOR *knr, *kns, *kxr, *kxs;

    NL_CURVE curFR, curGR;

    NL_RMATRIX pm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get new degree and set flag */

    N_CrvGetDegree( curF, &p );
    N_CrvGetDegree( curG, &q );

    pq = p + q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    if( N_IsCrvRat( curF ) )
        rat = NL_YES;

    else if( N_IsCrvRat( curG ) )
        rat = NL_YES;

    /* Handle special case p=0 or q=0 */

    if( p EQ 0 OR q EQ 0 )
    {
        N_CrvGetArraySizes( curF, &i, &j );
        N_CrvGetArraySizes( curG, &k, &l );

        if( i NEQ p OR k NEQ q )
            NL_ERROR( NL_INP_ERR );

        N_CrvGetCPtsAndKnots( curF, &Fw, &R );
        N_CrvGetCPtsAndKnots( curG, &Gw, &S );

        error = N_CFuncSizeArrays( num, pq, pq, pq + pq + 1, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( num, &fn, &N );

        if( rat EQ NL_YES )
        {
            error = N_CFuncSizeArrays( den, pq, pq, pq + pq + 1, rname, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvFuncCntrlValKnots( den, &fd, &D );
        }

        if( p EQ 0 )
        {
            for ( j = 0; j <= q; j++ )
            {
                N_Dot2CPts( Fw[0], Gw[j], &fn[j], &dw );

                if( rat EQ NL_YES )
                    fd[j] = dw;

                N[j] = S[j];
                N[q + j + 1] = S[q + j + 1];

                if( rat EQ NL_YES )
                {
                    D[j] = S[j];
                    D[q + j + 1] = S[q + j + 1];
                }
            }
        }
        else
        {
            for ( i = 0; i <= p; i++ )
            {
                N_Dot2CPts( Fw[i], Gw[0], &fn[i], &dw );

                if( rat EQ NL_YES )
                    fd[i] = dw;

                N[i] = R[i];
                N[p + i + 1] = R[p + i + 1];

                if( rat EQ NL_YES )
                {
                    D[i] = R[i];
                    D[p + i + 1] = R[p + i + 1];
                }
            }
        }

        NL_OUT;
    }

    /* Merge knots */

    N_CrvGetKnotVector( curF, &knr );
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

    /* Refine curves */

    N_KnotVectorGetKnots( kxr, &mxr, &XR );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( mxr GE 0 )
    {
        N_CrvInitArrays( &curFR );
        error = N_CrvRefine( curF, kxr, &curFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsDegreeAndKnots( &curFR, &nf, &Fw, &p, &mr, &R );
        N_CrvGetKnotVector( &curFR, &knr );
        N_BasisGetSpanCount( knr, p, &nsr );
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( curF, &nf, &Fw, &p, &mr, &R );
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

    error = N_CFuncSizeArrays( num, nh, pq, mt, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncCntrlValKnots( num, &fn, &N );

    if( rat EQ NL_YES )
    {
        error = N_CFuncSizeArrays( den, nh, pq, mt, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvFuncCntrlValKnots( den, &fd, &D );
    }

    /* Get local memory */

    FBw = N_AllocCPt1dArray( p, &SL );

    if( FBw EQ NULL )
        NL_QUIT;

    FNw = N_AllocCPt1dArray( p, &SL );

    if( FNw EQ NULL )
        NL_QUIT;

    GBw = N_AllocCPt1dArray( q, &SL );

    if( GBw EQ NULL )
        NL_QUIT;

    GNw = N_AllocCPt1dArray( q, &SL );

    if( GNw EQ NULL )
        NL_QUIT;

    dot = N_AllocReal1dArray( pq, &SL );

    if( dot EQ NULL )
        NL_QUIT;

    if( rat EQ NL_YES )
    {
        dow = N_AllocReal1dArray( pq, &SL );

        if( dow EQ NULL )
            NL_QUIT;
    }

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

    nind = 1;
    dind = 1;
    kind = pq + 1;
    lind = pq + 1;
    rem = -1;

    for ( i = 0; i <= pq; i++ )
        N[i] = S[as];

    if( rat EQ NL_YES )
        for ( i = 0; i <= pq; i++ )
            D[i] = S[as];

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Fw[i], &FBw[i] );

    for ( j = 0; j <= q; j++ )
        N_CopyCPt( Gw[j], &GBw[j] );

    N_Dot2CPts( Fw[0], Gw[0], &fn[0], &dw );

    if( rat EQ NL_YES )
        fd[0] = dw;

    N_InitRealMatrix( &pm );
    error = N_BezFuncMultiplyBezCrvMatrix( p, q, &pm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /***************************************************************/
    /* Loop through the knot vectors and do the following:         */
    /*   (1) Extract the i-th Bezier curves.                       */
    /*   (2) Compute their dot product.                            */
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

        /* Insert knot to get Bezier segments */

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
            numer = R[br] - R[ar];

            for ( k = p; k > mlr; k-- )
            {
                alfr[k - mlr - 1] = numer / (R[ar + k] - R[ar]);
                omar[k - mlr - 1] = 1.0 - alfr[k - mlr - 1];
            }

            for ( j = 1; j <= rr; j++ )
            {
                save = rr - j;
                s = mlr + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfr[k - s], FBw[k], omar[k - s], FBw[k - 1], &FBw[k] );
                }
                N_CopyCPt( FBw[p], &FNw[save] );
            }
        }

        if( rs GT 0 )
        {
            numer = S[bs] - S[as];

            for ( k = q; k > mls; k-- )
            {
                alfs[k - mls - 1] = numer / (S[as + k] - S[as]);
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

        /* Compute dot product of Bezier curves */

        error = N_BezCrvDotProduct( FBw, p, GBw, q, &pm, lbz, pq, dot, dow );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot R[ar] = S[as] */

        if( rem GT 1 )
        {
            first = kind - 2;
            last = kind;
            denom = S[bs] - S[as];
            bet = (S[bs] - N[kind - 1]) / denom;
            omb = 1.0 - bet;

            for ( k = 1; k < rem; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT nind )
                    {
                        alf = (S[bs] - N[i]) / (S[as] - N[i]);
                        oma = 1.0 - alf;
                        fn[i] = alf * fn[i] + oma * fn[i - 1];

                        if( rat EQ NL_YES )
                            fd[i] = alf * fd[i] + oma * fd[i - 1];
                    }

                    if( j GE lbz )
                    {
                        if( j - k LE kind - pq + rem )
                        {
                            gam = (S[bs] - N[j - k]) / denom;
                            omg = 1.0 - gam;
                            dot[l] = gam * dot[l] + omg * dot[l + 1];

                            if( rat EQ NL_YES )
                                dow[l] = gam * dow[l] + omg * dow[l + 1];
                        }
                        else
                        {
                            dot[l] = bet * dot[l] + omb * dot[l + 1];

                            if( rat EQ NL_YES )
                                dow[l] = bet * dow[l] + omb * dow[l + 1];
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
                N[kind] = S[as];
                kind++;

                if( rat EQ NL_YES )
                {
                    D[lind] = S[as];
                    lind++;
                }
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            fn[nind] = dot[i];
            nind++;

            if( rat EQ NL_YES )
            {
                fd[dind] = dow[i];
                dind++;
            }
        }

        /* Initialize for next pass through */

        if( br LT mr AND bs LT ms )
        {
            for ( i = 0; i < rr; i++ )
                N_CopyCPt( FNw[i], &FBw[i] );

            for ( i = rr; i <= p; i++ )
                N_CopyCPt( Fw[br - p + i], &FBw[i] );

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
                N[kind] = S[bs];
                kind++;

                if( rat EQ NL_YES )
                {
                    D[lind] = S[bs];
                    lind++;
                }
            }
        }
    }

    /* Compact output functions */

    if( nind LT nh + 1 )
    {
        N_CFuncSetSizeIndices( num, nind - 1, pq, kind - 1 );

        error = N_CrvFuncCompact( num, SG );

        if( error EQ NL_YES )
            NL_OUT;

        if( rat EQ NL_YES )
        {
            N_CFuncSetSizeIndices( den, dind - 1, pq, lind - 1 );

            error = N_CrvFuncCompact( den, SG );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvDotCrv */

#endif //NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic operators  routine computes the sum/difference  of 
     two NURBS curves. The result is obtained by  using other symbolic 
     operators to compute the following expression:

          N_p       N_q   N_p*D_q +/- N_q*D_p   N_r
          ---  +/-  --- = ------------------- = ---
          D_p       D_q         D_p*D_q         D_r

     If the  curves are  non-rational, the  computation  simplifies to 
     N_p +/- N_q. A typical calling example is:

       NL_CURVE   curP, curQ, curR;
       NL_STACKS  SG;
       ...
       (define curP and curQ);
       ...
       N_CrvInitArrays(&curR);
       N_CrvSumDiffCrv(&curP,&curQ,NL_PLUS,&curR,&SG);

     If memory is available, curR is not  initialized and  the routine 
     assumes that memory  allocation has been done. However, it checks 
     for the proper amount by looking at the highest indexes in curR's
     knot vector and control polygon objects.


   ACCESS:
   
     curP , input  ,  Curve
     curQ , input  ,  Curve
     opr  , input  ,  Operator flag:
                        NL_PLUS : sum
                        NL_MINUS: difference
     curR , output ,  Sum/difference of curP and curQ
     SG   , input  ,  curR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvSumDiffCrv( NL_CURVE *curP, NL_CURVE *curQ, NL_FLAG opr, NL_CURVE *curR, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvSumDiffCrv");

    NL_FLAG error = NL_NO, ratP = NL_NO, ratQ = NL_NO, rat = NL_NO;

    NL_INDEX i, n, m, nsp, nsq, mxp, mxq;

    NL_DEGREE p, q;

    NL_REAL *UP, *UQ, *UR, *A;

    NL_CPOINT *Pw, *Qw, *Rw;

    NL_CURVE numP, numQ, num, tmp, curPW, curQW;

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

    /* Compute sum/difference */

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

            N_CrvInitArrays( &num );
            error = N_CrvFuncMultiplyCrv( &denQ, &numP, &num, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvInitArrays( &tmp );
            error = N_CrvFuncMultiplyCrv( &denP, &numQ, &tmp, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            /* Add terms in the numerator */

            N_CrvGetCPts( &num, &n, &Pw );
            N_CrvGetCPts( &tmp, &n, &Qw );

            switch( opr )
            {
                case NL_PLUS:
                    for ( i = 0; i <= n; i++ )
                        N_Sum2CPts( Pw[i], Qw[i], &Pw[i] );
                    break;

                case NL_MINUS:
                    for ( i = 0; i <= n; i++ )
                        N_Diff2CPts( Pw[i], Qw[i], &Pw[i] );
                    break;

                default:

                    NL_ERROR( NL_CAL_ERR );
            }

            /* Create output curve and reset rationality of input curves */

            error = N_CreateCrvFromNumAndDenom( &num, &den, curR, SG );

            if( error EQ NL_YES )
                NL_OUT;

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

            N_KnotVectorGetKnots( kxp, &mxp, &A );
            N_KnotVectorGetKnots( kxq, &mxq, &A );

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

            /* Check memory of output curve */

            error = N_CrvSizeArrays( curR, n, p, m, rname, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_CrvGetCPtsAndKnots( curR, &Rw, &UR );

            /* Compute output curve */

            switch( opr )
            {
                case NL_PLUS:
                    for ( i = 0; i <= n; i++ )
                        N_Sum2CPts( Pw[i], Qw[i], &Rw[i] );
                    break;

                case NL_MINUS:
                    for ( i = 0; i <= n; i++ )
                        N_Diff2CPts( Pw[i], Qw[i], &Rw[i] );
                    break;

                default:

                    NL_ERROR( NL_CAL_ERR );
            }

            for ( i = 0; i <= m; i++ )
                UR[i] = UP[i];
            break;

        default:

            NL_ERROR( NL_INP_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvSumDiffCrv */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators routine computes the cross product of two
     NURBS curves. The cross  product is a  rational curve  if at least 
     one of  the curves is  rational. Otherwise, the  product curve  is 
     non-rational.  The knot vectors of both curves are rescaled to the 
     unit span [0,1]. This  changes the position of the knots, however, 
     it   leaves   the   curves   unaltered  both   parametrically  and  
     geometrically. A typical calling example is:

       NL_CURVE   curF, curG, curH;
       NL_STACKS  SG;
       ...
       (define curF and curG);
       ...
       N_CrvInitArrays(&curH);
       N_CrvCrossMultiplyCrv(&curF,&curG,&curH,&SG);

     If memory is  available, curH  is not initialized and  the routine 
     assumes  that memory  allocation has been done. However, it checks 
     for the proper  amount by looking at the highest indexes in curH's 
     knot vector and control value  objects. THE STORAGE  OF THE OUTPUT 
     NL_CURVE IS COMPACTED, I.E. THE MEMORY PASSED IN IS DESTROYED.


   ACCESS:
   
     curF , in/out ,  Curve (ITS KNOT NL_VECTOR IS RESCALED)
     curG , in/out ,  Curve (ITS KNOT NL_VECTOR IS RESCALED)
     curH , output ,  Cross product of curF and curG
     SG   , input  ,  curH's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvCrossMultiplyCrv( NL_CURVE *curF, NL_CURVE *curG, NL_CURVE *curH, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvCrossMultiplyCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, nf, mr, ng, ms, nh, mt, mlr, mls, mir, mis, mi, rr, rs, s, ar, as, br, bs, mxr, mxs, ixr, ixs, t, kind, cind, first, last, rem, save, nsr, nss, lbz, rbz;

    NL_DEGREE p, q, pq;

    NL_REAL *R, *S, *T, *XR, *XS, *alfr, *omar, *alfs, *omas, alf, oma, bet, omb, gam, omg, numer, denom;

    NL_CPOINT *Fw, *Gw, *Hw, *FBw, *GBw, *FNw, *GNw, *FGw;

    NL_KNOTVECTOR *knr, *kns, *kxr, *kxs;

    NL_CURVE curFR, curGR;

    NL_RMATRIX pm;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get new degree and set flag */

    N_CrvGetDegree( curF, &p );
    N_CrvGetDegree( curG, &q );

    pq = p + q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Handle special case p=0 or q=0 */

    if( p EQ 0 OR q EQ 0 )
    {
        N_CrvGetArraySizes( curF, &i, &j );
        N_CrvGetArraySizes( curG, &k, &l );

        if( i NEQ p OR k NEQ q )
            NL_ERROR( NL_INP_ERR );

        error = N_CrvSizeArrays( curH, pq, pq, pq + pq + 1, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curF, &Fw, &R );
        N_CrvGetCPtsAndKnots( curG, &Gw, &S );
        N_CrvGetCPtsAndKnots( curH, &Hw, &T );

        if( p EQ 0 )
        {
            for ( j = 0; j <= q; j++ )
            {
                N_Cross2CPts( Fw[0], Gw[j], &Hw[j] );
                T[j] = S[j];
                T[q + j + 1] = S[q + j + 1];
            }
        }
        else
        {
            for ( i = 0; i <= p; i++ )
            {
                N_Cross2CPts( Fw[i], Gw[0], &Hw[i] );
                T[i] = R[i];
                T[p + i + 1] = R[p + i + 1];
            }
        }

        NL_OUT;
    }

    /* Merge knots */

    N_CrvGetKnotVector( curF, &knr );
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

    /* Refine curves */

    N_KnotVectorGetKnots( kxr, &mxr, &XR );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( mxr GE 0 )
    {
        N_CrvInitArrays( &curFR );
        error = N_CrvRefine( curF, kxr, &curFR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsDegreeAndKnots( &curFR, &nf, &Fw, &p, &mr, &R );
        N_CrvGetKnotVector( &curFR, &knr );
        N_BasisGetSpanCount( knr, p, &nsr );
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( curF, &nf, &Fw, &p, &mr, &R );
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

    error = N_CrvSizeArrays( curH, nh, pq, mt, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( curH, &Hw, &T );

    /* Get local memory */

    FBw = N_AllocCPt1dArray( p, &SL );

    if( FBw EQ NULL )
        NL_QUIT;

    FNw = N_AllocCPt1dArray( p, &SL );

    if( FNw EQ NULL )
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
        N_CopyCPt( Fw[i], &FBw[i] );

    for ( j = 0; j <= q; j++ )
        N_CopyCPt( Gw[j], &GBw[j] );

    N_Cross2CPts( Fw[0], Gw[0], &Hw[0] );

    N_InitRealMatrix( &pm );
    error = N_BezFuncMultiplyBezCrvMatrix( p, q, &pm, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /***************************************************************/
    /* Loop through the knot vectors and do the following:         */
    /*   (1) Extract the i-th Bezier curves.                       */
    /*   (2) Compute their cross product.                          */
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

        /* Insert knot to get Bezier segments */

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
            numer = R[br] - R[ar];

            for ( k = p; k > mlr; k-- )
            {
                alfr[k - mlr - 1] = numer / (R[ar + k] - R[ar]);
                omar[k - mlr - 1] = 1.0 - alfr[k - mlr - 1];
            }

            for ( j = 1; j <= rr; j++ )
            {
                save = rr - j;
                s = mlr + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfr[k - s], FBw[k], omar[k - s], FBw[k - 1], &FBw[k] );
                }
                N_CopyCPt( FBw[p], &FNw[save] );
            }
        }

        if( rs GT 0 )
        {
            numer = S[bs] - S[as];

            for ( k = q; k > mls; k-- )
            {
                alfs[k - mls - 1] = numer / (S[as + k] - S[as]);
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

        /* Compute cross product of Bezier curves */
        error = N_BezCrossProduct( FBw, p, GBw, q, &pm, lbz, pq, FGw );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot R[ar] = S[as] */

        if( rem GT 1 )
        {
            first = kind - 2;
            last = kind;
            denom = S[bs] - S[as];
            bet = (S[bs] - T[kind - 1]) / denom;
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
                            gam = (S[bs] - T[j - k]) / denom;
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
            N_CopyCPt( FGw[i], &Hw[cind] );
            cind++;
        }

        /* Initialize for next pass through */

        if( br LT mr AND bs LT ms )
        {
            for ( i = 0; i < rr; i++ )
                N_CopyCPt( FNw[i], &FBw[i] );

            for ( i = rr; i <= p; i++ )
                N_CopyCPt( Fw[br - p + i], &FBw[i] );

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

        error = N_CrvCompress( curH, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvCrossMultiplyCrv */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the sum/difference of 
     a NURBS curve and a vector. A typical calling example is:

       NL_CURVE   cur;
       NL_VECTOR  K;
       ...
       (define cur and get K);
       ...
       N_CrvSumDiffVector(&cur,K,NL_PLUS);

     THE COMPUTATION IS DONE  IN-PLACE, I.E. THE  ORIGINAL NL_CURVE IS 
     DESTROYED.


   ACCESS:
   
     cur , in/out ,  NURBS curve 
     K   , input  ,  Vector
     opr , input  ,  Operator flag:
                       NL_PLUS : sum
                       NL_MINUS: difference


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvSumDiffVector( NL_CURVE *cur, NL_VECTOR K, NL_FLAG opr )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvSumDiffVector");

    NL_FLAG error = NL_NO;

    NL_INDEX i, n;

    NL_REAL w;

    NL_CPOINT *Pw;

    NL_VECTOR L;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute sum/difference */

    N_CrvGetCPts( cur, &n, &Pw );

    if( N_IsCrvRat( cur ) )
    {
        switch( opr )
        {
            case NL_PLUS:
                for ( i = 0; i <= n; i++ )
                {
                    N_CPtGetW( Pw[i], &w );
                    N_VectorScale( K, w, &L );
                    N_SumCPtAndPt( Pw[i], L, &Pw[i] );
                }
                break;

            case NL_MINUS:
                for ( i = 0; i <= n; i++ )
                {
                    N_CPtGetW( Pw[i], &w );
                    N_VectorScale( K, w, &L );
                    N_DiffCPtPt( Pw[i], L, &Pw[i] );
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
                    N_SumCPtAndPt( Pw[i], K, &Pw[i] );
                break;

            case NL_MINUS:
                for ( i = 0; i <= n; i++ )
                    N_DiffCPtPt( Pw[i], K, &Pw[i] );
                break;

            default:

                NL_ERROR( NL_CAL_ERR );
        }
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvSumDiffVector */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the derivative curve of a
     NON-RATIONAL curve. The maximum derivative allowed is equal to the
     degree. A typical calling example is:

       NL_CURVE   cur, der;
       NL_INDEX   d;
       NL_STACKS  SG;
       ...
       (define cur; get highest derivative index d);
       ...
       N_CrvInitArrays(&der);
       N_CrvNonRatGetFirstDeriv(&cur,d,&der,&SG);


   ACCESS:
   
     cur  , input  ,  NON-RATIONAL curve
     d    , input  ,  Highest derivative required (MUST BE LESS THAN OR
                      EQUAL TO THE NL_CURVE NL_DEGREE)
     der  , output ,  Derivative curve
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvNonRatGetFirstDeriv( NL_CURVE *cur, NL_INDEX d, NL_CURVE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvNonRatGetFirstDeriv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, nd, md;

    NL_DEGREE p, pd;

    NL_REAL *U, *V, alf;

    NL_CPOINT *Pw, *Dw, *Tw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( d LT 0 OR d GT p )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( cur ) )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    nd = n - d;
    pd = (NL_DEGREE)( p - d );
    md = nd + pd + 1;

    error = N_CrvSizeArrays( der, nd, pd, md, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( der, &Dw, &V );

    /* Initialize control points */

    Tw = N_AllocCPt1dArray( n, &SL );

    if( Tw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Pw[i], &Tw[i] );

    /* Compute control points of derivative curve */

    for ( k = 1; k <= d; k++ )
    {
        for ( i = 0; i <= n - k; i++ )
        {
            if( U[i + p + 1]EQ U[i + k] )
                NL_ERROR( NL_DER_ERR );

            if( N_FloatOpIsBad( ( (NL_REAL)p - (NL_REAL)k + (NL_REAL)1 ), U[i + p + 1] - U[i + k], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            alf = ((NL_REAL)p - (NL_REAL)k + (NL_REAL)1) / (U[i + p + 1] - U[i + k]);
            N_Combine2CPts( alf, Tw[i + 1], -alf, Tw[i], &Tw[i] );
        }
    }

    for ( i = 0; i <= n - d; i++ )
        N_CopyCPt( Tw[i], &Dw[i] );

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
} /* end N_CrvNonRatGetFirstDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the  first derivative 
     curve of a rational curve. The result is  obtained by  using other 
     symbolic operators to compute the following expression:

         ( N )'   N'*D - N*D'
         ( - )  = -----------
         ( D )       D*D     

     A typical calling example is:

       NL_CURVE   cur, der;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&der);
       N_CrvRatGetFirstDeriv(&cur,&der,&SG);


   ACCESS:
   
     cur , input  ,  NURBS curve 
     der , output ,  Derivative of cur
     SG  , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvRatGetFirstDeriv( NL_CURVE *cur, NL_CURVE *der, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX nsp, mx;

    NL_DEGREE p;

    NL_REAL *X;

    NL_CURVE n, np, npd, ndp, num;

    NL_CFUN d, dp, den;

    NL_KNOTVECTOR *knd, *knx;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsCrvRat( cur ) )
    {
        error = N_CrvNonRatGetFirstDeriv( cur, 1, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_CrvInitArrays( &n );
    N_CFuncInitArrays( &d );
    error = N_CrvGetNumAndDenom( cur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output curve */

    N_CFuncInitArrays( &den );
    error = N_CrvFuncMultiplyCrvFunc( &d, &d, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncGetKnotVector( &den, &knd );
    N_CFuncGetDegree( &den, &p );
    N_BasisGetSpanCount( knd, p, &nsp );

    knx = N_AllocKnotVectorAndArray( nsp, &SL );

    if( knx EQ NULL )
        NL_QUIT;

    error = N_BasisIncreaseKnotMult( knd, p, 1, knx );

    if( error EQ NL_YES )
        NL_OUT;

    N_KnotVectorGetKnots( knx, &mx, &X );

    if( mx GE 0 )
    {
        error = N_CrvFuncRefine( &den, knx, &den, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute terms in the numerator */

    N_CrvInitArrays( &np );
    error = N_CrvNonRatGetFirstDeriv( &n, 1, &np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &dp );
    error = N_CrvFuncDeriv( &d, 1, &dp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &npd );
    error = N_CrvFuncMultiplyCrv( &d, &np, &npd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ndp );
    error = N_CrvFuncMultiplyCrv( &dp, &n, &ndp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_CrvInitArrays( &num );
    error = N_CrvSumDiffCrv( &npd, &ndp, NL_MINUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Degree elevate numerator */

    error = N_CrvElevateDegree( &num, 1, &num, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output curve */

    error = N_CreateCrvFromNumAndDenom( &num, &den, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvRatGetFirstDeriv */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes all derivative curves, up
     to a  specified derivative, of a  NON-RATIONAL curve. The  maximum 
     derivative  allowed is  equal to  the  degree. A  typical  calling 
     example is:

       NL_CURVE   cur, **der;
       NL_INDEX   d;
       NL_STACKS  SG;
       ...
       (define cur; get highest derivative index d);
       ...
       N_CrvNonRatGetDerivCrvsAll(&cur,d,&der,&SG);

     der[0], der[1],..., der[d] are pointers to the 0th, first,..., dth
     derivative curves. MEMORY TO STORE THESE CURVES IS ALLOCTED INSIDE
     THE ROUTINE.


   ACCESS:
   
     cur  , input  ,  NON-RATIONAL curve
     d    , input  ,  Highest derivative required (MUST BE LESS THAN OR
                      EQUAL TO THE NL_CURVE NL_DEGREE)
     der  , output ,  Derivative curves
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvNonRatGetDerivCrvsAll( NL_CURVE *cur, NL_INDEX d, NL_CURVE *** der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvNonRatGetDerivCrvsAll");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m;

    NL_DEGREE p;

    NL_REAL *U, *V, alf;

    NL_CPOINT *Pw, *Dw, *Tw;

    NL_CURVE ** curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );

    if( d LT 0 OR d GT p )
        NL_ERROR( NL_INP_ERR );

    if( N_IsCrvRat( cur ) )
        NL_ERROR( NL_INP_ERR );

    /* Allocate memory to hold derivative curves */

    curA = N_AllocArrayCrvPtrs( d, SG );

    if( curA EQ NULL )
        NL_QUIT;

    for ( k = 0; k <= d; k++ )
    {
        curA[k] = N_AllocCrvAndArrays( n - k, (NL_DEGREE)( p - k ), n - k + p - k + 1, SG );

        if( curA[k]EQ NULL )
            NL_QUIT;
    }

    /* Initialize control points */

    Tw = N_AllocCPt1dArray( n, &SL );

    if( Tw EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
        N_CopyCPt( Pw[i], &Tw[i] );

    /* Compute derivative curves */

    error = N_CrvCopy( cur, curA[0], SG );

    if( error EQ NL_YES )
        NL_OUT;

    for ( k = 1; k <= d; k++ )
    {
        N_CrvGetCPtsAndKnots( curA[k], &Dw, &V );

        for ( i = 0; i <= n - k; i++ )
        {
            if( U[i + p + 1]EQ U[i + k] )
                NL_ERROR( NL_DER_ERR );

            if( N_FloatOpIsBad( ( (NL_REAL)p - (NL_REAL)k + (NL_REAL)1 ), U[i + p + 1] - U[i + k], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );

            alf = ((NL_REAL)p - (NL_REAL)k + (NL_REAL)1) / (U[i + p + 1] - U[i + k]);
            N_Combine2CPts( alf, Tw[i + 1], -alf, Tw[i], &Tw[i] );
            N_CopyCPt( Tw[i], &Dw[i] );
        }

        j = -1;

        for ( i = 0; i <= p - k; i++ )
            V[++j] = U[0];

        for ( i = p + 1; i <= n; i++ )
            V[++j] = U[i];

        for ( i = 0; i <= p - k; i++ )
            V[++j] = U[m];
    }

    *der = curA;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvNonRatGetDerivCrvsAll */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes all derivative curves of a 
     NURBS  curve up to a  specified  derivative. If  the  curve  is non 
     rational, control point differencing is applied. If it is rational, 
     then the kth  derivative curve is  obtained by differentiating  the 
     (k-1)-th derivative curve. A typical calling example is:

       NL_CURVE   cur, **der;
       NL_INDEX   d;
       NL_STACKS  SG;
       ...
       (define cur; get d);
       ...
       N_CrvGetDerivCrvsAll(&cur,d,&der,&SG);

     der[0], der[1],..., der[d] are  pointers to the 0th, first,..., dth
     derivative curves. MEMORY TO STORE  THESE CURVES IS ALLOCTED INSIDE
     THE ROUTINE. NL_MAXIMUM  NL_DERIVATIVE IS  NL_DEGREE  NL_MINUS  NL_MAXIMUM INTERNAL
     KNOT MULTIPLICITY.


   ACCESS:
   
     cur , input  ,  NURBS curve 
     d   , input  ,  Highest derivative (MUST  BE  LESS  THAN THE  NL_CURVE 
                     NL_DEGREE FOR NON-RATIONAL AND NL_DEGREE-1 FOR RATIONAL)
     der , output ,  Derivative  curves 
     SG  , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvGetDerivCrvsAll( NL_CURVE *cur, NL_INDEX d, NL_CURVE *** der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvGetDerivCrvsAll");

    NL_FLAG error = NL_NO;

    NL_INDEX k;

    NL_DEGREE p;

    NL_CURVE ** curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsCrvRat( cur ) )
    {
        error = N_CrvNonRatGetDerivCrvsAll( cur, d, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Compute derivatives */

    N_CrvGetDegree( cur, &p );

    if( d LT 0 OR d GT p - 1 )
        NL_ERROR( NL_INP_ERR );

    curA = N_AllocArrayCrvPtrs( d, SG );

    if( curA EQ NULL )
        NL_QUIT;

    for ( k = 0; k <= d; k++ )
    {
        curA[k] = N_AllocCrv( SG );

        if( curA[k]EQ NULL )
            NL_QUIT;
    }

    N_CrvInitArrays( curA[0] );
    error = N_CrvCopy( cur, curA[0], SG );

    if( error EQ NL_YES )
        NL_OUT;

    for ( k = 1; k <= d; k++ )
    {
        N_CrvInitArrays( curA[k] );
        error = N_CrvRatGetFirstDeriv( curA[k - 1], curA[k], SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    *der = curA;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvGetDerivCrvsAll */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the k-th derivative curve 
     of a  NURBS  curve. If the  curve is  non rational,  control point 
     differencing  is  applied.  If  it  is  rational,  then  the  i-th  
     derivative  curve  is  obtained  by  differentiating the  (i-1)-th 
     derivative curve. A typical calling example is:

       NL_CURVE   cur, der;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cur; get k);
       ...
       N_CrvInitArrays(&der);
       N_CrvMakeDerivCrv(&cur,k,&der,&SG);

     IT IS SUGGESTED TO  ALLOW THE ROUTINE TO  ALLOCATE  MEMORY FOR der
     INTERNALLY. NL_MAXIMUM  NL_DERIVATIVE IS  NL_DEGREE NL_MINUS  NL_MAXIMUM INTERNAL
     KNOT MULTIPLICITY.
     

   ACCESS:
   
     cur , input  ,  NURBS curve 
     k   , input  ,  Highest derivative (MUST BE LESS  THAN  THE  NL_CURVE 
                     NL_DEGREE FOR NON-RATIONAL AND NL_DEGREE-1 FOR RATIONAL)
     der , output ,  Derivative  curve
     SG  , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvMakeDerivCrv( NL_CURVE *cur, NL_INDEX k, NL_CURVE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvMakeDerivCrv");

    NL_FLAG error = NL_NO, derA;    /* derB not used */

    NL_INDEX i;

    NL_DEGREE p;

    NL_CURVE curA, curB;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsCrvRat( cur ) )
    {
        error = N_CrvNonRatGetFirstDeriv( cur, k, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Compute derivatives */

    N_CrvGetDegree( cur, &p );

    if( k LT 0 OR k GT p - 1 )
        NL_ERROR( NL_INP_ERR );

    N_CrvInitArrays( &curA );
    error = N_CrvCopy( cur, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    derA = NL_YES;
    /* derB = NL_NO; */

    for ( i = 1; i <= k; i++ )
    {
        if( derA EQ NL_YES )
        {
            N_CrvInitArrays( &curB );
            error = N_CrvRatGetFirstDeriv( &curA, &curB, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_FreeCrv( &curA, &SL );

            derA = NL_NO;
            /* derB = NL_YES; */
        }
        else
        {
            N_CrvInitArrays( &curA );
            error = N_CrvRatGetFirstDeriv( &curB, &curA, SG );

            if( error EQ NL_YES )
                NL_OUT;

            N_FreeCrv( &curB, &SL );

            derA = NL_YES;
            /* derB = NL_NO; */
        }
    }

    if( derA EQ NL_YES )
    {
        error = N_CrvCopy( &curA, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_CrvCopy( &curB, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvMakeDerivCrv */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This symbolic operators routine computes an upper bound on the 
     difference of two curves. That is, it computes
 
        | C_1(u) - C_2(u) | < bound
 
     A typical calling example is:
 
       NL_CURVE  curP, curQ;
       NL_REAL   bnd; 
       ...
       (define curP and curQ);
       ...
       N_CrvDiffCrvGetMaxChange(&curP,&curQ,&bnd);
 
 
   ACCESS:
   
     curP , input  ,  NURBS curve
     curQ , input  ,  NURBS curve
     bnd  , output ,  Bound of |curP-curQ|
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_CrvDiffCrvGetMaxChange( NL_CURVE *curP, NL_CURVE *curQ, NL_REAL *bnd )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, k;

    NL_REAL len;

    NL_POINT P;

    NL_CURVE ** bez, curR;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get difference of two curves */

    N_CrvInitArrays( &curR );
    error = N_CrvSumDiffCrv( curP, curQ, NL_MINUS, &curR, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get Bezier pieces */

    error = N_CrvDecomposeBez( &curR, &bez, &k, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute bound */

    *bnd = 0.0;

    for ( i = 0; i <= k; i++ )
    {
        N_CrvGetMaxPosVector( bez[i], &P, &len );

        if( len GT *bnd )
            *bnd = len;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvDiffCrvGetMaxChange */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators routine  computes an  upper  bound on the 
     change  of a curve  obtained by  moving a knot. A  typical calling 
     example is:

       NL_CURVE  cur;
       NL_INDEX  k;
       NL_REAL   du, bnd; 
       ...
       (define cur, get k and du);
       ...
       N_CrvMoveKnotGetMaxChange(&cur,k,du,&bnd);
 

   ACCESS:
   
     cur , input  ,  NURBS curve
     k   , input  ,  Index of knot to be moved
     du  , input  ,  Distance u_k to be moved:
                       du > 0.0: the right  knot of a  multiple knot is 
                                  moved to the right by du
                       du < 0.0: the  left  knot of a  multiple knot is 
                                  moved to the left by du
     bnd , output ,  Bound on curve change


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvMoveKnotGetMaxChange( NL_CURVE *cur, NL_INDEX k, NL_REAL du, NL_REAL *bnd )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvMoveKnotGetMaxChange");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    NL_DEGREE p;

    NL_REAL *U;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check error */

    N_CrvGetArraySizes( cur, &n, &m );
    N_CrvGetKnots( cur, &m, &U );
    N_CrvGetDegree( cur, &p );

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

    /* Copy curve and move knot */

    N_CrvInitArrays( &curA );
    error = N_CrvCopy( cur, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetKnots( &curA, &m, &U );

    U[k] = U[k] + du;

    /* Compute bound */

    error = N_CrvDiffCrvGetMaxChange( cur, &curA, bnd );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvMoveKnotGetMaxChange */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the derivative of a NON-
     RATIONAL curve with respect to a knot. A typical calling example:

       NL_CURVE   cur, der;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&der);
       N_CrvNonRatEvalDeriv(&cur,k,NL_LEFT,&der,&SG);

     Since NLIB  does not allow knot  multiplicities  greater than the
     degree, the curve cannot be differentiated with respect to a knot
     with  multiplicity  equal  to the  degree. The  error  NL_KML_ERR is 
     returned if such a knot is found.


   ACCESS:
   
     cur  , input  ,  NON-RATIONAL curve
     k    , input  ,  Index of  knot, i.e. the derivative with respect 
                      to u_k is computed
     flg  , input  ,  Flag:
                        NL_LEFT : left derivative. NL_INDEX  k  MUST SATISFY
                               u_(k) != u_(k-1)
                        NL_RIGHT: right derivative. NL_INDEX k  MUST SATISFY
                               u_(k) != u_(k+1) 
     der  , output ,  Derivative curve
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvNonRatEvalDeriv( NL_CURVE *cur, NL_INDEX k, NL_FLAG flg, NL_CURVE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvNonRatEvalDeriv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, mlt;

    NL_DEGREE p;

    NL_REAL *UC, *UD, fac;

    NL_CPOINT *Pw, *Dw, Z;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &UC );

    if( N_IsCrvRat( cur ) )
        NL_ERROR( NL_INP_ERR );

    if( k LE p OR k GE n + 1 )
        NL_ERROR( NL_INP_ERR );

    switch( flg )
    {
        case NL_LEFT:
            if( UC[k]EQ UC[k - 1] )
                NL_ERROR( NL_INP_ERR );

            i = k;

            while( UC[i]EQ UC[i + 1] )
                i++;
            mlt = i - k + 1;

            if( mlt GE p )
                NL_ERROR( NL_KML_ERR );
            break;

        case NL_RIGHT:
            if( UC[k]EQ UC[k + 1] )
                NL_ERROR( NL_INP_ERR );

            i = k;

            while( UC[i]EQ UC[i - 1] )
                i--;
            mlt = k - i + 1;

            if( mlt GE p )
                NL_ERROR( NL_KML_ERR );
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_CrvSizeArrays( der, n + 1, p, m + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( der, &Dw, &UD );

    /* Compute the knot vector */

    switch( flg )
    {
        case NL_LEFT:

            j = -1;

            for ( i = 0; i <= k + mlt - 1; i++ )
                UD[++j] = UC[i];
            UD[++j] = UC[k];

            for ( i = k + mlt; i <= m; i++ )
                UD[++j] = UC[i];
            break;

        case NL_RIGHT:

            j = -1;

            for ( i = 0; i <= k; i++ )
                UD[++j] = UC[i];
            UD[++j] = UC[k];

            for ( i = k + 1; i <= m; i++ )
                UD[++j] = UC[i];
            break;
    }

    /* Compute control points */

    N_CPtFromWxWyWz( 0.0, 0.0, 0.0, NL_NOW, &Z );

    if( NOT N_CrvIs3d( cur ) )
        N_CPtSetZ( NL_NOZ, &Z );

    for ( i = 0; i <= k - p - 1; i++ )
        N_CopyCPt( Z, &Dw[i] );

    for ( i = k - p; i <= k; i++ )
    {
        fac = 1.0 / (UC[i + p] - UC[i]);
        N_Diff2CPts( Pw[i - 1], Pw[i], &Dw[i] );
        N_ScaleCPt( fac, Dw[i], &Dw[i] );
    }

    for ( i = k + 1; i <= n + 1; i++ )
        N_CopyCPt( Z, &Dw[i] );

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvNonRatEvalDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes  higher derivatives of a 
     NON-RATIONAL  curve  with  respect to  a knot. A  typical calling 
     example is:

       NL_CURVE   cur, der;
       NL_INDEX   k, r;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&der);
       N_CrvEvalHighDerivsKnot(&cur,k,NL_LEFT,r,&der,&SG);

     Since NLIB  does not allow knot  multiplicities  greater than the
     degree, the curve cannot be differentiated with respect to a knot
     with  multiplicity  equal  to the  degree. The  error  NL_KML_ERR is 
     returned if such a knot is found.


   ACCESS:
   
     cur  , input  ,  NON-RATIONAL curve
     k    , input  ,  Index of  knot, i.e. the derivative with respect 
                      to u_k is computed
     flg  , input  ,  Flag:
                        NL_LEFT : left derivative. NL_INDEX  k  MUST SATISFY
                               u_(k) != u_(k-1)
                        NL_RIGHT: right derivative. NL_INDEX k  MUST SATISFY
                               u_(k) != u_(k+1) 
     r    , input  ,  Highest derivative required (1 < r < degree-1)
     der  , output ,  Derivative curve
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalHighDerivsKnot( NL_CURVE *cur, NL_INDEX k, NL_FLAG flg, NL_INDEX r, NL_CURVE *der, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvEvalHighDerivsKnot");

    NL_FLAG tgl, error = NL_NO;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_CURVE curA, curB;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals and check error */

    N_CrvGetArraySizes( cur, &n, &m );
    N_CrvGetDegree( cur, &p );

    if( r LT 1 OR r GT p - 1 )
        NL_ERROR( NL_INP_ERR );

    if( k LE p OR k GE n + 1 )
        NL_ERROR( NL_INP_ERR );

    /* Allocate memory */

    error = N_CrvSizeArrays( der, n + r, p, m + r, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    if( r GT 1 )
    {
        error = N_AllocCrvArrays( &curA, n + r, p, m + r, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( r GT 2 )
    {
        error = N_AllocCrvArrays( &curB, n + r, p, m + r, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute derivatives */

    if( r EQ 1 )
    {
        error = N_CrvNonRatEvalDeriv( cur, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    error = N_CrvNonRatEvalDeriv( cur, k, flg, &curA, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    tgl = NL_YES;

    for ( i = 2; i <= r - 1; i++ )
    {
        if( flg EQ NL_RIGHT )
            k++;

        if( tgl EQ NL_YES )
        {
            N_CrvSetSizeIndices( &curB, n + r, p, m + r );

            error = N_CrvNonRatEvalDeriv( &curA, k, flg, &curB, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            tgl = NL_NO;
        }
        else
        {
            N_CrvSetSizeIndices( &curA, n + r, p, m + r );

            error = N_CrvNonRatEvalDeriv( &curB, k, flg, &curA, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            tgl = NL_YES;
        }
    }

    if( flg EQ NL_RIGHT )
        k++;

    if( tgl EQ NL_YES )
    {
        error = N_CrvNonRatEvalDeriv( &curA, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_CrvNonRatEvalDeriv( &curB, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvEvalHighDerivsKnot */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine computes the  first derivative 
     curve of a rational  curve with respect  to a knot. The result is  
     obtained  by  using  other  symbolic  operators  to  compute  the 
     following expression:

         ( N )'   N'*D - N*D'
         ( - )  = -----------
         ( D )       D*D     

     where  N' and  D' are the  derivatives of  the  numerator and the 
     denominator  with respect to a kot. A typical calling example is:

       NL_CURVE   cur, der;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&der);
       N_CrvEvalFirstDerivKnot(&cur,k,NL_LEFT,&der,&SG);

     Since NLIB  does not allow knot  multiplicities  greater than the
     degree, the curve cannot be differentiated with respect to a knot
     with  multiplicity  equal  to the  degree. The  error  NL_KML_ERR is 
     returned if such a knot is found.


   ACCESS:
   
     cur  , input  ,  NURBS curve
     k    , input  ,  Index of  knot, i.e. the derivative with respect 
                      to u_k is computed
     flg  , input  ,  Flag:
                        NL_LEFT : left derivative. NL_INDEX  k  MUST SATISFY
                               u_(k) != u_(k-1)
                        NL_RIGHT: right derivative. NL_INDEX k  MUST SATISFY
                               u_(k) != u_(k+1) 
     der  , output ,  Derivative curve
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalFirstDerivKnot( NL_CURVE *cur, NL_INDEX k, NL_FLAG flg, NL_CURVE *der, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX m;

    NL_REAL *U;

    NL_CURVE n, np, npd, ndp, num;

    NL_CFUN d, dp, den;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsCrvRat( cur ) )
    {
        error = N_CrvNonRatEvalDeriv( cur, k, flg, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_CrvInitArrays( &n );
    N_CFuncInitArrays( &d );
    error = N_CrvGetNumAndDenom( cur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output curve */

    N_CrvGetKnots( cur, &m, &U );
    N_CFuncInitArrays( &den );

    error = N_CrvFuncMultiplyCrvFunc( &d, &d, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvFuncInsertKnot( &den, U[k], 1, &den, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute terms in the numerator */

    N_CrvInitArrays( &np );
    error = N_CrvNonRatEvalDeriv( &n, k, flg, &np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &dp );
    error = N_CrvFuncDerivKnot( &d, k, flg, &dp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &npd );
    error = N_CrvFuncMultiplyCrv( &d, &np, &npd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ndp );
    error = N_CrvFuncMultiplyCrv( &dp, &n, &ndp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_CrvInitArrays( &num );
    error = N_CrvSumDiffCrv( &npd, &ndp, NL_MINUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output curve */

    error = N_CreateCrvFromNumAndDenom( &num, &den, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvEvalFirstDerivKnot */

/*******************************************************************//**


   DESCRIPTION:

     This  symbolic  operators  routine  computes the  second derivative 
     curve of a  rational  curve  with  respect to a knot. The result is  
     obtained by using other symbolic operators to compute the following 
     expression:

         ( N )''   D*D*N''- 2*D*D'*N'+N*(2*D'*D'-D*D'')
         ( - )   = ------------------------------------
         ( D )                   D*D*D     

     where  N',  N'', D',  D'' are derivatives with respect to a knot. A 
     typical calling example is:

       NL_CURVE   cur, der;
       NL_INDEX   k;
       NL_STACKS  SG;
       ...
       (define cur);
       ...
       N_CrvInitArrays(&der);
       N_CrvEvalSecondDerivKnot(&cur,k,NL_LEFT,&der,&SG);

     Since  NLIB  does not  allow  knot multiplicities  greater than the 
     degree, the curve cannot be  differentiated  with respect to a knot
     with  multiplicity  equal  to  the  degree. The  error  NL_KML_ERR  is 
     returned if such a knot is found.


   ACCESS:
   
     cur  , input  ,  NURBS curve
     k    , input  ,  Index of  knot, i.e. the  derivative  with respect 
                      to u_k is computed
     flg  , input  ,  Flag:
                        NL_LEFT : left  derivative. NL_INDEX  k  MUST  SATISFY
                               u_(k) != u_(k-1)
                        NL_RIGHT: right  derivative. NL_INDEX k  MUST  SATISFY
                               u_(k) != u_(k+1) 
     der  , output ,  Derivative curve
     SG   , input  ,  der's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvEvalSecondDerivKnot( NL_CURVE *cur, NL_INDEX k, NL_FLAG flg, NL_CURVE *der, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX m;

    NL_REAL *U;

    NL_CURVE n, np, npp, ddnpp, ddpnp, ddppn, nsum, num;

    NL_CFUN d, dd, dp, dpp, ddp, dpdp, ddpp, dsum, den;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check if non-rational */

    if( NOT N_IsCrvRat( cur ) )
    {
        error = N_CrvEvalHighDerivsKnot( cur, k, flg, 2, der, SG );

        if( error EQ NL_YES )
            NL_OUT;

        NL_OUT;
    }

    /* Extract numerator and denominator */

    N_CrvInitArrays( &n );
    N_CFuncInitArrays( &d );
    error = N_CrvGetNumAndDenom( cur, &n, &d, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute denominator of output curve */

    N_CrvGetKnots( cur, &m, &U );
    N_CFuncInitArrays( &dd );
    N_CFuncInitArrays( &den );

    error = N_CrvFuncMultiplyCrvFunc( &d, &d, &dd, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvFuncMultiplyCrvFunc( &d, &dd, &den, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvFuncInsertKnot( &den, U[k], 2, &den, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute terms in the numerator */

    /*--------- DDN'' ------------*/

    N_CrvInitArrays( &npp );
    error = N_CrvEvalHighDerivsKnot( &n, k, flg, 2, &npp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ddnpp );
    error = N_CrvFuncMultiplyCrv( &dd, &npp, &ddnpp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*--------- 2DD'N' ------------*/

    N_CFuncInitArrays( &dp );
    error = N_CrvFuncDerivKnot( &d, k, flg, &dp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &ddp );
    error = N_CrvFuncMultiplyCrvFunc( &d, &dp, &ddp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncMultiplyConstant( 2.0, &ddp );

    N_CrvInitArrays( &np );
    error = N_CrvNonRatEvalDeriv( &n, k, flg, &np, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ddpnp );
    error = N_CrvFuncMultiplyCrv( &ddp, &np, &ddpnp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_CrvInsertKnot( &ddpnp, U[k], 1, &ddpnp, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*--------- N(2D'D'-DD'') ------------*/

    N_CFuncInitArrays( &dpdp );
    error = N_CrvFuncMultiplyCrvFunc( &dp, &dp, &dpdp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvFuncMultiplyConstant( 2.0, &dpdp );

    error = N_CrvFuncInsertKnot( &dpdp, U[k], 1, &dpdp, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &dpp );
    error = N_CrvFuncHigherDerivKnot( &d, k, flg, 2, &dpp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &ddpp );
    error = N_CrvFuncMultiplyCrvFunc( &d, &dpp, &ddpp, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CFuncInitArrays( &dsum );
    error = N_CrvFuncSumDiffCrvFunc( &dpdp, &ddpp, NL_MINUS, NL_YES, &dsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &ddppn );
    error = N_CrvFuncMultiplyCrv( &dsum, &n, &ddppn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add terms in the numerator */

    N_CrvInitArrays( &nsum );
    error = N_CrvSumDiffCrv( &ddnpp, &ddpnp, NL_MINUS, &nsum, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &num );
    error = N_CrvSumDiffCrv( &nsum, &ddppn, NL_PLUS, &num, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Create output curve */

    error = N_CreateCrvFromNumAndDenom( &num, &den, der, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvEvalSecondDerivKnot */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a constant 
     and a curve. A typical calling example is:

       NL_REAL   alpha; 
       NL_CURVE  cur;
       ...
       (define cur and alpha);
       ...
       N_ConstantMultiplyCrv(alpha,&cur);

     THE  PRODUCT  IS  COMPUTED  IN-PLACE,  I.E. THE  ORIGINAL NL_CURVE IS 
     DESTROYED.


   ACCESS:
   
     alpha , input  ,  Scalar
     cur   , in/out ,  Product of alpha and cur


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ConstantMultiplyCrv( NL_REAL alpha, NL_CURVE *cur )
{

    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get locals */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Compute product */

    for ( i = 0; i <= n; i++ )
        N_ScaleCPtXYZ( alpha, Pw[i], &Pw[i] );
} /* end N_ConstantMultiplyCrv */



/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a linear
     function and a curve.  The linear function is defined by the two
     input values, which are its values at the beginning and end of the
     curve, respectively.  This function is intended for curve that
     represent vector fields.  If used on a normal 3d curve, it will have
     the effect of moving each point on the curve towards or away from the
     origin.  A typical calling example is:

       CURVE    pCrv = ...;
       NL_REAL  dS0, dS1;
       ...
       (define pCrv and dS0, dS1);
       ...
       N_LinearMultiplyCrv( CURVE * pCurve, NL_REAL dS0, NL_REAL dS1 );

     THE PRODUCT IS COMPUTED IN PLACE, I.E. THE ORIGINAL NL_CURVE IS DESTROYED.

   ACCESS:

     dS0, dS1 , input  ,  Scale factors at curve start and end
     pCurve   , in/out ,  Multiplied by linear interpolation from dS0 to dS1


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_LinearMultiplyCrv( NL_REAL dScale0, NL_REAL dScale1, NL_CURVE *pCurve )
{
  /*
   * Method: Scale each control point by the linear interpolation between dS0 and dS1.
   * Interpolate using Greville abscissae of each control point.
   * Although if dS0 and dS1 are the same, we don't have to bother with that.
   */

  /* Get locals */
  NL_INDEX  ii, lNPts, lNKts;
  NL_CPOINT *pCtrlPts;
  NL_REAL dScale, dFrac, dT0, dT1, dT;
  NL_DEGREE lDeg;
  NL_REAL   *pKnots;

  if ( fabs( dScale0 - dScale1 ) < NL_PTOL )
  {
      dScale = ( dScale0 + dScale1 ) / 2.0;
      N_ConstantMultiplyCrv( dScale, pCurve );
      return;
  }

  /* Different scale factors, have to interpolate. */

  N_CrvGetCPtsDegreeAndKnots( pCurve, &lNPts, &pCtrlPts, &lDeg, &lNKts, &pKnots );

  dT0 = pKnots[0];
  dT1 = pKnots[lNKts];
  for ( ii = 0; ii <= lNPts; ii++ )
  {
      N_CrvGetGrevilleAbscissa( pKnots, lNKts, lDeg, ii, &dT );
      dFrac = ( dT - dT0 ) / ( dT1 - dT0 );
      dScale = dScale0 + dFrac * ( dScale1 - dScale0 );
      N_ScaleCPtXYZ( dScale, pCtrlPts[ii], &pCtrlPts[ii] );
  }

} /* end N_ConstantMultiplyCrv */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic  operators  routine computes the  combination of two
     curves, i.e. it computes alpha*curP +- beta*curQ = curR. A typical 
     calling example is:

       NL_REAL    alpha, beta; 
       NL_CURVE   curP, curQ, curR;
       NL_STACKS  SG;
       ...
       (define curP and curQ; get alpha and beta);
       ...
       N_CrvCombine(alpha,&curP,beta,&curQ,NL_PLUS,&curR,&SG);


   ACCESS:
   
     alpha , input  ,  Scalar
     curP  , input  ,  NURBS curve
     beta  , input  ,  Scalar
     curQ  , input  ,  NURBS curve
     opr   , input  ,  Flag:
                         NL_PLUS : sum
                         NL_MINUS: difference
     curR  , output ,  Combination alpha*curP +- beta*curQ
     SG    , input  ,  curR's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvCombine( NL_REAL alpha, NL_CURVE *curP, NL_REAL beta, NL_CURVE *curQ, NL_FLAG opr, NL_CURVE *curR, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_CURVE curPW, curQW;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Scale curves */

    N_CrvInitArrays( &curPW );
    error = N_CrvCopy( curP, &curPW, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvInitArrays( &curQW );
    error = N_CrvCopy( curQ, &curQW, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_ConstantMultiplyCrv( alpha, &curPW );
    N_ConstantMultiplyCrv( beta, &curQW );

    /* Compute sum/difference */

    error = N_CrvSumDiffCrv( &curPW, &curQW, opr, curR, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvCombine */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This symbolic operators routine computes the product of a constant 
     and a curve in 4-D. A typical calling example is:

       NL_REAL   alpha; 
       NL_CURVE  cur;
       ...
       (define cur and alpha);
       ...
       N_ConstantMultiplyCrv4d(alpha,&cur);

     THE  PRODUCT  IS  COMPUTED  IN-PLACE,  I.E. THE  ORIGINAL NL_CURVE IS 
     DESTROYED.


   ACCESS:
   
     alpha , input  ,  Scalar
     cur   , in/out ,  Product of alpha and cur


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ConstantMultiplyCrv4d( NL_REAL alpha, NL_CURVE *cur )
{

    NL_INDEX i, n;

    NL_CPOINT *Pw;

    /* Get locals */

    N_CrvGetCPts( cur, &n, &Pw );

    /* Compute product */

    for ( i = 0; i <= n; i++ )
        N_ScaleCPt( alpha, Pw[i], &Pw[i] );
} /* end N_ConstantMultiplyCrv4d */



/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine reparametrizes a NURBS curve using
     a rational linear function of the form:

                            alf*u+bet
                 s = g(u) = ---------
                            gam*u+del
 
     It is assumed that the following conditions hold:

           (1) alf*del-bet*gam >  0
           (2) gam*u+del       != 0
           (3) gam*s-alf       != 0

     where u and s are the old and new parameters, respectively. If  the 
     output curve is  initialized to NULL,  memory to store new  control 
     points and knots is  allocated. If the output curve is  the same as 
     the  input curve,  reparametrization is  in place  and the original 
     curve is destroyed. A typical calling example is: 

       NL_CURVE   curP, curQ;
       NL_REAL    alf, bet, gam, del;
       NL_STACKS  SG;
       ...
       (define curP, get alf, bet, gam & del);
       ...
       N_CrvInitArrays(&curQ);
       N_CrvReparamRat(&curP,alf,bet,gam,del,&curQ,&SG);
       N_CrvReparamRat(&curP,alf,bet,gam,del,&curP,&SG);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot  vector  and  polygon  objects. IF  THE  INPUT  NL_CURVE  IS NON 
     RATIONAL, AFTER REPARAMETRIZATION IT BECOMES A RATIONAL NL_CURVE.


   ACCESS:
   
     curP    , input  ,  NURBS curve
     alf,bet , input  ,  Numerator's coefficients
     gam,del , input  ,  Denominator's coefficients
     curQ    , output ,  Curve after reparametrization
     SG      , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReparamRat( NL_CURVE *curP, NL_REAL alf, NL_REAL bet, NL_REAL gam, NL_REAL del, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvReparamRat");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, *w, num, den, fact;

    NL_CPOINT *Pw, *Qw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check coefficients */

    if( (alf * del - bet * gam)LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    /* Get local notation */

    if( NOT N_IsCrvRat( curP ) )
        N_CrvNonRatToRat( curP );

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Get new knots */

    for ( i = 0; i <= m; i++ )
    {
        num = alf * UP[i] + bet;
        den = gam * UP[i] + del;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_INP_ERR );

        UQ[i] = num / den;
    }

    /* Get new weighted control points */

    if( curP NEQ curQ )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( Pw[i], &Qw[i] );
        }
    }

    w = N_AllocReal1dArray( n, &SL );

    if( w EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        N_CPtGetW( Qw[i], &w[i] );
        fact = 1.0 / w[i];
        N_ScaleCPt( fact, Qw[i], &Qw[i] );
    }

    for ( i = 0; i <= n; i++ )
    {
        fact = 1.0;

        for ( j = 1; j <= p; j++ )
            fact = fact * (gam * UQ[i + j] - alf);

        if( fact LT 0.0 )
            fact = -fact;

        w[i] = w[i] * fact;
        N_ScaleCPt( w[i], Qw[i], &Qw[i] );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvReparamRat */

/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine reparametrizes a NURBS curve such
     that the end weights become equal. A typical calling example is: 

       NL_CURVE  curP;
       NL_REAL   w, c, d;
       ...
       (define curP, get w, c & d);
       ...
       N_CrvReparamWeights(&curP,w,c,d);

     The  reparametrization is  performed  in-place, i.e.  the original 
     curve is destroyed.


   ACCESS:
   
     curP , in/out ,  NURBS curve
     w    , input  ,  New end weights
     c,d  , input  ,  The reparametrized curved is defined over [c,d]


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvReparamWeights( NL_CURVE *curP, NL_REAL w, NL_REAL c, NL_REAL d )
{

    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    NL_DEGREE p;

    NL_REAL *UP, alf, bet, gam, del, exp, w0, wn, f;

    NL_CPOINT *Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check rationality */

    if( NOT N_IsCrvRat( curP ) )
        NL_OUT;

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* Reparametrize curve */

    N_CPtGetW( Pw[0], &w0 );
    N_CPtGetW( Pw[n], &wn );

    exp = 1.0 / p;
    w0 = pow( w0, exp );
    wn = pow( wn, exp );

    alf = wn * d - w0 * c;
    bet = w0 * c * UP[m] - wn * d * UP[0];
    gam = wn - w0;
    del = w0 * UP[m] - wn * UP[0];

    error = N_CrvReparamRat( curP, alf, bet, gam, del, curP, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Re-weight control points */

    N_CPtGetW( Pw[0], &w0 );
    N_CPtGetW( Pw[n], &wn );

    f = (2.0 *w) / (w0 + wn);
    N_CrvScaleCPts( curP, f );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CrvReparamWeights */


/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine reparametrizes a NURBS curve C(u)
     with a  B-spline  function  f(s). The  function  must  satisfy the 
     following conditions:

       (1) f'(s) > 0 for all s in [c,d] (must be monotonic)
       (2) a = f(c) and b = f(d)

     where [a,b] and [c,d] are the domains over which C(u) and f(s) are
     defined, respectively. If the output curve is initialized to NULL, 
     memory to store new control points and  knots is allocated. If the  
     output curve is the same as the  input curve, reparametrization is
     done  in  place and the  original  curve is  destroyed. A  typical 
     calling example is:

       NL_CURVE   curP, curQ;
       NL_CFUN    cfn;
       NL_STACKS  SP, SQ;
       ...
       (define curP, get cfn);
       ...
       N_CrvInitArrays(&curQ);
       N_SrfReparamFunc(&curP,&cfn,&curQ,&SP,&SQ);
       N_SrfReparamFunc(&curP,&cfn,&curP,&SP,&SP);

     If memory is  available, curQ  is not initialized and  the routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector and polygon objects.


   ACCESS:
   
     curP , input  ,  NURBS curve
     cfn  , input  ,  Reparametrization function
     curQ , output ,  Curve after reparametrization
     SP   , input  ,  curP's stack
     SQ   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReparamFunc( NL_CURVE *curP, NL_CFUN *cfn, NL_CURVE *curQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReparamFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, np, mp, ns, ms, nq, mq, mlu, mls, miu, mis, mi, ru, rs, s, au, as, bu, bs, mxu, mxs, ixu, ixs, t, kind, cind, first, last, rem, save, lbz, rbz;

    /*        NL_INTEGER     **bin;      */

    NL_DEGREE p, q, pq;

    NL_REAL *UP, *UQ, *S, *XU, *XS, *fu, *fb, *fn, *alfs, *omas, *alfu, *omau, alf, oma, bet, omb, num, den, f0, fac;

    NL_KNOTVECTOR *knu, *kns, *kxu, *kxs;

    NL_CPOINT *Pw, *Qw, *Bw, *Nw, *Rw;

    NL_CURVE curA, curR;

    NL_CFUN cfnR;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check degree */

    N_CrvGetDegree( curP, &p );
    N_CFuncGetDegree( cfn, &q );

    pq = p * q;

    if( pq GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Map knot vectors */

    N_CrvGetKnotVector( curP, &knu );
    N_CFuncGetKnotVector( cfn, &kns );

    N_BasisGetSpanCount( knu, p, &i );
    N_BasisGetSpanCount( kns, q, &j );

    kxu = N_AllocKnotVectorAndArray( j, &SL );

    if( kxu EQ NULL )
        NL_QUIT;

    kxs = N_AllocKnotVectorAndArray( i, &SL );

    if( kxs EQ NULL )
        NL_QUIT;

    error = N_MapKnotsBetweenCrvFuncAndKnotVector( cfn, knu, p, kxu, kxs );

    if( error EQ NL_YES )
        NL_OUT;

    /* Refine curve and reparametrization function */

    N_KnotVectorGetKnots( kxu, &mxu, &XU );
    N_KnotVectorGetKnots( kxs, &mxs, &XS );

    if( mxu GE 0 )
    {
        N_CrvInitArrays( &curR );
        error = N_CrvRefine( curP, kxu, &curR, SP, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( mxs GE 0 )
    {
        N_CFuncInitArrays( &cfnR );
        error = N_CrvFuncRefine( cfn, kxs, &cfnR, &SL, &SL );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Get local notation and check error */

    if( mxu GE 0 )
    {
        N_CrvGetCPtsDegreeAndKnots( &curR, &np, &Pw, &p, &mp, &UP );
        N_CrvGetKnotVector( &curR, &knu );
    }
    else
    {
        N_CrvGetCPtsDegreeAndKnots( curP, &np, &Pw, &p, &mp, &UP );
    }

    if( mxs GE 0 )
    {
        N_CFuncGetData( &cfnR, &ns, &fu, &q, &ms, &S );
        N_CFuncGetKnotVector( &cfnR, &kns );
    }
    else
    {
        N_CFuncGetData( cfn, &ns, &fu, &q, &ms, &S );
    }

    N_BasisGetSpanCount( knu, p, &i );
    N_BasisGetSpanCount( kns, q, &j );

    if( i NEQ j )
        NL_ERROR( NL_NUM_ERR );

    /* See if memory is needed */

    nq = i * pq;
    mq = nq + pq + 1;

    if( curP EQ curQ )
    {
        curA = *curP;

        error = N_AllocCrvArrays( curP, nq, pq, mq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, nq, pq, mq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Get local memory */

    Bw = N_AllocCPt1dArray( p, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt1dArray( p, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( pq, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    fb = N_AllocReal1dArray( q, &SL );

    if( fb EQ NULL )
        NL_QUIT;

    fn = N_AllocReal1dArray( q, &SL );

    if( fn EQ NULL )
        NL_QUIT;

    alfu = N_AllocReal1dArray( p, &SL );

    if( alfu EQ NULL )
        NL_QUIT;

    omau = N_AllocReal1dArray( p, &SL );

    if( omau EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( q, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( q, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /*        bin = N_AllocInt2dArray(pq,pq,&SL);    */
    /*        if( bin EQ NULL )  NL_QUIT;      */

    /* Initialize */

    au = p;
    bu = p + 1;
    as = q;
    bs = q + 1;
    ru = -1;
    rs = -1;
    ixu = 0;
    ixs = 0;

    cind = 1;
    kind = pq + 1;
    rem = -1;

    /*        N_PascalTriRow(bin,pq);        */

    for ( i = 0; i <= pq; i++ )
        UQ[i] = S[as];

    N_CopyCPt( Pw[0], &Qw[0] );

    for ( i = 0; i <= p; i++ )
        N_CopyCPt( Pw[i], &Bw[i] );

    for ( j = 0; j <= q; j++ )
        fb[j] = fu[j];

    /*************************************************************/
    /* Loop through the knot vectors and do the following:       */
    /*   (1) Extract the i-th Bezier segment.                    */
    /*   (2) Reparametrize the segment.                          */
    /*   (3) Remove the knot between the i-th and the (i-1)-th   */
    /*       segment.                                            */
    /*************************************************************/

    while( bu LT mp AND bs LT ms )
    {
        /* Get multiplicity of the knots */

        i = bu;

        while( bu LT mp AND UP[bu]EQ UP[bu + 1] )
            bu++;
        mlu = bu - i + 1;
        miu = mlu;

        j = bs;

        while( bs LT ms AND S[bs]EQ S[bs + 1] )
            bs++;
        mls = bs - j + 1;
        mis = mls;

        /* Check multiplicities of internal knots */

        if( bu LT mp AND bs LT ms )
        {
            if( mlu GT p OR mls GT q )
                NL_ERROR( NL_KNT_ERR );
        }

        /* Adjust multiplicities and compute multiplicty of output knot */

        if( miu EQ 1 AND ixu LE mxu )
        {
            if( fabs( UP[bu] - XU[ixu] )LT NL_PTOL )
            {
                miu = 0;
                ixu++;
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
            mi = pq - p + miu;

        else if( miu EQ 0 )
            mi = pq - q + mis;

        else
            mi = NL_MAX( pq - p + miu, pq - q + mis );

        /* Insert knot to get Bezier segments */

        ru = p - mlu;
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

        if( ru GT 0 )
        {
            num = UP[bu] - UP[au];

            for ( k = p; k > mlu; k-- )
            {
                alfu[k - mlu - 1] = num / (UP[au + k] - UP[au]);
                omau[k - mlu - 1] = 1.0 - alfu[k - mlu - 1];
            }

            for ( j = 1; j <= ru; j++ )
            {
                save = ru - j;
                s = mlu + j;

                for ( k = p; k >= s; k-- )
                {
                    N_Combine2CPts( alfu[k - s], Bw[k], omau[k - s], Bw[k - 1], &Bw[k] );
                }
                N_CopyCPt( Bw[p], &Nw[save] );
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
                    fb[k] = alfs[k - s] * fb[k] + omas[k - s] * fb[k - 1];
                }
                fn[save] = fb[q];
            }
        }

        /* Now reparametrize Bezier segment */

        f0 = fb[0];
        fac = 1.0 / (fb[q] - fb[0]);
        fb[0] = 0.0;
        fb[q] = 1.0;

        for ( i = 1; i < q; i++ )
            fb[i] = fac * (fb[i] - f0);

        error = N_BezReparam( Bw, p, fb, q, NULL, Rw );

        if( error EQ NL_YES )
            NL_OUT;

        /* Remove the knot S[as] */

        if( rem GT 1 )
        {
            first = kind - 2;
            last = kind;
            den = S[bs] - S[as];

            for ( k = 1; k < rem; k++ )
            {
                i = first;
                j = last;
                l = j - kind + 1;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        alf = (S[bs] - UQ[i]) / (S[as] - UQ[i]);
                        oma = 1.0 - alf;
                        N_Combine2CPts( alf, Qw[i], oma, Qw[i - 1], &Qw[i] );
                    }

                    if( j GE lbz )
                    {
                        bet = (S[bs] - UQ[j - k]) / den;
                        omb = 1.0 - bet;
                        N_Combine2CPts( bet, Rw[l], omb, Rw[l + 1], &Rw[l] );
                    }
                    i++;
                    j--;
                    l--;
                }
                first--;
                last++;
            }
        }

        /* Load knot vector and control points */

        if( au NEQ p )
        {
            for ( i = 0; i < pq - rem; i++ )
            {
                UQ[kind] = S[as];
                kind++;
            }
        }

        for ( i = lbz; i <= rbz; i++ )
        {
            N_CopyCPt( Rw[i], &Qw[cind] );
            cind++;
        }

        /* Initialize for next pass through */

        if( bu LT mp AND bs LT ms )
        {
            for ( i = 0; i < ru; i++ )
                N_CopyCPt( Nw[i], &Bw[i] );

            for ( i = ru; i <= p; i++ )
                N_CopyCPt( Pw[bu - p + i], &Bw[i] );

            for ( j = 0; j < rs; j++ )
                fb[j] = fn[j];

            for ( j = rs; j <= q; j++ )
                fb[j] = fu[bs - q + j];

            au = bu;
            bu++;
            rem = pq - mi;
            as = bs;
            bs++;
        }
        else
        {
            /* Get the end knot */

            for ( i = 0; i <= pq; i++ )
            {
                UQ[kind] = S[bs];
                kind++;
            }
        }
    }

    /* If reparametrization is in place, kill old curve */

    if( curP EQ curQ )
        N_FreeCrv( &curA, SP );

    /* Compact output curve */

    if( cind LT nq + 1 )
    {
        if( curP EQ curQ )
        {
            N_CrvSetSizeIndices( curP, cind - 1, pq, kind - 1 );

            error = N_CrvCompress( curP, SP );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            N_CrvSetSizeIndices( curQ, cind - 1, pq, kind - 1 );

            error = N_CrvCompress( curQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReparamFunc */



/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine reparametrizes NURBS curves to be
     generally with respect to arc length.  Note that it does not result
     in a curve with even approxiamte constant-speed parameterization.
     The curve should have some multiple interior knots with multiplici-
     ties >= the degree.  These multiple knots are placed using arc
     length parametrization, and all knots lying between these multiple
     knots are rescaled.  The reparametrization does not change the curve
     geometrically, however, it changes the magnitudes of the derivatives.
     If the output curve is initialized to NULL, memory to store new
     control points and knots is allocated. If the output curve is the
     same as the input curve, reparametrization is in place and the
     original knots are destroyed.  A typical calling example is:

       NL_CURVE   curP, curQ;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (define curP and get tol);
       ...
       N_CrvInitArrays(&curQ);
       N_SrfReparamMultKnots(&curP,tol,&curQ,&SG);
       N_SrfReparamMultKnots(&curP,tol,&curP,&SG);

     If memory is  available, curQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in curQ's  
     knot vector  and  polygon  objects. This  routine is  particularly 
     suitable to  reparametrize piecewise  Bezier curves given in NURBS
     form with degree-fold  multiple knots.


   ACCESS:
   
     curP , input  ,  NURBS curve
     tol  , input  ,  Relative tolerance for arc length computation   
     curQ , output ,  Curve after reparametrization
     SG   , input  ,  curQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReparamMultKnots( NL_CURVE *curP, NL_REAL tol, NL_CURVE *curQ, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReparamMultKnots");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, n, m, mlt, oldmlt;

    NL_DEGREE p;

    NL_REAL *UP, *UQ, fact, len, d, ua, ub, ul, ur;

    NL_CPOINT *Pw, *Qw;

    NL_CURVE curA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_CrvGetCPtsDegreeAndKnots( curP, &n, &Pw, &p, &m, &UP );

    /* See if memory is needed */

    if( curP EQ curQ )
    {
        N_CrvGetCPtsAndKnots( curP, &Qw, &UQ );
    }
    else
    {
        error = N_CrvSizeArrays( curQ, n, p, m, rname, SG );

        if( error EQ NL_YES )
            NL_OUT;

        N_CrvGetCPtsAndKnots( curQ, &Qw, &UQ );
    }

    /* Copy CurP into CurA and get its total arc length. */

    N_CrvInitArrays( &curA );
    error = N_CrvCopy( curP, &curA, &SL );

    if( error EQ NL_YES )
      { NL_OUT; }

    error = N_CrvArcLength( &curA, UP[0], UP[m], tol, NL_RELATIVE, &len );

    if( error EQ NL_YES )
      { NL_OUT; }


    /* Zip through the knot vector and reparametrize.
     * Whenever we find a knot with full multiplicity ( == degree ),
     * set the parameter there to the arc length there, and scale all
     * interior knots between the previous multiple knot and this one.
     * Note, start and end of knto vector have multiplicity == deg+1,
     * and so are included in this algorithm.
     */

    ua = UP[0];
    ul = 0.0;
    ur = 0.0;
    jj = p;
    kk = p + 1;
    oldmlt = -1;

    for ( ii = 0; ii <= p; ii++ )
      { UQ[ii] = 0.0; }

    while( kk LT m )
    {
        ii = kk;

        /* Check multiplicity of this knot. */
        while ( kk LT m AND UP[kk] EQ UP[kk+1] )
          { kk++; }

        mlt = kk - ii + 1;

        if( mlt GE p )
        {
            ub = UP[kk];

            error = N_CrvArcLength( &curA, ua, ub, tol, NL_RELATIVE, &d );

            if( error EQ NL_YES )
              { NL_OUT; }

            ur += d;

            if( kk EQ m )
              { ur = len; }

            fact = (ur - ul) / (ub - ua);

            for ( ii = jj + 1; ii <= kk - mlt; ii++ )
              { UQ[ii] = ul + fact * (UP[ii] - ua); }

            if( oldmlt GT 0 )
            {
                for ( ii = jj - oldmlt + 1; ii <= jj; ii++ )
                  { UQ[ii] = ul; }
            }
            oldmlt = mlt;

            ua = ub;
            ul = ur;
            jj = kk;
        }

        kk++;
    }

    for ( ii = 0; ii <= p; ii++ )
      { UQ[n + ii + 1] = len; }

    /* Output control points if necessary */

    if( curP NEQ curQ )
    {
        for ( ii = 0; ii <= n; ii++ )
          { N_CopyCPt( Pw[ii], &Qw[ii] ); }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReparamMultKnots */


/*******************************************************************//**


   DESCRIPTION:

     This geometry processing routine reparametrizes a NURBS curve such
     that the end weights become specific  values. All the weights will
     change along  with the  position of  the knots. A  typical calling 
     example is: 

       NL_CURVE  cur;
       NL_REAL   wt0, wtn;
       ...
       (define cur, get wt0 & wtn);
       ...
       N_SrfReparamWeights(&cur,wt0,wtn);

     THE  REPARAMETRIZATION IS  PERFORMED  IN-PLACE, I.E.  THE ORIGINAL 
     NL_CURVE IS DESTROYED. IF THE NL_CURVE IS NON-RATIONAL, AFTER REPARAMET-
     RIZATION IT BECOMES RATIONAL.


   ACCESS:
   
     cur  , in/out ,  NURBS curve
     wt0  , input  ,  New weight at the beginning
     wtn  , input  ,  New weight at the end


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReparamWeights( NL_CURVE *cur, NL_REAL wt0, NL_REAL wtn )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReparamWeights");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_DEGREE p;

    NL_REAL *U, *w, alf, bet, gam, del, exp, w0, wn, num, den, fact;

    NL_CPOINT *Pw;

    NL_INTERVAL OLDSPAN;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check rationality */

    if( NOT N_IsCrvRat( cur ) )
        N_CrvNonRatToRat( cur );

    /* Get local notation and rescale span */

    N_CrvGetCPtsDegreeAndKnots( cur, &n, &Pw, &p, &m, &U );
    N_CreateInterval( &OLDSPAN, U[0], U[m] );
    N_CrvReparamToInterval( cur, NL_UNITSPAN );

    /* Reparametrize curve */

    N_CPtGetW( Pw[0], &w0 );
    N_CPtGetW( Pw[n], &wn );

    exp = 1.0 / p;
    w0 = pow( w0 / wt0, exp );
    wn = pow( wn / wtn, exp );

    alf = wn;
    bet = 0.0;
    gam = wn - w0;
    del = w0;

    if( (alf * del - bet * gam)LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    w = N_AllocReal1dArray( n, &SL );

    if( w EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        N_CPtGetW( Pw[i], &w[i] );

        if( N_FloatOpIsBad( 1.0, w[i], NL_DIVISION ) )
            NL_ERROR( NL_WEI_ERR );

        fact = 1.0 / w[i];
        N_ScaleCPt( fact, Pw[i], &Pw[i] );
    }

    N_ScaleCPt( wt0, Pw[0], &Pw[0] );

    for ( i = 1; i < n; i++ )
    {
        fact = 1.0;

        for ( j = 1; j <= p; j++ )
            fact *= (gam * U[i + j] + del);

        if( fact LT 0.0 )
            fact = -fact;

        if( N_FloatOpIsBad( w[i], fact, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        w[i] = w[i] / fact;
        N_ScaleCPt( w[i], Pw[i], &Pw[i] );
    }
    N_ScaleCPt( wtn, Pw[n], &Pw[n] );

    N_CPtSetW( wt0, &Pw[0] );
    N_CPtSetW( wtn, &Pw[n] );

    for ( i = 0; i <= m; i++ )
    {
        num = alf * U[i] + bet;
        den = gam * U[i] + del;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_INP_ERR );

        U[i] = num / den;
    }

    N_CrvReparamToInterval( cur, OLDSPAN );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReparamWeights */

